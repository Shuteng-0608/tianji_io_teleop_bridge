"""Local FK and explicitly opened telemetry / right-arm Position connections.

Importing this module does not load a shared library or connect to a robot.
The only setting sent by Feedback is the right-arm telemetry selector (216).
PositionFeedback is separate and used only by the explicit auto command.
Neither client has sensor-zeroing or tool-upload methods.
"""
import ctypes as C
import importlib.util
import ipaddress
from pathlib import Path
import time

import numpy as np

from ft_calibration_core import finite_array, rotation, sha256


CONTROL_HASH = 'dd6b7afdbda8de3bafb6797c6f94547302ae60bf285ee8daa9762de45be636d6'
KINEMATICS_HASH = '0f592a4c3add356d3f48e3ccb97174750c591f2a636db686d4499812f51caefa'
WRAPPER_HASH = 'fae947726ca67f9b5ea644710e12154c508015bc4534db9803048d323fc11a3d'


def checked_file(root, relative, digest):
    path = Path(root).expanduser() / relative
    if sha256(path) != digest:
        raise ValueError(f'Unverified SDK file: {path}')
    return path


class Kinematics:
    """Local mathematical library only; no controller connection."""
    def __init__(self, root, model, *, library_path=None):
        # The HUD uses the installed mathematical library, independently of the SDK checkout.
        path = (checked_file(root, 'DEMO_C++/libKine.so', KINEMATICS_HASH)
                if library_path is None else Path(library_path))
        if sha256(path) != KINEMATICS_HASH:
            raise ValueError(f'Unverified kinematics library: {path}')
        self.lib = C.CDLL(str(path))
        self.lib.FX_LOG_SWITCH.argtypes = [C.c_int32]
        self.lib.FX_LOG_SWITCH.restype = None
        self.lib.FX_LOG_SWITCH(0)
        self.lib.FX_Robot_Init_Type.argtypes = [C.c_int32, C.c_int32]
        self.lib.FX_Robot_Init_Type.restype = C.c_bool
        self.lib.FX_Robot_Init_Kine.argtypes = [C.c_int32, (C.c_double * 4) * 8]
        self.lib.FX_Robot_Init_Kine.restype = C.c_bool
        for name in ['FX_Robot_Tool_Rmv', 'FX_Robot_UserFrame_Rmv']:
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = [C.c_int32], C.c_bool
        self.lib.FX_Robot_Kine_FK.argtypes = [C.c_int32, C.c_double * 7,
                                             (C.c_double * 4) * 4]
        self.lib.FX_Robot_Kine_FK.restype = C.c_bool
        dh = ((C.c_double * 4) * 8)(*[ (C.c_double * 4)(*row) for row in model.dh])
        if not (self.lib.FX_Robot_Init_Type(1, model.robot_type) and
                self.lib.FX_Robot_Init_Kine(1, dh) and
                self.lib.FX_Robot_Tool_Rmv(1) and self.lib.FX_Robot_UserFrame_Rmv(1)):
            raise RuntimeError('Local right-arm FK initialization failed')

    def rotation(self, joints):
        joints = finite_array(joints, (7,), 'FK joint degrees')
        result = ((C.c_double * 4) * 4)()
        if not self.lib.FX_Robot_Kine_FK(1, (C.c_double * 7)(*joints), result):
            raise RuntimeError('Local FK failed')
        return rotation(np.array(result)[:3, :3], 'SDK flange rotation')


def feedback_row(dcss, wall_ns, mono_s):
    """Decode one coherent DCSS snapshot; raw SDK channel order is preserved."""
    fb = dcss.m_Out[1]
    if dcss.m_State[1].m_ERRCode != 0:
        raise RuntimeError(f'Right arm reports error {dcss.m_State[1].m_ERRCode}')
    q = finite_array(list(fb.m_FB_Joint_Pos), (7,), 'joint degrees')
    v = finite_array(list(fb.m_FB_Joint_Vel), (7,), 'joint velocity')
    raw = finite_array(list(fb.m_EST_Joint_Firc_Dot)[:6], (6,), 'raw six-axis data')
    channel = float(fb.m_EST_Joint_Firc[0])
    if not np.isfinite(channel):
        raise ValueError('Invalid telemetry channel')
    return {'host_time_ns': wall_ns, 'monotonic_s': mono_s,
            'frame': int(fb.m_OutFrameSerial), 'channel': channel,
            'mode': int(dcss.m_State[1].m_CurState),
            'low_speed': (fb.m_LowSpdFlag[0] if isinstance(fb.m_LowSpdFlag, bytes)
                          else int(fb.m_LowSpdFlag)),
            'velocity_ratio': int(dcss.m_In[1].m_Joint_Vel_Ratio),
            'acceleration_ratio': int(dcss.m_In[1].m_Joint_Acc_Ratio),
            'command_deg': list(dcss.m_In[1].m_Joint_CMD_Pos),
            'joint_deg': q.tolist(), 'velocity_deg_s': v.tolist(),
            'wrench_raw': raw.tolist(), 'wrench_si': (raw * .0001).tolist()}


class Feedback:
    def __init__(self, root, ip, *, connect=False):
        if connect is not True:
            raise ValueError('Live telemetry requires explicit --connect')
        self.ip = tuple(int(x) for x in str(ipaddress.IPv4Address(ip)).split('.'))
        self.root = root
        self.lib = None
        self.connected = False

    def __enter__(self):
        # Load DCSS declarations without constructing the SDK's high-level robot class.
        wrapper = checked_file(self.root, 'SDK_PYTHON/fx_robot.py', WRAPPER_HASH)
        control = checked_file(self.root, 'SDK_PYTHON/libMarvinSDK.so', CONTROL_HASH)
        spec = importlib.util.spec_from_file_location('_ft_sdk_structures', wrapper)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.buf = module.DCSS()
        self.lib = C.CDLL(str(control))
        signatures = {
            'CheckSDKTypeCompat': ([C.POINTER(C.c_int)], C.c_int),
            'OnGetSDKVersion': ([], C.c_long),
            'OnLinkTo': ([C.c_ubyte] * 4, C.c_bool),
            'OnRelease': ([], C.c_bool),
            'OnGetBuf': ([C.POINTER(module.DCSS)], C.c_bool),
            'OnSetUserSpcfData_B': ([C.c_long], C.c_bool),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, result
        order = C.c_int()
        if self.lib.CheckSDKTypeCompat(C.byref(order)) < 0:
            raise RuntimeError('SDK ABI check failed')
        if self.lib.OnGetSDKVersion() != 100343014:
            raise RuntimeError('Unexpected control SDK version')
        try:
            # OnRelease is also called after a failed/partial connection attempt.
            self.connected = True
            if not self.lib.OnLinkTo(*self.ip):
                raise RuntimeError('Connection failed; release other SDK clients (UDP 4730)')
            if not self.lib.OnSetUserSpcfData_B(216):
                raise RuntimeError('Right-arm telemetry selection failed')
            previous, changes = None, 0
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                row = self.poll()
                if row is not None:
                    if previous is not None and row['frame'] != previous:
                        changes += 1
                    previous = row['frame']
                    if changes >= 2 and row['channel'] == 216:
                        self.initial = row
                        return self
                time.sleep(.005)
            raise RuntimeError('No fresh right-arm feedback with channel 216')
        except BaseException:
            self.close()
            raise

    def poll(self):
        if self.lib.OnGetBuf(C.byref(self.buf)):
            return feedback_row(self.buf, time.time_ns(), time.monotonic())
        return None

    def close(self):
        if self.connected:
            self.connected = False
            if not self.lib.OnRelease():
                print('Warning: SDK release returned false; check connection before reusing it')

    def __exit__(self, *_):
        self.close()


class PositionFeedback(Feedback):
    """Right-arm-only motion facade, at the operator's fixed 10% / 10%."""
    def __enter__(self):
        super().__enter__()
        try:
            signatures = {
                'OnClearSet': ([], C.c_bool),
                'OnSetJointLmt_B': ([C.c_int, C.c_int], C.c_bool),
                'OnSetJointCmdPos_B': ([C.c_double * 7], C.c_bool),
                'OnSetTargetState_B': ([C.c_int], C.c_bool),
                'OnSetSendWaitResponse': ([C.c_long], C.c_long),
                'OnSetIntPara': ([C.c_char_p, C.c_long], C.c_long),
            }
            for name, (args, result) in signatures.items():
                fn = getattr(self.lib, name)
                fn.argtypes, fn.restype = args, result
            return self
        except BaseException:
            self.close()
            raise

    def position(self, joints, *, enter=False):
        q = finite_array(joints, (7,), 'right Position target')
        # One packet: set conservative limits and latch a known target before
        # enabling Position. No high-level helper that could restore 100% limits.
        if not self.lib.OnClearSet():
            raise RuntimeError('Cannot start right-arm command packet')
        if not (self.lib.OnSetJointLmt_B(10, 10) and
                self.lib.OnSetJointCmdPos_B((C.c_double * 7)(*q))):
            raise RuntimeError('Cannot stage right-arm Position target / limits')
        if enter and not self.lib.OnSetTargetState_B(1):
            raise RuntimeError('Cannot stage right-arm Position mode')
        if self.lib.OnSetSendWaitResponse(100) < 0:
            raise RuntimeError('Right-arm Position packet failed')
        # A nonnegative SDK result alone is not proof: the runner also checks
        # actual mode, both ratios and the echoed target before accepting it.

    def stop(self):
        # OnSetStopRunning("B") returns early when LowSpdFlag is set, even if
        # a new goal is pending. Send its documented RSTA1 request directly.
        # SDK reads byte 29, so a complete zero-filled 30-byte buffer is needed.
        name = C.create_string_buffer(b'RSTA1', 30)
        if self.lib.OnSetIntPara(name, 0) != 0:
            raise RuntimeError('Right-arm stop request was not acknowledged')
