"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class force_torque_t(object):
    __slots__ = ["l_foot_force_z", "l_foot_torque_x", "l_foot_torque_y", "r_foot_force_z", "r_foot_torque_x", "r_foot_torque_y", "l_hand_force", "l_hand_torque", "r_hand_force", "r_hand_torque"]

    def __init__(self):
        self.l_foot_force_z = 0.0
        self.l_foot_torque_x = 0.0
        self.l_foot_torque_y = 0.0
        self.r_foot_force_z = 0.0
        self.r_foot_torque_x = 0.0
        self.r_foot_torque_y = 0.0
        self.l_hand_force = [ 0.0 for dim0 in range(3) ]
        self.l_hand_torque = [ 0.0 for dim0 in range(3) ]
        self.r_hand_force = [ 0.0 for dim0 in range(3) ]
        self.r_hand_torque = [ 0.0 for dim0 in range(3) ]

    def encode(self):
        buf = BytesIO()
        buf.write(force_torque_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ffffff", self.l_foot_force_z, self.l_foot_torque_x, self.l_foot_torque_y, self.r_foot_force_z, self.r_foot_torque_x, self.r_foot_torque_y))
        buf.write(struct.pack('>3f', *self.l_hand_force[:3]))
        buf.write(struct.pack('>3f', *self.l_hand_torque[:3]))
        buf.write(struct.pack('>3f', *self.r_hand_force[:3]))
        buf.write(struct.pack('>3f', *self.r_hand_torque[:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != force_torque_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return force_torque_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = force_torque_t()
        self.l_foot_force_z, self.l_foot_torque_x, self.l_foot_torque_y, self.r_foot_force_z, self.r_foot_torque_x, self.r_foot_torque_y = struct.unpack(">ffffff", buf.read(24))
        self.l_hand_force = struct.unpack('>3f', buf.read(12))
        self.l_hand_torque = struct.unpack('>3f', buf.read(12))
        self.r_hand_force = struct.unpack('>3f', buf.read(12))
        self.r_hand_torque = struct.unpack('>3f', buf.read(12))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if force_torque_t in parents: return 0
        tmphash = (0xf629e2e961e01fc) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if force_torque_t._packed_fingerprint is None:
            force_torque_t._packed_fingerprint = struct.pack(">Q", force_torque_t._get_hash_recursive([]))
        return force_torque_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

