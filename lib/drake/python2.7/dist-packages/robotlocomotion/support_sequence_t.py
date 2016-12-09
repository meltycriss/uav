"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import robotlocomotion.support_element_t

class support_sequence_t(object):
    __slots__ = ["utime", "num_ts", "ts", "supports"]

    def __init__(self):
        self.utime = 0
        self.num_ts = 0
        self.ts = []
        self.supports = []

    def encode(self):
        buf = BytesIO()
        buf.write(support_sequence_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.num_ts))
        buf.write(struct.pack('>%dd' % self.num_ts, *self.ts[:self.num_ts]))
        for i0 in range(self.num_ts):
            assert self.supports[i0]._get_packed_fingerprint() == robotlocomotion.support_element_t._get_packed_fingerprint()
            self.supports[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != support_sequence_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return support_sequence_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = support_sequence_t()
        self.utime, self.num_ts = struct.unpack(">qi", buf.read(12))
        self.ts = struct.unpack('>%dd' % self.num_ts, buf.read(self.num_ts * 8))
        self.supports = []
        for i0 in range(self.num_ts):
            self.supports.append(robotlocomotion.support_element_t._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if support_sequence_t in parents: return 0
        newparents = parents + [support_sequence_t]
        tmphash = (0xf184858f59b0b6df+ robotlocomotion.support_element_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if support_sequence_t._packed_fingerprint is None:
            support_sequence_t._packed_fingerprint = struct.pack(">Q", support_sequence_t._get_hash_recursive([]))
        return support_sequence_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
