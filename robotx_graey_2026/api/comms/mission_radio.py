"""Bounded, retry-safe application messages carried in MAVLink TUNNEL.

This is GRAEY's vehicle-specific RFD mission adaptation.  fleet_link remains the
only laptop serial owner and telemetry_gateway remains the only Jetson-side Cube
connection.  The Cube/RFD pair only routes these frames; the Jetson stores and
executes the mission.
"""
import hashlib
import json
import math
import struct
import time
import zlib

PAYLOAD_TYPE = 0x9001
MAGIC = b'GM'
VERSION = 1
KIND_REQUEST = 1
KIND_RESPONSE = 2
MAX_TUNNEL = 128
MAX_COMPRESSED = 32 * 1024
MAX_UNCOMPRESSED = 128 * 1024
MAX_CHUNKS = 384
ASSEMBLY_TIMEOUT_S = 12.0
_HEADER = struct.Struct('<2sBBIHHII')
CHUNK_DATA = MAX_TUNNEL - _HEADER.size


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                      allow_nan=False).encode('ascii')


def revision(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def encode(value, request_id, kind):
    if type(request_id) is not int or not 0 < request_id <= 0xffffffff:
        raise ValueError('request_id must be a non-zero uint32')
    if kind not in (KIND_REQUEST, KIND_RESPONSE):
        raise ValueError('invalid mission-radio message kind')
    raw = canonical(value)
    if len(raw) > MAX_UNCOMPRESSED:
        raise ValueError('mission-radio message is too large')
    body = zlib.compress(raw, 9)
    if len(body) > MAX_COMPRESSED:
        raise ValueError('compressed mission-radio message is too large')
    count = max(1, math.ceil(len(body) / CHUNK_DATA))
    if count > MAX_CHUNKS:
        raise ValueError('mission-radio message has too many chunks')
    checksum = zlib.crc32(body) & 0xffffffff
    return [_HEADER.pack(MAGIC, VERSION, kind, request_id, index, count,
                         len(body), checksum) + body[index * CHUNK_DATA:(index + 1) * CHUNK_DATA]
            for index in range(count)]


def inspect_chunk(payload):
    if not isinstance(payload, (bytes, bytearray)) or not _HEADER.size <= len(payload) <= MAX_TUNNEL:
        raise ValueError('invalid mission-radio chunk length')
    magic, version, kind, request_id, index, count, total, checksum = _HEADER.unpack_from(payload)
    if magic != MAGIC or version != VERSION or kind not in (KIND_REQUEST, KIND_RESPONSE):
        raise ValueError('invalid mission-radio header')
    if not request_id or not 1 <= count <= MAX_CHUNKS or index >= count:
        raise ValueError('invalid mission-radio chunk index')
    if not 0 < total <= MAX_COMPRESSED:
        raise ValueError('invalid mission-radio compressed length')
    expected = CHUNK_DATA if index < count - 1 else total - CHUNK_DATA * (count - 1)
    data = bytes(payload[_HEADER.size:])
    if expected <= 0 or len(data) != expected:
        raise ValueError('invalid mission-radio chunk payload length')
    return dict(kind=kind, request_id=request_id, index=index, count=count,
                total=total, checksum=checksum, data=data)


class Assembler:
    def __init__(self, timeout=ASSEMBLY_TIMEOUT_S, clock=time.monotonic):
        self.timeout, self.clock, self.pending = timeout, clock, {}

    def expire(self, now=None):
        now = self.clock() if now is None else now
        expired = [key for key, item in self.pending.items() if now - item['at'] > self.timeout]
        for key in expired:
            del self.pending[key]
        return len(expired)

    def add(self, payload, now=None):
        now = self.clock() if now is None else now
        self.expire(now)
        part = inspect_chunk(payload)
        key = (part['kind'], part['request_id'])
        signature = (part['count'], part['total'], part['checksum'])
        item = self.pending.get(key)
        if item is None or item['signature'] != signature:
            item = self.pending[key] = dict(at=now, signature=signature, chunks={})
        item['at'] = now
        old = item['chunks'].get(part['index'])
        if old is not None and old != part['data']:
            del self.pending[key]
            raise ValueError('conflicting duplicate mission-radio chunk')
        item['chunks'][part['index']] = part['data']
        if len(item['chunks']) != part['count']:
            return None
        body = b''.join(item['chunks'][i] for i in range(part['count']))
        del self.pending[key]
        if len(body) != part['total'] or zlib.crc32(body) & 0xffffffff != part['checksum']:
            raise ValueError('mission-radio checksum failed')
        try:
            obj = zlib.decompressobj()
            raw = obj.decompress(body, MAX_UNCOMPRESSED + 1)
            if obj.unconsumed_tail or len(raw) > MAX_UNCOMPRESSED or not obj.eof:
                raise ValueError('mission-radio expanded message is too large or incomplete')
            raw += obj.flush()
        except zlib.error as exc:
            raise ValueError('mission-radio compression failed') from exc
        if len(raw) > MAX_UNCOMPRESSED:
            raise ValueError('mission-radio expanded message is too large')
        try:
            value = json.loads(raw.decode('ascii'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError('mission-radio JSON failed') from exc
        if not isinstance(value, dict):
            raise ValueError('mission-radio message must be an object')
        return part['kind'], part['request_id'], value
