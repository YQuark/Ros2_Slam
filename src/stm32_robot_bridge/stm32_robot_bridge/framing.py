"""Version-neutral upper-protocol framing primitives."""

from dataclasses import dataclass
from typing import List, Tuple

FRAME_SOF = 0xA5
FRAME_SOF2 = 0x5A
MAX_PAYLOAD_SIZE = 99
MAX_LENGTH = MAX_PAYLOAD_SIZE + 1


def crc8(data: bytes) -> int:
    """CRC-8/MAXIM, bit-identical to the STM32 implementation."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 1) else (crc >> 1)
    return crc & 0xFF


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload too large: {len(payload)}")
    body = bytes([len(payload) + 1, int(cmd) & 0xFF]) + payload
    return bytes([FRAME_SOF, FRAME_SOF2]) + body + bytes([crc8(body)])


@dataclass
class ParserStats:
    crc_errors: int = 0
    bad_length: int = 0
    resync_bytes: int = 0


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = ParserStats()

    def feed(self, chunk: bytes) -> List[Tuple[int, bytes]]:
        self._buffer.extend(chunk)
        frames: List[Tuple[int, bytes]] = []
        while True:
            header = self._find_header()
            if header < 0:
                self.stats.resync_bytes += self._retain_header_tail()
                break
            if header:
                self.stats.resync_bytes += header
                del self._buffer[:header]
            if len(self._buffer) < 3:
                break
            length = self._buffer[2]
            if length == 0 or length > MAX_LENGTH:
                self.stats.bad_length += 1
                self.stats.resync_bytes += 1
                del self._buffer[0]
                continue
            frame_length = length + 4
            if len(self._buffer) < frame_length:
                break
            body = bytes(self._buffer[2 : 3 + length])
            if crc8(body) != self._buffer[frame_length - 1]:
                self.stats.crc_errors += 1
                self.stats.resync_bytes += 1
                del self._buffer[0]
                continue
            frames.append((self._buffer[3], bytes(self._buffer[4 : 3 + length])))
            del self._buffer[:frame_length]
        return frames

    def _find_header(self) -> int:
        for index in range(max(0, len(self._buffer) - 1)):
            if self._buffer[index] == FRAME_SOF and self._buffer[index + 1] == FRAME_SOF2:
                return index
        return -1

    def _retain_header_tail(self) -> int:
        if self._buffer and self._buffer[-1] == FRAME_SOF:
            removed = len(self._buffer) - 1
            del self._buffer[:-1]
            return removed
        removed = len(self._buffer)
        self._buffer.clear()
        return removed
