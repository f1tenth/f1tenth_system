"""
 * Copyright(c) 2021 to 2022 ZettaScale Technology and others
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v. 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
 * v. 1.0 which is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
"""

import sys
import struct

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, List, Optional, Tuple



class Endianness(Enum):
    Little = auto()
    Big = auto()

    @staticmethod
    def native() -> 'Endianness':
        return Endianness.Little if sys.byteorder == "little" else Endianness.Big


class Buffer:
    def __init__(self, _bytes: Optional[bytes] = None, align_offset: int = 0, align_max: int = 8) -> None:
        self._bytes: bytearray = bytearray(_bytes) if _bytes else bytearray(512)
        self._pos: int = 0
        self._size: int = len(self._bytes)
        self._align_offset: int = align_offset
        self._align_max: int = align_max
        self.set_endianness(Endianness.native())

    def set_endianness(self, endianness: Endianness) -> None:
        self.endianness = endianness
        if self.endianness == Endianness.Little:
            self._endian = "<"
        else:
            self._endian = ">"

    def zero_out(self) -> None:
        # As per testing (https://stackoverflow.com/questions/19671145)
        # Quickest way to zero is to re-alloc..
        self._bytes = bytearray(self._size)

    def set_align_offset(self, offset: int) -> None:
        self._align_offset = offset

    def seek(self, pos: int) -> 'Buffer':
        self._pos = pos
        return self

    def tell(self) -> int:
        return self._pos

    def ensure_size(self, size: int) -> None:
        if self._pos + size > self._size:
            old_bytes = self._bytes
            old_size = self._size
            while self._pos + size > self._size:
                self._size *= 2
            self._bytes = bytearray(self._size)
            self._bytes[0:old_size] = old_bytes

    def align(self, alignment: int) -> 'Buffer':
        alignment = min(alignment, self._align_max)
        self._pos = ((self._pos - self._align_offset + alignment - 1) & ~(alignment - 1)) + self._align_offset
        return self

    def write(self, pack: str, size: int, value: Any) -> 'Buffer':
        self.ensure_size(size)
        struct.pack_into(self._endian + pack, self._bytes, self._pos, value)
        self._pos += size
        return self

    def write_bytes(self, _bytes: bytes) -> 'Buffer':
        length = len(_bytes)
        self.ensure_size(length)
        self._bytes[self._pos:self._pos + length] = _bytes
        self._pos += length
        return self

    def write_multi(self, pack: str, size: int, *values: Any) -> 'Buffer':
        self.ensure_size(size)
        struct.pack_into(self._endian + pack, self._bytes, self._pos, *values)
        self._pos += size
        return self

    def read_bytes(self, length: int) -> bytes:
        b = bytes(self._bytes[self._pos:self._pos + length])
        self._pos += length
        return b

    def read(self, pack: str, size: int) -> Any:
        v = struct.unpack_from(self._endian + pack, buffer=self._bytes, offset=self._pos)
        self._pos += size
        return v[0]

    def read_multi(self, pack: str, size: int) -> Tuple[Any, ...]:
        v = struct.unpack_from(self._endian + pack, buffer=self._bytes, offset=self._pos)
        self._pos += size
        return v

    def asbytes(self) -> bytes:
        return bytes(self._bytes[0:self._pos])


class KeyScanResult(Enum):
    FixedSize = 1
    BoundSize = 2
    PossiblyInfinite = 3


@dataclass
class KeyScanner:
    entries: List[List[Tuple[int, int]]] = field(default_factory=list)
    rtype: KeyScanResult = KeyScanResult.FixedSize

    @classmethod
    def infinity(cls):
        return cls(rtype=KeyScanResult.PossiblyInfinite)

    @classmethod
    def simple(cls, alignment, size):
        return cls(entries=[[(alignment, size)]])

    @classmethod
    def with_bound(cls, alignment, bound):
        return cls(entries=[[(alignment, bound)]], rtype=KeyScanResult.BoundSize)

    @property
    def size(self):
        if self.rtype != KeyScanResult.PossiblyInfinite:
            size = 0
            for possible_route in self.entries:
                ssize = 0
                for (alignment, subsize) in possible_route:
                    ssize = ((ssize + alignment - 1) & ~(alignment - 1))
                    ssize += subsize
                size = max(size, ssize)
            return size
        return 1_000_000_000

    def increase_by_bytes(self, alignment, size):
        if self.rtype != KeyScanResult.PossiblyInfinite:
            if not self.entries:
                self.entries = [(alignment, size)]
            else:
                for entry in self.entries:
                    entry.append((alignment, size))

    def increase_by_multiplied_subresult(self, subresult, times):
        if self.rtype == KeyScanResult.PossiblyInfinite:
            return

        if subresult.rtype == KeyScanResult.PossiblyInfinite:
            self.rtype = KeyScanResult.PossiblyInfinite
            self.entries.clear()
            return

        if subresult.entries and len(subresult.entries) == 1 and len(subresult.entries[0]) == 1:
            # special case to keep things managable
            entry = (subresult.entries[0][0][0], subresult.entries[0][0][1] * times)
            if not self.entries:
                self.entries = [[entry]]
            else:
                self.entries = [entry_ + [entry] for entry_ in self.entries]
            return

        entries = []
        if not self.entries and not subresult.entries:
            pass

        if not self.entries:
            entries = [entry * times for entry in subresult.entries]
        elif not subresult.entries:
            entries = self.entries.copy()
        else:
            for a_entry in self.entries:
                for b_entry in subresult.entries:
                    entries.append(a_entry + b_entry * times)

        self.entries = entries
        if subresult.rtype == KeyScanResult.BoundSize:
            self.rtype = KeyScanResult.BoundSize

    def increase_by_possibly_infinity(self):
        self.rtype = KeyScanResult.PossiblyInfinite
        self.entries.clear()

    def max(self, other):
        if self.rtype == KeyScanResult.PossiblyInfinite or other.rtype == KeyScanResult.PossiblyInfinite:
            return KeyScanner.infinity()

        return KeyScanner(entries=self.entries + other.entries, rtype=KeyScanResult.BoundSize)
