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

from math import log2
from enum import Enum
from dataclasses import dataclass

from .types import _type_code_align_size_default_mapping
from ._support import KeyScanner
from . import types as types


class Machine:
    """Given a type, serialize and deserialize"""
    def __init__(self, type):
        self.alignment = 1

    def serialize(self, buffer, value, for_key=False):
        pass

    def deserialize(self, buffer):
        pass

    def key_scan(self) -> KeyScanner:
        pass

    def default_initialize(self):
        pass


class NoneMachine(Machine):
    def __init__(self):
        self.alignment = 1

    def serialize(self, buffer, value, for_key=False):
        pass

    def deserialize(self, buffer):
        pass

    def key_scan(self) -> KeyScanner:
        return KeyScanner()

    def default_initialize(self):
        return None


class PrimitiveMachine(Machine):
    def __init__(self, type):
        self.type = type
        self.code, self.alignment, self.size, self.default = _type_code_align_size_default_mapping[type]

    def serialize(self, buffer, value, for_key=False):
        buffer.align(self.alignment)
        buffer.write(self.code, self.size, value)

    def deserialize(self, buffer):
        buffer.align(self.alignment)
        return buffer.read(self.code, self.size)

    def key_scan(self) -> KeyScanner:
        return KeyScanner.simple(self.alignment, self.size)

    def default_initialize(self):
        return self.default


class CharMachine(Machine):
    def __init__(self):
        self.alignment = 1

    def serialize(self, buffer, value, for_key=False):
        buffer.write('b', 1, ord(value))

    def deserialize(self, buffer):
        return chr(buffer.read('b', 1))

    def default_initialize(self):
        return '\0'

    def key_scan(self) -> KeyScanner:
        return KeyScanner.simple(1, 1)


class StringMachine(Machine):
    def __init__(self, bound=None):
        self.alignment = 4
        self.bound = bound

    def serialize(self, buffer, value, for_key=False):
        if self.bound and len(value) > self.bound:
            raise Exception("String longer than bound.")
        buffer.align(4)
        bytes = value.encode('utf-8')
        buffer.write('I', 4, len(bytes) + 1)
        buffer.write_bytes(bytes)
        buffer.write('b', 1, 0)

    def deserialize(self, buffer):
        buffer.align(4)
        numbytes = buffer.read('I', 4)
        bytes = buffer.read_bytes(numbytes - 1)
        buffer.read('b', 1)
        return bytes.decode('utf-8')

    def key_scan(self) -> KeyScanner:
        if self.bound:
            return KeyScanner.with_bound(4, self.bound + 4)
        else:
            return KeyScanner.infinity()

    def default_initialize(self):
        return ""


class BytesMachine(Machine):
    def __init__(self, bound=None):
        self.alignment = 4
        self.bound = bound

    def serialize(self, buffer, value, for_key=False):
        if self.bound and len(value) > self.bound:
            raise Exception("Bytes longer than bound.")
        buffer.align(4)
        buffer.write('I', 4, len(value))
        buffer.write_bytes(value)

    def deserialize(self, buffer):
        buffer.align(4)
        numbytes = buffer.read('I', 4)
        return buffer.read_bytes(numbytes)

    def key_scan(self) -> KeyScanner:
        if self.bound:
            return KeyScanner.with_bound(4, self.bound + 4)
        else:
            return KeyScanner.infinity()

    def default_initialize(self):
        return bytes(0)


class ByteArrayMachine(Machine):
    def __init__(self, size):
        self.alignment = 1
        self.size = size

    def serialize(self, buffer, value, for_key=False):
        if len(value) != self.size:
            raise Exception("Incorrectly sized array.")

        buffer.write_bytes(value)

    def deserialize(self, buffer):
        return buffer.read_bytes(self.size)

    def key_scan(self) -> KeyScanner:
        return KeyScanner.simple(1, self.size)

    def default_initialize(self):
        return bytearray(self.size)


class ArrayMachine(Machine):
    def __init__(self, submachine, size, add_size_header=False):
        self.size = size
        self.submachine = submachine
        self.alignment = submachine.alignment
        self.add_size_header = add_size_header

    def serialize(self, buffer, value, for_key=False):
        assert len(value) == self.size

        if self.add_size_header:
            buffer.align(4)
            buffer.write('I', 4, 0)
            hpos = buffer.tell()

        for v in value:
            self.submachine.serialize(buffer, v, for_key)

        if self.add_size_header:
            mpos = buffer.tell()
            buffer.seek(hpos - 4)
            buffer.write('I', 4, mpos - hpos)
            buffer.seek(mpos)

    def deserialize(self, buffer):
        if self.add_size_header:
            buffer.align(4)
            size = buffer.read('I', 4)
            mpos = buffer.tell()

        v = [self.submachine.deserialize(buffer) for i in range(self.size)]

        if self.add_size_header:
            assert buffer.tell() == mpos + size

        return v

    def key_scan(self) -> KeyScanner:
        scan = KeyScanner()
        scan.increase_by_multiplied_subresult(self.submachine.key_scan(), self.size)
        return scan

    def default_initialize(self):
        return [self.submachine.default_initialize() for i in range(self.size)]


class SequenceMachine(Machine):
    def __init__(self, submachine, maxlen=None, add_size_header=False):
        self.submachine = submachine
        self.alignment = 4
        self.maxlen = maxlen
        self.add_size_header = add_size_header

    def serialize(self, buffer, value, for_key=False):
        if self.maxlen is not None:
            assert len(value) <= self.maxlen

        buffer.align(4)

        if self.add_size_header:
            buffer.write('I', 4, 0)
            hpos = buffer.tell()

        buffer.write('I', 4, len(value))

        for v in value:
            self.submachine.serialize(buffer, v, for_key)

        if self.add_size_header:
            mpos = buffer.tell()
            buffer.seek(hpos - 4)
            buffer.write('I', 4, mpos - hpos)
            buffer.seek(mpos)

    def deserialize(self, buffer):
        buffer.align(4)

        if self.add_size_header:
            size = buffer.read('I', 4)
            mpos = buffer.tell()

        num = buffer.read('I', 4)
        v = [self.submachine.deserialize(buffer) for i in range(num)]

        if self.add_size_header:
            buffer.seek(mpos + size)

        return v

    def key_scan(self) -> KeyScanner:
        if not self.maxlen:
            return KeyScanner.infinity()

        if self.add_size_header:
            scan = KeyScanner.with_bound(4, 8)
        else:
            scan = KeyScanner.with_bound(4, 4)

        scan.increase_by_multiplied_subresult(self.submachine.key_scan(), self.maxlen)

        if scan.size > 1_000_000:
            return KeyScanner.infinity()
        return scan

    def default_initialize(self):
        return []


class UnionMachine(Machine):
    def __init__(self, type, discriminator_machine, labels_submachines, default_case=None):
        self.type = type
        self.labels_submachines = labels_submachines
        self.alignment = max(s.alignment for s in labels_submachines.values())
        self.alignment = max(self.alignment, discriminator_machine.alignment)
        self.discriminator = discriminator_machine
        self.default = default_case
        self.discriminator_is_key = type.__idl_annotations__.get("discriminator_is_key", False)

    def serialize(self, buffer, union, for_key=False):
        discr, value = union.get()

        if for_key and self.discriminator_is_key:
            try:
                if discr is None:
                    self.discriminator.serialize(buffer, union.__idl_default_discriminator__, for_key)
                else:
                    self.discriminator.serialize(buffer, discr, for_key)
                return
            except Exception as e:
                raise Exception(f"Failed to encode union, {self.type}, value is {value}, discriminator is {discr}") from e

        try:
            if discr is None:
                self.discriminator.serialize(buffer, union.__idl_default_discriminator__, for_key)
                if self.default:
                    self.default.serialize(buffer, value, for_key)
            elif discr not in self.labels_submachines:
                self.discriminator.serialize(buffer, discr, for_key)
                if self.default:
                    self.default.serialize(buffer, value, for_key)
            else:
                self.discriminator.serialize(buffer, discr, for_key)
                self.labels_submachines[discr].serialize(buffer, value, for_key)
        except Exception as e:
            raise Exception(f"Failed to encode union, {self.type}, value is {value}, discriminator is {discr}") from e

    def deserialize(self, buffer):
        label = self.discriminator.deserialize(buffer)

        if label not in self.labels_submachines:
            if self.default:
                contents = self.default.deserialize(buffer)
            else:
                contents = None
        else:
            contents = self.labels_submachines[label].deserialize(buffer)

        return self.type(discriminator=label, value=contents)

    def key_scan(self) -> KeyScanner:
        dscan = self.discriminator.key_scan()
        if self.discriminator_is_key:
            return dscan

        scan = KeyScanner()
        for _, machine in self.labels_submachines.items():
            scan = scan.max(machine.key_scan())

        dscan.increase_by_multiplied_subresult(scan, 1)
        return dscan

    def default_initialize(self):
        return self.type(
            discriminator=self.type.__idl_default_discriminator__,
            value=None if self.default is None else self.default.default_initialize()
        )


class MappingMachine(Machine):
    def __init__(self, key_machine, value_machine):
        self.key_machine = key_machine
        self.value_machine = value_machine
        self.alignment = 4

    def serialize(self, buffer, values, for_key=False):
        buffer.align(4)
        buffer.write('I', 4, len(values))

        for key, value in values.items():
            self.key_machine.serialize(buffer, key, for_key)
            self.value_machine.serialize(buffer, value, for_key)

    def deserialize(self, buffer):
        ret = {}
        buffer.align(4)
        num = buffer.read('I', 4)

        for _i in range(num):
            key = self.key_machine.deserialize(buffer)
            value = self.value_machine.deserialize(buffer)
            ret[key] = value

        return ret

    def key_scan(self) -> KeyScanner:
        return KeyScanner.infinity()

    def default_initialize(self):
        return {}


class StructMachine(Machine):
    def __init__(self, object, members_machines, keylist):
        self.type = object
        self.members_machines = members_machines
        self.keylist = keylist

    def serialize(self, buffer, value, for_key=False):
        #  We use the fact here that dicts retain their insertion order
        #  This is guaranteed from python 3.7

        for member, machine in self.members_machines.items():
            if for_key and self.keylist and member not in self.keylist:
                continue

            try:
                machine.serialize(buffer, getattr(value, member), for_key)
            except Exception as e:
                raise Exception(f"Failed to encode member {member}, value is {getattr(value, member)}") from e

    def deserialize(self, buffer):
        valuedict = {}
        for member, machine in self.members_machines.items():
            valuedict[member] = machine.deserialize(buffer)
        return self.type(**valuedict)

    def key_scan(self) -> KeyScanner:
        scan = KeyScanner()

        for member, machine in self.members_machines.items():
            if self.keylist and member not in self.keylist:
                continue
            scan.increase_by_multiplied_subresult(machine.key_scan(), 1)

        return scan

    def default_initialize(self):
        valuedict = {}
        for member, machine in self.members_machines.items():
            valuedict[member] = machine.default_initialize()
        return self.type(**valuedict)


class InstanceMachine(Machine):
    def __init__(self, object, use_version_2):
        self.type = object
        self.alignment = 1
        self.use_version_2 = use_version_2

    def serialize(self, buffer, value, for_key=False):
        if self.type.__idl__.v0_machine is None:
            self.type.__idl__.populate()

        if self.use_version_2:
            return self.type.__idl__.v2_machine.serialize(buffer, value, for_key)
        else:
            return self.type.__idl__.v0_machine.serialize(buffer, value, for_key)

    def deserialize(self, buffer):
        if self.type.__idl__.v0_machine is None:
            self.type.__idl__.populate()

        if self.use_version_2:
            return self.type.__idl__.v2_machine.deserialize(buffer)
        else:
            return self.type.__idl__.v0_machine.deserialize(buffer)

    def key_scan(self):
        return self.type.__idl__.key_scan(use_version_2=self.use_version_2)

    def default_initialize(self):
        if self.type.__idl__.v0_machine is None:
            self.type.__idl__.populate()

        if self.use_version_2:
            return self.type.__idl__.v2_machine.default_initialize()
        else:
            return self.type.__idl__.v0_machine.default_initialize()


class EnumMachine(Machine):
    def __init__(self, enum):
        self.enum: Enum = enum
        self.alignment = 4
        self.size = 4

    def serialize(self, buffer, value, for_key=False):
        buffer.align(4)
        if type(value) == int:
            buffer.write("I", 4, value)
            return
        buffer.write("I", 4, value.value)

    def deserialize(self, buffer):
        buffer.align(4)
        v = buffer.read("I", 4)
        try:
            return self.enum(v)
        except ValueError:
            return v

    def key_scan(self) -> KeyScanner:
        return KeyScanner.simple(4, 4)

    def default_initialize(self):
        return self.enum.__idl_enum_default_value__


class BitBoundEnumMachine(Machine):
    def __init__(self, enum, bit_bound):
        self.bit_bound = bit_bound
        self.encoder = [types.uint8, types.uint16, types.uint32, types.uint64][int(log2(bit_bound)) - 3]
        self.enum: Enum = enum
        self.code, self.alignment, self.size, _ = _type_code_align_size_default_mapping[self.encoder]

    def serialize(self, buffer, value, for_key=False):
        if type(value) == int:
            buffer.align(self.alignment)
            buffer.write(self.code, self.size, value)
            return
        buffer.align(self.alignment)
        buffer.write(self.code, self.size, value.value)

    def deserialize(self, buffer):
        buffer.align(self.alignment)
        v = buffer.read(self.code, self.size)
        try:
            return self.enum(v)
        except ValueError:
            return v

    def key_scan(self) -> KeyScanner:
        return KeyScanner.simple(self.alignment, self.size)

    def default_initialize(self):
        return self.enum.__idl_enum_default_value__


class OptionalMachine(Machine):
    def __init__(self, submachine):
        self.submachine = submachine

    def serialize(self, buffer, value, for_key=False):
        assert not for_key
        if value is None:
            buffer.write('?', 1, False)
        else:
            buffer.write('?', 1, True)
            self.submachine.serialize(buffer, value, for_key)

    def deserialize(self, buffer):
        if buffer.read('?', 1):
            return self.submachine.deserialize(buffer)
        return None

    def key_scan(self) -> KeyScanner:
        scan = KeyScanner.simple(1, 1)
        scan.increase_by_multiplied_subresult(self.submachine.key_scan(), 1)
        return scan

    def default_initialize(self):
        return None


class PlainCdrV2ArrayOfPrimitiveMachine(Machine):
    def __init__(self, type, length):
        code, self.alignment, size, default = types._type_code_align_size_default_mapping[type]
        self.length = length
        self.size = size * length
        self.code = str(length) + code
        self.default = [default] * length
        self.subtype = type

    def serialize(self, buffer, value, for_key=False):
        assert len(value) == self.length
        buffer.align(self.alignment)
        buffer.write_multi(self.code, self.size, *value)

    def deserialize(self, buffer):
        buffer.align(self.alignment)
        return list(buffer.read_multi(self.code, self.size))

    def key_scan(self) -> KeyScanner:
        return KeyScanner.simple(self.alignment, self.size)

    def default_initialize(self):
        return self.default.copy()


class PlainCdrV2SequenceOfPrimitiveMachine(Machine):
    def __init__(self, type, max_length=None):
        self.code, self.alignment, self.size, _ = types._type_code_align_size_default_mapping[type]
        self.max_length = max_length

    def serialize(self, buffer, value, for_key=False):
        assert self.max_length is None or len(value) <= self.max_length
        buffer.align(4)
        buffer.write('I', 4, len(value))
        if value:
            buffer.align(self.alignment)
            buffer.write_multi(f"{len(value)}{self.code}", self.size * len(value), *value)

    def deserialize(self, buffer):
        buffer.align(4)
        length = buffer.read('I', 4)
        if length:
            buffer.align(self.alignment)
            return list(buffer.read_multi(f"{length}{self.code}", self.size * length))
        else:
            return []

    def key_scan(self) -> KeyScanner:
        if not self.max_length:
            return KeyScanner.infinity()

        scan = KeyScanner.with_bound(4, 4)
        scan.increase_by_multiplied_subresult(
            KeyScanner.simple(self.size, self.size),
            self.max_length
        )

        if scan.size > 1_000_000:
            return KeyScanner.infinity()
        return scan

    def default_initialize(self):
        return []


class DelimitedCdrAppendableStructMachine(Machine):
    def __init__(self, type, member_machines, keylist):
        self.alignment = 4
        self.type = type
        self.member_machines = member_machines
        self.keylist = keylist

    def serialize(self, buffer, value, for_key=False):
        # write dummy header
        if not for_key:
            buffer.align(4)
            hpos = buffer.tell()
            buffer.write('I', 4, 0)

        # write member data
        dpos = buffer.tell()
        for member, machine in self.member_machines.items():
            if for_key and self.keylist and member not in self.keylist:
                continue

            try:
                machine.serialize(buffer, getattr(value, member), for_key)
            except Exception as e:
                raise Exception(f"Failed to encode member {member}, value is {getattr(value, member)}") from e

        if not for_key:
            fpos = buffer.tell()

            # Write size header word back
            buffer.seek(hpos)
            buffer.write('I', 4, fpos - dpos)
            buffer.seek(fpos)

    def deserialize(self, buffer):
        # read header
        buffer.align(4)
        size = buffer.read('I', 4)
        hpos = buffer.tell()

        data = {}
        for member, machine in self.member_machines.items():
            if buffer.tell() - hpos < size:
                data[member] = machine.deserialize(buffer)
            else:
                data[member] = machine.default_initialize()

            if buffer.tell() - hpos > size:
                raise Exception("Struct was not contained inside header indicated size, stream corrupt.")

        buffer.seek(hpos + size)
        return self.type(**data)

    def key_scan(self) -> KeyScanner:
        scan = KeyScanner()

        for member, machine in self.member_machines.items():
            if self.keylist and member not in self.keylist:
                continue
            scan.increase_by_multiplied_subresult(machine.key_scan(), 1)

        return scan

    def default_initialize(self):
        valuedict = {}
        for member, machine in self.member_machines.items():
            valuedict[member] = machine.default_initialize()
        return self.type(**valuedict)


class DelimitedCdrAppendableUnionMachine(Machine):
    def __init__(self, type, discriminator_machine, labels_submachines, default_case=None):
        self.type = type
        self.labels_submachines = labels_submachines
        self.alignment = 4
        self.discriminator = discriminator_machine
        self.default = default_case
        self.discriminator_is_key = type.__idl_annotations__.get("discriminator_is_key", False)

    def serialize(self, buffer, union, for_key=False):
        # write dummy header
        if not for_key:
            buffer.align(4)
            hpos = buffer.tell()
            buffer.write('I', 4, 0)

        dpos = buffer.tell()
        discr, value = union.get()

        if for_key and self.discriminator_is_key:
            try:
                if discr is None:
                    self.discriminator.serialize(buffer, union.__idl_default_discriminator__, for_key)
                else:
                    self.discriminator.serialize(buffer, discr, for_key)
                return
            except Exception as e:
                raise Exception(f"Failed to encode union, {self.type}, value is {value}") from e

        try:
            if discr is None:
                self.discriminator.serialize(buffer, union.__idl_default_discriminator__)
                if self.default:
                    self.default.serialize(buffer, value, for_key)
            elif discr not in self.labels_submachines:
                self.discriminator.serialize(buffer, discr, for_key)
                if self.default:
                    self.default.serialize(buffer, value, for_key)
            else:
                self.discriminator.serialize(buffer, discr, for_key)
                self.labels_submachines[discr].serialize(buffer, value, for_key)
        except Exception as e:
            raise Exception(f"Failed to encode union, {self.type}, value is {value}") from e

        if not for_key:
            fpos = buffer.tell()

            # Write size header word back
            buffer.seek(hpos)
            buffer.write('I', 4, fpos - dpos)
            buffer.seek(fpos)

    def deserialize(self, buffer):
        # read header
        buffer.align(4)
        size = buffer.read('I', 4)
        hpos = buffer.tell()

        label = self.discriminator.deserialize(buffer)
        lpos = buffer.tell()

        if label not in self.labels_submachines:
            if lpos == hpos + size:
                return self.default_initialize()

            if self.default:
                contents = self.default.deserialize(buffer)
            else:
                contents = None
        else:
            if lpos == hpos + size:
                contents = self.labels_submachines[label].default_initialize()
            else:
                contents = self.labels_submachines[label].deserialize(buffer)

        buffer.seek(hpos + size)
        return self.type(discriminator=label, value=contents)

    def key_scan(self) -> KeyScanner:
        dscan = self.discriminator.key_scan()
        if self.discriminator_is_key:
            return dscan

        scan = KeyScanner()
        for _, machine in self.labels_submachines.items():
            scan = scan.max(machine.key_scan())

        dscan.increase_by_multiplied_subresult(scan, 1)
        return dscan

    def default_initialize(self):
        return self.type(
            discriminator=self.type.__idl_default_discriminator__,
            value=None if self.default is None else self.default.default_initialize()
        )


# Todo: Mutable unions

class LenType(Enum):
    OneByte = 0
    TwoByte = 1
    FourByte = 2
    EightByte = 3
    NextIntLen = 4
    NextIntDualUseLen = 5
    NextIntDualUse4Len = 6
    NextIntDualUse8Len = 7


@dataclass
class MutableMember:
    name: str
    key: bool
    optional: bool
    lentype: LenType
    must_understand: bool
    memberid: int
    machine: Machine
    header: int = 0

    def __post_init__(self):
        self.header = ((1 if self.must_understand else 0) << 31) + (self.lentype.value << 28) + (self.memberid & 0x0fffffff)


class MustUnderstandFailure(Exception):
    pass


class PLCdrMutableStructMachine(Machine):
    def __init__(self, type, mutablemembers):
        self.alignment = 4
        self.type = type
        self.mutablemembers = mutablemembers
        self.mutmem_by_id = {
            m.memberid: m for m in mutablemembers
        }
        self.init_map = {
            m.name: None for m in mutablemembers
        }

    def serialize(self, buffer, value, for_key=False):
        if not for_key:
            # write dummy header
            buffer.align(4)
            hpos = buffer.tell()
            buffer.write('I', 4, 0)

        if for_key:
            for m_id in sorted(self.mutmem_by_id.keys()):
                mutablemember = self.mutmem_by_id[m_id]
                if mutablemember.key:
                    member_value = getattr(value, mutablemember.name)
                    mutablemember.machine.serialize(buffer, member_value, True)
        else:
            # write member data
            dpos = buffer.tell()
            for mutablemember in self.mutablemembers:
                member_value = getattr(value, mutablemember.name)

                if mutablemember.optional and member_value is None:
                    continue

                buffer.align(4)
                buffer.write('I', 4, mutablemember.header)

                mpos = buffer.tell()
                if mutablemember.lentype == LenType.NextIntLen:
                    buffer.write('I', 4, 0)

                mutablemember.machine.serialize(buffer, member_value)

                if mutablemember.lentype == LenType.NextIntLen:
                    ampos = buffer.tell()
                    buffer.seek(mpos)
                    buffer.write('I', 4, ampos - mpos - 4)
                    buffer.seek(ampos)

        if not for_key:
            fpos = buffer.tell()

            # Write size header word back
            buffer.seek(hpos)
            buffer.write('I', 4, fpos - dpos)
            buffer.seek(fpos)

    def deserialize(self, buffer):
        # read header
        buffer.align(4)
        struct_size = buffer.read('I', 4)
        hpos = buffer.tell()

        data = self.init_map.copy()
        while buffer.tell() - hpos < struct_size:
            buffer.align(4)
            header = buffer.read('I', 4)
            must_understand = ((header >> 31) & 1) > 0
            lc = (header >> 28) & 0x7
            memberid = header & 0x0fffffff
            mutmem = self.mutmem_by_id.get(memberid)

            if mutmem:
                if lc == 4:
                    buffer.read('I', 4)
                data[mutmem.name] = mutmem.machine.deserialize(buffer)
            else:
                if must_understand:
                    # Got a member that we don't know and marked as must understand: failure
                    raise MustUnderstandFailure()
                mpos = buffer.tell()
                if lc < 4:
                    buffer.seek(mpos + 2 ** lc)
                else:
                    size = buffer.read('I', 4)
                    if lc == 6:
                        size *= 4
                    elif lc == 7:
                        size *= 8
                    buffer.seek(mpos + size + 4)

        for mutmem in self.mutablemembers:
            if data[mutmem.name] is None and not mutmem.optional:
                data[mutmem.name] = mutmem.machine.default_initialize()

        buffer.seek(hpos + struct_size)
        return self.type(**data)

    def key_scan(self) -> KeyScanner:
        scan = KeyScanner()

        for m_id in sorted(self.mutmem_by_id.keys()):
            mutablemember = self.mutmem_by_id[m_id]
            if not mutablemember.key:
                continue
            scan.increase_by_multiplied_subresult(mutablemember.machine.key_scan(), 1)

        return scan

    def default_initialize(self):
        data = {}
        for mutmem in self.mutablemembers:
            if not mutmem.optional:
                data[mutmem.name] = mutmem.machine.default_initialize()
            else:
                data[mutmem.name] = None
        return self.type(**data)


class BitMaskMachine(Machine):
    def __init__(self, type, bit_bound=64):
        self.bit_bound = bit_bound
        self.type = type
        self.primitive_type = \
            types.uint8 if bit_bound < 9 else \
            types.uint16 if bit_bound < 17 else \
            types.uint32 if bit_bound < 33 else \
            types.uint64
        self.code, self.alignment, self.size, self.default = \
            types._type_code_align_size_default_mapping[self.primitive_type]

    def serialize(self, buffer, value, for_key=False):
        buffer.align(self.alignment)
        buffer.write(self.code, self.size, value.as_mask())

    def deserialize(self, buffer):
        buffer.align(self.alignment)
        return self.type.from_mask(buffer.read(self.code, self.size))

    def key_scan(self) -> KeyScanner:
        return KeyScanner.simple(self.alignment, self.size)

    def default_initialize(self):
        return self.default
