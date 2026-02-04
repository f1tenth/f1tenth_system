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

from enum import Enum, IntFlag, auto
from inspect import isclass
from typing import Tuple, Type, Union

from . import IdlStruct, IdlUnion, IdlBitmask
from ._type_helper import get_origin, get_args
from ._type_normalize import get_idl_annotations, get_idl_field_annotations, get_extended_type_hints, WrapOpt
from ._machinery import Machine, NoneMachine, PrimitiveMachine, StringMachine, BytesMachine, ByteArrayMachine, UnionMachine, \
    ArrayMachine, SequenceMachine, InstanceMachine, MappingMachine, EnumMachine, StructMachine, OptionalMachine, CharMachine, \
    PLCdrMutableStructMachine, DelimitedCdrAppendableStructMachine, MutableMember, DelimitedCdrAppendableUnionMachine, \
    PlainCdrV2ArrayOfPrimitiveMachine, PlainCdrV2SequenceOfPrimitiveMachine, LenType, BitMaskMachine, BitBoundEnumMachine

from .types import array, bounded_str, sequence, _type_code_align_size_default_mapping, NoneType, char, typedef, uint8, \
    case, default


class XCDRSupported(IntFlag):
    SupportsBasic = 1
    SupportsV2 = 2
    SupportsBoth = SupportsBasic | SupportsV2


class Builder:
    easy_types = {
        char: CharMachine,
        str: StringMachine,
        bytes: BytesMachine,
        bytearray: ByteArrayMachine,
        NoneType: NoneMachine,
        None: NoneMachine
    }

    @classmethod
    def _scan_for_support(cls, _type, done) -> XCDRSupported:
        if id(_type) in done:
            return XCDRSupported.SupportsBoth
        # to avoid infinite recursing. Since this is the 'all' flag it
        # will not affect the result, because of the tree structure it is
        # guaranteed to be binary and'ed with the real result.
        done.add(id(_type))

        if isinstance(_type, WrapOpt):
            return XCDRSupported.SupportsV2
        elif isinstance(_type, (typedef, sequence, array)):
            return cls._scan_for_support(_type.subtype, done)
        elif get_origin(_type) == list:
            return cls._scan_for_support(get_args(_type)[0], done)
        elif get_origin(_type) == dict:
            return cls._scan_for_support(get_args(_type)[0], done) & cls._scan_for_support(get_args(_type)[1], done)
        elif isclass(_type) and issubclass(_type, IdlStruct):
            fields = get_extended_type_hints(_type)
            annotations = get_idl_annotations(_type)

            # Explicit setter
            if 'xcdrv2' in annotations:
                if annotations['xcdrv2']:
                    return XCDRSupported.SupportsV2
                else:
                    return XCDRSupported.SupportsBasic

            # Appendable and mutable is definitely V2
            if 'extensibility' in annotations:
                if annotations['extensibility'] in ['appendable', 'mutable']:
                    return XCDRSupported.SupportsV2

            # Check for optionals or nested mutable/appendable
            support = XCDRSupported.SupportsBoth
            for _, _ftype in fields.items():
                support &= cls._scan_for_support(_ftype, done)

            return support
        elif isclass(_type) and issubclass(_type, IdlUnion):
            fields = get_extended_type_hints(_type)
            annotations = get_idl_annotations(_type)

            # Explicit setter
            if 'xcdrv2' in annotations:
                if annotations['xcdrv2']:
                    return XCDRSupported.SupportsV2
                else:
                    return XCDRSupported.SupportsBasic

            # Appendable and mutable is definitely V2
            if 'extensibility' in annotations:
                if annotations['extensibility'] in ['appendable', 'mutable']:
                    return XCDRSupported.SupportsV2

            # Check for optionals or nested mutable/appendable
            support = XCDRSupported.SupportsBoth
            for _, _ftype in fields.items():
                support &= cls._scan_for_support(_ftype.subtype, done)

            return support
        return XCDRSupported.SupportsBoth


    @classmethod
    def _machine_for_type(cls, _type, add_size_header, use_version_2):
        if _type in cls.easy_types:
            return cls.easy_types[_type]()
        elif _type in _type_code_align_size_default_mapping:
            return PrimitiveMachine(_type)
        elif isinstance(_type, WrapOpt):
            return OptionalMachine(cls._machine_for_type(_type.inner, add_size_header, use_version_2))
        elif isclass(_type) and issubclass(_type, Enum):
            if "bit_bound" in get_idl_annotations(_type) and use_version_2:
                return BitBoundEnumMachine(_type, get_idl_annotations(_type)["bit_bound"])
            return EnumMachine(_type)
        elif isclass(_type) and (issubclass(_type, IdlStruct) or issubclass(_type, IdlUnion)):
            return InstanceMachine(_type, use_version_2)
        elif isclass(_type) and (issubclass(_type, IdlBitmask)):
            return BitMaskMachine(_type, get_idl_annotations(_type)["bit_bound"])
        elif get_origin(_type) == list:
            return SequenceMachine(
                cls._machine_for_type(get_args(_type)[0], add_size_header, use_version_2),
                add_size_header=add_size_header
            )
        elif get_origin(_type) == dict:
            return MappingMachine(
                cls._machine_for_type(get_args(_type)[0], add_size_header, use_version_2),
                cls._machine_for_type(get_args(_type)[1], add_size_header, use_version_2)
            )
        elif isinstance(_type, typedef):
            return cls._machine_for_type(_type.subtype, add_size_header, use_version_2)
        elif isinstance(_type, array):
            submachine = cls._machine_for_type(_type.subtype, add_size_header, use_version_2)

            if isinstance(submachine, PrimitiveMachine):
                if submachine.type == uint8:
                    return ByteArrayMachine(_type.length)

                return PlainCdrV2ArrayOfPrimitiveMachine(submachine.type, _type.length)

            asubmachine = submachine
            while isinstance(asubmachine, ArrayMachine):
                asubmachine.add_size_header = False
                asubmachine = asubmachine.submachine

            if isinstance(asubmachine, (ByteArrayMachine, PlainCdrV2ArrayOfPrimitiveMachine, CharMachine)):
                add_size_header = False

            return ArrayMachine(
                submachine,
                size=_type.length,
                add_size_header=add_size_header
            )
        elif isinstance(_type, sequence):
            submachine = cls._machine_for_type(_type.subtype, add_size_header, use_version_2)

            if isinstance(submachine, PrimitiveMachine):
                return PlainCdrV2SequenceOfPrimitiveMachine(submachine.type, max_length=_type.max_length)

            if isinstance(submachine, (CharMachine)):
                add_size_header = False

            return SequenceMachine(
                submachine,
                maxlen=_type.max_length,
                add_size_header=add_size_header
            )
        elif isinstance(_type, bounded_str):
            return StringMachine(
                bound=_type.max_length
            )

        raise TypeError(f"{_type} is not valid in IDL classes because it cannot be encoded.")

    @classmethod
    def _machine_struct(cls, struct: Type[IdlStruct]) -> Tuple[Machine, bool]:
        fields = get_extended_type_hints(struct)
        annotations = get_idl_annotations(struct)
        field_annotations = get_idl_field_annotations(struct)

        keylist = annotations.get("keylist")
        if not keylist:
            keylist = []
            for name, f_annotations in field_annotations.items():
                if "key" in f_annotations:
                    keylist.append(name)

            if not keylist:
                keylist = None

        keyless = keylist is None or len(keylist) == 0

        extensibility = annotations.get("extensibility")

        v0_members = {
            name: cls._machine_for_type(field_type, False, False)
            for name, field_type in fields.items()
        }
        v2_members = {
            name: cls._machine_for_type(field_type, True, True)
            for name, field_type in fields.items()
        }

        v0_machine = StructMachine(struct, v0_members, keylist)

        if extensibility and extensibility != "final":
            if extensibility == "appendable":
                v2_machine = DelimitedCdrAppendableStructMachine(
                    struct, v2_members, keylist
                )
            elif extensibility == "mutable":
                mutablemembers = []

                for name, machine in v2_members.items():
                    optional = False
                    if isinstance(machine, OptionalMachine):
                        optional = True
                        machine = machine.submachine

                    lentype = None
                    if isinstance(machine, PrimitiveMachine):
                        lentype = {
                            1: LenType.OneByte,
                            2: LenType.TwoByte,
                            4: LenType.FourByte,
                            8: LenType.EightByte
                        }[machine.size]
                    elif isinstance(machine, PlainCdrV2SequenceOfPrimitiveMachine):
                        if machine.size == 1:
                            lentype = LenType.NextIntDualUseLen
                        elif machine.size == 4:
                            lentype = LenType.NextIntDualUse4Len
                        elif machine.size == 8:
                            lentype = LenType.NextIntDualUse8Len
                        else:
                            lentype = LenType.NextIntLen
                    elif isinstance(machine, SequenceMachine):
                        lentype = LenType.NextIntDualUseLen
                    else:
                        lentype = LenType.NextIntLen

                    mutablemembers.append(MutableMember(
                        name=name,
                        key=not keylist or name in keylist,
                        optional=optional,
                        lentype=lentype,
                        must_understand=field_annotations.get(name, {}).get("must_understand", False),
                        memberid=struct.__idl__.get_member_id(name),
                        machine=machine
                    ))

                v2_machine = PLCdrMutableStructMachine(struct, mutablemembers)
        else:
            v2_machine = StructMachine(struct, v2_members, keylist)

        return v0_machine, v2_machine, keyless

    @classmethod
    def _machine_union(cls, union: Type[IdlUnion]):
        annotations = get_idl_annotations(union)
        extensibility = annotations.get("extensibility")

        v0_cases = {}
        v2_cases = {}
        v0_default = None
        v2_default = None

        for _case in get_extended_type_hints(union).values():
            if isinstance(_case, default):
                v0_default = cls._machine_for_type(_case.subtype, False, False)
                v2_default = cls._machine_for_type(_case.subtype, True, True)
                continue

            if not isinstance(_case, case):
                continue

            v0_machine = cls._machine_for_type(_case.subtype, False, False)
            v2_machine = cls._machine_for_type(_case.subtype, True, True)

            for label in _case.labels:
                v0_cases[label] = v0_machine
                v2_cases[label] = v2_machine

        v0_discriminator = cls._machine_for_type(union.__idl_discriminator__, False, False)
        v2_discriminator = cls._machine_for_type(union.__idl_discriminator__, True, True)

        v0_machine = UnionMachine(union, v0_discriminator, v0_cases, v0_default)

        if extensibility is None or extensibility == "final":
            v2_machine = UnionMachine(union, v2_discriminator, v2_cases, v2_default)

        elif extensibility == "appendable":
            v2_machine = DelimitedCdrAppendableUnionMachine(union, v2_discriminator, v2_cases, v2_default)
        else:
            # mutable
            raise NotImplementedError()

        return v0_machine, v2_machine

    @classmethod
    def build_machines(cls, _type):
        if issubclass(_type, IdlUnion):
            v0_machine, v2_machine = cls._machine_union(_type)
            keyless = False
        elif issubclass(_type, IdlStruct):
            v0_machine, v2_machine, keyless = cls._machine_struct(_type)
        else:
            raise Exception(f"Cannot build for {_type}, not struct or union.")

        return v0_machine, v2_machine, keyless, cls._scan_for_support(_type, set())
