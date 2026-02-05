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

from typing import Optional, cast, Any, ClassVar, Mapping, Dict, Tuple, TYPE_CHECKING
from collections import deque
from enum import EnumMeta, Enum
from inspect import isclass
from struct import unpack
from hashlib import md5

from ._support import Buffer, Endianness, KeyScanner, KeyScanResult
from ._type_helper import get_origin, get_args, Annotated
from ._type_normalize import get_idl_annotations, get_idl_field_annotations, get_extended_type_hints
from ._machinery import Machine

from . import types


if TYPE_CHECKING:
    from ._typesupport.DDS.XTypes import TypeMapping, TypeInformation, TypeIdentifier
    from ._builder import XCDRSupported
    from pycdr2 import IdlEnum


class IDLNamespaceScope:
    current = None
    stack = deque()

    @classmethod
    def enter(cls, scope):
        if cls.current is not None:
            cls.stack.append(cls.current)
        cls.current = scope

    @classmethod
    def exit(cls):
        if cls.stack:
            cls.current = cls.stack.pop()
        else:
            cls.current = None


class IDL:
    def __init__(self, datatype):
        self._populated: bool = False
        self.buffer: Buffer = Buffer()
        self.datatype: type = datatype
        self.keyless: bool = None
        self.v0_machine: Machine = None
        self.v2_machine: Machine = None
        self.v0_key_max_size: int = None
        self.v2_key_max_size: int = None
        self.version_support: XCDRSupported = None

        self.idl_transformed_typename: str = self.datatype.__idl_typename__.replace(".", "::")
        self.re_entrancy_protection: bool = False
        self._xt_data: Tuple[TypeInformation, TypeMapping] = (None, None)
        self._xt_bytedata: Tuple[Optional[bytes], Optional[bytes]] = (None, None)
        self.member_ids: Dict[str, int] = None

    def populate(self):
        if not self._populated:
            self._populated = True
            annotations = get_idl_annotations(self.datatype)
            field_annotations = get_idl_field_annotations(self.datatype)

            a = annotations.get('extensibility', 'final')
            if a == 'appendable':
                self.xcdrv2_head = 0x08
            elif a == 'mutable':
                self.xcdrv2_head = 0x0a
            else:
                self.xcdrv2_head = 0x06

            if self.member_ids is None:
                ids = {}
                is_hash_id = annotations.get("autoid", "sequential") == "hash"
                idc = 0

                for name, _ in get_extended_type_hints(self.datatype).items():
                    f_annot = field_annotations.get(name, {})

                    if "id" in f_annot:
                        mid = f_annot["id"]
                    elif "hash_id" in f_annot or is_hash_id:
                        # compute 4 byte hash, interpret as little endian 32 bit integer and zero out top four bits
                        mid = unpack("<I", md5(f_annot.get("hash_id", "") or name.encode()).digest()[:4])[0] & 0x0FFFFFFF
                    else:
                        mid = idc

                    idc = mid + 1
                    ids[name] = mid

                self.member_ids = ids

            from ._builder import Builder
            self.v0_machine, self.v2_machine, self.keyless, self.version_support = Builder.build_machines(self.datatype)

            if self.version_support.SupportsBasic & self.version_support:
                self.v0_keyresult: KeyScanner = self.v0_machine.key_scan()
                if self.v0_keyresult.rtype != KeyScanResult.PossiblyInfinite and self.v0_keyresult.size <= 16:
                    self.v0_key_max_size = self.v0_keyresult.size
                else:
                    self.v0_key_max_size = 17  # or bigger ;)

            if self.version_support.SupportsV2 & self.version_support:
                self.v2_keyresult: KeyScanner = self.v2_machine.key_scan()
                if self.v2_keyresult.rtype != KeyScanResult.PossiblyInfinite and self.v2_keyresult.size <= 16:
                    self.v2_key_max_size = self.v2_keyresult.size
                else:
                    self.v2_key_max_size = 17  # or bigger ;)

    def serialize(self, object, use_version_2: bool = None, buffer=None, endianness=None) -> bytes:
        if not self._populated:
            self.populate()

        if self.version_support.SupportsBasic & self.version_support:
            use_version_2 = False if use_version_2 is None else use_version_2
        else:
            # version 0 not supported
            if use_version_2 is not None and not use_version_2:
                raise Exception("Cannot encode this type with version 0, contains xcdrv2-type structures")
            use_version_2 = True

        ibuffer = buffer or self.buffer
        ibuffer.seek(0)
        ibuffer.zero_out()
        ibuffer.set_align_offset(0)
        ibuffer.set_endianness(endianness or Endianness.native())
        ibuffer._align_max = 4 if use_version_2 else 8

        if ibuffer.endianness == Endianness.Big:
            ibuffer.write('b', 1, 0)
            ibuffer.write('b', 1, 0 | (self.xcdrv2_head if use_version_2 else 0))
            ibuffer.write('b', 1, 0)
            ibuffer.write('b', 1, 0)
        else:
            ibuffer.write('b', 1, 0)
            ibuffer.write('b', 1, 1 | (self.xcdrv2_head if use_version_2 else 0))
            ibuffer.write('b', 1, 0)
            ibuffer.write('b', 1, 0)

        ibuffer.set_align_offset(4)

        if use_version_2:
            self.v2_machine.serialize(ibuffer, object)
        else:
            self.v0_machine.serialize(ibuffer, object)

        return ibuffer.asbytes()

    def deserialize(self, data, has_header=True, use_version_2: bool = None) -> object:
        if not self._populated:
            self.populate()

        if has_header and use_version_2 is not None:
            raise Exception("Considered programmer error to set a version of xcdr to use if a header is present in the data.")
        elif not has_header and self.version_support.SupportsBasic & self.version_support:
            use_version_2 = False if use_version_2 is None else use_version_2
        else:
            # version 0 not supported
            if use_version_2 is not None and not use_version_2:
                raise Exception("Cannot encode this type with version 0, contains xcdrv2-type structures")
            use_version_2 = True

        buffer = Buffer(data, align_offset=4 if has_header else 0) if not isinstance(data, Buffer) else data

        if has_header and buffer.tell() == 0:
            buffer.read('b', 1)
            v = buffer.read('b', 1)
            if (v & 1) > 0:
                buffer.set_endianness(Endianness.Little)
            else:
                buffer.set_endianness(Endianness.Big)
            buffer.read('b', 1)
            buffer.read('b', 1)
            if v > 1:
                buffer._align_max = 4
                machine = self.v2_machine
            else:
                buffer._align_max = 8
                machine = self.v0_machine
        else:
            if use_version_2:
                buffer._align_max = 4
                machine = self.v2_machine
            else:
                buffer._align_max = 8
                machine = self.v0_machine

        return machine.deserialize(buffer)

    def key(self, object, use_version_2: bool = None) -> bytes:
        if not self._populated:
            self.populate()

        if self.version_support.SupportsBasic & self.version_support:
            use_version_2 = False if use_version_2 is None else use_version_2
        else:
            # version 0 not supported
            if use_version_2 is not None and not use_version_2:
                raise Exception("Cannot encode this type with version 0, contains xcdrv2-type structures")
            use_version_2 = True

        if self.keyless:
            return b''

        self.buffer.seek(0)
        self.buffer.zero_out()
        self.buffer.set_align_offset(0)
        self.buffer.set_endianness(Endianness.Big)
        self.buffer._align_max = 4 if use_version_2 else 8

        if use_version_2:
            self.v2_machine.serialize(self.buffer, object, for_key=True)
        else:
            self.v0_machine.serialize(self.buffer, object, for_key=True)

        return self.buffer.asbytes()

    def keyhash(self, object, use_version_2: bool = None) -> bytes:
        if not self._populated:
            self.populate()

        if self.version_support.SupportsBasic & self.version_support:
            use_version_2 = False if use_version_2 is None else use_version_2
        else:
            # version 0 not supported
            if use_version_2 is not None and not use_version_2:
                raise Exception("Cannot encode this type with version 0, contains xcdrv2-type structures")
            use_version_2 = True

        if use_version_2:
            if self.v2_key_max_size <= 16:
                return self.key(object, True).ljust(16, b'\0')
        else:
            if self.v0_key_max_size <= 16:
                return self.key(object, False).ljust(16, b'\0')

        m = md5()
        m.update(self.key(object, use_version_2))
        return m.digest()

    def key_scan(self, use_version_2: bool = None):
        if self.re_entrancy_protection:
            # If we get here then there is a recursion in the type
            # This always means the keysize can be infinite
            return KeyScanner.infinity()

        if not self._populated:
            self.populate()

        if self.version_support.SupportsBasic & self.version_support:
            use_version_2 = False if use_version_2 is None else use_version_2
        else:
            # version 0 not supported
            if use_version_2 is not None and not use_version_2:
                raise Exception("Cannot encode this type with version 0, contains xcdrv2-type structures")
            use_version_2 = True

        self.re_entrancy_protection = True

        if use_version_2:
            scan = self.v2_machine.key_scan()
        else:
            scan = self.v0_machine.key_scan()

        self.re_entrancy_protection = False

        return scan

    def get_member_id(self, member: str) -> int:
        return self.member_ids.get(member, -1) if self.member_ids else -1

    def fill_type_data(self) -> None:
        if not self._populated:
            self.populate()

        if self._xt_data[0] is None:
            from ._xt_builder import XTBuilder
            self._xt_data = XTBuilder.process_type(self.datatype)
            self._xt_bytedata = (
                self._xt_data[0].serialize(endianness=Endianness.Little, use_version_2=True)[4:],
                self._xt_data[1].serialize(endianness=Endianness.Little, use_version_2=True)[4:]
            )

    def get_type_info(self) -> 'TypeInformation':
        if self._xt_data[0] is None:
            self.fill_type_data()
        return self._xt_data[0]

    def get_type_mapping(self) -> 'TypeMapping':
        if self._xt_data[0] is None:
            self.fill_type_data()
        return self._xt_data[1]

    def get_type_id(self) -> 'TypeIdentifier':
        if self._xt_data[0] is None:
            self.fill_type_data()
        return self._xt_data[0].complete.typeid_with_size.type_id


class IdlMeta(type):
    __idl__: ClassVar['IDL']
    __idl_typename__: ClassVar[str]
    __idl_annotations__: ClassVar[Dict[str, Any]]
    __idl_field_annotations__: ClassVar[Dict[str, Dict[str, Any]]]

    @classmethod
    def __prepare__(metacls, __name: str, __bases: Tuple[type, ...], **kwds: Any) -> Mapping[str, Any]:
        typename = None
        if "typename" in kwds:
            typename = kwds["typename"]
            del kwds["typename"]

        namespace = super().__prepare__(__name, __bases, **kwds)
        namespace["__annotations__"] = {}
        namespace["__idl_annotations__"] = {}
        namespace["__idl_field_annotations__"] = {}
        if typename:
            namespace["__idl_typename__"] = typename

        IDLNamespaceScope.enter(namespace)
        return namespace

    def __new__(metacls, name, bases, namespace, **kwds):
        IDLNamespaceScope.exit()
        new_cls = super().__new__(metacls, name, bases, dict(**namespace))

        if "__idl_typename__" not in namespace:
            new_cls.__idl_typename__ = name

        new_cls.__idl__ = IDL(new_cls)
        return new_cls

    def __repr__(cls):
        # Note, this is the _class_ repr
        if cls.__name__ == "IdlStruct":
            return "IdlStruct"
        return f"{cls.__name__}(IdlStruct, idl_typename='{cls.__idl_typename__}')"


def _union_default_finder(_type, cases):
    if isclass(_type) and issubclass(_type, Enum):
        # non-optimal for sure, but should be used rarely
        used = set(e.value for e in _type)
        i = 0
        while i in used:
            i += 1
        # TODO: bit bound check?
        return i

    if _type == bool:
        if True not in cases:
            return True
        elif False not in cases:
            return False
        raise TypeError("No space in discriminated union for default value.")

    val, inc, end = {
        types.int8: (-1, -1, -128),
        types.int16: (-1, -1, -32768),
        types.int32: (-1, -1, -2147483648),
        types.int64: (-1, -1, -9223372036854775808),
        int: (-1, -1, -9223372036854775808),
        types.uint8: (0, 1, 255),
        types.uint16: (0, 1, 65535),
        types.uint32: (0, 1, 4294967295),
        types.uint64: (0, 1, 18446744073709551615),
        types.char: (0, 1, 127)
    }.get(_type, (None, None, None))

    if val is None:
        raise TypeError(f"{_type} is not a valid discriminator type for an IdlUnion.")

    while True:
        if val not in cases:
            if _type == types.char:
                return chr(val)
            return val
        if val == end:
            raise TypeError("No space in discriminated union for default value.")
        val += inc


class IdlUnionMeta(IdlMeta):
    __idl_discriminator__: Any
    __idl_discriminator_is_key__: bool
    __idl_cases__: Dict[Any, Any]
    __idl_default__: Optional[Tuple[Any, Any]]

    @classmethod
    def __prepare__(metacls, __name: str, __bases: Tuple[type, ...], **kwds: Any) -> Mapping[str, Any]:
        if not len(__bases):
            return super().__prepare__(__name, __bases, **kwds)

        discriminator_is_key = False

        if "discriminator" not in kwds:
            raise TypeError("Union class needs a 'discriminator=type'")

        if "discriminator_is_key" in kwds:
            discriminator_is_key = kwds["discriminator_is_key"]
            del kwds["discriminator_is_key"]

        discriminator = kwds["discriminator"]
        del kwds["discriminator"]

        namespace = cast(Dict[str, Any], super().__prepare__(__name, __bases, **kwds))
        namespace["__idl_discriminator__"] = discriminator
        namespace["__idl_discriminator_is_key__"] = discriminator_is_key
        namespace["__annotations__"] = {}
        namespace["__idl_annotations__"] = {}
        namespace["__idl_field_annotations__"] = {}
        IDLNamespaceScope.enter(namespace)
        return namespace

    def __new__(metacls, name, bases, namespace, **kwds):
        IDLNamespaceScope.exit()
        new_cls = super().__new__(metacls, name, bases, dict(**namespace))
        if not len(bases):
            return new_cls

        cases = {}
        default = None
        names = set()

        # Use RAW annotations here because the type strings can maybe not be resolved yet
        for name, _type in new_cls.__annotations__.items():
            if get_origin(_type) != Annotated and len(get_args(_type)) != 2:
                raise TypeError(f"Fields of a union need to be case or default, '{name}: {_type}' is not.")

            _type = get_args(_type)[1]
            if isinstance(_type, types.case):
                for label in _type.labels:
                    if label in cases:
                        raise TypeError(f"Discriminator values must uniquely define a case, "
                                        f"but the case {label} occurred multiple times.")
                    cases[label] = (name, _type.subtype)
                    names.add(name)
            elif isinstance(_type, types.default):
                if default:
                    raise TypeError("A discriminated union can only have one default.")
                default = (name, _type.subtype)
                names.add(name)
            else:
                raise TypeError(f"Fields of a union need to be case or default, '{name}: {_type}' is not.")

        new_cls.__idl_cases__ = cases
        new_cls.__idl_default__ = default
        new_cls.__idl_names__ = names
        new_cls.__idl_discriminator__ = namespace['__idl_discriminator__']
        new_cls.__idl_default_discriminator__ = _union_default_finder(namespace['__idl_discriminator__'], cases)

        return new_cls

    def __repr__(cls):
        # Note, this is the _class_ repr
        if cls.__name__ == "IdlUnion":
            return "IdlUnion"
        return f"{cls.__name__}(IdlUnion, idl_typename='{cls.__idl_typename__}')"


class IdlBitmaskMeta(type):
    __idl_typename__: ClassVar[str]
    __idl_annotations__: ClassVar[Dict[str, Any]]
    __idl_field_annotations__: ClassVar[Dict[str, Dict[str, Any]]]
    __idl_bits__: ClassVar[Dict[int, str]]
    __idl_positions__: ClassVar[Dict[str, int]]
    __idl_highest_position__: ClassVar[int]

    @classmethod
    def __prepare__(metacls, __name: str, __bases: Tuple[type, ...], **kwds: Any) -> Mapping[str, Any]:
        typename = None
        if "typename" in kwds:
            typename = kwds["typename"]
            del kwds["typename"]

        namespace = super().__prepare__(__name, __bases, **kwds)
        namespace["__annotations__"] = {}
        namespace["__idl_annotations__"] = {}
        namespace["__idl_field_annotations__"] = {}
        IDLNamespaceScope.enter(namespace)

        if typename:
            namespace["__idl_typename__"] = typename

        return namespace

    def __new__(metacls, name, bases, namespace, **kwds):
        IDLNamespaceScope.exit()
        new_cls = super().__new__(metacls, name, bases, dict(**namespace))

        if "__idl_typename__" not in namespace:
            new_cls.__idl_typename__ = name

        idl_bits = {}
        idl_positions = {}
        position = 0
        idl_highest_position = None

        if "bit_bound" not in new_cls.__idl_annotations__:
            new_cls.__idl_annotations__["bit_bound"] = 32

        for name, type in new_cls.__annotations__.items():
            if get_origin(type) == ClassVar:
                continue

            if type != bool:
                raise TypeError("Bitmask type contains non-bools.")

            if name in new_cls.__idl_field_annotations__:
                if 'position' in new_cls.__idl_field_annotations__[name]:
                    position = new_cls.__idl_field_annotations__[name]["position"]

            if position < 0 or position >= new_cls.__idl_annotations__["bit_bound"]:
                raise TypeError(
                    f"Position {position} for {name} is outside of the bit boundary"
                    f"[0, {new_cls.__idl_annotations__['bit_bound']})."
                )

            idl_bit = 2 ** position
            if idl_bit in idl_bits:
                raise TypeError(f"Duplicate bit for position {position}.")

            idl_bits[idl_bit] = name
            idl_positions[name] = position
            idl_highest_position = max(idl_highest_position, position) if idl_highest_position is not None else position
            position += 1

        new_cls.__idl_bits__ = idl_bits
        new_cls.__idl_highest_position__ = idl_highest_position
        new_cls.__idl_positions__ = idl_positions

        return new_cls

    def __repr__(cls):
        # Note, this is the _class_ repr
        if cls.__name__ == "IdlBitmask":
            return "IdlBitmask"
        return f"{cls.__name__}(IdlBitmask, idl_typename='{cls.__idl_typename__}')"


class IdlEnumMeta(EnumMeta):
    __idl_typename__: ClassVar[str]
    __idl_annotations__: ClassVar[Dict[str, Any]]
    __idl_field_annotations__: ClassVar[Dict[str, Dict[str, Any]]]
    __idl_enum_default_value__: ClassVar[Optional['IdlEnum']]

    @classmethod
    def __prepare__(metacls, __name: str, __bases: Tuple[type, ...], **kwds: Any) -> Mapping[str, Any]:
        typename = None
        if "typename" in kwds:
            typename = kwds["typename"]
            del kwds["typename"]

        default_ = None
        if "default" in kwds:
            default_ = kwds["default"]
            del kwds["default"]

        namespace: Dict[str, Any] = super().__prepare__(__name, __bases, **kwds)

        if typename:
            namespace["__idl_typename__"] = typename

        namespace["__idl_annotations__"] = {}
        namespace["__idl_field_annotations__"] = {}
        namespace["__idl_enum_default_value__"] = default_
        IDLNamespaceScope.enter(namespace)

        return namespace

    def __new__(metacls, name, bases, namespace, **kwds):
        IDLNamespaceScope.exit()
        new_cls = super().__new__(metacls, name, bases, namespace)

        if "__idl_typename__" not in namespace:
            new_cls.__idl_typename__ = name
        else:
            new_cls.__idl_typename__ = namespace["__idl_typename__"]

        if namespace.get("__idl_enum_default_value__"):
            new_cls.__idl_enum_default_value__ = new_cls[namespace.get("__idl_enum_default_value__")]

        return new_cls

    _default = object()

    def __call__(cls, value=_default, *args, **kwargs):
        if value is IdlEnumMeta._default:
            return cls.__idl_enum_default_value__
        return super().__call__(value, *args, **kwargs)

    def __repr__(cls):
        # Note, this is the _class_ repr
        if cls.__name__ == "IdlEnum":
            return "IdlEnum"
        return f"{cls.__name__}(IdlEnum, idl_typename='{cls.__idl_typename__}')"
