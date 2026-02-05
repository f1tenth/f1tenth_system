"""
 * Copyright(c) 2021 ZettaScale Technology and others
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v. 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
 * v. 1.0 which is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
"""

import typing as _typing
from . import _type_helper as _th


def _type_repr(obj):
    """Avoid printing <class 'int'>"""
    if type(obj) == str:
        return obj
    if isinstance(obj, type):
        if obj.__module__ == 'builtins':
            return obj.__qualname__
        return f'{obj.__module__}.{obj.__qualname__}'
    if obj is ...:
        return '...'
    if _th.get_origin(obj) == _th.Annotated:
        return _type_repr(_th.get_args(obj)[1])
    return repr(obj)


class array:
    @classmethod
    def __class_getitem__(cls, tup):
        if type(tup) != tuple:
            tup = (tup,)

        if len(tup) != 2 or type(tup[1]) != int:
            raise TypeError("An array takes two arguments: an element type and a constant length.")
        return _th.Annotated[_typing.Sequence[tup[0]], cls(*tup)]

    def __init__(self, subtype: type, length: int):
        self.subtype: type = subtype
        self.length: int = length

    def __repr__(self) -> str:
        return f"array[{_type_repr(self.subtype)}, {self.length}]"

    def __eq__(self, o: object) -> bool:
        return isinstance(o, array) and o.subtype == self.subtype and o.length == self.length

    def __hash__(self) -> int:
        return (241813571056069 * self.length) ^ hash(self.subtype)

    __str__ = __repr__


class sequence:
    @classmethod
    def __class_getitem__(cls, tup):
        if type(tup) != tuple:
            tup = (tup,)

        if len(tup) not in [1, 2] or (len(tup) == 2 and type(tup[1]) != int):
            raise TypeError("A sequence takes a subtype and an optional maximum length.")
        if len(tup) > 1 and (tup[1] <= 0 or tup[1] > 65535):
            return TypeError("Sequence max length should be between 0 and 65536.")
        return _th.Annotated[_typing.Sequence[tup[0]], cls(*tup)]

    def __init__(self, subtype: type, max_length: _typing.Optional[int] = None) -> None:
        self.subtype: type = subtype
        self.max_length: _typing.Optional[int] = max_length

    def __repr__(self) -> str:
        if self.max_length:
            return f"sequence[{_type_repr(self.subtype)}, {self.max_length}]"
        else:
            return f"sequence[{_type_repr(self.subtype)}]"

    def __eq__(self, o: object) -> bool:
        return isinstance(o, sequence) and o.subtype == self.subtype and o.max_length == self.max_length

    def __hash__(self) -> int:
        if self.max_length:
            return (406820761152607 * self.max_length) ^ hash(self.subtype)
        else:
            return 406820761152609 ^ hash(self.subtype)

    __str__ = __repr__


class typedef:
    @classmethod
    def __class_getitem__(cls, tup):
        if type(tup) != tuple:
            tup = (tup,)

        if len(tup) != 2:
            raise TypeError("A typedef takes a name and real type as argument.")

        rtype = tup[1]
        while _th.get_origin(rtype) == _th.Annotated:
            rtype = _th.get_args(rtype)[0]
        return _th.Annotated[rtype, cls(*tup)]

    def __init__(self, name: str, subtype: type) -> None:
        self.name: str = name
        self.subtype: type = subtype

    def __repr__(self) -> str:
        return f"typedef[{self.name}, {_type_repr(self.subtype)}]"

    def __eq__(self, o: object) -> bool:
        return (isinstance(o, typedef) and o.subtype == self.subtype)

    def __hash__(self) -> int:
        return hash(self.name) ^ hash(self.subtype)

    def __call__(self, *args: _typing.Any, **kwds: _typing.Any) -> _typing.Any:
        return self.subtype(*args, **kwds)

    @property
    def __idl_typename__(self):
        return self.name

    __str__ = __repr__


class bounded_str:
    @classmethod
    def __class_getitem__(cls, tup):
        if type(tup) != tuple:
            tup = (tup,)

        if len(tup) != 1 or type(tup[0]) != int:
            raise TypeError("A bounded str takes one argument, a maximum length.")
        if tup[0] <= 0:
            return TypeError("Bound string max length should be bigger than 0.")
        return _th.Annotated[str, cls(*tup)]

    def __init__(self, max_length: int) -> None:
        self.max_length: int = max_length

    def __repr__(self) -> str:
        return f"bounded_str[{self.max_length}]"

    def __eq__(self, o: object) -> bool:
        return isinstance(o, bounded_str) and o.max_length == self.max_length

    def __hash__(self) -> int:
        return 277146416319491 * self.max_length

    __str__ = __repr__


class ValidUnionHolder:
    pass


class case(ValidUnionHolder):
    @classmethod
    def __class_getitem__(cls, tup):
        if type(tup) != tuple:
            tup = (tup,)

        if len(tup) != 2:
            raise TypeError("A case takes two arguments: the discriminator value(s) and the type.")
        return _th.Annotated[_typing.Optional[tup[1]], cls(*tup)]

    def __init__(self, discriminator_value, subtype: type) -> None:
        self.labels: _typing.List[int] = discriminator_value if type(discriminator_value) == list else [discriminator_value]
        self.subtype: type = subtype

    def __repr__(self) -> str:
        return f"case[{self.labels}, {_type_repr(self.subtype)}]"

    def __eq__(self, o: object) -> bool:
        return isinstance(o, case) and o.subtype == self.subtype and o.labels == self.labels

    def __hash__(self) -> int:
        return hash(self.subtype)

    __str__ = __repr__


class default(ValidUnionHolder):
    @classmethod
    def __class_getitem__(cls, tup):
        if type(tup) != tuple:
            tup = (tup,)

        if len(tup) != 1:
            raise TypeError("A default takes one arguments: the type.")
        return _th.Annotated[_typing.Optional[tup[0]], cls(*tup)]

    def __init__(self, subtype: type) -> None:
        self.subtype: type = subtype

    def __repr__(self) -> str:
        return f"default[{_type_repr(self.subtype)}]"

    def __eq__(self, o: object) -> bool:
        return isinstance(o, default) and o.subtype == self.subtype

    def __hash__(self) -> int:
        return hash(self.subtype)

    __str__ = __repr__


char = _th.Annotated[str, "char"]
wchar = _th.Annotated[int, "wchar"]
int8 = _th.Annotated[int, "int8"]
int16 = _th.Annotated[int, "int16"]
int32 = _th.Annotated[int, "int32"]
int64 = _th.Annotated[int, "int64"]
uint8 = _th.Annotated[int, "uint8"]
uint16 = _th.Annotated[int, "uint16"]
uint32 = _th.Annotated[int, "uint32"]
uint64 = _th.Annotated[int, "uint64"]
float32 = _th.Annotated[float, "float32"]
float64 = _th.Annotated[float, "float64"]
NoneType = type(None)

_type_code_align_size_default_mapping = {
    int8: ('b', 1, 1, 0),
    int16: ('h', 2, 2, 0),
    int32: ('i', 4, 4, 0),
    int64: ('q', 8, 8, 0),
    uint8: ('B', 1, 1, 0),
    uint16: ('H', 2, 2, 0),
    uint32: ('I', 4, 4, 0),
    uint64: ('Q', 8, 8, 0),
    float32: ('f', 4, 4, 0.0),
    float64: ('d', 8, 8, 0.0),
    wchar: ('h', 2, 2, 0),
    int: ('q', 8, 8, 0),
    bool: ('?', 1, 1, False),
    float: ('d', 8, 8, 0.0),
}
