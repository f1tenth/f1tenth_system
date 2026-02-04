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

from typing import Any, Dict, Union, ForwardRef
from importlib import import_module

from ._type_helper import Annotated, get_origin, get_args
from .types import array, sequence, typedef, case, default, NoneType


class WrapOpt:
    inner: type
    def __init__(self, inner) -> None:
        self.inner = inner

def _strip_unextended_type(module, _type: Any) -> Any:
    if type(_type) == str:
        # Import
        try:
            if '.' in _type:
                # Fully qualified name
                rname, rmodule = _type[::-1].split(".", 1)
                typename, modulename = rname[::-1], rmodule[::-1]
            else:
                # Not fully qualified -> must be same module
                typename, modulename = (_type, module)

            pymodule = import_module(modulename)
            return _strip_unextended_type(modulename, getattr(pymodule, typename))
        except (ImportError, AttributeError):
            raise TypeError(f"Type {_type} as used in {module} cannot be resolved.")
    elif isinstance(_type, (array, sequence, typedef, case, default)):
        _type.subtype = _strip_unextended_type(module, _type.subtype)
        return _type
    if isinstance(_type, ForwardRef):
        return _strip_unextended_type(module, _type.__forward_arg__)

    ori = get_origin(_type)
    if ori is None:
        return _type
    args = get_args(_type)

    if args is None or len(args) == 0:
        return _type
    if ori == Union and NoneType in args and len(args) == 2:
        # optionals
        if args[0] == NoneType:
            return WrapOpt(_strip_unextended_type(module, args[1]))
        else:
            return WrapOpt(_strip_unextended_type(module, args[0]))
    if ori == ForwardRef:
        return _strip_unextended_type(module, args[0])
    if ori == Annotated and len(args) == 2 and type(args[1]) == str:
        # primitive type
        return _type
    if ori == Annotated and len(args) == 2:
        return _strip_unextended_type(module, args[1])

    if len(args) == 1:
        return ori[_strip_unextended_type(module, args[0])]
    elif len(args) == 2:
        return ori[_strip_unextended_type(module, args[0]), _strip_unextended_type(module, args[1])]
    elif len(args) == 3:
        return ori[
            _strip_unextended_type(module, args[0]),
            _strip_unextended_type(module, args[1]),
            _strip_unextended_type(module, args[2])
        ]

    return _type


def _make_extended_type_hints(cls: Any) -> Dict[str, Any]:
    hints = cls.__annotations__
    return {k: _strip_unextended_type(cls.__module__, v) for k, v in hints.items() if not k.startswith("__")}


def _normalize_idl_class(cls: Any) -> None:
    if hasattr(cls, "__idl_type_annotations__"):
        return
    if cls.__name__.startswith("Idl") and cls.__module__ == "pycdr2":
        return

    cls.__idl_type_annotations__ = _make_extended_type_hints(cls)

    if hasattr(cls, '__idl_discriminator__'):
        cls.__idl_discriminator__ = _strip_unextended_type(cls.__module__, cls.__idl_discriminator__)

    for k in cls.__idl_type_annotations__.keys():
        if k not in cls.__idl_field_annotations__:
            cls.__idl_field_annotations__[k] = {}


def get_extended_type_hints(cls: Any) -> Dict[str, Any]:
    if isinstance(cls, typedef):
        return {'_': _strip_unextended_type(cls.__module__, cls.subtype)}
    _normalize_idl_class(cls)
    return cls.__idl_type_annotations__


def get_idl_field_annotations(cls: Any) -> Dict[str, Dict[str, Any]]:
    return cls.__idl_field_annotations__


def get_idl_annotations(cls: Any) -> Dict[str, Any]:
    return cls.__idl_annotations__


__all__ = ["get_extended_type_hints", "get_idl_field_annotations", "get_idl_annotations"]
