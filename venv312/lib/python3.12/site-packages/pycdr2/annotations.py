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

from typing import Any, Union, Optional, Callable, List, Type, TypeVar
from . import IdlStruct, IdlUnion, IdlBitmask, IdlEnum
from ._main import IDLNamespaceScope


class AnnotationException(Exception):
    pass


T = TypeVar('T', bound=Union[Type[IdlStruct], Type[IdlUnion], Type[IdlBitmask], Type[IdlEnum]])
TIS = TypeVar('TIS', bound=Type[IdlStruct])
TIB = TypeVar('TIB', bound=Type[IdlBitmask])
TIBE = TypeVar('TIBE', Type[IdlEnum], Type[IdlBitmask])


def __annotate(cls: T, annotation: str, value: Any) -> None:
    cls.__idl_annotations__[annotation] = value


def __field_annotate(pfield: str, annotation: str, value: Any) -> None:
    if not IDLNamespaceScope.current:
        raise TypeError("Cannot annotate fields while not in class scope")

    if pfield not in IDLNamespaceScope.current["__annotations__"]:
        raise TypeError(f"Member {pfield} is not defined.")

    if pfield not in IDLNamespaceScope.current["__idl_field_annotations__"]:
        IDLNamespaceScope.current["__idl_field_annotations__"][pfield] = {}

    IDLNamespaceScope.current["__idl_field_annotations__"][pfield][annotation] = value


def default_literal(value: Any) -> Callable[[Any], None]:
    def apply_default_literal(apply_to: Any) -> None:
        __field_annotate(apply_to, "default_literal", value)
    return apply_default_literal


def key(apply_to: str) -> None:
    __field_annotate(apply_to, "key", True)


def position(apply_to: str, value: int) -> None:
    __field_annotate(apply_to, "position", value)


def member_id(apply_to: str, value: int) -> None:
    __field_annotate(apply_to, "id", value)


def member_hash_id(apply_to: str, value: Optional[str] = None) -> None:
    __field_annotate(apply_to, "hash_id", value)


def external(apply_to: str) -> None:
    __field_annotate(apply_to, "external", True)


def xcdrv2(cls: T) -> T:
    __annotate(cls, "xcdrv2", True)
    return cls


def cdrv0(cls: T) -> T:
    __annotate(cls, "xcdrv2", False)
    return cls


def nested(cls: T) -> T:
    __annotate(cls, "nested", True)
    return cls


def must_understand(str: Any) -> None:
    __field_annotate(str, "must_understand", True)


def autoid(autoid_type: str) -> Callable[[T], T]:
    if autoid_type not in ["hash", "sequential"]:
        raise AnnotationException("Autoid is either 'hash' or 'sequential'.")

    def autoid_inner(cls: T) -> T:
        __annotate(cls, "autoid", autoid_type)
        return cls

    return autoid_inner


def extensibility(extensibility_type: str) -> Callable[[T], T]:
    if extensibility_type not in ["final", "mutable", "appendable"]:
        raise AnnotationException("Extensibility is either 'final', 'appendable' or 'mutable'.")

    def extensibility_inner(cls: T) -> T:
        __annotate(cls, "extensibility", extensibility_type)
        return cls

    return extensibility_inner


def final(cls: T) -> T:
    return extensibility("final")(cls)


def appendable(cls: T) -> T:
    return extensibility("appendable")(cls)


def mutable(cls: T) -> T:
    return extensibility("mutable")(cls)


def keylist(list_of_keys: List[str]) -> Callable[[TIS], TIS]:
    def keylist_inner(cls: TIS) -> TIS:
        __annotate(cls, "keylist", list_of_keys)
        return cls

    return keylist_inner


def bit_bound(amount: int) -> Callable[[TIBE], TIBE]:
    if amount <= 0 or amount > 64:
        raise AnnotationException(f"{amount} is not a valid bit_bound, must be between 1 and 64 inclusive.")

    def bit_bound_inner(cls: TIBE) -> TIBE:
        __annotate(cls, "bit_bound", amount)

        if hasattr(cls, "__idl_highest_position__") and amount <= cls.__idl_highest_position__:
            raise AnnotationException(f"Position {cls.__idl_highest_position__} is outside of the bit boundary [0, {amount}).")
        return cls

    return bit_bound_inner


__all__ = [
    "default_literal", "key", "position", "member_id", "member_hash_id", "xcdrv2", "cdrv0",
    "nested", "must_understand", "autoid", "extensibility", "final", "appendable", "mutable",
    "keylist", "bit_bound"
]
