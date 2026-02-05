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

from mypy.plugin import Plugin, AnalyzeTypeContext
from mypy.types import RawExpressionType, AnyType, TypeOfAny
from mypy.typeanal import make_optional_type


class IdlMyPyPlugin(Plugin):
    def get_type_analyze_hook(self, fullname: str):
        if fullname.startswith("pycdr2.types"):
            if fullname == "pycdr2.types.array":
                return strip_length_array_type
            elif fullname == "pycdr2.types.sequence":
                return strip_length_sequence_type
            elif fullname == "pycdr2.types.typedef":
                return resolve_type_def
            elif fullname == "pycdr2.types.bounded_str":
                return bounded_str_to_str
            elif fullname == "pycdr2.types.default":
                return union_default_type
            elif fullname == "pycdr2.types.case":
                return union_case_type
        return None

    def get_class_decorator_hook(self, fullname: str):
        from mypy.plugins import dataclasses

        if fullname in ["pycdr2.IdlStruct", "pycdr2.IdlUnion"]:
            return dataclasses.dataclass_class_maker_callback


def plugin(version: str):
    return IdlMyPyPlugin


def strip_length_array_type(ctx: AnalyzeTypeContext):
    if len(ctx.type.args) == 0:
        # ref of type itself
        return AnyType(TypeOfAny.special_form)

    if len(ctx.type.args) != 2 or not isinstance(ctx.type.args[1], RawExpressionType) or \
            ctx.type.args[1].base_type_name != "builtins.int":
        ctx.api.fail("pycdr2.types.array requires two arguments, a subtype and a fixed size.", ctx.context)
        return AnyType(TypeOfAny.from_error)
    return ctx.api.named_type('typing.Sequence', [ctx.api.analyze_type(ctx.type.args[0])])


def resolve_type_def(ctx: AnalyzeTypeContext):
    if len(ctx.type.args) == 0:
        # ref of type itself
        return AnyType(TypeOfAny.special_form)

    if len(ctx.type.args) != 2:
        ctx.api.fail("pycdr2.types.typedef requires a name and one type argument.", ctx.context)
        return AnyType(TypeOfAny.from_error)
    return ctx.api.analyze_type(ctx.type.args[1])


def strip_length_sequence_type(ctx: AnalyzeTypeContext):
    if len(ctx.type.args) == 0:
        # ref of type itself
        return AnyType(TypeOfAny.special_form)

    if len(ctx.type.args) not in [1, 2]:
        ctx.api.fail("pycdr2.types.sequence requires a subtype and an optional max size.", ctx.context)
        return AnyType(TypeOfAny.from_error)
    elif len(ctx.type.args) == 2 and not isinstance(ctx.type.args[1], RawExpressionType):
        ctx.api.fail("pycdr2.types.sequence max size should be an integer.", ctx.context)
        return AnyType(TypeOfAny.from_error)
    elif len(ctx.type.args) == 2 and ctx.type.args[1].base_type_name != "builtins.int":
        ctx.api.fail("pycdr2.types.sequence max size should be an integer.", ctx.context)
        return AnyType(TypeOfAny.from_error)

    return ctx.api.named_type('typing.Sequence', [ctx.api.analyze_type(ctx.type.args[0])])


def bounded_str_to_str(ctx: AnalyzeTypeContext):
    if len(ctx.type.args) == 0:
        # ref of type itself
        return AnyType(TypeOfAny.special_form)

    if len(ctx.type.args) != 1 or not isinstance(ctx.type.args[0], RawExpressionType) or \
            ctx.type.args[0].base_type_name != "builtins.int":
        ctx.api.fail("pycdr2.types.bound_str requires one argument, a fixed size.", ctx.context)
        return AnyType(TypeOfAny.from_error)
    return ctx.api.named_type('builtins.str', [])


def union_default_type(ctx: AnalyzeTypeContext):
    if len(ctx.type.args) == 0:
        # ref of type itself
        return AnyType(TypeOfAny.special_form)

    if len(ctx.type.args) != 1:
        ctx.api.fail("pycdr2.types.default requires one argument, a type.", ctx.context)
        return AnyType(TypeOfAny.from_error)
    return make_optional_type(ctx.api.analyze_type(ctx.type.args[0]))


def union_case_type(ctx: AnalyzeTypeContext):
    if len(ctx.type.args) == 0:
        # ref of type itself
        return AnyType(TypeOfAny.special_form)

    if len(ctx.type.args) != 2:
        ctx.api.fail("pycdr2.types.case requires two arguments, a discriminator label and a type.", ctx.context)
        return AnyType(TypeOfAny.from_error)
    return make_optional_type(ctx.api.analyze_type(ctx.type.args[1]))
