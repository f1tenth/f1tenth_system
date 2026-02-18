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

from inspect import isclass
from typing import List, Set, Tuple, Type, Optional, Union, Any, Dict
from collections import deque
from dataclasses import dataclass
from hashlib import md5

from ._typesupport.DDS import XTypes as xt
from . import types as pt
from . import annotations as annotate
from . import IdlStruct, IdlUnion, IdlBitmask, IdlEnum, make_idl_struct, make_idl_union, make_idl_enum, make_idl_bitmask
from ._support import Endianness
from ._type_helper import get_origin, get_args
from ._type_normalize import get_extended_type_hints, get_idl_annotations, get_idl_field_annotations, WrapOpt


uint32_max = 2 ** 32 - 1


def _is_optional(_type: Any) -> bool:
    return isinstance(_type, WrapOpt)


def kosaraju(graph: List[List[int]]) -> Tuple[List[int], Dict[int, int], List[Set[int]]]:
    # https://en.wikipedia.org/wiki/Kosaraju%27s_algorithm

    number_of_nodes = len(graph)

    # 1. For each vertex u of the graph, mark u as unvisited. Let L be empty.
    visited = [False] * number_of_nodes
    L = []
    reverse_graph: List[List[int]] = [[] for _ in range(number_of_nodes)]

    def visit(u):
        if not visited[u]:
            visited[u] = True
            for v in graph[u]:
                visit(v)
                reverse_graph[v] += [u]

            # we keep L in normal order instead of prepend
            #  and reverse iterate instead
            L.append(u)

    # 2. For each vertex u of the graph do visit(u)
    for u in range(number_of_nodes):
        visit(u)

    roots = [0] * number_of_nodes
    rootset: Set[int] = set()

    def assign(u, root):
        if visited[u]:
            visited[u] = False
            roots[u] = root
            rootset.add(root)
            for v in reverse_graph[u]:
                assign(v, root)

    # 3: For each element u of l in order, do assign(u, u)
    for u in reversed(L):
        assign(u, u)

    # Busywork: turn root node into component id + component index
    roots_to_components = {r: i for i, r in enumerate(sorted(rootset))}
    component_sizes = {r: 0 for r in range(len(rootset))}
    component_ids = [0] * number_of_nodes
    component_graph: List[Set[int]] = [set() for _ in range(len(rootset))]

    for i in range(number_of_nodes):
        component_id = roots_to_components[roots[i]]
        component_sizes[component_id] += 1
        component_ids[i] = component_id

    for i in range(number_of_nodes):
        c = component_ids[i]
        for u in graph[i]:
            if component_ids[u] != c:
                component_graph[c].add(component_ids[u])

    # component indexes - component sizes - component adjecency graph
    return component_ids, component_sizes, component_graph


class TypeHash:
    def __init__(
        self, *, cls: Type[Any],
        minimal_type_object: Union[xt.TypeObject, xt.MinimalTypeObject, pt.NoneType] = None,
        complete_type_object: Union[xt.TypeObject, xt.CompleteTypeObject, pt.NoneType] = None,
        minimal_type_identifier: Optional[xt.TypeIdentifier] = None,
        complete_type_identifier: Optional[xt.TypeIdentifier] = None,
        minimal_type_object_serialized: Optional[bytes] = None,
        complete_type_object_serialized: Optional[bytes] = None,
        minimal_hash: Optional[xt.TypeObjectHashId] = None,
        complete_hash: Optional[xt.TypeObjectHashId] = None
    ) -> None:
        self.cls: Type[Any] = cls

        if minimal_type_object is not None and complete_type_object is not None:
            self.minimal_type_object: xt.TypeObject = (
                xt.TypeObject(minimal=minimal_type_object) if isinstance(minimal_type_object, xt.MinimalTypeObject)
                else minimal_type_object
            )
            self.complete_type_object: xt.TypeObject = (
                xt.TypeObject(complete=complete_type_object) if isinstance(complete_type_object, xt.CompleteTypeObject)
                else complete_type_object
            )
            self.minimal_type_object_serialized: bytes = (
                minimal_type_object_serialized or self.minimal_type_object.serialize(endianness=Endianness.Little, use_version_2=True)[4:]
            )
            self.complete_type_object_serialized: bytes = (
                complete_type_object_serialized or self.complete_type_object.serialize(endianness=Endianness.Little, use_version_2=True)[4:]
            )

            self.minimal_hash: xt.TypeObjectHashId = minimal_hash or xt.TypeObjectHashId(
                discriminator=xt.EK_MINIMAL,
                value=md5(self.minimal_type_object_serialized).digest()[:14]
            )

            self.complete_hash: xt.TypeObjectHashId = complete_hash or xt.TypeObjectHashId(
                discriminator=xt.EK_COMPLETE,
                value=md5(self.complete_type_object_serialized).digest()[:14]
            )

        self.minimal_type_identifier: xt.TypeIdentifier = minimal_type_identifier or xt.TypeIdentifier(
            discriminator=xt.EK_MINIMAL,
            value=self.minimal_hash.value
        )

        self.complete_type_identifier: xt.TypeIdentifier = complete_type_identifier or xt.TypeIdentifier(
            discriminator=xt.EK_COMPLETE,
            value=self.complete_hash.value
        )


class _StrongComponentResolver:
    def __init__(self, names, types, component_ids, component_size, component_id) -> None:
        self.cnames = [name for i, name in enumerate(names) if component_ids[i] == component_id]
        self.ctypes = [type for i, type in enumerate(types) if component_ids[i] == component_id]

        # 4.b If the Strongly Connected Component (SCC) has multiple types, then sort them
        #     using the lexicographic order of their fully qualified type name
        self.ctypes = [value for i, value in sorted(enumerate(self.ctypes), key=lambda x: self.cnames[x[0]])]
        self.cnames = list(sorted(self.cnames))

        # 4.b.i
        self.minimal_components = [
            xt.StronglyConnectedComponentId(
                sc_component_id=xt.TypeObjectHashId(
                    discriminator=xt.EK_MINIMAL,
                    value=b"\0" * 14
                ),
                scc_length=component_size,
                scc_index=i + 1
            )
            for i, name in enumerate(self.cnames)
        ]

        self.complete_components = [
            xt.StronglyConnectedComponentId(
                sc_component_id=xt.TypeObjectHashId(
                    discriminator=xt.EK_COMPLETE,
                    value=b"\0" * 14
                ),
                scc_length=component_size,
                scc_index=i + 1
            )
            for i, name in enumerate(self.cnames)
        ]

        self.minimal_typemap_by_name = {
            name: self.minimal_components[i]
            for i, name in enumerate(self.cnames)
        }
        self.complete_typemap_by_name = {
            name: self.complete_components[i]
            for i, name in enumerate(self.cnames)
        }

        self.minimal_typemap_by_type = {
            type: self.minimal_components[i]
            for i, type in enumerate(self.ctypes)
            if '.' not in self.cnames[i]
        }
        self.complete_typemap_by_type = {
            type: self.complete_components[i]
            for i, type in enumerate(self.ctypes)
            if '.' not in self.cnames[i]
        }

    def resolve(self, name, type):
        if name in self.minimal_typemap_by_name:
            return self.minimal_typemap_by_name.get(name), self.complete_typemap_by_name.get(name)
        if type in self.minimal_typemap_by_type:
            return self.minimal_typemap_by_type.get(type), self.complete_typemap_by_type.get(type)
        return None, None


class XTBuilder:
    _default_extensibility = "final"
    _type_hash_db: Dict[Union[type, Tuple[str, type]], TypeHash] = {}
    _current_sc_resolver: Optional[_StrongComponentResolver] = None

    @classmethod
    def process_type(cls, topic_type) -> Tuple[xt.TypeInformation, xt.TypeMapping]:
        _names, _types, _graph, no_dependency_types = cls.gather_types(topic_type)

        no_dependency_hashes = [cls._resolve_typehash('', _type) for _type in no_dependency_types]

        if not _graph:
            # Hurray our graph was fully resolved by no_deps types, which means our type is a no_dep type
            hash: TypeHash = cls._resolve_typehash('', topic_type)
            return xt.TypeInformation(
                minimal=xt.TypeIdentifierWithDependencies(
                    typeid_with_size=xt.TypeIdentifierWithSize(
                        type_id=hash.minimal_type_identifier,
                        typeobject_serialized_size=len(hash.minimal_type_object_serialized)
                    ),
                    dependent_typeid_count=0,
                    dependent_typeids=[]
                ),
                complete=xt.TypeIdentifierWithDependencies(
                    typeid_with_size=xt.TypeIdentifierWithSize(
                        type_id=hash.complete_type_identifier,
                        typeobject_serialized_size=len(hash.complete_type_object_serialized)
                    ),
                    dependent_typeid_count=0,
                    dependent_typeids=[]
                )
            ), xt.TypeMapping(
                identifier_object_pair_minimal=[
                    xt.TypeIdentifierTypeObjectPair(
                        type_identifier=hash.minimal_type_identifier,
                        type_object=hash.minimal_type_object
                    )
                ],
                identifier_object_pair_complete=[
                    xt.TypeIdentifierTypeObjectPair(
                        type_identifier=hash.complete_type_identifier,
                        type_object=hash.complete_type_object
                    )
                ],
                identifier_complete_minimal=[
                    xt.TypeIdentifierPair(
                        type_identifier1=hash.complete_type_identifier,
                        type_identifier2=hash.minimal_type_identifier
                    )
                ]
            )

        _type_component_ids, _component_sizes, _component_graph = kosaraju(_graph)

        # Top level type is at position 0 in component 0, guaranteed by gather_types
        assert _names[0] == topic_type.__idl_typename__.replace('.', '::')
        scc_typeids = cls._handle_component(_names, _types, _graph, _type_component_ids, _component_sizes, _component_graph, 0)

        # Construct xt.TypeInformation
        dependent_typeids: List[Tuple[xt.TypeIdentifierWithSize, xt.TypeIdentifierWithSize]] = [
            (
                xt.TypeIdentifierWithSize(
                    type_id=dhash.minimal_type_identifier,
                    typeobject_serialized_size=len(dhash.minimal_type_object_serialized)
                ),
                xt.TypeIdentifierWithSize(
                    type_id=dhash.complete_type_identifier,
                    typeobject_serialized_size=len(dhash.complete_type_object_serialized)
                ),
            )
            for dhash in no_dependency_hashes
        ]
        dependent_typeids += [
            (
                xt.TypeIdentifierWithSize(
                    type_id=dhash.minimal_type_identifier,
                    typeobject_serialized_size=len(dhash.minimal_type_object_serialized)
                ),
                xt.TypeIdentifierWithSize(
                    type_id=dhash.complete_type_identifier,
                    typeobject_serialized_size=len(dhash.complete_type_object_serialized)
                ),
            )
            for dhash in scc_typeids if dhash.cls != topic_type
        ]

        # TODO redo this
        full_list_typehashes: List[TypeHash] = []

        for (name, _type) in zip(_names, _types):
            if _type in cls._type_hash_db:
                full_list_typehashes.append(cls._type_hash_db[_type])
            elif '.' in name and (name, _type) in cls._type_hash_db:
                # member
                full_list_typehashes.append(cls._type_hash_db[(name, _type)])
            else:
                full_list_typehashes.append(cls._resolve_typehash(name, _type))

        for _type in no_dependency_types:
            full_list_typehashes.append(cls._resolve_typehash("", _type))

        hash = full_list_typehashes[0]
        minimal_seen = set()
        minimal_dependend_typeids = [
            d[0] for d in dependent_typeids
            if not (d[0].type_id in minimal_seen or minimal_seen.add(d[0].type_id))
        ]

        return xt.TypeInformation(
            minimal=xt.TypeIdentifierWithDependencies(
                typeid_with_size=xt.TypeIdentifierWithSize(
                    type_id=hash.minimal_type_identifier,
                    typeobject_serialized_size=len(hash.minimal_type_object_serialized)
                ),
                dependent_typeid_count=len(minimal_dependend_typeids),
                dependent_typeids=minimal_dependend_typeids
            ),
            complete=xt.TypeIdentifierWithDependencies(
                typeid_with_size=xt.TypeIdentifierWithSize(
                    type_id=hash.complete_type_identifier,
                    typeobject_serialized_size=len(hash.complete_type_object_serialized)
                ),
                dependent_typeid_count=len(dependent_typeids),
                dependent_typeids=[d[1] for d in dependent_typeids]
            )
        ), xt.TypeMapping(
            identifier_object_pair_minimal=[
                xt.TypeIdentifierTypeObjectPair(
                    type_identifier=dhash.minimal_type_identifier,
                    type_object=dhash.minimal_type_object
                )
                for dhash in full_list_typehashes
            ],
            identifier_object_pair_complete=[
                xt.TypeIdentifierTypeObjectPair(
                    type_identifier=dhash.complete_type_identifier,
                    type_object=dhash.complete_type_object
                )
                for dhash in full_list_typehashes
            ],
            identifier_complete_minimal=[
                xt.TypeIdentifierPair(
                    type_identifier1=dhash.complete_type_identifier,
                    type_identifier2=dhash.minimal_type_identifier
                )
                for dhash in full_list_typehashes
            ]
        )

    @classmethod
    def _deep_gather_type(cls, _type, deep=False):
        # TODO: maps are a problem...

        if isclass(_type) and issubclass(_type, (IdlStruct, IdlUnion, IdlEnum, IdlBitmask)):
            return _type, deep

        if isinstance(_type, pt.sequence) or isinstance(_type, pt.array):
            return cls._deep_gather_type(_type.subtype, False)

        if isinstance(_type, pt.typedef):
            return _type, True

        if isinstance(_type, pt.case) or isinstance(_type, pt.default):
            return cls._deep_gather_type(_type.subtype, False)

        if _is_optional(_type):
            return cls._deep_gather_type(_type.inner, False)

        return None, False

    @classmethod
    def gather_types(cls, _type):
        toscan = deque()
        toscan.append(_type)

        # Start with graph with single node no connections
        graph = {
            _type.__idl_typename__.replace('.', '::'): set()
        }
        graph_types = {
            _type.__idl_typename__.replace('.', '::'): _type
        }

        # 1. Let TypeDependencyDG(T) be the dependency digraph that contains only the types that are reachable from T
        while toscan:
            _ctype = toscan.pop()
            my_node_name = _ctype.__idl_typename__.replace('.', '::')

            if isclass(_ctype) and issubclass(_ctype, IdlUnion):
                # get_extended_type_hints will not inspect the discriminator, and that can be an enum
                discriminator_type = _ctype.__idl_discriminator__
                if isclass(discriminator_type) and issubclass(discriminator_type, IdlEnum):
                    scan_node_name = discriminator_type.__idl_typename__.replace('.', '::')
                    if scan_node_name not in graph:
                        graph[scan_node_name] = set()
                        graph_types[scan_node_name] = discriminator_type

                    graph[my_node_name].add(scan_node_name)

            for name, fieldtype in get_extended_type_hints(_ctype).items():
                m, deep = cls._deep_gather_type(fieldtype)
                plain = cls._impl_xt_is_plain(fieldtype)

                # m is None if primitive or a type with fields
                if m is not None:
                    if not plain and not isinstance(_ctype, pt.typedef):
                        scan_node_name = m.__idl_typename__.replace('.', '::')
                        depending_name = f"{my_node_name}.{name}"

                        if scan_node_name not in graph:
                            graph[scan_node_name] = set()
                            graph_types[scan_node_name] = m
                            toscan.append(m)

                        graph[depending_name] = set([scan_node_name])
                        graph[my_node_name].add(depending_name)
                        graph_types[depending_name] = fieldtype
                    else:
                        scan_node_name = m.__idl_typename__.replace('.', '::')
                        if scan_node_name not in graph:
                            graph[scan_node_name] = set()
                            graph_types[scan_node_name] = m
                            toscan.append(m)

                        graph[my_node_name].add(scan_node_name)

        # 2. Let ReducedDependencyDG(T) be the subdigraph of TypeDependencyDG(T) where all
        #    the vertices that have no outgoing edges are removed.
        no_dependency_types = []
        for name, edges in graph.copy().items():
            if not edges:
                del graph[name]
                no_dependency_types.append(graph_types[name])
                del graph_types[name]

        # Busywork, need graph made of indexes
        node_names = list(graph.keys())
        node_indexes = {name: i for i, name in enumerate(node_names)}
        node_graph = [[node_indexes[name] for name in graph[nname] if name in node_indexes] for nname in node_names]
        node_types = [graph_types[name] for name in node_names]

        return node_names, node_types, node_graph, no_dependency_types

    @classmethod
    def _resolve_typehash(cls, name: str, _type: Type[Any]) -> TypeHash:
        if _type in cls._type_hash_db:
            return cls._type_hash_db[_type]

        minimal_scc, complete_scc = (
            (None, None) if (cls._current_sc_resolver is None or name == "")
            else cls._current_sc_resolver.resolve(name, _type)
        )

        if minimal_scc is not None and complete_scc is not None:
            return TypeHash(
                cls=_type,
                minimal_type_identifier=xt.TypeIdentifier(sc_component_id=minimal_scc),
                complete_type_identifier=xt.TypeIdentifier(sc_component_id=complete_scc)
            )

        minimal_object = cls._xt_minimal_type_object(_type)
        complete_object = cls._xt_complete_type_object(_type)

        hash = TypeHash(
            cls=_type,
            minimal_type_object=minimal_object,
            complete_type_object=complete_object
        )
        cls._type_hash_db[_type] = hash
        return hash

    @dataclass
    @annotate.final
    class _TypeSeqWrap(IdlStruct):
        seq: pt.sequence[xt.TypeObject]

    @classmethod
    def _handle_component(cls, _names, _types, _graph, _type_component_ids, _component_sizes, _component_graph, component_id):
        # Because the component graph has no cycles depth first is guaranteed to be non-infinite

        scc_typeids = sum([
            cls._handle_component(
                _names, _types, _graph, _type_component_ids, _component_sizes, _component_graph, subcomponent_id
            )
            for subcomponent_id in _component_graph[component_id]
        ], [])

        if _component_sizes[component_id] == 1:
            # 4.a. If the Strongly Connected Component (SCC) has a single type, then use the
            # regular algorithm to compute its xt.TypeIdentifier
            # EXCEPTION: type depends on itself (struct Node {@optional Node n;};)
            for i in range(len(_types)):
                if _type_component_ids[i] == component_id:
                    if i not in _graph[i]:  # Catch the exception of self dependence
                        hash: TypeHash = cls._resolve_typehash(_names[i], _types[i])
                        scc_typeids.append(hash)
                        return scc_typeids
                    break

        cls._current_sc_resolver = _StrongComponentResolver(
            _names,
            _types,
            _type_component_ids,
            _component_sizes[component_id],
            component_id
        )

        minimal_type_objects = [None] * _component_sizes[component_id]
        complete_type_objects = [None] * _component_sizes[component_id]

        # 4.b.ii
        for i, (name, _type) in enumerate(zip(_names, _types)):
            if _type_component_ids[i] == component_id:
                minimal_type_objects[cls._current_sc_resolver.cnames.index(name)] = cls._xt_minimal_type_object(_type)
                complete_type_objects[cls._current_sc_resolver.cnames.index(name)] = cls._xt_complete_type_object(_type)

        # 4.c, 4.d
        # Note: the [4:] is to skip the XCDR header
        minimal_bytes = cls._TypeSeqWrap(seq=minimal_type_objects).serialize(endianness=Endianness.Little, use_version_2=True)[4:]
        complete_bytes = cls._TypeSeqWrap(seq=complete_type_objects).serialize(endianness=Endianness.Little, use_version_2=True)[4:]

        # 4.e
        minimal_hash = xt.TypeObjectHashId(
            discriminator=xt.EK_MINIMAL,
            value=md5(minimal_bytes).digest()[:14]
        )
        complete_hash = xt.TypeObjectHashId(
            discriminator=xt.EK_COMPLETE,
            value=md5(complete_bytes).digest()[:14]
        )

        # 4.f
        for i, (name, _type) in enumerate(zip(_names, _types)):
            if _type_component_ids[i] == component_id:
                if '.' in name:
                    key = (name, _type)
                else:
                    key = _type
                index = cls._current_sc_resolver.cnames.index(name)
                cls._type_hash_db[key] = TypeHash(
                    cls=_type,
                    minimal_type_object=minimal_type_objects[index],
                    complete_type_object=complete_type_objects[index],
                    minimal_type_identifier=xt.TypeIdentifier(
                        sc_component_id=xt.StronglyConnectedComponentId(
                            sc_component_id=minimal_hash,
                            scc_length=_component_sizes[component_id],
                            scc_index=index + 1
                        )
                    ),
                    complete_type_identifier=xt.TypeIdentifier(
                        sc_component_id=xt.StronglyConnectedComponentId(
                            sc_component_id=complete_hash,
                            scc_length=_component_sizes[component_id],
                            scc_index=index + 1
                        )
                    )
                )

        for comp in cls._current_sc_resolver.complete_components:
            comp.sc_component_id = complete_hash

        for comp in cls._current_sc_resolver.minimal_components:
            comp.sc_component_id = minimal_hash

        cls._current_sc_resolver = None

        # Index 0 is used to indicate the full StronglyConnectedComponent
        scc_typeids.append(TypeHash(
            cls=None,
            minimal_type_object=minimal_type_objects,
            minimal_type_object_serialized=minimal_bytes,
            minimal_hash=minimal_hash,
            minimal_type_identifier=xt.TypeIdentifier(
                sc_component_id=xt.StronglyConnectedComponentId(
                    sc_component_id=minimal_hash,
                    scc_length=_component_sizes[component_id],
                    scc_index=0
                )
            ),
            complete_type_object=complete_type_objects,
            complete_type_object_serialized=complete_bytes,
            complete_hash=complete_hash,
            complete_type_identifier=xt.TypeIdentifier(
                sc_component_id=xt.StronglyConnectedComponentId(
                    sc_component_id=complete_hash,
                    scc_length=_component_sizes[component_id],
                    scc_index=0
                )
            )
        ))
        return scc_typeids

    @classmethod
    def _impl_xt_is_fully_descriptive(cls, _type: type) -> bool:
        """Check if a type is fully descriptive, e.g. does not contain any type hashes"""
        if _type in pt._type_code_align_size_default_mapping:
            return True
        if _type in [str, pt.char]:
            return True
        if isinstance(_type, pt.bounded_str):
            return True
        if isinstance(_type, pt.array) or isinstance(_type, pt.sequence):
            return cls._impl_xt_is_fully_descriptive(_type.subtype)
        if get_origin(_type) == Dict:
            args = get_args(_type)
            return cls._impl_xt_is_fully_descriptive(args[0]) and cls._impl_xt_is_fully_descriptive(args[1])
        if _is_optional(_type):
            return cls._impl_xt_is_fully_descriptive(_type.inner)

        return False

    @classmethod
    def _impl_xt_is_plain(cls, _type: type) -> bool:
        """Check if a type is fully descriptive, e.g. does not contain any type hashes"""
        if _type in pt._type_code_align_size_default_mapping:
            return True
        if _type in [str, pt.char]:
            return True
        if isinstance(_type, pt.bounded_str):
            return True
        if isinstance(_type, pt.array):
            inner = _type
            while isinstance(inner, pt.array):
                inner = inner.subtype

            return cls._impl_xt_is_plain(inner)
        if isinstance(_type, pt.sequence):
            return cls._impl_xt_is_plain(_type.subtype)
        if get_origin(_type) == Dict:
            args = get_args(_type)
            return cls._impl_xt_is_plain(args[0]) and cls._impl_xt_is_plain(args[1])
        if isclass(_type) and issubclass(_type, (IdlStruct, IdlUnion, IdlEnum, IdlBitmask)):
            return True
        if isinstance(_type, pt.typedef):
            return True
        if isinstance(_type, (pt.case, pt.default)):
            return cls._impl_xt_is_plain(_type.subtype)
        if isinstance(_type, WrapOpt):
            return cls._impl_xt_is_plain(_type.inner)

        return False

    @classmethod
    def _xt_type_identifier(cls, name: str, entity: Any, minimal: bool) -> xt.TypeIdentifier:
        if entity is None or entity == object:
            return xt.TypeIdentifier(discriminator=xt.TK_NONE, value=None)

        if _is_optional(entity):
            # turned into annotation one level up
            entity = entity.inner

        if entity == pt.char:
            return xt.TypeIdentifier(discriminator=xt.TK_CHAR8, value=None)

        if entity in pt._type_code_align_size_default_mapping:
            return {
                bool: xt.TypeIdentifier(discriminator=xt.TK_BOOLEAN, value=None),
                float: xt.TypeIdentifier(discriminator=xt.TK_FLOAT64, value=None),
                int: xt.TypeIdentifier(discriminator=xt.TK_INT64, value=None),
                pt.int8: xt.TypeIdentifier(discriminator=xt.TK_CHAR8, value=None),
                pt.int16: xt.TypeIdentifier(discriminator=xt.TK_INT16, value=None),
                pt.int32: xt.TypeIdentifier(discriminator=xt.TK_INT32, value=None),
                pt.int64: xt.TypeIdentifier(discriminator=xt.TK_INT64, value=None),
                pt.uint8: xt.TypeIdentifier(discriminator=xt.TK_BYTE, value=None),
                pt.uint16: xt.TypeIdentifier(discriminator=xt.TK_UINT16, value=None),
                pt.uint32: xt.TypeIdentifier(discriminator=xt.TK_UINT32, value=None),
                pt.uint64: xt.TypeIdentifier(discriminator=xt.TK_UINT64, value=None),
                pt.char: xt.TypeIdentifier(discriminator=xt.TK_CHAR8, value=None),
                pt.wchar: xt.TypeIdentifier(discriminator=xt.TK_CHAR16, value=None),
                pt.float32: xt.TypeIdentifier(discriminator=xt.TK_FLOAT32, value=None),
                pt.float64: xt.TypeIdentifier(discriminator=xt.TK_FLOAT64, value=None)
            }[entity]

        if cls._impl_xt_is_plain(entity):
            if entity is str:
                return xt.TypeIdentifier(string_sdefn=xt.StringSTypeDefn(bound=0))
            elif isinstance(entity, pt.bounded_str):
                if entity.max_length <= 255:
                    return xt.TypeIdentifier(string_sdefn=xt.StringSTypeDefn(bound=entity.max_length))
                else:
                    return xt.TypeIdentifier(string_ldefn=xt.StringLTypeDefn(bound=entity.max_length))
            elif isinstance(entity, pt.sequence):
                fully_descriptive = cls._impl_xt_is_fully_descriptive(entity)
                header = xt.PlainCollectionHeader(
                    equiv_kind=xt.EK_BOTH if fully_descriptive else (xt.EK_MINIMAL if minimal else xt.EK_COMPLETE),
                    element_flags=xt.MemberFlag(TRY_CONSTRUCT1=True)  # TODO: elements can be external, try_construct
                )

                if entity.max_length is None:
                    return xt.TypeIdentifier(
                        seq_sdefn=xt.PlainSequenceSElemDefn(
                            header=header,
                            bound=0,
                            element_identifier=cls._xt_type_identifier(name, entity.subtype, minimal)
                        )
                    )
                elif entity.max_length <= 255:
                    return xt.TypeIdentifier(
                        seq_sdefn=xt.PlainSequenceSElemDefn(
                            header=header,
                            bound=entity.max_length,
                            element_identifier=cls._xt_type_identifier(name, entity.subtype, minimal)
                        )
                    )
                else:
                    return xt.TypeIdentifier(
                        seq_ldefn=xt.PlainSequenceLElemDefn(
                            header=header,
                            bound=entity.max_length or uint32_max,
                            element_identifier=cls._xt_type_identifier(name, entity.subtype, minimal)
                        )
                    )

            elif isinstance(entity, pt.array):
                fully_descriptive = cls._impl_xt_is_fully_descriptive(entity)
                header = xt.PlainCollectionHeader(
                    equiv_kind=xt.EK_BOTH if fully_descriptive else (xt.EK_MINIMAL if minimal else xt.EK_COMPLETE),
                    element_flags=xt.MemberFlag(TRY_CONSTRUCT1=True)  # TODO: elements can be external, try_construct
                )

                inner: Type[Any] = entity
                bounds: List[int] = []
                while isinstance(inner, pt.array):
                    bounds.append(inner.length)
                    inner = inner.subtype

                if all(b < 256 for b in bounds):
                    return xt.TypeIdentifier(
                        array_sdefn=xt.PlainArraySElemDefn(
                            header=header,
                            array_bound_seq=bounds,
                            element_identifier=cls._xt_type_identifier(name, inner, minimal)
                        )
                    )
                else:
                    return xt.TypeIdentifier(
                        array_ldefn=xt.PlainArrayLElemDefn(
                            header=header,
                            array_bound_seq=bounds,
                            element_identifier=cls._xt_type_identifier(name, inner, minimal)
                        )
                    )
            elif get_origin(entity) == Dict:
                fully_descriptive = cls._impl_xt_is_fully_descriptive(entity)
                keytype, valuetype = get_args(entity)
                header = xt.PlainCollectionHeader(
                    equiv_kind=xt.EK_BOTH if fully_descriptive else (xt.EK_MINIMAL if minimal else xt.EK_COMPLETE),
                    element_flags=xt.MemberFlag()  # TODO: elements can be external, try_construct
                )
                # No small maps
                return xt.TypeIdentifier(
                    map_ldefn=xt.PlainMapLTypeDefn(
                        header=header,
                        bound=uint32_max,
                        element_identifier=cls._xt_type_identifier(name, valuetype, minimal),
                        key_flags=xt.MemberFlag(),  # TODO: keys can be external, try_construct
                        key_identifier=cls._xt_type_identifier(name, keytype, minimal)
                    )
                )

        typehash = cls._resolve_typehash(name, entity)
        if minimal:
            return typehash.minimal_type_identifier
        else:
            return typehash.complete_type_identifier

    @classmethod
    def _xt_minimal_type_object(cls, entity: Any) -> xt.MinimalTypeObject:
        if _is_optional(entity):
            return cls._xt_minimal_type_object(entity.inner)

        if isinstance(entity, pt.typedef):
            return xt.MinimalTypeObject(alias_type=cls._xt_minimal_alias_type(entity))
        elif False:
            # python has no annotation types yet
            return xt.MinimalTypeObject(annotation_type=None)
        elif isclass(entity) and issubclass(entity, IdlStruct):
            entity.__idl__.populate()
            return xt.MinimalTypeObject(struct_type=cls._xt_minimal_struct_type(entity))
        elif isclass(entity) and issubclass(entity, IdlUnion):
            entity.__idl__.populate()
            return xt.MinimalTypeObject(union_type=cls._xt_minimal_union_type(entity))
        elif isinstance(entity, pt.sequence):
            return xt.MinimalTypeObject(sequence_type=cls._xt_minimal_sequence_type(entity))
        elif isinstance(entity, pt.array):
            return xt.MinimalTypeObject(array_type=cls._xt_minimal_array_type(entity))
        elif isinstance(entity, Dict):
            return xt.MinimalTypeObject(map_type=cls._xt_minimal_map_type(entity))
        elif isclass(entity) and issubclass(entity, IdlEnum):
            return xt.MinimalTypeObject(enumerated_type=cls._xt_minimal_enumerated_type(entity))
        elif isclass(entity) and issubclass(entity, IdlBitmask):
            return xt.MinimalTypeObject(bitmask_type=cls._xt_minimal_bitmask_type(entity))
        raise Exception(f"Cannot construct typeobject for entity {entity}")

    @classmethod
    def _xt_complete_type_object(cls, entity: Any) -> xt.CompleteTypeObject:
        if _is_optional(entity):
            return cls._xt_complete_type_object(entity.inner)

        if isinstance(entity, pt.typedef):
            return xt.CompleteTypeObject(alias_type=cls._xt_complete_alias_type(entity))
        elif False:
            # python has no annotation types yet
            return xt.CompleteTypeObject(annotation_type=None)
        elif isclass(entity) and issubclass(entity, IdlStruct):
            entity.__idl__.populate()
            return xt.CompleteTypeObject(struct_type=cls._xt_complete_struct_type(entity))
        elif isclass(entity) and issubclass(entity, IdlUnion):
            entity.__idl__.populate()
            return xt.CompleteTypeObject(union_type=cls._xt_complete_union_type(entity))
        elif isinstance(entity, pt.sequence):
            return xt.CompleteTypeObject(sequence_type=cls._xt_complete_sequence_type(entity))
        elif isinstance(entity, pt.array):
            return xt.CompleteTypeObject(array_type=cls._xt_complete_array_type(entity))
        elif isinstance(entity, Dict):
            return xt.CompleteTypeObject(map_type=cls._xt_complete_map_type(entity))
        elif isclass(entity) and issubclass(entity, IdlEnum):
            return xt.CompleteTypeObject(enumerated_type=cls._xt_complete_enumerated_type(entity))
        elif isclass(entity) and issubclass(entity, IdlBitmask):
            return xt.CompleteTypeObject(bitmask_type=cls._xt_complete_bitmask_type(entity))
        raise Exception(f"Cannot construct typeobject for entity {entity}")

    @classmethod
    def _xt_minimal_alias_type(cls, entity: Any) -> xt.MinimalAliasType:
        return xt.MinimalAliasType(
            alias_flags=xt.TypeFlag(),  # unused, no flags apply
            header=xt.MinimalAliasHeader(),  # empty...
            body=xt.MinimalAliasBody(
                common=xt.CommonAliasBody(
                    related_flags=xt.MemberFlag(),  # unused, no flags apply
                    related_type=cls._xt_type_identifier('', entity.subtype, True)
                )
            )
        )

    @classmethod
    def _xt_complete_alias_type(cls, entity: Any) -> xt.CompleteAliasType:
        return xt.CompleteAliasType(
            alias_flags=xt.TypeFlag(),  # unused, no flags apply
            header=xt.CompleteAliasHeader(
                detail=xt.CompleteTypeDetail(
                    ann_builtin=None,
                    ann_custom=None,
                    type_name=entity.__idl_typename__.replace('.', '::')
                )
            ),
            body=xt.CompleteAliasBody(
                common=xt.CommonAliasBody(
                    related_flags=xt.MemberFlag(),  # unused, no flags apply
                    related_type=cls._xt_type_identifier('', entity.subtype, False)
                ),
                ann_builtin=None,
                ann_custom=None
            )
        )

    @classmethod
    def _xt_minimal_struct_type(cls, entity: Type[IdlStruct]) -> xt.MinimalStructType:
        return xt.MinimalStructType(
            struct_flags=cls._xt_type_flag(entity),
            header=cls._xt_minimal_struct_header(entity),
            member_seq=cls._xt_minimal_struct_member_seq(entity)
        )

    @classmethod
    def _xt_complete_struct_type(cls, entity: Type[IdlStruct]) -> xt.CompleteStructType:
        return xt.CompleteStructType(
            struct_flags=cls._xt_type_flag(entity),
            header=cls._xt_complete_struct_header(entity),
            member_seq=cls._xt_complete_struct_member_seq(entity)
        )

    @classmethod
    def _xt_type_flag(cls, entity: Any) -> xt.TypeFlag:
        flag = xt.TypeFlag()
        cls_a = get_idl_annotations(entity)

        extensibility = cls_a.get("extensibility", cls._default_extensibility)

        if extensibility == "final":
            flag.IS_FINAL = True
        elif extensibility == "appendable":
            flag.IS_APPENDABLE = True
        elif extensibility == "mutable":
            flag.IS_MUTABLE = True

        if cls_a.get("nested", False):
            flag.IS_NESTED = True

        if cls_a.get("autoid", "sequential") == "hash":
            flag.IS_AUTOID_HASH = True

        return flag

    @classmethod
    def _xt_minimal_struct_header(cls, entity: Type[IdlStruct]) -> xt.MinimalStructHeader:
        if entity.__base__ is None or entity.__base__ == IdlStruct:
            return xt.MinimalStructHeader(
                base_type=xt.TypeIdentifier(discriminator=xt.TK_NONE, value=None),
                detail=cls._xt_minimal_type_detail(entity)
            )

        return xt.MinimalStructHeader(
            base_type=cls._xt_type_identifier("_base_", entity.__base__, True),
            detail=cls._xt_minimal_type_detail(entity)
        )

    @classmethod
    def _xt_complete_struct_header(cls, entity: Type[IdlStruct]) -> xt.CompleteStructHeader:
        if entity.__base__ is None or entity.__base__ == IdlStruct:
            return xt.CompleteStructHeader(
                base_type=xt.TypeIdentifier(discriminator=xt.TK_NONE, value=None),
                detail=cls._xt_complete_type_detail(entity)
            )

        return xt.CompleteStructHeader(
            base_type=cls._xt_type_identifier("_base_", entity.__base__, True),
            detail=cls._xt_complete_type_detail(entity)
        )

    @classmethod
    def _xt_minimal_type_detail(cls, entity: Any) -> xt.MinimalTypeDetail:
        # xtypes spec: left for future expansion
        return xt.MinimalTypeDetail()

    @classmethod
    def _xt_complete_type_detail(cls, entity: Any) -> xt.CompleteTypeDetail:
        return xt.CompleteTypeDetail(
            ann_builtin=None,
            ann_custom=None,
            type_name=entity.__idl_typename__.replace(".", "::") if hasattr(entity, "__idl_typename__") else "anonymous"
        )

    @classmethod
    def _xt_minimal_struct_member_seq(cls, entity: Type[IdlStruct]) -> xt.MinimalStructMemberSeq:
        parent_fields = set(
            get_extended_type_hints(entity.__base__).keys()
        ) if entity.__base__ not in (IdlStruct, None) else set()

        return [
            cls._xt_minimal_struct_member(entity, name, _type)
            for name, _type in get_extended_type_hints(entity).items()
            if name not in parent_fields
        ]

    @classmethod
    def _xt_complete_struct_member_seq(cls, entity: Type[IdlStruct]) -> xt.CompleteStructMemberSeq:
        parent_fields = set(
            get_extended_type_hints(entity.__base__).keys()
        ) if entity.__base__ not in (IdlStruct, None) else set()

        return [
            cls._xt_complete_struct_member(entity, name, _type)
            for name, _type in get_extended_type_hints(entity).items()
            if name not in parent_fields
        ]

    @classmethod
    def _xt_minimal_struct_member(cls, entity: Type[IdlStruct], name: str, _type: type) -> xt.MinimalStructMember:
        return xt.MinimalStructMember(
            common=cls._xt_common_struct_member(entity, name, _type, True),
            detail=cls._xt_minimal_member_detail(entity, name, _type)
        )

    @classmethod
    def _xt_complete_struct_member(cls, entity: Type[IdlStruct], name: str, _type: type) -> xt.CompleteStructMember:
        return xt.CompleteStructMember(
            common=cls._xt_common_struct_member(entity, name, _type, False),
            detail=cls._xt_complete_member_detail(entity, name, _type)
        )

    @classmethod
    def _xt_common_struct_member(cls, entity: Type[IdlStruct], name: str, _type: type, minimal: bool) -> xt.CommonStructMember:
        return xt.CommonStructMember(
            member_id=entity.__idl__.get_member_id(name),
            member_flags=cls._xt_member_flag(entity, name, _type),
            member_type_id=cls._xt_type_identifier(f"{entity.__idl_typename__.replace('.','::')}.{name}", _type, minimal)
        )

    @classmethod
    def _xt_member_flag(cls, entity: Any, name: str, _type: type) -> xt.MemberFlag:
        flag = xt.MemberFlag()
        field_a = get_idl_field_annotations(entity)
        annotations = field_a.get(name, {})

        # TRY CONSTRUCT TODO: INVALID, DISCARD, TRIM?
        if "default" in annotations:
            # 10 = use_default
            flag.TRY_CONSTRUCT2 = True
        else:
            # 01 = discard
            flag.TRY_CONSTRUCT1 = True

        if annotations.get("external", False):
            flag.IS_EXTERNAL = True

        if _is_optional(_type):
            flag.IS_OPTIONAL = True

        if annotations.get("must_understand", False) or annotations.get("key", False):
            flag.IS_MUST_UNDERSTAND = True

        if annotations.get("key", False):
            flag.IS_KEY = True

        if isinstance(_type, pt.default):
            flag.IS_DEFAULT = True

        return flag

    @classmethod
    def _xt_minimal_member_detail(cls, entity: Any, name: str, _type: type) -> xt.MinimalMemberDetail:
        hash = md5(name.encode('utf8')).digest()
        return xt.MinimalMemberDetail(
            name_hash=hash[0:4]
        )

    @classmethod
    def _xt_complete_member_detail(cls, entity: Any, name: str, _type: type) -> xt.CompleteMemberDetail:
        field_a = get_idl_field_annotations(entity)
        annotations = field_a.get(name, {})
        return xt.CompleteMemberDetail(
            name=name,
            ann_builtin=cls._xt_applied_builtin_member_annotations(annotations, _type),
            ann_custom=None
        )

    @classmethod
    def _xt_applied_builtin_member_annotations(
        cls, annotations: Dict[str, Any], _type: Any
    ) -> Optional[xt.AppliedBuiltinMemberAnnotations]:
        if "hash_id" in annotations:
            return xt.AppliedBuiltinMemberAnnotations(
                unit=None,  # unused so far
                min=None,  # unused so far
                max=None,  # unused so far
                hash_id=annotations["hash_id"] or ""
            )
        return None

    @classmethod
    def _xt_minimal_union_type(cls, entity: Type[IdlUnion]) -> xt.MinimalUnionType:
        return xt.MinimalUnionType(
            union_flags=cls._xt_type_flag(entity),
            header=cls._xt_minimal_union_header(entity),
            discriminator=cls._xt_minimal_discriminator_member(entity),
            member_seq=cls._xt_minimal_union_member_seq(entity)
        )

    @classmethod
    def _xt_complete_union_type(cls, entity: Type[IdlUnion]) -> xt.CompleteUnionType:
        return xt.CompleteUnionType(
            union_flags=cls._xt_type_flag(entity),
            header=cls._xt_complete_union_header(entity),
            discriminator=cls._xt_complete_discriminator_member(entity),
            member_seq=cls._xt_complete_union_member_seq(entity)
        )

    @classmethod
    def _xt_minimal_union_header(cls, entity: Type[IdlUnion]) -> xt.MinimalUnionHeader:
        return xt.MinimalUnionHeader(
            detail=cls._xt_minimal_type_detail(entity)
        )

    @classmethod
    def _xt_complete_union_header(cls, entity: Type[IdlUnion]) -> xt.CompleteUnionHeader:
        return xt.CompleteUnionHeader(
            detail=cls._xt_complete_type_detail(entity)
        )

    @classmethod
    def _xt_minimal_discriminator_member(cls, entity: Type[IdlUnion]) -> xt.MinimalDiscriminatorMember:
        return xt.MinimalDiscriminatorMember(
            common=cls._xt_common_discriminator_member(entity, True)
        )

    @classmethod
    def _xt_complete_discriminator_member(cls, entity: Type[IdlUnion]) -> xt.CompleteDiscriminatorMember:
        return xt.CompleteDiscriminatorMember(
            common=cls._xt_common_discriminator_member(entity, False),
            ann_builtin=None,
            ann_custom=None,
        )

    @classmethod
    def _xt_common_discriminator_member(cls, entity: Type[IdlUnion], minimal: bool) -> xt.CommonDiscriminatorMember:
        return xt.CommonDiscriminatorMember(
            member_flags=xt.MemberFlag(IS_KEY=entity.__idl_discriminator_is_key__, IS_MUST_UNDERSTAND=True, TRY_CONSTRUCT1=True),
            type_id=cls._xt_type_identifier("discriminator", entity.__idl_discriminator__, minimal)
        )

    @classmethod
    def _xt_minimal_union_member_seq(cls, entity: Type[IdlUnion]) -> xt.MinimalUnionMemberSeq:
        parent_fields = set(
            get_extended_type_hints(entity.__base__).keys()
        ) if entity.__base__ not in (IdlUnion, None) else set()

        return [
            cls._xt_minimal_union_member(entity, name, _type)
            for name, _type in get_extended_type_hints(entity).items()
            if name not in parent_fields
        ]

    @classmethod
    def _xt_complete_union_member_seq(cls, entity: Type[IdlUnion]) -> xt.CompleteUnionMemberSeq:
        parent_fields = set(
            get_extended_type_hints(entity.__base__).keys()
        ) if entity.__base__ not in (IdlUnion, None) else set()

        return [
            cls._xt_complete_union_member(entity, name, _type)
            for name, _type in get_extended_type_hints(entity).items()
            if name not in parent_fields
        ]

    @classmethod
    def _xt_minimal_union_member(
        cls, entity: Type[IdlUnion], name: str, _type: Type[Any]
    ) -> xt.MinimalUnionMember:
        return xt.MinimalUnionMember(
            common=cls._xt_common_union_member(entity, name, _type, True),
            detail=cls._xt_minimal_member_detail(entity, name, _type.subtype)
        )

    @classmethod
    def _xt_complete_union_member(
        cls, entity: Type[IdlUnion], name: str, _type: Type[Any]
    ) -> xt.CompleteUnionMember:
        return xt.CompleteUnionMember(
            common=cls._xt_common_union_member(entity, name, _type, False),
            detail=cls._xt_complete_member_detail(entity, name, _type.subtype)
        )

    @classmethod
    def _xt_common_union_member(
        cls, entity: Type[IdlUnion], name: str, _type: Type[Any], minimal: bool
    ) -> xt.CommonUnionMember:
        return xt.CommonUnionMember(
            member_id=entity.__idl__.get_member_id(name),
            member_flags=cls._xt_member_flag(entity, name, _type),
            type_id=cls._xt_type_identifier(f"{entity.__idl_typename__.replace('.','::')}.{name}", _type.subtype, minimal),
            label_seq=cls._xt_union_case_label_seq(entity, name, _type)
        )

    @classmethod
    def _xt_union_case_label_seq(cls, entity: Type[IdlUnion], name: str, _type: Type[Any]) -> xt.UnionCaseLabelSeq:
        if isclass(entity.__idl_discriminator__) and issubclass(entity.__idl_discriminator__, IdlEnum):
            return [l.value for l in _type.labels] if isinstance(_type, pt.case) else []
        return _type.labels if isinstance(_type, pt.case) else []

    @classmethod
    def _xt_minimal_sequence_type(cls, seq: pt.sequence) -> xt.MinimalSequenceType:
        return xt.MinimalSequenceType(
            collection_flag=xt.TypeFlag(),  # Unused, no flags apply
            header=cls._xt_minimal_collection_header(seq),
            element=cls._xt_minimal_collection_element(seq.subtype, xt.TypeFlag(TRY_CONSTRUCT1=True))
        )

    @classmethod
    def _xt_complete_sequence_type(cls, seq: pt.sequence) -> xt.CompleteSequenceType:
        return xt.CompleteSequenceType(
            collection_flag=xt.TypeFlag(),  # Unused, no flags apply
            header=cls._xt_complete_collection_header(seq),
            element=cls._xt_complete_collection_element(seq.subtype, xt.TypeFlag(TRY_CONSTRUCT1=True))
        )

    @classmethod
    def _xt_minimal_collection_header(cls, collection: Union[pt.array, pt.sequence]) -> xt.MinimalCollectionHeader:
        return xt.MinimalCollectionHeader(
            common=cls._xt_common_collection_header(collection)
        )

    @classmethod
    def _xt_complete_collection_header(cls, collection: Union[pt.array, pt.sequence]) -> xt.CompleteCollectionHeader:
        return xt.CompleteCollectionHeader(
            common=cls._xt_common_collection_header(collection),
            detail=cls._xt_complete_type_detail(collection)
        )

    @classmethod
    def _xt_common_collection_header(cls, collection: Union[pt.array, pt.sequence]) -> xt.CommonCollectionHeader:
        return xt.CommonCollectionHeader(
            bound=uint32_max if isinstance(collection, pt.array) or collection.max_length is None else collection.max_length
        )

    @classmethod
    def _xt_minimal_collection_element(cls, _type: Any, flags: xt.TypeFlag) -> xt.MinimalCollectionElement:
        return xt.MinimalCollectionElement(
            common=cls._xt_common_collection_element(_type, True, flags)
        )

    @classmethod
    def _xt_complete_collection_element(cls, _type: Any, flags: xt.TypeFlag) -> xt.CompleteCollectionElement:
        return xt.CompleteCollectionElement(
            common=cls._xt_common_collection_element(_type, True, flags),
            detail=cls._xt_complete_element_detail(_type)
        )

    @classmethod
    def _xt_complete_element_detail(cls, _type: Any) -> xt.CompleteElementDetail:
        return xt.CompleteElementDetail(
            ann_builtin=cls._xt_applied_builtin_member_annotations({}, _type),
            ann_custom=None
        )

    @classmethod
    def _xt_common_collection_element(cls, _type: Any, minimal: bool, flags: xt.TypeFlag) -> xt.CommonCollectionElement:
        # TODO: how to scope this? Annotations can only be on the parent object,
        # how to get the try_construct/external flags here?
        return xt.CommonCollectionElement(
            element_flags=flags,
            type=cls._xt_type_identifier("anonymous", _type, minimal)
        )

    @classmethod
    def _xt_minimal_array_type(cls, arr: pt.array) -> xt.MinimalArrayType:
        inner = arr
        while isinstance(inner, pt.array):
            inner = inner.subtype

        return xt.MinimalArrayType(
            collection_flag=xt.TypeFlag(),  # Unused, no flags apply
            header=cls._xt_minimal_array_header(arr),
            element=cls._xt_minimal_collection_element(inner, xt.TypeFlag(TRY_CONSTRUCT1=True))

        )

    @classmethod
    def _xt_complete_array_type(cls, arr: pt.array) -> xt.CompleteArrayType:
        inner = arr
        while isinstance(inner, pt.array):
            inner = inner.subtype

        return xt.CompleteArrayType(
            collection_flag=xt.TypeFlag(),  # Unused, no flags apply
            header=cls._xt_complete_array_header(arr),
            element=cls._xt_complete_collection_element(inner, xt.TypeFlag(TRY_CONSTRUCT1=True))

        )

    @classmethod
    def _xt_minimal_array_header(cls, arr: pt.array) -> xt.MinimalArrayHeader:
        return xt.MinimalArrayHeader(
            common=cls._xt_common_array_header(arr)
        )

    @classmethod
    def _xt_complete_array_header(cls, arr: pt.array) -> xt.CompleteArrayHeader:
        return xt.CompleteArrayHeader(
            common=cls._xt_common_array_header(arr),
            detail=cls._xt_complete_type_detail(arr)
        )

    @classmethod
    def _xt_common_array_header(cls, arr: pt.array) -> xt.CommonArrayHeader:
        inner = arr
        bounds = []
        while isinstance(inner, pt.array):
            bounds.append(inner.length)
            inner = inner.subtype

        return xt.CommonArrayHeader(
            bound_seq=bounds
        )

    @classmethod
    def _xt_minimal_bitmask_type(cls, entity: Type[IdlBitmask]) -> xt.MinimalBitmaskType:
        return xt.MinimalBitmaskType(
            bitmask_flags=cls._xt_type_flag(entity),
            header=cls._xt_minimal_enumerated_header(entity),
            flag_seq=cls._xt_minimal_bitflag_seq(entity)
        )

    @classmethod
    def _xt_complete_bitmask_type(cls, entity: Type[IdlBitmask]) -> xt.CompleteBitmaskType:
        return xt.CompleteBitmaskType(
            bitmask_flags=cls._xt_type_flag(entity),
            header=cls._xt_complete_enumerated_header(entity),
            flag_seq=cls._xt_complete_bitflag_seq(entity)
        )

    @classmethod
    def _xt_minimal_enumerated_header(cls, entity: Union[Type[IdlEnum], Type[IdlBitmask]]) -> xt.MinimalEnumeratedHeader:
        return xt.MinimalEnumeratedHeader(
            common=cls._xt_common_enumerated_header(entity)
        )

    @classmethod
    def _xt_complete_enumerated_header(cls, entity: Union[Type[IdlEnum], Type[IdlBitmask]]) -> xt.CompleteEnumeratedHeader:
        return xt.CompleteEnumeratedHeader(
            common=cls._xt_common_enumerated_header(entity),
            detail=cls._xt_complete_type_detail(entity)
        )

    @classmethod
    def _xt_common_enumerated_header(cls, entity: Union[Type[IdlEnum], Type[IdlBitmask]]) -> xt.CommonEnumeratedHeader:
        return xt.CommonEnumeratedHeader(
            bit_bound=get_idl_annotations(entity).get("bit_bound", 32)
        )

    @classmethod
    def _xt_minimal_bitflag_seq(cls, entity: Type[IdlBitmask]) -> xt.MinimalBitflagSeq:
        seq = [
            cls._xt_minimal_bitflag(entity, name)
            for name, _ in get_extended_type_hints(entity).items()
        ]
        return list(sorted(seq, key=lambda x: x.common.position))

    @classmethod
    def _xt_complete_bitflag_seq(cls, entity: Type[IdlBitmask]) -> xt.CompleteBitflagSeq:
        seq = [
            cls._xt_complete_bitflag(entity, name)
            for name, _ in get_extended_type_hints(entity).items()
        ]
        return list(sorted(seq, key=lambda x: x.common.position))

    @classmethod
    def _xt_minimal_bitflag(cls, entity: Type[IdlBitmask], name: str) -> xt.MinimalBitflag:
        return xt.MinimalBitflag(
            common=cls._xt_common_bit_flag(entity, name),
            detail=cls._xt_minimal_member_detail(entity, name, bool)
        )

    @classmethod
    def _xt_complete_bitflag(cls, entity: Type[IdlBitmask], name: str) -> xt.CompleteBitflag:
        return xt.CompleteBitflag(
            common=cls._xt_common_bit_flag(entity, name),
            detail=cls._xt_complete_member_detail(entity, name, bool)
        )

    @classmethod
    def _xt_common_bit_flag(cls, entity: Type[IdlBitmask], name: str) -> xt.CommonBitflag:
        return xt.CommonBitflag(
            position=entity.__idl_positions__[name],
            flags=xt.MemberFlag()  # Unused, no flags apply
        )

    @classmethod
    def _xt_complete_enumerated_type(cls, entity: Type[IdlEnum]) -> xt.CompleteEnumeratedType:
        return xt.CompleteEnumeratedType(
            enum_flags=cls._xt_type_flag(entity),
            header=cls._xt_complete_enumerated_header(entity),
            literal_seq=cls._xt_complete_enumerated_literal_seq(entity)
        )

    @classmethod
    def _xt_minimal_enumerated_type(cls, entity: Type[IdlEnum]) -> xt.MinimalEnumeratedType:
        return xt.MinimalEnumeratedType(
            enum_flags=cls._xt_type_flag(entity),
            header=cls._xt_minimal_enumerated_header(entity),
            literal_seq=cls._xt_minimal_enumerated_literal_seq(entity)
        )

    @classmethod
    def _xt_complete_enumerated_literal_seq(cls, entity: Type[IdlEnum]) -> xt.CompleteEnumeratedLiteralSeq:
        seq = [
            cls._xt_complete_enumerated_literal(entity, entry)
            for entry in entity
        ]
        return list(sorted(seq, key=lambda x: x.common.value))

    @classmethod
    def _xt_minimal_enumerated_literal_seq(cls, entity: Type[IdlEnum]) -> xt.MinimalEnumeratedLiteralSeq:
        seq = [
            cls._xt_minimal_enumerated_literal(entity, entry)
            for entry in entity
        ]
        return list(sorted(seq, key=lambda x: x.common.value))

    @classmethod
    def _xt_complete_enumerated_literal(cls, entity: Type[IdlEnum], instance: IdlEnum) -> xt.CompleteEnumeratedLiteral:
        return xt.CompleteEnumeratedLiteral(
            common=cls._xt_common_enumerated_literal(entity, instance),
            detail=cls._xt_complete_member_detail(entity, instance.name, entity)
        )

    @classmethod
    def _xt_minimal_enumerated_literal(cls, entity: Type[IdlEnum], instance: IdlEnum) -> xt.MinimalEnumeratedLiteral:
        return xt.MinimalEnumeratedLiteral(
            common=cls._xt_common_enumerated_literal(entity, instance),
            detail=cls._xt_minimal_member_detail(entity, instance.name, entity)
        )

    @classmethod
    def _xt_common_enumerated_literal(cls, entity: Type[IdlEnum], instance: IdlEnum) -> xt.CommonEnumeratedLiteral:
        return xt.CommonEnumeratedLiteral(
            value=instance.value,
            flags=xt.MemberFlag(IS_DEFAULT=instance.value == entity.__idl_enum_default_value__)
        )


class XTParseState:
    class Deferred:
        def __init__(self):
            self.callbacks = []

    def __init__(self, database):
        self.database: Dict[xt.TypeIdentifier, xt.TypeObject] = database
        self.resolved: Dict[xt.TypeIdentifier, Any] = {}
        self.reentrant: Set[xt.TypeIdentifier] = set()
        self.waiting: Dict[xt.TypeIdentifier, List[XTParseState.DeferredMember]] = {}


class XTInterpreter:
    _simple_mappings = {
        xt.TK_BOOLEAN: bool,
        xt.TK_BYTE: pt.uint8,
        xt.TK_INT16: pt.int16,
        xt.TK_INT32: pt.int32,
        xt.TK_INT64: pt.int64,
        xt.TK_UINT16: pt.uint16,
        xt.TK_UINT32: pt.uint32,
        xt.TK_UINT64: pt.uint64,
        xt.TK_FLOAT32: pt.float32,
        xt.TK_FLOAT64: pt.float64,
        # float128 omitted
        xt.TK_CHAR8: pt.char,
        xt.TK_CHAR16: pt.wchar,
        xt.TK_STRING8: str
    }

    @classmethod
    def xt_to_class(cls, ident: xt.TypeIdentifier,
                    tmap: Dict[xt.TypeIdentifier, xt.TypeObject]) -> \
            Tuple[Union[IdlUnion, IdlStruct], Dict[str, Union[IdlUnion, IdlStruct, IdlEnum, IdlBitmask]]]:
        state = XTParseState(tmap)

        main_type = cls._from_typeid(ident, state)
        all_types = {
            (c.name if isinstance(c, pt.typedef) else c.__name__): c
            for c in state.resolved.values()
        }
        return main_type, all_types

    @classmethod
    def _from_typeid(cls, ident: xt.TypeIdentifier, state: XTParseState) -> Any:
        tk = ident.discriminator

        if not tk or tk == xt.TK_NONE:
            return None
        elif tk in cls._simple_mappings:
            return cls._simple_mappings[tk]
        elif tk in [xt.TK_FLOAT128, xt.TK_STRING16, xt.TI_STRING16_SMALL, xt.TI_STRING16_LARGE]:
            raise NotImplementedError("Float128 and wstring types are not supported in Python.")
        elif tk in [xt.TI_STRING8_SMALL, xt.TI_STRING8_LARGE]:
            descriptor: Union[xt.StringSTypeDefn, xt.StringLTypeDefn] = ident.value
            if descriptor.bound == 0 or descriptor.bound == uint32_max:
                return str
            return pt.bounded_str[descriptor.bound]
        elif tk in [xt.TI_PLAIN_SEQUENCE_SMALL, xt.TI_PLAIN_SEQUENCE_LARGE]:
            descriptor: Union[xt.PlainSequenceSElemDefn, xt.PlainSequenceLElemDefn] = ident.value
            if descriptor.header.element_flags.IS_OPTIONAL:
                if descriptor.bound:
                    return Optional[pt.sequence[cls._from_typeid(descriptor.element_identifier, state), descriptor.bound]]
                return Optional[pt.sequence[cls._from_typeid(descriptor.element_identifier, state)]]

            if descriptor.bound:
                return pt.sequence[cls._from_typeid(descriptor.element_identifier, state), descriptor.bound]
            return pt.sequence[cls._from_typeid(descriptor.element_identifier, state)]
        elif tk in [xt.TI_PLAIN_ARRAY_SMALL, xt.TI_PLAIN_ARRAY_LARGE]:
            descriptor: Union[xt.PlainArraySElemDefn, xt.PlainArrayLElemDefn] = ident.value
            inner = cls._from_typeid(descriptor.element_identifier, state)

            for bound in reversed(descriptor.array_bound_seq):
                inner = pt.array[inner, bound]

            if descriptor.header.element_flags.IS_OPTIONAL:
                inner = Optional[inner]

            return inner
        # add map types...
        else:
            return cls._from_typeobject(ident, state)

    @classmethod
    def _from_typeobject(cls, ident: xt.TypeIdentifier, state: XTParseState) -> Any:
        if ident in state.resolved:
            return state.resolved[ident]

        if ident in state.reentrant:
            defer = XTParseState.Deferred()
            state.waiting[ident].append(defer)
            return defer

        state.reentrant.add(ident)
        state.waiting[ident] = []

        obj = state.database[ident]
        if (obj.discriminator & xt.EK_COMPLETE) == xt.EK_COMPLETE:
            complete_obj: xt.CompleteTypeObject = obj.complete
            tk = complete_obj.discriminator

            if tk == xt.TK_STRUCTURE:
                return cls._make_complete_struct(ident, complete_obj.struct_type, state)
            elif tk == xt.TK_UNION:
                return cls._make_complete_union(ident, complete_obj.union_type, state)
            elif tk == xt.TK_BITMASK:
                return cls._make_complete_bitmask(ident, complete_obj.bitmask_type, state)
            elif tk == xt.TK_ARRAY:
                return cls._make_complete_array(ident, complete_obj.array_type, state)
            elif tk == xt.TK_SEQUENCE:
                return cls._make_complete_sequence(ident, complete_obj.sequence_type, state)
            elif tk == xt.TK_ENUM:
                return cls._make_complete_enum(ident, complete_obj.enumerated_type, state)
            elif tk == xt.TK_ALIAS:
                return cls._make_complete_alias(ident, complete_obj.alias_type, state)
            # map types, annotation types
            raise NotImplementedError()
        else:
            # We don't convert minimal type ids back into types
            raise NotImplementedError()

    @classmethod
    def _resolve_reentrant(cls, ident: xt.TypeIdentifier, resolver: Union[IdlStruct, IdlUnion], state: XTParseState) -> None:
        state.reentrant.remove(ident)
        state.resolved[ident] = resolver

        for waiter in state.waiting[ident]:
            for callback in waiter.callbacks:
                callback()

        del state.waiting[ident]

    @classmethod
    def _make_complete_struct(cls, ident: xt.TypeIdentifier, pre_struct: xt.CompleteStructType,
                              state: XTParseState) -> Type[IdlStruct]:
        base = cls._from_typeid(pre_struct.header.base_type, state)

        types = {}
        annotations = {}
        member_ids = {}

        bases = (base,) if base is not None else tuple()
        defers = []

        for m in pre_struct.member_seq:
            m_id = m.common.member_id
            m_type = cls._from_typeid(m.common.member_type_id, state)
            m_name = m.detail.name
            m_annotations = {}

            if m.common.member_flags.IS_MUST_UNDERSTAND:
                m_annotations["must_understand"] = True

            if m.common.member_flags.IS_KEY:
                m_annotations["key"] = True

            if m.common.member_flags.IS_EXTERNAL:
                m_annotations["external"] = True

            if isinstance(m_type, XTParseState.Deferred):
                defers.append((m_name, m.common.member_type_id, m_type, m.common.member_flags.IS_OPTIONAL))
                m_type = Any
            elif m.common.member_flags.IS_OPTIONAL:
                m_type = Optional[m_type]

            types[m_name] = m_type
            if m_annotations:
                annotations[m_name] = m_annotations
            member_ids[m_name] = m_id

        struct = make_idl_struct(
            class_name=pre_struct.header.detail.type_name.split('::')[-1],
            typename=pre_struct.header.detail.type_name.replace('::', '.'),
            fields=types,
            bases=bases,
            field_annotations=annotations
        )
        struct.__idl__.member_ids = member_ids

        if pre_struct.struct_flags.IS_FINAL:
            struct = annotate.final(struct)
        if pre_struct.struct_flags.IS_APPENDABLE:
            struct = annotate.appendable(struct)
        if pre_struct.struct_flags.IS_MUTABLE:
            struct = annotate.mutable(struct)

        for (m_name, type_ident, defer, is_optional) in defers:
            def def_callback():
                _type = XTInterpreter._from_typeid(type_ident, state)
                struct.__annotations__[m_name] = Optional[_type] if is_optional else _type

            defer.callbacks.append(def_callback)

        cls._resolve_reentrant(ident, struct, state)
        return struct

    @classmethod
    def _make_complete_union(cls, ident: xt.TypeIdentifier, pre_union: xt.CompleteUnionType,
                             state: XTParseState) -> Type[IdlStruct]:
        discriminator = cls._from_typeid(pre_union.discriminator.common.type_id, state)

        cases = {}
        annotations = {}
        member_ids = {}
        defers = []

        for m in pre_union.member_seq:
            m_id = m.common.member_id
            m_type = cls._from_typeid(m.common.type_id, state)
            m_name = m.detail.name
            m_annotations = {}

            if m.common.member_flags.IS_MUST_UNDERSTAND:
                m_annotations["must_understand"] = True

            if m.common.member_flags.IS_EXTERNAL:
                m_annotations["external"] = True

            if isinstance(m_type, XTParseState.Deferred):
                defers.append((m_name, m.common.member_type_id, m_type))
                m_type = Any

            if m.common.member_flags.IS_DEFAULT or not m.common.label_seq:
                m_type = pt.default[m_type]
            else:
                m_type = pt.case[m.common.label_seq, m_type]

            cases[m_name] = m_type
            if m_annotations:
                annotations[m_name] = m_annotations
            member_ids[m_name] = m_id

        union = make_idl_union(
            class_name=pre_union.header.detail.type_name.split('::')[-1],
            typename=pre_union.header.detail.type_name.replace('::', '.'),
            fields=cases,
            field_annotations=annotations,
            discriminator=discriminator,
            discriminator_is_key=pre_union.discriminator.common.member_flags.IS_KEY
        )
        union.__idl__.member_ids = member_ids

        if pre_union.union_flags.IS_FINAL:
            union = annotate.final(union)
        if pre_union.union_flags.IS_APPENDABLE:
            union = annotate.appendable(union)
        if pre_union.union_flags.IS_MUTABLE:
            union = annotate.mutable(union)

        for (m_name, type_ident, defer) in defers:
            def def_callback():
                _type = XTInterpreter._from_typeid(type_ident, state)
                union.__annotations__[m_name] = _type

            defer.callbacks.append(def_callback)

        cls._resolve_reentrant(ident, union, state)
        return union

    @classmethod
    def _make_complete_bitmask(cls, ident: xt.TypeIdentifier, bitmask_type: xt.CompleteBitmaskType,
                               state: XTParseState) -> Type[IdlStruct]:
        bitmask = make_idl_bitmask(
            class_name=bitmask_type.header.detail.type_name.split('::')[-1],
            typename=bitmask_type.header.detail.type_name.replace('::', '.'),
            fields=[flag.detail.name for flag in bitmask_type.flag_seq],
            dataclassify=True,
            field_annotations={
                flag.detail.name: {
                    'position': flag.common.position
                }
                for flag in bitmask_type.flag_seq
            }
        )

        if bitmask_type.header.common.bit_bound != 32:
            annotate.bit_bound(bitmask_type.header.common.bit_bound)(bitmask)

        cls._resolve_reentrant(ident, bitmask, state)
        return bitmask

    @classmethod
    def _make_complete_enum(cls, ident: xt.TypeIdentifier, enum_type: xt.CompleteEnumeratedType,
                            state: XTParseState) -> Type[IdlStruct]:
        # Enums cannot be re-entrant, no deps so no possible cycles.

        _default = None
        for literal in enum_type.literal_seq:
            if literal.common.flags.IS_DEFAULT:
                _default = literal.detail.name
                break

        enum_ = make_idl_enum(
            class_name=enum_type.header.detail.type_name.split('::')[-1],
            typename=enum_type.header.detail.type_name.replace('::', '.'),
            fields={
                literal.detail.name: literal.common.value
                for literal in enum_type.literal_seq
            },
            default=_default
        )

        cls._resolve_reentrant(ident, enum_, state)
        return enum_

    @classmethod
    def _make_complete_sequence(cls, ident: xt.TypeIdentifier, pre_seq: xt.CompleteSequenceType,
                                state: XTParseState) -> pt.sequence:
        inner = cls._from_typeid(pre_seq.element.common.type, state)

        if isinstance(inner, XTParseState.Deferred):
            return inner

        if pre_seq.header.common.bound:
            return pt.sequence[inner, pre_seq.header.common.bound]

        seq = pt.sequence[inner]
        cls._resolve_reentrant(ident, seq, state)
        return seq

    @classmethod
    def _make_complete_array(cls, ident: xt.TypeIdentifier, pre_arr: xt.CompleteArrayType, state: XTParseState) -> pt.array:
        inner = cls._from_typeid(pre_arr.element.common.type, state)

        if isinstance(inner, XTParseState.Deferred):
            return inner

        for bound in reversed(pre_arr.header.common.bound_seq):
            inner = pt.array[inner, bound]

        cls._resolve_reentrant(ident, inner, state)
        return inner

    @classmethod
    def _make_complete_alias(cls, ident: xt.TypeIdentifier, pre_alias: xt.CompleteAliasType,
                             state: XTParseState) -> pt.typedef:
        inner = cls._from_typeid(pre_alias.body.common.related_type, state)

        if isinstance(inner, XTParseState.Deferred):
            return inner

        typedef = pt.typedef(pre_alias.header.detail.type_name, inner)
        cls._resolve_reentrant(ident, typedef, state)
        return typedef


class XTTypeIdScanner:
    """Utility to discover all Type Id's mentioned in a TypeObject"""
    @classmethod
    def find_all_typeids(cls, type_object: xt.TypeObject) -> List[xt.TypeIdentifier]:
        if (type_object.discriminator & xt.EK_COMPLETE) == xt.EK_COMPLETE:
            return cls._scan_complete(type_object.complete)
        raise NotImplementedError()

    @classmethod
    def _scan_complete(cls, complete_type_object: xt.CompleteTypeObject) -> List[xt.TypeIdentifier]:
        tk = complete_type_object.discriminator

        if tk == xt.TK_STRUCTURE:
            return cls._scan_complete_struct(complete_type_object.struct_type)
        elif tk == xt.TK_UNION:
            return cls._scan_complete_union(complete_type_object.union_type)
        elif tk == xt.TK_ARRAY:
            return cls._scan_complete_array(complete_type_object.array_type)
        elif tk == xt.TK_SEQUENCE:
            return cls._scan_complete_sequence(complete_type_object.sequence_type)
        elif tk == xt.TK_ALIAS:
            return cls._scan_complete_alias(complete_type_object.alias_type)
        elif tk in [xt.TK_ENUM, xt.TK_BITMASK]:
            return []  # No deps
        # Unsupported: map, annotation, bitset
        raise NotImplementedError()

    @classmethod
    def _scan_complete_struct(cls, complete_struct: xt.CompleteStructType):
        ret = []
        if complete_struct.header.base_type.discriminator != xt.TK_NONE:
            ret.append(complete_struct.header.base_type)

        for member in complete_struct.member_seq:
            # EK_BOTH signifies _plain_
            if member.common.member_type_id.discriminator != xt.EK_BOTH:
                ret += cls._scan_type_id(member.common.member_type_id)

        return ret

    @classmethod
    def _scan_complete_union(cls, complete_union: xt.CompleteUnionType):
        ret = []

        if complete_union.discriminator.common.type_id.discriminator != xt.EK_BOTH:
            ret += cls._scan_type_id(complete_union.discriminator.common.type_id)

        for member in complete_union.member_seq:
            # EK_BOTH signifies _plain_
            if member.common.type_id.discriminator != xt.EK_BOTH:
                ret += cls._scan_type_id(member.common.type_id)

        return ret

    @classmethod
    def _scan_complete_sequence(cls, pre_seq: xt.CompleteSequenceType):
        return cls._scan_type_id(pre_seq.element.common.type)

    @classmethod
    def _scan_complete_array(cls, pre_arr: xt.CompleteArrayType):
        return cls._scan_type_id(pre_arr.element.common.type)

    @classmethod
    def _scan_complete_alias(cls, pre_alias: xt.CompleteAliasType):
        return cls._scan_type_id(pre_alias.body.common.related_type)

    @classmethod
    def _scan_type_id(cls, type_id: xt.TypeIdentifier):
        tk = type_id.discriminator

        if tk <= xt.TK_CHAR16:
            # simple type
            return []
        if tk in [xt.TI_STRING8_SMALL, xt.TI_STRING16_SMALL, xt.TI_STRING8_LARGE,
                  xt.TI_STRING16_LARGE, xt.TK_STRING8, xt.TK_STRING16]:
            return []
        elif tk in [xt.TI_PLAIN_SEQUENCE_SMALL, xt.TI_PLAIN_SEQUENCE_LARGE]:
            descriptor: Union[xt.PlainSequenceSElemDefn, xt.PlainSequenceLElemDefn] = type_id.value
            return cls._scan_type_id(descriptor.element_identifier)
        elif tk in [xt.TI_PLAIN_ARRAY_SMALL, xt.TI_PLAIN_ARRAY_LARGE]:
            descriptor: Union[xt.PlainArraySElemDefn, xt.PlainArrayLElemDefn] = type_id.value
            return cls._scan_type_id(descriptor.element_identifier)
        return [type_id]
