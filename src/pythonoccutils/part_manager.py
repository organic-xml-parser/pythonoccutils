from __future__ import annotations

import math
import pdb
import re
import typing

import OCC
import OCC.Core.Addons
import OCC.Core.BOPAlgo
import OCC.Core.BRepAlgoAPI
import OCC.Core.BRepAlgoAPI
import OCC.Core.BRepBuilderAPI
import OCC.Core.BRepBuilderAPI
import OCC.Core.BRepCheck
import OCC.Core.BRepFilletAPI
import OCC.Core.BRepOffset
import OCC.Core.BRepOffsetAPI
import OCC.Core.BRepPrimAPI
import OCC.Core.BRepLib
import OCC.Core.BOPAlgo
import OCC.Core.BRepTools as BRepTools
import OCC.Core.GeomAbs
import OCC.Core.GC
import OCC.Core.GCE2d
import OCC.Core.Geom
import OCC.Core.Geom2d
import OCC.Core.ShapeAnalysis
import OCC.Core.ShapeFix
import OCC.Core.ShapeUpgrade
import OCC.Core.TopAbs as ta
import OCC.Core.TopOpeBRepBuild
import OCC.Core.TopTools
import OCC.Core.TopoDS
import OCC.Core.gp
import OCC.Core.gp as gp
import parsimonious
from OCC.Core.Geom import Geom_CylindricalSurface
from OCC.Core.Message import Message_Gravity
from OCC.Core._TopAbs import TopAbs_WIRE, TopAbs_EDGE
from parsimonious import Grammar

import pythonoccutils.occutils_python as op

import logging

logger = logging.getLogger(__name__)


class Part:

    def pruned(self) -> Part:
        """
        :return: A Part with identical root shape, and subshape map with any parts that are not
        in the object hierarchy of the root shape removed. e.g. orphaned faces/edges that no longer
        belong to the root shape.
        """
        new_named_subshapes = {}

        shape_set = set()
        for s in op.InterrogateUtils.traverse_all_subshapes(self._shape):
            shape_set.add(op.SetPlaceableShape(s))

        logger.debug(f"Filtering shape set {shape_set}")

        for n, l in self._named_subshapes.items():
            sublist = []
            for s in l:
                if op.SetPlaceableShape(s) in shape_set:
                    logger.debug(f"Preserving {s}")
                    sublist.append(s)
                else:
                    logger.debug(f"Discarding: {s}")

            if len(sublist) > 0:
                new_named_subshapes[n] = sublist

        return Part(self._shape, new_named_subshapes)

    def __init__(self,
                 shape: OCC.Core.TopoDS.TopoDS_Shape,
                 subshapes: typing.Dict[str, typing.List[OCC.Core.TopoDS.TopoDS_Shape]] = None):
        self._shape = shape
        self._extents = None
        self._named_subshapes = {}

        if subshapes is not None:
            for n, l in subshapes.items():
                self._named_subshapes[n] = [s for s in l]

    def raise_exception(self) -> Part:
        """
        This method never returns. Instead, it raises a RuntimeError. This can be used to halt project execution without
        having to comment out large chunks of code.
        """
        raise RuntimeError("Run cancelled")

    @property
    def validate(self):
        return PartValidate(self)

    def preview(self, *preview_with: Part, **kwargs) -> Part:
        Part.visualize(self, *preview_with, **kwargs)
        return self

    def print(self, title: str = None) -> Part:
        """
        Print a debug description of the part/partmap
        """
        if title is not None:
            print(title + ':')

        print(str(self._shape) + " " + str(id(self._shape)))
        print("    " + str([s for s in op.InterrogateUtils.traverse_direct_subshapes(self._shape)]))

        for n, l in self._named_subshapes.items():
            print(f"    {n}: {', '.join([str(s) + str(id(s)) for s in l])}")

        print(f"BBox: xyz_span{self.extents.xyz_span} xyz_min:{self.extents.xyz_min} - xyz_max{self.extents.xyz_max}")

        print()

        return self

    def name(self, name: str) -> Part:
        """
        Returns a new Part with this Part's root shape, and the subshape map containing the root shape
        Renamed to name.
        :param name:
        :return:
        """
        subshapes = self.subshapes
        for n, lst in subshapes.items():
            if self.shape in lst:
                lst.remove(self.shape)

        subshapes[name] = [self.shape]
        return Part(self.shape, subshapes)

    def name_recurse(self, name: str, subshape_filter: typing.Callable[[OCC.Core.TopoDS.TopoDS_Shape], bool] = None) -> Part:
        """
        Returns a new Part with this Part's root shape, and the subshape map containing the root shape
        Renamed to name.
        :param name:
        :return:
        """

        if subshape_filter is None:
            subshape_filter = lambda s: True

        subshapes = self.subshapes
        for n, lst in subshapes.items():
            if self.shape in lst:
                lst.remove(self.shape)

        named_subshapes = [self.shape]
        for s in op.InterrogateUtils.traverse_all_subshapes(self.shape):
            if subshape_filter(s):
                named_subshapes.append(s)

        subshapes[name] = named_subshapes
        return Part(self.shape, subshapes)

    def do(self, consumer: typing.Callable[[Part], Part]) -> Part:
        """
        Applies the consumer to this part and returns the result.
        :param consumer:
        :return:
        """
        return consumer(self)

    def do_and_add(self, part_modifier: typing.Callable[[Part], Part]) -> Part:
        """
        Applies the consumer to this part, and adds the result using the add method.
        :param part_modifier:
        :return:
        """
        return self.add(part_modifier(self))

    def add(self, *others, sublabel: str = "") -> Part:
        """
        Creates a new part equal to this one, with the other parts added also.
        Subshape maps are merged, and the root shapes are combined using a compound.
        :param others:
        :param sublabel:
        :return:
        """

        builder = OCC.Core.BRep.BRep_Builder()
        result = OCC.Core.TopoDS.TopoDS_Compound()

        builder.MakeCompound(result)
        builder.Add(result, self._shape)

        subshape_list = self.subshapes

        for other in others:
            builder.Add(result, other.shape)

            subshapes_to_add = { sublabel + label: lst for label, lst in other.subshapes.items() }

            subshape_list = Part.merge_subshape_lists(subshape_list, subshapes_to_add)

        return Part(result, subshape_list)

    def pattern(self, range_supplier: typing.Iterable[int], part_modifier: typing.Callable[[int, Part], Part]) -> Part:
        """
        Iterates over the supplied range, adding the result parts.
        part_modifier is provided with this part instance.
        :param range_supplier: provides indices.
        :param part_modifier: modifies the original part according to the index.
        :return: this Part modified according to modifier and i for all i in range
        """
        result = None
        for i in range_supplier:
            intermediate_result = part_modifier(i, self)
            if result is None:
                result = intermediate_result
            else:
                result = result.add(intermediate_result)

        return result

    @staticmethod
    def visualize(*parts):
        if len(parts) == 0:
            raise ValueError("No arguments. Did you mean to call non-static method preview()?")

        import pythonoccutils.cad.gui.visualization as pv

        pv.visualize_parts(*parts)

    def align(self, subshape_name: str = None) -> PartAligner:
        return PartAligner(self, subshape_name)

    @property
    def extents(self):
        """
        :return: the occutils_python.Extents for this Part's root shape.
        """

        if self._extents is None:
            self._extents = op.Extents(self._shape)

        return self._extents

    @property
    def shape(self):
        """
        :return: this Parts root shape
        """
        return self._shape

    @property
    def array(self) -> PartArray:
        return PartArray(self)

    @property
    def save(self) -> PartSave:
        return PartSave(self)

    @property
    def explore(self) -> PartExplore:
        return PartExplore(self)

    @property
    def query(self) -> PartQuery:
        return PartQuery(self, True)

    @property
    def query_shapes(self) -> PartQuery:
        return PartQuery(self, False)

    @property
    def transform(self) -> PartTransformer:
        return PartTransformer(self)

    @property
    def bool(self):
        return PartBool(self)

    @property
    def mirror(self) -> PartMirror:
        return PartMirror(self)

    @property
    def extrude(self):
        return PartExtruder(self)

    @property
    def loft(self):
        return PartLoft(self)

    @property
    def revol(self):
        return PartRevol(self)

    @property
    def make(self):
        return PartMake(self)

    def drill(self, drill: op.Drill):
        drill_ops = drill.get_drill_ops(self.shape)

        union_tools = [Part(s) for d in drill_ops for s in d.shapes if d.is_inverted]
        cut_tools = [Part(s) for d in drill_ops for s in d.shapes if not d.is_inverted]

        result = self

        if len(union_tools) > 0:
            result = result.bool.union(*union_tools)

        if len(cut_tools) > 0:
            result = result.bool.cut(*cut_tools)

        return result

    @property
    def subshapes(self) -> typing.Dict[str, typing.List[OCC.Core.TopoDS.TopoDS_Shape]]:
        """
        :return: All the named subshapes for this Part. Note that named subshapes may not necessarily
        belong to the root shape, in terms of the OCC data structures. Use the #prune method to remove
        orphaned subshapes.
        """

        result = {}
        for n, l in self._named_subshapes.items():
            result[n] = [s for s in l]

        return result

    def compound_subpart(self, name: str):
        """
        :return: A new Part, with root shape as a compound of all subshapes that have the specified name.
        Note that the subshape map is unaltered.
        """
        subshape = self.get_compound(name)

        return Part(subshape, self.subshapes)

    def single_subpart(self, name: str):
        """
        Ensures that a single subshape exists with given name, and returns a Part with it as the root shape.
        subshape map is unaltered.
        """
        subshape = self.get_single(name)

        return Part(subshape, self.subshapes)

    def subpart(self, prefix: str, trim_prefix: bool = True):
        """
        Returns a new Part, whith the same root shape, however the subshape map is culled according to prefix.
        Only entries in the subshape map with the specified prefix are preserved.
        :trim_prefix: if True (default) the prefixes are removed from the subshapes in the result part.
        """

        new_subshapes = {}
        for subshape_label, subshapes in self.subshapes.items():
            if subshape_label.startswith(prefix):

                trimmed_subshape_label = subshape_label[len(prefix):] if trim_prefix else subshape_label


                new_subshapes[trimmed_subshape_label] = [s for s in subshapes]

        return Part(self.shape, new_subshapes)

    @property
    def sew(self):
        return PartSew(self)

    @property
    def fillet(self):
        return PartFilleter(self)

    @property
    def cleanup(self) -> PartCleanup:
        return PartCleanup(self)

    @property
    def inspect(self) -> PartInspect:
        return PartInspect(self)

    def get(self, name: str) -> typing.List[OCC.Core.TopoDS.TopoDS_Shape]:
        """
        :return: The list of shapes in the subshape map that have the given name.
        """

        if name not in self._named_subshapes:
            raise ValueError(f"Unknown subshape: {name}")

        return self._named_subshapes[name]

    def get_single(self, name: str) -> OCC.Core.TopoDS.TopoDS_Shape:
        """
        Checks that there is a single subpart with the given name, and returns it.
        """
        subshapes = self.get(name)
        if len(subshapes) != 1:
            raise ValueError(f"Cannot reduce to single subshape, result of subshape query is: {subshapes}")

        return subshapes[0]

    def get_compound(self, name):
        """
        :return: a compound of the subshapes in the subshape map that have the given name.
        """
        return op.GeomUtils.make_compound(*self.get(name))

    def rename_subshape(self, src_name: str, dst_name: str):
        """
        Checks that the dst_name is free, and renames all subshapes with src_name to dst_name
        :return: a new Part with updated subshape map
        """

        if dst_name in self._named_subshapes:
            raise ValueError("Name already in use")

        updated_subshapes = {}

        for n, s in self._named_subshapes.items():
            if n == src_name:
                n = dst_name

            updated_subshapes[n] = s

        return Part(self.shape, updated_subshapes)

    @staticmethod
    def clone_subshape_map(subshape_map: typing.Dict[str, typing.List[OCC.Core.TopoDS.TopoDS_Shape]]):
        """
        Copies the specified subshape map and returns it. TopoDS_Shapes are not copied.
        """

        result = {}

        for k, v in subshape_map.items():
            result[k] = [s for s in v]

        return result

    @staticmethod
    def merge_subshape_lists(s0: typing.Dict[str, typing.List[OCC.Core.TopoDS.TopoDS_Shape]],
                             s1: typing.Dict[str, typing.List[OCC.Core.TopoDS.TopoDS_Shape]]):

        """
        Combines the subshape lists. Uniqueness of shape instances is not checked, so two identical shapes
        may end up in the subshape map.
        """

        result = {}

        for k, v in s0.items():
            if k not in result:
                result[k] = []

            for s in v:
                result[k].append(s)

        for k, v in s1.items():
            if k not in result:
                result[k] = []

            for s in v:
                result[k].append(s)

        return result

    @staticmethod
    def map_subshape_changes(
            new_shape: OCC.Core.TopoDS.TopoDS_Shape,
            existing_subshapes: typing.Dict[str, typing.List[OCC.Core.TopoDS.TopoDS_Shape]],
            mks: typing.Union[
                OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeShape,
                OCC.Core.BRepTools.BRepTools_History,
                OCC.Core.BRepOffset.BRepOffset_MakeOffset],
            map_is_partner: bool = False,
            map_is_same: bool = False) -> typing.Dict[str, typing.List[OCC.Core.TopoDS.TopoDS_Shape]]:
        """
        Tries to track the history of the subshape map according to the changes applied by mks.
        """

        if isinstance(mks, OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeShape):
            get_is_deleted = mks.IsDeleted
        elif isinstance(mks, OCC.Core.BRepOffset.BRepOffset_MakeOffset):
            get_is_deleted = mks.IsDeleted
        elif isinstance(mks, OCC.Core.BRepTools.BRepTools_History):
            get_is_deleted = mks.IsRemoved
        else:
            raise ValueError("Unsupported MakeShape type")

        result = Part.clone_subshape_map(existing_subshapes)

        if map_is_partner or map_is_same:
            all_new_subshapes = [s for s in op.InterrogateUtils.traverse_all_subshapes(new_shape)]
        else:
            all_new_subshapes = None

        for name in result.keys():
            updated_shape_list = []

            for shape in result[name]:
                if get_is_deleted(shape):
                    continue

                new_subshapes = []
                if map_is_same:
                    new_subshapes += [s for s in all_new_subshapes if s.IsSame(shape)]

                if map_is_partner:
                    new_subshapes += [s for s in all_new_subshapes if s.IsPartner(shape)]

                new_subshapes += [l for l in op.ListUtils.iterate_list(mks.Modified(shape))]
                generated_from = [l for l in op.ListUtils.iterate_list(mks.Generated(shape))]

                replaced_subshapes = [shape] if len(new_subshapes) == 0 else new_subshapes
                replaced_subshapes += generated_from

                replaced_subshapes = [s for s in replaced_subshapes if s not in updated_shape_list]

                updated_shape_list += replaced_subshapes

            result[name] = updated_shape_list

        return result

    def perform_make_shape(self,
                           mks: typing.Union[
                               OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeShape,
                               OCC.Core.BRepOffset.BRepOffset_MakeOffset],
                           **kwargs) -> Part:
        """
        Applies a generic makeshape/offset to this Part and returns a new one with updated subshape maps.
        """

        if not mks.IsDone():
            if isinstance(mks, OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeShape):
                mks.Check()
                raise ValueError()
            else:
                raise ValueError("Make Offset error: " + OCC.Core.BRepOffset.BRepOffset_Error(mks.Error()).name)

        shape = mks.Shape()

        return Part(
            shape,
            Part.map_subshape_changes(shape, self._named_subshapes, mks, **kwargs))


class PartSew:

    def __init__(self, part: Part):
        self._part = part

    def faces(self) -> Part:
        sew = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_Sewing()
        for f in self._part.explore.face.get():
            sew.Add(f.shape)

        sew.Perform()

        sewed_shape = sew.SewedShape()

        return Part(sewed_shape,
                    Part.map_subshape_changes(sew.SewedShape(), self._part.subshapes, sew.GetContext().History()))


class PartValidate:

    def __init__(self, part: Part):
        self._part = part

    def __call__(self, preview_invalid: bool = True) -> Part:
        analyzer = OCC.Core.BRepCheck.BRepCheck_Analyzer(self._part.shape)
        analyzer.Init(self._part.shape)

        invalid_shapes = {}

        if not analyzer.IsValid():
            for subshape in op.InterrogateUtils.traverse_all_subshapes(self._part.shape):
                statuses = [s for s in op.ListUtils.consume_ncollst(analyzer.Result(subshape).Status())]
                statuses = [s for s in statuses if s != OCC.Core.BRepCheck.BRepCheck_Status.BRepCheck_NoError]
                statuses = [OCC.Core.BRepCheck.BRepCheck_Status(s).name for s in statuses]

                if len(statuses) != 0:
                    invalid_shapes[op.SetPlaceableShape(subshape)] = statuses

            error_str = "Part shape is not valid:\n" + \
                        '\n'.join([str(k.shape) + ": " + str(s) for k, s in invalid_shapes.items()])

            if preview_invalid:
                invalid = Part(op.GeomUtils.make_compound(*[s.shape for s in invalid_shapes.keys()]))

                logger.error(error_str)

                invalid.align().x_min_to_max(self._part).add(self._part).preview()

            raise ValueError(error_str)

        return self._part


class PartInspect:

    def __init__(self, part: Part):
        self._part = part

    def face_normal(self, **kwargs) -> typing.Tuple[
            typing.Tuple[float, float, float],
            typing.Tuple[float, float, float]]:

        point, direction = op.InterrogateUtils.face_normal(self._part.make.face().shape, **kwargs)

        return (
            (point.X(), point.Y(), point.Z()),
            (direction.X(), direction.Y(), direction.Z())
        )


class PartSave:

    def __init__(self, part: Part):
        self._part = part

    def single_stl(self, name: str, **kwargs) -> Part:
        filename = f"{name}.stl"
        logger.debug(f"Writing {filename}")
        op.IOUtils.save_shape_stl(self._part.shape, filename, **kwargs)
        return self._part

    def stl_solids(self, name: str, **kwargs) -> Part:
        for i, s in enumerate(self._part.explore.solid.get()):
            filename = f"{name}-{i}.stl"
            logger.debug(f"Writing {filename}")
            op.IOUtils.save_shape_stl(s.shape, filename, **kwargs)

        return self._part


class PartArray:

    def __init__(self, part: Part):
        self._part = part

    def on_verts_of(self, other: Part) -> Part:
        result = []

        for v in other.explore.vertex.get():
            x, y, z = op.InterrogateUtils.vertex_to_xyz(v.shape)
            result.append(self._part.transform.translate(x, y, z))

        return PartFactory.compound(*result)


class PartMake:

    def __init__(self, part: Part):
        self._part = part

    def face(self) -> Part:
        if self._part.shape.ShapeType() == OCC.Core.TopAbs.TopAbs_FACE:
            #nothing to do
            return self._part

        if op.InterrogateUtils.is_compound_of(self._part.shape, OCC.Core.TopAbs.TopAbs_EDGE):
            return op.WireSketcher.from_edges(*set(op.Explorer.edge_explorer(self._part.shape).get()), tolerance=0.001)\
                .close()\
                .get_face_part()

        return self._part.do(lambda p:
                             p.perform_make_shape(
                                OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(p.shape),
                                map_is_partner=True))

    def wire(self) -> Part:

        if self._part.shape.ShapeType() == OCC.Core.TopAbs.TopAbs_WIRE:
            # no action needed
            return self._part
        elif self._part.shape.ShapeType() == OCC.Core.TopAbs.TopAbs_EDGE:
            return self._part.do(lambda p:
                                 p.perform_make_shape(
                                    OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire(p.shape),
                                    map_is_partner=True))
        elif self._part.shape.ShapeType() == OCC.Core.TopAbs.TopAbs_COMPOUND:
            #if any(e.ShapeType() != OCC.Core.TopAbs.TopAbs_EDGE for e in edges):
            #    raise ValueError("Can only create a wire from a compound of edges")

            mkw = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire()
            mkw.Add(op.ListUtils.list([s for s in op.InterrogateUtils.traverse_all_subshapes(self._part.shape) if s.ShapeType() == TopAbs_EDGE]))
            mkw.Build()

            return self._part.perform_make_shape(mkw)
        else:
            raise RuntimeError(f"Cannot convert part shape {self._part.shape} to wire.")


class PartCleanup:

    def __init__(self, part: Part):
        self._part = part

    # note: general cleanup probably required after this as wires may not be connected
    def fuse_wires(self) -> Part:
        if self._part.shape.ShapeType() == OCC.Core.TopAbs.TopAbs_WIRE:
            # nothing to do
            return self._part

        if self._part.shape.ShapeType() != OCC.Core.TopAbs.TopAbs_COMPOUND:
            raise ValueError("Wire fuse can only be performed on a compound of wires")

        wires = [w for w in op.ExploreUtils.iterate_compound(self._part.shape)]

        if any(w.ShapeType() != OCC.Core.TopAbs.TopAbs_WIRE for w in wires):
            raise ValueError("Only wires can be fused")

        mkw = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire()

        edges_to_add = []
        for w in wires:
            edges_to_add += op.Explorer.edge_explorer(w).get()

        mkw.Add(op.ListUtils.list(edges_to_add))

        shape = mkw.Shape()

        return Part(
            shape,
            Part.map_subshape_changes(shape, self._part.subshapes, mkw, map_is_partner=True, map_is_same=True))

    def fix_small_face(self):
        sf = OCC.Core.ShapeFix.ShapeFix_FixSmallFace()
        sf.SetMaxTolerance(0.001)
        sf.Init(self._part.shape)
        sf.Perform()

        shape = sf.Shape()
        return Part(
            shape,
            Part.map_subshape_changes(shape, self._part.subshapes, sf.Context().History()))

    def fix_solid(self):
        sf = OCC.Core.ShapeFix.ShapeFix_Solid()
        sf.SetMaxTolerance(0.001)
        sf.Init(self._part.shape)
        sf.Perform()

        shape = sf.Shape()

        return Part(shape, Part.map_subshape_changes(shape, self._part.subshapes, sf.Context().History()))

    def __call__(self,
                 unify_edges: bool = True,
                 unify_faces: bool = True,
                 concat_b_splines: bool = False,
                 fix_small_face: bool = False) -> Part:
        unif = OCC.Core.ShapeUpgrade.ShapeUpgrade_UnifySameDomain()
        unif.Initialize(self._part.shape, unify_edges, unify_faces, concat_b_splines)
        unif.AllowInternalEdges(True)
        unif.Build()

        shape = unif.Shape()

        return Part(
            shape,
            Part.map_subshape_changes(shape, self._part.subshapes, unif.History()))


class PartExplorer:
    """
    Tried to find a more elegant wrapping of op.Explorer. but it turned out to be easier
    to just re-implement it here.
    """

    def __init__(self, part: Part, shape_type: OCC.Core.TopAbs.TopAbs_ShapeEnum):
        self._part = part
        self._shape_type = shape_type
        self._predicate: typing.Callable[[Part], bool] = lambda p: True
        self._key: typing.Callable[[Part], float] = lambda p: 0.0

    def filter_by(self, predicate: typing.Callable[[Part], bool]):
        old_pred = self._predicate

        self._predicate = lambda e: (old_pred(e) and predicate(e))
        return self

    def order_by(self, key: typing.Callable[[Part], float]):
        self._key = key
        return self

    def get(self) -> typing.List[Part]:
        result = [Part(s, self._part.subshapes) for s in
                  op.ExploreUtils.explore_iterate(self._part.shape, self._shape_type)]
        result = [p for p in result if self._predicate(p)]
        result.sort(key=self._key)
        return result

    def get_single(self) -> Part:
        result = self.get()

        if len(result) != 1:
            raise ValueError(f"Expected result of explore to be a single element, was instead {len(result)}")

        return result[0]

    @staticmethod
    def solid_explorer(part: Part):
        return PartExplorer(part, OCC.Core.TopAbs.TopAbs_ShapeEnum.TopAbs_SOLID)

    @staticmethod
    def face_explorer(part: Part):
        return PartExplorer(part, OCC.Core.TopAbs.TopAbs_ShapeEnum.TopAbs_FACE)

    @staticmethod
    def vertex_explorer(part: Part):
        return PartExplorer(part, OCC.Core.TopAbs.TopAbs_ShapeEnum.TopAbs_VERTEX)

    @staticmethod
    def edge_explorer(part: Part):
        return PartExplorer(part, OCC.Core.TopAbs.TopAbs_ShapeEnum.TopAbs_EDGE)

    @staticmethod
    def wire_explorer(part: Part):
        return PartExplorer(part, OCC.Core.TopAbs.TopAbs_ShapeEnum.TopAbs_WIRE)


class PartExplore:

    def __init__(self, part: Part):
        self._part = part

    def __getattr__(self, item) -> PartExplorer:
        explore_method = getattr(PartExplorer, f"{item}_explorer")

        return explore_method(self._part)


class PartExtruder:

    def __init__(self, part: Part):
        self._part = part

    @staticmethod
    def _solidify_face(face: OCC.Core.TopoDS.TopoDS_Face,
                       amount: float):

        mko = OCC.Core.BRepOffsetAPI.BRepOffsetAPI_MakeOffsetShape()
        mko.PerformBySimple(face, amount / 2)

        offset_surface0 = op.Explorer.face_explorer(mko.Shape()).get()[0]

        reversed_face = face.Reversed()
        mko = OCC.Core.BRepOffsetAPI.BRepOffsetAPI_MakeOffsetShape()
        mko.PerformBySimple(reversed_face, amount / 2)

        offset_surface1 = op.Explorer.face_explorer(mko.Shape()).get()[0]

        return PartFactory.loft([
            op.InterrogateUtils.outer_wire(offset_surface0),
            op.InterrogateUtils.outer_wire(face),
            op.InterrogateUtils.outer_wire(offset_surface1)
        ]).cleanup()

    @staticmethod
    def _solidify_edge(edge: OCC.Core.TopoDS.TopoDS_Edge, amount: float):
        lp = op.InterrogateUtils.line_points(edge)
        tp = op.InterrogateUtils.line_tangent_points(edge)
        circ = gp.gp_Circ(
            gp.gp_Ax2(lp[0], gp.gp_Dir(tp[0])),
            amount / 2)

        pipe_wire = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire(
            OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(circ).Edge()).Shape()

        spine_wire = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire(edge).Wire()

        op.GeomUtils.build_curves_3d(pipe_wire)
        op.GeomUtils.build_curves_3d(spine_wire)

        pipe = Part(pipe_wire)

        pipe = pipe.loft.pipe(
            spine_wire,
            transition_mode=OCC.Core.BRepBuilderAPI.BRepBuilderAPI_TransitionMode.BRepBuilderAPI_RoundCorner) \
            .cleanup.fix_solid() \
            .validate()

        return pipe

    @staticmethod
    def _inflate_faces(shell: OCC.Core.TopoDS.TopoDS_Shape,
                       amount: float,
                       union: bool = True):

        verts = [v for v in op.Explorer.vertex_explorer(shell).get()]
        edges = [e for e in op.Explorer.edge_explorer(shell).get()]
        faces = [f for f in op.Explorer.face_explorer(shell).get()]

        # duplicate a sphere on each vertex
        solid_verts = [PartFactory.sphere(amount / 2)
                    .transform.translate(*op.InterrogateUtils.vertex_to_xyz(v)) for v in verts]

        solid_faces = [PartExtruder._solidify_face(f, amount) for f in faces]

        solid_edges = [PartExtruder._solidify_edge(e, amount) for e in edges]

        if not union:
            return PartFactory.compound(*solid_verts, *solid_edges, *solid_faces)

        result = solid_faces[0]

        for i, sf in enumerate(solid_faces[1:]):
            logger.debug(f"Fusing with bool face {i + 1}/{len(solid_faces)}")
            result = result.bool.union(sf)

        for i, sf in enumerate(solid_edges):
            logger.debug(f"Fusing with bool edge {i + 1}/{len(solid_edges)}")
            try:
                result = result.bool.union(sf)
            except RuntimeError:
                result.bool.union(sf).preview()
                raise

        for i, sf in enumerate(solid_verts):
            logger.debug(f"Fusing with bool vert {i + 1}/{len(solid_verts)}")
            result = result.bool.union(sf)

        return result

    def inflate_faces(self, amount: float, union: bool = True):
        """
        Can be thought of as a 3D version of a 2D offset.
        i.e. Minkowski sum
        e.g. a straight line would become a cylinder with hemisphere
        end caps. The diameter of the cylinder would be equal to amount

        @param union if true, the result will have a bool.union() applied to it. Otherwise, the returned result will
        consist of solids representing the input faces.

        """

        return Part(self._inflate_faces(self._part.shape, amount, union).shape, self._part.subshapes)

    def square_offset(self,
                      amount: float,
                      join_type: OCC.Core.GeomAbs.GeomAbs_JoinType = OCC.Core.GeomAbs.GeomAbs_Tangent,
                      spine: OCC.Core.TopoDS.TopoDS_Face = None,
                      flip_endpoints: bool = False) -> Part:

        if self._part.shape.ShapeType() != OCC.Core.TopAbs.TopAbs_WIRE:
            raise ValueError("Part must be Wire to perform square offset.")

        w1 = self._part.extrude.offset(
            amount=amount,
            join_type=join_type,
            spine=spine,
            is_open_result=True)

        # connect end of w1 to beginning of w
        # connect end of w to beginning of w1
        w_start, w_end = op.InterrogateUtils.wire_points(self._part.shape)
        if flip_endpoints:
            w_end, w_start = w_start, w_end

        w1_start, w1_end = op.InterrogateUtils.wire_points(w1.shape)

        mkw = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire()

        mkw.Add(OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(w1_end, w_start).Shape())
        for e in op.Explorer.edge_explorer(self._part.shape).get():
            mkw.Add(e)

        mkw.Add(OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(w_end, w1_start).Shape())

        for e in op.Explorer.edge_explorer(w1.shape).get():
            mkw.Add(e)

        return self._part.perform_make_shape(mkw)

    def offset(self,
               amount: float,
               join_type: OCC.Core.GeomAbs.GeomAbs_JoinType = OCC.Core.GeomAbs.GeomAbs_Arc,
               is_open_result: bool = False,
               spine: typing.Union[OCC.Core.TopoDS.TopoDS_Face, gp.gp_Ax2] = None) -> Part:

        if isinstance(spine, gp.gp_Ax2):
            spine_pln = gp.gp_Pln(gp.gp_Pnt(*self._part.extents.xyz_mid), spine.Direction())
            spine = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(spine_pln).Shape()

        result: typing.Optional[Part] = None

        for w in op.Explorer.wire_explorer(self._part.shape).get():
            if spine is None:
                mko = OCC.Core.BRepOffsetAPI.BRepOffsetAPI_MakeOffset(w, join_type, is_open_result)
            else:
                mko = OCC.Core.BRepOffsetAPI.BRepOffsetAPI_MakeOffset(spine, join_type, is_open_result)
                mko.AddWire(w)

            mko.Perform(amount)

            if result is None:
                result = self._part.perform_make_shape(mko)
            else:
                result = result.add(self._part.perform_make_shape(mko))

        return result

    def normal_symmetric_prism(self,
                     length: float,
                     first_shape_name: str = None,
                     last_shape_name: str = None,
                     loft_profile_name: str = None) -> Part:

        norm_dir = op.InterrogateUtils.face_normal(self._part.make.face().shape)[1]
        dx = length * norm_dir.X()
        dy = length * norm_dir.Y()
        dz = length * norm_dir.Z()

        return self.symmetric_prism(dx=dx,
                                    dy=dy,
                                    dz=dz,
                                    first_shape_name=first_shape_name,
                                    last_shape_name=last_shape_name,
                                    loft_profile_name=loft_profile_name)

    def symmetric_prism(self, dx: float = 0, dy: float = 0, dz: float = 0,
                        first_shape_name: str = None,
                        last_shape_name: str = None,
                        loft_profile_name: str = None) -> Part:

        p0 = self.prism(dx=dx, dy=dy, dz=dz, first_shape_name=first_shape_name, last_shape_name=last_shape_name, loft_profile_name=loft_profile_name)
        p1 = self.prism(dx=-dx, dy=-dy, dz=-dz, first_shape_name=first_shape_name, last_shape_name=last_shape_name, loft_profile_name=loft_profile_name)

        return p0.bool.union(p1)

    def normal_prism(self,
                     length: float,
                     first_shape_name: str = None,
                     last_shape_name: str = None,
                     loft_profile_name: str = None) -> Part:

        norm_dir = op.InterrogateUtils.face_normal(self._part.make.face().shape)[1]
        dx = length * norm_dir.X()
        dy = length * norm_dir.Y()
        dz = length * norm_dir.Z()

        return self.prism(dx=dx,
                          dy=dy,
                          dz=dz,
                          first_shape_name=first_shape_name,
                          last_shape_name=last_shape_name,
                          loft_profile_name=loft_profile_name)

    def prism(self, dx: float = 0, dy: float = 0, dz: float = 0,
              first_shape_name: str = None,
              last_shape_name: str = None,
              loft_profile_name: str = None) -> Part:
        mkp = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakePrism(self._part.shape, OCC.Core.gp.gp_Vec(dx, dy, dz), True)
        mkp.Build()

        if not mkp.IsDone():
            raise RuntimeError("Prism operation failed.")

        result = self._part.perform_make_shape(mkp)

        extra_shapes = {}

        if first_shape_name is not None:
            extra_shapes[first_shape_name] = extra_shapes.get(first_shape_name, []) + [mkp.FirstShape()]

        if last_shape_name is not None:
            extra_shapes[last_shape_name] = extra_shapes.get(last_shape_name, []) + [mkp.LastShape()]

        if loft_profile_name is not None:
            extra_shapes[loft_profile_name] = extra_shapes.get(loft_profile_name, [])

            for s in op.InterrogateUtils.traverse_all_subshapes(self._part.shape):
                for ss in op.ListUtils.iterate_list(mkp.Generated(s)):
                    if ss not in extra_shapes[loft_profile_name]:
                        extra_shapes[loft_profile_name].append(ss)

        return Part(result.shape, Part.merge_subshape_lists(result.subshapes, extra_shapes))


class PartTransformer:

    def __init__(self, part):
        self._part = part

    def __call__(self, trsf_configurer: typing.Callable[[OCC.Core.gp.gp_Trsf], None]) -> Part:
        trsf = OCC.Core.gp.gp_Trsf()
        trsf_configurer(trsf)
        # todo: for some reason I get errors when copy geom is enabled... In theory copy should be performed
        # to ensure underlying geometry is not modified...
        transformer = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_Transform(self._part.shape, trsf, False)

        return self._part.perform_make_shape(transformer)

    def translate(self, dx: float = 0, dy: float = 0, dz: float = 0) -> Part:
        return self(lambda t: t.SetTranslation(gp.gp_Vec(dx, dy, dz)))

    def rotate(self, ax1: OCC.Core.gp.gp_Ax1, angle: float, offset: typing.Tuple[float, float, float] = None) -> Part:
        if offset is not None:
            ax1 = ax1.Translated(gp.gp_Vec(*offset))

        return self(lambda t: t.SetRotation(ax1, angle))

    def scale(self, factor: float, ox: float = 0, oy: float = 0, oz: float = 0) -> Part:
        return self(lambda t: t.SetScale(gp.gp_Pnt(ox, oy, oz), factor))

    def scale_axis(self, fx: float = 1, fy: float = 1, fz: float = 1) -> Part:
        return self.g_transform_fields(r0c0=fx, r1c1=fy, r2c2=fz)

    def g_transform(self, *trsf: OCC.Core.gp.gp_GTrsf) -> Part:
        result = self._part
        for t in trsf:
            transformer = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_GTransform(result.shape, t, True)
            result = result.perform_make_shape(transformer)

        return result

    def g_transform_fields(self,
                    r0c0: float = 1, r0c1: float = 0, r0c2: float = 0,
                    r1c0: float = 0, r1c1: float = 1, r1c2: float = 0,
                    r2c0: float = 0, r2c1: float = 0, r2c2: float = 1):
        trsf = OCC.Core.gp.gp_GTrsf()
        trsf.SetVectorialPart(OCC.Core.gp.gp_Mat(r0c0, r0c1, r0c2, r1c0, r1c1, r1c2, r2c0, r2c1, r2c2))

        transformer = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_GTransform(self._part.shape, trsf, True)

        return self._part.perform_make_shape(transformer)

    def scale_to_x_span(self, desired_x_span: float, scale_other_axes: bool = False) -> Part:
        x_scale_factor = desired_x_span / self._part.extents.x_span
        other_axes_val = x_scale_factor if scale_other_axes else 1

        return self.g_transform_fields(r0c0=x_scale_factor, r1c1=other_axes_val, r2c2=other_axes_val)

    def scale_to_y_span(self, desired_y_span: float, scale_other_axes: bool = False) -> Part:
        y_scale_factor = desired_y_span / self._part.extents.y_span
        other_axes_val = y_scale_factor if scale_other_axes else 1

        return self.g_transform_fields(r0c0=other_axes_val, r1c1=y_scale_factor, r2c2=other_axes_val)

    def scale_to_z_span(self, desired_z_span: float, scale_other_axes: bool = False) -> Part:
        z_scale_factor = desired_z_span / self._part.extents.z_span
        other_axes_val = z_scale_factor if scale_other_axes else 1

        return self.g_transform_fields(r0c0=other_axes_val, r1c1=other_axes_val, r2c2=z_scale_factor)


class PartBool:

    def __init__(self, part):
        self._part = part

        # resulting part is a union of shapes from both parts

    def union(self, *others: Part, glue: OCC.Core.BOPAlgo.BOPAlgo_GlueEnum = None) -> Part:
        fuse = OCC.Core.BRepAlgoAPI.BRepAlgoAPI_Fuse()
        if glue is not None:
            fuse.SetGlue(glue)

        if len(others) == 0:
            if self._part.shape.ShapeType() != OCC.Core.TopAbs.TopAbs_COMPOUND:
                raise ValueError("Argumentless bool can only be performed on compound shapes, "
                                 f"this shape has type {self._part.shape}")

            tools = [Part(s, self._part.subshapes) for s in op.InterrogateUtils.traverse_direct_subshapes(self._part.shape)]

            return self._boolop(fuse, tools[0:1], tools[1:])

        return self._boolop(fuse, [self._part], [p for p in others])

    def cut(self, *others: Part) -> Part:
        if len(others) == 0:
            raise ValueError("No other parts specified")

        return self._boolop(OCC.Core.BRepAlgoAPI.BRepAlgoAPI_Cut(),
                                        [self._part],
                                        [p for p in others])

    def common(self, *others: Part) -> Part:
        if len(others) == 0:
            raise ValueError("No other parts specified")

        return self._boolop(OCC.Core.BRepAlgoAPI.BRepAlgoAPI_Common(),
                            [self._part],
                            [p for p in others])

    def section(self, *others: Part):
        if len(others) == 0:
            raise ValueError("No other parts specified")

        return self._boolop(OCC.Core.BRepAlgoAPI.BRepAlgoAPI_Section(),
                            [self._part],
                            [p for p in others])

    @staticmethod
    def _boolop(algo: OCC.Core.BRepAlgoAPI.BRepAlgoAPI_BooleanOperation,
                args : typing.List[Part],
                tools : typing.List[Part]) -> Part:

        algo.SetNonDestructive(True)
        algo.SetArguments(op.ListUtils.list([p.shape for p in args]))
        algo.SetTools(op.ListUtils.list([p.shape for p in tools]))

        union_subshapes = args[0].subshapes
        for p in args[1:]:
            union_subshapes = Part.merge_subshape_lists(union_subshapes, p.subshapes)

        for p in tools:
            union_subshapes = Part.merge_subshape_lists(union_subshapes, p.subshapes)

        algo.Build()

        if algo.HasErrors():
            report = algo.GetReport()

            alerts = report.GetAlerts(Message_Gravity.Message_Fail)

            alert_values = []
            while alerts.Size() != 0:
                alert_values.append(alerts.First())
                alerts.RemoveFirst()

            alert_values = [a.GetMessageKey() for a in alert_values]

            raise RuntimeError(f"bool op failed with the following alerts: {alert_values} "
                               f"(operation was performed with args {args}, tools: {tools})")

        shape = algo.Shape()

        return Part(
            shape,
            Part.map_subshape_changes(shape, union_subshapes, algo))


class PartMirror:

    def __init__(self, part: Part):
        self._part = part

    def x(self, union: bool = False):
        result = PartTransformer(self._part).g_transform_fields(r0c0=-1)

        if union:
            return PartBool(self._part).union(result)
        else:
            return result

    def y(self, union: bool = False):
        result = PartTransformer(self._part).g_transform_fields(r1c1=-1)

        if union:
            return PartBool(self._part).union(result)
        else:
            return result

    def z(self, union: bool = False):
        result = PartTransformer(self._part).g_transform_fields(r2c2=-1)

        if union:
            return PartBool(self._part).union(result)
        else:
            return result


class Fillet2dDefaultVertSelector:

    def __init__(self,
                 source_edges: typing.List[OCC.Core.TopoDS.TopoDS_Edge],
                 allowed_edges: typing.List[OCC.Core.TopoDS.TopoDS_Edge]):
        self._source_edges = source_edges
        self._allowed_edges = allowed_edges
        self._default_added_verts = []

    def __call__(self, vert: OCC.Core.TopoDS.TopoDS_Vertex) -> bool:
        # determine number of edges for vert, if 2, then a fillet can be performed

        owner_edges = [e for e in self._allowed_edges if any(v.IsSame(vert) for v in op.Explorer.vertex_explorer(e).get())]

        if len(owner_edges) == 0:
            # vertex not owned by any edge allowed to be filleted
            return False

        connected_edge_count = 0
        for edge in self._source_edges:
            for v in op.Explorer.vertex_explorer(edge).get():
                if v.IsSame(vert):
                    connected_edge_count += 1
                    if connected_edge_count > 1 and not any(v1.IsSame(vert) for v1 in self._default_added_verts):
                        self._default_added_verts.append(vert)
                        return True

        return False


class PartFilleter:

    def __init__(self, part: Part):
        self._part = part

    @staticmethod
    def _guard_mkf2d_fail(mkf: OCC.Core.BRepFilletAPI.BRepFilletAPI_MakeFillet2d):
        if not mkf.IsDone():
            status = mkf.Status()
            import OCC.Core.ChFi2d

            raise RuntimeError("ChFi2d_ConstructionError raised during makefillet: " +
                               OCC.Core.ChFi2d.ChFi2d_ConstructionError(status).name)

    def fillet_edges(self, radius: float, edge_selector: typing.Callable[[OCC.Core.TopoDS.TopoDS_Edge], bool] = None) -> Part:
        if edge_selector is None:
            edge_selector = lambda e: True

        mkf = OCC.Core.BRepFilletAPI.BRepFilletAPI_MakeFillet(self._part.shape)

        for e in op.Explorer.edge_explorer(self._part.shape).filter_by(edge_selector).get():
            mkf.Add(radius, e)

        mkf.Build()

        return self._part.perform_make_shape(mkf)

    def chamfer_edges(self, radius: float, edge_selector: typing.Callable[[OCC.Core.TopoDS.TopoDS_Edge], bool] = None) -> Part:
        if edge_selector is None:
            edge_selector = lambda e: True

        mkf = OCC.Core.BRepFilletAPI.BRepFilletAPI_MakeChamfer(self._part.shape)

        for e in op.Explorer.edge_explorer(self._part.shape).filter_by(edge_selector).get():
            mkf.Add(radius, e)

        mkf.Build()

        return self._part.perform_make_shape(mkf)

    def fillet2d_verts(self,
                       radius: float,
                       vert_selector: typing.Tuple[
                           str,
                           typing.Callable[[OCC.Core.TopoDS.TopoDS_Vertex], bool]] = None) -> Part:
        if isinstance(vert_selector, str):
            verts_to_allow = set(self._part.subshapes[vert_selector])

            def vert_selector(vert):
                return vert in verts_to_allow

        if vert_selector is None:
            all_edges = op.Explorer.edge_explorer(self._part.shape).get()
            vert_selector = Fillet2dDefaultVertSelector(all_edges, all_edges)

        if self._part.shape.ShapeType() == OCC.Core.TopAbs.TopAbs_WIRE:
            result = self._part.make.face()
        else:
            result = self._part

        mkf = OCC.Core.BRepFilletAPI.BRepFilletAPI_MakeFillet2d(result.shape)

        for v in op.Explorer.vertex_explorer(result.shape).filter_by(vert_selector).get():
            mkf.AddFillet(v, radius)

        mkf.Build()

        PartFilleter._guard_mkf2d_fail(mkf)

        # seems like BRepFilletAPI_MakeFillet2D:Modified does a cast of the supplied shape to TopoDS_Edge.
        # this fails when the input shape is a vertex, so strip out any labelled verts. Not ideal, but better
        # than a crash
        result_subshapes = {s:[e for e in ss if e.ShapeType() != OCC.Core.TopAbs.TopAbs_VERTEX] for s,ss in result.subshapes.items()}
        result = Part(result, result_subshapes).perform_make_shape(mkf)

        if self._part.shape.ShapeType() == OCC.Core.TopAbs.TopAbs_WIRE:
            root_face = result.shape
            updated_subshapes = {n: [l1 for l1 in l if l1 != root_face] for n, l in result.subshapes.items()}
            wire = op.Explorer.wire_explorer(result.shape).get()[0]

            return Part(wire, updated_subshapes)
        else:

            return result

    def fillet_faces(self, radius: float, face_selector: typing.Callable[[OCC.Core.TopoDS.TopoDS_Face], None] = None) -> Part:
        if face_selector is None:
            face_selector = lambda e: True

        mkf = OCC.Core.BRepFilletAPI.BRepFilletAPI_MakeFillet(self._part.shape)

        for f in op.Explorer.face_explorer(self._part.shape).filter_by(face_selector).get():
            for e in op.Explorer.edge_explorer(f).get():
                mkf.Add(radius, e)

        mkf.Build()

        return self._part.perform_make_shape(mkf)

    def chamfer_faces(self, radius: float, face_selector: typing.Callable[[OCC.Core.TopoDS.TopoDS_Face], None] = None) -> Part:
        if face_selector is None:
            face_selector = lambda e: True

        mkf = OCC.Core.BRepFilletAPI.BRepFilletAPI_MakeChamfer(self._part.shape)

        for f in op.Explorer.face_explorer(self._part.shape).filter_by(face_selector).get():
            for e in op.Explorer.edge_explorer(f).get():
                mkf.Add(radius, e)

        mkf.Build()

        return self._part.perform_make_shape(mkf)

    def fillet_by_name(self, radius: float, *names: str) -> Part:
        if len(names) == 0:
            raise ValueError("At least one name must be specified.")

        mkf = OCC.Core.BRepFilletAPI.BRepFilletAPI_MakeFillet(self._part.shape)

        shapes = [s for n in names for s in self._part.get(n)] #[s for n in self._part.get(n) for n in names]
        for s in shapes:
            if isinstance(s, OCC.Core.TopoDS.TopoDS_Edge):
                mkf.Add(radius, s)
            else:
                for e in op.Explorer.edge_explorer(s).get():
                    mkf.Add(radius, e)

        mkf.Build()

        return self._part.perform_make_shape(mkf)

    def fillet_edges_by_query(self, radius, query: str):
        to_fillet = set(op.Explorer.edge_explorer(self._part.query(query).shape).get())

        return self.fillet_edges(radius, lambda e: e in to_fillet)

    def fillet2d_by_name(self, radius: float, *names: str) -> Part:
        if len(names) == 0:
            raise ValueError("At least one name must be specified.")

        edge_list = []

        shapes = [s for n in names for s in self._part.get(n)] #[s for n in self._part.get(n) for n in names]
        for s in shapes:
            if s.ShapeType() != OCC.Core.TopAbs.TopAbs_EDGE:
                raise ValueError("Shape is not an edge")

            edge_list.append(s)

        return self.fillet2d_verts(radius, vert_selector=Fillet2dDefaultVertSelector(
            op.Explorer.edge_explorer(self._part.shape).get(),
            edge_list))

    def __call__(self, radius: float) -> Part:
        return self.fillet_edges(radius)


class PartAligner:

    align_re = re.compile("[xyz]+_(min|mid|max)_to(_(min|mid|max))?")

    def __init__(self, part, subshape_name: str = None):
        self._part = part
        self._subshape_name = subshape_name

    def com_to_origin(self) -> Part:
        com = op.InterrogateUtils.center_of_mass(self._part.shape)
        return self._part.transform.translate(-com[0], -com[1], -com[2])

    def com(self, other: Part):
        other_com = op.InterrogateUtils.center_of_mass(other.shape)
        return self.com_to_origin().transform.translate(*other_com)

    def __getattr__(self, item: str) -> typing.Callable[[typing.Union[Part, OCC.Core.TopoDS.TopoDS_Shape]], Part]:
        if not PartAligner.align_re.fullmatch(item):
            raise ValueError("Invalid attribute")

        align_args = item.split('_')

        axes = align_args[0]
        source_part = align_args[1]

        source_extents = self._part.extents \
            if self._subshape_name is None \
            else op.Extents(op.GeomUtils.make_compound(*self._part.get(self._subshape_name)))

        if len(align_args) < 4:
            def result_fn_coords(**kwargs) -> Part:
                align_source = getattr(source_extents, f"xyz_{source_part}")

                dest_x = kwargs.get("x", 0)
                dest_y = kwargs.get("y", 0)
                dest_z = kwargs.get("z", 0)

                dx = dest_x - align_source[0] if "x" in axes else 0
                dy = dest_y - align_source[1] if "y" in axes else 0
                dz = dest_z - align_source[2] if "z" in axes else 0

                return self._part.transform.translate(dx, dy, dz)

            return result_fn_coords


        dest_part = align_args[3]

        def result_fn(dest_shape: typing.Union[OCC.Core.TopoDS.TopoDS_Shape, Part]) -> Part:
            if not isinstance(dest_shape, OCC.Core.TopoDS.TopoDS_Shape):
                # must be a part
                dest_shape = dest_shape.shape

            align_source = getattr(source_extents, f"xyz_{source_part}")
            align_dest = getattr(op.Extents(dest_shape), f"xyz_{dest_part}")

            dx = align_dest[0] - align_source[0] if "x" in axes else 0
            dy = align_dest[1] - align_source[1] if "y" in axes else 0
            dz = align_dest[2] - align_source[2] if "z" in axes else 0

            return self._part.transform.translate(dx, dy, dz)

        return result_fn


class PartLoft:
    
    def __init__(self, part: Part):
        self._part = part

    def pipe(self,
             spine: typing.Union[OCC.Core.TopoDS.TopoDS_Wire, Part],
             transition_mode: OCC.Core.BRepBuilderAPI.BRepBuilderAPI_TransitionMode =
                OCC.Core.BRepBuilderAPI.BRepBuilderAPI_TransitionMode.BRepBuilderAPI_RightCorner,
             bi_normal_mode: gp.gp_Dir = None):
        profile = self._part.make.wire().shape

        if isinstance(spine, Part):
            spine = spine.make.wire().shape

        mps = OCC.Core.BRepOffsetAPI.BRepOffsetAPI_MakePipeShell(spine)
        mps.Add(profile)
        mps.SetTransitionMode(transition_mode)

        if bi_normal_mode is not None:
            mps.SetMode(bi_normal_mode)

        if not mps.IsReady():
            raise RuntimeError("PipeBuilder failure")

        try:
            mps.Build()
        except RuntimeError as e:
            raise RuntimeError("Exception during build, have you run BuildCurves3d ?", e)

        mps.MakeSolid()

        return self._part.perform_make_shape(mps)

    def between(self, from_name: str, to_name: str, fix_small_faces: bool=True, **kwargs):
        from_shape = self._part.get_single(from_name)
        to_shape = self._part.get_single(to_name)

        result = PartFactory.loft([
            op.InterrogateUtils.get_outer_wire(from_shape),
            op.InterrogateUtils.get_outer_wire(to_shape)
        ], first_shape_name=from_name, last_shape_name=to_name, **kwargs)

        if fix_small_faces:
            fsf = OCC.Core.ShapeFix.ShapeFix_FixSmallFace()
            fsf.Init(result.shape)
            fsf.Perform()

            shape = fsf.Shape()

            new_subshapes = Part.map_subshape_changes(
                shape,
                result.subshapes,
                fsf.Context().History())

            result = Part(shape, new_subshapes)

        return result


class PartRevol:

    def __init__(self, part):
        self._part = part

    def about(self, ax: gp.gp_Ax1, radians: float):
        make_shape = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeRevol(self._part.shape, ax, radians)
        return self._part.perform_make_shape(make_shape)


class ThreadSpec:

    METRIC_THREAD_TABLE = {
        "M1.6": (0.35, 1.6),
        "M1.8": (0.35, 1.8),
        "M2": (0.4, 2),
        "M2.2": (0.45, 2.2),
        "M2.5": (0.45, 2.5),
        "M3": (0.5, 3),
        "M3.5": (0.6, 3.5),
        "M4": (0.7, 4),
        "M4.5": (0.75, 4.5),
        "M5": (0.8, 5),
        "M6": (1, 6),
        "M7": (1, 7),
        "M8": (1.25, 8),
        "M10": (1.5, 10),
        "M12": (1.75, 12),
        "M14": (2, 14),
        "M16": (2, 16),
        "M18": (2.5, 18),
        "M20": (2.5, 20),
        "M22": (2.5, 22),
        "M24": (3, 24),
        "M27": (3, 27),
        "M30": (3.5, 30),
        "M33": (3.5, 33),
        "M36": (4, 36),
        "M39": (4, 39),
        "M42": (4.5, 42),
        "M45": (4.5, 45),
        "M48": (5, 48),
        "M52": (5, 52),
        "M56": (5.5, 56),
        "M60": (5.5, 60),
        "M64": (6, 64),
        "M68": (6, 68)
    }

    def __init__(self, pitch: float, basic_major_diameter: float):
        self.basic_major_diameter = basic_major_diameter
        self.pitch = pitch

        self.is_exterior = False
        self.fundamental_triangle_height = self.pitch * math.sqrt(3) / 2

        self.d1_basic_minor_diameter = self.basic_major_diameter - 1.25 * self.fundamental_triangle_height
        self.d2_basic_pitch_diameter = self.basic_major_diameter - 0.75 * self.fundamental_triangle_height

    @staticmethod
    def metric(name: str) -> ThreadSpec:
        name_upper = name.upper()

        if name_upper not in ThreadSpec.METRIC_THREAD_TABLE:
            raise ValueError(f"Cannot find thread profile {name} in thread table.")

        return ThreadSpec(*ThreadSpec.METRIC_THREAD_TABLE[name_upper])


class PartFactory:

    @staticmethod
    def arrange(*parts: Part, spacing: float=0):
        part = parts[0]

        result = [part]

        x_max = part.extents.x_max

        for p in parts[1:]:
            p = p.transform.translate(dx=x_max - p.extents.x_min)
            result.append(p)
            x_max = p.extents.x_max + spacing

        return PartFactory.compound(*result)

    @staticmethod
    def face(outer_wire: Part, *inner_wires: Part):

        outer_wire = outer_wire.make.wire().shape

        mkf = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(outer_wire)

        for w in inner_wires:
            mkf.Add(w.make.wire().shape.Reversed())

        return Part(mkf.Face())

    @staticmethod
    def thread(thread_spec: typing.Union[ThreadSpec, str], exterior: bool, length: float):
        if isinstance(thread_spec, str):
            thread_spec = ThreadSpec.metric(thread_spec)

        #overshoot and trim down excess thread
        build_length = length + 4 * thread_spec.pitch

        l2 = thread_spec.pitch * 1 / 16
        l1 = thread_spec.pitch * 1 / 8

        # first generate the fundamental triangle
        if exterior:
            tri = op.WireSketcher(0, 0, -l1)\
                .line_to(z=-thread_spec.pitch + l1)\
                .line_to(x=thread_spec.fundamental_triangle_height / 4, is_relative=True)\
                .line_to(x=thread_spec.fundamental_triangle_height * 7/8, z=-thread_spec.pitch /2 - l2)\
                .line_to(z=2 * l2, is_relative=True)\
                .line_to(x=thread_spec.fundamental_triangle_height / 4, z=-l1) \
                .close()\
                .get_wire_part()
        else:
            tri = op.WireSketcher(thread_spec.fundamental_triangle_height / 4, 0, l1)\
                .line_to(z=-2 * l1, is_relative=True)\
                .line_to(x=thread_spec.fundamental_triangle_height * 7 / 8, z=-thread_spec.pitch / 2 + l2)\
                .line_to(x=thread_spec.fundamental_triangle_height / 8, is_relative=True)\
                .line_to(z=thread_spec.pitch - 2 * l2, is_relative=True)\
                .line_to(x=-thread_spec.fundamental_triangle_height / 8, is_relative=True)\
                .close()\
                .get_wire_part()

        tri = tri\
            .transform.translate(dx=thread_spec.d1_basic_minor_diameter / 2 - thread_spec.fundamental_triangle_height / 4)\
            .transform.translate(dz=-thread_spec.pitch)

        n_turns = build_length / thread_spec.pitch

        helix = PartFactory.helix(
            height=build_length,
            radius=thread_spec.d2_basic_pitch_diameter / 2,
            n_turns=n_turns)

        thread = tri.loft.pipe(helix, bi_normal_mode=gp.gp_DZ())

        # use end caps to slice the thread start and end
        cut_box = PartFactory.box_surrounding(thread, 1, 1, 0)
        cut_bottom = cut_box \
            .transform.translate(dz=-cut_box.extents.z_max)

        cut_top = cut_box\
            .transform.translate(dz=-cut_box.extents.z_min + length)

        thread = thread.bool.cut(cut_bottom)

        thread = thread.bool.cut(cut_top)

        return thread


    @staticmethod
    def trapezoid(height: float, l_top: float, l_bottom: float):
        return op.WireSketcher(-l_bottom / 2, 0, 0)\
            .line_to(x=l_bottom / 2)\
            .line_to(x=l_top / 2, y=height)\
            .line_to(x=-l_top / 2)\
            .close()\
            .get_wire_part()

    @staticmethod
    def helix(height: float,
              radius: float,
              n_turns: float):

        end_angle = n_turns * 2.0 * math.pi

        pnt_start = OCC.Core.gp.gp_Pnt2d(0, 0)
        pnt_end = OCC.Core.gp.gp_Pnt2d(end_angle, height)

        lin = OCC.Core.GCE2d.GCE2d_MakeSegment(pnt_start, pnt_end).Value()

        surf = Geom_CylindricalSurface(gp.gp_Ax3(gp.gp_XOY()), radius)

        edge = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(lin, surf).Edge()
        op.GeomUtils.build_curves_3d(edge)

        return Part(OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire(edge).Wire())

    @staticmethod
    def cone(r1: float, r2: float, height: float):
        return Part(OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeCone(r1, r2, height).Shape())

    @staticmethod
    def polygon(radius: float, segments: int):
        d_theta = 2.0 * math.pi / segments

        ws = op.WireSketcher(
            radius * math.cos(0),
            radius * math.sin(0),
            0)

        for i in range(1, segments):
            ws.line_to(radius * math.cos(d_theta * i), radius * math.sin(d_theta * i), 0)

        return ws.close().get_wire_part()

    @staticmethod
    def capsule(center_distance: float, diameter: float) -> Part:
        return op.WireSketcher().line_to(x=center_distance, is_relative=True)\
            .circle_arc_to(0, -diameter, 0, radius=diameter / 2, is_relative=True, direction=gp.gp_Dir(0, 0, -1))\
            .line_to(x=-center_distance, is_relative=True)\
            .circle_arc_to(0, diameter, 0, radius=diameter / 2, is_relative=True, direction=gp.gp_Dir(0, 0, -1))\
            .close()\
            .get_face_part()\
            .align().com_to_origin()


    @staticmethod
    def compound(*parts: Part) -> Part:
        if len(parts) == 0:
            raise ValueError("No parts specified for compound")

        p = parts[0]

        for p2 in parts[1:]:
            p = p.add(p2)

        return p

    @staticmethod
    def union(*parts: Part) -> Part:
        if len(parts) == 0:
            raise ValueError("At least one part required")

        start = parts[0]

        if len(parts) > 1:
            start = start.bool.union(*parts[1:])

        return start

    @staticmethod
    def text(text: str,
             font_name: str,
             size: float,
             font_aspect: int = OCC.Core.Addons.Font_FontAspect_Regular,
             is_composite_curve: bool = False):

        return Part(OCC.Core.Addons.text_to_brep(text, font_name, font_aspect, size, is_composite_curve))

    @staticmethod
    def hex_lattice(rows: int, cols: int, hex_radius: float = 0.9, grid_radius: float = 1) -> Part:
        base_shape = Part(op.GeomUtils.regular_polygon(hex_radius, 6)).make.face()

        part = None

        col_spacing = grid_radius * 3
        row_spacing = math.sqrt(3) * 0.5 * grid_radius

        for row in range(0, rows):
            y = row_spacing * row

            x_offs = 0 if row % 2 == 0 else 3 * grid_radius * 0.5
            for c in range(0, cols):
                x = col_spacing * c + x_offs

                subpart = base_shape.transform.translate(dx=x, dy=y)

                part = subpart if part is None else part.add(subpart)

        return part

    @staticmethod
    def lattice(rows: int, cols: int, diag_a: bool=False, diag_b: bool=False) -> Part:
        wires = []

        # create coordinate array
        for r in range(0, rows):
            for c in range(0, cols):
                wires.append(op.WireSketcher(c, r, 0).line_to(x=1, is_relative=True).get_wire())
                wires.append(op.WireSketcher(c, r, 0).line_to(y=1, is_relative=True).get_wire())

                if diag_a:
                    wires.append(op.WireSketcher(c, r, 0).line_to(x=1, y=1, is_relative=True).get_wire())

                if diag_b:
                    wires.append(op.WireSketcher(c, r + 1, 0).line_to(x=1, y=-1, is_relative=True).get_wire())

        for r in range(0, rows):
            wires.append(op.WireSketcher(cols, r, 0).line_to(y=1, is_relative=True).get_wire())

        for c in range(0, cols):
            wires.append(op.WireSketcher(c, rows, 0).line_to(x=1, is_relative=True).get_wire())

        return Part(op.BoolUtils.incremental_fuse(wires))

    @staticmethod
    def sphere(radius: float) -> Part:
        return Part(OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeSphere(radius).Shape())

    @staticmethod
    def circle(radius: float) -> Part:
        edge = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeEdge(gp.gp_Circ(gp.gp_XOY(), radius)).Edge()

        wire = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeWire(edge).Wire()

        return Part(wire)

    @staticmethod
    def right_angle_triangle(
            hypot: float,
            angle: float,
            h_label: str = None,
            adj_label: str = None,
            op_label: str = None,
            pln: gp.gp_Ax2 = None) -> Part:

        if pln is None:
            pln = gp.gp_XOY()

        adj_length = hypot * math.cos(angle)
        op_length = hypot * math.sin(angle)

        xdir = pln.XDirection()
        ydir = pln.YDirection()

        da = gp.gp_Vec(xdir).Normalized().Scaled(adj_length)
        do = gp.gp_Vec(ydir).Normalized().Scaled(op_length)

        return op.WireSketcher()\
            .line_to(x=da.X(), y=da.Y(), z=da.Z(), label=adj_label)\
            .line_to(x=do.X(), y=do.Y(), z=do.Z(), label=op_label)\
            .close(label=h_label)\
            .get_wire_part()

    @staticmethod
    def vertex(x: float = 0, y: float = 0, z: float = 0, name: str = None):
        shape = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeVertex(OCC.Core.gp.gp_Pnt(x, y, z)).Shape()

        if name is None:
            return Part(shape)
        else:
            return Part(shape, {name: [shape]})

    @staticmethod
    def x_line(length: float, symmetric: bool = False):
        return op.WireSketcher().line_to(x=length).get_wire_part()\
            .do(lambda p: p.align().x_mid_to((0, 0, 0)) if symmetric else p)

    @staticmethod
    def y_line(length: float, symmetric: bool = False):
        return op.WireSketcher().line_to(y=length).get_wire_part()\
            .do(lambda p: p.align().y_mid_to((0, 0, 0)) if symmetric else p)

    @staticmethod
    def z_line(length: float, symmetric: bool = False):
        return op.WireSketcher().line_to(z=length).get_wire_part()\
            .do(lambda p: p.align().z_mid_to((0, 0, 0)) if symmetric else p)

    @staticmethod
    def loft(wires_or_faces: typing.List[typing.Union[Part, OCC.Core.TopoDS.TopoDS_Wire, OCC.Core.TopoDS.TopoDS_Face]],
             is_solid: bool = True,
             is_ruled: bool = True,
             pres3d: bool = 1.0e-6,
             first_shape_name: str = None,
             last_shape_name: str = None,
             loft_profile_name: str = None):

        ts = OCC.Core.BRepOffsetAPI.BRepOffsetAPI_ThruSections(is_solid, is_ruled, pres3d)

        if len(wires_or_faces) < 2:
            raise ValueError("Must specify at least 2 wires")

        for w in wires_or_faces:
            if isinstance(w, Part):
                w = w.shape

            ts.AddWire(op.InterrogateUtils.outer_wire(w))

        ts.Build()

        named_subshapes = {}
        if first_shape_name is not None:
            named_subshapes[first_shape_name] = [ts.FirstShape()]

        if last_shape_name is not None:
            named_subshapes[last_shape_name] = [ts.LastShape()]

        if loft_profile_name is not None:
            named_subshapes[loft_profile_name] = []
            for w in wires_or_faces:
                for e in op.Explorer.edge_explorer(w).get():
                    for f in op.ListUtils.iterate_list(ts.Generated(e)):
                        named_subshapes[loft_profile_name] += [f]

        return Part(ts.Shape(), named_subshapes)

    @staticmethod
    def square_centered(dx: float,
                        dy: float,
                        x_min_name: typing.Optional[str] = None,
                        x_max_name: typing.Optional[str] = None,
                        y_min_name: typing.Optional[str] = None,
                        y_max_name: typing.Optional[str] = None,
                        fill_face: bool = True) -> Part:

        hdx = 0.5 * dx
        hdy = 0.5 * dy

        ws = op.WireSketcher(gp.gp_Pnt(-hdx, -hdy, 0))\
            .line_to(x=hdx, y=-hdy, label=y_min_name, is_relative=False)\
            .line_to(x=hdx, y=hdy, label=x_max_name, is_relative=False)\
            .line_to(x=-hdx, y=hdy, label=y_max_name, is_relative=False)\
            .close(label=x_min_name)

        if fill_face:
            return ws.get_face_part()
        else:
            return ws.get_wire_part()

    @staticmethod
    def cylinder(radius: float,
                 height: float,
                 top_wire_name: str = None,
                 bottom_wire_name: str = None) -> Part:

        height_abs = abs(height)

        cylinder = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeCylinder(radius, height_abs).Shape()
        wires = op.Explorer(cylinder, OCC.Core.TopAbs.TopAbs_WIRE) \
            .filter_by(lambda w: op.Extents(w).z_span < height_abs) \
            .order_by(lambda s: op.Extents(s).z_mid) \
            .get()

        named_subshapes = {}
        if top_wire_name is not None:
            named_subshapes[top_wire_name] = [wires[1]]

        if bottom_wire_name is not None:
            named_subshapes[bottom_wire_name] = [wires[0]]

        result = Part(cylinder, named_subshapes)

        if height < 0:
            result = result.transform.translate(dz=height)

        return result

    @staticmethod
    def box(dx: float, dy: float, dz: float,
            x_min_face_name: str = None,
            x_max_face_name: str = None,
            y_min_face_name: str = None,
            y_max_face_name: str = None,
            z_min_face_name: str = None,
            z_max_face_name: str = None) -> Part:

        mkbox = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeBox(dx, dy, dz)

        named_parts = {}
        if x_min_face_name is not None:
            named_parts[x_min_face_name] = [mkbox.BackFace()]
        if x_max_face_name is not None:
            named_parts[x_max_face_name] = [mkbox.FrontFace()]

        if y_min_face_name is not None:
            named_parts[y_min_face_name] = [mkbox.LeftFace()]
        if y_max_face_name is not None:
            named_parts[y_max_face_name] = [mkbox.RightFace()]

        if z_min_face_name is not None:
            named_parts[z_min_face_name] = [mkbox.BottomFace()]
        if z_max_face_name is not None:
            named_parts[z_max_face_name] = [mkbox.TopFace()]

        try:
            shape = mkbox.Shape()
        except RuntimeError as e:
            print(f"Failure to create box with dimensions: {dx}, {dy}, {dz}")
            raise e

        return Part(mkbox.Shape(), named_parts)

    @staticmethod
    def box_centered(dx: float, dy: float, dz: float,
                     x_min_face_name: str = None,
                     x_max_face_name: str = None,
                     y_min_face_name: str = None,
                     y_max_face_name: str = None,
                     z_min_face_name: str = None,
                     z_max_face_name: str = None) -> Part:

        result_part = PartFactory.box(dx, dy, dz,
                                 x_min_face_name=x_min_face_name,
                                 x_max_face_name=x_max_face_name,
                                 y_min_face_name=y_min_face_name,
                                 y_max_face_name=y_max_face_name,
                                 z_min_face_name=z_min_face_name,
                                 z_max_face_name=z_max_face_name)

        return result_part.transform.translate(-dx / 2, -dy / 2, -dz / 2)

    @staticmethod
    def box_surrounding(part: Part,
                        x_clearance: float = 0,
                        y_clearance: float = 0,
                        z_clearance: float = 0) -> Part:

        return PartFactory.box(
                dx=part.extents.x_span + 2 * x_clearance,
                dy=part.extents.y_span + 2 * y_clearance,
                dz=part.extents.z_span + 2 * z_clearance)\
            .align().xyz_mid_to_mid(part)


"""
=========================
Part query section
=========================
"""

Shape = OCC.Core.TopoDS.TopoDS_Shape
Edge = OCC.Core.TopoDS.TopoDS_Edge
Wire = OCC.Core.TopoDS.TopoDS_Wire
Shell = OCC.Core.TopoDS.TopoDS_Shell
Solid = OCC.Core.TopoDS.TopoDS_Solid
Compound = OCC.Core.TopoDS.TopoDS_Compound


class QuantityResolver:

    def get_quantity(self, *args: Shape) -> typing.List[Shape]:
        raise NotImplementedError()


class ExactQuantityResolver(QuantityResolver):

    def __init__(self, amount: int):
        self._amount = amount

    def get_quantity(self, *args: Shape) -> typing.List[Shape]:
        if len(args) != self._amount:
            raise ValueError("Unexpected args number")

        return [s for s in args]


class SliceQuantityResolver(QuantityResolver):

    def __init__(self, index_from: typing.Optional[int], index_to: typing.Optional[int]):
        self._index_from = index_from
        self._index_to = index_to

    def get_quantity(self, *args: Shape) -> typing.List[Shape]:
        return [s for s in args[self._index_from:self._index_to]]


class AllQuantityResolver(QuantityResolver):

    def get_quantity(self, *args: Shape) -> typing.List[Shape]:
        return [s for s in args]


class ShapeSpecifier:

    SHAPE_TYPES = {
        "s": OCC.Core.TopAbs.TopAbs_SHAPE,
        "v": OCC.Core.TopAbs.TopAbs_VERTEX,
        "e": OCC.Core.TopAbs.TopAbs_EDGE,
        "w": OCC.Core.TopAbs.TopAbs_WIRE,
        "f": OCC.Core.TopAbs.TopAbs_FACE,
        "sh": OCC.Core.TopAbs.TopAbs_SHELL,
        "so": OCC.Core.TopAbs.TopAbs_SOLID,
        "c": OCC.Core.TopAbs.TopAbs_COMPOUND
    }

    def __init__(self, shape_name: str):
        if not shape_name in ShapeSpecifier.SHAPE_TYPES:
            raise ValueError("Unrecognized shape type")

        self._expected_shape_type = ShapeSpecifier.SHAPE_TYPES[shape_name]

    def get_shapes(self, part: Part) -> typing.List[typing.Any]:
        """
        :return: The set of shapes to be considered for filtering
        """

        return op.Explorer(part.shape, self._expected_shape_type).get()


class ShapeFilter:

    def filter(self, part: Part, filter_inputs: typing.Generator[Shape, None, None]) -> \
            typing.Generator[Shape, None, None]:
        raise NotImplementedError()


class ShapeLabelledFilter(ShapeFilter):

    def __init__(self, label: str):
        self._is_prefix = label.endswith("*")

        if self._is_prefix:
            self._label = label[:-1]
        else:
            self._label = label

    def filter(self, part: Part, filter_inputs: typing.Generator[Shape, None, None]) -> \
            typing.Generator[Shape, None, None]:

        candidate_shapes = set()

        # apply the label filter
        if self._is_prefix:
            part = part.subpart(self._label)
            for l, subshapes in part.subshapes.items():
                for s in subshapes:
                    candidate_shapes.add(s)
        else:
            subshapes = part.subshapes

            if not self._label in subshapes.keys():
                raise ValueError(f"Label: \"{self._label}\" is not present in the part.")

            candidate_shapes = {s for s in subshapes[self._label]}

        for s in filter_inputs:
            if s in candidate_shapes:
                yield s


class ShapeValidation:

    def __init__(self, quantity_resolver: QuantityResolver, shape_specifier: ShapeSpecifier):
        self._quantity_resolver = quantity_resolver
        self._shape_specifier = shape_specifier

    def get_shapes(self, part: Part) -> typing.List[Shape]:
        shapes = self._shape_specifier.get_shapes(part)
        return self._quantity_resolver.get_quantity(*shapes)


class SubshapeResolver:

    def __init__(self,
                 shape_speicifer: ShapeSpecifier,
                 quantity_resolver: QuantityResolver,
                 filters: typing.List[ShapeFilter]):
        self._shape_specifier = shape_speicifer
        self._quantity_resolver = quantity_resolver
        self._filters = [s for s in filters]

    def get_shapes(self, part: Part) -> typing.List[Shape]:
        typed_shapes = self._shape_specifier.get_shapes(part)

        filtered_shapes = typed_shapes
        for shape_filter in self._filters:
            filtered_shapes = [s for s in shape_filter.filter(part, (fs for fs in filtered_shapes))]

        return self._quantity_resolver.get_quantity(*filtered_shapes)


# noinspection PyMethodMayBeStatic
class SubshapeResolverVisitor(parsimonious.NodeVisitor):

    def visit_subshape_resolver(self, node, visited_children):
        quantity_resolver, shape_specifier = visited_children[0]

        shape_filters = visited_children[1]

        return SubshapeResolver(shape_specifier, quantity_resolver, shape_filters)

    def visit_quantity_exact(self, node, visited_children):
        return ExactQuantityResolver(int(node.text))

    def visit_quantity_all(self, node, visited_children):
        return AllQuantityResolver()

    def visit_quantity_slice(self, node, visited_children):
        r0, r1 = node.children[1].text.split(":")
        return SliceQuantityResolver(
            int(r0) if r0.isdigit() else None,
            int(r1) if r1.isdigit() else None
        )

    def visit_quantity_resolver(self, node, visited_children):
        # can consist of multiple types, all implementing QuantityResolver
        return visited_children[0]

    def visit_shape_specifier(self, node, visited_children):
        return ShapeSpecifier(node.text)

    def visit_filter(self, node, visited_children):
        # can consist of multiple types, all implementing ShapeFilter
        return visited_children[0]

    def visit_filters(self, node, visited_children):
        return [f for c in visited_children for f in c if isinstance(f, ShapeFilter)]

    def visit_label_filter(self, node, visited_children):
        return ShapeLabelledFilter(node.children[1].text)

    def generic_visit(self, node, visited_children):
        """ The generic visit method. """
        return visited_children or node


class PartQuery:

    grammar = Grammar(
        """
        subshape_resolver               = validation_part filters

        validation_part                 = quantity_resolver shape_specifier

        filters                         = ("," filter)*
        filter                          = label_filter

        # p: modified in previous operation
        # l: has label
        label_filter                    = "l(" (label "*"?) ")"

        shape_specifier                 = "v" / "e" / "w" / "f" / "sh" / "so" / "c" / "s"

        quantity_resolver               = quantity_slice / quantity_exact / quantity_all

        quantity_slice                  = "[" ((integer ":" integer) / (":" integer) / (integer ":") / integer / ":") "]"
        quantity_exact                  = abs_integer / digit
        quantity_all                    = "*"

        label                           = ~"[a-z]"i (alphanum / "_" / "/")+
        alphanum                        = ~"[a-z 0-9]+"i
        integer                         = "-"? abs_integer
        abs_integer                     = ((~"[1-9]" digit+) / digit)
        digit                           = ~"[0-9]"
        """)

    def __init__(self, part: Part, to_subpart: bool):
        self._part = part
        self._to_subpart = to_subpart

    def __call__(self, query: str):
        syntax_tree = PartQuery.grammar.parse(query)

        visitor = SubshapeResolverVisitor()

        subshape_resolver: SubshapeResolver = visitor.visit(syntax_tree)

        shapes = subshape_resolver.get_shapes(self._part)

        if not self._to_subpart:
            return shapes
        else:
            return Part(op.GeomUtils.make_compound(*shapes), self._part.subshapes)


class PartSelectionResolver:

    SHAPE_TYPE_LOOKUP = {v: k for k, v in ShapeSpecifier.SHAPE_TYPES.items()}

    def __init__(self, part: Part, *selection: OCC.Core.TopoDS.TopoDS_Shape):
        self._part = part
        self._selection = {}

        # group the selections into types
        for s in selection:
            self._selection[s.ShapeType()] = self._selection.get(s.ShapeType(), []) + [s]

        # reverse lookup for shape labels
        self._label_cache = {}
        for label, shape_list in self._part.subshapes.items():
            for shape in shape_list:
                self._label_cache[shape] = label

    def get_suggested_selections(self) -> typing.Generator[str, None, None]:
        for shape_type, shapes in self._selection.items():
            shapes = set(shapes)

            shape_type_query = PartSelectionResolver.SHAPE_TYPE_LOOKUP[shape_type]

            all_shapes: typing.List[OCC.Core.TopoDS.TopoDS_Shape] = self._part.query_shapes(f"*{shape_type_query}")

            if set(all_shapes) == set(shapes):
                yield f"*{shape_type_query}"

            if shapes.issubset(all_shapes):
                for i0, i1 in PartSelectionResolver.get_index_ranges(shapes, all_shapes):
                    if i1 is not None:
                        yield f"{shape_type_query}[{i0}:{i1 + 1}]"
                    else:
                        yield f"{shape_type_query}[{i0}]"

            shape_labels = set(self._label_cache[s] for s in shapes if s in self._label_cache.keys())
            if len(shape_labels) == 1:
                yield f"{shape_type_query},l({shape_labels.pop()})"

    @staticmethod
    def get_index_ranges(sublist: typing.List,
                         superlist: typing.List) -> typing.Generator[typing.Tuple[int, typing.Optional[int]], None, None]:
        indices = [superlist.index(s) for s in sublist]
        indices.sort()

        while len(indices) > 0:
            start_index = indices.pop(0)
            end_index = start_index
            while len(indices) > 0 and indices[0] == end_index + 1:
                end_index = indices.pop(0)

            # by this point, have consumed all contiguous elements
            if start_index == end_index:
                yield start_index, None
            else:
                yield start_index, end_index
