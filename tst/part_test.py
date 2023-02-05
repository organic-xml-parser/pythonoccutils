import math
import unittest

import OCC
import OCC.Core.BOPAlgo
import OCC.Core.BRepAlgoAPI
import OCC.Core.BRepAlgoAPI
import OCC.Core.BRepBuilderAPI
import OCC.Core.BRepBuilderAPI
import OCC.Core.BRepFilletAPI
import OCC.Core.BRepOffsetAPI
import OCC.Core.BRepPrimAPI
import OCC.Core.GeomAbs
import OCC.Core.ShapeUpgrade
import OCC.Core.TopOpeBRepBuild
import OCC.Core.TopoDS
import OCC.Core.gp
import OCC.Core.gp as gp
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.gp import gp_Vec

import pythonoccutils.occutils_python as op
from pythonoccutils.part_manager import Part, PartFactory

from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE


class TestPart(unittest.TestCase):

    def test_part_prism(self):
        p0 = op.WireSketcher()\
            .line_to(x=5, label="bottom")\
            .line_to(y=5, label="back")\
            .close()\
            .get_face_part()\
            .extrude.symmetric_prism(dz=10)

        bottom_parts = {*p0.get("bottom")}
        back_parts = {*p0.get("back")}

        bottom_types = sorted(s.ShapeType() for s in bottom_parts)
        back_types = sorted(s.ShapeType() for s in back_parts)

        self.assertEqual([TopAbs_FACE, TopAbs_FACE, TopAbs_EDGE], bottom_types)
        self.assertEqual([TopAbs_FACE, TopAbs_FACE, TopAbs_EDGE], back_types)

        self.assertEqual(0, len(bottom_parts.intersection(back_parts)))

        p0 = p0.cleanup().pruned()

        bottom = p0.get_single("bottom")
        back = p0.get_single("back")

        self.assertEqual(TopAbs_FACE, bottom.ShapeType())
        self.assertEqual(TopAbs_FACE, back.ShapeType())

        self.assertNotEqual(bottom, back)

    def test_part_translate(self):
        p0 = PartFactory.box(1, 1, 1)

        p1 = p0.transform.translate(dx=1)
        p1_extents = p1.extents

        p0_extents = p0.extents

        self.assertNotEqual(p0.shape, p1.shape)

        self.assertEqual(p0_extents.xyz_mid, [0.5, 0.5, 0.5])
        self.assertEqual(p1_extents.xyz_mid, [1.5, 0.5, 0.5])

    def test_create_part_with_named_subpart(self):
        mkbox = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeBox(10, 2, 3)

        part = Part(mkbox.Shape(), {"front_face": [mkbox.FrontFace()]})

        self.assertEqual(part.shape, mkbox.Shape())

        self.assertEqual(part.get_single("front_face"), mkbox.FrontFace())

    def test_create_part_with_named_subpart_after_prune(self):
        mkbox = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeBox(10, 2, 3)

        part = Part(mkbox.Shape(), {"front_face": [mkbox.FrontFace()]}).pruned()

        self.assertEqual(part.shape, mkbox.Shape())
        self.assertEqual(part.get_single("front_face"), mkbox.FrontFace())

    def test_part_prune(self):
        mkbox = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeBox(10, 2, 3)

        orphan_face = OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(gp.gp_Pln(gp.gp_Origin(), gp.gp_DZ())).Shape()
        pruned_part = Part(mkbox.Shape(), {"front_face": [orphan_face]}).pruned()

        self.assertFalse("front_face" in pruned_part.subshapes.keys())

    def test_subpart_pruned(self):
        mkbox_a = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeBox(10, 10, 10)
        mkbox_b = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeBox(10, 10, 10)

        part = Part(op.GeomUtils.make_compound(mkbox_a.Shape(), mkbox_b.Shape()), {
            "box_a": [mkbox_a.Shape()],
            "box_b": [mkbox_b.Shape()],
            "box_a_front_face": [mkbox_a.FrontFace()],
            "box_b_front_face": [mkbox_b.FrontFace()],
            "back_faces": [mkbox_a.BackFace(), mkbox_b.BackFace()]
        })

        # sanity check
        self.assertNotEqual(mkbox_a.BackFace(), mkbox_b.BackFace())

        box_a_part = part.single_subpart("box_a").pruned()
        box_b_part = part.single_subpart("box_b").pruned()

        self.assertFalse("box_b_front_face" in box_a_part.subshapes.keys())
        self.assertFalse("box_a_front_face" in box_b_part.subshapes.keys())

        self.assertEqual(box_a_part.get_single("back_faces"), mkbox_a.BackFace())
        self.assertEqual(box_b_part.get_single("back_faces"), mkbox_b.BackFace())
        self.assertEqual(box_a_part.get_single("box_a_front_face"), mkbox_a.FrontFace())
        self.assertEqual(box_b_part.get_single("box_b_front_face"), mkbox_b.FrontFace())

        self.assertFalse("box_a" in box_a_part.subshapes.keys())
        self.assertFalse("box_b" in box_a_part.subshapes.keys())

        self.assertFalse("box_a" in box_b_part.subshapes.keys())
        self.assertFalse("box_b" in box_b_part.subshapes.keys())

    def test_partfactory_loft(self):
        wires_or_faces = [
            PartFactory.right_angle_triangle(10, math.pi / 3),
            PartFactory.square_centered(10, 10).transform.translate(dz=10)
        ]

        part = PartFactory.loft(
            [w.shape for w in wires_or_faces],
            is_solid=True,
            first_shape_name="bottom",
            loft_profile_name="loft-profile",
            last_shape_name="top")

        loft_profiles = part.get("loft-profile")

        bottom = part.get_single("bottom")
        top = part.get_single("top")

        self.assertTrue(top not in loft_profiles)
        self.assertTrue(bottom not in loft_profiles)

        self.assertEqual(len(set(loft_profiles)), 6)
        self.assertNotEqual(bottom, top)

        self.assertTrue(op.Extents(bottom).z_max < op.Extents(top).z_max)
