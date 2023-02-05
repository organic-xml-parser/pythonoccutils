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


class PartFactoryTest(unittest.TestCase):

    def test_ra_triangle(self):
        t = PartFactory.right_angle_triangle(math.sqrt(2), math.pi / 4)
        self.assertAlmostEqual(t.extents.x_span, 1)
        self.assertAlmostEqual(t.extents.y_span, 1)

        t = PartFactory.right_angle_triangle(10, math.pi / 3, pln=gp.gp_YOZ())
        self.assertAlmostEqual(t.extents.y_span, 10 * math.cos(math.pi/3))
        self.assertAlmostEqual(t.extents.z_span, 10 * math.sin(math.pi/3))

