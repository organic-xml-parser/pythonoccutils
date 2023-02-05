import importlib
import importlib.resources
import logging
import math
import pdb

import OCC.Core.GCE2d
import OCC.Core.Geom2d
import js2py
import typing
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
from OCC.Core.Geom import Geom_CylindricalSurface
from OCC.Core._gp import gp_XOY
from OCC.Core.gp import gp_Pnt2d, gp_Ax3, gp_OZ, gp_OX

import pythonoccutils.gears.gears_js_translated as gear
from pythonoccutils.occutils_python import WireSketcher
from pythonoccutils.part_manager import Part, PartFactory
from pythonoccutils.svg_parser import SVGPathParser

gear_outline_fn = gear.var.own['createGearOutline']['value']

logger = logging.getLogger(__name__)


class GearMath:

    @staticmethod
    def pitch_diameter(module: float, num_teeth: int) -> float:
        return num_teeth * module

    @staticmethod
    def outside_diameter(module: float, num_teeth: int) -> float:
        return module * (num_teeth + 2)

    @staticmethod
    def root_diameter(module: float, num_teeth: int) -> float:
        return module * (num_teeth - 2.5)


class GearSpec:

    def __init__(self, module: float, num_teeth: int, pressure_angle_deg: float = 20):
        self.module = module
        self.num_teeth = num_teeth
        self.pressure_angle_deg = pressure_angle_deg

    @property
    def pitch_diameter(self) -> float:
        return GearMath.pitch_diameter(self.module, self.num_teeth)

    @property
    def outside_diameter(self) -> float:
        return GearMath.outside_diameter(self.module, self.num_teeth)

    @property
    def root_diameter(self) -> float:
        return GearMath.root_diameter(self.module, self.num_teeth)


class GearPairSpec:

    def __init__(self,
                 gear_spec_bull: GearSpec,
                 gear_spec_pinion: GearSpec,
                 center_distance: float):
        self.gear_spec_bull = gear_spec_bull
        self.gear_spec_pinion = gear_spec_pinion
        self.center_distance = center_distance

    @staticmethod
    def matched_pair(module: float,
                     num_teeth_bull: int,
                     num_teeth_pinion: int,
                     **kwargs):

        # z = d / m
        pitch_diameter_bull = GearMath.pitch_diameter(module, num_teeth_bull)
        pitch_diameter_pinion = GearMath.pitch_diameter(module, num_teeth_pinion) # num_teeth_pinion * module

        minimum_distance = (pitch_diameter_bull + pitch_diameter_pinion) / 2

        return GearPairSpec(
            GearSpec(module, num_teeth_bull, **kwargs),
            GearSpec(module, num_teeth_pinion, **kwargs),
            minimum_distance)


class InvoluteGearFactory:

    def __init__(self):
        def noop_profile_hook(part: Part) -> Part:
            return part

        self.profile_modifier_hook: typing.Callable[[Part], Part] = noop_profile_hook

    def create_involute_profile(self, gear_spec: GearSpec) -> Part:

        logger.info("Generating gear...")
        gear_result = gear_outline_fn(gear_spec.module, gear_spec.num_teeth, gear_spec.pressure_angle_deg)

        # generate the svg path string
        logger.info("Creating svg path")
        svg_path = ' '.join(str(s) for s in gear_result.to_python())

        logger.info("Building wire")
        wire = SVGPathParser.parse_wire(svg_path)

        result = Part(wire).explore.wire.get()[0]

        return self.profile_modifier_hook(result)

    def create_herringbone_gear(
            self,
            gear_spec: GearSpec,
            height: float,
            sweep_sense: bool = True,
            helix_angle_deg: float = 45,
            chamfer: typing.Optional[float] = None,
            sweep_cycles: int = 1,
            make_solid: bool = True):

        profile = self.create_involute_profile(gear_spec)

        spine = WireSketcher(0, 0, 0).line_to(z=height).get_wire_part()

        # create a spiral aux spine to "encourage" OCC to make a helix sweep. unit radius
        logger.info(f"Height of cylinder surface: {height}")

        surf = Geom_CylindricalSurface(gp_Ax3(gp_XOY()), gear_spec.pitch_diameter / 2)

        # the actual angle of the circle that needs to be swept depends on the gear height
        helix_angle_rad = math.radians(helix_angle_deg)
        logger.info(f"Helix angle in degrees: {helix_angle_deg} -> radians: {helix_angle_rad}")

        sweep_distance_du = 0.5 * (height / sweep_cycles) * math.tan(helix_angle_rad)
        logger.info(f"Sweep distance (on pitch diameter) of gear: {sweep_distance_du}")

        sweep_angle_rad = sweep_distance_du / (gear_spec.pitch_diameter / 2)
        logger.info(f"Sweep angle in radians: {sweep_angle_rad}")

        if not sweep_sense:
            sweep_angle_rad *= -1

        mkw = BRepBuilderAPI_MakeWire()
        sweep_height = height / sweep_cycles
        for i in range(0, sweep_cycles):
            height_start = i * sweep_height
            height_mid = (i + 0.5) * sweep_height
            height_end = (i + 1) * sweep_height

            edge0 = BRepBuilderAPI_MakeEdge(
                OCC.Core.GCE2d.GCE2d_MakeSegment(gp_Pnt2d(sweep_angle_rad, height_start), gp_Pnt2d(0, height_mid)).Value(), surf).Edge()
            edge1 = BRepBuilderAPI_MakeEdge(
                OCC.Core.GCE2d.GCE2d_MakeSegment(gp_Pnt2d(0, height_mid), gp_Pnt2d(sweep_angle_rad, height_end)).Value(), surf).Edge()

            mkw.Add(edge0)
            mkw.Add(edge1)

        aux_spine = Part(mkw.Wire())

        mkps = BRepOffsetAPI_MakePipeShell(spine.shape)
        mkps.SetMode(aux_spine.shape, True)
        mkps.Add(profile.shape)
        mkps.Build()

        if make_solid:
            mkps.MakeSolid()

        result = Part(mkps.Shape())

        if chamfer is not None:
            logger.info(f"Applying chamfer: {chamfer}")
            cylinder_common = PartFactory.cylinder(gear_spec.outside_diameter / 2, height).fillet.chamfer_edges(chamfer)

            return result.bool.common(cylinder_common)
        else:
            return result


def translate_js_lib_to_python():
    """
    Translation of the js source to python for easy use.
    """

    with importlib.resources.open_text(package="pythonoccutils.gears", resource="gears.js") as text:
        js_source = text.read()

    translate = js2py.translate_js6(js_source)

    pdb.set_trace()

