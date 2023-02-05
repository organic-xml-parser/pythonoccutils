import math
import typing

import OCC.Core.TopAbs
import OCC.Core.gp as gp
import OCC.Core.BRepBuilderAPI
import OCC.Core.GeomAbs
from OCC.Core.gp import gp_Vec as vec

import pythonoccutils.occutils_python as op
from pythonoccutils.constants import Constants
from pythonoccutils.part_manager import Part, PartFactory
from pythonoccutils.occutils_python import WireSketcher


class StockParts:

    @staticmethod
    def switch_momentary(button_height: float, body_height=3.5) -> Part:
        result = PartFactory.square_centered(dx=6, dy=6)\
            .extrude.prism(dz=body_height, last_shape_name="switch_face")

        pole = PartFactory.square_centered(6.5, 1).extrude.prism(dz=1)

        result = result.bool.union(
            pole.align().y_max_to_max(result),
            pole.align().y_min_to_min(result)).cleanup()

        knob = PartFactory.loft([
            op.GeomUtils.circle_wire(3.5 / 2),
            op.TransformUtils.translate(op.GeomUtils.circle_wire(3 / 2), vec(0, 0, button_height))
        ], loft_profile_name="knob_surround")

        knob = knob.align()\
            .xy_mid_to_mid(result)\
            .align()\
            .z_min_to_max(result)

        result = result.bool.union(knob).cleanup().fillet.fillet_by_name(0.3, "knob_surround")

        ext = op.Extents(result.shape)
        p = Constants.perfboard_pitch()

        return result\
            .add(PartFactory.vertex(-p, -p, ext.z_min, name="P0_0"))\
            .add(PartFactory.vertex(p, -p, ext.z_min, name="P1_0"))\
            .add(PartFactory.vertex(-p, p, ext.z_min, name="P0_1"))\
            .add(PartFactory.vertex(p, p, ext.z_min, name="P1_1"))

    @staticmethod
    def perfboard_50x70(**kwargs) -> Part:
        return StockParts._perfboard(50, 70, 18, 24, **kwargs)

    @staticmethod
    def arduino_micro(add_legs: bool = True) -> Part:
        result = PartFactory.square_centered(18, 48).extrude.prism(0, 0, -1.5).name("board")

        pin_template: Part = PartFactory.cylinder(0.5, 2, top_wire_name="top_wire") \
            .fillet.fillet_by_name(0.2, "top_wire").align().z_min_to_max(result)

        if add_legs:
            pin_template = pin_template.add(
                PartFactory.square_centered(0.6, 0.6)
                    .align().xy_mid_to_mid(pin_template)
                    .align().z_max_to_min(result)
                    .extrude.prism(0, 0, -9.5)
            )

        lpins = pin_template.pattern(
            range(0, 17),
            lambda i, p: p.transform.translate(0, i * Constants.perfboard_pitch(), 0).name(f"pin_{16 - i}"))
        rpins = pin_template.transform.translate(6 * Constants.perfboard_pitch()).pattern(
            range(0, 17),
            lambda i, p: p.transform.translate(0, i * Constants.perfboard_pitch(), 0).name(f"pin_{i + 17}"))

        pins = lpins.add(rpins).align().xy_mid_to_mid(result)

        switch = StockParts.switch_momentary(1, body_height=2).align().y_max_to_max(result)

        chip = PartFactory.square_centered(7, 7)\
            .transform.rotate(gp.gp_OZ(), math.pi / 4)\
            .align().x_mid_to_mid(result)\
            .extrude.prism(0, 0, 1)

        usb_connector = op.WireSketcher(gp.gp_Pnt(0, 0, 0))\
            .line_to(x=6.5 / 2, is_relative=True)\
            .line_to(x=1, y=1, is_relative=True)\
            .line_to(x=-1.5, y=1.5, is_relative=True)\
            .line_to(x=0, y=2.5)\
            .close()\
            .get_face()

        usb_connector = Part(usb_connector)
        usb_connector = usb_connector.bool.union(usb_connector.transform(lambda t: t.SetMirror(gp.gp_YOZ())))\
            .transform.rotate(gp.gp_OX(), math.pi / 2)\
            .extrude.prism(0, 5.5, 0)\
            .cleanup()\
            .align().y_min_to_min(result)\
            .transform.translate(0, 1, 0)

        return result.add(pins, switch, chip, usb_connector)

    @staticmethod
    def _perfboard(dx: float, dy: float, cols: int, rows: int, include_z: bool = True, include_holes: bool = True) -> Part:
        result = Part(op.GeomUtils.square_centered(side_length_0=dx, side_length_1=dy).get_face())
        hole_face = op.GeomUtils.circle_face(0.5)
        hole_part = Part(hole_face, {
            "hole_edge": [op.Explorer.edge_explorer(hole_face).get_single()]
        })

        holes = None

        for col in range(0, cols):
            for row in range(0, rows):
                coord = (col * Constants.perfboard_pitch(), row * Constants.perfboard_pitch(), 0)

                if include_holes:
                    hole = hole_part.rename_subshape("hole_edge", f"{col}_{row}")\
                        .align().xy_min_to_min(result.shape)\
                        .transform.translate(*coord)

                    if holes is None:
                        holes = hole
                    else:
                        holes = holes.add(hole)

        holes = holes.align().xy_mid_to_mid(result)

        result = result.bool.cut(holes)

        if include_z:
            result = result.extrude.prism(0, 0, -1.5)

        return result

    @staticmethod
    def sd_card_micro() -> Part:
        chamfer_length = 11 - 9.7

        return WireSketcher()\
            .line_to(x=9.7, is_relative=True)\
            .line_to(y=-6.4 + chamfer_length, is_relative=True)\
            .line_to(x=chamfer_length, y=-chamfer_length, is_relative=True)\
            .line_to(y=-15)\
            .line_to(x=0)\
            .close()\
            .get_face_part()\
            .extrude.prism(dz=1)

    @staticmethod
    def raspberry_pi_4() -> Part:
        board = PartFactory.square_centered(85, 56).fillet.fillet2d_verts(3)
        hole = PartFactory.cylinder(radius=2.7/2, height=-1.5)\
            .align().xy_mid_to_min(board)\
            .transform.translate(3.5, 3.5)

        hole_cut = hole.name_recurse("hole_bottom_left", lambda s: s.ShapeType() == OCC.Core.TopAbs.TopAbs_EDGE)\
            .add(hole.transform.translate(dy=49).name_recurse("hole_top_left", lambda s: s.ShapeType() == OCC.Core.TopAbs.TopAbs_EDGE))\
            .add(hole.transform.translate(dx=58, dy=49).name_recurse("hole_top_right", lambda s: s.ShapeType() == OCC.Core.TopAbs.TopAbs_EDGE))\
            .add(hole.transform.translate(dx=58).name_recurse("hole_bottom_right", lambda s: s.ShapeType() == OCC.Core.TopAbs.TopAbs_EDGE))

        hole_cut.compound_subpart("hole_bottom_right")

        board = board.bool.cut(hole_cut)\
            .extrude.prism(dz=-1.5).name("board_circumference")

        gpio_slot = PartFactory.square_centered(50, 5)\
            .align().x_mid_to_mid(hole_cut)\
            .align().y_mid_to_max(board)\
            .transform.translate(dy=-3.5)\
            .extrude.prism(dz=8.5)\
            .name("gpio_port")

        ethernet_slot = PartFactory.square_centered(dx=17.5, dy=16)\
            .extrude.prism(dz=13.5)\
            .align().z_min_to_max(board)\
            .align().y_mid_to_min(board).transform.translate(dy=45.75)\
            .align().x_max_to_max(board).transform.translate(dx=3)\
            .name("ethernet_port")

        usb_slot = PartFactory.square_centered(dx=21.5, dy=15)\
            .extrude.prism(dz=16)\
            .align().z_min_to_max(board)\
            .align().y_mid_to_min(board)\
            .align().x_max_to_max(board).transform.translate(dx=3)

        usb_slot_a = usb_slot.transform.translate(dy=9).name("usb_port_a")
        usb_slot_b = usb_slot.transform.translate(dy=27).name("usb_port_b")

        usbc_port = PartFactory.square_centered(dx=9, dy=3)\
            .fillet.fillet2d_verts(1)\
            .transform.rotate(gp.gp_OX(), angle=math.pi / 2)\
            .extrude.prism(dy=7.5)\
            .align().z_min_to_max(board)\
            .align().x_mid_to_min(board).transform.translate(dx=3.5+7.7)\
            .align().y_min_to_min(board).transform.translate(dy=-1.5)\
            .name("usbc_port")

        hdmi_micro_port = PartFactory.square_centered(dx=7, dy=7)\
            .extrude.prism(dz=3.0)\
            .align().z_min_to_max(board)\
            .align().x_mid_to_min(board)\
            .align().y_min_to_min(board).transform.translate(dy=-1.5)

        hdmi_micro_port_a = hdmi_micro_port\
            .transform.translate(dx=3.5 + 7.7 + 14.8)\
            .name("hdmi_micro_a")

        hdmi_micro_port_b = hdmi_micro_port_a.transform\
            .translate(dx=13.5)\
            .name("hdmi_micro_b")

        sd_card = StockParts.sd_card_micro()\
            .transform.rotate(gp.gp_OZ(), math.pi / 2)\
            .transform.rotate(gp.gp_OY(), math.pi)\
            .align().z_max_to_min(board).transform.translate(dz=-2)\
            .align().y_mid_to_mid(board)\
            .align().x_min_to_min(board).transform.translate(dx=-2.5)\
            .name("sd_card")

        barrel_jack = PartFactory.square_centered(dx=7, dy=6)\
            .extrude.prism(dz=12.5)\
            .bool.union(PartFactory.cylinder(radius=3, height=15))\
            .cleanup()\
            .transform.rotate(gp.gp_OX(), math.pi/2)\
            .align().z_min_to_max(board)\
            .align().y_min_to_min(board).transform.translate(dy=-2.5)\
            .align().x_mid_to_mid(hdmi_micro_port_b).transform.translate(dx=14.5)\
            .name("audio_jack")

        ribbon_connector = PartFactory.square_centered(dx=2.5, dy=22)\
            .extrude.prism(dz=5.5)\
            .align().x_min_to_max(board)

        ribbon_display = ribbon_connector\
            .align().y_mid_to_min(board).transform.translate(dy=3.5+24.5)\
            .align().x_mid_to_min(board).transform.translate(dx=4)\
            .name("ribbon_display")

        ribbon_camera = ribbon_connector\
            .align().x_mid_to_mid(hdmi_micro_port_b).transform.translate(dx=7)\
            .align().y_mid_to_min(board).transform.translate(dy=11.5)\
            .name("ribbon_camera")

        solder_blobs = PartFactory.cylinder(1, 2)\
            .align().z_max_to_min(board)\
            .align().xy_mid_to_mid(usb_slot_a).name("solder_blobs")

        usb_c_and_hdmi = usbc_port.add(hdmi_micro_port_a).add(hdmi_micro_port_b).add(barrel_jack).name("usbc_and_hdmi")

        board = board.add(gpio_slot,
                          ethernet_slot,
                          usb_slot_a,
                          usb_slot_b,
                          usb_c_and_hdmi,
                          sd_card,
                          ribbon_display,
                          ribbon_camera,
                          solder_blobs)

        return board

    @staticmethod
    def x_y_plane()-> Part:
        return Part(
            OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(gp.gp_Pln(gp.gp_Origin(), gp.gp_DZ())).Shape())

    @staticmethod
    def y_z_plane()-> Part:
        return Part(
            OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(gp.gp_Pln(gp.gp_Origin(), gp.gp_DX())).Shape())

    @staticmethod
    def z_x_plane()-> Part:
        return Part(
            OCC.Core.BRepBuilderAPI.BRepBuilderAPI_MakeFace(gp.gp_Pln(gp.gp_Origin(), gp.gp_DZ())).Shape())

    @staticmethod
    def din_rail_35mm(length: float)-> Part:
        return WireSketcher().line_to(z=25/2, is_relative=True)\
            .line_to(y=7.5, is_relative=True)\
            .line_to(z=35/2, is_relative=False)\
            .get_wire_part().extrude.square_offset(1).make.face()\
            .extrude.prism(dx=length)\
            .mirror.z(True)\
            .cleanup()\
            .fillet.fillet_edges(0.2, lambda e: op.InterrogateUtils.is_dx_line(e))

    @staticmethod
    def screw_m3(length: float,
                 head_diameter: float = 5.5,
                 hex_socket_x_span: float = 3,
                 head_z_span: float = 3)-> Part:

        hex = Part(op.GeomUtils.regular_polygon(1, 6))\
            .transform.rotate(gp.gp_OZ(), math.pi / 2)\
            .make.face()\
            .transform.scale_to_x_span(hex_socket_x_span, scale_other_axes=True)\
            .extrude.prism(dz=head_z_span - 1)

        shaft = PartFactory.cylinder(radius=1.5, height=length)

        head = PartFactory.cylinder(radius=head_diameter / 2, height=head_z_span)\
            .do(lambda p: p.bool.cut(hex.align().xy_mid_to_mid(p).align().z_max_to_max(p))) \
            .do(lambda p:
                p.fillet.fillet_edges(0.15, lambda f: Part(f).extents.z_max > p.extents.z_min))

        return head.bool.union(shaft.align().xy_mid_to_mid(head).align().z_max_to_min(head))

    @staticmethod
    def ziptie_cavity(ziptie_width: float,
                      ziptie_thickness: float,
                      inner_curvature_radius: float = 5,
                      cavity_depth: float = 4,
                      clearance: float = 0.2) -> Part:
        cylinder_inner = PartFactory.cylinder(inner_curvature_radius, height=ziptie_width + 2 * clearance)
        cylinder_outer = PartFactory.cylinder(inner_curvature_radius + ziptie_thickness * 2 * clearance, height=ziptie_width + 2 * clearance)

        cylinder = cylinder_outer.bool.cut(cylinder_inner)

        return PartFactory.box_surrounding(cylinder)\
            .align().x_min_to_min(cylinder)\
            .transform.translate(dx=cavity_depth)\
            .do(lambda p: cylinder.bool.cut(p))\
            .transform.rotate(gp.gp_OY(), -math.pi / 2)\
            .transform.rotate(gp.gp_OZ(), math.pi / 2)

    @staticmethod
    def enclosure(
            enclosed_part: Part,
            lid_offset: float = 0,
            wall_thickness: float = 3)-> Part:

        box = PartFactory.box_surrounding(enclosed_part)\
            .do(
                lambda p: PartFactory.box_surrounding(p, wall_thickness, wall_thickness, wall_thickness)\
                    .fillet.fillet_edges(radius=wall_thickness)
                    .bool.cut(p))

        if lid_offset == 0:
            return box

        lid_section = PartFactory.square_centered(box.extents.x_span + 1, box.extents.y_span + 1, box.extents.z_span + 1)\
            .align().xyz_mid_to_mid(box)\
            .bool.common(box)\
            .explore.wire\
            .order_by(lambda w: op.InterrogateUtils.length(w.shape)).get()[-1]

        lid_section: Part = WireSketcher(0, 0, 0)\
            .line_to(x=wall_thickness / 3)\
            .line_to(z=wall_thickness / 3)\
            .line_to(x=2 * wall_thickness / 3)\
            .line_to(z=0)\
            .line_to(x=wall_thickness)\
            .get_wire_part() \
            .extrude.square_offset(0.15) \
            .align().xz_min_to_min(lid_section)\
            .align().y_mid_to_mid(lid_section)\
            .loft.pipe(lid_section.shape)

        return box.bool.cut(lid_section)

    @staticmethod
    def standoffs(surround_extents: Part,
                  x_hole_spacing: float,
                  y_hole_spacing: float,
                  height: float,
                  fillet_diameter: float,
                  drill_bit: op.Bit)-> Part:

        standoff: Part = PartFactory.box(
            (surround_extents.extents.x_span - x_hole_spacing + fillet_diameter) / 2,
            (surround_extents.extents.y_span - y_hole_spacing + fillet_diameter) / 2, height) \
            .do(lambda p: p.fillet.fillet_edges(
                fillet_diameter / 2,
                lambda e: op.InterrogateUtils.is_dz_line(e) and Part(e).extents.xy_mid == [p.extents.x_max, p.extents.y_min]))\
            .align().z_min_to_min(surround_extents)

        sa1 = standoff\
            .align().x_min_to_min(surround_extents).align().y_max_to_max(surround_extents)

        sa2 = standoff\
            .mirror.x()\
            .align().x_max_to_max(surround_extents).align().y_max_to_max(surround_extents)

        sa3 = standoff\
            .mirror.y()\
            .align().x_min_to_min(surround_extents).align().y_min_to_min(surround_extents)

        sa4 = standoff\
            .mirror.y().mirror.x()\
            .align().x_max_to_max(surround_extents).align().y_min_to_min(surround_extents)

        standoffs = PartFactory.compound(sa1, sa2, sa3, sa4)

        return standoffs \
            .drill(op.Drill(drill_bit).square_pattern_centered(standoffs.shape,
                                                               *standoffs.extents.xyz_mid,
                                                               du=x_hole_spacing,
                                                               dv=y_hole_spacing))

    @staticmethod
    def dovetail(height: float,
                 h_bottom: float,
                 h_top: float,
                 pln: gp.gp_Ax2 = gp.gp_XOY()) -> Part:

        dx = gp.gp_Vec(pln.XDirection()).Normalized()
        dy = gp.gp_Vec(pln.YDirection()).Normalized()

        def v_to_xyz(vec: gp.gp_Vec):
            return vec.X(), vec.Y(), vec.Z()

        return WireSketcher()\
            .line_to(*v_to_xyz(dx.Scaled(h_bottom)), is_relative=True)\
            .line_to(*v_to_xyz(dx.Scaled(-h_bottom / 2 + h_top / 2).Added(dy.Scaled(height))), is_relative=True)\
            .line_to(*v_to_xyz(dx.Scaled(-h_top)), is_relative=True)\
            .line_to(*v_to_xyz(dx.Scaled(-h_bottom / 2 + h_top / 2).Added(dy.Scaled(-height))), is_relative=True)\
            .get_wire_part()

    @staticmethod
    def ruler() -> Part:
        return PartFactory.box(10, 30, 2)\
            .pattern(range(0, 20), lambda i, p: p.transform.translate(dx=i * 10))\
            .bool.union()

    @staticmethod
    def bearing_608():
        INNER_RACE_ID = 8
        INNER_RACE_OD = 12.1

        OUTER_RACE_ID = 19.2
        OUTER_RACE_OD = 22

        inner_race = PartFactory.cylinder(INNER_RACE_OD/2, 7)\
            .fillet.chamfer_edges(0.2)

        outer_race = PartFactory.cylinder(OUTER_RACE_OD / 2, 7)\
            .bool.cut(PartFactory.cylinder(OUTER_RACE_ID / 2, 7))\
            .fillet.chamfer_edges(0.2)

        cut = PartFactory.circle(2)\
            .transform.rotate(gp.gp_OY(), math.pi / 2)\
            .transform.translate(dy=op.MathUtils.lerp(y0=INNER_RACE_OD / 2, y1=OUTER_RACE_ID / 2, coordinate_proportional=0.5)[1])\
            .align().z_mid_to_mid(outer_race)\
            .loft.pipe(PartFactory.circle(10))

        race = PartFactory.sphere(3.55/2)\
            .align().z_mid_to_mid(outer_race)\
            .align().x_min_to_max(inner_race)\
            .pattern(range(0, 9), lambda i, p: p.transform.rotate(gp.gp_OZ(), 2 * math.pi * i / 9)) #PartFactory.cylinder(11, 5).align().z_mid_to_mid(outer_race)

        inner_race = inner_race.bool.cut(cut).cleanup()
        outer_race = outer_race.bool.cut(cut).cleanup()

        return PartFactory.compound(inner_race, outer_race, race).bool.cut(PartFactory.cylinder(INNER_RACE_ID / 2, 7)).preview().cleanup()


if __name__ == "__main__":
    nano = PartFactory.box(45, 18, 18)

    box_area = PartFactory.box_surrounding(nano, 5, 5, 5)

    enclosure = StockParts.enclosure(
        box_area,
        lid_offset=6,
        wall_thickness=2)

    standoffs = StockParts.standoffs(box_area,
                                     x_hole_spacing=nano.extents.x_span - 2 * 1.26,
                                     y_hole_spacing=nano.extents.y_span - 2 * 1.26,
                                     height=10,
                                     fillet_diameter=4,
                                     drill_bit=op.CylinderBit(1, 100))

    enclosure.bool.union(standoffs).preview().save.stl_solids("/wsp/output/ard")