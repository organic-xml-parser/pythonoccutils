import logging
import math
import random

import typing
from OCC.Core.gp import gp_OZ, gp_OX, gp_Vec
from OCC.Core.gp import gp_OY
import OCC.Core.BRepBuilderAPI

from pythonoccutils.occutils_python import InterrogateUtils, WireSketcher
from pythonoccutils.part_manager import PartFactory, Part
from pythonoccutils.precision import Compare

logger = logging.getLogger(__name__)


def file_pattern(file: Part):
    #file_inner = file.transform.rotate(gp_OY(), math.radians(0))\
    #    .transform.translate(dx=-12) \
    #    .transform.rotate(gp_OX(), math.radians(-15))

    file_outer = file.transform.rotate(gp_OX().Translated(gp_Vec(0, 0, 20)), math.radians(15))\
        .transform.translate(dx=-15)

    file_pattern_base = file_outer# file_inner.add(file_outer)

    file_base_count = 5

    return file_pattern_base\
        .pattern(range(0, file_base_count),
                 lambda i, p: p.transform.rotate(gp_OZ(), i / file_base_count * 2.0 * math.pi).name_recurse(f"file-{i}"))


def make_files(taper: bool):

    shank = PartFactory.cylinder(10, height=110)

    if taper:
        shank = shank.bool.union(PartFactory.cone(0, 10, 10).align().z_max_to_min(shank))\
            .cleanup()

    shank = shank.transform.scale_to_x_span(9)\
        .transform.scale_to_y_span(6)

    handle = PartFactory.cylinder(radius=18/2, height=59.5)\
        .do(lambda p: p.fillet.fillet_edges(5, lambda e: Compare.lin_eq(Part(e).extents.z_mid, p.extents.z_min)))\
        .do(lambda p: p.fillet.fillet_edges(3, lambda e: Compare.lin_eq(Part(e).extents.z_mid, p.extents.z_max)))

    z_offset = -3 if taper else -10

    file: Part = handle.align().z_min_to_max(shank)\
        .bool.union(shank)\
        .do(lambda p: p.transform.translate(dz=-p.extents.z_min + z_offset))

    return file_pattern(file)


def make_counterbore_pattern():
    base_shape = PartFactory.cylinder(10, 50)\
        .fillet.fillet_edges(2)\
        .do(lambda p: p.transform.translate(dz=-p.extents.z_min + 33))
    return file_pattern(base_shape)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    files = make_files(False) # .save.single_stl("/wsp/output/files.stl")

    # check to see if any two files intersect
    collision_check = files.bool.union().cleanup()
    collision_solids = collision_check.explore.solid.get()
    if len(collision_solids) != len(files.explore.solid.get()):
        logger.warning("INTERSECTION")
        collision_check.preview()
        raise ValueError("Appears to be overlap with files.")

    def get_section_poly(z_height: float, com_bulge: float = 0, poly_offset: float = 0):
        sq = PartFactory.square_centered(files.extents.x_span + 1, files.extents.y_span + 1)\
            .transform.translate(dz=z_height)

        faces = [f.pruned() for f in files.bool.common(sq).pruned().explore.face.get()]

        sq.save.single_stl(f"/wsp/output/section-sq-{z_height}")
        PartFactory.compound(*faces).save.single_stl(f"/wsp/output/section-faces-{z_height}")

        coords: typing.List[Part] = []
        for f in faces:
            coords += [InterrogateUtils.center_of_mass(f.shape)]

        coords.sort(key=lambda c: math.atan2(c[0], c[1]))

        ws = WireSketcher(*coords[0])
        for c in coords[1:]:
            ws.line_to(*c)

        face = ws.close().get_face_part()
        if poly_offset != 0:
            face = face.extrude.offset(poly_offset).make.face()

        if com_bulge == 0:
            return face

        return face.bool.union(*[PartFactory.circle(com_bulge).make.face().transform.translate(*c) for c in coords])\
            .sew.faces().cleanup(concat_b_splines=True)\
            .explore.face.get_single()

    z_heights_ext = [files.extents.z_min + 5 + i for i in range(0, 60, 10)]
    z_heights_int = [files.extents.z_min + 5 + i for i in range(0, 60, 5)]

    print(z_heights_ext)
    print(z_heights_int)

    def smooth_for_z(profile: Part) -> Part:
        smoothing_minima_z = 25

        z_height = profile.extents.z_mid

        smoothing_amount = max(4, abs(z_height - smoothing_minima_z))

        smoothed_profile = profile.extrude.offset(smoothing_amount).extrude.offset(-smoothing_amount)

        return smoothed_profile

    def flared_for_z(profile: Part) -> Part:
        # need at least 15 pieces of flare...

        z_height = profile.extents.z_mid

        # amount of flare proportional to z height
        flare_offset = max(0, 30 / abs(z_height))

        flare_offset += 0.2 * max(0, 20 - abs(z_height * z_height))

        return profile.extrude.offset(flare_offset)

    def bulge_for_z(z_height: float) -> float:
        z_offs = math.sqrt(abs(z_height - 20)) # minima midway

        return 4 + z_offs

    ext_sections_rough = [get_section_poly(z, com_bulge=bulge_for_z(z), poly_offset=1 - z * 0.1) for z in z_heights_ext]
    ext_sections_smoothed = [smooth_for_z(e) for e in ext_sections_rough]
    ext_sections = [flared_for_z(e) for e in ext_sections_smoothed]

    PartFactory.compound(*[e.make.face() for e in ext_sections_rough]).save.single_stl("/wsp/output/ext_sections_rough")
    PartFactory.compound(*[e.make.face() for e in ext_sections_smoothed]).save.single_stl("/wsp/output/ext_sections_smoothed")
    PartFactory.compound(*[e.make.face() for e in ext_sections]).save.single_stl("/wsp/output/ext_sections")

    #PartFactory.compound(*ext_sections).add(*ext_sections_smoothed).add(*ext_sections_rough).preview()

    logger.info(f"Exterior profile created")

    PartFactory.compound(*ext_sections)

    loft_ext = PartFactory.loft(ext_sections, is_ruled=False).save.single_stl("/wsp/output/loft_ext")

    #PartFactory.compound(loft_ext, *ext_sections).preview().raise_exception()

    #loft_ext_top_chamfer = PartFactory.loft([
    #    ext_sections[-1],
    #    ext_sections[-1].transform.translate(dz=2).extrude.offset(-2)]
    #)
#
    #loft_ext = loft_ext.bool.union(loft_ext_top_chamfer)

    logging.info("Creating the fluted interior profile")
    loft_int = PartFactory.loft([
        PartFactory.circle(4).align().z_max_to_max(loft_ext).transform.translate(dz=-25), # int_sections[-1].transform.scale_to_x_span(3, scale_other_axes=True).align().z_max_to_max(loft_ext).transform.translate(dz=-25),
        PartFactory.circle(5).align().z_max_to_max(loft_ext).transform.translate(dz=-15), # int_sections[-1].transform.scale_to_x_span(3, scale_other_axes=True).align().z_max_to_max(loft_ext).transform.translate(dz=-25),
        PartFactory.circle(6).align().z_max_to_max(loft_ext).transform.translate(dz=-5), #int_sections[-1].transform.translate(dz=-1).extrude.offset(5),
        get_section_poly(50).extrude.offset(3)
    ], is_ruled=False).save.single_stl("/wsp/output/loft_int")

    loft_ext = loft_ext.fillet.fillet_faces(4, lambda e: Compare.lin_eq(abs(InterrogateUtils.face_normal(e)[1].Z()),1) and Part(e).extents.z_mid > loft_ext.extents.z_mid)

    logger.info(f"Interior profile created")

    logger.info(f"Performing cut")

    result = loft_ext
    bottom_face: Part = result.explore.face \
        .filter_by(lambda f: Compare.lin_eq(abs(InterrogateUtils.face_normal(f.shape)[1].Z()), 1)) \
        .order_by(lambda f: f.extents.z_mid) \
        .get()[0]
    result = result.bool.union(PartFactory.loft([
        bottom_face,
        bottom_face.transform.translate(dz=-1).extrude.offset(-0.5)]))

    PartFactory.compound(*make_counterbore_pattern().align().z_min_to_max(result).transform.translate(dz=-3).explore.solid.get()).save.single_stl("/wsp/output/counterbores")
    make_files(True).save.single_stl("/wsp/output/cut_files")

    result = result.bool.cut(loft_int)\
        .bool.cut(*make_counterbore_pattern().align().z_min_to_max(result).transform.translate(dz=-3).explore.solid.get())\
        .bool.cut(make_files(True))

    hex_cavity = PartFactory.polygon(1, 6) \
        .transform.scale_to_y_span(13, scale_other_axes=True) \
        .make.face() \
        .extrude.prism(dz=20.6) \
        .transform.translate(dz=2) \
        .bool.union(PartFactory.cylinder(4, 100)) \
        .cleanup().save.single_stl("/wsp/output/hex_cavity")

    result = result.bool.cut(hex_cavity.align().z_min_to_min(result))

    result.preview(files).save.single_stl("/wsp/output/to-print", lin_deflection=0.01, ang_deflection=0.1)


    #    .preview(files)\
    #    .save.single_stl("/wsp/output/to-print")
