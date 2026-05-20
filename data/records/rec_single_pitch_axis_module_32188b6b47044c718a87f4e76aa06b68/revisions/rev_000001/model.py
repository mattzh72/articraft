from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


PLATE_X = 0.380
PLATE_Y = 0.200
PLATE_Z = 0.014
AXIS_Z = 0.092
CHEEK_GAP = 0.108
CHEEK_T = 0.032
CHEEK_Y = 0.116
CHEEK_Z = 0.120
BORE_R = 0.018
SHAFT_R = 0.018


def _cylinder_x(radius: float, length: float, y: float, z: float) -> cq.Workplane:
    """CadQuery cylinder whose axis is aligned to world X."""
    return cq.Workplane("YZ").center(y, z).circle(radius).extrude(length, both=True)


def _fixed_housing_shape() -> cq.Workplane:
    """One cast-looking fixed housing: broad plate, cheek yoke, bosses, and ribs."""
    plate = (
        cq.Workplane("XY")
        .box(PLATE_X, PLATE_Y, PLATE_Z)
        .translate((0.0, 0.0, PLATE_Z / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    bolt_holes = cq.Workplane("XY")
    for x in (-0.145, 0.145):
        for y in (-0.066, 0.066):
            bolt_holes = bolt_holes.union(
                cq.Workplane("XY").center(x, y).circle(0.010).extrude(0.060, both=True)
            )
    plate = plate.cut(bolt_holes)

    pedestal = (
        cq.Workplane("XY")
        .box(0.215, 0.106, 0.020)
        .translate((0.0, 0.0, PLATE_Z + 0.010))
    )

    cheek_center_x = CHEEK_GAP / 2.0 + CHEEK_T / 2.0
    cheek_center_z = PLATE_Z + CHEEK_Z / 2.0
    cheek_l = (
        cq.Workplane("XY")
        .box(CHEEK_T, CHEEK_Y, CHEEK_Z)
        .translate((-cheek_center_x, 0.0, cheek_center_z))
    )
    cheek_r = (
        cq.Workplane("XY")
        .box(CHEEK_T, CHEEK_Y, CHEEK_Z)
        .translate((cheek_center_x, 0.0, cheek_center_z))
    )

    fixed = plate.union(pedestal).union(cheek_l).union(cheek_r)

    # Raised bearing pads around the pitch bores make the trunnion axis explicit.
    boss_len = 0.010
    boss_r = 0.028
    boss_x = cheek_center_x + CHEEK_T / 2.0 + boss_len / 2.0
    boss_r_shape = _cylinder_x(boss_r, boss_len, 0.0, AXIS_Z).translate((boss_x, 0.0, 0.0))
    boss_l_shape = _cylinder_x(boss_r, boss_len, 0.0, AXIS_Z).translate((-boss_x, 0.0, 0.0))
    fixed = fixed.union(boss_r_shape).union(boss_l_shape)

    # Four triangular gusset webs tying the side cheeks back into the low base.
    for side in (-1.0, 1.0):
        for y in (-0.046, 0.046):
            rib_profile = [
                (side * 0.030, PLATE_Z),
                (side * 0.058, PLATE_Z),
                (side * 0.058, AXIS_Z - 0.014),
            ]
            rib = (
                cq.Workplane("XZ")
                .polyline(rib_profile)
                .close()
                .extrude(0.012, both=True)
                .translate((0.0, y, 0.0))
            )
            fixed = fixed.union(rib)

    # Clear bearing bores through both cheeks and their raised bosses.
    fixed = fixed.cut(_cylinder_x(BORE_R, 0.260, 0.0, AXIS_Z))
    return fixed


def _tilt_flange_shape() -> cq.Workplane:
    """Compact moving cradle/flange authored in the joint frame."""
    flange = (
        cq.Workplane("XY")
        .box(0.086, 0.116, 0.016)
        .translate((0.0, 0.064, 0.010))
        .edges("|Z")
        .fillet(0.006)
    )

    # Two small through holes read as real payload mounting holes in the flange.
    payload_holes = cq.Workplane("XY")
    for x in (-0.024, 0.024):
        payload_holes = payload_holes.union(
            cq.Workplane("XY").center(x, 0.086).circle(0.0055).extrude(0.060, both=True)
        )
    flange = flange.cut(payload_holes)

    # Short web under the flange blends it into the hub without adding a second link.
    web = (
        cq.Workplane("YZ")
        .polyline([(0.006, -0.016), (0.072, 0.002), (0.072, 0.018), (0.006, 0.018)])
        .close()
        .extrude(0.038, both=True)
    )
    return flange.union(web)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_trunnion_module")

    cast_aluminum = model.material("matte_cast_aluminum", rgba=(0.34, 0.36, 0.36, 1.0))
    blue_anodized = model.material("blue_anodized_flange", rgba=(0.08, 0.22, 0.42, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.68, 0.64, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_fixed_housing_shape(), "fixed_housing", tolerance=0.0007),
        material=cast_aluminum,
        name="fixed_housing",
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_tilt_flange_shape(), "tilt_flange", tolerance=0.0007),
        material=blue_anodized,
        name="flange",
    )
    cradle.visual(
        Cylinder(radius=0.030, length=0.094),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_anodized,
        name="hub",
    )
    cradle.visual(
        Cylinder(radius=SHAFT_R, length=0.206),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="trunnion_pin",
    )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.50, upper=0.65, effort=18.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cradle = object_model.get_part("cradle")
    pitch = object_model.get_articulation("pitch_axis")

    ctx.check(
        "single pitch revolute joint",
        len(object_model.articulations) == 1
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in pitch.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_within(
        cradle,
        housing,
        axes="x",
        inner_elem="hub",
        outer_elem="fixed_housing",
        margin=0.002,
        name="hub fits between side cheeks",
    )
    ctx.expect_overlap(
        cradle,
        housing,
        axes="x",
        elem_a="trunnion_pin",
        elem_b="fixed_housing",
        min_overlap=0.150,
        name="trunnion pin spans both cheek bearings",
    )
    ctx.allow_overlap(
        cradle,
        housing,
        elem_a="trunnion_pin",
        elem_b="fixed_housing",
        reason=(
            "The moving trunnion shaft is intentionally captured through the "
            "fixed cheek bearing bores; the mesh proxy treats the seated shaft "
            "as local bearing overlap."
        ),
    )
    ctx.expect_contact(
        cradle,
        housing,
        elem_a="trunnion_pin",
        elem_b="fixed_housing",
        contact_tol=0.0005,
        name="trunnion pin rides in cheek bores",
    )

    rest_aabb = ctx.part_element_world_aabb(cradle, elem="flange")
    with ctx.pose({pitch: 0.55}):
        raised_aabb = ctx.part_element_world_aabb(cradle, elem="flange")

    with ctx.pose({pitch: -0.42}):
        lowered_aabb = ctx.part_element_world_aabb(cradle, elem="flange")
        lowered_min_z = lowered_aabb[0][2] if lowered_aabb is not None else None
        ctx.check(
            "lowered flange stays above mounting plate",
            lowered_min_z is not None and lowered_min_z > PLATE_Z + 0.006,
            details=f"lowered flange aabb={lowered_aabb}",
        )

    rest_center_z = None if rest_aabb is None else 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
    raised_center_z = None if raised_aabb is None else 0.5 * (raised_aabb[0][2] + raised_aabb[1][2])
    ctx.check(
        "positive pitch raises the cradle nose",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.025,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
