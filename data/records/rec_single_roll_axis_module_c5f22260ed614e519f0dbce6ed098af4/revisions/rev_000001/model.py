from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.012
PLATE_W = 0.150
PLATE_H = 0.220
PLATE_HOLE_R = 0.007
PLATE_HOLE_Y = 0.052
PLATE_HOLE_Z = 0.078

PAD_L = 0.020
PAD_W = 0.084
PAD_H = 0.094

REAR_HOUSING_R = 0.032
REAR_HOUSING_L = 0.032
SLEEVE_R = 0.022
SLEEVE_L = 0.048

FACE_X0 = REAR_HOUSING_L + SLEEVE_L
FACE_T = 0.010
FACE_W = 0.086
FACE_H = 0.110
FACE_HOLE_R = 0.003
FACE_HOLE_Y = 0.024
FACE_HOLE_Z = 0.034

WEB_W = 0.024
WEB_T = 0.020
WEB_Z = 0.043

BORE_R = 0.0158

SHAFT_R = 0.0142
SHAFT_L = FACE_X0 + FACE_T
FLANGE_R = 0.036
FLANGE_T = 0.014
FLANGE_BOLT_R = 0.0038
FLANGE_BOLT_PCD_R = 0.024
INDEX_PIN_R = 0.0045
INDEX_PIN_T = 0.008
INDEX_PIN_OFFSET_Y = 0.023


def _support_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H).translate((-PLATE_T / 2.0, 0.0, 0.0))
    pad = cq.Workplane("XY").box(PAD_L, PAD_W, PAD_H, centered=(False, True, True))
    rear_housing = cq.Workplane("YZ").circle(REAR_HOUSING_R).extrude(REAR_HOUSING_L)
    sleeve = cq.Workplane("YZ").circle(SLEEVE_R).extrude(SLEEVE_L).translate((REAR_HOUSING_L, 0.0, 0.0))
    faceplate = cq.Workplane("XY").box(FACE_T, FACE_W, FACE_H, centered=(False, True, True)).translate((FACE_X0, 0.0, 0.0))
    top_web = cq.Workplane("XY").box(FACE_X0, WEB_W, WEB_T, centered=(False, True, True)).translate((0.0, 0.0, WEB_Z))
    bottom_web = cq.Workplane("XY").box(FACE_X0, WEB_W, WEB_T, centered=(False, True, True)).translate((0.0, 0.0, -WEB_Z))

    support = (
        plate.union(pad)
        .union(rear_housing)
        .union(sleeve)
        .union(faceplate)
        .union(top_web)
        .union(bottom_web)
    )

    bore = (
        cq.Workplane("YZ")
        .circle(BORE_R)
        .extrude(PLATE_T + FACE_X0 + FACE_T)
        .translate((-PLATE_T, 0.0, 0.0))
    )
    support = support.cut(bore)

    plate_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-PLATE_HOLE_Y, -PLATE_HOLE_Z),
                (-PLATE_HOLE_Y, PLATE_HOLE_Z),
                (PLATE_HOLE_Y, -PLATE_HOLE_Z),
                (PLATE_HOLE_Y, PLATE_HOLE_Z),
            ]
        )
        .circle(PLATE_HOLE_R)
        .extrude(PLATE_T + 0.002)
        .translate((-PLATE_T - 0.001, 0.0, 0.0))
    )
    support = support.cut(plate_holes)

    face_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-FACE_HOLE_Y, -FACE_HOLE_Z),
                (-FACE_HOLE_Y, FACE_HOLE_Z),
                (FACE_HOLE_Y, -FACE_HOLE_Z),
                (FACE_HOLE_Y, FACE_HOLE_Z),
            ]
        )
        .circle(FACE_HOLE_R)
        .extrude(FACE_T + 0.002)
        .translate((FACE_X0 - 0.001, 0.0, 0.0))
    )
    return support.cut(face_holes)


def _flange_shape() -> cq.Workplane:
    flange = cq.Workplane("YZ").circle(FLANGE_R).extrude(FLANGE_T).translate((SHAFT_L, 0.0, 0.0))
    bolt_holes = []
    for angle in (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0):
        bolt_holes.append(
            (
                FLANGE_BOLT_PCD_R * cos(angle),
                FLANGE_BOLT_PCD_R * sin(angle),
            )
        )
    cutters = (
        cq.Workplane("YZ")
        .pushPoints(bolt_holes)
        .circle(FLANGE_BOLT_R)
        .extrude(FLANGE_T + 0.002)
        .translate((SHAFT_L - 0.001, 0.0, 0.0))
    )
    return flange.cut(cutters)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mn, mx = aabb
    return tuple((a + b) / 2.0 for a, b in zip(mn, mx))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_spindle_module")

    model.material("support_gray", rgba=(0.31, 0.33, 0.36, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("oxide_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_support_shape(), "support_shell"),
        material="support_gray",
        name="support_shell",
    )
    support.inertial = Inertial.from_geometry(
        Box((PLATE_T + FACE_X0 + FACE_T, PLATE_W, PLATE_H)),
        mass=4.8,
        origin=Origin(xyz=((FACE_X0 + FACE_T - PLATE_T) / 2.0, 0.0, 0.0)),
    )

    output_flange = model.part("output_flange")
    output_flange.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_L),
        origin=Origin(xyz=(SHAFT_L / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="shaft_body",
    )
    output_flange.visual(
        mesh_from_cadquery(_flange_shape(), "rotor_flange"),
        material="machined_steel",
        name="flange_body",
    )
    output_flange.visual(
        Cylinder(radius=INDEX_PIN_R, length=INDEX_PIN_T),
        origin=Origin(
            xyz=(SHAFT_L + FLANGE_T + INDEX_PIN_T / 2.0, INDEX_PIN_OFFSET_Y, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="oxide_dark",
        name="index_pin",
    )
    output_flange.inertial = Inertial.from_geometry(
        Box((SHAFT_L + FLANGE_T + INDEX_PIN_T, 2.0 * FLANGE_R, 2.0 * FLANGE_R)),
        mass=0.9,
        origin=Origin(xyz=((SHAFT_L + FLANGE_T + INDEX_PIN_T) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_output",
        ArticulationType.REVOLUTE,
        parent=support,
        child=output_flange,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=12.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    output_flange = object_model.get_part("output_flange")
    spin = object_model.get_articulation("support_to_output")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "spin axis is longitudinal x",
        tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_within(
        output_flange,
        support,
        axes="yz",
        inner_elem="shaft_body",
        margin=0.0,
        name="shaft stays inside support footprint",
    )
    ctx.expect_overlap(
        output_flange,
        support,
        axes="yz",
        elem_a="flange_body",
        min_overlap=0.065,
        name="output flange remains centered on support face",
    )
    ctx.expect_gap(
        output_flange,
        support,
        axis="x",
        positive_elem="flange_body",
        max_gap=1e-6,
        max_penetration=0.0,
        name="output flange bears against carried faceplate",
    )
    ctx.expect_contact(
        output_flange,
        support,
        elem_a="flange_body",
        elem_b="support_shell",
        name="support path reaches output flange",
    )

    with ctx.pose({spin: 0.0}):
        pin_aabb_closed = ctx.part_element_world_aabb(output_flange, elem="index_pin")
    with ctx.pose({spin: pi / 2.0}):
        pin_aabb_quarter = ctx.part_element_world_aabb(output_flange, elem="index_pin")

    if pin_aabb_closed is None or pin_aabb_quarter is None:
        ctx.fail("index pin orbits around shaft axis", "index pin AABB could not be resolved")
    else:
        pin_center_closed = _aabb_center(pin_aabb_closed)
        pin_center_quarter = _aabb_center(pin_aabb_quarter)
        ctx.check(
            "index pin orbits around shaft axis",
            abs(pin_center_closed[0] - pin_center_quarter[0]) < 1e-4
            and pin_center_closed[1] > INDEX_PIN_OFFSET_Y - 0.003
            and abs(pin_center_closed[2]) < 0.003
            and abs(pin_center_quarter[1]) < 0.003
            and pin_center_quarter[2] > INDEX_PIN_OFFSET_Y - 0.003,
            details=f"closed={pin_center_closed}, quarter_turn={pin_center_quarter}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
