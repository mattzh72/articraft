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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_SPAN = 0.220
SUPPORT_X = SUPPORT_SPAN / 2.0
TOP_PLATE_Z = 0.078
TOP_PLATE_LENGTH = 0.300
TOP_PLATE_DEPTH = 0.100
TOP_PLATE_THICKNESS = 0.012

SUPPORT_THICKNESS = 0.018
SUPPORT_DEPTH = 0.078
SUPPORT_HEIGHT = 0.122
SUPPORT_Z = 0.018
SUPPORT_CHEEK_DEPTH = 0.020
SUPPORT_CHEEK_Y = 0.029
SUPPORT_CAP_DEPTH = 0.058
SUPPORT_CAP_HEIGHT = 0.018
SUPPORT_UPPER_CAP_Z = 0.033
SUPPORT_LOWER_CAP_Z = -0.033

SHAFT_RADIUS = 0.0105
SHAFT_LENGTH = 0.260
HUB_RADIUS = 0.025
HUB_LENGTH = 0.052
FLANGE_RADIUS = 0.055
FLANGE_THICKNESS = 0.014
PILOT_RADIUS = 0.018
PILOT_LENGTH = 0.012
PILOT_X = FLANGE_THICKNESS / 2.0 + PILOT_LENGTH / 2.0 - 0.001
COLLAR_RADIUS = 0.022
COLLAR_THICKNESS = 0.005
COLLAR_X = SUPPORT_X + SUPPORT_THICKNESS / 2.0 + COLLAR_THICKNESS / 2.0
BOLT_CIRCLE_RADIUS = 0.036
BOLT_HOLE_RADIUS = 0.0042


def _add_mesh_visual(part, shape: cq.Workplane, name: str, material: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, name),
        origin=Origin(),
        material=material,
        name=name,
    )


def _make_top_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(TOP_PLATE_LENGTH, TOP_PLATE_DEPTH, TOP_PLATE_THICKNESS)
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.090, 0.0), (0.090, 0.0)])
        .slot2D(0.042, 0.014, angle=0.0)
        .cutThruAll()
    )
    return plate.translate((0.0, 0.0, TOP_PLATE_Z))


def _make_support_block(
    x_center: float,
    y_center: float,
    z_center: float,
    y_size: float,
    z_size: float,
) -> cq.Workplane:
    return cq.Workplane("XY").box(SUPPORT_THICKNESS, y_size, z_size).translate(
        (x_center, y_center, z_center)
    )


def _bolt_circle_points(radius: float, count: int) -> list[tuple[float, float]]:
    return [
        (radius * cos((2.0 * pi * idx) / count), radius * sin((2.0 * pi * idx) / count))
        for idx in range(count)
    ]


def _make_axial_cylinder(radius: float, length: float, x_center: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x_center, 0.0, 0.0))
    )


def _make_flange() -> cq.Workplane:
    flange = _make_axial_cylinder(FLANGE_RADIUS, FLANGE_THICKNESS)
    flange_holes = (
        cq.Workplane("YZ")
        .pushPoints(_bolt_circle_points(BOLT_CIRCLE_RADIUS, 6))
        .circle(BOLT_HOLE_RADIUS)
        .extrude((FLANGE_THICKNESS + 0.020) / 2.0, both=True)
    )
    return flange.cut(flange_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_roll_axis_spindle")

    model.material("painted_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.75, 0.78, 1.0))

    bracket = model.part("support_bracket")
    spindle = model.part("spindle")

    _add_mesh_visual(bracket, _make_top_plate(), "top_plate", "painted_steel")
    for side_name, x_center in (("left", -SUPPORT_X), ("right", SUPPORT_X)):
        _add_mesh_visual(
            bracket,
            _make_support_block(
                x_center,
                SUPPORT_CHEEK_Y,
                SUPPORT_Z,
                SUPPORT_CHEEK_DEPTH,
                SUPPORT_HEIGHT,
            ),
            f"{side_name}_front_cheek",
            "painted_steel",
        )
        _add_mesh_visual(
            bracket,
            _make_support_block(
                x_center,
                -SUPPORT_CHEEK_Y,
                SUPPORT_Z,
                SUPPORT_CHEEK_DEPTH,
                SUPPORT_HEIGHT,
            ),
            f"{side_name}_rear_cheek",
            "painted_steel",
        )
        _add_mesh_visual(
            bracket,
            _make_support_block(
                x_center,
                0.0,
                SUPPORT_UPPER_CAP_Z,
                SUPPORT_CAP_DEPTH,
                SUPPORT_CAP_HEIGHT,
            ),
            f"{side_name}_upper_cap",
            "painted_steel",
        )
        _add_mesh_visual(
            bracket,
            _make_support_block(
                x_center,
                0.0,
                SUPPORT_LOWER_CAP_Z,
                SUPPORT_CAP_DEPTH,
                SUPPORT_CAP_HEIGHT,
            ),
            f"{side_name}_lower_cap",
            "painted_steel",
        )
    bracket.inertial = Inertial.from_geometry(
        Box((0.300, 0.100, 0.130)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    _add_mesh_visual(spindle, _make_axial_cylinder(SHAFT_RADIUS, SHAFT_LENGTH), "shaft_core", "machined_steel")
    _add_mesh_visual(spindle, _make_axial_cylinder(HUB_RADIUS, HUB_LENGTH), "drive_hub", "machined_steel")
    _add_mesh_visual(spindle, _make_flange(), "output_flange", "machined_steel")
    _add_mesh_visual(
        spindle,
        _make_axial_cylinder(PILOT_RADIUS, PILOT_LENGTH, PILOT_X),
        "pilot_register",
        "machined_steel",
    )
    _add_mesh_visual(
        spindle,
        _make_axial_cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, -COLLAR_X),
        "left_retainer",
        "machined_steel",
    )
    _add_mesh_visual(
        spindle,
        _make_axial_cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, COLLAR_X),
        "right_retainer",
        "machined_steel",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((0.260, 0.110, 0.110)),
        mass=1.7,
        origin=Origin(),
    )

    model.articulation(
        "bracket_to_spindle",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("support_bracket")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("bracket_to_spindle")

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

    ctx.expect_contact(
        spindle,
        bracket,
        contact_tol=0.0005,
        name="spindle is captured by the fixed supports",
    )
    ctx.expect_gap(
        bracket,
        spindle,
        axis="z",
        positive_elem="top_plate",
        min_gap=0.012,
        max_gap=0.030,
        name="top plate hangs above the under-slung spindle",
    )
    ctx.check(
        "roll joint aligns with the spindle shaft axis",
        tuple(roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll.axis}",
    )

    limits = roll.motion_limits
    ctx.check(
        "roll joint has symmetric travel about zero",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and abs(limits.lower + limits.upper) < 1e-6,
        details=f"limits={limits}",
    )

    rest_position = ctx.part_world_position(spindle)
    with ctx.pose({roll: 1.4}):
        turned_position = ctx.part_world_position(spindle)
        ctx.expect_contact(
            spindle,
            bracket,
            contact_tol=0.0005,
            name="spindle remains supported while rolling",
        )

    ctx.check(
        "spindle rotates in place about its own axis",
        rest_position is not None
        and turned_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, turned_position)) <= 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
