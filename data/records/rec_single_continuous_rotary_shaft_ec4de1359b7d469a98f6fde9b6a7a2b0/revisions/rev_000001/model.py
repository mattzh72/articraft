from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


PLATE_LENGTH = 0.34
PLATE_WIDTH = 0.12
PLATE_THICKNESS = 0.016

BEARING_CENTER_SPAN = 0.24
HANGER_DROP = 0.072
BEARING_FLANGE_LENGTH = 0.078
BEARING_FLANGE_WIDTH = 0.050
BEARING_FLANGE_THICKNESS = 0.010
BEARING_LENGTH = 0.060
WEB_LENGTH_X = 0.020
WEB_THICKNESS_Y = 0.010
BEARING_CAP_RADIUS = 0.022

SHAFT_RADIUS = 0.016
SHAFT_LENGTH = 0.400
SHAFT_HUB_RADIUS = 0.024
SHAFT_HUB_LENGTH = 0.080
COLLAR_RADIUS = 0.022
COLLAR_LENGTH = 0.012
COLLAR_CENTER_X = BEARING_CENTER_SPAN / 2.0 + BEARING_LENGTH / 2.0 + COLLAR_LENGTH / 2.0

DRIVE_LUG_SIZE = (0.032, 0.012, 0.018)
DRIVE_LUG_CENTER = (0.178, 0.021, 0.0)


def _top_bracket_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)
        .edges("|Z")
        .fillet(0.004)
    )

    hole_x = PLATE_LENGTH * 0.34
    hole_y = PLATE_WIDTH * 0.27
    holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-hole_x, -hole_y),
                (-hole_x, hole_y),
                (hole_x, -hole_y),
                (hole_x, hole_y),
            ]
        )
        .circle(0.006)
        .extrude(PLATE_THICKNESS * 2.0, both=True)
    )
    return plate.cut(holes)
def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_rotary_shaft_unit")

    model.material("bracket_gray", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("bearing_blue", rgba=(0.16, 0.28, 0.48, 1.0))
    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_steel", rgba=(0.36, 0.38, 0.41, 1.0))

    bracket = model.part("top_bracket")
    bracket.visual(
        mesh_from_cadquery(_top_bracket_shape(), "top_bracket"),
        origin=Origin(xyz=(0.0, 0.0, HANGER_DROP + PLATE_THICKNESS / 2.0)),
        material="bracket_gray",
        name="plate",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, HANGER_DROP + PLATE_THICKNESS / 2.0)),
    )

    left_bearing = model.part("left_bearing")
    left_bearing.visual(
        Box((BEARING_FLANGE_LENGTH, BEARING_FLANGE_WIDTH, BEARING_FLANGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, HANGER_DROP - BEARING_FLANGE_THICKNESS / 2.0)),
        material="bearing_blue",
        name="mount_flange",
    )
    left_bearing.visual(
        Box((WEB_LENGTH_X, WEB_THICKNESS_Y, HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                SHAFT_RADIUS + WEB_THICKNESS_Y / 2.0,
                (HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002) / 2.0,
            )
        ),
        material="bearing_blue",
        name="front_cheek",
    )
    left_bearing.visual(
        Box((WEB_LENGTH_X, WEB_THICKNESS_Y, HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                -(SHAFT_RADIUS + WEB_THICKNESS_Y / 2.0),
                (HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002) / 2.0,
            )
        ),
        material="bearing_blue",
        name="rear_cheek",
    )
    left_bearing.visual(
        Cylinder(radius=BEARING_CAP_RADIUS, length=BEARING_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, SHAFT_RADIUS + BEARING_CAP_RADIUS),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bearing_blue",
        name="journal_cap",
    )
    left_bearing.inertial = Inertial.from_geometry(
        Box((BEARING_FLANGE_LENGTH, BEARING_FLANGE_WIDTH, HANGER_DROP)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, HANGER_DROP / 2.0)),
    )

    right_bearing = model.part("right_bearing")
    right_bearing.visual(
        Box((BEARING_FLANGE_LENGTH, BEARING_FLANGE_WIDTH, BEARING_FLANGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, HANGER_DROP - BEARING_FLANGE_THICKNESS / 2.0)),
        material="bearing_blue",
        name="mount_flange",
    )
    right_bearing.visual(
        Box((WEB_LENGTH_X, WEB_THICKNESS_Y, HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                SHAFT_RADIUS + WEB_THICKNESS_Y / 2.0,
                (HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002) / 2.0,
            )
        ),
        material="bearing_blue",
        name="front_cheek",
    )
    right_bearing.visual(
        Box((WEB_LENGTH_X, WEB_THICKNESS_Y, HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                -(SHAFT_RADIUS + WEB_THICKNESS_Y / 2.0),
                (HANGER_DROP - BEARING_FLANGE_THICKNESS + 0.002) / 2.0,
            )
        ),
        material="bearing_blue",
        name="rear_cheek",
    )
    right_bearing.visual(
        Cylinder(radius=BEARING_CAP_RADIUS, length=BEARING_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, SHAFT_RADIUS + BEARING_CAP_RADIUS),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="bearing_blue",
        name="journal_cap",
    )
    right_bearing.inertial = Inertial.from_geometry(
        Box((BEARING_FLANGE_LENGTH, BEARING_FLANGE_WIDTH, HANGER_DROP)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, HANGER_DROP / 2.0)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=SHAFT_HUB_RADIUS, length=SHAFT_HUB_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="shaft_hub",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(-COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="left_collar",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="right_collar",
    )
    shaft.visual(
        Box(DRIVE_LUG_SIZE),
        origin=Origin(xyz=DRIVE_LUG_CENTER),
        material="dark_steel",
        name="drive_lug",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((SHAFT_LENGTH, SHAFT_HUB_RADIUS * 2.0, SHAFT_HUB_RADIUS * 2.0)),
        mass=1.1,
        origin=Origin(),
    )

    model.articulation(
        "bracket_to_left_bearing",
        ArticulationType.FIXED,
        parent=bracket,
        child=left_bearing,
        origin=Origin(xyz=(-BEARING_CENTER_SPAN / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "bracket_to_right_bearing",
        ArticulationType.FIXED,
        parent=bracket,
        child=right_bearing,
        origin=Origin(xyz=(BEARING_CENTER_SPAN / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "bracket_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=bracket,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("top_bracket")
    left_bearing = object_model.get_part("left_bearing")
    right_bearing = object_model.get_part("right_bearing")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("bracket_to_shaft")

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
        left_bearing,
        bracket,
        name="left bearing is mounted to the top bracket",
    )
    ctx.expect_contact(
        right_bearing,
        bracket,
        name="right bearing is mounted to the top bracket",
    )
    ctx.expect_contact(
        shaft,
        left_bearing,
        name="shaft is supported by the left bearing",
    )
    ctx.expect_contact(
        shaft,
        right_bearing,
        name="shaft is supported by the right bearing",
    )
    ctx.expect_gap(
        bracket,
        shaft,
        axis="z",
        min_gap=0.035,
        name="shaft hangs clearly below the support bracket",
    )
    ctx.check(
        "shaft articulation is continuous about the supported x-axis",
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in shaft_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={shaft_spin.articulation_type}, axis={shaft_spin.axis}",
    )

    with ctx.pose({shaft_spin: 0.0}):
        lug_rest = _aabb_center(ctx.part_element_world_aabb(shaft, elem="drive_lug"))
    with ctx.pose({shaft_spin: math.pi / 2.0}):
        lug_quarter_turn = _aabb_center(ctx.part_element_world_aabb(shaft, elem="drive_lug"))
        ctx.expect_gap(
            bracket,
            shaft,
            axis="z",
            min_gap=0.030,
            name="raised drive lug still clears the bracket",
        )

    ctx.check(
        "positive shaft rotation lifts the off-axis drive lug around the shaft centerline",
        lug_rest is not None
        and lug_quarter_turn is not None
        and lug_rest[1] > 0.010
        and abs(lug_rest[2]) < 0.010
        and abs(lug_quarter_turn[1]) < 0.010
        and lug_quarter_turn[2] > 0.014
        and abs(lug_quarter_turn[0] - lug_rest[0]) < 0.002,
        details=f"rest={lug_rest}, quarter_turn={lug_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
