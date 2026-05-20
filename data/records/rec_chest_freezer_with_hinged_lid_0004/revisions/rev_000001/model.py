from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

HERE = Path(__file__).resolve().parent
ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.68
BODY_DEPTH = 0.56
BODY_HEIGHT = 0.84
OUTER_WALL = 0.032
BASE_THICKNESS = 0.075

LINER_OUTER_WIDTH = 0.596
LINER_OUTER_DEPTH = 0.482
LINER_WALL = 0.018
LINER_FLOOR = 0.018
LINER_HEIGHT = BODY_HEIGHT - BASE_THICKNESS - 0.055

HINGE_RADIUS = 0.012
HINGE_AXIS_Y = BODY_DEPTH / 2.0 + 0.016
HINGE_AXIS_Z = BODY_HEIGHT + 0.028

BODY_HINGE_X = 0.258
BODY_HINGE_BARREL_LENGTH = 0.046
LID_HINGE_X = 0.214
LID_HINGE_BARREL_LENGTH = 0.042

LID_WIDTH = BODY_WIDTH + 0.016
LID_DEPTH = BODY_DEPTH + 0.016
LID_THICKNESS = 0.065
LID_TOP_THICKNESS = 0.016
LID_BOTTOM_THICKNESS = 0.012
LID_AXIS_ABOVE_BOTTOM = 0.028
LID_BOTTOM_Z = -LID_AXIS_ABOVE_BOTTOM
LID_CENTER_Z = LID_BOTTOM_Z + LID_THICKNESS / 2.0
LID_CENTER_Y = -(0.018 + LID_DEPTH / 2.0)
LID_SIDE_SKIRT = 0.030
LID_FRONT_SKIRT = 0.030
LID_SKIRT_HEIGHT = LID_THICKNESS - LID_TOP_THICKNESS
LID_SKIRT_CENTER_Z = LID_BOTTOM_Z + LID_SKIRT_HEIGHT / 2.0
LID_UNDERSIDE_WIDTH = LID_WIDTH - 2.0 * LID_SIDE_SKIRT
LID_UNDERSIDE_DEPTH = LID_DEPTH - LID_FRONT_SKIRT
LID_UNDERSIDE_CENTER_Y = (-0.018 + -(0.018 + LID_DEPTH - LID_FRONT_SKIRT)) / 2.0
LID_UNDERSIDE_CENTER_Z = LID_BOTTOM_Z + LID_BOTTOM_THICKNESS / 2.0

HANDLE_OPENING_WIDTH = 0.20
HANDLE_OPENING_DEPTH = 0.082
HANDLE_FRONT_MARGIN = 0.086
HANDLE_CENTER_Y = -(0.018 + LID_DEPTH) + HANDLE_FRONT_MARGIN + HANDLE_OPENING_DEPTH / 2.0
HANDLE_SIDE_STRIP = (LID_WIDTH - HANDLE_OPENING_WIDTH) / 2.0
TOP_FRONT_PANEL_DEPTH = HANDLE_FRONT_MARGIN
TOP_REAR_PANEL_DEPTH = LID_DEPTH - TOP_FRONT_PANEL_DEPTH - HANDLE_OPENING_DEPTH

HANDLE_RECESS_DEPTH = 0.030
HANDLE_FLOOR_THICKNESS = 0.010
HANDLE_FLOOR_WIDTH = 0.18
HANDLE_FLOOR_DEPTH = HANDLE_OPENING_DEPTH - 2.0 * 0.010
HANDLE_WALL_THICKNESS = 0.010
HANDLE_FLOOR_TOP_Z = (LID_THICKNESS - LID_AXIS_ABOVE_BOTTOM) - HANDLE_RECESS_DEPTH
HANDLE_FLOOR_CENTER_Z = HANDLE_FLOOR_TOP_Z - HANDLE_FLOOR_THICKNESS / 2.0
HANDLE_WALL_HEIGHT = (LID_THICKNESS - LID_AXIS_ABOVE_BOTTOM - LID_TOP_THICKNESS) - HANDLE_FLOOR_TOP_Z
HANDLE_WALL_CENTER_Z = HANDLE_FLOOR_TOP_Z + HANDLE_WALL_HEIGHT / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_chest_freezer", assets=ASSETS)

    white = model.material("appliance_white", rgba=(0.95, 0.96, 0.97, 1.0))
    liner = model.material("liner_gray", rgba=(0.87, 0.89, 0.90, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.60, 0.63, 0.67, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=white,
        name="base_plinth",
    )
    body.visual(
        Box((OUTER_WALL, BODY_DEPTH, BODY_HEIGHT - BASE_THICKNESS)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH / 2.0 - OUTER_WALL / 2.0),
                0.0,
                BASE_THICKNESS + (BODY_HEIGHT - BASE_THICKNESS) / 2.0,
            )
        ),
        material=white,
        name="left_side",
    )
    body.visual(
        Box((OUTER_WALL, BODY_DEPTH, BODY_HEIGHT - BASE_THICKNESS)),
        origin=Origin(
            xyz=(
                BODY_WIDTH / 2.0 - OUTER_WALL / 2.0,
                0.0,
                BASE_THICKNESS + (BODY_HEIGHT - BASE_THICKNESS) / 2.0,
            )
        ),
        material=white,
        name="right_side",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * OUTER_WALL, OUTER_WALL, BODY_HEIGHT - BASE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH / 2.0 - OUTER_WALL / 2.0),
                BASE_THICKNESS + (BODY_HEIGHT - BASE_THICKNESS) / 2.0,
            )
        ),
        material=white,
        name="front_side",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * OUTER_WALL, OUTER_WALL, BODY_HEIGHT - BASE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH / 2.0 - OUTER_WALL / 2.0,
                BASE_THICKNESS + (BODY_HEIGHT - BASE_THICKNESS) / 2.0,
            )
        ),
        material=white,
        name="rear_side",
    )
    body.visual(
        Box((LINER_OUTER_WIDTH, LINER_OUTER_DEPTH, LINER_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + LINER_FLOOR / 2.0)),
        material=liner,
        name="liner_floor",
    )
    body.visual(
        Box((LINER_WALL, LINER_OUTER_DEPTH, LINER_HEIGHT)),
        origin=Origin(
            xyz=(
                -(LINER_OUTER_WIDTH / 2.0 - LINER_WALL / 2.0),
                0.0,
                BASE_THICKNESS + LINER_FLOOR + LINER_HEIGHT / 2.0,
            )
        ),
        material=liner,
        name="liner_left_side",
    )
    body.visual(
        Box((LINER_WALL, LINER_OUTER_DEPTH, LINER_HEIGHT)),
        origin=Origin(
            xyz=(
                LINER_OUTER_WIDTH / 2.0 - LINER_WALL / 2.0,
                0.0,
                BASE_THICKNESS + LINER_FLOOR + LINER_HEIGHT / 2.0,
            )
        ),
        material=liner,
        name="liner_right_side",
    )
    body.visual(
        Box((LINER_OUTER_WIDTH - 2.0 * LINER_WALL, LINER_WALL, LINER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(LINER_OUTER_DEPTH / 2.0 - LINER_WALL / 2.0),
                BASE_THICKNESS + LINER_FLOOR + LINER_HEIGHT / 2.0,
            )
        ),
        material=liner,
        name="liner_front_side",
    )
    body.visual(
        Box((LINER_OUTER_WIDTH - 2.0 * LINER_WALL, LINER_WALL, LINER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                LINER_OUTER_DEPTH / 2.0 - LINER_WALL / 2.0,
                BASE_THICKNESS + LINER_FLOOR + LINER_HEIGHT / 2.0,
            )
        ),
        material=liner,
        name="liner_rear_side",
    )
    body.visual(
        Box((0.040, 0.032, 0.034)),
        origin=Origin(
            xyz=(-BODY_HINGE_X, BODY_DEPTH / 2.0 + 0.010, BODY_HEIGHT + 0.005)
        ),
        material=hinge_metal,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.040, 0.032, 0.034)),
        origin=Origin(
            xyz=(BODY_HINGE_X, BODY_DEPTH / 2.0 + 0.010, BODY_HEIGHT + 0.005)
        ),
        material=hinge_metal,
        name="right_hinge_bracket",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(-BODY_HINGE_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="left_body_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(BODY_HINGE_X, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="right_body_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_WIDTH, TOP_REAR_PANEL_DEPTH, LID_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -0.018 - TOP_REAR_PANEL_DEPTH / 2.0,
                LID_THICKNESS - LID_AXIS_ABOVE_BOTTOM - LID_TOP_THICKNESS / 2.0,
            )
        ),
        material=white,
        name="top_rear_panel",
    )
    lid.visual(
        Box((LID_WIDTH, TOP_FRONT_PANEL_DEPTH, LID_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(0.018 + LID_DEPTH) + TOP_FRONT_PANEL_DEPTH / 2.0,
                LID_THICKNESS - LID_AXIS_ABOVE_BOTTOM - LID_TOP_THICKNESS / 2.0,
            )
        ),
        material=white,
        name="top_front_panel",
    )
    lid.visual(
        Box((HANDLE_SIDE_STRIP, HANDLE_OPENING_DEPTH, LID_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                -(HANDLE_OPENING_WIDTH / 2.0 + HANDLE_SIDE_STRIP / 2.0),
                HANDLE_CENTER_Y,
                LID_THICKNESS - LID_AXIS_ABOVE_BOTTOM - LID_TOP_THICKNESS / 2.0,
            )
        ),
        material=white,
        name="top_left_panel",
    )
    lid.visual(
        Box((HANDLE_SIDE_STRIP, HANDLE_OPENING_DEPTH, LID_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                HANDLE_OPENING_WIDTH / 2.0 + HANDLE_SIDE_STRIP / 2.0,
                HANDLE_CENTER_Y,
                LID_THICKNESS - LID_AXIS_ABOVE_BOTTOM - LID_TOP_THICKNESS / 2.0,
            )
        ),
        material=white,
        name="top_right_panel",
    )
    lid.visual(
        Box((LID_UNDERSIDE_WIDTH, LID_UNDERSIDE_DEPTH, LID_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(0.0, LID_UNDERSIDE_CENTER_Y, LID_UNDERSIDE_CENTER_Z)
        ),
        material=white,
        name="underside_panel",
    )
    lid.visual(
        Box((LID_SIDE_SKIRT, LID_DEPTH, LID_SKIRT_HEIGHT)),
        origin=Origin(
            xyz=(
                -(LID_WIDTH / 2.0 - LID_SIDE_SKIRT / 2.0),
                LID_CENTER_Y,
                LID_SKIRT_CENTER_Z,
            )
        ),
        material=white,
        name="left_skirt",
    )
    lid.visual(
        Box((LID_SIDE_SKIRT, LID_DEPTH, LID_SKIRT_HEIGHT)),
        origin=Origin(
            xyz=(
                LID_WIDTH / 2.0 - LID_SIDE_SKIRT / 2.0,
                LID_CENTER_Y,
                LID_SKIRT_CENTER_Z,
            )
        ),
        material=white,
        name="right_skirt",
    )
    lid.visual(
        Box((LID_WIDTH - 2.0 * LID_SIDE_SKIRT, LID_FRONT_SKIRT, LID_SKIRT_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(0.018 + LID_DEPTH) + LID_FRONT_SKIRT / 2.0,
                LID_SKIRT_CENTER_Z,
            )
        ),
        material=white,
        name="front_skirt",
    )
    lid.visual(
        Box((HANDLE_FLOOR_WIDTH, HANDLE_FLOOR_DEPTH, HANDLE_FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(0.0, HANDLE_CENTER_Y, HANDLE_FLOOR_CENTER_Z)
        ),
        material=handle_gray,
        name="handle_floor",
    )
    lid.visual(
        Box((HANDLE_FLOOR_WIDTH, HANDLE_WALL_THICKNESS, HANDLE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HANDLE_CENTER_Y
                - (HANDLE_OPENING_DEPTH / 2.0 - HANDLE_WALL_THICKNESS / 2.0),
                HANDLE_WALL_CENTER_Z,
            )
        ),
        material=handle_gray,
        name="handle_front_wall",
    )
    lid.visual(
        Box((HANDLE_FLOOR_WIDTH, HANDLE_WALL_THICKNESS, HANDLE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HANDLE_CENTER_Y
                + (HANDLE_OPENING_DEPTH / 2.0 - HANDLE_WALL_THICKNESS / 2.0),
                HANDLE_WALL_CENTER_Z,
            )
        ),
        material=handle_gray,
        name="handle_back_wall",
    )
    lid.visual(
        Box((HANDLE_WALL_THICKNESS, HANDLE_OPENING_DEPTH, HANDLE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(HANDLE_OPENING_WIDTH / 2.0 - HANDLE_WALL_THICKNESS / 2.0),
                HANDLE_CENTER_Y,
                HANDLE_WALL_CENTER_Z,
            )
        ),
        material=handle_gray,
        name="handle_left_wall",
    )
    lid.visual(
        Box((HANDLE_WALL_THICKNESS, HANDLE_OPENING_DEPTH, HANDLE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                HANDLE_OPENING_WIDTH / 2.0 - HANDLE_WALL_THICKNESS / 2.0,
                HANDLE_CENTER_Y,
                HANDLE_WALL_CENTER_Z,
            )
        ),
        material=handle_gray,
        name="handle_right_wall",
    )
    lid.visual(
        Box((0.038, 0.036, 0.022)),
        origin=Origin(xyz=(-LID_HINGE_X, -0.018, 0.011)),
        material=hinge_metal,
        name="left_hinge_bracket",
    )
    lid.visual(
        Box((0.038, 0.036, 0.022)),
        origin=Origin(xyz=(LID_HINGE_X, -0.018, 0.011)),
        material=hinge_metal,
        name="right_hinge_bracket",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=LID_HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(-LID_HINGE_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="left_lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=LID_HINGE_BARREL_LENGTH),
        origin=Origin(
            xyz=(LID_HINGE_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hinge_metal,
        name="right_lid_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        mass=4.0,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_CENTER_Z)),
    )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6, lower=-1.32, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rear_lid_hinge = object_model.get_articulation("rear_lid_hinge")
    limits = rear_lid_hinge.motion_limits

    front_side = body.get_visual("front_side")
    body_left_hinge = body.get_visual("left_body_hinge_barrel")
    body_right_hinge = body.get_visual("right_body_hinge_barrel")
    underside_panel = lid.get_visual("underside_panel")
    top_front_panel = lid.get_visual("top_front_panel")
    front_skirt = lid.get_visual("front_skirt")
    handle_floor = lid.get_visual("handle_floor")
    lid_left_hinge = lid.get_visual("left_lid_hinge_barrel")
    lid_right_hinge = lid.get_visual("right_lid_hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check("body_present", body.name == "body", "body part lookup failed")
    ctx.check("lid_present", lid.name == "lid", "lid part lookup failed")
    ctx.check(
        "rear_lid_hinge_axis",
        tuple(rear_lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis hinge, got {rear_lid_hinge.axis}",
    )
    ctx.check(
        "rear_lid_hinge_limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and -1.45 <= limits.lower <= -1.15
        and limits.upper == 0.0,
        "lid hinge should close at 0 rad and open to a realistic negative-angle raised pose",
    )

    with ctx.pose({rear_lid_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_closed_no_floating")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.40, name="lid_covers_body")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.003,
            max_penetration=0.001,
            positive_elem=front_skirt,
            negative_elem=front_side,
            name="lid_seats_on_body_rim",
        )
        ctx.expect_within(
            lid,
            lid,
            axes="xy",
            inner_elem=handle_floor,
            outer_elem=underside_panel,
            name="handle_recess_within_lid_footprint",
        )
        ctx.expect_gap(
            lid,
            lid,
            axis="z",
            min_gap=0.012,
            max_gap=0.020,
            positive_elem=top_front_panel,
            negative_elem=handle_floor,
            name="handle_recess_below_flat_lid_top",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=lid_left_hinge,
            elem_b=body_left_hinge,
            name="left_barrel_hinge_contact_closed",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=lid_right_hinge,
            elem_b=body_right_hinge,
            name="right_barrel_hinge_contact_closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            min_overlap=0.020,
            elem_a=lid_left_hinge,
            elem_b=body_left_hinge,
            name="left_barrel_hinge_alignment_closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            min_overlap=0.020,
            elem_a=lid_right_hinge,
            elem_b=body_right_hinge,
            name="right_barrel_hinge_alignment_closed",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({rear_lid_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_open_no_floating")
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                min_gap=0.18,
                positive_elem=front_skirt,
                negative_elem=front_side,
                name="lid_front_lifts_clear_when_open",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=lid_left_hinge,
                elem_b=body_left_hinge,
                name="left_barrel_hinge_contact_open",
            )
            ctx.expect_contact(
                lid,
                body,
                elem_a=lid_right_hinge,
                elem_b=body_right_hinge,
                name="right_barrel_hinge_contact_open",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="yz",
                min_overlap=0.020,
                elem_a=lid_left_hinge,
                elem_b=body_left_hinge,
                name="left_barrel_hinge_alignment_open",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="yz",
                min_overlap=0.020,
                elem_a=lid_right_hinge,
                elem_b=body_right_hinge,
                name="right_barrel_hinge_alignment_open",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
