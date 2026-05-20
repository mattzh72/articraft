from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.90
BODY_DEPTH = 0.62
BODY_HEIGHT = 0.88
WALL_THICKNESS = 0.03
BACK_THICKNESS = 0.02
INNER_WIDTH = BODY_WIDTH - 2.0 * WALL_THICKNESS
INNER_DEPTH_REAR = 0.55

COOKTOP_THICKNESS = 0.02
CONTROL_PANEL_WIDTH = INNER_WIDTH
CONTROL_PANEL_HEIGHT = 0.11
CONTROL_PANEL_THICKNESS = 0.05
BUTTON_SLOT_WIDTH = 0.046
BUTTON_SLOT_HEIGHT = 0.02
BUTTON_GROUP_WIDTH = 0.21
BUTTON_MULLION_WIDTH = (BUTTON_GROUP_WIDTH - 3.0 * BUTTON_SLOT_WIDTH) / 4.0
SIDE_CONTROL_ZONE_WIDTH = (CONTROL_PANEL_WIDTH - BUTTON_GROUP_WIDTH) / 2.0

DOOR_WIDTH = 0.82
DOOR_THICKNESS = 0.045
DOOR_HEIGHT = 0.65
DOOR_HINGE_RADIUS = 0.01


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_induction_range", assets=ASSETS)

    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.77, 0.80, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.11, 0.13, 1.0))
    panel_black = model.material("panel_black", rgba=(0.15, 0.16, 0.18, 1.0))
    control_white = model.material("control_white", rgba=(0.90, 0.91, 0.92, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.84, 0.85, 0.87, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=brushed_steel,
        name="left_wall",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - WALL_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=brushed_steel,
        name="right_wall",
    )
    cabinet.visual(
        Box((INNER_WIDTH, BACK_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - BACK_THICKNESS / 2.0, BODY_HEIGHT / 2.0)),
        material=brushed_steel,
        name="back_panel",
    )
    cabinet.visual(
        Box((INNER_WIDTH, BODY_DEPTH - BACK_THICKNESS, 0.02)),
        origin=Origin(xyz=(0.0, -0.01, 0.01)),
        material=brushed_steel,
        name="oven_floor",
    )
    cabinet.visual(
        Box((INNER_WIDTH, INNER_DEPTH_REAR, 0.02)),
        origin=Origin(xyz=(0.0, 0.015, BODY_HEIGHT - 0.01)),
        material=brushed_steel,
        name="oven_ceiling",
    )
    cabinet.visual(
        Box((INNER_WIDTH, CONTROL_PANEL_THICKNESS, 0.10)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + CONTROL_PANEL_THICKNESS / 2.0, 0.05)),
        material=brushed_steel,
        name="lower_rail",
    )
    cabinet.visual(
        Box((INNER_WIDTH, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 - 0.01, 0.09)),
        material=brushed_steel,
        name="hinge_shelf",
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((BODY_WIDTH, BODY_DEPTH, COOKTOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_THICKNESS / 2.0)),
        material=dark_glass,
        name="glass_top",
    )
    for zone_name, radius, x_pos, y_pos in (
        ("zone_front_left", 0.095, -0.23, -0.14),
        ("zone_rear_left", 0.105, -0.23, 0.12),
        ("zone_center", 0.135, 0.0, 0.02),
        ("zone_rear_right", 0.105, 0.23, 0.12),
        ("zone_front_right", 0.095, 0.23, -0.14),
    ):
        cooktop.visual(
            Cylinder(radius=radius, length=0.0015),
            origin=Origin(xyz=(x_pos, y_pos, COOKTOP_THICKNESS - 0.00025)),
            material=control_white,
            name=zone_name,
        )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((SIDE_CONTROL_ZONE_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(xyz=(-BUTTON_GROUP_WIDTH / 2.0 - SIDE_CONTROL_ZONE_WIDTH / 2.0, 0.0, 0.0)),
        material=panel_black,
        name="left_control_zone",
    )
    control_panel.visual(
        Box((SIDE_CONTROL_ZONE_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(xyz=(BUTTON_GROUP_WIDTH / 2.0 + SIDE_CONTROL_ZONE_WIDTH / 2.0, 0.0, 0.0)),
        material=panel_black,
        name="right_control_zone",
    )
    control_panel.visual(
        Box((BUTTON_GROUP_WIDTH, CONTROL_PANEL_THICKNESS, (CONTROL_PANEL_HEIGHT - BUTTON_SLOT_HEIGHT) / 2.0)),
        origin=Origin(
            xyz=(0.0, 0.0, CONTROL_PANEL_HEIGHT / 2.0 - (CONTROL_PANEL_HEIGHT - BUTTON_SLOT_HEIGHT) / 4.0)
        ),
        material=panel_black,
        name="button_row_top_rail",
    )
    control_panel.visual(
        Box((BUTTON_GROUP_WIDTH, CONTROL_PANEL_THICKNESS, (CONTROL_PANEL_HEIGHT - BUTTON_SLOT_HEIGHT) / 2.0)),
        origin=Origin(
            xyz=(0.0, 0.0, -CONTROL_PANEL_HEIGHT / 2.0 + (CONTROL_PANEL_HEIGHT - BUTTON_SLOT_HEIGHT) / 4.0)
        ),
        material=panel_black,
        name="button_row_bottom_rail",
    )
    for mullion_x in (-0.096, -0.028, 0.028, 0.096):
        control_panel.visual(
            Box((BUTTON_MULLION_WIDTH, CONTROL_PANEL_THICKNESS, BUTTON_SLOT_HEIGHT)),
            origin=Origin(xyz=(mullion_x, 0.0, 0.0)),
            material=panel_black,
            name=f"button_mullion_{mullion_x:+.3f}",
        )
    control_panel.visual(
        Box((CONTROL_PANEL_WIDTH, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, CONTROL_PANEL_THICKNESS / 2.0 - 0.006, -CONTROL_PANEL_HEIGHT / 2.0 + 0.008)),
        material=brushed_steel,
        name="panel_lower_return",
    )

    door = model.part("oven_door")
    door.visual(
        Cylinder(radius=DOOR_HINGE_RADIUS, length=0.78),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="hinge_barrel",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0125, 0.335)),
        material=brushed_steel,
        name="door_shell",
    )
    door.visual(
        Box((0.60, 0.012, 0.38)),
        origin=Origin(xyz=(0.0, -0.028, 0.385)),
        material=dark_glass,
        name="door_window",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.64),
        origin=Origin(xyz=(0.0, -0.075, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="handle_bar",
    )
    door.visual(
        Cylinder(radius=0.01, length=0.05),
        origin=Origin(xyz=(-0.24, -0.05, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="handle_standoff_left",
    )
    door.visual(
        Cylinder(radius=0.01, length=0.05),
        origin=Origin(xyz=(0.24, -0.05, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_metal,
        name="handle_standoff_right",
    )

    knob_x_positions = {
        "left_outer_knob": -0.33,
        "left_inner_knob": -0.22,
        "right_inner_knob": 0.22,
        "right_outer_knob": 0.33,
    }
    for knob_name in knob_x_positions:
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=0.022, length=0.032),
            origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=panel_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, -0.028, 0.014)),
            material=control_white,
            name="indicator_mark",
        )

    for button_name in ("button_left", "button_center", "button_right"):
        button = model.part(button_name)
        button.visual(
            Box((BUTTON_SLOT_WIDTH, 0.03, BUTTON_SLOT_HEIGHT)),
            origin=Origin(),
            material=control_white,
            name="button_body",
        )

    model.articulation(
        "cabinet_to_cooktop",
        ArticulationType.FIXED,
        parent=cabinet,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
    )
    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(
            xyz=(
                0.0,
                -BODY_DEPTH / 2.0 + CONTROL_PANEL_THICKNESS / 2.0,
                BODY_HEIGHT - CONTROL_PANEL_HEIGHT / 2.0,
            )
        ),
    )
    model.articulation(
        "cabinet_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 - 0.01, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.57),
    )

    for joint_name, part_name in (
        ("control_panel_to_left_outer_knob", "left_outer_knob"),
        ("control_panel_to_left_inner_knob", "left_inner_knob"),
        ("control_panel_to_right_inner_knob", "right_inner_knob"),
        ("control_panel_to_right_outer_knob", "right_outer_knob"),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=control_panel,
            child=part_name,
            origin=Origin(xyz=(knob_x_positions[part_name], -CONTROL_PANEL_THICKNESS / 2.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=6.0),
        )

    for button_name, x_pos in (
        ("button_left", -0.06),
        ("button_center", 0.0),
        ("button_right", 0.06),
    ):
        model.articulation(
            f"control_panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button_name,
            origin=Origin(xyz=(x_pos, -CONTROL_PANEL_THICKNESS / 2.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    cooktop = object_model.get_part("cooktop")
    control_panel = object_model.get_part("control_panel")
    door = object_model.get_part("oven_door")
    left_outer_knob = object_model.get_part("left_outer_knob")
    left_inner_knob = object_model.get_part("left_inner_knob")
    right_inner_knob = object_model.get_part("right_inner_knob")
    right_outer_knob = object_model.get_part("right_outer_knob")
    button_left = object_model.get_part("button_left")
    button_center = object_model.get_part("button_center")
    button_right = object_model.get_part("button_right")

    door_hinge = object_model.get_articulation("cabinet_to_oven_door")
    knob_joints = [
        object_model.get_articulation("control_panel_to_left_outer_knob"),
        object_model.get_articulation("control_panel_to_left_inner_knob"),
        object_model.get_articulation("control_panel_to_right_inner_knob"),
        object_model.get_articulation("control_panel_to_right_outer_knob"),
    ]
    button_joints = [
        object_model.get_articulation("control_panel_to_button_left"),
        object_model.get_articulation("control_panel_to_button_center"),
        object_model.get_articulation("control_panel_to_button_right"),
    ]

    def elem_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_contact(cooktop, cabinet, name="cooktop_seated_on_cabinet")
    ctx.expect_contact(control_panel, cabinet, name="control_panel_attached_to_cabinet")
    ctx.expect_contact(door, cabinet, name="door_supported_by_hinge_closed")
    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.60, name="door_covers_oven_opening")

    for knob in (left_outer_knob, left_inner_knob, right_inner_knob, right_outer_knob):
        ctx.expect_contact(knob, control_panel, name=f"{knob.name}_touches_panel")

    for button in (button_left, button_center, button_right):
        ctx.expect_contact(button, control_panel, name=f"{button.name}_guided_by_panel")
        ctx.expect_overlap(
            button,
            control_panel,
            axes="xz",
            min_overlap=0.018,
            name=f"{button.name}_within_panel_face",
        )

    for zone_name in (
        "zone_front_left",
        "zone_rear_left",
        "zone_center",
        "zone_rear_right",
        "zone_front_right",
    ):
        ctx.expect_within(
            cooktop,
            cooktop,
            axes="xy",
            inner_elem=zone_name,
            outer_elem="glass_top",
            margin=0.0,
            name=f"{zone_name}_within_glass_top",
        )

    for joint in knob_joints:
        limits = joint.motion_limits
        axis_ok = joint.axis == (0.0, 1.0, 0.0)
        continuous_ok = joint.joint_type == ArticulationType.CONTINUOUS
        limits_ok = limits is not None and limits.lower is None and limits.upper is None
        ctx.check(
            f"{joint.name}_continuous_axis",
            axis_ok and continuous_ok and limits_ok,
            details=f"{joint.name} should be continuous about +Y with no finite stops.",
        )

    for joint in button_joints:
        limits = joint.motion_limits
        limits_ok = (
            limits is not None
            and limits.lower == 0.0
            and abs((limits.upper or 0.0) - 0.006) < 1e-9
        )
        ctx.check(
            f"{joint.name}_prismatic_axis",
            joint.joint_type == ArticulationType.PRISMATIC and joint.axis == (0.0, 1.0, 0.0) and limits_ok,
            details=f"{joint.name} should be a short front-to-back plunger.",
        )

    door_limits = door_hinge.motion_limits
    ctx.check(
        "oven_door_hinge_properties",
        door_hinge.joint_type == ArticulationType.REVOLUTE
        and door_hinge.axis == (1.0, 0.0, 0.0)
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and 1.45 <= door_limits.upper <= 1.65,
        details="Oven door should swing down on a lower left-to-right hinge with about a 90 degree range.",
    )

    knob_centers = {
        "left_outer": elem_center(left_outer_knob, "knob_body"),
        "left_inner": elem_center(left_inner_knob, "knob_body"),
        "right_inner": elem_center(right_inner_knob, "knob_body"),
        "right_outer": elem_center(right_outer_knob, "knob_body"),
    }
    button_centers = {
        "left": elem_center(button_left, "button_body"),
        "center": elem_center(button_center, "button_body"),
        "right": elem_center(button_right, "button_body"),
    }
    controls_present = all(center is not None for center in (*knob_centers.values(), *button_centers.values()))
    layout_ok = False
    same_row_ok = False
    if controls_present:
        left_outer_x = knob_centers["left_outer"][0]
        left_inner_x = knob_centers["left_inner"][0]
        right_inner_x = knob_centers["right_inner"][0]
        right_outer_x = knob_centers["right_outer"][0]
        button_left_x = button_centers["left"][0]
        button_center_x = button_centers["center"][0]
        button_right_x = button_centers["right"][0]
        all_z = [center[2] for center in (*knob_centers.values(), *button_centers.values())]
        layout_ok = (
            left_outer_x < left_inner_x < button_left_x < button_center_x < button_right_x < right_inner_x < right_outer_x
            and left_outer_x < -0.25
            and right_outer_x > 0.25
        )
        same_row_ok = max(all_z) - min(all_z) < 0.01
    ctx.check(
        "control_layout_matches_prompt",
        controls_present and layout_ok and same_row_ok,
        details="Expected two knobs on the far left, three centered buttons, and two knobs on the far right in one row.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="oven_door_closed_no_floating")

    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_open_no_floating")
            ctx.expect_contact(door, cabinet, name="oven_door_hinge_contact_open")

    pressed_pose = {joint: 0.006 for joint in button_joints}
    with ctx.pose(pressed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="buttons_pressed_no_floating")
        for button in (button_left, button_center, button_right):
            ctx.expect_contact(button, control_panel, name=f"{button.name}_pressed_guided")

    with ctx.pose(
        {
            knob_joints[0]: math.pi / 2.0,
            knob_joints[1]: -math.pi / 3.0,
            knob_joints[2]: math.pi,
            knob_joints[3]: -math.pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_rotated_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
