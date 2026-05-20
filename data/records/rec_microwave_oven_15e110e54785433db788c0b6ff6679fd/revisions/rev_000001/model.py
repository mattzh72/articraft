from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


OUTER_W = 0.49
OUTER_D = 0.39
OUTER_H = 0.29

SIDE_T = 0.02
DIVIDER_T = 0.02
PANEL_W = 0.11
BOTTOM_T = 0.025
TOP_T = 0.025
BACK_T = 0.02

FRONT_Y = -OUTER_D / 2.0
BACK_Y = OUTER_D / 2.0
SHELL_MID_Z = BOTTOM_T + (OUTER_H - BOTTOM_T - TOP_T) / 2.0

CAVITY_W = OUTER_W - (2.0 * SIDE_T) - DIVIDER_T - PANEL_W
CAVITY_H = OUTER_H - BOTTOM_T - TOP_T
CAVITY_D = OUTER_D - BACK_T
CAVITY_LEFT = -OUTER_W / 2.0 + SIDE_T
CAVITY_RIGHT = CAVITY_LEFT + CAVITY_W
CAVITY_CENTER_X = (CAVITY_LEFT + CAVITY_RIGHT) / 2.0
CAVITY_CENTER_Y = (FRONT_Y + (BACK_Y - BACK_T)) / 2.0

CONTROL_CENTER_X = CAVITY_RIGHT + DIVIDER_T + PANEL_W / 2.0
CHANNEL_W = 0.056
CHANNEL_H = 0.192
CHANNEL_D = 0.022
CHANNEL_CENTER_Z = 0.145
CHANNEL_BOTTOM_Z = CHANNEL_CENTER_Z - CHANNEL_H / 2.0
CHANNEL_TOP_Z = CHANNEL_CENTER_Z + CHANNEL_H / 2.0

HINGE_R = 0.006
HINGE_X = (-OUTER_W / 2.0) - 0.002
HINGE_Y = FRONT_Y - 0.012

DOOR_W = 0.33
DOOR_H = 0.248
DOOR_T = 0.024
DOOR_CENTER_X_REL = HINGE_R + DOOR_W / 2.0
DOOR_CENTER_Y_REL = (FRONT_Y - DOOR_T / 2.0) - HINGE_Y
DOOR_CENTER_Z = 0.145
DOOR_LEFT_X_REL = DOOR_CENTER_X_REL - DOOR_W / 2.0
DOOR_RIGHT_X_REL = DOOR_CENTER_X_REL + DOOR_W / 2.0
DOOR_BOTTOM_Z_REL = -DOOR_H / 2.0
DOOR_TOP_Z_REL = DOOR_H / 2.0
DOOR_STILE_W = 0.03
DOOR_RAIL_H = 0.038
DOOR_FRAME_LEFT_X_REL = 0.05
DOOR_FRAME_W = DOOR_RIGHT_X_REL - DOOR_FRAME_LEFT_X_REL
HINGE_BARREL_L = 0.19
HINGE_LEAF_T = 0.008

BUTTON_W = CHANNEL_W
BUTTON_H = 0.03
BUTTON_D = 0.014
BUTTON_REST_Y = FRONT_Y + BUTTON_D / 2.0 - 0.004
BUTTON_TRAVEL = 0.006
BUTTON_Z_CENTERS = (0.082, 0.122, 0.162, 0.202)

TURNTABLE_R = 0.12
TURNTABLE_T = 0.008
TURNTABLE_SUPPORT_R = 0.095
TURNTABLE_SUPPORT_T = 0.004
TURNTABLE_AXIS_Z = BOTTOM_T + 0.0015 + TURNTABLE_SUPPORT_T


def _button_part(model: ArticulatedObject, index: int, material) -> object:
    button = model.part(f"button_{index}")
    button.visual(
        Box((BUTTON_W, BUTTON_D, BUTTON_H)),
        material=material,
        name="button_cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((BUTTON_W, BUTTON_D, BUTTON_H)),
        mass=0.025,
    )
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_microwave")

    body_shell = model.material("body_shell", rgba=(0.17, 0.18, 0.20, 1.0))
    body_trim = model.material("body_trim", rgba=(0.08, 0.08, 0.09, 1.0))
    cavity_liner = model.material("cavity_liner", rgba=(0.74, 0.75, 0.77, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.16, 0.22, 0.27, 0.42))
    button_grey = model.material("button_grey", rgba=(0.76, 0.78, 0.80, 1.0))
    glass_plate = model.material("glass_plate", rgba=(0.78, 0.86, 0.90, 0.55))
    handle_dark = model.material("handle_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        Box((OUTER_W, OUTER_D, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T / 2.0)),
        material=body_shell,
        name="bottom_shell",
    )
    body.visual(
        Box((OUTER_W, OUTER_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_H - TOP_T / 2.0)),
        material=body_shell,
        name="top_shell",
    )
    body.visual(
        Box((SIDE_T, OUTER_D, CAVITY_H)),
        origin=Origin(xyz=(-OUTER_W / 2.0 + SIDE_T / 2.0, 0.0, SHELL_MID_Z)),
        material=body_shell,
        name="left_shell",
    )
    body.visual(
        Box((DIVIDER_T, OUTER_D, CAVITY_H)),
        origin=Origin(xyz=(CAVITY_RIGHT + DIVIDER_T / 2.0, 0.0, SHELL_MID_Z)),
        material=body_shell,
        name="divider_shell",
    )
    body.visual(
        Box((SIDE_T, OUTER_D, CAVITY_H)),
        origin=Origin(xyz=(OUTER_W / 2.0 - SIDE_T / 2.0, 0.0, SHELL_MID_Z)),
        material=body_shell,
        name="right_shell",
    )
    body.visual(
        Box((OUTER_W - 2.0 * SIDE_T, BACK_T, CAVITY_H)),
        origin=Origin(xyz=(0.0, BACK_Y - BACK_T / 2.0, SHELL_MID_Z)),
        material=body_shell,
        name="back_shell",
    )
    body.visual(
        Box((PANEL_W, OUTER_D - CHANNEL_D, CAVITY_H)),
        origin=Origin(
            xyz=(
                CONTROL_CENTER_X,
                FRONT_Y + CHANNEL_D + (OUTER_D - CHANNEL_D) / 2.0,
                SHELL_MID_Z,
            )
        ),
        material=body_shell,
        name="control_core",
    )

    frame_side_w = (PANEL_W - CHANNEL_W) / 2.0
    body.visual(
        Box((frame_side_w, CHANNEL_D, CHANNEL_H)),
        origin=Origin(
            xyz=(
                CONTROL_CENTER_X - CHANNEL_W / 2.0 - frame_side_w / 2.0,
                FRONT_Y + CHANNEL_D / 2.0,
                CHANNEL_CENTER_Z,
            )
        ),
        material=body_trim,
        name="control_channel_left",
    )
    body.visual(
        Box((frame_side_w, CHANNEL_D, CHANNEL_H)),
        origin=Origin(
            xyz=(
                CONTROL_CENTER_X + CHANNEL_W / 2.0 + frame_side_w / 2.0,
                FRONT_Y + CHANNEL_D / 2.0,
                CHANNEL_CENTER_Z,
            )
        ),
        material=body_trim,
        name="control_channel_right",
    )
    body.visual(
        Box((PANEL_W, CHANNEL_D, CHANNEL_BOTTOM_Z - BOTTOM_T)),
        origin=Origin(
            xyz=(
                CONTROL_CENTER_X,
                FRONT_Y + CHANNEL_D / 2.0,
                (BOTTOM_T + CHANNEL_BOTTOM_Z) / 2.0,
            )
        ),
        material=body_trim,
        name="control_channel_bottom",
    )
    body.visual(
        Box((PANEL_W, CHANNEL_D, (OUTER_H - TOP_T) - CHANNEL_TOP_Z)),
        origin=Origin(
            xyz=(
                CONTROL_CENTER_X,
                FRONT_Y + CHANNEL_D / 2.0,
                (CHANNEL_TOP_Z + (OUTER_H - TOP_T)) / 2.0,
            )
        ),
        material=body_trim,
        name="control_channel_top",
    )

    liner_t = 0.0015
    body.visual(
        Box((CAVITY_W, CAVITY_D, liner_t)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y, BOTTOM_T + liner_t / 2.0)
        ),
        material=cavity_liner,
        name="cavity_floor_liner",
    )
    body.visual(
        Box((CAVITY_W, CAVITY_D, liner_t)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y, OUTER_H - TOP_T - liner_t / 2.0)
        ),
        material=cavity_liner,
        name="cavity_ceiling_liner",
    )
    body.visual(
        Box((liner_t, CAVITY_D, CAVITY_H)),
        origin=Origin(
            xyz=(CAVITY_LEFT + liner_t / 2.0, CAVITY_CENTER_Y, SHELL_MID_Z)
        ),
        material=cavity_liner,
        name="cavity_left_liner",
    )
    body.visual(
        Box((liner_t, CAVITY_D, CAVITY_H)),
        origin=Origin(
            xyz=(CAVITY_RIGHT - liner_t / 2.0, CAVITY_CENTER_Y, SHELL_MID_Z)
        ),
        material=cavity_liner,
        name="cavity_right_liner",
    )
    body.visual(
        Box((CAVITY_W, liner_t, CAVITY_H)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X, BACK_Y - BACK_T - liner_t / 2.0, SHELL_MID_Z)
        ),
        material=cavity_liner,
        name="cavity_back_liner",
    )

    body.visual(
        Cylinder(radius=TURNTABLE_SUPPORT_R, length=TURNTABLE_SUPPORT_T),
        origin=Origin(
            xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y, TURNTABLE_AXIS_Z - TURNTABLE_SUPPORT_T / 2.0)
        ),
        material=cavity_liner,
        name="turntable_support",
    )
    body.inertial = Inertial.from_geometry(
        Box((OUTER_W, OUTER_D, OUTER_H)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, OUTER_H / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_STILE_W, DOOR_T, DOOR_H)),
        origin=Origin(
            xyz=(
                DOOR_FRAME_LEFT_X_REL + DOOR_STILE_W / 2.0,
                DOOR_CENTER_Y_REL,
                0.0,
            )
        ),
        material=body_trim,
        name="left_stile",
    )
    door.visual(
        Box((DOOR_STILE_W, DOOR_T, DOOR_H)),
        origin=Origin(
            xyz=(
                DOOR_RIGHT_X_REL - DOOR_STILE_W / 2.0,
                DOOR_CENTER_Y_REL,
                0.0,
            )
        ),
        material=body_trim,
        name="right_stile",
    )
    door.visual(
        Box((DOOR_FRAME_W, DOOR_T, DOOR_RAIL_H)),
        origin=Origin(
            xyz=(
                (DOOR_FRAME_LEFT_X_REL + DOOR_RIGHT_X_REL) / 2.0,
                DOOR_CENTER_Y_REL,
                DOOR_TOP_Z_REL - DOOR_RAIL_H / 2.0,
            )
        ),
        material=body_trim,
        name="top_rail",
    )
    door.visual(
        Box((DOOR_FRAME_W, DOOR_T, DOOR_RAIL_H)),
        origin=Origin(
            xyz=(
                (DOOR_FRAME_LEFT_X_REL + DOOR_RIGHT_X_REL) / 2.0,
                DOOR_CENTER_Y_REL,
                DOOR_BOTTOM_Z_REL + DOOR_RAIL_H / 2.0,
            )
        ),
        material=body_trim,
        name="bottom_rail",
    )

    glass_w = DOOR_FRAME_W - 2.0 * DOOR_STILE_W
    glass_h = DOOR_H - 2.0 * DOOR_RAIL_H
    door.visual(
        Box((glass_w, 0.008, glass_h)),
        origin=Origin(
            xyz=(
                (DOOR_FRAME_LEFT_X_REL + DOOR_RIGHT_X_REL) / 2.0,
                DOOR_CENTER_Y_REL - 0.001,
                0.0,
            )
        ),
        material=glass_tint,
        name="window_glass",
    )
    door.visual(
        Box((0.016, 0.012, 0.13)),
        origin=Origin(
            xyz=(DOOR_RIGHT_X_REL - 0.032, DOOR_CENTER_Y_REL - 0.018, 0.0)
        ),
        material=handle_dark,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=1.15,
        origin=Origin(xyz=(DOOR_CENTER_X_REL, DOOR_CENTER_Y_REL, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    for index, button_z in enumerate(BUTTON_Z_CENTERS):
        button = _button_part(model, index, button_grey)
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(CONTROL_CENTER_X, BUTTON_REST_Y, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=TURNTABLE_R, length=TURNTABLE_T),
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_T / 2.0)),
        material=glass_plate,
        name="glass_plate",
    )
    turntable.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=glass_plate,
        name="center_boss",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=TURNTABLE_R, length=TURNTABLE_T),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_T / 2.0)),
    )
    model.articulation(
        "body_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y, TURNTABLE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("body_to_door")
    turntable_spin = object_model.get_articulation("body_to_turntable")
    button_parts = [object_model.get_part(f"button_{index}") for index in range(4)]
    button_joints = [
        object_model.get_articulation(f"body_to_button_{index}") for index in range(4)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    door_limits = door_hinge.motion_limits
    ctx.check(
        "door_joint_is_revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"Expected REVOLUTE door joint, got {door_hinge.articulation_type!r}.",
    )
    ctx.check(
        "door_axis_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical hinge axis, got {door_hinge.axis!r}.",
    )
    ctx.check(
        "door_limit_realistic",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and math.radians(95.0) <= door_limits.upper <= math.radians(120.0),
        "Microwave door should open from closed to a little over ninety degrees.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="left_shell",
            negative_elem="left_stile",
            max_gap=0.001,
            max_penetration=0.0,
            name="door_closed_front_gap",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.20,
            name="door_closed_covers_opening",
        )
    door_rest_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
    if door_rest_aabb is not None and door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            door_open_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_no_floating")
            ctx.check(
                "door_swings_left_on_open",
                door_open_aabb is not None
                and ((door_open_aabb[0][0] + door_open_aabb[1][0]) / 2.0)
                < ((door_rest_aabb[0][0] + door_rest_aabb[1][0]) / 2.0) - 0.12,
                "Door glass center should move left as the side-hinged door opens.",
            )

    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        limits = joint.motion_limits
        ctx.check(
            f"button_{index}_joint_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"Button {index} should use a PRISMATIC joint.",
        )
        ctx.check(
            f"button_{index}_axis_inward",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"Button {index} should plunge inward along +Y.",
        )
        ctx.expect_contact(
            button,
            body,
            name=f"button_{index}_rest_contact",
        )
        rest_pos = ctx.part_world_position(button)
        if rest_pos is not None and limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"button_{index}_pressed_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"button_{index}_pressed_no_floating")
                ctx.expect_contact(
                    button,
                    body,
                    name=f"button_{index}_pressed_contact",
                )
                ctx.check(
                    f"button_{index}_moves_inward",
                    pressed_pos is not None
                    and pressed_pos[1] > rest_pos[1] + (limits.upper - 0.001),
                    f"Button {index} should translate inward by roughly {limits.upper:.3f} m.",
                )

    ctx.check(
        "turntable_joint_is_continuous",
        turntable_spin.articulation_type == ArticulationType.CONTINUOUS,
        f"Expected CONTINUOUS turntable joint, got {turntable_spin.articulation_type!r}.",
    )
    ctx.check(
        "turntable_axis_vertical",
        tuple(turntable_spin.axis) == (0.0, 0.0, 1.0),
        f"Turntable should spin about vertical axis, got {turntable_spin.axis!r}.",
    )
    ctx.expect_contact(
        turntable,
        body,
        elem_a="glass_plate",
        elem_b="turntable_support",
        name="turntable_rest_support_contact",
    )
    ctx.expect_within(
        turntable,
        body,
        axes="xy",
        inner_elem="glass_plate",
        outer_elem="cavity_floor_liner",
        margin=0.0,
        name="turntable_within_cavity_floor",
    )
    with ctx.pose({turntable_spin: math.pi / 2.0}):
        ctx.fail_if_isolated_parts(name="turntable_rotated_no_floating")
        ctx.expect_contact(
            turntable,
            body,
            elem_a="glass_plate",
            elem_b="turntable_support",
            name="turntable_rotated_support_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
