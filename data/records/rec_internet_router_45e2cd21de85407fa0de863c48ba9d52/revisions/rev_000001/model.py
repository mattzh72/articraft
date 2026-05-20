from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.255
BODY_D = 0.168
BODY_H = 0.036
BOTTOM_T = 0.004
WALL_T = 0.0045
TOP_T = 0.004
INTERIOR_TOP_Z = BODY_H - TOP_T

BUTTON_SLOT_Z0 = 0.012
BUTTON_SLOT_H = 0.0056
BUTTON_CENTER_Z = BUTTON_SLOT_Z0 + BUTTON_SLOT_H / 2.0
BUTTON_TRAVEL = 0.0022
BUTTON_GUIDE_W = 0.0115
BUTTON_GUIDE_D = 0.010
BUTTON_GUIDE_H = 0.0056
BUTTON_CAP_W = 0.0105
BUTTON_CAP_D = 0.0026
BUTTON_CAP_H = 0.0048
BUTTON_SLOT_W = 0.0115
BUTTON_XS = (-0.048, -0.016, 0.016, 0.048)

ANTENNA_BARREL_R = 0.004
ANTENNA_BARREL_L = 0.018
ANTENNA_FOOT_H = 0.012
ANTENNA_BODY_H = 0.142
REAR_HINGE_Z = BODY_H - 0.003
SIDE_HINGE_Z = 0.026


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _add_button(model: ArticulatedObject, body, name: str, x_pos: float) -> None:
    button = model.part(name)
    button.visual(
        Box((BUTTON_CAP_W, BUTTON_CAP_D, BUTTON_CAP_H)),
        origin=Origin(xyz=(0.0, -BUTTON_CAP_D / 2.0, 0.0)),
        material="button",
        name="cap",
    )
    button.visual(
        Box((BUTTON_GUIDE_W, BUTTON_GUIDE_D, BUTTON_GUIDE_H)),
        origin=Origin(xyz=(0.0, BUTTON_GUIDE_D / 2.0, 0.0)),
        material="button_guide",
        name="guide",
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(x_pos, -BODY_D / 2.0, BUTTON_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.04,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def _add_antenna(
    model: ArticulatedObject,
    body,
    *,
    part_name: str,
    joint_name: str,
    origin_xyz: tuple[float, float, float],
    axis: tuple[float, float, float],
    blade_size: tuple[float, float, float],
    foot_size: tuple[float, float, float],
    barrel_rpy: tuple[float, float, float],
    lower: float,
    upper: float,
) -> None:
    antenna = model.part(part_name)
    antenna.visual(
        Cylinder(radius=ANTENNA_BARREL_R, length=ANTENNA_BARREL_L),
        origin=Origin(rpy=barrel_rpy),
        material="antenna_hinge",
        name="barrel",
    )
    antenna.visual(
        Box(foot_size),
        origin=Origin(xyz=(0.0, 0.0, ANTENNA_FOOT_H / 2.0)),
        material="antenna_hinge",
        name="foot",
    )
    antenna.visual(
        Box(blade_size),
        origin=Origin(xyz=(0.0, 0.0, 0.006 + ANTENNA_BODY_H / 2.0)),
        material="antenna_blade",
        name="blade",
    )
    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=body,
        child=antenna,
        origin=Origin(xyz=origin_xyz),
        axis=axis,
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=lower,
            upper=upper,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wifi_router")

    model.material("body_shell", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("top_panel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("antenna_blade", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("antenna_hinge", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("button", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("button_guide", rgba=(0.17, 0.18, 0.19, 1.0))

    body = model.part("body")

    body.visual(
        Box((BODY_W, BODY_D, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T / 2.0)),
        material="body_shell",
        name="bottom_plate",
    )

    wall_h = INTERIOR_TOP_Z - BOTTOM_T
    for side_x, name in (
        (-(BODY_W / 2.0 - WALL_T / 2.0), "left_wall"),
        (BODY_W / 2.0 - WALL_T / 2.0, "right_wall"),
    ):
        body.visual(
            Box((WALL_T, BODY_D, wall_h)),
            origin=Origin(xyz=(side_x, 0.0, BOTTOM_T + wall_h / 2.0)),
            material="body_shell",
            name=name,
        )

    body.visual(
        Box((BODY_W - 2.0 * WALL_T, WALL_T, wall_h)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - WALL_T / 2.0, BOTTOM_T + wall_h / 2.0)
        ),
        material="body_shell",
        name="rear_wall",
    )

    front_inner_w = BODY_W - 2.0 * WALL_T
    lower_strip_h = BUTTON_SLOT_Z0 - BOTTOM_T
    upper_strip_h = INTERIOR_TOP_Z - (BUTTON_SLOT_Z0 + BUTTON_SLOT_H)

    body.visual(
        Box((front_inner_w, WALL_T, lower_strip_h)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 + WALL_T / 2.0,
                BOTTOM_T + lower_strip_h / 2.0,
            )
        ),
        material="body_shell",
        name="front_lower_strip",
    )
    body.visual(
        Box((front_inner_w, WALL_T, upper_strip_h)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D / 2.0 + WALL_T / 2.0,
                BUTTON_SLOT_Z0 + BUTTON_SLOT_H + upper_strip_h / 2.0,
            )
        ),
        material="body_shell",
        name="front_upper_strip",
    )

    slot_left = [x - BUTTON_SLOT_W / 2.0 for x in BUTTON_XS]
    slot_right = [x + BUTTON_SLOT_W / 2.0 for x in BUTTON_XS]
    segment_start = -front_inner_w / 2.0
    separator_index = 0
    for left, right in zip(slot_left, slot_right):
        segment_w = left - segment_start
        if segment_w > 1e-5:
            body.visual(
                Box((segment_w, WALL_T, BUTTON_SLOT_H)),
                origin=Origin(
                    xyz=(
                        (segment_start + left) / 2.0,
                        -BODY_D / 2.0 + WALL_T / 2.0,
                        BUTTON_CENTER_Z,
                    )
                ),
                material="body_shell",
                name=f"front_separator_{separator_index}",
            )
            separator_index += 1
        segment_start = right
    end_w = front_inner_w / 2.0 - segment_start
    if end_w > 1e-5:
        body.visual(
            Box((end_w, WALL_T, BUTTON_SLOT_H)),
            origin=Origin(
                xyz=(
                    (segment_start + front_inner_w / 2.0) / 2.0,
                    -BODY_D / 2.0 + WALL_T / 2.0,
                    BUTTON_CENTER_Z,
                )
            ),
            material="body_shell",
            name=f"front_separator_{separator_index}",
        )

    body.visual(
        Box((BODY_W - 2.0 * WALL_T, BODY_D - 2.0 * WALL_T, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - TOP_T / 2.0)),
        material="top_panel",
        name="top_panel",
    )

    for idx, vent_x in enumerate((-0.044, -0.026, -0.008, 0.010, 0.028, 0.046)):
        body.visual(
            Box((0.008, 0.060, 0.0012)),
            origin=Origin(xyz=(vent_x, 0.018, BODY_H + 0.0006)),
            material="top_panel",
            name=f"vent_ridge_{idx}",
        )

    for idx, x_pos in enumerate(BUTTON_XS, start=1):
        _add_button(model, body, f"front_button_{idx}", x_pos)

    _add_antenna(
        model,
        body,
        part_name="rear_left_antenna",
        joint_name="body_to_rear_left_antenna",
        origin_xyz=(
            -BODY_W / 2.0 + 0.026,
            BODY_D / 2.0 + ANTENNA_BARREL_R - 0.0003,
            REAR_HINGE_Z,
        ),
        axis=(-1.0, 0.0, 0.0),
        blade_size=(0.016, 0.005, ANTENNA_BODY_H),
        foot_size=(0.018, 0.008, ANTENNA_FOOT_H),
        barrel_rpy=(0.0, 1.57079632679, 0.0),
        lower=-1.15,
        upper=0.45,
    )
    _add_antenna(
        model,
        body,
        part_name="rear_right_antenna",
        joint_name="body_to_rear_right_antenna",
        origin_xyz=(
            BODY_W / 2.0 - 0.026,
            BODY_D / 2.0 + ANTENNA_BARREL_R - 0.0003,
            REAR_HINGE_Z,
        ),
        axis=(-1.0, 0.0, 0.0),
        blade_size=(0.016, 0.005, ANTENNA_BODY_H),
        foot_size=(0.018, 0.008, ANTENNA_FOOT_H),
        barrel_rpy=(0.0, 1.57079632679, 0.0),
        lower=-1.15,
        upper=0.45,
    )
    _add_antenna(
        model,
        body,
        part_name="left_side_antenna",
        joint_name="body_to_left_side_antenna",
        origin_xyz=(-BODY_W / 2.0 - ANTENNA_BARREL_R, 0.028, SIDE_HINGE_Z),
        axis=(0.0, -1.0, 0.0),
        blade_size=(0.005, 0.016, ANTENNA_BODY_H),
        foot_size=(0.008, 0.018, ANTENNA_FOOT_H),
        barrel_rpy=(1.57079632679, 0.0, 0.0),
        lower=-0.35,
        upper=1.05,
    )
    _add_antenna(
        model,
        body,
        part_name="right_side_antenna",
        joint_name="body_to_right_side_antenna",
        origin_xyz=(BODY_W / 2.0 + ANTENNA_BARREL_R, 0.028, SIDE_HINGE_Z),
        axis=(0.0, 1.0, 0.0),
        blade_size=(0.005, 0.016, ANTENNA_BODY_H),
        foot_size=(0.008, 0.018, ANTENNA_FOOT_H),
        barrel_rpy=(1.57079632679, 0.0, 0.0),
        lower=-0.35,
        upper=1.05,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")

    for idx in range(1, 5):
        button = object_model.get_part(f"front_button_{idx}")
        button_joint = object_model.get_articulation(f"body_to_front_button_{idx}")
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            min_overlap=0.004,
            elem_a="cap",
            name=f"front_button_{idx} stays aligned with the front slot",
        )
        ctx.expect_gap(
            body,
            button,
            axis="y",
            min_gap=0.0,
            max_gap=0.0005,
            negative_elem="cap",
            name=f"front_button_{idx} cap rests at the front face",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"front_button_{idx} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    antenna_expectations = (
        ("rear_left_antenna", "body_to_rear_left_antenna", 1, 1.0, 0.02),
        ("rear_right_antenna", "body_to_rear_right_antenna", 1, 1.0, 0.02),
        ("left_side_antenna", "body_to_left_side_antenna", 0, -1.0, 0.02),
        ("right_side_antenna", "body_to_right_side_antenna", 0, 1.0, 0.02),
    )
    for part_name, joint_name, axis_index, sign, min_delta in antenna_expectations:
        antenna = object_model.get_part(part_name)
        antenna_joint = object_model.get_articulation(joint_name)
        rest_center = _aabb_center(ctx.part_element_world_aabb(antenna, elem="blade"))
        upper = antenna_joint.motion_limits.upper if antenna_joint.motion_limits else 0.0
        with ctx.pose({antenna_joint: upper}):
            opened_center = _aabb_center(ctx.part_element_world_aabb(antenna, elem="blade"))
        ctx.check(
            f"{part_name} rotates away from the body",
            rest_center is not None
            and opened_center is not None
            and sign * (opened_center[axis_index] - rest_center[axis_index]) > min_delta,
            details=f"rest={rest_center}, opened={opened_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
