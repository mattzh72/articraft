from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.292
BODY_D = 0.118
BASE_T = 0.0032
SWITCH_BED_W = 0.264
SWITCH_BED_D = 0.084
SWITCH_BED_T = 0.0018
FRONT_LIP_D = 0.012
FRONT_LIP_T = 0.0046
SIDE_RAIL_W = 0.014
SIDE_RAIL_D = 0.094
SIDE_RAIL_T = 0.0056
FUNCTION_SHELF_W = 0.264
FUNCTION_SHELF_D = 0.008
FUNCTION_SHELF_T = 0.0025
REAR_SPINE_D = 0.012
REAR_SPINE_T = 0.0080
CHEEK_W = 0.012
CHEEK_D = 0.026
CHEEK_T = 0.0064

KEY_GAP = 0.004
MAIN_KEY_H = 0.0024
FUNCTION_KEY_H = 0.0022
MAIN_KEY_TRAVEL = 0.0008
FUNCTION_KEY_TRAVEL = 0.0006
MAIN_KEY_JOINT_Z = 0.0070
FUNCTION_KEY_JOINT_Z = 0.0064

STAND_W = 0.240
STAND_D = 0.014
STAND_T = 0.0024
STAND_STOP_D = 0.0030
STAND_STOP_T = 0.0016
STAND_HINGE_Y = 0.047
STAND_HINGE_Z = 0.0060


def _add_key_row(
    model: ArticulatedObject,
    body,
    *,
    row_name: str,
    y: float,
    widths: list[float],
    depth: float,
    height: float,
    joint_z: float,
    travel: float,
    key_material,
    x_offset: float = 0.0,
) -> list[str]:
    part_names: list[str] = []
    total_width = sum(widths) + KEY_GAP * (len(widths) - 1)
    cursor = -0.5 * total_width + x_offset
    support_z = BASE_T + SWITCH_BED_T

    for idx, width in enumerate(widths, start=1):
        cursor += width / 2.0
        part_name = f"key_{row_name}_{idx:02d}"
        key = model.part(part_name)
        key.visual(
            Box((width, depth, height)),
            origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
            material=key_material,
            name="keycap",
        )
        stem_height = max(joint_z - support_z, 0.0008)
        key.visual(
            Box(
                (
                    min(width * 0.38, 0.009),
                    min(depth * 0.45, 0.007),
                    stem_height,
                )
            ),
            origin=Origin(xyz=(0.0, 0.0, -stem_height / 2.0)),
            material=key_material,
            name="plunger",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(cursor, y, joint_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.06,
                lower=0.0,
                upper=travel,
            ),
        )
        part_names.append(part_name)
        cursor += width / 2.0 + KEY_GAP

    return part_names


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_keyboard_with_stand")

    body_color = model.material("body_color", rgba=(0.18, 0.19, 0.21, 1.0))
    tray_color = model.material("tray_color", rgba=(0.24, 0.25, 0.28, 1.0))
    key_color = model.material("key_color", rgba=(0.14, 0.15, 0.16, 1.0))
    stand_color = model.material("stand_color", rgba=(0.22, 0.23, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=body_color,
        name="base_shell",
    )
    body.visual(
        Box((SWITCH_BED_W, SWITCH_BED_D, SWITCH_BED_T)),
        origin=Origin(xyz=(0.0, -0.012, BASE_T + SWITCH_BED_T / 2.0)),
        material=tray_color,
        name="switch_bed",
    )
    body.visual(
        Box((BODY_W, FRONT_LIP_D, FRONT_LIP_T)),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + FRONT_LIP_D / 2.0, BASE_T + FRONT_LIP_T / 2.0)
        ),
        material=tray_color,
        name="front_lip",
    )
    for side, x in (
        ("left", -(BODY_W / 2.0 - SIDE_RAIL_W / 2.0)),
        ("right", BODY_W / 2.0 - SIDE_RAIL_W / 2.0),
    ):
        body.visual(
            Box((SIDE_RAIL_W, SIDE_RAIL_D, SIDE_RAIL_T)),
            origin=Origin(xyz=(x, -0.012, BASE_T + SIDE_RAIL_T / 2.0)),
            material=tray_color,
            name=f"{side}_rail",
        )
    body.visual(
        Box((FUNCTION_SHELF_W, FUNCTION_SHELF_D, FUNCTION_SHELF_T)),
        origin=Origin(xyz=(0.0, 0.026, BASE_T + FUNCTION_SHELF_T / 2.0)),
        material=tray_color,
        name="function_row_shelf",
    )
    body.visual(
        Box((BODY_W, REAR_SPINE_D, REAR_SPINE_T)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - REAR_SPINE_D / 2.0, BASE_T + REAR_SPINE_T / 2.0)
        ),
        material=tray_color,
        name="rear_spine",
    )
    for side, x in (("left", -0.132), ("right", 0.132)):
        body.visual(
            Box((CHEEK_W, CHEEK_D, CHEEK_T)),
            origin=Origin(xyz=(x, 0.034, BASE_T + CHEEK_T / 2.0)),
            material=tray_color,
            name=f"{side}_stand_cheek",
        )

    stand = model.part("stand")
    stand.visual(
        Box((STAND_W, STAND_D, STAND_T)),
        origin=Origin(xyz=(0.0, -STAND_D / 2.0, 0.0)),
        material=stand_color,
        name="stand_panel",
    )
    stand.visual(
        Box((STAND_W * 0.90, STAND_STOP_D, STAND_STOP_T)),
        origin=Origin(
            xyz=(
                0.0,
                -STAND_D + STAND_STOP_D / 2.0,
                STAND_T / 2.0 + STAND_STOP_T / 2.0,
            )
        ),
        material=stand_color,
        name="stand_stop",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, STAND_HINGE_Y, STAND_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.2,
            lower=0.0,
            upper=1.18,
        ),
    )

    _add_key_row(
        model,
        body,
        row_name="fn",
        y=0.016,
        widths=[0.018] * 10,
        depth=0.010,
        height=FUNCTION_KEY_H,
        joint_z=FUNCTION_KEY_JOINT_Z,
        travel=FUNCTION_KEY_TRAVEL,
        key_material=key_color,
    )
    _add_key_row(
        model,
        body,
        row_name="upper",
        y=-0.002,
        widths=[0.019] * 10,
        depth=0.013,
        height=MAIN_KEY_H,
        joint_z=MAIN_KEY_JOINT_Z,
        travel=MAIN_KEY_TRAVEL,
        key_material=key_color,
        x_offset=-0.004,
    )
    _add_key_row(
        model,
        body,
        row_name="home",
        y=-0.020,
        widths=[0.020] * 9,
        depth=0.014,
        height=MAIN_KEY_H,
        joint_z=MAIN_KEY_JOINT_Z,
        travel=MAIN_KEY_TRAVEL,
        key_material=key_color,
        x_offset=-0.008,
    )
    _add_key_row(
        model,
        body,
        row_name="lower",
        y=-0.038,
        widths=[0.022, 0.022, 0.022, 0.070, 0.022, 0.022, 0.028],
        depth=0.013,
        height=MAIN_KEY_H,
        joint_z=MAIN_KEY_JOINT_Z,
        travel=MAIN_KEY_TRAVEL,
        key_material=key_color,
        x_offset=-0.006,
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

    missing: list[str] = []
    for part_name in ("body", "stand", "key_fn_05", "key_home_04", "key_lower_04"):
        try:
            object_model.get_part(part_name)
        except Exception:
            missing.append(part_name)
    ctx.check("core parts exist", not missing, details=f"missing={missing}")

    body = object_model.get_part("body")
    stand = object_model.get_part("stand")
    stand_hinge = object_model.get_articulation("body_to_stand")
    fn_key = object_model.get_part("key_fn_05")
    fn_joint = object_model.get_articulation("body_to_key_fn_05")
    home_key = object_model.get_part("key_home_04")
    home_joint = object_model.get_articulation("body_to_key_home_04")

    ctx.expect_within(
        home_key,
        body,
        axes="xy",
        inner_elem="keycap",
        outer_elem="base_shell",
        margin=0.0,
        name="home-row key stays within the tray footprint",
    )

    with ctx.pose({stand_hinge: 0.0}):
        ctx.expect_gap(
            stand,
            fn_key,
            axis="y",
            min_gap=0.006,
            max_gap=0.020,
            positive_elem="stand_panel",
            negative_elem="keycap",
            name="stand flap is recessed behind the function row",
        )

        stand_closed = ctx.part_element_world_aabb(stand, elem="stand_panel")
        rear_spine = ctx.part_element_world_aabb(body, elem="rear_spine")
        closed_recessed = (
            stand_closed is not None
            and rear_spine is not None
            and stand_closed[1][2] < rear_spine[1][2] - 0.0025
        )
        ctx.check(
            "closed stand sits below the rear spine",
            closed_recessed,
            details=f"stand_closed={stand_closed}, rear_spine={rear_spine}",
        )

    closed_panel = ctx.part_element_world_aabb(stand, elem="stand_panel")
    with ctx.pose({stand_hinge: 1.05}):
        open_panel = ctx.part_element_world_aabb(stand, elem="stand_panel")
    ctx.check(
        "stand flap rotates upward for tablet support",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][2] > closed_panel[1][2] + 0.01,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )

    fn_rest = ctx.part_world_position(fn_key)
    with ctx.pose({fn_joint: FUNCTION_KEY_TRAVEL}):
        fn_pressed = ctx.part_world_position(fn_key)
    ctx.check(
        "function key plunges downward",
        fn_rest is not None
        and fn_pressed is not None
        and fn_pressed[2] < fn_rest[2] - 0.0004,
        details=f"rest={fn_rest}, pressed={fn_pressed}",
    )

    home_rest = ctx.part_world_position(home_key)
    with ctx.pose({home_joint: MAIN_KEY_TRAVEL}):
        home_pressed = ctx.part_world_position(home_key)
    ctx.check(
        "home-row key plunges downward",
        home_rest is not None
        and home_pressed is not None
        and home_pressed[2] < home_rest[2] - 0.0006,
        details=f"rest={home_rest}, pressed={home_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
