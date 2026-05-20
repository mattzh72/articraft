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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rolled_point(
    x: float,
    center_y: float,
    center_z: float,
    local_y: float,
    local_z: float,
    roll: float,
) -> tuple[float, float, float]:
    return (
        x,
        center_y + (local_y * math.cos(roll)) - (local_z * math.sin(roll)),
        center_z + (local_y * math.sin(roll)) + (local_z * math.cos(roll)),
    )


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 32,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + (radius * math.cos((2.0 * math.pi * index) / segments)),
            cy + (radius * math.sin((2.0 * math.pi * index) / segments)),
        )
        for index in range(segments)
    ]


def _control_deck_mesh() -> object:
    deck = ExtrudeWithHolesGeometry(
        [
            (-0.39, -0.17),
            (0.39, -0.17),
            (0.39, 0.17),
            (-0.39, 0.17),
        ],
        [
            _circle_profile(0.0225, center=(-0.070, 0.022)),
            _circle_profile(0.0225, center=(0.070, 0.022)),
        ],
        0.026,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(deck, "control_deck_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_player_arcade_machine")

    cabinet_black = model.material("cabinet_black", rgba=(0.11, 0.11, 0.13, 1.0))
    cabinet_trim = model.material("cabinet_trim", rgba=(0.18, 0.18, 0.20, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.08, 0.12, 0.14, 0.55))
    marquee_glow = model.material("marquee_glow", rgba=(0.88, 0.92, 0.98, 0.82))
    slot_black = model.material("slot_black", rgba=(0.03, 0.03, 0.04, 1.0))
    coin_metal = model.material("coin_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    hatch_metal = model.material("hatch_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    sleeve_black = model.material("sleeve_black", rgba=(0.07, 0.07, 0.08, 1.0))
    joystick_red = model.material("joystick_red", rgba=(0.70, 0.14, 0.13, 1.0))
    action_orange = model.material("action_orange", rgba=(0.91, 0.39, 0.12, 1.0))
    action_yellow = model.material("action_yellow", rgba=(0.96, 0.72, 0.12, 1.0))
    action_green = model.material("action_green", rgba=(0.18, 0.74, 0.30, 1.0))
    button_blue = model.material("button_blue", rgba=(0.40, 0.77, 1.0, 0.88))
    button_red = model.material("button_red", rgba=(1.0, 0.34, 0.30, 0.88))

    deck_center_y = -0.118
    deck_center_z = 0.944
    deck_roll = 0.26
    deck_thickness = 0.026
    deck_top_local_z = deck_thickness * 0.5
    screen_center_y = -0.142
    screen_center_z = 1.285
    screen_roll = -0.17
    joystick_specs: list[tuple[str, tuple[float, float, float]]] = []
    action_button_specs: list[tuple[str, tuple[float, float, float], object, str]] = []

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.82, 0.78, 0.92)),
        origin=Origin(xyz=(0.0, 0.04, 0.46)),
        material=cabinet_black,
        name="lower_shell",
    )
    cabinet.visual(
        Box((0.82, 0.48, 0.72)),
        origin=Origin(xyz=(0.0, 0.17, 1.26)),
        material=cabinet_black,
        name="rear_tower",
    )
    cabinet.visual(
        Box((0.82, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, -0.095, 1.68)),
        material=cabinet_black,
        name="marquee_top",
    )
    cabinet.visual(
        Box((0.70, 0.012, 0.082)),
        origin=Origin(xyz=(0.0, -0.195, 1.68)),
        material=marquee_glow,
        name="marquee_face",
    )
    cabinet.visual(
        _control_deck_mesh(),
        origin=Origin(xyz=(0.0, deck_center_y, deck_center_z), rpy=(deck_roll, 0.0, 0.0)),
        material=cabinet_trim,
        name="control_deck",
    )
    cabinet.visual(
        Box((0.78, 0.06, 0.11)),
        origin=Origin(xyz=(0.0, -0.205, 0.875), rpy=(deck_roll, 0.0, 0.0)),
        material=cabinet_black,
        name="deck_skirt",
    )
    cabinet.visual(
        Box((0.060, 0.045, 0.490)),
        origin=Origin(
            xyz=_rolled_point(-0.293, screen_center_y, screen_center_z, 0.0, 0.0, screen_roll),
            rpy=(screen_roll, 0.0, 0.0),
        ),
        material=slot_black,
        name="monitor_left_bezel",
    )
    cabinet.visual(
        Box((0.060, 0.045, 0.490)),
        origin=Origin(
            xyz=_rolled_point(0.293, screen_center_y, screen_center_z, 0.0, 0.0, screen_roll),
            rpy=(screen_roll, 0.0, 0.0),
        ),
        material=slot_black,
        name="monitor_right_bezel",
    )
    cabinet.visual(
        Box((0.646, 0.045, 0.058)),
        origin=Origin(
            xyz=_rolled_point(0.0, screen_center_y, screen_center_z, 0.0, 0.218, screen_roll),
            rpy=(screen_roll, 0.0, 0.0),
        ),
        material=slot_black,
        name="monitor_top_bezel",
    )
    cabinet.visual(
        Box((0.646, 0.045, 0.058)),
        origin=Origin(
            xyz=_rolled_point(0.0, screen_center_y, screen_center_z, 0.0, -0.218, screen_roll),
            rpy=(screen_roll, 0.0, 0.0),
        ),
        material=slot_black,
        name="monitor_bottom_bezel",
    )
    cabinet.visual(
        Box((0.520, 0.016, 0.366)),
        origin=Origin(
            xyz=_rolled_point(0.0, screen_center_y, screen_center_z, 0.030, 0.0, screen_roll),
            rpy=(screen_roll, 0.0, 0.0),
        ),
        material=screen_glass,
        name="monitor_screen",
    )
    cabinet.visual(
        Box((0.690, 0.180, 0.540)),
        origin=Origin(xyz=(0.0, -0.030, 1.285)),
        material=cabinet_trim,
        name="monitor_backer",
    )
    cabinet.visual(
        Box((0.42, 0.018, 0.16)),
        origin=Origin(xyz=(0.0, -0.359, 0.585)),
        material=cabinet_trim,
        name="coin_panel",
    )
    cabinet.visual(
        Box((0.112, 0.008, 0.018)),
        origin=Origin(xyz=(-0.120, -0.372, 0.614)),
        material=slot_black,
        name="left_coin_slot",
    )
    cabinet.visual(
        Box((0.112, 0.008, 0.018)),
        origin=Origin(xyz=(0.120, -0.372, 0.614)),
        material=slot_black,
        name="right_coin_slot",
    )
    cabinet.visual(
        Box((0.030, 0.010, 0.030)),
        origin=Origin(xyz=(-0.120, -0.371, 0.552)),
        material=coin_metal,
        name="left_coin_return",
    )
    cabinet.visual(
        Box((0.030, 0.010, 0.030)),
        origin=Origin(xyz=(0.120, -0.371, 0.552)),
        material=coin_metal,
        name="right_coin_return",
    )
    cabinet.visual(
        Box((0.48, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.356, 0.401)),
        material=coin_metal,
        name="service_frame_top",
    )
    cabinet.visual(
        Box((0.020, 0.012, 0.262)),
        origin=Origin(xyz=(-0.240, -0.356, 0.265)),
        material=coin_metal,
        name="service_frame_left",
    )
    cabinet.visual(
        Box((0.020, 0.012, 0.262)),
        origin=Origin(xyz=(0.240, -0.356, 0.265)),
        material=coin_metal,
        name="service_frame_right",
    )

    for side_index, x_sign in enumerate((-1.0, 1.0)):
        joy_x = 0.230 * x_sign
        joy_joint_xyz = _rolled_point(
            joy_x,
            deck_center_y,
            deck_center_z,
            -0.030,
            deck_top_local_z,
            deck_roll,
        )
        joy_collar_xyz = _rolled_point(
            joy_x,
            deck_center_y,
            deck_center_z,
            -0.030,
            deck_top_local_z + 0.003,
            deck_roll,
        )
        cabinet.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=joy_collar_xyz, rpy=(deck_roll, 0.0, 0.0)),
            material=slot_black,
            name=f"joystick_collar_{side_index}",
        )
        joystick_specs.append((f"player_{side_index}_joystick", joy_joint_xyz))
        action_materials = (action_orange, action_yellow, action_green)
        action_offsets_x = (0.135 * x_sign, 0.180 * x_sign, 0.225 * x_sign)
        action_offsets_y = (0.018, 0.043, 0.008)
        for button_index, (button_x, button_y, button_material) in enumerate(
            zip(action_offsets_x, action_offsets_y, action_materials, strict=True)
        ):
            action_joint_xyz = _rolled_point(
                button_x,
                deck_center_y,
                deck_center_z,
                button_y,
                deck_top_local_z,
                deck_roll,
            )
            action_guide_xyz = _rolled_point(
                button_x,
                deck_center_y,
                deck_center_z,
                button_y,
                deck_top_local_z + 0.002,
                deck_roll,
            )
            cabinet.visual(
                Cylinder(radius=0.020, length=0.004),
                origin=Origin(xyz=action_guide_xyz, rpy=(deck_roll, 0.0, 0.0)),
                material=sleeve_black,
                name=f"action_button_guide_{side_index}_{button_index}",
            )
            action_button_specs.append(
                (
                    f"action_button_{side_index}_{button_index}",
                    action_joint_xyz,
                    button_material,
                    f"action_button_guide_{side_index}_{button_index}",
                )
            )

    left_button_joint_xyz = _rolled_point(
        -0.070,
        deck_center_y,
        deck_center_z,
        0.022,
        deck_top_local_z,
        deck_roll,
    )
    right_button_joint_xyz = _rolled_point(
        0.070,
        deck_center_y,
        deck_center_z,
        0.022,
        deck_top_local_z,
        deck_roll,
    )
    left_sleeve_xyz = _rolled_point(
        -0.070,
        deck_center_y,
        deck_center_z,
        0.022,
        deck_top_local_z - 0.020,
        deck_roll,
    )
    right_sleeve_xyz = _rolled_point(
        0.070,
        deck_center_y,
        deck_center_z,
        0.022,
        deck_top_local_z - 0.020,
        deck_roll,
    )
    cabinet.visual(
        Box((0.056, 0.056, 0.028)),
        origin=Origin(xyz=left_sleeve_xyz, rpy=(deck_roll, 0.0, 0.0)),
        material=sleeve_black,
        name="left_button_sleeve",
    )
    cabinet.visual(
        Box((0.056, 0.056, 0.028)),
        origin=Origin(xyz=right_sleeve_xyz, rpy=(deck_roll, 0.0, 0.0)),
        material=sleeve_black,
        name="right_button_sleeve",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.82, 0.78, 1.82)),
        mass=88.0,
        origin=Origin(xyz=(0.0, 0.03, 0.91)),
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Box((0.46, 0.018, 0.25)),
        origin=Origin(xyz=(0.0, 0.009, 0.125)),
        material=hatch_metal,
        name="hatch_panel",
    )
    service_hatch.visual(
        Box((0.16, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, -0.004, 0.205)),
        material=coin_metal,
        name="hatch_pull",
    )
    service_hatch.inertial = Inertial.from_geometry(
        Box((0.46, 0.018, 0.25)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.009, 0.125)),
    )

    for part_name, origin_xyz in joystick_specs:
        joystick = model.part(part_name)
        joystick.visual(
            Cylinder(radius=0.010, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.031)),
            material=slot_black,
            name="joystick_shaft",
        )
        joystick.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.066)),
            material=joystick_red,
            name="joystick_ball",
        )
        joystick.inertial = Inertial.from_geometry(
            Box((0.050, 0.050, 0.090)),
            mass=0.10,
            origin=Origin(xyz=(0.0, 0.0, 0.045)),
        )
        model.articulation(
            f"cabinet_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=joystick,
            origin=Origin(xyz=origin_xyz, rpy=(deck_roll, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=2.5,
                lower=-0.35,
                upper=0.35,
            ),
        )

    for part_name, origin_xyz, button_material, _guide_name in action_button_specs:
        action_button = model.part(part_name)
        action_button.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=button_material,
            name="button_cap",
        )
        action_button.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=0.010),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
        )
        model.articulation(
            f"cabinet_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=action_button,
            origin=Origin(xyz=origin_xyz, rpy=(deck_roll, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=0.12,
                lower=0.0,
                upper=0.003,
            ),
        )

    left_start_button = model.part("left_start_button")
    left_start_button.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=slot_black,
        name="button_plunger",
    )
    left_start_button.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=button_blue,
        name="button_cap",
    )
    left_start_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.034),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    right_start_button = model.part("right_start_button")
    right_start_button.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=slot_black,
        name="button_plunger",
    )
    right_start_button.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=button_red,
        name="button_cap",
    )
    right_start_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.034),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    model.articulation(
        "cabinet_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_hatch,
        origin=Origin(xyz=(0.0, -0.368, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "cabinet_to_left_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_start_button,
        origin=Origin(xyz=left_button_joint_xyz, rpy=(deck_roll, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "cabinet_to_right_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_start_button,
        origin=Origin(xyz=right_button_joint_xyz, rpy=(deck_roll, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
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

    cabinet = object_model.get_part("cabinet")
    service_hatch = object_model.get_part("service_hatch")
    player_0_joystick = object_model.get_part("player_0_joystick")
    player_1_joystick = object_model.get_part("player_1_joystick")
    action_button_0_0 = object_model.get_part("action_button_0_0")
    action_button_0_1 = object_model.get_part("action_button_0_1")
    action_button_0_2 = object_model.get_part("action_button_0_2")
    action_button_1_0 = object_model.get_part("action_button_1_0")
    action_button_1_1 = object_model.get_part("action_button_1_1")
    action_button_1_2 = object_model.get_part("action_button_1_2")
    left_start_button = object_model.get_part("left_start_button")
    right_start_button = object_model.get_part("right_start_button")
    hatch_joint = object_model.get_articulation("cabinet_to_service_hatch")
    player_0_joystick_joint = object_model.get_articulation("cabinet_to_player_0_joystick")
    player_1_joystick_joint = object_model.get_articulation("cabinet_to_player_1_joystick")
    action_button_0_0_joint = object_model.get_articulation("cabinet_to_action_button_0_0")
    action_button_0_1_joint = object_model.get_articulation("cabinet_to_action_button_0_1")
    action_button_0_2_joint = object_model.get_articulation("cabinet_to_action_button_0_2")
    action_button_1_0_joint = object_model.get_articulation("cabinet_to_action_button_1_0")
    action_button_1_1_joint = object_model.get_articulation("cabinet_to_action_button_1_1")
    action_button_1_2_joint = object_model.get_articulation("cabinet_to_action_button_1_2")
    left_button_joint = object_model.get_articulation("cabinet_to_left_start_button")
    right_button_joint = object_model.get_articulation("cabinet_to_right_start_button")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.allow_overlap(
        cabinet,
        left_start_button,
        elem_a="left_button_sleeve",
        elem_b="button_plunger",
        reason="The under-deck guide sleeve is modeled as a simple solid proxy around the moving left start-button plunger.",
    )
    ctx.allow_overlap(
        cabinet,
        right_start_button,
        elem_a="right_button_sleeve",
        elem_b="button_plunger",
        reason="The under-deck guide sleeve is modeled as a simple solid proxy around the moving right start-button plunger.",
    )

    ctx.check(
        "service hatch hinge uses bottom horizontal x axis",
        hatch_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={hatch_joint.axis}",
    )
    ctx.check(
        "start buttons use independent downward prismatic axes",
        left_button_joint.axis == (0.0, 0.0, -1.0)
        and right_button_joint.axis == (0.0, 0.0, -1.0),
        details=f"left={left_button_joint.axis}, right={right_button_joint.axis}",
    )
    ctx.check(
        "player joysticks tilt independently from the control deck",
        player_0_joystick_joint.articulation_type == ArticulationType.REVOLUTE
        and player_1_joystick_joint.articulation_type == ArticulationType.REVOLUTE
        and player_0_joystick_joint.axis == (1.0, 0.0, 0.0)
        and player_1_joystick_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"left={player_0_joystick_joint.axis}/{player_0_joystick_joint.articulation_type}, "
            f"right={player_1_joystick_joint.axis}/{player_1_joystick_joint.articulation_type}"
        ),
    )
    ctx.check(
        "action buttons use independent downward prismatic axes",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC and joint.axis == (0.0, 0.0, -1.0)
            for joint in (
                action_button_0_0_joint,
                action_button_0_1_joint,
                action_button_0_2_joint,
                action_button_1_0_joint,
                action_button_1_1_joint,
                action_button_1_2_joint,
            )
        ),
        details=(
            f"left={[action_button_0_0_joint.axis, action_button_0_1_joint.axis, action_button_0_2_joint.axis]}, "
            f"right={[action_button_1_0_joint.axis, action_button_1_1_joint.axis, action_button_1_2_joint.axis]}"
        ),
    )
    ctx.expect_contact(
        service_hatch,
        cabinet,
        elem_a="hatch_panel",
        elem_b="lower_shell",
        name="service hatch seats against the cabinet front",
    )
    ctx.expect_overlap(
        left_start_button,
        cabinet,
        axes="xy",
        elem_a="button_cap",
        elem_b="control_deck",
        min_overlap=0.05,
        name="left start button sits over the broad control deck",
    )
    ctx.expect_overlap(
        right_start_button,
        cabinet,
        axes="xy",
        elem_a="button_cap",
        elem_b="control_deck",
        min_overlap=0.05,
        name="right start button sits over the broad control deck",
    )
    ctx.expect_overlap(
        left_start_button,
        cabinet,
        axes="z",
        elem_a="button_plunger",
        elem_b="left_button_sleeve",
        min_overlap=0.015,
        name="left plunger stays captured inside its sleeve at rest",
    )
    ctx.expect_overlap(
        right_start_button,
        cabinet,
        axes="z",
        elem_a="button_plunger",
        elem_b="right_button_sleeve",
        min_overlap=0.015,
        name="right plunger stays captured inside its sleeve at rest",
    )
    for button, guide_name, label in (
        (action_button_0_0, "action_button_guide_0_0", "left outer"),
        (action_button_0_1, "action_button_guide_0_1", "left middle"),
        (action_button_0_2, "action_button_guide_0_2", "left inner"),
        (action_button_1_0, "action_button_guide_1_0", "right outer"),
        (action_button_1_1, "action_button_guide_1_1", "right middle"),
        (action_button_1_2, "action_button_guide_1_2", "right inner"),
    ):
        ctx.expect_overlap(
            button,
            cabinet,
            axes="xy",
            elem_a="button_cap",
            elem_b=guide_name,
            min_overlap=0.025,
            name=f"{label} action button stays centered in its guide",
        )
    ctx.expect_origin_gap(
        right_start_button,
        left_start_button,
        axis="x",
        min_gap=0.10,
        name="start buttons are clearly separated for two players",
    )

    left_coin_slot_aabb = ctx.part_element_world_aabb(cabinet, elem="left_coin_slot")
    right_coin_slot_aabb = ctx.part_element_world_aabb(cabinet, elem="right_coin_slot")
    closed_hatch_aabb = ctx.part_world_aabb(service_hatch)
    ctx.check(
        "coin slots sit above the service hatch",
        left_coin_slot_aabb is not None
        and right_coin_slot_aabb is not None
        and closed_hatch_aabb is not None
        and left_coin_slot_aabb[0][2] > closed_hatch_aabb[1][2] + 0.14
        and right_coin_slot_aabb[0][2] > closed_hatch_aabb[1][2] + 0.14,
        details=(
            f"left_slot={left_coin_slot_aabb}, right_slot={right_coin_slot_aabb}, "
            f"hatch={closed_hatch_aabb}"
        ),
    )

    left_rest = ctx.part_world_position(left_start_button)
    right_rest = ctx.part_world_position(right_start_button)
    closed_hatch = ctx.part_world_aabb(service_hatch)
    with ctx.pose({hatch_joint: 1.05}):
        opened_hatch = ctx.part_world_aabb(service_hatch)
    ctx.check(
        "service hatch opens outward and swings down",
        closed_hatch is not None
        and opened_hatch is not None
        and opened_hatch[0][1] < closed_hatch[0][1] - 0.08
        and opened_hatch[1][2] < closed_hatch[1][2] - 0.05,
        details=f"closed={closed_hatch}, opened={opened_hatch}",
    )

    for joystick, joint, label in (
        (player_0_joystick, player_0_joystick_joint, "left"),
        (player_1_joystick, player_1_joystick_joint, "right"),
    ):
        rest_ball = aabb_center(ctx.part_element_world_aabb(joystick, elem="joystick_ball"))
        with ctx.pose({joint: 0.25}):
            tilted_ball = aabb_center(ctx.part_element_world_aabb(joystick, elem="joystick_ball"))
        ctx.check(
            f"{label} joystick visibly tilts from center",
            rest_ball is not None
            and tilted_ball is not None
            and abs(tilted_ball[0] - rest_ball[0]) < 1e-6
            and abs(tilted_ball[1] - rest_ball[1]) > 0.01
            and abs(tilted_ball[2] - rest_ball[2]) > 0.004,
            details=f"rest={rest_ball}, tilted={tilted_ball}",
        )

    left_pressed = None
    right_pressed = None
    with ctx.pose({left_button_joint: 0.004}):
        left_pressed = ctx.part_world_position(left_start_button)
        ctx.expect_overlap(
            left_start_button,
            cabinet,
            axes="z",
            elem_a="button_plunger",
            elem_b="left_button_sleeve",
            min_overlap=0.011,
            name="left plunger remains retained when pressed",
        )
    with ctx.pose({right_button_joint: 0.004}):
        right_pressed = ctx.part_world_position(right_start_button)
        ctx.expect_overlap(
            right_start_button,
            cabinet,
            axes="z",
            elem_a="button_plunger",
            elem_b="right_button_sleeve",
            min_overlap=0.011,
            name="right plunger remains retained when pressed",
        )
    ctx.check(
        "left start button depresses downward",
        left_rest is not None and left_pressed is not None and left_pressed[2] < left_rest[2] - 0.002,
        details=f"rest={left_rest}, pressed={left_pressed}",
    )
    ctx.check(
        "right start button depresses downward",
        right_rest is not None and right_pressed is not None and right_pressed[2] < right_rest[2] - 0.002,
        details=f"rest={right_rest}, pressed={right_pressed}",
    )

    for button, joint, label in (
        (action_button_0_0, action_button_0_0_joint, "left outer"),
        (action_button_0_1, action_button_0_1_joint, "left middle"),
        (action_button_0_2, action_button_0_2_joint, "left inner"),
        (action_button_1_0, action_button_1_0_joint, "right outer"),
        (action_button_1_1, action_button_1_1_joint, "right middle"),
        (action_button_1_2, action_button_1_2_joint, "right inner"),
    ):
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{label} action button depresses downward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
