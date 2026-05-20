from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _shell_of_revolution(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
):
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _tube_shell(*, outer_radius: float, inner_radius: float, length: float, segments: int = 56):
    half = 0.5 * length
    return _shell_of_revolution(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_front_load_washer", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.71, 0.74, 0.77, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.66, 0.71, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.43, 0.46, 0.50, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.82, 0.90, 0.30))
    console_black = model.material("console_black", rgba=(0.08, 0.09, 0.10, 1.0))
    indicator_gray = model.material("indicator_gray", rgba=(0.20, 0.23, 0.26, 1.0))

    body_width = 0.86
    body_depth = 0.95
    body_height = 1.15
    wall = 0.025
    front_panel_y = -(0.5 * body_depth) + (0.5 * wall)
    rear_panel_y = (0.5 * body_depth) - (0.5 * wall)
    opening_z = 0.57

    front_bezel_mesh = _save_mesh(
        _shell_of_revolution(
            [(0.338, -0.018), (0.346, -0.004), (0.340, 0.014), (0.330, 0.030)],
            [(0.258, -0.014), (0.266, 0.000), (0.260, 0.020)],
        ),
        "front_bezel.obj",
    )
    door_outer_ring_mesh = _save_mesh(
        _shell_of_revolution(
            [(0.340, -0.030), (0.350, -0.014), (0.346, 0.010), (0.334, 0.034)],
            [(0.220, -0.024), (0.228, -0.006), (0.224, 0.020)],
        ),
        "door_outer_ring.obj",
    )
    door_inner_clamp_mesh = _save_mesh(
        _shell_of_revolution(
            [(0.246, -0.020), (0.252, -0.008), (0.248, 0.016)],
            [(0.212, -0.018), (0.218, -0.006), (0.214, 0.012)],
        ),
        "door_inner_clamp.obj",
    )
    drum_shell_mesh = _save_mesh(
        _shell_of_revolution(
            [(0.282, -0.290), (0.282, 0.255), (0.120, 0.292)],
            [(0.245, -0.268), (0.245, 0.232), (0.000, 0.288)],
            segments=84,
        ),
        "drum_shell.obj",
    )
    door_boot_mesh = _save_mesh(
        _shell_of_revolution(
            [(0.320, -0.055), (0.332, -0.010), (0.322, 0.052)],
            [(0.289, -0.050), (0.296, -0.004), (0.290, 0.046)],
        ),
        "door_boot.obj",
    )
    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=185.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * body_height)),
    )
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(-(0.5 * body_width) + (0.5 * wall), 0.0, 0.5 * body_height)),
        material=stainless,
        name="left_side",
    )
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=((0.5 * body_width) - (0.5 * wall), 0.0, 0.5 * body_height)),
        material=stainless,
        name="right_side",
    )
    body.visual(
        Box((body_width - (2.0 * wall), body_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_height - (0.5 * wall))),
        material=stainless,
        name="top_panel",
    )
    body.visual(
        Box((body_width - (2.0 * wall), body_depth, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_steel,
        name="base_plinth",
    )
    body.visual(
        Box((body_width - (2.0 * wall), wall, body_height)),
        origin=Origin(xyz=(0.0, rear_panel_y, 0.5 * body_height)),
        material=stainless,
        name="rear_panel",
    )
    body.visual(
        Box((0.81, wall, 0.225)),
        origin=Origin(xyz=(0.0, front_panel_y, 1.0125)),
        material=stainless,
        name="front_header",
    )
    body.visual(
        Box((0.78, wall, 0.22)),
        origin=Origin(xyz=(0.0, front_panel_y, 0.130)),
        material=stainless,
        name="front_sill",
    )
    body.visual(
        Box((0.12, wall, 0.67)),
        origin=Origin(xyz=(-0.325, front_panel_y, opening_z)),
        material=stainless,
        name="left_jamb",
    )
    body.visual(
        Box((0.12, wall, 0.67)),
        origin=Origin(xyz=(0.325, front_panel_y, opening_z)),
        material=stainless,
        name="right_jamb",
    )
    body.visual(
        Box((0.52, 0.080, 0.140)),
        origin=Origin(xyz=(0.0, front_panel_y - 0.030, 1.000)),
        material=console_black,
        name="control_console",
    )
    body.visual(
        Box((0.27, 0.008, 0.080)),
        origin=Origin(xyz=(-0.080, front_panel_y - 0.070, 1.015)),
        material=indicator_gray,
        name="display_panel",
    )
    body.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.0, front_panel_y + 0.014, opening_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="front_bezel",
    )
    body.visual(
        door_boot_mesh,
        origin=Origin(xyz=(0.0, -0.385, opening_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="door_boot",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, 0.457, opening_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing_housing",
    )
    body.visual(
        Box((0.18, 0.11, 0.08)),
        origin=Origin(xyz=(0.0, 0.395, opening_z + 0.120)),
        material=dark_steel,
        name="rear_support_upper",
    )
    body.visual(
        Box((0.18, 0.11, 0.08)),
        origin=Origin(xyz=(0.0, 0.395, opening_z - 0.120)),
        material=dark_steel,
        name="rear_support_lower",
    )
    body.visual(
        Box((0.100, 0.110, 0.460)),
        origin=Origin(xyz=(-0.435, -0.450, opening_z)),
        material=hinge_steel,
        name="hinge_pillar",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(-0.475, -0.495, opening_z + 0.150)),
        material=hinge_steel,
        name="upper_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(-0.475, -0.495, opening_z - 0.150)),
        material=hinge_steel,
        name="lower_hinge_barrel",
    )

    drum = model.part("drum")
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.28, length=0.60),
        mass=34.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    drum.visual(
        drum_shell_mesh,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.280, length=0.028),
        origin=Origin(xyz=(0.0, -0.276, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="drum_front_lip",
    )
    drum.visual(
        Cylinder(radius=0.245, length=0.036),
        origin=Origin(xyz=(0.0, 0.274, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_web",
    )
    drum.visual(
        Cylinder(radius=0.095, length=0.110),
        origin=Origin(xyz=(0.0, 0.347, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.013, length=0.140),
        origin=Origin(xyz=(0.0, 0.472, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_pin",
    )

    door_hinge_knuckle_mesh = _save_mesh(
        _tube_shell(outer_radius=0.021, inner_radius=0.014, length=0.090),
        "door_hinge_knuckle.obj",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((0.72, 0.11, 0.72)),
        mass=24.0,
        origin=Origin(xyz=(0.420, -0.025, 0.0)),
    )
    door.visual(
        door_outer_ring_mesh,
        origin=Origin(xyz=(0.460, -0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="door_outer_ring",
    )
    door.visual(
        door_inner_clamp_mesh,
        origin=Origin(xyz=(0.460, -0.024, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="door_inner_clamp",
    )
    door.visual(
        Cylinder(radius=0.212, length=0.014),
        origin=Origin(xyz=(0.460, -0.030, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="window_glass",
    )
    door.visual(
        Box((0.050, 0.018, 0.440)),
        origin=Origin(xyz=(0.105, -0.030, 0.0)),
        material=stainless,
        name="hinge_bridge",
    )
    door.visual(
        Box((0.090, 0.014, 0.050)),
        origin=Origin(xyz=(0.040, -0.022, 0.150)),
        material=stainless,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.090, 0.014, 0.050)),
        origin=Origin(xyz=(0.040, -0.022, -0.150)),
        material=stainless,
        name="lower_hinge_arm",
    )
    door.visual(
        door_hinge_knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=hinge_steel,
        name="upper_hinge_leaf",
    )
    door.visual(
        door_hinge_knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=hinge_steel,
        name="lower_hinge_leaf",
    )
    door.visual(
        Box((0.085, 0.040, 0.190)),
        origin=Origin(xyz=(0.765, -0.040, 0.0)),
        material=hinge_steel,
        name="handle_base",
    )
    door.visual(
        Box((0.060, 0.045, 0.180)),
        origin=Origin(xyz=(0.805, -0.074, 0.0)),
        material=hinge_steel,
        name="handle_grip",
    )
    door.visual(
        Box((0.090, 0.018, 0.100)),
        origin=Origin(xyz=(0.690, -0.030, 0.0)),
        material=rubber,
        name="latch_pad",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.050),
        mass=0.4,
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    selector_knob.visual(
        Cylinder(radius=0.050, length=0.050),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="dial_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=console_black,
        name="dial_center_cap",
    )
    selector_knob.visual(
        Box((0.014, 0.014, 0.028)),
        origin=Origin(xyz=(0.032, -0.050, 0.0)),
        material=console_black,
        name="grip_tab",
    )

    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, -0.115, opening_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=18.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.475, -0.495, opening_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.225, front_panel_y - 0.070, 1.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    selector_knob = object_model.get_part("selector_knob")

    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")
    selector_dial = object_model.get_articulation("selector_dial")

    front_bezel = body.get_visual("front_bezel")
    control_console = body.get_visual("control_console")
    rear_bearing_housing = body.get_visual("rear_bearing_housing")
    upper_hinge_barrel = body.get_visual("upper_hinge_barrel")
    lower_hinge_barrel = body.get_visual("lower_hinge_barrel")

    drum_shell = drum.get_visual("drum_shell")
    drum_front_lip = drum.get_visual("drum_front_lip")
    axle_pin = drum.get_visual("axle_pin")

    door_outer_ring = door.get_visual("door_outer_ring")
    door_glass = door.get_visual("window_glass")
    upper_hinge_leaf = door.get_visual("upper_hinge_leaf")
    lower_hinge_leaf = door.get_visual("lower_hinge_leaf")

    dial_body = selector_knob.get_visual("dial_body")
    grip_tab = selector_knob.get_visual("grip_tab")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        body,
        door,
        elem_a=upper_hinge_barrel,
        elem_b=upper_hinge_leaf,
        reason="Upper industrial hinge knuckle wraps around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a=lower_hinge_barrel,
        elem_b=lower_hinge_leaf,
        reason="Lower industrial hinge knuckle wraps around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pillar",
        elem_b=upper_hinge_leaf,
        reason="Upper hinge knuckle nests into the clevis face of the hinge pillar.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pillar",
        elem_b=lower_hinge_leaf,
        reason="Lower hinge knuckle nests into the clevis face of the hinge pillar.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48, name="articulations_clear_through_motion")

    ctx.expect_contact(
        drum,
        body,
        elem_a=axle_pin,
        elem_b=rear_bearing_housing,
        name="drum_axle_seats_against_the_rear_bearing_housing",
    )
    ctx.expect_gap(
        drum,
        body,
        axis="y",
        min_gap=0.010,
        max_gap=0.040,
        positive_elem=drum_front_lip,
        negative_elem=front_bezel,
        name="drum_sits_just_behind_the_front_bezel",
    )
    ctx.expect_overlap(
        drum,
        body,
        axes="xz",
        min_overlap=0.54,
        elem_a=drum_shell,
        elem_b=front_bezel,
        name="drum_fills_the_porthole_opening",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem=front_bezel,
        negative_elem=door_outer_ring,
        name="door_closes_tight_to_the_front_bezel",
    )
    ctx.expect_overlap(
        door,
        drum,
        axes="xz",
        min_overlap=0.42,
        elem_a=door_glass,
        elem_b=drum_shell,
        name="door_glass_is_centered_on_the_drum",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=upper_hinge_leaf,
        elem_b=upper_hinge_barrel,
        name="upper_industrial_hinge_knuckle_is_seated",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=lower_hinge_leaf,
        elem_b=lower_hinge_barrel,
        name="lower_industrial_hinge_knuckle_is_seated",
    )
    ctx.expect_contact(
        selector_knob,
        body,
        elem_a=dial_body,
        elem_b=control_console,
        name="selector_knob_is_mounted_on_the_console",
    )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="door_closed_pose_no_floating")
        door_rest_aabb = ctx.part_world_aabb(door)
        assert door_rest_aabb is not None
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_pose_no_floating")
            ctx.expect_contact(
                door,
                body,
                elem_a=upper_hinge_leaf,
                elem_b=upper_hinge_barrel,
                name="upper_hinge_keeps_contact_when_open",
            )
            ctx.expect_contact(
                door,
                body,
                elem_a=lower_hinge_leaf,
                elem_b=lower_hinge_barrel,
                name="lower_hinge_keeps_contact_when_open",
            )
            door_open_aabb = ctx.part_world_aabb(door)
            assert door_open_aabb is not None
            assert door_open_aabb[0][1] < door_rest_aabb[0][1] - 0.18

    with ctx.pose({drum_spin: pi}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drum_half_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="drum_half_turn_no_floating")
        ctx.expect_contact(
            drum,
            body,
            elem_a=axle_pin,
            elem_b=rear_bearing_housing,
            name="drum_axle_stays_supported_while_spinning",
        )

    grip_rest_aabb = ctx.part_element_world_aabb(selector_knob, elem=grip_tab)
    assert grip_rest_aabb is not None
    with ctx.pose({selector_dial: pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="selector_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="selector_quarter_turn_no_floating")
        ctx.expect_contact(
            selector_knob,
            body,
            elem_a=dial_body,
            elem_b=control_console,
            name="selector_knob_stays_seated_while_turning",
        )
        grip_quarter_aabb = ctx.part_element_world_aabb(selector_knob, elem=grip_tab)
        assert grip_quarter_aabb is not None
        assert grip_quarter_aabb[0][2] < grip_rest_aabb[0][2] - 0.020

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
