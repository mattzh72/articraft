from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

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

ASSETS = AssetContext.from_script(__file__)

BASE_OUTER = 0.140
BASE_INNER = 0.100
BASE_PLATE = 0.010
BASE_RAIL = 0.020
TOWER_THICK = 0.014
TOWER_WIDTH = 0.030
TOWER_HEIGHT = 0.060
TOWER_CENTER_X = 0.046

PIVOT_Z = 0.062

OUTER_CHEEK_X = 0.008
OUTER_CHEEK_Y = 0.008
OUTER_CHEEK_Z = 0.056
OUTER_CHEEK_CENTER_Y = 0.030
OUTER_CHEEK_CENTER_Z = -0.006
OUTER_BRIDGE_Y = 0.052
OUTER_BRIDGE_Z = 0.008
OUTER_BRIDGE_CENTER_Z = -0.034
OUTER_PAD_X = 0.014
OUTER_PAD_Y = 0.014
OUTER_PAD_Z = 0.012
OUTER_PAD_CENTER_X = 0.032
OUTER_PAD_CENTER_Z = 0.000
OUTER_LINK_X = 0.008
OUTER_LINK_Y = 0.014
OUTER_LINK_Z = 0.024
OUTER_LINK_CENTER_X = 0.028
OUTER_LINK_CENTER_Z = -0.018

INNER_FRAME_Y = 0.008
INNER_SIDE_X = 0.008
INNER_SIDE_Z = 0.022
INNER_SIDE_CENTER_X = 0.014
INNER_SIDE_CENTER_Z = -0.014
INNER_LINK_X = 0.032
INNER_LINK_Z = 0.008
INNER_LINK_CENTER_Z = -0.022
INNER_COLUMN_X = 0.006
INNER_COLUMN_Z = 0.044
INNER_COLUMN_CENTER_Z = -0.002
INNER_TRUNNION_RADIUS = 0.0045
INNER_TRUNNION_SEG_LEN = 0.022
INNER_TRUNNION_CENTER_Y = 0.015
INNER_MOUNT_RADIUS = 0.010
INNER_MOUNT_HEIGHT = 0.006
INNER_MOUNT_CENTER_Z = 0.015

STICK_COLLAR_RADIUS = 0.009
STICK_COLLAR_HEIGHT = 0.010
STICK_COLLAR_CENTER_Z = 0.023
STICK_SHAFT_RADIUS = 0.0055
STICK_SHAFT_HEIGHT = 0.070
STICK_SHAFT_CENTER_Z = 0.063
STICK_GRIP_RADIUS = 0.0105
STICK_GRIP_HEIGHT = 0.024
STICK_GRIP_CENTER_Z = 0.110
STICK_KNOB_RADIUS = 0.012
STICK_KNOB_CENTER_Z = 0.128

PITCH_LIMIT = math.radians(20.0)
ROLL_LIMIT = math.radians(20.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gimballed_joystick", assets=ASSETS)

    base_color = model.material("base_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    yoke_color = model.material("yoke_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    stick_color = model.material("stick_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((BASE_OUTER, BASE_RAIL, BASE_PLATE)),
        material=base_color,
        origin=Origin(xyz=(0.0, BASE_OUTER / 2.0 - BASE_RAIL / 2.0, BASE_PLATE / 2.0)),
        name="front_rail",
    )
    base.visual(
        Box((BASE_OUTER, BASE_RAIL, BASE_PLATE)),
        material=base_color,
        origin=Origin(xyz=(0.0, -(BASE_OUTER / 2.0 - BASE_RAIL / 2.0), BASE_PLATE / 2.0)),
        name="back_rail",
    )
    base.visual(
        Box((BASE_RAIL, BASE_INNER, BASE_PLATE)),
        material=base_color,
        origin=Origin(xyz=(BASE_OUTER / 2.0 - BASE_RAIL / 2.0, 0.0, BASE_PLATE / 2.0)),
        name="right_rail",
    )
    base.visual(
        Box((BASE_RAIL, BASE_INNER, BASE_PLATE)),
        material=base_color,
        origin=Origin(xyz=(-(BASE_OUTER / 2.0 - BASE_RAIL / 2.0), 0.0, BASE_PLATE / 2.0)),
        name="left_rail",
    )
    base.visual(
        Box((TOWER_THICK, TOWER_WIDTH, TOWER_HEIGHT)),
        origin=Origin(xyz=(-TOWER_CENTER_X, 0.0, BASE_PLATE + TOWER_HEIGHT / 2.0)),
        material=base_color,
        name="left_tower",
    )
    base.visual(
        Box((TOWER_THICK, TOWER_WIDTH, TOWER_HEIGHT)),
        origin=Origin(xyz=(TOWER_CENTER_X, 0.0, BASE_PLATE + TOWER_HEIGHT / 2.0)),
        material=base_color,
        name="right_tower",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_OUTER, BASE_OUTER, PIVOT_Z + 0.030)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, (PIVOT_Z + 0.030) / 2.0)),
    )

    outer = model.part("outer_pitch_yoke")
    outer.visual(
        Box((OUTER_CHEEK_X, OUTER_CHEEK_Y, OUTER_CHEEK_Z)),
        material=yoke_color,
        origin=Origin(xyz=(0.0, OUTER_CHEEK_CENTER_Y, OUTER_CHEEK_CENTER_Z)),
        name="right_cheek",
    )
    outer.visual(
        Box((OUTER_CHEEK_X, OUTER_CHEEK_Y, OUTER_CHEEK_Z)),
        material=yoke_color,
        origin=Origin(xyz=(0.0, -OUTER_CHEEK_CENTER_Y, OUTER_CHEEK_CENTER_Z)),
        name="left_cheek",
    )
    outer.visual(
        Box((0.064, OUTER_BRIDGE_Y, OUTER_BRIDGE_Z)),
        material=yoke_color,
        origin=Origin(xyz=(0.0, 0.0, OUTER_BRIDGE_CENTER_Z)),
        name="lower_bridge",
    )
    outer.visual(
        Box((OUTER_LINK_X, OUTER_LINK_Y, OUTER_LINK_Z)),
        material=yoke_color,
        origin=Origin(xyz=(OUTER_LINK_CENTER_X, 0.0, OUTER_LINK_CENTER_Z)),
        name="right_link",
    )
    outer.visual(
        Box((OUTER_LINK_X, OUTER_LINK_Y, OUTER_LINK_Z)),
        material=yoke_color,
        origin=Origin(xyz=(-OUTER_LINK_CENTER_X, 0.0, OUTER_LINK_CENTER_Z)),
        name="left_link",
    )
    outer.visual(
        Box((OUTER_PAD_X, OUTER_PAD_Y, OUTER_PAD_Z)),
        origin=Origin(xyz=(-OUTER_PAD_CENTER_X, 0.0, OUTER_PAD_CENTER_Z)),
        material=yoke_color,
        name="left_pitch_pad",
    )
    outer.visual(
        Box((OUTER_PAD_X, OUTER_PAD_Y, OUTER_PAD_Z)),
        origin=Origin(xyz=(OUTER_PAD_CENTER_X, 0.0, OUTER_PAD_CENTER_Z)),
        material=yoke_color,
        name="right_pitch_pad",
    )
    outer.inertial = Inertial.from_geometry(Box((0.080, 0.072, 0.060)), mass=0.25)

    inner = model.part("inner_roll_yoke")
    inner.visual(
        Box((INNER_SIDE_X, INNER_FRAME_Y, INNER_SIDE_Z)),
        material=yoke_color,
        origin=Origin(xyz=(INNER_SIDE_CENTER_X, 0.0, INNER_SIDE_CENTER_Z)),
        name="right_side",
    )
    inner.visual(
        Box((INNER_SIDE_X, INNER_FRAME_Y, INNER_SIDE_Z)),
        material=yoke_color,
        origin=Origin(xyz=(-INNER_SIDE_CENTER_X, 0.0, INNER_SIDE_CENTER_Z)),
        name="left_side",
    )
    inner.visual(
        Box((INNER_LINK_X, INNER_FRAME_Y, INNER_LINK_Z)),
        material=yoke_color,
        origin=Origin(xyz=(0.0, 0.0, INNER_LINK_CENTER_Z)),
        name="lower_link",
    )
    inner.visual(
        Box((INNER_COLUMN_X, INNER_FRAME_Y, INNER_COLUMN_Z)),
        material=yoke_color,
        origin=Origin(xyz=(0.0, 0.0, INNER_COLUMN_CENTER_Z)),
        name="center_column",
    )
    inner.visual(
        Cylinder(radius=INNER_TRUNNION_RADIUS, length=INNER_TRUNNION_SEG_LEN),
        origin=Origin(xyz=(0.0, INNER_TRUNNION_CENTER_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yoke_color,
        name="right_trunnion",
    )
    inner.visual(
        Cylinder(radius=INNER_TRUNNION_RADIUS, length=INNER_TRUNNION_SEG_LEN),
        origin=Origin(xyz=(0.0, -INNER_TRUNNION_CENTER_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yoke_color,
        name="left_trunnion",
    )
    inner.visual(
        Cylinder(radius=INNER_MOUNT_RADIUS, length=INNER_MOUNT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, INNER_MOUNT_CENTER_Z)),
        material=yoke_color,
        name="stick_mount",
    )
    inner.inertial = Inertial.from_geometry(Box((0.045, 0.040, 0.050)), mass=0.18)

    stick = model.part("control_stick")
    stick.visual(
        Cylinder(radius=STICK_COLLAR_RADIUS, length=STICK_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, STICK_COLLAR_CENTER_Z)),
        material=stick_color,
        name="collar",
    )
    stick.visual(
        Cylinder(radius=STICK_SHAFT_RADIUS, length=STICK_SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, STICK_SHAFT_CENTER_Z)),
        material=stick_color,
        name="shaft",
    )
    stick.visual(
        Cylinder(radius=STICK_GRIP_RADIUS, length=STICK_GRIP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, STICK_GRIP_CENTER_Z)),
        material=stick_color,
        name="grip",
    )
    stick.visual(
        Cylinder(radius=STICK_KNOB_RADIUS, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, STICK_KNOB_CENTER_Z)),
        material=stick_color,
        name="knob",
    )
    stick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.120),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    model.articulation(
        "base_to_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-PITCH_LIMIT,
            upper=PITCH_LIMIT,
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
        ),
    )
    model.articulation(
        "roll_to_stick",
        ArticulationType.FIXED,
        parent=inner,
        child=stick,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    outer = object_model.get_part("outer_pitch_yoke")
    inner = object_model.get_part("inner_roll_yoke")
    stick = object_model.get_part("control_stick")
    pitch = object_model.get_articulation("base_to_pitch")
    roll = object_model.get_articulation("pitch_to_roll")
    base_left_tower = base.get_visual("left_tower")
    base_right_tower = base.get_visual("right_tower")
    outer_left_pad = outer.get_visual("left_pitch_pad")
    outer_right_pad = outer.get_visual("right_pitch_pad")
    outer_left_cheek = outer.get_visual("left_cheek")
    outer_right_cheek = outer.get_visual("right_cheek")
    inner_left_hub = inner.get_visual("left_trunnion")
    inner_right_hub = inner.get_visual("right_trunnion")
    inner_mount = inner.get_visual("stick_mount")
    stick_collar = stick.get_visual("collar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24, ignore_adjacent=True, ignore_fixed=True)

    ctx.check(
        "pitch_axis_is_x",
        tuple(round(v, 6) for v in pitch.axis) == (1.0, 0.0, 0.0),
        details=f"expected pitch axis (1,0,0), got {pitch.axis}",
    )
    ctx.check(
        "roll_axis_is_y",
        tuple(round(v, 6) for v in roll.axis) == (0.0, 1.0, 0.0),
        details=f"expected roll axis (0,1,0), got {roll.axis}",
    )

    ctx.expect_contact(
        base,
        outer,
        elem_a=base_left_tower,
        elem_b=outer_left_pad,
        name="left_pitch_pivot_is_mounted",
    )
    ctx.expect_contact(
        base,
        outer,
        elem_a=base_right_tower,
        elem_b=outer_right_pad,
        name="right_pitch_pivot_is_mounted",
    )
    ctx.expect_contact(
        outer,
        inner,
        elem_a=outer_left_cheek,
        elem_b=inner_left_hub,
        name="left_roll_pivot_is_mounted",
    )
    ctx.expect_contact(
        outer,
        inner,
        elem_a=outer_right_cheek,
        elem_b=inner_right_hub,
        name="right_roll_pivot_is_mounted",
    )
    ctx.expect_contact(
        inner,
        stick,
        elem_a=inner_mount,
        elem_b=stick_collar,
        name="stick_is_fixed_to_inner_gimbal",
    )
    ctx.expect_overlap(
        outer,
        base,
        axes="xy",
        min_overlap=0.055,
        name="outer_yoke_sits_inside_base_footprint",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        margin=0.006,
        name="inner_yoke_stays_inside_outer_pivot_envelope",
    )
    ctx.expect_origin_distance(
        stick,
        outer,
        axes="xy",
        max_dist=1e-6,
        name="stick_origin_intersects_pitch_axis",
    )
    ctx.expect_origin_distance(
        inner,
        outer,
        axes="xyz",
        max_dist=1e-6,
        name="gimbal_axes_intersect_at_center",
    )

    with ctx.pose({pitch: PITCH_LIMIT, roll: -ROLL_LIMIT}):
        ctx.expect_contact(
            base,
            outer,
            elem_a=base_left_tower,
            elem_b=outer_left_pad,
            name="left_pitch_pivot_keeps_contact_at_limit",
        )
        ctx.expect_contact(
            outer,
            inner,
            elem_a=outer_right_cheek,
            elem_b=inner_right_hub,
            name="roll_pivot_keeps_contact_at_combined_limit",
        )
        ctx.expect_origin_distance(
            stick,
            base,
            axes="xy",
            max_dist=1e-6,
            name="stick_pivots_about_base_center",
        )
        ctx.expect_gap(
            stick,
            base,
            axis="z",
            min_gap=0.002,
            name="tilted_stick_clears_base_frame",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
