from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

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


def _build_base_bracket() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(0.110, 0.180, 0.012, centered=(True, True, True))
        .translate((0.0, 0.0, -0.071))
    )

    wall_blank = cq.Workplane("XY").box(0.012, 0.150, 0.130, centered=(True, True, True))
    window_cutter = (
        cq.Workplane("XY")
        .box(0.020, 0.090, 0.080, centered=(True, True, True))
        .translate((0.0, 0.0, 0.005))
    )
    axle_cutter = cq.Workplane("YZ").circle(0.0095).extrude(0.020, both=True)

    front_wall = wall_blank.translate((0.056, 0.0, 0.0)).cut(window_cutter.translate((0.056, 0.0, 0.0))).cut(
        axle_cutter.translate((0.056, 0.0, 0.0))
    )
    rear_wall = wall_blank.translate((-0.056, 0.0, 0.0)).cut(window_cutter.translate((-0.056, 0.0, 0.0))).cut(
        axle_cutter.translate((-0.056, 0.0, 0.0))
    )

    foot_rib = (
        cq.Workplane("XY")
        .box(0.090, 0.020, 0.036, centered=(True, True, True))
        .translate((0.0, 0.0, -0.047))
    )
    cross_rib = (
        cq.Workplane("XY")
        .box(0.020, 0.120, 0.028, centered=(True, True, True))
        .translate((0.0, 0.0, -0.051))
    )

    return base_plate.union(front_wall).union(rear_wall).union(foot_rib).union(cross_rib)


def _build_roll_frame() -> cq.Workplane:
    outer_ring = cq.Workplane("XY").box(0.020, 0.120, 0.100, centered=(True, True, True))
    inner_void = cq.Workplane("XY").box(0.030, 0.092, 0.072, centered=(True, True, True))
    pitch_bore = cq.Workplane("XZ").circle(0.0066).extrude(0.140, both=True)
    roll_shaft = cq.Workplane("YZ").circle(0.0086).extrude(0.048, both=True)

    return outer_ring.cut(inner_void).cut(pitch_bore).union(roll_shaft)


def _build_pitch_cradle() -> cq.Workplane:
    left_side = (
        cq.Workplane("XY")
        .box(0.070, 0.008, 0.060, centered=(True, True, True))
        .translate((0.0, 0.038, 0.005))
    )
    right_side = (
        cq.Workplane("XY")
        .box(0.070, 0.008, 0.060, centered=(True, True, True))
        .translate((0.0, -0.038, 0.005))
    )
    lower_beam = (
        cq.Workplane("XY")
        .box(0.050, 0.068, 0.008, centered=(True, True, True))
        .translate((0.0, 0.0, -0.021))
    )
    rear_tie = (
        cq.Workplane("XY")
        .box(0.008, 0.068, 0.030, centered=(True, True, True))
        .translate((-0.026, 0.0, 0.000))
    )
    pitch_shaft = cq.Workplane("XZ").circle(0.0060).extrude(0.043, both=True)
    standoffs = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.020, -0.025),
                (-0.020, 0.025),
                (0.020, -0.025),
                (0.020, 0.025),
            ]
        )
        .circle(0.0035)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.012))
    )

    return left_side.union(right_side).union(lower_beam).union(rear_tie).union(pitch_shaft).union(standoffs)


def _build_top_plate() -> cq.Workplane:
    return cq.Workplane("XY").box(0.060, 0.050, 0.005, centered=(True, True, True)).edges("|Z").fillet(0.003)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_roll_module", assets=ASSETS)

    anodized_gray = model.material("anodized_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    matte_black = model.material("matte_black", rgba=(0.14, 0.15, 0.16, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((0.110, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.071)),
        material=anodized_gray,
        name="base_plate",
    )
    base_bracket.visual(
        Box((0.012, 0.150, 0.130)),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material=anodized_gray,
        name="front_cheek",
    )
    base_bracket.visual(
        Box((0.012, 0.150, 0.130)),
        origin=Origin(xyz=(-0.056, 0.0, 0.0)),
        material=anodized_gray,
        name="rear_cheek",
    )
    base_bracket.visual(
        Box((0.020, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=anodized_gray,
        name="pedestal",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.124, 0.180, 0.142)),
        mass=0.80,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    roll_frame = model.part("roll_frame")
    roll_frame.visual(
        Box((0.010, 0.016, 0.092)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=machined_aluminum,
        name="left_rail",
    )
    roll_frame.visual(
        Box((0.010, 0.016, 0.092)),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=machined_aluminum,
        name="right_rail",
    )
    roll_frame.visual(
        Box((0.010, 0.084, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=machined_aluminum,
        name="top_rail",
    )
    roll_frame.visual(
        Box((0.010, 0.084, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=machined_aluminum,
        name="bottom_rail",
    )
    roll_frame.visual(
        Box((0.024, 0.016, 0.016)),
        origin=Origin(),
        material=machined_aluminum,
        name="center_hub",
    )
    roll_frame.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=machined_aluminum,
        name="lower_strut",
    )
    roll_frame.visual(
        Cylinder(radius=0.008, length=0.038),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_aluminum,
        name="front_trunnion",
    )
    roll_frame.visual(
        Cylinder(radius=0.008, length=0.038),
        origin=Origin(xyz=(-0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_aluminum,
        name="rear_trunnion",
    )
    roll_frame.inertial = Inertial.from_geometry(Box((0.096, 0.120, 0.100)), mass=0.28)

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Box((0.060, 0.008, 0.040)),
        origin=Origin(xyz=(0.008, 0.022, 0.006)),
        material=machined_aluminum,
        name="left_arm",
    )
    pitch_cradle.visual(
        Box((0.060, 0.008, 0.040)),
        origin=Origin(xyz=(0.008, -0.022, 0.006)),
        material=machined_aluminum,
        name="right_arm",
    )
    pitch_cradle.visual(
        Box((0.040, 0.044, 0.006)),
        origin=Origin(xyz=(0.008, 0.0, 0.022)),
        material=machined_aluminum,
        name="plate_seat",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_aluminum,
        name="pitch_core",
    )
    pitch_cradle.inertial = Inertial.from_geometry(Box((0.060, 0.044, 0.046)), mass=0.18)

    top_plate = model.part("top_plate")
    top_plate.visual(
        Box((0.060, 0.050, 0.005)),
        material=matte_black,
        name="plate_shell",
    )
    top_plate.inertial = Inertial.from_geometry(Box((0.060, 0.050, 0.005)), mass=0.06)

    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=roll_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=roll_frame,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-math.radians(60.0),
            upper=math.radians(60.0),
        ),
    )

    model.articulation(
        "plate_mount",
        ArticulationType.FIXED,
        parent=pitch_cradle,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_bracket = object_model.get_part("base_bracket")
    roll_frame = object_model.get_part("roll_frame")
    pitch_cradle = object_model.get_part("pitch_cradle")
    top_plate = object_model.get_part("top_plate")
    roll_joint = object_model.get_articulation("roll_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    plate_mount = object_model.get_articulation("plate_mount")
    center_hub = roll_frame.get_visual("center_hub")
    pitch_core = pitch_cradle.get_visual("pitch_core")
    plate_seat = pitch_cradle.get_visual("plate_seat")
    plate_shell = top_plate.get_visual("plate_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        pitch_cradle,
        roll_frame,
        reason="The pitch pivot uses a nested axle running through the roll frame's center hub.",
        elem_a=pitch_core,
        elem_b=center_hub,
    )

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

    ctx.check(
        "all_expected_parts_present",
        all(part is not None for part in (base_bracket, roll_frame, pitch_cradle, top_plate)),
        "Pitch-roll module should contain base bracket, roll frame, pitch cradle, and top plate.",
    )
    ctx.check(
        "roll_joint_axis_and_limits",
        tuple(roll_joint.axis) == (1.0, 0.0, 0.0)
        and abs(roll_joint.motion_limits.lower + math.radians(45.0)) < 1e-6
        and abs(roll_joint.motion_limits.upper - math.radians(45.0)) < 1e-6,
        "Outer roll joint must rotate about the front-back x-axis through ±45°.",
    )
    ctx.check(
        "pitch_joint_axis_and_limits",
        tuple(pitch_joint.axis) == (0.0, 1.0, 0.0)
        and abs(pitch_joint.motion_limits.lower + math.radians(60.0)) < 1e-6
        and abs(pitch_joint.motion_limits.upper - math.radians(60.0)) < 1e-6,
        "Inner pitch joint must rotate about the left-right y-axis through ±60°.",
    )
    ctx.check(
        "top_plate_fixed_to_cradle",
        plate_mount.articulation_type == ArticulationType.FIXED,
        "The top plate should be rigidly mounted to the pitch cradle.",
    )

    ctx.expect_origin_distance(
        roll_frame,
        base_bracket,
        axes="yz",
        max_dist=0.001,
        name="roll_frame_centered_in_bracket",
    )
    ctx.expect_origin_distance(
        pitch_cradle,
        roll_frame,
        axes="xyz",
        max_dist=0.001,
        name="pitch_cradle_centered_in_roll_frame",
    )
    ctx.expect_origin_distance(
        top_plate,
        pitch_cradle,
        axes="xy",
        max_dist=0.001,
        name="top_plate_centered_on_pitch_cradle",
    )
    ctx.expect_contact(
        top_plate,
        pitch_cradle,
        elem_a=plate_shell,
        elem_b=plate_seat,
        name="top_plate_seated_on_cradle_standoffs",
    )
    ctx.expect_within(
        top_plate,
        roll_frame,
        axes="yz",
        margin=0.0,
        name="top_plate_within_roll_frame_window_at_rest",
    )
    ctx.expect_within(
        pitch_cradle,
        roll_frame,
        axes="yz",
        margin=0.003,
        name="pitch_cradle_nested_inside_roll_frame",
    )

    with ctx.pose({roll_joint: math.radians(35.0)}):
        ctx.expect_origin_distance(
            pitch_cradle,
            base_bracket,
            axes="xyz",
            max_dist=0.001,
            name="roll_pose_keeps_pitch_axis_concentric",
        )
        ctx.expect_within(
            top_plate,
            base_bracket,
            axes="xz",
            margin=0.020,
            name="rolled_module_stays_inside_bracket_envelope",
        )

    with ctx.pose({pitch_joint: math.radians(50.0)}):
        ctx.expect_within(
            top_plate,
            roll_frame,
            axes="yz",
            margin=0.006,
            name="pitched_top_plate_remains_inside_roll_frame_window",
        )
        ctx.expect_contact(
            top_plate,
            pitch_cradle,
            elem_a=plate_shell,
            elem_b=plate_seat,
            name="pitched_top_plate_remains_registered_to_cradle",
        )

    with ctx.pose({roll_joint: -math.radians(30.0), pitch_joint: -math.radians(45.0)}):
        ctx.expect_within(
            top_plate,
            base_bracket,
            axes="x",
            margin=0.020,
            name="combined_pose_keeps_top_plate_between_bracket_cheeks",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
