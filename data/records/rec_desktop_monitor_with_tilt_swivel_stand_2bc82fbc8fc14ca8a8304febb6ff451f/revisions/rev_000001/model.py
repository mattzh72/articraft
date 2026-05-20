from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    graphite = model.material("satin_graphite", rgba=(0.06, 0.065, 0.07, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.015, 0.017, 0.02, 1.0))
    soft_black = model.material("soft_black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("dark_gloss_glass", rgba=(0.0, 0.002, 0.006, 1.0))
    active_screen = model.material("deep_anti_glare_panel", rgba=(0.015, 0.020, 0.027, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    # Desk-scale, broad weighted base.
    base = model.part("base")
    base.visual(
        Box((0.56, 0.34, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.125, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0405)),
        material=dark_metal,
        name="swivel_turntable",
    )
    base.visual(
        Box((0.42, 0.22, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, 0.031)),
        material=soft_black,
        name="recessed_top_inlay",
    )

    # Rotating stand spine.  The four interlocking walls form a hollow sleeve
    # so the height stage can slide without being represented as a collision.
    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.095, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_metal,
        name="lower_collar",
    )
    stand.visual(
        Box((0.016, 0.070, 0.450)),
        origin=Origin(xyz=(-0.062, 0.045, 0.260)),
        material=graphite,
        name="sleeve_wall_0",
    )
    stand.visual(
        Box((0.016, 0.070, 0.450)),
        origin=Origin(xyz=(0.062, 0.045, 0.260)),
        material=graphite,
        name="sleeve_wall_1",
    )
    stand.visual(
        Box((0.140, 0.014, 0.450)),
        origin=Origin(xyz=(0.0, 0.017, 0.260)),
        material=graphite,
        name="front_sleeve_wall",
    )
    stand.visual(
        Box((0.140, 0.014, 0.450)),
        origin=Origin(xyz=(0.0, 0.073, 0.260)),
        material=graphite,
        name="rear_sleeve_wall",
    )
    stand.visual(
        Box((0.110, 0.004, 0.320)),
        origin=Origin(xyz=(0.0, 0.082, 0.250)),
        material=soft_black,
        name="rear_cable_channel",
    )

    # The height-adjusting carriage is sized with hidden insertion left in the
    # sleeve at the top of travel.
    height_stage = model.part("height_stage")
    height_stage.visual(
        Box((0.090, 0.034, 0.540)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_metal,
        name="inner_spine",
    )
    height_stage.visual(
        Box((0.060, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, -0.017, 0.130)),
        material=rubber,
        name="front_guide_pad",
    )
    height_stage.visual(
        Box((0.060, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, 0.017, 0.260)),
        material=rubber,
        name="rear_guide_pad",
    )
    height_stage.visual(
        Box((0.110, 0.085, 0.060)),
        origin=Origin(xyz=(0.0, -0.0425, 0.380)),
        material=dark_metal,
        name="stand_head",
    )
    height_stage.visual(
        Box((0.025, 0.030, 0.050)),
        origin=Origin(xyz=(-0.065, -0.095, 0.372)),
        material=dark_metal,
        name="yoke_bridge_0",
    )
    height_stage.visual(
        Box((0.025, 0.030, 0.050)),
        origin=Origin(xyz=(0.065, -0.095, 0.372)),
        material=dark_metal,
        name="yoke_bridge_1",
    )
    height_stage.visual(
        Box((0.025, 0.050, 0.090)),
        origin=Origin(xyz=(-0.065, -0.130, 0.400)),
        material=dark_metal,
        name="tilt_yoke_0",
    )
    height_stage.visual(
        Box((0.025, 0.050, 0.090)),
        origin=Origin(xyz=(0.065, -0.130, 0.400)),
        material=dark_metal,
        name="tilt_yoke_1",
    )

    # Tilt carrier behind the display: an exposed horizontal barrel plus a
    # coaxial circular rotation hub for portrait/landscape switching.
    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="tilt_barrel",
    )
    tilt_bracket.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="rotation_hub",
    )

    # Thin display housing.  The monitor is offset forward from the roll/tilt
    # axis, while that axis still passes through the central viewing line.
    screen = model.part("screen")
    screen.visual(
        Box((0.740, 0.026, 0.460)),
        origin=Origin(xyz=(0.0, -0.063, 0.0)),
        material=graphite,
        name="rear_shell",
    )
    screen.visual(
        Box((0.740, 0.007, 0.030)),
        origin=Origin(xyz=(0.0, -0.0795, 0.215)),
        material=soft_black,
        name="top_bezel",
    )
    screen.visual(
        Box((0.740, 0.007, 0.030)),
        origin=Origin(xyz=(0.0, -0.0795, -0.215)),
        material=soft_black,
        name="bottom_bezel",
    )
    screen.visual(
        Box((0.030, 0.007, 0.430)),
        origin=Origin(xyz=(-0.355, -0.0795, 0.0)),
        material=soft_black,
        name="side_bezel_0",
    )
    screen.visual(
        Box((0.030, 0.007, 0.430)),
        origin=Origin(xyz=(0.355, -0.0795, 0.0)),
        material=soft_black,
        name="side_bezel_1",
    )
    screen.visual(
        Box((0.710, 0.002, 0.400)),
        origin=Origin(xyz=(0.0, -0.083, 0.0)),
        material=active_screen,
        name="glass_panel",
    )
    screen.visual(
        Cylinder(radius=0.095, length=0.018),
        origin=Origin(xyz=(0.0, -0.038, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="vesa_receiver",
    )
    screen.visual(
        Box((0.180, 0.006, 0.120)),
        origin=Origin(xyz=(0.0, -0.047, 0.0)),
        material=soft_black,
        name="rear_mount_pad",
    )

    # Small hinged rear cable cover on the stand spine.
    cable_door = model.part("cable_door")
    cable_door.visual(
        Cylinder(radius=0.006, length=0.270),
        origin=Origin(),
        material=dark_metal,
        name="door_hinge_barrel",
    )
    cable_door.visual(
        Box((0.105, 0.006, 0.270)),
        origin=Origin(xyz=(0.0525, 0.003, 0.0)),
        material=graphite,
        name="door_panel",
    )
    cable_door.visual(
        Box((0.034, 0.003, 0.044)),
        origin=Origin(xyz=(0.054, 0.0075, -0.060)),
        material=rubber,
        name="finger_pull",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0),
    )
    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=height_stage,
        origin=Origin(xyz=(0.0, 0.045, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.180),
    )
    model.articulation(
        "screen_tilt",
        ArticulationType.REVOLUTE,
        parent=height_stage,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, -0.130, 0.400)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.25, upper=0.35),
    )
    model.articulation(
        "screen_roll",
        ArticulationType.CONTINUOUS,
        parent=tilt_bracket,
        child=screen,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )
    model.articulation(
        "cable_door_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cable_door,
        origin=Origin(xyz=(-0.0525, 0.086, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.35),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    height_stage = object_model.get_part("height_stage")
    tilt_bracket = object_model.get_part("tilt_bracket")
    screen = object_model.get_part("screen")
    cable_door = object_model.get_part("cable_door")

    base_swivel = object_model.get_articulation("base_swivel")
    height_slide = object_model.get_articulation("height_slide")
    screen_tilt = object_model.get_articulation("screen_tilt")
    screen_roll = object_model.get_articulation("screen_roll")
    door_hinge = object_model.get_articulation("cable_door_hinge")

    ctx.check(
        "primary workstation mechanisms are articulated",
        len(object_model.articulations) == 5
        and base_swivel.articulation_type == ArticulationType.CONTINUOUS
        and height_slide.articulation_type == ArticulationType.PRISMATIC
        and screen_tilt.articulation_type == ArticulationType.REVOLUTE
        and screen_roll.articulation_type == ArticulationType.CONTINUOUS
        and door_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.expect_gap(
        stand,
        base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="swivel_turntable",
        max_gap=0.001,
        max_penetration=0.0,
        name="stand collar sits on turntable",
    )
    ctx.expect_within(
        height_stage,
        stand,
        axes="xy",
        inner_elem="inner_spine",
        outer_elem="rear_sleeve_wall",
        margin=0.055,
        name="height spine remains centered in sleeve width",
    )
    ctx.expect_overlap(
        height_stage,
        stand,
        axes="z",
        elem_a="inner_spine",
        elem_b="rear_sleeve_wall",
        min_overlap=0.20,
        name="height spine has deep retained insertion",
    )
    with ctx.pose({height_slide: 0.180}):
        ctx.expect_overlap(
            height_stage,
            stand,
            axes="z",
            elem_a="inner_spine",
            elem_b="rear_sleeve_wall",
            min_overlap=0.08,
            name="height spine remains inserted when raised",
        )
    with ctx.pose({screen_roll: pi / 2.0}):
        ctx.expect_origin_distance(
            screen,
            tilt_bracket,
            axes="xy",
            max_dist=0.001,
            name="portrait roll stays on central viewing axis",
        )
    with ctx.pose({door_hinge: 1.0}):
        ctx.expect_origin_distance(
            cable_door,
            stand,
            axes="xy",
            min_dist=0.09,
            name="cable cover swings rearward from spine",
        )

    return ctx.report()


object_model = build_object_model()
