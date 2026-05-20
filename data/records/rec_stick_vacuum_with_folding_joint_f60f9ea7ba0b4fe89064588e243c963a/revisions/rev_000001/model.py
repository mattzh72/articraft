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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_stick_vacuum")

    charcoal = Material("charcoal_plastic", color=(0.04, 0.045, 0.05, 1.0))
    dark = Material("dark_rubber", color=(0.015, 0.015, 0.016, 1.0))
    satin = Material("satin_graphite", color=(0.22, 0.23, 0.24, 1.0))
    aluminum = Material("brushed_aluminum", color=(0.68, 0.72, 0.73, 1.0))
    translucent = Material("smoky_clear_bin", color=(0.42, 0.66, 0.78, 0.45))
    accent = Material("red_release_tabs", color=(0.85, 0.08, 0.045, 1.0))
    blue = Material("blue_filter_ring", color=(0.02, 0.18, 0.65, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=Origin(xyz=(-0.14, 0.0, 0.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="motor_pod",
    )
    motor_body.visual(
        Sphere(radius=0.057),
        origin=Origin(xyz=(-0.235, 0.0, 0.02)),
        material=satin,
        name="rear_motor_cap",
    )
    motor_body.visual(
        Cylinder(radius=0.044, length=0.10),
        origin=Origin(xyz=(-0.12, 0.0, -0.047)),
        material=translucent,
        name="dust_cup",
    )
    motor_body.visual(
        Cylinder(radius=0.047, length=0.018),
        origin=Origin(xyz=(-0.12, 0.0, 0.008)),
        material=blue,
        name="filter_ring",
    )
    motor_body.visual(
        Box((0.19, 0.026, 0.026)),
        origin=Origin(xyz=(-0.145, 0.0, 0.115)),
        material=charcoal,
        name="handle_grip",
    )
    motor_body.visual(
        Box((0.028, 0.026, 0.12)),
        origin=Origin(xyz=(-0.235, 0.0, 0.065)),
        material=charcoal,
        name="rear_handle_post",
    )
    motor_body.visual(
        Box((0.028, 0.026, 0.12)),
        origin=Origin(xyz=(-0.055, 0.0, 0.065)),
        material=charcoal,
        name="front_handle_post",
    )
    motor_body.visual(
        Box((0.038, 0.090, 0.045)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=charcoal,
        name="fold_hinge_boss",
    )
    motor_body.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, 0.036, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="fold_lug_0",
    )
    motor_body.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="fold_lug_1",
    )
    motor_body.visual(
        Box((0.035, 0.012, 0.016)),
        origin=Origin(xyz=(-0.07, -0.019, 0.033)),
        material=accent,
        name="release_tab",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.024, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="fold_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.016, length=0.365),
        origin=Origin(xyz=(0.200, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="straight_tube",
    )
    wand.visual(
        Box((0.055, 0.046, 0.042)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=satin,
        name="fold_collar",
    )
    wand.visual(
        Box((0.030, 0.100, 0.032)),
        origin=Origin(xyz=(0.343, 0.0, 0.0)),
        material=satin,
        name="head_yoke_bridge",
    )
    wand.visual(
        Box((0.068, 0.020, 0.030)),
        origin=Origin(xyz=(0.372, 0.045, 0.0)),
        material=satin,
        name="head_yoke_0",
    )
    wand.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.405, 0.039, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="head_hinge_lug_0",
    )
    wand.visual(
        Box((0.068, 0.020, 0.030)),
        origin=Origin(xyz=(0.372, -0.045, 0.0)),
        material=satin,
        name="head_yoke_1",
    )
    wand.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.405, -0.039, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="head_hinge_lug_1",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.022, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="head_knuckle",
    )
    floor_head.visual(
        Box((0.038, 0.042, 0.108)),
        origin=Origin(xyz=(0.022, 0.0, -0.055)),
        material=charcoal,
        name="swivel_neck",
    )
    floor_head.visual(
        Box((0.190, 0.170, 0.045)),
        origin=Origin(xyz=(0.088, 0.0, -0.126)),
        material=charcoal,
        name="nozzle_shell",
    )
    floor_head.visual(
        Box((0.160, 0.020, 0.018)),
        origin=Origin(xyz=(0.108, 0.085, -0.122)),
        material=dark,
        name="side_bumper_0",
    )
    floor_head.visual(
        Box((0.160, 0.020, 0.018)),
        origin=Origin(xyz=(0.108, -0.085, -0.122)),
        material=dark,
        name="side_bumper_1",
    )
    floor_head.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(0.120, 0.0, -0.146), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="brush_roll",
    )
    for y, name in ((0.084, "rear_wheel_0"), (-0.084, "rear_wheel_1")):
        floor_head.visual(
            Cylinder(radius=0.020, length=0.020),
            origin=Origin(xyz=(0.006, y, -0.130), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fold = object_model.get_articulation("fold_joint")
    pitch = object_model.get_articulation("head_pitch")
    body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    head = object_model.get_part("floor_head")

    ctx.check(
        "two requested revolute joints",
        fold.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"fold={fold.articulation_type}, pitch={pitch.articulation_type}",
    )
    ctx.check(
        "hinge axes are horizontal",
        abs(fold.axis[1]) > 0.99
        and abs(pitch.axis[1]) > 0.99
        and abs(fold.axis[0]) < 0.01
        and abs(fold.axis[2]) < 0.01
        and abs(pitch.axis[0]) < 0.01
        and abs(pitch.axis[2]) < 0.01,
        details=f"fold_axis={fold.axis}, pitch_axis={pitch.axis}",
    )
    ctx.expect_contact(
        wand,
        body,
        elem_a="fold_knuckle",
        elem_b="fold_lug_0",
        name="fold knuckle contacts hinge lug",
    )
    ctx.expect_overlap(
        wand,
        body,
        axes="xz",
        elem_a="fold_knuckle",
        elem_b="fold_lug_0",
        min_overlap=0.015,
        name="fold knuckle shares hinge line",
    )
    ctx.expect_contact(
        head,
        wand,
        elem_a="head_knuckle",
        elem_b="head_hinge_lug_0",
        name="floor head contacts pitch lug",
    )
    ctx.expect_overlap(
        head,
        wand,
        axes="xz",
        elem_a="head_knuckle",
        elem_b="head_hinge_lug_0",
        min_overlap=0.014,
        name="floor head shares pitch hinge line",
    )
    ctx.expect_contact(
        head,
        wand,
        elem_a="head_knuckle",
        elem_b="head_hinge_lug_1",
        name="opposite pitch lug contacts head",
    )

    straight_tip = ctx.part_element_world_aabb(wand, elem="head_yoke_bridge")
    with ctx.pose({fold: 1.35}):
        folded_tip = ctx.part_element_world_aabb(wand, elem="head_yoke_bridge")
    ctx.check(
        "fold joint lifts wand compactly",
        straight_tip is not None
        and folded_tip is not None
        and folded_tip[0][2] > straight_tip[0][2] + 0.12,
        details=f"straight={straight_tip}, folded={folded_tip}",
    )

    level_head = ctx.part_element_world_aabb(head, elem="nozzle_shell")
    with ctx.pose({pitch: 0.45}):
        pitched_head = ctx.part_element_world_aabb(head, elem="nozzle_shell")
    ctx.check(
        "floor head pitch changes nozzle angle",
        level_head is not None
        and pitched_head is not None
        and pitched_head[0][2] < level_head[0][2] - 0.025,
        details=f"level={level_head}, pitched={pitched_head}",
    )

    return ctx.report()


object_model = build_object_model()
