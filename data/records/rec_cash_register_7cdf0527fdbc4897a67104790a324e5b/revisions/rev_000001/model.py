from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_register")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    body_mid = model.material("body_mid", rgba=(0.28, 0.30, 0.33, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    button_grey = model.material("button_grey", rgba=(0.74, 0.75, 0.77, 1.0))
    accent_key = model.material("accent_key", rgba=(0.81, 0.71, 0.34, 1.0))
    metal = model.material("metal", rgba=(0.67, 0.69, 0.72, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.30, 0.34, 0.55))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.350, 0.340, 0.012)),
        origin=Origin(xyz=(-0.005, 0.000, 0.006)),
        material=body_dark,
        name="bottom_plate",
    )
    chassis.visual(
        Box((0.350, 0.014, 0.100)),
        origin=Origin(xyz=(-0.005, -0.163, 0.062)),
        material=body_dark,
        name="left_wall",
    )
    chassis.visual(
        Box((0.350, 0.014, 0.100)),
        origin=Origin(xyz=(-0.005, 0.163, 0.062)),
        material=body_dark,
        name="right_wall",
    )
    chassis.visual(
        Box((0.014, 0.312, 0.100)),
        origin=Origin(xyz=(-0.173, 0.000, 0.062)),
        material=body_dark,
        name="rear_wall",
    )
    chassis.visual(
        Box((0.350, 0.340, 0.012)),
        origin=Origin(xyz=(-0.005, 0.000, 0.118)),
        material=body_mid,
        name="top_deck",
    )
    chassis.visual(
        Box((0.002, 0.320, 0.020)),
        origin=Origin(xyz=(0.170, 0.000, 0.102)),
        material=body_mid,
        name="front_frame",
    )
    chassis.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(-0.010, 0.000, 0.130)),
        material=body_dark,
        name="pod_mount",
    )
    chassis.visual(
        Box((0.270, 0.008, 0.012)),
        origin=Origin(xyz=(-0.015, -0.154, 0.055)),
        material=trim_dark,
        name="left_runner",
    )
    chassis.visual(
        Box((0.270, 0.008, 0.012)),
        origin=Origin(xyz=(-0.015, 0.154, 0.055)),
        material=trim_dark,
        name="right_runner",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.305, 0.308, 0.078)),
        origin=Origin(xyz=(-0.1525, 0.000, 0.039)),
        material=body_mid,
        name="tray",
    )
    drawer.visual(
        Box((0.016, 0.316, 0.082)),
        origin=Origin(xyz=(0.008, 0.000, 0.041)),
        material=body_mid,
        name="front_face",
    )
    drawer.visual(
        Box((0.012, 0.070, 0.014)),
        origin=Origin(xyz=(0.018, 0.000, 0.041)),
        material=trim_dark,
        name="pull_bar",
    )
    drawer.visual(
        Box((0.018, 0.010, 0.014)),
        origin=Origin(xyz=(0.015, -0.024, 0.041)),
        material=trim_dark,
        name="pull_post_0",
    )
    drawer.visual(
        Box((0.018, 0.010, 0.014)),
        origin=Origin(xyz=(0.015, 0.024, 0.041)),
        material=trim_dark,
        name="pull_post_1",
    )
    drawer.visual(
        Box((0.240, 0.010, 0.010)),
        origin=Origin(xyz=(-0.185, -0.146, 0.043)),
        material=trim_dark,
        name="left_runner",
    )
    drawer.visual(
        Box((0.240, 0.010, 0.010)),
        origin=Origin(xyz=(-0.185, 0.146, 0.043)),
        material=trim_dark,
        name="right_runner",
    )

    model.articulation(
        "chassis_to_drawer",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=drawer,
        origin=Origin(xyz=(0.171, 0.000, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.30, lower=0.0, upper=0.160),
    )

    pod = model.part("pod")
    pod.visual(
        Cylinder(radius=0.020, length=0.074),
        origin=Origin(xyz=(0.000, 0.000, 0.037)),
        material=body_dark,
        name="support_neck",
    )
    pod.visual(
        Box((0.275, 0.185, 0.060)),
        origin=Origin(xyz=(-0.020, 0.000, 0.100)),
        material=body_mid,
        name="pod_shell",
    )
    pod.visual(
        Box((0.196, 0.126, 0.001)),
        origin=Origin(xyz=(-0.022, 0.000, 0.1294)),
        material=trim_dark,
        name="button_bank",
    )
    pod.visual(
        Box((0.180, 0.018, 0.014)),
        origin=Origin(xyz=(0.026, 0.000, 0.071)),
        material=body_dark,
        name="front_lip",
    )

    model.articulation(
        "chassis_to_pod",
        ArticulationType.FIXED,
        parent=chassis,
        child=pod,
        origin=Origin(xyz=(-0.010, 0.000, 0.136), rpy=(0.0, 0.38, 0.0)),
    )

    button_x = (-0.076, -0.032, 0.012)
    button_y = (-0.046, -0.015, 0.016, 0.047)
    for row, y_pos in enumerate(button_y):
        for col, x_pos in enumerate(button_x):
            button = model.part(f"button_{row}_{col}")
            button.visual(
                Box((0.032, 0.024, 0.006)),
                origin=Origin(xyz=(0.000, 0.000, 0.003)),
                material=button_grey,
                name="cap",
            )
            model.articulation(
                f"pod_to_button_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=pod,
                child=button,
                origin=Origin(xyz=(x_pos, y_pos, 0.130)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=4.0,
                    velocity=0.06,
                    lower=0.0,
                    upper=0.002,
                ),
            )

    mode_switch = model.part("mode_switch")
    mode_switch.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.000, 0.002, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="escutcheon",
    )
    mode_switch.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(xyz=(0.000, 0.009, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="shaft",
    )
    mode_switch.visual(
        Box((0.022, 0.003, 0.016)),
        origin=Origin(xyz=(0.010, 0.0185, 0.000)),
        material=accent_key,
        name="key_blade",
    )
    mode_switch.visual(
        Box((0.012, 0.004, 0.026)),
        origin=Origin(xyz=(-0.006, 0.018, 0.000)),
        material=accent_key,
        name="key_head",
    )

    model.articulation(
        "pod_to_mode_switch",
        ArticulationType.CONTINUOUS,
        parent=pod,
        child=mode_switch,
        origin=Origin(xyz=(-0.088, 0.0925, 0.103)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    screen_mount = model.part("screen_mount")
    screen_mount.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=metal,
        name="base_collar",
    )
    screen_mount.visual(
        Cylinder(radius=0.011, length=0.150),
        origin=Origin(xyz=(0.000, 0.000, 0.080)),
        material=metal,
        name="post",
    )
    screen_mount.visual(
        Box((0.018, 0.060, 0.014)),
        origin=Origin(xyz=(0.008, 0.000, 0.158)),
        material=metal,
        name="hinge_block",
    )
    screen_mount.visual(
        Box((0.012, 0.012, 0.036)),
        origin=Origin(xyz=(0.008, -0.024, 0.176)),
        material=metal,
        name="hinge_cheek_0",
    )
    screen_mount.visual(
        Box((0.012, 0.012, 0.036)),
        origin=Origin(xyz=(0.008, 0.024, 0.176)),
        material=metal,
        name="hinge_cheek_1",
    )

    model.articulation(
        "chassis_to_screen_mount",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=screen_mount,
        origin=Origin(xyz=(-0.155, 0.000, 0.124)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-1.15,
            upper=1.15,
        ),
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.026, 0.172, 0.124)),
        origin=Origin(xyz=(-0.011, 0.000, 0.062)),
        material=trim_dark,
        name="bezel",
    )
    screen.visual(
        Box((0.020, 0.160, 0.112)),
        origin=Origin(xyz=(-0.008, 0.000, 0.062)),
        material=body_mid,
        name="rear_shell",
    )
    screen.visual(
        Box((0.002, 0.146, 0.094)),
        origin=Origin(xyz=(-0.024, 0.000, 0.064)),
        material=glass,
        name="glass",
    )

    model.articulation(
        "screen_mount_to_screen",
        ArticulationType.REVOLUTE,
        parent=screen_mount,
        child=screen,
        origin=Origin(xyz=(0.000, 0.000, 0.176)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    drawer = object_model.get_part("drawer")
    pod = object_model.get_part("pod")
    mode_switch = object_model.get_part("mode_switch")
    screen = object_model.get_part("screen")

    for row in range(4):
        for col in range(3):
            ctx.allow_overlap(
                object_model.get_part(f"button_{row}_{col}"),
                pod,
                elem_a="cap",
                elem_b="pod_shell",
                reason="The sloped control pod is represented as a continuous shell without individual button apertures, so each key cap is intentionally seated into that simplified face.",
            )
    ctx.allow_overlap(
        chassis,
        pod,
        elem_a="pod_mount",
        elem_b="support_neck",
        reason="The upper control pod is intentionally seated into a short pedestal socket so the pod stays visibly separate from the drawer housing while remaining mechanically supported.",
    )

    drawer_slide = object_model.get_articulation("chassis_to_drawer")
    screen_pan = object_model.get_articulation("chassis_to_screen_mount")
    screen_tilt = object_model.get_articulation("screen_mount_to_screen")

    drawer_limits = drawer_slide.motion_limits
    if drawer_limits is not None and drawer_limits.lower is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_limits.lower}):
            ctx.expect_gap(
                drawer,
                chassis,
                axis="x",
                positive_elem="front_face",
                negative_elem="front_frame",
                max_gap=0.002,
                max_penetration=0.0,
                name="drawer front sits flush with the housing opening",
            )
            drawer_rest = ctx.part_world_position(drawer)

        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_overlap(
                drawer,
                chassis,
                axes="x",
                elem_a="left_runner",
                elem_b="left_runner",
                min_overlap=0.080,
                name="drawer runners retain insertion at full extension",
            )
            drawer_extended = ctx.part_world_position(drawer)

        ctx.check(
            "drawer extends forward",
            drawer_rest is not None
            and drawer_extended is not None
            and drawer_extended[0] > drawer_rest[0] + 0.120,
            details=f"rest={drawer_rest}, extended={drawer_extended}",
        )

    ctx.expect_gap(
        pod,
        chassis,
        axis="z",
        positive_elem="pod_shell",
        negative_elem="top_deck",
        min_gap=0.020,
        name="control pod stays visibly above the drawer housing",
    )

    ctx.expect_gap(
        mode_switch,
        pod,
        axis="y",
        positive_elem="key_blade",
        negative_elem="button_bank",
        min_gap=0.040,
        name="mode switch stays clearly separate from the button bank",
    )

    with ctx.pose({screen_pan: 0.0, screen_tilt: 0.0}):
        pan_rest_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({screen_pan: 1.10, screen_tilt: 0.0}):
        pan_swiveled_aabb = ctx.part_world_aabb(screen)

    pan_rest_center_y = None
    pan_swiveled_center_y = None
    if pan_rest_aabb is not None:
        pan_rest_center_y = (float(pan_rest_aabb[0][1]) + float(pan_rest_aabb[1][1])) / 2.0
    if pan_swiveled_aabb is not None:
        pan_swiveled_center_y = (float(pan_swiveled_aabb[0][1]) + float(pan_swiveled_aabb[1][1])) / 2.0

    ctx.check(
        "screen swivels on the vertical mount",
        pan_rest_center_y is not None
        and pan_swiveled_center_y is not None
        and abs(pan_swiveled_center_y - pan_rest_center_y) > 0.008,
        details=f"rest_center_y={pan_rest_center_y}, swiveled_center_y={pan_swiveled_center_y}",
    )

    with ctx.pose({screen_pan: 0.0, screen_tilt: 0.0}):
        tilt_rest = ctx.part_element_world_aabb(screen, elem="glass")
    with ctx.pose({screen_pan: 0.0, screen_tilt: 0.45}):
        tilt_open = ctx.part_element_world_aabb(screen, elem="glass")

    ctx.check(
        "screen tilts rearward at the head hinge",
        tilt_rest is not None
        and tilt_open is not None
        and float(tilt_open[0][0]) < float(tilt_rest[0][0]) - 0.012,
        details=f"rest={tilt_rest}, tilted={tilt_open}",
    )

    with ctx.pose(pod_to_mode_switch=0.0):
        key_rest = ctx.part_element_world_aabb(mode_switch, elem="key_blade")
    with ctx.pose(pod_to_mode_switch=math.pi / 2.0):
        key_rotated = ctx.part_element_world_aabb(mode_switch, elem="key_blade")

    ctx.check(
        "mode key rotates on its short shaft",
        key_rest is not None
        and key_rotated is not None
        and abs(float(key_rotated[0][2]) - float(key_rest[0][2])) > 0.006,
        details=f"rest={key_rest}, rotated={key_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
