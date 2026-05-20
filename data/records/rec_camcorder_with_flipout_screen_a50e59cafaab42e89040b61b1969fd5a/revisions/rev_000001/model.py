from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="palm_camcorder")

    matte_charcoal = model.material("matte_charcoal", rgba=(0.035, 0.038, 0.043, 1.0))
    satin_black = model.material("satin_black", rgba=(0.005, 0.006, 0.008, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.010, 0.011, 0.012, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.30, 0.32, 0.34, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.02, 0.05, 0.075, 0.96))
    screen = model.material("lcd_glass", rgba=(0.015, 0.030, 0.040, 1.0))
    button_mat = model.material("button_silver", rgba=(0.52, 0.54, 0.55, 1.0))

    body = model.part("body")

    body_shell = ExtrudeGeometry(
        rounded_rect_profile(0.065, 0.055, 0.013, corner_segments=10),
        0.160,
        cap=True,
        center=True,
    ).rotate_y(math.pi / 2.0)
    body.visual(
        mesh_from_geometry(body_shell, "rounded_body_shell"),
        material=matte_charcoal,
        name="rounded_body_shell",
    )

    side_wall = Box((0.100, 0.006, 0.050))
    body.visual(
        side_wall,
        origin=Origin(xyz=(-0.030, 0.0275, -0.001)),
        material=warm_gray,
        name="side_control_wall",
    )
    body.visual(
        Box((0.070, 0.0012, 0.031)),
        origin=Origin(xyz=(-0.043, 0.0341, 0.007)),
        material=satin_black,
        name="side_screen_opening",
    )
    body.visual(
        Box((0.074, 0.0020, 0.035)),
        origin=Origin(xyz=(-0.043, 0.0334, 0.007)),
        material=hinge_gray,
        name="screen_recess_lip",
    )

    grip_mesh = CapsuleGeometry(0.023, 0.090, radial_segments=32, height_segments=10)
    grip_mesh.rotate_y(math.pi / 2.0).translate(-0.020, -0.038, -0.004)
    body.visual(
        mesh_from_geometry(grip_mesh, "rounded_handgrip"),
        material=dark_rubber,
        name="rounded_handgrip",
    )
    body.visual(
        Box((0.084, 0.005, 0.012)),
        origin=Origin(xyz=(-0.020, -0.052, 0.018)),
        material=satin_black,
        name="grip_strap_band",
    )

    body.visual(
        Cylinder(radius=0.021, length=0.040),
        origin=Origin(xyz=(0.093, 0.0, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.1145, 0.0, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens_glass",
    )
    body.visual(
        Cylinder(radius=0.0255, length=0.003),
        origin=Origin(xyz=(0.0958, 0.0, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lens_ring_rear_stop",
    )
    body.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(0.079, 0.0, 0.026)),
        material=matte_charcoal,
        name="top_viewfinder_bump",
    )

    body.visual(
        Box((0.016, 0.011, 0.058)),
        origin=Origin(xyz=(-0.010, 0.0345, 0.000)),
        material=hinge_gray,
        name="monitor_hinge_block",
    )

    lens_ring = model.part("lens_ring")
    ring_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0248, -0.008),
            (0.0264, -0.006),
            (0.0264, 0.006),
            (0.0248, 0.008),
        ],
        [
            (0.0224, -0.0072),
            (0.0222, -0.004),
            (0.0222, 0.004),
            (0.0224, 0.0072),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    lens_ring.visual(
        mesh_from_geometry(ring_shell, "lens_focus_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="lens_focus_ring",
    )
    lens_ring.visual(
        Box((0.014, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=warm_gray,
        name="ring_index_rib",
    )

    monitor = model.part("side_monitor")
    monitor.visual(
        Cylinder(radius=0.003, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_gray,
        name="monitor_knuckle",
    )
    monitor.visual(
        Box((0.006, 0.008, 0.052)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=hinge_gray,
        name="monitor_hinge_leaf",
    )
    monitor.visual(
        Box((0.007, 0.072, 0.052)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=matte_charcoal,
        name="monitor_outer_frame",
    )
    monitor.visual(
        Box((0.0012, 0.058, 0.036)),
        origin=Origin(xyz=(0.0041, 0.042, 0.002)),
        material=screen,
        name="monitor_lcd_glass",
    )
    monitor.visual(
        Box((0.0020, 0.058, 0.005)),
        origin=Origin(xyz=(0.0038, 0.042, -0.019)),
        material=satin_black,
        name="monitor_lower_bezel",
    )

    button_positions = [(-0.062, 0.0), (-0.046, 0.0), (-0.030, 0.0)]
    buttons = []
    for index, (x_pos, _) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0041, length=0.0046),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0024, length=0.0030),
            origin=Origin(xyz=(0.0, -0.0038, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="button_stem",
        )
        buttons.append((button, x_pos))

    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.105, 0.0, 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=8.0),
    )

    model.articulation(
        "body_to_side_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(-0.010, 0.0430, 0.000)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=-1.57, upper=0.0),
    )

    for index, (button, x_pos) in enumerate(buttons):
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0328, -0.022)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.06, lower=0.0, upper=0.0035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lens_ring = object_model.get_part("lens_ring")
    monitor = object_model.get_part("side_monitor")
    monitor_joint = object_model.get_articulation("body_to_side_monitor")
    lens_joint = object_model.get_articulation("body_to_lens_ring")

    ctx.allow_overlap(
        body,
        lens_ring,
        elem_a="lens_ring_rear_stop",
        elem_b="lens_focus_ring",
        reason="The rotating focus ring is lightly captured by a rear retaining collar on the lens barrel.",
    )
    ctx.expect_gap(
        lens_ring,
        body,
        axis="x",
        positive_elem="lens_focus_ring",
        negative_elem="lens_ring_rear_stop",
        max_gap=0.001,
        max_penetration=0.001,
        name="lens ring is locally captured by rear stop",
    )

    ctx.expect_within(
        lens_ring,
        body,
        axes="yz",
        inner_elem="lens_focus_ring",
        outer_elem="front_lens_barrel",
        margin=0.006,
        name="lens ring is concentric with the barrel",
    )
    ctx.expect_overlap(
        lens_ring,
        body,
        axes="x",
        elem_a="lens_focus_ring",
        elem_b="front_lens_barrel",
        min_overlap=0.010,
        name="lens ring sits around the short barrel",
    )

    with ctx.pose({monitor_joint: 0.0}):
        ctx.expect_gap(
            monitor,
            body,
            axis="y",
            positive_elem="monitor_knuckle",
            negative_elem="monitor_hinge_block",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="monitor knuckle sits on the visible hinge block",
        )
        open_aabb = ctx.part_element_world_aabb(monitor, elem="monitor_outer_frame")

    with ctx.pose({monitor_joint: -1.57}):
        ctx.expect_overlap(
            monitor,
            body,
            axes="z",
            elem_a="monitor_outer_frame",
            elem_b="side_control_wall",
            min_overlap=0.030,
            name="folded monitor remains aligned with side body height",
        )
        folded_aabb = ctx.part_element_world_aabb(monitor, elem="monitor_outer_frame")

    open_y = (open_aabb[0][1] + open_aabb[1][1]) * 0.5 if open_aabb is not None else None
    folded_y = (folded_aabb[0][1] + folded_aabb[1][1]) * 0.5 if folded_aabb is not None else None
    ctx.check(
        "side monitor opens outward from the side wall",
        open_y is not None and folded_y is not None and open_y > folded_y + 0.030,
        details=f"open_y={open_y}, folded_y={folded_y}",
    )

    with ctx.pose({lens_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            lens_ring,
            body,
            axes="x",
            elem_a="lens_focus_ring",
            elem_b="front_lens_barrel",
            min_overlap=0.010,
            name="lens ring remains retained while rotating",
        )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="button_cap",
            negative_elem="side_control_wall",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"button {index} cap is seated in the body wall",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.0035}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {index} depresses independently inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
