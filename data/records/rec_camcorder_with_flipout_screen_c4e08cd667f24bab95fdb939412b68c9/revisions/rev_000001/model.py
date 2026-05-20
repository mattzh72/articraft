from __future__ import annotations

import math

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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_camcorder")

    matte_graphite = Material("matte_graphite", color=(0.045, 0.047, 0.050, 1.0))
    dark_plastic = Material("dark_plastic", color=(0.015, 0.016, 0.018, 1.0))
    satin_black = Material("satin_black", color=(0.005, 0.006, 0.007, 1.0))
    rubber = Material("soft_black_rubber", color=(0.010, 0.010, 0.009, 1.0))
    lens_glass = Material("blue_coated_glass", color=(0.025, 0.070, 0.105, 0.78))
    lcd_glass = Material("dark_lcd_glass", color=(0.010, 0.025, 0.045, 1.0))
    silver = Material("brushed_dark_silver", color=(0.45, 0.47, 0.48, 1.0))
    white = Material("white_print", color=(0.88, 0.88, 0.82, 1.0))

    body = model.part("body")

    # Small consumer camcorder body: about 17 cm long, 7 cm wide, 8 cm tall.
    body.visual(
        Box((0.170, 0.070, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_graphite,
        name="body_shell",
    )
    body.visual(
        Box((0.130, 0.040, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.046)),
        material=dark_plastic,
        name="top_hump",
    )
    body.visual(
        Cylinder(0.018, 0.030),
        origin=Origin(xyz=(-0.100, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_eyecup",
    )

    # Fixed front mount collar for the rotating lens barrel.
    body.visual(
        Cylinder(0.036, 0.012),
        origin=Origin(xyz=(0.091, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="front_mount",
    )
    body.visual(
        Cylinder(0.030, 0.006),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_shadow",
    )

    # The side hand strap is a single swept, flat loop anchored to the +Y side.
    strap = sweep_profile_along_spline(
        [
            (-0.066, 0.046, -0.012),
            (-0.038, 0.070, -0.002),
            (0.038, 0.070, -0.002),
            (0.066, 0.046, -0.012),
        ],
        profile=rounded_rect_profile(0.017, 0.0045, 0.0016, corner_segments=5),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    body.visual(mesh_from_geometry(strap, "side_hand_strap"), material=rubber, name="hand_strap")
    for i, x in enumerate((-0.066, 0.066)):
        body.visual(
            Box((0.020, 0.014, 0.024)),
            origin=Origin(xyz=(x, 0.039, -0.012)),
            material=dark_plastic,
            name=f"strap_anchor_{i}",
        )

    # Exposed vertical hinge knuckles mounted to the display side of the body.
    hinge_x = -0.062
    hinge_y = -0.041
    for i, z in enumerate((-0.028, 0.028)):
        body.visual(
            Box((0.014, 0.010, 0.018)),
            origin=Origin(xyz=(hinge_x, -0.0375, z)),
            material=dark_plastic,
            name=f"display_hinge_leaf_{i}",
        )
        body.visual(
            Cylinder(0.0042, 0.018),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=silver,
            name=f"display_hinge_knuckle_{i}",
        )

    lens = model.part("lens_barrel")
    lens.visual(
        Cylinder(0.027, 0.054),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="barrel_shell",
    )
    lens.visual(
        Cylinder(0.030, 0.024),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="grip_ring",
    )
    lens.visual(
        Cylinder(0.032, 0.010),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="front_rim",
    )
    lens.visual(
        Cylinder(0.023, 0.003),
        origin=Origin(xyz=(0.0565, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )
    lens.visual(
        Box((0.014, 0.004, 0.0022)),
        origin=Origin(xyz=(0.025, 0.0, 0.031)),
        material=white,
        name="lens_index",
    )

    model.articulation(
        "body_to_lens",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens,
        origin=Origin(xyz=(0.097, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    display = model.part("display_panel")
    display.visual(
        Box((0.096, 0.008, 0.060)),
        origin=Origin(xyz=(0.052, -0.008, 0.0)),
        material=dark_plastic,
        name="panel_case",
    )
    display.visual(
        Box((0.078, 0.0018, 0.044)),
        origin=Origin(xyz=(0.057, -0.0129, 0.0)),
        material=lcd_glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.010, 0.006, 0.060)),
        origin=Origin(xyz=(0.004, -0.006, 0.0)),
        material=dark_plastic,
        name="panel_hinge_leaf",
    )
    display.visual(
        Cylinder(0.0035, 0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=silver,
        name="panel_hinge_knuckle",
    )

    model.articulation(
        "body_to_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens = object_model.get_part("lens_barrel")
    display = object_model.get_part("display_panel")
    lens_joint = object_model.get_articulation("body_to_lens")
    display_joint = object_model.get_articulation("body_to_display")

    ctx.expect_gap(
        lens,
        body,
        axis="x",
        positive_elem="barrel_shell",
        negative_elem="front_mount",
        min_gap=0.000,
        max_gap=0.004,
        name="lens barrel seats just ahead of mount",
    )
    ctx.expect_overlap(
        lens,
        body,
        axes="yz",
        elem_a="barrel_shell",
        elem_b="front_mount",
        min_overlap=0.045,
        name="lens barrel is coaxial with the front mount",
    )
    ctx.expect_gap(
        body,
        display,
        axis="y",
        positive_elem="body_shell",
        negative_elem="panel_case",
        min_gap=0.006,
        max_gap=0.014,
        name="closed display panel sits outside side wall",
    )

    closed_screen_aabb = ctx.part_element_world_aabb(display, elem="screen_glass")
    with ctx.pose({display_joint: 1.45}):
        ctx.expect_gap(
            body,
            display,
            axis="y",
            positive_elem="body_shell",
            negative_elem="panel_case",
            min_gap=0.008,
            name="opened display clears the side wall at hinge edge",
        )
        opened_screen_aabb = ctx.part_element_world_aabb(display, elem="screen_glass")
    if closed_screen_aabb is not None and opened_screen_aabb is not None:
        closed_min, closed_max = closed_screen_aabb
        opened_min, opened_max = opened_screen_aabb
        closed_center_y = (closed_min[1] + closed_max[1]) * 0.5
        opened_center_y = (opened_min[1] + opened_max[1]) * 0.5
        ctx.check(
            "display moves to the left of the camcorder body",
            opened_center_y < closed_center_y - 0.035,
            details=f"closed_y={closed_center_y}, opened_y={opened_center_y}",
        )
    else:
        ctx.fail("display screen pose aabb available", "screen_glass AABB was not available")

    rest_index_aabb = ctx.part_element_world_aabb(lens, elem="lens_index")
    with ctx.pose({lens_joint: math.pi / 2.0}):
        spun_index_aabb = ctx.part_element_world_aabb(lens, elem="lens_index")
    if rest_index_aabb is not None and spun_index_aabb is not None:
        rest_min, rest_max = rest_index_aabb
        spun_min, spun_max = spun_index_aabb
        rest_center_z = (rest_min[2] + rest_max[2]) * 0.5
        spun_center_y = (spun_min[1] + spun_max[1]) * 0.5
        ctx.check(
            "lens barrel spins about viewing axis",
            rest_center_z > 0.025 and spun_center_y < -0.025,
            details=f"rest_z={rest_center_z}, spun_y={spun_center_y}",
        )
    else:
        ctx.fail("lens index pose aabb available", "lens_index AABB was not available")

    return ctx.report()


object_model = build_object_model()
