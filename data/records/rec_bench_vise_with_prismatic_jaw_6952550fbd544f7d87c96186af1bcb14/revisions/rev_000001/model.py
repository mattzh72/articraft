from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_x(length: float, outer_radius: float, inner_radius: float, center: tuple[float, float, float]):
    """CadQuery tube whose axis is the local X axis."""

    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_bench_vise")

    cast = model.material("old_blue_cast_iron", rgba=(0.05, 0.10, 0.16, 1.0))
    edge_wear = model.material("polished_worn_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.06, 0.06, 0.055, 1.0))
    screw_steel = model.material("oiled_lead_screw", rgba=(0.28, 0.29, 0.27, 1.0))
    bolt_dark = model.material("dark_bolt_heads", rgba=(0.035, 0.035, 0.032, 1.0))

    fixed = model.part("fixed_body")

    # Broad, bolted-down base and lower casting.  The underside is deliberately
    # thick and wide so the vise reads as a dense workshop tool.
    fixed.visual(Box((0.56, 0.31, 0.052)), origin=Origin(xyz=(0.015, 0.0, 0.026)), material=cast, name="base_plate")
    fixed.visual(Cylinder(radius=0.118, length=0.012), origin=Origin(xyz=(0.020, 0.0, 0.058)), material=cast, name="swivel_ring")
    fixed.visual(Cylinder(radius=0.092, length=0.008), origin=Origin(xyz=(0.020, 0.0, 0.065)), material=cast, name="swivel_boss")

    # Two side cheek castings leave a clear central tunnel for the lead screw.
    fixed.visual(Box((0.36, 0.044, 0.080)), origin=Origin(xyz=(0.025, -0.104, 0.088)), material=cast, name="side_cheek_0")
    fixed.visual(Box((0.36, 0.044, 0.080)), origin=Origin(xyz=(0.025, 0.104, 0.088)), material=cast, name="side_cheek_1")
    fixed.visual(Box((0.42, 0.040, 0.060)), origin=Origin(xyz=(0.010, -0.085, 0.154)), material=cast, name="guide_rail_0")
    fixed.visual(Box((0.42, 0.040, 0.060)), origin=Origin(xyz=(0.010, 0.085, 0.154)), material=cast, name="guide_rail_1")
    fixed.visual(Box((0.23, 0.028, 0.060)), origin=Origin(xyz=(0.075, -0.128, 0.116)), material=cast, name="screw_boss_web_0")
    fixed.visual(Box((0.23, 0.028, 0.060)), origin=Origin(xyz=(0.075, 0.128, 0.116)), material=cast, name="screw_boss_web_1")

    # Fixed jaw upright, rear anvil pad, and replaceable serrated jaw insert.
    fixed.visual(Box((0.092, 0.225, 0.150)), origin=Origin(xyz=(0.116, 0.0, 0.252)), material=cast, name="fixed_jaw_casting")
    fixed.visual(Box((0.118, 0.205, 0.035)), origin=Origin(xyz=(0.164, 0.0, 0.3445)), material=edge_wear, name="anvil_pad")
    fixed.visual(Box((0.016, 0.182, 0.058)), origin=Origin(xyz=(0.062, 0.0, 0.316)), material=edge_wear, name="fixed_jaw_plate")
    for i, z in enumerate((0.292, 0.303, 0.314, 0.325, 0.336)):
        fixed.visual(
            Box((0.004, 0.185, 0.004)),
            origin=Origin(xyz=(0.053, 0.0, z), rpy=(0.50, 0.0, 0.0)),
            material=dark_steel,
            name=f"fixed_tooth_{i}",
        )
    for i, z in enumerate((0.298, 0.309, 0.320, 0.331)):
        fixed.visual(
            Box((0.004, 0.185, 0.004)),
            origin=Origin(xyz=(0.052, 0.0, z), rpy=(-0.50, 0.0, 0.0)),
            material=dark_steel,
            name=f"fixed_cross_tooth_{i}",
        )

    # Hollow fixed nut and external webs.  It is a real tube, not a solid proxy,
    # so the rotating screw can pass through with visible clearance.
    fixed.visual(
        mesh_from_cadquery(_tube_x(0.082, 0.037, 0.020, (0.086, 0.0, 0.088)), "fixed_nut_sleeve"),
        material=dark_steel,
        name="fixed_nut_sleeve",
    )

    # Bench mounting bolts and swivel lock details.
    for ix, x in enumerate((-0.215, 0.245)):
        for iy, y in enumerate((-0.118, 0.118)):
            fixed.visual(Cylinder(radius=0.020, length=0.014), origin=Origin(xyz=(x, y, 0.055)), material=bolt_dark, name=f"base_bolt_{ix}_{iy}")
            fixed.visual(Cylinder(radius=0.027, length=0.004), origin=Origin(xyz=(x, y, 0.052)), material=dark_steel, name=f"base_washer_{ix}_{iy}")
    fixed.visual(Cylinder(radius=0.014, length=0.13), origin=Origin(xyz=(0.000, -0.162, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_steel, name="swivel_lock_bar")
    fixed.visual(Sphere(radius=0.023), origin=Origin(xyz=(0.000, -0.230, 0.080)), material=dark_steel, name="swivel_lock_knob")

    moving = model.part("sliding_jaw")
    # Child frame is at the closed-pose carriage reference.  Positive joint
    # travel is along -X, opening the jaw away from the fixed jaw.
    moving.visual(Box((0.094, 0.214, 0.142)), origin=Origin(xyz=(0.000, 0.0, 0.257)), material=cast, name="moving_jaw_casting")
    moving.visual(Box((0.340, 0.130, 0.045)), origin=Origin(xyz=(0.075, 0.0, 0.150)), material=cast, name="slide_tongue")
    moving.visual(Box((0.016, 0.182, 0.058)), origin=Origin(xyz=(0.053, 0.0, 0.316)), material=edge_wear, name="moving_jaw_plate")
    for i, z in enumerate((0.292, 0.303, 0.314, 0.325, 0.336)):
        moving.visual(
            Box((0.004, 0.185, 0.004)),
            origin=Origin(xyz=(0.063, 0.0, z), rpy=(0.50, 0.0, 0.0)),
            material=dark_steel,
            name=f"moving_tooth_{i}",
        )
    for i, z in enumerate((0.298, 0.309, 0.320, 0.331)):
        moving.visual(
            Box((0.004, 0.185, 0.004)),
            origin=Origin(xyz=(0.064, 0.0, z), rpy=(-0.50, 0.0, 0.0)),
            material=dark_steel,
            name=f"moving_cross_tooth_{i}",
        )

    # Front yoke plates wrap around the screw without occupying its centerline.
    moving.visual(Box((0.082, 0.026, 0.070)), origin=Origin(xyz=(-0.135, -0.044, 0.098)), material=cast, name="screw_yoke_0")
    moving.visual(Box((0.066, 0.026, 0.060)), origin=Origin(xyz=(-0.105, -0.044, 0.148)), material=cast, name="yoke_neck_0")
    moving.visual(Box((0.082, 0.026, 0.070)), origin=Origin(xyz=(-0.135, 0.044, 0.098)), material=cast, name="screw_yoke_1")
    moving.visual(Box((0.066, 0.026, 0.060)), origin=Origin(xyz=(-0.105, 0.044, 0.148)), material=cast, name="yoke_neck_1")
    moving.visual(Box((0.098, 0.115, 0.040)), origin=Origin(xyz=(-0.032, 0.0, 0.180)), material=cast, name="jaw_lower_bridge")

    slide = model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=moving,
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.12, lower=0.0, upper=0.140),
        motion_properties=MotionProperties(damping=9.0, friction=7.0),
    )

    handle = model.part("handle_screw")
    # Lead screw: a core shaft plus many proud thread crests.  The part rotates
    # about its own X axis and translates with the sliding jaw carriage.
    handle.visual(
        Cylinder(radius=0.0125, length=0.430),
        origin=Origin(xyz=(0.185, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_steel,
        name="lead_screw_core",
    )
    for i in range(28):
        x = -0.008 + i * 0.014
        handle.visual(
            Cylinder(radius=0.0157, length=0.0048),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=edge_wear if i % 2 == 0 else screw_steel,
            name=f"thread_crest_{i}",
        )
    handle.visual(
        Cylinder(radius=0.033, length=0.060),
        origin=Origin(xyz=(-0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    handle.visual(Cylinder(radius=0.011, length=0.255), origin=Origin(xyz=(-0.054, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=edge_wear, name="tommy_bar")
    handle.visual(Sphere(radius=0.024), origin=Origin(xyz=(-0.054, -0.139, 0.0)), material=edge_wear, name="bar_knob_0")
    handle.visual(Sphere(radius=0.024), origin=Origin(xyz=(-0.054, 0.139, 0.0)), material=edge_wear, name="bar_knob_1")

    model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=moving,
        child=handle,
        origin=Origin(xyz=(-0.140, 0.0, 0.088)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=8.0),
        motion_properties=MotionProperties(damping=0.2, friction=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_body")
    moving = object_model.get_part("sliding_jaw")
    handle = object_model.get_part("handle_screw")
    slide = object_model.get_articulation("jaw_slide")
    spin = object_model.get_articulation("screw_spin")

    ctx.allow_overlap(
        moving,
        handle,
        elem_a="screw_yoke_0",
        elem_b="handle_hub",
        reason="The hub is locally captured in the front yoke cheeks so the screw can pull and push the sliding jaw.",
    )
    ctx.allow_overlap(
        moving,
        handle,
        elem_a="screw_yoke_1",
        elem_b="handle_hub",
        reason="The hub is locally captured in the front yoke cheeks so the screw can pull and push the sliding jaw.",
    )
    ctx.expect_gap(
        handle,
        moving,
        axis="y",
        max_penetration=0.003,
        positive_elem="handle_hub",
        negative_elem="screw_yoke_0",
        name="lower yoke cheek lightly captures the screw hub",
    )
    ctx.expect_gap(
        moving,
        handle,
        axis="y",
        max_penetration=0.003,
        positive_elem="screw_yoke_1",
        negative_elem="handle_hub",
        name="upper yoke cheek lightly captures the screw hub",
    )

    ctx.expect_gap(
        fixed,
        moving,
        axis="x",
        min_gap=0.018,
        max_gap=0.040,
        positive_elem="fixed_jaw_plate",
        negative_elem="moving_jaw_plate",
        name="closed jaws leave a small usable gap",
    )
    ctx.expect_overlap(
        fixed,
        moving,
        axes="yz",
        min_overlap=0.050,
        elem_a="fixed_jaw_plate",
        elem_b="moving_jaw_plate",
        name="opposed serrated jaw plates line up",
    )
    ctx.expect_within(
        handle,
        fixed,
        axes="yz",
        inner_elem="lead_screw_core",
        outer_elem="fixed_nut_sleeve",
        margin=0.004,
        name="lead screw is centered in the fixed nut bore",
    )
    ctx.expect_overlap(
        handle,
        fixed,
        axes="x",
        min_overlap=0.055,
        elem_a="lead_screw_core",
        elem_b="fixed_nut_sleeve",
        name="lead screw remains engaged in the fixed nut",
    )

    rest_pos = ctx.part_world_position(moving)
    with ctx.pose({slide: 0.140}):
        ctx.expect_gap(
            fixed,
            moving,
            axis="x",
            min_gap=0.150,
            positive_elem="fixed_jaw_plate",
            negative_elem="moving_jaw_plate",
            name="opened jaw moves away from the fixed jaw",
        )
        ctx.expect_overlap(
            moving,
            fixed,
            axes="x",
            min_overlap=0.070,
            elem_a="slide_tongue",
            elem_b="guide_rail_0",
            name="slide tongue is still retained by the guide ways",
        )
        ctx.expect_overlap(
            handle,
            fixed,
            axes="x",
            min_overlap=0.030,
            elem_a="lead_screw_core",
            elem_b="fixed_nut_sleeve",
            name="extended screw remains threaded through the nut",
        )
        open_pos = ctx.part_world_position(moving)

    ctx.check(
        "jaw slide opens in the natural direction",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.12,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    before = ctx.part_world_aabb(handle)
    with ctx.pose({spin: math.pi / 2.0}):
        after = ctx.part_world_aabb(handle)
    if before is not None and after is not None:
        before_y = before[1][1] - before[0][1]
        before_z = before[1][2] - before[0][2]
        after_y = after[1][1] - after[0][1]
        after_z = after[1][2] - after[0][2]
        handle_rotates = after_z > before_z + 0.16 and after_y < before_y - 0.12
        detail = f"before_yz=({before_y:.3f}, {before_z:.3f}), after_yz=({after_y:.3f}, {after_z:.3f})"
    else:
        handle_rotates = False
        detail = f"before={before}, after={after}"
    ctx.check("tommy bar rotates about the screw axis", handle_rotates, details=detail)

    return ctx.report()


object_model = build_object_model()
