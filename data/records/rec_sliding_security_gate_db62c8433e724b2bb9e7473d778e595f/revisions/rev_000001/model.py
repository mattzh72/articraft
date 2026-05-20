from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_security_gate")

    matte = model.material("matte_anthracite", rgba=(0.055, 0.060, 0.062, 1.0))
    satin = model.material("satin_graphite", rgba=(0.18, 0.19, 0.19, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.012, 0.012, 0.011, 1.0))
    concrete = model.material("honed_concrete", rgba=(0.34, 0.34, 0.32, 1.0))
    shadow = model.material("shadow_gap_black", rgba=(0.006, 0.006, 0.005, 1.0))

    def add_box(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material: Material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_cyl(
        part,
        name: str,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        material: Material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Root: one continuous fixed assembly tying the foundation, latch post, end
    # supports, ground channel, and overhead guide into an obvious sliding path.
    frame = model.part("track_frame")
    add_box(frame, "concrete_sill", (6.80, 0.52, 0.05), (0.80, 0.0, 0.025), concrete)
    add_box(frame, "track_spine", (6.60, 0.060, 0.060), (0.80, 0.0, 0.080), brushed)
    add_box(frame, "front_track_lip", (6.60, 0.026, 0.080), (0.80, -0.086, 0.090), satin)
    add_box(frame, "rear_track_lip", (6.60, 0.026, 0.080), (0.80, 0.086, 0.090), satin)
    add_box(frame, "drain_shadow_slot", (6.35, 0.020, 0.012), (0.88, -0.132, 0.052), shadow)

    add_box(frame, "latch_post", (0.18, 0.22, 2.20), (-2.37, 0.0, 1.10), matte)
    add_box(frame, "guide_post", (0.16, 0.20, 2.15), (2.35, 0.0, 1.075), matte)
    add_box(frame, "end_post", (0.16, 0.20, 2.15), (4.13, 0.0, 1.075), matte)
    add_box(frame, "overhead_beam", (6.55, 0.30, 0.060), (0.88, 0.0, 2.150), matte)
    add_box(frame, "overhead_front_channel", (6.45, 0.024, 0.180), (0.88, -0.105, 2.030), satin)
    add_box(frame, "overhead_rear_channel", (6.45, 0.024, 0.180), (0.88, 0.105, 2.030), satin)
    add_box(frame, "overhead_shadow_reveal", (6.30, 0.020, 0.020), (0.88, 0.0, 2.108), shadow)

    # Fixed latch keeper: the striker travels between the jaws with visible
    # clearance rather than intersecting the receiver.
    add_box(frame, "keeper_backplate", (0.040, 0.090, 0.290), (-2.270, -0.065, 1.150), brushed)
    add_box(frame, "keeper_upper_jaw", (0.130, 0.075, 0.035), (-2.245, -0.065, 1.200), brushed)
    add_box(frame, "keeper_lower_jaw", (0.130, 0.075, 0.035), (-2.245, -0.065, 1.100), brushed)
    add_box(frame, "keeper_shadow_pocket", (0.014, 0.070, 0.035), (-2.250, -0.065, 1.150), shadow)

    # Moving rigid leaf: a welded rectangular tube frame, closely-spaced pickets,
    # subtle trim strips, diagonal bracing, latch tongue, and forked roller mounts.
    leaf = model.part("leaf")
    add_box(leaf, "bottom_beam", (4.20, 0.075, 0.120), (0.0, 0.0, 0.345), matte)
    add_box(leaf, "top_beam", (4.20, 0.075, 0.100), (0.0, 0.0, 1.860), matte)
    add_box(leaf, "front_stile", (0.120, 0.075, 1.550), (-2.10, 0.0, 1.085), matte)
    add_box(leaf, "rear_stile", (0.120, 0.075, 1.550), (2.10, 0.0, 1.085), matte)
    add_box(leaf, "lower_crossbar", (4.02, 0.052, 0.060), (0.0, 0.0, 0.720), matte)
    add_box(leaf, "upper_crossbar", (4.02, 0.052, 0.060), (0.0, 0.0, 1.300), matte)

    for i, x in enumerate((-1.78, -1.38, -0.98, -0.58, -0.18, 0.22, 0.62, 1.02, 1.42, 1.82)):
        add_box(leaf, f"picket_{i}", (0.045, 0.045, 1.430), (x, 0.0, 1.090), matte)
        add_box(leaf, f"picket_cap_{i}", (0.070, 0.052, 0.035), (x, 0.0, 1.822), satin)

    brace_angle = math.atan2(1.18, 3.72)
    add_box(leaf, "diagonal_brace", (3.92, 0.056, 0.050), (0.0, 0.0, 1.075), satin, rpy=(0.0, -brace_angle, 0.0))

    add_box(leaf, "top_satin_cap", (4.05, 0.085, 0.018), (0.0, 0.0, 1.920), satin)
    add_box(leaf, "bottom_shadow_reveal", (4.00, 0.012, 0.018), (0.0, -0.043, 0.397), shadow)
    add_box(leaf, "upper_shadow_reveal", (4.00, 0.012, 0.016), (0.0, -0.043, 1.805), shadow)
    add_box(leaf, "front_trim_strip", (0.018, 0.014, 1.36), (-2.024, -0.043, 1.080), brushed)
    add_box(leaf, "rear_trim_strip", (0.018, 0.014, 1.36), (2.024, -0.043, 1.080), brushed)

    # Latch tongue aligned to the fixed keeper jaws.
    add_box(leaf, "latch_plate", (0.050, 0.025, 0.220), (-2.080, -0.050, 1.150), brushed)
    add_box(leaf, "striker", (0.120, 0.035, 0.035), (-2.160, -0.055, 1.150), brushed)
    add_cyl(leaf, "latch_pin", 0.018, 0.018, (-2.060, -0.064, 1.150), satin, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Flush bolt heads and restrained satin hardware, slightly seated into the
    # tube faces to avoid a toy-like floating appearance.
    for i, x in enumerate((-1.80, -1.05, -0.30, 0.45, 1.20, 1.85)):
        add_cyl(leaf, f"bolt_top_{i}", 0.015, 0.012, (x, -0.043, 1.862), brushed, rpy=(math.pi / 2.0, 0.0, 0.0))
        add_cyl(leaf, f"bolt_bottom_{i}", 0.015, 0.012, (x, -0.043, 0.312), brushed, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Roller forks are part of the leaf; the separate wheels revolve around the
    # same axle centers and ride visibly on the continuous track spine.
    roller_xs = (-1.35, 1.35)
    for i, x in enumerate(roller_xs):
        add_box(leaf, f"roller_fork_front_{i}", (0.180, 0.018, 0.180), (x, -0.055, 0.200), satin)
        add_box(leaf, f"roller_fork_rear_{i}", (0.180, 0.018, 0.180), (x, 0.055, 0.200), satin)
        add_box(leaf, f"roller_mount_bridge_{i}", (0.200, 0.120, 0.025), (x, 0.0, 0.300), satin)
        add_cyl(leaf, f"axle_cap_front_{i}", 0.020, 0.010, (x, -0.064, 0.195), brushed, rpy=(math.pi / 2.0, 0.0, 0.0))
        add_cyl(leaf, f"axle_cap_rear_{i}", 0.020, 0.010, (x, 0.064, 0.195), brushed, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Upper guide followers show that the leaf is captured by the overhead channel.
    for i, x in enumerate((-1.55, 1.55)):
        add_box(leaf, f"top_guide_stem_{i}", (0.050, 0.040, 0.130), (x, 0.0, 1.955), satin)
        add_cyl(leaf, f"top_guide_roller_{i}", 0.036, 0.042, (x, 0.0, 2.005), rubber)

    # Separate rolling wheels, mimicked from the leaf's prismatic travel so the
    # hardware behaves like a rolling support rather than decorative discs.
    wheel_parts = []
    for i, x in enumerate(roller_xs):
        wheel = model.part(f"roller_{i}")
        add_cyl(wheel, "tire", 0.085, 0.045, (0.0, 0.0, 0.0), rubber, rpy=(math.pi / 2.0, 0.0, 0.0))
        add_cyl(wheel, "hub", 0.043, 0.052, (0.0, 0.0, 0.0), brushed, rpy=(math.pi / 2.0, 0.0, 0.0))
        add_cyl(wheel, "bearing_collar_front", 0.030, 0.020, (0.0, -0.036, 0.0), brushed, rpy=(math.pi / 2.0, 0.0, 0.0))
        add_cyl(wheel, "bearing_collar_rear", 0.030, 0.020, (0.0, 0.036, 0.0), brushed, rpy=(math.pi / 2.0, 0.0, 0.0))
        wheel_parts.append((wheel, x))

    slide = model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=0.0, upper=1.80),
        motion_properties=MotionProperties(damping=18.0, friction=4.0),
    )

    for i, (wheel, x) in enumerate(wheel_parts):
        model.articulation(
            f"roller_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=leaf,
            child=wheel,
            origin=Origin(xyz=(x, 0.0, 0.195)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
            motion_properties=MotionProperties(damping=0.3, friction=0.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("track_frame")
    leaf = object_model.get_part("leaf")
    roller_0 = object_model.get_part("roller_0")
    roller_1 = object_model.get_part("roller_1")
    slide = object_model.get_articulation("leaf_slide")

    ctx.expect_contact(
        roller_0,
        frame,
        elem_a="tire",
        elem_b="track_spine",
        contact_tol=0.003,
        name="front roller bears on ground track",
    )
    ctx.expect_contact(
        roller_1,
        frame,
        elem_a="tire",
        elem_b="track_spine",
        contact_tol=0.003,
        name="rear roller bears on ground track",
    )
    ctx.expect_contact(
        roller_0,
        leaf,
        elem_a="bearing_collar_front",
        elem_b="roller_fork_front_0",
        contact_tol=0.002,
        name="front roller collar is captured by its fork",
    )
    ctx.expect_contact(
        roller_1,
        leaf,
        elem_a="bearing_collar_front",
        elem_b="roller_fork_front_1",
        contact_tol=0.002,
        name="rear roller collar is captured by its fork",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="x",
        elem_a="bottom_beam",
        elem_b="track_spine",
        min_overlap=4.0,
        name="closed leaf is guided over the long track",
    )

    ctx.expect_overlap(
        leaf,
        frame,
        axes="xy",
        elem_a="striker",
        elem_b="keeper_upper_jaw",
        min_overlap=0.020,
        name="striker aligns horizontally with keeper jaws",
    )
    ctx.expect_gap(
        frame,
        leaf,
        axis="z",
        positive_elem="keeper_upper_jaw",
        negative_elem="striker",
        min_gap=0.006,
        max_gap=0.030,
        name="upper keeper jaw clears the striker",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="z",
        positive_elem="striker",
        negative_elem="keeper_lower_jaw",
        min_gap=0.006,
        max_gap=0.030,
        name="lower keeper jaw clears the striker",
    )

    rest_pos = ctx.part_world_position(leaf)
    with ctx.pose({slide: 1.80}):
        ctx.expect_contact(
            roller_0,
            frame,
            elem_a="tire",
            elem_b="track_spine",
            contact_tol=0.003,
            name="opened front roller stays on track",
        )
        ctx.expect_contact(
            roller_1,
            frame,
            elem_a="tire",
            elem_b="track_spine",
            contact_tol=0.003,
            name="opened rear roller stays on track",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="x",
            elem_a="bottom_beam",
            elem_b="track_spine",
            min_overlap=4.0,
            name="opened leaf remains guided by the track",
        )
        open_pos = ctx.part_world_position(leaf)

    ctx.check(
        "leaf slides along the rail",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 1.70,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
