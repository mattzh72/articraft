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


def _cyl_x(length: float, radius: float) -> tuple[Cylinder]:
    """One-item tuple for a cylinder later rotated to run along local +X."""
    return (Cylinder(radius=radius, length=length),)


def _cyl_y(length: float, radius: float) -> tuple[Cylinder]:
    """One-item tuple for a cylinder later rotated to run along local +Y."""
    return (Cylinder(radius=radius, length=length),)


def _origin_xyz_rpy(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tubular_farm_gate")

    galvanized = model.material("galvanized_green", rgba=(0.30, 0.42, 0.32, 1.0))
    dark_metal = model.material("dark_hinge_metal", rgba=(0.08, 0.09, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.48, 0.45, 0.40, 1.0))
    zinc = model.material("zinc_hardware", rgba=(0.70, 0.72, 0.68, 1.0))

    tube_r = 0.025
    gate_len = 2.70
    gate_x0 = 0.12
    gate_x1 = gate_len
    gate_z0 = 0.36
    gate_z1 = 1.28

    posts = model.part("posts")
    posts.visual(
        Box((3.42, 0.16, 0.055)),
        origin=Origin(xyz=(1.46, -0.03, -0.0275)),
        material=concrete,
        name="ground_footing",
    )
    posts.visual(
        Cylinder(radius=0.060, length=1.55),
        origin=Origin(xyz=(-0.12, 0.0, 0.775)),
        material=galvanized,
        name="hinge_post",
    )
    posts.visual(
        Cylinder(radius=0.060, length=1.45),
        origin=Origin(xyz=(3.08, 0.0, 0.725)),
        material=galvanized,
        name="latch_post",
    )

    # Welded hinge-side lugs reach from the fixed post to the gate sleeve line.
    for i, z in enumerate((0.48, 0.86, 1.20)):
        posts.visual(
            Box((0.080, 0.125, 0.028)),
            origin=Origin(xyz=(-0.078, 0.0, z + 0.075)),
            material=zinc,
            name=f"upper_hinge_lug_{i}",
        )
        posts.visual(
            Box((0.080, 0.125, 0.028)),
            origin=Origin(xyz=(-0.078, 0.0, z - 0.075)),
            material=zinc,
            name=f"lower_hinge_lug_{i}",
        )
    posts.visual(
        *_cyl_y(0.13, 0.014),
        origin=_origin_xyz_rpy((3.025, 0.060, 1.05), (-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="latch_catch_pin",
    )

    gate = model.part("gate_frame")
    # Welded perimeter tube.
    gate.visual(
        *_cyl_x(gate_x1 - gate_x0 + 0.05, tube_r),
        origin=_origin_xyz_rpy(((gate_x0 + gate_x1) / 2.0, 0.0, gate_z1), (0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="top_perimeter_tube",
    )
    gate.visual(
        *_cyl_x(gate_x1 - gate_x0 + 0.05, tube_r),
        origin=_origin_xyz_rpy(((gate_x0 + gate_x1) / 2.0, 0.0, gate_z0), (0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="bottom_perimeter_tube",
    )
    gate.visual(
        Cylinder(radius=tube_r, length=gate_z1 - gate_z0 + 0.05),
        origin=Origin(xyz=(gate_x0, 0.0, (gate_z0 + gate_z1) / 2.0)),
        material=galvanized,
        name="hinge_upright_tube",
    )
    gate.visual(
        Cylinder(radius=tube_r, length=gate_z1 - gate_z0 + 0.05),
        origin=Origin(xyz=(gate_x1, 0.0, (gate_z0 + gate_z1) / 2.0)),
        material=galvanized,
        name="latch_upright_tube",
    )
    # Several straight horizontal rails typical of livestock and farm gates.
    for i, z in enumerate((0.58, 0.80, 1.02)):
        gate.visual(
            *_cyl_x(gate_x1 - gate_x0 + 0.03, 0.020),
            origin=_origin_xyz_rpy(((gate_x0 + gate_x1) / 2.0, 0.0, z), (0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"horizontal_rail_{i}",
        )
    # A diagonal anti-sag tube gives the long welded frame a farm-gate silhouette.
    diagonal_len = math.hypot(gate_x1 - gate_x0, gate_z1 - gate_z0)
    diagonal_pitch = math.atan2(gate_x1 - gate_x0, gate_z1 - gate_z0)
    gate.visual(
        Cylinder(radius=0.018, length=diagonal_len + 0.02),
        origin=Origin(
            xyz=((gate_x0 + gate_x1) / 2.0, 0.012, (gate_z0 + gate_z1) / 2.0),
            rpy=(0.0, diagonal_pitch, 0.0),
        ),
        material=galvanized,
        name="diagonal_brace",
    )
    # Three vertical hinge sleeves, welded to the gate upright by tabs.
    for i, z in enumerate((0.48, 0.86, 1.20)):
        gate.visual(
            Cylinder(radius=0.041, length=0.145),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=zinc,
            name=f"hinge_sleeve_{i}",
        )
        gate.visual(
            Box((0.075, 0.068, 0.030)),
            origin=Origin(xyz=(0.064, 0.0, z)),
            material=zinc,
            name=f"sleeve_weld_tab_{i}",
        )

    # Free-end latch bracket and caster mounting plate are welded onto the frame.
    gate.visual(
        Box((0.090, 0.040, 0.130)),
        origin=Origin(xyz=(gate_x1, 0.045, 1.05)),
        material=zinc,
        name="latch_mount_plate",
    )
    gate.visual(
        *_cyl_y(0.070, 0.010),
        origin=_origin_xyz_rpy((gate_x1, 0.095, 1.05), (-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="latch_pivot_pin",
    )
    gate.visual(
        Box((0.220, 0.150, 0.030)),
        origin=Origin(xyz=(2.61, 0.0, 0.315)),
        material=zinc,
        name="caster_mount_plate",
    )
    gate.visual(
        Box((0.050, 0.050, 0.105)),
        origin=Origin(xyz=(2.69, 0.0, 0.365)),
        material=zinc,
        name="caster_corner_web",
    )

    latch_loop = model.part("latch_loop")
    latch_loop.visual(
        Cylinder(radius=0.014, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="pivot_barrel",
    )
    latch_loop.visual(
        *_cyl_y(0.105, 0.012),
        origin=_origin_xyz_rpy((0.020, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="rear_loop_bar",
    )
    latch_loop.visual(
        *_cyl_x(0.250, 0.012),
        origin=_origin_xyz_rpy((0.155, 0.048, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="upper_loop_side",
    )
    latch_loop.visual(
        *_cyl_x(0.250, 0.012),
        origin=_origin_xyz_rpy((0.155, -0.048, 0.0), (0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="lower_loop_side",
    )
    latch_loop.visual(
        *_cyl_y(0.105, 0.012),
        origin=_origin_xyz_rpy((0.280, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="front_loop_bar",
    )

    caster_fork = model.part("caster_fork")
    caster_fork.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=dark_metal,
        name="swivel_stem",
    )
    caster_fork.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=zinc,
        name="swivel_collar",
    )
    caster_fork.visual(
        Box((0.125, 0.128, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=zinc,
        name="fork_crown",
    )
    caster_fork.visual(
        Box((0.045, 0.014, 0.155)),
        origin=Origin(xyz=(0.0, 0.046, -0.174)),
        material=zinc,
        name="fork_side_0",
    )
    caster_fork.visual(
        Box((0.045, 0.014, 0.155)),
        origin=Origin(xyz=(0.0, -0.046, -0.174)),
        material=zinc,
        name="fork_side_1",
    )
    caster_fork.visual(
        Box((0.078, 0.118, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=zinc,
        name="rear_fork_bridge",
    )
    caster_fork.visual(
        *_cyl_y(0.010, 0.019),
        origin=_origin_xyz_rpy((0.0, 0.054, -0.210), (-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="outer_axle_cap_0",
    )
    caster_fork.visual(
        *_cyl_y(0.010, 0.019),
        origin=_origin_xyz_rpy((0.0, -0.054, -0.210), (-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="outer_axle_cap_1",
    )

    wheel = model.part("support_wheel")
    wheel.visual(
        *_cyl_y(0.052, 0.086),
        origin=_origin_xyz_rpy((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_tire",
    )
    wheel.visual(
        *_cyl_y(0.066, 0.032),
        origin=_origin_xyz_rpy((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="metal_hub",
    )
    for i in range(8):
        a = i * math.tau / 8.0
        wheel.visual(
            Box((0.016, 0.058, 0.040)),
            origin=Origin(
                xyz=(0.086 * math.cos(a), 0.0, 0.086 * math.sin(a)),
                rpy=(0.0, -a, 0.0),
            ),
            material=rubber,
            name=f"tread_block_{i}",
        )

    model.articulation(
        "post_to_gate",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "gate_to_latch_loop",
        ArticulationType.REVOLUTE,
        parent=gate,
        child=latch_loop,
        origin=Origin(xyz=(gate_x1, 0.128, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-1.10, upper=0.45),
    )
    model.articulation(
        "gate_to_caster",
        ArticulationType.CONTINUOUS,
        parent=gate,
        child=caster_fork,
        origin=Origin(xyz=(2.61, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=5.0),
    )
    model.articulation(
        "caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=caster_fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.210)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    gate = object_model.get_part("gate_frame")
    latch_loop = object_model.get_part("latch_loop")
    caster = object_model.get_part("caster_fork")
    wheel = object_model.get_part("support_wheel")
    gate_hinge = object_model.get_articulation("post_to_gate")
    caster_swivel = object_model.get_articulation("gate_to_caster")
    wheel_spin = object_model.get_articulation("caster_to_wheel")
    latch_pivot = object_model.get_articulation("gate_to_latch_loop")

    ctx.allow_overlap(
        gate,
        latch_loop,
        elem_a="latch_pivot_pin",
        elem_b="pivot_barrel",
        reason="The latch loop rotates around a small pin captured inside its pivot barrel.",
    )
    ctx.expect_overlap(
        gate,
        latch_loop,
        axes="xz",
        elem_a="latch_pivot_pin",
        elem_b="pivot_barrel",
        min_overlap=0.015,
        name="latch pivot pin passes through the loop barrel",
    )
    ctx.expect_overlap(
        gate,
        posts,
        axes="z",
        elem_a="hinge_sleeve_1",
        elem_b="hinge_post",
        min_overlap=0.10,
        name="gate hinge sleeves share the vertical post axis",
    )
    ctx.expect_gap(
        wheel,
        caster,
        axis="y",
        positive_elem="metal_hub",
        negative_elem="fork_side_1",
        min_gap=0.001,
        max_gap=0.025,
        name="wheel hub is clipped between fork cheeks",
    )
    ctx.expect_within(
        wheel,
        caster,
        axes="y",
        inner_elem="metal_hub",
        outer_elem="fork_crown",
        margin=0.0,
        name="wheel axle hub remains within swivel bracket width",
    )
    ctx.expect_gap(
        wheel,
        posts,
        axis="z",
        positive_elem="rubber_tire",
        negative_elem="ground_footing",
        min_gap=0.0,
        max_gap=0.020,
        name="support wheel runs just above the ground footing",
    )

    rest_gate_pos = ctx.part_world_position(gate)
    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({gate_hinge: 1.15, caster_swivel: 0.80, wheel_spin: math.pi, latch_pivot: -0.70}):
        swung_gate_pos = ctx.part_world_position(gate)
        swung_wheel_pos = ctx.part_world_position(wheel)
        ctx.expect_origin_distance(
            caster,
            wheel,
            axes="xy",
            max_dist=0.002,
            name="support wheel stays centered under caster while swinging",
        )
        ctx.expect_gap(
            gate,
            wheel,
            axis="z",
            positive_elem="bottom_perimeter_tube",
            negative_elem="rubber_tire",
            min_gap=0.10,
            max_gap=0.22,
            name="support wheel remains under the lower gate corner",
        )

    ctx.check(
        "gate swing moves the free end and wheel together",
        rest_gate_pos is not None
        and swung_gate_pos is not None
        and rest_wheel_pos is not None
        and swung_wheel_pos is not None
        and abs(swung_wheel_pos[0] - rest_wheel_pos[0]) > 0.40
        and abs(swung_wheel_pos[1] - rest_wheel_pos[1]) > 1.00,
        details=f"gate rest={rest_gate_pos}, gate swung={swung_gate_pos}, wheel rest={rest_wheel_pos}, wheel swung={swung_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
