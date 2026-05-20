from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_ring_light_caster_stand")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.007, 1.0))
    satin_black = model.material("satin_black", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.58, 0.60, 0.58, 1.0))
    rubber = model.material("rubber", rgba=(0.018, 0.018, 0.017, 1.0))
    diffuser = model.material("warm_diffuser", rgba=(1.0, 0.91, 0.62, 0.78))

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.072,
            0.038,
            body_style="lobed",
            grip=KnobGrip(style="ribbed", count=12, depth=0.002, width=0.004),
            crown_radius=0.002,
            center=False,
        ),
        "side_adjustment_knob",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.035,
            0.026,
            inner_radius=0.025,
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )

    base = model.part("base")
    base.visual(Cylinder(radius=0.145, length=0.090), origin=Origin(xyz=(0.0, 0.0, 0.045)), material=dark_metal, name="weighted_hub")
    base.visual(Cylinder(radius=0.084, length=0.070), origin=Origin(xyz=(0.0, 0.0, 0.125)), material=matte_black, name="lower_collar")
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        base.visual(
            Box((0.022, 0.019, 1.020)),
            origin=Origin(
                xyz=(0.047 * math.cos(theta), 0.047 * math.sin(theta), 0.610),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            material=satin_black,
            name=f"lower_sleeve_{i}",
        )
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        base.visual(
            Box((0.030, 0.030, 0.034)),
            origin=Origin(
                xyz=(0.054 * math.cos(theta), 0.054 * math.sin(theta), 1.122),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            material=brushed_metal,
            name=f"height_clamp_{i}",
        )

    caster_radius = 0.430
    for i in range(5):
        yaw = 2.0 * math.pi * i / 5.0
        c = math.cos(yaw)
        s = math.sin(yaw)
        base.visual(
            Box((0.390, 0.060, 0.045)),
            origin=Origin(xyz=(0.220 * c, 0.220 * s, 0.125), rpy=(0.0, 0.0, yaw)),
            material=dark_metal,
            name=f"base_leg_{i}",
        )
        base.visual(
            Cylinder(radius=0.049, length=0.018),
            origin=Origin(xyz=(caster_radius * c, caster_radius * s, 0.0935)),
            material=brushed_metal,
            name=f"caster_bearing_{i}",
        )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.029, length=1.110),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=brushed_metal,
        name="inner_mast",
    )
    upper_column.visual(
        Cylinder(radius=0.032, length=0.110),
        origin=Origin(xyz=(0.0, 0.080, 0.675)),
        material=matte_black,
        name="top_stem",
    )
    upper_column.visual(
        Box((0.060, 0.100, 0.036)),
        origin=Origin(xyz=(0.0, 0.035, 0.620)),
        material=matte_black,
        name="rear_stem_web",
    )
    upper_column.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=brushed_metal,
        name="mast_stop_collar",
    )
    upper_column.visual(
        Box((0.050, 0.100, 0.070)),
        origin=Origin(xyz=(-0.315, 0.0, 0.815)),
        material=matte_black,
        name="tilt_cheek_0_lower",
    )
    upper_column.visual(
        Box((0.050, 0.100, 0.090)),
        origin=Origin(xyz=(-0.315, 0.0, 0.973)),
        material=matte_black,
        name="tilt_cheek_0_upper",
    )
    upper_column.visual(
        Box((0.050, 0.100, 0.070)),
        origin=Origin(xyz=(0.315, 0.0, 0.815)),
        material=matte_black,
        name="tilt_cheek_1_lower",
    )
    upper_column.visual(
        Box((0.050, 0.100, 0.090)),
        origin=Origin(xyz=(0.315, 0.0, 0.973)),
        material=matte_black,
        name="tilt_cheek_1_upper",
    )
    upper_column.visual(
        Box((0.680, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, 0.055, 0.765)),
        material=matte_black,
        name="rear_yoke_bridge",
    )
    upper_column.visual(
        Box((0.090, 0.040, 0.250)),
        origin=Origin(xyz=(0.0, 0.055, 0.855)),
        material=matte_black,
        name="rear_yoke_spine",
    )
    upper_column.visual(
        Box((0.050, 0.020, 0.250)),
        origin=Origin(xyz=(-0.315, 0.047, 0.895)),
        material=matte_black,
        name="tilt_cheek_0_rear",
    )
    upper_column.visual(
        Box((0.050, 0.020, 0.250)),
        origin=Origin(xyz=(0.315, 0.047, 0.895)),
        material=matte_black,
        name="tilt_cheek_1_rear",
    )

    ring_head = model.part("ring_head")
    for i in range(32):
        theta = 2.0 * math.pi * i / 32.0
        phi = math.pi / 2.0 - theta
        cx = 0.203 * math.cos(theta)
        cz = 0.203 * math.sin(theta)
        ring_head.visual(
            Box((0.045, 0.050, 0.108)),
            origin=Origin(xyz=(cx, 0.0, cz), rpy=(0.0, phi, 0.0)),
            material=matte_black,
            name=f"housing_segment_{i}",
        )
        ring_head.visual(
            Box((0.038, 0.009, 0.072)),
            origin=Origin(xyz=(cx, -0.025, cz), rpy=(0.0, phi, 0.0)),
            material=diffuser,
            name=f"diffuser_segment_{i}",
        )
    for side, x in enumerate((-0.286, 0.286)):
        ring_head.visual(
            Cylinder(radius=0.014, length=0.088),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"tilt_pin_{side}",
        )
    ring_head.visual(
        Box((0.030, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, -0.040, -0.205)),
        material=satin_black,
        name="lower_mount_pad",
    )

    adjustment_knob = model.part("adjustment_knob")
    adjustment_knob.visual(knob_mesh, material=matte_black, name="knob_cap")
    adjustment_knob.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=brushed_metal,
        name="clamp_washer",
    )
    adjustment_knob.visual(
        Cylinder(radius=0.007, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=brushed_metal,
        name="clamp_screw",
    )

    caster_parts = []
    wheel_parts = []
    for i in range(5):
        caster = model.part(f"caster_{i}")
        caster_parts.append(caster)
        caster.visual(Cylinder(radius=0.026, length=0.018), origin=Origin(xyz=(0.0, 0.0, -0.009)), material=brushed_metal, name="swivel_puck")
        caster.visual(Cylinder(radius=0.009, length=0.040), origin=Origin(xyz=(0.0, 0.0, -0.020)), material=dark_metal, name="trailing_stem")
        caster.visual(Box((0.006, 0.084, 0.066)), origin=Origin(xyz=(-0.023, -0.047, -0.049)), material=dark_metal, name="fork_plate_0")
        caster.visual(Box((0.006, 0.084, 0.066)), origin=Origin(xyz=(0.023, -0.047, -0.049)), material=dark_metal, name="fork_plate_1")
        caster.visual(Box((0.052, 0.014, 0.012)), origin=Origin(xyz=(0.0, 0.0, -0.010)), material=dark_metal, name="fork_bridge")

        wheel = model.part(f"wheel_{i}")
        wheel_parts.append(wheel)
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.026, length=0.024),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name="wheel_hub",
        )
        wheel.visual(
            Cylinder(radius=0.0045, length=0.040),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name="axle_stub",
        )

    model.articulation(
        "base_to_upper_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 1.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.22, lower=0.0, upper=0.360),
    )
    model.articulation(
        "upper_column_to_ring_head",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.0, 0.895)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "upper_column_to_adjustment_knob",
        ArticulationType.CONTINUOUS,
        parent=upper_column,
        child=adjustment_knob,
        origin=Origin(xyz=(0.345, 0.047, 0.895), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    for i, caster in enumerate(caster_parts):
        yaw = 2.0 * math.pi * i / 5.0
        c = math.cos(yaw)
        s = math.sin(yaw)
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(caster_radius * c, caster_radius * s, 0.0845)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=7.0),
        )
        model.articulation(
            f"caster_{i}_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel_parts[i],
            origin=Origin(xyz=(0.0, -0.047, -0.0495)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper = object_model.get_part("upper_column")
    ring = object_model.get_part("ring_head")
    knob = object_model.get_part("adjustment_knob")
    caster = object_model.get_part("caster_0")
    wheel = object_model.get_part("wheel_0")

    slide = object_model.get_articulation("base_to_upper_column")
    tilt = object_model.get_articulation("upper_column_to_ring_head")
    knob_spin = object_model.get_articulation("upper_column_to_adjustment_knob")
    caster_swivel = object_model.get_articulation("base_to_caster_0")
    wheel_spin = object_model.get_articulation("caster_0_to_wheel_0")

    ctx.allow_overlap(
        knob,
        upper,
        elem_a="clamp_screw",
        elem_b="tilt_cheek_1_rear",
        reason="The adjustment knob screw is intentionally seated into the tilt bracket cheek to clamp the ring tilt axis.",
    )

    ctx.check(
        "primary articulations are present",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and caster_swivel.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="stand slide, ring tilt, knob spin, caster swivel, and wheel spin joints should be authored",
    )

    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({slide: 0.360}):
        raised_upper = ctx.part_world_position(upper)
        ctx.expect_overlap(
            upper,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="lower_sleeve_0",
            min_overlap=0.15,
            name="raised mast remains inserted in sleeve",
        )
    ctx.check(
        "height column slides upward",
        rest_upper is not None
        and raised_upper is not None
        and raised_upper[2] > rest_upper[2] + 0.34,
        details=f"rest={rest_upper}, raised={raised_upper}",
    )

    rest_ring_aabb = ctx.part_world_aabb(ring)
    with ctx.pose({tilt: 0.65}):
        tilted_ring_aabb = ctx.part_world_aabb(ring)
    if rest_ring_aabb is not None and tilted_ring_aabb is not None:
        rest_y = rest_ring_aabb[1][1] - rest_ring_aabb[0][1]
        tilted_y = tilted_ring_aabb[1][1] - tilted_ring_aabb[0][1]
    else:
        rest_y = tilted_y = None
    ctx.check(
        "ring head visibly tilts on horizontal axis",
        rest_y is not None and tilted_y is not None and tilted_y > rest_y + 0.20,
        details=f"rest_y_extent={rest_y}, tilted_y_extent={tilted_y}",
    )

    ctx.expect_overlap(
        ring,
        upper,
        axes="xz",
        elem_a="tilt_pin_0",
        elem_b="tilt_cheek_0_rear",
        min_overlap=0.02,
        name="first trunnion pin is captured by yoke",
    )
    ctx.expect_overlap(
        ring,
        upper,
        axes="xz",
        elem_a="tilt_pin_1",
        elem_b="tilt_cheek_1_rear",
        min_overlap=0.02,
        name="second trunnion pin is captured by yoke",
    )
    ctx.expect_overlap(
        knob,
        upper,
        axes="yz",
        elem_a="clamp_screw",
        elem_b="tilt_cheek_1_rear",
        min_overlap=0.010,
        name="side knob screw is coaxial with tilt bracket",
    )

    ctx.expect_contact(
        wheel,
        caster,
        elem_a="axle_stub",
        elem_b="fork_plate_0",
        contact_tol=0.001,
        name="caster wheel axle seats in first fork plate",
    )
    ctx.expect_contact(
        wheel,
        caster,
        elem_a="axle_stub",
        elem_b="fork_plate_1",
        contact_tol=0.001,
        name="caster wheel axle seats in second fork plate",
    )

    rest_wheel = ctx.part_world_position(wheel)
    with ctx.pose({caster_swivel: 1.0}):
        swiveled_wheel = ctx.part_world_position(wheel)
    if rest_wheel is not None and swiveled_wheel is not None:
        swivel_travel = math.hypot(swiveled_wheel[0] - rest_wheel[0], swiveled_wheel[1] - rest_wheel[1])
    else:
        swivel_travel = None
    ctx.check(
        "caster swivel swings trailing wheel around vertical mount",
        swivel_travel is not None and swivel_travel > 0.035,
        details=f"wheel center travel={swivel_travel}",
    )

    return ctx.report()


object_model = build_object_model()
