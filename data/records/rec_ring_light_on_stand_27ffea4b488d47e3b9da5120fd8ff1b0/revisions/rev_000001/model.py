from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    PivotForkGeometry,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TorusGeometry,
    WheelBore,
    WheelGeometry,
    WheelHub,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
    *,
    segments: int = 112,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [list(reversed(_circle_profile(inner_radius, segments)))],
        thickness,
        center=True,
    )
    # The extrusion's thickness starts on local Z. Rotate it so the ring light is
    # a vertical X-Z annulus with its illuminated face normal along Y.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _tube_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    *,
    segments: int = 72,
):
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, length)],
        [(inner_radius, 0.0), (inner_radius, length)],
        segments=segments,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="salon_ring_light_u_base_stand")

    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.022, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.90, 0.68, 1.0))

    # Shared procedural meshes.
    outer_sleeve_mesh = _tube_mesh(0.040, 0.030, 0.720, "outer_sleeve")
    clamp_collar_mesh = _tube_mesh(0.052, 0.024, 0.055, "clamp_collar")
    ring_housing_geom = TorusGeometry(0.250, 0.055, radial_segments=104, tubular_segments=18)
    ring_housing_geom.rotate_x(math.pi / 2.0)
    ring_housing_mesh = mesh_from_geometry(ring_housing_geom, "ring_housing")
    ring_diffuser_geom = TorusGeometry(0.250, 0.025, radial_segments=104, tubular_segments=12)
    ring_diffuser_geom.rotate_x(math.pi / 2.0)
    ring_diffuser_mesh = mesh_from_geometry(ring_diffuser_geom, "ring_diffuser")
    caster_fork_mesh = mesh_from_geometry(
        PivotForkGeometry(
            (0.082, 0.040, 0.078),
            gap_width=0.038,
            bore_diameter=0.014,
            bore_center_z=0.032,
            bridge_thickness=0.012,
            corner_radius=0.003,
            center=False,
        ),
        "caster_fork",
    )
    caster_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.032,
            0.024,
            hub=WheelHub(radius=0.012, width=0.024, cap_style="domed"),
            bore=WheelBore(style="round", diameter=0.012),
        ),
        "caster_rim",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.045,
            0.028,
            inner_radius=0.032,
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.003, radius=0.002),
        ),
        "caster_tire",
    )

    # Root U-base and lower telescoping sleeve.
    base = model.part("u_base")
    base.visual(
        Box((0.760, 0.078, 0.070)),
        origin=Origin(xyz=(0.0, -0.380, 0.140)),
        material=satin_black,
        name="rear_cross_tube",
    )
    for idx, x in enumerate((-0.340, 0.340)):
        base.visual(
            Box((0.078, 0.900, 0.070)),
            origin=Origin(xyz=(x, 0.020, 0.140)),
            material=satin_black,
            name=f"side_tube_{idx}",
        )
        base.visual(
            Cylinder(radius=0.039, length=0.080),
            origin=Origin(xyz=(x, 0.470, 0.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name=f"front_end_cap_{idx}",
        )
    base.visual(
        Box((0.220, 0.160, 0.035)),
        origin=Origin(xyz=(0.0, -0.380, 0.190)),
        material=satin_black,
        name="mast_plate",
    )
    base.visual(
        outer_sleeve_mesh,
        origin=Origin(xyz=(0.0, -0.380, 0.205)),
        material=brushed_steel,
        name="outer_sleeve",
    )
    base.visual(
        clamp_collar_mesh,
        origin=Origin(xyz=(0.0, -0.380, 0.895)),
        material=satin_black,
        name="clamp_collar",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.024, length=1.450),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=brushed_steel,
        name="inner_tube",
    )
    mast.visual(
        Box((0.058, 0.190, 0.050)),
        origin=Origin(xyz=(0.0, 0.075, 0.820)),
        material=satin_black,
        name="tilt_boom",
    )
    mast.visual(
        Box((0.058, 0.122, 0.050)),
        origin=Origin(xyz=(0.0, 0.231, 0.820)),
        material=satin_black,
        name="yoke_neck",
    )
    mast.visual(
        Box((0.740, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, 0.305, 0.820)),
        material=satin_black,
        name="tilt_yoke",
    )
    mast.visual(
        Box((0.045, 0.130, 0.170)),
        origin=Origin(xyz=(-0.335, 0.255, 0.820)),
        material=satin_black,
        name="yoke_cheek_0",
    )
    mast.visual(
        Box((0.045, 0.130, 0.170)),
        origin=Origin(xyz=(0.335, 0.255, 0.820)),
        material=satin_black,
        name="yoke_cheek_1",
    )

    model.articulation(
        "stand_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, -0.380, 0.925)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.25, lower=0.0, upper=0.420),
    )

    ring = model.part("ring_head")
    ring.visual(
        ring_housing_mesh,
        material=black_plastic,
        name="housing",
    )
    ring.visual(
        ring_diffuser_mesh,
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=warm_diffuser,
        name="diffuser",
    )
    ring.visual(
        Cylinder(radius=0.016, length=0.700),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tilt_pin",
    )
    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=ring,
        origin=Origin(xyz=(0.0, 0.220, 0.820)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.75, upper=0.75),
    )

    caster_positions = (
        (-0.340, -0.380),
        (0.340, -0.380),
        (-0.340, 0.440),
        (0.340, 0.440),
    )
    for idx, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{idx}")
        caster.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=brushed_steel,
            name="swivel_stem",
        )
        caster.visual(
            Cylinder(radius=0.025, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.037)),
            material=satin_black,
            name="swivel_bearing",
        )
        caster.visual(
            Box((0.076, 0.038, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, -0.041)),
            material=satin_black,
            name="fork_bridge",
        )
        caster.visual(
            Box((0.007, 0.038, 0.082)),
            origin=Origin(xyz=(-0.024, 0.0, -0.085)),
            material=satin_black,
            name="fork_plate_0",
        )
        caster.visual(
            Box((0.007, 0.038, 0.082)),
            origin=Origin(xyz=(0.024, 0.0, -0.085)),
            material=satin_black,
            name="fork_plate_1",
        )
        caster.visual(
            Cylinder(radius=0.005, length=0.072),
            origin=Origin(xyz=(0.0, 0.0, -0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name="axle",
        )

        wheel = model.part(f"caster_wheel_{idx}")
        wheel.visual(
            caster_tire_mesh,
            material=dark_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.028, length=0.032),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name="rim",
        )

        model.articulation(
            f"caster_swivel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.105)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        )
        model.articulation(
            f"wheel_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.095)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("u_base")
    mast = object_model.get_part("mast")
    ring = object_model.get_part("ring_head")
    slide = object_model.get_articulation("stand_slide")
    tilt = object_model.get_articulation("ring_tilt")

    ctx.allow_overlap(
        mast,
        base,
        elem_a="inner_tube",
        elem_b="clamp_collar",
        reason="The locking collar visibly wraps the sliding mast as a snug telescoping guide.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="sliding mast is centered inside the lower sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.12,
        name="collapsed mast remains retained in sleeve",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="clamp_collar",
        margin=0.0,
        name="mast passes through the collar bore",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="clamp_collar",
        min_overlap=0.04,
        name="collar surrounds the mast locally",
    )

    rest_position = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.420}):
        raised_position = ctx.part_world_position(mast)
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.10,
            name="raised mast remains inserted in sleeve",
        )
    ctx.check(
        "stand slide raises the mast",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.35,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    ctx.expect_within(
        ring,
        mast,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_yoke",
        margin=0.010,
        name="tilt pin stays captured by the yoke bracket",
    )
    ctx.expect_overlap(
        ring,
        mast,
        axes="x",
        elem_a="tilt_pin",
        elem_b="tilt_yoke",
        min_overlap=0.60,
        name="tilt pin spans both yoke cheeks",
    )
    ctx.allow_overlap(
        mast,
        ring,
        elem_a="yoke_neck",
        elem_b="tilt_pin",
        reason="The horizontal tilt pin is intentionally captured through the simplified yoke bore.",
    )
    ctx.expect_within(
        ring,
        mast,
        axes="yz",
        inner_elem="tilt_pin",
        outer_elem="yoke_neck",
        margin=0.0,
        name="tilt pin is centered in the yoke bore",
    )
    ctx.allow_overlap(
        mast,
        ring,
        elem_a="yoke_cheek_0",
        elem_b="tilt_pin",
        reason="The side cheek is modeled as a solid proxy for a bored tilt bracket around the pin.",
    )
    ctx.expect_within(
        ring,
        mast,
        axes="yz",
        inner_elem="tilt_pin",
        outer_elem="yoke_cheek_0",
        margin=0.0,
        name="tilt pin is centered in one side cheek",
    )
    ctx.expect_overlap(
        ring,
        mast,
        axes="x",
        elem_a="tilt_pin",
        elem_b="yoke_cheek_0",
        min_overlap=0.030,
        name="tilt pin passes through one side cheek",
    )
    ctx.allow_overlap(
        mast,
        ring,
        elem_a="yoke_cheek_1",
        elem_b="tilt_pin",
        reason="The opposite side cheek is modeled as a solid proxy for a bored tilt bracket around the pin.",
    )
    ctx.expect_within(
        ring,
        mast,
        axes="yz",
        inner_elem="tilt_pin",
        outer_elem="yoke_cheek_1",
        margin=0.0,
        name="tilt pin is centered in the opposite side cheek",
    )
    ctx.expect_overlap(
        ring,
        mast,
        axes="x",
        elem_a="tilt_pin",
        elem_b="yoke_cheek_1",
        min_overlap=0.030,
        name="tilt pin passes through the opposite side cheek",
    )

    rest_aabb = ctx.part_element_world_aabb(ring, elem="diffuser")
    with ctx.pose({tilt: 0.60}):
        tilted_aabb = ctx.part_element_world_aabb(ring, elem="diffuser")
    rest_depth = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else 0.0
    tilted_depth = tilted_aabb[1][1] - tilted_aabb[0][1] if tilted_aabb is not None else 0.0
    ctx.check(
        "ring head tilts about a horizontal axis",
        tilted_depth > rest_depth + 0.12,
        details=f"rest_depth={rest_depth}, tilted_depth={tilted_depth}",
    )

    for idx in range(4):
        caster = object_model.get_part(f"caster_{idx}")
        wheel = object_model.get_part(f"caster_wheel_{idx}")
        swivel = object_model.get_articulation(f"caster_swivel_{idx}")
        spin = object_model.get_articulation(f"wheel_spin_{idx}")
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The caster axle is intentionally captured through the wheel hub.",
        )
        ctx.expect_contact(
            caster,
            base,
            elem_a="swivel_stem",
            elem_b="side_tube_0" if idx in (0, 2) else "side_tube_1",
            contact_tol=0.002,
            name=f"caster {idx} stem reaches the U-base tube",
        )
        ctx.expect_gap(
            caster,
            wheel,
            axis="x",
            positive_elem="fork_plate_1",
            negative_elem="tire",
            min_gap=0.001,
            max_gap=0.012,
            name=f"caster {idx} wheel clears one fork cheek",
        )
        ctx.expect_gap(
            wheel,
            caster,
            axis="x",
            positive_elem="tire",
            negative_elem="fork_plate_0",
            min_gap=0.001,
            max_gap=0.012,
            name=f"caster {idx} wheel clears the opposite fork cheek",
        )
        ctx.expect_within(
            caster,
            wheel,
            axes="yz",
            inner_elem="axle",
            outer_elem="rim",
            margin=0.0,
            name=f"caster {idx} axle is centered in the hub",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="x",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.030,
            name=f"caster {idx} axle passes through the hub width",
        )
        ctx.check(
            f"caster {idx} has swivel and rolling axes",
            swivel.axis == (0.0, 0.0, 1.0) and spin.axis == (1.0, 0.0, 0.0),
            details=f"swivel_axis={swivel.axis}, spin_axis={spin.axis}",
        )

    return ctx.report()


object_model = build_object_model()
