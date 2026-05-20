from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    """Hollow cylinder mesh centered on local X, used for guide/nut bushings."""
    tube = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_woodworking_vise")

    cast_iron = Material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    worn_edge = Material("worn_cast_edges", rgba=(0.16, 0.16, 0.15, 1.0))
    oiled_steel = Material("oiled_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    dark_steel = Material("darkened_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    bench_wood = Material("aged_bench_wood", rgba=(0.50, 0.32, 0.17, 1.0))
    jaw_wood = Material("maple_jaw_liners", rgba=(0.70, 0.50, 0.27, 1.0))

    guide_sleeve_mesh = _tube_mesh(
        "fixed_guide_sleeve", outer_radius=0.042, inner_radius=0.024, length=0.12
    )
    screw_nut_mesh = _tube_mesh(
        "fixed_screw_nut", outer_radius=0.047, inner_radius=0.027, length=0.13
    )
    sliding_screw_boss_mesh = _tube_mesh(
        "sliding_screw_boss", outer_radius=0.047, inner_radius=0.0138, length=0.080
    )
    bench_mount = model.part("bench_mount")

    # Bench edge and apron that the fixed vise casting is bolted to.
    bench_mount.visual(
        Box((0.70, 0.55, 0.065)),
        origin=Origin(xyz=(-0.285, 0.0, 0.795)),
        material=bench_wood,
        name="bench_top",
    )
    bench_mount.visual(
        Box((0.085, 0.55, 0.13)),
        origin=Origin(xyz=(-0.030, 0.0, 0.490)),
        material=bench_wood,
        name="lower_apron",
    )
    for index, y in enumerate((-0.250, 0.250)):
        bench_mount.visual(
            Box((0.085, 0.055, 0.36)),
            origin=Origin(xyz=(-0.030, y, 0.600)),
            material=bench_wood,
            name=f"side_apron_{index}",
        )

    # Fixed jaw casting, guide-bushing yoke, and bolted back plate.
    bench_mount.visual(
        Box((0.070, 0.405, 0.205)),
        origin=Origin(xyz=(0.020, 0.0, 0.835)),
        material=cast_iron,
        name="fixed_backplate",
    )
    bench_mount.visual(
        Box((0.025, 0.350, 0.155)),
        origin=Origin(xyz=(0.0675, 0.0, 0.850)),
        material=jaw_wood,
        name="fixed_pad",
    )
    bench_mount.visual(
        Box((0.078, 0.370, 0.038)),
        origin=Origin(xyz=(0.000, 0.0, 0.565)),
        material=cast_iron,
        name="lower_yoke",
    )
    for index, y in enumerate((-0.115, 0.115)):
        bench_mount.visual(
            guide_sleeve_mesh,
            origin=Origin(xyz=(0.0, y, 0.620)),
            material=cast_iron,
            name=f"guide_sleeve_{index}",
        )
        bench_mount.visual(
            Box((0.070, 0.075, 0.135)),
            origin=Origin(xyz=(0.010, y, 0.7225)),
            material=cast_iron,
            name=f"sleeve_web_{index}",
        )

    bench_mount.visual(
        screw_nut_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=cast_iron,
        name="screw_nut",
    )
    bench_mount.visual(
        Box((0.082, 0.080, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, 0.770)),
        material=cast_iron,
        name="nut_web",
    )

    # Bolt heads through the fixed casting into the bench apron.
    for index, (y, z) in enumerate(((-0.160, 0.895), (0.160, 0.895), (-0.160, 0.772), (0.160, 0.772))):
        bench_mount.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.084, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"bolt_head_{index}",
        )

    # Subtle horizontal grain/groove lines on the fixed wooden jaw liner.
    for index, z in enumerate((0.812, 0.850, 0.888)):
        bench_mount.visual(
            Box((0.004, 0.330, 0.005)),
            origin=Origin(xyz=(0.082, 0.0, z)),
            material=bench_wood,
            name=f"fixed_grain_{index}",
        )

    sliding_jaw = model.part("sliding_jaw")
    sliding_jaw.visual(
        Box((0.078, 0.395, 0.205)),
        origin=Origin(xyz=(0.164, 0.0, 0.835)),
        material=cast_iron,
        name="sliding_casting",
    )
    sliding_jaw.visual(
        Box((0.025, 0.350, 0.155)),
        origin=Origin(xyz=(0.1175, 0.0, 0.850)),
        material=jaw_wood,
        name="sliding_pad",
    )
    sliding_jaw.visual(
        Box((0.060, 0.310, 0.030)),
        origin=Origin(xyz=(0.156, 0.0, 0.745)),
        material=cast_iron,
        name="jaw_throat_rib",
    )

    for index, y in enumerate((-0.115, 0.115)):
        sliding_jaw.visual(
            Cylinder(radius=0.039, length=0.085),
            origin=Origin(xyz=(0.158, y, 0.620), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cast_iron,
            name=f"rod_boss_{index}",
        )
        sliding_jaw.visual(
            Box((0.072, 0.072, 0.110)),
            origin=Origin(xyz=(0.158, y, 0.700)),
            material=cast_iron,
            name=f"rod_lug_{index}",
        )

    sliding_jaw.visual(
        sliding_screw_boss_mesh,
        origin=Origin(xyz=(0.158, 0.0, 0.700)),
        material=cast_iron,
        name="screw_boss",
    )

    for index, z in enumerate((0.812, 0.850, 0.888)):
        sliding_jaw.visual(
            Box((0.004, 0.330, 0.005)),
            origin=Origin(xyz=(0.104, 0.0, z)),
            material=bench_wood,
            name=f"sliding_grain_{index}",
        )

    guide_rod_0 = model.part("guide_rod_0")
    guide_rod_0.visual(
        Cylinder(radius=0.0244, length=0.330),
        origin=Origin(xyz=(-0.035, -0.115, 0.620), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oiled_steel,
        name="rod",
    )
    guide_rod_1 = model.part("guide_rod_1")
    guide_rod_1.visual(
        Cylinder(radius=0.0244, length=0.330),
        origin=Origin(xyz=(-0.035, 0.115, 0.620), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oiled_steel,
        name="rod",
    )

    screw_spindle = model.part("screw_spindle")
    screw_spindle.visual(
        Cylinder(radius=0.014, length=0.580),
        origin=Origin(xyz=(0.110, 0.0, 0.700), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oiled_steel,
        name="spindle",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.330),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=oiled_steel,
        name="tommy_bar",
    )
    for index, y in enumerate((-0.175, 0.175)):
        handle.visual(
            Sphere(radius=0.022),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_steel,
            name=f"bar_knob_{index}",
        )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=bench_mount,
        child=sliding_jaw,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.08, lower=0.0, upper=0.20),
    )
    model.articulation(
        "jaw_to_guide_0",
        ArticulationType.FIXED,
        parent=sliding_jaw,
        child=guide_rod_0,
        origin=Origin(),
    )
    model.articulation(
        "jaw_to_guide_1",
        ArticulationType.FIXED,
        parent=sliding_jaw,
        child=guide_rod_1,
        origin=Origin(),
    )
    model.articulation(
        "jaw_to_screw",
        ArticulationType.FIXED,
        parent=sliding_jaw,
        child=screw_spindle,
        origin=Origin(),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=screw_spindle,
        child=handle,
        origin=Origin(xyz=(0.430, 0.0, 0.700)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bench_mount = object_model.get_part("bench_mount")
    sliding_jaw = object_model.get_part("sliding_jaw")
    guide_rod_0 = object_model.get_part("guide_rod_0")
    guide_rod_1 = object_model.get_part("guide_rod_1")
    screw_spindle = object_model.get_part("screw_spindle")
    handle = object_model.get_part("handle")
    jaw_slide = object_model.get_articulation("jaw_slide")
    handle_spin = object_model.get_articulation("handle_spin")

    ctx.check(
        "sliding jaw uses prismatic guide motion",
        jaw_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(jaw_slide.axis) == (1.0, 0.0, 0.0)
        and jaw_slide.motion_limits is not None
        and abs(jaw_slide.motion_limits.upper - 0.20) < 1e-6,
        details=f"type={jaw_slide.articulation_type}, axis={jaw_slide.axis}, limits={jaw_slide.motion_limits}",
    )
    ctx.check(
        "tommy bar rotates about the screw axis",
        handle_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(handle_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={handle_spin.articulation_type}, axis={handle_spin.axis}",
    )

    with ctx.pose({jaw_slide: 0.0}):
        for rod_part, sleeve_name in (
            (guide_rod_0, "guide_sleeve_0"),
            (guide_rod_1, "guide_sleeve_1"),
        ):
            ctx.allow_overlap(
                bench_mount,
                rod_part,
                elem_a=sleeve_name,
                elem_b="rod",
                reason="Each steel guide rod is captured in its fixed cast-iron sleeve with a tiny bearing fit.",
            )
        for rod_part, boss_name in (
            (guide_rod_0, "rod_boss_0"),
            (guide_rod_1, "rod_boss_1"),
        ):
            ctx.allow_overlap(
                sliding_jaw,
                rod_part,
                elem_a=boss_name,
                elem_b="rod",
                reason="Each guide rod is pressed into the moving jaw boss as a fixed retained member.",
            )
        ctx.allow_overlap(
            bench_mount,
            screw_spindle,
            elem_a="screw_nut",
            elem_b="spindle",
            reason="The spindle is intentionally seated through the fixed threaded nut proxy.",
        )
        ctx.allow_overlap(
            sliding_jaw,
            screw_spindle,
            elem_a="screw_boss",
            elem_b="spindle",
            reason="The screw spindle is captured through the moving jaw boss with a tight bearing fit.",
        )
        ctx.expect_gap(
            sliding_jaw,
            bench_mount,
            axis="x",
            positive_elem="sliding_pad",
            negative_elem="fixed_pad",
            min_gap=0.015,
            max_gap=0.035,
            name="closed pads have a small clamping gap",
        )
        for index, rod_part in enumerate((guide_rod_0, guide_rod_1)):
            ctx.expect_within(
                rod_part,
                bench_mount,
                axes="yz",
                inner_elem="rod",
                outer_elem=f"guide_sleeve_{index}",
                margin=0.0,
                name=f"guide rod {index} is centered in its sleeve",
            )
            ctx.expect_overlap(
                rod_part,
                bench_mount,
                axes="x",
                elem_a="rod",
                elem_b=f"guide_sleeve_{index}",
                min_overlap=0.10,
                name=f"guide rod {index} is deeply retained when closed",
            )
            ctx.expect_overlap(
                rod_part,
                sliding_jaw,
                axes="x",
                elem_a="rod",
                elem_b=f"rod_boss_{index}",
                min_overlap=0.010,
                name=f"guide rod {index} is retained in the moving jaw boss",
            )
        ctx.expect_overlap(
            screw_spindle,
            bench_mount,
            axes="x",
            elem_a="spindle",
            elem_b="screw_nut",
            min_overlap=0.10,
            name="screw spindle remains in the fixed nut when closed",
        )
        ctx.expect_overlap(
            screw_spindle,
            sliding_jaw,
            axes="x",
            elem_a="spindle",
            elem_b="screw_boss",
            min_overlap=0.060,
            name="screw spindle is carried by the moving jaw boss",
        )

    rest_pos = ctx.part_world_position(sliding_jaw)
    with ctx.pose({jaw_slide: 0.20, handle_spin: math.pi / 2.0}):
        ctx.expect_gap(
            sliding_jaw,
            bench_mount,
            axis="x",
            positive_elem="sliding_pad",
            negative_elem="fixed_pad",
            min_gap=0.20,
            name="opened pads separate along the bench-front axis",
        )
        for index, rod_part in enumerate((guide_rod_0, guide_rod_1)):
            ctx.expect_within(
                rod_part,
                bench_mount,
                axes="yz",
                inner_elem="rod",
                outer_elem=f"guide_sleeve_{index}",
                margin=0.0,
                name=f"extended guide rod {index} stays aligned in its sleeve",
            )
            ctx.expect_overlap(
                rod_part,
                bench_mount,
                axes="x",
                elem_a="rod",
                elem_b=f"guide_sleeve_{index}",
                min_overlap=0.035,
                name=f"extended guide rod {index} retains insertion",
            )
        ctx.expect_contact(
            handle,
            screw_spindle,
            elem_a="handle_hub",
            elem_b="spindle",
            contact_tol=0.002,
            name="rotating handle hub seats on the screw nose",
        )
        extended_pos = ctx.part_world_position(sliding_jaw)

    ctx.check(
        "positive prismatic travel moves the sliding jaw outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.18,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
