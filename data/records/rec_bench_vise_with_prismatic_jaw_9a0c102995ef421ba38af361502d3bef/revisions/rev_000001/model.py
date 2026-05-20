from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _x_cylinder(
    radius: float,
    length: float,
    *,
    center_x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    """CadQuery cylinder whose axis is the model's horizontal X screw/rod axis."""
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length)
        .translate((center_x - 0.5 * length, 0.0, 0.0))
    )


def _annular_x_bushing(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    center_x: float,
    y: float,
    z: float,
) -> cq.Workplane:
    outer = _x_cylinder(outer_radius, length, center_x=center_x, y=y, z=z)
    inner = _x_cylinder(inner_radius, length + 0.006, center_x=center_x, y=y, z=z)
    return outer.cut(inner)


def _box_with_x_holes(
    *,
    x_min: float,
    x_max: float,
    y_size: float,
    z_size: float,
    holes: tuple[tuple[float, float, float], ...],
) -> cq.Workplane:
    """Box in global axes, with circular through-holes bored along X."""
    x_size = x_max - x_min
    center_x = 0.5 * (x_min + x_max)
    body = cq.Workplane("XY").box(x_size, y_size, z_size).translate((center_x, 0.0, 0.0))
    for y, z, radius in holes:
        cutter = _x_cylinder(radius, x_size + 0.030, center_x=center_x, y=y, z=z)
        body = body.cut(cutter)
    return body


def _threaded_screw_mesh() -> cq.Workplane:
    """A ridged lead-screw: a slim shaft with repeated crest bands."""
    shaft = _x_cylinder(0.0165, 0.440, center_x=-0.080)
    pitch = 0.010
    for i in range(34):
        x = -0.245 + i * pitch
        crest = _x_cylinder(0.0205, 0.0045, center_x=x)
        shaft = shaft.union(crest)
    return shaft


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woodworking_front_vise")

    jaw_wood = model.material("oiled_maple", rgba=(0.72, 0.48, 0.25, 1.0))
    bench_wood = model.material("dark_bench_wood", rgba=(0.38, 0.22, 0.11, 1.0))
    end_grain = model.material("end_grain", rgba=(0.56, 0.34, 0.17, 1.0))
    steel = model.material("brushed_steel", rgba=(0.48, 0.50, 0.50, 1.0))
    dark_steel = model.material("darkened_thread", rgba=(0.18, 0.19, 0.19, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.67, 0.39, 0.18, 1.0))

    screw_hole = (0.0, 0.0, 0.029)
    guide_holes = ((-0.225, -0.055, 0.021), (0.225, -0.055, 0.021))
    all_holes = (screw_hole, *guide_holes)

    bench_front = model.part("bench_front")
    bench_front.visual(
        mesh_from_cadquery(
            _box_with_x_holes(
                x_min=-0.122,
                x_max=-0.052,
                y_size=0.800,
                z_size=0.320,
                holes=all_holes,
            ),
            "bench_apron",
        ),
        material=bench_wood,
        name="bench_apron",
    )
    bench_front.visual(
        Box((0.320, 0.820, 0.050)),
        origin=Origin(xyz=(-0.175, 0.0, 0.185)),
        material=bench_wood,
        name="bench_top",
    )
    bench_front.visual(
        mesh_from_cadquery(
            _box_with_x_holes(
                x_min=-0.055,
                x_max=0.0,
                y_size=0.680,
                z_size=0.180,
                holes=all_holes,
            ),
            "fixed_jaw",
        ),
        material=jaw_wood,
        name="fixed_jaw",
    )
    bench_front.visual(
        Box((0.006, 0.680, 0.020)),
        origin=Origin(xyz=(0.003, 0.0, 0.080)),
        material=end_grain,
        name="fixed_top_edge",
    )

    bench_front.visual(
        mesh_from_cadquery(
            _annular_x_bushing(
                outer_radius=0.047,
                inner_radius=0.026,
                length=0.026,
                center_x=0.013,
                y=0.0,
                z=0.0,
            ),
            "screw_nut",
        ),
        material=steel,
        name="screw_nut",
    )
    bench_front.visual(
        mesh_from_cadquery(
            _annular_x_bushing(
                outer_radius=0.032,
                inner_radius=0.0135,
                length=0.022,
                center_x=0.011,
                y=-0.225,
                z=-0.055,
            ),
            "guide_bushing_0",
        ),
        material=steel,
        name="guide_bushing_0",
    )
    bench_front.visual(
        mesh_from_cadquery(
            _annular_x_bushing(
                outer_radius=0.032,
                inner_radius=0.0135,
                length=0.022,
                center_x=0.011,
                y=0.225,
                z=-0.055,
            ),
            "guide_bushing_1",
        ),
        material=steel,
        name="guide_bushing_1",
    )

    bolt_locations = (
        (-0.285, 0.055),
        (0.285, 0.055),
        (-0.285, -0.055),
        (0.285, -0.055),
    )
    for idx, (y, z) in enumerate(bolt_locations):
        bench_front.visual(
            Cylinder(radius=0.018, length=0.009),
            origin=Origin(xyz=(0.0045, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"bolt_head_{idx}",
        )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        mesh_from_cadquery(
            _box_with_x_holes(
                x_min=0.0,
                x_max=0.072,
                y_size=0.660,
                z_size=0.205,
                holes=((0.0, 0.0, 0.028),),
            ),
            "moving_jaw_face",
        ),
        material=jaw_wood,
        name="jaw_face",
    )
    moving_jaw.visual(
        Box((0.008, 0.660, 0.018)),
        origin=Origin(xyz=(0.076, 0.0, 0.090)),
        material=end_grain,
        name="moving_top_edge",
    )
    moving_jaw.visual(
        mesh_from_cadquery(
            _annular_x_bushing(
                outer_radius=0.038,
                inner_radius=0.024,
                length=0.022,
                center_x=0.083,
                y=0.0,
                z=0.0,
            ),
            "moving_screw_collar",
        ),
        material=steel,
        name="screw_collar",
    )

    moving_jaw.visual(
        Cylinder(radius=0.014, length=0.380),
        origin=Origin(xyz=(-0.155, -0.225, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="guide_rod_0",
    )
    moving_jaw.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.015, -0.225, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rod_socket_0",
    )
    moving_jaw.visual(
        Cylinder(radius=0.014, length=0.380),
        origin=Origin(xyz=(-0.155, 0.225, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="guide_rod_1",
    )
    moving_jaw.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.015, 0.225, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rod_socket_1",
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=bench_front,
        child=moving_jaw,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=750.0, velocity=0.18, lower=0.0, upper=0.180),
    )

    lead_screw = model.part("lead_screw")
    lead_screw.visual(
        mesh_from_cadquery(_threaded_screw_mesh(), "threaded_screw"),
        material=dark_steel,
        name="threaded_shaft",
    )
    lead_screw.visual(
        Cylinder(radius=0.032, length=0.064),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handle_hub",
    )
    lead_screw.visual(
        Cylinder(radius=0.017, length=0.390),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_wood,
        name="round_handle",
    )
    for idx, y in enumerate((-0.210, 0.210)):
        lead_screw.visual(
            Sphere(radius=0.033),
            origin=Origin(xyz=(0.145, y, 0.0)),
            material=handle_wood,
            name=f"handle_knob_{idx}",
        )

    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=lead_screw,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bench_front = object_model.get_part("bench_front")
    moving_jaw = object_model.get_part("moving_jaw")
    lead_screw = object_model.get_part("lead_screw")
    slide = object_model.get_articulation("jaw_slide")
    spin = object_model.get_articulation("handle_spin")

    ctx.allow_overlap(
        bench_front,
        moving_jaw,
        elem_a="guide_bushing_0",
        elem_b="guide_rod_0",
        reason="The guide rod is intentionally captured as a close sliding fit inside the fixed jaw bushing.",
    )
    ctx.allow_overlap(
        bench_front,
        moving_jaw,
        elem_a="guide_bushing_1",
        elem_b="guide_rod_1",
        reason="The guide rod is intentionally captured as a close sliding fit inside the fixed jaw bushing.",
    )

    ctx.check(
        "moving jaw uses horizontal prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.17,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "handle is a continuous screw-axis rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_gap(
        moving_jaw,
        bench_front,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_jaw",
        min_gap=0.030,
        max_gap=0.040,
        name="closed jaws have a narrow starting gap",
    )
    ctx.expect_within(
        moving_jaw,
        bench_front,
        axes="yz",
        inner_elem="guide_rod_0",
        outer_elem="guide_bushing_0",
        margin=0.002,
        name="lower guide rod stays centered in its bushing",
    )
    ctx.expect_overlap(
        moving_jaw,
        bench_front,
        axes="x",
        elem_a="guide_rod_0",
        elem_b="guide_bushing_0",
        min_overlap=0.020,
        name="lower guide rod is inserted in its bushing",
    )
    ctx.expect_within(
        moving_jaw,
        bench_front,
        axes="yz",
        inner_elem="guide_rod_1",
        outer_elem="guide_bushing_1",
        margin=0.002,
        name="upper guide rod stays centered in its bushing",
    )
    ctx.expect_overlap(
        moving_jaw,
        bench_front,
        axes="x",
        elem_a="guide_rod_1",
        elem_b="guide_bushing_1",
        min_overlap=0.020,
        name="upper guide rod is inserted in its bushing",
    )
    ctx.expect_overlap(
        moving_jaw,
        bench_front,
        axes="x",
        elem_a="guide_rod_0",
        elem_b="fixed_jaw",
        min_overlap=0.040,
        name="closed guide rod remains captured by fixed jaw",
    )
    ctx.expect_within(
        lead_screw,
        moving_jaw,
        axes="yz",
        inner_elem="threaded_shaft",
        outer_elem="jaw_face",
        margin=0.002,
        name="lead screw is centered in moving jaw bore",
    )

    rest_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({slide: 0.180}):
        ctx.expect_gap(
            moving_jaw,
            bench_front,
            axis="x",
            positive_elem="jaw_face",
            negative_elem="fixed_jaw",
            min_gap=0.205,
            max_gap=0.225,
            name="opened jaw moves away from fixed jaw",
        )
        ctx.expect_overlap(
            moving_jaw,
            bench_front,
            axes="x",
            elem_a="guide_rod_0",
            elem_b="fixed_jaw",
            min_overlap=0.030,
            name="extended guide rod is still retained",
        )
        extended_pos = ctx.part_world_position(moving_jaw)

    ctx.check(
        "slide travel is outward along the guide rods",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.170,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            lead_screw,
            moving_jaw,
            axes="yz",
            max_dist=0.001,
            name="rotating handle stays coaxial with screw bore",
        )

    return ctx.report()


object_model = build_object_model()
