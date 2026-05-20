from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    center_y: float,
    center_z: float,
    radius: float,
    *,
    segments: int = 28,
) -> list[tuple[float, float]]:
    return [
        (
            center_y + radius * math.cos(2.0 * math.pi * i / segments),
            center_z + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _yz_to_local(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    # Build the cross-section in world YZ, then map it into ExtrudeWithHolesGeometry's
    # local XY plane. A +90 deg rotation around Y sends local +Z to world +X.
    return [(-z, y) for y, z in profile]


def _extrude_along_x_with_holes(
    outer_profile_yz: list[tuple[float, float]],
    hole_profiles_yz: list[list[tuple[float, float]]],
    length: float,
):
    return (
        ExtrudeWithHolesGeometry(
            _yz_to_local(outer_profile_yz),
            [_yz_to_local(hole) for hole in hole_profiles_yz],
            height=length,
            center=False,
        ).rotate_y(math.pi / 2.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_vise")

    bench_wood = model.material("bench_wood", rgba=(0.54, 0.37, 0.20, 1.0))
    painted_iron = model.material("painted_iron", rgba=(0.12, 0.26, 0.52, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.78, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.14, 0.15, 1.0))

    bench_height = 0.90
    top_thickness = 0.06
    top_size = (1.40, 0.75, top_thickness)

    body_length = 0.18
    body_width = 0.085
    body_bottom = 0.02
    body_top = 0.12
    axis_z = 0.09

    flange_length = 0.14
    flange_width = 0.11
    flange_thickness = 0.016

    fixed_jaw_thickness = 0.028
    fixed_jaw_height = 0.125
    fixed_jaw_face_x = body_length - 0.002

    moving_jaw_length = 0.05
    moving_jaw_height = 0.125
    open_front_x = -0.095
    closed_gap = 0.004
    jaw_travel = fixed_jaw_face_x - closed_gap - open_front_x

    rod_radius = 0.007
    rod_hole_radius = 0.0085
    rod_offset_y = 0.028
    rod_rear_x = -0.165
    rod_front_x = fixed_jaw_face_x - 0.002
    rod_length = rod_front_x - rod_rear_x

    screw_radius = 0.0085
    screw_hole_radius = 0.0105
    screw_rear_x = -0.195
    screw_front_x = fixed_jaw_face_x - 0.001
    screw_length = screw_front_x - screw_rear_x

    handle_span = 0.13
    handle_knob_radius = 0.012

    bench = model.part("workbench")
    bench.visual(
        Box(top_size),
        origin=Origin(xyz=(0.0, 0.0, bench_height - top_thickness / 2.0)),
        material=bench_wood,
        name="top_slab",
    )
    bench.visual(
        Box((1.24, 0.07, 0.16)),
        origin=Origin(xyz=(0.0, -0.31, 0.80)),
        material=bench_wood,
        name="front_apron",
    )
    bench.visual(
        Box((0.09, 0.09, 0.78)),
        origin=Origin(xyz=(-0.56, -0.28, 0.39)),
        material=bench_wood,
        name="left_front_leg",
    )
    bench.visual(
        Box((0.09, 0.09, 0.78)),
        origin=Origin(xyz=(0.56, -0.28, 0.39)),
        material=bench_wood,
        name="right_front_leg",
    )
    bench.inertial = Inertial.from_geometry(
        Box((1.40, 0.75, 0.90)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    body = model.part("vise_body")
    body_outer_profile = [
        (-body_width / 2.0, body_bottom),
        (body_width / 2.0, body_bottom),
        (body_width / 2.0, body_top),
        (-body_width / 2.0, body_top),
    ]
    body_holes = [
        _circle_profile(-rod_offset_y, axis_z, rod_hole_radius),
        _circle_profile(rod_offset_y, axis_z, rod_hole_radius),
        _circle_profile(0.0, axis_z, screw_hole_radius),
    ]
    body_mesh = mesh_from_geometry(
        _extrude_along_x_with_holes(body_outer_profile, body_holes, body_length),
        "vise_body_shell",
    )
    body.visual(body_mesh, material=painted_iron, name="body_shell")
    body.visual(
        Box((flange_length, flange_width, flange_thickness)),
        origin=Origin(
            xyz=(0.074, 0.0, flange_thickness / 2.0),
        ),
        material=painted_iron,
        name="mounting_flange",
    )
    body.visual(
        Box((fixed_jaw_thickness, 0.10, fixed_jaw_height)),
        origin=Origin(
            xyz=(
                fixed_jaw_face_x + fixed_jaw_thickness / 2.0,
                0.0,
                fixed_jaw_height / 2.0,
            )
        ),
        material=painted_iron,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.055, 0.08, 0.02)),
        origin=Origin(xyz=(body_length - 0.03, 0.0, body_top + 0.01)),
        material=dark_iron,
        name="anvil_pad",
    )
    for idx, bolt_x in enumerate((0.03, 0.11), start=1):
        for side, bolt_y in (("left", -0.034), ("right", 0.034)):
            body.visual(
                Cylinder(radius=0.009, length=0.005),
                origin=Origin(xyz=(bolt_x, bolt_y, flange_thickness + 0.0025)),
                material=black_oxide,
                name=f"bolt_head_{idx}_{side}",
            )
    body.inertial = Inertial.from_geometry(
        Box((body_length + fixed_jaw_thickness, flange_width, fixed_jaw_height)),
        mass=18.0,
        origin=Origin(
            xyz=((body_length + fixed_jaw_thickness) / 2.0, 0.0, fixed_jaw_height / 2.0)
        ),
    )

    left_rod = model.part("left_guide_rod")
    left_rod.visual(
        Cylinder(radius=rod_radius, length=rod_length),
        origin=Origin(
            xyz=(rod_length / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="guide_rod",
    )
    left_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=rod_radius, length=rod_length),
        mass=0.35,
        origin=Origin(
            xyz=(rod_length / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    right_rod = model.part("right_guide_rod")
    right_rod.visual(
        Cylinder(radius=rod_radius, length=rod_length),
        origin=Origin(
            xyz=(rod_length / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="guide_rod",
    )
    right_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=rod_radius, length=rod_length),
        mass=0.35,
        origin=Origin(
            xyz=(rod_length / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw_profile = [
        (-0.05, 0.0),
        (0.05, 0.0),
        (0.05, moving_jaw_height),
        (-0.05, moving_jaw_height),
    ]
    moving_jaw_holes = [
        _circle_profile(-rod_offset_y, axis_z, rod_hole_radius + 0.0006),
        _circle_profile(rod_offset_y, axis_z, rod_hole_radius + 0.0006),
        _circle_profile(0.0, axis_z, screw_hole_radius + 0.0008),
    ]
    moving_jaw_mesh = mesh_from_geometry(
        _extrude_along_x_with_holes(moving_jaw_profile, moving_jaw_holes, moving_jaw_length),
        "moving_jaw_block",
    )
    moving_jaw.visual(
        moving_jaw_mesh,
        origin=Origin(xyz=(-moving_jaw_length, 0.0, 0.0)),
        material=painted_iron,
        name="moving_jaw_block",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((moving_jaw_length, 0.10, moving_jaw_height)),
        mass=5.0,
        origin=Origin(xyz=(-moving_jaw_length / 2.0, 0.0, moving_jaw_height / 2.0)),
    )

    screw = model.part("lead_screw")
    screw.visual(
        Cylinder(radius=screw_radius, length=screw_length),
        origin=Origin(
            xyz=(screw_length / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="lead_screw_shaft",
    )
    screw.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(
            xyz=(0.018, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_iron,
        name="handle_hub",
    )
    screw.visual(
        Cylinder(radius=0.006, length=handle_span),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="t_bar",
    )
    screw.visual(
        Sphere(radius=handle_knob_radius),
        origin=Origin(xyz=(0.018, -handle_span / 2.0, 0.0)),
        material=black_oxide,
        name="left_handle_knob",
    )
    screw.visual(
        Sphere(radius=handle_knob_radius),
        origin=Origin(xyz=(0.018, handle_span / 2.0, 0.0)),
        material=black_oxide,
        name="right_handle_knob",
    )
    screw.inertial = Inertial.from_geometry(
        Box((screw_length, handle_span, 0.03)),
        mass=1.3,
        origin=Origin(xyz=(screw_length / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "bench_to_vise",
        ArticulationType.FIXED,
        parent=bench,
        child=body,
        origin=Origin(xyz=(0.20, 0.0, bench_height)),
    )
    model.articulation(
        "body_to_left_rod",
        ArticulationType.FIXED,
        parent=body,
        child=left_rod,
        origin=Origin(xyz=(rod_rear_x, -rod_offset_y, axis_z)),
    )
    model.articulation(
        "body_to_right_rod",
        ArticulationType.FIXED,
        parent=body,
        child=right_rod,
        origin=Origin(xyz=(rod_rear_x, rod_offset_y, axis_z)),
    )
    model.articulation(
        "body_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moving_jaw,
        origin=Origin(xyz=(open_front_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.08,
            lower=0.0,
            upper=jaw_travel,
        ),
    )
    model.articulation(
        "body_to_lead_screw",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=screw,
        origin=Origin(xyz=(screw_rear_x, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bench = object_model.get_part("workbench")
    body = object_model.get_part("vise_body")
    left_rod = object_model.get_part("left_guide_rod")
    moving_jaw = object_model.get_part("moving_jaw")
    screw = object_model.get_part("lead_screw")

    jaw_slide = object_model.get_articulation("body_to_moving_jaw")
    handle_spin = object_model.get_articulation("body_to_lead_screw")

    ctx.allow_overlap(
        body,
        left_rod,
        elem_a="body_shell",
        elem_b="guide_rod",
        reason="The cast body shell stands in for a bored guide-rod passage.",
    )
    ctx.allow_overlap(
        body,
        object_model.get_part("right_guide_rod"),
        elem_a="body_shell",
        elem_b="guide_rod",
        reason="The cast body shell stands in for a bored guide-rod passage.",
    )
    ctx.allow_overlap(
        body,
        screw,
        elem_a="body_shell",
        elem_b="lead_screw_shaft",
        reason="The cast body shell stands in for the internal lead-screw bore.",
    )
    ctx.allow_overlap(
        moving_jaw,
        left_rod,
        elem_a="moving_jaw_block",
        elem_b="guide_rod",
        reason="The moving jaw block is a simplified proxy for drilled rod bores.",
    )
    ctx.allow_overlap(
        moving_jaw,
        object_model.get_part("right_guide_rod"),
        elem_a="moving_jaw_block",
        elem_b="guide_rod",
        reason="The moving jaw block is a simplified proxy for drilled rod bores.",
    )
    ctx.allow_overlap(
        moving_jaw,
        screw,
        elem_a="moving_jaw_block",
        elem_b="lead_screw_shaft",
        reason="The moving jaw block is a simplified proxy for the threaded screw bore.",
    )

    ctx.expect_gap(
        body,
        bench,
        axis="z",
        positive_elem="mounting_flange",
        negative_elem="top_slab",
        max_gap=0.001,
        max_penetration=0.0,
        name="vise flange sits flush on the bench top",
    )

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_gap(
            body,
            moving_jaw,
            axis="x",
            positive_elem="fixed_jaw",
            negative_elem="moving_jaw_block",
            min_gap=0.26,
            name="open vise leaves a wide jaw opening",
        )
        ctx.expect_overlap(
            moving_jaw,
            left_rod,
            axes="x",
            elem_a="moving_jaw_block",
            elem_b="guide_rod",
            min_overlap=0.045,
            name="open moving jaw remains engaged on the left guide rod",
        )
        ctx.expect_overlap(
            moving_jaw,
            screw,
            axes="x",
            elem_a="moving_jaw_block",
            elem_b="lead_screw_shaft",
            min_overlap=0.045,
            name="open moving jaw remains threaded onto the lead screw span",
        )
        open_pos = ctx.part_world_position(moving_jaw)

    with ctx.pose({jaw_slide: jaw_slide.motion_limits.upper}):
        ctx.expect_gap(
            body,
            moving_jaw,
            axis="x",
            positive_elem="fixed_jaw",
            negative_elem="moving_jaw_block",
            min_gap=0.002,
            max_gap=0.008,
            name="closed vise brings the jaws nearly together",
        )
        ctx.expect_overlap(
            moving_jaw,
            left_rod,
            axes="x",
            elem_a="moving_jaw_block",
            elem_b="guide_rod",
            min_overlap=0.045,
            name="closed moving jaw remains engaged on the left guide rod",
        )
        ctx.expect_overlap(
            moving_jaw,
            screw,
            axes="x",
            elem_a="moving_jaw_block",
            elem_b="lead_screw_shaft",
            min_overlap=0.045,
            name="closed moving jaw remains threaded onto the lead screw span",
        )
        closed_pos = ctx.part_world_position(moving_jaw)

    ctx.check(
        "moving jaw advances toward the fixed jaw",
        open_pos is not None and closed_pos is not None and closed_pos[0] > open_pos[0] + 0.20,
        details=f"open={open_pos}, closed={closed_pos}",
    )

    with ctx.pose({handle_spin: math.pi / 2.0}):
        ctx.expect_gap(
            screw,
            bench,
            axis="z",
            negative_elem="top_slab",
            min_gap=0.005,
            name="vertical T-bar handle clears the bench top",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
