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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TAU = 2.0 * math.pi


def _annulus(outer: float, inner: float, height: float, *, z_center: float = 0.0):
    """CadQuery annular cylinder, authored in local metres."""

    outer_solid = (
        cq.Workplane("XY")
        .circle(outer)
        .extrude(height)
        .translate((0.0, 0.0, z_center - height / 2.0))
    )
    inner_void = (
        cq.Workplane("XY")
        .circle(inner)
        .extrude(height + 0.004)
        .translate((0.0, 0.0, z_center - height / 2.0 - 0.002))
    )
    return outer_solid.cut(inner_void)


def _cyl(radius: float, height: float, *, z_center: float = 0.0):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z_center - height / 2.0))
    )


def _annular_cut(outer: float, inner: float, depth: float, z_top: float):
    return _annulus(outer, inner, depth, z_center=z_top - depth / 2.0)


def _bolt_hole_cutter(
    count: int,
    pitch_radius: float,
    hole_radius: float,
    *,
    z_min: float,
    z_max: float,
    phase: float = 0.0,
):
    points = [
        (
            pitch_radius * math.cos(phase + TAU * index / count),
            pitch_radius * math.sin(phase + TAU * index / count),
        )
        for index in range(count)
    ]
    return (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(hole_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _arc_segment(
    outer: float,
    inner: float,
    height: float,
    start_angle: float,
    end_angle: float,
    *,
    z_center: float = 0.0,
    samples: int = 32,
):
    outer_points = [
        (
            outer * math.cos(start_angle + (end_angle - start_angle) * i / samples),
            outer * math.sin(start_angle + (end_angle - start_angle) * i / samples),
        )
        for i in range(samples + 1)
    ]
    inner_points = [
        (
            inner * math.cos(start_angle + (end_angle - start_angle) * i / samples),
            inner * math.sin(start_angle + (end_angle - start_angle) * i / samples),
        )
        for i in range(samples, -1, -1)
    ]
    return (
        cq.Workplane("XY")
        .polyline(outer_points + inner_points)
        .close()
        .extrude(height)
        .translate((0.0, 0.0, z_center - height / 2.0))
    )


def _root_support_mesh():
    """Stationary layered bearing tower and guards for the three rotary stages."""

    shape = _cyl(0.54, 0.075, z_center=0.0375)
    shape = shape.union(_annulus(0.56, 0.515, 0.060, z_center=0.105))
    shape = shape.union(_cyl(0.052, 0.535, z_center=0.310))
    shape = shape.union(_annulus(0.36, 0.052, 0.026, z_center=0.090))
    shape = shape.union(_annulus(0.245, 0.052, 0.024, z_center=0.201))
    shape = shape.union(_annulus(0.145, 0.052, 0.022, z_center=0.392))
    shape = shape.union(_annulus(0.095, 0.052, 0.018, z_center=0.568))
    shape = shape.cut(_annular_cut(0.348, 0.322, 0.004, 0.103))
    shape = shape.cut(_annular_cut(0.232, 0.205, 0.004, 0.233))
    shape = shape.cut(_annular_cut(0.136, 0.110, 0.003, 0.403))
    return shape


def _lower_turntable_mesh():
    shape = _annulus(0.485, 0.082, 0.090, z_center=0.0)
    shape = shape.union(_annulus(0.455, 0.370, 0.018, z_center=0.054))
    shape = shape.union(_annulus(0.200, 0.082, 0.024, z_center=0.057))
    shape = shape.cut(_annular_cut(0.432, 0.410, 0.006, 0.063))
    shape = shape.cut(_annular_cut(0.278, 0.258, 0.005, 0.063))
    shape = shape.cut(_bolt_hole_cutter(16, 0.405, 0.0105, z_min=-0.060, z_max=0.070))
    shape = shape.cut(_bolt_hole_cutter(8, 0.300, 0.0170, z_min=-0.060, z_max=0.070, phase=math.pi / 8))
    return shape


def _drum_ring_mesh():
    shape = _annulus(0.315, 0.132, 0.140, z_center=0.0)
    shape = shape.union(_annulus(0.315, 0.132, 0.026, z_center=0.083))
    shape = shape.union(_annulus(0.300, 0.145, 0.022, z_center=-0.081))
    shape = shape.cut(_annular_cut(0.287, 0.266, 0.006, 0.096))
    shape = shape.cut(_annular_cut(0.202, 0.178, 0.006, 0.096))
    shape = shape.cut(_bolt_hole_cutter(12, 0.238, 0.009, z_min=-0.095, z_max=0.105, phase=math.pi / 12))
    return shape


def _top_platen_mesh():
    shape = _annulus(0.190, 0.066, 0.066, z_center=0.0)
    shape = shape.union(_annulus(0.150, 0.066, 0.012, z_center=0.039))
    shape = shape.cut(_annular_cut(0.168, 0.154, 0.004, 0.045))
    shape = shape.cut(_bolt_hole_cutter(6, 0.128, 0.0065, z_min=-0.044, z_max=0.055, phase=math.pi / 6))
    # Two shallow crossed tooling slots cut into the top face.
    slot_x = cq.Workplane("XY").box(0.250, 0.030, 0.014).translate((0.0, 0.0, 0.040))
    slot_y = cq.Workplane("XY").box(0.030, 0.250, 0.014).translate((0.0, 0.0, 0.040))
    shape = shape.cut(slot_x).cut(slot_y)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_fixture_stack")

    cast_iron = model.material("satin_black_cast_iron", color=(0.035, 0.038, 0.040, 1.0))
    dark_oxide = model.material("black_oxide_fasteners", color=(0.010, 0.011, 0.012, 1.0))
    parkerized = model.material("parkerized_steel", color=(0.18, 0.19, 0.19, 1.0))
    machined = model.material("machined_steel", color=(0.58, 0.60, 0.59, 1.0))
    o_ring = model.material("dark_rubber_seals", color=(0.005, 0.005, 0.004, 1.0))
    brass = model.material("brass_index_marks", color=(0.85, 0.63, 0.22, 1.0))

    fixed_base = model.part("fixed_base")
    fixed_base.visual(
        Cylinder(radius=0.54, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=cast_iron,
        name="fixed_housing",
    )
    fixed_base.visual(
        Cylinder(radius=0.56, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=cast_iron,
        name="lower_cast_shelf",
    )
    fixed_base.visual(
        Cylinder(radius=0.265, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.231)),
        material=cast_iron,
        name="drum_cast_shelf",
    )
    fixed_base.visual(
        Cylinder(radius=0.120, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.422)),
        material=cast_iron,
        name="platen_cast_shelf",
    )
    fixed_base.visual(
        Cylinder(radius=0.048, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.322)),
        material=machined,
        name="center_spindle",
    )
    fixed_base.visual(
        Cylinder(radius=0.382, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=machined,
        name="lower_support",
    )
    fixed_base.visual(
        Cylinder(radius=0.258, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.2355)),
        material=machined,
        name="drum_support",
    )
    fixed_base.visual(
        Cylinder(radius=0.118, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=machined,
        name="platen_support",
    )

    lower = model.part("lower_turntable")
    lower.visual(
        mesh_from_cadquery(_lower_turntable_mesh(), "lower_turntable_body", tolerance=0.0008),
        material=machined,
        name="lower_body",
    )
    for index in range(16):
        angle = TAU * index / 16
        lower.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(
                xyz=(0.405 * math.cos(angle), 0.405 * math.sin(angle), 0.068)
            ),
            material=dark_oxide,
            name=f"lower_bolt_{index:02d}",
        )
    lower.visual(
        Box((0.058, 0.020, 0.012)),
        origin=Origin(xyz=(0.345, 0.0, 0.051)),
        material=brass,
        name="lower_zero_mark",
    )
    lower.visual(
        mesh_from_cadquery(_annulus(0.433, 0.410, 0.006, z_center=0.066), "lower_outer_seal"),
        material=o_ring,
        name="lower_outer_seal",
    )

    drum = model.part("drum_ring")
    drum.visual(
        mesh_from_cadquery(_drum_ring_mesh(), "drum_ring_body", tolerance=0.0008),
        material=parkerized,
        name="drum_body",
    )
    drum.visual(
        mesh_from_cadquery(
            _arc_segment(0.248, 0.205, 0.014, 0.12, math.pi - 0.12, z_center=0.103),
            "drum_retainer_0",
            tolerance=0.0008,
        ),
        material=dark_oxide,
        name="retainer_0",
    )
    drum.visual(
        mesh_from_cadquery(
            _arc_segment(0.248, 0.205, 0.014, math.pi + 0.12, TAU - 0.12, z_center=0.103),
            "drum_retainer_1",
            tolerance=0.0008,
        ),
        material=dark_oxide,
        name="retainer_1",
    )
    drum.visual(
        mesh_from_cadquery(_annulus(0.321, 0.315, 0.010, z_center=0.0), "drum_black_split_line"),
        material=o_ring,
        name="drum_split_line",
    )
    for index in range(12):
        angle = math.pi / 12 + TAU * index / 12
        drum.visual(
            Cylinder(radius=0.0125, length=0.009),
            origin=Origin(
                xyz=(0.238 * math.cos(angle), 0.238 * math.sin(angle), 0.1145)
            ),
            material=dark_oxide,
            name=f"drum_bolt_{index:02d}",
        )
    drum.visual(
        Box((0.044, 0.018, 0.012)),
        origin=Origin(xyz=(0.248, 0.0, 0.102)),
        material=brass,
        name="drum_zero_mark",
    )

    platen = model.part("top_platen")
    platen.visual(
        mesh_from_cadquery(_top_platen_mesh(), "top_platen_body", tolerance=0.0007),
        material=machined,
        name="platen_body",
    )
    for index in range(6):
        angle = math.pi / 6 + TAU * index / 6
        platen.visual(
            Cylinder(radius=0.0095, length=0.008),
            origin=Origin(
                xyz=(0.128 * math.cos(angle), 0.128 * math.sin(angle), 0.049)
            ),
            material=dark_oxide,
            name=f"platen_bolt_{index:02d}",
        )
    platen.visual(
        Box((0.048, 0.018, 0.010)),
        origin=Origin(xyz=(0.122, 0.0, 0.050)),
        material=brass,
        name="platen_tool_key",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "base_to_drum",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "base_to_platen",
        ArticulationType.REVOLUTE,
        parent=fixed_base,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, 0.465)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_turntable")
    drum = object_model.get_part("drum_ring")
    platen = object_model.get_part("top_platen")
    base = object_model.get_part("fixed_base")
    lower_joint = object_model.get_articulation("base_to_lower")
    drum_joint = object_model.get_articulation("base_to_drum")
    platen_joint = object_model.get_articulation("base_to_platen")

    joints = (lower_joint, drum_joint, platen_joint)
    ctx.check(
        "three vertical revolute stages",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    ctx.expect_gap(
        lower,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_body",
        negative_elem="lower_support",
        name="lower turntable clears lower bearing cover",
    )
    ctx.expect_gap(
        drum,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="drum_body",
        negative_elem="drum_support",
        name="drum ring clears secondary bearing cover",
    )
    ctx.expect_gap(
        platen,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="platen_body",
        negative_elem="platen_support",
        name="top platen clears upper bearing cover",
    )
    ctx.expect_overlap(
        lower,
        base,
        axes="xy",
        min_overlap=0.25,
        elem_a="lower_body",
        elem_b="lower_support",
        name="lower bearing race under broad turntable",
    )
    ctx.expect_overlap(
        drum,
        base,
        axes="xy",
        min_overlap=0.15,
        elem_a="drum_body",
        elem_b="drum_support",
        name="drum bearing race under indexing body",
    )
    ctx.expect_overlap(
        platen,
        base,
        axes="xy",
        min_overlap=0.07,
        elem_a="platen_body",
        elem_b="platen_support",
        name="platen bearing race under tooling interface",
    )

    def _aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    lower_rest = _aabb_center(ctx.part_element_world_aabb(lower, elem="lower_zero_mark"))
    drum_rest = _aabb_center(ctx.part_element_world_aabb(drum, elem="drum_zero_mark"))
    platen_rest = _aabb_center(ctx.part_element_world_aabb(platen, elem="platen_tool_key"))
    with ctx.pose({lower_joint: math.pi / 2.0, drum_joint: math.pi / 2.0, platen_joint: math.pi / 2.0}):
        lower_rot = _aabb_center(ctx.part_element_world_aabb(lower, elem="lower_zero_mark"))
        drum_rot = _aabb_center(ctx.part_element_world_aabb(drum, elem="drum_zero_mark"))
        platen_rot = _aabb_center(ctx.part_element_world_aabb(platen, elem="platen_tool_key"))
        ctx.expect_gap(
            lower,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="lower_body",
            negative_elem="lower_support",
            name="lower bearing clearance persists when indexed",
        )
        ctx.expect_gap(
            drum,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="drum_body",
            negative_elem="drum_support",
            name="drum bearing clearance persists when indexed",
        )
        ctx.expect_gap(
            platen,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="platen_body",
            negative_elem="platen_support",
            name="platen bearing clearance persists when indexed",
        )

    ctx.check(
        "stage index marks rotate about the shared centerline",
        lower_rot[1] > lower_rest[1] + 0.25
        and drum_rot[1] > drum_rest[1] + 0.16
        and platen_rot[1] > platen_rest[1] + 0.08,
        details=f"lower {lower_rest}->{lower_rot}, drum {drum_rest}->{drum_rot}, platen {platen_rest}->{platen_rot}",
    )

    return ctx.report()


object_model = build_object_model()
