from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_X = 0.22
LOWER_HINGE_Z = 0.45
UPPER_HINGE_Z = 1.25
HINGE_SPACING = UPPER_HINGE_Z - LOWER_HINGE_Z


def _hollow_sleeve_mesh(name: str):
    """A short gudgeon sleeve with a real bore for the fixed pintle pin."""
    sleeve = (
        cq.Workplane("XY")
        .circle(0.055)
        .circle(0.028)
        .extrude(0.080, both=True)
    )
    return mesh_from_cadquery(sleeve, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farm_pintle_swing_gate")

    weathered_wood = model.material("weathered_wood", rgba=(0.48, 0.30, 0.16, 1.0))
    end_grain = model.material("dark_end_grain", rgba=(0.30, 0.18, 0.09, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.04, 0.04, 0.035, 1.0))
    worn_steel = model.material("worn_pintle_steel", rgba=(0.23, 0.22, 0.20, 1.0))

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.14, length=1.70),
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        material=weathered_wood,
        name="round_post",
    )
    post.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 1.7175)),
        material=end_grain,
        name="post_cap",
    )
    post.visual(
        Cylinder(radius=0.150, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=end_grain,
        name="buried_butt",
    )

    for tag, z in (("lower", LOWER_HINGE_Z), ("upper", UPPER_HINGE_Z)):
        post.visual(
            Cylinder(radius=0.018, length=0.340),
            origin=Origin(xyz=(HINGE_X, 0.0, z)),
            material=worn_steel,
            name=f"{tag}_pin",
        )
        post.visual(
            Cylinder(radius=0.052, length=0.015),
            origin=Origin(xyz=(HINGE_X, 0.0, z - 0.0875)),
            material=black_iron,
            name=f"{tag}_washer",
        )
        post.visual(
            Box((0.170, 0.060, 0.035)),
            origin=Origin(xyz=(0.135, 0.0, z - 0.130)),
            material=black_iron,
            name=f"{tag}_pintle_arm",
        )
        post.visual(
            Box((0.055, 0.120, 0.160)),
            origin=Origin(xyz=(0.132, 0.0, z - 0.135)),
            material=black_iron,
            name=f"{tag}_post_plate",
        )

    lower_hinge = model.part("lower_hinge")
    lower_hinge.visual(
        _hollow_sleeve_mesh("lower_gudgeon_sleeve"),
        material=black_iron,
        name="sleeve",
    )
    lower_hinge.visual(
        Box((0.820, 0.020, 0.055)),
        origin=Origin(xyz=(0.430, -0.065, 0.0)),
        material=black_iron,
        name="strap",
    )
    lower_hinge.visual(
        Box((0.070, 0.030, 0.070)),
        origin=Origin(xyz=(0.045, -0.055, 0.0)),
        material=black_iron,
        name="sleeve_web",
    )
    for i, x in enumerate((0.23, 0.48, 0.72)):
        lower_hinge.visual(
            Cylinder(radius=0.023, length=0.014),
            origin=Origin(xyz=(x, -0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bolt_{i}",
        )

    upper_hinge = model.part("upper_hinge")
    upper_hinge.visual(
        _hollow_sleeve_mesh("upper_gudgeon_sleeve"),
        material=black_iron,
        name="sleeve",
    )
    upper_hinge.visual(
        Box((0.820, 0.020, 0.055)),
        origin=Origin(xyz=(0.430, -0.065, 0.0)),
        material=black_iron,
        name="strap",
    )
    upper_hinge.visual(
        Box((0.070, 0.030, 0.070)),
        origin=Origin(xyz=(0.045, -0.055, 0.0)),
        material=black_iron,
        name="sleeve_web",
    )
    for i, x in enumerate((0.23, 0.48, 0.72)):
        upper_hinge.visual(
            Cylinder(radius=0.023, length=0.014),
            origin=Origin(xyz=(x, -0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bolt_{i}",
        )

    gate_panel = model.part("gate_panel")
    gate_panel.visual(
        Box((0.145, 0.110, 1.250)),
        origin=Origin(xyz=(0.155, 0.0, 0.375)),
        material=weathered_wood,
        name="hinge_stile",
    )
    gate_panel.visual(
        Box((0.145, 0.110, 1.250)),
        origin=Origin(xyz=(3.115, 0.0, 0.375)),
        material=weathered_wood,
        name="latch_stile",
    )
    gate_panel.visual(
        Box((3.050, 0.105, 0.135)),
        origin=Origin(xyz=(1.615, 0.0, 0.925)),
        material=weathered_wood,
        name="top_rail",
    )
    gate_panel.visual(
        Box((3.050, 0.105, 0.130)),
        origin=Origin(xyz=(1.615, 0.0, -0.175)),
        material=weathered_wood,
        name="bottom_rail",
    )
    gate_panel.visual(
        Box((3.000, 0.095, 0.105)),
        origin=Origin(xyz=(1.610, 0.0, 0.375)),
        material=weathered_wood,
        name="middle_rail",
    )

    brace_dx = 2.78
    brace_dz = 0.98
    gate_panel.visual(
        Box((math.hypot(brace_dx, brace_dz), 0.095, 0.120)),
        origin=Origin(
            xyz=(1.610, 0.0, 0.365),
            rpy=(0.0, -math.atan2(brace_dz, brace_dx), 0.0),
        ),
        material=weathered_wood,
        name="diagonal_brace",
    )
    for i, x in enumerate((0.86, 1.55, 2.24)):
        gate_panel.visual(
            Box((0.075, 0.085, 0.860)),
            origin=Origin(xyz=(x, 0.0, 0.375)),
            material=weathered_wood,
            name=f"upright_{i}",
        )
    for i, z in enumerate((-0.240, 1.000)):
        gate_panel.visual(
            Box((3.080, 0.006, 0.018)),
            origin=Origin(xyz=(1.615, -0.055, z)),
            material=end_grain,
            name=f"front_edge_{i}",
        )

    lower_pintle = model.articulation(
        "lower_pintle",
        ArticulationType.REVOLUTE,
        parent=post,
        child=lower_hinge,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "hinge_to_panel",
        ArticulationType.FIXED,
        parent=lower_hinge,
        child=gate_panel,
        origin=Origin(),
    )
    model.articulation(
        "upper_pintle",
        ArticulationType.REVOLUTE,
        parent=post,
        child=upper_hinge,
        origin=Origin(xyz=(HINGE_X, 0.0, UPPER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.55),
        mimic=Mimic(joint=lower_pintle.name),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("post")
    lower_hinge = object_model.get_part("lower_hinge")
    upper_hinge = object_model.get_part("upper_hinge")
    gate_panel = object_model.get_part("gate_panel")
    lower = object_model.get_articulation("lower_pintle")
    upper = object_model.get_articulation("upper_pintle")

    ctx.check(
        "two vertical pintle revolutes",
        lower.articulation_type == ArticulationType.REVOLUTE
        and upper.articulation_type == ArticulationType.REVOLUTE
        and lower.axis == (0.0, 0.0, 1.0)
        and upper.axis == (0.0, 0.0, 1.0),
        details=f"lower={lower.articulation_type}/{lower.axis}, upper={upper.articulation_type}/{upper.axis}",
    )
    lower_xyz = lower.origin.xyz
    upper_xyz = upper.origin.xyz
    ctx.check(
        "pintle axes vertically aligned",
        abs(lower_xyz[0] - upper_xyz[0]) < 1e-9
        and abs(lower_xyz[1] - upper_xyz[1]) < 1e-9
        and abs((upper_xyz[2] - lower_xyz[2]) - HINGE_SPACING) < 1e-9,
        details=f"lower={lower_xyz}, upper={upper_xyz}",
    )
    ctx.check(
        "upper hinge follows panel swing",
        upper.mimic is not None and upper.mimic.joint == lower.name,
        details=f"mimic={upper.mimic}",
    )

    ctx.expect_within(
        post,
        lower_hinge,
        axes="xy",
        inner_elem="lower_pin",
        outer_elem="sleeve",
        margin=0.0,
        name="lower pin sits inside sleeve footprint",
    )
    ctx.expect_within(
        post,
        upper_hinge,
        axes="xy",
        inner_elem="upper_pin",
        outer_elem="sleeve",
        margin=0.0,
        name="upper pin sits inside sleeve footprint",
    )
    ctx.expect_overlap(
        post,
        lower_hinge,
        axes="z",
        elem_a="lower_pin",
        elem_b="sleeve",
        min_overlap=0.140,
        name="lower sleeve captures pintle height",
    )
    ctx.expect_overlap(
        post,
        upper_hinge,
        axes="z",
        elem_a="upper_pin",
        elem_b="sleeve",
        min_overlap=0.140,
        name="upper sleeve captures pintle height",
    )
    ctx.expect_contact(
        lower_hinge,
        gate_panel,
        elem_a="strap",
        elem_b="hinge_stile",
        contact_tol=0.0005,
        name="lower strap bears on timber stile",
    )
    ctx.expect_contact(
        upper_hinge,
        gate_panel,
        elem_a="strap",
        elem_b="hinge_stile",
        contact_tol=0.0005,
        name="upper strap bears on timber stile",
    )

    rest_aabb = ctx.part_element_world_aabb(gate_panel, elem="latch_stile")
    with ctx.pose({lower: 1.15}):
        swung_aabb = ctx.part_element_world_aabb(gate_panel, elem="latch_stile")
        ctx.expect_contact(
            upper_hinge,
            gate_panel,
            elem_a="strap",
            elem_b="hinge_stile",
            contact_tol=0.0005,
            name="upper strap stays aligned while swinging",
        )

    def _center_y(aabb):
        return None if aabb is None else 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "free end swings around post",
        rest_aabb is not None
        and swung_aabb is not None
        and _center_y(swung_aabb) is not None
        and _center_y(rest_aabb) is not None
        and _center_y(swung_aabb) > _center_y(rest_aabb) + 2.0,
        details=f"rest={rest_aabb}, swung={swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
