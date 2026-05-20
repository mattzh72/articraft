from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_pedestal_spindle_module")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    bearing_paint = model.material("machined_black_bearing", rgba=(0.03, 0.035, 0.04, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    bolt_steel = model.material("dark_bolt_heads", rgba=(0.025, 0.025, 0.025, 1.0))

    frame = model.part("frame")

    foot_shape = (
        cq.Workplane("XY")
        .box(1.22, 0.44, 0.12)
        .edges("|Z")
        .fillet(0.025)
        .edges(">Z")
        .chamfer(0.010)
    )
    frame.visual(
        mesh_from_cadquery(foot_shape, "heavy_foot"),
        origin=Origin(xyz=(0.10, 0.0, 0.06)),
        material=cast_iron,
        name="heavy_foot",
    )

    shaft_z = 0.43
    ring_radius = 0.128
    ring_tube = 0.026
    ring_mesh = mesh_from_geometry(
        TorusGeometry(ring_radius, ring_tube, radial_segments=28, tubular_segments=56),
        "bearing_ring",
    )
    bushing_shape = (
        cq.Workplane("YZ")
        .circle(0.072)
        .circle(0.054)
        .extrude(0.090)
        .translate((-0.045, 0.0, 0.0))
    )
    bushing_mesh = mesh_from_cadquery(bushing_shape, "bearing_bushing")

    for idx, x in enumerate((-0.25, 0.25)):
        frame.visual(
            Box((0.16, 0.19, 0.215)),
            origin=Origin(xyz=(x, 0.0, 0.218)),
            material=cast_iron,
            name=f"pedestal_{idx}",
        )
        frame.visual(
            Box((0.20, 0.30, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.142)),
            material=cast_iron,
            name=f"pedestal_foot_{idx}",
        )
        frame.visual(
            Box((0.12, 0.055, 0.12)),
            origin=Origin(xyz=(x, -0.118, 0.352)),
            material=cast_iron,
            name=f"cap_lug_{idx}_0",
        )
        frame.visual(
            Box((0.12, 0.055, 0.12)),
            origin=Origin(xyz=(x, 0.118, 0.352)),
            material=cast_iron,
            name=f"cap_lug_{idx}_1",
        )

    frame.visual(
        ring_mesh,
        origin=Origin(xyz=(-0.25, 0.0, shaft_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_paint,
        name="bearing_ring_0",
    )
    frame.visual(
        ring_mesh,
        origin=Origin(xyz=(0.25, 0.0, shaft_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_paint,
        name="bearing_ring_1",
    )
    frame.visual(
        bushing_mesh,
        origin=Origin(xyz=(-0.25, 0.0, shaft_z)),
        material=bearing_paint,
        name="bearing_bushing_0",
    )
    frame.visual(
        bushing_mesh,
        origin=Origin(xyz=(0.25, 0.0, shaft_z)),
        material=bearing_paint,
        name="bearing_bushing_1",
    )
    for idx, x in enumerate((-0.25, 0.25)):
        frame.visual(
            Box((0.070, 0.040, 0.052)),
            origin=Origin(xyz=(x, 0.0, shaft_z + 0.087)),
            material=bearing_paint,
            name=f"bearing_web_{idx}_0",
        )
        frame.visual(
            Box((0.070, 0.040, 0.052)),
            origin=Origin(xyz=(x, 0.0, shaft_z - 0.087)),
            material=bearing_paint,
            name=f"bearing_web_{idx}_1",
        )
        frame.visual(
            Box((0.070, 0.052, 0.040)),
            origin=Origin(xyz=(x, 0.087, shaft_z)),
            material=bearing_paint,
            name=f"bearing_web_{idx}_2",
        )
        frame.visual(
            Box((0.070, 0.052, 0.040)),
            origin=Origin(xyz=(x, -0.087, shaft_z)),
            material=bearing_paint,
            name=f"bearing_web_{idx}_3",
        )

    for idx, (x, y) in enumerate(
        (
            (-0.43, -0.155),
            (-0.43, 0.155),
            (0.52, -0.155),
            (0.52, 0.155),
        )
    ):
        frame.visual(
            Cylinder(radius=0.025, length=0.014),
            origin=Origin(xyz=(x, y, 0.127)),
            material=bolt_steel,
            name=f"mount_bolt_{idx}",
        )

    spindle = model.part("spindle")
    cylinder_rpy = (0.0, pi / 2.0, 0.0)
    spindle.visual(
        Cylinder(radius=0.055, length=1.10),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=cylinder_rpy),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(xyz=(-0.335, 0.0, 0.0), rpy=cylinder_rpy),
        material=steel,
        name="shaft_collar_0",
    )
    spindle.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=cylinder_rpy),
        material=steel,
        name="shaft_collar_1",
    )
    spindle.visual(
        Cylinder(radius=0.090, length=0.19),
        origin=Origin(xyz=(0.590, 0.0, 0.0), rpy=cylinder_rpy),
        material=steel,
        name="faceplate_hub",
    )
    spindle.visual(
        Cylinder(radius=0.180, length=0.085),
        origin=Origin(xyz=(0.720, 0.0, 0.0), rpy=cylinder_rpy),
        material=steel,
        name="faceplate",
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    spindle = object_model.get_part("spindle")
    spin = object_model.get_articulation("shaft_spin")

    ctx.check(
        "single continuous spindle joint",
        len(object_model.articulations) == 1
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"joint_type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.allow_overlap(
        frame,
        spindle,
        elem_a="bearing_bushing_0",
        elem_b="shaft",
        reason=(
            "The rotating shaft is intentionally captured in a close bearing "
            "bushing proxy at the first pedestal."
        ),
    )
    ctx.allow_overlap(
        frame,
        spindle,
        elem_a="bearing_bushing_1",
        elem_b="shaft",
        reason=(
            "The rotating shaft is intentionally captured in a close bearing "
            "bushing proxy at the second pedestal."
        ),
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_ring_0",
        min_overlap=0.040,
        name="shaft passes through first bearing",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_ring_1",
        min_overlap=0.040,
        name="shaft passes through second bearing",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_bushing_0",
        min_overlap=0.060,
        name="shaft retained in first bushing",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_bushing_1",
        min_overlap=0.060,
        name="shaft retained in second bushing",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_ring_0",
        margin=0.0,
        name="shaft centered in first bearing envelope",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_ring_1",
        margin=0.0,
        name="shaft centered in second bearing envelope",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_bushing_0",
        margin=0.002,
        name="shaft centered in first bushing",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_bushing_1",
        margin=0.002,
        name="shaft centered in second bushing",
    )
    ctx.expect_gap(
        spindle,
        frame,
        axis="x",
        positive_elem="faceplate",
        negative_elem="bearing_ring_1",
        min_gap=0.30,
        name="faceplate projects beyond one support",
    )

    return ctx.report()


object_model = build_object_model()
