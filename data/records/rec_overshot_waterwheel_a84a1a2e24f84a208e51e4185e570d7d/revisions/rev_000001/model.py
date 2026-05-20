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


WHEEL_CENTER_Z = 1.0
WHEEL_RADIUS = 0.82
WHEEL_WIDTH = 0.44
GATE_TRAVEL = 0.42


def _annulus(outer_radius: float, inner_radius: float, thickness: float, z_center: float = 0.0):
    """CadQuery annular disk centered on the local Z axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness / 2.0))
    )


def _rotated_box(
    length: float,
    width: float,
    thickness: float,
    radius: float,
    z_center: float,
    angle_degrees: float,
):
    """A box whose length runs radially before rotation around local Z."""
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .translate((radius, 0.0, z_center))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_degrees)
    )


def _wheel_mesh():
    """Open cast-iron overshot wheel: side rings, spokes, hub, and bucket plates."""
    side_thickness = 0.035
    side_z = WHEEL_WIDTH / 2.0 - side_thickness / 2.0

    wheel = _annulus(WHEEL_RADIUS, 0.72, side_thickness, -side_z)
    wheel = wheel.union(_annulus(WHEEL_RADIUS, 0.72, side_thickness, side_z))

    # Inner cast rings and a full-width hub tie both side frames together.
    for zc in (-side_z, side_z):
        wheel = wheel.union(_annulus(0.34, 0.24, side_thickness, zc))
    hub = (
        cq.Workplane("XY")
        .circle(0.155)
        .extrude(WHEEL_WIDTH)
        .translate((0.0, 0.0, -WHEEL_WIDTH / 2.0))
    )
    wheel = wheel.union(hub)

    # Twelve slender cast spokes on each side frame.
    for zc in (-side_z, side_z):
        for index in range(12):
            angle = index * 30.0
            wheel = wheel.union(_rotated_box(0.66, 0.050, side_thickness, 0.462, zc, angle))

    # Overshot bucket dividers spanning the width.  These make the rim read as
    # a water wheel rather than a plain flywheel, while also bracing the sides.
    for index in range(16):
        angle = index * 22.5 + 8.0
        wheel = wheel.union(_rotated_box(0.18, 0.055, WHEEL_WIDTH * 0.92, 0.735, 0.0, angle))

    return wheel


def _bearing_ring_mesh():
    return _annulus(0.165, 0.086, 0.080, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_wheel_with_flume")

    stone = model.material("weathered_stone", rgba=(0.45, 0.43, 0.39, 1.0))
    mortar = model.material("dark_mortar", rgba=(0.19, 0.18, 0.17, 1.0))
    iron = model.material("black_cast_iron", rgba=(0.03, 0.032, 0.03, 1.0))
    wet_iron = model.material("oiled_gate_iron", rgba=(0.08, 0.075, 0.065, 1.0))
    water = model.material("flume_water", rgba=(0.18, 0.42, 0.72, 0.55))

    masonry = model.part("masonry")

    # Heavy stone foundation and side piers.
    masonry.visual(
        Box((2.55, 2.35, 0.18)),
        origin=Origin(xyz=(-0.10, 0.0, 0.09)),
        material=stone,
        name="stone_plinth",
    )
    for y in (-0.95, 0.95):
        masonry.visual(
            Box((0.56, 0.44, 2.20)),
            origin=Origin(xyz=(0.0, y, 1.18)),
            material=stone,
            name=f"stone_pier_{'neg' if y < 0 else 'pos'}",
        )
        masonry.visual(
            Box((0.68, 0.54, 0.18)),
            origin=Origin(xyz=(0.0, y, 2.37)),
            material=stone,
            name=f"pier_cap_{'neg' if y < 0 else 'pos'}",
        )
        # Raised/darker mortar bands on the outside faces make the piers read as
        # block masonry without splitting them into unsupported stones.
        face_y = y + (0.217 if y > 0 else -0.217)
        for course, z in enumerate((0.46, 0.78, 1.10, 1.42, 1.74, 2.06)):
            masonry.visual(
                Box((0.58, 0.014, 0.020)),
                origin=Origin(xyz=(0.0, face_y, z)),
                material=mortar,
                name=f"mortar_course_{'pos' if y > 0 else 'neg'}_{course}",
            )

    # A masonry flume carried on a lintel just above the wheel's upper rim.
    masonry.visual(
        Box((1.75, 1.62, 0.10)),
        origin=Origin(xyz=(-0.47, 0.0, 1.93)),
        material=stone,
        name="flume_lintel",
    )
    masonry.visual(
        Box((1.78, 0.78, 0.10)),
        origin=Origin(xyz=(-0.48, 0.0, 2.02)),
        material=stone,
        name="flume_floor",
    )
    for y in (-0.44, 0.44):
        masonry.visual(
            Box((1.78, 0.12, 0.38)),
            origin=Origin(xyz=(-0.48, y, 2.22)),
            material=stone,
            name=f"flume_side_{'neg' if y < 0 else 'pos'}",
        )
    masonry.visual(
        Box((1.15, 0.58, 0.035)),
        origin=Origin(xyz=(-0.70, 0.0, 2.0875)),
        material=water,
        name="standing_water",
    )

    # Iron guide cheeks at the flume mouth: two rails on each side form vertical
    # slots that retain the drop-in gate panel while leaving running clearance.
    masonry.visual(
        Box((0.035, 0.055, 0.82)),
        origin=Origin(xyz=(0.410, -0.372, 2.21)),
        material=iron,
        name="guide_upstream_neg",
    )
    masonry.visual(
        Box((0.035, 0.055, 0.82)),
        origin=Origin(xyz=(0.535, -0.372, 2.21)),
        material=iron,
        name="guide_downstream_neg",
    )
    masonry.visual(
        Box((0.035, 0.055, 0.82)),
        origin=Origin(xyz=(0.410, 0.372, 2.21)),
        material=iron,
        name="guide_upstream_pos",
    )
    masonry.visual(
        Box((0.035, 0.055, 0.82)),
        origin=Origin(xyz=(0.535, 0.372, 2.21)),
        material=iron,
        name="guide_downstream_pos",
    )
    masonry.visual(
        Box((0.13, 0.055, 0.82)),
        origin=Origin(xyz=(0.472, -0.3575, 2.21)),
        material=iron,
        name="guide_liner_neg",
    )
    masonry.visual(
        Box((0.13, 0.055, 0.82)),
        origin=Origin(xyz=(0.472, 0.3575, 2.21)),
        material=iron,
        name="guide_liner_pos",
    )

    # Bearing rings are hollow mesh annuli, so the rotating axle passes through
    # them instead of being hidden by an artificial solid overlap.
    bearing_shape = _bearing_ring_mesh()
    for y in (-0.690, 0.690):
        masonry.visual(
            mesh_from_cadquery(bearing_shape, f"bearing_ring_{'neg' if y < 0 else 'pos'}"),
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"bearing_ring_{'neg' if y < 0 else 'pos'}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_wheel_mesh(), "wheel_frame", tolerance=0.002, angular_tolerance=0.08),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="wheel_frame",
    )
    wheel.visual(
        Cylinder(radius=0.060, length=1.34),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_shaft",
    )

    gate = model.part("gate_panel")
    gate.visual(
        Box((0.052, 0.66, 0.54)),
        origin=Origin(),
        material=wet_iron,
        name="gate_plate",
    )
    gate.visual(
        Box((0.040, 0.11, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=wet_iron,
        name="lift_stem",
    )
    gate.visual(
        Cylinder(radius=0.024, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wet_iron,
        name="lift_grip",
    )

    model.articulation(
        "axle_rotation",
        ArticulationType.CONTINUOUS,
        parent=masonry,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.4),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=masonry,
        child=gate,
        origin=Origin(xyz=(0.472, 0.0, 2.13)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.25, lower=0.0, upper=GATE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    masonry = object_model.get_part("masonry")
    wheel = object_model.get_part("wheel")
    gate = object_model.get_part("gate_panel")
    axle = object_model.get_articulation("axle_rotation")
    gate_slide = object_model.get_articulation("gate_slide")

    ctx.check(
        "wheel has continuous axle joint",
        axle.articulation_type == ArticulationType.CONTINUOUS and tuple(axle.axis) == (0.0, 1.0, 0.0),
        details=f"type={axle.articulation_type}, axis={axle.axis}",
    )
    ctx.check(
        "gate has vertical prismatic travel",
        gate_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(gate_slide.axis) == (0.0, 0.0, 1.0)
        and gate_slide.motion_limits is not None
        and gate_slide.motion_limits.upper == GATE_TRAVEL,
        details=f"type={gate_slide.articulation_type}, axis={gate_slide.axis}, limits={gate_slide.motion_limits}",
    )

    ctx.expect_gap(
        gate,
        wheel,
        axis="z",
        positive_elem="gate_plate",
        negative_elem="wheel_frame",
        min_gap=0.025,
        name="lowered gate clears wheel rim",
    )
    ctx.expect_overlap(
        gate,
        masonry,
        axes="z",
        elem_a="gate_plate",
        elem_b="guide_upstream_pos",
        min_overlap=0.45,
        name="lowered gate is retained by guide slot",
    )
    ctx.expect_within(
        wheel,
        masonry,
        axes="y",
        inner_elem="axle_shaft",
        outer_elem="flume_lintel",
        margin=0.18,
        name="wheel axle sits between the stone piers",
    )

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: GATE_TRAVEL}):
        raised_pos = ctx.part_world_position(gate)
        ctx.expect_overlap(
            gate,
            masonry,
            axes="z",
            elem_a="gate_plate",
            elem_b="guide_upstream_pos",
            min_overlap=0.20,
            name="raised gate remains captured in guides",
        )
    ctx.check(
        "gate slide raises the panel",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.35,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
