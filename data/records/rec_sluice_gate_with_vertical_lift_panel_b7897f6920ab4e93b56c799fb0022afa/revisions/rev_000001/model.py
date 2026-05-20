from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _sluice_frame_mesh():
    """One continuous concrete/painted-metal frame with a real through opening."""
    outer_width = 1.08
    depth = 0.18
    outer_height = 1.12
    opening_width = 0.72
    opening_height = 0.72
    opening_center_z = 0.50

    frame = (
        cq.Workplane("XY")
        .box(outer_width, depth, outer_height)
        .translate((0.0, 0.0, outer_height / 2.0))
    )
    cutout = (
        cq.Workplane("XY")
        .box(opening_width, depth + 0.04, opening_height)
        .translate((0.0, 0.0, opening_center_z))
    )
    return frame.cut(cutout)


def _handwheel_mesh():
    """A compact annular handwheel with four flat spokes and a raised hub."""
    rim = (
        cq.Workplane("XY")
        .circle(0.125)
        .circle(0.103)
        .extrude(0.014)
        .translate((0.0, 0.0, -0.007))
    )
    spoke_x = (
        cq.Workplane("XY")
        .rect(0.215, 0.018)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.008))
    )
    spoke_z = (
        cq.Workplane("XY")
        .rect(0.018, 0.215)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.008))
    )
    hub = (
        cq.Workplane("XY")
        .circle(0.038)
        .extrude(0.050)
        .translate((0.0, 0.0, -0.025))
    )
    return rim.union(spoke_x).union(spoke_z).union(hub)


def _stem_sleeve_mesh():
    """A short hollow guide sleeve for the rising lift stem."""
    return (
        cq.Workplane("XY")
        .circle(0.028)
        .circle(0.017)
        .extrude(0.090)
        .translate((0.0, 0.0, -0.045))
    )


def _shaft_boss_mesh():
    """A hollow front bearing boss around the handwheel shaft."""
    return (
        cq.Workplane("XY")
        .circle(0.038)
        .circle(0.021)
        .extrude(0.038)
        .translate((0.0, 0.0, -0.019))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="irrigation_sluice_gate")

    weathered_green = Material("weathered_green", rgba=(0.10, 0.34, 0.25, 1.0))
    dark_plate = Material("dark_gate_plate", rgba=(0.05, 0.08, 0.07, 1.0))
    bare_steel = Material("bare_steel", rgba=(0.58, 0.60, 0.56, 1.0))
    red_paint = Material("red_handwheel", rgba=(0.72, 0.08, 0.05, 1.0))
    blackened = Material("blackened_pawl", rgba=(0.02, 0.02, 0.018, 1.0))
    brass = Material("oiled_bronze", rgba=(0.72, 0.52, 0.23, 1.0))
    concrete = Material("cast_concrete", rgba=(0.50, 0.53, 0.48, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_sluice_frame_mesh(), "framed_opening", tolerance=0.001),
        material=concrete,
        name="framed_opening",
    )
    # Front-mounted channel guides: they are bolted to the frame beside the clear opening.
    frame.visual(
        Box((0.050, 0.050, 0.920)),
        origin=Origin(xyz=(-0.385, -0.112, 0.560)),
        material=weathered_green,
        name="guide_0",
    )
    frame.visual(
        Box((0.030, 0.030, 0.850)),
        origin=Origin(xyz=(-0.3619, -0.144, 0.560)),
        material=weathered_green,
        name="guide_0_front_lip",
    )
    frame.visual(
        Box((0.050, 0.050, 0.920)),
        origin=Origin(xyz=(0.385, -0.112, 0.560)),
        material=weathered_green,
        name="guide_1",
    )
    frame.visual(
        Box((0.030, 0.030, 0.850)),
        origin=Origin(xyz=(0.3619, -0.144, 0.560)),
        material=weathered_green,
        name="guide_1_front_lip",
    )
    frame.visual(
        Box((0.90, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.112, 0.110)),
        material=weathered_green,
        name="lower_guide_tie",
    )
    frame.visual(
        Box((0.90, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.112, 1.030)),
        material=weathered_green,
        name="upper_guide_tie",
    )

    gate = model.part("gate_panel")
    gate.visual(
        Box((0.660, 0.035, 0.720)),
        origin=Origin(),
        material=dark_plate,
        name="rising_plate",
    )
    gate.visual(
        Box((0.019, 0.024, 0.680)),
        origin=Origin(xyz=(-0.3375, -0.020, 0.000)),
        material=bare_steel,
        name="guide_shoe_0",
    )
    gate.visual(
        Box((0.019, 0.024, 0.680)),
        origin=Origin(xyz=(0.3375, -0.020, 0.000)),
        material=bare_steel,
        name="guide_shoe_1",
    )
    gate.visual(
        Box((0.620, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, 0.250)),
        material=bare_steel,
        name="stiffener_0",
    )
    gate.visual(
        Box((0.620, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, -0.150)),
        material=bare_steel,
        name="stiffener_1",
    )
    gate.visual(
        Box((0.180, 0.120, 0.065)),
        origin=Origin(xyz=(0.0, -0.055, 0.390)),
        material=bare_steel,
        name="lift_clevis",
    )
    gate.visual(
        Cylinder(radius=0.012, length=0.280),
        origin=Origin(xyz=(0.0, -0.080, 0.525)),
        material=bare_steel,
        name="lift_stem",
    )
    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, -0.125, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.12, lower=0.0, upper=0.280),
    )

    housing = model.part("operator_housing")
    housing.visual(
        Box((0.420, 0.220, 0.034)),
        origin=Origin(xyz=(0.0, 0.200, -0.143)),
        material=weathered_green,
        name="mounting_foot",
    )
    housing.visual(
        Box((0.280, 0.125, 0.210)),
        origin=Origin(xyz=(0.0, 0.085, -0.028)),
        material=weathered_green,
        name="gearbox_case",
    )
    housing.visual(
        Box((0.310, 0.145, 0.035)),
        origin=Origin(xyz=(0.0, 0.085, 0.093)),
        material=weathered_green,
        name="case_cap",
    )
    housing.visual(
        mesh_from_cadquery(_shaft_boss_mesh(), "shaft_boss", tolerance=0.0006),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="shaft_boss",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.245, 0.002, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pawl_boss",
    )
    housing.visual(
        Box((0.150, 0.045, 0.060)),
        origin=Origin(xyz=(0.190, 0.012, 0.000)),
        material=weathered_green,
        name="pawl_ear",
    )
    housing.visual(
        mesh_from_cadquery(_stem_sleeve_mesh(), "stem_sleeve", tolerance=0.0006),
        origin=Origin(xyz=(0.0, -0.005, -0.105)),
        material=bare_steel,
        name="stem_socket",
    )
    model.articulation(
        "frame_to_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, -0.200, 1.280)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_mesh(), "handwheel", tolerance=0.0008),
        origin=Origin(xyz=(0.0, -0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_paint,
        name="spoked_wheel",
    )
    handwheel.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="input_shaft",
    )
    handwheel.visual(
        Cylinder(radius=0.033, length=0.008),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="retaining_collar",
    )
    model.articulation(
        "housing_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=handwheel,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0),
    )

    pawl = model.part("locking_pawl")
    pawl.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.0, -0.0195, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_collar",
    )
    pawl.visual(
        Box((0.110, 0.016, 0.022)),
        origin=Origin(xyz=(-0.045, -0.030, -0.026), rpy=(0.0, -0.50, 0.0)),
        material=blackened,
        name="pawl_lever",
    )
    pawl.visual(
        Box((0.028, 0.018, 0.032)),
        origin=Origin(xyz=(-0.092, -0.030, -0.055), rpy=(0.0, -0.50, 0.0)),
        material=blackened,
        name="pawl_tooth",
    )
    model.articulation(
        "housing_to_pawl",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pawl,
        origin=Origin(xyz=(0.245, 0.000, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.55, upper=0.32),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate_panel")
    housing = object_model.get_part("operator_housing")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("locking_pawl")
    gate_slide = object_model.get_articulation("frame_to_gate")
    wheel_spin = object_model.get_articulation("housing_to_handwheel")
    pawl_pivot = object_model.get_articulation("housing_to_pawl")

    ctx.check(
        "sluice has the requested primary joints",
        gate_slide.articulation_type == ArticulationType.PRISMATIC
        and wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and pawl_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"gate={gate_slide.articulation_type}, wheel={wheel_spin.articulation_type}, "
            f"pawl={pawl_pivot.articulation_type}"
        ),
    )

    ctx.expect_gap(
        housing,
        frame,
        axis="z",
        positive_elem="mounting_foot",
        negative_elem="framed_opening",
        max_gap=0.001,
        max_penetration=0.001,
        name="operator housing foot sits on the top crossmember",
    )
    ctx.expect_contact(
        handwheel,
        housing,
        elem_a="retaining_collar",
        elem_b="shaft_boss",
        contact_tol=0.003,
        name="handwheel shaft is supported at the housing boss",
    )
    ctx.expect_contact(
        pawl,
        housing,
        elem_a="pivot_collar",
        elem_b="pawl_boss",
        contact_tol=0.003,
        name="pawl pivot collar is mounted on the housing boss",
    )
    ctx.expect_contact(
        gate,
        frame,
        elem_a="guide_shoe_0",
        elem_b="guide_0_front_lip",
        contact_tol=0.003,
        name="gate shoe bears in one vertical guide",
    )
    ctx.expect_contact(
        gate,
        frame,
        elem_a="guide_shoe_1",
        elem_b="guide_1_front_lip",
        contact_tol=0.003,
        name="gate shoe bears in the opposite vertical guide",
    )

    rest_gate_position = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: 0.280}):
        raised_gate_position = ctx.part_world_position(gate)
        ctx.expect_contact(
            gate,
            frame,
            elem_a="guide_shoe_0",
            elem_b="guide_0_front_lip",
            contact_tol=0.003,
            name="raised gate remains captured by the guide",
        )
    ctx.check(
        "gate panel rises vertically on the prismatic slide",
        rest_gate_position is not None
        and raised_gate_position is not None
        and raised_gate_position[2] > rest_gate_position[2] + 0.25,
        details=f"rest={rest_gate_position}, raised={raised_gate_position}",
    )

    def _elem_center_z(part, elem: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        return (box[0][2] + box[1][2]) / 2.0

    with ctx.pose({pawl_pivot: pawl_pivot.motion_limits.upper}):
        pawl_up_z = _elem_center_z(pawl, "pawl_tooth")
    with ctx.pose({pawl_pivot: pawl_pivot.motion_limits.lower}):
        pawl_down_z = _elem_center_z(pawl, "pawl_tooth")
    ctx.check(
        "locking pawl tooth moves on its short pivot",
        pawl_up_z is not None and pawl_down_z is not None and abs(pawl_up_z - pawl_down_z) > 0.015,
        details=f"upper_z={pawl_up_z}, lower_z={pawl_down_z}",
    )

    return ctx.report()


object_model = build_object_model()
