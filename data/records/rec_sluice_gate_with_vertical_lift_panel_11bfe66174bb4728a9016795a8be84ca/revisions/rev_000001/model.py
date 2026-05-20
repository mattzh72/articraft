from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _moved(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _handwheel_mesh():
    outer = superellipse_profile(0.38, 0.38, exponent=2.0, segments=72)
    window = rounded_rect_profile(0.105, 0.105, 0.018, corner_segments=6)
    holes = [
        _moved(window, 0.075, 0.075),
        _moved(window, -0.075, 0.075),
        _moved(window, -0.075, -0.075),
        _moved(window, 0.075, -0.075),
    ]
    # Profile is in local XY and thickness is local Z; rotate so the wheel is
    # a vertical handwheel whose axis is local Y.
    return ExtrudeWithHolesGeometry(outer, holes, height=0.050, center=True).rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stormwater_sluice_gate")

    concrete = model.material("weathered_concrete", rgba=(0.46, 0.48, 0.47, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.60, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    gate_blue = model.material("painted_gate_blue", rgba=(0.08, 0.22, 0.36, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.10, 1.0))
    grease_black = model.material("grease_black", rgba=(0.03, 0.03, 0.025, 1.0))

    frame = model.part("channel_frame")
    # Flood-control scale concrete and steel channel: roughly 4 m wide and 5 m
    # tall, with an open rectangular waterway and raised guide extensions.
    frame.visual(Box((4.15, 0.72, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=concrete, name="sill")
    frame.visual(Box((0.42, 0.70, 3.35)), origin=Origin(xyz=(-1.78, 0.0, 1.82)), material=concrete, name="pier_0")
    frame.visual(Box((0.42, 0.70, 3.35)), origin=Origin(xyz=(1.78, 0.0, 1.82)), material=concrete, name="pier_1")
    frame.visual(Box((4.15, 0.72, 0.46)), origin=Origin(xyz=(0.0, 0.0, 3.48)), material=concrete, name="top_beam")
    frame.visual(Box((4.35, 1.80, 0.12)), origin=Origin(xyz=(0.0, 0.55, 0.06)), material=concrete, name="channel_floor")
    frame.visual(Box((0.20, 1.80, 1.10)), origin=Origin(xyz=(-2.05, 0.55, 0.62)), material=concrete, name="wing_wall_0")
    frame.visual(Box((0.20, 1.80, 1.10)), origin=Origin(xyz=(2.05, 0.55, 0.62)), material=concrete, name="wing_wall_1")

    # Stainless C-channel guides beside the opening.  The front/back retaining
    # lips leave clearance for the sliding gate leaf but read as real tracks.
    frame.visual(Box((0.15, 0.38, 4.72)), origin=Origin(xyz=(-1.34, 0.0, 2.46)), material=galvanized, name="guide_web_0")
    frame.visual(Box((0.13, 0.065, 4.72)), origin=Origin(xyz=(-1.26, -0.205, 2.46)), material=galvanized, name="front_lip_0")
    frame.visual(Box((0.13, 0.065, 4.72)), origin=Origin(xyz=(-1.26, 0.205, 2.46)), material=galvanized, name="rear_lip_0")
    frame.visual(Box((0.28, 0.42, 0.16)), origin=Origin(xyz=(-1.34, 0.0, 4.88)), material=galvanized, name="guide_cap_0")
    frame.visual(Box((0.15, 0.38, 4.72)), origin=Origin(xyz=(1.34, 0.0, 2.46)), material=galvanized, name="guide_web_1")
    frame.visual(Box((0.13, 0.065, 4.72)), origin=Origin(xyz=(1.26, -0.205, 2.46)), material=galvanized, name="front_lip_1")
    frame.visual(Box((0.13, 0.065, 4.72)), origin=Origin(xyz=(1.26, 0.205, 2.46)), material=galvanized, name="rear_lip_1")
    frame.visual(Box((0.28, 0.42, 0.16)), origin=Origin(xyz=(1.34, 0.0, 4.88)), material=galvanized, name="guide_cap_1")

    # Fixed ladder-like gauge marks and a small cross beam emphasize scale and
    # the vertical slide travel.
    frame.visual(Box((2.94, 0.10, 0.16)), origin=Origin(xyz=(0.0, -0.31, 3.24)), material=galvanized, name="lintel_edge")
    frame.visual(Box((0.055, 0.060, 2.75)), origin=Origin(xyz=(-1.56, -0.365, 1.60)), material=safety_yellow, name="staff_gauge")
    for i, z in enumerate((0.55, 0.95, 1.35, 1.75, 2.15, 2.55)):
        frame.visual(Box((0.18, 0.040, 0.035)), origin=Origin(xyz=(-1.56, -0.39, z)), material=grease_black, name=f"gauge_tick_{i}")

    panel = model.part("lift_panel")
    panel.visual(Box((2.22, 0.12, 2.55)), origin=Origin(xyz=(0.0, 0.0, 1.275)), material=gate_blue, name="gate_leaf")
    panel.visual(Box((2.16, 0.08, 0.13)), origin=Origin(xyz=(0.0, -0.098, 0.50)), material=galvanized, name="stiffener_0")
    panel.visual(Box((2.16, 0.08, 0.13)), origin=Origin(xyz=(0.0, -0.098, 1.28)), material=galvanized, name="stiffener_1")
    panel.visual(Box((2.16, 0.08, 0.13)), origin=Origin(xyz=(0.0, -0.098, 2.06)), material=galvanized, name="stiffener_2")
    panel.visual(Box((0.13, 0.08, 2.36)), origin=Origin(xyz=(-0.68, -0.098, 1.28)), material=galvanized, name="vertical_rib_0")
    panel.visual(Box((0.13, 0.08, 2.36)), origin=Origin(xyz=(0.68, -0.098, 1.28)), material=galvanized, name="vertical_rib_1")
    panel.visual(Box((0.22, 0.38, 0.18)), origin=Origin(xyz=(0.0, -0.24, 2.55)), material=galvanized, name="lift_yoke")
    panel.visual(Cylinder(radius=0.036, length=1.20), origin=Origin(xyz=(0.0, -0.405, 3.15)), material=dark_steel, name="lift_screw")
    panel.visual(Box((0.16, 0.09, 0.10)), origin=Origin(xyz=(-1.16, 0.0, 1.25)), material=dark_steel, name="side_shoe_0")
    panel.visual(Box((0.16, 0.09, 0.10)), origin=Origin(xyz=(1.16, 0.0, 1.25)), material=dark_steel, name="side_shoe_1")

    housing = model.part("operator_housing")
    housing.visual(Box((0.76, 0.28, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material=dark_steel, name="mounting_foot")
    housing.visual(Box((0.64, 0.34, 0.43)), origin=Origin(xyz=(0.0, 0.0, 0.295)), material=galvanized, name="gear_case")
    housing.visual(Box((0.40, 0.10, 0.10)), origin=Origin(xyz=(0.0, -0.22, 0.34)), material=dark_steel, name="input_boss")
    housing.visual(
        Cylinder(radius=0.055, length=0.30),
        origin=Origin(xyz=(0.0, -0.31, 0.34), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="input_shaft",
    )
    housing.visual(
        Cylinder(radius=0.052, length=1.30),
        origin=Origin(xyz=(-0.65, 0.0, 0.35), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="output_shaft",
    )
    housing.visual(Box((0.22, 0.22, 0.18)), origin=Origin(xyz=(-1.25, -0.03, 0.34)), material=galvanized, name="lift_nut_box")
    housing.visual(Box((0.20, 0.28, 0.22)), origin=Origin(xyz=(-0.34, -0.30, 0.60)), material=dark_steel, name="pawl_lug")
    housing.visual(Box((0.35, 0.06, 0.12)), origin=Origin(xyz=(-0.13, 0.20, 0.14)), material=dark_steel, name="rear_stiffener")

    handwheel = model.part("handwheel")
    handwheel.visual(mesh_from_geometry(_handwheel_mesh(), "handwheel_spoked_disk"), material=safety_yellow, name="spoked_disk")
    handwheel.visual(
        Cylinder(radius=0.075, length=0.12),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    handwheel.visual(
        Cylinder(radius=0.026, length=0.17),
        origin=Origin(xyz=(0.125, -0.085, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="crank_pin",
    )
    handwheel.visual(
        Cylinder(radius=0.045, length=0.13),
        origin=Origin(xyz=(0.125, -0.215, 0.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grease_black,
        name="spinner_grip",
    )

    pawl = model.part("pawl")
    pawl.visual(
        Cylinder(radius=0.047, length=0.075),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    pawl.visual(Box((0.10, 0.045, 0.075)), origin=Origin(xyz=(0.035, 0.0, -0.035)), material=dark_steel, name="root_block")
    pawl.visual(Box((0.055, 0.045, 0.26)), origin=Origin(xyz=(0.070, 0.0, -0.155), rpy=(0.0, -0.22, 0.0)), material=dark_steel, name="lever_blade")
    pawl.visual(Box((0.12, 0.045, 0.055)), origin=Origin(xyz=(0.145, 0.0, -0.285), rpy=(0.0, -0.22, 0.0)), material=dark_steel, name="locking_tooth")

    model.articulation(
        "panel_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85000.0, velocity=0.12, lower=0.0, upper=2.10),
    )
    model.articulation(
        "housing_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(1.12, -0.49, 3.71)),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=handwheel,
        origin=Origin(xyz=(0.0, -0.50, 0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=6.0),
    )
    model.articulation(
        "pawl_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pawl,
        origin=Origin(xyz=(-0.405, -0.475, 0.625)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.45, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("channel_frame")
    panel = object_model.get_part("lift_panel")
    housing = object_model.get_part("operator_housing")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("pawl")
    panel_slide = object_model.get_articulation("panel_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")
    pawl_pivot = object_model.get_articulation("pawl_pivot")

    ctx.allow_overlap(
        handwheel,
        housing,
        elem_a="hub",
        elem_b="input_shaft",
        reason="The handwheel hub is intentionally seated over the round gearbox input shaft.",
    )

    ctx.expect_gap(panel, frame, axis="z", min_gap=-0.001, max_gap=0.002, positive_elem="gate_leaf", negative_elem="sill", name="closed panel bears on sill")
    ctx.expect_gap(panel, frame, axis="x", min_gap=0.015, max_gap=0.040, positive_elem="side_shoe_0", negative_elem="guide_web_0", name="left guide shoe has running clearance")
    ctx.expect_gap(frame, panel, axis="x", min_gap=0.015, max_gap=0.040, positive_elem="guide_web_1", negative_elem="side_shoe_1", name="right guide shoe has running clearance")
    ctx.expect_overlap(panel, frame, axes="yz", elem_a="side_shoe_0", elem_b="guide_web_0", min_overlap=0.075, name="left shoe stays in channel throat")
    ctx.expect_overlap(panel, frame, axes="yz", elem_a="side_shoe_1", elem_b="guide_web_1", min_overlap=0.075, name="right shoe stays in channel throat")
    ctx.expect_contact(housing, frame, elem_a="mounting_foot", elem_b="top_beam", contact_tol=0.002, name="gearbox foot is seated on top beam")
    ctx.expect_contact(handwheel, housing, elem_a="hub", elem_b="input_shaft", contact_tol=0.010, name="handwheel hub is on input shaft")
    ctx.expect_contact(pawl, housing, elem_a="pivot_pin", elem_b="pawl_lug", contact_tol=0.010, name="pawl pivots on housing lug")

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({panel_slide: 2.10, wheel_spin: math.pi / 2.0, pawl_pivot: 0.35}):
        raised_pos = ctx.part_world_position(panel)
        ctx.expect_overlap(panel, frame, axes="z", elem_a="side_shoe_0", elem_b="guide_web_0", min_overlap=0.08, name="raised gate remains captured in guides")

    ctx.check(
        "panel slide raises gate leaf",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 2.0,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
