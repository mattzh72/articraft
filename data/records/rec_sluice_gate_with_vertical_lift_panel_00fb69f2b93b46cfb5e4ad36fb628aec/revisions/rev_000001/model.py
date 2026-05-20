from __future__ import annotations

import math

import cadquery as cq

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


def _handwheel_mesh():
    """Build a spoked industrial handwheel in its local XY plane.

    The mesh's local Z axis is the rotation shaft axis; the visual transform
    rotates that axis onto the gate's front/back Y axis.
    """

    thickness = 0.065
    outer_radius = 0.33
    inner_radius = 0.255
    hub_radius = 0.105
    spoke_width = 0.060

    rim = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(thickness)
    spoke_x = cq.Workplane("XY").rect(2.0 * inner_radius + 0.035, spoke_width).extrude(thickness)
    spoke_z = cq.Workplane("XY").rect(spoke_width, 2.0 * inner_radius + 0.035).extrude(thickness)
    hub = cq.Workplane("XY").circle(hub_radius).extrude(thickness * 1.25)
    wheel = rim.union(spoke_x).union(spoke_z).union(hub).translate((0.0, 0.0, -thickness / 2.0))
    return mesh_from_cadquery(
        wheel,
        "handwheel_mesh",
        tolerance=0.002,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flood_control_sluice_gate")

    concrete = Material("weathered_concrete", color=(0.48, 0.50, 0.49, 1.0))
    galvanized = Material("galvanized_steel", color=(0.54, 0.58, 0.60, 1.0))
    dark_steel = Material("dark_oxide_steel", color=(0.08, 0.09, 0.10, 1.0))
    painted_panel = Material("blue_gray_gate_plate", color=(0.22, 0.34, 0.42, 1.0))
    safety_yellow = Material("safety_yellow", color=(0.95, 0.68, 0.08, 1.0))
    pawl_red = Material("lockout_red", color=(0.70, 0.10, 0.06, 1.0))
    rubber = Material("black_rubber_seal", color=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")
    # Civil-scale concrete/steel portal frame: roughly 3.3 m wide, 3.75 m high.
    frame.visual(Box((0.45, 0.65, 3.75)), origin=Origin(xyz=(-1.425, 0.0, 1.875)), material=concrete, name="side_pier_0")
    frame.visual(Box((0.45, 0.65, 3.75)), origin=Origin(xyz=(1.425, 0.0, 1.875)), material=concrete, name="side_pier_1")
    frame.visual(Box((3.30, 0.65, 0.35)), origin=Origin(xyz=(0.0, 0.0, 0.175)), material=concrete, name="bottom_sill")
    frame.visual(Box((3.30, 0.78, 0.55)), origin=Origin(xyz=(0.0, 0.0, 3.475)), material=concrete, name="top_beam")
    # Dark steel side guide channels stand proud on the front face and continue
    # above the opening so the lifted leaf remains captured.
    frame.visual(Box((0.12, 0.22, 3.35)), origin=Origin(xyz=(-1.26, 0.43, 2.025)), material=dark_steel, name="guide_left")
    frame.visual(Box((0.12, 0.22, 3.35)), origin=Origin(xyz=(1.26, 0.43, 2.025)), material=dark_steel, name="guide_right")
    frame.visual(Box((2.75, 0.12, 0.12)), origin=Origin(xyz=(0.0, 0.29, 0.39)), material=dark_steel, name="guide_sill")
    frame.visual(Box((2.55, 0.10, 0.12)), origin=Origin(xyz=(0.0, 0.29, 3.66)), material=dark_steel, name="guide_header")

    lift_panel = model.part("lift_panel")
    lift_panel.visual(Box((2.20, 0.16, 2.85)), origin=Origin(), material=painted_panel, name="gate_plate")
    # Reinforcing ribs and edge seals are proud on the front face.
    for idx, z in enumerate((-0.95, 0.0, 0.95)):
        lift_panel.visual(Box((2.05, 0.075, 0.13)), origin=Origin(xyz=(0.0, 0.113, z)), material=galvanized, name=f"horizontal_rib_{idx}")
    for idx, x in enumerate((-0.72, 0.0, 0.72)):
        lift_panel.visual(Box((0.13, 0.075, 2.55)), origin=Origin(xyz=(x, 0.113, 0.0)), material=galvanized, name=f"vertical_rib_{idx}")
    lift_panel.visual(Box((2.18, 0.030, 0.10)), origin=Origin(xyz=(0.0, -0.096, -1.41)), material=rubber, name="bottom_seal")
    lift_panel.visual(Box((0.11, 0.035, 2.75)), origin=Origin(xyz=(-1.14, -0.090, 0.0)), material=rubber, name="edge_seal_0")
    lift_panel.visual(Box((0.11, 0.035, 2.75)), origin=Origin(xyz=(1.14, -0.090, 0.0)), material=rubber, name="edge_seal_1")
    lift_panel.visual(Box((0.11, 0.12, 2.62)), origin=Origin(xyz=(-1.145, 0.000, 0.0)), material=dark_steel, name="sliding_shoe_0")
    lift_panel.visual(Box((0.11, 0.12, 2.62)), origin=Origin(xyz=(1.145, 0.000, 0.0)), material=dark_steel, name="sliding_shoe_1")
    # A visible lift stem rises from the reinforced plate toward the manual operator.
    lift_panel.visual(Cylinder(radius=0.045, length=0.48), origin=Origin(xyz=(0.0, 0.0, 1.665)), material=dark_steel, name="lift_stem")
    lift_panel.visual(Box((0.32, 0.14, 0.16)), origin=Origin(xyz=(0.0, 0.045, 1.47)), material=galvanized, name="stem_lug")
    for idx, (x, z) in enumerate(((-0.88, -1.05), (0.88, -1.05), (-0.88, 1.05), (0.88, 1.05))):
        lift_panel.visual(
            Cylinder(radius=0.035, length=0.030),
            origin=Origin(xyz=(x, 0.092, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bolt_head_{idx}",
        )

    operator_box = model.part("operator_box")
    operator_box.visual(Box((1.08, 0.70, 0.12)), origin=Origin(xyz=(0.0, 0.0, -0.405)), material=dark_steel, name="base_flange")
    operator_box.visual(Box((0.86, 0.55, 0.66)), origin=Origin(xyz=(0.0, 0.0, -0.015)), material=galvanized, name="housing_body")
    operator_box.visual(Box((0.94, 0.60, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.355)), material=dark_steel, name="housing_cap")
    operator_box.visual(
        Cylinder(radius=0.060, length=0.295),
        origin=Origin(xyz=(0.0, 0.4225, 0.145), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handwheel_shaft",
    )
    operator_box.visual(
        Cylinder(radius=0.058, length=0.390),
        origin=Origin(xyz=(0.52, 0.465, 0.185), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pawl_pivot",
    )
    operator_box.visual(
        Box((0.22, 0.22, 0.17)),
        origin=Origin(xyz=(0.49, 0.360, 0.185)),
        material=dark_steel,
        name="pawl_bracket",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        _handwheel_mesh(),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="wheel_rim_spokes",
    )
    handwheel.visual(
        Cylinder(radius=0.075, length=0.10),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_hub_cap",
    )

    locking_pawl = model.part("locking_pawl")
    locking_pawl.visual(
        Cylinder(radius=0.090, length=0.055),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pawl_red,
        name="pawl_hub",
    )
    locking_pawl.visual(Box((0.080, 0.050, 0.44)), origin=Origin(xyz=(0.0, 0.0, -0.245)), material=pawl_red, name="pawl_lever")
    locking_pawl.visual(Box((0.14, 0.055, 0.08)), origin=Origin(xyz=(-0.020, 0.0, -0.495), rpy=(0.0, 0.35, 0.0)), material=dark_steel, name="pawl_tooth")

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_panel,
        origin=Origin(xyz=(0.0, 0.50, 1.775)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.08, lower=0.0, upper=0.75),
    )
    model.articulation(
        "operator_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=operator_box,
        origin=Origin(xyz=(0.0, 0.0, 4.215)),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=operator_box,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.620, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5),
    )
    model.articulation(
        "pawl_pivot",
        ArticulationType.REVOLUTE,
        parent=operator_box,
        child=locking_pawl,
        origin=Origin(xyz=(0.52, 0.6875, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("lift_panel")
    operator = object_model.get_part("operator_box")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("locking_pawl")
    gate_slide = object_model.get_articulation("gate_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")
    pawl_joint = object_model.get_articulation("pawl_pivot")

    ctx.expect_gap(
        operator,
        frame,
        axis="z",
        positive_elem="base_flange",
        negative_elem="top_beam",
        max_gap=0.001,
        max_penetration=0.00001,
        name="operator housing base sits on top beam",
    )
    ctx.expect_within(
        operator,
        frame,
        axes="xy",
        inner_elem="base_flange",
        outer_elem="top_beam",
        margin=0.0,
        name="operator housing is centered on the beam footprint",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="z",
        elem_a="gate_plate",
        elem_b="guide_left",
        min_overlap=2.7,
        name="closed panel is retained by left side guide",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="z",
        elem_a="gate_plate",
        elem_b="guide_right",
        min_overlap=2.7,
        name="closed panel is retained by right side guide",
    )
    ctx.expect_gap(
        handwheel,
        operator,
        axis="y",
        positive_elem="front_hub_cap",
        negative_elem="handwheel_shaft",
        max_gap=0.008,
        max_penetration=0.00001,
        name="handwheel seats just in front of its shaft",
    )
    ctx.expect_gap(
        pawl,
        operator,
        axis="y",
        positive_elem="pawl_hub",
        negative_elem="pawl_pivot",
        max_gap=0.010,
        max_penetration=0.00001,
        name="locking pawl hub is mounted on short pivot",
    )

    ctx.check(
        "handwheel uses continuous rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_spin.articulation_type}",
    )

    rest_panel_pos = ctx.part_world_position(panel)
    with ctx.pose({gate_slide: 0.75}):
        raised_panel_pos = ctx.part_world_position(panel)
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="gate_plate",
            elem_b="guide_left",
            min_overlap=2.0,
            name="raised panel remains captured in the left guide",
        )
    ctx.check(
        "lift panel slides upward",
        rest_panel_pos is not None
        and raised_panel_pos is not None
        and raised_panel_pos[2] > rest_panel_pos[2] + 0.70,
        details=f"rest={rest_panel_pos}, raised={raised_panel_pos}",
    )

    rest_pawl_aabb = ctx.part_element_world_aabb(pawl, elem="pawl_tooth")
    with ctx.pose({pawl_joint: 0.55}):
        moved_pawl_aabb = ctx.part_element_world_aabb(pawl, elem="pawl_tooth")
    rest_tip_x = None if rest_pawl_aabb is None else 0.5 * (rest_pawl_aabb[0][0] + rest_pawl_aabb[1][0])
    moved_tip_x = None if moved_pawl_aabb is None else 0.5 * (moved_pawl_aabb[0][0] + moved_pawl_aabb[1][0])
    ctx.check(
        "locking pawl rotates beside handwheel",
        rest_tip_x is not None and moved_tip_x is not None and abs(moved_tip_x - rest_tip_x) > 0.10,
        details=f"rest_tip_x={rest_tip_x}, moved_tip_x={moved_tip_x}",
    )

    return ctx.report()


object_model = build_object_model()
