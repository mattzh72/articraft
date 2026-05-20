from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stormwater_sluice_gate")

    concrete = model.material("weathered_concrete", rgba=(0.48, 0.50, 0.48, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.18, 0.20, 0.21, 1.0))
    blue_steel = model.material("painted_gate_steel", rgba=(0.05, 0.22, 0.34, 1.0))
    hazard_yellow = model.material("crank_yellow", rgba=(0.95, 0.70, 0.10, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("channel_frame")

    # Infrastructure-scale concrete portal around a stormwater channel.
    frame.visual(Box((4.40, 0.70, 0.25)), origin=Origin(xyz=(0.0, 0.0, 0.125)), material=concrete, name="sill")
    frame.visual(Box((0.60, 0.70, 3.00)), origin=Origin(xyz=(-1.90, 0.0, 1.50)), material=concrete, name="side_pier_0")
    frame.visual(Box((0.60, 0.70, 3.00)), origin=Origin(xyz=(1.90, 0.0, 1.50)), material=concrete, name="side_pier_1")
    frame.visual(Box((4.40, 0.70, 0.40)), origin=Origin(xyz=(0.0, 0.0, 2.95)), material=concrete, name="top_beam")

    # Steel guide channels bolted to the front of the piers.  The sliding plate
    # runs in the clear space between these cheeks.
    for x, guide_name, retainer_name, bumper_name in [
        (-1.54, "guide_web_0", "front_retainer_0", "bottom_guide_bumper_0"),
        (1.54, "guide_web_1", "front_retainer_1", "bottom_guide_bumper_1"),
    ]:
        frame.visual(Box((0.12, 0.26, 2.65)), origin=Origin(xyz=(x, -0.46, 1.55)), material=dark_steel, name=guide_name)
        frame.visual(Box((0.08, 0.05, 2.65)), origin=Origin(xyz=(x, -0.575, 1.55)), material=dark_steel, name=retainer_name)
        frame.visual(Box((0.20, 0.035, 0.18)), origin=Origin(xyz=(x, -0.61, 0.35)), material=rubber, name=bumper_name)

    frame.visual(Box((3.15, 0.05, 0.07)), origin=Origin(xyz=(0.0, -0.565, 0.285)), material=rubber, name="bottom_seal_seat")

    # Gearbox housing mounted directly on the top beam and to one side of the
    # lift path.  The bottom face sits on the concrete beam, not floating above it.
    frame.visual(Box((0.70, 0.50, 0.65)), origin=Origin(xyz=(1.85, -0.60, 3.475)), material=dark_steel, name="gearbox_body")
    frame.visual(Box((0.86, 0.60, 0.08)), origin=Origin(xyz=(1.85, -0.60, 3.19)), material=dark_steel, name="gearbox_base_flange")
    frame.visual(Box((0.54, 0.035, 0.44)), origin=Origin(xyz=(1.84, -0.867, 3.48)), material=dark_steel, name="front_cover_plate")

    # Fixed input bearing boss on the gearbox side.  The wheel part rotates on
    # this axis and its hub bears against the boss face.
    frame.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(2.25, -0.60, 3.48), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="input_bearing_boss",
    )
    frame.visual(
        Cylinder(radius=0.06, length=0.12),
        origin=Origin(xyz=(2.26, -0.60, 3.48), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="input_shaft_stub",
    )

    # Stationary inspection-door hinge support, attached to the housing face.
    frame.visual(Box((0.04, 0.024, 0.34)), origin=Origin(xyz=(1.605, -0.842, 3.48)), material=dark_steel, name="door_hinge_leaf")
    frame.visual(
        Cylinder(radius=0.017, length=0.08),
        origin=Origin(xyz=(1.63, -0.873, 3.48)),
        material=dark_steel,
        name="door_hinge_knuckle",
    )

    # Cover and flange bolt heads.
    for i, (x, z) in enumerate([(1.58, 3.27), (2.12, 3.27), (1.58, 3.69), (2.12, 3.69)]):
        frame.visual(
            Cylinder(radius=0.035, length=0.018),
            origin=Origin(xyz=(x, -0.878, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"cover_bolt_{i}",
        )
    for i, x in enumerate([1.50, 1.75, 1.95, 2.20]):
        frame.visual(
            Cylinder(radius=0.03, length=0.035),
            origin=Origin(xyz=(x, -0.33, 3.23)),
            material=dark_steel,
            name=f"flange_bolt_{i}",
        )

    gate_panel = model.part("lift_panel")
    gate_panel.visual(Box((2.90, 0.10, 2.20)), origin=Origin(), material=blue_steel, name="panel_plate")
    gate_panel.visual(Box((2.70, 0.06, 0.12)), origin=Origin(xyz=(0.0, -0.08, 0.78)), material=dark_steel, name="top_stiffener")
    gate_panel.visual(Box((2.70, 0.06, 0.12)), origin=Origin(xyz=(0.0, -0.08, 0.05)), material=dark_steel, name="middle_stiffener")
    gate_panel.visual(Box((2.70, 0.06, 0.12)), origin=Origin(xyz=(0.0, -0.08, -0.72)), material=dark_steel, name="bottom_stiffener")
    gate_panel.visual(Box((0.10, 0.06, 2.15)), origin=Origin(xyz=(-1.43, -0.08, 0.0)), material=dark_steel, name="edge_shoe_0")
    gate_panel.visual(Box((0.10, 0.06, 2.15)), origin=Origin(xyz=(1.43, -0.08, 0.0)), material=dark_steel, name="edge_shoe_1")
    gate_panel.visual(Box((0.80, 0.07, 0.16)), origin=Origin(xyz=(0.0, -0.08, 1.16)), material=dark_steel, name="lifting_lug_base")
    gate_panel.visual(
        Cylinder(radius=0.09, length=0.08),
        origin=Origin(xyz=(0.0, -0.10, 1.29), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lifting_pin_eye",
    )

    crank_wheel = model.part("crank_wheel")
    crank_wheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.35, tube=0.025, radial_segments=18, tubular_segments=48), "crank_wheel_ring"),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hazard_yellow,
        name="wheel_ring",
    )
    crank_wheel.visual(
        Cylinder(radius=0.12, length=0.24),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hazard_yellow,
        name="wheel_hub",
    )
    crank_wheel.visual(Box((0.045, 0.72, 0.035)), origin=Origin(xyz=(0.22, 0.0, 0.0)), material=hazard_yellow, name="spoke_cross")
    crank_wheel.visual(Box((0.045, 0.035, 0.72)), origin=Origin(xyz=(0.22, 0.0, 0.0)), material=hazard_yellow, name="spoke_upright")
    crank_wheel.visual(
        Cylinder(radius=0.04, length=0.22),
        origin=Origin(xyz=(0.255, 0.0, -0.35), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rim_grip",
    )

    inspection_door = model.part("inspection_door")
    inspection_door.visual(Box((0.34, 0.04, 0.28)), origin=Origin(xyz=(0.187, -0.02, 0.0)), material=dark_steel, name="door_plate")
    inspection_door.visual(Box((0.18, 0.025, 0.07)), origin=Origin(xyz=(0.237, -0.045, -0.02)), material=dark_steel, name="pull_latch")
    inspection_door.visual(
        Cylinder(radius=0.017, length=0.08),
        origin=Origin(xyz=(0.0, -0.023, 0.095)),
        material=dark_steel,
        name="hinge_knuckle_0",
    )
    inspection_door.visual(
        Cylinder(radius=0.017, length=0.08),
        origin=Origin(xyz=(0.0, -0.023, -0.095)),
        material=dark_steel,
        name="hinge_knuckle_1",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_panel,
        origin=Origin(xyz=(0.0, -0.47, 1.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80000.0, velocity=0.15, lower=0.0, upper=1.25),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_wheel,
        origin=Origin(xyz=(2.30, -0.60, 3.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5),
    )
    model.articulation(
        "frame_to_inspection_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=inspection_door,
        origin=Origin(xyz=(1.63, -0.8845, 3.48)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("channel_frame")
    panel = object_model.get_part("lift_panel")
    crank = object_model.get_part("crank_wheel")
    door = object_model.get_part("inspection_door")
    panel_slide = object_model.get_articulation("frame_to_panel")
    door_hinge = object_model.get_articulation("frame_to_inspection_door")

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="input_shaft_stub",
        elem_b="wheel_hub",
        reason="The gearbox input shaft is intentionally seated inside the crank wheel hub bore.",
    )
    ctx.expect_within(
        frame,
        crank,
        axes="yz",
        inner_elem="input_shaft_stub",
        outer_elem="wheel_hub",
        margin=0.002,
        name="input shaft is centered in crank hub",
    )
    ctx.expect_overlap(
        frame,
        crank,
        axes="x",
        elem_a="input_shaft_stub",
        elem_b="wheel_hub",
        min_overlap=0.015,
        name="crank hub remains seated on input shaft",
    )
    ctx.expect_within(
        panel,
        frame,
        axes="x",
        inner_elem="panel_plate",
        outer_elem="bottom_seal_seat",
        margin=0.15,
        name="panel spans the channel opening",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="panel_plate",
        negative_elem="sill",
        max_gap=0.06,
        max_penetration=0.0,
        name="closed panel seats above sill",
    )
    ctx.expect_contact(
        crank,
        frame,
        elem_a="wheel_hub",
        elem_b="input_bearing_boss",
        contact_tol=0.002,
        name="crank hub bears on gearbox boss",
    )
    ctx.expect_contact(
        door,
        frame,
        elem_a="door_plate",
        elem_b="front_cover_plate",
        contact_tol=0.002,
        name="inspection door closes on housing face",
    )

    rest_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_slide: 1.25}):
        raised_aabb = ctx.part_world_aabb(panel)
        ctx.expect_gap(
            panel,
            frame,
            axis="x",
            positive_elem="panel_plate",
            negative_elem="guide_web_0",
            min_gap=0.02,
            name="raised panel clears guide side",
        )
    ctx.check(
        "panel raises vertically in side channels",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[0][2] > rest_aabb[0][2] + 1.0,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    closed_pos = ctx.part_world_position(door)
    with ctx.pose({door_hinge: 1.2}):
        open_pos = ctx.part_world_position(door)
        ctx.expect_gap(
            frame,
            door,
            axis="y",
            positive_elem="gearbox_body",
            negative_elem="door_plate",
            min_gap=0.01,
            name="inspection door swings outward from housing",
        )
    ctx.check(
        "inspection door remains hinged to gearbox",
        closed_pos is not None and open_pos is not None,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
