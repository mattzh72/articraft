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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_channel_sluice_gate")

    concrete = Material("weathered_concrete", rgba=(0.48, 0.48, 0.43, 1.0))
    dark_steel = Material("dark_galvanized_steel", rgba=(0.23, 0.25, 0.26, 1.0))
    gate_steel = Material("wet_blued_steel", rgba=(0.10, 0.16, 0.20, 1.0))
    zinc = Material("worn_zinc_edges", rgba=(0.55, 0.58, 0.56, 1.0))
    black = Material("black_handwheel", rgba=(0.015, 0.017, 0.018, 1.0))
    brass = Material("brass_fasteners", rgba=(0.74, 0.58, 0.25, 1.0))

    frame = model.part("frame")
    frame.visual(Box((0.35, 0.38, 2.30)), origin=Origin(xyz=(-0.98, 0.04, 1.15)), material=concrete, name="pier_0")
    frame.visual(Box((0.35, 0.38, 2.30)), origin=Origin(xyz=(0.98, 0.04, 1.15)), material=concrete, name="pier_1")
    frame.visual(Box((2.30, 0.42, 0.25)), origin=Origin(xyz=(0.0, 0.04, 0.125)), material=concrete, name="sill")
    frame.visual(Box((2.30, 0.42, 0.30)), origin=Origin(xyz=(0.0, 0.04, 2.10)), material=concrete, name="top_lintel")
    frame.visual(Box((1.92, 0.08, 0.07)), origin=Origin(xyz=(0.0, -0.12, 0.285)), material=dark_steel, name="steel_sill")
    frame.visual(Box((1.95, 0.06, 0.10)), origin=Origin(xyz=(0.0, -0.12, 1.94)), material=dark_steel, name="upper_stop")

    for i, x in enumerate((-0.82, 0.82)):
        guide = model.part(f"guide_{i}")
        sign = -1.0 if x < 0.0 else 1.0
        guide.visual(Box((0.035, 0.12, 1.90)), origin=Origin(xyz=(sign * 0.045, 0.0, 0.95)), material=dark_steel, name="channel_web")
        guide.visual(Box((0.14, 0.025, 1.90)), origin=Origin(xyz=(0.0, -0.047, 0.95)), material=dark_steel, name="front_lip")
        guide.visual(Box((0.14, 0.025, 1.90)), origin=Origin(xyz=(0.0, 0.047, 0.95)), material=dark_steel, name="rear_lip")
        for z in (0.24, 0.95, 1.62):
            guide.visual(Box((0.22, 0.08, 0.10)), origin=Origin(xyz=(sign * 0.01, 0.06, z)), material=zinc, name=f"standoff_{z:.2f}")
            guide.visual(Cylinder(0.022, 0.014), origin=Origin(xyz=(sign * 0.075, 0.015, z), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name=f"bolt_{z:.2f}")
        model.articulation(
            f"frame_to_guide_{i}",
            ArticulationType.FIXED,
            parent=frame,
            child=guide,
            origin=Origin(xyz=(x, -0.25, 0.25)),
        )

    lift_plate = model.part("lift_plate")
    lift_plate.visual(Box((1.44, 0.045, 1.48)), origin=Origin(xyz=(0.0, 0.0, 0.74)), material=gate_steel, name="gate_leaf")
    lift_plate.visual(Box((1.32, 0.032, 0.055)), origin=Origin(xyz=(0.0, -0.036, 1.40)), material=zinc, name="top_stiffener")
    lift_plate.visual(Box((1.32, 0.032, 0.055)), origin=Origin(xyz=(0.0, -0.036, 0.18)), material=zinc, name="bottom_stiffener")
    lift_plate.visual(Box((0.055, 0.036, 1.32)), origin=Origin(xyz=(-0.68, -0.036, 0.76)), material=zinc, name="side_wear_0")
    lift_plate.visual(Box((0.055, 0.036, 1.32)), origin=Origin(xyz=(0.68, -0.036, 0.76)), material=zinc, name="side_wear_1")
    lift_plate.visual(Cylinder(0.028, 0.69), origin=Origin(xyz=(0.0, 0.0, 1.825)), material=dark_steel, name="threaded_stem")
    lift_plate.visual(Box((0.22, 0.055, 0.08)), origin=Origin(xyz=(0.0, -0.025, 1.50)), material=dark_steel, name="stem_clevis")
    model.articulation(
        "frame_to_lift_plate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_plate,
        origin=Origin(xyz=(0.0, -0.25, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16000.0, velocity=0.12, lower=0.0, upper=0.45),
    )

    gearbox = model.part("gearbox")
    gearbox.visual(Box((0.20, 0.30, 0.35)), origin=Origin(xyz=(-0.17, 0.0, 0.10)), material=dark_steel, name="housing_left")
    gearbox.visual(Box((0.20, 0.30, 0.35)), origin=Origin(xyz=(0.17, 0.0, 0.10)), material=dark_steel, name="housing_right")
    gearbox.visual(Box((0.50, 0.32, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.30)), material=zinc, name="top_cover")
    gearbox.visual(Cylinder(0.085, 0.16), origin=Origin(xyz=(0.0, 0.0, -0.14)), material=dark_steel, name="screw_sleeve")
    gearbox.visual(Cylinder(0.040, 0.235), origin=Origin(xyz=(0.0, -0.2675, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)), material=zinc, name="handwheel_shaft")
    gearbox.visual(Cylinder(0.088, 0.045), origin=Origin(xyz=(0.0, -0.162, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_steel, name="front_bearing")
    for x in (-0.18, 0.18):
        for y in (-0.09, 0.09):
            gearbox.visual(Box((0.08, 0.08, 0.10)), origin=Origin(xyz=(x, y, -0.05)), material=dark_steel, name=f"mount_foot_{x}_{y}")
    model.articulation(
        "frame_to_gearbox",
        ArticulationType.FIXED,
        parent=frame,
        child=gearbox,
        origin=Origin(xyz=(0.0, -0.28, 2.35)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(Cylinder(0.070, 0.070), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=black, name="hub")
    radius = 0.225
    handwheel.visual(
        mesh_from_geometry(TorusGeometry(radius=radius, tube=0.018, radial_segments=16, tubular_segments=72), "handwheel_rim"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="rim",
    )
    for n, theta in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        handwheel.visual(
            Box((0.34, 0.026, 0.026)),
            origin=Origin(xyz=(0.085 * math.cos(theta), 0.0, 0.085 * math.sin(theta)), rpy=(0.0, -theta, 0.0)),
            material=black,
            name=f"spoke_{n}",
        )
    handwheel.visual(Cylinder(0.026, 0.095), origin=Origin(xyz=(0.0, -0.060, radius), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=black, name="spinner_grip")
    model.articulation(
        "gearbox_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=gearbox,
        child=handwheel,
        origin=Origin(xyz=(0.0, -0.42, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0),
    )

    inspection_door = model.part("inspection_door")
    inspection_door.visual(Box((0.15, 0.024, 0.16)), origin=Origin(xyz=(0.075, -0.012, 0.0)), material=zinc, name="door_panel")
    inspection_door.visual(Cylinder(0.018, 0.19), origin=Origin(xyz=(0.0, -0.020, 0.0)), material=dark_steel, name="hinge_barrel")
    inspection_door.visual(Box((0.045, 0.012, 0.018)), origin=Origin(xyz=(0.12, -0.030, 0.0)), material=brass, name="pull_tab")
    model.articulation(
        "gearbox_to_inspection_door",
        ArticulationType.REVOLUTE,
        parent=gearbox,
        child=inspection_door,
        origin=Origin(xyz=(-0.25, -0.149, 0.10)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    guide_0 = object_model.get_part("guide_0")
    guide_1 = object_model.get_part("guide_1")
    lift_plate = object_model.get_part("lift_plate")
    gearbox = object_model.get_part("gearbox")
    handwheel = object_model.get_part("handwheel")
    door = object_model.get_part("inspection_door")
    lift_joint = object_model.get_articulation("frame_to_lift_plate")
    handwheel_joint = object_model.get_articulation("gearbox_to_handwheel")
    door_joint = object_model.get_articulation("gearbox_to_inspection_door")

    ctx.check("lift plate is prismatic", lift_joint.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("handwheel is continuous", handwheel_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("inspection door is hinged", door_joint.articulation_type == ArticulationType.REVOLUTE)

    ctx.allow_overlap(
        gearbox,
        lift_plate,
        elem_a="screw_sleeve",
        elem_b="threaded_stem",
        reason="The vertical lifting stem is intentionally captured inside the gearbox screw sleeve.",
    )
    ctx.expect_within(lift_plate, gearbox, axes="xy", inner_elem="threaded_stem", outer_elem="screw_sleeve", margin=0.002, name="stem is centered in screw sleeve")
    ctx.expect_overlap(lift_plate, gearbox, axes="z", min_overlap=0.10, elem_a="threaded_stem", elem_b="screw_sleeve", name="stem remains inserted in screw sleeve")

    ctx.expect_gap(frame, guide_0, axis="y", positive_elem="pier_0", negative_elem="channel_web", min_gap=0.025, name="guide 0 channel is separated from wall face")
    ctx.expect_gap(frame, guide_1, axis="y", positive_elem="pier_1", negative_elem="channel_web", min_gap=0.025, name="guide 1 channel is separated from wall face")
    ctx.expect_overlap(lift_plate, guide_0, axes="z", min_overlap=1.20, elem_a="gate_leaf", elem_b="channel_web", name="gate leaf is retained in guide 0")
    ctx.expect_overlap(lift_plate, guide_1, axes="z", min_overlap=1.20, elem_a="gate_leaf", elem_b="channel_web", name="gate leaf is retained in guide 1")

    ctx.expect_gap(gearbox, door, axis="y", positive_elem="housing_left", negative_elem="door_panel", max_penetration=0.002, max_gap=0.006, name="inspection door sits on housing face")
    ctx.expect_overlap(gearbox, door, axes="xz", min_overlap=0.10, elem_a="housing_left", elem_b="door_panel", name="inspection door covers housing opening")

    rest_plate_pos = ctx.part_world_position(lift_plate)
    with ctx.pose({lift_joint: 0.45}):
        raised_plate_pos = ctx.part_world_position(lift_plate)
        ctx.expect_overlap(lift_plate, guide_0, axes="z", min_overlap=1.20, elem_a="gate_leaf", elem_b="channel_web", name="raised gate remains in guide 0")
        ctx.expect_overlap(lift_plate, guide_1, axes="z", min_overlap=1.20, elem_a="gate_leaf", elem_b="channel_web", name="raised gate remains in guide 1")
    ctx.check(
        "lift plate travels upward",
        rest_plate_pos is not None and raised_plate_pos is not None and raised_plate_pos[2] > rest_plate_pos[2] + 0.40,
        details=f"rest={rest_plate_pos}, raised={raised_plate_pos}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 0.9}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    if closed_door_aabb is not None and open_door_aabb is not None:
        closed_min_y = closed_door_aabb[0][1]
        open_min_y = open_door_aabb[0][1]
    else:
        closed_min_y = open_min_y = None
    ctx.check(
        "inspection door opens outward",
        closed_min_y is not None and open_min_y is not None and open_min_y < closed_min_y - 0.04,
        details=f"closed_min_y={closed_min_y}, open_min_y={open_min_y}",
    )

    with ctx.pose({handwheel_joint: 1.57}):
        ctx.expect_contact(handwheel, gearbox, elem_a="hub", elem_b="handwheel_shaft", contact_tol=0.008, name="handwheel remains on gearbox shaft")

    return ctx.report()


object_model = build_object_model()
