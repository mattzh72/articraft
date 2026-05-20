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
    model = ArticulatedObject(name="field_irrigation_sluice_gate")

    galvanized = Material("weathered_galvanized_steel", rgba=(0.56, 0.58, 0.54, 1.0))
    dark_steel = Material("dark_water_stained_steel", rgba=(0.16, 0.18, 0.18, 1.0))
    rubbed_steel = Material("rubbed_bare_steel", rgba=(0.66, 0.65, 0.58, 1.0))
    bronze = Material("worn_bronze_guides", rgba=(0.58, 0.41, 0.20, 1.0))
    housing_green = Material("painted_gearbox_green", rgba=(0.20, 0.34, 0.27, 1.0))
    handwheel_black = Material("black_cast_handwheel", rgba=(0.04, 0.045, 0.04, 1.0))
    seal_rubber = Material("black_rubber_seal", rgba=(0.015, 0.018, 0.015, 1.0))

    frame = model.part("frame")
    # A meter-scale irrigation opening built from discrete steel members.
    frame.visual(
        Box((0.16, 0.22, 1.10)),
        origin=Origin(xyz=(-0.60, 0.0, 0.55)),
        material=galvanized,
        name="side_post_0",
    )
    frame.visual(
        Box((0.16, 0.22, 1.10)),
        origin=Origin(xyz=(0.60, 0.0, 0.55)),
        material=galvanized,
        name="side_post_1",
    )
    frame.visual(
        Box((1.40, 0.22, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=galvanized,
        name="bottom_sill",
    )
    frame.visual(
        Box((1.40, 0.22, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=galvanized,
        name="top_crossmember",
    )
    # Bronze guide liners and front retaining lips make the vertical slide path explicit.
    frame.visual(
        Box((0.052, 0.080, 0.88)),
        origin=Origin(xyz=(-0.485, -0.145, 0.57)),
        material=bronze,
        name="guide_liner_0",
    )
    frame.visual(
        Box((0.052, 0.080, 0.88)),
        origin=Origin(xyz=(0.485, -0.145, 0.57)),
        material=bronze,
        name="guide_liner_1",
    )
    frame.visual(
        Box((0.025, 0.025, 0.88)),
        origin=Origin(xyz=(-0.445, -0.170, 0.57)),
        material=galvanized,
        name="front_retain_lip_0",
    )
    frame.visual(
        Box((0.025, 0.025, 0.88)),
        origin=Origin(xyz=(0.445, -0.170, 0.57)),
        material=galvanized,
        name="front_retain_lip_1",
    )
    frame.visual(
        Box((0.070, 0.025, 0.09)),
        origin=Origin(xyz=(-0.485, -0.170, 1.00)),
        material=galvanized,
        name="upper_guide_stop_0",
    )
    frame.visual(
        Box((0.070, 0.025, 0.09)),
        origin=Origin(xyz=(0.485, -0.170, 1.00)),
        material=galvanized,
        name="upper_guide_stop_1",
    )
    # Bolt heads on the frame and on the top crossmember flange.
    bolt_positions = [
        (-0.60, -0.116, 0.25),
        (-0.60, -0.116, 0.55),
        (-0.60, -0.116, 0.85),
        (0.60, -0.116, 0.25),
        (0.60, -0.116, 0.55),
        (0.60, -0.116, 0.85),
        (-0.42, -0.116, 1.06),
        (0.42, -0.116, 1.06),
    ]
    for i, xyz in enumerate(bolt_positions):
        frame.visual(
            Cylinder(radius=0.020, length=0.016),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubbed_steel,
            name=f"frame_bolt_{i}",
        )

    gate = model.part("gate_panel")
    gate.visual(
        Box((0.86, 0.035, 0.72)),
        origin=Origin(),
        material=dark_steel,
        name="sliding_plate",
    )
    gate.visual(
        Box((0.82, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.025, -0.20)),
        material=rubbed_steel,
        name="lower_stiffener",
    )
    gate.visual(
        Box((0.82, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.025, 0.02)),
        material=rubbed_steel,
        name="middle_stiffener",
    )
    gate.visual(
        Box((0.82, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.025, 0.24)),
        material=rubbed_steel,
        name="upper_stiffener",
    )
    for i, sx in enumerate((-1.0, 1.0)):
        gate.visual(
            Box((0.035, 0.020, 0.66)),
            origin=Origin(xyz=(sx * 0.390, -0.025, 0.0)),
            material=rubbed_steel,
            name=f"side_stiffener_{i}",
        )
    gate.visual(
        Box((0.052, 0.035, 0.62)),
        origin=Origin(xyz=(-0.433, 0.0, 0.0)),
        material=bronze,
        name="slide_shoe_0",
    )
    gate.visual(
        Box((0.052, 0.035, 0.62)),
        origin=Origin(xyz=(0.433, 0.0, 0.0)),
        material=bronze,
        name="slide_shoe_1",
    )
    gate.visual(
        Box((0.18, 0.085, 0.050)),
        origin=Origin(xyz=(-0.11, -0.050, 0.375)),
        material=rubbed_steel,
        name="lift_yoke",
    )
    gate.visual(
        Cylinder(radius=0.012, length=0.260),
        origin=Origin(xyz=(-0.11, -0.050, 0.490)),
        material=rubbed_steel,
        name="lift_screw",
    )
    gate.visual(
        Box((0.88, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, 0.0265, -0.337)),
        material=seal_rubber,
        name="bottom_seal",
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, -0.130, 0.510)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.08, lower=0.0, upper=0.34),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.46, 0.30, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=housing_green,
        name="mounting_foot",
    )
    housing.visual(
        Box((0.36, 0.24, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=housing_green,
        name="gearbox_body",
    )
    housing.visual(
        Box((0.30, 0.20, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.2975)),
        material=housing_green,
        name="top_cap",
    )
    housing.visual(
        Cylinder(radius=0.048, length=0.070),
        origin=Origin(xyz=(0.0, -0.155, 0.160), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing_green,
        name="input_boss",
    )
    for i, (x, y) in enumerate(((-0.18, -0.105), (0.18, -0.105), (-0.18, 0.105), (0.18, 0.105))):
        housing.visual(
            Cylinder(radius=0.015, length=0.012),
            origin=Origin(xyz=(x, y, 0.046)),
            material=rubbed_steel,
            name=f"foot_bolt_{i}",
        )
    # Interleaved fixed hinge knuckles for the inspection flap.
    housing.visual(
        Box((0.018, 0.045, 0.170)),
        origin=Origin(xyz=(0.184, -0.095, 0.160)),
        material=rubbed_steel,
        name="fixed_hinge_leaf",
    )
    for i, z in enumerate((0.105, 0.215)):
        housing.visual(
            Cylinder(radius=0.010, length=0.036),
            origin=Origin(xyz=(0.197, -0.065, z), rpy=(0.0, 0.0, 0.0)),
            material=rubbed_steel,
            name=f"fixed_hinge_knuckle_{i}",
        )
    model.articulation(
        "frame_to_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, -0.020, 1.140)),
    )

    wheel = model.part("handwheel")
    wheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.130, tube=0.012, radial_segments=20, tubular_segments=48), "handwheel_rim"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handwheel_black,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.034, length=0.055),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handwheel_black,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubbed_steel,
        name="input_stub",
    )
    wheel.visual(
        Box((0.250, 0.020, 0.018)),
        origin=Origin(),
        material=handwheel_black,
        name="horizontal_spoke",
    )
    wheel.visual(
        Box((0.018, 0.020, 0.250)),
        origin=Origin(),
        material=handwheel_black,
        name="vertical_spoke",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.072),
        origin=Origin(xyz=(0.092, -0.040, 0.092), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handwheel_black,
        name="spinner_grip",
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=wheel,
        origin=Origin(xyz=(0.0, -0.230, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )

    door = model.part("inspection_door")
    door.visual(
        Box((0.012, 0.118, 0.115)),
        origin=Origin(xyz=(-0.011, 0.071, 0.0)),
        material=housing_green,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(),
        material=rubbed_steel,
        name="door_hinge_knuckle",
    )
    door.visual(
        Box((0.006, 0.025, 0.036)),
        origin=Origin(xyz=(-0.006, 0.012, 0.0)),
        material=rubbed_steel,
        name="hinge_leaf",
    )
    door.visual(
        Box((0.010, 0.025, 0.025)),
        origin=Origin(xyz=(0.000, 0.112, 0.0)),
        material=rubbed_steel,
        name="pull_tab",
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.197, -0.065, 0.160)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate_panel")
    housing = object_model.get_part("housing")
    wheel = object_model.get_part("handwheel")
    door = object_model.get_part("inspection_door")
    gate_slide = object_model.get_articulation("gate_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        wheel,
        housing,
        elem_a="input_stub",
        elem_b="input_boss",
        reason="The handwheel input shaft is intentionally captured inside the gearbox boss.",
    )

    ctx.expect_contact(
        housing,
        frame,
        elem_a="mounting_foot",
        elem_b="top_crossmember",
        name="operator housing foot is seated on the top crossmember",
    )
    ctx.expect_overlap(
        housing,
        frame,
        axes="xy",
        elem_a="mounting_foot",
        elem_b="top_crossmember",
        min_overlap=0.20,
        name="operator housing footprint is over the crossmember",
    )
    ctx.expect_within(
        gate,
        frame,
        axes="x",
        inner_elem="sliding_plate",
        outer_elem="top_crossmember",
        margin=0.0,
        name="gate panel width remains inside the framed opening",
    )
    ctx.expect_overlap(
        gate,
        frame,
        axes="z",
        elem_a="sliding_plate",
        elem_b="guide_liner_0",
        min_overlap=0.55,
        name="gate panel is retained vertically in the guide liners",
    )
    ctx.expect_contact(
        wheel,
        housing,
        elem_a="input_stub",
        elem_b="input_boss",
        contact_tol=0.002,
        name="handwheel input stub is supported by the housing boss",
    )
    ctx.expect_overlap(
        wheel,
        housing,
        axes="y",
        elem_a="input_stub",
        elem_b="input_boss",
        min_overlap=0.008,
        name="handwheel shaft remains inserted in the gearbox boss",
    )
    ctx.expect_contact(
        door,
        housing,
        elem_a="door_panel",
        elem_b="gearbox_body",
        contact_tol=0.002,
        name="inspection door flap rests on the housing face",
    )

    closed_gate_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: 0.34}):
        raised_gate_pos = ctx.part_world_position(gate)
        ctx.expect_overlap(
            gate,
            frame,
            axes="z",
            elem_a="sliding_plate",
            elem_b="guide_liner_0",
            min_overlap=0.20,
            name="raised gate remains engaged in the guides",
        )
    ctx.check(
        "gate slide moves upward",
        closed_gate_pos is not None
        and raised_gate_pos is not None
        and raised_gate_pos[2] > closed_gate_pos[2] + 0.30,
        details=f"closed={closed_gate_pos}, raised={raised_gate_pos}",
    )

    door_closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.0}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "inspection door swings outward from side hinge",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][0] > door_closed_aabb[1][0] + 0.045,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: 1.25}):
        spun_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "handwheel rotates in place on continuous input shaft",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_wheel_pos, spun_wheel_pos)),
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
