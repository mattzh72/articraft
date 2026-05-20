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
)


def _add_handwheel_geometry(part, material) -> None:
    """Build a spoked municipal handwheel in the part's XZ plane."""
    rim_radius = 0.28
    rim_segments = 16
    chord = 2.0 * rim_radius * math.sin(math.pi / rim_segments) * 1.10

    for i in range(rim_segments):
        angle = (i + 0.5) * 2.0 * math.pi / rim_segments
        x = rim_radius * math.cos(angle)
        z = rim_radius * math.sin(angle)
        # Cylinder local +Z is rotated into the tangent direction in the XZ plane.
        tangent_x = -math.sin(angle)
        tangent_z = math.cos(angle)
        part.visual(
            Cylinder(radius=0.018, length=chord),
            origin=Origin(
                xyz=(x, 0.0, z),
                rpy=(0.0, math.atan2(tangent_x, tangent_z), 0.0),
            ),
            material=material,
            name=f"rim_segment_{i}",
        )

    for i in range(6):
        angle = i * 2.0 * math.pi / 6.0
        x = 0.165 * math.cos(angle)
        z = 0.165 * math.sin(angle)
        radial_yaw = math.atan2(math.cos(angle), math.sin(angle))
        part.visual(
            Cylinder(radius=0.012, length=0.23),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, radial_yaw, 0.0)),
            material=material,
            name=f"spoke_{i}",
        )

    part.visual(
        Cylinder(radius=0.075, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="wheel_hub",
    )
    part.visual(
        Cylinder(radius=0.035, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="hub_cap",
    )

    knob_angle = math.radians(30.0)
    part.visual(
        Cylinder(radius=0.026, length=0.14),
        origin=Origin(
            xyz=(
                rim_radius * math.cos(knob_angle),
                -0.070,
                rim_radius * math.sin(knob_angle),
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="spinner_grip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="municipal_lift_gate")

    weathered_steel = model.material("weathered_steel", rgba=(0.28, 0.31, 0.32, 1.0))
    dark_channel = model.material("dark_channel", rgba=(0.10, 0.13, 0.14, 1.0))
    gate_blue = model.material("gate_blue", rgba=(0.05, 0.28, 0.36, 1.0))
    rubbed_steel = model.material("rubbed_steel", rgba=(0.62, 0.66, 0.65, 1.0))
    black_iron = model.material("black_iron", rgba=(0.02, 0.025, 0.025, 1.0))
    brass = model.material("aged_brass", rgba=(0.70, 0.52, 0.22, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.32, 0.65, 3.40)),
        origin=Origin(xyz=(-1.45, 0.0, 1.70)),
        material=weathered_steel,
        name="side_column_0",
    )
    frame.visual(
        Box((0.32, 0.65, 3.40)),
        origin=Origin(xyz=(1.45, 0.0, 1.70)),
        material=weathered_steel,
        name="side_column_1",
    )
    frame.visual(
        Box((3.25, 0.22, 0.40)),
        origin=Origin(xyz=(0.0, -0.24, 3.25)),
        material=weathered_steel,
        name="top_beam_front",
    )
    frame.visual(
        Box((3.25, 0.22, 0.40)),
        origin=Origin(xyz=(0.0, 0.24, 3.25)),
        material=weathered_steel,
        name="top_beam_rear",
    )
    frame.visual(
        Box((3.25, 0.65, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_channel,
        name="bottom_sill",
    )

    for side, x, wear_name in (
        ("left", -1.285, "left_wear_strip"),
        ("right", 1.285, "right_wear_strip"),
    ):
        frame.visual(
            Box((0.10, 0.12, 2.90)),
            origin=Origin(xyz=(x, -0.18, 1.62)),
            material=dark_channel,
            name=f"{side}_front_guide",
        )
        frame.visual(
            Box((0.10, 0.12, 2.90)),
            origin=Origin(xyz=(x, 0.18, 1.62)),
            material=dark_channel,
            name=f"{side}_rear_guide",
        )
        frame.visual(
            Box((0.035, 0.16, 2.85)),
            origin=Origin(xyz=(1.24 if side == "right" else -1.24, 0.0, 1.62)),
            material=rubbed_steel,
            name=wear_name,
        )

    # A visible slotted cap across the guide tops ties the frame together while
    # leaving a central passage for the rising stem and panel.
    frame.visual(
        Box((0.72, 0.26, 0.12)),
        origin=Origin(xyz=(-0.82, 0.0, 3.02)),
        material=weathered_steel,
        name="top_slot_lip_0",
    )
    frame.visual(
        Box((0.72, 0.26, 0.12)),
        origin=Origin(xyz=(0.82, 0.0, 3.02)),
        material=weathered_steel,
        name="top_slot_lip_1",
    )

    lift_panel = model.part("lift_panel")
    lift_panel.visual(
        Box((2.25, 0.12, 2.25)),
        origin=Origin(),
        material=gate_blue,
        name="gate_plate",
    )
    lift_panel.visual(
        Box((2.35, 0.20, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=gate_blue,
        name="top_edge_girder",
    )
    lift_panel.visual(
        Box((2.35, 0.20, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -1.06)),
        material=gate_blue,
        name="bottom_edge_girder",
    )
    for i, x in enumerate((-0.82, 0.0, 0.82)):
        lift_panel.visual(
            Box((0.16, 0.22, 2.08)),
            origin=Origin(xyz=(x, -0.015, 0.0)),
            material=weathered_steel,
            name=f"vertical_stiffener_{i}",
        )
    for i, z in enumerate((-0.62, 0.06, 0.72)):
        lift_panel.visual(
            Box((2.18, 0.22, 0.13)),
            origin=Origin(xyz=(0.0, -0.020, z)),
            material=weathered_steel,
            name=f"horizontal_stiffener_{i}",
        )
    lift_panel.visual(
        Box((0.36, 0.26, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
        material=weathered_steel,
        name="lifting_boss",
    )
    lift_panel.visual(
        Cylinder(radius=0.035, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
        material=rubbed_steel,
        name="lift_stem",
    )
    lift_panel.visual(
        Box((0.0975, 0.16, 1.90)),
        origin=Origin(xyz=(-1.17375, 0.0, 0.0)),
        material=rubbed_steel,
        name="guide_shoe_0",
    )
    lift_panel.visual(
        Box((0.0975, 0.16, 1.90)),
        origin=Origin(xyz=(1.17375, 0.0, 0.0)),
        material=rubbed_steel,
        name="guide_shoe_1",
    )

    operator_box = model.part("operator_box")
    operator_box.visual(
        Box((0.85, 0.50, 0.55)),
        origin=Origin(),
        material=weathered_steel,
        name="housing",
    )
    operator_box.visual(
        Box((1.02, 0.56, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
        material=dark_channel,
        name="mounting_flange",
    )
    operator_box.visual(
        Box((0.92, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=dark_channel,
        name="rain_cap",
    )
    operator_box.visual(
        Cylinder(radius=0.062, length=0.34),
        origin=Origin(xyz=(0.0, -0.42, 0.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubbed_steel,
        name="handwheel_shaft",
    )
    operator_box.visual(
        Box((0.18, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, -0.275, 0.02)),
        material=dark_channel,
        name="shaft_boss",
    )
    for i, (x, y) in enumerate(((-0.39, -0.20), (0.39, -0.20), (-0.39, 0.20), (0.39, 0.20))):
        operator_box.visual(
            Cylinder(radius=0.032, length=0.026),
            origin=Origin(xyz=(x, y, -0.255)),
            material=rubbed_steel,
            name=f"flange_bolt_{i}",
        )

    access_door = model.part("access_door")
    access_door.visual(
        Box((0.035, 0.40, 0.42)),
        origin=Origin(xyz=(0.0, 0.20, 0.0)),
        material=dark_channel,
        name="door_leaf",
    )
    access_door.visual(
        Box((0.018, 0.30, 0.30)),
        origin=Origin(xyz=(0.021, 0.20, 0.0)),
        material=weathered_steel,
        name="raised_panel",
    )
    access_door.visual(
        Cylinder(radius=0.025, length=0.13),
        origin=Origin(xyz=(-0.005, 0.0, 0.145)),
        material=rubbed_steel,
        name="hinge_barrel_0",
    )
    access_door.visual(
        Cylinder(radius=0.025, length=0.13),
        origin=Origin(xyz=(-0.005, 0.0, -0.145)),
        material=rubbed_steel,
        name="hinge_barrel_1",
    )
    access_door.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.040, 0.33, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="latch_knob",
    )

    handwheel = model.part("handwheel")
    _add_handwheel_geometry(handwheel, black_iron)

    model.articulation(
        "panel_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_panel,
        origin=Origin(xyz=(0.0, 0.0, 1.425)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30000.0, velocity=0.18, lower=0.0, upper=1.05),
    )

    model.articulation(
        "box_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=operator_box,
        origin=Origin(xyz=(0.0, -0.50, 3.80)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=operator_box,
        child=access_door,
        origin=Origin(xyz=(0.455, -0.20, 0.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=math.radians(95.0)),
    )

    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=operator_box,
        child=handwheel,
        origin=Origin(xyz=(0.0, -0.66, 0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    lift_panel = object_model.get_part("lift_panel")
    operator_box = object_model.get_part("operator_box")
    access_door = object_model.get_part("access_door")
    handwheel = object_model.get_part("handwheel")

    panel_lift = object_model.get_articulation("panel_lift")
    door_hinge = object_model.get_articulation("door_hinge")
    handwheel_spin = object_model.get_articulation("handwheel_spin")

    ctx.check(
        "panel lift is vertical prismatic",
        panel_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(panel_lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={panel_lift.articulation_type}, axis={panel_lift.axis}",
    )
    ctx.check(
        "front handwheel is continuous",
        handwheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={handwheel_spin.articulation_type}",
    )
    ctx.check(
        "access door has side hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )

    ctx.expect_gap(
        lift_panel,
        frame,
        axis="z",
        positive_elem="gate_plate",
        negative_elem="bottom_sill",
        min_gap=0.035,
        max_gap=0.065,
        name="closed gate plate seats just above sill",
    )
    ctx.expect_gap(
        frame,
        lift_panel,
        axis="x",
        positive_elem="right_wear_strip",
        negative_elem="top_edge_girder",
        min_gap=0.025,
        max_gap=0.080,
        name="right guide clears panel edge",
    )
    ctx.expect_gap(
        lift_panel,
        frame,
        axis="x",
        positive_elem="top_edge_girder",
        negative_elem="left_wear_strip",
        min_gap=0.025,
        max_gap=0.080,
        name="left guide clears panel edge",
    )
    ctx.expect_gap(
        access_door,
        operator_box,
        axis="x",
        positive_elem="door_leaf",
        negative_elem="housing",
        min_gap=0.005,
        max_gap=0.020,
        name="side access door sits outside housing",
    )

    rest_panel_pos = ctx.part_world_position(lift_panel)
    with ctx.pose({panel_lift: 1.05}):
        lifted_panel_pos = ctx.part_world_position(lift_panel)
        ctx.expect_overlap(
            lift_panel,
            frame,
            axes="z",
            elem_a="gate_plate",
            elem_b="right_wear_strip",
            min_overlap=1.50,
            name="raised panel remains captured in vertical guide",
        )

    ctx.check(
        "panel travels upward",
        rest_panel_pos is not None
        and lifted_panel_pos is not None
        and lifted_panel_pos[2] > rest_panel_pos[2] + 1.0,
        details=f"rest={rest_panel_pos}, lifted={lifted_panel_pos}",
    )

    def element_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    closed_door_center = element_center(access_door, "door_leaf")
    with ctx.pose({door_hinge: math.radians(70.0)}):
        open_door_center = element_center(access_door, "door_leaf")
    ctx.check(
        "side access door swings outward",
        closed_door_center is not None
        and open_door_center is not None
        and open_door_center[0] > closed_door_center[0] + 0.12,
        details=f"closed={closed_door_center}, open={open_door_center}",
    )

    crank_rest = element_center(handwheel, "spinner_grip")
    with ctx.pose({handwheel_spin: math.pi / 2.0}):
        crank_turned = element_center(handwheel, "spinner_grip")
    ctx.check(
        "handwheel crank changes position when spun",
        crank_rest is not None
        and crank_turned is not None
        and abs(crank_rest[0] - crank_turned[0]) > 0.12
        and abs(crank_rest[2] - crank_turned[2]) > 0.08,
        details=f"rest={crank_rest}, turned={crank_turned}",
    )

    return ctx.report()


object_model = build_object_model()
