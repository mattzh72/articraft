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
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mine_shaft_elevator_cage")

    dark_steel = model.material("dark_blued_steel", rgba=(0.07, 0.08, 0.085, 1.0))
    worn_steel = model.material("worn_galvanized_steel", rgba=(0.48, 0.50, 0.48, 1.0))
    rust = model.material("rust_stained_steel", rgba=(0.42, 0.22, 0.10, 1.0))
    grated_floor = model.material("black_grated_floor", rgba=(0.03, 0.035, 0.035, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    sheave_mat = model.material("oiled_sheave_iron", rgba=(0.12, 0.125, 0.12, 1.0))

    shaft_frame = model.part("shaft_frame")

    def add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Fixed shaft: two continuous guide rails tied into a base sill and a head frame.
    add_box(shaft_frame, "base_sill", (1.70, 0.18, 0.10), (0.0, 0.0, 0.05), rust)
    add_box(shaft_frame, "top_sill", (1.70, 0.18, 0.10), (0.0, 0.0, 4.49), rust)
    for x, tag in [(-0.68, "rail_0"), (0.68, "rail_1")]:
        add_box(shaft_frame, tag, (0.08, 0.10, 4.15), (x, 0.0, 2.125), dark_steel)
        add_box(shaft_frame, f"{tag}_wear_strip", (0.018, 0.125, 4.05), (x * 0.98, -0.075, 2.12), worn_steel)

    # Overhead A-frame and sheave bearing supports.
    for x, tag in [(-0.78, "post_0"), (0.78, "post_1")]:
        add_box(shaft_frame, tag, (0.08, 0.12, 4.48), (x, 0.02, 2.24), dark_steel)
    add_box(shaft_frame, "head_beam", (1.85, 0.16, 0.14), (0.0, 0.02, 4.55), dark_steel)
    add_box(shaft_frame, "rear_tie", (1.55, 0.08, 0.08), (0.0, 0.33, 3.80), dark_steel)
    add_box(shaft_frame, "front_tie", (1.55, 0.08, 0.08), (0.0, -0.33, 3.55), dark_steel)
    add_box(shaft_frame, "side_tie_0", (0.08, 0.74, 0.08), (-0.78, 0.0, 3.80), dark_steel)
    add_box(shaft_frame, "side_tie_1", (0.08, 0.74, 0.08), (0.78, 0.0, 3.80), dark_steel)
    add_box(shaft_frame, "front_tie_arm_0", (0.08, 0.35, 0.08), (-0.78, -0.165, 3.55), dark_steel)
    add_box(shaft_frame, "front_tie_arm_1", (0.08, 0.35, 0.08), (0.78, -0.165, 3.55), dark_steel)
    add_box(shaft_frame, "bearing_plate_0", (0.035, 0.20, 0.70), (-0.13, -0.04, 4.10), rust)
    add_box(shaft_frame, "bearing_plate_1", (0.035, 0.20, 0.70), (0.13, -0.04, 4.10), rust)
    add_cyl(
        shaft_frame,
        "fixed_axle",
        0.025,
        0.34,
        (0.0, -0.04, 4.05),
        worn_steel,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    cage = model.part("cage")

    # The moving cage is an open steel basket: corner posts, rails, roof hoop,
    # rear bars and a solid, grippy floor pan.  The child frame is at the
    # bottom-center carriage datum for the vertical slide.
    add_box(cage, "floor_pan", (0.96, 0.72, 0.06), (0.0, 0.0, 0.09), grated_floor)
    add_box(cage, "front_floor_lip", (0.96, 0.045, 0.11), (0.0, -0.385, 0.155), dark_steel)
    add_box(cage, "rear_floor_lip", (0.96, 0.045, 0.11), (0.0, 0.385, 0.155), dark_steel)
    add_box(cage, "side_floor_lip_0", (0.045, 0.72, 0.11), (-0.5025, 0.0, 0.155), dark_steel)
    add_box(cage, "side_floor_lip_1", (0.045, 0.72, 0.11), (0.5025, 0.0, 0.155), dark_steel)

    for x, xname in [(-0.48, "left"), (0.48, "right")]:
        for y, yname in [(-0.36, "front"), (0.36, "rear")]:
            add_box(cage, f"{yname}_{xname}_post", (0.055, 0.055, 2.05), (x, y, 1.17), dark_steel)

    for z, zname in [(0.46, "lower"), (1.20, "middle"), (1.94, "upper")]:
        add_box(cage, f"{zname}_rear_rail", (0.96, 0.045, 0.055), (0.0, 0.36, z), dark_steel)
        add_box(cage, f"{zname}_side_rail_0", (0.055, 0.72, 0.055), (-0.48, 0.0, z), dark_steel)
        add_box(cage, f"{zname}_side_rail_1", (0.055, 0.72, 0.055), (0.48, 0.0, z), dark_steel)

    for x, tag in [(-0.24, "rear_bar_0"), (0.0, "rear_bar_1"), (0.24, "rear_bar_2")]:
        add_box(cage, tag, (0.035, 0.035, 1.48), (x, 0.36, 1.20), worn_steel)

    add_box(cage, "roof_front_rail", (0.96, 0.055, 0.08), (0.0, -0.36, 2.19), dark_steel)
    add_box(cage, "roof_rear_rail", (0.96, 0.055, 0.08), (0.0, 0.36, 2.19), dark_steel)
    add_box(cage, "roof_crossbar", (0.96, 0.055, 0.08), (0.0, 0.0, 2.19), dark_steel)
    add_box(cage, "roof_side_rail_0", (0.055, 0.72, 0.08), (-0.48, 0.0, 2.19), dark_steel)
    add_box(cage, "roof_side_rail_1", (0.055, 0.72, 0.08), (0.48, 0.0, 2.19), dark_steel)
    add_box(cage, "lift_bail", (0.18, 0.07, 0.12), (0.0, 0.0, 2.285), yellow)

    # Guide shoes are visibly mounted to the cage side rails but remain clear of
    # the fixed guide rails.
    for x, sign, side in [(-0.565, -1.0, "left"), (0.565, 1.0, "right")]:
        for z, tag in [(0.72, "lower"), (1.72, "upper")]:
            add_box(cage, f"{side}_{tag}_shoe", (0.15, 0.13, 0.10), (x, -0.02, z), worn_steel)
            add_box(cage, f"{side}_{tag}_shoe_web", (0.09, 0.72, 0.10), (0.51 * sign, -0.02, z), dark_steel)

    gate = model.part("gate")
    # Gate frame: local origin is the vertical hinge pin.  In the closed pose it
    # spans along local +X across the cage front.
    add_box(gate, "gate_hinge_stile", (0.045, 0.040, 1.70), (0.025, 0.0, 1.14), yellow)
    add_box(gate, "gate_latch_stile", (0.045, 0.040, 1.70), (0.89, 0.0, 1.14), yellow)
    for z, name in [(0.35, "bottom"), (1.14, "middle"), (1.93, "top")]:
        add_box(gate, f"gate_{name}_rail", (0.91, 0.040, 0.055), (0.455, 0.0, z), yellow)
    for x, name in [(0.23, "bar_0"), (0.455, "bar_1"), (0.68, "bar_2")]:
        add_box(gate, f"gate_{name}", (0.035, 0.035, 1.48), (x, 0.0, 1.14), worn_steel)
    for z, name in [(0.57, "lower"), (1.52, "upper")]:
        add_cyl(gate, f"{name}_hinge_barrel", 0.023, 0.34, (0.0, 0.0, z), dark_steel)
        add_box(gate, f"{name}_hinge_leaf", (0.07, 0.012, 0.26), (0.035, -0.026, z), dark_steel)
    add_box(gate, "latch_keeper", (0.075, 0.08, 0.08), (0.93, -0.01, 1.18), rust)

    sheave = model.part("sheave")
    sheave_mesh = mesh_from_geometry(
        WheelGeometry(
            0.34,
            0.075,
            rim=WheelRim(inner_radius=0.255, flange_height=0.030, flange_thickness=0.012, bead_seat_depth=0.012),
            hub=WheelHub(radius=0.070, width=0.095, cap_style="flat"),
            face=WheelFace(dish_depth=0.010, front_inset=0.006, rear_inset=0.006),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.018, window_radius=0.048),
            bore=WheelBore(style="round", diameter=0.070),
        ),
        "grooved_sheave_wheel",
    )
    sheave.visual(sheave_mesh, origin=Origin(), material=sheave_mat, name="sheave_wheel")
    add_cyl(
        sheave,
        "hub_collar",
        0.060,
        0.045,
        (0.0, 0.0, 0.0),
        worn_steel,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    model.articulation(
        "cage_lift",
        ArticulationType.PRISMATIC,
        parent=shaft_frame,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.8, lower=0.0, upper=1.65),
    )
    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=gate,
        origin=Origin(xyz=(-0.5305, -0.4075, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.65),
    )
    model.articulation(
        "sheave_axle",
        ArticulationType.REVOLUTE,
        parent=shaft_frame,
        child=sheave,
        origin=Origin(xyz=(0.0, -0.04, 4.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft_frame")
    cage = object_model.get_part("cage")
    gate = object_model.get_part("gate")
    sheave = object_model.get_part("sheave")
    cage_lift = object_model.get_articulation("cage_lift")
    gate_hinge = object_model.get_articulation("gate_hinge")
    sheave_axle = object_model.get_articulation("sheave_axle")

    ctx.allow_overlap(
        shaft,
        sheave,
        elem_a="fixed_axle",
        elem_b="hub_collar",
        reason="The visible rotating hub collar is intentionally captured around the fixed axle shaft.",
    )
    ctx.expect_within(
        shaft,
        sheave,
        axes="yz",
        inner_elem="fixed_axle",
        outer_elem="hub_collar",
        margin=0.001,
        name="axle is concentric inside the sheave hub collar",
    )
    ctx.expect_overlap(
        shaft,
        sheave,
        axes="x",
        elem_a="fixed_axle",
        elem_b="hub_collar",
        min_overlap=0.04,
        name="hub collar remains captured on the axle",
    )

    ctx.expect_within(
        cage,
        shaft,
        axes="x",
        inner_elem="floor_pan",
        outer_elem="base_sill",
        margin=0.0,
        name="cage sits between the two guide rails",
    )
    ctx.expect_overlap(
        cage,
        shaft,
        axes="z",
        elem_a="left_lower_shoe",
        elem_b="rail_0",
        min_overlap=0.08,
        name="left guide shoe tracks the left rail height",
    )
    ctx.expect_overlap(
        cage,
        shaft,
        axes="z",
        elem_a="right_lower_shoe",
        elem_b="rail_1",
        min_overlap=0.08,
        name="right guide shoe tracks the right rail height",
    )
    ctx.expect_contact(
        gate,
        cage,
        elem_a="gate_middle_rail",
        elem_b="front_left_post",
        contact_tol=0.001,
        name="closed gate bears against the front post",
    )
    ctx.expect_gap(
        shaft,
        sheave,
        axis="x",
        positive_elem="bearing_plate_1",
        negative_elem="sheave_wheel",
        min_gap=0.01,
        max_gap=0.08,
        name="sheave clears one bearing cheek",
    )

    rest_pos = ctx.part_world_position(cage)
    with ctx.pose({cage_lift: 1.25}):
        raised_pos = ctx.part_world_position(cage)
        ctx.expect_overlap(
            cage,
            shaft,
            axes="z",
            elem_a="right_upper_shoe",
            elem_b="rail_1",
            min_overlap=0.08,
            name="raised cage remains captured by guide rail",
        )
    ctx.check(
        "prismatic lift moves cage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 1.0,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    closed_gate_pos = ctx.part_world_position(gate)
    with ctx.pose({gate_hinge: 1.25}):
        open_gate_pos = ctx.part_world_position(gate)
        ctx.expect_origin_gap(gate, cage, axis="y", min_gap=-0.55, max_gap=-0.15, name="gate swings outward at the front")
    ctx.check(
        "gate hinge part remains on hinge line",
        closed_gate_pos is not None and open_gate_pos is not None and abs(open_gate_pos[0] - closed_gate_pos[0]) < 0.01,
        details=f"closed={closed_gate_pos}, open={open_gate_pos}",
    )
    ctx.check(
        "sheave wheel has a revolute axle",
        sheave_axle.axis == (1.0, 0.0, 0.0),
        details=f"axis={sheave_axle.axis}",
    )

    return ctx.report()


object_model = build_object_model()
