from __future__ import annotations

from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_latch_pedestrian_gate")

    dark_galv = model.material("dark_galvanized_steel", rgba=(0.18, 0.20, 0.21, 1.0))
    worn_edge = model.material("worn_steel_edges", rgba=(0.48, 0.50, 0.48, 1.0))
    black = model.material("black_latch_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    magnet_red = model.material("red_magnetic_latch_flag", rgba=(0.85, 0.06, 0.03, 1.0))
    concrete = model.material("pale_concrete_footing", rgba=(0.55, 0.54, 0.50, 1.0))

    hinge_z_upper = 1.25
    hinge_z_lower = 0.45

    fixture = model.part("fence_fixture")
    fixture.visual(
        Box((1.36, 0.18, 0.045)),
        origin=Origin(xyz=(0.53, 0.0, 0.0225)),
        material=concrete,
        name="shared_footing",
    )
    fixture.visual(
        Box((0.09, 0.09, 1.90)),
        origin=Origin(xyz=(-0.085, 0.0, 0.95)),
        material=dark_galv,
        name="hinge_post",
    )
    fixture.visual(
        Box((0.075, 0.075, 1.75)),
        origin=Origin(xyz=(1.17, 0.0, 0.875)),
        material=dark_galv,
        name="latch_post",
    )

    for prefix, z in (("upper", hinge_z_upper), ("lower", hinge_z_lower)):
        fixture.visual(
            Cylinder(radius=0.010, length=0.24),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=worn_edge,
            name=f"{prefix}_pintle_pin",
        )
        fixture.visual(
            Box((0.075, 0.030, 0.035)),
            origin=Origin(xyz=(-0.0375, 0.0, z - 0.092)),
            material=dark_galv,
            name=f"{prefix}_pintle_arm",
        )

    fixture.visual(
        Box((0.064, 0.070, 0.130)),
        origin=Origin(xyz=(1.104, 0.0, 1.05)),
        material=black,
        name="magnetic_receiver",
    )
    fixture.visual(
        Box((0.012, 0.072, 0.085)),
        origin=Origin(xyz=(1.068, 0.0, 1.05)),
        material=worn_edge,
        name="keeper_face",
    )
    fixture.visual(
        Box((0.012, 0.074, 0.030)),
        origin=Origin(xyz=(1.069, 0.0, 1.115)),
        material=magnet_red,
        name="release_flag",
    )

    gate = model.part("gate_frame")
    # The gate part frame is on the upper pintle axis.  All local z positions
    # below are therefore measured relative to the upper hinge center.
    gate.visual(
        Box((0.040, 0.040, 1.38)),
        origin=Origin(xyz=(0.065, 0.0, 0.83 - hinge_z_upper)),
        material=dark_galv,
        name="hinge_stile",
    )
    gate.visual(
        Box((0.040, 0.040, 1.38)),
        origin=Origin(xyz=(1.035, 0.0, 0.83 - hinge_z_upper)),
        material=dark_galv,
        name="latch_stile",
    )
    gate.visual(
        Box((1.01, 0.040, 0.040)),
        origin=Origin(xyz=(0.550, 0.0, 1.50 - hinge_z_upper)),
        material=dark_galv,
        name="top_rail",
    )
    gate.visual(
        Box((1.01, 0.040, 0.040)),
        origin=Origin(xyz=(0.550, 0.0, 0.16 - hinge_z_upper)),
        material=dark_galv,
        name="bottom_rail",
    )
    for i, x in enumerate((0.245, 0.425, 0.605, 0.785)):
        gate.visual(
            Box((0.022, 0.022, 1.31)),
            origin=Origin(xyz=(x, 0.0, 0.83 - hinge_z_upper)),
            material=dark_galv,
            name=f"picket_{i}",
        )
    gate.visual(
        Cylinder(radius=0.018, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_galv,
        name="upper_gate_barrel",
    )
    gate.visual(
        Box((0.085, 0.010, 0.050)),
        origin=Origin(xyz=(0.0425, -0.019, 0.0)),
        material=dark_galv,
        name="upper_hinge_strap",
    )
    gate.visual(
        Box((0.012, 0.068, 0.110)),
        origin=Origin(xyz=(1.053, 0.0, 1.05 - hinge_z_upper)),
        material=worn_edge,
        name="strike_plate",
    )

    lower_hinge = model.part("lower_hinge")
    lower_hinge.visual(
        Cylinder(radius=0.0205, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_galv,
        name="lower_gate_barrel",
    )
    lower_hinge.visual(
        Box((0.085, 0.010, 0.050)),
        origin=Origin(xyz=(0.0425, -0.025, 0.0)),
        material=dark_galv,
        name="lower_hinge_strap",
    )

    swing_limits = MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.57)
    model.articulation(
        "upper_pintle",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, hinge_z_upper)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swing_limits,
        motion_properties=MotionProperties(damping=0.4, friction=0.2),
    )
    model.articulation(
        "lower_pintle",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=lower_hinge,
        origin=Origin(xyz=(0.0, 0.0, hinge_z_lower)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swing_limits,
        motion_properties=MotionProperties(damping=0.4, friction=0.2),
        mimic=Mimic(joint="upper_pintle", multiplier=1.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixture = object_model.get_part("fence_fixture")
    gate = object_model.get_part("gate_frame")
    lower_hinge = object_model.get_part("lower_hinge")
    upper = object_model.get_articulation("upper_pintle")
    lower = object_model.get_articulation("lower_pintle")

    ctx.allow_overlap(
        fixture,
        gate,
        elem_a="upper_pintle_pin",
        elem_b="upper_gate_barrel",
        reason="The fixed pintle pin is intentionally captured inside the rotating upper hinge barrel.",
    )
    ctx.allow_overlap(
        fixture,
        lower_hinge,
        elem_a="lower_pintle_pin",
        elem_b="lower_gate_barrel",
        reason="The fixed pintle pin is intentionally captured inside the rotating lower hinge barrel.",
    )

    upper_origin = upper.origin.xyz
    lower_origin = lower.origin.xyz
    ctx.check(
        "two vertical pintle revolutes share hinge line",
        upper.articulation_type == ArticulationType.REVOLUTE
        and lower.articulation_type == ArticulationType.REVOLUTE
        and upper.axis == (0.0, 0.0, 1.0)
        and lower.axis == (0.0, 0.0, 1.0)
        and isclose(upper_origin[0], lower_origin[0], abs_tol=1e-6)
        and isclose(upper_origin[1], lower_origin[1], abs_tol=1e-6)
        and abs(upper_origin[2] - lower_origin[2]) > 0.70,
        details=f"upper={upper_origin}, lower={lower_origin}, axes={upper.axis}/{lower.axis}",
    )
    ctx.check(
        "lower pintle follows gate swing",
        lower.mimic is not None and lower.mimic.joint == "upper_pintle",
        details=f"mimic={lower.mimic}",
    )

    ctx.expect_within(
        fixture,
        gate,
        axes="xy",
        inner_elem="upper_pintle_pin",
        outer_elem="upper_gate_barrel",
        margin=0.002,
        name="upper pin centered inside upper barrel",
    )
    ctx.expect_overlap(
        fixture,
        gate,
        axes="z",
        elem_a="upper_pintle_pin",
        elem_b="upper_gate_barrel",
        min_overlap=0.13,
        name="upper pintle retains barrel height",
    )
    ctx.expect_within(
        fixture,
        lower_hinge,
        axes="xy",
        inner_elem="lower_pintle_pin",
        outer_elem="lower_gate_barrel",
        margin=0.002,
        name="lower pin centered inside lower barrel",
    )
    ctx.expect_overlap(
        fixture,
        lower_hinge,
        axes="z",
        elem_a="lower_pintle_pin",
        elem_b="lower_gate_barrel",
        min_overlap=0.13,
        name="lower pintle retains barrel height",
    )

    ctx.expect_gap(
        fixture,
        gate,
        axis="x",
        positive_elem="keeper_face",
        negative_elem="strike_plate",
        min_gap=0.001,
        max_gap=0.006,
        name="magnetic latch closes to a small air gap",
    )
    ctx.expect_overlap(
        fixture,
        gate,
        axes="yz",
        elem_a="keeper_face",
        elem_b="strike_plate",
        min_overlap=0.045,
        name="strike plate aligns with magnetic receiver",
    )

    closed_aabb = ctx.part_element_world_aabb(gate, elem="strike_plate")
    closed_y = None if closed_aabb is None else (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
    with ctx.pose({upper: 1.20}):
        open_aabb = ctx.part_element_world_aabb(gate, elem="strike_plate")
        open_y = None if open_aabb is None else (open_aabb[0][1] + open_aabb[1][1]) * 0.5
    ctx.check(
        "gate swings outward from latch post",
        closed_y is not None and open_y is not None and open_y > closed_y + 0.50,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    return ctx.report()


object_model = build_object_model()
