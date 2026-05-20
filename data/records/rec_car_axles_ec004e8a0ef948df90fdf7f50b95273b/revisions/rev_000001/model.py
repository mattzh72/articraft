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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_live_axle")

    # --------------------------------------------------------------------------
    # Beam
    # --------------------------------------------------------------------------
    beam = model.part("beam")
    beam.visual(
        Box((1.5, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="main_bar",
    )
    beam.visual(
        Cylinder(radius=0.04, length=0.118),
        origin=Origin(xyz=(-0.75, 0.0, 0.0)),
        name="left_boss",
    )
    beam.visual(
        Cylinder(radius=0.04, length=0.118),
        origin=Origin(xyz=(0.75, 0.0, 0.0)),
        name="right_boss",
    )

    # --------------------------------------------------------------------------
    # Left Knuckle
    # --------------------------------------------------------------------------
    left_knuckle = model.part("left_knuckle")
    
    # Clevis top and bottom plates
    left_knuckle.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        name="top_plate",
    )
    left_knuckle.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        name="bottom_plate",
    )
    # Outboard back wall connecting the plates
    left_knuckle.visual(
        Box((0.04, 0.08, 0.16)),
        origin=Origin(xyz=(-0.06, 0.0, 0.0)),
        name="back_wall",
    )
    # Kingpin captured inside the beam boss
    left_knuckle.visual(
        Cylinder(radius=0.015, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="kingpin",
    )
    # Spindle extending outboard to hold the wheel hub
    left_knuckle.visual(
        Cylinder(radius=0.025, length=0.10),
        origin=Origin(xyz=(-0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="spindle",
    )

    model.articulation(
        "beam_to_left_knuckle",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=left_knuckle,
        origin=Origin(xyz=(-0.75, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=5.0, lower=-0.6, upper=0.6),
    )

    # --------------------------------------------------------------------------
    # Right Knuckle
    # --------------------------------------------------------------------------
    right_knuckle = model.part("right_knuckle")
    
    # Clevis top and bottom plates
    right_knuckle.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        name="top_plate",
    )
    right_knuckle.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        name="bottom_plate",
    )
    # Outboard back wall connecting the plates
    right_knuckle.visual(
        Box((0.04, 0.08, 0.16)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        name="back_wall",
    )
    # Kingpin captured inside the beam boss
    right_knuckle.visual(
        Cylinder(radius=0.015, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="kingpin",
    )
    # Spindle extending outboard to hold the wheel hub
    right_knuckle.visual(
        Cylinder(radius=0.025, length=0.10),
        origin=Origin(xyz=(0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="spindle",
    )

    model.articulation(
        "beam_to_right_knuckle",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=right_knuckle,
        origin=Origin(xyz=(0.75, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=5.0, lower=-0.6, upper=0.6),
    )

    # --------------------------------------------------------------------------
    # Left Hub
    # --------------------------------------------------------------------------
    left_hub = model.part("left_hub")
    
    left_hub.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="hub_body",
    )
    left_hub.visual(
        Cylinder(radius=0.12, length=0.01),
        origin=Origin(xyz=(-0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="wheel_flange",
    )
    for i, (y, z) in enumerate([(0.06, 0.0), (-0.06, 0.0), (0.0, 0.06), (0.0, -0.06)]):
        left_hub.visual(
            Cylinder(radius=0.01, length=0.02),
            origin=Origin(xyz=(-0.03, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            name=f"lug_nut_{i}",
        )

    model.articulation(
        "left_knuckle_to_hub",
        ArticulationType.CONTINUOUS,
        parent=left_knuckle,
        child=left_hub,
        origin=Origin(xyz=(-0.14, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=20.0),
    )

    # --------------------------------------------------------------------------
    # Right Hub
    # --------------------------------------------------------------------------
    right_hub = model.part("right_hub")
    
    right_hub.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="hub_body",
    )
    right_hub.visual(
        Cylinder(radius=0.12, length=0.01),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="wheel_flange",
    )
    for i, (y, z) in enumerate([(0.06, 0.0), (-0.06, 0.0), (0.0, 0.06), (0.0, -0.06)]):
        right_hub.visual(
            Cylinder(radius=0.01, length=0.02),
            origin=Origin(xyz=(0.03, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            name=f"lug_nut_{i}",
        )

    model.articulation(
        "right_knuckle_to_hub",
        ArticulationType.CONTINUOUS,
        parent=right_knuckle,
        child=right_hub,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    beam = object_model.get_part("beam")
    left_knuckle = object_model.get_part("left_knuckle")
    right_knuckle = object_model.get_part("right_knuckle")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    # Allowances for captured pins and axles
    ctx.allow_overlap(
        beam, left_knuckle,
        elem_a="left_boss", elem_b="kingpin",
        reason="The kingpin is intentionally captured inside the beam boss."
    )
    ctx.allow_overlap(
        beam, left_knuckle,
        elem_a="main_bar", elem_b="kingpin",
        reason="The kingpin is intentionally captured inside the main bar end."
    )
    ctx.allow_overlap(
        beam, right_knuckle,
        elem_a="right_boss", elem_b="kingpin",
        reason="The kingpin is intentionally captured inside the beam boss."
    )
    ctx.allow_overlap(
        beam, right_knuckle,
        elem_a="main_bar", elem_b="kingpin",
        reason="The kingpin is intentionally captured inside the main bar end."
    )
    ctx.allow_overlap(
        left_knuckle, left_hub,
        elem_a="spindle", elem_b="hub_body",
        reason="The spindle is intentionally captured inside the wheel hub."
    )
    ctx.allow_overlap(
        left_knuckle, left_hub,
        elem_a="spindle", elem_b="wheel_flange",
        reason="The spindle is intentionally captured inside the wheel flange."
    )
    ctx.allow_overlap(
        right_knuckle, right_hub,
        elem_a="spindle", elem_b="hub_body",
        reason="The spindle is intentionally captured inside the wheel hub."
    )
    ctx.allow_overlap(
        right_knuckle, right_hub,
        elem_a="spindle", elem_b="wheel_flange",
        reason="The spindle is intentionally captured inside the wheel flange."
    )

    # Proof of seating and containment
    ctx.expect_within(
        left_knuckle, beam,
        axes="xy",
        inner_elem="kingpin", outer_elem="left_boss",
        name="left kingpin stays within the boss footprint"
    )
    ctx.expect_within(
        right_knuckle, beam,
        axes="xy",
        inner_elem="kingpin", outer_elem="right_boss",
        name="right kingpin stays within the boss footprint"
    )
    
    ctx.expect_within(
        left_knuckle, left_hub,
        axes="yz",
        inner_elem="spindle", outer_elem="hub_body",
        name="left spindle stays within the hub body footprint"
    )
    ctx.expect_within(
        right_knuckle, right_hub,
        axes="yz",
        inner_elem="spindle", outer_elem="hub_body",
        name="right spindle stays within the hub body footprint"
    )

    # Gap checks for clevis plates
    ctx.expect_gap(
        left_knuckle, beam,
        axis="z",
        positive_elem="top_plate", negative_elem="left_boss",
        max_gap=0.002,
        name="left top plate sits flush on the boss"
    )
    ctx.expect_gap(
        beam, left_knuckle,
        axis="z",
        positive_elem="left_boss", negative_elem="bottom_plate",
        max_gap=0.002,
        name="left bottom plate sits flush under the boss"
    )
    
    ctx.expect_gap(
        right_knuckle, beam,
        axis="z",
        positive_elem="top_plate", negative_elem="right_boss",
        max_gap=0.002,
        name="right top plate sits flush on the boss"
    )
    ctx.expect_gap(
        beam, right_knuckle,
        axis="z",
        positive_elem="right_boss", negative_elem="bottom_plate",
        max_gap=0.002,
        name="right bottom plate sits flush under the boss"
    )

    return ctx.report()


object_model = build_object_model()
