from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extending_manipulator")

    dark_steel = model.material("dark_steel", rgba=(0.07, 0.075, 0.08, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.19, 0.205, 0.22, 1.0))
    brushed = model.material("brushed_machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    rail_steel = model.material("ground_slide_steel", rgba=(0.46, 0.49, 0.51, 1.0))
    black = model.material("black_oxide", rgba=(0.015, 0.017, 0.018, 1.0))
    caution = model.material("caution_stop", rgba=(0.85, 0.36, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.56, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.265, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=cast_iron,
        name="fixed_pedestal",
    )
    base.visual(
        Cylinder(radius=0.245, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.2475)),
        material=brushed,
        name="bearing_ring",
    )
    for index, (x, y) in enumerate(
        ((-0.285, -0.205), (-0.285, 0.205), (0.285, -0.205), (0.285, 0.205))
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.056)),
            material=black,
            name=f"anchor_bolt_{index}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.230, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=brushed,
        name="rotary_disk",
    )
    turntable.visual(
        Cylinder(radius=0.180, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=rail_steel,
        name="machined_face",
    )
    turntable.visual(
        Cylinder(radius=0.085, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=brushed,
        name="center_boss",
    )
    for index in range(8):
        angle = index * math.tau / 8.0
        turntable.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(0.148 * math.cos(angle), 0.148 * math.sin(angle), 0.085)),
            material=black,
            name=f"face_bolt_{index}",
        )
    turntable.visual(
        Box((0.440, 0.235, 0.0875)),
        origin=Origin(xyz=(0.020, 0.0, 0.12375)),
        material=cast_iron,
        name="carriage_pedestal",
    )

    # A true guide sleeve made from separate walls leaves a clear rectangular
    # tunnel for the sliding beam instead of representing the slide as a solid
    # proxy.
    turntable.visual(
        Box((0.620, 0.150, 0.025)),
        origin=Origin(xyz=(0.020, 0.0, 0.180)),
        material=dark_steel,
        name="sleeve_bottom",
    )
    turntable.visual(
        Box((0.620, 0.150, 0.025)),
        origin=Origin(xyz=(0.020, 0.0, 0.320)),
        material=dark_steel,
        name="sleeve_top",
    )
    turntable.visual(
        Box((0.620, 0.024, 0.165)),
        origin=Origin(xyz=(0.020, 0.075, 0.250)),
        material=dark_steel,
        name="sleeve_side_pos",
    )
    turntable.visual(
        Box((0.620, 0.024, 0.165)),
        origin=Origin(xyz=(0.020, -0.075, 0.250)),
        material=dark_steel,
        name="sleeve_side_neg",
    )
    turntable.visual(
        Box((0.035, 0.180, 0.022)),
        origin=Origin(xyz=(0.3475, 0.0, 0.3215)),
        material=black,
        name="mouth_top_wiper",
    )
    turntable.visual(
        Box((0.035, 0.180, 0.022)),
        origin=Origin(xyz=(0.3475, 0.0, 0.1785)),
        material=black,
        name="mouth_bottom_wiper",
    )
    turntable.visual(
        Box((0.035, 0.022, 0.165)),
        origin=Origin(xyz=(0.3475, 0.086, 0.250)),
        material=black,
        name="mouth_side_pos",
    )
    turntable.visual(
        Box((0.035, 0.022, 0.165)),
        origin=Origin(xyz=(0.3475, -0.086, 0.250)),
        material=black,
        name="mouth_side_neg",
    )

    beam = model.part("beam")
    beam.visual(
        Box((0.800, 0.100, 0.100)),
        origin=Origin(xyz=(-0.180, 0.0, 0.0)),
        material=rail_steel,
        name="slide_member",
    )
    beam.visual(
        Box((0.040, 0.112, 0.108)),
        origin=Origin(xyz=(-0.565, 0.0, 0.0)),
        material=caution,
        name="rear_stop",
    )
    beam.visual(
        Box((0.200, 0.055, 0.0075)),
        origin=Origin(xyz=(-0.360, 0.0, -0.05375)),
        material=black,
        name="lower_linear_pad",
    )
    beam.visual(
        Box((0.060, 0.205, 0.205)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=cast_iron,
        name="nose_mount_block",
    )
    beam.visual(
        Box((0.120, 0.190, 0.040)),
        origin=Origin(xyz=(0.280, 0.0, 0.080)),
        material=cast_iron,
        name="nose_support_top",
    )
    beam.visual(
        Box((0.120, 0.190, 0.040)),
        origin=Origin(xyz=(0.280, 0.0, -0.080)),
        material=cast_iron,
        name="nose_support_bottom",
    )
    beam.visual(
        Box((0.120, 0.040, 0.190)),
        origin=Origin(xyz=(0.280, 0.080, 0.0)),
        material=cast_iron,
        name="nose_support_pos",
    )
    beam.visual(
        Box((0.120, 0.040, 0.190)),
        origin=Origin(xyz=(0.280, -0.080, 0.0)),
        material=cast_iron,
        name="nose_support_neg",
    )
    beam.visual(
        Box((0.150, 0.080, 0.010)),
        origin=Origin(xyz=(0.140, 0.0, 0.055)),
        material=black,
        name="upper_wear_strip",
    )
    beam.visual(
        Box((0.150, 0.080, 0.010)),
        origin=Origin(xyz=(0.140, 0.0, -0.055)),
        material=black,
        name="lower_wear_strip",
    )

    nose = model.part("nose")
    nose.visual(
        Cylinder(radius=0.040, length=0.205),
        origin=Origin(xyz=(0.0725, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="nose_shaft",
    )
    nose.visual(
        Cylinder(radius=0.056, length=0.035),
        origin=Origin(xyz=(0.1925, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="front_collar",
    )
    nose.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="tool_stub",
    )
    nose.visual(
        Box((0.036, 0.018, 0.012)),
        origin=Origin(xyz=(0.1925, 0.0, 0.060)),
        material=caution,
        name="roll_index_key",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.15, friction=0.05),
    )
    model.articulation(
        "turntable_to_beam",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=beam,
        origin=Origin(xyz=(0.330, 0.0, 0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.320),
        motion_properties=MotionProperties(damping=0.35, friction=0.12),
    )
    model.articulation(
        "beam_to_nose",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=nose,
        origin=Origin(xyz=(0.250, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    beam = object_model.get_part("beam")
    nose = object_model.get_part("nose")
    rotary = object_model.get_articulation("base_to_turntable")
    slide = object_model.get_articulation("turntable_to_beam")
    roll = object_model.get_articulation("beam_to_nose")

    ctx.expect_contact(
        base,
        turntable,
        elem_a="bearing_ring",
        elem_b="rotary_disk",
        contact_tol=0.001,
        name="rotary disk is seated on the bearing ring",
    )

    def expect_slide_clearance(label: str) -> None:
        ctx.expect_gap(
            turntable,
            beam,
            axis="z",
            positive_elem="sleeve_top",
            negative_elem="slide_member",
            min_gap=0.005,
            max_gap=0.015,
            name=f"{label} slide clears upper sleeve wall",
        )
        ctx.expect_gap(
            beam,
            turntable,
            axis="z",
            positive_elem="slide_member",
            negative_elem="sleeve_bottom",
            min_gap=0.005,
            max_gap=0.015,
            name=f"{label} slide clears lower sleeve wall",
        )
        ctx.expect_gap(
            turntable,
            beam,
            axis="y",
            positive_elem="sleeve_side_pos",
            negative_elem="slide_member",
            min_gap=0.008,
            max_gap=0.025,
            name=f"{label} slide clears positive guide wall",
        )
        ctx.expect_gap(
            beam,
            turntable,
            axis="y",
            positive_elem="slide_member",
            negative_elem="sleeve_side_neg",
            min_gap=0.008,
            max_gap=0.025,
            name=f"{label} slide clears negative guide wall",
        )

    expect_slide_clearance("retracted")
    ctx.expect_overlap(
        beam,
        turntable,
        axes="x",
        elem_a="slide_member",
        elem_b="sleeve_side_pos",
        min_overlap=0.50,
        name="retracted beam remains deeply engaged in the guide sleeve",
    )
    rest_beam_position = ctx.part_world_position(beam)
    with ctx.pose({slide: 0.320}):
        expect_slide_clearance("extended")
        ctx.expect_overlap(
            beam,
            turntable,
            axes="x",
            elem_a="slide_member",
            elem_b="sleeve_side_pos",
            min_overlap=0.22,
            name="extended beam keeps retained insertion in the guide sleeve",
        )
        extended_beam_position = ctx.part_world_position(beam)
    ctx.check(
        "prismatic beam extends outward along the sleeve axis",
        rest_beam_position is not None
        and extended_beam_position is not None
        and extended_beam_position[0] > rest_beam_position[0] + 0.30,
        details=f"rest={rest_beam_position}, extended={extended_beam_position}",
    )

    def expect_nose_clearance(label: str) -> None:
        ctx.expect_gap(
            beam,
            nose,
            axis="z",
            positive_elem="nose_support_top",
            negative_elem="nose_shaft",
            min_gap=0.014,
            max_gap=0.030,
            name=f"{label} nose shaft clears upper bearing support",
        )
        ctx.expect_gap(
            nose,
            beam,
            axis="z",
            positive_elem="nose_shaft",
            negative_elem="nose_support_bottom",
            min_gap=0.014,
            max_gap=0.030,
            name=f"{label} nose shaft clears lower bearing support",
        )
        ctx.expect_gap(
            beam,
            nose,
            axis="y",
            positive_elem="nose_support_pos",
            negative_elem="nose_shaft",
            min_gap=0.014,
            max_gap=0.030,
            name=f"{label} nose shaft clears positive bearing cheek",
        )
        ctx.expect_gap(
            nose,
            beam,
            axis="y",
            positive_elem="nose_shaft",
            negative_elem="nose_support_neg",
            min_gap=0.014,
            max_gap=0.030,
            name=f"{label} nose shaft clears negative bearing cheek",
        )

    expect_nose_clearance("neutral roll")
    rest_key_aabb = ctx.part_element_world_aabb(nose, elem="roll_index_key")
    with ctx.pose({roll: math.pi / 2.0}):
        expect_nose_clearance("rolled")
        rolled_key_aabb = ctx.part_element_world_aabb(nose, elem="roll_index_key")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_key_center = aabb_center(rest_key_aabb)
    rolled_key_center = aabb_center(rolled_key_aabb)
    ctx.check(
        "roll index key visibly follows the nose rotation",
        rest_key_center is not None
        and rolled_key_center is not None
        and abs(rolled_key_center[1] - rest_key_center[1]) > 0.045
        and rolled_key_center[2] < rest_key_center[2] - 0.040,
        details=f"rest_key={rest_key_center}, rolled_key={rolled_key_center}",
    )

    with ctx.pose({rotary: math.pi / 2.0, slide: 0.200, roll: -math.pi / 3.0}):
        ctx.expect_gap(
            beam,
            base,
            axis="z",
            positive_elem="slide_member",
            negative_elem="bearing_ring",
            min_gap=0.17,
            name="rotated raised beam stays clear above the grounded base",
        )

    return ctx.report()


object_model = build_object_model()
