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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_trunnion_table")

    cast_iron = model.material("cast_iron", rgba=(0.10, 0.12, 0.13, 1.0))
    blue_gray = model.material("blue_gray_machine_paint", rgba=(0.18, 0.27, 0.32, 1.0))
    dark_rim = model.material("dark_burnished_steel", rgba=(0.05, 0.055, 0.055, 1.0))
    bright_steel = model.material("machined_steel", rgba=(0.64, 0.66, 0.62, 1.0))
    slot_dark = model.material("oiled_slot_shadow", rgba=(0.02, 0.022, 0.024, 1.0))

    body = model.part("body")
    # A broad, heavy service pedestal, grounded on four low pads.
    for x in (-0.22, 0.22):
        for y in (-0.155, 0.155):
            body.visual(
                Box((0.090, 0.070, 0.012)),
                origin=Origin(xyz=(x, y, 0.006)),
                material=dark_rim,
                name=f"foot_{x:+.0f}_{y:+.0f}",
            )
    body.visual(
        Box((0.58, 0.42, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=cast_iron,
        name="grounded_body",
    )
    body.visual(
        Box((0.42, 0.30, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=blue_gray,
        name="raised_plinth",
    )
    body.visual(
        Cylinder(radius=0.170, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=dark_rim,
        name="fixed_bearing_cap",
    )

    rotary_base = model.part("rotary_base")
    # The lower base is the indexed rotary member.  Its cheeks/yoke rotate with it.
    rotary_base.visual(
        Cylinder(radius=0.205, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=blue_gray,
        name="rotary_disk",
    )
    rotary_base.visual(
        Cylinder(radius=0.214, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_rim,
        name="outer_index_rim",
    )
    rotary_base.visual(
        Box((0.30, 0.260, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=blue_gray,
        name="saddle_block",
    )
    for idx, y in enumerate((-0.152, 0.152)):
        rotary_base.visual(
            Box((0.240, 0.030, 0.180)),
            origin=Origin(xyz=(0.0, y, 0.130)),
            material=blue_gray,
            name=f"cheek_{idx}_web",
        )
        rotary_base.visual(
            Cylinder(radius=0.066, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue_gray,
            name=f"cheek_{idx}_ear",
        )
        rotary_base.visual(
            Cylinder(radius=0.040, length=0.018),
            origin=Origin(xyz=(0.0, y + math.copysign(0.024, y), 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_rim,
            name=f"cheek_{idx}_bushing",
        )

    work_face = model.part("work_face")
    # Child frame is the trunnion axis; the face tilts about local/world Y at q=0.
    work_face.visual(
        Box((0.340, 0.240, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bright_steel,
        name="tilting_table_plate",
    )
    work_face.visual(
        Box((0.310, 0.200, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        material=bright_steel,
        name="machined_work_face",
    )
    for idx, y in enumerate((-0.060, 0.060)):
        work_face.visual(
            Box((0.265, 0.018, 0.003)),
            origin=Origin(xyz=(0.0, y, 0.023)),
            material=slot_dark,
            name=f"tee_slot_{idx}",
        )
    work_face.visual(
        Cylinder(radius=0.024, length=0.274),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rim,
        name="trunnion_bar",
    )
    for y in (-0.119, 0.119):
        work_face.visual(
            Cylinder(radius=0.031, length=0.016),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_rim,
            name=f"pin_collar_{'neg' if y < 0.0 else 'pos'}",
        )

    model.articulation(
        "body_to_rotary_base",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rotary_base,
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "rotary_base_to_work_face",
        ArticulationType.REVOLUTE,
        parent=rotary_base,
        child=work_face,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.8, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rotary_base = object_model.get_part("rotary_base")
    work_face = object_model.get_part("work_face")
    rotary_joint = object_model.get_articulation("body_to_rotary_base")
    tilt_joint = object_model.get_articulation("rotary_base_to_work_face")

    ctx.check(
        "two driven revolute mechanisms",
        rotary_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"rotary={rotary_joint.articulation_type}, tilt={tilt_joint.articulation_type}",
    )
    ctx.expect_gap(
        rotary_base,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rotary_disk",
        negative_elem="fixed_bearing_cap",
        name="rotary disk bears on fixed cap",
    )
    ctx.expect_within(
        work_face,
        rotary_base,
        axes="y",
        inner_elem="tilting_table_plate",
        outer_elem="saddle_block",
        margin=0.000,
        name="work face fits between the side cheeks",
    )
    ctx.expect_gap(
        work_face,
        rotary_base,
        axis="z",
        min_gap=0.070,
        positive_elem="tilting_table_plate",
        negative_elem="saddle_block",
        name="tilting face clears lower saddle at rest",
    )

    rest_aabb = ctx.part_element_world_aabb(work_face, elem="tilting_table_plate")
    with ctx.pose({tilt_joint: 0.45}):
        tilted_aabb = ctx.part_element_world_aabb(work_face, elem="tilting_table_plate")
        ctx.expect_gap(
            work_face,
            rotary_base,
            axis="z",
            min_gap=0.020,
            positive_elem="tilting_table_plate",
            negative_elem="saddle_block",
            name="tilted face still clears the saddle",
        )
    ctx.check(
        "trunnion tilt changes work face height envelope",
        rest_aabb is not None
        and tilted_aabb is not None
        and (tilted_aabb[1][2] - tilted_aabb[0][2]) > (rest_aabb[1][2] - rest_aabb[0][2]) + 0.035,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
