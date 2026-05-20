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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_hinge_slider_wrist")

    dark_cast = Material("dark_cast_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    blue_anodized = Material("blue_anodized_frame", rgba=(0.05, 0.19, 0.38, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    orange_carriage = Material("orange_carriage", rgba=(0.9, 0.38, 0.08, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    red_tip = Material("red_service_tip", rgba=(0.78, 0.08, 0.05, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.56, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_cast,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=brushed_steel,
        name="bearing_pad",
    )
    base.visual(
        Box((0.48, 0.30, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=black_rubber,
        name="rubber_underside",
    )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        Cylinder(radius=0.095, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=brushed_steel,
        name="turntable_disk",
    )
    pivot_frame.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=blue_anodized,
        name="pivot_column",
    )
    pivot_frame.visual(
        Box((0.060, 0.125, 0.095)),
        origin=Origin(xyz=(-0.005, 0.0, 0.145)),
        material=blue_anodized,
        name="rear_crosshead",
    )
    pivot_frame.visual(
        Box((0.050, 0.060, 0.065)),
        origin=Origin(xyz=(-0.005, 0.0, 0.090)),
        material=blue_anodized,
        name="column_web",
    )
    pivot_frame.visual(
        Box((0.385, 0.018, 0.050)),
        origin=Origin(xyz=(0.200, 0.050, 0.145)),
        material=brushed_steel,
        name="guide_rail_0",
    )
    pivot_frame.visual(
        Box((0.385, 0.018, 0.050)),
        origin=Origin(xyz=(0.200, -0.050, 0.145)),
        material=brushed_steel,
        name="guide_rail_1",
    )
    pivot_frame.visual(
        Box((0.080, 0.118, 0.012)),
        origin=Origin(xyz=(0.350, 0.0, 0.176)),
        material=blue_anodized,
        name="front_tie_bar",
    )

    extension = model.part("extension_stage")
    extension.visual(
        Box((0.360, 0.054, 0.035)),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=orange_carriage,
        name="sliding_bar",
    )
    extension.visual(
        Box((0.190, 0.014, 0.020)),
        origin=Origin(xyz=(0.170, 0.034, 0.0)),
        material=brushed_steel,
        name="linear_bearing_0",
    )
    extension.visual(
        Box((0.190, 0.014, 0.020)),
        origin=Origin(xyz=(0.170, -0.034, 0.0)),
        material=brushed_steel,
        name="linear_bearing_1",
    )
    extension.visual(
        Box((0.055, 0.072, 0.058)),
        origin=Origin(xyz=(0.382, 0.0, 0.0)),
        material=orange_carriage,
        name="wrist_block",
    )
    extension.visual(
        Box((0.070, 0.016, 0.076)),
        origin=Origin(xyz=(0.438, 0.043, 0.0)),
        material=orange_carriage,
        name="wrist_cheek_0",
    )
    extension.visual(
        Box((0.070, 0.016, 0.076)),
        origin=Origin(xyz=(0.438, -0.043, 0.0)),
        material=orange_carriage,
        name="wrist_cheek_1",
    )

    tip = model.part("tip_member")
    tip.visual(
        Cylinder(radius=0.023, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="wrist_pin",
    )
    tip.visual(
        Box((0.110, 0.034, 0.030)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=red_tip,
        name="tip_paddle",
    )
    tip.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="soft_nose",
    )

    model.articulation(
        "base_to_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pivot_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "pivot_to_extension",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=extension,
        origin=Origin(xyz=(0.040, 0.0, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.200),
    )
    model.articulation(
        "extension_to_tip",
        ArticulationType.REVOLUTE,
        parent=extension,
        child=tip,
        origin=Origin(xyz=(0.438, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.2, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    pivot = object_model.get_part("pivot_frame")
    extension = object_model.get_part("extension_stage")
    tip = object_model.get_part("tip_member")
    pivot_joint = object_model.get_articulation("base_to_pivot")
    slide_joint = object_model.get_articulation("pivot_to_extension")
    wrist_joint = object_model.get_articulation("extension_to_tip")

    ctx.check(
        "module has pivot slider wrist joints",
        len(object_model.articulations) == 3
        and pivot_joint.articulation_type == ArticulationType.REVOLUTE
        and slide_joint.articulation_type == ArticulationType.PRISMATIC
        and wrist_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "slide has compact service travel",
        slide_joint.motion_limits is not None
        and slide_joint.motion_limits.lower == 0.0
        and 0.18 <= slide_joint.motion_limits.upper <= 0.22,
        details=f"limits={slide_joint.motion_limits}",
    )

    ctx.expect_contact(
        pivot,
        base,
        elem_a="turntable_disk",
        elem_b="bearing_pad",
        name="pivot frame is seated on base bearing",
    )
    ctx.expect_contact(
        extension,
        pivot,
        elem_a="linear_bearing_0",
        elem_b="guide_rail_0",
        name="upper slide bearing rides in guide rail",
    )
    ctx.expect_contact(
        extension,
        pivot,
        elem_a="linear_bearing_1",
        elem_b="guide_rail_1",
        name="lower slide bearing rides in guide rail",
    )
    ctx.expect_contact(
        tip,
        extension,
        elem_a="wrist_pin",
        elem_b="wrist_cheek_0",
        name="wrist pin is captured by cheek",
    )

    ctx.expect_overlap(
        extension,
        pivot,
        axes="x",
        elem_a="linear_bearing_0",
        elem_b="guide_rail_0",
        min_overlap=0.12,
        name="collapsed slider remains engaged in rail",
    )

    rest_extension_pos = ctx.part_world_position(extension)
    with ctx.pose({slide_joint: 0.200}):
        extended_extension_pos = ctx.part_world_position(extension)
        ctx.expect_overlap(
            extension,
            pivot,
            axes="x",
            elem_a="linear_bearing_0",
            elem_b="guide_rail_0",
            min_overlap=0.07,
            name="extended slider remains retained by rail",
        )
    ctx.check(
        "prismatic stage extends outward",
        rest_extension_pos is not None
        and extended_extension_pos is not None
        and extended_extension_pos[0] > rest_extension_pos[0] + 0.18,
        details=f"rest={rest_extension_pos}, extended={extended_extension_pos}",
    )

    def _center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({wrist_joint: -0.8}):
        raised_tip_z = _center_z(ctx.part_element_world_aabb(tip, elem="tip_paddle"))
    with ctx.pose({wrist_joint: 0.8}):
        lowered_tip_z = _center_z(ctx.part_element_world_aabb(tip, elem="tip_paddle"))
    ctx.check(
        "wrist revolute swings the tip",
        raised_tip_z is not None and lowered_tip_z is not None and raised_tip_z > lowered_tip_z + 0.08,
        details=f"raised={raised_tip_z}, lowered={lowered_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
