from __future__ import annotations

from math import pi

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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_mounted_elbow_arm")

    dark_steel = Material("dark_powder_coated_steel", color=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.55, 0.56, 0.54, 1.0))
    safety_orange = Material("safety_orange_paint", color=(0.95, 0.33, 0.05, 1.0))
    black_rubber = Material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))
    bench_wood = Material("sealed_bench_top", color=(0.55, 0.38, 0.20, 1.0))

    shoulder_z = 0.420
    upper_len = 0.580

    base = model.part("base")
    base.visual(
        Box((0.62, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=bench_wood,
        name="bench_top",
    )
    base.visual(
        Box((0.42, 0.30, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="mounting_plate",
    )
    for ix, x in enumerate((-0.155, 0.155)):
        for iy, y in enumerate((-0.105, 0.105)):
            base.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(x, y, 0.040)),
                material=brushed_steel,
                name=f"bolt_cap_{ix}_{iy}",
            )
    base.visual(
        Cylinder(radius=0.055, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=dark_steel,
        name="base_column",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3025)),
        material=dark_steel,
        name="column_collar",
    )
    base.visual(
        Box((0.160, 0.190, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.2975)),
        material=dark_steel,
        name="shoulder_yoke_bridge",
    )
    for cheek_name, y in (("shoulder_cheek_0", -0.078), ("shoulder_cheek_1", 0.078)):
        base.visual(
            Box((0.130, 0.026, 0.202)),
            origin=Origin(xyz=(0.0, y, shoulder_z - 0.001)),
            material=dark_steel,
            name=cheek_name,
        )
    for side, y in enumerate((-0.094, 0.094)):
        base.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(0.0, y, shoulder_z), rpy=(-pi / 2, 0.0, 0.0)),
            material=brushed_steel,
            name=f"shoulder_pin_cap_{side}",
        )
    base.visual(
        Cylinder(radius=0.026, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, shoulder_z), rpy=(-pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="shoulder_pin",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.090, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=safety_orange,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.410, 0.096, 0.145)),
        origin=Origin(xyz=(0.295, 0.0, 0.0)),
        material=safety_orange,
        name="deep_box_beam",
    )
    for side, y in enumerate((-0.0495, 0.0495)):
        upper_arm.visual(
            Box((0.280, 0.006, 0.075)),
            origin=Origin(xyz=(0.295, y, 0.0)),
            material=dark_steel,
            name=f"side_recess_{side}",
        )
    for cheek_name, y in (("elbow_cheek_0", -0.058), ("elbow_cheek_1", 0.058)):
        upper_arm.visual(
            Box((0.160, 0.024, 0.150)),
            origin=Origin(xyz=(upper_len, y, 0.0)),
            material=safety_orange,
            name=cheek_name,
        )
    for side, y in enumerate((-0.075, 0.075)):
        upper_arm.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(upper_len, y, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material=brushed_steel,
            name=f"elbow_pin_cap_{side}",
        )
    upper_arm.visual(
        Cylinder(radius=0.023, length=0.150),
        origin=Origin(xyz=(upper_len, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.060, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=safety_orange,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.400, 0.060, 0.086)),
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        material=safety_orange,
        name="slim_forearm_beam",
    )
    forearm.visual(
        Box((0.038, 0.145, 0.115)),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=dark_steel,
        name="tool_face_body",
    )
    forearm.visual(
        Box((0.007, 0.122, 0.092)),
        origin=Origin(xyz=(0.4775, 0.0, 0.0)),
        material=black_rubber,
        name="tool_face_pad",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-0.35, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.25, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.check("shoulder is revolute", shoulder.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("elbow is revolute", elbow.articulation_type == ArticulationType.REVOLUTE)

    ctx.allow_overlap(
        base,
        upper_arm,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The fixed shoulder pin is intentionally captured through the rotating hub bore.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The fixed elbow pin is intentionally captured through the rotating forearm hub bore.",
    )
    ctx.expect_overlap(
        base,
        upper_arm,
        axes="xyz",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.045,
        name="shoulder pin passes through hub",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="xyz",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.040,
        name="elbow pin passes through hub",
    )

    ctx.expect_gap(
        base,
        upper_arm,
        axis="y",
        positive_elem="shoulder_cheek_1",
        negative_elem="shoulder_hub",
        min_gap=0.003,
        max_gap=0.015,
        name="shoulder hub clears positive yoke cheek",
    )
    ctx.expect_gap(
        upper_arm,
        base,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_cheek_0",
        min_gap=0.003,
        max_gap=0.015,
        name="shoulder hub clears negative yoke cheek",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_1",
        negative_elem="elbow_hub",
        min_gap=0.003,
        max_gap=0.015,
        name="elbow hub clears positive fork cheek",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_0",
        min_gap=0.003,
        max_gap=0.015,
        name="elbow hub clears negative fork cheek",
    )

    rest_forearm = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.75}):
        raised_forearm = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder rotation lifts elbow assembly",
        rest_forearm is not None
        and raised_forearm is not None
        and raised_forearm[2] > rest_forearm[2] + 0.10,
        details=f"rest={rest_forearm}, raised={raised_forearm}",
    )

    rest_tool_aabb = ctx.part_element_world_aabb(forearm, elem="tool_face_body")
    with ctx.pose({elbow: 1.0}):
        bent_tool_aabb = ctx.part_element_world_aabb(forearm, elem="tool_face_body")
    ctx.check(
        "elbow rotation moves tool face upward",
        rest_tool_aabb is not None
        and bent_tool_aabb is not None
        and bent_tool_aabb[0][2] > rest_tool_aabb[0][2] + 0.12,
        details=f"rest={rest_tool_aabb}, bent={bent_tool_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
