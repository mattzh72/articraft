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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_floodlight_mast")

    galvanized_steel = Material("galvanized_steel", rgba=(0.58, 0.61, 0.60, 1.0))
    dark_steel = Material("dark_powder_coated_steel", rgba=(0.04, 0.045, 0.045, 1.0))
    black = Material("black_trim", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = Material("pale_prismatic_glass", rgba=(0.72, 0.88, 0.95, 0.55))
    warm_led = Material("warm_led_array", rgba=(1.0, 0.78, 0.34, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.46, 0.46, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=galvanized_steel,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.044, length=2.42),
        origin=Origin(xyz=(0.0, 0.0, 1.245)),
        material=galvanized_steel,
        name="round_pole",
    )
    mast.visual(
        Cylinder(radius=0.078, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=galvanized_steel,
        name="base_collar",
    )
    mast.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.165, 0.165, 0.045)),
        material=dark_steel,
        name="anchor_bolt_0",
    )
    mast.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(-0.165, 0.165, 0.045)),
        material=dark_steel,
        name="anchor_bolt_1",
    )
    mast.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.165, -0.165, 0.045)),
        material=dark_steel,
        name="anchor_bolt_2",
    )
    mast.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(-0.165, -0.165, 0.045)),
        material=dark_steel,
        name="anchor_bolt_3",
    )
    mast.visual(
        Cylinder(radius=0.033, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 2.47), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized_steel,
        name="top_crossarm",
    )
    mast.visual(
        Cylinder(radius=0.030, length=0.48),
        origin=Origin(xyz=(0.22, 0.0, 2.40), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized_steel,
        name="front_standoff",
    )
    mast.visual(
        Box((0.105, 0.105, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 2.42)),
        material=galvanized_steel,
        name="top_hub",
    )
    mast.visual(
        Box((0.055, 0.54, 0.22)),
        origin=Origin(xyz=(0.245, 0.0, 2.32)),
        material=galvanized_steel,
        name="yoke_bridge",
    )
    mast.visual(
        Box((0.27, 0.035, 0.26)),
        origin=Origin(xyz=(0.38, 0.235, 2.30)),
        material=galvanized_steel,
        name="yoke_cheek_0",
    )
    mast.visual(
        Box((0.27, 0.035, 0.26)),
        origin=Origin(xyz=(0.38, -0.235, 2.30)),
        material=galvanized_steel,
        name="yoke_cheek_1",
    )
    mast.visual(
        Cylinder(radius=0.050, length=0.038),
        origin=Origin(xyz=(0.46, 0.235, 2.30), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_boss_0",
    )
    mast.visual(
        Cylinder(radius=0.050, length=0.038),
        origin=Origin(xyz=(0.46, -0.235, 2.30), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_boss_1",
    )

    head = model.part("floodlight_head")
    head.visual(
        Box((0.36, 0.40, 0.24)),
        origin=Origin(xyz=(0.205, 0.0, -0.035)),
        material=dark_steel,
        name="housing",
    )
    head.visual(
        Box((0.018, 0.34, 0.17)),
        origin=Origin(xyz=(0.394, 0.0, -0.035)),
        material=glass,
        name="front_glass",
    )
    head.visual(
        Box((0.012, 0.29, 0.12)),
        origin=Origin(xyz=(0.405, 0.0, -0.035)),
        material=warm_led,
        name="reflector_panel",
    )
    head.visual(
        Box((0.035, 0.44, 0.025)),
        origin=Origin(xyz=(0.385, 0.0, 0.097)),
        material=black,
        name="top_bezel",
    )
    head.visual(
        Box((0.035, 0.44, 0.025)),
        origin=Origin(xyz=(0.385, 0.0, -0.167)),
        material=black,
        name="bottom_bezel",
    )
    head.visual(
        Box((0.035, 0.025, 0.24)),
        origin=Origin(xyz=(0.385, 0.2125, -0.035)),
        material=black,
        name="side_bezel_0",
    )
    head.visual(
        Box((0.035, 0.025, 0.24)),
        origin=Origin(xyz=(0.385, -0.2125, -0.035)),
        material=black,
        name="side_bezel_1",
    )
    for index, x in enumerate((0.095, 0.145, 0.195, 0.245, 0.295)):
        head.visual(
            Box((0.020, 0.34, 0.058)),
            origin=Origin(xyz=(x, 0.0, 0.113)),
            material=black,
            name=f"cooling_fin_{index}",
        )
    head.visual(
        Cylinder(radius=0.039, length=0.041),
        origin=Origin(xyz=(0.0, 0.197, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="side_trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.039, length=0.041),
        origin=Origin(xyz=(0.0, -0.197, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="side_trunnion_1",
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.46, 0.0, 2.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    head = object_model.get_part("floodlight_head")
    tilt = object_model.get_articulation("head_tilt")

    ctx.expect_contact(
        mast,
        head,
        elem_a="yoke_cheek_0",
        elem_b="side_trunnion_0",
        contact_tol=0.004,
        name="upper yoke cheek captures trunnion",
    )
    ctx.expect_contact(
        mast,
        head,
        elem_a="yoke_cheek_1",
        elem_b="side_trunnion_1",
        contact_tol=0.004,
        name="lower yoke cheek captures trunnion",
    )
    ctx.expect_gap(
        head,
        mast,
        axis="x",
        positive_elem="housing",
        negative_elem="yoke_bridge",
        min_gap=0.010,
        name="housing clears yoke bridge",
    )

    rest_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt: 0.55}):
        raised_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt: -0.55}):
        lowered_aabb = ctx.part_world_aabb(head)

    ctx.check(
        "head tilts about bracket hinge",
        rest_aabb is not None
        and raised_aabb is not None
        and lowered_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.030
        and lowered_aabb[0][2] < rest_aabb[0][2] - 0.030,
        details=f"rest={rest_aabb}, raised={raised_aabb}, lowered={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
