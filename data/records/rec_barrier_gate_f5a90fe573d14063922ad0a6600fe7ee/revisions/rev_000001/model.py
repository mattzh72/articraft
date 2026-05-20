from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="folding_arm_road_barrier")

    steel = model.material("powder_coated_dark_grey", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    white = model.material("painted_white", rgba=(0.94, 0.93, 0.86, 1.0))
    red = model.material("red_reflective_panels", rgba=(0.82, 0.03, 0.025, 1.0))
    yellow = model.material("yellow_warning_label", rgba=(1.0, 0.78, 0.06, 1.0))
    amber = model.material("amber_beacon_lens", rgba=(1.0, 0.42, 0.04, 0.78))
    pin = model.material("brushed_pin_caps", rgba=(0.55, 0.57, 0.58, 1.0))

    housing = model.part("housing")
    # The part frame is the main arm pivot center.  The enclosure sits behind it
    # like a wall-mounted barrier motor cabinet, and a yoke projects forward.
    housing.visual(
        Box((0.18, 0.38, 0.74)),
        origin=Origin(xyz=(-0.19, 0.0, -0.03)),
        material=steel,
        name="cabinet",
    )
    housing.visual(
        Box((0.035, 0.48, 0.84)),
        origin=Origin(xyz=(-0.2975, 0.0, -0.03)),
        material=steel,
        name="wall_plate",
    )
    housing.visual(
        Box((0.012, 0.32, 0.55)),
        origin=Origin(xyz=(-0.091, 0.0, -0.03)),
        material=black,
        name="front_panel",
    )
    housing.visual(
        Box((0.014, 0.22, 0.09)),
        origin=Origin(xyz=(-0.086, 0.0, 0.18)),
        material=yellow,
        name="warning_label",
    )
    for z in (-0.27, 0.21):
        for y in (-0.125, 0.125):
            housing.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(xyz=(-0.080, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=pin,
                name=f"panel_screw_{z}_{y}",
            )
    housing.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(-0.19, 0.0, 0.360)),
        material=amber,
        name="beacon",
    )
    for y in (-0.135, 0.135):
        housing.visual(
            Box((0.19, 0.045, 0.26)),
            origin=Origin(xyz=(-0.005, y, 0.0)),
            material=steel,
            name=f"main_yoke_{y}",
        )
    for y in (-0.175, 0.175):
        housing.visual(
            Cylinder(radius=0.085, length=0.035),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=pin,
            name=f"main_pivot_cap_{y}",
        )

    inner_boom = model.part("inner_boom")
    inner_boom.visual(
        Cylinder(radius=0.085, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="root_hub",
    )
    inner_boom.visual(
        Box((0.16, 0.110, 0.105)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=white,
        name="root_neck",
    )
    inner_boom.visual(
        Box((1.82, 0.115, 0.105)),
        origin=Origin(xyz=(1.000, 0.0, 0.0)),
        material=white,
        name="inner_beam",
    )
    for x in (0.44, 0.84, 1.24, 1.64):
        inner_boom.visual(
            Box((0.18, 0.124, 0.114)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=red,
            name=f"inner_red_band_{x}",
        )
    for y in (-0.0775, 0.0775):
        inner_boom.visual(
            Box((0.16, 0.040, 0.220)),
            origin=Origin(xyz=(1.985, y, 0.0)),
            material=white,
            name=f"mid_yoke_{y}",
        )
    for y in (-0.110, 0.110):
        inner_boom.visual(
            Cylinder(radius=0.070, length=0.025),
            origin=Origin(xyz=(1.985, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=pin,
            name=f"mid_pivot_cap_{y}",
        )

    outer_boom = model.part("outer_boom")
    outer_boom.visual(
        Cylinder(radius=0.075, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="fold_hub",
    )
    outer_boom.visual(
        Box((0.08, 0.102, 0.105)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=white,
        name="fold_neck",
    )
    outer_boom.visual(
        Box((1.82, 0.115, 0.105)),
        origin=Origin(xyz=(1.000, 0.0, 0.0)),
        material=white,
        name="outer_beam",
    )
    for x in (0.35, 0.75, 1.15, 1.55):
        outer_boom.visual(
            Box((0.18, 0.124, 0.114)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=red,
            name=f"outer_red_band_{x}",
        )
    outer_boom.visual(
        Box((0.050, 0.130, 0.120)),
        origin=Origin(xyz=(1.935, 0.0, 0.0)),
        material=black,
        name="rubber_tip",
    )

    model.articulation(
        "housing_to_inner",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=inner_boom,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        # The inner boom extends along local +X, so -Y makes positive motion lift it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "inner_to_outer",
        ArticulationType.REVOLUTE,
        parent=inner_boom,
        child=outer_boom,
        origin=Origin(xyz=(2.0, 0.0, 0.0)),
        # Positive motion breaks the outer half downward from the straight boom.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    inner = object_model.get_part("inner_boom")
    outer = object_model.get_part("outer_boom")
    lift = object_model.get_articulation("housing_to_inner")
    fold = object_model.get_articulation("inner_to_outer")

    ctx.expect_gap(
        inner,
        housing,
        axis="x",
        max_gap=0.008,
        max_penetration=0.0005,
        positive_elem="root_hub",
        negative_elem="front_panel",
        name="main hub clears the housing face",
    )
    ctx.expect_origin_gap(
        outer,
        inner,
        axis="x",
        min_gap=1.95,
        max_gap=2.05,
        name="fold hinge is at the boom midpoint",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="yz",
        min_overlap=0.05,
        name="boom sections share the same rectangular section at rest",
    )

    rest_inner_aabb = ctx.part_world_aabb(inner)
    rest_outer_aabb = ctx.part_world_aabb(outer)
    with ctx.pose({lift: 1.20}):
        raised_inner_aabb = ctx.part_world_aabb(inner)
    with ctx.pose({fold: 1.35}):
        folded_outer_aabb = ctx.part_world_aabb(outer)

    ctx.check(
        "full arm rotates upward at the housing",
        rest_inner_aabb is not None
        and raised_inner_aabb is not None
        and raised_inner_aabb[1][2] > rest_inner_aabb[1][2] + 0.85,
        details=f"rest={rest_inner_aabb}, raised={raised_inner_aabb}",
    )
    ctx.check(
        "outer half folds downward from the midpoint hinge",
        rest_outer_aabb is not None
        and folded_outer_aabb is not None
        and folded_outer_aabb[0][2] < rest_outer_aabb[0][2] - 1.20,
        details=f"rest={rest_outer_aabb}, folded={folded_outer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
