from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalance_drafting_floor_lamp")

    dark_metal = Material("satin_black_metal", rgba=(0.015, 0.015, 0.013, 1.0))
    brushed_metal = Material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    inner_white = Material("warm_white_enamel", rgba=(0.95, 0.90, 0.78, 1.0))
    warm_glass = Material("warm_bulb_glass", rgba=(1.0, 0.82, 0.42, 0.82))

    pivot_z = 1.385

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.195, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="rubber_foot",
    )
    stand.visual(
        Cylinder(radius=0.185, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=brushed_metal,
        name="round_base",
    )
    stand.visual(
        Cylinder(radius=0.040, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=dark_metal,
        name="base_collar",
    )
    stand.visual(
        Cylinder(radius=0.017, length=1.255),
        origin=Origin(xyz=(0.0, 0.0, 0.6725)),
        material=dark_metal,
        name="tall_post",
    )
    stand.visual(
        Box((0.120, 0.140, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 1.3075)),
        material=dark_metal,
        name="yoke_bridge",
    )
    for index, y in enumerate((-0.066, 0.066)):
        stand.visual(
            Box((0.115, 0.014, 0.140)),
            origin=Origin(xyz=(0.0, y, pivot_z)),
            material=dark_metal,
            name=f"yoke_plate_{index}",
        )
    for index, y in enumerate((-0.080, 0.080)):
        stand.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(xyz=(0.0, y, pivot_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"pivot_cap_{index}",
        )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.033, length=0.118),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_hub",
    )
    for index, y in enumerate((-0.052, 0.052)):
        boom.visual(
            Cylinder(radius=0.038, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"hub_washer_{index}",
        )
    boom.visual(
        Cylinder(radius=0.012, length=0.780),
        origin=Origin(xyz=(0.405, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_tube",
    )
    boom.visual(
        Cylinder(radius=0.010, length=0.220),
        origin=Origin(xyz=(-0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_tube",
    )
    boom.visual(
        Cylinder(radius=0.046, length=0.110),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="counterweight",
    )
    boom.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-0.340, 0.0, 0.0)),
        material=dark_metal,
        name="rear_knob",
    )
    boom.visual(
        Box((0.034, 0.045, 0.060)),
        origin=Origin(xyz=(0.740, 0.0, 0.0)),
        material=dark_metal,
        name="terminal_block",
    )
    boom.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.770, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="terminal_socket",
    )

    shade_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.055, -0.090),
            (0.072, -0.135),
            (0.100, -0.200),
            (0.128, -0.255),
        ],
        inner_profile=[
            (0.042, -0.096),
            (0.061, -0.140),
            (0.090, -0.200),
            (0.116, -0.248),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.025, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="shade_collar",
    )
    shade.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=dark_metal,
        name="swivel_ball",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.052, 0.0, -0.045)),
        material=dark_metal,
        name="neck_stem",
    )
    shade.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.052, 0.0, -0.092)),
        material=dark_metal,
        name="socket_cap",
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "shade_shell"),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=inner_white,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.052, 0.0, -0.124)),
        material=brushed_metal,
        name="bulb_stem",
    )
    shade.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.052, 0.0, -0.170)),
        material=warm_glass,
        name="bulb",
    )

    model.articulation(
        "stand_to_boom",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.45, upper=0.75),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(0.800, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")
    boom_joint = object_model.get_articulation("stand_to_boom")
    shade_joint = object_model.get_articulation("boom_to_shade")

    ctx.check(
        "boom pivot axis is horizontal",
        boom_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={boom_joint.axis}",
    )
    ctx.check(
        "shade joint axis follows terminal arm",
        shade_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={shade_joint.axis}",
    )

    ctx.expect_gap(
        stand,
        boom,
        axis="y",
        positive_elem="yoke_plate_1",
        negative_elem="pivot_hub",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="pivot hub bears against positive yoke cheek",
    )
    ctx.expect_gap(
        boom,
        stand,
        axis="y",
        positive_elem="pivot_hub",
        negative_elem="yoke_plate_0",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="pivot hub bears against negative yoke cheek",
    )
    ctx.expect_overlap(
        boom,
        stand,
        axes="xz",
        elem_a="pivot_hub",
        elem_b="yoke_plate_0",
        min_overlap=0.050,
        name="pivot hub lies inside yoke height",
    )
    ctx.expect_gap(
        shade,
        boom,
        axis="x",
        positive_elem="shade_collar",
        negative_elem="terminal_socket",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        name="shade collar seats on boom terminal socket",
    )
    ctx.expect_within(
        shade,
        shade,
        axes="xy",
        inner_elem="bulb",
        outer_elem="shade_shell",
        margin=0.0,
        name="bulb sits inside shade footprint",
    )

    rest_shade_position = ctx.part_world_position(shade)
    with ctx.pose({boom_joint: 0.60}):
        raised_shade_position = ctx.part_world_position(shade)
    ctx.check(
        "boom pivot raises the lamp head",
        rest_shade_position is not None
        and raised_shade_position is not None
        and raised_shade_position[2] > rest_shade_position[2] + 0.35,
        details=f"rest={rest_shade_position}, raised={raised_shade_position}",
    )

    def element_center(part, elem):
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lower, upper = box
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_bulb_center = element_center(shade, "bulb")
    rest_origin = ctx.part_world_position(shade)
    with ctx.pose({shade_joint: 0.80}):
        rotated_bulb_center = element_center(shade, "bulb")
        rotated_origin = ctx.part_world_position(shade)
    ctx.check(
        "shade rotates around fixed terminal arm axis",
        rest_bulb_center is not None
        and rotated_bulb_center is not None
        and rest_origin is not None
        and rotated_origin is not None
        and abs(rotated_bulb_center[1] - rest_bulb_center[1]) > 0.08
        and abs(rotated_origin[0] - rest_origin[0]) < 1e-6
        and abs(rotated_origin[1] - rest_origin[1]) < 1e-6
        and abs(rotated_origin[2] - rest_origin[2]) < 1e-6,
        details=(
            f"rest_bulb={rest_bulb_center}, rotated_bulb={rotated_bulb_center}, "
            f"rest_origin={rest_origin}, rotated_origin={rotated_origin}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
