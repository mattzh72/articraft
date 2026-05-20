from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float):
    """Centered vertical ring used for visible bearing collars."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_offset_axis_rotary")

    dark_iron = Material("dark_iron", rgba=(0.09, 0.10, 0.11, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.66, 0.68, 0.68, 1.0))
    bearing_bronze = Material("bearing_bronze", rgba=(0.86, 0.58, 0.22, 1.0))
    stage_blue = Material("anodized_blue", rgba=(0.08, 0.30, 0.75, 1.0))
    rubber = Material("matte_rubber", rgba=(0.015, 0.015, 0.015, 1.0))

    # Grounded first level: a broad machine base with a hollow bearing at the
    # central rotary axis.  The upper moving arm is left exposed rather than
    # enclosed by a housing.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.23, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_iron,
        name="base_disk",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.050, 0.031, 0.045), "base_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=bearing_bronze,
        name="base_bearing",
    )
    for index, (x, y) in enumerate(((0.16, 0.13), (-0.16, 0.13), (0.16, -0.13), (-0.16, -0.13))):
        base.visual(
            Cylinder(radius=0.025, length=0.010),
            origin=Origin(xyz=(x, y, 0.005)),
            material=rubber,
            name=f"foot_{index}",
        )

    # First moving rotary stage.  Its own local origin is the lower vertical
    # axis; the open side arm carries a second bearing whose axis is offset
    # laterally by 0.32 m and raised to a second level.
    side_arm = model.part("side_arm")
    side_arm.visual(
        Cylinder(radius=0.110, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brushed_steel,
        name="lower_turntable",
    )
    side_arm.visual(
        Cylinder(radius=0.021, length=0.043),
        origin=Origin(xyz=(0.0, 0.0, -0.0215)),
        material=brushed_steel,
        name="lower_spindle",
    )
    side_arm.visual(
        Box((0.350, 0.040, 0.040)),
        origin=Origin(xyz=(0.165, -0.080, 0.036)),
        material=brushed_steel,
        name="lower_side_beam",
    )
    side_arm.visual(
        Box((0.050, 0.042, 0.240)),
        origin=Origin(xyz=(0.320, -0.080, 0.155)),
        material=brushed_steel,
        name="upright",
    )
    side_arm.visual(
        Box((0.160, 0.026, 0.026)),
        origin=Origin(xyz=(0.260, -0.080, 0.240), rpy=(0.0, math.radians(-32.0), 0.0)),
        material=brushed_steel,
        name="diagonal_brace",
    )
    side_arm.visual(
        mesh_from_cadquery(_annular_cylinder(0.070, 0.028, 0.035), "upper_bearing"),
        origin=Origin(xyz=(0.320, 0.0, 0.2875)),
        material=bearing_bronze,
        name="upper_bearing",
    )

    # Second rotary stage, mounted in the offset upper bearing.  The pointer tab
    # makes the stage's independent rotation visually legible.
    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.066, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=stage_blue,
        name="stage_disk",
    )
    upper_stage.visual(
        Cylinder(radius=0.020, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, -0.0165)),
        material=brushed_steel,
        name="upper_spindle",
    )
    upper_stage.visual(
        Box((0.090, 0.018, 0.012)),
        origin=Origin(xyz=(0.070, 0.0, 0.026)),
        material=stage_blue,
        name="pointer_tab",
    )
    upper_stage.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=brushed_steel,
        name="top_cap",
    )

    model.articulation(
        "base_to_side_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=side_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "side_arm_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=side_arm,
        child=upper_stage,
        origin=Origin(xyz=(0.320, 0.0, 0.305)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    side_arm = object_model.get_part("side_arm")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("base_to_side_arm")
    upper_joint = object_model.get_articulation("side_arm_to_upper_stage")

    ctx.check(
        "two independent revolute stages",
        len(object_model.articulations) == 2
        and lower_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_joint.articulation_type == ArticulationType.REVOLUTE
        and lower_joint.mimic is None
        and upper_joint.mimic is None,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_origin_distance(
        side_arm,
        upper_stage,
        axes="xy",
        min_dist=0.30,
        max_dist=0.34,
        name="rotary axes have visible lateral offset",
    )
    ctx.expect_origin_gap(
        upper_stage,
        side_arm,
        axis="z",
        min_gap=0.29,
        max_gap=0.32,
        name="upper axis is raised above lower rotary level",
    )
    ctx.expect_gap(
        side_arm,
        base,
        axis="z",
        positive_elem="lower_turntable",
        negative_elem="base_bearing",
        max_gap=0.001,
        max_penetration=0.0005,
        name="lower turntable seats on base bearing",
    )
    ctx.expect_gap(
        upper_stage,
        side_arm,
        axis="z",
        positive_elem="stage_disk",
        negative_elem="upper_bearing",
        max_gap=0.001,
        max_penetration=0.0005,
        name="upper stage seats on offset bearing",
    )

    def aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    rest_upper_axis = ctx.part_world_position(upper_stage)
    with ctx.pose({lower_joint: math.pi / 2.0}):
        turned_upper_axis = ctx.part_world_position(upper_stage)
    ctx.check(
        "lower stage carries offset upper axis",
        rest_upper_axis is not None
        and turned_upper_axis is not None
        and turned_upper_axis[1] > rest_upper_axis[1] + 0.25
        and turned_upper_axis[0] < rest_upper_axis[0] - 0.25,
        details=f"rest={rest_upper_axis}, turned={turned_upper_axis}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(upper_stage, elem="pointer_tab")
    with ctx.pose({upper_joint: math.pi / 2.0}):
        turned_tab_aabb = ctx.part_element_world_aabb(upper_stage, elem="pointer_tab")
    rest_tab = aabb_center(rest_tab_aabb) if rest_tab_aabb is not None else None
    turned_tab = aabb_center(turned_tab_aabb) if turned_tab_aabb is not None else None
    ctx.check(
        "upper stage rotates independently",
        rest_tab is not None
        and turned_tab is not None
        and turned_tab[1] > rest_tab[1] + 0.045
        and turned_tab[0] < rest_tab[0] - 0.045,
        details=f"rest_tab={rest_tab}, turned_tab={turned_tab}",
    )

    return ctx.report()


object_model = build_object_model()
