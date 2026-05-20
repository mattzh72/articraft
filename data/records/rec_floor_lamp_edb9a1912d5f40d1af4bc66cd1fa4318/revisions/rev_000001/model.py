from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


POLE_HEIGHT = 2.45
POLE_RADIUS = 0.018
BRACKET_HOME_Z = 0.95
BRACKET_TRAVEL = 0.75
BRACKET_HINGE_X = 0.38
BRACKET_HINGE_Z = 0.02

SHADE_AXIS_ANGLE = math.radians(115.0)
SHADE_AXIS = (math.sin(SHADE_AXIS_ANGLE), 0.0, math.cos(SHADE_AXIS_ANGLE))


def _axis_point(distance: float, start=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    return (
        start[0] + SHADE_AXIS[0] * distance,
        start[1] + SHADE_AXIS[1] * distance,
        start[2] + SHADE_AXIS[2] * distance,
    )


def _lofted_frustum(length: float, small_radius: float, large_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(small_radius)
        .workplane(offset=length)
        .circle(large_radius)
        .loft(combine=True)
    )


def _build_shade_shell() -> cq.Workplane:
    """Thin open conical shade, sloped downward from the hinge like a reading lamp."""
    length = 0.23
    outer = _lofted_frustum(length, 0.050, 0.145)
    inner = _lofted_frustum(length + 0.020, 0.043, 0.138).translate((0.0, 0.0, -0.010))
    shell = outer.cut(inner)

    small_end = _axis_point(0.095)
    return shell.rotate((0, 0, 0), (0, 1, 0), math.degrees(SHADE_AXIS_ANGLE)).translate(small_end)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_to_ceiling_reading_lamp")

    model.material("satin_black", rgba=(0.03, 0.032, 0.036, 1.0))
    model.material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    model.material("warm_white", rgba=(1.0, 0.86, 0.48, 1.0))
    model.material("shade_cream", rgba=(0.94, 0.89, 0.76, 1.0))
    model.material("rubber_dark", rgba=(0.01, 0.01, 0.012, 1.0))

    pole = model.part("tension_pole")
    pole.visual(
        Cylinder(radius=POLE_RADIUS, length=POLE_HEIGHT - 0.06),
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="pole_tube",
    )
    pole.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="rubber_dark",
        name="floor_pad",
    )
    pole.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT - 0.015)),
        material="rubber_dark",
        name="ceiling_pad",
    )
    pole.visual(
        Cylinder(radius=0.030, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT - 0.18)),
        material="satin_black",
        name="tension_sleeve",
    )
    pole.visual(
        Cylinder(radius=0.038, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT - 0.10)),
        material="satin_black",
        name="adjuster_collar",
    )

    bracket = model.part("arm_bracket")
    bracket.visual(
        Box((0.030, 0.095, 0.120)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material="satin_black",
        name="front_collar",
    )
    bracket.visual(
        Box((0.030, 0.095, 0.120)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material="satin_black",
        name="rear_collar",
    )
    bracket.visual(
        Box((0.080, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material="satin_black",
        name="side_collar_0",
    )
    bracket.visual(
        Box((0.080, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material="satin_black",
        name="side_collar_1",
    )
    bracket.visual(
        Box((0.007, 0.014, 0.095)),
        origin=Origin(xyz=(0.0215, 0.0, 0.0)),
        material="satin_black",
        name="pole_glide_x",
    )
    bracket.visual(
        Box((0.014, 0.007, 0.095)),
        origin=Origin(xyz=(0.0, 0.0215, 0.0)),
        material="satin_black",
        name="pole_glide_y",
    )
    bracket.visual(
        Box((0.320, 0.026, 0.026)),
        origin=Origin(xyz=(0.200, 0.0, 0.020)),
        material="satin_black",
        name="arm_tube",
    )
    bracket.visual(
        Box((0.270, 0.010, 0.012)),
        origin=Origin(xyz=(0.210, 0.0, 0.046)),
        material="satin_black",
        name="top_rib",
    )
    bracket.visual(
        Box((0.040, 0.078, 0.052)),
        origin=Origin(xyz=(0.338, 0.0, BRACKET_HINGE_Z)),
        material="satin_black",
        name="clevis_bridge",
    )
    bracket.visual(
        Box((0.078, 0.014, 0.052)),
        origin=Origin(xyz=(0.377, 0.032, BRACKET_HINGE_Z)),
        material="satin_black",
        name="clevis_0",
    )
    bracket.visual(
        Box((0.078, 0.014, 0.052)),
        origin=Origin(xyz=(0.377, -0.032, BRACKET_HINGE_Z)),
        material="satin_black",
        name="clevis_1",
    )
    bracket.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, -0.060, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="clamp_screw",
    )
    bracket.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.0, -0.095, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="satin_black",
        name="clamp_knob",
    )

    shade = model.part("cone_shade")
    shade.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="hinge_knuckle",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=_axis_point(0.060), rpy=(0.0, SHADE_AXIS_ANGLE, 0.0)),
        material="brushed_steel",
        name="shade_neck",
    )
    shade.visual(
        mesh_from_cadquery(_build_shade_shell(), "cone_shade_shell", tolerance=0.0008),
        material="shade_cream",
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=_axis_point(0.095), rpy=(0.0, SHADE_AXIS_ANGLE, 0.0)),
        material="brushed_steel",
        name="shade_collar",
    )
    shade.visual(
        Cylinder(radius=0.026, length=0.045),
        origin=Origin(xyz=_axis_point(0.125), rpy=(0.0, SHADE_AXIS_ANGLE, 0.0)),
        material="brushed_steel",
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=_axis_point(0.170)),
        material="warm_white",
        name="bulb",
    )

    model.articulation(
        "pole_to_bracket",
        ArticulationType.PRISMATIC,
        parent=pole,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=BRACKET_TRAVEL),
    )
    model.articulation(
        "bracket_to_shade",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=shade,
        origin=Origin(xyz=(BRACKET_HINGE_X, 0.0, BRACKET_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.65, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pole = object_model.get_part("tension_pole")
    bracket = object_model.get_part("arm_bracket")
    shade = object_model.get_part("cone_shade")
    slide = object_model.get_articulation("pole_to_bracket")
    tilt = object_model.get_articulation("bracket_to_shade")

    ctx.check(
        "bracket slides on a prismatic pole joint",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.70,
        details=f"joint={slide}",
    )
    ctx.check(
        "cone shade tilts on a revolute hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"joint={tilt}",
    )

    ctx.expect_gap(
        bracket,
        pole,
        axis="x",
        positive_elem="pole_glide_x",
        negative_elem="pole_tube",
        max_gap=0.001,
        max_penetration=0.0,
        name="bracket glide touches the pole without penetrating",
    )
    ctx.expect_overlap(
        bracket,
        pole,
        axes="z",
        elem_a="front_collar",
        elem_b="pole_tube",
        min_overlap=0.10,
        name="sliding collar remains engaged around the pole",
    )
    ctx.expect_gap(
        bracket,
        shade,
        axis="y",
        positive_elem="clevis_0",
        negative_elem="hinge_knuckle",
        max_gap=0.002,
        max_penetration=0.0,
        name="upper clevis cheek captures the hinge knuckle",
    )

    rest_pos = ctx.part_world_position(bracket)
    with ctx.pose({slide: BRACKET_TRAVEL}):
        raised_pos = ctx.part_world_position(bracket)
        ctx.expect_gap(
            bracket,
            pole,
            axis="x",
            positive_elem="pole_glide_x",
            negative_elem="pole_tube",
            max_gap=0.001,
            max_penetration=0.0,
            name="raised bracket still rides on the pole",
        )
        ctx.expect_overlap(
            bracket,
            pole,
            axes="z",
            elem_a="front_collar",
            elem_b="pole_tube",
            min_overlap=0.10,
            name="raised collar remains on the tension pole",
        )
    ctx.check(
        "bracket raises along the floor-to-ceiling pole",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_bulb = ctx.part_element_world_aabb(shade, elem="bulb")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        raised_bulb = ctx.part_element_world_aabb(shade, elem="bulb")
    if rest_bulb is not None and raised_bulb is not None:
        rest_z = 0.5 * (rest_bulb[0][2] + rest_bulb[1][2])
        raised_z = 0.5 * (raised_bulb[0][2] + raised_bulb[1][2])
    else:
        rest_z = raised_z = None
    ctx.check(
        "positive shade hinge tilt raises the cone",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.05,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
