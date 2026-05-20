from __future__ import annotations

import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LOWER_AXIS_Z = 0.105
UPPER_AXIS_X = 0.500
UPPER_AXIS_Z = 0.143


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(
    radius: float,
    height: float,
    z0: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z0))


def _annular_cylinder_z(
    outer_radius: float,
    inner_radius: float,
    height: float,
    z0: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(cutter).translate((x, y, z0))


def _ground_base_geometry() -> cq.Workplane:
    base = _box((0.250, 0.0, 0.0175), (0.780, 0.380, 0.035))
    lower_bearing = _cylinder_z(0.130, 0.072, 0.033)

    # Four wide feet make the root part read as a grounded machine base.
    foot_size = (0.120, 0.070, 0.012)
    for x in (-0.050, 0.550):
        for y in (-0.135, 0.135):
            base = base.union(_box((x, y, -0.004), foot_size))

    # Low bearing bolt pads around the lower axis; they are fused to the top
    # plate and pedestal, not loose decorative islands.
    for x, y in ((0.0, 0.150), (0.0, -0.150), (0.115, 0.0), (-0.115, 0.0)):
        base = base.union(_cylinder_z(0.018, 0.010, 0.033, x=x, y=y))

    return base.union(lower_bearing).combine()


def _lower_platform_geometry() -> cq.Workplane:
    # The child frame lies on the lower bearing top face. Geometry is authored
    # relative to that rotary axis so the edge arm and upper bearing orbit it.
    carrier = _cylinder_z(0.160, 0.035, 0.000)

    # A thick edge arm overlaps the turntable rim slightly and carries a raised
    # pair of side ribs for a stiff, non-flimsy silhouette.
    carrier = carrier.union(_box((0.315, 0.0, 0.058), (0.410, 0.076, 0.050)))
    for y in (-0.044, 0.044):
        carrier = carrier.union(_box((0.315, y, 0.101), (0.350, 0.012, 0.038)))

    # Offset upper bearing: an annular supported axis mounted on the far end of
    # the side arm, with its top face carrying the smaller rotary stage.
    carrier = carrier.union(
        _annular_cylinder_z(0.085, 0.040, 0.062, 0.081, x=UPPER_AXIS_X)
    )

    # A small boss on the lower turntable makes the lower rotary stage read as
    # a supported platform rather than only a flat disk.
    carrier = carrier.union(_cylinder_z(0.075, 0.020, 0.033))
    return carrier.combine()


def _upper_stage_geometry() -> cq.Workplane:
    stage = _cylinder_z(0.115, 0.025, 0.000)
    stage = stage.union(_cylinder_z(0.043, 0.040, 0.023))

    # An integral pointer lug makes rotation of the smaller upper stage visibly
    # meaningful and gives the tests a non-axisymmetric pose cue.
    stage = stage.union(_box((0.055, 0.0, 0.034), (0.160, 0.026, 0.018)))
    return stage.combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="edge_arm_offset_rotary_stack")

    base_material = model.material("dark_machined_base", rgba=(0.18, 0.19, 0.20, 1.0))
    lower_material = model.material("blue_carrier_casting", rgba=(0.08, 0.24, 0.55, 1.0))
    upper_material = model.material("amber_upper_stage", rgba=(0.95, 0.50, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_ground_base_geometry(), "ground_base"),
        origin=Origin(),
        material=base_material,
        name="base_frame",
    )

    lower = model.part("lower_platform")
    lower.visual(
        mesh_from_cadquery(_lower_platform_geometry(), "lower_platform"),
        origin=Origin(),
        material=lower_material,
        name="lower_carrier",
    )

    upper = model.part("upper_stage")
    upper.visual(
        mesh_from_cadquery(_upper_stage_geometry(), "upper_stage"),
        origin=Origin(),
        material=upper_material,
        name="upper_rotor",
    )

    model.articulation(
        "lower_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    model.articulation(
        "upper_axis",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_platform")
    upper = object_model.get_part("upper_stage")
    lower_axis = object_model.get_articulation("lower_axis")
    upper_axis = object_model.get_articulation("upper_axis")

    ctx.check(
        "two parallel vertical rotary axes",
        lower_axis.axis == (0.0, 0.0, 1.0)
        and upper_axis.axis == (0.0, 0.0, 1.0)
        and lower_axis.articulation_type == ArticulationType.REVOLUTE
        and upper_axis.articulation_type == ArticulationType.REVOLUTE,
        details=f"lower={lower_axis.axis}, upper={upper_axis.axis}",
    )

    with ctx.pose({lower_axis: 0.0, upper_axis: 0.0}):
        ctx.expect_contact(
            lower,
            base,
            contact_tol=0.002,
            elem_a="lower_carrier",
            elem_b="base_frame",
            name="lower platform is seated on base bearing",
        )
        ctx.expect_contact(
            upper,
            lower,
            contact_tol=0.002,
            elem_a="upper_rotor",
            elem_b="lower_carrier",
            name="upper stage is seated on offset bearing",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            min_overlap=0.080,
            elem_a="upper_rotor",
            elem_b="lower_carrier",
            name="upper stage footprint overlaps its bearing",
        )

    rest_upper_pos = ctx.part_world_position(upper)
    with ctx.pose({lower_axis: math.pi / 2.0, upper_axis: 0.0}):
        rotated_upper_pos = ctx.part_world_position(upper)

    ctx.check(
        "lower axis carries offset arm around base",
        rest_upper_pos is not None
        and rotated_upper_pos is not None
        and abs(rest_upper_pos[0] - UPPER_AXIS_X) < 0.004
        and abs(rest_upper_pos[1]) < 0.004
        and abs(rotated_upper_pos[0]) < 0.004
        and abs(rotated_upper_pos[1] - UPPER_AXIS_X) < 0.004,
        details=f"rest={rest_upper_pos}, rotated={rotated_upper_pos}",
    )

    with ctx.pose({lower_axis: 0.0, upper_axis: 0.0}):
        upper_aabb_0 = ctx.part_world_aabb(upper)
    with ctx.pose({lower_axis: 0.0, upper_axis: math.pi / 2.0}):
        upper_aabb_90 = ctx.part_world_aabb(upper)

    ctx.check(
        "upper stage rotates independently on offset axis",
        upper_aabb_0 is not None
        and upper_aabb_90 is not None
        and upper_aabb_0[1][0] > upper_aabb_90[1][0] + 0.010
        and upper_aabb_90[1][1] > upper_aabb_0[1][1] + 0.010,
        details=f"q0={upper_aabb_0}, q90={upper_aabb_90}",
    )

    return ctx.report()


object_model = build_object_model()
