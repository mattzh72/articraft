from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _annular_collar_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    name: str,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=72)
    inner = CylinderGeometry(radius=inner_radius, height=height + 0.006, radial_segments=72)
    return mesh_from_geometry(boolean_difference(outer, inner), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_rotary_stand")

    cast_iron = model.material("cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.16, 0.17, 0.18, 1.0))
    safety_blue = model.material("safety_blue", rgba=(0.12, 0.30, 0.62, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.86, 0.39, 0.12, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.30, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_iron,
        name="heavy_base",
    )
    stand.visual(
        Cylinder(radius=0.205, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_bearing,
        name="raised_plinth",
    )
    stand.visual(
        Cylinder(radius=0.045, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=satin_steel,
        name="short_tower",
    )
    stand.visual(
        Cylinder(radius=0.058, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=dark_bearing,
        name="lower_bearing",
    )
    stand.visual(
        Cylinder(radius=0.058, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=dark_bearing,
        name="upper_bearing",
    )
    stand.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.683)),
        material=dark_bearing,
        name="top_cap",
    )
    stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=0.68),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        _annular_collar_mesh(
            outer_radius=0.096,
            inner_radius=0.056,
            height=0.068,
            name="lower_collar_mesh",
        ),
        material=safety_blue,
        name="lower_collar",
    )
    lower_arm.visual(
        Box((0.110, 0.078, 0.052)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=safety_blue,
        name="lower_weld_lug",
    )
    lower_arm.visual(
        Box((0.590, 0.052, 0.038)),
        origin=Origin(xyz=(0.365, 0.0, 0.0)),
        material=safety_blue,
        name="lower_beam",
    )
    lower_arm.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.675, 0.0, 0.0)),
        material=dark_bearing,
        name="lower_end_pad",
    )
    lower_arm.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(0.675, 0.0, 0.044)),
        material=satin_steel,
        name="lower_stop_pin",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.74, 0.14, 0.09)),
        mass=1.6,
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _annular_collar_mesh(
            outer_radius=0.092,
            inner_radius=0.056,
            height=0.064,
            name="upper_collar_mesh",
        ),
        material=safety_orange,
        name="upper_collar",
    )
    upper_arm.visual(
        Box((0.110, 0.074, 0.050)),
        origin=Origin(xyz=(0.0, 0.130, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=safety_orange,
        name="upper_weld_lug",
    )
    upper_arm.visual(
        Box((0.540, 0.048, 0.036)),
        origin=Origin(xyz=(0.0, 0.340, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=safety_orange,
        name="upper_beam",
    )
    upper_arm.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, 0.620, 0.0)),
        material=dark_bearing,
        name="upper_end_pad",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.080),
        origin=Origin(xyz=(0.0, 0.620, 0.039)),
        material=satin_steel,
        name="upper_stop_pin",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.13, 0.68, 0.09)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.32, 0.0)),
    )

    model.articulation(
        "lower_rotation",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "upper_rotation",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_rotation = object_model.get_articulation("lower_rotation")
    upper_rotation = object_model.get_articulation("upper_rotation")

    ctx.allow_overlap(
        stand,
        lower_arm,
        elem_a="lower_bearing",
        elem_b="lower_collar",
        reason="The lower collar is a captured sleeve around the tower bearing, modeled with a small local interference to show supported rotation.",
    )
    ctx.allow_overlap(
        stand,
        upper_arm,
        elem_a="upper_bearing",
        elem_b="upper_collar",
        reason="The upper collar is a captured sleeve around the tower bearing, modeled with a small local interference to show supported rotation.",
    )

    ctx.expect_overlap(
        lower_arm,
        stand,
        axes="xy",
        elem_a="lower_collar",
        elem_b="lower_bearing",
        min_overlap=0.105,
        name="lower collar encircles its bearing",
    )
    ctx.expect_overlap(
        lower_arm,
        stand,
        axes="z",
        elem_a="lower_collar",
        elem_b="lower_bearing",
        min_overlap=0.060,
        name="lower collar is supported over bearing height",
    )
    ctx.expect_overlap(
        upper_arm,
        stand,
        axes="xy",
        elem_a="upper_collar",
        elem_b="upper_bearing",
        min_overlap=0.105,
        name="upper collar encircles its bearing",
    )
    ctx.expect_overlap(
        upper_arm,
        stand,
        axes="z",
        elem_a="upper_collar",
        elem_b="upper_bearing",
        min_overlap=0.056,
        name="upper collar is supported over bearing height",
    )

    def _center_of_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    lower_rest = _center_of_aabb(ctx.part_element_world_aabb(lower_arm, elem="lower_beam"))
    upper_rest = _center_of_aabb(ctx.part_element_world_aabb(upper_arm, elem="upper_beam"))
    lower_collar_center = _center_of_aabb(ctx.part_element_world_aabb(lower_arm, elem="lower_collar"))
    upper_collar_center = _center_of_aabb(ctx.part_element_world_aabb(upper_arm, elem="upper_collar"))

    with ctx.pose({lower_rotation: math.pi / 2.0}):
        lower_rotated = _center_of_aabb(ctx.part_element_world_aabb(lower_arm, elem="lower_beam"))

    with ctx.pose({upper_rotation: -math.pi / 2.0}):
        upper_rotated = _center_of_aabb(ctx.part_element_world_aabb(upper_arm, elem="upper_beam"))

    ctx.check(
        "lower arm swings as one rigid branch",
        lower_rest is not None
        and lower_rotated is not None
        and lower_rest[0] > 0.28
        and abs(lower_rest[1]) < 0.04
        and lower_rotated[1] > 0.28
        and abs(lower_rotated[0]) < 0.06,
        details=f"rest={lower_rest}, rotated={lower_rotated}",
    )
    ctx.check(
        "upper arm swings as one rigid branch",
        upper_rest is not None
        and upper_rotated is not None
        and upper_rest[1] > 0.26
        and abs(upper_rest[0]) < 0.04
        and upper_rotated[0] > 0.26
        and abs(upper_rotated[1]) < 0.06,
        details=f"rest={upper_rest}, rotated={upper_rotated}",
    )
    ctx.check(
        "collars are separate tower levels",
        lower_collar_center is not None
        and upper_collar_center is not None
        and upper_collar_center[2] - lower_collar_center[2] > 0.15,
        details=f"lower={lower_collar_center}, upper={upper_collar_center}",
    )

    return ctx.report()


object_model = build_object_model()
