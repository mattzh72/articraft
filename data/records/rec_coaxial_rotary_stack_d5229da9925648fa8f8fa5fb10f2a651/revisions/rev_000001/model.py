from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    chamfer: float,
    name: str,
):
    half = thickness / 2.0
    c = min(chamfer, thickness * 0.25, (outer_radius - inner_radius) * 0.25)
    profile = [
        (inner_radius + c, -half),
        (outer_radius - c, -half),
        (outer_radius, -half + c),
        (outer_radius, half - c),
        (outer_radius - c, half),
        (inner_radius + c, half),
        (inner_radius, half - c),
        (inner_radius, -half + c),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=96), name)


def _add_index_marker(
    part,
    *,
    radius: float,
    angle: float,
    radial_length: float,
    tangential_width: float,
    height: float,
    z_center: float,
    material,
    name: str,
) -> None:
    center_radius = radius - radial_length / 2.0 - 0.003
    part.visual(
        Box((radial_length, tangential_width, height)),
        origin=Origin(
            xyz=(
                center_radius * math.cos(angle),
                center_radius * math.sin(angle),
                z_center,
            ),
            rpy=(0.0, 0.0, angle),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_coaxial_turntable")

    matte_black = model.material("matte_black", rgba=(0.04, 0.045, 0.05, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.62, 0.65, 0.68, 1.0))
    lower_blue = model.material("lower_blue", rgba=(0.10, 0.22, 0.42, 1.0))
    middle_aluminum = model.material("middle_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    upper_bronze = model.material("upper_bronze", rgba=(0.74, 0.55, 0.28, 1.0))
    faceplate_gray = model.material("faceplate_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    index_white = model.material("index_white", rgba=(0.92, 0.93, 0.90, 1.0))

    lower_thickness = 0.018
    middle_thickness = 0.014
    upper_thickness = 0.012
    lower_z = 0.037
    middle_z = 0.057
    upper_z = 0.072

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.115, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="base_plinth",
    )
    base.visual(
        Cylinder(radius=0.100, length=0.0085),
        origin=Origin(xyz=(0.0, 0.0, 0.02375)),
        material=dark_steel,
        name="lower_bearing",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.0210),
        origin=Origin(xyz=(0.0, 0.0, 0.0380)),
        material=dark_steel,
        name="lower_spindle",
    )
    base.visual(
        Cylinder(radius=0.039, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0485)),
        material=bearing_steel,
        name="middle_bearing",
    )
    base.visual(
        Cylinder(radius=0.027, length=0.0150),
        origin=Origin(xyz=(0.0, 0.0, 0.0570)),
        material=dark_steel,
        name="middle_spindle",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0645)),
        material=bearing_steel,
        name="upper_bearing",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.0110),
        origin=Origin(xyz=(0.0, 0.0, 0.0715)),
        material=dark_steel,
        name="axis_nose",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        _ring_mesh(
            outer_radius=0.092,
            inner_radius=0.042,
            thickness=lower_thickness,
            chamfer=0.0020,
            name="lower_stage_ring",
        ),
        material=lower_blue,
        name="lower_ring",
    )
    _add_index_marker(
        lower_stage,
        radius=0.092,
        angle=math.radians(18.0),
        radial_length=0.020,
        tangential_width=0.008,
        height=0.004,
        z_center=lower_thickness / 2.0 + 0.002,
        material=index_white,
        name="lower_index",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        _ring_mesh(
            outer_radius=0.074,
            inner_radius=0.030,
            thickness=middle_thickness,
            chamfer=0.0016,
            name="middle_stage_ring",
        ),
        material=middle_aluminum,
        name="middle_ring",
    )
    _add_index_marker(
        middle_stage,
        radius=0.074,
        angle=math.radians(142.0),
        radial_length=0.016,
        tangential_width=0.007,
        height=0.0035,
        z_center=middle_thickness / 2.0 + 0.00175,
        material=dark_steel,
        name="middle_index",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        _ring_mesh(
            outer_radius=0.055,
            inner_radius=0.024,
            thickness=upper_thickness,
            chamfer=0.0013,
            name="upper_stage_ring",
        ),
        material=upper_bronze,
        name="upper_ring",
    )
    upper_stage.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, upper_thickness / 2.0 + 0.0025)),
        material=faceplate_gray,
        name="top_faceplate",
    )
    _add_index_marker(
        upper_stage,
        radius=0.055,
        angle=math.radians(265.0),
        radial_length=0.012,
        tangential_width=0.006,
        height=0.003,
        z_center=upper_thickness / 2.0 + 0.0012,
        material=index_white,
        name="upper_index",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, lower_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "base_to_middle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, middle_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.8, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "base_to_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, upper_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.2, lower=-2.2, upper=2.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    middle_stage = object_model.get_part("middle_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("base_to_lower")
    middle_joint = object_model.get_articulation("base_to_middle")
    upper_joint = object_model.get_articulation("base_to_upper")

    for joint in (lower_joint, middle_joint, upper_joint):
        ctx.check(
            f"{joint.name} uses the common vertical axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_within(
        upper_stage,
        middle_stage,
        axes="xy",
        inner_elem="upper_ring",
        outer_elem="middle_ring",
        margin=0.001,
        name="upper stage is nested inside middle diameter",
    )
    ctx.expect_within(
        middle_stage,
        lower_stage,
        axes="xy",
        inner_elem="middle_ring",
        outer_elem="lower_ring",
        margin=0.001,
        name="middle stage is nested inside lower diameter",
    )

    def _elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    lower_center_at_rest = _elem_center(lower_stage, "lower_index")
    middle_center_at_rest = _elem_center(middle_stage, "middle_index")
    upper_center_at_rest = _elem_center(upper_stage, "upper_index")
    with ctx.pose({lower_joint: 0.8, middle_joint: -0.6, upper_joint: 0.5}):
        lower_center_turned = _elem_center(lower_stage, "lower_index")
        middle_center_turned = _elem_center(middle_stage, "middle_index")
        upper_center_turned = _elem_center(upper_stage, "upper_index")
    ctx.check(
        "lower stage marker rotates independently",
        lower_center_at_rest is not None
        and lower_center_turned is not None
        and abs(lower_center_turned[0] - lower_center_at_rest[0]) > 0.010,
        details=f"rest={lower_center_at_rest}, turned={lower_center_turned}",
    )
    ctx.check(
        "middle stage marker rotates independently",
        middle_center_at_rest is not None
        and middle_center_turned is not None
        and abs(middle_center_turned[1] - middle_center_at_rest[1]) > 0.010,
        details=f"rest={middle_center_at_rest}, turned={middle_center_turned}",
    )
    ctx.check(
        "upper stage marker rotates independently",
        upper_center_at_rest is not None
        and upper_center_turned is not None
        and abs(upper_center_turned[0] - upper_center_at_rest[0]) > 0.004,
        details=f"rest={upper_center_at_rest}, turned={upper_center_turned}",
    )

    return ctx.report()


object_model = build_object_model()
