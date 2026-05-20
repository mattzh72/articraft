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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder_x(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A through-bored cylindrical shell whose axis is the local X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _aabb_center_z(aabb) -> float:
    return (aabb[0][2] + aabb[1][2]) * 0.5


def _aabb_center_xy(aabb) -> tuple[float, float]:
    return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_tabletop_reflector")

    navy = model.material("dark_navy_tube", rgba=(0.02, 0.035, 0.07, 1.0))
    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.005, 1.0))
    pale_wood = model.material("pale_wood", rgba=(0.72, 0.55, 0.34, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    bearing_gray = model.material("bearing_gray", rgba=(0.42, 0.43, 0.42, 1.0))
    mirror = model.material("mirror_glass", rgba=(0.70, 0.86, 0.95, 1.0))
    focuser = model.material("focuser_black", rgba=(0.02, 0.02, 0.018, 1.0))

    tube_radius = 0.082
    tube_inner_radius = 0.070
    tube_length = 0.400
    tube_center_x = 0.020
    pivot_z = 0.360

    tabletop_disc = model.part("tabletop_disc")
    tabletop_disc.visual(
        Cylinder(radius=0.240, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=pale_wood,
        name="round_table_base",
    )
    tabletop_disc.visual(
        Cylinder(radius=0.155, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=bearing_gray,
        name="azimuth_bearing_ring",
    )
    tabletop_disc.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=rubber,
        name="center_bearing_button",
    )

    rocker = model.part("rocker_box")
    rocker.visual(
        Box((0.380, 0.340, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=pale_wood,
        name="rocker_floor",
    )
    rocker.visual(
        Box((0.340, 0.030, 0.420)),
        origin=Origin(xyz=(0.0, 0.150, 0.2445)),
        material=pale_wood,
        name="side_wall_0",
    )
    rocker.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.0, 0.131, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="bearing_pad_0",
    )
    rocker.visual(
        Box((0.340, 0.030, 0.420)),
        origin=Origin(xyz=(0.0, -0.150, 0.2445)),
        material=pale_wood,
        name="side_wall_1",
    )
    rocker.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.0, -0.131, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="bearing_pad_1",
    )
    rocker.visual(
        Box((0.040, 0.280, 0.060)),
        origin=Origin(xyz=(-0.160, 0.0, 0.080)),
        material=pale_wood,
        name="rear_cross_brace",
    )
    rocker.visual(
        Box((0.040, 0.280, 0.060)),
        origin=Origin(xyz=(0.160, 0.0, 0.080)),
        material=pale_wood,
        name="front_cross_brace",
    )

    tube = model.part("reflector_tube")
    tube.visual(
        mesh_from_cadquery(
            _annular_cylinder_x(tube_radius, tube_inner_radius, tube_length),
            "tube_shell",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(tube_center_x, 0.0, 0.0)),
        material=navy,
        name="tube_shell",
    )
    tube.visual(
        mesh_from_cadquery(
            _annular_cylinder_x(tube_radius + 0.006, tube_inner_radius - 0.004, 0.018),
            "front_rim",
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(tube_center_x + tube_length / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="front_rim",
    )
    tube.visual(
        Cylinder(radius=tube_inner_radius - 0.010, length=0.008),
        origin=Origin(xyz=(tube_center_x - tube_length / 2.0 + 0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror,
        name="primary_mirror",
    )
    tube.visual(
        Cylinder(radius=tube_radius + 0.003, length=0.012),
        origin=Origin(xyz=(tube_center_x - tube_length / 2.0 - 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_cell",
    )
    tube.visual(
        Box((0.010, 0.160, 0.004)),
        origin=Origin(xyz=(tube_center_x + tube_length / 2.0 - 0.010, 0.0, 0.0)),
        material=matte_black,
        name="spider_vane_0",
    )
    tube.visual(
        Box((0.010, 0.004, 0.160)),
        origin=Origin(xyz=(tube_center_x + tube_length / 2.0 - 0.010, 0.0, 0.0)),
        material=matte_black,
        name="spider_vane_1",
    )
    tube.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(tube_center_x + tube_length / 2.0 - 0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror,
        name="secondary_mirror",
    )
    tube.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.1075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_gray,
        name="bearing_stub_0",
    )
    tube.visual(
        Cylinder(radius=0.035, length=0.030),
        origin=Origin(xyz=(0.0, 0.092, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=navy,
        name="bearing_boss_0",
    )
    tube.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, -0.1075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_gray,
        name="bearing_stub_1",
    )
    tube.visual(
        Cylinder(radius=0.035, length=0.030),
        origin=Origin(xyz=(0.0, -0.092, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=navy,
        name="bearing_boss_1",
    )
    tube.visual(
        Box((0.055, 0.050, 0.026)),
        origin=Origin(xyz=(tube_center_x + 0.110, 0.0, tube_radius + 0.010)),
        material=focuser,
        name="focuser_block",
    )
    tube.visual(
        Cylinder(radius=0.018, length=0.065),
        origin=Origin(xyz=(tube_center_x + 0.110, 0.0, tube_radius + 0.050)),
        material=focuser,
        name="eyepiece_tube",
    )
    tube.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(tube_center_x + 0.110, 0.0, tube_radius + 0.091)),
        material=matte_black,
        name="eyepiece_cap",
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=tabletop_disc,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2),
    )
    model.articulation(
        "altitude_bearing",
        ArticulationType.REVOLUTE,
        parent=rocker,
        child=tube,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.8, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    disc = object_model.get_part("tabletop_disc")
    rocker = object_model.get_part("rocker_box")
    tube = object_model.get_part("reflector_tube")
    azimuth = object_model.get_articulation("azimuth_bearing")
    altitude = object_model.get_articulation("altitude_bearing")

    ctx.check(
        "rocker uses a continuous azimuth bearing",
        azimuth.articulation_type == ArticulationType.CONTINUOUS and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    ctx.check(
        "tube uses a limited altitude hinge",
        altitude.articulation_type == ArticulationType.REVOLUTE
        and tuple(altitude.axis) == (0.0, -1.0, 0.0)
        and altitude.motion_limits is not None
        and altitude.motion_limits.upper is not None
        and altitude.motion_limits.upper > 1.0,
        details=f"type={altitude.articulation_type}, axis={altitude.axis}, limits={altitude.motion_limits}",
    )
    ctx.expect_gap(
        rocker,
        disc,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="rocker floor sits on the tabletop azimuth bearing",
    )
    ctx.expect_gap(
        rocker,
        tube,
        axis="y",
        positive_elem="bearing_pad_0",
        negative_elem="bearing_stub_0",
        max_gap=0.003,
        max_penetration=0.0,
        name="positive altitude stub rides on its side pad",
    )
    ctx.expect_gap(
        tube,
        rocker,
        axis="y",
        positive_elem="bearing_stub_1",
        negative_elem="bearing_pad_1",
        max_gap=0.003,
        max_penetration=0.0,
        name="negative altitude stub rides on its side pad",
    )
    ctx.expect_overlap(
        tube,
        rocker,
        axes="xz",
        elem_a="bearing_stub_0",
        elem_b="bearing_pad_0",
        min_overlap=0.045,
        name="positive altitude bearing is centered in its pad",
    )
    ctx.expect_overlap(
        tube,
        rocker,
        axes="xz",
        elem_a="bearing_stub_1",
        elem_b="bearing_pad_1",
        min_overlap=0.045,
        name="negative altitude bearing is centered in its pad",
    )

    rest_front = ctx.part_element_world_aabb(tube, elem="front_rim")
    rest_pad = ctx.part_element_world_aabb(rocker, elem="bearing_pad_0")
    with ctx.pose({altitude: altitude.motion_limits.upper}):
        raised_front = ctx.part_element_world_aabb(tube, elem="front_rim")
    ctx.check(
        "front aperture rises when altitude increases",
        rest_front is not None
        and raised_front is not None
        and _aabb_center_z(raised_front) > _aabb_center_z(rest_front) + 0.10,
        details=f"rest={rest_front}, raised={raised_front}",
    )

    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_pad = ctx.part_element_world_aabb(rocker, elem="bearing_pad_0")
    if rest_pad is not None and turned_pad is not None:
        rest_x, rest_y = _aabb_center_xy(rest_pad)
        turned_x, turned_y = _aabb_center_xy(turned_pad)
    else:
        rest_x = rest_y = turned_x = turned_y = 0.0
    ctx.check(
        "rocker box turns around the tabletop disc",
        rest_pad is not None and turned_pad is not None and rest_y > 0.10 and turned_x < -0.10 and abs(turned_y) < 0.02,
        details=f"rest=({rest_x:.3f}, {rest_y:.3f}), turned=({turned_x:.3f}, {turned_y:.3f})",
    )

    return ctx.report()


object_model = build_object_model()
