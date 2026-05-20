from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="binocular_astronomy_yoke_mount")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    matte_black = model.material("matte_black", rgba=(0.055, 0.055, 0.060, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    anodized = model.material("anodized", rgba=(0.30, 0.32, 0.35, 1.0))
    polished = model.material("polished", rgba=(0.62, 0.64, 0.66, 1.0))
    ivory = model.material("ivory_tube", rgba=(0.86, 0.84, 0.76, 1.0))
    dark_glass = model.material("coated_glass", rgba=(0.11, 0.26, 0.34, 0.58))
    rubber = model.material("rubber", rgba=(0.018, 0.018, 0.020, 1.0))

    tripod_head = model.part("tripod_head")
    tripod_head.visual(
        Cylinder(radius=0.035, length=0.810),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=satin_black,
        name="center_column",
    )
    tripod_head.visual(
        Cylinder(radius=0.090, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=graphite,
        name="leg_hub",
    )
    tripod_head.visual(
        Cylinder(radius=0.105, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        material=graphite,
        name="tripod_head_casting",
    )
    tripod_head.visual(
        Cylinder(radius=0.140, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.030)),
        material=polished,
        name="azimuth_lower_race",
    )
    tripod_head.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        material=matte_black,
        name="head_neck",
    )
    tripod_head.visual(
        Cylinder(radius=0.080, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 1.011)),
        material=matte_black,
        name="bearing_post",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        leg = tube_from_spline_points(
            [
                (0.065 * c, 0.065 * s, 0.685),
                (0.260 * c, 0.260 * s, 0.330),
                (0.620 * c, 0.620 * s, 0.055),
            ],
            radius=0.018,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        tripod_head.visual(
            mesh_from_geometry(leg, f"tripod_leg_{index}"),
            material=satin_black,
            name=f"leg_{index}",
        )
        tripod_head.visual(
            Sphere(radius=0.028),
            origin=Origin(xyz=(0.620 * c, 0.620 * s, 0.055)),
            material=rubber,
            name=f"foot_{index}",
        )

    yoke_base = model.part("yoke_base")
    yoke_base.visual(
        Cylinder(radius=0.155, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=polished,
        name="azimuth_upper_race",
    )
    yoke_base.visual(
        Cylinder(radius=0.070, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=matte_black,
        name="rotating_pedestal",
    )
    yoke_base.visual(
        Box((0.240, 0.500, 0.080)),
        origin=Origin(xyz=(0.0, -0.220, 0.205)),
        material=graphite,
        name="arm_shoulder",
    )
    yoke_base.visual(
        Box((0.130, 0.100, 0.680)),
        origin=Origin(xyz=(0.0, -0.360, 0.420)),
        material=graphite,
        name="single_yoke_arm",
    )
    yoke_base.visual(
        Cylinder(radius=0.092, length=0.070),
        origin=Origin(xyz=(0.0, -0.335, 0.700), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="altitude_boss",
    )
    yoke_base.visual(
        Cylinder(radius=0.045, length=0.042),
        origin=Origin(xyz=(0.0, -0.291, 0.700), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="bearing_socket",
    )
    _add_member(
        yoke_base,
        (-0.070, -0.315, 0.235),
        (-0.070, -0.335, 0.620),
        0.018,
        anodized,
        name="arm_web_0",
    )
    _add_member(
        yoke_base,
        (0.070, -0.315, 0.235),
        (0.070, -0.335, 0.620),
        0.018,
        anodized,
        name="arm_web_1",
    )
    yoke_base.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.092, -0.360, 0.700), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="altitude_lock_knob",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(xyz=(0.0, 0.065, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="pivot_trunnion",
    )
    cradle.visual(
        Cylinder(radius=0.035, length=0.700),
        origin=Origin(xyz=(0.0, 0.380, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="cradle_bar",
    )
    cradle.visual(
        Box((0.620, 0.620, 0.052)),
        origin=Origin(xyz=(0.080, 0.380, 0.068)),
        material=anodized,
        name="flat_saddle_beam",
    )
    tube_y_positions = (0.225, 0.535)
    saddle_x_positions = (-0.155, 0.285)
    for tube_index, tube_y in enumerate(tube_y_positions):
        cradle.visual(
            Cylinder(radius=0.105, length=0.930),
            origin=Origin(xyz=(0.090, tube_y, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=ivory,
            name=f"tube_{tube_index}_shell",
        )
        cradle.visual(
            Cylinder(radius=0.119, length=0.078),
            origin=Origin(xyz=(0.575, tube_y, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"objective_cell_{tube_index}",
        )
        cradle.visual(
            Cylinder(radius=0.087, length=0.014),
            origin=Origin(xyz=(0.621, tube_y, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_glass,
            name=f"objective_lens_{tube_index}",
        )
        cradle.visual(
            Cylinder(radius=0.086, length=0.064),
            origin=Origin(xyz=(-0.400, tube_y, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"rear_cell_{tube_index}",
        )
        cradle.visual(
            Cylinder(radius=0.046, length=0.126),
            origin=Origin(xyz=(-0.493, tube_y, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_black,
            name=f"eyepiece_barrel_{tube_index}",
        )
        cradle.visual(
            Cylinder(radius=0.056, length=0.040),
            origin=Origin(xyz=(-0.573, tube_y, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"rubber_eyecup_{tube_index}",
        )
        for saddle_index, saddle_x in enumerate(saddle_x_positions):
            cradle.visual(
                Box((0.116, 0.100, 0.172)),
                origin=Origin(xyz=(saddle_x, tube_y, 0.121)),
                material=anodized,
                name=f"saddle_{tube_index}_{saddle_index}",
            )
            cradle.visual(
                Cylinder(radius=0.113, length=0.018),
                origin=Origin(xyz=(saddle_x, tube_y, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=matte_black,
                name=f"clamp_band_{tube_index}_{saddle_index}",
            )

    cradle.visual(
        Box((0.105, 0.205, 0.048)),
        origin=Origin(xyz=(-0.310, 0.380, 0.225)),
        material=matte_black,
        name="rear_bridge",
    )
    cradle.visual(
        Box((0.125, 0.550, 0.035)),
        origin=Origin(xyz=(0.392, 0.380, 0.165)),
        material=anodized,
        name="front_balance_plate",
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=tripod_head,
        child=yoke_base,
        origin=Origin(xyz=(0.0, 0.0, 1.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.0),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=yoke_base,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.300, 0.700)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.75, lower=-0.35, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_head = object_model.get_part("tripod_head")
    yoke_base = object_model.get_part("yoke_base")
    cradle = object_model.get_part("cradle")
    azimuth = object_model.get_articulation("azimuth_bearing")
    altitude = object_model.get_articulation("altitude_axis")

    ctx.check(
        "azimuth is continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS,
        details=f"azimuth type={azimuth.articulation_type}",
    )
    ctx.check(
        "altitude has realistic stops",
        altitude.articulation_type == ArticulationType.REVOLUTE
        and altitude.motion_limits is not None
        and altitude.motion_limits.lower < -0.2
        and altitude.motion_limits.upper > 1.0,
        details=f"altitude={altitude}",
    )

    ctx.expect_gap(
        yoke_base,
        tripod_head,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="azimuth_upper_race",
        negative_elem="azimuth_lower_race",
        name="azimuth bearing races are seated",
    )
    ctx.expect_gap(
        cradle,
        yoke_base,
        axis="y",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="pivot_trunnion",
        negative_elem="bearing_socket",
        name="cradle trunnion seats in bearing socket",
    )
    visual_names = {visual.name for visual in cradle.visuals}
    ctx.check(
        "two parallel refractor tubes are authored",
        "tube_0_shell" in visual_names and "tube_1_shell" in visual_names,
        details=f"cradle visuals={sorted(visual_names)}",
    )

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    def _aabb_center_xy(aabb) -> tuple[float, float] | None:
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)

    rest_lens = ctx.part_element_world_aabb(cradle, elem="objective_lens_0")
    with ctx.pose({altitude: 0.70}):
        raised_lens = ctx.part_element_world_aabb(cradle, elem="objective_lens_0")
    rest_z = _aabb_center_z(rest_lens)
    raised_z = _aabb_center_z(raised_lens)
    ctx.check(
        "positive altitude raises objectives",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.12,
        details=f"rest objective z={rest_z}, raised objective z={raised_z}",
    )

    rest_boss = ctx.part_element_world_aabb(yoke_base, elem="altitude_boss")
    with ctx.pose({azimuth: math.pi / 2.0}):
        slewed_boss = ctx.part_element_world_aabb(yoke_base, elem="altitude_boss")
    rest_xy = _aabb_center_xy(rest_boss)
    slewed_xy = _aabb_center_xy(slewed_boss)
    ctx.check(
        "azimuth slews the single yoke arm about the tripod head",
        rest_xy is not None
        and slewed_xy is not None
        and abs(slewed_xy[0] - rest_xy[0]) > 0.20
        and abs(slewed_xy[1] - rest_xy[1]) > 0.20,
        details=f"rest boss xy={rest_xy}, slewed boss xy={slewed_xy}",
    )

    return ctx.report()


object_model = build_object_model()
