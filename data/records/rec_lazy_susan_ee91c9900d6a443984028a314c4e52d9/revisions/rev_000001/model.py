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
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _annular_plate(outer_radius: float, inner_radius: float, height: float, *, segments: int = 96):
    """Low cylindrical annulus with flat top/bottom faces."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _rounded_plate(width: float, depth: float, thickness: float, radius: float):
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, depth, radius, corner_segments=8),
        thickness,
        cap=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_monitor_stand_lazy_susan")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    charcoal = model.material("charcoal_powdercoat", rgba=(0.08, 0.085, 0.09, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.58, 0.54, 1.0))
    bearing_steel = model.material("polished_bearing_steel", rgba=(0.82, 0.82, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.20, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=charcoal,
        name="bottom_disc",
    )
    base.visual(
        mesh_from_geometry(_annular_plate(0.148, 0.088, 0.017), "lower_turntable_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_steel,
        name="lower_turntable_ring",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.0415)),
        material=brushed_steel,
        name="center_bearing_boss",
    )
    base.visual(
        mesh_from_geometry(_annular_plate(0.133, 0.108, 0.004, segments=96), "bearing_cage"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_rubber,
        name="bearing_cage",
    )

    for index in range(16):
        theta = 2.0 * math.pi * index / 16.0
        base.visual(
            Sphere(radius=0.0075),
            origin=Origin(xyz=(0.120 * math.cos(theta), 0.120 * math.sin(theta), 0.0495)),
            material=bearing_steel,
            name=f"bearing_ball_{index}",
        )

    top_platform = model.part("top_platform")
    top_platform.visual(
        Cylinder(radius=0.148, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=brushed_steel,
        name="upper_bearing_disc",
    )
    top_platform.visual(
        mesh_from_geometry(_rounded_plate(0.64, 0.34, 0.045, 0.018), "top_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=matte_black,
        name="top_plate",
    )
    top_platform.visual(
        mesh_from_geometry(_rounded_plate(0.56, 0.26, 0.004, 0.012), "top_grip_mat"),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=dark_rubber,
        name="top_grip_mat",
    )

    model.articulation(
        "center_bearing",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_platform = object_model.get_part("top_platform")
    bearing = object_model.get_articulation("center_bearing")

    ctx.check(
        "top uses continuous vertical bearing",
        bearing.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in bearing.axis) == (0.0, 0.0, 1.0),
        details=f"type={bearing.articulation_type}, axis={bearing.axis}",
    )
    ctx.expect_contact(
        top_platform,
        base,
        elem_a="upper_bearing_disc",
        elem_b="center_bearing_boss",
        contact_tol=1e-5,
        name="upper disc rests on center bearing",
    )
    ctx.expect_overlap(
        top_platform,
        base,
        axes="xy",
        elem_a="upper_bearing_disc",
        elem_b="center_bearing_boss",
        min_overlap=0.09,
        name="bearing parts are concentrically nested",
    )
    ctx.expect_overlap(
        top_platform,
        base,
        axes="xy",
        elem_a="top_plate",
        elem_b="bottom_disc",
        min_overlap=0.30,
        name="broad rectangular plate is centered over round base",
    )

    rest_position = ctx.part_world_position(top_platform)
    with ctx.pose({bearing: math.pi / 2.0}):
        turned_position = ctx.part_world_position(top_platform)
        ctx.expect_contact(
            top_platform,
            base,
            elem_a="upper_bearing_disc",
            elem_b="center_bearing_boss",
            contact_tol=1e-5,
            name="bearing contact remains through rotation",
        )
    ctx.check(
        "rotation keeps platform centered",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
