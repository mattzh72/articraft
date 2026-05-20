from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _section_loop(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.72, 1.0))
    galvanized = model.material("galvanized", rgba=(0.60, 0.63, 0.66, 1.0))
    machinery = model.material("machinery", rgba=(0.24, 0.28, 0.31, 1.0))
    lamp_paint = model.material("lamp_paint", rgba=(0.78, 0.80, 0.76, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.85, 0.92, 0.45))

    mast_shell = mesh_from_geometry(
        section_loft(
            [
                _section_loop(0.56, 0.56, 0.06, 0.52),
                _section_loop(0.44, 0.44, 0.05, 1.80),
                _section_loop(0.30, 0.30, 0.04, 3.05),
            ]
        ),
        "mast_shell",
    )

    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.05, 0.0, 0.25),
                (0.00, 0.0, 0.35),
                (0.12, 0.0, 0.35),
                (0.21, 0.0, 0.26),
            ],
            radius=0.015,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "lamp_handle",
    )

    tower_support = model.part("tower_support")
    tower_support.visual(
        Box((1.80, 1.80, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=concrete,
        name="plinth",
    )
    tower_support.visual(
        Cylinder(radius=0.42, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=galvanized,
        name="pedestal_collar",
    )
    tower_support.visual(mast_shell, material=galvanized, name="mast_shell")
    tower_support.visual(
        Cylinder(radius=0.20, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 3.11)),
        material=machinery,
        name="mast_cap",
    )
    tower_support.visual(
        Box((0.22, 0.08, 1.25)),
        origin=Origin(xyz=(0.24, 0.0, 0.90), rpy=(0.0, -0.28, 0.0)),
        material=machinery,
        name="front_gusset",
    )
    tower_support.visual(
        Box((0.22, 0.08, 1.25)),
        origin=Origin(xyz=(-0.24, 0.0, 0.90), rpy=(0.0, 0.28, 0.0)),
        material=machinery,
        name="rear_gusset",
    )
    tower_support.inertial = Inertial.from_geometry(
        Box((1.80, 1.80, 3.23)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.615)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.17, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=machinery,
        name="turntable_drum",
    )
    pan_yoke.visual(
        Box((0.22, 0.74, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=machinery,
        name="yoke_saddle",
    )
    pan_yoke.visual(
        Box((0.10, 0.08, 0.52)),
        origin=Origin(xyz=(0.02, 0.37, 0.36), rpy=(0.0, -0.10, 0.0)),
        material=lamp_paint,
        name="left_arm",
    )
    pan_yoke.visual(
        Box((0.10, 0.08, 0.52)),
        origin=Origin(xyz=(0.02, -0.37, 0.36), rpy=(0.0, -0.10, 0.0)),
        material=lamp_paint,
        name="right_arm",
    )
    pan_yoke.visual(
        Box((0.16, 0.10, 0.14)),
        origin=Origin(xyz=(0.06, 0.39, 0.57)),
        material=lamp_paint,
        name="left_trunnion_block",
    )
    pan_yoke.visual(
        Box((0.16, 0.10, 0.14)),
        origin=Origin(xyz=(0.06, -0.39, 0.57)),
        material=lamp_paint,
        name="right_trunnion_block",
    )
    pan_yoke.visual(
        Cylinder(radius=0.065, length=0.08),
        origin=Origin(xyz=(0.06, 0.40, 0.57), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="left_bearing",
    )
    pan_yoke.visual(
        Cylinder(radius=0.065, length=0.08),
        origin=Origin(xyz=(0.06, -0.40, 0.57), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="right_bearing",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.24, 0.74, 0.72)),
        mass=120.0,
        origin=Origin(xyz=(0.03, 0.0, 0.36)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.29, length=0.44),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_paint,
        name="main_barrel",
    )
    lamp_head.visual(
        Cylinder(radius=0.34, length=0.06),
        origin=Origin(xyz=(0.35, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.28, length=0.008),
        origin=Origin(xyz=(0.382, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.20, length=0.20),
        origin=Origin(xyz=(-0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery,
        name="rear_housing",
    )
    lamp_head.visual(
        Box((0.16, 0.24, 0.20)),
        origin=Origin(xyz=(-0.34, 0.0, 0.0)),
        material=machinery,
        name="rear_ballast",
    )
    lamp_head.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.28, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, -0.28, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="right_trunnion",
    )
    lamp_head.visual(handle_mesh, material=machinery, name="carry_handle")
    lamp_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.82),
        mass=95.0,
        origin=Origin(xyz=(0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "mast_to_pan",
        ArticulationType.REVOLUTE,
        parent=tower_support,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 3.17)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.06, 0.0, 0.57)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.9,
            lower=math.radians(-18.0),
            upper=math.radians(68.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_support = object_model.get_part("tower_support")
    pan_yoke = object_model.get_part("pan_yoke")
    lamp_head = object_model.get_part("lamp_head")
    pan = object_model.get_articulation("mast_to_pan")
    tilt = object_model.get_articulation("yoke_to_head")

    ctx.expect_gap(
        pan_yoke,
        tower_support,
        axis="z",
        positive_elem="turntable_drum",
        negative_elem="mast_cap",
        min_gap=0.0,
        max_gap=0.005,
        name="pan stage sits tightly on mast cap",
    )
    ctx.expect_origin_distance(
        lamp_head,
        pan_yoke,
        axes="y",
        max_dist=0.001,
        name="lamp head stays centered between yoke arms",
    )

    rest_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))

    with ctx.pose({pan: 0.70}):
        panned_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))

    ctx.check(
        "pan motion swings head around the mast",
        rest_lens_center is not None
        and panned_lens_center is not None
        and panned_lens_center[1] > rest_lens_center[1] + 0.20
        and abs(panned_lens_center[2] - rest_lens_center[2]) < 0.03,
        details=f"rest={rest_lens_center}, panned={panned_lens_center}",
    )

    with ctx.pose({tilt: math.radians(60.0)}):
        tilted_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))

    ctx.check(
        "positive tilt raises the searchlight beam",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.18
        and tilted_lens_center[0] < rest_lens_center[0] - 0.05,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
