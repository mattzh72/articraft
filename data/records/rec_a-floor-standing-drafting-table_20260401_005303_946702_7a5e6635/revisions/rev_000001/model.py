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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_drafting_table")

    powder_coat = model.material("powder_coat", rgba=(0.24, 0.26, 0.28, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    beech = model.material("beech", rgba=(0.79, 0.67, 0.49, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    stand = model.part("stand")
    stand.inertial = Inertial.from_geometry(
        Box((0.90, 0.70, 1.10)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.02, 0.55)),
    )

    tube_r = 0.018
    foot_h = 0.035

    left_front_foot = (-0.50, -0.32, foot_h)
    left_rear_foot = (-0.50, 0.38, foot_h)
    right_front_foot = (0.50, -0.32, foot_h)
    right_rear_foot = (0.50, 0.38, foot_h)

    left_front_top = (-0.50, -0.22, 0.72)
    right_front_top = (0.50, -0.22, 0.72)
    left_rear_top = (-0.50, 0.22, 0.80)
    right_rear_top = (0.50, 0.22, 0.80)

    _add_tube(stand, left_front_foot, left_rear_foot, radius=tube_r, material=powder_coat, name="left_floor_rail")
    _add_tube(stand, right_front_foot, right_rear_foot, radius=tube_r, material=powder_coat, name="right_floor_rail")
    _add_tube(stand, left_front_foot, right_front_foot, radius=tube_r, material=powder_coat, name="front_floor_stretcher")
    _add_tube(stand, left_rear_foot, right_rear_foot, radius=tube_r, material=powder_coat, name="rear_floor_stretcher")

    _add_tube(stand, left_front_foot, left_front_top, radius=tube_r, material=powder_coat, name="left_front_post")
    _add_tube(stand, right_front_foot, right_front_top, radius=tube_r, material=powder_coat, name="right_front_post")
    _add_tube(stand, left_rear_foot, left_rear_top, radius=tube_r, material=powder_coat, name="left_rear_post")
    _add_tube(stand, right_rear_foot, right_rear_top, radius=tube_r, material=powder_coat, name="right_rear_post")

    _add_tube(stand, left_front_top, left_rear_top, radius=tube_r, material=powder_coat, name="left_top_rail")
    _add_tube(stand, right_front_top, right_rear_top, radius=tube_r, material=powder_coat, name="right_top_rail")
    _add_tube(stand, left_rear_top, right_rear_top, radius=tube_r, material=powder_coat, name="rear_head_rail")

    stand.visual(
        Cylinder(radius=0.012, length=0.08),
        origin=Origin(xyz=(-0.482, -0.22, 0.88), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="left_pivot_pin",
    )
    stand.visual(
        Cylinder(radius=0.012, length=0.08),
        origin=Origin(xyz=(0.482, -0.22, 0.88), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="right_pivot_pin",
    )
    stand.visual(
        Box((0.02, 0.07, 0.18)),
        origin=Origin(xyz=(-0.51, -0.22, 0.81)),
        material=powder_coat,
        name="left_hinge_support",
    )
    stand.visual(
        Box((0.02, 0.07, 0.18)),
        origin=Origin(xyz=(0.51, -0.22, 0.81)),
        material=powder_coat,
        name="right_hinge_support",
    )
    stand.visual(
        Box((1.02, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.27, 0.38)),
        material=powder_coat,
        name="front_mid_stretcher",
    )

    for name, center in (
        ("left_front_glide", left_front_foot),
        ("left_rear_glide", left_rear_foot),
        ("right_front_glide", right_front_foot),
        ("right_rear_glide", right_rear_foot),
    ):
        stand.visual(
            Box((0.05, 0.07, 0.03)),
            origin=Origin(xyz=(center[0], center[1], 0.015)),
            material=dark_rubber,
            name=name,
        )

    tabletop = model.part("tabletop")
    tabletop.inertial = Inertial.from_geometry(
        Box((1.10, 0.80, 0.10)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.40, 0.02)),
    )

    top_width = 1.06
    top_depth = 0.74
    top_thickness = 0.028
    top_surface = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(top_width, top_depth, 0.022), top_thickness),
        "drafting_table_top_surface",
    )
    tabletop.visual(
        top_surface,
        origin=Origin(xyz=(0.0, 0.39, 0.024)),
        material=beech,
        name="work_surface",
    )
    tabletop.visual(
        Box((0.94, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.725, -0.010)),
        material=powder_coat,
        name="rear_subframe",
    )
    tabletop.visual(
        Box((0.05, 0.65, 0.04)),
        origin=Origin(xyz=(-0.455, 0.375, -0.010)),
        material=powder_coat,
        name="left_subframe",
    )
    tabletop.visual(
        Box((0.05, 0.65, 0.04)),
        origin=Origin(xyz=(0.455, 0.375, -0.010)),
        material=powder_coat,
        name="right_subframe",
    )
    tabletop.visual(
        Cylinder(radius=0.020, length=0.084),
        origin=Origin(xyz=(-0.452, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="left_hinge_barrel",
    )
    tabletop.visual(
        Cylinder(radius=0.020, length=0.084),
        origin=Origin(xyz=(0.452, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="right_hinge_barrel",
    )
    tabletop.visual(
        Box((0.04, 0.15, 0.09)),
        origin=Origin(xyz=(-0.452, 0.085, -0.040)),
        material=powder_coat,
        name="left_hinge_bracket",
    )
    tabletop.visual(
        Box((0.04, 0.15, 0.09)),
        origin=Origin(xyz=(0.452, 0.085, -0.040)),
        material=powder_coat,
        name="right_hinge_bracket",
    )
    tabletop.visual(
        Box((0.86, 0.02, 0.014)),
        origin=Origin(xyz=(0.0, 0.03, 0.045)),
        material=beech,
        name="paper_stop",
    )

    rest_angle = math.radians(18.0)
    model.articulation(
        "stand_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tabletop,
        origin=Origin(xyz=(0.0, -0.22, 0.88), rpy=(rest_angle, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=math.radians(-10.0),
            upper=math.radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    tabletop = object_model.get_part("tabletop")
    tilt = object_model.get_articulation("stand_to_tabletop")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        tabletop,
        stand,
        elem_a="left_hinge_barrel",
        elem_b="left_pivot_pin",
        reason="left hinge barrel rotates around the fixed axle",
    )
    ctx.allow_overlap(
        tabletop,
        stand,
        elem_a="right_hinge_barrel",
        elem_b="right_pivot_pin",
        reason="right hinge barrel rotates around the fixed axle",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(tabletop, stand, axes="x", max_dist=0.02, name="tabletop_centered_on_stand")
    ctx.expect_overlap(
        tabletop,
        stand,
        axes="yz",
        elem_a="left_hinge_barrel",
        elem_b="left_pivot_pin",
        min_overlap=0.02,
        name="left_hinge_barrel_aligned_to_pin",
    )
    ctx.expect_overlap(
        tabletop,
        stand,
        axes="yz",
        elem_a="right_hinge_barrel",
        elem_b="right_pivot_pin",
        min_overlap=0.02,
        name="right_hinge_barrel_aligned_to_pin",
    )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        low_aabb = ctx.part_element_world_aabb(tabletop, elem="work_surface")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        high_aabb = ctx.part_element_world_aabb(tabletop, elem="work_surface")

    low_max_z = low_aabb[1][2] if low_aabb is not None else None
    high_max_z = high_aabb[1][2] if high_aabb is not None else None
    ctx.check(
        "tabletop_raises_for_drafting_pose",
        low_max_z is not None and high_max_z is not None and high_max_z > low_max_z + 0.22,
        details=f"expected upper-pose work surface max z to exceed lower pose by >0.22 m, got low={low_max_z} high={high_max_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
