from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import pathlib
from math import pi

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd
if _safe_getcwd() == "/":
    os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


LADDER_HALF_SPAN = 0.205
BASE_RAIL_OUTER = (0.045, 0.030, 1.85)
BASE_RAIL_WALL = 0.004
FLY_RAIL_OUTER = (0.030, 0.018, 1.65)
FLY_RAIL_WALL = 0.003
BASE_RUNG_RADIUS = 0.012
FLY_RUNG_RADIUS = 0.011
BASE_RUNG_LENGTH = 0.369
FLY_RUNG_LENGTH = 0.384
GUIDE_BACK_SIZE = (0.040, 0.005, 0.170)
GUIDE_WALL_SIZE = (0.005, 0.028, 0.170)
GUIDE_BACK_Y = 0.0175
GUIDE_WALL_Y = 0.034
GUIDE_Z = 1.515
BASE_RUNG_Z = [0.18 + 0.17 * i for i in range(10)]
FLY_RUNG_Z = [0.47 + 0.195 * i for i in range(8)]
FLY_RAIL_Y = 0.036
FLY_BOTTOM_Z = 0.28


def _inward_sign(x_pos: float) -> float:
    return 1.0 if x_pos < 0.0 else -1.0


def _inner_outer_x(x_pos: float, offset: float) -> tuple[float, float]:
    inward = _inward_sign(x_pos) * offset
    return (x_pos + inward, x_pos - inward)


def _add_hollow_rail(
    part,
    *,
    prefix: str,
    x_pos: float,
    y_pos: float,
    bottom_z: float,
    outer_size: tuple[float, float, float],
    wall: float,
    material,
) -> None:
    outer_width, outer_depth, length = outer_size
    center_z = bottom_z + length * 0.5
    front_y = y_pos + (outer_depth * 0.5 - wall * 0.5)
    back_y = y_pos - (outer_depth * 0.5 - wall * 0.5)
    wall_offset = outer_width * 0.5 - wall * 0.5
    inner_x, outer_x = _inner_outer_x(x_pos, wall_offset)
    side_depth = max(outer_depth - 2.0 * wall, wall)

    part.visual(
        Box((outer_width, wall, length)),
        origin=Origin(xyz=(x_pos, front_y, center_z)),
        material=material,
        name=f"{prefix}_front_wall",
    )
    part.visual(
        Box((outer_width, wall, length)),
        origin=Origin(xyz=(x_pos, back_y, center_z)),
        material=material,
        name=f"{prefix}_back_wall",
    )
    part.visual(
        Box((wall, side_depth, length)),
        origin=Origin(xyz=(outer_x, y_pos, center_z)),
        material=material,
        name=f"{prefix}_outer_wall",
    )
    part.visual(
        Box((wall, side_depth, length)),
        origin=Origin(xyz=(inner_x, y_pos, center_z)),
        material=material,
        name=f"{prefix}_inner_wall",
    )


def _add_rungs(
    part,
    prefix: str,
    z_positions: list[float],
    length: float,
    radius: float,
    y_center: float,
    material,
) -> None:
    for index, z_pos in enumerate(z_positions, start=1):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, y_center, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=f"{prefix}_rung_{index}",
        )


def _add_guide_sleeve(part, *, side_name: str, x_pos: float, material) -> None:
    guide_offset = GUIDE_BACK_SIZE[0] * 0.5 - GUIDE_WALL_SIZE[0] * 0.5
    _, outer_x = _inner_outer_x(x_pos, guide_offset)
    part.visual(
        Box(GUIDE_BACK_SIZE),
        origin=Origin(xyz=(x_pos, GUIDE_BACK_Y, GUIDE_Z)),
        material=material,
        name=f"{side_name}_guide_back",
    )
    part.visual(
        Box(GUIDE_WALL_SIZE),
        origin=Origin(xyz=(outer_x, GUIDE_WALL_Y, GUIDE_Z)),
        material=material,
        name=f"{side_name}_guide_outer_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder", assets=ASSETS)

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    guide_material = model.material("guide_sleeve", rgba=(0.18, 0.20, 0.22, 1.0))
    foot_material = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_section")
    fly = model.part("fly_section")

    base.inertial = Inertial.from_geometry(
        Box((0.460, 0.065, 1.85)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.012, 0.925)),
    )
    fly.inertial = Inertial.from_geometry(
        Box((0.446, 0.022, 1.65)),
        mass=6.0,
        origin=Origin(xyz=(0.0, FLY_RAIL_Y, FLY_BOTTOM_Z + FLY_RAIL_OUTER[2] * 0.5)),
    )

    for side_name, x_pos in (("left", -LADDER_HALF_SPAN), ("right", LADDER_HALF_SPAN)):
        _add_hollow_rail(
            base,
            prefix=f"{side_name}_base",
            x_pos=x_pos,
            y_pos=0.0,
            bottom_z=0.0,
            outer_size=BASE_RAIL_OUTER,
            wall=BASE_RAIL_WALL,
            material=aluminum,
        )
        base.visual(
            Box((0.050, 0.034, 0.028)),
            origin=Origin(xyz=(x_pos, 0.0, 0.014)),
            material=foot_material,
            name=f"{side_name}_foot",
        )

    _add_rungs(
        base,
        "base",
        BASE_RUNG_Z,
        BASE_RUNG_LENGTH,
        BASE_RUNG_RADIUS,
        y_center=0.004,
        material=aluminum,
    )

    for side_name, x_pos in (("left", -LADDER_HALF_SPAN), ("right", LADDER_HALF_SPAN)):
        _add_guide_sleeve(base, side_name=side_name, x_pos=x_pos, material=guide_material)
        _add_hollow_rail(
            fly,
            prefix=f"{side_name}_fly",
            x_pos=x_pos,
            y_pos=FLY_RAIL_Y,
            bottom_z=FLY_BOTTOM_Z,
            outer_size=FLY_RAIL_OUTER,
            wall=FLY_RAIL_WALL,
            material=aluminum,
        )
        fly.visual(
            Box((0.036, 0.020, 0.020)),
            origin=Origin(
                xyz=(x_pos, FLY_RAIL_Y, FLY_BOTTOM_Z + FLY_RAIL_OUTER[2] - 0.010)
            ),
            material=guide_material,
            name=f"{side_name}_rail_cap",
        )

    _add_rungs(
        fly,
        "fly",
        FLY_RUNG_Z,
        FLY_RUNG_LENGTH,
        FLY_RUNG_RADIUS,
        y_center=FLY_RAIL_Y,
        material=aluminum,
    )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.75,
            lower=0.0,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    slide = object_model.get_articulation("base_to_fly")
    left_fly_back = fly.get_visual("left_fly_back_wall")
    right_fly_back = fly.get_visual("right_fly_back_wall")
    left_fly_outer = fly.get_visual("left_fly_outer_wall")
    right_fly_outer = fly.get_visual("right_fly_outer_wall")
    left_fly_inner = fly.get_visual("left_fly_inner_wall")
    right_fly_inner = fly.get_visual("right_fly_inner_wall")
    left_guide_back = base.get_visual("left_guide_back")
    right_guide_back = base.get_visual("right_guide_back")
    left_guide_outer = base.get_visual("left_guide_outer_wall")
    right_guide_outer = base.get_visual("right_guide_outer_wall")
    base_top_rung = base.get_visual("base_rung_10")
    fly_top_rung = fly.get_visual("fly_rung_8")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(name="rest_no_floating")
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose(
        overlap_tol=1e-5,
        overlap_volume_tol=1e-9,
        name="rest_no_unintended_overlaps",
    )
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=48,
        overlap_tol=1e-5,
        overlap_volume_tol=1e-9,
        name="slide_no_unintended_overlaps",
    )

    limits = slide.motion_limits
    ctx.check(
        "slide_axis_is_vertical",
        tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical prismatic axis, got {slide.axis}",
    )
    ctx.check(
        "slide_limits_are_realistic",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.70 <= limits.upper <= 0.90,
        details=f"unexpected ladder extension limits: {limits}",
    )

    ctx.expect_overlap(fly, base, axes="x", min_overlap=0.40, name="fly_stays_centered_in_width")
    ctx.expect_overlap(fly, base, axes="z", min_overlap=1.50, name="sections_have_safe_rest_overlap")
    ctx.expect_origin_distance(fly, base, axes="x", max_dist=0.001, name="fly_origin_stays_centered")
    ctx.expect_contact(
        fly,
        base,
        elem_a=left_fly_outer,
        elem_b=left_guide_outer,
        contact_tol=1e-5,
        name="left_outer_sleeve_contacts_left_fly_rail",
    )
    ctx.expect_contact(
        fly,
        base,
        elem_a=right_fly_outer,
        elem_b=right_guide_outer,
        contact_tol=1e-5,
        name="right_outer_sleeve_contacts_right_fly_rail",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        min_gap=0.006,
        max_gap=0.008,
        positive_elem=left_fly_back,
        negative_elem=left_guide_back,
        name="left_sleeve_back_clearance",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        min_gap=0.006,
        max_gap=0.008,
        positive_elem=right_fly_back,
        negative_elem=right_guide_back,
        name="right_sleeve_back_clearance",
    )
    ctx.expect_within(
        fly,
        base,
        axes="x",
        inner_elem=left_fly_back,
        outer_elem=left_guide_back,
        margin=0.0,
        name="left_fly_rail_centered_in_left_sleeve",
    )
    ctx.expect_within(
        fly,
        base,
        axes="x",
        inner_elem=right_fly_back,
        outer_elem=right_guide_back,
        margin=0.0,
        name="right_fly_rail_centered_in_right_sleeve",
    )
    ctx.expect_overlap(
        base,
        fly,
        axes="z",
        min_overlap=0.12,
        elem_a=left_guide_back,
        elem_b=left_fly_back,
        name="left_sleeve_engages_left_fly_rail",
    )
    ctx.expect_overlap(
        base,
        fly,
        axes="z",
        min_overlap=0.12,
        elem_a=right_guide_back,
        elem_b=right_fly_back,
        name="right_sleeve_engages_right_fly_rail",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="z",
        min_gap=0.09,
        positive_elem=fly_top_rung,
        negative_elem=base_top_rung,
        name="fly_top_rung_sits_above_base_top_rung",
    )

    if limits is not None and limits.upper is not None:
        with ctx.pose({slide: limits.upper}):
            ctx.fail_if_isolated_parts(name="extended_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(
                overlap_tol=1e-5,
                overlap_volume_tol=1e-9,
                name="extended_no_unintended_overlaps",
            )
            ctx.expect_overlap(
                fly,
                base,
                axes="x",
                min_overlap=0.40,
                name="fly_stays_centered_in_width_extended",
            )
            ctx.expect_overlap(
                fly,
                base,
                axes="z",
                min_overlap=0.75,
                name="sections_keep_safe_overlap_extended",
            )
            ctx.expect_contact(
                fly,
                base,
                elem_a=left_fly_outer,
                elem_b=left_guide_outer,
                contact_tol=1e-5,
                name="left_outer_sleeve_contacts_left_fly_rail_extended",
            )
            ctx.expect_contact(
                fly,
                base,
                elem_a=right_fly_outer,
                elem_b=right_guide_outer,
                contact_tol=1e-5,
                name="right_outer_sleeve_contacts_right_fly_rail_extended",
            )
            ctx.expect_gap(
                fly,
                base,
                axis="y",
                min_gap=0.006,
                max_gap=0.008,
                positive_elem=left_fly_back,
                negative_elem=left_guide_back,
                name="left_sleeve_back_clearance_extended",
            )
            ctx.expect_gap(
                fly,
                base,
                axis="y",
                min_gap=0.006,
                max_gap=0.008,
                positive_elem=right_fly_back,
                negative_elem=right_guide_back,
                name="right_sleeve_back_clearance_extended",
            )
            ctx.expect_overlap(
                base,
                fly,
                axes="z",
                min_overlap=0.12,
                elem_a=left_guide_back,
                elem_b=left_fly_back,
                name="left_sleeve_engages_left_fly_rail_extended",
            )
            ctx.expect_overlap(
                base,
                fly,
                axes="z",
                min_overlap=0.12,
                elem_a=right_guide_back,
                elem_b=right_fly_back,
                name="right_sleeve_engages_right_fly_rail_extended",
            )
            ctx.expect_gap(
                fly,
                base,
                axis="z",
                min_gap=0.88,
                positive_elem=fly_top_rung,
                negative_elem=base_top_rung,
                name="fly_section_extends_high_above_base",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
