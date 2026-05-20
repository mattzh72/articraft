from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib
from pathlib import Path

_ORIGINAL_PATH_ABSOLUTE = pathlib.Path.absolute
_ORIGINAL_PATH_CWD = pathlib.Path.cwd


def _safe_path_absolute(self):
    try:
        return _ORIGINAL_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        if self.is_absolute():
            return self
        return pathlib.Path("/tmp") / self


@classmethod
def _safe_path_cwd(cls):
    try:
        return _ORIGINAL_PATH_CWD()
    except FileNotFoundError:
        return pathlib.Path("/tmp")


pathlib.Path.absolute = _safe_path_absolute
pathlib.PosixPath.absolute = _safe_path_absolute
pathlib.Path.cwd = _safe_path_cwd
pathlib.PosixPath.cwd = _safe_path_cwd
try:
    os.chdir("/tmp")
except FileNotFoundError:
    pass
globals()["__file__"] = "/tmp/model.py"

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

ASSETS = AssetContext(Path("/tmp"))

WALL_THICKNESS = 0.004
RUNG_RADIUS = 0.011

BASE_SPEC = {
    "section_name": "base",
    "length": 2.70,
    "outer_half_width": 0.260,
    "flange_width": 0.060,
    "depth": 0.090,
    "rung_count": 8,
    "rung_start": 0.19,
    "rung_pitch": 0.30,
    "rung_offset_y": -0.012,
}
MIDDLE_SPEC = {
    "section_name": "middle",
    "length": 2.36,
    "outer_half_width": 0.188,
    "flange_width": 0.050,
    "depth": 0.070,
    "rung_count": 7,
    "rung_start": 0.17,
    "rung_pitch": 0.30,
    "rung_offset_y": 0.0,
}
FLY_SPEC = {
    "section_name": "fly",
    "length": 2.08,
    "outer_half_width": 0.128,
    "flange_width": 0.040,
    "depth": 0.050,
    "rung_count": 6,
    "rung_start": 0.16,
    "rung_pitch": 0.30,
    "rung_offset_y": 0.012,
}

BASE_TO_MIDDLE_Z = 0.22
MIDDLE_TO_FLY_Z = 0.18
MIDDLE_EXTENSION = 1.20
FLY_EXTENSION = 1.08


def _add_section(
    part,
    *,
    section_name: str,
    length: float,
    outer_half_width: float,
    flange_width: float,
    depth: float,
    rung_count: int,
    rung_start: float,
    rung_pitch: float,
    rung_offset_y: float,
    rail_material,
    rung_material,
    hardware_material,
    parent_inner_edge: float | None = None,
    parent_depth: float | None = None,
    add_feet: bool = False,
) -> None:
    web_center_x = outer_half_width - (WALL_THICKNESS / 2.0)
    flange_center_x = outer_half_width - (flange_width / 2.0)

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((WALL_THICKNESS, depth, length)),
            origin=Origin(xyz=(sign * web_center_x, 0.0, length / 2.0)),
            material=rail_material,
            name=f"{section_name}_{side_name}_web",
        )
        part.visual(
            Box((flange_width, WALL_THICKNESS, length)),
            origin=Origin(
                xyz=(
                    sign * flange_center_x,
                    (depth / 2.0) - (WALL_THICKNESS / 2.0),
                    length / 2.0,
                )
            ),
            material=rail_material,
            name=f"{section_name}_{side_name}_front_flange",
        )
        part.visual(
            Box((flange_width, WALL_THICKNESS, length)),
            origin=Origin(
                xyz=(
                    sign * flange_center_x,
                    -(depth / 2.0) + (WALL_THICKNESS / 2.0),
                    length / 2.0,
                )
            ),
            material=rail_material,
            name=f"{section_name}_{side_name}_rear_flange",
        )

        if parent_inner_edge is not None and parent_depth is not None:
            guide_thickness = parent_inner_edge - outer_half_width
            guide_depth = ((parent_depth - depth) / 2.0) + WALL_THICKNESS
            guide_height = 0.26
            guide_center_x = outer_half_width + (guide_thickness / 2.0)
            guide_center_z = 0.20 + (guide_height / 2.0)
            guide_y = (parent_depth + depth) / 4.0
            for face_name, guide_sign in (("front", 1.0), ("rear", -1.0)):
                part.visual(
                    Box((guide_thickness, guide_depth, guide_height)),
                    origin=Origin(
                        xyz=(
                            sign * guide_center_x,
                            guide_sign * guide_y,
                            guide_center_z,
                        )
                    ),
                    material=hardware_material,
                    name=f"{section_name}_{side_name}_{face_name}_guide",
                )

    rung_length = 2.0 * web_center_x
    for index in range(rung_count):
        rung_z = rung_start + (index * rung_pitch)
        part.visual(
            Cylinder(radius=RUNG_RADIUS, length=rung_length),
            origin=Origin(
                xyz=(0.0, rung_offset_y, rung_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rung_material,
            name=f"{section_name}_rung_{index + 1}",
        )

    top_cap_height = 0.028
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((flange_width, depth, top_cap_height)),
            origin=Origin(
                xyz=(
                    sign * flange_center_x,
                    0.0,
                    length - (top_cap_height / 2.0),
                )
            ),
            material=rail_material,
            name=f"{section_name}_{side_name}_top_cap",
        )

    if add_feet:
        foot_size = (0.068, depth + 0.014, 0.028)
        foot_x = outer_half_width - (flange_width * 0.60)
        foot_z = foot_size[2] / 2.0
        part.visual(
            Box(foot_size),
            origin=Origin(xyz=(-foot_x, 0.0, foot_z)),
            material=hardware_material,
            name=f"{section_name}_left_foot",
        )
        part.visual(
            Box(foot_size),
            origin=Origin(xyz=(foot_x, 0.0, foot_z)),
            material=hardware_material,
            name=f"{section_name}_right_foot",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fiberglass_extension_ladder", assets=ASSETS)

    fiberglass = model.material("fiberglass_yellow", rgba=(0.94, 0.80, 0.22, 1.0))
    aluminum = model.material("aluminum_rungs", rgba=(0.77, 0.80, 0.83, 1.0))
    black_hardware = model.material("black_hardware", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    _add_section(
        base,
        rail_material=fiberglass,
        rung_material=aluminum,
        hardware_material=black_hardware,
        add_feet=True,
        **BASE_SPEC,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.54, 0.10, BASE_SPEC["length"])),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_SPEC["length"] / 2.0)),
    )

    middle = model.part("middle")
    _add_section(
        middle,
        rail_material=fiberglass,
        rung_material=aluminum,
        hardware_material=black_hardware,
        parent_inner_edge=BASE_SPEC["outer_half_width"] - BASE_SPEC["flange_width"],
        parent_depth=BASE_SPEC["depth"],
        **MIDDLE_SPEC,
    )
    middle.inertial = Inertial.from_geometry(
        Box((0.40, 0.08, MIDDLE_SPEC["length"])),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SPEC["length"] / 2.0)),
    )

    fly = model.part("fly")
    _add_section(
        fly,
        rail_material=fiberglass,
        rung_material=aluminum,
        hardware_material=black_hardware,
        parent_inner_edge=MIDDLE_SPEC["outer_half_width"] - MIDDLE_SPEC["flange_width"],
        parent_depth=MIDDLE_SPEC["depth"],
        **FLY_SPEC,
    )
    fly.inertial = Inertial.from_geometry(
        Box((0.28, 0.06, FLY_SPEC["length"])),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, FLY_SPEC["length"] / 2.0)),
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_MIDDLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.8,
            lower=0.0,
            upper=MIDDLE_EXTENSION,
        ),
    )
    model.articulation(
        "middle_to_fly",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_FLY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.8,
            lower=0.0,
            upper=FLY_EXTENSION,
        ),
    )

    return model


def _aabb_extent(aabb, axis: str) -> float:
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[1][axis_index] - aabb[0][axis_index]


def _limits_match(limits: MotionLimits | None, *, lower: float, upper: float) -> bool:
    if limits is None or limits.lower is None or limits.upper is None:
        return False
    return (
        math.isclose(limits.lower, lower, abs_tol=1e-9)
        and math.isclose(limits.upper, upper, abs_tol=1e-9)
        and limits.effort > 0.0
        and limits.velocity > 0.0
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    middle = object_model.get_part("middle")
    fly = object_model.get_part("fly")
    base_to_middle = object_model.get_articulation("base_to_middle")
    middle_to_fly = object_model.get_articulation("middle_to_fly")

    base_left_front_flange = base.get_visual("base_left_front_flange")
    base_left_rear_flange = base.get_visual("base_left_rear_flange")
    base_right_front_flange = base.get_visual("base_right_front_flange")
    base_right_rear_flange = base.get_visual("base_right_rear_flange")
    middle_left_front_flange = middle.get_visual("middle_left_front_flange")
    middle_left_rear_flange = middle.get_visual("middle_left_rear_flange")
    middle_right_front_flange = middle.get_visual("middle_right_front_flange")
    middle_right_rear_flange = middle.get_visual("middle_right_rear_flange")
    middle_left_front_guide = middle.get_visual("middle_left_front_guide")
    middle_left_rear_guide = middle.get_visual("middle_left_rear_guide")
    middle_right_front_guide = middle.get_visual("middle_right_front_guide")
    middle_right_rear_guide = middle.get_visual("middle_right_rear_guide")
    fly_left_front_guide = fly.get_visual("fly_left_front_guide")
    fly_left_rear_guide = fly.get_visual("fly_left_rear_guide")
    fly_right_front_guide = fly.get_visual("fly_right_front_guide")
    fly_right_rear_guide = fly.get_visual("fly_right_rear_guide")
    base_right_top_cap = base.get_visual("base_right_top_cap")
    middle_right_top_cap = middle.get_visual("middle_right_top_cap")
    fly_right_top_cap = fly.get_visual("fly_right_top_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=64,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    base_aabb = ctx.part_world_aabb(base)
    middle_aabb = ctx.part_world_aabb(middle)
    fly_aabb = ctx.part_world_aabb(fly)
    if base_aabb is not None and middle_aabb is not None and fly_aabb is not None:
        base_width = _aabb_extent(base_aabb, "x")
        middle_width = _aabb_extent(middle_aabb, "x")
        fly_width = _aabb_extent(fly_aabb, "x")
        base_length = _aabb_extent(base_aabb, "z")
        middle_length = _aabb_extent(middle_aabb, "z")
        fly_length = _aabb_extent(fly_aabb, "z")

        ctx.check(
            "section_widths_taper_inward",
            base_width > middle_width > fly_width,
            details=(
                f"expected base > middle > fly widths, got "
                f"{base_width:.3f}, {middle_width:.3f}, {fly_width:.3f}"
            ),
        )
        ctx.check(
            "section_lengths_taper_upward",
            base_length > middle_length > fly_length,
            details=(
                f"expected base > middle > fly lengths, got "
                f"{base_length:.3f}, {middle_length:.3f}, {fly_length:.3f}"
            ),
        )
        ctx.check(
            "base_section_realistic_size",
            0.50 <= base_width <= 0.55 and 2.60 <= base_length <= 2.80,
            details=(
                f"base section should read as a full-size extension ladder, "
                f"got width={base_width:.3f} length={base_length:.3f}"
            ),
        )

    ctx.check(
        "base_to_middle_prismatic_axis_and_limits",
        base_to_middle.articulation_type == ArticulationType.PRISMATIC
        and tuple(base_to_middle.axis) == (0.0, 0.0, 1.0)
        and _limits_match(
            base_to_middle.motion_limits,
            lower=0.0,
            upper=MIDDLE_EXTENSION,
        ),
        details=(
            "base_to_middle should be a vertical prismatic section slide "
            "with the authored extension travel"
        ),
    )
    ctx.check(
        "middle_to_fly_prismatic_axis_and_limits",
        middle_to_fly.articulation_type == ArticulationType.PRISMATIC
        and tuple(middle_to_fly.axis) == (0.0, 0.0, 1.0)
        and _limits_match(
            middle_to_fly.motion_limits,
            lower=0.0,
            upper=FLY_EXTENSION,
        ),
        details=(
            "middle_to_fly should be a vertical prismatic section slide "
            "with the authored extension travel"
        ),
    )

    ctx.expect_origin_distance(middle, base, axes="xy", max_dist=0.002)
    ctx.expect_origin_distance(fly, middle, axes="xy", max_dist=0.002)
    ctx.expect_origin_gap(middle, base, axis="z", min_gap=0.219, max_gap=0.221)
    ctx.expect_origin_gap(fly, middle, axis="z", min_gap=0.179, max_gap=0.181)
    ctx.expect_within(middle, base, axes="xy")
    ctx.expect_within(fly, middle, axes="xy")
    ctx.expect_contact(middle, base, elem_a=middle_left_front_guide, elem_b=base_left_front_flange)
    ctx.expect_contact(middle, base, elem_a=middle_left_rear_guide, elem_b=base_left_rear_flange)
    ctx.expect_contact(middle, base, elem_a=middle_right_front_guide, elem_b=base_right_front_flange)
    ctx.expect_contact(middle, base, elem_a=middle_right_rear_guide, elem_b=base_right_rear_flange)
    ctx.expect_contact(fly, middle, elem_a=fly_left_front_guide, elem_b=middle_left_front_flange)
    ctx.expect_contact(fly, middle, elem_a=fly_left_rear_guide, elem_b=middle_left_rear_flange)
    ctx.expect_contact(fly, middle, elem_a=fly_right_front_guide, elem_b=middle_right_front_flange)
    ctx.expect_contact(fly, middle, elem_a=fly_right_rear_guide, elem_b=middle_right_rear_flange)

    with ctx.pose({base_to_middle: 0.0, middle_to_fly: 0.0}):
        ctx.expect_gap(
            base,
            middle,
            axis="z",
            positive_elem=base_right_top_cap,
            negative_elem=middle_right_top_cap,
            min_gap=0.07,
            max_gap=0.10,
            name="middle_retracted_below_base_top",
        )
        ctx.expect_gap(
            middle,
            fly,
            axis="z",
            positive_elem=middle_right_top_cap,
            negative_elem=fly_right_top_cap,
            min_gap=0.05,
            max_gap=0.08,
            name="fly_retracted_below_middle_top",
        )

    with ctx.pose({base_to_middle: MIDDLE_EXTENSION * 0.85}):
        ctx.expect_contact(middle, base, elem_a=middle_left_front_guide, elem_b=base_left_front_flange)
        ctx.expect_contact(middle, base, elem_a=middle_left_rear_guide, elem_b=base_left_rear_flange)
        ctx.expect_contact(middle, base, elem_a=middle_right_front_guide, elem_b=base_right_front_flange)
        ctx.expect_contact(middle, base, elem_a=middle_right_rear_guide, elem_b=base_right_rear_flange)
        ctx.expect_gap(
            middle,
            base,
            axis="z",
            positive_elem=middle_right_top_cap,
            negative_elem=base_right_top_cap,
            min_gap=0.80,
            name="middle_projects_above_base_when_extended",
        )

    with ctx.pose({base_to_middle: MIDDLE_EXTENSION, middle_to_fly: FLY_EXTENSION}):
        ctx.expect_contact(middle, base, elem_a=middle_left_front_guide, elem_b=base_left_front_flange)
        ctx.expect_contact(middle, base, elem_a=middle_left_rear_guide, elem_b=base_left_rear_flange)
        ctx.expect_contact(middle, base, elem_a=middle_right_front_guide, elem_b=base_right_front_flange)
        ctx.expect_contact(middle, base, elem_a=middle_right_rear_guide, elem_b=base_right_rear_flange)
        ctx.expect_contact(fly, middle, elem_a=fly_left_front_guide, elem_b=middle_left_front_flange)
        ctx.expect_contact(fly, middle, elem_a=fly_left_rear_guide, elem_b=middle_left_rear_flange)
        ctx.expect_contact(fly, middle, elem_a=fly_right_front_guide, elem_b=middle_right_front_flange)
        ctx.expect_contact(fly, middle, elem_a=fly_right_rear_guide, elem_b=middle_right_rear_flange)
        ctx.expect_gap(
            fly,
            middle,
            axis="z",
            positive_elem=fly_right_top_cap,
            negative_elem=middle_right_top_cap,
            min_gap=0.90,
            name="fly_projects_above_middle_when_extended",
        )
        ctx.expect_gap(
            fly,
            base,
            axis="z",
            positive_elem=fly_right_top_cap,
            negative_elem=base_right_top_cap,
            min_gap=1.95,
            name="fly_projects_well_above_base_when_extended",
        )

    for articulation_name, articulation in (
        ("base_to_middle", base_to_middle),
        ("middle_to_fly", middle_to_fly),
    ):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{articulation_name}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{articulation_name}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{articulation_name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{articulation_name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
