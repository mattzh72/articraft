from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
from math import pi

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
)

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

MODEL_NAME = "two_section_extension_ladder"

STEEL_RGBA = (0.60, 0.63, 0.67, 1.0)
RUNG_RGBA = (0.74, 0.76, 0.79, 1.0)
RUBBER_RGBA = (0.14, 0.15, 0.17, 1.0)

CONNECT_EPS = 0.0002

BASE_SECTION_LENGTH = 3.55
FLY_SECTION_LENGTH = 3.10
FLY_REST_Z = 1.05
FLY_TRAVEL = 1.35

BASE_RAIL_CENTER_X = 0.215
BASE_RAIL_OUTER = (0.055, 0.030, BASE_SECTION_LENGTH)
BASE_RAIL_WALL = 0.0035

FLY_RAIL_OUTER = (0.028, 0.016, FLY_SECTION_LENGTH)
FLY_RAIL_WALL = 0.0030

CHANNEL_WEB_THICKNESS = 0.005
CHANNEL_FLANGE_REACH = 0.034
CHANNEL_FLANGE_THICKNESS = 0.004
CHANNEL_DEPTH = 0.026
CHANNEL_LENGTH = 3.10
CHANNEL_BOTTOM_Z = 0.27

FOOT_BARREL_RADIUS = 0.011
FOOT_BARREL_LENGTH = 0.040
FOOT_BARREL_Z = 0.060
FOOT_SWIVEL_LIMIT = 0.55

BASE_RAIL_BOTTOM_Z = FOOT_BARREL_Z + FOOT_BARREL_RADIUS
BASE_RAIL_CENTER_Z = BASE_RAIL_BOTTOM_Z + BASE_SECTION_LENGTH / 2.0
FLY_RAIL_CENTER_Z = FLY_SECTION_LENGTH / 2.0
CHANNEL_CENTER_Z = CHANNEL_BOTTOM_Z + CHANNEL_LENGTH / 2.0

FLY_LEFT_OUTER_WALL_X = (
    -BASE_RAIL_CENTER_X
    + BASE_RAIL_OUTER[0] / 2.0
    + CHANNEL_WEB_THICKNESS
    + 0.002
    + FLY_RAIL_OUTER[0] / 2.0
)
FLY_RIGHT_OUTER_WALL_X = -FLY_LEFT_OUTER_WALL_X

RUNG_PITCH = 0.295
BASE_RUNG_OUTER = (
    2.0 * BASE_RAIL_CENTER_X - BASE_RAIL_OUTER[0],
    0.026,
    0.024,
)
FLY_RUNG_OUTER = (
    2.0 * abs(FLY_LEFT_OUTER_WALL_X) - FLY_RAIL_OUTER[0],
    0.020,
    0.022,
)
RUNG_WALL = 0.0025
BASE_RUNG_ZS = [BASE_RAIL_BOTTOM_Z + 0.30 + i * RUNG_PITCH for i in range(11)]
FLY_RUNG_LOCAL_ZS = [0.24 + i * RUNG_PITCH for i in range(9)]

FOOT_STEM_SIZE = (0.026, 0.018, 0.032)
FOOT_STEM_CENTER_Z = -0.020
FOOT_SHOE_SIZE = (0.090, 0.034, 0.006)
FOOT_SHOE_CENTER_Z = -0.038
FOOT_PAD_SIZE = (0.095, 0.040, 0.012)
FOOT_PAD_CENTER_Z = -0.046


def _add_rect_tube(
    part,
    *,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    wall: float,
    material,
    name_map: dict[str, str] | None = None,
    cap_z: bool = False,
) -> None:
    cx, cy, cz = center
    sx, sy, sz = size
    names = name_map or {}
    side_depth = max(sy - 2.0 * wall + 2.0 * CONNECT_EPS, wall)

    part.visual(
        Box((wall, side_depth, sz)),
        origin=Origin(xyz=(cx - sx / 2.0 + wall / 2.0, cy, cz)),
        material=material,
        name=names.get("neg_x"),
    )
    part.visual(
        Box((wall, side_depth, sz)),
        origin=Origin(xyz=(cx + sx / 2.0 - wall / 2.0, cy, cz)),
        material=material,
        name=names.get("pos_x"),
    )
    part.visual(
        Box((sx, wall, sz)),
        origin=Origin(xyz=(cx, cy + sy / 2.0 - wall / 2.0, cz)),
        material=material,
        name=names.get("pos_y"),
    )
    part.visual(
        Box((sx, wall, sz)),
        origin=Origin(xyz=(cx, cy - sy / 2.0 + wall / 2.0, cz)),
        material=material,
        name=names.get("neg_y"),
    )
    if cap_z:
        part.visual(
            Box((sx, sy, wall)),
            origin=Origin(xyz=(cx, cy, cz + sz / 2.0 - wall / 2.0)),
            material=material,
            name=names.get("pos_z"),
        )
        part.visual(
            Box((sx, sy, wall)),
            origin=Origin(xyz=(cx, cy, cz - sz / 2.0 + wall / 2.0)),
            material=material,
            name=names.get("neg_z"),
        )


def _stile_name_map(prefix: str, *, left_side: bool) -> dict[str, str]:
    if left_side:
        return {
            "neg_x": f"{prefix}_outer_wall",
            "pos_x": f"{prefix}_inner_wall",
            "pos_y": f"{prefix}_front_wall",
            "neg_y": f"{prefix}_rear_wall",
            "pos_z": f"{prefix}_top_cap",
            "neg_z": f"{prefix}_bottom_cap",
        }
    return {
        "neg_x": f"{prefix}_inner_wall",
        "pos_x": f"{prefix}_outer_wall",
        "pos_y": f"{prefix}_front_wall",
        "neg_y": f"{prefix}_rear_wall",
        "pos_z": f"{prefix}_top_cap",
        "neg_z": f"{prefix}_bottom_cap",
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name=MODEL_NAME)

    steel = model.material("steel", rgba=STEEL_RGBA)
    rung_steel = model.material("rung_steel", rgba=RUNG_RGBA)
    rubber = model.material("rubber", rgba=RUBBER_RGBA)

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((2.0 * BASE_RAIL_CENTER_X + BASE_RAIL_OUTER[0], BASE_RAIL_OUTER[1], BASE_SECTION_LENGTH)),
        mass=14.5,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_CENTER_Z)),
    )

    fly = model.part("fly")
    fly.inertial = Inertial.from_geometry(
        Box((2.0 * abs(FLY_LEFT_OUTER_WALL_X) + FLY_RAIL_OUTER[0], FLY_RAIL_OUTER[1], FLY_SECTION_LENGTH)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, FLY_RAIL_CENTER_Z)),
    )

    left_foot = model.part("left_foot")
    left_foot.inertial = Inertial.from_geometry(
        Box((FOOT_PAD_SIZE[0], FOOT_PAD_SIZE[1], 0.071)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )
    right_foot = model.part("right_foot")
    right_foot.inertial = Inertial.from_geometry(
        Box((FOOT_PAD_SIZE[0], FOOT_PAD_SIZE[1], 0.071)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )

    _add_rect_tube(
        base,
        center=(-BASE_RAIL_CENTER_X, 0.0, BASE_RAIL_CENTER_Z),
        size=BASE_RAIL_OUTER,
        wall=BASE_RAIL_WALL,
        material=steel,
        name_map=_stile_name_map("left_base", left_side=True),
        cap_z=True,
    )
    _add_rect_tube(
        base,
        center=(BASE_RAIL_CENTER_X, 0.0, BASE_RAIL_CENTER_Z),
        size=BASE_RAIL_OUTER,
        wall=BASE_RAIL_WALL,
        material=steel,
        name_map=_stile_name_map("right_base", left_side=False),
        cap_z=True,
    )

    left_channel_web_center_x = (
        -BASE_RAIL_CENTER_X + BASE_RAIL_OUTER[0] / 2.0 + CHANNEL_WEB_THICKNESS / 2.0
    )
    right_channel_web_center_x = (
        BASE_RAIL_CENTER_X - BASE_RAIL_OUTER[0] / 2.0 - CHANNEL_WEB_THICKNESS / 2.0
    )
    left_channel_flange_center_x = (
        left_channel_web_center_x + CHANNEL_WEB_THICKNESS / 2.0 + CHANNEL_FLANGE_REACH / 2.0
    )
    right_channel_flange_center_x = (
        right_channel_web_center_x - CHANNEL_WEB_THICKNESS / 2.0 - CHANNEL_FLANGE_REACH / 2.0
    )
    flange_center_y = CHANNEL_DEPTH / 2.0 - CHANNEL_FLANGE_THICKNESS / 2.0

    for side, web_x, flange_x in (
        ("left", left_channel_web_center_x, left_channel_flange_center_x),
        ("right", right_channel_web_center_x, right_channel_flange_center_x),
    ):
        base.visual(
            Box((CHANNEL_WEB_THICKNESS, CHANNEL_DEPTH, CHANNEL_LENGTH)),
            origin=Origin(xyz=(web_x, 0.0, CHANNEL_CENTER_Z)),
            material=steel,
            name=f"{side}_channel_web",
        )
        base.visual(
            Box((CHANNEL_FLANGE_REACH, CHANNEL_FLANGE_THICKNESS, CHANNEL_LENGTH)),
            origin=Origin(xyz=(flange_x, flange_center_y, CHANNEL_CENTER_Z)),
            material=steel,
            name=f"{side}_channel_front_flange",
        )
        base.visual(
            Box((CHANNEL_FLANGE_REACH, CHANNEL_FLANGE_THICKNESS, CHANNEL_LENGTH)),
            origin=Origin(xyz=(flange_x, -flange_center_y, CHANNEL_CENTER_Z)),
            material=steel,
            name=f"{side}_channel_rear_flange",
        )

    for idx, rung_z in enumerate(BASE_RUNG_ZS):
        _add_rect_tube(
            base,
            center=(0.0, 0.0, rung_z),
            size=BASE_RUNG_OUTER,
            wall=RUNG_WALL,
            material=rung_steel,
            name_map={"pos_y": f"base_rung_{idx}_front_wall"},
        )

    _add_rect_tube(
        fly,
        center=(FLY_LEFT_OUTER_WALL_X, 0.0, FLY_RAIL_CENTER_Z),
        size=FLY_RAIL_OUTER,
        wall=FLY_RAIL_WALL,
        material=steel,
        name_map=_stile_name_map("left_fly", left_side=True),
        cap_z=True,
    )
    _add_rect_tube(
        fly,
        center=(FLY_RIGHT_OUTER_WALL_X, 0.0, FLY_RAIL_CENTER_Z),
        size=FLY_RAIL_OUTER,
        wall=FLY_RAIL_WALL,
        material=steel,
        name_map=_stile_name_map("right_fly", left_side=False),
        cap_z=True,
    )

    for idx, rung_z in enumerate(FLY_RUNG_LOCAL_ZS):
        _add_rect_tube(
            fly,
            center=(0.0, 0.0, rung_z),
            size=FLY_RUNG_OUTER,
            wall=RUNG_WALL,
            material=rung_steel,
            name_map={"pos_y": f"fly_rung_{idx}_front_wall"},
        )

    for foot_part, side in ((left_foot, "left"), (right_foot, "right")):
        foot_part.visual(
            Cylinder(radius=FOOT_BARREL_RADIUS, length=FOOT_BARREL_LENGTH),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"{side}_foot_barrel",
        )
        foot_part.visual(
            Box(FOOT_STEM_SIZE),
            origin=Origin(xyz=(0.0, 0.0, FOOT_STEM_CENTER_Z)),
            material=steel,
            name=f"{side}_foot_stem",
        )
        foot_part.visual(
            Box(FOOT_SHOE_SIZE),
            origin=Origin(xyz=(0.0, 0.0, FOOT_SHOE_CENTER_Z)),
            material=steel,
            name=f"{side}_foot_shoe",
        )
        foot_part.visual(
            Box(FOOT_PAD_SIZE),
            origin=Origin(xyz=(0.0, 0.0, FOOT_PAD_CENTER_Z)),
            material=rubber,
            name=f"{side}_foot_pad",
        )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, FLY_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.45,
            lower=0.0,
            upper=FLY_TRAVEL,
        ),
    )
    model.articulation(
        "left_foot_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_foot,
        origin=Origin(xyz=(-BASE_RAIL_CENTER_X, 0.0, FOOT_BARREL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-FOOT_SWIVEL_LIMIT,
            upper=FOOT_SWIVEL_LIMIT,
        ),
    )
    model.articulation(
        "right_foot_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_foot,
        origin=Origin(xyz=(BASE_RAIL_CENTER_X, 0.0, FOOT_BARREL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-FOOT_SWIVEL_LIMIT,
            upper=FOOT_SWIVEL_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")

    fly_slide = object_model.get_articulation("fly_slide")
    left_foot_swivel = object_model.get_articulation("left_foot_swivel")
    right_foot_swivel = object_model.get_articulation("right_foot_swivel")

    left_channel_web = base.get_visual("left_channel_web")
    left_channel_front_flange = base.get_visual("left_channel_front_flange")
    left_channel_rear_flange = base.get_visual("left_channel_rear_flange")
    right_channel_web = base.get_visual("right_channel_web")
    right_channel_front_flange = base.get_visual("right_channel_front_flange")
    right_channel_rear_flange = base.get_visual("right_channel_rear_flange")
    left_base_bottom_cap = base.get_visual("left_base_bottom_cap")
    right_base_bottom_cap = base.get_visual("right_base_bottom_cap")

    left_fly_outer_wall = fly.get_visual("left_fly_outer_wall")
    left_fly_front_wall = fly.get_visual("left_fly_front_wall")
    left_fly_rear_wall = fly.get_visual("left_fly_rear_wall")
    right_fly_outer_wall = fly.get_visual("right_fly_outer_wall")
    right_fly_front_wall = fly.get_visual("right_fly_front_wall")
    right_fly_rear_wall = fly.get_visual("right_fly_rear_wall")

    left_foot_barrel = left_foot.get_visual("left_foot_barrel")
    right_foot_barrel = right_foot.get_visual("right_foot_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is None:
        ctx.fail("base_aabb_exists", "Base section did not produce a world-space AABB.")
    else:
        base_width = base_aabb[1][0] - base_aabb[0][0]
        base_height = base_aabb[1][2] - base_aabb[0][2]
        ctx.check(
            "base_section_width_realistic",
            0.45 <= base_width <= 0.50,
            f"Expected realistic extension-ladder width, got {base_width:.3f} m.",
        )
        ctx.check(
            "base_section_height_realistic",
            3.45 <= base_height <= 3.70,
            f"Expected realistic base-section height, got {base_height:.3f} m.",
        )

    fly_aabb = ctx.part_world_aabb(fly)
    if fly_aabb is None:
        ctx.fail("fly_aabb_exists", "Fly section did not produce a world-space AABB.")
    else:
        fly_height = fly_aabb[1][2] - fly_aabb[0][2]
        ctx.check(
            "fly_section_height_realistic",
            3.00 <= fly_height <= 3.20,
            f"Expected realistic fly-section height, got {fly_height:.3f} m.",
        )

    ctx.check(
        "fly_slide_axis_vertical",
        tuple(fly_slide.axis) == (0.0, 0.0, 1.0),
        f"Fly slide axis should be +Z, got {fly_slide.axis}.",
    )
    ctx.check(
        "left_foot_swivel_axis_transverse",
        tuple(left_foot_swivel.axis) == (1.0, 0.0, 0.0),
        f"Left foot axis should be +X, got {left_foot_swivel.axis}.",
    )
    ctx.check(
        "right_foot_swivel_axis_transverse",
        tuple(right_foot_swivel.axis) == (1.0, 0.0, 0.0),
        f"Right foot axis should be +X, got {right_foot_swivel.axis}.",
    )

    ctx.expect_gap(
        fly,
        base,
        axis="x",
        min_gap=0.001,
        max_gap=0.003,
        positive_elem=left_fly_outer_wall,
        negative_elem=left_channel_web,
        name="left fly stile keeps a narrow running gap to the left channel web",
    )
    ctx.expect_gap(
        base,
        fly,
        axis="x",
        min_gap=0.001,
        max_gap=0.003,
        positive_elem=right_channel_web,
        negative_elem=right_fly_outer_wall,
        name="right fly stile keeps a narrow running gap to the right channel web",
    )
    ctx.expect_gap(
        base,
        fly,
        axis="y",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=left_channel_front_flange,
        negative_elem=left_fly_front_wall,
        name="left front guide flange clears the fly stile with a slim sliding gap",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=left_fly_rear_wall,
        negative_elem=left_channel_rear_flange,
        name="left rear guide flange clears the fly stile with a slim sliding gap",
    )
    ctx.expect_gap(
        base,
        fly,
        axis="y",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=right_channel_front_flange,
        negative_elem=right_fly_front_wall,
        name="right front guide flange clears the fly stile with a slim sliding gap",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        min_gap=0.0005,
        max_gap=0.002,
        positive_elem=right_fly_rear_wall,
        negative_elem=right_channel_rear_flange,
        name="right rear guide flange clears the fly stile with a slim sliding gap",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=2.0,
        elem_a=left_fly_outer_wall,
        elem_b=left_channel_web,
        name="fly section has substantial guide engagement at rest",
    )
    ctx.expect_contact(
        left_foot,
        base,
        elem_a=left_foot_barrel,
        elem_b=left_base_bottom_cap,
        name="left swivel foot barrel is mounted under the left rail",
    )
    ctx.expect_contact(
        right_foot,
        base,
        elem_a=right_foot_barrel,
        elem_b=right_base_bottom_cap,
        name="right swivel foot barrel is mounted under the right rail",
    )

    rest_fly_pos = ctx.part_world_position(fly)
    rest_left_foot_aabb = ctx.part_world_aabb(left_foot)
    rest_right_foot_aabb = ctx.part_world_aabb(right_foot)

    limits = fly_slide.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({fly_slide: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fly_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="fly_slide_lower_no_floating")
        with ctx.pose({fly_slide: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fly_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="fly_slide_upper_no_floating")

    for swivel_joint in (left_foot_swivel, right_foot_swivel):
        swivel_limits = swivel_joint.motion_limits
        if swivel_limits is not None and swivel_limits.lower is not None and swivel_limits.upper is not None:
            with ctx.pose({swivel_joint: swivel_limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{swivel_joint.name}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{swivel_joint.name}_lower_no_floating")
            with ctx.pose({swivel_joint: swivel_limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{swivel_joint.name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{swivel_joint.name}_upper_no_floating")

    with ctx.pose({fly_slide: FLY_TRAVEL}):
        ctx.expect_gap(
            fly,
            base,
            axis="x",
            min_gap=0.001,
            max_gap=0.003,
            positive_elem=left_fly_outer_wall,
            negative_elem=left_channel_web,
            name="left fly stile keeps its web clearance through full extension",
        )
        ctx.expect_gap(
            base,
            fly,
            axis="x",
            min_gap=0.001,
            max_gap=0.003,
            positive_elem=right_channel_web,
            negative_elem=right_fly_outer_wall,
            name="right fly stile keeps its web clearance through full extension",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=0.9,
            elem_a=left_fly_outer_wall,
            elem_b=left_channel_web,
            name="fly section still remains captured in the guide channels when extended",
        )
        extended_fly_pos = ctx.part_world_position(fly)
        extended_base_aabb = ctx.part_world_aabb(base)
        extended_fly_aabb = ctx.part_world_aabb(fly)
        if rest_fly_pos is not None and extended_fly_pos is not None:
            travel = extended_fly_pos[2] - rest_fly_pos[2]
            ctx.check(
                "fly_section_prismatic_travel_matches_spec",
                abs(travel - FLY_TRAVEL) <= 0.002,
                f"Expected {FLY_TRAVEL:.3f} m fly travel, got {travel:.3f} m.",
            )
        if extended_base_aabb is not None and extended_fly_aabb is not None:
            top_exposure = extended_fly_aabb[1][2] - extended_base_aabb[1][2]
            ctx.check(
                "fly_section_extends_well_above_base",
                top_exposure >= 1.75,
                f"Expected at least 1.75 m of exposed fly section, got {top_exposure:.3f} m.",
            )

    with ctx.pose({left_foot_swivel: 0.42, right_foot_swivel: -0.42}):
        ctx.expect_contact(
            left_foot,
            base,
            elem_a=left_foot_barrel,
            elem_b=left_base_bottom_cap,
            name="left foot remains attached while swiveled",
        )
        ctx.expect_contact(
            right_foot,
            base,
            elem_a=right_foot_barrel,
            elem_b=right_base_bottom_cap,
            name="right foot remains attached while swiveled",
        )
        swiveled_left_aabb = ctx.part_world_aabb(left_foot)
        swiveled_right_aabb = ctx.part_world_aabb(right_foot)
        if rest_left_foot_aabb is not None and swiveled_left_aabb is not None:
            left_motion = max(
                abs(swiveled_left_aabb[0][1] - rest_left_foot_aabb[0][1]),
                abs(swiveled_left_aabb[1][2] - rest_left_foot_aabb[1][2]),
            )
            ctx.check(
                "left_foot_changes_attitude_when_swiveled",
                left_motion >= 0.004,
                f"Expected visible left-foot motion, got {left_motion:.4f} m AABB change.",
            )
        if rest_right_foot_aabb is not None and swiveled_right_aabb is not None:
            right_motion = max(
                abs(swiveled_right_aabb[1][1] - rest_right_foot_aabb[1][1]),
                abs(swiveled_right_aabb[1][2] - rest_right_foot_aabb[1][2]),
            )
            ctx.check(
                "right_foot_changes_attitude_when_swiveled",
                right_motion >= 0.004,
                f"Expected visible right-foot motion, got {right_motion:.4f} m AABB change.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
