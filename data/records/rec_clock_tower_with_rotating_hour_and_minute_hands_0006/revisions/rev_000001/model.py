from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

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

_ORIGINAL_GETCWD = os.getcwd
_ORIGINAL_PATH_ABSOLUTE = pathlib.Path.absolute
_ORIGINAL_PATH_RESOLVE = pathlib.Path.resolve


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd


def _safe_absolute(self: pathlib.Path) -> pathlib.Path:
    if self.is_absolute():
        return self
    return type(self)("/").joinpath(self)


def _safe_resolve(self: pathlib.Path, *args, **kwargs) -> pathlib.Path:
    try:
        return _ORIGINAL_PATH_RESOLVE(self, *args, **kwargs)
    except FileNotFoundError:
        return _safe_absolute(self)


pathlib.Path.absolute = _safe_absolute
pathlib.Path.resolve = _safe_resolve
pathlib.Path.cwd = classmethod(lambda cls: cls("/"))
if "__file__" in globals() and not os.path.isabs(__file__):
    __file__ = "/" + os.path.basename(__file__)
try:
    _ORIGINAL_GETCWD()
except FileNotFoundError:
    os.chdir("/")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_village_clock_tower")

    timber = model.material("timber", rgba=(0.63, 0.47, 0.29, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.40, 0.28, 0.18, 1.0))
    stone = model.material("stone", rgba=(0.44, 0.42, 0.39, 1.0))
    roof = model.material("roof", rgba=(0.34, 0.25, 0.20, 1.0))
    clock_paint = model.material("clock_paint", rgba=(0.93, 0.90, 0.80, 1.0))
    hand_metal = model.material("hand_metal", rgba=(0.15, 0.14, 0.12, 1.0))
    shadow = model.material("shadow", rgba=(0.18, 0.12, 0.08, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((2.60, 2.60, 0.90)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=stone,
        name="plinth",
    )
    tower.visual(
        Box((1.96, 1.96, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=dark_timber,
        name="plinth_cap",
    )
    tower.visual(
        Box((1.48, 1.48, 3.90)),
        origin=Origin(xyz=(0.0, 0.0, 3.01)),
        material=timber,
        name="lower_shaft",
    )
    tower.visual(
        Box((1.82, 1.82, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 2.06)),
        material=dark_timber,
        name="mid_trim_lower",
    )
    tower.visual(
        Box((1.82, 1.82, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 3.72)),
        material=dark_timber,
        name="mid_trim_upper",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.18, 0.18, 4.06)),
                origin=Origin(xyz=(x_sign * 0.74, y_sign * 0.74, 2.93)),
                material=dark_timber,
                name=f"lower_corner_post_{int(x_sign)}_{int(y_sign)}",
            )
    tower.visual(
        Box((2.00, 2.00, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 5.04)),
        material=dark_timber,
        name="belfry_floor_ring",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.16, 0.16, 1.10)),
                origin=Origin(xyz=(x_sign * 0.84, y_sign * 0.84, 5.67)),
                material=dark_timber,
                name=f"belfry_post_{int(x_sign)}_{int(y_sign)}",
            )
    tower.visual(
        Box((1.52, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, -0.88, 5.36)),
        material=dark_timber,
        name="rear_lower_rail",
    )
    tower.visual(
        Box((1.52, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, -0.88, 5.98)),
        material=dark_timber,
        name="rear_upper_rail",
    )
    tower.visual(
        Box((0.08, 1.52, 0.14)),
        origin=Origin(xyz=(-0.88, 0.0, 5.36)),
        material=dark_timber,
        name="left_lower_rail",
    )
    tower.visual(
        Box((0.08, 1.52, 0.14)),
        origin=Origin(xyz=(-0.88, 0.0, 5.98)),
        material=dark_timber,
        name="left_upper_rail",
    )
    tower.visual(
        Box((0.08, 1.52, 0.14)),
        origin=Origin(xyz=(0.88, 0.0, 5.36)),
        material=dark_timber,
        name="right_lower_rail",
    )
    tower.visual(
        Box((0.08, 1.52, 0.14)),
        origin=Origin(xyz=(0.88, 0.0, 5.98)),
        material=dark_timber,
        name="right_upper_rail",
    )
    for index, z in enumerate((5.54, 5.72, 5.90)):
        tower.visual(
            Box((1.52, 0.06, 0.05)),
            origin=Origin(xyz=(0.0, -0.88, z)),
            material=shadow,
            name=f"rear_louver_{index}",
        )
        tower.visual(
            Box((0.06, 1.52, 0.05)),
            origin=Origin(xyz=(-0.88, 0.0, z)),
            material=shadow,
            name=f"left_louver_{index}",
        )
        tower.visual(
            Box((0.06, 1.52, 0.05)),
            origin=Origin(xyz=(0.88, 0.0, z)),
            material=shadow,
            name=f"right_louver_{index}",
        )
    tower.visual(
        Box((1.52, 0.08, 0.88)),
        origin=Origin(xyz=(0.0, 0.88, 5.67)),
        material=timber,
        name="front_clock_board",
    )
    tower.visual(
        Box((1.68, 0.20, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 6.06)),
        material=dark_timber,
        name="bell_beam",
    )
    tower.visual(
        Cylinder(radius=0.34, length=0.06),
        origin=Origin(xyz=(0.0, 0.91, 5.67), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_paint,
        name="clock_face",
    )
    tower.visual(
        Box((0.03, 0.012, 0.10)),
        origin=Origin(xyz=(0.0, 0.935, 5.91)),
        material=dark_timber,
        name="tick_top",
    )
    tower.visual(
        Box((0.10, 0.012, 0.03)),
        origin=Origin(xyz=(0.24, 0.935, 5.67)),
        material=dark_timber,
        name="tick_right",
    )
    tower.visual(
        Box((0.03, 0.012, 0.10)),
        origin=Origin(xyz=(0.0, 0.935, 5.43)),
        material=dark_timber,
        name="tick_bottom",
    )
    tower.visual(
        Box((0.10, 0.012, 0.03)),
        origin=Origin(xyz=(-0.24, 0.935, 5.67)),
        material=dark_timber,
        name="tick_left",
    )
    tower.visual(
        Box((2.20, 2.20, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 6.31)),
        material=dark_timber,
        name="steeple_eave",
    )
    tower.visual(
        Box((1.18, 1.18, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 6.54)),
        material=dark_timber,
        name="steeple_base",
    )
    tower.visual(
        Box((1.34, 0.10, 1.58)),
        origin=Origin(xyz=(0.0, 0.30, 7.39), rpy=(0.39, 0.0, 0.0)),
        material=roof,
        name="steeple_front_face",
    )
    tower.visual(
        Box((1.34, 0.10, 1.58)),
        origin=Origin(xyz=(0.0, -0.30, 7.39), rpy=(-0.39, 0.0, 0.0)),
        material=roof,
        name="steeple_back_face",
    )
    tower.visual(
        Box((0.10, 1.34, 1.58)),
        origin=Origin(xyz=(0.30, 0.0, 7.39), rpy=(0.0, -0.39, 0.0)),
        material=roof,
        name="steeple_right_face",
    )
    tower.visual(
        Box((0.10, 1.34, 1.58)),
        origin=Origin(xyz=(-0.30, 0.0, 7.39), rpy=(0.0, 0.39, 0.0)),
        material=roof,
        name="steeple_left_face",
    )
    tower.visual(
        Box((0.14, 0.14, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 8.09)),
        material=dark_timber,
        name="steeple_apex",
    )
    tower.visual(
        Cylinder(radius=0.05, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 8.39)),
        material=dark_timber,
        name="spire_finial",
    )
    tower.inertial = Inertial.from_geometry(
        Box((2.60, 2.60, 8.60)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 4.30)),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.055, length=0.04),
        origin=Origin(xyz=(0.0, 0.02, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hand_metal,
        name="hour_hub",
    )
    hour_hand.visual(
        Box((0.035, 0.008, 0.17)),
        origin=Origin(xyz=(0.0, 0.024, 0.085)),
        material=hand_metal,
        name="hour_blade",
    )
    hour_hand.visual(
        Box((0.070, 0.008, 0.08)),
        origin=Origin(xyz=(0.0, 0.024, 0.21)),
        material=hand_metal,
        name="hour_tip",
    )
    hour_hand.visual(
        Box((0.022, 0.008, 0.07)),
        origin=Origin(xyz=(0.0, 0.024, -0.035)),
        material=hand_metal,
        name="hour_tail",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.08, 0.04, 0.31)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.038, length=0.02),
        origin=Origin(xyz=(0.0, 0.01, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hand_metal,
        name="minute_hub",
    )
    minute_hand.visual(
        Box((0.034, 0.008, 0.24)),
        origin=Origin(xyz=(0.0, 0.012, 0.12)),
        material=hand_metal,
        name="minute_blade",
    )
    minute_hand.visual(
        Box((0.062, 0.008, 0.08)),
        origin=Origin(xyz=(0.0, 0.012, 0.28)),
        material=hand_metal,
        name="minute_tip",
    )
    minute_hand.visual(
        Box((0.018, 0.008, 0.08)),
        origin=Origin(xyz=(0.0, 0.012, -0.04)),
        material=hand_metal,
        name="minute_tail",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.06, 0.03, 0.40)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, 0.94, 5.67)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=0.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, 0.98, 5.67)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=1.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    clock_face = tower.get_visual("clock_face")
    hour_hub = hour_hand.get_visual("hour_hub")
    hour_blade = hour_hand.get_visual("hour_blade")
    minute_hub = minute_hand.get_visual("minute_hub")
    minute_blade = minute_hand.get_visual("minute_blade")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=32,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    tower_aabb = ctx.part_world_aabb(tower)
    clock_aabb = ctx.part_element_world_aabb(tower, elem=clock_face)
    hour_rest_aabb = ctx.part_element_world_aabb(hour_hand, elem=hour_blade)
    assert tower_aabb is not None
    assert clock_aabb is not None
    assert hour_rest_aabb is not None

    tower_width = tower_aabb[1][0] - tower_aabb[0][0]
    tower_depth = tower_aabb[1][1] - tower_aabb[0][1]
    tower_height = tower_aabb[1][2] - tower_aabb[0][2]
    clock_diameter = clock_aabb[1][0] - clock_aabb[0][0]
    clock_center_z = (clock_aabb[0][2] + clock_aabb[1][2]) * 0.5
    ctx.check(
        "tower proportions read as a small village clock tower",
        2.5 <= tower_width <= 2.7 and 2.5 <= tower_depth <= 2.7 and 8.4 <= tower_height <= 8.7,
        (
            f"expected about 2.6 x 2.6 x 8.6 m, "
            f"got {tower_width:.3f} x {tower_depth:.3f} x {tower_height:.3f} m"
        ),
    )
    ctx.check(
        "clock face stays small and high on the belfry tier",
        0.65 <= clock_diameter <= 0.70 and 5.5 <= clock_center_z <= 5.8,
        f"clock diameter {clock_diameter:.3f} m at z={clock_center_z:.3f} m",
    )

    ctx.expect_contact(
        hour_hand,
        tower,
        elem_a=hour_hub,
        elem_b=clock_face,
        name="hour hand hub seats directly on the clock face",
    )
    ctx.expect_contact(
        minute_hand,
        hour_hand,
        elem_a=minute_hub,
        elem_b=hour_hub,
        name="minute hand hub stacks on the hour hand hub",
    )
    ctx.expect_within(
        hour_hand,
        tower,
        axes="xz",
        inner_elem=hour_blade,
        outer_elem=clock_face,
        margin=0.0,
        name="hour hand stays within the small front clock face",
    )
    ctx.expect_within(
        minute_hand,
        tower,
        axes="xz",
        inner_elem=minute_blade,
        outer_elem=clock_face,
        margin=0.0,
        name="minute hand stays within the small front clock face",
    )
    ctx.expect_overlap(
        hour_hand,
        tower,
        axes="xz",
        elem_a=hour_blade,
        elem_b=clock_face,
        min_overlap=0.03,
        name="hour hand visibly crosses the clock face",
    )
    ctx.expect_overlap(
        minute_hand,
        tower,
        axes="xz",
        elem_a=minute_blade,
        elem_b=clock_face,
        min_overlap=0.03,
        name="minute hand visibly crosses the clock face",
    )
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="y",
        positive_elem=minute_blade,
        negative_elem=hour_blade,
        min_gap=0.018,
        max_gap=0.032,
        name="minute hand sits slightly proud of the hour hand",
    )

    hour_rest_dx = hour_rest_aabb[1][0] - hour_rest_aabb[0][0]
    hour_rest_dz = hour_rest_aabb[1][2] - hour_rest_aabb[0][2]
    ctx.check(
        "hour hand starts in a vertical rest pose",
        hour_rest_dz > hour_rest_dx * 2.0,
        f"rest blade extents dx={hour_rest_dx:.3f}, dz={hour_rest_dz:.3f}",
    )

    with ctx.pose({hour_joint: math.pi / 2.0}):
        hour_quarter_aabb = ctx.part_element_world_aabb(hour_hand, elem=hour_blade)
        assert hour_quarter_aabb is not None
        hour_quarter_dx = hour_quarter_aabb[1][0] - hour_quarter_aabb[0][0]
        hour_quarter_dz = hour_quarter_aabb[1][2] - hour_quarter_aabb[0][2]
        ctx.check(
            "hour hand rotates around the clock center on its revolute joint",
            hour_quarter_dx > hour_quarter_dz * 2.0,
            f"quarter-turn blade extents dx={hour_quarter_dx:.3f}, dz={hour_quarter_dz:.3f}",
        )
        ctx.expect_contact(
            hour_hand,
            tower,
            elem_a=hour_hub,
            elem_b=clock_face,
            name="hour hand hub remains seated at quarter turn",
        )
        ctx.expect_within(
            hour_hand,
            tower,
            axes="xz",
            inner_elem=hour_blade,
            outer_elem=clock_face,
            name="hour hand remains on the dial at quarter turn",
        )

    with ctx.pose({hour_joint: 1.35, minute_joint: -1.65}):
        ctx.expect_contact(
            hour_hand,
            tower,
            elem_a=hour_hub,
            elem_b=clock_face,
            name="hour hand hub remains seated while the hand turns",
        )
        ctx.expect_contact(
            minute_hand,
            hour_hand,
            elem_a=minute_hub,
            elem_b=hour_hub,
            name="minute hand remains stacked while the hands turn",
        )
        ctx.expect_within(
            hour_hand,
            tower,
            axes="xz",
            inner_elem=hour_blade,
            outer_elem=clock_face,
            name="hour hand remains on the clock in an alternate pose",
        )
        ctx.expect_within(
            minute_hand,
            tower,
            axes="xz",
            inner_elem=minute_blade,
            outer_elem=clock_face,
            name="minute hand remains on the clock in an alternate pose",
        )

    hour_limits = hour_joint.motion_limits
    minute_limits = minute_joint.motion_limits
    if hour_limits is not None and hour_limits.lower is not None and hour_limits.upper is not None:
        with ctx.pose({hour_joint: hour_limits.lower, minute_joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hour_hand_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="hour_hand_lower_no_floating")
            ctx.expect_contact(
                hour_hand,
                tower,
                elem_a=hour_hub,
                elem_b=clock_face,
                name="hour hand lower limit keeps hub seated",
            )
        with ctx.pose({hour_joint: hour_limits.upper, minute_joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hour_hand_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="hour_hand_upper_no_floating")
            ctx.expect_contact(
                hour_hand,
                tower,
                elem_a=hour_hub,
                elem_b=clock_face,
                name="hour hand upper limit keeps hub seated",
            )
    if minute_limits is not None and minute_limits.lower is not None and minute_limits.upper is not None:
        with ctx.pose({hour_joint: 0.0, minute_joint: minute_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="minute_hand_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="minute_hand_lower_no_floating")
            ctx.expect_contact(
                minute_hand,
                hour_hand,
                elem_a=minute_hub,
                elem_b=hour_hub,
                name="minute hand lower limit keeps stacked hub contact",
            )
        with ctx.pose({hour_joint: 0.0, minute_joint: minute_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="minute_hand_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="minute_hand_upper_no_floating")
            ctx.expect_contact(
                minute_hand,
                hour_hand,
                elem_a=minute_hub,
                elem_b=hour_hub,
                name="minute hand upper limit keeps stacked hub contact",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
