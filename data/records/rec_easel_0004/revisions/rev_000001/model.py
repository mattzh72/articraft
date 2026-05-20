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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

FRAME_HEIGHT = 1.62
POST_WIDTH = 0.05
POST_DEPTH = 0.03
POST_X = 0.21

TRAY_HOME_Z = 0.42
TRAY_TRAVEL = 0.56

BRACE_OPEN_ANGLE = 0.34
BRACE_LENGTH = 1.68


def _brace_leg_center(top_y: float, top_z: float) -> tuple[float, float]:
    return (
        top_y - 0.5 * BRACE_LENGTH * math.sin(BRACE_OPEN_ANGLE),
        top_z - 0.5 * BRACE_LENGTH * math.cos(BRACE_OPEN_ANGLE),
    )


def _extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
    return aabb[1][axis] - aabb[0][axis]


def _check_joint_limits_clear(ctx: TestContext, joint) -> None:
    limits = joint.motion_limits
    if limits is None or limits.lower is None or limits.upper is None:
        return
    for label, value in (("lower", limits.lower), ("upper", limits.upper)):
        with ctx.pose({joint: value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_{label}_no_floating")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="h_frame_painters_easel")

    oak = model.material("oak", rgba=(0.68, 0.54, 0.36, 1.0))
    warm_oak = model.material("warm_oak", rgba=(0.76, 0.62, 0.42, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.42, 0.30, 0.20, 1.0))
    hardware = model.material("hardware", rgba=(0.22, 0.23, 0.25, 1.0))

    frame = model.part("front_frame")
    frame.visual(
        Box((POST_WIDTH, POST_DEPTH, 1.52)),
        origin=Origin(xyz=(-POST_X, 0.0, 0.81)),
        material=oak,
        name="left_post",
    )
    frame.visual(
        Box((POST_WIDTH, POST_DEPTH, 1.52)),
        origin=Origin(xyz=(POST_X, 0.0, 0.81)),
        material=oak,
        name="right_post",
    )
    frame.visual(
        Box((0.47, POST_DEPTH, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT - 0.025)),
        material=oak,
        name="top_beam",
    )
    frame.visual(
        Box((0.37, POST_DEPTH, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=oak,
        name="lower_beam",
    )
    frame.visual(
        Box((0.07, 0.42, 0.05)),
        origin=Origin(xyz=(-POST_X, 0.11, 0.025)),
        material=dark_wood,
        name="left_foot",
    )
    frame.visual(
        Box((0.07, 0.42, 0.05)),
        origin=Origin(xyz=(POST_X, 0.11, 0.025)),
        material=dark_wood,
        name="right_foot",
    )
    frame.visual(
        Box((0.40, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.11, 0.075)),
        material=dark_wood,
        name="front_stretcher",
    )

    tray = model.part("canvas_tray")
    tray.visual(
        Box((0.38, 0.11, 0.02)),
        origin=Origin(xyz=(0.0, 0.07, 0.0)),
        material=warm_oak,
        name="tray_shelf",
    )
    tray.visual(
        Box((0.38, 0.014, 0.035)),
        origin=Origin(xyz=(0.0, 0.118, 0.0275)),
        material=warm_oak,
        name="tray_lip",
    )
    tray.visual(
        Box((0.38, 0.025, 0.04)),
        origin=Origin(xyz=(0.0, 0.0275, -0.015)),
        material=warm_oak,
        name="tray_back_rib",
    )
    tray.visual(
        Box((POST_WIDTH, 0.03, 0.22)),
        origin=Origin(xyz=(-POST_X, 0.03, 0.04)),
        material=hardware,
        name="left_guide",
    )
    tray.visual(
        Box((POST_WIDTH, 0.03, 0.22)),
        origin=Origin(xyz=(POST_X, 0.03, 0.04)),
        material=hardware,
        name="right_guide",
    )

    rear_brace = model.part("rear_brace")
    rear_brace.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(xyz=(-0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="left_hinge_barrel",
    )
    rear_brace.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="right_hinge_barrel",
    )
    rear_brace.visual(
        Box((0.31, 0.022, 0.05)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=dark_wood,
        name="brace_yoke",
    )
    brace_top_y = -0.04
    brace_top_z = -0.012
    leg_center_y, leg_center_z = _brace_leg_center(brace_top_y, brace_top_z)
    rear_brace.visual(
        Box((0.06, 0.03, BRACE_LENGTH)),
        origin=Origin(
            xyz=(0.0, leg_center_y, leg_center_z),
            rpy=(-BRACE_OPEN_ANGLE, 0.0, 0.0),
        ),
        material=dark_wood,
        name="brace_leg",
    )
    rear_brace.visual(
        Box((0.24, 0.08, 0.035)),
        origin=Origin(
            xyz=(
                0.0,
                brace_top_y - math.sin(BRACE_OPEN_ANGLE) * BRACE_LENGTH,
                brace_top_z - math.cos(BRACE_OPEN_ANGLE) * BRACE_LENGTH + 0.0175,
            ),
        ),
        material=dark_wood,
        name="rear_foot",
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.25,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )
    model.articulation(
        "rear_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_brace,
        origin=Origin(xyz=(0.0, -0.026, FRAME_HEIGHT - 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.15,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("front_frame")
    tray = object_model.get_part("canvas_tray")
    rear_brace = object_model.get_part("rear_brace")
    tray_slide = object_model.get_articulation("tray_slide")
    rear_brace_hinge = object_model.get_articulation("rear_brace_hinge")

    left_post = frame.get_visual("left_post")
    right_post = frame.get_visual("right_post")
    top_beam = frame.get_visual("top_beam")
    lower_beam = frame.get_visual("lower_beam")
    left_foot = frame.get_visual("left_foot")
    tray_shelf = tray.get_visual("tray_shelf")
    tray_lip = tray.get_visual("tray_lip")
    left_guide = tray.get_visual("left_guide")
    right_guide = tray.get_visual("right_guide")
    left_hinge_barrel = rear_brace.get_visual("left_hinge_barrel")
    right_hinge_barrel = rear_brace.get_visual("right_hinge_barrel")
    rear_foot = rear_brace.get_visual("rear_foot")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=64,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    frame_aabb = ctx.part_world_aabb(frame)
    rear_foot_aabb = ctx.part_element_world_aabb(rear_brace, elem=rear_foot)
    if frame_aabb is not None:
        frame_height = _extent(frame_aabb, 2)
        frame_width = _extent(frame_aabb, 0)
        ctx.check(
            "frame_realistic_scale",
            1.55 <= frame_height <= 1.68 and 0.44 <= frame_width <= 0.52,
            f"frame dims were width={frame_width:.3f} m height={frame_height:.3f} m",
        )
    if rear_foot_aabb is not None:
        rear_foot_bottom = rear_foot_aabb[0][2]
        ctx.check(
            "rear_foot_near_floor",
            -0.005 <= rear_foot_bottom <= 0.08,
            f"rear foot bottom was z={rear_foot_bottom:.3f} m",
        )

    ctx.expect_origin_distance(tray, frame, axes="x", max_dist=0.001)
    ctx.expect_overlap(tray, frame, axes="x", min_overlap=0.36)
    ctx.expect_contact(tray, frame, elem_a=left_guide, elem_b=left_post)
    ctx.expect_contact(tray, frame, elem_a=right_guide, elem_b=right_post)
    ctx.expect_gap(
        tray,
        frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_guide,
        negative_elem=left_post,
        name="left_guide_seats_on_left_post",
    )
    ctx.expect_gap(
        tray,
        frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_guide,
        negative_elem=right_post,
        name="right_guide_seats_on_right_post",
    )
    ctx.expect_contact(rear_brace, frame, elem_a=left_hinge_barrel, elem_b=top_beam)
    ctx.expect_contact(rear_brace, frame, elem_a=right_hinge_barrel, elem_b=top_beam)
    ctx.expect_gap(
        frame,
        rear_brace,
        axis="y",
        min_gap=0.40,
        positive_elem=left_foot,
        negative_elem=rear_foot,
        name="rear_brace_spreads_behind_front_feet",
    )

    with ctx.pose({tray_slide: tray_slide.motion_limits.lower}):
        ctx.expect_gap(
            tray,
            frame,
            axis="z",
            min_gap=0.03,
            max_gap=0.12,
            positive_elem=tray_shelf,
            negative_elem=lower_beam,
            name="tray_low_pose_above_lower_beam",
        )
        ctx.expect_contact(tray, frame, elem_a=left_guide, elem_b=left_post)
        ctx.expect_contact(tray, frame, elem_a=right_guide, elem_b=right_post)

    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_gap(
            frame,
            tray,
            axis="z",
            min_gap=0.12,
            max_gap=0.70,
            positive_elem=top_beam,
            negative_elem=tray_lip,
            name="tray_raised_below_top_beam",
        )
        ctx.expect_contact(tray, frame, elem_a=left_guide, elem_b=left_post)
        ctx.expect_contact(tray, frame, elem_a=right_guide, elem_b=right_post)

    with ctx.pose({rear_brace_hinge: rear_brace_hinge.motion_limits.upper}):
        ctx.expect_gap(
            frame,
            rear_brace,
            axis="y",
            min_gap=0.01,
            max_gap=0.30,
            positive_elem=top_beam,
            negative_elem=rear_foot,
            name="rear_brace_folds_toward_frame",
        )

    _check_joint_limits_clear(ctx, tray_slide)
    _check_joint_limits_clear(ctx, rear_brace_hinge)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
