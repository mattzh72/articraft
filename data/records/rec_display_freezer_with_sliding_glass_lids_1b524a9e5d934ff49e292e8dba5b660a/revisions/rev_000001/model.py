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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convenience_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.95, 0.96, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.17, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    glass = model.material("glass", rgba=(0.69, 0.84, 0.94, 0.30))
    brass = model.material("brass", rgba=(0.73, 0.62, 0.27, 1.0))

    outer_x = 1.40
    outer_y = 0.76
    outer_z = 0.86
    floor_t = 0.08
    wall_t = 0.04
    wall_h = outer_z - floor_t - 0.05
    top_cap_h = 0.05
    top_rail_w = 0.065
    track_len = outer_x - 2.0 * wall_t
    track_w = 0.014
    lower_track_z = 0.823
    upper_track_z = 0.840
    track_y = 0.297

    panel_len = 0.72
    panel_depth = 0.608
    frame_w = 0.028
    runner_w = 0.014
    runner_t = 0.005
    glass_t = 0.004
    frame_inset = 0.014
    slide_travel = 0.48

    body = model.part("cabinet")
    body.visual(
        Box((outer_x, outer_y, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=cabinet_white,
        name="base_pan",
    )
    body.visual(
        Box((wall_t, outer_y, wall_h)),
        origin=Origin(
            xyz=(-outer_x / 2.0 + wall_t / 2.0, 0.0, floor_t + wall_h / 2.0)
        ),
        material=cabinet_white,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_y, wall_h)),
        origin=Origin(
            xyz=(outer_x / 2.0 - wall_t / 2.0, 0.0, floor_t + wall_h / 2.0)
        ),
        material=cabinet_white,
        name="right_wall",
    )
    body.visual(
        Box((outer_x - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(
            xyz=(0.0, -outer_y / 2.0 + wall_t / 2.0, floor_t + wall_h / 2.0)
        ),
        material=cabinet_white,
        name="front_wall",
    )
    body.visual(
        Box((outer_x - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(
            xyz=(0.0, outer_y / 2.0 - wall_t / 2.0, floor_t + wall_h / 2.0)
        ),
        material=cabinet_white,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, outer_y, top_cap_h)),
        origin=Origin(
            xyz=(-outer_x / 2.0 + wall_t / 2.0, 0.0, outer_z - top_cap_h / 2.0)
        ),
        material=aluminum,
        name="left_top_cap",
    )
    body.visual(
        Box((wall_t, outer_y, top_cap_h)),
        origin=Origin(
            xyz=(outer_x / 2.0 - wall_t / 2.0, 0.0, outer_z - top_cap_h / 2.0)
        ),
        material=aluminum,
        name="right_top_cap",
    )
    body.visual(
        Box((track_len, top_rail_w, top_cap_h)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + top_rail_w / 2.0, outer_z - top_cap_h / 2.0)),
        material=aluminum,
        name="front_top_rail",
    )
    body.visual(
        Box((track_len, top_rail_w, top_cap_h)),
        origin=Origin(xyz=(0.0, outer_y / 2.0 - top_rail_w / 2.0, outer_z - top_cap_h / 2.0)),
        material=aluminum,
        name="rear_top_rail",
    )
    body.visual(
        Box((track_len, track_w, 0.008)),
        origin=Origin(xyz=(0.0, -track_y, lower_track_z)),
        material=trim_dark,
        name="front_track",
    )
    body.visual(
        Box((track_len, track_w, 0.008)),
        origin=Origin(xyz=(0.0, track_y, lower_track_z)),
        material=trim_dark,
        name="rear_track",
    )
    body.visual(
        Box((track_len, track_w, 0.008)),
        origin=Origin(xyz=(0.0, -track_y, upper_track_z)),
        material=aluminum,
        name="upper_front_track",
    )
    body.visual(
        Box((track_len, track_w, 0.008)),
        origin=Origin(xyz=(0.0, track_y, upper_track_z)),
        material=aluminum,
        name="upper_rear_track",
    )
    body.visual(
        Box((outer_x - 0.02, outer_y - 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=trim_dark,
        name="base_skirt",
    )

    lock_y = -0.12
    key_z = 0.735
    flap_h = 0.084
    hinge_z = key_z + flap_h / 2.0
    body.visual(
        Box((0.004, 0.050, 0.076)),
        origin=Origin(xyz=(outer_x / 2.0 + 0.002, lock_y, key_z)),
        material=aluminum,
        name="lock_escutcheon",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(
            xyz=(outer_x / 2.0 + 0.009, lock_y, key_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="key_cylinder",
    )
    body.visual(
        Box((0.020, 0.060, 0.010)),
        origin=Origin(xyz=(outer_x / 2.0 + 0.010, lock_y, hinge_z)),
        material=aluminum,
        name="lock_hinge_mount",
    )

    def add_slider_panel(
        part_name: str,
        sign: float,
        z0: float,
        *,
        frame_h: float,
        meeting_h: float,
    ) -> None:
        panel = model.part(part_name)
        panel_center_x = sign * (panel_len / 2.0)
        outer_rail_x = sign * (frame_w / 2.0)
        meeting_rail_x = sign * (panel_len - frame_w / 2.0)
        frame_center_z = runner_t + frame_h / 2.0
        meeting_center_z = runner_t + meeting_h / 2.0
        glass_center_z = runner_t + frame_h - glass_t / 2.0 - 0.001

        panel.visual(
            Box((panel_len - 2.0 * frame_w, runner_w, runner_t)),
            origin=Origin(
                xyz=(panel_center_x, -(panel_depth / 2.0 - runner_w / 2.0), runner_t / 2.0)
            ),
            material=trim_dark,
            name="runner_front",
        )
        panel.visual(
            Box((panel_len - 2.0 * frame_w, runner_w, runner_t)),
            origin=Origin(
                xyz=(panel_center_x, panel_depth / 2.0 - runner_w / 2.0, runner_t / 2.0)
            ),
            material=trim_dark,
            name="runner_rear",
        )
        panel.visual(
            Box((panel_len - 2.0 * frame_w, frame_w, frame_h)),
            origin=Origin(
                xyz=(
                    panel_center_x,
                    -(panel_depth / 2.0 - frame_w / 2.0 - frame_inset),
                    frame_center_z,
                )
            ),
            material=aluminum,
            name="front_frame",
        )
        panel.visual(
            Box((panel_len - 2.0 * frame_w, frame_w, frame_h)),
            origin=Origin(
                xyz=(
                    panel_center_x,
                    panel_depth / 2.0 - frame_w / 2.0 - frame_inset,
                    frame_center_z,
                )
            ),
            material=aluminum,
            name="rear_frame",
        )
        panel.visual(
            Box((frame_w, panel_depth - 2.0 * frame_w, frame_h)),
            origin=Origin(xyz=(outer_rail_x, 0.0, frame_center_z)),
            material=aluminum,
            name="outer_end_frame",
        )
        panel.visual(
            Box((frame_w, panel_depth - 2.0 * frame_w, meeting_h)),
            origin=Origin(xyz=(meeting_rail_x, 0.0, meeting_center_z)),
            material=trim_dark,
            name="meeting_rail",
        )
        panel.visual(
            Box((panel_len - 2.0 * frame_w, panel_depth - 2.0 * frame_w, glass_t)),
            origin=Origin(xyz=(panel_center_x, 0.0, glass_center_z)),
            material=glass,
            name="glass_panel",
        )

        if sign > 0.0:
            axis = (1.0, 0.0, 0.0)
            joint_origin = Origin(xyz=(-track_len / 2.0, 0.0, z0))
            joint_name = "cabinet_to_left_lid"
        else:
            axis = (-1.0, 0.0, 0.0)
            joint_origin = Origin(xyz=(track_len / 2.0, 0.0, z0))
            joint_name = "cabinet_to_right_lid"

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=panel,
            origin=joint_origin,
            axis=axis,
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=0.45,
                lower=0.0,
                upper=slide_travel,
            ),
        )

    add_slider_panel(
        "left_lid",
        sign=1.0,
        z0=0.828,
        frame_h=0.010,
        meeting_h=0.009,
    )
    add_slider_panel(
        "right_lid",
        sign=-1.0,
        z0=0.845,
        frame_h=0.012,
        meeting_h=0.018,
    )

    flap = model.part("lock_flap")
    flap.visual(
        Cylinder(radius=0.0035, length=0.058),
        origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.004, 0.058, flap_h)),
        origin=Origin(xyz=(0.002, 0.0, -flap_h / 2.0)),
        material=aluminum,
        name="cover_plate",
    )
    flap.visual(
        Box((0.010, 0.028, 0.008)),
        origin=Origin(xyz=(0.007, 0.0, -flap_h + 0.004)),
        material=trim_dark,
        name="pull_tab",
    )

    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(outer_x / 2.0 + 0.019, lock_y, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    cabinet = object_model.get_part("cabinet")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    lock_flap = object_model.get_part("lock_flap")

    left_slide = object_model.get_articulation("cabinet_to_left_lid")
    right_slide = object_model.get_articulation("cabinet_to_right_lid")
    flap_hinge = object_model.get_articulation("cabinet_to_lock_flap")

    ctx.check("cabinet part exists", cabinet is not None)
    ctx.check("left lid part exists", left_lid is not None)
    ctx.check("right lid part exists", right_lid is not None)
    ctx.check("lock flap part exists", lock_flap is not None)
    ctx.check("left lid joint exists", left_slide is not None)
    ctx.check("right lid joint exists", right_slide is not None)
    ctx.check("lock flap joint exists", flap_hinge is not None)

    with ctx.pose({left_slide: 0.0, right_slide: 0.0, flap_hinge: 0.0}):
        ctx.expect_overlap(
            left_lid,
            right_lid,
            axes="xy",
            elem_a="glass_panel",
            elem_b="glass_panel",
            min_overlap=0.06,
            name="closed sliders overlap at the center seam",
        )
        ctx.expect_gap(
            left_lid,
            cabinet,
            axis="z",
            positive_elem="runner_front",
            negative_elem="front_track",
            min_gap=0.0,
            max_gap=0.003,
            name="left lid sits just above the lower front track",
        )
        ctx.expect_gap(
            right_lid,
            cabinet,
            axis="z",
            positive_elem="runner_front",
            negative_elem="upper_front_track",
            min_gap=0.0,
            max_gap=0.003,
            name="right lid sits just above the upper front track",
        )
        ctx.expect_overlap(
            lock_flap,
            cabinet,
            axes="yz",
            elem_a="cover_plate",
            elem_b="key_cylinder",
            min_overlap=0.020,
            name="closed lock flap covers the side key cylinder",
        )
        ctx.expect_gap(
            lock_flap,
            cabinet,
            axis="x",
            positive_elem="cover_plate",
            negative_elem="key_cylinder",
            min_gap=0.0005,
            max_gap=0.010,
            name="closed lock flap sits just proud of the key cylinder",
        )
        left_rest = ctx.part_world_position(left_lid)
        right_rest = ctx.part_world_position(right_lid)
        flap_closed_aabb = ctx.part_element_world_aabb(lock_flap, elem="cover_plate")

    with ctx.pose({left_slide: left_slide.motion_limits.upper}):
        left_open = ctx.part_world_position(left_lid)

    with ctx.pose({right_slide: right_slide.motion_limits.upper}):
        right_open = ctx.part_world_position(right_lid)

    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        flap_open_aabb = ctx.part_element_world_aabb(lock_flap, elem="cover_plate")

    ctx.check(
        "left lid opens toward the center",
        left_rest is not None
        and left_open is not None
        and left_open[0] > left_rest[0] + 0.20,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right lid opens toward the center",
        right_rest is not None
        and right_open is not None
        and right_open[0] < right_rest[0] - 0.20,
        details=f"rest={right_rest}, open={right_open}",
    )
    ctx.check(
        "lock flap opens outward",
        flap_closed_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][0] > flap_closed_aabb[1][0] + 0.020,
        details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
