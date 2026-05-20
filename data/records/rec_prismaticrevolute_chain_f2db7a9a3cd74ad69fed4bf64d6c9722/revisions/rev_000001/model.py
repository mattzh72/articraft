from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drawer_shuttle_inspection")

    base_l = 0.72
    base_w = 0.24
    plate_t = 0.012
    rail_l = 0.60
    rail_w = 0.024
    rail_h = 0.026
    rail_y = 0.055

    shuttle_rest_x = -0.12
    shuttle_travel = 0.26
    rail_top_z = plate_t + rail_h

    tray_l = 0.30
    tray_w = 0.17
    tray_h = 0.052
    wall_t = 0.006
    floor_t = 0.004
    tray_z0 = 0.018
    slider_l = 0.20
    slider_w = 0.030
    slider_h = tray_z0

    pivot_x = 0.090
    pivot_y = tray_w / 2.0 + 0.028
    pivot_z = 0.043
    pivot_block_l = 0.040
    pivot_block_t = 0.028
    pivot_block_h = 0.050

    hub_r = 0.012
    hub_len = 0.014
    arm_l = 0.076
    arm_t = 0.012
    arm_h = 0.016
    paddle_l = 0.026
    paddle_t = 0.004
    paddle_h = 0.038
    paddle_x = 0.097

    frame_mat = model.material("frame_graphite", rgba=(0.22, 0.24, 0.28, 1.0))
    rail_mat = model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    shuttle_mat = model.material("shuttle_silver", rgba=(0.66, 0.70, 0.75, 1.0))
    slider_mat = model.material("slider_polymer", rgba=(0.10, 0.10, 0.11, 1.0))
    arm_mat = model.material("arm_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    paddle_mat = model.material("paddle_orange", rgba=(0.95, 0.48, 0.12, 1.0))

    model = ArticulatedObject(name="drawer_shuttle_inspection")

    guide_base = model.part("guide_base")

    base_plate = (
        cq.Workplane("XY")
        .box(base_l, base_w, plate_t)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, plate_t / 2.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.030, 0.168, 0.040)
        .translate((-base_l / 2.0 + 0.060, 0.0, plate_t + 0.020))
    )
    front_bridge = (
        cq.Workplane("XY")
        .box(0.030, 0.168, 0.040)
        .translate((base_l / 2.0 - 0.060, 0.0, plate_t + 0.020))
    )
    foot_offsets = (
        (-0.24, -0.08),
        (-0.24, 0.08),
        (0.24, -0.08),
        (0.24, 0.08),
    )
    feet = None
    for x_off, y_off in foot_offsets:
        foot = (
            cq.Workplane("XY")
            .box(0.090, 0.040, 0.012)
            .edges("|Z")
            .fillet(0.004)
            .translate((x_off, y_off, 0.006))
        )
        feet = foot if feet is None else feet.union(foot)
    base_frame_shape = base_plate.union(rear_bridge).union(front_bridge).union(feet)

    guide_base.visual(
        mesh_from_cadquery(base_frame_shape, "guide_base_frame"),
        material=frame_mat,
        name="base_frame",
    )
    guide_base.visual(
        Box((rail_l, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, rail_y, plate_t + rail_h / 2.0)),
        material=rail_mat,
        name="left_rail",
    )
    guide_base.visual(
        Box((rail_l, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, -rail_y, plate_t + rail_h / 2.0)),
        material=rail_mat,
        name="right_rail",
    )

    shuttle = model.part("shuttle")

    tray_outer = cq.Workplane("XY").box(tray_l, tray_w, tray_h).translate(
        (0.0, 0.0, tray_z0 + tray_h / 2.0)
    )
    tray_inner = cq.Workplane("XY").box(
        tray_l - 2.0 * wall_t,
        tray_w - 2.0 * wall_t,
        tray_h - floor_t,
    ).translate((0.0, 0.0, tray_z0 + floor_t + (tray_h - floor_t) / 2.0))
    front_fascia = cq.Workplane("XY").box(0.016, tray_w, 0.060).translate(
        (tray_l / 2.0 - 0.008, 0.0, tray_z0 + 0.030)
    )
    tray_shell_shape = tray_outer.cut(tray_inner).union(front_fascia)

    pivot_block_shape = (
        cq.Workplane("XY")
        .box(pivot_block_l, pivot_block_t, pivot_block_h)
        .translate(
            (
                pivot_x,
                tray_w / 2.0 + pivot_block_t / 2.0,
                tray_z0 + pivot_block_h / 2.0,
            )
        )
    )

    shuttle.visual(
        mesh_from_cadquery(tray_shell_shape, "shuttle_tray_shell"),
        material=shuttle_mat,
        name="tray_shell",
    )
    shuttle.visual(
        Box((slider_l, slider_w, slider_h)),
        origin=Origin(xyz=(0.0, rail_y, slider_h / 2.0)),
        material=slider_mat,
        name="left_slider",
    )
    shuttle.visual(
        Box((slider_l, slider_w, slider_h)),
        origin=Origin(xyz=(0.0, -rail_y, slider_h / 2.0)),
        material=slider_mat,
        name="right_slider",
    )
    shuttle.visual(
        mesh_from_cadquery(pivot_block_shape, "shuttle_pivot_block"),
        material=frame_mat,
        name="pivot_block",
    )

    inspection_link = model.part("inspection_link")

    arm_body_shape = (
        cq.Workplane("XY")
        .box(arm_l, arm_t, arm_h)
        .translate((hub_r + arm_l / 2.0 - 0.004, 0.0, 0.0))
    )
    paddle_shape = (
        cq.Workplane("XY")
        .box(paddle_l, paddle_t, paddle_h)
        .edges("|Y")
        .fillet(0.004)
        .translate((paddle_x, 0.0, 0.0))
    )

    inspection_link.visual(
        Cylinder(radius=hub_r, length=hub_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_mat,
        name="arm_hub",
    )
    inspection_link.visual(
        mesh_from_cadquery(arm_body_shape, "inspection_link_arm_body"),
        material=arm_mat,
        name="arm_body",
    )
    inspection_link.visual(
        mesh_from_cadquery(paddle_shape, "inspection_link_paddle"),
        material=paddle_mat,
        name="inspection_paddle",
    )

    model.articulation(
        "guide_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=guide_base,
        child=shuttle,
        origin=Origin(xyz=(shuttle_rest_x, 0.0, rail_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=shuttle_travel,
        ),
    )
    model.articulation(
        "shuttle_to_inspection_link",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=inspection_link,
        origin=Origin(xyz=(pivot_x, pivot_y + hub_len / 2.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    guide_base = object_model.get_part("guide_base")
    shuttle = object_model.get_part("shuttle")
    inspection_link = object_model.get_part("inspection_link")
    guide_joint = object_model.get_articulation("guide_to_shuttle")
    link_joint = object_model.get_articulation("shuttle_to_inspection_link")

    left_rail = guide_base.get_visual("left_rail")
    right_rail = guide_base.get_visual("right_rail")
    left_slider = shuttle.get_visual("left_slider")
    right_slider = shuttle.get_visual("right_slider")
    pivot_block = shuttle.get_visual("pivot_block")
    arm_hub = inspection_link.get_visual("arm_hub")
    paddle = inspection_link.get_visual("inspection_paddle")

    ctx.check(
        "core parts present",
        all(part is not None for part in (guide_base, shuttle, inspection_link)),
    )
    ctx.check(
        "core articulations present",
        all(joint is not None for joint in (guide_joint, link_joint)),
    )
    ctx.check(
        "guide joint is prismatic on x",
        guide_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in guide_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={guide_joint.articulation_type}, axis={guide_joint.axis}",
    )
    ctx.check(
        "inspection link joint is revolute on y",
        link_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in link_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={link_joint.articulation_type}, axis={link_joint.axis}",
    )

    ctx.expect_contact(
        shuttle,
        guide_base,
        elem_a=left_slider,
        elem_b=left_rail,
        name="left slider rides on left rail",
    )
    ctx.expect_contact(
        shuttle,
        guide_base,
        elem_a=right_slider,
        elem_b=right_rail,
        name="right slider rides on right rail",
    )
    ctx.expect_contact(
        inspection_link,
        shuttle,
        elem_a=arm_hub,
        elem_b=pivot_block,
        name="inspection link hub seats on pivot block",
    )
    ctx.expect_gap(
        shuttle,
        guide_base,
        axis="y",
        positive_elem=pivot_block,
        negative_elem=right_rail,
        min_gap=0.015,
        name="pivot block sits outboard of guide rail support",
    )

    with ctx.pose({guide_joint: 0.0}):
        shuttle_p0 = ctx.part_world_position(shuttle)
    with ctx.pose({guide_joint: 0.20}):
        shuttle_p1 = ctx.part_world_position(shuttle)

    if shuttle_p0 is not None and shuttle_p1 is not None:
        dx = shuttle_p1[0] - shuttle_p0[0]
        dy = shuttle_p1[1] - shuttle_p0[1]
        dz = shuttle_p1[2] - shuttle_p0[2]
        ctx.check(
            "shuttle translates along guide axis",
            abs(dx - 0.20) <= 0.003 and abs(dy) <= 0.001 and abs(dz) <= 0.001,
            details=f"delta=({dx:.4f}, {dy:.4f}, {dz:.4f})",
        )
    else:
        ctx.fail("shuttle translates along guide axis", "missing shuttle world positions")

    def aabb_center(aabb):
        if aabb is None:
            return None
        a_min, a_max = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(a_min, a_max))

    with ctx.pose({link_joint: 0.0}):
        paddle_aabb_0 = ctx.part_element_world_aabb(inspection_link, elem=paddle)
    with ctx.pose({link_joint: 0.45}):
        paddle_aabb_1 = ctx.part_element_world_aabb(inspection_link, elem=paddle)

    paddle_c0 = aabb_center(paddle_aabb_0)
    paddle_c1 = aabb_center(paddle_aabb_1)
    if paddle_c0 is not None and paddle_c1 is not None:
        ctx.check(
            "inspection paddle swings in xz plane",
            abs(paddle_c1[2] - paddle_c0[2]) >= 0.020
            and abs(paddle_c1[0] - paddle_c0[0]) >= 0.008
            and abs(paddle_c1[1] - paddle_c0[1]) <= 0.001,
            details=(
                f"center0={paddle_c0}, center1={paddle_c1}, "
                f"delta=({paddle_c1[0] - paddle_c0[0]:.4f}, "
                f"{paddle_c1[1] - paddle_c0[1]:.4f}, "
                f"{paddle_c1[2] - paddle_c0[2]:.4f})"
            ),
        )
    else:
        ctx.fail("inspection paddle swings in xz plane", "missing paddle AABBs")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
