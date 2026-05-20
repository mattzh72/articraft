from __future__ import annotations

from math import pi

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


BODY_W = 0.58
BODY_D = 0.39
BODY_H = 0.305
LID_H = 0.112
WALL = 0.016
LID_WALL = 0.014
SEAM_GAP = 0.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    shell = model.material("shell", rgba=(0.15, 0.16, 0.18, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    latch_color = model.material("latch", rgba=(0.84, 0.19, 0.09, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.28, 0.29, 0.31, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.62, 0.70, 0.78, 0.45))
    organizer_bin = model.material("organizer_bin", rgba=(0.18, 0.18, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=shell,
        name="base_floor",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2 + WALL / 2, 0.0, BODY_H / 2)),
        material=shell,
        name="left_wall",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2 - WALL / 2, 0.0, BODY_H / 2)),
        material=shell,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 2 * WALL, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_D / 2 + WALL / 2, BODY_H / 2)),
        material=shell,
        name="front_wall",
    )
    body.visual(
        Box((0.084, WALL, BODY_H - 0.055)),
        origin=Origin(xyz=(-0.232, BODY_D / 2 - WALL / 2, (BODY_H - 0.055) / 2)),
        material=shell,
        name="rear_wall_0",
    )
    body.visual(
        Box((0.084, WALL, BODY_H - 0.055)),
        origin=Origin(xyz=(0.232, BODY_D / 2 - WALL / 2, (BODY_H - 0.055) / 2)),
        material=shell,
        name="rear_wall_1",
    )
    body.visual(
        Box((0.188, WALL, 0.110)),
        origin=Origin(xyz=(0.0, BODY_D / 2 - WALL / 2, 0.055)),
        material=shell,
        name="rear_wall_center",
    )
    body.visual(
        Box((BODY_W - 2 * WALL, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, -BODY_D / 2 + 0.030, BODY_H - 0.008)),
        material=shell_dark,
        name="front_rim",
    )
    body.visual(
        Box((0.028, BODY_D - 0.060, 0.020)),
        origin=Origin(xyz=(-BODY_W / 2 + 0.030, 0.0, BODY_H - 0.008)),
        material=shell_dark,
        name="left_rim",
    )
    body.visual(
        Box((0.028, BODY_D - 0.060, 0.020)),
        origin=Origin(xyz=(BODY_W / 2 - 0.030, 0.0, BODY_H - 0.008)),
        material=shell_dark,
        name="right_rim",
    )
    body.visual(
        Box((0.050, 0.030, 0.360)),
        origin=Origin(xyz=(-0.162, BODY_D / 2 - 0.013, 0.235)),
        material=shell_dark,
        name="left_handle_sleeve",
    )
    body.visual(
        Box((0.050, 0.030, 0.360)),
        origin=Origin(xyz=(0.162, BODY_D / 2 - 0.013, 0.235)),
        material=shell_dark,
        name="right_handle_sleeve",
    )
    body.visual(
        Box((0.374, 0.014, 0.072)),
        origin=Origin(xyz=(0.0, BODY_D / 2 + 0.006, 0.118)),
        material=shell_dark,
        name="handle_bridge",
    )
    body.visual(
        Box((0.018, 0.060, 0.095)),
        origin=Origin(xyz=(-BODY_W / 2 - 0.009, 0.118, 0.094)),
        material=shell_dark,
        name="left_axle_mount",
    )
    body.visual(
        Box((0.018, 0.060, 0.095)),
        origin=Origin(xyz=(BODY_W / 2 + 0.009, 0.118, 0.094)),
        material=shell_dark,
        name="right_axle_mount",
    )
    body.visual(
        Box((BODY_W + 0.025, BODY_D - 0.02, 0.030)),
        origin=Origin(xyz=(0.0, 0.010, 0.030)),
        material=shell_dark,
        name="base_bumper",
    )

    lid = model.part("lid")
    lid.visual(
        Box((BODY_W, 0.286, 0.016)),
        origin=Origin(xyz=(0.0, -0.052, LID_H - 0.008)),
        material=shell,
        name="front_deck",
    )
    lid.visual(
        Box((0.102, 0.104, 0.016)),
        origin=Origin(xyz=(-0.240, 0.143, LID_H - 0.008)),
        material=shell,
        name="rear_deck_0",
    )
    lid.visual(
        Box((0.102, 0.104, 0.016)),
        origin=Origin(xyz=(0.240, 0.143, LID_H - 0.008)),
        material=shell,
        name="rear_deck_1",
    )
    lid.visual(
        Box((LID_WALL, BODY_D, LID_H - 0.016)),
        origin=Origin(xyz=(-BODY_W / 2 + LID_WALL / 2, 0.0, (LID_H - 0.016) / 2)),
        material=shell,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((LID_WALL, BODY_D, LID_H - 0.016)),
        origin=Origin(xyz=(BODY_W / 2 - LID_WALL / 2, 0.0, (LID_H - 0.016) / 2)),
        material=shell,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((BODY_W - 2 * LID_WALL, LID_WALL, LID_H - 0.016)),
        origin=Origin(xyz=(0.0, -BODY_D / 2 + LID_WALL / 2, (LID_H - 0.016) / 2)),
        material=shell,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.090, LID_WALL, LID_H - 0.016)),
        origin=Origin(xyz=(-0.238, BODY_D / 2 - LID_WALL / 2, (LID_H - 0.016) / 2)),
        material=shell,
        name="lid_rear_skirt_0",
    )
    lid.visual(
        Box((0.090, LID_WALL, LID_H - 0.016)),
        origin=Origin(xyz=(0.238, BODY_D / 2 - LID_WALL / 2, (LID_H - 0.016) / 2)),
        material=shell,
        name="lid_rear_skirt_1",
    )
    lid.visual(
        Box((0.238, 0.154, 0.020)),
        origin=Origin(xyz=(-0.140, -0.006, LID_H - 0.026)),
        material=organizer_bin,
        name="left_bin_floor",
    )
    lid.visual(
        Box((0.238, 0.154, 0.020)),
        origin=Origin(xyz=(0.140, -0.006, LID_H - 0.026)),
        material=organizer_bin,
        name="right_bin_floor",
    )
    lid.visual(
        Box((0.254, 0.012, 0.016)),
        origin=Origin(xyz=(-0.140, -0.083, LID_H - 0.024)),
        material=shell_dark,
        name="left_bin_front_rim",
    )
    lid.visual(
        Box((0.254, 0.012, 0.016)),
        origin=Origin(xyz=(0.140, -0.083, LID_H - 0.024)),
        material=shell_dark,
        name="right_bin_front_rim",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.FIXED,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BODY_H + SEAM_GAP)),
    )

    cover_0 = model.part("organizer_cover_0")
    cover_0.visual(
        Box((0.252, 0.156, 0.008)),
        origin=Origin(xyz=(0.0, -0.078, 0.004)),
        material=clear_smoke,
        name="cover_plate",
    )
    cover_0.visual(
        Box((0.252, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.005)),
        material=shell_dark,
        name="rear_frame",
    )
    cover_0.visual(
        Box((0.012, 0.156, 0.010)),
        origin=Origin(xyz=(-0.120, -0.078, 0.005)),
        material=shell_dark,
        name="left_frame",
    )
    cover_0.visual(
        Box((0.012, 0.156, 0.010)),
        origin=Origin(xyz=(0.120, -0.078, 0.005)),
        material=shell_dark,
        name="right_frame",
    )
    cover_0.visual(
        Box((0.068, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.151, 0.006)),
        material=shell_dark,
        name="finger_pull",
    )
    cover_0.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.007), rpy=(0.0, pi / 2, 0.0)),
        material=shell_dark,
        name="hinge_barrel",
    )

    cover_1 = model.part("organizer_cover_1")
    cover_1.visual(
        Box((0.252, 0.156, 0.008)),
        origin=Origin(xyz=(0.0, -0.078, 0.004)),
        material=clear_smoke,
        name="cover_plate",
    )
    cover_1.visual(
        Box((0.252, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.005)),
        material=shell_dark,
        name="rear_frame",
    )
    cover_1.visual(
        Box((0.012, 0.156, 0.010)),
        origin=Origin(xyz=(-0.120, -0.078, 0.005)),
        material=shell_dark,
        name="left_frame",
    )
    cover_1.visual(
        Box((0.012, 0.156, 0.010)),
        origin=Origin(xyz=(0.120, -0.078, 0.005)),
        material=shell_dark,
        name="right_frame",
    )
    cover_1.visual(
        Box((0.068, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.151, 0.006)),
        material=shell_dark,
        name="finger_pull",
    )
    cover_1.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.007), rpy=(0.0, pi / 2, 0.0)),
        material=shell_dark,
        name="hinge_barrel",
    )

    model.articulation(
        "lid_to_organizer_cover_0",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cover_0,
        origin=Origin(xyz=(-0.140, 0.071, LID_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=4.0, velocity=2.5),
    )
    model.articulation(
        "lid_to_organizer_cover_1",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cover_1,
        origin=Origin(xyz=(0.140, 0.071, LID_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=4.0, velocity=2.5),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.020, 0.014, 0.560)),
        origin=Origin(xyz=(-0.162, 0.0, -0.050)),
        material=rail_metal,
        name="left_rail",
    )
    handle.visual(
        Box((0.020, 0.014, 0.560)),
        origin=Origin(xyz=(0.162, 0.0, -0.050)),
        material=rail_metal,
        name="right_rail",
    )
    handle.visual(
        Box((0.360, 0.034, 0.024)),
        origin=Origin(xyz=(0.0, -0.004, 0.242)),
        material=shell_dark,
        name="grip_bar",
    )
    handle.visual(
        Box((0.290, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.004, 0.221)),
        material=shell,
        name="grip_face",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, BODY_D / 2 - 0.013, 0.415)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.290, effort=120.0, velocity=0.30),
    )

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.086, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    wheel_0.visual(
        Cylinder(radius=0.050, length=0.054),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=wheel_hub,
        name="hub",
    )

    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.086, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    wheel_1.visual(
        Cylinder(radius=0.050, length=0.054),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=wheel_hub,
        name="hub",
    )

    model.articulation(
        "body_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_0,
        origin=Origin(xyz=(-0.333, 0.118, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )
    model.articulation(
        "body_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_1,
        origin=Origin(xyz=(0.333, 0.118, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )

    latch_0 = model.part("latch_0")
    latch_0.visual(
        Cylinder(radius=0.008, length=0.062),
        origin=Origin(xyz=(-0.008, 0.0, 0.010), rpy=(pi / 2, 0.0, 0.0)),
        material=latch_color,
        name="pivot_barrel",
    )
    latch_0.visual(
        Box((0.012, 0.062, 0.122)),
        origin=Origin(xyz=(-0.006, 0.0, 0.061)),
        material=latch_color,
        name="latch_bar",
    )
    latch_0.visual(
        Box((0.018, 0.062, 0.020)),
        origin=Origin(xyz=(-0.009, 0.0, 0.114)),
        material=latch_color,
        name="hook",
    )

    latch_1 = model.part("latch_1")
    latch_1.visual(
        Cylinder(radius=0.008, length=0.062),
        origin=Origin(xyz=(0.008, 0.0, 0.010), rpy=(pi / 2, 0.0, 0.0)),
        material=latch_color,
        name="pivot_barrel",
    )
    latch_1.visual(
        Box((0.012, 0.062, 0.122)),
        origin=Origin(xyz=(0.006, 0.0, 0.061)),
        material=latch_color,
        name="latch_bar",
    )
    latch_1.visual(
        Box((0.018, 0.062, 0.020)),
        origin=Origin(xyz=(0.009, 0.0, 0.114)),
        material=latch_color,
        name="hook",
    )

    model.articulation(
        "body_to_latch_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch_0,
        origin=Origin(xyz=(-BODY_W / 2, -0.016, 0.212)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=8.0, velocity=3.0),
    )
    model.articulation(
        "body_to_latch_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch_1,
        origin=Origin(xyz=(BODY_W / 2, -0.016, 0.212)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=8.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cover_0 = object_model.get_part("organizer_cover_0")
    cover_1 = object_model.get_part("organizer_cover_1")
    handle = object_model.get_part("handle")
    latch_0 = object_model.get_part("latch_0")
    latch_1 = object_model.get_part("latch_1")
    handle_slide = object_model.get_articulation("body_to_handle")
    cover_hinge_0 = object_model.get_articulation("lid_to_organizer_cover_0")
    cover_hinge_1 = object_model.get_articulation("lid_to_organizer_cover_1")
    latch_joint_0 = object_model.get_articulation("body_to_latch_0")
    latch_joint_1 = object_model.get_articulation("body_to_latch_1")

    ctx.allow_overlap(
        body,
        handle,
        elem_a="left_handle_sleeve",
        elem_b="left_rail",
        reason="The telescoping inner rail is intentionally represented inside the molded sleeve proxy.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="right_handle_sleeve",
        elem_b="right_rail",
        reason="The telescoping inner rail is intentionally represented inside the molded sleeve proxy.",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_front_skirt",
        negative_elem="front_wall",
        min_gap=-0.0005,
        max_gap=0.0015,
        name="lid seats on the lower shell seam",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.34,
        name="lid footprint covers the body opening",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="left_rail",
        outer_elem="left_handle_sleeve",
        margin=0.016,
        name="left handle rail stays centered in its sleeve",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="right_rail",
        outer_elem="right_handle_sleeve",
        margin=0.016,
        name="right handle rail stays centered in its sleeve",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="left_rail",
        elem_b="left_handle_sleeve",
        min_overlap=0.14,
        name="collapsed handle keeps left rail retained in the sleeve",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="right_rail",
        elem_b="right_handle_sleeve",
        min_overlap=0.14,
        name="collapsed handle keeps right rail retained in the sleeve",
    )

    rest_handle = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.290}):
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="left_rail",
            outer_elem="left_handle_sleeve",
            margin=0.016,
            name="extended left rail stays inside the sleeve footprint",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="right_rail",
            outer_elem="right_handle_sleeve",
            margin=0.016,
            name="extended right rail stays inside the sleeve footprint",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="left_rail",
            elem_b="left_handle_sleeve",
            min_overlap=0.035,
            name="extended left rail keeps retained insertion",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="right_rail",
            elem_b="right_handle_sleeve",
            min_overlap=0.035,
            name="extended right rail keeps retained insertion",
        )
        extended_handle = ctx.part_world_position(handle)
    ctx.check(
        "handle extends upward from the rear sleeves",
        rest_handle is not None
        and extended_handle is not None
        and extended_handle[2] > rest_handle[2] + 0.20,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    ctx.expect_overlap(
        latch_0,
        body,
        axes="z",
        min_overlap=0.06,
        name="left latch overlaps the lower shell height",
    )
    ctx.expect_overlap(
        latch_0,
        lid,
        axes="z",
        min_overlap=0.03,
        name="left latch bridges onto the lid shell",
    )
    ctx.expect_overlap(
        latch_1,
        body,
        axes="z",
        min_overlap=0.06,
        name="right latch overlaps the lower shell height",
    )
    ctx.expect_overlap(
        latch_1,
        lid,
        axes="z",
        min_overlap=0.03,
        name="right latch bridges onto the lid shell",
    )

    with ctx.pose({cover_hinge_0: 1.10, cover_hinge_1: 1.10}):
        left_cover_aabb = ctx.part_element_world_aabb(cover_0, elem="cover_plate")
        right_cover_aabb = ctx.part_element_world_aabb(cover_1, elem="cover_plate")
    closed_left_aabb = ctx.part_element_world_aabb(cover_0, elem="cover_plate")
    closed_right_aabb = ctx.part_element_world_aabb(cover_1, elem="cover_plate")
    ctx.check(
        "organizer cover 0 opens upward on its rear hinge",
        closed_left_aabb is not None
        and left_cover_aabb is not None
        and left_cover_aabb[1][2] > closed_left_aabb[1][2] + 0.08,
        details=f"closed={closed_left_aabb}, open={left_cover_aabb}",
    )
    ctx.check(
        "organizer cover 1 opens upward on its rear hinge",
        closed_right_aabb is not None
        and right_cover_aabb is not None
        and right_cover_aabb[1][2] > closed_right_aabb[1][2] + 0.08,
        details=f"closed={closed_right_aabb}, open={right_cover_aabb}",
    )

    with ctx.pose({latch_joint_0: 0.95, latch_joint_1: 0.95}):
        open_latch_0 = ctx.part_world_aabb(latch_0)
        open_latch_1 = ctx.part_world_aabb(latch_1)
    closed_latch_0 = ctx.part_world_aabb(latch_0)
    closed_latch_1 = ctx.part_world_aabb(latch_1)
    ctx.check(
        "left latch swings outward from the shell side",
        closed_latch_0 is not None
        and open_latch_0 is not None
        and open_latch_0[0][0] < closed_latch_0[0][0] - 0.04,
        details=f"closed={closed_latch_0}, open={open_latch_0}",
    )
    ctx.check(
        "right latch swings outward from the shell side",
        closed_latch_1 is not None
        and open_latch_1 is not None
        and open_latch_1[1][0] > closed_latch_1[1][0] + 0.04,
        details=f"closed={closed_latch_1}, open={open_latch_1}",
    )

    return ctx.report()


object_model = build_object_model()
