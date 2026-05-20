from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(part, size, xyz, material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_drawer(
    part,
    *,
    front_w: float,
    front_h: float,
    front_t: float,
    box_w: float,
    box_h: float,
    box_d: float,
    side_t: float,
    bottom_t: float,
    back_t: float,
    box_center_z: float,
    runner_w: float,
    runner_h: float,
    runner_len: float,
    runner_front_setback: float,
    runner_z: float,
    handle_w: float,
    handle_h: float,
    handle_t: float,
    handle_z: float,
    front_material,
    case_material,
    hardware_material,
) -> None:
    _add_box(
        part,
        (front_w, front_t, front_h),
        (0.0, front_t / 2.0, 0.0),
        front_material,
        "front",
    )

    side_y = -box_d / 2.0
    side_x = box_w / 2.0 - side_t / 2.0
    _add_box(
        part,
        (side_t, box_d, box_h),
        (-side_x, side_y, box_center_z),
        case_material,
        "side_0",
    )
    _add_box(
        part,
        (side_t, box_d, box_h),
        (side_x, side_y, box_center_z),
        case_material,
        "side_1",
    )

    bottom_w = box_w - 2.0 * side_t
    bottom_d = box_d - back_t
    bottom_z = box_center_z - box_h / 2.0 + bottom_t / 2.0
    _add_box(
        part,
        (bottom_w, bottom_d, bottom_t),
        (0.0, -bottom_d / 2.0, bottom_z),
        case_material,
        "bottom",
    )

    _add_box(
        part,
        (bottom_w, back_t, box_h),
        (0.0, -box_d + back_t / 2.0, box_center_z),
        case_material,
        "back",
    )

    runner_y = -(runner_front_setback + runner_len / 2.0)
    runner_x = box_w / 2.0 + runner_w / 2.0
    _add_box(
        part,
        (runner_w, runner_len, runner_h),
        (-runner_x, runner_y, runner_z),
        hardware_material,
        "runner_0",
    )
    _add_box(
        part,
        (runner_w, runner_len, runner_h),
        (runner_x, runner_y, runner_z),
        hardware_material,
        "runner_1",
    )

    _add_box(
        part,
        (handle_w, handle_t, handle_h),
        (0.0, front_t + handle_t / 2.0, handle_z),
        hardware_material,
        "pull",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="media_drawer_cabinet")

    wood = model.material("wood", rgba=(0.56, 0.40, 0.26, 1.0))
    drawer_front = model.material("drawer_front", rgba=(0.50, 0.35, 0.22, 1.0))
    drawer_box = model.material("drawer_box", rgba=(0.84, 0.79, 0.71, 1.0))
    plinth = model.material("plinth", rgba=(0.16, 0.13, 0.11, 1.0))
    metal = model.material("metal", rgba=(0.20, 0.21, 0.22, 1.0))

    width = 1.50
    depth = 0.46
    height = 0.56
    panel_t = 0.018
    back_t = 0.006
    front_t = 0.018
    plinth_h = 0.050

    side_h = height - plinth_h - panel_t
    top_underside_z = height - panel_t
    bottom_top_z = plinth_h + panel_t
    inner_w = width - 2.0 * panel_t
    inner_depth = depth - back_t

    shelf_clear_h = 0.122
    shelf_top_z = top_underside_z - shelf_clear_h
    shelf_center_z = shelf_top_z - panel_t / 2.0
    shelf_bottom_z = shelf_top_z - panel_t

    lower_open_h = shelf_bottom_z - bottom_top_z
    lower_open_w = (inner_w - panel_t) / 2.0
    lower_front_w = lower_open_w - 0.008
    lower_front_h = lower_open_h - 0.008
    lower_front_center_z = bottom_top_z + 0.004 + lower_front_h / 2.0
    lower_drawer_x = lower_open_w / 2.0 + panel_t / 2.0

    upper_open_w = 0.460
    upper_front_w = upper_open_w - 0.008
    upper_front_h = shelf_clear_h - 0.008
    upper_front_center_z = shelf_top_z + 0.004 + upper_front_h / 2.0

    runner_body_w = 0.010
    runner_drawer_w = 0.006
    runner_h = 0.028
    runner_gap = 0.0
    runner_front_setback = 0.045

    lower_track_w = lower_open_w - 2.0 * runner_body_w
    lower_box_w = lower_track_w - 2.0 * (runner_drawer_w + runner_gap)
    lower_box_h = 0.190
    lower_box_d = 0.390
    lower_box_center_z = -0.052
    lower_runner_z_world = lower_front_center_z - 0.010
    lower_runner_z_local = -0.010
    lower_runner_len_body = 0.330
    lower_runner_len_drawer = 0.290

    upper_track_w = upper_open_w - 2.0 * runner_body_w
    upper_box_w = upper_track_w - 2.0 * (runner_drawer_w + runner_gap)
    upper_box_h = 0.065
    upper_box_d = 0.310
    upper_box_center_z = -0.017
    upper_runner_z_world = upper_front_center_z - 0.010
    upper_runner_z_local = -0.010
    upper_runner_len_body = 0.270
    upper_runner_len_drawer = 0.240

    cabinet = model.part("cabinet")
    _add_box(
        cabinet,
        (width, depth, panel_t),
        (0.0, 0.0, height - panel_t / 2.0),
        wood,
        "top",
    )
    _add_box(
        cabinet,
        (panel_t, depth, side_h),
        (-(width / 2.0 - panel_t / 2.0), 0.0, plinth_h + side_h / 2.0),
        wood,
        "side_0",
    )
    _add_box(
        cabinet,
        (panel_t, depth, side_h),
        (width / 2.0 - panel_t / 2.0, 0.0, plinth_h + side_h / 2.0),
        wood,
        "side_1",
    )
    _add_box(
        cabinet,
        (inner_w, inner_depth, panel_t),
        (0.0, back_t / 2.0, plinth_h + panel_t / 2.0),
        wood,
        "bottom",
    )
    _add_box(
        cabinet,
        (inner_w, back_t, top_underside_z - bottom_top_z),
        (0.0, -(depth / 2.0 - back_t / 2.0), (top_underside_z + bottom_top_z) / 2.0),
        wood,
        "back",
    )
    _add_box(
        cabinet,
        (inner_w, inner_depth, panel_t),
        (0.0, back_t / 2.0, shelf_center_z),
        wood,
        "shelf",
    )

    divider_h = shelf_bottom_z - bottom_top_z
    _add_box(
        cabinet,
        (panel_t, inner_depth, divider_h),
        (0.0, back_t / 2.0, bottom_top_z + divider_h / 2.0),
        wood,
        "divider",
    )

    upper_cheek_h = top_underside_z - shelf_top_z
    cheek_x = upper_open_w / 2.0 + panel_t / 2.0
    _add_box(
        cabinet,
        (panel_t, inner_depth, upper_cheek_h),
        (-cheek_x, back_t / 2.0, shelf_top_z + upper_cheek_h / 2.0),
        wood,
        "cheek_0",
    )
    _add_box(
        cabinet,
        (panel_t, inner_depth, upper_cheek_h),
        (cheek_x, back_t / 2.0, shelf_top_z + upper_cheek_h / 2.0),
        wood,
        "cheek_1",
    )

    filler_w = inner_w / 2.0 - upper_open_w / 2.0 - panel_t
    filler_x = upper_open_w / 2.0 + panel_t + filler_w / 2.0
    filler_z = upper_front_center_z
    _add_box(
        cabinet,
        (filler_w, front_t, upper_front_h),
        (-filler_x, depth / 2.0 + front_t / 2.0, filler_z),
        drawer_front,
        "filler_0",
    )
    _add_box(
        cabinet,
        (filler_w, front_t, upper_front_h),
        (filler_x, depth / 2.0 + front_t / 2.0, filler_z),
        drawer_front,
        "filler_1",
    )

    _add_box(
        cabinet,
        (width - 0.100, depth - 0.070, plinth_h),
        (0.0, -0.010, plinth_h / 2.0),
        plinth,
        "plinth",
    )

    lower_runner_y = depth / 2.0 - runner_front_setback - lower_runner_len_body / 2.0
    upper_runner_y = depth / 2.0 - runner_front_setback - upper_runner_len_body / 2.0

    left_outer_runner_x = -(inner_w / 2.0 - runner_body_w / 2.0)
    left_inner_runner_x = -(panel_t / 2.0 + runner_body_w / 2.0)
    right_inner_runner_x = panel_t / 2.0 + runner_body_w / 2.0
    right_outer_runner_x = inner_w / 2.0 - runner_body_w / 2.0
    upper_runner_x = upper_open_w / 2.0 - runner_body_w / 2.0

    _add_box(
        cabinet,
        (runner_body_w, lower_runner_len_body, runner_h),
        (left_outer_runner_x, lower_runner_y, lower_runner_z_world),
        metal,
        "left_runner_0",
    )
    _add_box(
        cabinet,
        (runner_body_w, lower_runner_len_body, runner_h),
        (left_inner_runner_x, lower_runner_y, lower_runner_z_world),
        metal,
        "left_runner_1",
    )
    _add_box(
        cabinet,
        (runner_body_w, lower_runner_len_body, runner_h),
        (right_inner_runner_x, lower_runner_y, lower_runner_z_world),
        metal,
        "right_runner_0",
    )
    _add_box(
        cabinet,
        (runner_body_w, lower_runner_len_body, runner_h),
        (right_outer_runner_x, lower_runner_y, lower_runner_z_world),
        metal,
        "right_runner_1",
    )
    _add_box(
        cabinet,
        (runner_body_w, upper_runner_len_body, runner_h),
        (-upper_runner_x, upper_runner_y, upper_runner_z_world),
        metal,
        "upper_runner_0",
    )
    _add_box(
        cabinet,
        (runner_body_w, upper_runner_len_body, runner_h),
        (upper_runner_x, upper_runner_y, upper_runner_z_world),
        metal,
        "upper_runner_1",
    )

    left_drawer = model.part("left_drawer")
    _add_drawer(
        left_drawer,
        front_w=lower_front_w,
        front_h=lower_front_h,
        front_t=front_t,
        box_w=lower_box_w,
        box_h=lower_box_h,
        box_d=lower_box_d,
        side_t=0.014,
        bottom_t=0.012,
        back_t=0.014,
        box_center_z=lower_box_center_z,
        runner_w=runner_drawer_w,
        runner_h=runner_h,
        runner_len=lower_runner_len_drawer,
        runner_front_setback=runner_front_setback,
        runner_z=lower_runner_z_local,
        handle_w=0.200,
        handle_h=0.014,
        handle_t=0.012,
        handle_z=lower_front_h / 2.0 - 0.052,
        front_material=drawer_front,
        case_material=drawer_box,
        hardware_material=metal,
    )

    right_drawer = model.part("right_drawer")
    _add_drawer(
        right_drawer,
        front_w=lower_front_w,
        front_h=lower_front_h,
        front_t=front_t,
        box_w=lower_box_w,
        box_h=lower_box_h,
        box_d=lower_box_d,
        side_t=0.014,
        bottom_t=0.012,
        back_t=0.014,
        box_center_z=lower_box_center_z,
        runner_w=runner_drawer_w,
        runner_h=runner_h,
        runner_len=lower_runner_len_drawer,
        runner_front_setback=runner_front_setback,
        runner_z=lower_runner_z_local,
        handle_w=0.200,
        handle_h=0.014,
        handle_t=0.012,
        handle_z=lower_front_h / 2.0 - 0.052,
        front_material=drawer_front,
        case_material=drawer_box,
        hardware_material=metal,
    )

    accessory_drawer = model.part("accessory_drawer")
    _add_drawer(
        accessory_drawer,
        front_w=upper_front_w,
        front_h=upper_front_h,
        front_t=front_t,
        box_w=upper_box_w,
        box_h=upper_box_h,
        box_d=upper_box_d,
        side_t=0.012,
        bottom_t=0.010,
        back_t=0.012,
        box_center_z=upper_box_center_z,
        runner_w=runner_drawer_w,
        runner_h=runner_h,
        runner_len=upper_runner_len_drawer,
        runner_front_setback=runner_front_setback,
        runner_z=upper_runner_z_local,
        handle_w=0.140,
        handle_h=0.012,
        handle_t=0.012,
        handle_z=upper_front_h / 2.0 - 0.030,
        front_material=drawer_front,
        case_material=drawer_box,
        hardware_material=metal,
    )

    model.articulation(
        "cabinet_to_left_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_drawer,
        origin=Origin(xyz=(-lower_drawer_x, depth / 2.0, lower_front_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.240, effort=90.0, velocity=0.35),
    )
    model.articulation(
        "cabinet_to_right_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_drawer,
        origin=Origin(xyz=(lower_drawer_x, depth / 2.0, lower_front_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.240, effort=90.0, velocity=0.35),
    )
    model.articulation(
        "cabinet_to_accessory_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=accessory_drawer,
        origin=Origin(xyz=(0.0, depth / 2.0, upper_front_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.180, effort=50.0, velocity=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    left_drawer = object_model.get_part("left_drawer")
    right_drawer = object_model.get_part("right_drawer")
    accessory_drawer = object_model.get_part("accessory_drawer")

    left_joint = object_model.get_articulation("cabinet_to_left_drawer")
    right_joint = object_model.get_articulation("cabinet_to_right_drawer")
    accessory_joint = object_model.get_articulation("cabinet_to_accessory_drawer")

    ctx.expect_origin_gap(
        accessory_drawer,
        left_drawer,
        axis="z",
        min_gap=0.20,
        name="accessory drawer sits above the main drawers",
    )

    for drawer, check_name in (
        (left_drawer, "left drawer stays laterally within the cabinet"),
        (right_drawer, "right drawer stays laterally within the cabinet"),
        (accessory_drawer, "accessory drawer stays laterally within the cabinet"),
    ):
        ctx.expect_within(drawer, cabinet, axes="xz", margin=0.03, name=check_name)

    left_rest = ctx.part_world_position(left_drawer)
    with ctx.pose({left_joint: left_joint.motion_limits.upper}):
        ctx.expect_overlap(
            left_drawer,
            cabinet,
            axes="y",
            min_overlap=0.14,
            name="left drawer remains captured by its runners",
        )
        left_extended = ctx.part_world_position(left_drawer)

    right_rest = ctx.part_world_position(right_drawer)
    with ctx.pose({right_joint: right_joint.motion_limits.upper}):
        ctx.expect_overlap(
            right_drawer,
            cabinet,
            axes="y",
            min_overlap=0.14,
            name="right drawer remains captured by its runners",
        )
        right_extended = ctx.part_world_position(right_drawer)

    accessory_rest = ctx.part_world_position(accessory_drawer)
    with ctx.pose({accessory_joint: accessory_joint.motion_limits.upper}):
        ctx.expect_overlap(
            accessory_drawer,
            cabinet,
            axes="y",
            min_overlap=0.10,
            name="accessory drawer remains captured by its runners",
        )
        accessory_extended = ctx.part_world_position(accessory_drawer)

    ctx.check(
        "left drawer extends forward",
        left_rest is not None
        and left_extended is not None
        and left_extended[1] > left_rest[1] + 0.20,
        details=f"rest={left_rest}, extended={left_extended}",
    )
    ctx.check(
        "right drawer extends forward",
        right_rest is not None
        and right_extended is not None
        and right_extended[1] > right_rest[1] + 0.20,
        details=f"rest={right_rest}, extended={right_extended}",
    )
    ctx.check(
        "accessory drawer extends forward",
        accessory_rest is not None
        and accessory_extended is not None
        and accessory_extended[1] > accessory_rest[1] + 0.15,
        details=f"rest={accessory_rest}, extended={accessory_extended}",
    )

    return ctx.report()


object_model = build_object_model()
