from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _add_rect_ring(
    part,
    *,
    prefix: str,
    outer_w: float,
    outer_h: float,
    inner_w: float,
    inner_h: float,
    depth: float,
    y_center: float,
    material,
    z_center: float = 0.0,
) -> None:
    side_w = (outer_w - inner_w) * 0.5
    rail_h = (outer_h - inner_h) * 0.5
    x_side = (inner_w * 0.5) + (side_w * 0.5)
    z_rail = z_center + (inner_h * 0.5) + (rail_h * 0.5)

    part.visual(
        Box((side_w, depth, outer_h)),
        origin=Origin(xyz=(-x_side, y_center, z_center)),
        material=material,
        name=f"{prefix}_left",
    )
    part.visual(
        Box((side_w, depth, outer_h)),
        origin=Origin(xyz=(x_side, y_center, z_center)),
        material=material,
        name=f"{prefix}_right",
    )
    part.visual(
        Box((inner_w, depth, rail_h)),
        origin=Origin(xyz=(0.0, y_center, z_rail)),
        material=material,
        name=f"{prefix}_top",
    )
    part.visual(
        Box((inner_w, depth, rail_h)),
        origin=Origin(xyz=(0.0, y_center, -z_rail)),
        material=material,
        name=f"{prefix}_bottom",
    )


def _aabb_max_y(aabb) -> float | None:
    if aabb is None:
        return None
    return aabb[1][1]


def _aabb_min_z(aabb) -> float | None:
    if aabb is None:
        return None
    return aabb[0][2]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_panel_board")

    wall_paint = model.material("wall_paint", rgba=(0.93, 0.93, 0.91, 1.0))
    steel_frame = model.material("steel_frame", rgba=(0.45, 0.48, 0.51, 1.0))
    galvanized = model.material("galvanized", rgba=(0.69, 0.71, 0.72, 1.0))
    door_finish = model.material("door_finish", rgba=(0.88, 0.89, 0.90, 1.0))
    breaker_black = model.material("breaker_black", rgba=(0.12, 0.12, 0.13, 1.0))
    breaker_gray = model.material("breaker_gray", rgba=(0.65, 0.67, 0.69, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    label_white = model.material("label_white", rgba=(0.96, 0.96, 0.95, 1.0))

    wall_open_w = 0.334
    wall_open_h = 0.574
    frame_open_w = 0.320
    frame_open_h = 0.560
    door_w = 0.311
    door_h = 0.554
    door_axis_x = -(frame_open_w * 0.5) + 0.008

    wall_section = model.part("wall_section")
    wall_section.inertial = Inertial.from_geometry(
        Box((0.720, 0.120, 0.880)),
        mass=42.0,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
    )
    _add_rect_ring(
        wall_section,
        prefix="wall",
        outer_w=0.720,
        outer_h=0.880,
        inner_w=wall_open_w,
        inner_h=wall_open_h,
        depth=0.120,
        y_center=-0.060,
        material=wall_paint,
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.380, 0.034, 0.620)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.013, 0.0)),
    )
    _add_rect_ring(
        frame,
        prefix="front_trim",
        outer_w=0.380,
        outer_h=0.620,
        inner_w=frame_open_w,
        inner_h=frame_open_h,
        depth=0.004,
        y_center=0.002,
        material=steel_frame,
    )
    _add_rect_ring(
        frame,
        prefix="jamb",
        outer_w=wall_open_w,
        outer_h=wall_open_h,
        inner_w=frame_open_w,
        inner_h=frame_open_h,
        depth=0.030,
        y_center=-0.015,
        material=steel_frame,
    )
    frame.visual(
        Box((0.015, 0.008, 0.090)),
        origin=Origin(xyz=(door_axis_x - 0.005, 0.000, 0.190)),
        material=steel_frame,
        name="upper_hinge_leaf",
    )
    frame.visual(
        Box((0.015, 0.008, 0.090)),
        origin=Origin(xyz=(door_axis_x - 0.005, 0.000, -0.190)),
        material=steel_frame,
        name="lower_hinge_leaf",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.075),
        origin=Origin(xyz=(door_axis_x - 0.008, 0.004, 0.190)),
        material=steel_frame,
        name="upper_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.075),
        origin=Origin(xyz=(door_axis_x - 0.008, 0.004, -0.190)),
        material=steel_frame,
        name="lower_hinge_barrel",
    )
    frame.visual(
        Box((0.286, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.036, 0.187)),
        material=steel_frame,
        name="inner_cover_hinge_plate",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.252),
        origin=Origin(xyz=(0.0, -0.032, 0.188), rpy=(0.0, pi * 0.5, 0.0)),
        material=steel_frame,
        name="inner_cover_barrel",
    )
    frame.visual(
        Box((0.012, 0.010, 0.104)),
        origin=Origin(xyz=(-0.138, -0.031, 0.235)),
        material=steel_frame,
        name="inner_cover_left_bracket",
    )
    frame.visual(
        Box((0.012, 0.010, 0.104)),
        origin=Origin(xyz=(0.138, -0.031, 0.235)),
        material=steel_frame,
        name="inner_cover_right_bracket",
    )

    wall_box = model.part("wall_box")
    wall_box.inertial = Inertial.from_geometry(
        Box((0.320, 0.090, 0.560)),
        mass=11.5,
        origin=Origin(xyz=(0.0, -0.075, 0.0)),
    )
    _add_rect_ring(
        wall_box,
        prefix="front_lip",
        outer_w=0.307,
        outer_h=0.547,
        inner_w=0.300,
        inner_h=0.540,
        depth=0.003,
        y_center=-0.032,
        material=galvanized,
    )
    wall_box.visual(
        Box((0.300, 0.003, 0.540)),
        origin=Origin(xyz=(0.0, -0.1185, 0.0)),
        material=galvanized,
        name="back_plate",
    )
    wall_box.visual(
        Box((0.0035, 0.0845, 0.540)),
        origin=Origin(xyz=(-0.14825, -0.07575, 0.0)),
        material=galvanized,
        name="left_wall",
    )
    wall_box.visual(
        Box((0.0035, 0.0845, 0.540)),
        origin=Origin(xyz=(0.14825, -0.07575, 0.0)),
        material=galvanized,
        name="right_wall",
    )
    wall_box.visual(
        Box((0.293, 0.0845, 0.0035)),
        origin=Origin(xyz=(0.0, -0.07575, 0.26825)),
        material=galvanized,
        name="top_wall",
    )
    wall_box.visual(
        Box((0.293, 0.0845, 0.0035)),
        origin=Origin(xyz=(0.0, -0.07575, -0.26825)),
        material=galvanized,
        name="bottom_wall",
    )
    wall_box.visual(
        Box((0.210, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.102, 0.120)),
        material=steel_frame,
        name="din_rail",
    )
    wall_box.visual(
        Box((0.022, 0.009, 0.030)),
        origin=Origin(xyz=(-0.080, -0.1125, 0.120)),
        material=steel_frame,
        name="din_rail_left_standoff",
    )
    wall_box.visual(
        Box((0.022, 0.009, 0.030)),
        origin=Origin(xyz=(0.080, -0.1125, 0.120)),
        material=steel_frame,
        name="din_rail_right_standoff",
    )

    breaker_row = model.part("breaker_row")
    breaker_row.inertial = Inertial.from_geometry(
        Box((0.284, 0.020, 0.110)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.010, 0.055)),
    )
    breaker_row.visual(
        Box((0.284, 0.012, 0.108)),
        origin=Origin(xyz=(0.0, 0.036, 0.054)),
        material=breaker_gray,
        name="breaker_base",
    )
    breaker_row.visual(
        Box((0.236, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.024, 0.085)),
        material=breaker_gray,
        name="rail_clip_bar",
    )
    for rib_index in range(4):
        rib_x = -0.090 + rib_index * 0.060
        breaker_row.visual(
            Box((0.016, 0.010, 0.050)),
            origin=Origin(xyz=(rib_x, 0.029, 0.064)),
            material=breaker_gray,
            name=f"clip_rib_{rib_index}",
        )
    breaker_row.visual(
        Box((0.240, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.044, 0.094)),
        material=label_white,
        name="label_strip",
    )
    for index in range(8):
        x_center = -0.1225 + index * 0.035
        breaker_row.visual(
            Box((0.026, 0.012, 0.060)),
            origin=Origin(xyz=(x_center, 0.048, 0.055)),
            material=breaker_black,
            name=f"toggle_{index}",
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.012, door_h)),
        mass=6.3,
        origin=Origin(xyz=(door_w * 0.5, -0.004, 0.0)),
    )
    door.visual(
        Box((door_w, 0.0016, door_h)),
        origin=Origin(xyz=(door_w * 0.5, -0.0008, 0.0)),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        Box((door_w, 0.0092, 0.016)),
        origin=Origin(xyz=(door_w * 0.5, -0.0062, 0.269)),
        material=door_finish,
        name="door_top_return",
    )
    door.visual(
        Box((door_w, 0.0092, 0.016)),
        origin=Origin(xyz=(door_w * 0.5, -0.0062, -0.269)),
        material=door_finish,
        name="door_bottom_return",
    )
    door.visual(
        Box((0.016, 0.0092, 0.522)),
        origin=Origin(xyz=(door_w - 0.008, -0.0062, 0.0)),
        material=door_finish,
        name="door_latch_return",
    )
    door.visual(
        Box((0.014, 0.008, 0.090)),
        origin=Origin(xyz=(0.007, 0.000, 0.190)),
        material=door_finish,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.014, 0.008, 0.090)),
        origin=Origin(xyz=(0.007, 0.000, -0.190)),
        material=door_finish,
        name="lower_hinge_leaf",
    )
    door.visual(
        Box((0.016, 0.010, 0.110)),
        origin=Origin(xyz=(door_w - 0.020, 0.006, 0.0)),
        material=latch_dark,
        name="door_handle",
    )
    door.visual(
        Box((0.005, 0.018, 0.095)),
        origin=Origin(xyz=(door_w - 0.028, -0.002, 0.0)),
        material=latch_dark,
        name="door_pull",
    )

    inner_cover = model.part("inner_cover")
    inner_cover.inertial = Inertial.from_geometry(
        Box((0.272, 0.010, 0.180)),
        mass=2.1,
        origin=Origin(xyz=(0.0, -0.004, -0.090)),
    )
    inner_cover.visual(
        Box((0.272, 0.0012, 0.180)),
        origin=Origin(xyz=(0.0, -0.0006, -0.090)),
        material=door_finish,
        name="cover_panel",
    )
    inner_cover.visual(
        Box((0.272, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, -0.005)),
        material=door_finish,
        name="cover_top_fold",
    )
    inner_cover.visual(
        Box((0.272, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, -0.174)),
        material=door_finish,
        name="cover_bottom_return",
    )
    inner_cover.visual(
        Box((0.012, 0.008, 0.164)),
        origin=Origin(xyz=(-0.130, -0.004, -0.092)),
        material=door_finish,
        name="cover_left_return",
    )
    inner_cover.visual(
        Box((0.012, 0.008, 0.164)),
        origin=Origin(xyz=(0.130, -0.004, -0.092)),
        material=door_finish,
        name="cover_right_return",
    )
    inner_cover.visual(
        Box((0.046, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.003, -0.166)),
        material=latch_dark,
        name="cover_pull",
    )

    model.articulation(
        "wall_to_frame",
        ArticulationType.FIXED,
        parent=wall_section,
        child=frame,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_wall_box",
        ArticulationType.FIXED,
        parent=frame,
        child=wall_box,
        origin=Origin(),
    )
    model.articulation(
        "wall_box_to_breaker_row",
        ArticulationType.FIXED,
        parent=wall_box,
        child=breaker_row,
        origin=Origin(xyz=(0.0, -0.117, 0.035)),
    )
    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(door_axis_x, 0.004, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.95),
    )
    model.articulation(
        "frame_to_inner_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=inner_cover,
        origin=Origin(xyz=(0.0, -0.036, 0.182)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_section = object_model.get_part("wall_section")
    frame = object_model.get_part("frame")
    wall_box = object_model.get_part("wall_box")
    breaker_row = object_model.get_part("breaker_row")
    door = object_model.get_part("door")
    inner_cover = object_model.get_part("inner_cover")
    door_hinge = object_model.get_articulation("frame_to_door")
    cover_hinge = object_model.get_articulation("frame_to_inner_cover")

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

    ctx.expect_contact(frame, wall_section, name="frame is mounted in the wall opening")
    ctx.expect_contact(wall_box, frame, name="wall box seats against the recessed frame")
    ctx.expect_contact(breaker_row, wall_box, name="breaker row is mounted to the back of the wall box")
    ctx.expect_contact(door, frame, name="main door is supported by the left-side hinge leaves")
    ctx.expect_contact(inner_cover, frame, name="inner cover is supported by the top hinge bracket")
    ctx.expect_gap(
        inner_cover,
        breaker_row,
        axis="y",
        min_gap=0.035,
        max_gap=0.085,
        positive_elem="cover_panel",
        negative_elem="breaker_base",
        name="closed inner cover stands in front of the breaker row",
    )

    closed_door = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.75}):
        opened_door = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "main door swings outward from the left hinge",
        _aabb_max_y(closed_door) is not None
        and _aabb_max_y(opened_door) is not None
        and _aabb_max_y(opened_door) > _aabb_max_y(closed_door) + 0.20,
        details=f"closed={closed_door}, opened={opened_door}",
    )

    closed_cover = ctx.part_element_world_aabb(inner_cover, elem="cover_panel")
    with ctx.pose({door_hinge: 1.75, cover_hinge: 1.15}):
        opened_cover = ctx.part_element_world_aabb(inner_cover, elem="cover_panel")
    ctx.check(
        "inner cover lifts upward and outward from the top hinge",
        _aabb_max_y(closed_cover) is not None
        and _aabb_max_y(opened_cover) is not None
        and _aabb_min_z(closed_cover) is not None
        and _aabb_min_z(opened_cover) is not None
        and _aabb_max_y(opened_cover) > _aabb_max_y(closed_cover) + 0.08
        and _aabb_min_z(opened_cover) > _aabb_min_z(closed_cover) + 0.06,
        details=f"closed={closed_cover}, opened={opened_cover}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
