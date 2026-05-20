from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 12,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos((2.0 * pi * index) / segments),
            cy + radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _pegboard_mesh(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
    columns: int,
    rows: int,
) -> object:
    outer = _rect_profile(width, height)
    pitch_x = width / (columns + 1)
    pitch_y = height / (rows + 1)
    hole_radius = min(pitch_x, pitch_y) * 0.18
    hole_profiles = []
    for col in range(columns):
        x = -width * 0.5 + pitch_x * (col + 1)
        for row in range(rows):
            y = -height * 0.5 + pitch_y * (row + 1)
            hole_profiles.append(_circle_profile(hole_radius, center=(x, y), segments=12))

    geometry = ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        height=thickness,
        center=True,
    ).rotate_x(pi * 0.5)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_storage_cabinet")

    cabinet_blue = model.material("cabinet_blue", rgba=(0.30, 0.38, 0.46, 1.0))
    door_blue = model.material("door_blue", rgba=(0.42, 0.50, 0.58, 1.0))
    interior_gray = model.material("interior_gray", rgba=(0.74, 0.77, 0.80, 1.0))
    pegboard_cream = model.material("pegboard_cream", rgba=(0.89, 0.87, 0.79, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.80, 0.81, 0.83, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.15, 0.16, 0.18, 1.0))

    cabinet_width = 0.92
    cabinet_depth = 0.46
    cabinet_height = 1.82
    side_thickness = 0.020
    back_thickness = 0.016
    frame_depth = 0.030
    door_thickness = 0.022
    door_height = 1.76
    door_bottom = 0.03
    seam_gap = 0.006
    hinge_axis_offset = 0.006
    door_panel_offset = 0.004
    door_width = (cabinet_width * 0.5 - hinge_axis_offset) - (seam_gap * 0.5) - door_panel_offset

    pegboard_width = 0.33
    pegboard_height = 1.34
    pegboard_thickness = 0.004
    standoff_length = 0.012
    standoff_radius = 0.008

    body = model.part("cabinet_body")
    body.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(-cabinet_width * 0.5 + side_thickness * 0.5, 0.0, cabinet_height * 0.5)
        ),
        material=cabinet_blue,
        name="left_side",
    )
    body.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(cabinet_width * 0.5 - side_thickness * 0.5, 0.0, cabinet_height * 0.5)
        ),
        material=cabinet_blue,
        name="right_side",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, cabinet_depth, side_thickness)),
        origin=Origin(xyz=(0.0, 0.0, side_thickness * 0.5)),
        material=cabinet_blue,
        name="bottom_panel",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, cabinet_depth, side_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - side_thickness * 0.5)),
        material=cabinet_blue,
        name="top_panel",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, back_thickness, cabinet_height - 2.0 * side_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth * 0.5 + back_thickness * 0.5,
                cabinet_height * 0.5,
            )
        ),
        material=interior_gray,
        name="back_panel",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, frame_depth, side_thickness)),
        origin=Origin(
            xyz=(0.0, cabinet_depth * 0.5 - frame_depth * 0.5, side_thickness * 0.5)
        ),
        material=cabinet_blue,
        name="front_sill",
    )
    body.visual(
        Box((cabinet_width - 2.0 * side_thickness, frame_depth, side_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth * 0.5 - frame_depth * 0.5,
                cabinet_height - side_thickness * 0.5,
            )
        ),
        material=cabinet_blue,
        name="front_header",
    )
    clear_height = cabinet_height - 2.0 * side_thickness
    body.visual(
        Box((side_thickness, frame_depth, clear_height)),
        origin=Origin(
            xyz=(
                -cabinet_width * 0.5 + side_thickness * 0.5,
                cabinet_depth * 0.5 - frame_depth * 0.5,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_blue,
        name="front_left_jamb",
    )
    body.visual(
        Box((side_thickness, frame_depth, clear_height)),
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - side_thickness * 0.5,
                cabinet_depth * 0.5 - frame_depth * 0.5,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_blue,
        name="front_right_jamb",
    )
    body.visual(
        Box((cabinet_width - 0.12, cabinet_depth - 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_hardware,
        name="base_plinth",
    )

    hinge_leaf_width = 0.010
    hinge_leaf_depth = 0.030
    hinge_leaf_height = door_height - 0.10
    hinge_barrel_radius = 0.006
    hinge_barrel_length = 0.16
    left_body_leaf_x = -cabinet_width * 0.5 + 0.001
    right_body_leaf_x = cabinet_width * 0.5 - 0.001
    body_hinge_y = cabinet_depth * 0.5 + 0.003
    barrel_z_positions = (door_bottom + 0.17, door_bottom + door_height * 0.5, door_bottom + door_height - 0.17)

    body.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(left_body_leaf_x, body_hinge_y - 0.010, door_bottom + door_height * 0.5)),
        material=hinge_metal,
        name="left_hinge_leaf",
    )
    body.visual(
        Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
        origin=Origin(xyz=(right_body_leaf_x, body_hinge_y - 0.010, door_bottom + door_height * 0.5)),
        material=hinge_metal,
        name="right_hinge_leaf",
    )
    for index, z_pos in enumerate(barrel_z_positions):
        body.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(xyz=(left_body_leaf_x - 0.006, body_hinge_y, z_pos)),
            material=hinge_metal,
            name=f"left_hinge_barrel_{index}",
        )
        body.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(xyz=(right_body_leaf_x + 0.006, body_hinge_y, z_pos)),
            material=hinge_metal,
            name=f"right_hinge_barrel_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    left_pegboard = model.part("left_pegboard_panel")
    left_pegboard.visual(
        _pegboard_mesh(
            "left_internal_pegboard",
            width=pegboard_width,
            height=pegboard_height,
            thickness=pegboard_thickness,
            columns=5,
            rows=17,
        ),
        origin=Origin(xyz=(0.0, 0.0, pegboard_height * 0.5)),
        material=pegboard_cream,
        name="panel_sheet",
    )
    for index, (x_pos, z_pos) in enumerate(
        (
            (-pegboard_width * 0.5 + 0.03, 0.06),
            (pegboard_width * 0.5 - 0.03, 0.06),
            (-pegboard_width * 0.5 + 0.03, pegboard_height - 0.06),
            (pegboard_width * 0.5 - 0.03, pegboard_height - 0.06),
        )
    ):
        left_pegboard.visual(
            Cylinder(radius=standoff_radius, length=standoff_length),
            origin=Origin(
                xyz=(x_pos, -pegboard_thickness * 0.5 - standoff_length * 0.5, z_pos),
                rpy=(-pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"standoff_{index}",
        )
    left_pegboard.inertial = Inertial.from_geometry(
        Box((pegboard_width, pegboard_thickness + standoff_length, pegboard_height)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.006, pegboard_height * 0.5)),
    )

    right_pegboard = model.part("right_pegboard_panel")
    right_pegboard.visual(
        _pegboard_mesh(
            "right_internal_pegboard",
            width=pegboard_width,
            height=pegboard_height,
            thickness=pegboard_thickness,
            columns=5,
            rows=17,
        ),
        origin=Origin(xyz=(0.0, 0.0, pegboard_height * 0.5)),
        material=pegboard_cream,
        name="panel_sheet",
    )
    for index, (x_pos, z_pos) in enumerate(
        (
            (-pegboard_width * 0.5 + 0.03, 0.06),
            (pegboard_width * 0.5 - 0.03, 0.06),
            (-pegboard_width * 0.5 + 0.03, pegboard_height - 0.06),
            (pegboard_width * 0.5 - 0.03, pegboard_height - 0.06),
        )
    ):
        right_pegboard.visual(
            Cylinder(radius=standoff_radius, length=standoff_length),
            origin=Origin(
                xyz=(x_pos, -pegboard_thickness * 0.5 - standoff_length * 0.5, z_pos),
                rpy=(-pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"standoff_{index}",
        )
    right_pegboard.inertial = Inertial.from_geometry(
        Box((pegboard_width, pegboard_thickness + standoff_length, pegboard_height)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.006, pegboard_height * 0.5)),
    )

    for door_name, side_sign, material in (
        ("left_door", 1.0, door_blue),
        ("right_door", -1.0, door_blue),
    ):
        door = model.part(door_name)
        panel_center_x = side_sign * (door_panel_offset + door_width * 0.5)
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(panel_center_x, 0.0, door_height * 0.5)),
            material=material,
            name="outer_skin",
        )
        frame_y = -0.004
        frame_depth_inner = 0.014
        frame_rail = 0.036
        door.visual(
            Box((door_width - 0.06, frame_depth_inner, frame_rail)),
            origin=Origin(xyz=(panel_center_x, frame_y, 0.05)),
            material=interior_gray,
            name="inner_bottom_rail",
        )
        door.visual(
            Box((door_width - 0.06, frame_depth_inner, frame_rail)),
            origin=Origin(xyz=(panel_center_x, frame_y, door_height - 0.05)),
            material=interior_gray,
            name="inner_top_rail",
        )
        door.visual(
            Box((frame_rail, frame_depth_inner, door_height - 0.10)),
            origin=Origin(
                xyz=(side_sign * (door_panel_offset + frame_rail * 0.5), frame_y, door_height * 0.5)
            ),
            material=interior_gray,
            name="inner_hinge_rail",
        )
        door.visual(
            Box((frame_rail, frame_depth_inner, door_height - 0.10)),
            origin=Origin(
                xyz=(
                    side_sign * (door_panel_offset + door_width - frame_rail * 0.5),
                    frame_y,
                    door_height * 0.5,
                )
            ),
            material=interior_gray,
            name="inner_meeting_rail",
        )
        door.visual(
            Box((0.012, 0.030, hinge_leaf_height)),
            origin=Origin(
                xyz=(side_sign * 0.010, 0.010, door_height * 0.5),
            ),
            material=hinge_metal,
            name="hinge_leaf",
        )
        for index, z_pos in enumerate((0.17, door_height * 0.5, door_height - 0.17)):
            door.visual(
                Cylinder(radius=0.007, length=hinge_barrel_length),
                origin=Origin(
                    xyz=(side_sign * 0.010, 0.019, z_pos),
                ),
                material=hinge_metal,
                name=f"hinge_knuckle_{index}",
            )
        door.inertial = Inertial.from_geometry(
            Box((door_width, door_thickness, door_height)),
            mass=14.0,
            origin=Origin(xyz=(panel_center_x, 0.0, door_height * 0.5)),
        )

    handle_plate_width = 0.060
    handle_plate_height = 0.190
    handle_plate_thickness = 0.010
    handle_bar_length = 0.135
    handle_pivot_height = door_bottom + 0.92
    handle_x_from_hinge = door_panel_offset + door_width - 0.070
    for handle_name in ("left_handle_latch", "right_handle_latch"):
        handle = model.part(handle_name)
        handle.visual(
            Box((handle_plate_width, handle_plate_thickness, handle_plate_height)),
            origin=Origin(
                xyz=(0.0, handle_plate_thickness * 0.5, 0.0),
            ),
            material=dark_hardware,
            name="escutcheon",
        )
        handle.visual(
            Cylinder(radius=0.013, length=0.028),
            origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(-pi * 0.5, 0.0, 0.0)),
            material=latch_metal,
            name="hub",
        )
        handle.visual(
            Box((0.024, 0.016, handle_bar_length)),
            origin=Origin(xyz=(0.0, 0.024, 0.0)),
            material=latch_metal,
            name="grip_bar",
        )
        handle.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(xyz=(0.0, 0.030, handle_bar_length * 0.5 - 0.010), rpy=(-pi * 0.5, 0.0, 0.0)),
            material=latch_metal,
            name="upper_grip",
        )
        handle.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(xyz=(0.0, 0.030, -handle_bar_length * 0.5 + 0.010), rpy=(-pi * 0.5, 0.0, 0.0)),
            material=latch_metal,
            name="lower_grip",
        )
        handle.inertial = Inertial.from_geometry(
            Box((handle_plate_width, 0.04, handle_plate_height)),
            mass=0.6,
            origin=Origin(xyz=(0.0, 0.020, 0.0)),
        )

    model.articulation(
        "body_to_left_pegboard",
        ArticulationType.FIXED,
        parent=body,
        child=left_pegboard,
        origin=Origin(
            xyz=(
                -0.20,
                -cabinet_depth * 0.5 + back_thickness + standoff_length + pegboard_thickness * 0.5,
                0.24,
            )
        ),
    )
    model.articulation(
        "body_to_right_pegboard",
        ArticulationType.FIXED,
        parent=body,
        child=right_pegboard,
        origin=Origin(
            xyz=(
                0.20,
                -cabinet_depth * 0.5 + back_thickness + standoff_length + pegboard_thickness * 0.5,
                0.24,
            )
        ),
    )
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child="left_door",
        origin=Origin(
            xyz=(
                -cabinet_width * 0.5 + hinge_axis_offset,
                cabinet_depth * 0.5 + door_thickness * 0.5,
                door_bottom,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=0.0,
            upper=2.25,
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child="right_door",
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - hinge_axis_offset,
                cabinet_depth * 0.5 + door_thickness * 0.5,
                door_bottom,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=0.0,
            upper=2.25,
        ),
    )
    model.articulation(
        "left_latch_pivot",
        ArticulationType.REVOLUTE,
        parent="left_door",
        child="left_handle_latch",
        origin=Origin(
            xyz=(
                handle_x_from_hinge,
                door_thickness * 0.5,
                handle_pivot_height - door_bottom,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-1.4,
            upper=1.4,
        ),
    )
    model.articulation(
        "right_latch_pivot",
        ArticulationType.REVOLUTE,
        parent="right_door",
        child="right_handle_latch",
        origin=Origin(
            xyz=(
                -handle_x_from_hinge,
                door_thickness * 0.5,
                handle_pivot_height - door_bottom,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-1.4,
            upper=1.4,
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

    body = object_model.get_part("cabinet_body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_handle = object_model.get_part("left_handle_latch")
    right_handle = object_model.get_part("right_handle_latch")
    left_pegboard = object_model.get_part("left_pegboard_panel")
    right_pegboard = object_model.get_part("right_pegboard_panel")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    left_latch = object_model.get_articulation("left_latch_pivot")
    right_latch = object_model.get_articulation("right_latch_pivot")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, left_latch: 0.0, right_latch: 0.0}):
        ctx.expect_gap(
            left_door,
            body,
            axis="y",
            max_gap=0.02,
            max_penetration=1e-6,
            positive_elem="outer_skin",
            negative_elem="front_left_jamb",
            name="left door closes at cabinet front",
        )
        ctx.expect_gap(
            right_door,
            body,
            axis="y",
            max_gap=0.02,
            max_penetration=1e-6,
            positive_elem="outer_skin",
            negative_elem="front_right_jamb",
            name="right door closes at cabinet front",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.003,
            max_gap=0.012,
            positive_elem="outer_skin",
            negative_elem="outer_skin",
            name="door meeting seam stays narrow",
        )
        ctx.expect_contact(
            left_handle,
            left_door,
            elem_a="escutcheon",
            elem_b="outer_skin",
            name="left latch escutcheon mounts flush to the left door",
        )
        ctx.expect_contact(
            right_handle,
            right_door,
            elem_a="escutcheon",
            elem_b="outer_skin",
            name="right latch escutcheon mounts flush to the right door",
        )
        ctx.expect_contact(
            left_pegboard,
            body,
            elem_b="back_panel",
            name="left pegboard is mounted off the cabinet back panel",
        )
        ctx.expect_contact(
            right_pegboard,
            body,
            elem_b="back_panel",
            name="right pegboard is mounted off the cabinet back panel",
        )
        ctx.expect_within(
            left_pegboard,
            body,
            axes="xz",
            margin=0.0,
            name="left pegboard stays inside cabinet envelope",
        )
        ctx.expect_within(
            right_pegboard,
            body,
            axes="xz",
            margin=0.0,
            name="right pegboard stays inside cabinet envelope",
        )

        left_closed = ctx.part_world_aabb(left_door)
        right_closed = ctx.part_world_aabb(right_door)
        left_bar_vertical = ctx.part_element_world_aabb(left_handle, elem="grip_bar")
        right_bar_vertical = ctx.part_element_world_aabb(right_handle, elem="grip_bar")

    with ctx.pose({left_hinge: 1.15, right_hinge: 1.15}):
        left_open = ctx.part_world_aabb(left_door)
        right_open = ctx.part_world_aabb(right_door)

    ctx.check(
        "left door opens outward on its side hinge",
        left_closed is not None
        and left_open is not None
        and left_open[1][1] > left_closed[1][1] + 0.14,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door opens outward on its side hinge",
        right_closed is not None
        and right_open is not None
        and right_open[1][1] > right_closed[1][1] + 0.14,
        details=f"closed={right_closed}, open={right_open}",
    )

    with ctx.pose({left_latch: 1.35, right_latch: 1.35}):
        left_bar_turned = ctx.part_element_world_aabb(left_handle, elem="grip_bar")
        right_bar_turned = ctx.part_element_world_aabb(right_handle, elem="grip_bar")

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    left_vertical_x = _span(left_bar_vertical, 0)
    left_vertical_z = _span(left_bar_vertical, 2)
    left_turned_x = _span(left_bar_turned, 0)
    left_turned_z = _span(left_bar_turned, 2)
    right_vertical_x = _span(right_bar_vertical, 0)
    right_vertical_z = _span(right_bar_vertical, 2)
    right_turned_x = _span(right_bar_turned, 0)
    right_turned_z = _span(right_bar_turned, 2)

    ctx.check(
        "left latch rotates from vertical parked position",
        left_vertical_x is not None
        and left_vertical_z is not None
        and left_turned_x is not None
        and left_turned_z is not None
        and left_vertical_z > left_vertical_x
        and left_turned_x > left_turned_z,
        details=(
            f"vertical_spans=({left_vertical_x}, {left_vertical_z}), "
            f"turned_spans=({left_turned_x}, {left_turned_z})"
        ),
    )
    ctx.check(
        "right latch rotates from vertical parked position",
        right_vertical_x is not None
        and right_vertical_z is not None
        and right_turned_x is not None
        and right_turned_z is not None
        and right_vertical_z > right_vertical_x
        and right_turned_x > right_turned_z,
        details=(
            f"vertical_spans=({right_vertical_x}, {right_vertical_z}), "
            f"turned_spans=({right_turned_x}, {right_turned_z})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
