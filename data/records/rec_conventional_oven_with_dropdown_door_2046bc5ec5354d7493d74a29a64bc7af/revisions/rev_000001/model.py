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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _rect_loop(width: float, height: float, *, bottom: float = 0.0) -> list[tuple[float, float]]:
    half_w = width * 0.5
    top = bottom + height
    return [
        (-half_w, bottom),
        (half_w, bottom),
        (half_w, top),
        (-half_w, top),
    ]


def _build_door_frame_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    border_x: float,
    border_z: float,
):
    inner_width = width - 2.0 * border_x
    inner_height = height - 2.0 * border_z
    geom = ExtrudeWithHolesGeometry(
        _rect_loop(width, height),
        [_rect_loop(inner_width, inner_height, bottom=border_z)],
        thickness,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, -thickness * 0.5, 0.0)
    return mesh_from_geometry(geom, "toaster_oven_door_frame")


def _build_hinge_knuckles_mesh(*, width: float, radius: float):
    segment_length = width * 0.16
    segment_centers = (-width * 0.31, 0.0, width * 0.31)
    geom = None
    for center_x in segment_centers:
        segment = CylinderGeometry(radius, segment_length, radial_segments=22).rotate_y(
            math.pi / 2.0
        )
        segment.translate(center_x, 0.0, 0.0)
        if geom is None:
            geom = segment
        else:
            geom.merge(segment)
    assert geom is not None
    return mesh_from_geometry(geom, "toaster_oven_hinge_knuckles")


def _build_wire_rack_mesh(*, width: float, depth: float):
    z = 0.010
    geom = wire_from_points(
        [
            (-width * 0.5, -depth * 0.5, z),
            (-width * 0.5, depth * 0.5, z),
            (width * 0.5, depth * 0.5, z),
            (width * 0.5, -depth * 0.5, z),
        ],
        radius=0.0026,
        radial_segments=16,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.006,
        corner_segments=8,
    )

    slat_count = 6
    for index in range(slat_count):
        t = (index + 1) / (slat_count + 1)
        x = -width * 0.5 + t * width
        geom.merge(
            wire_from_points(
                [
                    (x, -depth * 0.5, z),
                    (x, depth * 0.5, z),
                ],
                radius=0.0014,
                radial_segments=12,
                cap_ends=True,
                corner_mode="miter",
            )
        )

    for y in (-depth * 0.18, depth * 0.18):
        geom.merge(
            wire_from_points(
                [
                    (-width * 0.5, y, z),
                    (width * 0.5, y, z),
                ],
                radius=0.0017,
                radial_segments=12,
                cap_ends=True,
                corner_mode="miter",
            )
        )

    return mesh_from_geometry(geom, "toaster_oven_wire_rack_v2")


def _add_knob(
    model: ArticulatedObject,
    cabinet,
    *,
    part_name: str,
    joint_name: str,
    position: tuple[float, float, float],
    knob_material,
    accent_material,
):
    knob = model.part(part_name)
    knob.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_face",
    )
    knob.visual(
        Box((0.004, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, 0.016)),
        material=accent_material,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.050)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
    )
    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=position),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=-2.35,
            upper=2.35,
        ),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_convection_toaster_oven")

    stainless = model.material("stainless", rgba=(0.76, 0.77, 0.79, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.58, 0.71, 0.80, 0.26))
    chrome = model.material("chrome", rgba=(0.80, 0.81, 0.83, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_mark = model.material("knob_mark", rgba=(0.88, 0.24, 0.18, 1.0))
    foot_black = model.material("foot_black", rgba=(0.12, 0.12, 0.12, 1.0))

    outer_w = 0.46
    outer_d = 0.34
    outer_h = 0.28
    foot_h = 0.012
    wall = 0.014
    front_panel_t = 0.018
    control_w = 0.090

    chamber_xmin = -outer_w * 0.5 + wall
    chamber_xmax = outer_w * 0.5 - control_w - wall
    chamber_center_x = (chamber_xmin + chamber_xmax) * 0.5
    chamber_w = chamber_xmax - chamber_xmin

    opening_bottom_z = foot_h + 0.050
    door_h = 0.180
    opening_top_z = opening_bottom_z + door_h
    door_w = chamber_w - 0.018
    door_t = 0.022

    shelf_z = foot_h + 0.105
    support_rail_w = 0.012
    support_rail_h = 0.010
    support_rail_len = 0.300
    rack_w = 0.3128
    rack_d = 0.228
    rack_runner_w = 0.012
    rack_runner_h = 0.010

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((outer_w, outer_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + wall * 0.5)),
        material=stainless,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((outer_w, outer_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + outer_h - wall * 0.5)),
        material=stainless,
        name="top_panel",
    )
    cabinet.visual(
        Box((wall, outer_d, outer_h - 2.0 * wall)),
        origin=Origin(
            xyz=(-outer_w * 0.5 + wall * 0.5, 0.0, foot_h + outer_h * 0.5)
        ),
        material=stainless,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall, outer_d, outer_h - 2.0 * wall)),
        origin=Origin(
            xyz=(outer_w * 0.5 - wall * 0.5, 0.0, foot_h + outer_h * 0.5)
        ),
        material=stainless,
        name="right_wall",
    )
    cabinet.visual(
        Box((wall, outer_d, outer_h - 2.0 * wall)),
        origin=Origin(xyz=(chamber_xmax + wall * 0.5, 0.0, foot_h + outer_h * 0.5)),
        material=stainless,
        name="divider_wall",
    )
    cabinet.visual(
        Box((outer_w - 2.0 * wall, wall, outer_h - 2.0 * wall)),
        origin=Origin(
            xyz=(0.0, outer_d * 0.5 - wall * 0.5, foot_h + outer_h * 0.5)
        ),
        material=stainless,
        name="back_wall",
    )
    cabinet.visual(
        Box((chamber_w, front_panel_t, foot_h + outer_h - wall - opening_top_z)),
        origin=Origin(
            xyz=(
                chamber_center_x,
                -outer_d * 0.5 + front_panel_t * 0.5,
                (opening_top_z + (foot_h + outer_h - wall)) * 0.5,
            )
        ),
        material=dark_trim,
        name="front_header",
    )
    cabinet.visual(
        Box((chamber_w, front_panel_t, opening_bottom_z - (foot_h + wall))),
        origin=Origin(
            xyz=(
                chamber_center_x,
                -outer_d * 0.5 + front_panel_t * 0.5,
                (opening_bottom_z + (foot_h + wall)) * 0.5,
            )
        ),
        material=dark_trim,
        name="front_sill",
    )
    cabinet.visual(
        Box((control_w - wall, front_panel_t, outer_h - 2.0 * wall)),
        origin=Origin(
            xyz=(
                (chamber_xmax + wall + (outer_w * 0.5 - wall)) * 0.5,
                -outer_d * 0.5 + front_panel_t * 0.5,
                foot_h + outer_h * 0.5,
            )
        ),
        material=dark_trim,
        name="control_panel",
    )
    cabinet.visual(
        Box((support_rail_w, support_rail_len, support_rail_h)),
        origin=Origin(
            xyz=(
                chamber_xmin + support_rail_w * 0.5,
                -0.006,
                shelf_z - support_rail_h * 0.5,
            )
        ),
        material=chrome,
        name="left_support_rail",
    )
    cabinet.visual(
        Box((support_rail_w, support_rail_len, support_rail_h)),
        origin=Origin(
            xyz=(
                chamber_xmax - support_rail_w * 0.5,
                -0.006,
                shelf_z - support_rail_h * 0.5,
            )
        ),
        material=chrome,
        name="right_support_rail",
    )
    for sx in (-0.17, 0.17):
        for sy in (-0.12, 0.12):
            cabinet.visual(
                Box((0.032, 0.032, foot_h)),
                origin=Origin(xyz=(sx, sy, foot_h * 0.5)),
                material=foot_black,
            )
    cabinet.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h + foot_h)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (outer_h + foot_h) * 0.5)),
    )

    door = model.part("door")
    door.visual(
        _build_door_frame_mesh(
            width=door_w,
            height=door_h,
            thickness=door_t,
            border_x=0.028,
            border_z=0.030,
        ),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=dark_trim,
        name="door_frame",
    )
    door.visual(
        Box((door_w - 0.046, 0.006, door_h - 0.050)),
        origin=Origin(xyz=(0.0, 0.002, door_h * 0.5)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.0065, length=door_w * 0.86),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_knuckles",
    )
    door.visual(
        Box((0.172, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.006, door_h - 0.020)),
        material=chrome,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -door_t * 0.5, door_h * 0.5)),
    )

    rack = model.part("wire_rack")
    rack.visual(
        _build_wire_rack_mesh(width=rack_w, depth=rack_d),
        material=chrome,
        name="rack_mesh",
    )
    rack.visual(
        Box((rack_runner_w, 0.252, rack_runner_h)),
        origin=Origin(
            xyz=(
                -(chamber_w * 0.5 - rack_runner_w * 0.5 - 0.001),
                0.0,
                rack_runner_h * 0.5,
            )
        ),
        material=chrome,
        name="left_runner",
    )
    rack.visual(
        Box((rack_runner_w, 0.252, rack_runner_h)),
        origin=Origin(
            xyz=(
                chamber_w * 0.5 - rack_runner_w * 0.5 - 0.001,
                0.0,
                rack_runner_h * 0.5,
            )
        ),
        material=chrome,
        name="right_runner",
    )
    rack.inertial = Inertial.from_geometry(
        Box((rack_w + 0.040, 0.252, 0.024)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    temperature_knob = _add_knob(
        model,
        cabinet,
        part_name="temperature_knob",
        joint_name="temperature_knob_turn",
        position=(0.178, -outer_d * 0.5, foot_h + 0.215),
        knob_material=knob_black,
        accent_material=knob_mark,
    )
    function_knob = _add_knob(
        model,
        cabinet,
        part_name="function_knob",
        joint_name="function_knob_turn",
        position=(0.178, -outer_d * 0.5, foot_h + 0.155),
        knob_material=knob_black,
        accent_material=knob_mark,
    )
    timer_knob = _add_knob(
        model,
        cabinet,
        part_name="timer_knob",
        joint_name="timer_knob_turn",
        position=(0.178, -outer_d * 0.5, foot_h + 0.095),
        knob_material=knob_black,
        accent_material=knob_mark,
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(chamber_center_x, -outer_d * 0.5 - 0.010, opening_bottom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=rack,
        origin=Origin(xyz=(chamber_center_x, -0.006, shelf_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.12,
            lower=0.0,
            upper=0.115,
        ),
    )

    model.meta["primary_parts"] = [
        cabinet.name,
        door.name,
        rack.name,
        temperature_knob.name,
        function_knob.name,
        timer_knob.name,
    ]
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
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    rack = object_model.get_part("wire_rack")
    temperature_knob = object_model.get_part("temperature_knob")
    function_knob = object_model.get_part("function_knob")
    timer_knob = object_model.get_part("timer_knob")

    door_hinge = object_model.get_articulation("door_hinge")
    rack_slide = object_model.get_articulation("rack_slide")
    temperature_knob_turn = object_model.get_articulation("temperature_knob_turn")
    function_knob_turn = object_model.get_articulation("function_knob_turn")
    timer_knob_turn = object_model.get_articulation("timer_knob_turn")

    front_header = cabinet.get_visual("front_header")
    left_support_rail = cabinet.get_visual("left_support_rail")
    right_support_rail = cabinet.get_visual("right_support_rail")
    door_frame = door.get_visual("door_frame")
    left_runner = rack.get_visual("left_runner")
    right_runner = rack.get_visual("right_runner")

    ctx.fail_if_parts_overlap_in_current_pose()

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    if cabinet_aabb is None:
        ctx.fail("cabinet_bounds_available", "Cabinet world AABB could not be resolved.")
    else:
        size_x = cabinet_aabb[1][0] - cabinet_aabb[0][0]
        size_y = cabinet_aabb[1][1] - cabinet_aabb[0][1]
        size_z = cabinet_aabb[1][2] - cabinet_aabb[0][2]
        ctx.check(
            "countertop_oven_realistic_size",
            0.42 <= size_x <= 0.50 and 0.31 <= size_y <= 0.36 and 0.28 <= size_z <= 0.31,
            f"Unexpected cabinet size {(size_x, size_y, size_z)}",
        )

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem=front_header,
        negative_elem=door_frame,
        min_gap=0.0,
        max_gap=0.012,
        name="door_closed_flush_to_front",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.18,
        elem_a=door_frame,
        name="door_covers_opening_projection",
    )
    ctx.expect_contact(
        rack,
        cabinet,
        elem_a=left_runner,
        elem_b=left_support_rail,
        name="left_runner_supported",
    )
    ctx.expect_contact(
        rack,
        cabinet,
        elem_a=right_runner,
        elem_b=right_support_rail,
        name="right_runner_supported",
    )
    ctx.expect_within(rack, cabinet, axes="x", margin=0.0, name="rack_within_chamber_width")
    ctx.expect_contact(temperature_knob, cabinet, name="temperature_knob_mounted")
    ctx.expect_contact(function_knob, cabinet, name="function_knob_mounted")
    ctx.expect_contact(timer_knob, cabinet, name="timer_knob_mounted")

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_lower_no_floating")

        handle_closed = ctx.part_element_world_aabb(door, elem="door_handle")
        if handle_closed is None:
            ctx.fail("door_handle_closed_aabb", "Door handle AABB unavailable at closed pose.")
            handle_closed = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        with ctx.pose({door_hinge: door_limits.upper}):
            handle_open = ctx.part_element_world_aabb(door, elem="door_handle")
            if handle_open is None:
                ctx.fail("door_handle_open_aabb", "Door handle AABB unavailable at open pose.")
                handle_open = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            closed_handle_min_y = handle_closed[0][1]
            open_handle_min_y = handle_open[0][1]
            closed_handle_max_z = handle_closed[1][2]
            open_handle_max_z = handle_open[1][2]
            ctx.check(
                "door_handle_swings_forward",
                open_handle_min_y < closed_handle_min_y - 0.06,
                f"closed_min_y={closed_handle_min_y} open_min_y={open_handle_min_y}",
            )
            ctx.check(
                "door_handle_drops_when_open",
                open_handle_max_z < closed_handle_max_z - 0.08,
                f"closed_max_z={closed_handle_max_z} open_max_z={open_handle_max_z}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_upper_no_floating")

    rack_limits = rack_slide.motion_limits
    if rack_limits is not None and rack_limits.lower is not None and rack_limits.upper is not None:
        with ctx.pose({rack_slide: rack_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rack_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="rack_slide_lower_no_floating")

        rack_rest = ctx.part_world_position(rack)
        if rack_rest is None:
            ctx.fail("rack_rest_position", "Rack world position unavailable at rest pose.")
            rack_rest = (0.0, 0.0, 0.0)
        door_upper = 0.0
        if door_limits is not None and door_limits.upper is not None:
            door_upper = door_limits.upper
        with ctx.pose({rack_slide: rack_limits.upper}):
            rack_open = ctx.part_world_position(rack)
            if rack_open is None:
                ctx.fail("rack_open_position", "Rack world position unavailable at extended pose.")
                rack_open = rack_rest
            ctx.check(
                "rack_extends_forward",
                rack_open[1] < rack_rest[1] - 0.10,
                f"rack_rest_y={rack_rest[1]} rack_open_y={rack_open[1]}",
            )
        with ctx.pose({rack_slide: rack_limits.upper, door_hinge: door_upper}):
            ctx.expect_contact(
                rack,
                cabinet,
                elem_a=left_runner,
                elem_b=left_support_rail,
                name="left_runner_supported_open",
            )
            ctx.expect_contact(
                rack,
                cabinet,
                elem_a=right_runner,
                elem_b=right_support_rail,
                name="right_runner_supported_open",
            )
            rack_aabb = ctx.part_world_aabb(rack)
            cabinet_pose_aabb = ctx.part_world_aabb(cabinet)
            if rack_aabb is None or cabinet_pose_aabb is None:
                ctx.fail(
                    "rack_open_pose_aabbs",
                    "Rack or cabinet AABB unavailable in open-door open-rack pose.",
                )
            else:
                ctx.check(
                    "rack_projects_outside_front_when_extended",
                    rack_aabb[0][1] < cabinet_pose_aabb[0][1] - 0.05,
                    f"rack_min_y={rack_aabb[0][1]} cabinet_min_y={cabinet_pose_aabb[0][1]}",
                )
            ctx.fail_if_parts_overlap_in_current_pose(name="rack_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rack_slide_upper_no_floating")

    def _visual_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    for knob_part, knob_joint in (
        (temperature_knob, temperature_knob_turn),
        (function_knob, function_knob_turn),
        (timer_knob, timer_knob_turn),
    ):
        limits = knob_joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({knob_joint: limits.lower}):
            lower_indicator = ctx.part_element_world_aabb(knob_part, elem="indicator")
            if lower_indicator is None:
                ctx.fail(f"{knob_joint.name}_lower_indicator", "Indicator AABB unavailable at lower pose.")
                lower_indicator = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{knob_joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{knob_joint.name}_lower_no_floating")
        with ctx.pose({knob_joint: limits.upper}):
            upper_indicator = ctx.part_element_world_aabb(knob_part, elem="indicator")
            if upper_indicator is None:
                ctx.fail(f"{knob_joint.name}_upper_indicator", "Indicator AABB unavailable at upper pose.")
                upper_indicator = lower_indicator
            lower_center = _visual_center(lower_indicator)
            upper_center = _visual_center(upper_indicator)
            ctx.check(
                f"{knob_joint.name}_indicator_rotates",
                abs(upper_center[0] - lower_center[0]) > 0.010,
                f"lower_center={lower_center} upper_center={upper_center}",
            )
            ctx.expect_contact(knob_part, cabinet, name=f"{knob_joint.name}_mounted_open")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{knob_joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{knob_joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
