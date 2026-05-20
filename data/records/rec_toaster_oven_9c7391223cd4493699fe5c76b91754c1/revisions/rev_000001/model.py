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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rect_profile(width: float, height: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _translate_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 24,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _add_knob(
    model: ArticulatedObject,
    body,
    *,
    part_name: str,
    joint_name: str,
    x: float,
    z: float,
    body_depth: float,
    knob_radius: float,
    knob_material,
    cap_material,
    shaft_material,
    mark_material,
) -> None:
    knob = model.part(part_name)
    knob.visual(
        Cylinder(radius=0.0055, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cap_material,
        name="rear_collar",
    )
    knob.visual(
        Cylinder(radius=knob_radius, length=0.020),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=knob_radius * 0.92, length=0.004),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cap_material,
        name="front_cap",
    )
    knob.visual(
        Box((knob_radius * 0.42, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, -0.0285, knob_radius * 0.62)),
        material=mark_material,
        name="pointer",
    )
    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(x, -body_depth * 0.5, z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=-2.6, upper=2.6),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_toaster_oven")

    body_w = 0.46
    body_d = 0.34
    body_h = 0.28
    sheet_t = 0.008

    control_strip_w = 0.108
    left_border = 0.018
    opening_w = 0.315
    opening_h = 0.185
    opening_left = -body_w * 0.5 + left_border
    opening_cx = opening_left + opening_w * 0.5
    opening_bottom = 0.045
    opening_cz = opening_bottom + opening_h * 0.5

    knob_x = body_w * 0.5 - control_strip_w * 0.46
    knob_zs = (0.212, 0.152, 0.092)
    knob_hole_r = 0.0075

    door_w = opening_w + 0.028
    door_h = opening_h + 0.030
    door_t = 0.022
    door_bottom_z = opening_bottom - 0.015
    hinge_y = -body_d * 0.5

    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.15, 0.16, 0.17, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.18, 0.18, 0.19, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.28, 0.34, 0.38))
    mark_red = model.material("mark_red", rgba=(0.85, 0.28, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, sheet_t)),
        origin=Origin(xyz=(0.0, 0.0, sheet_t * 0.5)),
        material=stainless,
        name="bottom_pan",
    )
    body.visual(
        Box((body_w, body_d, sheet_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - sheet_t * 0.5)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((sheet_t, body_d, body_h - 2.0 * sheet_t)),
        origin=Origin(xyz=(-body_w * 0.5 + sheet_t * 0.5, 0.0, body_h * 0.5)),
        material=stainless,
        name="left_shell",
    )
    body.visual(
        Box((sheet_t, body_d, body_h - 2.0 * sheet_t)),
        origin=Origin(xyz=(body_w * 0.5 - sheet_t * 0.5, 0.0, body_h * 0.5)),
        material=stainless,
        name="right_shell",
    )
    body.visual(
        Box((body_w - 2.0 * sheet_t, sheet_t, body_h - 2.0 * sheet_t)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - sheet_t * 0.5, body_h * 0.5)),
        material=stainless,
        name="rear_shell",
    )

    outer_profile = _rect_profile(body_w, body_h)
    hole_profiles = [
        _translate_profile(
            rounded_rect_profile(opening_w, opening_h, 0.012, corner_segments=6),
            opening_cx,
            body_h * 0.5 - opening_cz,
        )
    ]
    for knob_z in knob_zs:
        hole_profiles.append(
            _circle_profile(
                knob_hole_r,
                cx=knob_x,
                cy=body_h * 0.5 - knob_z,
                segments=24,
            )
        )
    front_fascia_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=sheet_t,
        center=True,
    ).rotate_x(-pi / 2.0)
    body.visual(
        mesh_from_geometry(front_fascia_geom, "toaster_oven_front_fascia"),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + sheet_t * 0.5, body_h * 0.5)),
        material=dark_paint,
        name="front_fascia",
    )

    body.visual(
        Box((0.018, 0.022, 0.038)),
        origin=Origin(
            xyz=(opening_cx - door_w * 0.5 + 0.016, -body_d * 0.5 + 0.011, door_bottom_z + 0.019)
        ),
        material=stainless,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.018, 0.022, 0.038)),
        origin=Origin(
            xyz=(opening_cx + door_w * 0.5 - 0.016, -body_d * 0.5 + 0.011, door_bottom_z + 0.019)
        ),
        material=stainless,
        name="right_hinge_bracket",
    )
    reinforcement_w = control_strip_w * 0.82
    reinforcement_h = 0.172
    reinforcement_z = knob_zs[1]
    reinforcement_holes = [
        _circle_profile(
            knob_hole_r + 0.001,
            cx=0.0,
            cy=reinforcement_z - knob_z,
            segments=24,
        )
        for knob_z in knob_zs
    ]
    control_reinforcement_geom = ExtrudeWithHolesGeometry(
        _rect_profile(reinforcement_w, reinforcement_h),
        reinforcement_holes,
        height=0.012,
        center=True,
    ).rotate_x(-pi / 2.0)
    body.visual(
        mesh_from_geometry(control_reinforcement_geom, "toaster_oven_control_reinforcement"),
        origin=Origin(xyz=(knob_x, -body_d * 0.5 + 0.014, reinforcement_z)),
        material=stainless,
        name="control_reinforcement",
    )

    door = model.part("door")
    frame_side_w = 0.019
    frame_top_h = 0.020
    frame_bottom_h = 0.022
    hinge_ear_x = door_w * 0.5 - 0.012

    door.visual(
        Box((door_w, door_t, frame_bottom_h)),
        origin=Origin(xyz=(0.0, -door_t * 0.5, frame_bottom_h * 0.5)),
        material=dark_paint,
        name="bottom_rail",
    )
    door.visual(
        Box((door_w, door_t, frame_top_h)),
        origin=Origin(xyz=(0.0, -door_t * 0.5, door_h - frame_top_h * 0.5)),
        material=dark_paint,
        name="top_rail",
    )
    door.visual(
        Box((frame_side_w, door_t, door_h)),
        origin=Origin(xyz=(-door_w * 0.5 + frame_side_w * 0.5, -door_t * 0.5, door_h * 0.5)),
        material=dark_paint,
        name="left_rail",
    )
    door.visual(
        Box((frame_side_w, door_t, door_h)),
        origin=Origin(xyz=(door_w * 0.5 - frame_side_w * 0.5, -door_t * 0.5, door_h * 0.5)),
        material=dark_paint,
        name="right_rail",
    )
    glass_w = door_w - 2.0 * frame_side_w + 0.001
    glass_h = door_h - frame_top_h - frame_bottom_h + 0.001
    door.visual(
        Box((glass_w, 0.014, glass_h)),
        origin=Origin(xyz=(0.0, -0.007, frame_bottom_h + glass_h * 0.5)),
        material=glass,
        name="glass_panel",
    )
    door.visual(
        Box((door_w * 0.68, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, -0.046, door_h - 0.032)),
        material=black_plastic,
        name="handle_grip",
    )
    door.visual(
        Box((0.018, 0.038, 0.026)),
        origin=Origin(xyz=(-door_w * 0.22, -0.029, door_h - 0.032)),
        material=black_plastic,
        name="left_handle_post",
    )
    door.visual(
        Box((0.018, 0.038, 0.026)),
        origin=Origin(xyz=(door_w * 0.22, -0.029, door_h - 0.032)),
        material=black_plastic,
        name="right_handle_post",
    )
    door.visual(
        Box((0.014, 0.018, 0.030)),
        origin=Origin(xyz=(-hinge_ear_x, -0.009, 0.015)),
        material=dark_paint,
        name="left_hinge_ear",
    )
    door.visual(
        Box((0.014, 0.018, 0.030)),
        origin=Origin(xyz=(hinge_ear_x, -0.009, 0.015)),
        material=dark_paint,
        name="right_hinge_ear",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_cx, hinge_y, door_bottom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    _add_knob(
        model,
        body,
        part_name="function_knob",
        joint_name="function_knob_turn",
        x=knob_x,
        z=knob_zs[0],
        body_depth=body_d,
        knob_radius=0.019,
        knob_material=black_plastic,
        cap_material=knob_cap,
        shaft_material=shaft_steel,
        mark_material=mark_red,
    )
    _add_knob(
        model,
        body,
        part_name="temperature_knob",
        joint_name="temperature_knob_turn",
        x=knob_x,
        z=knob_zs[1],
        body_depth=body_d,
        knob_radius=0.019,
        knob_material=black_plastic,
        cap_material=knob_cap,
        shaft_material=shaft_steel,
        mark_material=mark_red,
    )
    _add_knob(
        model,
        body,
        part_name="timer_knob",
        joint_name="timer_knob_turn",
        x=knob_x,
        z=knob_zs[2],
        body_depth=body_d,
        knob_radius=0.019,
        knob_material=black_plastic,
        cap_material=knob_cap,
        shaft_material=shaft_steel,
        mark_material=mark_red,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    function_knob = object_model.get_part("function_knob")
    temperature_knob = object_model.get_part("temperature_knob")
    timer_knob = object_model.get_part("timer_knob")

    door_hinge = object_model.get_articulation("body_to_door")
    function_turn = object_model.get_articulation("function_knob_turn")
    temperature_turn = object_model.get_articulation("temperature_knob_turn")
    timer_turn = object_model.get_articulation("timer_knob_turn")

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

    ctx.check(
        "door_hinge_axis_is_x",
        tuple(round(v, 3) for v in door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected body_to_door axis (1, 0, 0), got {door_hinge.axis}",
    )
    for joint in (function_turn, temperature_turn, timer_turn):
        ctx.check(
            f"{joint.name}_axis_is_y",
            tuple(round(v, 3) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"expected {joint.name} axis (0, 1, 0), got {joint.axis}",
        )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(body, door, name="door_closed_contact")
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.20,
            name="door_covers_front_opening",
        )
        ctx.expect_contact(function_knob, body, name="function_knob_mounted")
        ctx.expect_contact(temperature_knob, body, name="temperature_knob_mounted")
        ctx.expect_contact(timer_knob, body, name="timer_knob_mounted")

    ctx.expect_origin_gap(
        function_knob,
        temperature_knob,
        axis="z",
        min_gap=0.045,
        max_gap=0.075,
        name="upper_to_middle_knob_spacing",
    )
    ctx.expect_origin_gap(
        temperature_knob,
        timer_knob,
        axis="z",
        min_gap=0.045,
        max_gap=0.075,
        name="middle_to_lower_knob_spacing",
    )

    with ctx.pose({door_hinge: 0.0}):
        closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_pose_clearance")
        open_door_aabb = ctx.part_world_aabb(door)

    door_moves_outward = (
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.05
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.05
    )
    ctx.check(
        "door_opens_downward_and_forward",
        door_moves_outward,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
