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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(part, name, size, xyz, material) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder_x(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_neg_y(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_pos_y(part, name, radius, length, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_toaster_oven")

    enamel = model.material("enamel_cream", color=(0.83, 0.81, 0.76))
    dark_steel = model.material("dark_steel", color=(0.23, 0.24, 0.26))
    brushed = model.material("brushed_aluminum", color=(0.67, 0.68, 0.70))
    zinc = model.material("zinc_plated", color=(0.74, 0.76, 0.78))
    glass = model.material("smoked_glass", rgba=(0.12, 0.13, 0.15, 0.72))
    bakelite = model.material("bakelite_black", color=(0.08, 0.07, 0.06))
    indicator = model.material("indicator_red", color=(0.73, 0.17, 0.12))
    rubber = model.material("rubber_foot", color=(0.08, 0.08, 0.08))

    width = 0.48
    depth = 0.36
    shell_height = 0.29
    wall = 0.012
    foot_height = 0.018

    left_x = -width / 2.0
    right_x = width / 2.0
    front_y = -depth / 2.0
    rear_y = depth / 2.0
    shell_mid_z = foot_height + shell_height / 2.0
    top_z = foot_height + shell_height

    opening_width = 0.30
    opening_height = 0.165
    opening_center_x = -0.065
    opening_left = opening_center_x - opening_width / 2.0
    opening_right = opening_center_x + opening_width / 2.0
    opening_bottom = 0.073
    opening_top = opening_bottom + opening_height

    front_frame_y = front_y + wall / 2.0

    knob_x = 0.176
    knob_zs = {
        "temperature": 0.240,
        "function": 0.177,
        "timer": 0.114,
    }

    body = model.part("body")

    # Main shell panels
    _add_box(
        body,
        "bottom_panel",
        (width, depth, wall),
        (0.0, 0.0, foot_height + wall / 2.0),
        enamel,
    )
    _add_box(
        body,
        "top_panel",
        (width, depth, wall),
        (0.0, 0.0, top_z - wall / 2.0),
        enamel,
    )
    _add_box(
        body,
        "left_wall",
        (wall, depth, shell_height),
        (left_x + wall / 2.0, 0.0, shell_mid_z),
        enamel,
    )
    _add_box(
        body,
        "right_wall",
        (wall, depth, shell_height),
        (right_x - wall / 2.0, 0.0, shell_mid_z),
        enamel,
    )
    _add_box(
        body,
        "rear_panel",
        (width - 2.0 * wall, wall, shell_height),
        (0.0, rear_y - wall / 2.0, shell_mid_z),
        enamel,
    )

    # Front fascia: aperture frame plus separate control column.
    _add_box(
        body,
        "left_jamb",
        (0.025, wall, 0.256),
        (opening_left - 0.0125, front_frame_y, 0.146),
        dark_steel,
    )
    _add_box(
        body,
        "top_frame",
        (0.327, wall, top_z - opening_top),
        ((opening_left + 0.112) / 2.0, front_frame_y, (top_z + opening_top) / 2.0),
        dark_steel,
    )
    _add_box(
        body,
        "bottom_sill",
        (0.327, wall, opening_bottom - foot_height),
        (
            (opening_left + 0.112) / 2.0,
            front_frame_y,
            (opening_bottom + foot_height) / 2.0,
        ),
        dark_steel,
    )
    _add_box(
        body,
        "opening_divider",
        (0.027, wall, 0.256),
        (0.0985, front_frame_y, 0.146),
        dark_steel,
    )
    _add_box(
        body,
        "control_column",
        (right_x - 0.112, wall, 0.256),
        ((0.112 + right_x) / 2.0, front_frame_y, 0.146),
        dark_steel,
    )

    # External reinforcements and feet.
    _add_box(
        body,
        "rear_left_corner_strap",
        (0.018, 0.018, 0.180),
        (left_x + 0.009, rear_y - 0.009, 0.165),
        brushed,
    )
    _add_box(
        body,
        "rear_right_corner_strap",
        (0.018, 0.018, 0.180),
        (right_x - 0.009, rear_y - 0.009, 0.165),
        brushed,
    )
    _add_box(
        body,
        "front_control_strap",
        (0.018, 0.070, 0.040),
        (right_x - 0.009, -0.145, 0.278),
        brushed,
    )

    for name, x, y in (
        ("front_left_foot", -0.180, -0.125),
        ("front_right_foot", 0.180, -0.125),
        ("rear_left_foot", -0.180, 0.125),
        ("rear_right_foot", 0.180, 0.125),
    ):
        _add_box(body, name, (0.040, 0.040, foot_height), (x, y, foot_height / 2.0), rubber)

    # Service hatches with bolted covers.
    _add_box(
        body,
        "left_service_hatch",
        (0.003, 0.180, 0.120),
        (left_x - 0.0015, 0.035, 0.192),
        brushed,
    )
    for idx, (y, z) in enumerate(
        (
            (-0.040, 0.147),
            (0.110, 0.147),
            (-0.040, 0.237),
            (0.110, 0.237),
        ),
        start=1,
    ):
        _add_cylinder_x(
            body,
            f"left_hatch_bolt_{idx}",
            radius=0.005,
            length=0.0025,
            xyz=(left_x - 0.00325, y, z),
            material=zinc,
        )

    _add_box(
        body,
        "rear_service_hatch",
        (0.190, 0.003, 0.130),
        (-0.025, rear_y + 0.0015, 0.176),
        brushed,
    )
    for idx, (x, z) in enumerate(
        (
            (-0.095, 0.126),
            (0.045, 0.126),
            (-0.095, 0.226),
            (0.045, 0.226),
        ),
        start=1,
    ):
        _add_cylinder_pos_y(
            body,
            f"rear_hatch_bolt_{idx}",
            radius=0.005,
            length=0.0025,
            xyz=(x, rear_y + 0.00325, z),
            material=zinc,
        )

    # Door hinge brackets and body-side knuckles.
    hinge_axis_y = front_y - 0.004
    hinge_axis_z = opening_bottom - 0.005
    body_knuckle_length = 0.050
    center_knuckle_length = 0.255
    body_knuckle_offset = center_knuckle_length / 2.0 + body_knuckle_length / 2.0

    for side, x in (
        ("left", opening_center_x - body_knuckle_offset),
        ("right", opening_center_x + body_knuckle_offset),
    ):
        _add_cylinder_x(
            body,
            f"{side}_hinge_knuckle",
            radius=0.008,
            length=body_knuckle_length,
            xyz=(x, hinge_axis_y, hinge_axis_z),
            material=brushed,
        )
        _add_box(
            body,
            f"{side}_hinge_bracket",
            (0.030, 0.026, 0.030),
            (x, front_y + 0.010, hinge_axis_z + 0.011),
            dark_steel,
        )

    # Knob adapter plates, bolts, and bushings.
    for label, z in knob_zs.items():
        _add_box(
            body,
            f"{label}_adapter_plate",
            (0.062, 0.006, 0.062),
            (knob_x, front_y - 0.003, z),
            brushed,
        )
        for idx, (dx, dz) in enumerate(
            ((-0.022, -0.022), (0.022, -0.022), (-0.022, 0.022), (0.022, 0.022)),
            start=1,
        ):
            _add_cylinder_neg_y(
                body,
                f"{label}_adapter_bolt_{idx}",
                radius=0.004,
                length=0.003,
                xyz=(knob_x + dx, front_y - 0.0075, z + dz),
                material=zinc,
            )
        _add_cylinder_neg_y(
            body,
            f"{label}_bushing",
            radius=0.011,
            length=0.014,
            xyz=(knob_x, front_y - 0.013, z),
            material=brushed,
        )

    # Door assembly
    door = model.part("door")
    door_width = 0.355
    door_height = 0.205
    door_thickness = 0.018

    _add_cylinder_x(
        door,
        "center_knuckle",
        radius=0.008,
        length=center_knuckle_length,
        xyz=(0.0, 0.0, 0.0),
        material=brushed,
    )
    _add_box(
        door,
        "hinge_strap",
        (center_knuckle_length, 0.008, 0.016),
        (0.0, -0.004, 0.008),
        dark_steel,
    )
    _add_box(
        door,
        "bottom_rail",
        (door_width, door_thickness, 0.036),
        (0.0, -door_thickness / 2.0, 0.026),
        dark_steel,
    )
    _add_box(
        door,
        "left_stile",
        (0.030, door_thickness, 0.160),
        (-door_width / 2.0 + 0.015, -door_thickness / 2.0, 0.116),
        dark_steel,
    )
    _add_box(
        door,
        "right_stile",
        (0.030, door_thickness, 0.160),
        (door_width / 2.0 - 0.015, -door_thickness / 2.0, 0.116),
        dark_steel,
    )
    _add_box(
        door,
        "top_rail",
        (door_width, door_thickness, 0.028),
        (0.0, -door_thickness / 2.0, 0.191),
        dark_steel,
    )
    _add_box(
        door,
        "window_glass",
        (0.287, 0.005, 0.141),
        (0.0, -0.006, 0.1125),
        glass,
    )
    _add_box(
        door,
        "handle_backer_plate",
        (0.330, 0.006, 0.060),
        (0.0, -0.015, 0.145),
        dark_steel,
    )
    _add_box(
        door,
        "left_handle_post",
        (0.018, 0.030, 0.018),
        (-0.105, -0.033, 0.145),
        brushed,
    )
    _add_box(
        door,
        "right_handle_post",
        (0.018, 0.030, 0.018),
        (0.105, -0.033, 0.145),
        brushed,
    )
    _add_cylinder_x(
        door,
        "handle_grip",
        radius=0.011,
        length=0.240,
        xyz=(0.0, -0.059, 0.145),
        material=brushed,
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_center_x, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    # Shaft-driven knobs
    for label in ("temperature", "function", "timer"):
        knob = model.part(f"{label}_knob")
        _add_cylinder_neg_y(knob, "shaft", radius=0.006, length=0.018, xyz=(0.0, -0.009, 0.0), material=zinc)
        _add_cylinder_neg_y(knob, "hub", radius=0.014, length=0.010, xyz=(0.0, -0.023, 0.0), material=brushed)
        _add_cylinder_neg_y(knob, "body", radius=0.022, length=0.020, xyz=(0.0, -0.038, 0.0), material=bakelite)
        _add_cylinder_neg_y(knob, "face_cap", radius=0.024, length=0.004, xyz=(0.0, -0.050, 0.0), material=bakelite)
        _add_box(knob, "pointer", (0.005, 0.003, 0.014), (0.0, -0.0535, 0.014), indicator)

        model.articulation(
            f"body_to_{label}_knob",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, front_y - 0.020, knob_zs[label])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=4.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    temp_knob = object_model.get_part("temperature_knob")
    function_knob = object_model.get_part("function_knob")
    timer_knob = object_model.get_part("timer_knob")
    door_hinge = object_model.get_articulation("body_to_door")
    temp_joint = object_model.get_articulation("body_to_temperature_knob")
    function_joint = object_model.get_articulation("body_to_function_knob")
    timer_joint = object_model.get_articulation("body_to_timer_knob")

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

    ctx.expect_contact(
        door,
        body,
        elem_a="center_knuckle",
        elem_b="left_hinge_knuckle",
        name="door_contacts_left_hinge_knuckle",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="center_knuckle",
        elem_b="right_hinge_knuckle",
        name="door_contacts_right_hinge_knuckle",
    )

    ctx.expect_contact(
        temp_knob,
        body,
        elem_a="shaft",
        elem_b="temperature_bushing",
        name="temperature_knob_supported_by_bushing",
    )
    ctx.expect_contact(
        function_knob,
        body,
        elem_a="shaft",
        elem_b="function_bushing",
        name="function_knob_supported_by_bushing",
    )
    ctx.expect_contact(
        timer_knob,
        body,
        elem_a="shaft",
        elem_b="timer_bushing",
        name="timer_knob_supported_by_bushing",
    )

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.18,
        name="door_covers_front_aperture_in_closed_pose",
    )

    ctx.check(
        "door_hinge_axis_is_horizontal",
        door_hinge.articulation_type == ArticulationType.REVOLUTE and door_hinge.axis == (1.0, 0.0, 0.0),
        details=f"door hinge type/axis was {door_hinge.articulation_type} / {door_hinge.axis}",
    )

    for joint in (temp_joint, function_joint, timer_joint):
        ctx.check(
            f"{joint.name}_is_shaft_rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, -1.0, 0.0),
            details=f"expected continuous knob rotation about -Y, got {joint.articulation_type} / {joint.axis}",
        )

    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
    with ctx.pose({door_hinge: 1.1}):
        open_top = ctx.part_element_world_aabb(door, elem="top_rail")
    if closed_top is None or open_top is None:
        ctx.fail("door_top_rail_pose_probe_available", "missing top_rail AABB in one or more poses")
    else:
        closed_center = _center(closed_top)
        open_center = _center(open_top)
        ctx.check(
            "door_opens_forward_and_down",
            open_center[1] < closed_center[1] - 0.08 and open_center[2] < closed_center[2] - 0.06,
            details=f"closed center={closed_center}, open center={open_center}",
        )

    timer_pointer_closed = ctx.part_element_world_aabb(timer_knob, elem="pointer")
    with ctx.pose({timer_joint: pi / 2.0}):
        timer_pointer_turned = ctx.part_element_world_aabb(timer_knob, elem="pointer")
    if timer_pointer_closed is None or timer_pointer_turned is None:
        ctx.fail("timer_pointer_pose_probe_available", "missing timer pointer AABB in one or more poses")
    else:
        closed_center = _center(timer_pointer_closed)
        turned_center = _center(timer_pointer_turned)
        motion = abs(turned_center[0] - closed_center[0]) + abs(turned_center[2] - closed_center[2])
        ctx.check(
            "timer_knob_pointer_rotates_with_joint",
            motion > 0.012,
            details=f"timer pointer moved only {motion:.5f} m: closed={closed_center}, turned={turned_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
