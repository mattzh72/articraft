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
)


def _rect_profile(height: float, width: float) -> list[tuple[float, float]]:
    half_h = height * 0.5
    half_w = width * 0.5
    return [
        (-half_h, -half_w),
        (half_h, -half_w),
        (half_h, half_w),
        (-half_h, half_w),
    ]


def _circle_profile(
    center_h: float,
    center_w: float,
    radius: float,
    *,
    segments: int = 20,
) -> list[tuple[float, float]]:
    return [
        (
            center_h + radius * cos(2.0 * pi * index / segments),
            center_w + radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _add_x_cylinder(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_y_cylinder(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_toaster_oven")

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    liner_steel = model.material("liner_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.08, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.87, 0.75, 0.15, 1.0))
    signal_red = model.material("signal_red", rgba=(0.72, 0.12, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.24, 0.30, 0.34, 0.60))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    body_width = 0.62
    body_depth = 0.44
    body_height = 0.34
    foot_height = 0.020
    wall = 0.006
    frame_depth = 0.028
    front_x = -body_depth * 0.5
    rear_x = body_depth * 0.5
    body_bottom = foot_height
    body_top = foot_height + body_height

    door_width = 0.44
    door_height = 0.23
    door_thickness = 0.024
    door_center_y = -0.070
    door_bottom_z = body_bottom + 0.050
    door_right = door_center_y + door_width * 0.5
    door_left = door_center_y - door_width * 0.5

    control_width = 0.135
    control_height = 0.240
    control_center_y = 0.2375
    control_center_z = 0.205
    control_plate_t = 0.004
    control_hole_radius = 0.013
    knob_zs = {
        "power_knob": 0.125,
        "temperature_knob": 0.205,
        "timer_knob": 0.285,
    }

    body = model.part("body")

    body.visual(
        Box((body_depth, wall, body_height)),
        origin=Origin(xyz=(0.0, -body_width * 0.5 + wall * 0.5, body_bottom + body_height * 0.5)),
        material=stainless,
        name="left_shell",
    )
    body.visual(
        Box((body_depth, wall, body_height)),
        origin=Origin(xyz=(0.0, body_width * 0.5 - wall * 0.5, body_bottom + body_height * 0.5)),
        material=stainless,
        name="right_shell",
    )
    body.visual(
        Box((body_depth, body_width - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall * 0.5)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((body_depth, body_width - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_top - wall * 0.5)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((wall, body_width - 2.0 * wall, body_height - 2.0 * wall)),
        origin=Origin(xyz=(rear_x - wall * 0.5, 0.0, body_bottom + body_height * 0.5)),
        material=stainless,
        name="rear_shell",
    )

    left_jamb_width = door_left - (-body_width * 0.5 + wall)
    body.visual(
        Box((frame_depth, left_jamb_width, 0.042)),
        origin=Origin(
            xyz=(
                front_x + frame_depth * 0.5,
                -body_width * 0.5 + wall + left_jamb_width * 0.5,
                0.041,
            )
        ),
        material=dark_steel,
        name="front_left_jamb_lower",
    )
    body.visual(
        Box((frame_depth, left_jamb_width, 0.213)),
        origin=Origin(
            xyz=(
                front_x + frame_depth * 0.5,
                -body_width * 0.5 + wall + left_jamb_width * 0.5,
                0.2025,
            )
        ),
        material=dark_steel,
        name="front_left_jamb_upper",
    )
    body.visual(
        Box((frame_depth, 0.020, 0.170)),
        origin=Origin(xyz=(front_x + frame_depth * 0.5, 0.160, 0.245)),
        material=dark_steel,
        name="front_center_stile",
    )
    body.visual(
        Box((frame_depth, door_width, body_top - (door_bottom_z + door_height))),
        origin=Origin(
            xyz=(
                front_x + frame_depth * 0.5,
                door_center_y,
                door_bottom_z + door_height + (body_top - (door_bottom_z + door_height)) * 0.5,
            )
        ),
        material=dark_steel,
        name="front_header",
    )
    body.visual(
        Box((frame_depth, door_width, door_bottom_z - body_bottom)),
        origin=Origin(
            xyz=(
                front_x + frame_depth * 0.5,
                door_center_y,
                body_bottom + (door_bottom_z - body_bottom) * 0.5,
            )
        ),
        material=dark_steel,
        name="front_sill",
    )
    body.visual(
        Box((frame_depth, control_width, 0.020)),
        origin=Origin(
            xyz=(
                front_x + frame_depth * 0.5,
                control_center_y,
                control_center_z + control_height * 0.5 + 0.010,
            )
        ),
        material=dark_steel,
        name="control_top_brace",
    )
    body.visual(
        Box((frame_depth, control_width, 0.028)),
        origin=Origin(
            xyz=(
                front_x + frame_depth * 0.5,
                control_center_y,
                control_center_z - control_height * 0.5 - 0.014,
            )
        ),
        material=dark_steel,
        name="control_bottom_brace",
    )

    control_face = ExtrudeWithHolesGeometry(
        _rect_profile(control_height, control_width),
        [
            _circle_profile(knob_z - control_center_z, 0.0, control_hole_radius)
            for knob_z in knob_zs.values()
        ],
        height=control_plate_t,
        center=True,
    ).rotate_y(pi / 2.0)
    body.visual(
        mesh_from_geometry(control_face, "industrial_toaster_control_face"),
        origin=Origin(xyz=(front_x + control_plate_t * 0.5, control_center_y, control_center_z)),
        material=stainless,
        name="control_face",
    )

    chamber_depth = 0.310
    chamber_width = 0.400
    chamber_height = 0.205
    chamber_front_x = front_x + frame_depth + 0.006
    chamber_center_x = chamber_front_x + chamber_depth * 0.5
    chamber_center_z = 0.190
    liner = 0.004

    body.visual(
        Box((chamber_depth, chamber_width, liner)),
        origin=Origin(xyz=(chamber_center_x, door_center_y, chamber_center_z - chamber_height * 0.5 + liner * 0.5)),
        material=liner_steel,
        name="chamber_floor",
    )
    body.visual(
        Box((chamber_depth, chamber_width, liner)),
        origin=Origin(xyz=(chamber_center_x, door_center_y, chamber_center_z + chamber_height * 0.5 - liner * 0.5)),
        material=liner_steel,
        name="chamber_ceiling",
    )
    body.visual(
        Box((chamber_depth, liner, chamber_height)),
        origin=Origin(
            xyz=(
                chamber_center_x,
                door_center_y - chamber_width * 0.5 + liner * 0.5,
                chamber_center_z,
            )
        ),
        material=liner_steel,
        name="chamber_left_wall",
    )
    body.visual(
        Box((chamber_depth, liner, chamber_height)),
        origin=Origin(
            xyz=(
                chamber_center_x,
                door_center_y + chamber_width * 0.5 - liner * 0.5,
                chamber_center_z,
            )
        ),
        material=liner_steel,
        name="chamber_right_wall",
    )
    body.visual(
        Box((liner, chamber_width, chamber_height)),
        origin=Origin(
            xyz=(
                chamber_front_x + chamber_depth - liner * 0.5,
                door_center_y,
                chamber_center_z,
            )
        ),
        material=liner_steel,
        name="chamber_back_wall",
    )

    for side_name, side_y in (
        ("left", door_center_y - chamber_width * 0.5 + 0.008),
        ("right", door_center_y + chamber_width * 0.5 - 0.008),
    ):
        for rail_index, rail_z in enumerate((0.145, 0.205), start=1):
            body.visual(
                Box((0.200, 0.012, 0.008)),
                origin=Origin(xyz=(chamber_front_x + 0.120, side_y, rail_z)),
                material=liner_steel,
                name=f"{side_name}_rack_rail_{rail_index}",
            )

    guard_center_x = front_x - 0.034
    guard_width = 0.152
    guard_height = 0.220
    guard_bar = 0.012
    guard_top_z = 0.314
    guard_bottom_z = 0.106
    guard_side_offset = guard_width * 0.5 - guard_bar * 0.5
    guard_center_z = (guard_top_z + guard_bottom_z) * 0.5
    standoff_x_len = 0.028
    standoff_center_x = (front_x + (guard_center_x + guard_bar * 0.5)) * 0.5

    body.visual(
        Box((guard_bar, guard_width, guard_bar)),
        origin=Origin(xyz=(guard_center_x, control_center_y, guard_top_z)),
        material=guard_yellow,
        name="guard_top_bar",
    )
    body.visual(
        Box((guard_bar, guard_width, guard_bar)),
        origin=Origin(xyz=(guard_center_x, control_center_y, guard_bottom_z)),
        material=guard_yellow,
        name="guard_bottom_bar",
    )
    body.visual(
        Box((guard_bar, guard_bar, guard_height)),
        origin=Origin(xyz=(guard_center_x, control_center_y - guard_side_offset, guard_center_z)),
        material=guard_yellow,
        name="guard_left_bar",
    )
    body.visual(
        Box((guard_bar, guard_bar, guard_height)),
        origin=Origin(xyz=(guard_center_x, control_center_y + guard_side_offset, guard_center_z)),
        material=guard_yellow,
        name="guard_right_bar",
    )

    guard_corner_specs = [
        ("upper_left", control_center_y - guard_side_offset, 0.293),
        ("lower_left", control_center_y - guard_side_offset, 0.127),
        ("upper_right", control_center_y + guard_side_offset, 0.293),
        ("lower_right", control_center_y + guard_side_offset, 0.127),
    ]
    for corner_name, corner_y, corner_z in guard_corner_specs:
        body.visual(
            Box((standoff_x_len, guard_bar, guard_bar)),
            origin=Origin(xyz=(standoff_center_x, corner_y, corner_z)),
            material=guard_yellow,
            name=f"{corner_name}_standoff",
        )
        body.visual(
            Box((0.012, 0.020, 0.032)),
            origin=Origin(xyz=(front_x - 0.010, corner_y, corner_z)),
            material=guard_yellow,
            name=f"{corner_name}_gusset",
        )

    hinge_y_offset = door_width * 0.5 - 0.028
    for side_name, hinge_y in (("left", door_center_y - hinge_y_offset), ("right", door_center_y + hinge_y_offset)):
        body.visual(
            Box((0.036, 0.044, 0.074)),
            origin=Origin(xyz=(front_x + 0.028, hinge_y, 0.057)),
            material=dark_steel,
            name=f"{side_name}_hinge_reinforcement",
        )
        body.visual(
            Box((0.018, 0.034, 0.026)),
            origin=Origin(xyz=(front_x + 0.018, hinge_y, 0.083)),
            material=dark_steel,
            name=f"{side_name}_hinge_leaf",
        )
        body.visual(
            Box((0.014, 0.022, 0.020)),
            origin=Origin(xyz=(front_x + 0.026, hinge_y, 0.095)),
            material=dark_steel,
            name=f"{side_name}_door_stop",
        )
        for bolt_index, bolt_z in enumerate((0.035, 0.079), start=1):
            _add_x_cylinder(
                body,
                radius=0.004,
                length=0.004,
                xyz=(front_x + 0.011, hinge_y - 0.010, bolt_z),
                material=black,
                name=f"{side_name}_hinge_bolt_{bolt_index}a",
            )
            _add_x_cylinder(
                body,
                radius=0.004,
                length=0.004,
                xyz=(front_x + 0.011, hinge_y + 0.010, bolt_z),
                material=black,
                name=f"{side_name}_hinge_bolt_{bolt_index}b",
            )

    body.visual(
        Box((0.018, 0.016, 0.032)),
        origin=Origin(xyz=(front_x + 0.009, 0.162, 0.248)),
        material=dark_steel,
        name="body_lockout_tab",
    )

    body.visual(
        Box((0.096, 0.080, 0.020)),
        origin=Origin(xyz=(0.171, door_center_y, 0.112)),
        material=dark_steel,
        name="rear_liner_bridge_lower",
    )
    body.visual(
        Box((0.096, 0.080, 0.020)),
        origin=Origin(xyz=(0.171, door_center_y, 0.268)),
        material=dark_steel,
        name="rear_liner_bridge_upper",
    )

    for knob_name, knob_z in knob_zs.items():
        _add_x_cylinder(
            body,
            radius=0.004,
            length=0.010,
            xyz=(front_x - 0.005, control_center_y - 0.032, knob_z + 0.022),
            material=black,
            name=f"{knob_name}_stop_pin_upper",
        )
        _add_x_cylinder(
            body,
            radius=0.004,
            length=0.010,
            xyz=(front_x - 0.005, control_center_y - 0.032, knob_z - 0.022),
            material=black,
            name=f"{knob_name}_stop_pin_lower",
        )

    for fastener_name, fastener_y, fastener_z in guard_corner_specs:
        _add_x_cylinder(
            body,
            radius=0.004,
            length=0.003,
            xyz=(front_x - 0.0015, fastener_y, fastener_z),
            material=black,
            name=f"{fastener_name}_guard_fastener",
        )

    for foot_name, foot_x, foot_y in (
        ("front_left", -0.150, -0.225),
        ("front_right", -0.150, 0.225),
        ("rear_left", 0.150, -0.225),
        ("rear_right", 0.150, 0.225),
    ):
        body.visual(
            Box((0.060, 0.060, foot_height)),
            origin=Origin(xyz=(foot_x, foot_y, foot_height * 0.5)),
            material=rubber,
            name=f"{foot_name}_foot",
        )

    door_hinge_axis_offset = 0.010
    door = model.part("door")
    side_rail_w = 0.055
    top_rail_h = 0.048
    bottom_rail_h = 0.044

    door.visual(
        Box((door_thickness, side_rail_w, door_height)),
        origin=Origin(
            xyz=(
                door_hinge_axis_offset - door_thickness * 0.5,
                -door_width * 0.5 + side_rail_w * 0.5,
                door_height * 0.5,
            )
        ),
        material=stainless,
        name="left_rail",
    )
    door.visual(
        Box((door_thickness, side_rail_w, door_height)),
        origin=Origin(
            xyz=(
                door_hinge_axis_offset - door_thickness * 0.5,
                door_width * 0.5 - side_rail_w * 0.5,
                door_height * 0.5,
            )
        ),
        material=stainless,
        name="right_rail",
    )
    door.visual(
        Box((door_thickness, door_width, bottom_rail_h)),
        origin=Origin(xyz=(door_hinge_axis_offset - door_thickness * 0.5, 0.0, bottom_rail_h * 0.5)),
        material=stainless,
        name="door_bottom_rail",
    )
    door.visual(
        Box((door_thickness, door_width, top_rail_h)),
        origin=Origin(
            xyz=(door_hinge_axis_offset - door_thickness * 0.5, 0.0, door_height - top_rail_h * 0.5),
        ),
        material=stainless,
        name="door_top_rail",
    )
    door.visual(
        Box((0.006, 0.340, 0.180)),
        origin=Origin(xyz=(door_hinge_axis_offset - door_thickness + 0.003, 0.0, 0.115)),
        material=glass,
        name="door_window",
    )
    door.visual(
        Box((0.018, 0.460, 0.018)),
        origin=Origin(xyz=(door_hinge_axis_offset - 0.030, 0.0, 0.012)),
        material=dark_steel,
        name="door_kick_lip",
    )

    door.visual(
        Box((0.004, 0.050, 0.052)),
        origin=Origin(xyz=(door_hinge_axis_offset - door_thickness + 0.002, -0.120, 0.145)),
        material=dark_steel,
        name="left_handle_plate",
    )
    door.visual(
        Box((0.004, 0.050, 0.052)),
        origin=Origin(xyz=(door_hinge_axis_offset - door_thickness + 0.002, 0.120, 0.145)),
        material=dark_steel,
        name="right_handle_plate",
    )
    door.visual(
        Box((0.042, 0.016, 0.024)),
        origin=Origin(xyz=(door_hinge_axis_offset - 0.045, -0.120, 0.145)),
        material=dark_steel,
        name="left_handle_standoff",
    )
    door.visual(
        Box((0.042, 0.016, 0.024)),
        origin=Origin(xyz=(door_hinge_axis_offset - 0.045, 0.120, 0.145)),
        material=dark_steel,
        name="right_handle_standoff",
    )
    door.visual(
        Box((0.018, 0.270, 0.026)),
        origin=Origin(xyz=(door_hinge_axis_offset - 0.066, 0.0, 0.145)),
        material=dark_steel,
        name="handle_bar",
    )

    for side_name, hinge_y in (("left", -hinge_y_offset), ("right", hinge_y_offset)):
        door.visual(
            Box((0.020, 0.034, 0.040)),
            origin=Origin(xyz=(door_hinge_axis_offset - 0.010, hinge_y, 0.020)),
            material=dark_steel,
            name=f"{side_name}_hinge_strap",
        )
        _add_y_cylinder(
            door,
            radius=0.007,
            length=0.018,
            xyz=(0.0, hinge_y, 0.0),
            material=dark_steel,
            name=f"{side_name}_hinge_barrel",
        )

    door.visual(
        Box((0.008, 0.024, 0.030)),
        origin=Origin(xyz=(door_hinge_axis_offset - 0.014, door_width * 0.5 + 0.012, 0.178)),
        material=signal_red,
        name="door_lockout_tab",
    )

    for side_name, plate_y in (("left", -0.120), ("right", 0.120)):
        _add_x_cylinder(
            door,
            radius=0.0035,
            length=0.003,
            xyz=(door_hinge_axis_offset - 0.0255, plate_y - 0.016, 0.157),
            material=black,
            name=f"{side_name}_handle_bolt_upper",
        )
        _add_x_cylinder(
            door,
            radius=0.0035,
            length=0.003,
            xyz=(door_hinge_axis_offset - 0.0255, plate_y + 0.016, 0.133),
            material=black,
            name=f"{side_name}_handle_bolt_lower",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(front_x - door_hinge_axis_offset, door_center_y, door_bottom_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.30),
    )

    for knob_name, knob_z in knob_zs.items():
        knob = model.part(knob_name)
        _add_x_cylinder(
            knob,
            radius=0.004,
            length=0.028,
            xyz=(0.014, 0.0, 0.0),
            material=dark_steel,
            name="shaft",
        )
        _add_x_cylinder(
            knob,
            radius=0.017,
            length=0.008,
            xyz=(-0.004, 0.0, 0.0),
            material=dark_steel,
            name="knob_flange",
        )
        _add_x_cylinder(
            knob,
            radius=0.020,
            length=0.008,
            xyz=(-0.010, 0.0, 0.0),
            material=dark_steel,
            name="knob_body",
        )
        _add_x_cylinder(
            knob,
            radius=0.026,
            length=0.020,
            xyz=(-0.018, 0.0, 0.0),
            material=black,
            name="knob_grip",
        )
        knob.visual(
            Box((0.010, 0.006, 0.018)),
            origin=Origin(xyz=(-0.026, 0.0, 0.020)),
            material=signal_red,
            name="pointer_fin",
        )
        model.articulation(
            f"body_to_{knob_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(front_x, control_center_y, knob_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-2.2, upper=2.2),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")
    knobs = [
        object_model.get_part("power_knob"),
        object_model.get_part("temperature_knob"),
        object_model.get_part("timer_knob"),
    ]
    knob_joints = [
        object_model.get_articulation("body_to_power_knob"),
        object_model.get_articulation("body_to_temperature_knob"),
        object_model.get_articulation("body_to_timer_knob"),
    ]

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
        "door_hinge_axis_is_bottom_pivot",
        door_hinge.axis == (0.0, -1.0, 0.0),
        f"Expected bottom-hinged door axis (0,-1,0), got {door_hinge.axis}.",
    )
    for knob_joint in knob_joints:
        ctx.check(
            f"{knob_joint.name}_axis_is_shaft_aligned",
            knob_joint.axis == (-1.0, 0.0, 0.0),
            f"Expected shaft axis (-1,0,0), got {knob_joint.axis}.",
        )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            positive_elem="front_header",
            negative_elem="door_top_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="door_seats_against_front_header",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="door_lockout_tab",
            elem_b="body_lockout_tab",
            min_overlap=0.014,
            name="lockout_tabs_align_when_closed",
        )
        for knob in knobs:
            ctx.expect_contact(
                knob,
                body,
                elem_a="knob_flange",
                elem_b="control_face",
                name=f"{knob.name}_flange_contacts_control_face",
            )

    with ctx.pose({door_hinge: 0.0}):
        closed_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    with ctx.pose({door_hinge: 1.15}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle_bar")

    if closed_handle is None or open_handle is None:
        ctx.fail("door_handle_pose_measurement", "Could not resolve handle-bar AABBs for door pose check.")
    else:
        closed_center_x = (closed_handle[0][0] + closed_handle[1][0]) * 0.5
        closed_center_z = (closed_handle[0][2] + closed_handle[1][2]) * 0.5
        open_center_x = (open_handle[0][0] + open_handle[1][0]) * 0.5
        open_center_z = (open_handle[0][2] + open_handle[1][2]) * 0.5
        ctx.check(
            "door_opens_downward_and_outward",
            open_center_z < closed_center_z - 0.08 and open_center_x < closed_center_x - 0.02,
            (
                "Expected handle bar to move downward and outward when opened, "
                f"but centers moved from ({closed_center_x:.3f}, {closed_center_z:.3f}) "
                f"to ({open_center_x:.3f}, {open_center_z:.3f})."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
