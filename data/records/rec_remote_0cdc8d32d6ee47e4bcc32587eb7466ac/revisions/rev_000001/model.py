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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _arc_points_2d(
    cx: float,
    cy: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 6,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def _fob_outline_profile(
    *,
    length: float,
    width: float,
    corner_radius: float,
    slot_left: float,
    slot_right: float,
    slot_depth: float,
) -> list[tuple[float, float]]:
    half_length = length * 0.5
    half_width = width * 0.5
    slot_floor_y = half_width - slot_depth

    profile: list[tuple[float, float]] = [(slot_left, half_width)]
    profile.append((-half_length + corner_radius, half_width))
    profile.extend(
        _arc_points_2d(
            -half_length + corner_radius,
            half_width - corner_radius,
            corner_radius,
            math.pi * 0.5,
            math.pi,
        )[1:]
    )
    profile.append((-half_length, -half_width + corner_radius))
    profile.extend(
        _arc_points_2d(
            -half_length + corner_radius,
            -half_width + corner_radius,
            corner_radius,
            math.pi,
            math.pi * 1.5,
        )[1:]
    )
    profile.append((half_length - corner_radius, -half_width))
    profile.extend(
        _arc_points_2d(
            half_length - corner_radius,
            -half_width + corner_radius,
            corner_radius,
            math.pi * 1.5,
            math.pi * 2.0,
        )[1:]
    )
    profile.append((half_length, half_width - corner_radius))
    profile.extend(
        _arc_points_2d(
            half_length - corner_radius,
            half_width - corner_radius,
            corner_radius,
            0.0,
            math.pi * 0.5,
        )[1:]
    )
    profile.append((slot_right, half_width))
    profile.append((slot_right, slot_floor_y))
    profile.append((slot_left, slot_floor_y))
    return profile


def _blade_outline_profile(
    *,
    length: float,
    shoulder_length: float,
    shoulder_half_width: float,
    shank_half_width: float,
    tip_length: float,
) -> list[tuple[float, float]]:
    shoulder_mid = shoulder_length * 0.55
    shank_start = shoulder_length
    tip_start = length - tip_length
    return [
        (0.0, shoulder_half_width * 0.92),
        (shoulder_mid, shoulder_half_width),
        (shoulder_length, shoulder_half_width * 0.82),
        (shank_start + 0.006, shank_half_width),
        (tip_start, shank_half_width * 0.88),
        (length - tip_length * 0.42, shank_half_width * 0.52),
        (length, 0.0),
        (length - tip_length * 0.42, -shank_half_width * 0.52),
        (tip_start, -shank_half_width * 0.88),
        (shank_start + 0.006, -shank_half_width),
        (shoulder_length, -shoulder_half_width * 0.82),
        (shoulder_mid, -shoulder_half_width),
        (0.0, -shoulder_half_width * 0.92),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_key_fob")

    body_color = model.material("body_plastic", rgba=(0.13, 0.13, 0.14, 1.0))
    button_color = model.material("button_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    plate_color = model.material("rear_plate", rgba=(0.10, 0.10, 0.11, 1.0))
    hinge_cap_color = model.material("hinge_cap", rgba=(0.42, 0.44, 0.46, 1.0))
    blade_color = model.material("blade_steel", rgba=(0.79, 0.80, 0.82, 1.0))

    body_length = 0.068
    body_width = 0.038
    body_thickness = 0.013
    corner_radius = 0.008

    slot_left = -0.0215
    slot_right = 0.0230
    slot_depth = 0.0105
    pivot_x = slot_right
    pivot_y = body_width * 0.5 - slot_depth * 0.5

    outer_profile = _fob_outline_profile(
        length=body_length,
        width=body_width,
        corner_radius=corner_radius,
        slot_left=slot_left,
        slot_right=slot_right,
        slot_depth=slot_depth,
    )

    plate_width = 0.031
    plate_height = 0.021
    plate_radius = 0.0035
    plate_thickness = 0.0014
    plate_press = 0.0011
    plate_center = (-0.004, -0.002)
    rear_pocket_profile = [
        (x + plate_center[0], y + plate_center[1])
        for x, y in rounded_rect_profile(
            plate_width + 0.003,
            plate_height + 0.003,
            plate_radius + 0.001,
            corner_segments=6,
        )
    ]

    front_core_thickness = 0.0103
    front_core_center_z = body_thickness * 0.5 - front_core_thickness * 0.5
    rear_frame_thickness = 0.0029
    rear_frame_center_z = -body_thickness * 0.5 + rear_frame_thickness * 0.5

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(outer_profile, front_core_thickness, cap=True, center=True),
            "key_fob_front_core",
        ),
        origin=Origin(xyz=(0.0, 0.0, front_core_center_z)),
        material=body_color,
        name="body_core",
    )
    body.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                outer_profile,
                [rear_pocket_profile],
                rear_frame_thickness,
                cap=True,
                center=True,
            ),
            "key_fob_rear_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0, rear_frame_center_z)),
        material=body_color,
        name="rear_frame",
    )
    body.visual(
        Cylinder(radius=0.0054, length=0.0022),
        origin=Origin(xyz=(pivot_x, pivot_y, body_thickness * 0.5 - 0.0011)),
        material=hinge_cap_color,
        name="hinge_front_cap",
    )
    body.visual(
        Cylinder(radius=0.0054, length=0.0022),
        origin=Origin(xyz=(pivot_x, pivot_y, -body_thickness * 0.5 + 0.0011)),
        material=hinge_cap_color,
        name="hinge_rear_cap",
    )
    body.visual(
        Box((0.014, 0.010, 0.0012)),
        origin=Origin(xyz=(-0.010, 0.002, body_thickness * 0.5 - 0.0005)),
        material=button_color,
        name="unlock_button",
    )
    body.visual(
        Box((0.014, 0.010, 0.0012)),
        origin=Origin(xyz=(-0.010, -0.011, body_thickness * 0.5 - 0.0005)),
        material=button_color,
        name="lock_button",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_thickness)),
        mass=0.09,
    )

    blade_length = 0.043
    blade_thickness = 0.0024
    blade = model.part("key_blade")
    blade.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                _blade_outline_profile(
                    length=blade_length,
                    shoulder_length=0.010,
                    shoulder_half_width=0.0047,
                    shank_half_width=0.0039,
                    tip_length=0.009,
                ),
                blade_thickness,
                cap=True,
                center=True,
            ),
            "flip_key_blade",
        ),
        material=blade_color,
        name="blade_panel",
    )
    blade.inertial = Inertial.from_geometry(
        Box((blade_length, 0.010, blade_thickness)),
        mass=0.024,
        origin=Origin(xyz=(blade_length * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_key_blade",
        ArticulationType.REVOLUTE,
        parent=body,
        child=blade,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.7,
            velocity=8.0,
            lower=0.0,
            upper=1.70,
        ),
    )

    battery_plate = model.part("battery_plate")
    battery_plate.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(
                    plate_width,
                    plate_height,
                    plate_radius,
                    corner_segments=6,
                ),
                plate_thickness,
                cap=True,
                closed=True,
            ),
            "battery_plate_panel",
        ),
        material=plate_color,
        name="plate_panel",
    )
    battery_plate.inertial = Inertial.from_geometry(
        Box((plate_width, plate_height, plate_thickness)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
    )

    model.articulation(
        "body_to_battery_plate",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_plate,
        origin=Origin(xyz=(plate_center[0], plate_center[1], -body_thickness * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.03,
            lower=0.0,
            upper=plate_press,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    blade = object_model.get_part("key_blade")
    battery_plate = object_model.get_part("battery_plate")
    blade_joint = object_model.get_articulation("body_to_key_blade")
    plate_joint = object_model.get_articulation("body_to_battery_plate")

    blade_open = blade_joint.motion_limits.upper if blade_joint.motion_limits is not None else 2.95
    plate_press = plate_joint.motion_limits.upper if plate_joint.motion_limits is not None else 0.0011

    with ctx.pose({blade_joint: 0.0, plate_joint: 0.0}):
        ctx.expect_within(
            blade,
            body,
            axes="xy",
            inner_elem="blade_panel",
            outer_elem="body_core",
            margin=0.0005,
            name="folded blade stays within the fob footprint",
        )
        ctx.expect_within(
            battery_plate,
            body,
            axes="xy",
            inner_elem="plate_panel",
            outer_elem="rear_frame",
            margin=0.0005,
            name="battery plate sits within the rear opening footprint",
        )

    closed_blade_box = ctx.part_element_world_aabb(blade, elem="blade_panel")
    body_box = ctx.part_world_aabb(body)
    with ctx.pose({blade_joint: blade_open}):
        open_blade_box = ctx.part_element_world_aabb(blade, elem="blade_panel")

    ctx.check(
        "blade swings out from the top edge",
        closed_blade_box is not None
        and open_blade_box is not None
        and body_box is not None
        and closed_blade_box[1][1] <= body_box[1][1] + 0.0005
        and open_blade_box[1][1] > body_box[1][1] + 0.020,
        details=f"closed_blade_box={closed_blade_box}, open_blade_box={open_blade_box}, body_box={body_box}",
    )

    rest_plate_pos = ctx.part_world_position(battery_plate)
    with ctx.pose({plate_joint: plate_press}):
        pressed_plate_pos = ctx.part_world_position(battery_plate)
        ctx.expect_within(
            battery_plate,
            body,
            axes="xy",
            inner_elem="plate_panel",
            outer_elem="rear_frame",
            margin=0.0005,
            name="pressed battery plate stays guided by the rear opening",
        )

    ctx.check(
        "battery plate presses inward toward the body",
        rest_plate_pos is not None
        and pressed_plate_pos is not None
        and pressed_plate_pos[2] > rest_plate_pos[2] + 0.0009,
        details=f"rest_plate_pos={rest_plate_pos}, pressed_plate_pos={pressed_plate_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
