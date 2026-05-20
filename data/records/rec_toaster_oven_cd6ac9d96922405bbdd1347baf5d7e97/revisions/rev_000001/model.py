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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_toaster_oven")

    body_w = 0.62
    body_d = 0.48
    body_h = 0.37
    shell_t = 0.014
    foot_h = 0.03
    top_cap_t = 0.018
    top_overhang_x = 0.018
    top_overhang_y = 0.022

    control_w = 0.16
    chamber_x_min = -body_w / 2.0 + shell_t
    control_x_min = body_w / 2.0 - control_w
    chamber_x_max = control_x_min - shell_t
    chamber_center_x = (chamber_x_min + chamber_x_max) / 2.0
    chamber_w = chamber_x_max - chamber_x_min
    divider_x = control_x_min - shell_t / 2.0
    control_center_x = (control_x_min + (body_w / 2.0 - shell_t)) / 2.0

    opening_w = 0.40
    opening_h = 0.228
    opening_z0 = foot_h + 0.048
    opening_z1 = opening_z0 + opening_h
    opening_left = chamber_center_x - opening_w / 2.0
    opening_right = chamber_center_x + opening_w / 2.0

    door_w = 0.414
    door_h = 0.252
    door_t = 0.028
    hinge_r = 0.010
    hinge_y = -0.244
    hinge_z = opening_z0 - hinge_r

    gland_len = 0.014
    knob_origin_y = -0.234 - gland_len
    upper_knob_z = foot_h + 0.255
    lower_knob_z = foot_h + 0.155

    stainless = model.material("stainless_body", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.22, 0.24, 0.26, 1.0))
    gasket = model.material("silicone_gasket", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_mat = model.material("knob_polymer", rgba=(0.14, 0.15, 0.16, 1.0))
    accent = model.material("weather_seal_metal", rgba=(0.55, 0.57, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + shell_t / 2.0)),
        material=stainless,
        name="floor_pan",
    )
    body.visual(
        Box((shell_t, body_d, body_h - 2.0 * shell_t)),
        origin=Origin(xyz=(-body_w / 2.0 + shell_t / 2.0, 0.0, foot_h + body_h / 2.0)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((shell_t, body_d, body_h - 2.0 * shell_t)),
        origin=Origin(xyz=(body_w / 2.0 - shell_t / 2.0, 0.0, foot_h + body_h / 2.0)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((shell_t, body_d, body_h - 2.0 * shell_t)),
        origin=Origin(xyz=(divider_x, 0.0, foot_h + body_h / 2.0)),
        material=stainless,
        name="control_divider",
    )
    body.visual(
        Box((body_w, shell_t, body_h - 2.0 * shell_t)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - shell_t / 2.0, foot_h + body_h / 2.0)),
        material=stainless,
        name="back_wall",
    )
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + body_h - shell_t / 2.0)),
        material=stainless,
        name="roof_pan",
    )
    body.visual(
        Box((body_w + 2.0 * top_overhang_x, body_d + 2.0 * top_overhang_y, top_cap_t)),
        origin=Origin(xyz=(0.0, 0.004, foot_h + body_h + top_cap_t / 2.0)),
        material=accent,
        name="top_cap",
    )

    body.visual(
        Box((chamber_w, 0.030, 0.024)),
        origin=Origin(xyz=(chamber_center_x, -0.188, opening_z0 - 0.010)),
        material=stainless,
        name="threshold_frame",
    )
    body.visual(
        Box((chamber_w, 0.030, 0.024)),
        origin=Origin(xyz=(chamber_center_x, -0.188, opening_z1 + 0.012)),
        material=stainless,
        name="lintel_frame",
    )
    body.visual(
        Box((control_w - 2.0 * shell_t, 0.030, body_h - 2.0 * shell_t)),
        origin=Origin(xyz=(control_center_x, -0.219, foot_h + body_h / 2.0)),
        material=dark_panel,
        name="control_face",
    )
    body.visual(
        Box((control_w - 0.012, 0.060, 0.014)),
        origin=Origin(xyz=(control_center_x, -0.255, foot_h + 0.305)),
        material=accent,
        name="control_hood_top",
    )
    body.visual(
        Box((0.014, 0.050, 0.060)),
        origin=Origin(xyz=(control_center_x - 0.068, -0.250, foot_h + 0.278)),
        material=accent,
        name="control_hood_left_cheek",
    )
    body.visual(
        Box((0.014, 0.050, 0.060)),
        origin=Origin(xyz=(control_center_x + 0.068, -0.250, foot_h + 0.278)),
        material=accent,
        name="control_hood_right_cheek",
    )

    body.visual(
        Cylinder(radius=hinge_r, length=door_w + 0.046),
        origin=Origin(xyz=(chamber_center_x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=accent,
        name="hinge_barrel",
    )

    gasket_y_center = -0.237
    body.visual(
        Box((0.016, 0.006, opening_h)),
        origin=Origin(
            xyz=((opening_left + chamber_x_min) / 2.0, gasket_y_center, opening_z0 + opening_h / 2.0)
        ),
        material=gasket,
        name="door_gasket_left",
    )
    body.visual(
        Box((0.016, 0.006, opening_h)),
        origin=Origin(
            xyz=((opening_right + chamber_x_max) / 2.0, gasket_y_center, opening_z0 + opening_h / 2.0)
        ),
        material=gasket,
        name="door_gasket_right",
    )
    body.visual(
        Box((opening_w + 0.032, 0.006, 0.012)),
        origin=Origin(xyz=(chamber_center_x, gasket_y_center, opening_z1 + 0.006)),
        material=gasket,
        name="door_gasket_top",
    )
    body.visual(
        Box((opening_w + 0.032, 0.006, 0.012)),
        origin=Origin(xyz=(chamber_center_x, gasket_y_center, opening_z0 - 0.006)),
        material=gasket,
        name="door_gasket_bottom",
    )

    body.visual(
        Cylinder(radius=0.020, length=gland_len),
        origin=Origin(
            xyz=(control_center_x, -0.234 - gland_len / 2.0, upper_knob_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=accent,
        name="temp_gland",
    )
    body.visual(
        Cylinder(radius=0.020, length=gland_len),
        origin=Origin(
            xyz=(control_center_x, -0.234 - gland_len / 2.0, lower_knob_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=accent,
        name="timer_gland",
    )

    foot_radius = 0.016
    for suffix, x_pos, y_pos in (
        ("front_left", -0.245, -0.180),
        ("front_right", 0.245, -0.180),
        ("rear_left", -0.245, 0.180),
        ("rear_right", 0.245, 0.180),
    ):
        body.visual(
            Cylinder(radius=foot_radius, length=foot_h),
            origin=Origin(xyz=(x_pos, y_pos, foot_h / 2.0)),
            material=dark_panel,
            name=f"foot_{suffix}",
        )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, hinge_r + door_t / 2.0, hinge_r + door_h / 2.0)),
        material=stainless,
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.080, 0.008, door_h - 0.090)),
        origin=Origin(xyz=(0.0, hinge_r + 0.010, hinge_r + door_h / 2.0)),
        material=accent,
        name="door_outer_stiffener",
    )
    handle_z = hinge_r + door_h * 0.68
    handle_y = -0.020
    post_y = -0.002
    for suffix, x_pos in (("left", -0.128), ("right", 0.128)):
        door.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(xyz=(x_pos, post_y, handle_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_panel,
            name=f"handle_post_{suffix}",
        )
    door.visual(
        Cylinder(radius=0.008, length=0.290),
        origin=Origin(xyz=(0.0, handle_y, handle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_panel,
        name="handle_grip",
    )

    def build_knob(part_name: str, pointer_name: str) -> None:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.021, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="knob_collar",
        )
        knob.visual(
            Cylinder(radius=0.029, length=0.026),
            origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_mat,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.033, length=0.008),
            origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_mat,
            name="knob_face",
        )
        knob.visual(
            Cylinder(radius=0.008, length=0.020),
            origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="shaft",
        )
        knob.visual(
            Box((0.004, 0.002, 0.012)),
            origin=Origin(xyz=(0.0, -0.031, 0.011)),
            material=accent,
            name=pointer_name,
        )

    build_knob("temperature_knob", "temp_pointer")
    build_knob("timer_knob", "timer_pointer")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(chamber_center_x, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_temperature_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child="temperature_knob",
        origin=Origin(xyz=(control_center_x, knob_origin_y, upper_knob_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=4.6),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child="timer_knob",
        origin=Origin(xyz=(control_center_x, knob_origin_y, lower_knob_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=5.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    temp_knob = object_model.get_part("temperature_knob")
    timer_knob = object_model.get_part("timer_knob")
    door_hinge = object_model.get_articulation("body_to_door")
    temp_joint = object_model.get_articulation("body_to_temperature_knob")
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
        elem_a="door_panel",
        elem_b="door_gasket_top",
        name="door_contacts_top_gasket",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="door_panel",
        elem_b="door_gasket_left",
        name="door_contacts_side_gasket",
    )
    ctx.expect_contact(
        temp_knob,
        body,
        elem_a="knob_collar",
        elem_b="temp_gland",
        name="temperature_knob_is_shaft_supported",
    )
    ctx.expect_contact(
        timer_knob,
        body,
        elem_a="knob_collar",
        elem_b="timer_gland",
        name="timer_knob_is_shaft_supported",
    )

    ctx.check(
        "articulation_axes_are_practical",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(temp_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(timer_joint.axis) == (0.0, -1.0, 0.0),
        details=(
            f"door axis={door_hinge.axis}, "
            f"temperature axis={temp_joint.axis}, timer axis={timer_joint.axis}"
        ),
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.15}):
        open_aabb = ctx.part_world_aabb(door)
        if closed_aabb is None or open_aabb is None:
            ctx.fail("door_open_pose_is_measurable", "door AABB unavailable in one or both poses")
        else:
            ctx.check(
                "door_opens_downward_and_outward",
                open_aabb[1][2] < closed_aabb[1][2] - 0.12
                and open_aabb[0][1] < closed_aabb[0][1] - 0.12,
                details=(
                    f"closed_aabb={closed_aabb}, open_aabb={open_aabb}; "
                    "expected open door to drop in z and project farther forward"
                ),
            )

    with ctx.pose({temp_joint: 2.3, timer_joint: 3.4}):
        ctx.expect_contact(
            temp_knob,
            body,
            elem_a="knob_collar",
            elem_b="temp_gland",
            name="temperature_knob_remains_seated_when_rotated",
        )
        ctx.expect_contact(
            timer_knob,
            body,
            elem_a="knob_collar",
            elem_b="timer_gland",
            name="timer_knob_remains_seated_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
