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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_microwave_oven")

    housing = model.material("housing_cream", rgba=(0.84, 0.79, 0.69, 1.0))
    panel_chrome = model.material("panel_chrome", rgba=(0.76, 0.76, 0.74, 1.0))
    cavity_steel = model.material("cavity_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.22, 0.25, 0.35))
    bakelite = model.material("bakelite_brown", rgba=(0.25, 0.19, 0.14, 1.0))
    brass = model.material("warm_brass", rgba=(0.78, 0.64, 0.34, 1.0))
    ivory = model.material("ivory_button", rgba=(0.92, 0.90, 0.84, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.11, 0.11, 0.11, 1.0))
    platter_glass = model.material("platter_glass", rgba=(0.74, 0.83, 0.90, 0.45))

    body_w = 0.545
    body_d = 0.420
    body_h = 0.335
    bezel_t = 0.018
    shell_t = 0.014
    base_t = 0.016
    front_face_y = -body_d * 0.5
    bezel_center_y = front_face_y + bezel_t * 0.5
    inner_front_y = front_face_y + bezel_t
    back_face_y = body_d * 0.5
    body_top_z = body_h

    door_w = 0.392
    door_h = 0.286
    door_t = 0.026
    door_left_x = -0.252
    door_bottom_z = 0.024
    door_window_w = 0.296
    door_window_h = 0.196

    opening_w = 0.346
    opening_h = 0.226
    opening_left_x = -0.226
    opening_bottom_z = 0.050
    cavity_x = opening_left_x + opening_w * 0.5
    cavity_front_y = inner_front_y
    cavity_d = 0.292
    cavity_center_y = cavity_front_y + cavity_d * 0.5
    cavity_bottom_z = opening_bottom_z
    cavity_floor_top = cavity_bottom_z + 0.010
    cavity_h = opening_h
    cavity_roof_z = cavity_bottom_z + cavity_h

    button_x = 0.205
    button_z = 0.068
    button_travel = 0.012

    opening_right_x = opening_left_x + opening_w
    left_jamb_w = opening_left_x + body_w * 0.5
    top_bar_h = body_h - (opening_bottom_z + opening_h)
    bottom_bar_h = opening_bottom_z
    button_slot_w = 0.028
    button_slot_h = 0.054
    right_panel_left_w = button_x - button_slot_w * 0.5 - opening_right_x
    right_panel_right_w = body_w * 0.5 - (button_x + button_slot_w * 0.5)
    button_slot_bottom_h = button_z - button_slot_h * 0.5
    button_slot_top_h = body_h - (button_z + button_slot_h * 0.5)

    door_outer = rounded_rect_profile(door_w, door_h, 0.036, corner_segments=8)
    door_window_profile = rounded_rect_profile(door_window_w, door_window_h, 0.022, corner_segments=8)
    door_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            door_outer,
            [door_window_profile],
            height=door_t,
            center=True,
        ).rotate_x(math.pi / 2.0),
        "microwave_door_frame",
    )
    trim_outer = rounded_rect_profile(door_window_w + 0.040, door_window_h + 0.028, 0.025, corner_segments=8)
    trim_inner = rounded_rect_profile(door_window_w - 0.006, door_window_h - 0.006, 0.020, corner_segments=8)
    door_trim_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            trim_outer,
            [trim_inner],
            height=0.004,
            center=True,
        ).rotate_x(math.pi / 2.0),
        "microwave_door_trim",
    )

    chassis = model.part("chassis")
    chassis.visual(
        Box((left_jamb_w, bezel_t, body_h)),
        origin=Origin(
            xyz=(-body_w * 0.5 + left_jamb_w * 0.5, bezel_center_y, body_h * 0.5),
        ),
        material=housing,
        name="left_front_bezel",
    )
    chassis.visual(
        Box((opening_w, bezel_t, top_bar_h)),
        origin=Origin(
            xyz=(
                cavity_x,
                bezel_center_y,
                opening_bottom_z + opening_h + top_bar_h * 0.5,
            )
        ),
        material=housing,
        name="top_front_bezel",
    )
    chassis.visual(
        Box((opening_w, bezel_t, bottom_bar_h)),
        origin=Origin(
            xyz=(cavity_x, bezel_center_y, bottom_bar_h * 0.5),
        ),
        material=housing,
        name="bottom_front_bezel",
    )
    chassis.visual(
        Box((right_panel_left_w, bezel_t, body_h)),
        origin=Origin(
            xyz=(
                opening_right_x + right_panel_left_w * 0.5,
                bezel_center_y,
                body_h * 0.5,
            )
        ),
        material=housing,
        name="right_panel_left",
    )
    chassis.visual(
        Box((right_panel_right_w, bezel_t, body_h)),
        origin=Origin(
            xyz=(
                button_x + button_slot_w * 0.5 + right_panel_right_w * 0.5,
                bezel_center_y,
                body_h * 0.5,
            )
        ),
        material=housing,
        name="right_panel_right",
    )
    chassis.visual(
        Box((body_w * 0.5 - opening_right_x, bezel_t, button_slot_top_h)),
        origin=Origin(
            xyz=(
                opening_right_x + (body_w * 0.5 - opening_right_x) * 0.5,
                bezel_center_y,
                button_z + button_slot_h * 0.5 + button_slot_top_h * 0.5,
            )
        ),
        material=housing,
        name="button_slot_top_bezel",
    )
    chassis.visual(
        Box((body_w * 0.5 - opening_right_x, bezel_t, button_slot_bottom_h)),
        origin=Origin(
            xyz=(
                opening_right_x + (body_w * 0.5 - opening_right_x) * 0.5,
                bezel_center_y,
                button_slot_bottom_h * 0.5,
            )
        ),
        material=housing,
        name="button_slot_bottom_bezel",
    )
    chassis.visual(
        Box((body_w, body_d - bezel_t, base_t)),
        origin=Origin(xyz=(0.0, (back_face_y + inner_front_y) * 0.5, base_t * 0.5)),
        material=housing,
        name="base_tray",
    )
    chassis.visual(
        Box((body_w - 2.0 * shell_t, shell_t, body_h - 2.0 * base_t)),
        origin=Origin(
            xyz=(
                0.0,
                back_face_y - shell_t * 0.5,
                base_t + (body_h - 2.0 * base_t) * 0.5,
            )
        ),
        material=housing,
        name="rear_shell",
    )
    chassis.visual(
        Box((shell_t, body_d - bezel_t, body_h - base_t)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + shell_t * 0.5,
                (back_face_y + inner_front_y) * 0.5,
                base_t + (body_h - base_t) * 0.5,
            )
        ),
        material=housing,
        name="left_shell",
    )
    chassis.visual(
        Box((shell_t, body_d - bezel_t, body_h - base_t)),
        origin=Origin(
            xyz=(
                body_w * 0.5 - shell_t * 0.5,
                (back_face_y + inner_front_y) * 0.5,
                base_t + (body_h - base_t) * 0.5,
            )
        ),
        material=housing,
        name="right_shell",
    )
    chassis.visual(
        Box((body_w - 2.0 * shell_t, body_d - bezel_t, shell_t)),
        origin=Origin(
            xyz=(
                0.0,
                (back_face_y + inner_front_y) * 0.5,
                body_top_z - shell_t * 0.5,
            )
        ),
        material=housing,
        name="top_shell",
    )
    chassis.visual(
        Cylinder(radius=0.032, length=body_w - 0.060),
        origin=Origin(
            xyz=(0.0, front_face_y + 0.032, body_top_z - 0.032),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=housing,
        name="rounded_brow",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=body_h - 0.064),
        origin=Origin(
            xyz=(-body_w * 0.5 + 0.028, front_face_y + 0.028, body_h * 0.5),
        ),
        material=housing,
        name="left_front_corner",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=body_h - 0.064),
        origin=Origin(
            xyz=(body_w * 0.5 - 0.028, front_face_y + 0.028, body_h * 0.5),
        ),
        material=housing,
        name="right_front_corner",
    )
    chassis.visual(
        Box((0.014, 0.006, 0.248)),
        origin=Origin(xyz=(0.246, front_face_y + 0.003, 0.170)),
        material=panel_chrome,
        name="control_panel_trim",
    )
    chassis.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(
            xyz=(0.205, front_face_y + 0.002, 0.236),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=panel_chrome,
        name="timer_escutcheon",
    )
    chassis.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(
            xyz=(0.205, front_face_y + 0.002, 0.152),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=panel_chrome,
        name="power_escutcheon",
    )

    chassis.visual(
        Box((opening_w - 0.020, cavity_d, cavity_floor_top - base_t)),
        origin=Origin(
            xyz=(
                cavity_x,
                cavity_center_y,
                base_t + (cavity_floor_top - base_t) * 0.5,
            )
        ),
        material=cavity_steel,
        name="cavity_support_block",
    )
    chassis.visual(
        Box((opening_w - 0.020, cavity_d, 0.010)),
        origin=Origin(
            xyz=(cavity_x, cavity_center_y, cavity_floor_top - 0.005),
        ),
        material=cavity_steel,
        name="cavity_floor",
    )
    chassis.visual(
        Box((0.010, cavity_d, cavity_h)),
        origin=Origin(
            xyz=(
                cavity_x - (opening_w - 0.020) * 0.5 + 0.005,
                cavity_center_y,
                cavity_bottom_z + cavity_h * 0.5,
            )
        ),
        material=cavity_steel,
        name="cavity_left_wall",
    )
    chassis.visual(
        Box((0.010, cavity_d, cavity_h)),
        origin=Origin(
            xyz=(
                cavity_x + (opening_w - 0.020) * 0.5 - 0.005,
                cavity_center_y,
                cavity_bottom_z + cavity_h * 0.5,
            )
        ),
        material=cavity_steel,
        name="cavity_right_wall",
    )
    chassis.visual(
        Box((opening_w - 0.020, cavity_d, 0.010)),
        origin=Origin(
            xyz=(cavity_x, cavity_center_y, cavity_roof_z - 0.005),
        ),
        material=cavity_steel,
        name="cavity_roof",
    )
    chassis.visual(
        Box((opening_w - 0.020, 0.010, cavity_h)),
        origin=Origin(
            xyz=(
                cavity_x,
                cavity_front_y + cavity_d - 0.005,
                cavity_bottom_z + cavity_h * 0.5,
            )
        ),
        material=cavity_steel,
        name="cavity_back_wall",
    )
    chassis.visual(
        Box((0.018, 0.014, opening_h + 0.028)),
        origin=Origin(
            xyz=(opening_left_x - 0.009, inner_front_y + 0.007, opening_bottom_z + opening_h * 0.5),
        ),
        material=cavity_steel,
        name="left_inner_lip",
    )
    chassis.visual(
        Box((0.018, 0.014, opening_h + 0.028)),
        origin=Origin(
            xyz=(
                opening_left_x + opening_w + 0.009,
                inner_front_y + 0.007,
                opening_bottom_z + opening_h * 0.5,
            )
        ),
        material=cavity_steel,
        name="right_inner_lip",
    )
    chassis.visual(
        Box((opening_w + 0.036, 0.014, 0.018)),
        origin=Origin(
            xyz=(cavity_x, inner_front_y + 0.007, opening_bottom_z - 0.009),
        ),
        material=cavity_steel,
        name="bottom_inner_lip",
    )
    chassis.visual(
        Box((opening_w + 0.036, 0.014, 0.018)),
        origin=Origin(
            xyz=(cavity_x, inner_front_y + 0.007, opening_bottom_z + opening_h + 0.009),
        ),
        material=cavity_steel,
        name="top_inner_lip",
    )
    chassis.visual(
        Box((0.006, 0.020, 0.052)),
        origin=Origin(xyz=(button_x - 0.020, inner_front_y + 0.013, button_z)),
        material=cavity_steel,
        name="button_guide_left",
    )
    chassis.visual(
        Box((0.006, 0.020, 0.052)),
        origin=Origin(xyz=(button_x + 0.020, inner_front_y + 0.013, button_z)),
        material=cavity_steel,
        name="button_guide_right",
    )
    chassis.visual(
        Box((0.034, 0.026, 0.006)),
        origin=Origin(xyz=(button_x, inner_front_y + 0.013, button_z + 0.025)),
        material=cavity_steel,
        name="button_guide_top",
    )
    chassis.visual(
        Box((0.034, 0.026, 0.006)),
        origin=Origin(xyz=(button_x, inner_front_y + 0.013, button_z - 0.025)),
        material=cavity_steel,
        name="button_guide_bottom",
    )
    chassis.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(cavity_x, cavity_center_y, cavity_floor_top + 0.006)),
        material=cavity_steel,
        name="turntable_spindle",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            chassis.visual(
                Box((0.028, 0.028, 0.010)),
                origin=Origin(
                    xyz=(
                        x_sign * (body_w * 0.5 - 0.070),
                        y_sign * (body_d * 0.5 - 0.070),
                        0.005,
                    )
                ),
                material=foot_rubber,
                name=f"foot_{'r' if x_sign > 0 else 'l'}_{'b' if y_sign > 0 else 'f'}",
            )
    chassis.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    door = model.part("door")
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(door_w * 0.5, -door_t * 0.5, door_h * 0.5)),
        material=housing,
        name="door_frame",
    )
    door.visual(
        door_trim_mesh,
        origin=Origin(xyz=(door_w * 0.5, -door_t + 0.002, door_h * 0.5)),
        material=panel_chrome,
        name="window_trim",
    )
    door.visual(
        Box((door_window_w + 0.010, 0.006, door_window_h + 0.010)),
        origin=Origin(xyz=(door_w * 0.5, -door_t * 0.55, door_h * 0.5)),
        material=dark_glass,
        name="door_glass",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=2.4,
        origin=Origin(xyz=(door_w * 0.5, -door_t * 0.5, door_h * 0.5)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.124, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=platter_glass,
        name="glass_plate",
    )
    turntable.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cavity_steel,
        name="hub",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.124, length=0.014),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(
            xyz=(0.0, -0.013, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bakelite,
        name="timer_knob_body",
    )
    timer_knob.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(
            xyz=(0.0, -0.024, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="timer_knob_cap",
    )
    timer_knob.visual(
        Box((0.003, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.024, 0.012)),
        material=ivory,
        name="timer_indicator",
    )
    timer_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.026),
        mass=0.10,
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    power_knob = model.part("power_knob")
    power_knob.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(
            xyz=(0.0, -0.012, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bakelite,
        name="power_knob_body",
    )
    power_knob.visual(
        Cylinder(radius=0.0125, length=0.004),
        origin=Origin(
            xyz=(0.0, -0.022, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="power_knob_cap",
    )
    power_knob.visual(
        Box((0.003, 0.004, 0.009)),
        origin=Origin(xyz=(0.0, -0.022, 0.011)),
        material=ivory,
        name="power_indicator",
    )
    power_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.024),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.024, 0.010, 0.050)),
        origin=Origin(),
        material=ivory,
        name="button_cap",
    )
    release_button.visual(
        Box((0.014, 0.027, 0.038)),
        origin=Origin(xyz=(0.0, 0.0185, 0.0)),
        material=ivory,
        name="button_stem",
    )
    release_button.visual(
        Box((0.030, 0.006, 0.044)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=panel_chrome,
        name="button_keeper",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.030, 0.041, 0.050)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(door_left_x, front_face_y, door_bottom_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=turntable,
        origin=Origin(xyz=(cavity_x, cavity_center_y, cavity_floor_top + 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )
    model.articulation(
        "timer_knob_spin",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=timer_knob,
        origin=Origin(xyz=(0.205, front_face_y, 0.236)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=-math.radians(140.0),
            upper=math.radians(140.0),
        ),
    )
    model.articulation(
        "power_knob_spin",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=power_knob,
        origin=Origin(xyz=(0.205, front_face_y, 0.152)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "release_button_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=release_button,
        origin=Origin(xyz=(button_x, front_face_y - 0.003, button_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=button_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    timer_knob = object_model.get_part("timer_knob")
    power_knob = object_model.get_part("power_knob")
    release_button = object_model.get_part("release_button")

    door_hinge = object_model.get_articulation("door_hinge")
    turntable_spin = object_model.get_articulation("turntable_spin")
    timer_knob_spin = object_model.get_articulation("timer_knob_spin")
    power_knob_spin = object_model.get_articulation("power_knob_spin")
    release_button_slide = object_model.get_articulation("release_button_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(door, chassis, name="door_seats_against_front_bezel")
    ctx.expect_contact(turntable, chassis, name="turntable_rests_on_spindle")
    ctx.expect_contact(timer_knob, chassis, name="timer_knob_mount_contact")
    ctx.expect_contact(power_knob, chassis, name="power_knob_mount_contact")
    ctx.expect_contact(release_button, chassis, name="button_guided_contact_at_rest")
    ctx.expect_overlap(door, chassis, axes="xz", min_overlap=0.20, name="door_covers_front_opening")

    ctx.check(
        "door_hinge_axis_and_limits",
        door_hinge.axis == (0.0, 0.0, -1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > math.radians(100.0),
        details="Door hinge should rotate about a vertical side hinge axis with realistic opening travel.",
    )
    ctx.check(
        "turntable_continuous_axis",
        turntable_spin.axis == (0.0, 0.0, 1.0)
        and turntable_spin.motion_limits is not None
        and turntable_spin.motion_limits.lower is None
        and turntable_spin.motion_limits.upper is None,
        details="Turntable should spin continuously about the cavity center spindle.",
    )
    ctx.check(
        "control_axes",
        timer_knob_spin.axis == (0.0, 1.0, 0.0)
        and power_knob_spin.axis == (0.0, 1.0, 0.0)
        and release_button_slide.axis == (0.0, 1.0, 0.0),
        details="Knobs should rotate on front-to-back axes and the release button should translate inward.",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    door_rest_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    assert door_rest_aabb is not None
    with ctx.pose({door_hinge: math.radians(100.0)}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
        assert door_open_aabb is not None
        rest_center = _aabb_center(door_rest_aabb)
        open_center = _aabb_center(door_open_aabb)
        assert open_center[0] < rest_center[0] - 0.14
        assert open_center[1] < rest_center[1] - 0.14

    button_rest_pos = ctx.part_world_position(release_button)
    assert button_rest_pos is not None
    with ctx.pose({release_button_slide: 0.011}):
        button_press_pos = ctx.part_world_position(release_button)
        assert button_press_pos is not None
        assert button_press_pos[1] > button_rest_pos[1] + 0.009
        assert abs(button_press_pos[0] - button_rest_pos[0]) < 1e-6
        assert abs(button_press_pos[2] - button_rest_pos[2]) < 1e-6
        ctx.expect_contact(release_button, chassis, name="button_guided_contact_when_pressed")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
