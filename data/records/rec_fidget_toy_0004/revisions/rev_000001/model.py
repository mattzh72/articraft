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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_SIZE = 0.036
BODY_HALF = BODY_SIZE * 0.5


def _rounded_rect_section(width: float, height: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, height, radius, corner_segments=8)]


def _build_cube_shell_mesh():
    return section_loft(
        [
            _rounded_rect_section(0.030, 0.030, 0.0042, -0.018),
            _rounded_rect_section(0.034, 0.034, 0.0046, -0.013),
            _rounded_rect_section(0.036, 0.036, 0.0050, 0.000),
            _rounded_rect_section(0.034, 0.034, 0.0046, 0.013),
            _rounded_rect_section(0.030, 0.030, 0.0042, 0.018),
        ]
    )


def _aabb_center(aabb):
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_cube")

    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.91, 1.0))
    shell_trim = model.material("shell_trim", rgba=(0.19, 0.20, 0.23, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    accent_red = model.material("accent_red", rgba=(0.82, 0.18, 0.16, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.18, 0.42, 0.74, 1.0))

    body = model.part("cube_body")
    body.visual(
        mesh_from_geometry(_build_cube_shell_mesh(), "fidget_cube_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.0078, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, BODY_HALF + 0.0010)),
        material=shell_trim,
        name="joystick_base",
    )
    body.visual(
        Box((0.0100, 0.0020, 0.0080)),
        origin=Origin(xyz=(0.0, -0.0050, BODY_HALF + 0.0040)),
        material=shell_trim,
        name="joystick_rear_cheek",
    )
    body.visual(
        Box((0.0100, 0.0020, 0.0080)),
        origin=Origin(xyz=(0.0, 0.0050, BODY_HALF + 0.0040)),
        material=shell_trim,
        name="joystick_front_cheek",
    )
    body.visual(
        Box((0.0030, 0.0040, 0.0060)),
        origin=Origin(xyz=(BODY_HALF + 0.0005, -0.0110, 0.0000)),
        material=shell_trim,
        name="dial_rear_bezel",
    )
    body.visual(
        Box((0.0030, 0.0040, 0.0060)),
        origin=Origin(xyz=(BODY_HALF + 0.0005, 0.0110, 0.0000)),
        material=shell_trim,
        name="dial_front_bezel",
    )
    body.visual(
        Box((0.0030, 0.0140, 0.0040)),
        origin=Origin(xyz=(BODY_HALF + 0.0005, 0.0000, -0.0110)),
        material=shell_trim,
        name="dial_lower_bezel",
    )
    body.visual(
        Box((0.0030, 0.0140, 0.0040)),
        origin=Origin(xyz=(BODY_HALF + 0.0005, 0.0000, 0.0110)),
        material=shell_trim,
        name="dial_upper_bezel",
    )
    body.visual(
        Box((0.0030, 0.0030, 0.0140)),
        origin=Origin(xyz=(-0.0060, BODY_HALF + 0.0005, 0.0000)),
        material=shell_trim,
        name="toggle_left_guard",
    )
    body.visual(
        Box((0.0030, 0.0030, 0.0140)),
        origin=Origin(xyz=(0.0060, BODY_HALF + 0.0005, 0.0000)),
        material=shell_trim,
        name="toggle_right_guard",
    )
    body.visual(
        Box((0.0030, 0.0040, 0.0240)),
        origin=Origin(xyz=(-BODY_HALF - 0.0005, -0.0070, 0.0000)),
        material=shell_trim,
        name="slider_lower_rail",
    )
    body.visual(
        Box((0.0030, 0.0040, 0.0240)),
        origin=Origin(xyz=(-BODY_HALF - 0.0005, 0.0070, 0.0000)),
        material=shell_trim,
        name="slider_upper_rail",
    )
    body.visual(
        Box((0.0030, 0.0180, 0.0030)),
        origin=Origin(xyz=(-BODY_HALF - 0.0005, 0.0000, -0.0120)),
        material=shell_trim,
        name="slider_bottom_stop",
    )
    body.visual(
        Box((0.0030, 0.0180, 0.0030)),
        origin=Origin(xyz=(-BODY_HALF - 0.0005, 0.0000, 0.0120)),
        material=shell_trim,
        name="slider_top_stop",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_SIZE, BODY_SIZE, BODY_SIZE)),
        mass=0.060,
    )

    joystick = model.part("joystick_nub")
    joystick.visual(
        Cylinder(radius=0.0012, length=0.0080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="joystick_axle",
    )
    joystick.visual(
        Cylinder(radius=0.0013, length=0.0130),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=control_black,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
        material=accent_red,
        name="joystick_cap",
    )
    joystick.inertial = Inertial.from_geometry(
        Box((0.010, 0.010, 0.024)),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "body_to_joystick",
        ArticulationType.REVOLUTE,
        parent=body,
        child=joystick,
        origin=Origin(xyz=(0.0, 0.0, BODY_HALF + 0.0040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-0.42,
            upper=0.42,
        ),
    )

    dial = model.part("dial_wheel")
    dial.visual(
        Cylinder(radius=0.0090, length=0.0040),
        origin=Origin(xyz=(0.0020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_black,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.0030, length=0.0016),
        origin=Origin(xyz=(0.0008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_trim,
        name="dial_hub",
    )
    dial.visual(
        Box((0.0020, 0.0025, 0.0040)),
        origin=Origin(xyz=(0.0030, 0.0, 0.0075)),
        material=accent_blue,
        name="dial_marker",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.008, 0.020, 0.020)),
        mass=0.006,
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_HALF, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=8.0,
        ),
    )

    toggle = model.part("toggle_switch")
    toggle.visual(
        Cylinder(radius=0.0016, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0016, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_black,
        name="toggle_hinge",
    )
    toggle.visual(
        Box((0.0032, 0.0060, 0.0100)),
        origin=Origin(xyz=(0.0, 0.0050, 0.0)),
        material=control_black,
        name="toggle_stem",
    )
    toggle.visual(
        Box((0.0050, 0.0040, 0.0070)),
        origin=Origin(xyz=(0.0, 0.0085, 0.0)),
        material=accent_red,
        name="toggle_tip",
    )
    toggle.inertial = Inertial.from_geometry(
        Box((0.010, 0.014, 0.012)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0060, 0.0)),
    )
    model.articulation(
        "body_to_toggle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=toggle,
        origin=Origin(xyz=(0.0, BODY_HALF, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=3.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    slider = model.part("slider_control")
    slider.visual(
        Box((0.0030, 0.0100, 0.0080)),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
        material=control_black,
        name="slider_carriage",
    )
    slider.visual(
        Box((0.0060, 0.0120, 0.0060)),
        origin=Origin(xyz=(-0.0060, 0.0, 0.0)),
        material=accent_blue,
        name="slider_cap",
    )
    slider.visual(
        Box((0.0010, 0.0080, 0.0012)),
        origin=Origin(xyz=(-0.0090, 0.0, -0.0018)),
        material=shell_white,
        name="slider_grip_low",
    )
    slider.visual(
        Box((0.0010, 0.0080, 0.0012)),
        origin=Origin(xyz=(-0.0090, 0.0, 0.0018)),
        material=shell_white,
        name="slider_grip_high",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.010, 0.014, 0.010)),
        mass=0.004,
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(-BODY_HALF, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.05,
            lower=-0.007,
            upper=0.007,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cube_body")
    joystick = object_model.get_part("joystick_nub")
    dial = object_model.get_part("dial_wheel")
    toggle = object_model.get_part("toggle_switch")
    slider = object_model.get_part("slider_control")

    joystick_joint = object_model.get_articulation("body_to_joystick")
    dial_joint = object_model.get_articulation("body_to_dial")
    toggle_joint = object_model.get_articulation("body_to_toggle")
    slider_joint = object_model.get_articulation("body_to_slider")

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
        "joint_types_and_axes",
        joystick_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(joystick_joint.axis) == (0.0, 1.0, 0.0)
        and dial_joint.joint_type == ArticulationType.CONTINUOUS
        and tuple(dial_joint.axis) == (1.0, 0.0, 0.0)
        and toggle_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(toggle_joint.axis) == (1.0, 0.0, 0.0)
        and slider_joint.joint_type == ArticulationType.PRISMATIC
        and tuple(slider_joint.axis) == (0.0, 0.0, 1.0),
        details="Each face control should use its intended articulation axis and joint type.",
    )

    ctx.expect_contact(joystick, body, name="joystick_contacts_body")
    ctx.expect_contact(dial, body, name="dial_contacts_body")
    ctx.expect_contact(toggle, body, name="toggle_contacts_body")
    ctx.expect_contact(slider, body, name="slider_contacts_body")

    joystick_rest_aabb = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
    dial_rest_aabb = ctx.part_element_world_aabb(dial, elem="dial_marker")
    toggle_rest_aabb = ctx.part_element_world_aabb(toggle, elem="toggle_tip")
    slider_rest_pos = ctx.part_world_position(slider)

    ctx.check(
        "widget_measurements_available",
        joystick_rest_aabb is not None and dial_rest_aabb is not None and toggle_rest_aabb is not None and slider_rest_pos is not None,
        details="Named widget visuals should resolve for pose tests.",
    )

    if joystick_rest_aabb is not None:
        joystick_rest_center = _aabb_center(joystick_rest_aabb)
        with ctx.pose({joystick_joint: 0.35}):
            ctx.expect_contact(joystick, body, name="joystick_contact_at_tilt")
            joystick_tilt_aabb = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
            if joystick_tilt_aabb is not None:
                joystick_tilt_center = _aabb_center(joystick_tilt_aabb)
                ctx.check(
                    "joystick_tilts_sideways",
                    joystick_tilt_center[0] > joystick_rest_center[0] + 0.004
                    and joystick_tilt_center[2] < joystick_rest_center[2] - 0.0007,
                    details="Joystick cap should sweep sideways and slightly downward when tilted.",
                )

    if dial_rest_aabb is not None:
        dial_rest_center = _aabb_center(dial_rest_aabb)
        with ctx.pose({dial_joint: math.pi / 2.0}):
            ctx.expect_contact(dial, body, name="dial_contact_while_rotating")
            dial_turn_aabb = ctx.part_element_world_aabb(dial, elem="dial_marker")
            if dial_turn_aabb is not None:
                dial_turn_center = _aabb_center(dial_turn_aabb)
                ctx.check(
                    "dial_marker_orbits",
                    abs(dial_turn_center[1] - dial_rest_center[1]) > 0.006
                    and abs(dial_turn_center[2] - dial_rest_center[2]) > 0.006,
                    details="The wheel marker should travel around the dial face when rotated.",
                )

    if toggle_rest_aabb is not None:
        toggle_rest_center = _aabb_center(toggle_rest_aabb)
        with ctx.pose({toggle_joint: 0.45}):
            ctx.expect_contact(toggle, body, name="toggle_contact_when_flipped")
            toggle_flip_aabb = ctx.part_element_world_aabb(toggle, elem="toggle_tip")
            if toggle_flip_aabb is not None:
                toggle_flip_center = _aabb_center(toggle_flip_aabb)
                ctx.check(
                    "toggle_switch_flips_up",
                    toggle_flip_center[2] > toggle_rest_center[2] + 0.003,
                    details="The toggle tip should rise when the switch is flipped.",
                )

    if slider_rest_pos is not None:
        with ctx.pose({slider_joint: 0.007}):
            ctx.expect_contact(slider, body, name="slider_contact_at_upper_travel")
            slider_upper_pos = ctx.part_world_position(slider)
            if slider_upper_pos is not None:
                ctx.check(
                    "slider_translates_along_track",
                    slider_upper_pos[2] > slider_rest_pos[2] + 0.0065,
                    details="The slider should translate vertically along its guide face.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
