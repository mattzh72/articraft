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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(x_pos: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def _half_shell_mesh():
    return section_loft(
        [
            _yz_section(-0.040, 0.058, 0.050, 0.010),
            _yz_section(0.000, 0.078, 0.064, 0.012),
            _yz_section(0.050, 0.074, 0.062, 0.010),
        ]
    )


def _cylindrical_shell(outer_radius: float, inner_radius: float, length: float, *, segments: int = 48):
    half = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rangefinder_binocular")

    armor_black = model.material("armor_black", rgba=(0.09, 0.10, 0.11, 1.0))
    bridge_gray = model.material("bridge_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    control_gray = model.material("control_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.45, 0.58, 0.68, 0.65))
    accent_orange = model.material("accent_orange", rgba=(0.86, 0.45, 0.14, 1.0))

    half_shell = mesh_from_geometry(_half_shell_mesh(), "binocular_half_shell")
    hinge_collar = mesh_from_geometry(_cylindrical_shell(0.018, 0.0125, 0.028), "hinge_collar")
    selector_shell = mesh_from_geometry(_cylindrical_shell(0.013, 0.0045, 0.010), "selector_shell")

    body_x = 0.020
    body_y = 0.071
    objective_x = 0.103
    objective_z = -0.006
    eyecup_x = -0.034
    bridge_arm_x = 0.002
    focus_center = (0.026, 0.0, 0.052)
    selector_center = (0.014, 0.070, 0.037)

    left_body = model.part("left_body")
    left_body.visual(
        half_shell,
        origin=Origin(xyz=(body_x, -body_y, 0.0)),
        material=armor_black,
        name="left_shell",
    )
    left_body.visual(
        Box((0.054, 0.030, 0.038)),
        origin=Origin(xyz=(bridge_arm_x, -0.033, -0.005)),
        material=bridge_gray,
        name="left_bridge_arm",
    )
    left_body.visual(
        Cylinder(radius=0.0125, length=0.028),
        origin=Origin(),
        material=bridge_gray,
        name="hinge_pin",
    )
    left_body.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=bridge_gray,
        name="hinge_tower",
    )
    left_body.visual(
        Box((0.020, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, -0.013, 0.023)),
        material=bridge_gray,
        name="hinge_stanchion",
    )
    left_body.visual(
        Box((0.058, 0.026, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.029)),
        material=bridge_gray,
        name="focus_bridge_deck",
    )
    left_body.visual(
        Box((0.008, 0.006, 0.022)),
        origin=Origin(xyz=(focus_center[0], -0.011, 0.043)),
        material=bridge_gray,
        name="focus_left_cheek",
    )
    left_body.visual(
        Box((0.008, 0.006, 0.022)),
        origin=Origin(xyz=(focus_center[0], 0.011, 0.043)),
        material=bridge_gray,
        name="focus_right_cheek",
    )
    left_body.visual(
        Cylinder(radius=0.004, length=0.005),
        origin=Origin(xyz=(focus_center[0], -0.0105, focus_center[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bridge_gray,
        name="focus_left_stub",
    )
    left_body.visual(
        Cylinder(radius=0.004, length=0.005),
        origin=Origin(xyz=(focus_center[0], 0.0105, focus_center[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bridge_gray,
        name="focus_right_stub",
    )
    left_body.visual(
        Cylinder(radius=0.024, length=0.066),
        origin=Origin(xyz=(objective_x, -body_y, objective_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor_black,
        name="left_objective_tube",
    )
    left_body.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.133, -body_y, objective_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bridge_gray,
        name="left_objective_bezel",
    )
    left_body.visual(
        Cylinder(radius=0.0215, length=0.003),
        origin=Origin(xyz=(0.1315, -body_y, objective_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="left_objective_lens",
    )
    left_body.visual(
        Cylinder(radius=0.019, length=0.028),
        origin=Origin(xyz=(eyecup_x, -0.069, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor_black,
        name="left_eyecup",
    )
    left_body.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(-0.046, -0.069, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bridge_gray,
        name="left_ocular",
    )
    left_body.inertial = Inertial.from_geometry(
        Box((0.240, 0.140, 0.090)),
        mass=0.58,
        origin=Origin(xyz=(0.030, -0.060, 0.005)),
    )

    right_body = model.part("right_body")
    right_body.visual(
        half_shell,
        origin=Origin(xyz=(body_x, body_y, 0.0)),
        material=armor_black,
        name="right_shell",
    )
    right_body.visual(
        Box((0.054, 0.032, 0.038)),
        origin=Origin(xyz=(bridge_arm_x, 0.031, -0.005)),
        material=bridge_gray,
        name="right_bridge_arm",
    )
    right_body.visual(
        hinge_collar,
        material=bridge_gray,
        name="hinge_collar",
    )
    right_body.visual(
        Box((0.030, 0.024, 0.008)),
        origin=Origin(xyz=(0.014, 0.069, 0.028)),
        material=bridge_gray,
        name="dial_pad",
    )
    right_body.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=selector_center),
        material=bridge_gray,
        name="selector_spindle",
    )
    right_body.visual(
        Cylinder(radius=0.024, length=0.066),
        origin=Origin(xyz=(objective_x, body_y, objective_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor_black,
        name="right_objective_tube",
    )
    right_body.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.133, body_y, objective_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bridge_gray,
        name="right_objective_bezel",
    )
    right_body.visual(
        Cylinder(radius=0.0215, length=0.003),
        origin=Origin(xyz=(0.1315, body_y, objective_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="right_objective_lens",
    )
    right_body.visual(
        Cylinder(radius=0.019, length=0.028),
        origin=Origin(xyz=(eyecup_x, 0.069, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor_black,
        name="right_eyecup",
    )
    right_body.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(-0.046, 0.069, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bridge_gray,
        name="right_ocular",
    )
    right_body.inertial = Inertial.from_geometry(
        Box((0.240, 0.140, 0.090)),
        mass=0.58,
        origin=Origin(xyz=(0.030, 0.060, 0.005)),
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_gray,
        name="focus_hub",
    )
    focus_wheel.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_black,
        name="focus_tread",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.016),
        mass=0.06,
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        selector_shell,
        material=control_gray,
        name="selector_knob",
    )
    selector_dial.visual(
        Box((0.010, 0.003, 0.004)),
        origin=Origin(xyz=(0.006, 0.0, 0.007)),
        material=accent_orange,
        name="selector_pointer",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.012),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "interpupillary_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.22,
            upper=0.22,
        ),
    )
    model.articulation(
        "center_focus_joint",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=focus_wheel,
        origin=Origin(xyz=focus_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=8.0,
            lower=-4.5,
            upper=4.5,
        ),
    )
    model.articulation(
        "selector_dial_joint",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=selector_dial,
        origin=Origin(xyz=selector_center),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=-0.9,
            upper=0.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_wheel = object_model.get_part("focus_wheel")
    selector_dial = object_model.get_part("selector_dial")

    hinge = object_model.get_articulation("interpupillary_hinge")
    focus_joint = object_model.get_articulation("center_focus_joint")
    selector_joint = object_model.get_articulation("selector_dial_joint")

    hinge_pin = left_body.get_visual("hinge_pin")
    hinge_collar = right_body.get_visual("hinge_collar")
    focus_left_stub = left_body.get_visual("focus_left_stub")
    focus_hub = focus_wheel.get_visual("focus_hub")
    selector_spindle = right_body.get_visual("selector_spindle")
    selector_knob = selector_dial.get_visual("selector_knob")

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
    ctx.allow_overlap(
        left_body,
        right_body,
        elem_a=hinge_pin,
        elem_b=hinge_collar,
        reason="The exposed hinge pin and rotating collar visually represent a fitted revolute hinge with hidden bearing clearance.",
    )
    ctx.allow_overlap(
        right_body,
        selector_dial,
        elem_a=selector_spindle,
        elem_b=selector_knob,
        reason="The selector knob shell stands in for a dial rotating around a concealed spindle and detent hub.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_body,
        right_body,
        elem_a=hinge_pin,
        elem_b=hinge_collar,
        name="center hinge collar rides on the pin",
    )
    ctx.expect_contact(
        left_body,
        focus_wheel,
        elem_a=focus_left_stub,
        elem_b=focus_hub,
        name="focus wheel turns between bridge support stubs",
    )
    ctx.expect_contact(
        right_body,
        selector_dial,
        elem_a=selector_spindle,
        elem_b=selector_knob,
        name="selector dial turns on a spindle",
    )

    ctx.check(
        "hinge and controls use intended axes",
        hinge.axis == (0.0, 0.0, 1.0)
        and focus_joint.axis == (0.0, 1.0, 0.0)
        and selector_joint.axis == (0.0, 0.0, 1.0),
        details=(
            f"hinge={hinge.axis}, focus={focus_joint.axis}, selector={selector_joint.axis}"
        ),
    )

    rest_right_tube = ctx.part_element_world_aabb(right_body, elem="right_objective_tube")
    upper_hinge = hinge.motion_limits.upper if hinge.motion_limits is not None else 0.22
    with ctx.pose({hinge: upper_hinge}):
        opened_right_tube = ctx.part_element_world_aabb(right_body, elem="right_objective_tube")
    ctx.check(
        "interpupillary hinge widens the right barrel stance",
        rest_right_tube is not None
        and opened_right_tube is not None
        and 0.5 * (opened_right_tube[0][1] + opened_right_tube[1][1])
        > 0.5 * (rest_right_tube[0][1] + rest_right_tube[1][1]) + 0.003,
        details=f"rest={rest_right_tube}, opened={opened_right_tube}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
