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
    ExtrudeGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _profile_loop(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _rounded_slab_mesh(
    name: str,
    *,
    width: float,
    length: float,
    thickness: float,
    radius: float,
    side_taper: float,
) :
    bottom = rounded_rect_profile(width, length, radius, corner_segments=8)
    mid = rounded_rect_profile(
        width - side_taper,
        length - side_taper,
        max(radius - side_taper * 0.35, 0.0012),
        corner_segments=8,
    )
    top = rounded_rect_profile(
        width - side_taper * 1.5,
        length - side_taper * 1.5,
        max(radius - side_taper * 0.6, 0.0010),
        corner_segments=8,
    )
    geom = LoftGeometry(
        [
            _profile_loop(bottom, -thickness * 0.5),
            _profile_loop(mid, 0.0),
            _profile_loop(top, thickness * 0.5),
        ],
        cap=True,
        closed=True,
    )
    return _mesh(name, geom)


def _button_mesh(name: str, *, width: float, height: float, thickness: float, radius: float):
    return _mesh(
        name,
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, height, radius, corner_segments=6),
            thickness,
            cap=True,
            closed=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_flip_phone")

    shell_silver = model.material("shell_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.12, 0.13, 0.15, 1.0))
    keypad_black = model.material("keypad_black", rgba=(0.16, 0.17, 0.19, 1.0))
    keypad_gray = model.material("keypad_gray", rgba=(0.34, 0.35, 0.38, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.18, 0.42, 0.66, 0.55))
    hinge_metal = model.material("hinge_metal", rgba=(0.48, 0.50, 0.54, 1.0))
    slot_black = model.material("slot_black", rgba=(0.05, 0.05, 0.06, 1.0))

    lower_shell_mesh = _rounded_slab_mesh(
        "lower_body_shell",
        width=0.050,
        length=0.092,
        thickness=0.018,
        radius=0.0075,
        side_taper=0.0018,
    )
    upper_shell_mesh = _rounded_slab_mesh(
        "upper_body_shell",
        width=0.048,
        length=0.088,
        thickness=0.016,
        radius=0.0070,
        side_taper=0.0015,
    )
    t9_button_mesh = _button_mesh(
        "t9_button",
        width=0.0100,
        height=0.0080,
        thickness=0.0026,
        radius=0.0016,
    )
    soft_key_mesh = _button_mesh(
        "soft_key_button",
        width=0.0115,
        height=0.0070,
        thickness=0.0024,
        radius=0.0015,
    )

    lower = model.part("lower_body")
    lower.visual(
        lower_shell_mesh,
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
        material=shell_silver,
        name="lower_shell",
    )
    lower.visual(
        Box((0.043, 0.079, 0.0014)),
        origin=Origin(xyz=(0.0, -0.047, 0.0084)),
        material=bezel_black,
        name="keypad_deck",
    )
    lower.visual(
        Box((0.042, 0.008, 0.007)),
        origin=Origin(xyz=(0.0, -0.003, 0.0080)),
        material=shell_silver,
        name="lower_hinge_bridge",
    )
    lower.visual(
        Cylinder(radius=0.0050, length=0.0140),
        origin=Origin(xyz=(-0.0160, 0.0, 0.0130), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="lower_hinge_left",
    )
    lower.visual(
        Cylinder(radius=0.0050, length=0.0140),
        origin=Origin(xyz=(0.0160, 0.0, 0.0130), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="lower_hinge_right",
    )
    lower.visual(
        Cylinder(radius=0.0110, length=0.0028),
        origin=Origin(xyz=(0.0, -0.024, 0.0104)),
        material=keypad_gray,
        name="nav_pad",
    )
    lower.visual(
        Cylinder(radius=0.0045, length=0.0030),
        origin=Origin(xyz=(0.0, -0.024, 0.0106)),
        material=keypad_black,
        name="nav_select",
    )
    lower.visual(
        soft_key_mesh,
        origin=Origin(xyz=(-0.015, -0.024, 0.0104)),
        material=keypad_black,
        name="call_key",
    )
    lower.visual(
        soft_key_mesh,
        origin=Origin(xyz=(0.015, -0.024, 0.0104)),
        material=keypad_black,
        name="end_key",
    )

    t9_layout = [
        ("t9_1", -0.014, -0.039),
        ("t9_2", 0.000, -0.039),
        ("t9_3", 0.014, -0.039),
        ("t9_4", -0.014, -0.051),
        ("t9_5", 0.000, -0.051),
        ("t9_6", 0.014, -0.051),
        ("t9_7", -0.014, -0.063),
        ("t9_8", 0.000, -0.063),
        ("t9_9", 0.014, -0.063),
        ("t9_star", -0.014, -0.075),
        ("t9_0", 0.000, -0.075),
        ("t9_hash", 0.014, -0.075),
    ]
    for name, x_pos, y_pos in t9_layout:
        lower.visual(
            t9_button_mesh,
            origin=Origin(xyz=(x_pos, y_pos, 0.0100)),
            material=keypad_black,
            name=name,
        )

    lower.visual(
        Box((0.010, 0.0022, 0.0012)),
        origin=Origin(xyz=(0.0, -0.086, 0.0089)),
        material=slot_black,
        name="microphone_slot",
    )
    lower.inertial = Inertial.from_geometry(
        Box((0.052, 0.094, 0.024)),
        mass=0.11,
        origin=Origin(xyz=(0.0, -0.046, 0.004)),
    )

    upper = model.part("upper_body")
    upper.visual(
        upper_shell_mesh,
        origin=Origin(xyz=(0.0, 0.051, -0.0026)),
        material=shell_silver,
        name="upper_shell",
    )
    upper.visual(
        Box((0.039, 0.066, 0.0014)),
        origin=Origin(xyz=(0.0, 0.059, 0.0052)),
        material=bezel_black,
        name="display_bezel",
    )
    upper.visual(
        Box((0.031, 0.043, 0.0016)),
        origin=Origin(xyz=(0.0, 0.059, 0.0060)),
        material=glass_blue,
        name="display_glass",
    )
    upper.visual(
        Box((0.012, 0.0028, 0.0012)),
        origin=Origin(xyz=(0.0, 0.085, 0.0057)),
        material=slot_black,
        name="earpiece_slot",
    )
    upper.visual(
        Box((0.014, 0.011, 0.007)),
        origin=Origin(xyz=(0.0, 0.006, -0.0004)),
        material=shell_silver,
        name="upper_hinge_bridge",
    )
    upper.visual(
        Cylinder(radius=0.0050, length=0.0180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="upper_hinge_center",
    )
    upper.inertial = Inertial.from_geometry(
        Box((0.050, 0.090, 0.022)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.051, 0.003)),
    )

    model.articulation(
        "clamshell_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.013), rpy=(0.12, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-0.20,
            upper=2.75,
        ),
    )

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
    ctx.fail_if_parts_overlap_in_current_pose()

    try:
        lower = object_model.get_part("lower_body")
        upper = object_model.get_part("upper_body")
        hinge = object_model.get_articulation("clamshell_hinge")
    except Exception as exc:
        ctx.fail("required_phone_structure_present", str(exc))
        return ctx.report()

    lower_visual_names = {visual.name for visual in lower.visuals}
    upper_visual_names = {visual.name for visual in upper.visuals}
    ctx.check(
        "lower_body_features_present",
        {
            "lower_shell",
            "nav_pad",
            "t9_1",
            "t9_0",
            "lower_hinge_left",
            "lower_hinge_right",
        }.issubset(lower_visual_names),
        details=f"lower visuals: {sorted(name for name in lower_visual_names if name is not None)}",
    )
    ctx.check(
        "upper_body_features_present",
        {
            "upper_shell",
            "display_bezel",
            "display_glass",
            "upper_hinge_center",
        }.issubset(upper_visual_names),
        details=f"upper visuals: {sorted(name for name in upper_visual_names if name is not None)}",
    )

    axis = hinge.axis
    limits = hinge.motion_limits
    ctx.check(
        "hinge_axis_runs_across_phone_width",
        abs(abs(axis[0]) - 1.0) < 1e-6 and abs(axis[1]) < 1e-6 and abs(axis[2]) < 1e-6,
        details=f"axis={axis}",
    )
    ctx.check(
        "hinge_has_clamshell_motion_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and (limits.upper - limits.lower) > 2.5,
        details=f"limits={limits}",
    )
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.008, name="hinge_origin_near_visible_barrel")

    ctx.expect_contact(
        lower,
        upper,
        elem_a="lower_hinge_left",
        elem_b="upper_hinge_center",
        contact_tol=1e-6,
        name="left_hinge_knuckle_meets_center_barrel",
    )
    ctx.expect_contact(
        lower,
        upper,
        elem_a="lower_hinge_right",
        elem_b="upper_hinge_center",
        contact_tol=1e-6,
        name="right_hinge_knuckle_meets_center_barrel",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            upper,
            lower,
            axis="y",
            positive_elem="display_glass",
            negative_elem="nav_pad",
            min_gap=0.040,
            name="screen_sits_above_keypad_in_open_pose",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="x",
            elem_a="display_glass",
            elem_b="nav_pad",
            min_overlap=0.020,
            name="screen_and_keypad_stay_centered_widthwise",
        )

    with ctx.pose({hinge: 2.35}):
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            elem_a="upper_shell",
            elem_b="lower_shell",
            min_overlap=0.020,
            name="lid_covers_base_when_nearly_closed",
        )
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem="upper_shell",
            negative_elem="lower_shell",
            min_gap=0.0,
            max_gap=0.035,
            name="lid_stays_close_without_interpenetration_when_nearly_closed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
