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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_CENTER_OFFSET = 0.033
HINGE_Y = -0.011
EYEPIECE_Y = -0.045


def _xz_section(
    *,
    center_x: float,
    y_pos: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + px, y_pos, pz)
        for px, pz in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _build_housing_shell(center_x: float):
    return section_loft(
        [
            _xz_section(center_x=center_x, y_pos=-0.003, width=0.031, height=0.034, radius=0.006),
            _xz_section(center_x=center_x, y_pos=-0.013, width=0.032, height=0.035, radius=0.006),
            _xz_section(center_x=center_x, y_pos=-0.023, width=0.028, height=0.031, radius=0.005),
            _xz_section(center_x=center_x, y_pos=-0.033, width=0.023, height=0.024, radius=0.004),
        ]
    )


def _build_eyecup_shell():
    outer_profile = [
        (0.0114, 0.000),
        (0.0123, 0.002),
        (0.0144, 0.007),
        (0.0152, 0.014),
        (0.0142, 0.017),
    ]
    inner_profile = [
        (0.0108, 0.001),
        (0.0110, 0.003),
        (0.0126, 0.010),
        (0.0130, 0.016),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def _add_housing_visuals(
    part,
    *,
    side: int,
    y_offset: float,
    housing_material,
    trim_material,
    glass_material,
) -> None:
    center_x = side * BODY_CENTER_OFFSET
    part.visual(
        mesh_from_geometry(
            _build_housing_shell(center_x),
            "left_body_shell" if side < 0 else "right_body_shell",
        ),
        origin=Origin(xyz=(0.0, y_offset, 0.0)),
        material=housing_material,
        name="body_shell",
    )
    part.visual(
        Cylinder(radius=0.0135, length=0.044),
        origin=Origin(xyz=(center_x, 0.018 + y_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_material,
        name="objective_barrel",
    )
    part.visual(
        Cylinder(radius=0.0142, length=0.013),
        origin=Origin(xyz=(center_x, 0.012 + y_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="objective_armor",
    )
    part.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(center_x, 0.040 + y_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="objective_bezel",
    )
    part.visual(
        Cylinder(radius=0.0115, length=0.0012),
        origin=Origin(xyz=(center_x, 0.0425 + y_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass_material,
        name="objective_lens",
    )
    part.visual(
        Cylinder(radius=0.0112, length=0.028),
        origin=Origin(xyz=(center_x, -0.031 + y_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_material,
        name="eyepiece_barrel",
    )
    part.visual(
        Cylinder(radius=0.0126, length=0.008),
        origin=Origin(xyz=(center_x, -0.019 + y_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="eyepiece_shoulder",
    )
    part.visual(
        Box((0.0264, 0.014, 0.012)),
        origin=Origin(xyz=(side * 0.0177, HINGE_Y + y_offset, 0.0)),
        material=housing_material,
        name="bridge_arm",
    )

    if side < 0:
        part.visual(
            Cylinder(radius=0.0047, length=0.006),
            origin=Origin(xyz=(0.0, HINGE_Y + y_offset, 0.009)),
            material=trim_material,
            name="hinge_knuckle_upper",
        )
        part.visual(
            Cylinder(radius=0.0047, length=0.006),
            origin=Origin(xyz=(0.0, HINGE_Y + y_offset, -0.009)),
            material=trim_material,
            name="hinge_knuckle_lower",
        )
        part.visual(
            Box((0.006, 0.008, 0.024)),
            origin=Origin(xyz=(-0.015, -0.012 + y_offset, 0.018)),
            material=housing_material,
            name="focus_stem",
        )
        part.visual(
            Box((0.032, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, -0.012 + y_offset, 0.031)),
            material=housing_material,
            name="focus_arch",
        )
        part.visual(
            Box((0.004, 0.010, 0.018)),
            origin=Origin(xyz=(-0.011, -0.012 + y_offset, 0.022)),
            material=housing_material,
            name="focus_post_left",
        )
        part.visual(
            Box((0.004, 0.010, 0.018)),
            origin=Origin(xyz=(0.011, -0.012 + y_offset, 0.022)),
            material=housing_material,
            name="focus_post_right",
        )
    else:
        part.visual(
            Cylinder(radius=0.0047, length=0.012),
            origin=Origin(xyz=(0.0, HINGE_Y + y_offset, 0.0)),
            material=trim_material,
            name="hinge_knuckle_center",
        )


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) <= 1e-6 for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_theatre_binocular")

    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    armor_black = model.material("armor_black", rgba=(0.16, 0.16, 0.17, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.29, 0.30, 0.33, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.12, 0.17, 0.22, 1.0))

    left_housing = model.part("left_housing")
    right_housing = model.part("right_housing")
    focus_knob = model.part("focus_knob")
    left_eyecup = model.part("left_eyecup")
    right_eyecup = model.part("right_eyecup")

    _add_housing_visuals(
        left_housing,
        side=-1,
        y_offset=0.0,
        housing_material=body_black,
        trim_material=armor_black,
        glass_material=glass_blue,
    )
    _add_housing_visuals(
        right_housing,
        side=1,
        y_offset=-HINGE_Y,
        housing_material=body_black,
        trim_material=armor_black,
        glass_material=glass_blue,
    )

    focus_knob.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="focus_wheel_core",
    )
    focus_knob.visual(
        Cylinder(radius=0.0102, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor_black,
        name="focus_wheel_tread",
    )
    focus_knob.visual(
        Cylinder(radius=0.0035, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="focus_axle_hub",
    )

    eyecup_mesh = mesh_from_geometry(_build_eyecup_shell(), "eyecup_shell")
    left_eyecup.visual(
        eyecup_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_black,
        name="eyecup_shell",
    )
    left_eyecup.visual(
        Cylinder(radius=0.0110, length=0.002),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_black,
        name="eyecup_mount_collar",
    )
    right_eyecup.visual(
        eyecup_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_black,
        name="eyecup_shell",
    )
    right_eyecup.visual(
        Cylinder(radius=0.0110, length=0.002),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_black,
        name="eyecup_mount_collar",
    )

    model.articulation(
        "central_hinge",
        ArticulationType.REVOLUTE,
        parent=left_housing,
        child=right_housing,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.12,
        ),
    )
    model.articulation(
        "focus_knob_spin",
        ArticulationType.REVOLUTE,
        parent=left_housing,
        child=focus_knob,
        origin=Origin(xyz=(0.0, -0.012, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=8.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "left_eyecup_twist",
        ArticulationType.REVOLUTE,
        parent=left_housing,
        child=left_eyecup,
        origin=Origin(xyz=(-BODY_CENTER_OFFSET, EYEPIECE_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=5.0,
            lower=0.0,
            upper=4.2,
        ),
    )
    model.articulation(
        "right_eyecup_twist",
        ArticulationType.REVOLUTE,
        parent=right_housing,
        child=right_eyecup,
        origin=Origin(xyz=(BODY_CENTER_OFFSET, EYEPIECE_Y - HINGE_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=5.0,
            lower=0.0,
            upper=4.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_housing = object_model.get_part("left_housing")
    right_housing = object_model.get_part("right_housing")
    focus_knob = object_model.get_part("focus_knob")
    left_eyecup = object_model.get_part("left_eyecup")
    right_eyecup = object_model.get_part("right_eyecup")

    central_hinge = object_model.get_articulation("central_hinge")
    focus_knob_spin = object_model.get_articulation("focus_knob_spin")
    left_eyecup_twist = object_model.get_articulation("left_eyecup_twist")
    right_eyecup_twist = object_model.get_articulation("right_eyecup_twist")

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

    ctx.expect_contact(left_housing, right_housing, name="central_hinge_contact")
    ctx.expect_contact(focus_knob, left_housing, name="focus_knob_supported")
    ctx.expect_contact(left_eyecup, left_housing, name="left_eyecup_supported")
    ctx.expect_contact(right_eyecup, right_housing, name="right_eyecup_supported")
    ctx.expect_origin_distance(
        left_eyecup,
        right_eyecup,
        axes="x",
        min_dist=0.055,
        max_dist=0.075,
        name="interpupillary_spacing_open",
    )

    ctx.check(
        "central_hinge_axis_is_vertical",
        _axis_matches(central_hinge.axis, (0.0, 0.0, 1.0)),
        f"axis={central_hinge.axis}",
    )
    ctx.check(
        "focus_knob_axis_is_transverse",
        _axis_matches(focus_knob_spin.axis, (1.0, 0.0, 0.0)),
        f"axis={focus_knob_spin.axis}",
    )
    ctx.check(
        "left_eyecup_axis_matches_optical_axis",
        _axis_matches(left_eyecup_twist.axis, (0.0, 1.0, 0.0)),
        f"axis={left_eyecup_twist.axis}",
    )
    ctx.check(
        "right_eyecup_axis_matches_optical_axis",
        _axis_matches(right_eyecup_twist.axis, (0.0, 1.0, 0.0)),
        f"axis={right_eyecup_twist.axis}",
    )

    right_open = ctx.part_world_position(right_eyecup)
    assert right_open is not None
    with ctx.pose({central_hinge: -0.45}):
        right_folded = ctx.part_world_position(right_eyecup)
        assert right_folded is not None
        ctx.check(
            "central_hinge_folds_right_barrel_inward",
            right_folded[0] < right_open[0] - 0.012,
            f"open_x={right_open[0]:.4f}, folded_x={right_folded[0]:.4f}",
        )
        ctx.expect_contact(left_housing, right_housing, name="central_hinge_contact_folded")

    with ctx.pose({focus_knob_spin: math.pi, left_eyecup_twist: math.pi, right_eyecup_twist: math.pi}):
        ctx.expect_contact(focus_knob, left_housing, name="focus_knob_supported_when_rotated")
        ctx.expect_contact(left_eyecup, left_housing, name="left_eyecup_supported_when_rotated")
        ctx.expect_contact(right_eyecup, right_housing, name="right_eyecup_supported_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
