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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_monocular_microscope")

    enamel = model.material("enamel", rgba=(0.86, 0.88, 0.84, 1.0))
    enamel_shadow = model.material("enamel_shadow", rgba=(0.74, 0.76, 0.72, 1.0))
    stage_black = model.material("stage_black", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    objective_metal = model.material("objective_metal", rgba=(0.63, 0.65, 0.68, 1.0))
    glass_black = model.material("glass_black", rgba=(0.09, 0.10, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.24, 0.20, 0.31)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    base_shell = section_loft(
        [
            _xy_section(0.220, 0.185, 0.030, 0.000),
            _xy_section(0.205, 0.165, 0.028, 0.018),
            _xy_section(0.152, 0.112, 0.022, 0.040),
        ]
    )
    frame.visual(_mesh("microscope_base_shell", base_shell), material=enamel, name="base_shell")
    frame.visual(
        Box((0.035, 0.045, 0.012)),
        origin=Origin(xyz=(0.0, 0.050, 0.046)),
        material=enamel_shadow,
        name="base_neck",
    )
    frame.visual(
        Box((0.082, 0.068, 0.024)),
        origin=Origin(xyz=(0.0, 0.034, 0.050)),
        material=enamel_shadow,
        name="column_pedestal",
    )
    frame.visual(
        Box((0.050, 0.040, 0.250)),
        origin=Origin(xyz=(0.0, 0.048, 0.180)),
        material=enamel_shadow,
        name="column_spine",
    )

    guide_profile = rounded_rect_profile(0.024, 0.018, 0.003, corner_segments=6)
    guide_rail = ExtrudeGeometry(guide_profile, 0.170, center=True)
    frame.visual(
        _mesh("microscope_guide_rail", guide_rail),
        origin=Origin(xyz=(0.0, 0.048, 0.225)),
        material=enamel_shadow,
        name="guide_rail",
    )
    frame.visual(
        Box((0.060, 0.048, 0.030)),
        origin=Origin(xyz=(0.0, 0.048, 0.305)),
        material=enamel_shadow,
        name="column_cap",
    )
    frame.visual(
        Box((0.070, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.022, 0.114)),
        material=enamel_shadow,
        name="stage_support_bridge",
    )
    frame.visual(
        Box((0.032, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, 0.123)),
        material=enamel_shadow,
        name="stage_support_rib",
    )

    stage_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.126, 0.110, 0.008, corner_segments=8),
        [superellipse_profile(0.028, 0.028, exponent=2.0, segments=28)],
        height=0.006,
        center=True,
    )
    frame.visual(
        _mesh("microscope_stage_plate", stage_plate),
        origin=Origin(xyz=(0.0, -0.005, 0.132)),
        material=stage_black,
        name="stage_plate",
    )
    frame.visual(
        Box((0.040, 0.070, 0.005)),
        origin=Origin(xyz=(0.0, -0.020, 0.1355)),
        material=stage_black,
        name="mechanical_stage_pad",
    )

    optical_carriage = model.part("optical_carriage")
    optical_carriage.inertial = Inertial.from_geometry(
        Box((0.110, 0.160, 0.160)),
        mass=1.9,
        origin=Origin(xyz=(0.0, -0.010, 0.010)),
    )

    sleeve_outer = rounded_rect_profile(0.074, 0.060, 0.010, corner_segments=8)
    sleeve_hole = rounded_rect_profile(0.056, 0.046, 0.006, corner_segments=6)
    sleeve = ExtrudeWithHolesGeometry(
        sleeve_outer,
        [sleeve_hole],
        height=0.075,
        center=True,
    )
    optical_carriage.visual(
        _mesh("microscope_carriage_sleeve", sleeve),
        material=enamel,
        name="sleeve",
    )
    optical_carriage.visual(
        Box((0.024, 0.020, 0.075)),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=enamel_shadow,
        name="slide_shoe",
    )
    optical_carriage.visual(
        Box((0.024, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, -0.060, -0.020)),
        material=enamel_shadow,
        name="carriage_arm",
    )
    optical_carriage.visual(
        Box((0.034, 0.052, 0.050)),
        origin=Origin(xyz=(0.0, -0.096, 0.008)),
        material=enamel,
        name="head_body",
    )
    optical_carriage.visual(
        Box((0.044, 0.032, 0.046)),
        origin=Origin(xyz=(0.0, -0.096, -0.016)),
        material=enamel_shadow,
        name="nose_bridge",
    )

    focus_bearing = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.052, 0.050, 0.010, corner_segments=8),
        [superellipse_profile(0.017, 0.017, exponent=2.0, segments=28)],
        height=0.038,
        center=True,
    )
    optical_carriage.visual(
        _mesh("microscope_focus_bearing", focus_bearing),
        origin=Origin(xyz=(0.0, -0.050, -0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=enamel_shadow,
        name="focus_bearing",
    )

    optical_carriage.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, -0.083, 0.064), rpy=(0.92, 0.0, 0.0)),
        material=enamel,
        name="eyepiece_tube",
    )
    optical_carriage.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, -0.070, 0.047), rpy=(0.92, 0.0, 0.0)),
        material=enamel_shadow,
        name="eyepiece_collar",
    )
    optical_carriage.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, -0.092, -0.004)),
        material=dark_metal,
        name="objective_turret",
    )
    optical_carriage.visual(
        Cylinder(radius=0.008, length=0.036),
        origin=Origin(xyz=(0.0, -0.092, -0.023)),
        material=objective_metal,
        name="objective_primary",
    )
    optical_carriage.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.016, -0.081, -0.020)),
        material=objective_metal,
        name="objective_side_left",
    )
    optical_carriage.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(-0.016, -0.081, -0.020)),
        material=objective_metal,
        name="objective_side_right",
    )
    optical_carriage.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.0, -0.092, -0.045)),
        material=glass_black,
        name="objective_tip",
    )

    focus_knob_pair = model.part("focus_knob_pair")
    focus_knob_pair.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.092),
        mass=0.22,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="knob_shaft",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="knob_hub_right",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="knob_hub_left",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_right",
    )
    focus_knob_pair.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.039, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_left",
    )

    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=optical_carriage,
        origin=Origin(xyz=(0.0, 0.048, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.08, lower=0.0, upper=0.055),
    )
    model.articulation(
        "focus_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=optical_carriage,
        child=focus_knob_pair,
        origin=Origin(xyz=(0.0, -0.050, -0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    optical_carriage = object_model.get_part("optical_carriage")
    focus_knob_pair = object_model.get_part("focus_knob_pair")
    focus_slide = object_model.get_articulation("focus_slide")
    focus_knob_spin = object_model.get_articulation("focus_knob_spin")

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
        "focus_slide_joint_is_vertical_prismatic",
        focus_slide.joint_type == ArticulationType.PRISMATIC
        and tuple(focus_slide.axis) == (0.0, 0.0, 1.0)
        and focus_slide.motion_limits is not None
        and focus_slide.motion_limits.lower == 0.0
        and focus_slide.motion_limits.upper is not None
        and focus_slide.motion_limits.upper >= 0.05,
        details="Optical body carriage should translate upward along the column over a realistic focus range.",
    )
    ctx.check(
        "focus_knob_joint_is_horizontal_continuous",
        focus_knob_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(focus_knob_spin.axis) == (1.0, 0.0, 0.0),
        details="The paired focus knobs should co-rotate continuously about a shared horizontal axis.",
    )

    with ctx.pose({focus_slide: 0.0}):
        ctx.expect_contact(
            optical_carriage,
            frame,
            elem_a="slide_shoe",
            elem_b="column_spine",
            contact_tol=1e-6,
            name="carriage_slide_shoe_contacts_column",
        )
        ctx.expect_overlap(
            optical_carriage,
            frame,
            elem_a="sleeve",
            elem_b="guide_rail",
            axes="xz",
            min_overlap=0.020,
            name="carriage_sleeve_tracks_guide_rail",
        )
        ctx.expect_overlap(
            focus_knob_pair,
            optical_carriage,
            elem_a="knob_shaft",
            elem_b="focus_bearing",
            axes="yz",
            min_overlap=0.010,
            name="focus_knob_shaft_runs_through_bearing",
        )
        ctx.expect_gap(
            frame,
            focus_knob_pair,
            axis="y",
            positive_elem="guide_rail",
            negative_elem="knob_shaft",
            min_gap=0.020,
            name="focus_knob_shaft_clears_column",
        )
        ctx.expect_gap(
            optical_carriage,
            frame,
            axis="z",
            positive_elem="objective_tip",
            negative_elem="stage_plate",
            min_gap=0.008,
            max_gap=0.030,
            name="objective_tip_clears_stage",
        )
        ctx.expect_overlap(
            optical_carriage,
            frame,
            axes="xy",
            elem_a="objective_primary",
            elem_b="stage_plate",
            min_overlap=0.010,
            name="objective_sits_over_stage_opening_region",
        )

    upper_focus = 0.055
    with ctx.pose({focus_slide: 0.0}):
        low_pos = ctx.part_world_position(optical_carriage)
    with ctx.pose({focus_slide: upper_focus}):
        high_pos = ctx.part_world_position(optical_carriage)
    ctx.check(
        "focus_slide_moves_carriage_upward",
        low_pos is not None and high_pos is not None and (high_pos[2] - low_pos[2]) > 0.050,
        details=f"Expected at least 5 cm of upward carriage travel, got low={low_pos} high={high_pos}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
