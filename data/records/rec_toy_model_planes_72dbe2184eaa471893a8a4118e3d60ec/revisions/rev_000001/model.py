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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _ellipse_section(
    x_pos: float,
    width: float,
    height: float,
    *,
    z_center: float = 0.0,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (
            x_pos,
            half_w * math.cos((math.tau * idx) / samples),
            z_center + (half_h * math.sin((math.tau * idx) / samples)),
        )
        for idx in range(samples)
    ]


def _wing_panel_profile(side_sign: float) -> list[tuple[float, float]]:
    return [
        (-0.024, side_sign * 0.012),
        (0.020, side_sign * 0.012),
        (0.050, side_sign * 0.090),
        (0.018, side_sign * 0.106),
        (-0.018, side_sign * 0.098),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_toy_plane")

    stand_dark = model.material("stand_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    stand_mid = model.material("stand_mid", rgba=(0.33, 0.35, 0.38, 1.0))
    fuselage_paint = model.material("fuselage_paint", rgba=(0.92, 0.92, 0.88, 1.0))
    wing_red = model.material("wing_red", rgba=(0.70, 0.14, 0.14, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.38, 0.52, 0.68, 0.95))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))

    stand_base = model.part("stand_base")
    stand_base.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(0.120, 0.080, 0.016, corner_segments=8),
                0.012,
            ),
            "stand_base_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stand_dark,
        name="base_plate",
    )
    stand_base.visual(
        Cylinder(radius=0.0075, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=stand_mid,
        name="mast",
    )
    stand_base.visual(
        Box((0.018, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=stand_mid,
        name="top_block",
    )
    stand_base.visual(
        Box((0.014, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.016, 0.094)),
        material=stand_mid,
        name="front_ear",
    )
    stand_base.visual(
        Box((0.014, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.016, 0.094)),
        material=stand_mid,
        name="rear_ear",
    )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, 0.110)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    stand_cradle = model.part("stand_cradle")
    stand_cradle.visual(
        Box((0.004, 0.024, 0.008)),
        material=stand_mid,
        name="pivot_bar",
    )
    stand_cradle.visual(
        Box((0.012, 0.012, 0.048)),
        origin=Origin(xyz=(0.008, 0.0, 0.028)),
        material=stand_mid,
        name="support_arm",
    )
    stand_cradle.visual(
        Box((0.034, 0.016, 0.006)),
        origin=Origin(xyz=(0.020, 0.0, 0.052)),
        material=stand_mid,
        name="saddle",
    )
    stand_cradle.visual(
        Box((0.020, 0.010, 0.018)),
        origin=Origin(xyz=(0.015, 0.0, 0.040)),
        material=stand_mid,
        name="brace",
    )
    stand_cradle.inertial = Inertial.from_geometry(
        Box((0.040, 0.030, 0.060)),
        mass=0.08,
        origin=Origin(xyz=(0.014, 0.0, 0.030)),
    )

    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _ellipse_section(-0.090, 0.010, 0.010, z_center=0.004),
                    _ellipse_section(-0.072, 0.020, 0.018, z_center=0.006),
                    _ellipse_section(-0.028, 0.032, 0.032, z_center=0.008),
                    _ellipse_section(0.020, 0.038, 0.040, z_center=0.010),
                    _ellipse_section(0.060, 0.030, 0.028, z_center=0.008),
                    _ellipse_section(0.098, 0.012, 0.014, z_center=0.006),
                ]
            ),
            "fuselage_shell",
        ),
        material=fuselage_paint,
        name="fuselage_shell",
    )
    fuselage.visual(
        Box((0.034, 0.022, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, 0.020)),
        material=canopy_tint,
        name="canopy",
    )
    fuselage.visual(
        Box((0.030, 0.088, 0.004)),
        origin=Origin(xyz=(-0.074, 0.0, 0.008)),
        material=wing_red,
        name="tailplane",
    )
    fuselage.visual(
        Box((0.030, 0.004, 0.030)),
        origin=Origin(xyz=(-0.074, 0.0, 0.020)),
        material=wing_red,
        name="fin",
    )
    fuselage.visual(
        Box((0.020, 0.016, 0.006)),
        origin=Origin(xyz=(0.004, 0.013, 0.0155)),
        material=wing_red,
        name="left_wing_pad",
    )
    fuselage.visual(
        Box((0.020, 0.016, 0.006)),
        origin=Origin(xyz=(0.004, -0.013, 0.0155)),
        material=wing_red,
        name="right_wing_pad",
    )
    fuselage.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, -0.011)),
        material=fuselage_paint,
        name="mount_web",
    )
    fuselage.visual(
        Box((0.028, 0.016, 0.006)),
        origin=Origin(xyz=(0.000, 0.0, -0.017)),
        material=stand_mid,
        name="belly_mount",
    )
    fuselage.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.099, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wing_red,
        name="nose_cap",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((0.200, 0.100, 0.060)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    left_wing = model.part("left_wing")
    left_wing.visual(
        Box((0.016, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=wing_red,
        name="root_tab",
    )
    left_wing.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(_wing_panel_profile(1.0), 0.004),
            "left_wing_panel",
        ),
        material=wing_red,
        name="panel",
    )
    left_wing.inertial = Inertial.from_geometry(
        Box((0.080, 0.110, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(0.010, 0.055, 0.0)),
    )

    right_wing = model.part("right_wing")
    right_wing.visual(
        Box((0.016, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=wing_red,
        name="root_tab",
    )
    right_wing.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(_wing_panel_profile(-1.0), 0.004),
            "right_wing_panel",
        ),
        material=wing_red,
        name="panel",
    )
    right_wing.inertial = Inertial.from_geometry(
        Box((0.080, 0.110, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(0.010, -0.055, 0.0)),
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="hub",
    )
    propeller.visual(
        Box((0.003, 0.088, 0.010)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=prop_black,
        name="blade",
    )
    propeller.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wing_red,
        name="spinner",
    )
    propeller.inertial = Inertial.from_geometry(
        Box((0.024, 0.088, 0.016)),
        mass=0.015,
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
    )

    stand_tilt = model.articulation(
        "stand_base_to_cradle",
        ArticulationType.REVOLUTE,
        parent=stand_base,
        child=stand_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.2,
            lower=-0.25,
            upper=0.55,
        ),
    )

    model.articulation(
        "cradle_to_fuselage",
        ArticulationType.FIXED,
        parent=stand_cradle,
        child=fuselage,
        origin=Origin(xyz=(0.020, 0.0, 0.075)),
    )

    left_fold = model.articulation(
        "fuselage_to_left_wing",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=left_wing,
        origin=Origin(xyz=(0.004, 0.021, 0.0155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    right_fold = model.articulation(
        "fuselage_to_right_wing",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=right_wing,
        origin=Origin(xyz=(0.004, -0.021, 0.0155)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    model.articulation(
        "fuselage_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=propeller,
        origin=Origin(xyz=(0.103, 0.0, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_base = object_model.get_part("stand_base")
    stand_cradle = object_model.get_part("stand_cradle")
    fuselage = object_model.get_part("fuselage")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    propeller = object_model.get_part("propeller")

    stand_tilt = object_model.get_articulation("stand_base_to_cradle")
    left_fold = object_model.get_articulation("fuselage_to_left_wing")
    right_fold = object_model.get_articulation("fuselage_to_right_wing")

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
    ctx.warn_if_articulation_overlaps(max_pose_samples=20)

    ctx.expect_contact(
        stand_cradle,
        stand_base,
        elem_a="pivot_bar",
        elem_b="front_ear",
        name="cradle pivot bar bears on front stand ear",
    )
    ctx.expect_contact(
        stand_cradle,
        stand_base,
        elem_a="pivot_bar",
        elem_b="rear_ear",
        name="cradle pivot bar bears on rear stand ear",
    )
    ctx.expect_contact(
        fuselage,
        stand_cradle,
        elem_a="belly_mount",
        elem_b="saddle",
        name="fuselage is seated on the stand saddle",
    )
    ctx.expect_contact(
        left_wing,
        fuselage,
        elem_a="root_tab",
        elem_b="left_wing_pad",
        name="left wing root is mounted to the fuselage pad",
    )
    ctx.expect_contact(
        right_wing,
        fuselage,
        elem_a="root_tab",
        elem_b="right_wing_pad",
        name="right wing root is mounted to the fuselage pad",
    )
    ctx.expect_contact(
        propeller,
        fuselage,
        elem_a="hub",
        elem_b="nose_cap",
        name="propeller hub is mounted on the nose cap",
    )

    fold_angle = left_fold.motion_limits.upper or math.radians(85.0)
    lower_tilt = stand_tilt.motion_limits.lower or -0.25
    display_tilt = min(0.40, stand_tilt.motion_limits.upper or 0.40)

    with ctx.pose({left_fold: fold_angle, right_fold: fold_angle}):
        ctx.expect_gap(
            left_wing,
            fuselage,
            axis="z",
            min_gap=0.001,
            positive_elem="panel",
            negative_elem="canopy",
            name="left wing panel clears the canopy when folded",
        )
        ctx.expect_gap(
            right_wing,
            fuselage,
            axis="z",
            min_gap=0.001,
            positive_elem="panel",
            negative_elem="canopy",
            name="right wing panel clears the canopy when folded",
        )

    with ctx.pose({stand_tilt: lower_tilt}):
        ctx.expect_gap(
            propeller,
            stand_base,
            axis="z",
            min_gap=0.012,
            positive_elem="blade",
            negative_elem="base_plate",
            name="nose-down stand pose still keeps the propeller above the base",
        )

    def _center_z(part_name, elem_name):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def _group_span_y(parts):
        aabbs = [ctx.part_world_aabb(part) for part in parts]
        if any(aabb is None for aabb in aabbs):
            return None
        return max(aabb[1][1] for aabb in aabbs) - min(aabb[0][1] for aabb in aabbs)

    with ctx.pose({stand_tilt: 0.0}):
        neutral_nose_z = _center_z("fuselage", "nose_cap")
    with ctx.pose({stand_tilt: display_tilt}):
        raised_nose_z = _center_z("fuselage", "nose_cap")
    ctx.check(
        "positive stand tilt raises the toy plane nose",
        neutral_nose_z is not None
        and raised_nose_z is not None
        and raised_nose_z > neutral_nose_z + 0.015,
        details=(
            f"neutral_nose_z={neutral_nose_z}, "
            f"raised_nose_z={raised_nose_z}, tilt={display_tilt}"
        ),
    )

    with ctx.pose({left_fold: 0.0, right_fold: 0.0}):
        deployed_span = _group_span_y([fuselage, left_wing, right_wing, propeller])
    with ctx.pose({left_fold: fold_angle, right_fold: fold_angle}):
        stowed_span = _group_span_y([fuselage, left_wing, right_wing, propeller])
    ctx.check(
        "folded wings reduce the desktop span",
        deployed_span is not None
        and stowed_span is not None
        and stowed_span < deployed_span * 0.50
        and stowed_span < 0.10,
        details=f"deployed_span={deployed_span}, stowed_span={stowed_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
