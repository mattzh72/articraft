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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    rounded_rect_profile,
    mesh_from_geometry,
    ConeGeometry,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_syringe")

    clear_body = model.material("clear_body", rgba=(0.82, 0.88, 0.95, 0.42))
    translucent_tip = model.material("translucent_tip", rgba=(0.84, 0.89, 0.96, 0.72))
    dark_mark = model.material("dark_mark", rgba=(0.20, 0.23, 0.27, 1.0))
    plunger_white = model.material("plunger_white", rgba=(0.94, 0.95, 0.96, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.32, 0.33, 0.35, 1.0))

    def x_aligned_mesh(geometry, name: str):
        geometry.rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geometry, name)

    thumb_pad_mesh = x_aligned_mesh(
        ExtrudeGeometry(
            rounded_rect_profile(0.026, 0.016, 0.0042, corner_segments=8),
            0.004,
            center=True,
        ),
        "thumb_pad",
    )
    nozzle_cone = x_aligned_mesh(
        ConeGeometry(radius=0.0020, height=0.008, radial_segments=36, closed=True),
        "nozzle_cone",
    )
    barrel_shell = x_aligned_mesh(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.0172, 0.0172, exponent=2.7, segments=40),
            [superellipse_profile(0.0139, 0.0139, exponent=2.4, segments=40)],
            0.030,
            cap=True,
            center=True,
            closed=True,
        ),
        "barrel_shell",
    )
    rear_guide = x_aligned_mesh(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.0190, 0.0190, exponent=2.8, segments=40),
            [superellipse_profile(0.0096, 0.0096, exponent=2.4, segments=40)],
            0.008,
            cap=True,
            center=True,
            closed=True,
        ),
        "rear_guide",
    )
    front_transition = x_aligned_mesh(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.0172, 0.0172, exponent=2.8, segments=40),
            [superellipse_profile(0.0090, 0.0090, exponent=2.4, segments=40)],
            0.003,
            cap=True,
            center=True,
            closed=True,
        ),
        "front_transition",
    )

    body = model.part("body")
    body.visual(
        barrel_shell,
        origin=Origin(xyz=(-0.0108, 0.0, 0.0)),
        material=clear_body,
        name="barrel_shell",
    )
    body.visual(
        rear_guide,
        origin=Origin(xyz=(0.0075, 0.0, 0.0)),
        material=translucent_tip,
        name="rear_guide",
    )
    body.visual(
        front_transition,
        origin=Origin(xyz=(-0.0268, 0.0, 0.0)),
        material=translucent_tip,
        name="front_transition",
    )
    body.visual(
        Cylinder(radius=0.0047, length=0.010),
        origin=Origin(xyz=(-0.0268, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=translucent_tip,
        name="front_hub",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(xyz=(-0.0336, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=translucent_tip,
        name="tip_collar",
    )
    body.visual(
        nozzle_cone,
        origin=Origin(xyz=(-0.0401, 0.0, 0.0)),
        material=translucent_tip,
        name="nozzle_tip",
    )
    body.visual(
        Cylinder(radius=0.00095, length=0.004),
        origin=Origin(xyz=(-0.0457, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=translucent_tip,
        name="tip_outlet",
    )
    body.visual(
        Box((0.010, 0.012, 0.0035)),
        origin=Origin(xyz=(0.0120, 0.0135, 0.0)),
        material=clear_body,
        name="finger_flange_left",
    )
    body.visual(
        Box((0.010, 0.012, 0.0035)),
        origin=Origin(xyz=(0.0120, -0.0135, 0.0)),
        material=clear_body,
        name="finger_flange_right",
    )

    graduation_x = (-0.019, -0.015, -0.011, -0.007, -0.003, 0.001)
    for index, x_pos in enumerate(graduation_x):
        major = index % 2 == 0
        body.visual(
            Box((0.00045, 0.0006, 0.0062 if major else 0.0042)),
            origin=Origin(xyz=(x_pos, 0.00835, 0.0)),
            material=dark_mark,
            name=f"graduation_{index:02d}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.060, 0.032, 0.024)),
        mass=0.06,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0061, length=0.0045),
        origin=Origin(xyz=(-0.02575, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_gray,
        name="piston_seal",
    )
    plunger.visual(
        Cylinder(radius=0.0048, length=0.0045),
        origin=Origin(xyz=(-0.02125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_white,
        name="seal_backer",
    )
    plunger.visual(
        Cylinder(radius=0.0019, length=0.031),
        origin=Origin(xyz=(-0.0060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_white,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_white,
        name="rear_stop_collar",
    )
    plunger.visual(
        Box((0.008, 0.009, 0.009)),
        origin=Origin(xyz=(0.0115, 0.0, 0.0)),
        material=plunger_white,
        name="thumb_stem",
    )
    plunger.visual(
        thumb_pad_mesh,
        origin=Origin(xyz=(0.0170, 0.0, 0.0)),
        material=plunger_white,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.050, 0.026, 0.018)),
        mass=0.02,
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.021,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    slider = object_model.get_articulation("body_to_plunger")

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
        "plunger_joint_axis_is_linear_and_coaxial",
        tuple(slider.axis) == (1.0, 0.0, 0.0),
        details=f"expected prismatic axis (1,0,0), got {slider.axis}",
    )
    ctx.check(
        "plunger_joint_has_compact_bounded_travel",
        slider.motion_limits is not None
        and slider.motion_limits.lower == 0.0
        and slider.motion_limits.upper is not None
        and 0.015 <= slider.motion_limits.upper <= 0.025,
        details="plunger travel should be a short bounded desktop-scale stroke",
    )

    ctx.expect_overlap(
        body,
        plunger,
        axes="x",
        elem_a="barrel_shell",
        elem_b="piston_seal",
        min_overlap=0.003,
        name="piston_is_guided_inside_barrel",
    )
    ctx.expect_overlap(
        body,
        plunger,
        axes="yz",
        elem_a="barrel_shell",
        elem_b="piston_seal",
        min_overlap=0.012,
        name="piston_stays_coaxial_with_barrel",
    )
    ctx.expect_gap(
        plunger,
        body,
        axis="x",
        positive_elem="thumb_pad",
        negative_elem="rear_guide",
        min_gap=0.001,
        max_gap=0.012,
        name="thumb_pad_sits_close_for_stowed_pose",
    )
    ctx.expect_contact(
        body,
        plunger,
        elem_a="rear_guide",
        elem_b="rear_stop_collar",
        contact_tol=0.0002,
        name="stowed_stop_keeps_plunger_supported",
    )

    with ctx.pose({slider: 0.021}):
        ctx.expect_contact(
            body,
            plunger,
            elem_a="rear_guide",
            elem_b="piston_seal",
            contact_tol=0.001,
            name="rear_guide_limits_extraction",
        )
        ctx.expect_overlap(
            body,
            plunger,
            axes="yz",
            elem_a="rear_guide",
            elem_b="piston_seal",
            min_overlap=0.011,
            name="piston_remains_centered_at_full_extension",
        )
        ctx.expect_gap(
            plunger,
            body,
            axis="x",
            positive_elem="thumb_pad",
            negative_elem="rear_guide",
            min_gap=0.020,
            max_gap=0.040,
            name="thumb_pad_clears_body_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
