from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_work_headset")

    shell_dark = model.material("shell_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    yoke_dark = model.material("yoke_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    pad_black = model.material("pad_black", rgba=(0.08, 0.08, 0.09, 1.0))
    mic_black = model.material("mic_black", rgba=(0.05, 0.05, 0.06, 1.0))

    def yz_section(x_pos: float, width_y: float, height_z: float, radius: float) -> list[tuple[float, float, float]]:
        return [(x_pos, y, z) for y, z in rounded_rect_profile(width_y, height_z, radius)]

    def make_headband_shell():
        band_path = [
            (-0.090, 0.0, 0.042),
            (-0.080, 0.0, 0.100),
            (-0.048, 0.0, 0.154),
            (0.000, 0.0, 0.176),
            (0.048, 0.0, 0.154),
            (0.080, 0.0, 0.100),
            (0.090, 0.0, 0.042),
        ]
        shell = sweep_profile_along_spline(
            band_path,
            profile=rounded_rect_profile(0.030, 0.012, 0.0045),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        )
        left_block = BoxGeometry((0.022, 0.024, 0.023)).translate(-0.090, 0.0, 0.035)
        right_block = BoxGeometry((0.022, 0.024, 0.023)).translate(0.090, 0.0, 0.035)
        left_barrel = (
            CylinderGeometry(radius=0.0075, height=0.008, radial_segments=20)
            .rotate_x(math.pi / 2.0)
            .translate(-0.090, -0.004, 0.035)
        )
        right_barrel = (
            CylinderGeometry(radius=0.0075, height=0.008, radial_segments=20)
            .rotate_x(math.pi / 2.0)
            .translate(0.090, -0.004, 0.035)
        )
        shell.merge(left_block).merge(right_block).merge(left_barrel).merge(right_barrel)
        return shell

    def make_headband_pad():
        pad_path = [
            (-0.070, 0.0, 0.044),
            (-0.058, 0.0, 0.092),
            (-0.032, 0.0, 0.126),
            (0.000, 0.0, 0.138),
            (0.032, 0.0, 0.126),
            (0.058, 0.0, 0.092),
            (0.070, 0.0, 0.044),
        ]
        return sweep_profile_along_spline(
            pad_path,
            profile=rounded_rect_profile(0.042, 0.016, 0.006),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        )

    def make_yoke_structure():
        yoke = (
            CylinderGeometry(radius=0.006, height=0.010, radial_segments=20)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.0, 0.0)
        )
        yoke.merge(BoxGeometry((0.010, 0.022, 0.012)).translate(0.0, 0.0, -0.010))
        yoke.merge(BoxGeometry((0.006, 0.104, 0.008)).translate(0.0, 0.0, -0.015))
        yoke.merge(BoxGeometry((0.006, 0.008, 0.074)).translate(0.0, -0.046, -0.051))
        yoke.merge(BoxGeometry((0.006, 0.008, 0.074)).translate(0.0, 0.046, -0.051))
        yoke.merge(
            CylinderGeometry(radius=0.005, height=0.008, radial_segments=18)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, -0.046, -0.090)
        )
        yoke.merge(
            CylinderGeometry(radius=0.005, height=0.008, radial_segments=18)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.046, -0.090)
        )
        yoke.merge(
            CylinderGeometry(radius=0.0035, height=0.100, radial_segments=18)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.0, -0.090)
        )
        return yoke

    def make_earcup_structure(*, with_mic_mount: bool):
        cup = section_loft(
            [
                yz_section(-0.056, 0.066, 0.090, 0.018),
                yz_section(-0.034, 0.082, 0.112, 0.024),
                yz_section(-0.012, 0.072, 0.098, 0.020),
            ]
        )
        cup.translate(0.0, 0.0, -0.040)
        cup.merge(
            CylinderGeometry(radius=0.0045, height=0.090, radial_segments=18)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.0, 0.0)
        )
        cup.merge(BoxGeometry((0.010, 0.026, 0.016)).translate(-0.010, 0.0, -0.014))
        if with_mic_mount:
            cup.merge(
                CylinderGeometry(radius=0.004, height=0.012, radial_segments=18)
                .rotate_x(math.pi / 2.0)
                .translate(-0.004, 0.042, -0.016)
            )
            cup.merge(BoxGeometry((0.008, 0.010, 0.014)).translate(-0.008, 0.042, -0.020))
        return cup

    def make_earpad():
        pad = ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.070, 0.088, 0.018),
            [rounded_rect_profile(0.042, 0.058, 0.012)],
            0.018,
            center=True,
        )
        pad.rotate_y(math.pi / 2.0)
        pad.translate(-0.003, 0.0, -0.040)
        return pad

    def make_boom_microphone():
        boom = (
            CylinderGeometry(radius=0.004, height=0.008, radial_segments=18)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, 0.0, 0.0)
        )
        boom.merge(BoxGeometry((0.008, 0.010, 0.006)).translate(0.007, 0.008, -0.002))
        boom.merge(
            tube_from_spline_points(
                [
                    (0.004, 0.008, -0.002),
                    (0.010, 0.022, -0.004),
                    (0.015, 0.039, -0.008),
                    (0.018, 0.054, -0.012),
                ],
                radius=0.0025,
                samples_per_segment=18,
                radial_segments=14,
            )
        )
        boom.merge(
            SphereGeometry(radius=0.0055, width_segments=18, height_segments=12).translate(
                0.018, 0.060, -0.013
            )
        )
        return boom

    headband = model.part("headband")
    headband.visual(
        mesh_from_geometry(make_headband_shell(), "headband_shell"),
        material=shell_dark,
        name="headband_shell",
    )
    headband.visual(
        mesh_from_geometry(make_headband_pad(), "headband_pad"),
        material=pad_black,
        name="headband_pad",
    )
    headband.inertial = Inertial.from_geometry(
        Box((0.205, 0.045, 0.185)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
    )

    yoke_mesh = mesh_from_geometry(make_yoke_structure(), "earcup_yoke")
    earpad_mesh = mesh_from_geometry(make_earpad(), "earpad_ring")

    left_yoke = model.part("left_yoke")
    left_yoke.visual(yoke_mesh, material=yoke_dark, name="left_yoke_frame")
    left_yoke.inertial = Inertial.from_geometry(
        Box((0.018, 0.108, 0.100)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    right_yoke = model.part("right_yoke")
    right_yoke.visual(yoke_mesh, material=yoke_dark, name="right_yoke_frame")
    right_yoke.inertial = Inertial.from_geometry(
        Box((0.018, 0.108, 0.100)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    left_cup = model.part("left_cup")
    left_cup.visual(
        mesh_from_geometry(make_earcup_structure(with_mic_mount=True), "left_earcup_shell"),
        material=shell_dark,
        name="left_cup_shell",
    )
    left_cup.visual(earpad_mesh, material=pad_black, name="left_earpad")
    left_cup.inertial = Inertial.from_geometry(
        Box((0.060, 0.088, 0.120)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    right_cup = model.part("right_cup")
    right_cup.visual(
        mesh_from_geometry(make_earcup_structure(with_mic_mount=False), "right_earcup_shell"),
        material=shell_dark,
        name="right_cup_shell",
    )
    right_cup.visual(earpad_mesh, material=pad_black, name="right_earpad")
    right_cup.inertial = Inertial.from_geometry(
        Box((0.060, 0.088, 0.120)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    boom_mic = model.part("boom_mic")
    boom_mic.visual(
        mesh_from_geometry(make_boom_microphone(), "boom_microphone"),
        material=mic_black,
        name="boom_mic",
    )
    boom_mic.inertial = Inertial.from_geometry(
        Box((0.035, 0.080, 0.020)),
        mass=0.04,
        origin=Origin(xyz=(0.010, 0.034, -0.006)),
    )

    model.articulation(
        "headband_to_left_yoke",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.090, 0.0, 0.0195)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "headband_to_right_yoke",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.090, 0.0, 0.0195)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "left_yoke_to_left_cup",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=-math.radians(26.0),
            upper=math.radians(26.0),
        ),
    )
    model.articulation(
        "right_yoke_to_right_cup",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=-math.radians(26.0),
            upper=math.radians(26.0),
        ),
    )
    model.articulation(
        "left_cup_to_boom_mic",
        ArticulationType.REVOLUTE,
        parent=left_cup,
        child=boom_mic,
        origin=Origin(xyz=(-0.004, 0.042, -0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-math.radians(70.0),
            upper=math.radians(55.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")
    boom_mic = object_model.get_part("boom_mic")

    left_fold = object_model.get_articulation("headband_to_left_yoke")
    right_fold = object_model.get_articulation("headband_to_right_yoke")
    left_swivel = object_model.get_articulation("left_yoke_to_left_cup")
    right_swivel = object_model.get_articulation("right_yoke_to_right_cup")
    mic_hinge = object_model.get_articulation("left_cup_to_boom_mic")

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
        headband,
        left_yoke,
        reason="Fold hinge barrels are simplified as coaxial interlocking parts.",
    )
    ctx.allow_overlap(
        headband,
        right_yoke,
        reason="Fold hinge barrels are simplified as coaxial interlocking parts.",
    )
    ctx.allow_overlap(
        left_cup,
        left_yoke,
        reason="Cup swivel axle is modeled as an interlocking hinge barrel.",
    )
    ctx.allow_overlap(
        right_cup,
        right_yoke,
        reason="Cup swivel axle is modeled as an interlocking hinge barrel.",
    )
    ctx.allow_overlap(
        left_cup,
        boom_mic,
        reason="Boom mic clip wraps the earcup hinge barrel at the microphone pivot.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "left fold axis",
        tuple(left_fold.axis) == (0.0, -1.0, 0.0),
        details=f"unexpected axis {left_fold.axis}",
    )
    ctx.check(
        "right fold axis",
        tuple(right_fold.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected axis {right_fold.axis}",
    )
    ctx.check(
        "cup swivel axes",
        tuple(left_swivel.axis) == (0.0, 1.0, 0.0) and tuple(right_swivel.axis) == (0.0, 1.0, 0.0),
        details=f"left={left_swivel.axis} right={right_swivel.axis}",
    )
    ctx.check(
        "boom hinge axis",
        tuple(mic_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected axis {mic_hinge.axis}",
    )

    ctx.expect_contact(left_yoke, headband, contact_tol=0.001, name="left yoke mounted to headband")
    ctx.expect_contact(right_yoke, headband, contact_tol=0.001, name="right yoke mounted to headband")
    ctx.expect_contact(left_cup, left_yoke, contact_tol=0.001, name="left cup captured in yoke")
    ctx.expect_contact(right_cup, right_yoke, contact_tol=0.001, name="right cup captured in yoke")
    ctx.expect_contact(boom_mic, left_cup, contact_tol=0.001, name="boom remains clipped to cup")

    def center_from_aabb(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    left_rest = center_from_aabb(ctx.part_world_aabb(left_cup))
    right_rest = center_from_aabb(ctx.part_world_aabb(right_cup))

    with ctx.pose({left_fold: math.radians(78.0), right_fold: math.radians(78.0)}):
        left_folded = center_from_aabb(ctx.part_world_aabb(left_cup))
        right_folded = center_from_aabb(ctx.part_world_aabb(right_cup))
        ctx.check(
            "left cup folds inward",
            left_folded[0] > left_rest[0] + 0.015 and left_folded[2] > left_rest[2] + 0.020,
            details=f"rest={left_rest} folded={left_folded}",
        )
        ctx.check(
            "right cup folds inward",
            right_folded[0] < right_rest[0] - 0.015 and right_folded[2] > right_rest[2] + 0.020,
            details=f"rest={right_rest} folded={right_folded}",
        )

    with ctx.pose({left_swivel: math.radians(22.0), right_swivel: -math.radians(22.0)}):
        left_swiveled = center_from_aabb(ctx.part_world_aabb(left_cup))
        right_swiveled = center_from_aabb(ctx.part_world_aabb(right_cup))
        ctx.check(
            "left cup swivel moves shell",
            abs(left_swiveled[0] - left_rest[0]) > 0.003 or abs(left_swiveled[2] - left_rest[2]) > 0.003,
            details=f"rest={left_rest} swiveled={left_swiveled}",
        )
        ctx.check(
            "right cup swivel moves shell",
            abs(right_swiveled[0] - right_rest[0]) > 0.003 or abs(right_swiveled[2] - right_rest[2]) > 0.003,
            details=f"rest={right_rest} swiveled={right_swiveled}",
        )
        ctx.expect_contact(left_cup, left_yoke, name="left swivel stays attached")
        ctx.expect_contact(right_cup, right_yoke, name="right swivel stays attached")

    mic_rest = center_from_aabb(ctx.part_world_aabb(boom_mic))
    with ctx.pose({mic_hinge: -math.radians(55.0)}):
        mic_raised = center_from_aabb(ctx.part_world_aabb(boom_mic))
        ctx.check(
            "boom rotates upward",
            mic_raised[2] > mic_rest[2] + 0.010,
            details=f"rest={mic_rest} raised={mic_raised}",
        )
        ctx.expect_contact(boom_mic, left_cup, contact_tol=0.001, name="boom hinge stays attached in raised pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
