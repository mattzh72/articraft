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
    section_loft,
    superellipse_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_loop(
    x_pos: float,
    width_y: float,
    height_z: float,
    *,
    exponent: float = 2.4,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(height_z, width_y, exponent=exponent, segments=segments)
    return [(x_pos, y_pos, z_pos) for z_pos, y_pos in profile]


def _yz_profile(
    width_y: float,
    height_z: float,
    *,
    exponent: float = 2.4,
    segments: int = 48,
) -> list[tuple[float, float]]:
    profile = superellipse_profile(height_z, width_y, exponent=exponent, segments=segments)
    return [(z_pos, y_pos) for z_pos, y_pos in profile]


def _headband_frame_geometry():
    return sweep_profile_along_spline(
        [
            (-0.074, 0.0, 0.104),
            (-0.060, 0.0, 0.142),
            (-0.036, 0.0, 0.171),
            (0.0, 0.0, 0.188),
            (0.036, 0.0, 0.171),
            (0.060, 0.0, 0.142),
            (0.074, 0.0, 0.104),
        ],
        profile=rounded_rect_profile(0.028, 0.010, 0.004, corner_segments=6),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def _headband_pad_geometry():
    return sweep_profile_along_spline(
        [
            (-0.060, 0.0, 0.096),
            (-0.046, 0.0, 0.123),
            (-0.028, 0.0, 0.146),
            (0.0, 0.0, 0.158),
            (0.028, 0.0, 0.146),
            (0.046, 0.0, 0.123),
            (0.060, 0.0, 0.096),
        ],
        profile=rounded_rect_profile(0.022, 0.008, 0.003, corner_segments=6),
        samples_per_segment=16,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def _earcup_shell_geometry():
    return section_loft(
        [
            _yz_loop(-0.014, 0.068, 0.086, exponent=2.6),
            _yz_loop(0.004, 0.078, 0.094, exponent=2.3),
            _yz_loop(0.018, 0.072, 0.088, exponent=2.55),
        ]
    )


def _earpad_ring_geometry():
    return ExtrudeWithHolesGeometry(
        _yz_profile(0.082, 0.094, exponent=2.3),
        [_yz_profile(0.044, 0.060, exponent=2.5)],
        height=0.018,
        center=True,
    ).rotate_y(math.pi / 2.0)


def _boom_tube_geometry():
    return tube_from_spline_points(
        [
            (0.0, 0.010, -0.003),
            (0.0, 0.040, -0.014),
            (0.0, 0.082, -0.027),
            (0.0, 0.118, -0.033),
        ],
        radius=0.0032,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_headset")

    plastic_black = model.material("plastic_black", rgba=(0.11, 0.12, 0.13, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.18, 0.19, 0.20, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.24, 0.25, 0.28, 1.0))
    fabric_black = model.material("fabric_black", rgba=(0.06, 0.06, 0.07, 1.0))
    mic_tip = model.material("mic_tip", rgba=(0.08, 0.08, 0.09, 1.0))

    headband_frame_mesh = _mesh("headband_frame", _headband_frame_geometry())
    headband_pad_mesh = _mesh("headband_pad", _headband_pad_geometry())
    earcup_shell_mesh = _mesh("earcup_shell", _earcup_shell_geometry())
    earpad_ring_mesh = _mesh("earpad_ring", _earpad_ring_geometry())
    boom_tube_mesh = _mesh("boom_tube", _boom_tube_geometry())

    headband = model.part("headband")
    headband.visual(
        headband_frame_mesh,
        material=plastic_black,
        name="headband_frame",
    )
    headband.visual(
        headband_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=soft_pad,
        name="headband_pad",
    )
    for prefix, side_sign in (("left", -1.0), ("right", 1.0)):
        hinge_x = side_sign * 0.079
        headband.visual(
            Box((0.014, 0.014, 0.010)),
            origin=Origin(xyz=(hinge_x, 0.0, 0.100)),
            material=metal_dark,
            name=f"{prefix}_hinge_block",
        )
        headband.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(
                xyz=(hinge_x, -0.008, 0.090),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name=f"{prefix}_hinge_knuckle_front",
        )
        headband.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(
                xyz=(hinge_x, 0.008, 0.090),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name=f"{prefix}_hinge_knuckle_rear",
        )
    headband.inertial = Inertial.from_geometry(
        Box((0.19, 0.032, 0.21)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    def add_yoke(name: str, side_sign: float):
        yoke = model.part(name)
        yoke.visual(
            Cylinder(radius=0.007, length=0.010),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_dark,
            name="hinge_sleeve",
        )
        yoke.visual(
            Box((0.018, 0.012, 0.018)),
            origin=Origin(xyz=(side_sign * 0.010, 0.0, -0.010)),
            material=metal_dark,
            name="hanger_block",
        )
        yoke.visual(
            Box((0.016, 0.090, 0.008)),
            origin=Origin(xyz=(side_sign * 0.020, 0.0, -0.008)),
            material=metal_dark,
            name="yoke_bridge",
        )
        yoke.visual(
            Box((0.008, 0.010, 0.052)),
            origin=Origin(xyz=(side_sign * 0.020, -0.045, -0.038)),
            material=metal_dark,
            name="front_arm",
        )
        yoke.visual(
            Box((0.008, 0.010, 0.052)),
            origin=Origin(xyz=(side_sign * 0.020, 0.045, -0.038)),
            material=metal_dark,
            name="rear_arm",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(
                xyz=(side_sign * 0.020, -0.042, -0.064),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name="front_sleeve",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(
                xyz=(side_sign * 0.020, 0.042, -0.064),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name="rear_sleeve",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.032, 0.10, 0.078)),
            mass=0.06,
            origin=Origin(xyz=(side_sign * 0.015, 0.0, -0.032)),
        )
        return yoke

    def add_earcup(name: str, side_sign: float, *, add_mic_mount: bool):
        cup = model.part(name)
        cup.visual(
            earcup_shell_mesh,
            origin=Origin(xyz=(side_sign * 0.008, 0.0, 0.0)),
            material=plastic_black,
            name="cup_shell",
        )
        cup.visual(
            earpad_ring_mesh,
            origin=Origin(xyz=(-side_sign * 0.010, 0.0, 0.0)),
            material=soft_pad,
            name="ear_pad",
        )
        cup.visual(
            Box((0.0025, 0.042, 0.056)),
            origin=Origin(xyz=(-side_sign * 0.018, 0.0, 0.0)),
            material=fabric_black,
            name="driver_cloth",
        )
        cup.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(
                xyz=(0.0, -0.036, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name="front_trunnion",
        )
        cup.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(
                xyz=(0.0, 0.036, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name="rear_trunnion",
        )
        if add_mic_mount:
            cup.visual(
                Box((0.019, 0.018, 0.014)),
                origin=Origin(xyz=(-0.0295, 0.019, -0.010)),
                material=metal_dark,
                name="mic_mount_block",
            )
            cup.visual(
                Cylinder(radius=0.005, length=0.005),
                origin=Origin(
                    xyz=(-0.0365, 0.031, -0.010),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=metal_dark,
                name="mic_knuckle_outer",
            )
            cup.visual(
                Cylinder(radius=0.005, length=0.005),
                origin=Origin(
                    xyz=(-0.0255, 0.031, -0.010),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=metal_dark,
                name="mic_knuckle_inner",
            )
        cup.inertial = Inertial.from_geometry(
            Box((0.046, 0.084, 0.096)),
            mass=0.14,
            origin=Origin(xyz=(side_sign * 0.004, 0.0, 0.0)),
        )
        return cup

    left_yoke = add_yoke("left_yoke", -1.0)
    right_yoke = add_yoke("right_yoke", 1.0)
    left_earcup = add_earcup("left_earcup", -1.0, add_mic_mount=True)
    right_earcup = add_earcup("right_earcup", 1.0, add_mic_mount=False)

    mic_boom = model.part("mic_boom")
    mic_boom.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="pivot_sleeve",
    )
    mic_boom.visual(
        Box((0.010, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.011, -0.003)),
        material=metal_dark,
        name="pivot_clip",
    )
    mic_boom.visual(
        boom_tube_mesh,
        material=metal_dark,
        name="boom_tube",
    )
    mic_boom.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(
            xyz=(0.0, 0.126, -0.033),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=mic_tip,
        name="boom_tip",
    )
    mic_boom.inertial = Inertial.from_geometry(
        Box((0.014, 0.15, 0.05)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.075, -0.016)),
    )

    model.articulation(
        "headband_to_left_yoke",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.079, 0.0, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "headband_to_right_yoke",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.079, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "left_yoke_to_left_earcup",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(-0.020, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "right_yoke_to_right_earcup",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.020, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "left_earcup_to_mic_boom",
        ArticulationType.REVOLUTE,
        parent=left_earcup,
        child=mic_boom,
        origin=Origin(xyz=(-0.031, 0.031, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")
    mic_boom = object_model.get_part("mic_boom")

    left_fold = object_model.get_articulation("headband_to_left_yoke")
    right_fold = object_model.get_articulation("headband_to_right_yoke")
    left_swivel = object_model.get_articulation("left_yoke_to_left_earcup")
    mic_flip = object_model.get_articulation("left_earcup_to_mic_boom")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(headband, left_yoke, name="left_yoke_hinged_to_headband")
    ctx.expect_contact(headband, right_yoke, name="right_yoke_hinged_to_headband")
    ctx.expect_contact(left_yoke, left_earcup, name="left_earcup_captured_in_yoke")
    ctx.expect_contact(right_yoke, right_earcup, name="right_earcup_captured_in_yoke")
    ctx.expect_contact(left_earcup, mic_boom, name="mic_boom_clipped_to_left_earcup")

    def _center_x(aabb) -> float:
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def _center_z(aabb) -> float:
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def _span_x(aabb) -> float:
        return aabb[1][0] - aabb[0][0]

    left_shell_rest = ctx.part_element_world_aabb(left_earcup, elem="cup_shell")
    right_shell_rest = ctx.part_element_world_aabb(right_earcup, elem="cup_shell")
    boom_tip_rest = ctx.part_element_world_aabb(mic_boom, elem="boom_tip")

    if left_shell_rest is None or right_shell_rest is None or boom_tip_rest is None:
        ctx.fail("named_visuals_resolve", "Expected cup_shell and boom_tip visuals to be measurable.")
        return ctx.report()

    with ctx.pose({left_fold: 1.35, right_fold: 1.35}):
        left_shell_folded = ctx.part_element_world_aabb(left_earcup, elem="cup_shell")
        right_shell_folded = ctx.part_element_world_aabb(right_earcup, elem="cup_shell")
        if left_shell_folded is None or right_shell_folded is None:
            ctx.fail("folded_cup_shells_resolve", "Folded earcup shell AABBs were unavailable.")
        else:
            ctx.check(
                "left_fold_moves_cup_inward",
                abs(_center_x(left_shell_folded)) < abs(_center_x(left_shell_rest)) - 0.015,
                details=(
                    f"left rest x={_center_x(left_shell_rest):.4f}, "
                    f"folded x={_center_x(left_shell_folded):.4f}"
                ),
            )
            ctx.check(
                "right_fold_moves_cup_inward",
                abs(_center_x(right_shell_folded)) < abs(_center_x(right_shell_rest)) - 0.015,
                details=(
                    f"right rest x={_center_x(right_shell_rest):.4f}, "
                    f"folded x={_center_x(right_shell_folded):.4f}"
                ),
            )
            ctx.expect_contact(headband, left_yoke, name="left_fold_hinge_remains_attached")
            ctx.expect_contact(headband, right_yoke, name="right_fold_hinge_remains_attached")

    with ctx.pose({left_swivel: 0.45}):
        left_shell_swiveled = ctx.part_element_world_aabb(left_earcup, elem="cup_shell")
        if left_shell_swiveled is None:
            ctx.fail("left_swiveled_shell_resolves", "Swiveled left earcup shell AABB was unavailable.")
        else:
            ctx.check(
                "left_earcup_swivel_changes_shell_pose",
                _span_x(left_shell_swiveled) > _span_x(left_shell_rest) + 0.02,
                details=(
                    f"rest x-span={_span_x(left_shell_rest):.4f}, "
                    f"swiveled x-span={_span_x(left_shell_swiveled):.4f}"
                ),
            )
            ctx.expect_contact(left_yoke, left_earcup, name="left_swivel_trunnion_stays_engaged")

    with ctx.pose({mic_flip: 1.35}):
        boom_tip_up = ctx.part_element_world_aabb(mic_boom, elem="boom_tip")
        if boom_tip_up is None:
            ctx.fail("boom_tip_up_resolves", "Raised microphone tip AABB was unavailable.")
        else:
            ctx.check(
                "mic_boom_flips_upward",
                _center_z(boom_tip_up) > _center_z(boom_tip_rest) + 0.045,
                details=(
                    f"rest z={_center_z(boom_tip_rest):.4f}, "
                    f"raised z={_center_z(boom_tip_up):.4f}"
                ),
            )
            ctx.expect_contact(left_earcup, mic_boom, name="mic_pivot_stays_attached_when_flipped")

    with ctx.pose({left_fold: 1.2, mic_flip: 1.35}):
        ctx.expect_contact(
            left_earcup,
            mic_boom,
            name="mic_boom_stays_attached_while_left_cup_folds",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
