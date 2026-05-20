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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _yz_profile(width_y: float, height_z: float, radius: float, *, corner_segments: int = 8):
    base = rounded_rect_profile(width_y, height_z, radius, corner_segments=corner_segments)
    return [(-u, v) for u, v in base]


def _extrude_profile_x(
    profile_2d,
    thickness: float,
    *,
    mesh_name: str,
    side_sign: int,
):
    geom = ExtrudeGeometry.from_z0(profile_2d, thickness, cap=True, closed=True)
    geom.rotate_y(math.pi / 2 if side_sign > 0 else -math.pi / 2).rotate_x(math.pi / 2)
    return mesh_from_geometry(geom, mesh_name)


def _extrude_ring_x(
    outer_profile_2d,
    hole_profile_2d,
    thickness: float,
    *,
    mesh_name: str,
    side_sign: int,
):
    geom = ExtrudeWithHolesGeometry(
        outer_profile_2d,
        [hole_profile_2d],
        thickness,
        cap=True,
        center=False,
        closed=True,
    )
    geom.rotate_y(math.pi / 2 if side_sign > 0 else -math.pi / 2).rotate_x(math.pi / 2)
    return mesh_from_geometry(geom, mesh_name)


def _aabb_size(aabb):
    if aabb is None:
        return None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    return (xmax - xmin, ymax - ymin, zmax - zmin)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)


def _add_earcup_geometry(
    part,
    *,
    side: str,
    side_sign: int,
    shell_material,
    cushion_material,
    fabric_material,
    mic_material,
    with_mic_mount: bool,
    front_boss_name: str,
    rear_boss_name: str,
):
    shell_outer = _yz_profile(0.046, 0.098, 0.012)
    shell_inner = _yz_profile(0.027, 0.064, 0.010)
    cushion_outer = _yz_profile(0.051, 0.104, 0.014)
    cushion_inner = _yz_profile(0.029, 0.068, 0.012)
    screen_profile = _yz_profile(0.030, 0.070, 0.010)

    part.visual(
        _extrude_profile_x(
            shell_outer,
            0.008,
            mesh_name=f"{side}_cup_outer_cap",
            side_sign=side_sign,
        ),
        material=shell_material,
        name=f"{side}_outer_cap",
    )
    part.visual(
        _extrude_ring_x(
            shell_outer,
            shell_inner,
            0.026,
            mesh_name=f"{side}_cup_shell_ring",
            side_sign=side_sign,
        ),
        origin=Origin(xyz=(side_sign * 0.006, 0.0, 0.0)),
        material=shell_material,
        name=f"{side}_shell_ring",
    )
    part.visual(
        _extrude_ring_x(
            cushion_outer,
            cushion_inner,
            0.016,
            mesh_name=f"{side}_cup_cushion",
            side_sign=side_sign,
        ),
        origin=Origin(xyz=(side_sign * 0.030, 0.0, 0.0)),
        material=cushion_material,
        name=f"{side}_cushion_ring",
    )
    part.visual(
        _extrude_profile_x(
            screen_profile,
            0.0015,
            mesh_name=f"{side}_cup_inner_screen",
            side_sign=side_sign,
        ),
        origin=Origin(xyz=(side_sign * 0.034, 0.0, 0.0)),
        material=fabric_material,
        name=f"{side}_driver_screen",
    )

    for boss_name, boss_y in ((front_boss_name, 0.023), (rear_boss_name, -0.023)):
        part.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(
                xyz=(side_sign * 0.0015, boss_y, 0.0),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=shell_material,
            name=boss_name,
        )

    if with_mic_mount:
        part.visual(
            Box((0.008, 0.014, 0.015)),
            origin=Origin(xyz=(side_sign * 0.004, 0.023, -0.028)),
            material=mic_material,
            name="mic_mount",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_headset")

    frame_black = model.material("frame_black", rgba=(0.10, 0.10, 0.11, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    cushion_dark = model.material("cushion_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    fabric_dark = model.material("fabric_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.25, 0.26, 0.28, 1.0))

    headband = model.part("headband")
    outer_frame_path = [
        (-0.095, 0.0, 0.111),
        (-0.080, 0.0, 0.154),
        (-0.040, 0.0, 0.184),
        (0.000, 0.0, 0.196),
        (0.040, 0.0, 0.184),
        (0.080, 0.0, 0.154),
        (0.095, 0.0, 0.111),
    ]
    headband.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                outer_frame_path,
                radius=0.0075,
                samples_per_segment=16,
                radial_segments=20,
                cap_ends=True,
            ),
            "headband_outer_frame",
        ),
        material=frame_black,
        name="outer_frame",
    )
    headband.visual(
        mesh_from_geometry(
            sweep_profile_along_spline(
                [
                    (-0.101, 0.0, 0.118),
                    (-0.082, 0.0, 0.154),
                    (-0.042, 0.0, 0.177),
                    (0.000, 0.0, 0.186),
                    (0.042, 0.0, 0.177),
                    (0.082, 0.0, 0.154),
                    (0.101, 0.0, 0.118),
                ],
                profile=rounded_rect_profile(0.032, 0.010, 0.004),
                samples_per_segment=16,
                cap_profile=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "headband_crown_pad",
        ),
        material=cushion_dark,
        name="crown_pad",
    )

    for side, x_sign, front_lug_name, rear_lug_name in (
        ("left", -1, "left_front_lug", "left_rear_lug"),
        ("right", 1, "right_front_lug", "right_rear_lug"),
    ):
        headband.visual(
            Box((0.010, 0.028, 0.060)),
            origin=Origin(xyz=(x_sign * 0.104, 0.0, 0.121)),
            material=frame_black,
            name=f"{side}_support",
        )
        for y_sign, stem_name, lug_name in (
            (1, "front", front_lug_name),
            (-1, "rear", rear_lug_name),
        ):
            headband.visual(
                Box((0.014, 0.018, 0.018)),
                origin=Origin(xyz=(x_sign * 0.111, y_sign * 0.021, 0.092)),
                material=frame_black,
                name=f"{side}_{stem_name}_bridge",
            )
            headband.visual(
                Box((0.008, 0.006, 0.048)),
                origin=Origin(xyz=(x_sign * 0.116, y_sign * 0.028, 0.092)),
                material=metal_dark,
                name=lug_name,
            )

    left_cup = model.part("left_cup")
    _add_earcup_geometry(
        left_cup,
        side="left",
        side_sign=-1,
        shell_material=plastic_dark,
        cushion_material=cushion_dark,
        fabric_material=fabric_dark,
        mic_material=metal_dark,
        with_mic_mount=True,
        front_boss_name="left_front_boss",
        rear_boss_name="left_rear_boss",
    )

    right_cup = model.part("right_cup")
    _add_earcup_geometry(
        right_cup,
        side="right",
        side_sign=1,
        shell_material=plastic_dark,
        cushion_material=cushion_dark,
        fabric_material=fabric_dark,
        mic_material=metal_dark,
        with_mic_mount=False,
        front_boss_name="right_front_boss",
        rear_boss_name="right_rear_boss",
    )

    mic_boom = model.part("mic_boom")
    mic_boom.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=metal_dark,
        name="boom_collar",
    )
    mic_boom.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.006, 0.0, 0.0),
                    (0.010, 0.030, -0.005),
                    (0.010, 0.084, -0.016),
                    (0.016, 0.136, -0.022),
                ],
                radius=0.0022,
                samples_per_segment=18,
                radial_segments=16,
                cap_ends=True,
            ),
            "microphone_boom_stem",
        ),
        material=metal_dark,
        name="boom_stem",
    )
    mic_boom.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.017, 0.126, -0.022), rpy=(math.pi / 2, 0.0, 0.0)),
        material=fabric_dark,
        name="microphone_capsule",
    )

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_cup,
        origin=Origin(xyz=(-0.116, 0.0, 0.092)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_cup,
        origin=Origin(xyz=(0.116, 0.0, 0.092)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "mic_hinge",
        ArticulationType.REVOLUTE,
        parent=left_cup,
        child=mic_boom,
        origin=Origin(xyz=(0.0, 0.023, -0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    headband = object_model.get_part("headband")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")
    mic_boom = object_model.get_part("mic_boom")

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    mic_hinge = object_model.get_articulation("mic_hinge")

    ctx.expect_contact(
        left_cup,
        headband,
        elem_a="left_front_boss",
        elem_b="left_front_lug",
        contact_tol=0.0005,
        name="left cup front hinge boss seats on yoke lug",
    )
    ctx.expect_contact(
        left_cup,
        headband,
        elem_a="left_rear_boss",
        elem_b="left_rear_lug",
        contact_tol=0.0005,
        name="left cup rear hinge boss seats on yoke lug",
    )
    ctx.expect_contact(
        right_cup,
        headband,
        elem_a="right_front_boss",
        elem_b="right_front_lug",
        contact_tol=0.0005,
        name="right cup front hinge boss seats on yoke lug",
    )
    ctx.expect_contact(
        right_cup,
        headband,
        elem_a="right_rear_boss",
        elem_b="right_rear_lug",
        contact_tol=0.0005,
        name="right cup rear hinge boss seats on yoke lug",
    )
    ctx.expect_contact(
        mic_boom,
        left_cup,
        elem_a="boom_collar",
        elem_b="mic_mount",
        contact_tol=0.0005,
        name="microphone boom collar mounts to left cup bracket",
    )

    left_rest = ctx.part_world_aabb(left_cup)
    right_rest = ctx.part_world_aabb(right_cup)
    with ctx.pose({left_fold: 1.35}):
        left_folded = ctx.part_world_aabb(left_cup)
    with ctx.pose({right_fold: 1.35}):
        right_folded = ctx.part_world_aabb(right_cup)

    left_rest_size = _aabb_size(left_rest)
    left_folded_size = _aabb_size(left_folded)
    right_rest_size = _aabb_size(right_rest)
    right_folded_size = _aabb_size(right_folded)

    ctx.check(
        "left earcup rotates toward a flat storage pose",
        left_rest_size is not None
        and left_folded_size is not None
        and left_folded_size[0] > left_rest_size[0] + 0.020
        and left_folded_size[2] < left_rest_size[2] - 0.020,
        details=f"rest={left_rest_size}, folded={left_folded_size}",
    )
    ctx.check(
        "right earcup rotates toward a flat storage pose",
        right_rest_size is not None
        and right_folded_size is not None
        and right_folded_size[0] > right_rest_size[0] + 0.020
        and right_folded_size[2] < right_rest_size[2] - 0.020,
        details=f"rest={right_rest_size}, folded={right_folded_size}",
    )

    boom_rest = ctx.part_element_world_aabb(mic_boom, elem="boom_stem")
    with ctx.pose({mic_hinge: 1.25}):
        boom_raised = ctx.part_element_world_aabb(mic_boom, elem="boom_stem")

    boom_rest_center = _aabb_center(boom_rest)
    boom_raised_center = _aabb_center(boom_raised)
    ctx.check(
        "microphone boom swings upward to stow",
        boom_rest_center is not None
        and boom_raised_center is not None
        and boom_raised_center[2] > boom_rest_center[2] + 0.035
        and boom_raised_center[1] < boom_rest_center[1],
        details=f"rest={boom_rest_center}, raised={boom_raised_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
