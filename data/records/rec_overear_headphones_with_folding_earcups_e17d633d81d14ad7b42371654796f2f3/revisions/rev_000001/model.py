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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_yoke_visuals(part, *, sign: float, material) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="fold_barrel",
    )
    part.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(sign * 0.008, 0.0, -0.004)),
        material=material,
        name="upper_bridge",
    )
    part.visual(
        Box((0.006, 0.012, 0.050)),
        origin=Origin(xyz=(sign * 0.015, 0.0, -0.029)),
        material=material,
        name="center_stem",
    )
    part.visual(
        Box((0.008, 0.050, 0.010)),
        origin=Origin(xyz=(sign * 0.012, 0.0, -0.026)),
        material=material,
        name="upper_crossbar",
    )
    part.visual(
        Box((0.008, 0.010, 0.044)),
        origin=Origin(xyz=(sign * 0.012, 0.021, -0.048)),
        material=material,
        name="front_arm",
    )
    part.visual(
        Box((0.008, 0.010, 0.044)),
        origin=Origin(xyz=(sign * 0.012, -0.021, -0.048)),
        material=material,
        name="rear_arm",
    )
    part.visual(
        Box((0.008, 0.050, 0.010)),
        origin=Origin(xyz=(sign * 0.012, 0.0, -0.070)),
        material=material,
        name="lower_crossbar",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(sign * 0.012, 0.0, -0.026)),
        material=material,
        name="upper_pivot_boss",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(sign * 0.012, 0.0, -0.070)),
        material=material,
        name="lower_pivot_boss",
    )


def _add_earcup_visuals(
    part,
    *,
    sign: float,
    shell_mesh,
    pad_mesh,
    shell_material,
    pad_material,
    accent_material,
) -> None:
    part.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.006 * sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="swivel_hub",
    )
    part.visual(
        Box((0.016, 0.014, 0.024)),
        origin=Origin(xyz=(0.012 * sign, 0.0, 0.0)),
        material=accent_material,
        name="hub_spine",
    )
    part.visual(
        shell_mesh,
        origin=Origin(xyz=(0.020 * sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_material,
        name="shell",
    )
    part.visual(
        pad_mesh,
        origin=Origin(xyz=(0.030 * sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pad_material,
        name="pad_ring",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.004 * sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="outer_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_wireless_headphones")

    shell_black = model.material("shell_black", rgba=(0.14, 0.15, 0.17, 1.0))
    pad_black = model.material("pad_black", rgba=(0.09, 0.09, 0.10, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.27, 0.29, 0.31, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.46, 0.48, 0.51, 1.0))

    outer_band_mesh = _mesh(
        "headband_outer",
        sweep_profile_along_spline(
            [
                (-0.080, 0.0, -0.080),
                (-0.072, 0.0, -0.052),
                (-0.050, 0.0, -0.022),
                (-0.020, 0.0, -0.005),
                (0.0, 0.0, 0.0),
                (0.020, 0.0, -0.005),
                (0.050, 0.0, -0.022),
                (0.072, 0.0, -0.052),
                (0.080, 0.0, -0.080),
            ],
            profile=rounded_rect_profile(0.030, 0.014, 0.005, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )
    inner_band_mesh = _mesh(
        "headband_pad",
        sweep_profile_along_spline(
            [
                (-0.066, 0.0, -0.081),
                (-0.046, 0.0, -0.060),
                (-0.020, 0.0, -0.041),
                (0.0, 0.0, -0.034),
                (0.020, 0.0, -0.041),
                (0.046, 0.0, -0.060),
                (0.066, 0.0, -0.081),
            ],
            profile=rounded_rect_profile(0.022, 0.008, 0.003, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )
    earcup_shell_mesh = _mesh(
        "earcup_shell",
        ExtrudeGeometry(
            rounded_rect_profile(0.074, 0.088, 0.016, corner_segments=8),
            0.028,
            center=True,
        ),
    )
    ear_pad_mesh = _mesh(
        "earcup_pad_ring",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.078, 0.094, 0.018, corner_segments=8),
            [rounded_rect_profile(0.046, 0.060, 0.014, corner_segments=8)],
            0.014,
            center=True,
        ),
    )
    top_handle_block_mesh = _mesh(
        "top_handle_block",
        ExtrudeGeometry(
            rounded_rect_profile(0.082, 0.030, 0.010, corner_segments=8),
            0.018,
            center=True,
        ),
    )
    hinge_housing_mesh = _mesh(
        "hinge_housing",
        ExtrudeGeometry(
            rounded_rect_profile(0.018, 0.028, 0.006, corner_segments=8),
            0.018,
            center=True,
        ),
    )
    carry_loop_mesh = _mesh(
        "carry_loop_wire",
        tube_from_spline_points(
            [
                (-0.023, 0.000, 0.000),
                (-0.020, 0.010, 0.004),
                (-0.012, 0.018, 0.007),
                (0.000, 0.022, 0.008),
                (0.012, 0.018, 0.007),
                (0.020, 0.010, 0.004),
                (0.023, 0.000, 0.000),
            ],
            radius=0.0035,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    headband = model.part("headband")
    headband.visual(outer_band_mesh, material=shell_black, name="band_outer")
    headband.visual(inner_band_mesh, material=pad_black, name="band_pad")
    headband.visual(
        top_handle_block_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=shell_black,
        name="top_handle_block",
    )
    for x_sign in (-1.0, 1.0):
        headband.visual(
            hinge_housing_mesh,
            origin=Origin(xyz=(x_sign * 0.100, 0.0, -0.058)),
            material=shell_black,
            name=f"{'left' if x_sign < 0.0 else 'right'}_hinge_housing",
        )
        headband.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(
                xyz=(x_sign * 0.100, 0.012, -0.080),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name=f"{'left' if x_sign < 0.0 else 'right'}_hinge_knuckle_front",
        )
        headband.visual(
            Box((0.010, 0.010, 0.024)),
            origin=Origin(xyz=(x_sign * 0.100, 0.012, -0.069)),
            material=shell_black,
            name=f"{'left' if x_sign < 0.0 else 'right'}_hinge_cheek_front",
        )
        headband.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(
                xyz=(x_sign * 0.100, -0.012, -0.080),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name=f"{'left' if x_sign < 0.0 else 'right'}_hinge_knuckle_rear",
        )
        headband.visual(
            Box((0.010, 0.010, 0.024)),
            origin=Origin(xyz=(x_sign * 0.100, -0.012, -0.069)),
            material=shell_black,
            name=f"{'left' if x_sign < 0.0 else 'right'}_hinge_cheek_rear",
        )
    headband.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(-0.034, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="carry_pivot_left",
    )
    headband.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.034, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="carry_pivot_right",
    )
    headband.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(-0.034, 0.0, 0.011)),
        material=shell_black,
        name="carry_support_left",
    )
    headband.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(0.034, 0.0, 0.011)),
        material=shell_black,
        name="carry_support_right",
    )
    headband.inertial = Inertial.from_geometry(
        Box((0.180, 0.040, 0.170)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
    )

    carry_loop = model.part("carry_loop")
    carry_loop.visual(carry_loop_mesh, material=metal_dark, name="loop_wire")
    carry_loop.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_gray,
        name="left_stub",
    )
    carry_loop.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_gray,
        name="right_stub",
    )
    carry_loop.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.011, 0.004)),
    )

    left_yoke = model.part("left_yoke")
    _add_yoke_visuals(left_yoke, sign=-1.0, material=metal_dark)
    left_yoke.inertial = Inertial.from_geometry(
        Box((0.024, 0.060, 0.070)),
        mass=0.025,
        origin=Origin(xyz=(-0.006, 0.0, -0.040)),
    )

    right_yoke = model.part("right_yoke")
    _add_yoke_visuals(right_yoke, sign=1.0, material=metal_dark)
    right_yoke.inertial = Inertial.from_geometry(
        Box((0.024, 0.060, 0.070)),
        mass=0.025,
        origin=Origin(xyz=(0.006, 0.0, -0.040)),
    )

    left_earcup = model.part("left_earcup")
    _add_earcup_visuals(
        left_earcup,
        sign=1.0,
        shell_mesh=earcup_shell_mesh,
        pad_mesh=ear_pad_mesh,
        shell_material=shell_black,
        pad_material=pad_black,
        accent_material=accent_gray,
    )
    left_earcup.inertial = Inertial.from_geometry(
        Box((0.038, 0.082, 0.094)),
        mass=0.11,
    )

    right_earcup = model.part("right_earcup")
    _add_earcup_visuals(
        right_earcup,
        sign=-1.0,
        shell_mesh=earcup_shell_mesh,
        pad_mesh=ear_pad_mesh,
        shell_material=shell_black,
        pad_material=pad_black,
        accent_material=accent_gray,
    )
    right_earcup.inertial = Inertial.from_geometry(
        Box((0.038, 0.082, 0.094)),
        mass=0.11,
    )

    model.articulation(
        "headband_to_carry_loop",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=carry_loop,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "headband_to_left_yoke",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.100, 0.0, -0.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "headband_to_right_yoke",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.100, 0.0, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "left_yoke_to_left_earcup",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(-0.012, 0.0, -0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "right_yoke_to_right_earcup",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.012, 0.0, -0.048)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headband = object_model.get_part("headband")
    carry_loop = object_model.get_part("carry_loop")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")

    carry_joint = object_model.get_articulation("headband_to_carry_loop")
    left_fold = object_model.get_articulation("headband_to_left_yoke")
    right_fold = object_model.get_articulation("headband_to_right_yoke")
    left_swivel = object_model.get_articulation("left_yoke_to_left_earcup")
    right_swivel = object_model.get_articulation("right_yoke_to_right_earcup")

    ctx.expect_contact(
        headband,
        left_yoke,
        contact_tol=0.0008,
        name="left yoke seats in the headband hinge",
    )
    ctx.expect_contact(
        headband,
        right_yoke,
        contact_tol=0.0008,
        name="right yoke seats in the headband hinge",
    )
    ctx.expect_contact(
        left_yoke,
        left_earcup,
        contact_tol=0.0008,
        name="left earcup is supported by its yoke pivots",
    )
    ctx.expect_contact(
        right_yoke,
        right_earcup,
        contact_tol=0.0008,
        name="right earcup is supported by its yoke pivots",
    )
    ctx.expect_contact(
        headband,
        carry_loop,
        contact_tol=0.0008,
        name="carry loop sits on its headband pivots",
    )

    left_shell_rest = ctx.part_element_world_aabb(left_earcup, elem="shell")
    right_shell_rest = ctx.part_element_world_aabb(right_earcup, elem="shell")
    with ctx.pose({left_fold: 1.10}):
        left_shell_folded = ctx.part_element_world_aabb(left_earcup, elem="shell")
    with ctx.pose({right_fold: 1.10}):
        right_shell_folded = ctx.part_element_world_aabb(right_earcup, elem="shell")
    left_rest_z = None if left_shell_rest is None else (left_shell_rest[0][2] + left_shell_rest[1][2]) * 0.5
    left_folded_z = (
        None if left_shell_folded is None else (left_shell_folded[0][2] + left_shell_folded[1][2]) * 0.5
    )
    right_rest_z = (
        None if right_shell_rest is None else (right_shell_rest[0][2] + right_shell_rest[1][2]) * 0.5
    )
    right_folded_z = (
        None
        if right_shell_folded is None
        else (right_shell_folded[0][2] + right_shell_folded[1][2]) * 0.5
    )
    ctx.check(
        "left earcup folds upward",
        left_rest_z is not None and left_folded_z is not None and left_folded_z > left_rest_z + 0.020,
        details=f"rest_z={left_rest_z}, folded_z={left_folded_z}",
    )
    ctx.check(
        "right earcup folds upward",
        right_rest_z is not None and right_folded_z is not None and right_folded_z > right_rest_z + 0.020,
        details=f"rest_z={right_rest_z}, folded_z={right_folded_z}",
    )

    left_pad_rest = ctx.part_element_world_aabb(left_earcup, elem="pad_ring")
    right_pad_rest = ctx.part_element_world_aabb(right_earcup, elem="pad_ring")
    with ctx.pose({left_swivel: 0.75}):
        left_pad_swiveled = ctx.part_element_world_aabb(left_earcup, elem="pad_ring")
    with ctx.pose({right_swivel: 0.75}):
        right_pad_swiveled = ctx.part_element_world_aabb(right_earcup, elem="pad_ring")
    left_rest_y = None if left_pad_rest is None else (left_pad_rest[0][1] + left_pad_rest[1][1]) * 0.5
    left_swiveled_y = (
        None if left_pad_swiveled is None else (left_pad_swiveled[0][1] + left_pad_swiveled[1][1]) * 0.5
    )
    right_rest_y = None if right_pad_rest is None else (right_pad_rest[0][1] + right_pad_rest[1][1]) * 0.5
    right_swiveled_y = (
        None
        if right_pad_swiveled is None
        else (right_pad_swiveled[0][1] + right_pad_swiveled[1][1]) * 0.5
    )
    ctx.check(
        "left earcup swivel changes cup facing",
        left_rest_y is not None and left_swiveled_y is not None and left_swiveled_y > left_rest_y + 0.004,
        details=f"rest_y={left_rest_y}, swiveled_y={left_swiveled_y}",
    )
    ctx.check(
        "right earcup swivel changes cup facing",
        right_rest_y is not None and right_swiveled_y is not None and right_swiveled_y > right_rest_y + 0.004,
        details=f"rest_y={right_rest_y}, swiveled_y={right_swiveled_y}",
    )

    carry_rest = ctx.part_world_aabb(carry_loop)
    with ctx.pose({carry_joint: 1.10}):
        carry_open = ctx.part_world_aabb(carry_loop)
    carry_rest_top = None if carry_rest is None else carry_rest[1][2]
    carry_open_top = None if carry_open is None else carry_open[1][2]
    ctx.check(
        "carry loop folds out above the headband",
        carry_rest_top is not None and carry_open_top is not None and carry_open_top > carry_rest_top + 0.012,
        details=f"rest_top={carry_rest_top}, open_top={carry_open_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
