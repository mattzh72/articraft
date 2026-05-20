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


def _aabb_dims(aabb):
    if aabb is None:
        return None
    (mn_x, mn_y, mn_z), (mx_x, mx_y, mx_z) = aabb
    return (mx_x - mn_x, mx_y - mn_y, mx_z - mn_z)


def _ear_pad_ring_mesh() -> object:
    oval_points = [
        (0.0, 0.000, 0.041),
        (0.0, 0.022, 0.029),
        (0.0, 0.031, 0.000),
        (0.0, 0.022, -0.029),
        (0.0, 0.000, -0.041),
        (0.0, -0.022, -0.029),
        (0.0, -0.031, 0.000),
        (0.0, -0.022, 0.029),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            oval_points,
            radius=0.008,
            samples_per_segment=10,
            closed_spline=True,
            radial_segments=18,
            cap_ends=False,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "ear_pad_ring",
    )


def _earcup_shell_mesh() -> object:
    shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.086, 0.098, radius=0.022, corner_segments=8),
        [rounded_rect_profile(0.056, 0.068, radius=0.015, corner_segments=8)],
        0.028,
        center=True,
        cap=True,
        closed=True,
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, "earcup_shell")


def _earcup_back_cap_mesh() -> object:
    rear_cap = ExtrudeGeometry(
        rounded_rect_profile(0.078, 0.090, radius=0.020, corner_segments=8),
        0.008,
        center=True,
        cap=True,
        closed=True,
    )
    rear_cap.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(rear_cap, "earcup_back_cap")


def _driver_baffle_mesh() -> object:
    baffle = ExtrudeGeometry(
        rounded_rect_profile(0.060, 0.074, radius=0.017, corner_segments=8),
        0.003,
        center=True,
        cap=True,
        closed=True,
    )
    baffle.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(baffle, "driver_baffle")


def _pad_retainer_mesh() -> object:
    retainer = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.074, 0.088, radius=0.019, corner_segments=8),
        [rounded_rect_profile(0.050, 0.062, radius=0.013, corner_segments=8)],
        0.004,
        center=True,
        cap=True,
        closed=True,
    )
    retainer.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(retainer, "pad_retainer")


def _headband_outer_mesh() -> object:
    arch_points = [
        (-0.102, 0.0, 0.060),
        (-0.088, 0.0, 0.094),
        (-0.050, 0.0, 0.118),
        (0.000, 0.0, 0.130),
        (0.050, 0.0, 0.118),
        (0.088, 0.0, 0.094),
        (0.102, 0.0, 0.060),
    ]
    return mesh_from_geometry(
        sweep_profile_along_spline(
            arch_points,
            profile=rounded_rect_profile(0.040, 0.011, radius=0.0045, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "headband_outer",
    )


def _headband_pad_mesh() -> object:
    pad_points = [
        (-0.078, 0.0, 0.050),
        (-0.068, 0.0, 0.074),
        (-0.044, 0.0, 0.095),
        (0.000, 0.0, 0.104),
        (0.044, 0.0, 0.095),
        (0.068, 0.0, 0.074),
        (0.078, 0.0, 0.050),
    ]
    return mesh_from_geometry(
        sweep_profile_along_spline(
            pad_points,
            profile=rounded_rect_profile(0.031, 0.016, radius=0.007, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "headband_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_headphones")

    plastic_black = model.material("plastic_black", rgba=(0.08, 0.08, 0.09, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    metal_gun = model.material("metal_gun", rgba=(0.38, 0.40, 0.44, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_silver = model.material("accent_silver", rgba=(0.70, 0.72, 0.75, 1.0))

    headband_outer_mesh = _headband_outer_mesh()
    headband_pad_mesh = _headband_pad_mesh()
    ear_pad_ring_mesh = _ear_pad_ring_mesh()
    earcup_shell_mesh = _earcup_shell_mesh()
    earcup_back_cap_mesh = _earcup_back_cap_mesh()
    driver_baffle_mesh = _driver_baffle_mesh()
    pad_retainer_mesh = _pad_retainer_mesh()

    headband = model.part("headband")
    headband.visual(headband_outer_mesh, material=plastic_dark, name="outer_band")
    headband.visual(headband_pad_mesh, material=cushion_black, name="underside_pad")
    for side in (-1.0, 1.0):
        prefix = "left" if side < 0.0 else "right"
        x_end = side * 0.102
        headband.visual(
            Box((0.018, 0.008, 0.056)),
            origin=Origin(xyz=(x_end, 0.009, 0.028)),
            material=plastic_dark,
            name=f"{prefix}_guide_front",
        )
        headband.visual(
            Box((0.018, 0.008, 0.056)),
            origin=Origin(xyz=(x_end, -0.009, 0.028)),
            material=plastic_dark,
            name=f"{prefix}_guide_back",
        )
        headband.visual(
            Box((0.018, 0.026, 0.006)),
            origin=Origin(xyz=(x_end, 0.0, 0.057)),
            material=plastic_dark,
            name=f"{prefix}_guide_bridge",
        )
        headband.visual(
            Box((0.014, 0.026, 0.010)),
            origin=Origin(xyz=(side * 0.100, 0.0, 0.062)),
            material=plastic_dark,
            name=f"{prefix}_end_cap",
        )
    headband.inertial = Inertial.from_geometry(
        Box((0.240, 0.050, 0.150)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    def build_slider(part_name: str) -> object:
        slider = model.part(part_name)
        slider.visual(
            Box((0.010, 0.010, 0.092)),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=metal_gun,
            name="slider_stem",
        )
        slider.visual(
            Box((0.020, 0.024, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.080)),
            material=plastic_black,
            name="hinge_block",
        )
        slider.visual(
            Cylinder(radius=0.005, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, -0.074), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent_silver,
            name="fold_pin_cover",
        )
        slider.inertial = Inertial.from_geometry(
            Box((0.020, 0.024, 0.098)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
        )
        return slider

    left_slider = build_slider("left_slider")
    right_slider = build_slider("right_slider")

    def build_yoke(part_name: str) -> object:
        yoke = model.part(part_name)
        yoke.visual(
            Box((0.018, 0.124, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=plastic_black,
            name="top_bridge",
        )
        yoke.visual(
            Box((0.012, 0.010, 0.052)),
            origin=Origin(xyz=(0.0, 0.057, -0.036)),
            material=plastic_black,
            name="front_arm",
        )
        yoke.visual(
            Box((0.012, 0.010, 0.052)),
            origin=Origin(xyz=(0.0, -0.057, -0.036)),
            material=plastic_black,
            name="rear_arm",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(xyz=(0.0, 0.057, -0.061), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=accent_silver,
            name="front_pivot_cap",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(xyz=(0.0, -0.057, -0.061), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=accent_silver,
            name="rear_pivot_cap",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((0.024, 0.130, 0.074)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, -0.036)),
        )
        return yoke

    left_yoke = build_yoke("left_yoke")
    right_yoke = build_yoke("right_yoke")

    def build_earcup(part_name: str, inward_sign: float) -> object:
        cup = model.part(part_name)
        x_open = inward_sign * 0.018
        cup.visual(
            earcup_shell_mesh,
            material=plastic_black,
            name="cup_shell",
        )
        cup.visual(
            earcup_back_cap_mesh,
            origin=Origin(xyz=(-inward_sign * 0.011, 0.0, 0.0)),
            material=plastic_dark,
            name="rear_cap",
        )
        cup.visual(
            driver_baffle_mesh,
            origin=Origin(xyz=(inward_sign * 0.006, 0.0, 0.0)),
            material=plastic_dark,
            name="driver_baffle",
        )
        cup.visual(
            ear_pad_ring_mesh,
            origin=Origin(xyz=(x_open, 0.0, 0.0)),
            material=cushion_black,
            name="ear_pad",
        )
        cup.visual(
            Cylinder(radius=0.0045, length=0.104),
            origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_gun,
            name="pivot_axle",
        )
        cup.visual(
            pad_retainer_mesh,
            origin=Origin(xyz=(inward_sign * 0.013, 0.0, 0.0)),
            material=accent_silver,
            name="pad_retainer_ring",
        )
        cup.inertial = Inertial.from_geometry(
            Cylinder(radius=0.050, length=0.040),
            mass=0.085,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        return cup

    left_cup = build_earcup("left_earcup", inward_sign=1.0)
    right_cup = build_earcup("right_earcup", inward_sign=-1.0)

    model.articulation(
        "headband_to_left_slider",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_slider,
        origin=Origin(xyz=(-0.102, 0.0, 0.046)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.08, lower=0.0, upper=0.030),
    )
    model.articulation(
        "headband_to_right_slider",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_slider,
        origin=Origin(xyz=(0.102, 0.0, 0.046)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.08, lower=0.0, upper=0.030),
    )

    model.articulation(
        "left_slider_to_yoke",
        ArticulationType.REVOLUTE,
        parent=left_slider,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "right_slider_to_yoke",
        ArticulationType.REVOLUTE,
        parent=right_slider,
        child=right_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    model.articulation(
        "left_yoke_to_earcup",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "right_yoke_to_earcup",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
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
    left_slider = object_model.get_part("left_slider")
    right_slider = object_model.get_part("right_slider")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_earcup")
    right_cup = object_model.get_part("right_earcup")

    left_slide = object_model.get_articulation("headband_to_left_slider")
    right_slide = object_model.get_articulation("headband_to_right_slider")
    left_fold = object_model.get_articulation("left_slider_to_yoke")
    right_fold = object_model.get_articulation("right_slider_to_yoke")
    left_swivel = object_model.get_articulation("left_yoke_to_earcup")
    right_swivel = object_model.get_articulation("right_yoke_to_earcup")

    for part_name in (
        "headband",
        "left_slider",
        "right_slider",
        "left_yoke",
        "right_yoke",
        "left_earcup",
        "right_earcup",
    ):
        ctx.check(
            f"{part_name} exists",
            object_model.get_part(part_name) is not None,
            details=f"missing {part_name}",
        )

    ctx.check(
        "left fold axis pitches inward",
        tuple(left_fold.axis) == (0.0, -1.0, 0.0),
        details=f"axis={left_fold.axis}",
    )
    ctx.check(
        "right fold axis pitches inward",
        tuple(right_fold.axis) == (0.0, 1.0, 0.0),
        details=f"axis={right_fold.axis}",
    )
    ctx.check(
        "both sliders extend downward",
        tuple(left_slide.axis) == (0.0, 0.0, -1.0) and tuple(right_slide.axis) == (0.0, 0.0, -1.0),
        details=f"left={left_slide.axis}, right={right_slide.axis}",
    )
    ctx.check(
        "cup swivels use front-back pivots",
        tuple(left_swivel.axis) == (0.0, 1.0, 0.0) and tuple(right_swivel.axis) == (0.0, 1.0, 0.0),
        details=f"left={left_swivel.axis}, right={right_swivel.axis}",
    )

    left_slider_rest = ctx.part_world_position(left_slider)
    right_slider_rest = ctx.part_world_position(right_slider)
    with ctx.pose({left_slide: 0.030, right_slide: 0.030}):
        left_slider_extended = ctx.part_world_position(left_slider)
        right_slider_extended = ctx.part_world_position(right_slider)

    ctx.check(
        "left slider extends downward from headband",
        left_slider_rest is not None
        and left_slider_extended is not None
        and left_slider_extended[2] < left_slider_rest[2] - 0.020,
        details=f"rest={left_slider_rest}, extended={left_slider_extended}",
    )
    ctx.check(
        "right slider extends downward from headband",
        right_slider_rest is not None
        and right_slider_extended is not None
        and right_slider_extended[2] < right_slider_rest[2] - 0.020,
        details=f"rest={right_slider_rest}, extended={right_slider_extended}",
    )

    left_cup_rest = ctx.part_world_position(left_cup)
    right_cup_rest = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: math.radians(100.0), right_fold: math.radians(100.0)}):
        left_cup_folded = ctx.part_world_position(left_cup)
        right_cup_folded = ctx.part_world_position(right_cup)
    ctx.check(
        "left cup folds inward for storage",
        left_cup_rest is not None
        and left_cup_folded is not None
        and left_cup_folded[0] > left_cup_rest[0] + 0.040
        and left_cup_folded[2] > left_cup_rest[2] + 0.015,
        details=f"rest={left_cup_rest}, folded={left_cup_folded}",
    )
    ctx.check(
        "right cup folds inward for storage",
        right_cup_rest is not None
        and right_cup_folded is not None
        and right_cup_folded[0] < right_cup_rest[0] - 0.040
        and right_cup_folded[2] > right_cup_rest[2] + 0.015,
        details=f"rest={right_cup_rest}, folded={right_cup_folded}",
    )

    left_rest_dims = _aabb_dims(ctx.part_world_aabb(left_cup))
    right_rest_dims = _aabb_dims(ctx.part_world_aabb(right_cup))
    with ctx.pose({left_swivel: math.radians(90.0), right_swivel: -math.radians(90.0)}):
        left_swivel_dims = _aabb_dims(ctx.part_world_aabb(left_cup))
        right_swivel_dims = _aabb_dims(ctx.part_world_aabb(right_cup))
    ctx.check(
        "left cup swivel turns shell out of the side-facing orientation",
        left_rest_dims is not None
        and left_swivel_dims is not None
        and left_swivel_dims[0] > left_rest_dims[0] * 1.25
        and left_swivel_dims[2] < left_rest_dims[2] * 0.80,
        details=f"rest={left_rest_dims}, swivel={left_swivel_dims}",
    )
    ctx.check(
        "right cup swivel turns shell out of the side-facing orientation",
        right_rest_dims is not None
        and right_swivel_dims is not None
        and right_swivel_dims[0] > right_rest_dims[0] * 1.25
        and right_swivel_dims[2] < right_rest_dims[2] * 0.80,
        details=f"rest={right_rest_dims}, swivel={right_swivel_dims}",
    )

    with ctx.pose(
        {
            left_slide: 0.020,
            right_slide: 0.020,
            left_fold: math.radians(95.0),
            right_fold: math.radians(95.0),
        }
    ):
        left_fold_storage = ctx.part_world_position(left_cup)
        right_fold_storage = ctx.part_world_position(right_cup)
        ctx.check(
            "stored left cup remains below headband centerline",
            left_fold_storage is not None and left_fold_storage[2] < 0.090,
            details=f"storage={left_fold_storage}",
        )
        ctx.check(
            "stored right cup remains below headband centerline",
            right_fold_storage is not None and right_fold_storage[2] < 0.090,
            details=f"storage={right_fold_storage}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
