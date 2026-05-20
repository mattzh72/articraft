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
    section_loft,
    superellipse_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_loop(
    width: float,
    height: float,
    *,
    y: float,
    z_center: float,
    exponent: float = 2.4,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for x, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _build_headband(model: ArticulatedObject, *, metal, pad):
    headband = model.part("headband")

    outer_band_path = [
        (0.0, -0.088, 0.108),
        (0.0, -0.072, 0.146),
        (0.0, -0.040, 0.176),
        (0.0, 0.000, 0.188),
        (0.0, 0.040, 0.176),
        (0.0, 0.072, 0.146),
        (0.0, 0.088, 0.108),
    ]
    outer_band_geom = sweep_profile_along_spline(
        outer_band_path,
        profile=rounded_rect_profile(0.020, 0.010, radius=0.0045),
        samples_per_segment=16,
        up_hint=(1.0, 0.0, 0.0),
        cap_profile=True,
    )
    headband.visual(
        _mesh("headband_outer_band", outer_band_geom),
        material=metal,
        name="headband_outer_band",
    )

    inner_pad_path = [
        (0.0, -0.060, 0.138),
        (0.0, -0.034, 0.159),
        (0.0, 0.000, 0.167),
        (0.0, 0.034, 0.159),
        (0.0, 0.060, 0.138),
    ]
    inner_pad_geom = sweep_profile_along_spline(
        inner_pad_path,
        profile=rounded_rect_profile(0.034, 0.008, radius=0.0035),
        samples_per_segment=14,
        up_hint=(1.0, 0.0, 0.0),
        cap_profile=True,
    )
    headband.visual(
        _mesh("headband_inner_pad", inner_pad_geom),
        material=pad,
        name="headband_inner_pad",
    )

    headband.visual(
        Box((0.026, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.089, 0.108)),
        material=metal,
        name="left_hinge_block",
    )
    headband.visual(
        Box((0.018, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.089, 0.121)),
        material=metal,
        name="left_hinge_cap",
    )
    headband.visual(
        Box((0.026, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.089, 0.108)),
        material=metal,
        name="right_hinge_block",
    )
    headband.visual(
        Box((0.018, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.089, 0.121)),
        material=metal,
        name="right_hinge_cap",
    )

    headband.inertial = Inertial.from_geometry(
        Box((0.070, 0.210, 0.100)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
    )
    return headband


def _build_yoke(model: ArticulatedObject, name: str, *, metal, accent):
    yoke = model.part(name)

    strap_path = [
        (-0.031, 0.0, -0.018),
        (-0.031, 0.0, -0.041),
        (-0.029, 0.0, -0.064),
        (-0.020, 0.0, -0.080),
        (0.020, 0.0, -0.080),
        (0.029, 0.0, -0.064),
        (0.031, 0.0, -0.041),
        (0.031, 0.0, -0.018),
    ]
    strap_geom = sweep_profile_along_spline(
        strap_path,
        profile=rounded_rect_profile(0.0065, 0.0040, radius=0.0012),
        samples_per_segment=12,
        up_hint=(0.0, 1.0, 0.0),
        cap_profile=True,
    )
    yoke.visual(
        _mesh(f"{name}_strap", strap_geom),
        material=metal,
        name="yoke_strap",
    )
    yoke.visual(
        Box((0.072, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=accent,
        name="top_bridge_block",
    )
    yoke.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="top_hinge_collar",
    )
    yoke.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(-0.031, 0.0, -0.031), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="front_pivot_boss",
    )
    yoke.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.031, 0.0, -0.031), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="rear_pivot_boss",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.076, 0.016, 0.090)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
    )
    return yoke


def _build_earcup(
    model: ArticulatedObject,
    name: str,
    *,
    outer_sign: float,
    shell_material,
    pad_material,
    fabric_material,
    accent_material,
):
    cup = model.part(name)

    shell_sections = [
        _xz_loop(0.058, 0.078, y=outer_sign * 0.004, z_center=-0.040, exponent=2.45, segments=54),
        _xz_loop(0.074, 0.096, y=outer_sign * -0.014, z_center=-0.042, exponent=2.35, segments=54),
        _xz_loop(0.066, 0.090, y=outer_sign * -0.030, z_center=-0.044, exponent=2.15, segments=54),
    ]
    shell_geom = section_loft(shell_sections)
    cup.visual(
        _mesh(f"{name}_shell", shell_geom),
        material=shell_material,
        name="earcup_shell",
    )

    pad_outer = superellipse_profile(0.078, 0.102, exponent=2.1, segments=56)
    pad_opening = superellipse_profile(0.050, 0.072, exponent=2.0, segments=56)
    pad_geom = (
        ExtrudeWithHolesGeometry(pad_outer, [pad_opening], height=0.014, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, outer_sign * -0.030, -0.044)
    )
    cup.visual(
        _mesh(f"{name}_pad_ring", pad_geom),
        material=pad_material,
        name="ear_pad_ring",
    )

    baffle_geom = (
        ExtrudeGeometry(superellipse_profile(0.054, 0.078, exponent=2.0, segments=48), 0.0045, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, outer_sign * -0.030, -0.044)
    )
    cup.visual(
        _mesh(f"{name}_baffle", baffle_geom),
        material=fabric_material,
        name="inner_baffle",
    )

    outer_cap_geom = (
        ExtrudeGeometry(superellipse_profile(0.056, 0.078, exponent=2.2, segments=48), 0.006, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, outer_sign * 0.002, -0.038)
    )
    cup.visual(
        _mesh(f"{name}_outer_cap", outer_cap_geom),
        material=accent_material,
        name="outer_cap",
    )
    cup.visual(
        Box((0.066, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, outer_sign * 0.001, -0.010)),
        material=accent_material,
        name="pivot_spine",
    )

    cup.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(xyz=(-0.028, outer_sign * 0.001, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="front_trunnion",
    )
    cup.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(xyz=(0.028, outer_sign * 0.001, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="rear_trunnion",
    )

    cup.inertial = Inertial.from_geometry(
        Box((0.078, 0.038, 0.104)),
        mass=0.19,
        origin=Origin(xyz=(0.0, outer_sign * -0.014, -0.044)),
    )
    return cup


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_over_ear_headphones")

    aluminum = model.material("aluminum", rgba=(0.74, 0.75, 0.78, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.34, 0.36, 0.39, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    leather_black = model.material("leather_black", rgba=(0.11, 0.11, 0.12, 1.0))
    fabric_dark = model.material("fabric_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    headband = _build_headband(model, metal=dark_aluminum, pad=leather_black)
    left_yoke = _build_yoke(model, "left_yoke", metal=aluminum, accent=graphite)
    right_yoke = _build_yoke(model, "right_yoke", metal=aluminum, accent=graphite)
    left_earcup = _build_earcup(
        model,
        "left_earcup",
        outer_sign=1.0,
        shell_material=graphite,
        pad_material=leather_black,
        fabric_material=fabric_dark,
        accent_material=dark_aluminum,
    )
    right_earcup = _build_earcup(
        model,
        "right_earcup",
        outer_sign=-1.0,
        shell_material=graphite,
        pad_material=leather_black,
        fabric_material=fabric_dark,
        accent_material=dark_aluminum,
    )

    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.089, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.0, -0.089, 0.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "left_swivel_pivot",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "right_swivel_pivot",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=-0.55,
            upper=0.55,
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
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")

    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_swivel = object_model.get_articulation("left_swivel_pivot")
    right_swivel = object_model.get_articulation("right_swivel_pivot")

    ctx.expect_gap(
        headband,
        left_yoke,
        axis="z",
        positive_elem="left_hinge_block",
        negative_elem="top_bridge_block",
        min_gap=0.0,
        max_gap=0.0025,
        name="left yoke top bridge seats directly under the left hinge block",
    )
    ctx.expect_gap(
        headband,
        right_yoke,
        axis="z",
        positive_elem="right_hinge_block",
        negative_elem="top_bridge_block",
        min_gap=0.0,
        max_gap=0.0025,
        name="right yoke top bridge seats directly under the right hinge block",
    )

    rest_left_origin = ctx.part_world_position(left_earcup)
    rest_right_origin = ctx.part_world_position(right_earcup)

    with ctx.pose({left_fold: 0.95, right_swivel: 0.0, left_swivel: 0.0, right_fold: 0.0}):
        folded_left_origin = ctx.part_world_position(left_earcup)
    with ctx.pose({right_fold: 0.95, right_swivel: 0.0, left_swivel: 0.0, left_fold: 0.0}):
        folded_right_origin = ctx.part_world_position(right_earcup)

    ctx.check(
        "left fold hinge raises and repositions the left earcup",
        rest_left_origin is not None
        and folded_left_origin is not None
        and folded_left_origin[2] > rest_left_origin[2] + 0.010
        and abs(folded_left_origin[0] - rest_left_origin[0]) > 0.020,
        details=f"rest={rest_left_origin}, folded={folded_left_origin}",
    )
    ctx.check(
        "right fold hinge raises and repositions the right earcup",
        rest_right_origin is not None
        and folded_right_origin is not None
        and folded_right_origin[2] > rest_right_origin[2] + 0.010
        and abs(folded_right_origin[0] - rest_right_origin[0]) > 0.020,
        details=f"rest={rest_right_origin}, folded={folded_right_origin}",
    )

    rest_left_aabb = ctx.part_world_aabb(left_earcup)
    rest_right_aabb = ctx.part_world_aabb(right_earcup)
    with ctx.pose({left_swivel: 0.42, left_fold: 0.0, right_fold: 0.0, right_swivel: 0.0}):
        swiveled_left_aabb = ctx.part_world_aabb(left_earcup)
    with ctx.pose({right_swivel: -0.42, left_fold: 0.0, right_fold: 0.0, left_swivel: 0.0}):
        swiveled_right_aabb = ctx.part_world_aabb(right_earcup)

    left_swivel_changed = (
        rest_left_aabb is not None
        and swiveled_left_aabb is not None
        and max(
            abs(swiveled_left_aabb[0][1] - rest_left_aabb[0][1]),
            abs(swiveled_left_aabb[1][1] - rest_left_aabb[1][1]),
            abs(swiveled_left_aabb[0][2] - rest_left_aabb[0][2]),
            abs(swiveled_left_aabb[1][2] - rest_left_aabb[1][2]),
        )
        > 0.010
    )
    right_swivel_changed = (
        rest_right_aabb is not None
        and swiveled_right_aabb is not None
        and max(
            abs(swiveled_right_aabb[0][1] - rest_right_aabb[0][1]),
            abs(swiveled_right_aabb[1][1] - rest_right_aabb[1][1]),
            abs(swiveled_right_aabb[0][2] - rest_right_aabb[0][2]),
            abs(swiveled_right_aabb[1][2] - rest_right_aabb[1][2]),
        )
        > 0.010
    )

    ctx.check(
        "left earcup rotates on the inner yoke swivel pivot",
        left_swivel_changed,
        details=f"rest={rest_left_aabb}, swiveled={swiveled_left_aabb}",
    )
    ctx.check(
        "right earcup rotates on the inner yoke swivel pivot",
        right_swivel_changed,
        details=f"rest={rest_right_aabb}, swiveled={swiveled_right_aabb}",
    )

    ctx.expect_origin_gap(
        left_yoke,
        right_yoke,
        axis="y",
        min_gap=0.160,
        name="the yokes span a realistic over-ear headband width",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
