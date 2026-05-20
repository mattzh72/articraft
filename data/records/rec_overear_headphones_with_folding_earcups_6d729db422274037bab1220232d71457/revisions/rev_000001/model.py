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
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_loop(
    x: float,
    width_y: float,
    height_z: float,
    *,
    exponent: float = 2.8,
    segments: int = 52,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for y, z in superellipse_profile(
            width_y,
            height_z,
            exponent=exponent,
            segments=segments,
        )
    ]


def _build_headband_mesh(name: str):
    band_path = [
        (-0.102, 0.0, 0.164),
        (-0.086, 0.0, 0.188),
        (-0.048, 0.0, 0.210),
        (0.000, 0.0, 0.220),
        (0.048, 0.0, 0.210),
        (0.086, 0.0, 0.188),
        (0.102, 0.0, 0.164),
    ]
    band_profile = rounded_rect_profile(0.042, 0.010, 0.004, corner_segments=6)
    return _mesh(
        name,
        sweep_profile_along_spline(
            band_path,
            profile=band_profile,
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )


def _build_headband_pad_mesh(name: str):
    pad_path = [
        (-0.062, 0.0, 0.194),
        (-0.052, 0.0, 0.201),
        (-0.032, 0.0, 0.204),
        (0.000, 0.0, 0.208),
        (0.032, 0.0, 0.204),
        (0.052, 0.0, 0.201),
        (0.062, 0.0, 0.194),
    ]
    pad_profile = rounded_rect_profile(0.028, 0.008, 0.003, corner_segments=6)
    return _mesh(
        name,
        sweep_profile_along_spline(
            pad_path,
            profile=pad_profile,
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )


def _build_earcup_shell_mesh(name: str):
    sections = [
        _yz_loop(-0.018, 0.050, 0.070, exponent=2.5),
        _yz_loop(-0.006, 0.066, 0.092, exponent=2.9),
        _yz_loop(0.010, 0.064, 0.090, exponent=2.9),
        _yz_loop(0.022, 0.054, 0.078, exponent=2.6),
    ]
    return _mesh(name, section_loft(sections))


def _build_pad_ring_mesh(name: str):
    outer = superellipse_profile(0.078, 0.098, exponent=2.8, segments=52)
    inner = superellipse_profile(0.040, 0.058, exponent=2.5, segments=52)
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            height=0.020,
            center=True,
            cap=True,
            closed=True,
        ).rotate_y(math.pi / 2.0),
    )


def _add_slider_geometry(part, *, metal, accent) -> None:
    part.visual(
        Box((0.010, 0.014, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=metal,
        name="arm_bar",
    )
    for index, ridge_z in enumerate((0.020, 0.008, -0.004)):
        part.visual(
            Box((0.014, 0.008, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, ridge_z)),
            material=accent,
            name=f"detent_ridge_{index}",
        )
    part.visual(
        Box((0.012, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=metal,
        name="hinge_neck",
    )
    part.visual(
        Box((0.012, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, 0.011, -0.046)),
        material=metal,
        name="hinge_cheek_front",
    )
    part.visual(
        Box((0.012, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, -0.011, -0.046)),
        material=metal,
        name="hinge_cheek_back",
    )


def _add_earcup_geometry(part, *, sign: float, shell_mesh, pad_mesh, shell, trim, cushion) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.016, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=trim,
        name="hinge_saddle",
    )
    part.visual(
        Box((0.024, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=trim,
        name="yoke_bridge",
    )
    for suffix, y in (("front", 0.029), ("back", -0.029)):
        part.visual(
            Box((0.006, 0.010, 0.042)),
            origin=Origin(xyz=(0.0, y, -0.039)),
            material=trim,
            name=f"fork_arm_{suffix}",
        )
        part.visual(
            Box((0.012, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, y, -0.062)),
            material=trim,
            name=f"fork_lug_{suffix}",
        )

    part.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=shell,
        name="cup_shell",
    )
    part.visual(
        pad_mesh,
        origin=Origin(
            xyz=(-0.018 * sign, 0.0, -0.060),
            rpy=(0.0, 0.0, math.pi / 2.0),
        ),
        material=cushion,
        name="ear_pad",
    )
    part.visual(
        Box((0.003, 0.044, 0.068)),
        origin=Origin(xyz=(-0.012 * sign, 0.0, -0.060)),
        material=trim,
        name="speaker_baffle",
    )
    part.visual(
        Box((0.012, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=trim,
        name="cup_top_cap",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_headphones")

    shell_black = model.material("shell_black", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    cushion_black = model.material("cushion_black", rgba=(0.09, 0.09, 0.10, 1.0))
    pad_gray = model.material("pad_gray", rgba=(0.22, 0.23, 0.24, 1.0))

    headband_mesh = _build_headband_mesh("headband_outer")
    headband_pad_mesh = _build_headband_pad_mesh("headband_pad")
    earcup_shell_mesh = _build_earcup_shell_mesh("earcup_shell")
    earpad_mesh = _build_pad_ring_mesh("earpad_ring")

    model.meta["headphone_style"] = "studio_monitor"

    headband = model.part("headband")
    headband.visual(headband_mesh, material=shell_black, name="headband_outer")
    headband.visual(headband_pad_mesh, material=pad_gray, name="headband_pad")
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        side_x = 0.102 * side_sign
        headband.visual(
            Box((0.010, 0.006, 0.068)),
            origin=Origin(xyz=(side_x, 0.010, 0.129)),
            material=metal_dark,
            name=f"{side_name}_sleeve_front",
        )
        headband.visual(
            Box((0.010, 0.006, 0.068)),
            origin=Origin(xyz=(side_x, -0.010, 0.129)),
            material=metal_dark,
            name=f"{side_name}_sleeve_back",
        )
        headband.visual(
            Box((0.018, 0.006, 0.014)),
            origin=Origin(xyz=(side_x, 0.010, 0.158)),
            material=metal_dark,
            name=f"{side_name}_sleeve_front_top",
        )
        headband.visual(
            Box((0.018, 0.006, 0.014)),
            origin=Origin(xyz=(side_x, -0.010, 0.158)),
            material=metal_dark,
            name=f"{side_name}_sleeve_back_top",
        )
        headband.visual(
            Box((0.024, 0.010, 0.024)),
            origin=Origin(xyz=(side_sign * 0.095, 0.010, 0.173)),
            material=metal_dark,
            name=f"{side_name}_sleeve_front_bridge",
        )
        headband.visual(
            Box((0.024, 0.010, 0.024)),
            origin=Origin(xyz=(side_sign * 0.095, -0.010, 0.173)),
            material=metal_dark,
            name=f"{side_name}_sleeve_back_bridge",
        )
        headband.visual(
            Box((0.024, 0.028, 0.024)),
            origin=Origin(xyz=(side_sign * 0.095, 0.0, 0.173)),
            material=metal_dark,
            name=f"{side_name}_sleeve_anchor",
        )
    headband.inertial = Inertial.from_geometry(
        Box((0.240, 0.050, 0.220)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    left_slider = model.part("left_slider")
    _add_slider_geometry(left_slider, metal=metal_dark, accent=trim_dark)
    left_slider.inertial = Inertial.from_geometry(
        Box((0.020, 0.022, 0.100)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    right_slider = model.part("right_slider")
    _add_slider_geometry(right_slider, metal=metal_dark, accent=trim_dark)
    right_slider.inertial = Inertial.from_geometry(
        Box((0.020, 0.022, 0.100)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    left_earcup = model.part("left_earcup")
    _add_earcup_geometry(
        left_earcup,
        sign=-1.0,
        shell_mesh=earcup_shell_mesh,
        pad_mesh=earpad_mesh,
        shell=shell_black,
        trim=trim_dark,
        cushion=cushion_black,
    )
    left_earcup.inertial = Inertial.from_geometry(
        Box((0.056, 0.080, 0.100)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    right_earcup = model.part("right_earcup")
    _add_earcup_geometry(
        right_earcup,
        sign=1.0,
        shell_mesh=earcup_shell_mesh,
        pad_mesh=earpad_mesh,
        shell=shell_black,
        trim=trim_dark,
        cushion=cushion_black,
    )
    right_earcup.inertial = Inertial.from_geometry(
        Box((0.056, 0.080, 0.100)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    model.articulation(
        "left_height_adjust",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_slider,
        origin=Origin(xyz=(-0.102, 0.0, 0.118)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.12,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "right_height_adjust",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_slider,
        origin=Origin(xyz=(0.102, 0.0, 0.118)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.12,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=left_slider,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=right_slider,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
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
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")

    left_slide = object_model.get_articulation("left_height_adjust")
    right_slide = object_model.get_articulation("right_height_adjust")
    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")

    ctx.check(
        "primary parts present",
        all(
            part is not None
            for part in (headband, left_slider, right_slider, left_earcup, right_earcup)
        ),
        details="Expected headband, both sliders, and both earcups.",
    )

    ctx.expect_gap(
        headband,
        left_earcup,
        axis="z",
        min_gap=0.004,
        name="left earcup clears the headband at rest",
    )
    ctx.expect_gap(
        headband,
        right_earcup,
        axis="z",
        min_gap=0.004,
        name="right earcup clears the headband at rest",
    )

    left_rest_pos = ctx.part_world_position(left_earcup)
    right_rest_pos = ctx.part_world_position(right_earcup)
    with ctx.pose(
        {
            left_slide: left_slide.motion_limits.upper,
            right_slide: right_slide.motion_limits.upper,
        }
    ):
        left_extended_pos = ctx.part_world_position(left_earcup)
        right_extended_pos = ctx.part_world_position(right_earcup)

    ctx.check(
        "left slider lowers the earcup",
        left_rest_pos is not None
        and left_extended_pos is not None
        and left_extended_pos[2] < left_rest_pos[2] - 0.020,
        details=f"rest={left_rest_pos}, extended={left_extended_pos}",
    )
    ctx.check(
        "right slider lowers the earcup",
        right_rest_pos is not None
        and right_extended_pos is not None
        and right_extended_pos[2] < right_rest_pos[2] - 0.020,
        details=f"rest={right_rest_pos}, extended={right_extended_pos}",
    )

    left_shell_rest = _aabb_center(
        ctx.part_element_world_aabb(left_earcup, elem="cup_shell")
    )
    right_shell_rest = _aabb_center(
        ctx.part_element_world_aabb(right_earcup, elem="cup_shell")
    )
    with ctx.pose({left_fold: left_fold.motion_limits.upper}):
        left_shell_folded = _aabb_center(
            ctx.part_element_world_aabb(left_earcup, elem="cup_shell")
        )
    with ctx.pose({right_fold: right_fold.motion_limits.upper}):
        right_shell_folded = _aabb_center(
            ctx.part_element_world_aabb(right_earcup, elem="cup_shell")
        )

    ctx.check(
        "left earcup folds inward",
        left_shell_rest is not None
        and left_shell_folded is not None
        and left_shell_folded[0] > left_shell_rest[0] + 0.010
        and left_shell_folded[2] > left_shell_rest[2] + 0.008,
        details=f"rest={left_shell_rest}, folded={left_shell_folded}",
    )
    ctx.check(
        "right earcup folds inward",
        right_shell_rest is not None
        and right_shell_folded is not None
        and right_shell_folded[0] < right_shell_rest[0] - 0.010
        and right_shell_folded[2] > right_shell_rest[2] + 0.008,
        details=f"rest={right_shell_rest}, folded={right_shell_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
