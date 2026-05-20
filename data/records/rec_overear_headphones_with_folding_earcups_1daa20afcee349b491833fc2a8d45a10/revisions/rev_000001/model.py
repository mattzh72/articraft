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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_band_section(
    x_pos: float,
    width_y: float,
    height_z: float,
    corner_radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width_y,
        height_z,
        corner_radius,
        corner_segments=8,
    )
    return [(x_pos, y, z_center + z) for y, z in profile]


def _axis_close(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
    return all(abs(a - b) <= 1e-6 for a, b in zip(axis, target))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def _add_arm_sleeve(
    headband,
    *,
    side_name: str,
    side_sign: float,
    x_pos: float,
    material,
) -> None:
    headband.visual(
        Box((0.014, 0.018, 0.006)),
        origin=Origin(xyz=(x_pos + side_sign * 0.004, 0.0, 0.149)),
        material=material,
        name=f"{side_name}_sleeve_top",
    )
    headband.visual(
        Box((0.004, 0.018, 0.052)),
        origin=Origin(xyz=(x_pos + side_sign * 0.008, 0.0, 0.126)),
        material=material,
        name=f"{side_name}_sleeve_outer",
    )
    headband.visual(
        Box((0.010, 0.003, 0.052)),
        origin=Origin(xyz=(x_pos + side_sign * 0.002, 0.0075, 0.126)),
        material=material,
        name=f"{side_name}_sleeve_front",
    )
    headband.visual(
        Box((0.010, 0.003, 0.052)),
        origin=Origin(xyz=(x_pos + side_sign * 0.002, -0.0075, 0.126)),
        material=material,
        name=f"{side_name}_sleeve_back",
    )


def _add_slider_geometry(slider, *, side_name: str, side_sign: float, material) -> None:
    slider.visual(
        Box((0.008, 0.012, 0.042)),
        origin=Origin(xyz=(side_sign * 0.002, 0.0, -0.021)),
        material=material,
        name=f"{side_name}_stem",
    )
    slider.visual(
        Box((0.010, 0.012, 0.004)),
        origin=Origin(xyz=(side_sign * 0.002, 0.0, -0.004)),
        material=material,
        name=f"{side_name}_stop",
    )
    slider.visual(
        Box((0.012, 0.014, 0.010)),
        origin=Origin(xyz=(side_sign * 0.008, 0.0, -0.036)),
        material=material,
        name=f"{side_name}_shoulder",
    )
    slider.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(side_sign * 0.009, 0.0, -0.045)),
        material=material,
        name=f"{side_name}_mount",
    )


def _add_yoke_geometry(yoke, *, side_name: str, side_sign: float, material) -> None:
    yoke.visual(
        Box((0.010, 0.012, 0.006)),
        origin=Origin(xyz=(side_sign * 0.009, 0.0, -0.003)),
        material=material,
        name=f"{side_name}_top_plate",
    )
    yoke.visual(
        Box((0.006, 0.012, 0.046)),
        origin=Origin(xyz=(side_sign * 0.015, 0.0, -0.023)),
        material=material,
        name=f"{side_name}_spine",
    )
    yoke.visual(
        Box((0.008, 0.012, 0.006)),
        origin=Origin(xyz=(side_sign * 0.021, 0.0, -0.010)),
        material=material,
        name=f"{side_name}_upper_arm",
    )
    yoke.visual(
        Box((0.008, 0.012, 0.006)),
        origin=Origin(xyz=(side_sign * 0.021, 0.0, -0.036)),
        material=material,
        name=f"{side_name}_lower_arm",
    )


def _add_cup_geometry(
    cup,
    *,
    side_name: str,
    side_sign: float,
    shell_material,
    pad_mesh,
    pad_material,
    accent_material,
) -> None:
    cup.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(
            xyz=(side_sign * 0.034, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=shell_material,
        name=f"{side_name}_shell",
    )
    cup.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(
            xyz=(side_sign * 0.038, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=accent_material,
        name=f"{side_name}_outer_badge",
    )
    cup.visual(
        pad_mesh,
        origin=Origin(
            xyz=(side_sign * 0.024, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pad_material,
        name=f"{side_name}_pad",
    )
    cup.visual(
        Cylinder(radius=0.022, length=0.002),
        origin=Origin(
            xyz=(side_sign * 0.027, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=accent_material,
        name=f"{side_name}_driver_grille",
    )
    cup.visual(
        Cylinder(radius=0.003, length=0.008),
        origin=Origin(
            xyz=(side_sign * 0.027, 0.034, -0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=accent_material,
        name=f"{side_name}_cable_jack",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_over_ear_headphones")

    shell_black = model.material("shell_black", rgba=(0.13, 0.13, 0.14, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.07, 0.08, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    pad_black = model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))

    headband_outer_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_band_section(-0.080, 0.022, 0.010, 0.004, 0.157),
                _yz_band_section(-0.045, 0.028, 0.010, 0.004, 0.184),
                _yz_band_section(0.0, 0.034, 0.011, 0.0045, 0.199),
                _yz_band_section(0.045, 0.028, 0.010, 0.004, 0.184),
                _yz_band_section(0.080, 0.022, 0.010, 0.004, 0.157),
            ]
        ),
        "headband_outer_band",
    )
    headband_pad_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_band_section(-0.052, 0.020, 0.006, 0.0025, 0.175),
                _yz_band_section(0.0, 0.024, 0.006, 0.003, 0.186),
                _yz_band_section(0.052, 0.020, 0.006, 0.0025, 0.175),
            ]
        ),
        "headband_inner_pad",
    )
    ear_pad_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.033, tube=0.0085, radial_segments=18, tubular_segments=42),
        "ear_pad_ring",
    )

    headband = model.part("headband")
    headband.visual(headband_outer_mesh, material=graphite, name="outer_band")
    headband.visual(headband_pad_mesh, material=pad_black, name="inner_pad")
    _add_arm_sleeve(headband, side_name="left", side_sign=-1.0, x_pos=-0.078, material=graphite)
    _add_arm_sleeve(headband, side_name="right", side_sign=1.0, x_pos=0.078, material=graphite)
    headband.inertial = Inertial.from_geometry(
        Box((0.190, 0.040, 0.060)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    left_slider = model.part("left_slider")
    _add_slider_geometry(left_slider, side_name="left", side_sign=-1.0, material=metal_gray)
    left_slider.inertial = Inertial.from_geometry(
        Box((0.016, 0.016, 0.064)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
    )

    right_slider = model.part("right_slider")
    _add_slider_geometry(right_slider, side_name="right", side_sign=1.0, material=metal_gray)
    right_slider.inertial = Inertial.from_geometry(
        Box((0.016, 0.016, 0.064)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
    )

    left_yoke = model.part("left_yoke")
    _add_yoke_geometry(left_yoke, side_name="left", side_sign=-1.0, material=shell_black)
    left_yoke.inertial = Inertial.from_geometry(
        Box((0.012, 0.016, 0.048)),
        mass=0.018,
        origin=Origin(xyz=(-0.004, 0.0, -0.024)),
    )

    right_yoke = model.part("right_yoke")
    _add_yoke_geometry(right_yoke, side_name="right", side_sign=1.0, material=shell_black)
    right_yoke.inertial = Inertial.from_geometry(
        Box((0.012, 0.016, 0.048)),
        mass=0.018,
        origin=Origin(xyz=(0.004, 0.0, -0.024)),
    )

    left_cup = model.part("left_cup")
    _add_cup_geometry(
        left_cup,
        side_name="left",
        side_sign=-1.0,
        shell_material=shell_black,
        pad_mesh=ear_pad_mesh,
        pad_material=pad_black,
        accent_material=trim_black,
    )
    left_cup.inertial = Inertial.from_geometry(
        Box((0.030, 0.096, 0.096)),
        mass=0.11,
        origin=Origin(xyz=(-0.021, 0.0, 0.0)),
    )

    right_cup = model.part("right_cup")
    _add_cup_geometry(
        right_cup,
        side_name="right",
        side_sign=1.0,
        shell_material=shell_black,
        pad_mesh=ear_pad_mesh,
        pad_material=pad_black,
        accent_material=trim_black,
    )
    right_cup.inertial = Inertial.from_geometry(
        Box((0.030, 0.096, 0.096)),
        mass=0.11,
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
    )

    model.articulation(
        "headband_to_left_slider",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_slider,
        origin=Origin(xyz=(-0.078, 0.0, 0.146)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.06,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "headband_to_right_slider",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_slider,
        origin=Origin(xyz=(0.078, 0.0, 0.146)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.06,
            lower=0.0,
            upper=0.028,
        ),
    )

    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=left_slider,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=right_slider,
        child=right_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    model.articulation(
        "left_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-math.radians(25.0),
            upper=math.radians(25.0),
        ),
    )
    model.articulation(
        "right_cup_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-math.radians(25.0),
            upper=math.radians(25.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_slider = object_model.get_part("left_slider")
    right_slider = object_model.get_part("right_slider")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_slide = object_model.get_articulation("headband_to_left_slider")
    right_slide = object_model.get_articulation("headband_to_right_slider")
    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_swivel = object_model.get_articulation("left_cup_swivel")
    right_swivel = object_model.get_articulation("right_cup_swivel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_slider,
        headband,
        elem_a="left_stem",
        elem_b="left_sleeve_front",
        contact_tol=1e-6,
    )
    ctx.expect_contact(
        left_slider,
        headband,
        elem_a="left_stem",
        elem_b="left_sleeve_back",
        contact_tol=1e-6,
    )
    ctx.expect_contact(
        right_slider,
        headband,
        elem_a="right_stem",
        elem_b="right_sleeve_front",
        contact_tol=1e-6,
    )
    ctx.expect_contact(
        right_slider,
        headband,
        elem_a="right_stem",
        elem_b="right_sleeve_back",
        contact_tol=1e-6,
    )
    ctx.expect_gap(
        headband,
        left_slider,
        axis="z",
        positive_elem="left_sleeve_top",
        negative_elem="left_stop",
        min_gap=0.0015,
        max_gap=0.0035,
    )
    ctx.expect_gap(
        headband,
        right_slider,
        axis="z",
        positive_elem="right_sleeve_top",
        negative_elem="right_stop",
        min_gap=0.0015,
        max_gap=0.0035,
    )

    ctx.expect_contact(left_slider, left_yoke, elem_a="left_mount", elem_b="left_top_plate")
    ctx.expect_contact(right_slider, right_yoke, elem_a="right_mount", elem_b="right_top_plate")
    ctx.expect_contact(left_yoke, left_cup, elem_a="left_upper_arm", elem_b="left_shell")
    ctx.expect_contact(right_yoke, right_cup, elem_a="right_upper_arm", elem_b="right_shell")

    ctx.check(
        "left_slide_axis_is_downward",
        _axis_close(left_slide.axis, (0.0, 0.0, -1.0)),
        f"axis was {left_slide.axis}",
    )
    ctx.check(
        "right_slide_axis_is_downward",
        _axis_close(right_slide.axis, (0.0, 0.0, -1.0)),
        f"axis was {right_slide.axis}",
    )
    ctx.check(
        "left_fold_axis_folds_inward",
        _axis_close(left_fold.axis, (0.0, -1.0, 0.0)),
        f"axis was {left_fold.axis}",
    )
    ctx.check(
        "right_fold_axis_folds_inward",
        _axis_close(right_fold.axis, (0.0, 1.0, 0.0)),
        f"axis was {right_fold.axis}",
    )
    ctx.check(
        "left_cup_swivel_axis_is_vertical",
        _axis_close(left_swivel.axis, (0.0, 0.0, 1.0)),
        f"axis was {left_swivel.axis}",
    )
    ctx.check(
        "right_cup_swivel_axis_is_vertical",
        _axis_close(right_swivel.axis, (0.0, 0.0, -1.0)),
        f"axis was {right_swivel.axis}",
    )

    left_yoke_rest = ctx.part_world_position(left_yoke)
    right_yoke_rest = ctx.part_world_position(right_yoke)
    assert left_yoke_rest is not None
    assert right_yoke_rest is not None
    with ctx.pose({left_slide: 0.028, right_slide: 0.028}):
        left_yoke_extended = ctx.part_world_position(left_yoke)
        right_yoke_extended = ctx.part_world_position(right_yoke)
        assert left_yoke_extended is not None
        assert right_yoke_extended is not None
        ctx.check(
            "left_slider_extension_moves_yoke_down",
            left_yoke_extended[2] < left_yoke_rest[2] - 0.020,
            f"rest z={left_yoke_rest[2]:.4f}, extended z={left_yoke_extended[2]:.4f}",
        )
        ctx.check(
            "right_slider_extension_moves_yoke_down",
            right_yoke_extended[2] < right_yoke_rest[2] - 0.020,
            f"rest z={right_yoke_rest[2]:.4f}, extended z={right_yoke_extended[2]:.4f}",
        )
        ctx.expect_overlap(
            left_slider,
            headband,
            axes="z",
            min_overlap=0.007,
            elem_a="left_stem",
            elem_b="left_sleeve_front",
        )
        ctx.expect_overlap(
            right_slider,
            headband,
            axes="z",
            min_overlap=0.007,
            elem_a="right_stem",
            elem_b="right_sleeve_front",
        )

    left_cup_rest = ctx.part_world_position(left_cup)
    right_cup_rest = ctx.part_world_position(right_cup)
    assert left_cup_rest is not None
    assert right_cup_rest is not None
    with ctx.pose({left_fold: math.radians(90.0), right_fold: math.radians(90.0)}):
        left_cup_folded = ctx.part_world_position(left_cup)
        right_cup_folded = ctx.part_world_position(right_cup)
        assert left_cup_folded is not None
        assert right_cup_folded is not None
        ctx.check(
            "left_fold_hinge_raises_and_tucks_cup",
            left_cup_folded[2] > left_cup_rest[2] + 0.015
            and left_cup_folded[0] > left_cup_rest[0] + 0.015,
            f"rest={left_cup_rest}, folded={left_cup_folded}",
        )
        ctx.check(
            "right_fold_hinge_raises_and_tucks_cup",
            right_cup_folded[2] > right_cup_rest[2] + 0.015
            and right_cup_folded[0] < right_cup_rest[0] - 0.015,
            f"rest={right_cup_rest}, folded={right_cup_folded}",
        )

    left_cup_rest_aabb = ctx.part_world_aabb(left_cup)
    left_jack_rest_aabb = ctx.part_element_world_aabb(left_cup, elem="left_cable_jack")
    assert left_cup_rest_aabb is not None
    assert left_jack_rest_aabb is not None
    left_jack_rest_center = _aabb_center(left_jack_rest_aabb)
    with ctx.pose({left_swivel: math.radians(25.0)}):
        left_cup_swiveled_aabb = ctx.part_world_aabb(left_cup)
        left_jack_swiveled_aabb = ctx.part_element_world_aabb(left_cup, elem="left_cable_jack")
        assert left_cup_swiveled_aabb is not None
        assert left_jack_swiveled_aabb is not None
        left_jack_swiveled_center = _aabb_center(left_jack_swiveled_aabb)
        ctx.check(
            "left_cup_swivel_changes_cup_orientation",
            left_jack_swiveled_center[0] < left_jack_rest_center[0] - 0.008
            and left_jack_swiveled_center[1] < left_jack_rest_center[1] - 0.008,
            f"rest jack center={left_jack_rest_center}, swiveled jack center={left_jack_swiveled_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
