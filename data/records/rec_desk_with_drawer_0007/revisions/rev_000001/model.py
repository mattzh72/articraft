from __future__ import annotations

import os
from pathlib import Path

__file__ = "/tmp/model.py"

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        fallback = "/tmp"
        try:
            os.chdir(fallback)
        except OSError:
            fallback = "/"
            os.chdir(fallback)
        return fallback


os.getcwd = _safe_getcwd
_safe_getcwd()

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_l_desk", assets=ASSETS)

    walnut = model.material("walnut", rgba=(0.46, 0.31, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.20, 0.22, 1.0))
    graphite = model.material("graphite", rgba=(0.29, 0.31, 0.34, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.67, 1.0))

    desk_height = 0.75
    top_thickness = 0.036
    top_center_z = desk_height - (top_thickness * 0.5)

    main_desk = model.part("main_desk")
    main_desk.visual(
        Box((1.40, 0.75, top_thickness)),
        origin=Origin(xyz=(-0.70, 0.0, top_center_z)),
        material=walnut,
        name="main_top",
    )
    main_desk.visual(
        Box((0.036, 0.68, desk_height - top_thickness)),
        origin=Origin(xyz=(-1.382, 0.0, (desk_height - top_thickness) * 0.5)),
        material=charcoal,
        name="left_leg_panel",
    )
    main_desk.visual(
        Box((1.14, 0.018, 0.54)),
        origin=Origin(xyz=(-0.81, -0.316, 0.444)),
        material=charcoal,
        name="main_modesty_panel",
    )
    main_desk.visual(
        Box((0.036, 0.28, desk_height - top_thickness)),
        origin=Origin(xyz=(-0.048, -0.205, (desk_height - top_thickness) * 0.5)),
        material=charcoal,
        name="corner_support_panel",
    )
    main_desk.inertial = Inertial.from_geometry(
        Box((1.40, 0.75, desk_height)),
        mass=44.0,
        origin=Origin(xyz=(-0.70, 0.0, desk_height * 0.5)),
    )

    return_pedestal = model.part("return_pedestal")
    return_pedestal.visual(
        Box((0.558, 1.06, top_thickness)),
        origin=Origin(xyz=(0.281, 0.53, 0.0)),
        material=walnut,
        name="return_top",
    )
    return_pedestal.visual(
        Box((0.018, 0.72, 0.676)),
        origin=Origin(xyz=(0.079, 0.63, -0.356)),
        material=charcoal,
        name="pedestal_left_panel",
    )
    return_pedestal.visual(
        Box((0.018, 0.72, 0.676)),
        origin=Origin(xyz=(0.481, 0.63, -0.356)),
        material=charcoal,
        name="pedestal_right_panel",
    )
    return_pedestal.visual(
        Box((0.384, 0.018, 0.676)),
        origin=Origin(xyz=(0.280, 0.279, -0.356)),
        material=charcoal,
        name="pedestal_back_panel",
    )
    return_pedestal.visual(
        Box((0.420, 0.72, 0.036)),
        origin=Origin(xyz=(0.280, 0.63, -0.712)),
        material=graphite,
        name="pedestal_plinth",
    )
    return_pedestal.visual(
        Box((0.384, 0.018, 0.060)),
        origin=Origin(xyz=(0.280, 0.981, -0.048)),
        material=charcoal,
        name="pedestal_top_rail",
    )
    return_pedestal.visual(
        Box((0.384, 0.018, 0.180)),
        origin=Origin(xyz=(0.280, 0.981, -0.340)),
        material=charcoal,
        name="pedestal_lower_panel",
    )
    return_pedestal.visual(
        Box((0.050, 0.52, 0.012)),
        origin=Origin(xyz=(0.113, 0.78, -0.184)),
        material=steel,
        name="pedestal_left_runner",
    )
    return_pedestal.visual(
        Box((0.050, 0.52, 0.012)),
        origin=Origin(xyz=(0.447, 0.78, -0.184)),
        material=steel,
        name="pedestal_right_runner",
    )
    return_pedestal.visual(
        Box((0.050, 0.22, 0.016)),
        origin=Origin(xyz=(-0.014, 0.50, -0.026)),
        material=steel,
        name="seam_cleat",
    )
    return_pedestal.inertial = Inertial.from_geometry(
        Box((0.56, 1.06, desk_height)),
        mass=34.0,
        origin=Origin(xyz=(0.28, 0.53, -0.375)),
    )

    keyboard_drawer = model.part("keyboard_drawer")
    keyboard_drawer.visual(
        Box((0.34, 0.28, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="drawer_tray",
    )
    keyboard_drawer.visual(
        Box((0.376, 0.018, 0.110)),
        origin=Origin(xyz=(0.0, 0.149, 0.0)),
        material=charcoal,
        name="drawer_front",
    )
    keyboard_drawer.visual(
        Box((0.024, 0.30, 0.012)),
        origin=Origin(xyz=(-0.158, -0.030, -0.017)),
        material=steel,
        name="drawer_left_skid",
    )
    keyboard_drawer.visual(
        Box((0.024, 0.30, 0.012)),
        origin=Origin(xyz=(0.158, -0.030, -0.017)),
        material=steel,
        name="drawer_right_skid",
    )
    keyboard_drawer.visual(
        Box((0.30, 0.26, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, 0.016)),
        material=walnut,
        name="keyboard_surface",
    )
    keyboard_drawer.inertial = Inertial.from_geometry(
        Box((0.376, 0.36, 0.110)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "main_to_return",
        ArticulationType.FIXED,
        parent=main_desk,
        child=return_pedestal,
        origin=Origin(xyz=(0.0, -0.35, top_center_z)),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=return_pedestal,
        child=keyboard_drawer,
        origin=Origin(xyz=(0.280, 0.902, -0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.40,
            lower=0.0,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    main_desk = object_model.get_part("main_desk")
    return_pedestal = object_model.get_part("return_pedestal")
    keyboard_drawer = object_model.get_part("keyboard_drawer")
    drawer_slide = object_model.get_articulation("drawer_slide")

    main_top = main_desk.get_visual("main_top")
    return_top = return_pedestal.get_visual("return_top")
    seam_cleat = return_pedestal.get_visual("seam_cleat")
    pedestal_top_rail = return_pedestal.get_visual("pedestal_top_rail")
    pedestal_lower_panel = return_pedestal.get_visual("pedestal_lower_panel")
    pedestal_left_runner = return_pedestal.get_visual("pedestal_left_runner")
    pedestal_right_runner = return_pedestal.get_visual("pedestal_right_runner")
    drawer_front = keyboard_drawer.get_visual("drawer_front")
    drawer_left_skid = keyboard_drawer.get_visual("drawer_left_skid")
    drawer_right_skid = keyboard_drawer.get_visual("drawer_right_skid")
    keyboard_surface = keyboard_drawer.get_visual("keyboard_surface")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.check(
        "drawer_slide_axis_is_forward",
        tuple(drawer_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected drawer slide axis (0, 1, 0), got {drawer_slide.axis}",
    )

    limits = drawer_slide.motion_limits
    ctx.check(
        "drawer_slide_has_realistic_travel",
        limits is not None and limits.lower == 0.0 and limits.upper == 0.26,
        details=f"expected 0.26 m of travel, got {limits}",
    )

    ctx.expect_overlap(
        return_pedestal,
        main_desk,
        axes="y",
        min_overlap=0.72,
        elem_a=return_top,
        elem_b=main_top,
    )
    ctx.expect_gap(
        return_pedestal,
        main_desk,
        axis="x",
        min_gap=0.001,
        max_gap=0.003,
        positive_elem=return_top,
        negative_elem=main_top,
    )
    ctx.expect_contact(
        return_pedestal,
        main_desk,
        elem_a=seam_cleat,
        elem_b=main_top,
    )

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_within(keyboard_drawer, return_pedestal, axes="x")
        ctx.expect_within(keyboard_drawer, return_pedestal, axes="z")
        ctx.expect_contact(
            keyboard_drawer,
            return_pedestal,
            elem_a=drawer_left_skid,
            elem_b=pedestal_left_runner,
        )
        ctx.expect_contact(
            keyboard_drawer,
            return_pedestal,
            elem_a=drawer_right_skid,
            elem_b=pedestal_right_runner,
        )
        ctx.expect_gap(
            return_pedestal,
            keyboard_drawer,
            axis="z",
            min_gap=0.10,
            max_gap=0.13,
            positive_elem=return_top,
            negative_elem=keyboard_surface,
        )
        ctx.expect_gap(
            keyboard_drawer,
            return_pedestal,
            axis="y",
            min_gap=0.04,
            max_gap=0.08,
            positive_elem=drawer_front,
            negative_elem=pedestal_top_rail,
        )
        ctx.expect_overlap(
            keyboard_drawer,
            return_pedestal,
            axes="x",
            min_overlap=0.30,
            elem_a=drawer_front,
            elem_b=pedestal_top_rail,
        )
        ctx.expect_gap(
            keyboard_drawer,
            return_pedestal,
            axis="z",
            min_gap=0.039,
            positive_elem=drawer_front,
            negative_elem=pedestal_lower_panel,
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({drawer_slide: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="drawer_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="drawer_slide_lower_no_floating")

        with ctx.pose({drawer_slide: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="drawer_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="drawer_slide_upper_no_floating")
            ctx.expect_within(keyboard_drawer, return_pedestal, axes="x")
            ctx.expect_within(keyboard_drawer, return_pedestal, axes="z")
            ctx.expect_contact(
                keyboard_drawer,
                return_pedestal,
                elem_a=drawer_left_skid,
                elem_b=pedestal_left_runner,
            )
            ctx.expect_contact(
                keyboard_drawer,
                return_pedestal,
                elem_a=drawer_right_skid,
                elem_b=pedestal_right_runner,
            )
            ctx.expect_gap(
                keyboard_drawer,
                return_pedestal,
                axis="y",
                min_gap=0.30,
                max_gap=0.33,
                positive_elem=drawer_front,
                negative_elem=pedestal_top_rail,
            )
            ctx.expect_gap(
                return_pedestal,
                keyboard_drawer,
                axis="z",
                min_gap=0.10,
                max_gap=0.13,
                positive_elem=return_top,
                negative_elem=keyboard_surface,
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
