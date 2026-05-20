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
)


def _rounded_rect_panel_mesh(
    width: float,
    height: float,
    thickness: float,
    radius: float,
    name: str,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        thickness,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _rounded_rect_ring_mesh(
    outer_width: float,
    outer_height: float,
    depth: float,
    outer_radius: float,
    inner_width: float,
    inner_height: float,
    inner_radius: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, outer_radius, corner_segments=10),
        [rounded_rect_profile(inner_width, inner_height, inner_radius, corner_segments=10)],
        depth,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _axis_close(
    actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6
) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hung_air_purifier")

    housing_white = model.material("housing_white", rgba=(0.93, 0.94, 0.95, 1.0))
    housing_shadow = model.material("housing_shadow", rgba=(0.77, 0.79, 0.81, 1.0))
    bracket_grey = model.material("bracket_grey", rgba=(0.43, 0.45, 0.48, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    filter_frame_mat = model.material("filter_frame", rgba=(0.20, 0.21, 0.22, 1.0))
    filter_media_mat = model.material("filter_media", rgba=(0.82, 0.85, 0.79, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.66, 0.69, 0.72, 1.0))
    led_blue = model.material("led_blue", rgba=(0.42, 0.73, 0.94, 0.75))

    housing_frame_mesh = _rounded_rect_ring_mesh(
        outer_width=0.480,
        outer_height=0.700,
        depth=0.090,
        outer_radius=0.040,
        inner_width=0.436,
        inner_height=0.656,
        inner_radius=0.022,
        name="purifier_housing_frame",
    )
    cover_panel_mesh = _rounded_rect_panel_mesh(
        width=0.462,
        height=0.682,
        thickness=0.014,
        radius=0.036,
        name="purifier_front_cover",
    )
    filter_frame_mesh = _rounded_rect_ring_mesh(
        outer_width=0.402,
        outer_height=0.586,
        depth=0.010,
        outer_radius=0.018,
        inner_width=0.352,
        inner_height=0.536,
        inner_radius=0.012,
        name="purifier_filter_frame",
    )

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.250, 0.006, 0.560)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=bracket_grey,
        name="back_plate",
    )
    wall_bracket.visual(
        Box((0.180, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.017, 0.220)),
        material=bracket_grey,
        name="upper_hook",
    )
    wall_bracket.visual(
        Box((0.180, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.017, -0.220)),
        material=bracket_grey,
        name="lower_hook",
    )
    wall_bracket.visual(
        Box((0.038, 0.016, 0.100)),
        origin=Origin(xyz=(-0.096, 0.014, 0.0)),
        material=bracket_grey,
        name="left_standoff",
    )
    wall_bracket.visual(
        Box((0.038, 0.016, 0.100)),
        origin=Origin(xyz=(0.096, 0.014, 0.0)),
        material=bracket_grey,
        name="right_standoff",
    )
    wall_bracket.visual(
        Box((0.060, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=bracket_grey,
        name="center_latch",
    )

    housing = model.part("housing")
    housing.visual(
        housing_frame_mesh,
        origin=Origin(xyz=(0.0, 0.066, 0.0)),
        material=housing_white,
        name="front_frame",
    )
    housing.visual(
        Box((0.460, 0.004, 0.660)),
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
        material=housing_shadow,
        name="rear_panel",
    )
    housing.visual(
        Box((0.300, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, 0.035, 0.300)),
        material=housing_shadow,
        name="top_plenum",
    )
    housing.visual(
        Box((0.008, 0.086, 0.560)),
        origin=Origin(xyz=(-0.206, 0.066, 0.0)),
        material=hinge_dark,
        name="left_rail",
    )
    housing.visual(
        Box((0.008, 0.086, 0.560)),
        origin=Origin(xyz=(0.206, 0.066, 0.0)),
        material=hinge_dark,
        name="right_rail",
    )
    housing.visual(
        Box((0.014, 0.026, 0.094)),
        origin=Origin(xyz=(-0.231, 0.104, 0.235)),
        material=hinge_dark,
        name="upper_hinge_leaf",
    )
    housing.visual(
        Box((0.014, 0.026, 0.094)),
        origin=Origin(xyz=(-0.231, 0.104, -0.235)),
        material=hinge_dark,
        name="lower_hinge_leaf",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.078),
        origin=Origin(xyz=(-0.243, 0.114, 0.235)),
        material=hinge_dark,
        name="upper_knuckle",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.078),
        origin=Origin(xyz=(-0.243, 0.114, -0.235)),
        material=hinge_dark,
        name="lower_knuckle",
    )

    front_cover = model.part("front_cover")
    front_cover.visual(
        cover_panel_mesh,
        origin=Origin(xyz=(0.252, 0.0, 0.0)),
        material=housing_white,
        name="cover_panel",
    )
    front_cover.visual(
        Box((0.340, 0.004, 0.228)),
        origin=Origin(xyz=(0.252, 0.004, -0.148)),
        material=housing_shadow,
        name="grille_recess",
    )
    for index in range(8):
        front_cover.visual(
            Box((0.308, 0.004, 0.008)),
            origin=Origin(xyz=(0.252, 0.006, -0.070 - 0.026 * index)),
            material=accent_grey,
            name=f"grille_slat_{index}",
        )
    front_cover.visual(
        Box((0.084, 0.003, 0.012)),
        origin=Origin(xyz=(0.252, 0.0065, 0.264)),
        material=led_blue,
        name="status_strip",
    )
    front_cover.visual(
        Box((0.020, 0.020, 0.090)),
        origin=Origin(xyz=(0.032, 0.0, 0.235)),
        material=hinge_dark,
        name="upper_hinge_strap",
    )
    front_cover.visual(
        Box((0.020, 0.020, 0.090)),
        origin=Origin(xyz=(0.032, 0.0, -0.235)),
        material=hinge_dark,
        name="lower_hinge_strap",
    )

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(
        filter_frame_mesh,
        origin=Origin(),
        material=filter_frame_mat,
        name="filter_frame",
    )
    filter_cartridge.visual(
        Box((0.356, 0.034, 0.540)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=filter_media_mat,
        name="media_block",
    )
    for index in range(11):
        filter_cartridge.visual(
            Box((0.010, 0.006, 0.518)),
            origin=Origin(xyz=(-0.130 + 0.026 * index, 0.014, 0.0)),
            material=accent_grey,
            name=f"media_pleat_{index}",
        )
    filter_cartridge.visual(
        Box((0.014, 0.060, 0.560)),
        origin=Origin(xyz=(-0.195, -0.004, 0.0)),
        material=filter_frame_mat,
        name="left_runner",
    )
    filter_cartridge.visual(
        Box((0.014, 0.060, 0.560)),
        origin=Origin(xyz=(0.195, -0.004, 0.0)),
        material=filter_frame_mat,
        name="right_runner",
    )
    filter_cartridge.visual(
        Box((0.100, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.008, 0.282)),
        material=accent_grey,
        name="pull_tab",
    )

    model.articulation(
        "bracket_to_housing",
        ArticulationType.FIXED,
        parent=wall_bracket,
        child=housing,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_cover",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_cover,
        origin=Origin(xyz=(-0.243, 0.119, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "housing_to_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_cartridge,
        origin=Origin(xyz=(0.0, 0.067, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.120,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    front_cover = object_model.get_part("front_cover")
    filter_cartridge = object_model.get_part("filter_cartridge")
    cover_hinge = object_model.get_articulation("housing_to_cover")
    filter_slide = object_model.get_articulation("housing_to_filter")

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

    ctx.expect_contact(
        housing,
        wall_bracket,
        elem_a="rear_panel",
        elem_b="upper_hook",
        name="housing rear panel seats on upper bracket hook",
    )
    ctx.expect_contact(
        housing,
        wall_bracket,
        elem_a="rear_panel",
        elem_b="lower_hook",
        name="housing rear panel seats on lower bracket hook",
    )
    ctx.expect_contact(
        filter_cartridge,
        housing,
        elem_a="left_runner",
        elem_b="left_rail",
        name="left filter runner engages left guide rail",
    )
    ctx.expect_contact(
        filter_cartridge,
        housing,
        elem_a="right_runner",
        elem_b="right_rail",
        name="right filter runner engages right guide rail",
    )
    ctx.expect_within(
        filter_cartridge,
        housing,
        axes="xz",
        margin=0.0,
        inner_elem="filter_frame",
        name="filter stays within housing opening when inserted",
    )

    hinge_limits = cover_hinge.motion_limits
    slide_limits = filter_slide.motion_limits
    ctx.check(
        "cover hinge axis is vertical",
        _axis_close(cover_hinge.axis, (0.0, 0.0, 1.0)),
        f"expected hinge axis (0, 0, 1), got {cover_hinge.axis}",
    )
    ctx.check(
        "filter slide axis is front-back",
        _axis_close(filter_slide.axis, (0.0, 1.0, 0.0)),
        f"expected filter slide axis (0, 1, 0), got {filter_slide.axis}",
    )
    ctx.check(
        "cover hinge limits are realistic",
        hinge_limits is not None
        and hinge_limits.lower == 0.0
        and hinge_limits.upper is not None
        and 1.3 <= hinge_limits.upper <= 1.9,
        f"unexpected cover hinge limits: {hinge_limits}",
    )
    ctx.check(
        "filter slide travel is realistic",
        slide_limits is not None
        and slide_limits.lower == 0.0
        and slide_limits.upper is not None
        and 0.08 <= slide_limits.upper <= 0.16,
        f"unexpected filter slide limits: {slide_limits}",
    )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_overlap(
            front_cover,
            housing,
            axes="xz",
            min_overlap=0.42,
            elem_a="cover_panel",
            elem_b="front_frame",
            name="closed cover spans housing face",
        )
        ctx.expect_gap(
            front_cover,
            housing,
            axis="y",
            min_gap=0.0005,
            max_gap=0.015,
            positive_elem="cover_panel",
            negative_elem="front_frame",
            name="closed cover sits just proud of housing frame",
        )

    with ctx.pose({cover_hinge: 1.57, filter_slide: 0.11}):
        ctx.expect_gap(
            filter_cartridge,
            housing,
            axis="y",
            min_gap=0.030,
            positive_elem="filter_frame",
            negative_elem="front_frame",
            name="extended filter projects forward of housing",
        )
        ctx.expect_gap(
            filter_cartridge,
            front_cover,
            axis="x",
            min_gap=0.020,
            positive_elem="filter_frame",
            negative_elem="cover_panel",
            name="open cover clears the filter extraction path",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
