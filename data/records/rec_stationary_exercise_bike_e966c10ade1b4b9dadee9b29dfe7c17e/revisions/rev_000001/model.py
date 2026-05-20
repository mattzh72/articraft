from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_mesh(*, outer_radius: float, inner_radius: float, length: float, name: str):
    ring = LatheGeometry.from_shell_profiles(
        [(outer_radius, -0.5 * length), (outer_radius, 0.5 * length)],
        [(inner_radius, -0.5 * length), (inner_radius, 0.5 * length)],
        segments=48,
    ).rotate_x(pi / 2.0)
    return _mesh(name, ring)


def _rect_tube_mesh(
    *,
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    name: str,
):
    tube = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_x, outer_y, radius=min(outer_x, outer_y) * 0.16),
        [rounded_rect_profile(inner_x, inner_y, radius=min(inner_x, inner_y) * 0.12)],
        height=height,
        center=True,
    )
    return _mesh(name, tube)


def _housing_panel_mesh(name: str):
    outer = rounded_rect_profile(0.46, 0.50, radius=0.10)
    crank_hole = [(x - 0.02, y - 0.05) for x, y in superellipse_profile(0.076, 0.076, exponent=2.0, segments=28)]
    panel = ExtrudeWithHolesGeometry(
        outer,
        [crank_hole],
        height=0.004,
        center=True,
    ).rotate_x(pi / 2.0)
    return _mesh(name, panel)


def _saddle_section(x_pos: float, width_y: float, height_z: float, radius: float):
    return tuple((x_pos, y, z) for z, y in rounded_rect_profile(height_z, width_y, radius))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_stationary_bike")

    frame_paint = model.material("frame_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    housing_paint = model.material("housing_paint", rgba=(0.10, 0.11, 0.12, 1.0))
    anodized = model.material("anodized", rgba=(0.54, 0.58, 0.62, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    elastomer = model.material("elastomer", rgba=(0.07, 0.07, 0.07, 1.0))
    accent = model.material("accent", rgba=(0.76, 0.14, 0.10, 1.0))
    pale_mark = model.material("pale_mark", rgba=(0.92, 0.92, 0.90, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.18, 0.64, 1.22)),
        mass=54.0,
        origin=Origin(xyz=(0.02, 0.0, 0.28)),
    )

    frame.visual(
        Box((0.12, 0.58, 0.04)),
        origin=Origin(xyz=(-0.43, 0.0, -0.37)),
        material=frame_paint,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.12, 0.62, 0.04)),
        origin=Origin(xyz=(0.52, 0.0, -0.37)),
        material=frame_paint,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.06, 0.08, 0.012)),
        origin=Origin(xyz=(-0.43, 0.0, -0.346)),
        material=dark_metal,
        name="rear_center_pad",
    )
    frame.visual(
        Box((0.06, 0.08, 0.012)),
        origin=Origin(xyz=(0.52, 0.0, -0.346)),
        material=dark_metal,
        name="front_center_pad",
    )

    frame.visual(
        _housing_panel_mesh("bike_housing_left_panel"),
        origin=Origin(xyz=(0.02, -0.058, 0.05)),
        material=housing_paint,
        name="left_housing_panel",
    )
    frame.visual(
        _housing_panel_mesh("bike_housing_right_panel"),
        origin=Origin(xyz=(0.02, 0.058, 0.05)),
        material=housing_paint,
        name="right_housing_panel",
    )
    frame.visual(
        Box((0.24, 0.116, 0.020)),
        origin=Origin(xyz=(0.02, 0.0, 0.255)),
        material=housing_paint,
        name="housing_top_bridge",
    )
    frame.visual(
        Box((0.10, 0.116, 0.18)),
        origin=Origin(xyz=(0.20, 0.0, 0.04)),
        material=housing_paint,
        name="housing_nose_bridge",
    )
    frame.visual(
        Box((0.14, 0.116, 0.14)),
        origin=Origin(xyz=(-0.16, 0.0, 0.00)),
        material=housing_paint,
        name="housing_tail_bridge",
    )
    frame.visual(
        Box((0.36, 0.116, 0.040)),
        origin=Origin(xyz=(0.00, 0.0, -0.16)),
        material=housing_paint,
        name="housing_lower_bridge",
    )
    frame.visual(
        Box((0.10, 0.050, 0.012)),
        origin=Origin(xyz=(0.06, 0.0, 0.265)),
        material=anodized,
        name="housing_datum_pad",
    )
    frame.visual(
        Box((0.080, 0.004, 0.002)),
        origin=Origin(xyz=(0.06, 0.0, 0.272)),
        material=pale_mark,
        name="housing_datum_mark",
    )

    frame.visual(
        _mesh(
            "bike_rear_support_tube",
            tube_from_spline_points(
                [(-0.43, 0.0, -0.37), (-0.34, 0.0, -0.12), (-0.18, 0.0, 0.18)],
                radius=0.030,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="rear_support_tube",
    )
    frame.visual(
        _mesh(
            "bike_seat_beam",
            tube_from_spline_points(
                [(-0.04, 0.0, 0.16), (-0.10, 0.0, 0.34), (-0.18, 0.0, 0.55)],
                radius=0.034,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="seat_beam",
    )
    frame.visual(
        _mesh(
            "bike_front_beam",
            tube_from_spline_points(
                [(0.04, 0.0, -0.10), (0.18, 0.0, 0.14), (0.34, 0.0, 0.58)],
                radius=0.032,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="front_beam",
    )
    frame.visual(
        _mesh(
            "bike_front_support_tube",
            tube_from_spline_points(
                [(0.52, 0.0, -0.37), (0.48, 0.0, -0.02), (0.34, 0.0, 0.58)],
                radius=0.028,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="front_support_tube",
    )
    frame.visual(
        _mesh(
            "bike_mid_tie_tube",
            tube_from_spline_points(
                [(-0.18, 0.0, 0.14), (0.00, 0.0, 0.10), (0.18, 0.0, 0.16)],
                radius=0.024,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=frame_paint,
        name="mid_tie_tube",
    )

    frame.visual(
        _rect_tube_mesh(
            outer_x=0.070,
            outer_y=0.050,
            inner_x=0.052,
            inner_y=0.032,
            height=0.180,
            name="bike_seat_sleeve",
        ),
        origin=Origin(xyz=(-0.18, 0.0, 0.64)),
        material=anodized,
        name="seat_sleeve",
    )
    frame.visual(
        Box((0.044, 0.066, 0.018)),
        origin=Origin(xyz=(-0.236, 0.0, 0.724)),
        material=dark_metal,
        name="seat_clamp_block",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(-0.258, 0.0, 0.724), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="seat_clamp_knob",
    )
    frame.visual(
        Box((0.004, 0.018, 0.120)),
        origin=Origin(xyz=(-0.145, 0.009, 0.675)),
        material=pale_mark,
        name="seat_scale_strip",
    )
    for index, z_pos in enumerate((0.620, 0.648, 0.676, 0.704)):
        frame.visual(
            Box((0.008 + 0.003 * (index % 2), 0.003, 0.003)),
            origin=Origin(xyz=(-0.143, 0.018, z_pos)),
            material=accent,
            name=f"seat_tick_{index + 1}",
        )

    frame.visual(
        _rect_tube_mesh(
            outer_x=0.082,
            outer_y=0.058,
            inner_x=0.058,
            inner_y=0.036,
            height=0.220,
            name="bike_bar_sleeve",
        ),
        origin=Origin(xyz=(0.34, 0.0, 0.69)),
        material=anodized,
        name="bar_sleeve",
    )
    frame.visual(
        Box((0.048, 0.074, 0.020)),
        origin=Origin(xyz=(0.289, 0.0, 0.792)),
        material=dark_metal,
        name="bar_clamp_block",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.078),
        origin=Origin(xyz=(0.266, 0.0, 0.792), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="bar_clamp_knob",
    )
    frame.visual(
        Box((0.004, 0.018, 0.150)),
        origin=Origin(xyz=(0.304, 0.0, 0.735)),
        material=pale_mark,
        name="bar_scale_strip",
    )
    for index, z_pos in enumerate((0.640, 0.674, 0.708, 0.742, 0.776)):
        frame.visual(
            Box((0.010 + 0.003 * (index % 2), 0.003, 0.003)),
            origin=Origin(xyz=(0.304, -0.020, z_pos)),
            material=accent,
            name=f"bar_tick_{index + 1}",
        )

    frame.visual(
        _ring_mesh(outer_radius=0.048, inner_radius=0.029, length=0.014, name="bike_left_bearing_ring"),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=dark_metal,
        name="left_bearing",
    )
    frame.visual(
        _ring_mesh(outer_radius=0.048, inner_radius=0.029, length=0.014, name="bike_right_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=dark_metal,
        name="right_bearing",
    )
    frame.visual(
        Box((0.050, 0.130, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=dark_metal,
        name="bottom_bracket_bridge",
    )

    crank = model.part("crank_assembly")
    crank.inertial = Inertial.from_geometry(
        Box((0.50, 0.42, 0.48)),
        mass=14.0,
        origin=Origin(xyz=(-0.01, -0.02, 0.0)),
    )
    crank.visual(
        Cylinder(radius=0.024, length=0.086),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="spindle_core",
    )
    crank.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_shoulder",
    )
    crank.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_shoulder",
    )
    crank.visual(
        Cylinder(radius=0.023, length=0.138),
        origin=Origin(xyz=(0.0, -0.109, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="left_stub",
    )
    crank.visual(
        Cylinder(radius=0.023, length=0.138),
        origin=Origin(xyz=(0.0, 0.109, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="right_stub",
    )
    crank.visual(
        Cylinder(radius=0.220, length=0.016),
        origin=Origin(xyz=(0.0, -0.102, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="flywheel_disk",
    )
    crank.visual(
        Cylinder(radius=0.182, length=0.010),
        origin=Origin(xyz=(0.0, -0.102, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="flywheel_rim",
    )
    crank.visual(
        Cylinder(radius=0.104, length=0.012),
        origin=Origin(xyz=(0.0, 0.093, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="drive_disc",
    )
    crank.visual(
        Box((0.170, 0.020, 0.028)),
        origin=Origin(xyz=(0.085, 0.155, 0.0)),
        material=dark_metal,
        name="right_arm",
    )
    crank.visual(
        Box((0.170, 0.020, 0.028)),
        origin=Origin(xyz=(-0.085, -0.155, 0.0)),
        material=dark_metal,
        name="left_arm",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.170, 0.188, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="right_pedal_spindle",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(-0.170, -0.188, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="left_pedal_spindle",
    )
    crank.visual(
        Box((0.050, 0.014, 0.022)),
        origin=Origin(xyz=(0.170, 0.215, 0.0)),
        material=elastomer,
        name="right_pedal",
    )
    crank.visual(
        Box((0.050, 0.014, 0.022)),
        origin=Origin(xyz=(-0.170, -0.215, 0.0)),
        material=elastomer,
        name="left_pedal",
    )
    crank.visual(
        Box((0.012, 0.016, 0.040)),
        origin=Origin(xyz=(0.000, 0.094, 0.058)),
        material=accent,
        name="phase_index",
    )

    seat_post = model.part("saddle_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.08, 0.05, 0.42)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )
    seat_post.visual(
        Box((0.045, 0.025, 0.340)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=anodized,
        name="seat_post_shaft",
    )
    seat_post.visual(
        Box((0.064, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="seat_collar",
    )
    seat_post.visual(
        Box((0.030, 0.054, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=dark_metal,
        name="seat_head",
    )
    seat_post.visual(
        Box((0.022, 0.004, 0.170)),
        origin=Origin(xyz=(0.025, 0.0, 0.060)),
        material=pale_mark,
        name="seat_index_strip",
    )
    for index, z_pos in enumerate((-0.045, -0.010, 0.025, 0.060)):
        seat_post.visual(
            Box((0.004, 0.012 + 0.004 * (index % 2), 0.003)),
            origin=Origin(xyz=(0.025, 0.0, z_pos + 0.025)),
            material=accent,
            name=f"seat_index_tick_{index + 1}",
        )

    saddle = model.part("saddle")
    saddle.inertial = Inertial.from_geometry(
        Box((0.32, 0.18, 0.12)),
        mass=3.5,
        origin=Origin(xyz=(-0.02, 0.0, 0.030)),
    )
    saddle.visual(
        _mesh(
            "bike_saddle_shell",
            section_loft(
                [
                    _saddle_section(-0.16, 0.110, 0.028, 0.010),
                    _saddle_section(-0.07, 0.162, 0.050, 0.018),
                    _saddle_section(0.02, 0.110, 0.040, 0.016),
                    _saddle_section(0.12, 0.044, 0.020, 0.008),
                ]
            ),
        ),
        origin=Origin(xyz=(-0.02, 0.0, 0.040)),
        material=elastomer,
        name="saddle_shell",
    )
    saddle.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(xyz=(-0.015, -0.030, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=anodized,
        name="left_rail",
    )
    saddle.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(xyz=(-0.015, 0.030, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=anodized,
        name="right_rail",
    )
    saddle.visual(
        Box((0.050, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="rail_bridge",
    )

    handlebar_post = model.part("handlebar_post")
    handlebar_post.inertial = Inertial.from_geometry(
        Box((0.09, 0.06, 0.50)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )
    handlebar_post.visual(
        Box((0.050, 0.030, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=anodized,
        name="bar_post_shaft",
    )
    handlebar_post.visual(
        Box((0.074, 0.046, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="bar_collar",
    )
    handlebar_post.visual(
        Box((0.060, 0.042, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.298)),
        material=dark_metal,
        name="bar_head",
    )
    handlebar_post.visual(
        Box((0.026, 0.004, 0.220)),
        origin=Origin(xyz=(-0.028, 0.0, 0.095)),
        material=pale_mark,
        name="bar_index_strip",
    )
    for index, z_pos in enumerate((-0.070, -0.025, 0.020, 0.065, 0.110)):
        handlebar_post.visual(
            Box((0.004, 0.016 + 0.004 * (index % 2), 0.003)),
            origin=Origin(xyz=(-0.028, 0.0, z_pos + 0.095)),
            material=accent,
            name=f"bar_index_tick_{index + 1}",
        )

    handlebars = model.part("handlebars")
    handlebars.inertial = Inertial.from_geometry(
        Box((0.26, 0.56, 0.24)),
        mass=4.5,
        origin=Origin(xyz=(0.06, 0.0, 0.10)),
    )
    handlebars.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_metal,
        name="stem_base",
    )
    handlebars.visual(
        Box((0.070, 0.060, 0.050)),
        origin=Origin(xyz=(0.040, 0.0, 0.090)),
        material=dark_metal,
        name="stem_block",
    )
    handlebars.visual(
        Cylinder(radius=0.017, length=0.500),
        origin=Origin(xyz=(0.060, 0.0, 0.130), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="crossbar",
    )
    handlebars.visual(
        Box((0.130, 0.060, 0.012)),
        origin=Origin(xyz=(0.060, 0.0, 0.153)),
        material=anodized,
        name="bar_datum_pad",
    )
    handlebars.visual(
        Box((0.004, 0.050, 0.002)),
        origin=Origin(xyz=(0.060, 0.0, 0.160)),
        material=pale_mark,
        name="bar_center_mark",
    )
    handlebars.visual(
        _mesh(
            "bike_right_horn",
            tube_from_spline_points(
                [(0.060, 0.235, 0.130), (0.150, 0.245, 0.126), (0.230, 0.236, 0.102), (0.285, 0.220, 0.055)],
                radius=0.014,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=dark_metal,
        name="right_horn",
    )
    handlebars.visual(
        _mesh(
            "bike_left_horn",
            tube_from_spline_points(
                [(0.060, -0.235, 0.130), (0.150, -0.245, 0.126), (0.230, -0.236, 0.102), (0.285, -0.220, 0.055)],
                radius=0.014,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=dark_metal,
        name="left_horn",
    )
    handlebars.visual(
        Cylinder(radius=0.017, length=0.090),
        origin=Origin(xyz=(0.323, 0.220, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=elastomer,
        name="right_grip",
    )
    handlebars.visual(
        Cylinder(radius=0.017, length=0.090),
        origin=Origin(xyz=(0.323, -0.220, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=elastomer,
        name="left_grip",
    )

    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_saddle_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.18, 0.0, 0.730)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.15, lower=0.0, upper=0.12),
    )
    model.articulation(
        "saddle_post_to_saddle",
        ArticulationType.FIXED,
        parent=seat_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
    )
    model.articulation(
        "frame_to_handlebar_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handlebar_post,
        origin=Origin(xyz=(0.34, 0.0, 0.800)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.15, lower=0.0, upper=0.14),
    )
    model.articulation(
        "handlebar_post_to_handlebars",
        ArticulationType.FIXED,
        parent=handlebar_post,
        child=handlebars,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crank = object_model.get_part("crank_assembly")
    seat_post = object_model.get_part("saddle_post")
    saddle = object_model.get_part("saddle")
    handlebar_post = object_model.get_part("handlebar_post")
    handlebars = object_model.get_part("handlebars")
    crank_joint = object_model.get_articulation("frame_to_crank")
    seat_joint = object_model.get_articulation("frame_to_saddle_post")
    bar_joint = object_model.get_articulation("frame_to_handlebar_post")

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

    ctx.expect_contact(crank, frame, elem_a="left_shoulder", elem_b="left_bearing")
    ctx.expect_contact(crank, frame, elem_a="right_shoulder", elem_b="right_bearing")
    ctx.expect_contact(seat_post, frame, elem_a="seat_collar", elem_b="seat_sleeve")
    ctx.expect_contact(handlebar_post, frame, elem_a="bar_collar", elem_b="bar_sleeve")
    ctx.expect_contact(saddle, seat_post, elem_a="rail_bridge", elem_b="seat_head")
    ctx.expect_contact(handlebars, handlebar_post, elem_a="stem_base", elem_b="bar_head")

    with ctx.pose({seat_joint: 0.12}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="seat_post_shaft",
            outer_elem="seat_sleeve",
            margin=0.0,
        )

    with ctx.pose({bar_joint: 0.14}):
        ctx.expect_within(
            handlebar_post,
            frame,
            axes="xy",
            inner_elem="bar_post_shaft",
            outer_elem="bar_sleeve",
            margin=0.0,
        )

    def _elem_center_z(part_name: str, elem_name: str) -> float:
        min_corner, max_corner = ctx.part_element_world_aabb(part_name, elem=elem_name)
        return 0.5 * (min_corner[2] + max_corner[2])

    pedal_z_rest = _elem_center_z("crank_assembly", "right_pedal")
    with ctx.pose({crank_joint: pi / 2.0}):
        pedal_z_raised = _elem_center_z("crank_assembly", "right_pedal")
    ctx.check(
        "crank_positive_rotation_lifts_right_pedal",
        pedal_z_raised > pedal_z_rest + 0.12,
        details=f"rest_z={pedal_z_rest:.3f}, raised_z={pedal_z_raised:.3f}",
    )

    saddle_z_rest = ctx.part_world_position(saddle)[2]
    with ctx.pose({seat_joint: 0.12}):
        saddle_z_high = ctx.part_world_position(saddle)[2]
    ctx.check(
        "saddle_adjusts_upward",
        saddle_z_high > saddle_z_rest + 0.10,
        details=f"rest_z={saddle_z_rest:.3f}, high_z={saddle_z_high:.3f}",
    )

    handlebar_z_rest = ctx.part_world_position(handlebars)[2]
    with ctx.pose({bar_joint: 0.14}):
        handlebar_z_high = ctx.part_world_position(handlebars)[2]
    ctx.check(
        "handlebars_adjust_upward",
        handlebar_z_high > handlebar_z_rest + 0.12,
        details=f"rest_z={handlebar_z_rest:.3f}, high_z={handlebar_z_high:.3f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
