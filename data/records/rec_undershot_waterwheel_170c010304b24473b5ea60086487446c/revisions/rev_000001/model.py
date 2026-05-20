from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    sample_catmull_rom_spline_2d,
    superellipse_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 72,
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(pi / 2.0)


def _side_plate_geometry(*, thickness: float):
    outer_profile = sample_catmull_rom_spline_2d(
        [
            (-0.40, 0.00),
            (-0.18, 0.00),
            (0.18, 0.00),
            (0.36, 0.00),
            (0.40, 0.10),
            (0.38, 0.34),
            (0.31, 0.57),
            (0.16, 0.78),
            (-0.02, 0.86),
            (-0.22, 0.81),
            (-0.35, 0.62),
            (-0.40, 0.30),
        ],
        samples_per_segment=10,
        closed=True,
    )
    lower_window = _translate_profile(
        rounded_rect_profile(0.34, 0.22, 0.045, corner_segments=8),
        dx=-0.05,
        dy=0.25,
    )
    axle_bore = _translate_profile(
        superellipse_profile(0.112, 0.112, exponent=2.0, segments=28),
        dx=0.0,
        dy=0.48,
    )
    return ExtrudeWithHolesGeometry(
        outer_profile,
        [lower_window, axle_bore],
        height=thickness,
        center=True,
    ).rotate_x(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_undershot_waterwheel")

    frame_paint = model.material("frame_paint", rgba=(0.28, 0.31, 0.34, 1.0))
    wheel_paint = model.material("wheel_paint", rgba=(0.44, 0.46, 0.48, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    elastomer_black = model.material("elastomer_black", rgba=(0.05, 0.05, 0.06, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.86, 0.44, 0.92)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    side_plate_mesh = _mesh("waterwheel_side_plate", _side_plate_geometry(thickness=0.018))
    frame.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.190, 0.0)),
        material=frame_paint,
        name="left_plate",
    )
    frame.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.190, 0.0)),
        material=frame_paint,
        name="right_plate",
    )

    frame.visual(
        Box((0.80, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, 0.190, 0.030)),
        material=frame_paint,
        name="left_skid",
    )
    frame.visual(
        Box((0.80, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, -0.190, 0.030)),
        material=frame_paint,
        name="right_skid",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.430),
        origin=Origin(xyz=(0.305, 0.0, 0.085), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="front_tie",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.430),
        origin=Origin(xyz=(-0.290, 0.0, 0.085), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="rear_tie",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.430),
        origin=Origin(xyz=(-0.290, 0.0, 0.760), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="top_tie",
    )

    for side_y, side_name in ((0.190, "left"), (-0.190, "right")):
        for x_pos, pad_name in ((-0.250, "rear"), (0.250, "front")):
            frame.visual(
                Box((0.150, 0.060, 0.016)),
                origin=Origin(xyz=(x_pos, side_y, 0.008)),
                material=elastomer_black,
                name=f"{side_name}_{pad_name}_foot",
            )

    left_bearing = model.part("left_bearing")
    left_bearing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.052),
        mass=0.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    bearing_sleeve_mesh = _mesh(
        "waterwheel_bearing_sleeve",
        _ring_shell(outer_radius=0.046, inner_radius=0.028, length=0.020, segments=56),
    )
    bearing_flange_mesh = _mesh(
        "waterwheel_bearing_flange",
        _ring_shell(outer_radius=0.066, inner_radius=0.028, length=0.016, segments=56),
    )
    inner_race_mesh = _mesh(
        "waterwheel_inner_race",
        _ring_shell(outer_radius=0.040, inner_radius=0.0255, length=0.010, segments=56),
    )
    left_bearing.visual(
        bearing_sleeve_mesh,
        material=polymer_dark,
        name="sleeve",
    )
    left_bearing.visual(
        bearing_flange_mesh,
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=polymer_dark,
        name="flange",
    )
    left_bearing.visual(
        inner_race_mesh,
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=brushed_steel,
        name="inner_race",
    )

    right_bearing = model.part("right_bearing")
    right_bearing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.052),
        mass=0.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    right_bearing.visual(
        bearing_sleeve_mesh,
        material=polymer_dark,
        name="sleeve",
    )
    right_bearing.visual(
        bearing_flange_mesh,
        origin=Origin(xyz=(0.0, -0.017, 0.0)),
        material=polymer_dark,
        name="flange",
    )
    right_bearing.visual(
        inner_race_mesh,
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=brushed_steel,
        name="inner_race",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.340, length=0.220),
        mass=16.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rim_mesh = _mesh(
        "waterwheel_rim",
        _ring_shell(outer_radius=0.340, inner_radius=0.306, length=0.016, segments=72),
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.103, 0.0)),
        material=wheel_paint,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -0.103, 0.0)),
        material=wheel_paint,
        name="right_rim",
    )
    wheel.visual(
        Cylinder(radius=0.024, length=0.400),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.080, length=0.220),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.094, length=0.018),
        origin=Origin(xyz=(0.0, 0.093, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="left_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.094, length=0.018),
        origin=Origin(xyz=(0.0, -0.093, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="right_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="left_axle_collar",
    )
    wheel.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(0.0, -0.170, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="right_axle_collar",
    )

    spoke_radius = 0.010
    spoke_length = 0.226
    spoke_mid_radius = 0.193
    for spoke_index in range(8):
        theta = pi / 8.0 + spoke_index * (pi / 4.0)
        for side_y, side_name in ((0.093, "left"), (-0.093, "right")):
            wheel.visual(
                Cylinder(radius=spoke_radius, length=spoke_length),
                origin=Origin(
                    xyz=(spoke_mid_radius * cos(theta), side_y, spoke_mid_radius * sin(theta)),
                    rpy=(0.0, pi / 2.0 - theta, 0.0),
                ),
                material=brushed_steel,
                name=f"{side_name}_spoke_{spoke_index:02d}",
            )

    paddle_radius = 0.300
    paddle_width = 0.198
    paddle_span = 0.094
    paddle_thickness = 0.016
    for paddle_index in range(12):
        theta = paddle_index * (pi / 6.0)
        wheel.visual(
            Box((paddle_span, paddle_width, paddle_thickness)),
            origin=Origin(
                xyz=(paddle_radius * cos(theta), 0.0, paddle_radius * sin(theta)),
                rpy=(0.0, pi / 2.0 - theta, 0.0),
            ),
            material=polymer_dark,
            name=f"paddle_{paddle_index:02d}",
        )

    model.articulation(
        "frame_to_left_bearing",
        ArticulationType.FIXED,
        parent=frame,
        child=left_bearing,
        origin=Origin(xyz=(0.0, 0.190, 0.480)),
    )
    model.articulation(
        "frame_to_right_bearing",
        ArticulationType.FIXED,
        parent=frame,
        child=right_bearing,
        origin=Origin(xyz=(0.0, -0.190, 0.480)),
    )
    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_bearing = object_model.get_part("left_bearing")
    right_bearing = object_model.get_part("right_bearing")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("frame_to_wheel")

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

    ctx.check(
        "wheel spin articulation is continuous",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        f"articulation_type={wheel_spin.articulation_type}",
    )
    ctx.check(
        "wheel spin axis is lateral",
        wheel_spin.axis == (0.0, 1.0, 0.0),
        f"axis={wheel_spin.axis}",
    )
    ctx.expect_contact(left_bearing, frame, name="left bearing mounted to frame")
    ctx.expect_contact(right_bearing, frame, name="right bearing mounted to frame")
    ctx.expect_origin_distance(
        left_bearing,
        wheel,
        axes="xz",
        max_dist=0.001,
        name="left bearing concentric with wheel axle",
    )
    ctx.expect_origin_distance(
        right_bearing,
        wheel,
        axes="xz",
        max_dist=0.001,
        name="right bearing concentric with wheel axle",
    )
    ctx.expect_origin_distance(
        left_bearing,
        right_bearing,
        axes="xz",
        max_dist=0.001,
        name="bearing pair shares one axle centerline",
    )

    with ctx.pose({wheel_spin: 0.0}):
        ctx.expect_gap(
            frame,
            wheel,
            axis="y",
            positive_elem="left_plate",
            negative_elem="left_rim",
            min_gap=0.055,
            max_gap=0.075,
            name="left rim clears left frame plate",
        )
        ctx.expect_gap(
            wheel,
            frame,
            axis="y",
            positive_elem="right_rim",
            negative_elem="right_plate",
            min_gap=0.055,
            max_gap=0.075,
            name="right rim clears right frame plate",
        )
        ctx.expect_gap(
            frame,
            wheel,
            axis="z",
            positive_elem="top_tie",
            negative_elem="hub_barrel",
            min_gap=0.15,
            name="top tie clears the rotating hub",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
