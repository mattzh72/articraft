from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _offset_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return hypot(hypot(b[0] - a[0], b[1] - a[1]), b[2] - a[2])


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(width: float, height: float, radius: float, x_pos: float):
    return [(x_pos, y, z) for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)]


def _build_pedal(part, *, side_sign: float, steel, rubber) -> None:
    spindle_offset = 0.018 * side_sign
    body_offset = 0.044 * side_sign
    part.visual(
        Cylinder(radius=0.009, length=0.036),
        origin=Origin(xyz=(0.0, spindle_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )
    part.visual(
        Box((0.108, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, body_offset, 0.0)),
        material=steel,
        name="platform",
    )
    part.visual(
        Box((0.096, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, body_offset, 0.010)),
        material=rubber,
        name="tread",
    )
    part.visual(
        Box((0.016, 0.022, 0.030)),
        origin=Origin(xyz=(0.045, body_offset, 0.0)),
        material=steel,
    )
    part.visual(
        Box((0.016, 0.022, 0.030)),
        origin=Origin(xyz=(-0.045, body_offset, 0.0)),
        material=steel,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_stationary_exercise_bike", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.22, 0.23, 0.24, 1.0))
    housing_plastic = model.material("housing_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    molded_black = model.material("molded_black", rgba=(0.10, 0.10, 0.11, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.15, 0.15, 0.16, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    flywheel_steel = model.material("flywheel_steel", rgba=(0.47, 0.49, 0.52, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.26, 0.62, 1.24)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )
    frame.visual(Box((0.14, 0.60, 0.06)), origin=Origin(xyz=(-0.50, 0.0, 0.03)), material=frame_paint, name="rear_stabilizer")
    frame.visual(Box((0.14, 0.52, 0.06)), origin=Origin(xyz=(0.52, 0.0, 0.03)), material=frame_paint, name="front_stabilizer")
    frame.visual(Box((1.04, 0.04, 0.06)), origin=Origin(xyz=(0.01, -0.09, 0.06)), material=frame_paint, name="left_floor_rail")
    frame.visual(Box((1.04, 0.04, 0.06)), origin=Origin(xyz=(0.01, 0.09, 0.06)), material=frame_paint, name="right_floor_rail")
    frame.visual(Box((0.30, 0.22, 0.12)), origin=Origin(xyz=(0.02, 0.0, 0.15)), material=dark_steel, name="crank_shell_lower")
    frame.visual(Box((0.34, 0.010, 0.38)), origin=Origin(xyz=(0.02, -0.180, 0.44)), material=housing_plastic, name="left_housing_plate")
    frame.visual(Box((0.34, 0.010, 0.38)), origin=Origin(xyz=(0.02, 0.180, 0.44)), material=housing_plastic, name="right_housing_plate")
    frame.visual(Box((0.30, 0.360, 0.04)), origin=Origin(xyz=(0.02, 0.0, 0.69)), material=housing_plastic, name="housing_top_band")
    frame.visual(Box((0.10, 0.360, 0.10)), origin=Origin(xyz=(0.18, 0.0, 0.72)), material=housing_plastic, name="housing_front_band")
    frame.visual(Box((0.08, 0.360, 0.18)), origin=Origin(xyz=(-0.16, 0.0, 0.72)), material=housing_plastic, name="housing_rear_band")
    frame.visual(Box((0.12, 0.08, 0.06)), origin=Origin(xyz=(-0.24, 0.0, 0.25)), material=dark_steel, name="seat_base_bridge")
    frame.visual(Box((0.28, 0.08, 0.06)), origin=Origin(xyz=(0.28, 0.0, 0.25)), material=dark_steel, name="handle_base_bridge")
    frame.visual(Box((0.09, 0.08, 0.58)), origin=Origin(xyz=(-0.24, 0.0, 0.34)), material=frame_paint, name="seat_mast")
    frame.visual(Box((0.09, 0.08, 0.68)), origin=Origin(xyz=(0.42, 0.0, 0.38)), material=frame_paint, name="handlebar_mast")
    frame.visual(Box((0.60, 0.08, 0.05)), origin=Origin(xyz=(0.13, 0.0, 0.65)), material=frame_paint, name="upper_bridge")
    frame.visual(Box((0.12, 0.12, 0.03)), origin=Origin(xyz=(0.16, 0.0, 0.685)), material=dark_steel, name="resistance_mount_web")
    frame.visual(Box((0.09, 0.12, 0.05)), origin=Origin(xyz=(0.18, 0.0, 0.725)), material=dark_steel, name="resistance_mount_block")
    for side_sign, side_name in [(-1.0, "left"), (1.0, "right")]:
        frame.visual(
            Cylinder(radius=0.035, length=0.074),
            origin=Origin(xyz=(0.02, side_sign * 0.152, 0.42), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"{side_name}_bearing_boss",
        )
        for x_pos, z_pos in [(0.14, 0.68)]:
            frame.visual(
                Cylinder(radius=0.010, length=0.016),
                origin=Origin(xyz=(x_pos, side_sign * 0.116, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
                material=steel,
            )

    seat_center = (-0.24, 0.0, 0.75)
    frame.visual(Box((0.010, 0.052, 0.24)), origin=Origin(xyz=(seat_center[0] - 0.028, 0.0, seat_center[2])), material=dark_steel, name="seat_sleeve_back_wall")
    frame.visual(Box((0.010, 0.052, 0.24)), origin=Origin(xyz=(seat_center[0] + 0.028, 0.0, seat_center[2])), material=dark_steel, name="seat_sleeve_front_wall")
    frame.visual(Box((0.046, 0.010, 0.24)), origin=Origin(xyz=(seat_center[0], -0.021, seat_center[2])), material=dark_steel)
    frame.visual(Box((0.046, 0.010, 0.24)), origin=Origin(xyz=(seat_center[0], 0.021, seat_center[2])), material=dark_steel)
    frame.visual(Box((0.030, 0.060, 0.050)), origin=Origin(xyz=(-0.278, 0.0, 0.77)), material=dark_steel, name="seat_clamp_block")
    frame.visual(Cylinder(radius=0.018, length=0.020), origin=Origin(xyz=(-0.303, 0.0, 0.77), rpy=(0.0, pi / 2.0, 0.0)), material=molded_black)

    handle_center = (0.44, 0.0, 0.84)
    frame.visual(Box((0.010, 0.062, 0.24)), origin=Origin(xyz=(handle_center[0] - 0.030, 0.0, handle_center[2])), material=dark_steel, name="handlebar_sleeve_back_wall")
    frame.visual(Box((0.010, 0.062, 0.24)), origin=Origin(xyz=(handle_center[0] + 0.030, 0.0, handle_center[2])), material=dark_steel, name="handlebar_sleeve_front_wall")
    frame.visual(Box((0.050, 0.010, 0.24)), origin=Origin(xyz=(handle_center[0], -0.026, handle_center[2])), material=dark_steel)
    frame.visual(Box((0.050, 0.010, 0.24)), origin=Origin(xyz=(handle_center[0], 0.026, handle_center[2])), material=dark_steel)
    frame.visual(Box((0.032, 0.072, 0.052)), origin=Origin(xyz=(0.489, 0.0, 0.86)), material=dark_steel, name="handlebar_clamp_block")
    frame.visual(Cylinder(radius=0.019, length=0.022), origin=Origin(xyz=(0.516, 0.0, 0.86), rpy=(0.0, pi / 2.0, 0.0)), material=molded_black)

    drivetrain = model.part("drivetrain")
    drivetrain.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.192),
        mass=18.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    drivetrain.visual(Cylinder(radius=0.19, length=0.040), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=flywheel_steel, name="flywheel")
    drivetrain.visual(Cylinder(radius=0.045, length=0.230), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name="hub_barrel")
    drivetrain.visual(Cylinder(radius=0.034, length=0.012), origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name="left_crank_spider")
    drivetrain.visual(Cylinder(radius=0.034, length=0.012), origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name="right_crank_spider")
    drivetrain.visual(Box((0.18, 0.020, 0.024)), origin=Origin(xyz=(-0.09, -0.094, 0.0)), material=frame_paint, name="left_crank_arm")
    drivetrain.visual(Box((0.18, 0.020, 0.024)), origin=Origin(xyz=(0.09, 0.094, 0.0)), material=frame_paint, name="right_crank_arm")
    drivetrain.visual(Cylinder(radius=0.015, length=0.012), origin=Origin(xyz=(-0.18, -0.108, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name="left_pedal_boss")
    drivetrain.visual(Cylinder(radius=0.015, length=0.012), origin=Origin(xyz=(0.18, 0.108, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name="right_pedal_boss")

    left_pedal = model.part("left_pedal")
    left_pedal.inertial = Inertial.from_geometry(Box((0.11, 0.09, 0.03)), mass=0.55, origin=Origin(xyz=(0.0, -0.05, 0.0)))
    _build_pedal(left_pedal, side_sign=-1.0, steel=dark_steel, rubber=grip_rubber)

    right_pedal = model.part("right_pedal")
    right_pedal.inertial = Inertial.from_geometry(Box((0.11, 0.09, 0.03)), mass=0.55, origin=Origin(xyz=(0.0, 0.05, 0.0)))
    _build_pedal(right_pedal, side_sign=1.0, steel=dark_steel, rubber=grip_rubber)

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(Box((0.09, 0.06, 0.60)), mass=5.0, origin=Origin(xyz=(0.0, 0.0, 0.30)))
    seat_post.visual(Box((0.046, 0.036, 0.56)), origin=Origin(xyz=(0.0, 0.0, 0.28)), material=steel, name="post_shaft")
    seat_post.visual(Box((0.10, 0.05, 0.04)), origin=Origin(xyz=(0.008, 0.0, 0.56)), material=dark_steel, name="saddle_clamp_head")
    seat_post.visual(Box((0.030, 0.024, 0.04)), origin=Origin(xyz=(-0.028, 0.0, 0.56)), material=dark_steel, name="saddle_pivot_lug")

    saddle = model.part("saddle")
    saddle.inertial = Inertial.from_geometry(Box((0.32, 0.20, 0.10)), mass=2.3, origin=Origin(xyz=(0.0, 0.0, 0.05)))
    saddle.visual(Box((0.04, 0.05, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_steel, name="pivot_block")
    saddle.visual(Box((0.18, 0.18, 0.05)), origin=Origin(xyz=(-0.05, 0.0, 0.045)), material=saddle_vinyl, name="rear_pad")
    saddle.visual(Box((0.20, 0.10, 0.04)), origin=Origin(xyz=(0.08, 0.0, 0.040)), material=saddle_vinyl, name="nose_pad")

    handlebar_post = model.part("handlebar_post")
    handlebar_post.inertial = Inertial.from_geometry(Box((0.12, 0.08, 0.58)), mass=5.8, origin=Origin(xyz=(0.0, 0.0, 0.29)))
    handlebar_post.visual(Box((0.050, 0.038, 0.54)), origin=Origin(xyz=(0.0, 0.0, 0.27)), material=steel, name="mast_shaft")
    handlebar_post.visual(Box((0.11, 0.06, 0.04)), origin=Origin(xyz=(0.008, 0.0, 0.56)), material=dark_steel, name="handlebar_pivot_head")
    handlebar_post.visual(Box((0.032, 0.025, 0.04)), origin=Origin(xyz=(-0.030, 0.0, 0.56)), material=dark_steel, name="handlebar_pivot_lug")

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(Box((0.28, 0.62, 0.24)), mass=3.6, origin=Origin(xyz=(0.05, 0.0, 0.12)))
    handlebar.visual(Box((0.04, 0.06, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_steel, name="pivot_block")
    handlebar.visual(Box((0.10, 0.04, 0.12)), origin=Origin(xyz=(0.05, 0.0, 0.08)), material=dark_steel, name="center_stem")
    _add_member(handlebar, (0.08, 0.0, 0.13), (0.02, -0.24, 0.07), 0.016, dark_steel, name="left_bar")
    _add_member(handlebar, (0.08, 0.0, 0.13), (0.02, 0.24, 0.07), 0.016, dark_steel, name="right_bar")
    handlebar.visual(Box((0.10, 0.07, 0.025)), origin=Origin(xyz=(0.09, 0.0, 0.125)), material=housing_plastic, name="utility_tray")
    handlebar.visual(Cylinder(radius=0.021, length=0.070), origin=Origin(xyz=(0.02, -0.275, 0.07), rpy=(pi / 2.0, 0.0, 0.0)), material=grip_rubber, name="left_grip")
    handlebar.visual(Cylinder(radius=0.021, length=0.070), origin=Origin(xyz=(0.02, 0.275, 0.07), rpy=(pi / 2.0, 0.0, 0.0)), material=grip_rubber, name="right_grip")

    resistance_knob = model.part("resistance_knob")
    resistance_knob.inertial = Inertial.from_geometry(Cylinder(radius=0.038, length=0.050), mass=0.25, origin=Origin(xyz=(0.0, 0.0, 0.010)))
    resistance_knob.visual(Cylinder(radius=0.010, length=0.030), origin=Origin(xyz=(0.0, 0.0, -0.015)), material=steel, name="spindle")
    resistance_knob.visual(Cylinder(radius=0.035, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.013)), material=molded_black, name="knob_body")
    resistance_knob.visual(Box((0.018, 0.006, 0.018)), origin=Origin(xyz=(0.031, 0.0, 0.015)), material=steel, name="indicator")

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=drivetrain,
        origin=Origin(xyz=(0.02, 0.0, 0.43)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=drivetrain,
        child=left_pedal,
        origin=Origin(xyz=(-0.18, -0.114, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=drivetrain,
        child=right_pedal,
        origin=Origin(xyz=(0.18, 0.114, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.24, 0.0, 0.63)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.10),
    )
    model.articulation(
        "saddle_tilt",
        ArticulationType.REVOLUTE,
        parent=seat_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.5, lower=-0.16, upper=0.14),
    )
    model.articulation(
        "handlebar_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handlebar_post,
        origin=Origin(xyz=(0.44, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.10),
    )
    model.articulation(
        "handlebar_tilt",
        ArticulationType.REVOLUTE,
        parent=handlebar_post,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.6, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "resistance_adjust",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=resistance_knob,
        origin=Origin(xyz=(0.18, 0.0, 0.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.4, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    drivetrain = object_model.get_part("drivetrain")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")
    seat_post = object_model.get_part("seat_post")
    saddle = object_model.get_part("saddle")
    handlebar_post = object_model.get_part("handlebar_post")
    handlebar = object_model.get_part("handlebar")
    resistance_knob = object_model.get_part("resistance_knob")

    crank_spin = object_model.get_articulation("crank_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    seat_height = object_model.get_articulation("seat_height")
    saddle_tilt = object_model.get_articulation("saddle_tilt")
    handlebar_height = object_model.get_articulation("handlebar_height")
    handlebar_tilt = object_model.get_articulation("handlebar_tilt")
    resistance_adjust = object_model.get_articulation("resistance_adjust")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(seat_post, saddle, reason="Saddle pivot block nests into the seatpost clamp head.")
    ctx.allow_overlap(handlebar_post, handlebar, reason="Handlebar pivot block nests into the mast head clamp.")
    ctx.allow_overlap(frame, resistance_knob, reason="Knob spindle seats into the resistance mount block.")
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(drivetrain, frame, name="drivetrain_supported_at_bearing_bosses")
    ctx.expect_contact(left_pedal, drivetrain, name="left_pedal_connected")
    ctx.expect_contact(right_pedal, drivetrain, name="right_pedal_connected")
    ctx.expect_contact(seat_post, frame, name="seat_post_guided_in_sleeve")
    ctx.expect_contact(saddle, seat_post, name="saddle_supported_by_clamp")
    ctx.expect_contact(handlebar_post, frame, name="handlebar_post_guided_in_sleeve")
    ctx.expect_contact(handlebar, handlebar_post, name="handlebar_supported_by_pivot")
    ctx.expect_contact(resistance_knob, frame, name="resistance_knob_mounted")

    ctx.expect_origin_gap(saddle, drivetrain, axis="z", min_gap=0.50, name="saddle_above_crank")
    ctx.expect_origin_gap(handlebar, drivetrain, axis="x", min_gap=0.30, name="handlebar_forward_of_crank")
    ctx.expect_origin_distance(handlebar, saddle, axes="x", min_dist=0.50, name="rider_triangle_stretched")

    seat_rest = ctx.part_world_position(seat_post)
    handle_rest = ctx.part_world_position(handlebar_post)
    assert seat_rest is not None
    assert handle_rest is not None

    with ctx.pose({seat_height: 0.10}):
        seat_high = ctx.part_world_position(seat_post)
        assert seat_high is not None
        ctx.check(
            "seat_height_moves_up",
            seat_high[2] > seat_rest[2] + 0.09,
            f"seat_post z did not rise enough: rest={seat_rest[2]:.3f}, high={seat_high[2]:.3f}",
        )
        ctx.expect_contact(seat_post, frame, name="seat_height_upper_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="seat_height_upper_no_overlap")

    with ctx.pose({handlebar_height: 0.10}):
        handle_high = ctx.part_world_position(handlebar_post)
        assert handle_high is not None
        ctx.check(
            "handlebar_height_moves_up",
            handle_high[2] > handle_rest[2] + 0.09,
            f"handlebar_post z did not rise enough: rest={handle_rest[2]:.3f}, high={handle_high[2]:.3f}",
        )
        ctx.expect_contact(handlebar_post, frame, name="handlebar_height_upper_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="handlebar_height_upper_no_overlap")

    saddle_rest_aabb = ctx.part_element_world_aabb(saddle, elem="nose_pad")
    assert saddle_rest_aabb is not None
    with ctx.pose({saddle_tilt: 0.14}):
        saddle_up_aabb = ctx.part_element_world_aabb(saddle, elem="nose_pad")
        assert saddle_up_aabb is not None
        ctx.check(
            "saddle_tilt_changes_shell_pose",
            saddle_up_aabb[1][2] > saddle_rest_aabb[1][2] + 0.004,
            f"saddle max z did not increase: rest={saddle_rest_aabb[1][2]:.3f}, up={saddle_up_aabb[1][2]:.3f}",
        )
        ctx.expect_contact(saddle, seat_post, name="saddle_tilt_upper_contact")

    tray_rest_aabb = ctx.part_element_world_aabb(handlebar, elem="utility_tray")
    assert tray_rest_aabb is not None
    with ctx.pose({handlebar_tilt: 0.30}):
        tray_tilt_aabb = ctx.part_element_world_aabb(handlebar, elem="utility_tray")
        assert tray_tilt_aabb is not None
        ctx.check(
            "handlebar_tilt_moves_tray",
            tray_tilt_aabb[1][2] > tray_rest_aabb[1][2] + 0.015,
            f"tray max z did not increase enough: rest={tray_rest_aabb[1][2]:.3f}, tilt={tray_tilt_aabb[1][2]:.3f}",
        )
        ctx.expect_contact(handlebar, handlebar_post, name="handlebar_tilt_upper_contact")

    with ctx.pose({crank_spin: 0.0, right_pedal_spin: 0.0, left_pedal_spin: 0.0}):
        pedal_front = ctx.part_world_position(right_pedal)
        assert pedal_front is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="crank_zero_no_overlap")
    with ctx.pose({crank_spin: pi / 2.0, right_pedal_spin: 0.0, left_pedal_spin: 0.0}):
        pedal_top = ctx.part_world_position(right_pedal)
        assert pedal_top is not None
        ctx.check(
            "crank_rotation_moves_right_pedal",
            pedal_top[2] > pedal_front[2] + 0.15 and pedal_top[0] < pedal_front[0] - 0.12,
            f"right pedal did not arc upward/backward enough: front={pedal_front}, top={pedal_top}",
        )
        ctx.expect_contact(drivetrain, frame, name="crank_quarter_turn_bearing_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="crank_quarter_turn_no_overlap")

    indicator_rest = ctx.part_element_world_aabb(resistance_knob, elem="indicator")
    assert indicator_rest is not None
    with ctx.pose({resistance_adjust: 1.1}):
        indicator_turn = ctx.part_element_world_aabb(resistance_knob, elem="indicator")
        assert indicator_turn is not None
        ctx.check(
            "resistance_knob_rotates_indicator",
            indicator_turn[1][1] > indicator_rest[1][1] + 0.018,
            f"indicator did not swing around knob: rest={indicator_rest}, turned={indicator_turn}",
        )
        ctx.expect_contact(resistance_knob, frame, name="resistance_knob_rotated_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
