from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi

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
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_stationary_bike")

    frame_black = model.material("frame_black", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    housing_grey = model.material("housing_grey", rgba=(0.68, 0.70, 0.73, 1.0))
    accent_red = model.material("accent_red", rgba=(0.73, 0.16, 0.14, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.11, 0.11, 0.12, 1.0))
    grip_black = model.material("grip_black", rgba=(0.06, 0.06, 0.07, 1.0))
    display_dark = model.material("display_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.46, 0.72)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )
    frame.visual(
        Cylinder(radius=0.028, length=0.42),
        origin=Origin(xyz=(-0.30, 0.0, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_stabilizer",
    )
    frame.visual(
        Cylinder(radius=0.028, length=0.44),
        origin=Origin(xyz=(0.31, 0.0, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_stabilizer",
    )
    _add_tube(
        frame,
        (-0.30, 0.0, 0.055),
        (-0.19, 0.0, 0.34),
        radius=0.024,
        material=frame_black,
        name="rear_upright",
    )
    _add_tube(
        frame,
        (-0.19, 0.0, 0.34),
        (-0.20, 0.0, 0.58),
        radius=0.027,
        material=frame_black,
        name="seat_tube",
    )
    _add_tube(
        frame,
        (-0.19, 0.0, 0.34),
        (-0.01, 0.0, 0.33),
        radius=0.028,
        material=frame_black,
        name="main_beam",
    )
    _add_tube(
        frame,
        (-0.30, 0.0, 0.055),
        (-0.04, 0.0, 0.18),
        radius=0.020,
        material=frame_black,
        name="lower_rear_brace",
    )
    _add_tube(
        frame,
        (-0.04, 0.0, 0.18),
        (0.12, 0.0, 0.21),
        radius=0.020,
        material=frame_black,
        name="lower_front_brace",
    )
    frame.visual(
        Box((0.050, 0.160, 0.040)),
        origin=Origin(xyz=(0.12, 0.0, 0.21)),
        material=dark_metal,
        name="front_lower_node",
    )
    _add_tube(
        frame,
        (0.12, 0.0, 0.21),
        (0.108, 0.0, 0.365),
        radius=0.022,
        material=frame_black,
        name="front_upright",
    )
    _add_tube(
        frame,
        (0.12, 0.070, 0.21),
        (0.31, 0.070, 0.055),
        radius=0.018,
        material=frame_black,
        name="right_front_support",
    )
    _add_tube(
        frame,
        (0.12, -0.070, 0.21),
        (0.31, -0.070, 0.055),
        radius=0.018,
        material=frame_black,
        name="left_front_support",
    )
    frame.visual(
        Box((0.090, 0.052, 0.060)),
        origin=Origin(xyz=(-0.040, 0.0, 0.225)),
        material=dark_metal,
        name="bb_bridge",
    )
    frame.visual(
        Box((0.050, 0.080, 0.090)),
        origin=Origin(xyz=(0.050, 0.0, 0.305)),
        material=dark_metal,
        name="housing_mount_plate",
    )
    _add_tube(
        frame,
        (-0.005, 0.0, 0.331),
        (0.025, 0.0, 0.305),
        radius=0.014,
        material=dark_metal,
        name="housing_brace",
    )
    frame.visual(
        Box((0.050, 0.024, 0.070)),
        origin=Origin(xyz=(0.0, 0.038, 0.27)),
        material=steel,
        name="right_bb_cup",
    )
    frame.visual(
        Box((0.050, 0.024, 0.070)),
        origin=Origin(xyz=(0.0, -0.038, 0.27)),
        material=steel,
        name="left_bb_cup",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(-0.20, 0.0, 0.60)),
        material=dark_metal,
        name="seat_collar",
    )
    frame.visual(
        Box((0.050, 0.040, 0.025)),
        origin=Origin(xyz=(-0.160, 0.0, 0.60)),
        material=dark_metal,
        name="seat_clamp_ear",
    )
    frame.visual(
        Box((0.028, 0.012, 0.100)),
        origin=Origin(xyz=(0.107, 0.056, 0.405)),
        material=dark_metal,
        name="hinge_plate_right",
    )
    frame.visual(
        Box((0.028, 0.012, 0.100)),
        origin=Origin(xyz=(0.107, -0.056, 0.405)),
        material=dark_metal,
        name="hinge_plate_left",
    )
    frame.visual(
        Box((0.030, 0.112, 0.030)),
        origin=Origin(xyz=(0.107, 0.0, 0.375)),
        material=dark_metal,
        name="hinge_block",
    )

    flywheel_housing = model.part("flywheel_housing")
    flywheel_housing.inertial = Inertial.from_geometry(
        Box((0.36, 0.12, 0.36)),
        mass=6.0,
    )
    flywheel_housing.visual(
        Cylinder(radius=0.140, length=0.072),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=housing_grey,
        name="housing_body",
    )
    flywheel_housing.visual(
        Cylinder(radius=0.150, length=0.012),
        origin=Origin(xyz=(0.080, -0.036, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=housing_grey,
        name="left_cover",
    )
    flywheel_housing.visual(
        Cylinder(radius=0.150, length=0.012),
        origin=Origin(xyz=(0.080, 0.036, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=housing_grey,
        name="right_cover",
    )
    flywheel_housing.visual(
        Cylinder(radius=0.053, length=0.086),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub_tunnel",
    )
    flywheel_housing.visual(
        Box((0.120, 0.080, 0.070)),
        origin=Origin(xyz=(0.095, 0.0, 0.080)),
        material=accent_red,
        name="top_cover",
    )
    flywheel_housing.visual(
        Box((0.080, 0.070, 0.100)),
        origin=Origin(xyz=(-0.100, 0.0, 0.090)),
        material=dark_metal,
        name="mount_boss",
    )

    crankset = model.part("crankset")
    crankset.inertial = Inertial.from_geometry(
        Box((0.52, 0.22, 0.18)),
        mass=3.5,
    )
    crankset.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.046, length=0.016),
        origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_bearing_collar",
    )
    crankset.visual(
        Cylinder(radius=0.046, length=0.016),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_bearing_collar",
    )
    crankset.visual(
        Cylinder(radius=0.066, length=0.012),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="chainring_cover",
    )
    crankset.visual(
        Box((0.140, 0.016, 0.036)),
        origin=Origin(xyz=(0.100, 0.072, 0.0)),
        material=dark_metal,
        name="right_arm",
    )
    crankset.visual(
        Box((0.140, 0.016, 0.036)),
        origin=Origin(xyz=(-0.100, -0.072, 0.0)),
        material=dark_metal,
        name="left_arm",
    )
    crankset.visual(
        Cylinder(radius=0.009, length=0.042),
        origin=Origin(xyz=(0.170, 0.097, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_pedal_spindle",
    )
    crankset.visual(
        Cylinder(radius=0.009, length=0.042),
        origin=Origin(xyz=(-0.170, -0.097, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_pedal_spindle",
    )
    crankset.visual(
        Box((0.090, 0.028, 0.016)),
        origin=Origin(xyz=(0.215, 0.118, 0.0)),
        material=grip_black,
        name="right_pedal",
    )
    crankset.visual(
        Box((0.090, 0.028, 0.016)),
        origin=Origin(xyz=(-0.215, -0.118, 0.0)),
        material=grip_black,
        name="left_pedal",
    )

    saddle_post = model.part("saddle_post")
    saddle_post.inertial = Inertial.from_geometry(
        Box((0.28, 0.18, 0.42)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
    )
    saddle_post.visual(
        Cylinder(radius=0.026, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=steel,
        name="post_shaft",
    )
    saddle_post.visual(
        Box((0.050, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material=dark_metal,
        name="seat_mount",
    )
    saddle_post.visual(
        Box((0.170, 0.080, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, 0.325)),
        material=dark_metal,
        name="saddle_base",
    )
    saddle_post.visual(
        Box((0.180, 0.150, 0.050)),
        origin=Origin(xyz=(-0.030, 0.0, 0.350)),
        material=saddle_black,
        name="saddle_rear",
    )
    saddle_post.visual(
        Box((0.210, 0.090, 0.035)),
        origin=Origin(xyz=(0.050, 0.0, 0.345)),
        material=saddle_black,
        name="saddle_nose",
    )

    handlebar_mast = model.part("handlebar_mast")
    handlebar_mast.inertial = Inertial.from_geometry(
        Box((0.46, 0.44, 0.60)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )
    handlebar_mast.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    _add_tube(
        handlebar_mast,
        (0.0, 0.0, 0.0),
        (0.060, 0.0, 0.300),
        radius=0.022,
        material=frame_black,
        name="mast_tube",
    )
    handlebar_mast.visual(
        Box((0.090, 0.080, 0.050)),
        origin=Origin(xyz=(0.065, 0.0, 0.225)),
        material=dark_metal,
        name="bar_stem_block",
    )
    handlebar_mast.visual(
        Cylinder(radius=0.015, length=0.300),
        origin=Origin(xyz=(0.070, 0.0, 0.220), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="bar_cross",
    )
    handlebar_mast.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(0.070, -0.1975, 0.220), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    handlebar_mast.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(0.070, 0.1975, 0.220), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    handlebar_mast.visual(
        Box((0.080, 0.060, 0.040)),
        origin=Origin(xyz=(0.095, 0.0, 0.250)),
        material=display_dark,
        name="console",
    )
    handlebar_mast.visual(
        Box((0.045, 0.080, 0.050)),
        origin=Origin(xyz=(0.030, 0.0, 0.120)),
        material=dark_metal,
        name="mast_clamp",
    )

    seat_clamp_knob = model.part("seat_clamp_knob")
    seat_clamp_knob.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
    )
    seat_clamp_knob.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="stem",
    )
    seat_clamp_knob.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.042, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="knob",
    )
    seat_clamp_knob.visual(
        Box((0.012, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.042, 0.0)),
        material=grip_black,
        name="lobe",
    )

    mast_lock_knob = model.part("mast_lock_knob")
    mast_lock_knob.inertial = Inertial.from_geometry(
        Box((0.060, 0.070, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
    )
    mast_lock_knob.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="stem",
    )
    mast_lock_knob.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.044, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="knob",
    )
    mast_lock_knob.visual(
        Box((0.014, 0.010, 0.042)),
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
        material=grip_black,
        name="lobe",
    )

    model.articulation(
        "frame_to_flywheel_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=flywheel_housing,
        origin=Origin(xyz=(0.215, 0.0, 0.21)),
    )
    model.articulation(
        "frame_to_crankset",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_saddle_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle_post,
        origin=Origin(xyz=(-0.20, 0.0, 0.625)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.18, lower=0.0, upper=0.16),
    )
    model.articulation(
        "frame_to_handlebar_mast",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handlebar_mast,
        origin=Origin(xyz=(0.145, 0.0, 0.42)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-1.20, upper=0.05),
    )
    model.articulation(
        "frame_to_seat_clamp_knob",
        ArticulationType.FIXED,
        parent=frame,
        child=seat_clamp_knob,
        origin=Origin(xyz=(-0.145, 0.020, 0.60)),
    )
    model.articulation(
        "frame_to_mast_lock_knob",
        ArticulationType.FIXED,
        parent=frame,
        child=mast_lock_knob,
        origin=Origin(xyz=(0.114, 0.062, 0.40)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flywheel_housing = object_model.get_part("flywheel_housing")
    crankset = object_model.get_part("crankset")
    saddle_post = object_model.get_part("saddle_post")
    handlebar_mast = object_model.get_part("handlebar_mast")
    seat_clamp_knob = object_model.get_part("seat_clamp_knob")
    mast_lock_knob = object_model.get_part("mast_lock_knob")

    crank_joint = object_model.get_articulation("frame_to_crankset")
    seat_joint = object_model.get_articulation("frame_to_saddle_post")
    bar_joint = object_model.get_articulation("frame_to_handlebar_mast")

    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="right_bb_cup",
        elem_b="spindle",
        reason="bottom bracket spindle intentionally passes through the right bearing block",
    )
    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="left_bb_cup",
        elem_b="spindle",
        reason="bottom bracket spindle intentionally passes through the left bearing block",
    )
    ctx.allow_overlap(
        frame,
        flywheel_housing,
        elem_a="front_upright",
        elem_b="mount_boss",
        reason="flywheel housing uses a captured clamp boss around the front upright tube",
    )

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=10,
        name="moving_clearance_samples",
    )

    ctx.expect_contact(
        frame,
        crankset,
        elem_a="right_bb_cup",
        elem_b="right_bearing_collar",
        name="right_crank_bearing_mount",
    )
    ctx.expect_contact(
        frame,
        crankset,
        elem_a="left_bb_cup",
        elem_b="left_bearing_collar",
        name="left_crank_bearing_mount",
    )
    ctx.expect_contact(
        frame,
        flywheel_housing,
        elem_a="housing_mount_plate",
        elem_b="mount_boss",
        name="housing_is_bolted_to_frame_mount",
    )
    ctx.expect_contact(
        frame,
        seat_clamp_knob,
        elem_a="seat_clamp_ear",
        elem_b="stem",
        name="seat_adjuster_is_mounted_on_clamp_ear",
    )
    ctx.expect_contact(
        frame,
        mast_lock_knob,
        elem_a="hinge_plate_right",
        elem_b="stem",
        name="mast_lock_is_mounted_on_hinge_plate",
    )
    ctx.expect_contact(
        frame,
        saddle_post,
        elem_a="seat_collar",
        elem_b="post_shaft",
        name="seatpost_starts_at_collar",
    )
    ctx.expect_overlap(
        saddle_post,
        frame,
        axes="xy",
        elem_a="post_shaft",
        elem_b="seat_collar",
        min_overlap=0.05,
        name="seatpost_stays_centered_in_collar_projection",
    )

    with ctx.pose({crank_joint: 0.0}):
        ctx.expect_gap(
            crankset,
            flywheel_housing,
            axis="y",
            positive_elem="right_pedal",
            negative_elem="housing_body",
            min_gap=0.025,
            name="right_pedal_clears_housing_side",
        )
        ctx.expect_gap(
            flywheel_housing,
            crankset,
            axis="y",
            positive_elem="housing_body",
            negative_elem="left_pedal",
            min_gap=0.025,
            name="left_pedal_clears_housing_side",
        )

    with ctx.pose({seat_joint: 0.16}):
        ctx.expect_gap(
            saddle_post,
            frame,
            axis="z",
            positive_elem="seat_mount",
            negative_elem="seat_collar",
            min_gap=0.09,
            name="raised_saddle_clears_collar",
        )

    with ctx.pose({bar_joint: -1.05}):
        ctx.expect_gap(
            handlebar_mast,
            flywheel_housing,
            axis="z",
            positive_elem="bar_cross",
            negative_elem="housing_body",
            min_gap=0.015,
            max_gap=0.120,
            name="folded_bar_clears_housing_top",
        )
        ctx.expect_overlap(
            handlebar_mast,
            flywheel_housing,
            axes="xy",
            elem_a="bar_cross",
            elem_b="housing_body",
            min_overlap=0.020,
            name="folded_bar_stays_over_compact_body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
