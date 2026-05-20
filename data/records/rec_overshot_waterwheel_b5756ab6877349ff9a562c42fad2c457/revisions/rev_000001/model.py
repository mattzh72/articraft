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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_points(radius: float, *, y: float, samples: int = 18) -> list[tuple[float, float, float]]:
    return [
        (
            radius * cos(2.0 * pi * index / samples),
            y,
            radius * sin(2.0 * pi * index / samples),
        )
        for index in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rural_overshot_waterwheel")

    weathered_oak = model.material("weathered_oak", rgba=(0.47, 0.35, 0.23, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.36, 0.26, 0.17, 1.0))
    aged_plank = model.material("aged_plank", rgba=(0.58, 0.44, 0.30, 1.0))
    ironwork = model.material("ironwork", rgba=(0.24, 0.25, 0.27, 1.0))

    wheel_radius = 1.20
    wheel_width = 0.34
    rim_radius = 0.030
    rim_center_offset = 0.14
    axle_radius = 0.045
    axle_length = 0.44
    axle_height = 1.45
    launder_length = 0.64
    launder_center_x = -0.50
    launder_front_x = launder_center_x + launder_length / 2.0
    launder_support_center_x = -0.44
    launder_support_length = 0.76
    launder_support_y = 0.24
    launder_outer_width = 0.16
    launder_inner_width = 0.10
    launder_bottom_z = 2.72
    launder_wall_z = 2.81

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.10, 0.86, 2.70)),
        mass=260.0,
        origin=Origin(xyz=(-0.45, 0.0, 1.35)),
    )

    frame.visual(
        Box((0.12, 0.12, 2.84)),
        origin=Origin(xyz=(0.0, -0.31, 1.42)),
        material=weathered_oak,
        name="left_post",
    )
    frame.visual(
        Box((0.12, 0.12, 2.84)),
        origin=Origin(xyz=(0.0, 0.31, 1.42)),
        material=weathered_oak,
        name="right_post",
    )
    frame.visual(
        Box((0.10, 0.74, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.74)),
        material=weathered_oak,
        name="cap_beam",
    )
    frame.visual(
        Box((0.16, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, -0.26, axle_height)),
        material=weathered_oak,
        name="left_bearing_block",
    )
    frame.visual(
        Box((0.16, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.26, axle_height)),
        material=weathered_oak,
        name="right_bearing_block",
    )

    launder = model.part("launder")
    launder.inertial = Inertial.from_geometry(
        Box((launder_support_length, 0.52, 0.42)),
        mass=55.0,
        origin=Origin(xyz=(launder_support_center_x, 0.0, 2.61)),
    )
    launder.visual(
        Box((launder_support_length, 0.06, 0.03)),
        origin=Origin(xyz=(launder_support_center_x, -launder_support_y, 2.685)),
        material=weathered_oak,
        name="left_launder_support",
    )
    launder.visual(
        Box((launder_support_length, 0.06, 0.03)),
        origin=Origin(xyz=(launder_support_center_x, launder_support_y, 2.685)),
        material=weathered_oak,
        name="right_launder_support",
    )
    launder.visual(
        Box((0.08, 0.40, 0.03)),
        origin=Origin(xyz=(-0.76, 0.0, 2.685)),
        material=weathered_oak,
        name="rear_trestle_tie",
    )
    launder.visual(
        Box((0.08, 0.40, 0.03)),
        origin=Origin(xyz=(-0.50, 0.0, 2.685)),
        material=weathered_oak,
        name="mid_trestle_tie",
    )
    launder.visual(
        Box((0.08, 0.40, 0.03)),
        origin=Origin(xyz=(-0.24, 0.0, 2.685)),
        material=weathered_oak,
        name="front_trestle_tie",
    )
    launder.visual(
        Box((0.08, 0.08, 0.36)),
        origin=Origin(xyz=(-0.80, -launder_support_y, 2.505)),
        material=weathered_oak,
        name="left_rear_trestle_post",
    )
    launder.visual(
        Box((0.08, 0.08, 0.36)),
        origin=Origin(xyz=(-0.80, launder_support_y, 2.505)),
        material=weathered_oak,
        name="right_rear_trestle_post",
    )
    launder.visual(
        Box((launder_length, launder_outer_width, 0.04)),
        origin=Origin(xyz=(launder_center_x, 0.0, launder_bottom_z)),
        material=aged_plank,
        name="launder_bottom",
    )
    launder.visual(
        Box((launder_length, 0.04, 0.14)),
        origin=Origin(xyz=(launder_center_x, -0.08, launder_wall_z)),
        material=aged_plank,
        name="launder_left_wall",
    )
    launder.visual(
        Box((launder_length, 0.04, 0.14)),
        origin=Origin(xyz=(launder_center_x, 0.08, launder_wall_z)),
        material=aged_plank,
        name="launder_right_wall",
    )
    launder.visual(
        Box((0.04, launder_outer_width, 0.14)),
        origin=Origin(xyz=(launder_center_x - launder_length / 2.0 + 0.02, 0.0, launder_wall_z)),
        material=aged_plank,
        name="launder_backboard",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=axle_length),
        mass=145.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rim_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _circle_points(wheel_radius, y=0.0, samples=20),
            radius=rim_radius,
            samples_per_segment=6,
            radial_segments=16,
            closed_spline=True,
            cap_ends=False,
        ),
        "waterwheel_rim_ring",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -rim_center_offset, 0.0)),
        material=wet_timber,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, rim_center_offset, 0.0)),
        material=wet_timber,
        name="right_rim",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.30),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wet_timber,
        name="wheel_hub",
    )
    wheel.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=ironwork,
        name="wheel_axle",
    )

    spoke_count = 12
    for index in range(spoke_count):
        angle = 2.0 * pi * index / spoke_count
        wheel.visual(
            Box((1.24, 0.28, 0.045)),
            origin=Origin(
                xyz=(0.62 * cos(angle), 0.0, 0.62 * sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=weathered_oak,
            name=f"spoke_{index}",
        )

    paddle_count = 16
    for index in range(paddle_count):
        angle = 2.0 * pi * index / paddle_count
        wheel.visual(
            Box((0.16, wheel_width, 0.20)),
            origin=Origin(
                xyz=(1.10 * cos(angle), 0.0, 1.10 * sin(angle)),
                rpy=(0.0, -(angle + pi / 2.0), 0.0),
            ),
            material=aged_plank,
            name=f"bucket_board_{index}",
        )

    shutoff_flap = model.part("shutoff_flap")
    shutoff_flap.inertial = Inertial.from_geometry(
        Box((0.024, launder_inner_width, 0.18)),
        mass=5.5,
        origin=Origin(xyz=(0.012, 0.0, -0.09)),
    )
    shutoff_flap.visual(
        Cylinder(radius=0.012, length=launder_inner_width + 0.02),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=ironwork,
        name="flap_hinge_barrel",
    )
    shutoff_flap.visual(
        Box((0.024, launder_inner_width, 0.18)),
        origin=Origin(xyz=(0.012, 0.0, -0.09)),
        material=aged_plank,
        name="flap_board",
    )

    model.articulation(
        "frame_to_launder",
        ArticulationType.FIXED,
        parent=frame,
        child=launder,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.4),
    )
    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=launder,
        child=shutoff_flap,
        origin=Origin(xyz=(launder_front_x, 0.0, 2.88)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.15),
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

    frame = object_model.get_part("frame")
    launder = object_model.get_part("launder")
    wheel = object_model.get_part("wheel")
    shutoff_flap = object_model.get_part("shutoff_flap")
    launder_mount = object_model.get_articulation("frame_to_launder")
    wheel_spin = object_model.get_articulation("frame_to_wheel")
    flap_hinge = object_model.get_articulation("frame_to_flap")

    ctx.check(
        "launder is a fixed timber trough assembly carried by the frame",
        launder_mount.articulation_type == ArticulationType.FIXED
        and launder is not None,
        details=f"type={launder_mount.articulation_type}",
    )
    ctx.check(
        "wheel uses a continuous axle rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "flap hinge opens about the launder width axis",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE
        and flap_hinge.axis == (0.0, 1.0, 0.0)
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.lower == 0.0
        and flap_hinge.motion_limits.upper is not None
        and flap_hinge.motion_limits.upper >= 1.0,
        details=f"type={flap_hinge.articulation_type}, axis={flap_hinge.axis}, limits={flap_hinge.motion_limits}",
    )

    ctx.expect_contact(
        launder,
        frame,
        elem_a="left_launder_support",
        elem_b="left_post",
        name="left launder rail bears against left support post",
    )
    ctx.expect_contact(
        launder,
        frame,
        elem_a="right_launder_support",
        elem_b="right_post",
        name="right launder rail bears against right support post",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="wheel_axle",
        elem_b="left_bearing_block",
        name="left axle journal meets left timber bearing",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="wheel_axle",
        elem_b="right_bearing_block",
        name="right axle journal meets right timber bearing",
    )
    ctx.expect_contact(
        shutoff_flap,
        launder,
        elem_a="flap_hinge_barrel",
        elem_b="launder_left_wall",
        name="flap hinge barrel seats against left launder cheek",
    )
    ctx.expect_contact(
        shutoff_flap,
        launder,
        elem_a="flap_hinge_barrel",
        elem_b="launder_right_wall",
        name="flap hinge barrel seats against right launder cheek",
    )

    closed_board = ctx.part_element_world_aabb(shutoff_flap, elem="flap_board")
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_board = ctx.part_element_world_aabb(shutoff_flap, elem="flap_board")
    ctx.check(
        "shutoff flap swings upward and upstream",
        closed_board is not None
        and open_board is not None
        and open_board[0][0] < closed_board[0][0] - 0.05
        and open_board[0][2] > closed_board[0][2] + 0.05,
        details=f"closed={closed_board}, open={open_board}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
