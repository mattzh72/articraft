from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)
FRAME_HALF_WIDTH = 0.255
FRONT_Y = 0.198
REAR_Y = -0.198
HANDLE_Z = 0.828
SOCKET_CENTER_Z = 0.128
SOCKET_LENGTH = 0.036
WHEEL_CENTER_OFFSET = (0.0, -0.012, -0.086)


def _tube_mesh(
    points: list[tuple[float, float, float]],
    radius: float,
    filename: str,
    *,
    samples_per_segment: int = 18,
    radial_segments: int = 18,
):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples_per_segment,
        radial_segments=radial_segments,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_rolling_walker", assets=ASSETS)

    satin_aluminum = model.material("satin_aluminum", rgba=(0.79, 0.81, 0.83, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    matte_graphite = model.material("matte_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.15, 0.16, 0.17, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.50, 0.52, 0.55, 1.0))

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.48, 0.84)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )

    left_side_points = [
        (-FRAME_HALF_WIDTH, REAR_Y, 0.0),
        (-FRAME_HALF_WIDTH, REAR_Y, 0.44),
        (-FRAME_HALF_WIDTH, -0.155, 0.76),
        (-FRAME_HALF_WIDTH, -0.060, HANDLE_Z),
        (-FRAME_HALF_WIDTH, 0.072, HANDLE_Z),
        (-FRAME_HALF_WIDTH, 0.155, 0.782),
        (-FRAME_HALF_WIDTH, 0.187, 0.50),
        (-FRAME_HALF_WIDTH, FRONT_Y, SOCKET_CENTER_Z + SOCKET_LENGTH / 2.0),
    ]
    right_side_points = [(abs(x), y, z) for x, y, z in left_side_points]

    left_side_mesh = _tube_mesh(left_side_points, 0.0125, "left_side_frame.obj")
    right_side_mesh = _tube_mesh(right_side_points, 0.0125, "right_side_frame.obj")
    left_lower_brace = _tube_mesh(
        [
            (-FRAME_HALF_WIDTH, -0.170, 0.235),
            (-FRAME_HALF_WIDTH, -0.020, 0.255),
            (-FRAME_HALF_WIDTH, 0.120, 0.276),
        ],
        0.009,
        "left_lower_brace.obj",
        samples_per_segment=12,
        radial_segments=16,
    )
    right_lower_brace = _tube_mesh(
        [
            (FRAME_HALF_WIDTH, -0.170, 0.235),
            (FRAME_HALF_WIDTH, -0.020, 0.255),
            (FRAME_HALF_WIDTH, 0.120, 0.276),
        ],
        0.009,
        "right_lower_brace.obj",
        samples_per_segment=12,
        radial_segments=16,
    )

    main_frame.visual(left_side_mesh, material=satin_aluminum, name="left_side_frame")
    main_frame.visual(right_side_mesh, material=satin_aluminum, name="right_side_frame")
    main_frame.visual(left_lower_brace, material=satin_aluminum, name="left_lower_brace")
    main_frame.visual(right_lower_brace, material=satin_aluminum, name="right_lower_brace")

    main_frame.visual(
        Cylinder(radius=0.010, length=0.500),
        origin=Origin(xyz=(0.0, 0.024, 0.775), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="upper_crossbar",
    )
    main_frame.visual(
        Cylinder(radius=0.009, length=0.468),
        origin=Origin(xyz=(0.0, -0.026, 0.535), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="mid_crossbar",
    )
    main_frame.visual(
        Cylinder(radius=0.0085, length=0.470),
        origin=Origin(xyz=(0.0, 0.128, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="front_brace_bar",
    )

    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        x = x_sign * FRAME_HALF_WIDTH
        housing_x = x_sign * 0.231
        button_x = x_sign * 0.258
        main_frame.visual(
            Cylinder(radius=0.018, length=0.182),
            origin=Origin(xyz=(x, -0.004, 0.821), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name=f"{side}_grip_sleeve",
        )
        main_frame.visual(
            Box((0.056, 0.022, 0.030)),
            origin=Origin(xyz=(housing_x, 0.050, 0.775)),
            material=matte_graphite,
            name=f"{side}_lock_housing",
        )
        main_frame.visual(
            Box((0.040, 0.009, 0.018)),
            origin=Origin(
                xyz=(x_sign * 0.239, 0.080, 0.792),
                rpy=(0.0, x_sign * 0.16, 0.0),
            ),
            material=dark_plastic,
            name=f"{side}_lock_paddle",
        )
        main_frame.visual(
            Cylinder(radius=0.0052, length=0.010),
            origin=Origin(xyz=(button_x, 0.050, 0.780), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"{side}_lock_button",
        )
        main_frame.visual(
            Cylinder(radius=0.019, length=SOCKET_LENGTH),
            origin=Origin(xyz=(x, FRONT_Y, SOCKET_CENTER_Z)),
            material=matte_graphite,
            name=f"{side}_socket",
        )
        main_frame.visual(
            Cylinder(radius=0.017, length=0.038),
            origin=Origin(xyz=(x, REAR_Y, 0.019)),
            material=rubber_black,
            name=f"{side}_rear_tip",
        )

    def _build_caster_fork(part_name: str) -> None:
        fork = model.part(part_name)
        fork.inertial = Inertial.from_geometry(
            Box((0.070, 0.040, 0.150)),
            mass=0.28,
            origin=Origin(xyz=(0.0, -0.010, -0.028)),
        )
        fork.visual(
            Cylinder(radius=0.0095, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=satin_steel,
            name="stem",
        )
        fork.visual(
            Cylinder(radius=0.0155, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=matte_graphite,
            name="swivel_body",
        )
        fork.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=matte_graphite,
            name="dust_cap",
        )
        fork.visual(
            Box((0.056, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, -0.010, -0.027)),
            material=matte_graphite,
            name="bridge",
        )
        fork.visual(
            Box((0.010, 0.014, 0.060)),
            origin=Origin(xyz=(-0.021, -0.012, -0.062)),
            material=matte_graphite,
            name="left_blade",
        )
        fork.visual(
            Box((0.010, 0.014, 0.060)),
            origin=Origin(xyz=(0.021, -0.012, -0.062)),
            material=matte_graphite,
            name="right_blade",
        )
        fork.visual(
            Cylinder(radius=0.0045, length=0.044),
            origin=Origin(
                xyz=(0.0, WHEEL_CENTER_OFFSET[1], WHEEL_CENTER_OFFSET[2]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_steel,
            name="axle_pin",
        )

    def _build_front_wheel(part_name: str) -> None:
        wheel = model.part(part_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.042, length=0.018),
            mass=0.17,
            origin=Origin(),
        )
        wheel.visual(
            Cylinder(radius=0.042, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.026, length=0.022),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_gray,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name="left_hub_cap",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name="right_hub_cap",
        )

    _build_caster_fork("left_caster")
    _build_caster_fork("right_caster")
    _build_front_wheel("left_front_wheel")
    _build_front_wheel("right_front_wheel")

    model.articulation(
        "main_to_left_caster",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="left_caster",
        origin=Origin(xyz=(-FRAME_HALF_WIDTH, FRONT_Y, SOCKET_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "main_to_right_caster",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="right_caster",
        origin=Origin(xyz=(FRAME_HALF_WIDTH, FRONT_Y, SOCKET_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent="left_caster",
        child="left_front_wheel",
        origin=Origin(xyz=WHEEL_CENTER_OFFSET),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent="right_caster",
        child="right_front_wheel",
        origin=Origin(xyz=WHEEL_CENTER_OFFSET),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        "main_frame",
        "left_caster",
        reason="left caster kingpin stem nests inside the headset collar",
    )
    ctx.allow_overlap(
        "main_frame",
        "right_caster",
        reason="right caster kingpin stem nests inside the headset collar",
    )
    ctx.allow_overlap(
        "left_caster",
        "left_front_wheel",
        reason="wheel hub is centered on an internal axle pin",
    )
    ctx.allow_overlap(
        "right_caster",
        "right_front_wheel",
        reason="wheel hub is centered on an internal axle pin",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.005, overlap_volume_tol=0.0)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("left_caster", "main_frame")
    ctx.expect_aabb_contact("right_caster", "main_frame")
    ctx.expect_aabb_contact("left_front_wheel", "left_caster")
    ctx.expect_aabb_contact("right_front_wheel", "right_caster")
    ctx.expect_aabb_gap(
        "main_frame",
        "left_front_wheel",
        axis="z",
        min_gap=0.015,
        max_gap=0.090,
        positive_elem="left_socket",
        name="left wheel hangs beneath left socket",
    )
    ctx.expect_aabb_gap(
        "main_frame",
        "right_front_wheel",
        axis="z",
        min_gap=0.015,
        max_gap=0.090,
        positive_elem="right_socket",
        name="right wheel hangs beneath right socket",
    )
    ctx.expect_aabb_overlap("left_front_wheel", "main_frame", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_overlap("right_front_wheel", "main_frame", axes="xy", min_overlap=0.010)

    def _require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    left_caster_pos = ctx.part_world_position("left_caster")
    right_caster_pos = ctx.part_world_position("right_caster")
    left_wheel_pos = ctx.part_world_position("left_front_wheel")
    right_wheel_pos = ctx.part_world_position("right_front_wheel")

    _require(left_caster_pos[0] < -0.20, "left caster should sit outboard to create a stable premium stance")
    _require(right_caster_pos[0] > 0.20, "right caster should sit outboard to create a stable premium stance")
    _require(abs(left_caster_pos[1] - right_caster_pos[1]) < 1e-6, "front caster y positions should stay symmetric")
    _require(abs(left_caster_pos[2] - right_caster_pos[2]) < 1e-6, "front caster kingpins should stay level")
    _require(left_wheel_pos[2] < left_caster_pos[2], "left wheel center must sit below the caster kingpin")
    _require(right_wheel_pos[2] < right_caster_pos[2], "right wheel center must sit below the caster kingpin")
    _require(left_wheel_pos[1] > 0.16, "left front wheel should clearly lead the walker footprint")
    _require(right_wheel_pos[1] > 0.16, "right front wheel should clearly lead the walker footprint")
    _require(0.035 <= left_wheel_pos[2] <= 0.050, "left wheel center height should match a medical caster size")
    _require(0.035 <= right_wheel_pos[2] <= 0.050, "right wheel center height should match a medical caster size")

    with ctx.pose(main_to_left_caster=1.10, main_to_right_caster=-1.10):
        ctx.expect_aabb_contact("left_caster", "main_frame")
        ctx.expect_aabb_contact("right_caster", "main_frame")
        ctx.expect_aabb_contact("left_front_wheel", "left_caster")
        ctx.expect_aabb_contact("right_front_wheel", "right_caster")
        swiveled_left_wheel = ctx.part_world_position("left_front_wheel")
        swiveled_right_wheel = ctx.part_world_position("right_front_wheel")
        _require(
            swiveled_left_wheel[0] > left_wheel_pos[0] + 0.007,
            "left caster swivel should sweep the wheel laterally around the kingpin",
        )
        _require(
            swiveled_right_wheel[0] < right_wheel_pos[0] - 0.007,
            "right caster swivel should sweep the wheel laterally around the kingpin",
        )
        _require(
            swiveled_left_wheel[2] == left_wheel_pos[2] and swiveled_right_wheel[2] == right_wheel_pos[2],
            "caster swivel should preserve wheel ride height",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
