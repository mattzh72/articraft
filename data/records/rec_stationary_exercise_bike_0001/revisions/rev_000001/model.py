from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir


def _mesh(name: str, geometry) -> object:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, MESH_DIR / name)


def _closed_profile(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    sampled = sample_catmull_rom_spline_2d(
        points,
        samples_per_segment=12,
        closed=True,
    )
    if sampled and sampled[0] == sampled[-1]:
        return sampled[:-1]
    return sampled


def _housing_mesh(name: str) -> object:
    profile = _closed_profile(
        [
            (-0.18, -0.18),
            (-0.07, -0.21),
            (0.07, -0.18),
            (0.19, -0.08),
            (0.22, 0.06),
            (0.12, 0.19),
            (-0.04, 0.22),
            (-0.17, 0.10),
        ]
    )
    return _mesh(
        name,
        ExtrudeGeometry(
            profile,
            height=0.016,
            cap=True,
            center=True,
            closed=True,
        ),
    )


def _saddle_mesh(name: str) -> object:
    sections = [
        (-0.12, -0.016, 0.020, 0.180),
        (-0.06, -0.018, 0.021, 0.170),
        (0.00, -0.015, 0.020, 0.145),
        (0.08, -0.011, 0.016, 0.095),
        (0.15, -0.005, 0.010, 0.040),
    ]
    return _mesh(
        name,
        superellipse_side_loft(
            sections,
            exponents=[3.0, 3.0, 2.7, 2.4, 2.1],
            segments=52,
            cap=True,
            closed=True,
        ),
    )


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    radius: float,
    samples_per_segment: int = 16,
    radial_segments: int = 20,
) -> object:
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
    )


def _pedal_visuals(part, matte_black, rubber, steel) -> None:
    part.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=steel,
    )
    part.visual(
        Box((0.100, 0.028, 0.015)),
        origin=Origin(),
        material=matte_black,
    )
    part.visual(
        Box((0.084, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=rubber,
    )
    part.visual(
        Box((0.012, 0.030, 0.030)),
        origin=Origin(xyz=(0.046, 0.0, 0.008)),
        material=matte_black,
    )
    part.inertial = Inertial.from_geometry(
        Box((0.100, 0.030, 0.030)),
        mass=0.45,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_exercise_bike", assets=ASSETS)

    frame_black = model.material("frame_black", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.70, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.37, 0.40, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_red = model.material("accent_red", rgba=(0.78, 0.10, 0.10, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.58, 0.75, 0.88, 0.42))

    base = model.part("base")
    base.visual(
        Box((0.48, 0.10, 0.05)),
        origin=Origin(xyz=(-0.42, 0.0, 0.025)),
        material=graphite,
    )
    base.visual(
        Box((0.40, 0.09, 0.045)),
        origin=Origin(xyz=(0.46, 0.0, 0.0225)),
        material=graphite,
    )
    base.visual(
        _tube_mesh(
            "frame_main_rail.obj",
            [
                (-0.44, 0.0, 0.060),
                (-0.16, 0.0, 0.080),
                (0.02, 0.0, 0.160),
                (0.12, 0.0, 0.245),
            ],
            radius=0.028,
        ),
        material=frame_black,
    )
    base.visual(
        _tube_mesh(
            "frame_seat_support.obj",
            [
                (-0.32, 0.0, 0.052),
                (-0.24, 0.0, 0.330),
                (-0.16, 0.0, 0.540),
            ],
            radius=0.026,
        ),
        material=frame_black,
    )
    base.visual(
        _tube_mesh(
            "frame_top_tube.obj",
            [
                (-0.16, 0.0, 0.580),
                (-0.02, 0.0, 0.500),
                (0.12, 0.0, 0.300),
            ],
            radius=0.022,
        ),
        material=frame_black,
    )
    base.visual(
        _tube_mesh(
            "frame_front_support.obj",
            [
                (0.16, 0.0, 0.320),
                (0.34, 0.0, 0.560),
                (0.50, 0.0, 0.800),
            ],
            radius=0.027,
        ),
        material=frame_black,
    )
    base.visual(
        _tube_mesh(
            "frame_lower_front.obj",
            [
                (0.08, 0.0, 0.175),
                (0.24, 0.0, 0.145),
                (0.46, 0.0, 0.050),
            ],
            radius=0.022,
        ),
        material=frame_black,
    )
    base.visual(
        Box((0.22, 0.12, 0.12)),
        origin=Origin(xyz=(0.11, 0.0, 0.200)),
        material=graphite,
    )
    base.visual(
        Box((0.10, 0.14, 0.11)),
        origin=Origin(xyz=(-0.02, 0.0, 0.270)),
        material=graphite,
    )
    base.visual(
        Box((0.075, 0.055, 0.24)),
        origin=Origin(xyz=(-0.16, 0.0, 0.640)),
        material=graphite,
    )
    base.visual(
        Box((0.080, 0.060, 0.26)),
        origin=Origin(xyz=(0.50, 0.0, 0.770)),
        material=graphite,
    )
    base.visual(
        _housing_mesh("housing_left.obj"),
        origin=Origin(xyz=(0.24, 0.056, 0.430), rpy=(math.pi / 2, 0.0, 0.0)),
        material=graphite,
    )
    base.visual(
        _housing_mesh("housing_right.obj"),
        origin=Origin(xyz=(0.24, -0.056, 0.430), rpy=(math.pi / 2, 0.0, 0.0)),
        material=graphite,
    )
    base.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.62, 0.055, 0.040), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rubber,
    )
    base.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.62, -0.055, 0.040), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rubber,
    )
    base.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(-0.16, 0.042, 0.620), rpy=(math.pi / 2, 0.0, 0.0)),
        material=accent_red,
    )
    base.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.50, 0.045, 0.750), rpy=(math.pi / 2, 0.0, 0.0)),
        material=accent_red,
    )
    base.inertial = Inertial.from_geometry(
        Box((1.28, 0.28, 0.98)),
        mass=58.0,
        origin=Origin(xyz=(0.02, 0.0, 0.49)),
    )

    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.185, length=0.060),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_steel,
    )
    flywheel.visual(
        Cylinder(radius=0.174, length=0.050),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=accent_red,
    )
    flywheel.visual(
        Cylinder(radius=0.122, length=0.074),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=charcoal,
    )
    flywheel.visual(
        Cylinder(radius=0.045, length=0.112),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed_aluminum,
    )
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.185, length=0.060),
        mass=13.5,
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.022, length=0.220),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed_aluminum,
    )
    crank.visual(
        Cylinder(radius=0.085, length=0.012),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=charcoal,
    )
    crank.visual(
        Box((0.180, 0.022, 0.036)),
        origin=Origin(xyz=(0.090, 0.105, 0.0)),
        material=frame_black,
    )
    crank.visual(
        Box((0.180, 0.022, 0.036)),
        origin=Origin(xyz=(-0.090, -0.105, 0.0)),
        material=frame_black,
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.170, 0.105, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed_aluminum,
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(-0.170, -0.105, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed_aluminum,
    )
    crank.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.240),
        mass=4.0,
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    _pedal_visuals(left_pedal, frame_black, rubber, steel)

    right_pedal = model.part("right_pedal")
    _pedal_visuals(right_pedal, frame_black, rubber, steel)

    seat_post = model.part("seat_post")
    seat_post.visual(
        Box((0.050, 0.040, 0.340)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=steel,
    )
    seat_post.visual(
        Box((0.100, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=graphite,
    )
    seat_post.visual(
        Cylinder(radius=0.016, length=0.055),
        origin=Origin(xyz=(-0.028, 0.040, 0.120), rpy=(math.pi / 2, 0.0, 0.0)),
        material=accent_red,
    )
    seat_post.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.360)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    seat_slider = model.part("seat_slider")
    seat_slider.visual(
        Box((0.180, 0.050, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.015)),
        material=steel,
    )
    seat_slider.visual(
        Box((0.090, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=graphite,
    )
    seat_slider.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(xyz=(0.020, 0.018, 0.043), rpy=(0.0, math.pi / 2, 0.0)),
        material=brushed_aluminum,
    )
    seat_slider.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(xyz=(0.020, -0.018, 0.043), rpy=(0.0, math.pi / 2, 0.0)),
        material=brushed_aluminum,
    )
    seat_slider.visual(
        _saddle_mesh("saddle.obj"),
        origin=Origin(xyz=(0.045, 0.0, 0.066), rpy=(0.0, -0.10, -math.pi / 2)),
        material=saddle_black,
    )
    seat_slider.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(-0.040, -0.034, 0.018), rpy=(math.pi / 2, 0.0, 0.0)),
        material=accent_red,
    )
    seat_slider.inertial = Inertial.from_geometry(
        Box((0.280, 0.180, 0.120)),
        mass=3.4,
        origin=Origin(xyz=(0.040, 0.0, 0.060)),
    )

    handlebar_post = model.part("handlebar_post")
    handlebar_post.visual(
        Box((0.050, 0.040, 0.300)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=steel,
    )
    handlebar_post.visual(
        Box((0.110, 0.065, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=graphite,
    )
    handlebar_post.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(-0.022, 0.045, 0.120), rpy=(math.pi / 2, 0.0, 0.0)),
        material=accent_red,
    )
    handlebar_post.inertial = Inertial.from_geometry(
        Box((0.100, 0.080, 0.320)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    handlebar_slider = model.part("handlebar_slider")
    handlebar_slider.visual(
        Box((0.150, 0.060, 0.040)),
        origin=Origin(xyz=(0.045, 0.0, 0.020)),
        material=steel,
    )
    handlebar_slider.visual(
        Box((0.070, 0.120, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=graphite,
    )
    handlebar_slider.visual(
        _tube_mesh(
            "handlebar_left.obj",
            [
                (-0.010, 0.036, 0.050),
                (0.040, 0.100, 0.130),
                (0.130, 0.185, 0.205),
                (0.245, 0.235, 0.155),
            ],
            radius=0.014,
            samples_per_segment=18,
            radial_segments=22,
        ),
        material=frame_black,
    )
    handlebar_slider.visual(
        _tube_mesh(
            "handlebar_right.obj",
            [
                (-0.010, -0.036, 0.050),
                (0.040, -0.100, 0.130),
                (0.130, -0.185, 0.205),
                (0.245, -0.235, 0.155),
            ],
            radius=0.014,
            samples_per_segment=18,
            radial_segments=22,
        ),
        material=frame_black,
    )
    handlebar_slider.visual(
        _tube_mesh(
            "handlebar_crossbar.obj",
            [
                (0.050, -0.120, 0.120),
                (0.090, 0.000, 0.175),
                (0.050, 0.120, 0.120),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
        ),
        material=frame_black,
    )
    handlebar_slider.visual(
        Box((0.050, 0.030, 0.100)),
        origin=Origin(xyz=(0.135, 0.0, 0.155)),
        material=charcoal,
    )
    handlebar_slider.visual(
        Box((0.090, 0.050, 0.120)),
        origin=Origin(xyz=(0.190, 0.0, 0.205)),
        material=graphite,
    )
    handlebar_slider.visual(
        Box((0.070, 0.003, 0.090)),
        origin=Origin(xyz=(0.208, 0.0, 0.225), rpy=(0.0, -0.40, 0.0)),
        material=screen_glass,
    )
    handlebar_slider.inertial = Inertial.from_geometry(
        Box((0.380, 0.520, 0.250)),
        mass=4.4,
        origin=Origin(xyz=(0.110, 0.0, 0.140)),
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="flywheel",
        origin=Origin(xyz=(0.24, 0.0, 0.43)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=25.0,
        ),
    )
    model.articulation(
        "crank_rotation",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="crank",
        origin=Origin(xyz=(-0.02, 0.0, 0.27)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crank",
        child="left_pedal",
        origin=Origin(xyz=(-0.17, -0.105, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=30.0,
        ),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crank",
        child="right_pedal",
        origin=Origin(xyz=(0.17, 0.105, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=30.0,
        ),
    )
    model.articulation(
        "seat_height_adjust",
        ArticulationType.PRISMATIC,
        parent="base",
        child="seat_post",
        origin=Origin(xyz=(-0.16, 0.0, 0.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.50,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "seat_fore_aft_adjust",
        ArticulationType.PRISMATIC,
        parent="seat_post",
        child="seat_slider",
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.40,
            lower=-0.05,
            upper=0.05,
        ),
    )
    model.articulation(
        "handlebar_height_adjust",
        ArticulationType.PRISMATIC,
        parent="base",
        child="handlebar_post",
        origin=Origin(xyz=(0.50, 0.0, 0.64)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.50,
            lower=0.0,
            upper=0.16,
        ),
    )
    model.articulation(
        "handlebar_reach_adjust",
        ArticulationType.PRISMATIC,
        parent="handlebar_post",
        child="handlebar_slider",
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.40,
            lower=-0.03,
            upper=0.08,
        ),
    )

    return model


def _expect(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _position(ctx: TestContext, part_name: str) -> tuple[float, float, float]:
    x, y, z = ctx.part_world_position(part_name)
    return float(x), float(y), float(z)


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("seat_post", "base")
    ctx.expect_aabb_contact("seat_slider", "seat_post")
    ctx.expect_aabb_contact("handlebar_post", "base")
    ctx.expect_aabb_contact("handlebar_slider", "handlebar_post")
    ctx.expect_aabb_contact("crank", "base")
    ctx.expect_aabb_contact("left_pedal", "crank")
    ctx.expect_aabb_contact("right_pedal", "crank")
    ctx.expect_aabb_overlap("seat_post", "base", axes="xy", min_overlap=0.025)
    ctx.expect_aabb_overlap("seat_slider", "seat_post", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("handlebar_post", "base", axes="xy", min_overlap=0.025)
    ctx.expect_aabb_overlap("handlebar_slider", "handlebar_post", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_gap("seat_post", "base", axis="z", max_gap=0.001, max_penetration=0.400)
    ctx.expect_aabb_gap("handlebar_post", "base", axis="z", max_gap=0.001, max_penetration=0.280)

    ctx.expect_joint_motion_axis(
        "seat_height_adjust",
        "seat_post",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "seat_fore_aft_adjust",
        "seat_slider",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "handlebar_height_adjust",
        "handlebar_post",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "handlebar_reach_adjust",
        "handlebar_slider",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )

    right_x, right_y, right_z = _position(ctx, "right_pedal")
    left_x, left_y, left_z = _position(ctx, "left_pedal")
    flywheel_x, _, flywheel_z = _position(ctx, "flywheel")
    saddle_x, _, saddle_z = _position(ctx, "seat_slider")
    bar_x, _, bar_z = _position(ctx, "handlebar_slider")
    crank_x, _, crank_z = _position(ctx, "crank")
    _expect(right_y > 0.09, "right pedal should sit on the rider's right side")
    _expect(left_y < -0.09, "left pedal should sit on the rider's left side")
    _expect(right_x > left_x + 0.28, "default crank pose should place pedals fore-aft opposite")
    _expect(
        abs(right_z - left_z) < 0.03, "default crank pose should keep pedal heights nearly level"
    )
    _expect(flywheel_x > crank_x + 0.20, "flywheel should sit well ahead of the crank axle")
    _expect(flywheel_z > crank_z + 0.10, "flywheel should sit above the crank axle")
    _expect(saddle_z > crank_z + 0.55, "saddle should ride well above the crank")
    _expect(bar_x > saddle_x + 0.55, "handlebars should sit clearly ahead of the saddle")
    _expect(bar_z > saddle_z - 0.05, "handlebars should not sit far below the saddle")

    with ctx.pose(seat_height_adjust=0.18, seat_fore_aft_adjust=0.05):
        ctx.expect_aabb_contact("seat_post", "base")
        ctx.expect_aabb_contact("seat_slider", "seat_post")
        ctx.expect_aabb_overlap("seat_slider", "seat_post", axes="xy", min_overlap=0.015)
        seat_x, _, seat_z = _position(ctx, "seat_slider")
        crank_x, _, crank_z = _position(ctx, "crank")
        _expect(seat_z > crank_z + 0.70, "raised saddle should stay clearly above the drivetrain")
        _expect(
            seat_x < crank_x + 0.08, "saddle should remain near the rider envelope at max fore-aft"
        )

    with ctx.pose(handlebar_height_adjust=0.16, handlebar_reach_adjust=0.08):
        ctx.expect_aabb_contact("handlebar_post", "base")
        ctx.expect_aabb_contact("handlebar_slider", "handlebar_post")
        ctx.expect_aabb_overlap("handlebar_slider", "handlebar_post", axes="xy", min_overlap=0.008)
        handle_x, _, handle_z = _position(ctx, "handlebar_slider")
        seat_x, _, seat_z = _position(ctx, "seat_slider")
        _expect(
            handle_x > seat_x + 0.58, "forward handlebar adjustment should preserve cockpit length"
        )
        _expect(handle_z > seat_z - 0.02, "raised bars should remain at least saddle height")

    with ctx.pose(crank_rotation=math.pi / 2):
        right_x, _, right_z = _position(ctx, "right_pedal")
        left_x, _, left_z = _position(ctx, "left_pedal")
        _expect(right_z > left_z + 0.28, "positive crank rotation should lift the right pedal")
        _expect(left_z > 0.09, "lower pedal should still clear the floor comfortably")
        _expect(
            abs(right_x - left_x) < 0.04,
            "quarter-turn crank pose should align pedals nearly vertically",
        )

    with ctx.pose(crank_rotation=math.pi):
        right_x, _, right_z = _position(ctx, "right_pedal")
        left_x, _, left_z = _position(ctx, "left_pedal")
        _expect(
            left_x > right_x + 0.28, "half-turn crank pose should swap the fore-aft pedal ordering"
        )
        _expect(abs(right_z - left_z) < 0.03, "half-turn crank pose should bring pedals back level")

    with ctx.pose(crank_rotation=math.pi / 2, left_pedal_spin=1.2, right_pedal_spin=-1.0):
        ctx.expect_aabb_contact("left_pedal", "crank")
        ctx.expect_aabb_contact("right_pedal", "crank")

    with ctx.pose(flywheel_spin=math.pi * 0.75):
        flywheel_x, _, flywheel_z = _position(ctx, "flywheel")
        crank_x, _, crank_z = _position(ctx, "crank")
        _expect(
            flywheel_x > crank_x + 0.20, "flywheel should stay ahead of the crank through rotation"
        )
        _expect(flywheel_z > crank_z + 0.10, "flywheel height should remain fixed through rotation")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
