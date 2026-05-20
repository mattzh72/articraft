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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_dish")

    powder_gray = model.material("powder_gray", rgba=(0.69, 0.71, 0.73, 1.0))
    dish_white = model.material("dish_white", rgba=(0.88, 0.89, 0.90, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    graphite = model.material("graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.074, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=graphite,
        name="center_hub",
    )
    base.visual(
        Cylinder(radius=0.110, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=dark_steel,
        name="hub_flange",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=powder_gray,
        name="bearing_housing",
    )
    leg_radius = 0.018
    brace_radius = 0.011
    footprint_radius = 0.440
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        c = math.cos(angle)
        s = math.sin(angle)
        leg = tube_from_spline_points(
            [
                (0.078 * c, 0.078 * s, 0.100),
                (0.220 * c, 0.220 * s, 0.063),
                (footprint_radius * c, footprint_radius * s, 0.018),
            ],
            radius=leg_radius,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
        brace = tube_from_spline_points(
            [
                (0.0, 0.0, 0.055),
                (0.145 * c, 0.145 * s, 0.085),
                (0.270 * c, 0.270 * s, 0.060),
            ],
            radius=brace_radius,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        )
        base.visual(
            _mesh(f"base_leg_{index}", leg),
            material=dark_steel,
            name=f"leg_{index}",
        )
        base.visual(
            _mesh(f"base_brace_{index}", brace),
            material=graphite,
            name=f"brace_{index}",
        )
        base.visual(
            Cylinder(radius=0.040, length=0.012),
            origin=Origin(xyz=(footprint_radius * c, footprint_radius * s, 0.006)),
            material=rubber,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.96, 0.96, 0.28)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.094, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=graphite,
        name="turntable",
    )
    yoke.visual(
        Cylinder(radius=0.040, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.336)),
        material=powder_gray,
        name="center_mast",
    )
    yoke.visual(
        Cylinder(radius=0.068, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material=graphite,
        name="shoulder_collar",
    )
    yoke.visual(
        Box((0.160, 0.400, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        material=powder_gray,
        name="shoulder_block",
    )
    yoke.visual(
        Box((0.080, 0.740, 0.040)),
        origin=Origin(xyz=(0.030, 0.0, 0.740)),
        material=graphite,
        name="arm_cross_tie",
    )

    arm_profile = rounded_rect_profile(0.055, 0.085, radius=0.012, corner_segments=8)
    left_arm = sweep_profile_along_spline(
        [
            (0.010, 0.370, 0.660),
            (0.050, 0.370, 0.748),
            (0.120, 0.370, 0.840),
        ],
        profile=arm_profile,
        samples_per_segment=18,
        cap_profile=True,
    )
    right_arm = sweep_profile_along_spline(
        [
            (0.010, -0.370, 0.660),
            (0.050, -0.370, 0.748),
            (0.120, -0.370, 0.840),
        ],
        profile=arm_profile,
        samples_per_segment=18,
        cap_profile=True,
    )
    yoke.visual(_mesh("left_yoke_arm", left_arm), material=powder_gray, name="left_arm")
    yoke.visual(_mesh("right_yoke_arm", right_arm), material=powder_gray, name="right_arm")
    yoke.visual(
        Cylinder(radius=0.050, length=0.050),
        origin=Origin(xyz=(0.120, 0.365, 0.840), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="left_bearing",
    )
    yoke.visual(
        Cylinder(radius=0.050, length=0.050),
        origin=Origin(xyz=(0.120, -0.365, 0.840), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="right_bearing",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.40, 0.72, 0.92)),
        mass=8.5,
        origin=Origin(xyz=(0.02, 0.0, 0.460)),
    )

    dish_shell_profile = [
        (0.000, -0.030),
        (0.050, -0.028),
        (0.120, -0.018),
        (0.220, 0.010),
        (0.310, 0.055),
        (0.370, 0.095),
        (0.390, 0.112),
        (0.392, 0.118),
        (0.378, 0.118),
        (0.350, 0.104),
        (0.280, 0.066),
        (0.180, 0.022),
        (0.100, -0.004),
        (0.030, -0.012),
        (0.000, -0.016),
    ]
    dish_shell_mesh = _mesh("reflector_shell", LatheGeometry(dish_shell_profile, segments=72))
    outer_ring_mesh = _mesh(
        "rear_outer_ring",
        TorusGeometry(radius=0.235, tube=0.013, radial_segments=18, tubular_segments=64),
    )
    inner_ring_mesh = _mesh(
        "rear_inner_ring",
        TorusGeometry(radius=0.108, tube=0.010, radial_segments=16, tubular_segments=48),
    )
    rib_mesh = _mesh(
        "rear_rib",
        tube_from_spline_points(
            [
                (0.010, 0.0, 0.010),
                (0.050, 0.0, 0.040),
                (0.090, 0.0, 0.120),
                (0.105, 0.0, 0.232),
            ],
            radius=0.009,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    feed_strut_mesh = _mesh(
        "feed_strut",
        tube_from_spline_points(
            [
                (0.070, 0.0, 0.100),
                (0.170, 0.0, 0.055),
                (0.262, 0.0, 0.000),
            ],
            radius=0.006,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        ),
    )

    dish = model.part("dish")
    dish.visual(
        Box((0.120, 0.160, 0.140)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=graphite,
        name="hub_block",
    )
    dish.visual(
        Box((0.070, 0.160, 0.100)),
        origin=Origin(xyz=(0.000, 0.130, 0.0)),
        material=dark_steel,
        name="left_trunnion_support",
    )
    dish.visual(
        Box((0.070, 0.160, 0.100)),
        origin=Origin(xyz=(0.000, -0.130, 0.0)),
        material=dark_steel,
        name="right_trunnion_support",
    )
    dish.visual(
        Cylinder(radius=0.055, length=0.180),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="back_spine",
    )
    dish.visual(
        Cylinder(radius=0.035, length=0.130),
        origin=Origin(xyz=(0.0, 0.275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    dish.visual(
        Cylinder(radius=0.035, length=0.130),
        origin=Origin(xyz=(0.0, -0.275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    dish.visual(
        dish_shell_mesh,
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dish_white,
        name="reflector_shell",
    )
    dish.visual(
        outer_ring_mesh,
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_outer_ring",
    )
    dish.visual(
        inner_ring_mesh,
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_inner_ring",
    )
    for index in range(6):
        angle = (2.0 * math.pi * index) / 6.0
        dish.visual(
            rib_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=graphite,
            name=f"rear_rib_{index}",
        )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        dish.visual(
            feed_strut_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=aluminum,
            name=f"feed_strut_{index}",
        )
    dish.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.250, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="feed_receiver",
    )
    dish.visual(
        Cylinder(radius=0.018, length=0.058),
        origin=Origin(xyz=(0.290, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    dish.inertial = Inertial.from_geometry(
        Box((0.76, 0.68, 0.82)),
        mass=7.2,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=240.0, velocity=1.2),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=dish,
        origin=Origin(xyz=(0.120, 0.0, 0.840)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.8,
            lower=math.radians(-10.0),
            upper=math.radians(75.0),
        ),
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

    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    dish = object_model.get_part("dish")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.expect_contact(
        yoke,
        base,
        elem_a="turntable",
        elem_b="bearing_housing",
        name="turntable seats on bearing housing",
    )
    ctx.expect_contact(
        dish,
        yoke,
        elem_a="left_trunnion",
        elem_b="left_bearing",
        name="left trunnion seats in left bearing",
    )
    ctx.expect_contact(
        dish,
        yoke,
        elem_a="right_trunnion",
        elem_b="right_bearing",
        name="right trunnion seats in right bearing",
    )
    ctx.expect_origin_gap(
        dish,
        base,
        axis="z",
        min_gap=0.75,
        name="dish elevation axis sits well above the tripod base",
    )

    rest_dish_pos = ctx.part_world_position(dish)
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_dish_pos = ctx.part_world_position(dish)
    ctx.check(
        "azimuth rotates the dish around the vertical mast",
        rest_dish_pos is not None
        and turned_dish_pos is not None
        and abs(turned_dish_pos[1]) > 0.08
        and abs(turned_dish_pos[0]) < abs(rest_dish_pos[0]),
        details=f"rest={rest_dish_pos}, turned={turned_dish_pos}",
    )

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_feed = ctx.part_element_world_aabb(dish, elem="feed_horn")
    with ctx.pose({elevation: math.radians(55.0)}):
        raised_feed = ctx.part_element_world_aabb(dish, elem="feed_horn")
    rest_feed_z = _aabb_center_z(rest_feed)
    raised_feed_z = _aabb_center_z(raised_feed)
    ctx.check(
        "positive elevation raises the feed horn",
        rest_feed_z is not None and raised_feed_z is not None and raised_feed_z > rest_feed_z + 0.16,
        details=f"rest_z={rest_feed_z}, raised_z={raised_feed_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
