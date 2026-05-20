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
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_satellite_dish_tripod")

    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.77, 1.0))
    cast_gray = model.material("cast_gray", rgba=(0.40, 0.43, 0.46, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    dish_white = model.material("dish_white", rgba=(0.90, 0.92, 0.93, 1.0))
    lnb_gray = model.material("lnb_gray", rgba=(0.78, 0.80, 0.82, 1.0))

    dish_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.018, 0.000),
                (0.078, 0.006),
                (0.172, 0.026),
                (0.260, 0.056),
                (0.320, 0.095),
            ],
            [
                (0.010, 0.008),
                (0.068, 0.014),
                (0.162, 0.034),
                (0.250, 0.063),
                (0.308, 0.087),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "portable_satellite_dish_shell_v2",
    )
    dish_rim = mesh_from_geometry(
        TorusGeometry(radius=0.314, tube=0.006, radial_segments=16, tubular_segments=72),
        "portable_satellite_dish_rim_v2",
    )
    brace_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.045, 0.000, -0.010),
                (0.105, 0.000, 0.040),
                (0.165, 0.000, 0.120),
                (0.205, 0.000, 0.188),
            ],
            radius=0.0055,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
        "portable_satellite_dish_back_brace_v2",
    )

    feed_arm_geom = tube_from_spline_points(
        [
            (0.055, 0.000, -0.045),
            (0.165, 0.000, -0.062),
            (0.265, 0.000, -0.055),
            (0.345, 0.000, -0.030),
        ],
        radius=0.009,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    feed_arm_geom.merge(
        wire_from_points(
            [
                (0.060, 0.000, -0.010),
                (0.205, 0.000, -0.053),
            ],
            radius=0.006,
            radial_segments=14,
            cap_ends=True,
        )
    )
    feed_arm = mesh_from_geometry(feed_arm_geom, "portable_satellite_feed_arm_v2")

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.085, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=cast_gray,
        name="crown_hub",
    )
    tripod_base.visual(
        Cylinder(radius=0.028, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        material=aluminum,
        name="center_mast",
    )
    tripod_base.visual(
        Cylinder(radius=0.080, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.990)),
        material=cast_gray,
        name="top_plate",
    )
    tripod_base.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=cast_gray,
        name="lower_collar",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        c = math.cos(angle)
        s = math.sin(angle)
        tripod_base.visual(
            mesh_from_geometry(
                wire_from_points(
                    [
                        (0.080 * c, 0.080 * s, 0.775),
                        (0.490 * c, 0.490 * s, 0.028),
                    ],
                    radius=0.018,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"portable_satellite_tripod_leg_v2_{index}",
            ),
            material=aluminum,
            name=f"leg_{index}",
        )
        tripod_base.visual(
            Cylinder(radius=0.035, length=0.024),
            origin=Origin(xyz=(0.490 * c, 0.490 * s, 0.016)),
            material=rubber,
            name=f"foot_{index}",
        )
    tripod_base.inertial = Inertial.from_geometry(
        Box((1.10, 1.10, 1.00)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
    )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.090, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="turntable_base",
    )
    azimuth_head.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=cast_gray,
        name="tilt_pedestal",
    )
    azimuth_head.visual(
        Box((0.120, 0.120, 0.070)),
        origin=Origin(xyz=(-0.010, 0.0, 0.135)),
        material=cast_gray,
        name="gearbox_housing",
    )
    azimuth_head.visual(
        Box((0.065, 0.232, 0.060)),
        origin=Origin(xyz=(-0.025, 0.0, 0.170)),
        material=cast_gray,
        name="yoke_bridge",
    )
    azimuth_head.visual(
        Box((0.035, 0.232, 0.165)),
        origin=Origin(xyz=(-0.040, 0.0, 0.2225)),
        material=cast_gray,
        name="yoke_back_web",
    )
    azimuth_head.visual(
        Box((0.080, 0.018, 0.310)),
        origin=Origin(xyz=(0.000, 0.125, 0.205)),
        material=cast_gray,
        name="left_yoke_plate",
    )
    azimuth_head.visual(
        Box((0.080, 0.018, 0.310)),
        origin=Origin(xyz=(0.000, -0.125, 0.205)),
        material=cast_gray,
        name="right_yoke_plate",
    )
    azimuth_head.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.000, 0.137, 0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_bearing_cap",
    )
    azimuth_head.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.000, -0.137, 0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_bearing_cap",
    )
    azimuth_head.inertial = Inertial.from_geometry(
        Box((0.180, 0.280, 0.420)),
        mass=2.4,
        origin=Origin(xyz=(-0.005, 0.0, 0.180)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        Cylinder(radius=0.012, length=0.232),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    dish_assembly.visual(
        Box((0.115, 0.100, 0.090)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=cast_gray,
        name="hub_block",
    )
    dish_assembly.visual(
        Cylinder(radius=0.026, length=0.100),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_spar",
    )
    dish_assembly.visual(
        dish_shell,
        origin=Origin(xyz=(0.170, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dish_white,
        name="dish_shell",
    )
    dish_assembly.visual(
        dish_rim,
        origin=Origin(xyz=(0.265, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="dish_rim",
    )
    for index in range(4):
        dish_assembly.visual(
            brace_mesh,
            origin=Origin(rpy=(index * (math.pi / 2.0), 0.0, 0.0)),
            material=dark_metal,
            name=f"back_brace_{index}",
        )
    dish_assembly.visual(feed_arm, material=aluminum, name="feed_arm")
    dish_assembly.visual(
        Box((0.032, 0.018, 0.018)),
        origin=Origin(xyz=(0.362, 0.0, -0.028)),
        material=aluminum,
        name="lnb_mount_collar",
    )
    dish_assembly.visual(
        Box((0.050, 0.030, 0.035)),
        origin=Origin(xyz=(0.385, 0.0, -0.028)),
        material=lnb_gray,
        name="lnb_body",
    )
    dish_assembly.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.430, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lnb_gray,
        name="feed_horn",
    )
    dish_assembly.inertial = Inertial.from_geometry(
        Box((0.560, 0.680, 0.320)),
        mass=1.8,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=tripod_base,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=dish_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=math.radians(-10.0),
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))

    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    azimuth_head = object_model.get_part("azimuth_head")
    dish_assembly = object_model.get_part("dish_assembly")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_rotation = object_model.get_articulation("elevation_rotation")

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
        "assembly_parts_present",
        tripod_base.name == "tripod_base"
        and azimuth_head.name == "azimuth_head"
        and dish_assembly.name == "dish_assembly",
        "Expected tripod_base, azimuth_head, and dish_assembly parts.",
    )
    ctx.check(
        "articulations_present",
        azimuth_rotation.name == "azimuth_rotation"
        and elevation_rotation.name == "elevation_rotation",
        "Expected azimuth and elevation articulations.",
    )
    ctx.check(
        "azimuth_axis_is_vertical",
        azimuth_rotation.axis == (0.0, 0.0, 1.0),
        f"Found azimuth axis {azimuth_rotation.axis!r}",
    )
    ctx.check(
        "elevation_axis_is_horizontal",
        elevation_rotation.axis == (0.0, -1.0, 0.0),
        f"Found elevation axis {elevation_rotation.axis!r}",
    )

    ctx.expect_contact(
        azimuth_head,
        tripod_base,
        elem_a="turntable_base",
        elem_b="top_plate",
        name="turntable_seats_on_tripod_plate",
    )
    ctx.expect_contact(
        dish_assembly,
        azimuth_head,
        elem_a="trunnion_shaft",
        elem_b="left_yoke_plate",
        name="left_trunnion_supported_by_yoke",
    )
    ctx.expect_contact(
        dish_assembly,
        azimuth_head,
        elem_a="trunnion_shaft",
        elem_b="right_yoke_plate",
        name="right_trunnion_supported_by_yoke",
    )
    ctx.expect_gap(
        dish_assembly,
        tripod_base,
        axis="z",
        positive_elem="dish_shell",
        min_gap=0.020,
        name="dish_clears_tripod_in_rest_pose",
    )

    feed_rest = ctx.part_element_world_aabb(dish_assembly, elem="feed_horn")
    assert feed_rest is not None
    feed_rest_center = aabb_center(feed_rest)

    with ctx.pose({azimuth_rotation: math.pi / 2.0}):
        feed_az = ctx.part_element_world_aabb(dish_assembly, elem="feed_horn")
        assert feed_az is not None
        feed_az_center = aabb_center(feed_az)
        ctx.check(
            "azimuth_turns_dish_around_vertical_axis",
            feed_az_center[1] > feed_rest_center[1] + 0.25
            and feed_az_center[0] < feed_rest_center[0] - 0.20,
            (
                "Expected feed horn center to swing from +X toward +Y; "
                f"rest={feed_rest_center!r}, turned={feed_az_center!r}"
            ),
        )
        ctx.expect_contact(
            azimuth_head,
            tripod_base,
            elem_a="turntable_base",
            elem_b="top_plate",
            name="turntable_remains_seated_when_azimuthed",
        )

    with ctx.pose({elevation_rotation: math.radians(45.0)}):
        feed_up = ctx.part_element_world_aabb(dish_assembly, elem="feed_horn")
        assert feed_up is not None
        feed_up_center = aabb_center(feed_up)
        ctx.check(
            "elevation_raises_feed_horn",
            feed_up_center[2] > feed_rest_center[2] + 0.18,
            f"Expected feed horn to rise under elevation; rest={feed_rest_center!r}, up={feed_up_center!r}",
        )
        ctx.expect_contact(
            dish_assembly,
            azimuth_head,
            elem_a="trunnion_shaft",
            elem_b="left_yoke_plate",
            name="left_trunnion_stays_supported_when_elevated",
        )
        ctx.expect_contact(
            dish_assembly,
            azimuth_head,
            elem_a="trunnion_shaft",
            elem_b="right_yoke_plate",
            name="right_trunnion_stays_supported_when_elevated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
