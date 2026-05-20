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
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _center_of_aabb(aabb):
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_mount_satellite_dish")

    roof_gray = model.material("roof_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.71, 1.0))
    dish_white = model.material("dish_white", rgba=(0.92, 0.93, 0.94, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    receiver_black = model.material("receiver_black", rgba=(0.10, 0.11, 0.12, 1.0))

    rest_elevation = math.radians(28.0)
    dish_forward = 0.24

    reflector_profile = [
        (0.0, -0.078),
        (0.060, -0.074),
        (0.140, -0.064),
        (0.230, -0.048),
        (0.315, -0.026),
        (0.372, -0.007),
        (0.382, 0.000),
        (0.376, 0.004),
        (0.340, -0.003),
        (0.260, -0.021),
        (0.165, -0.040),
        (0.075, -0.057),
        (0.020, -0.066),
        (0.0, -0.069),
    ]
    reflector_geom = LatheGeometry(reflector_profile, segments=72)
    reflector_geom.rotate_y(math.pi / 2.0)
    reflector_geom.translate(0.072 + dish_forward, 0.0, 0.0)
    reflector_geom.rotate_y(rest_elevation)
    reflector_mesh = _mesh("reflector_shell", reflector_geom)

    rim_geom = TorusGeometry(
        radius=0.378,
        tube=0.0055,
        radial_segments=16,
        tubular_segments=72,
    )
    rim_geom.rotate_y(math.pi / 2.0)
    rim_geom.translate(0.071 + dish_forward, 0.0, 0.0)
    rim_geom.rotate_y(rest_elevation)
    rim_mesh = _mesh("reflector_rim", rim_geom)

    feed_arm_points = [
        (-0.020, 0.0, -0.050),
        (0.008, 0.0, -0.071),
        (0.058, 0.0, -0.072),
        (0.118, 0.0, -0.055),
        (0.176, 0.0, -0.022),
        (0.214, 0.0, 0.000),
    ]
    feed_arm_geom = tube_from_spline_points(
        feed_arm_points,
        radius=0.009,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    feed_arm_geom.translate(dish_forward, 0.0, 0.0)
    feed_arm_geom.rotate_y(rest_elevation)
    feed_arm_mesh = _mesh("feed_arm", feed_arm_geom)

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.340, 0.260, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=roof_gray,
        name="roof_plate",
    )
    roof_mount.visual(
        Box((0.190, 0.140, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=galvanized,
        name="mount_block",
    )
    roof_mount.visual(
        Cylinder(radius=0.056, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=hardware_dark,
        name="azimuth_bearing_housing",
    )
    roof_mount.visual(
        Box((0.070, 0.180, 0.034)),
        origin=Origin(xyz=(-0.060, 0.0, 0.047)),
        material=galvanized,
        name="rear_stiffener",
    )
    roof_mount.inertial = Inertial.from_geometry(
        Box((0.340, 0.260, 0.112)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    pedestal = model.part("pedestal_yoke")
    pedestal.visual(
        Cylinder(radius=0.058, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=hardware_dark,
        name="azimuth_rotor",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=galvanized,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.180, 0.140, 0.050)),
        origin=Origin(xyz=(-0.028, 0.0, 0.320)),
        material=galvanized,
        name="column_head",
    )
    pedestal.visual(
        Box((0.060, 0.182, 0.120)),
        origin=Origin(xyz=(-0.060, 0.0, 0.390)),
        material=galvanized,
        name="yoke_web",
    )
    pedestal.visual(
        Box((0.070, 0.018, 0.220)),
        origin=Origin(xyz=(-0.034, 0.100, 0.505)),
        material=galvanized,
        name="left_cheek",
    )
    pedestal.visual(
        Box((0.070, 0.018, 0.220)),
        origin=Origin(xyz=(-0.034, -0.100, 0.505)),
        material=galvanized,
        name="right_cheek",
    )
    pedestal.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(xyz=(-0.004, 0.090, 0.505), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="left_trunnion_boss",
    )
    pedestal.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(xyz=(-0.004, -0.090, 0.505), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="right_trunnion_boss",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.180, 0.220, 0.625)),
        mass=8.5,
        origin=Origin(xyz=(-0.005, 0.0, 0.3125)),
    )

    dish = model.part("dish_assembly")
    dish.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="trunnion_shaft",
    )

    rear_frame_spine_center = _rotate_y((0.030, 0.0, -0.004), rest_elevation)
    dish.visual(
        Box((0.150, 0.056, 0.050)),
        origin=Origin(xyz=rear_frame_spine_center, rpy=(0.0, rest_elevation, 0.0)),
        material=hardware_dark,
        name="rear_frame_spine",
    )

    rear_frame_cradle_center = _rotate_y((0.155, 0.0, -0.042), rest_elevation)
    dish.visual(
        Box((0.220, 0.074, 0.028)),
        origin=Origin(xyz=rear_frame_cradle_center, rpy=(0.0, rest_elevation, 0.0)),
        material=hardware_dark,
        name="rear_frame_cradle",
    )

    back_hub_center = _rotate_y((0.118, 0.0, 0.0), rest_elevation)
    dish.visual(
        Cylinder(radius=0.050, length=0.034),
        origin=Origin(
            xyz=back_hub_center,
            rpy=(0.0, (math.pi / 2.0) + rest_elevation, 0.0),
        ),
        material=galvanized,
        name="back_hub",
    )

    dish.visual(reflector_mesh, material=dish_white, name="reflector")
    dish.visual(rim_mesh, material=galvanized, name="reflector_rim")
    dish.visual(feed_arm_mesh, material=galvanized, name="feed_arm")

    feed_mount_center = _rotate_y((dish_forward + 0.213, 0.0, 0.0), rest_elevation)
    dish.visual(
        Box((0.022, 0.028, 0.028)),
        origin=Origin(xyz=feed_mount_center, rpy=(0.0, rest_elevation, 0.0)),
        material=galvanized,
        name="feed_mount",
    )

    feed_horn_center = _rotate_y((dish_forward + 0.240, 0.0, 0.0), rest_elevation)
    dish.visual(
        Cylinder(radius=0.016, length=0.052),
        origin=Origin(
            xyz=feed_horn_center,
            rpy=(0.0, (math.pi / 2.0) + rest_elevation, 0.0),
        ),
        material=receiver_black,
        name="feed_horn",
    )

    lnb_center = _rotate_y((dish_forward + 0.274, 0.0, 0.0), rest_elevation)
    dish.visual(
        Box((0.036, 0.024, 0.022)),
        origin=Origin(xyz=lnb_center, rpy=(0.0, rest_elevation, 0.0)),
        material=receiver_black,
        name="lnb_box",
    )
    dish.inertial = Inertial.from_geometry(
        Box((1.020, 0.760, 0.360)),
        mass=4.2,
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=roof_mount,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2),
    )
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.9,
            lower=math.radians(-8.0),
            upper=math.radians(42.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    pedestal = object_model.get_part("pedestal_yoke")
    dish = object_model.get_part("dish_assembly")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        roof_mount,
        pedestal,
        elem_a="azimuth_bearing_housing",
        elem_b="azimuth_rotor",
    )
    ctx.expect_contact(
        pedestal,
        dish,
        elem_a="left_trunnion_boss",
        elem_b="trunnion_shaft",
    )
    ctx.expect_contact(
        pedestal,
        dish,
        elem_a="right_trunnion_boss",
        elem_b="trunnion_shaft",
    )
    ctx.expect_gap(dish, roof_mount, axis="z", min_gap=0.015)

    left_cheek = ctx.part_element_world_aabb(pedestal, elem="left_cheek")
    right_cheek = ctx.part_element_world_aabb(pedestal, elem="right_cheek")
    rear_spine = ctx.part_element_world_aabb(dish, elem="rear_frame_spine")
    horn_rest = ctx.part_element_world_aabb(dish, elem="feed_horn")
    assert left_cheek is not None
    assert right_cheek is not None
    assert rear_spine is not None
    assert horn_rest is not None
    assert rear_spine[1][1] < left_cheek[0][1]
    assert rear_spine[0][1] > right_cheek[1][1]

    horn_rest_center = _center_of_aabb(horn_rest)

    with ctx.pose({azimuth: math.pi / 2.0}):
        horn_azimuth = ctx.part_element_world_aabb(dish, elem="feed_horn")
        assert horn_azimuth is not None
        horn_azimuth_center = _center_of_aabb(horn_azimuth)
        assert abs(horn_azimuth_center[0]) < 0.07
        assert horn_azimuth_center[1] > 0.18

    with ctx.pose({elevation: math.radians(30.0)}):
        horn_high = ctx.part_element_world_aabb(dish, elem="feed_horn")
        left_cheek_high = ctx.part_element_world_aabb(pedestal, elem="left_cheek")
        right_cheek_high = ctx.part_element_world_aabb(pedestal, elem="right_cheek")
        rear_spine_high = ctx.part_element_world_aabb(dish, elem="rear_frame_spine")
        assert horn_high is not None
        assert left_cheek_high is not None
        assert right_cheek_high is not None
        assert rear_spine_high is not None
        ctx.expect_contact(
            pedestal,
            dish,
            elem_a="left_trunnion_boss",
            elem_b="trunnion_shaft",
        )
        ctx.expect_contact(
            pedestal,
            dish,
            elem_a="right_trunnion_boss",
            elem_b="trunnion_shaft",
        )
        ctx.expect_gap(dish, roof_mount, axis="z", min_gap=0.015)
        assert _center_of_aabb(horn_high)[2] > horn_rest_center[2] + 0.08
        assert rear_spine_high[1][1] < left_cheek_high[0][1]
        assert rear_spine_high[0][1] > right_cheek_high[1][1]

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
