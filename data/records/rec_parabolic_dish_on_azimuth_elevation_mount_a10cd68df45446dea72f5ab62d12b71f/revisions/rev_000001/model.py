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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tracking_dish")

    base_dark = model.material("base_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.43, 0.46, 0.49, 1.0))
    dish_white = model.material("dish_white", rgba=(0.86, 0.88, 0.90, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))
    warning_gray = model.material("warning_gray", rgba=(0.56, 0.57, 0.55, 1.0))

    reflector_outer = [
        (0.025, 0.00),
        (0.14, 0.010),
        (0.34, 0.034),
        (0.56, 0.078),
        (0.75, 0.145),
        (0.86, 0.205),
        (0.90, 0.238),
    ]
    reflector_inner = [
        (0.012, 0.018),
        (0.12, 0.026),
        (0.30, 0.048),
        (0.51, 0.088),
        (0.69, 0.149),
        (0.80, 0.204),
        (0.86, 0.228),
    ]
    reflector_mesh = _mesh(
        "tracking_reflector_shell",
        LatheGeometry.from_shell_profiles(
            reflector_outer,
            reflector_inner,
            segments=72,
            end_cap="flat",
            lip_samples=8,
        ),
    )

    feed_strut_mesh = _mesh(
        "tracking_feed_strut",
        tube_from_spline_points(
            [
                (0.44, 0.0, 0.87),
                (0.56, 0.0, 0.72),
                (0.72, 0.0, 0.44),
                (0.90, 0.0, 0.16),
                (1.03, 0.0, 0.0),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    base = model.part("base")
    base.visual(
        Box((1.40, 1.20, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=base_dark,
        name="ballast_block",
    )
    base.visual(
        Box((1.05, 0.92, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=machinery_gray,
        name="upper_plinth",
    )
    base.visual(
        Cylinder(radius=0.32, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=charcoal,
        name="stationary_azimuth_bearing",
    )
    base.inertial = Inertial.from_geometry(
        Box((1.40, 1.20, 0.58)),
        mass=680.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    azimuth_pedestal = model.part("azimuth_pedestal")
    azimuth_pedestal.visual(
        Cylinder(radius=0.30, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=charcoal,
        name="turntable_drum",
    )
    azimuth_pedestal.visual(
        Cylinder(radius=0.38, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=machinery_gray,
        name="rotating_deck",
    )
    azimuth_pedestal.visual(
        Cylinder(radius=0.17, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=machinery_gray,
        name="pedestal_column",
    )
    azimuth_pedestal.visual(
        Box((0.34, 0.68, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=machinery_gray,
        name="cradle_saddle",
    )
    azimuth_pedestal.visual(
        Box((0.18, 0.10, 0.50)),
        origin=Origin(xyz=(0.0, 0.34, 0.84)),
        material=dish_white,
        name="left_cradle_cheek",
    )
    azimuth_pedestal.visual(
        Box((0.18, 0.10, 0.50)),
        origin=Origin(xyz=(0.0, -0.34, 0.84)),
        material=dish_white,
        name="right_cradle_cheek",
    )
    azimuth_pedestal.visual(
        Cylinder(radius=0.08, length=0.18),
        origin=Origin(xyz=(0.0, 0.32, 1.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_elevation_bearing",
    )
    azimuth_pedestal.visual(
        Cylinder(radius=0.08, length=0.18),
        origin=Origin(xyz=(0.0, -0.32, 1.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_elevation_bearing",
    )
    azimuth_pedestal.inertial = Inertial.from_geometry(
        Box((0.76, 0.76, 1.18)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        Cylinder(radius=0.06, length=0.46),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_shaft",
    )
    dish_assembly.visual(
        Box((0.26, 0.36, 0.30)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        material=machinery_gray,
        name="hub_block",
    )
    dish_assembly.visual(
        Cylinder(radius=0.16, length=0.34),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warning_gray,
        name="hub_barrel",
    )
    dish_assembly.visual(
        Cylinder(radius=0.20, length=0.08),
        origin=Origin(xyz=(0.33, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="reflector_mount_collar",
    )
    dish_assembly.visual(
        reflector_mesh,
        origin=Origin(xyz=(0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dish_white,
        name="reflector_shell",
    )
    dish_assembly.visual(
        Box((0.06, 0.82, 0.08)),
        origin=Origin(xyz=(-0.24, 0.0, 0.33)),
        material=machinery_gray,
        name="rear_frame_top",
    )
    dish_assembly.visual(
        Box((0.06, 0.82, 0.08)),
        origin=Origin(xyz=(-0.24, 0.0, -0.33)),
        material=machinery_gray,
        name="rear_frame_bottom",
    )
    dish_assembly.visual(
        Box((0.06, 0.08, 0.74)),
        origin=Origin(xyz=(-0.24, 0.37, 0.0)),
        material=machinery_gray,
        name="rear_frame_left",
    )
    dish_assembly.visual(
        Box((0.06, 0.08, 0.74)),
        origin=Origin(xyz=(-0.24, -0.37, 0.0)),
        material=machinery_gray,
        name="rear_frame_right",
    )
    dish_assembly.visual(
        Box((0.06, 0.08, 0.66)),
        origin=Origin(xyz=(-0.24, 0.0, 0.0)),
        material=machinery_gray,
        name="rear_frame_upright",
    )
    dish_assembly.visual(
        Box((0.06, 0.74, 0.08)),
        origin=Origin(xyz=(-0.24, 0.0, 0.0)),
        material=machinery_gray,
        name="rear_frame_crossbar",
    )
    dish_assembly.visual(
        Box((0.32, 0.06, 0.06)),
        origin=Origin(xyz=(-0.11, 0.0, 0.18)),
        material=steel,
        name="upper_back_brace",
    )
    dish_assembly.visual(
        Box((0.32, 0.06, 0.06)),
        origin=Origin(xyz=(-0.11, 0.0, -0.18)),
        material=steel,
        name="lower_back_brace",
    )
    dish_assembly.visual(
        Box((0.32, 0.06, 0.06)),
        origin=Origin(xyz=(-0.11, 0.20, 0.0)),
        material=steel,
        name="left_back_brace",
    )
    dish_assembly.visual(
        Box((0.32, 0.06, 0.06)),
        origin=Origin(xyz=(-0.11, -0.20, 0.0)),
        material=steel,
        name="right_back_brace",
    )
    dish_assembly.visual(
        Box((0.05, 0.24, 0.03)),
        origin=Origin(xyz=(-0.255, 0.24, 0.045)),
        material=charcoal,
        name="electronics_bay_top",
    )
    dish_assembly.visual(
        Box((0.05, 0.24, 0.03)),
        origin=Origin(xyz=(-0.255, 0.24, -0.175)),
        material=charcoal,
        name="electronics_bay_bottom",
    )
    dish_assembly.visual(
        Box((0.05, 0.025, 0.22)),
        origin=Origin(xyz=(-0.255, 0.12, -0.065)),
        material=charcoal,
        name="electronics_bay_left",
    )
    dish_assembly.visual(
        Box((0.05, 0.025, 0.22)),
        origin=Origin(xyz=(-0.255, 0.36, -0.065)),
        material=charcoal,
        name="electronics_bay_right",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        dish_assembly.visual(
            feed_strut_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=steel,
            name=f"feed_strut_{index + 1}",
        )
    dish_assembly.visual(
        Box((0.12, 0.12, 0.10)),
        origin=Origin(xyz=(1.03, 0.0, 0.0)),
        material=charcoal,
        name="feed_support_hub",
    )
    dish_assembly.visual(
        Cylinder(radius=0.04, length=0.18),
        origin=Origin(xyz=(1.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="feed_horn",
    )
    dish_assembly.inertial = Inertial.from_geometry(
        Box((1.54, 1.90, 1.90)),
        mass=160.0,
        origin=Origin(xyz=(0.27, 0.0, 0.0)),
    )

    rear_cover = model.part("rear_cover")
    rear_cover.visual(
        Box((0.012, 0.18, 0.16)),
        origin=Origin(xyz=(0.007, 0.0, -0.08)),
        material=warning_gray,
        name="cover_leaf",
    )
    rear_cover.visual(
        Cylinder(radius=0.006, length=0.05),
        origin=Origin(xyz=(0.0, -0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_knuckle_left",
    )
    rear_cover.visual(
        Cylinder(radius=0.006, length=0.05),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_knuckle_center",
    )
    rear_cover.visual(
        Cylinder(radius=0.006, length=0.05),
        origin=Origin(xyz=(0.0, 0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_knuckle_right",
    )
    rear_cover.visual(
        Box((0.018, 0.04, 0.016)),
        origin=Origin(xyz=(0.013, 0.0, -0.085)),
        material=charcoal,
        name="cover_pull",
    )
    rear_cover.inertial = Inertial.from_geometry(
        Box((0.024, 0.18, 0.17)),
        mass=1.2,
        origin=Origin(xyz=(0.008, 0.0, -0.08)),
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=azimuth_pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=1.1),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=azimuth_pedestal,
        child=dish_assembly,
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.7,
            lower=-0.30,
            upper=1.35,
        ),
    )
    model.articulation(
        "rear_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=dish_assembly,
        child=rear_cover,
        origin=Origin(xyz=(-0.283, 0.24, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
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
    azimuth_pedestal = object_model.get_part("azimuth_pedestal")
    dish_assembly = object_model.get_part("dish_assembly")
    rear_cover = object_model.get_part("rear_cover")

    azimuth_axis = object_model.get_articulation("azimuth_axis")
    elevation_axis = object_model.get_articulation("elevation_axis")
    rear_cover_hinge = object_model.get_articulation("rear_cover_hinge")

    ctx.check(
        "tracking dish parts and articulations exist",
        all(
            item is not None
            for item in (
                base,
                azimuth_pedestal,
                dish_assembly,
                rear_cover,
                azimuth_axis,
                elevation_axis,
                rear_cover_hinge,
            )
        ),
        details="One or more named parts or articulations could not be resolved.",
    )

    with ctx.pose({azimuth_axis: 0.0, elevation_axis: 0.0, rear_cover_hinge: 0.0}):
        feed_horn_rest = _aabb_center(ctx.part_element_world_aabb(dish_assembly, elem="feed_horn"))
        cover_leaf_rest_aabb = ctx.part_element_world_aabb(rear_cover, elem="cover_leaf")
        cover_leaf_rest = _aabb_center(cover_leaf_rest_aabb)
        dish_origin_rest = ctx.part_world_position(dish_assembly)

        ctx.check(
            "feed horn sits forward of the elevation axis at rest",
            feed_horn_rest is not None and feed_horn_rest[0] > 0.75 and abs(feed_horn_rest[1]) < 0.02,
            details=f"feed_horn_rest={feed_horn_rest}",
        )

        if cover_leaf_rest_aabb is None:
            ctx.fail("rear cover leaf has measurable geometry", "No AABB for rear cover leaf.")
        else:
            minimum, maximum = cover_leaf_rest_aabb
            ctx.check(
                "rear cover closes inside the electronics bay opening",
                dish_origin_rest is not None
                and minimum[1] - dish_origin_rest[1] > 0.145
                and maximum[1] - dish_origin_rest[1] < 0.335
                and minimum[2] - dish_origin_rest[2] > -0.145
                and maximum[2] - dish_origin_rest[2] < 0.03,
                details=f"cover_leaf_rest_aabb={cover_leaf_rest_aabb}, dish_origin_rest={dish_origin_rest}",
            )
            ctx.check(
                "rear cover sits on the back face of the support frame",
                cover_leaf_rest is not None
                and dish_origin_rest is not None
                and cover_leaf_rest[0] - dish_origin_rest[0] < -0.265,
                details=f"cover_leaf_rest={cover_leaf_rest}, dish_origin_rest={dish_origin_rest}",
            )

    with ctx.pose({azimuth_axis: math.pi / 2.0, elevation_axis: 0.0, rear_cover_hinge: 0.0}):
        feed_horn_az = _aabb_center(ctx.part_element_world_aabb(dish_assembly, elem="feed_horn"))
        ctx.check(
            "azimuth rotation swings the feed horn around the vertical axis",
            feed_horn_az is not None and feed_horn_az[1] > 0.75 and abs(feed_horn_az[0]) < 0.15,
            details=f"feed_horn_az={feed_horn_az}",
        )

    with ctx.pose({azimuth_axis: 0.0, elevation_axis: 0.75, rear_cover_hinge: 0.0}):
        feed_horn_elevated = _aabb_center(ctx.part_element_world_aabb(dish_assembly, elem="feed_horn"))
        ctx.check(
            "positive elevation raises the dish nose",
            feed_horn_rest is not None
            and feed_horn_elevated is not None
            and feed_horn_elevated[2] > feed_horn_rest[2] + 0.35,
            details=f"rest={feed_horn_rest}, elevated={feed_horn_elevated}",
        )

    with ctx.pose({azimuth_axis: 0.0, elevation_axis: 0.0, rear_cover_hinge: 1.0}):
        cover_leaf_open = _aabb_center(ctx.part_element_world_aabb(rear_cover, elem="cover_leaf"))
        ctx.check(
            "rear cover opens outward from the support frame",
            cover_leaf_rest is not None
            and cover_leaf_open is not None
            and cover_leaf_open[0] < cover_leaf_rest[0] - 0.05,
            details=f"rest={cover_leaf_rest}, open={cover_leaf_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
