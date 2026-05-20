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


def _dish_shell_mesh():
    outer_profile = [
        (0.05, -0.03),
        (0.14, 0.03),
        (0.32, 0.16),
        (0.55, 0.34),
        (0.72, 0.54),
    ]
    inner_profile = [
        (0.03, -0.01),
        (0.12, 0.05),
        (0.29, 0.17),
        (0.52, 0.35),
        (0.72, 0.54),
    ]
    return _mesh(
        "dish_reflector_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )


def _feed_strut_mesh():
    return _mesh(
        "feed_support_strut",
        tube_from_spline_points(
            [
                (0.10, 0.0, 0.0),
                (0.42, 0.0, 0.18),
                (0.70, 0.0, 0.04),
            ],
            radius=0.014,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radio_telescope")

    concrete = model.material("concrete", rgba=(0.68, 0.69, 0.70, 1.0))
    off_white = model.material("off_white", rgba=(0.90, 0.92, 0.93, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    dish_shell = _dish_shell_mesh()
    feed_strut = _feed_strut_mesh()

    base = model.part("azimuth_base")
    base.visual(
        Cylinder(radius=0.54, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=concrete,
        name="foundation_pier",
    )
    base.visual(
        Cylinder(radius=0.40, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=dark_metal,
        name="top_plate",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.54, length=0.24),
        mass=1400.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    pedestal = model.part("pedestal_yoke")
    pedestal.visual(
        Cylinder(radius=0.34, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=machinery_gray,
        name="slew_drum",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=off_white,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.22, 1.10, 0.16)),
        origin=Origin(xyz=(-0.18, 0.0, 0.98)),
        material=machinery_gray,
        name="head_block",
    )
    pedestal.visual(
        Box((0.14, 0.16, 0.50)),
        origin=Origin(xyz=(0.0, 0.62, 1.27)),
        material=off_white,
        name="right_cheek",
    )
    pedestal.visual(
        Box((0.14, 0.16, 0.50)),
        origin=Origin(xyz=(0.0, -0.62, 1.27)),
        material=off_white,
        name="left_cheek",
    )
    pedestal.visual(
        Box((0.20, 1.34, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.57)),
        material=machinery_gray,
        name="tie_beam",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.16),
        origin=Origin(xyz=(0.0, 0.60, 1.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.16),
        origin=Origin(xyz=(0.0, -0.60, 1.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_bearing",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.80, 1.40, 1.70)),
        mass=450.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )

    dish = model.part("dish_assembly")
    dish.visual(
        Box((0.26, 0.44, 0.28)),
        material=machinery_gray,
        name="hub_block",
    )
    dish.visual(
        Cylinder(radius=0.07, length=0.88),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="cross_tube",
    )
    dish.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(xyz=(0.0, 0.48, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    dish.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(xyz=(0.0, -0.48, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    dish.visual(
        Cylinder(radius=0.14, length=0.26),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_barrel",
    )
    dish.visual(
        dish_shell,
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="dish_reflector",
    )
    dish.visual(
        _mesh(
            "dish_rim_stiffener",
            TorusGeometry(radius=0.72, tube=0.025, radial_segments=14, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.56, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="rim_stiffener",
    )
    for index in range(3):
        dish.visual(
            feed_strut,
            origin=Origin(rpy=((2.0 * math.pi * index) / 3.0, 0.0, 0.0)),
            material=aluminum,
            name=f"feed_strut_{index}",
        )
    dish.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.70, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="feed_collar",
    )
    dish.visual(
        Cylinder(radius=0.04, length=0.18),
        origin=Origin(xyz=(0.83, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    dish.visual(
        Box((0.54, 0.08, 0.08)),
        origin=Origin(xyz=(-0.40, 0.14, 0.0)),
        material=dark_metal,
        name="counterweight_beam_upper",
    )
    dish.visual(
        Box((0.54, 0.08, 0.08)),
        origin=Origin(xyz=(-0.40, -0.14, 0.0)),
        material=dark_metal,
        name="counterweight_beam_lower",
    )
    dish.visual(
        Box((0.24, 0.38, 0.34)),
        origin=Origin(xyz=(-0.79, 0.0, 0.0)),
        material=machinery_gray,
        name="counterweight_mass",
    )
    dish.inertial = Inertial.from_geometry(
        Box((1.75, 1.55, 1.20)),
        mass=180.0,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.8),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.8,
            lower=0.0,
            upper=1.25,
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

    base = object_model.get_part("azimuth_base")
    pedestal = object_model.get_part("pedestal_yoke")
    dish = object_model.get_part("dish_assembly")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_rotation")

    ctx.check(
        "azimuth joint is vertical continuous rotation",
        azimuth.axis == (0.0, 0.0, 1.0)
        and azimuth.motion_limits is not None
        and azimuth.motion_limits.lower is None
        and azimuth.motion_limits.upper is None,
        details=f"axis={azimuth.axis}, limits={azimuth.motion_limits}",
    )
    ctx.check(
        "elevation joint uses horizontal trunnion axis",
        elevation.axis == (0.0, -1.0, 0.0)
        and elevation.motion_limits is not None
        and elevation.motion_limits.lower == 0.0
        and elevation.motion_limits.upper is not None
        and elevation.motion_limits.upper > 1.0,
        details=f"axis={elevation.axis}, limits={elevation.motion_limits}",
    )

    with ctx.pose({elevation: 0.0, azimuth: 0.0}):
        ctx.expect_gap(
            pedestal,
            base,
            axis="z",
            positive_elem="slew_drum",
            negative_elem="top_plate",
            max_gap=0.001,
            max_penetration=1e-6,
            name="pedestal sits on the azimuth base",
        )
        ctx.expect_contact(
            dish,
            pedestal,
            elem_a="right_trunnion",
            elem_b="right_bearing",
            contact_tol=1e-4,
            name="right trunnion meets right bearing housing",
        )
        ctx.expect_contact(
            dish,
            pedestal,
            elem_a="left_trunnion",
            elem_b="left_bearing",
            contact_tol=1e-4,
            name="left trunnion meets left bearing housing",
        )

        rest_reflector = ctx.part_element_world_aabb(dish, elem="dish_reflector")

    with ctx.pose({elevation: elevation.motion_limits.upper, azimuth: 0.0}):
        elevated_reflector = ctx.part_element_world_aabb(dish, elem="dish_reflector")

    def aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((mn + mx) * 0.5 for mn, mx in zip(min_corner, max_corner))

    rest_center = aabb_center(rest_reflector)
    elevated_center = aabb_center(elevated_reflector)
    ctx.check(
        "positive elevation raises the reflector",
        rest_center is not None
        and elevated_center is not None
        and elevated_center[2] > rest_center[2] + 0.20,
        details=f"rest_center={rest_center}, elevated_center={elevated_center}",
    )

    with ctx.pose({elevation: 0.25, azimuth: 0.0}):
        az_rest_reflector = ctx.part_element_world_aabb(dish, elem="feed_horn")
    with ctx.pose({elevation: 0.25, azimuth: math.pi / 2.0}):
        az_turned_reflector = ctx.part_element_world_aabb(dish, elem="feed_horn")

    az_rest_center = aabb_center(az_rest_reflector)
    az_turned_center = aabb_center(az_turned_reflector)
    ctx.check(
        "positive azimuth swings the telescope around the pedestal",
        az_rest_center is not None
        and az_turned_center is not None
        and az_rest_center[0] > 0.4
        and abs(az_turned_center[0]) < 0.25
        and az_turned_center[1] > 0.4,
        details=f"rest_center={az_rest_center}, turned_center={az_turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
