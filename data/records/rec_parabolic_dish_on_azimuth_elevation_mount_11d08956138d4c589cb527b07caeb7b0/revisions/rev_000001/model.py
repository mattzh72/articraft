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


def _build_collar_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.045, -0.048),
                (0.046, -0.028),
                (0.046, 0.052),
                (0.043, 0.070),
            ],
            inner_profile=[
                (0.034, -0.046),
                (0.035, -0.028),
                (0.035, 0.052),
                (0.032, 0.068),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "azimuth_collar_shell",
    )


def _build_reflector_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.000, -0.110),
                (0.050, -0.108),
                (0.160, -0.098),
                (0.275, -0.073),
                (0.338, -0.036),
                (0.360, 0.000),
            ],
            inner_profile=[
                (0.000, -0.104),
                (0.046, -0.102),
                (0.153, -0.093),
                (0.268, -0.068),
                (0.330, -0.033),
                (0.352, 0.000),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "reflector_shell",
    )


def _build_feed_strut():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.055, 0.0, -0.160),
                (0.165, 0.0, -0.125),
                (0.280, 0.0, -0.070),
                (0.392, 0.0, 0.000),
            ],
            radius=0.008,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
        "feed_support_boom",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_dish_az_el_mount")

    mast_gray = model.material("mast_gray", rgba=(0.37, 0.39, 0.42, 1.0))
    galvanized = model.material("galvanized", rgba=(0.68, 0.71, 0.74, 1.0))
    dish_white = model.material("dish_white", rgba=(0.88, 0.89, 0.90, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    receiver_gray = model.material("receiver_gray", rgba=(0.72, 0.75, 0.78, 1.0))

    collar_shell = _build_collar_shell()
    reflector_shell = _build_reflector_shell()
    feed_strut = _build_feed_strut()

    mast = model.part("mast")
    mast.visual(
        Box((0.20, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=hardware_dark,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.028, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=mast_gray,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.042, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=galvanized,
        name="base_socket",
    )
    mast.visual(
        Box((0.085, 0.012, 0.16)),
        origin=Origin(xyz=(0.022, 0.0, 0.10), rpy=(0.0, -0.50, 0.0)),
        material=hardware_dark,
        name="front_gusset",
    )
    mast.visual(
        Box((0.085, 0.012, 0.16)),
        origin=Origin(xyz=(-0.022, 0.0, 0.10), rpy=(0.0, 0.50, 0.0)),
        material=hardware_dark,
        name="rear_gusset",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, 0.54)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
    )

    collar = model.part("collar")
    collar.visual(collar_shell, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=galvanized, name="collar_shell")
    collar.visual(
        Box((0.070, 0.040, 0.008)),
        origin=Origin(xyz=(0.045, 0.0, 0.004)),
        material=hardware_dark,
        name="thrust_pad",
    )
    collar.visual(
        Box((0.020, 0.23, 0.14)),
        origin=Origin(xyz=(0.085, 0.0, 0.020)),
        material=mast_gray,
        name="pivot_panel",
    )
    collar.visual(
        Box((0.075, 0.022, 0.155)),
        origin=Origin(xyz=(0.118, 0.106, 0.028)),
        material=galvanized,
        name="left_fork_arm",
    )
    collar.visual(
        Box((0.075, 0.022, 0.155)),
        origin=Origin(xyz=(0.118, -0.106, 0.028)),
        material=galvanized,
        name="right_fork_arm",
    )
    collar.inertial = Inertial.from_geometry(
        Box((0.18, 0.28, 0.18)),
        mass=2.2,
        origin=Origin(xyz=(0.080, 0.0, 0.020)),
    )

    dish_yoke = model.part("dish_yoke")
    dish_yoke.visual(
        Cylinder(radius=0.013, length=0.19),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="trunnion_shaft",
    )
    dish_yoke.visual(
        Box((0.110, 0.065, 0.125)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=mast_gray,
        name="yoke_block",
    )
    dish_yoke.visual(
        Cylinder(radius=0.082, length=0.020),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="back_plate",
    )
    dish_yoke.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hub_barrel",
    )
    dish_yoke.visual(
        reflector_shell,
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dish_white,
        name="reflector_shell",
    )
    dish_yoke.visual(feed_strut, material=receiver_gray, name="feed_support_boom")
    dish_yoke.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=receiver_gray,
        name="feed_horn",
    )
    dish_yoke.visual(
        Box((0.045, 0.035, 0.030)),
        origin=Origin(xyz=(0.437, 0.0, 0.0)),
        material=hardware_dark,
        name="lnb_body",
    )
    dish_yoke.inertial = Inertial.from_geometry(
        Box((0.50, 0.72, 0.72)),
        mass=3.5,
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-3.0,
            upper=3.0,
        ),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=dish_yoke,
        origin=Origin(xyz=(0.120, 0.0, 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.0,
            lower=-0.25,
            upper=1.10,
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

    mast = object_model.get_part("mast")
    collar = object_model.get_part("collar")
    dish_yoke = object_model.get_part("dish_yoke")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_rotation")

    def elem_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            0.5 * (lo[0] + hi[0]),
            0.5 * (lo[1] + hi[1]),
            0.5 * (lo[2] + hi[2]),
        )

    ctx.check(
        "joint axes match azimuth-elevation mount",
        azimuth.axis == (0.0, 0.0, 1.0) and elevation.axis == (0.0, -1.0, 0.0),
        details=f"azimuth_axis={azimuth.axis}, elevation_axis={elevation.axis}",
    )
    ctx.check(
        "motion limits are realistic for dish aiming",
        (
            azimuth.motion_limits is not None
            and azimuth.motion_limits.lower is not None
            and azimuth.motion_limits.upper is not None
            and azimuth.motion_limits.lower < 0.0 < azimuth.motion_limits.upper
            and elevation.motion_limits is not None
            and elevation.motion_limits.lower is not None
            and elevation.motion_limits.upper is not None
            and elevation.motion_limits.lower <= -0.2
            and elevation.motion_limits.upper >= 1.0
        ),
        details=f"azimuth_limits={azimuth.motion_limits}, elevation_limits={elevation.motion_limits}",
    )

    ctx.expect_gap(
        dish_yoke,
        mast,
        axis="x",
        positive_elem="reflector_shell",
        negative_elem="mast_tube",
        min_gap=0.015,
        name="reflector sits forward of the mast",
    )
    ctx.expect_within(
        dish_yoke,
        collar,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="pivot_panel",
        margin=0.03,
        name="trunnion shaft stays centered in the collar pivot zone",
    )

    rest_feed = elem_center(dish_yoke, "feed_horn")
    with ctx.pose({elevation: 0.75}):
        raised_feed = elem_center(dish_yoke, "feed_horn")
    ctx.check(
        "positive elevation raises the dish nose",
        rest_feed is not None
        and raised_feed is not None
        and raised_feed[2] > rest_feed[2] + 0.12,
        details=f"rest_feed={rest_feed}, raised_feed={raised_feed}",
    )

    with ctx.pose({azimuth: 0.90}):
        swung_feed = elem_center(dish_yoke, "feed_horn")
    ctx.check(
        "positive azimuth swings the dish around the mast",
        rest_feed is not None
        and swung_feed is not None
        and abs(swung_feed[1] - rest_feed[1]) > 0.20
        and abs(swung_feed[2] - rest_feed[2]) < 0.05,
        details=f"rest_feed={rest_feed}, swung_feed={swung_feed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
