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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_structure_cctv_mount")

    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.70, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    housing_white = model.material("housing_white", rgba=(0.88, 0.89, 0.90, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.20, 0.24, 0.28, 0.35))
    sensor_black = model.material("sensor_black", rgba=(0.05, 0.05, 0.06, 1.0))

    beam_bracket = model.part("beam_bracket")
    beam_bracket.visual(
        Box((0.28, 0.09, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=galvanized,
        name="elem_top_saddle",
    )
    beam_bracket.visual(
        Box((0.024, 0.09, 0.082)),
        origin=Origin(xyz=(-0.128, 0.0, 0.041)),
        material=galvanized,
        name="elem_left_cheek",
    )
    beam_bracket.visual(
        Box((0.024, 0.09, 0.082)),
        origin=Origin(xyz=(0.128, 0.0, 0.041)),
        material=galvanized,
        name="elem_right_cheek",
    )
    beam_bracket.visual(
        Box((0.18, 0.06, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=galvanized,
        name="elem_lower_jaw",
    )
    beam_bracket.visual(
        Cylinder(radius=0.008, length=0.07),
        origin=Origin(xyz=(-0.08, 0.0, 0.041)),
        material=dark_paint,
        name="elem_left_bolt",
    )
    beam_bracket.visual(
        Cylinder(radius=0.008, length=0.07),
        origin=Origin(xyz=(0.08, 0.0, 0.041)),
        material=dark_paint,
        name="elem_right_bolt",
    )
    beam_bracket.visual(
        Cylinder(radius=0.025, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_paint,
        name="elem_hanger_lug",
    )
    beam_bracket.inertial = Inertial.from_geometry(
        Box((0.28, 0.10, 0.132)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    drop_rod = model.part("drop_rod")
    drop_rod.visual(
        Cylinder(radius=0.021, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=dark_paint,
        name="elem_upper_coupler",
    )
    drop_rod.visual(
        Cylinder(radius=0.014, length=0.39),
        origin=Origin(xyz=(0.0, 0.0, -0.219)),
        material=dark_paint,
        name="elem_rod_tube",
    )
    drop_rod.visual(
        Cylinder(radius=0.022, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, -0.429)),
        material=dark_paint,
        name="elem_lower_coupler",
    )
    drop_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.444),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.222)),
    )

    model.articulation(
        "beam_bracket_to_drop_rod",
        ArticulationType.FIXED,
        parent=beam_bracket,
        child=drop_rod,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.03, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_paint,
        name="elem_pan_bearing",
    )
    pan_yoke.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=dark_paint,
        name="elem_pan_neck",
    )
    pan_yoke.visual(
        Box((0.06, 0.24, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=dark_paint,
        name="elem_yoke_bridge",
    )
    pan_yoke.visual(
        Box((0.024, 0.02, 0.15)),
        origin=Origin(xyz=(0.0, -0.10, -0.155)),
        material=dark_paint,
        name="elem_left_arm",
    )
    pan_yoke.visual(
        Box((0.024, 0.02, 0.15)),
        origin=Origin(xyz=(0.0, 0.10, -0.155)),
        material=dark_paint,
        name="elem_right_arm",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.08, 0.24, 0.23)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
    )

    model.articulation(
        "drop_rod_to_pan",
        ArticulationType.CONTINUOUS,
        parent=drop_rod,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.444)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )

    cap_outer_profile = [
        (0.018, 0.03),
        (0.044, 0.028),
        (0.07, 0.013),
        (0.078, -0.006),
        (0.074, -0.03),
    ]
    cap_inner_profile = [
        (0.010, 0.025),
        (0.036, 0.023),
        (0.064, 0.011),
        (0.07, -0.004),
        (0.068, -0.028),
    ]
    bubble_outer_profile = [
        (0.074, -0.03),
        (0.082, -0.055),
        (0.086, -0.095),
        (0.074, -0.145),
        (0.018, -0.176),
    ]
    bubble_inner_profile = [
        (0.07, -0.03),
        (0.078, -0.055),
        (0.082, -0.095),
        (0.07, -0.145),
        (0.01, -0.173),
    ]

    dome_cap_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(cap_outer_profile, cap_inner_profile, segments=56),
        "dome_cap_shell",
    )
    clear_bubble_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            bubble_outer_profile,
            bubble_inner_profile,
            segments=64,
        ),
        "dome_clear_bubble",
    )

    dome_housing = model.part("dome_housing")
    dome_housing.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="elem_trunnion_axle",
    )
    dome_housing.visual(
        dome_cap_mesh,
        material=housing_white,
        name="elem_upper_cap",
    )
    dome_housing.visual(
        Cylinder(radius=0.073, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=sensor_black,
        name="elem_seam_collar",
    )
    dome_housing.visual(
        clear_bubble_mesh,
        material=smoked_clear,
        name="elem_clear_dome",
    )
    dome_housing.visual(
        Cylinder(radius=0.01, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=sensor_black,
        name="elem_support_stem",
    )
    dome_housing.visual(
        Box((0.05, 0.034, 0.045)),
        origin=Origin(xyz=(0.018, 0.0, -0.13)),
        material=sensor_black,
        name="elem_camera_pod",
    )
    dome_housing.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.018, 0.0, -0.168)),
        material=sensor_black,
        name="elem_lens_barrel",
    )
    dome_housing.inertial = Inertial.from_geometry(
        Box((0.19, 0.19, 0.22)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
    )

    model.articulation(
        "pan_to_dome_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=dome_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=math.radians(-15.0),
            upper=math.radians(80.0),
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
    beam_bracket = object_model.get_part("beam_bracket")
    drop_rod = object_model.get_part("drop_rod")
    pan_yoke = object_model.get_part("pan_yoke")
    dome_housing = object_model.get_part("dome_housing")
    pan_joint = object_model.get_articulation("drop_rod_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_dome_tilt")

    ctx.expect_contact(
        drop_rod,
        beam_bracket,
        elem_a="elem_upper_coupler",
        elem_b="elem_hanger_lug",
        name="drop rod is mounted to the beam bracket lug",
    )
    ctx.expect_contact(
        pan_yoke,
        drop_rod,
        elem_a="elem_pan_bearing",
        elem_b="elem_lower_coupler",
        name="pan bearing seats against the drop rod coupler",
    )
    ctx.expect_contact(
        dome_housing,
        pan_yoke,
        elem_a="elem_trunnion_axle",
        elem_b="elem_left_arm",
        name="left tilt trunnion meets the yoke arm",
    )
    ctx.expect_contact(
        dome_housing,
        pan_yoke,
        elem_a="elem_trunnion_axle",
        elem_b="elem_right_arm",
        name="right tilt trunnion meets the yoke arm",
    )
    ctx.expect_origin_gap(
        beam_bracket,
        dome_housing,
        axis="z",
        min_gap=0.55,
        name="dome housing hangs well below the overhead bracket",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        rest_lens = aabb_center(ctx.part_element_world_aabb(dome_housing, elem="elem_lens_barrel"))

    with ctx.pose({pan_joint: math.pi / 2.0, tilt_joint: 0.0}):
        panned_lens = aabb_center(ctx.part_element_world_aabb(dome_housing, elem="elem_lens_barrel"))

    pan_ok = (
        rest_lens is not None
        and panned_lens is not None
        and math.hypot(rest_lens[0], rest_lens[1]) > 0.01
        and abs(math.hypot(rest_lens[0], rest_lens[1]) - math.hypot(panned_lens[0], panned_lens[1])) < 0.01
        and panned_lens[1] > 0.01
        and abs(panned_lens[0]) < abs(rest_lens[0]) + 0.01
    )
    ctx.check(
        "pan joint rotates the lens around the vertical bearing",
        pan_ok,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    tilt_target = math.radians(55.0)
    with ctx.pose({pan_joint: 0.0, tilt_joint: tilt_target}):
        tilted_lens = aabb_center(ctx.part_element_world_aabb(dome_housing, elem="elem_lens_barrel"))

    tilt_ok = (
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[0] > rest_lens[0] + 0.03
        and tilted_lens[2] > rest_lens[2] + 0.02
    )
    ctx.check(
        "tilt joint pitches the dome optics forward and upward",
        tilt_ok,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
