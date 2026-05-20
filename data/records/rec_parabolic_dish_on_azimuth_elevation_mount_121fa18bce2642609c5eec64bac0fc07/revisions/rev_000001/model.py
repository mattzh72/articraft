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
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    min_corner, max_corner = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_satellite_dish")

    case_gray = model.material("case_gray", rgba=(0.46, 0.49, 0.52, 1.0))
    case_dark = model.material("case_dark", rgba=(0.26, 0.28, 0.30, 1.0))
    dish_white = model.material("dish_white", rgba=(0.90, 0.92, 0.94, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    carry_handle_mesh = _save_mesh(
        "carry_handle",
        wire_from_points(
            [
                (-0.055, 0.0, 0.060),
                (-0.055, 0.020, 0.108),
                (0.055, 0.020, 0.108),
                (0.055, 0.0, 0.060),
            ],
            radius=0.007,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.018,
            corner_segments=8,
        ),
    )

    reflector_shell_mesh = _save_mesh(
        "reflector_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.016, -0.145),
                (0.050, -0.143),
                (0.160, -0.134),
                (0.295, -0.102),
                (0.400, -0.056),
                (0.455, -0.006),
            ],
            [
                (0.008, -0.137),
                (0.044, -0.136),
                (0.150, -0.128),
                (0.285, -0.099),
                (0.392, -0.055),
                (0.446, -0.015),
            ],
            segments=72,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ),
    )
    reflector_rim_mesh = _save_mesh(
        "reflector_rim",
        TorusGeometry(radius=0.452, tube=0.010, radial_segments=18, tubular_segments=84),
    )
    rear_cradle_mesh = _save_mesh(
        "rear_cradle",
        wire_from_points(
            [
                (0.020, 0.105, -0.010),
                (0.070, 0.165, 0.060),
                (0.155, 0.160, 0.155),
                (0.215, 0.000, 0.215),
                (0.155, -0.160, 0.155),
                (0.070, -0.165, 0.060),
                (0.020, -0.105, -0.010),
            ],
            radius=0.010,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=10,
        ),
    )
    left_shell_brace_mesh = _save_mesh(
        "left_shell_brace",
        tube_from_spline_points(
            [
                (0.020, 0.080, -0.002),
                (0.105, 0.110, 0.055),
                (0.175, 0.082, 0.070),
                (0.215, 0.050, 0.018),
            ],
            radius=0.008,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    right_shell_brace_mesh = _save_mesh(
        "right_shell_brace",
        tube_from_spline_points(
            [
                (0.020, -0.080, -0.002),
                (0.105, -0.110, 0.055),
                (0.175, -0.082, 0.070),
                (0.215, -0.050, 0.018),
            ],
            radius=0.008,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    top_shell_brace_mesh = _save_mesh(
        "top_shell_brace",
        tube_from_spline_points(
            [
                (0.030, 0.000, 0.010),
                (0.110, 0.000, 0.110),
                (0.185, 0.000, 0.175),
                (0.215, 0.000, 0.145),
            ],
            radius=0.008,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    feed_boom_mesh = _save_mesh(
        "feed_boom",
        sweep_profile_along_spline(
            [
                (0.030, 0.0, 0.020),
                (0.150, 0.0, 0.040),
                (0.310, 0.0, 0.038),
                (0.470, 0.0, 0.022),
            ],
            profile=rounded_rect_profile(0.028, 0.022, radius=0.004, corner_segments=6),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    left_feed_strut_mesh = _save_mesh(
        "left_feed_strut",
        tube_from_spline_points(
            [
                (0.150, 0.110, 0.105),
                (0.245, 0.080, 0.080),
                (0.345, 0.035, 0.055),
                (0.400, 0.008, 0.036),
            ],
            radius=0.006,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    right_feed_strut_mesh = _save_mesh(
        "right_feed_strut",
        tube_from_spline_points(
            [
                (0.150, -0.110, 0.105),
                (0.245, -0.080, 0.080),
                (0.345, -0.035, 0.055),
                (0.400, -0.008, 0.036),
            ],
            radius=0.006,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )

    ground_base = model.part("ground_base")
    ground_base.visual(
        Box((0.56, 0.40, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=rubber,
        name="skid_plate",
    )
    ground_base.visual(
        Box((0.48, 0.34, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=case_gray,
        name="base_case",
    )
    ground_base.visual(
        Box((0.38, 0.24, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=case_dark,
        name="lid_step",
    )
    ground_base.visual(
        Cylinder(radius=0.12, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.173)),
        material=metal_dark,
        name="bearing_plate",
    )
    ground_base.visual(
        carry_handle_mesh,
        origin=Origin(xyz=(0.0, 0.170, 0.0)),
        material=metal_dark,
        name="left_handle",
    )
    ground_base.visual(
        carry_handle_mesh,
        origin=Origin(xyz=(0.0, -0.170, 0.0), rpy=(0.0, 0.0, math.pi)),
        material=metal_dark,
        name="right_handle",
    )
    ground_base.inertial = Inertial.from_geometry(
        Box((0.56, 0.40, 0.186)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
    )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.115, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=metal_dark,
        name="turntable_drum",
    )
    azimuth_head.visual(
        Box((0.38, 0.28, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=case_dark,
        name="rotating_housing",
    )
    azimuth_head.visual(
        Box((0.30, 0.22, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=case_gray,
        name="top_deck",
    )
    azimuth_head.visual(
        Box((0.08, 0.08, 0.10)),
        origin=Origin(xyz=(0.050, -0.180, 0.090)),
        material=metal_dark,
        name="gearbox_housing",
    )
    azimuth_head.visual(
        Box((0.10, 0.07, 0.12)),
        origin=Origin(xyz=(-0.060, 0.145, 0.256)),
        material=case_gray,
        name="left_yoke_pedestal",
    )
    azimuth_head.visual(
        Box((0.10, 0.07, 0.12)),
        origin=Origin(xyz=(-0.060, -0.145, 0.256)),
        material=case_gray,
        name="right_yoke_pedestal",
    )
    azimuth_head.visual(
        Box((0.064, 0.04, 0.18)),
        origin=Origin(xyz=(-0.060, 0.125, 0.406)),
        material=case_gray,
        name="left_yoke_arm",
    )
    azimuth_head.visual(
        Box((0.064, 0.04, 0.18)),
        origin=Origin(xyz=(-0.060, -0.125, 0.406)),
        material=case_gray,
        name="right_yoke_arm",
    )
    azimuth_head.visual(
        Box((0.12, 0.34, 0.04)),
        origin=Origin(xyz=(-0.060, 0.0, 0.516)),
        material=case_gray,
        name="rear_bridge",
    )
    azimuth_head.inertial = Inertial.from_geometry(
        Box((0.46, 0.36, 0.54)),
        mass=14.0,
        origin=Origin(xyz=(-0.01, 0.0, 0.27)),
    )

    dish_frame = model.part("dish_frame")
    dish_frame.visual(
        Box((0.08, 0.20, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_dark,
        name="pivot_block",
    )
    dish_frame.visual(
        Cylinder(radius=0.032, length=0.21),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_dark,
        name="hub_tube",
    )
    dish_frame.visual(
        Box((0.18, 0.05, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, 0.010)),
        material=metal_dark,
        name="rear_spine",
    )
    dish_frame.visual(rear_cradle_mesh, material=metal_dark, name="rear_cradle")
    dish_frame.visual(left_shell_brace_mesh, material=metal_dark, name="left_shell_brace")
    dish_frame.visual(right_shell_brace_mesh, material=metal_dark, name="right_shell_brace")
    dish_frame.visual(top_shell_brace_mesh, material=metal_dark, name="top_shell_brace")
    dish_frame.visual(
        reflector_shell_mesh,
        origin=Origin(xyz=(0.360, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dish_white,
        name="reflector_shell",
    )
    dish_frame.visual(
        reflector_rim_mesh,
        origin=Origin(xyz=(0.360, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="reflector_rim",
    )
    dish_frame.visual(feed_boom_mesh, material=aluminum, name="feed_boom")
    dish_frame.visual(left_feed_strut_mesh, material=aluminum, name="left_feed_strut")
    dish_frame.visual(right_feed_strut_mesh, material=aluminum, name="right_feed_strut")
    dish_frame.visual(
        Box((0.06, 0.05, 0.05)),
        origin=Origin(xyz=(0.470, 0.0, 0.022)),
        material=case_dark,
        name="feed_block",
    )
    dish_frame.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(0.500, 0.0, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="lnb_neck",
    )
    dish_frame.visual(
        Cylinder(radius=0.028, length=0.09),
        origin=Origin(xyz=(0.575, 0.0, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    dish_frame.inertial = Inertial.from_geometry(
        Box((0.70, 0.92, 0.92)),
        mass=8.5,
        origin=Origin(xyz=(0.280, 0.0, 0.020)),
    )

    side_crank = model.part("side_crank")
    side_crank.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_dark,
        name="crank_hub",
    )
    side_crank.visual(
        Box((0.09, 0.018, 0.014)),
        origin=Origin(xyz=(0.045, -0.014, -0.005)),
        material=metal_dark,
        name="crank_arm",
    )
    side_crank.visual(
        Box((0.016, 0.014, 0.060)),
        origin=Origin(xyz=(0.090, -0.014, -0.032)),
        material=metal_dark,
        name="crank_post",
    )
    side_crank.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(xyz=(0.090, -0.014, -0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="crank_grip",
    )
    side_crank.inertial = Inertial.from_geometry(
        Box((0.12, 0.04, 0.09)),
        mass=0.6,
        origin=Origin(xyz=(0.050, -0.014, -0.030)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=ground_base,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0),
    )
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=dish_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.406)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.1,
            lower=math.radians(-12.0),
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "crank_rotation",
        ArticulationType.CONTINUOUS,
        parent=azimuth_head,
        child=side_crank,
        origin=Origin(xyz=(0.090, -0.220, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ground_base = object_model.get_part("ground_base")
    azimuth_head = object_model.get_part("azimuth_head")
    dish_frame = object_model.get_part("dish_frame")
    side_crank = object_model.get_part("side_crank")

    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_tilt = object_model.get_articulation("elevation_tilt")
    crank_rotation = object_model.get_articulation("crank_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(azimuth_head, ground_base, elem_a="turntable_drum", elem_b="bearing_plate")
    ctx.expect_contact(dish_frame, azimuth_head, elem_a="hub_tube", elem_b="left_yoke_arm")
    ctx.expect_contact(dish_frame, azimuth_head, elem_a="hub_tube", elem_b="right_yoke_arm")
    ctx.expect_contact(side_crank, azimuth_head, elem_a="crank_hub", elem_b="gearbox_housing")
    ctx.expect_origin_gap(dish_frame, azimuth_head, axis="z", min_gap=0.40, max_gap=0.41)

    feed_rest_aabb = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
    grip_rest_aabb = ctx.part_element_world_aabb(side_crank, elem="crank_grip")
    if feed_rest_aabb is None:
        ctx.fail("feed horn visual resolved", "feed_horn AABB was not available")
        return ctx.report()
    if grip_rest_aabb is None:
        ctx.fail("crank grip visual resolved", "crank_grip AABB was not available")
        return ctx.report()

    feed_rest_center = _aabb_center(feed_rest_aabb)
    grip_rest_center = _aabb_center(grip_rest_aabb)

    with ctx.pose({elevation_tilt: math.radians(50.0)}):
        feed_raised_aabb = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        if feed_raised_aabb is None:
            ctx.fail("elevated feed horn visual resolved", "feed_horn AABB missing in elevated pose")
        else:
            feed_raised_center = _aabb_center(feed_raised_aabb)
            ctx.check(
                "elevation raises feed horn",
                feed_raised_center[2] > feed_rest_center[2] + 0.22,
                details=(
                    f"feed horn z did not rise enough: rest={feed_rest_center[2]:.3f}, "
                    f"raised={feed_raised_center[2]:.3f}"
                ),
            )
        ctx.expect_contact(dish_frame, azimuth_head, elem_a="hub_tube", elem_b="left_yoke_arm")
        ctx.expect_contact(dish_frame, azimuth_head, elem_a="hub_tube", elem_b="right_yoke_arm")

    with ctx.pose({azimuth_rotation: math.radians(35.0)}):
        feed_swept_aabb = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        if feed_swept_aabb is None:
            ctx.fail("azimuth-swept feed horn visual resolved", "feed_horn AABB missing in azimuth pose")
        else:
            feed_swept_center = _aabb_center(feed_swept_aabb)
            xy_shift = math.hypot(
                feed_swept_center[0] - feed_rest_center[0],
                feed_swept_center[1] - feed_rest_center[1],
            )
            ctx.check(
                "azimuth sweeps reflector around vertical axis",
                xy_shift > 0.24,
                details=f"feed horn XY shift was only {xy_shift:.3f} m",
            )

    with ctx.pose({crank_rotation: math.radians(120.0)}):
        grip_rotated_aabb = ctx.part_element_world_aabb(side_crank, elem="crank_grip")
        if grip_rotated_aabb is None:
            ctx.fail("rotated crank grip visual resolved", "crank_grip AABB missing in crank pose")
        else:
            grip_rotated_center = _aabb_center(grip_rotated_aabb)
            crank_swing = math.hypot(
                grip_rotated_center[0] - grip_rest_center[0],
                grip_rotated_center[2] - grip_rest_center[2],
            )
            ctx.check(
                "side crank rotates on its shaft axis",
                crank_swing > 0.09,
                details=f"crank grip swing was only {crank_swing:.3f} m",
            )
        ctx.expect_contact(side_crank, azimuth_head, elem_a="crank_hub", elem_b="gearbox_housing")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
