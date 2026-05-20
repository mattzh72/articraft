from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_perimeter_floodlight")

    bracket_gray = model.material("bracket_gray", rgba=(0.38, 0.40, 0.42, 1.0))
    head_black = model.material("head_black", rgba=(0.12, 0.13, 0.14, 1.0))
    anodized_dark = model.material("anodized_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.79, 0.84, 0.35))

    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.012, 0.120, 0.180)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=bracket_gray,
        name="mount_plate",
    )
    wall_mount.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.021, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_gray,
        name="mount_hub",
    )
    wall_mount.visual(
        Box((0.020, 0.060, 0.028)),
        origin=Origin(xyz=(0.022, 0.0, -0.034)),
        material=bracket_gray,
        name="lower_mount_pad",
    )
    wall_mount.inertial = Inertial.from_geometry(
        Box((0.034, 0.120, 0.180)),
        mass=1.6,
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
    )

    arm = model.part("arm")
    arm_geom = sweep_profile_along_spline(
        [
            (0.010, 0.0, 0.000),
            (0.070, 0.0, 0.014),
            (0.155, 0.0, 0.040),
            (0.225, 0.0, 0.046),
        ],
        profile=rounded_rect_profile(0.046, 0.030, radius=0.007),
        samples_per_segment=18,
        cap_profile=True,
    )
    arm_geom.merge(
        tube_from_spline_points(
            [
                (0.016, 0.0, -0.026),
                (0.090, 0.0, 0.000),
                (0.190, 0.0, 0.030),
            ],
            radius=0.010,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        )
    )
    arm_geom.merge(BoxGeometry((0.024, 0.056, 0.060)).translate(0.012, 0.0, 0.0))
    arm_geom.merge(CylinderGeometry(radius=0.029, height=0.016).translate(0.225, 0.0, 0.052))
    arm_mesh = mesh_from_geometry(arm_geom, "floodlight_arm")
    arm.visual(arm_mesh, material=bracket_gray, name="arm_assembly")
    arm.inertial = Inertial.from_geometry(
        Box((0.245, 0.060, 0.090)),
        mass=1.2,
        origin=Origin(xyz=(0.120, 0.0, 0.020)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=anodized_dark,
        name="bearing_drum",
    )
    pan_yoke.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=anodized_dark,
        name="turntable_cap",
    )
    pan_yoke.visual(
        Box((0.036, 0.132, 0.018)),
        origin=Origin(xyz=(-0.008, 0.0, 0.037)),
        material=bracket_gray,
        name="yoke_saddle",
    )
    pan_yoke.visual(
        Box((0.018, 0.020, 0.144)),
        origin=Origin(xyz=(-0.008, -0.074, 0.095)),
        material=bracket_gray,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.018, 0.020, 0.144)),
        origin=Origin(xyz=(-0.008, 0.074, 0.095)),
        material=bracket_gray,
        name="right_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.018, 0.168, 0.016)),
        origin=Origin(xyz=(-0.008, 0.0, 0.161)),
        material=bracket_gray,
        name="top_bridge",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.080, 0.168, 0.150)),
        mass=0.7,
        origin=Origin(xyz=(-0.004, 0.0, 0.075)),
    )

    flood_head = model.part("flood_head")
    flood_head.visual(
        Box((0.010, 0.104, 0.100)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=head_black,
        name="rear_panel",
    )
    flood_head.visual(
        Box((0.088, 0.104, 0.010)),
        origin=Origin(xyz=(0.032, 0.0, 0.045)),
        material=head_black,
        name="top_wall",
    )
    flood_head.visual(
        Box((0.088, 0.104, 0.010)),
        origin=Origin(xyz=(0.032, 0.0, -0.045)),
        material=head_black,
        name="bottom_wall",
    )
    flood_head.visual(
        Box((0.088, 0.010, 0.090)),
        origin=Origin(xyz=(0.032, -0.047, 0.0)),
        material=head_black,
        name="left_wall",
    )
    flood_head.visual(
        Box((0.088, 0.010, 0.090)),
        origin=Origin(xyz=(0.032, 0.047, 0.0)),
        material=head_black,
        name="right_wall",
    )
    flood_head.visual(
        Box((0.010, 0.104, 0.010)),
        origin=Origin(xyz=(0.074, 0.0, 0.045)),
        material=anodized_dark,
        name="bezel_top",
    )
    flood_head.visual(
        Box((0.010, 0.104, 0.010)),
        origin=Origin(xyz=(0.074, 0.0, -0.045)),
        material=anodized_dark,
        name="bezel_bottom",
    )
    flood_head.visual(
        Box((0.010, 0.010, 0.082)),
        origin=Origin(xyz=(0.074, -0.047, 0.0)),
        material=anodized_dark,
        name="bezel_left",
    )
    flood_head.visual(
        Box((0.010, 0.010, 0.082)),
        origin=Origin(xyz=(0.074, 0.047, 0.0)),
        material=anodized_dark,
        name="bezel_right",
    )
    flood_head.visual(
        Box((0.020, 0.108, 0.008)),
        origin=Origin(xyz=(0.070, 0.0, 0.054)),
        material=head_black,
        name="visor_lip",
    )
    flood_head.visual(
        Box((0.004, 0.094, 0.082)),
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )
    for index, z_pos in enumerate((-0.028, -0.014, 0.0, 0.014, 0.028)):
        flood_head.visual(
            Box((0.020, 0.080, 0.006)),
            origin=Origin(xyz=(-0.020, 0.0, z_pos)),
            material=anodized_dark,
            name=f"cooling_fin_{index}",
        )
    flood_head.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(-0.004, -0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_dark,
        name="left_trunnion",
    )
    flood_head.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(-0.004, 0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_dark,
        name="right_trunnion",
    )
    flood_head.inertial = Inertial.from_geometry(
        Box((0.110, 0.114, 0.110)),
        mass=1.3,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "wall_to_arm",
        ArticulationType.FIXED,
        parent=wall_mount,
        child=arm,
        origin=Origin(xyz=(0.030, 0.0, 0.010)),
    )
    model.articulation(
        "arm_to_pan_yoke",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=pan_yoke,
        origin=Origin(xyz=(0.225, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0),
    )
    model.articulation(
        "pan_yoke_to_flood_head",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=flood_head,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=math.radians(-35.0),
            upper=math.radians(95.0),
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

    wall_mount = object_model.get_part("wall_mount")
    arm = object_model.get_part("arm")
    pan_yoke = object_model.get_part("pan_yoke")
    flood_head = object_model.get_part("flood_head")

    azimuth = object_model.get_articulation("arm_to_pan_yoke")
    tilt = object_model.get_articulation("pan_yoke_to_flood_head")

    def aabb_center(aabb):
        if aabb is None:
            return None
        min_pt, max_pt = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))

    ctx.check(
        "assembly parts resolved",
        all(part is not None for part in (wall_mount, arm, pan_yoke, flood_head)),
        details="One or more expected parts could not be resolved.",
    )
    ctx.expect_contact(arm, wall_mount, name="arm seats against wall mount")
    ctx.expect_contact(pan_yoke, arm, name="pan bearing seats on arm tip")
    ctx.expect_contact(flood_head, pan_yoke, name="tilt trunnions seat in yoke")

    ctx.check(
        "azimuth axis is vertical",
        tuple(round(value, 6) for value in azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"axis={azimuth.axis}",
    )
    ctx.check(
        "tilt axis is lateral",
        tuple(round(value, 6) for value in tilt.axis) == (0.0, 1.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    azimuth_origin = ctx.part_world_position(pan_yoke)
    rest_glass = aabb_center(ctx.part_element_world_aabb(flood_head, elem="front_glass"))
    with ctx.pose({azimuth: math.pi / 2.0, tilt: 0.0}):
        quarter_turn_origin = ctx.part_world_position(pan_yoke)
        quarter_turn_glass = aabb_center(
            ctx.part_element_world_aabb(flood_head, elem="front_glass")
        )

    azimuth_ok = (
        azimuth_origin is not None
        and rest_glass is not None
        and quarter_turn_origin is not None
        and quarter_turn_glass is not None
        and abs(rest_glass[1] - azimuth_origin[1]) < 0.010
        and rest_glass[0] > azimuth_origin[0] + 0.050
        and abs(quarter_turn_glass[0] - quarter_turn_origin[0]) < 0.015
        and quarter_turn_glass[1] > quarter_turn_origin[1] + 0.050
    )
    ctx.check(
        "azimuth rotation swings head around bearing",
        azimuth_ok,
        details=(
            f"origin_rest={azimuth_origin}, glass_rest={rest_glass}, "
            f"origin_q90={quarter_turn_origin}, glass_q90={quarter_turn_glass}"
        ),
    )

    tilt_axis_center = ctx.part_world_position(flood_head)
    with ctx.pose({azimuth: 0.0, tilt: math.radians(-25.0)}):
        aimed_up_glass = aabb_center(ctx.part_element_world_aabb(flood_head, elem="front_glass"))
    with ctx.pose({azimuth: 0.0, tilt: math.radians(55.0)}):
        aimed_down_glass = aabb_center(
            ctx.part_element_world_aabb(flood_head, elem="front_glass")
        )

    tilt_ok = (
        tilt_axis_center is not None
        and aimed_up_glass is not None
        and aimed_down_glass is not None
        and aimed_up_glass[2] > tilt_axis_center[2] + 0.020
        and aimed_down_glass[2] < tilt_axis_center[2] - 0.020
        and aimed_up_glass[0] > tilt_axis_center[0] + 0.040
        and aimed_down_glass[0] > tilt_axis_center[0] + 0.025
    )
    ctx.check(
        "tilt hinge pitches the flood head",
        tilt_ok,
        details=(
            f"axis_center={tilt_axis_center}, "
            f"up_glass={aimed_up_glass}, down_glass={aimed_down_glass}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
