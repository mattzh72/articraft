from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _elem_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shipboard_comm_dish")

    deck_gray = model.material("deck_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    paint_white = model.material("paint_white", rgba=(0.90, 0.92, 0.94, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.62, 0.66, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.18, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.75, 0.78, 0.81, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.08, 0.09, 0.10, 1.0))

    dish_shell = _mesh(
        "main_reflector",
        LatheGeometry(
            [
                (0.00, 0.000),
                (0.08, 0.006),
                (0.18, 0.020),
                (0.34, 0.058),
                (0.48, 0.112),
                (0.56, 0.156),
                (0.60, 0.182),
                (0.592, 0.189),
                (0.565, 0.177),
                (0.47, 0.138),
                (0.32, 0.082),
                (0.16, 0.028),
                (0.04, 0.010),
            ],
            segments=72,
        ),
    )
    yoke_arm_profile = rounded_rect_profile(0.060, 0.115, radius=0.014, corner_segments=6)
    left_yoke_arm = _mesh(
        "left_yoke_arm",
        sweep_profile_along_spline(
            [
                (-0.015, 0.305, 0.030),
                (0.015, 0.305, 0.165),
                (0.095, 0.300, 0.325),
                (0.205, 0.300, 0.465),
            ],
            profile=yoke_arm_profile,
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    right_yoke_arm = _mesh(
        "right_yoke_arm",
        sweep_profile_along_spline(
            [
                (-0.015, -0.305, 0.030),
                (0.015, -0.305, 0.165),
                (0.095, -0.300, 0.325),
                (0.205, -0.300, 0.465),
            ],
            profile=yoke_arm_profile,
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    feed_strut = _mesh(
        "feed_support_strut",
        tube_from_spline_points(
            [
                (0.302, 0.000, 0.560),
                (0.390, 0.000, 0.430),
                (0.475, 0.000, 0.215),
                (0.535, 0.000, 0.032),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    feed_horn = _mesh(
        "feed_horn_body",
        ConeGeometry(radius=0.050, height=0.110, radial_segments=28, closed=True),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((0.68, 0.68, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=deck_gray,
        name="deck_flange",
    )
    pedestal_base.visual(
        Cylinder(radius=0.220, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=machinery_gray,
        name="pedestal_skirt",
    )
    pedestal_base.visual(
        Cylinder(radius=0.185, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=dark_metal,
        name="azimuth_bearing_cap",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.68, 0.68, 0.390)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
    )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        Cylinder(radius=0.240, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="turntable_platter",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.140, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=machinery_gray,
        name="turntable_riser",
    )
    azimuth_stage.visual(
        Box((0.180, 0.560, 0.240)),
        origin=Origin(xyz=(0.005, 0.0, 0.230)),
        material=machinery_gray,
        name="yoke_core",
    )
    azimuth_stage.visual(
        Box((0.190, 0.560, 0.090)),
        origin=Origin(xyz=(0.040, 0.0, 0.390)),
        material=machinery_gray,
        name="rear_crossbrace",
    )
    azimuth_stage.visual(left_yoke_arm, material=paint_white, name="left_yoke_arm")
    azimuth_stage.visual(right_yoke_arm, material=paint_white, name="right_yoke_arm")
    azimuth_stage.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.205, 0.270, 0.465), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion_boss",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.205, -0.270, 0.465), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion_boss",
    )
    azimuth_stage.inertial = Inertial.from_geometry(
        Box((0.52, 0.62, 0.55)),
        mass=180.0,
        origin=Origin(xyz=(0.03, 0.0, 0.200)),
    )

    antenna_head = model.part("antenna_head")
    antenna_head.visual(
        Cylinder(radius=0.030, length=0.470),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    antenna_head.visual(
        Cylinder(radius=0.055, length=0.320),
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="backbone_tube",
    )
    antenna_head.visual(
        Box((0.210, 0.045, 0.180)),
        origin=Origin(xyz=(0.110, 0.110, 0.0)),
        material=machinery_gray,
        name="left_cradle_plate",
    )
    antenna_head.visual(
        Box((0.210, 0.045, 0.180)),
        origin=Origin(xyz=(0.110, -0.110, 0.0)),
        material=machinery_gray,
        name="right_cradle_plate",
    )
    antenna_head.visual(
        Cylinder(radius=0.105, length=0.085),
        origin=Origin(xyz=(0.182, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="dish_hub",
    )
    antenna_head.visual(
        dish_shell,
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=paint_white,
        name="main_reflector",
    )
    antenna_head.visual(
        Cylinder(radius=0.034, length=0.120),
        origin=Origin(xyz=(0.560, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_support_core",
    )
    antenna_head.visual(
        feed_horn,
        origin=Origin(xyz=(0.675, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_black,
        name="feed_horn",
    )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        antenna_head.visual(
            feed_strut,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=aluminum,
            name=f"feed_strut_{index + 1}",
        )
    antenna_head.visual(
        Box((0.100, 0.080, 0.160)),
        origin=Origin(xyz=(0.030, 0.165, -0.010)),
        material=dark_metal,
        name="elevation_gearbox",
    )
    antenna_head.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.010, 0.190, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="elevation_motor",
    )
    antenna_head.inertial = Inertial.from_geometry(
        Box((0.820, 1.220, 1.220)),
        mass=120.0,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=antenna_head,
        origin=Origin(xyz=(0.205, 0.0, 0.465)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.8,
            lower=-0.35,
            upper=1.15,
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

    base = object_model.get_part("pedestal_base")
    stage = object_model.get_part("azimuth_stage")
    head = object_model.get_part("antenna_head")
    azimuth = object_model.get_articulation("azimuth_rotation")
    elevation = object_model.get_articulation("elevation_rotation")

    ctx.check("pedestal base exists", base is not None)
    ctx.check("azimuth stage exists", stage is not None)
    ctx.check("antenna head exists", head is not None)

    with ctx.pose({azimuth: 0.0, elevation: 0.0}):
        ctx.expect_contact(
            stage,
            base,
            elem_a="turntable_platter",
            elem_b="azimuth_bearing_cap",
            name="turntable seats on pedestal bearing cap",
        )
        ctx.expect_contact(
            head,
            stage,
            elem_a="trunnion_shaft",
            elem_b="left_trunnion_boss",
            name="left trunnion meets left yoke boss",
        )
        ctx.expect_contact(
            head,
            stage,
            elem_a="trunnion_shaft",
            elem_b="right_trunnion_boss",
            name="right trunnion meets right yoke boss",
        )

    horn_rest = None
    horn_raised = None
    with ctx.pose({azimuth: 0.0, elevation: 0.0}):
        horn_rest = _elem_center(ctx.part_element_world_aabb(head, elem="feed_horn"))
    with ctx.pose({azimuth: 0.0, elevation: 0.70}):
        horn_raised = _elem_center(ctx.part_element_world_aabb(head, elem="feed_horn"))
    ctx.check(
        "positive elevation raises feed horn",
        horn_rest is not None and horn_raised is not None and horn_raised[2] > horn_rest[2] + 0.10,
        details=f"rest={horn_rest}, raised={horn_raised}",
    )

    horn_az_0 = None
    horn_az_90 = None
    with ctx.pose({azimuth: 0.0, elevation: 0.20}):
        horn_az_0 = _elem_center(ctx.part_element_world_aabb(head, elem="feed_horn"))
    with ctx.pose({azimuth: math.pi / 2.0, elevation: 0.20}):
        horn_az_90 = _elem_center(ctx.part_element_world_aabb(head, elem="feed_horn"))
    ctx.check(
        "azimuth rotation swings antenna about vertical axis",
        horn_az_0 is not None
        and horn_az_90 is not None
        and horn_az_0[0] > 0.35
        and horn_az_90[1] > 0.35
        and abs(horn_az_90[0]) < abs(horn_az_0[0]),
        details=f"az0={horn_az_0}, az90={horn_az_90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
