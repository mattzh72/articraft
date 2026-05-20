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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swan_neck_wall_mount_cctv")

    housing_white = model.material("housing_white", rgba=(0.87, 0.89, 0.90, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.46, 0.49, 0.52, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    smoked_dome = model.material("smoked_dome", rgba=(0.18, 0.22, 0.26, 0.38))
    lens_black = model.material("lens_black", rgba=(0.06, 0.07, 0.08, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.23, 0.33, 0.40, 0.55))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.012, 0.100, 0.180)),
        origin=Origin(xyz=(0.006, 0.0, 0.090)),
        material=housing_white,
        name="plate",
    )
    wall_plate.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.019, 0.0, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_white,
        name="boss",
    )
    wall_plate.visual(
        Box((0.018, 0.040, 0.070)),
        origin=Origin(xyz=(0.015, 0.0, 0.100)),
        material=trim_gray,
        name="boss_reinforcement",
    )
    for index, (y_pos, z_pos) in enumerate(((-0.032, 0.032), (0.032, 0.032), (-0.032, 0.148), (0.032, 0.148))):
        wall_plate.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.010, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener_steel,
            name=f"anchor_{index}",
        )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.040, 0.100, 0.180)),
        mass=1.8,
        origin=Origin(xyz=(0.020, 0.0, 0.090)),
    )

    gooseneck_arm = model.part("gooseneck_arm")
    arm_curve = tube_from_spline_points(
        [
            (0.010, 0.0, 0.000),
            (0.055, 0.0, 0.020),
            (0.125, 0.0, 0.056),
            (0.220, 0.0, 0.022),
            (0.294, 0.0, -0.018),
            (0.328, 0.0, -0.039),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=18,
    )
    gooseneck_arm.visual(
        _mesh("gooseneck_arm_tube", arm_curve),
        material=housing_white,
        name="arm_tube",
    )
    gooseneck_arm.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_white,
        name="mount_collar",
    )
    gooseneck_arm.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.328, 0.0, -0.042)),
        material=housing_white,
        name="tip_pad",
    )
    gooseneck_arm.inertial = Inertial.from_geometry(
        Box((0.350, 0.040, 0.140)),
        mass=1.3,
        origin=Origin(xyz=(0.175, 0.0, -0.010)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=trim_gray,
        name="pan_base",
    )
    pan_yoke.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=trim_gray,
        name="pan_stem",
    )
    pan_yoke.visual(
        Box((0.044, 0.120, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, -0.037)),
        material=trim_gray,
        name="yoke_bridge",
    )
    pan_yoke.visual(
        Box((0.012, 0.010, 0.058)),
        origin=Origin(xyz=(0.018, 0.055, -0.061)),
        material=trim_gray,
        name="left_yoke_arm",
    )
    pan_yoke.visual(
        Box((0.012, 0.010, 0.058)),
        origin=Origin(xyz=(0.018, -0.055, -0.061)),
        material=trim_gray,
        name="right_yoke_arm",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.060, 0.120, 0.100)),
        mass=0.75,
        origin=Origin(xyz=(0.015, 0.0, -0.050)),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="tilt_axle",
    )
    camera_head.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.020, 0.0, -0.012)),
        material=housing_white,
        name="housing_cap",
    )
    camera_head.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.022, 0.0, -0.030)),
        material=trim_gray,
        name="trim_ring",
    )
    camera_head.visual(
        Sphere(radius=0.044),
        origin=Origin(xyz=(0.022, 0.0, -0.052)),
        material=smoked_dome,
        name="dome_bubble",
    )
    camera_head.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.058, 0.0, -0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_barrel",
    )
    camera_head.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(xyz=(0.070, 0.0, -0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_glass",
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.120, 0.100, 0.110)),
        mass=1.1,
        origin=Origin(xyz=(0.028, 0.0, -0.035)),
    )

    model.articulation(
        "wall_to_arm",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=gooseneck_arm,
        origin=Origin(xyz=(0.032, 0.0, 0.100)),
    )
    model.articulation(
        "arm_to_pan",
        ArticulationType.REVOLUTE,
        parent=gooseneck_arm,
        child=pan_yoke,
        origin=Origin(xyz=(0.328, 0.0, -0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_head,
        origin=Origin(xyz=(0.018, 0.0, -0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.35,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    gooseneck_arm = object_model.get_part("gooseneck_arm")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_head = object_model.get_part("camera_head")

    arm_to_pan = object_model.get_articulation("arm_to_pan")
    pan_to_camera = object_model.get_articulation("pan_to_camera")

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

    ctx.expect_contact(gooseneck_arm, wall_plate)
    ctx.expect_contact(pan_yoke, gooseneck_arm)
    ctx.expect_contact(camera_head, pan_yoke)
    ctx.expect_origin_gap(camera_head, wall_plate, axis="x", min_gap=0.34)

    ctx.check(
        "pan axis is vertical",
        arm_to_pan.axis == (0.0, 0.0, 1.0),
        f"Expected pan axis (0, 0, 1), got {arm_to_pan.axis}",
    )
    ctx.check(
        "tilt axis is horizontal",
        pan_to_camera.axis == (0.0, 1.0, 0.0),
        f"Expected tilt axis (0, 1, 0), got {pan_to_camera.axis}",
    )
    ctx.check(
        "pan limits are wide sweep",
        arm_to_pan.motion_limits is not None
        and arm_to_pan.motion_limits.lower is not None
        and arm_to_pan.motion_limits.upper is not None
        and arm_to_pan.motion_limits.lower < -2.9
        and arm_to_pan.motion_limits.upper > 2.9,
        "Pan articulation should allow roughly ±170° sweep.",
    )
    ctx.check(
        "tilt favors downward viewing",
        pan_to_camera.motion_limits is not None
        and pan_to_camera.motion_limits.lower is not None
        and pan_to_camera.motion_limits.upper is not None
        and pan_to_camera.motion_limits.lower <= -0.3
        and pan_to_camera.motion_limits.upper >= 1.0,
        "Tilt articulation should allow modest up-tilt and strong down-tilt.",
    )

    camera_rest = ctx.part_world_position(camera_head)
    with ctx.pose({arm_to_pan: 1.20}):
        camera_panned = ctx.part_world_position(camera_head)
        ctx.expect_contact(pan_yoke, gooseneck_arm)
    ctx.check(
        "pan swings camera laterally",
        camera_rest is not None and camera_panned is not None and camera_panned[1] > camera_rest[1] + 0.012,
        f"Expected positive Y sweep under pan; rest={camera_rest}, panned={camera_panned}",
    )

    lens_rest = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
    with ctx.pose({pan_to_camera: 0.80}):
        lens_tilted = ctx.part_element_world_aabb(camera_head, elem="lens_barrel")
        ctx.expect_contact(camera_head, pan_yoke)
    ctx.check(
        "tilt drives lens downward",
        lens_rest is not None and lens_tilted is not None and lens_tilted[0][2] < lens_rest[0][2] - 0.010,
        f"Expected lower lens AABB after tilt; rest={lens_rest}, tilted={lens_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
