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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_device_mount")

    tripod_black = model.material("tripod_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    matte_gray = model.material("matte_gray", rgba=(0.46, 0.47, 0.49, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.14, 0.18, 0.95))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.020)),
        material=tripod_black,
        name="crown_hub",
    )
    crown.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 1.053)),
        material=graphite,
        name="upper_collar",
    )
    crown.visual(
        Cylinder(radius=0.018, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        material=tripod_black,
        name="center_support",
    )
    crown.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.245)),
        material=graphite,
        name="pan_bearing_cap",
    )
    hinge_radius = 0.070
    hinge_z = 1.005
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        crown.visual(
            Box((0.028, 0.020, 0.010)),
            origin=Origin(
                xyz=(0.056 * c, 0.056 * s, 1.020),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"leg_mount_root_{index}",
        )
        crown.visual(
            Box((0.020, 0.004, 0.036)),
            origin=Origin(
                xyz=(hinge_radius * c - 0.012 * s, hinge_radius * s + 0.012 * c, hinge_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"leg_hinge_ear_outer_{index}",
        )
        crown.visual(
            Box((0.020, 0.004, 0.036)),
            origin=Origin(
                xyz=(hinge_radius * c + 0.012 * s, hinge_radius * s - 0.012 * c, hinge_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"leg_hinge_ear_inner_{index}",
        )
    crown.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.28)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 1.120)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=graphite,
        name="pan_stage",
    )
    head.visual(
        Box((0.060, 0.078, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=tripod_black,
        name="tilt_yoke_base",
    )
    head.visual(
        Box((0.018, 0.010, 0.090)),
        origin=Origin(xyz=(0.0, 0.036, 0.070)),
        material=tripod_black,
        name="left_yoke_arm",
    )
    head.visual(
        Box((0.018, 0.010, 0.090)),
        origin=Origin(xyz=(0.0, -0.036, 0.070)),
        material=tripod_black,
        name="right_yoke_arm",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.12)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    device_plate = model.part("device_plate")
    device_plate.visual(
        Cylinder(radius=0.006, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tilt_axle",
    )
    device_plate.visual(
        Box((0.030, 0.058, 0.022)),
        origin=Origin(xyz=(0.016, 0.0, 0.008)),
        material=tripod_black,
        name="quick_release_block",
    )
    device_plate.visual(
        Box((0.145, 0.085, 0.006)),
        origin=Origin(xyz=(0.080, 0.0, 0.021)),
        material=graphite,
        name="plate_surface",
    )
    device_plate.visual(
        Box((0.112, 0.068, 0.062)),
        origin=Origin(xyz=(0.085, 0.0, 0.055)),
        material=matte_gray,
        name="device_body",
    )
    device_plate.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.151, 0.0, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tripod_black,
        name="device_lens_barrel",
    )
    device_plate.visual(
        Box((0.072, 0.003, 0.040)),
        origin=Origin(xyz=(0.082, -0.034, 0.058)),
        material=glass,
        name="device_screen",
    )
    device_plate.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.12)),
        mass=1.3,
        origin=Origin(xyz=(0.090, 0.0, 0.050)),
    )

    model.articulation(
        "crown_to_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 1.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "head_to_device_plate",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=math.radians(-55.0),
            upper=math.radians(70.0),
        ),
    )

    deploy_angle = math.radians(22.0)
    leg_dx = math.sin(deploy_angle)
    leg_dz = math.cos(deploy_angle)
    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")

        def along_leg(distance: float) -> tuple[float, float, float]:
            return (leg_dx * distance, 0.0, -leg_dz * distance)

        leg.visual(
            Cylinder(radius=0.008, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.024, 0.018, 0.100)),
            origin=Origin(xyz=along_leg(0.050), rpy=(0.0, -deploy_angle, 0.0)),
            material=tripod_black,
            name="upper_knuckle",
        )
        leg.visual(
            Box((0.030, 0.020, 0.380)),
            origin=Origin(xyz=along_leg(0.290), rpy=(0.0, -deploy_angle, 0.0)),
            material=tripod_black,
            name="upper_leg",
        )
        leg.visual(
            Box((0.022, 0.016, 0.560)),
            origin=Origin(xyz=along_leg(0.760), rpy=(0.0, -deploy_angle, 0.0)),
            material=graphite,
            name="lower_leg",
        )
        leg.visual(
            Box((0.032, 0.024, 0.028)),
            origin=Origin(xyz=along_leg(0.480), rpy=(0.0, -deploy_angle, 0.0)),
            material=matte_gray,
            name="leg_clamp",
        )
        leg.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=along_leg(1.055)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.06, 0.04, 1.12)),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.0, -0.560)),
        )
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.5,
                lower=0.0,
                upper=math.radians(72.0),
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

    crown = object_model.get_part("crown")
    head = object_model.get_part("head")
    device_plate = object_model.get_part("device_plate")
    pan = object_model.get_articulation("crown_to_head")
    tilt = object_model.get_articulation("head_to_device_plate")

    ctx.expect_gap(
        head,
        crown,
        axis="z",
        positive_elem="pan_stage",
        negative_elem="pan_bearing_cap",
        max_gap=0.001,
        max_penetration=1e-5,
        name="pan stage sits directly on the center support bearing",
    )

    leg_foot_bottoms: list[float] = []
    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
        bottom_z = None if foot_aabb is None else foot_aabb[0][2]
        if bottom_z is not None:
            leg_foot_bottoms.append(bottom_z)
        ctx.check(
            f"leg {index} foot rests near the ground plane",
            bottom_z is not None and abs(bottom_z) <= 0.02,
            details=f"bottom_z={bottom_z}",
        )

    rest_plate_aabb = ctx.part_world_aabb(device_plate)
    rest_center = None
    if rest_plate_aabb is not None:
        rest_center = tuple((lo + hi) / 2.0 for lo, hi in zip(rest_plate_aabb[0], rest_plate_aabb[1]))

    pan_center = None
    with ctx.pose({pan: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(device_plate)
        if turned_aabb is not None:
            pan_center = tuple((lo + hi) / 2.0 for lo, hi in zip(turned_aabb[0], turned_aabb[1]))

    ctx.check(
        "pan joint swings the device around the vertical axis",
        rest_center is not None
        and pan_center is not None
        and rest_center[0] > 0.05
        and abs(rest_center[1]) < 0.03
        and pan_center[1] > 0.05
        and abs(pan_center[0]) < 0.03,
        details=f"rest_center={rest_center}, pan_center={pan_center}",
    )

    rest_top_z = None if rest_plate_aabb is None else rest_plate_aabb[1][2]
    tilted_top_z = None
    with ctx.pose({tilt: tilt.motion_limits.upper if tilt.motion_limits is not None else 0.0}):
        tilted_aabb = ctx.part_world_aabb(device_plate)
        if tilted_aabb is not None:
            tilted_top_z = tilted_aabb[1][2]

    ctx.check(
        "tilt joint raises the front of the mounted device",
        rest_top_z is not None and tilted_top_z is not None and tilted_top_z > rest_top_z + 0.05,
        details=f"rest_top_z={rest_top_z}, tilted_top_z={tilted_top_z}",
    )

    folded_foot_bottom = None
    with ctx.pose({"crown_to_leg_0": math.radians(72.0)}):
        folded_aabb = ctx.part_element_world_aabb("leg_0", elem="foot_pad")
        if folded_aabb is not None:
            folded_foot_bottom = folded_aabb[0][2]

    ctx.check(
        "one leg can fold upward toward the crown",
        bool(leg_foot_bottoms)
        and folded_foot_bottom is not None
        and folded_foot_bottom > min(leg_foot_bottoms) + 0.10,
        details=f"rest_bottoms={leg_foot_bottoms}, folded_foot_bottom={folded_foot_bottom}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
