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

    painted_metal = model.material("painted_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.33, 0.35, 0.38, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    device_gray = model.material("device_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=painted_metal,
        name="crown_hub",
    )
    for index, yaw_deg in enumerate((0.0, 120.0, 240.0), start=1):
        yaw = math.radians(yaw_deg)
        c = math.cos(yaw)
        s = math.sin(yaw)
        crown.visual(
            Box((0.016, 0.020, 0.010)),
            origin=Origin(xyz=(0.048 * c, 0.048 * s, -0.001), rpy=(0.0, 0.0, yaw)),
            material=painted_metal,
            name=f"leg_hinge_bracket_{index}",
        )
        crown.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(
                xyz=(0.051 * c, 0.051 * s, -0.004),
                rpy=(math.pi / 2.0, 0.0, yaw),
            ),
            material=matte_black,
            name=f"leg_hinge_boss_{index}",
        )
    crown.visual(
        Cylinder(radius=0.030, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=anodized_aluminum,
        name="support_column",
    )
    crown.visual(
        Cylinder(radius=0.042, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.169)),
        material=matte_black,
        name="pan_bearing_cap",
    )
    crown.inertial = Inertial.from_geometry(
        Box((0.130, 0.130, 0.190)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    leg_length = 0.660
    upper_leg_length = 0.360
    lower_leg_length = 0.240

    for index, yaw_deg in enumerate((0.0, 120.0, 240.0), start=1):
        yaw = math.radians(yaw_deg)
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.042, 0.024, 0.020)),
            origin=Origin(xyz=(0.024, 0.0, 0.0)),
            material=painted_metal,
            name="upper_yoke",
        )
        leg.visual(
            Cylinder(radius=0.012, length=upper_leg_length),
            origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=anodized_aluminum,
            name="upper_tube",
        )
        leg.visual(
            Box((0.020, 0.030, 0.030)),
            origin=Origin(xyz=(0.390, 0.0, 0.0)),
            material=matte_black,
            name="leg_lock",
        )
        leg.visual(
            Cylinder(radius=0.009, length=lower_leg_length),
            origin=Origin(xyz=(0.500, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=anodized_aluminum,
            name="lower_tube",
        )
        leg.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(0.620, 0.0, 0.0)),
            material=rubber,
            name="foot_tip",
        )
        leg.inertial = Inertial.from_geometry(
            Box((leg_length, 0.050, 0.050)),
            mass=0.55,
            origin=Origin(xyz=(leg_length / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(0.065 * math.cos(yaw), 0.065 * math.sin(yaw), -0.004),
                rpy=(0.0, math.radians(57.0), yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.2,
                lower=0.0,
                upper=1.18,
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=matte_black,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.022, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=painted_metal,
        name="head_post",
    )
    pan_head.visual(
        Box((0.102, 0.060, 0.028)),
        origin=Origin(xyz=(0.056, 0.0, 0.128)),
        material=painted_metal,
        name="offset_arm",
    )
    pan_head.visual(
        Box((0.020, 0.010, 0.074)),
        origin=Origin(xyz=(0.108, 0.030, 0.128)),
        material=painted_metal,
        name="left_yoke_cheek",
    )
    pan_head.visual(
        Box((0.020, 0.010, 0.074)),
        origin=Origin(xyz=(0.108, -0.030, 0.128)),
        material=painted_metal,
        name="right_yoke_cheek",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.140, 0.090, 0.165)),
        mass=0.9,
        origin=Origin(xyz=(0.060, 0.0, 0.085)),
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.5),
    )

    device_plate = model.part("device_plate")
    device_plate.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="tilt_trunnion",
    )
    device_plate.visual(
        Box((0.020, 0.058, 0.052)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=painted_metal,
        name="tilt_block",
    )
    device_plate.visual(
        Box((0.010, 0.104, 0.148)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=anodized_aluminum,
        name="quick_release_plate",
    )
    device_plate.visual(
        Box((0.050, 0.110, 0.082)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=device_gray,
        name="device_body",
    )
    device_plate.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.077, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lens_housing",
    )
    device_plate.inertial = Inertial.from_geometry(
        Box((0.110, 0.120, 0.160)),
        mass=0.7,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "pan_head_to_device_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device_plate,
        origin=Origin(xyz=(0.114, 0.0, 0.128)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-0.60,
            upper=1.00,
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
    device_plate = object_model.get_part("device_plate")
    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_device_plate")
    leg_joint = object_model.get_articulation("crown_to_leg_1")

    ctx.expect_gap(
        device_plate,
        crown,
        axis="z",
        min_gap=0.030,
        name="device stays above the tripod crown",
    )

    tilt_axis_offset = math.hypot(tilt_joint.origin.xyz[0], tilt_joint.origin.xyz[1])
    ctx.check(
        "tilt axis is offset forward from the support column",
        tilt_axis_offset >= 0.10,
        details=f"tilt axis offset={tilt_axis_offset:.4f} m",
    )

    ctx.check(
        "pan axis is vertical and tilt axis is horizontal",
        pan_joint.axis == (0.0, 0.0, 1.0)
        and abs(tilt_joint.axis[0]) < 1e-9
        and abs(abs(tilt_joint.axis[1]) - 1.0) < 1e-9
        and abs(tilt_joint.axis[2]) < 1e-9,
        details=f"pan axis={pan_joint.axis}, tilt axis={tilt_joint.axis}",
    )

    crown_aabb = ctx.part_world_aabb(crown)
    for leg_name in ("leg_1", "leg_2", "leg_3"):
        leg_aabb = ctx.part_world_aabb(leg_name)
        ctx.check(
            f"{leg_name} reaches well below the crown",
            crown_aabb is not None
            and leg_aabb is not None
            and leg_aabb[0][2] < crown_aabb[0][2] - 0.45,
            details=f"crown_aabb={crown_aabb}, leg_aabb={leg_aabb}",
        )

    rest_device_pos = ctx.part_world_position(device_plate)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        turned_device_pos = ctx.part_world_position(device_plate)
    ctx.check(
        "panning swings the offset device around the column",
        rest_device_pos is not None
        and turned_device_pos is not None
        and rest_device_pos[0] > 0.08
        and abs(rest_device_pos[1]) < 0.03
        and turned_device_pos[1] > 0.08
        and abs(turned_device_pos[0]) < 0.03,
        details=f"rest={rest_device_pos}, turned={turned_device_pos}",
    )

    leg_rest_aabb = ctx.part_world_aabb("leg_1")
    with ctx.pose({leg_joint: 1.0}):
        leg_folded_aabb = ctx.part_world_aabb("leg_1")
    ctx.check(
        "one leg folds upward at the crown hinge",
        leg_rest_aabb is not None
        and leg_folded_aabb is not None
        and leg_folded_aabb[0][2] > leg_rest_aabb[0][2] + 0.20,
        details=f"rest={leg_rest_aabb}, folded={leg_folded_aabb}",
    )

    lens_rest = ctx.part_element_world_aabb("device_plate", elem="lens_housing")
    with ctx.pose({tilt_joint: math.radians(35.0)}):
        lens_tilted = ctx.part_element_world_aabb("device_plate", elem="lens_housing")
    ctx.check(
        "positive tilt raises the device nose",
        lens_rest is not None
        and lens_tilted is not None
        and 0.5 * (lens_tilted[0][2] + lens_tilted[1][2])
        > 0.5 * (lens_rest[0][2] + lens_rest[1][2]) + 0.02,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
