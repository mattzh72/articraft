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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_with_compact_pan_tilt_head")

    crown_dark = model.material("crown_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    leg_black = model.material("leg_black", rgba=(0.12, 0.13, 0.14, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.10, 0.10, 0.11, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    camera_black = model.material("camera_black", rgba=(0.11, 0.11, 0.12, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.10, 0.15, 0.18, 0.75))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.046, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.904)),
        material=crown_dark,
        name="crown_hub",
    )
    crown.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.934)),
        material=aluminum,
        name="pan_bearing_seat",
    )
    crown.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.14)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
    )

    leg_splay = math.radians(23.0)
    leg_joint_radius = 0.125
    leg_joint_z = 0.918
    arm_radius = 0.079
    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        crown.visual(
            Box((0.092, 0.026, 0.024)),
            origin=Origin(
                xyz=(arm_radius * math.cos(yaw), arm_radius * math.sin(yaw), 0.898),
                rpy=(0.0, 0.0, yaw),
            ),
            material=crown_dark,
            name=f"leg_housing_{index}",
        )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=aluminum,
        name="pan_disc",
    )
    pan_stage.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=crown_dark,
        name="pan_neck",
    )
    pan_stage.visual(
        Box((0.036, 0.036, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=crown_dark,
        name="tilt_bridge",
    )
    pan_stage.visual(
        Box((0.040, 0.010, 0.050)),
        origin=Origin(xyz=(0.004, 0.021, 0.051)),
        material=crown_dark,
        name="left_yoke_cheek",
    )
    pan_stage.visual(
        Box((0.040, 0.010, 0.050)),
        origin=Origin(xyz=(0.004, -0.021, 0.051)),
        material=crown_dark,
        name="right_yoke_cheek",
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.08)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    model.articulation(
        "crown_to_pan",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.942)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5),
    )

    device_mount = model.part("device_mount")
    device_mount.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="tilt_trunnion",
    )
    device_mount.visual(
        Box((0.022, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=crown_dark,
        name="tilt_knuckle",
    )
    device_mount.visual(
        Box((0.068, 0.040, 0.006)),
        origin=Origin(xyz=(0.004, 0.0, 0.023)),
        material=aluminum,
        name="device_plate",
    )
    device_mount.visual(
        Box((0.118, 0.064, 0.072)),
        origin=Origin(xyz=(0.006, 0.0, 0.062)),
        material=camera_black,
        name="camera_body",
    )
    device_mount.visual(
        Box((0.044, 0.016, 0.020)),
        origin=Origin(xyz=(-0.026, 0.0, 0.108)),
        material=camera_black,
        name="viewfinder_hump",
    )
    device_mount.visual(
        Cylinder(radius=0.023, length=0.056),
        origin=Origin(xyz=(0.088, 0.0, 0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_black,
        name="lens_barrel",
    )
    device_mount.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.120, 0.0, 0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_dark,
        name="lens_front",
    )
    device_mount.inertial = Inertial.from_geometry(
        Box((0.18, 0.09, 0.13)),
        mass=1.1,
        origin=Origin(xyz=(0.020, 0.0, 0.060)),
    )

    model.articulation(
        "pan_to_device_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=device_mount,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.5,
            lower=math.radians(-55.0),
            upper=math.radians(75.0),
        ),
    )

    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Box((0.024, 0.020, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=aluminum,
            name="top_clamp",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.340),
            origin=Origin(xyz=(0.0, 0.0, -0.202)),
            material=leg_black,
            name="upper_tube",
        )
        leg.visual(
            Cylinder(radius=0.012, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.372)),
            material=clamp_black,
            name="upper_lock",
        )
        leg.visual(
            Cylinder(radius=0.008, length=0.320),
            origin=Origin(xyz=(0.0, 0.0, -0.524)),
            material=leg_black,
            name="mid_tube",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.684)),
            material=clamp_black,
            name="lower_lock",
        )
        leg.visual(
            Cylinder(radius=0.0065, length=0.290),
            origin=Origin(xyz=(0.0, 0.0, -0.834)),
            material=leg_black,
            name="lower_tube",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, -0.999)),
            material=clamp_black,
            name="rubber_foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.05, 0.05, 1.02)),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.0, -0.50)),
        )
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(
                    leg_joint_radius * math.cos(yaw),
                    leg_joint_radius * math.sin(yaw),
                    leg_joint_z,
                ),
                rpy=(0.0, -leg_splay, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.4,
                lower=math.radians(-8.0),
                upper=math.radians(82.0),
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
    pan_stage = object_model.get_part("pan_stage")
    device_mount = object_model.get_part("device_mount")
    leg_0 = object_model.get_part("leg_0")
    pan_joint = object_model.get_articulation("crown_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_device_tilt")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    for index in range(3):
        ctx.allow_overlap(
            crown,
            object_model.get_part(f"leg_{index}"),
            elem_a=f"leg_housing_{index}",
            elem_b="top_clamp",
            reason="The fold hinge is simplified as a compact clamp nested into the crown housing.",
        )

    ctx.expect_gap(
        pan_stage,
        crown,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0002,
        positive_elem="pan_disc",
        negative_elem="pan_bearing_seat",
        name="pan stage stays compact above crown",
    )
    ctx.expect_origin_distance(
        device_mount,
        crown,
        axes="xy",
        max_dist=0.02,
        name="device stays centered over the support",
    )

    lens_rest = ctx.part_element_world_aabb(device_mount, elem="lens_front")
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper, pan_joint: 0.85}):
        lens_tilted = ctx.part_element_world_aabb(device_mount, elem="lens_front")
    rest_center_z = None if lens_rest is None else (lens_rest[0][2] + lens_rest[1][2]) * 0.5
    tilted_center_z = None if lens_tilted is None else (lens_tilted[0][2] + lens_tilted[1][2]) * 0.5
    ctx.check(
        "positive tilt raises the lens",
        rest_center_z is not None
        and tilted_center_z is not None
        and tilted_center_z > rest_center_z + 0.05,
        details=f"rest_z={rest_center_z}, tilted_z={tilted_center_z}",
    )

    foot_rest = ctx.part_element_world_aabb(leg_0, elem="rubber_foot")
    with ctx.pose({leg_joint: leg_joint.motion_limits.upper}):
        foot_folded = ctx.part_element_world_aabb(leg_0, elem="rubber_foot")
    foot_rest_z = None if foot_rest is None else (foot_rest[0][2] + foot_rest[1][2]) * 0.5
    foot_folded_z = None if foot_folded is None else (foot_folded[0][2] + foot_folded[1][2]) * 0.5
    ctx.check(
        "leg folds upward toward the crown",
        foot_rest_z is not None
        and foot_folded_z is not None
        and foot_folded_z > foot_rest_z + 0.35,
        details=f"rest_z={foot_rest_z}, folded_z={foot_folded_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
