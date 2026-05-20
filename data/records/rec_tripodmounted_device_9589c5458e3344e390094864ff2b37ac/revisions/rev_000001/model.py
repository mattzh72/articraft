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
    model = ArticulatedObject(name="tripod_pan_tilt_mount")

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    grip_pad = model.material("grip_pad", rgba=(0.18, 0.18, 0.18, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.048, length=0.056),
        material=matte_black,
        name="crown_hub",
    )
    crown.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=anodized_black,
        name="pan_bearing",
    )

    leg_radius = 0.109
    hinge_z = -0.006
    for idx, azimuth_deg in enumerate((0.0, 120.0, 240.0), start=1):
        yaw = math.radians(azimuth_deg)
        arm_x = 0.066 * math.cos(yaw)
        arm_y = 0.066 * math.sin(yaw)
        crown.visual(
            Box((0.060, 0.034, 0.018)),
            origin=Origin(xyz=(arm_x, arm_y, hinge_z), rpy=(0.0, 0.0, yaw)),
            material=anodized_black,
            name=f"leg_{idx}_arm",
        )
        for side, y_off in (("inner", -0.014), ("outer", 0.014)):
            lug_x = leg_radius * math.cos(yaw) - y_off * math.sin(yaw)
            lug_y = leg_radius * math.sin(yaw) + y_off * math.cos(yaw)
            crown.visual(
                Box((0.026, 0.006, 0.028)),
                origin=Origin(xyz=(lug_x, lug_y, hinge_z), rpy=(0.0, 0.0, yaw)),
                material=anodized_black,
                name=f"leg_{idx}_{side}_lug",
            )

    crown.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.11)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=aluminum,
        name="pan_drum",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=aluminum,
        name="head_post",
    )
    head.visual(
        Box((0.040, 0.100, 0.020)),
        origin=Origin(xyz=(0.008, 0.0, 0.094)),
        material=aluminum,
        name="yoke_bridge",
    )
    for side, y_off in (("left", -0.043), ("right", 0.043)):
        head.visual(
            Box((0.018, 0.012, 0.074)),
            origin=Origin(xyz=(0.020, y_off, 0.067)),
            material=aluminum,
            name=f"{side}_yoke_arm",
        )
    head.inertial = Inertial.from_geometry(
        Box((0.10, 0.12, 0.12)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    device_plate = model.part("device_plate")
    device_plate.visual(
        Cylinder(radius=0.011, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="tilt_trunnion",
    )
    device_plate.visual(
        Box((0.018, 0.036, 0.020)),
        origin=Origin(xyz=(0.008, 0.0, 0.006)),
        material=matte_black,
        name="tilt_block",
    )
    device_plate.visual(
        Box((0.028, 0.032, 0.050)),
        origin=Origin(xyz=(0.026, 0.0, 0.025)),
        material=matte_black,
        name="plate_riser",
    )
    device_plate.visual(
        Box((0.115, 0.055, 0.008)),
        origin=Origin(xyz=(0.048, 0.0, 0.045)),
        material=aluminum,
        name="mount_plate",
    )
    device_plate.visual(
        Box((0.088, 0.040, 0.003)),
        origin=Origin(xyz=(0.048, 0.0, 0.0495)),
        material=grip_pad,
        name="plate_pad",
    )
    device_plate.inertial = Inertial.from_geometry(
        Box((0.13, 0.07, 0.07)),
        mass=0.28,
        origin=Origin(xyz=(0.018, 0.0, 0.030)),
    )

    leg_drop = 1.12
    leg_spread = 0.30
    leg_length = math.sqrt(leg_drop * leg_drop + leg_spread * leg_spread)
    leg_pitch = math.atan2(leg_spread, leg_drop)
    upper_leg_pitch = leg_pitch * 0.55
    lower_tube_length = leg_length - 0.085
    lower_tube_top_x = 0.030
    lower_tube_top_z = -0.060
    lower_tube_center_x = 0.5 * (lower_tube_top_x + leg_spread)
    lower_tube_center_z = 0.5 * (lower_tube_top_z - leg_drop)

    def add_leg(name: str, foot_name: str) -> None:
        leg = model.part(name)
        leg.visual(
            Cylinder(radius=0.010, length=0.022),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=anodized_black,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.022, 0.008, 0.112)),
            origin=Origin(xyz=(0.016, 0.0, -0.050), rpy=(0.0, -upper_leg_pitch, 0.0)),
            material=anodized_black,
            name="hinge_tongue",
        )
        leg.visual(
            Box((0.030, 0.020, 0.180)),
            origin=Origin(xyz=(0.040, 0.0, -0.142), rpy=(0.0, -(leg_pitch * 0.8), 0.0)),
            material=anodized_black,
            name="upper_shoulder",
        )
        leg.visual(
            Box((0.032, 0.024, lower_tube_length)),
            origin=Origin(
                xyz=(lower_tube_center_x, 0.0, lower_tube_center_z),
                rpy=(0.0, -leg_pitch, 0.0),
            ),
            material=anodized_black,
            name="leg_tube",
        )
        leg.visual(
            Box((0.046, 0.030, 0.090)),
            origin=Origin(
                xyz=(leg_spread - 0.010, 0.0, -leg_drop + 0.040),
                rpy=(0.0, -leg_pitch, 0.0),
            ),
            material=dark_rubber,
            name=foot_name,
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.08, 0.05, 1.18)),
            mass=0.62,
            origin=Origin(xyz=(leg_spread * 0.5, 0.0, -leg_drop * 0.5)),
        )

    add_leg("leg_1", "foot_pad")
    add_leg("leg_2", "foot_pad")
    add_leg("leg_3", "foot_pad")

    model.articulation(
        "crown_to_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )
    model.articulation(
        "head_to_device_plate",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device_plate,
        origin=Origin(xyz=(0.022, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=math.radians(-55.0),
            upper=math.radians(75.0),
        ),
    )

    for idx, azimuth_deg in enumerate((0.0, 120.0, 240.0), start=1):
        model.articulation(
            f"crown_to_leg_{idx}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=f"leg_{idx}",
            origin=Origin(
                xyz=(leg_radius * math.cos(math.radians(azimuth_deg)), leg_radius * math.sin(math.radians(azimuth_deg)), hinge_z),
                rpy=(0.0, 0.0, math.radians(azimuth_deg)),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.2,
                lower=0.0,
                upper=math.radians(38.0),
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
    leg_joints = [object_model.get_articulation(f"crown_to_leg_{idx}") for idx in (1, 2, 3)]
    legs = [object_model.get_part(f"leg_{idx}") for idx in (1, 2, 3)]

    ctx.expect_gap(
        head,
        crown,
        axis="z",
        max_gap=0.0025,
        max_penetration=0.0,
        name="pan head seats on crown",
    )

    crown_aabb = ctx.part_world_aabb(crown)
    foot_centers = []
    for idx, leg in enumerate(legs, start=1):
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
        if foot_aabb is not None:
            foot_centers.append(
                (
                    0.5 * (foot_aabb[0][0] + foot_aabb[1][0]),
                    0.5 * (foot_aabb[0][1] + foot_aabb[1][1]),
                    0.5 * (foot_aabb[0][2] + foot_aabb[1][2]),
                )
            )
        else:
            foot_centers.append(None)

    radial_distances = []
    min_foot_z = None
    for center in foot_centers:
        if center is None:
            continue
        radial_distances.append(math.hypot(center[0], center[1]))
        min_foot_z = center[2] if min_foot_z is None else min(min_foot_z, center[2])

    ctx.check(
        "tripod feet spread under crown",
        crown_aabb is not None
        and len(radial_distances) == 3
        and min(radial_distances) > 0.26
        and max(radial_distances) < 0.45
        and min_foot_z is not None
        and min_foot_z < crown_aabb[0][2] - 1.00,
        details=f"foot_centers={foot_centers}, crown_aabb={crown_aabb}",
    )

    plate_rest = ctx.part_element_world_aabb(device_plate, elem="mount_plate")
    with ctx.pose({pan: math.pi / 2.0}):
        plate_panned = ctx.part_element_world_aabb(device_plate, elem="mount_plate")
    def aabb_center(aabb):
        if aabb is None:
            return None
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    plate_rest_center = aabb_center(plate_rest)
    plate_panned_center = aabb_center(plate_panned)
    ctx.check(
        "pan rotates plate around vertical axis",
        plate_rest_center is not None
        and plate_panned_center is not None
        and plate_rest_center[0] > 0.01
        and abs(plate_panned_center[0]) < 0.01
        and plate_panned_center[1] > 0.01
        and abs(plate_panned_center[2] - plate_rest_center[2]) < 0.002,
        details=f"rest={plate_rest_center}, panned={plate_panned_center}",
    )

    plate_neutral = ctx.part_element_world_aabb(device_plate, elem="mount_plate")
    with ctx.pose({tilt: math.radians(35.0)}):
        plate_tilted = ctx.part_element_world_aabb(device_plate, elem="mount_plate")
    plate_neutral_center = aabb_center(plate_neutral)
    plate_tilted_center = aabb_center(plate_tilted)
    ctx.check(
        "tilt raises the forward plate edge",
        plate_neutral_center is not None
        and plate_tilted_center is not None
        and plate_tilted_center[2] > plate_neutral_center[2] + 0.005,
        details=f"neutral={plate_neutral_center}, tilted={plate_tilted_center}",
    )

    leg_1_foot_rest = ctx.part_element_world_aabb(legs[0], elem="foot_pad")
    with ctx.pose({leg_joints[0]: math.radians(30.0)}):
        leg_1_foot_folded = ctx.part_element_world_aabb(legs[0], elem="foot_pad")
    leg_rest_center = aabb_center(leg_1_foot_rest)
    leg_folded_center = aabb_center(leg_1_foot_folded)
    ctx.check(
        "leg folds inward toward the crown",
        leg_rest_center is not None
        and leg_folded_center is not None
        and math.hypot(leg_folded_center[0], leg_folded_center[1])
        < math.hypot(leg_rest_center[0], leg_rest_center[1]) - 0.10,
        details=f"rest={leg_rest_center}, folded={leg_folded_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
