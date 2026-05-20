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
    model = ArticulatedObject(name="tripod_device_mount")

    dark_anodized = model.material("dark_anodized", rgba=(0.16, 0.17, 0.18, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    device_body = model.material("device_body", rgba=(0.21, 0.22, 0.24, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.14, 0.18, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.055, length=0.060),
        material=dark_anodized,
        name="crown_hub",
    )
    crown.visual(
        Cylinder(radius=0.032, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_anodized,
        name="center_column",
    )
    crown.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=plate_gray,
        name="top_bearing_plate",
    )
    for leg_index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        crown.visual(
            Box((0.090, 0.036, 0.010)),
            origin=Origin(
                xyz=(0.076 * math.cos(yaw), 0.076 * math.sin(yaw), 0.01076),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_anodized,
            name=f"leg_{leg_index}_hinge_bridge",
        )
        for side, y_local in (("left", 0.020), ("right", -0.020)):
            crown.visual(
                Box((0.018, 0.008, 0.034)),
                origin=Origin(
                    xyz=(
                        0.120 * math.cos(yaw) - y_local * math.sin(yaw),
                        0.120 * math.sin(yaw) + y_local * math.cos(yaw),
                        -0.008,
                    ),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark_anodized,
                name=f"leg_{leg_index}_{side}_hinge_lug",
            )
    crown.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.24)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    leg_length = 0.82
    leg_spread_angle = math.radians(35.0)
    leg_pitch = math.pi - leg_spread_angle
    fold_limit = math.radians(26.0)
    hinge_radius = 0.098

    for leg_index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        dir_x = math.sin(leg_spread_angle)
        dir_z = -math.cos(leg_spread_angle)

        def along_leg(distance: float) -> tuple[float, float, float]:
            return (dir_x * distance, 0.0, dir_z * distance)

        leg = model.part(f"leg_{leg_index}")
        leg.visual(
            Cylinder(radius=0.011, length=0.026),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.028, 0.030, 0.078)),
            origin=Origin(
                xyz=along_leg(0.032),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=matte_black,
            name="upper_knuckle",
        )
        leg.visual(
            Cylinder(radius=0.016, length=0.430),
            origin=Origin(
                xyz=along_leg(0.270),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=dark_anodized,
            name="main_tube",
        )
        leg.visual(
            Cylinder(radius=0.012, length=0.290),
            origin=Origin(
                xyz=along_leg(0.600),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=plate_gray,
            name="lower_stage",
        )
        leg.visual(
            Cylinder(radius=0.019, length=0.070),
            origin=Origin(
                xyz=along_leg(0.730),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.08, 0.06, leg_length)),
            mass=0.55,
            origin=Origin(
                xyz=(
                    math.sin(leg_spread_angle) * leg_length * 0.5,
                    0.0,
                    -math.cos(leg_spread_angle) * leg_length * 0.5,
                ),
            ),
        )

        model.articulation(
            f"crown_to_leg_{leg_index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), -0.008),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.0,
                lower=0.0,
                upper=fold_limit,
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=matte_black,
        name="pan_base",
    )
    pan_head.visual(
        Box((0.030, 0.090, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=matte_black,
        name="yoke_base_block",
    )
    for side, y in (("left", 0.048), ("right", -0.048)):
        pan_head.visual(
            Box((0.020, 0.016, 0.104)),
            origin=Origin(xyz=(0.0, y, 0.087)),
            material=matte_black,
            name=f"{side}_yoke_arm",
        )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.11, 0.12, 0.15)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    device_plate = model.part("device_plate")
    for side, y in (("left", 0.033), ("right", -0.033)):
        device_plate.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=plate_gray,
            name=f"{side}_tilt_stub",
        )
    device_plate.visual(
        Box((0.024, 0.070, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=plate_gray,
        name="tilt_carriage",
    )
    device_plate.visual(
        Box((0.120, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=plate_gray,
        name="quick_release_plate",
    )
    device_plate.visual(
        Box((0.098, 0.060, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=device_body,
        name="device_body",
    )
    device_plate.visual(
        Cylinder(radius=0.019, length=0.020),
        origin=Origin(xyz=(0.058, 0.0, 0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="device_lens",
    )
    device_plate.visual(
        Box((0.040, 0.030, 0.004)),
        origin=Origin(xyz=(-0.010, 0.0, 0.098)),
        material=glass,
        name="screen",
    )
    device_plate.inertial = Inertial.from_geometry(
        Box((0.14, 0.08, 0.12)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4),
    )
    model.articulation(
        "pan_head_to_device_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=-0.75,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    device_plate = object_model.get_part("device_plate")
    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_device_plate")
    crown_aabb = ctx.part_world_aabb(crown)

    ctx.expect_origin_distance(
        pan_head,
        crown,
        axes="xy",
        max_dist=0.001,
        name="pan head stays centered on tripod axis",
    )
    ctx.expect_origin_distance(
        device_plate,
        crown,
        axes="xy",
        max_dist=0.001,
        name="tilt member stays centered on tripod axis",
    )

    with ctx.pose({pan_joint: math.radians(70.0), tilt_joint: 0.55}):
        ctx.expect_origin_distance(
            pan_head,
            crown,
            axes="xy",
            max_dist=0.001,
            name="pan rotation remains on centerline",
        )
        ctx.expect_origin_distance(
            device_plate,
            crown,
            axes="xy",
            max_dist=0.001,
            name="tilted device still rotates about the support axis",
        )

    for leg_index in (1, 2, 3):
        leg = object_model.get_part(f"leg_{leg_index}")
        leg_joint = object_model.get_articulation(f"crown_to_leg_{leg_index}")
        rest_foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
        with ctx.pose({leg_joint: math.radians(22.0)}):
            folded_foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")

        rest_ok = (
            crown_aabb is not None
            and rest_foot_aabb is not None
            and rest_foot_aabb[1][2] < crown_aabb[0][2] - 0.45
        )
        ctx.check(
            f"leg {leg_index} reaches down to tripod stance",
            rest_ok,
            details=f"crown_aabb={crown_aabb}, foot_aabb={rest_foot_aabb}",
        )

        fold_ok = (
            rest_foot_aabb is not None
            and folded_foot_aabb is not None
            and folded_foot_aabb[1][2] > rest_foot_aabb[1][2] + 0.10
        )
        ctx.check(
            f"leg {leg_index} folds upward at the crown",
            fold_ok,
            details=f"rest={rest_foot_aabb}, folded={folded_foot_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
