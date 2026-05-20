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
    model = ArticulatedObject(name="tripod_pan_tilt_device")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.12, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    crown_z = 0.60
    crown_radius = 0.050
    hinge_z = crown_z + 0.010
    hinge_radius = 0.067
    deployed_leg_pitch = math.radians(65.0)

    base = model.part("base_module")
    base.visual(
        Cylinder(radius=crown_radius, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, crown_z + 0.0225)),
        material=dark_gray,
        name="crown_shell",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, crown_z + 0.060)),
        material=satin_silver,
        name="pan_pedestal",
    )
    for index, yaw_deg in enumerate((0.0, 120.0, 240.0), start=1):
        yaw = math.radians(yaw_deg)
        ux = math.cos(yaw)
        uy = math.sin(yaw)
        lug_center = (hinge_radius * ux, hinge_radius * uy, crown_z + 0.012)
        rib_center = (0.041 * ux, 0.041 * uy, crown_z + 0.012)
        tx = -uy
        ty = ux
        for side, sign in (("a", -1.0), ("b", 1.0)):
            base.visual(
                Box((0.040, 0.008, 0.028)),
                origin=Origin(
                    xyz=(
                        lug_center[0] + sign * 0.015 * tx,
                        lug_center[1] + sign * 0.015 * ty,
                        lug_center[2],
                    ),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark_gray,
                name=f"hinge_ear_{index}_{side}",
            )
        base.visual(
            Box((0.030, 0.018, 0.022)),
            origin=Origin(xyz=rib_center, rpy=(0.0, 0.0, yaw)),
            material=dark_gray,
            name=f"crown_rib_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.14, 0.14, 0.11)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, crown_z + 0.040)),
    )

    def add_leg(name: str, yaw_deg: float) -> None:
        leg = model.part(name)
        leg.visual(
            Cylinder(radius=0.009, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_silver,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.028, 0.016, 0.050)),
            origin=Origin(
                xyz=(0.003, 0.0, -0.024),
            ),
            material=satin_silver,
            name="hinge_shoulder",
        )
        leg.visual(
            Cylinder(radius=0.011, length=0.340),
            origin=Origin(
                xyz=(0.088, 0.0, -0.188),
                rpy=(0.0, math.pi / 2.0 + deployed_leg_pitch, 0.0),
            ),
            material=matte_black,
            name="upper_tube",
        )
        leg.visual(
            Cylinder(radius=0.0085, length=0.300),
            origin=Origin(
                xyz=(0.220, 0.0, -0.471),
                rpy=(0.0, math.pi / 2.0 + deployed_leg_pitch, 0.0),
            ),
            material=matte_black,
            name="lower_tube",
        )
        leg.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(0.287, 0.0, -0.616)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.31, 0.04, 0.62)),
            mass=0.55,
            origin=Origin(xyz=(0.16, 0.0, -0.29)),
        )

        yaw = math.radians(yaw_deg)
        model.articulation(
            f"base_to_{name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), hinge_z),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.4,
                lower=0.0,
                upper=math.radians(160.0),
            ),
        )

    add_leg("front_leg", 0.0)
    add_leg("left_leg", 120.0)
    add_leg("right_leg", 240.0)

    pan_axis_z = crown_z + 0.090

    bearing = model.part("bearing_module")
    bearing.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=satin_silver,
        name="lower_race",
    )
    bearing.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(),
        material=dark_gray,
        name="upper_bearing_cap",
    )
    bearing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.032),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )
    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing,
        origin=Origin(xyz=(0.0, 0.0, pan_axis_z)),
    )

    head = model.part("head_module")
    head.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_gray,
        name="pan_hub",
    )
    head.visual(
        Box((0.028, 0.104, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_gray,
        name="yoke_bridge",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        head.visual(
            Box((0.014, 0.018, 0.070)),
            origin=Origin(xyz=(0.0, sign * 0.045, 0.061)),
            material=dark_gray,
            name=f"tilt_arm_{side}",
        )
        head.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(
                xyz=(0.0, sign * 0.045, 0.060),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_silver,
            name=f"tilt_boss_{side}",
        )
    head.inertial = Inertial.from_geometry(
        Box((0.08, 0.11, 0.10)),
        mass=0.42,
        origin=Origin(xyz=(-0.005, 0.0, 0.048)),
    )
    model.articulation(
        "bearing_to_head_pan",
        ArticulationType.REVOLUTE,
        parent=bearing,
        child=head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.4,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )

    device_plate = model.part("device_plate")
    device_plate.visual(
        Box((0.022, 0.072, 0.032)),
        origin=Origin(xyz=(-0.010, 0.0, 0.008)),
        material=satin_silver,
        name="tilt_trunnion",
    )
    device_plate.visual(
        Box((0.110, 0.070, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, 0.020)),
        material=satin_silver,
        name="plate_top",
    )
    device_plate.visual(
        Box((0.010, 0.070, 0.016)),
        origin=Origin(xyz=(0.060, 0.0, 0.028)),
        material=satin_silver,
        name="front_lip",
    )
    device_plate.visual(
        Box((0.018, 0.050, 0.018)),
        origin=Origin(xyz=(-0.042, 0.0, 0.029)),
        material=dark_gray,
        name="rear_stop",
    )
    device_plate.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.05)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )
    model.articulation(
        "head_to_device_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=math.radians(-75.0),
            upper=math.radians(35.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_module")
    bearing = object_model.get_part("bearing_module")
    head = object_model.get_part("head_module")
    device_plate = object_model.get_part("device_plate")
    pan = object_model.get_articulation("bearing_to_head_pan")
    tilt = object_model.get_articulation("head_to_device_tilt")

    ctx.expect_gap(
        bearing,
        base,
        axis="z",
        positive_elem="lower_race",
        negative_elem="pan_pedestal",
        max_gap=0.0,
        max_penetration=0.0,
        name="bearing seats on the pan pedestal",
    )
    ctx.expect_gap(
        head,
        bearing,
        axis="z",
        positive_elem="pan_hub",
        negative_elem="upper_bearing_cap",
        max_gap=0.0,
        max_penetration=0.0,
        name="pan head sits on the bearing cap",
    )
    ctx.expect_within(
        device_plate,
        head,
        axes="y",
        inner_elem="tilt_trunnion",
        outer_elem="yoke_bridge",
        margin=0.0,
        name="tilt trunnion stays between the yoke cheeks",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) / 2.0 for axis in range(3))

    rest_front = aabb_center(ctx.part_element_world_aabb(device_plate, elem="front_lip"))
    with ctx.pose({pan: math.pi / 2.0}):
        panned_front = aabb_center(ctx.part_element_world_aabb(device_plate, elem="front_lip"))
    ctx.check(
        "pan joint swings the device plate around vertical",
        rest_front is not None
        and panned_front is not None
        and panned_front[1] > rest_front[1] + 0.045
        and abs(panned_front[0]) < abs(rest_front[0]),
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    with ctx.pose({tilt: math.radians(25.0)}):
        tilted_front = aabb_center(ctx.part_element_world_aabb(device_plate, elem="front_lip"))
    ctx.check(
        "tilt joint raises the front lip",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.020
        and tilted_front[0] < rest_front[0] - 0.005,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
