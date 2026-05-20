from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    frame_paint = model.material("frame_paint", rgba=(0.28, 0.31, 0.34, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.18, 0.20, 0.22, 1.0))
    accent = model.material("accent", rgba=(0.19, 0.49, 0.77, 1.0))

    wheel_radius = 0.10
    tire_width = 0.042
    hub_width = 0.046
    wheel_y = 0.155

    frame = model.part("frame")
    frame.visual(
        Box((0.44, 0.14, 0.028)),
        origin=Origin(xyz=(-0.02, 0.0, 0.144)),
        material=frame_paint,
        name="deck",
    )
    frame.visual(
        Box((0.38, 0.12, 0.006)),
        origin=Origin(xyz=(-0.02, 0.0, 0.161)),
        material=satin_black,
        name="deck_grip",
    )
    frame.visual(
        Box((0.09, 0.08, 0.16)),
        origin=Origin(xyz=(0.17, 0.0, 0.159)),
        material=frame_paint,
        name="steering_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.05, length=0.022),
        origin=Origin(xyz=(0.20, 0.0, 0.228)),
        material=satin_black,
        name="steering_turntable",
    )
    frame.visual(
        Box((0.12, 0.10, 0.05)),
        origin=Origin(xyz=(0.11, 0.0, 0.155)),
        material=frame_paint,
        name="front_gusset",
    )
    frame.visual(
        Box((0.18, 0.09, 0.032)),
        origin=Origin(xyz=(-0.17, 0.085, 0.126)),
        material=frame_paint,
        name="rear_left_arm",
    )
    frame.visual(
        Box((0.18, 0.09, 0.032)),
        origin=Origin(xyz=(-0.17, -0.085, 0.126)),
        material=frame_paint,
        name="rear_right_arm",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(-0.26, 0.121, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="rear_left_stub",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(-0.26, -0.121, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="rear_right_stub",
    )
    frame.visual(
        Box((0.07, 0.05, 0.30)),
        origin=Origin(xyz=(-0.04, 0.045, 0.305)),
        material=frame_paint,
        name="knee_support_mast",
    )
    frame.visual(
        Box((0.10, 0.10, 0.03)),
        origin=Origin(xyz=(0.01, 0.085, 0.405)),
        material=frame_paint,
        name="tilt_bracket_arm",
    )
    frame.visual(
        Box((0.16, 0.014, 0.14)),
        origin=Origin(xyz=(0.02, 0.117, 0.455)),
        material=frame_paint,
        name="tilt_bracket_plate",
    )
    frame.visual(
        Box((0.06, 0.05, 0.10)),
        origin=Origin(xyz=(-0.03, 0.045, 0.395)),
        material=frame_paint,
        name="tilt_bracket_gusset",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.34, 0.60)),
        mass=8.5,
        origin=Origin(xyz=(0.00, 0.0, 0.30)),
    )

    steering = model.part("steering_assembly")
    steering.visual(
        Cylinder(radius=0.047, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_black,
        name="steering_collar",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=frame_paint,
        name="steering_stem",
    )
    steering.visual(
        Cylinder(radius=0.014, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.72), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="handlebar",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.09),
        origin=Origin(xyz=(0.0, 0.245, 0.72), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.09),
        origin=Origin(xyz=(0.0, -0.245, 0.72), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    steering.visual(
        Box((0.12, 0.23, 0.07)),
        origin=Origin(xyz=(0.05, 0.0, 0.055)),
        material=frame_paint,
        name="fork_crown",
    )
    steering.visual(
        Box((0.04, 0.028, 0.17)),
        origin=Origin(xyz=(0.085, wheel_y - 0.055, -0.055)),
        material=frame_paint,
        name="left_fork_leg",
    )
    steering.visual(
        Box((0.04, 0.028, 0.17)),
        origin=Origin(xyz=(0.085, -(wheel_y - 0.055), -0.055)),
        material=frame_paint,
        name="right_fork_leg",
    )
    steering.visual(
        Cylinder(radius=0.015, length=0.264),
        origin=Origin(xyz=(0.095, 0.0, -0.14), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="front_axle",
    )
    steering.visual(
        Box((0.08, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=accent,
        name="handlebar_clamp",
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.35, 0.55, 0.92)),
        mass=3.2,
        origin=Origin(xyz=(0.04, 0.0, 0.34)),
    )

    def add_wheel(part_name: str) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=wheel_radius, length=tire_width),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.074, length=0.038),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_metal,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.048, length=hub_width),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=hub_width + 0.004),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_metal,
            name="axle_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=tire_width),
            mass=0.85,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )

    add_wheel("front_left_wheel")
    add_wheel("front_right_wheel")
    add_wheel("rear_left_wheel")
    add_wheel("rear_right_wheel")

    knee_platform = model.part("knee_platform")
    knee_platform.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="pivot_boss",
    )
    knee_platform.visual(
        Box((0.04, 0.018, 0.10)),
        origin=Origin(xyz=(-0.006, -0.027, 0.050)),
        material=frame_paint,
        name="pivot_ear",
    )
    knee_platform.visual(
        Box((0.17, 0.034, 0.035)),
        origin=Origin(xyz=(0.015, -0.050, 0.060)),
        material=frame_paint,
        name="support_arm",
    )
    knee_platform.visual(
        Box((0.08, 0.15, 0.026)),
        origin=Origin(xyz=(-0.045, -0.110, 0.064)),
        material=frame_paint,
        name="rear_support_block",
    )
    knee_platform.visual(
        Box((0.25, 0.18, 0.014)),
        origin=Origin(xyz=(0.000, -0.100, 0.084)),
        material=frame_paint,
        name="underside_plate",
    )
    knee_platform.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(rounded_rect_profile(0.28, 0.19, 0.025), 0.014),
            "knee_pad_base_mesh",
        ),
        origin=Origin(xyz=(0.000, -0.100, 0.098)),
        material=satin_black,
        name="pad_base",
    )
    knee_platform.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(rounded_rect_profile(0.28, 0.19, 0.038), 0.048),
            "knee_pad_cushion_mesh",
        ),
        origin=Origin(xyz=(0.000, -0.100, 0.129)),
        material=pad_vinyl,
        name="pad_cushion",
    )
    knee_platform.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(rounded_rect_profile(0.07, 0.19, 0.022), 0.022),
            "knee_pad_bolster_mesh",
        ),
        origin=Origin(xyz=(0.105, -0.100, 0.142)),
        material=pad_vinyl,
        name="front_pad_bolster",
    )
    knee_platform.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.17)),
        mass=1.1,
        origin=Origin(xyz=(0.015, -0.090, 0.090)),
    )

    steering_yaw = model.articulation(
        "frame_to_steering",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering,
        origin=Origin(xyz=(0.20, 0.0, 0.239)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.65, upper=0.65),
    )
    front_left_spin = model.articulation(
        "steering_to_front_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child="front_left_wheel",
        origin=Origin(xyz=(0.095, wheel_y, -0.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=30.0),
    )
    front_right_spin = model.articulation(
        "steering_to_front_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child="front_right_wheel",
        origin=Origin(xyz=(0.095, -wheel_y, -0.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=30.0),
    )
    rear_left_spin = model.articulation(
        "frame_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child="rear_left_wheel",
        origin=Origin(xyz=(-0.26, wheel_y, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=30.0),
    )
    rear_right_spin = model.articulation(
        "frame_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child="rear_right_wheel",
        origin=Origin(xyz=(-0.26, -wheel_y, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=30.0),
    )
    knee_tilt = model.articulation(
        "frame_to_knee_platform",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=knee_platform,
        origin=Origin(xyz=(0.02, 0.110, 0.455)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.20, upper=0.38),
    )

    model.meta["key_joints"] = {
        "steering": steering_yaw.name,
        "front_wheels": (front_left_spin.name, front_right_spin.name),
        "rear_wheels": (rear_left_spin.name, rear_right_spin.name),
        "knee_tilt": knee_tilt.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    steering = object_model.get_part("steering_assembly")
    knee_platform = object_model.get_part("knee_platform")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    steering_yaw = object_model.get_articulation("frame_to_steering")
    knee_tilt = object_model.get_articulation("frame_to_knee_platform")

    for part_name in (
        "frame",
        "steering_assembly",
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
        "knee_platform",
    ):
        ctx.check(
            f"part {part_name} exists",
            object_model.get_part(part_name) is not None,
        )

    ctx.expect_contact(
        steering,
        front_left_wheel,
        elem_a="front_axle",
        elem_b="hub",
        name="front left wheel rides on steering axle",
    )
    ctx.expect_contact(
        steering,
        front_right_wheel,
        elem_a="front_axle",
        elem_b="hub",
        name="front right wheel rides on steering axle",
    )
    ctx.expect_contact(
        frame,
        rear_left_wheel,
        elem_a="rear_left_stub",
        elem_b="hub",
        name="rear left wheel rides on split axle stub",
    )
    ctx.expect_contact(
        frame,
        rear_right_wheel,
        elem_a="rear_right_stub",
        elem_b="hub",
        name="rear right wheel rides on split axle stub",
    )
    ctx.expect_contact(
        knee_platform,
        frame,
        elem_a="pivot_boss",
        elem_b="tilt_bracket_plate",
        name="knee platform pivot sits on side bracket",
    )
    ctx.expect_gap(
        knee_platform,
        frame,
        axis="z",
        positive_elem="pad_base",
        negative_elem="deck",
        min_gap=0.22,
        name="knee platform sits well above standing deck",
    )
    ctx.expect_origin_gap(
        front_left_wheel,
        rear_left_wheel,
        axis="x",
        min_gap=0.50,
        name="front axle stays ahead of rear split axle",
    )

    left_front_rest = ctx.part_world_position(front_left_wheel)
    with ctx.pose({steering_yaw: 0.45}):
        left_front_turned = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "steering yaw swings the front wheel leftward",
        left_front_rest is not None
        and left_front_turned is not None
        and left_front_turned[1] > left_front_rest[1] + 0.015
        and left_front_turned[0] < left_front_rest[0] - 0.010,
        details=f"rest={left_front_rest}, turned={left_front_turned}",
    )

    def elem_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    bolster_rest = elem_center_z(knee_platform, "front_pad_bolster")
    with ctx.pose({knee_tilt: 0.30}):
        bolster_tilted = elem_center_z(knee_platform, "front_pad_bolster")
    ctx.check(
        "positive knee tilt raises the front of the pad",
        bolster_rest is not None
        and bolster_tilted is not None
        and bolster_tilted > bolster_rest + 0.02,
        details=f"rest={bolster_rest}, tilted={bolster_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
