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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
):
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_wheel_visuals(
    part,
    *,
    wheel_radius: float,
    wheel_width: float,
    tire_material,
    rim_material,
    hub_material,
) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.74, length=wheel_width * 0.60),
        origin=spin_origin,
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.30, length=wheel_width),
        origin=spin_origin,
        material=hub_material,
        name="wheel_hub",
    )
    part.visual(
        Cylinder(radius=0.005, length=wheel_radius * 1.35),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="spoke_x",
    )
    part.visual(
        Cylinder(radius=0.005, length=wheel_radius * 1.35),
        material=hub_material,
        name="spoke_z",
    )
    part.inertial = Inertial.from_geometry(
        Box((wheel_radius * 2.0, wheel_width, wheel_radius * 2.0)),
        mass=1.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_knee_scooter")

    frame_black = model.material("frame_black", rgba=(0.18, 0.18, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.11, 0.12, 1.0))
    pad_black = model.material("pad_black", rgba=(0.15, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.70, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.48, 0.90)),
        mass=13.0,
        origin=Origin(xyz=(0.02, 0.0, 0.45)),
    )

    _add_tube(
        frame,
        (-0.22, 0.135, 0.24),
        (0.14, 0.135, 0.24),
        radius=0.017,
        material=frame_black,
        name="left_side_rail",
    )
    _add_tube(
        frame,
        (-0.22, -0.135, 0.24),
        (0.14, -0.135, 0.24),
        radius=0.017,
        material=frame_black,
        name="right_side_rail",
    )
    _add_tube(
        frame,
        (-0.19, -0.118, 0.24),
        (-0.19, 0.118, 0.24),
        radius=0.017,
        material=frame_black,
        name="rear_crossmember",
    )
    _add_tube(
        frame,
        (-0.02, -0.118, 0.24),
        (-0.02, 0.118, 0.24),
        radius=0.015,
        material=frame_black,
        name="center_crossmember",
    )
    _add_tube(
        frame,
        (0.13, -0.118, 0.24),
        (0.13, 0.118, 0.24),
        radius=0.017,
        material=frame_black,
        name="front_crossmember",
    )
    _add_tube(
        frame,
        (0.12, 0.118, 0.24),
        (0.18, 0.0, 0.31),
        radius=0.014,
        material=frame_black,
        name="left_steering_brace",
    )
    _add_tube(
        frame,
        (0.12, -0.118, 0.24),
        (0.18, 0.0, 0.31),
        radius=0.014,
        material=frame_black,
        name="right_steering_brace",
    )
    _add_tube(
        frame,
        (0.03, 0.0, 0.24),
        (0.18, 0.0, 0.28),
        radius=0.013,
        material=frame_black,
        name="lower_steering_brace",
    )
    frame.visual(
        Box((0.08, 0.07, 0.06)),
        origin=Origin(xyz=(0.15, 0.0, 0.27)),
        material=frame_black,
        name="steering_head_block",
    )
    frame.visual(
        Box((0.068, 0.11, 0.12)),
        origin=Origin(xyz=(0.17, 0.0, 0.30)),
        material=frame_black,
        name="steering_upright_plate",
    )

    for prefix, x in (("rear", -0.10), ("front", 0.02)):
        _add_tube(
            frame,
            (x, 0.09, 0.24),
            (x, 0.09, 0.46),
            radius=0.014,
            material=frame_black,
            name=f"{prefix}_left_pad_post",
        )
        _add_tube(
            frame,
            (x, -0.09, 0.24),
            (x, -0.09, 0.46),
            radius=0.014,
            material=frame_black,
            name=f"{prefix}_right_pad_post",
        )
    _add_tube(
        frame,
        (-0.12, 0.135, 0.24),
        (-0.10, 0.09, 0.24),
        radius=0.013,
        material=frame_black,
        name="rear_left_pad_base_link",
    )
    _add_tube(
        frame,
        (-0.12, -0.135, 0.24),
        (-0.10, -0.09, 0.24),
        radius=0.013,
        material=frame_black,
        name="rear_right_pad_base_link",
    )
    _add_tube(
        frame,
        (0.05, 0.135, 0.24),
        (0.02, 0.09, 0.24),
        radius=0.013,
        material=frame_black,
        name="front_left_pad_base_link",
    )
    _add_tube(
        frame,
        (0.05, -0.135, 0.24),
        (0.02, -0.09, 0.24),
        radius=0.013,
        material=frame_black,
        name="front_right_pad_base_link",
    )
    _add_tube(
        frame,
        (-0.12, 0.09, 0.46),
        (0.04, 0.09, 0.46),
        radius=0.014,
        material=frame_black,
        name="left_pad_rail",
    )
    _add_tube(
        frame,
        (-0.12, -0.09, 0.46),
        (0.04, -0.09, 0.46),
        radius=0.014,
        material=frame_black,
        name="right_pad_rail",
    )
    _add_tube(
        frame,
        (-0.12, -0.09, 0.46),
        (-0.12, 0.09, 0.46),
        radius=0.014,
        material=frame_black,
        name="rear_pad_cross",
    )
    _add_tube(
        frame,
        (0.04, -0.09, 0.46),
        (0.04, 0.09, 0.46),
        radius=0.014,
        material=frame_black,
        name="front_pad_cross",
    )
    frame.visual(
        Box((0.22, 0.16, 0.022)),
        origin=Origin(xyz=(-0.04, 0.0, 0.471)),
        material=satin_black,
        name="pad_base",
    )
    frame.visual(
        Box((0.24, 0.18, 0.05)),
        origin=Origin(xyz=(-0.04, 0.0, 0.507)),
        material=pad_black,
        name="knee_pad_lower",
    )
    frame.visual(
        Box((0.18, 0.14, 0.03)),
        origin=Origin(xyz=(-0.01, 0.0, 0.545), rpy=(0.0, -0.10, 0.0)),
        material=pad_black,
        name="knee_pad_upper",
    )

    _add_tube(
        frame,
        (-0.14, 0.135, 0.24),
        (-0.20, 0.155, 0.10),
        radius=0.012,
        material=frame_black,
        name="rear_left_dropout",
    )
    _add_tube(
        frame,
        (-0.14, -0.135, 0.24),
        (-0.20, -0.155, 0.10),
        radius=0.012,
        material=frame_black,
        name="rear_right_dropout",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.045),
        origin=Origin(xyz=(-0.20, 0.155, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.045),
        origin=Origin(xyz=(-0.20, -0.155, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_right_axle_stub",
    )
    frame.visual(
        Box((0.06, 0.042, 0.03)),
        origin=Origin(xyz=(-0.03, 0.104, 0.225)),
        material=frame_black,
        name="stand_mount_bracket",
    )

    fork = model.part("fork_assembly")
    fork.inertial = Inertial.from_geometry(
        Box((0.42, 0.34, 0.74)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
    )
    fork.visual(
        Cylinder(radius=0.016, length=_distance((0.0, 0.0, -0.06), (0.0, 0.0, 0.56))),
        origin=Origin(
            xyz=_midpoint((0.0, 0.0, -0.06), (0.0, 0.0, 0.56)),
            rpy=_rpy_for_cylinder((0.0, 0.0, -0.06), (0.0, 0.0, 0.56)),
        ),
        material=frame_black,
        name="steering_stem",
    )
    fork.visual(
        Box((0.06, 0.08, 0.04)),
        origin=Origin(xyz=(-0.02, 0.0, 0.52)),
        material=frame_black,
        name="handlebar_clamp",
    )
    _add_tube(
        fork,
        (-0.03, -0.21, 0.56),
        (-0.03, 0.21, 0.56),
        radius=0.014,
        material=frame_black,
        name="handlebar",
    )
    fork.visual(
        Cylinder(radius=0.017, length=0.11),
        origin=Origin(xyz=(-0.03, 0.175, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    fork.visual(
        Cylinder(radius=0.017, length=0.11),
        origin=Origin(xyz=(-0.03, -0.175, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    _add_tube(
        fork,
        (0.0, 0.0, -0.02),
        (0.02, 0.0, -0.14),
        radius=0.014,
        material=frame_black,
        name="center_fork_spine",
    )
    _add_tube(
        fork,
        (0.02, -0.105, -0.12),
        (0.02, 0.105, -0.12),
        radius=0.016,
        material=frame_black,
        name="fork_crossbar",
    )
    _add_tube(
        fork,
        (0.02, 0.095, -0.12),
        (0.05, 0.095, -0.23),
        radius=0.011,
        material=frame_black,
        name="left_fork_leg",
    )
    _add_tube(
        fork,
        (0.02, -0.095, -0.12),
        (0.05, -0.095, -0.23),
        radius=0.011,
        material=frame_black,
        name="right_fork_leg",
    )
    fork.visual(
        Cylinder(radius=0.0105, length=0.040),
        origin=Origin(xyz=(0.05, 0.1025, -0.23), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_left_axle_stub",
    )
    fork.visual(
        Cylinder(radius=0.0105, length=0.040),
        origin=Origin(xyz=(0.05, -0.1025, -0.23), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_right_axle_stub",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        wheel_radius=0.10,
        wheel_width=0.045,
        tire_material=rubber,
        rim_material=wheel_gray,
        hub_material=steel,
    )
    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        wheel_radius=0.10,
        wheel_width=0.045,
        tire_material=rubber,
        rim_material=wheel_gray,
        hub_material=steel,
    )
    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        wheel_radius=0.10,
        wheel_width=0.045,
        tire_material=rubber,
        rim_material=wheel_gray,
        hub_material=steel,
    )
    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        wheel_radius=0.10,
        wheel_width=0.045,
        tire_material=rubber,
        rim_material=wheel_gray,
        hub_material=steel,
    )

    stand = model.part("parking_stand")
    stand.inertial = Inertial.from_geometry(
        Box((0.05, 0.12, 0.22)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.045, -0.09)),
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.045),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="stand_hinge_barrel",
    )
    _add_tube(
        stand,
        (0.0, 0.0, 0.0),
        (0.0, 0.085, -0.18),
        radius=0.010,
        material=frame_black,
        name="stand_leg",
    )
    stand.visual(
        Box((0.045, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.090, -0.186)),
        material=rubber,
        name="stand_foot",
    )

    model.articulation(
        "frame_to_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork,
        origin=Origin(xyz=(0.22, 0.0, 0.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "frame_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.20, 0.20, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "frame_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.20, -0.20, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "fork_to_front_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.05, 0.145, -0.23)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "fork_to_front_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.05, -0.145, -0.23)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "frame_to_parking_stand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=stand,
        origin=Origin(xyz=(-0.03, 0.135, 0.21)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fork = object_model.get_part("fork_assembly")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    stand = object_model.get_part("parking_stand")

    steer_joint = object_model.get_articulation("frame_to_fork")
    stand_joint = object_model.get_articulation("frame_to_parking_stand")
    wheel_joints = [
        object_model.get_articulation("frame_to_rear_left_wheel"),
        object_model.get_articulation("frame_to_rear_right_wheel"),
        object_model.get_articulation("fork_to_front_left_wheel"),
        object_model.get_articulation("fork_to_front_right_wheel"),
    ]

    ctx.check(
        "steering joint is a vertical yaw axis",
        steer_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(steer_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={steer_joint.articulation_type}, axis={steer_joint.axis}",
    )
    ctx.check(
        "all wheel joints spin about their axles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 1.0, 0.0)
            for joint in wheel_joints
        ),
        details=str([(joint.name, joint.articulation_type, joint.axis) for joint in wheel_joints]),
    )
    ctx.check(
        "parking stand hinges on a fore-aft axis",
        stand_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(stand_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={stand_joint.articulation_type}, axis={stand_joint.axis}",
    )

    ctx.expect_contact(
        fork,
        frame,
        elem_a="steering_stem",
        elem_b="steering_upright_plate",
        contact_tol=0.0005,
        name="fork stem bears against the steering mount",
    )
    ctx.expect_contact(
        stand,
        frame,
        elem_a="stand_hinge_barrel",
        elem_b="stand_mount_bracket",
        contact_tol=0.0005,
        name="parking stand hinge mounts under the side rail",
    )
    ctx.expect_contact(
        rear_left_wheel,
        frame,
        elem_a="wheel_hub",
        elem_b="rear_left_axle_stub",
        contact_tol=0.0005,
        name="rear left wheel meets left axle stub",
    )
    ctx.expect_contact(
        rear_right_wheel,
        frame,
        elem_a="wheel_hub",
        elem_b="rear_right_axle_stub",
        contact_tol=0.0005,
        name="rear right wheel meets right axle stub",
    )
    ctx.expect_contact(
        front_left_wheel,
        fork,
        elem_a="wheel_hub",
        elem_b="front_left_axle_stub",
        contact_tol=0.0005,
        name="front left wheel meets fork axle stub",
    )
    ctx.expect_contact(
        front_right_wheel,
        fork,
        elem_a="wheel_hub",
        elem_b="front_right_axle_stub",
        contact_tol=0.0005,
        name="front right wheel meets fork axle stub",
    )

    rest_front_left = ctx.part_world_position(front_left_wheel)
    rest_front_right = ctx.part_world_position(front_right_wheel)
    with ctx.pose({steer_joint: math.radians(30.0)}):
        turned_front_left = ctx.part_world_position(front_left_wheel)
        turned_front_right = ctx.part_world_position(front_right_wheel)
    ctx.check(
        "positive steering turns the fork to the rider's left",
        rest_front_left is not None
        and rest_front_right is not None
        and turned_front_left is not None
        and turned_front_right is not None
        and turned_front_left[0] < rest_front_left[0] - 0.05
        and turned_front_right[1] > rest_front_right[1] + 0.03,
        details=(
            f"rest_left={rest_front_left}, turned_left={turned_front_left}, "
            f"rest_right={rest_front_right}, turned_right={turned_front_right}"
        ),
    )

    deployed_foot = ctx.part_element_world_aabb(stand, elem="stand_foot")
    with ctx.pose({stand_joint: math.radians(68.0)}):
        folded_foot = ctx.part_element_world_aabb(stand, elem="stand_foot")
    ctx.check(
        "parking stand folds upward toward the frame",
        deployed_foot is not None
        and folded_foot is not None
        and folded_foot[0][2] > deployed_foot[0][2] + 0.08,
        details=f"deployed={deployed_foot}, folded={folded_foot}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
