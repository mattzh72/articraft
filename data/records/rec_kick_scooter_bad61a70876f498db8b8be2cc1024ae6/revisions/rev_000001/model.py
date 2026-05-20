from __future__ import annotations

from math import cos, pi, radians, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)

RAKE_DEG = 18.0
RAKE_RAD = radians(RAKE_DEG)
STEERING_AXIS = (0.0, -sin(RAKE_RAD), cos(RAKE_RAD))


def _box(size: tuple[float, float, float], center: tuple[float, float, float], *, rotate_x_deg: float = 0.0):
    solid = cq.Workplane("XY").box(*size)
    if rotate_x_deg:
        solid = solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), rotate_x_deg)
    return solid.translate(center)


def _cylinder_x(radius: float, length: float, *, center: tuple[float, float, float], inner_radius: float | None = None):
    solid = cq.Workplane("YZ").circle(radius)
    if inner_radius is not None:
        solid = solid.circle(inner_radius)
    return solid.extrude(length / 2.0, both=True).translate(center)


def _cylinder_z(radius: float, length: float, *, center: tuple[float, float, float]):
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _tilted_tube(radius: float, length: float, *, center: tuple[float, float, float], inner_radius: float | None = None):
    solid = cq.Workplane("XY").circle(radius)
    if inner_radius is not None:
        solid = solid.circle(inner_radius)
    solid = solid.extrude(length / 2.0, both=True)
    solid = solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), RAKE_DEG)
    return solid.translate(center)


def _build_frame_shape():
    deck = (
        cq.Workplane("XY")
        .box(0.120, 0.460, 0.018)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, -0.020, 0.066))
    )
    head_tube = _tilted_tube(0.024, 0.110, center=(0.0, 0.190, 0.126), inner_radius=0.0155)
    left_gusset = _box((0.014, 0.040, 0.150), (-0.032, 0.178, 0.116), rotate_x_deg=RAKE_DEG)
    right_gusset = _box((0.014, 0.040, 0.150), (0.032, 0.178, 0.116), rotate_x_deg=RAKE_DEG)

    left_stay = _box((0.010, 0.130, 0.045), (-0.050, -0.252, 0.083))
    right_stay = _box((0.010, 0.130, 0.045), (0.050, -0.252, 0.083))
    left_riser = _box((0.010, 0.032, 0.162), (-0.050, -0.265, 0.148))
    right_riser = _box((0.010, 0.032, 0.162), (0.050, -0.265, 0.148))
    left_bridge = _box((0.030, 0.038, 0.012), (-0.041, -0.265, 0.218))
    right_bridge = _box((0.030, 0.038, 0.012), (0.041, -0.265, 0.218))
    left_hinge_lug = _cylinder_x(0.008, 0.020, center=(-0.032, -0.265, 0.212))
    right_hinge_lug = _cylinder_x(0.008, 0.020, center=(0.032, -0.265, 0.212))
    left_dropout = _box((0.010, 0.040, 0.022), (-0.050, -0.333, 0.100))
    right_dropout = _box((0.010, 0.040, 0.022), (0.050, -0.333, 0.100))

    frame = deck
    for solid in (
        head_tube,
        left_gusset,
        right_gusset,
        left_stay,
        right_stay,
        left_riser,
        right_riser,
        left_bridge,
        right_bridge,
        left_hinge_lug,
        right_hinge_lug,
        left_dropout,
        right_dropout,
    ):
        frame = frame.union(solid)

    return frame


def _build_steering_shape():
    steerer = _tilted_tube(0.013, 0.180, center=(0.0, 0.0, 0.0))
    headset_collar = _tilted_tube(0.020, 0.030, center=(0.0, -0.023, 0.071), inner_radius=0.0132)
    column = _tilted_tube(0.017, 0.600, center=(0.0, -0.117, 0.362))

    stem_block = _box((0.060, 0.050, 0.036), (0.0, -0.198, 0.620))
    handlebar = _cylinder_x(0.014, 0.400, center=(0.0, -0.208, 0.648))
    left_grip = _cylinder_x(0.017, 0.100, center=(-0.190, -0.208, 0.648))
    right_grip = _cylinder_x(0.017, 0.100, center=(0.190, -0.208, 0.648))

    fork_bridge = _box((0.086, 0.016, 0.035), (0.0, 0.028, 0.015))
    fork_yoke = _box((0.090, 0.020, 0.040), (0.0, 0.024, -0.020))
    left_cheek = _box((0.010, 0.075, 0.060), (-0.035, 0.060, 0.025), rotate_x_deg=-20.0)
    right_cheek = _box((0.010, 0.075, 0.060), (0.035, 0.060, 0.025), rotate_x_deg=-20.0)
    left_leg = _box((0.010, 0.040, 0.145), (-0.043, 0.118, 0.030))
    right_leg = _box((0.010, 0.040, 0.145), (0.043, 0.118, 0.030))
    left_axle_boss = _box((0.010, 0.034, 0.022), (-0.043, 0.145, -0.026))
    right_axle_boss = _box((0.010, 0.034, 0.022), (0.043, 0.145, -0.026))

    steering = steerer
    for solid in (
        headset_collar,
        column,
        stem_block,
        handlebar,
        left_grip,
        right_grip,
        fork_bridge,
        fork_yoke,
        left_cheek,
        right_cheek,
        left_leg,
        right_leg,
        left_axle_boss,
        right_axle_boss,
    ):
        steering = steering.union(solid)

    return steering


def _build_brake_shape():
    shell = _box((0.088, 0.210, 0.010), (0.0, -0.125, 0.012), rotate_x_deg=-10.0)
    brace = _box((0.040, 0.026, 0.022), (0.0, -0.014, 0.004))
    brake = shell.union(brace)
    return brake


def _add_wheel(part, prefix: str, *, wheel_finish: str, tire_finish: str) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.082,
                0.034,
                rim=WheelRim(inner_radius=0.056, flange_height=0.008, flange_thickness=0.0035),
                hub=WheelHub(
                    radius=0.020,
                    width=0.024,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.024, hole_diameter=0.003),
                ),
                face=WheelFace(dish_depth=0.004, front_inset=0.002),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.008),
                bore=WheelBore(style="round", diameter=0.010),
            ),
            f"{prefix}_rim",
        ),
        material=wheel_finish,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.100,
                0.038,
                inner_radius=0.083,
                tread=TireTread(style="circumferential", depth=0.003, count=3),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
            ),
            f"{prefix}_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    part.visual(
        Box((0.060, 0.020, 0.020)),
        origin=Origin(),
        material=wheel_finish,
        name="hub_spacer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuter_kick_scooter")

    frame_finish = model.material("frame_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    deck_grip = model.material("deck_grip", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    tire_finish = model.material("tire_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.12, 0.13, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_build_frame_shape(), "scooter_frame"), material=frame_finish, name="frame_shell")
    frame.visual(
        Box((0.096, 0.340, 0.003)),
        origin=Origin(xyz=(0.0, -0.035, 0.0765)),
        material=deck_grip,
        name="deck_grip",
    )

    steering = model.part("steering")
    steering.visual(
        mesh_from_cadquery(_build_steering_shape(), "scooter_steering"),
        material=frame_finish,
        name="steering_assembly",
    )

    front_wheel = model.part("front_wheel")
    _add_wheel(front_wheel, "front_wheel", wheel_finish="wheel_finish", tire_finish="tire_finish")
    front_wheel.visual(
        Box((0.078, 0.012, 0.012)),
        origin=Origin(),
        material="wheel_finish",
        name="axle_sleeve",
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel(rear_wheel, "rear_wheel", wheel_finish="wheel_finish", tire_finish="tire_finish")
    rear_wheel.visual(
        Box((0.092, 0.012, 0.012)),
        origin=Origin(),
        material="wheel_finish",
        name="axle_sleeve",
    )

    brake = model.part("brake")
    brake.visual(mesh_from_cadquery(_build_brake_shape(), "scooter_brake"), material=trim_finish, name="shell")
    brake.visual(
        Cylinder(radius=0.0075, length=0.044),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_finish,
        name="barrel",
    )
    brake.visual(
        Box((0.060, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, -0.082, 0.002)),
        material=trim_finish,
        name="pad",
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering,
        origin=Origin(xyz=(0.0, 0.190, 0.126)),
        axis=STEERING_AXIS,
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.155, -0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=24.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel,
        origin=Origin(xyz=(0.0, -0.355, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=24.0),
    )
    model.articulation(
        "brake_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brake,
        origin=Origin(xyz=(0.0, -0.265, 0.212)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=0.11),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    brake = object_model.get_part("brake")

    steering_yaw = object_model.get_articulation("steering_yaw")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")
    brake_hinge = object_model.get_articulation("brake_hinge")

    ctx.allow_overlap(
        "frame",
        "steering",
        elem_a="frame_shell",
        elem_b="steering_assembly",
        reason="The steering stem is intentionally simplified as a single mesh nested through the frame head tube sleeve.",
    )
    ctx.allow_overlap(
        "frame",
        "brake",
        elem_a="frame_shell",
        elem_b="barrel",
        reason="The brake hinge barrel is intentionally captured between the frame hinge lugs at the fender pivot.",
    )
    ctx.allow_overlap(
        "front_wheel",
        "steering",
        elem_a="axle_sleeve",
        elem_b="steering_assembly",
        reason="The front axle sleeve is intentionally represented as seated inside the fork-end bosses.",
    )

    ctx.expect_origin_gap(front_wheel, rear_wheel, axis="y", min_gap=0.64, max_gap=0.72, name="urban wheelbase")
    ctx.expect_gap(
        brake,
        rear_wheel,
        axis="z",
        positive_elem="pad",
        negative_elem="tire",
        min_gap=0.002,
        max_gap=0.012,
        name="rear brake pad hovers just above the tire",
    )
    ctx.expect_overlap(
        brake,
        rear_wheel,
        axes="xy",
        elem_a="pad",
        elem_b="tire",
        min_overlap=0.020,
        name="rear brake pad stays over the tire footprint",
    )

    spin_ok = (
        front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_wheel_spin.motion_limits is not None
        and rear_wheel_spin.motion_limits is not None
        and front_wheel_spin.motion_limits.lower is None
        and front_wheel_spin.motion_limits.upper is None
        and rear_wheel_spin.motion_limits.lower is None
        and rear_wheel_spin.motion_limits.upper is None
    )
    ctx.check("both wheels spin continuously", spin_ok, details=f"front={front_wheel_spin.motion_limits}, rear={rear_wheel_spin.motion_limits}")

    steering_limits = steering_yaw.motion_limits
    brake_limits = brake_hinge.motion_limits
    rest_front = ctx.part_world_position(front_wheel)
    rest_pad = ctx.part_element_world_aabb(brake, elem="pad")

    if steering_limits is not None and steering_limits.upper is not None:
        with ctx.pose({steering_yaw: steering_limits.upper}):
            turned_front = ctx.part_world_position(front_wheel)
        ctx.check(
            "steering yaws the front wheel laterally",
            rest_front is not None and turned_front is not None and abs(turned_front[0] - rest_front[0]) >= 0.035,
            details=f"rest={rest_front}, turned={turned_front}",
        )

    if brake_limits is not None and brake_limits.upper is not None:
        with ctx.pose({brake_hinge: brake_limits.upper}):
            ctx.expect_gap(
                brake,
                rear_wheel,
                axis="z",
                positive_elem="pad",
                negative_elem="tire",
                min_gap=0.0,
                max_gap=0.004,
                name="pressed brake closes the tire gap",
            )
            pressed_pad = ctx.part_element_world_aabb(brake, elem="pad")
        ctx.check(
            "brake pedal moves downward when pressed",
            rest_pad is not None and pressed_pad is not None and pressed_pad[0][2] < rest_pad[0][2] - 0.010,
            details=f"rest={rest_pad}, pressed={pressed_pad}",
        )

    return ctx.report()


object_model = build_object_model()
