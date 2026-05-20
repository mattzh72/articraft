from __future__ import annotations

from math import pi

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


DECK_LENGTH = 0.52
DECK_WIDTH = 0.12
DECK_THICKNESS = 0.024
DECK_UNDERSIDE_Z = 0.054
DECK_TOP_Z = DECK_UNDERSIDE_Z + DECK_THICKNESS

WHEEL_RADIUS = 0.055
WHEEL_CORE_RADIUS = 0.046
WHEEL_WIDTH = 0.024

REAR_AXLE_X = -0.215
FRONT_AXLE_LOCAL_X = 0.070
FRONT_AXLE_LOCAL_Z = -0.054
AXLE_Z = 0.055

STEERING_RAKE = 0.20
STEERING_ORIGIN_X = 0.247
STEERING_ORIGIN_Z = 0.115


def _z_cylinder(radius: float, length: float, z_min: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, z_min))


def _rotated_about_y(shape: cq.Workplane, angle_rad: float) -> cq.Workplane:
    return shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_rad * 180.0 / pi)


def _make_deck_mesh() -> cq.Workplane:
    deck_center_z = DECK_UNDERSIDE_Z + DECK_THICKNESS * 0.5
    base = (
        cq.Workplane("XY")
        .box(DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)
        .translate((0.0, 0.0, deck_center_z))
        .edges("|Z")
        .fillet(0.006)
    )

    neck = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.175, DECK_UNDERSIDE_Z),
                (0.175, DECK_TOP_Z),
                (0.238, 0.089),
                (0.252, 0.083),
                (0.238, 0.067),
            ]
        )
        .close()
        .extrude(0.076)
    )

    head_tube = _rotated_about_y(
        cq.Workplane("XY").circle(0.028).circle(0.020).extrude(0.074).translate((0.0, 0.0, -0.037)),
        -STEERING_RAKE,
    ).translate((STEERING_ORIGIN_X, 0.0, STEERING_ORIGIN_Z))

    rear_bridge = cq.Workplane("XY").box(0.020, 0.060, 0.014).translate((-0.286, 0.0, 0.099))
    left_dropout = cq.Workplane("XY").box(0.076, 0.014, 0.056).translate((REAR_AXLE_X, 0.053, 0.082))
    right_dropout = cq.Workplane("XY").box(0.076, 0.014, 0.056).translate((REAR_AXLE_X, -0.053, 0.082))

    rear_arch = (
        cq.Workplane("XZ")
        .circle(WHEEL_RADIUS + 0.006)
        .extrude(0.086)
        .translate((REAR_AXLE_X, 0.0, AXLE_Z))
    )
    rear_notch = cq.Workplane("XY").box(0.135, 0.060, 0.130).translate((REAR_AXLE_X + 0.010, 0.0, 0.085))

    deck_shell = base.union(neck).cut(rear_arch).cut(rear_notch)
    deck_shell = deck_shell.union(head_tube).union(rear_bridge).union(left_dropout).union(right_dropout)
    return deck_shell


def _make_steering_mesh() -> cq.Workplane:
    steering_shaft = _z_cylinder(0.018, 0.150, 0.020)
    stem_tube = _z_cylinder(0.022, 0.640, 0.040)
    clamp_block = cq.Workplane("XY").box(0.050, 0.038, 0.052).translate((0.0, 0.0, 0.698))

    front_gusset = (
        cq.Workplane("XZ")
        .polyline([(0.0, 0.630), (0.066, 0.695), (0.0, 0.705)])
        .close()
        .extrude(0.010)
    )
    rear_gusset = (
        cq.Workplane("XZ")
        .polyline([(0.0, 0.630), (-0.040, 0.688), (0.0, 0.700)])
        .close()
        .extrude(0.010)
    )

    return steering_shaft.union(stem_tube).union(clamp_block).union(front_gusset).union(rear_gusset)


def _make_fork_mesh() -> cq.Workplane:
    fork_bridge = cq.Workplane("XY").box(0.016, 0.048, 0.010).translate((0.0, 0.0, -0.006))
    left_rail = cq.Workplane("XY").box(0.080, 0.008, 0.010).translate((0.040, 0.024, -0.010))
    right_rail = cq.Workplane("XY").box(0.080, 0.008, 0.010).translate((0.040, -0.024, -0.010))
    left_leg = cq.Workplane("XY").box(0.012, 0.010, 0.056).translate((0.068, 0.024, -0.043))
    right_leg = cq.Workplane("XY").box(0.012, 0.010, 0.056).translate((0.068, -0.024, -0.043))
    left_dropout = cq.Workplane("XY").box(0.020, 0.012, 0.016).translate((0.072, 0.018, -0.074))
    right_dropout = cq.Workplane("XY").box(0.020, 0.012, 0.016).translate((0.072, -0.018, -0.074))
    return fork_bridge.union(left_rail).union(right_rail).union(left_leg).union(right_leg).union(left_dropout).union(right_dropout)


def _make_brake_mesh() -> cq.Workplane:
    top_flap = cq.Workplane("XY").box(0.082, 0.056, 0.004).translate((0.042, 0.0, 0.012))
    side_web = cq.Workplane("XY").box(0.020, 0.056, 0.010).translate((0.010, 0.0, 0.006))
    return top_flap.union(side_web)


def _add_wheel(part, mesh_prefix: str, wheel_finish: str, tire_finish: str) -> None:
    wheel_origin = Origin(rpy=(0.0, 0.0, pi * 0.5))
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                WHEEL_CORE_RADIUS,
                WHEEL_WIDTH,
                rim=WheelRim(
                    inner_radius=0.030,
                    flange_height=0.006,
                    flange_thickness=0.003,
                    bead_seat_depth=0.003,
                ),
                hub=WheelHub(
                    radius=0.015,
                    width=0.018,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.022, hole_diameter=0.0028),
                ),
                face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.006),
                bore=WheelBore(style="round", diameter=0.008),
            ),
            f"{mesh_prefix}_core",
        ),
        origin=wheel_origin,
        material=wheel_finish,
        name="wheel",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=WHEEL_CORE_RADIUS,
                tread=TireTread(style="circumferential", depth=0.0025, count=2),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
            ),
            f"{mesh_prefix}_tire",
        ),
        origin=wheel_origin,
        material=tire_finish,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stunt_kick_scooter")

    deck_finish = model.material("deck_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.77, 0.79, 0.82, 1.0))
    tire_finish = model.material("tire_finish", rgba=(0.93, 0.93, 0.91, 1.0))
    grip_finish = model.material("grip_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    brake_pad_finish = model.material("brake_pad_finish", rgba=(0.06, 0.06, 0.07, 1.0))

    deck = model.part("deck")
    deck.visual(mesh_from_cadquery(_make_deck_mesh(), "deck_shell"), material=deck_finish, name="deck_shell")
    deck.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(REAR_AXLE_X, 0.035, AXLE_Z), rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal_finish,
        name="rear_axle_0",
    )
    deck.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(REAR_AXLE_X, -0.035, AXLE_Z), rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal_finish,
        name="rear_axle_1",
    )

    steering = model.part("steering")
    steering.visual(
        mesh_from_cadquery(_make_steering_mesh(), "steering_shell"),
        material=metal_finish,
        name="steering_shell",
    )
    steering.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=metal_finish,
        name="headset",
    )
    steering.visual(
        Cylinder(radius=0.017, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.720), rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal_finish,
        name="crossbar",
    )
    steering.visual(
        Cylinder(radius=0.019, length=0.110),
        origin=Origin(xyz=(0.0, 0.245, 0.720), rpy=(pi * 0.5, 0.0, 0.0)),
        material=grip_finish,
        name="grip_0",
    )
    steering.visual(
        Cylinder(radius=0.019, length=0.110),
        origin=Origin(xyz=(0.0, -0.245, 0.720), rpy=(pi * 0.5, 0.0, 0.0)),
        material=grip_finish,
        name="grip_1",
    )

    fork = model.part("fork")
    fork.visual(mesh_from_cadquery(_make_fork_mesh(), "fork_shell"), material=metal_finish, name="fork_shell")
    fork.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(FRONT_AXLE_LOCAL_X, 0.017, FRONT_AXLE_LOCAL_Z), rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal_finish,
        name="front_axle_0",
    )
    fork.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(FRONT_AXLE_LOCAL_X, -0.017, FRONT_AXLE_LOCAL_Z), rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal_finish,
        name="front_axle_1",
    )

    front_wheel = model.part("front_wheel")
    _add_wheel(front_wheel, "front_wheel", wheel_finish, tire_finish)

    rear_wheel = model.part("rear_wheel")
    _add_wheel(rear_wheel, "rear_wheel", wheel_finish, tire_finish)

    brake = model.part("brake")
    brake.visual(mesh_from_cadquery(_make_brake_mesh(), "brake_shell"), material=metal_finish, name="brake_shell")
    brake.visual(
        Cylinder(radius=0.007, length=0.086),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal_finish,
        name="hinge_barrel",
    )
    brake.visual(
        Box((0.022, 0.050, 0.006)),
        origin=Origin(xyz=(0.058, 0.0, 0.012)),
        material=brake_pad_finish,
        name="brake_pad",
    )

    model.articulation(
        "deck_to_steering",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=steering,
        origin=Origin(xyz=(STEERING_ORIGIN_X, 0.0, STEERING_ORIGIN_Z), rpy=(0.0, -STEERING_RAKE, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=7.5),
    )
    model.articulation(
        "steering_to_fork",
        ArticulationType.FIXED,
        parent=steering,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )
    model.articulation(
        "steering_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_wheel,
        origin=Origin(xyz=(FRONT_AXLE_LOCAL_X, 0.0, FRONT_AXLE_LOCAL_Z), rpy=(0.0, STEERING_RAKE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=40.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=40.0),
    )
    model.articulation(
        "deck_to_brake",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=brake,
        origin=Origin(xyz=(-0.278, 0.0, 0.113)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=0.16),
    )

    return model


def _union_aabb(aabbs: list[tuple[tuple[float, float, float], tuple[float, float, float]]]) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    if not aabbs:
        return None
    mins = [min(aabb[0][axis] for aabb in aabbs) for axis in range(3)]
    maxs = [max(aabb[1][axis] for aabb in aabbs) for axis in range(3)]
    return (tuple(mins), tuple(maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    steering = object_model.get_part("steering")
    fork = object_model.get_part("fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    brake = object_model.get_part("brake")

    steering_joint = object_model.get_articulation("deck_to_steering")
    front_spin = object_model.get_articulation("steering_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")
    brake_joint = object_model.get_articulation("deck_to_brake")

    ctx.check(
        "steering_joint_is_continuous",
        steering_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={steering_joint.articulation_type!r}",
    )
    ctx.check(
        "front_wheel_spin_is_continuous",
        front_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={front_spin.articulation_type!r}",
    )
    ctx.check(
        "rear_wheel_spin_is_continuous",
        rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rear_spin.articulation_type!r}",
    )

    brake_limits = brake_joint.motion_limits
    ctx.allow_overlap(
        deck,
        steering,
        elem_a="deck_shell",
        elem_b="headset",
        reason="The headset cap intentionally nests into the simplified head tube collar.",
    )
    ctx.allow_overlap(
        deck,
        fork,
        elem_a="deck_shell",
        elem_b="fork_shell",
        reason="The fork crown is simplified as continuing through the head tube region under the deck neck.",
    )
    ctx.allow_overlap(
        fork,
        front_wheel,
        elem_a="fork_shell",
        elem_b="wheel",
        reason="The fork dropouts and front hub are simplified as a retained axle fit around the wheel core.",
    )
    ctx.check(
        "brake_has_realistic_travel",
        brake_joint.articulation_type == ArticulationType.REVOLUTE
        and brake_limits is not None
        and brake_limits.lower is not None
        and brake_limits.upper is not None
        and brake_limits.lower <= 0.0
        and brake_limits.upper >= 0.15,
        details=f"type={brake_joint.articulation_type!r}, limits={brake_limits!r}",
    )

    ctx.expect_gap(
        brake,
        rear_wheel,
        axis="z",
        positive_elem="brake_pad",
        negative_elem="tire",
        min_gap=0.004,
        max_gap=0.014,
        name="brake_pad_rests_just_above_rear_wheel",
    )

    if brake_limits is not None and brake_limits.upper is not None:
        with ctx.pose({brake_joint: brake_limits.upper}):
            ctx.expect_gap(
                brake,
                rear_wheel,
                axis="z",
                positive_elem="brake_pad",
                negative_elem="tire",
                max_gap=0.0025,
                max_penetration=0.0,
                name="brake_pad_can_close_to_the_tire",
            )

    deck_aabb = ctx.part_world_aabb(deck)
    steering_aabb = ctx.part_world_aabb(steering)
    fork_aabb = ctx.part_world_aabb(fork)
    front_aabb = ctx.part_world_aabb(front_wheel)
    rear_aabb = ctx.part_world_aabb(rear_wheel)
    brake_aabb = ctx.part_world_aabb(brake)
    model_aabb = _union_aabb([aabb for aabb in (deck_aabb, steering_aabb, fork_aabb, front_aabb, rear_aabb, brake_aabb) if aabb is not None])
    ctx.check("model_aabb_present", model_aabb is not None, details=f"aabb={model_aabb!r}")
    if model_aabb is not None:
        mins, maxs = model_aabb
        size = tuple(maxs[axis] - mins[axis] for axis in range(3))
        ctx.check("park_scooter_length", 0.60 <= size[0] <= 0.72, details=f"size={size!r}")
        ctx.check("park_scooter_width", 0.10 <= size[1] <= 0.62, details=f"size={size!r}")
        ctx.check("park_scooter_height", 0.78 <= size[2] <= 0.92, details=f"size={size!r}")

    rest_front_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({steering_joint: 0.65}):
        steered_front_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "front_wheel_moves_when_steering_turns",
        rest_front_pos is not None
        and steered_front_pos is not None
        and abs(steered_front_pos[1] - rest_front_pos[1]) > 0.01,
        details=f"rest={rest_front_pos!r}, steered={steered_front_pos!r}",
    )

    return ctx.report()


object_model = build_object_model()
