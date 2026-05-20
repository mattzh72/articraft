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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


DECK_LENGTH = 1.02
DECK_WIDTH = 0.28
DECK_THICKNESS = 0.022
TRUCK_STATION_X = 0.31
WHEEL_RADIUS = 0.060
WHEEL_WIDTH = 0.050
WHEEL_CENTER_Y = 0.170
KINGPIN_ANGLE = math.radians(32.0)
TRUCK_JOINT_Z = -0.052
AXLE_CENTER_Z = -0.056


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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rotate_y(
    point: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _offset_from_joint(
    joint_xyz: tuple[float, float, float],
    pitch: float,
    local_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx, dy, dz = _rotate_y(local_xyz, pitch)
    return (joint_xyz[0] + dx, joint_xyz[1] + dy, joint_xyz[2] + dz)


def _make_wheel_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (0.018, -half_width),
        (0.040, -half_width),
        (0.051, -half_width * 0.92),
        (0.057, -half_width * 0.62),
        (WHEEL_RADIUS, -half_width * 0.22),
        (WHEEL_RADIUS, half_width * 0.22),
        (0.057, half_width * 0.62),
        (0.051, half_width * 0.92),
        (0.040, half_width),
        (0.018, half_width),
        (0.014, half_width * 0.55),
        (0.012, 0.0),
        (0.014, -half_width * 0.55),
        (0.018, -half_width),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=56).rotate_x(math.pi / 2.0),
        "industrial_safety_wheel",
    )


def _add_mount_bolts(deck, *, x_center: float, fastener_material) -> None:
    for dx in (-0.031, 0.031):
        for dy in (-0.026, 0.026):
            deck.visual(
                Cylinder(radius=0.005, length=0.004),
                origin=Origin(xyz=(x_center + dx, dy, 0.019)),
                material=fastener_material,
                name=f"bolt_head_{'front' if x_center > 0 else 'rear'}_{dx:+.3f}_{dy:+.3f}",
            )


def _build_truck_base(
    model: ArticulatedObject,
    name: str,
    *,
    truck_sign: float,
    structure_material,
    guard_material,
    fastener_material,
    bushing_material,
    stop_material,
):
    base = model.part(name)
    joint_pitch = truck_sign * KINGPIN_ANGLE
    joint_xyz = (0.0, 0.0, TRUCK_JOINT_Z)

    base.visual(
        Box((0.160, 0.100, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=structure_material,
        name="mount_plate",
    )
    base.visual(
        Box((0.132, 0.086, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=structure_material,
        name="riser_block",
    )
    base.visual(
        Box((0.074, 0.058, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=structure_material,
        name="tower_block",
    )
    base.visual(
        Box((0.040, 0.050, 0.040)),
        origin=Origin(xyz=(truck_sign * 0.026, 0.0, -0.050)),
        material=structure_material,
        name="pivot_block",
    )

    for dx in (-0.031, 0.031):
        for dy in (-0.026, 0.026):
            base.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(dx, dy, -0.008)),
                material=fastener_material,
                name=f"mount_nut_{dx:+.3f}_{dy:+.3f}",
            )

    for side_name, side in (("left", 1.0), ("right", -1.0)):
        base.visual(
            Box((0.112, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, side * 0.214, -0.026)),
            material=guard_material,
            name=f"{side_name}_guard_top",
        )
        base.visual(
            Box((0.120, 0.012, 0.068)),
            origin=Origin(xyz=(0.0, side * 0.214, -0.057)),
            material=guard_material,
            name=f"{side_name}_guard_outer",
        )
        base.visual(
            Box((0.064, 0.022, 0.050)),
            origin=Origin(xyz=(0.0, side * 0.040, -0.044)),
            material=guard_material,
            name=f"{side_name}_lockout_cheek",
        )
        base.visual(
            Box((0.028, 0.166, 0.008)),
            origin=Origin(xyz=(0.0, side * 0.124, -0.027)),
            material=structure_material,
            name=f"{side_name}_guard_bridge",
        )

    for stop_x in (-0.036, 0.036):
        for side_name, side in (("left", 1.0), ("right", -1.0)):
            base.visual(
                Box((0.020, 0.006, 0.016)),
                origin=Origin(xyz=(stop_x, side * 0.029, -0.056)),
                material=stop_material,
                name=f"{side_name}_travel_stop_{stop_x:+.3f}",
            )

    base.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(
            xyz=_offset_from_joint(joint_xyz, joint_pitch, (0.0, 0.0, 0.004)),
            rpy=(0.0, joint_pitch, 0.0),
        ),
        material=fastener_material,
        name="upper_retainer",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(
            xyz=_offset_from_joint(joint_xyz, joint_pitch, (0.0, 0.0, 0.012)),
            rpy=(0.0, joint_pitch, 0.0),
        ),
        material=bushing_material,
        name="upper_bushing",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(
            xyz=_offset_from_joint(joint_xyz, joint_pitch, (0.0, 0.0, 0.014)),
            rpy=(0.0, joint_pitch, 0.0),
        ),
        material=structure_material,
        name="kingpin_barrel",
    )

    base.inertial = Inertial.from_geometry(
        Box((0.240, 0.420, 0.100)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )
    return base


def _build_hanger(
    model: ArticulatedObject,
    name: str,
    *,
    truck_sign: float,
    structure_material,
    bushing_material,
    fastener_material,
):
    hanger = model.part(name)
    hanger.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=structure_material,
        name="kingpin_socket",
    )
    hanger.visual(
        Box((0.118, 0.046, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=structure_material,
        name="hanger_body",
    )
    hanger.visual(
        Box((0.048, 0.032, 0.026)),
        origin=Origin(xyz=(truck_sign * 0.028, 0.0, -0.050)),
        material=structure_material,
        name="pivot_nose",
    )
    hanger.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=bushing_material,
        name="lower_bushing",
    )
    hanger.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=fastener_material,
        name="lower_washer",
    )
    hanger.visual(
        Box((0.024, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=fastener_material,
        name="kingpin_nut",
    )
    hanger.visual(
        Cylinder(radius=0.008, length=0.298),
        origin=Origin(xyz=(0.0, 0.0, AXLE_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structure_material,
        name="axle_beam",
    )
    for side_name, side in (("left", 1.0), ("right", -1.0)):
        hanger.visual(
            Box((0.044, 0.026, 0.024)),
            origin=Origin(xyz=(truck_sign * 0.018, side * 0.074, -0.038)),
            material=structure_material,
            name=f"{side_name}_axle_gusset",
        )

    hanger.inertial = Inertial.from_geometry(
        Box((0.150, 0.320, 0.100)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
    )
    return hanger


def _build_wheel(
    model: ArticulatedObject,
    name: str,
    *,
    wheel_mesh,
    tire_material,
    core_material,
    fastener_material,
):
    wheel = model.part(name)
    wheel.visual(wheel_mesh, material=tire_material, name="tire")
    wheel.visual(
        Cylinder(radius=0.024, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=core_material,
        name="hub_core",
    )
    wheel.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_material,
        name="outer_cap",
    )
    wheel.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_material,
        name="inner_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.8,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_skateboard")

    deck_dark = model.material("deck_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    deck_plate = model.material("deck_plate", rgba=(0.39, 0.41, 0.43, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.85, 0.76, 0.12, 1.0))
    bushing_orange = model.material("bushing_orange", rgba=(0.86, 0.42, 0.12, 1.0))
    stop_black = model.material("stop_black", rgba=(0.12, 0.12, 0.13, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.76, 0.71, 0.42, 1.0))

    wheel_mesh = _make_wheel_mesh()

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        material=deck_dark,
        name="main_plate",
    )
    deck.visual(
        Box((0.190, 0.140, 0.006)),
        origin=Origin(xyz=(TRUCK_STATION_X, 0.0, 0.014)),
        material=deck_plate,
        name="front_doubler_plate",
    )
    deck.visual(
        Box((0.190, 0.140, 0.006)),
        origin=Origin(xyz=(-TRUCK_STATION_X, 0.0, 0.014)),
        material=deck_plate,
        name="rear_doubler_plate",
    )
    deck.visual(
        Box((0.440, 0.060, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=dark_steel,
        name="center_spine",
    )
    deck.visual(
        Box((0.058, 0.242, 0.030)),
        origin=Origin(xyz=(0.481, 0.0, 0.004)),
        material=deck_plate,
        name="nose_bumper",
    )
    deck.visual(
        Box((0.058, 0.242, 0.030)),
        origin=Origin(xyz=(-0.481, 0.0, 0.004)),
        material=deck_plate,
        name="tail_bumper",
    )
    _add_mount_bolts(deck, x_center=TRUCK_STATION_X, fastener_material=steel)
    _add_mount_bolts(deck, x_center=-TRUCK_STATION_X, fastener_material=steel)
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.060)),
        mass=8.5,
        origin=Origin(),
    )

    front_base = _build_truck_base(
        model,
        "front_truck_base",
        truck_sign=1.0,
        structure_material=dark_steel,
        guard_material=safety_yellow,
        fastener_material=steel,
        bushing_material=bushing_orange,
        stop_material=stop_black,
    )
    rear_base = _build_truck_base(
        model,
        "rear_truck_base",
        truck_sign=-1.0,
        structure_material=dark_steel,
        guard_material=safety_yellow,
        fastener_material=steel,
        bushing_material=bushing_orange,
        stop_material=stop_black,
    )

    front_hanger = _build_hanger(
        model,
        "front_hanger",
        truck_sign=1.0,
        structure_material=dark_steel,
        bushing_material=bushing_orange,
        fastener_material=steel,
    )
    rear_hanger = _build_hanger(
        model,
        "rear_hanger",
        truck_sign=-1.0,
        structure_material=dark_steel,
        bushing_material=bushing_orange,
        fastener_material=steel,
    )

    front_left_wheel = _build_wheel(
        model,
        "front_left_wheel",
        wheel_mesh=wheel_mesh,
        tire_material=wheel_urethane,
        core_material=steel,
        fastener_material=dark_steel,
    )
    front_right_wheel = _build_wheel(
        model,
        "front_right_wheel",
        wheel_mesh=wheel_mesh,
        tire_material=wheel_urethane,
        core_material=steel,
        fastener_material=dark_steel,
    )
    rear_left_wheel = _build_wheel(
        model,
        "rear_left_wheel",
        wheel_mesh=wheel_mesh,
        tire_material=wheel_urethane,
        core_material=steel,
        fastener_material=dark_steel,
    )
    rear_right_wheel = _build_wheel(
        model,
        "rear_right_wheel",
        wheel_mesh=wheel_mesh,
        tire_material=wheel_urethane,
        core_material=steel,
        fastener_material=dark_steel,
    )

    model.articulation(
        "deck_to_front_truck_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_base,
        origin=Origin(xyz=(TRUCK_STATION_X, 0.0, -DECK_THICKNESS * 0.5)),
    )
    model.articulation(
        "deck_to_rear_truck_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_base,
        origin=Origin(xyz=(-TRUCK_STATION_X, 0.0, -DECK_THICKNESS * 0.5)),
    )

    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_base,
        child=front_hanger,
        origin=Origin(xyz=(0.0, 0.0, TRUCK_JOINT_Z), rpy=(0.0, KINGPIN_ANGLE, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=-0.34,
            upper=0.34,
        ),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_base,
        child=rear_hanger,
        origin=Origin(xyz=(0.0, 0.0, TRUCK_JOINT_Z), rpy=(0.0, -KINGPIN_ANGLE, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=-0.34,
            upper=0.34,
        ),
    )

    for articulation_name, hanger, wheel, wheel_side in (
        ("front_left_spin", front_hanger, front_left_wheel, 1.0),
        ("front_right_spin", front_hanger, front_right_wheel, -1.0),
        ("rear_left_spin", rear_hanger, rear_left_wheel, 1.0),
        ("rear_right_spin", rear_hanger, rear_right_wheel, -1.0),
    ):
        model.articulation(
            articulation_name,
            ArticulationType.CONTINUOUS,
            parent=hanger,
            child=wheel,
            origin=Origin(xyz=(0.0, wheel_side * WHEEL_CENTER_Y, AXLE_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_base = object_model.get_part("front_truck_base")
    rear_base = object_model.get_part("rear_truck_base")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")
    front_left_spin = object_model.get_articulation("front_left_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        front_base,
        deck,
        elem_a="mount_plate",
        elem_b="main_plate",
        name="front_truck_base_mounts_to_deck",
    )
    ctx.expect_contact(
        rear_base,
        deck,
        elem_a="mount_plate",
        elem_b="main_plate",
        name="rear_truck_base_mounts_to_deck",
    )
    ctx.expect_origin_distance(
        front_hanger,
        front_base,
        axes="xy",
        max_dist=0.001,
        name="front_hanger_kingpin_axis_aligned",
    )
    ctx.expect_origin_gap(
        front_base,
        front_hanger,
        axis="z",
        min_gap=0.045,
        max_gap=0.060,
        name="front_hanger_hangs_below_base",
    )
    ctx.expect_origin_distance(
        rear_hanger,
        rear_base,
        axes="xy",
        max_dist=0.001,
        name="rear_hanger_kingpin_axis_aligned",
    )
    ctx.expect_origin_gap(
        rear_base,
        rear_hanger,
        axis="z",
        min_gap=0.045,
        max_gap=0.060,
        name="rear_hanger_hangs_below_base",
    )
    ctx.expect_contact(
        front_left_wheel,
        front_hanger,
        elem_a="hub_core",
        elem_b="axle_beam",
        name="front_left_wheel_supported_on_axle",
    )
    ctx.expect_contact(
        front_right_wheel,
        front_hanger,
        elem_a="hub_core",
        elem_b="axle_beam",
        name="front_right_wheel_supported_on_axle",
    )
    ctx.expect_contact(
        rear_left_wheel,
        rear_hanger,
        elem_a="hub_core",
        elem_b="axle_beam",
        name="rear_left_wheel_supported_on_axle",
    )
    ctx.expect_contact(
        rear_right_wheel,
        rear_hanger,
        elem_a="hub_core",
        elem_b="axle_beam",
        name="rear_right_wheel_supported_on_axle",
    )

    rest_center = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_left_spin: 1.7}):
        spun_center = ctx.part_world_position(front_left_wheel)
    if rest_center is None or spun_center is None:
        ctx.fail("front_left_spin_pose_available", "front-left wheel world position was unavailable")
    else:
        translation = math.sqrt(sum((a - b) ** 2 for a, b in zip(rest_center, spun_center)))
        ctx.check(
            "front_left_wheel_spin_keeps_axle_center",
            translation <= 1e-6,
            details=f"wheel center drifted {translation:.6f} m during spin articulation",
        )

    with ctx.pose({front_steer: 0.26}):
        front_left_pos = ctx.part_world_position(front_left_wheel)
        front_right_pos = ctx.part_world_position(front_right_wheel)
        if front_left_pos is None or front_right_pos is None:
            ctx.fail("front_steer_pose_available", "front wheel positions were unavailable in steer pose")
        else:
            front_axle_skew = abs(front_left_pos[0] - front_right_pos[0])
            ctx.check(
                "front_truck_steer_changes_axle_heading",
                front_axle_skew >= 0.020,
                details=f"front axle x-skew only {front_axle_skew:.4f} m in steer pose",
            )
        ctx.expect_gap(
            front_base,
            front_left_wheel,
            axis="y",
            min_gap=0.005,
            positive_elem="left_guard_outer",
            negative_elem="tire",
            name="front_left_guard_keeps_lateral_clearance",
        )
        ctx.expect_gap(
            front_right_wheel,
            front_base,
            axis="y",
            min_gap=0.005,
            positive_elem="tire",
            negative_elem="right_guard_outer",
            name="front_right_guard_keeps_lateral_clearance",
        )

    with ctx.pose({rear_steer: 0.26}):
        rear_left_pos = ctx.part_world_position(rear_left_wheel)
        rear_right_pos = ctx.part_world_position(rear_right_wheel)
        if rear_left_pos is None or rear_right_pos is None:
            ctx.fail("rear_steer_pose_available", "rear wheel positions were unavailable in steer pose")
        else:
            rear_axle_skew = abs(rear_left_pos[0] - rear_right_pos[0])
            ctx.check(
                "rear_truck_steer_changes_axle_heading",
                rear_axle_skew >= 0.020,
                details=f"rear axle x-skew only {rear_axle_skew:.4f} m in steer pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
