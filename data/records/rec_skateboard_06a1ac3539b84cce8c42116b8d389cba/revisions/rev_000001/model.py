from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

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
    section_loft,
)


DECK_LENGTH = 0.81
DECK_WIDTH = 0.21
DECK_THICKNESS = 0.012
DECK_CENTER_Z = 0.090
TRUCK_WHEELBASE = 0.365
TRUCK_JOINT_Z = DECK_CENTER_Z - DECK_THICKNESS * 0.5 - 0.018
TRUCK_AXLE_Z = -0.014
HANGER_WIDTH = 0.138
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.033
AXLE_HALF_SPAN = 0.102
KINGPIN_TILT = 0.42
KINGPIN_NORM = sqrt(1.0 + KINGPIN_TILT * KINGPIN_TILT)
FRONT_KINGPIN_AXIS = (-KINGPIN_TILT / KINGPIN_NORM, 0.0, 1.0 / KINGPIN_NORM)
REAR_KINGPIN_AXIS = (KINGPIN_TILT / KINGPIN_NORM, 0.0, 1.0 / KINGPIN_NORM)


def _deck_section(
    x: float,
    *,
    width: float,
    z_center: float,
    thickness: float = DECK_THICKNESS,
    top_concave: float = 0.0018,
    bottom_belly: float = 0.0012,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    top_rail = z_center + thickness * 0.5
    top_center = top_rail - top_concave
    bottom_rail = z_center - thickness * 0.5
    bottom_center = bottom_rail - bottom_belly
    return [
        (x, -half_width, top_rail - 0.0008),
        (x, -half_width * 0.72, top_rail),
        (x, -half_width * 0.22, top_center),
        (x, half_width * 0.22, top_center),
        (x, half_width * 0.72, top_rail),
        (x, half_width, top_rail - 0.0008),
        (x, half_width, bottom_rail + 0.0008),
        (x, half_width * 0.64, bottom_rail),
        (x, half_width * 0.20, bottom_center),
        (x, -half_width * 0.20, bottom_center),
        (x, -half_width * 0.64, bottom_rail),
        (x, -half_width, bottom_rail + 0.0008),
    ]


def _deck_mesh():
    return section_loft(
        [
            _deck_section(-0.405, width=0.146, z_center=0.107),
            _deck_section(-0.340, width=0.184, z_center=0.093),
            _deck_section(-0.215, width=0.206, z_center=0.089),
            _deck_section(0.000, width=0.210, z_center=0.090),
            _deck_section(0.215, width=0.206, z_center=0.089),
            _deck_section(0.340, width=0.186, z_center=0.093),
            _deck_section(0.405, width=0.150, z_center=0.108),
        ]
    )


def _wheel_shell_mesh():
    return LatheGeometry(
        [
            (0.014, -WHEEL_WIDTH * 0.5),
            (0.021, -WHEEL_WIDTH * 0.49),
            (0.0245, -0.0135),
            (0.0262, -0.0085),
            (WHEEL_RADIUS, -0.0020),
            (WHEEL_RADIUS, 0.0020),
            (0.0262, 0.0085),
            (0.0245, 0.0135),
            (0.021, WHEEL_WIDTH * 0.49),
            (0.014, WHEEL_WIDTH * 0.5),
            (0.0108, 0.010),
            (0.0096, 0.000),
            (0.0108, -0.010),
            (0.014, -WHEEL_WIDTH * 0.5),
        ],
        segments=64,
    ).rotate_x(pi / 2.0)


def _add_truck_visuals(part, *, metal, kingpin_pitch: float) -> None:
    part.visual(
        Box((0.052, 0.030, 0.021)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=metal,
        name="hanger_body",
    )
    part.visual(
        Cylinder(radius=0.011, length=HANGER_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, TRUCK_AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle_hanger",
    )
    part.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.020), rpy=(0.0, kingpin_pitch, 0.0)),
        material=metal,
        name="kingpin_boss",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.018 if kingpin_pitch > 0.0 else -0.018, 0.0, -0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="pivot_bridge",
    )
    for side, y_pos in (("left", AXLE_HALF_SPAN - 0.0255), ("right", -AXLE_HALF_SPAN + 0.0255)):
        part.visual(
            Cylinder(radius=0.005, length=0.018),
            origin=Origin(xyz=(0.0, y_pos, TRUCK_AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"{side}_axle_stub",
        )


def _add_wheel_visuals(part, *, wheel_shell, wheel_color, core_color) -> None:
    part.visual(wheel_shell, material=wheel_color, name="wheel_shell")
    part.visual(
        Cylinder(radius=0.014, length=0.021),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=core_color,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=0.0095, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=core_color,
        name="bearing_barrel",
    )
    part.visual(
        Box((0.004, 0.004, 0.0025)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=core_color,
        name="logo_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_standard_trucks")

    maple = model.material("maple", rgba=(0.73, 0.56, 0.36, 1.0))
    grip = model.material("grip", rgba=(0.07, 0.07, 0.08, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.95, 0.89, 0.72, 1.0))
    urethane_core = model.material("urethane_core", rgba=(0.82, 0.80, 0.78, 1.0))
    axle_black = model.material("axle_black", rgba=(0.18, 0.18, 0.19, 1.0))

    deck = model.part("deck")
    deck.visual(mesh_from_geometry(_deck_mesh(), "deck_shell"), material=maple, name="deck_shell")
    deck.visual(
        Box((0.740, 0.188, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z + DECK_THICKNESS * 0.5 + 0.0008)),
        material=grip,
        name="griptape",
    )

    truck_mount_z = DECK_CENTER_Z - DECK_THICKNESS * 0.5 - 0.004
    for prefix, x_pos, yaw in (
        ("front", TRUCK_WHEELBASE * 0.5, 0.0),
        ("rear", -TRUCK_WHEELBASE * 0.5, pi),
    ):
        deck.visual(
            Box((0.060, 0.044, 0.008)),
            origin=Origin(xyz=(x_pos, 0.0, truck_mount_z), rpy=(0.0, 0.0, yaw)),
            material=aluminum,
            name=f"{prefix}_baseplate",
        )
        deck.visual(
            Cylinder(radius=0.009, length=0.018),
            origin=Origin(xyz=(x_pos, 0.0, truck_mount_z - 0.008), rpy=(0.0, 0.35, yaw)),
            material=aluminum,
            name=f"{prefix}_kingpin_seat",
        )
        deck.visual(
            Box((0.030, 0.020, 0.007)),
            origin=Origin(xyz=(x_pos, 0.0, truck_mount_z - 0.011), rpy=(0.0, 0.35, yaw)),
            material=urethane_core,
            name=f"{prefix}_bushing_cap",
        )

    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.040)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
    )

    wheel_shell = mesh_from_geometry(_wheel_shell_mesh(), "wheel_shell")

    front_truck = model.part("front_truck")
    _add_truck_visuals(front_truck, metal=aluminum, kingpin_pitch=0.35)
    front_truck.inertial = Inertial.from_geometry(
        Box((0.170, 0.150, 0.050)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    rear_truck = model.part("rear_truck")
    _add_truck_visuals(rear_truck, metal=aluminum, kingpin_pitch=-0.35)
    rear_truck.inertial = Inertial.from_geometry(
        Box((0.170, 0.150, 0.050)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    for wheel_part_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_part_name)
        _add_wheel_visuals(wheel, wheel_shell=wheel_shell, wheel_color=rubber, core_color=axle_black)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.14,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )

    front_truck_joint = model.articulation(
        "deck_to_front_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_truck,
        origin=Origin(xyz=(TRUCK_WHEELBASE * 0.5, 0.0, TRUCK_JOINT_Z)),
        axis=FRONT_KINGPIN_AXIS,
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-0.32, upper=0.32),
    )
    rear_truck_joint = model.articulation(
        "deck_to_rear_truck",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_truck,
        origin=Origin(xyz=(-TRUCK_WHEELBASE * 0.5, 0.0, TRUCK_JOINT_Z)),
        axis=REAR_KINGPIN_AXIS,
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-0.32, upper=0.32),
    )

    wheel_mounts = (
        ("front_left_spin", front_truck, "front_left_wheel", AXLE_HALF_SPAN),
        ("front_right_spin", front_truck, "front_right_wheel", -AXLE_HALF_SPAN),
        ("rear_left_spin", rear_truck, "rear_left_wheel", AXLE_HALF_SPAN),
        ("rear_right_spin", rear_truck, "rear_right_wheel", -AXLE_HALF_SPAN),
    )
    for joint_name, parent_part, child_name, y_pos in wheel_mounts:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent_part,
            child=child_name,
            origin=Origin(xyz=(0.0, y_pos, TRUCK_AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=28.0),
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

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    front_steer = object_model.get_articulation("deck_to_front_truck")
    rear_steer = object_model.get_articulation("deck_to_rear_truck")
    front_left_spin = object_model.get_articulation("front_left_spin")
    deck_aabb = ctx.part_world_aabb(deck)
    if deck_aabb is not None:
        deck_length = deck_aabb[1][0] - deck_aabb[0][0]
        deck_width = deck_aabb[1][1] - deck_aabb[0][1]
        ctx.check(
            "deck has realistic overall planform",
            0.78 <= deck_length <= 0.84 and 0.18 <= deck_width <= 0.23,
            details=f"length={deck_length:.4f}, width={deck_width:.4f}",
        )

    ctx.expect_gap(
        deck,
        front_left_wheel,
        axis="z",
        positive_elem="deck_shell",
        negative_elem="wheel_shell",
        min_gap=0.002,
        max_gap=0.020,
        name="front wheel sits just below the deck",
    )
    ctx.expect_gap(
        deck,
        rear_left_wheel,
        axis="z",
        positive_elem="deck_shell",
        negative_elem="wheel_shell",
        min_gap=0.002,
        max_gap=0.020,
        name="rear wheel sits just below the deck",
    )
    ctx.expect_origin_gap(
        front_truck,
        rear_truck,
        axis="x",
        min_gap=0.34,
        max_gap=0.39,
        name="truck wheelbase is realistic",
    )
    ctx.check(
        "front and rear kingpin axes oppose along board length",
        front_steer.axis[0] * rear_steer.axis[0] < -0.05,
        details=f"front_axis={front_steer.axis}, rear_axis={rear_steer.axis}",
    )

    rest_wheel_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: front_steer.motion_limits.upper}):
        steered_wheel_pos = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front truck steering moves the wheel center",
        rest_wheel_pos is not None
        and steered_wheel_pos is not None
        and sqrt(
            (steered_wheel_pos[0] - rest_wheel_pos[0]) ** 2
            + (steered_wheel_pos[1] - rest_wheel_pos[1]) ** 2
            + (steered_wheel_pos[2] - rest_wheel_pos[2]) ** 2
        )
        > 0.010,
        details=f"rest={rest_wheel_pos}, steered={steered_wheel_pos}",
    )

    rest_logo_center = _aabb_center(ctx.part_element_world_aabb(front_left_wheel, elem="logo_pad"))
    with ctx.pose({front_left_spin: pi / 2.0}):
        spun_logo_center = _aabb_center(ctx.part_element_world_aabb(front_left_wheel, elem="logo_pad"))
    ctx.check(
        "front wheel spin rotates a visible sidewall feature",
        rest_logo_center is not None
        and spun_logo_center is not None
        and (
            abs(spun_logo_center[0] - rest_logo_center[0]) > 0.010
            or abs(spun_logo_center[2] - rest_logo_center[2]) > 0.010
        ),
        details=f"rest_logo={rest_logo_center}, spun_logo={spun_logo_center}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
