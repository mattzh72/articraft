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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


DECK_LENGTH = 0.81
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.013
TRUCK_X = 0.195
AXLE_HALF_SPAN = 0.102
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.032
TRUCK_PIVOT_Z = -0.022
AXLE_X_OFFSET = 0.022
KINGPIN_DROP = 0.042
KINGPIN_TILT = 0.38


def _deck_section(x: float, width: float, thickness: float, z_center: float) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width, thickness, radius=min(thickness * 0.45, width * 0.12))
    ]


def _build_deck_mesh():
    half_len = DECK_LENGTH * 0.5
    sections = [
        _deck_section(-half_len, 0.030, 0.008, 0.062),
        _deck_section(-0.345, 0.110, 0.010, 0.040),
        _deck_section(-0.285, 0.178, 0.012, 0.018),
        _deck_section(-0.120, DECK_WIDTH, DECK_THICKNESS, 0.002),
        _deck_section(0.000, DECK_WIDTH, DECK_THICKNESS, 0.000),
        _deck_section(0.120, DECK_WIDTH, DECK_THICKNESS, 0.002),
        _deck_section(0.285, 0.178, 0.012, 0.018),
        _deck_section(0.345, 0.110, 0.010, 0.040),
        _deck_section(half_len, 0.030, 0.008, 0.062),
    ]
    return section_loft(sections)


def _build_handle_loop():
    path = [
        (-0.040, 0.0, 0.0),
        (-0.040, 0.0, 0.220),
        (-0.036, 0.0, 0.520),
        (0.000, 0.0, 0.700),
        (0.036, 0.0, 0.520),
        (0.040, 0.0, 0.220),
        (0.040, 0.0, 0.0),
    ]
    return tube_from_spline_points(
        path,
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handled_skateboard")

    maple = model.material("maple", rgba=(0.73, 0.58, 0.38, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    truck_silver = model.material("truck_silver", rgba=(0.69, 0.71, 0.74, 1.0))
    truck_dark = model.material("truck_dark", rgba=(0.26, 0.28, 0.31, 1.0))
    wheel_cream = model.material("wheel_cream", rgba=(0.89, 0.84, 0.72, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.16, 0.17, 0.18, 1.0))

    deck = model.part("deck")
    deck.visual(mesh_from_geometry(_build_deck_mesh(), "deck_shell"), material=maple, name="deck_shell")
    deck.visual(
        Box((0.620, 0.175, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0078)),
        material=grip_black,
        name="grip_tape",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.090)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    handle_support = model.part("handle_support")
    handle_support.visual(
        Box((0.090, 0.038, 0.010)),
        origin=Origin(xyz=(-0.045, 0.0, 0.005)),
        material=truck_dark,
        name="base_plate",
    )
    handle_support.visual(
        Box((0.024, 0.024, 0.120)),
        origin=Origin(xyz=(-0.014, 0.0, 0.060)),
        material=truck_dark,
        name="rear_brace",
    )
    handle_support.visual(
        mesh_from_geometry(_build_handle_loop(), "handle_loop"),
        origin=Origin(xyz=(-0.045, 0.0, 0.010)),
        material=truck_silver,
        name="upright_loop",
    )
    handle_support.visual(
        Cylinder(radius=0.016, length=0.160),
        origin=Origin(xyz=(-0.045, 0.0, 0.705), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_foam,
        name="handle_grip",
    )
    handle_support.inertial = Inertial.from_geometry(
        Box((0.110, 0.050, 0.720)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )

    axle_length = 2.0 * (AXLE_HALF_SPAN - WHEEL_WIDTH * 0.5)

    def add_truck_base(name: str):
        truck_base = model.part(name)
        truck_base.visual(
            Box((0.078, 0.050, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
            material=truck_dark,
            name="mount_pad",
        )
        truck_base.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=truck_dark,
            name="bushing_housing",
        )
        truck_base.visual(
            Box((0.042, 0.022, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=truck_silver,
            name="pivot_block",
        )
        truck_base.inertial = Inertial.from_geometry(
            Box((0.078, 0.050, 0.040)),
            mass=0.30,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
        )
        return truck_base

    def add_hanger(name: str, axle_x: float):
        hanger = model.part(name)
        hanger.visual(
            Cylinder(radius=0.0105, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=truck_dark,
            name="kingpin_head",
        )
        hanger.visual(
            Box((0.058, 0.030, 0.018)),
            origin=Origin(xyz=(0.82 * axle_x, 0.0, -0.024)),
            material=truck_silver,
            name="hanger_body",
        )
        hanger.visual(
            Box((0.018, 0.136, 0.016)),
            origin=Origin(xyz=(axle_x, 0.0, -0.041)),
            material=truck_silver,
            name="crossbar",
        )
        hanger.visual(
            Cylinder(radius=0.0042, length=axle_length),
            origin=Origin(xyz=(axle_x, 0.0, -KINGPIN_DROP), rpy=(pi / 2.0, 0.0, 0.0)),
            material=truck_dark,
            name="axle_rod",
        )
        hanger.inertial = Inertial.from_geometry(
            Box((0.070, 0.140, 0.050)),
            mass=0.45,
            origin=Origin(xyz=(0.7 * axle_x, 0.0, -0.030)),
        )
        return hanger

    def add_wheel(name: str):
        wheel = model.part(name)
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_cream,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=WHEEL_WIDTH + 0.002),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=truck_dark,
            name="hub_core",
        )
        wheel.visual(
            Cylinder(radius=0.007, length=WHEEL_WIDTH + 0.004),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=truck_silver,
            name="bearing_cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.16,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        return wheel

    front_truck_base = add_truck_base("front_truck_base")
    rear_truck_base = add_truck_base("rear_truck_base")
    front_hanger = add_hanger("front_hanger", AXLE_X_OFFSET)
    rear_hanger = add_hanger("rear_hanger", -AXLE_X_OFFSET)
    front_left_wheel = add_wheel("front_left_wheel")
    front_right_wheel = add_wheel("front_right_wheel")
    rear_left_wheel = add_wheel("rear_left_wheel")
    rear_right_wheel = add_wheel("rear_right_wheel")

    model.articulation(
        "deck_to_handle_support",
        ArticulationType.FIXED,
        parent=deck,
        child=handle_support,
        origin=Origin(xyz=(-0.405, 0.0, 0.060)),
    )
    model.articulation(
        "deck_to_front_truck_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_truck_base,
        origin=Origin(xyz=(TRUCK_X, 0.0, TRUCK_PIVOT_Z)),
    )
    model.articulation(
        "deck_to_rear_truck_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_truck_base,
        origin=Origin(xyz=(-TRUCK_X, 0.0, TRUCK_PIVOT_Z)),
    )
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=front_truck_base,
        child=front_hanger,
        origin=Origin(),
        axis=(-0.37, 0.0, 0.93),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=rear_truck_base,
        child=rear_hanger,
        origin=Origin(),
        axis=(0.37, 0.0, 0.93),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child=front_left_wheel,
        origin=Origin(xyz=(AXLE_X_OFFSET, AXLE_HALF_SPAN, -KINGPIN_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=25.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_hanger,
        child=front_right_wheel,
        origin=Origin(xyz=(AXLE_X_OFFSET, -AXLE_HALF_SPAN, -KINGPIN_DROP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=25.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child=rear_left_wheel,
        origin=Origin(xyz=(-AXLE_X_OFFSET, AXLE_HALF_SPAN, -KINGPIN_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=25.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_hanger,
        child=rear_right_wheel,
        origin=Origin(xyz=(-AXLE_X_OFFSET, -AXLE_HALF_SPAN, -KINGPIN_DROP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=25.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")
    rear_right_spin = object_model.get_articulation("rear_right_wheel_spin")

    ctx.expect_gap(
        deck,
        front_left_wheel,
        axis="z",
        min_gap=0.025,
        name="front left wheel hangs below the deck",
    )
    ctx.expect_gap(
        deck,
        rear_right_wheel,
        axis="z",
        min_gap=0.025,
        name="rear right wheel hangs below the deck",
    )
    ctx.expect_origin_gap(
        front_left_wheel,
        front_right_wheel,
        axis="y",
        min_gap=0.19,
        max_gap=0.21,
        name="front axle keeps a standard wheel track",
    )
    ctx.expect_origin_gap(
        rear_left_wheel,
        rear_right_wheel,
        axis="y",
        min_gap=0.19,
        max_gap=0.21,
        name="rear axle keeps a standard wheel track",
    )

    front_left_rest = ctx.part_world_position(front_left_wheel)
    rear_left_rest = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({front_steer: 0.25, rear_steer: 0.25}):
        front_left_steered = ctx.part_world_position(front_left_wheel)
        rear_left_steered = ctx.part_world_position(rear_left_wheel)
    ctx.check(
        "front truck steering swings the hanger on its kingpin axis",
        front_left_rest is not None
        and front_left_steered is not None
        and abs(front_left_steered[0] - front_left_rest[0]) > 0.01
        and abs(front_left_steered[2] - front_left_rest[2]) > 0.005,
        details=f"rest={front_left_rest}, steered={front_left_steered}",
    )
    ctx.check(
        "rear truck steering swings the hanger on its kingpin axis",
        rear_left_rest is not None
        and rear_left_steered is not None
        and abs(rear_left_steered[0] - rear_left_rest[0]) > 0.01
        and abs(rear_left_steered[2] - rear_left_rest[2]) > 0.005,
        details=f"rest={rear_left_rest}, steered={rear_left_steered}",
    )

    front_left_spin_rest = ctx.part_world_position(front_left_wheel)
    rear_right_spin_rest = ctx.part_world_position(rear_right_wheel)
    with ctx.pose({front_left_spin: 1.2, rear_right_spin: -1.0}):
        front_left_spin_pose = ctx.part_world_position(front_left_wheel)
        rear_right_spin_pose = ctx.part_world_position(rear_right_wheel)
    ctx.check(
        "front wheel spin stays centered on the axle",
        front_left_spin_rest is not None
        and front_left_spin_pose is not None
        and max(abs(a - b) for a, b in zip(front_left_spin_rest, front_left_spin_pose)) < 1e-6,
        details=f"rest={front_left_spin_rest}, spun={front_left_spin_pose}",
    )
    ctx.check(
        "rear wheel spin stays centered on the axle",
        rear_right_spin_rest is not None
        and rear_right_spin_pose is not None
        and max(abs(a - b) for a, b in zip(rear_right_spin_rest, rear_right_spin_pose)) < 1e-6,
        details=f"rest={rear_right_spin_rest}, spun={rear_right_spin_pose}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
