from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


DECK_LENGTH = 0.800
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
TRUCK_X = 0.240
WHEEL_Y = 0.126
WHEEL_Z = -0.072
WHEEL_RADIUS = 0.034
WHEEL_WIDTH = 0.038


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _deck_half_width(x: float, *, length: float, width: float) -> float:
    half_length = length * 0.5
    half_width = width * 0.5
    straight_half = half_length - half_width
    ax = abs(x)
    if ax <= straight_half:
        return half_width
    nose_u = min(1.0, (ax - straight_half) / max(half_width, 1e-6))
    # Rounded skateboard ends: an elliptical nose planform, kept slightly blunt
    # so the mesh has robust non-degenerate end caps.
    return max(0.016, half_width * math.sqrt(max(0.0, 1.0 - nose_u * nose_u)))


def _deck_kick_rise(x: float, *, length: float) -> float:
    ax = abs(x)
    kick_start = length * 0.33
    kick_end = length * 0.5
    return 0.036 * _smoothstep((ax - kick_start) / (kick_end - kick_start))


def _build_deck_mesh(
    *,
    length: float = DECK_LENGTH,
    width: float = DECK_WIDTH,
    thickness: float = DECK_THICKNESS,
) -> MeshGeometry:
    """Single connected deck shell with rounded ends, camber, and kicked nose/tail."""
    geom = MeshGeometry()
    x_count = 35
    y_count = 17
    xs = [-length * 0.5 + length * i / (x_count - 1) for i in range(x_count)]
    ts = [-1.0 + 2.0 * j / (y_count - 1) for j in range(y_count)]

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for x in xs:
        half_width = _deck_half_width(x, length=length, width=width)
        row_top: list[int] = []
        row_bottom: list[int] = []
        rise = _deck_kick_rise(x, length=length)
        for t in ts:
            y = t * half_width
            # Real skateboard decks are laterally concave: rails sit higher than
            # the centerline.  Keep the bottom parallel enough to read as a thick
            # laminated board rather than a thin sheet.
            concave = 0.0045 * (abs(t) ** 1.7)
            top_z = rise + concave + thickness * 0.5
            bottom_z = rise + 0.0020 * (abs(t) ** 1.3) - thickness * 0.5
            row_top.append(geom.add_vertex(x, y, top_z))
            row_bottom.append(geom.add_vertex(x, y, bottom_z))
        top.append(row_top)
        bottom.append(row_bottom)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(x_count - 1):
        for j in range(y_count - 1):
            quad(top[i][j], top[i + 1][j], top[i + 1][j + 1], top[i][j + 1])
            quad(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j], bottom[i][j])

    for i in range(x_count - 1):
        quad(bottom[i][0], bottom[i + 1][0], top[i + 1][0], top[i][0])
        quad(top[i][y_count - 1], top[i + 1][y_count - 1], bottom[i + 1][y_count - 1], bottom[i][y_count - 1])

    for j in range(y_count - 1):
        quad(bottom[0][j], top[0][j], top[0][j + 1], bottom[0][j + 1])
        quad(top[-1][j], bottom[-1][j], bottom[-1][j + 1], top[-1][j + 1])

    return geom


def _build_grip_mesh() -> MeshGeometry:
    """Thin black grip layer that follows the deck's concave top surface."""
    geom = MeshGeometry()
    x_count = 31
    y_count = 13
    grip_length = 0.735
    inset = 0.012
    xs = [-grip_length * 0.5 + grip_length * i / (x_count - 1) for i in range(x_count)]
    ts = [-1.0 + 2.0 * j / (y_count - 1) for j in range(y_count)]

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for x in xs:
        half_width = max(0.010, min(DECK_WIDTH * 0.5 - inset, _deck_half_width(x, length=DECK_LENGTH, width=DECK_WIDTH) - inset))
        rise = _deck_kick_rise(x, length=DECK_LENGTH)
        row_top: list[int] = []
        row_bottom: list[int] = []
        for t in ts:
            y = t * half_width
            concave = 0.0045 * (abs(t) ** 1.7)
            deck_top = rise + concave + DECK_THICKNESS * 0.5
            row_top.append(geom.add_vertex(x, y, deck_top + 0.0008))
            row_bottom.append(geom.add_vertex(x, y, deck_top - 0.0002))
        top.append(row_top)
        bottom.append(row_bottom)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(x_count - 1):
        for j in range(y_count - 1):
            quad(top[i][j], top[i + 1][j], top[i + 1][j + 1], top[i][j + 1])
            quad(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j], bottom[i][j])
    for i in range(x_count - 1):
        quad(bottom[i][0], bottom[i + 1][0], top[i + 1][0], top[i][0])
        quad(top[i][y_count - 1], top[i + 1][y_count - 1], bottom[i + 1][y_count - 1], bottom[i][y_count - 1])
    for j in range(y_count - 1):
        quad(bottom[0][j], top[0][j], top[0][j + 1], bottom[0][j + 1])
        quad(top[-1][j], bottom[-1][j], bottom[-1][j + 1], top[-1][j + 1])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_truck_skateboard")

    maple = model.material("maple_laminate", rgba=(0.74, 0.51, 0.29, 1.0))
    grip = model.material("black_grip_tape", rgba=(0.015, 0.016, 0.017, 1.0))
    truck_metal = model.material("cast_aluminum", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_metal = model.material("dark_axle_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    bolt_black = model.material("black_bolt_heads", rgba=(0.02, 0.02, 0.022, 1.0))
    bushing = model.material("red_urethane_bushing", rgba=(0.60, 0.04, 0.03, 1.0))
    wheel_urethane = model.material("amber_urethane", rgba=(0.95, 0.56, 0.17, 0.72))
    hub_white = model.material("white_wheel_core", rgba=(0.88, 0.88, 0.84, 1.0))
    riser_rubber = model.material("black_riser_pad", rgba=(0.04, 0.04, 0.045, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_build_deck_mesh(), "kicked_concave_deck"),
        material=maple,
        name="wood_deck",
    )
    deck.visual(
        mesh_from_geometry(_build_grip_mesh(), "grip_tape_sheet"),
        material=grip,
        name="grip_tape",
    )
    for x in (-TRUCK_X, TRUCK_X):
        deck.visual(
            Box((0.096, 0.074, 0.006)),
            origin=Origin(xyz=(x, 0.0, -0.009)),
            material=riser_rubber,
            name=f"riser_pad_{'front' if x > 0 else 'rear'}",
        )
        deck.visual(
            Box((0.106, 0.080, 0.010)),
            origin=Origin(xyz=(x, 0.0, -0.017)),
            material=truck_metal,
            name=f"baseplate_{'front' if x > 0 else 'rear'}",
        )
        deck.visual(
            Cylinder(radius=0.019, length=0.012),
            origin=Origin(xyz=(x, 0.0, -0.021)),
            material=truck_metal,
            name=f"kingpin_socket_{'front' if x > 0 else 'rear'}",
        )
        for bx in (-0.030, 0.030):
            for by in (-0.026, 0.026):
                deck.visual(
                    Cylinder(radius=0.0052, length=0.0030),
                    origin=Origin(xyz=(x + bx, by, 0.0088)),
                    material=bolt_black,
                    name=f"bolt_{'front' if x > 0 else 'rear'}",
                )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.055)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.025,
            0.033,
            rim=WheelRim(inner_radius=0.017, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.014,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.020, hole_diameter=0.0022),
            ),
            face=WheelFace(dish_depth=0.0025, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0022, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.012),
        ),
        "skateboard_wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.0255,
            tread=TireTread(style="circumferential", depth=0.0015, count=3, land_ratio=0.65),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0012),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
        ),
        "skateboard_urethane_tire",
    )

    def add_hanger(name: str, x: float):
        hanger = model.part(name)
        hanger.visual(
            Cylinder(radius=0.014, length=0.216),
            origin=Origin(xyz=(0.0, 0.0, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=truck_metal,
            name="hanger_bar",
        )
        hanger.visual(
            Cylinder(radius=0.0052, length=0.305),
            origin=Origin(xyz=(0.0, 0.0, WHEEL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="axle_shaft",
        )
        hanger.visual(
            Box((0.066, 0.052, 0.040)),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material=truck_metal,
            name="center_block",
        )
        hanger.visual(
            Box((0.012, 0.078, 0.048)),
            origin=Origin(xyz=(-0.026, 0.0, -0.034)),
            material=truck_metal,
            name="fork_cheek_0",
        )
        hanger.visual(
            Box((0.012, 0.078, 0.048)),
            origin=Origin(xyz=(0.026, 0.0, -0.034)),
            material=truck_metal,
            name="fork_cheek_1",
        )
        for side_index, y in enumerate((-0.087, 0.087)):
            hanger.visual(
                Box((0.034, 0.024, 0.030)),
                origin=Origin(xyz=(0.0, y, -0.064)),
                material=truck_metal,
                name=f"axle_block_{side_index}",
            )
        hanger.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=bushing,
            name="upper_bushing",
        )
        hanger.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.052)),
            material=bushing,
            name="lower_bushing",
        )
        hanger.visual(
            Cylinder(radius=0.006, length=0.068),
            origin=Origin(xyz=(0.0, 0.0, -0.036)),
            material=dark_metal,
            name="kingpin_bolt",
        )
        hanger.inertial = Inertial.from_geometry(
            Box((0.090, 0.305, 0.090)),
            mass=0.36,
            origin=Origin(xyz=(0.0, 0.0, -0.050)),
        )
        model.articulation(
            f"{name.split('_')[0]}_steer",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=hanger,
            origin=Origin(xyz=(x, 0.0, -0.025)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.35, upper=0.35),
        )
        return hanger

    front_hanger = add_hanger("front_hanger", TRUCK_X)
    rear_hanger = add_hanger("rear_hanger", -TRUCK_X)

    def add_wheel(parent, prefix: str, index: int, y: float):
        wheel = model.part(f"{prefix}_wheel_{index}")
        wheel.visual(wheel_mesh, material=hub_white, name="hub")
        wheel.visual(tire_mesh, material=wheel_urethane, name="tire")
        wheel.inertial = Inertial.from_geometry(
            Box((WHEEL_WIDTH, WHEEL_RADIUS * 2.0, WHEEL_RADIUS * 2.0)),
            mass=0.11,
        )
        model.articulation(
            f"{prefix}_wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel,
            # Rotate the joint frame so the wheel helper's local X spin axis lies
            # along the truck axle, which runs across the skateboard width.
            origin=Origin(xyz=(0.0, y, WHEEL_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=45.0),
        )
        return wheel

    add_wheel(front_hanger, "front", 0, -WHEEL_Y)
    add_wheel(front_hanger, "front", 1, WHEEL_Y)
    add_wheel(rear_hanger, "rear", 0, -WHEEL_Y)
    add_wheel(rear_hanger, "rear", 1, WHEEL_Y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    wheels = [
        object_model.get_part("front_wheel_0"),
        object_model.get_part("front_wheel_1"),
        object_model.get_part("rear_wheel_0"),
        object_model.get_part("rear_wheel_1"),
    ]
    steer = object_model.get_articulation("front_steer")

    ctx.check("deck_and_trucks_present", deck is not None and front_hanger is not None and rear_hanger is not None)
    ctx.check("four_wheels_present", all(wheel is not None for wheel in wheels))
    ctx.check(
        "wheel_spin_joints_are_continuous",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in (
                "front_wheel_spin_0",
                "front_wheel_spin_1",
                "rear_wheel_spin_0",
                "rear_wheel_spin_1",
            )
        ),
    )
    ctx.check(
        "truck_steering_limits",
        steer is not None
        and steer.motion_limits is not None
        and steer.motion_limits.lower <= -0.30
        and steer.motion_limits.upper >= 0.30,
    )

    if deck is not None and all(wheel is not None for wheel in wheels):
        for index, wheel in enumerate(wheels):
            ctx.expect_gap(
                deck,
                wheel,
                axis="z",
                min_gap=0.020,
                name=f"wheel_{index}_runs_below_deck",
            )

    if steer is not None:
        wheel = object_model.get_part("front_wheel_1")
        rest_position = ctx.part_world_position(wheel)
        with ctx.pose({steer: 0.35}):
            turned_position = ctx.part_world_position(wheel)
        ctx.check(
            "front_truck_steers_wheels",
            rest_position is not None
            and turned_position is not None
            and abs(turned_position[0] - rest_position[0]) > 0.035,
            details=f"rest={rest_position}, turned={turned_position}",
        )

    return ctx.report()


object_model = build_object_model()
