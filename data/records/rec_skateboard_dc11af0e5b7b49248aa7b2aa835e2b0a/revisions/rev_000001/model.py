from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


DECK_LENGTH = 0.82
DECK_WIDTH = 0.215
DECK_THICKNESS = 0.012
DECK_CENTER_Z = 0.088
TRUCK_X = 0.245
KINGPIN_Z = 0.072
AXLE_LOCAL_Z = -0.042
WHEEL_CENTER_Y = 0.118
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.032
AXLE_RADIUS = 0.0042


def _skateboard_deck_mesh() -> MeshGeometry:
    """Closed cambered deck mesh with rounded nose/tail and mild kick."""
    geom = MeshGeometry()

    x_count = 29
    y_count = 15
    half_length = DECK_LENGTH / 2.0
    half_width = DECK_WIDTH / 2.0
    flat_half = 0.300
    nose_span = half_length - flat_half
    y_norms = [(-1.0 + 2.0 * j / (y_count - 1)) for j in range(y_count)]

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for i in range(x_count):
        x = -half_length + DECK_LENGTH * i / (x_count - 1)
        ax = abs(x)
        if ax <= flat_half:
            hw = half_width
            kick = 0.0
        else:
            t = min(1.0, (ax - flat_half) / nose_span)
            hw = max(0.018, half_width * math.sqrt(max(0.0, 1.0 - t * t)))
            kick = 0.038 * (t * t * (3.0 - 2.0 * t))

        row_top: list[int] = []
        row_bottom: list[int] = []
        for yn in y_norms:
            y = yn * hw
            edge_lift = 0.006 * (abs(yn) ** 1.8)
            z_mid = DECK_CENTER_Z + kick
            row_top.append(
                geom.add_vertex(x, y, z_mid + DECK_THICKNESS / 2.0 + edge_lift)
            )
            row_bottom.append(
                geom.add_vertex(x, y, z_mid - DECK_THICKNESS / 2.0 + 0.0015 * (abs(yn) ** 1.5))
            )
        top.append(row_top)
        bottom.append(row_bottom)

    for i in range(x_count - 1):
        for j in range(y_count - 1):
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j], bottom[i][j])

    # Long side walls.
    for i in range(x_count - 1):
        for j in (0, y_count - 1):
            geom.add_face(top[i][j], bottom[i + 1][j], top[i + 1][j])
            geom.add_face(top[i][j], bottom[i][j], bottom[i + 1][j])

    # Rounded nose and tail caps.
    for i in (0, x_count - 1):
        for j in range(y_count - 1):
            geom.add_face(top[i][j], top[i][j + 1], bottom[i][j + 1])
            geom.add_face(top[i][j], bottom[i][j + 1], bottom[i][j])

    return geom


def _deck_top_z(x: float, y: float) -> float:
    half_length = DECK_LENGTH / 2.0
    half_width = DECK_WIDTH / 2.0
    flat_half = 0.300
    nose_span = half_length - flat_half
    ax = abs(x)
    if ax <= flat_half:
        hw = half_width
        kick = 0.0
    else:
        t = min(1.0, (ax - flat_half) / nose_span)
        hw = max(0.018, half_width * math.sqrt(max(0.0, 1.0 - t * t)))
        kick = 0.038 * (t * t * (3.0 - 2.0 * t))
    yn = min(1.0, abs(y) / hw)
    return DECK_CENTER_Z + kick + DECK_THICKNESS / 2.0 + 0.006 * (yn ** 1.8)


def _grip_tape_mesh() -> MeshGeometry:
    """Thin black sheet conforming to the deck camber instead of floating flat."""
    geom = MeshGeometry()
    x_count = 23
    y_count = 9
    x_min, x_max = -0.345, 0.345
    half_tape_width = 0.087
    tape_thickness = 0.0008

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for i in range(x_count):
        x = x_min + (x_max - x_min) * i / (x_count - 1)
        row_top: list[int] = []
        row_bottom: list[int] = []
        for j in range(y_count):
            y = -half_tape_width + 2.0 * half_tape_width * j / (y_count - 1)
            z = _deck_top_z(x, y)
            row_top.append(geom.add_vertex(x, y, z + tape_thickness * 0.55))
            row_bottom.append(geom.add_vertex(x, y, z - tape_thickness * 0.45))
        top.append(row_top)
        bottom.append(row_bottom)

    for i in range(x_count - 1):
        for j in range(y_count - 1):
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j], bottom[i][j])

    for i in range(x_count - 1):
        for j in (0, y_count - 1):
            geom.add_face(top[i][j], bottom[i + 1][j], top[i + 1][j])
            geom.add_face(top[i][j], bottom[i][j], bottom[i + 1][j])
    for i in (0, x_count - 1):
        for j in range(y_count - 1):
            geom.add_face(top[i][j], top[i][j + 1], bottom[i][j + 1])
            geom.add_face(top[i][j], bottom[i][j + 1], bottom[i][j])

    return geom


def _add_truck_mount(deck, x: float, prefix: str, materials: dict[str, Material]) -> None:
    deck.visual(
        Box((0.096, 0.074, 0.010)),
        origin=Origin(xyz=(x, 0.0, 0.077)),
        material=materials["brushed_aluminum"],
        name=f"{prefix}_baseplate",
    )
    deck.visual(
        Box((0.070, 0.050, 0.004)),
        origin=Origin(xyz=(x, 0.0, 0.074)),
        material=materials["brushed_aluminum"],
        name=f"{prefix}_kingpin_boss",
    )
    for dx in (-0.030, 0.030):
        for dy in (-0.022, 0.022):
            deck.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(xyz=(x + dx, dy, _deck_top_z(x + dx, dy) + 0.0015)),
                material=materials["dark_bolt"],
                name=f"{prefix}_bolt_{'p' if dx > 0 else 'n'}_{'p' if dy > 0 else 'n'}",
            )


def _add_truck_geometry(truck, materials: dict[str, Material]) -> None:
    truck.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=materials["red_bushing"],
        name="kingpin_bushing",
    )
    truck.visual(
        Box((0.052, 0.056, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=materials["cast_aluminum"],
        name="hanger_core",
    )
    truck.visual(
        Box((0.060, 0.174, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, AXLE_LOCAL_Z)),
        material=materials["cast_aluminum"],
        name="hanger_bar",
    )
    truck.visual(
        Cylinder(radius=AXLE_RADIUS, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, AXLE_LOCAL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["polished_steel"],
        name="axle",
    )
    for side, label in ((-1.0, "left"), (1.0, "right")):
        truck.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(
                xyz=(0.0, side * 0.142, AXLE_LOCAL_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=materials["dark_bolt"],
            name=f"{label}_axle_nut",
        )


def _add_wheel_geometry(wheel, wheel_mesh, tire_mesh, materials: dict[str, Material]) -> None:
    wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=materials["warm_white_urethane"],
        name="urethane_wheel",
    )
    wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=materials["cream_hub"],
        name="bearing_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_truck_skateboard")

    materials = {
        "maple": model.material("maple_ply", rgba=(0.76, 0.55, 0.32, 1.0)),
        "grip": model.material("black_grip_tape", rgba=(0.015, 0.015, 0.014, 1.0)),
        "brushed_aluminum": model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.64, 1.0)),
        "cast_aluminum": model.material("cast_aluminum", rgba=(0.47, 0.49, 0.50, 1.0)),
        "polished_steel": model.material("polished_steel", rgba=(0.82, 0.84, 0.84, 1.0)),
        "dark_bolt": model.material("blackened_steel", rgba=(0.04, 0.04, 0.045, 1.0)),
        "red_bushing": model.material("red_urethane_bushing", rgba=(0.75, 0.07, 0.04, 1.0)),
        "warm_white_urethane": model.material("warm_white_urethane", rgba=(0.94, 0.90, 0.78, 1.0)),
        "cream_hub": model.material("cream_bearing_core", rgba=(0.85, 0.82, 0.70, 1.0)),
    }

    deck_mesh = mesh_from_geometry(_skateboard_deck_mesh(), "cambered_skateboard_deck")
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.018,
            WHEEL_WIDTH * 0.92,
            rim=WheelRim(inner_radius=0.012, flange_height=0.002, flange_thickness=0.0015),
            hub=WheelHub(
                radius=0.010,
                width=0.020,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.014, hole_diameter=0.002),
            ),
            face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0018, window_radius=0.004),
            bore=WheelBore(style="round", diameter=AXLE_RADIUS * 2.0),
        ),
        "skateboard_bearing_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.017,
            carcass=TireCarcass(belt_width_ratio=0.86, sidewall_bulge=0.03),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.003, radius=0.002),
        ),
        "smooth_urethane_wheel",
    )

    deck = model.part("deck")
    deck.visual(deck_mesh, material=materials["maple"], name="deck_shell")
    deck.visual(
        mesh_from_geometry(_grip_tape_mesh(), "conforming_grip_tape"),
        material=materials["grip"],
        name="grip_tape",
    )
    _add_truck_mount(deck, TRUCK_X, "front", materials)
    _add_truck_mount(deck, -TRUCK_X, "rear", materials)

    front_truck = model.part("front_truck")
    rear_truck = model.part("rear_truck")
    _add_truck_geometry(front_truck, materials)
    _add_truck_geometry(rear_truck, materials)

    front_left_wheel = model.part("front_left_wheel")
    front_right_wheel = model.part("front_right_wheel")
    rear_left_wheel = model.part("rear_left_wheel")
    rear_right_wheel = model.part("rear_right_wheel")
    for wheel in (front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel):
        _add_wheel_geometry(wheel, wheel_mesh, tire_mesh, materials)

    steer_limits = MotionLimits(effort=12.0, velocity=2.2, lower=-0.45, upper=0.45)
    model.articulation(
        "front_kingpin",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_truck,
        origin=Origin(xyz=(TRUCK_X, 0.0, KINGPIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=steer_limits,
    )
    model.articulation(
        "rear_kingpin",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_truck,
        origin=Origin(xyz=(-TRUCK_X, 0.0, KINGPIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=steer_limits,
    )

    spin_limits = MotionLimits(effort=1.5, velocity=60.0)
    for prefix, truck, x_sign in (
        ("front", front_truck, 1.0),
        ("rear", rear_truck, -1.0),
    ):
        for side_name, side in (("left", -1.0), ("right", 1.0)):
            wheel = model.get_part(f"{prefix}_{side_name}_wheel")
            model.articulation(
                f"{prefix}_{side_name}_spin",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(xyz=(0.0, side * WHEEL_CENTER_Y, AXLE_LOCAL_Z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=spin_limits,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_kingpin = object_model.get_articulation("front_kingpin")
    rear_kingpin = object_model.get_articulation("rear_kingpin")

    ctx.check(
        "two trucks steer on kingpins",
        front_kingpin.articulation_type == ArticulationType.REVOLUTE
        and rear_kingpin.articulation_type == ArticulationType.REVOLUTE
        and front_kingpin.motion_limits.lower < 0.0
        and front_kingpin.motion_limits.upper > 0.0
        and rear_kingpin.motion_limits.lower < 0.0
        and rear_kingpin.motion_limits.upper > 0.0,
        details="Front and rear trucks must have limited revolute steering about their kingpin axes.",
    )

    spin_names = (
        "front_left_spin",
        "front_right_spin",
        "rear_left_spin",
        "rear_right_spin",
    )
    ctx.check(
        "four continuous wheel spin joints",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in spin_names
        ),
        details="Each of the four wheels needs its own continuous axle spin joint.",
    )

    for prefix in ("front", "rear"):
        truck = object_model.get_part(f"{prefix}_truck")
        ctx.expect_gap(
            "deck",
            truck,
            axis="z",
            positive_elem=f"{prefix}_baseplate",
            negative_elem="kingpin_bushing",
            min_gap=0.0,
            max_gap=0.002,
            name=f"{prefix} truck bushing seats under baseplate",
        )

        left = object_model.get_part(f"{prefix}_left_wheel")
        right = object_model.get_part(f"{prefix}_right_wheel")
        for wheel in (left, right):
            ctx.allow_overlap(
                wheel,
                truck,
                elem_a="bearing_core",
                elem_b="axle",
                reason="The wheel bearing core is intentionally captured around the continuous axle shaft.",
            )
            ctx.expect_within(
                truck,
                wheel,
                axes="xz",
                inner_elem="axle",
                outer_elem="bearing_core",
                margin=0.0,
                name=f"{wheel.name} axle centered in bearing core",
            )
            ctx.expect_overlap(
                wheel,
                truck,
                axes="y",
                elem_a="bearing_core",
                elem_b="axle",
                min_overlap=0.025,
                name=f"{wheel.name} bearing retained on axle",
            )

        lp = ctx.part_world_position(left)
        rp = ctx.part_world_position(right)
        ctx.check(
            f"{prefix} wheels symmetric about centerline",
            lp is not None
            and rp is not None
            and abs(lp[0] - rp[0]) < 1e-6
            and abs(lp[1] + rp[1]) < 1e-6
            and abs(lp[2] - rp[2]) < 1e-6,
            details=f"left={lp}, right={rp}",
        )

    with ctx.pose({front_kingpin: 0.35}):
        steered = ctx.part_world_position(object_model.get_part("front_left_wheel"))
    with ctx.pose({front_kingpin: 0.0}):
        straight = ctx.part_world_position(object_model.get_part("front_left_wheel"))
    ctx.check(
        "front truck steering moves wheel around kingpin",
        steered is not None
        and straight is not None
        and abs(steered[0] - straight[0]) > 0.020,
        details=f"straight={straight}, steered={steered}",
    )

    return ctx.report()


object_model = build_object_model()
