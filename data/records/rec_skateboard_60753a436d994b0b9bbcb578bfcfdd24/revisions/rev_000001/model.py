from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BOARD_LENGTH = 0.82
DECK_THICKNESS = 0.018
DECK_TOP_Z = 0.100
TRUCK_X = 0.255
TRUCK_JOINT_Z = 0.083
WHEEL_Y = 0.168
WHEEL_RADIUS = 0.034
WHEEL_WIDTH = 0.032


def _tube_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    width: float,
    segments: int = 48,
) -> MeshGeometry:
    """Annular wheel/bearing solid centered at the origin with its bore on +Y."""
    geom = MeshGeometry()
    verts: list[tuple[int, int, int, int]] = []
    y0 = -width / 2.0
    y1 = width / 2.0
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        x_outer = outer_radius * math.cos(a)
        z_outer = outer_radius * math.sin(a)
        x_inner = inner_radius * math.cos(a)
        z_inner = inner_radius * math.sin(a)
        verts.append(
            (
                geom.add_vertex(x_outer, y0, z_outer),
                geom.add_vertex(x_outer, y1, z_outer),
                geom.add_vertex(x_inner, y0, z_inner),
                geom.add_vertex(x_inner, y1, z_inner),
            )
        )

    for i in range(segments):
        j = (i + 1) % segments
        o0, o1, i0, i1 = verts[i]
        no0, no1, ni0, ni1 = verts[j]
        # Outer tire tread.
        geom.add_face(o0, no0, no1)
        geom.add_face(o0, no1, o1)
        # Inner bore.
        geom.add_face(i0, i1, ni1)
        geom.add_face(i0, ni1, ni0)
        # Side faces at -Y and +Y.
        geom.add_face(o0, i0, ni0)
        geom.add_face(o0, ni0, no0)
        geom.add_face(o1, no1, ni1)
        geom.add_face(o1, ni1, i1)
    return geom


def _deck_mesh() -> MeshGeometry:
    """Continuous old-school pool-deck outline with kicked nose and tail."""

    def half_width(x: float) -> float:
        broad = 0.097 + 0.022 * math.exp(-((x + 0.04) / 0.27) ** 2)
        nose_shoulder = 0.007 * math.exp(-((x - 0.27) / 0.12) ** 2)
        raw = broad + nose_shoulder
        # Rounded ends close down to a point over the last 70 mm.
        end_start = 0.34
        half_len = BOARD_LENGTH / 2.0
        ax = abs(x)
        if ax <= end_start:
            return raw
        t = min(1.0, (ax - end_start) / (half_len - end_start))
        return raw * math.sqrt(max(0.0, 1.0 - t * t))

    def kick(x: float) -> float:
        start = 0.280
        half_len = BOARD_LENGTH / 2.0
        ax = abs(x)
        if ax <= start:
            return 0.0
        t = (ax - start) / (half_len - start)
        return 0.030 * t * t

    x_values = [
        -BOARD_LENGTH / 2.0 + i * BOARD_LENGTH / 32.0 for i in range(33)
    ]
    upper = [(x, half_width(x)) for x in x_values]
    lower = [(x, -half_width(x)) for x in reversed(x_values[1:-1])]
    profile = upper + lower

    # Ensure the top-face loop is counter-clockwise when viewed from above.
    area = 0.0
    for i, (x0, y0) in enumerate(profile):
        x1, y1 = profile[(i + 1) % len(profile)]
        area += x0 * y1 - x1 * y0
    if area < 0.0:
        profile = list(reversed(profile))

    geom = MeshGeometry()
    top: list[int] = []
    bottom: list[int] = []
    for x, y in profile:
        top_z = DECK_TOP_Z + kick(x)
        top.append(geom.add_vertex(x, y, top_z))
        bottom.append(geom.add_vertex(x, y, top_z - DECK_THICKNESS))

    center_top = geom.add_vertex(0.0, 0.0, DECK_TOP_Z)
    center_bottom = geom.add_vertex(0.0, 0.0, DECK_TOP_Z - DECK_THICKNESS)
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(center_top, top[i], top[j])
        geom.add_face(center_bottom, bottom[j], bottom[i])
        geom.add_face(top[i], bottom[i], bottom[j])
        geom.add_face(top[i], bottom[j], top[j])
    return geom


def _axis_angle_y(axis: tuple[float, float, float]) -> float:
    return math.atan2(axis[0], axis[2])


def _along_axis(
    axis: tuple[float, float, float], distance: float
) -> tuple[float, float, float]:
    return (-axis[0] * distance, -axis[1] * distance, -axis[2] * distance)


def _add_axis_cylinder(
    part,
    *,
    radius: float,
    length: float,
    axis: tuple[float, float, float],
    distance: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_along_axis(axis, distance), rpy=(0.0, _axis_angle_y(axis), 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_skateboard")

    maple = model.material("laminated_maple", rgba=(0.78, 0.55, 0.30, 1.0))
    black = model.material("black_grip_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_fasteners", rgba=(0.12, 0.12, 0.13, 1.0))
    aluminum = model.material("cast_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    urethane_red = model.material("red_urethane", rgba=(0.70, 0.06, 0.035, 1.0))
    tire_black = model.material("smoked_urethane", rgba=(0.025, 0.025, 0.023, 1.0))
    cream = model.material("aged_ivory_hub", rgba=(0.92, 0.82, 0.55, 1.0))
    orange = model.material("retro_orange", rgba=(1.0, 0.42, 0.08, 1.0))
    yellow = model.material("retro_yellow", rgba=(0.95, 0.80, 0.15, 1.0))

    deck_mesh = mesh_from_geometry(_deck_mesh(), "continuous_kicked_deck")
    tire_mesh = mesh_from_geometry(
        _tube_mesh(outer_radius=WHEEL_RADIUS, inner_radius=0.020, width=WHEEL_WIDTH),
        "open_bore_tire",
    )
    hub_mesh = mesh_from_geometry(
        _tube_mesh(outer_radius=0.0208, inner_radius=0.0095, width=WHEEL_WIDTH + 0.002),
        "ivory_hub_annulus",
    )
    bearing_mesh = mesh_from_geometry(
        _tube_mesh(outer_radius=0.0105, inner_radius=0.0068, width=WHEEL_WIDTH + 0.004),
        "steel_bearing_annulus",
    )

    deck = model.part("deck")
    deck.visual(deck_mesh, material=maple, name="kicked_maple_deck")

    # Grip tape is broken by visible metal service hatches and warm retro stripes.
    deck.visual(
        Box((0.540, 0.170, 0.0025)),
        origin=Origin(xyz=(0.000, 0.000, DECK_TOP_Z + 0.0008)),
        material=black,
        name="center_grip",
    )
    for y, mat, nm in ((0.095, orange, "orange_side_stripe"), (-0.095, yellow, "yellow_side_stripe")):
        deck.visual(
            Box((0.610, 0.008, 0.0028)),
            origin=Origin(xyz=(-0.020, y, DECK_TOP_Z + 0.0012)),
            material=mat,
            name=nm,
        )

    for idx, x in enumerate((-0.080, 0.115)):
        deck.visual(
            Box((0.128, 0.066, 0.0032)),
            origin=Origin(xyz=(x, 0.0, DECK_TOP_Z + 0.0020)),
            material=steel,
            name=f"service_hatch_{idx}",
        )
        for sx in (-0.048, 0.048):
            for sy in (-0.022, 0.022):
                deck.visual(
                    Cylinder(radius=0.0042, length=0.0026),
                    origin=Origin(xyz=(x + sx, sy, DECK_TOP_Z + 0.0045)),
                    material=dark_steel,
                    name=f"hatch_bolt_{idx}_{'p' if sx > 0 else 'n'}_{'p' if sy > 0 else 'n'}",
                )

    # Underside adapter plates, gussets, and bolt heads are fixed to the deck root.
    for label, x in (("front", TRUCK_X), ("rear", -TRUCK_X)):
        deck.visual(
            Box((0.135, 0.108, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.0790)),
            material=steel,
            name=f"{label}_adapter",
        )
        deck.visual(
            Box((0.168, 0.017, 0.013)),
            origin=Origin(xyz=(x, 0.052, 0.0765)),
            material=dark_steel,
            name=f"{label}_reinforcement_0",
        )
        deck.visual(
            Box((0.168, 0.017, 0.013)),
            origin=Origin(xyz=(x, -0.052, 0.0765)),
            material=dark_steel,
            name=f"{label}_reinforcement_1",
        )
        for sx in (-0.043, 0.043):
            for sy in (-0.034, 0.034):
                deck.visual(
                    Cylinder(radius=0.0055, length=0.0040),
                    origin=Origin(xyz=(x + sx, sy, 0.0732)),
                    material=dark_steel,
                    name=f"{label}_adapter_bolt_{'p' if sx > 0 else 'n'}_{'p' if sy > 0 else 'n'}",
                )

    def add_truck(label: str, x_world: float, axis_x: float):
        truck = model.part(f"{label}_truck")
        az = math.sqrt(1.0 - axis_x * axis_x)
        axis = (axis_x, 0.0, az)
        axle_center = _along_axis(axis, 0.050)

        _add_axis_cylinder(
            truck,
            radius=0.0044,
            length=0.072,
            axis=axis,
            distance=0.039,
            material=steel,
            name="kingpin",
        )
        for distance, name in ((0.018, "top_washer"), (0.044, "middle_washer"), (0.073, "lower_washer")):
            _add_axis_cylinder(
                truck,
                radius=0.024,
                length=0.004,
                axis=axis,
                distance=distance,
                material=steel,
                name=name,
            )
        _add_axis_cylinder(
            truck,
            radius=0.022,
            length=0.017,
            axis=axis,
            distance=0.031,
            material=urethane_red,
            name="upper_bushing",
        )
        _add_axis_cylinder(
            truck,
            radius=0.023,
            length=0.018,
            axis=axis,
            distance=0.058,
            material=urethane_red,
            name="lower_bushing",
        )
        truck.visual(
            Box((0.074, 0.184, 0.024)),
            origin=Origin(xyz=axle_center),
            material=aluminum,
            name="hanger_yoke",
        )
        truck.visual(
            Cylinder(radius=0.0068, length=0.356),
            origin=Origin(xyz=axle_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="axle_shaft",
        )
        # Small old-school rib blocks visually tie the axle tube into the bushing seat.
        for y in (-0.064, 0.064):
            truck.visual(
                Box((0.050, 0.022, 0.030)),
                origin=Origin(xyz=(axle_center[0], y, axle_center[2] + 0.010)),
                material=aluminum,
                name=f"web_rib_{0 if y < 0 else 1}",
            )

        model.articulation(
            f"{label}_steer",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=truck,
            origin=Origin(xyz=(x_world, 0.0, TRUCK_JOINT_Z)),
            axis=axis,
            motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.38, upper=0.38),
        )

        for wheel_index, y in enumerate((-WHEEL_Y, WHEEL_Y)):
            wheel = model.part(f"{label}_wheel_{wheel_index}")
            wheel.visual(tire_mesh, material=tire_black, name="tire")
            wheel.visual(hub_mesh, material=cream, name="hub")
            wheel.visual(bearing_mesh, material=steel, name="bearing")
            # Asymmetric balance dot makes wheel spin readable.
            wheel.visual(
                Cylinder(radius=0.0040, length=0.0035),
                origin=Origin(
                    xyz=(0.014, 0.018 if y > 0 else -0.018, 0.018),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=orange,
                name="spin_mark",
            )
            model.articulation(
                f"{label}_wheel_spin_{wheel_index}",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(xyz=(axle_center[0], y, axle_center[2])),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=3.0, velocity=60.0),
            )

    add_truck("front", TRUCK_X, axis_x=-0.34)
    add_truck("rear", -TRUCK_X, axis_x=0.34)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    deck = object_model.get_part("deck")
    front_steer = object_model.get_articulation("front_steer")
    rear_steer = object_model.get_articulation("rear_steer")

    for label, truck in (("front", front_truck), ("rear", rear_truck)):
        ctx.allow_overlap(
            deck,
            truck,
            elem_a=f"{label}_adapter",
            elem_b="kingpin",
            reason="The serviceable kingpin is intentionally captured through the bolted adapter plate.",
        )
        ctx.expect_gap(
            deck,
            truck,
            axis="z",
            positive_elem=f"{label}_adapter",
            negative_elem="hanger_yoke",
            min_gap=0.0,
            max_gap=0.035,
            name=f"{label} hanger hangs from adapter stack",
        )
        for wheel_index in (0, 1):
            wheel = object_model.get_part(f"{label}_wheel_{wheel_index}")
            ctx.allow_overlap(
                truck,
                wheel,
                elem_a="axle_shaft",
                elem_b="bearing",
                reason="The axle is represented as a captured shaft running through the wheel bearing bore.",
            )
            ctx.expect_overlap(
                wheel,
                truck,
                axes="y",
                elem_a="bearing",
                elem_b="axle_shaft",
                min_overlap=0.025,
                name=f"{label} wheel {wheel_index} retained on axle",
            )
            ctx.expect_overlap(
                wheel,
                truck,
                axes="xz",
                elem_a="bearing",
                elem_b="axle_shaft",
                min_overlap=0.010,
                name=f"{label} wheel {wheel_index} coaxial with axle",
            )

    for joint in (front_steer, rear_steer):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has realistic steer stops",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < -0.25
            and limits.upper > 0.25
            and abs(joint.axis[0]) > 0.25
            and abs(joint.axis[2]) > 0.85,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    for label in ("front", "rear"):
        for wheel_index in (0, 1):
            spin = object_model.get_articulation(f"{label}_wheel_spin_{wheel_index}")
            ctx.check(
                f"{label} wheel {wheel_index} spins about axle",
                spin.articulation_type == ArticulationType.CONTINUOUS
                and abs(spin.axis[1]) > 0.99,
                details=f"type={spin.articulation_type}, axis={spin.axis}",
            )

    rest = ctx.part_world_position(object_model.get_part("front_wheel_1"))
    with ctx.pose({front_steer: 0.30}):
        steered = ctx.part_world_position(object_model.get_part("front_wheel_1"))
    ctx.check(
        "steering rotates wheel center about kingpin",
        rest is not None
        and steered is not None
        and math.dist((rest[0], rest[2]), (steered[0], steered[2])) > 0.010,
        details=f"rest={rest}, steered={steered}",
    )

    return ctx.report()


object_model = build_object_model()
