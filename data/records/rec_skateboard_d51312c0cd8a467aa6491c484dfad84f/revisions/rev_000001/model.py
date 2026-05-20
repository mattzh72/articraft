from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
)


DECK_LENGTH = 0.805
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
TRUCK_X = 0.245
WHEEL_RADIUS = 0.028
WHEEL_WIDTH = 0.034


def _truck_axis(truck_x: float) -> tuple[float, float, float]:
    """Tilted kingpin/bushing axis; positive z goes upward toward the baseplate."""

    x_toward_center = -1.0 if truck_x > 0.0 else 1.0
    raw = (0.33 * x_toward_center, 0.0, 0.944)
    mag = math.sqrt(sum(v * v for v in raw))
    return (raw[0] / mag, 0.0, raw[2] / mag)


def _axis_tilt_y(axis: tuple[float, float, float]) -> float:
    """Rotation about local Y that maps a cylinder's local +Z to the axis."""

    return math.atan2(axis[0], axis[2])


def _bushing_ring_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Low-cost hollow urethane/washer ring aligned to local Z."""

    segments = 40
    geom = MeshGeometry()
    # Four loops: outer bottom/top, inner bottom/top.
    loops: list[list[int]] = []
    for radius, z in (
        (outer_radius, -length / 2.0),
        (outer_radius, length / 2.0),
        (inner_radius, -length / 2.0),
        (inner_radius, length / 2.0),
    ):
        loop: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            loop.append(geom.add_vertex(radius * math.cos(a), radius * math.sin(a), z))
        loops.append(loop)

    outer_bottom, outer_top, inner_bottom, inner_top = loops
    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner wall, reversed normals
        geom.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[j], inner_top[i], inner_top[j])
        # Top annulus
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        # Bottom annulus
        geom.add_face(outer_bottom[j], outer_bottom[i], inner_bottom[i])
        geom.add_face(outer_bottom[j], inner_bottom[i], inner_bottom[j])
    return mesh_from_geometry(geom, name)


def _deck_mesh(name: str, *, grip: bool = False):
    """Popsicle skateboard deck: rounded nose/tail, transverse concave, kicktails."""

    length = DECK_LENGTH * (0.86 if grip else 1.0)
    width = DECK_WIDTH * (0.88 if grip else 1.0)
    thickness = 0.0012 if grip else DECK_THICKNESS
    edge_lift = 0.0045 if grip else 0.006
    kick_height = 0.002 if grip else 0.030
    straight_half = length / 2.0 - width / 2.0
    x_count = 31
    y_count = 15

    geom = MeshGeometry()
    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for ix in range(x_count):
        u = ix / (x_count - 1)
        x = -length / 2.0 + u * length
        ax = abs(x)
        if ax <= straight_half:
            half_width = width / 2.0
        else:
            dx = min(width / 2.0, ax - straight_half)
            half_width = max(0.008, math.sqrt(max((width / 2.0) ** 2 - dx * dx, 0.0)))

        if ax < length * 0.34:
            kick = 0.0
        else:
            t = min(1.0, (ax - length * 0.34) / (length * 0.16))
            kick = kick_height * t * t

        top_row: list[int] = []
        bottom_row: list[int] = []
        for iy in range(y_count):
            v = iy / (y_count - 1)
            y = -half_width + 2.0 * half_width * v
            edge = abs(y) / max(half_width, 1.0e-6)
            camber = edge_lift * edge * edge
            z_top = thickness / 2.0 + kick + camber
            z_bottom = z_top - thickness
            top_row.append(geom.add_vertex(x, y, z_top))
            bottom_row.append(geom.add_vertex(x, y, z_bottom))
        top.append(top_row)
        bottom.append(bottom_row)

    for ix in range(x_count - 1):
        for iy in range(y_count - 1):
            # Top surface
            geom.add_face(top[ix][iy], top[ix + 1][iy], top[ix + 1][iy + 1])
            geom.add_face(top[ix][iy], top[ix + 1][iy + 1], top[ix][iy + 1])
            # Bottom surface, reversed
            geom.add_face(bottom[ix + 1][iy], bottom[ix][iy], bottom[ix][iy + 1])
            geom.add_face(bottom[ix + 1][iy], bottom[ix][iy + 1], bottom[ix + 1][iy + 1])

    # Long side walls.
    for ix in range(x_count - 1):
        for iy in (0, y_count - 1):
            geom.add_face(top[ix][iy], bottom[ix][iy], bottom[ix + 1][iy])
            geom.add_face(top[ix][iy], bottom[ix + 1][iy], top[ix + 1][iy])

    # Nose/tail caps.
    for ix in (0, x_count - 1):
        for iy in range(y_count - 1):
            geom.add_face(top[ix][iy], top[ix][iy + 1], bottom[ix][iy + 1])
            geom.add_face(top[ix][iy], bottom[ix][iy + 1], bottom[ix][iy])

    if grip:
        geom.translate(0.0, 0.0, DECK_THICKNESS / 2.0 + 0.0010)
    return mesh_from_geometry(geom, name)


def _add_truck_visuals(model: ArticulatedObject, truck, truck_x: float, metal: Material, urethane: Material):
    axis = _truck_axis(truck_x)
    tilt = _axis_tilt_y(axis)

    truck.visual(
        Box((0.052, 0.112, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
        material=metal,
        name="hanger_body",
    )
    truck.visual(
        Box((0.030, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=metal,
        name="bushing_saddle",
    )
    truck.visual(
        Cylinder(radius=0.00525, length=0.282),
        origin=Origin(xyz=(0.0, 0.0, -0.054), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    truck.visual(
        Cylinder(radius=0.0042, length=0.075),
        origin=Origin(xyz=(axis[0] * -0.013, 0.0, axis[2] * -0.013), rpy=(0.0, tilt, 0.0)),
        material=metal,
        name="kingpin",
    )

    for name, center_s, radius, length, material in (
        ("upper_bushing", -0.0030, 0.018, 0.014, urethane),
        ("lower_bushing", -0.0180, 0.019, 0.014, urethane),
        ("top_washer", 0.0040, 0.015, 0.0025, metal),
        ("bottom_washer", -0.0280, 0.020, 0.0025, metal),
    ):
        truck.visual(
            _bushing_ring_mesh(radius, 0.0040, length, f"{truck.name}_{name}"),
            origin=Origin(
                xyz=(axis[0] * center_s, axis[1] * center_s, axis[2] * center_s),
                rpy=(0.0, tilt, 0.0),
            ),
            material=material,
            name=name,
        )

    truck.visual(
        Cylinder(radius=0.0105, length=0.0075),
        origin=Origin(xyz=(0.0, 0.139, -0.054), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle_nut_0",
    )
    truck.visual(
        Cylinder(radius=0.0105, length=0.0075),
        origin=Origin(xyz=(0.0, -0.139, -0.054), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle_nut_1",
    )


def _add_baseplate_visuals(baseplate, metal: Material, rubber: Material):
    baseplate.visual(
        Box((0.082, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="stamped_plate",
    )
    baseplate.visual(
        Box((0.092, 0.068, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="riser_pad",
    )
    baseplate.visual(
        _bushing_ring_mesh(0.024, 0.0195, 0.008, f"{baseplate.name}_pivot_cup"),
        origin=Origin(xyz=(0.0, 0.0, -0.008), rpy=(0.0, 0.0, 0.0)),
        material=metal,
        name="pivot_cup",
    )
    for ix, x in enumerate((-0.023, 0.023)):
        for iy, y in enumerate((-0.0205, 0.0205)):
            baseplate.visual(
                Cylinder(radius=0.0043, length=0.003),
                origin=Origin(xyz=(x, y, -0.0055)),
                material=metal,
                name=f"clamp_nut_{ix}_{iy}",
            )


def _add_wheel_visuals(wheel, tire_mesh, urethane: Material, bearing: Material):
    wheel.visual(tire_mesh, material=urethane, name="urethane_tire")
    wheel.visual(
        _bushing_ring_mesh(0.0108, 0.00505, WHEEL_WIDTH * 0.76, f"{wheel.name}_bearing_core"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="bearing_core",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_skateboard")

    maple = model.material("sealed_maple", rgba=(0.78, 0.58, 0.34, 1.0))
    grip = model.material("black_grip_tape", rgba=(0.015, 0.014, 0.012, 1.0))
    steel = model.material("zinc_plated_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    aluminum = model.material("cast_aluminum", rgba=(0.47, 0.49, 0.50, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.032, 1.0))
    bushing_urethane = model.material("red_urethane_bushings", rgba=(0.82, 0.08, 0.035, 1.0))
    wheel_urethane = model.material("amber_urethane_wheels", rgba=(1.0, 0.63, 0.18, 0.92))

    deck = model.part("deck")
    deck.visual(_deck_mesh("concave_deck"), material=maple, name="wood_deck")
    deck.visual(_deck_mesh("grip_tape", grip=True), material=grip, name="grip_sheet")
    # Shared through-bolt pattern clamps deck, riser pads, and baseplates in an
    # assembly order that is visible from above.
    for truck_x in (-TRUCK_X, TRUCK_X):
        for ix, dx in enumerate((-0.023, 0.023)):
            for iy, y in enumerate((-0.0205, 0.0205)):
                x = truck_x + dx
                top_z = DECK_THICKNESS / 2.0 + 0.0008 + 0.006 * (abs(y) / (DECK_WIDTH / 2.0)) ** 2
                deck.visual(
                    Cylinder(radius=0.0046, length=0.0024),
                    origin=Origin(xyz=(x, y, top_z), rpy=(0.0, 0.0, 0.0)),
                    material=steel,
                    name=f"bolt_head_{'rear' if truck_x < 0 else 'front'}_{ix}_{iy}",
                )

    wheel_tire = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.010,
        carcass=TireCarcass(belt_width_ratio=0.86, sidewall_bulge=0.03),
        tread=TireTread(style="smooth", depth=0.001, count=18, land_ratio=0.90),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.0035, radius=0.003),
    )
    tire_mesh = mesh_from_geometry(wheel_tire, "skate_wheel_urethane")

    for label, x in (("rear", -TRUCK_X), ("front", TRUCK_X)):
        baseplate = model.part(f"{label}_baseplate")
        _add_baseplate_visuals(baseplate, aluminum, black_rubber)
        model.articulation(
            f"deck_to_{label}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=baseplate,
            origin=Origin(xyz=(x, 0.0, -0.014)),
        )

        truck = model.part(f"{label}_truck")
        _add_truck_visuals(model, truck, x, aluminum, bushing_urethane)
        model.articulation(
            f"{label}_steer",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=truck,
            origin=Origin(xyz=(0.0, 0.0, -0.019)),
            axis=_truck_axis(x),
            motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.36, upper=0.36),
            motion_properties=MotionProperties(damping=0.45, friction=0.04),
        )

        for side, y in (("wheel_0", -0.115), ("wheel_1", 0.115)):
            wheel = model.part(f"{label}_{side}")
            _add_wheel_visuals(wheel, tire_mesh, wheel_urethane, steel)
            model.articulation(
                f"{label}_{side}_spin",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(xyz=(0.0, y, -0.054), rpy=(0.0, 0.0, math.pi / 2.0)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=0.8, velocity=80.0),
                motion_properties=MotionProperties(damping=0.02, friction=0.01),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    for label in ("front", "rear"):
        baseplate = object_model.get_part(f"{label}_baseplate")
        truck = object_model.get_part(f"{label}_truck")
        steer = object_model.get_articulation(f"{label}_steer")

        ctx.allow_overlap(
            baseplate,
            truck,
            elem_a="stamped_plate",
            elem_b="kingpin",
            reason="The simplified kingpin is a captured bolt passing through a molded bore in the baseplate.",
        )
        ctx.expect_gap(
            deck,
            baseplate,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0005,
            positive_elem="wood_deck",
            negative_elem="riser_pad",
            name=f"{label} riser pad is clamped under the deck",
        )
        ctx.expect_overlap(
            baseplate,
            truck,
            axes="xy",
            min_overlap=0.015,
            elem_a="pivot_cup",
            elem_b="upper_bushing",
            name=f"{label} bushing stack is centered under the pivot cup",
        )
        ctx.expect_overlap(
            baseplate,
            truck,
            axes="xy",
            min_overlap=0.004,
            elem_a="stamped_plate",
            elem_b="kingpin",
            name=f"{label} kingpin passes through the baseplate bore",
        )

        rest = ctx.part_world_position(truck)
        with ctx.pose({steer: 0.30}):
            steered = ctx.part_world_position(truck)
            ctx.expect_overlap(
                truck,
                baseplate,
                axes="xy",
                min_overlap=0.010,
                elem_a="upper_bushing",
                elem_b="pivot_cup",
                name=f"{label} steered truck stays captured by bushings",
            )
        ctx.check(
            f"{label} steer joint is a limited bushing pivot",
            steer.motion_limits is not None
            and steer.motion_limits.lower is not None
            and steer.motion_limits.upper is not None
            and steer.motion_limits.lower < -0.25
            and steer.motion_limits.upper > 0.25
            and rest is not None
            and steered is not None,
            details=f"limits={steer.motion_limits}, rest={rest}, steered={steered}",
        )

        for side in ("wheel_0", "wheel_1"):
            wheel = object_model.get_part(f"{label}_{side}")
            spin = object_model.get_articulation(f"{label}_{side}_spin")
            ctx.allow_overlap(
                truck,
                wheel,
                elem_a="axle",
                elem_b="bearing_core",
                reason="The axle is intentionally captured inside the simplified bearing bore while the wheel spins about it.",
            )
            ctx.expect_overlap(
                wheel,
                truck,
                axes="yz",
                min_overlap=0.006,
                elem_a="bearing_core",
                elem_b="axle",
                name=f"{label} {side} bearing surrounds its axle",
            )
            ctx.check(
                f"{label} {side} spins on the truck axle",
                spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (1.0, 0.0, 0.0),
                details=f"type={spin.articulation_type}, axis={spin.axis}",
            )

    return ctx.report()


object_model = build_object_model()
