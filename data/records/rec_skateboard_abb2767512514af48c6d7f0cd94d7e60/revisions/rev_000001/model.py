from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
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


def _deck_half_width(x: float, *, length: float, width: float) -> float:
    """Rounded/pinched skateboard planform half-width at station x."""
    a = length / 2.0
    b = width / 2.0
    # Superellipse end planform with a subtle waist pinch.
    u = min(1.0, abs(x) / a)
    n = 3.2
    base = b * max(0.0, 1.0 - u**n) ** (1.0 / n)
    waist_factor = 0.88 + 0.12 * (u**1.8)
    return base * waist_factor


def _deck_kick(x: float, *, length: float) -> float:
    flat = 0.305
    a = length / 2.0
    if abs(x) <= flat:
        return 0.0
    t = min(1.0, (abs(x) - flat) / (a - flat))
    return 0.034 * (t * t)


def _weatherproof_deck_mesh() -> MeshGeometry:
    """Continuous sealed composite deck with rounded ends and raised kicktails."""
    length = 0.84
    width = 0.255
    thickness = 0.022
    segments = 96
    top_center = (0.0, 0.0, thickness)
    bottom_center = (0.0, 0.0, 0.0)

    top_loop: list[tuple[float, float, float]] = []
    bottom_loop: list[tuple[float, float, float]] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        # Superellipse coordinates for a skateboard-like rounded rectangle.
        n = 3.2
        x = (length / 2.0) * math.copysign(abs(c) ** (2.0 / n), c)
        half_w = _deck_half_width(x, length=length, width=width)
        y = half_w * math.copysign(abs(s) ** (2.0 / n), s)
        kick = _deck_kick(x, length=length)
        # Slight transverse crown sheds water toward the sealed side rails.
        crown = 0.003 * (1.0 - min(1.0, abs(y) / max(half_w, 1e-6)) ** 2)
        top_loop.append((x, y, thickness + kick + crown))
        bottom_loop.append((x, y, kick))

    geom = MeshGeometry()
    top_center_i = geom.add_vertex(*top_center)
    bottom_center_i = geom.add_vertex(*bottom_center)
    top_indices = [geom.add_vertex(*p) for p in top_loop]
    bottom_indices = [geom.add_vertex(*p) for p in bottom_loop]

    for i in range(segments):
        j = (i + 1) % segments
        # Top fan, bottom fan, and sealed side wall.
        geom.add_face(top_center_i, top_indices[i], top_indices[j])
        geom.add_face(bottom_center_i, bottom_indices[j], bottom_indices[i])
        geom.add_face(top_indices[i], bottom_indices[i], bottom_indices[j])
        geom.add_face(top_indices[i], bottom_indices[j], top_indices[j])
    return geom


def _tilt_rpy_for_axis(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    # Cylinders are local +Z; a pure Y rotation maps +Z to (sin(theta), 0, cos(theta)).
    return (0.0, math.atan2(axis[0], axis[2]), 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_skateboard")

    deck_mat = model.material("sealed_teal_composite", color=(0.02, 0.22, 0.24, 1.0))
    grip_mat = model.material("coarse_black_grip", color=(0.015, 0.016, 0.014, 1.0))
    rubber_mat = model.material("black_uv_rubber", color=(0.005, 0.006, 0.006, 1.0))
    gasket_mat = model.material("matte_epdm_gasket", color=(0.02, 0.018, 0.015, 1.0))
    aluminum_mat = model.material("anodized_aluminum", color=(0.55, 0.60, 0.62, 1.0))
    stainless_mat = model.material("brushed_stainless", color=(0.82, 0.80, 0.76, 1.0))
    urethane_mat = model.material("storm_gray_urethane", color=(0.14, 0.16, 0.17, 1.0))
    bearing_mat = model.material("sealed_bearing_blue", color=(0.02, 0.08, 0.18, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(_weatherproof_deck_mesh(), "sealed_crowned_deck"),
        material=deck_mat,
        name="sealed_deck_shell",
    )

    # Bonded rubber edge and drip lips: slightly proud continuous side bands that
    # protect the composite core and create a water-break under each long edge.
    for side, y in (("rail_0", -0.119), ("rail_1", 0.119)):
        deck.visual(
            Box((0.735, 0.016, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.015)),
            material=rubber_mat,
            name=f"{side}_bumper",
        )
        deck.visual(
            Box((0.650, 0.009, 0.006)),
            origin=Origin(xyz=(0.0, y * 0.92, 0.0)),
            material=rubber_mat,
            name=f"{side}_drip_lip",
        )

    # Textured traction pads are separate bonded sheets but fully seated on the
    # sealed crown, leaving nose/tail color exposed for readability.
    for x, sx, name in ((-0.205, 0.270, "rear_grip"), (0.205, 0.270, "front_grip")):
        deck.visual(
            Box((sx, 0.190, 0.003)),
            origin=Origin(xyz=(x, 0.0, 0.026)),
            material=grip_mat,
            name=name,
        )
    deck.visual(
        Box((0.135, 0.060, 0.0025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=grip_mat,
        name="center_grip_bridge",
    )

    truck_xs = {"front": 0.255, "rear": -0.255}
    bolt_offsets = ((-0.038, -0.024), (-0.038, 0.024), (0.038, -0.024), (0.038, 0.024))
    for station, x in truck_xs.items():
        if station == "front":
            deck.visual(
                Box((0.138, 0.082, 0.006)),
                origin=Origin(xyz=(x, 0.0, -0.003)),
                material=gasket_mat,
                name="front_sealed_pad",
            )
            deck.visual(
                Box((0.112, 0.064, 0.008)),
                origin=Origin(xyz=(x, 0.0, -0.010)),
                material=stainless_mat,
                name="front_baseplate",
            )
            deck.visual(
                Cylinder(radius=0.020, length=0.008),
                origin=Origin(xyz=(x, 0.0, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=gasket_mat,
                name="front_pivot_cup",
            )
        else:
            deck.visual(
                Box((0.138, 0.082, 0.006)),
                origin=Origin(xyz=(x, 0.0, -0.003)),
                material=gasket_mat,
                name="rear_sealed_pad",
            )
            deck.visual(
                Box((0.112, 0.064, 0.008)),
                origin=Origin(xyz=(x, 0.0, -0.010)),
                material=stainless_mat,
                name="rear_baseplate",
            )
            deck.visual(
                Cylinder(radius=0.020, length=0.008),
                origin=Origin(xyz=(x, 0.0, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=gasket_mat,
                name="rear_pivot_cup",
            )
        for i, (dx, dy) in enumerate(bolt_offsets):
            bx = x + dx
            by = dy
            deck.visual(
                Cylinder(radius=0.006, length=0.034),
                origin=Origin(xyz=(bx, by, 0.008), rpy=(0.0, 0.0, 0.0)),
                material=stainless_mat,
                name=f"{station}_bolt_shank_{i}",
            )
            deck.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(bx, by, 0.0265), rpy=(0.0, 0.0, 0.0)),
                material=stainless_mat,
                name=f"{station}_sealed_bolt_{i}",
            )
            deck.visual(
                Cylinder(radius=0.009, length=0.003),
                origin=Origin(xyz=(bx, by, -0.0155), rpy=(0.0, 0.0, 0.0)),
                material=stainless_mat,
                name=f"{station}_washer_{i}",
            )

    # Wheel meshes: rugged outdoor urethane over a corrosion-resistant core with
    # a visible bore for the stainless axle and sealed bearing shields.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.026,
            0.038,
            rim=WheelRim(inner_radius=0.016, flange_height=0.0025, flange_thickness=0.002),
            hub=WheelHub(radius=0.015, width=0.026, cap_style="flat"),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "sealed_wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.036,
            0.044,
            inner_radius=0.0255,
            carcass=TireCarcass(belt_width_ratio=0.78, sidewall_bulge=0.025),
            tread=TireTread(style="block", depth=0.0025, count=28, land_ratio=0.64),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.030),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "outdoor_urethane_tire",
    )

    truck_parts = {}
    for station, x in truck_xs.items():
        sign = 1.0 if x > 0.0 else -1.0
        # The kingpin/steer axis leans inward toward the board center.
        steer_axis = (-sign * 0.34, 0.0, 0.94)
        axis_len = math.sqrt(sum(v * v for v in steer_axis))
        steer_axis = tuple(v / axis_len for v in steer_axis)  # type: ignore[assignment]
        axis_rpy = _tilt_rpy_for_axis(steer_axis)  # type: ignore[arg-type]

        truck = model.part(f"{station}_truck")
        truck_parts[station] = truck
        truck.visual(
            Cylinder(radius=0.0055, length=0.386),
            origin=Origin(xyz=(0.0, 0.0, -0.034), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless_mat,
            name="axle",
        )
        truck.visual(
            Box((0.088, 0.172, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.036)),
            material=aluminum_mat,
            name="hanger_body",
        )
        truck.visual(
            Box((0.038, 0.222, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=aluminum_mat,
            name="hanger_yoke",
        )
        truck.visual(
            Cylinder(radius=0.012, length=0.048),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=axis_rpy),
            material=stainless_mat,
            name="kingpin",
        )
        truck.visual(
            Cylinder(radius=0.022, length=0.015),
            origin=Origin(
                xyz=(steer_axis[0] * 0.009, 0.0, steer_axis[2] * 0.009),
                rpy=axis_rpy,
            ),
            material=rubber_mat,
            name="top_bushing",
        )
        truck.visual(
            Cylinder(radius=0.023, length=0.017),
            origin=Origin(
                xyz=(-steer_axis[0] * 0.012, 0.0, -steer_axis[2] * 0.012),
                rpy=axis_rpy,
            ),
            material=urethane_mat,
            name="bottom_bushing",
        )
        truck.visual(
            Cylinder(radius=0.024, length=0.0035),
            origin=Origin(
                xyz=(steer_axis[0] * 0.017, 0.0, steer_axis[2] * 0.017),
                rpy=axis_rpy,
            ),
            material=stainless_mat,
            name="top_cup_washer",
        )
        truck.visual(
            Cylinder(radius=0.024, length=0.0035),
            origin=Origin(
                xyz=(steer_axis[0] * -0.026, 0.0, steer_axis[2] * -0.026),
                rpy=axis_rpy,
            ),
            material=stainless_mat,
            name="bottom_cup_washer",
        )
        # Inner wheel spacers and outer nyloc nuts capture each wheel on the axle.
        for side_idx, ysign in enumerate((-1.0, 1.0)):
            truck.visual(
                Cylinder(radius=0.013, length=0.004),
                origin=Origin(xyz=(0.0, ysign * 0.136, -0.034), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=stainless_mat,
                name=f"inner_spacer_{side_idx}",
            )
            truck.visual(
                Cylinder(radius=0.014, length=0.007),
                origin=Origin(xyz=(0.0, ysign * 0.186, -0.034), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=stainless_mat,
                name=f"axle_nut_{side_idx}",
            )

        model.articulation(
            f"deck_to_{station}_truck",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=truck,
            origin=Origin(xyz=(x, 0.0, -0.040)),
            axis=steer_axis,  # type: ignore[arg-type]
            motion_limits=MotionLimits(effort=28.0, velocity=2.2, lower=-0.38, upper=0.38),
            motion_properties=MotionProperties(damping=0.6, friction=0.08),
        )

        for side_idx, ysign in enumerate((-1.0, 1.0)):
            wheel = model.part(f"{station}_wheel_{side_idx}")
            wheel.visual(
                tire_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
                material=urethane_mat,
                name="tire",
            )
            wheel.visual(
                wheel_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
                material=aluminum_mat,
                name="wheel_core",
            )
            for cap_y, cap_name in ((-0.0140, "bearing_shield_0"), (0.0140, "bearing_shield_1")):
                wheel.visual(
                    Cylinder(radius=0.0145, length=0.0025),
                    origin=Origin(xyz=(0.0, cap_y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                    material=bearing_mat,
                    name=cap_name,
                )
            model.articulation(
                f"{station}_wheel_{side_idx}_spin",
                ArticulationType.CONTINUOUS,
                parent=truck,
                child=wheel,
                origin=Origin(xyz=(0.0, ysign * 0.160, -0.034)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=60.0),
                motion_properties=MotionProperties(damping=0.015, friction=0.01),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_wheel = object_model.get_part("front_wheel_1")
    rear_wheel = object_model.get_part("rear_wheel_0")
    front_steer = object_model.get_articulation("deck_to_front_truck")
    front_spin = object_model.get_articulation("front_wheel_1_spin")

    ctx.allow_overlap(
        deck,
        front_truck,
        elem_a="front_pivot_cup",
        elem_b="kingpin",
        reason="The stainless kingpin is intentionally captured inside the sealed elastomer pivot cup.",
    )
    ctx.allow_overlap(
        deck,
        rear_truck,
        elem_a="rear_pivot_cup",
        elem_b="kingpin",
        reason="The stainless kingpin is intentionally captured inside the sealed elastomer pivot cup.",
    )
    ctx.allow_overlap(
        deck,
        front_truck,
        elem_a="front_pivot_cup",
        elem_b="top_bushing",
        reason="The upper bushing is intentionally nested into the sealed pivot-cup lip for weatherproof compression.",
    )
    ctx.allow_overlap(
        deck,
        rear_truck,
        elem_a="rear_pivot_cup",
        elem_b="top_bushing",
        reason="The upper bushing is intentionally nested into the sealed pivot-cup lip for weatherproof compression.",
    )
    ctx.allow_overlap(
        deck,
        front_truck,
        elem_a="front_pivot_cup",
        elem_b="top_cup_washer",
        reason="The cupped stainless washer is intentionally tucked into the pivot-cup seal to clamp out water.",
    )
    ctx.allow_overlap(
        deck,
        rear_truck,
        elem_a="rear_pivot_cup",
        elem_b="top_cup_washer",
        reason="The cupped stainless washer is intentionally tucked into the pivot-cup seal to clamp out water.",
    )
    ctx.expect_overlap(
        front_truck,
        deck,
        axes="z",
        elem_a="kingpin",
        elem_b="front_pivot_cup",
        min_overlap=0.020,
        name="front kingpin remains retained in the weather seal",
    )
    ctx.expect_overlap(
        rear_truck,
        deck,
        axes="z",
        elem_a="kingpin",
        elem_b="rear_pivot_cup",
        min_overlap=0.020,
        name="rear kingpin remains retained in the weather seal",
    )
    ctx.expect_overlap(
        front_truck,
        deck,
        axes="xy",
        elem_a="top_bushing",
        elem_b="front_pivot_cup",
        min_overlap=0.006,
        name="front upper bushing seats in the pivot seal",
    )
    ctx.expect_overlap(
        rear_truck,
        deck,
        axes="xy",
        elem_a="top_bushing",
        elem_b="rear_pivot_cup",
        min_overlap=0.006,
        name="rear upper bushing seats in the pivot seal",
    )
    ctx.expect_overlap(
        front_truck,
        deck,
        axes="xy",
        elem_a="top_cup_washer",
        elem_b="front_pivot_cup",
        min_overlap=0.006,
        name="front cup washer seats in the pivot seal",
    )
    ctx.expect_overlap(
        rear_truck,
        deck,
        axes="xy",
        elem_a="top_cup_washer",
        elem_b="rear_pivot_cup",
        min_overlap=0.006,
        name="rear cup washer seats in the pivot seal",
    )

    ctx.expect_overlap(
        front_truck,
        deck,
        axes="xy",
        elem_a="top_cup_washer",
        elem_b="front_baseplate",
        min_overlap=0.018,
        name="front bushing stack is under the sealed baseplate",
    )
    ctx.expect_overlap(
        rear_truck,
        deck,
        axes="xy",
        elem_a="top_cup_washer",
        elem_b="rear_baseplate",
        min_overlap=0.018,
        name="rear bushing stack is under the sealed baseplate",
    )
    ctx.expect_overlap(
        front_wheel,
        front_truck,
        axes="y",
        elem_a="wheel_core",
        elem_b="axle",
        min_overlap=0.035,
        name="front wheel is captured along the axle",
    )
    ctx.expect_overlap(
        rear_wheel,
        rear_truck,
        axes="y",
        elem_a="wheel_core",
        elem_b="axle",
        min_overlap=0.035,
        name="rear wheel is captured along the axle",
    )

    rest_aabb = ctx.part_world_aabb(front_wheel)
    with ctx.pose({front_spin: 1.25}):
        spun_aabb = ctx.part_world_aabb(front_wheel)
    ctx.check(
        "wheel spin keeps wheel on axle center",
        rest_aabb is not None
        and spun_aabb is not None
        and abs((rest_aabb[0][1] + rest_aabb[1][1]) - (spun_aabb[0][1] + spun_aabb[1][1])) < 0.002,
        details=f"rest={rest_aabb}, spun={spun_aabb}",
    )

    rest_wheel_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({front_steer: 0.30}):
        steered_wheel_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "truck steer pivots wheel about the kingpin",
        rest_wheel_pos is not None
        and steered_wheel_pos is not None
        and abs(steered_wheel_pos[0] - rest_wheel_pos[0]) > 0.025,
        details=f"rest={rest_wheel_pos}, steered={steered_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
