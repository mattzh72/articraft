from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
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
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


DECK_LENGTH = 0.82
DECK_WIDTH = 0.224
DECK_TOP_Z = 0.095
DECK_THICKNESS = 0.012
TRUCK_X = 0.270
TRUCK_MOUNT_Z = DECK_TOP_Z - DECK_THICKNESS
WHEEL_CENTER_Y = 0.154
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.038


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _deck_half_width(x: float) -> float:
    """Premium popsicle deck outline: waisted center, rounded nose and tail."""
    ax = abs(x)
    half = DECK_WIDTH * 0.5
    shoulder = 0.315
    end = DECK_LENGTH * 0.5
    if ax <= shoulder:
        # A subtle waist at center, fuller shoulders near the trucks.
        return half * (0.910 + 0.060 * (ax / shoulder) ** 1.7)
    t = (ax - shoulder) / (end - shoulder)
    # Keep a small rounded cap width at the extreme nose/tail rather than a point.
    return half * (0.185 + 0.785 * math.cos(t * math.pi * 0.5) ** 0.72)


def _deck_top_z(x: float, y: float) -> float:
    w = max(_deck_half_width(x), 1e-6)
    lateral = min(1.0, abs(y) / w)
    concave = 0.0052 * lateral**2
    tail_t = _smoothstep((abs(x) - 0.285) / (DECK_LENGTH * 0.5 - 0.285))
    kick = 0.030 * tail_t**1.55
    return DECK_TOP_Z + concave + kick


def _make_deck_geometry(nx: int = 41, ny: int = 19) -> MeshGeometry:
    geom = MeshGeometry()
    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for i in range(nx):
        x = -DECK_LENGTH * 0.5 + DECK_LENGTH * i / (nx - 1)
        w = _deck_half_width(x)
        top_row: list[int] = []
        bottom_row: list[int] = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * w
            zt = _deck_top_z(x, y)
            # Slightly thicker rails make the laminated edge read as structural.
            rail_build = 0.0012 * abs(v) ** 1.7
            zb = zt - DECK_THICKNESS - rail_build
            top_row.append(geom.add_vertex(x, y, zt))
            bottom_row.append(geom.add_vertex(x, y, zb))
        top.append(top_row)
        bottom.append(bottom_row)

    for i in range(nx - 1):
        for j in range(ny - 1):
            t00, t10, t11, t01 = top[i][j], top[i + 1][j], top[i + 1][j + 1], top[i][j + 1]
            b00, b10, b11, b01 = bottom[i][j], bottom[i + 1][j], bottom[i + 1][j + 1], bottom[i][j + 1]
            geom.add_face(t00, t10, t11)
            geom.add_face(t00, t11, t01)
            geom.add_face(b00, b11, b10)
            geom.add_face(b00, b01, b11)

    # Rails along the long sides.
    for i in range(nx - 1):
        for j in (0, ny - 1):
            t0, t1 = top[i][j], top[i + 1][j]
            b0, b1 = bottom[i][j], bottom[i + 1][j]
            if j == 0:
                geom.add_face(t0, b1, t1)
                geom.add_face(t0, b0, b1)
            else:
                geom.add_face(t0, t1, b1)
                geom.add_face(t0, b1, b0)

    # Rounded nose/tail cap surfaces.
    for i in (0, nx - 1):
        for j in range(ny - 1):
            t0, t1 = top[i][j], top[i][j + 1]
            b0, b1 = bottom[i][j], bottom[i][j + 1]
            if i == 0:
                geom.add_face(t0, t1, b1)
                geom.add_face(t0, b1, b0)
            else:
                geom.add_face(t0, b1, t1)
                geom.add_face(t0, b0, b1)
    return geom


def _make_grip_geometry(x_min: float, x_max: float, y_scale: float, name_offset: float = 0.0) -> MeshGeometry:
    """Thin adhered grip strip following the deck crown, slightly embedded at the bottom."""
    geom = MeshGeometry()
    nx, ny = 29, 9
    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for i in range(nx):
        x = x_min + (x_max - x_min) * i / (nx - 1)
        w = _deck_half_width(x) * y_scale
        top_row: list[int] = []
        bottom_row: list[int] = []
        for j in range(ny):
            v = -1.0 + 2.0 * j / (ny - 1)
            y = v * w + name_offset
            z = _deck_top_z(x, y)
            top_row.append(geom.add_vertex(x, y, z + 0.00055))
            bottom_row.append(geom.add_vertex(x, y, z - 0.00015))
        top.append(top_row)
        bottom.append(bottom_row)
    for i in range(nx - 1):
        for j in range(ny - 1):
            t00, t10, t11, t01 = top[i][j], top[i + 1][j], top[i + 1][j + 1], top[i][j + 1]
            b00, b10, b11, b01 = bottom[i][j], bottom[i + 1][j], bottom[i + 1][j + 1], bottom[i][j + 1]
            geom.add_face(t00, t10, t11)
            geom.add_face(t00, t11, t01)
            geom.add_face(b00, b11, b10)
            geom.add_face(b00, b01, b11)
    for i in range(nx - 1):
        for j in (0, ny - 1):
            t0, t1 = top[i][j], top[i + 1][j]
            b0, b1 = bottom[i][j], bottom[i + 1][j]
            geom.add_face(t0, t1, b1)
            geom.add_face(t0, b1, b0)
    for i in (0, nx - 1):
        for j in range(ny - 1):
            t0, t1 = top[i][j], top[i][j + 1]
            b0, b1 = bottom[i][j], bottom[i][j + 1]
            geom.add_face(t0, t1, b1)
            geom.add_face(t0, b1, b0)
    return geom


def _rounded_plate(width: float, depth: float, height: float, radius: float) -> MeshGeometry:
    return ExtrudeGeometry(rounded_rect_profile(width, depth, radius, corner_segments=8), height, center=True)


def _axis_pitch(axis: tuple[float, float, float]) -> float:
    ax, _, az = axis
    return math.atan2(ax, az)


def _axis_unit(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(sum(a * a for a in axis))
    return (axis[0] / n, axis[1] / n, axis[2] / n)


def _add_axis_cylinder(part, *, name: str, radius: float, length: float, axis, offset: float, material: Material) -> None:
    ux, uy, uz = _axis_unit(axis)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(ux * offset, uy * offset, uz * offset), rpy=(0.0, _axis_pitch(axis), 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_skateboard")

    maple = model.material("sealed_maple_edge", rgba=(0.78, 0.58, 0.34, 1.0))
    grip = model.material("micro_grit_black", rgba=(0.015, 0.016, 0.017, 1.0))
    stringer = model.material("maple_stringer", rgba=(0.88, 0.70, 0.43, 1.0))
    metal = model.material("satin_painted_metal", rgba=(0.44, 0.46, 0.47, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    pad = model.material("black_polymer_riser", rgba=(0.025, 0.025, 0.028, 1.0))
    bushing = model.material("smoked_elastomer", rgba=(0.92, 0.54, 0.18, 0.88))
    urethane = model.material("translucent_urethane", rgba=(0.92, 0.86, 0.66, 0.78))
    hub_white = model.material("warm_white_polymer", rgba=(0.92, 0.90, 0.84, 1.0))

    deck = model.part("deck")
    deck.visual(mesh_from_geometry(_make_deck_geometry(), "deck_shell"), material=maple, name="deck_shell")
    deck.visual(
        mesh_from_geometry(_make_grip_geometry(-0.350, 0.350, 0.835), "grip_sheet"),
        material=grip,
        name="grip_sheet",
    )
    deck.visual(
        Box((0.565, 0.007, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0, _deck_top_z(0.0, 0.0) + 0.0010)),
        material=stringer,
        name="center_stringer",
    )
    for x in (-TRUCK_X, TRUCK_X):
        for dx in (-0.031, 0.031):
            for y in (-0.023, 0.023):
                deck.visual(
                    Cylinder(radius=0.0042, length=0.0022),
                    origin=Origin(xyz=(x + dx, y, _deck_top_z(x + dx, y) + 0.0014)),
                    material=steel,
                    name=f"bolt_head_{len(deck.visuals)}",
                )

    def add_truck(prefix: str, x_pos: float, steer_axis: tuple[float, float, float]) -> None:
        baseplate = model.part(f"{prefix}_baseplate")
        baseplate.visual(
            mesh_from_geometry(_rounded_plate(0.132, 0.086, 0.004, 0.012), f"{prefix}_riser_pad"),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=pad,
            name="riser_pad",
        )
        baseplate.visual(
            mesh_from_geometry(_rounded_plate(0.112, 0.070, 0.008, 0.010), f"{prefix}_baseplate"),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=metal,
            name="baseplate",
        )
        baseplate.visual(
            Cylinder(radius=0.019, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=metal,
            name="kingpin_cup",
        )
        for dx in (-0.031, 0.031):
            for y in (-0.023, 0.023):
                baseplate.visual(
                    Cylinder(radius=0.0050, length=0.0030),
                    origin=Origin(xyz=(dx, y, -0.0135)),
                    material=steel,
                    name=f"mount_nut_{len(baseplate.visuals)}",
                )

        model.articulation(
            f"deck_to_{prefix}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=baseplate,
            origin=Origin(xyz=(x_pos, 0.0, TRUCK_MOUNT_Z)),
        )

        hanger = model.part(f"{prefix}_hanger")
        pitch = _axis_pitch(steer_axis)
        _add_axis_cylinder(
            hanger,
            name="bushing_stack",
            radius=0.016,
            length=0.034,
            axis=steer_axis,
            offset=-0.004,
            material=bushing,
        )
        _add_axis_cylinder(
            hanger,
            name="upper_washer",
            radius=0.018,
            length=0.003,
            axis=steer_axis,
            offset=0.014,
            material=steel,
        )
        _add_axis_cylinder(
            hanger,
            name="lower_washer",
            radius=0.018,
            length=0.003,
            axis=steer_axis,
            offset=-0.022,
            material=steel,
        )
        hanger.visual(
            Box((0.062, 0.064, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=metal,
            name="central_yoke",
        )
        hanger.visual(
            Cylinder(radius=0.013, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=metal,
            name="kingpin_neck",
        )
        hanger.visual(
            Cylinder(radius=0.014, length=0.215),
            origin=Origin(xyz=(0.0, 0.0, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="hanger_bar",
        )
        hanger.visual(
            Cylinder(radius=0.0045, length=0.272),
            origin=Origin(xyz=(0.0, 0.0, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="axle",
        )
        hanger.visual(
            Box((0.050, 0.070, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=metal,
            name="underside_blend",
        )

        model.articulation(
            f"{prefix}_steer",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            axis=steer_axis,
            motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-0.35, upper=0.35),
            motion_properties=MotionProperties(damping=0.18, friction=0.04),
        )

        tire_mesh = mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=0.015,
                carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
                grooves=(
                    TireGroove(center_offset=-0.010, width=0.0016, depth=0.0008),
                    TireGroove(center_offset=0.010, width=0.0016, depth=0.0008),
                ),
                sidewall=TireSidewall(style="rounded", bulge=0.040),
                shoulder=TireShoulder(width=0.004, radius=0.0025),
            ),
            f"{prefix}_wheel_tire",
        )
        core_mesh = mesh_from_geometry(
            WheelGeometry(
                0.0165,
                0.034,
                rim=WheelRim(
                    inner_radius=0.009,
                    flange_height=0.0025,
                    flange_thickness=0.002,
                    bead_seat_depth=0.001,
                ),
                hub=WheelHub(radius=0.0075, width=0.036, cap_style="domed"),
                face=WheelFace(dish_depth=0.0025, front_inset=0.0015, rear_inset=0.0010),
                spokes=WheelSpokes(style="split_y", count=5, thickness=0.0017, window_radius=0.0035),
                bore=WheelBore(style="round", diameter=0.008),
            ),
            f"{prefix}_wheel_core",
        )
        for idx, y_pos in enumerate((WHEEL_CENTER_Y, -WHEEL_CENTER_Y)):
            wheel = model.part(f"{prefix}_wheel_{idx}")
            wheel.visual(tire_mesh, material=urethane, name="urethane_tire")
            wheel.visual(core_mesh, material=hub_white, name="polymer_core")
            model.articulation(
                f"{prefix}_wheel_{idx}_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(
                    xyz=(0.0, y_pos, -0.030),
                    rpy=(0.0, 0.0, math.copysign(math.pi / 2.0, y_pos)),
                ),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=35.0),
                motion_properties=MotionProperties(damping=0.03, friction=0.01),
            )

    add_truck("front", TRUCK_X, (-0.350, 0.0, 0.937))
    add_truck("rear", -TRUCK_X, (0.350, 0.0, 0.937))
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")

    for prefix in ("front", "rear"):
        baseplate = object_model.get_part(f"{prefix}_baseplate")
        hanger = object_model.get_part(f"{prefix}_hanger")
        steer = object_model.get_articulation(f"{prefix}_steer")

        ctx.expect_gap(
            deck,
            baseplate,
            axis="z",
            positive_elem="deck_shell",
            negative_elem="riser_pad",
            max_gap=0.002,
            max_penetration=0.001,
            name=f"{prefix} riser is seated against the deck",
        )
        ctx.expect_gap(
            baseplate,
            hanger,
            axis="z",
            positive_elem="kingpin_cup",
            negative_elem="upper_washer",
            max_gap=0.003,
            max_penetration=0.008,
            name=f"{prefix} bushing stack is captured under cup",
        )
        ctx.check(
            f"{prefix} kingpin axis is visibly canted",
            abs(steer.axis[0]) > 0.25 and steer.axis[2] > 0.85,
            details=f"axis={steer.axis}",
        )

        wheel_pos = object_model.get_part(f"{prefix}_wheel_0")
        wheel_neg = object_model.get_part(f"{prefix}_wheel_1")
        ctx.expect_gap(
            wheel_pos,
            hanger,
            axis="y",
            positive_elem="polymer_core",
            negative_elem="axle",
            max_gap=0.003,
            max_penetration=0.003,
            name=f"{prefix} positive wheel sits on axle shoulder",
        )
        ctx.expect_gap(
            hanger,
            wheel_neg,
            axis="y",
            positive_elem="axle",
            negative_elem="polymer_core",
            max_gap=0.003,
            max_penetration=0.003,
            name=f"{prefix} negative wheel sits on axle shoulder",
        )

        spin = object_model.get_articulation(f"{prefix}_wheel_0_spin")
        rest_center = ctx.part_world_position(wheel_pos)
        with ctx.pose({spin: math.pi / 2.0}):
            spun_center = ctx.part_world_position(wheel_pos)
        center_shift = None
        if rest_center is not None and spun_center is not None:
            center_shift = math.sqrt(sum((a - b) ** 2 for a, b in zip(rest_center, spun_center)))
        ctx.check(
            f"{prefix} wheel spins about its axle center",
            center_shift is not None and center_shift < 1e-6,
            details=f"rest={rest_center}, spun={spun_center}, shift={center_shift}",
        )

        rest_wheel = ctx.part_world_position(wheel_pos)
        with ctx.pose({steer: 0.25}):
            steered_wheel = ctx.part_world_position(wheel_pos)
        steer_shift = None
        if rest_wheel is not None and steered_wheel is not None:
            steer_shift = math.sqrt(sum((a - b) ** 2 for a, b in zip(rest_wheel, steered_wheel)))
        ctx.check(
            f"{prefix} steering pivots wheel around bushing axis",
            steer_shift is not None and steer_shift > 0.004,
            details=f"rest={rest_wheel}, steered={steered_wheel}, shift={steer_shift}",
        )

    return ctx.report()


object_model = build_object_model()
