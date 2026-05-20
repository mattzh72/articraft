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
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


DECK_LENGTH = 0.82
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
TRUCK_X = 0.245
DECK_TOP_BASE_Z = 0.014


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _deck_half_width(x: float, *, inset: float = 0.0) -> float:
    half_length = DECK_LENGTH * 0.5 - inset
    half_width = DECK_WIDTH * 0.5 - inset
    cap_center = max(0.0, half_length - half_width)
    ax = abs(x)
    if ax <= cap_center:
        return max(0.004, half_width)
    dx = min(half_width, ax - cap_center)
    return max(0.004, math.sqrt(max(0.0, half_width * half_width - dx * dx)))


def _deck_top_z(x: float, y: float) -> float:
    half_width = _deck_half_width(x)
    y_frac = min(1.0, abs(y) / max(half_width, 0.004))
    edge_concave = 0.0060 * (y_frac**1.7)
    kick = 0.034 * (_smoothstep((abs(x) - 0.275) / 0.130) ** 1.15)
    mild_camber = 0.0012 * math.cos(math.pi * x / DECK_LENGTH)
    return DECK_TOP_BASE_Z + edge_concave + kick + mild_camber


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_deck_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    x_count = 35
    y_count = 17
    top_ids: list[list[int]] = []
    bottom_ids: list[list[int]] = []

    for ix in range(x_count):
        x = -DECK_LENGTH * 0.5 + DECK_LENGTH * ix / (x_count - 1)
        half_width = _deck_half_width(x)
        row_top: list[int] = []
        row_bottom: list[int] = []
        for iy in range(y_count):
            frac = -1.0 + 2.0 * iy / (y_count - 1)
            y = half_width * frac
            top_z = _deck_top_z(x, y)
            row_top.append(geom.add_vertex(x, y, top_z))
            row_bottom.append(geom.add_vertex(x, y, top_z - DECK_THICKNESS))
        top_ids.append(row_top)
        bottom_ids.append(row_bottom)

    for ix in range(x_count - 1):
        for iy in range(y_count - 1):
            _add_quad(geom, top_ids[ix][iy], top_ids[ix + 1][iy], top_ids[ix + 1][iy + 1], top_ids[ix][iy + 1])
            _add_quad(geom, bottom_ids[ix][iy + 1], bottom_ids[ix + 1][iy + 1], bottom_ids[ix + 1][iy], bottom_ids[ix][iy])

    for ix in range(x_count - 1):
        _add_quad(geom, top_ids[ix][0], bottom_ids[ix][0], bottom_ids[ix + 1][0], top_ids[ix + 1][0])
        _add_quad(
            geom,
            top_ids[ix + 1][y_count - 1],
            bottom_ids[ix + 1][y_count - 1],
            bottom_ids[ix][y_count - 1],
            top_ids[ix][y_count - 1],
        )
    for iy in range(y_count - 1):
        _add_quad(geom, top_ids[0][iy + 1], bottom_ids[0][iy + 1], bottom_ids[0][iy], top_ids[0][iy])
        _add_quad(
            geom,
            top_ids[x_count - 1][iy],
            bottom_ids[x_count - 1][iy],
            bottom_ids[x_count - 1][iy + 1],
            top_ids[x_count - 1][iy + 1],
        )
    return geom


def _build_grip_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    x_min = -0.350
    x_max = 0.350
    x_count = 29
    y_count = 13
    top_ids: list[list[int]] = []
    bottom_ids: list[list[int]] = []

    for ix in range(x_count):
        x = x_min + (x_max - x_min) * ix / (x_count - 1)
        half_width = min(0.085, _deck_half_width(x) - 0.012)
        row_top: list[int] = []
        row_bottom: list[int] = []
        for iy in range(y_count):
            frac = -1.0 + 2.0 * iy / (y_count - 1)
            y = half_width * frac
            deck_z = _deck_top_z(x, y)
            row_top.append(geom.add_vertex(x, y, deck_z + 0.0014))
            row_bottom.append(geom.add_vertex(x, y, deck_z - 0.0002))
        top_ids.append(row_top)
        bottom_ids.append(row_bottom)

    for ix in range(x_count - 1):
        for iy in range(y_count - 1):
            _add_quad(geom, top_ids[ix][iy], top_ids[ix + 1][iy], top_ids[ix + 1][iy + 1], top_ids[ix][iy + 1])
            _add_quad(geom, bottom_ids[ix][iy + 1], bottom_ids[ix + 1][iy + 1], bottom_ids[ix + 1][iy], bottom_ids[ix][iy])

    for ix in range(x_count - 1):
        _add_quad(geom, top_ids[ix][0], bottom_ids[ix][0], bottom_ids[ix + 1][0], top_ids[ix + 1][0])
        _add_quad(geom, top_ids[ix + 1][-1], bottom_ids[ix + 1][-1], bottom_ids[ix][-1], top_ids[ix][-1])
    for iy in range(y_count - 1):
        _add_quad(geom, top_ids[0][iy + 1], bottom_ids[0][iy + 1], bottom_ids[0][iy], top_ids[0][iy])
        _add_quad(geom, top_ids[-1][iy], bottom_ids[-1][iy], bottom_ids[-1][iy + 1], top_ids[-1][iy + 1])
    return geom


def _build_hanger_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    profile = [
        (-0.030, -0.048),
        (-0.018, -0.059),
        (0.018, -0.059),
        (0.030, -0.048),
        (0.026, -0.025),
        (0.000, -0.018),
        (-0.026, -0.025),
    ]
    y_front = 0.045
    y_back = -0.045
    front = [geom.add_vertex(x, y_front, z) for x, z in profile]
    back = [geom.add_vertex(x, y_back, z) for x, z in profile]
    n = len(profile)
    for index in range(n):
        _add_quad(geom, front[index], back[index], back[(index + 1) % n], front[(index + 1) % n])
    for index in range(1, n - 1):
        geom.add_face(front[0], front[index], front[index + 1])
        geom.add_face(back[0], back[index + 1], back[index])
    return geom


def _add_mount_bolts(
    deck_part,
    *,
    truck_x: float,
    material: Material,
) -> None:
    for dx in (-0.022, 0.022):
        for y in (-0.031, 0.031):
            x = truck_x + dx
            z = _deck_top_z(x, y)
            deck_part.visual(
                Cylinder(radius=0.0048, length=0.0024),
                origin=Origin(xyz=(x, y, z + 0.0010)),
                material=material,
                name=f"bolt_{'front' if truck_x > 0 else 'rear'}_{dx:+.3f}_{y:+.3f}",
            )


def _build_wheel_meshes():
    urethane = TireGeometry(
        0.027,
        0.036,
        inner_radius=0.017,
        carcass=TireCarcass(belt_width_ratio=0.76, sidewall_bulge=0.045),
        sidewall=TireSidewall(style="rounded", bulge=0.045),
        shoulder=TireShoulder(width=0.004, radius=0.003),
    )
    hub = WheelGeometry(
        0.0185,
        0.038,
        rim=WheelRim(inner_radius=0.013, flange_height=0.0022, flange_thickness=0.0015),
        hub=WheelHub(
            radius=0.0095,
            width=0.030,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.014, hole_diameter=0.0018),
        ),
        face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.0016, window_radius=0.0035),
        bore=WheelBore(style="round", diameter=0.0085),
    )
    return mesh_from_geometry(urethane, "skateboard_urethane_wheel"), mesh_from_geometry(hub, "skateboard_bearing_hub")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_skateboard")

    maple = model.material("satin_maple_edge", rgba=(0.62, 0.46, 0.27, 1.0))
    grip = model.material("matte_micro_grip", rgba=(0.025, 0.026, 0.027, 1.0))
    hardware = model.material("smoked_steel_hardware", rgba=(0.09, 0.095, 0.10, 1.0))
    aluminum = model.material("satin_forged_aluminum", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_aluminum = model.material("brushed_dark_aluminum", rgba=(0.30, 0.31, 0.31, 1.0))
    bushing_mat = model.material("translucent_smoke_bushing", rgba=(0.18, 0.16, 0.13, 0.90))
    riser_mat = model.material("matte_black_riser", rgba=(0.035, 0.036, 0.038, 1.0))
    urethane_mat = model.material("satin_ivory_urethane", rgba=(0.91, 0.86, 0.74, 1.0))
    hub_mat = model.material("satin_titanium_hub", rgba=(0.70, 0.68, 0.63, 1.0))

    deck = model.part("deck")
    deck.visual(mesh_from_geometry(_build_deck_mesh(), "premium_skateboard_deck"), material=maple, name="deck_body")
    deck.visual(mesh_from_geometry(_build_grip_mesh(), "premium_skateboard_grip"), material=grip, name="grip_tape")
    deck.visual(
        Box((0.010, 0.172, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, _deck_top_z(0.0, 0.0) + 0.0019)),
        material=hardware,
        name="center_seam",
    )
    _add_mount_bolts(deck, truck_x=TRUCK_X, material=hardware)
    _add_mount_bolts(deck, truck_x=-TRUCK_X, material=hardware)

    wheel_urethane_mesh, wheel_hub_mesh = _build_wheel_meshes()
    hanger_mesh = mesh_from_geometry(_build_hanger_mesh(), "skateboard_truck_hanger")

    def build_truck(prefix: str, x: float, kingpin_sign: float):
        underside_z = _deck_top_z(x, 0.0) - DECK_THICKNESS
        baseplate = model.part(f"{prefix}_baseplate")
        baseplate.visual(
            Box((0.106, 0.078, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=riser_mat,
            name="riser_pad",
        )
        baseplate.visual(
            Box((0.092, 0.064, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=dark_aluminum,
            name="baseplate_body",
        )
        baseplate.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=aluminum,
            name="kingpin_socket",
        )
        baseplate.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(xyz=(kingpin_sign * 0.014, 0.0, -0.017), rpy=(0.0, kingpin_sign * 0.24, 0.0)),
            material=hardware,
            name="kingpin_head",
        )
        for dx in (-0.022, 0.022):
            for y in (-0.031, 0.031):
                baseplate.visual(
                    Cylinder(radius=0.0042, length=0.0022),
                    origin=Origin(xyz=(dx, y, -0.0131)),
                    material=hardware,
                    name=f"mount_nut_{dx:+.3f}_{y:+.3f}",
                )
        model.articulation(
            f"deck_to_{prefix}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=baseplate,
            origin=Origin(xyz=(x, 0.0, underside_z)),
        )

        hanger = model.part(f"{prefix}_hanger")
        hanger.visual(hanger_mesh, material=aluminum, name="hanger_body")
        hanger.visual(
            Cylinder(radius=0.00425, length=0.296),
            origin=Origin(xyz=(0.0, 0.0, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name="axle",
        )
        for y in (-0.088, 0.088):
            hanger.visual(
                Cylinder(radius=0.0070, length=0.014),
                origin=Origin(xyz=(0.0, y, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_aluminum,
                name=f"axle_collar_{y:+.3f}",
            )
        for y in (-0.150, 0.150):
            hanger.visual(
                Cylinder(radius=0.0075, length=0.007),
                origin=Origin(xyz=(0.0, y, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=hardware,
                name=f"axle_nut_{y:+.3f}",
            )
        hanger.visual(
            Cylinder(radius=0.0175, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.000)),
            material=bushing_mat,
            name="upper_bushing",
        )
        hanger.visual(
            Cylinder(radius=0.0160, length=0.013),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=bushing_mat,
            name="lower_bushing",
        )
        hanger.visual(
            Cylinder(radius=0.0040, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=hardware,
            name="kingpin_shaft",
        )
        hanger.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=hardware,
            name="kingpin_nut",
        )
        steer_axis = (0.26 * kingpin_sign, 0.0, 0.9656)
        model.articulation(
            f"{prefix}_baseplate_to_hanger",
            ArticulationType.REVOLUTE,
            parent=baseplate,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, -0.031)),
            axis=steer_axis,
            motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.30, upper=0.30),
        )

        for index, y in enumerate((-0.128, 0.128)):
            wheel = model.part(f"{prefix}_wheel_{index}")
            wheel.visual(
                wheel_urethane_mesh,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=urethane_mat,
                name="urethane_tire",
            )
            wheel.visual(
                wheel_hub_mesh,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=hub_mat,
                name="bearing_hub",
            )
            model.articulation(
                f"{prefix}_hanger_to_wheel_{index}",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, y, -0.052)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.5, velocity=35.0),
            )

    build_truck("front", TRUCK_X, kingpin_sign=-1.0)
    build_truck("rear", -TRUCK_X, kingpin_sign=1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_baseplate = object_model.get_part("front_baseplate")
    rear_baseplate = object_model.get_part("rear_baseplate")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_steer = object_model.get_articulation("front_baseplate_to_hanger")
    rear_steer = object_model.get_articulation("rear_baseplate_to_hanger")

    for prefix, hanger in (("front", front_hanger), ("rear", rear_hanger)):
        for index in (0, 1):
            wheel = object_model.get_part(f"{prefix}_wheel_{index}")
            ctx.allow_overlap(
                hanger,
                wheel,
                elem_a="axle",
                elem_b="bearing_hub",
                reason="The visible steel axle is intentionally captured through the simplified bearing hub proxy.",
            )
            ctx.expect_within(
                hanger,
                wheel,
                axes="xz",
                inner_elem="axle",
                outer_elem="bearing_hub",
                margin=0.001,
                name=f"{prefix} wheel {index} bearing surrounds axle",
            )
            ctx.expect_overlap(
                hanger,
                wheel,
                axes="y",
                elem_a="axle",
                elem_b="bearing_hub",
                min_overlap=0.032,
                name=f"{prefix} wheel {index} axle spans bearing width",
            )

    ctx.expect_gap(
        deck,
        front_baseplate,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0002,
        positive_elem="deck_body",
        negative_elem="riser_pad",
        name="front riser seats under deck without cutting through it",
    )
    ctx.expect_gap(
        deck,
        rear_baseplate,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0002,
        positive_elem="deck_body",
        negative_elem="riser_pad",
        name="rear riser seats under deck without cutting through it",
    )
    ctx.expect_origin_gap(front_baseplate, rear_baseplate, axis="x", min_gap=0.45, max_gap=0.55, name="trucks are spaced like a full skateboard")
    ctx.expect_overlap(front_hanger, front_baseplate, axes="xy", min_overlap=0.035, name="front hanger stays centered below baseplate")
    ctx.expect_overlap(rear_hanger, rear_baseplate, axes="xy", min_overlap=0.035, name="rear hanger stays centered below baseplate")

    front_rest_aabb = ctx.part_world_aabb(front_hanger)
    rear_rest_aabb = ctx.part_world_aabb(rear_hanger)
    with ctx.pose({front_steer: 0.24, rear_steer: -0.24}):
        front_turned_aabb = ctx.part_world_aabb(front_hanger)
        rear_turned_aabb = ctx.part_world_aabb(rear_hanger)
        ctx.expect_overlap(front_hanger, front_baseplate, axes="xy", min_overlap=0.030, name="front steering pivot remains retained")
        ctx.expect_overlap(rear_hanger, rear_baseplate, axes="xy", min_overlap=0.030, name="rear steering pivot remains retained")

    def _aabb_x_span(aabb) -> float | None:
        if aabb is None:
            return None
        return aabb[1][0] - aabb[0][0]

    ctx.check(
        "truck pivot pose changes hanger silhouette",
        front_rest_aabb is not None
        and rear_rest_aabb is not None
        and front_turned_aabb is not None
        and rear_turned_aabb is not None
        and _aabb_x_span(front_turned_aabb) is not None
        and _aabb_x_span(front_rest_aabb) is not None
        and _aabb_x_span(front_turned_aabb) > _aabb_x_span(front_rest_aabb) + 0.010
        and _aabb_x_span(rear_turned_aabb) > _aabb_x_span(rear_rest_aabb) + 0.010,
        details=f"front_rest={front_rest_aabb}, front_turned={front_turned_aabb}, rear_rest={rear_rest_aabb}, rear_turned={rear_turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
