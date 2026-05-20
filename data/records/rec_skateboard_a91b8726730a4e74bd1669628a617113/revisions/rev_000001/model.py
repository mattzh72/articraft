from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _capsule_profile(length: float, width: float, *, segments: int = 20) -> list[tuple[float, float]]:
    """Rounded skateboard planform in local XY."""

    radius = width / 2.0
    straight = max(0.0, length - width)
    half_straight = straight / 2.0
    points: list[tuple[float, float]] = []

    for i in range(segments + 1):
        a = math.pi / 2.0 - math.pi * i / segments
        points.append((half_straight + radius * math.cos(a), radius * math.sin(a)))
    for i in range(segments + 1):
        a = -math.pi / 2.0 - math.pi * i / segments
        points.append((-half_straight + radius * math.cos(a), radius * math.sin(a)))
    return points


def _ring_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 48,
    outer_sides: int | None = None,
) -> MeshGeometry:
    """Closed annular prism centered on the origin, with its bore along local Z."""

    n = outer_sides or segments
    inner_n = n
    geom = MeshGeometry()
    rings: dict[str, list[int]] = {
        "outer_bottom": [],
        "outer_top": [],
        "inner_bottom": [],
        "inner_top": [],
    }
    z0 = -height / 2.0
    z1 = height / 2.0
    for i in range(n):
        a = 2.0 * math.pi * i / n
        ca, sa = math.cos(a), math.sin(a)
        rings["outer_bottom"].append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z0))
        rings["outer_top"].append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z1))
    for i in range(inner_n):
        a = 2.0 * math.pi * i / inner_n
        ca, sa = math.cos(a), math.sin(a)
        rings["inner_bottom"].append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z0))
        rings["inner_top"].append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z1))

    for i in range(n):
        j = (i + 1) % n
        ob0, ob1 = rings["outer_bottom"][i], rings["outer_bottom"][j]
        ot0, ot1 = rings["outer_top"][i], rings["outer_top"][j]
        ib0, ib1 = rings["inner_bottom"][i], rings["inner_bottom"][j]
        it0, it1 = rings["inner_top"][i], rings["inner_top"][j]

        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        geom.add_face(ib1, ib0, it0)
        geom.add_face(ib1, it0, it1)
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob1, ob0, ib0)
        geom.add_face(ob1, ib0, ib1)
    return geom


def _tilted_origin(
    base_xyz: tuple[float, float, float],
    local_xyz: tuple[float, float, float],
    tilt_y: float,
) -> Origin:
    """Place a feature whose local Z follows a tilted kingpin axis."""

    x, y, z = local_xyz
    c, s = math.cos(tilt_y), math.sin(tilt_y)
    return Origin(
        xyz=(base_xyz[0] + c * x + s * z, base_xyz[1] + y, base_xyz[2] - s * x + c * z),
        rpy=(0.0, tilt_y, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_skateboard")

    deck_mat = model.material("satin_carbon_deck", rgba=(0.035, 0.039, 0.045, 1.0))
    grip_mat = model.material("matte_black_grip", rgba=(0.006, 0.006, 0.006, 1.0))
    datum_mat = model.material("white_datum_ink", rgba=(0.92, 0.93, 0.88, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_metal = model.material("black_oxide_hardware", rgba=(0.035, 0.033, 0.031, 1.0))
    urethane_mat = model.material("translucent_blue_urethane", rgba=(0.10, 0.35, 0.95, 0.82))
    bushing_mat = model.material("amber_precision_bushing", rgba=(0.95, 0.56, 0.12, 1.0))

    deck_length = 0.820
    deck_width = 0.235
    deck_thickness = 0.014
    truck_x = 0.275
    base_top_z = -deck_thickness / 2.0
    kingpin_z = -0.055
    kingpin_tilt = 0.22

    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(_capsule_profile(deck_length, deck_width), deck_thickness, center=True),
        "calibrated_deck_body",
    )
    grip_mesh = mesh_from_geometry(
        ExtrudeGeometry(_capsule_profile(deck_length - 0.065, deck_width - 0.032), 0.0012, center=True),
        "precision_grip_sheet",
    )
    bushing_mesh = mesh_from_geometry(_ring_mesh(0.018, 0.0065, 0.020, segments=56), "bushing_annulus")
    washer_mesh = mesh_from_geometry(_ring_mesh(0.020, 0.0036, 0.003, segments=56), "kingpin_washer")
    nut_mesh = mesh_from_geometry(_ring_mesh(0.012, 0.0060, 0.010, segments=6), "hex_adjuster_nut")
    wheel_tire_mesh = mesh_from_geometry(_ring_mesh(0.032, 0.0130, 0.034, segments=72), "urethane_wheel_shell")
    bearing_mesh = mesh_from_geometry(_ring_mesh(0.0130, 0.0049, 0.003, segments=48), "bearing_shield_ring")
    spacer_mesh = mesh_from_geometry(_ring_mesh(0.012, 0.0050, 0.004, segments=40), "axle_spacer_ring")

    deck = model.part("deck")
    deck.visual(deck_mesh, material=deck_mat, name="deck_body")
    deck.visual(
        grip_mesh,
        origin=Origin(xyz=(0.0, 0.0, deck_thickness / 2.0 + 0.0006)),
        material=grip_mat,
        name="grip_sheet",
    )
    deck.visual(
        Box((0.700, 0.005, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0, deck_thickness / 2.0 + 0.0017)),
        material=datum_mat,
        name="center_datum",
    )
    deck.visual(
        Box((0.005, 0.192, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0, deck_thickness / 2.0 + 0.0017)),
        material=datum_mat,
        name="cross_datum",
    )
    for i, x in enumerate((-truck_x, truck_x)):
        deck.visual(
            Box((0.004, 0.185, 0.0010)),
            origin=Origin(xyz=(x, 0.0, deck_thickness / 2.0 + 0.0018)),
            material=datum_mat,
            name=f"truck_index_{i}",
        )
        for j, y in enumerate((-0.045, 0.045)):
            deck.visual(
                Box((0.008, 0.022, 0.0012)),
                origin=Origin(xyz=(x - 0.024, y, deck_thickness / 2.0 + 0.0018)),
                material=datum_mat,
                name=f"mount_tick_{i}_{j}",
            )

    def add_truck(prefix: str, x: float, tilt: float) -> None:
        base = model.part(f"{prefix}_base")
        base.visual(
            Box((0.095, 0.078, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=dark_metal,
            name="riser_pad",
        )
        base.visual(
            Box((0.090, 0.072, 0.009)),
            origin=Origin(xyz=(0.0, 0.0, -0.0105)),
            material=metal_mat,
            name="baseplate",
        )
        base.visual(
            Box((0.062, 0.004, 0.0012)),
            origin=Origin(xyz=(0.0, -0.038, -0.0156)),
            material=datum_mat,
            name="toe_scale",
        )
        for k, sx in enumerate((-0.030, 0.0, 0.030)):
            base.visual(
                Box((0.002, 0.010, 0.0013)),
                origin=Origin(xyz=(sx, -0.038, -0.01565)),
                material=datum_mat,
                name=f"scale_tick_{k}",
            )
        base.visual(
            Cylinder(radius=0.0038, length=0.086),
            origin=_tilted_origin((0.0, 0.0, kingpin_z), (0.0, 0.0, 0.0), tilt),
            material=dark_metal,
            name="kingpin_shaft",
        )
        base.visual(
            washer_mesh,
            origin=_tilted_origin((0.0, 0.0, kingpin_z), (0.0, 0.0, 0.0235), tilt),
            material=metal_mat,
            name="upper_washer",
        )
        base.visual(
            washer_mesh,
            origin=_tilted_origin((0.0, 0.0, kingpin_z), (0.0, 0.0, -0.0235), tilt),
            material=metal_mat,
            name="lower_washer",
        )

        model.articulation(
            f"deck_to_{prefix}_base",
            ArticulationType.FIXED,
            parent=deck,
            child=base,
            origin=Origin(xyz=(x, 0.0, base_top_z)),
        )

        adjuster = model.part(f"{prefix}_adjuster")
        adjuster.visual(nut_mesh, material=dark_metal, name="hex_nut")
        model.articulation(
            f"{prefix}_adjust",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=adjuster,
            origin=_tilted_origin((0.0, 0.0, kingpin_z), (0.0, 0.0, 0.030), tilt),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0),
            motion_properties=MotionProperties(damping=0.03, friction=0.18),
        )

        hanger = model.part(f"{prefix}_hanger")
        hanger.visual(
            bushing_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=bushing_mat,
            name="upper_bushing",
        )
        hanger.visual(
            bushing_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=bushing_mat,
            name="lower_bushing",
        )
        for side, y in enumerate((-0.022, 0.022)):
            hanger.visual(
                Box((0.060, 0.008, 0.050)),
                origin=Origin(xyz=(0.0, y, 0.0)),
                material=metal_mat,
                name=f"bushing_cheek_{side}",
            )
            hanger.visual(
                Box((0.032, 0.012, 0.042)),
                origin=Origin(xyz=(0.0, math.copysign(0.031, y), -0.031)),
                material=metal_mat,
                name=f"drop_link_{side}",
            )
        hanger.visual(
            Cylinder(radius=0.0052, length=0.410),
            origin=Origin(xyz=(0.0, 0.0, -0.050), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="axle",
        )
        for block_name, y in (("axle_block_0", -0.036), ("axle_block_1", 0.036)):
            hanger.visual(
                Box((0.045, 0.020, 0.014)),
                origin=Origin(xyz=(0.0, y, -0.045)),
                material=metal_mat,
                name=block_name,
            )
        for spacer_name, y in (("axle_spacer_0", -0.1665), ("axle_spacer_1", 0.1665)):
            hanger.visual(
                spacer_mesh,
                origin=Origin(xyz=(0.0, y, -0.050), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=metal_mat,
                name=spacer_name,
            )

        model.articulation(
            f"{prefix}_steer",
            ArticulationType.REVOLUTE,
            parent=base,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, kingpin_z), rpy=(0.0, tilt, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=30.0, velocity=1.2),
            motion_properties=MotionProperties(damping=0.7, friction=0.08),
        )

        for idx, y in enumerate((-0.186, 0.186)):
            wheel = model.part(f"{prefix}_wheel_{idx}")
            wheel.visual(
                wheel_tire_mesh,
                origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
                material=urethane_mat,
                name="urethane_tire",
            )
            wheel.visual(
                bearing_mesh,
                origin=Origin(xyz=(-0.0148, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal_mat,
                name="inner_bearing",
            )
            wheel.visual(
                bearing_mesh,
                origin=Origin(xyz=(0.0148, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal_mat,
                name="outer_bearing",
            )
            wheel.visual(
                Box((0.0012, 0.006, 0.026)),
                origin=Origin(xyz=(0.0174, 0.0, 0.0)),
                material=datum_mat,
                name="spin_index",
            )
            model.articulation(
                f"{prefix}_wheel_spin_{idx}",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, y, -0.050), rpy=(0.0, 0.0, math.pi / 2.0)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=1.0, velocity=40.0),
                motion_properties=MotionProperties(damping=0.02, friction=0.01),
            )

    add_truck("front", truck_x, kingpin_tilt)
    add_truck("rear", -truck_x, -kingpin_tilt)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_base = object_model.get_part("front_base")
    rear_base = object_model.get_part("rear_base")
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    front_steer = object_model.get_articulation("front_steer")
    front_spin = object_model.get_articulation("front_wheel_spin_1")

    for base, hanger, label in (
        (front_base, front_hanger, "front"),
        (rear_base, rear_hanger, "rear"),
    ):
        ctx.allow_overlap(
            base,
            hanger,
            elem_a="upper_washer",
            elem_b="upper_bushing",
            reason="The kingpin washer intentionally preloads the upper urethane bushing in compression.",
        )
        ctx.expect_gap(
            base,
            hanger,
            axis="z",
            max_gap=0.0,
            max_penetration=0.010,
            positive_elem="upper_washer",
            negative_elem="upper_bushing",
            name=f"{label} upper bushing is compressed by the washer",
        )
        ctx.allow_overlap(
            base,
            hanger,
            elem_a="lower_washer",
            elem_b="lower_bushing",
            reason="The lower kingpin washer intentionally preloads the lower urethane bushing in compression.",
        )
        ctx.expect_gap(
            hanger,
            base,
            axis="z",
            max_gap=0.0,
            max_penetration=0.010,
            positive_elem="lower_bushing",
            negative_elem="lower_washer",
            name=f"{label} lower bushing is compressed by the washer",
        )

    ctx.expect_gap(
        deck,
        front_base,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="deck_body",
        negative_elem="riser_pad",
        name="front riser is seated on the deck datum underside",
    )
    ctx.expect_gap(
        deck,
        rear_base,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem="deck_body",
        negative_elem="riser_pad",
        name="rear riser is seated on the deck datum underside",
    )
    ctx.expect_within(
        front_base,
        deck,
        axes="xy",
        inner_elem="riser_pad",
        outer_elem="deck_body",
        margin=0.0,
        name="front truck pad fits inside deck outline",
    )
    ctx.expect_gap(
        front_wheel_1,
        front_hanger,
        axis="y",
        min_gap=0.0002,
        max_gap=0.0012,
        positive_elem="urethane_tire",
        negative_elem="axle_spacer_1",
        name="positive-side wheel has controlled spacer gap",
    )

    rest_wheel_pos = ctx.part_world_position(front_wheel_1)
    with ctx.pose({front_steer: 0.25}):
        steered_wheel_pos = ctx.part_world_position(front_wheel_1)
    ctx.check(
        "front truck steer yaws the axle about the kingpin",
        rest_wheel_pos is not None
        and steered_wheel_pos is not None
        and abs(steered_wheel_pos[0] - rest_wheel_pos[0]) > 0.030,
        details=f"rest={rest_wheel_pos}, steered={steered_wheel_pos}",
    )

    rest_spin_pos = ctx.part_world_position(front_wheel_1)
    with ctx.pose({front_spin: 1.75}):
        spun_pos = ctx.part_world_position(front_wheel_1)
    ctx.check(
        "wheel spin keeps the bearing center fixed on the axle",
        rest_spin_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_spin_pos, spun_pos)) < 1.0e-6,
        details=f"rest={rest_spin_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
