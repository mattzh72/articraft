from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 1.20
BODY_DEPTH = 0.80
BODY_HEIGHT = 1.12
PANEL_THICKNESS = 0.035

DRUM_CENTER_Z = 0.55
DRUM_AXIS_Y = -0.02
DRUM_RADIUS = 0.315
DRUM_LENGTH = 0.56

DOOR_HINGE_X = 0.37
DOOR_HINGE_Y = -0.475
DOOR_CENTER_OFFSET = 0.37
DOOR_OUTER_DIAMETER = 0.72


def _drum_shell_geometry(
    radius: float = DRUM_RADIUS,
    length: float = DRUM_LENGTH,
    wall: float = 0.012,
    segments: int = 64,
) -> MeshGeometry:
    """Open-front cylindrical dryer drum shell, authored around local +Y axle."""

    geom = MeshGeometry()
    inner = radius - wall
    y_front = -length / 2.0
    y_back = length / 2.0

    def ring(r: float, y: float) -> list[int]:
        verts: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            verts.append(geom.add_vertex(r * math.cos(a), y, r * math.sin(a)))
        return verts

    outer_front = ring(radius, y_front)
    outer_back = ring(radius, y_back)
    inner_front = ring(inner, y_front + 0.006)
    inner_back = ring(inner, y_back - 0.006)
    rear_center = geom.add_vertex(0.0, y_back - 0.006, 0.0)

    for i in range(segments):
        j = (i + 1) % segments

        # Outer cylindrical skin.
        geom.add_face(outer_front[i], outer_front[j], outer_back[j])
        geom.add_face(outer_front[i], outer_back[j], outer_back[i])

        # Inner cylindrical skin.
        geom.add_face(inner_front[j], inner_front[i], inner_back[i])
        geom.add_face(inner_front[j], inner_back[i], inner_back[j])

        # Rolled front lip joining the inner and outer walls.
        geom.add_face(outer_front[i], inner_front[j], inner_front[i])
        geom.add_face(outer_front[i], outer_front[j], inner_front[j])

        # Solid rear drum plate.
        geom.add_face(outer_back[i], inner_back[i], inner_back[j])
        geom.add_face(outer_back[i], inner_back[j], outer_back[j])
        geom.add_face(rear_center, inner_back[j], inner_back[i])

    return geom


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coin_operated_commercial_dryer")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    darker_steel = model.material("darker_stainless", rgba=(0.52, 0.54, 0.53, 1.0))
    drum_metal = model.material("perforated_drum_metal", rgba=(0.82, 0.83, 0.80, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("smoky_glass", rgba=(0.38, 0.58, 0.70, 0.38))
    label_blue = model.material("blue_instruction_label", rgba=(0.06, 0.15, 0.45, 1.0))
    brass = model.material("coin_brass", rgba=(0.86, 0.66, 0.24, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.02, 0.02, 0.018, 1.0))

    body = model.part("body")
    t = PANEL_THICKNESS
    front_y = -BODY_DEPTH / 2.0 - t / 2.0

    # Wide stainless cabinet shell, open at the center of the front face.
    _add_box(body, (t, BODY_DEPTH, BODY_HEIGHT), (-BODY_WIDTH / 2.0 + t / 2.0, 0.0, BODY_HEIGHT / 2.0), stainless, "side_panel_0")
    _add_box(body, (t, BODY_DEPTH, BODY_HEIGHT), (BODY_WIDTH / 2.0 - t / 2.0, 0.0, BODY_HEIGHT / 2.0), stainless, "side_panel_1")
    _add_box(body, (BODY_WIDTH, BODY_DEPTH, t), (0.0, 0.0, t / 2.0), stainless, "bottom_panel")
    _add_box(body, (BODY_WIDTH, BODY_DEPTH, t), (0.0, 0.0, BODY_HEIGHT - t / 2.0), stainless, "top_panel")
    _add_box(body, (BODY_WIDTH, t, BODY_HEIGHT), (0.0, BODY_DEPTH / 2.0 - t / 2.0, BODY_HEIGHT / 2.0), stainless, "rear_panel")

    opening_outer = 0.74
    side_width = (BODY_WIDTH - opening_outer) / 2.0
    _add_box(body, (side_width, t, BODY_HEIGHT - 2.0 * t), (-(opening_outer / 2.0 + side_width / 2.0), front_y, BODY_HEIGHT / 2.0), stainless, "front_stile_0")
    _add_box(body, (side_width, t, BODY_HEIGHT - 2.0 * t), ((opening_outer / 2.0 + side_width / 2.0), front_y, BODY_HEIGHT / 2.0), stainless, "front_stile_1")
    top_height = BODY_HEIGHT - (DRUM_CENTER_Z + opening_outer / 2.0)
    bottom_height = DRUM_CENTER_Z - opening_outer / 2.0
    _add_box(body, (BODY_WIDTH, t, top_height), (0.0, front_y, DRUM_CENTER_Z + opening_outer / 2.0 + top_height / 2.0), stainless, "front_top_band")
    _add_box(body, (BODY_WIDTH, t, bottom_height), (0.0, front_y, bottom_height / 2.0), stainless, "front_bottom_band")

    front_gasket = BezelGeometry(
        (0.61, 0.61),
        (opening_outer, opening_outer),
        0.030,
        opening_shape="circle",
        outer_shape="circle",
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    body.visual(
        mesh_from_geometry(front_gasket, "front_gasket"),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 - 0.012, DRUM_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="front_gasket",
    )

    # Louvred toe area and leveling feet make the cabinet read as a commercial unit.
    for i, x in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
        _add_box(body, (0.080, 0.008, 0.018), (x, -BODY_DEPTH / 2.0 - 0.038, 0.105), dark_slot, f"toe_vent_{i}")
    for i, x in enumerate((-0.47, 0.47)):
        for j, y in enumerate((-0.28, 0.28)):
            body.visual(
                Cylinder(radius=0.035, length=0.035),
                origin=Origin(xyz=(x, y, -0.0175)),
                material=black,
                name=f"leveling_foot_{i}_{j}",
            )

    # Alternating body-side door hinge knuckles and hinge leaves.
    for idx, zc in enumerate((DRUM_CENTER_Z - 0.235, DRUM_CENTER_Z + 0.235)):
        _add_box(body, (0.100, 0.056, 0.180), (DOOR_HINGE_X + 0.068, -0.463, zc), darker_steel, f"door_hinge_leaf_{idx}")
        for seg, dz in enumerate((-0.075, 0.075)):
            body.visual(
                Cylinder(radius=0.018, length=0.055),
                origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, zc + dz)),
                material=darker_steel,
                name=f"door_hinge_knuckle_{idx}_{seg}",
            )

    # Rear bearing/axle support physically ties the rotating drum to the cabinet.
    body.visual(
        Cylinder(radius=0.050, length=0.088),
        origin=Origin(xyz=(0.0, 0.3385, DRUM_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="rear_drum_bearing",
    )

    # A hinged coin-box face in the upper front corner, with coin slot and lock details.
    coin_panel = model.part("coin_panel")
    _add_box(coin_panel, (0.38, 0.026, 0.165), (-0.19, 0.0, 0.0), darker_steel, "coin_panel_face")
    _add_box(coin_panel, (0.13, 0.010, 0.035), (-0.245, -0.018, 0.025), black, "coin_slot_plate")
    _add_box(coin_panel, (0.090, 0.012, 0.008), (-0.245, -0.025, 0.025), brass, "coin_slot")
    _add_box(coin_panel, (0.125, 0.008, 0.058), (-0.095, -0.017, 0.022), label_blue, "instruction_label")
    coin_panel.visual(
        Cylinder(radius=0.013, length=0.009),
        origin=Origin(xyz=(-0.080, -0.0175, -0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="coin_lock",
    )
    coin_panel.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=darker_steel,
        name="coin_hinge_barrel",
    )

    model.articulation(
        "body_to_coin_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=coin_panel,
        origin=Origin(xyz=(0.50, -BODY_DEPTH / 2.0 - 0.062, 1.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=1.2),
    )

    _add_box(body, (0.050, 0.030, 0.180), (0.537, -0.450, 1.015), darker_steel, "coin_hinge_leaf")

    # Large rotating drum on the front-to-back axle.
    drum = model.part("drum")
    drum.visual(
        mesh_from_geometry(_drum_shell_geometry(), "drum_shell"),
        material=drum_metal,
        name="drum_shell",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial_center = DRUM_RADIUS - 0.040
        _add_box(
            drum,
            (0.105, 0.420, 0.060),
            (math.sin(angle) * radial_center, -0.005, math.cos(angle) * radial_center),
            drum_metal,
            f"drum_baffle_{i}",
            rpy=(0.0, angle, 0.0),
        )
    drum.visual(
        Cylinder(radius=0.052, length=0.045),
        origin=Origin(xyz=(0.0, DRUM_LENGTH / 2.0 + 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="rear_axle_hub",
    )
    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, DRUM_AXIS_Y, DRUM_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=12.0),
    )

    # Wide porthole door: metal annular frame, captured glass window, pull handle,
    # and complementary barrel-hinge knuckles all attached to the same revolute link.
    door = model.part("porthole_door")
    door_frame = BezelGeometry(
        (0.485, 0.485),
        (DOOR_OUTER_DIAMETER, DOOR_OUTER_DIAMETER),
        0.060,
        opening_shape="circle",
        outer_shape="circle",
        face=BezelFace(style="radiused_step", front_lip=0.006, fillet=0.003),
    )
    door.visual(
        mesh_from_geometry(door_frame, "door_frame"),
        origin=Origin(xyz=(-DOOR_CENTER_OFFSET, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="door_frame",
    )
    gasket_ring = BezelGeometry((0.440, 0.440), (0.530, 0.530), 0.018, opening_shape="circle", outer_shape="circle")
    door.visual(
        mesh_from_geometry(gasket_ring, "door_inner_gasket"),
        origin=Origin(xyz=(-DOOR_CENTER_OFFSET, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="door_inner_gasket",
    )
    door.visual(
        Cylinder(radius=0.252, length=0.014),
        origin=Origin(xyz=(-DOOR_CENTER_OFFSET, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_window",
    )
    _add_box(door, (0.045, 0.040, 0.250), (-0.700, -0.025, 0.0), black, "pull_handle")

    for idx, zc in enumerate((-0.235, 0.235)):
        _add_box(door, (0.135, 0.025, 0.170), (-0.070, 0.030, zc), darker_steel, f"hinge_leaf_{idx}")
        _add_box(door, (0.020, 0.006, 0.060), (-0.006, 0.020, zc), darker_steel, f"hinge_web_{idx}")
        door.visual(
            Cylinder(radius=0.0175, length=0.065),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=darker_steel,
            name=f"hinge_knuckle_{idx}",
        )

    model.articulation(
        "body_to_porthole_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DRUM_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=14.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("porthole_door")
    coin_panel = object_model.get_part("coin_panel")
    door_hinge = object_model.get_articulation("body_to_porthole_door")
    coin_hinge = object_model.get_articulation("body_to_coin_panel")
    drum_axle = object_model.get_articulation("body_to_drum")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_gasket",
        negative_elem="door_frame",
        min_gap=0.001,
        max_gap=0.030,
        name="closed porthole door sits just proud of the front gasket",
    )
    ctx.expect_overlap(
        door,
        drum,
        axes="xz",
        elem_a="glass_window",
        elem_b="drum_shell",
        min_overlap=0.42,
        name="glass porthole frames the rotating drum",
    )
    ctx.expect_gap(
        body,
        coin_panel,
        axis="y",
        positive_elem="front_top_band",
        negative_elem="coin_panel_face",
        min_gap=0.003,
        max_gap=0.040,
        name="coin-box panel is mounted proud of the upper front face",
    )

    def aabb_center_y(part, elem: str) -> float:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return 0.0
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    def aabb_center_x(part, elem: str) -> float:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return 0.0
        lo, hi = aabb
        return (lo[0] + hi[0]) / 2.0

    closed_handle_y = aabb_center_y(door, "pull_handle")
    with ctx.pose({door_hinge: 1.20}):
        open_handle_y = aabb_center_y(door, "pull_handle")
    ctx.check(
        "porthole door swings outward from the right barrel hinges",
        open_handle_y < closed_handle_y - 0.12,
        details=f"closed_y={closed_handle_y:.3f}, open_y={open_handle_y:.3f}",
    )

    closed_coin_y = aabb_center_y(coin_panel, "coin_panel_face")
    with ctx.pose({coin_hinge: 0.90}):
        open_coin_y = aabb_center_y(coin_panel, "coin_panel_face")
    ctx.check(
        "coin-box panel hinges outward from its side edge",
        open_coin_y < closed_coin_y - 0.07,
        details=f"closed_y={closed_coin_y:.3f}, open_y={open_coin_y:.3f}",
    )

    baffle_x = aabb_center_x(drum, "drum_baffle_0")
    with ctx.pose({drum_axle: 1.0}):
        rotated_baffle_x = aabb_center_x(drum, "drum_baffle_0")
    ctx.check(
        "drum rotates about the front-to-back axle",
        abs(rotated_baffle_x - baffle_x) > 0.10,
        details=f"rest_x={baffle_x:.3f}, rotated_x={rotated_baffle_x:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
