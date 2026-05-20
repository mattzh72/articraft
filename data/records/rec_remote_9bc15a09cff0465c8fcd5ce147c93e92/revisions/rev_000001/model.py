from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)
import cadquery as cq


def _ribbed_annular_ring(
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    z_center: float,
    segments: int = 144,
    rib_count: int = 48,
    rib_depth: float = 0.0018,
) -> MeshGeometry:
    """Connected annular control ring with a subtly scalloped grip perimeter."""
    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    z0 = z_center - height / 2.0
    z1 = z_center + height / 2.0

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        # Raised rounded ridges around the outside; the inner bearing wall stays round.
        ridge = rib_depth * (0.5 + 0.5 * math.cos(rib_count * theta)) ** 2.0
        ro = outer_radius + ridge
        x_outer = ro * math.cos(theta)
        y_outer = ro * math.sin(theta)
        x_inner = inner_radius * math.cos(theta)
        y_inner = inner_radius * math.sin(theta)
        outer_bottom.append(geom.add_vertex(x_outer, y_outer, z0))
        outer_top.append(geom.add_vertex(x_outer, y_outer, z1))
        inner_bottom.append(geom.add_vertex(x_inner, y_inner, z0))
        inner_top.append(geom.add_vertex(x_inner, y_inner, z1))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer gripped wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner bearing wall.
        geom.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[j], inner_top[i], inner_top[j])
        # Top annular land.
        geom.add_face(inner_top[i], outer_top[i], outer_top[j])
        geom.add_face(inner_top[i], outer_top[j], inner_top[j])
        # Bottom annular land.
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_automation_hub_controller")

    soft_white = model.material("soft_white", rgba=(0.86, 0.88, 0.87, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.42, 0.44, 0.44, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.025, 0.027, 0.030, 1.0))
    black_glass = model.material("black_glass", rgba=(0.01, 0.014, 0.018, 0.92))
    blue_led = model.material("blue_led", rgba=(0.0, 0.38, 1.0, 1.0))

    body = model.part("body")
    body_profile = [
        (0.0, 0.000),
        (0.060, 0.000),
        (0.070, 0.002),
        (0.073, 0.006),
        (0.073, 0.025),
        (0.068, 0.031),
        (0.047, 0.034),
        (0.0, 0.034),
    ]
    body.visual(
        mesh_from_geometry(LatheGeometry(body_profile, segments=144), "body_shell"),
        material=soft_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.0767, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=warm_gray,
        name="bearing_race",
    )
    body.visual(
        Cylinder(radius=0.044, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, 0.0350)),
        material=black_glass,
        name="top_lens",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0012),
        origin=Origin(xyz=(0.0, 0.021, 0.0369)),
        material=blue_led,
        name="status_light",
    )
    # Underside slide rails that visibly retain the battery door.
    for x, name in [(-0.031, "battery_rail_0"), (0.031, "battery_rail_1")]:
        body.visual(
            Box((0.006, 0.083, 0.006)),
            origin=Origin(xyz=(x, 0.002, -0.0025)),
            material=warm_gray,
            name=name,
        )
    body.visual(
        Box((0.064, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, -0.043, -0.0025)),
        material=warm_gray,
        name="battery_stop",
    )

    selection_ring = model.part("selection_ring")
    ring_mesh = _ribbed_annular_ring(
        inner_radius=0.076,
        outer_radius=0.090,
        height=0.028,
        z_center=0.017,
        segments=144,
        rib_count=48,
        rib_depth=0.0018,
    )
    selection_ring.visual(
        mesh_from_geometry(ring_mesh, "selection_ring"),
        material=dark_rubber,
        name="selection_ring",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.052, 0.070, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=warm_gray,
        name="door_panel",
    )
    for y, name in [(-0.018, "grip_rib_0"), (-0.010, "grip_rib_1"), (-0.002, "grip_rib_2")]:
        battery_door.visual(
            Box((0.034, 0.0032, 0.0012)),
            origin=Origin(xyz=(0.0, y, -0.0046)),
            material=dark_rubber,
            name=name,
        )

    model.articulation(
        "body_to_selection_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selection_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.045),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    ring = object_model.get_part("selection_ring")
    door = object_model.get_part("battery_door")
    ring_joint = object_model.get_articulation("body_to_selection_ring")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.expect_origin_distance(
        body,
        ring,
        axes="xy",
        max_dist=0.001,
        name="outer ring is concentric with hub body",
    )
    ctx.allow_overlap(
        body,
        ring,
        elem_a="bearing_race",
        elem_b="selection_ring",
        reason=(
            "A narrow hidden bearing race is intentionally seated into the "
            "rotating selector ring so the ring is physically captured."
        ),
    )
    ctx.expect_overlap(
        body,
        ring,
        axes="z",
        elem_a="bearing_race",
        elem_b="selection_ring",
        min_overlap=0.005,
        name="bearing race sits inside the height of the selector ring",
    )
    ctx.expect_gap(
        body,
        door,
        axis="z",
        positive_elem="body_shell",
        negative_elem="door_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="battery door sits flush against underside",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="x",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.045,
        name="battery door is laterally retained by the body footprint",
    )

    closed_position = ctx.part_world_position(door)
    with ctx.pose({ring_joint: 4.7, door_joint: 0.045}):
        ctx.expect_origin_distance(
            body,
            ring,
            axes="xy",
            max_dist=0.001,
            name="continuous ring keeps its vertical spin axis",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="y",
            elem_a="door_panel",
            elem_b="body_shell",
            min_overlap=0.020,
            name="slid battery door remains partly inserted",
        )
        open_position = ctx.part_world_position(door)

    ctx.check(
        "battery door slides outward along its rail",
        closed_position is not None
        and open_position is not None
        and open_position[1] > closed_position[1] + 0.040,
        details=f"closed={closed_position}, open={open_position}",
    )

    return ctx.report()


object_model = build_object_model()
