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
    TorusGeometry,
    mesh_from_geometry,
)


def _annular_sector_geometry(
    inner_radius: float,
    outer_radius: float,
    height: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 36,
    z0: float = 0.0,
) -> MeshGeometry:
    """Thin cylindrical-wall sector with real thickness and capped radial ends."""

    geom = MeshGeometry()
    steps = max(2, segments)
    rings: list[tuple[int, int, int, int]] = []
    for index in range(steps + 1):
        t = index / steps
        angle = start_angle + (end_angle - start_angle) * t
        ca = math.cos(angle)
        sa = math.sin(angle)
        ib = geom.add_vertex(inner_radius * ca, inner_radius * sa, z0)
        it = geom.add_vertex(inner_radius * ca, inner_radius * sa, z0 + height)
        ob = geom.add_vertex(outer_radius * ca, outer_radius * sa, z0)
        ot = geom.add_vertex(outer_radius * ca, outer_radius * sa, z0 + height)
        rings.append((ib, it, ob, ot))

    for index in range(steps):
        ib0, it0, ob0, ot0 = rings[index]
        ib1, it1, ob1, ot1 = rings[index + 1]

        # Inner and outer curved faces.
        geom.add_face(ib0, ib1, it1)
        geom.add_face(ib0, it1, it0)
        geom.add_face(ob0, ot0, ot1)
        geom.add_face(ob0, ot1, ob1)

        # Bottom and top thickness faces.
        geom.add_face(ib0, ob1, ib1)
        geom.add_face(ib0, ob0, ob1)
        geom.add_face(it0, it1, ot1)
        geom.add_face(it0, ot1, ot0)

    # Radial end caps.
    for ib, it, ob, ot in (rings[0], rings[-1]):
        geom.add_face(ib, it, ot)
        geom.add_face(ib, ot, ob)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_three_wing_revolving_door")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.68, 0.62, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.035, 0.035, 0.032, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.58, 0.82, 0.95, 0.34))
    floor_mat = model.material("dark_stone_floor", rgba=(0.16, 0.16, 0.15, 1.0))

    radius_inner = 1.02
    radius_outer = 1.07
    base_radius = 1.16
    wall_z0 = 0.075
    wall_height = 2.22
    top_track_z = wall_z0 + wall_height + 0.025

    frame = model.part("drum_frame")
    frame.visual(
        Cylinder(radius=base_radius, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=floor_mat,
        name="base_slab",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(radius=1.045, tube=0.035), "bottom_outer_track"),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_rubber,
        name="bottom_track",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(radius=1.045, tube=0.045), "top_outer_track"),
        origin=Origin(xyz=(0.0, 0.0, top_track_z)),
        material=aluminum,
        name="top_track",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.145, tube=0.026), "bottom_center_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=aluminum,
        name="bottom_bearing",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.145, tube=0.026), "top_center_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 2.335)),
        material=aluminum,
        name="top_bearing",
    )
    for index in range(3):
        yaw = index * 2.0 * math.pi / 3.0
        frame.visual(
            Box((0.93, 0.050, 0.040)),
            origin=Origin(
                xyz=(0.595 * math.cos(yaw), 0.595 * math.sin(yaw), 2.335),
                rpy=(0.0, 0.0, yaw),
            ),
            material=aluminum,
            name=f"top_spoke_{index}",
        )

    for index, (start_deg, end_deg) in enumerate(((35.0, 145.0), (215.0, 325.0))):
        frame.visual(
            mesh_from_geometry(
                _annular_sector_geometry(
                    radius_inner,
                    radius_outer,
                    wall_height,
                    math.radians(start_deg),
                    math.radians(end_deg),
                    segments=40,
                    z0=wall_z0,
                ),
                f"curved_glass_wall_{index}",
            ),
            material=glass,
            name=f"wall_{index}",
        )

    post_height = wall_height + 0.04
    post_radius = 0.030
    for index, angle_deg in enumerate((35.0, 145.0, 215.0, 325.0)):
        angle = math.radians(angle_deg)
        post_radius_from_center = (radius_inner + radius_outer) * 0.5
        frame.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(
                xyz=(
                    post_radius_from_center * math.cos(angle),
                    post_radius_from_center * math.sin(angle),
                    wall_z0 + post_height / 2.0,
                )
            ),
            material=aluminum,
            name=f"post_{index}",
        )

    rotor = model.part("rotor")
    column_radius = 0.080
    column_height = 2.28
    rotor.visual(
        Cylinder(radius=column_radius, length=column_height),
        origin=Origin(xyz=(0.0, 0.0, column_height / 2.0)),
        material=aluminum,
        name="center_column",
    )
    rotor.visual(
        Cylinder(radius=0.135, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=aluminum,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.135, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 2.175)),
        material=aluminum,
        name="upper_hub",
    )

    wing_inner = 0.060
    wing_outer = 0.970
    wing_length = wing_outer - wing_inner
    wing_mid = (wing_outer + wing_inner) / 2.0
    wing_height = 2.08
    wing_z = 0.15 + wing_height / 2.0
    panel_thickness = 0.028

    for index in range(3):
        yaw = index * 2.0 * math.pi / 3.0
        c = math.cos(yaw)
        s = math.sin(yaw)

        def radial_origin(local_x: float, local_y: float, local_z: float) -> Origin:
            return Origin(
                xyz=(local_x * c - local_y * s, local_x * s + local_y * c, local_z),
                rpy=(0.0, 0.0, yaw),
            )

        rotor.visual(
            Box((wing_length, panel_thickness, wing_height)),
            origin=radial_origin(wing_mid, 0.0, wing_z),
            material=glass,
            name=f"wing_{index}_panel",
        )
        rotor.visual(
            Box((wing_length, 0.052, 0.050)),
            origin=radial_origin(wing_mid, 0.0, 0.175),
            material=aluminum,
            name=f"wing_{index}_bottom_rail",
        )
        rotor.visual(
            Box((wing_length, 0.052, 0.050)),
            origin=radial_origin(wing_mid, 0.0, 2.205),
            material=aluminum,
            name=f"wing_{index}_top_rail",
        )
        rotor.visual(
            Box((0.052, 0.058, wing_height)),
            origin=radial_origin(wing_outer - 0.026, 0.0, wing_z),
            material=aluminum,
            name=f"wing_{index}_outer_stile",
        )
        rotor.visual(
            Box((0.050, 0.058, wing_height)),
            origin=radial_origin(wing_inner + 0.025, 0.0, wing_z),
            material=aluminum,
            name=f"wing_{index}_inner_stile",
        )
        for side, local_y in enumerate((-0.020, 0.020)):
            rotor.visual(
                Box((0.62, 0.026, 0.045)),
                origin=radial_origin(0.51, local_y, 1.10),
                material=aluminum,
                name=f"wing_{index}_push_bar_{side}",
            )

    model.articulation(
        "spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("drum_frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("spin")

    ctx.check(
        "wing assembly has continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="z",
        elem_a="center_column",
        elem_b="wall_0",
        min_overlap=2.0,
        name="exposed center column spans the drum wall height",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="wing_0_panel",
        negative_elem="base_slab",
        min_gap=0.05,
        name="glass wings clear the floor slab",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="wing_0_panel",
        outer_elem="base_slab",
        margin=0.0,
        name="wing reaches outward while staying inside the circular drum footprint",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_panel")
    rest_center = None
    if rest_aabb is not None:
        rest_center = tuple((rest_aabb[0][i] + rest_aabb[1][i]) * 0.5 for i in range(3))
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_panel")
        turned_center = None
        if turned_aabb is not None:
            turned_center = tuple((turned_aabb[0][i] + turned_aabb[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "continuous joint rotates a wing around the center column",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.45
        and abs(rest_center[1]) < 0.05
        and abs(turned_center[0]) < 0.05
        and turned_center[1] > 0.45,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
