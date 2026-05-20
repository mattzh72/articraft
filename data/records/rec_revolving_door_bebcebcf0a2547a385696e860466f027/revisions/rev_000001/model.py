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


DRUM_HEIGHT = 2.25
DRUM_INNER_RADIUS = 0.90
DRUM_OUTER_RADIUS = 0.96
WING_OUTER_RADIUS = 0.875


def _annular_sector(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Closed thin cylindrical sector mesh, in meters."""
    geom = MeshGeometry()
    steps = max(3, segments)
    angles = [
        start_angle + (end_angle - start_angle) * i / steps for i in range(steps + 1)
    ]
    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for angle in angles:
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_min))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_max))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_min))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_max))

    for i in range(steps):
        ob0, ob1 = outer_bottom[i], outer_bottom[i + 1]
        ot0, ot1 = outer_top[i], outer_top[i + 1]
        ib0, ib1 = inner_bottom[i], inner_bottom[i + 1]
        it0, it1 = inner_top[i], inner_top[i + 1]

        # Outer and inner curved faces.
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        geom.add_face(ib1, ib0, it0)
        geom.add_face(ib1, it0, it1)

        # Top and bottom annular faces.
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob1, ob0, ib0)
        geom.add_face(ob1, ib0, ib1)

    # Radial end caps close partial wall sectors.
    for idx in (0, steps):
        ob, ot = outer_bottom[idx], outer_top[idx]
        ib, it = inner_bottom[idx], inner_top[idx]
        geom.add_face(ib, ob, ot)
        geom.add_face(ib, ot, it)

    return geom


def _annular_ring(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    return _annular_sector(
        inner_radius,
        outer_radius,
        z_min,
        z_max,
        0.0,
        2.0 * math.pi,
        segments=segments,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_mantrap_revolving_door")

    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.58, 0.82, 0.95, 0.34))
    smoked_glass = model.material("smoked_glass", rgba=(0.42, 0.62, 0.78, 0.42))
    floor_mat = model.material("brushed_floor", rgba=(0.36, 0.38, 0.38, 1.0))

    drum = model.part("drum")
    drum.visual(
        mesh_from_geometry(_annular_ring(0.88, DRUM_OUTER_RADIUS, 0.0, 0.09), "lower_hoop"),
        material=brushed_steel,
        name="lower_hoop",
    )
    drum.visual(
        mesh_from_geometry(
            _annular_ring(0.88, DRUM_OUTER_RADIUS, DRUM_HEIGHT - 0.09, DRUM_HEIGHT),
            "upper_hoop",
        ),
        material=brushed_steel,
        name="upper_hoop",
    )
    for idx, (start_deg, end_deg) in enumerate(((45.0, 135.0), (225.0, 315.0))):
        drum.visual(
            mesh_from_geometry(
                _annular_sector(
                    DRUM_INNER_RADIUS,
                    0.925,
                    0.08,
                    DRUM_HEIGHT - 0.09,
                    math.radians(start_deg),
                    math.radians(end_deg),
                    segments=32,
                ),
                f"curved_glass_{idx}",
            ),
            material=clear_glass,
            name=f"curved_glass_{idx}",
        )

    drum.visual(
        Cylinder(radius=0.89, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=floor_mat,
        name="floor_disc",
    )
    drum.visual(
        Cylinder(radius=0.12, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=brushed_steel,
        name="lower_bearing",
    )
    drum.visual(
        Cylinder(radius=0.12, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, DRUM_HEIGHT - 0.055)),
        material=brushed_steel,
        name="upper_bearing",
    )

    # Four fixed jambs mark the edges of the two opposed access openings.
    for idx, angle in enumerate(
        (math.radians(45.0), math.radians(135.0), math.radians(225.0), math.radians(315.0))
    ):
        drum.visual(
            Cylinder(radius=0.026, length=DRUM_HEIGHT - 0.05),
            origin=Origin(
                xyz=(0.918 * math.cos(angle), 0.918 * math.sin(angle), DRUM_HEIGHT / 2.0),
            ),
            material=brushed_steel,
            name=f"jamb_{idx}",
        )

    # Overhead spokes tie the central top bearing visibly back into the drum hoop.
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.77, 0.045, 0.045)),
            origin=Origin(
                xyz=(0.505 * math.cos(angle), 0.505 * math.sin(angle), DRUM_HEIGHT - 0.055),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_steel,
            name=f"top_spoke_{idx}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=2.09),
        origin=Origin(xyz=(0.0, 0.0, 1.045)),
        material=brushed_steel,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.105, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=brushed_steel,
        name="lower_rotor_collar",
    )
    rotor.visual(
        Cylinder(radius=0.105, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.98)),
        material=brushed_steel,
        name="upper_rotor_collar",
    )

    wing_inner_radius = 0.045
    wing_length = WING_OUTER_RADIUS - wing_inner_radius
    wing_center = wing_inner_radius + wing_length / 2.0
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        yaw = angle
        rotor.visual(
            Box((wing_length, 0.018, 1.74)),
            origin=Origin(xyz=(wing_center * c, wing_center * s, 1.05), rpy=(0.0, 0.0, yaw)),
            material=smoked_glass,
            name=f"wing_glass_{idx}",
        )
        rotor.visual(
            Box((wing_length, 0.045, 0.05)),
            origin=Origin(xyz=(wing_center * c, wing_center * s, 0.17), rpy=(0.0, 0.0, yaw)),
            material=brushed_steel,
            name=f"lower_wing_rail_{idx}",
        )
        rotor.visual(
            Box((wing_length, 0.045, 0.05)),
            origin=Origin(xyz=(wing_center * c, wing_center * s, 1.93), rpy=(0.0, 0.0, yaw)),
            material=brushed_steel,
            name=f"upper_wing_rail_{idx}",
        )
        inner_stile_r = 0.067
        outer_stile_r = WING_OUTER_RADIUS - 0.022
        rotor.visual(
            Box((0.045, 0.055, 1.86)),
            origin=Origin(
                xyz=(inner_stile_r * c, inner_stile_r * s, 1.05),
                rpy=(0.0, 0.0, yaw),
            ),
            material=brushed_steel,
            name=f"inner_stile_{idx}",
        )
        rotor.visual(
            Box((0.04, 0.055, 1.86)),
            origin=Origin(
                xyz=(outer_stile_r * c, outer_stile_r * s, 1.05),
                rpy=(0.0, 0.0, yaw),
            ),
            material=brushed_steel,
            name=f"outer_stile_{idx}",
        )
        rotor.visual(
            Box((0.012, 0.062, 1.80)),
            origin=Origin(
                xyz=(WING_OUTER_RADIUS * c, WING_OUTER_RADIUS * s, 1.05),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_rubber,
            name=f"outer_seal_{idx}",
        )

    model.articulation(
        "drum_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("drum_to_rotor")

    ctx.expect_contact(
        rotor,
        drum,
        elem_a="central_post",
        elem_b="lower_bearing",
        contact_tol=0.001,
        name="rotating post sits on lower bearing",
    )
    ctx.expect_gap(
        drum,
        rotor,
        axis="z",
        positive_elem="upper_bearing",
        negative_elem="central_post",
        min_gap=0.0,
        max_gap=0.002,
        name="upper bearing captures the post top",
    )
    ctx.expect_overlap(
        rotor,
        drum,
        axes="z",
        elem_a="wing_glass_1",
        elem_b="curved_glass_0",
        min_overlap=1.50,
        name="door wings span the drum height",
    )
    ctx.expect_within(
        rotor,
        drum,
        axes="xy",
        inner_elem="outer_seal_0",
        outer_elem="lower_hoop",
        margin=0.0,
        name="wing seal remains inside circular drum hoop",
    )

    wing_names = [v.name for v in rotor.visuals if v.name.startswith("wing_glass_")]
    ctx.check(
        "three equal revolving wings",
        len(wing_names) == 3,
        details=f"wing visuals={wing_names}",
    )

    rest_box = ctx.part_element_world_aabb(rotor, elem="wing_glass_0")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_box = ctx.part_element_world_aabb(rotor, elem="wing_glass_0")
    ctx.check(
        "continuous joint rotates the wing assembly about the post",
        rest_box is not None
        and turned_box is not None
        and turned_box[1][1] > rest_box[1][1] + 0.35
        and turned_box[1][0] < rest_box[1][0] - 0.35,
        details=f"rest={rest_box}, turned={turned_box}",
    )

    return ctx.report()


object_model = build_object_model()
