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


def _annular_wall_segment(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 28,
) -> MeshGeometry:
    """Solid curved glass wall segment for the fixed cylindrical door drum."""

    geom = MeshGeometry()
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    outer_bottom: list[int] = []
    outer_top: list[int] = []

    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        ca = math.cos(angle)
        sa = math.sin(angle)
        inner_bottom.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_min))
        inner_top.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_max))
        outer_bottom.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_min))
        outer_top.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_max))

    for index in range(segments):
        ib0, ib1 = inner_bottom[index], inner_bottom[index + 1]
        it0, it1 = inner_top[index], inner_top[index + 1]
        ob0, ob1 = outer_bottom[index], outer_bottom[index + 1]
        ot0, ot1 = outer_top[index], outer_top[index + 1]

        # Inner and outer curved faces.
        geom.add_face(ib0, it1, it0)
        geom.add_face(ib0, ib1, it1)
        geom.add_face(ob0, ot0, ot1)
        geom.add_face(ob0, ot1, ob1)

        # Top and bottom thickness faces.
        geom.add_face(it0, it1, ot1)
        geom.add_face(it0, ot1, ot0)
        geom.add_face(ib0, ob1, ib1)
        geom.add_face(ib0, ob0, ob1)

    for index in (0, segments):
        ib = inner_bottom[index]
        it = inner_top[index]
        ob = outer_bottom[index]
        ot = outer_top[index]
        geom.add_face(ib, it, ot)
        geom.add_face(ib, ot, ob)

    return geom


def _rotated_origin(radial_center: float, z: float, angle: float) -> Origin:
    return Origin(
        xyz=(radial_center * math.cos(angle), radial_center * math.sin(angle), z),
        rpy=(0.0, 0.0, angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theater_double_revolving_door")

    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.60, 0.84, 0.96, 0.34))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    floor_metal = model.material("satin_floor_track", rgba=(0.46, 0.44, 0.40, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(TorusGeometry(1.64, 0.055, radial_segments=24, tubular_segments=72), "floor_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=floor_metal,
        name="floor_ring",
    )
    frame.visual(
        Cylinder(radius=0.24, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_aluminum,
        name="floor_bearing",
    )
    for spoke_index in range(4):
        angle = spoke_index * math.pi * 0.5
        frame.visual(
            Box((1.64, 0.08, 0.045)),
            origin=_rotated_origin(0.82, 0.075, angle),
            material=floor_metal,
            name=f"floor_spoke_{spoke_index}",
        )
    frame.visual(
        mesh_from_geometry(TorusGeometry(1.64, 0.055, radial_segments=24, tubular_segments=72), "ceiling_ring"),
        origin=Origin(xyz=(0.0, 0.0, 2.525)),
        material=brushed_steel,
        name="ceiling_ring",
    )

    side_arc_half_width = math.radians(52.0)
    for suffix, center_angle in (("0", 0.0), ("1", math.pi)):
        wall = _annular_wall_segment(
            1.62,
            1.665,
            0.10,
            2.48,
            center_angle - side_arc_half_width,
            center_angle + side_arc_half_width,
        )
        frame.visual(
            mesh_from_geometry(wall, f"side_wall_{suffix}"),
            material=glass,
            name=f"side_wall_{suffix}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.20, length=2.32),
        origin=Origin(xyz=(0.0, 0.0, 1.29)),
        material=brushed_steel,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.235, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_aluminum,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.235, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        material=dark_aluminum,
        name="upper_hub",
    )

    inner_edge = 0.16
    outer_edge = 1.45
    wing_length = outer_edge - inner_edge
    wing_center = (inner_edge + outer_edge) * 0.5
    for wing_index in range(4):
        angle = wing_index * math.pi * 0.5
        suffix = str(wing_index)
        rotor.visual(
            Box((wing_length, 0.032, 2.03)),
            origin=_rotated_origin(wing_center, 1.29, angle),
            material=glass,
            name=f"wing_{suffix}_glass",
        )
        rotor.visual(
            Box((wing_length + 0.05, 0.075, 0.07)),
            origin=_rotated_origin(wing_center + 0.01, 0.255, angle),
            material=dark_aluminum,
            name=f"wing_{suffix}_bottom_rail",
        )
        rotor.visual(
            Box((wing_length + 0.05, 0.075, 0.07)),
            origin=_rotated_origin(wing_center + 0.01, 2.325, angle),
            material=dark_aluminum,
            name=f"wing_{suffix}_top_rail",
        )
        rotor.visual(
            Box((0.18, 0.11, 2.18)),
            origin=_rotated_origin(0.17, 1.29, angle),
            material=brushed_steel,
            name=f"wing_{suffix}_root_stile",
        )
        rotor.visual(
            Box((0.085, 0.085, 2.17)),
            origin=_rotated_origin(outer_edge - 0.035, 1.29, angle),
            material=dark_aluminum,
            name=f"wing_{suffix}_outer_stile",
        )
        rotor.visual(
            Box((0.028, 0.115, 2.04)),
            origin=_rotated_origin(outer_edge + 0.015, 1.29, angle),
            material=rubber,
            name=f"wing_{suffix}_bumper",
        )
        rotor.visual(
            Box((1.02, 0.060, 0.045)),
            origin=_rotated_origin(0.89, 1.08, angle),
            material=brushed_steel,
            name=f"wing_{suffix}_push_bar",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "continuous vertical rotor joint",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None,
        details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
    )
    ctx.check(
        "four rectangular door wings",
        all(rotor.get_visual(f"wing_{index}_glass") is not None for index in range(4)),
        details="Expected four named glass panel wings carried by the rotor.",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.0,
        name="wide wing sweep fits inside drum frame",
    )

    def _element_center(elem_name: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(rotor, elem=elem_name)
        if bounds is None:
            return None
        lower, upper = bounds
        return (
            (lower[0] + upper[0]) * 0.5,
            (lower[1] + upper[1]) * 0.5,
            (lower[2] + upper[2]) * 0.5,
        )

    with ctx.pose({joint: 0.0}):
        wing_0_rest = _element_center("wing_0_glass")
    with ctx.pose({joint: math.pi * 0.5}):
        wing_0_quarter_turn = _element_center("wing_0_glass")

    ctx.check(
        "wing assembly turns about central post",
        wing_0_rest is not None
        and wing_0_quarter_turn is not None
        and wing_0_rest[0] > 0.65
        and abs(wing_0_rest[1]) < 0.06
        and abs(wing_0_quarter_turn[0]) < 0.06
        and wing_0_quarter_turn[1] > 0.65,
        details=f"rest={wing_0_rest}, quarter_turn={wing_0_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
