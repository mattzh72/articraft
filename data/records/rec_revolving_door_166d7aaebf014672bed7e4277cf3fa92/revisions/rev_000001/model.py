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
    mesh_from_geometry,
    tube_from_spline_points,
)


TAU = 2.0 * math.pi


def _annular_sector_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    segments: int,
) -> MeshGeometry:
    """Closed thin annular prism used for curved glass drum sectors."""
    geom = MeshGeometry()
    angles = [
        start_angle + (end_angle - start_angle) * i / segments
        for i in range(segments + 1)
    ]

    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for angle in angles:
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_min))
        outer_top.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_max))
        inner_bottom.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_min))
        inner_top.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_max))

    for i in range(segments):
        ob0, ob1 = outer_bottom[i], outer_bottom[i + 1]
        ot0, ot1 = outer_top[i], outer_top[i + 1]
        ib0, ib1 = inner_bottom[i], inner_bottom[i + 1]
        it0, it1 = inner_top[i], inner_top[i + 1]

        # Outer cylindrical face.
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        # Inner cylindrical face.
        geom.add_face(ib1, ib0, it0)
        geom.add_face(ib1, it0, it1)
        # Top and bottom annular faces.
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob1, ob0, ib0)
        geom.add_face(ob1, ib0, ib1)

    # Radial end faces, important for a real wall thickness at the entry gaps.
    geom.add_face(inner_bottom[0], outer_bottom[0], outer_top[0])
    geom.add_face(inner_bottom[0], outer_top[0], inner_top[0])
    geom.add_face(outer_bottom[-1], inner_bottom[-1], inner_top[-1])
    geom.add_face(outer_bottom[-1], inner_top[-1], outer_top[-1])
    return geom


def _rotated_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (ca * x - sa * y, sa * x + ca * y)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wing_revolving_door")

    aluminum = Material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_rubber = Material("dark_push_grip", rgba=(0.04, 0.045, 0.045, 1.0))
    blue_glass = Material("pale_blue_glass", rgba=(0.55, 0.78, 0.95, 0.34))
    smoky_glass = Material("smoky_curved_glass", rgba=(0.46, 0.70, 0.88, 0.30))

    drum = model.part("drum")
    # A low floor plate and a shallow roof canopy tie the fixed cylindrical
    # enclosure together; the two transparent wall sectors leave entry gaps.
    drum.visual(
        Cylinder(radius=1.25, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=aluminum,
        name="floor_plate",
    )
    drum.visual(
        Cylinder(radius=1.25, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 2.370)),
        material=aluminum,
        name="top_canopy",
    )
    drum.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=aluminum,
        name="bottom_bearing",
    )
    drum.visual(
        Cylinder(radius=0.080, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 2.310)),
        material=aluminum,
        name="top_bearing",
    )

    for idx, (a0, a1) in enumerate(
        ((math.radians(30.0), math.radians(150.0)), (math.radians(210.0), math.radians(330.0)))
    ):
        wall = _annular_sector_mesh(
            inner_radius=1.165,
            outer_radius=1.215,
            z_min=0.020,
            z_max=2.345,
            start_angle=a0,
            end_angle=a1,
            segments=36,
        )
        drum.visual(
            mesh_from_geometry(wall, f"curved_wall_{idx}"),
            material=smoky_glass,
            name=f"curved_wall_{idx}",
        )

    # Vertical jambs at the four ends of the curved glass wall sectors make the
    # entry gaps legible and keep the drum looking like a built enclosure.
    for idx, angle in enumerate(
        (
            math.radians(30.0),
            math.radians(150.0),
            math.radians(210.0),
            math.radians(330.0),
        )
    ):
        x, y = _rotated_xy(1.190, 0.0, angle)
        drum.visual(
            Cylinder(radius=0.026, length=2.330),
            origin=Origin(xyz=(x, y, 1.182)),
            material=aluminum,
            name=f"jamb_{idx}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.060, length=2.215),
        origin=Origin(xyz=(0.0, 0.0, 1.1725)),
        material=aluminum,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.125, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=aluminum,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.125, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 2.095)),
        material=aluminum,
        name="upper_hub",
    )

    panel_length = 0.960
    panel_center_x = 0.585
    panel_thickness = 0.028
    panel_height = 1.900
    panel_z = 1.160
    rail_radius = 0.022

    for i in range(3):
        angle = i * TAU / 3.0
        x, y = _rotated_xy(panel_center_x, 0.0, angle)
        rotor.visual(
            Box((panel_length, panel_thickness, panel_height)),
            origin=Origin(xyz=(x, y, panel_z), rpy=(0.0, 0.0, angle)),
            material=blue_glass,
            name=f"panel_{i}",
        )

        # Slim metal perimeter around each glass wing.
        for name, px, sx in (
            ("inner_stile", 0.075, 0.050),
            ("outer_stile", 1.085, 0.050),
        ):
            vx, vy = _rotated_xy(px, 0.0, angle)
            rotor.visual(
                Box((sx, 0.050, 1.965)),
                origin=Origin(xyz=(vx, vy, panel_z), rpy=(0.0, 0.0, angle)),
                material=aluminum,
                name=f"{name}_{i}",
            )
        for name, pz in (("bottom_rail", 0.185), ("top_rail", 2.135)):
            vx, vy = _rotated_xy(0.580, 0.0, angle)
            rotor.visual(
                Box((1.010, 0.050, 0.050)),
                origin=Origin(xyz=(vx, vy, pz), rpy=(0.0, 0.0, angle)),
                material=aluminum,
                name=f"{name}_{i}",
            )

        # A low bowed push rail stands proud of the glass on one face, with
        # three short brackets physically tying it into the wing panel.
        local_points = (
            (0.235, 0.066, 0.880),
            (0.640, 0.105, 0.880),
            (0.985, 0.066, 0.880),
        )
        rail_points = []
        for px, py, pz in local_points:
            rx, ry = _rotated_xy(px, py, angle)
            rail_points.append((rx, ry, pz))
        rail_mesh = tube_from_spline_points(
            rail_points,
            radius=rail_radius,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        )
        rotor.visual(
            mesh_from_geometry(rail_mesh, f"push_rail_{i}"),
            material=dark_rubber,
            name=f"push_rail_{i}",
        )
        for j, (px, py, pz) in enumerate(local_points):
            bracket_depth = py - panel_thickness / 2.0 + 1.5 * rail_radius
            bx, by = _rotated_xy(px, panel_thickness / 2.0 + bracket_depth / 2.0, angle)
            rotor.visual(
                Box((0.060, bracket_depth, 0.065)),
                origin=Origin(xyz=(bx, by, pz), rpy=(0.0, 0.0, angle)),
                material=aluminum,
                name=f"rail_bracket_{i}_{j}",
            )

    model.articulation(
        "drum_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("drum_to_rotor")

    ctx.check(
        "continuous vertical rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.check(
        "three named wing panels",
        all(rotor.get_visual(f"panel_{i}") is not None for i in range(3)),
    )
    ctx.check(
        "three low push rails",
        all(rotor.get_visual(f"push_rail_{i}") is not None for i in range(3)),
    )
    ctx.expect_within(
        rotor,
        drum,
        axes="xy",
        margin=0.020,
        name="rotating wing assembly stays inside the cylindrical drum",
    )
    ctx.expect_contact(
        rotor,
        drum,
        elem_a="central_post",
        elem_b="bottom_bearing",
        contact_tol=0.003,
        name="central post sits on the lower bearing",
    )
    ctx.expect_contact(
        rotor,
        drum,
        elem_a="central_post",
        elem_b="top_bearing",
        contact_tol=0.003,
        name="central post is captured by the upper bearing",
    )

    with ctx.pose({joint: math.pi / 3.0}):
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            margin=0.020,
            name="rotating wings remain inside the drum after turning",
        )

    return ctx.report()


object_model = build_object_model()
