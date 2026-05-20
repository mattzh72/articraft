from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _annular_cylinder(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Closed washer/tube mesh centered on local Z."""
    geom = MeshGeometry()
    for z in (-height / 2.0, height / 2.0):
        for radius in (outer_radius, inner_radius):
            for i in range(segments):
                a = 2.0 * math.pi * i / segments
                geom.add_vertex(radius * math.cos(a), radius * math.sin(a), z)

    outer_bottom = 0
    inner_bottom = segments
    outer_top = 2 * segments
    inner_top = 3 * segments
    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom + i, outer_bottom + j, outer_top + j)
        geom.add_face(outer_bottom + i, outer_top + j, outer_top + i)
        # Inner wall, wound inward.
        geom.add_face(inner_bottom + j, inner_bottom + i, inner_top + i)
        geom.add_face(inner_bottom + j, inner_top + i, inner_top + j)
        # Bottom annular face.
        geom.add_face(outer_bottom + j, outer_bottom + i, inner_bottom + i)
        geom.add_face(outer_bottom + j, inner_bottom + i, inner_bottom + j)
        # Top annular face.
        geom.add_face(outer_top + i, outer_top + j, inner_top + j)
        geom.add_face(outer_top + i, inner_top + j, inner_top + i)
    return geom


def _translate(geom: MeshGeometry, xyz: tuple[float, float, float]) -> MeshGeometry:
    return geom.translate(xyz[0], xyz[1], xyz[2])


def _align_z_to_vector(geom: MeshGeometry, vector: tuple[float, float, float]) -> MeshGeometry:
    vx, vy, vz = vector
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1e-9:
        return geom
    vx, vy, vz = vx / length, vy / length, vz / length
    dot = max(-1.0, min(1.0, vz))
    ax, ay, az = -vy, vx, 0.0
    axis_len = math.sqrt(ax * ax + ay * ay + az * az)
    if axis_len <= 1e-9:
        if dot < 0.0:
            geom.rotate_x(math.pi)
        return geom
    geom.rotate((ax / axis_len, ay / axis_len, az / axis_len), math.acos(dot))
    return geom


def _oval_tube_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    rx: float,
    ry: float,
    segments: int = 32,
) -> MeshGeometry:
    dx, dy, dz = end[0] - start[0], end[1] - start[1], end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(1.0, length, radial_segments=segments, closed=True)
    geom.scale(rx, ry, 1.0)
    _align_z_to_vector(geom, (dx, dy, dz))
    geom.translate(
        (start[0] + end[0]) / 2.0,
        (start[1] + end[1]) / 2.0,
        (start[2] + end[2]) / 2.0,
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_fork_quill_pursuit_bars")

    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    darker_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    frame_paint = model.material("black_head_tube", rgba=(0.02, 0.025, 0.03, 1.0))

    # A fixed head tube and headset give the fork a realistic steering bearing.
    head_tube = model.part("head_tube")
    head_shell = _annular_cylinder(0.028, 0.0185, 0.260, segments=64)
    _translate(head_shell, (0.0, 0.0, 0.250))
    head_tube.visual(
        mesh_from_geometry(head_shell, "head_tube_shell"),
        material=frame_paint,
        name="head_tube_shell",
    )
    upper_cup = _annular_cylinder(0.036, 0.0172, 0.026, segments=64)
    _translate(upper_cup, (0.0, 0.0, 0.393))
    head_tube.visual(
        mesh_from_geometry(upper_cup, "upper_cup"),
        material=polished_steel,
        name="upper_cup",
    )
    lower_cup = _annular_cylinder(0.036, 0.0172, 0.026, segments=64)
    _translate(lower_cup, (0.0, 0.0, 0.107))
    head_tube.visual(
        mesh_from_geometry(lower_cup, "lower_cup"),
        material=polished_steel,
        name="lower_cup",
    )

    steering = model.part("steering")

    # Threaded steerer, locknut, and visible thread crests above the headset.
    steering.visual(
        Cylinder(radius=0.0132, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=polished_steel,
        name="steerer",
    )
    crown_race = _annular_cylinder(0.034, 0.0124, 0.014, segments=56)
    _translate(crown_race, (0.0, 0.0, 0.087))
    steering.visual(
        mesh_from_geometry(crown_race, "crown_race"),
        material=polished_steel,
        name="crown_race",
    )
    upper_cone = _annular_cylinder(0.032, 0.0124, 0.010, segments=56)
    _translate(upper_cone, (0.0, 0.0, 0.411))
    steering.visual(
        mesh_from_geometry(upper_cone, "upper_cone"),
        material=polished_steel,
        name="upper_cone",
    )
    for n, z in enumerate((0.440, 0.449, 0.458, 0.467, 0.476, 0.485)):
        steering.visual(
            Cylinder(radius=0.0154, length=0.0034),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=polished_steel,
            name=f"thread_crest_{n}",
        )
    locknut_profile = [
        (0.027 * math.cos(a), 0.027 * math.sin(a))
        for a in [math.pi / 6.0 + i * math.pi / 3.0 for i in range(6)]
    ]
    locknut = ExtrudeGeometry(locknut_profile, 0.014, cap=True, center=True)
    _translate(locknut, (0.0, 0.0, 0.428))
    steering.visual(
        mesh_from_geometry(locknut, "upper_locknut"),
        material=polished_steel,
        name="upper_locknut",
    )

    # Slim oval-section steel crown with sockets for two straight blades.
    crown = ExtrudeGeometry(superellipse_profile(0.120, 0.190, exponent=2.25, segments=72), 0.046)
    steering.visual(
        mesh_from_geometry(crown, "oval_crown"),
        material=polished_steel,
        name="oval_crown",
    )
    for side, y in enumerate((-0.055, 0.055)):
        socket = _oval_tube_between(
            (0.030, y, -0.010),
            (0.036, y, -0.080),
            rx=0.014,
            ry=0.010,
            segments=32,
        )
        steering.visual(
            mesh_from_geometry(socket, f"blade_socket_{side}"),
            material=polished_steel,
            name=f"blade_socket_{side}",
        )

    blade_profile = superellipse_profile(0.020, 0.031, exponent=2.15, segments=40)
    for side, y in enumerate((-0.055, 0.055)):
        blade = sweep_profile_along_spline(
            [(0.033, y, -0.034), (0.075, y, -0.710)],
            profile=blade_profile,
            samples_per_segment=3,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        )
        steering.visual(
            mesh_from_geometry(blade, f"straight_blade_{side}"),
            material=polished_steel,
            name=f"straight_blade_{side}",
        )
        steering.visual(
            Box((0.050, 0.014, 0.038)),
            origin=Origin(xyz=(0.086, y, -0.720)),
            material=polished_steel,
            name=f"dropout_plate_{side}",
        )
        steering.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(0.098, y, -0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=darker_steel,
            name=f"axle_slot_{side}",
        )

    # Quill stem: the slim quill descends inside the steerer and is tightened by
    # a center expander bolt at its top.
    steering.visual(
        Cylinder(radius=0.0105, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=polished_steel,
        name="quill",
    )
    steering.visual(
        Cylinder(radius=0.0135, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.616)),
        material=polished_steel,
        name="quill_top_cap",
    )
    steering.visual(
        Cylinder(radius=0.0040, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.626)),
        material=darker_steel,
        name="expander_bolt",
    )
    steering.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.646)),
        material=darker_steel,
        name="expander_bolt_head",
    )

    stem_extension = _oval_tube_between(
        (0.000, 0.0, 0.555),
        (0.205, 0.0, 0.500),
        rx=0.013,
        ry=0.010,
        segments=40,
    )
    steering.visual(
        mesh_from_geometry(stem_extension, "stem_extension"),
        material=polished_steel,
        name="stem_extension",
    )

    collar = _annular_cylinder(0.030, 0.0107, 0.078, segments=56)
    collar.rotate_x(-math.pi / 2.0)
    _translate(collar, (0.215, 0.0, 0.497))
    steering.visual(
        mesh_from_geometry(collar, "pinch_collar"),
        material=polished_steel,
        name="pinch_collar",
    )
    steering.visual(
        Box((0.016, 0.064, 0.012)),
        origin=Origin(xyz=(0.218, 0.0, 0.529)),
        material=polished_steel,
        name="pinch_lug",
    )
    steering.visual(
        Cylinder(radius=0.0035, length=0.070),
        origin=Origin(xyz=(0.218, 0.0, 0.529), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="pinch_bolt",
    )

    pursuit_bar = tube_from_spline_points(
        [
            (0.382, -0.225, 0.388),
            (0.320, -0.232, 0.392),
            (0.268, -0.235, 0.428),
            (0.224, -0.175, 0.488),
            (0.205, -0.070, 0.500),
            (0.205, 0.000, 0.500),
            (0.205, 0.070, 0.500),
            (0.224, 0.175, 0.488),
            (0.268, 0.235, 0.428),
            (0.320, 0.232, 0.392),
            (0.382, 0.225, 0.388),
        ],
        radius=0.0108,
        samples_per_segment=12,
        radial_segments=24,
        cap_ends=True,
    )
    steering.visual(
        mesh_from_geometry(pursuit_bar, "pursuit_bar"),
        material=polished_steel,
        name="pursuit_bar",
    )
    for side, y in enumerate((-0.225, 0.225)):
        steering.visual(
            Cylinder(radius=0.0136, length=0.080),
            origin=Origin(xyz=(0.355, y, 0.390), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"grip_{side}",
        )
        steering.visual(
            Cylinder(radius=0.0138, length=0.004),
            origin=Origin(xyz=(0.397, y, 0.390), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_steel,
            name=f"bar_end_plug_{side}",
        )

    model.articulation(
        "steering_axis",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0, lower=-1.22, upper=1.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    steering = object_model.get_part("steering")
    steering_axis = object_model.get_articulation("steering_axis")

    ctx.check(
        "steering uses limited revolute headset motion",
        steering_axis.articulation_type == ArticulationType.REVOLUTE
        and steering_axis.motion_limits is not None
        and steering_axis.motion_limits.lower <= -1.0
        and steering_axis.motion_limits.upper >= 1.0,
        details=f"type={steering_axis.articulation_type}, limits={steering_axis.motion_limits}",
    )
    ctx.expect_overlap(
        steering,
        head_tube,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube_shell",
        min_overlap=0.20,
        name="steerer passes through the headset shell",
    )
    ctx.expect_within(
        steering,
        head_tube,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube_shell",
        margin=0.0,
        name="steerer stays coaxial inside the head tube",
    )

    rest_bar = ctx.part_element_world_aabb(steering, elem="pursuit_bar")
    with ctx.pose({steering_axis: 0.65}):
        turned_bar = ctx.part_element_world_aabb(steering, elem="pursuit_bar")
    rest_y = None if rest_bar is None else (rest_bar[0][1] + rest_bar[1][1]) / 2.0
    turned_y = None if turned_bar is None else (turned_bar[0][1] + turned_bar[1][1]) / 2.0
    ctx.check(
        "handlebars swing about the steerer axis",
        rest_y is not None and turned_y is not None and turned_y > rest_y + 0.10,
        details=f"rest_y={rest_y}, turned_y={turned_y}",
    )

    return ctx.report()


object_model = build_object_model()
