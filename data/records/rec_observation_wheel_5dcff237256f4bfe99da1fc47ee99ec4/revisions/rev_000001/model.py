from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _tube_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    segments: int = 16,
) -> MeshGeometry:
    """Build a round tube between two points, using a centered +Z cylinder."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1e-9:
        raise ValueError("tube endpoints must be distinct")

    geom = CylinderGeometry(radius, length, radial_segments=segments, closed=True)
    ux, uy, uz = vx / length, vy / length, vz / length

    # Rotate the cylinder's local +Z axis onto the requested direction.
    dot = max(-1.0, min(1.0, uz))
    angle = math.acos(dot)
    ax, ay, az = -uy, ux, 0.0  # cross((0, 0, 1), direction)
    axis_len = math.sqrt(ax * ax + ay * ay + az * az)
    if axis_len > 1e-9:
        geom.rotate((ax / axis_len, ay / axis_len, az / axis_len), angle)
    elif uz < 0.0:
        geom.rotate((1.0, 0.0, 0.0), math.pi)

    geom.translate((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5)
    return geom


def _add_tube(
    assembly: MeshGeometry,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    segments: int = 16,
) -> None:
    assembly.merge(_tube_between(start, end, radius, segments=segments))


def _build_support_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    axle_z = 1.85
    tower_y = 0.42
    foot_x = 1.25
    tube = 0.032

    for y in (-tower_y, tower_y):
        # In each side tower, two splayed legs and a base tie form an A-frame.
        _add_tube(geom, (-foot_x, y, 0.08), (0.0, y, axle_z), tube, segments=18)
        _add_tube(geom, (foot_x, y, 0.08), (0.0, y, axle_z), tube, segments=18)
        _add_tube(geom, (-foot_x, y, 0.08), (foot_x, y, 0.08), tube * 0.9, segments=18)
        # Mid-height bracing keeps each tower visibly triangulated.
        _add_tube(geom, (-0.72, y, 0.76), (0.72, y, 0.76), tube * 0.65, segments=14)

    # Cross-members tie the two side towers together.
    for x in (-foot_x, foot_x):
        _add_tube(geom, (x, -tower_y, 0.08), (x, tower_y, 0.08), tube * 0.85, segments=18)
    _add_tube(geom, (0.0, -tower_y, axle_z), (0.0, -0.265, axle_z), tube, segments=18)
    _add_tube(geom, (0.0, 0.265, axle_z), (0.0, tower_y, axle_z), tube, segments=18)

    return geom


def _build_wheel_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    outer_r = 1.0
    inner_r = 0.77
    side_y = 0.155
    rim_tube = 0.024
    spoke_tube = 0.012
    angles = [2.0 * math.pi * i / 16 for i in range(16)]

    for y in (-side_y, side_y):
        outer = TorusGeometry(outer_r, rim_tube, radial_segments=18, tubular_segments=96)
        outer.rotate_x(math.pi / 2.0).translate(0.0, y, 0.0)
        geom.merge(outer)

        inner = TorusGeometry(inner_r, rim_tube * 0.75, radial_segments=14, tubular_segments=96)
        inner.rotate_x(math.pi / 2.0).translate(0.0, y, 0.0)
        geom.merge(inner)

        for theta in angles:
            c, s = math.cos(theta), math.sin(theta)
            _add_tube(
                geom,
                (0.08 * c, y, 0.08 * s),
                (outer_r * c, y, outer_r * s),
                spoke_tube,
                segments=12,
            )

    # A central axle drum and small radial brackets for the gondola hangers.
    _add_tube(geom, (0.0, -0.21, 0.0), (0.0, 0.21, 0.0), 0.070, segments=28)

    hanger_r = 1.20
    bracket_r = hanger_r
    for theta in [2.0 * math.pi * i / 8 for i in range(8)]:
        c, s = math.cos(theta), math.sin(theta)
        for y in (-0.130, 0.130):
            _add_tube(
                geom,
                (outer_r * c, y, outer_r * s),
                (bracket_r * c, y, bracket_r * s),
                0.010,
                segments=10,
            )

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a_frame_observation_wheel")

    painted_steel = model.material("painted_steel", rgba=(0.88, 0.90, 0.92, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.14, 0.16, 1.0))
    blue_glass = model.material("blue_glass", rgba=(0.35, 0.65, 0.90, 0.72))
    cabin_red = model.material("cabin_red", rgba=(0.86, 0.15, 0.10, 1.0))
    cabin_roof = model.material("cabin_roof", rgba=(0.95, 0.82, 0.18, 1.0))
    concrete = model.material("concrete", rgba=(0.50, 0.50, 0.48, 1.0))

    support = model.part("support")
    support.visual(
        Box((2.75, 1.05, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=concrete,
        name="foundation",
    )
    support.visual(
        mesh_from_geometry(_build_support_geometry(), "a_frame_support"),
        material=painted_steel,
        name="a_frame",
    )
    # Bearing housings flank the wheel hub on the side towers.
    for y, name in ((-0.245, "bearing_0"), (0.245, "bearing_1")):
        support.visual(
            Cylinder(radius=0.095, length=0.075),
            origin=Origin(xyz=(0.0, y, 1.85), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=name,
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(_build_wheel_geometry(), "wheel_rim_spokes"),
        material=painted_steel,
        name="rim_spokes",
    )
    wheel.visual(
        Cylinder(radius=0.115, length=0.30),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.85)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35),
    )

    hanger_r = 1.20
    gondola_count = 8
    for i in range(gondola_count):
        theta = 2.0 * math.pi * i / gondola_count
        c, s = math.cos(theta), math.sin(theta)
        gondola = model.part(f"gondola_{i}")
        # Child frame is the hanger pin. Geometry extends downward in the
        # gondola frame so mimic counter-rotation keeps the cabin vertical.
        gondola.visual(
            Cylinder(radius=0.014, length=0.250),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_pin",
        )
        for y, name in ((-0.082, "hanger_0"), (0.082, "hanger_1")):
            gondola.visual(
                Box((0.018, 0.012, 0.280)),
                origin=Origin(xyz=(0.0, y, -0.140)),
                material=dark_steel,
                name=name,
            )
        gondola.visual(
            Box((0.260, 0.180, 0.160)),
            origin=Origin(xyz=(0.0, 0.0, -0.390)),
            material=cabin_red,
            name="cabin_body",
        )
        gondola.visual(
            Box((0.290, 0.205, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, -0.295)),
            material=cabin_roof,
            name="roof",
        )
        for y, name in ((-0.093, "window_0"), (0.093, "window_1")):
            gondola.visual(
                Box((0.160, 0.006, 0.060)),
                origin=Origin(xyz=(0.0, 0.091 if y > 0 else -0.091, -0.390)),
                material=blue_glass,
                name=name,
            )

        model.articulation(
            f"wheel_to_gondola_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(hanger_r * c, 0.0, hanger_r * s)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=2.0),
            mimic=Mimic("support_to_wheel", multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("support_to_wheel")

    ctx.check(
        "wheel has a horizontal rotation axis",
        tuple(round(v, 6) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={wheel_joint.axis}",
    )
    ctx.expect_overlap(
        wheel,
        support,
        axes="y",
        min_overlap=0.25,
        elem_a="hub",
        elem_b="a_frame",
        name="hub spans between side towers",
    )

    gondolas = [object_model.get_part(f"gondola_{i}") for i in range(8)]
    for i, gondola in enumerate(gondolas):
        joint = object_model.get_articulation(f"wheel_to_gondola_{i}")
        ctx.check(
            f"gondola_{i} counter-rotates with wheel",
            joint.mimic is not None
            and joint.mimic.joint == "support_to_wheel"
            and abs(joint.mimic.multiplier + 1.0) < 1e-9,
            details=f"mimic={joint.mimic}",
        )
        ctx.allow_overlap(
            gondola,
            wheel,
            elem_a="pivot_pin",
            elem_b="rim_spokes",
            reason=(
                "The gondola hanger pin is intentionally captured in a small "
                "wheel-mounted bracket so the cabin has a visible pivot."
            ),
        )
        ctx.expect_overlap(
            gondola,
            wheel,
            axes="y",
            min_overlap=0.16,
            elem_a="pivot_pin",
            elem_b="rim_spokes",
            name=f"gondola_{i} pin is retained across the bracket width",
        )
        ox, _, oz = joint.origin.xyz
        ctx.check(
            f"gondola_{i} pivot is on outer rim",
            1.16 < math.hypot(ox, oz) < 1.24,
            details=f"origin={joint.origin.xyz}",
        )

    # At a quarter turn, the wheel has moved but the mimicked gondolas should
    # still hang below their pivot pins rather than rotating rigidly with the rim.
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        for i in (0, 2, 4, 6):
            gondola = object_model.get_part(f"gondola_{i}")
            pivot_pos = ctx.part_world_position(gondola)
            body_aabb = ctx.part_element_world_aabb(gondola, elem="cabin_body")
            body_max_z = body_aabb[1][2] if body_aabb is not None else None
            ctx.check(
                f"gondola_{i} hangs upright at quarter turn",
                pivot_pos is not None and body_max_z is not None and body_max_z < pivot_pos[2] - 0.11,
                details=f"pivot={pivot_pos}, body_aabb={body_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
