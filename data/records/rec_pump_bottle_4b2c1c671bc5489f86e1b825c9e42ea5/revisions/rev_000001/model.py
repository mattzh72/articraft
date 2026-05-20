from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _merge_geometries(geometries) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _add_ring_strip(
    geometry: MeshGeometry,
    ring_a: list[int],
    ring_b: list[int],
    *,
    reverse: bool = False,
) -> None:
    count = len(ring_a)
    for index in range(count):
        a0 = ring_a[index]
        a1 = ring_a[(index + 1) % count]
        b0 = ring_b[index]
        b1 = ring_b[(index + 1) % count]
        if reverse:
            geometry.add_face(a0, b1, b0)
            geometry.add_face(a0, a1, b1)
        else:
            geometry.add_face(a0, b0, b1)
            geometry.add_face(a0, b1, a1)


def _revolve_shell(
    outer_profile,
    inner_profile,
    *,
    segments: int = 64,
    cap_start: bool = True,
    cap_end: bool = True,
) -> MeshGeometry:
    if len(outer_profile) != len(inner_profile):
        raise ValueError("Outer and inner shell profiles must have matching point counts.")

    geometry = MeshGeometry()
    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []

    for radius, z_pos in outer_profile:
        ring = []
        for segment in range(segments):
            angle = (2.0 * math.pi * segment) / segments
            ring.append(geometry.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z_pos))
        outer_rings.append(ring)

    for radius, z_pos in inner_profile:
        ring = []
        for segment in range(segments):
            angle = (2.0 * math.pi * segment) / segments
            ring.append(geometry.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z_pos))
        inner_rings.append(ring)

    for index in range(len(outer_rings) - 1):
        _add_ring_strip(geometry, outer_rings[index], outer_rings[index + 1])

    for index in range(len(inner_rings) - 1):
        _add_ring_strip(geometry, inner_rings[index], inner_rings[index + 1], reverse=True)

    if cap_start:
        _add_ring_strip(geometry, outer_rings[0], inner_rings[0], reverse=True)
    if cap_end:
        _add_ring_strip(geometry, inner_rings[-1], outer_rings[-1], reverse=True)

    return geometry


def _shell_band(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    *,
    crown: float = 0.0,
    segments: int = 56,
) -> MeshGeometry:
    z_mid = 0.5 * (z0 + z1)
    outer_profile = [(outer_radius, z0)]
    inner_profile = [(inner_radius, z0)]
    if crown > 0.0:
        outer_profile.append((outer_radius + crown, z_mid))
        inner_profile.append((inner_radius, z_mid))
    outer_profile.append((outer_radius, z1))
    inner_profile.append((inner_radius, z1))
    return _revolve_shell(
        outer_profile,
        inner_profile,
        segments=segments,
    )


def _build_bottle_shell() -> MeshGeometry:
    outer_profile = [
        (0.012, 0.000),
        (0.027, 0.004),
        (0.036, 0.014),
        (0.039, 0.040),
        (0.039, 0.108),
        (0.038, 0.136),
        (0.033, 0.151),
        (0.024, 0.163),
        (0.018, 0.174),
        (0.018, 0.184),
    ]
    inner_profile = [
        (0.003, 0.003),
        (0.025, 0.007),
        (0.031, 0.017),
        (0.033, 0.041),
        (0.033, 0.107),
        (0.032, 0.134),
        (0.028, 0.146),
        (0.020, 0.158),
        (0.0115, 0.171),
        (0.0115, 0.184),
    ]
    return _revolve_shell(
        outer_profile,
        inner_profile,
        segments=80,
    )


def _build_pump_collar() -> MeshGeometry:
    outer_profile = [
        (0.026, 0.160),
        (0.026, 0.181),
        (0.0175, 0.181),
        (0.0132, 0.186),
        (0.0123, 0.196),
    ]
    inner_profile = [
        (0.018, 0.160),
        (0.018, 0.181),
        (0.0096, 0.181),
        (0.0084, 0.186),
        (0.0084, 0.196),
    ]
    return _revolve_shell(
        outer_profile,
        inner_profile,
        segments=72,
    )


def _build_thread_bands() -> MeshGeometry:
    return _merge_geometries(
        [
            _shell_band(0.0196, 0.0180, 0.165, 0.168, segments=64),
            _shell_band(0.0196, 0.0180, 0.171, 0.174, segments=64),
            _shell_band(0.0194, 0.0180, 0.177, 0.180, segments=64),
        ]
    )


def _build_head_platform() -> MeshGeometry:
    return ExtrudeGeometry(
        rounded_rect_profile(0.068, 0.038, 0.010, corner_segments=8),
        0.016,
        cap=True,
        center=True,
        closed=True,
    )


def _build_lock_ring() -> MeshGeometry:
    return _shell_band(
        0.0134,
        0.0100,
        0.0,
        0.0072,
        crown=0.0009,
        segments=64,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foaming_dispenser_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.79, 0.88, 0.92, 0.88))
    pump_white = model.material("pump_white", rgba=(0.96, 0.96, 0.95, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.77, 0.79, 0.80, 1.0))
    nozzle_white = model.material("nozzle_white", rgba=(0.98, 0.98, 0.97, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        mesh_from_geometry(_build_bottle_shell(), "bottle_shell"),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle_body.visual(
        mesh_from_geometry(_build_thread_bands(), "neck_threads"),
        material=soft_gray,
        name="neck_threads",
    )
    bottle_body.visual(
        mesh_from_geometry(_build_pump_collar(), "pump_collar"),
        material=pump_white,
        name="pump_collar",
    )
    for rib_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        bottle_body.visual(
            Box((0.0040, 0.0030, 0.024)),
            origin=Origin(xyz=(0.0085 * math.cos(angle), 0.0085 * math.sin(angle), 0.178), rpy=(0.0, 0.0, angle)),
            material=soft_gray,
            name=f"guide_rib_{rib_index}",
        )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.196),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0065, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=pump_white,
        name="plunger_shaft",
    )
    plunger.visual(
        Cylinder(radius=0.0115, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=pump_white,
        name="shaft_collar",
    )
    plunger.visual(
        mesh_from_geometry(_build_head_platform(), "head_platform"),
        origin=Origin(xyz=(0.004, 0.0, 0.056)),
        material=nozzle_white,
        name="head_platform",
    )
    plunger.visual(
        Box((0.018, 0.014, 0.010)),
        origin=Origin(xyz=(0.033, 0.0, 0.056)),
        material=nozzle_white,
        name="nozzle_bridge",
    )
    plunger.visual(
        Cylinder(radius=0.0060, length=0.022),
        origin=Origin(xyz=(0.049, 0.0, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_white,
        name="nozzle_body",
    )
    plunger.visual(
        Cylinder(radius=0.0042, length=0.010),
        origin=Origin(xyz=(0.064, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_white,
        name="nozzle_tip",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.082, 0.040, 0.100)),
        mass=0.05,
        origin=Origin(xyz=(0.010, 0.0, 0.045)),
    )

    lock_ring = model.part("lock_ring")
    lock_ring.visual(
        mesh_from_geometry(_build_lock_ring(), "lock_ring_band"),
        origin=Origin(xyz=(0.0, 0.0, -0.0036)),
        material=soft_gray,
        name="lock_band",
    )
    for lug_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        lock_ring.visual(
            Box((0.0056, 0.0024, 0.0056)),
            origin=Origin(xyz=(0.0093 * math.cos(angle), 0.0093 * math.sin(angle), 0.0), rpy=(0.0, 0.0, angle)),
            material=soft_gray,
            name=f"lock_lug_{lug_index}",
        )
    lock_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0135, length=0.0072),
        mass=0.006,
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle_body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.012,
        ),
    )
    model.articulation(
        "plunger_to_lock_ring",
        ArticulationType.CONTINUOUS,
        parent=plunger,
        child=lock_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    plunger = object_model.get_part("plunger")
    lock_ring = object_model.get_part("lock_ring")
    plunger_slide = object_model.get_articulation("body_to_plunger")
    ring_spin = object_model.get_articulation("plunger_to_lock_ring")

    ctx.check(
        "plunger joint follows bottle axis",
        tuple(plunger_slide.axis) == (0.0, 0.0, -1.0),
        details=f"axis={plunger_slide.axis}",
    )
    ctx.check(
        "lock ring rotates around shaft axis",
        tuple(ring_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={ring_spin.axis}",
    )

    with ctx.pose({plunger_slide: 0.0, ring_spin: 0.0}):
        ctx.expect_contact(
            plunger,
            bottle_body,
            contact_tol=0.0002,
            name="plunger assembly is supported by the bottle neck guide",
        )
        ctx.expect_within(
            plunger,
            bottle_body,
            axes="xy",
            inner_elem="plunger_shaft",
            outer_elem="pump_collar",
            margin=0.001,
            name="plunger shaft stays centered in the pump collar",
        )
        ctx.expect_overlap(
            plunger,
            bottle_body,
            axes="z",
            elem_a="plunger_shaft",
            elem_b="pump_collar",
            min_overlap=0.014,
            name="plunger shaft remains inserted through the pump collar at rest",
        )
        ctx.expect_contact(
            lock_ring,
            plunger,
            contact_tol=0.0002,
            name="lock ring is mounted directly on the plunger shaft",
        )
        ctx.expect_gap(
            lock_ring,
            bottle_body,
            axis="z",
            positive_elem="lock_band",
            negative_elem="pump_collar",
            min_gap=0.003,
            max_gap=0.020,
            name="lock ring sits above the fixed pump collar",
        )
        ctx.expect_gap(
            plunger,
            lock_ring,
            axis="z",
            positive_elem="head_platform",
            negative_elem="lock_band",
            min_gap=0.014,
            max_gap=0.030,
            name="lock ring stays below the actuator head",
        )
        ctx.expect_gap(
            plunger,
            plunger,
            axis="x",
            positive_elem="nozzle_tip",
            negative_elem="head_platform",
            min_gap=0.010,
            max_gap=0.035,
            name="front nozzle projects ahead of the actuator head",
        )

        rest_plunger_pos = ctx.part_world_position(plunger)
        rest_ring_pos = ctx.part_world_position(lock_ring)

    with ctx.pose({plunger_slide: 0.012, ring_spin: math.pi / 2.0}):
        ctx.expect_contact(
            plunger,
            bottle_body,
            contact_tol=0.0002,
            name="depressed plunger assembly stays guided by the bottle neck",
        )
        ctx.expect_within(
            plunger,
            bottle_body,
            axes="xy",
            inner_elem="plunger_shaft",
            outer_elem="pump_collar",
            margin=0.001,
            name="depressed plunger shaft stays centered in the pump collar",
        )
        ctx.expect_overlap(
            plunger,
            bottle_body,
            axes="z",
            elem_a="plunger_shaft",
            elem_b="pump_collar",
            min_overlap=0.020,
            name="depressed plunger still remains guided by the pump collar",
        )
        ctx.expect_contact(
            lock_ring,
            plunger,
            contact_tol=0.0002,
            name="lock ring remains seated on the shaft while rotated",
        )
        ctx.expect_gap(
            lock_ring,
            bottle_body,
            axis="z",
            positive_elem="lock_band",
            negative_elem="pump_collar",
            min_gap=0.0001,
            max_gap=0.010,
            name="depressed lock ring still clears the fixed pump collar",
        )

        depressed_plunger_pos = ctx.part_world_position(plunger)
        rotated_ring_pos = ctx.part_world_position(lock_ring)

    ctx.check(
        "plunger depresses downward",
        rest_plunger_pos is not None
        and depressed_plunger_pos is not None
        and depressed_plunger_pos[2] < rest_plunger_pos[2] - 0.008,
        details=f"rest={rest_plunger_pos}, depressed={depressed_plunger_pos}",
    )
    ctx.check(
        "lock ring stays concentric while rotating",
        rest_ring_pos is not None
        and rotated_ring_pos is not None
        and abs(rotated_ring_pos[0] - rest_ring_pos[0]) < 1e-6
        and abs(rotated_ring_pos[1] - rest_ring_pos[1]) < 1e-6,
        details=f"rest={rest_ring_pos}, rotated={rotated_ring_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
