from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helix_points(
    radius: float,
    z0: float,
    z1: float,
    turns: float,
    *,
    samples: int = 96,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples + 1):
        u = i / samples
        theta = phase + 2.0 * math.pi * turns * u
        points.append((radius * math.cos(theta), radius * math.sin(theta), z0 + (z1 - z0) * u))
    return points


def _cylindrical_ring_mesh(
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    *,
    bevel: float = 0.0,
    segments: int = 72,
) -> object:
    if bevel <= 0.0:
        profile = [
            (inner_radius, z0),
            (outer_radius, z0),
            (outer_radius, z1),
            (inner_radius, z1),
            (inner_radius, z0),
        ]
    else:
        profile = [
            (inner_radius, z0 + bevel),
            (inner_radius + bevel, z0),
            (outer_radius - bevel, z0),
            (outer_radius, z0 + bevel),
            (outer_radius, z1 - bevel),
            (outer_radius - bevel, z1),
            (inner_radius + bevel, z1),
            (inner_radius, z1 - bevel),
            (inner_radius, z0 + bevel),
        ]
    return LatheGeometry(profile, segments=segments, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_bulb_socket")

    ceramic = model.material("warm_white_ceramic", rgba=(0.86, 0.82, 0.74, 1.0))
    shadow = model.material("cavity_shadow", rgba=(0.05, 0.047, 0.043, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.76, 0.70, 1.0))
    brass = model.material("warm_brass", rgba=(0.95, 0.66, 0.25, 1.0))
    frosted_glass = model.material("frosted_warm_glass", rgba=(1.0, 0.92, 0.70, 0.58))
    ink = model.material("blue_gray_print", rgba=(0.10, 0.15, 0.20, 1.0))
    tungsten = model.material("dim_tungsten", rgba=(0.95, 0.58, 0.18, 1.0))

    socket = model.part("socket")

    socket_body = LatheGeometry.from_shell_profiles(
        [
            (0.055, 0.000),
            (0.074, 0.006),
            (0.078, 0.020),
            (0.066, 0.037),
            (0.061, 0.116),
            (0.057, 0.133),
        ],
        [
            (0.018, 0.038),
            (0.034, 0.039),
            (0.047, 0.055),
            (0.049, 0.118),
            (0.044, 0.131),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    socket.visual(
        mesh_from_geometry(socket_body, "socket_ceramic_body"),
        material=ceramic,
        name="ceramic_body",
    )

    socket.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=shadow,
        name="cavity_shadow",
    )

    collar = _cylindrical_ring_mesh(0.0365, 0.049, 0.040, 0.126, bevel=0.0025)
    socket.visual(
        mesh_from_geometry(collar, "socket_metal_collar"),
        material=aluminum,
        name="metal_collar",
    )

    collar_thread = tube_from_spline_points(
        _helix_points(0.0372, 0.051, 0.118, 3.1, samples=112, phase=0.35),
        radius=0.0011,
        samples_per_segment=2,
        radial_segments=8,
        cap_ends=True,
    )
    socket.visual(
        mesh_from_geometry(collar_thread, "socket_collar_thread"),
        material=aluminum,
        name="collar_thread",
    )

    socket.visual(
        Cylinder(radius=0.0185, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=shadow,
        name="contact_insulator",
    )

    socket.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=brass,
        name="socket_contact",
    )

    bulb = model.part("bulb")

    glass = LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.000),
            (0.030, 0.010),
            (0.052, 0.025),
            (0.080, 0.050),
            (0.088, 0.075),
            (0.072, 0.100),
            (0.038, 0.117),
            (0.006, 0.124),
        ],
        [
            (0.020, 0.004),
            (0.024, 0.013),
            (0.045, 0.028),
            (0.072, 0.052),
            (0.079, 0.075),
            (0.064, 0.097),
            (0.033, 0.110),
            (0.004, 0.117),
        ],
        segments=112,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    bulb.visual(
        mesh_from_geometry(glass, "bulb_glass_envelope"),
        material=frosted_glass,
        name="glass_envelope",
    )

    base_sleeve = LatheGeometry(
        [
            (0.000, -0.074),
            (0.012, -0.074),
            (0.017, -0.069),
            (0.028, -0.067),
            (0.029, -0.010),
            (0.033, -0.003),
            (0.033, 0.004),
            (0.026, 0.008),
            (0.000, 0.008),
            (0.000, -0.074),
        ],
        segments=96,
        closed=True,
    )
    bulb.visual(
        mesh_from_geometry(base_sleeve, "bulb_base_sleeve"),
        material=aluminum,
        name="base_sleeve",
    )

    base_thread = tube_from_spline_points(
        _helix_points(0.0307, -0.064, -0.011, 3.35, samples=128, phase=-0.45),
        radius=0.0017,
        samples_per_segment=2,
        radial_segments=10,
        cap_ends=True,
    )
    bulb.visual(
        mesh_from_geometry(base_thread, "bulb_base_thread"),
        material=aluminum,
        name="base_thread",
    )

    bulb.visual(
        Cylinder(radius=0.0115, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.073)),
        material=brass,
        name="central_contact",
    )

    filament = tube_from_spline_points(
        [
            (-0.010, 0.000, 0.004),
            (-0.010, 0.000, 0.039),
            (-0.006, 0.000, 0.050),
            (0.000, 0.000, 0.054),
            (0.006, 0.000, 0.050),
            (0.010, 0.000, 0.039),
            (0.010, 0.000, 0.004),
        ],
        radius=0.0008,
        samples_per_segment=9,
        radial_segments=8,
        cap_ends=True,
    )
    bulb.visual(
        mesh_from_geometry(filament, "bulb_filament"),
        material=tungsten,
        name="filament",
    )

    bulb.visual(
        Box((0.030, 0.0010, 0.012)),
        origin=Origin(xyz=(0.0, -0.087, 0.080)),
        material=ink,
        name="front_mark",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    joint = object_model.get_articulation("socket_to_bulb")

    ctx.check(
        "bulb uses a continuous screw axis",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="base_thread",
        elem_b="metal_collar",
        min_overlap=0.045,
        name="threaded bulb base is seated inside the collar height",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="base_thread",
        outer_elem="metal_collar",
        margin=0.0,
        name="bulb thread footprint stays within socket collar footprint",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="central_contact",
        elem_b="socket_contact",
        contact_tol=0.001,
        name="bottom electrical contacts meet when seated",
    )

    rest_pos = ctx.part_world_position(bulb)
    rest_aabb = ctx.part_element_world_aabb(bulb, elem="front_mark")
    with ctx.pose({joint: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(bulb)
        spun_aabb = ctx.part_element_world_aabb(bulb, elem="front_mark")
        ctx.expect_contact(
            bulb,
            socket,
            elem_a="central_contact",
            elem_b="socket_contact",
            contact_tol=0.001,
            name="contacts remain seated after screw rotation",
        )

    ctx.check(
        "rotation changes the visible bulb mark without translating the axis",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(rest_pos[i] - spun_pos[i]) for i in range(3)) < 1e-6
        and rest_aabb is not None
        and spun_aabb is not None
        and abs(((rest_aabb[0][0] + rest_aabb[1][0]) - (spun_aabb[0][0] + spun_aabb[1][0])) / 2.0) > 0.035,
        details=f"rest_pos={rest_pos}, spun_pos={spun_pos}, rest_mark={rest_aabb}, spun_mark={spun_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
