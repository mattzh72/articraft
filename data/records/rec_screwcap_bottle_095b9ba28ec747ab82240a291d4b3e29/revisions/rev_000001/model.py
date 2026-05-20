from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _revolved_profile_mesh(
    profile: list[tuple[float, float]],
    *,
    segments: int = 96,
) -> MeshGeometry:
    """Revolve a closed radius/z cross-section around the +Z axis."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for radius, z in profile:
        ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            ring.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        rings.append(ring)

    count = len(profile)
    for j in range(count):
        nxt_j = (j + 1) % count
        for i in range(segments):
            nxt_i = (i + 1) % segments
            a = rings[j][i]
            b = rings[j][nxt_i]
            c = rings[nxt_j][nxt_i]
            d = rings[nxt_j][i]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _annular_sleeve_mesh(inner_radius: float, outer_radius: float, z_min: float, z_max: float) -> MeshGeometry:
    return _revolved_profile_mesh(
        [
            (outer_radius, z_min),
            (outer_radius, z_max),
            (inner_radius, z_max),
            (inner_radius, z_min),
        ],
        segments=96,
    )


def _neck_thread_mesh(
    *,
    radius: float,
    z_min: float,
    z_max: float,
    turns: float,
    phase: float = 0.0,
) -> MeshGeometry:
    points: list[tuple[float, float, float]] = []
    samples = 72
    for i in range(samples + 1):
        t = i / samples
        theta = phase + turns * 2.0 * math.pi * t
        points.append((radius * math.cos(theta), radius * math.sin(theta), z_min + (z_max - z_min) * t))
    return tube_from_spline_points(
        points,
        radius=0.00075,
        samples_per_segment=3,
        radial_segments=10,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    clear_plastic = model.material("clear_plastic", rgba=(0.65, 0.92, 1.0, 0.46))
    thread_plastic = model.material("thread_plastic", rgba=(0.86, 0.96, 1.0, 0.72))
    cap_blue = model.material("cap_blue", rgba=(0.035, 0.16, 0.72, 1.0))
    cap_mark = model.material("cap_mark", rgba=(0.94, 0.96, 1.0, 1.0))
    sleeve_material = model.material("sleeve_material", rgba=(0.92, 0.92, 0.86, 0.62))

    bottle = model.part("bottle")
    bottle_shell = _revolved_profile_mesh(
        [
            (0.029, 0.004),
            (0.035, 0.012),
            (0.039, 0.035),
            (0.039, 0.125),
            (0.037, 0.150),
            (0.030, 0.172),
            (0.020, 0.187),
            (0.0135, 0.198),
            (0.0135, 0.258),
            (0.0090, 0.258),
            (0.0090, 0.202),
            (0.016, 0.190),
            (0.028, 0.175),
            (0.034, 0.152),
            (0.035, 0.035),
            (0.031, 0.012),
            (0.010, 0.006),
        ],
        segments=128,
    )
    bottle.visual(
        mesh_from_geometry(bottle_shell, "bottle_shell"),
        material=clear_plastic,
        name="bottle_shell",
    )
    bottle.visual(
        mesh_from_geometry(_annular_sleeve_mesh(0.0120, 0.0190, 0.193, 0.197), "support_flange"),
        material=thread_plastic,
        name="support_flange",
    )
    bottle.visual(
        mesh_from_geometry(
            _neck_thread_mesh(radius=0.01415, z_min=0.200, z_max=0.247, turns=2.35),
            "threaded_neck",
        ),
        material=thread_plastic,
        name="threaded_neck",
    )

    axis_sleeve = model.part("axis_sleeve")
    axis_sleeve.visual(
        mesh_from_geometry(_annular_sleeve_mesh(0.0160, 0.0180, 0.197, 0.207), "sleeve_shell"),
        material=sleeve_material,
        name="sleeve_shell",
    )
    model.articulation(
        "bottle_to_axis_sleeve",
        ArticulationType.FIXED,
        parent=bottle,
        child=axis_sleeve,
        origin=Origin(),
    )

    cap = model.part("cap")
    cap_shell = _revolved_profile_mesh(
        [
            (0.0240, 0.000),
            (0.0240, 0.052),
            (0.0225, 0.062),
            (0.0010, 0.064),
            (0.0010, 0.060),
            (0.0170, 0.060),
            (0.0170, 0.004),
            (0.0170, 0.000),
        ],
        segments=128,
    )
    cap.visual(mesh_from_geometry(cap_shell, "cap_shell"), material=cap_blue, name="cap_shell")

    for i in range(32):
        angle = 2.0 * math.pi * i / 32
        cap.visual(
            Box((0.0022, 0.0030, 0.042)),
            origin=Origin(
                xyz=(0.0241 * math.cos(angle), 0.0241 * math.sin(angle), 0.026),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_blue,
            name=f"grip_rib_{i:02d}",
        )
    cap.visual(
        Box((0.016, 0.003, 0.0012)),
        origin=Origin(xyz=(0.005, 0.0, 0.0644)),
        material=cap_mark,
        name="top_mark",
    )

    model.articulation(
        "axis_sleeve_to_cap",
        ArticulationType.CONTINUOUS,
        parent=axis_sleeve,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.207)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    axis_sleeve = object_model.get_part("axis_sleeve")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("axis_sleeve_to_cap")

    ctx.expect_contact(
        axis_sleeve,
        bottle,
        elem_a="sleeve_shell",
        elem_b="support_flange",
        contact_tol=0.0008,
        name="fixed sleeve sits on bottle flange",
    )
    ctx.expect_contact(
        cap,
        axis_sleeve,
        elem_a="cap_shell",
        elem_b="sleeve_shell",
        contact_tol=0.0008,
        name="cap bears on sleeve support",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="cap_shell",
        elem_b="threaded_neck",
        min_overlap=0.035,
        name="cap surrounds threaded neck height",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="threaded_neck",
        outer_elem="cap_shell",
        margin=0.001,
        name="threaded neck fits inside cap bore",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: math.pi}):
        spun_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap spins about fixed neck axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )
    return ctx.report()


object_model = build_object_model()
