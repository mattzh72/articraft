from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _ribbed_ring_geometry(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    rib_depth: float,
    rib_count: int,
    segments: int = 144,
    bevel: float = 0.001,
) -> MeshGeometry:
    """Build the visible ribbed outer skin of a rotating lens-control ring."""

    geom = MeshGeometry()
    z_levels = [
        -length / 2.0,
        -length / 2.0 + bevel,
        length / 2.0 - bevel,
        length / 2.0,
    ]
    outer_loops: list[list[int]] = []

    for z_index, z in enumerate(z_levels):
        at_edge = z_index in (0, len(z_levels) - 1)
        edge_relief = bevel * 0.55 if at_edge else 0.0
        outer_loop: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            phase = (i * rib_count / segments) % 1.0
            ridge = 1.0 - abs(2.0 * phase - 1.0)
            r_outer = outer_radius + rib_depth * (ridge**2.2) - edge_relief
            c, s = math.cos(theta), math.sin(theta)
            outer_loop.append(geom.add_vertex(r_outer * c, r_outer * s, z))
        outer_loops.append(outer_loop)

    for zi in range(len(z_levels) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            # Outer ribbed side.
            geom.add_face(outer_loops[zi][i], outer_loops[zi][j], outer_loops[zi + 1][j])
            geom.add_face(outer_loops[zi][i], outer_loops[zi + 1][j], outer_loops[zi + 1][i])

    return geom


def _spherical_cap_geometry(*, base_radius: float, cap_height: float, segments: int = 96, rings: int = 16) -> MeshGeometry:
    """Low-profile convex glass cap with a flat circular seating rim."""

    geom = MeshGeometry()
    sphere_radius = (base_radius * base_radius + cap_height * cap_height) / (2.0 * cap_height)
    z_center = cap_height - sphere_radius
    loops: list[list[int]] = []
    for r_index in range(rings + 1):
        t = r_index / rings
        radius = base_radius * (1.0 - t)
        z = z_center + math.sqrt(max(sphere_radius * sphere_radius - radius * radius, 0.0))
        if r_index == rings:
            loops.append([geom.add_vertex(0.0, 0.0, cap_height)])
        else:
            loop: list[int] = []
            for i in range(segments):
                theta = 2.0 * math.pi * i / segments
                loop.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
            loops.append(loop)

    for r_index in range(rings):
        current = loops[r_index]
        nxt = loops[r_index + 1]
        for i in range(segments):
            j = (i + 1) % segments
            if r_index == rings - 1:
                geom.add_face(current[i], current[j], nxt[0])
            else:
                geom.add_face(current[i], current[j], nxt[j])
                geom.add_face(current[i], nxt[j], nxt[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ultra_wide_rectilinear_prime_lens")

    satin_black = model.material("satin_black_anodized", rgba=(0.015, 0.016, 0.018, 1.0))
    matte_black = model.material("matte_black_paint", rgba=(0.045, 0.047, 0.052, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    dark_metal = model.material("dark_graphite_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    mount_metal = model.material("brushed_steel_mount", rgba=(0.62, 0.64, 0.66, 1.0))
    glass = model.material("green_coated_glass", rgba=(0.24, 0.54, 0.58, 0.42))
    white = model.material("engraved_white_paint", rgba=(0.92, 0.92, 0.86, 1.0))
    orange = model.material("aperture_orange_index", rgba=(1.0, 0.38, 0.06, 1.0))

    barrel = model.part("barrel")

    body_outer = [
        (0.031, -0.116),
        (0.036, -0.108),
        (0.036, -0.058),
        (0.038, -0.055),
        (0.038, -0.050),
        (0.033, -0.050),
        (0.033, -0.025),
        (0.038, -0.025),
        (0.038, -0.018),
        (0.042, -0.014),
        (0.044, 0.012),
        (0.050, 0.014),
        (0.052, 0.020),
        (0.047, 0.020),
        (0.047, 0.074),
        (0.047, 0.075),
        (0.052, 0.075),
        (0.050, 0.081),
        (0.056, 0.086),
        (0.063, 0.096),
        (0.067, 0.109),
        (0.064, 0.124),
        (0.058, 0.133),
    ]
    body_inner = [
        (0.022, -0.116),
        (0.023, -0.085),
        (0.024, -0.048),
        (0.026, -0.015),
        (0.029, 0.030),
        (0.034, 0.081),
        (0.042, 0.115),
        (0.046, 0.133),
    ]
    barrel.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                body_outer,
                body_inner,
                segments=112,
                start_cap="flat",
                end_cap="flat",
                lip_samples=8,
            ),
            "bulging_front_barrel_shell",
        ),
        material=satin_black,
        name="body_shell",
    )

    rear_mount_outer = [
        (0.026, -0.132),
        (0.034, -0.130),
        (0.037, -0.124),
        (0.037, -0.116),
        (0.034, -0.112),
    ]
    rear_mount_inner = [
        (0.019, -0.132),
        (0.020, -0.124),
        (0.022, -0.112),
    ]
    barrel.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                rear_mount_outer,
                rear_mount_inner,
                segments=96,
                start_cap="flat",
                end_cap="flat",
                lip_samples=4,
            ),
            "rear_metal_mount",
        ),
        material=mount_metal,
        name="rear_mount",
    )

    barrel.visual(
        mesh_from_geometry(_spherical_cap_geometry(base_radius=0.044, cap_height=0.018), "convex_front_glass"),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=glass,
        name="front_element",
    )
    barrel.visual(
        mesh_from_geometry(_spherical_cap_geometry(base_radius=0.020, cap_height=0.005, segments=64, rings=8), "rear_glass"),
        origin=Origin(xyz=(0.0, 0.0, -0.131), rpy=(math.pi, 0.0, 0.0)),
        material=glass,
        name="rear_element",
    )
    barrel.visual(
        Box((0.006, 0.0022, 0.020)),
        origin=Origin(xyz=(0.0, 0.044, 0.000)),
        material=white,
        name="focus_index_line",
    )
    barrel.visual(
        Sphere(radius=0.0025),
        origin=Origin(xyz=(0.0, 0.038, -0.061)),
        material=orange,
        name="aperture_index_dot",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(
            _ribbed_ring_geometry(
                inner_radius=0.050,
                outer_radius=0.054,
                length=0.055,
                rib_depth=0.0024,
                rib_count=52,
                segments=208,
                bevel=0.002,
            ),
            "front_focus_ring_ribbed",
        ),
        material=rubber,
        name="ribbed_grip",
    )
    for index, theta in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        focus_ring.visual(
            Box((0.0050, 0.0090, 0.006)),
            origin=Origin(
                xyz=(0.0515 * math.cos(theta), 0.0515 * math.sin(theta), 0.0),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            material=rubber,
            name=f"focus_bearing_pad_{index}",
        )
    focus_ring.visual(
        Box((0.0045, 0.0014, 0.038)),
        origin=Origin(xyz=(0.0, 0.0548, 0.0)),
        material=white,
        name="focus_mark",
    )
    for index, angle in enumerate((-0.42, -0.24, 0.24, 0.42)):
        theta = math.pi / 2.0 + angle
        focus_ring.visual(
            Box((0.0030, 0.0030, 0.012)),
            origin=Origin(
                xyz=(0.0548 * math.cos(theta), 0.0548 * math.sin(theta), -0.017 + index * 0.011),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            material=white,
            name=f"focus_scale_tick_{index}",
        )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_geometry(
            _ribbed_ring_geometry(
                inner_radius=0.036,
                outer_radius=0.0405,
                length=0.025,
                rib_depth=0.0013,
                rib_count=42,
                segments=168,
                bevel=0.0012,
            ),
            "rear_aperture_ring_fine_ribbed",
        ),
        material=dark_metal,
        name="fine_click_grip",
    )
    for index, theta in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        aperture_ring.visual(
            Box((0.0050, 0.0080, 0.007)),
            origin=Origin(
                xyz=(0.0370 * math.cos(theta), 0.0370 * math.sin(theta), 0.0),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            material=dark_metal,
            name=f"aperture_bearing_pad_{index}",
        )
    aperture_ring.visual(
        Box((0.0040, 0.0011, 0.014)),
        origin=Origin(xyz=(0.0, 0.0414, 0.0)),
        material=white,
        name="aperture_mark",
    )
    aperture_ring.visual(
        Sphere(radius=0.0018),
        origin=Origin(xyz=(0.010, 0.0402, -0.006)),
        material=orange,
        name="f_stop_dot",
    )

    model.articulation(
        "focus_ring_joint",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "aperture_ring_joint",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=1.5, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    focus_ring = object_model.get_part("focus_ring")
    aperture_ring = object_model.get_part("aperture_ring")
    focus_joint = object_model.get_articulation("focus_ring_joint")
    aperture_joint = object_model.get_articulation("aperture_ring_joint")

    ctx.check(
        "focus ring has a limited revolute lens-axis joint",
        focus_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(focus_joint.axis) == (0.0, 0.0, 1.0)
        and focus_joint.motion_limits.lower < 0.0
        and focus_joint.motion_limits.upper > 0.0,
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}, limits={focus_joint.motion_limits}",
    )
    ctx.check(
        "aperture ring has a separate limited revolute lens-axis joint",
        aperture_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(aperture_joint.axis) == (0.0, 0.0, 1.0)
        and aperture_joint.motion_limits.lower < 0.0
        and aperture_joint.motion_limits.upper > 0.0,
        details=f"type={aperture_joint.articulation_type}, axis={aperture_joint.axis}, limits={aperture_joint.motion_limits}",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="z",
        min_overlap=0.050,
        elem_a="ribbed_grip",
        elem_b="body_shell",
        name="front focus ring surrounds the front sleeve length",
    )
    ctx.expect_overlap(
        aperture_ring,
        barrel,
        axes="z",
        min_overlap=0.020,
        elem_a="fine_click_grip",
        elem_b="body_shell",
        name="rear aperture ring surrounds the rear sleeve length",
    )

    rest_focus_pos = ctx.part_world_position(focus_ring)
    rest_aperture_pos = ctx.part_world_position(aperture_ring)
    with ctx.pose({focus_joint: 0.9, aperture_joint: -0.45}):
        posed_focus_pos = ctx.part_world_position(focus_ring)
        posed_aperture_pos = ctx.part_world_position(aperture_ring)
    ctx.check(
        "ring controls rotate in place without axial translation",
        rest_focus_pos is not None
        and posed_focus_pos is not None
        and rest_aperture_pos is not None
        and posed_aperture_pos is not None
        and max(abs(rest_focus_pos[i] - posed_focus_pos[i]) for i in range(3)) < 1e-7
        and max(abs(rest_aperture_pos[i] - posed_aperture_pos[i]) for i in range(3)) < 1e-7,
        details=f"focus {rest_focus_pos}->{posed_focus_pos}, aperture {rest_aperture_pos}->{posed_aperture_pos}",
    )

    return ctx.report()


object_model = build_object_model()
