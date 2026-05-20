from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _superellipse_loop(
    width: float,
    depth: float,
    z: float,
    *,
    y_offset: float = 0.0,
    segments: int = 96,
    exponent: float = 2.6,
    scale: float = 1.0,
) -> list[int]:
    """Return point coordinates for a smooth oval/superellipse loop."""
    pts: list[tuple[float, float, float]] = []
    power = 2.0 / exponent
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = 0.5 * width * scale * math.copysign(abs(c) ** power, c)
        y = y_offset + 0.5 * depth * scale * math.copysign(abs(s) ** power, s)
        pts.append((x, y, z))
    return pts


def _add_loop(geom: MeshGeometry, pts: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in pts]


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _basin_shell_geometry() -> MeshGeometry:
    """A connected ceramic rim with a visible hollow oval basin."""
    geom = MeshGeometry()
    segments = 96

    outer_top = _add_loop(
        geom,
        _superellipse_loop(0.64, 0.46, 0.84, segments=segments, exponent=3.0),
    )
    outer_bottom = _add_loop(
        geom,
        _superellipse_loop(0.64, 0.46, 0.765, segments=segments, exponent=3.0),
    )
    inner_top_pts = _superellipse_loop(
        0.42, 0.27, 0.84, y_offset=-0.055, segments=segments, exponent=2.2
    )
    inner_top = _add_loop(geom, inner_top_pts)
    inner_lip = _add_loop(
        geom,
        _superellipse_loop(
            0.42, 0.27, 0.772, y_offset=-0.055, segments=segments, exponent=2.2
        ),
    )

    # Solid ceramic rim: top annulus, outside skirt, underside, and inner drop.
    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(geom, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        _add_quad(geom, outer_bottom[j], outer_bottom[i], inner_lip[i], inner_lip[j])
        _add_quad(geom, outer_top[j], outer_top[i], outer_bottom[i], outer_bottom[j])
        _add_quad(geom, inner_top[i], inner_top[j], inner_lip[j], inner_lip[i])

    # Curved concave bowl surface below the rim, sharing the lip loop vertices.
    bowl_loops = [inner_lip]
    for width, depth, z, scale in (
        (0.42, 0.27, 0.735, 0.82),
        (0.42, 0.27, 0.695, 0.56),
        (0.42, 0.27, 0.672, 0.22),
    ):
        bowl_loops.append(
            _add_loop(
                geom,
                _superellipse_loop(
                    width,
                    depth,
                    z,
                    y_offset=-0.055,
                    segments=segments,
                    exponent=2.0,
                    scale=scale,
                ),
            )
        )

    for lower, upper in zip(bowl_loops, bowl_loops[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            _add_quad(geom, lower[i], lower[j], upper[j], upper[i])

    drain_center = geom.add_vertex(0.0, -0.055, 0.666)
    bottom = bowl_loops[-1]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom[j], bottom[i], drain_center)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_basin_faucet")

    chrome = Material("polished_chrome", rgba=(0.72, 0.78, 0.84, 1.0))
    dark = Material("dark_aerator", rgba=(0.015, 0.016, 0.018, 1.0))
    ceramic = Material("gloss_white_ceramic", rgba=(0.96, 0.95, 0.91, 1.0))

    basin = model.part("basin")
    basin.visual(
        mesh_from_geometry(_basin_shell_geometry(), "ceramic_basin_shell"),
        material=ceramic,
        name="basin_shell",
    )

    body = model.part("column_body")
    body.visual(
        Cylinder(radius=0.052, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=chrome,
        name="base_collar",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=chrome,
        name="lower_plinth",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.315),
        origin=Origin(xyz=(0.0, 0.0, 0.2175)),
        material=chrome,
        name="tall_column",
    )
    body.visual(
        Cylinder(radius=0.041, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.315), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="valve_body",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.055, 0.0, 0.315), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="side_boss",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=chrome,
        name="neck_collar",
    )

    spout_path = [
        (0.0, 0.000, 0.370),
        (0.0, -0.035, 0.438),
        (0.0, -0.105, 0.468),
        (0.0, -0.185, 0.438),
        (0.0, -0.225, 0.382),
    ]
    spout_mesh = tube_from_spline_points(
        spout_path,
        radius=0.014,
        samples_per_segment=14,
        radial_segments=28,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    body.visual(
        mesh_from_geometry(spout_mesh, "arched_spout_tube"),
        material=chrome,
        name="arched_spout",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.065),
        origin=Origin(xyz=(0.0, -0.225, 0.352)),
        material=chrome,
        name="down_nozzle",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.005),
        origin=Origin(xyz=(0.0, -0.225, 0.3175)),
        material=dark,
        name="aerator_face",
    )

    model.articulation(
        "basin_to_column_body",
        ArticulationType.FIXED,
        parent=basin,
        child=body,
        origin=Origin(xyz=(0.0, 0.145, 0.84)),
    )

    lever = model.part("side_lever")
    lever.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_hub",
    )
    lever.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_stem",
    )
    lever_paddle = ExtrudeGeometry(
        rounded_rect_profile(0.074, 0.025, 0.011, corner_segments=10),
        0.014,
        center=True,
    ).rotate_y(math.pi / 2.0)
    lever.visual(
        mesh_from_geometry(lever_paddle, "rounded_lever_paddle"),
        origin=Origin(xyz=(0.094, 0.0, 0.0)),
        material=chrome,
        name="paddle",
    )

    model.articulation(
        "column_body_to_side_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.077, 0.0, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-0.35, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    basin = object_model.get_part("basin")
    body = object_model.get_part("column_body")
    lever = object_model.get_part("side_lever")
    lever_joint = object_model.get_articulation("column_body_to_side_lever")

    ctx.expect_contact(
        body,
        basin,
        elem_a="base_collar",
        elem_b="basin_shell",
        contact_tol=0.002,
        name="chrome body is seated on basin deck",
    )
    ctx.expect_overlap(
        body,
        basin,
        axes="xy",
        elem_a="base_collar",
        elem_b="basin_shell",
        min_overlap=0.04,
        name="mounting collar footprint rests on ceramic",
    )
    ctx.expect_gap(
        lever,
        body,
        axis="x",
        positive_elem="pivot_hub",
        negative_elem="side_boss",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="lever hub meets side valve boss",
    )
    ctx.check(
        "lever joint uses a horizontal valve axis",
        abs(lever_joint.axis[0]) < 1e-6
        and abs(abs(lever_joint.axis[1]) - 1.0) < 1e-6
        and abs(lever_joint.axis[2]) < 1e-6,
        details=f"axis={lever_joint.axis}",
    )

    rest_aabb = ctx.part_element_world_aabb(lever, elem="paddle")
    with ctx.pose({lever_joint: 0.70}):
        raised_aabb = ctx.part_element_world_aabb(lever, elem="paddle")
    rest_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    raised_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0
    ctx.check(
        "positive lever rotation lifts the paddle",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.045,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
