from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _scalloped_annular_ring(
    *,
    inner_layers: tuple[tuple[float, float], ...],
    outer_layers: tuple[tuple[float, float], ...],
    radial_segments: int = 128,
    grip_count: int = 48,
    grip_depth: float = 0.00065,
) -> MeshGeometry:
    """Connected annular mesh with a shallow ridged outside for finger grip."""

    geom = MeshGeometry()
    inner_indices: list[list[int]] = []
    outer_indices: list[list[int]] = []

    for r, z in inner_layers:
        loop = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            loop.append(geom.add_vertex(r * math.cos(theta), r * math.sin(theta), z))
        inner_indices.append(loop)

    for r, z in outer_layers:
        loop = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            scallop = grip_depth * math.cos(grip_count * theta)
            rr = r + scallop
            loop.append(geom.add_vertex(rr * math.cos(theta), rr * math.sin(theta), z))
        outer_indices.append(loop)

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    layer_count = len(inner_layers)
    for layer in range(layer_count - 1):
        for i in range(radial_segments):
            j = (i + 1) % radial_segments
            # Inner bore surface.
            add_quad(
                inner_indices[layer][j],
                inner_indices[layer][i],
                inner_indices[layer + 1][i],
                inner_indices[layer + 1][j],
            )
            # Outer gripped surface.
            add_quad(
                outer_indices[layer][i],
                outer_indices[layer][j],
                outer_indices[layer + 1][j],
                outer_indices[layer + 1][i],
            )

    back = 0
    front = layer_count - 1
    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        # Rear annular face.
        add_quad(
            inner_indices[back][i],
            inner_indices[back][j],
            outer_indices[back][j],
            outer_indices[back][i],
        )
        # Front annular face.
        add_quad(
            inner_indices[front][j],
            inner_indices[front][i],
            outer_indices[front][i],
            outer_indices[front][j],
        )

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_smart_wall_thermostat")

    model.material("warm_white_polymer", rgba=(0.86, 0.88, 0.86, 1.0))
    model.material("satin_aluminum", rgba=(0.72, 0.73, 0.71, 1.0))
    model.material("black_glass", rgba=(0.015, 0.018, 0.022, 1.0))
    model.material("soft_display_glow", rgba=(0.25, 0.78, 1.0, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.049, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material="warm_white_polymer",
        name="rear_plate",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="warm_white_polymer",
        name="inner_housing",
    )
    body.visual(
        mesh_from_geometry(
            _scalloped_annular_ring(
                inner_layers=((0.0305, 0.023), (0.0305, 0.029)),
                outer_layers=((0.0320, 0.023), (0.0320, 0.029)),
                radial_segments=96,
                grip_count=1,
                grip_depth=0.0,
            ),
            "button_guide_rim",
        ),
        material="warm_white_polymer",
        name="guide_rim",
    )

    ring = model.part("outer_ring")
    ring_mesh = _scalloped_annular_ring(
        inner_layers=((0.0370, 0.007), (0.0352, 0.012), (0.0352, 0.033), (0.0370, 0.036)),
        outer_layers=((0.0470, 0.007), (0.0500, 0.012), (0.0500, 0.033), (0.0475, 0.036)),
    )
    ring.visual(
        mesh_from_geometry(ring_mesh, "rotary_outer_ring"),
        material="satin_aluminum",
        name="gripped_ring",
    )

    center = model.part("center_button")
    button_profile = [
        (0.0, -0.0010),
        (0.0305, -0.0010),
        (0.0305, 0.0022),
        (0.0293, 0.0030),
        (0.0260, 0.0045),
        (0.0, 0.0049),
    ]
    center.visual(
        mesh_from_geometry(LatheGeometry(button_profile, segments=96, closed=True), "center_push_cap"),
        material="black_glass",
        name="push_surface",
    )
    center.visual(
        Cylinder(radius=0.010, length=0.00035),
        origin=Origin(xyz=(0.0, 0.0, 0.0049)),
        material="soft_display_glow",
        name="display_dot",
    )

    model.articulation(
        "body_to_outer_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=6.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    model.articulation(
        "body_to_center_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=center,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.004),
        motion_properties=MotionProperties(damping=0.15, friction=0.04),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    ring = object_model.get_part("outer_ring")
    button = object_model.get_part("center_button")
    ring_joint = object_model.get_articulation("body_to_outer_ring")
    push_joint = object_model.get_articulation("body_to_center_button")

    ctx.check(
        "outer ring is continuous rotary control",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS and ring_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={ring_joint.articulation_type}, axis={ring_joint.axis}",
    )
    ctx.check(
        "center surface is short inward push",
        push_joint.articulation_type == ArticulationType.PRISMATIC
        and push_joint.axis == (0.0, 0.0, -1.0)
        and push_joint.motion_limits is not None
        and 0.003 <= (push_joint.motion_limits.upper or 0.0) <= 0.006,
        details=f"type={push_joint.articulation_type}, axis={push_joint.axis}, limits={push_joint.motion_limits}",
    )

    ctx.expect_origin_distance(ring, body, axes="xy", max_dist=0.0005, name="ring rotates on central axis")
    ctx.expect_origin_distance(button, body, axes="xy", max_dist=0.0005, name="push surface is centered")
    ctx.expect_contact(
        button,
        body,
        elem_a="push_surface",
        elem_b="guide_rim",
        contact_tol=0.00025,
        name="button rides in the front guide pocket",
    )

    rest_z = ctx.part_world_position(button)[2]
    with ctx.pose({push_joint: push_joint.motion_limits.upper}):
        depressed_z = ctx.part_world_position(button)[2]
    ctx.check(
        "pressing moves the center surface inward",
        depressed_z < rest_z - 0.003,
        details=f"rest_z={rest_z}, depressed_z={depressed_z}",
    )

    return ctx.report()


object_model = build_object_model()
