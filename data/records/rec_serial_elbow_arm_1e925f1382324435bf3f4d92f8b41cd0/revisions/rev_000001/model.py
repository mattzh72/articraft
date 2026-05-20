from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _planar_link_mesh(
    *,
    length: float,
    width: float,
    thickness: float,
    hub_radius: float,
    hole_radius: float,
    end_hole: bool,
    name: str,
) -> object:
    """Rounded flat link with an annular shoulder hub and optional end bore."""
    body = cq.Workplane("XY").circle(hub_radius).extrude(thickness)
    body = body.union(
        cq.Workplane("XY").center(length, 0.0).circle(hub_radius).extrude(thickness)
    )
    body = body.union(
        cq.Workplane("XY").center(length / 2.0, 0.0).rect(length, width).extrude(thickness)
    )
    body = body.translate((0.0, 0.0, -thickness / 2.0))

    cutter_height = thickness + 0.012
    bore = (
        cq.Workplane("XY")
        .circle(hole_radius)
        .extrude(cutter_height)
        .translate((0.0, 0.0, -cutter_height / 2.0))
    )
    body = body.cut(bore)
    if end_hole:
        end_bore = (
            cq.Workplane("XY")
            .center(length, 0.0)
            .circle(hole_radius)
            .extrude(cutter_height)
            .translate((0.0, 0.0, -cutter_height / 2.0))
        )
        body = body.cut(end_bore)
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_arm")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    blue = model.material("proximal_blue", rgba=(0.08, 0.18, 0.34, 1.0))
    orange = model.material("distal_orange", rgba=(0.88, 0.40, 0.12, 1.0))
    black = model.material("black_fasteners", rgba=(0.02, 0.02, 0.018, 1.0))

    shoulder_z = 0.145
    proximal_length = 0.620
    proximal_thickness = 0.045
    distal_offset_z = 0.060
    distal_length = 0.380
    distal_thickness = 0.038

    bracket = model.part("root_bracket")
    bracket.visual(
        Box((0.300, 0.240, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    bracket.visual(
        Cylinder(radius=0.082, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_steel,
        name="pedestal",
    )
    bracket.visual(
        Cylinder(radius=0.055, length=0.0475),
        origin=Origin(xyz=(0.0, 0.0, 0.09875)),
        material=brushed,
        name="shoulder_collar",
    )
    bracket.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, shoulder_z + 0.010)),
        material=brushed,
        name="shoulder_pin",
    )
    bracket.visual(
        Cylinder(radius=0.051, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, shoulder_z + proximal_thickness / 2.0 + 0.005)),
        material=brushed,
        name="shoulder_cap",
    )
    for ix, x in enumerate((-0.105, 0.105)):
        for iy, y in enumerate((-0.075, 0.075)):
            bracket.visual(
                Cylinder(radius=0.011, length=0.006),
                origin=Origin(xyz=(x, y, 0.028)),
                material=black,
                name=f"base_bolt_{ix}_{iy}",
            )

    proximal = model.part("proximal_link")
    proximal.visual(
        _planar_link_mesh(
            length=proximal_length,
            width=0.112,
            thickness=proximal_thickness,
            hub_radius=0.074,
            hole_radius=0.033,
            end_hole=True,
            name="proximal_body",
        ),
        material=blue,
        name="proximal_body",
    )
    proximal.visual(
        Cylinder(radius=0.044, length=0.0185),
        origin=Origin(xyz=(proximal_length, 0.0, 0.03175)),
        material=brushed,
        name="elbow_spacer",
    )
    proximal.visual(
        Cylinder(radius=0.024, length=0.071),
        origin=Origin(xyz=(proximal_length, 0.0, 0.0555)),
        material=brushed,
        name="elbow_pin",
    )
    proximal.visual(
        Cylinder(radius=0.047, length=0.010),
        origin=Origin(xyz=(proximal_length, 0.0, distal_offset_z + distal_thickness / 2.0 + 0.005)),
        material=brushed,
        name="elbow_cap",
    )

    distal = model.part("distal_link")
    distal.visual(
        _planar_link_mesh(
            length=distal_length,
            width=0.082,
            thickness=distal_thickness,
            hub_radius=0.058,
            hole_radius=0.031,
            end_hole=False,
            name="distal_body",
        ),
        material=orange,
        name="distal_body",
    )
    distal.visual(
        Cylinder(radius=0.088, length=0.020),
        origin=Origin(xyz=(distal_length + 0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed,
        name="end_flange",
    )
    for iz, z in enumerate((-0.043, 0.043)):
        for iy, y in enumerate((-0.043, 0.043)):
            distal.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(
                    xyz=(distal_length + 0.022, y, z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=black,
                name=f"flange_bolt_{iy}_{iz}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        origin=Origin(xyz=(proximal_length, 0.0, distal_offset_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.8, lower=-2.20, upper=2.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("root_bracket")
    proximal = object_model.get_part("proximal_link")
    distal = object_model.get_part("distal_link")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.check(
        "two planar revolute joints",
        len(object_model.articulations) == 2
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tuple(shoulder.axis) == (0.0, 0.0, 1.0)
        and tuple(elbow.axis) == (0.0, 0.0, 1.0),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    ctx.expect_gap(
        proximal,
        bracket,
        axis="z",
        positive_elem="proximal_body",
        negative_elem="shoulder_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="proximal hub sits on shoulder collar",
    )
    ctx.expect_gap(
        distal,
        proximal,
        axis="z",
        positive_elem="distal_body",
        negative_elem="elbow_spacer",
        max_gap=0.001,
        max_penetration=0.0,
        name="distal hub sits on elbow spacer",
    )
    ctx.expect_overlap(
        proximal,
        bracket,
        axes="xy",
        elem_a="proximal_body",
        elem_b="shoulder_pin",
        min_overlap=0.040,
        name="shoulder pin passes through proximal hub footprint",
    )
    ctx.expect_overlap(
        distal,
        proximal,
        axes="xy",
        elem_a="distal_body",
        elem_b="elbow_pin",
        min_overlap=0.040,
        name="elbow pin passes through distal hub footprint",
    )

    prox_aabb = ctx.part_element_world_aabb(proximal, elem="proximal_body")
    dist_aabb = ctx.part_element_world_aabb(distal, elem="distal_body")
    prox_len = prox_aabb[1][0] - prox_aabb[0][0] if prox_aabb else 0.0
    dist_len = dist_aabb[1][0] - dist_aabb[0][0] if dist_aabb else 0.0
    ctx.check(
        "proximal link is visibly longer and stouter",
        prox_len > dist_len + 0.12,
        details=f"proximal_body_dx={prox_len:.3f}, distal_body_dx={dist_len:.3f}",
    )

    rest_distal = ctx.part_world_position(distal)
    with ctx.pose({shoulder: 0.75, elbow: -0.65}):
        posed_distal = ctx.part_world_position(distal)
    ctx.check(
        "joint motion stays in horizontal service-arm plane",
        rest_distal is not None
        and posed_distal is not None
        and abs(posed_distal[2] - rest_distal[2]) < 1e-6
        and abs(posed_distal[1] - rest_distal[1]) > 0.10,
        details=f"rest={rest_distal}, posed={posed_distal}",
    )

    return ctx.report()


object_model = build_object_model()
