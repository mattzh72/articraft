from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Return a coaxial tube centered on the local XY plane and optical Z axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def _bayonet_lug_origins(radius: float, z: float) -> list[Origin]:
    """Three evenly spaced tangential bayonet lug transforms around the Z axis."""
    origins: list[Origin] = []
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        origins.append(
            Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
                rpy=(0.0, 0.0, angle),
            )
        )
    return origins


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_x_teleconverter")

    matte_black = model.material("matte_black", rgba=(0.006, 0.006, 0.005, 1.0))
    satin_black = model.material("satin_black", rgba=(0.025, 0.025, 0.022, 1.0))
    gunmetal = model.material("dark_gunmetal", rgba=(0.18, 0.18, 0.17, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.66, 1.0))
    white = model.material("engraved_white", rgba=(0.95, 0.92, 0.84, 1.0))
    red = model.material("alignment_red", rgba=(0.85, 0.04, 0.02, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.22, 0.45, 0.70, 0.38))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_annular_cylinder(0.031, 0.0185, 0.055), "main_barrel"),
        material=matte_black,
        name="main_barrel",
    )
    body.visual(
        mesh_from_cadquery(_annular_cylinder(0.034, 0.0195, 0.006), "front_body_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=satin_black,
        name="front_body_shoulder",
    )
    body.visual(
        mesh_from_cadquery(_annular_cylinder(0.034, 0.0195, 0.006), "rear_body_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=satin_black,
        name="rear_body_shoulder",
    )
    body.visual(
        mesh_from_cadquery(_annular_cylinder(0.036, 0.019, 0.002), "front_retainer_rail"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=matte_black,
        name="front_retainer_rail",
    )
    body.visual(
        mesh_from_cadquery(_annular_cylinder(0.036, 0.019, 0.002), "rear_retainer_rail"),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=matte_black,
        name="rear_retainer_rail",
    )
    body.visual(
        Cylinder(radius=0.0185, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="recessed_optic",
    )

    front_ring = model.part("front_ring")
    front_ring.visual(
        mesh_from_cadquery(_annular_cylinder(0.039, 0.021, 0.014), "front_bayonet_ring"),
        material=chrome,
        name="front_bayonet_ring",
    )
    lug_shape = Box((0.007, 0.020, 0.0045))
    for index, origin in enumerate(_bayonet_lug_origins(0.0405, 0.0035)):
        front_ring.visual(
            lug_shape,
            origin=origin,
            material=chrome,
            name=f"front_lug_{index}",
        )
    front_ring.visual(
        Sphere(radius=0.0024),
        origin=Origin(xyz=(0.0, 0.0365, 0.0072)),
        material=red,
        name="front_index_dot",
    )

    rear_ring = model.part("rear_ring")
    rear_ring.visual(
        mesh_from_cadquery(_annular_cylinder(0.038, 0.021, 0.014), "rear_bayonet_ring"),
        material=chrome,
        name="rear_bayonet_ring",
    )
    for index, origin in enumerate(_bayonet_lug_origins(0.039, -0.0035)):
        rear_ring.visual(
            lug_shape,
            origin=origin,
            material=chrome,
            name=f"rear_lug_{index}",
        )
    rear_ring.visual(
        Box((0.016, 0.002, 0.003)),
        origin=Origin(xyz=(0.0, -0.039, -0.0072), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=white,
        name="rear_mount_mark",
    )

    peg_ring = model.part("peg_ring")
    peg_ring.visual(
        mesh_from_cadquery(_annular_cylinder(0.037, 0.0335, 0.010), "alignment_ring"),
        material=gunmetal,
        name="alignment_ring",
    )
    peg_ring.visual(
        Box((0.008, 0.006, 0.004)),
        origin=Origin(xyz=(0.0405, 0.0, 0.0)),
        material=red,
        name="alignment_peg",
    )
    peg_ring.visual(
        Box((0.0018, 0.012, 0.003)),
        origin=Origin(xyz=(0.0372, 0.0, 0.0032)),
        material=white,
        name="peg_index_tick",
    )

    model.articulation(
        "body_to_front_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "body_to_rear_ring",
        ArticulationType.FIXED,
        parent=body,
        child=rear_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
    )
    model.articulation(
        "body_to_peg_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=peg_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_ring = object_model.get_part("front_ring")
    rear_ring = object_model.get_part("rear_ring")
    peg_ring = object_model.get_part("peg_ring")
    front_joint = object_model.get_articulation("body_to_front_ring")
    rear_joint = object_model.get_articulation("body_to_rear_ring")
    peg_joint = object_model.get_articulation("body_to_peg_ring")

    ctx.check(
        "front bayonet ring is revolute",
        front_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={front_joint.articulation_type}",
    )
    ctx.check(
        "rear bayonet ring is fixed",
        rear_joint.articulation_type == ArticulationType.FIXED,
        details=f"type={rear_joint.articulation_type}",
    )
    ctx.check(
        "alignment peg ring is revolute",
        peg_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={peg_joint.articulation_type}",
    )

    ctx.expect_origin_distance(
        front_ring,
        body,
        axes="xy",
        max_dist=0.001,
        name="front bayonet ring is coaxial with body",
    )
    ctx.expect_origin_distance(
        rear_ring,
        body,
        axes="xy",
        max_dist=0.001,
        name="rear bayonet ring is coaxial with body",
    )
    ctx.expect_origin_distance(
        peg_ring,
        body,
        axes="xy",
        max_dist=0.001,
        name="alignment peg ring is coaxial with body",
    )
    ctx.expect_gap(
        front_ring,
        body,
        axis="z",
        positive_elem="front_bayonet_ring",
        negative_elem="front_body_shoulder",
        max_gap=0.001,
        max_penetration=0.0,
        name="front rotating ring seats on front shoulder",
    )
    ctx.expect_gap(
        body,
        rear_ring,
        axis="z",
        positive_elem="rear_body_shoulder",
        negative_elem="rear_bayonet_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear fixed ring seats on rear shoulder",
    )
    ctx.expect_gap(
        body,
        peg_ring,
        axis="z",
        positive_elem="front_retainer_rail",
        negative_elem="alignment_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="front rail captures alignment ring",
    )
    ctx.expect_gap(
        peg_ring,
        body,
        axis="z",
        positive_elem="alignment_ring",
        negative_elem="rear_retainer_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear rail captures alignment ring",
    )

    def _element_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    rest_front_dot = _element_center(front_ring, "front_index_dot")
    with ctx.pose({front_joint: 1.05}):
        turned_front_dot = _element_center(front_ring, "front_index_dot")
    ctx.check(
        "front ring index dot sweeps around optical axis",
        rest_front_dot is not None
        and turned_front_dot is not None
        and turned_front_dot[0] < rest_front_dot[0] - 0.025
        and turned_front_dot[1] < rest_front_dot[1] - 0.012,
        details=f"rest={rest_front_dot}, turned={turned_front_dot}",
    )

    rest_peg = _element_center(peg_ring, "alignment_peg")
    with ctx.pose({peg_joint: math.pi / 2.0}):
        turned_peg = _element_center(peg_ring, "alignment_peg")
    ctx.check(
        "alignment peg revolves around body",
        rest_peg is not None
        and turned_peg is not None
        and turned_peg[0] < rest_peg[0] - 0.025
        and turned_peg[1] > rest_peg[1] + 0.025,
        details=f"rest={rest_peg}, turned={turned_peg}",
    )

    return ctx.report()


object_model = build_object_model()
