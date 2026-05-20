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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    """Closed annular cylinder in local model coordinates."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    asphalt = model.material("dark_asphalt", rgba=(0.025, 0.026, 0.025, 1.0))
    rubber = model.material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    safety_yellow = model.material("reflective_yellow", rgba=(1.0, 0.78, 0.06, 1.0))
    key_dark = model.material("key_socket_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    sleeve = model.part("sleeve")
    pavement_cutout = (
        cq.Workplane("XY")
        .rect(0.92, 0.92)
        .circle(0.224)
        .extrude(0.055)
        .translate((0.0, 0.0, -0.055))
    )
    sleeve.visual(
        mesh_from_cadquery(pavement_cutout, "pavement_cutout", tolerance=0.001),
        material=asphalt,
        name="pavement_patch",
    )

    lower_sleeve = _annular_cylinder(0.150, 0.105, 1.08, -1.045)
    sleeve.visual(
        mesh_from_cadquery(lower_sleeve, "lower_sleeve", tolerance=0.0008),
        material=dark_steel,
        name="lower_sleeve",
    )

    collar_flange = _annular_cylinder(0.225, 0.130, 0.035, 0.0)
    sleeve.visual(
        mesh_from_cadquery(collar_flange, "collar_flange", tolerance=0.0008),
        material=brushed_steel,
        name="collar_flange",
    )

    wiper_ring = _annular_cylinder(0.128, 0.092, 0.012, 0.035)
    sleeve.visual(
        mesh_from_cadquery(wiper_ring, "rubber_wiper_ring", tolerance=0.0008),
        material=rubber,
        name="wiper_ring",
    )

    for idx in range(8):
        if idx == 6:
            continue
        angle = idx * math.tau / 8.0
        x = 0.188 * math.cos(angle)
        y = 0.188 * math.sin(angle)
        sleeve.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, y, 0.037)),
            material=dark_steel,
            name=f"bolt_{idx}",
        )

    sleeve.visual(
        Box((0.020, 0.030, 0.012)),
        origin=Origin(xyz=(0.095, 0.0, 0.041)),
        material=rubber,
        name="guide_shoe_px",
    )
    sleeve.visual(
        Box((0.020, 0.030, 0.012)),
        origin=Origin(xyz=(-0.095, 0.0, 0.041)),
        material=rubber,
        name="guide_shoe_nx",
    )
    sleeve.visual(
        Box((0.030, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.095, 0.041)),
        material=rubber,
        name="guide_shoe_py",
    )
    sleeve.visual(
        Box((0.030, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.095, 0.041)),
        material=rubber,
        name="guide_shoe_ny",
    )

    # Short exposed hinge brackets for the maintenance access flap.
    for side, x in enumerate((-0.054, 0.054)):
        sleeve.visual(
            Box((0.040, 0.026, 0.014)),
            origin=Origin(xyz=(x, -0.232, 0.028)),
            material=brushed_steel,
            name=f"hinge_leaf_{side}",
        )
        sleeve.visual(
            Cylinder(radius=0.0085, length=0.036),
            origin=Origin(xyz=(x, -0.222, 0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"hinge_knuckle_{side}",
        )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.085, length=1.250),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=brushed_steel,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=0.121, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.950)),
        material=brushed_steel,
        name="top_cap",
    )
    post.visual(
        Cylinder(radius=0.087, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material=safety_yellow,
        name="reflective_band_0",
    )
    post.visual(
        Cylinder(radius=0.087, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=safety_yellow,
        name="reflective_band_1",
    )

    key_cap = model.part("key_cap")
    key_cap.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_steel,
        name="key_disc",
    )
    key_cap.visual(
        Box((0.046, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=key_dark,
        name="key_slot",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.060, 0.085, 0.008)),
        origin=Origin(xyz=(0.0, 0.0425, -0.004)),
        material=brushed_steel,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.0080, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="flap_knuckle",
    )
    flap.visual(
        Box((0.040, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.068, 0.001)),
        material=dark_steel,
        name="finger_pull",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1400.0, velocity=0.18, lower=-0.860, upper=0.0),
    )
    model.articulation(
        "post_to_key_cap",
        ArticulationType.REVOLUTE,
        parent=post,
        child=key_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.977)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-math.pi / 2.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "sleeve_to_flap",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=flap,
        origin=Origin(xyz=(0.0, -0.222, 0.043)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    key_cap = object_model.get_part("key_cap")
    flap = object_model.get_part("flap")
    post_slide = object_model.get_articulation("sleeve_to_post")
    key_turn = object_model.get_articulation("post_to_key_cap")
    flap_hinge = object_model.get_articulation("sleeve_to_flap")

    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_tube",
        elem_b="lower_sleeve",
        min_overlap=0.20,
        name="raised post remains inserted in sleeve",
    )
    ctx.expect_contact(
        key_cap,
        post,
        elem_a="key_disc",
        elem_b="top_cap",
        contact_tol=0.001,
        name="rotating key cap sits on top cap",
    )
    ctx.expect_contact(
        flap,
        sleeve,
        elem_a="flap_panel",
        elem_b="collar_flange",
        contact_tol=0.001,
        name="maintenance flap rests on collar",
    )

    raised_position = ctx.part_world_position(post)
    with ctx.pose({post_slide: -0.60}):
        lowered_position = ctx.part_world_position(post)
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_tube",
            elem_b="lower_sleeve",
            min_overlap=0.35,
            name="lowered post stays captured by sleeve",
        )
    ctx.check(
        "post translates downward inside sleeve",
        raised_position is not None
        and lowered_position is not None
        and lowered_position[2] < raised_position[2] - 0.55,
        details=f"raised={raised_position}, lowered={lowered_position}",
    )

    closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 1.20}):
        opened_aabb = ctx.part_world_aabb(flap)
    closed_top = closed_aabb[1][2] if closed_aabb is not None else None
    opened_top = opened_aabb[1][2] if opened_aabb is not None else None
    ctx.check(
        "maintenance flap opens upward",
        closed_top is not None and opened_top is not None and opened_top > closed_top + 0.055,
        details=f"closed_top={closed_top}, opened_top={opened_top}",
    )

    slot_closed = ctx.part_element_world_aabb(key_cap, elem="key_slot")
    with ctx.pose({key_turn: math.pi / 2.0}):
        slot_rotated = ctx.part_element_world_aabb(key_cap, elem="key_slot")
    if slot_closed is not None and slot_rotated is not None:
        closed_dx = slot_closed[1][0] - slot_closed[0][0]
        rotated_dy = slot_rotated[1][1] - slot_rotated[0][1]
        slot_ok = rotated_dy > closed_dx * 0.85
        slot_details = f"closed_dx={closed_dx}, rotated_dy={rotated_dy}"
    else:
        slot_ok = False
        slot_details = f"closed={slot_closed}, rotated={slot_rotated}"
    ctx.check("key socket visibly rotates with cap", slot_ok, details=slot_details)

    return ctx.report()


object_model = build_object_model()
