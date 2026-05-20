from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _fuselage_shape() -> cq.Workplane:
    """Lofted balsa-style fuselage in the body link frame.

    The stand pitch pivot is the body frame origin.  The fuselage rides above
    that pivot, with +X toward the propeller and -X toward the tail.
    """

    return (
        cq.Workplane("YZ", origin=(-0.42, 0.0, 0.125))
        .ellipse(0.010, 0.010)
        .workplane(offset=0.08)
        .ellipse(0.035, 0.026)
        .workplane(offset=0.16)
        .ellipse(0.056, 0.047)
        .workplane(offset=0.24)
        .ellipse(0.060, 0.052)
        .workplane(offset=0.22)
        .ellipse(0.043, 0.042)
        .workplane(offset=0.13)
        .ellipse(0.016, 0.018)
        .loft(combine=True)
    )


def _add_bolt(part, name: str, x: float, y: float, z: float, material: Material) -> None:
    """Small round head mounted on a side-facing bracket."""

    part.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_model_plane")

    cream = model.material("aged_cream", rgba=(0.86, 0.78, 0.60, 1.0))
    balsa = model.material("sealed_balsa", rgba=(0.72, 0.52, 0.30, 1.0))
    red = model.material("oxide_red_trim", rgba=(0.63, 0.06, 0.04, 1.0))
    dark = model.material("dark_service_panel", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    rubber = model.material("black_rubber_foot", rgba=(0.02, 0.02, 0.018, 1.0))

    # Root: a compact workbench/display stand with a yoke around the pitch pin.
    stand = model.part("stand")
    stand.visual(
        Box((0.36, 0.22, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark,
        name="base_plate",
    )
    stand.visual(
        Box((0.29, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=steel,
        name="weighted_insert",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.306)),
        material=steel,
        name="upright_post",
    )
    stand.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=steel,
        name="post_collar",
    )
    stand.visual(
        Box((0.12, 0.15, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        material=steel,
        name="yoke_bridge",
    )
    stand.visual(
        Box((0.030, 0.11, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.484), rpy=(0.0, 0.48, 0.0)),
        material=steel,
        name="front_web",
    )
    stand.visual(
        Box((0.030, 0.11, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.484), rpy=(0.0, -0.48, 0.0)),
        material=steel,
        name="rear_web",
    )
    # Four bars on each cheek leave a square pin opening while staying tied to
    # the bridge; the open construction reads as a legacy clevis, not a block.
    for side_index, y in enumerate((-0.062, 0.062)):
        stand.visual(
            Box((0.102, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.607)),
            material=steel,
            name=f"yoke_lower_{side_index}",
        )
        stand.visual(
            Box((0.102, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.653)),
            material=steel,
            name=f"yoke_upper_{side_index}",
        )
        stand.visual(
            Box((0.027, 0.014, 0.072)),
            origin=Origin(xyz=(-0.046, y, 0.630)),
            material=steel,
            name=f"yoke_rear_{side_index}",
        )
        stand.visual(
            Box((0.027, 0.014, 0.072)),
            origin=Origin(xyz=(0.046, y, 0.630)),
            material=steel,
            name=f"yoke_front_{side_index}",
        )
        _add_bolt(stand, f"yoke_bolt_top_{side_index}", -0.030, y * 1.08, 0.653, steel)
        _add_bolt(stand, f"yoke_bolt_low_{side_index}", 0.030, y * 1.08, 0.607, steel)
    for x, y in ((-0.14, -0.08), (-0.14, 0.08), (0.14, -0.08), (0.14, 0.08)):
        stand.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.005)),
            material=rubber,
            name=f"rubber_foot_{x}_{y}",
        )

    # Body link: one old-school model airplane assembly carried by the stand.
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_fuselage_shape(), "lofted_fuselage", tolerance=0.001),
        material=cream,
        name="fuselage",
    )
    body.visual(
        Box((0.275, 0.88, 0.022)),
        origin=Origin(xyz=(0.015, 0.0, 0.122)),
        material=cream,
        name="main_wing",
    )
    body.visual(
        Box((0.030, 0.90, 0.010)),
        origin=Origin(xyz=(-0.105, 0.0, 0.137)),
        material=balsa,
        name="rear_spar_cap",
    )
    body.visual(
        Box((0.034, 0.90, 0.010)),
        origin=Origin(xyz=(0.125, 0.0, 0.137)),
        material=balsa,
        name="front_spar_cap",
    )
    for i, y in enumerate((-0.36, -0.24, -0.12, 0.12, 0.24, 0.36)):
        body.visual(
            Box((0.250, 0.010, 0.008)),
            origin=Origin(xyz=(0.010, y, 0.140)),
            material=balsa,
            name=f"wing_rib_{i}",
        )
    body.visual(
        Box((0.110, 0.145, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.101)),
        material=steel,
        name="wing_root_plate",
    )
    body.visual(
        Box((0.135, 0.36, 0.012)),
        origin=Origin(xyz=(-0.365, 0.0, 0.142)),
        material=cream,
        name="tailplane",
    )
    body.visual(
        Box((0.118, 0.014, 0.145)),
        origin=Origin(xyz=(-0.390, 0.0, 0.212)),
        material=cream,
        name="fin",
    )
    body.visual(
        Box((0.026, 0.052, 0.110)),
        origin=Origin(xyz=(-0.345, 0.0, 0.166), rpy=(0.0, -0.55, 0.0)),
        material=balsa,
        name="tail_gusset",
    )
    body.visual(
        Box((0.140, 0.062, 0.004)),
        origin=Origin(xyz=(0.085, 0.0, 0.174)),
        material=dark,
        name="top_hatch",
    )
    body.visual(
        Box((0.105, 0.004, 0.052)),
        origin=Origin(xyz=(-0.155, -0.055, 0.130)),
        material=dark,
        name="side_hatch",
    )
    for i, x in enumerate((0.030, 0.140)):
        for j, y in enumerate((-0.024, 0.024)):
            body.visual(
                Cylinder(radius=0.004, length=0.004),
                origin=Origin(xyz=(x, y, 0.177)),
                material=steel,
                name=f"hatch_screw_{i}_{j}",
            )
    # Belly adapter and pivot hardware visibly tie the model to the stand yoke.
    body.visual(
        Box((0.084, 0.044, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="pivot_lug",
    )
    body.visual(
        Box((0.065, 0.038, 0.064)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=steel,
        name="belly_adapter",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.152),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.180),
        origin=Origin(xyz=(0.490, 0.0, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="nose_cowl",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.105),
        origin=Origin(xyz=(0.588, 0.0, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="nose_bearing",
    )
    body.visual(
        Box((0.055, 0.070, 0.010)),
        origin=Origin(xyz=(0.540, 0.0, 0.161)),
        material=red,
        name="nose_service_band",
    )
    for i, y in enumerate((-0.024, 0.024)):
        for j, z in enumerate((0.106, 0.144)):
            body.visual(
                Cylinder(radius=0.0038, length=0.005),
                origin=Origin(xyz=(0.540, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"nose_adapter_bolt_{i}_{j}",
            )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="prop_hub",
    )
    propeller.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        material=red,
        name="spinner_cap",
    )
    propeller.visual(
        Box((0.016, 0.178, 0.034)),
        origin=Origin(xyz=(0.034, 0.100, 0.0), rpy=(0.0, 0.0, 0.10)),
        material=balsa,
        name="blade_0",
    )
    propeller.visual(
        Box((0.016, 0.178, 0.034)),
        origin=Origin(xyz=(0.034, -0.100, 0.0), rpy=(0.0, 0.0, 0.10)),
        material=balsa,
        name="blade_1",
    )
    propeller.visual(
        Box((0.018, 0.073, 0.041)),
        origin=Origin(xyz=(0.031, 0.0, 0.0)),
        material=steel,
        name="blade_clamp",
    )

    model.articulation(
        "stand_to_body",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.7, lower=-0.18, upper=0.28),
        motion_properties=MotionProperties(damping=0.12, friction=0.08),
    )
    model.articulation(
        "body_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=propeller,
        origin=Origin(xyz=(0.6405, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.005),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    body = object_model.get_part("body")
    propeller = object_model.get_part("propeller")
    stand_joint = object_model.get_articulation("stand_to_body")
    prop_joint = object_model.get_articulation("body_to_propeller")

    ctx.expect_gap(
        propeller,
        body,
        axis="x",
        positive_elem="prop_hub",
        negative_elem="nose_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="propeller hub seats on nose bearing",
    )
    ctx.expect_within(
        body,
        stand,
        axes="y",
        inner_elem="pivot_lug",
        outer_elem="yoke_bridge",
        margin=0.0,
        name="pivot lug remains between yoke cheeks",
    )
    ctx.expect_overlap(
        body,
        stand,
        axes="y",
        elem_a="pivot_pin",
        elem_b="yoke_bridge",
        min_overlap=0.010,
        name="pivot pin spans the yoke bridge width",
    )

    closed_prop_aabb = ctx.part_world_aabb(propeller)
    with ctx.pose({prop_joint: math.pi / 2.0}):
        turned_prop_aabb = ctx.part_world_aabb(propeller)
    ctx.check(
        "propeller rotates about nose shaft",
        closed_prop_aabb is not None
        and turned_prop_aabb is not None
        and (turned_prop_aabb[1][2] - turned_prop_aabb[0][2])
        > (closed_prop_aabb[1][2] - closed_prop_aabb[0][2]) + 0.05,
        details=f"rest={closed_prop_aabb}, turned={turned_prop_aabb}",
    )

    level_aabb = ctx.part_world_aabb(body)
    with ctx.pose({stand_joint: 0.28}):
        pitched_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "stand joint pitches the model upward",
        level_aabb is not None
        and pitched_aabb is not None
        and pitched_aabb[1][2] > level_aabb[1][2] + 0.03,
        details=f"level={level_aabb}, pitched={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
