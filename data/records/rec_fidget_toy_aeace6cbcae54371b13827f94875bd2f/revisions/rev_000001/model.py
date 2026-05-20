from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_THICKNESS = 0.008


def _make_wing_shell() -> cq.Workplane:
    """Swept two-lobe fidget spinner body, centered on the bearing axis."""

    # A single swept-back lobe made as a hull of offset circular sections.  The
    # two lobes and the center hub are fused in a 2-D sketch before extrusion so
    # the exported spinner shell is one continuous mesh, not three touching
    # islands.
    right_sketch = (
        cq.Sketch()
        .arc((0.016, 0.002), 0.016, 0.0, 360.0)
        .arc((0.032, -0.0055), 0.016, 0.0, 360.0)
        .arc((0.048, -0.008), 0.011, 0.0, 360.0)
        .hull()
    )
    left_sketch = (
        cq.Sketch()
        .arc((-0.016, -0.002), 0.016, 0.0, 360.0)
        .arc((-0.032, 0.0055), 0.016, 0.0, 360.0)
        .arc((-0.048, 0.008), 0.011, 0.0, 360.0)
        .hull()
    )
    body_sketch = (
        cq.Sketch()
        .circle(0.022)
        .reset()
        .face(right_sketch, mode="a")
        .reset()
        .face(left_sketch, mode="a")
        .reset()
        .circle(0.0105, mode="s")
        .clean()
    )

    return (
        cq.Workplane("XY")
        .placeSketch(body_sketch)
        .extrude(BODY_THICKNESS)
        .translate((0.0, 0.0, -BODY_THICKNESS / 2.0))
    )


def _make_pocket_rim() -> cq.Workplane:
    """Raised annular counterbore rim that makes the weight disk read recessed."""

    return cq.Workplane("XY").circle(0.0105).circle(0.0080).extrude(0.0025)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bi_wing_fidget_spinner")

    plastic = Material("satin_blue_plastic", rgba=(0.05, 0.28, 0.82, 1.0))
    dark_rubber = Material("dark_bearing_rubber", rgba=(0.015, 0.017, 0.020, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    brass = Material("brass_weights", rgba=(0.92, 0.66, 0.24, 1.0))

    center_hub = model.part("center_hub")
    center_hub.visual(
        Cylinder(radius=0.0175, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.00625)),
        material=brushed_steel,
        name="top_cap",
    )
    center_hub.visual(
        Cylinder(radius=0.0175, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, -0.00625)),
        material=brushed_steel,
        name="bottom_cap",
    )
    center_hub.visual(
        Cylinder(radius=0.0105, length=0.0100),
        material=dark_rubber,
        name="inner_race",
    )

    wing_body = model.part("wing_body")
    wing_body.visual(
        mesh_from_cadquery(
            _make_wing_shell(),
            "wing_shell",
            tolerance=0.00045,
            angular_tolerance=0.05,
        ),
        material=plastic,
        name="wing_shell",
    )
    # Small molded root blends overlap the hub disk and lobe roots, welding the
    # swept lobes into one supported plastic body rather than merely touching at
    # a mathematical seam.
    for name, x, y in (
        ("front_root_blend", 0.024, -0.006),
        ("rear_root_blend", -0.024, 0.006),
    ):
        wing_body.visual(
            Box((0.016, 0.028, BODY_THICKNESS)),
            origin=Origin(xyz=(x, y, 0.0), rpy=(0.0, 0.0, -0.22)),
            material=plastic,
            name=name,
        )

    spin_joint = model.articulation(
        "hub_to_wing",
        ArticulationType.CONTINUOUS,
        parent=center_hub,
        child=wing_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=80.0),
        motion_properties=MotionProperties(damping=0.002, friction=0.0005),
    )
    spin_joint.meta["description"] = "Free-spinning bi-wing assembly about the central bearing axis."

    for index, (x, y) in enumerate(((0.0405, -0.0055), (-0.0405, 0.0055))):
        weight = model.part(f"weight_{index}")
        weight.visual(
            mesh_from_cadquery(
                _make_pocket_rim(),
                f"pocket_rim_{index}",
                tolerance=0.00035,
                angular_tolerance=0.04,
            ),
            material=dark_rubber,
            name="pocket_rim",
        )
        weight.visual(
            Cylinder(radius=0.0082, length=0.0018),
            origin=Origin(xyz=(0.0, 0.0, 0.0009)),
            material=brass,
            name="weight_disk",
        )
        model.articulation(
            f"wing_to_weight_{index}",
            ArticulationType.FIXED,
            parent=wing_body,
            child=weight,
            origin=Origin(xyz=(x, y, BODY_THICKNESS / 2.0 - 0.00075)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("center_hub")
    wing = object_model.get_part("wing_body")
    weight_0 = object_model.get_part("weight_0")
    weight_1 = object_model.get_part("weight_1")
    spin = object_model.get_articulation("hub_to_wing")

    ctx.check(
        "continuous central spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 4) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.allow_overlap(
        hub,
        wing,
        elem_a="inner_race",
        elem_b="wing_shell",
        reason="The stationary bearing race is intentionally represented as a captured cylindrical proxy inside the rotating central bore.",
    )
    ctx.expect_overlap(
        hub,
        wing,
        axes="z",
        elem_a="inner_race",
        elem_b="wing_shell",
        min_overlap=0.0075,
        name="inner bearing race spans the spinner body thickness",
    )

    for index, weight in enumerate((weight_0, weight_1)):
        ctx.allow_overlap(
            weight,
            wing,
            elem_a="pocket_rim",
            elem_b="wing_shell",
            reason="The annular weight-pocket rim is slightly seated into the plastic wing surface to read as a recessed insert.",
        )
        ctx.expect_gap(
            weight,
            wing,
            axis="z",
            positive_elem="pocket_rim",
            negative_elem="wing_shell",
            max_penetration=0.0010,
            max_gap=0.0002,
            name=f"weight pocket {index} rim is seated into the wing skin",
        )

    ctx.expect_gap(
        hub,
        wing,
        axis="z",
        positive_elem="top_cap",
        negative_elem="wing_shell",
        min_gap=0.0005,
        max_gap=0.0016,
        name="top cap floats just above spinner body",
    )
    ctx.expect_gap(
        wing,
        hub,
        axis="z",
        positive_elem="wing_shell",
        negative_elem="bottom_cap",
        min_gap=0.0005,
        max_gap=0.0016,
        name="bottom cap clears underside of spinner body",
    )
    ctx.expect_overlap(
        wing,
        hub,
        axes="xy",
        elem_a="wing_shell",
        elem_b="top_cap",
        min_overlap=0.030,
        name="cap plates cover the central bearing hub",
    )

    ctx.expect_overlap(
        weight_0,
        wing,
        axes="xy",
        elem_a="pocket_rim",
        elem_b="wing_shell",
        min_overlap=0.015,
        name="first recessed weight pocket sits on the wing lobe",
    )
    ctx.expect_overlap(
        weight_1,
        wing,
        axes="xy",
        elem_a="pocket_rim",
        elem_b="wing_shell",
        min_overlap=0.015,
        name="second recessed weight pocket sits on the wing lobe",
    )
    ctx.expect_origin_distance(
        weight_0,
        weight_1,
        axes="xy",
        min_dist=0.078,
        max_dist=0.087,
        name="opposing weights are palm-scale and symmetric",
    )

    rest_weight = ctx.part_world_position(weight_0)
    with ctx.pose({spin: math.pi}):
        turned_weight = ctx.part_world_position(weight_0)
    ctx.check(
        "wing assembly rotates freely around hub",
        rest_weight is not None
        and turned_weight is not None
        and turned_weight[0] < -0.035
        and turned_weight[1] > 0.002,
        details=f"rest={rest_weight}, turned={turned_weight}",
    )

    return ctx.report()


object_model = build_object_model()
