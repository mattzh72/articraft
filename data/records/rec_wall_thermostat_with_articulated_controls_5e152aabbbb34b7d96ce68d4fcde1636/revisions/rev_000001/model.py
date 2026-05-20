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


def _rounded_box(width: float, height: float, depth: float, corner_radius: float) -> cq.Workplane:
    """A centered rounded-rectangle slab with softened vertical corners."""
    return (
        cq.Workplane("XY")
        .box(width, height, depth)
        .edges("|Z")
        .fillet(corner_radius)
    )


def _annular_prism(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    edge_radius: float = 0.0,
) -> cq.Workplane:
    """An annular cylinder extruded from local z=0 to z=height."""
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )
    if edge_radius > 0.0:
        ring = ring.edges(">Z").fillet(edge_radius).edges("<Z").fillet(edge_radius)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wall_thermostat")

    painted_metal = model.material("warm_painted_metal", rgba=(0.82, 0.80, 0.75, 1.0))
    satin_metal = model.material("satin_anodized_metal", rgba=(0.58, 0.56, 0.52, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    polymer = model.material("warm_white_polymer", rgba=(0.94, 0.93, 0.89, 1.0))
    elastomer = model.material("charcoal_elastomer", rgba=(0.055, 0.058, 0.060, 1.0))
    display_glass = model.material("smoked_glass", rgba=(0.05, 0.075, 0.085, 0.68))
    display_ink = model.material("soft_display_ink", rgba=(0.60, 0.88, 0.86, 1.0))
    dark_tick = model.material("engraved_dark_tick", rgba=(0.18, 0.18, 0.17, 1.0))

    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_rounded_box(0.136, 0.136, 0.006, 0.018), "wall_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=polymer,
        name="wall_plate",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box(0.124, 0.124, 0.003, 0.016), "shadow_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=elastomer,
        name="shadow_gasket",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box(0.118, 0.118, 0.017, 0.014), "front_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0140)),
        material=painted_metal,
        name="front_shell",
    )

    body.visual(
        mesh_from_cadquery(
            _annular_prism(0.0455, 0.0410, 0.0009, edge_radius=0.00015),
            "dial_shadow_seam",
            tolerance=0.00035,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.02225)),
        material=elastomer,
        name="dial_shadow_seam",
    )

    # Small, restrained printed tick marks sit outside the rotating dial.
    for index, angle in enumerate([-60, -40, -20, 0, 20, 40, 60]):
        theta = math.radians(angle)
        radius = 0.052
        tick_length = 0.0065 if angle == 0 else 0.0047
        tick_width = 0.0012 if angle == 0 else 0.0009
        body.visual(
            Box((tick_width, tick_length, 0.0007)),
            origin=Origin(
                xyz=(math.sin(theta) * radius, math.cos(theta) * radius, 0.02285),
                rpy=(0.0, 0.0, -theta),
            ),
            material=dark_tick,
            name=f"temperature_tick_{index}",
        )

    body.visual(
        mesh_from_cadquery(_rounded_box(0.052, 0.012, 0.0012, 0.003), "display_window"),
        origin=Origin(xyz=(0.0, 0.052, 0.02305)),
        material=display_glass,
        name="display_window",
    )
    for index, (x, y, sx, sy) in enumerate(
        [
            (-0.010, 0.052, 0.006, 0.0010),
            (-0.010, 0.055, 0.006, 0.0010),
            (-0.010, 0.049, 0.006, 0.0010),
            (-0.0068, 0.0505, 0.0010, 0.0040),
            (-0.0132, 0.0535, 0.0010, 0.0040),
            (0.0025, 0.052, 0.006, 0.0010),
            (0.0025, 0.049, 0.006, 0.0010),
            (0.0058, 0.0535, 0.0010, 0.0040),
            (0.0145, 0.0555, 0.0015, 0.0015),
        ]
    ):
        body.visual(
            Box((sx, sy, 0.00035)),
            origin=Origin(xyz=(x, y, 0.02375)),
            material=display_ink,
            name=f"display_segment_{index}",
        )

    body.visual(
        Cylinder(radius=0.0105, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0231)),
        material=brushed_steel,
        name="rear_bearing_shoulder",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.0149),
        origin=Origin(xyz=(0.0, 0.0, 0.02995)),
        material=brushed_steel,
        name="shaft",
    )
    body.visual(
        Cylinder(radius=0.0125, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0, 0.0363)),
        material=brushed_steel,
        name="retainer_cap",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(
            _annular_prism(0.0400, 0.0075, 0.0100, edge_radius=0.0008),
            "dial_shell",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        origin=Origin(),
        material=satin_metal,
        name="dial_shell",
    )
    dial.visual(
        mesh_from_cadquery(
            _annular_prism(0.0415, 0.0386, 0.0062, edge_radius=0.00025),
            "grip_band",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
        material=elastomer,
        name="grip_band",
    )
    dial.visual(
        Box((0.0022, 0.0140, 0.00055)),
        origin=Origin(xyz=(0.0, 0.0295, 0.01025)),
        material=polymer,
        name="indicator_mark",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.0237)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=4.0, lower=-2.45, upper=2.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_origin_distance(
        body,
        dial,
        axes="xy",
        max_dist=0.000001,
        name="dial axis is centered on thermostat body",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.00015,
        positive_elem="dial_shell",
        negative_elem="rear_bearing_shoulder",
        name="dial rides on rear bearing shoulder",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="z",
        min_gap=0.0003,
        max_gap=0.0025,
        positive_elem="retainer_cap",
        negative_elem="dial_shell",
        name="retainer cap captures dial with front clearance",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        min_overlap=0.008,
        elem_a="dial_shell",
        elem_b="shaft",
        name="shaft passes through dial centerline",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_indicator = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator_mark"))
    with ctx.pose({dial_joint: 1.0}):
        rotated_indicator = _aabb_center(
            ctx.part_element_world_aabb(dial, elem="indicator_mark")
        )
        ctx.expect_gap(
            body,
            dial,
            axis="z",
            min_gap=0.0003,
            max_gap=0.0025,
            positive_elem="retainer_cap",
            negative_elem="dial_shell",
            name="retainer clearance persists while dial rotates",
        )
    ctx.check(
        "dial indicator sweeps around the shaft",
        rest_indicator is not None
        and rotated_indicator is not None
        and rotated_indicator[0] < rest_indicator[0] - 0.020
        and rotated_indicator[1] < rest_indicator[1] - 0.008,
        details=f"rest={rest_indicator}, rotated={rotated_indicator}",
    )

    return ctx.report()


object_model = build_object_model()
