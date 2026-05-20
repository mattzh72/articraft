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
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HANDLE_LENGTH = 0.170
HANDLE_THICKNESS = 0.024


def _handle_body_mesh() -> cq.Workplane:
    """Molded utility-knife handle with a real hollow blade channel."""
    # Side silhouette in the X/Y plane; thickness is along local Z.
    outline = [
        (-0.082, -0.010),
        (-0.070, -0.019),
        (0.052, -0.020),
        (0.085, -0.010),
        (0.092, 0.004),
        (0.070, 0.018),
        (-0.052, 0.020),
        (-0.080, 0.011),
    ]
    body = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(HANDLE_THICKNESS)
        .translate((0.0, 0.0, -HANDLE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )

    # Internal central tunnel for the blade and steel carrier.  It opens at the
    # front nose but does not remove the outer side walls.
    blade_channel = cq.Workplane("XY").box(0.155, 0.020, 0.009).translate(
        (0.026, -0.005, 0.0)
    )
    body = body.cut(blade_channel)

    # A long rounded side guide slot exposes the moving thumb carrier and lets
    # the stem reach the internal blade carriage.
    guide_slot = (
        cq.Workplane("XY")
        .center(-0.002, 0.009)
        .slot2D(0.112, 0.010, 0.0)
        .extrude(0.040)
        .translate((0.0, 0.0, -0.020))
    )
    body = body.cut(guide_slot)
    guide_clearance = cq.Workplane("XY").box(0.118, 0.0145, 0.040).translate(
        (-0.002, 0.009, 0.0)
    )
    body = body.cut(guide_clearance)

    # Shallow molded relief pockets make the handle read as an injection-molded
    # two-piece workshop tool rather than a plain block.
    for cx in (-0.058, 0.033):
        pocket = (
            cq.Workplane("XY")
            .center(cx, -0.011)
            .slot2D(0.038, 0.007, 0.0)
            .extrude(0.003)
            .translate((0.0, 0.0, HANDLE_THICKNESS / 2.0 - 0.001))
        )
        body = body.cut(pocket)

    return body


def _grip_inlay_mesh() -> cq.Workplane:
    """Black rubber overmold strips, slightly proud of the side face."""
    base = None
    for i, x in enumerate((-0.060, -0.046, -0.032, -0.018, -0.004)):
        rib = (
            cq.Workplane("XY")
            .center(x, -0.0115 + 0.001 * i)
            .slot2D(0.025, 0.0042, 0.0)
            .extrude(0.0020)
            .translate((0.0, 0.0, HANDLE_THICKNESS / 2.0 - 0.0005))
        )
        base = rib if base is None else base.union(rib)
    return base


def _slider_cap_mesh() -> cq.Workplane:
    """Thumb slider with raised anti-slip ridges."""
    cap = (
        cq.Workplane("XY")
        .box(0.031, 0.015, 0.006)
        .translate((0.0, 0.0, 0.003))
        .edges("|Z")
        .fillet(0.0022)
    )
    for x in (-0.010, -0.005, 0.0, 0.005, 0.010):
        ridge = cq.Workplane("XY").box(0.0022, 0.012, 0.0014).translate(
            (x, 0.0, 0.00655)
        )
        cap = cap.union(ridge)
    return cap


def _blade_mesh() -> cq.Workplane:
    """Trapezoidal snap-off utility blade, modeled as a thin steel plate."""
    blade_profile = [
        (0.050, -0.022),
        (0.118, -0.022),
        (0.145, -0.012),
        (0.132, -0.004),
        (0.050, -0.004),
    ]
    return (
        cq.Workplane("XY")
        .polyline(blade_profile)
        .close()
        .extrude(0.0018)
        .translate((0.0, 0.0, -0.0149))
        .edges("|Z")
        .chamfer(0.00045)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    molded_orange = model.material("molded_orange", rgba=(0.92, 0.32, 0.08, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.11, 0.12, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.45, 0.46, 0.45, 1.0))
    etched = model.material("etched_dark", rgba=(0.16, 0.16, 0.15, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_body_mesh(), "molded_handle_body", tolerance=0.0005),
        material=molded_orange,
        name="molded_shell",
    )
    handle.visual(
        mesh_from_cadquery(_grip_inlay_mesh(), "rubber_grip_inlays", tolerance=0.0005),
        material=black_rubber,
        name="grip_inlays",
    )

    # Proud molded guide rails surrounding the real slot in the side shell.
    for name, y in (("upper_track_rail", 0.0162), ("lower_track_rail", 0.0018)):
        handle.visual(
            Box((0.118, 0.0022, 0.0020)),
            origin=Origin(xyz=(-0.002, y, HANDLE_THICKNESS / 2.0 + 0.00035)),
            material=dark_graphite,
            name=name,
        )

    # Small detent windows along the guide track.
    for i, x in enumerate((-0.045, -0.026, -0.007, 0.012, 0.031, 0.050)):
        handle.visual(
            Box((0.006, 0.0012, 0.0010)),
            origin=Origin(xyz=(x, 0.0177, HANDLE_THICKNESS / 2.0 + 0.0010)),
            material=etched,
            name=f"detent_mark_{i}",
        )

    # Exposed fasteners on the side cheek.
    for i, (x, y) in enumerate(((-0.064, 0.008), (-0.020, -0.015), (0.055, 0.006))):
        handle.visual(
            Cylinder(radius=0.0042, length=0.0024),
            origin=Origin(xyz=(x, y, HANDLE_THICKNESS / 2.0 + 0.0004)),
            material=screw_steel,
            name=f"screw_head_{i}",
        )
        handle.visual(
            Box((0.0062, 0.0009, 0.00060)),
            origin=Origin(
                xyz=(x, y, HANDLE_THICKNESS / 2.0 + 0.00145),
                rpy=(0.0, 0.0, math.radians(12 + 23 * i)),
            ),
            material=etched,
            name=f"screw_slot_{i}",
        )

    # Dark metal front throat plates frame the blade outlet and make the nose
    # read as a reinforced cutter mouth.
    for y in (-0.0145, 0.0045):
        handle.visual(
            Box((0.022, 0.0022, 0.010)),
            origin=Origin(xyz=(0.082, y, 0.0)),
            material=dark_graphite,
            name=f"blade_throat_{'lower' if y < 0 else 'upper'}",
        )

    carrier = model.part("blade_carrier")
    carrier.visual(
        mesh_from_cadquery(_slider_cap_mesh(), "ridged_thumb_slider", tolerance=0.0004),
        material=dark_graphite,
        name="thumb_slider",
    )
    carrier.visual(
        Box((0.010, 0.010, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, -0.0068)),
        material=dark_graphite,
        name="slider_stem",
    )
    carrier.visual(
        Box((0.012, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, -0.014)),
        material=dark_graphite,
        name="carrier_neck",
    )
    carrier.visual(
        Box((0.085, 0.010, 0.004)),
        origin=Origin(xyz=(0.040, -0.015, -0.014)),
        material=brushed_steel,
        name="carrier_plate",
    )
    carrier.visual(
        mesh_from_cadquery(_blade_mesh(), "snap_off_blade", tolerance=0.00035),
        material=brushed_steel,
        name="blade",
    )
    for i, x in enumerate((0.086, 0.103, 0.120)):
        carrier.visual(
            Box((0.020, 0.0012, 0.00120)),
            origin=Origin(
                xyz=(x, -0.0145, -0.01435),
                rpy=(0.0, 0.0, math.radians(22.0)),
            ),
            material=etched,
            name=f"blade_score_{i}",
        )

    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(xyz=(-0.030, 0.009, HANDLE_THICKNESS / 2.0 + 0.0020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=-0.025, upper=0.025),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    slide = object_model.get_articulation("handle_to_carrier")

    ctx.expect_within(
        carrier,
        handle,
        axes="yz",
        inner_elem="slider_stem",
        outer_elem="molded_shell",
        margin=0.006,
        name="slider stem remains captured in the side guide slot",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="carrier_plate",
        elem_b="molded_shell",
        min_overlap=0.030,
        name="blade carrier stays engaged inside handle body",
    )

    rest_tip = ctx.part_element_world_aabb(carrier, elem="blade")
    rest_slider = ctx.part_world_position(carrier)
    with ctx.pose({slide: 0.025}):
        extended_tip = ctx.part_element_world_aabb(carrier, elem="blade")
        extended_slider = ctx.part_world_position(carrier)
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_plate",
            elem_b="molded_shell",
            min_overlap=0.025,
            name="extended carrier still has retained insertion",
        )
    with ctx.pose({slide: -0.025}):
        retracted_tip = ctx.part_element_world_aabb(carrier, elem="blade")

    ctx.check(
        "carrier slides forward along the handle",
        rest_slider is not None
        and extended_slider is not None
        and extended_slider[0] > rest_slider[0] + 0.020,
        details=f"rest={rest_slider}, extended={extended_slider}",
    )
    ctx.check(
        "blade tip retracts and extends through the nose",
        rest_tip is not None
        and retracted_tip is not None
        and extended_tip is not None
        and retracted_tip[1][0] < rest_tip[1][0] - 0.020
        and extended_tip[1][0] > rest_tip[1][0] + 0.020,
        details=f"retracted={retracted_tip}, rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
