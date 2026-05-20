from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _turntable_body() -> cq.Workplane:
    """Machined rotary table with a raised rim, center boss, and index holes."""
    table = cq.Workplane("XY").circle(0.140).extrude(0.026)
    rim = cq.Workplane("XY").circle(0.140).circle(0.124).extrude(0.009).translate((0, 0, 0.026))
    boss = cq.Workplane("XY").circle(0.046).extrude(0.016).translate((0, 0, 0.026))
    table = table.union(rim).union(boss)

    # Twelve shallow-looking through holes on the indexing pitch circle.
    for i in range(12):
        angle = i * math.tau / 12.0
        x = 0.096 * math.cos(angle)
        y = 0.096 * math.sin(angle)
        cutter = cq.Workplane("XY").center(x, y).circle(0.0065).extrude(0.070).translate((0, 0, -0.012))
        table = table.cut(cutter)

    return table


def _bearing_sleeve() -> cq.Workplane:
    """Hollow stationary bearing sleeve that clears the rotating spindle."""
    return cq.Workplane("XY").circle(0.074).circle(0.041).extrude(0.0555)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rotary_indexing_base")

    frame_paint = model.material("powder_coated_frame", rgba=(0.035, 0.045, 0.055, 1.0))
    machined_steel = model.material("machined_turntable", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_fastener = model.material("black_oxide_fasteners", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber = model.material("black_stop_bumper", rgba=(0.02, 0.018, 0.016, 1.0))
    brass = model.material("engraved_index_mark", rgba=(0.92, 0.68, 0.20, 1.0))

    bearing_mesh = mesh_from_cadquery(_bearing_sleeve(), "bearing_sleeve", tolerance=0.0008)
    turntable_mesh = mesh_from_cadquery(_turntable_body(), "turntable_body", tolerance=0.0008)
    hex_bolt_mesh = mesh_from_geometry(
        CylinderGeometry(0.012, 0.007, radial_segments=6, closed=True),
        "hex_bolt_head",
    )

    support = model.part("support_frame")
    support.visual(
        Box((0.430, 0.340, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=frame_paint,
        name="base_plate",
    )
    support.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=frame_paint,
        name="bearing_sleeve",
    )

    # Four welded ribs support the bearing from the bolted base without
    # crossing the spindle clearance hole.
    for name, xyz, size, yaw in (
        ("rib_0", (0.125, 0.0, 0.047), (0.140, 0.026, 0.035), 0.0),
        ("rib_1", (-0.125, 0.0, 0.047), (0.140, 0.026, 0.035), 0.0),
        ("rib_2", (0.0, 0.105, 0.047), (0.026, 0.120, 0.035), 0.0),
        ("rib_3", (0.0, -0.105, 0.047), (0.026, 0.120, 0.035), 0.0),
    ):
        support.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            material=frame_paint,
            name=name,
        )

    # Bolted mounting details at the four corners of the support frame.
    for idx, (x, y) in enumerate(((-0.175, -0.130), (-0.175, 0.130), (0.175, -0.130), (0.175, 0.130))):
        support.visual(
            Cylinder(radius=0.020, length=0.0035),
            origin=Origin(xyz=(x, y, 0.0313)),
            material=dark_fastener,
            name=f"washer_{idx}",
        )
        support.visual(
            hex_bolt_mesh,
            origin=Origin(xyz=(x, y, 0.0333), rpy=(0.0, 0.0, math.pi / 6.0)),
            material=dark_fastener,
            name=f"bolt_{idx}",
        )

    # Physical end stops: two stationary blocks on the arc and the turntable tab
    # below contacts their inner radial faces at the yaw limits.
    stop_limit = 0.78
    tab_outer_radius = 0.201
    stop_thickness = 0.022
    stop_radius = tab_outer_radius + stop_thickness / 2.0
    for block_name, bumper_name, angle in (
        ("stop_block_positive", "stop_bumper_positive", stop_limit),
        ("stop_block_negative", "stop_bumper_negative", -stop_limit),
    ):
        x = stop_radius * math.cos(angle)
        y = stop_radius * math.sin(angle)
        support.visual(
            Box((stop_thickness, 0.060, 0.0825)),
            origin=Origin(xyz=(x, y, 0.07075), rpy=(0.0, 0.0, angle)),
            material=frame_paint,
            name=block_name,
        )
        # Thin black elastomer bumper on the contact face.
        bumper_r = tab_outer_radius + 0.001
        support.visual(
            Box((0.003, 0.052, 0.030)),
            origin=Origin(
                xyz=(bumper_r * math.cos(angle), bumper_r * math.sin(angle), 0.098),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=bumper_name,
        )

    # A small fixed pointer makes the table read as an indexing base.
    support.visual(
        Box((0.010, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, -0.146, 0.0315), rpy=(0.0, 0.0, 0.0)),
        material=brass,
        name="front_pointer",
    )

    turntable = model.part("turntable")
    turntable.visual(
        turntable_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="turntable_body",
    )
    turntable.visual(
        Cylinder(radius=0.033, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=machined_steel,
        name="spindle",
    )
    turntable.visual(
        Box((0.070, 0.032, 0.018)),
        origin=Origin(xyz=(0.166, 0.0, 0.013)),
        material=machined_steel,
        name="stop_tab",
    )
    turntable.visual(
        Box((0.004, 0.060, 0.002)),
        origin=Origin(xyz=(0.0, -0.117, 0.0275)),
        material=brass,
        name="zero_mark",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-stop_limit, upper=stop_limit),
        motion_properties=MotionProperties(damping=0.8, friction=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    turntable = object_model.get_part("turntable")
    yaw = object_model.get_articulation("yaw_axis")

    ctx.check("single yaw articulation", len(object_model.articulations) == 1)
    ctx.check("yaw axis is vertical", tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check(
        "yaw limits match end stops",
        yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower < -0.70
        and yaw.motion_limits.upper > 0.70,
    )

    ctx.expect_contact(
        turntable,
        support,
        elem_a="turntable_body",
        elem_b="bearing_sleeve",
        contact_tol=0.001,
        name="turntable rests on bearing sleeve",
    )
    ctx.expect_within(
        turntable,
        support,
        axes="xy",
        inner_elem="spindle",
        outer_elem="bearing_sleeve",
        margin=0.0,
        name="spindle is centered in bearing footprint",
    )
    ctx.expect_overlap(
        turntable,
        support,
        axes="z",
        elem_a="spindle",
        elem_b="bearing_sleeve",
        min_overlap=0.045,
        name="spindle remains inserted through the sleeve",
    )

    upper = yaw.motion_limits.upper
    lower = yaw.motion_limits.lower
    with ctx.pose({yaw: upper}):
        ctx.expect_contact(
            turntable,
            support,
            elem_a="stop_tab",
            elem_b="stop_bumper_positive",
            contact_tol=0.004,
            name="positive yaw limit contacts mechanical stop",
        )
    with ctx.pose({yaw: lower}):
        ctx.expect_contact(
            turntable,
            support,
            elem_a="stop_tab",
            elem_b="stop_bumper_negative",
            contact_tol=0.004,
            name="negative yaw limit contacts mechanical stop",
        )

    return ctx.report()


object_model = build_object_model()
