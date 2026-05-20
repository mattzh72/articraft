from __future__ import annotations

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.60
BODY_DEPTH = 0.30
BODY_HEIGHT = 0.18
SHELL_THICKNESS = 0.012
FRONT_FASCIA_HEIGHT = 0.070
FILTER_THICKNESS = 0.008
FILTER_CENTER_Y = -0.070
FILTER_CENTER_Z = 0.022
GUIDE_WIDTH = 0.014
GUIDE_HEIGHT = 0.010
GUIDE_LENGTH = 0.140
VISOR_TRAVEL = 0.105


def _filter_panel_shape() -> cq.Workplane:
    filter_panel = cq.Workplane("XY").box(BODY_WIDTH - 0.032, 0.140, FILTER_THICKNESS).translate(
        (0.0, FILTER_CENTER_Y, FILTER_CENTER_Z)
    )
    slot_x_positions = (-0.200, -0.120, -0.040, 0.040, 0.120, 0.200)
    for slot_x in slot_x_positions:
        slot = cq.Workplane("XY").box(0.045, 0.105, FILTER_THICKNESS + 0.010).translate(
            (slot_x, FILTER_CENTER_Y, FILTER_CENTER_Z)
        )
        filter_panel = filter_panel.cut(slot)

    return filter_panel.edges("|Z").fillet(0.002)


def _visor_shape() -> cq.Workplane:
    front_lip = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 0.028, 0.018, 0.046)
        .translate((0.0, 0.009, 0.023))
        .edges("|Z")
        .fillet(0.004)
    )
    tray = cq.Workplane("XY").box(BODY_WIDTH - 0.062, 0.172, 0.008).translate((0.0, -0.084, 0.050))
    left_runner = cq.Workplane("XY").box(0.012, 0.120, 0.010).translate(
        (-(BODY_WIDTH / 2.0 - SHELL_THICKNESS - GUIDE_WIDTH / 2.0 - 0.006), -0.100, 0.041)
    )
    right_runner = cq.Workplane("XY").box(0.012, 0.120, 0.010).translate(
        (BODY_WIDTH / 2.0 - SHELL_THICKNESS - GUIDE_WIDTH / 2.0 - 0.006, -0.100, 0.041)
    )

    visor = front_lip.union(tray)
    visor = visor.union(left_runner)
    visor = visor.union(right_runner)
    finger_pull = cq.Workplane("YZ").circle(0.010).extrude(BODY_WIDTH - 0.120).translate(
        (-(BODY_WIDTH - 0.120) / 2.0, 0.003, 0.010)
    )
    return visor.cut(finger_pull)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_out_visor_range_hood")

    body_finish = model.material("body_finish", rgba=(0.90, 0.91, 0.93, 1.0))
    filter_steel = model.material("filter_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    visor_finish = model.material("visor_finish", rgba=(0.16, 0.17, 0.19, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_knob = model.material("satin_knob", rgba=(0.14, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - SHELL_THICKNESS / 2.0)),
        material=body_finish,
        name="top_panel",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_WIDTH - SHELL_THICKNESS) / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="side_shell_0",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=((BODY_WIDTH - SHELL_THICKNESS) / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="side_shell_1",
    )
    body.visual(
        Box((BODY_WIDTH, SHELL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH - SHELL_THICKNESS) / 2.0, BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="back_panel",
    )
    body.visual(
        Box((BODY_WIDTH, SHELL_THICKNESS, FRONT_FASCIA_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH - SHELL_THICKNESS) / 2.0,
                BODY_HEIGHT - FRONT_FASCIA_HEIGHT / 2.0,
            )
        ),
        material=body_finish,
        name="front_fascia",
    )
    body.visual(
        mesh_from_cadquery(
            _filter_panel_shape(),
            "range_hood_filter",
        ),
        material=filter_steel,
        name="filter_panel",
    )
    body.visual(
        Box((GUIDE_WIDTH, GUIDE_LENGTH, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH / 2.0 - SHELL_THICKNESS - GUIDE_WIDTH / 2.0),
                0.055,
                0.051,
            )
        ),
        material=body_finish,
        name="guide_0",
    )
    body.visual(
        Box((GUIDE_WIDTH, GUIDE_LENGTH, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_WIDTH / 2.0 - SHELL_THICKNESS - GUIDE_WIDTH / 2.0,
                0.055,
                0.051,
            )
        ),
        material=body_finish,
        name="guide_1",
    )

    visor = model.part("visor")
    visor.visual(
        mesh_from_cadquery(_visor_shape(), "range_hood_visor"),
        material=visor_finish,
        name="visor_shell",
    )

    model.articulation(
        "body_to_visor",
        ArticulationType.PRISMATIC,
        parent=body,
        child=visor,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=VISOR_TRAVEL,
        ),
    )

    rocker = model.part("rocker")
    rocker.visual(
        Box((0.004, 0.018, 0.004)),
        origin=Origin(xyz=(0.002, 0.0, 0.002)),
        material=control_black,
        name="rocker_hinge",
    )
    rocker.visual(
        Box((0.010, 0.018, 0.030)),
        origin=Origin(xyz=(0.008, 0.0, 0.019)),
        material=control_black,
        name="rocker_paddle",
    )

    model.articulation(
        "body_to_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(BODY_WIDTH / 2.0, 0.080, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.10,
            upper=0.22,
        ),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.003, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_black,
        name="knob_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.024,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.040, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.001),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "hood_knob",
        ),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_knob,
        name="knob_shell",
    )

    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(BODY_WIDTH / 2.0, 0.010, 0.096)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    visor = object_model.get_part("visor")
    rocker = object_model.get_part("rocker")
    knob = object_model.get_part("knob")
    visor_slide = object_model.get_articulation("body_to_visor")
    rocker_hinge = object_model.get_articulation("body_to_rocker")
    knob_spin = object_model.get_articulation("body_to_knob")

    ctx.expect_overlap(
        visor,
        body,
        axes="x",
        min_overlap=0.52,
        name="visor stays aligned with hood width",
    )

    rest_pos = ctx.part_world_position(visor)
    with ctx.pose({visor_slide: VISOR_TRAVEL}):
        ctx.expect_overlap(
            visor,
            body,
            axes="y",
            min_overlap=0.040,
            name="visor retains insertion at full extension",
        )
        extended_pos = ctx.part_world_position(visor)

    ctx.check(
        "visor extends forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.090,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_contact(
        rocker,
        body,
        elem_a="rocker_hinge",
        elem_b="side_shell_1",
        name="rocker hinge stays mounted on the right side shell",
    )
    rocker_rest_aabb = ctx.part_element_world_aabb(rocker, elem="rocker_paddle")
    with ctx.pose({rocker_hinge: 0.20}):
        rocker_tipped_aabb = ctx.part_element_world_aabb(rocker, elem="rocker_paddle")
    ctx.check(
        "rocker tips outward",
        rocker_rest_aabb is not None
        and rocker_tipped_aabb is not None
        and rocker_tipped_aabb[1][0] > rocker_rest_aabb[1][0] + 0.004,
        details=f"rest={rocker_rest_aabb}, tipped={rocker_tipped_aabb}",
    )

    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_shaft",
        elem_b="side_shell_1",
        name="knob shaft seats on the side shell",
    )
    with ctx.pose({knob_spin: 1.30}):
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_shaft",
            elem_b="side_shell_1",
            name="knob stays seated while rotated",
        )
    knob_limits = knob_spin.motion_limits
    ctx.check(
        "knob uses a continuous side-facing shaft",
        knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None
        and abs(knob_spin.axis[0]) > 0.99,
        details=f"axis={knob_spin.axis}, limits={knob_limits}",
    )

    return ctx.report()


object_model = build_object_model()
