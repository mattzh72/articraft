from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OPENING_W = 1.20
OPENING_H = 1.35
FRAME_DEPTH = 0.22
COLUMN_W = 0.22
FRAME_W = OPENING_W + (2.0 * COLUMN_W)
SILL_H = 0.20
GUIDE_TOP = 2.75
TOP_BEAM_H = 0.24
GUIDE_CHANNEL_H = 2.30
GUIDE_SLOT = 0.074
LIP_PROJ = 0.060
LIP_D = 0.032

GATE_W = OPENING_W - 0.06
GATE_H = 1.68
GATE_T = 0.062
GATE_BOTTOM_CLOSED = SILL_H
GATE_CENTER_Z_CLOSED = GATE_BOTTOM_CLOSED + (GATE_H / 2.0)
GATE_TRAVEL = 1.30

GEARBOX_W = 0.34
GEARBOX_D = 0.24
GEARBOX_H = 0.22
AXLE_R = 0.055
AXLE_H = 0.08
HANDWHEEL_Z = GUIDE_TOP + GEARBOX_H + AXLE_H

WHEEL_R_OUTER = 0.33
WHEEL_R_INNER = 0.27
WHEEL_THICK = 0.028
WHEEL_HUB_R = 0.075
WHEEL_HUB_H = 0.042
KNOB_R = 0.014
KNOB_H = 0.11
KNOB_RADIUS = 0.235


def _build_handwheel_mesh() -> cq.Workplane:
    rim = (
        cq.Workplane("XY")
        .circle(WHEEL_R_OUTER)
        .circle(WHEEL_R_INNER)
        .extrude(WHEEL_THICK)
    )
    hub = cq.Workplane("XY").circle(WHEEL_HUB_R).extrude(WHEEL_HUB_H)

    wheel = rim.union(hub)
    spoke_width = 0.022
    spoke_length = (WHEEL_R_INNER - WHEEL_HUB_R) + 0.030
    spoke_center_x = (WHEEL_HUB_R + WHEEL_R_INNER) / 2.0
    for angle_deg in (0, 60, 120, 180, 240, 300):
        spoke = (
            cq.Workplane("XY")
            .box(spoke_length, spoke_width, WHEEL_THICK, centered=(True, True, False))
            .translate((spoke_center_x, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        wheel = wheel.union(spoke)

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate_assembly")

    model.material("frame_steel", rgba=(0.42, 0.46, 0.50, 1.0))
    model.material("gate_steel", rgba=(0.20, 0.33, 0.54, 1.0))
    model.material("operator_red", rgba=(0.70, 0.10, 0.08, 1.0))
    model.material("operator_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_W, FRAME_DEPTH, SILL_H)),
        origin=Origin(xyz=(0.0, 0.0, SILL_H / 2.0)),
        material="frame_steel",
        name="sill",
    )
    frame.visual(
        Box((COLUMN_W, FRAME_DEPTH, GUIDE_TOP)),
        origin=Origin(xyz=(-(OPENING_W / 2.0) - (COLUMN_W / 2.0), 0.0, GUIDE_TOP / 2.0)),
        material="frame_steel",
        name="left_column",
    )
    frame.visual(
        Box((COLUMN_W, FRAME_DEPTH, GUIDE_TOP)),
        origin=Origin(xyz=((OPENING_W / 2.0) + (COLUMN_W / 2.0), 0.0, GUIDE_TOP / 2.0)),
        material="frame_steel",
        name="right_column",
    )
    frame.visual(
        Box((FRAME_W, FRAME_DEPTH, TOP_BEAM_H)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP - (TOP_BEAM_H / 2.0))),
        material="frame_steel",
        name="top_beam",
    )

    lip_z = SILL_H + (GUIDE_CHANNEL_H / 2.0)
    front_lip_y = (GUIDE_SLOT / 2.0) + (LIP_D / 2.0)
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        lip_x = x_sign * ((OPENING_W / 2.0) - (LIP_PROJ / 2.0))
        frame.visual(
            Box((LIP_PROJ, LIP_D, GUIDE_CHANNEL_H)),
            origin=Origin(xyz=(lip_x, front_lip_y, lip_z)),
            material="frame_steel",
            name=f"{side_name}_front_lip",
        )
        frame.visual(
            Box((LIP_PROJ, LIP_D, GUIDE_CHANNEL_H)),
            origin=Origin(xyz=(lip_x, -front_lip_y, lip_z)),
            material="frame_steel",
            name=f"{side_name}_rear_lip",
        )

    frame.visual(
        Box((GEARBOX_W, GEARBOX_D, GEARBOX_H)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP + (GEARBOX_H / 2.0))),
        material="frame_steel",
        name="gearbox",
    )
    frame.visual(
        Cylinder(radius=AXLE_R, length=AXLE_H),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP + GEARBOX_H + (AXLE_H / 2.0))),
        material="operator_dark",
        name="axle_post",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_DEPTH, GUIDE_TOP + GEARBOX_H + AXLE_H)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, (GUIDE_TOP + GEARBOX_H + AXLE_H) / 2.0)),
    )

    gate = model.part("gate")
    gate.visual(
        Box((GATE_W, GATE_T, GATE_H)),
        material="gate_steel",
        name="gate_panel",
    )
    gate.visual(
        Box((GATE_W * 0.78, GATE_T, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, (GATE_H / 2.0) + 0.06)),
        material="gate_steel",
        name="top_header",
    )
    gate.visual(
        Box((0.24, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, (GATE_H / 2.0) + 0.21)),
        material="gate_steel",
        name="lifting_block",
    )
    gate.visual(
        Box((GATE_W * 0.94, GATE_T, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -(GATE_H / 2.0) + 0.03)),
        material="gate_steel",
        name="bottom_edge",
    )
    gate.inertial = Inertial.from_geometry(
        Box((GATE_W, GATE_T, GATE_H + 0.30)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_build_handwheel_mesh(), "handwheel_mesh"),
        material="operator_red",
        name="wheel_mesh",
    )
    handwheel.visual(
        Cylinder(radius=KNOB_R, length=KNOB_H),
        origin=Origin(xyz=(KNOB_RADIUS, 0.0, WHEEL_THICK + (KNOB_H / 2.0))),
        material="operator_dark",
        name="spinner_knob",
    )
    handwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_R_OUTER, length=WHEEL_THICK + KNOB_H),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, (WHEEL_THICK + KNOB_H) / 2.0)),
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, GATE_CENTER_Z_CLOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.18,
            lower=0.0,
            upper=GATE_TRAVEL,
        ),
    )
    model.articulation(
        "frame_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.0, HANDWHEEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate")
    handwheel = object_model.get_part("handwheel")
    gate_slide = object_model.get_articulation("frame_to_gate")
    wheel_spin = object_model.get_articulation("frame_to_handwheel")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        aabb_min, aabb_max = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb_min, aabb_max))

    ctx.check(
        "gate slide is vertical prismatic motion",
        gate_slide.articulation_type == ArticulationType.PRISMATIC and tuple(gate_slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={gate_slide.articulation_type}, axis={gate_slide.axis}",
    )
    ctx.check(
        "handwheel uses a continuous vertical axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )

    with ctx.pose({gate_slide: 0.0, wheel_spin: 0.0}):
        ctx.expect_gap(
            gate,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="gate_panel",
            negative_elem="sill",
            name="closed gate bears on the sill",
        )
        ctx.expect_gap(
            frame,
            gate,
            axis="y",
            max_gap=0.008,
            max_penetration=0.0,
            positive_elem="left_front_lip",
            negative_elem="gate_panel",
            name="front guide lip clears the closed gate panel",
        )
        ctx.expect_gap(
            gate,
            frame,
            axis="y",
            max_gap=0.008,
            max_penetration=0.0,
            positive_elem="gate_panel",
            negative_elem="left_rear_lip",
            name="rear guide lip clears the closed gate panel",
        )
        ctx.expect_gap(
            handwheel,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="wheel_mesh",
            negative_elem="axle_post",
            name="handwheel sits on the fixed axle post",
        )

    rest_pos = ctx.part_world_position(gate)
    upper = gate_slide.motion_limits.upper if gate_slide.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({gate_slide: upper}):
            ctx.expect_overlap(
                gate,
                frame,
                axes="xz",
                min_overlap=0.02,
                elem_a="gate_panel",
                elem_b="left_front_lip",
                name="raised gate remains captured by the left guide",
            )
            ctx.expect_overlap(
                gate,
                frame,
                axes="xz",
                min_overlap=0.02,
                elem_a="gate_panel",
                elem_b="right_front_lip",
                name="raised gate remains captured by the right guide",
            )
            raised_pos = ctx.part_world_position(gate)
        ctx.check(
            "gate lifts upward at its upper limit",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 1.0,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    knob_center_0 = None
    knob_center_90 = None
    with ctx.pose({wheel_spin: 0.0}):
        knob_center_0 = _aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner_knob"))
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        knob_center_90 = _aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner_knob"))
    ctx.check(
        "handwheel rotation carries the spinner knob around the axle",
        knob_center_0 is not None
        and knob_center_90 is not None
        and knob_center_0[0] > 0.20
        and abs(knob_center_0[1]) < 0.03
        and knob_center_90[1] > 0.20
        and abs(knob_center_90[0]) < 0.03,
        details=f"q0={knob_center_0}, q90={knob_center_90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
