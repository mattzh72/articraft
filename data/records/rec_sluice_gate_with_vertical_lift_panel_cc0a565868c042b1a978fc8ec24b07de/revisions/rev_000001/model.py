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

WALL_WIDTH = 2.10
WALL_THICKNESS = 0.58
WALL_HEIGHT = 2.55
OPENING_WIDTH = 1.20
OPENING_HEIGHT = 1.50
SILL_HEIGHT = 0.18
PIER_WIDTH = (WALL_WIDTH - OPENING_WIDTH) * 0.5

GUIDE_DEPTH = 0.20
GUIDE_WEB_THICKNESS = 0.018
GUIDE_FLANGE_THICKNESS = 0.020
GUIDE_FLANGE_REACH = 0.10
GUIDE_HEIGHT = 2.10
GUIDE_BOTTOM = 0.12
GUIDE_WEB_X = 0.620
GUIDE_BEAM_WIDTH = 1.50
GUIDE_BEAM_HEIGHT = 0.14
GUIDE_BEAM_Z = 2.13
GUIDE_OFFSET_Y = 0.10
BEAM_CENTER_GAP = 0.26
PAD_WIDTH = 0.12
PAD_CENTER_X = 0.13

GATE_TRAVEL = 0.70


def _build_handwheel_shape():
    ring = cq.Workplane("XZ").circle(0.160).circle(0.132).extrude(0.016, both=True)
    hub = cq.Workplane("XZ").circle(0.048).extrude(0.030, both=True)
    spoke_x = cq.Workplane("XY").box(0.24, 0.012, 0.030)
    spoke_z = cq.Workplane("XY").box(0.030, 0.012, 0.24)
    spoke_diag_a = spoke_x.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 45.0)
    spoke_diag_b = spoke_x.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -45.0)
    return ring.union(hub).union(spoke_x).union(spoke_z).union(spoke_diag_a).union(spoke_diag_b)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    masonry = model.material("masonry", rgba=(0.64, 0.62, 0.58, 1.0))
    damp_stone = model.material("damp_stone", rgba=(0.49, 0.50, 0.48, 1.0))
    guide_steel = model.material("guide_steel", rgba=(0.40, 0.44, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    gate_paint = model.material("gate_paint", rgba=(0.18, 0.31, 0.45, 1.0))
    gearbox_paint = model.material("gearbox_paint", rgba=(0.34, 0.37, 0.29, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    cover_paint = model.material("cover_paint", rgba=(0.54, 0.56, 0.49, 1.0))

    handwheel_mesh = mesh_from_cadquery(_build_handwheel_shape(), "sluice_handwheel")
    masonry_frame = model.part("masonry_frame")
    masonry_frame.visual(
        Box((PIER_WIDTH, WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH + PIER_WIDTH) * 0.5,
                -WALL_THICKNESS * 0.5,
                WALL_HEIGHT * 0.5,
            )
        ),
        material=masonry,
        name="left_pier",
    )
    masonry_frame.visual(
        Box((PIER_WIDTH, WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                (OPENING_WIDTH + PIER_WIDTH) * 0.5,
                -WALL_THICKNESS * 0.5,
                WALL_HEIGHT * 0.5,
            )
        ),
        material=masonry,
        name="right_pier",
    )
    masonry_frame.visual(
        Box((OPENING_WIDTH, WALL_THICKNESS, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, -WALL_THICKNESS * 0.5, SILL_HEIGHT * 0.5)),
        material=damp_stone,
        name="sill",
    )
    masonry_frame.visual(
        Box((OPENING_WIDTH, WALL_THICKNESS, WALL_HEIGHT - OPENING_HEIGHT - SILL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -WALL_THICKNESS * 0.5,
                SILL_HEIGHT + OPENING_HEIGHT + (WALL_HEIGHT - OPENING_HEIGHT - SILL_HEIGHT) * 0.5,
            )
        ),
        material=masonry,
        name="lintel",
    )
    masonry_frame.visual(
        Box((PIER_WIDTH + 0.12, WALL_THICKNESS + 0.08, 0.10)),
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH + PIER_WIDTH) * 0.5,
                -WALL_THICKNESS * 0.5,
                WALL_HEIGHT + 0.05,
            )
        ),
        material=masonry,
        name="left_coping",
    )
    masonry_frame.visual(
        Box((PIER_WIDTH + 0.12, WALL_THICKNESS + 0.08, 0.10)),
        origin=Origin(
            xyz=(
                (OPENING_WIDTH + PIER_WIDTH) * 0.5,
                -WALL_THICKNESS * 0.5,
                WALL_HEIGHT + 0.05,
            )
        ),
        material=masonry,
        name="right_coping",
    )

    guide_frame = model.part("guide_frame")
    guide_z = GUIDE_BOTTOM + GUIDE_HEIGHT * 0.5
    flange_y = GUIDE_DEPTH * 0.5 - GUIDE_FLANGE_THICKNESS * 0.5
    lip_x = GUIDE_WEB_X - 0.5 * GUIDE_FLANGE_REACH + 0.5 * GUIDE_WEB_THICKNESS
    beam_half_width = (GUIDE_BEAM_WIDTH - BEAM_CENTER_GAP) * 0.5
    beam_center_x = BEAM_CENTER_GAP * 0.5 + beam_half_width * 0.5
    guide_frame.visual(
        Box((GUIDE_WEB_THICKNESS, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_WEB_X, 0.0, guide_z)),
        material=guide_steel,
        name="left_web",
    )
    guide_frame.visual(
        Box((GUIDE_FLANGE_REACH, GUIDE_FLANGE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-lip_x, flange_y, guide_z)),
        material=guide_steel,
        name="left_front_lip",
    )
    guide_frame.visual(
        Box((GUIDE_FLANGE_REACH, GUIDE_FLANGE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-lip_x, -flange_y, guide_z)),
        material=guide_steel,
        name="left_rear_lip",
    )
    guide_frame.visual(
        Box((GUIDE_WEB_THICKNESS, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_WEB_X, 0.0, guide_z)),
        material=guide_steel,
        name="right_web",
    )
    guide_frame.visual(
        Box((GUIDE_FLANGE_REACH, GUIDE_FLANGE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(xyz=(lip_x, flange_y, guide_z)),
        material=guide_steel,
        name="right_front_lip",
    )
    guide_frame.visual(
        Box((GUIDE_FLANGE_REACH, GUIDE_FLANGE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(xyz=(lip_x, -flange_y, guide_z)),
        material=guide_steel,
        name="right_rear_lip",
    )
    guide_frame.visual(
        Box((beam_half_width, GUIDE_DEPTH, GUIDE_BEAM_HEIGHT)),
        origin=Origin(xyz=(-beam_center_x, 0.0, GUIDE_BEAM_Z)),
        material=dark_steel,
        name="left_beam",
    )
    guide_frame.visual(
        Box((beam_half_width, GUIDE_DEPTH, GUIDE_BEAM_HEIGHT)),
        origin=Origin(xyz=(beam_center_x, 0.0, GUIDE_BEAM_Z)),
        material=dark_steel,
        name="right_beam",
    )
    guide_frame.visual(
        Box((PAD_WIDTH, GUIDE_DEPTH, 0.10)),
        origin=Origin(xyz=(-PAD_CENTER_X, 0.0, GUIDE_BEAM_Z + 0.11)),
        material=dark_steel,
        name="left_pad",
    )
    guide_frame.visual(
        Box((PAD_WIDTH, GUIDE_DEPTH, 0.10)),
        origin=Origin(xyz=(PAD_CENTER_X, 0.0, GUIDE_BEAM_Z + 0.11)),
        material=dark_steel,
        name="right_pad",
    )
    guide_frame.visual(
        Box((GUIDE_BEAM_WIDTH, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, -0.075, GUIDE_BEAM_Z + 0.085)),
        material=dark_steel,
        name="rear_tie",
    )

    gate = model.part("gate")
    gate.visual(
        Box((1.00, 0.030, 1.50)),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=gate_paint,
        name="leaf",
    )
    gate.visual(
        Box((0.050, 0.070, 1.58)),
        origin=Origin(xyz=(-0.515, 0.0, 0.79)),
        material=dark_steel,
        name="left_edge",
    )
    gate.visual(
        Box((0.050, 0.070, 1.58)),
        origin=Origin(xyz=(0.515, 0.0, 0.79)),
        material=dark_steel,
        name="right_edge",
    )
    gate.visual(
        Box((1.10, 0.090, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        material=dark_steel,
        name="top_beam",
    )
    for index, z in enumerate((0.42, 0.85, 1.20)):
        gate.visual(
            Box((0.94, 0.055, 0.06)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name=f"stiffener_{index}",
        )
    gate.visual(
        Box((0.16, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.64)),
        material=dark_steel,
        name="stem_block",
    )
    gate.visual(
        Cylinder(radius=0.028, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 1.92)),
        material=wheel_steel,
        name="stem",
    )

    gearbox = model.part("gearbox")
    gearbox.visual(
        Box((0.42, 0.24, 0.24)),
        origin=Origin(xyz=(0.0, 0.03, 0.16)),
        material=gearbox_paint,
        name="housing",
    )
    gearbox.visual(
        Box((0.30, 0.18, 0.05)),
        origin=Origin(xyz=(0.0, 0.02, 0.285)),
        material=gearbox_paint,
        name="top_cap",
    )
    gearbox.visual(
        Box((0.12, 0.18, 0.04)),
        origin=Origin(xyz=(-0.13, 0.0, 0.02)),
        material=dark_steel,
        name="left_foot",
    )
    gearbox.visual(
        Box((0.12, 0.18, 0.04)),
        origin=Origin(xyz=(0.13, 0.0, 0.02)),
        material=dark_steel,
        name="right_foot",
    )
    gearbox.visual(
        Cylinder(radius=0.045, length=0.05),
        origin=Origin(xyz=(0.0, 0.145, 0.16), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="input_boss",
    )
    gearbox.visual(
        Cylinder(radius=0.040, length=0.10),
        origin=Origin(xyz=(0.0, 0.02, 0.06)),
        material=dark_steel,
        name="lift_nut",
    )

    wheel = model.part("wheel")
    wheel.visual(
        handwheel_mesh,
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
        material=wheel_steel,
        name="wheel_rim",
    )
    wheel.visual(
        Box((0.09, 0.012, 0.022)),
        origin=Origin(xyz=(0.11, 0.085, 0.11), rpy=(0.0, -math.pi * 0.25, 0.0)),
        material=wheel_steel,
        name="wheel_arm",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.06),
        origin=Origin(xyz=(0.145, 0.085, 0.145), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=wheel_steel,
        name="wheel_grip",
    )
    wheel.visual(
        Cylinder(radius=0.022, length=0.08),
        origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=wheel_steel,
        name="wheel_shaft",
    )

    cover = model.part("cover")
    cover.visual(
        Box((0.40, 0.036, 0.040)),
        origin=Origin(xyz=(-0.20, 0.018, 0.080)),
        material=cover_paint,
        name="cover_top_rail",
    )
    cover.visual(
        Box((0.40, 0.036, 0.040)),
        origin=Origin(xyz=(-0.20, 0.018, -0.080)),
        material=cover_paint,
        name="cover_bottom_rail",
    )
    cover.visual(
        Box((0.090, 0.036, 0.200)),
        origin=Origin(xyz=(-0.355, 0.018, 0.0)),
        material=cover_paint,
        name="cover_left_rail",
    )
    cover.visual(
        Box((0.090, 0.036, 0.200)),
        origin=Origin(xyz=(-0.045, 0.018, 0.0)),
        material=cover_paint,
        name="cover_right_rail",
    )
    cover.visual(
        Box((0.090, 0.018, 0.040)),
        origin=Origin(xyz=(-0.045, 0.027, 0.0)),
        material=cover_paint,
        name="cover_bridge",
    )
    cover.visual(
        Box((0.040, 0.016, 0.030)),
        origin=Origin(xyz=(-0.365, 0.023, 0.0)),
        material=wheel_steel,
        name="cover_latch",
    )
    cover.visual(
        Box((0.018, 0.052, 0.18)),
        origin=Origin(xyz=(-0.008, 0.010, 0.0)),
        material=dark_steel,
        name="hinge_leaf",
    )
    cover.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=dark_steel,
        name="hinge_lower",
    )
    cover.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_steel,
        name="hinge_upper",
    )

    model.articulation(
        "frame_to_guide",
        ArticulationType.FIXED,
        parent=masonry_frame,
        child=guide_frame,
        origin=Origin(xyz=(0.0, GUIDE_OFFSET_Y, 0.0)),
    )
    model.articulation(
        "guide_to_gate",
        ArticulationType.PRISMATIC,
        parent=guide_frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.20, lower=0.0, upper=GATE_TRAVEL),
    )
    model.articulation(
        "guide_to_gearbox",
        ArticulationType.FIXED,
        parent=guide_frame,
        child=gearbox,
        origin=Origin(xyz=(0.0, 0.02, 2.29)),
    )
    model.articulation(
        "gearbox_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=gearbox,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.170, 0.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=6.0),
    )
    model.articulation(
        "gearbox_to_cover",
        ArticulationType.REVOLUTE,
        parent=gearbox,
        child=cover,
        origin=Origin(xyz=(0.20, 0.166, 0.16)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    masonry_frame = object_model.get_part("masonry_frame")
    guide_frame = object_model.get_part("guide_frame")
    gate = object_model.get_part("gate")
    gearbox = object_model.get_part("gearbox")
    wheel = object_model.get_part("wheel")
    cover = object_model.get_part("cover")

    gate_lift = object_model.get_articulation("guide_to_gate")
    wheel_spin = object_model.get_articulation("gearbox_to_wheel")
    cover_hinge = object_model.get_articulation("gearbox_to_cover")

    ctx.expect_gap(
        gate,
        masonry_frame,
        axis="z",
        positive_elem="leaf",
        negative_elem="sill",
        min_gap=0.0,
        max_gap=0.01,
        name="gate leaf seats just above the sill",
    )
    ctx.expect_overlap(
        gate,
        masonry_frame,
        axes="x",
        elem_a="leaf",
        elem_b="sill",
        min_overlap=0.98,
        name="gate leaf spans the channel opening",
    )
    ctx.expect_gap(
        cover,
        gearbox,
        axis="y",
        positive_elem="cover_right_rail",
        negative_elem="housing",
        min_gap=0.001,
        max_gap=0.021,
        name="cover closes just proud of the gearbox face",
    )
    ctx.allow_overlap(
        gate,
        gearbox,
        elem_a="stem",
        elem_b="lift_nut",
        reason="The lifting stem is intentionally represented as a solid threaded shaft engaging a simplified solid gearbox nut.",
    )

    gate_closed = ctx.part_world_position(gate)
    gate_open = None
    with ctx.pose({gate_lift: GATE_TRAVEL}):
        gate_open = ctx.part_world_position(gate)
        ctx.expect_overlap(
            gate,
            guide_frame,
            axes="z",
            elem_a="left_edge",
            elem_b="left_web",
            min_overlap=1.0,
            name="gate remains captured by the left guide at full lift",
        )

    grip_closed = _aabb_center(ctx.part_element_world_aabb(wheel, elem="wheel_grip"))
    grip_turned = None
    with ctx.pose({wheel_spin: math.pi * 0.5}):
        grip_turned = _aabb_center(ctx.part_element_world_aabb(wheel, elem="wheel_grip"))

    latch_closed = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_latch"))
    latch_open = None
    with ctx.pose({cover_hinge: 1.10}):
        latch_open = _aabb_center(ctx.part_element_world_aabb(cover, elem="cover_latch"))

    ctx.check(
        "gate lifts upward",
        gate_closed is not None and gate_open is not None and gate_open[2] > gate_closed[2] + 0.60,
        details=f"closed={gate_closed}, open={gate_open}",
    )
    ctx.check(
        "handwheel grip sweeps around the shaft",
        grip_closed is not None and grip_turned is not None and grip_turned[2] < grip_closed[2] - 0.18,
        details=f"closed={grip_closed}, turned={grip_turned}",
    )
    ctx.check(
        "cover swings outward for service",
        latch_closed is not None and latch_open is not None and latch_open[1] > latch_closed[1] + 0.10,
        details=f"closed={latch_closed}, open={latch_open}",
    )

    return ctx.report()


object_model = build_object_model()
