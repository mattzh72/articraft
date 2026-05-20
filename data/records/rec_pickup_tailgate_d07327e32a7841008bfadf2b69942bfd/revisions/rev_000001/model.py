from __future__ import annotations

import math

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
import cadquery as cq


TAILGATE_WIDTH = 1.70
TAILGATE_HEIGHT = 0.68
TAILGATE_THICKNESS = 0.060
TAILGATE_BOTTOM_LIFT = 0.020
HINGE_Z = 0.55

HATCH_OPENING_CENTER_X = 0.52
HATCH_OPENING_CENTER_Z = 0.40
HATCH_OPENING_WIDTH = 0.44
HATCH_OPENING_HEIGHT = 0.36
HATCH_HINGE_X = HATCH_OPENING_CENTER_X + HATCH_OPENING_WIDTH / 2.0 - 0.020
HATCH_HINGE_Y = -0.065
HATCH_WIDTH = 0.36
HATCH_HEIGHT = 0.32
HATCH_THICKNESS = 0.040


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _tailgate_shell_geometry() -> cq.Workplane:
    """One connected tailgate stamping with a real utility-hatch cutout."""

    body = _cq_box(
        (TAILGATE_WIDTH, TAILGATE_THICKNESS, TAILGATE_HEIGHT),
        (0.0, -TAILGATE_THICKNESS / 2.0, TAILGATE_BOTTOM_LIFT + TAILGATE_HEIGHT / 2.0),
    )

    # The hatch is an actual through-opening, not a hatch panel pasted onto a
    # solid tailgate.  It leaves enough metal around the side and bottom to read
    # as a formed pickup tailgate stamping.
    cutter = _cq_box(
        (HATCH_OPENING_WIDTH, TAILGATE_THICKNESS * 4.0, HATCH_OPENING_HEIGHT),
        (HATCH_OPENING_CENTER_X, -TAILGATE_THICKNESS / 2.0, HATCH_OPENING_CENTER_Z),
    )
    body = body.cut(cutter)

    rear_y = -TAILGATE_THICKNESS - 0.005
    rib_depth = 0.012
    outer_rail = 0.045

    # Raised perimeter stampings and hatch surround.  The tiny 1 mm overlap
    # into the base sheet makes each rib one connected manufactured shell.
    features = [
        _cq_box((TAILGATE_WIDTH, rib_depth, outer_rail), (0.0, rear_y, TAILGATE_BOTTOM_LIFT + outer_rail / 2.0)),
        _cq_box((TAILGATE_WIDTH, rib_depth, outer_rail), (0.0, rear_y, TAILGATE_BOTTOM_LIFT + TAILGATE_HEIGHT - outer_rail / 2.0)),
        _cq_box((outer_rail, rib_depth, TAILGATE_HEIGHT), (-TAILGATE_WIDTH / 2.0 + outer_rail / 2.0, rear_y, TAILGATE_BOTTOM_LIFT + TAILGATE_HEIGHT / 2.0)),
        _cq_box((outer_rail, rib_depth, TAILGATE_HEIGHT), (TAILGATE_WIDTH / 2.0 - outer_rail / 2.0, rear_y, TAILGATE_BOTTOM_LIFT + TAILGATE_HEIGHT / 2.0)),
        _cq_box((0.78, rib_depth, 0.026), (-0.26, rear_y, 0.24)),
        _cq_box((0.68, rib_depth, 0.024), (-0.33, rear_y, 0.51)),
    ]

    surround = 0.030
    surround_z = HATCH_OPENING_CENTER_Z
    surround_h = HATCH_OPENING_HEIGHT + 2.0 * surround
    surround_w = HATCH_OPENING_WIDTH + 2.0 * surround
    features.extend(
        [
            _cq_box((surround, rib_depth, surround_h), (HATCH_OPENING_CENTER_X - HATCH_OPENING_WIDTH / 2.0 - surround / 2.0, rear_y, surround_z)),
            _cq_box((surround, rib_depth, surround_h), (HATCH_OPENING_CENTER_X + HATCH_OPENING_WIDTH / 2.0 + surround / 2.0, rear_y, surround_z)),
            _cq_box((surround_w, rib_depth, surround), (HATCH_OPENING_CENTER_X, rear_y, surround_z - HATCH_OPENING_HEIGHT / 2.0 - surround / 2.0)),
            _cq_box((surround_w, rib_depth, surround), (HATCH_OPENING_CENTER_X, rear_y, surround_z + HATCH_OPENING_HEIGHT / 2.0 + surround / 2.0)),
        ]
    )

    for feature in features:
        body = body.union(feature)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_utility_hatch")

    body_paint = model.material("fleet_white_paint", rgba=(0.90, 0.92, 0.88, 1.0))
    black = model.material("black_powdercoat", rgba=(0.015, 0.016, 0.014, 1.0))
    dark = model.material("dark_plastic", rgba=(0.035, 0.040, 0.040, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    red = model.material("tail_lamp_red", rgba=(0.75, 0.04, 0.03, 1.0))

    bed = model.part("bed")
    bed.visual(
        Box((1.90, 0.14, 0.11)),
        origin=Origin(xyz=(0.0, 0.10, HINGE_Z - 0.085)),
        material=black,
        name="lower_sill",
    )
    for x in (-0.96, 0.96):
        bed.visual(
            Box((0.085, 0.14, 0.74)),
            origin=Origin(xyz=(x, 0.095, HINGE_Z + 0.285)),
            material=body_paint,
            name=f"bed_post_{0 if x < 0 else 1}",
        )
        bed.visual(
            Box((0.020, 0.006, 0.32)),
            origin=Origin(xyz=(x, 0.023, HINGE_Z + 0.33)),
            material=red,
            name=f"tail_lamp_{0 if x < 0 else 1}",
        )
    bed.visual(
        Box((1.92, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.080, HINGE_Z + 0.675)),
        material=black,
        name="top_bed_lip",
    )
    bed.visual(
        Box((1.60, 0.52, 0.045)),
        origin=Origin(xyz=(0.0, 0.34, HINGE_Z - 0.020)),
        material=dark,
        name="bed_floor_stub",
    )

    # Alternating stationary hinge knuckles on the bed side of the lower axis.
    for i, (x, length) in enumerate(((-0.72, 0.22), (0.0, 0.18), (0.72, 0.22))):
        bed.visual(
            Cylinder(radius=0.021, length=length),
            origin=Origin(xyz=(x, 0.0, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"bed_hinge_knuckle_{i}",
        )
        bed.visual(
            Box((length * 0.72, 0.045, 0.040)),
            origin=Origin(xyz=(x, 0.025, HINGE_Z - 0.030)),
            material=black,
            name=f"hinge_bracket_{i}",
        )

    gate = model.part("tailgate")
    gate.visual(
        mesh_from_cadquery(_tailgate_shell_geometry(), "tailgate_shell", tolerance=0.001),
        origin=Origin(),
        material=body_paint,
        name="tailgate_shell",
    )
    gate.visual(
        Box((0.42, 0.018, 0.030)),
        origin=Origin(xyz=(-0.28, -TAILGATE_THICKNESS - 0.006, 0.615)),
        material=dark,
        name="tailgate_pull_handle",
    )
    gate.visual(
        Box((0.72, 0.018, 0.028)),
        origin=Origin(xyz=(-0.18, -TAILGATE_THICKNESS - 0.006, 0.105)),
        material=dark,
        name="license_plate_recess",
    )
    gate.visual(
        Box((0.026, 0.014, HATCH_OPENING_HEIGHT + 0.060)),
        origin=Origin(xyz=(HATCH_HINGE_X + 0.027, -TAILGATE_THICKNESS - 0.015, HATCH_OPENING_CENTER_Z)),
        material=black,
        name="hatch_hinge_mount",
    )
    # Moving-side hinge barrels alternate with the bed knuckles and ride on the
    # same horizontal axis as the articulated joint.
    for i, (x, length) in enumerate(((-0.39, 0.42), (0.39, 0.42))):
        gate.visual(
            Cylinder(radius=0.025, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"gate_hinge_knuckle_{i}",
        )

    utility_hatch = model.part("utility_hatch")
    utility_hatch.visual(
        Box((HATCH_WIDTH, HATCH_THICKNESS, HATCH_HEIGHT)),
        origin=Origin(xyz=(-HATCH_WIDTH / 2.0, HATCH_THICKNESS / 2.0, 0.0)),
        material=body_paint,
        name="hatch_panel",
    )
    utility_hatch.visual(
        Box((HATCH_WIDTH - 0.060, 0.010, 0.028)),
        origin=Origin(xyz=(-HATCH_WIDTH / 2.0, -0.004, HATCH_HEIGHT / 2.0 - 0.025)),
        material=black,
        name="hatch_top_seal",
    )
    utility_hatch.visual(
        Box((HATCH_WIDTH - 0.060, 0.010, 0.028)),
        origin=Origin(xyz=(-HATCH_WIDTH / 2.0, -0.004, -HATCH_HEIGHT / 2.0 + 0.025)),
        material=black,
        name="hatch_bottom_seal",
    )
    utility_hatch.visual(
        Box((0.025, 0.010, HATCH_HEIGHT - 0.020)),
        origin=Origin(xyz=(-HATCH_WIDTH + 0.020, -0.004, 0.0)),
        material=black,
        name="hatch_latch_seal",
    )
    utility_hatch.visual(
        Box((0.030, 0.012, HATCH_HEIGHT - 0.016)),
        origin=Origin(xyz=(-0.018, -0.006, 0.0)),
        material=steel,
        name="hatch_hinge_leaf",
    )
    utility_hatch.visual(
        Cylinder(radius=0.016, length=HATCH_HEIGHT - 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="hatch_hinge_barrel",
    )
    utility_hatch.visual(
        Box((0.110, 0.014, 0.040)),
        origin=Origin(xyz=(-HATCH_WIDTH + 0.080, -0.012, 0.0)),
        material=dark,
        name="hatch_pull_handle",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=280.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "tailgate_to_utility_hatch",
        ArticulationType.REVOLUTE,
        parent=gate,
        child=utility_hatch,
        origin=Origin(xyz=(HATCH_HINGE_X, HATCH_HINGE_Y, HATCH_OPENING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    gate = object_model.get_part("tailgate")
    hatch = object_model.get_part("utility_hatch")
    gate_joint = object_model.get_articulation("bed_to_tailgate")
    hatch_joint = object_model.get_articulation("tailgate_to_utility_hatch")

    ctx.allow_overlap(
        gate,
        hatch,
        elem_a="hatch_hinge_mount",
        elem_b="hatch_hinge_barrel",
        reason="The hatch barrel is intentionally captured in the side hinge mount so the access door is mechanically supported.",
    )

    with ctx.pose({gate_joint: 0.0, hatch_joint: 0.0}):
        ctx.expect_overlap(
            gate,
            hatch,
            axes="z",
            min_overlap=HATCH_HEIGHT - 0.030,
            elem_a="hatch_hinge_mount",
            elem_b="hatch_hinge_barrel",
            name="utility hatch hinge barrel is retained by the mount",
        )
        ctx.expect_within(
            hatch,
            gate,
            axes="xz",
            margin=0.020,
            inner_elem="hatch_panel",
            outer_elem="tailgate_shell",
            name="utility hatch sits inside the tailgate face",
        )
        closed_gate_aabb = ctx.part_element_world_aabb(gate, elem="tailgate_shell")
        closed_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")

        ctx.check(
            "closed hatch is flush with the gate skin",
            closed_gate_aabb is not None
            and closed_hatch_aabb is not None
            and abs(closed_hatch_aabb[0][1] - closed_gate_aabb[0][1]) < 0.015,
            details=f"gate={closed_gate_aabb}, hatch={closed_hatch_aabb}",
        )

    with ctx.pose({gate_joint: 1.20, hatch_joint: 0.0}):
        lowered_gate_aabb = ctx.part_element_world_aabb(gate, elem="tailgate_shell")

    with ctx.pose({gate_joint: 0.0, hatch_joint: 1.15}):
        opened_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")

    ctx.check(
        "main tailgate folds rearward and downward",
        closed_gate_aabb is not None
        and lowered_gate_aabb is not None
        and lowered_gate_aabb[0][1] < closed_gate_aabb[0][1] - 0.30
        and lowered_gate_aabb[1][2] < closed_gate_aabb[1][2] - 0.18,
        details=f"closed={closed_gate_aabb}, lowered={lowered_gate_aabb}",
    )
    ctx.check(
        "utility hatch swings out on a vertical side hinge",
        closed_hatch_aabb is not None
        and opened_hatch_aabb is not None
        and (opened_hatch_aabb[0][1] + opened_hatch_aabb[1][1]) / 2.0
        < (closed_hatch_aabb[0][1] + closed_hatch_aabb[1][1]) / 2.0 - 0.12,
        details=f"closed={closed_hatch_aabb}, opened={opened_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
