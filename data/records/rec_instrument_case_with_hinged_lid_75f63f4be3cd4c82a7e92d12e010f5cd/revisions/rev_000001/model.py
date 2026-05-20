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


CASE_LENGTH = 0.42
CASE_WIDTH = 0.30
LOWER_HEIGHT = 0.095
LID_HEIGHT = 0.075
WALL = 0.010
CORNER_RADIUS = 0.032

HINGE_X = -0.018
HINGE_Z = 0.012
FRONT_X = CASE_LENGTH
LID_LOCAL_FRONT_X = FRONT_X - HINGE_X


def _case_shell(*, z_low: float, z_high: float, open_down: bool) -> cq.Workplane:
    """CadQuery shell for one hard-case half in the lower-shell frame."""
    height = z_high - z_low
    outer = (
        cq.Workplane("XY")
        .box(CASE_LENGTH, CASE_WIDTH, height, centered=(True, True, True))
        .translate((CASE_LENGTH / 2.0, 0.0, (z_low + z_high) / 2.0))
        .edges("|Z")
        .fillet(CORNER_RADIUS)
    )

    if open_down:
        inner_z_low = z_low - 0.004
        inner_z_high = z_high - WALL
    else:
        inner_z_low = z_low + WALL
        inner_z_high = z_high + 0.004

    inner = (
        cq.Workplane("XY")
        .box(
            CASE_LENGTH - 2.0 * WALL,
            CASE_WIDTH - 2.0 * WALL,
            inner_z_high - inner_z_low,
            centered=(True, True, True),
        )
        .translate((CASE_LENGTH / 2.0, 0.0, (inner_z_low + inner_z_high) / 2.0))
    )
    return outer.cut(inner)


def _lid_shell_local() -> cq.Workplane:
    """Lid shell authored in the lid part frame whose origin is the hinge pin."""
    return _case_shell(z_low=0.008, z_high=0.008 + LID_HEIGHT, open_down=True).translate(
        (-HINGE_X, 0.0, -HINGE_Z)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_instrument_case")

    graphite = model.material("graphite_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    dark = model.material("black_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    latch_mat = model.material("black_latch_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    metal = model.material("dull_steel", rgba=(0.58, 0.56, 0.52, 1.0))
    label = model.material("recessed_label", rgba=(0.03, 0.035, 0.04, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_cadquery(_case_shell(z_low=-LOWER_HEIGHT, z_high=0.0, open_down=False), "lower_shell"),
        material=graphite,
        name="lower_shell",
    )

    # A slightly proud rubber gasket makes the split line and hollow rim legible.
    for name, xyz, size in (
        ("front_gasket", (FRONT_X - 0.007, 0.0, 0.003), (0.014, CASE_WIDTH - 0.060, 0.006)),
        ("rear_gasket", (0.009, 0.0, 0.003), (0.018, CASE_WIDTH - 0.070, 0.006)),
        ("side_gasket_0", (CASE_LENGTH / 2.0, -CASE_WIDTH / 2.0 + 0.008, 0.003), (CASE_LENGTH - 0.060, 0.016, 0.006)),
        ("side_gasket_1", (CASE_LENGTH / 2.0, CASE_WIDTH / 2.0 - 0.008, 0.003), (CASE_LENGTH - 0.060, 0.016, 0.006)),
    ):
        lower.visual(Box(size), origin=Origin(xyz=xyz), material=dark, name=name)

    lower.visual(
        Box((0.007, 0.190, 0.040)),
        origin=Origin(xyz=(FRONT_X + 0.0025, 0.0, -0.040)),
        material=label,
        name="flat_front_panel",
    )

    # Fixed rear hinge knuckles on the lower shell: two end barrels with leaves.
    hinge_radius = 0.010
    for idx, y in enumerate((-0.108, 0.108)):
        lower.visual(
            Cylinder(radius=hinge_radius, length=0.060),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"rear_hinge_barrel_{idx}",
        )
        lower.visual(
            Box((0.034, 0.060, 0.008)),
            origin=Origin(xyz=(-0.004, y, 0.004)),
            material=metal,
            name=f"rear_hinge_leaf_{idx}",
        )

    latch_y_positions = (-0.082, 0.082)
    latch_pivot_z = -0.045
    for latch_idx, y in enumerate(latch_y_positions):
        for side_idx, side in enumerate((-1.0, 1.0)):
            lower.visual(
                Box((0.020, 0.010, 0.036)),
                origin=Origin(xyz=(FRONT_X + 0.008, y + side * 0.034, latch_pivot_z)),
                material=graphite,
                name=f"latch_cheek_{latch_idx}_{side_idx}",
            )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_local(), "lid_shell"),
        material=graphite,
        name="lid_shell",
    )
    # Raised protective ribs on the lid identify it as a rugged molded case.
    for idx, y in enumerate((-0.085, 0.085)):
        lid.visual(
            Box((0.285, 0.024, 0.009)),
            origin=Origin(xyz=(0.245, y, 0.075)),
            material=dark,
            name=f"lid_top_rib_{idx}",
        )
    # Moving center hinge barrel and leaf tied to the lid.
    lid.visual(
        Cylinder(radius=hinge_radius * 0.94, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.034, 0.060, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, 0.004)),
        material=metal,
        name="lid_hinge_leaf",
    )
    for idx, y in enumerate(latch_y_positions):
        lid.visual(
            Box((0.010, 0.068, 0.026)),
            origin=Origin(xyz=(LID_LOCAL_FRONT_X + 0.004, y, 0.011)),
            material=metal,
            name=f"latch_keeper_{idx}",
        )

    lid_joint = model.articulation(
        "lower_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.35),
    )
    lid_joint.meta["description"] = "Rear hinge line for the case lid."

    for idx, y in enumerate(latch_y_positions):
        latch = model.part(f"latch_{idx}")
        latch.visual(
            Cylinder(radius=0.009, length=0.052),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pivot_sleeve",
        )
        latch.visual(
            Box((0.012, 0.052, 0.080)),
            origin=Origin(xyz=(0.006, 0.0, 0.037)),
            material=latch_mat,
            name="latch_arm",
        )
        latch.visual(
            Box((0.018, 0.052, 0.010)),
            origin=Origin(xyz=(0.002, 0.0, 0.080)),
            material=latch_mat,
            name="upper_hook",
        )
        latch.visual(
            Box((0.006, 0.030, 0.026)),
            origin=Origin(xyz=(0.014, 0.0, 0.036)),
            material=dark,
            name="thumb_pad",
        )
        model.articulation(
            f"latch_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=latch,
            origin=Origin(xyz=(FRONT_X + 0.016, y, latch_pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=5.0, lower=0.0, upper=0.95),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("lower_to_lid")

    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="lower_shell",
        min_gap=0.004,
        max_gap=0.012,
        name="closed lid sits just above lower shell rim",
    )
    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        elem_a="lid_shell",
        elem_b="lower_shell",
        min_overlap=0.24,
        name="lid footprint covers lower shell",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.15}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "rear hinge swings lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    for idx in (0, 1):
        latch = object_model.get_part(f"latch_{idx}")
        pivot = object_model.get_articulation(f"latch_pivot_{idx}")
        closed = ctx.part_element_world_aabb(latch, elem="latch_arm")
        with ctx.pose({pivot: 0.70}):
            rotated = ctx.part_element_world_aabb(latch, elem="latch_arm")
        ctx.check(
            f"latch {idx} rotates outward from front face",
            closed is not None and rotated is not None and rotated[1][0] > closed[1][0] + 0.015,
            details=f"closed={closed}, rotated={rotated}",
        )

    return ctx.report()


object_model = build_object_model()
