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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate")

    body_blue = model.material("deep_metallic_blue", rgba=(0.035, 0.12, 0.24, 1.0))
    shadow_blue = model.material("recessed_blue_shadow", rgba=(0.018, 0.055, 0.11, 1.0))
    bed_gray = model.material("bed_liner_dark_gray", rgba=(0.08, 0.085, 0.085, 1.0))
    black = model.material("black_latch_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.58, 0.55, 1.0))

    supports = model.part("bed_supports")
    # Simplified rear bed corners, tied together by a sill so the side supports
    # read as a single mounted truck-bed structure.
    supports.visual(
        Box((1.90, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.10, 0.40)),
        material=bed_gray,
        name="lower_sill",
    )
    for side, x in (("left", -0.88), ("right", 0.88)):
        supports.visual(
            Box((0.12, 0.38, 0.76)),
            origin=Origin(xyz=(x, 0.06, 0.80)),
            material=bed_gray,
            name=f"{side}_bed_post",
        )
        supports.visual(
            Box((0.12, 0.72, 0.08)),
            origin=Origin(xyz=(x, 0.39, 1.12)),
            material=bed_gray,
            name=f"{side}_bed_rail",
        )

    # Fixed hinge bushings and their welded brackets on the sill.  They alternate
    # with the moving tailgate knuckles so the closed pose has real clearance.
    for name, x, length in (
        ("left_outer_bushing", -0.82, 0.18),
        ("left_inner_bushing", -0.36, 0.18),
        ("right_inner_bushing", 0.36, 0.18),
        ("right_outer_bushing", 0.82, 0.18),
    ):
        supports.visual(
            Box((length + 0.04, 0.11, 0.07)),
            origin=Origin(xyz=(x, 0.015, 0.425)),
            material=steel,
            name=f"{name}_mount",
        )
        supports.visual(
            Cylinder(radius=0.030, length=length),
            origin=Origin(xyz=(x, -0.040, 0.450), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=name,
        )

    supports.visual(
        Box((0.030, 0.050, 0.090)),
        origin=Origin(xyz=(-0.817, -0.035, 0.885)),
        material=black,
        name="left_striker",
    )
    supports.visual(
        Box((0.030, 0.050, 0.090)),
        origin=Origin(xyz=(0.817, -0.035, 0.885)),
        material=black,
        name="right_striker",
    )

    tailgate = model.part("tailgate")
    # The child part frame is the bottom hinge line.  At q=0 the gate is
    # upright; positive rotation about +X drops the top edge outward and down.
    tailgate.visual(
        Box((1.56, 0.050, 0.555)),
        origin=Origin(xyz=(0.0, -0.015, 0.330)),
        material=body_blue,
        name="outer_skin",
    )
    tailgate.visual(
        Box((1.60, 0.120, 0.075)),
        origin=Origin(xyz=(0.0, 0.000, 0.615)),
        material=body_blue,
        name="top_cap",
    )
    tailgate.visual(
        Box((1.52, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, -0.015, 0.080)),
        material=body_blue,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((0.075, 0.080, 0.555)),
        origin=Origin(xyz=(-0.742, -0.010, 0.330)),
        material=body_blue,
        name="left_side_rail",
    )
    tailgate.visual(
        Box((0.075, 0.080, 0.555)),
        origin=Origin(xyz=(0.742, -0.010, 0.330)),
        material=body_blue,
        name="right_side_rail",
    )

    # Stamped rear outer panel: a darker recessed field with raised perimeter
    # beads and a shallow horizontal crease.
    tailgate.visual(
        Box((1.18, 0.010, 0.315)),
        origin=Origin(xyz=(0.0, -0.044, 0.345)),
        material=shadow_blue,
        name="outer_recess",
    )
    tailgate.visual(
        Box((1.27, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.052, 0.506)),
        material=body_blue,
        name="upper_stamp_bead",
    )
    tailgate.visual(
        Box((1.27, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.052, 0.184)),
        material=body_blue,
        name="lower_stamp_bead",
    )
    tailgate.visual(
        Box((0.042, 0.018, 0.345)),
        origin=Origin(xyz=(-0.600, -0.052, 0.345)),
        material=body_blue,
        name="left_stamp_bead",
    )
    tailgate.visual(
        Box((0.042, 0.018, 0.345)),
        origin=Origin(xyz=(0.600, -0.052, 0.345)),
        material=body_blue,
        name="right_stamp_bead",
    )
    tailgate.visual(
        Box((1.05, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -0.056, 0.345)),
        material=body_blue,
        name="center_crease",
    )

    # Bed-facing inner framed surface with raised perimeter and diagonal ribs.
    tailgate.visual(
        Box((1.34, 0.020, 0.385)),
        origin=Origin(xyz=(0.0, 0.018, 0.325)),
        material=bed_gray,
        name="inner_panel",
    )
    tailgate.visual(
        Box((1.38, 0.026, 0.038)),
        origin=Origin(xyz=(0.0, 0.030, 0.535)),
        material=bed_gray,
        name="inner_top_frame",
    )
    tailgate.visual(
        Box((1.38, 0.026, 0.038)),
        origin=Origin(xyz=(0.0, 0.030, 0.115)),
        material=bed_gray,
        name="inner_bottom_frame",
    )
    for side, x in (("left", -0.690), ("right", 0.690)):
        tailgate.visual(
            Box((0.040, 0.026, 0.420)),
            origin=Origin(xyz=(x, 0.030, 0.325)),
            material=bed_gray,
            name=f"{side}_inner_frame",
        )
    tailgate.visual(
        Box((0.050, 0.028, 0.420)),
        origin=Origin(xyz=(0.0, 0.033, 0.325)),
        material=bed_gray,
        name="center_inner_frame",
    )
    for name, x, angle in (
        ("left_diagonal_rib", -0.345, -0.57),
        ("right_diagonal_rib", 0.345, 0.57),
    ):
        tailgate.visual(
            Box((0.78, 0.026, 0.034)),
            origin=Origin(xyz=(x, 0.034, 0.325), rpy=(0.0, angle, 0.0)),
            material=bed_gray,
            name=name,
        )

    tailgate.visual(
        Box((0.040, 0.055, 0.115)),
        origin=Origin(xyz=(-0.770, 0.000, 0.435)),
        material=black,
        name="left_latch",
    )
    tailgate.visual(
        Box((0.040, 0.055, 0.115)),
        origin=Origin(xyz=(0.770, 0.000, 0.435)),
        material=black,
        name="right_latch",
    )

    for name, x, length in (
        ("left_hinge_knuckle", -0.600, 0.220),
        ("center_hinge_knuckle", 0.000, 0.450),
        ("right_hinge_knuckle", 0.600, 0.220),
    ):
        tailgate.visual(
            Box((length, 0.050, 0.040)),
            origin=Origin(xyz=(x, -0.015, 0.040)),
            material=steel,
            name=name.replace("knuckle", "leaf"),
        )
        tailgate.visual(
            Cylinder(radius=0.025, length=length),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=name,
        )

    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=supports,
        child=tailgate,
        origin=Origin(xyz=(0.0, -0.040, 0.450)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi / 2.0, effort=120.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    supports = object_model.get_part("bed_supports")
    tailgate = object_model.get_part("tailgate")
    hinge = object_model.get_articulation("hinge")

    limits = hinge.motion_limits
    ctx.check(
        "hinge folds tailgate ninety degrees",
        limits is not None
        and limits.lower == 0.0
        and abs((limits.upper or 0.0) - math.pi / 2.0) < 1.0e-6
        and tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"limits={limits}, axis={hinge.axis}",
    )

    # Closed latch points sit just inboard of the bedside strikers.
    ctx.expect_gap(
        tailgate,
        supports,
        axis="x",
        positive_elem="left_latch",
        negative_elem="left_striker",
        min_gap=0.001,
        max_gap=0.020,
        name="left latch has bedside clearance",
    )
    ctx.expect_gap(
        supports,
        tailgate,
        axis="x",
        positive_elem="right_striker",
        negative_elem="right_latch",
        min_gap=0.001,
        max_gap=0.020,
        name="right latch has bedside clearance",
    )
    ctx.expect_overlap(
        tailgate,
        supports,
        axes="yz",
        elem_a="left_latch",
        elem_b="left_striker",
        min_overlap=0.035,
        name="left latch aligns with striker",
    )
    ctx.expect_overlap(
        tailgate,
        supports,
        axes="yz",
        elem_a="right_latch",
        elem_b="right_striker",
        min_overlap=0.035,
        name="right latch aligns with striker",
    )

    # Hinge knuckles are coaxial with the fixed bushings while remaining
    # separated along X as alternating hinge leaves.
    ctx.expect_overlap(
        tailgate,
        supports,
        axes="yz",
        elem_a="center_hinge_knuckle",
        elem_b="left_inner_bushing",
        min_overlap=0.040,
        name="hinge knuckles share a bottom axis",
    )

    closed_top = ctx.part_element_world_aabb(tailgate, elem="top_cap")
    with ctx.pose({hinge: math.pi / 2.0}):
        open_gate = ctx.part_world_aabb(tailgate)
        open_top = ctx.part_element_world_aabb(tailgate, elem="top_cap")

    ctx.check(
        "open pose is horizontal and rearward",
        closed_top is not None
        and open_gate is not None
        and open_top is not None
        and closed_top[1][2] > 1.02
        and open_top[0][1] < -0.55
        and (open_gate[1][2] - open_gate[0][2]) < 0.16,
        details=f"closed_top={closed_top}, open_gate={open_gate}, open_top={open_top}",
    )

    return ctx.report()


object_model = build_object_model()
