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
    model = ArticulatedObject(name="pickup_tailgate_ramp")

    body_paint = model.material("deep_blue_paint", rgba=(0.02, 0.08, 0.18, 1.0))
    liner = model.material("black_bed_liner", rgba=(0.015, 0.015, 0.014, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.026, 0.024, 1.0))
    steel = model.material("brushed_steel", rgba=(0.48, 0.50, 0.50, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    red_lens = model.material("red_lens", rgba=(0.75, 0.04, 0.03, 1.0))

    bed = model.part("bed_frame")
    bed.visual(
        Box((1.10, 1.72, 0.08)),
        origin=Origin(xyz=(0.55, 0.0, 0.72)),
        material=liner,
        name="bed_floor",
    )
    bed.visual(
        Box((1.10, 0.08, 0.56)),
        origin=Origin(xyz=(0.55, 0.90, 1.04)),
        material=body_paint,
        name="side_wall_0",
    )
    bed.visual(
        Box((1.10, 0.08, 0.56)),
        origin=Origin(xyz=(0.55, -0.90, 1.04)),
        material=body_paint,
        name="side_wall_1",
    )
    bed.visual(
        Box((0.12, 0.10, 0.76)),
        origin=Origin(xyz=(-0.02, 0.89, 1.14)),
        material=body_paint,
        name="rear_post_0",
    )
    bed.visual(
        Box((0.12, 0.10, 0.76)),
        origin=Origin(xyz=(-0.02, -0.89, 1.14)),
        material=body_paint,
        name="rear_post_1",
    )
    bed.visual(
        Box((0.16, 1.88, 0.12)),
        origin=Origin(xyz=(-0.20, 0.0, 0.50)),
        material=steel,
        name="rear_bumper",
    )
    bed.visual(
        Box((0.08, 0.11, 0.22)),
        origin=Origin(xyz=(-0.13, 0.54, 0.62)),
        material=steel,
        name="bumper_bracket_0",
    )
    bed.visual(
        Box((0.08, 0.11, 0.22)),
        origin=Origin(xyz=(-0.13, -0.54, 0.62)),
        material=steel,
        name="bumper_bracket_1",
    )
    bed.visual(
        Box((0.09, 1.74, 0.06)),
        origin=Origin(xyz=(-0.015, 0.0, 0.704)),
        material=steel,
        name="hinge_sill",
    )
    bed.visual(
        Box((0.25, 0.08, 0.08)),
        origin=Origin(xyz=(-0.055, 0.60, 0.70)),
        material=steel,
        name="bumper_arm_0",
    )
    bed.visual(
        Box((0.25, 0.08, 0.08)),
        origin=Origin(xyz=(-0.055, -0.60, 0.70)),
        material=steel,
        name="bumper_arm_1",
    )
    for idx, (y, length) in enumerate(((-0.685, 0.34), (0.0, 0.35), (0.685, 0.34))):
        bed.visual(
            Cylinder(radius=0.026, length=length),
            origin=Origin(xyz=(-0.04, y, 0.76), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"bed_hinge_knuckle_{idx}",
        )
    bed.visual(
        Box((0.02, 0.055, 0.30)),
        origin=Origin(xyz=(-0.082, 0.89, 1.22)),
        material=red_lens,
        name="tail_lamp_0",
    )
    bed.visual(
        Box((0.02, 0.055, 0.30)),
        origin=Origin(xyz=(-0.082, -0.89, 1.22)),
        material=red_lens,
        name="tail_lamp_1",
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.070, 1.62, 0.710)),
        origin=Origin(xyz=(0.005, 0.0, 0.385)),
        material=body_paint,
        name="main_panel",
    )
    tailgate.visual(
        Box((0.008, 1.46, 0.550)),
        origin=Origin(xyz=(0.044, 0.0, 0.360)),
        material=liner,
        name="inner_liner",
    )
    tailgate.visual(
        Box((0.012, 1.66, 0.055)),
        origin=Origin(xyz=(-0.001, 0.0, 0.745)),
        material=body_paint,
        name="top_cap",
    )
    tailgate.visual(
        Box((0.018, 0.30, 0.045)),
        origin=Origin(xyz=(-0.037, 0.0, 0.520)),
        material=dark_plastic,
        name="outer_handle",
    )
    tailgate.visual(
        Box((0.006, 1.18, 0.035)),
        origin=Origin(xyz=(0.043, 0.0, 0.690)),
        material=steel,
        name="ramp_pocket_top",
    )
    tailgate.visual(
        Box((0.006, 1.18, 0.035)),
        origin=Origin(xyz=(0.043, 0.0, 0.060)),
        material=steel,
        name="ramp_pocket_bottom",
    )
    tailgate.visual(
        Box((0.006, 0.035, 0.620)),
        origin=Origin(xyz=(0.043, 0.515, 0.375)),
        material=steel,
        name="ramp_pocket_side_0",
    )
    tailgate.visual(
        Box((0.006, 0.035, 0.620)),
        origin=Origin(xyz=(0.043, -0.515, 0.375)),
        material=steel,
        name="ramp_pocket_side_1",
    )
    for idx, y in enumerate((-0.345, 0.345)):
        tailgate.visual(
            Cylinder(radius=0.025, length=0.34),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"tailgate_hinge_knuckle_{idx}",
        )
        tailgate.visual(
            Box((0.055, 0.30, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.034)),
            material=steel,
            name=f"tailgate_hinge_leaf_{idx}",
        )
    for idx, y in enumerate((-0.27, 0.27)):
        tailgate.visual(
            Box((0.016, 0.20, 0.026)),
            origin=Origin(xyz=(0.048, y, 0.682)),
            material=steel,
            name=f"ramp_hinge_saddle_{idx}",
        )
        tailgate.visual(
            Cylinder(radius=0.014, length=0.20),
            origin=Origin(xyz=(0.060, y, 0.683), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"ramp_hinge_socket_{idx}",
        )

    ramp = model.part("ramp_panel")
    ramp.visual(
        Box((0.024, 0.92, 0.530)),
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
        material=steel,
        name="ramp_deck",
    )
    ramp.visual(
        Box((0.035, 0.045, 0.540)),
        origin=Origin(xyz=(0.018, 0.445, -0.320)),
        material=steel,
        name="side_rail_0",
    )
    ramp.visual(
        Box((0.035, 0.045, 0.540)),
        origin=Origin(xyz=(0.018, -0.445, -0.320)),
        material=steel,
        name="side_rail_1",
    )
    for idx, z in enumerate((-0.110, -0.245, -0.380, -0.515)):
        ramp.visual(
            Box((0.036, 0.84, 0.028)),
            origin=Origin(xyz=(0.026, 0.0, z)),
            material=steel,
            name=f"cross_rib_{idx}",
        )
    for idx, y in enumerate((-0.25, 0.0, 0.25)):
        ramp.visual(
            Box((0.010, 0.060, 0.460)),
            origin=Origin(xyz=(0.040, y, -0.315)),
            material=rubber,
            name=f"traction_strip_{idx}",
        )
    for idx, y in enumerate((-0.09, 0.09)):
        ramp.visual(
            Box((0.024, 0.10, 0.060)),
            origin=Origin(xyz=(0.006, y, -0.025)),
            material=steel,
            name=f"hinge_tab_{idx}",
        )
    ramp.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="ramp_hinge_barrel",
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(-0.04, 0.0, 0.76)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=280.0, velocity=1.2, lower=0.0, upper=math.radians(92.0)),
    )
    model.articulation(
        "ramp_hinge",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=ramp,
        origin=Origin(xyz=(0.060, 0.0, 0.683)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    ramp = object_model.get_part("ramp_panel")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")
    ramp_hinge = object_model.get_articulation("ramp_hinge")

    def _coord(vec, idx: int) -> float:
        try:
            return float(vec[idx])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[idx])

    ctx.expect_within(
        ramp,
        tailgate,
        axes="yz",
        inner_elem="ramp_deck",
        outer_elem="main_panel",
        margin=0.0,
        name="stowed ramp is contained inside the tailgate outline",
    )
    ctx.expect_gap(
        ramp,
        tailgate,
        axis="x",
        positive_elem="ramp_deck",
        negative_elem="main_panel",
        min_gap=0.004,
        max_gap=0.020,
        name="stowed ramp sits proud of the inner tailgate face",
    )
    ctx.expect_gap(
        bed,
        tailgate,
        axis="x",
        positive_elem="bed_floor",
        negative_elem="main_panel",
        min_gap=0.0,
        max_gap=0.003,
        name="closed tailgate meets the bed floor line",
    )

    with ctx.pose({tailgate_hinge: math.radians(90.0), ramp_hinge: 0.0}):
        panel_box = ctx.part_element_world_aabb(tailgate, elem="main_panel")
        ok = panel_box is not None and _coord(panel_box[0], 0) < -0.70 and _coord(panel_box[1], 2) < 0.86
        ctx.check(
            "tailgate folds down to a horizontal loading surface",
            ok,
            details=f"main_panel_aabb={panel_box}",
        )

    with ctx.pose({tailgate_hinge: math.radians(90.0), ramp_hinge: math.pi}):
        tailgate_box = ctx.part_element_world_aabb(tailgate, elem="main_panel")
        ramp_box = ctx.part_element_world_aabb(ramp, elem="ramp_deck")
        ok = (
            tailgate_box is not None
            and ramp_box is not None
            and _coord(ramp_box[0], 0) < _coord(tailgate_box[0], 0) - 0.45
            and _coord(ramp_box[1], 2) < 0.86
        )
        ctx.check(
            "ramp flips beyond the tailgate free edge",
            ok,
            details=f"tailgate_aabb={tailgate_box}, ramp_aabb={ramp_box}",
        )

    return ctx.report()


object_model = build_object_model()
