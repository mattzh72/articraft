from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="pickup_tailgate_step")

    paint = model.material("painted_blue", color=(0.05, 0.18, 0.34, 1.0))
    dark = model.material("black_textured", color=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("rubber_seal", color=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    bedliner = model.material("bedliner_gray", color=(0.12, 0.12, 0.12, 1.0))
    amber = model.material("red_marker", color=(0.65, 0.02, 0.015, 1.0))

    bed = model.part("bed")
    # A short rear pickup-bed surround gives the lower hinge axis a believable mount.
    bed.visual(Box((1.80, 0.18, 0.080)), origin=Origin(xyz=(0.0, 0.080, 0.680)), material=paint, name="rear_sill")
    bed.visual(Box((1.58, 0.82, 0.040)), origin=Origin(xyz=(0.0, 0.410, 0.730)), material=bedliner, name="bed_floor")
    bed.visual(Box((0.085, 0.86, 0.46)), origin=Origin(xyz=(-0.865, 0.380, 0.940)), material=paint, name="bed_side_0")
    bed.visual(Box((0.085, 0.86, 0.46)), origin=Origin(xyz=(0.865, 0.380, 0.940)), material=paint, name="bed_side_1")
    bed.visual(Box((1.78, 0.10, 0.090)), origin=Origin(xyz=(0.0, -0.135, 0.570)), material=steel, name="bumper")
    bed.visual(Box((0.10, 0.15, 0.13)), origin=Origin(xyz=(-0.60, -0.060, 0.625)), material=steel, name="bumper_bracket_0")
    bed.visual(Box((0.10, 0.15, 0.13)), origin=Origin(xyz=(0.60, -0.060, 0.625)), material=steel, name="bumper_bracket_1")

    hinge_rpy = (0.0, pi / 2.0, 0.0)
    for i, (x, length) in enumerate(((-0.70, 0.16), (-0.33, 0.13), (0.33, 0.13), (0.70, 0.16))):
        bed.visual(
            Cylinder(radius=0.024, length=length),
            origin=Origin(xyz=(x, -0.035, 0.720), rpy=hinge_rpy),
            material=steel,
            name=f"bed_hinge_knuckle_{i}",
        )
        bed.visual(
            Box((length, 0.065, 0.040)),
            origin=Origin(xyz=(x, -0.004, 0.706)),
            material=steel,
            name=f"bed_hinge_stand_{i}",
        )
    bed.visual(
        Cylinder(radius=0.010, length=1.52),
        origin=Origin(xyz=(0.0, -0.035, 0.720), rpy=hinge_rpy),
        material=steel,
        name="hinge_pin",
    )

    tailgate = model.part("tailgate")
    # Closed tailgate: local Z is vertical from the lower hinge axis; local -Y is outward/rearward.
    tailgate.visual(Box((1.55, 0.070, 0.280)), origin=Origin(xyz=(0.0, -0.025, 0.180)), material=paint, name="lower_skin")
    tailgate.visual(Box((1.55, 0.070, 0.050)), origin=Origin(xyz=(0.0, -0.025, 0.555)), material=paint, name="top_skin")
    tailgate.visual(Box((0.405, 0.070, 0.210)), origin=Origin(xyz=(-0.5725, -0.025, 0.425)), material=paint, name="side_skin_0")
    tailgate.visual(Box((0.405, 0.070, 0.210)), origin=Origin(xyz=(0.5725, -0.025, 0.425)), material=paint, name="side_skin_1")
    tailgate.visual(Box((1.55, 0.012, 0.030)), origin=Origin(xyz=(0.0, -0.066, 0.315)), material=dark, name="belt_crease")
    tailgate.visual(Box((1.42, 0.010, 0.018)), origin=Origin(xyz=(0.0, -0.067, 0.522)), material=rubber, name="upper_reveal")
    tailgate.visual(Box((0.018, 0.012, 0.215)), origin=Origin(xyz=(-0.382, -0.067, 0.425)), material=rubber, name="step_reveal_0")
    tailgate.visual(Box((0.018, 0.012, 0.215)), origin=Origin(xyz=(0.382, -0.067, 0.425)), material=rubber, name="step_reveal_1")
    tailgate.visual(Box((0.155, 0.010, 0.050)), origin=Origin(xyz=(-0.735, -0.062, 0.455)), material=amber, name="tail_lamp_0")
    tailgate.visual(Box((0.155, 0.010, 0.050)), origin=Origin(xyz=(0.735, -0.062, 0.455)), material=amber, name="tail_lamp_1")

    # Gate-side lower hinge knuckles are staggered between the bed-side knuckles.
    for i, (x, length) in enumerate(((-0.5075, 0.225), (0.0, 0.530), (0.5075, 0.225))):
        tailgate.visual(
            Cylinder(radius=0.022, length=length),
            origin=Origin(xyz=(x, -0.035, 0.000), rpy=hinge_rpy),
            material=steel,
            name=f"gate_hinge_knuckle_{i}",
        )
        tailgate.visual(
            Box((length, 0.045, 0.045)),
            origin=Origin(xyz=(x, -0.035, 0.032)),
            material=steel,
            name=f"gate_hinge_lug_{i}",
        )

    # The small step door hinge lives inside the upper tailgate skin.
    for i, x in enumerate((-0.405, 0.405)):
        tailgate.visual(
            Cylinder(radius=0.014, length=0.090),
            origin=Origin(xyz=(x, -0.057, 0.340), rpy=hinge_rpy),
            material=steel,
            name=f"step_hinge_knuckle_{i}",
        )
        tailgate.visual(
            Box((0.090, 0.024, 0.018)),
            origin=Origin(xyz=(x, -0.057, 0.326)),
            material=steel,
            name=f"step_hinge_boss_{i}",
        )

    step_panel = model.part("step_panel")
    step_panel.visual(Box((0.660, 0.026, 0.170)), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=dark, name="step_plate")
    step_panel.visual(
        Cylinder(radius=0.013, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=hinge_rpy),
        material=steel,
        name="step_hinge_barrel",
    )
    for i, z in enumerate((0.040, 0.083, 0.126)):
        step_panel.visual(Box((0.585, 0.009, 0.010)), origin=Origin(xyz=(0.0, -0.014, z)), material=rubber, name=f"tread_rib_{i}")
    step_panel.visual(Box((0.080, 0.010, 0.020)), origin=Origin(xyz=(0.0, -0.014, 0.155)), material=steel, name="finger_pull")

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, -0.035, 0.720)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.8, lower=0.0, upper=1.55),
    )

    model.articulation(
        "tailgate_to_step",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=step_panel,
        origin=Origin(xyz=(0.0, -0.057, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    tailgate = object_model.get_part("tailgate")
    step_panel = object_model.get_part("step_panel")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    step_hinge = object_model.get_articulation("tailgate_to_step")

    for i in range(3):
        ctx.allow_overlap(
            bed,
            tailgate,
            elem_a="hinge_pin",
            elem_b=f"gate_hinge_knuckle_{i}",
            reason="The fixed hinge pin intentionally passes through the tailgate-side hinge barrel.",
        )
        ctx.expect_overlap(
            bed,
            tailgate,
            axes="x",
            elem_a="hinge_pin",
            elem_b=f"gate_hinge_knuckle_{i}",
            min_overlap=0.10,
            name=f"hinge pin captures gate barrel {i}",
        )

    with ctx.pose({tailgate_hinge: 0.0, step_hinge: 0.0}):
        ctx.expect_within(
            step_panel,
            tailgate,
            axes="x",
            inner_elem="step_plate",
            outer_elem="top_skin",
            margin=0.0,
            name="step panel is centered within tailgate width",
        )
        ctx.expect_gap(
            tailgate,
            step_panel,
            axis="z",
            positive_elem="top_skin",
            negative_elem="step_plate",
            min_gap=0.005,
            max_gap=0.040,
            name="step panel clears upper tailgate skin",
        )
        ctx.expect_gap(
            step_panel,
            tailgate,
            axis="z",
            positive_elem="step_plate",
            negative_elem="lower_skin",
            min_gap=0.005,
            max_gap=0.040,
            name="fold-out step clears lower tailgate skin",
        )

    closed_top = ctx.part_element_world_aabb(tailgate, elem="top_skin")
    with ctx.pose({tailgate_hinge: 1.45}):
        opened_top = ctx.part_element_world_aabb(tailgate, elem="top_skin")
    ctx.check(
        "tailgate rotates rearward and downward",
        closed_top is not None
        and opened_top is not None
        and opened_top[0][1] < closed_top[0][1] - 0.45
        and opened_top[1][2] < closed_top[1][2] - 0.35,
        details=f"closed_top={closed_top}, opened_top={opened_top}",
    )

    closed_step = ctx.part_element_world_aabb(step_panel, elem="step_plate")
    with ctx.pose({step_hinge: 1.20}):
        opened_step = ctx.part_element_world_aabb(step_panel, elem="step_plate")
    ctx.check(
        "upper-skin step panel folds outward from its own hinge",
        closed_step is not None
        and opened_step is not None
        and opened_step[0][1] < closed_step[0][1] - 0.09
        and opened_step[1][2] < closed_step[1][2] - 0.07,
        details=f"closed_step={closed_step}, opened_step={opened_step}",
    )

    return ctx.report()


object_model = build_object_model()
