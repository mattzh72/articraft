from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


PLATE_LEN = 0.82
PLATE_HEIGHT = 0.46
PLATE_THICKNESS = 0.02
FOOT_DEPTH = 0.12
FOOT_THICKNESS = 0.02

RAIL_LEN = 0.56
RAIL_DEPTH = 0.024
RAIL_HEIGHT = 0.024
RAIL_CENTER_Y = (PLATE_THICKNESS / 2.0) + (RAIL_DEPTH / 2.0)
RAIL_CENTER_Z = 0.26

SLIDE_HOME_X = -0.18
SLIDE_STROKE = 0.30

ARM_JOINT_Y = 0.126
ARM_JOINT_Z = 0.015
ARM_RAISED_TEST = 1.0


def _build_side_plate_shell() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        PLATE_LEN,
        PLATE_THICKNESS,
        PLATE_HEIGHT,
        centered=(True, True, False),
    )
    foot = (
        cq.Workplane("XY")
        .box(
            PLATE_LEN * 0.94,
            FOOT_DEPTH,
            FOOT_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, (FOOT_DEPTH / 2.0) - (PLATE_THICKNESS / 2.0), 0.0))
    )
    rear_buttress = (
        cq.Workplane("XY")
        .box(0.10, 0.055, 0.13, centered=(True, True, False))
        .translate((-0.29, 0.0175, 0.0))
    )
    front_buttress = (
        cq.Workplane("XY")
        .box(0.12, 0.055, 0.15, centered=(True, True, False))
        .translate((0.28, 0.0175, 0.0))
    )
    shell = plate.union(foot).union(rear_buttress).union(front_buttress)

    window_cut = (
        cq.Workplane("XY")
        .box(0.44, PLATE_THICKNESS * 1.6, 0.18, centered=(True, True, False))
        .translate((-0.08, 0.0, 0.13))
    )
    shell = shell.cut(window_cut)
    return shell


def _build_carriage_shell() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(0.110, 0.034, 0.012).translate((0.0, 0.0, 0.018))
    riser = cq.Workplane("XY").box(0.048, 0.020, 0.060).translate((0.0, 0.026, 0.054))
    backbone = cq.Workplane("XY").box(0.030, 0.150, 0.050).translate((0.0, 0.105, 0.055))
    left_ear = cq.Workplane("XY").box(0.012, 0.040, 0.050).translate((-0.020, ARM_JOINT_Y, ARM_JOINT_Z))
    right_ear = cq.Workplane("XY").box(0.012, 0.040, 0.050).translate((0.020, ARM_JOINT_Y, ARM_JOINT_Z))
    nose_block = cq.Workplane("XY").box(0.026, 0.050, 0.018).translate((0.0, 0.160, 0.015))
    stop_pad = cq.Workplane("XY").box(0.022, 0.020, 0.010).translate((0.0, 0.170, 0.003))
    shell = shoe.union(riser).union(backbone).union(left_ear).union(right_ear).union(nose_block).union(stop_pad)
    return shell


def _build_arm_shape() -> cq.Workplane:
    barrel = cq.Workplane("YZ").circle(0.012).extrude(0.013, both=True)
    neck = cq.Workplane("XY").box(0.018, 0.050, 0.014).translate((0.0, 0.025, 0.0))
    beam = cq.Workplane("XY").box(0.018, 0.110, 0.014).translate((0.0, 0.105, 0.0))
    paddle = cq.Workplane("XY").box(0.050, 0.080, 0.018).translate((0.0, 0.205, 0.0))
    lip = cq.Workplane("XY").box(0.050, 0.020, 0.028).translate((0.0, 0.240, -0.005))

    arm = barrel.union(neck).union(beam).union(paddle).union(lip)
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_transfer_slide")
    model.material("plate_steel", rgba=(0.26, 0.29, 0.32, 1.0))
    model.material("guide_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_paint", rgba=(0.49, 0.52, 0.55, 1.0))
    model.material("arm_yellow", rgba=(0.88, 0.73, 0.12, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_build_side_plate_shell(), "side_plate_shell"),
        material="plate_steel",
        name="plate_shell",
    )
    side_plate.visual(
        Box((RAIL_LEN, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        material="guide_steel",
        name="guide_rail",
    )
    side_plate.inertial = Inertial.from_geometry(
        Box((PLATE_LEN, FOOT_DEPTH, PLATE_HEIGHT)),
        mass=32.0,
        origin=Origin(xyz=(0.0, (FOOT_DEPTH / 2.0) - (PLATE_THICKNESS / 2.0), PLATE_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.110, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, 0.018, 0.019)),
        material="guide_steel",
        name="shoe",
    )
    carriage.visual(
        Box((0.056, 0.026, 0.062)),
        origin=Origin(xyz=(0.0, 0.030, 0.050)),
        material="carriage_paint",
        name="tower",
    )
    carriage.visual(
        Box((0.060, 0.096, 0.038)),
        origin=Origin(xyz=(0.0, 0.086, 0.046)),
        material="carriage_paint",
        name="spine",
    )
    carriage.visual(
        Box((0.060, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.109, ARM_JOINT_Z)),
        material="carriage_paint",
        name="nose_plate",
    )
    carriage.visual(
        Box((0.012, 0.026, 0.042)),
        origin=Origin(xyz=(-0.021, 0.119, ARM_JOINT_Z)),
        material="carriage_paint",
        name="left_ear",
    )
    carriage.visual(
        Box((0.012, 0.026, 0.042)),
        origin=Origin(xyz=(0.021, 0.119, ARM_JOINT_Z)),
        material="carriage_paint",
        name="right_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.110, 0.134, 0.076)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.067, 0.038)),
    )

    flap_arm = model.part("flap_arm")
    flap_arm.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="guide_steel",
        name="hinge_barrel",
    )
    flap_arm.visual(
        Box((0.018, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material="arm_yellow",
        name="neck",
    )
    flap_arm.visual(
        Box((0.018, 0.100, 0.014)),
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        material="arm_yellow",
        name="beam",
    )
    flap_arm.visual(
        Box((0.050, 0.076, 0.018)),
        origin=Origin(xyz=(0.0, 0.174, 0.0)),
        material="arm_yellow",
        name="paddle",
    )
    flap_arm.visual(
        Box((0.050, 0.016, 0.028)),
        origin=Origin(xyz=(0.0, 0.212, -0.005)),
        material="arm_yellow",
        name="lip",
    )
    flap_arm.inertial = Inertial.from_geometry(
        Box((0.050, 0.228, 0.028)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.114, -0.002)),
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_STROKE,
            effort=650.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=flap_arm,
        origin=Origin(xyz=(0.0, ARM_JOINT_Y, ARM_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.15,
            effort=45.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    flap_arm = object_model.get_part("flap_arm")
    guide_slide = object_model.get_articulation("guide_slide")
    arm_hinge = object_model.get_articulation("carriage_to_arm")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "guide stage is prismatic along +x",
        guide_slide.articulation_type == ArticulationType.PRISMATIC and tuple(guide_slide.axis) == (1.0, 0.0, 0.0),
        details=f"guide_slide config={guide_slide.articulation_type} axis={guide_slide.axis}",
    )
    ctx.check(
        "flap arm is revolute about +x",
        arm_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(arm_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"carriage_to_arm config={arm_hinge.articulation_type} axis={arm_hinge.axis}",
    )

    with ctx.pose({guide_slide: 0.0, arm_hinge: 0.0}):
        ctx.expect_contact(
            carriage,
            side_plate,
            elem_a="shoe",
            elem_b="guide_rail",
            name="runner pad stays on guide rail at home",
        )
        ctx.expect_contact(
            flap_arm,
            carriage,
            name="arm barrel stays seated in carriage clevis at home",
        )
        ctx.expect_gap(
            flap_arm,
            side_plate,
            axis="y",
            min_gap=0.015,
            name="arm clears grounded side plate at home",
        )

    with ctx.pose({guide_slide: SLIDE_STROKE, arm_hinge: 0.0}):
        ctx.expect_contact(
            carriage,
            side_plate,
            elem_a="shoe",
            elem_b="guide_rail",
            name="runner pad stays on guide rail at full extension",
        )
        ctx.expect_contact(
            flap_arm,
            carriage,
            name="arm barrel stays seated in carriage clevis at full extension",
        )

    with ctx.pose({guide_slide: SLIDE_STROKE * 0.75, arm_hinge: ARM_RAISED_TEST}):
        ctx.expect_gap(
            flap_arm,
            side_plate,
            axis="y",
            min_gap=0.015,
            name="raised arm clears grounded side plate",
        )

    with ctx.pose({guide_slide: 0.0}):
        home_pos = ctx.part_world_position(carriage)
    with ctx.pose({guide_slide: SLIDE_STROKE}):
        extended_pos = ctx.part_world_position(carriage)
    slide_ok = (
        home_pos is not None
        and extended_pos is not None
        and abs((extended_pos[0] - home_pos[0]) - SLIDE_STROKE) < 1e-5
        and abs(extended_pos[1] - home_pos[1]) < 1e-6
        and abs(extended_pos[2] - home_pos[2]) < 1e-6
    )
    ctx.check(
        "carriage translates cleanly on slide axis",
        slide_ok,
        details=f"home={home_pos} extended={extended_pos}",
    )

    with ctx.pose({guide_slide: SLIDE_STROKE * 0.75, arm_hinge: 0.0}):
        arm_low = ctx.part_world_aabb(flap_arm)
    with ctx.pose({guide_slide: SLIDE_STROKE * 0.75, arm_hinge: ARM_RAISED_TEST}):
        arm_high = ctx.part_world_aabb(flap_arm)
    arm_lift_ok = (
        arm_low is not None
        and arm_high is not None
        and (arm_high[1][2] - arm_low[1][2]) > 0.10
        and (arm_high[0][2] - arm_low[0][2]) > -0.005
    )
    ctx.check(
        "flap arm lifts upward from the carriage nose",
        arm_lift_ok,
        details=f"low_aabb={arm_low} high_aabb={arm_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
