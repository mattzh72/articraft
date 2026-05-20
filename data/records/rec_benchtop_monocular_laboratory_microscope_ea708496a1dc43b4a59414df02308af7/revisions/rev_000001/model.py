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


BASE_WIDTH = 0.230
BASE_DEPTH = 0.185
BASE_HEIGHT = 0.022
COLUMN_RADIUS = 0.013
COLUMN_Y = 0.034
BODY_JOINT_Z = 0.286
STAGE_CENTER = (0.0, -0.010, 0.184)
STAGE_RAIL_TOP_Z = 0.009
TUBE_ANGLE = math.radians(32.0)


def _base_foot_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)
    foot = foot.edges("|Z").fillet(0.016)
    pedestal = cq.Workplane("XY").box(0.095, 0.080, 0.022).translate((0.0, 0.040, 0.022))
    return foot.union(pedestal)


def _arm_shape() -> cq.Workplane:
    rear_spine = cq.Workplane("XY").box(0.040, 0.020, 0.260).translate((0.0, 0.079, 0.153))
    shoulder = cq.Workplane("XY").box(0.056, 0.022, 0.080).translate((0.0, 0.078, 0.300))
    stage_support = cq.Workplane("XY").box(0.026, 0.024, 0.022).translate((-0.050, 0.072, 0.180))
    stage_bridge = cq.Workplane("XY").box(0.038, 0.010, 0.014).translate((-0.025, 0.079, 0.180))
    return rear_spine.union(shoulder).union(stage_support).union(stage_bridge)


def _body_sleeve_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.034)
        .circle(COLUMN_RADIUS + 0.003)
        .extrude(0.115)
        .translate((0.0, 0.0, -0.055))
    )


def _body_head_shape() -> cq.Workplane:
    head = cq.Workplane("XY").box(0.060, 0.050, 0.032).translate((0.0, -0.050, 0.030))
    head = head.edges("|Z").fillet(0.006)
    side_neck = cq.Workplane("XY").box(0.012, 0.054, 0.028).translate((0.027, -0.026, 0.016))
    return head.union(side_neck)


def _stage_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.118, 0.106, 0.008)
    plate = plate.edges("|Z").fillet(0.004)
    plate = plate.faces(">Z").workplane().circle(0.016).cutThruAll()
    plate = plate.faces(">Z").workplane().center(0.0, 0.041).circle(0.022).cutThruAll()
    right_bracket = cq.Workplane("XY").box(0.018, 0.050, 0.020).translate((0.068, 0.010, 0.004))
    rail_front = cq.Workplane("XY").box(0.050, 0.008, 0.005).translate((0.040, -0.024, 0.0065))
    rail_rear = cq.Workplane("XY").box(0.050, 0.008, 0.005).translate((0.040, 0.024, 0.0065))
    return plate.union(right_bracket).union(rail_front).union(rail_rear)


def _carriage_shape() -> cq.Workplane:
    carriage_block = cq.Workplane("XY").box(0.030, 0.056, 0.014).translate((0.019, 0.0, 0.007))
    carriage_block = carriage_block.edges("|Z").fillet(0.003)
    clamp_bar = cq.Workplane("XY").box(0.030, 0.006, 0.010).translate((0.015, 0.026, 0.015))
    clamp_post = cq.Workplane("XY").box(0.008, 0.050, 0.018).translate((0.030, 0.000, 0.013))
    return carriage_block.union(clamp_bar).union(clamp_post)


def _outer_stage_knob_shape() -> cq.Workplane:
    body = cq.Workplane("YZ").circle(0.0165).circle(0.0065).extrude(0.010)
    rim = cq.Workplane("YZ").circle(0.018).circle(0.0065).extrude(0.002).translate((0.008, 0.0, 0.0))
    return body.union(rim)


def _inner_stage_knob_shape() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(0.0038).extrude(0.018)
    body = cq.Workplane("YZ").circle(0.009).extrude(0.010).translate((0.018, 0.0, 0.0))
    cap = cq.Workplane("YZ").circle(0.0105).extrude(0.002).translate((0.026, 0.0, 0.0))
    return shaft.union(body).union(cap)


def _tube_center(base_point: tuple[float, float, float], length: float) -> tuple[float, float, float]:
    axis = (0.0, math.sin(TUBE_ANGLE), math.cos(TUBE_ANGLE))
    return (
        base_point[0] + axis[0] * (length * 0.5),
        base_point[1] + axis[1] * (length * 0.5),
        base_point[2] + axis[2] * (length * 0.5),
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        (mins[0] + maxs[0]) * 0.5,
        (mins[1] + maxs[1]) * 0.5,
        (mins[2] + maxs[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_microscope")

    cast_white = model.material("cast_white", rgba=(0.93, 0.94, 0.92, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.74, 0.75, 0.77, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_foot_shape(), "microscope_base_foot"),
        material=cast_white,
        name="foot",
    )
    base.visual(
        Box((0.040, 0.020, 0.260)),
        origin=Origin(xyz=(0.0, 0.079, 0.153)),
        material=cast_white,
        name="spine",
    )
    base.visual(
        Box((0.056, 0.022, 0.080)),
        origin=Origin(xyz=(0.0, 0.078, 0.300)),
        material=cast_white,
        name="shoulder",
    )
    base.visual(
        Box((0.028, 0.010, 0.014)),
        origin=Origin(xyz=(-0.032, 0.079, 0.180)),
        material=cast_white,
        name="arm_bridge",
    )
    base.visual(
        Box((0.020, 0.024, 0.022)),
        origin=Origin(xyz=(-0.050, 0.072, 0.180)),
        material=cast_white,
        name="stage_support",
    )
    base.visual(
        Cylinder(radius=COLUMN_RADIUS, length=0.400),
        origin=Origin(xyz=(0.0, COLUMN_Y, 0.218)),
        material=steel,
        name="column",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_sleeve_shape(), "microscope_body_sleeve"),
        material=warm_gray,
        name="sleeve",
    )
    body.visual(
        mesh_from_cadquery(_body_head_shape(), "microscope_body_head"),
        material=cast_white,
        name="head",
    )
    body.visual(
        Box((0.012, 0.028, 0.022)),
        origin=Origin(xyz=(0.028, -0.006, 0.020)),
        material=cast_white,
        name="head_bridge",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, -0.044, 0.008)),
        material=dark_steel,
        name="nose_mount",
    )
    tube_base = (0.032, -0.056, 0.058)
    tube_length = 0.110
    tube_center = _tube_center(tube_base, tube_length)
    body.visual(
        Cylinder(radius=0.015, length=tube_length),
        origin=Origin(xyz=tube_center, rpy=(-TUBE_ANGLE, 0.0, 0.0)),
        material=charcoal,
        name="tube",
    )
    eyepiece_length = 0.045
    eyepiece_base = _tube_center(tube_base, tube_length * 2.0)
    eyepiece_center = _tube_center(eyepiece_base, eyepiece_length)
    body.visual(
        Cylinder(radius=0.012, length=eyepiece_length),
        origin=Origin(xyz=eyepiece_center, rpy=(-TUBE_ANGLE, 0.0, 0.0)),
        material=black,
        name="eyepiece",
    )
    eyecup_length = 0.018
    eyecup_base = _tube_center(eyepiece_base, eyepiece_length * 2.0)
    eyecup_center = _tube_center(eyecup_base, eyecup_length)
    body.visual(
        Cylinder(radius=0.0135, length=eyecup_length),
        origin=Origin(xyz=eyecup_center, rpy=(-TUBE_ANGLE, 0.0, 0.0)),
        material=black,
        name="eyecup",
    )
    body.visual(
        Box((0.014, 0.018, 0.010)),
        origin=Origin(xyz=(0.028, -0.048, 0.051)),
        material=charcoal,
        name="tube_mount",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=dark_steel,
        name="turret_cap",
    )
    turret.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="nosepiece",
    )
    objective_specs = (
        ("objective_long", (0.018, 0.000, -0.031), 0.0065, 0.044),
        ("objective_mid", (-0.009, 0.0156, -0.025), 0.0060, 0.032),
        ("objective_short", (-0.009, -0.0156, -0.021), 0.0054, 0.024),
    )
    for name, xyz, radius, length in objective_specs:
        turret.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz),
            material=steel,
            name=name,
        )
        turret.visual(
            Cylinder(radius=radius * 0.78, length=0.014),
            origin=Origin(xyz=(xyz[0], xyz[1], xyz[2] - length * 0.5 - 0.006)),
            material=black,
            name=f"{name}_tip",
        )

    stage = model.part("stage")
    stage.visual(
        mesh_from_cadquery(_stage_plate_shape(), "microscope_stage_plate"),
        material=charcoal,
        name="plate",
    )
    stage.visual(
        Box((0.028, 0.040, 0.022)),
        origin=Origin(xyz=(-0.038, 0.050, -0.004)),
        material=warm_gray,
        name="mount",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "microscope_stage_carriage"),
        material=dark_steel,
        name="carriage_plate",
    )

    stage_knob = model.part("stage_knob")
    stage_knob.visual(
        mesh_from_cadquery(_outer_stage_knob_shape(), "microscope_stage_knob"),
        material=black,
        name="knob",
    )

    stage_trim = model.part("stage_trim")
    stage_trim.visual(
        mesh_from_cadquery(_inner_stage_knob_shape(), "microscope_stage_trim"),
        material=black,
        name="knob",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.PRISMATIC,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, COLUMN_Y, BODY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.05, lower=0.0, upper=0.055),
    )
    model.articulation(
        "body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turret,
        origin=Origin(xyz=(0.0, -0.078, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )
    model.articulation(
        "base_to_stage",
        ArticulationType.FIXED,
        parent=base,
        child=stage,
        origin=Origin(xyz=STAGE_CENTER),
    )
    model.articulation(
        "stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=carriage,
        origin=Origin(xyz=(0.036, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.03, lower=-0.015, upper=0.015),
    )
    model.articulation(
        "stage_to_knob",
        ArticulationType.CONTINUOUS,
        parent=stage,
        child=stage_knob,
        origin=Origin(xyz=(0.077, 0.010, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=5.0),
    )
    model.articulation(
        "stage_to_trim",
        ArticulationType.CONTINUOUS,
        parent=stage,
        child=stage_trim,
        origin=Origin(xyz=(0.077, 0.010, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    turret = object_model.get_part("turret")
    stage = object_model.get_part("stage")
    carriage = object_model.get_part("carriage")
    stage_knob = object_model.get_part("stage_knob")
    stage_trim = object_model.get_part("stage_trim")

    focus = object_model.get_articulation("base_to_body")
    turret_spin = object_model.get_articulation("body_to_turret")
    carriage_slide = object_model.get_articulation("stage_to_carriage")

    focus_limits = focus.motion_limits
    carriage_limits = carriage_slide.motion_limits

    ctx.expect_gap(
        stage,
        base,
        axis="z",
        positive_elem="plate",
        negative_elem="foot",
        min_gap=0.140,
        name="mechanical stage sits clearly above the benchtop base",
    )

    lower_focus = 0.0 if focus_limits is None or focus_limits.lower is None else focus_limits.lower
    upper_focus = 0.055 if focus_limits is None or focus_limits.upper is None else focus_limits.upper

    with ctx.pose({focus: lower_focus}):
        ctx.expect_gap(
            turret,
            stage,
            axis="z",
            positive_elem="objective_long",
            negative_elem="plate",
            min_gap=0.018,
            name="lowest focus pose keeps the objective clear of the stage",
        )
        body_low = ctx.part_world_position(body)

    with ctx.pose({focus: upper_focus}):
        ctx.expect_gap(
            turret,
            stage,
            axis="z",
            positive_elem="objective_long",
            negative_elem="plate",
            min_gap=0.060,
            name="raised focus pose lifts the objective well above the stage",
        )
        body_high = ctx.part_world_position(body)

    ctx.check(
        "optical body slides upward on the focus column",
        body_low is not None and body_high is not None and body_high[2] > body_low[2] + 0.040,
        details=f"lower={body_low}, upper={body_high}",
    )

    lower_slide = -0.020 if carriage_limits is None or carriage_limits.lower is None else carriage_limits.lower
    upper_slide = 0.020 if carriage_limits is None or carriage_limits.upper is None else carriage_limits.upper

    with ctx.pose({carriage_slide: lower_slide}):
        ctx.expect_within(
            carriage,
            stage,
            axes="y",
            inner_elem="carriage_plate",
            outer_elem="plate",
            margin=0.006,
            name="carriage stays captured between the stage guide rails at minimum travel",
        )
        ctx.expect_overlap(
            carriage,
            stage,
            axes="x",
            elem_a="carriage_plate",
            elem_b="plate",
            min_overlap=0.024,
            name="carriage remains inserted in the stage guide at minimum travel",
        )
        carriage_low = ctx.part_world_position(carriage)

    with ctx.pose({carriage_slide: upper_slide}):
        ctx.expect_within(
            carriage,
            stage,
            axes="y",
            inner_elem="carriage_plate",
            outer_elem="plate",
            margin=0.006,
            name="carriage stays captured between the stage guide rails at maximum travel",
        )
        ctx.expect_overlap(
            carriage,
            stage,
            axes="x",
            elem_a="carriage_plate",
            elem_b="plate",
            min_overlap=0.024,
            name="carriage remains inserted in the stage guide at maximum travel",
        )
        carriage_high = ctx.part_world_position(carriage)

    ctx.check(
        "side carriage translates across the stage",
        carriage_low is not None and carriage_high is not None and carriage_high[0] > carriage_low[0] + 0.025,
        details=f"lower={carriage_low}, upper={carriage_high}",
    )

    outer_knob_center = _aabb_center(ctx.part_element_world_aabb(stage_knob, elem="knob"))
    inner_knob_center = _aabb_center(ctx.part_element_world_aabb(stage_trim, elem="knob"))
    ctx.check(
        "stage side knobs are coaxial",
        outer_knob_center is not None
        and inner_knob_center is not None
        and abs(outer_knob_center[1] - inner_knob_center[1]) < 0.001
        and abs(outer_knob_center[2] - inner_knob_center[2]) < 0.001,
        details=f"outer={outer_knob_center}, inner={inner_knob_center}",
    )
    ctx.check(
        "stage side knobs read as distinct stacked controls",
        outer_knob_center is not None
        and inner_knob_center is not None
        and inner_knob_center[0] > outer_knob_center[0] + 0.008,
        details=f"outer={outer_knob_center}, inner={inner_knob_center}",
    )

    with ctx.pose({turret_spin: 0.0}):
        turret_0 = _aabb_center(ctx.part_element_world_aabb(turret, elem="objective_long"))
    with ctx.pose({turret_spin: math.pi / 2.0}):
        turret_90 = _aabb_center(ctx.part_element_world_aabb(turret, elem="objective_long"))

    ctx.check(
        "objective turret rotates around the nosepiece axis",
        turret_0 is not None
        and turret_90 is not None
        and abs(turret_90[0] - turret_0[0]) > 0.010
        and abs(turret_90[1] - turret_0[1]) > 0.010,
        details=f"q0={turret_0}, q90={turret_90}",
    )

    return ctx.report()


object_model = build_object_model()
