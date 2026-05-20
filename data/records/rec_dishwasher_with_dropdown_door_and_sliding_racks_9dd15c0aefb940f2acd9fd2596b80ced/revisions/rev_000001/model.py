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


BODY_W = 0.62
BODY_D = 0.63
BODY_H = 0.82

CHAMBER_W = 0.54
CHAMBER_D = 0.59
CHAMBER_H = 0.48
CHAMBER_Y = 0.03
CHAMBER_BOTTOM = 0.13

POD_W = 0.36
POD_D = 0.075
POD_H = 0.11
POD_BOTTOM = 0.67
POD_FRONT_Y = BODY_D / 2.0

DOOR_W = 0.58
DOOR_H = 0.53
DOOR_T = 0.045
DOOR_HINGE_Z = 0.105
DOOR_LIP = 0.028
DOOR_FRONT_SKIN = 0.012
DOOR_INNER_PANEL_Y = DOOR_T - DOOR_FRONT_SKIN

FLAP_W = 0.11
FLAP_H = 0.075
FLAP_T = 0.012
FLAP_HINGE_Z = 0.365

GUIDE_W = 0.016
GUIDE_H = 0.008
GUIDE_LEN = 0.44
GUIDE_Y = 0.01
GUIDE_X = CHAMBER_W / 2.0 - GUIDE_W / 2.0

LOWER_GUIDE_Z = 0.216
UPPER_GUIDE_Z = 0.436

RUNNER_W = 0.012
RUNNER_H = 0.010
RUNNER_LEN = 0.43
RUNNER_X = 0.255
RUNNER_Y = -0.01

LOWER_RACK_W = 0.50
LOWER_RACK_D = 0.46
LOWER_RACK_H = 0.14
LOWER_RACK_Z = LOWER_GUIDE_Z + GUIDE_H / 2.0 + RUNNER_H / 2.0
LOWER_RACK_TRAVEL = 0.22

UPPER_RACK_W = 0.48
UPPER_RACK_D = 0.42
UPPER_RACK_H = 0.085
UPPER_RACK_Z = UPPER_GUIDE_Z + GUIDE_H / 2.0 + RUNNER_H / 2.0
UPPER_RACK_TRAVEL = 0.18

LOWER_ARM_Z = 0.156
UPPER_ARM_Z = 0.525

SWITCH_W = 0.026
SWITCH_D = 0.018
SWITCH_H = 0.036
SWITCH_CUT_W = 0.030
SWITCH_CUT_H = 0.042
SWITCH_Z = 0.055
SWITCH_0_X = -0.105
SWITCH_1_X = -0.055
SWITCH_TRAVEL = 0.32

KNOB_X = 0.11
KNOB_Z = 0.058


def add_box(
    shape: cq.Workplane | None,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    centered: tuple[bool, bool, bool] = (True, True, True),
) -> cq.Workplane:
    box = cq.Workplane("XY").box(*size, centered=centered).translate(xyz)
    return box if shape is None else shape.union(box)


def make_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    cavity = (
        cq.Workplane("XY")
        .box(CHAMBER_W, CHAMBER_D, CHAMBER_H, centered=(True, True, False))
        .translate((0.0, CHAMBER_Y, CHAMBER_BOTTOM))
    )
    return shell.cut(cavity)


def make_control_pod_shell() -> cq.Workplane:
    pod = cq.Workplane("XY").box(POD_W, POD_D, POD_H, centered=(True, False, False))
    pod = pod.edges("|Z").fillet(0.006)

    switch_0_cut = (
        cq.Workplane("XZ")
        .center(SWITCH_0_X, SWITCH_Z)
        .rect(SWITCH_CUT_W, SWITCH_CUT_H)
        .extrude(POD_D + 0.01)
        .translate((0.0, -0.005, 0.0))
    )
    switch_1_cut = (
        cq.Workplane("XZ")
        .center(SWITCH_1_X, SWITCH_Z)
        .rect(SWITCH_CUT_W, SWITCH_CUT_H)
        .extrude(POD_D + 0.01)
        .translate((0.0, -0.005, 0.0))
    )
    knob_cut = (
        cq.Workplane("XZ")
        .center(KNOB_X, KNOB_Z)
        .circle(0.016)
        .extrude(POD_D + 0.01)
        .translate((0.0, -0.005, 0.0))
    )

    return pod.cut(switch_0_cut).cut(switch_1_cut).cut(knob_cut)


def make_door_shell() -> cq.Workplane:
    door = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H, centered=(True, False, False))

    liner_cut = (
        cq.Workplane("XY")
        .box(
            DOOR_W - 2.0 * DOOR_LIP,
            DOOR_T - DOOR_FRONT_SKIN + 0.004,
            DOOR_H - 2.0 * DOOR_LIP,
            centered=(True, False, False),
        )
        .translate((0.0, -0.002, DOOR_LIP))
    )
    door = door.cut(liner_cut)

    outer_pull = (
        cq.Workplane("XY")
        .box(0.20, 0.018, 0.030, centered=(True, False, False))
        .translate((0.0, DOOR_T - 0.002, DOOR_H - 0.070))
    )
    lower_kick = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.06, 0.008, 0.085, centered=(True, False, False))
        .translate((0.0, DOOR_T - 0.001, 0.030))
    )
    return door.union(outer_pull).union(lower_kick)


def make_rack_mesh(width: float, depth: float, height: float) -> cq.Workplane:
    rail = 0.008
    overlap = 0.002
    shape: cq.Workplane | None = None
    half_w = width / 2.0
    half_d = depth / 2.0
    bottom_z = -height * 0.32
    top_z = bottom_z + height - rail
    post_height = height - rail

    shape = add_box(shape, (width + overlap, rail, rail), (0.0, half_d - rail / 2.0, bottom_z))
    shape = add_box(shape, (width + overlap, rail, rail), (0.0, -half_d + rail / 2.0, bottom_z))
    shape = add_box(shape, (rail, depth + overlap, rail), (half_w - rail / 2.0, 0.0, bottom_z))
    shape = add_box(shape, (rail, depth + overlap, rail), (-half_w + rail / 2.0, 0.0, bottom_z))

    shape = add_box(shape, (width + overlap, rail, rail), (0.0, half_d - rail / 2.0, top_z))
    shape = add_box(shape, (width + overlap, rail, rail), (0.0, -half_d + rail / 2.0, top_z))
    shape = add_box(shape, (rail, depth + overlap, rail), (half_w - rail / 2.0, 0.0, top_z))
    shape = add_box(shape, (rail, depth + overlap, rail), (-half_w + rail / 2.0, 0.0, top_z))

    post_z = (bottom_z + top_z) / 2.0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            shape = add_box(
                shape,
                (rail + overlap, rail + overlap, post_height + overlap),
                (sx * (half_w - rail / 2.0), sy * (half_d - rail / 2.0), post_z),
            )

    slat_count = 5 if height > 0.10 else 4
    for idx in range(slat_count):
        fraction = (idx + 1.0) / (slat_count + 1.0)
        x = -half_w + rail + fraction * (width - 2.0 * rail)
        shape = add_box(
            shape,
            (rail + overlap, depth - 2.0 * rail + overlap, rail),
            (x, 0.0, bottom_z + 0.018),
        )

    front_guard_z = top_z + 0.018
    shape = add_box(
        shape,
        (width * 0.74 + overlap, rail, rail),
        (0.0, half_d - rail / 2.0, front_guard_z),
    )

    if height > 0.10:
        shape = add_box(shape, (rail + overlap, depth * 0.45, rail), (0.0, 0.0, top_z + 0.012))

    return shape


def make_wash_arm(length: float, cross_length: float, thickness: float) -> cq.Workplane:
    shape: cq.Workplane | None = None
    shape = add_box(shape, (0.040, 0.040, thickness), (0.0, 0.0, 0.0))
    shape = add_box(shape, (length, 0.030, thickness), (0.0, 0.0, 0.0))
    shape = add_box(shape, (0.028, cross_length, thickness), (0.0, 0.0, 0.0))
    shape = add_box(shape, (0.050, 0.018, thickness), (length * 0.33, 0.0, 0.0))
    shape = add_box(shape, (0.050, 0.018, thickness), (-length * 0.28, 0.0, 0.0))
    return shape


def add_rack_visuals(part, width: float, depth: float, height: float, material) -> None:
    rail = 0.008
    overlap = 0.002
    half_w = width / 2.0
    half_d = depth / 2.0
    bottom_z = -height * 0.32
    top_z = bottom_z + height - rail
    post_height = height - rail + overlap

    def bar(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float]) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    bar("frame_front_bottom", (width + overlap, rail, rail), (0.0, half_d - rail / 2.0, bottom_z))
    bar("frame_rear_bottom", (width + overlap, rail, rail), (0.0, -half_d + rail / 2.0, bottom_z))
    bar("frame_side_0", (rail, depth + overlap, rail), (-half_w + rail / 2.0, 0.0, bottom_z))
    bar("frame_side_1", (rail, depth + overlap, rail), (half_w - rail / 2.0, 0.0, bottom_z))

    bar("frame_front_top", (width + overlap, rail, rail), (0.0, half_d - rail / 2.0, top_z))
    bar("frame_rear_top", (width + overlap, rail, rail), (0.0, -half_d + rail / 2.0, top_z))
    bar("frame_top_side_0", (rail, depth + overlap, rail), (-half_w + rail / 2.0, 0.0, top_z))
    bar("frame_top_side_1", (rail, depth + overlap, rail), (half_w - rail / 2.0, 0.0, top_z))

    post_z = (bottom_z + top_z) / 2.0
    bar("post_0", (rail + overlap, rail + overlap, post_height), (-half_w + rail / 2.0, -half_d + rail / 2.0, post_z))
    bar("post_1", (rail + overlap, rail + overlap, post_height), (-half_w + rail / 2.0, half_d - rail / 2.0, post_z))
    bar("post_2", (rail + overlap, rail + overlap, post_height), (half_w - rail / 2.0, -half_d + rail / 2.0, post_z))
    bar("post_3", (rail + overlap, rail + overlap, post_height), (half_w - rail / 2.0, half_d - rail / 2.0, post_z))

    slat_count = 5 if height > 0.10 else 4
    for idx in range(slat_count):
        fraction = (idx + 1.0) / (slat_count + 1.0)
        x = -half_w + rail + fraction * (width - 2.0 * rail)
        bar(
            f"floor_slat_{idx}",
            (rail + overlap, depth - 2.0 * rail + overlap, rail),
            (x, 0.0, bottom_z),
        )

    bar("front_guard", (width * 0.74 + overlap, rail, rail), (0.0, half_d - rail / 2.0, top_z))
    if height > 0.10:
        bar("center_upright", (rail + overlap, depth - 2.0 * rail + overlap, rail), (0.0, 0.0, top_z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underbar_glasswasher")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.22, 0.23, 0.24, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))
    rack_gray = model.material("rack_gray", rgba=(0.84, 0.86, 0.88, 1.0))
    wash_gray = model.material("wash_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    detergent_blue = model.material("detergent_blue", rgba=(0.24, 0.40, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shell(), "body_shell"),
        material=stainless,
        name="shell",
    )
    body.visual(
        Box((GUIDE_W, GUIDE_LEN, GUIDE_H)),
        origin=Origin(xyz=(-GUIDE_X, GUIDE_Y, LOWER_GUIDE_Z)),
        material=stainless,
        name="lower_guide_0",
    )
    body.visual(
        Box((GUIDE_W, GUIDE_LEN, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, LOWER_GUIDE_Z)),
        material=stainless,
        name="lower_guide_1",
    )
    body.visual(
        Box((GUIDE_W, GUIDE_LEN, GUIDE_H)),
        origin=Origin(xyz=(-GUIDE_X, GUIDE_Y, UPPER_GUIDE_Z)),
        material=stainless,
        name="upper_guide_0",
    )
    body.visual(
        Box((GUIDE_W, GUIDE_LEN, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, UPPER_GUIDE_Z)),
        material=stainless,
        name="upper_guide_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=wash_gray,
        name="lower_hub",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.570)),
        material=wash_gray,
        name="upper_hub",
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        mesh_from_cadquery(make_control_pod_shell(), "control_pod_shell"),
        material=dark_panel,
        name="pod_shell",
    )
    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(0.0, POD_FRONT_Y, POD_BOTTOM)),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(make_door_shell(), "door_shell"),
        material=stainless,
        name="door_shell",
    )
    door.visual(
        Box((0.13, 0.038, 0.09)),
        origin=Origin(xyz=(0.0, 0.019, 0.3275)),
        material=stainless,
        name="detergent_housing",
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, DOOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    detergent_flap = model.part("detergent_flap")
    detergent_flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(0.0, -FLAP_T / 2.0, -FLAP_H / 2.0)),
        material=detergent_blue,
        name="flap_panel",
    )
    model.articulation(
        "door_to_detergent_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_flap,
        origin=Origin(xyz=(0.0, 0.0, FLAP_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )

    lower_rack = model.part("lower_rack")
    add_rack_visuals(lower_rack, LOWER_RACK_W, LOWER_RACK_D, LOWER_RACK_H, rack_gray)
    lower_rack.visual(
        Box((RUNNER_W, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(-RUNNER_X, RUNNER_Y, 0.0)),
        material=rack_gray,
        name="runner_0",
    )
    lower_rack.visual(
        Box((RUNNER_W, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(RUNNER_X, RUNNER_Y, 0.0)),
        material=rack_gray,
        name="runner_1",
    )
    lower_rack.visual(
        Box((0.012, RUNNER_LEN, abs(-LOWER_RACK_H * 0.32) + RUNNER_H / 2.0 + 0.002)),
        origin=Origin(xyz=(-0.248, RUNNER_Y, (-LOWER_RACK_H * 0.32 + RUNNER_H / 2.0) / 2.0)),
        material=rack_gray,
        name="runner_support_0",
    )
    lower_rack.visual(
        Box((0.012, RUNNER_LEN, abs(-LOWER_RACK_H * 0.32) + RUNNER_H / 2.0 + 0.002)),
        origin=Origin(xyz=(0.248, RUNNER_Y, (-LOWER_RACK_H * 0.32 + RUNNER_H / 2.0) / 2.0)),
        material=rack_gray,
        name="runner_support_1",
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, GUIDE_Y, LOWER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.35,
            lower=0.0,
            upper=LOWER_RACK_TRAVEL,
        ),
    )

    upper_rack = model.part("upper_rack")
    add_rack_visuals(upper_rack, UPPER_RACK_W, UPPER_RACK_D, UPPER_RACK_H, rack_gray)
    upper_rack.visual(
        Box((RUNNER_W, RUNNER_LEN - 0.02, RUNNER_H)),
        origin=Origin(xyz=(-RUNNER_X, RUNNER_Y + 0.005, 0.0)),
        material=rack_gray,
        name="runner_0",
    )
    upper_rack.visual(
        Box((RUNNER_W, RUNNER_LEN - 0.02, RUNNER_H)),
        origin=Origin(xyz=(RUNNER_X, RUNNER_Y + 0.005, 0.0)),
        material=rack_gray,
        name="runner_1",
    )
    upper_rack.visual(
        Box((0.022, RUNNER_LEN - 0.02, abs(-UPPER_RACK_H * 0.32) + RUNNER_H / 2.0 + 0.002)),
        origin=Origin(xyz=(-0.243, RUNNER_Y + 0.005, (-UPPER_RACK_H * 0.32 + RUNNER_H / 2.0) / 2.0)),
        material=rack_gray,
        name="runner_support_0",
    )
    upper_rack.visual(
        Box((0.022, RUNNER_LEN - 0.02, abs(-UPPER_RACK_H * 0.32) + RUNNER_H / 2.0 + 0.002)),
        origin=Origin(xyz=(0.243, RUNNER_Y + 0.005, (-UPPER_RACK_H * 0.32 + RUNNER_H / 2.0) / 2.0)),
        material=rack_gray,
        name="runner_support_1",
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, GUIDE_Y, UPPER_RACK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=UPPER_RACK_TRAVEL,
        ),
    )

    lower_wash_arm = model.part("lower_wash_arm")
    lower_wash_arm.visual(
        mesh_from_cadquery(make_wash_arm(0.28, 0.16, 0.012), "lower_wash_arm"),
        material=wash_gray,
        name="arm",
    )
    model.articulation(
        "body_to_lower_wash_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_wash_arm,
        origin=Origin(xyz=(0.0, 0.0, LOWER_ARM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=10.0),
    )

    upper_wash_arm = model.part("upper_wash_arm")
    upper_wash_arm.visual(
        mesh_from_cadquery(make_wash_arm(0.22, 0.12, 0.010), "upper_wash_arm"),
        material=wash_gray,
        name="arm",
    )
    model.articulation(
        "body_to_upper_wash_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=upper_wash_arm,
        origin=Origin(xyz=(0.0, 0.0, UPPER_ARM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.022,
                body_style="skirted",
                top_diameter=0.030,
                skirt=KnobSkirt(0.044, 0.004, flare=0.04),
                grip=KnobGrip(style="fluted", count=18, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            ),
            "timer_knob",
        ),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_shell",
    )
    model.articulation(
        "control_pod_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=control_pod,
        child=timer_knob,
        origin=Origin(xyz=(KNOB_X, POD_D, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    switch_0 = model.part("rocker_0")
    switch_0.visual(
        Box((SWITCH_W, SWITCH_D, SWITCH_H)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=black,
        name="rocker_cap",
    )
    model.articulation(
        "control_pod_to_rocker_0",
        ArticulationType.REVOLUTE,
        parent=control_pod,
        child=switch_0,
        origin=Origin(xyz=(SWITCH_0_X, POD_D, SWITCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-SWITCH_TRAVEL,
            upper=SWITCH_TRAVEL,
        ),
    )

    switch_1 = model.part("rocker_1")
    switch_1.visual(
        Box((SWITCH_W, SWITCH_D, SWITCH_H)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=black,
        name="rocker_cap",
    )
    model.articulation(
        "control_pod_to_rocker_1",
        ArticulationType.REVOLUTE,
        parent=control_pod,
        child=switch_1,
        origin=Origin(xyz=(SWITCH_1_X, POD_D, SWITCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-SWITCH_TRAVEL,
            upper=SWITCH_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    detergent_flap = object_model.get_part("detergent_flap")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    rocker_0 = object_model.get_part("rocker_0")
    rocker_1 = object_model.get_part("rocker_1")

    door_hinge = object_model.get_articulation("body_to_door")
    flap_hinge = object_model.get_articulation("door_to_detergent_flap")
    lower_slide = object_model.get_articulation("body_to_lower_rack")
    upper_slide = object_model.get_articulation("body_to_upper_rack")
    rocker_0_joint = object_model.get_articulation("control_pod_to_rocker_0")
    rocker_1_joint = object_model.get_articulation("control_pod_to_rocker_1")
    timer_joint = object_model.get_articulation("control_pod_to_timer_knob")
    lower_arm_joint = object_model.get_articulation("body_to_lower_wash_arm")
    upper_arm_joint = object_model.get_articulation("body_to_upper_wash_arm")

    def aabb_shift(aabb_a, aabb_b) -> float | None:
        if aabb_a is None or aabb_b is None:
            return None
        return max(
            abs(aabb_a[i][j] - aabb_b[i][j])
            for i in range(2)
            for j in range(3)
        )

    ctx.check(
        "door is revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={door_hinge.articulation_type!r}",
    )
    ctx.check(
        "racks are prismatic",
        lower_slide.articulation_type == ArticulationType.PRISMATIC
        and upper_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"lower={lower_slide.articulation_type!r}, upper={upper_slide.articulation_type!r}",
    )
    ctx.check(
        "wash arms and timer are continuous",
        lower_arm_joint.articulation_type == ArticulationType.CONTINUOUS
        and upper_arm_joint.articulation_type == ArticulationType.CONTINUOUS
        and timer_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"lower_arm={lower_arm_joint.articulation_type!r}, "
            f"upper_arm={upper_arm_joint.articulation_type!r}, "
            f"timer={timer_joint.articulation_type!r}"
        ),
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_shell",
            negative_elem="shell",
            max_gap=0.004,
            max_penetration=0.001,
            name="door closes against the body front",
        )
        closed_door = ctx.part_element_world_aabb(door, elem="door_shell")

    door_upper = door_hinge.motion_limits.upper if door_hinge.motion_limits is not None else None
    if door_upper is not None:
        with ctx.pose({door_hinge: door_upper}):
            open_door = ctx.part_element_world_aabb(door, elem="door_shell")
        ctx.check(
            "door drops forward when opened",
            closed_door is not None
            and open_door is not None
            and open_door[1][1] > closed_door[1][1] + 0.16
            and open_door[1][2] < closed_door[1][2] - 0.20,
            details=f"closed={closed_door!r}, open={open_door!r}",
        )

    with ctx.pose({flap_hinge: 0.0}):
        closed_flap = ctx.part_element_world_aabb(detergent_flap, elem="flap_panel")
    flap_upper = flap_hinge.motion_limits.upper if flap_hinge.motion_limits is not None else None
    if flap_upper is not None:
        with ctx.pose({flap_hinge: flap_upper}):
            open_flap = ctx.part_element_world_aabb(detergent_flap, elem="flap_panel")
        ctx.check(
            "detergent flap opens toward the chamber",
            closed_flap is not None
            and open_flap is not None
            and open_flap[0][1] < closed_flap[0][1] - 0.02,
            details=f"closed={closed_flap!r}, open={open_flap!r}",
        )

    ctx.expect_gap(
        lower_rack,
        body,
        axis="z",
        positive_elem="runner_0",
        negative_elem="lower_guide_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="lower rack left runner sits on guide",
    )
    ctx.expect_gap(
        lower_rack,
        body,
        axis="z",
        positive_elem="runner_1",
        negative_elem="lower_guide_1",
        max_gap=0.002,
        max_penetration=0.001,
        name="lower rack right runner sits on guide",
    )
    ctx.expect_overlap(
        lower_rack,
        body,
        axes="xy",
        elem_a="runner_0",
        elem_b="lower_guide_0",
        min_overlap=0.006,
        name="lower rack left runner overlaps guide footprint at rest",
    )
    ctx.expect_overlap(
        lower_rack,
        body,
        axes="xy",
        elem_a="runner_1",
        elem_b="lower_guide_1",
        min_overlap=0.006,
        name="lower rack right runner overlaps guide footprint at rest",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    lower_upper = lower_slide.motion_limits.upper if lower_slide.motion_limits is not None else None
    if lower_upper is not None:
        with ctx.pose({lower_slide: lower_upper}):
            lower_extended = ctx.part_world_position(lower_rack)
            ctx.expect_gap(
                lower_rack,
                body,
                axis="z",
                positive_elem="runner_0",
                negative_elem="lower_guide_0",
                max_gap=0.002,
                max_penetration=0.001,
                name="lower rack left runner stays supported when extended",
            )
            ctx.expect_overlap(
                lower_rack,
                body,
                axes="xy",
                elem_a="runner_0",
                elem_b="lower_guide_0",
                min_overlap=0.006,
                name="lower rack left runner retains guide overlap when extended",
            )
        ctx.check(
            "lower rack slides forward",
            lower_rest is not None
            and lower_extended is not None
            and lower_extended[1] > lower_rest[1] + 0.18,
            details=f"rest={lower_rest!r}, extended={lower_extended!r}",
        )

    ctx.expect_gap(
        upper_rack,
        body,
        axis="z",
        positive_elem="runner_0",
        negative_elem="upper_guide_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="upper rack left runner sits on guide",
    )
    ctx.expect_gap(
        upper_rack,
        body,
        axis="z",
        positive_elem="runner_1",
        negative_elem="upper_guide_1",
        max_gap=0.002,
        max_penetration=0.001,
        name="upper rack right runner sits on guide",
    )

    upper_rest = ctx.part_world_position(upper_rack)
    upper_upper = upper_slide.motion_limits.upper if upper_slide.motion_limits is not None else None
    if upper_upper is not None:
        with ctx.pose({upper_slide: upper_upper}):
            upper_extended = ctx.part_world_position(upper_rack)
            ctx.expect_overlap(
                upper_rack,
                body,
                axes="xy",
                elem_a="runner_0",
                elem_b="upper_guide_0",
                min_overlap=0.006,
                name="upper rack left runner retains guide overlap when extended",
            )
        ctx.check(
            "upper rack slides forward",
            upper_rest is not None
            and upper_extended is not None
            and upper_extended[1] > upper_rest[1] + 0.14,
            details=f"rest={upper_rest!r}, extended={upper_extended!r}",
        )

    rocker_0_rest = ctx.part_element_world_aabb(rocker_0, elem="rocker_cap")
    rocker_1_rest = ctx.part_element_world_aabb(rocker_1, elem="rocker_cap")
    rocker_upper = rocker_0_joint.motion_limits.upper if rocker_0_joint.motion_limits is not None else None
    if rocker_upper is not None:
        with ctx.pose({rocker_0_joint: rocker_upper}):
            rocker_0_tilted = ctx.part_element_world_aabb(rocker_0, elem="rocker_cap")
            rocker_1_same = ctx.part_element_world_aabb(rocker_1, elem="rocker_cap")
        ctx.check(
            "rocker_0 moves independently",
            rocker_0_rest is not None
            and rocker_0_tilted is not None
            and rocker_1_rest is not None
            and rocker_1_same is not None
            and abs(rocker_0_tilted[1][1] - rocker_0_rest[1][1]) > 0.003
            and aabb_shift(rocker_1_rest, rocker_1_same) is not None
            and aabb_shift(rocker_1_rest, rocker_1_same) < 1e-6,
            details=(
                f"rocker_0_rest={rocker_0_rest!r}, rocker_0_tilted={rocker_0_tilted!r}, "
                f"rocker_1_rest={rocker_1_rest!r}, rocker_1_same={rocker_1_same!r}"
            ),
        )

    rocker_lower = rocker_1_joint.motion_limits.lower if rocker_1_joint.motion_limits is not None else None
    if rocker_lower is not None:
        with ctx.pose({rocker_1_joint: rocker_lower}):
            rocker_1_tilted = ctx.part_element_world_aabb(rocker_1, elem="rocker_cap")
        ctx.check(
            "rocker_1 tilts on its own pivot",
            rocker_1_rest is not None
            and rocker_1_tilted is not None
            and abs(rocker_1_tilted[0][1] - rocker_1_rest[0][1]) > 0.003,
            details=f"rest={rocker_1_rest!r}, tilted={rocker_1_tilted!r}",
        )

    return ctx.report()


object_model = build_object_model()
