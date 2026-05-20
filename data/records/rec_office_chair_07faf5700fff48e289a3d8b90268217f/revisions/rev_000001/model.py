from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WHEEL_COUNT = 5
WHEEL_RADIUS = 0.030
WHEEL_WIDTH = 0.024
BASE_RADIUS = 0.305
COLUMN_TRAVEL = 0.090
ARMREST_UPPER = 1.28
SEAT_PIVOT_Y = -0.060
SEAT_PIVOT_Z = 0.158


def _rotate_z(shape: cq.Workplane, angle_rad: float) -> cq.Workplane:
    return shape.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(angle_rad))


def _polar_xy(radius: float, angle_rad: float) -> tuple[float, float]:
    return (radius * math.cos(angle_rad), radius * math.sin(angle_rad))


def _build_base_star() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.062).extrude(0.050).translate((0.0, 0.0, 0.060))
    crown = cq.Workplane("XY").circle(0.050).extrude(0.022).translate((0.0, 0.0, 0.110))
    star = hub.union(crown)

    leg = cq.Workplane("XY").box(0.245, 0.060, 0.024).translate((0.1825, 0.0, 0.086))
    tip = cq.Workplane("XY").box(0.050, 0.048, 0.020).translate((0.280, 0.0, 0.084))
    mount_pad = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.012)
        .translate((BASE_RADIUS, 0.0, 0.082))
    )
    spoke = leg.union(tip).union(mount_pad)

    for index in range(WHEEL_COUNT):
        angle = math.radians(90.0 + 72.0 * index)
        star = star.union(_rotate_z(spoke, angle))

    center_hole = cq.Workplane("XY").circle(0.0205).extrude(0.090).translate((0.0, 0.0, 0.055))
    star = star.cut(center_hole)

    stem_hole = (
        cq.Workplane("XY")
        .circle(0.0105)
        .extrude(0.040)
        .translate((BASE_RADIUS, 0.0, 0.066))
    )
    for index in range(WHEEL_COUNT):
        angle = math.radians(90.0 + 72.0 * index)
        star = star.cut(_rotate_z(stem_hole, angle))

    return star


def _build_column_sleeve() -> cq.Workplane:
    lower_collar = (
        cq.Workplane("XY")
        .circle(0.040)
        .circle(0.025)
        .extrude(0.026)
        .translate((0.0, 0.0, 0.084))
    )
    sleeve = (
        cq.Workplane("XY")
        .circle(0.031)
        .circle(0.0225)
        .extrude(0.145)
        .translate((0.0, 0.0, 0.110))
    )
    return lower_collar.union(sleeve)


def _build_inner_column() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.016).extrude(0.240).translate((0.0, 0.0, -0.150))


def _build_seat_mechanism() -> cq.Workplane:
    guide_collar = (
        cq.Workplane("XY")
        .circle(0.0305)
        .circle(0.0165)
        .extrude(0.004)
        .translate((0.0, 0.0, 0.0))
    )
    collar = cq.Workplane("XY").circle(0.024).extrude(0.061).translate((0.0, 0.0, 0.003))
    block = cq.Workplane("XY").box(0.190, 0.160, 0.040).translate((0.0, 0.0, 0.082))
    rear_boss = cq.Workplane("XY").box(0.120, 0.090, 0.026).translate((0.0, -0.042, 0.107))
    return guide_collar.union(collar).union(block).union(rear_boss)


def _build_seat_frame() -> cq.Workplane:
    tray = cq.Workplane("XY").box(0.470, 0.405, 0.028).translate((0.0, 0.0, 0.018))
    mechanism_cover = cq.Workplane("XY").box(0.180, 0.160, 0.040).translate((0.0, -0.010, 0.052))
    spine = cq.Workplane("XY").box(0.074, 0.050, 0.500).translate((0.0, -0.205, 0.268))
    rear_bridge = cq.Workplane("XY").box(0.160, 0.100, 0.050).translate((0.0, -0.150, 0.082))
    return tray.union(mechanism_cover).union(spine).union(rear_bridge)


def _build_seat_cushion() -> cq.Workplane:
    cushion = cq.Workplane("XY").box(0.520, 0.485, 0.065).translate((0.0, 0.012, 0.046))
    return cushion.edges("|Z").fillet(0.028)


def _build_backrest_frame() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.450, 0.070, 0.590).translate((0.0, -0.238, 0.505))
    frame = frame.cut(cq.Workplane("XY").box(0.318, 0.100, 0.405).translate((0.0, -0.238, 0.500)))
    lumbar_bar = cq.Workplane("XY").box(0.250, 0.030, 0.090).translate((0.0, -0.220, 0.405))
    shoulder_bar = cq.Workplane("XY").box(0.220, 0.026, 0.065).translate((0.0, -0.220, 0.620))
    frame = frame.union(lumbar_bar).union(shoulder_bar)
    return frame.rotate((0.0, -0.238, 0.220), (1.0, -0.238, 0.220), -10.0)


def _build_backrest_mesh_panel() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.300, 0.012, 0.390).translate((0.0, -0.232, 0.502))
    tension_band = cq.Workplane("XY").box(0.220, 0.016, 0.070).translate((0.0, -0.226, 0.410))
    panel = panel.union(tension_band)
    return panel.rotate((0.0, -0.238, 0.220), (1.0, -0.238, 0.220), -10.0)


def _build_armrest_bracket(side_sign: float) -> cq.Workplane:
    x_support = side_sign * 0.248
    x_cheek = side_sign * 0.278
    support = cq.Workplane("XY").box(0.034, 0.056, 0.118).translate(
        (x_support, SEAT_PIVOT_Y, 0.086)
    )
    cheek_front = cq.Workplane("XY").box(0.026, 0.008, 0.052).translate(
        (x_cheek, SEAT_PIVOT_Y + 0.025, SEAT_PIVOT_Z)
    )
    cheek_rear = cq.Workplane("XY").box(0.026, 0.008, 0.052).translate(
        (x_cheek, SEAT_PIVOT_Y - 0.025, SEAT_PIVOT_Z)
    )
    bridge = cq.Workplane("XY").box(0.026, 0.058, 0.010).translate(
        (x_cheek, SEAT_PIVOT_Y, SEAT_PIVOT_Z + 0.026)
    )
    return support.union(cheek_front).union(cheek_rear).union(bridge)


def _build_armrest_body(side_sign: float) -> cq.Workplane:
    pivot_block = cq.Workplane("XY").box(0.008, 0.084, 0.022).translate(
        (side_sign * 0.020, 0.0, -0.010)
    )
    web = cq.Workplane("XY").box(0.030, 0.185, 0.030).translate(
        (side_sign * 0.037, 0.010, 0.010)
    )
    rear_hanger = cq.Workplane("XY").box(0.028, 0.055, 0.070).translate(
        (side_sign * 0.050, -0.078, 0.004)
    )
    front_hanger = cq.Workplane("XY").box(0.022, 0.050, 0.060).translate(
        (side_sign * 0.048, 0.096, 0.002)
    )
    pad = cq.Workplane("XY").box(0.060, 0.285, 0.040).translate(
        (side_sign * 0.075, 0.018, 0.038)
    )
    return pivot_block.union(web).union(rear_hanger).union(front_hanger).union(pad)


def _build_pivot_barrel() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.011)
        .extrude(0.042)
        .translate((0.0, 0.0, -0.021))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _build_caster_fork() -> cq.Workplane:
    cheek_left = cq.Workplane("XY").box(0.014, 0.006, 0.054).translate((0.0, -0.018, 0.007))
    cheek_right = cq.Workplane("XY").box(0.014, 0.006, 0.054).translate((0.0, 0.018, 0.007))
    bridge = cq.Workplane("XY").box(0.024, 0.042, 0.010).translate((0.0, 0.0, 0.038))
    stem = cq.Workplane("XY").circle(0.007).extrude(0.013).translate((0.0, 0.0, 0.043))
    cap = cq.Workplane("XY").circle(0.0105).extrude(0.008).translate((0.0, 0.0, 0.056))
    return cheek_left.union(cheek_right).union(bridge).union(stem).union(cap)


def _build_caster_wheel() -> cq.Workplane:
    wheel_left = (
        cq.Workplane("XY")
        .circle(WHEEL_RADIUS)
        .extrude(0.008)
        .translate((0.0, 0.0, -0.004))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, -0.008, 0.0))
    )
    wheel_right = (
        cq.Workplane("XY")
        .circle(WHEEL_RADIUS)
        .extrude(0.008)
        .translate((0.0, 0.0, -0.004))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, 0.008, 0.0))
    )
    hub = (
        cq.Workplane("XY")
        .circle(0.013)
        .extrude(0.030)
        .translate((0.0, 0.0, -0.015))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    return wheel_left.union(wheel_right).union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_chair")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.17, 0.18, 0.20, 1.0))
    fabric = model.material("fabric", rgba=(0.24, 0.26, 0.29, 1.0))
    mesh_back = model.material("mesh_back", rgba=(0.18, 0.20, 0.22, 0.42))
    chrome = model.material("chrome", rgba=(0.72, 0.74, 0.77, 1.0))
    nylon = model.material("nylon", rgba=(0.09, 0.09, 0.10, 1.0))

    base_star_mesh = mesh_from_cadquery(_build_base_star(), "chair_base_star")
    sleeve_mesh = mesh_from_cadquery(_build_column_sleeve(), "chair_column_sleeve")
    inner_column_mesh = mesh_from_cadquery(_build_inner_column(), "chair_inner_column")
    seat_mechanism_mesh = mesh_from_cadquery(_build_seat_mechanism(), "chair_seat_mechanism")
    seat_frame_mesh = mesh_from_cadquery(_build_seat_frame(), "chair_seat_frame")
    seat_cushion_mesh = mesh_from_cadquery(_build_seat_cushion(), "chair_seat_cushion")
    backrest_frame_mesh = mesh_from_cadquery(_build_backrest_frame(), "chair_backrest_frame")
    backrest_mesh_panel = mesh_from_cadquery(_build_backrest_mesh_panel(), "chair_backrest_mesh")
    left_bracket_mesh = mesh_from_cadquery(_build_armrest_bracket(-1.0), "chair_left_bracket")
    right_bracket_mesh = mesh_from_cadquery(_build_armrest_bracket(1.0), "chair_right_bracket")
    left_armrest_mesh = mesh_from_cadquery(_build_armrest_body(-1.0), "chair_left_armrest")
    right_armrest_mesh = mesh_from_cadquery(_build_armrest_body(1.0), "chair_right_armrest")
    pivot_barrel_mesh = mesh_from_cadquery(_build_pivot_barrel(), "chair_pivot_barrel")
    caster_fork_mesh = mesh_from_cadquery(_build_caster_fork(), "chair_caster_fork")
    caster_wheel_mesh = mesh_from_cadquery(_build_caster_wheel(), "chair_caster_wheel")

    base = model.part("base")
    base.visual(base_star_mesh, material=dark_frame, name="base_star")
    base.visual(sleeve_mesh, material=matte_black, name="column_sleeve")

    column_stage = model.part("column_stage")
    column_stage.visual(inner_column_mesh, material=chrome, name="inner_column")
    column_stage.visual(seat_mechanism_mesh, material=dark_frame, name="seat_mechanism")

    seat = model.part("seat")
    seat.visual(seat_frame_mesh, material=dark_frame, name="seat_frame")
    seat.visual(seat_cushion_mesh, material=fabric, name="seat_cushion")
    seat.visual(backrest_frame_mesh, material=dark_frame, name="back_frame")
    seat.visual(backrest_mesh_panel, material=mesh_back, name="back_mesh")
    seat.visual(left_bracket_mesh, material=dark_frame, name="left_bracket")
    seat.visual(right_bracket_mesh, material=dark_frame, name="right_bracket")

    left_armrest = model.part("left_armrest")
    left_armrest.visual(left_armrest_mesh, material=nylon, name="armrest_body")
    left_armrest.visual(pivot_barrel_mesh, material=matte_black, name="pivot_barrel")

    right_armrest = model.part("right_armrest")
    right_armrest.visual(right_armrest_mesh, material=nylon, name="armrest_body")
    right_armrest.visual(pivot_barrel_mesh, material=matte_black, name="pivot_barrel")

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.20,
            lower=0.0,
            upper=COLUMN_TRAVEL,
        ),
    )
    model.articulation(
        "column_to_seat",
        ArticulationType.FIXED,
        parent=column_stage,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
    )
    model.articulation(
        "seat_to_left_armrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=left_armrest,
        origin=Origin(xyz=(-0.278, SEAT_PIVOT_Y, SEAT_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=ARMREST_UPPER,
        ),
    )
    model.articulation(
        "seat_to_right_armrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=right_armrest,
        origin=Origin(xyz=(0.278, SEAT_PIVOT_Y, SEAT_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=ARMREST_UPPER,
        ),
    )

    for index in range(WHEEL_COUNT):
        angle = math.radians(90.0 + 72.0 * index)
        caster = model.part(f"caster_{index}")
        caster.visual(caster_fork_mesh, material=dark_frame, name="fork")

        wheel = model.part(f"wheel_{index}")
        wheel.visual(caster_wheel_mesh, material=nylon, name="wheel")

        x, y = _polar_xy(BASE_RADIUS, angle)
        model.articulation(
            f"base_to_caster_{index}",
            ArticulationType.FIXED,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, WHEEL_RADIUS), rpy=(0.0, 0.0, angle)),
        )
        model.articulation(
            f"caster_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=20.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column_stage = object_model.get_part("column_stage")
    seat = object_model.get_part("seat")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")

    column_joint = object_model.get_articulation("base_to_column")
    left_joint = object_model.get_articulation("seat_to_left_armrest")
    right_joint = object_model.get_articulation("seat_to_right_armrest")

    for index in range(WHEEL_COUNT):
        ctx.allow_overlap(
            base,
            f"caster_{index}",
            elem_a="base_star",
            elem_b="fork",
            reason="The caster stem and shoulder are intentionally simplified as seated into the star-base socket.",
        )
        ctx.allow_overlap(
            f"caster_{index}",
            f"wheel_{index}",
            elem_a="fork",
            elem_b="wheel",
            reason="The wheel hub is intentionally simplified around the caster axle support.",
        )

    ctx.allow_overlap(
        left_armrest,
        seat,
        elem_a="pivot_barrel",
        elem_b="left_bracket",
        reason="The armrest pivot barrel is intentionally represented as captured inside the seat-side bracket clevis.",
    )
    ctx.allow_overlap(
        right_armrest,
        seat,
        elem_a="pivot_barrel",
        elem_b="right_bracket",
        reason="The armrest pivot barrel is intentionally represented as captured inside the seat-side bracket clevis.",
    )

    ctx.expect_within(
        left_armrest,
        seat,
        axes="xyz",
        inner_elem="pivot_barrel",
        outer_elem="left_bracket",
        margin=0.0,
        name="left armrest pivot stays captured in the seat bracket",
    )
    ctx.expect_within(
        right_armrest,
        seat,
        axes="xyz",
        inner_elem="pivot_barrel",
        outer_elem="right_bracket",
        margin=0.0,
        name="right armrest pivot stays captured in the seat bracket",
    )

    with ctx.pose({column_joint: 0.0}):
        ctx.expect_within(
            column_stage,
            base,
            axes="xy",
            inner_elem="inner_column",
            outer_elem="column_sleeve",
            margin=0.003,
            name="inner column stays centered in the sleeve at rest",
        )
        ctx.expect_overlap(
            column_stage,
            base,
            axes="z",
            elem_a="inner_column",
            elem_b="column_sleeve",
            min_overlap=0.140,
            name="inner column remains deeply inserted at rest",
        )
        rest_seat_pos = ctx.part_world_position(seat)
        left_rest_aabb = ctx.part_element_world_aabb(left_armrest, elem="armrest_body")
        right_rest_aabb = ctx.part_element_world_aabb(right_armrest, elem="armrest_body")

    with ctx.pose({column_joint: COLUMN_TRAVEL}):
        ctx.expect_within(
            column_stage,
            base,
            axes="xy",
            inner_elem="inner_column",
            outer_elem="column_sleeve",
            margin=0.003,
            name="inner column stays centered in the sleeve when raised",
        )
        ctx.expect_overlap(
            column_stage,
            base,
            axes="z",
            elem_a="inner_column",
            elem_b="column_sleeve",
            min_overlap=0.050,
            name="inner column keeps retained insertion at max height",
        )
        raised_seat_pos = ctx.part_world_position(seat)

    ctx.check(
        "seat rises when the gas-lift column extends",
        rest_seat_pos is not None
        and raised_seat_pos is not None
        and raised_seat_pos[2] > rest_seat_pos[2] + 0.060,
        details=f"rest={rest_seat_pos}, raised={raised_seat_pos}",
    )

    with ctx.pose({left_joint: ARMREST_UPPER, right_joint: ARMREST_UPPER}):
        ctx.expect_gap(
            left_armrest,
            seat,
            axis="z",
            positive_elem="armrest_body",
            negative_elem="seat_cushion",
            min_gap=0.040,
            name="left armrest clears the seat cushion when flipped up",
        )
        ctx.expect_gap(
            right_armrest,
            seat,
            axis="z",
            positive_elem="armrest_body",
            negative_elem="seat_cushion",
            min_gap=0.040,
            name="right armrest clears the seat cushion when flipped up",
        )
        left_up_aabb = ctx.part_element_world_aabb(left_armrest, elem="armrest_body")
        right_up_aabb = ctx.part_element_world_aabb(right_armrest, elem="armrest_body")

    ctx.check(
        "left armrest rotates upward",
        left_rest_aabb is not None
        and left_up_aabb is not None
        and left_up_aabb[1][2] > left_rest_aabb[1][2] + 0.020,
        details=f"rest={left_rest_aabb}, up={left_up_aabb}",
    )
    ctx.check(
        "right armrest rotates upward",
        right_rest_aabb is not None
        and right_up_aabb is not None
        and right_up_aabb[1][2] > right_rest_aabb[1][2] + 0.020,
        details=f"rest={right_rest_aabb}, up={right_up_aabb}",
    )

    wheel_joint_names = [f"caster_to_wheel_{index}" for index in range(WHEEL_COUNT)]
    wheel_joints_ok = True
    for joint_name in wheel_joint_names:
        joint = object_model.get_articulation(joint_name)
        wheel_joints_ok = wheel_joints_ok and joint.articulation_type == ArticulationType.CONTINUOUS
    ctx.check(
        "five caster wheels use continuous axle joints",
        wheel_joints_ok and len(wheel_joint_names) == 5,
        details=f"wheel_joints={wheel_joint_names}",
    )

    return ctx.report()


object_model = build_object_model()
