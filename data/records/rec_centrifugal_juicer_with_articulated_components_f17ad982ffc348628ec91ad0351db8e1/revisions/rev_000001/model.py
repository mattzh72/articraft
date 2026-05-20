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


BASE_LENGTH = 0.30
BASE_WIDTH = 0.23
BASE_HEIGHT = 0.16
BOWL_OUTER_RADIUS = 0.098
BOWL_INNER_RADIUS = 0.084
BOWL_WALL_START_Z = 0.198
BOWL_WALL_HEIGHT = 0.097
HINGE_X = -0.098
HINGE_Z = 0.300
CHUTE_CENTER_X = 0.098
BUTTON_Z = 0.055


def _build_body_shell() -> object:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )

    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(0.072)
        .workplane(offset=0.032)
        .circle(BOWL_OUTER_RADIUS)
        .loft(combine=True)
    )

    bowl_wall = (
        cq.Workplane("XY")
        .workplane(offset=BOWL_WALL_START_Z)
        .circle(BOWL_OUTER_RADIUS)
        .extrude(BOWL_WALL_HEIGHT)
    )

    transition_collar = (
        cq.Workplane("XY")
        .workplane(offset=0.156)
        .circle(0.090)
        .extrude(0.020)
    )

    side_chamber = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.102, 0.226))
        .box(0.142, 0.060, 0.112, centered=(True, True, True))
    )

    shell = base.union(shoulder).union(transition_collar).union(bowl_wall).union(side_chamber)

    basket_cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.166)
        .circle(BOWL_INNER_RADIUS)
        .extrude(0.135)
    )
    shell = shell.cut(basket_cavity)

    entry_relief = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(0.074)
        .extrude(0.020)
    )
    shell = shell.cut(entry_relief)

    button_socket = (
        cq.Workplane("XY")
        .transformed(offset=(0.141, 0.0, BUTTON_Z))
        .box(0.020, 0.032, 0.032, centered=(True, True, True))
    )
    shell = shell.cut(button_socket)

    return shell


def _build_lid_shell() -> object:
    cover = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)
        .center(CHUTE_CENTER_X, 0.0)
        .circle(0.101)
        .circle(0.031)
        .extrude(0.006)
    )

    rear_bridge = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)
        .center(0.030, 0.0)
        .rect(0.060, 0.052)
        .extrude(0.010)
    )

    chute = (
        cq.Workplane("XY")
        .workplane(offset=-0.004)
        .center(CHUTE_CENTER_X, 0.0)
        .circle(0.041)
        .circle(0.030)
        .extrude(0.110)
    )
    return cover.union(rear_bridge).union(chute)


def _build_service_door_shape() -> object:
    panel = (
        cq.Workplane("XY")
        .transformed(offset=(0.059, 0.002, 0.0))
        .box(0.118, 0.004, 0.108, centered=(True, True, True))
    )
    grip = (
        cq.Workplane("XY")
        .transformed(offset=(0.096, 0.009, 0.0))
        .box(0.018, 0.010, 0.046, centered=(True, True, True))
    )
    return panel.union(grip)


def _build_basket_shape() -> object:
    basket = cq.Workplane("XY").circle(0.078).extrude(0.068)
    basket = basket.cut(cq.Workplane("XY").workplane(offset=0.008).circle(0.069).extrude(0.060))
    basket = basket.union(cq.Workplane("XY").circle(0.036).extrude(0.008))
    basket = basket.union(cq.Workplane("XY").circle(0.012).extrude(0.076))
    basket = basket.union(
        cq.Workplane("XY").workplane(offset=0.068).circle(0.082).circle(0.072).extrude(0.008)
    )

    for angle_deg in range(0, 360, 30):
        cutter = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, 0.0, 0.038), rotate=(0.0, 0.0, angle_deg))
            .center(0.073, 0.0)
            .box(0.010, 0.020, 0.044, centered=(True, True, True))
        )
        basket = basket.cut(cutter)

    return basket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_steel_juicer")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.17, 1.0))
    clear_poly = model.material("clear_poly", rgba=(0.78, 0.88, 0.92, 0.32))
    smoke_poly = model.material("smoke_poly", rgba=(0.20, 0.22, 0.24, 0.70))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "juicer_body_shell"),
        material=stainless,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=stainless,
        name="drive_collar",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "juicer_lid_shell"),
        material=clear_poly,
        name="lid_shell",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.026, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=dark_plastic,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_plastic,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_plastic,
        name="pusher_handle",
    )

    service_door = model.part("service_door")
    service_door.visual(
        mesh_from_cadquery(_build_service_door_shape(), "juicer_service_door"),
        material=smoke_poly,
        name="door_shell",
    )

    pulse_button = model.part("pulse_button")
    pulse_button.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="button_stem",
    )
    pulse_button.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="button_cap",
    )

    grater_basket = model.part("grater_basket")
    grater_basket.visual(
        mesh_from_cadquery(_build_basket_shape(), "juicer_grater_basket"),
        material=stainless,
        name="basket_shell",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_CENTER_X, 0.0, 0.106)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(-0.060, 0.132, 0.226)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )
    model.articulation(
        "body_to_pulse_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pulse_button,
        origin=Origin(xyz=(0.150, 0.0, BUTTON_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "body_to_grater_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=grater_basket,
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    service_door = object_model.get_part("service_door")
    pulse_button = object_model.get_part("pulse_button")
    grater_basket = object_model.get_part("grater_basket")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    door_hinge = object_model.get_articulation("body_to_service_door")
    button_slide = object_model.get_articulation("body_to_pulse_button")
    basket_spin = object_model.get_articulation("body_to_grater_basket")

    with ctx.pose(
        {
            lid_hinge: 0.0,
            pusher_slide: 0.0,
            door_hinge: 0.0,
            button_slide: 0.0,
            basket_spin: 0.0,
        }
    ):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            min_gap=0.0,
            max_gap=0.010,
            name="lid seats on the upper body without penetration",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.17,
            name="lid covers the juicing chamber",
        )
        ctx.expect_gap(
            service_door,
            body,
            axis="y",
            positive_elem="door_shell",
            negative_elem="body_shell",
            max_gap=0.002,
            max_penetration=0.0,
            name="service door rests against the side body",
        )
        ctx.expect_overlap(
            service_door,
            body,
            axes="xz",
            elem_a="door_shell",
            elem_b="body_shell",
            min_overlap=0.09,
            name="service door covers the side chamber",
        )
        ctx.expect_gap(
            lid,
            grater_basket,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="basket_shell",
            min_gap=0.015,
            name="grater basket clears the closed lid",
        )
        ctx.expect_overlap(
            grater_basket,
            body,
            axes="xy",
            elem_a="basket_shell",
            elem_b="body_shell",
            min_overlap=0.13,
            name="grater basket stays centered in the body cavity",
        )

        lid_rest_aabb = ctx.part_world_aabb(lid)
        door_rest_aabb = ctx.part_world_aabb(service_door)
        pusher_rest_pos = ctx.part_world_position(pusher)
        button_rest_pos = ctx.part_world_position(pulse_button)

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else 0.0
    with ctx.pose({lid_hinge: lid_upper}):
        lid_open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward on the rear hinge",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.10,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )

    door_upper = door_hinge.motion_limits.upper if door_hinge.motion_limits is not None else 0.0
    with ctx.pose({door_hinge: door_upper}):
        door_open_aabb = ctx.part_world_aabb(service_door)

    ctx.check(
        "service door swings outward from the side",
        door_rest_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][1] > door_rest_aabb[1][1] + 0.06,
        details=f"rest={door_rest_aabb}, open={door_open_aabb}",
    )

    pusher_upper = pusher_slide.motion_limits.upper if pusher_slide.motion_limits is not None else 0.0
    with ctx.pose({lid_hinge: 0.0, pusher_slide: pusher_upper}):
        pusher_extended_pos = ctx.part_world_position(pusher)

    ctx.check(
        "feed pusher slides upward through the chute",
        pusher_rest_pos is not None
        and pusher_extended_pos is not None
        and pusher_extended_pos[2] > pusher_rest_pos[2] + 0.06,
        details=f"rest={pusher_rest_pos}, extended={pusher_extended_pos}",
    )

    button_upper = button_slide.motion_limits.upper if button_slide.motion_limits is not None else 0.0
    with ctx.pose({button_slide: button_upper}):
        button_pressed_pos = ctx.part_world_position(pulse_button)

    ctx.check(
        "pulse button presses inward on the front panel",
        button_rest_pos is not None
        and button_pressed_pos is not None
        and button_pressed_pos[0] < button_rest_pos[0] - 0.0025,
        details=f"rest={button_rest_pos}, pressed={button_pressed_pos}",
    )

    ctx.check(
        "grater basket uses a continuous vertical drive axis",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and basket_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={basket_spin.articulation_type}, axis={basket_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
