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


BODY_FRONT_Y = 0.018
HEAD_FRONT_Y = 0.022
JAW_CENTER_Z = 0.206
JAW_OUTER_RADIUS = 0.030
JAW_INNER_RADIUS = 0.019
JAW_THICKNESS = 0.014
HINGE_PIN_RADIUS = 0.0044
SELECTOR_Z = 0.106
DISPLAY_Z = 0.145
BUTTON_Z = 0.061
TRIGGER_Z = 0.046
BUTTON_XS = (-0.017, 0.0, 0.017)


def _ring_segment(outer_radius: float, inner_radius: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .translate((0.0, -depth * 0.5, 0.0))
    )


def _body_shell_shape() -> cq.Workplane:
    lower_heel = cq.Workplane("XY").box(0.066, 0.044, 0.032).translate((0.0, 0.0, 0.016))
    grip = cq.Workplane("XY").box(0.054, 0.038, 0.098).translate((0.0, -0.001, 0.069))
    shoulder = cq.Workplane("XY").box(0.070, 0.042, 0.046).translate((0.0, 0.000, 0.122))
    head = cq.Workplane("XY").box(0.082, 0.044, 0.050).translate((0.0, 0.000, 0.166))

    body = lower_heel.union(grip).union(shoulder).union(head)

    display_recess = cq.Workplane("XY").box(0.046, 0.008, 0.032).translate((0.0, 0.018, DISPLAY_Z))
    trigger_slot = cq.Workplane("XY").box(0.032, 0.014, 0.028).translate((0.0, 0.015, TRIGGER_Z))

    body = body.cut(display_recess).cut(trigger_slot)
    for button_x in BUTTON_XS:
        button_pocket = cq.Workplane("XY").box(0.014, 0.011, 0.012).translate((button_x, 0.0165, BUTTON_Z))
        body = body.cut(button_pocket)

    return body


def _jaw_arc_shape(angles_deg: tuple[float, ...], center_x: float, center_z: float) -> cq.Workplane:
    arc_radius = 0.024
    rod_radius = 0.0068
    arc_shape = None
    centers: list[tuple[float, float, float]] = []
    for angle_deg in angles_deg:
        angle_rad = math.radians(angle_deg)
        bead_center = (
            center_x + arc_radius * math.cos(angle_rad),
            0.0,
            center_z + arc_radius * math.sin(angle_rad),
        )
        centers.append(bead_center)
        bead = (
            cq.Workplane("XZ")
            .circle(rod_radius)
            .extrude(JAW_THICKNESS)
            .translate(
                (
                    bead_center[0],
                    -JAW_THICKNESS * 0.5,
                    bead_center[2],
                )
            )
        )
        arc_shape = bead if arc_shape is None else arc_shape.union(bead)
    for start, end in zip(centers, centers[1:]):
        dx = end[0] - start[0]
        dz = end[2] - start[2]
        length = math.hypot(dx, dz)
        midpoint = ((start[0] + end[0]) * 0.5, 0.0, (start[2] + end[2]) * 0.5)
        connector = (
            cq.Workplane("XY")
            .box(length, JAW_THICKNESS, 0.010)
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), math.degrees(math.atan2(dz, dx)))
            .translate(midpoint)
        )
        arc_shape = arc_shape.union(connector)
    assert arc_shape is not None
    return arc_shape


def _fixed_jaw_shape() -> cq.Workplane:
    jaw = _jaw_arc_shape((195.0, 230.0, 265.0, 300.0, 335.0), 0.0, JAW_CENTER_Z)
    hinge_lug = cq.Workplane("XY").box(0.012, JAW_THICKNESS, 0.018).translate((0.026, 0.0, JAW_CENTER_Z - 0.010))
    rear_knuckle = (
        cq.Workplane("XZ")
        .circle(HINGE_PIN_RADIUS)
        .extrude(0.004)
        .translate((JAW_OUTER_RADIUS, -0.007, JAW_CENTER_Z))
    )
    front_knuckle = (
        cq.Workplane("XZ")
        .circle(HINGE_PIN_RADIUS)
        .extrude(0.004)
        .translate((JAW_OUTER_RADIUS, 0.003, JAW_CENTER_Z))
    )
    return jaw.union(hinge_lug).union(rear_knuckle).union(front_knuckle)


def _upper_jaw_shape() -> cq.Workplane:
    jaw = _jaw_arc_shape((25.0, 60.0, 95.0, 130.0, 165.0), -JAW_OUTER_RADIUS, 0.006)
    hinge_lug = cq.Workplane("XY").box(0.012, JAW_THICKNESS, 0.020).translate((-0.004, 0.0, 0.014))
    center_barrel = cq.Workplane("XZ").circle(HINGE_PIN_RADIUS - 0.0004).extrude(0.005).translate((0.0, -0.0025, 0.0))
    return jaw.union(hinge_lug).union(center_barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hvac_clamp_meter")

    body_finish = model.material("body_finish", rgba=(0.88, 0.49, 0.15, 1.0))
    jaw_finish = model.material("jaw_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    control_finish = model.material("control_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    button_finish = model.material("button_finish", rgba=(0.28, 0.29, 0.30, 1.0))
    trigger_finish = model.material("trigger_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    display_bezel_finish = model.material("display_bezel_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    display_glass_finish = model.material("display_glass_finish", rgba=(0.32, 0.56, 0.55, 0.55))
    indicator_finish = model.material("indicator_finish", rgba=(0.82, 0.84, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "clamp_meter_body_shell"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_fixed_jaw_shape(), "clamp_meter_fixed_jaw"),
        material=jaw_finish,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.048, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, DISPLAY_Z)),
        material=display_bezel_finish,
        name="display_bezel",
    )
    body.visual(
        Box((0.040, 0.002, 0.024)),
        origin=Origin(xyz=(0.0, 0.021, DISPLAY_Z)),
        material=display_glass_finish,
        name="display",
    )

    upper_jaw = model.part("upper_jaw")
    upper_jaw.visual(
        mesh_from_cadquery(_upper_jaw_shape(), "clamp_meter_upper_jaw"),
        material=jaw_finish,
        name="jaw_shell",
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_finish,
        name="selector_body",
    )
    selector.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=button_finish,
        name="selector_cap",
    )
    selector.visual(
        Box((0.004, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.009, 0.016)),
        material=indicator_finish,
        name="selector_pointer",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.032, 0.010, 0.024)),
        origin=Origin(),
        material=trigger_finish,
        name="trigger_body",
    )
    trigger.visual(
        Box((0.034, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.002, 0.010)),
        material=trigger_finish,
        name="trigger_lip",
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.014, 0.008, 0.011)),
            material=button_finish,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, 0.022, BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=0.0024),
        )

    model.articulation(
        "body_to_upper_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_jaw,
        origin=Origin(xyz=(JAW_OUTER_RADIUS, 0.0, JAW_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, 0.030, SELECTOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, 0.023, TRIGGER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    upper_jaw = object_model.get_part("upper_jaw")
    selector = object_model.get_part("selector")
    trigger = object_model.get_part("trigger")
    jaw_joint = object_model.get_articulation("body_to_upper_jaw")
    trigger_joint = object_model.get_articulation("body_to_trigger")

    ctx.check(
        "selector uses continuous articulation",
        object_model.get_articulation("body_to_selector").articulation_type == ArticulationType.CONTINUOUS,
        details="The central rotary selector should spin continuously.",
    )

    jaw_limits = jaw_joint.motion_limits
    closed_jaw_aabb = None
    fixed_jaw_aabb = None
    if jaw_limits is not None and jaw_limits.lower is not None and jaw_limits.upper is not None:
        with ctx.pose({jaw_joint: jaw_limits.lower}):
            ctx.expect_overlap(
                upper_jaw,
                body,
                axes="x",
                elem_a="jaw_shell",
                elem_b="fixed_jaw",
                min_overlap=0.045,
                name="jaw halves stay laterally aligned when closed",
            )
            closed_jaw_aabb = ctx.part_element_world_aabb(upper_jaw, elem="jaw_shell")
            fixed_jaw_aabb = ctx.part_element_world_aabb(body, elem="fixed_jaw")
            ctx.check(
                "closed jaw seam stays visually tight",
                closed_jaw_aabb is not None
                and fixed_jaw_aabb is not None
                and fixed_jaw_aabb[1][2] - 0.012 <= closed_jaw_aabb[0][2] <= fixed_jaw_aabb[1][2] + 0.005,
                details=f"upper={closed_jaw_aabb}, fixed={fixed_jaw_aabb}",
            )

        with ctx.pose({jaw_joint: jaw_limits.upper}):
            open_jaw_aabb = ctx.part_element_world_aabb(upper_jaw, elem="jaw_shell")
            ctx.check(
                "upper jaw swings upward to open",
                closed_jaw_aabb is not None
                and open_jaw_aabb is not None
                and open_jaw_aabb[1][2] > closed_jaw_aabb[1][2] + 0.012,
                details=f"closed={closed_jaw_aabb}, open={open_jaw_aabb}",
            )

    selector_aabb = ctx.part_element_world_aabb(selector, elem="selector_body")
    center_button = object_model.get_part("button_1")
    center_button_aabb = ctx.part_element_world_aabb(center_button, elem="button_cap")
    ctx.check(
        "button bank sits below selector dial",
        selector_aabb is not None
        and center_button_aabb is not None
        and selector_aabb[0][2] > center_button_aabb[1][2] + 0.010,
        details=f"selector={selector_aabb}, center_button={center_button_aabb}",
    )

    trigger_limits = trigger_joint.motion_limits
    if trigger_limits is not None and trigger_limits.upper is not None:
        trigger_rest = ctx.part_world_position(trigger)
        with ctx.pose({trigger_joint: trigger_limits.upper}):
            trigger_pressed = ctx.part_world_position(trigger)
        ctx.check(
            "trigger presses into the handle",
            trigger_rest is not None and trigger_pressed is not None and trigger_pressed[1] < trigger_rest[1] - 0.003,
            details=f"rest={trigger_rest}, pressed={trigger_pressed}",
        )

    for index in range(3):
        joint = object_model.get_articulation(f"body_to_button_{index}")
        button = object_model.get_part(f"button_{index}")
        neighbor = object_model.get_part(f"button_{(index + 1) % 3}")
        limits = joint.motion_limits
        rest_button = ctx.part_world_position(button)
        rest_neighbor = ctx.part_world_position(neighbor)
        moved_button = None
        moved_neighbor = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                moved_button = ctx.part_world_position(button)
                moved_neighbor = ctx.part_world_position(neighbor)
        ctx.check(
            f"button_{index} presses independently",
            rest_button is not None
            and moved_button is not None
            and moved_button[1] < rest_button[1] - 0.0015
            and rest_neighbor is not None
            and moved_neighbor is not None
            and abs(moved_neighbor[1] - rest_neighbor[1]) < 1e-6,
            details=f"button_rest={rest_button}, button_pressed={moved_button}, neighbor_rest={rest_neighbor}, neighbor_pressed={moved_neighbor}",
        )

    return ctx.report()


object_model = build_object_model()
