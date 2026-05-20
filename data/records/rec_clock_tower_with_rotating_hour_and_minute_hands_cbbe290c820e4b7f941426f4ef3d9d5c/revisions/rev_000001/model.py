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

BODY_WIDTH = 6.0
BODY_DEPTH = 2.0
BODY_HEIGHT = 8.8
PARAPET_HEIGHT = 0.75
CLOCK_CENTER = (0.0, -1.105, 6.35)


def _arched_gatehouse_body() -> cq.Workplane:
    """Single stone mass with a full-depth arched gate opening."""
    body = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).translate(
        (0.0, 0.0, BODY_HEIGHT / 2.0)
    )

    gate_width = 2.15
    gate_rect_height = 2.95
    cutter_depth = BODY_DEPTH + 0.60
    lower_cutter = cq.Workplane("XY").box(gate_width, cutter_depth, gate_rect_height).translate(
        (0.0, 0.0, gate_rect_height / 2.0)
    )
    arch_cutter = (
        cq.Workplane("XY")
        .cylinder(cutter_depth, gate_width / 2.0)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, 0.0, gate_rect_height))
    )
    return body.cut(lower_cutter.union(arch_cutter))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="castle_gatehouse_clock_tower")

    stone = model.material("warm_gray_limestone", rgba=(0.50, 0.48, 0.43, 1.0))
    dark_stone = model.material("shadowed_stone", rgba=(0.23, 0.22, 0.20, 1.0))
    mortar = model.material("pale_mortar", rgba=(0.67, 0.65, 0.59, 1.0))
    clock_ivory = model.material("aged_clock_face", rgba=(0.88, 0.82, 0.64, 1.0))
    clock_rim = model.material("dark_bronze_rim", rgba=(0.13, 0.10, 0.07, 1.0))
    iron = model.material("blackened_iron", rgba=(0.02, 0.018, 0.015, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_arched_gatehouse_body(), "arched_stone_body", tolerance=0.005),
        material=stone,
        name="arched_stone_body",
    )

    # Slightly wider plinth and two forward buttresses make the wide rectangular
    # stone body read as a medieval gatehouse rather than a plain block.
    tower.visual(
        Box((6.35, 2.25, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=dark_stone,
        name="base_plinth",
    )
    for x in (-2.25, 2.25):
        tower.visual(
            Box((0.58, 0.56, 4.2)),
            origin=Origin(xyz=(x, -1.22, 2.1)),
            material=stone,
            name=f"front_buttress_{'west' if x < 0 else 'east'}",
        )

    # Crenellated parapet: a continuous cap with five merlons on top.
    tower.visual(
        Box((6.25, 2.20, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + 0.17)),
        material=dark_stone,
        name="parapet_cap",
    )
    for i, x in enumerate((-2.55, -1.28, 0.0, 1.28, 2.55)):
        tower.visual(
            Box((0.74, 2.16, PARAPET_HEIGHT)),
            origin=Origin(xyz=(x, 0.0, BODY_HEIGHT + 0.34 + PARAPET_HEIGHT / 2.0)),
            material=stone,
            name=f"crenel_{i}",
        )

    # Portcullis bars sit in the arched opening, with rails extending into the
    # stone jambs so the gate equipment is visibly supported.
    for i, x in enumerate((-0.72, -0.36, 0.0, 0.36, 0.72)):
        tower.visual(
            Box((0.055, 0.11, 2.85)),
            origin=Origin(xyz=(x, -1.08, 1.425)),
            material=iron,
            name=f"portcullis_bar_{i}",
        )
    tower.visual(
        Box((2.42, 0.13, 0.11)),
        origin=Origin(xyz=(0.0, -1.08, 2.88)),
        material=iron,
        name="portcullis_rail",
    )

    # Surface courses and jamb stones add scale without becoming separate parts.
    for i, z in enumerate((1.1, 2.2, 3.35, 4.55, 5.75, 6.95, 8.15)):
        tower.visual(
            Box((5.85, 0.035, 0.045)),
            origin=Origin(xyz=(0.0, -1.015, z)),
            material=mortar,
            name=f"mortar_course_{i}",
        )
    for i, x in enumerate((-1.24, 1.24)):
        tower.visual(
            Box((0.10, 0.055, 3.35)),
            origin=Origin(xyz=(x, -1.035, 1.675)),
            material=mortar,
            name=f"gate_jamb_line_{i}",
        )

    # Clock face and hour ticks are proud of the front wall; all are static
    # tower details, while the hands below are separate continuous joints.
    tower.visual(
        Cylinder(radius=0.95, length=0.065),
        origin=Origin(xyz=(0.0, -1.0325, CLOCK_CENTER[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_rim,
        name="clock_outer_rim",
    )
    tower.visual(
        Cylinder(radius=0.82, length=0.012),
        origin=Origin(xyz=(0.0, -1.069, CLOCK_CENTER[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_ivory,
        name="clock_face",
    )
    for i in range(12):
        angle = i * math.tau / 12.0
        x = 0.68 * math.sin(angle)
        z = CLOCK_CENTER[2] + 0.68 * math.cos(angle)
        long = i % 3 == 0
        tower.visual(
            Box((0.055 if long else 0.035, 0.012, 0.18 if long else 0.11)),
            origin=Origin(xyz=(x, -1.078, z), rpy=(0.0, angle, 0.0)),
            material=clock_rim,
            name=f"hour_tick_{i}",
        )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Box((0.105, 0.014, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=iron,
        name="hour_pointer",
    )
    hour_hand.visual(
        Cylinder(radius=0.105, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_rim,
        name="hour_hub",
    )
    hour_hand.visual(
        Cylinder(radius=0.035, length=0.024),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_rim,
        name="hour_axle",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Box((0.060, 0.014, 0.69)),
        origin=Origin(xyz=(0.0, -0.034, 0.345)),
        material=iron,
        name="minute_pointer",
    )
    minute_hand.visual(
        Cylinder(radius=0.075, length=0.014),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="minute_hub",
    )
    minute_hand.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="minute_axle",
    )

    for child, name, speed in (
        (hour_hand, "tower_to_hour_hand", 0.15),
        (minute_hand, "tower_to_minute_hand", 1.0),
    ):
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=child,
            origin=Origin(xyz=CLOCK_CENTER),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=speed),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    tower_aabb = ctx.part_world_aabb(tower)
    if tower_aabb is not None:
        mins, maxs = tower_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "wide_rectangular_gatehouse_body",
            size[0] > 5.8 and size[1] > 2.0 and size[2] > 9.3,
            details=f"tower size={size!r}",
        )

    for joint, label in ((hour_joint, "hour"), (minute_joint, "minute")):
        ctx.check(
            f"{label}_hand_is_continuous",
            str(joint.articulation_type).lower().endswith("continuous"),
            details=f"joint type={joint.articulation_type!r}",
        )
        ctx.check(
            f"{label}_hand_rotates_about_clock_normal",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis!r}",
        )

    ctx.check(
        "clock_hands_are_coaxial",
        hour_joint.origin.xyz == minute_joint.origin.xyz and hour_joint.axis == minute_joint.axis,
        details=f"hour origin={hour_joint.origin.xyz!r}, minute origin={minute_joint.origin.xyz!r}",
    )
    ctx.expect_contact(
        hour_hand,
        tower,
        elem_a="hour_axle",
        elem_b="clock_face",
        contact_tol=0.002,
        name="hour_hand_axle_seats_in_clock_face",
    )
    ctx.expect_contact(
        minute_hand,
        hour_hand,
        elem_a="minute_axle",
        elem_b="hour_hub",
        contact_tol=0.002,
        name="minute_hand_is_carried_on_coaxial_stack",
    )

    def _element_size(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(object_model.get_part(part_name), elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(float(maxs[i] - mins[i]) for i in range(3))

    at_twelve = _element_size("minute_hand", "minute_pointer")
    with ctx.pose({minute_joint: math.pi / 2.0, hour_joint: math.pi / 2.0}):
        minute_at_three = _element_size("minute_hand", "minute_pointer")
        hour_at_three = _element_size("hour_hand", "hour_pointer")
        ctx.check(
            "minute_hand_reaches_three_oclock",
            minute_at_three is not None and minute_at_three[0] > 0.60 and minute_at_three[2] < 0.12,
            details=f"minute_at_three={minute_at_three!r}",
        )
        ctx.check(
            "hour_hand_reaches_three_oclock",
            hour_at_three is not None and hour_at_three[0] > 0.42 and hour_at_three[2] < 0.16,
            details=f"hour_at_three={hour_at_three!r}",
        )
    ctx.check(
        "minute_hand_starts_at_twelve",
        at_twelve is not None and at_twelve[2] > 0.62 and at_twelve[0] < 0.12,
        details=f"minute_at_twelve={at_twelve!r}",
    )

    return ctx.report()


object_model = build_object_model()
