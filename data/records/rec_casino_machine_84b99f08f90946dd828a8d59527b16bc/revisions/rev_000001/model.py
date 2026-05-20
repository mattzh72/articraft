from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bank_end_slot_machine")

    model.material("cabinet_red", rgba=(0.42, 0.02, 0.035, 1.0))
    model.material("deep_maroon", rgba=(0.14, 0.012, 0.018, 1.0))
    model.material("black_glass", rgba=(0.015, 0.02, 0.028, 1.0))
    model.material("blue_screen", rgba=(0.05, 0.18, 0.36, 1.0))
    model.material("gold_trim", rgba=(0.95, 0.63, 0.12, 1.0))
    model.material("brushed_steel", rgba=(0.70, 0.68, 0.62, 1.0))
    model.material("dark_slot", rgba=(0.002, 0.002, 0.004, 1.0))
    model.material("rubber_black", rgba=(0.01, 0.009, 0.008, 1.0))

    cabinet = model.part("cabinet")

    # Wide, heavy lower end cabinet.
    cabinet.visual(
        Box((1.24, 0.62, 0.90)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material="cabinet_red",
        name="lower_body",
    )
    cabinet.visual(
        Box((1.30, 0.68, 0.10)),
        origin=Origin(xyz=(0.0, 0.02, 0.05)),
        material="deep_maroon",
        name="kick_plinth",
    )
    cabinet.visual(
        Box((1.28, 0.035, 0.065)),
        origin=Origin(xyz=(0.0, -0.333, 0.875)),
        material="gold_trim",
        name="cabinet_belt",
    )

    # Upright screen stack and top sign, deliberately narrower than the base.
    cabinet.visual(
        Box((0.92, 0.36, 0.86)),
        origin=Origin(xyz=(0.0, -0.01, 1.33)),
        material="cabinet_red",
        name="screen_tower",
    )
    cabinet.visual(
        Box((1.02, 0.40, 0.20)),
        origin=Origin(xyz=(0.0, -0.01, 1.86)),
        material="deep_maroon",
        name="top_marquee",
    )
    cabinet.visual(
        Box((0.86, 0.006, 0.14)),
        origin=Origin(xyz=(0.0, -0.212, 1.86)),
        material="gold_trim",
        name="marquee_face",
    )

    screen_frame = mesh_from_geometry(
        BezelGeometry(
            (0.63, 0.285),
            (0.75, 0.385),
            0.028,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=0.020,
            outer_corner_radius=0.030,
            face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
            center=False,
        ),
        "screen_bezel",
    )
    for name, z in (("upper_screen_bezel", 1.52), ("lower_screen_bezel", 1.13)):
        cabinet.visual(
            screen_frame,
            origin=Origin(xyz=(0.0, -0.189, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="gold_trim",
            name=name,
        )
    for name, z in (("upper_screen_glass", 1.52), ("lower_screen_glass", 1.13)):
        cabinet.visual(
            Box((0.61, 0.035, 0.265)),
            origin=Origin(xyz=(0.0, -0.2075, z)),
            material="blue_screen",
            name=name,
        )

    # Front bill/ticket validator panel with two visibly separate openings.
    validator_bezel = mesh_from_geometry(
        BezelGeometry(
            (0.405, 0.075),
            (0.505, 0.145),
            0.026,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=0.012,
            outer_corner_radius=0.019,
            face=BezelFace(style="radiused_step", front_lip=0.003, fillet=0.0015),
            center=False,
        ),
        "validator_bezel",
    )
    for name, z in (("bill_bezel", 0.785), ("ticket_bezel", 0.645)):
        cabinet.visual(
            validator_bezel,
            origin=Origin(xyz=(0.24, -0.309, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_steel",
            name=name,
        )
    cabinet.visual(
        Box((0.39, 0.008, 0.050)),
        origin=Origin(xyz=(0.24, -0.313, 0.785)),
        material="dark_slot",
        name="bill_void",
    )
    cabinet.visual(
        Box((0.39, 0.008, 0.050)),
        origin=Origin(xyz=(0.24, -0.313, 0.645)),
        material="dark_slot",
        name="ticket_void",
    )

    # Control deck details remain fixed to the cabinet.
    cabinet.visual(
        Box((0.80, 0.18, 0.075)),
        origin=Origin(xyz=(-0.15, -0.365, 0.91), rpy=(0.12, 0.0, 0.0)),
        material="deep_maroon",
        name="button_deck",
    )
    for idx, x in enumerate((-0.42, -0.28, -0.14, 0.0, 0.14)):
        cabinet.visual(
            Cylinder(radius=0.035, length=0.018),
            origin=Origin(xyz=(x, -0.462, 0.945), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="gold_trim",
            name=f"play_button_{idx}",
        )

    # Articulated bill flap: child frame is the upper hinge line.
    bill_flap = model.part("bill_flap")
    bill_flap.visual(
        Box((0.335, 0.014, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material="rubber_black",
        name="flap_panel",
    )
    bill_flap.visual(
        Cylinder(radius=0.006, length=0.345),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="hinge_barrel",
    )
    model.articulation(
        "cabinet_to_bill_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=bill_flap,
        origin=Origin(xyz=(0.24, -0.324, 0.813)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=0.95),
    )

    # Articulated ticket flap, on its own lower horizontal hinge.
    ticket_flap = model.part("ticket_flap")
    ticket_flap.visual(
        Box((0.335, 0.014, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material="rubber_black",
        name="flap_panel",
    )
    ticket_flap.visual(
        Cylinder(radius=0.006, length=0.345),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="hinge_barrel",
    )
    model.articulation(
        "cabinet_to_ticket_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=ticket_flap,
        origin=Origin(xyz=(0.24, -0.324, 0.673)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=0.95),
    )

    # Lower service door, hinged vertically along the left cabinet side.
    service_door = model.part("service_door")
    service_door.visual(
        Box((1.13, 0.026, 0.43)),
        origin=Origin(xyz=(0.565, 0.0, 0.215)),
        material="deep_maroon",
        name="door_panel",
    )
    service_door.visual(
        Box((1.05, 0.008, 0.36)),
        origin=Origin(xyz=(0.565, -0.0165, 0.225)),
        material="cabinet_red",
        name="door_inset",
    )
    service_door.visual(
        Cylinder(radius=0.018, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material="brushed_steel",
        name="hinge_barrel",
    )
    service_door.visual(
        Box((0.11, 0.020, 0.055)),
        origin=Origin(xyz=(1.03, -0.026, 0.25)),
        material="brushed_steel",
        name="pull_handle",
    )
    model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_door,
        origin=Origin(xyz=(-0.585, -0.336, 0.080)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    bill_flap = object_model.get_part("bill_flap")
    ticket_flap = object_model.get_part("ticket_flap")
    service_door = object_model.get_part("service_door")
    bill_joint = object_model.get_articulation("cabinet_to_bill_flap")
    ticket_joint = object_model.get_articulation("cabinet_to_ticket_flap")
    service_joint = object_model.get_articulation("cabinet_to_service_door")

    ctx.expect_overlap(
        bill_flap,
        cabinet,
        axes="xz",
        elem_a="flap_panel",
        elem_b="bill_void",
        min_overlap=0.040,
        name="bill flap covers upper opening",
    )
    ctx.expect_gap(
        cabinet,
        bill_flap,
        axis="y",
        positive_elem="bill_void",
        negative_elem="flap_panel",
        max_gap=0.040,
        max_penetration=0.0,
        name="bill flap sits proud of opening",
    )
    ctx.expect_overlap(
        ticket_flap,
        cabinet,
        axes="xz",
        elem_a="flap_panel",
        elem_b="ticket_void",
        min_overlap=0.040,
        name="ticket flap covers lower opening",
    )
    ctx.expect_gap(
        cabinet,
        ticket_flap,
        axis="y",
        positive_elem="ticket_void",
        negative_elem="flap_panel",
        max_gap=0.040,
        max_penetration=0.0,
        name="ticket flap sits proud of opening",
    )
    ctx.expect_overlap(
        service_door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="lower_body",
        min_overlap=0.40,
        name="service door spans lower cabinet front",
    )
    ctx.expect_gap(
        cabinet,
        service_door,
        axis="y",
        positive_elem="lower_body",
        negative_elem="door_panel",
        max_gap=0.020,
        max_penetration=0.0,
        name="service door is just proud of cabinet",
    )

    bill_closed = ctx.part_world_aabb(bill_flap)
    ticket_closed = ctx.part_world_aabb(ticket_flap)
    door_closed = ctx.part_world_aabb(service_door)
    with ctx.pose({bill_joint: 0.75, ticket_joint: 0.75, service_joint: 1.25}):
        bill_open = ctx.part_world_aabb(bill_flap)
        ticket_open = ctx.part_world_aabb(ticket_flap)
        door_open = ctx.part_world_aabb(service_door)

    ctx.check(
        "bill flap opens outward on horizontal hinge",
        bill_closed is not None
        and bill_open is not None
        and bill_open[0][1] < bill_closed[0][1] - 0.015,
        details=f"closed={bill_closed}, open={bill_open}",
    )
    ctx.check(
        "ticket flap opens outward on horizontal hinge",
        ticket_closed is not None
        and ticket_open is not None
        and ticket_open[0][1] < ticket_closed[0][1] - 0.015,
        details=f"closed={ticket_closed}, open={ticket_open}",
    )
    ctx.check(
        "service door swings out from side hinge",
        door_closed is not None
        and door_open is not None
        and door_open[0][1] < door_closed[0][1] - 0.20,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()
