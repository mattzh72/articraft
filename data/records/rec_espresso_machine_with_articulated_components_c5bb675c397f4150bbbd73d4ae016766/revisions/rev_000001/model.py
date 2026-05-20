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
    model = ArticulatedObject(name="office_espresso_machine")

    body_color = model.material("body_color", rgba=(0.18, 0.19, 0.20, 1.0))
    fascia_metal = model.material("fascia_metal", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.67, 0.69, 0.71, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.78, 0.79, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.34, 0.392, 0.372)),
        origin=Origin(xyz=(0.0, 0.0, 0.376)),
        material=body_color,
        name="upper_shell",
    )
    body.visual(
        Box((0.22, 0.08, 0.19)),
        origin=Origin(xyz=(-0.06, -0.156, 0.095)),
        material=body_color,
        name="left_leg",
    )
    body.visual(
        Box((0.22, 0.08, 0.19)),
        origin=Origin(xyz=(-0.06, 0.156, 0.095)),
        material=body_color,
        name="right_leg",
    )
    body.visual(
        Box((0.11, 0.232, 0.19)),
        origin=Origin(xyz=(-0.115, 0.0, 0.095)),
        material=body_color,
        name="rear_plinth",
    )
    body.visual(
        Box((0.016, 0.304, 0.17)),
        origin=Origin(xyz=(0.178, 0.0, 0.335)),
        material=fascia_metal,
        name="front_fascia",
    )
    body.visual(
        Box((0.08, 0.18, 0.14)),
        origin=Origin(xyz=(0.130, -0.070, 0.165)),
        material=fascia_metal,
        name="control_pod",
    )
    body.visual(
        Box((0.30, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.564)),
        material=fascia_metal,
        name="top_cap",
    )
    body.visual(
        Cylinder(radius=0.044, length=0.054),
        origin=Origin(xyz=(0.197, 0.0, 0.278), rpy=(0.0, pi / 2.0, 0.0)),
        material=fascia_metal,
        name="group_body",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.205, 0.0, 0.237)),
        material=wand_metal,
        name="group_face",
    )
    body.visual(
        Box((0.026, 0.090, 0.030)),
        origin=Origin(xyz=(0.190, 0.0, 0.257)),
        material=fascia_metal,
        name="group_mount",
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=tray_metal,
        name="basket_flange",
    )
    portafilter.visual(
        Cylinder(radius=0.032, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=tray_metal,
        name="basket",
    )
    portafilter.visual(
        Box((0.018, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.059)),
        material=tray_metal,
        name="spout_block",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.11),
        origin=Origin(xyz=(0.055, 0.0, -0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="handle_shaft",
    )
    portafilter.visual(
        Box((0.12, 0.026, 0.026)),
        origin=Origin(xyz=(0.125, 0.0, -0.040)),
        material=dark_trim,
        name="handle_grip",
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.014, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=wand_metal,
        name="pivot_barrel",
    )
    steam_wand.visual(
        Cylinder(radius=0.007, length=0.064),
        origin=Origin(xyz=(0.0, -0.032, -0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wand_metal,
        name="wand_arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.21),
        origin=Origin(xyz=(0.0, -0.064, -0.111)),
        material=wand_metal,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, -0.064, -0.230)),
        material=wand_metal,
        name="wand_tip",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.22, 0.024, 0.30)),
        origin=Origin(xyz=(-0.11, -0.012, 0.0)),
        material=fascia_metal,
        name="door_panel",
    )
    service_door.visual(
        Box((0.016, 0.032, 0.090)),
        origin=Origin(xyz=(-0.165, 0.016, 0.0)),
        material=dark_trim,
        name="door_pull",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wand_metal,
        name="shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="knob_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=fascia_metal,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.008, 0.010, 0.020)),
        origin=Origin(xyz=(0.050, 0.018, 0.0)),
        material=fascia_metal,
        name="pointer",
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.20, 0.24, 0.025)),
        origin=Origin(xyz=(0.09, 0.0, 0.0125)),
        material=dark_trim,
        name="tray_pan",
    )
    drip_tray.visual(
        Box((0.18, 0.22, 0.006)),
        origin=Origin(xyz=(0.09, 0.0, 0.028)),
        material=tray_metal,
        name="tray_grate",
    )
    drip_tray.visual(
        Box((0.012, 0.22, 0.04)),
        origin=Origin(xyz=(0.194, 0.0, 0.020)),
        material=tray_metal,
        name="tray_lip",
    )

    model.articulation(
        "portafilter_mount",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.208, 0.0, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-0.60, upper=0.30),
    )
    model.articulation(
        "steam_wand_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.200, 0.145, 0.338)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "service_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(0.145, 0.220, 0.330)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "selector_knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.170, -0.110, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "drip_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.11),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    service_door = object_model.get_part("service_door")
    selector_knob = object_model.get_part("selector_knob")
    drip_tray = object_model.get_part("drip_tray")

    portafilter_mount = object_model.get_articulation("portafilter_mount")
    steam_wand_pivot = object_model.get_articulation("steam_wand_pivot")
    service_door_hinge = object_model.get_articulation("service_door_hinge")
    selector_knob_turn = object_model.get_articulation("selector_knob_turn")
    drip_tray_slide = object_model.get_articulation("drip_tray_slide")

    ctx.expect_gap(
        service_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="upper_shell",
        max_gap=0.001,
        max_penetration=0.00001,
        name="service door sits flush against the side shell",
    )
    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_face",
        negative_elem="basket",
        min_gap=0.003,
        max_gap=0.012,
        name="portafilter basket nests just below the group head",
    )
    ctx.expect_overlap(
        portafilter,
        body,
        axes="xy",
        elem_a="basket",
        elem_b="group_face",
        min_overlap=0.060,
        name="portafilter stays centered under the brew group",
    )
    ctx.expect_overlap(
        drip_tray,
        body,
        axes="y",
        elem_a="tray_pan",
        elem_b="front_fascia",
        min_overlap=0.18,
        name="drip tray spans the brew area width",
    )
    ctx.expect_gap(
        body,
        drip_tray,
        axis="z",
        positive_elem="group_face",
        negative_elem="tray_grate",
        min_gap=0.18,
        max_gap=0.25,
        name="group head clears the drip tray deck",
    )

    closed_door = ctx.part_element_world_aabb(service_door, elem="door_panel")
    with ctx.pose({service_door_hinge: 1.10}):
        open_door = ctx.part_element_world_aabb(service_door, elem="door_panel")
    ctx.check(
        "service door swings outward",
        closed_door is not None
        and open_door is not None
        and open_door[1][1] > closed_door[1][1] + 0.08,
        details=f"closed={closed_door}, open={open_door}",
    )

    locked_handle = ctx.part_element_world_aabb(portafilter, elem="handle_grip")
    with ctx.pose({portafilter_mount: -0.55}):
        unlocked_handle = ctx.part_element_world_aabb(portafilter, elem="handle_grip")
    ctx.check(
        "portafilter rotates around the brew axis",
        locked_handle is not None
        and unlocked_handle is not None
        and unlocked_handle[0][1] < locked_handle[0][1] - 0.05,
        details=f"locked={locked_handle}, unlocked={unlocked_handle}",
    )

    parked_wand = ctx.part_element_world_aabb(steam_wand, elem="wand_tube")
    with ctx.pose({steam_wand_pivot: 0.95}):
        swung_wand = ctx.part_element_world_aabb(steam_wand, elem="wand_tube")
    ctx.check(
        "steam wand swings out from the front corner",
        parked_wand is not None
        and swung_wand is not None
        and swung_wand[1][0] > parked_wand[1][0] + 0.04,
        details=f"parked={parked_wand}, swung={swung_wand}",
    )

    rest_pointer = ctx.part_element_world_aabb(selector_knob, elem="pointer")
    with ctx.pose({selector_knob_turn: 1.10}):
        turned_pointer = ctx.part_element_world_aabb(selector_knob, elem="pointer")
    ctx.check(
        "selector knob turns around its front-facing shaft",
        rest_pointer is not None
        and turned_pointer is not None
        and turned_pointer[1][2] > rest_pointer[1][2] + 0.010,
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    tray_rest = ctx.part_world_position(drip_tray)
    with ctx.pose({drip_tray_slide: 0.10}):
        tray_extended = ctx.part_world_position(drip_tray)
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="y",
            elem_a="tray_pan",
            elem_b="front_fascia",
            min_overlap=0.18,
            name="extended drip tray stays centered under the front fascia",
        )
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.08,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


object_model = build_object_model()
