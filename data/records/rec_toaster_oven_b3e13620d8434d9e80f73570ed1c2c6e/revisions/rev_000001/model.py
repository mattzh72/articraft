from __future__ import annotations

import math

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
    model = ArticulatedObject(name="field_service_toaster_oven")

    steel = model.material("brushed_service_steel", rgba=(0.58, 0.58, 0.55, 1.0))
    dark = model.material("black_powdercoat", rgba=(0.03, 0.035, 0.035, 1.0))
    warm = model.material("heat_stained_steel", rgba=(0.72, 0.48, 0.30, 1.0))
    glass = model.material("smoked_service_glass", rgba=(0.12, 0.17, 0.20, 0.42))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    brass = model.material("oiled_bronze_bushing", rgba=(0.55, 0.39, 0.18, 1.0))
    label = model.material("engraved_white_fill", rgba=(0.92, 0.88, 0.74, 1.0))

    body = model.part("body")

    # Heavy rectangular shell, deliberately built from serviceable panels and
    # welded rails rather than a single anonymous block.  X is depth
    # (front=-X), Y is width, and Z is up.
    body.visual(Box((0.42, 0.52, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=dark, name="bottom_panel")
    body.visual(Box((0.42, 0.035, 0.29)), origin=Origin(xyz=(0.0, -0.2425, 0.145)), material=steel, name="side_wall_0")
    body.visual(Box((0.42, 0.035, 0.29)), origin=Origin(xyz=(0.0, 0.2425, 0.145)), material=steel, name="side_wall_1")
    body.visual(Box((0.035, 0.52, 0.29)), origin=Origin(xyz=(0.1925, 0.0, 0.145)), material=steel, name="rear_panel")
    body.visual(Box((0.035, 0.52, 0.035)), origin=Origin(xyz=(-0.1925, 0.0, 0.0175)), material=steel, name="front_base_rail")
    body.visual(Box((0.035, 0.52, 0.050)), origin=Origin(xyz=(-0.1925, 0.0, 0.274)), material=steel, name="front_lintel")
    body.visual(Box((0.035, 0.035, 0.225)), origin=Origin(xyz=(-0.1925, -0.2175, 0.1475)), material=steel, name="front_jamb")
    body.visual(Box((0.035, 0.125, 0.245)), origin=Origin(xyz=(-0.1925, 0.1825, 0.1525)), material=steel, name="control_panel")

    # The top has a real opening covered by a hinged service cover.
    body.visual(Box((0.42, 0.040, 0.030)), origin=Origin(xyz=(0.0, -0.230, 0.300)), material=steel, name="top_side_rail_0")
    body.visual(Box((0.42, 0.040, 0.030)), origin=Origin(xyz=(0.0, 0.230, 0.300)), material=steel, name="top_side_rail_1")
    body.visual(Box((0.055, 0.52, 0.030)), origin=Origin(xyz=(0.1725, 0.0, 0.300)), material=steel, name="top_rear_rail")
    body.visual(Box((0.055, 0.52, 0.030)), origin=Origin(xyz=(-0.1725, 0.0, 0.300)), material=steel, name="top_front_rail")

    # Internal rack ledges and crumb-tray ledges are welded to the side walls.
    for y, suffix in [(-0.210, "0"), (0.210, "1")]:
        body.visual(Box((0.310, 0.055, 0.014)), origin=Origin(xyz=(0.015, y, 0.126)), material=steel, name=f"rack_ledge_{suffix}")
        body.visual(Box((0.315, 0.060, 0.012)), origin=Origin(xyz=(0.015, y, 0.041)), material=steel, name=f"tray_ledge_{suffix}")

    # Replaceable heater rods have obvious side-wall terminations.
    for x, z, suffix in [(-0.090, 0.078, "lower_front"), (0.105, 0.078, "lower_rear"), (-0.075, 0.225, "upper_front"), (0.095, 0.225, "upper_rear")]:
        body.visual(
            Cylinder(radius=0.006, length=0.455),
            origin=Origin(xyz=(x, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm,
            name=f"heater_{suffix}",
        )
        body.visual(
            Box((0.018, 0.035, 0.020)),
            origin=Origin(xyz=(x, -0.223, z)),
            material=brass,
            name=f"heater_socket_{suffix}_0",
        )
        body.visual(
            Box((0.018, 0.035, 0.020)),
            origin=Origin(xyz=(x, 0.223, z)),
            material=brass,
            name=f"heater_socket_{suffix}_1",
        )

    # Door hinge bearing blocks are chunky and connected to the front base rail.
    for y, suffix in [(-0.120, "0"), (0.045, "1")]:
        body.visual(Box((0.026, 0.055, 0.064)), origin=Origin(xyz=(-0.197, y, 0.054)), material=steel, name=f"door_hinge_block_{suffix}")
        body.visual(Box((0.024, 0.055, 0.014)), origin=Origin(xyz=(-0.198, y, 0.070)), material=brass, name=f"door_bearing_face_{suffix}")

    # Shaft collars and tick marks on the serviceable front control panel.
    knob_layout = [
        ("temperature", 0.205, 0.235, 0.022),
        ("mode", 0.205, 0.155, 0.021),
        ("timer", 0.205, 0.075, 0.022),
    ]
    for knob_name, y, z, radius in knob_layout:
        body.visual(
            Cylinder(radius=radius, length=0.024),
            origin=Origin(xyz=(-0.217, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"{knob_name}_bushing",
        )
        for i, dz in enumerate([-0.034, 0.034]):
            body.visual(
                Box((0.006, 0.018, 0.004)),
                origin=Origin(xyz=(-0.212, y, z + dz)),
                material=label,
                name=f"{knob_name}_tick_{i}",
            )

    # Captive panel screws make the maintenance intent legible.
    for y, z, suffix in [(0.132, 0.255, "upper_0"), (0.233, 0.255, "upper_1"), (0.132, 0.050, "lower_0"), (0.233, 0.050, "lower_1")]:
        body.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(-0.208, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"panel_screw_{suffix}",
        )

    # Rear hinge brackets for the top service cover.
    for y, suffix in [(-0.115, "0"), (0.115, "1")]:
        body.visual(Box((0.026, 0.070, 0.040)), origin=Origin(xyz=(0.160, y, 0.315)), material=steel, name=f"service_hinge_block_{suffix}")
        body.visual(Box((0.026, 0.070, 0.010)), origin=Origin(xyz=(0.160, y, 0.333)), material=brass, name=f"service_bearing_face_{suffix}")

    door = model.part("door")
    door_width = 0.300
    door_height = 0.185
    door.visual(Box((0.028, door_width, 0.032)), origin=Origin(xyz=(-0.014, 0.0, 0.016)), material=steel, name="bottom_rail")
    door.visual(Box((0.026, door_width, 0.030)), origin=Origin(xyz=(-0.013, 0.0, door_height - 0.015)), material=steel, name="top_rail")
    door.visual(Box((0.026, 0.028, door_height)), origin=Origin(xyz=(-0.013, -door_width / 2.0 + 0.014, door_height / 2.0)), material=steel, name="side_rail_0")
    door.visual(Box((0.026, 0.028, door_height)), origin=Origin(xyz=(-0.013, door_width / 2.0 - 0.014, door_height / 2.0)), material=steel, name="side_rail_1")
    door.visual(Box((0.006, 0.220, 0.112)), origin=Origin(xyz=(-0.020, -0.010, 0.100)), material=glass, name="glass_pane")
    for y, suffix in [(-0.120, "0"), (0.045, "1")]:
        door.visual(
            Cylinder(radius=0.012, length=0.062),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"hinge_barrel_{suffix}",
        )
    door.visual(Cylinder(radius=0.013, length=0.185), origin=Origin(xyz=(-0.060, -0.060, 0.116), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=rubber, name="front_handle")
    for y, suffix in [(-0.125, "0"), (0.005, "1")]:
        door.visual(Box((0.050, 0.024, 0.030)), origin=Origin(xyz=(-0.040, y, 0.116)), material=steel, name=f"handle_standoff_{suffix}")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.222, 0.0, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    service_cover = model.part("service_cover")
    service_cover.visual(Box((0.280, 0.395, 0.025)), origin=Origin(xyz=(-0.140, 0.0, 0.0125)), material=steel, name="cover_panel")
    service_cover.visual(Cylinder(radius=0.012, length=0.330), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brass, name="hinge_barrel")
    for y, suffix in [(-0.140, "0"), (0.140, "1")]:
        service_cover.visual(Box((0.215, 0.018, 0.016)), origin=Origin(xyz=(-0.145, y, 0.031)), material=dark, name=f"stiffener_{suffix}")
    service_cover.visual(Cylinder(radius=0.010, length=0.220), origin=Origin(xyz=(-0.150, 0.0, 0.062), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=rubber, name="service_handle")
    for y, suffix in [(-0.085, "0"), (0.085, "1")]:
        service_cover.visual(Box((0.028, 0.020, 0.050)), origin=Origin(xyz=(-0.150, y, 0.040)), material=steel, name=f"handle_post_{suffix}")
    for x, y, suffix in [(-0.045, -0.160, "0"), (-0.225, -0.160, "1"), (-0.045, 0.160, "2"), (-0.225, 0.160, "3")]:
        service_cover.visual(Cylinder(radius=0.007, length=0.006), origin=Origin(xyz=(x, y, 0.0275)), material=dark, name=f"cover_screw_{suffix}")

    model.articulation(
        "service_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_cover,
        origin=Origin(xyz=(0.135, 0.0, 0.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=1.15),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(Box((0.270, 0.300, 0.012)), origin=Origin(xyz=(0.155, -0.045, 0.0)), material=steel, name="tray_pan")
    for y, suffix in [(-0.190, "0"), (0.100, "1")]:
        crumb_tray.visual(Box((0.270, 0.018, 0.020)), origin=Origin(xyz=(0.155, y, 0.010)), material=steel, name=f"tray_side_lip_{suffix}")
    crumb_tray.visual(Box((0.032, 0.080, 0.034)), origin=Origin(xyz=(-0.055, -0.040, -0.030)), material=dark, name="tray_pull")
    crumb_tray.visual(Box((0.080, 0.080, 0.014)), origin=Origin(xyz=(-0.005, -0.040, -0.008)), material=steel, name="tray_pull_bridge")

    model.articulation(
        "crumb_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crumb_tray,
        origin=Origin(xyz=(-0.185, 0.0, 0.048)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    knob_specs = [
        ("temperature_knob", "temperature", 0.205, 0.235, 0.034, 0.032, 4.8),
        ("mode_knob", "mode", 0.205, 0.155, 0.032, 0.030, 3.2),
        ("timer_knob", "timer", 0.205, 0.075, 0.034, 0.032, 5.4),
    ]
    for part_name, stem, y, z, radius, depth, upper in knob_specs:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=radius, length=depth),
            origin=Origin(xyz=(-0.037, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="knob_cap",
        )
        knob.visual(Box((0.008, 0.006, radius * 1.45)), origin=Origin(xyz=(-0.056, 0.0, radius * 0.18)), material=label, name="pointer_rib")
        model.articulation(
            f"{stem}_shaft",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(-0.220, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.2, velocity=6.0, lower=0.0, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    service_cover = object_model.get_part("service_cover")
    crumb_tray = object_model.get_part("crumb_tray")
    door_hinge = object_model.get_articulation("door_hinge")
    cover_hinge = object_model.get_articulation("service_cover_hinge")
    tray_slide = object_model.get_articulation("crumb_tray_slide")

    # Each knob intentionally carries a metal shaft through a solid service-panel
    # proxy and bronze bushing.  The scoped allowances are paired with exact
    # retained-insertion and centering checks.
    for knob_name, stem in [
        ("temperature_knob", "temperature"),
        ("mode_knob", "mode"),
        ("timer_knob", "timer"),
    ]:
        knob = object_model.get_part(knob_name)
        ctx.allow_overlap(
            body,
            knob,
            elem_a="control_panel",
            elem_b="shaft",
            reason="The knob shaft intentionally passes through the solid control-panel proxy.",
        )
        ctx.allow_overlap(
            body,
            knob,
            elem_a=f"{stem}_bushing",
            elem_b="shaft",
            reason="The bronze bushing is a simplified solid collar around the rotating shaft.",
        )
        ctx.expect_within(
            knob,
            body,
            axes="yz",
            inner_elem="shaft",
            outer_elem=f"{stem}_bushing",
            margin=0.001,
            name=f"{stem} shaft centered in bushing",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="x",
            elem_a="shaft",
            elem_b="control_panel",
            min_overlap=0.018,
            name=f"{stem} shaft penetrates control panel",
        )

    ctx.expect_overlap(door, body, axes="yz", min_overlap=0.12, name="door covers oven opening in closed pose")

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward and down",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.055
        and open_door_aabb[1][2] < closed_door_aabb[1][2] + 0.020,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_cover_aabb = ctx.part_world_aabb(service_cover)
    with ctx.pose({cover_hinge: 1.05}):
        open_cover_aabb = ctx.part_world_aabb(service_cover)
    ctx.check(
        "service cover hinges upward for maintenance",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.070,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    ctx.expect_within(crumb_tray, body, axes="y", margin=0.010, name="crumb tray fits between side rails")
    ctx.expect_overlap(
        crumb_tray,
        body,
        axes="x",
        elem_a="tray_pan",
        elem_b="bottom_panel",
        min_overlap=0.18,
        name="crumb tray retained when stowed",
    )
    tray_rest = ctx.part_world_position(crumb_tray)
    with ctx.pose({tray_slide: 0.180}):
        ctx.expect_overlap(
            crumb_tray,
            body,
            axes="x",
            elem_a="tray_pan",
            elem_b="bottom_panel",
            min_overlap=0.085,
            name="crumb tray retains insertion at service extension",
        )
        tray_extended = ctx.part_world_position(crumb_tray)
    ctx.check(
        "crumb tray slides forward for replacement",
        tray_rest is not None and tray_extended is not None and tray_extended[0] < tray_rest[0] - 0.150,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


object_model = build_object_model()
