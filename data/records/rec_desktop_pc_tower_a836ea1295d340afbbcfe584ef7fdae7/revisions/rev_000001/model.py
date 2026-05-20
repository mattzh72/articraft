from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    FanRotorShroud,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_pc_tower")

    # Real mid-tower proportions, in meters.
    depth = 0.480
    width = 0.220
    height = 0.500

    black = model.material("satin_black_powdercoat", rgba=(0.015, 0.016, 0.018, 1.0))
    dark = model.material("dark_interior_metal", rgba=(0.030, 0.032, 0.036, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.055, 0.058, 0.065, 1.0))
    mesh_black = model.material("fine_black_mesh", rgba=(0.005, 0.006, 0.008, 1.0))
    glass = model.material("smoked_tempered_glass", rgba=(0.10, 0.16, 0.20, 0.38))
    rubber = model.material("matte_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    blue = model.material("blue_power_led", rgba=(0.15, 0.45, 1.0, 1.0))
    port_blue = model.material("usb_blue_insert", rgba=(0.05, 0.22, 0.75, 1.0))
    port_black = model.material("port_black_insert", rgba=(0.0, 0.0, 0.0, 1.0))
    io_silver = model.material("brushed_io_shield", rgba=(0.55, 0.57, 0.58, 1.0))
    gpu_metal = model.material("brushed_slot_covers", rgba=(0.35, 0.36, 0.37, 1.0))
    copper = model.material("copper_heatpipe", rgba=(0.73, 0.33, 0.12, 1.0))
    pcb = model.material("matte_black_pcb", rgba=(0.010, 0.035, 0.026, 1.0))

    chassis = model.part("chassis")

    def box(
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material: Material | str,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        chassis.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Main hollow sheet-metal structure: top, bottom, rear, front rails and the
    # open left side that receives the hinged glass panel.
    box("bottom_tray", (depth, width, 0.018), (0.0, 0.0, 0.009), black)
    box("top_panel", (depth, width, 0.016), (0.0, 0.0, height - 0.008), black)
    box("right_side_panel", (depth, 0.014, height - 0.030), (0.0, width / 2 - 0.007, height / 2), black)
    box("rear_panel", (0.014, width, height), (depth / 2 - 0.007, 0.0, height / 2), black)
    box("front_left_post", (0.020, 0.018, height), (-depth / 2 + 0.010, -width / 2 + 0.009, height / 2), black)
    box("front_right_post", (0.020, 0.018, height), (-depth / 2 + 0.010, width / 2 - 0.009, height / 2), black)
    box("front_top_crossbar", (0.022, width, 0.030), (-depth / 2 + 0.011, 0.0, height - 0.015), black)
    box("front_bottom_crossbar", (0.022, width, 0.030), (-depth / 2 + 0.011, 0.0, 0.015), black)
    box("front_io_shelf", (0.030, width - 0.030, 0.055), (-depth / 2 + 0.015, 0.0, height - 0.060), charcoal)
    box("left_top_rail", (depth, 0.008, 0.024), (0.0, -width / 2 + 0.004, height - 0.044), black)
    box("left_bottom_rail", (depth, 0.008, 0.030), (0.0, -width / 2 + 0.004, 0.060), black)
    box("left_front_latch_rail", (0.026, 0.008, height - 0.100), (-depth / 2 + 0.013, -width / 2 + 0.004, height / 2), black)
    box("left_rear_hinge_rail", (0.032, 0.008, height - 0.090), (depth / 2 - 0.024, -width / 2 + 0.004, height / 2), black)

    # Four raised feet make the tower feel grounded on the desk.
    for i, x in enumerate((-0.165, 0.165)):
        for j, y in enumerate((-0.075, 0.075)):
            box(f"foot_{i}_{j}", (0.075, 0.034, 0.014), (x, y, -0.007), rubber)

    # Top exhaust and front intake use real perforation/slot meshes, not flat
    # decals.  The mesh faces are seated against the sheet-metal panels.
    top_vent = PerforatedPanelGeometry(
        (0.300, 0.145),
        0.004,
        hole_diameter=0.006,
        pitch=(0.012, 0.012),
        frame=0.014,
        corner_radius=0.010,
        stagger=True,
    )
    chassis.visual(
        mesh_from_geometry(top_vent, "top_vent"),
        origin=Origin(xyz=(-0.015, 0.0, height + 0.002)),
        material=mesh_black,
        name="top_vent",
    )

    # Rear I/O area, PSU grille and expansion slots are proud details on the
    # rear panel surface, arranged like an ATX tower.
    rear_x = depth / 2 + 0.0020
    box("rear_io_shield", (0.004, 0.082, 0.125), (rear_x, -0.048, 0.348), io_silver)
    for row, z in enumerate((0.383, 0.357, 0.331)):
        box(f"rear_usb_{row}_0", (0.006, 0.020, 0.010), (rear_x + 0.003, -0.070, z), port_blue)
        box(f"rear_usb_{row}_1", (0.006, 0.020, 0.010), (rear_x + 0.003, -0.043, z), port_blue)
    box("rear_ethernet", (0.006, 0.030, 0.018), (rear_x + 0.003, -0.056, 0.300), port_black)
    for idx, z in enumerate((0.275, 0.253, 0.231, 0.209, 0.187, 0.165, 0.143)):
        box(f"slot_cover_{idx}", (0.004, 0.120, 0.012), (rear_x, 0.030, z), gpu_metal)
    rear_exhaust = VentGrilleGeometry(
        (0.100, 0.100),
        frame=0.010,
        face_thickness=0.003,
        duct_depth=0.012,
        slat_pitch=0.014,
        slat_width=0.006,
        slat_angle_deg=25.0,
        corner_radius=0.006,
        slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=1, divider_width=0.004),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
        mounts=VentGrilleMounts(style="holes", inset=0.010, hole_diameter=0.003),
        sleeve=VentGrilleSleeve(style="short", depth=0.010, wall=0.002),
    )
    chassis.visual(
        mesh_from_geometry(rear_exhaust, "rear_exhaust_grille"),
        origin=Origin(xyz=(rear_x + 0.006, 0.060, 0.365), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mesh_black,
        name="rear_exhaust_grille",
    )
    psu_grille = PerforatedPanelGeometry(
        (0.118, 0.070),
        0.003,
        hole_diameter=0.004,
        pitch=(0.009, 0.009),
        frame=0.008,
        corner_radius=0.004,
        stagger=True,
    )
    chassis.visual(
        mesh_from_geometry(psu_grille, "psu_rear_grille"),
        origin=Origin(xyz=(depth / 2 + 0.0015, -0.052, 0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mesh_black,
        name="psu_rear_grille",
    )

    # Interior parts visible through the glass panel: motherboard tray, PSU
    # shroud, GPU silhouette, cable channel and a CPU cooler.
    box("motherboard_tray", (0.365, 0.005, 0.340), (0.050, 0.063, 0.285), dark)
    box("motherboard", (0.305, 0.006, 0.285), (0.040, 0.056, 0.300), pcb)
    box("psu_shroud", (0.350, 0.095, 0.065), (0.035, -0.022, 0.0505), charcoal)
    box("gpu_body", (0.230, 0.040, 0.045), (0.055, -0.010, 0.215), dark)
    box("gpu_rear_bracket", (0.056, 0.025, 0.045), (0.198, -0.010, 0.215), gpu_metal)
    box("gpu_edge_light", (0.175, 0.003, 0.007), (0.040, -0.0315, 0.240), blue)
    box("cable_channel", (0.025, 0.028, 0.275), (0.190, 0.040, 0.255), charcoal)
    box("cpu_coldplate", (0.055, 0.014, 0.055), (0.010, 0.044, 0.340), gpu_metal)
    for k, x in enumerate((-0.018, 0.0, 0.018)):
        chassis.visual(
            Cylinder(radius=0.004, length=0.115),
            origin=Origin(xyz=(x, 0.034, 0.340 + (k - 1) * 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=copper,
            name=f"heatpipe_{k}",
        )
    for k, x in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        box(f"cooler_fin_{k}", (0.004, 0.038, 0.075), (x, 0.028, 0.363), gpu_metal)
    box("cooler_fin_bridge", (0.075, 0.030, 0.008), (0.0, 0.036, 0.328), gpu_metal)

    # The front has two fan silhouettes mounted behind the opening.  They are
    # static visual detail inside the rigid chassis rather than serviceable fans.
    front_fan = FanRotorGeometry(
        0.062,
        0.018,
        9,
        thickness=0.009,
        blade_pitch_deg=31.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.14),
        hub=FanRotorHub(style="spinner", bore_diameter=0.004),
        shroud=FanRotorShroud(thickness=0.004, depth=0.010, clearance=0.001, lip_depth=0.001),
    )
    for idx, z in enumerate((0.205, 0.335)):
        chassis.visual(
            mesh_from_geometry(front_fan, f"front_fan_{idx}"),
            origin=Origin(xyz=(-depth / 2 + 0.023, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=mesh_black,
            name=f"front_fan_{idx}",
        )
        box(f"fan_mount_bar_{idx}", (0.006, 0.155, 0.008), (-depth / 2 + 0.020, 0.0, z), charcoal)
        box(f"front_fan_hub_mount_{idx}", (0.012, 0.042, 0.042), (-depth / 2 + 0.020, 0.0, z), charcoal)
    for side, y in enumerate((-0.082, 0.082)):
        box(f"fan_side_rail_{side}", (0.006, 0.020, 0.285), (-depth / 2 + 0.020, y, 0.270), charcoal)

    # Hinged tempered-glass side panel.
    side_panel = model.part("side_panel")
    panel_depth = 0.445
    panel_height = 0.430
    side_panel.visual(
        Box((panel_depth - 0.036, 0.004, panel_height - 0.036)),
        origin=Origin(xyz=(-panel_depth / 2 - 0.006, -0.006, 0.0)),
        material=glass,
        name="glass_pane",
    )
    side_panel.visual(Box((panel_depth, 0.008, 0.018)), origin=Origin(xyz=(-panel_depth / 2, -0.005, panel_height / 2 - 0.009)), material=black, name="top_frame")
    side_panel.visual(Box((panel_depth, 0.008, 0.018)), origin=Origin(xyz=(-panel_depth / 2, -0.005, -panel_height / 2 + 0.009)), material=black, name="bottom_frame")
    side_panel.visual(Box((0.018, 0.008, panel_height)), origin=Origin(xyz=(-0.009, -0.005, 0.0)), material=black, name="hinge_frame")
    side_panel.visual(Box((0.018, 0.008, panel_height)), origin=Origin(xyz=(-panel_depth + 0.009, -0.005, 0.0)), material=black, name="latch_frame")
    side_panel.visual(
        Cylinder(radius=0.0055, length=panel_height - 0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )
    side_panel.visual(Box((0.018, 0.010, 0.070)), origin=Origin(xyz=(-panel_depth + 0.012, -0.014, 0.000)), material=charcoal, name="pull_tab")
    side_hinge = model.articulation(
        "chassis_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(depth / 2 - 0.024, -width / 2 - 0.0055, height / 2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    # Hinged front intake/filter door.  The slotted mesh is part of the door and
    # clears the fixed front fan silhouettes behind it.
    front_door = model.part("front_door")
    door_width = 0.188
    door_height = 0.355
    door_center_z = 0.270
    front_door.visual(Box((0.007, door_width, 0.018)), origin=Origin(xyz=(-0.004, door_width / 2, door_height / 2 - 0.009)), material=black, name="top_frame")
    front_door.visual(Box((0.007, door_width, 0.018)), origin=Origin(xyz=(-0.004, door_width / 2, -door_height / 2 + 0.009)), material=black, name="bottom_frame")
    front_door.visual(Box((0.007, 0.018, door_height)), origin=Origin(xyz=(-0.004, 0.009, 0.0)), material=black, name="hinge_stile")
    front_door.visual(Box((0.007, 0.018, door_height)), origin=Origin(xyz=(-0.004, door_width - 0.009, 0.0)), material=black, name="latch_stile")
    door_mesh = SlotPatternPanelGeometry(
        (door_height - 0.018, door_width - 0.018),
        0.004,
        slot_size=(0.030, 0.0045),
        pitch=(0.040, 0.012),
        frame=0.012,
        corner_radius=0.006,
        slot_angle_deg=0.0,
        stagger=True,
    )
    front_door.visual(
        mesh_from_geometry(door_mesh, "front_filter_mesh"),
        origin=Origin(xyz=(-0.006, door_width / 2, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mesh_black,
        name="front_filter_mesh",
    )
    front_door.visual(Box((0.006, 0.010, 0.075)), origin=Origin(xyz=(-0.010, door_width - 0.018, 0.0)), material=charcoal, name="recessed_pull")
    front_hinge = model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(-depth / 2 + 0.0005, -door_width / 2 - 0.009, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.65),
    )

    # Distinct top-front controls are authored as moving push buttons.
    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=blue,
        name="button_cap",
    )
    power_joint = model.articulation(
        "chassis_to_power_button",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=power_button,
        origin=Origin(xyz=(-0.170, -0.045, height)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    reset_button = model.part("reset_button")
    reset_button.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=charcoal,
        name="button_cap",
    )
    reset_joint = model.articulation(
        "chassis_to_reset_button",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=reset_button,
        origin=Origin(xyz=(-0.130, -0.045, height)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.06, lower=0.0, upper=0.003),
    )

    # Keep named joints in metadata for easy inspection.
    model.meta["primary_articulations"] = [side_hinge.name, front_hinge.name, power_joint.name, reset_joint.name]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    front_door = object_model.get_part("front_door")
    power_button = object_model.get_part("power_button")
    reset_button = object_model.get_part("reset_button")
    side_hinge = object_model.get_articulation("chassis_to_side_panel")
    front_hinge = object_model.get_articulation("chassis_to_front_door")
    power_joint = object_model.get_articulation("chassis_to_power_button")
    reset_joint = object_model.get_articulation("chassis_to_reset_button")

    def aabb_center_yz(aabb):
        lo, hi = aabb
        return ((lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    def aabb_center_x(aabb):
        lo, hi = aabb
        return (lo[0] + hi[0]) * 0.5

    ctx.expect_overlap(
        side_panel,
        chassis,
        axes="xz",
        min_overlap=0.25,
        elem_a="glass_pane",
        name="closed side panel spans the side opening",
    )
    ctx.expect_overlap(
        front_door,
        chassis,
        axes="yz",
        min_overlap=0.15,
        name="front intake door covers the front opening",
    )
    ctx.expect_contact(
        power_button,
        chassis,
        elem_a="button_cap",
        contact_tol=0.001,
        name="power button cap seats on top panel",
    )
    ctx.expect_contact(
        reset_button,
        chassis,
        elem_a="button_cap",
        contact_tol=0.001,
        name="reset button cap seats on top panel",
    )

    chassis_box = ctx.part_world_aabb(chassis)
    glass_box = ctx.part_element_world_aabb(side_panel, elem="glass_pane")
    front_door_box = ctx.part_element_world_aabb(front_door, elem="hinge_stile")
    side_closed_box = ctx.part_element_world_aabb(side_panel, elem="latch_frame")
    front_closed_box = ctx.part_element_world_aabb(front_door, elem="latch_stile")
    power_up = ctx.part_world_position(power_button)
    reset_up = ctx.part_world_position(reset_button)

    with ctx.pose({side_hinge: 1.20, front_hinge: 1.05, power_joint: 0.004, reset_joint: 0.003}):
        side_open_box = ctx.part_element_world_aabb(side_panel, elem="latch_frame")
        front_open_box = ctx.part_element_world_aabb(front_door, elem="latch_stile")
        power_down = ctx.part_world_position(power_button)
        reset_down = ctx.part_world_position(reset_button)

    side_closed_y = aabb_center_yz(side_closed_box)[0] if side_closed_box is not None else None
    side_open_y = aabb_center_yz(side_open_box)[0] if side_open_box is not None else None
    front_closed_x = aabb_center_x(front_closed_box) if front_closed_box is not None else None
    front_open_x = aabb_center_x(front_open_box) if front_open_box is not None else None

    side_gap = None
    front_gap = None
    if chassis_box is not None and glass_box is not None:
        side_gap = chassis_box[0][1] - glass_box[1][1]
    if chassis_box is not None and front_door_box is not None:
        front_gap = chassis_box[0][0] - front_door_box[1][0]

    ctx.check(
        "closed glass stays outside the chassis side envelope",
        side_gap is not None and side_gap >= 0.001,
        details=f"side_gap={side_gap}",
    )
    ctx.check(
        "front door closes in front of the chassis envelope",
        front_gap is not None and -0.001 <= front_gap <= 0.004,
        details=f"front_gap={front_gap}",
    )

    ctx.check(
        "side panel swings outward from the left side",
        side_closed_y is not None and side_open_y is not None and side_open_y < side_closed_y - 0.08,
        details=f"closed_y={side_closed_y}, open_y={side_open_y}",
    )
    ctx.check(
        "front door swings outward from the front",
        front_closed_x is not None and front_open_x is not None and front_open_x < front_closed_x - 0.08,
        details=f"closed_x={front_closed_x}, open_x={front_open_x}",
    )
    ctx.check(
        "top controls depress downward",
        power_up is not None
        and power_down is not None
        and reset_up is not None
        and reset_down is not None
        and power_down[2] < power_up[2] - 0.003
        and reset_down[2] < reset_up[2] - 0.002,
        details=f"power {power_up}->{power_down}, reset {reset_up}->{reset_down}",
    )

    return ctx.report()


object_model = build_object_model()
