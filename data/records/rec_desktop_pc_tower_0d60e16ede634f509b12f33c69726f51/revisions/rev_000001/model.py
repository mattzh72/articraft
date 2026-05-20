from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


CASE_W = 0.24
CASE_D = 0.56
CASE_H = 0.64
FRONT_Y = -0.295


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_optical_tray(model, chassis, index: int, bay_z: float, materials) -> None:
    tray = model.part(f"tray_{index}")
    tray.visual(
        Box((0.168, 0.010, 0.043)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=materials["drive_black"],
        name="front_plate",
    )
    tray.visual(
        Box((0.166, 0.270, 0.008)),
        origin=Origin(xyz=(0.0, 0.135, -0.012)),
        material=materials["tray_plastic"],
        name="tray_slab",
    )
    for side, x in (("a", -0.087), ("b", 0.087)):
        tray.visual(
            Box((0.008, 0.250, 0.020)),
            origin=Origin(xyz=(x, 0.125, -0.006)),
            material=materials["tray_plastic"],
            name=f"side_rail_{side}",
        )
    tray.visual(
        Cylinder(radius=0.055, length=0.002),
        origin=Origin(xyz=(0.0, 0.105, -0.007)),
        material=materials["recess_dark"],
        name="disc_recess",
    )
    tray.visual(
        Box((0.050, 0.004, 0.004)),
        origin=Origin(xyz=(-0.038, -0.012, -0.006)),
        material=materials["led_blue"],
        name="status_light",
    )

    model.articulation(
        f"chassis_to_tray_{index}",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=tray,
        origin=Origin(xyz=(0.0, FRONT_Y, bay_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.22, lower=0.0, upper=0.160),
    )

    eject = model.part(f"eject_button_{index}")
    eject.visual(
        Box((0.018, 0.004, 0.006)),
        origin=Origin(),
        material=materials["button_gray"],
        name="button_cap",
    )
    model.articulation(
        f"tray_{index}_to_eject",
        ArticulationType.PRISMATIC,
        parent=tray,
        child=eject,
        origin=Origin(xyz=(0.060, -0.012, -0.007)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.002),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_tower_gaming_pc")

    materials = {
        "matte_black": model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0)),
        "soft_black": model.material("soft_black", rgba=(0.045, 0.047, 0.052, 1.0)),
        "edge_black": model.material("edge_black", rgba=(0.0, 0.0, 0.0, 1.0)),
        "glass": model.material("smoked_tempered_glass", rgba=(0.08, 0.12, 0.16, 0.34)),
        "tray_plastic": model.material("satin_tray_plastic", rgba=(0.18, 0.18, 0.18, 1.0)),
        "drive_black": model.material("drive_face_black", rgba=(0.025, 0.025, 0.028, 1.0)),
        "recess_dark": model.material("disc_recess_dark", rgba=(0.04, 0.04, 0.045, 1.0)),
        "button_gray": model.material("button_gray", rgba=(0.35, 0.36, 0.38, 1.0)),
        "pcb": model.material("dark_pcb", rgba=(0.02, 0.18, 0.10, 1.0)),
        "metal": model.material("brushed_dark_metal", rgba=(0.28, 0.29, 0.30, 1.0)),
        "led_blue": model.material("electric_blue_led", rgba=(0.0, 0.40, 1.0, 1.0)),
        "led_purple": model.material("rgb_purple", rgba=(0.55, 0.05, 0.95, 1.0)),
    }

    chassis = model.part("chassis")

    # Main tower shell: one connected fixed frame with an open glass side and
    # a detailed front bezel cut into two optical-drive bays.
    _box(chassis, "left_side_panel", (0.025, CASE_D, CASE_H), (-0.1125, 0.0, CASE_H / 2), materials["matte_black"])
    _box(chassis, "top_panel", (CASE_W, CASE_D, 0.025), (0.0, 0.0, CASE_H - 0.0125), materials["matte_black"])
    _box(chassis, "bottom_panel", (CASE_W, CASE_D, 0.025), (0.0, 0.0, 0.0125), materials["matte_black"])
    _box(chassis, "rear_panel", (CASE_W, 0.025, CASE_H), (0.0, 0.2675, CASE_H / 2), materials["soft_black"])

    # Front bezel pieces around the real bay openings.
    front_t = 0.026
    _box(chassis, "front_lower", (CASE_W, front_t, 0.360), (0.0, -0.282, 0.205), materials["soft_black"])
    _box(chassis, "front_top", (CASE_W, front_t, 0.055), (0.0, -0.282, 0.6125), materials["soft_black"])
    _box(chassis, "bay_left_stile", (0.031, front_t, 0.200), (-0.1045, -0.282, 0.485), materials["soft_black"])
    _box(chassis, "bay_right_stile", (0.031, front_t, 0.200), (0.1045, -0.282, 0.485), materials["soft_black"])
    _box(chassis, "bay_lower_rail", (CASE_W, front_t, 0.045), (0.0, -0.282, 0.4075), materials["soft_black"])
    _box(chassis, "bay_middle_rail", (CASE_W, front_t, 0.007), (0.0, -0.282, 0.4835), materials["soft_black"])
    _box(chassis, "bay_upper_rail", (CASE_W, front_t, 0.047), (0.0, -0.282, 0.5605), materials["soft_black"])

    # Drive-bay sleeves: thin rails behind the front opening so the tray visibly
    # slides inside supported rails instead of floating in an empty hole.
    for i, bay_z in enumerate((0.455, 0.512)):
        for side, x in (("left", -0.094), ("right", 0.094)):
            _box(chassis, f"bay_{i}_{side}_sleeve", (0.006, 0.180, 0.052), (x, -0.185, bay_z), materials["matte_black"])
        _box(chassis, f"bay_{i}_top_shelf", (0.184, 0.180, 0.004), (0.0, -0.185, bay_z + 0.027), materials["matte_black"])
        _box(chassis, f"bay_{i}_bottom_shelf", (0.184, 0.180, 0.004), (0.0, -0.185, bay_z - 0.027), materials["matte_black"])

    # Right-side perimeter lip for the hinged glass side panel.
    chassis.visual(
        Box((0.020, 0.026, 0.580)),
        origin=Origin(xyz=(0.110, -0.252, 0.320)),
        material=materials["edge_black"],
        name="side_front_lip",
    )
    _box(chassis, "side_rear_lip", (0.020, 0.030, 0.580), (0.110, 0.252, 0.320), materials["edge_black"])
    _box(chassis, "side_top_lip", (0.020, 0.504, 0.025), (0.110, 0.0, 0.6125), materials["edge_black"])
    _box(chassis, "side_bottom_lip", (0.020, 0.504, 0.025), (0.110, 0.0, 0.0275), materials["edge_black"])
    for j, z in enumerate((0.115, 0.325, 0.535)):
        _box(chassis, f"hinge_leaf_{j}", (0.011, 0.006, 0.105), (0.1255, 0.259, z), materials["metal"])

    # Decorative but connected gaming-PC details visible through the glass.
    _box(chassis, "motherboard", (0.007, 0.330, 0.355), (-0.097, 0.015, 0.365), materials["pcb"])
    _box(chassis, "cpu_block", (0.035, 0.055, 0.055), (-0.076, 0.040, 0.405), materials["metal"])
    _box(chassis, "gpu_body", (0.095, 0.245, 0.040), (-0.050, -0.030, 0.270), materials["metal"])
    _box(chassis, "gpu_backplate", (0.100, 0.248, 0.008), (-0.048, -0.030, 0.294), materials["edge_black"])
    for y in (-0.095, 0.035):
        chassis.visual(
            Cylinder(radius=0.027, length=0.008),
            origin=Origin(xyz=(-0.006, y, 0.270), rpy=(0.0, pi / 2, 0.0)),
            material=materials["edge_black"],
            name=f"gpu_fan_{'front' if y < 0 else 'rear'}",
        )
        chassis.visual(
            Cylinder(radius=0.019, length=0.009),
            origin=Origin(xyz=(-0.001, y, 0.270), rpy=(0.0, pi / 2, 0.0)),
            material=materials["led_purple"],
            name=f"gpu_rgb_{'front' if y < 0 else 'rear'}",
        )

    for idx, z in enumerate((0.135, 0.245, 0.355)):
        chassis.visual(
            Cylinder(radius=0.050, length=0.012),
            origin=Origin(xyz=(0.0, -0.267, z), rpy=(pi / 2, 0.0, 0.0)),
            material=materials["led_blue"],
            name=f"front_fan_{idx}",
        )

    front_grille = VentGrilleGeometry(
        (0.190, 0.310),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.020,
        duct_wall=0.003,
        slat_pitch=0.020,
        slat_width=0.008,
        slat_angle_deg=28.0,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=1, divider_width=0.004),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
        sleeve=VentGrilleSleeve(style="short", depth=0.012),
    )
    chassis.visual(
        mesh_from_geometry(front_grille, "front_intake_grille"),
        origin=Origin(xyz=(0.0, -0.304, 0.235), rpy=(pi / 2, 0.0, 0.0)),
        material=materials["edge_black"],
        name="front_intake_grille",
    )
    top_slots = SlotPatternPanelGeometry(
        (0.155, 0.350),
        0.004,
        slot_size=(0.085, 0.012),
        pitch=(0.105, 0.022),
        frame=0.012,
        corner_radius=0.004,
        stagger=True,
    )
    chassis.visual(
        mesh_from_geometry(top_slots, "top_vent_slots"),
        origin=Origin(xyz=(0.0, -0.010, 0.642)),
        material=materials["edge_black"],
        name="top_vent_slots",
    )

    # Rubber feet are overlapped slightly into the bottom panel so they read as
    # bolted on, not as separate floating pads.
    for i, (x, y) in enumerate(((-0.075, -0.210), (0.075, -0.210), (-0.075, 0.210), (0.075, 0.210))):
        _box(chassis, f"foot_{i}", (0.045, 0.060, 0.014), (x, y, -0.001), materials["edge_black"])

    # Hinged tempered-glass side panel, with rear hinge barrels and a front pull.
    panel = model.part("glass_panel")
    panel_d = 0.520
    panel_h = 0.570
    panel.visual(
        Box((0.006, panel_d - 0.044, panel_h - 0.044)),
        origin=Origin(xyz=(0.010, -panel_d / 2, 0.0)),
        material=materials["glass"],
        name="glass_pane",
    )
    panel.visual(
        Box((0.014, 0.022, panel_h)),
        origin=Origin(xyz=(0.006, -0.011, 0.0)),
        material=materials["edge_black"],
        name="rear_frame",
    )
    panel.visual(
        Box((0.014, 0.022, panel_h)),
        origin=Origin(xyz=(0.006, -panel_d + 0.011, 0.0)),
        material=materials["edge_black"],
        name="front_frame",
    )
    panel.visual(
        Box((0.014, panel_d, 0.022)),
        origin=Origin(xyz=(0.006, -panel_d / 2, panel_h / 2 - 0.011)),
        material=materials["edge_black"],
        name="top_frame",
    )
    panel.visual(
        Box((0.014, panel_d, 0.022)),
        origin=Origin(xyz=(0.006, -panel_d / 2, -panel_h / 2 + 0.011)),
        material=materials["edge_black"],
        name="bottom_frame",
    )
    for j, z in enumerate((-0.210, 0.0, 0.210)):
        panel.visual(
            Cylinder(radius=0.008, length=0.105),
            origin=Origin(xyz=(0.012, 0.003, z)),
            material=materials["metal"],
            name=f"hinge_barrel_{j}",
        )
    panel.visual(
        Box((0.025, 0.010, 0.170)),
        origin=Origin(xyz=(0.023, -panel_d + 0.012, 0.0)),
        material=materials["metal"],
        name="pull_handle",
    )
    model.articulation(
        "chassis_to_glass_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=panel,
        origin=Origin(xyz=(0.127, 0.255, 0.325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    _add_optical_tray(model, chassis, 0, 0.455, materials)
    _add_optical_tray(model, chassis, 1, 0.512, materials)

    # Front power control is a separate push button rather than a painted disk.
    power = model.part("power_button")
    power.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=materials["led_blue"],
        name="button_cap",
    )
    model.articulation(
        "chassis_to_power_button",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=power,
        origin=Origin(xyz=(0.082, FRONT_Y, 0.613)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.03, lower=0.0, upper=0.003),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    panel = object_model.get_part("glass_panel")
    panel_hinge = object_model.get_articulation("chassis_to_glass_panel")

    closed_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_hinge: 1.20}):
        open_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "glass panel swings outward on rear hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][0] > closed_panel_aabb[1][0] + 0.16,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )
    ctx.expect_gap(
        panel,
        chassis,
        axis="x",
        min_gap=0.001,
        max_gap=0.035,
        elem_a="glass_pane",
        elem_b="side_front_lip",
        name="closed glass sits just outside side frame",
    )

    for index in (0, 1):
        tray = object_model.get_part(f"tray_{index}")
        joint = object_model.get_articulation(f"chassis_to_tray_{index}")
        rest_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            chassis,
            axes="xz",
            margin=0.002,
            elem_a="front_plate",
            name=f"tray {index} front face fits tower opening",
        )
        with ctx.pose({joint: 0.160}):
            extended_pos = ctx.part_world_position(tray)
            ctx.expect_overlap(
                tray,
                chassis,
                axes="y",
                min_overlap=0.018,
                elem_a="tray_slab",
                name=f"tray {index} remains retained in bay when extended",
            )
        ctx.check(
            f"tray {index} slides forward",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.12,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
