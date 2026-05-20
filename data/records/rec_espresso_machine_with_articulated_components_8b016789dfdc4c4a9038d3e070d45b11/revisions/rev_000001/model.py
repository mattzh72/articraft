from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_automatic_espresso_machine")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.14, 0.15, 0.16, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    tank_clear = model.material("tank_clear", rgba=(0.72, 0.80, 0.90, 0.42))
    water_blue = model.material("water_blue", rgba=(0.48, 0.66, 0.88, 0.30))

    body = model.part("body")
    body.visual(
        Box((0.34, 0.27, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=stainless,
        name="base_plinth",
    )
    body.visual(
        Box((0.30, 0.19, 0.28)),
        origin=Origin(xyz=(-0.005, -0.040, 0.190)),
        material=stainless,
        name="main_shell",
    )
    body.visual(
        Box((0.18, 0.19, 0.03)),
        origin=Origin(xyz=(0.060, -0.040, 0.335)),
        material=dark_panel,
        name="control_cap",
    )
    body.visual(
        Box((0.29, 0.012, 0.25)),
        origin=Origin(xyz=(-0.005, 0.049, 0.175)),
        material=dark_panel,
        name="tank_divider",
    )
    body.visual(
        Box((0.29, 0.086, 0.04)),
        origin=Origin(xyz=(-0.005, 0.092, 0.340)),
        material=stainless,
        name="tank_roof",
    )
    body.visual(
        Box((0.028, 0.086, 0.28)),
        origin=Origin(xyz=(-0.156, 0.092, 0.190)),
        material=stainless,
        name="tank_rear_jamb",
    )
    body.visual(
        Box((0.016, 0.086, 0.24)),
        origin=Origin(xyz=(0.157, 0.092, 0.170)),
        material=stainless,
        name="tank_front_jamb",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.27, 0.36)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    tank = model.part("tank")
    tank.visual(
        Box((0.25, 0.060, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=tank_clear,
        name="reservoir",
    )
    tank.visual(
        Box((0.23, 0.050, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=water_blue,
        name="water_volume",
    )
    tank.visual(
        Box((0.085, 0.050, 0.020)),
        origin=Origin(xyz=(-0.050, 0.0, 0.230)),
        material=black_plastic,
        name="tank_lid",
    )
    tank.inertial = Inertial.from_geometry(
        Box((0.25, 0.06, 0.24)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    model.articulation(
        "body_to_tank",
        ArticulationType.FIXED,
        parent=body,
        child=tank,
        origin=Origin(xyz=(-0.005, 0.095, 0.050)),
    )

    door = model.part("door")
    door.visual(
        Box((0.286, 0.008, 0.255)),
        origin=Origin(xyz=(0.143, 0.004, 0.1275)),
        material=stainless,
        name="door_panel",
    )
    door.visual(
        Box((0.248, 0.010, 0.215)),
        origin=Origin(xyz=(0.150, 0.005, 0.1275)),
        material=dark_panel,
        name="door_frame",
    )
    door.visual(
        Box((0.020, 0.020, 0.070)),
        origin=Origin(xyz=(0.252, 0.010, 0.145)),
        material=black_plastic,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.286, 0.02, 0.255)),
        mass=0.8,
        origin=Origin(xyz=(0.143, 0.01, 0.1275)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.146, 0.135, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    group_head = model.part("group_head")
    group_head.visual(
        Box((0.030, 0.120, 0.070)),
        origin=Origin(xyz=(0.015, 0.0, 0.035)),
        material=dark_panel,
        name="head_mount",
    )
    group_head.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(0.050, 0.0, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="head_body",
    )
    group_head.visual(
        Cylinder(radius=0.043, length=0.008),
        origin=Origin(xyz=(0.076, 0.0, 0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="head_trim",
    )
    group_head.inertial = Inertial.from_geometry(
        Box((0.085, 0.12, 0.08)),
        mass=0.9,
        origin=Origin(xyz=(0.042, 0.0, 0.040)),
    )

    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=group_head,
        origin=Origin(xyz=(0.145, -0.012, 0.215)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=stainless,
        name="pf_flange",
    )
    portafilter.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=stainless,
        name="pf_basket",
    )
    portafilter.visual(
        Box((0.014, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.028, -0.003)),
        material=stainless,
        name="lug_0",
    )
    portafilter.visual(
        Box((0.014, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.028, -0.003)),
        material=stainless,
        name="lug_1",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.160),
        origin=Origin(xyz=(0.090, 0.0, -0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="handle_core",
    )
    portafilter.visual(
        Box((0.070, 0.024, 0.028)),
        origin=Origin(xyz=(0.135, 0.0, -0.038)),
        material=black_plastic,
        name="handle_grip",
    )
    portafilter.visual(
        Box((0.020, 0.018, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, -0.044)),
        material=stainless,
        name="spout_block",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.18, 0.08, 0.06)),
        mass=0.55,
        origin=Origin(xyz=(0.080, 0.0, -0.030)),
    )

    model.articulation(
        "group_head_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=group_head,
        child=portafilter,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=math.radians(-70.0),
            upper=math.radians(20.0),
        ),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=stainless,
        name="pivot_collar",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="elbow_arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.005, length=0.160),
        origin=Origin(xyz=(0.040, 0.0, -0.100)),
        material=stainless,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.003, length=0.028),
        origin=Origin(xyz=(0.040, 0.0, -0.194)),
        material=stainless,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.06, 0.03, 0.22)),
        mass=0.25,
        origin=Origin(xyz=(0.030, 0.0, -0.095)),
    )

    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.155, -0.084, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=math.radians(-40.0),
            upper=math.radians(110.0),
        ),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_panel,
        name="dial_shaft",
    )
    selector_dial.visual(
        Cylinder(radius=0.027, length=0.020),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="dial_knob",
    )
    selector_dial.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="dial_rim",
    )
    selector_dial.visual(
        Box((0.006, 0.010, 0.018)),
        origin=Origin(xyz=(0.042, 0.0, 0.021)),
        material=stainless,
        name="dial_pointer",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Box((0.05, 0.06, 0.06)),
        mass=0.12,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(0.145, -0.012, 0.112)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=math.radians(-110.0),
            upper=math.radians(110.0),
        ),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.110, 0.130, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, 0.009)),
        material=stainless,
        name="tray_body",
    )
    drip_tray.visual(
        Box((0.016, 0.130, 0.035)),
        origin=Origin(xyz=(0.102, 0.0, 0.0175)),
        material=stainless,
        name="tray_lip",
    )
    drip_tray.visual(
        Box((0.090, 0.100, 0.004)),
        origin=Origin(xyz=(0.055, 0.0, 0.020)),
        material=dark_panel,
        name="tray_grate",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.11, 0.13, 0.035)),
        mass=0.45,
        origin=Origin(xyz=(0.055, 0.0, 0.0175)),
    )

    model.articulation(
        "body_to_drip_tray",
        ArticulationType.FIXED,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.145, -0.012, 0.050)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tank = object_model.get_part("tank")
    door = object_model.get_part("door")
    group_head = object_model.get_part("group_head")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    selector_dial = object_model.get_part("selector_dial")
    door_hinge = object_model.get_articulation("body_to_door")
    portafilter_joint = object_model.get_articulation("group_head_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    dial_joint = object_model.get_articulation("body_to_selector_dial")

    ctx.expect_gap(
        door,
        tank,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        name="door closes just outside the tank",
    )
    ctx.expect_overlap(
        door,
        tank,
        axes="xz",
        min_overlap=0.18,
        name="door covers the water tank opening",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: math.radians(90.0)}):
        opened_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings outward from the side bay",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.12,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.expect_overlap(
        tank,
        body,
        axes="xz",
        min_overlap=0.20,
        name="tank remains nested within the right side compartment footprint",
    )
    ctx.expect_origin_gap(
        group_head,
        selector_dial,
        axis="z",
        min_gap=0.08,
        max_gap=0.14,
        name="selector dial sits below the group head",
    )
    ctx.expect_gap(
        group_head,
        portafilter,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="portafilter seats directly beneath the group head",
    )

    pf_handle_rest = ctx.part_element_world_aabb(portafilter, elem="handle_grip")
    with ctx.pose({portafilter_joint: math.radians(18.0)}):
        pf_handle_turned = ctx.part_element_world_aabb(portafilter, elem="handle_grip")
    ctx.check(
        "portafilter rotates about the brew axis",
        pf_handle_rest is not None
        and pf_handle_turned is not None
        and 0.5 * (pf_handle_turned[0][1] + pf_handle_turned[1][1])
        > 0.5 * (pf_handle_rest[0][1] + pf_handle_rest[1][1]) + 0.03,
        details=f"rest={pf_handle_rest}, turned={pf_handle_turned}",
    )

    wand_rest = ctx.part_element_world_aabb(steam_wand, elem="wand_tube")
    with ctx.pose({steam_joint: math.radians(95.0)}):
        wand_swung = ctx.part_element_world_aabb(steam_wand, elem="wand_tube")
    ctx.check(
        "steam wand swings on its vertical pivot",
        wand_rest is not None
        and wand_swung is not None
        and 0.5 * (wand_swung[0][1] + wand_swung[1][1])
        > 0.5 * (wand_rest[0][1] + wand_rest[1][1]) + 0.03,
        details=f"rest={wand_rest}, swung={wand_swung}",
    )

    dial_pointer_rest = ctx.part_element_world_aabb(selector_dial, elem="dial_pointer")
    with ctx.pose({dial_joint: math.radians(70.0)}):
        dial_pointer_turned = ctx.part_element_world_aabb(selector_dial, elem="dial_pointer")
    ctx.check(
        "selector dial turns about its front-facing shaft axis",
        dial_pointer_rest is not None
        and dial_pointer_turned is not None
        and 0.5 * (dial_pointer_turned[0][1] + dial_pointer_turned[1][1])
        < 0.5 * (dial_pointer_rest[0][1] + dial_pointer_rest[1][1]) - 0.012,
        details=f"rest={dial_pointer_rest}, turned={dial_pointer_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
