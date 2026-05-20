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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


BODY_W = 0.215
BODY_D = 0.355
BODY_H = 0.322
SHELL_T = 0.004


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _control_panel_mesh():
    outer = rounded_rect_profile(0.179, 0.060, 0.006)
    holes = [
        _shift_profile(rounded_rect_profile(0.026, 0.018, 0.0025), x, 0.0)
        for x in (-0.060, -0.020, 0.020, 0.060)
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, 0.004, center=True),
        "control_panel",
    )


def _cup_tray_mesh():
    return mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.166, 0.116),
            0.0025,
            slot_size=(0.020, 0.004),
            pitch=(0.027, 0.013),
            frame=0.010,
            corner_radius=0.005,
            center=True,
        ),
        "cup_tray",
    )


def _drip_grate_mesh():
    return mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.138, 0.130),
            0.003,
            slot_size=(0.020, 0.004),
            pitch=(0.027, 0.012),
            frame=0.009,
            corner_radius=0.004,
            center=True,
        ),
        "drip_grate",
    )


def _group_head_collar_mesh():
    outer = [
        (0.016, -0.012),
        (0.023, -0.012),
        (0.026, -0.006),
        (0.026, 0.006),
        (0.021, 0.012),
    ]
    inner = [
        (0.000, -0.009),
        (0.017, -0.009),
        (0.019, -0.004),
        (0.019, 0.006),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=48),
        "group_head_collar",
    )


def _portafilter_basket_mesh():
    outer = [
        (0.020, -0.054),
        (0.020, -0.008),
        (0.024, -0.003),
        (0.029, 0.000),
    ]
    inner = [
        (0.000, -0.050),
        (0.016, -0.050),
        (0.016, -0.004),
        (0.023, -0.001),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=48),
        "portafilter_basket",
    )


def _wand_mesh():
    wand_geom = tube_from_spline_points(
        [
            (0.008, 0.000, 0.004),
            (0.010, 0.000, -0.040),
            (0.016, -0.006, -0.115),
            (0.020, -0.012, -0.188),
            (0.023, -0.018, -0.246),
        ],
        radius=0.0034,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(wand_geom, "wand_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thermoblock_espresso_machine")

    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark = model.material("dark", rgba=(0.16, 0.17, 0.18, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.90, 1.0))
    water_tint = model.material("water_tint", rgba=(0.67, 0.83, 0.92, 0.26))
    water_fill = model.material("water_fill", rgba=(0.46, 0.71, 0.90, 0.34))
    control_dark = model.material("control_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    body = model.part("body")
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H - 0.002)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SHELL_T / 2.0, BODY_D / 2.0, (BODY_H - 0.002) / 2.0)),
        material=steel,
        name="left_shell",
    )
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H - 0.002)),
        origin=Origin(xyz=(BODY_W / 2.0 - SHELL_T / 2.0, BODY_D / 2.0, (BODY_H - 0.002) / 2.0)),
        material=steel,
        name="right_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, BODY_D, 0.006)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0, 0.003)),
        material=dark,
        name="base",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, BODY_H - 0.002)),
        origin=Origin(xyz=(0.0, BODY_D - SHELL_T / 2.0, (BODY_H - 0.002) / 2.0)),
        material=steel,
        name="rear_shell",
    )
    body.visual(
        Box((BODY_W, 0.165, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.112, BODY_H - SHELL_T / 2.0)),
        material=steel,
        name="roof_front",
    )
    body.visual(
        Box((0.040, 0.103, SHELL_T)),
        origin=Origin(xyz=(-0.0875, 0.2465, BODY_H - SHELL_T / 2.0)),
        material=steel,
        name="roof_rail_0",
    )
    body.visual(
        Box((0.040, 0.103, SHELL_T)),
        origin=Origin(xyz=(0.0875, 0.2465, BODY_H - SHELL_T / 2.0)),
        material=steel,
        name="roof_rail_1",
    )
    body.visual(
        Box((BODY_W, 0.057, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.3265, BODY_H - SHELL_T / 2.0)),
        material=steel,
        name="roof_rear",
    )
    body.visual(
        Box((0.018, SHELL_T, BODY_H)),
        origin=Origin(xyz=(-0.0985, SHELL_T / 2.0, BODY_H / 2.0)),
        material=dark,
        name="front_pillar_0",
    )
    body.visual(
        Box((0.018, SHELL_T, BODY_H)),
        origin=Origin(xyz=(0.0985, SHELL_T / 2.0, BODY_H / 2.0)),
        material=dark,
        name="front_pillar_1",
    )
    body.visual(
        Box((BODY_W, SHELL_T, 0.018)),
        origin=Origin(xyz=(0.0, SHELL_T / 2.0, 0.313)),
        material=dark,
        name="front_cap",
    )
    body.visual(
        Box((0.179, SHELL_T, 0.016)),
        origin=Origin(xyz=(0.0, SHELL_T / 2.0, 0.236)),
        material=dark,
        name="control_sill",
    )
    body.visual(
        Box((0.179, SHELL_T, 0.044)),
        origin=Origin(xyz=(0.0, SHELL_T / 2.0, 0.121)),
        material=dark,
        name="tray_bridge",
    )
    body.visual(
        Box((BODY_W, SHELL_T, 0.016)),
        origin=Origin(xyz=(0.0, SHELL_T / 2.0, 0.008)),
        material=dark,
        name="front_lip",
    )
    body.visual(
        Box((0.010, 0.152, 0.016)),
        origin=Origin(xyz=(-0.072, 0.086, 0.030)),
        material=dark,
        name="left_rail",
    )
    body.visual(
        Box((0.010, 0.152, 0.016)),
        origin=Origin(xyz=(0.072, 0.086, 0.030)),
        material=dark,
        name="right_rail",
    )
    body.visual(
        Box((0.032, 0.152, 0.008)),
        origin=Origin(xyz=(-0.090, 0.086, 0.018)),
        material=dark,
        name="left_guide",
    )
    body.visual(
        Box((0.032, 0.152, 0.008)),
        origin=Origin(xyz=(0.090, 0.086, 0.018)),
        material=dark,
        name="right_guide",
    )
    body.visual(
        Box((0.150, SHELL_T, 0.016)),
        origin=Origin(xyz=(0.0, 0.160, 0.030)),
        material=dark,
        name="tray_stop",
    )
    body.visual(
        Box((0.170, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, 0.016, 0.251)),
        material=dark,
        name="button_support",
    )
    body.visual(
        Box((0.100, 0.040, 0.094)),
        origin=Origin(xyz=(0.0, 0.022, 0.189)),
        material=dark,
        name="front_core",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=7.8,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, BODY_H / 2.0)),
    )

    cup_tray = model.part("cup_tray")
    cup_tray.visual(
        _cup_tray_mesh(),
        material=dark,
        name="panel",
    )
    cup_tray.inertial = Inertial.from_geometry(Box((0.166, 0.116, 0.003)), mass=0.08)
    model.articulation(
        "body_to_cup_tray",
        ArticulationType.FIXED,
        parent=body,
        child=cup_tray,
        origin=Origin(xyz=(0.0, 0.112, BODY_H + 0.00125)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        _control_panel_mesh(),
        material=control_dark,
        name="panel",
    )
    control_panel.inertial = Inertial.from_geometry(Box((0.179, 0.004, 0.060)), mass=0.08)
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.0, 0.002, 0.274), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    group_head = model.part("group_head")
    group_head.visual(
        Box((0.070, 0.040, 0.036)),
        origin=Origin(xyz=(0.0, 0.020, 0.028)),
        material=dark,
        name="mount",
    )
    group_head.visual(
        _group_head_collar_mesh(),
        material=chrome,
        name="collar",
    )
    group_head.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0105)),
        material=black,
        name="screen",
    )
    group_head.inertial = Inertial.from_geometry(
        Box((0.080, 0.052, 0.060)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.012, 0.018)),
    )
    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=group_head,
        origin=Origin(xyz=(0.0, -0.040, 0.200)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        _portafilter_basket_mesh(),
        material=chrome,
        name="basket",
    )
    portafilter.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(-0.024, 0.0, -0.001)),
        material=chrome,
        name="lug_0",
    )
    portafilter.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.024, 0.0, -0.001)),
        material=chrome,
        name="lug_1",
    )
    portafilter.visual(
        Box((0.018, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.014, -0.018)),
        material=black,
        name="neck",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(0.0, -0.072, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="handle",
    )
    portafilter.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, -0.128, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.060, 0.145, 0.060)),
        mass=0.42,
        origin=Origin(xyz=(0.0, -0.055, -0.020)),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.0, -0.040, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.18,
        ),
    )

    tank = model.part("tank")
    tank.visual(
        Box((0.136, 0.100, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=water_tint,
        name="floor",
    )
    tank.visual(
        Box((0.004, 0.100, 0.304)),
        origin=Origin(xyz=(-0.066, 0.0, 0.152)),
        material=water_tint,
        name="wall_0",
    )
    tank.visual(
        Box((0.004, 0.100, 0.304)),
        origin=Origin(xyz=(0.066, 0.0, 0.152)),
        material=water_tint,
        name="wall_1",
    )
    tank.visual(
        Box((0.136, 0.004, 0.304)),
        origin=Origin(xyz=(0.0, -0.048, 0.152)),
        material=water_tint,
        name="wall_2",
    )
    tank.visual(
        Box((0.136, 0.004, 0.304)),
        origin=Origin(xyz=(0.0, 0.048, 0.152)),
        material=water_tint,
        name="wall_3",
    )
    tank.visual(
        Box((0.128, 0.092, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=water_fill,
        name="water",
    )
    tank.inertial = Inertial.from_geometry(
        Box((0.136, 0.100, 0.304)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
    )
    model.articulation(
        "body_to_tank",
        ArticulationType.FIXED,
        parent=body,
        child=tank,
        origin=Origin(xyz=(0.0, 0.247, 0.006)),
    )

    tank_lid = model.part("tank_lid")
    tank_lid.visual(
        Box((0.148, 0.112, 0.006)),
        origin=Origin(xyz=(0.0, -0.056, 0.003)),
        material=dark,
        name="panel",
    )
    tank_lid.visual(
        Box((0.120, 0.084, 0.008)),
        origin=Origin(xyz=(0.0, -0.054, -0.004)),
        material=dark,
        name="rib",
    )
    tank_lid.visual(
        Box((0.036, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.108, 0.005)),
        material=black,
        name="pull_tab",
    )
    tank_lid.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(-0.045, 0.004, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hinge_0",
    )
    tank_lid.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.045, 0.004, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hinge_1",
    )
    tank_lid.inertial = Inertial.from_geometry(
        Box((0.148, 0.112, 0.016)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.056, 0.002)),
    )
    model.articulation(
        "body_to_tank_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tank_lid,
        origin=Origin(xyz=(0.0, 0.307, BODY_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.7,
        ),
    )

    wand_support = model.part("wand_support")
    wand_support.visual(
        Box((0.007, 0.028, 0.024)),
        origin=Origin(xyz=(-0.0035, 0.0, 0.012)),
        material=dark,
        name="bracket",
    )
    wand_support.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chrome,
        name="pivot",
    )
    wand_support.inertial = Inertial.from_geometry(
        Box((0.014, 0.028, 0.024)),
        mass=0.08,
        origin=Origin(xyz=(-0.003, 0.0, 0.012)),
    )
    model.articulation(
        "body_to_wand_support",
        ArticulationType.FIXED,
        parent=body,
        child=wand_support,
        origin=Origin(xyz=(0.114, 0.104, 0.236)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=chrome,
        name="hub",
    )
    wand.visual(
        _wand_mesh(),
        material=chrome,
        name="tube",
    )
    wand.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(xyz=(0.024, -0.019, -0.248), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="tip",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.280)),
        mass=0.14,
        origin=Origin(xyz=(0.016, -0.010, -0.110)),
    )
    model.articulation(
        "wand_support_to_wand",
        ArticulationType.REVOLUTE,
        parent=wand_support,
        child=wand,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.25,
            upper=0.5,
        ),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.156, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, -0.075, 0.037)),
        material=dark,
        name="front",
    )
    drip_tray.visual(
        Box((0.148, 0.156, 0.003)),
        origin=Origin(xyz=(0.0, 0.012, 0.005)),
        material=dark,
        name="base",
    )
    drip_tray.visual(
        Box((0.004, 0.156, 0.030)),
        origin=Origin(xyz=(-0.072, 0.012, 0.018)),
        material=dark,
        name="left_wall",
    )
    drip_tray.visual(
        Box((0.004, 0.156, 0.030)),
        origin=Origin(xyz=(0.072, 0.012, 0.018)),
        material=dark,
        name="right_wall",
    )
    drip_tray.visual(
        Box((0.148, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, 0.088, 0.018)),
        material=dark,
        name="rear_wall",
    )
    drip_tray.visual(
        Box((0.006, 0.126, 0.004)),
        origin=Origin(xyz=(-0.068, 0.006, 0.028)),
        material=dark,
        name="left_ledge",
    )
    drip_tray.visual(
        Box((0.006, 0.126, 0.004)),
        origin=Origin(xyz=(0.068, 0.006, 0.028)),
        material=dark,
        name="right_ledge",
    )
    drip_tray.visual(
        Box((0.126, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.070, 0.028)),
        material=dark,
        name="rear_ledge",
    )
    drip_tray.visual(
        _drip_grate_mesh(),
        origin=Origin(xyz=(0.0, 0.006, 0.0315)),
        material=steel,
        name="grate",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.156, 0.168, 0.074)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.010, 0.028)),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.0, 0.070, 0.013)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.12,
            lower=0.0,
            upper=0.085,
        ),
    )

    button_x = (-0.060, -0.020, 0.020, 0.060)
    for index, x_pos in enumerate(button_x):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.022, 0.007, 0.014)),
            origin=Origin(xyz=(0.0, 0.0035, 0.007)),
            material=black,
            name="cap",
        )
        button.visual(
            Box((0.012, 0.010, 0.008)),
            origin=Origin(xyz=(0.0, 0.012, 0.007)),
            material=black,
            name="stem",
        )
        button.visual(
            Box((0.032, 0.002, 0.022)),
            origin=Origin(xyz=(0.0, 0.005, 0.007)),
            material=black,
            name="retainer",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.022, 0.017, 0.014)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0085, 0.007)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, 0.274)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    group_head = object_model.get_part("group_head")
    drip_tray = object_model.get_part("drip_tray")
    tank_lid = object_model.get_part("tank_lid")
    portafilter = object_model.get_part("portafilter")
    wand = object_model.get_part("wand")
    button_0 = object_model.get_part("button_0")

    tray_joint = object_model.get_articulation("body_to_drip_tray")
    lid_joint = object_model.get_articulation("body_to_tank_lid")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("wand_support_to_wand")
    button_joint = object_model.get_articulation("body_to_button_0")

    ctx.expect_origin_gap(
        button_0,
        drip_tray,
        axis="z",
        min_gap=0.18,
        name="buttons sit above the drip tray",
    )
    ctx.expect_gap(
        group_head,
        drip_tray,
        axis="z",
        min_gap=0.07,
        name="group head clears the drip tray opening",
    )
    ctx.expect_overlap(
        drip_tray,
        body,
        axes="y",
        elem_a="left_wall",
        elem_b="left_rail",
        min_overlap=0.06,
        name="drip tray remains engaged in the lower guide at rest",
    )

    rest_button_pos = ctx.part_world_position(button_0)
    with ctx.pose({button_joint: button_joint.motion_limits.upper}):
        pressed_button_pos = ctx.part_world_position(button_0)
    ctx.check(
        "button travel moves inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.0025,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_tray_pos = ctx.part_world_position(drip_tray)
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        extended_tray_pos = ctx.part_world_position(drip_tray)
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="y",
            elem_a="left_wall",
            elem_b="left_rail",
            min_overlap=0.02,
            name="drip tray retains insertion at full extension",
        )
    ctx.check(
        "drip tray slides forward",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] < rest_tray_pos[1] - 0.05,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    closed_tab = ctx.part_element_world_aabb(tank_lid, elem="pull_tab")
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        open_tab = ctx.part_element_world_aabb(tank_lid, elem="pull_tab")
    ctx.check(
        "tank lid opens upward",
        closed_tab is not None
        and open_tab is not None
        and open_tab[1][2] > closed_tab[1][2] + 0.06,
        details=f"closed={closed_tab}, open={open_tab}",
    )

    locked_handle = ctx.part_element_world_aabb(portafilter, elem="handle")
    with ctx.pose({portafilter_joint: portafilter_joint.motion_limits.lower}):
        rotated_handle = ctx.part_element_world_aabb(portafilter, elem="handle")
    ctx.check(
        "portafilter handle swings around the brew axis",
        locked_handle is not None
        and rotated_handle is not None
        and abs(((rotated_handle[0][0] + rotated_handle[1][0]) * 0.5) - ((locked_handle[0][0] + locked_handle[1][0]) * 0.5))
        > 0.03,
        details=f"locked={locked_handle}, rotated={rotated_handle}",
    )

    parked_tip = ctx.part_element_world_aabb(wand, elem="tip")
    with ctx.pose({wand_joint: wand_joint.motion_limits.lower}):
        swung_tip = ctx.part_element_world_aabb(wand, elem="tip")
    ctx.check(
        "wand pivots around the side support",
        parked_tip is not None
        and swung_tip is not None
        and (
            (
                (((swung_tip[0][0] + swung_tip[1][0]) * 0.5) - ((parked_tip[0][0] + parked_tip[1][0]) * 0.5)) ** 2
                + (((swung_tip[0][1] + swung_tip[1][1]) * 0.5) - ((parked_tip[0][1] + parked_tip[1][1]) * 0.5)) ** 2
            )
            ** 0.5
        )
        > 0.02,
        details=f"parked={parked_tip}, swung={swung_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
