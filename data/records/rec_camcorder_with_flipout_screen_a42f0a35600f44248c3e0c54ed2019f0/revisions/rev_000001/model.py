from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)
    if radius > 0.0:
        shape = shape.edges().fillet(radius)
    return shape


def _body_shell() -> cq.Workplane:
    """Rounded camcorder housing with real side pockets cut into the left wall."""
    length = 0.195
    width = 0.076
    height = 0.085
    body = _rounded_box((length, width, height), (0.0025, 0.0, height / 2.0), 0.008)

    side_cut_y = 0.052
    lcd_cut = cq.Workplane("XY").box(0.086, side_cut_y, 0.050).translate(
        (-0.004, 0.038 + side_cut_y / 2.0 - 0.009, 0.060)
    )
    tape_cut = cq.Workplane("XY").box(0.088, side_cut_y, 0.038).translate(
        (-0.005, 0.038 + side_cut_y / 2.0 - 0.009, 0.023)
    )
    body = body.cut(lcd_cut).cut(tape_cut)

    top_slope = cq.Workplane("XY").box(0.052, 0.090, 0.040).rotate((0, 0, 0), (0, 1, 0), -18).translate(
        (0.092, 0.0, 0.088)
    )
    return body.cut(top_slope)


def _battery_bulge() -> cq.Workplane:
    return _rounded_box((0.058, 0.084, 0.072), (-0.117, 0.0, 0.044), 0.014)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shoulder_mini_camcorder")

    charcoal = model.material("matte_charcoal", rgba=(0.055, 0.060, 0.065, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.12, 0.125, 0.13, 1.0))
    soft_black = model.material("soft_black", rgba=(0.005, 0.006, 0.007, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.011, 0.010, 1.0))
    lens_glass = model.material("blue_coated_glass", rgba=(0.02, 0.055, 0.095, 0.86))
    screen_glass = model.material("dark_lcd_glass", rgba=(0.015, 0.030, 0.045, 0.94))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.020, 0.022, 0.024, 1.0))
    dial_mat = model.material("satin_selector_dial", rgba=(0.42, 0.43, 0.42, 1.0))
    label_red = model.material("record_red_accent", rgba=(0.65, 0.025, 0.020, 1.0))

    camcorder = model.part("camcorder")
    camcorder.visual(
        mesh_from_cadquery(_body_shell(), "camcorder_body", tolerance=0.0008, angular_tolerance=0.08),
        material=charcoal,
        name="body_shell",
    )
    camcorder.visual(
        mesh_from_cadquery(_battery_bulge(), "rear_battery_bulge", tolerance=0.0008, angular_tolerance=0.08),
        material=graphite,
        name="battery_bulge",
    )

    camcorder.visual(Box((0.086, 0.0020, 0.050)), origin=Origin(xyz=(-0.004, 0.0305, 0.060)), material=soft_black, name="lcd_recess_floor")
    camcorder.visual(Box((0.088, 0.0020, 0.030)), origin=Origin(xyz=(-0.005, 0.0305, 0.024)), material=soft_black, name="tape_recess_floor")
    for name, xyz, size in (
        ("side_upper_rib", (-0.004, 0.0395, 0.086), (0.096, 0.003, 0.005)),
        ("side_middle_rib", (-0.004, 0.0395, 0.037), (0.096, 0.003, 0.004)),
        ("side_lower_rib", (-0.005, 0.0395, 0.007), (0.096, 0.003, 0.004)),
        ("side_front_rib", (0.0405, 0.0395, 0.047), (0.006, 0.003, 0.079)),
        ("side_rear_rib", (-0.052, 0.0395, 0.047), (0.004, 0.003, 0.079)),
    ):
        camcorder.visual(Box(size), origin=Origin(xyz=xyz), material=graphite, name=name)

    camcorder.visual(Cylinder(radius=0.026, length=0.070), origin=Origin(xyz=(0.128, 0.0, 0.045), rpy=(0.0, pi / 2.0, 0.0)), material=rubber, name="lens_barrel")
    camcorder.visual(Cylinder(radius=0.029, length=0.018), origin=Origin(xyz=(0.097, 0.0, 0.045), rpy=(0.0, pi / 2.0, 0.0)), material=graphite, name="front_lens_mount")
    camcorder.visual(Cylinder(radius=0.0275, length=0.010), origin=Origin(xyz=(0.165, 0.0, 0.045), rpy=(0.0, pi / 2.0, 0.0)), material=soft_black, name="lens_hood_lip")
    camcorder.visual(Cylinder(radius=0.020, length=0.0025), origin=Origin(xyz=(0.1708, 0.0, 0.045), rpy=(0.0, pi / 2.0, 0.0)), material=lens_glass, name="front_glass")
    camcorder.visual(Cylinder(radius=0.027, length=0.006), origin=Origin(xyz=(0.142, 0.0, 0.045), rpy=(0.0, pi / 2.0, 0.0)), material=charcoal, name="focus_ring")

    camcorder.visual(Box((0.138, 0.026, 0.018)), origin=Origin(xyz=(0.012, 0.0, 0.112)), material=rubber, name="top_handle_grip")
    camcorder.visual(Box((0.020, 0.030, 0.036)), origin=Origin(xyz=(-0.052, 0.0, 0.091)), material=graphite, name="rear_handle_pylon")
    camcorder.visual(Box((0.020, 0.030, 0.036)), origin=Origin(xyz=(0.074, 0.0, 0.091)), material=graphite, name="front_handle_pylon")
    camcorder.visual(Box((0.060, 0.004, 0.006)), origin=Origin(xyz=(-0.010, -0.0365, 0.070)), material=label_red, name="record_stripe")

    lcd_hinge_x = 0.034
    lcd_hinge_y = 0.034
    lcd_center_z = 0.060
    for name, z in (("lcd_hinge_lower", lcd_center_z - 0.018), ("lcd_hinge_upper", lcd_center_z + 0.018)):
        camcorder.visual(Cylinder(radius=0.0042, length=0.009), origin=Origin(xyz=(lcd_hinge_x, lcd_hinge_y + 0.002, z)), material=hinge_metal, name=name)

    tape_joint_x = -0.005
    tape_joint_y = 0.034
    tape_joint_z = 0.008
    for name, x in (("tape_hinge_lug_0", tape_joint_x - 0.032), ("tape_hinge_lug_1", tape_joint_x + 0.032)):
        camcorder.visual(Box((0.014, 0.007, 0.007)), origin=Origin(xyz=(x, tape_joint_y + 0.0015, tape_joint_z)), material=hinge_metal, name=name)
        camcorder.visual(Cylinder(radius=0.0034, length=0.012), origin=Origin(xyz=(x, tape_joint_y + 0.002, tape_joint_z), rpy=(0.0, pi / 2.0, 0.0)), material=hinge_metal, name=f"{name}_barrel")

    selector_x = -0.074
    selector_y = -0.017
    selector_z = 0.087
    camcorder.visual(Cylinder(radius=0.017, length=0.004), origin=Origin(xyz=(selector_x, selector_y, selector_z - 0.002)), material=graphite, name="selector_base_boss")

    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(Box((0.076, 0.004, 0.044)), origin=Origin(xyz=(-0.038, 0.002, 0.0)), material=graphite, name="lcd_shell")
    lcd_panel.visual(Box((0.061, 0.0010, 0.033)), origin=Origin(xyz=(-0.041, 0.00435, 0.0)), material=screen_glass, name="lcd_screen")
    lcd_panel.visual(Cylinder(radius=0.0038, length=0.026), origin=Origin(xyz=(0.0, 0.002, 0.0)), material=hinge_metal, name="lcd_hinge_sleeve")

    model.articulation(
        "lcd_hinge",
        ArticulationType.REVOLUTE,
        parent=camcorder,
        child=lcd_panel,
        origin=Origin(xyz=(lcd_hinge_x, lcd_hinge_y, lcd_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    tape_door = model.part("tape_door")
    tape_door.visual(Box((0.078, 0.004, 0.024)), origin=Origin(xyz=(0.0, 0.002, 0.015)), material=graphite, name="door_shell")
    tape_door.visual(Box((0.060, 0.001, 0.004)), origin=Origin(xyz=(-0.004, 0.00435, 0.025)), material=charcoal, name="door_grip_recess")
    tape_door.visual(Cylinder(radius=0.0032, length=0.044), origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=hinge_metal, name="door_hinge_sleeve")

    model.articulation(
        "tape_door_hinge",
        ArticulationType.REVOLUTE,
        parent=camcorder,
        child=tape_door,
        origin=Origin(xyz=(tape_joint_x, tape_joint_y, tape_joint_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.35),
    )

    selector_dial = model.part("selector_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.028,
            0.012,
            body_style="faceted",
            base_diameter=0.030,
            top_diameter=0.023,
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0009, width=0.0014),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "selector_dial",
    )
    selector_dial.visual(dial_mesh, material=dial_mat, name="dial_cap")

    model.articulation(
        "selector_spin",
        ArticulationType.CONTINUOUS,
        parent=camcorder,
        child=selector_dial,
        origin=Origin(xyz=(selector_x, selector_y, selector_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    camcorder = object_model.get_part("camcorder")
    lcd_panel = object_model.get_part("lcd_panel")
    tape_door = object_model.get_part("tape_door")
    selector_dial = object_model.get_part("selector_dial")
    lcd_hinge = object_model.get_articulation("lcd_hinge")
    tape_hinge = object_model.get_articulation("tape_door_hinge")
    selector_spin = object_model.get_articulation("selector_spin")

    body_aabb = ctx.part_world_aabb(camcorder)
    ctx.check(
        "portable camcorder scale",
        body_aabb is not None
        and 0.25 <= (body_aabb[1][0] - body_aabb[0][0]) <= 0.34
        and 0.11 <= (body_aabb[1][2] - body_aabb[0][2]) <= 0.15,
        details=f"camcorder aabb={body_aabb}",
    )
    ctx.check(
        "selector dial is continuous",
        selector_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_spin.articulation_type}",
    )

    ctx.expect_within(
        lcd_panel,
        camcorder,
        axes="xz",
        margin=0.002,
        elem_a="lcd_shell",
        elem_b="lcd_recess_floor",
        name="lcd panel sits in recessed side opening",
    )
    ctx.expect_within(
        tape_door,
        camcorder,
        axes="xz",
        margin=0.002,
        elem_a="door_shell",
        elem_b="tape_recess_floor",
        name="tape door covers lower side pocket",
    )
    ctx.expect_contact(
        selector_dial,
        camcorder,
        elem_a="dial_cap",
        elem_b="selector_base_boss",
        contact_tol=0.001,
        name="selector dial is seated on top boss",
    )

    lcd_closed = ctx.part_element_world_aabb(lcd_panel, elem="lcd_shell")
    with ctx.pose({lcd_hinge: 1.30}):
        lcd_open = ctx.part_element_world_aabb(lcd_panel, elem="lcd_shell")
    ctx.check(
        "lcd panel swings outward on vertical hinge",
        lcd_closed is not None and lcd_open is not None and lcd_open[1][1] > lcd_closed[1][1] + 0.045,
        details=f"closed={lcd_closed}, open={lcd_open}",
    )

    door_closed = ctx.part_element_world_aabb(tape_door, elem="door_shell")
    with ctx.pose({tape_hinge: 1.10}):
        door_open = ctx.part_element_world_aabb(tape_door, elem="door_shell")
    ctx.check(
        "tape door rotates downward and outward",
        door_closed is not None
        and door_open is not None
        and door_open[1][1] > door_closed[1][1] + 0.018
        and door_open[1][2] < door_closed[1][2] - 0.005,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()
