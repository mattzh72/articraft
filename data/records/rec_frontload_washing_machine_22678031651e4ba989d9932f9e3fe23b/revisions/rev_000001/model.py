from __future__ import annotations

import math

from sdk import ArticulatedObject, ArticulationType, Box, Cylinder, Material, MotionLimits, Origin, Sphere, TestContext, TestReport


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washing_machine")
    enamel = Material("white_enamel", color=(0.92, 0.94, 0.93, 1)); dark = Material("black_shadow", color=(0.02, 0.022, 0.024, 1))
    rubber = Material("dark_rubber", color=(0.03, 0.035, 0.035, 1)); steel = Material("brushed_steel", color=(0.72, 0.72, 0.68, 1))
    glass = Material("blue_tinted_glass", color=(0.40, 0.65, 0.85, 0.38)); grey = Material("warm_grey_plastic", color=(0.55, 0.56, 0.55, 1)); blue = Material("button_blue", color=(0.1, 0.28, 0.8, 1))
    model.materials.extend([enamel, dark, rubber, steel, glass, grey, blue])

    cabinet = model.part("cabinet")
    cabinet.visual(Box((0.72, 0.62, 0.88)), origin=Origin(xyz=(0, 0, 0.44)), material=enamel, name="enamel_shell")
    cabinet.visual(Box((0.76, 0.045, 0.13)), origin=Origin(xyz=(0, -0.333, 0.79)), material=enamel, name="raised_fascia")
    cabinet.visual(Box((0.70, 0.018, 0.055)), origin=Origin(xyz=(0, -0.352, 0.055)), material=dark, name="recessed_toe_kick")
    cabinet.visual(Cylinder(0.255, 0.045), origin=Origin(xyz=(0, -0.356, 0.45), rpy=(math.pi/2, 0, 0)), material=grey, name="front_bezel")
    cabinet.visual(Cylinder(0.207, 0.052), origin=Origin(xyz=(0, -0.365, 0.45), rpy=(math.pi/2, 0, 0)), material=rubber, name="gasket_lip")
    cabinet.visual(Cylinder(0.235, 0.20), origin=Origin(xyz=(0, -0.235, 0.45), rpy=(math.pi/2, 0, 0)), material=grey, name="fixed_outer_tub")
    cabinet.visual(Cylinder(0.035, 0.035), origin=Origin(xyz=(0.285, -0.372, 0.45), rpy=(math.pi/2, 0, 0)), material=dark, name="latch_receiver")
    cabinet.visual(Box((0.05, 0.055, 0.075)), origin=Origin(xyz=(-0.285, -0.365, 0.56)), material=steel, name="hinge_bracket_0")
    cabinet.visual(Cylinder(0.018, 0.095), origin=Origin(xyz=(-0.315, -0.375, 0.56)), material=steel, name="hinge_barrel_0")
    cabinet.visual(Box((0.05, 0.055, 0.075)), origin=Origin(xyz=(-0.285, -0.365, 0.34)), material=steel, name="hinge_bracket_1")
    cabinet.visual(Cylinder(0.018, 0.095), origin=Origin(xyz=(-0.315, -0.375, 0.34)), material=steel, name="hinge_barrel_1")
    cabinet.visual(Box((0.235, 0.030, 0.070)), origin=Origin(xyz=(-0.19, -0.367, 0.795)), material=grey, name="drawer_socket")
    cabinet.visual(Box((0.18, 0.025, 0.018)), origin=Origin(xyz=(-0.19, -0.39, 0.83)), material=steel, name="drawer_rail_upper")
    cabinet.visual(Box((0.18, 0.025, 0.018)), origin=Origin(xyz=(-0.19, -0.39, 0.76)), material=steel, name="drawer_rail_lower")
    for j, x in enumerate((-0.31, 0.31)):
        cabinet.visual(Box((0.08, 0.08, 0.025)), origin=Origin(xyz=(x, 0.25, -0.012)), material=dark, name=f"rear_leveling_pad_{j}")

    door = model.part("door_frame")
    door.visual(Cylinder(0.255, 0.055), origin=Origin(xyz=(0.285, -0.022, 0), rpy=(math.pi/2, 0, 0)), material=steel, name="metal_door_frame")
    door.visual(Cylinder(0.175, 0.060), origin=Origin(xyz=(0.285, -0.055, 0), rpy=(math.pi/2, 0, 0)), material=glass, name="convex_glass_window")
    door.visual(Box((0.045, 0.05, 0.08)), origin=Origin(xyz=(0.565, -0.02, 0)), material=dark, name="door_latch_tongue")
    door.visual(Cylinder(0.014, 0.10), origin=Origin(xyz=(-0.025, 0.005, 0.11)), material=steel, name="hinge_pin_0")
    door.visual(Cylinder(0.014, 0.10), origin=Origin(xyz=(-0.025, 0.005, -0.11)), material=steel, name="hinge_pin_1")
    model.articulation("door_hinge", ArticulationType.REVOLUTE, parent=cabinet, child=door, origin=Origin(xyz=(-0.285, -0.405, 0.45)), axis=(0, 0, -1), motion_limits=MotionLimits(5, 1.5, 0, 1.9))

    drum = model.part("wash_drum")
    drum.visual(Cylinder(0.175, 0.28), origin=Origin(rpy=(math.pi/2, 0, 0)), material=steel, name="perforated_drum_shell")
    drum.visual(Cylinder(0.055, 0.035), origin=Origin(xyz=(0, 0.16, 0), rpy=(math.pi/2, 0, 0)), material=steel, name="rear_hub")
    drum.visual(Sphere(0.003), origin=Origin(xyz=(0.115, -0.075, 0.0)), material=dark, name="drum_perforation_0")
    for i in range(1, 18):
        a = 2 * math.pi * i / 18
        drum.visual(Sphere(0.003), origin=Origin(xyz=(0.115 * math.cos(a), -0.075, 0.115 * math.sin(a))), material=dark, name=f"drum_perforation_{i}")
    model.articulation("drum_spin", ArticulationType.CONTINUOUS, parent=cabinet, child=drum, origin=Origin(xyz=(0, -0.235, 0.45)), axis=(0, 1, 0), motion_limits=MotionLimits(10, 20))

    drawer = model.part("detergent_drawer")
    drawer.visual(Box((0.22, 0.22, 0.065)), origin=Origin(xyz=(0, -0.11, 0)), material=grey, name="drawer_tray")
    for j, x in enumerate((-0.075, 0, 0.075)):
        drawer.visual(Box((0.055, 0.14, 0.010)), origin=Origin(xyz=(x, -0.11, 0.04)), material=dark, name=f"open_compartment_{j}")
    drawer.visual(Box((0.006, 0.18, 0.052)), origin=Origin(xyz=(-0.035, -0.11, 0.015)), material=enamel, name="divider_0")
    drawer.visual(Box((0.006, 0.18, 0.052)), origin=Origin(xyz=(0.035, -0.11, 0.015)), material=enamel, name="divider_1")
    model.articulation("drawer_slide", ArticulationType.PRISMATIC, parent=cabinet, child=drawer, origin=Origin(xyz=(-0.19, -0.37, 0.795)), axis=(0, -1, 0), motion_limits=MotionLimits(8, 0.3, 0, 0.18))

    knob = model.part("cycle_knob")
    knob.visual(Cylinder(0.052, 0.045), origin=Origin(xyz=(0, -0.025, 0), rpy=(math.pi/2, 0, 0)), material=grey, name="knob_cap")
    knob.visual(Cylinder(0.062, 0.015), origin=Origin(xyz=(0, 0.006, 0), rpy=(math.pi/2, 0, 0)), material=steel, name="knob_collar")
    knob.visual(Box((0.010, 0.012, 0.045)), origin=Origin(xyz=(0, -0.052, 0.025)), material=dark, name="knob_pointer")
    model.articulation("knob_rotate", ArticulationType.CONTINUOUS, parent=cabinet, child=knob, origin=Origin(xyz=(0.245, -0.36, 0.80)), axis=(0, 1, 0), motion_limits=MotionLimits(1, 4))

    for i, x in enumerate((-0.02, 0.035, 0.09, 0.145)):
        b = model.part(f"button_{i}")
        b.visual(Box((0.040, 0.020, 0.026)), origin=Origin(xyz=(0, -0.010, 0)), material=grey, name="button_cap")
        b.visual(Box((0.050, 0.010, 0.036)), origin=Origin(xyz=(0, 0.002, 0)), material=dark, name="button_socket")
        model.articulation(f"button_{i}_press", ArticulationType.PRISMATIC, parent=cabinet, child=b, origin=Origin(xyz=(x, -0.36, 0.735)), axis=(0, 1, 0), motion_limits=MotionLimits(1, 0.1, -0.012, 0))
    start = model.part("start_button")
    start.visual(Box((0.075, 0.018, 0.020)), origin=Origin(xyz=(0, -0.009, 0)), material=blue, name="slim_start_cap")
    start.visual(Box((0.085, 0.010, 0.030)), origin=Origin(xyz=(0, 0.002, 0)), material=dark, name="start_socket")
    model.articulation("start_press", ArticulationType.PRISMATIC, parent=cabinet, child=start, origin=Origin(xyz=(0.33, -0.36, 0.735)), axis=(0, 1, 0), motion_limits=MotionLimits(1, 0.1, -0.010, 0))

    hatch = model.part("service_hatch")
    hatch.visual(Box((0.20, 0.025, 0.11)), origin=Origin(xyz=(0, -0.012, 0.055)), material=enamel, name="hatch_panel")
    hatch.visual(Cylinder(0.012, 0.22), origin=Origin(xyz=(0, -0.012, 0.0), rpy=(0, math.pi/2, 0)), material=steel, name="hatch_hinge_pin")
    model.articulation("hatch_hinge", ArticulationType.REVOLUTE, parent=cabinet, child=hatch, origin=Origin(xyz=(0.15, -0.355, 0.075)), axis=(1, 0, 0), motion_limits=MotionLimits(2, 1, -1.4, 0))
    cap = model.part("drain_cap")
    cap.visual(Cylinder(0.035, 0.020), origin=Origin(xyz=(0, -0.010, 0), rpy=(math.pi/2, 0, 0)), material=dark, name="round_filter_cap")
    model.articulation("drain_cap_twist", ArticulationType.REVOLUTE, parent=hatch, child=cap, origin=Origin(xyz=(0.055, -0.014, 0.055)), axis=(0, 1, 0), motion_limits=MotionLimits(1, 2, -0.8, 0.8))
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.allow_overlap("cabinet", "wash_drum", elem_a="fixed_outer_tub", elem_b="perforated_drum_shell", reason="The rotating perforated inner drum is intentionally nested inside the fixed outer tub volume.")
    ctx.allow_overlap("cabinet", "wash_drum", elem_a="fixed_outer_tub", elem_b="drum_perforation_0", reason="Perforation detail is seated on the inner drum surface within the fixed tub cavity.")
    ctx.allow_overlap("cabinet", "wash_drum", elem_a="enamel_shell", elem_b="perforated_drum_shell", reason="The cabinet is a simplified hollow appliance body, so the drum occupies its internal void.")
    ctx.allow_overlap("cabinet", "detergent_drawer", elem_a="drawer_socket", elem_b="drawer_tray", reason="The detergent drawer tray is intentionally retained inside the fascia socket at the closed position.")
    ctx.allow_overlap("cabinet", "detergent_drawer", elem_a="drawer_rail_upper", elem_b="drawer_tray", reason="Drawer guide rails are seated into shallow slots in the tray sidewalls.")
    ctx.allow_overlap("cabinet", "detergent_drawer", elem_a="drawer_rail_lower", elem_b="drawer_tray", reason="Drawer guide rails are seated into shallow slots in the tray sidewalls.")
    ctx.allow_overlap("cabinet", "cycle_knob", elem_a="raised_fascia", elem_b="knob_collar", reason="The knob collar is a seated rotary bushing captured in the control fascia.")
    ctx.allow_overlap("cabinet", "door_frame", elem_a="hinge_barrel_0", elem_b="hinge_pin_0", reason="The visible hinge pin is captured inside the hinge barrel.")
    ctx.allow_overlap("cabinet", "door_frame", elem_a="hinge_bracket_0", elem_b="hinge_pin_0", reason="The hinge pin passes through the hinge bracket knuckle.")
    ctx.allow_overlap("cabinet", "door_frame", elem_a="hinge_barrel_1", elem_b="hinge_pin_1", reason="The second visible hinge pin is captured inside the lower hinge barrel.")
    ctx.allow_overlap("cabinet", "door_frame", elem_a="hinge_bracket_1", elem_b="hinge_pin_1", reason="The lower hinge pin passes through the hinge bracket knuckle.")
    ctx.allow_overlap("cabinet", "service_hatch", elem_a="recessed_toe_kick", elem_b="hatch_panel", reason="The service hatch lower lip seats into the shallow toe-kick recess.")
    ctx.allow_overlap("cabinet", "service_hatch", elem_a="recessed_toe_kick", elem_b="hatch_hinge_pin", reason="The hatch hinge pin runs through the toe-kick hinge knuckle.")
    ctx.allow_overlap("cabinet", "wash_drum", elem_a="front_bezel", elem_b="perforated_drum_shell", reason="The drum front rim sits behind and partly within the circular front bezel opening.")
    ctx.allow_overlap("cabinet", "wash_drum", elem_a="gasket_lip", elem_b="perforated_drum_shell", reason="The rubber gasket lip slightly overlaps the rotating drum mouth as a simplified flexible seal.")
    ctx.allow_overlap("cabinet", "detergent_drawer", elem_a="drawer_rail_upper", elem_b="divider_0", reason="The open drawer divider is locally notched by the upper guide rail in this simplified tray.")
    ctx.allow_overlap("cabinet", "detergent_drawer", elem_a="drawer_rail_upper", elem_b="divider_1", reason="The open drawer divider is locally notched by the upper guide rail in this simplified tray.")
    ctx.allow_overlap("cabinet", "wash_drum", elem_a="enamel_shell", elem_b="rear_hub", reason="The rear drum hub is housed inside the simplified hollow cabinet shell.")
    ctx.allow_overlap("drain_cap", "service_hatch", elem_a="round_filter_cap", elem_b="hatch_panel", reason="The drain filter cap is a twist plug captured in the service hatch panel.")
    ctx.expect_overlap("door_frame", "cabinet", axes="xz", min_overlap=0.08, name="door covers front gasket")
    ctx.expect_overlap("cabinet", "door_frame", axes="z", min_overlap=0.08, elem_a="hinge_barrel_0", elem_b="hinge_pin_0", name="hinge pin retained in barrel")
    ctx.expect_overlap("detergent_drawer", "cabinet", axes="y", min_overlap=0.01, elem_a="drawer_tray", elem_b="drawer_socket", name="drawer remains captured in socket")
    ctx.expect_overlap("drain_cap", "service_hatch", axes="xz", min_overlap=0.03, name="drain cap seated in hatch")
    with ctx.pose(door_hinge=1.2):
        aabb = ctx.part_world_aabb("door_frame")
        ctx.check("door swings clear of front opening", aabb is not None and aabb[0][1] < -0.90, details=f"door_aabb={aabb}")
    with ctx.pose(drawer_slide=0.16):
        pos = ctx.part_world_position("detergent_drawer")
        ctx.check("detergent drawer slides out forward", pos is not None and pos[1] < -0.50, details=f"drawer_pos={pos}")
    ctx.expect_within("wash_drum", "cabinet", axes="xz", margin=0.0, name="drum nested within cabinet front")
    return ctx.report()


object_model = build_object_model()
