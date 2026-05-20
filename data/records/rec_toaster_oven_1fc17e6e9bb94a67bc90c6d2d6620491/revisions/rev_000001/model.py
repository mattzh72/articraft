from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_toaster_oven")

    steel = Material("dark_blasted_steel", rgba=(0.18, 0.19, 0.19, 1.0))
    plate = Material("oiled_plate_steel", rgba=(0.30, 0.32, 0.32, 1.0))
    black = Material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    glass = Material("smoked_heat_glass", rgba=(0.08, 0.12, 0.14, 0.45))
    rubber = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    yellow = Material("safety_yellow", rgba=(1.0, 0.75, 0.05, 1.0))
    red = Material("lockout_red", rgba=(0.78, 0.04, 0.03, 1.0))
    bolt = Material("zinc_fastener", rgba=(0.68, 0.69, 0.65, 1.0))

    def box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl_x(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_y(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_z(part, name, radius, length, xyz, material):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz), material=material, name=name)

    chassis = model.part("chassis")

    # Welded rectangular oven shell.  The front is at negative X; the control
    # column is the right-hand front strip.  Plates overlap slightly so the
    # visible body reads as one heavy welded assembly.
    box(chassis, "top_plate", (0.50, 0.62, 0.035), (0.0, 0.0, 0.1725), steel)
    box(chassis, "bottom_plate", (0.50, 0.62, 0.035), (0.0, 0.0, -0.1725), steel)
    box(chassis, "rear_plate", (0.035, 0.62, 0.38), (0.2325, 0.0, 0.0), steel)
    box(chassis, "side_plate_0", (0.50, 0.035, 0.38), (0.0, -0.2925, 0.0), steel)
    box(chassis, "side_plate_1", (0.50, 0.035, 0.38), (0.0, 0.2925, 0.0), steel)

    box(chassis, "front_left_post", (0.04, 0.055, 0.35), (-0.225, -0.265, 0.0), plate)
    box(chassis, "control_panel", (0.04, 0.135, 0.35), (-0.225, 0.242, 0.0), plate)
    box(chassis, "front_top_rail", (0.04, 0.47, 0.045), (-0.225, -0.045, 0.152), plate)
    box(chassis, "front_bottom_rail", (0.04, 0.47, 0.045), (-0.225, -0.045, -0.152), plate)
    box(chassis, "dark_cavity", (0.012, 0.395, 0.245), (-0.247, -0.065, 0.0), black)

    # Skid feet and fork pockets make the appliance look industrial rather
    # than domestic, and tie into the bottom plate with wide weld pads.
    for i, y in enumerate((-0.22, 0.22)):
        box(chassis, f"front_foot_{i}", (0.12, 0.055, 0.04), (-0.17, y, -0.215), rubber)
        box(chassis, f"rear_foot_{i}", (0.12, 0.055, 0.04), (0.17, y, -0.210), rubber)
    box(chassis, "left_fork_pocket", (0.34, 0.030, 0.045), (0.0, -0.292, -0.205), black)
    box(chassis, "right_fork_pocket", (0.34, 0.030, 0.045), (0.0, 0.292, -0.205), black)

    # External reinforcement and load paths at the high stress front corners.
    box(chassis, "top_front_angle", (0.055, 0.60, 0.024), (-0.252, 0.0, 0.190), steel)
    box(chassis, "bottom_front_angle", (0.055, 0.60, 0.024), (-0.252, 0.0, -0.190), steel)
    box(chassis, "left_corner_strap", (0.032, 0.020, 0.35), (-0.265, -0.296, 0.0), steel)
    box(chassis, "right_corner_strap", (0.032, 0.020, 0.35), (-0.265, 0.296, 0.0), steel)
    box(chassis, "hinge_backer_bar", (0.040, 0.50, 0.022), (-0.248, -0.060, -0.195), steel)
    box(chassis, "door_stop_bar", (0.080, 0.46, 0.030), (-0.289, -0.060, -0.205), yellow)
    box(chassis, "left_overtravel_stop", (0.050, 0.040, 0.055), (-0.322, -0.255, -0.115), yellow)
    box(chassis, "right_overtravel_stop", (0.050, 0.040, 0.055), (-0.322, 0.145, -0.115), yellow)
    box(chassis, "left_stop_web", (0.035, 0.040, 0.070), (-0.322, -0.255, -0.166), yellow)
    box(chassis, "right_stop_web", (0.035, 0.040, 0.070), (-0.322, 0.145, -0.166), yellow)

    # Obvious alternating door-hinge knuckles on the fixed frame.
    for i, (y, length) in enumerate(((-0.255, 0.105), (0.160, 0.105))):
        cyl_y(chassis, f"fixed_hinge_barrel_{i}", 0.018, length, (-0.263, y, -0.155), steel)
        box(chassis, f"fixed_hinge_leaf_{i}", (0.025, length + 0.020, 0.040), (-0.246, y, -0.146), steel)
    box(chassis, "hinge_pin_end_0", (0.020, 0.012, 0.030), (-0.263, -0.315, -0.155), bolt)
    box(chassis, "hinge_pin_end_1", (0.020, 0.012, 0.030), (-0.263, 0.220, -0.155), bolt)

    # Shaft bushings and mechanical over-travel stops for the three rotary
    # controls.  The stops are fixed to the control panel; the knobs rotate
    # between them.
    knob_zs = (0.085, 0.000, -0.085)
    for i, z in enumerate(knob_zs):
        box(chassis, f"bearing_block_{i}", (0.014, 0.065, 0.065), (-0.252, 0.242, z), steel)
        cyl_x(chassis, f"shaft_bushing_{i}", 0.018, 0.010, (-0.252, 0.242, z), bolt)
        cyl_x(chassis, f"stop_pin_hi_{i}", 0.006, 0.024, (-0.260, 0.242 + 0.050, z + 0.032), bolt)
        cyl_x(chassis, f"stop_pin_lo_{i}", 0.006, 0.024, (-0.260, 0.242 - 0.036, z - 0.032), bolt)

    # Control-guard cage: bars sit proud of the knobs and are carried by
    # standoff plates back to the front panel.
    box(chassis, "guard_standoff_top", (0.105, 0.165, 0.018), (-0.300, 0.242, 0.153), yellow)
    box(chassis, "guard_standoff_bottom", (0.105, 0.165, 0.018), (-0.300, 0.242, -0.153), yellow)
    box(chassis, "guard_side_0", (0.018, 0.018, 0.325), (-0.350, 0.155, 0.0), yellow)
    box(chassis, "guard_side_1", (0.018, 0.018, 0.325), (-0.350, 0.329, 0.0), yellow)
    box(chassis, "guard_top_bar", (0.018, 0.190, 0.018), (-0.350, 0.242, 0.162), yellow)
    box(chassis, "guard_bottom_bar", (0.018, 0.190, 0.018), (-0.350, 0.242, -0.162), yellow)

    # Hinge supports for a red lockout hasp that blocks the guarded controls.
    cyl_z(chassis, "lockout_fixed_barrel_0", 0.012, 0.065, (-0.382, 0.145, 0.118), yellow)
    cyl_z(chassis, "lockout_fixed_barrel_1", 0.012, 0.065, (-0.382, 0.145, -0.118), yellow)
    cyl_z(chassis, "lockout_hinge_pin", 0.0045, 0.290, (-0.382, 0.145, 0.0), bolt)
    box(chassis, "lockout_hinge_mount", (0.025, 0.030, 0.285), (-0.340, 0.145, 0.0), yellow)
    box(chassis, "lockout_leaf_0", (0.014, 0.030, 0.045), (-0.366, 0.145, 0.118), yellow)
    box(chassis, "lockout_leaf_1", (0.014, 0.030, 0.045), (-0.366, 0.145, -0.118), yellow)
    box(chassis, "lockout_stop_tab", (0.050, 0.022, 0.050), (-0.375, 0.335, 0.075), yellow)

    # Visible bolt logic on the frame, reinforcement, and control panel.
    screw_points = [
        (-0.248, -0.278, 0.145), (-0.248, -0.278, 0.030), (-0.248, -0.278, -0.120),
        (-0.248, 0.175, 0.145), (-0.248, 0.300, 0.145), (-0.248, 0.175, -0.145),
        (-0.248, 0.300, -0.145), (-0.248, -0.210, 0.170), (-0.248, 0.105, 0.170),
        (-0.248, -0.210, -0.170), (-0.248, 0.105, -0.170),
    ]
    for i, pt in enumerate(screw_points):
        cyl_x(chassis, f"frame_bolt_{i}", 0.0065, 0.006, pt, bolt)

    # Door link: frame, glass, hinge leaf, handle, latch tongue, and fasteners
    # are one welded/bolted moving assembly.  The door part frame is the hinge
    # axis; in the closed pose local +Z rises up the front of the oven.
    door = model.part("door")
    box(door, "door_bottom_rail", (0.028, 0.440, 0.026), (-0.012, -0.060, 0.045), plate)
    box(door, "door_top_rail", (0.028, 0.440, 0.035), (-0.012, -0.060, 0.276), plate)
    box(door, "door_side_0", (0.028, 0.035, 0.255), (-0.012, -0.263, 0.160), plate)
    box(door, "door_side_1", (0.028, 0.035, 0.255), (-0.012, 0.143, 0.160), plate)
    box(door, "window_glass", (0.007, 0.380, 0.190), (-0.021, -0.060, 0.166), glass)
    cyl_y(door, "moving_hinge_barrel", 0.017, 0.250, (0.0, -0.060, 0.0), steel)
    box(door, "moving_hinge_leaf", (0.020, 0.265, 0.030), (-0.012, -0.060, 0.025), steel)
    cyl_y(door, "door_handle", 0.015, 0.330, (-0.070, -0.060, 0.235), rubber)
    box(door, "handle_standoff_0", (0.055, 0.022, 0.070), (-0.045, -0.185, 0.235), steel)
    box(door, "handle_standoff_1", (0.055, 0.022, 0.070), (-0.045, 0.065, 0.235), steel)
    box(door, "latch_tongue", (0.030, 0.050, 0.030), (0.006, 0.154, 0.142), steel)
    for i, (y, z) in enumerate(((-0.230, 0.255), (0.110, 0.255), (-0.230, 0.065), (0.110, 0.065))):
        cyl_x(door, f"door_bolt_{i}", 0.006, 0.008, (-0.028, y, z), bolt)

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(-0.263, 0.0, -0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    # Three independent shaft-driven knobs.  The child frame is the shaft axis
    # on the front bearing face; the shaft ends at the bearing instead of
    # passing through a simplified solid panel.
    for i, z in enumerate(knob_zs):
        knob = model.part(f"knob_{i}")
        cyl_x(knob, "shaft", 0.010, 0.040, (-0.020, 0.0, 0.0), steel)
        cyl_x(knob, "knob_cap", 0.035, 0.030, (-0.055, 0.0, 0.0), rubber)
        box(knob, "pointer_rib", (0.008, 0.010, 0.043), (-0.073, 0.0, 0.017), yellow)
        cyl_x(knob, "set_screw_head", 0.0045, 0.005, (-0.055, 0.031, 0.0), bolt)
        model.articulation(
            f"knob_{i}_shaft",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=knob,
            origin=Origin(xyz=(-0.245, 0.242, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-2.35, upper=2.35),
        )

    lockout_bar = model.part("lockout_bar")
    cyl_z(lockout_bar, "lockout_hinge_barrel", 0.011, 0.125, (0.0, 0.0, 0.0), red)
    box(lockout_bar, "lockout_plate", (0.016, 0.172, 0.056), (-0.010, 0.088, 0.0), red)
    box(lockout_bar, "padlock_eye", (0.018, 0.034, 0.075), (-0.012, 0.176, 0.0), red)
    cyl_x(lockout_bar, "padlock_hole_marker", 0.009, 0.010, (-0.023, 0.188, 0.0), black)
    model.articulation(
        "lockout_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lockout_bar,
        origin=Origin(xyz=(-0.382, 0.145, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    # Keep a reference in metadata for tests/readability without changing the
    # authored kinematic tree.
    model.meta["primary_door_joint"] = door_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")
    lockout_hinge = object_model.get_articulation("lockout_hinge")
    control_panel = chassis.get_visual("control_panel")

    ctx.expect_gap(
        chassis,
        door,
        axis="x",
        positive_elem=chassis.get_visual("front_bottom_rail"),
        negative_elem=door.get_visual("door_bottom_rail"),
        min_gap=0.006,
        max_gap=0.030,
        name="closed door sits proud of the front frame",
    )
    ctx.expect_overlap(
        door,
        chassis,
        axes="yz",
        elem_a=door.get_visual("window_glass"),
        elem_b=chassis.get_visual("dark_cavity"),
        min_overlap=0.15,
        name="door window covers the oven opening",
    )

    for i in range(3):
        knob = object_model.get_part(f"knob_{i}")
        shaft = knob.get_visual("shaft")
        knob_cap = knob.get_visual("knob_cap")
        bearing_block = chassis.get_visual(f"bearing_block_{i}")
        shaft_bushing = chassis.get_visual(f"shaft_bushing_{i}")
        ctx.allow_overlap(
            chassis,
            knob,
            elem_a=f"bearing_block_{i}",
            elem_b="shaft",
            reason="The control shaft is intentionally captured in the bolted front bearing block.",
        )
        ctx.allow_overlap(
            chassis,
            knob,
            elem_a=f"shaft_bushing_{i}",
            elem_b="shaft",
            reason="The rotary shaft intentionally passes through the replaceable shaft bushing.",
        )
        ctx.expect_gap(
            chassis,
            knob,
            axis="x",
            positive_elem=control_panel,
            negative_elem=shaft,
            min_gap=0.0,
            max_gap=0.002,
            name=f"knob {i} shaft seats on its front bearing",
        )
        ctx.expect_overlap(
            knob,
            chassis,
            axes="yz",
            elem_a=knob_cap,
            elem_b=bearing_block,
            min_overlap=0.035,
            name=f"knob {i} aligns with its bearing block",
        )
        ctx.expect_overlap(
            knob,
            chassis,
            axes="yz",
            elem_a=shaft,
            elem_b=shaft_bushing,
            min_overlap=0.015,
            name=f"knob {i} shaft is coaxial with its bushing",
        )
        ctx.expect_gap(
            chassis,
            knob,
            axis="x",
            positive_elem=shaft_bushing,
            negative_elem=shaft,
            max_penetration=0.012,
            name=f"knob {i} shaft insertion is local to the bushing",
        )

    rest_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    with ctx.pose({door_hinge: 1.20}):
        open_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    if rest_handle is not None and open_handle is not None:
        rest_center = (
            (rest_handle[0][0] + rest_handle[1][0]) * 0.5,
            (rest_handle[0][2] + rest_handle[1][2]) * 0.5,
        )
        open_center = (
            (open_handle[0][0] + open_handle[1][0]) * 0.5,
            (open_handle[0][2] + open_handle[1][2]) * 0.5,
        )
        ctx.check(
            "door hinge drops the handle outward and down",
            open_center[0] < rest_center[0] - 0.10 and open_center[1] < rest_center[1] - 0.10,
            details=f"rest_xz={rest_center}, open_xz={open_center}",
        )
    else:
        ctx.fail("door hinge drops the handle outward and down", "door handle AABB unavailable")

    lockout = object_model.get_part("lockout_bar")
    ctx.allow_overlap(
        chassis,
        lockout,
        elem_a="lockout_hinge_pin",
        elem_b="lockout_hinge_barrel",
        reason="The hasp barrel rotates around a captured hinge pin carried by the guard frame.",
    )
    ctx.expect_overlap(
        lockout,
        chassis,
        axes="z",
        elem_a=lockout.get_visual("lockout_hinge_barrel"),
        elem_b=chassis.get_visual("lockout_hinge_pin"),
        min_overlap=0.10,
        name="lockout barrel is retained on the hinge pin",
    )
    rest_eye = ctx.part_element_world_aabb(lockout, elem="padlock_eye")
    with ctx.pose({lockout_hinge: 1.0}):
        open_eye = ctx.part_element_world_aabb(lockout, elem="padlock_eye")
    if rest_eye is not None and open_eye is not None:
        rest_x = (rest_eye[0][0] + rest_eye[1][0]) * 0.5
        open_x = (open_eye[0][0] + open_eye[1][0]) * 0.5
        ctx.check(
            "lockout hasp swings clear of the guarded knobs",
            open_x < rest_x - 0.08,
            details=f"rest_x={rest_x}, open_x={open_x}",
        )
    else:
        ctx.fail("lockout hasp swings clear of the guarded knobs", "lockout AABB unavailable")

    return ctx.report()


object_model = build_object_model()
