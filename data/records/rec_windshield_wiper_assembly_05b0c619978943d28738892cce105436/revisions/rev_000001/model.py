from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HALF_PI = math.pi / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_windshield_wiper")

    black = model.material("satin_black", rgba=(0.015, 0.017, 0.016, 1.0))
    rubber = model.material("aged_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    steel = model.material("cadmium_steel", rgba=(0.55, 0.52, 0.45, 1.0))
    cast = model.material("cast_aluminum", rgba=(0.48, 0.50, 0.49, 1.0))
    brass = model.material("brass_bushings", rgba=(0.72, 0.56, 0.22, 1.0))
    glass = model.material("smoky_glass", rgba=(0.22, 0.40, 0.52, 0.34))
    gasket = model.material("black_gasket", rgba=(0.01, 0.01, 0.009, 1.0))
    red = model.material("red_service_paint", rgba=(0.55, 0.07, 0.035, 1.0))

    frame = model.part("cowl_frame")
    frame.visual(Box((1.40, 0.20, 0.028)), origin=Origin(xyz=(0.0, 0.0, 0.014)), material=black, name="cowl_tray")
    frame.visual(Box((1.40, 0.020, 0.030)), origin=Origin(xyz=(0.0, -0.185, 0.015)), material=black, name="front_lip")
    frame.visual(Box((0.72, 0.095, 0.012)), origin=Origin(xyz=(0.0, -0.139, 0.014)), material=black, name="front_bridge")
    frame.visual(Box((1.40, 0.020, 0.120)), origin=Origin(xyz=(0.0, 0.096, 0.074)), material=black, name="rear_riser")
    frame.visual(Box((0.720, 0.352, 0.018)), origin=Origin(xyz=(0.0, 0.266, 0.136)), material=gasket, name="glass_gasket")
    frame.visual(Box((0.170, 0.040, 0.018)), origin=Origin(xyz=(-0.585, 0.426, 0.136)), material=gasket, name="side_gasket_0")
    frame.visual(Box((0.170, 0.040, 0.018)), origin=Origin(xyz=(0.585, 0.426, 0.136)), material=gasket, name="side_gasket_1")
    frame.visual(Box((1.34, 0.010, 0.420)), origin=Origin(xyz=(0.0, 0.426, 0.355)), material=glass, name="windshield_lower")

    # Legacy motor and stamped cowl reinforcement structure.
    frame.visual(Box((0.26, 0.12, 0.060)), origin=Origin(xyz=(0.0, -0.008, 0.058)), material=cast, name="motor_pedestal")
    frame.visual(Cylinder(radius=0.078, length=0.040), origin=Origin(xyz=(0.0, 0.025, 0.118)), material=cast, name="gear_cover")
    frame.visual(Cylinder(radius=0.055, length=0.120), origin=Origin(xyz=(0.0, -0.070, 0.094), rpy=(HALF_PI, 0.0, 0.0)), material=black, name="motor_can")
    frame.visual(Cylinder(radius=0.058, length=0.012), origin=Origin(xyz=(0.0, -0.132, 0.094), rpy=(HALF_PI, 0.0, 0.0)), material=cast, name="motor_endcap")
    frame.visual(Cylinder(radius=0.018, length=0.96), origin=Origin(xyz=(0.0, 0.085, 0.094), rpy=(0.0, HALF_PI, 0.0)), material=steel, name="cross_tube")
    frame.visual(Box((0.18, 0.034, 0.035)), origin=Origin(xyz=(-0.16, 0.005, 0.080), rpy=(0.0, 0.0, -0.22)), material=steel, name="reinforcing_strap_0")
    frame.visual(Box((0.18, 0.034, 0.035)), origin=Origin(xyz=(0.16, 0.005, 0.080), rpy=(0.0, 0.0, 0.22)), material=steel, name="reinforcing_strap_1")

    # Service hatch frames, hinges, and old-fashioned slotted fasteners.
    hatch_specs = (
        (0, -0.31, "hatch_knuckle_0a", "hatch_knuckle_0b"),
        (1, 0.31, "hatch_knuckle_1a", "hatch_knuckle_1b"),
    )
    for i, x, hinge_knuckle_a, hinge_knuckle_b in hatch_specs:
        frame.visual(Box((0.34, 0.096, 0.028)), origin=Origin(xyz=(x, -0.151, 0.050)), material=black, name=f"hatch_plinth_{i}")
        frame.visual(Box((0.040, 0.050, 0.056)), origin=Origin(xyz=(x + (0.060 if x < 0.0 else -0.060), -0.139, 0.028)), material=black, name=f"hatch_post_{i}")
        frame.visual(Box((0.32, 0.014, 0.018)), origin=Origin(xyz=(x, -0.188, 0.066)), material=steel, name=f"hatch_front_rail_{i}")
        frame.visual(Box((0.32, 0.014, 0.018)), origin=Origin(xyz=(x, -0.113, 0.051)), material=steel, name=f"hatch_hinge_rail_{i}")
        frame.visual(Box((0.014, 0.088, 0.018)), origin=Origin(xyz=(x - 0.160, -0.151, 0.066)), material=steel, name=f"hatch_side_rail_{i}a")
        frame.visual(Box((0.014, 0.088, 0.018)), origin=Origin(xyz=(x + 0.160, -0.151, 0.066)), material=steel, name=f"hatch_side_rail_{i}b")
        if i == 0:
            frame.visual(Cylinder(radius=0.006, length=0.068), origin=Origin(xyz=(x - 0.120, -0.113, 0.076), rpy=(0.0, HALF_PI, 0.0)), material=steel, name="hatch_knuckle_0a")
            frame.visual(Cylinder(radius=0.006, length=0.068), origin=Origin(xyz=(x + 0.120, -0.113, 0.076), rpy=(0.0, HALF_PI, 0.0)), material=steel, name="hatch_knuckle_0b")
        else:
            frame.visual(Cylinder(radius=0.006, length=0.068), origin=Origin(xyz=(x - 0.120, -0.113, 0.076), rpy=(0.0, HALF_PI, 0.0)), material=steel, name="hatch_knuckle_1a")
            frame.visual(Cylinder(radius=0.006, length=0.068), origin=Origin(xyz=(x + 0.120, -0.113, 0.076), rpy=(0.0, HALF_PI, 0.0)), material=steel, name="hatch_knuckle_1b")
        for bx in (-0.115, 0.115):
            frame.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(x + bx, -0.178, 0.078)), material=steel, name=f"hatch_frame_bolt_{i}_{bx}")

    # Supported spindle pivots with bolted adapters and brass sleeves.
    spindle_specs = ((0, -0.48, "bearing_sleeve_0"), (1, 0.48, "bearing_sleeve_1"))
    for i, x, sleeve_name in spindle_specs:
        if i == 0:
            frame.visual(Box((0.16, 0.11, 0.060)), origin=Origin(xyz=(x, 0.025, 0.058)), material=cast, name="spindle_pedestal_0")
        else:
            frame.visual(Box((0.16, 0.11, 0.060)), origin=Origin(xyz=(x, 0.025, 0.058)), material=cast, name="spindle_pedestal_1")
        if i == 0:
            frame.visual(Box((0.205, 0.135, 0.012)), origin=Origin(xyz=(x, 0.025, 0.093)), material=steel, name="spindle_adapter_0")
        else:
            frame.visual(Box((0.205, 0.135, 0.012)), origin=Origin(xyz=(x, 0.025, 0.093)), material=steel, name="spindle_adapter_1")
        if i == 0:
            frame.visual(Cylinder(radius=0.040, length=0.095), origin=Origin(xyz=(x, 0.025, 0.139)), material=brass, name="bearing_sleeve_0")
        else:
            frame.visual(Cylinder(radius=0.040, length=0.095), origin=Origin(xyz=(x, 0.025, 0.139)), material=brass, name="bearing_sleeve_1")
        frame.visual(Box((0.030, 0.120, 0.040)), origin=Origin(xyz=(x - 0.062, 0.075, 0.078), rpy=(0.0, 0.0, -0.20)), material=steel, name=f"gusset_{i}a")
        frame.visual(Box((0.030, 0.120, 0.040)), origin=Origin(xyz=(x + 0.062, 0.075, 0.078), rpy=(0.0, 0.0, 0.20)), material=steel, name=f"gusset_{i}b")
        for bx in (-0.070, 0.070):
            for by in (-0.045, 0.045):
                frame.visual(Cylinder(radius=0.010, length=0.007), origin=Origin(xyz=(x + bx, 0.025 + by, 0.102)), material=steel, name=f"adapter_bolt_{i}_{bx}_{by}")

    # Motor crank: a visible eccentric lever with a counterweight and service nut.
    crank = model.part("motor_crank")
    crank.visual(Cylinder(radius=0.028, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.014)), material=steel, name="crank_hub")
    crank.visual(Box((0.168, 0.030, 0.012)), origin=Origin(xyz=(0.084, 0.0, 0.030)), material=steel, name="crank_arm")
    crank.visual(Cylinder(radius=0.013, length=0.040), origin=Origin(xyz=(0.160, 0.0, 0.055)), material=steel, name="crank_pin")
    crank.visual(Cylinder(radius=0.018, length=0.010), origin=Origin(xyz=(-0.060, 0.0, 0.031)), material=cast, name="counterweight")
    crank.visual(Box((0.056, 0.018, 0.010)), origin=Origin(xyz=(-0.030, 0.0, 0.031)), material=cast, name="weight_web")
    crank.visual(Cylinder(radius=0.017, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.031)), material=brass, name="hub_nut")

    model.articulation(
        "motor_to_crank",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.0, 0.025, 0.146)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-0.65, upper=0.65),
    )

    # Crank-to-spindle drag link with pressed eyes.
    drive_angle = math.atan2(-0.080, 0.400)
    drive_length = math.hypot(0.400, 0.080)
    drive_link = model.part("drive_link")
    drive_link.visual(Box((drive_length - 0.052, 0.020, 0.010)), origin=Origin(xyz=(drive_length / 2.0, 0.0, 0.0)), material=steel, name="drive_bar")
    drive_link.visual(Cylinder(radius=0.030, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=steel, name="crank_eye")
    drive_link.visual(Cylinder(radius=0.030, length=0.012), origin=Origin(xyz=(drive_length, 0.0, 0.0)), material=steel, name="spindle_eye")
    drive_link.visual(Box((0.060, 0.014, 0.017)), origin=Origin(xyz=(drive_length / 2.0, 0.0, 0.011)), material=red, name="paint_mark")

    model.articulation(
        "crank_to_drive_link",
        ArticulationType.REVOLUTE,
        parent=crank,
        child=drive_link,
        origin=Origin(xyz=(0.160, 0.0, 0.055), rpy=(0.0, 0.0, drive_angle)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.95, upper=0.95),
    )

    # Two supported wiper carriers.  Their shafts run through the brass sleeves;
    # the blade end uses a modern rubber insert on a stamped, retro-looking arm.
    carriers = []
    for i, x in enumerate((-0.48, 0.48)):
        carrier = model.part(f"wiper_carrier_{i}")
        carrier.visual(Cylinder(radius=0.016, length=0.240), origin=Origin(xyz=(0.0, 0.0, -0.055)), material=steel, name="spindle_shaft")
        carrier.visual(Box((0.090, 0.060, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.052)), material=cast, name="arm_adapter")
        carrier.visual(Box((0.030, 0.360, 0.012)), origin=Origin(xyz=(0.0, 0.180, 0.061)), material=black, name="stamped_arm")
        carrier.visual(Cylinder(radius=0.009, length=0.006), origin=Origin(xyz=(0.0, 0.105, 0.070)), material=steel, name="arm_rivet_0")
        carrier.visual(Cylinder(radius=0.009, length=0.006), origin=Origin(xyz=(0.0, 0.250, 0.070)), material=steel, name="arm_rivet_1")
        carrier.visual(Box((0.085, 0.055, 0.026)), origin=Origin(xyz=(0.0, 0.360, 0.054)), material=cast, name="blade_saddle")
        carrier.visual(Box((0.440, 0.020, 0.018)), origin=Origin(xyz=(0.0, 0.380, 0.050)), material=steel, name="blade_backing")
        carrier.visual(Box((0.460, 0.012, 0.035)), origin=Origin(xyz=(0.0, 0.391, 0.031)), material=rubber, name="rubber_blade")
        if i == 0:
            carrier.visual(Box((0.080, 0.220, 0.008)), origin=Origin(xyz=(0.025, -0.070, -0.155)), material=steel, name="bellcrank_arm")
            carrier.visual(Cylinder(radius=0.010, length=0.050), origin=Origin(xyz=(0.055, -0.150, -0.155)), material=steel, name="tie_pin")
        else:
            carrier.visual(Box((0.080, 0.220, 0.008)), origin=Origin(xyz=(-0.025, -0.070, -0.155)), material=steel, name="bellcrank_arm")
            carrier.visual(Cylinder(radius=0.010, length=0.050), origin=Origin(xyz=(-0.055, -0.150, -0.155)), material=steel, name="tie_pin")
            carrier.visual(Box((0.120, 0.090, 0.010)), origin=Origin(xyz=(0.040, -0.040, 0.055)), material=steel, name="drive_bellcrank")
            carrier.visual(Cylinder(radius=0.010, length=0.040), origin=Origin(xyz=(0.080, -0.080, 0.055)), material=steel, name="drive_pin")
        carriers.append(carrier)
        model.articulation(
            f"frame_to_wiper_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=carrier,
            origin=Origin(xyz=(x, 0.025, 0.154)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.55, upper=0.55),
            mimic=Mimic(joint="motor_to_crank", multiplier=0.80, offset=0.0),
        )

    # Cross tie rod between the two spindle bellcranks.
    tie_rod = model.part("tie_rod")
    tie_rod.visual(Box((0.806, 0.018, 0.010)), origin=Origin(xyz=(0.425, 0.0, 0.0)), material=steel, name="tie_bar")
    tie_rod.visual(Cylinder(radius=0.028, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=steel, name="left_eye")
    tie_rod.visual(Cylinder(radius=0.028, length=0.012), origin=Origin(xyz=(0.850, 0.0, 0.0)), material=steel, name="right_eye")
    model.articulation(
        "wiper_to_tie_rod",
        ArticulationType.REVOLUTE,
        parent=carriers[0],
        child=tie_rod,
        origin=Origin(xyz=(0.055, -0.150, -0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.75, upper=0.75),
    )

    # Hinged service hatches are separate articulated panels.
    for i, x in enumerate((-0.31, 0.31)):
        hatch = model.part(f"service_hatch_{i}")
        hatch.visual(Box((0.205, 0.096, 0.008)), origin=Origin(xyz=(0.0, -0.065, 0.004)), material=black, name="hatch_panel")
        hatch.visual(Box((0.130, 0.024, 0.004)), origin=Origin(xyz=(0.0, -0.012, 0.005)), material=steel, name="hinge_leaf")
        hatch.visual(Box((0.230, 0.012, 0.010)), origin=Origin(xyz=(0.0, -0.088, 0.013)), material=red, name="service_label")
        hatch.visual(Cylinder(radius=0.006, length=0.130), origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(0.0, HALF_PI, 0.0)), material=steel, name="hatch_barrel")
        hatch.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(-0.095, -0.030, 0.011)), material=steel, name="hatch_screw_0")
        hatch.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(0.095, -0.030, 0.011)), material=steel, name="hatch_screw_1")
        hatch.visual(Box((0.055, 0.010, 0.010)), origin=Origin(xyz=(0.0, -0.069, 0.013)), material=steel, name="pull_tab")
        model.articulation(
            f"frame_to_hatch_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=hatch,
            origin=Origin(xyz=(x, -0.113, 0.071)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=0.0, upper=1.05),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("cowl_frame")
    crank = object_model.get_part("motor_crank")
    drive_link = object_model.get_part("drive_link")
    wiper_0 = object_model.get_part("wiper_carrier_0")
    wiper_1 = object_model.get_part("wiper_carrier_1")
    tie_rod = object_model.get_part("tie_rod")
    motor_joint = object_model.get_articulation("motor_to_crank")
    hatch_joint = object_model.get_articulation("frame_to_hatch_0")

    # Intentional local embeds: real assemblies have shafts/pins captured inside
    # bushings and pressed eyes.  The model scopes each allowance to the named
    # bearing or pin elements rather than hiding broad moving-part collisions.
    ctx.allow_overlap(
        frame,
        wiper_0,
        elem_a="bearing_sleeve_0",
        elem_b="spindle_shaft",
        reason="Supported spindle shaft is intentionally captured inside the brass sleeve.",
    )
    ctx.allow_overlap(
        frame,
        wiper_1,
        elem_a="bearing_sleeve_1",
        elem_b="spindle_shaft",
        reason="Supported spindle shaft is intentionally captured inside the brass sleeve.",
    )
    ctx.allow_overlap(
        frame,
        wiper_0,
        elem_a="spindle_pedestal_0",
        elem_b="spindle_shaft",
        reason="The spindle shaft passes through a simplified solid pedestal that represents a bored legacy pivot support.",
    )
    ctx.allow_overlap(
        frame,
        wiper_1,
        elem_a="spindle_pedestal_1",
        elem_b="spindle_shaft",
        reason="The spindle shaft passes through a simplified solid pedestal that represents a bored legacy pivot support.",
    )
    ctx.allow_overlap(
        frame,
        wiper_0,
        elem_a="spindle_adapter_0",
        elem_b="spindle_shaft",
        reason="The bolted adapter plate is represented as a solid plate with the spindle shaft passing through its bored center.",
    )
    ctx.allow_overlap(
        frame,
        wiper_1,
        elem_a="spindle_adapter_1",
        elem_b="spindle_shaft",
        reason="The bolted adapter plate is represented as a solid plate with the spindle shaft passing through its bored center.",
    )
    ctx.allow_overlap(
        frame,
        wiper_0,
        elem_a="cowl_tray",
        elem_b="spindle_shaft",
        reason="The long supported spindle passes through the cowl tray opening in this simplified solid tray.",
    )
    ctx.allow_overlap(
        frame,
        wiper_1,
        elem_a="cowl_tray",
        elem_b="spindle_shaft",
        reason="The long supported spindle passes through the cowl tray opening in this simplified solid tray.",
    )
    ctx.allow_overlap(
        crank,
        drive_link,
        elem_a="crank_pin",
        elem_b="crank_eye",
        reason="Crank pin is intentionally represented seated through the pressed link eye.",
    )
    ctx.allow_overlap(
        wiper_1,
        drive_link,
        elem_a="drive_pin",
        elem_b="spindle_eye",
        reason="Drive-link eye is intentionally captured on the wiper bellcrank pin.",
    )
    ctx.allow_overlap(
        wiper_0,
        tie_rod,
        elem_a="tie_pin",
        elem_b="left_eye",
        reason="Tie-rod eye is intentionally captured on the first spindle bellcrank pin.",
    )
    ctx.allow_overlap(
        wiper_1,
        tie_rod,
        elem_a="tie_pin",
        elem_b="right_eye",
        reason="Tie-rod eye is intentionally captured on the second spindle bellcrank pin.",
    )
    ctx.allow_overlap(
        wiper_0,
        tie_rod,
        elem_a="bellcrank_arm",
        elem_b="left_eye",
        reason="Tie-rod eye is stacked on the bellcrank plate around the captured pin.",
    )
    ctx.allow_overlap(
        wiper_1,
        tie_rod,
        elem_a="bellcrank_arm",
        elem_b="right_eye",
        reason="Tie-rod eye is stacked on the bellcrank plate around the captured pin.",
    )
    ctx.allow_overlap(
        frame,
        object_model.get_part("service_hatch_0"),
        elem_a="hatch_knuckle_0b",
        elem_b="hatch_barrel",
        reason="The exposed service-hatch hinge barrel intentionally interleaves with the frame knuckle.",
    )
    ctx.allow_overlap(
        frame,
        object_model.get_part("service_hatch_1"),
        elem_a="hatch_knuckle_1a",
        elem_b="hatch_barrel",
        reason="The exposed service-hatch hinge barrel intentionally interleaves with the frame knuckle.",
    )

    ctx.expect_within(wiper_0, frame, axes="xy", inner_elem="spindle_shaft", outer_elem="bearing_sleeve_0", margin=0.002, name="first spindle centered in sleeve")
    ctx.expect_overlap(wiper_0, frame, axes="z", elem_a="spindle_shaft", elem_b="bearing_sleeve_0", min_overlap=0.070, name="first spindle retained vertically")
    ctx.expect_within(wiper_0, frame, axes="xy", inner_elem="spindle_shaft", outer_elem="spindle_pedestal_0", margin=0.002, name="first spindle passes through pedestal")
    ctx.expect_overlap(wiper_0, frame, axes="z", elem_a="spindle_shaft", elem_b="spindle_pedestal_0", min_overlap=0.025, name="first spindle supported by pedestal")
    ctx.expect_within(wiper_0, frame, axes="xy", inner_elem="spindle_shaft", outer_elem="spindle_adapter_0", margin=0.002, name="first spindle passes through adapter")
    ctx.expect_overlap(wiper_0, frame, axes="z", elem_a="spindle_shaft", elem_b="spindle_adapter_0", min_overlap=0.010, name="first adapter captures spindle")
    ctx.expect_overlap(wiper_0, frame, axes="z", elem_a="spindle_shaft", elem_b="cowl_tray", min_overlap=0.015, name="first spindle passes through cowl tray")
    ctx.expect_within(wiper_1, frame, axes="xy", inner_elem="spindle_shaft", outer_elem="bearing_sleeve_1", margin=0.002, name="second spindle centered in sleeve")
    ctx.expect_overlap(wiper_1, frame, axes="z", elem_a="spindle_shaft", elem_b="bearing_sleeve_1", min_overlap=0.070, name="second spindle retained vertically")
    ctx.expect_within(wiper_1, frame, axes="xy", inner_elem="spindle_shaft", outer_elem="spindle_pedestal_1", margin=0.002, name="second spindle passes through pedestal")
    ctx.expect_overlap(wiper_1, frame, axes="z", elem_a="spindle_shaft", elem_b="spindle_pedestal_1", min_overlap=0.025, name="second spindle supported by pedestal")
    ctx.expect_within(wiper_1, frame, axes="xy", inner_elem="spindle_shaft", outer_elem="spindle_adapter_1", margin=0.002, name="second spindle passes through adapter")
    ctx.expect_overlap(wiper_1, frame, axes="z", elem_a="spindle_shaft", elem_b="spindle_adapter_1", min_overlap=0.010, name="second adapter captures spindle")
    ctx.expect_overlap(wiper_1, frame, axes="z", elem_a="spindle_shaft", elem_b="cowl_tray", min_overlap=0.015, name="second spindle passes through cowl tray")
    ctx.expect_within(crank, drive_link, axes="xy", inner_elem="crank_pin", outer_elem="crank_eye", margin=0.003, name="crank pin seated in link eye")
    ctx.expect_within(wiper_1, drive_link, axes="xy", inner_elem="drive_pin", outer_elem="spindle_eye", margin=0.004, name="drive link reaches spindle bellcrank")
    ctx.expect_within(wiper_0, tie_rod, axes="xy", inner_elem="tie_pin", outer_elem="left_eye", margin=0.004, name="tie rod seated on first pin")
    ctx.expect_within(wiper_1, tie_rod, axes="xy", inner_elem="tie_pin", outer_elem="right_eye", margin=0.004, name="tie rod seated on second pin")
    ctx.expect_overlap(wiper_0, tie_rod, axes="xy", elem_a="bellcrank_arm", elem_b="left_eye", min_overlap=0.015, name="first tie eye bears on bellcrank")
    ctx.expect_overlap(wiper_1, tie_rod, axes="xy", elem_a="bellcrank_arm", elem_b="right_eye", min_overlap=0.015, name="second tie eye bears on bellcrank")
    ctx.expect_contact(object_model.get_part("service_hatch_0"), frame, elem_a="hatch_barrel", elem_b="hatch_knuckle_0b", contact_tol=0.025, name="first service hatch has hinge support")
    ctx.expect_contact(object_model.get_part("service_hatch_1"), frame, elem_a="hatch_barrel", elem_b="hatch_knuckle_1a", contact_tol=0.025, name="second service hatch has hinge support")

    rest_blade = ctx.part_element_world_aabb(wiper_0, elem="rubber_blade")
    with ctx.pose({motor_joint: 0.55}):
        swept_blade = ctx.part_element_world_aabb(wiper_0, elem="rubber_blade")
    ctx.check(
        "motor crank sweeps wiper blade",
        rest_blade is not None and swept_blade is not None and abs(swept_blade[0][0] - rest_blade[0][0]) > 0.015,
        details=f"rest={rest_blade}, swept={swept_blade}",
    )

    closed_hatch = ctx.part_world_aabb("service_hatch_0")
    with ctx.pose({hatch_joint: 0.75}):
        raised_hatch = ctx.part_world_aabb("service_hatch_0")
    ctx.check(
        "service hatch opens upward",
        closed_hatch is not None and raised_hatch is not None and raised_hatch[1][2] > closed_hatch[1][2] + 0.020,
        details=f"closed={closed_hatch}, raised={raised_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
