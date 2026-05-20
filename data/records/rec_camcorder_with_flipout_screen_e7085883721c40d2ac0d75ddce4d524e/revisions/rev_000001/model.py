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
    model = ArticulatedObject(name="professional_shoulder_camcorder")

    black = Material("matte_black", color=(0.015, 0.015, 0.014, 1.0))
    dark = Material("dark_rubber", color=(0.005, 0.005, 0.006, 1.0))
    charcoal = Material("charcoal_plastic", color=(0.08, 0.085, 0.085, 1.0))
    glass = Material("blue_black_glass", color=(0.02, 0.05, 0.08, 0.75))
    lcd_blue = Material("lit_lcd", color=(0.08, 0.18, 0.28, 1.0))
    red = Material("record_red", color=(0.85, 0.02, 0.015, 1.0))
    metal = Material("dark_metal", color=(0.18, 0.18, 0.17, 1.0))

    body = model.part("body")
    body.visual(Box((0.46, 0.20, 0.24)), origin=Origin(xyz=(0.0, 0.0, 0.22)), material=black, name="main_body")
    body.visual(Box((0.31, 0.18, 0.07)), origin=Origin(xyz=(-0.03, 0.0, 0.065)), material=charcoal, name="shoulder_pad")
    body.visual(Box((0.39, 0.08, 0.055)), origin=Origin(xyz=(-0.005, 0.0, 0.365)), material=black, name="top_rail")
    body.visual(Box((0.11, 0.035, 0.14)), origin=Origin(xyz=(-0.16, 0.0, 0.459)), material=black, name="rear_handle_post")
    body.visual(Box((0.11, 0.035, 0.14)), origin=Origin(xyz=(0.16, 0.0, 0.459)), material=black, name="front_handle_post")
    body.visual(Box((0.43, 0.07, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.55)), material=black, name="carry_handle")
    body.visual(Box((0.055, 0.24, 0.17)), origin=Origin(xyz=(-0.255, 0.0, 0.24)), material=black, name="rear_block")
    body.visual(Cylinder(0.045, 0.18), origin=Origin(xyz=(-0.37, 0.0, 0.29), rpy=(0, math.pi/2, 0)), material=dark, name="eyepiece_tube")
    body.visual(Cylinder(0.033, 0.035), origin=Origin(xyz=(-0.46, 0.0, 0.29), rpy=(0, math.pi/2, 0)), material=glass, name="eyecup_glass")
    body.visual(Cylinder(0.082, 0.035), origin=Origin(xyz=(0.246, 0.0, 0.245), rpy=(0, math.pi/2, 0)), material=metal, name="lens_mount")
    body.visual(Box((0.055, 0.020, 0.060)), origin=Origin(xyz=(0.03, 0.108, 0.26)), material=metal, name="lcd_hinge_socket")
    body.visual(Box((0.09, 0.022, 0.070)), origin=Origin(xyz=(-0.03, -0.109, 0.25)), material=metal, name="media_hinge_leaf")
    body.visual(Box((0.020, 0.016, 0.045)), origin=Origin(xyz=(-0.11, -0.104, 0.155)), material=red, name="record_button_guard")
    body.visual(Box((0.065, 0.050, 0.026)), origin=Origin(xyz=(0.10, 0.0, 0.565)), material=black, name="rocker_plinth")

    lens = model.part("lens")
    lens.visual(Cylinder(0.070, 0.24), origin=Origin(xyz=(0.12, 0, 0), rpy=(0, math.pi/2, 0)), material=black, name="lens_barrel")
    lens.visual(Cylinder(0.085, 0.055), origin=Origin(xyz=(0.26, 0, 0), rpy=(0, math.pi/2, 0)), material=dark, name="front_hood")
    lens.visual(Cylinder(0.060, 0.012), origin=Origin(xyz=(0.292, 0, 0), rpy=(0, math.pi/2, 0)), material=glass, name="front_glass")
    model.articulation("body_to_lens", ArticulationType.FIXED, parent=body, child=lens, origin=Origin(xyz=(0.255, 0, 0.245)))

    focus_ring = model.part("focus_ring")
    focus_ring.visual(Cylinder(0.078, 0.050), origin=Origin(rpy=(0, math.pi/2, 0)), material=dark, name="ribbed_focus_ring")
    model.articulation("lens_to_focus_ring", ArticulationType.CONTINUOUS, parent=lens, child=focus_ring, origin=Origin(xyz=(0.12, 0, 0)), axis=(1, 0, 0), motion_limits=MotionLimits(effort=0.4, velocity=4.0))

    lcd_arm = model.part("lcd_arm")
    lcd_arm.visual(Cylinder(0.018, 0.065), origin=Origin(rpy=(math.pi/2, 0, 0)), material=metal, name="body_hinge_barrel")
    lcd_arm.visual(Box((0.105, 0.080, 0.030)), origin=Origin(xyz=(0.0, 0.045, 0.0)), material=metal, name="swing_arm")
    lcd_arm.visual(Cylinder(0.016, 0.052), origin=Origin(xyz=(0.0, 0.092, 0.0), rpy=(math.pi/2, 0, 0)), material=metal, name="screen_pivot_barrel")
    model.articulation("body_to_lcd_arm", ArticulationType.REVOLUTE, parent=body, child=lcd_arm, origin=Origin(xyz=(0.03, 0.128, 0.26)), axis=(0, 0, 1), motion_limits=MotionLimits(effort=2, velocity=2, lower=0, upper=1.7))

    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(Box((0.155, 0.018, 0.095)), origin=Origin(xyz=(0, 0.010, 0)), material=black, name="lcd_case")
    lcd_panel.visual(Box((0.130, 0.006, 0.070)), origin=Origin(xyz=(0, 0.0215, 0.002)), material=lcd_blue, name="lcd_screen")
    lcd_panel.visual(Cylinder(0.018, 0.035), origin=Origin(xyz=(0, -0.010, 0), rpy=(math.pi/2, 0, 0)), material=metal, name="pivot_knuckle")
    model.articulation("lcd_arm_to_panel", ArticulationType.REVOLUTE, parent=lcd_arm, child=lcd_panel, origin=Origin(xyz=(0, 0.116, 0)), axis=(1, 0, 0), motion_limits=MotionLimits(effort=1, velocity=2, lower=-1.55, upper=1.55))

    media_door = model.part("media_door")
    media_door.visual(Box((0.13, 0.014, 0.12)), origin=Origin(xyz=(0, -0.007, 0)), material=charcoal, name="media_door_panel")
    media_door.visual(Cylinder(0.012, 0.09), origin=Origin(xyz=(-0.075, 0, 0), rpy=(0, 0, 0)), material=metal, name="door_hinge_pin")
    model.articulation("body_to_media_door", ArticulationType.REVOLUTE, parent=body, child=media_door, origin=Origin(xyz=(-0.03, -0.116, 0.25)), axis=(0, 0, 1), motion_limits=MotionLimits(effort=1, velocity=1.5, lower=0, upper=1.35))

    rocker = model.part("zoom_rocker")
    rocker.visual(Box((0.09, 0.035, 0.018)), origin=Origin(xyz=(0, 0, 0.006)), material=charcoal, name="rocker_paddle")
    rocker.visual(Cylinder(0.009, 0.050), origin=Origin(rpy=(0, math.pi/2, 0)), material=metal, name="rocker_axle")
    model.articulation("body_to_zoom_rocker", ArticulationType.REVOLUTE, parent=body, child=rocker, origin=Origin(xyz=(0.10, 0.0, 0.585)), axis=(1, 0, 0), motion_limits=MotionLimits(effort=0.5, velocity=3, lower=-0.18, upper=0.18))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap("body", "lens", elem_a="lens_mount", elem_b="lens_barrel", reason="The fixed lens barrel is seated inside the camera's front bayonet collar.")
    ctx.expect_overlap("body", "lens", axes="x", elem_a="lens_mount", elem_b="lens_barrel", min_overlap=0.008, name="lens seats in mount")

    ctx.allow_overlap("lens", "focus_ring", elem_a="lens_barrel", elem_b="ribbed_focus_ring", reason="The focus ring is a rotating sleeve captured around the lens barrel.")
    ctx.expect_within("lens", "focus_ring", axes="yz", inner_elem="lens_barrel", outer_elem="ribbed_focus_ring", margin=0.0, name="focus ring encircles barrel")

    ctx.allow_overlap("body", "lcd_arm", elem_a="lcd_hinge_socket", elem_b="body_hinge_barrel", reason="The LCD swing arm hinge barrel is captured inside the side socket.")
    ctx.expect_contact("body", "lcd_arm", elem_a="lcd_hinge_socket", elem_b="body_hinge_barrel", contact_tol=0.03, name="lcd arm socket captures hinge")

    ctx.allow_overlap("lcd_arm", "lcd_panel", elem_a="screen_pivot_barrel", elem_b="pivot_knuckle", reason="The screen knuckle nests in the arm-end pivot barrel.")
    ctx.expect_contact("lcd_arm", "lcd_panel", elem_a="screen_pivot_barrel", elem_b="pivot_knuckle", contact_tol=0.03, name="panel pivot is supported")

    with ctx.pose({"body_to_lcd_arm": 1.0, "lcd_arm_to_panel": 0.7}):
        ctx.expect_origin_distance("lcd_panel", "body", axes="y", min_dist=0.16, name="lcd assembly swings out from side")

    with ctx.pose({"body_to_media_door": 0.9}):
        ctx.expect_origin_distance("media_door", "body", axes="y", min_dist=0.08, name="media door swings on side hinge")

    return ctx.report()


object_model = build_object_model()
