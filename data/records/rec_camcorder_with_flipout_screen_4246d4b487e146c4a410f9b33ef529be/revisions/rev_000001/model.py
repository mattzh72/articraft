from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    """Thin hollow cylindrical shell with open bore, authored along local Z."""
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2), (outer_radius, length / 2)],
        [(inner_radius, -length / 2), (inner_radius, length / 2)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def triangle_arrow_mesh(name: str, width: float, height: float, thickness: float):
    """Small raised triangular record/play arrow, extruded along local Z."""
    geom = MeshGeometry()
    pts = [(-width / 2, -height / 2), (-width / 2, height / 2), (width / 2, 0.0)]
    front = [geom.add_vertex(x, y, thickness / 2) for x, y in pts]
    back = [geom.add_vertex(x, y, -thickness / 2) for x, y in pts]
    geom.add_face(front[0], front[1], front[2])
    geom.add_face(back[2], back[1], back[0])
    for a, b in ((0, 1), (1, 2), (2, 0)):
        geom.add_face(front[a], back[a], back[b])
        geom.add_face(front[a], back[b], front[b])
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_shoulder_camcorder")

    matte = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    dark = model.material("dark_graphite", rgba=(0.06, 0.065, 0.07, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    lens_black = model.material("lens_black", rgba=(0.0, 0.0, 0.0, 1.0))
    glass = model.material("coated_glass", rgba=(0.08, 0.16, 0.22, 0.55))
    screen_glass = model.material("blue_lcd_glass", rgba=(0.03, 0.10, 0.16, 1.0))
    label = model.material("white_marking", rgba=(0.85, 0.87, 0.82, 1.0))

    body = model.part("body")
    body.visual(Box((0.34, 0.16, 0.17)), origin=Origin(xyz=(0.0, 0.0, 0.18)), material=matte, name="main_shell")
    body.visual(Box((0.26, 0.11, 0.035)), origin=Origin(xyz=(-0.01, 0.0, 0.0775)), material=rubber, name="shoulder_pad")
    body.visual(Box((0.24, 0.045, 0.035)), origin=Origin(xyz=(-0.01, 0.0, 0.2825)), material=dark, name="top_deck")
    body.visual(Box((0.030, 0.035, 0.085)), origin=Origin(xyz=(-0.105, 0.0, 0.3425)), material=matte, name="rear_handle_post")
    body.visual(Box((0.030, 0.035, 0.085)), origin=Origin(xyz=(0.105, 0.0, 0.3425)), material=matte, name="front_handle_post")
    body.visual(Box((0.255, 0.045, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.4025)), material=matte, name="carry_handle")
    body.visual(Box((0.19, 0.028, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.429)), material=dark, name="handle_grip_pad")

    # Long optical tube and fixed collars on the front face.
    body.visual(Cylinder(radius=0.060, length=0.220), origin=Origin(xyz=(0.280, 0.0, 0.185), rpy=(0.0, math.pi / 2, 0.0)), material=lens_black, name="lens_barrel")
    body.visual(tube_mesh("front_lens_hood", outer_radius=0.078, inner_radius=0.055, length=0.070), origin=Origin(xyz=(0.423, 0.0, 0.185), rpy=(0.0, math.pi / 2, 0.0)), material=matte, name="front_hood")
    body.visual(tube_mesh("body_lens_collar", outer_radius=0.070, inner_radius=0.060, length=0.030), origin=Origin(xyz=(0.185, 0.0, 0.185), rpy=(0.0, math.pi / 2, 0.0)), material=dark, name="lens_mount_collar")
    body.visual(Box((0.050, 0.026, 0.008)), origin=Origin(xyz=(0.280, 0.073, 0.185)), material=dark, name="focus_bearing_rail")
    body.visual(Cylinder(radius=0.055, length=0.004), origin=Origin(xyz=(0.456, 0.0, 0.185), rpy=(0.0, math.pi / 2, 0.0)), material=glass, name="front_glass")

    # Rear viewfinder, eyepiece, and the side sockets/hinges that support moving panels.
    body.visual(Box((0.09, 0.060, 0.040)), origin=Origin(xyz=(-0.205, 0.0, 0.225)), material=dark, name="viewfinder_housing")
    body.visual(tube_mesh("rear_eyecup", outer_radius=0.043, inner_radius=0.024, length=0.042), origin=Origin(xyz=(-0.271, 0.0, 0.225), rpy=(0.0, math.pi / 2, 0.0)), material=rubber, name="eyecup")
    body.visual(Box((0.040, 0.012, 0.095)), origin=Origin(xyz=(0.015, 0.086, 0.185)), material=dark, name="lcd_hinge_block")
    body.visual(Cylinder(radius=0.010, length=0.026), origin=Origin(xyz=(0.015, 0.098, 0.223)), material=dark, name="lcd_upper_knuckle")
    body.visual(Cylinder(radius=0.010, length=0.026), origin=Origin(xyz=(0.015, 0.098, 0.147)), material=dark, name="lcd_lower_knuckle")
    body.visual(Box((0.10, 0.008, 0.11)), origin=Origin(xyz=(-0.055, -0.084, 0.180)), material=dark, name="media_door_recess")
    body.visual(Cylinder(radius=0.0065, length=0.030), origin=Origin(xyz=(-0.105, -0.090, 0.215)), material=dark, name="media_upper_knuckle")
    body.visual(Cylinder(radius=0.0065, length=0.030), origin=Origin(xyz=(-0.105, -0.090, 0.145)), material=dark, name="media_lower_knuckle")

    # Fixed sockets under articulated controls keep buttons and rocker visibly mounted.
    body.visual(Box((0.025, 0.020, 0.006)), origin=Origin(xyz=(-0.060, 0.031, 0.268)), material=dark, name="button_socket_0")
    body.visual(Box((0.025, 0.020, 0.006)), origin=Origin(xyz=(-0.025, 0.031, 0.268)), material=dark, name="button_socket_1")
    body.visual(Box((0.025, 0.020, 0.006)), origin=Origin(xyz=(0.010, 0.031, 0.268)), material=dark, name="button_socket_2")
    body.visual(Box((0.072, 0.030, 0.006)), origin=Origin(xyz=(0.060, 0.0, 0.441)), material=dark, name="rocker_socket")

    focus_ring = model.part("focus_ring")
    focus_ring.visual(tube_mesh("focus_ring_shell", outer_radius=0.070, inner_radius=0.062, length=0.050), origin=Origin(rpy=(0.0, math.pi / 2, 0.0)), material=rubber, name="ring_shell")
    model.articulation(
        "body_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.280, 0.0, 0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    lcd_arm = model.part("lcd_arm")
    lcd_arm.visual(Cylinder(radius=0.007, length=0.050), origin=Origin(xyz=(0.0, 0.004, 0.0)), material=dark, name="hinge_sleeve")
    lcd_arm.visual(Box((0.018, 0.105, 0.018)), origin=Origin(xyz=(0.0, 0.055, 0.0)), material=matte, name="swing_arm")
    lcd_arm.visual(Box((0.004, 0.012, 0.045)), origin=Origin(xyz=(0.011, 0.108, 0.030)), material=dark, name="upper_yoke_web")
    lcd_arm.visual(Box((0.004, 0.012, 0.045)), origin=Origin(xyz=(-0.011, 0.108, -0.030)), material=dark, name="lower_yoke_web")
    lcd_arm.visual(Cylinder(radius=0.010, length=0.025), origin=Origin(xyz=(0.0, 0.112, 0.053)), material=dark, name="upper_panel_socket")
    lcd_arm.visual(Cylinder(radius=0.010, length=0.025), origin=Origin(xyz=(0.0, 0.112, -0.053)), material=dark, name="lower_panel_socket")
    model.articulation(
        "body_to_lcd_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd_arm,
        origin=Origin(xyz=(0.015, 0.098, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.70, upper=0.25),
    )

    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(Cylinder(radius=0.008, length=0.075), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark, name="pivot_knuckle")
    lcd_panel.visual(Box((0.140, 0.012, 0.090)), origin=Origin(xyz=(0.078, 0.0, 0.0)), material=matte, name="screen_frame")
    lcd_panel.visual(Box((0.112, 0.004, 0.064)), origin=Origin(xyz=(0.084, 0.008, 0.0)), material=screen_glass, name="lcd_glass")
    model.articulation(
        "lcd_arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=lcd_arm,
        child=lcd_panel,
        origin=Origin(xyz=(0.0, 0.112, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=-2.7, upper=2.7),
    )

    media_door = model.part("media_door")
    media_door.visual(Cylinder(radius=0.006, length=0.040), origin=Origin(xyz=(0.0, -0.003, 0.0)), material=dark, name="center_knuckle")
    media_door.visual(Box((0.080, 0.008, 0.100)), origin=Origin(xyz=(0.046, -0.004, 0.0)), material=matte, name="door_panel")
    media_door.visual(Box((0.050, 0.002, 0.012)), origin=Origin(xyz=(0.058, -0.009, -0.025)), material=label, name="media_label")
    model.articulation(
        "body_to_media_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=media_door,
        origin=Origin(xyz=(-0.105, -0.088, 0.180)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.2, lower=0.0, upper=1.45),
    )

    for i, x in enumerate((-0.060, -0.025, 0.010)):
        button = model.part(f"button_{i}")
        button.visual(Cylinder(radius=0.0085, length=0.006), origin=Origin(), material=dark, name="button_cap")
        if i == 0:
            button.visual(triangle_arrow_mesh("record_arrow", 0.009, 0.007, 0.001), origin=Origin(xyz=(0.0, 0.0, 0.0035)), material=label, name="button_mark")
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, 0.031, 0.274)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=0.04, lower=0.0, upper=0.004),
        )

    zoom_rocker = model.part("zoom_rocker")
    zoom_rocker.visual(Box((0.060, 0.022, 0.010)), origin=Origin(), material=dark, name="rocker_bar")
    zoom_rocker.visual(Box((0.018, 0.024, 0.002)), origin=Origin(xyz=(-0.019, 0.0, 0.006)), material=label, name="wide_mark")
    zoom_rocker.visual(Box((0.018, 0.024, 0.002)), origin=Origin(xyz=(0.019, 0.0, 0.006)), material=label, name="tele_mark")
    model.articulation(
        "body_to_zoom_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=zoom_rocker,
        origin=Origin(xyz=(0.060, 0.0, 0.449)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.0, lower=-0.22, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lcd_arm = object_model.get_part("lcd_arm")
    lcd_panel = object_model.get_part("lcd_panel")
    focus_ring = object_model.get_part("focus_ring")
    media_door = object_model.get_part("media_door")

    ctx.allow_overlap(
        body,
        focus_ring,
        elem_a="focus_bearing_rail",
        elem_b="ring_shell",
        reason="A small fixed bearing rail is intentionally seated in the rotating focus-ring groove to keep the collar captured on the lens.",
    )
    ctx.expect_contact(focus_ring, body, elem_a="ring_shell", elem_b="focus_bearing_rail", contact_tol=0.002, name="rotating focus ring rides on bearing rail")
    ctx.expect_overlap(focus_ring, body, axes="x", elem_a="ring_shell", elem_b="lens_barrel", min_overlap=0.045, name="focus ring surrounds lens length")
    ctx.expect_contact(lcd_arm, body, elem_a="hinge_sleeve", elem_b="lcd_upper_knuckle", contact_tol=0.012, name="lcd arm captured by side hinge")
    ctx.expect_contact(lcd_panel, lcd_arm, elem_a="pivot_knuckle", elem_b="upper_panel_socket", contact_tol=0.004, name="lcd panel carried by arm socket")
    ctx.expect_contact(media_door, body, elem_a="center_knuckle", elem_b="media_upper_knuckle", contact_tol=0.030, name="media door knuckle is on hinge line")

    with ctx.pose({"body_to_lcd_arm": -1.2, "lcd_arm_to_panel": 1.1}):
        ctx.expect_origin_distance(lcd_panel, body, axes="xy", min_dist=0.12, name="lcd assembly swings away from body")

    with ctx.pose({"body_to_media_door": 1.0}):
        ctx.expect_origin_distance(media_door, body, axes="xy", min_dist=0.09, name="media door opens outward")

    return ctx.report()


object_model = build_object_model()
