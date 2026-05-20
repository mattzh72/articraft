from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.094
BODY_D = 0.050
BODY_H = 0.186

PANEL_W = 0.074
PANEL_H = 0.148
PANEL_D = 0.0042
PANEL_FRONT_Y = BODY_D / 2.0 - 0.0008
PANEL_CENTER_Y = PANEL_FRONT_Y - PANEL_D / 2.0

DISPLAY_W = 0.056
DISPLAY_H = 0.033
DISPLAY_Z = 0.054

KEY_CAP_W = 0.0120
KEY_CAP_H = 0.0072
KEY_STEM_W = 0.0072
KEY_STEM_H = 0.0052
KEY_TOTAL_D = 0.0068
KEY_FRONT_PROUD = 0.0020
KEY_ROW_Z = 0.018
KEY_XS = (-0.026, -0.013, 0.0, 0.013, 0.026)
KEY_TRAVEL = 0.0015

DIAL_RADIUS = 0.029
DIAL_Z = -0.020
DIAL_DEPTH = 0.020

JACK_XS = (-0.028, -0.0095, 0.0095, 0.028)
JACK_Z = -0.064
JACK_RADIUS = 0.0058

KICKSTAND_T = 0.003
KICKSTAND_W = 0.058
KICKSTAND_L = 0.120
KICKSTAND_HINGE_Z = -0.064


def _boot_shape() -> cq.Workplane:
    boot = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    boot = boot.edges("|Z").fillet(0.010)

    side_guard_w = 0.007
    side_guard_y = 0.038
    side_guard_z = 0.122
    for sign in (-1.0, 1.0):
        side_guard = (
            cq.Workplane("XY")
            .box(side_guard_w, side_guard_y, side_guard_z)
            .translate((sign * (BODY_W / 2.0 - side_guard_w / 2.0 + 0.0012), 0.0, 0.0))
        )
        boot = boot.union(side_guard)

    for z_sign in (-1.0, 1.0):
        bumper = (
            cq.Workplane("XY")
            .box(0.060, 0.038, 0.018)
            .translate((0.0, 0.0, z_sign * (BODY_H / 2.0 - 0.012)))
        )
        boot = boot.union(bumper)

    front_recess = cq.Workplane("XY").box(
        PANEL_W + 0.008,
        0.0048,
        PANEL_H + 0.004,
    ).translate((0.0, BODY_D / 2.0 - 0.0048 / 2.0, 0.0))
    boot = boot.cut(front_recess)

    button_pocket = cq.Workplane("XY").box(
        0.070,
        0.0036,
        0.018,
    ).translate((0.0, BODY_D / 2.0 - 0.0048 - 0.0036 / 2.0 + 0.0002, KEY_ROW_Z))
    boot = boot.cut(button_pocket)

    dial_pocket = (
        cq.Workplane("XZ")
        .circle(0.032)
        .extrude(0.0032)
        .translate((0.0, BODY_D / 2.0 - 0.0048 - 0.0032 + 0.0002, DIAL_Z))
    )
    boot = boot.cut(dial_pocket)

    back_recess = cq.Workplane("XY").box(
        KICKSTAND_W + 0.004,
        0.0032,
        KICKSTAND_L + 0.006,
    ).translate((0.0, -BODY_D / 2.0 + 0.0032 / 2.0, KICKSTAND_HINGE_Z + KICKSTAND_L / 2.0))
    boot = boot.cut(back_recess)

    rear_ledge = cq.Workplane("XY").box(
        0.064,
        0.004,
        0.010,
    ).translate((0.0, -BODY_D / 2.0 + 0.002, KICKSTAND_HINGE_Z + 0.006))
    boot = boot.union(rear_ledge)

    return boot


def _panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(PANEL_W, PANEL_D, PANEL_H)
    panel = panel.edges("|Z").fillet(0.0014)

    display_recess = cq.Workplane("XY").box(
        DISPLAY_W + 0.008,
        0.0013,
        DISPLAY_H + 0.010,
    ).translate((0.0, PANEL_D / 2.0 - 0.0013 / 2.0 + 0.00005, DISPLAY_Z))
    panel = panel.cut(display_recess)

    for x in KEY_XS:
        key_slot = cq.Workplane("XY").box(KEY_STEM_W + 0.0010, PANEL_D + 0.002, KEY_STEM_H + 0.0008).translate(
            (x, 0.0, KEY_ROW_Z)
        )
        panel = panel.cut(key_slot)

    for x in JACK_XS:
        jack_hole = cq.Workplane("XZ").circle(JACK_RADIUS).extrude(PANEL_D + 0.002).translate(
            (x, -(PANEL_D + 0.002) / 2.0, JACK_Z)
        )
        panel = panel.cut(jack_hole)

    return panel


def _dial_shape() -> cq.Workplane:
    skirt = cq.Workplane("XZ").circle(0.031).extrude(0.007)
    hub = cq.Workplane("XZ").circle(0.025).extrude(0.020)
    dial = skirt.union(hub)

    pointer = cq.Workplane("XY").box(0.0024, 0.0010, 0.013).translate((0.0, -0.0005, 0.014))
    dial = dial.union(pointer)

    return dial


def _soft_key_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(KEY_CAP_W, 0.0028, KEY_CAP_H).translate((0.0, 0.0014, 0.0))
    stem = cq.Workplane("XY").box(KEY_STEM_W, 0.0040, KEY_STEM_H).translate((0.0, -0.0020, 0.0))
    return cap.union(stem)


def _kickstand_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(KICKSTAND_W, KICKSTAND_T, KICKSTAND_L).translate(
        (0.0, -KICKSTAND_T / 2.0, KICKSTAND_L / 2.0)
    )
    frame = frame.cut(
        cq.Workplane("XY").box(0.040, KICKSTAND_T + 0.001, 0.082).translate(
            (0.0, -KICKSTAND_T / 2.0, KICKSTAND_L / 2.0 + 0.004)
        )
    )
    hinge_bar = cq.Workplane("XY").box(0.062, 0.005, 0.010).translate((0.0, -0.0025, 0.005))
    top_pad = cq.Workplane("XY").box(0.030, 0.0035, 0.012).translate((0.0, -0.00175, KICKSTAND_L - 0.006))
    return frame.union(hinge_bar).union(top_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_digital_multimeter")

    boot_yellow = model.material("boot_yellow", color=(0.86, 0.76, 0.19))
    panel_gray = model.material("panel_gray", color=(0.19, 0.21, 0.23))
    dial_black = model.material("dial_black", color=(0.08, 0.08, 0.09))
    key_gray = model.material("key_gray", color=(0.44, 0.48, 0.52))
    key_dark = model.material("key_dark", color=(0.16, 0.18, 0.20))
    glass_green = model.material("glass_green", rgba=(0.30, 0.42, 0.34, 0.85))
    jack_black = model.material("jack_black", color=(0.07, 0.07, 0.08))
    jack_red = model.material("jack_red", color=(0.63, 0.11, 0.10))
    jack_orange = model.material("jack_orange", color=(0.77, 0.32, 0.06))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_boot_shape(), "multimeter_boot"),
        material=boot_yellow,
        name="boot_shell",
    )
    body.visual(
        mesh_from_cadquery(_panel_shape(), "multimeter_panel"),
        origin=Origin(xyz=(0.0, PANEL_CENTER_Y, 0.0)),
        material=panel_gray,
        name="front_panel",
    )
    body.visual(
        Box((DISPLAY_W, 0.0016, DISPLAY_H)),
        origin=Origin(
            xyz=(0.0, PANEL_FRONT_Y - 0.0009, DISPLAY_Z),
        ),
        material=glass_green,
        name="display_window",
    )

    jack_materials = (jack_black, jack_black, jack_red, jack_orange)
    for i, (x, mat) in enumerate(zip(JACK_XS, jack_materials)):
        body.visual(
            Cylinder(radius=JACK_RADIUS + 0.0015, length=0.0035),
            origin=Origin(
                xyz=(x, PANEL_FRONT_Y - 0.00175, JACK_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=mat,
            name=f"jack_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_shape(), "range_dial"),
        origin=Origin(xyz=(0.0, DIAL_DEPTH, 0.0)),
        material=dial_black,
        name="dial_knob",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, PANEL_FRONT_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    soft_key_mesh = mesh_from_cadquery(_soft_key_shape(), "soft_key")
    for i, x in enumerate(KEY_XS):
        key = model.part(f"key_{i}")
        key.visual(
            soft_key_mesh,
            material=key_gray if i != 2 else key_dark,
            name="key_cap",
        )
        model.articulation(
            f"body_to_key_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(x, PANEL_FRONT_Y, KEY_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        mesh_from_cadquery(_kickstand_shape(), "kickstand"),
        material=panel_gray,
        name="stand_frame",
    )
    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.0032, KICKSTAND_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    kickstand = object_model.get_part("kickstand")
    dial_joint = object_model.get_articulation("body_to_dial")
    kickstand_joint = object_model.get_articulation("body_to_kickstand")
    keys = [object_model.get_part(f"key_{i}") for i in range(5)]
    key_joints = [object_model.get_articulation(f"body_to_key_{i}") for i in range(5)]

    for i, key in enumerate(keys):
        ctx.expect_contact(
            key,
            body,
            contact_tol=0.0002,
            elem_a="key_cap",
            name=f"key_{i} seats on the front panel",
        )

    display_box = ctx.part_element_world_aabb(body, elem="display_window")
    jack_boxes = [ctx.part_element_world_aabb(body, elem=f"jack_{i}") for i in range(4)]
    dial_box = ctx.part_world_aabb(dial)
    key_boxes = [ctx.part_world_aabb(key) for key in keys]

    if display_box is not None and dial_box is not None and all(box is not None for box in key_boxes):
        display_bottom = display_box[0][2]
        dial_top = dial_box[1][2]
        key_top = max(box[1][2] for box in key_boxes if box is not None)
        key_bottom = min(box[0][2] for box in key_boxes if box is not None)

        ctx.check(
            "soft keys sit below the display window",
            display_bottom > key_top + 0.004,
            details=f"display_bottom={display_bottom:.4f}, key_top={key_top:.4f}",
        )
        ctx.check(
            "range dial stays below the key row",
            key_bottom > dial_top + 0.003,
            details=f"key_bottom={key_bottom:.4f}, dial_top={dial_top:.4f}",
        )

    if all(box is not None for box in jack_boxes) and all(box is not None for box in key_boxes):
        jack_top = max(box[1][2] for box in jack_boxes if box is not None)
        key_bottom = min(box[0][2] for box in key_boxes if box is not None)
        ctx.check(
            "input jacks stay low and clearly separated from the controls",
            key_bottom > jack_top + 0.014,
            details=f"key_bottom={key_bottom:.4f}, jack_top={jack_top:.4f}",
        )

    key_positions = [ctx.part_world_position(key) for key in keys]
    if all(pos is not None for pos in key_positions):
        z_spread = max(pos[2] for pos in key_positions if pos is not None) - min(
            pos[2] for pos in key_positions if pos is not None
        )
        spacings = [
            key_positions[i + 1][0] - key_positions[i][0]
            for i in range(len(key_positions) - 1)
            if key_positions[i] is not None and key_positions[i + 1] is not None
        ]
        uniform = max(spacings) - min(spacings) if spacings else 0.0
        ctx.check(
            "soft keys form a neat row",
            z_spread < 0.001 and uniform < 0.002,
            details=f"z_spread={z_spread:.4f}, spacing_range={uniform:.4f}, spacings={spacings}",
        )

    center_key = keys[2]
    center_joint = key_joints[2]
    center_rest = ctx.part_world_position(center_key)
    neighbor_rest = ctx.part_world_position(keys[1])
    upper = center_joint.motion_limits.upper if center_joint.motion_limits is not None else None
    if center_rest is not None and neighbor_rest is not None and upper is not None:
        with ctx.pose({center_joint: upper}):
            center_pressed = ctx.part_world_position(center_key)
            neighbor_pressed = ctx.part_world_position(keys[1])
        ctx.check(
            "center soft key presses inward",
            center_pressed is not None and center_pressed[1] < center_rest[1] - 0.001,
            details=f"rest={center_rest}, pressed={center_pressed}",
        )
        ctx.check(
            "soft keys articulate independently",
            neighbor_pressed is not None and abs(neighbor_pressed[1] - neighbor_rest[1]) < 1e-6,
            details=f"neighbor_rest={neighbor_rest}, neighbor_pressed={neighbor_pressed}",
        )

    dial_rest = ctx.part_world_position(dial)
    if dial_rest is not None:
        with ctx.pose({dial_joint: math.pi / 2.0}):
            dial_rot = ctx.part_world_position(dial)
        ctx.check(
            "range dial rotates in place",
            dial_rot is not None
            and abs(dial_rot[0] - dial_rest[0]) < 1e-6
            and abs(dial_rot[1] - dial_rest[1]) < 1e-6
            and abs(dial_rot[2] - dial_rest[2]) < 1e-6,
            details=f"rest={dial_rest}, rotated={dial_rot}",
        )

    stand_rest = ctx.part_world_aabb(kickstand)
    stand_upper = kickstand_joint.motion_limits.upper if kickstand_joint.motion_limits is not None else None
    if stand_rest is not None and stand_upper is not None:
        with ctx.pose({kickstand_joint: stand_upper}):
            stand_open = ctx.part_world_aabb(kickstand)
        ctx.check(
            "kickstand swings out behind the meter",
            stand_open is not None and stand_open[0][1] < stand_rest[0][1] - 0.030,
            details=f"rest={stand_rest}, open={stand_open}",
        )

    return ctx.report()


object_model = build_object_model()
