from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _box(dx: float, dy: float, dz: float, x: float, y: float, z: float) -> cq.Workplane:
    """CadQuery box with center-style placement in model coordinates."""
    return cq.Workplane("XY").box(dx, dy, dz).translate((x, y, z))


def _ring_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A centered hollow cylinder, authored along local Z for later placement."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length, both=True)
    )


def _housing_mesh() -> cq.Workplane:
    """Curved casino cabinet body with an arched top and open reel window."""
    width = 0.78

    # Main body and a forward belly panel give the front a convex casino-machine
    # stance while still leaving a through-opening for the reel window.
    body = _box(width, 0.42, 1.10, 0.0, 0.0, 0.58)
    body = body.union(_box(0.72, 0.075, 0.74, 0.0, -0.228, 0.55))

    # Rounded top box: an arched Y/Z profile extruded across the machine width.
    top = (
        cq.Workplane("YZ")
        .moveTo(-0.225, 1.08)
        .lineTo(0.225, 1.08)
        .lineTo(0.225, 1.25)
        .threePointArc((0.0, 1.43), (-0.225, 1.25))
        .close()
        .extrude(width, both=True)
    )
    body = body.union(top)

    # A lower plinth and sloped control brow make the front read as one curved
    # shell rather than a plain rectangular box.
    body = body.union(_box(0.82, 0.46, 0.10, 0.0, 0.0, 0.05))
    body = body.union(_box(0.70, 0.080, 0.16, 0.0, -0.270, 0.61))

    # Through aperture for the reel window.  This keeps the reels physically in
    # a real opening instead of intersecting a solid cabinet face.
    body = body.cut(_box(0.62, 0.95, 0.34, 0.0, -0.04, 0.91))
    # Bore clear push-button guide holes through the protruding control brow.
    for x in (-0.115, 0.115):
        body = body.cut(
            cq.Workplane("XZ")
            .circle(0.042)
            .extrude(0.42, both=True)
            .translate((x, -0.255, 0.590))
        )

    # Slightly soften the manufactured cabinet silhouette.  Fillets are kept
    # modest so the window cut and plinth remain robust.
    try:
        body = body.edges("|Z").fillet(0.018)
        body = body.edges(">Z").fillet(0.030)
    except Exception:
        pass
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_front_casino_machine")

    red = model.material("deep_red_lacquer", rgba=(0.55, 0.02, 0.035, 1.0))
    black = model.material("gloss_black", rgba=(0.005, 0.005, 0.008, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.66, 1.0))
    warm = model.material("warm_lamp", rgba=(1.0, 0.76, 0.12, 1.0))
    green = model.material("green_lamp", rgba=(0.0, 0.95, 0.22, 1.0))
    red_lamp = model.material("red_lamp", rgba=(1.0, 0.08, 0.03, 1.0))
    reel_white = model.material("reel_ivory", rgba=(0.94, 0.88, 0.72, 1.0))
    reel_red = model.material("symbol_red", rgba=(0.90, 0.02, 0.02, 1.0))
    reel_blue = model.material("symbol_blue", rgba=(0.05, 0.20, 0.95, 1.0))
    reel_green = model.material("symbol_green", rgba=(0.02, 0.65, 0.10, 1.0))
    steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    brass = model.material("brass_lock", rgba=(0.86, 0.61, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_mesh(), "curved_cabinet", tolerance=0.002),
        material=red,
        name="curved_cabinet",
    )

    # Central reel-window surround, glass lip, and marquee lighting.
    body.visual(Box((0.70, 0.030, 0.050)), origin=Origin(xyz=(0.0, -0.310, 1.105)), material=black, name="window_bezel_top")
    body.visual(Box((0.70, 0.030, 0.050)), origin=Origin(xyz=(0.0, -0.310, 0.715)), material=black, name="window_bezel_bottom")
    body.visual(Box((0.050, 0.030, 0.360)), origin=Origin(xyz=(-0.345, -0.310, 0.910)), material=black, name="window_bezel_side_0")
    body.visual(Box((0.050, 0.030, 0.360)), origin=Origin(xyz=(0.345, -0.310, 0.910)), material=black, name="window_bezel_side_1")
    body.visual(Box((0.52, 0.014, 0.095)), origin=Origin(xyz=(0.0, -0.221, 1.245)), material=warm, name="rounded_top_sign")
    for idx, x in enumerate((-0.18, 0.0, 0.18)):
        body.visual(
            Cylinder(radius=0.034, length=0.010),
            origin=Origin(xyz=(x, -0.230, 1.245), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=red_lamp if idx != 1 else green,
            name=f"marquee_lamp_{idx}",
        )

    # Separate cylindrical guides for the two push buttons.  They are hollow
    # mesh tubes so the prismatic button caps can move without a collision
    # waiver.
    guide_mesh = _ring_tube(0.055, 0.038, 0.070)
    for idx, x in enumerate((-0.115, 0.115)):
        body.visual(
            mesh_from_cadquery(guide_mesh, f"button_guide_{idx}", tolerance=0.001),
            origin=Origin(xyz=(x, -0.285, 0.590), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=chrome,
            name=f"button_guide_{idx}",
        )
        body.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(x, -0.328, 0.655), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=warm,
            name=f"button_pilot_{idx}",
        )
        for pad_idx, z_sign in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.022, 0.050, 0.007)),
                origin=Origin(xyz=(x, -0.285, 0.590 + z_sign * 0.0355)),
                material=black,
                name=f"button_bushing_{idx}_{pad_idx}",
            )

    # Narrow reel separators double as visible bearing supports; each rotating
    # shaft touches these faces without interpenetrating them.
    for idx, x in enumerate((-0.2775, -0.0925, 0.0925, 0.2775)):
        body.visual(
            Box((0.010, 0.055, 0.390)),
            origin=Origin(xyz=(x, -0.180, 0.910)),
            material=black,
            name=f"reel_support_{idx}",
        )

    # Lower cashbox trim frame, mounted to the curved body behind the hinged
    # door.
    body.visual(Box((0.52, 0.022, 0.035)), origin=Origin(xyz=(-0.010, -0.281, 0.5175)), material=black, name="cashbox_frame_top")
    body.visual(Box((0.52, 0.022, 0.035)), origin=Origin(xyz=(-0.010, -0.281, 0.1625)), material=black, name="cashbox_frame_bottom")
    body.visual(Box((0.035, 0.022, 0.370)), origin=Origin(xyz=(-0.280, -0.281, 0.340)), material=black, name="cashbox_frame_hinge")
    body.visual(Box((0.035, 0.022, 0.370)), origin=Origin(xyz=(0.260, -0.281, 0.340)), material=black, name="cashbox_frame_latch")

    # Rotating reels on horizontal shafts behind the front window.
    reel_centers = (-0.185, 0.0, 0.185)
    symbol_mats = (reel_red, reel_blue, reel_green)
    for idx, x in enumerate(reel_centers):
        reel = model.part(f"reel_{idx}")
        reel.visual(
            Cylinder(radius=0.106, length=0.148),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=reel_white,
            name="reel_drum",
        )
        reel.visual(
            Cylinder(radius=0.017, length=0.175),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=steel,
            name="shaft",
        )
        for sym, (y, z) in enumerate(((-0.098, -0.040), (-0.106, 0.0), (-0.098, 0.040))):
            reel.visual(
                Box((0.112, 0.006, 0.032)),
                origin=Origin(xyz=(0.0, y, z)),
                material=symbol_mats[(idx + sym) % len(symbol_mats)],
                name=f"symbol_{sym}",
            )
        model.articulation(
            f"body_to_reel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=reel,
            origin=Origin(xyz=(x, -0.180, 0.910)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        )

    # Two illuminated front push buttons that translate inward inside the guide
    # barrels.
    for idx, (x, mat) in enumerate(((-0.115, green), (0.115, red_lamp))):
        button = model.part(f"button_{idx}")
        button.visual(
            Cylinder(radius=0.032, length=0.046),
            origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=mat,
            name="lit_cap",
        )
        button.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(0.0, -0.048, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=warm,
            name="front_lens",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.285, 0.590)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=0.30, lower=0.0, upper=0.026),
        )

    # Lower cashbox door on a vertical side hinge.
    door_w = 0.460
    door_h = 0.320
    cashbox_door = model.part("cashbox_door")
    cashbox_door.visual(
        Box((door_w, 0.024, door_h)),
        origin=Origin(xyz=(door_w / 2, -0.012, door_h / 2)),
        material=steel,
        name="door_panel",
    )
    cashbox_door.visual(
        Cylinder(radius=0.016, length=door_h + 0.035),
        origin=Origin(xyz=(0.0, -0.020, door_h / 2)),
        material=chrome,
        name="hinge_barrel",
    )
    cashbox_door.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(door_w * 0.66, -0.029, door_h * 0.55), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="round_lock",
    )
    model.articulation(
        "body_to_cashbox_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cashbox_door,
        origin=Origin(xyz=(-0.255, -0.292, 0.180)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cashbox_door = object_model.get_part("cashbox_door")
    door_joint = object_model.get_articulation("body_to_cashbox_door")

    for idx in range(3):
        reel = object_model.get_part(f"reel_{idx}")
        reel_joint = object_model.get_articulation(f"body_to_reel_{idx}")
        ctx.check(
            f"reel_{idx} uses a continuous horizontal shaft joint",
            reel_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(reel_joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={reel_joint.articulation_type}, axis={reel_joint.axis}",
        )
        ctx.expect_contact(
            reel,
            body,
            elem_a="shaft",
            elem_b=f"reel_support_{idx}",
            name=f"reel_{idx} shaft seats in one bearing support",
        )
        ctx.expect_contact(
            reel,
            body,
            elem_a="shaft",
            elem_b=f"reel_support_{idx + 1}",
            name=f"reel_{idx} shaft seats in the opposite bearing support",
        )

    for idx in range(2):
        button = object_model.get_part(f"button_{idx}")
        button_joint = object_model.get_articulation(f"body_to_button_{idx}")
        ctx.check(
            f"button_{idx} is an inward prismatic control",
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(button_joint.axis) == (0.0, 1.0, 0.0)
            and button_joint.motion_limits is not None
            and button_joint.motion_limits.upper is not None
            and button_joint.motion_limits.upper > 0.02,
            details=f"type={button_joint.articulation_type}, axis={button_joint.axis}, limits={button_joint.motion_limits}",
        )
        ctx.expect_contact(
            button,
            body,
            elem_a="lit_cap",
            elem_b=f"button_bushing_{idx}_0",
            name=f"button_{idx} rides in its lower guide bushing",
        )
        ctx.expect_contact(
            button,
            body,
            elem_a="lit_cap",
            elem_b=f"button_bushing_{idx}_1",
            name=f"button_{idx} rides in its upper guide bushing",
        )
        rest_position = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed_position = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} translates inward when pressed",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[1] > rest_position[1] + 0.020,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    ctx.check(
        "cashbox door has a vertical side hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_joint.axis) == (0.0, 0.0, -1.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper > 1.0,
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}, limits={door_joint.motion_limits}",
    )
    ctx.expect_contact(
        cashbox_door,
        body,
        elem_a="door_panel",
        elem_b="cashbox_frame_top",
        name="closed cashbox door seats against the upper frame",
    )
    rest_aabb = ctx.part_world_aabb(cashbox_door)
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        opened_aabb = ctx.part_world_aabb(cashbox_door)
    ctx.check(
        "cashbox door swings outward from the front",
        rest_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < rest_aabb[0][1] - 0.20,
        details=f"rest_aabb={rest_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
