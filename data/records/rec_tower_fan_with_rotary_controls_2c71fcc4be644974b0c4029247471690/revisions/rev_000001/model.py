from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _oval_extrusion(x_radius: float, y_radius: float, height: float, z: float = 0.0):
    return cq.Workplane("XY").ellipse(x_radius, y_radius).extrude(height).translate((0.0, 0.0, z))


def _tower_body_shell():
    """Hollow oval tower shell with a real cut-out behind the front grille."""

    outer = _oval_extrusion(0.080, 0.064, 0.770, z=0.055)
    inner = _oval_extrusion(0.064, 0.049, 0.790, z=0.045)
    shell = outer.cut(inner)

    # Tall front window: the slotted grille visual covers this opening, while the
    # side rails and top/bottom rims remain one continuous molded shell.
    front_window = cq.Workplane("XY").box(0.108, 0.095, 0.620).translate((0.0, -0.061, 0.420))
    return shell.cut(front_window)


def _top_cap_with_handle():
    cap = _oval_extrusion(0.086, 0.066, 0.045, z=0.825)

    # A through oblong slot at the rear of the cap forms the carry-handle
    # opening.  Material behind and around the slot reads as the supported bridge.
    handle_slot = (
        cq.Workplane("XY")
        .center(0.0, 0.034)
        .slot2D(0.074, 0.023, 0.0)
        .extrude(0.080)
        .translate((0.0, 0.0, 0.808))
    )
    return cap.cut(handle_slot)


def _control_deck():
    return (
        cq.Workplane("XY")
        .center(0.0, -0.022)
        .slot2D(0.118, 0.052, 0.0)
        .extrude(0.006)
        .translate((0.0, 0.0, 0.870))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedroom_tower_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    satin_gray = model.material("satin_gray_plastic", rgba=(0.38, 0.40, 0.42, 1.0))
    dark = model.material("charcoal_grille", rgba=(0.035, 0.038, 0.042, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    label_white = model.material("printed_white", rgba=(0.95, 0.95, 0.90, 1.0))
    translucent_blade = model.material("smoked_blower_plastic", rgba=(0.14, 0.17, 0.19, 0.72))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_oval_extrusion(0.245, 0.165, 0.050), "low_oval_base"),
        name="base_shell",
        material=warm_white,
    )
    base.visual(
        Cylinder(radius=0.085, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        name="turntable_bearing",
        material=satin_gray,
    )
    base.visual(
        mesh_from_cadquery(_oval_extrusion(0.215, 0.140, 0.006), "base_shadow_pad"),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        name="rubber_foot_pad",
        material=rubber,
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_oval_extrusion(0.090, 0.068, 0.056), "lower_rotating_collar"),
        name="lower_collar",
        material=warm_white,
    )
    body.visual(
        mesh_from_cadquery(_tower_body_shell(), "hollow_body_shell", tolerance=0.0008),
        name="body_shell",
        material=warm_white,
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.122, 0.660),
                0.006,
                slot_size=(0.088, 0.006),
                pitch=(0.014, 0.095),
                frame=0.010,
                corner_radius=0.010,
                slot_angle_deg=89.0,
                stagger=False,
            ),
            "front_slotted_grille",
        ),
        origin=Origin(xyz=(0.0, -0.0580, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="front_grille",
        material=dark,
    )
    body.visual(
        mesh_from_cadquery(_top_cap_with_handle(), "top_cap_handle", tolerance=0.0008),
        name="top_cap",
        material=warm_white,
    )
    body.visual(
        mesh_from_cadquery(_control_deck(), "separate_control_deck"),
        name="control_deck",
        material=satin_gray,
    )
    body.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(-0.035, -0.022, 0.879)),
        name="timer_shaft_collar",
        material=dark,
    )
    body.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.036, -0.022, 0.879)),
        name="speed_shaft_collar",
        material=dark,
    )
    body.visual(
        Box((0.004, 0.014, 0.0015)),
        origin=Origin(xyz=(-0.035, -0.052, 0.8762)),
        name="timer_tick",
        material=label_white,
    )
    body.visual(
        Box((0.004, 0.014, 0.0015)),
        origin=Origin(xyz=(0.036, -0.049, 0.8762)),
        name="speed_tick",
        material=label_white,
    )
    body.visual(
        Box((0.018, 0.056, 0.006)),
        origin=Origin(xyz=(0.0, 0.026, 0.067)),
        name="lower_blower_rib",
        material=satin_gray,
    )
    body.visual(
        Box((0.018, 0.056, 0.006)),
        origin=Origin(xyz=(0.0, 0.026, 0.773)),
        name="upper_blower_rib",
        material=satin_gray,
    )
    body.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        name="lower_blower_bearing",
        material=satin_gray,
    )
    body.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.773)),
        name="upper_blower_bearing",
        material=satin_gray,
    )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.039,
                0.019,
                0.640,
                24,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "vertical_blower_wheel",
        ),
        name="blower_wheel",
        material=translucent_blade,
    )
    blower.visual(
        Cylinder(radius=0.0045, length=0.700),
        name="blower_shaft",
        material=satin_gray,
    )
    blower.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
        name="lower_shaft_hub",
        material=satin_gray,
    )
    blower.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        name="upper_shaft_hub",
        material=satin_gray,
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.020,
                body_style="skirted",
                top_diameter=0.030,
                skirt=KnobSkirt(0.042, 0.004, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=20, depth=0.0009),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                bore=KnobBore(style="round", diameter=0.006),
                center=False,
            ),
            "timer_dial_cap",
        ),
        name="timer_cap",
        material=warm_white,
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.031,
                0.018,
                body_style="tapered",
                base_diameter=0.033,
                top_diameter=0.026,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=30.0),
                bore=KnobBore(style="round", diameter=0.005),
                center=False,
            ),
            "speed_dial_cap",
        ),
        name="speed_cap",
        material=warm_white,
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(-0.035, -0.022, 0.882)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_dial,
        origin=Origin(xyz=(0.036, -0.022, 0.882)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower")
    timer = object_model.get_part("timer_dial")
    speed = object_model.get_part("speed_dial")
    oscillation = object_model.get_articulation("base_to_body")
    blower_spin = object_model.get_articulation("body_to_blower")
    timer_spin = object_model.get_articulation("body_to_timer_dial")
    speed_spin = object_model.get_articulation("body_to_speed_dial")

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="turntable_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotating body is seated on the base bearing",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        inner_elem="blower_wheel",
        outer_elem="body_shell",
        margin=0.004,
        name="blower wheel fits inside tower shell envelope",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        elem_a="blower_wheel",
        elem_b="front_grille",
        min_overlap=0.45,
        name="blower spans the louvered outlet height",
    )
    ctx.expect_gap(
        timer,
        body,
        axis="z",
        positive_elem="timer_cap",
        negative_elem="timer_shaft_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="timer dial sits on its shaft collar",
    )
    ctx.expect_gap(
        speed,
        body,
        axis="z",
        positive_elem="speed_cap",
        negative_elem="speed_shaft_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="speed dial sits on its shaft collar",
    )

    ctx.check(
        "body oscillates about a limited vertical base joint",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and tuple(oscillation.axis) == (0.0, 0.0, 1.0)
        and oscillation.motion_limits.lower < -0.8
        and oscillation.motion_limits.upper > 0.8,
    )
    ctx.check(
        "blower and both dials use continuous rotary joints",
        blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and timer_spin.articulation_type == ArticulationType.CONTINUOUS
        and speed_spin.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "top cap includes separate deck and handle elements",
        body.get_visual("top_cap") is not None
        and body.get_visual("control_deck") is not None
        and body.get_visual("front_grille") is not None,
    )

    rest_aabb = ctx.part_world_aabb(body)
    with ctx.pose({oscillation: 0.75}):
        yawed_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "oscillation pose changes the body footprint",
        rest_aabb is not None
        and yawed_aabb is not None
        and abs((yawed_aabb[1][0] - yawed_aabb[0][0]) - (rest_aabb[1][0] - rest_aabb[0][0])) > 0.010,
    )

    return ctx.report()


object_model = build_object_model()
