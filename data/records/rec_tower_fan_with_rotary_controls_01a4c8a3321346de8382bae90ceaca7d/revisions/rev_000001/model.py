from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
    rounded_rect_profile,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    satin_white = model.material("satin_white", color=(0.86, 0.88, 0.86, 1.0))
    warm_gray = model.material("warm_gray", color=(0.50, 0.52, 0.52, 1.0))
    dark_plastic = model.material("dark_plastic", color=(0.025, 0.027, 0.030, 1.0))
    soft_black = model.material("soft_black", color=(0.07, 0.075, 0.08, 1.0))
    charcoal = model.material("charcoal", color=(0.13, 0.14, 0.15, 1.0))
    tick_white = model.material("tick_white", color=(0.95, 0.95, 0.90, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.205, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_plastic,
        name="rubber_foot",
    )
    base.visual(
        Cylinder(radius=0.185, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=warm_gray,
        name="weighted_disk",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=satin_white,
        name="oscillation_collar",
    )

    body = model.part("body")

    lower_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.225, 0.175, 0.040, corner_segments=10),
            0.070,
            center=True,
        ),
        "lower_body_cap",
    )
    top_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.235, 0.190, 0.045, corner_segments=10),
            0.082,
            center=True,
        ),
        "top_cap",
    )
    body.visual(
        lower_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=satin_white,
        name="lower_cap",
    )
    body.visual(
        Box((0.030, 0.165, 0.820)),
        origin=Origin(xyz=(-0.105, 0.000, 0.450)),
        material=satin_white,
        name="side_shell_0",
    )
    body.visual(
        Box((0.030, 0.165, 0.820)),
        origin=Origin(xyz=(0.105, 0.000, 0.450)),
        material=satin_white,
        name="side_shell_1",
    )
    body.visual(
        Box((0.190, 0.026, 0.820)),
        origin=Origin(xyz=(0.0, -0.083, 0.450)),
        material=satin_white,
        name="rear_shell",
    )

    grille_mesh = mesh_from_geometry(
        VentGrilleGeometry(
            (0.155, 0.735),
            frame=0.012,
            face_thickness=0.004,
            duct_depth=0.018,
            duct_wall=0.0025,
            slat_pitch=0.018,
            slat_width=0.006,
            slat_angle_deg=18.0,
            corner_radius=0.010,
            slats=VentGrilleSlats(
                profile="airfoil",
                direction="down",
                inset=0.002,
                divider_count=2,
                divider_width=0.0035,
            ),
            frame_profile=VentGrilleFrame(style="radiused", depth=0.0015),
            sleeve=VentGrilleSleeve(style="short", depth=0.012, wall=0.0025),
        ),
        "front_vent_grille",
    )
    body.visual(
        grille_mesh,
        origin=Origin(xyz=(0.0, 0.080, 0.435), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_grille",
    )
    body.visual(
        Box((0.018, 0.022, 0.810)),
        origin=Origin(xyz=(-0.086, 0.078, 0.445)),
        material=satin_white,
        name="front_post_0",
    )
    body.visual(
        Box((0.018, 0.022, 0.810)),
        origin=Origin(xyz=(0.086, 0.078, 0.445)),
        material=satin_white,
        name="front_post_1",
    )
    body.visual(
        Box((0.182, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.078, 0.815)),
        material=satin_white,
        name="upper_vent_bridge",
    )

    body.visual(
        top_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        material=satin_white,
        name="top_cap",
    )
    body.visual(
        Box((0.160, 0.078, 0.006)),
        origin=Origin(xyz=(0.0, 0.025, 0.943)),
        material=soft_black,
        name="top_panel",
    )

    # Stationary tick marks molded/printed on the panel around the two rotating dials.
    for dial_x, prefix in [(-0.044, "speed"), (0.044, "timer")]:
        for i, angle in enumerate((-60.0, -25.0, 25.0, 60.0)):
            radius = 0.030
            rad = math.radians(angle)
            x = dial_x + radius * math.sin(rad)
            y = 0.025 + radius * math.cos(rad)
            body.visual(
                Box((0.010, 0.0022, 0.0015)),
                origin=Origin(xyz=(x, y, 0.9465), rpy=(0.0, 0.0, -rad)),
                material=tick_white,
                name=f"{prefix}_tick_{i}",
            )

    # The rear carry handle is an actual raised bridge with an open hand slot below it.
    body.visual(
        Box((0.024, 0.024, 0.118)),
        origin=Origin(xyz=(-0.070, -0.102, 0.972)),
        material=satin_white,
        name="handle_post_0",
    )
    body.visual(
        Box((0.024, 0.024, 0.118)),
        origin=Origin(xyz=(0.070, -0.102, 0.972)),
        material=satin_white,
        name="handle_post_1",
    )
    body.visual(
        Box((0.164, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, -0.102, 1.018)),
        material=satin_white,
        name="handle_bridge",
    )

    blower = model.part("blower_wheel")
    blower_mesh = mesh_from_geometry(
        BlowerWheelGeometry(
            0.047,
            0.020,
            0.740,
            32,
            blade_thickness=0.0025,
            blade_sweep_deg=26.0,
            backplate=True,
            shroud=True,
        ),
        "blower_wheel",
    )
    blower.visual(
        blower_mesh,
        material=dark_plastic,
        name="blower_wheel",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.024,
                body_style="skirted",
                top_diameter=0.031,
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=18, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", depth=0.001),
                center=False,
            ),
            "speed_dial",
        ),
        material=warm_gray,
        name="dial_cap",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.026,
                body_style="skirted",
                top_diameter=0.034,
                edge_radius=0.001,
                grip=KnobGrip(style="ribbed", count=24, depth=0.0008, width=0.0012),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_dial",
        ),
        material=warm_gray,
        name="dial_cap",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, -0.014, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=80.0),
    )
    model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_dial,
        origin=Origin(xyz=(-0.044, 0.025, 0.946)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(0.044, 0.025, 0.946)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower_wheel")
    speed_dial = object_model.get_part("speed_dial")
    timer_dial = object_model.get_part("timer_dial")
    body_joint = object_model.get_articulation("base_to_body")
    blower_joint = object_model.get_articulation("body_to_blower")
    speed_joint = object_model.get_articulation("body_to_speed_dial")
    timer_joint = object_model.get_articulation("body_to_timer_dial")

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="lower_cap",
        negative_elem="oscillation_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="oscillating body seats on base collar",
    )
    ctx.expect_gap(
        speed_dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="top_panel",
        max_gap=0.002,
        max_penetration=0.001,
        name="speed dial sits on the top panel",
    )
    ctx.expect_gap(
        timer_dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="top_panel",
        max_gap=0.002,
        max_penetration=0.001,
        name="timer dial sits on the top panel",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        inner_elem="blower_wheel",
        outer_elem="lower_cap",
        margin=0.002,
        name="blower wheel remains inside the tower footprint",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        elem_a="blower_wheel",
        elem_b="front_grille",
        min_overlap=0.60,
        name="blower spans the vented grille height",
    )

    ctx.check(
        "four primary articulations are present",
        len(object_model.articulations) == 4,
        details=f"found {[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "controls and blower are continuous rotations",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            for joint in (blower_joint, speed_joint, timer_joint)
        ),
        details=(
            f"types: blower={blower_joint.articulation_type}, "
            f"speed={speed_joint.articulation_type}, timer={timer_joint.articulation_type}"
        ),
    )
    ctx.check(
        "body oscillates on a limited vertical revolute joint",
        body_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(body_joint.axis) == (0.0, 0.0, 1.0)
        and body_joint.motion_limits is not None
        and body_joint.motion_limits.lower < 0.0
        and body_joint.motion_limits.upper > 0.0,
        details=f"type={body_joint.articulation_type}, axis={body_joint.axis}, limits={body_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
