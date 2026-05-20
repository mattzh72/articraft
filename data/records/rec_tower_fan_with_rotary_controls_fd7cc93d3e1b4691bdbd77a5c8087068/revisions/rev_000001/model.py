from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_tower_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.86, 0.82, 1.0))
    light_gray = model.material("light_gray_plastic", rgba=(0.63, 0.64, 0.62, 1.0))
    dark_gray = model.material("charcoal_gray", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    grille_black = model.material("deep_black_grille", rgba=(0.0, 0.0, 0.0, 1.0))
    soft_blue = model.material("translucent_smoke_blower", rgba=(0.12, 0.18, 0.22, 0.72))
    white_mark = model.material("white_control_mark", rgba=(0.95, 0.95, 0.90, 1.0))

    # Root: a residential-scale circular pedestal and the stationary half of the
    # oscillation bearing.  The rotating tower sits on a visible seam above it.
    base = model.part("pedestal_base")
    base.visual(
        Cylinder(radius=0.230, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=light_gray,
        name="round_pedestal",
    )
    base.visual(
        Cylinder(radius=0.165, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=warm_white,
        name="raised_plinth",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_gray,
        name="stationary_pivot",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=satin_black,
        name="bearing_ring",
    )

    # Oscillating tower body.  Its local origin is the vertical oscillation axis
    # on top of the base bearing, making the seam and motion source explicit.
    body = model.part("tower_body")
    body.visual(
        Cylinder(radius=0.074, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=warm_white,
        name="lower_collar",
    )
    body.visual(
        Box((0.034, 0.160, 0.900)),
        origin=Origin(xyz=(-0.093, 0.0, 0.480)),
        material=warm_white,
        name="side_rail_0",
    )
    body.visual(
        Box((0.034, 0.160, 0.900)),
        origin=Origin(xyz=(0.093, 0.0, 0.480)),
        material=warm_white,
        name="side_rail_1",
    )
    body.visual(
        Box((0.220, 0.024, 0.900)),
        origin=Origin(xyz=(0.0, 0.068, 0.480)),
        material=warm_white,
        name="rear_wall",
    )
    body.visual(
        Box((0.220, 0.160, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=warm_white,
        name="bottom_cap",
    )
    body.visual(
        Box((0.220, 0.160, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        material=warm_white,
        name="top_cap",
    )

    # Front grille: a real open barred grille, not a printed solid texture.
    body.visual(
        Box((0.012, 0.008, 0.770)),
        origin=Origin(xyz=(-0.073, -0.080, 0.485)),
        material=grille_black,
        name="front_grille_side_0",
    )
    body.visual(
        Box((0.012, 0.008, 0.770)),
        origin=Origin(xyz=(0.073, -0.080, 0.485)),
        material=grille_black,
        name="front_grille_side_1",
    )
    body.visual(
        Box((0.158, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.080, 0.866)),
        material=grille_black,
        name="front_grille_top",
    )
    body.visual(
        Box((0.158, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.080, 0.104)),
        material=grille_black,
        name="front_grille_bottom",
    )
    for idx in range(25):
        z = 0.130 + idx * 0.029
        body.visual(
            Box((0.158, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, -0.083, z)),
            material=grille_black,
            name=f"front_slat_{idx}",
        )

    body.visual(
        Box((0.145, 0.006, 0.520)),
        origin=Origin(xyz=(0.0, 0.083, 0.520)),
        material=dark_gray,
        name="rear_intake_panel",
    )
    for idx in range(13):
        body.visual(
            Box((0.125, 0.003, 0.004)),
            origin=Origin(xyz=(0.0, 0.087, 0.285 + idx * 0.039)),
            material=light_gray,
            name=f"rear_intake_slot_{idx}",
        )

    # The top deck is a seated dark inset on the cap.  The carry handle is a
    # supported bridge at the rear; the space under it remains a real opening.
    body.visual(
        Box((0.165, 0.095, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, 0.934)),
        material=dark_gray,
        name="control_deck",
    )
    body.visual(
        Box((0.030, 0.032, 0.070)),
        origin=Origin(xyz=(-0.073, 0.094, 0.964)),
        material=warm_white,
        name="handle_post_0",
    )
    body.visual(
        Box((0.030, 0.032, 0.070)),
        origin=Origin(xyz=(0.073, 0.094, 0.964)),
        material=warm_white,
        name="handle_post_1",
    )
    body.visual(
        Box((0.176, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.094, 1.014)),
        material=warm_white,
        name="rear_handle_bridge",
    )

    # Fixed white tick marks on the control deck emphasize that the separate
    # rotary knobs are user controls rather than static molded bumps.
    for idx, x in enumerate((-0.056, 0.0, 0.056)):
        body.visual(
            Box((0.004, 0.020, 0.002)),
            origin=Origin(xyz=(x, -0.052, 0.940), rpy=(0.0, 0.0, math.radians(12.0))),
            material=white_mark,
            name=f"deck_tick_{idx}",
        )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.15, upper=1.15),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    # Internal vertical cross-flow blower wheel.  It is clear of the hollow shell
    # and spins continuously about the same vertical center axis.
    blower = model.part("blower_wheel")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.047,
                0.027,
                0.700,
                28,
                blade_thickness=0.0032,
                blade_sweep_deg=24.0,
            ),
            "blower_wheel",
        ),
        material=soft_blue,
        name="blower_wheel",
    )
    blower.visual(
        Cylinder(radius=0.009, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="blower_shaft",
    )
    blower.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="blower_hub",
    )
    model.articulation(
        "tower_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=80.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.0),
    )

    knob_specs = [
        ("speed_knob", -0.056, 0.040, 0.024, 0.022, 0.019),
        ("timer_knob", 0.000, 0.040, 0.023, 0.021, 0.018),
        ("mode_knob", 0.056, 0.040, 0.020, 0.019, 0.016),
    ]

    for part_name, x, y, skirt_radius, body_radius, cap_radius in knob_specs:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=skirt_radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=light_gray,
            name=f"{part_name}_skirt",
        )
        knob.visual(
            Cylinder(radius=body_radius, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=light_gray,
            name=part_name,
        )
        knob.visual(
            Cylinder(radius=cap_radius, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
            material=warm_white,
            name=f"{part_name}_cap",
        )
        knob.visual(
            Box((0.004, body_radius * 1.35, 0.002)),
            origin=Origin(xyz=(0.0, -body_radius * 0.25, 0.028)),
            material=white_mark,
            name=f"{part_name}_pointer",
        )
        model.articulation(
            f"tower_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, y, 0.939)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal_base")
    body = object_model.get_part("tower_body")
    blower = object_model.get_part("blower_wheel")
    oscillation = object_model.get_articulation("base_to_tower")
    blower_spin = object_model.get_articulation("tower_to_blower")

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="bearing_ring",
        max_gap=0.001,
        max_penetration=0.000001,
        name="oscillating collar seats on base bearing",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        inner_elem="blower_wheel",
        margin=0.002,
        name="blower remains inside tower footprint",
    )
    ctx.expect_gap(
        blower,
        body,
        axis="y",
        positive_elem="blower_wheel",
        negative_elem="front_grille_side_0",
        min_gap=0.010,
        name="blower clears front grille",
    )

    # Prompt-critical articulations and axes.
    ctx.check(
        "oscillation is limited vertical revolute",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in oscillation.axis) == (0.0, 0.0, 1.0)
        and oscillation.motion_limits is not None
        and oscillation.motion_limits.lower < -1.0
        and oscillation.motion_limits.upper > 1.0,
        details=f"type={oscillation.articulation_type}, axis={oscillation.axis}, limits={oscillation.motion_limits}",
    )
    ctx.check(
        "blower is continuous on vertical axis",
        blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in blower_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={blower_spin.articulation_type}, axis={blower_spin.axis}",
    )

    for joint_name in ("tower_to_speed_knob", "tower_to_timer_knob", "tower_to_mode_knob"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous vertical rotary control",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    deck_bb = ctx.part_element_world_aabb(body, elem="control_deck")
    bridge_bb = ctx.part_element_world_aabb(body, elem="rear_handle_bridge")
    post_0_bb = ctx.part_element_world_aabb(body, elem="handle_post_0")
    post_1_bb = ctx.part_element_world_aabb(body, elem="handle_post_1")
    opening_ok = (
        deck_bb is not None
        and bridge_bb is not None
        and post_0_bb is not None
        and post_1_bb is not None
        and bridge_bb[0][2] - deck_bb[1][2] > 0.045
        and post_1_bb[0][0] - post_0_bb[1][0] > 0.085
    )
    ctx.check(
        "rear handle has a real supported opening",
        opening_ok,
        details=f"deck={deck_bb}, bridge={bridge_bb}, post0={post_0_bb}, post1={post_1_bb}",
    )

    rest_pos = ctx.part_world_position(body)
    with ctx.pose({oscillation: 0.75}):
        swept_pos = ctx.part_world_position(body)
        ctx.expect_gap(
            blower,
            body,
            axis="y",
            positive_elem="blower_wheel",
            negative_elem="front_grille_side_0",
            min_gap=0.010,
            name="blower clears grille while tower is oscillated",
        )

    ctx.check(
        "oscillation rotates about a fixed base center",
        rest_pos is not None
        and swept_pos is not None
        and abs(rest_pos[0] - swept_pos[0]) < 1e-6
        and abs(rest_pos[1] - swept_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
