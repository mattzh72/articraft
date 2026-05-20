from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.14
FRONT_PANEL_Y = -0.277
BUTTON_TRAVEL = 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = Material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_stainless = Material("dark_stainless", rgba=(0.42, 0.43, 0.41, 1.0))
    shadow = Material("recess_shadow", rgba=(0.03, 0.035, 0.035, 1.0))
    black = Material("black_control", rgba=(0.015, 0.015, 0.014, 1.0))
    white = Material("white_marking", rgba=(0.95, 0.94, 0.88, 1.0))
    filter_metal = Material("aluminum_filter", rgba=(0.58, 0.60, 0.58, 1.0))

    hood = model.part("hood")

    # Wide, flat lower canopy.
    hood.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT / 2.0)),
        material=stainless,
        name="flat_canopy",
    )
    hood.visual(
        Box((CANOPY_WIDTH + 0.035, CANOPY_DEPTH + 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_stainless,
        name="lower_lip",
    )
    hood.visual(
        Box((0.880, 0.027, 0.105)),
        origin=Origin(xyz=(0.0, -0.2635, 0.080)),
        material=stainless,
        name="front_panel",
    )
    hood.visual(
        Box((0.860, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, FRONT_PANEL_Y - 0.004, 0.128)),
        material=dark_stainless,
        name="front_shadow_seam",
    )

    # Narrow chimney chase centered on the rear of the canopy.
    hood.visual(
        Box((0.280, 0.200, 0.790)),
        origin=Origin(xyz=(0.0, 0.150, 0.535)),
        material=stainless,
        name="chimney_chase",
    )
    hood.visual(
        Box((0.360, 0.280, 0.052)),
        origin=Origin(xyz=(0.0, 0.115, 0.166)),
        material=dark_stainless,
        name="chase_base_collar",
    )
    hood.visual(
        Box((0.290, 0.010, 0.760)),
        origin=Origin(xyz=(0.0, 0.047, 0.535)),
        material=dark_stainless,
        name="chase_front_seam",
    )
    hood.visual(
        Box((0.910, 0.030, 0.034)),
        origin=Origin(xyz=(0.0, -0.251, 0.018)),
        material=dark_stainless,
        name="front_rolled_edge",
    )

    # Visible underside grease filters set into the flat bottom.
    filter_geom = SlotPatternPanelGeometry(
        (0.350, 0.170),
        0.004,
        slot_size=(0.030, 0.006),
        pitch=(0.043, 0.018),
        frame=0.012,
        corner_radius=0.004,
        slot_angle_deg=16.0,
    )
    filter_mesh = mesh_from_geometry(filter_geom, "grease_filter")
    for idx, x in enumerate((-0.205, 0.205)):
        hood.visual(
            filter_mesh,
            origin=Origin(xyz=(x, 0.050, -0.002)),
            material=filter_metal,
            name=f"filter_panel_{idx}",
        )

    # Small legends and a pointer tick make the controls read as appliance controls.
    for idx, z in enumerate((0.104, 0.058)):
        hood.visual(
            Box((0.020, 0.001, 0.004)),
            origin=Origin(xyz=(-0.375, FRONT_PANEL_Y - 0.0005, z)),
            material=white,
            name=f"button_mark_{idx}",
        )
    hood.visual(
        Box((0.006, 0.001, 0.022)),
        origin=Origin(xyz=(0.320, FRONT_PANEL_Y - 0.0005, 0.122)),
        material=white,
        name="knob_top_mark",
    )

    # Two vertically stacked push-buttons on the left side of the front panel.
    for idx, z in enumerate((0.104, 0.058)):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.060, 0.018, 0.028)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=black,
            name="button_cap",
        )
        button.visual(
            Box((0.052, 0.006, 0.020)),
            origin=Origin(xyz=(0.0, -0.021, 0.0)),
            material=dark_stainless,
            name="button_face_inset",
        )
        model.articulation(
            f"hood_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=hood,
            child=button,
            origin=Origin(xyz=(-0.315, FRONT_PANEL_Y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    # Single rotary knob on the right; its local +Z axis points out of the front.
    knob = model.part("knob")
    knob_geom = KnobGeometry(
        0.052,
        0.026,
        body_style="skirted",
        top_diameter=0.041,
        skirt=KnobSkirt(0.060, 0.006, flare=0.06, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=20, depth=0.0011),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "rotary_knob"),
        material=black,
        name="knob_cap",
    )
    model.articulation(
        "hood_to_knob",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=knob,
        origin=Origin(xyz=(0.320, FRONT_PANEL_Y, 0.083), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood = object_model.get_part("hood")
    buttons = [object_model.get_part(f"button_{idx}") for idx in range(2)]
    knob = object_model.get_part("knob")
    button_joints = [object_model.get_articulation(f"hood_to_button_{idx}") for idx in range(2)]
    knob_joint = object_model.get_articulation("hood_to_knob")

    ctx.check(
        "only requested controls articulate",
        len(object_model.articulations) == 3,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "buttons use prismatic joints and knob uses continuous rotation",
        all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints)
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"button_types={[joint.articulation_type for joint in button_joints]}, "
            f"knob_type={knob_joint.articulation_type}"
        ),
    )

    # The simplified solid front fascia stands in for shallow molded button sockets.
    # Scope the allowance to the cap/fascia interface and prove the seating/travel.
    for idx, (button, joint) in enumerate(zip(buttons, button_joints)):
        ctx.allow_overlap(
            hood,
            button,
            elem_a="front_panel",
            elem_b="button_cap",
            reason="The push-button cap intentionally slides into a shallow front-panel socket.",
        )
        ctx.expect_gap(
            hood,
            button,
            axis="y",
            positive_elem="front_panel",
            negative_elem="button_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"button {idx} starts seated on front panel",
        )
        ctx.expect_overlap(
            hood,
            button,
            axes="xz",
            elem_a="front_panel",
            elem_b="button_cap",
            min_overlap=0.025,
            name=f"button {idx} lies within front panel footprint",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            ctx.expect_gap(
                hood,
                button,
                axis="y",
                positive_elem="front_panel",
                negative_elem="button_cap",
                max_penetration=BUTTON_TRAVEL + 0.001,
                name=f"button {idx} pressed travel remains shallow",
            )
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {idx} moves inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + BUTTON_TRAVEL * 0.8,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    upper_pos = ctx.part_world_position(buttons[0])
    lower_pos = ctx.part_world_position(buttons[1])
    knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "two push-buttons are vertically stacked on the left",
        upper_pos is not None
        and lower_pos is not None
        and abs(upper_pos[0] - lower_pos[0]) < 0.002
        and upper_pos[2] > lower_pos[2] + 0.035
        and upper_pos[0] < -0.20,
        details=f"upper={upper_pos}, lower={lower_pos}",
    )
    ctx.check(
        "rotary knob sits alone on the right",
        knob_pos is not None and upper_pos is not None and knob_pos[0] > upper_pos[0] + 0.50,
        details=f"knob={knob_pos}, upper_button={upper_pos}",
    )
    ctx.expect_gap(
        hood,
        knob,
        axis="y",
        positive_elem="front_panel",
        negative_elem="knob_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="knob mounts proud of front panel",
    )

    return ctx.report()


object_model = build_object_model()
