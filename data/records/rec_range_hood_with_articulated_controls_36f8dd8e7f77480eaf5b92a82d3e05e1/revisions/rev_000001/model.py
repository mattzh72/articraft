from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_out_visor_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark = model.material("shadow_black", rgba=(0.015, 0.016, 0.018, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.025, 1.0))
    grey_plastic = model.material("warm_grey_plastic", rgba=(0.18, 0.18, 0.17, 1.0))
    filter_metal = model.material("mesh_filter_aluminum", rgba=(0.55, 0.56, 0.52, 1.0))

    fixed_body = model.part("fixed_body")

    # Compact 600 mm apartment range-hood shell: open below, with metal side walls
    # and a shallow upper plenum rather than a solid block.
    fixed_body.visual(
        Box((0.34, 0.60, 0.035)),
        origin=Origin(xyz=(-0.17, 0.0, 0.1325)),
        material=stainless,
        name="top_cover",
    )
    fixed_body.visual(
        Box((0.035, 0.60, 0.120)),
        origin=Origin(xyz=(-0.3225, 0.0, 0.075)),
        material=stainless,
        name="rear_wall",
    )
    fixed_body.visual(
        Box((0.34, 0.025, 0.120)),
        origin=Origin(xyz=(-0.17, -0.2875, 0.075)),
        material=stainless,
        name="side_wall_0",
    )
    fixed_body.visual(
        Box((0.34, 0.025, 0.120)),
        origin=Origin(xyz=(-0.17, 0.2875, 0.075)),
        material=stainless,
        name="side_wall_1",
    )
    fixed_body.visual(
        Box((0.035, 0.60, 0.065)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0925)),
        material=stainless,
        name="front_header",
    )

    # Dark recessed underside and a slotted grease-filter face.
    fixed_body.visual(
        Box((0.245, 0.50, 0.008)),
        origin=Origin(xyz=(-0.175, 0.0, 0.034)),
        material=dark,
        name="underside_shadow",
    )
    fixed_body.visual(
        Box((0.205, 0.035, 0.012)),
        origin=Origin(xyz=(-0.17, -0.2575, 0.040)),
        material=stainless,
        name="filter_side_frame_0",
    )
    fixed_body.visual(
        Box((0.205, 0.035, 0.012)),
        origin=Origin(xyz=(-0.17, 0.2575, 0.040)),
        material=stainless,
        name="filter_side_frame_1",
    )
    fixed_body.visual(
        Box((0.030, 0.55, 0.012)),
        origin=Origin(xyz=(-0.2875, 0.0, 0.040)),
        material=stainless,
        name="filter_rear_frame",
    )
    fixed_body.visual(
        Box((0.030, 0.55, 0.012)),
        origin=Origin(xyz=(-0.0525, 0.0, 0.040)),
        material=stainless,
        name="filter_front_frame",
    )
    fixed_body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.205, 0.460),
                0.005,
                slot_size=(0.030, 0.006),
                pitch=(0.040, 0.018),
                frame=0.012,
                corner_radius=0.004,
                slot_angle_deg=8.0,
                stagger=True,
            ),
            "grease_filter",
        ),
        origin=Origin(xyz=(-0.17, 0.0, 0.040)),
        material=filter_metal,
        name="grease_filter",
    )

    # Straight internal guides visible along the sides of the sliding visor.
    fixed_body.visual(
        Box((0.275, 0.026, 0.014)),
        origin=Origin(xyz=(-0.1375, -0.247, 0.038)),
        material=stainless,
        name="guide_top_0",
    )
    fixed_body.visual(
        Box((0.275, 0.010, 0.034)),
        origin=Origin(xyz=(-0.1375, -0.265, 0.014)),
        material=stainless,
        name="guide_lip_0",
    )
    fixed_body.visual(
        Box((0.275, 0.026, 0.014)),
        origin=Origin(xyz=(-0.1375, 0.247, 0.038)),
        material=stainless,
        name="guide_top_1",
    )
    fixed_body.visual(
        Box((0.275, 0.010, 0.034)),
        origin=Origin(xyz=(-0.1375, 0.265, 0.014)),
        material=stainless,
        name="guide_lip_1",
    )

    # Proud black side control panel, tied into the metal side wall.
    fixed_body.visual(
        Box((0.205, 0.008, 0.070)),
        origin=Origin(xyz=(-0.165, 0.304, 0.075)),
        material=black_plastic,
        name="control_panel",
    )
    fixed_body.visual(
        Box((0.005, 0.014, 0.070)),
        origin=Origin(xyz=(-0.2675, 0.302, 0.075)),
        material=black_plastic,
        name="control_panel_rear_tab",
    )
    fixed_body.visual(
        Box((0.005, 0.014, 0.070)),
        origin=Origin(xyz=(-0.0625, 0.302, 0.075)),
        material=black_plastic,
        name="control_panel_front_tab",
    )

    visor = model.part("front_visor")
    # The child frame is at the fixed front entry plane.  Hidden length extends
    # rearward so the tray remains engaged in the guide channels when extended.
    visor.visual(
        Box((0.310, 0.465, 0.024)),
        origin=Origin(xyz=(-0.095, 0.0, 0.012)),
        material=stainless,
        name="slide_tray",
    )
    visor.visual(
        Box((0.040, 0.580, 0.070)),
        origin=Origin(xyz=(0.020, 0.0, 0.040)),
        material=stainless,
        name="front_fascia",
    )
    visor.visual(
        Box((0.018, 0.420, 0.014)),
        origin=Origin(xyz=(0.050, 0.0, 0.010)),
        material=dark,
        name="finger_pull_shadow",
    )
    visor.visual(
        Box((0.305, 0.018, 0.016)),
        origin=Origin(xyz=(-0.0875, -0.247, 0.022)),
        material=stainless,
        name="side_runner_0",
    )
    visor.visual(
        Box((0.305, 0.018, 0.016)),
        origin=Origin(xyz=(-0.0875, 0.247, 0.022)),
        material=stainless,
        name="side_runner_1",
    )

    rocker = model.part("side_rocker")
    rocker.visual(
        Box((0.048, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=grey_plastic,
        name="rocker_paddle",
    )
    rocker.visual(
        Cylinder(radius=0.004, length=0.052),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_plastic,
        name="pivot_pin",
    )
    rocker.visual(
        Box((0.040, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.016, 0.008)),
        material=black_plastic,
        name="rocker_high_mark",
    )

    knob = model.part("side_knob")
    knob.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=black_plastic,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.024,
                body_style="skirted",
                top_diameter=0.032,
                skirt=None,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                bore=KnobBore(style="round", diameter=0.007),
                center=False,
            ),
            "fan_speed_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=grey_plastic,
        name="knob_cap",
    )

    model.articulation(
        "body_to_visor",
        ArticulationType.PRISMATIC,
        parent=fixed_body,
        child=visor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.28, lower=0.0, upper=0.180),
    )
    model.articulation(
        "body_to_rocker",
        ArticulationType.REVOLUTE,
        parent=fixed_body,
        child=rocker,
        origin=Origin(xyz=(-0.210, 0.308, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=fixed_body,
        child=knob,
        origin=Origin(xyz=(-0.120, 0.308, 0.075), rpy=(-pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_body = object_model.get_part("fixed_body")
    visor = object_model.get_part("front_visor")
    rocker = object_model.get_part("side_rocker")
    knob = object_model.get_part("side_knob")
    visor_slide = object_model.get_articulation("body_to_visor")
    rocker_hinge = object_model.get_articulation("body_to_rocker")
    knob_turn = object_model.get_articulation("body_to_knob")

    ctx.expect_gap(
        visor,
        fixed_body,
        axis="x",
        positive_elem="front_fascia",
        negative_elem="front_header",
        min_gap=-0.001,
        max_gap=0.001,
        name="closed visor fascia seats at fixed front",
    )
    ctx.expect_overlap(
        visor,
        fixed_body,
        axes="x",
        elem_a="side_runner_0",
        elem_b="guide_top_0",
        min_overlap=0.22,
        name="visor runner is deeply engaged when closed",
    )
    ctx.expect_within(
        visor,
        fixed_body,
        axes="y",
        inner_elem="slide_tray",
        outer_elem="top_cover",
        margin=0.001,
        name="slide tray fits inside compact hood width",
    )
    ctx.expect_contact(
        rocker,
        fixed_body,
        elem_a="pivot_pin",
        elem_b="control_panel",
        contact_tol=0.001,
        name="rocker hinge pin is seated on side panel",
    )
    ctx.expect_contact(
        knob,
        fixed_body,
        elem_a="shaft",
        elem_b="control_panel",
        contact_tol=0.001,
        name="knob shaft emerges from side control panel",
    )

    rest_pos = ctx.part_world_position(visor)
    with ctx.pose({visor_slide: 0.180}):
        ctx.expect_gap(
            visor,
            fixed_body,
            axis="x",
            positive_elem="front_fascia",
            negative_elem="front_header",
            min_gap=0.175,
            max_gap=0.185,
            name="visor extends forward on straight guides",
        )
        ctx.expect_overlap(
            visor,
            fixed_body,
            axes="x",
            elem_a="side_runner_0",
            elem_b="guide_top_0",
            min_overlap=0.045,
            name="extended visor still retains guide insertion",
        )
        extended_pos = ctx.part_world_position(visor)
    ctx.check(
        "prismatic visor motion is forward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.17,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "rocker has short limited pivot",
        rocker_hinge.articulation_type == ArticulationType.REVOLUTE
        and rocker_hinge.motion_limits is not None
        and rocker_hinge.motion_limits.lower < 0.0
        and rocker_hinge.motion_limits.upper > 0.0
        and rocker_hinge.motion_limits.upper <= 0.30,
        details=f"limits={rocker_hinge.motion_limits}",
    )
    ctx.check(
        "side knob is continuously rotary",
        knob_turn.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_turn.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
