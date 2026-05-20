from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_timber_louvered_shutter")

    timber = model.material("weathered_timber", rgba=(0.42, 0.24, 0.11, 1.0))
    endgrain = model.material("dark_endgrain", rgba=(0.24, 0.13, 0.06, 1.0))
    iron = model.material("blackened_iron", rgba=(0.035, 0.033, 0.030, 1.0))
    pin_metal = model.material("rubbed_pin_steel", rgba=(0.22, 0.21, 0.19, 1.0))
    stone = model.material("pale_stone_opening", rgba=(0.62, 0.58, 0.51, 1.0))

    panel_width = 0.85
    panel_height = 1.40
    panel_thickness = 0.075
    panel_y = 0.035
    stile_w = 0.10
    rail_h = 0.10
    inner_left = 0.13
    inner_right = 0.75
    louver_mid_x = (inner_left + inner_right) / 2.0
    louver_len = 0.60
    louver_zs = (0.23, 0.39, 0.55, 0.71, 0.87, 1.03, 1.19)
    hinge_zs = (0.32, 1.08)
    rod_x = panel_width * 0.52
    rod_y = -0.025
    rod_z = panel_height / 2.0

    opening = model.part("opening_frame")
    # A simple masonry/wooden opening surround sits behind the shutter panel.
    opening.visual(
        Box((0.10, 0.10, 1.56)),
        origin=Origin(xyz=(-0.055, 0.060, 0.700)),
        material=stone,
        name="hinge_jamb",
    )
    opening.visual(
        Box((0.10, 0.10, 1.56)),
        origin=Origin(xyz=(0.905, 0.060, 0.700)),
        material=stone,
        name="far_jamb",
    )
    opening.visual(
        Box((1.06, 0.10, 0.10)),
        origin=Origin(xyz=(0.425, 0.060, 1.455)),
        material=stone,
        name="top_lintel",
    )
    opening.visual(
        Box((1.06, 0.10, 0.10)),
        origin=Origin(xyz=(0.425, 0.060, -0.055)),
        material=stone,
        name="bottom_sill",
    )

    for idx, hz in enumerate(hinge_zs):
        opening.visual(
            Box((0.065, 0.012, 0.245)),
            origin=Origin(xyz=(-0.047, 0.006, hz)),
            material=iron,
            name=f"pintle_plate_{idx}",
        )
        # Separate upper and lower support ears hold the hinge pin without
        # colliding with the moving barrel collar at the middle of the hinge.
        for ear_idx, dz in enumerate((-0.096, 0.096)):
            opening.visual(
                Box((0.026, 0.082, 0.032)),
                origin=Origin(xyz=(-0.014, -0.033, hz + dz)),
                material=iron,
                name=f"pintle_ear_{idx}_{ear_idx}",
            )
        opening.visual(
            Cylinder(radius=0.009, length=0.250),
            origin=Origin(xyz=(0.0, -0.070, hz)),
            material=pin_metal,
            name=f"hinge_pin_{idx}",
        )

    panel = model.part("shutter_panel")
    # Thick continuous stiles and rails, with small overlaps at the corners so
    # the frame is one heavy timber assembly.
    panel.visual(
        Box((stile_w, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.080, panel_y, panel_height / 2.0)),
        material=timber,
        name="hinge_stile",
    )
    panel.visual(
        Box((stile_w, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.800, panel_y, panel_height / 2.0)),
        material=timber,
        name="free_stile",
    )
    panel.visual(
        Box((panel_width - 0.03, panel_thickness, rail_h)),
        origin=Origin(xyz=(0.440, panel_y, panel_height - rail_h / 2.0)),
        material=timber,
        name="top_rail",
    )
    panel.visual(
        Box((panel_width - 0.03, panel_thickness, rail_h)),
        origin=Origin(xyz=(0.440, panel_y, rail_h / 2.0)),
        material=timber,
        name="bottom_rail",
    )
    # Darker end-grain blocks make the oversized through-tenon construction
    # legible on the front face.
    for x, z, name in (
        (0.080, 1.345, "tenon_hinge_top"),
        (0.800, 1.345, "tenon_free_top"),
        (0.080, 0.055, "tenon_hinge_bottom"),
        (0.800, 0.055, "tenon_free_bottom"),
    ):
        panel.visual(
            Box((0.062, 0.008, 0.052)),
            origin=Origin(xyz=(x, -0.006, z)),
            material=endgrain,
            name=name,
        )

    for idx, hz in enumerate(hinge_zs):
        panel.visual(
            Cylinder(radius=0.0175, length=0.140),
            origin=Origin(xyz=(0.0, 0.0, hz)),
            material=iron,
            name=f"hinge_barrel_{idx}",
        )
        panel.visual(
            Box((0.528, 0.012, 0.055)),
            origin=Origin(xyz=(0.276, -0.006, hz)),
            material=iron,
            name=f"strap_leaf_{idx}",
        )
        panel.visual(
            Box((0.070, 0.014, 0.072)),
            origin=Origin(xyz=(0.500, -0.006, hz)),
            material=iron,
            name=f"strap_bolt_pad_{idx}",
        )
        for bx in (0.145, 0.350, 0.500):
            panel.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(bx, -0.014, hz), rpy=(pi / 2.0, 0.0, 0.0)),
                material=pin_metal,
                name=f"strap_bolt_{idx}_{int(bx * 1000)}",
            )

    control_rod = model.part("control_rod")
    control_rod.visual(
        Cylinder(radius=0.010, length=1.055),
        origin=Origin(),
        material=iron,
        name="vertical_rod",
    )
    control_rod.visual(
        Box((0.060, 0.026, 0.085)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=iron,
        name="pull_paddle",
    )
    for idx, z in enumerate(louver_zs):
        control_rod.visual(
            Box((0.038, 0.020, 0.020)),
            origin=Origin(xyz=(0.0, 0.020, z - rod_z)),
            material=pin_metal,
            name=f"link_tab_{idx}",
        )

    louvers = []
    for idx, z in enumerate(louver_zs):
        louver = model.part(f"louver_{idx}")
        louver.visual(
            Box((louver_len, 0.035, 0.130)),
            origin=Origin(rpy=(-0.32, 0.0, 0.0)),
            material=timber,
            name="slat",
        )
        louver.visual(
            Cylinder(radius=0.008, length=0.720),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_metal,
            name="pivot_axle",
        )
        # A small center boss makes the control-rod pin connection visible and
        # overlaps the slat body so it remains a connected moving louver part.
        louver.visual(
            Box((0.050, 0.020, 0.024)),
            origin=Origin(xyz=(0.0, -0.025, 0.000)),
            material=iron,
            name="center_boss",
        )
        louvers.append(louver)

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=opening,
        child=panel,
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=80.0, velocity=0.9),
    )

    model.articulation(
        "rod_pivot",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=control_rod,
        origin=Origin(xyz=(rod_x, rod_y, rod_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.27, upper=0.27, effort=12.0, velocity=1.0),
    )

    for idx, (louver, z) in enumerate(zip(louvers, louver_zs)):
        model.articulation(
            f"louver_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(louver_mid_x, panel_y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=6.0, velocity=1.4),
            mimic=Mimic(joint="rod_pivot", multiplier=2.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    opening = object_model.get_part("opening_frame")
    panel = object_model.get_part("shutter_panel")
    rod = object_model.get_part("control_rod")
    panel_hinge = object_model.get_articulation("panel_hinge")
    rod_pivot = object_model.get_articulation("rod_pivot")

    for idx in range(2):
        ctx.allow_overlap(
            opening,
            panel,
            elem_a=f"hinge_pin_{idx}",
            elem_b=f"hinge_barrel_{idx}",
            reason="The fixed pintle pin is intentionally captured inside the shutter-side barrel hinge collar.",
        )
        ctx.expect_within(
            opening,
            panel,
            axes="xy",
            inner_elem=f"hinge_pin_{idx}",
            outer_elem=f"hinge_barrel_{idx}",
            margin=0.003,
            name=f"hinge pin {idx} centered in barrel",
        )
        ctx.expect_overlap(
            opening,
            panel,
            axes="z",
            elem_a=f"hinge_pin_{idx}",
            elem_b=f"hinge_barrel_{idx}",
            min_overlap=0.120,
            name=f"hinge pin {idx} retained through barrel",
        )

    for idx in range(7):
        louver = object_model.get_part(f"louver_{idx}")
        ctx.allow_overlap(
            panel,
            louver,
            elem_a="hinge_stile",
            elem_b="pivot_axle",
            reason="The louver's long-axis axle is seated into a bored pivot in the hinge-side stile.",
        )
        ctx.allow_overlap(
            panel,
            louver,
            elem_a="free_stile",
            elem_b="pivot_axle",
            reason="The louver's long-axis axle is seated into a bored pivot in the free-side stile.",
        )
        ctx.expect_overlap(
            panel,
            louver,
            axes="x",
            elem_a="hinge_stile",
            elem_b="pivot_axle",
            min_overlap=0.020,
            name=f"louver {idx} hinge-side axle insertion",
        )
        ctx.expect_overlap(
            panel,
            louver,
            axes="x",
            elem_a="free_stile",
            elem_b="pivot_axle",
            min_overlap=0.020,
            name=f"louver {idx} free-side axle insertion",
        )
        ctx.allow_overlap(
            rod,
            louver,
            elem_a=f"link_tab_{idx}",
            elem_b="center_boss",
            reason="The tilt rod tab and louver boss are represented as a pinned control linkage with a tiny shared fastener volume.",
        )
        ctx.expect_contact(
            rod,
            louver,
            elem_a=f"link_tab_{idx}",
            elem_b="center_boss",
            contact_tol=0.012,
            name=f"control rod tab reaches louver {idx}",
        )

    ctx.expect_within(
        rod,
        panel,
        axes="xz",
        margin=0.015,
        name="central control rod stays within the shutter panel footprint",
    )

    free_stile_closed = ctx.part_element_world_aabb(panel, elem="free_stile")
    with ctx.pose({panel_hinge: 1.15}):
        free_stile_open = ctx.part_element_world_aabb(panel, elem="free_stile")
    ctx.check(
        "panel hinge swings the free stile outward",
        free_stile_closed is not None
        and free_stile_open is not None
        and free_stile_open[0][1] > free_stile_closed[0][1] + 0.25,
        details=f"closed={free_stile_closed}, open={free_stile_open}",
    )

    sample_louver = object_model.get_part("louver_3")
    slat_rest = ctx.part_element_world_aabb(sample_louver, elem="slat")
    paddle_rest = ctx.part_element_world_aabb(rod, elem="pull_paddle")
    with ctx.pose({rod_pivot: 0.25}):
        slat_raised = ctx.part_element_world_aabb(sample_louver, elem="slat")
        paddle_raised = ctx.part_element_world_aabb(rod, elem="pull_paddle")
    ctx.check(
        "control rod rocks the handle",
        paddle_rest is not None
        and paddle_raised is not None
        and abs(paddle_raised[0][1] - paddle_rest[0][1]) > 0.008,
        details=f"rest={paddle_rest}, raised={paddle_raised}",
    )
    ctx.check(
        "control rod motion tilts the louvers",
        slat_rest is not None
        and slat_raised is not None
        and abs((slat_raised[1][1] - slat_raised[0][1]) - (slat_rest[1][1] - slat_rest[0][1])) > 0.010,
        details=f"rest={slat_rest}, raised={slat_raised}",
    )

    return ctx.report()


object_model = build_object_model()
