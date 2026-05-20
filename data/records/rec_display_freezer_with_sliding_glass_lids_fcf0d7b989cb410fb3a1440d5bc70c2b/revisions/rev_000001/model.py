from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supermarket_island_display_freezer")

    white = model.material("warm_white_powder_coat", rgba=(0.92, 0.94, 0.92, 1.0))
    liner = model.material("pale_blue_white_liner", rgba=(0.80, 0.92, 0.96, 1.0))
    black = model.material("black_plastic_toe_kick", rgba=(0.02, 0.022, 0.025, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.70, 1.0))
    dark = model.material("dark_gray_control", rgba=(0.05, 0.055, 0.06, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.62, 0.86, 0.96, 0.38))
    label = model.material("control_label_print", rgba=(0.08, 0.10, 0.11, 1.0))

    cabinet = model.part("cabinet")

    # Supermarket island-display scale: a long insulated open tub with a thick
    # rim.  The separate wall/floor solids leave the top and product well visibly
    # hollow instead of capping it with a placeholder block.
    cabinet.visual(Box((2.42, 0.98, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=black, name="toe_kick")
    cabinet.visual(Box((2.40, 0.95, 0.16)), origin=Origin(xyz=(0.0, 0.0, 0.18)), material=white, name="insulated_floor")
    cabinet.visual(Box((2.40, 0.10, 0.58)), origin=Origin(xyz=(0.0, -0.425, 0.53)), material=white, name="long_wall_0")
    cabinet.visual(Box((2.40, 0.10, 0.58)), origin=Origin(xyz=(0.0, 0.425, 0.53)), material=white, name="long_wall_1")
    cabinet.visual(Box((0.10, 0.95, 0.58)), origin=Origin(xyz=(-1.15, 0.0, 0.53)), material=white, name="end_panel")
    cabinet.visual(Box((0.10, 0.95, 0.58)), origin=Origin(xyz=(1.15, 0.0, 0.53)), material=white, name="control_end_panel")

    cabinet.visual(Box((2.18, 0.74, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.272)), material=liner, name="well_floor_liner")
    cabinet.visual(Box((2.18, 0.025, 0.47)), origin=Origin(xyz=(0.0, -0.372, 0.505)), material=liner, name="well_liner_0")
    cabinet.visual(Box((2.18, 0.025, 0.47)), origin=Origin(xyz=(0.0, 0.372, 0.505)), material=liner, name="well_liner_1")
    cabinet.visual(Box((0.025, 0.74, 0.47)), origin=Origin(xyz=(-1.092, 0.0, 0.505)), material=liner, name="well_end_liner")
    cabinet.visual(Box((0.025, 0.74, 0.47)), origin=Origin(xyz=(1.092, 0.0, 0.505)), material=liner, name="well_control_liner")

    cabinet.visual(Box((2.50, 0.14, 0.08)), origin=Origin(xyz=(0.0, -0.455, 0.84)), material=white, name="rim_0")
    cabinet.visual(Box((2.50, 0.14, 0.08)), origin=Origin(xyz=(0.0, 0.455, 0.84)), material=white, name="rim_1")
    cabinet.visual(Box((0.14, 0.98, 0.08)), origin=Origin(xyz=(-1.22, 0.0, 0.84)), material=white, name="rim_end")
    cabinet.visual(Box((0.14, 0.98, 0.08)), origin=Origin(xyz=(1.22, 0.0, 0.84)), material=white, name="rim_control_end")

    # Visible rail tracks run lengthwise across the open well.  The rear pair is
    # raised, matching the stacked tracks found on sliding freezer lids.
    cabinet.visual(Box((2.30, 0.030, 0.030)), origin=Origin(xyz=(0.0, -0.345, 0.865)), material=aluminum, name="lower_rail_0")
    cabinet.visual(Box((2.30, 0.030, 0.030)), origin=Origin(xyz=(0.0, 0.345, 0.865)), material=aluminum, name="lower_rail_1")
    cabinet.visual(Box((2.30, 0.030, 0.030)), origin=Origin(xyz=(0.0, -0.345, 0.955)), material=aluminum, name="upper_rail_0")
    cabinet.visual(Box((2.30, 0.030, 0.030)), origin=Origin(xyz=(0.0, 0.345, 0.955)), material=aluminum, name="upper_rail_1")
    for suffix, x in (("end", -1.165), ("control_end", 1.165)):
        cabinet.visual(Box((0.030, 0.040, 0.065)), origin=Origin(xyz=(x, -0.345, 0.9125)), material=aluminum, name=f"upper_rail_post_0_{suffix}")
        cabinet.visual(Box((0.030, 0.040, 0.065)), origin=Origin(xyz=(x, 0.345, 0.9125)), material=aluminum, name=f"upper_rail_post_1_{suffix}")

    # A printed dial plate and temperature ticks are fixed to the control end
    # panel; the knob itself remains a separate rotating part below.
    cabinet.visual(Box((0.006, 0.018, 0.135)), origin=Origin(xyz=(1.202, -0.085, 0.48)), material=aluminum, name="dial_plate_0")
    cabinet.visual(Box((0.006, 0.018, 0.135)), origin=Origin(xyz=(1.202, 0.085, 0.48)), material=aluminum, name="dial_plate_1")
    cabinet.visual(Box((0.006, 0.160, 0.018)), origin=Origin(xyz=(1.202, 0.0, 0.405)), material=aluminum, name="dial_plate_2")
    cabinet.visual(Box((0.006, 0.160, 0.018)), origin=Origin(xyz=(1.202, 0.0, 0.555)), material=aluminum, name="dial_plate_3")
    for i, angle in enumerate((-60, -30, 0, 30, 60)):
        y = 0.065 * math.sin(math.radians(angle))
        z = 0.48 + 0.050 * math.cos(math.radians(angle))
        cabinet.visual(
            Box((0.007, 0.010, 0.030 if angle == 0 else 0.020)),
            origin=Origin(xyz=(1.202, y, z), rpy=(math.radians(90), 0.0, math.radians(angle))),
            material=label,
            name=f"temp_tick_{i}",
        )

    def add_lid(part_name: str, track: str) -> object:
        lid = model.part(part_name)
        runner_y = 0.345
        lid.visual(Box((1.22, 0.045, 0.035)), origin=Origin(xyz=(0.0, -0.365, 0.0)), material=aluminum, name="frame_rail_0")
        lid.visual(Box((1.22, 0.045, 0.035)), origin=Origin(xyz=(0.0, 0.365, 0.0)), material=aluminum, name="frame_rail_1")
        lid.visual(Box((0.045, 0.73, 0.035)), origin=Origin(xyz=(-0.610, 0.0, 0.0)), material=aluminum, name="frame_end_0")
        lid.visual(Box((0.045, 0.73, 0.035)), origin=Origin(xyz=(0.610, 0.0, 0.0)), material=aluminum, name="frame_end_1")
        lid.visual(Box((1.19, 0.69, 0.010)), origin=Origin(xyz=(0.0, 0.0, -0.003)), material=glass, name="glass_pane")
        lid.visual(Box((1.16, 0.026, 0.015)), origin=Origin(xyz=(0.0, -runner_y, -0.025)), material=aluminum, name="runner_0")
        lid.visual(Box((1.16, 0.026, 0.015)), origin=Origin(xyz=(0.0, runner_y, -0.025)), material=aluminum, name="runner_1")
        lid.visual(Box((0.18, 0.030, 0.030)), origin=Origin(xyz=(0.0, -0.400, 0.020)), material=dark, name="pull_grip_0")
        lid.visual(Box((0.18, 0.030, 0.030)), origin=Origin(xyz=(0.0, 0.400, 0.020)), material=dark, name="pull_grip_1")
        return lid

    lid_0 = add_lid("lid_0", "lower")
    lid_1 = add_lid("lid_1", "upper")

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-0.52, 0.0, 0.9125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.46),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=(0.52, 0.0, 1.0025)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.46),
    )

    knob = model.part("knob")
    knob.visual(Cylinder(radius=0.016, length=0.052), origin=Origin(xyz=(0.0, 0.0, 0.026)), material=aluminum, name="shaft")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.105,
                0.045,
                body_style="skirted",
                top_diameter=0.080,
                grip=KnobGrip(style="fluted", count=18, depth=0.003),
                indicator=KnobIndicator(style="line", mode="raised", depth=0.001, angle_deg=0.0),
                center=False,
            ),
            "temperature_control_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=dark,
        name="cap",
    )
    model.articulation(
        "cabinet_to_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(1.20, 0.0, 0.48), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    knob = object_model.get_part("knob")
    slide_0 = object_model.get_articulation("cabinet_to_lid_0")
    slide_1 = object_model.get_articulation("cabinet_to_lid_1")
    knob_joint = object_model.get_articulation("cabinet_to_knob")

    ctx.expect_gap(
        lid_0,
        cabinet,
        axis="z",
        positive_elem="runner_0",
        negative_elem="lower_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower lid runner sits on its rail",
    )
    ctx.expect_gap(
        lid_1,
        cabinet,
        axis="z",
        positive_elem="runner_0",
        negative_elem="upper_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper lid runner sits on its rail",
    )
    ctx.expect_overlap(lid_0, cabinet, axes="xy", elem_a="runner_0", elem_b="lower_rail_0", min_overlap=0.020, name="lower lid runner follows rail footprint")
    ctx.expect_overlap(lid_1, cabinet, axes="xy", elem_a="runner_0", elem_b="upper_rail_0", min_overlap=0.020, name="upper lid runner follows rail footprint")

    rest_0 = ctx.part_world_position(lid_0)
    rest_1 = ctx.part_world_position(lid_1)
    with ctx.pose({slide_0: 0.46, slide_1: 0.46, knob_joint: math.tau}):
        extended_0 = ctx.part_world_position(lid_0)
        extended_1 = ctx.part_world_position(lid_1)
        ctx.expect_within(lid_0, cabinet, axes="x", inner_elem="runner_0", outer_elem="lower_rail_0", margin=0.002, name="lower lid remains retained on rail")
        ctx.expect_within(lid_1, cabinet, axes="x", inner_elem="runner_0", outer_elem="upper_rail_0", margin=0.002, name="upper lid remains retained on rail")

    ctx.check(
        "lower lid slides toward control end",
        rest_0 is not None and extended_0 is not None and extended_0[0] > rest_0[0] + 0.40,
        details=f"rest={rest_0}, extended={extended_0}",
    )
    ctx.check(
        "upper lid slides toward opposite end",
        rest_1 is not None and extended_1 is not None and extended_1[0] < rest_1[0] - 0.40,
        details=f"rest={rest_1}, extended={extended_1}",
    )
    ctx.expect_gap(
        knob,
        cabinet,
        axis="x",
        positive_elem="shaft",
        negative_elem="control_end_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="knob shaft emerges from end wall",
    )

    return ctx.report()


object_model = build_object_model()
