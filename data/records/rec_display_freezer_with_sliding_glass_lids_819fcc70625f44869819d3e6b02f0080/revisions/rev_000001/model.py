from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cap_display_freezer")

    cabinet_white = model.material("insulated_warm_white", rgba=(0.92, 0.90, 0.84, 1.0))
    plinth_mat = model.material("dark_rubber_plinth", rgba=(0.08, 0.08, 0.075, 1.0))
    liner_mat = model.material("cool_blue_liner", rgba=(0.62, 0.78, 0.86, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    glass_mat = model.material("slightly_blue_glass", rgba=(0.60, 0.82, 0.95, 0.34))
    black_mat = model.material("matte_black_panel", rgba=(0.03, 0.035, 0.04, 1.0))
    lcd_mat = model.material("green_lcd", rgba=(0.18, 0.80, 0.52, 1.0))
    knob_mat = model.material("black_plastic_knob", rgba=(0.015, 0.015, 0.016, 1.0))

    # Grocery-store chest/end-cap freezer proportions: long enough for two
    # top sliders, low enough to browse over, with a shallow continuous plinth.
    cabinet = model.part("cabinet")
    cabinet.visual(Box((2.04, 0.94, 0.09)), origin=Origin(xyz=(0.0, 0.0, 0.045)), material=plinth_mat, name="plinth")
    cabinet.visual(Box((1.92, 0.82, 0.13)), origin=Origin(xyz=(0.0, 0.0, 0.155)), material=cabinet_white, name="insulated_floor")
    cabinet.visual(Box((1.92, 0.075, 0.66)), origin=Origin(xyz=(0.0, -0.4125, 0.50)), material=cabinet_white, name="side_wall_0")
    cabinet.visual(Box((1.92, 0.075, 0.66)), origin=Origin(xyz=(0.0, 0.4125, 0.50)), material=cabinet_white, name="side_wall_1")
    cabinet.visual(Box((0.075, 0.90, 0.66)), origin=Origin(xyz=(-0.9625, 0.0, 0.50)), material=cabinet_white, name="end_wall_0")
    cabinet.visual(Box((0.075, 0.90, 0.66)), origin=Origin(xyz=(0.9625, 0.0, 0.50)), material=cabinet_white, name="end_wall")
    cabinet.visual(Box((1.70, 0.60, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.235)), material=liner_mat, name="recessed_liner_floor")
    cabinet.visual(Box((1.74, 0.030, 0.50)), origin=Origin(xyz=(0.0, -0.362, 0.485)), material=liner_mat, name="inner_liner_0")
    cabinet.visual(Box((1.74, 0.030, 0.50)), origin=Origin(xyz=(0.0, 0.362, 0.485)), material=liner_mat, name="inner_liner_1")
    cabinet.visual(Box((0.030, 0.66, 0.50)), origin=Origin(xyz=(-0.912, 0.0, 0.485)), material=liner_mat, name="inner_end_liner_0")
    cabinet.visual(Box((0.030, 0.66, 0.50)), origin=Origin(xyz=(0.912, 0.0, 0.485)), material=liner_mat, name="inner_end_liner_1")
    cabinet.visual(Box((1.98, 0.055, 0.055)), origin=Origin(xyz=(0.0, -0.4425, 0.84)), material=metal_mat, name="top_rail_0")
    cabinet.visual(Box((1.98, 0.055, 0.055)), origin=Origin(xyz=(0.0, 0.4425, 0.84)), material=metal_mat, name="top_rail_1")
    cabinet.visual(Box((0.055, 0.89, 0.055)), origin=Origin(xyz=(-0.9925, 0.0, 0.84)), material=metal_mat, name="top_end_rail_0")
    cabinet.visual(Box((0.055, 0.89, 0.055)), origin=Origin(xyz=(0.9925, 0.0, 0.84)), material=metal_mat, name="top_end_rail_1")
    # Two shallow track lips make the sliding tracks visible without blocking the
    # glass panels.
    cabinet.visual(Box((1.90, 0.095, 0.030)), origin=Origin(xyz=(0.0, -0.3775, 0.875)), material=metal_mat, name="inner_track_0")
    cabinet.visual(Box((1.90, 0.095, 0.030)), origin=Origin(xyz=(0.0, 0.3775, 0.875)), material=metal_mat, name="inner_track_1")

    # Separate fixed panel on the +X end face.  Its part frame is the panel's
    # mounting plane; the visual body extends outward along +X.
    control_panel = model.part("control_panel")
    control_panel.visual(Box((0.024, 0.46, 0.25)), origin=Origin(xyz=(0.012, 0.0, 0.0)), material=black_mat, name="panel_plate")
    control_panel.visual(Box((0.006, 0.18, 0.055)), origin=Origin(xyz=(0.027, -0.105, 0.052)), material=lcd_mat, name="temperature_display")
    control_panel.visual(Box((0.006, 0.052, 0.022)), origin=Origin(xyz=(0.0265, 0.105, 0.073)), material=metal_mat, name="scale_mark_0")
    control_panel.visual(Box((0.006, 0.045, 0.012)), origin=Origin(xyz=(0.0265, 0.105, 0.036)), material=metal_mat, name="scale_mark_1")
    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(xyz=(1.000, 0.0, 0.54)),
    )

    # The rotary temperature control is its own moving part.  It includes a
    # visible short shaft plus a detailed appliance-style cap.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.135,
            0.045,
            body_style="skirted",
            top_diameter=0.105,
            grip=KnobGrip(style="fluted", count=22, depth=0.003),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "temperature_knob",
    )
    knob = model.part("knob")
    knob.visual(Cylinder(radius=0.018, length=0.040), origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=metal_mat, name="short_shaft")
    knob.visual(knob_mesh, origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=knob_mat, name="dial_cap")
    model.articulation(
        "panel_to_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=knob,
        origin=Origin(xyz=(0.024, 0.105, -0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )

    def add_lid(
        name: str,
        rest_x: float,
        z: float,
        direction: float,
        runner_drop: float,
        runner_y: float,
        raised_runner: bool = False,
    ) -> None:
        lid = model.part(name)
        lid.visual(Box((0.91, 0.695, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=glass_mat, name="glass_pane")
        lid.visual(Box((0.94, 0.022, 0.035)), origin=Origin(xyz=(0.0, -0.358, 0.003)), material=metal_mat, name="side_frame_0")
        lid.visual(Box((0.94, 0.022, 0.035)), origin=Origin(xyz=(0.0, 0.358, 0.003)), material=metal_mat, name="side_frame_1")
        lid.visual(Box((0.024, 0.718, 0.035)), origin=Origin(xyz=(-0.470, 0.0, 0.003)), material=metal_mat, name="end_frame_0")
        lid.visual(Box((0.024, 0.718, 0.035)), origin=Origin(xyz=(0.470, 0.0, 0.003)), material=metal_mat, name="end_frame_1")
        lid.visual(Box((0.82, 0.018, runner_drop)), origin=Origin(xyz=(0.0, -runner_y, -runner_drop / 2.0)), material=metal_mat, name="track_runner_0")
        lid.visual(Box((0.82, 0.018, runner_drop)), origin=Origin(xyz=(0.0, runner_y, -runner_drop / 2.0)), material=metal_mat, name="track_runner_1")
        if raised_runner:
            lid.visual(Box((0.82, 0.065, 0.030)), origin=Origin(xyz=(0.0, -0.384, 0.005)), material=metal_mat, name="runner_bridge_0")
            lid.visual(Box((0.82, 0.065, 0.030)), origin=Origin(xyz=(0.0, 0.384, 0.005)), material=metal_mat, name="runner_bridge_1")
        # Low-profile pull strip: visible to shoppers, but shallow enough for
        # the opposite glass panel to pass over it on the staggered track.
        lid.visual(Box((0.22, 0.040, 0.012)), origin=Origin(xyz=(-0.34 * direction, 0.0, 0.014)), material=metal_mat, name="pull_handle")
        lid.visual(Box((0.030, 0.028, 0.010)), origin=Origin(xyz=(-0.42 * direction, 0.0, 0.014)), material=metal_mat, name="handle_post_0")
        lid.visual(Box((0.030, 0.028, 0.010)), origin=Origin(xyz=(-0.26 * direction, 0.0, 0.014)), material=metal_mat, name="handle_post_1")
        model.articulation(
            f"cabinet_to_{name}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=lid,
            origin=Origin(xyz=(rest_x, 0.0, z)),
            axis=(direction, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.72),
        )

    add_lid("lid_0", rest_x=-0.505, z=0.910, direction=1.0, runner_drop=0.020, runner_y=0.342)
    add_lid("lid_1", rest_x=0.505, z=0.950, direction=-1.0, runner_drop=0.060, runner_y=0.410, raised_runner=True)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    control_panel = object_model.get_part("control_panel")
    knob = object_model.get_part("knob")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")

    panel_joint = object_model.get_articulation("cabinet_to_control_panel")
    knob_joint = object_model.get_articulation("panel_to_knob")
    lid_0_joint = object_model.get_articulation("cabinet_to_lid_0")
    lid_1_joint = object_model.get_articulation("cabinet_to_lid_1")

    ctx.check(
        "control panel is a separate fixed mounted part",
        panel_joint.articulation_type == ArticulationType.FIXED
        and panel_joint.parent == "cabinet"
        and panel_joint.child == "control_panel",
    )
    ctx.expect_gap(
        control_panel,
        cabinet,
        axis="x",
        positive_elem="panel_plate",
        negative_elem="end_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="control panel seats on end wall without merging",
    )
    ctx.expect_overlap(
        control_panel,
        cabinet,
        axes="yz",
        elem_a="panel_plate",
        elem_b="end_wall",
        min_overlap=0.20,
        name="control panel visibly occupies the end wall",
    )

    ctx.check(
        "knob is a continuous rotary control",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_joint.axis) == (1.0, 0.0, 0.0),
    )
    ctx.expect_gap(
        knob,
        control_panel,
        axis="x",
        positive_elem="short_shaft",
        negative_elem="panel_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="temperature knob shaft seats on panel",
    )
    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: 1.75}):
        knob_rotated = ctx.part_world_position(knob)
    ctx.check(
        "knob rotation keeps shaft center on the panel",
        knob_rest is not None and knob_rotated is not None and knob_rest == knob_rotated,
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    for lid, joint, runner in (
        (lid_0, lid_0_joint, "track_runner_0"),
        (lid_1, lid_1_joint, "track_runner_0"),
    ):
        ctx.check(
            f"{lid.name} uses a prismatic rail joint",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 0.5,
        )
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem=runner,
            negative_elem="inner_track_0",
            max_gap=0.001,
            max_penetration=0.0002,
            name=f"{lid.name} runner rides on rail",
        )
        ctx.expect_within(
            lid,
            cabinet,
            axes="x",
            inner_elem=runner,
            outer_elem="inner_track_0",
            margin=0.0,
            name=f"{lid.name} runner retained in rail at rest",
        )

    lid_0_rest = ctx.part_world_position(lid_0)
    with ctx.pose({lid_0_joint: lid_0_joint.motion_limits.upper}):
        lid_0_extended = ctx.part_world_position(lid_0)
        ctx.expect_within(
            lid_0,
            cabinet,
            axes="x",
            inner_elem="track_runner_0",
            outer_elem="inner_track_0",
            margin=0.0,
            name="lid_0 runner retained at full travel",
        )
    ctx.check(
        "lid_0 slides along the rail",
        lid_0_rest is not None and lid_0_extended is not None and lid_0_extended[0] > lid_0_rest[0] + 0.50,
        details=f"rest={lid_0_rest}, extended={lid_0_extended}",
    )

    lid_1_rest = ctx.part_world_position(lid_1)
    with ctx.pose({lid_1_joint: lid_1_joint.motion_limits.upper}):
        lid_1_extended = ctx.part_world_position(lid_1)
        ctx.expect_within(
            lid_1,
            cabinet,
            axes="x",
            inner_elem="track_runner_0",
            outer_elem="inner_track_0",
            margin=0.0,
            name="lid_1 runner retained at full travel",
        )
        ctx.expect_gap(
            lid_1,
            lid_0,
            axis="z",
            positive_elem="glass_pane",
            negative_elem="pull_handle",
            min_gap=0.004,
            name="raised lid clears the low-profile lower handle",
        )
    ctx.check(
        "lid_1 slides along the opposite rail",
        lid_1_rest is not None and lid_1_extended is not None and lid_1_extended[0] < lid_1_rest[0] - 0.50,
        details=f"rest={lid_1_rest}, extended={lid_1_extended}",
    )

    return ctx.report()


object_model = build_object_model()
