from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_service_access_panel")

    anodized = Material("dark_hard_anodized_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    panel_paint = Material("warm_powder_coated_panel", rgba=(0.73, 0.75, 0.74, 1.0))
    black = Material("black_controlled_reveal", rgba=(0.01, 0.012, 0.012, 1.0))
    steel = Material("brushed_stainless_hardware", rgba=(0.66, 0.66, 0.62, 1.0))
    datum = Material("ground_datum_faces", rgba=(0.82, 0.84, 0.78, 1.0))
    blue = Material("blue_adjustment_marks", rgba=(0.05, 0.22, 0.85, 1.0))
    green = Material("green_locked_index", rgba=(0.05, 0.72, 0.18, 1.0))
    red = Material("red_release_index", rgba=(0.90, 0.06, 0.04, 1.0))

    frame = model.part("frame")

    # Fixed machine opening: four connected rails leave a true clear aperture.
    frame.visual(Box((0.560, 0.035, 0.052)), origin=Origin(xyz=(0.0, 0.0, 0.176)), material=anodized, name="top_rail")
    frame.visual(Box((0.560, 0.035, 0.052)), origin=Origin(xyz=(0.0, 0.0, -0.176)), material=anodized, name="bottom_rail")
    frame.visual(Box((0.052, 0.035, 0.405)), origin=Origin(xyz=(-0.254, 0.0, 0.0)), material=anodized, name="hinge_rail")
    frame.visual(Box((0.052, 0.035, 0.405)), origin=Origin(xyz=(0.254, 0.0, 0.0)), material=anodized, name="latch_rail")

    # Dark reveal strips explicitly show the controlled perimeter gap around the
    # panel rather than hiding the clearance in a solid face.
    frame.visual(Box((0.456, 0.002, 0.004)), origin=Origin(xyz=(0.0, -0.0185, 0.1515)), material=black, name="top_reveal")
    frame.visual(Box((0.456, 0.002, 0.004)), origin=Origin(xyz=(0.0, -0.0185, -0.1515)), material=black, name="bottom_reveal")
    frame.visual(Box((0.004, 0.002, 0.304)), origin=Origin(xyz=(-0.228, -0.0185, 0.0)), material=black, name="hinge_reveal")
    frame.visual(Box((0.004, 0.002, 0.304)), origin=Origin(xyz=(0.228, -0.0185, 0.0)), material=black, name="latch_reveal")

    # Precision datum and calibration surfaces on the stationary side.
    frame.visual(Box((0.035, 0.003, 0.045)), origin=Origin(xyz=(0.254, -0.0190, 0.096)), material=datum, name="upper_datum_land")
    frame.visual(Box((0.035, 0.003, 0.045)), origin=Origin(xyz=(0.254, -0.0190, -0.096)), material=datum, name="lower_datum_land")
    frame.visual(Box((0.125, 0.003, 0.018)), origin=Origin(xyz=(0.000, -0.0190, -0.176)), material=datum, name="bottom_datum_land")
    frame.visual(Box((0.040, 0.0015, 0.052)), origin=Origin(xyz=(0.260, -0.01675, 0.000)), material=steel, name="latch_strike_face")

    # Frame-side fixed hinge leaf and alternating knuckles.
    frame.visual(Box((0.032, 0.0042, 0.318)), origin=Origin(xyz=(-0.254, -0.0193, 0.0)), material=steel, name="fixed_hinge_leaf")
    for idx, zc in enumerate((-0.1175, 0.0, 0.1175)):
        frame.visual(
            Cylinder(radius=0.0080, length=0.055),
            origin=Origin(xyz=(-0.238, -0.0290, zc)),
            material=steel,
            name=f"fixed_knuckle_{idx}",
        )

    # Latch-side lock/release datum marks on the fixed rail.
    frame.visual(Box((0.020, 0.002, 0.004)), origin=Origin(xyz=(0.277, -0.0180, 0.034)), material=green, name="locked_index")
    frame.visual(Box((0.004, 0.002, 0.020)), origin=Origin(xyz=(0.277, -0.0180, -0.034)), material=red, name="release_index")
    for idx, zc in enumerate((-0.060, -0.030, 0.030, 0.060)):
        frame.visual(Box((0.014, 0.002, 0.0025)), origin=Origin(xyz=(0.275, -0.0180, zc)), material=blue, name=f"frame_index_mark_{idx}")

    service_panel = model.part("service_panel")

    # The panel part frame is on the hinge axis.  The panel extends in +X to the
    # latch side, matching a real hinged service cover.
    service_panel.visual(Box((0.434, 0.010, 0.284)), origin=Origin(xyz=(0.238, 0.0, 0.0)), material=panel_paint, name="panel_plate")
    service_panel.visual(Box((0.330, 0.0015, 0.0025)), origin=Origin(xyz=(0.246, -0.0056, 0.118)), material=black, name="upper_gap_line")
    service_panel.visual(Box((0.330, 0.0015, 0.0025)), origin=Origin(xyz=(0.246, -0.0056, -0.118)), material=black, name="lower_gap_line")
    service_panel.visual(Box((0.0025, 0.0015, 0.230)), origin=Origin(xyz=(0.038, -0.0056, 0.0)), material=black, name="hinge_gap_line")
    service_panel.visual(Box((0.0025, 0.0015, 0.230)), origin=Origin(xyz=(0.438, -0.0056, 0.0)), material=black, name="latch_edge_datum")

    # Moving hinge leaf and two intermediate knuckles.
    for idx, zc in enumerate((-0.0585, 0.0585)):
        service_panel.visual(
            Cylinder(radius=0.0076, length=0.047),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=steel,
            name=f"moving_knuckle_{idx}",
        )
        service_panel.visual(
            Box((0.045, 0.0040, 0.044)),
            origin=Origin(xyz=(0.021, 0.0, zc)),
            material=steel,
            name=f"moving_hinge_leaf_{idx}",
        )

    # Repeatable alignment features: pads, witness marks, and adjustment screws.
    service_panel.visual(Box((0.052, 0.002, 0.020)), origin=Origin(xyz=(0.395, -0.0060, 0.090)), material=datum, name="upper_panel_datum")
    service_panel.visual(Box((0.052, 0.002, 0.020)), origin=Origin(xyz=(0.395, -0.0060, -0.090)), material=datum, name="lower_panel_datum")
    service_panel.visual(Box((0.080, 0.002, 0.014)), origin=Origin(xyz=(0.238, -0.0060, -0.129)), material=datum, name="bottom_panel_datum")

    screw_positions = ((0.105, 0.090), (0.315, 0.090), (0.105, -0.090), (0.315, -0.090))
    for idx, (x, z) in enumerate(screw_positions):
        service_panel.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, -0.0068, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"adjuster_screw_{idx}",
        )
        service_panel.visual(
            Box((0.017, 0.001, 0.0022)),
            origin=Origin(xyz=(x, -0.0093, z)),
            material=black,
            name=f"adjuster_slot_{idx}",
        )
        service_panel.visual(
            Box((0.0025, 0.0012, 0.013)),
            origin=Origin(xyz=(x + 0.018, -0.0055, z)),
            material=blue,
            name=f"adjuster_witness_{idx}",
        )

    for idx, zc in enumerate((-0.060, -0.030, 0.030, 0.060)):
        service_panel.visual(Box((0.012, 0.0015, 0.0025)), origin=Origin(xyz=(0.436, -0.0062, zc)), material=blue, name=f"panel_index_mark_{idx}")

    latch = model.part("latch")

    # A quarter-turn latch: front knob and handle, shaft through the panel, and a
    # rear cam visibly engaging the latch-side strike.
    latch.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="latch_knob",
    )
    latch.visual(
        Box((0.052, 0.004, 0.010)),
        origin=Origin(xyz=(0.023, -0.025, 0.0)),
        material=steel,
        name="latch_handle",
    )
    latch.visual(
        Cylinder(radius=0.0050, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="latch_shaft",
    )
    latch.visual(
        Box((0.090, 0.004, 0.018)),
        origin=Origin(xyz=(0.045, 0.0085, 0.0)),
        material=steel,
        name="latch_cam",
    )
    latch.visual(
        Box((0.020, 0.0015, 0.004)),
        origin=Origin(xyz=(0.022, -0.0275, 0.0)),
        material=green,
        name="latch_pointer",
    )

    model.articulation(
        "frame_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=service_panel,
        origin=Origin(xyz=(-0.238, -0.029, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.70),
    )

    model.articulation(
        "service_panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=service_panel,
        child=latch,
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("service_panel")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("frame_to_service_panel")
    latch_turn = object_model.get_articulation("service_panel_to_latch")

    ctx.allow_overlap(
        panel,
        latch,
        elem_a="panel_plate",
        elem_b="latch_shaft",
        reason="The quarter-turn latch shaft is intentionally captured through the service panel bushing.",
    )
    ctx.expect_within(
        latch,
        panel,
        axes="xz",
        inner_elem="latch_shaft",
        outer_elem="panel_plate",
        margin=0.0,
        name="latch shaft is centered inside the panel footprint",
    )
    ctx.expect_overlap(
        latch,
        panel,
        axes="y",
        elem_a="latch_shaft",
        elem_b="panel_plate",
        min_overlap=0.009,
        name="latch shaft passes fully through the panel thickness",
    )

    ctx.expect_gap(
        frame,
        panel,
        axis="y",
        positive_elem="latch_strike_face",
        negative_elem="panel_plate",
        min_gap=0.003,
        max_gap=0.010,
        name="closed panel keeps a controlled front standoff from the frame",
    )
    ctx.expect_overlap(
        latch,
        frame,
        axes="xz",
        elem_a="latch_cam",
        elem_b="latch_strike_face",
        min_overlap=0.010,
        name="closed latch cam projects behind the latch-side strike",
    )
    ctx.expect_gap(
        frame,
        latch,
        axis="y",
        positive_elem="latch_strike_face",
        negative_elem="latch_cam",
        min_gap=0.0005,
        max_gap=0.004,
        name="closed latch cam has a small controlled strike clearance",
    )

    panel_box = ctx.part_element_world_aabb(panel, elem="panel_plate")
    if panel_box is not None:
        panel_min, panel_max = panel_box
        ctx.check(
            "panel reveal is symmetric inside the framed opening",
            panel_min[0] > -0.222 and panel_max[0] < 0.222 and panel_min[2] > -0.146 and panel_max[2] < 0.146,
            details=f"panel_min={panel_min}, panel_max={panel_max}",
        )

    closed_edge = ctx.part_element_world_aabb(panel, elem="latch_edge_datum")
    with ctx.pose({door_hinge: 1.20}):
        opened_edge = ctx.part_element_world_aabb(panel, elem="latch_edge_datum")
    ctx.check(
        "door hinge swings the latch side outward",
        closed_edge is not None
        and opened_edge is not None
        and opened_edge[0][1] < closed_edge[0][1] - 0.25,
        details=f"closed_edge={closed_edge}, opened_edge={opened_edge}",
    )

    with ctx.pose({latch_turn: math.pi / 2.0}):
        ctx.expect_gap(
            frame,
            latch,
            axis="x",
            positive_elem="latch_strike_face",
            negative_elem="latch_cam",
            min_gap=0.030,
            name="rotated latch cam clears the strike in release position",
        )

    return ctx.report()


object_model = build_object_model()
