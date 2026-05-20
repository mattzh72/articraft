from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PANEL_WIDTH = 2.95
PANEL_HALF = PANEL_WIDTH / 2.0
LEFT_CLOSED_X = -1.525
RIGHT_CLOSED_X = 1.525
SLIDE_TRAVEL = 1.25


def _add_box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_gate_panel(part, *, side: str, frame_material, mesh_material, wheel_material, latch_material):
    """Author one rectangular mesh leaf in its own local closed-pose frame."""
    frame_y = 0.08
    frame_t = 0.09
    bottom_z = 0.37
    top_z = 1.67
    stile_z = (bottom_z + top_z) / 2.0
    stile_h = top_z - bottom_z + frame_t

    _add_box(part, (PANEL_WIDTH, frame_y, frame_t), (0.0, 0.0, bottom_z), frame_material, "bottom_frame")
    _add_box(part, (PANEL_WIDTH, frame_y, frame_t), (0.0, 0.0, top_z), frame_material, "top_frame")

    outer_x = -PANEL_HALF + frame_t / 2.0 if side == "left" else PANEL_HALF - frame_t / 2.0
    inner_x = PANEL_HALF - frame_t / 2.0 if side == "left" else -PANEL_HALF + frame_t / 2.0
    _add_box(part, (frame_t, frame_y, stile_h), (outer_x, 0.0, stile_z), frame_material, "outer_stile")
    _add_box(part, (frame_t, frame_y, stile_h), (inner_x, 0.0, stile_z), frame_material, "inner_stile")

    # Welded rectangular steel-mesh infill: slender bars touch the perimeter frame.
    mesh_bottom = bottom_z + frame_t / 2.0
    mesh_top = top_z - frame_t / 2.0
    mesh_h = mesh_top - mesh_bottom
    mesh_center_z = (mesh_top + mesh_bottom) / 2.0
    inner_span = PANEL_WIDTH - 2.0 * frame_t
    for i, x in enumerate([-1.08, -0.72, -0.36, 0.0, 0.36, 0.72, 1.08]):
        _add_box(part, (0.024, 0.030, mesh_h), (x, 0.0, mesh_center_z), mesh_material, f"mesh_vertical_{i}")
    for i, z in enumerate([0.58, 0.79, 1.00, 1.21, 1.42]):
        _add_box(part, (inner_span, 0.030, 0.024), (0.0, 0.0, z), mesh_material, f"mesh_horizontal_{i}")

    # Two flanged load wheels sit on the raised rail rib and are tied back into the lower frame.
    for i, x in enumerate([-0.95, 0.95]):
        _add_y_cylinder(part, 0.09, 0.075, (x, 0.0, 0.215), wheel_material, f"wheel_{i}")
        _add_y_cylinder(part, 0.018, 0.160, (x, 0.0, 0.215), frame_material, f"wheel_axle_{i}")
        _add_box(part, (0.11, 0.020, 0.290), (x, -0.065, 0.305), frame_material, f"wheel_fork_front_{i}")
        _add_box(part, (0.11, 0.020, 0.290), (x, 0.065, 0.305), frame_material, f"wheel_fork_rear_{i}")
        _add_box(part, (0.14, 0.170, 0.045), (x, 0.0, 0.425), frame_material, f"wheel_fork_bridge_{i}")

    # Guide rollers project upward into the stationary upper channel without intersecting it.
    for i, x in enumerate([-0.95, 0.95]):
        _add_box(part, (0.060, 0.045, 0.180), (x, 0.0, 1.770), frame_material, f"top_guide_stem_{i}")
        part.visual(
            Cylinder(radius=0.055, length=0.080),
            origin=Origin(xyz=(x, 0.0, 1.865)),
            material=wheel_material,
            name=f"top_roller_{i}",
        )

    # Center latch hardware on the meeting stile, offset toward the roadside face.
    latch_x = inner_x + (0.035 if side == "left" else -0.035)
    _add_box(part, (0.085, 0.040, 0.095), (latch_x, -0.040, 1.045), latch_material, "latch_striker")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bi_parting_sliding_road_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.67, 1.0))
    dark_mesh = model.material("dark_welded_mesh", rgba=(0.18, 0.20, 0.21, 1.0))
    rail_black = model.material("blackened_track", rgba=(0.06, 0.065, 0.07, 1.0))
    rubber = model.material("dark_rubber_wheels", rgba=(0.025, 0.025, 0.022, 1.0))
    latch_yellow = model.material("yellow_latch_hardware", rgba=(0.95, 0.74, 0.10, 1.0))

    frame = model.part("track_frame")

    # Ground rail runs past both leaves so they remain supported at full travel.
    _add_box(frame, (9.50, 0.110, 0.070), (0.0, 0.0, 0.035), rail_black, "bottom_rail")
    _add_box(frame, (9.45, 0.035, 0.055), (0.0, 0.0, 0.0975), galvanized, "raised_rail_rib")

    # End stanchions and the center latch post connect the lower rail to the upper guide track.
    _add_box(frame, (0.160, 0.300, 2.050), (-4.65, 0.0, 1.025), galvanized, "end_post_0")
    _add_box(frame, (0.160, 0.300, 2.050), (4.65, 0.0, 1.025), galvanized, "end_post_1")
    _add_box(frame, (0.080, 0.080, 1.900), (0.0, -0.140, 0.950), galvanized, "center_latch_post")
    _add_box(frame, (0.210, 0.035, 0.220), (0.0, -0.0875, 1.045), latch_yellow, "latch_keeper")

    # Upper C-channel: a cap with two cheeks leaves a clear slot for the panel guide rollers.
    _add_box(frame, (9.50, 0.320, 0.060), (0.0, 0.0, 2.020), galvanized, "upper_track_cap")
    _add_box(frame, (9.50, 0.050, 0.170), (0.0, -0.160, 1.905), galvanized, "upper_track_front")
    _add_box(frame, (9.50, 0.050, 0.170), (0.0, 0.160, 1.905), galvanized, "upper_track_rear")

    left_panel = model.part("panel_0")
    _add_gate_panel(
        left_panel,
        side="left",
        frame_material=galvanized,
        mesh_material=dark_mesh,
        wheel_material=rubber,
        latch_material=latch_yellow,
    )

    right_panel = model.part("panel_1")
    _add_gate_panel(
        right_panel,
        side="right",
        frame_material=galvanized,
        mesh_material=dark_mesh,
        wheel_material=rubber,
        latch_material=latch_yellow,
    )

    model.articulation(
        "slide_0",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_panel,
        origin=Origin(xyz=(LEFT_CLOSED_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "slide_1",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_panel,
        origin=Origin(xyz=(RIGHT_CLOSED_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("track_frame")
    left_panel = object_model.get_part("panel_0")
    right_panel = object_model.get_part("panel_1")
    left_slide = object_model.get_articulation("slide_0")
    right_slide = object_model.get_articulation("slide_1")

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            frame,
            left_panel,
            axis="x",
            positive_elem="center_latch_post",
            negative_elem="inner_stile",
            min_gap=0.0,
            max_gap=0.020,
            name="left leaf closes to the center latch post",
        )
        ctx.expect_gap(
            right_panel,
            frame,
            axis="x",
            positive_elem="inner_stile",
            negative_elem="center_latch_post",
            min_gap=0.0,
            max_gap=0.020,
            name="right leaf closes to the center latch post",
        )
        ctx.expect_gap(
            left_panel,
            frame,
            axis="z",
            positive_elem="wheel_0",
            negative_elem="raised_rail_rib",
            max_gap=0.002,
            max_penetration=0.0,
            name="left panel wheel sits on the raised rail",
        )
        ctx.expect_gap(
            right_panel,
            frame,
            axis="z",
            positive_elem="wheel_0",
            negative_elem="raised_rail_rib",
            max_gap=0.002,
            max_penetration=0.0,
            name="right panel wheel sits on the raised rail",
        )

    left_rest = ctx.part_world_position(left_panel)
    right_rest = ctx.part_world_position(right_panel)
    with ctx.pose({left_slide: SLIDE_TRAVEL, right_slide: SLIDE_TRAVEL}):
        ctx.expect_gap(
            right_panel,
            left_panel,
            axis="x",
            positive_elem="inner_stile",
            negative_elem="inner_stile",
            min_gap=2.45,
            name="open leaves slide apart from the center",
        )
        ctx.expect_overlap(
            left_panel,
            frame,
            axes="x",
            elem_a="wheel_1",
            elem_b="raised_rail_rib",
            min_overlap=0.05,
            name="left leaf remains supported on the rail at full travel",
        )
        ctx.expect_overlap(
            right_panel,
            frame,
            axes="x",
            elem_a="wheel_1",
            elem_b="raised_rail_rib",
            min_overlap=0.05,
            name="right leaf remains supported on the rail at full travel",
        )
        left_open = ctx.part_world_position(left_panel)
        right_open = ctx.part_world_position(right_panel)

    ctx.check(
        "prismatic joints move the two leaves in opposite directions",
        left_rest is not None
        and right_rest is not None
        and left_open is not None
        and right_open is not None
        and left_open[0] < left_rest[0] - 1.0
        and right_open[0] > right_rest[0] + 1.0,
        details=f"left_rest={left_rest}, left_open={left_open}, right_rest={right_rest}, right_open={right_open}",
    )

    return ctx.report()


object_model = build_object_model()
