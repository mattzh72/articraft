from __future__ import annotations

import math

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
    model = ArticulatedObject(name="plantation_shutter")

    painted_wood = model.material("painted_wood", rgba=(0.92, 0.89, 0.80, 1.0))
    shadow = model.material("recess_shadow", rgba=(0.12, 0.11, 0.10, 1.0))
    brass = model.material("brass_hinge", rgba=(0.70, 0.50, 0.22, 1.0))

    frame = model.part("outer_frame")
    # Fixed jambs and rails of the surrounding window frame.
    frame.visual(Box((0.070, 0.070, 1.650)), origin=Origin(xyz=(-0.450, 0.000, 0.825)), material=painted_wood, name="left_jamb")
    frame.visual(Box((0.070, 0.070, 1.650)), origin=Origin(xyz=(0.450, 0.000, 0.825)), material=painted_wood, name="right_jamb")
    frame.visual(Box((0.900, 0.070, 0.070)), origin=Origin(xyz=(0.000, 0.000, 1.615)), material=painted_wood, name="top_rail")
    frame.visual(Box((0.900, 0.070, 0.070)), origin=Origin(xyz=(0.000, 0.000, 0.035)), material=painted_wood, name="sill_rail")
    # A darker reveal behind the shutter makes the open louver field read as a window opening.
    frame.visual(Box((0.850, 0.012, 1.585)), origin=Origin(xyz=(0.000, 0.041, 0.825)), material=shadow, name="window_reveal")
    # Thin stops on the fixed frame, intentionally part of the same fixed assembly.
    frame.visual(Box((0.030, 0.020, 1.475)), origin=Origin(xyz=(-0.405, -0.042, 0.825)), material=painted_wood, name="left_stop")
    frame.visual(Box((0.030, 0.020, 1.475)), origin=Origin(xyz=(0.405, -0.042, 0.825)), material=painted_wood, name="right_stop")
    frame.visual(Box((0.740, 0.020, 0.030)), origin=Origin(xyz=(0.000, -0.042, 1.568)), material=painted_wood, name="top_stop")
    frame.visual(Box((0.740, 0.020, 0.030)), origin=Origin(xyz=(0.000, -0.042, 0.085)), material=painted_wood, name="bottom_stop")

    # Alternating fixed hinge knuckles mounted to the left jamb.
    hinge_x = -0.365
    hinge_y = -0.055
    for i, zc in enumerate((0.340, 0.825, 1.310)):
        frame.visual(Box((0.060, 0.012, 0.145)), origin=Origin(xyz=(-0.392, -0.041, zc)), material=brass, name=f"frame_hinge_leaf_{i}")
        frame.visual(Cylinder(radius=0.012, length=0.045), origin=Origin(xyz=(hinge_x, hinge_y, zc - 0.052)), material=brass, name=f"frame_knuckle_{i}_0")
        frame.visual(Cylinder(radius=0.012, length=0.045), origin=Origin(xyz=(hinge_x, hinge_y, zc + 0.052)), material=brass, name=f"frame_knuckle_{i}_1")

    panel = model.part("panel")
    # The panel frame is authored in a hinge-line frame.  Its closed pose places
    # the louvered leaf neatly inside the fixed frame with a small reveal gap.
    panel_y = 0.055
    panel.visual(Box((0.060, 0.045, 1.420)), origin=Origin(xyz=(0.045, panel_y, 0.000)), material=painted_wood, name="hinge_stile")
    panel.visual(Box((0.060, 0.045, 1.420)), origin=Origin(xyz=(0.695, panel_y, 0.000)), material=painted_wood, name="free_stile")
    panel.visual(Box((0.710, 0.045, 0.070)), origin=Origin(xyz=(0.370, panel_y, 0.675)), material=painted_wood, name="top_rail")
    panel.visual(Box((0.710, 0.045, 0.070)), origin=Origin(xyz=(0.370, panel_y, -0.675)), material=painted_wood, name="bottom_rail")
    panel.visual(Box((0.022, 0.018, 1.230)), origin=Origin(xyz=(0.085, 0.024, 0.000)), material=painted_wood, name="left_louver_stop")
    panel.visual(Box((0.022, 0.018, 1.230)), origin=Origin(xyz=(0.655, 0.024, 0.000)), material=painted_wood, name="right_louver_stop")
    for i, zc in enumerate((-0.485, 0.000, 0.485)):
        panel.visual(Box((0.060, 0.018, 0.145)), origin=Origin(xyz=(0.040, 0.024, zc)), material=brass, name=f"panel_hinge_leaf_{i}")
        panel.visual(Box((0.032, 0.011, 0.049)), origin=Origin(xyz=(0.020, 0.0115, zc)), material=brass, name=f"panel_leaf_curl_{i}")
        panel.visual(Cylinder(radius=0.012, length=0.055), origin=Origin(xyz=(0.000, 0.000, zc)), material=brass, name=f"panel_knuckle_{i}")

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.825)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    tilt_rod = model.part("tilt_rod")
    tilt_rod.visual(Box((0.024, 0.014, 1.150)), origin=Origin(), material=painted_wood, name="rod")
    tilt_rod.visual(Box((0.034, 0.018, 0.050)), origin=Origin(xyz=(0.0, -0.002, 0.575)), material=painted_wood, name="top_grip")
    tilt_rod.visual(Box((0.034, 0.018, 0.050)), origin=Origin(xyz=(0.0, -0.002, -0.575)), material=painted_wood, name="bottom_grip")
    model.articulation(
        "panel_to_tilt_rod",
        ArticulationType.PRISMATIC,
        parent=panel,
        child=tilt_rod,
        origin=Origin(xyz=(0.370, 0.020, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.12, lower=-0.035, upper=0.035),
    )

    louver_centers = [-0.520, -0.390, -0.260, -0.130, 0.000, 0.130, 0.260, 0.390, 0.520]
    for i, zc in enumerate(louver_centers):
        louver = model.part(f"louver_{i}")
        louver.visual(Box((0.550, 0.012, 0.095)), origin=Origin(), material=painted_wood, name="blade")
        louver.visual(
            Cylinder(radius=0.005, length=0.590),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=painted_wood,
            name="pivot_pin",
        )
        louver.visual(Box((0.035, 0.022, 0.012)), origin=Origin(xyz=(0.000, -0.017, 0.000)), material=brass, name="connector_tab")
        model.articulation(
            f"panel_to_louver_{i}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(0.370, panel_y, zc)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.72, upper=0.72),
            mimic=None if i == 0 else Mimic(joint="panel_to_louver_0", multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    panel = object_model.get_part("panel")
    tilt_rod = object_model.get_part("tilt_rod")
    mid_louver = object_model.get_part("louver_4")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    tilt_slide = object_model.get_articulation("panel_to_tilt_rod")
    louver_drive = object_model.get_articulation("panel_to_louver_0")

    with ctx.pose({panel_hinge: 0.0, tilt_slide: 0.0}):
        ctx.expect_within(
            panel,
            frame,
            axes="xz",
            margin=0.002,
            name="closed panel sits inside fixed frame outline",
        )
        ctx.expect_contact(
            mid_louver,
            tilt_rod,
            elem_a="connector_tab",
            elem_b="rod",
            contact_tol=0.002,
            name="center louver tab meets tilt rod",
        )
        ctx.expect_overlap(
            mid_louver,
            panel,
            axes="x",
            elem_a="pivot_pin",
            min_overlap=0.55,
            name="louver pivot spans between stiles",
        )

    rest_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_hinge: 1.05}):
        open_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "panel swings outward on vertical hinges",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < rest_panel_aabb[0][1] - 0.20,
        details=f"closed={rest_panel_aabb}, open={open_panel_aabb}",
    )

    rest_rod_pos = ctx.part_world_position(tilt_rod)
    with ctx.pose({tilt_slide: 0.030}):
        raised_rod_pos = ctx.part_world_position(tilt_rod)
    ctx.check(
        "tilt rod slides vertically",
        rest_rod_pos is not None and raised_rod_pos is not None and raised_rod_pos[2] > rest_rod_pos[2] + 0.025,
        details=f"rest={rest_rod_pos}, raised={raised_rod_pos}",
    )

    rest_blade = ctx.part_element_world_aabb(mid_louver, elem="blade")
    with ctx.pose({louver_drive: 0.60}):
        tilted_blade = ctx.part_element_world_aabb(mid_louver, elem="blade")
    ctx.check(
        "louver bank rotates together",
        rest_blade is not None
        and tilted_blade is not None
        and (tilted_blade[1][1] - tilted_blade[0][1]) > (rest_blade[1][1] - rest_blade[0][1]) + 0.035,
        details=f"rest_blade={rest_blade}, tilted_blade={tilted_blade}",
    )

    return ctx.report()


object_model = build_object_model()
