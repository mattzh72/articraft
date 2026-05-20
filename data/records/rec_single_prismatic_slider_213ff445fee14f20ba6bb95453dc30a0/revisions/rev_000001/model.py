from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="control_fader_module")
    model.meta["author"] = "ArticraftAgent"
    model.meta["description"] = "Realistic control fader module with prismatic slider, tick marks, and end stops."

    # Base panel (root part, fixed)
    base_panel = model.part("base_panel")
    base_panel.visual(
        mesh_from_cadquery(
            # Base panel shape: flat plate with slot cutout, chamfered edges
            (
                cq.Workplane("XY")
                .box(0.2, 0.06, 0.008)  # Total dims: 0.2m (X) x 0.06m (Y) x 0.008m (Z)
                .faces("+Z").workplane()  # Workplane on top face to define slot
                .rect(0.15, 0.008)  # Slot: 0.15m long (X), 0.008m wide (Y)
                .cutThruAll()  # Cut through entire panel thickness
                .edges("|Z").fillet(0.001)  # Fillet edges parallel to Z (corners)
            ),
            "base_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered at world origin
        material=Material(name="base_panel_material", color=(0.2, 0.2, 0.2)),  # Dark gray
        name="base_shell",
    )

    # Tick marks (raised geometry on base panel along slot)
    tick_spacing = 0.025  # 2.5cm between ticks
    tick_start_x = -0.075  # Left end of slot (slot spans X: -0.075 to +0.075)
    num_ticks = 7
    for i in range(num_ticks):
        tick_x = tick_start_x + i * tick_spacing
        tick_name = f"tick_mark_{i}"
        base_panel.visual(
            Box((0.005, 0.002, 0.001)),  # Tick: 5mm (X) x 2mm (Y) x 1mm (Z)
            origin=Origin(xyz=(tick_x, 0.0, 0.004 + 0.0005)),  # On top of panel (panel top surface at Z=+0.004)
            material=Material(name="tick_material", color=(0.9, 0.9, 0.9)),  # White
            name=tick_name,
        )

    # End stops (raised bumps at slot ends)
    end_stop_dims = (0.01, 0.01, 0.002)  # 1cm (X) x 1cm (Y) x 2mm (Z)
    end_stop_z = 0.004 + 0.001  # Top of panel + half end stop thickness
    # Left end stop
    base_panel.visual(
        Box(end_stop_dims),
        origin=Origin(xyz=(-0.075, 0.0, end_stop_z)),
        material=Material(name="tick_material", color=(0.9, 0.9, 0.9)),
        name="end_stop_left",
    )
    # Right end stop
    base_panel.visual(
        Box(end_stop_dims),
        origin=Origin(xyz=(0.075, 0.0, end_stop_z)),
        material=Material(name="tick_material", color=(0.9, 0.9, 0.9)),
        name="end_stop_right",
    )

    # Fader cap (moving part, prismatic joint)
    fader_cap = model.part("fader_cap")

    # Fader stem (slides inside base panel slot)
    stem_shape = cq.Workplane("XY").box(0.02, 0.007, 0.008)  # 2cm (X) x 7mm (Y) x 8mm (Z)
    fader_cap.visual(
        mesh_from_cadquery(stem_shape, "fader_stem_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Stem centered at fader_cap origin
        material=Material(name="fader_material", color=(0.8, 0.1, 0.1)),  # Red
        name="fader_stem",
    )

    # Fader cap top (above panel)
    cap_shape = (
        cq.Workplane("XY")
        .box(0.02, 0.04, 0.01)  # 2cm (X) x 4cm (Y) x 1cm (Z)
        .faces("+Z").edges().fillet(0.002)  # Round top edges
        .translate((0.0, 0.0, 0.010))  # Position on top of stem (stem top at Z=0.004, cap center at 0.010, bottom at 0.005, 0.001 above base top)
    )
    fader_cap.visual(
        mesh_from_cadquery(cap_shape, "fader_cap_top_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Relative to fader_cap origin
        material=Material(name="fader_material", color=(0.8, 0.1, 0.1)),  # Red
        name="fader_cap_top",
    )

    # Prismatic joint for fader travel
    model.articulation(
        "base_to_fader",
        ArticulationType.PRISMATIC,
        parent=base_panel,
        child=fader_cap,
        origin=Origin(xyz=(-0.05, 0.0, 0.0)),  # Start position at X=-0.05 (left side of travel)
        axis=(1.0, 0.0, 0.0),  # Slide along X axis
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.1,  # 10cm total travel
            effort=10.0,
            velocity=0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_panel")
    fader = object_model.get_part("fader_cap")
    joint = object_model.get_articulation("base_to_fader")

    # Allow intentional overlap between fader stem and base panel slot
    ctx.allow_overlap(
        "fader_cap",
        "base_panel",
        elem_a="fader_stem",
        elem_b="base_shell",
        reason="Fader stem intentionally slides inside base panel slot",
    )

    # Rest pose (q=0.0)
    with ctx.pose({joint: 0.0}):
        # Check fader stem overlaps with base slot along X axis
        ctx.expect_overlap(
            fader,
            base,
            axes="x",
            min_overlap=0.01,
            elem_a="fader_stem",
            elem_b="base_shell",
            name="rest_pose_stem_overlap_x",
        )
        # Check fader stem is contained within base slot along X and Y axes
        ctx.expect_within(
            fader,
            base,
            axes=("x", "y"),
            margin=0.001,
            elem_a="fader_stem",
            elem_b="base_shell",
            name="rest_pose_stem_within_base",
        )
        # Check fader cap top is above base panel (no Z overlap)
        ctx.expect_gap(
            fader,
            base,
            axis="z",
            min_gap=0.001,
            elem_a="fader_cap_top",
            elem_b="base_shell",
            name="rest_pose_cap_above_base",
        )
        # Verify tick marks are present (7 total)
        tick_marks = [v for v in base.visuals if v.name.startswith("tick_mark_")]
        ctx.check(
            "rest_pose_tick_marks_present",
            len(tick_marks) == 7,
            details=f"Found {len(tick_marks)} tick marks, expected 7",
        )
        # Verify end stops are present (2 total)
        end_stops = [v for v in base.visuals if v.name.startswith("end_stop_")]
        ctx.check(
            "rest_pose_end_stops_present",
            len(end_stops) == 2,
            details=f"Found {len(end_stops)} end stops, expected 2",
        )

    # Extended pose (q=0.1, maximum travel)
    rest_pos = ctx.part_world_position(fader)
    with ctx.pose({joint: 0.1}):
        extended_pos = ctx.part_world_position(fader)
        # Check fader moved exactly 0.1m along X axis
        ctx.check(
            "extended_pose_x_travel",
            abs(extended_pos[0] - rest_pos[0] - 0.1) < 0.001,
            details=f"Rest X: {rest_pos[0]:.3f}, Extended X: {extended_pos[0]:.3f}, expected delta 0.1m",
        )
        # Check stem still overlaps with base slot
        ctx.expect_overlap(
            fader,
            base,
            axes="x",
            min_overlap=0.01,
            elem_a="fader_stem",
            elem_b="base_shell",
            name="extended_pose_stem_overlap_x",
        )
        # Check stem still contained within base slot
        ctx.expect_within(
            fader,
            base,
            axes=("x", "y"),
            margin=0.001,
            elem_a="fader_stem",
            elem_b="base_shell",
            name="extended_pose_stem_within_base",
        )

    return ctx.report()


object_model = build_object_model()