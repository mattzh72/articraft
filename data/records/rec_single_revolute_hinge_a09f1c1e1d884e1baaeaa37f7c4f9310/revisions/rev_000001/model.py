from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gate_hinge_bracket")

    # Root part: wall mounting bracket (fixed to wall)
    wall_bracket = model.part("wall_bracket")

    # --- Wall bracket visuals ---
    # 1. Mounting plate (flat plate attached to wall)
    # 0.01m thick (X), 0.15m wide (Y), 0.2m tall (Z)
    plate = cq.Workplane("XY").box(0.01, 0.15, 0.2)
    wall_bracket.visual(
        mesh_from_cadquery(plate, "mounting_plate"),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        name="mounting_plate",
        color=(0.2, 0.2, 0.2, 1.0),  # Dark gray metal
    )

    # 2. Hinge ears with pin hole (two parallel ears projecting from plate)
    # Each ear: 0.02m long (X), 0.01m thick (Y), 0.1m tall (Z)
    # Pin hole: 0.005m diameter at center of ear
    ear = (
        cq.Workplane("XY")
        .box(0.02, 0.01, 0.1)
        .faces(">X")
        .workplane()
        .hole(0.005)  # 5mm diameter hole through ear
    )

    # Top ear: Y=+0.015m (center), inner face at Y=+0.010m
    wall_bracket.visual(
        mesh_from_cadquery(ear, "top_ear"),
        origin=Origin(xyz=(0.02, 0.015, 0.0)),
        name="top_ear",
        color=(0.25, 0.25, 0.25, 1.0),  # Slightly lighter than plate
    )

    # Bottom ear: Y=-0.015m (center), inner face at Y=-0.010m
    wall_bracket.visual(
        mesh_from_cadquery(ear, "bottom_ear"),
        origin=Origin(xyz=(0.02, -0.015, 0.0)),
        name="bottom_ear",
        color=(0.25, 0.25, 0.25, 1.0),
    )

    # 3. Blocky welded support gussets (reinforce ear-plate joint)
    # Simple block: 0.02m x 0.01m x 0.02m
    gusset = cq.Workplane("XY").box(0.02, 0.01, 0.02)

    # Top ear gusset (at bottom of top ear, against plate)
    wall_bracket.visual(
        mesh_from_cadquery(gusset, "top_gusset"),
        origin=Origin(xyz=(0.02, 0.015, -0.04)),
        name="top_gusset",
        color=(0.2, 0.2, 0.2, 1.0),
    )

    # Bottom ear gusset (at top of bottom ear, against plate)
    wall_bracket.visual(
        mesh_from_cadquery(gusset, "bottom_gusset"),
        origin=Origin(xyz=(0.02, -0.015, 0.04)),
        name="bottom_gusset",
        color=(0.2, 0.2, 0.2, 1.0),
    )

    # 4. Vertical hinge pin (passes through ears and gate tab)
    # 0.12m long (Z), 0.005m diameter, with chamfered ends
    pin = (
        cq.Workplane("XY")
        .cylinder(0.12, 0.0025)
        .faces("|Z")  # Select both end faces
        .chamfer(0.0005)  # Small chamfer on pin ends for realism
    )
    wall_bracket.visual(
        mesh_from_cadquery(pin, "pin"),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        name="pin",
        color=(0.8, 0.8, 0.8, 1.0),  # Steel color
    )

    # 5. Washers (seat against ears, pin passes through)
    # 0.002m thick (Z), outer radius 0.005m
    washer = cq.Workplane("XY").cylinder(0.002, 0.005)

    # Top washer (at Y=+0.015 to align with top ear, Z=0.051 above ear top face)
    wall_bracket.visual(
        mesh_from_cadquery(washer, "top_washer"),
        origin=Origin(xyz=(0.02, 0.015, 0.051)),
        name="top_washer",
        color=(0.7, 0.7, 0.7, 1.0),  # Slightly darker than pin
    )

    # Bottom washer (at Y=-0.015 to align with bottom ear, Z=-0.051 below ear bottom face)
    wall_bracket.visual(
        mesh_from_cadquery(washer, "bottom_washer"),
        origin=Origin(xyz=(0.02, -0.015, -0.051)),
        name="bottom_washer",
        color=(0.7, 0.7, 0.7, 1.0),
    )

    # --- Moving part: gate tab (rotates on vertical pin) ---
    gate_tab = model.part("gate_tab")

    # Tab with pin hole: 0.1m long (X, from hinge to free end), 0.015m thick (Y), 0.08m tall (Z)
    # Tab extends from X=0 to X=0.1 in part frame (hinge at X=0, free end at X=0.1)
    # Pin hole at X=0 (at the hinge point)
    tab_body = (
        cq.Workplane("XY")
        .box(0.1, 0.015, 0.08)
        .translate((0.05, 0, 0))  # Offset so hinge is at X=0 in part frame
        .faces(">X")  # Select the free end face
        .workplane()
        .hole(0.005)  # Drill 5mm hole through entire tab
    )
    gate_tab.visual(
        mesh_from_cadquery(tab_body, "tab_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Part frame at hinge point (X=0 in part frame)
        name="tab_body",
        color=(0.6, 0.6, 0.6, 1.0),  # Medium gray contrast
    )

    # --- Primary articulation: vertical revolute joint ---
    model.articulation(
        "wall_to_tab",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=gate_tab,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),  # Pin center = articulation origin (in world coords)
        axis=(0.0, 0.0, 1.0),  # Vertical Z-axis
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-1.5708, upper=1.5708),  # ±90°
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    gate_tab = object_model.get_part("gate_tab")
    joint = object_model.get_articulation("wall_to_tab")

    # 1. Main mechanism: joint is vertical revolute
    ctx.check(
        "joint_is_vertical_revolute",
        joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint type: {joint.articulation_type}, axis: {joint.axis}",
    )

    # 2. Gate tab is physically captured between hinge ears (Y-axis)
    # Tab Y extent: -0.0075 to 0.0075 (centered at Y=0, thickness=0.015)
    # Ears at Y=±0.015 with thickness 0.01, so inner faces at Y=±0.010
    # Tab should have clearance from both ears
    with ctx.pose({joint: 0.0}):
        # Check tab has clearance from top ear (top ear inner face at Y=+0.010)
        ctx.expect_gap(
            wall_bracket,  # positive_link (top_ear is on wall_bracket)
            gate_tab,      # negative_link (tab_body is on gate_tab)
            axis="y",
            min_gap=0.002,  # At least 2mm clearance
            positive_elem="top_ear",
            negative_elem="tab_body",
            name="tab_clearance_from_top_ear",
        )
        # Check tab has clearance from bottom ear (bottom ear inner face at Y=-0.010)
        ctx.expect_gap(
            gate_tab,      # positive_link (tab_body is on gate_tab)
            wall_bracket,  # negative_link (bottom_ear is on wall_bracket)
            axis="y",
            min_gap=0.002,  # At least 2mm clearance
            positive_elem="tab_body",
            negative_elem="bottom_ear",
            name="tab_clearance_from_bottom_ear",
        )

    # 3. Joint rotation moves tab correctly (tab swings in XY plane)
    with ctx.pose({joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(gate_tab)
    with ctx.pose({joint: 1.5708}):  # 90° open
        open_aabb = ctx.part_world_aabb(gate_tab)
        ctx.check(
            "tab_swings_in_xy_plane",
            abs(open_aabb[1][0] - rest_aabb[1][0]) > 0.02 or abs(open_aabb[0][1] - rest_aabb[0][1]) > 0.02,
            details=f"Rest X range: {rest_aabb[0][0]:.3f}-{rest_aabb[1][0]:.3f}, "
                    f"Open X range: {open_aabb[0][0]:.3f}-{open_aabb[1][0]:.3f}",
        )

    # 4. Support: Ears are reinforced by gussets (contact in XZ plane)
    ctx.expect_contact(
        wall_bracket,
        wall_bracket,
        elem_a="top_ear",
        elem_b="top_gusset",
        contact_tol=0.005,
        name="top_ear_supported_by_gusset",
    )
    ctx.expect_contact(
        wall_bracket,
        wall_bracket,
        elem_a="bottom_ear",
        elem_b="bottom_gusset",
        contact_tol=0.005,
        name="bottom_ear_supported_by_gusset",
    )

    # 5. Washers seat against ears (Y-aligned washers contact Y-aligned ears)
    ctx.expect_contact(
        wall_bracket,
        wall_bracket,
        elem_a="top_washer",
        elem_b="top_ear",
        contact_tol=0.002,
        name="top_washer_seated_on_ear",
    )
    ctx.expect_contact(
        wall_bracket,
        wall_bracket,
        elem_a="bottom_washer",
        elem_b="bottom_ear",
        contact_tol=0.002,
        name="bottom_washer_seated_on_ear",
    )

    # 6. Pin passes through tab (intentional overlap - pin captured in tab)
    ctx.allow_overlap(
        "wall_bracket",
        "gate_tab",
        elem_a="pin",
        elem_b="tab_body",
        reason="Pin intentionally passes through hole in gate tab",
    )

    # 7. Pin passes through ears (intentional overlap)
    ctx.allow_overlap(
        "wall_bracket",
        "wall_bracket",
        elem_a="pin",
        elem_b="top_ear",
        reason="Pin intentionally passes through hole in top ear",
    )
    ctx.allow_overlap(
        "wall_bracket",
        "wall_bracket",
        elem_a="pin",
        elem_b="bottom_ear",
        reason="Pin intentionally passes through hole in bottom ear",
    )

    return ctx.report()


object_model = build_object_model()
