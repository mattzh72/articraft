from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_security_gate")

    painted_metal = model.material("painted_metal", rgba=(0.18, 0.20, 0.23, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.10, 0.11, 0.12, 1.0))
    elastomer = model.material("elastomer", rgba=(0.05, 0.05, 0.05, 1.0))

    overall_w = 1.94
    overall_d = 0.14
    overall_h = 2.12
    left_post_w = 0.085
    right_post_w = 0.070
    header_h = 0.085
    sill_h = 0.055

    inner_left = -overall_w / 2.0 + left_post_w
    inner_right = overall_w / 2.0 - right_post_w
    travel_span = inner_right - inner_left
    opening_w = 0.84
    pocket_w = travel_span - opening_w

    leaf_w = 0.92
    leaf_h = 1.90
    leaf_d = 0.045
    leaf_bottom_z = 0.095
    leaf_center_z = leaf_bottom_z + leaf_h / 2.0
    leaf_closed_right = inner_right - 0.013
    leaf_closed_center_x = leaf_closed_right - leaf_w / 2.0
    slide_travel = opening_w

    frame = model.part("frame")
    frame.visual(
        Box((left_post_w, overall_d, overall_h)),
        origin=Origin(xyz=(-overall_w / 2.0 + left_post_w / 2.0, 0.0, overall_h / 2.0)),
        material=painted_metal,
        name="left_post",
    )
    frame.visual(
        Box((right_post_w, overall_d, overall_h)),
        origin=Origin(xyz=(overall_w / 2.0 - right_post_w / 2.0, 0.0, overall_h / 2.0)),
        material=painted_metal,
        name="right_post",
    )
    frame.visual(
        Box((travel_span, overall_d, header_h)),
        origin=Origin(xyz=((inner_left + inner_right) / 2.0, 0.0, overall_h - header_h / 2.0)),
        material=painted_metal,
        name="header",
    )
    frame.visual(
        Box((travel_span, 0.10, sill_h)),
        origin=Origin(xyz=((inner_left + inner_right) / 2.0, 0.0, sill_h / 2.0)),
        material=painted_metal,
        name="sill",
    )
    frame.visual(
        Box((pocket_w, 0.024, overall_h - header_h - sill_h)),
        origin=Origin(
            xyz=(
                inner_left + pocket_w / 2.0,
                -0.058,
                sill_h + (overall_h - header_h - sill_h) / 2.0,
            )
        ),
        material=painted_metal,
        name="pocket_back_panel",
    )
    frame.visual(
        Box((travel_span, 0.020, 0.030)),
        origin=Origin(xyz=((inner_left + inner_right) / 2.0, 0.022, overall_h - header_h - 0.015)),
        material=polymer_black,
        name="top_front_lip",
    )
    frame.visual(
        Box((travel_span, 0.020, 0.030)),
        origin=Origin(xyz=((inner_left + inner_right) / 2.0, -0.022, overall_h - header_h - 0.015)),
        material=polymer_black,
        name="top_rear_lip",
    )
    frame.visual(
        Box((travel_span, 0.018, 0.012)),
        origin=Origin(xyz=((inner_left + inner_right) / 2.0, 0.0, sill_h + 0.006)),
        material=satin_metal,
        name="bottom_track",
    )
    frame.visual(
        Box((travel_span, 0.012, 0.026)),
        origin=Origin(xyz=((inner_left + inner_right) / 2.0, 0.019, sill_h + 0.013)),
        material=polymer_black,
        name="slot_front_wall",
    )
    frame.visual(
        Box((travel_span, 0.012, 0.026)),
        origin=Origin(xyz=((inner_left + inner_right) / 2.0, -0.019, sill_h + 0.013)),
        material=polymer_black,
        name="slot_rear_wall",
    )
    frame.visual(
        Box((0.006, 0.028, 0.18)),
        origin=Origin(xyz=(0.896, 0.0, 1.045)),
        material=elastomer,
        name="closed_stop",
    )
    frame.visual(
        Box((0.006, 0.028, 0.18)),
        origin=Origin(xyz=(-0.882, 0.0, 1.045)),
        material=elastomer,
        name="open_stop",
    )
    frame.visual(
        Box((0.014, 0.014, 0.045)),
        origin=Origin(xyz=(0.893, 0.0, 1.145)),
        material=satin_metal,
        name="keeper_upper",
    )
    frame.visual(
        Box((0.014, 0.014, 0.045)),
        origin=Origin(xyz=(0.893, 0.0, 0.945)),
        material=satin_metal,
        name="keeper_lower",
    )
    frame.inertial = Inertial.from_geometry(
        Box((overall_w, overall_d, overall_h)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, overall_h / 2.0)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((leaf_w, leaf_d, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, leaf_h / 2.0 - 0.0275)),
        material=painted_metal,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((leaf_w, leaf_d, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, -leaf_h / 2.0 + 0.0325)),
        material=painted_metal,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.045, leaf_d, leaf_h)),
        origin=Origin(xyz=(-leaf_w / 2.0 + 0.0225, 0.0, 0.0)),
        material=painted_metal,
        name="trail_stile",
    )
    gate_leaf.visual(
        Box((0.055, leaf_d, leaf_h)),
        origin=Origin(xyz=(leaf_w / 2.0 - 0.0275, 0.0, 0.0)),
        material=painted_metal,
        name="lock_stile",
    )

    slat_height = leaf_h - 0.055 - 0.065
    for idx, x in enumerate((-0.300, -0.180, -0.060, 0.060, 0.180, 0.300), start=1):
        gate_leaf.visual(
            Box((0.016, 0.022, slat_height)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=painted_metal,
            name=f"slat_{idx}",
        )

    gate_leaf.visual(
        Box((0.780, 0.012, 0.024)),
        origin=Origin(xyz=(-0.015, 0.0, leaf_h / 2.0 + 0.0115)),
        material=polymer_black,
        name="top_guide_tongue",
    )
    gate_leaf.visual(
        Box((0.720, 0.014, 0.026)),
        origin=Origin(xyz=(-0.020, 0.0, -leaf_h / 2.0 - 0.012)),
        material=polymer_black,
        name="guide_fin",
    )
    gate_leaf.visual(
        Box((0.040, 0.012, 0.070)),
        origin=Origin(xyz=(-0.250, 0.0, -leaf_h / 2.0 + 0.030)),
        material=satin_metal,
        name="trail_roller_bracket",
    )
    gate_leaf.visual(
        Box((0.040, 0.012, 0.070)),
        origin=Origin(xyz=(0.270, 0.0, -leaf_h / 2.0 + 0.030)),
        material=satin_metal,
        name="lead_roller_bracket",
    )
    gate_leaf.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(-0.250, 0.0, -leaf_h / 2.0 - 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer_black,
        name="trail_roller",
    )
    gate_leaf.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.270, 0.0, -leaf_h / 2.0 - 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer_black,
        name="lead_roller",
    )
    gate_leaf.visual(
        Box((0.120, 0.008, 0.280)),
        origin=Origin(xyz=(0.400, 0.0260, 0.0)),
        material=satin_metal,
        name="handle_escutcheon",
    )
    gate_leaf.visual(
        Box((0.120, 0.016, 0.034)),
        origin=Origin(xyz=(0.400, 0.0375, 0.0)),
        material=polymer_black,
        name="pull_handle",
    )
    gate_leaf.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.408, 0.031, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer_black,
        name="lock_cylinder_guard",
    )
    gate_leaf.visual(
        Box((0.006, 0.026, 0.140)),
        origin=Origin(xyz=(0.463, 0.0, 0.0)),
        material=elastomer,
        name="strike_bumper",
    )
    gate_leaf.visual(
        Box((0.006, 0.026, 0.140)),
        origin=Origin(xyz=(-0.463, 0.0, 0.0)),
        material=elastomer,
        name="open_bumper",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((leaf_w, leaf_d, leaf_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(leaf_closed_center_x, 0.0, leaf_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=slide_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")

    top_front_lip = frame.get_visual("top_front_lip")
    top_rear_lip = frame.get_visual("top_rear_lip")
    bottom_track = frame.get_visual("bottom_track")
    slot_front_wall = frame.get_visual("slot_front_wall")
    slot_rear_wall = frame.get_visual("slot_rear_wall")
    closed_stop = frame.get_visual("closed_stop")
    open_stop = frame.get_visual("open_stop")

    top_guide_tongue = gate_leaf.get_visual("top_guide_tongue")
    guide_fin = gate_leaf.get_visual("guide_fin")
    lead_roller = gate_leaf.get_visual("lead_roller")
    strike_bumper = gate_leaf.get_visual("strike_bumper")
    open_bumper = gate_leaf.get_visual("open_bumper")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    closed_pos = ctx.part_world_position(gate_leaf)

    ctx.expect_contact(
        gate_leaf,
        frame,
        elem_a=lead_roller,
        elem_b=bottom_track,
        contact_tol=0.0005,
        name="lead_roller_bears_on_bottom_track",
    )
    ctx.expect_gap(
        frame,
        gate_leaf,
        axis="y",
        positive_elem=top_front_lip,
        negative_elem=top_guide_tongue,
        min_gap=0.004,
        max_gap=0.008,
        name="top_front_capture_gap",
    )
    ctx.expect_gap(
        gate_leaf,
        frame,
        axis="y",
        positive_elem=top_guide_tongue,
        negative_elem=top_rear_lip,
        min_gap=0.004,
        max_gap=0.008,
        name="top_rear_capture_gap",
    )
    ctx.expect_gap(
        frame,
        gate_leaf,
        axis="y",
        positive_elem=slot_front_wall,
        negative_elem=guide_fin,
        min_gap=0.004,
        max_gap=0.008,
        name="bottom_front_slot_gap",
    )
    ctx.expect_gap(
        gate_leaf,
        frame,
        axis="y",
        positive_elem=guide_fin,
        negative_elem=slot_rear_wall,
        min_gap=0.004,
        max_gap=0.008,
        name="bottom_rear_slot_gap",
    )
    ctx.expect_contact(
        frame,
        gate_leaf,
        elem_a=closed_stop,
        elem_b=strike_bumper,
        contact_tol=0.0005,
        name="closed_stop_contacts_strike_bumper",
    )

    with ctx.pose({slide: slide.motion_limits.upper}):
        open_pos = ctx.part_world_position(gate_leaf)
        ctx.expect_contact(
            gate_leaf,
            frame,
            elem_a=open_bumper,
            elem_b=open_stop,
            contact_tol=0.0005,
            name="open_stop_contacts_open_bumper",
        )

    open_pos = locals().get("open_pos")
    if closed_pos is None or open_pos is None:
        ctx.fail("gate_leaf_pose_query", "Could not resolve gate leaf world position in one or more poses.")
    else:
        ctx.check(
            "gate_travels_left_when_opening",
            open_pos[0] < closed_pos[0] - 0.80,
            details=f"closed_x={closed_pos[0]:.4f}, open_x={open_pos[0]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
