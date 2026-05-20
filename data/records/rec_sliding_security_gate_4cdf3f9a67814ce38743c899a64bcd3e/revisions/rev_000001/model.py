from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _add_box(part, name, size, center, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, center, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sliding_security_gate")

    model.material("frame_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("gate_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("hardware", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("bumper", rgba=(0.72, 0.56, 0.22, 1.0))

    overall_w = 0.67
    overall_d = 0.048
    overall_h = 0.43
    x_min = -overall_w / 2.0
    x_max = overall_w / 2.0

    left_jamb_w = 0.028
    right_post_w = 0.03
    parking_span_w = 0.335
    opening_min_x = x_min + parking_span_w
    opening_max_x = x_max - right_post_w
    opening_w = opening_max_x - opening_min_x

    head_top_t = 0.005
    head_lip_t = 0.004
    head_depth = 0.024
    head_cavity_h = 0.016
    head_lip_bottom_z = 0.393
    head_lip_center_z = head_lip_bottom_z + head_cavity_h / 2.0
    head_top_center_z = head_lip_bottom_z + head_cavity_h + head_top_t / 2.0
    head_cover_h = overall_h - (head_lip_bottom_z + head_cavity_h + head_top_t)
    head_cover_center_z = overall_h - head_cover_h / 2.0
    head_wall_y = head_depth / 2.0 - head_lip_t / 2.0

    sill_floor_t = 0.005
    sill_lip_t = 0.004
    sill_depth = 0.024
    sill_lip_h = 0.028
    sill_floor_center_z = sill_floor_t / 2.0
    sill_lip_center_z = sill_lip_h / 2.0
    sill_wall_y = sill_depth / 2.0 - sill_lip_t / 2.0

    frame = model.part("frame")
    _add_box(
        frame,
        "left_jamb",
        (left_jamb_w, overall_d, overall_h),
        (x_min + left_jamb_w / 2.0, 0.0, overall_h / 2.0),
        "frame_steel",
    )
    _add_box(
        frame,
        "right_post",
        (right_post_w, overall_d, overall_h),
        (x_max - right_post_w / 2.0, 0.0, overall_h / 2.0),
        "frame_steel",
    )
    _add_box(
        frame,
        "parking_back_plate",
        (parking_span_w - left_jamb_w, 0.004, overall_h - 0.06),
        (x_min + left_jamb_w + (parking_span_w - left_jamb_w) / 2.0, -0.018, overall_h / 2.0),
        "frame_steel",
    )
    _add_box(
        frame,
        "head_top_wall",
        (overall_w, head_depth, head_top_t),
        (0.0, 0.0, head_top_center_z),
        "frame_steel",
    )
    _add_box(
        frame,
        "head_front_lip",
        (overall_w, head_lip_t, head_cavity_h),
        (0.0, head_wall_y, head_lip_center_z),
        "frame_steel",
    )
    _add_box(
        frame,
        "head_rear_lip",
        (overall_w, head_lip_t, head_cavity_h),
        (0.0, -head_wall_y, head_lip_center_z),
        "frame_steel",
    )
    _add_box(
        frame,
        "head_cover",
        (overall_w, overall_d, head_cover_h),
        (0.0, 0.0, head_cover_center_z),
        "frame_steel",
    )
    _add_box(
        frame,
        "sill_floor",
        (overall_w, sill_depth, sill_floor_t),
        (0.0, 0.0, sill_floor_center_z),
        "frame_steel",
    )
    _add_box(
        frame,
        "sill_front_guide",
        (overall_w, sill_lip_t, sill_lip_h),
        (0.0, sill_wall_y, sill_lip_center_z),
        "frame_steel",
    )
    _add_box(
        frame,
        "sill_rear_guide",
        (overall_w, sill_lip_t, sill_lip_h),
        (0.0, -sill_wall_y, sill_lip_center_z),
        "frame_steel",
    )
    _add_box(
        frame,
        "open_stop",
        (0.008, 0.016, 0.012),
        (-0.311, 0.0, 0.399),
        "bumper",
    )
    _add_box(
        frame,
        "closed_stop",
        (0.008, 0.016, 0.012),
        (0.290, 0.0, 0.399),
        "bumper",
    )
    _add_box(
        frame,
        "striker_block",
        (0.014, 0.014, 0.08),
        (opening_max_x + 0.007, 0.0, 0.215),
        "hardware",
    )

    leaf = model.part("gate_leaf")
    leaf_w = opening_w - 0.008
    leaf_h = 0.354
    leaf_d = 0.012
    stile_w = 0.018
    rail_h = 0.018
    inner_h = leaf_h - 2.0 * rail_h
    inner_w = leaf_w - 2.0 * stile_w
    leaf_bottom_z = 0.036
    leaf_center_z = leaf_bottom_z + leaf_h / 2.0
    stile_center_x = leaf_w / 2.0 - stile_w / 2.0
    rail_center_z = leaf_h / 2.0 - rail_h / 2.0

    _add_box(
        leaf,
        "left_stile",
        (stile_w, leaf_d, leaf_h),
        (-stile_center_x, 0.0, 0.0),
        "gate_steel",
    )
    _add_box(
        leaf,
        "right_stile",
        (stile_w, leaf_d, leaf_h),
        (stile_center_x, 0.0, 0.0),
        "gate_steel",
    )
    _add_box(
        leaf,
        "top_rail",
        (inner_w, leaf_d, rail_h),
        (0.0, 0.0, rail_center_z),
        "gate_steel",
    )
    _add_box(
        leaf,
        "bottom_rail",
        (inner_w, leaf_d, rail_h),
        (0.0, 0.0, -rail_center_z),
        "gate_steel",
    )

    for index, x_pos in enumerate(
        (-inner_w / 3.0, -inner_w / 6.0, 0.0, inner_w / 6.0, inner_w / 3.0),
        start=1,
    ):
        _add_box(
            leaf,
            f"bar_{index}",
            (0.008, 0.010, inner_h),
            (x_pos, 0.0, 0.0),
            "gate_steel",
        )

    _add_box(
        leaf,
        "latch_housing",
        (0.034, 0.018, 0.078),
        (leaf_w / 2.0 - 0.026, 0.0, 0.0),
        "hardware",
    )
    _add_box(
        leaf,
        "latch_nose",
        (0.006, 0.010, 0.024),
        (leaf_w / 2.0, 0.0, 0.0),
        "hardware",
    )

    roller_x = leaf_w / 2.0 - 0.026
    _add_box(
        leaf,
        "roller_bracket_left",
        (0.018, 0.010, 0.016),
        (-roller_x, 0.0, leaf_h / 2.0 + 0.006),
        "hardware",
    )
    _add_box(
        leaf,
        "roller_bracket_right",
        (0.018, 0.010, 0.016),
        (roller_x, 0.0, leaf_h / 2.0 + 0.006),
        "hardware",
    )
    _add_cylinder(
        leaf,
        "top_roller_left",
        0.007,
        0.006,
        (-roller_x, 0.0, leaf_h / 2.0 + 0.012),
        "hardware",
        rpy=(1.5707963267948966, 0.0, 0.0),
    )
    _add_cylinder(
        leaf,
        "top_roller_right",
        0.007,
        0.006,
        (roller_x, 0.0, leaf_h / 2.0 + 0.012),
        "hardware",
        rpy=(1.5707963267948966, 0.0, 0.0),
    )
    _add_box(
        leaf,
        "guide_shoe",
        (0.055, 0.008, 0.028),
        (0.0, -0.004, -leaf_h / 2.0 - 0.014),
        "hardware",
    )

    closed_center_x = (opening_min_x + opening_max_x) / 2.0
    parked_center_x = x_min + left_jamb_w + 0.004 + leaf_w / 2.0
    travel = closed_center_x - parked_center_x

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(closed_center_x, 0.0, leaf_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.25,
            lower=0.0,
            upper=travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")

    head_top_wall = frame.get_visual("head_top_wall")
    sill_floor = frame.get_visual("sill_floor")
    sill_rear_guide = frame.get_visual("sill_rear_guide")
    latch_post = frame.get_visual("right_post")
    striker = frame.get_visual("striker_block")
    open_stop = frame.get_visual("open_stop")

    top_roller_left = leaf.get_visual("top_roller_left")
    top_roller_right = leaf.get_visual("top_roller_right")
    right_stile = leaf.get_visual("right_stile")
    left_stile = leaf.get_visual("left_stile")
    latch_nose = leaf.get_visual("latch_nose")
    guide_shoe = leaf.get_visual("guide_shoe")

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

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            frame,
            leaf,
            elem_a=head_top_wall,
            elem_b=top_roller_left,
            name="closed_leaf_supported_by_head_track",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem=striker,
            negative_elem=latch_nose,
            min_gap=0.001,
            max_gap=0.004,
            name="closed_latch_has_safe_standoff",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem=guide_shoe,
            negative_elem=sill_floor,
            min_gap=0.002,
            max_gap=0.004,
            name="closed_bottom_guide_clears_channel_floor",
        )

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_contact(
            frame,
            leaf,
            elem_a=sill_rear_guide,
            elem_b=guide_shoe,
            name="open_leaf_stays_captured_in_bottom_channel",
        )
        ctx.expect_contact(
            frame,
            leaf,
            elem_a=head_top_wall,
            elem_b=top_roller_right,
            name="open_leaf_stays_hung_from_head_track",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem=latch_post,
            negative_elem=right_stile,
            min_gap=0.30,
            name="open_position_clears_passage",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="x",
            positive_elem=left_stile,
            negative_elem=open_stop,
            min_gap=0.003,
            max_gap=0.008,
            name="open_travel_finishes_at_left_stop_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
