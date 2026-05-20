from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def add_box(
    part,
    size,
    xyz,
    material,
    *,
    name=None,
    rpy=(0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder(
    part,
    radius,
    length,
    xyz,
    material,
    *,
    name=None,
    rpy=(0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mine_shaft_elevator_cage")

    frame_steel = model.material("frame_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    cage_steel = model.material("cage_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.17, 0.18, 0.19, 1.0))
    sheave_finish = model.material("sheave_finish", rgba=(0.67, 0.58, 0.18, 1.0))

    frame = model.part("shaft_frame")
    add_box(
        frame,
        (1.84, 0.24, 0.16),
        (0.0, 0.0, 0.08),
        frame_steel,
        name="base_sill",
    )
    add_box(
        frame,
        (1.92, 0.38, 0.22),
        (0.0, -0.02, 6.89),
        frame_steel,
        name="head_beam",
    )
    for x, name in ((-0.86, "left_tower_post"), (0.86, "right_tower_post")):
        add_box(
            frame,
            (0.12, 0.12, 6.84),
            (x, 0.0, 3.58),
            frame_steel,
            name=name,
        )
    add_box(
        frame,
        (1.60, 0.08, 0.12),
        (0.0, 0.10, 6.18),
        frame_steel,
        name="mid_tie",
    )
    for x, block_name, hanger_name in (
        (-0.085, "left_bearing_block", "left_sheave_hanger"),
        (0.085, "right_bearing_block", "right_sheave_hanger"),
    ):
        add_box(
            frame,
            (0.05, 0.12, 0.12),
            (x, -0.16, 6.49),
            frame_steel,
            name=block_name,
        )
        add_box(
            frame,
            (0.05, 0.12, 0.23),
            (x, -0.16, 6.665),
            frame_steel,
            name=hanger_name,
        )

    rail_height = 6.62
    left_rail = model.part("left_guide_rail")
    add_box(
        left_rail,
        (0.08, 0.12, rail_height),
        (0.0, 0.0, 0.0),
        rail_steel,
        name="left_rail_body",
    )
    add_box(
        left_rail,
        (0.04, 0.06, rail_height),
        (0.04, 0.0, 0.0),
        rail_steel,
        name="left_rail_keeper",
    )

    right_rail = model.part("right_guide_rail")
    add_box(
        right_rail,
        (0.08, 0.12, rail_height),
        (0.0, 0.0, 0.0),
        rail_steel,
        name="right_rail_body",
    )
    add_box(
        right_rail,
        (0.04, 0.06, rail_height),
        (-0.04, 0.0, 0.0),
        rail_steel,
        name="right_rail_keeper",
    )

    cage = model.part("cage")
    cage_width = 1.18
    cage_depth = 0.96
    cage_height = 2.18
    post = 0.05
    half_w = cage_width / 2.0
    half_d = cage_depth / 2.0
    opening_width = 1.08
    add_box(
        cage,
        (cage_width, cage_depth, 0.05),
        (0.0, 0.0, 0.025),
        deck_steel,
        name="floor_deck",
    )
    add_box(
        cage,
        (1.10, 0.88, 0.02),
        (0.0, 0.0, 2.16),
        deck_steel,
        name="roof_plate",
    )
    for x in (-half_w + post / 2.0, half_w - post / 2.0):
        for y in (-half_d + post / 2.0, half_d - post / 2.0):
            add_box(
                cage,
                (post, post, 2.10),
                (x, y, 1.10),
                cage_steel,
            )
    add_box(
        cage,
        (cage_width, post, 0.06),
        (0.0, -half_d + post / 2.0, 2.12),
        cage_steel,
        name="rear_roof_beam",
    )
    add_box(
        cage,
        (cage_width, post, 0.06),
        (0.0, half_d - post / 2.0, 2.12),
        cage_steel,
        name="front_roof_beam",
    )
    add_box(
        cage,
        (post, cage_depth - 2.0 * post, 0.06),
        (-half_w + post / 2.0, 0.0, 2.12),
        cage_steel,
        name="left_roof_rail",
    )
    add_box(
        cage,
        (post, cage_depth - 2.0 * post, 0.06),
        (half_w - post / 2.0, 0.0, 2.12),
        cage_steel,
        name="right_roof_rail",
    )
    for z, side in ((0.85, "lower"), (1.45, "upper")):
        add_box(
            cage,
            (post, cage_depth - 2.0 * post, 0.04),
            (-half_w + post / 2.0, 0.0, z),
            cage_steel,
            name=f"left_side_{side}_rail",
        )
        add_box(
            cage,
            (post, cage_depth - 2.0 * post, 0.04),
            (half_w - post / 2.0, 0.0, z),
            cage_steel,
            name=f"right_side_{side}_rail",
        )
        add_box(
            cage,
            (opening_width, post, 0.04),
            (0.0, -half_d + post / 2.0, z),
            cage_steel,
            name=f"rear_{side}_rail",
        )
    add_box(
        cage,
        (opening_width, post, 0.18),
        (0.0, half_d - post / 2.0, 0.14),
        cage_steel,
        name="front_threshold",
    )
    add_box(
        cage,
        (opening_width, post, 0.06),
        (0.0, half_d - post / 2.0, 1.98),
        cage_steel,
        name="front_lintel",
    )
    for x, name in ((-0.25, "back_slat_left"), (0.0, "back_slat_center"), (0.25, "back_slat_right")):
        add_box(
            cage,
            (0.04, post, 2.10),
            (x, -half_d + post / 2.0, 1.10),
            cage_steel,
            name=name,
        )
    for x, side in (
        (-0.635, "left"),
        (0.635, "right"),
    ):
        for z, level in ((0.85, "lower"), (1.45, "upper")):
            add_box(
                cage,
                (0.09, 0.10, 0.12),
                (x, 0.11, z),
                rail_steel,
                name=f"{side}_guide_shoe_{level}",
            )

    gate = model.part("safety_gate")
    gate_thickness = 0.035
    gate_width = opening_width
    gate_height = 1.72
    add_box(
        gate,
        (0.05, gate_thickness, gate_height),
        (0.025, 0.0, gate_height / 2.0),
        cage_steel,
        name="hinge_stile",
    )
    add_box(
        gate,
        (0.05, gate_thickness, gate_height),
        (gate_width - 0.025, 0.0, gate_height / 2.0),
        cage_steel,
        name="latch_stile",
    )
    add_box(
        gate,
        (gate_width, gate_thickness, 0.05),
        (gate_width / 2.0, 0.0, 0.025),
        cage_steel,
        name="bottom_rail",
    )
    add_box(
        gate,
        (gate_width, gate_thickness, 0.05),
        (gate_width / 2.0, 0.0, gate_height - 0.025),
        cage_steel,
        name="top_rail",
    )
    for x, name in ((0.36, "bar_left"), (0.72, "bar_right")):
        add_box(
            gate,
            (0.03, gate_thickness, gate_height - 0.10),
            (x, 0.0, gate_height / 2.0),
            cage_steel,
            name=name,
        )

    sheave = model.part("sheave_wheel")
    wheel_rpy = (0.0, pi / 2.0, 0.0)
    add_cylinder(
        sheave,
        0.31,
        0.02,
        (-0.035, 0.0, 0.0),
        sheave_finish,
        name="left_flange",
        rpy=wheel_rpy,
    )
    add_cylinder(
        sheave,
        0.31,
        0.02,
        (0.035, 0.0, 0.0),
        sheave_finish,
        name="right_flange",
        rpy=wheel_rpy,
    )
    add_cylinder(
        sheave,
        0.26,
        0.05,
        (0.0, 0.0, 0.0),
        sheave_finish,
        name="rim_web",
        rpy=wheel_rpy,
    )
    add_cylinder(
        sheave,
        0.08,
        0.12,
        (0.0, 0.0, 0.0),
        frame_steel,
        name="hub",
        rpy=wheel_rpy,
    )
    add_cylinder(
        sheave,
        0.018,
        0.12,
        (0.0, 0.0, 0.0),
        frame_steel,
        name="axle",
        rpy=wheel_rpy,
    )
    add_box(
        sheave,
        (0.03, 0.50, 0.04),
        (0.0, 0.0, 0.0),
        frame_steel,
        name="horizontal_spoke",
    )
    add_box(
        sheave,
        (0.03, 0.04, 0.50),
        (0.0, 0.0, 0.0),
        frame_steel,
        name="vertical_spoke",
    )

    model.articulation(
        "frame_to_left_rail",
        ArticulationType.FIXED,
        parent=frame,
        child=left_rail,
        origin=Origin(xyz=(-0.72, 0.0, 3.47)),
    )
    model.articulation(
        "frame_to_right_rail",
        ArticulationType.FIXED,
        parent=frame,
        child=right_rail,
        origin=Origin(xyz=(0.72, 0.0, 3.47)),
    )
    model.articulation(
        "cage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=1.5,
            lower=0.0,
            upper=4.0,
        ),
    )
    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=gate,
        origin=Origin(xyz=(-opening_width / 2.0, half_d + gate_thickness / 2.0, 0.23)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "sheave_axle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sheave,
        origin=Origin(xyz=(0.0, -0.16, 6.47)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1000.0,
            velocity=6.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shaft_frame")
    left_rail = object_model.get_part("left_guide_rail")
    right_rail = object_model.get_part("right_guide_rail")
    cage = object_model.get_part("cage")
    gate = object_model.get_part("safety_gate")
    sheave = object_model.get_part("sheave_wheel")

    cage_slide = object_model.get_articulation("cage_slide")
    gate_hinge = object_model.get_articulation("gate_hinge")
    sheave_axle = object_model.get_articulation("sheave_axle")

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

    ctx.check(
        "cage_slide_is_vertical_prismatic",
        cage_slide.articulation_type == ArticulationType.PRISMATIC
        and cage_slide.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical prismatic joint, got type={cage_slide.articulation_type}, axis={cage_slide.axis}",
    )
    ctx.check(
        "gate_is_front_revolute_hinge",
        gate_hinge.articulation_type == ArticulationType.REVOLUTE
        and gate_hinge.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical revolute hinge, got type={gate_hinge.articulation_type}, axis={gate_hinge.axis}",
    )
    ctx.check(
        "sheave_spins_on_horizontal_axle",
        sheave_axle.articulation_type == ArticulationType.REVOLUTE
        and sheave_axle.axis == (1.0, 0.0, 0.0),
        details=f"expected horizontal sheave axle, got type={sheave_axle.articulation_type}, axis={sheave_axle.axis}",
    )

    frame_pos = ctx.part_world_position(frame)
    left_pos = ctx.part_world_position(left_rail)
    right_pos = ctx.part_world_position(right_rail)
    cage_pos = ctx.part_world_position(cage)
    centered_between_rails = (
        frame_pos is not None
        and left_pos is not None
        and right_pos is not None
        and cage_pos is not None
        and abs(cage_pos[0] - ((left_pos[0] + right_pos[0]) * 0.5)) < 1e-6
        and left_pos[0] < cage_pos[0] < right_pos[0]
    )
    ctx.check(
        "cage_runs_between_guide_rails",
        centered_between_rails,
        details=f"frame={frame_pos}, left={left_pos}, cage={cage_pos}, right={right_pos}",
    )

    ctx.expect_contact(left_rail, frame, name="left_rail_attached_to_frame")
    ctx.expect_contact(right_rail, frame, name="right_rail_attached_to_frame")
    ctx.expect_contact(cage, left_rail, name="cage_contacts_left_rail")
    ctx.expect_contact(cage, right_rail, name="cage_contacts_right_rail")
    ctx.expect_contact(gate, cage, name="gate_attached_to_cage")
    ctx.expect_contact(sheave, frame, name="sheave_supported_by_head_frame")

    with ctx.pose({gate_hinge: 0.0}):
        ctx.expect_gap(
            gate,
            cage,
            axis="y",
            min_gap=0.0,
            max_gap=0.003,
            max_penetration=0.0,
            name="closed_gate_seats_at_cage_front",
        )
        ctx.expect_overlap(
            gate,
            cage,
            axes="xz",
            min_overlap=1.0,
            name="closed_gate_covers_front_opening",
        )

    with ctx.pose({gate_hinge: 1.2}):
        ctx.expect_origin_distance(
            gate,
            cage,
            axes="xy",
            min_dist=0.65,
            max_dist=0.85,
            name="open_gate_swings_clear_of_front_opening",
        )

    with ctx.pose({cage_slide: 4.0}):
        ctx.expect_contact(cage, left_rail, name="raised_cage_still_guided_left")
        ctx.expect_contact(cage, right_rail, name="raised_cage_still_guided_right")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
