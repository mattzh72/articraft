from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


BRACKET_PLATE_T = 0.012
BRACKET_PLATE_W = 0.120
BRACKET_PLATE_H = 0.170
BRACKET_ARM_LENGTH = 0.062
BRACKET_ARM_W = 0.056
BRACKET_ARM_T = 0.010
BRACKET_GAP = 0.034
BRACKET_ARM_Z = (BRACKET_GAP / 2.0) + (BRACKET_ARM_T / 2.0)
PIVOT_BOSS_R = 0.023
BRACKET_GUSSET_L = 0.040
BRACKET_GUSSET_W = 0.010
BRACKET_GUSSET_H = 0.052
CAP_R = 0.026
CAP_T = 0.004
THRUST_PAD_R = 0.023
THRUST_PAD_T = 0.003
THRUST_PAD_Z = (BRACKET_GAP / 2.0) - (THRUST_PAD_T / 2.0)

HUB_R = 0.018
HUB_H = 0.020
COLLAR_R = 0.021
COLLAR_T = 0.004
COLLAR_Z = (HUB_H / 2.0) + (COLLAR_T / 2.0)
ROOT_NECK_LEN = 0.092
ROOT_NECK_W = 0.040
ROOT_NECK_H = (2.0 * THRUST_PAD_Z)

LINK_START_X = 0.075
LINK_LENGTH = 0.465
LINK_END_X = LINK_START_X + LINK_LENGTH
LINK_W = 0.090
LINK_H = 0.060

CAVITY_START_X = 0.150
CAVITY_LENGTH = 0.375
CAVITY_W = 0.060
CAVITY_H = 0.040

TOP_SLOT_START_X = 0.225
TOP_SLOT_LENGTH = 0.290
TOP_SLOT_W = 0.036
TOP_SLOT_H = 0.020

FRONT_SLOT_LEN = 0.030
FRONT_SLOT_W = 0.034
FRONT_SLOT_H = 0.028

WEAR_STRIP_LEN = 0.250
WEAR_STRIP_W = 0.004
WEAR_STRIP_H = 0.024
WEAR_STRIP_X = 0.385
WEAR_STRIP_Y = (CAVITY_W / 2.0) - (WEAR_STRIP_W / 2.0)

SHOE_LEN = 0.150
SHOE_W = 0.052
SHOE_H = 0.024
STEM_LEN = 0.056
STEM_W = 0.026
STEM_H = 0.052
STEM_CENTER_X = 0.055
STEM_CENTER_Z = (SHOE_H / 2.0) + (STEM_H / 2.0)
UPPER_BLOCK_LEN = 0.140
UPPER_BLOCK_W = 0.100
UPPER_BLOCK_H = 0.034
UPPER_BLOCK_CENTER_X = 0.140
UPPER_BLOCK_CENTER_Z = 0.066
FRONT_FACE_T = 0.010
FRONT_FACE_W = 0.114
FRONT_FACE_H = 0.050
FRONT_FACE_CENTER_X = UPPER_BLOCK_CENTER_X + (UPPER_BLOCK_LEN / 2.0) + (FRONT_FACE_T / 2.0)
FRONT_FACE_CENTER_Z = UPPER_BLOCK_CENTER_Z
SLIDE_ORIGIN_X = 0.330
SLIDE_TRAVEL = 0.100

SWING_LIMIT = 0.70


def _make_wall_bracket_body() -> cq.Workplane:
    back_plate = cq.Workplane("XY").box(
        BRACKET_PLATE_T,
        BRACKET_PLATE_W,
        BRACKET_PLATE_H,
    ).translate((-(BRACKET_ARM_LENGTH + BRACKET_PLATE_T) / 2.0, 0.0, 0.0))

    upper_arm = cq.Workplane("XY").box(
        BRACKET_ARM_LENGTH,
        BRACKET_ARM_W,
        BRACKET_ARM_T,
    ).translate((-(BRACKET_ARM_LENGTH / 2.0) - 0.010, 0.0, BRACKET_ARM_Z))

    lower_arm = cq.Workplane("XY").box(
        BRACKET_ARM_LENGTH,
        BRACKET_ARM_W,
        BRACKET_ARM_T,
    ).translate((-(BRACKET_ARM_LENGTH / 2.0) - 0.010, 0.0, -BRACKET_ARM_Z))

    left_gusset = cq.Workplane("XY").box(
        BRACKET_GUSSET_L,
        BRACKET_GUSSET_W,
        BRACKET_GUSSET_H,
    ).translate((-(BRACKET_GUSSET_L / 2.0) - 0.010, 0.023, 0.0))
    right_gusset = cq.Workplane("XY").box(
        BRACKET_GUSSET_L,
        BRACKET_GUSSET_W,
        BRACKET_GUSSET_H,
    ).translate((-(BRACKET_GUSSET_L / 2.0) - 0.010, -0.023, 0.0))

    return back_plate.union(upper_arm).union(lower_arm).union(left_gusset).union(right_gusset)


def _make_main_link_body() -> cq.Workplane:
    root_neck = cq.Workplane("XY").box(
        ROOT_NECK_LEN,
        ROOT_NECK_W,
        ROOT_NECK_H,
    ).translate((ROOT_NECK_LEN / 2.0, 0.0, 0.0))
    hub = cq.Workplane("XY").cylinder(HUB_H, HUB_R)

    beam = cq.Workplane("XY").box(
        LINK_LENGTH,
        LINK_W,
        LINK_H,
    ).translate((LINK_START_X + (LINK_LENGTH / 2.0), 0.0, 0.0))

    cavity = cq.Workplane("XY").box(
        CAVITY_LENGTH,
        CAVITY_W,
        CAVITY_H,
    ).translate((CAVITY_START_X + (CAVITY_LENGTH / 2.0), 0.0, 0.0))
    beam = beam.cut(cavity)

    top_slot = cq.Workplane("XY").box(
        TOP_SLOT_LENGTH,
        TOP_SLOT_W,
        TOP_SLOT_H,
    ).translate(
        (
            TOP_SLOT_START_X + (TOP_SLOT_LENGTH / 2.0),
            0.0,
            (LINK_H / 2.0) - (TOP_SLOT_H / 2.0),
        )
    )
    beam = beam.cut(top_slot)

    front_slot = cq.Workplane("XY").box(
        FRONT_SLOT_LEN,
        FRONT_SLOT_W,
        FRONT_SLOT_H,
    ).translate((LINK_END_X - (FRONT_SLOT_LEN / 2.0), 0.0, 0.0))
    beam = beam.cut(front_slot)

    shoulder = cq.Workplane("XY").box(
        0.050,
        0.066,
        LINK_H,
    ).translate((0.105, 0.0, 0.0))

    return root_neck.union(hub).union(shoulder).union(beam)


def _make_carriage_head() -> cq.Workplane:
    stem = cq.Workplane("XY").box(
        STEM_LEN,
        STEM_W,
        STEM_H,
    ).translate((STEM_CENTER_X, 0.0, STEM_CENTER_Z))

    upper_block = cq.Workplane("XY").box(
        UPPER_BLOCK_LEN,
        UPPER_BLOCK_W,
        UPPER_BLOCK_H,
    ).translate((UPPER_BLOCK_CENTER_X, 0.0, UPPER_BLOCK_CENTER_Z))

    return stem.union(upper_block)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hinged_support_arm")

    model.material("bracket_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("arm_silver", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("carriage_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("polymer_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("cap_black", rgba=(0.08, 0.08, 0.09, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        mesh_from_cadquery(_make_wall_bracket_body(), "wall_bracket_body"),
        material="bracket_steel",
        name="bracket_body",
    )
    wall_bracket.visual(
        Cylinder(radius=CAP_R, length=CAP_T),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_ARM_Z + (BRACKET_ARM_T / 2.0) + (CAP_T / 2.0))),
        material="cap_black",
        name="upper_cap",
    )
    wall_bracket.visual(
        Cylinder(radius=CAP_R, length=CAP_T),
        origin=Origin(xyz=(0.0, 0.0, -BRACKET_ARM_Z - (BRACKET_ARM_T / 2.0) - (CAP_T / 2.0))),
        material="cap_black",
        name="lower_cap",
    )
    wall_bracket.visual(
        Cylinder(radius=THRUST_PAD_R, length=THRUST_PAD_T),
        origin=Origin(xyz=(0.0, 0.0, THRUST_PAD_Z)),
        material="polymer_black",
        name="upper_thrust_pad",
    )
    wall_bracket.visual(
        Cylinder(radius=THRUST_PAD_R, length=THRUST_PAD_T),
        origin=Origin(xyz=(0.0, 0.0, -THRUST_PAD_Z)),
        material="polymer_black",
        name="lower_thrust_pad",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_ARM_LENGTH + BRACKET_PLATE_T, BRACKET_PLATE_W, BRACKET_PLATE_H)),
        mass=2.8,
        origin=Origin(xyz=(-(BRACKET_ARM_LENGTH + BRACKET_PLATE_T) / 2.0, 0.0, 0.0)),
    )

    main_link = model.part("main_link")
    main_link.visual(
        mesh_from_cadquery(_make_main_link_body(), "main_link_body"),
        material="arm_silver",
        name="link_body",
    )
    main_link.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_T),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material="cap_black",
        name="upper_collar",
    )
    main_link.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_T),
        origin=Origin(xyz=(0.0, 0.0, -COLLAR_Z)),
        material="cap_black",
        name="lower_collar",
    )
    main_link.visual(
        Box((WEAR_STRIP_LEN, WEAR_STRIP_W, WEAR_STRIP_H)),
        origin=Origin(xyz=(WEAR_STRIP_X, WEAR_STRIP_Y, 0.0)),
        material="polymer_black",
        name="left_wear_strip",
    )
    main_link.visual(
        Box((WEAR_STRIP_LEN, WEAR_STRIP_W, WEAR_STRIP_H)),
        origin=Origin(xyz=(WEAR_STRIP_X, -WEAR_STRIP_Y, 0.0)),
        material="polymer_black",
        name="right_wear_strip",
    )
    main_link.inertial = Inertial.from_geometry(
        Box((LINK_END_X, LINK_W, LINK_H)),
        mass=3.4,
        origin=Origin(xyz=(LINK_END_X / 2.0, 0.0, 0.0)),
    )

    tip_carriage = model.part("tip_carriage")
    tip_carriage.visual(
        mesh_from_cadquery(_make_carriage_head(), "tip_carriage_head"),
        material="carriage_silver",
        name="carriage_head",
    )
    tip_carriage.visual(
        Box((SHOE_LEN, SHOE_W, SHOE_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="carriage_silver",
        name="guide_shoe",
    )
    tip_carriage.visual(
        Box((FRONT_FACE_T, FRONT_FACE_W, FRONT_FACE_H)),
        origin=Origin(xyz=(FRONT_FACE_CENTER_X, 0.0, FRONT_FACE_CENTER_Z)),
        material="carriage_silver",
        name="front_face",
    )
    tip_carriage.inertial = Inertial.from_geometry(
        Box((FRONT_FACE_CENTER_X + (FRONT_FACE_T / 2.0) + (SHOE_LEN / 2.0), FRONT_FACE_W, 0.086)),
        mass=1.1,
        origin=Origin(
            xyz=(
                ((FRONT_FACE_CENTER_X + (FRONT_FACE_T / 2.0)) - (SHOE_LEN / 2.0)) / 2.0,
                0.0,
                0.031,
            )
        ),
    )

    model.articulation(
        "wall_to_link",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=main_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=-SWING_LIMIT,
            upper=SWING_LIMIT,
        ),
    )
    model.articulation(
        "link_to_carriage",
        ArticulationType.PRISMATIC,
        parent=main_link,
        child=tip_carriage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    main_link = object_model.get_part("main_link")
    tip_carriage = object_model.get_part("tip_carriage")

    swing = object_model.get_articulation("wall_to_link")
    slide = object_model.get_articulation("link_to_carriage")

    upper_thrust_pad = wall_bracket.get_visual("upper_thrust_pad")
    lower_thrust_pad = wall_bracket.get_visual("lower_thrust_pad")
    upper_collar = main_link.get_visual("upper_collar")
    lower_collar = main_link.get_visual("lower_collar")
    link_body = main_link.get_visual("link_body")
    left_wear_strip = main_link.get_visual("left_wear_strip")
    right_wear_strip = main_link.get_visual("right_wear_strip")
    carriage_head = tip_carriage.get_visual("carriage_head")
    guide_shoe = tip_carriage.get_visual("guide_shoe")
    front_face = tip_carriage.get_visual("front_face")

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
        "motion_stack_is_swing_then_extension",
        swing.parent == wall_bracket.name and swing.child == main_link.name and slide.parent == main_link.name,
        "Expected a wall-bracket revolute feeding a distal prismatic carriage.",
    )
    ctx.check(
        "swing_axis_is_vertical",
        tuple(swing.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical swing axis, got {tuple(swing.axis)}.",
    )
    ctx.check(
        "extension_axis_follows_link",
        tuple(slide.axis) == (1.0, 0.0, 0.0) and slide.motion_limits is not None and slide.motion_limits.lower == 0.0,
        "Expected the tip carriage to extend outward along the main link.",
    )

    with ctx.pose({swing: 0.0, slide: 0.0}):
        ctx.expect_contact(
            wall_bracket,
            main_link,
            elem_a=upper_thrust_pad,
            elem_b=upper_collar,
            contact_tol=5e-4,
            name="upper_trunnion_is_supported",
        )
        ctx.expect_contact(
            wall_bracket,
            main_link,
            elem_a=lower_thrust_pad,
            elem_b=lower_collar,
            contact_tol=5e-4,
            name="lower_trunnion_is_supported",
        )
        ctx.expect_contact(
            main_link,
            tip_carriage,
            elem_a=left_wear_strip,
            elem_b=guide_shoe,
            contact_tol=5e-4,
            name="left_guide_strip_supports_carriage",
        )
        ctx.expect_contact(
            main_link,
            tip_carriage,
            elem_a=right_wear_strip,
            elem_b=guide_shoe,
            contact_tol=5e-4,
            name="right_guide_strip_supports_carriage",
        )
        ctx.expect_within(
            tip_carriage,
            main_link,
            axes="xyz",
            inner_elem=guide_shoe,
            outer_elem=link_body,
            margin=5e-4,
            name="guide_shoe_stays_inside_link_channel_at_rest",
        )
        ctx.expect_gap(
            tip_carriage,
            main_link,
            axis="x",
            positive_elem=front_face,
            negative_elem=link_body,
            min_gap=0.0,
            max_gap=0.002,
            name="carriage_seats_at_distal_nose_when_retracted",
        )

    with ctx.pose({swing: 0.0, slide: SLIDE_TRAVEL}):
        ctx.expect_gap(
            tip_carriage,
            main_link,
            axis="x",
            positive_elem=front_face,
            negative_elem=link_body,
            min_gap=0.099,
            max_gap=0.101,
            name="carriage_projects_forward_when_extended",
        )

    with ctx.pose({swing: SWING_LIMIT, slide: SLIDE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_swung_and_extended_pose")
        ctx.expect_within(
            tip_carriage,
            main_link,
            axes="xyz",
            inner_elem=guide_shoe,
            outer_elem=link_body,
            margin=5e-4,
            name="guide_shoe_stays_inside_link_channel_when_extended",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulation_paths_remain_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
