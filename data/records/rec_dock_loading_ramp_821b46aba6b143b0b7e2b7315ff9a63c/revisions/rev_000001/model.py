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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pivoting_dock_board")

    frame_gray = model.material("frame_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.77, 0.14, 1.0))
    pin_black = model.material("pin_black", rgba=(0.12, 0.12, 0.13, 1.0))

    deck_length = 1.72
    deck_width = 1.38
    deck_plate_thickness = 0.012
    deck_plate_center_x = 0.88
    deck_plate_center_z = 0.028
    deck_top_z = deck_plate_center_z + deck_plate_thickness / 2.0
    side_hinge_y = 0.715
    side_hinge_x = 0.88
    side_hinge_z = 0.022
    curb_barrel_length = 1.38
    curb_barrel_radius = 0.017

    dock_frame = model.part("dock_frame")
    dock_frame.visual(
        Box((0.10, 1.58, 0.22)),
        origin=Origin(xyz=(-0.05, 0.0, 0.11)),
        material=frame_gray,
        name="frame_back_plate",
    )
    dock_frame.visual(
        Box((0.16, 1.58, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=frame_gray,
        name="frame_cap_beam",
    )
    dock_frame.visual(
        Box((0.06, 0.03, 0.12)),
        origin=Origin(xyz=(0.08, 0.725, 0.245)),
        material=frame_gray,
        name="frame_right_cheek",
    )
    dock_frame.visual(
        Box((0.06, 0.03, 0.12)),
        origin=Origin(xyz=(0.08, -0.725, 0.245)),
        material=frame_gray,
        name="frame_left_cheek",
    )
    dock_frame.visual(
        Box((0.08, 0.10, 0.10)),
        origin=Origin(xyz=(-0.01, 0.70, 0.19)),
        material=frame_gray,
        name="frame_right_gusset",
    )
    dock_frame.visual(
        Box((0.08, 0.10, 0.10)),
        origin=Origin(xyz=(-0.01, -0.70, 0.19)),
        material=frame_gray,
        name="frame_left_gusset",
    )

    deck = model.part("deck")
    deck.visual(
        Cylinder(radius=0.022, length=1.42),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_black,
        name="deck_hinge_barrel",
    )
    deck.visual(
        Box((deck_length, deck_width, deck_plate_thickness)),
        origin=Origin(xyz=(deck_plate_center_x, 0.0, deck_plate_center_z)),
        material=deck_steel,
        name="deck_plate",
    )
    deck.visual(
        Box((0.12, 1.34, 0.010)),
        origin=Origin(xyz=(1.68, 0.0, 0.029)),
        material=deck_steel,
        name="front_nose",
    )
    deck.visual(
        Box((0.06, deck_width, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, -0.028)),
        material=deck_steel,
        name="rear_cleat_bar",
    )

    for index, y_pos in enumerate((-0.52, -0.31, -0.10, 0.10, 0.31, 0.52), start=1):
        deck.visual(
            Box((1.62, 0.03, 0.004)),
            origin=Origin(xyz=(0.86, y_pos, deck_top_z + 0.002)),
            material=deck_steel,
            name=f"top_rib_{index}",
        )

    for index, y_pos in enumerate((-0.40, 0.0, 0.40), start=1):
        deck.visual(
            Box((1.60, 0.08, 0.05)),
            origin=Origin(xyz=(0.84, y_pos, -0.003)),
            material=deck_steel,
            name=f"underside_beam_{index}",
        )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        deck.visual(
            Box((1.66, 0.06, 0.05)),
            origin=Origin(xyz=(0.85, side_sign * 0.66, -0.003)),
            material=deck_steel,
            name=f"{side_name}_edge_rail",
        )
        for end_name, x_pos in (("rear", 0.17), ("front", 1.59)):
            deck.visual(
                Box((0.04, 0.05, 0.10)),
                origin=Origin(xyz=(x_pos, side_sign * 0.673, 0.04)),
                material=frame_gray,
                name=f"{side_name}_{end_name}_curb_mount",
            )

    for index, y_pos in enumerate((-0.52, -0.26, 0.0, 0.26, 0.52), start=1):
        deck.visual(
            Box((0.02, 0.16, 0.025)),
            origin=Origin(xyz=(0.04, y_pos, -0.066)),
            material=pin_black,
            name=f"rear_cleat_tooth_{index}",
        )

    def add_curb(part_name: str, side_sign: float) -> None:
        curb = model.part(part_name)
        side_label = "left" if side_sign < 0.0 else "right"
        curb.visual(
            Cylinder(radius=curb_barrel_radius, length=curb_barrel_length),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_black,
            name=f"{side_label}_curb_barrel",
        )
        curb.visual(
            Box((1.38, 0.028, 0.15)),
            origin=Origin(xyz=(0.0, side_sign * 0.026, 0.092)),
            material=safety_yellow,
            name=f"{side_label}_curb_wall",
        )
        curb.visual(
            Box((1.38, 0.04, 0.02)),
            origin=Origin(xyz=(0.0, side_sign * 0.020, 0.175)),
            material=safety_yellow,
            name=f"{side_label}_curb_cap",
        )
        for index, x_pos in enumerate((-0.50, 0.0, 0.50), start=1):
            curb.visual(
                Box((0.05, 0.018, 0.08)),
                origin=Origin(xyz=(x_pos, side_sign * 0.012, 0.055)),
                material=safety_yellow,
                name=f"{side_label}_curb_post_{index}",
            )

    add_curb("left_curb", -1.0)
    add_curb("right_curb", 1.0)

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=dock_frame,
        child=deck,
        origin=Origin(xyz=(0.08, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.8,
            lower=-0.30,
            upper=0.30,
        ),
    )
    model.articulation(
        "deck_to_left_curb",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="left_curb",
        origin=Origin(xyz=(side_hinge_x, -side_hinge_y, side_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.2,
            lower=0.0,
            upper=1.50,
        ),
    )
    model.articulation(
        "deck_to_right_curb",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="right_curb",
        origin=Origin(xyz=(side_hinge_x, side_hinge_y, side_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.2,
            lower=0.0,
            upper=1.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock_frame = object_model.get_part("dock_frame")
    deck = object_model.get_part("deck")
    left_curb = object_model.get_part("left_curb")
    right_curb = object_model.get_part("right_curb")
    deck_hinge = object_model.get_articulation("frame_to_deck")
    left_hinge = object_model.get_articulation("deck_to_left_curb")
    right_hinge = object_model.get_articulation("deck_to_right_curb")

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
        "expected parts and joints exist",
        all(
            item is not None
            for item in (dock_frame, deck, left_curb, right_curb, deck_hinge, left_hinge, right_hinge)
        ),
        details="One or more authored parts or articulations could not be resolved.",
    )
    ctx.check(
        "hinge axes match dock-board mechanics",
        deck_hinge.axis == (0.0, -1.0, 0.0)
        and left_hinge.axis == (1.0, 0.0, 0.0)
        and right_hinge.axis == (-1.0, 0.0, 0.0),
        details=(
            f"deck={deck_hinge.axis}, left={left_hinge.axis}, right={right_hinge.axis}"
        ),
    )

    ctx.expect_contact(
        deck,
        dock_frame,
        elem_a="deck_hinge_barrel",
        elem_b="frame_left_cheek",
        name="deck hinge barrel touches left frame cheek",
    )
    ctx.expect_contact(
        deck,
        dock_frame,
        elem_a="deck_hinge_barrel",
        elem_b="frame_right_cheek",
        name="deck hinge barrel touches right frame cheek",
    )
    ctx.expect_contact(
        left_curb,
        deck,
        elem_a="left_curb_barrel",
        elem_b="left_front_curb_mount",
        name="left curb hinge barrel is captured at the front mount",
    )
    ctx.expect_contact(
        right_curb,
        deck,
        elem_a="right_curb_barrel",
        elem_b="right_rear_curb_mount",
        name="right curb hinge barrel is captured at the rear mount",
    )

    def center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) * 0.5 if aabb is not None else None

    def center_y(aabb):
        return (aabb[0][1] + aabb[1][1]) * 0.5 if aabb is not None else None

    rest_front = ctx.part_element_world_aabb(deck, elem="front_nose")
    lower_limit = deck_hinge.motion_limits.lower if deck_hinge.motion_limits is not None else None
    upper_limit = deck_hinge.motion_limits.upper if deck_hinge.motion_limits is not None else None

    lowered_front = None
    raised_front = None
    if lower_limit is not None:
        with ctx.pose({deck_hinge: lower_limit}):
            lowered_front = ctx.part_element_world_aabb(deck, elem="front_nose")
            ctx.fail_if_parts_overlap_in_current_pose(name="lowered deck pose remains clear")
    if upper_limit is not None:
        with ctx.pose({deck_hinge: upper_limit}):
            raised_front = ctx.part_element_world_aabb(deck, elem="front_nose")
            ctx.fail_if_parts_overlap_in_current_pose(name="raised deck pose remains clear")

    ctx.check(
        "deck rocks both above and below level",
        center_z(lowered_front) is not None
        and center_z(rest_front) is not None
        and center_z(raised_front) is not None
        and center_z(lowered_front) < center_z(rest_front) - 0.06
        and center_z(raised_front) > center_z(rest_front) + 0.06,
        details=(
            f"lowered={center_z(lowered_front)}, rest={center_z(rest_front)}, "
            f"raised={center_z(raised_front)}"
        ),
    )

    left_folded = None
    right_folded = None
    left_rest = ctx.part_element_world_aabb(left_curb, elem="left_curb_cap")
    right_rest = ctx.part_element_world_aabb(right_curb, elem="right_curb_cap")
    left_upper = left_hinge.motion_limits.upper if left_hinge.motion_limits is not None else None
    right_upper = right_hinge.motion_limits.upper if right_hinge.motion_limits is not None else None
    if left_upper is not None and right_upper is not None:
        with ctx.pose({left_hinge: left_upper, right_hinge: right_upper}):
            left_folded = ctx.part_element_world_aabb(left_curb, elem="left_curb_cap")
            right_folded = ctx.part_element_world_aabb(right_curb, elem="right_curb_cap")
            ctx.fail_if_parts_overlap_in_current_pose(name="folded curbs clear the deck")

    ctx.check(
        "side curbs fold outward from the deck edges",
        center_y(left_folded) is not None
        and center_y(left_rest) is not None
        and center_y(right_folded) is not None
        and center_y(right_rest) is not None
        and center_y(left_folded) < center_y(left_rest) - 0.08
        and center_y(right_folded) > center_y(right_rest) + 0.08,
        details=(
            f"left_rest={center_y(left_rest)}, left_folded={center_y(left_folded)}, "
            f"right_rest={center_y(right_rest)}, right_folded={center_y(right_folded)}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
