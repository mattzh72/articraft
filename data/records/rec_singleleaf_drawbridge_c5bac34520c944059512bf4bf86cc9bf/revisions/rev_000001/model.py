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
    model = ArticulatedObject(name="singleleaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.65, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.30, 0.33, 0.36, 1.0))
    deck_paint = model.material("deck_paint", rgba=(0.36, 0.39, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    wear_steel = model.material("wear_steel", rgba=(0.50, 0.52, 0.54, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.80, 0.67, 0.15, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((3.20, 6.20, 0.90)),
        origin=Origin(xyz=(-1.80, 0.0, 0.45)),
        material=concrete,
        name="foundation_block",
    )
    support_frame.visual(
        Box((2.20, 4.40, 0.18)),
        origin=Origin(xyz=(-1.30, 0.0, 0.99)),
        material=concrete,
        name="approach_slab",
    )
    support_frame.visual(
        Box((1.25, 4.40, 0.08)),
        origin=Origin(xyz=(-0.95, 0.0, 0.94)),
        material=frame_paint,
        name="service_deck",
    )
    support_frame.visual(
        Box((0.82, 5.40, 0.30)),
        origin=Origin(xyz=(-0.92, 0.0, 1.32)),
        material=frame_paint,
        name="rear_tie_beam",
    )
    support_frame.visual(
        Box((0.70, 4.90, 0.20)),
        origin=Origin(xyz=(-0.86, 0.0, 1.06)),
        material=dark_steel,
        name="lower_torsion_beam",
    )
    support_frame.visual(
        Box((0.72, 1.10, 0.98)),
        origin=Origin(xyz=(-1.92, 1.55, 1.39)),
        material=frame_paint,
        name="left_drive_cabinet",
    )
    support_frame.visual(
        Box((0.72, 1.10, 0.98)),
        origin=Origin(xyz=(-1.92, -1.55, 1.39)),
        material=frame_paint,
        name="right_drive_cabinet",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        y_tower = side_sign * 2.55
        y_inner = side_sign * 2.23
        y_outer = side_sign * 2.87

        support_frame.visual(
            Box((0.58, 0.18, 0.92)),
            origin=Origin(xyz=(-0.55, y_inner, 1.30)),
            material=frame_paint,
            name=f"{side_name}_inner_web",
        )
        support_frame.visual(
            Box((0.58, 0.18, 0.92)),
            origin=Origin(xyz=(-0.55, y_outer, 1.30)),
            material=frame_paint,
            name=f"{side_name}_outer_web",
        )
        support_frame.visual(
            Box((0.58, 0.90, 0.62)),
            origin=Origin(xyz=(-0.55, y_tower, 1.21)),
            material=dark_steel,
            name=f"{side_name}_front_cheek",
        )
        support_frame.visual(
            Box((0.54, 0.96, 0.34)),
            origin=Origin(xyz=(-0.57, y_tower, 1.69)),
            material=frame_paint,
            name=f"{side_name}_back_cheek",
        )
        support_frame.visual(
            Box((0.14, 0.60, 0.40)),
            origin=Origin(xyz=(-0.27, y_tower, 1.02)),
            material=wear_steel,
            name=f"{side_name}_saddle_pad",
        )
        support_frame.visual(
            Box((0.18, 0.72, 0.14)),
            origin=Origin(xyz=(-0.37, y_tower, 1.43)),
            material=frame_paint,
            name=f"{side_name}_bearing_cap",
        )
        support_frame.visual(
            Box((0.62, 0.96, 0.20)),
            origin=Origin(xyz=(-0.86, y_tower, 1.89)),
            material=frame_paint,
            name=f"{side_name}_tower_crown",
        )
        support_frame.visual(
            Box((0.94, 0.18, 0.22)),
            origin=Origin(xyz=(-0.92, y_tower, 1.36)),
            material=frame_paint,
            name=f"{side_name}_diagonal_brace",
        )
        support_frame.visual(
            Box((0.46, 0.22, 0.14)),
            origin=Origin(xyz=(-0.92, y_tower, 1.56)),
            material=dark_steel,
            name=f"{side_name}_service_step",
        )

    support_frame.inertial = Inertial.from_geometry(
        Box((3.20, 6.20, 2.10)),
        mass=68000.0,
        origin=Origin(xyz=(-1.80, 0.0, 1.05)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((6.35, 4.10, 0.12)),
        origin=Origin(xyz=(3.23, 0.0, 0.28)),
        material=deck_paint,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((0.90, 3.10, 0.18)),
        origin=Origin(xyz=(0.35, 0.0, 0.14)),
        material=deck_paint,
        name="heel_deck",
    )
    bridge_leaf.visual(
        Box((6.00, 0.34, 0.46)),
        origin=Origin(xyz=(3.10, 1.95, 0.03)),
        material=frame_paint,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((6.00, 0.34, 0.46)),
        origin=Origin(xyz=(3.10, -1.95, 0.03)),
        material=frame_paint,
        name="right_girder",
    )
    for index, x_pos in enumerate((0.72, 1.92, 3.12, 4.32, 5.52), start=1):
        bridge_leaf.visual(
            Box((0.22, 3.62, 0.24)),
            origin=Origin(xyz=(x_pos, 0.0, 0.02)),
            material=dark_steel,
            name=f"crossbeam_{index}",
        )
    bridge_leaf.visual(
        Box((0.74, 0.38, 0.30)),
        origin=Origin(xyz=(0.22, 2.23, 0.14)),
        material=frame_paint,
        name="left_trunnion_arm",
    )
    bridge_leaf.visual(
        Box((0.74, 0.38, 0.30)),
        origin=Origin(xyz=(0.22, -2.23, 0.14)),
        material=frame_paint,
        name="right_trunnion_arm",
    )
    bridge_leaf.visual(
        Box((0.30, 0.18, 0.32)),
        origin=Origin(xyz=(0.02, 2.41, 0.10)),
        material=dark_steel,
        name="left_trunnion_neck",
    )
    bridge_leaf.visual(
        Box((0.30, 0.18, 0.32)),
        origin=Origin(xyz=(0.02, -2.41, 0.10)),
        material=dark_steel,
        name="right_trunnion_neck",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.20, length=0.46),
        origin=Origin(xyz=(0.0, 2.55, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.20, length=0.46),
        origin=Origin(xyz=(0.0, -2.55, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="right_trunnion",
    )
    bridge_leaf.visual(
        Box((0.18, 0.28, 0.28)),
        origin=Origin(xyz=(0.08, 2.28, 0.02)),
        material=dark_steel,
        name="left_hub",
    )
    bridge_leaf.visual(
        Box((0.18, 0.28, 0.28)),
        origin=Origin(xyz=(0.08, -2.28, 0.02)),
        material=dark_steel,
        name="right_hub",
    )
    bridge_leaf.visual(
        Box((0.26, 4.28, 0.28)),
        origin=Origin(xyz=(6.15, 0.0, 0.06)),
        material=frame_paint,
        name="nose_beam",
    )
    bridge_leaf.visual(
        Box((0.18, 0.82, 0.10)),
        origin=Origin(xyz=(6.19, 0.0, -0.08)),
        material=safety_yellow,
        name="toe_wear_shoe",
    )
    bridge_leaf.visual(
        Box((5.90, 0.16, 0.12)),
        origin=Origin(xyz=(3.18, 2.13, 0.40)),
        material=safety_yellow,
        name="left_edge_curb",
    )
    bridge_leaf.visual(
        Box((5.90, 0.16, 0.12)),
        origin=Origin(xyz=(3.18, -2.13, 0.40)),
        material=safety_yellow,
        name="right_edge_curb",
    )
    bridge_leaf.visual(
        Box((4.90, 0.32, 0.16)),
        origin=Origin(xyz=(3.05, 0.0, -0.14)),
        material=dark_steel,
        name="center_torque_tube_cover",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((6.70, 4.70, 0.70)),
        mass=18000.0,
        origin=Origin(xyz=(3.00, 0.0, 0.10)),
    )

    model.articulation(
        "leaf_trunnion",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.22)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220000.0,
            velocity=0.30,
            lower=0.0,
            upper=1.20,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_joint = object_model.get_articulation("leaf_trunnion")

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

    limits = leaf_joint.motion_limits
    ctx.check(
        "leaf_joint_configured_as_upward_trunnion",
        leaf_joint.axis == (0.0, -1.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 1.0,
        details=f"axis={leaf_joint.axis}, limits={limits}",
    )

    with ctx.pose({leaf_joint: 0.0}):
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="left_trunnion",
            elem_b="left_saddle_pad",
            contact_tol=0.002,
            name="left_trunnion_seated_on_wear_pad",
        )
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="right_trunnion",
            elem_b="right_saddle_pad",
            contact_tol=0.002,
            name="right_trunnion_seated_on_wear_pad",
        )
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            positive_elem="heel_deck",
            negative_elem="service_deck",
            min_gap=0.22,
            max_gap=0.36,
            name="maintenance_access_under_heel",
        )

    with ctx.pose({leaf_joint: 0.0}):
        closed_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
    with ctx.pose({leaf_joint: 1.05}):
        open_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")

    if closed_nose is None or open_nose is None:
        ctx.fail("nose_beam_pose_measurement", "Could not measure nose beam in closed and open poses.")
    else:
        closed_center = tuple((a + b) * 0.5 for a, b in zip(closed_nose[0], closed_nose[1]))
        open_center = tuple((a + b) * 0.5 for a, b in zip(open_nose[0], open_nose[1]))
        ctx.check(
            "leaf_opens_upward_and_back",
            open_center[2] > closed_center[2] + 3.0 and open_center[0] < closed_center[0] - 2.0,
            details=f"closed_center={closed_center}, open_center={open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
