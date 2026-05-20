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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_singleleaf_drawbridge")

    frame_paint = model.material("frame_paint", rgba=(0.25, 0.27, 0.30, 1.0))
    bridge_paint = model.material("bridge_paint", rgba=(0.36, 0.43, 0.46, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.57, 0.60, 0.62, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    accent = model.material("accent", rgba=(0.71, 0.58, 0.22, 1.0))

    hinge_axis_z = 0.082

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.300, 0.240, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=frame_paint,
        name="base_plinth",
    )
    support_frame.visual(
        Box((0.080, 0.130, 0.062)),
        origin=Origin(xyz=(-0.105, 0.0, 0.049)),
        material=frame_paint,
        name="machine_box",
    )
    support_frame.visual(
        Box((0.060, 0.120, 0.010)),
        origin=Origin(xyz=(-0.095, 0.0, 0.085)),
        material=deck_steel,
        name="service_deck",
    )
    support_frame.visual(
        Box((0.050, 0.090, 0.020)),
        origin=Origin(xyz=(-0.100, 0.0, 0.090)),
        material=accent,
        name="counter_cover",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        support_frame.visual(
            Box((0.155, 0.020, 0.038)),
            origin=Origin(xyz=(-0.005, sign * 0.088, 0.037)),
            material=frame_paint,
            name=f"{side}_cheek_wall",
        )
        support_frame.visual(
            Box((0.050, 0.020, 0.070)),
            origin=Origin(xyz=(-0.002, sign * 0.108, 0.053)),
            material=frame_paint,
            name=f"{side}_bearing_tower",
        )
        support_frame.visual(
            Cylinder(radius=0.022, length=0.020),
            origin=Origin(
                xyz=(0.0, sign * 0.108, hinge_axis_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=bearing_steel,
            name=f"{side}_bearing_housing",
        )
        support_frame.visual(
            Box((0.028, 0.020, 0.024)),
            origin=Origin(xyz=(-0.018, sign * 0.108, hinge_axis_z + 0.020)),
            material=bearing_steel,
            name=f"{side}_bearing_cap",
        )
    support_frame.visual(
        Box((0.030, 0.090, 0.062)),
        origin=Origin(xyz=(0.065, 0.0, 0.049)),
        material=deck_steel,
        name="mid_rest_pad",
    )
    support_frame.visual(
        Box((0.030, 0.090, 0.062)),
        origin=Origin(xyz=(0.165, 0.0, 0.049)),
        material=deck_steel,
        name="forward_rest_pad",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.300, 0.240, 0.120)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((0.220, 0.130, 0.010)),
        origin=Origin(xyz=(0.112, 0.0, 0.003)),
        material=deck_steel,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((0.030, 0.130, 0.030)),
        origin=Origin(xyz=(0.015, 0.0, -0.006)),
        material=bridge_paint,
        name="root_crossbeam",
    )
    bridge_leaf.visual(
        Box((0.018, 0.130, 0.012)),
        origin=Origin(xyz=(0.211, 0.0, 0.001)),
        material=bridge_paint,
        name="toe_beam",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        bridge_leaf.visual(
            Box((0.204, 0.012, 0.022)),
            origin=Origin(xyz=(0.108, sign * 0.056, -0.006)),
            material=bridge_paint,
            name=f"{side}_girder",
        )
        bridge_leaf.visual(
            Box((0.194, 0.008, 0.008)),
            origin=Origin(xyz=(0.112, sign * 0.044, 0.012)),
            material=accent,
            name=f"{side}_curb",
        )
        bridge_leaf.visual(
            Box((0.032, 0.024, 0.024)),
            origin=Origin(xyz=(0.006, sign * 0.073, -0.004)),
            material=bridge_paint,
            name=f"{side}_trunnion_arm",
        )
        bridge_leaf.visual(
            Cylinder(radius=0.016, length=0.016),
            origin=Origin(
                xyz=(0.0, sign * 0.089, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=bearing_steel,
            name=f"{side}_trunnion",
        )
    for index, x_pos in enumerate((0.050, 0.102, 0.154)):
        bridge_leaf.visual(
            Box((0.010, 0.106, 0.008)),
            origin=Origin(xyz=(x_pos, 0.0, 0.011)),
            material=accent,
            name=f"tread_bar_{index}",
        )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((0.220, 0.180, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=1.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_hinge = object_model.get_articulation("leaf_hinge")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    with ctx.pose({leaf_hinge: 0.0}):
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="deck_plate",
            elem_b="mid_rest_pad",
            name="leaf_is_seated_on_mid_pad",
        )
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="deck_plate",
            elem_b="forward_rest_pad",
            name="leaf_is_seated_on_forward_pad",
        )
        ctx.expect_gap(
            support_frame,
            bridge_leaf,
            axis="y",
            positive_elem="left_bearing_housing",
            negative_elem="left_trunnion",
            min_gap=0.0005,
            max_gap=0.0025,
            name="left_trunnion_has_running_clearance",
        )
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="y",
            positive_elem="right_trunnion",
            negative_elem="right_bearing_housing",
            min_gap=0.0005,
            max_gap=0.0025,
            name="right_trunnion_has_running_clearance",
        )

    closed_toe = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")
    with ctx.pose({leaf_hinge: 1.20}):
        open_toe = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")

    toe_rises = (
        closed_toe is not None
        and open_toe is not None
        and open_toe[0][2] > closed_toe[1][2] + 0.12
    )
    ctx.check(
        "leaf_opens_upward_into_stow_pose",
        toe_rises,
        details=(
            f"closed_toe={closed_toe}, open_toe={open_toe}; "
            "expected the free end to rise decisively when opening."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
