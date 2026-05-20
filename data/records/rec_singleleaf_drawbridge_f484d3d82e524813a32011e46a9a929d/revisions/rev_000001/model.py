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
    model = ArticulatedObject(name="singleleaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.62, 0.63, 0.64, 1.0))
    structural_steel = model.material("structural_steel", rgba=(0.34, 0.38, 0.42, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.48, 0.50, 0.46, 1.0))
    roadway = model.material("roadway", rgba=(0.14, 0.14, 0.15, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.58, 0.60, 0.62, 1.0))

    frame = model.part("support_frame")
    frame.visual(
        Box((4.2, 7.6, 0.45)),
        origin=Origin(xyz=(-1.0, 0.0, 0.225)),
        material=concrete,
        name="base_slab",
    )
    frame.visual(
        Box((0.45, 7.0, 2.2)),
        origin=Origin(xyz=(-2.0, 0.0, 1.55)),
        material=concrete,
        name="rear_wall",
    )
    frame.visual(
        Box((1.2, 0.36, 1.46)),
        origin=Origin(xyz=(0.0, 3.33, 1.18)),
        material=structural_steel,
        name="left_bearing_pedestal",
    )
    frame.visual(
        Box((1.2, 0.36, 1.46)),
        origin=Origin(xyz=(0.0, -3.33, 1.18)),
        material=structural_steel,
        name="right_bearing_pedestal",
    )
    frame.visual(
        Box((0.20, 0.36, 0.95)),
        origin=Origin(xyz=(0.50, 3.33, 2.385)),
        material=painted_steel,
        name="left_front_cheek",
    )
    frame.visual(
        Box((0.20, 0.36, 0.95)),
        origin=Origin(xyz=(0.50, -3.33, 2.385)),
        material=painted_steel,
        name="right_front_cheek",
    )
    frame.visual(
        Box((0.20, 0.36, 0.95)),
        origin=Origin(xyz=(-0.50, 3.33, 2.385)),
        material=painted_steel,
        name="left_rear_cheek",
    )
    frame.visual(
        Box((0.20, 0.36, 0.95)),
        origin=Origin(xyz=(-0.50, -3.33, 2.385)),
        material=painted_steel,
        name="right_rear_cheek",
    )
    frame.visual(
        Box((1.2, 0.36, 0.16)),
        origin=Origin(xyz=(0.0, 3.33, 2.78)),
        material=painted_steel,
        name="left_top_cap",
    )
    frame.visual(
        Box((1.2, 0.36, 0.16)),
        origin=Origin(xyz=(0.0, -3.33, 2.78)),
        material=painted_steel,
        name="right_top_cap",
    )
    frame.visual(
        Box((0.22, 6.30, 0.18)),
        origin=Origin(xyz=(-0.50, 0.0, 2.77)),
        material=painted_steel,
        name="rear_tie_beam",
    )
    frame.visual(
        Box((0.60, 0.56, 1.20)),
        origin=Origin(xyz=(-1.55, 2.20, 1.05)),
        material=structural_steel,
        name="left_rest_pad",
    )
    frame.visual(
        Box((0.60, 0.56, 1.20)),
        origin=Origin(xyz=(-1.55, -2.20, 1.05)),
        material=structural_steel,
        name="right_rest_pad",
    )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Box((11.8, 5.8, 0.08)),
        origin=Origin(xyz=(5.40, 0.0, 0.24)),
        material=painted_steel,
        name="deck_plate",
    )
    leaf.visual(
        Box((11.6, 5.4, 0.03)),
        origin=Origin(xyz=(5.45, 0.0, 0.295)),
        material=roadway,
        name="wearing_surface",
    )
    leaf.visual(
        Box((11.6, 4.9, 0.08)),
        origin=Origin(xyz=(5.50, 0.0, -0.46)),
        material=painted_steel,
        name="underside_plate",
    )
    leaf.visual(
        Box((12.4, 0.42, 0.70)),
        origin=Origin(xyz=(5.10, 2.69, -0.11)),
        material=painted_steel,
        name="left_side_girder",
    )
    leaf.visual(
        Box((12.4, 0.42, 0.70)),
        origin=Origin(xyz=(5.10, -2.69, -0.11)),
        material=painted_steel,
        name="right_side_girder",
    )
    leaf.visual(
        Box((1.50, 5.20, 0.80)),
        origin=Origin(xyz=(-0.80, 0.0, -0.20)),
        material=painted_steel,
        name="counterweight_box",
    )
    leaf.visual(
        Box((1.20, 5.60, 0.86)),
        origin=Origin(xyz=(-0.10, 0.0, -0.19)),
        material=painted_steel,
        name="pivot_bulkhead",
    )
    leaf.visual(
        Box((1.00, 0.28, 0.54)),
        origin=Origin(xyz=(-0.05, 2.96, -0.03)),
        material=painted_steel,
        name="left_trunnion_arm",
    )
    leaf.visual(
        Box((1.00, 0.28, 0.54)),
        origin=Origin(xyz=(-0.05, -2.96, -0.03)),
        material=painted_steel,
        name="right_trunnion_arm",
    )
    leaf.visual(
        Cylinder(radius=0.34, length=0.50),
        origin=Origin(xyz=(0.0, 3.30, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="left_trunnion",
    )
    leaf.visual(
        Cylinder(radius=0.34, length=0.50),
        origin=Origin(xyz=(0.0, -3.30, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="right_trunnion",
    )
    leaf.visual(
        Box((0.35, 5.82, 0.70)),
        origin=Origin(xyz=(11.15, 0.0, -0.11)),
        material=painted_steel,
        name="toe_beam",
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0e6, velocity=0.35, lower=0.0, upper=1.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("frame_to_leaf")
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

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            frame,
            frame,
            elem_a="rear_tie_beam",
            elem_b="left_top_cap",
            name="rear tie beam keys into left top cap",
        )
        ctx.expect_contact(
            frame,
            frame,
            elem_a="rear_tie_beam",
            elem_b="right_top_cap",
            name="rear tie beam keys into right top cap",
        )
        ctx.expect_contact(
            frame,
            leaf,
            elem_a="left_bearing_pedestal",
            elem_b="left_trunnion",
            name="left trunnion seated on pedestal",
        )
        ctx.expect_contact(
            frame,
            leaf,
            elem_a="right_bearing_pedestal",
            elem_b="right_trunnion",
            name="right trunnion seated on pedestal",
        )
        ctx.expect_contact(
            frame,
            leaf,
            elem_a="left_rest_pad",
            elem_b="counterweight_box",
            name="left rest pad supports counterweight box",
        )
        ctx.expect_contact(
            frame,
            leaf,
            elem_a="right_rest_pad",
            elem_b="counterweight_box",
            name="right rest pad supports counterweight box",
        )

    closed_tip_aabb = None
    open_tip_aabb = None
    with ctx.pose({hinge: 0.0}):
        closed_tip_aabb = ctx.part_element_world_aabb(leaf, elem="toe_beam")
    with ctx.pose({hinge: 1.05}):
        open_tip_aabb = ctx.part_element_world_aabb(leaf, elem="toe_beam")
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps when leaf is raised")

    tip_ok = closed_tip_aabb is not None and open_tip_aabb is not None
    if tip_ok:
        closed_tip_top = closed_tip_aabb[1][2]
        open_tip_top = open_tip_aabb[1][2]
        ctx.check(
            "leaf opens upward",
            open_tip_top > closed_tip_top + 4.0,
            details=(
                f"expected raised toe beam to climb more than 4.0 m; "
                f"closed_top={closed_tip_top:.3f}, open_top={open_tip_top:.3f}"
            ),
        )
    else:
        ctx.fail("leaf opens upward", "toe_beam AABB was unavailable in one or both poses")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
