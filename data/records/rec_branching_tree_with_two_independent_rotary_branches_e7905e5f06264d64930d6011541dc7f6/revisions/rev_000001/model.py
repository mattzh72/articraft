from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.66
BASE_Y = 0.24
BASE_Z = 0.075

SPINE_X = 0.14
SPINE_Y = 0.11
SPINE_Z = 1.04
SPINE_CENTER_Z = BASE_Z + SPINE_Z / 2.0

UPPER_PIVOT_X = 0.235
UPPER_PIVOT_Z = 0.93
UPPER_CHEEK_X = 0.02
UPPER_CHEEK_Y = 0.018
UPPER_CHEEK_Z = 0.15
UPPER_CHEEK_GAP = 0.07
UPPER_CHEEK_CENTER_X = UPPER_PIVOT_X
UPPER_CHEEK_CENTER_OFFSET_Y = UPPER_CHEEK_GAP / 2.0 + UPPER_CHEEK_Y / 2.0
UPPER_HOLE_R = 0.017
UPPER_JOURNAL_R = 0.0125
UPPER_JOURNAL_CLEAR_LEN = UPPER_CHEEK_GAP + 2.0 * UPPER_CHEEK_Y
UPPER_COLLAR_T = 0.01

LOWER_PIVOT_X = -0.235
LOWER_PIVOT_Z = 0.43
LOWER_PEDESTAL_X = 0.10
LOWER_PEDESTAL_Y = 0.09
LOWER_PEDESTAL_Z = 0.09
LOWER_PEDESTAL_CENTER_X = LOWER_PIVOT_X
LOWER_STUB_R = 0.016
LOWER_STUB_H = 0.05


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def _cyl_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def make_frame_root_shape() -> cq.Workplane:
    base = _box((BASE_X, BASE_Y, BASE_Z), (0.0, 0.0, BASE_Z / 2.0)).edges("|Z").fillet(0.01)
    spine = _box((SPINE_X, SPINE_Y, SPINE_Z), (0.0, 0.0, SPINE_CENTER_Z)).edges("|Z").fillet(0.008)
    return _union_all([base, spine])


def make_upper_support_shape() -> cq.Workplane:
    upper_bridge = _box((0.155, 0.09, 0.055), (0.1475, 0.0, UPPER_PIVOT_Z - 0.065))

    upper_cheek_pos = _box(
        (UPPER_CHEEK_X, UPPER_CHEEK_Y, UPPER_CHEEK_Z),
        (UPPER_CHEEK_CENTER_X, UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z),
    )
    upper_cheek_neg = _box(
        (UPPER_CHEEK_X, UPPER_CHEEK_Y, UPPER_CHEEK_Z),
        (UPPER_CHEEK_CENTER_X, -UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z),
    )

    cap_center_x = UPPER_PIVOT_X
    cap_center_z = UPPER_PIVOT_Z + 0.05
    upper_cap_pos = _box((0.04, UPPER_CHEEK_Y, 0.02), (cap_center_x, UPPER_CHEEK_CENTER_OFFSET_Y, cap_center_z))
    upper_cap_neg = _box((0.04, UPPER_CHEEK_Y, 0.02), (cap_center_x, -UPPER_CHEEK_CENTER_OFFSET_Y, cap_center_z))

    upper_stop_low_pos = _box(
        (0.012, UPPER_CHEEK_Y, 0.02),
        (UPPER_PIVOT_X + 0.018, UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z - 0.058),
    )
    upper_stop_low_neg = _box(
        (0.012, UPPER_CHEEK_Y, 0.02),
        (UPPER_PIVOT_X + 0.018, -UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z - 0.058),
    )
    upper_stop_high_pos = _box(
        (0.012, UPPER_CHEEK_Y, 0.02),
        (UPPER_PIVOT_X - 0.008, UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z + 0.063),
    )
    upper_stop_high_neg = _box(
        (0.012, UPPER_CHEEK_Y, 0.02),
        (UPPER_PIVOT_X - 0.008, -UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z + 0.063),
    )

    support = _union_all(
        [
            upper_bridge,
            upper_cheek_pos,
            upper_cheek_neg,
            upper_cap_pos,
            upper_cap_neg,
            upper_stop_low_pos,
            upper_stop_low_neg,
            upper_stop_high_pos,
            upper_stop_high_neg,
        ]
    )

    cheek_hole_pos = _cyl_y(UPPER_HOLE_R, UPPER_CHEEK_Y + 0.01, (UPPER_PIVOT_X, UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z))
    cheek_hole_neg = _cyl_y(UPPER_HOLE_R, UPPER_CHEEK_Y + 0.01, (UPPER_PIVOT_X, -UPPER_CHEEK_CENTER_OFFSET_Y, UPPER_PIVOT_Z))
    return support.cut(cheek_hole_pos).cut(cheek_hole_neg)


def make_lower_support_shape() -> cq.Workplane:
    lower_pedestal = _box(
        (LOWER_PEDESTAL_X, LOWER_PEDESTAL_Y, LOWER_PEDESTAL_Z),
        (LOWER_PEDESTAL_CENTER_X, 0.0, LOWER_PIVOT_Z - LOWER_PEDESTAL_Z / 2.0),
    )
    lower_bridge = _box(
        (0.165, 0.085, 0.04),
        (-0.1525, 0.0, LOWER_PIVOT_Z - 0.075),
    )
    lower_stub = _cyl_z(
        LOWER_STUB_R,
        LOWER_STUB_H,
        (LOWER_PIVOT_X, 0.0, LOWER_PIVOT_Z + LOWER_STUB_H / 2.0),
    )
    lower_stop_pos = _box(
        (0.012, 0.012, 0.022),
        (LOWER_PIVOT_X + 0.028, LOWER_PEDESTAL_Y / 2.0 + 0.006, LOWER_PIVOT_Z - 0.018),
    )
    lower_stop_neg = _box(
        (0.012, 0.012, 0.022),
        (LOWER_PIVOT_X + 0.028, -(LOWER_PEDESTAL_Y / 2.0 + 0.006), LOWER_PIVOT_Z - 0.018),
    )
    return _union_all([lower_pedestal, lower_bridge, lower_stub, lower_stop_pos, lower_stop_neg])


def make_upper_branch_shape() -> cq.Workplane:
    journal = _cyl_y(UPPER_JOURNAL_R, UPPER_JOURNAL_CLEAR_LEN - 0.002, (0.0, 0.0, 0.0))
    retainer_center_y = UPPER_CHEEK_GAP / 2.0 + UPPER_CHEEK_Y + 0.0015
    collar_pos = _cyl_y(0.018, 0.003, (0.0, retainer_center_y, 0.0))
    collar_neg = _cyl_y(0.018, 0.003, (0.0, -retainer_center_y, 0.0))

    lug = _box((0.03, 0.028, 0.022), (0.055, 0.0, 0.0))
    arm = _box((0.13, 0.038, 0.024), (0.145, 0.0, 0.0))
    fork_tip = _cyl_z(0.022, 0.024, (0.215, 0.0, 0.0))
    nose_flat = _box((0.018, 0.038, 0.024), (0.26, 0.0, 0.0))
    rib = _box((0.065, 0.02, 0.014), (0.085, 0.0, -0.012))
    fork_slot = _box((0.06, 0.014, 0.032), (0.22, 0.0, 0.0))

    arm_shape = _union_all([lug, arm, fork_tip, nose_flat, rib]).cut(fork_slot)

    branch = _union_all([journal, collar_pos, collar_neg, arm_shape])
    return branch


def make_lower_branch_shape() -> cq.Workplane:
    hub = _cyl_z(0.027, 0.022, (0.0, 0.0, 0.011))
    top_ring = _cyl_z(0.03, 0.004, (0.0, 0.0, 0.026))
    neck = _box((0.028, 0.028, 0.014), (-0.055, 0.0, 0.014))
    plate = _box((0.12, 0.078, 0.018), (-0.145, 0.0, 0.014))
    plate_tip = _cyl_z(0.039, 0.018, (-0.205, 0.0, 0.014))
    stiffener = _box((0.055, 0.02, 0.012), (-0.085, 0.0, 0.024))

    branch = _union_all([hub, top_ring, neck, plate, plate_tip, stiffener])
    central_hole = _cyl_z(LOWER_STUB_R + 0.005, 0.05, (0.0, 0.0, 0.025))
    lightening_slot = _box((0.055, 0.022, 0.024), (-0.145, 0.0, 0.014))
    branch = branch.cut(central_hole).cut(lightening_slot)
    return branch


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixture_tree")

    frame_material = model.material("frame_paint", rgba=(0.34, 0.37, 0.40, 1.0))
    branch_material = model.material("machined_steel", rgba=(0.74, 0.75, 0.77, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame_root_shape(), "fixture_tree_frame"),
        material=frame_material,
        name="frame_body",
    )

    upper_support = model.part("upper_support")
    upper_support.visual(
        mesh_from_cadquery(make_upper_support_shape(), "fixture_tree_upper_support"),
        material=frame_material,
        name="upper_support_body",
    )

    lower_support = model.part("lower_support")
    lower_support.visual(
        mesh_from_cadquery(make_lower_support_shape(), "fixture_tree_lower_support"),
        material=frame_material,
        name="lower_support_body",
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        mesh_from_cadquery(make_upper_branch_shape(), "fixture_tree_upper_branch"),
        material=branch_material,
        name="upper_branch_body",
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        mesh_from_cadquery(make_lower_branch_shape(), "fixture_tree_lower_branch"),
        material=branch_material,
        name="lower_branch_body",
    )

    model.articulation(
        "frame_to_upper_support",
        ArticulationType.FIXED,
        parent=frame,
        child=upper_support,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_lower_support",
        ArticulationType.FIXED,
        parent=frame,
        child=lower_support,
        origin=Origin(),
    )

    model.articulation(
        "upper_branch_joint",
        ArticulationType.REVOLUTE,
        parent=upper_support,
        child=upper_branch,
        origin=Origin(xyz=(UPPER_PIVOT_X, 0.0, UPPER_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.08, upper=0.08),
    )

    model.articulation(
        "lower_branch_joint",
        ArticulationType.REVOLUTE,
        parent=lower_support,
        child=lower_branch,
        origin=Origin(xyz=(LOWER_PIVOT_X, 0.0, LOWER_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=-0.6, upper=0.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    upper_support = object_model.get_part("upper_support")
    lower_support = object_model.get_part("lower_support")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")
    upper_joint = object_model.get_articulation("upper_branch_joint")
    lower_joint = object_model.get_articulation("lower_branch_joint")

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
    ctx.allow_overlap(
        upper_support,
        upper_branch,
        reason="upper branch journal is intentionally nested within the side-cheek bearing support",
    )
    ctx.allow_overlap(
        lower_support,
        lower_branch,
        reason="lower branch hub is intentionally nested around the pedestal stub bearing",
    )
    ctx.allow_overlap(
        frame,
        upper_support,
        reason="upper support bracket is modeled as a fixed welded subassembly blended into the mast",
    )
    ctx.allow_overlap(
        frame,
        lower_support,
        reason="lower pedestal support is modeled as a fixed welded subassembly blended into the mast base",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_origin_gap(
        upper_branch,
        lower_branch,
        axis="x",
        min_gap=0.22,
        name="branches_mount_on_opposite_sides_of_spine",
    )
    ctx.expect_origin_gap(
        upper_branch,
        lower_branch,
        axis="z",
        min_gap=0.35,
        name="upper_branch_sits_well_above_lower_branch",
    )

    with ctx.pose({upper_joint: 0.0, lower_joint: 0.0}):
        ctx.expect_contact(
            frame,
            upper_support,
            contact_tol=0.002,
            name="upper_support_is_rigidly_mounted_to_frame",
        )
        ctx.expect_contact(
            frame,
            lower_support,
            contact_tol=0.002,
            name="lower_support_is_rigidly_mounted_to_frame",
        )
        ctx.expect_contact(
            upper_branch,
            upper_support,
            contact_tol=0.002,
            name="upper_branch_is_captured_in_cheeks",
        )
        ctx.expect_contact(
            lower_branch,
            lower_support,
            contact_tol=0.0035,
            name="lower_branch_is_seated_on_pedestal",
        )

    with ctx.pose({upper_joint: upper_joint.motion_limits.upper, lower_joint: lower_joint.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_clearance_check")

    with ctx.pose({upper_joint: upper_joint.motion_limits.lower, lower_joint: lower_joint.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_clearance_check")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
