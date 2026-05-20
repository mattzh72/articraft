from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


POST_SIZE = 0.12
POST_HEIGHT_ABOVE_GRADE = 1.55
POST_BURIED_DEPTH = 0.45
POST_CENTER_X = 1.06
POST_DEPTH = 0.12
FOOTING_DEPTH = 0.18
FOOTING_TOP_Z = -0.27
GATE_BOTTOM_CLEARANCE = 0.08
LEAF_WIDTH = 0.965
LEAF_HEIGHT = 1.20
FRAME_WIDTH = 0.06
FRAME_DEPTH = 0.035
RAIL_HEIGHT = 0.04
HINGE_BARREL_RADIUS = 0.018
HINGE_AXIS_Y = POST_DEPTH * 0.5 + HINGE_BARREL_RADIUS
HINGE_TO_FRAME_OFFSET = 0.03
LOWER_HINGE_Z = 0.20
UPPER_HINGE_Z = 1.00
OPEN_LIMIT = radians(120.0)


def _add_leaf_geometry(part, *, span_sign: float, frame_material, hinge_material) -> None:
    inner_span = LEAF_WIDTH - 2.0 * FRAME_WIDTH
    leaf_mid_x = span_sign * (HINGE_TO_FRAME_OFFSET + LEAF_WIDTH * 0.5)
    hinge_stile_x = span_sign * (HINGE_TO_FRAME_OFFSET + FRAME_WIDTH * 0.5)
    latch_stile_x = span_sign * (HINGE_TO_FRAME_OFFSET + LEAF_WIDTH - FRAME_WIDTH * 0.5)
    hinge_bracket_x = span_sign * (HINGE_TO_FRAME_OFFSET * 0.5)

    part.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, LEAF_HEIGHT)),
        origin=Origin(xyz=(hinge_stile_x, 0.0, LEAF_HEIGHT * 0.5)),
        material=frame_material,
        name="hinge_stile",
    )
    part.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, LEAF_HEIGHT)),
        origin=Origin(xyz=(latch_stile_x, 0.0, LEAF_HEIGHT * 0.5)),
        material=frame_material,
        name="latch_stile",
    )
    part.visual(
        Box((inner_span, FRAME_DEPTH, FRAME_WIDTH)),
        origin=Origin(xyz=(leaf_mid_x, 0.0, FRAME_WIDTH * 0.5)),
        material=frame_material,
        name="bottom_rail_frame",
    )
    part.visual(
        Box((inner_span, FRAME_DEPTH, FRAME_WIDTH)),
        origin=Origin(xyz=(leaf_mid_x, 0.0, LEAF_HEIGHT - FRAME_WIDTH * 0.5)),
        material=frame_material,
        name="top_rail_frame",
    )

    for rail_name, rail_z in (
        ("lower_rail", 0.33),
        ("middle_rail", 0.62),
        ("upper_rail", 0.91),
    ):
        part.visual(
            Box((inner_span, FRAME_DEPTH * 0.85, RAIL_HEIGHT)),
            origin=Origin(xyz=(leaf_mid_x, 0.0, rail_z)),
            material=frame_material,
            name=rail_name,
        )

    for bracket_name, bracket_z in (("lower_hinge_bracket", LOWER_HINGE_Z), ("upper_hinge_bracket", UPPER_HINGE_Z)):
        part.visual(
            Box((HINGE_TO_FRAME_OFFSET + 0.006, 0.026, 0.11)),
            origin=Origin(xyz=(hinge_bracket_x, 0.0, bracket_z)),
            material=hinge_material,
            name=bracket_name,
        )
    part.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HINGE_Z)),
        material=hinge_material,
        name="lower_hinge_barrel",
    )
    part.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, UPPER_HINGE_Z)),
        material=hinge_material,
        name="upper_hinge_barrel",
    )

    latch_plate_x = span_sign * (HINGE_TO_FRAME_OFFSET + LEAF_WIDTH - FRAME_WIDTH - 0.012)
    part.visual(
        Box((0.012, FRAME_DEPTH * 0.9, 0.18)),
        origin=Origin(xyz=(latch_plate_x, 0.0, LEAF_HEIGHT * 0.5)),
        material=hinge_material,
        name="latch_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_driveway_garden_gate", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.28, 0.19, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.36, 0.38, 0.37, 1.0))
    concrete = model.material("concrete", rgba=(0.46, 0.46, 0.45, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((2.36, 0.16, POST_HEIGHT_ABOVE_GRADE + POST_BURIED_DEPTH)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, (POST_HEIGHT_ABOVE_GRADE - POST_BURIED_DEPTH) * 0.5)),
    )
    support_frame.visual(
        Box((POST_SIZE, POST_DEPTH, POST_HEIGHT_ABOVE_GRADE + POST_BURIED_DEPTH)),
        origin=Origin(
            xyz=(
                -POST_CENTER_X,
                0.0,
                (POST_HEIGHT_ABOVE_GRADE - POST_BURIED_DEPTH) * 0.5,
            )
        ),
        material=painted_steel,
        name="left_post",
    )
    support_frame.visual(
        Box((POST_SIZE, POST_DEPTH, POST_HEIGHT_ABOVE_GRADE + POST_BURIED_DEPTH)),
        origin=Origin(
            xyz=(
                POST_CENTER_X,
                0.0,
                (POST_HEIGHT_ABOVE_GRADE - POST_BURIED_DEPTH) * 0.5,
            )
        ),
        material=painted_steel,
        name="right_post",
    )
    support_frame.visual(
        Box((2.24, 0.16, FOOTING_DEPTH)),
        origin=Origin(xyz=(0.0, 0.0, FOOTING_TOP_Z - FOOTING_DEPTH * 0.5)),
        material=concrete,
        name="footing_beam",
    )
    for cap_name, cap_x in (("left_post_cap", -POST_CENTER_X), ("right_post_cap", POST_CENTER_X)):
        support_frame.visual(
            Box((POST_SIZE * 1.05, POST_DEPTH * 1.05, 0.02)),
            origin=Origin(xyz=(cap_x, 0.0, POST_HEIGHT_ABOVE_GRADE + 0.01)),
            material=hinge_steel,
            name=cap_name,
        )

    for hinge_name, hinge_x, hinge_z in (
        ("left_post_lower_hinge", -1.015, GATE_BOTTOM_CLEARANCE + LOWER_HINGE_Z),
        ("left_post_upper_hinge", -1.015, GATE_BOTTOM_CLEARANCE + UPPER_HINGE_Z),
        ("right_post_lower_hinge", 1.015, GATE_BOTTOM_CLEARANCE + LOWER_HINGE_Z),
        ("right_post_upper_hinge", 1.015, GATE_BOTTOM_CLEARANCE + UPPER_HINGE_Z),
    ):
        support_frame.visual(
            Box((0.03, 0.016, 0.11)),
            origin=Origin(xyz=(hinge_x, POST_DEPTH * 0.5 - 0.008, hinge_z)),
            material=hinge_steel,
            name=hinge_name,
        )

    left_leaf = model.part("left_leaf")
    left_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH, 0.06, LEAF_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(HINGE_TO_FRAME_OFFSET + LEAF_WIDTH * 0.5, 0.0, LEAF_HEIGHT * 0.5)),
    )
    _add_leaf_geometry(
        left_leaf,
        span_sign=1.0,
        frame_material=painted_steel,
        hinge_material=hinge_steel,
    )

    right_leaf = model.part("right_leaf")
    right_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH, 0.06, LEAF_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(-(HINGE_TO_FRAME_OFFSET + LEAF_WIDTH * 0.5), 0.0, LEAF_HEIGHT * 0.5)),
    )
    _add_leaf_geometry(
        right_leaf,
        span_sign=-1.0,
        frame_material=painted_steel,
        hinge_material=hinge_steel,
    )

    model.articulation(
        "left_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=left_leaf,
        origin=Origin(xyz=(-1.0, HINGE_AXIS_Y, GATE_BOTTOM_CLEARANCE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=OPEN_LIMIT,
        ),
    )
    model.articulation(
        "right_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=right_leaf,
        origin=Origin(xyz=(1.0, HINGE_AXIS_Y, GATE_BOTTOM_CLEARANCE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-OPEN_LIMIT,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    left_hinge = object_model.get_articulation("left_leaf_hinge")
    right_hinge = object_model.get_articulation("right_leaf_hinge")

    left_post = support_frame.get_visual("left_post")
    right_post = support_frame.get_visual("right_post")
    left_post_lower_hinge = support_frame.get_visual("left_post_lower_hinge")
    left_post_upper_hinge = support_frame.get_visual("left_post_upper_hinge")
    right_post_lower_hinge = support_frame.get_visual("right_post_lower_hinge")
    right_post_upper_hinge = support_frame.get_visual("right_post_upper_hinge")
    left_hinge_stile = left_leaf.get_visual("hinge_stile")
    right_hinge_stile = right_leaf.get_visual("hinge_stile")
    left_latch_stile = left_leaf.get_visual("latch_stile")
    right_latch_stile = right_leaf.get_visual("latch_stile")
    left_lower_hinge_barrel = left_leaf.get_visual("lower_hinge_barrel")
    left_upper_hinge_barrel = left_leaf.get_visual("upper_hinge_barrel")
    right_lower_hinge_barrel = right_leaf.get_visual("lower_hinge_barrel")
    right_upper_hinge_barrel = right_leaf.get_visual("upper_hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20, name="articulation_clearance_sweep")

    ctx.expect_gap(
        left_leaf,
        support_frame,
        axis="x",
        min_gap=0.025,
        max_gap=0.035,
        positive_elem=left_hinge_stile,
        negative_elem=left_post,
        name="left_leaf_hinge_stile_sits_close_to_left_post",
    )
    ctx.expect_gap(
        support_frame,
        right_leaf,
        axis="x",
        min_gap=0.025,
        max_gap=0.035,
        positive_elem=right_post,
        negative_elem=right_hinge_stile,
        name="right_leaf_hinge_stile_sits_close_to_right_post",
    )
    ctx.expect_contact(
        left_leaf,
        support_frame,
        elem_a=left_lower_hinge_barrel,
        elem_b=left_post_lower_hinge,
        name="left_lower_hinge_contacts_left_post_hinge",
    )
    ctx.expect_contact(
        left_leaf,
        support_frame,
        elem_a=left_upper_hinge_barrel,
        elem_b=left_post_upper_hinge,
        name="left_upper_hinge_contacts_left_post_hinge",
    )
    ctx.expect_contact(
        right_leaf,
        support_frame,
        elem_a=right_lower_hinge_barrel,
        elem_b=right_post_lower_hinge,
        name="right_lower_hinge_contacts_right_post_hinge",
    )
    ctx.expect_contact(
        right_leaf,
        support_frame,
        elem_a=right_upper_hinge_barrel,
        elem_b=right_post_upper_hinge,
        name="right_upper_hinge_contacts_right_post_hinge",
    )
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        min_gap=0.008,
        max_gap=0.012,
        positive_elem=right_latch_stile,
        negative_elem=left_latch_stile,
        name="closed_leaves_meet_with_small_center_gap",
    )

    left_leaf_bounds = ctx.part_world_aabb(left_leaf)
    right_leaf_bounds = ctx.part_world_aabb(right_leaf)
    support_bounds = ctx.part_world_aabb(support_frame)
    if left_leaf_bounds is not None and right_leaf_bounds is not None and support_bounds is not None:
        left_width = left_leaf_bounds[1][0] - left_leaf_bounds[0][0]
        left_height = left_leaf_bounds[1][2] - left_leaf_bounds[0][2]
        support_width = support_bounds[1][0] - support_bounds[0][0]
        ctx.check(
            "left_leaf_has_driveway_gate_scale",
            0.95 <= left_width <= 1.05 and 1.15 <= left_height <= 1.25,
            details=f"left leaf bounds={left_leaf_bounds}",
        )
        ctx.check(
            "support_span_reads_as_double_gate_opening",
            2.20 <= support_width <= 2.30,
            details=f"support bounds={support_bounds}",
        )

    with ctx.pose({left_hinge: 1.6, right_hinge: -1.6}):
        ctx.expect_contact(
            left_leaf,
            support_frame,
            elem_a=left_lower_hinge_barrel,
            elem_b=left_post_lower_hinge,
            name="left_lower_hinge_stays_connected_when_open",
        )
        ctx.expect_contact(
            right_leaf,
            support_frame,
            elem_a=right_lower_hinge_barrel,
            elem_b=right_post_lower_hinge,
            name="right_lower_hinge_stays_connected_when_open",
        )
        ctx.expect_gap(
            left_leaf,
            support_frame,
            axis="y",
            min_gap=0.70,
            positive_elem=left_latch_stile,
            negative_elem=left_post,
            name="left_leaf_swings_outward",
        )
        ctx.expect_gap(
            right_leaf,
            support_frame,
            axis="y",
            min_gap=0.70,
            positive_elem=right_latch_stile,
            negative_elem=right_post,
            name="right_leaf_swings_outward",
        )
        ctx.expect_gap(
            right_leaf,
            left_leaf,
            axis="x",
            min_gap=1.90,
            positive_elem=right_latch_stile,
            negative_elem=left_latch_stile,
            name="open_leaves_clear_the_center_span",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
