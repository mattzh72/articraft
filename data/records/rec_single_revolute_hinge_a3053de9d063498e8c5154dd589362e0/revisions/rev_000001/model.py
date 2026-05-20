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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LEAF_HEIGHT = 0.180
LEAF_WIDTH = 0.052
PLATE_THICKNESS = 0.006

KNUCKLE_OUTER_RADIUS = 0.009

BARREL_MARGIN = 0.010
KNUCKLE_GAP = 0.0
KNUCKLE_COUNT = 5
LEAF_Y_OFFSET = KNUCKLE_OUTER_RADIUS - (PLATE_THICKNESS / 2.0)
PLATE_KNUCKLE_OVERLAP = 0.0008
LEAF_X_CENTER = -(KNUCKLE_OUTER_RADIUS + (LEAF_WIDTH / 2.0)) + PLATE_KNUCKLE_OVERLAP

HINGE_LOWER = 0.0
HINGE_UPPER = math.pi


def _knuckle_length() -> float:
    working_length = LEAF_HEIGHT - (2.0 * BARREL_MARGIN)
    total_gaps = (KNUCKLE_COUNT - 1) * KNUCKLE_GAP
    return (working_length - total_gaps) / KNUCKLE_COUNT


def _knuckle_centers() -> tuple[float, ...]:
    length = _knuckle_length()
    start = -(LEAF_HEIGHT / 2.0) + BARREL_MARGIN
    centers: list[float] = []
    for index in range(KNUCKLE_COUNT):
        centers.append(start + (length / 2.0) + index * (length + KNUCKLE_GAP))
    return tuple(centers)


KNUCKLE_LENGTH = _knuckle_length()
KNUCKLE_CENTERS = _knuckle_centers()


def _add_leaf_visuals(model: ArticulatedObject, part, *, hand: str, material) -> None:
    sign_y = 1.0 if hand == "left" else -1.0
    knuckle_indexes = (0, 2, 4) if hand == "left" else (1, 3)

    part.visual(
        Box((LEAF_WIDTH, PLATE_THICKNESS, LEAF_HEIGHT)),
        origin=Origin(
            xyz=(LEAF_X_CENTER, sign_y * LEAF_Y_OFFSET, 0.0),
        ),
        material=material,
        name=f"{hand}_plate",
    )

    for index in knuckle_indexes:
        part.visual(
            Cylinder(radius=KNUCKLE_OUTER_RADIUS, length=KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, KNUCKLE_CENTERS[index])),
            material=material,
            name=f"{hand}_knuckle_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_leaf_hinge")

    steel = model.material("hinge_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    darker_steel = model.material("hinge_steel_dark", rgba=(0.35, 0.37, 0.40, 1.0))

    left_leaf = model.part("left_leaf")
    _add_leaf_visuals(model, left_leaf, hand="left", material=darker_steel)

    right_leaf = model.part("right_leaf")
    _add_leaf_visuals(model, right_leaf, hand="right", material=steel)

    model.articulation(
        "hinge_rotation",
        ArticulationType.REVOLUTE,
        parent=left_leaf,
        child=right_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=HINGE_LOWER,
            upper=HINGE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    hinge = object_model.get_articulation("hinge_rotation")
    left_plate = left_leaf.get_visual("left_plate")
    right_plate = right_leaf.get_visual("right_plate")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="hinge_pose_sweep_no_overlaps")

    limits = hinge.motion_limits
    ctx.check(
        "hinge_axis_is_barrel_axis",
        tuple(round(value, 6) for value in hinge.axis) == (0.0, 0.0, 1.0),
        f"expected revolute axis (0, 0, 1), got {hinge.axis}",
    )
    ctx.check(
        "hinge_is_single_revolute_leaf_pair",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        f"expected REVOLUTE articulation, got {hinge.articulation_type}",
    )
    ctx.check(
        "hinge_motion_range_is_realistic",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 3.0 <= limits.upper <= 3.2,
        f"unexpected motion limits {limits}",
    )

    left_aabb = ctx.part_world_aabb(left_leaf)
    right_aabb = ctx.part_world_aabb(right_leaf)
    if left_aabb is not None and right_aabb is not None:
        left_size = tuple(max_v - min_v for min_v, max_v in zip(left_aabb[0], left_aabb[1]))
        right_size = tuple(max_v - min_v for min_v, max_v in zip(right_aabb[0], right_aabb[1]))
        ctx.check(
            "left_leaf_scale_reads_heavy_duty",
            0.17 <= left_size[2] <= 0.19 and 0.065 <= left_size[0] <= 0.072 and 0.017 <= left_size[1] <= 0.020,
            f"left leaf extents {left_size} are out of expected heavy-duty hinge range",
        )
        ctx.check(
            "right_leaf_scale_matches_left",
            abs(right_size[2] - left_size[2]) <= 0.006
            and abs(right_size[0] - left_size[0]) <= 0.002
            and abs(right_size[1] - left_size[1]) <= 0.002,
            f"right leaf extents {right_size} do not match left leaf {left_size}",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(left_leaf, right_leaf, name="leafs_contact_through_barrel_at_rest")
        ctx.expect_gap(
            left_leaf,
            right_leaf,
            axis="y",
            positive_elem=left_plate,
            negative_elem=right_plate,
            min_gap=0.005,
            max_gap=0.0075,
            name="closed_leaf_stack_has_thickness_clearance_at_rest",
        )
        ctx.expect_overlap(
            left_leaf,
            right_leaf,
            axes="xz",
            min_overlap=0.05,
            name="closed_leaf_stack_remains_registered_at_rest",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="hinge_lower_no_floating")
            ctx.expect_contact(left_leaf, right_leaf, name="leafs_contact_through_barrel_at_lower_limit")
            ctx.expect_gap(
                left_leaf,
                right_leaf,
                axis="y",
                positive_elem=left_plate,
                negative_elem=right_plate,
                min_gap=0.005,
                max_gap=0.0075,
                name="lower_limit_closed_stack_clearance",
            )
            ctx.expect_overlap(left_leaf, right_leaf, axes="xz", min_overlap=0.05, name="lower_limit_closed_stack_registration")
        with ctx.pose({hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="hinge_upper_no_floating")
            ctx.expect_contact(left_leaf, right_leaf, name="leafs_contact_through_barrel_at_upper_limit")
            ctx.expect_gap(
                right_leaf,
                left_leaf,
                axis="x",
                positive_elem=right_plate,
                negative_elem=left_plate,
                min_gap=0.016,
                max_gap=0.0175,
                name="upper_limit_open_flat_plate_gap_matches_barrel_diameter",
            )
            ctx.expect_overlap(
                left_leaf,
                right_leaf,
                axes="z",
                min_overlap=0.16,
                name="upper_limit_open_flat_leaves_share_barrel_height",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
