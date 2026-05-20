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


HINGE_LENGTH = 0.72
LEAF_WIDTH = 0.04
LEAF_THICKNESS = 0.003
BARREL_RADIUS = 0.006
PIN_STEM_RADIUS = 0.0042
PIN_HEAD_RADIUS = 0.0076
PIN_HEAD_THICKNESS = 0.003
KNUCKLE_COUNT = 7
KNUCKLE_GAP = 0.005
BARREL_LENGTH = HINGE_LENGTH - KNUCKLE_GAP * (KNUCKLE_COUNT - 1)
KNUCKLE_LENGTH = BARREL_LENGTH / KNUCKLE_COUNT
PIN_STEM_EXPOSED = 0.0045
OPEN_LIMIT_RAD = 1.95


def _knuckle_center_x(index: int) -> float:
    return -HINGE_LENGTH / 2.0 + index * (KNUCKLE_LENGTH + KNUCKLE_GAP) + KNUCKLE_LENGTH / 2.0


def _x_axis_origin(x_center: float) -> Origin:
    return Origin(
        xyz=(x_center, 0.0, 0.0),
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="access_door_hinge")

    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    frame_leaf = model.part("frame_leaf")
    frame_leaf.visual(
        Box((HINGE_LENGTH, LEAF_WIDTH, LEAF_THICKNESS)),
        origin=Origin(xyz=(0.0, -LEAF_WIDTH / 2.0, -LEAF_THICKNESS / 2.0)),
        material=satin_steel,
        name="frame_plate",
    )
    frame_leaf.visual(
        Cylinder(radius=PIN_STEM_RADIUS, length=PIN_STEM_EXPOSED),
        origin=_x_axis_origin(-HINGE_LENGTH / 2.0 - PIN_STEM_EXPOSED / 2.0),
        material=satin_steel,
        name="pin_stem_exposed",
    )
    frame_leaf.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=_x_axis_origin(
            -HINGE_LENGTH / 2.0 - PIN_STEM_EXPOSED - PIN_HEAD_THICKNESS / 2.0
        ),
        material=satin_steel,
        name="pin_head",
    )
    for index in range(KNUCKLE_COUNT):
        if index % 2 == 0:
            x_center = _knuckle_center_x(index)
            frame_leaf.visual(
                Cylinder(radius=BARREL_RADIUS, length=KNUCKLE_LENGTH),
                origin=_x_axis_origin(x_center),
                material=satin_steel,
                name=f"frame_knuckle_{index}",
            )
            frame_leaf.visual(
                Box((KNUCKLE_LENGTH, BARREL_RADIUS, LEAF_THICKNESS)),
                origin=Origin(
                    xyz=(x_center, -BARREL_RADIUS / 2.0, -LEAF_THICKNESS / 2.0)
                ),
                material=satin_steel,
                name=f"frame_web_{index}",
            )

    door_leaf = model.part("door_leaf")
    door_leaf.visual(
        Box((HINGE_LENGTH, LEAF_WIDTH, LEAF_THICKNESS)),
        origin=Origin(xyz=(0.0, LEAF_WIDTH / 2.0, -LEAF_THICKNESS / 2.0)),
        material=satin_steel,
        name="door_plate",
    )
    for index in range(KNUCKLE_COUNT):
        if index % 2 == 1:
            x_center = _knuckle_center_x(index)
            door_leaf.visual(
                Cylinder(radius=BARREL_RADIUS, length=KNUCKLE_LENGTH),
                origin=_x_axis_origin(x_center),
                material=satin_steel,
                name=f"door_knuckle_{index}",
            )
            door_leaf.visual(
                Box((KNUCKLE_LENGTH, BARREL_RADIUS, LEAF_THICKNESS)),
                origin=Origin(
                    xyz=(x_center, BARREL_RADIUS / 2.0, -LEAF_THICKNESS / 2.0)
                ),
                material=satin_steel,
                name=f"door_web_{index}",
            )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame_leaf,
        child=door_leaf,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=OPEN_LIMIT_RAD,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame_leaf = object_model.get_part("frame_leaf")
    door_leaf = object_model.get_part("door_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

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
            frame_leaf,
            door_leaf,
            elem_a="frame_plate",
            elem_b="door_plate",
            name="closed_leaves_touch_at_seam",
        )
        ctx.expect_origin_distance(
            frame_leaf,
            door_leaf,
            axes="yz",
            max_dist=1e-6,
            name="leaves_share_barrel_centerline",
        )
        ctx.expect_overlap(
            frame_leaf,
            door_leaf,
            axes="x",
            min_overlap=0.68,
            name="leaves_share_long_barrel_axis",
        )

        frame_aabb = ctx.part_world_aabb(frame_leaf)
        door_aabb = ctx.part_world_aabb(door_leaf)
        pin_head_visible = (
            frame_aabb is not None
            and door_aabb is not None
            and frame_aabb[0][0] < door_aabb[0][0] - 0.002
        )
        ctx.check(
            "pin_head_projects_beyond_leaf_end",
            pin_head_visible,
            details=(
                f"expected frame leaf to extend farther in -X than door leaf; "
                f"frame_aabb={frame_aabb}, door_aabb={door_aabb}"
            ),
        )

    with ctx.pose({hinge: 1.2}):
        frame_aabb = ctx.part_world_aabb(frame_leaf)
        door_aabb = ctx.part_world_aabb(door_leaf)
        opens_upward = (
            frame_aabb is not None
            and door_aabb is not None
            and door_aabb[1][2] > frame_aabb[1][2] + 0.008
        )
        ctx.check(
            "positive_rotation_lifts_door_leaf",
            opens_upward,
            details=(
                f"expected positive hinge rotation to raise the door leaf; "
                f"frame_aabb={frame_aabb}, door_aabb={door_aabb}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
