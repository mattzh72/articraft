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


SUPPORT_LENGTH = 0.62
SUPPORT_WIDTH = 0.085
SUPPORT_PLATE_THICKNESS = 0.006
SUPPORT_DROP = 0.030
SUPPORT_BLOCK_LENGTH = 0.034
SUPPORT_BLOCK_WIDTH = 0.024

OUTER_LENGTH = 0.50
OUTER_WIDTH = 0.050
OUTER_HEIGHT = 0.030
OUTER_TOP_THICKNESS = 0.003
OUTER_WALL_THICKNESS = 0.003
OUTER_LIP_DEPTH = 0.006
OUTER_LIP_THICKNESS = 0.002

RUNNER_LENGTH = 0.46
RUNNER_HEAD_WIDTH = 0.040
RUNNER_HEAD_THICKNESS = 0.003
RUNNER_BODY_WIDTH = 0.024
RUNNER_BODY_HEIGHT = 0.017

OUTER_X_ON_SUPPORT = (SUPPORT_LENGTH - OUTER_LENGTH) / 2.0
RUNNER_HOME_INSET = 0.020
RUNNER_TOP_Z_IN_OUTER = -0.025
RUNNER_TRAVEL = 0.22


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    *,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _support_shape() -> cq.Workplane:
    slot_x = SUPPORT_LENGTH / 2.0 - 0.10
    slot_y = 0.024

    plate = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH,
            SUPPORT_WIDTH,
            SUPPORT_PLATE_THICKNESS,
            centered=(False, True, False),
        )
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints(
            [
                (-slot_x, -slot_y),
                (-slot_x, slot_y),
                (slot_x, -slot_y),
                (slot_x, slot_y),
            ]
        )
        .slot2D(0.018, 0.006, angle=0.0)
        .cutThruAll()
    )

    left_block_x = OUTER_X_ON_SUPPORT + 0.13
    right_block_x = OUTER_X_ON_SUPPORT + OUTER_LENGTH - 0.13

    for block_x in (left_block_x, right_block_x):
        block = (
            cq.Workplane("XY")
            .box(
                SUPPORT_BLOCK_LENGTH,
                SUPPORT_BLOCK_WIDTH,
                SUPPORT_DROP,
                centered=(True, True, False),
            )
            .translate((block_x, 0.0, -SUPPORT_DROP))
        )
        plate = plate.union(block)

    return plate


def _outer_sleeve_shape() -> cq.Workplane:
    top = (
        cq.Workplane("XY")
        .box(
            OUTER_LENGTH,
            OUTER_WIDTH,
            OUTER_TOP_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, -OUTER_TOP_THICKNESS))
    )

    left_wall = (
        cq.Workplane("XY")
        .box(
            OUTER_LENGTH,
            OUTER_WALL_THICKNESS,
            OUTER_HEIGHT - OUTER_TOP_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.0, OUTER_WIDTH / 2.0 - OUTER_WALL_THICKNESS / 2.0, -OUTER_HEIGHT))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(
            OUTER_LENGTH,
            OUTER_WALL_THICKNESS,
            OUTER_HEIGHT - OUTER_TOP_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.0, -OUTER_WIDTH / 2.0 + OUTER_WALL_THICKNESS / 2.0, -OUTER_HEIGHT))
    )

    left_lip = (
        cq.Workplane("XY")
        .box(
            OUTER_LENGTH,
            OUTER_LIP_DEPTH,
            OUTER_LIP_THICKNESS,
            centered=(False, True, False),
        )
        .translate(
            (
                0.0,
                OUTER_WIDTH / 2.0
                - OUTER_WALL_THICKNESS
                - OUTER_LIP_DEPTH / 2.0,
                -OUTER_HEIGHT,
            )
        )
    )
    right_lip = (
        cq.Workplane("XY")
        .box(
            OUTER_LENGTH,
            OUTER_LIP_DEPTH,
            OUTER_LIP_THICKNESS,
            centered=(False, True, False),
        )
        .translate(
            (
                0.0,
                -OUTER_WIDTH / 2.0
                + OUTER_WALL_THICKNESS
                + OUTER_LIP_DEPTH / 2.0,
                -OUTER_HEIGHT,
            )
        )
    )

    return top.union(left_wall).union(right_wall).union(left_lip).union(right_lip)


def _runner_shape() -> cq.Workplane:
    head = (
        cq.Workplane("XY")
        .box(
            RUNNER_LENGTH,
            RUNNER_HEAD_WIDTH,
            RUNNER_HEAD_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, -RUNNER_HEAD_THICKNESS))
    )
    body = (
        cq.Workplane("XY")
        .box(
            RUNNER_LENGTH,
            RUNNER_BODY_WIDTH,
            RUNNER_BODY_HEIGHT,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, -(RUNNER_HEAD_THICKNESS + RUNNER_BODY_HEIGHT)))
    )
    front_plate = (
        cq.Workplane("XY")
        .box(0.006, 0.030, 0.022, centered=(False, True, False))
        .translate((RUNNER_LENGTH - 0.006, 0.0, -0.022))
    )
    rear_stop = (
        cq.Workplane("XY")
        .box(0.010, RUNNER_BODY_WIDTH, 0.008, centered=(False, True, False))
        .translate((0.0, 0.0, -0.011))
    )

    return head.union(body).union(front_plate).union(rear_stop)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_telescoping_runner")

    model.material("support_aluminum", rgba=(0.82, 0.83, 0.85, 1.0))
    model.material("outer_steel", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("runner_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    support = model.part("support")
    _add_mesh_visual(
        support,
        _support_shape(),
        mesh_name="support_body",
        material="support_aluminum",
        visual_name="support_body",
    )

    outer_sleeve = model.part("outer_sleeve")
    _add_mesh_visual(
        outer_sleeve,
        _outer_sleeve_shape(),
        mesh_name="outer_sleeve_body",
        material="outer_steel",
        visual_name="outer_sleeve_body",
    )

    runner = model.part("runner")
    _add_mesh_visual(
        runner,
        _runner_shape(),
        mesh_name="runner_body",
        material="runner_steel",
        visual_name="runner_body",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support,
        child=outer_sleeve,
        origin=Origin(xyz=(OUTER_X_ON_SUPPORT, 0.0, -SUPPORT_DROP)),
    )
    model.articulation(
        "outer_to_runner",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=runner,
        origin=Origin(xyz=(RUNNER_HOME_INSET, 0.0, RUNNER_TOP_Z_IN_OUTER)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=RUNNER_TRAVEL,
            effort=180.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    outer_sleeve = object_model.get_part("outer_sleeve")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("outer_to_runner")

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

    ctx.expect_contact(support, outer_sleeve, name="support_carries_outer_sleeve")
    ctx.expect_contact(outer_sleeve, runner, name="runner_is_captured_by_outer_sleeve")
    ctx.expect_within(
        runner,
        outer_sleeve,
        axes="y",
        margin=0.001,
        name="runner_stays_between_outer_side_walls",
    )
    ctx.expect_gap(
        support,
        runner,
        axis="z",
        min_gap=0.020,
        max_gap=0.030,
        name="runner_hangs_below_support",
    )

    with ctx.pose({slide: slide.motion_limits.upper or 0.0}):
        ctx.expect_contact(
            outer_sleeve,
            runner,
            name="runner_remains_supported_when_extended",
        )
        ctx.expect_within(
            runner,
            outer_sleeve,
            axes="y",
            margin=0.001,
            name="runner_stays_laterally_captured_when_extended",
        )

        outer_aabb = ctx.part_world_aabb(outer_sleeve)
        runner_aabb = ctx.part_world_aabb(runner)
        if outer_aabb is None or runner_aabb is None:
            ctx.fail("extended_pose_aabbs_available", "missing world AABB for sleeve or runner")
        else:
            outer_bottom_z = outer_aabb[0][2]
            runner_bottom_z = runner_aabb[0][2]
            ctx.check(
                "runner_projects_below_outer_sleeve",
                runner_bottom_z < outer_bottom_z - 0.010,
                (
                    f"expected runner to hang visibly below sleeve; "
                    f"runner_bottom_z={runner_bottom_z:.4f}, outer_bottom_z={outer_bottom_z:.4f}"
                ),
            )

    closed_position = None
    extended_position = None
    with ctx.pose({slide: 0.0}):
        closed_position = ctx.part_world_position(runner)
    with ctx.pose({slide: slide.motion_limits.upper or 0.0}):
        extended_position = ctx.part_world_position(runner)

    if closed_position is None or extended_position is None:
        ctx.fail("runner_pose_positions_available", "missing runner world position in closed or extended pose")
    else:
        delta_x = extended_position[0] - closed_position[0]
        delta_y = extended_position[1] - closed_position[1]
        delta_z = extended_position[2] - closed_position[2]
        ctx.check(
            "runner_translates_along_slide_axis",
            abs(delta_x - RUNNER_TRAVEL) < 0.002 and abs(delta_y) < 1e-6 and abs(delta_z) < 1e-6,
            (
                f"expected pure +X prismatic travel of {RUNNER_TRAVEL:.3f} m, "
                f"got Δx={delta_x:.4f}, Δy={delta_y:.6f}, Δz={delta_z:.6f}"
            ),
        )

    ctx.check(
        "slide_joint_is_prismatic_along_x",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"joint type={slide.articulation_type}, axis={slide.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
