from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

WALL_T = 0.004
BOTTOM_T = 0.004
RUNNER_T = 0.003
RUNNER_H = 0.010
RUNNER_START = 0.050

OUTER_LENGTH = 0.340
OUTER_WIDTH = 0.240
OUTER_HEIGHT = 0.080

MIDDLE_LENGTH = 0.290
MIDDLE_WIDTH = 0.212
MIDDLE_HEIGHT = 0.066

INNER_LENGTH = 0.240
INNER_WIDTH = 0.184
INNER_HEIGHT = 0.054

STAGE_OFFSET_X = 0.010
STAGE_OFFSET_Z = 0.006

MIDDLE_TRAVEL = 0.190
INNER_TRAVEL = 0.072


def _add_box(part, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material)


def _tray_runner_length(length: float) -> float:
    return max(length - 0.100, length * 0.58)


def _add_tray(
    part,
    *,
    length: float,
    width: float,
    height: float,
    body_material: str,
    runner_material: str,
    accent_material: str,
    front_drop: float,
    front_band_depth: float,
    add_front_wall: bool,
    top_bridge_height: float,
    add_inner_runners: bool,
    add_outer_runners: bool,
):
    wall_height = height - BOTTOM_T
    front_height = max(0.018, wall_height - front_drop)
    side_z = BOTTOM_T + wall_height / 2.0
    front_z = BOTTOM_T + front_height / 2.0

    _add_box(
        part,
        (length, width, BOTTOM_T),
        (length / 2.0, 0.0, BOTTOM_T / 2.0),
        body_material,
    )
    _add_box(
        part,
        (length, WALL_T, wall_height),
        (length / 2.0, width / 2.0 - WALL_T / 2.0, side_z),
        body_material,
    )
    _add_box(
        part,
        (length, WALL_T, wall_height),
        (length / 2.0, -width / 2.0 + WALL_T / 2.0, side_z),
        body_material,
    )
    _add_box(
        part,
        (WALL_T, width - 2.0 * WALL_T, wall_height),
        (WALL_T / 2.0, 0.0, side_z),
        body_material,
    )
    if add_front_wall:
        _add_box(
            part,
            (WALL_T, width - 2.0 * WALL_T, front_height),
            (length - WALL_T / 2.0, 0.0, front_z),
            body_material,
        )
    elif top_bridge_height > 0.0:
        _add_box(
            part,
            (WALL_T, width - 2.0 * WALL_T, top_bridge_height),
            (length - WALL_T / 2.0, 0.0, height - top_bridge_height / 2.0),
            body_material,
        )

    if front_band_depth > 0.0:
        band_z = (
            BOTTOM_T + front_height * 0.56
            if add_front_wall
            else height - max(top_bridge_height, 0.004) / 2.0
        )
        _add_box(
            part,
            (front_band_depth, width * 0.64, min(front_height * 0.52, 0.024)),
            (
                length + front_band_depth / 2.0,
                0.0,
                band_z,
            ),
            accent_material,
        )

    runner_len = _tray_runner_length(length)
    runner_x = RUNNER_START + runner_len / 2.0
    runner_z = BOTTOM_T + 0.013

    if add_inner_runners:
        inner_y = width / 2.0 - WALL_T - RUNNER_T / 2.0
        _add_box(
            part,
            (runner_len, RUNNER_T, RUNNER_H),
            (runner_x, inner_y, runner_z),
            runner_material,
        )
        _add_box(
            part,
            (runner_len, RUNNER_T, RUNNER_H),
            (runner_x, -inner_y, runner_z),
            runner_material,
        )

    if add_outer_runners:
        outer_y = width / 2.0 + RUNNER_T / 2.0
        _add_box(
            part,
            (runner_len, RUNNER_T, RUNNER_H),
            (runner_x, outer_y, runner_z),
            runner_material,
        )
        _add_box(
            part,
            (runner_len, RUNNER_T, RUNNER_H),
            (runner_x, -outer_y, runner_z),
            runner_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_storage_drawer")

    model.material("outer_body", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("middle_body", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("inner_body", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("runner_dark", rgba=(0.10, 0.11, 0.13, 1.0))
    model.material("grip_outer", rgba=(0.42, 0.46, 0.52, 1.0))
    model.material("grip_middle", rgba=(0.18, 0.45, 0.70, 1.0))
    model.material("grip_inner", rgba=(0.83, 0.48, 0.18, 1.0))

    outer = model.part("outer_tray")
    _add_tray(
        outer,
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        body_material="outer_body",
        runner_material="runner_dark",
        accent_material="grip_outer",
        front_drop=0.022,
        front_band_depth=0.004,
        add_front_wall=False,
        top_bridge_height=0.006,
        add_inner_runners=True,
        add_outer_runners=False,
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH + 0.004, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=1.5,
        origin=Origin(xyz=((OUTER_LENGTH + 0.004) / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    middle = model.part("middle_tray")
    _add_tray(
        middle,
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        body_material="middle_body",
        runner_material="runner_dark",
        accent_material="grip_middle",
        front_drop=0.014,
        front_band_depth=0.006,
        add_front_wall=False,
        top_bridge_height=0.004,
        add_inner_runners=True,
        add_outer_runners=True,
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH + 0.006, MIDDLE_WIDTH + 2.0 * RUNNER_T, MIDDLE_HEIGHT)),
        mass=0.95,
        origin=Origin(
            xyz=(
                (MIDDLE_LENGTH + 0.006) / 2.0,
                0.0,
                MIDDLE_HEIGHT / 2.0,
            )
        ),
    )

    inner = model.part("inner_tray")
    _add_tray(
        inner,
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        body_material="inner_body",
        runner_material="runner_dark",
        accent_material="grip_inner",
        front_drop=0.010,
        front_band_depth=0.006,
        add_front_wall=True,
        top_bridge_height=0.0,
        add_inner_runners=False,
        add_outer_runners=True,
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH + 0.006, INNER_WIDTH + 2.0 * RUNNER_T, INNER_HEIGHT)),
        mass=0.60,
        origin=Origin(
            xyz=(
                (INNER_LENGTH + 0.006) / 2.0,
                0.0,
                INNER_HEIGHT / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_middle_slide",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(STAGE_OFFSET_X, 0.0, STAGE_OFFSET_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TRAVEL,
            effort=120.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "middle_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(STAGE_OFFSET_X, 0.0, STAGE_OFFSET_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=90.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("middle_tray", "outer_tray", axes="y", max_dist=0.001)
    ctx.expect_origin_distance("inner_tray", "middle_tray", axes="y", max_dist=0.001)
    ctx.expect_aabb_overlap("middle_tray", "outer_tray", axes="yz", min_overlap=0.06)
    ctx.expect_aabb_overlap("inner_tray", "middle_tray", axes="yz", min_overlap=0.05)
    ctx.expect_aabb_overlap("middle_tray", "outer_tray", axes="x", min_overlap=0.25)
    ctx.expect_aabb_overlap("inner_tray", "middle_tray", axes="x", min_overlap=0.20)
    ctx.expect_joint_motion_axis(
        "outer_to_middle_slide",
        "middle_tray",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_inner_slide",
        "inner_tray",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(middle_to_inner_slide=INNER_TRAVEL):
        ctx.expect_aabb_overlap("inner_tray", "middle_tray", axes="yz", min_overlap=0.05)
        ctx.expect_aabb_overlap("inner_tray", "middle_tray", axes="x", min_overlap=0.16)
        ctx.expect_aabb_overlap("inner_tray", "outer_tray", axes="yz", min_overlap=0.05)
        ctx.expect_aabb_overlap("inner_tray", "outer_tray", axes="x", min_overlap=0.20)

    with ctx.pose(outer_to_middle_slide=MIDDLE_TRAVEL):
        ctx.expect_aabb_overlap("middle_tray", "outer_tray", axes="yz", min_overlap=0.06)
        ctx.expect_aabb_overlap("middle_tray", "outer_tray", axes="x", min_overlap=0.12)
        ctx.expect_aabb_overlap("inner_tray", "middle_tray", axes="x", min_overlap=0.20)

    with ctx.pose(
        outer_to_middle_slide=MIDDLE_TRAVEL,
        middle_to_inner_slide=INNER_TRAVEL,
    ):
        ctx.expect_aabb_overlap("middle_tray", "outer_tray", axes="x", min_overlap=0.12)
        ctx.expect_aabb_overlap("inner_tray", "middle_tray", axes="yz", min_overlap=0.05)
        ctx.expect_aabb_overlap("inner_tray", "middle_tray", axes="x", min_overlap=0.10)
        ctx.expect_aabb_overlap("inner_tray", "outer_tray", axes="yz", min_overlap=0.05)
        ctx.expect_aabb_overlap("inner_tray", "outer_tray", axes="x", min_overlap=0.06)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
