from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START

CABINET_WIDTH = 0.62
CABINET_HEIGHT = 0.72
CABINET_DEPTH = 0.24
SIDE_WALL_THICKNESS = 0.018
BACK_PANEL_THICKNESS = 0.012
DOOR_THICKNESS = 0.020
DOOR_OVERLAY = 0.002
FRONT_GAP = 0.0015
HINGE_SIDE_OFFSET = 0.002

DOOR_WIDTH = CABINET_WIDTH + (2.0 * DOOR_OVERLAY)
DOOR_HEIGHT = CABINET_HEIGHT + (2.0 * DOOR_OVERLAY)

HANDLE_WIDTH = 0.014
HANDLE_BAR_DEPTH = 0.012
HANDLE_STANDOFF_DEPTH = 0.012
HANDLE_LENGTH = 0.28
HANDLE_POST_LENGTH = 0.032
HANDLE_POST_SPACING = 0.16
HANDLE_FROM_LATCH_EDGE = 0.045


def _body_mesh():
    import cadquery as cq

    outer = cq.Workplane("XY").box(CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .box(
            CABINET_WIDTH - (2.0 * SIDE_WALL_THICKNESS),
            CABINET_DEPTH - BACK_PANEL_THICKNESS,
            CABINET_HEIGHT - (2.0 * SIDE_WALL_THICKNESS),
        )
        .translate((0.0, BACK_PANEL_THICKNESS / 2.0, 0.0))
    )
    shell = outer.cut(cavity)
    return mesh_from_cadquery(shell, MESH_DIR / "cabinet_body.obj")


def _door_panel_mesh():
    import cadquery as cq

    panel = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)
        .translate((DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, 0.0))
    )
    return mesh_from_cadquery(panel, MESH_DIR / "cabinet_door_panel.obj")


def _door_handle_mesh():
    import cadquery as cq

    handle_center_x = DOOR_WIDTH - HANDLE_FROM_LATCH_EDGE
    bar_center_y = DOOR_THICKNESS + HANDLE_STANDOFF_DEPTH + (HANDLE_BAR_DEPTH / 2.0)
    post_center_y = DOOR_THICKNESS + (HANDLE_STANDOFF_DEPTH / 2.0)

    bar = (
        cq.Workplane("XY")
        .box(HANDLE_WIDTH, HANDLE_BAR_DEPTH, HANDLE_LENGTH)
        .translate((handle_center_x, bar_center_y, 0.0))
    )
    upper_post = (
        cq.Workplane("XY")
        .box(HANDLE_WIDTH * 0.85, HANDLE_STANDOFF_DEPTH, HANDLE_POST_LENGTH)
        .translate((handle_center_x, post_center_y, HANDLE_POST_SPACING / 2.0))
    )
    lower_post = (
        cq.Workplane("XY")
        .box(HANDLE_WIDTH * 0.85, HANDLE_STANDOFF_DEPTH, HANDLE_POST_LENGTH)
        .translate((handle_center_x, post_center_y, -HANDLE_POST_SPACING / 2.0))
    )
    handle = bar.union(upper_post).union(lower_post)
    return mesh_from_cadquery(handle, MESH_DIR / "cabinet_door_handle.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_cabinet", assets=ASSETS)

    carcass_finish = model.material("carcass_finish", rgba=(0.93, 0.94, 0.95, 1.0))
    door_finish = model.material("door_finish", rgba=(0.90, 0.91, 0.92, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("cabinet_body")
    body.visual(_body_mesh(), material=carcass_finish)
    body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=8.5,
    )

    door = model.part("door")
    door.visual(_door_panel_mesh(), material=door_finish)
    door.visual(_door_handle_mesh(), material=handle_finish)
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent="cabinet_body",
        child="door",
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 - HINGE_SIDE_OFFSET,
                CABINET_DEPTH / 2.0 + FRONT_GAP,
                0.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.85,
            effort=10.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.expect_joint_motion_axis(
        "body_to_door",
        "door",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )

    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    assert len(object_model.parts) == 2, "Cabinet should contain only the body and door parts."
    assert len(object_model.articulations) == 1, (
        "Cabinet should use a single clean hinge articulation."
    )
    assert hinge.articulation_type == ArticulationType.REVOLUTE
    assert tuple(hinge.axis) == (0.0, 0.0, 1.0), (
        "Door hinge should rotate about the vertical z axis."
    )
    assert hinge.motion_limits is not None
    assert hinge.motion_limits.lower == 0.0
    assert hinge.motion_limits.upper >= 1.7
    assert hinge.origin.xyz == (
        -CABINET_WIDTH / 2.0 - HINGE_SIDE_OFFSET,
        CABINET_DEPTH / 2.0 + FRONT_GAP,
        0.0,
    )
    assert len(door.visuals) >= 2, "Door should include a separate visible pull handle."

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
