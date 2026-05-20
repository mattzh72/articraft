from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
FRAME_TO_STAGE1_STROKE = 0.030
STAGE1_TO_STAGE2_STROKE = 0.025
STAGE2_TO_STAGE3_STROKE = 0.022


def _x_cylinder(radius: float, length: float, start_x: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((start_x, 0.0, 0.0))


def _x_hollow_cylinder(
    inner_radius: float,
    outer_radius: float,
    length: float,
    start_x: float,
) -> cq.Workplane:
    outer = _x_cylinder(outer_radius, length, start_x)
    inner = _x_cylinder(inner_radius, length + 0.002, start_x - 0.001)
    return outer.cut(inner)


def _box(
    length: float, width: float, height: float, center: tuple[float, float, float]
) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _add_box_collision(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> None:
    pass


def _add_sleeve_wall_collisions(
    part,
    *,
    prefix: str,
    center_x: float,
    length: float,
    inner_span: float,
    wall: float,
) -> None:
    outer_span = inner_span + 2.0 * wall
    wall_offset = inner_span / 2.0 + wall / 2.0

    _add_box_collision(
        part,
        name=f"{prefix}_top",
        size=(length, outer_span, wall),
        center=(center_x, 0.0, wall_offset),
    )
    _add_box_collision(
        part,
        name=f"{prefix}_bottom",
        size=(length, outer_span, wall),
        center=(center_x, 0.0, -wall_offset),
    )
    _add_box_collision(
        part,
        name=f"{prefix}_left",
        size=(length, wall, inner_span),
        center=(center_x, wall_offset, 0.0),
    )
    _add_box_collision(
        part,
        name=f"{prefix}_right",
        size=(length, wall, inner_span),
        center=(center_x, -wall_offset, 0.0),
    )


def _frame_shape() -> cq.Workplane:
    body = _box(0.190, 0.100, 0.044, (-0.050, 0.0, 0.000))
    rear_block = _box(0.060, 0.110, 0.078, (-0.135, 0.0, 0.017))
    foot = _box(0.240, 0.060, 0.012, (-0.030, 0.0, -0.028))
    for hole_x in (-0.115, 0.055):
        foot = foot.cut(
            cq.Workplane("XY").circle(0.009).extrude(0.020).translate((hole_x, 0.0, -0.040))
        )

    guide = _x_hollow_cylinder(0.0128, 0.0180, 0.066, -0.012)
    nose = _x_hollow_cylinder(0.0128, 0.0220, 0.008, 0.054)
    gland = _box(0.024, 0.062, 0.036, (0.010, 0.0, 0.000))
    oil_boss = _x_cylinder(0.008, 0.012, -0.048).translate((0.0, 0.0, 0.030))

    return body.union(rear_block).union(foot).union(guide).union(nose).union(gland).union(oil_boss)


def _stage1_shape() -> cq.Workplane:
    rod = _x_cylinder(0.0105, 0.115, -0.055)
    seal = _x_cylinder(0.0150, 0.010, 0.002)
    wrench_flat = _box(0.024, 0.026, 0.018, (0.042, 0.0, 0.000))
    front_sleeve = _x_hollow_cylinder(0.0087, 0.0145, 0.055, 0.055)
    front_collar = _x_hollow_cylinder(0.0087, 0.0170, 0.008, 0.110)
    return rod.union(seal).union(wrench_flat).union(front_sleeve).union(front_collar)


def _stage2_shape() -> cq.Workplane:
    rod = _x_cylinder(0.0070, 0.096, -0.045)
    seal = _x_cylinder(0.0115, 0.008, 0.000)
    wrench_flat = _box(0.018, 0.020, 0.014, (0.028, 0.0, 0.000))
    front_sleeve = _x_hollow_cylinder(0.0058, 0.0108, 0.050, 0.045)
    front_collar = _x_hollow_cylinder(0.0058, 0.0125, 0.007, 0.095)
    return rod.union(seal).union(wrench_flat).union(front_sleeve).union(front_collar)


def _stage3_shape() -> cq.Workplane:
    rod = _x_cylinder(0.0045, 0.090, -0.040)
    seal = _x_cylinder(0.0078, 0.008, -0.002)
    tip_shank = _x_cylinder(0.0055, 0.014, 0.050)
    contact_pad = _x_cylinder(0.0085, 0.005, 0.064)
    ball_tip = cq.Workplane("XY").sphere(0.0065).translate((0.071, 0.0, 0.0))
    return rod.union(seal).union(tip_shank).union(contact_pad).union(ball_tip)


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger_chain", assets=ASSETS)

    model.material("housing", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("tool_steel", rgba=(0.48, 0.50, 0.54, 1.0))

    frame = model.part("frame")
    _add_mesh_visual(frame, _frame_shape(), "frame.obj", "housing")
    _add_box_collision(
        frame, name="rear_body", size=(0.125, 0.100, 0.052), center=(-0.0875, 0.0, 0.000)
    )
    _add_box_collision(
        frame, name="rear_block", size=(0.055, 0.110, 0.075), center=(-0.135, 0.0, 0.014)
    )
    _add_box_collision(
        frame, name="base_foot", size=(0.240, 0.060, 0.012), center=(-0.030, 0.0, -0.028)
    )
    _add_sleeve_wall_collisions(
        frame, prefix="guide", center_x=0.025, length=0.074, inner_span=0.026, wall=0.005
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.240, 0.110, 0.090)),
        mass=4.2,
        origin=Origin(xyz=(-0.055, 0.0, -0.004)),
    )

    stage1 = model.part("stage1")
    _add_mesh_visual(stage1, _stage1_shape(), "stage1.obj", "steel")
    _add_box_collision(stage1, name="rod", size=(0.112, 0.021, 0.021), center=(0.001, 0.0, 0.0))
    _add_sleeve_wall_collisions(
        stage1, prefix="guide", center_x=0.0825, length=0.055, inner_span=0.018, wall=0.005
    )
    stage1.inertial = Inertial.from_geometry(
        Box((0.175, 0.030, 0.030)),
        mass=1.1,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    stage2 = model.part("stage2")
    _add_mesh_visual(stage2, _stage2_shape(), "stage2.obj", "steel")
    _add_box_collision(stage2, name="rod", size=(0.090, 0.014, 0.014), center=(0.000, 0.0, 0.0))
    _add_sleeve_wall_collisions(
        stage2, prefix="guide", center_x=0.070, length=0.050, inner_span=0.012, wall=0.004
    )
    stage2.inertial = Inertial.from_geometry(
        Box((0.145, 0.022, 0.022)),
        mass=0.55,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    stage3 = model.part("stage3")
    _add_mesh_visual(stage3, _stage3_shape(), "stage3.obj", "tool_steel")
    _add_box_collision(stage3, name="rod", size=(0.090, 0.009, 0.009), center=(0.005, 0.0, 0.0))
    _add_box_collision(stage3, name="tip", size=(0.018, 0.015, 0.015), center=(0.063, 0.0, 0.0))
    stage3.inertial = Inertial.from_geometry(
        Box((0.115, 0.016, 0.016)),
        mass=0.18,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_stage1",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage1,
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FRAME_TO_STAGE1_STROKE,
            effort=180.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE1_TO_STAGE2_STROKE,
            effort=120.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE2_TO_STAGE3_STROKE,
            effort=80.0,
            velocity=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("stage1", "frame", axes="xy", min_overlap=0.018)
    ctx.expect_aabb_overlap("stage2", "stage1", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("stage3", "stage2", axes="xy", min_overlap=0.008)

    ctx.expect_joint_motion_axis(
        "frame_to_stage1",
        "stage1",
        world_axis="x",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "stage1_to_stage2",
        "stage2",
        world_axis="x",
        direction="positive",
        min_delta=0.012,
    )
    ctx.expect_joint_motion_axis(
        "stage2_to_stage3",
        "stage3",
        world_axis="x",
        direction="positive",
        min_delta=0.010,
    )

    with ctx.pose(
        frame_to_stage1=FRAME_TO_STAGE1_STROKE,
        stage1_to_stage2=STAGE1_TO_STAGE2_STROKE,
        stage2_to_stage3=STAGE2_TO_STAGE3_STROKE,
    ):
        ctx.expect_aabb_overlap("stage1", "frame", axes="xy", min_overlap=0.018)
        ctx.expect_aabb_overlap("stage2", "stage1", axes="xy", min_overlap=0.012)
        ctx.expect_aabb_overlap("stage3", "stage2", axes="xy", min_overlap=0.008)

    with ctx.pose(
        frame_to_stage1=FRAME_TO_STAGE1_STROKE * 0.5,
        stage1_to_stage2=STAGE1_TO_STAGE2_STROKE * 0.5,
        stage2_to_stage3=STAGE2_TO_STAGE3_STROKE * 0.5,
    ):
        ctx.expect_aabb_overlap("stage1", "frame", axes="xy", min_overlap=0.018)
        ctx.expect_aabb_overlap("stage2", "stage1", axes="xy", min_overlap=0.012)
        ctx.expect_aabb_overlap("stage3", "stage2", axes="xy", min_overlap=0.008)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
