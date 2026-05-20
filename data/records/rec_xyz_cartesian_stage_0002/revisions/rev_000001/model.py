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

BASE_X_TRAVEL = 0.075
Y_STAGE_TRAVEL = 0.040
Z_STAGE_TRAVEL = 0.045


def _add_box(part, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage")

    model.material("anodized_black", rgba=(0.16, 0.17, 0.20, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("rail_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    model.material("plate_blue", rgba=(0.22, 0.35, 0.66, 1.0))
    model.material("dark_trim", rgba=(0.10, 0.11, 0.13, 1.0))

    base = model.part("base")
    _add_box(base, (0.36, 0.25, 0.03), (0.0, 0.0, 0.015), "anodized_black")
    _add_box(base, (0.30, 0.028, 0.010), (0.0, -0.092, 0.005), "dark_trim")
    _add_box(base, (0.30, 0.028, 0.010), (0.0, 0.092, 0.005), "dark_trim")
    _add_box(base, (0.28, 0.024, 0.014), (0.0, -0.078, 0.037), "rail_steel")
    _add_box(base, (0.28, 0.024, 0.014), (0.0, 0.078, 0.037), "rail_steel")
    _add_box(base, (0.05, 0.16, 0.022), (-0.145, 0.0, 0.021), "dark_trim")
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.25, 0.044)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    x_gantry = model.part("x_gantry")
    _add_box(x_gantry, (0.19, 0.20, 0.028), (0.0, 0.0, 0.021), "machined_aluminum")
    _add_box(x_gantry, (0.16, 0.19, 0.012), (0.0, 0.0, 0.041), "anodized_black")
    _add_box(x_gantry, (0.020, 0.180, 0.012), (-0.032, 0.0, 0.053), "rail_steel")
    _add_box(x_gantry, (0.020, 0.180, 0.012), (0.032, 0.0, 0.053), "rail_steel")
    _add_box(x_gantry, (0.145, 0.020, 0.010), (0.0, -0.085, 0.040), "dark_trim")
    _add_box(x_gantry, (0.145, 0.020, 0.010), (0.0, 0.085, 0.040), "dark_trim")
    x_gantry.inertial = Inertial.from_geometry(
        Box((0.19, 0.20, 0.052)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
    )

    y_saddle = model.part("y_saddle")
    _add_box(y_saddle, (0.112, 0.120, 0.024), (-0.016, 0.0, 0.018), "machined_aluminum")
    _add_box(y_saddle, (0.094, 0.100, 0.016), (-0.012, 0.0, 0.038), "anodized_black")
    _add_box(y_saddle, (0.028, 0.024, 0.180), (0.021, -0.033, 0.120), "machined_aluminum")
    _add_box(y_saddle, (0.028, 0.024, 0.180), (0.021, 0.033, 0.120), "machined_aluminum")
    _add_box(y_saddle, (0.010, 0.084, 0.150), (0.035, 0.0, 0.105), "anodized_black")
    _add_box(y_saddle, (0.028, 0.090, 0.016), (0.021, 0.0, 0.202), "anodized_black")
    _add_box(y_saddle, (0.006, 0.024, 0.150), (0.038, -0.033, 0.105), "rail_steel")
    _add_box(y_saddle, (0.006, 0.024, 0.150), (0.038, 0.033, 0.105), "rail_steel")
    y_saddle.inertial = Inertial.from_geometry(
        Box((0.118, 0.120, 0.204)),
        mass=1.9,
        origin=Origin(xyz=(0.006, 0.0, 0.098)),
    )

    z_carriage = model.part("z_carriage")
    _add_box(z_carriage, (0.032, 0.076, 0.090), (0.019, 0.0, 0.0), "machined_aluminum")
    _add_box(z_carriage, (0.036, 0.050, 0.014), (0.053, 0.0, -0.038), "anodized_black")
    _add_box(z_carriage, (0.008, 0.078, 0.078), (0.075, 0.0, -0.038), "plate_blue")
    _add_box(z_carriage, (0.012, 0.024, 0.024), (0.085, 0.0, -0.038), "dark_trim")
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.078, 0.123)),
        mass=0.9,
        origin=Origin(xyz=(0.048, 0.0, -0.016)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_gantry,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BASE_X_TRAVEL,
            upper=BASE_X_TRAVEL,
            effort=120.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_gantry,
        child=y_saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_STAGE_TRAVEL,
            upper=Y_STAGE_TRAVEL,
            effort=90.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_saddle,
        child=z_carriage,
        origin=Origin(xyz=(0.038, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-Z_STAGE_TRAVEL,
            upper=Z_STAGE_TRAVEL,
            effort=70.0,
            velocity=0.18,
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

    ctx.expect_aabb_gap("x_gantry", "base", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_overlap("x_gantry", "base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_gap("y_saddle", "x_gantry", axis="z", max_gap=0.001, max_penetration=1e-5)
    ctx.expect_aabb_overlap("y_saddle", "x_gantry", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_gap("z_carriage", "y_saddle", axis="x", max_gap=0.001, max_penetration=1e-5)
    ctx.expect_aabb_overlap("z_carriage", "y_saddle", axes="yz", min_overlap=0.05)

    ctx.expect_joint_motion_axis(
        "base_to_x",
        "x_gantry",
        world_axis="x",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "x_to_y",
        "y_saddle",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "y_to_z",
        "z_carriage",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(base_to_x=BASE_X_TRAVEL, x_to_y=Y_STAGE_TRAVEL, y_to_z=Z_STAGE_TRAVEL):
        ctx.expect_aabb_gap("x_gantry", "base", axis="z", max_gap=0.001, max_penetration=0.0)
        ctx.expect_aabb_overlap("x_gantry", "base", axes="xy", min_overlap=0.17)
        ctx.expect_aabb_gap("y_saddle", "x_gantry", axis="z", max_gap=0.001, max_penetration=1e-5)
        ctx.expect_aabb_overlap("y_saddle", "x_gantry", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_gap("z_carriage", "y_saddle", axis="x", max_gap=0.001, max_penetration=1e-5)
        ctx.expect_aabb_overlap("z_carriage", "y_saddle", axes="yz", min_overlap=0.05)

    with ctx.pose(base_to_x=-BASE_X_TRAVEL, x_to_y=-Y_STAGE_TRAVEL, y_to_z=-Z_STAGE_TRAVEL):
        ctx.expect_aabb_gap("x_gantry", "base", axis="z", max_gap=0.001, max_penetration=0.0)
        ctx.expect_aabb_overlap("x_gantry", "base", axes="xy", min_overlap=0.17)
        ctx.expect_aabb_gap("y_saddle", "x_gantry", axis="z", max_gap=0.001, max_penetration=1e-5)
        ctx.expect_aabb_overlap("y_saddle", "x_gantry", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_gap("z_carriage", "y_saddle", axis="x", max_gap=0.001, max_penetration=1e-5)
        ctx.expect_aabb_overlap("z_carriage", "y_saddle", axes="yz", min_overlap=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
