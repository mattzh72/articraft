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
def _rounded_box(
    size: tuple[float, float, float], center: tuple[float, float, float], fillet: float
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size)
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid.translate(center)


def _gusset(
    *,
    center_x: float,
    base_y: float,
    base_z: float,
    depth_y: float,
    height_z: float,
    width_x: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline([(0.0, 0.0), (depth_y, 0.0), (0.0, height_z)])
        .close()
        .extrude(width_x, both=True)
        .translate((center_x, base_y, base_z))
    )


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_visuals() -> tuple[cq.Workplane, cq.Workplane]:
    base_plate = _rounded_box((0.52, 0.18, 0.02), (0.0, 0.0, 0.01), 0.006)
    foot_fl = _rounded_box((0.05, 0.035, 0.008), (0.19, 0.065, 0.004), 0.002)
    foot_fr = _rounded_box((0.05, 0.035, 0.008), (0.19, -0.065, 0.004), 0.002)
    foot_rl = _rounded_box((0.05, 0.035, 0.008), (-0.19, 0.065, 0.004), 0.002)
    foot_rr = _rounded_box((0.05, 0.035, 0.008), (-0.19, -0.065, 0.004), 0.002)
    structure = base_plate.union(foot_fl).union(foot_fr).union(foot_rl).union(foot_rr)

    rail_left = _rounded_box((0.40, 0.018, 0.012), (0.0, 0.055, 0.026), 0.002)
    rail_right = _rounded_box((0.40, 0.018, 0.012), (0.0, -0.055, 0.026), 0.002)
    pad_lf = _rounded_box((0.028, 0.024, 0.012), (0.186, 0.055, 0.026), 0.0015)
    pad_lr = _rounded_box((0.028, 0.024, 0.012), (-0.186, 0.055, 0.026), 0.0015)
    pad_rf = _rounded_box((0.028, 0.024, 0.012), (0.186, -0.055, 0.026), 0.0015)
    pad_rr = _rounded_box((0.028, 0.024, 0.012), (-0.186, -0.055, 0.026), 0.0015)
    rails = rail_left.union(rail_right).union(pad_lf).union(pad_lr).union(pad_rf).union(pad_rr)
    return structure, rails


def _build_x_stage_visuals() -> tuple[cq.Workplane, cq.Workplane]:
    left_truck = _rounded_box((0.055, 0.030, 0.018), (0.0, 0.0, 0.009), 0.002)
    right_truck = _rounded_box((0.055, 0.030, 0.018), (0.0, -0.110, 0.009), 0.002)
    carriage_plate = _rounded_box((0.13, 0.15, 0.014), (0.0, -0.055, 0.025), 0.004)
    backplate = _rounded_box((0.10, 0.018, 0.26), (0.0, -0.070, 0.162), 0.003)
    top_cap = _rounded_box((0.078, 0.026, 0.010), (0.0, -0.060, 0.287), 0.002)
    gusset_left = _gusset(
        center_x=0.036,
        base_y=-0.061,
        base_z=0.032,
        depth_y=0.048,
        height_z=0.085,
        width_x=0.016,
    )
    gusset_right = _gusset(
        center_x=-0.036,
        base_y=-0.061,
        base_z=0.032,
        depth_y=0.048,
        height_z=0.085,
        width_x=0.016,
    )
    body = (
        left_truck.union(right_truck)
        .union(carriage_plate)
        .union(backplate)
        .union(top_cap)
        .union(gusset_left)
        .union(gusset_right)
    )

    vertical_rail = _rounded_box((0.028, 0.014, 0.20), (0.0, -0.049, 0.155), 0.002)
    rail_head = _rounded_box((0.040, 0.016, 0.010), (0.0, -0.049, 0.262), 0.0015)
    rail_foot = _rounded_box((0.040, 0.016, 0.010), (0.0, -0.049, 0.048), 0.0015)
    rail = vertical_rail.union(rail_head).union(rail_foot)
    return body, rail


def _build_z_stage_visual() -> cq.Workplane:
    carriage_block = _rounded_box((0.076, 0.038, 0.080), (0.0, 0.019, 0.0), 0.004)
    tool_plate = _rounded_box((0.11, 0.012, 0.13), (0.0, 0.051, 0.0), 0.004)
    lower_shelf = _rounded_box((0.09, 0.028, 0.012), (0.0, 0.032, -0.046), 0.002)
    top_cap = _rounded_box((0.060, 0.018, 0.010), (0.0, 0.022, 0.045), 0.0015)
    return carriage_block.union(tool_plate).union(lower_shelf).union(top_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xz_stage", assets=ASSETS)

    model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("anodized", rgba=(0.56, 0.60, 0.64, 1.0))
    model.material("light_panel", rgba=(0.82, 0.84, 0.86, 1.0))

    base = model.part("base")
    base_structure, base_rails = _build_base_visuals()
    _add_mesh_visual(base, base_structure, "base_structure.obj", "graphite")
    _add_mesh_visual(base, base_rails, "base_rails.obj", "steel")



    base.inertial = Inertial.from_geometry(
        Box((0.52, 0.18, 0.032)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    x_stage = model.part("x_stage")
    x_body, x_rail = _build_x_stage_visuals()
    _add_mesh_visual(x_stage, x_body, "x_stage_body.obj", "anodized")
    _add_mesh_visual(x_stage, x_rail, "x_stage_rail.obj", "steel")





    x_stage.inertial = Inertial.from_geometry(
        Box((0.13, 0.15, 0.29)),
        mass=6.5,
        origin=Origin(xyz=(0.0, -0.055, 0.145)),
    )

    z_stage = model.part("z_stage")
    z_visual = _build_z_stage_visual()
    _add_mesh_visual(z_stage, z_visual, "z_stage.obj", "light_panel")



    z_stage.inertial = Inertial.from_geometry(
        Box((0.11, 0.064, 0.13)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
    )

    model.articulation(
        "base_to_x_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.055, 0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.13, upper=0.13, effort=180.0, velocity=0.40),
    )
    model.articulation(
        "x_stage_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=z_stage,
        origin=Origin(xyz=(0.0, -0.044, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.10, effort=140.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_x_stage",
        "x_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "x_stage_to_z_stage",
        "z_stage",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.001, max_penetration=0.001)
    ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_gap("z_stage", "base", axis="z", max_gap=0.08, max_penetration=0.0)
    ctx.expect_aabb_overlap("z_stage", "base", axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance("z_stage", "base", axes="xy", max_dist=0.10)

    with ctx.pose(base_to_x_stage=-0.13, x_stage_to_z_stage=0.0):
        ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.001, max_penetration=0.001)
        ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_gap("z_stage", "base", axis="z", max_gap=0.08, max_penetration=0.0)
        ctx.expect_aabb_overlap("z_stage", "base", axes="xy", min_overlap=0.02)

    with ctx.pose(base_to_x_stage=0.13, x_stage_to_z_stage=0.10):
        ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.001, max_penetration=0.001)
        ctx.expect_aabb_gap("z_stage", "base", axis="z", max_gap=0.18, max_penetration=0.0)
        ctx.expect_aabb_overlap("z_stage", "base", axes="xy", min_overlap=0.02)
        ctx.expect_origin_distance("z_stage", "base", axes="xy", max_dist=0.18)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
