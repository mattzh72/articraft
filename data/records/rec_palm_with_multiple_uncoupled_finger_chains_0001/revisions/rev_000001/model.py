from __future__ import annotations

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
import cadquery as cq

PALM_MAIN = (0.102, 0.056, 0.038)
PALM_RAIL = (0.018, 0.046, 0.014)
PALM_RAIL_X = (-0.024, 0.0, 0.024)
PALM_TOP_Z = PALM_MAIN[2] / 2.0 + PALM_RAIL[2]
THUMB_SADDLE = (0.020, 0.040, 0.022)
THUMB_SADDLE_CENTER = (
    -(PALM_MAIN[0] / 2.0 + THUMB_SADDLE[0] * 0.35),
    -0.004,
    0.002,
)

FINGER_SPECS = (
    {
        "name": "index",
        "mount_xyz": (-0.024, 0.012, PALM_TOP_Z),
        "mount_rpy": (0.0, 0.0, 0.0),
        "axis": (1.0, 0.0, 0.0),
        "prox_len": 0.047,
        "dist_len": 0.034,
        "width": 0.016,
        "depth": 0.018,
        "root_upper": 1.20,
        "mid_upper": 1.32,
        "prox_mass": 0.090,
        "dist_mass": 0.055,
    },
    {
        "name": "middle",
        "mount_xyz": (0.0, 0.0, PALM_TOP_Z),
        "mount_rpy": (0.0, 0.0, 0.0),
        "axis": (1.0, 0.0, 0.0),
        "prox_len": 0.053,
        "dist_len": 0.039,
        "width": 0.017,
        "depth": 0.019,
        "root_upper": 1.22,
        "mid_upper": 1.35,
        "prox_mass": 0.098,
        "dist_mass": 0.060,
    },
    {
        "name": "ring",
        "mount_xyz": (0.024, -0.012, PALM_TOP_Z),
        "mount_rpy": (0.0, 0.0, 0.0),
        "axis": (1.0, 0.0, 0.0),
        "prox_len": 0.045,
        "dist_len": 0.032,
        "width": 0.015,
        "depth": 0.017,
        "root_upper": 1.18,
        "mid_upper": 1.28,
        "prox_mass": 0.082,
        "dist_mass": 0.050,
    },
    {
        "name": "thumb",
        "mount_xyz": (
            THUMB_SADDLE_CENTER[0] - THUMB_SADDLE[0] / 2.0,
            -0.004,
            0.002,
        ),
        "mount_rpy": (0.0, -0.95, 0.0),
        "axis": (0.0, 1.0, 0.0),
        "prox_len": 0.038,
        "dist_len": 0.029,
        "width": 0.018,
        "depth": 0.021,
        "root_upper": 1.05,
        "mid_upper": 1.10,
        "prox_mass": 0.078,
        "dist_mass": 0.047,
    },
)


def _material_name(spec_name: str) -> str:
    return "thumb_polymer" if spec_name == "thumb" else "finger_polymer"


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_palm_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(*PALM_MAIN)
    body = body.edges("|Z").fillet(0.005)
    for rail_x in PALM_RAIL_X:
        rail = (
            cq.Workplane("XY")
            .box(*PALM_RAIL)
            .translate((rail_x, 0.0, PALM_MAIN[2] / 2.0 + PALM_RAIL[2] / 2.0))
        )
        body = body.union(rail)
    thumb_saddle = cq.Workplane("XY").box(*THUMB_SADDLE).translate(THUMB_SADDLE_CENTER)
    return body.union(thumb_saddle)


def _make_phalanx_shape(length: float, width: float, depth: float) -> cq.Workplane:
    beam = cq.Workplane("XY").box(width, depth, length, centered=(True, True, False))
    beam = beam.edges("|Z").fillet(min(width, depth) * 0.16)
    knuckle_radius = depth * 0.34
    knuckle_barrel = (
        cq.Workplane("YZ")
        .cylinder(width * 0.78, knuckle_radius)
        .translate((0.0, 0.0, knuckle_radius))
    )
    return beam.union(knuckle_barrel)


def _configure_link_physics(
    part,
    *,
    width: float,
    depth: float,
    length: float,
    mass: float,
) -> None:
    collision_size = (
        width * 0.82,
        depth * 0.78,
        max(length - 0.006, length * 0.85),
    )
    collision_origin = Origin(
        xyz=(0.0, 0.0, collision_size[2] / 2.0),
    )

    part.inertial = Inertial.from_geometry(
        Box(collision_size),
        mass=mass,
        origin=collision_origin,
    )


def _build_finger_chain(model: ArticulatedObject, palm, spec: dict[str, object]) -> None:
    name = str(spec["name"])
    width = float(spec["width"])
    depth = float(spec["depth"])
    prox_len = float(spec["prox_len"])
    dist_len = float(spec["dist_len"])

    proximal = model.part(f"{name}_prox")
    _add_visual_mesh(
        proximal,
        _make_phalanx_shape(prox_len, width, depth),
        f"{name}_prox.obj",
        _material_name(name),
    )
    _configure_link_physics(
        proximal,
        width=width,
        depth=depth,
        length=prox_len,
        mass=float(spec["prox_mass"]),
    )

    distal = model.part(f"{name}_dist")
    _add_visual_mesh(
        distal,
        _make_phalanx_shape(dist_len, width * 0.92, depth * 0.92),
        f"{name}_dist.obj",
        _material_name(name),
    )
    _configure_link_physics(
        distal,
        width=width * 0.92,
        depth=depth * 0.92,
        length=dist_len,
        mass=float(spec["dist_mass"]),
    )

    model.articulation(
        f"palm_to_{name}_prox",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=proximal,
        origin=Origin(
            xyz=tuple(spec["mount_xyz"]),
            rpy=tuple(spec["mount_rpy"]),
        ),
        axis=tuple(spec["axis"]),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=float(spec["root_upper"]),
            effort=8.0,
            velocity=2.8,
        ),
    )
    model.articulation(
        f"{name}_prox_to_{name}_dist",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        origin=Origin(xyz=(0.0, 0.0, prox_len)),
        axis=tuple(spec["axis"]),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=float(spec["mid_upper"]),
            effort=6.0,
            velocity=3.4,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_palm_block", assets=ASSETS)
    model.material("anodized_black", rgba=(0.16, 0.17, 0.20, 1.0))
    model.material("finger_polymer", rgba=(0.42, 0.44, 0.48, 1.0))
    model.material("thumb_polymer", rgba=(0.33, 0.35, 0.39, 1.0))

    palm = model.part("palm")
    _add_visual_mesh(palm, _make_palm_shape(), "palm.obj", "anodized_black")

    for rail_x in PALM_RAIL_X:
        pass

    palm.inertial = Inertial.from_geometry(Box(PALM_MAIN), mass=0.82)

    for spec in FINGER_SPECS:
        _build_finger_chain(model, palm, spec)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_gap("index_prox", "palm", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("middle_prox", "palm", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("ring_prox", "palm", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("index_dist", "palm", axis="z", min_gap=0.025)
    ctx.expect_origin_gap("middle_dist", "palm", axis="z", min_gap=0.030)
    ctx.expect_origin_gap("ring_dist", "palm", axis="z", min_gap=0.022)

    ctx.expect_aabb_gap("index_prox", "palm", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("middle_prox", "palm", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("ring_prox", "palm", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("index_dist", "index_prox", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("middle_dist", "middle_prox", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("ring_dist", "ring_prox", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_aabb_gap("thumb_dist", "thumb_prox", axis="z", max_gap=0.010, max_penetration=0.012)

    ctx.expect_aabb_overlap("index_prox", "palm", axes="xy", min_overlap=0.008)
    ctx.expect_aabb_overlap("middle_prox", "palm", axes="xy", min_overlap=0.009)
    ctx.expect_aabb_overlap("ring_prox", "palm", axes="xy", min_overlap=0.008)
    ctx.expect_aabb_overlap("index_dist", "index_prox", axes="xy", min_overlap=0.008)
    ctx.expect_aabb_overlap("middle_dist", "middle_prox", axes="xy", min_overlap=0.009)
    ctx.expect_aabb_overlap("ring_dist", "ring_prox", axes="xy", min_overlap=0.008)

    ctx.expect_origin_distance("index_dist", "index_prox", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("middle_dist", "middle_prox", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("ring_dist", "ring_prox", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("thumb_dist", "thumb_prox", axes="xy", max_dist=0.05)

    ctx.expect_joint_motion_axis(
        "palm_to_index_prox",
        "index_prox",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "index_prox_to_index_dist",
        "index_dist",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "palm_to_middle_prox",
        "middle_prox",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "middle_prox_to_middle_dist",
        "middle_dist",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "palm_to_ring_prox",
        "ring_prox",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "ring_prox_to_ring_dist",
        "ring_dist",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "palm_to_thumb_prox",
        "thumb_prox",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "thumb_prox_to_thumb_dist",
        "thumb_dist",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
