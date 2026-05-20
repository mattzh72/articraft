from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in ({"name": name, "rgba": rgba}, {"name": name, "color": rgba}):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    return Material(name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cctv_mast", assets=ASSETS)
    model.meta["category"] = "surveillance"

    galvanized_steel = _make_material("galvanized_steel", (0.70, 0.72, 0.74, 1.0))
    painted_white = _make_material("painted_white", (0.90, 0.91, 0.92, 1.0))
    black_polymer = _make_material("black_polymer", (0.12, 0.13, 0.14, 1.0))
    smoked_glass = _make_material("smoked_glass", (0.18, 0.24, 0.28, 0.42))
    dark_rubber = _make_material("dark_rubber", (0.08, 0.08, 0.08, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.13, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=galvanized_steel,
    )
    mast.visual(
        Cylinder(radius=0.085, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=galvanized_steel,
    )
    mast.visual(
        Cylinder(radius=0.065, length=2.82),
        origin=Origin(xyz=(0.0, 0.0, 1.435)),
        material=galvanized_steel,
    )
    mast.visual(
        Box((0.10, 0.012, 0.28)),
        origin=Origin(xyz=(0.0, 0.071, 0.95)),
        material=painted_white,
    )
    mast.visual(
        Box((0.012, 0.02, 0.06)),
        origin=Origin(xyz=(0.03, 0.083, 0.95)),
        material=black_polymer,
    )
    mast.visual(
        Box((0.20, 0.12, 0.26)),
        origin=Origin(xyz=(0.0, -0.125, 2.18)),
        material=painted_white,
    )
    mast.visual(
        Box((0.16, 0.004, 0.22)),
        origin=Origin(xyz=(0.0, -0.187, 2.18)),
        material=painted_white,
    )
    mast.visual(
        Box((0.014, 0.02, 0.05)),
        origin=Origin(xyz=(0.055, -0.197, 2.18)),
        material=black_polymer,
    )
    mast.visual(
        Box((0.22, 0.14, 0.014)),
        origin=Origin(xyz=(0.0, -0.125, 2.317)),
        material=painted_white,
    )
    mast.visual(
        Cylinder(radius=0.082, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 2.9225)),
        material=galvanized_steel,
    )
    mast.visual(
        Cylinder(radius=0.10, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 2.98)),
        material=black_polymer,
    )

    conduit = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -0.14, 2.30),
                (0.0, -0.14, 2.46),
                (0.0, -0.10, 2.67),
                (0.0, -0.06, 2.82),
                (0.0, -0.03, 2.94),
            ],
            radius=0.009,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        MESH_DIR / "mast_conduit.obj",
    )
    mast.visual(conduit, material=black_polymer)
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=3.02),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 1.51)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.115, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=black_polymer,
    )
    pan_head.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=painted_white,
    )
    pan_head.visual(
        Box((0.12, 0.028, 0.36)),
        origin=Origin(xyz=(-0.02, 0.102, 0.34)),
        material=painted_white,
    )
    pan_head.visual(
        Box((0.12, 0.028, 0.36)),
        origin=Origin(xyz=(-0.02, -0.102, 0.34)),
        material=painted_white,
    )
    pan_head.visual(
        Box((0.04, 0.176, 0.28)),
        origin=Origin(xyz=(-0.09, 0.0, 0.34)),
        material=painted_white,
    )
    pan_head.visual(
        Box((0.08, 0.176, 0.04)),
        origin=Origin(xyz=(-0.07, 0.0, 0.50)),
        material=painted_white,
    )
    pan_head.visual(
        Cylinder(radius=0.03, length=0.082),
        origin=Origin(xyz=(0.0, 0.049, 0.32), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
    )
    pan_head.visual(
        Cylinder(radius=0.03, length=0.082),
        origin=Origin(xyz=(0.0, -0.049, 0.32), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.25, 0.25, 0.54)),
        mass=9.5,
        origin=Origin(xyz=(-0.01, 0.0, 0.27)),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Box((0.27, 0.16, 0.15)),
        origin=Origin(xyz=(0.125, 0.0, -0.005)),
        material=painted_white,
    )
    camera_head.visual(
        Box((0.05, 0.12, 0.11)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=painted_white,
    )
    camera_head.visual(
        Box((0.24, 0.18, 0.02)),
        origin=Origin(xyz=(0.17, 0.0, 0.08)),
        material=black_polymer,
    )
    camera_head.visual(
        Box((0.12, 0.11, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, -0.102)),
        material=black_polymer,
    )
    camera_head.visual(
        Cylinder(radius=0.072, length=0.07),
        origin=Origin(xyz=(0.295, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_white,
    )
    camera_head.visual(
        Cylinder(radius=0.050, length=0.03),
        origin=Origin(xyz=(0.338, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_polymer,
    )
    camera_head.visual(
        Cylinder(radius=0.035, length=0.008),
        origin=Origin(xyz=(0.356, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_glass,
    )
    camera_head.visual(
        Cylinder(radius=0.023, length=0.036),
        origin=Origin(xyz=(0.0, 0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
    )
    camera_head.visual(
        Cylinder(radius=0.023, length=0.036),
        origin=Origin(xyz=(0.0, -0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.42, 0.20, 0.22)),
        mass=6.8,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
    )

    model.articulation(
        "camera_pan",
        ArticulationType.REVOLUTE,
        parent="mast",
        child="pan_head",
        origin=Origin(xyz=(0.0, 0.0, 3.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent="pan_head",
        child="camera_head",
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.7,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "mast",
        "pan_head",
        reason="The rotary bearing skirt seats tightly on the mast cap as a weather-sealed interface.",
    )
    ctx.allow_overlap(
        "pan_head",
        "camera_head",
        reason="The tilt trunnions nest inside the yoke bearing bosses.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("pan_head", "mast", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("pan_head", "mast", axes="xy", min_overlap=0.13)
    ctx.expect_aabb_gap("pan_head", "mast", axis="z", max_gap=0.012, max_penetration=0.003)

    ctx.expect_aabb_overlap("camera_head", "pan_head", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("camera_head", "mast", axes="xy", min_overlap=0.05)
    ctx.expect_joint_motion_axis(
        "camera_tilt",
        "camera_head",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(camera_tilt=-0.6):
        ctx.expect_aabb_overlap("camera_head", "pan_head", axes="xy", min_overlap=0.07)
        ctx.expect_aabb_overlap("camera_head", "mast", axes="xy", min_overlap=0.05)

    with ctx.pose(camera_tilt=0.4):
        ctx.expect_aabb_overlap("camera_head", "pan_head", axes="xy", min_overlap=0.06)
        ctx.expect_aabb_overlap("camera_head", "mast", axes="xy", min_overlap=0.04)

    with ctx.pose(camera_pan=math.pi / 2.0):
        ctx.expect_aabb_overlap("pan_head", "mast", axes="xy", min_overlap=0.13)
        ctx.expect_aabb_overlap("camera_head", "mast", axes="xy", min_overlap=0.05)

    with ctx.pose(camera_pan=-math.pi / 2.0, camera_tilt=-0.35):
        ctx.expect_aabb_overlap("camera_head", "pan_head", axes="xy", min_overlap=0.06)
        ctx.expect_aabb_overlap("camera_head", "mast", axes="xy", min_overlap=0.05)

    with ctx.pose(camera_pan=2.5, camera_tilt=0.35):
        ctx.expect_aabb_overlap("camera_head", "mast", axes="xy", min_overlap=0.04)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
