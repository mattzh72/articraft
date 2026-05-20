from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]):
    for kwargs in (
        {"name": name, "color": rgba},
        {"name": name, "rgba": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    for args in ((name, rgba), (name,)):
        try:
            return Material(*args)
        except TypeError:
            pass
    return None


def _register_materials(model: ArticulatedObject, materials: dict[str, object]) -> None:
    if not hasattr(model, "materials"):
        return
    existing = {getattr(material, "name", None) for material in model.materials}
    for material in materials.values():
        material_name = getattr(material, "name", None)
        if material is not None and material_name not in existing:
            model.materials.append(material)
            existing.add(material_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="missile_launcher_assembly", assets=ASSETS)

    materials = {
        "olive": _make_material("olive_painted_metal", (0.34, 0.38, 0.27, 1.0)),
        "steel": _make_material("gunmetal_steel", (0.25, 0.27, 0.29, 1.0)),
        "black": _make_material("matte_black", (0.10, 0.10, 0.11, 1.0)),
        "rubber": _make_material("rubber_pad", (0.06, 0.06, 0.06, 1.0)),
    }
    _register_materials(model, materials)

    base = model.part("support_base")
    base.visual(
        Box((0.86, 0.60, 0.06)),
        origin=Origin(xyz=(0.02, 0.0, 0.03)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.84, 0.08, 0.08)),
        origin=Origin(xyz=(0.02, 0.22, 0.07)),
        material=materials["steel"],
    )
    base.visual(
        Box((0.84, 0.08, 0.08)),
        origin=Origin(xyz=(0.02, -0.22, 0.07)),
        material=materials["steel"],
    )
    base.visual(
        Box((0.14, 0.48, 0.04)),
        origin=Origin(xyz=(0.34, 0.0, 0.02)),
        material=materials["steel"],
    )
    base.visual(
        Box((0.16, 0.46, 0.04)),
        origin=Origin(xyz=(-0.32, 0.0, 0.02)),
        material=materials["steel"],
    )
    for pad_x in (-0.28, 0.30):
        for pad_y in (-0.18, 0.18):
            base.visual(
                Box((0.10, 0.12, 0.015)),
                origin=Origin(xyz=(pad_x, pad_y, 0.0075)),
                material=materials["rubber"],
            )
    base.visual(
        Cylinder(radius=0.12, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=materials["steel"],
    )
    base.visual(
        Cylinder(radius=0.21, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=materials["steel"],
    )
    base.visual(
        Box((0.32, 0.28, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.22, 0.24, 0.11)),
        origin=Origin(xyz=(-0.16, 0.0, 0.115)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.10, 0.18, 0.06)),
        origin=Origin(xyz=(0.18, 0.0, 0.11)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.24, 0.34, 0.04)),
        origin=Origin(xyz=(0.07, 0.0, 0.07)),
        material=materials["steel"],
    )
    base.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.05, 0.15, 0.13)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.05, -0.15, 0.13)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.14, 0.06, 0.28)),
        origin=Origin(xyz=(-0.02, 0.19, 0.49)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.14, 0.06, 0.28)),
        origin=Origin(xyz=(-0.02, -0.19, 0.49)),
        material=materials["olive"],
    )
    base.visual(
        Cylinder(radius=0.055, length=0.40),
        origin=Origin(xyz=(-0.02, 0.0, 0.49), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
    )
    base.visual(
        Cylinder(radius=0.065, length=0.03),
        origin=Origin(xyz=(-0.02, 0.235, 0.49), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
    )
    base.visual(
        Cylinder(radius=0.065, length=0.03),
        origin=Origin(xyz=(-0.02, -0.235, 0.49), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
    )
    base.visual(
        Box((0.26, 0.05, 0.05)),
        origin=Origin(xyz=(-0.08, 0.13, 0.37), rpy=(0.0, 0.92, 0.0)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.26, 0.05, 0.05)),
        origin=Origin(xyz=(-0.08, -0.13, 0.37), rpy=(0.0, 0.92, 0.0)),
        material=materials["olive"],
    )
    base.visual(
        Box((0.14, 0.18, 0.16)),
        origin=Origin(xyz=(-0.09, 0.0, 0.32)),
        material=materials["olive"],
    )
    base.inertial = Inertial.from_geometry(
        Box((0.86, 0.60, 0.66)),
        mass=240.0,
        origin=Origin(xyz=(0.02, 0.0, 0.33)),
    )

    launcher = model.part("launcher_frame")
    launcher.visual(
        Cylinder(radius=0.045, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
    )
    launcher.visual(
        Box((0.10, 0.28, 0.16)),
        origin=Origin(xyz=(0.02, 0.0, -0.02)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.08, 0.44, 0.52)),
        origin=Origin(xyz=(0.06, 0.0, 0.05)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.16, 0.03, 0.28)),
        origin=Origin(xyz=(0.04, 0.18, 0.05)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.16, 0.03, 0.28)),
        origin=Origin(xyz=(0.04, -0.18, 0.05)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.70, 0.05, 0.05)),
        origin=Origin(xyz=(0.40, 0.17, 0.31)),
        material=materials["steel"],
    )
    launcher.visual(
        Box((0.70, 0.05, 0.05)),
        origin=Origin(xyz=(0.40, -0.17, 0.31)),
        material=materials["steel"],
    )
    launcher.visual(
        Box((0.68, 0.04, 0.04)),
        origin=Origin(xyz=(0.40, 0.17, 0.05)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.68, 0.04, 0.04)),
        origin=Origin(xyz=(0.40, -0.17, 0.05)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.62, 0.34, 0.04)),
        origin=Origin(xyz=(0.46, 0.0, 0.35)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.18, 0.12, 0.08)),
        origin=Origin(xyz=(0.20, 0.0, 0.39)),
        material=materials["olive"],
    )
    launcher.visual(
        Cylinder(radius=0.03, length=0.05),
        origin=Origin(xyz=(0.285, 0.0, 0.39), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["black"],
    )
    launcher.visual(
        Cylinder(radius=0.012, length=0.58),
        origin=Origin(xyz=(0.46, 0.0, 0.37), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["steel"],
    )
    launcher.visual(
        Box((0.04, 0.04, 0.52)),
        origin=Origin(xyz=(0.81, 0.17, 0.05)),
        material=materials["olive"],
    )
    launcher.visual(
        Box((0.04, 0.04, 0.52)),
        origin=Origin(xyz=(0.81, -0.17, 0.05)),
        material=materials["olive"],
    )
    for bar_z in (-0.13, 0.05, 0.23):
        launcher.visual(
            Box((0.04, 0.34, 0.04)),
            origin=Origin(xyz=(0.81, 0.0, bar_z)),
            material=materials["olive"],
        )
        launcher.visual(
            Box((0.58, 0.24, 0.03)),
            origin=Origin(xyz=(0.47, 0.0, bar_z)),
            material=materials["steel"],
        )

    tube_y_positions = (-0.11, 0.11)
    tube_z_positions = (-0.13, 0.05, 0.23)
    for tube_y in tube_y_positions:
        for tube_z in tube_z_positions:
            launcher.visual(
                Cylinder(radius=0.075, length=0.78),
                origin=Origin(
                    xyz=(0.42, tube_y, tube_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=materials["olive"],
            )
            launcher.visual(
                Cylinder(radius=0.079, length=0.018),
                origin=Origin(
                    xyz=(0.80, tube_y, tube_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=materials["steel"],
            )
            launcher.visual(
                Cylinder(radius=0.063, length=0.08),
                origin=Origin(
                    xyz=(0.77, tube_y, tube_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=materials["black"],
            )

    launcher.inertial = Inertial.from_geometry(
        Box((0.92, 0.46, 0.58)),
        mass=180.0,
        origin=Origin(xyz=(0.38, 0.0, 0.10)),
    )

    model.articulation(
        "launcher_elevation",
        ArticulationType.REVOLUTE,
        parent="support_base",
        child="launcher_frame",
        origin=Origin(xyz=(-0.02, 0.0, 0.49)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.6,
            lower=-0.15,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_joint_motion_axis(
        "launcher_elevation",
        "launcher_frame",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(launcher_elevation=-0.15):
        ctx.expect_aabb_overlap("launcher_frame", "support_base", axes="xy", min_overlap=0.10)
        ctx.expect_origin_distance("launcher_frame", "support_base", axes="xy", max_dist=0.22)

    with ctx.pose(launcher_elevation=0.25):
        ctx.expect_aabb_overlap("launcher_frame", "support_base", axes="xy", min_overlap=0.10)
        ctx.expect_origin_distance("launcher_frame", "support_base", axes="xy", max_dist=0.20)

    with ctx.pose(launcher_elevation=0.95):
        ctx.expect_aabb_overlap("launcher_frame", "support_base", axes="xy", min_overlap=0.04)
        ctx.expect_origin_distance("launcher_frame", "support_base", axes="xy", max_dist=0.30)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
