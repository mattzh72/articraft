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
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    constructors = (
        lambda: Material(name=name, rgba=rgba),
        lambda: Material(name=name, color=rgba),
        lambda: Material(name=name, diffuse=rgba),
        lambda: Material(name=name, rgb=rgba[:3]),
        lambda: Material(name, rgba),
        lambda: Material(name, rgba[:3]),
        lambda: Material(name=name),
        lambda: Material(name),
    )
    for factory in constructors:
        try:
            material = factory()
            break
        except TypeError:
            continue
    else:
        material = Material(name=name)

    for attr in ("rgba", "color", "diffuse"):
        if hasattr(material, attr):
            try:
                setattr(material, attr, rgba)
                break
            except Exception:
                pass
    return material


def _register_material(model: ArticulatedObject, material: Material) -> None:
    try:
        model.materials.append(material)
    except Exception:
        pass


def _lathe_mesh(profile: list[tuple[float, float]], filename: str):
    return mesh_from_geometry(LatheGeometry(profile, segments=72), MESH_DIR / filename)


def build_object_model() -> ArticulatedObject:
    MESH_DIR.mkdir(parents=True, exist_ok=True)

    model = ArticulatedObject(name="dining_table_lazy_susan", assets=ASSETS)

    walnut = _make_material("walnut_veneer", (0.43, 0.29, 0.18, 1.0))
    espresso = _make_material("espresso_stain", (0.29, 0.20, 0.13, 1.0))
    steel = _make_material("brushed_steel", (0.69, 0.70, 0.73, 1.0))
    rubber = _make_material("black_rubber", (0.10, 0.10, 0.10, 1.0))
    for material in (walnut, espresso, steel, rubber):
        _register_material(model, material)

    tabletop_mesh = _lathe_mesh(
        [
            (0.0, -0.019),
            (0.18, -0.019),
            (0.42, -0.020),
            (0.54, -0.018),
            (0.582, -0.013),
            (0.598, -0.006),
            (0.600, 0.000),
            (0.594, 0.007),
            (0.570, 0.012),
            (0.0, 0.012),
        ],
        "tabletop.obj",
    )
    pedestal_mesh = _lathe_mesh(
        [
            (0.0, -0.320),
            (0.095, -0.320),
            (0.115, -0.300),
            (0.122, -0.250),
            (0.092, -0.205),
            (0.078, -0.120),
            (0.090, -0.030),
            (0.070, 0.080),
            (0.082, 0.175),
            (0.102, 0.255),
            (0.085, 0.320),
            (0.0, 0.320),
        ],
        "pedestal.obj",
    )
    tray_mesh = _lathe_mesh(
        [
            (0.0, 0.006),
            (0.07, 0.008),
            (0.18, 0.009),
            (0.23, 0.012),
            (0.252, 0.020),
            (0.268, 0.028),
            (0.270, 0.031),
            (0.250, 0.036),
            (0.0, 0.033),
        ],
        "lazy_susan_tray.obj",
    )

    base = model.part("table_base")
    base.visual(
        tabletop_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.748)),
        material=walnut,
        name="tabletop",
    )
    base.visual(
        Cylinder(radius=0.24, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.721)),
        material=espresso,
        name="mounting_plate",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.6725)),
        material=espresso,
        name="pedestal_collar",
    )
    for yaw in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        base.visual(
            Box((0.32, 0.045, 0.018)),
            origin=Origin(
                xyz=(0.16 * math.cos(yaw), 0.16 * math.sin(yaw), 0.714),
                rpy=(0.0, 0.0, yaw),
            ),
            material=espresso,
            name=f"support_beam_{int(round(yaw * 1000))}",
        )
    base.visual(
        pedestal_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.323)),
        material=espresso,
        name="turned_pedestal",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=espresso,
        name="pedestal_hub",
    )
    foot_specs = (
        ((0.21, 0.0, 0.024), 0.0, "foot_east"),
        ((-0.21, 0.0, 0.024), 0.0, "foot_west"),
        ((0.0, 0.21, 0.024), math.pi / 2.0, "foot_north"),
        ((0.0, -0.21, 0.024), math.pi / 2.0, "foot_south"),
    )
    for xyz, yaw, name in foot_specs:
        base.visual(
            Box((0.42, 0.095, 0.048)),
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
            material=espresso,
            name=name,
        )
    for xyz, name in (
        ((0.395, 0.0, 0.003), "pad_east"),
        ((-0.395, 0.0, 0.003), "pad_west"),
        ((0.0, 0.395, 0.003), "pad_north"),
        ((0.0, -0.395, 0.003), "pad_south"),
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=xyz),
            material=rubber,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.11, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.762)),
        material=steel,
        name="turntable_base_ring",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.60, length=0.76),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
    )

    tray = model.part("lazy_susan")
    tray.visual(
        Cylinder(radius=0.105, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="turntable_hub",
    )
    tray.visual(
        tray_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=walnut,
        name="serving_tray",
    )
    tray.visual(
        Cylinder(radius=0.20, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=espresso,
        name="tray_inlay",
    )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.27, length=0.036),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    model.articulation(
        "lazy_susan_spin",
        ArticulationType.CONTINUOUS,
        parent="table_base",
        child="lazy_susan",
        origin=Origin(xyz=(0.0, 0.0, 0.7628)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "table_base",
        "lazy_susan",
        reason="closely nested turntable bearing surfaces may conservatively register as overlapping",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    for angle in (0.0, math.pi / 2.0, math.pi, 1.5 * math.pi):
        with ctx.pose(lazy_susan_spin=angle):
            ctx.expect_origin_distance("lazy_susan", "table_base", axes="xy", max_dist=0.005)
            ctx.expect_aabb_overlap("lazy_susan", "table_base", axes="xy", min_overlap=0.50)
            ctx.expect_aabb_gap("lazy_susan", "table_base", axis="z", max_gap=0.001, max_penetration=0.012)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
