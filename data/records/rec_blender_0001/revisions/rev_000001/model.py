from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        try:
            return Material(name=name, color=rgba)
        except TypeError:
            try:
                return Material(name, rgba)
            except TypeError:
                return Material(name=name)


def _register_materials(model: ArticulatedObject, *materials: Material) -> None:
    if isinstance(getattr(model, "materials", None), list):
        model.materials.extend(materials)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, MESH_DIR / filename)


def _rounded_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    spout: float = 0.0,
) -> list[tuple[float, float, float]]:
    profile = []
    half_width = max(width * 0.5, 1e-6)
    half_depth = max(depth * 0.5, 1e-6)
    for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8):
        if spout > 0.0 and x > 0.0:
            front_weight = min(1.0, x / half_width)
            center_weight = max(0.0, 1.0 - abs(y) / (half_depth * 0.72))
            x += spout * front_weight * center_weight
        profile.append((x + x_shift, y + y_shift, z))
    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_blender", assets=ASSETS)

    painted_steel = _make_material("painted_steel", (0.73, 0.75, 0.78, 1.0))
    brushed_stainless = _make_material("brushed_stainless", (0.84, 0.85, 0.86, 1.0))
    black_plastic = _make_material("black_plastic", (0.12, 0.12, 0.13, 1.0))
    charcoal_rubber = _make_material("charcoal_rubber", (0.05, 0.05, 0.06, 1.0))
    clear_polycarbonate = _make_material("clear_polycarbonate", (0.88, 0.93, 0.97, 0.28))
    smoked_plastic = _make_material("smoked_plastic", (0.18, 0.20, 0.22, 0.45))
    satin_trim = _make_material("satin_trim", (0.56, 0.58, 0.60, 1.0))
    _register_materials(
        model,
        painted_steel,
        brushed_stainless,
        black_plastic,
        charcoal_rubber,
        clear_polycarbonate,
        smoked_plastic,
        satin_trim,
    )

    base = model.part("base")
    base_body = LoftGeometry(
        [
            _rounded_section(0.198, 0.186, 0.028, 0.000, x_shift=0.000),
            _rounded_section(0.192, 0.180, 0.026, 0.026, x_shift=-0.001),
            _rounded_section(0.166, 0.166, 0.024, 0.072, x_shift=-0.005),
            _rounded_section(0.138, 0.148, 0.020, 0.110, x_shift=-0.010),
        ],
        cap=True,
        closed=True,
    )
    base.visual(_save_mesh(base_body, "blender_base_body.obj"), material=painted_steel)
    base.visual(
        Box((0.022, 0.126, 0.074)),
        origin=Origin(xyz=(0.090, 0.000, 0.052)),
        material=black_plastic,
        name="control_panel",
    )
    base.visual(
        Box((0.086, 0.086, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.107)),
        material=black_plastic,
        name="drive_pad",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.117)),
        material=brushed_stainless,
        name="drive_coupler",
    )
    base.visual(
        Box((0.018, 0.132, 0.010)),
        origin=Origin(xyz=(0.082, 0.000, 0.090)),
        material=satin_trim,
        name="panel_trim",
    )
    for x in (-0.068, 0.068):
        for y in (-0.070, 0.070):
            base.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(xyz=(x, y, 0.004)),
                material=charcoal_rubber,
                name=f"foot_{'front' if x > 0 else 'rear'}_{'right' if y > 0 else 'left'}",
            )
    base.inertial = Inertial.from_geometry(
        Box((0.198, 0.186, 0.112)),
        mass=3.8,
        origin=Origin(xyz=(0.000, 0.000, 0.056)),
    )

    jar = model.part("jar")
    jar_body = LoftGeometry(
        [
            _rounded_section(0.112, 0.108, 0.018, 0.005, x_shift=-0.003),
            _rounded_section(0.126, 0.122, 0.019, 0.075, x_shift=-0.001, spout=0.003),
            _rounded_section(0.148, 0.144, 0.020, 0.165, x_shift=0.002, spout=0.010),
            _rounded_section(0.160, 0.154, 0.022, 0.228, x_shift=0.004, spout=0.016),
        ],
        cap=True,
        closed=True,
    )
    jar_handle = tube_from_spline_points(
        [
            (0.020, 0.060, 0.188),
            (0.010, 0.097, 0.170),
            (-0.004, 0.120, 0.115),
            (0.006, 0.116, 0.060),
            (0.022, 0.056, 0.038),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    jar.visual(_save_mesh(jar_body, "blender_pitcher_body.obj"), material=clear_polycarbonate)
    jar.visual(_save_mesh(jar_handle, "blender_pitcher_handle.obj"), material=black_plastic)
    jar.visual(
        Box((0.126, 0.120, 0.024)),
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=black_plastic,
        name="jar_collar",
    )
    jar.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.028)),
        material=brushed_stainless,
        name="blade_hub",
    )
    jar.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.000, 0.000, 0.038)),
        material=brushed_stainless,
        name="blade_post",
    )
    jar.visual(
        Box((0.064, 0.010, 0.002)),
        origin=Origin(xyz=(0.000, 0.000, 0.039), rpy=(0.000, 0.16, 0.50)),
        material=brushed_stainless,
        name="blade_a",
    )
    jar.visual(
        Box((0.052, 0.010, 0.002)),
        origin=Origin(xyz=(0.000, 0.000, 0.034), rpy=(0.000, -0.22, -0.55)),
        material=brushed_stainless,
        name="blade_b",
    )
    jar.visual(
        Box((0.056, 0.010, 0.002)),
        origin=Origin(xyz=(0.000, 0.000, 0.038), rpy=(0.000, 0.20, 2.20)),
        material=brushed_stainless,
        name="blade_c",
    )
    jar.visual(
        Box((0.044, 0.010, 0.002)),
        origin=Origin(xyz=(0.000, 0.000, 0.033), rpy=(0.000, -0.18, -2.10)),
        material=brushed_stainless,
        name="blade_d",
    )
    jar.inertial = Inertial.from_geometry(
        Box((0.170, 0.250, 0.235)),
        mass=1.25,
        origin=Origin(xyz=(0.000, 0.000, 0.118)),
    )

    lid = model.part("lid")
    lid_body = LoftGeometry(
        [
            _rounded_section(0.166, 0.160, 0.018, 0.000, x_shift=0.001, spout=0.003),
            _rounded_section(0.168, 0.162, 0.020, 0.010, x_shift=0.002, spout=0.004),
            _rounded_section(0.150, 0.144, 0.015, 0.018, x_shift=0.001, spout=0.002),
        ],
        cap=True,
        closed=True,
    )
    lid.visual(_save_mesh(lid_body, "blender_lid.obj"), material=charcoal_rubber)
    lid.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=smoked_plastic,
        name="measuring_cap",
    )
    lid.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.024)),
        material=black_plastic,
        name="cap_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.170, 0.164, 0.030)),
        mass=0.18,
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
    )

    switch_y_positions = {
        "low_switch": -0.040,
        "high_switch": 0.000,
        "pulse_switch": 0.040,
    }
    for part_name, y_pos in switch_y_positions.items():
        switch = model.part(part_name)
        switch.visual(
            Box((0.012, 0.024, 0.032)),
            origin=Origin(xyz=(0.006, 0.000, 0.016)),
            material=black_plastic,
            name="paddle",
        )
        switch.visual(
            Box((0.004, 0.018, 0.006)),
            origin=Origin(xyz=(0.010, 0.000, 0.027)),
            material=satin_trim,
            name="switch_cap",
        )
        switch.inertial = Inertial.from_geometry(
            Box((0.012, 0.024, 0.032)),
            mass=0.03,
            origin=Origin(xyz=(0.006, 0.000, 0.016)),
        )
        model.articulation(
            f"{part_name}_joint",
            ArticulationType.REVOLUTE,
            parent="base",
            child=part_name,
            origin=Origin(xyz=(0.101, y_pos, 0.030)),
            axis=(0.000, 1.000, 0.000),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.5,
                lower=0.000,
                upper=0.42,
            ),
        )

    model.articulation(
        "base_to_jar",
        ArticulationType.REVOLUTE,
        parent="base",
        child="jar",
        origin=Origin(xyz=(0.000, 0.000, 0.119)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.PRISMATIC,
        parent="jar",
        child="lid",
        origin=Origin(xyz=(0.000, 0.000, 0.228)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.15,
            lower=0.000,
            upper=0.036,
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
        max_pose_samples=256,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("jar", "base", axes="xy", min_overlap=0.105)
    ctx.expect_origin_distance("jar", "base", axes="xy", max_dist=0.012)
    ctx.expect_aabb_gap("jar", "base", axis="z", max_gap=0.003, max_penetration=1e-5)

    with ctx.pose(jar_to_lid=0.0):
        ctx.expect_aabb_overlap("lid", "jar", axes="xy", min_overlap=0.110)
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.090)
        ctx.expect_origin_distance("lid", "jar", axes="xy", max_dist=0.020)
        ctx.expect_aabb_gap("lid", "jar", axis="z", max_gap=0.004, max_penetration=0.0)

    with ctx.pose(base_to_jar=-0.35):
        ctx.expect_aabb_overlap("jar", "base", axes="xy", min_overlap=0.100)
        ctx.expect_origin_distance("jar", "base", axes="xy", max_dist=0.014)
        ctx.expect_aabb_gap("jar", "base", axis="z", max_gap=0.003, max_penetration=1e-5)
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.085)

    with ctx.pose(base_to_jar=0.35):
        ctx.expect_aabb_overlap("jar", "base", axes="xy", min_overlap=0.100)
        ctx.expect_origin_distance("jar", "base", axes="xy", max_dist=0.014)
        ctx.expect_aabb_gap("jar", "base", axis="z", max_gap=0.003, max_penetration=1e-5)
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.085)

    with ctx.pose(base_to_jar=0.35, jar_to_lid=0.036):
        ctx.expect_aabb_overlap("lid", "jar", axes="xy", min_overlap=0.110)
        ctx.expect_origin_distance("lid", "jar", axes="xy", max_dist=0.022)
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.085)

    ctx.expect_joint_motion_axis(
        "jar_to_lid",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.020,
    )
    ctx.expect_joint_motion_axis(
        "low_switch_joint",
        "low_switch",
        world_axis="x",
        direction="positive",
        min_delta=0.002,
    )
    ctx.expect_joint_motion_axis(
        "high_switch_joint",
        "high_switch",
        world_axis="x",
        direction="positive",
        min_delta=0.002,
    )
    ctx.expect_joint_motion_axis(
        "pulse_switch_joint",
        "pulse_switch",
        world_axis="x",
        direction="positive",
        min_delta=0.002,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
