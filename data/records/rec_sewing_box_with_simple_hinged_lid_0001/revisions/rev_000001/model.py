from __future__ import annotations

import math

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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
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


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _rounded_shell_mesh(
    width: float,
    depth: float,
    height: float,
    wall: float,
    radius: float,
    filename: str,
):
    outer = rounded_rect_profile(width, depth, radius, corner_segments=10)
    inner = rounded_rect_profile(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        max(radius - wall, 0.001),
        corner_segments=10,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        height=height,
        cap=True,
        center=False,
        closed=True,
    )
    return mesh_from_geometry(geom, MESH_DIR / filename)


def _rounded_panel_mesh(
    width: float,
    depth: float,
    thickness: float,
    radius: float,
    filename: str,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=10),
        height=thickness,
        cap=True,
        center=False,
        closed=True,
    )
    return mesh_from_geometry(geom, MESH_DIR / filename)


def _handle_arch_mesh(
    width: float,
    y_pos: float,
    base_z: float,
    peak_z: float,
    radius: float,
    filename: str,
):
    geom = tube_from_spline_points(
        [
            (-0.5 * width, y_pos, base_z),
            (-0.22 * width, y_pos, peak_z),
            (0.22 * width, y_pos, peak_z),
            (0.5 * width, y_pos, base_z),
        ],
        radius=radius,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, MESH_DIR / filename)


def build_object_model() -> ArticulatedObject:
    MESH_DIR.mkdir(parents=True, exist_ok=True)

    model = ArticulatedObject(name="sewing_box", assets=ASSETS)

    painted_metal = _make_material("painted_metal", (0.41, 0.58, 0.55, 1.0))
    cream_plastic = _make_material("cream_plastic", (0.89, 0.86, 0.77, 1.0))
    brushed_steel = _make_material("brushed_steel", (0.73, 0.74, 0.76, 1.0))
    black_rubber = _make_material("black_rubber", (0.09, 0.09, 0.10, 1.0))
    tan_fabric = _make_material("tan_fabric", (0.76, 0.64, 0.51, 1.0))
    warm_beige = _make_material("warm_beige", (0.82, 0.75, 0.65, 1.0))
    model.materials.extend(
        [
            painted_metal,
            cream_plastic,
            brushed_steel,
            black_rubber,
            tan_fabric,
            warm_beige,
        ]
    )

    body_w = 0.300
    body_d = 0.190
    body_h = 0.118
    body_wall = 0.0035
    body_radius = 0.022
    bottom_t = 0.004

    lid_w = 0.308
    lid_d = 0.198
    lid_h = 0.040
    lid_wall = 0.003
    lid_radius = 0.024
    lid_top_t = 0.0035
    lid_bottom_z = -0.004
    lid_center_y = 0.5 * lid_d - 0.0015

    hinge_radius = 0.005
    hinge_y = -0.5 * body_d + 0.0015
    hinge_z = body_h + hinge_radius

    body_shell = _rounded_shell_mesh(
        body_w,
        body_d,
        body_h,
        body_wall,
        body_radius,
        "sewing_box_body_shell.obj",
    )
    lid_shell = _rounded_shell_mesh(
        lid_w,
        lid_d,
        lid_h,
        lid_wall,
        lid_radius,
        "sewing_box_lid_shell.obj",
    )
    lid_top = _rounded_panel_mesh(
        lid_w,
        lid_d,
        lid_top_t,
        lid_radius,
        "sewing_box_lid_top.obj",
    )
    handle_arch = _handle_arch_mesh(
        width=0.168,
        y_pos=lid_center_y - 0.002,
        base_z=0.039,
        peak_z=0.067,
        radius=0.0045,
        filename="sewing_box_handle.obj",
    )

    body = model.part("body")
    body.visual(body_shell, material=painted_metal)
    body.visual(
        Box((body_w - 0.034, body_d - 0.034, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * bottom_t)),
        material=painted_metal,
    )

    tray_floor_w = 0.258
    tray_floor_d = 0.148
    tray_floor_t = 0.003
    tray_wall_t = 0.0025
    tray_h = 0.018
    tray_floor_z = bottom_t + 0.5 * tray_floor_t + 0.002

    body.visual(
        Box((tray_floor_w, tray_floor_d, tray_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z)),
        material=cream_plastic,
    )
    wall_center_z = tray_floor_z + 0.5 * tray_floor_t + 0.5 * tray_h
    body.visual(
        Box((tray_floor_w, tray_wall_t, tray_h)),
        origin=Origin(xyz=(0.0, 0.5 * tray_floor_d - 0.5 * tray_wall_t, wall_center_z)),
        material=cream_plastic,
    )
    body.visual(
        Box((tray_floor_w, tray_wall_t, tray_h)),
        origin=Origin(xyz=(0.0, -0.5 * tray_floor_d + 0.5 * tray_wall_t, wall_center_z)),
        material=cream_plastic,
    )
    body.visual(
        Box((tray_wall_t, tray_floor_d - 2.0 * tray_wall_t, tray_h)),
        origin=Origin(xyz=(-0.5 * tray_floor_w + 0.5 * tray_wall_t, 0.0, wall_center_z)),
        material=cream_plastic,
    )
    body.visual(
        Box((tray_wall_t, tray_floor_d - 2.0 * tray_wall_t, tray_h)),
        origin=Origin(xyz=(0.5 * tray_floor_w - 0.5 * tray_wall_t, 0.0, wall_center_z)),
        material=cream_plastic,
    )
    body.visual(
        Box((tray_wall_t, tray_floor_d - 2.0 * tray_wall_t, tray_h - 0.003)),
        origin=Origin(xyz=(-0.041, 0.0, wall_center_z - 0.0015)),
        material=cream_plastic,
    )
    body.visual(
        Box((0.112, tray_wall_t, tray_h - 0.004)),
        origin=Origin(xyz=(0.058, 0.021, wall_center_z - 0.002)),
        material=cream_plastic,
    )
    body.visual(
        Box((0.072, 0.050, 0.004)),
        origin=Origin(
            xyz=(
                0.078,
                -0.036,
                tray_floor_z + 0.5 * tray_floor_t + 0.002,
            )
        ),
        material=warm_beige,
    )
    body.visual(
        Box((0.068, 0.046, 0.010)),
        origin=Origin(
            xyz=(
                0.078,
                -0.036,
                tray_floor_z + 0.5 * tray_floor_t + 0.009,
            )
        ),
        material=tan_fabric,
    )

    spool_center_z = tray_floor_z + 0.5 * tray_floor_t + 0.010
    for x_pos, y_pos in [(-0.095, 0.040), (-0.077, 0.040)]:
        body.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(x_pos, y_pos, spool_center_z)),
            material=warm_beige,
        )

    for x_pos in (-0.105, 0.105):
        for y_pos in (-0.062, 0.062):
            body.visual(
                Box((0.030, 0.018, 0.004)),
                origin=Origin(xyz=(x_pos, y_pos, 0.002)),
                material=black_rubber,
            )

    body.visual(
        Box((0.030, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.5 * body_d + 0.003, body_h - 0.014)),
        material=brushed_steel,
    )

    for x_pos, length in [(-0.104, 0.048), (0.040, 0.076)]:
        body.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(x_pos, hinge_y, hinge_z),
                rpy=(0.0, 0.5 * math.pi, 0.0),
            ),
            material=brushed_steel,
        )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=1.20,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * body_h)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_shell,
        origin=Origin(xyz=(0.0, lid_center_y, lid_bottom_z)),
        material=painted_metal,
    )
    lid.visual(
        lid_top,
        origin=Origin(xyz=(0.0, lid_center_y, lid_bottom_z + lid_h - lid_top_t)),
        material=painted_metal,
    )
    lid.visual(
        Box((0.224, 0.100, 0.004)),
        origin=Origin(xyz=(0.0, lid_center_y - 0.006, lid_bottom_z + lid_h - lid_top_t - 0.002)),
        material=tan_fabric,
    )
    lid.visual(
        Box((0.028, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, lid_center_y + 0.5 * lid_d + 0.001, 0.011)),
        material=brushed_steel,
    )
    lid.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, lid_center_y + 0.5 * lid_d + 0.002, 0.003)),
        material=black_rubber,
    )

    bracket_y = lid_center_y - 0.002
    for x_pos in (-0.084, 0.084):
        lid.visual(
            Box((0.022, 0.014, 0.010)),
            origin=Origin(xyz=(x_pos, bracket_y, 0.041)),
            material=brushed_steel,
        )
        lid.visual(
            Box((0.010, 0.010, 0.009)),
            origin=Origin(xyz=(x_pos, bracket_y, 0.048)),
            material=black_rubber,
        )

    lid.visual(handle_arch, material=black_rubber)

    for x_pos, length in [(-0.040, 0.076), (0.104, 0.048)]:
        lid.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, 0.5 * math.pi, 0.0),
            ),
            material=brushed_steel,
        )

    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_h)),
        mass=0.54,
        origin=Origin(xyz=(0.0, lid_center_y, lid_bottom_z + 0.5 * lid_h)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
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
        "body",
        "lid",
        reason=(
            "The close-fitting cap and interleaved rear hinge knuckles can produce "
            "conservative collision-hull overlap around the seam."
        ),
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.18)
    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.06,
    )

    with ctx.pose(lid_hinge=0.20):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.16)

    with ctx.pose(lid_hinge=0.60):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.10)

    with ctx.pose(lid_hinge=1.25):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.04)

    with ctx.pose(lid_hinge=1.35):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.02)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
