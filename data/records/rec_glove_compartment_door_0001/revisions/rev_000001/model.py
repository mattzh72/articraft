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
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _rounded_panel_mesh(
    filename: str,
    *,
    width: float,
    height: float,
    depth: float,
    radius: float,
):
    safe_radius = min(radius, 0.5 * min(width, height) - 1e-6)
    profile = rounded_rect_profile(width, height, safe_radius, corner_segments=10)
    geom = ExtrudeGeometry(profile, depth, cap=True, center=True, closed=True)
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _rounded_ring_mesh(
    filename: str,
    *,
    outer_width: float,
    outer_height: float,
    outer_radius: float,
    inner_width: float,
    inner_height: float,
    inner_radius: float,
    depth: float,
):
    safe_outer_radius = min(outer_radius, 0.5 * min(outer_width, outer_height) - 1e-6)
    safe_inner_radius = min(inner_radius, 0.5 * min(inner_width, inner_height) - 1e-6)
    outer = rounded_rect_profile(
        outer_width,
        outer_height,
        safe_outer_radius,
        corner_segments=10,
    )
    inner = rounded_rect_profile(
        inner_width,
        inner_height,
        safe_inner_radius,
        corner_segments=10,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glove_compartment_assembly", assets=ASSETS)

    dashboard_plastic = Material("dashboard_plastic", (0.15, 0.16, 0.17, 1.0))
    soft_touch = Material("soft_touch_trim", (0.19, 0.20, 0.21, 1.0))
    satin_trim = Material("satin_trim", (0.52, 0.53, 0.55, 1.0))
    dark_rubber = Material("dark_rubber", (0.06, 0.06, 0.07, 1.0))
    liner_felt = Material("liner_felt", (0.10, 0.10, 0.11, 1.0))
    model.materials.extend([dashboard_plastic, soft_touch, satin_trim, dark_rubber, liner_felt])

    frame_outer_w = 0.402
    frame_outer_h = 0.210
    opening_w = 0.332
    opening_h = 0.136
    shell_depth = 0.170

    door_w = 0.322
    door_h = 0.132
    door_t = 0.016

    bezel_mesh = _rounded_ring_mesh(
        "dashboard_bezel.obj",
        outer_width=frame_outer_w,
        outer_height=frame_outer_h,
        outer_radius=0.028,
        inner_width=opening_w,
        inner_height=opening_h,
        inner_radius=0.016,
        depth=0.010,
    )
    trim_mesh = _rounded_ring_mesh(
        "bezel_trim.obj",
        outer_width=0.350,
        outer_height=0.152,
        outer_radius=0.020,
        inner_width=opening_w,
        inner_height=opening_h,
        inner_radius=0.016,
        depth=0.0025,
    )

    lower_panel_mesh = _rounded_panel_mesh(
        "door_lower_panel.obj",
        width=door_w,
        height=0.096,
        depth=0.010,
        radius=0.016,
    )
    upper_bridge_mesh = _rounded_panel_mesh(
        "door_upper_bridge.obj",
        width=door_w,
        height=0.018,
        depth=0.010,
        radius=0.012,
    )
    shoulder_panel_mesh = _rounded_panel_mesh(
        "door_shoulder_panel.obj",
        width=0.111,
        height=0.040,
        depth=0.010,
        radius=0.012,
    )
    appliqué_mesh = _rounded_panel_mesh(
        "door_applique.obj",
        width=0.264,
        height=0.064,
        depth=0.003,
        radius=0.012,
    )

    frame_mount_xyz = (-0.166, 0.0, -0.005)
    dashboard_frame = model.part("dashboard_frame")
    dashboard_frame.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.166, 0.0, 0.005)),
        material=dashboard_plastic,
    )
    dashboard_frame.visual(
        trim_mesh,
        origin=Origin(xyz=(0.166, 0.0, 0.0095)),
        material=satin_trim,
    )
    dashboard_frame.inertial = Inertial.from_geometry(
        Box((frame_outer_w, frame_outer_h, 0.012)),
        mass=0.55,
        origin=Origin(xyz=(0.166, 0.0, 0.005)),
    )

    frame = model.part("compartment_bin")

    frame.visual(
        Box((0.011, opening_h, shell_depth)),
        origin=Origin(xyz=(-0.1605, 0.0, -0.090)),
        material=soft_touch,
    )
    frame.visual(
        Box((0.011, opening_h, shell_depth)),
        origin=Origin(xyz=(0.1605, 0.0, -0.090)),
        material=soft_touch,
    )
    frame.visual(
        Box((opening_w, 0.011, shell_depth)),
        origin=Origin(xyz=(0.0, 0.074, -0.090)),
        material=soft_touch,
    )
    frame.visual(
        Box((opening_w, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, -0.072, -0.035)),
        material=soft_touch,
    )
    frame.visual(
        Box((0.300, 0.010, 0.112)),
        origin=Origin(xyz=(0.0, -0.058, -0.095)),
        material=dashboard_plastic,
    )
    frame.visual(
        Box((0.304, 0.114, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, -0.170)),
        material=dashboard_plastic,
    )
    frame.visual(
        Box((0.276, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.066, -0.012)),
        material=dashboard_plastic,
    )
    frame.visual(
        Box((0.026, 0.024, 0.022)),
        origin=Origin(xyz=(-0.136, -0.058, -0.014)),
        material=dashboard_plastic,
    )
    frame.visual(
        Box((0.026, 0.024, 0.022)),
        origin=Origin(xyz=(0.136, -0.058, -0.014)),
        material=dashboard_plastic,
    )
    frame.visual(
        Box((0.286, 0.002, 0.108)),
        origin=Origin(xyz=(0.0, -0.061, -0.094)),
        material=liner_felt,
    )
    frame.visual(
        Box((0.012, 0.014, 0.006)),
        origin=Origin(xyz=(-0.142, 0.062, -0.002)),
        material=dark_rubber,
    )
    frame.visual(
        Box((0.012, 0.014, 0.006)),
        origin=Origin(xyz=(0.142, 0.062, -0.002)),
        material=dark_rubber,
    )
    frame.inertial = Inertial.from_geometry(
        Box((frame_outer_w, frame_outer_h, 0.175)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.0825)),
    )

    door = model.part("glovebox_door")
    door.visual(
        lower_panel_mesh,
        origin=Origin(xyz=(0.0, 0.048, -0.0005)),
        material=dashboard_plastic,
    )
    door.visual(
        upper_bridge_mesh,
        origin=Origin(xyz=(0.0, 0.123, -0.0003)),
        material=dashboard_plastic,
    )
    door.visual(
        shoulder_panel_mesh,
        origin=Origin(xyz=(-0.1055, 0.104, -0.0005)),
        material=dashboard_plastic,
    )
    door.visual(
        shoulder_panel_mesh,
        origin=Origin(xyz=(0.1055, 0.104, -0.0005)),
        material=dashboard_plastic,
    )
    door.visual(
        appliqué_mesh,
        origin=Origin(xyz=(0.0, 0.051, 0.0015)),
        material=soft_touch,
    )

    door.visual(
        Box((0.086, 0.024, 0.003)),
        origin=Origin(xyz=(0.0, 0.105, -0.0035)),
        material=dark_rubber,
    )
    door.visual(
        Box((0.004, 0.028, 0.008)),
        origin=Origin(xyz=(-0.043, 0.105, 0.0005)),
        material=dark_rubber,
    )
    door.visual(
        Box((0.004, 0.028, 0.008)),
        origin=Origin(xyz=(0.043, 0.105, 0.0005)),
        material=dark_rubber,
    )
    door.visual(
        Box((0.086, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.119, 0.0005)),
        material=dark_rubber,
    )
    door.visual(
        Box((0.086, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.091, 0.0005)),
        material=dark_rubber,
    )
    door.visual(
        Box((0.074, 0.012, 0.0015)),
        origin=Origin(xyz=(0.0, 0.105, 0.0025)),
        material=satin_trim,
    )
    door.visual(
        Cylinder(radius=0.003, length=0.050),
        origin=Origin(
            xyz=(0.0, 0.105, -0.001),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_trim,
    )

    door.visual(
        Box((0.300, 0.110, 0.004)),
        origin=Origin(xyz=(0.0, 0.056, -0.0065)),
        material=liner_felt,
    )
    door.visual(
        Box((0.250, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.117, -0.006)),
        material=dashboard_plastic,
    )
    door.visual(
        Box((0.008, 0.100, 0.004)),
        origin=Origin(xyz=(-0.146, 0.056, -0.006)),
        material=dashboard_plastic,
    )
    door.visual(
        Box((0.008, 0.100, 0.004)),
        origin=Origin(xyz=(0.146, 0.056, -0.006)),
        material=dashboard_plastic,
    )
    door.visual(
        Cylinder(radius=0.005, length=0.262),
        origin=Origin(
            xyz=(0.0, 0.004, -0.004),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_rubber,
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_h, door_t)),
        mass=0.7,
        origin=Origin(xyz=(0.0, door_h / 2.0, -0.003)),
    )

    model.articulation(
        "bin_to_frame",
        ArticulationType.FIXED,
        parent="compartment_bin",
        child="dashboard_frame",
        origin=Origin(xyz=frame_mount_xyz),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="compartment_bin",
        child="glovebox_door",
        origin=Origin(xyz=(0.0, -0.066, -0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
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
        "dashboard_frame",
        "glovebox_door",
        reason="convex collision hulls for the thin bezel and recessed closed door conservatively overlap at the opening",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("glovebox_door", "dashboard_frame", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("glovebox_door", "compartment_bin", axes="xy", min_overlap=0.08)
    ctx.expect_joint_motion_axis(
        "door_hinge",
        "glovebox_door",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "door_hinge",
        "glovebox_door",
        world_axis="y",
        direction="negative",
        min_delta=0.03,
    )

    with ctx.pose(door_hinge=0.55):
        ctx.expect_aabb_overlap("glovebox_door", "dashboard_frame", axes="xy", min_overlap=0.06)

    with ctx.pose(door_hinge=1.20):
        ctx.expect_aabb_overlap("glovebox_door", "dashboard_frame", axes="xy", min_overlap=0.025)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
