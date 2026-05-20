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
    BoxGeometry,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_materials() -> dict[str, Material]:
    return {
        "body_red": Material(name="body_red", rgba=(0.78, 0.10, 0.08, 1.0)),
        "charcoal": Material(name="charcoal", rgba=(0.18, 0.19, 0.21, 1.0)),
        "rubber": Material(name="rubber", rgba=(0.10, 0.10, 0.11, 1.0)),
        "steel": Material(name="steel", rgba=(0.73, 0.76, 0.79, 1.0)),
        "smoke_clear": Material(name="smoke_clear", rgba=(0.58, 0.68, 0.76, 0.35)),
        "mid_gray": Material(name="mid_gray", rgba=(0.42, 0.45, 0.48, 1.0)),
    }


def _body_shell_mesh():
    shell = superellipse_side_loft(
        [
            (-0.19, 0.015, 0.16, 0.19),
            (-0.11, 0.006, 0.20, 0.25),
            (0.00, 0.000, 0.23, 0.30),
            (0.10, 0.000, 0.21, 0.26),
            (0.18, 0.020, 0.16, 0.17),
        ],
        exponents=(3.0, 2.8, 2.7, 2.8, 3.1),
        segments=56,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(shell, ASSETS.mesh_path("vacuum_body_shell.obj"))


def _carry_handle_mesh():
    handle = tube_from_spline_points(
        [
            (0.0, -0.045, 0.188),
            (0.0, -0.015, 0.228),
            (0.0, 0.040, 0.236),
            (0.0, 0.085, 0.208),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(handle, ASSETS.mesh_path("vacuum_carry_handle.obj"))


def _hose_mesh():
    hose = tube_from_spline_points(
        [
            (0.0, 0.000, 0.000),
            (0.0, 0.050, 0.030),
            (0.0, 0.100, 0.120),
            (0.0, 0.180, 0.300),
            (0.0, 0.230, 0.500),
            (0.0, 0.250, 0.660),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(hose, ASSETS.mesh_path("vacuum_hose.obj"))


def _wand_handle_mesh():
    handle = tube_from_spline_points(
        [
            (0.0, -0.100, 0.040),
            (0.0, -0.140, 0.110),
            (0.0, -0.112, 0.205),
            (0.0, -0.035, 0.232),
            (0.0, 0.020, 0.170),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    return mesh_from_geometry(handle, ASSETS.mesh_path("vacuum_wand_handle.obj"))


def _floor_head_top_mesh():
    top_shell = BoxGeometry((0.21, 0.075, 0.028))
    return mesh_from_geometry(top_shell, ASSETS.mesh_path("vacuum_floor_head_top.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_cleaner", assets=ASSETS)
    materials = _make_materials()
    model.materials.extend(materials.values())

    main_body = model.part("main_body")
    main_body.visual(_body_shell_mesh(), material=materials["body_red"])
    main_body.visual(
        Box((0.22, 0.14, 0.05)),
        origin=Origin(xyz=(0.0, 0.020, 0.025)),
        material=materials["charcoal"],
    )
    main_body.visual(
        Box((0.18, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.205, 0.030)),
        material=materials["charcoal"],
    )
    main_body.visual(
        Box((0.14, 0.10, 0.09)),
        origin=Origin(xyz=(0.0, 0.055, 0.145)),
        material=materials["smoke_clear"],
    )
    main_body.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(0.145, -0.050, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["rubber"],
    )
    main_body.visual(
        Cylinder(radius=0.075, length=0.030),
        origin=Origin(xyz=(-0.145, -0.050, 0.075), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["rubber"],
    )
    main_body.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.085, 0.120, 0.026), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["rubber"],
    )
    main_body.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(-0.085, 0.120, 0.026), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["rubber"],
    )
    main_body.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.0, 0.205, 0.140), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["mid_gray"],
    )
    main_body.visual(_carry_handle_mesh(), material=materials["charcoal"])
    main_body.inertial = Inertial.from_geometry(
        Box((0.30, 0.42, 0.24)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    connector_hose = model.part("connector_hose")
    connector_hose.visual(_hose_mesh(), material=materials["charcoal"])
    connector_hose.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["mid_gray"],
    )
    connector_hose.visual(
        Cylinder(radius=0.024, length=0.085),
        origin=Origin(xyz=(0.0, 0.250, 0.705)),
        material=materials["mid_gray"],
    )
    connector_hose.inertial = Inertial.from_geometry(
        Box((0.06, 0.30, 0.76)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.13, 0.34)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.021, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=materials["mid_gray"],
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=materials["steel"],
    )
    wand.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.760)),
        material=materials["charcoal"],
    )
    wand.visual(
        Box((0.075, 0.055, 0.110)),
        origin=Origin(xyz=(0.0, -0.030, 0.040)),
        material=materials["charcoal"],
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.140),
        origin=Origin(xyz=(0.0, -0.095, 0.100), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["charcoal"],
    )
    wand.visual(
        Box((0.022, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.015, 0.105)),
        material=materials["body_red"],
    )
    wand.visual(_wand_handle_mesh(), material=materials["charcoal"])
    wand.inertial = Inertial.from_geometry(
        Box((0.12, 0.20, 0.94)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.01, -0.36)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.018, length=0.085),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["mid_gray"],
    )
    floor_head.visual(
        Box((0.070, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, 0.000, -0.040)),
        material=materials["charcoal"],
    )
    floor_head.visual(
        Box((0.310, 0.120, 0.048)),
        origin=Origin(xyz=(0.0, 0.095, -0.033)),
        material=materials["charcoal"],
    )
    floor_head.visual(
        _floor_head_top_mesh(),
        origin=Origin(xyz=(0.0, 0.030, -0.010)),
        material=materials["body_red"],
    )
    floor_head.visual(
        Box((0.280, 0.090, 0.010)),
        origin=Origin(xyz=(0.0, 0.105, -0.056)),
        material=materials["rubber"],
    )
    floor_head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.120, 0.026, -0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["rubber"],
    )
    floor_head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(-0.120, 0.026, -0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["rubber"],
    )
    floor_head.visual(
        Box((0.270, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.030, -0.046)),
        material=materials["rubber"],
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 0.09)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.10, -0.03)),
    )

    model.articulation(
        "body_to_hose",
        ArticulationType.FIXED,
        parent="main_body",
        child="connector_hose",
        origin=Origin(xyz=(0.0, 0.208, 0.140)),
    )
    model.articulation(
        "hose_to_wand",
        ArticulationType.REVOLUTE,
        parent="connector_hose",
        child="wand",
        origin=Origin(xyz=(0.0, 0.250, 0.748), rpy=(0.58, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.35,
            upper=0.45,
        ),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent="wand",
        child="floor_head",
        origin=Origin(xyz=(0.0, 0.0, -0.820), rpy=(-0.58, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=-0.35,
            upper=0.60,
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
        "main_body", "connector_hose", reason="hose cuff plugs into the intake collar"
    )
    ctx.allow_overlap("connector_hose", "wand", reason="wand socket nests inside the hose cuff")
    ctx.allow_overlap(
        "wand", "floor_head", reason="swivel neck shares conservative collision volume at the pivot"
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("main_body", "connector_hose", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("wand", "floor_head", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_gap("wand", "floor_head", axis="z", max_gap=0.06, max_penetration=0.11)
    ctx.expect_joint_motion_axis(
        "hose_to_wand",
        "floor_head",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "wand_to_floor_head",
        "floor_head",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )

    with ctx.pose(hose_to_wand=-0.30):
        ctx.expect_aabb_overlap("wand", "floor_head", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_gap("wand", "floor_head", axis="z", max_gap=0.07, max_penetration=0.11)

    with ctx.pose(hose_to_wand=0.35):
        ctx.expect_aabb_overlap("wand", "floor_head", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_gap("wand", "floor_head", axis="z", max_gap=0.08, max_penetration=0.11)

    with ctx.pose(wand_to_floor_head=-0.25):
        ctx.expect_aabb_overlap("wand", "floor_head", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_gap("wand", "floor_head", axis="z", max_gap=0.06, max_penetration=0.11)

    with ctx.pose(wand_to_floor_head=0.45):
        ctx.expect_aabb_overlap("wand", "floor_head", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_gap("wand", "floor_head", axis="z", max_gap=0.08, max_penetration=0.11)

    with ctx.pose(hose_to_wand=0.25, wand_to_floor_head=0.35):
        ctx.expect_aabb_overlap("wand", "floor_head", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_gap("wand", "floor_head", axis="z", max_gap=0.09, max_penetration=0.11)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
