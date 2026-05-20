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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _body_shell_mesh():
    profile = [
        (0.016, 0.000),
        (0.021, 0.004),
        (0.024, 0.010),
        (0.023, 0.020),
        (0.021, 0.036),
        (0.019, 0.054),
        (0.0175, 0.070),
        (0.0165, 0.084),
        (0.0175, 0.092),
        (0.014, 0.098),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=64),
        ASSETS.mesh_path("faucet_body_shell.obj"),
    )


def _spout_mesh():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.052),
                (0.020, 0.000, 0.060),
                (0.080, 0.000, 0.094),
                (0.148, 0.000, 0.078),
            ],
            radius=0.0105,
            samples_per_segment=20,
            radial_segments=24,
            cap_ends=True,
        ),
        ASSETS.mesh_path("faucet_spout.obj"),
    )


def _handle_lever_mesh():
    return mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.000, 0.000, 0.000),
                (0.002, 0.018, 0.001),
                (0.005, 0.050, 0.004),
                (0.008, 0.086, 0.007),
            ],
            profile=rounded_rect_profile(0.011, 0.0048, radius=0.0014),
            samples_per_segment=18,
            cap_profile=True,
        ),
        ASSETS.mesh_path("faucet_handle_lever.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_handle_faucet", assets=ASSETS)

    polished_chrome = model.material(
        "polished_chrome",
        rgba=(0.83, 0.85, 0.88, 1.0),
    )
    brushed_nickel = model.material(
        "brushed_nickel",
        rgba=(0.72, 0.74, 0.77, 1.0),
    )
    dark_rubber = model.material(
        "dark_rubber",
        rgba=(0.08, 0.08, 0.09, 1.0),
    )
    dark_plastic = model.material(
        "dark_plastic",
        rgba=(0.16, 0.17, 0.18, 1.0),
    )

    base_flange = model.part("base_flange")
    base_flange.visual(
        Cylinder(radius=0.0315, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dark_rubber,
        name="deck_gasket",
    )
    base_flange.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=brushed_nickel,
        name="flange_cap",
    )
    base_flange.visual(
        Cylinder(radius=0.022, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=polished_chrome,
        name="inner_trim_ring",
    )
    base_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.010),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    faucet_body = model.part("faucet_body")
    faucet_body.visual(
        _body_shell_mesh(),
        material=polished_chrome,
        name="body_shell",
    )
    faucet_body.visual(
        Cylinder(radius=0.0245, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_nickel,
        name="lower_body_band",
    )
    faucet_body.visual(
        Cylinder(radius=0.0145, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=polished_chrome,
        name="neck_cap",
    )
    faucet_body.visual(
        _spout_mesh(),
        material=polished_chrome,
        name="arched_spout",
    )
    faucet_body.visual(
        Cylinder(radius=0.0112, length=0.010),
        origin=Origin(
            xyz=(0.149, 0.0, 0.078),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=polished_chrome,
        name="spout_nozzle_ring",
    )
    faucet_body.visual(
        Cylinder(radius=0.0062, length=0.004),
        origin=Origin(
            xyz=(0.154, 0.0, 0.078),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_plastic,
        name="aerator_insert",
    )
    faucet_body.visual(
        Cylinder(radius=0.011, length=0.038),
        origin=Origin(
            xyz=(0.0, 0.030, 0.056),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=polished_chrome,
        name="cartridge_housing",
    )
    faucet_body.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(
            xyz=(0.0, 0.047, 0.056),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brushed_nickel,
        name="handle_trim_cap",
    )
    faucet_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.100),
        mass=1.65,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    control_handle = model.part("control_handle")
    control_handle.visual(
        Cylinder(radius=0.0125, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=polished_chrome,
        name="handle_hub",
    )
    control_handle.visual(
        _handle_lever_mesh(),
        material=polished_chrome,
        name="handle_lever",
    )
    control_handle.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(
            xyz=(0.008, 0.086, 0.007),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=polished_chrome,
        name="handle_tip",
    )
    control_handle.inertial = Inertial.from_geometry(
        Box((0.024, 0.096, 0.018)),
        mass=0.24,
        origin=Origin(xyz=(0.004, 0.048, 0.007)),
    )

    model.articulation(
        "flange_to_body",
        ArticulationType.FIXED,
        parent="base_flange",
        child="faucet_body",
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "side_handle",
        ArticulationType.REVOLUTE,
        parent="faucet_body",
        child="control_handle",
        origin=Origin(xyz=(0.0, 0.047, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(52.0),
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

    ctx.expect_aabb_overlap("faucet_body", "base_flange", axes="xy", min_overlap=0.035)
    ctx.expect_aabb_gap(
        "faucet_body",
        "base_flange",
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
    )

    ctx.expect_aabb_overlap("control_handle", "faucet_body", axes="xz", min_overlap=0.018)
    ctx.expect_aabb_contact("control_handle", "faucet_body")
    ctx.expect_aabb_gap(
        "control_handle",
        "faucet_body",
        axis="y",
        max_gap=0.001,
        max_penetration=0.018,
    )
    ctx.expect_joint_motion_axis(
        "side_handle",
        "control_handle",
        world_axis="z",
        direction="positive",
        min_delta=0.012,
    )

    with ctx.pose(side_handle=0.0):
        ctx.expect_aabb_overlap("control_handle", "faucet_body", axes="xz", min_overlap=0.018)
        ctx.expect_aabb_contact("control_handle", "faucet_body")
        ctx.expect_aabb_gap(
            "control_handle",
            "faucet_body",
            axis="y",
            max_gap=0.001,
            max_penetration=0.018,
        )

    with ctx.pose(side_handle=math.radians(52.0)):
        ctx.expect_aabb_overlap("control_handle", "faucet_body", axes="xz", min_overlap=0.014)
        ctx.expect_aabb_contact("control_handle", "faucet_body")
        ctx.expect_aabb_gap(
            "control_handle",
            "faucet_body",
            axis="y",
            max_gap=0.001,
            max_penetration=0.018,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
