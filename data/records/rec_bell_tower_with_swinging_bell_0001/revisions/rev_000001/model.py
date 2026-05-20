from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bell_tower", assets=ASSETS)

    stone = Material(name="weathered_stone", rgba=(0.71, 0.69, 0.66, 1.0))
    timber = Material(name="aged_oak", rgba=(0.36, 0.24, 0.15, 1.0))
    bronze = Material(name="patinated_bronze", rgba=(0.57, 0.43, 0.24, 1.0))
    steel = Material(name="forged_steel", rgba=(0.24, 0.24, 0.25, 1.0))
    slate = Material(name="slate_roof", rgba=(0.26, 0.30, 0.35, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((1.96, 1.96, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=stone,
    )
    pedestal.visual(
        Box((1.70, 1.70, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=stone,
    )
    pedestal.visual(
        Box((1.44, 1.44, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=stone,
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((1.96, 1.96, 0.48)),
        mass=2500.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )

    frame = model.part("belfry_frame")
    frame.visual(
        Box((1.24, 1.24, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=timber,
    )
    for px in (-0.68, 0.68):
        for py in (-0.82, 0.82):
            frame.visual(
                Box((0.18, 0.18, 1.96)),
                origin=Origin(xyz=(px, py, 0.98)),
                material=timber,
            )
    for y in (-0.82, 0.82):
        frame.visual(
            Box((1.54, 0.18, 0.14)),
            origin=Origin(xyz=(0.0, y, 0.07)),
            material=timber,
        )
        frame.visual(
            Box((1.36, 0.12, 0.10)),
            origin=Origin(xyz=(0.0, y, 0.86)),
            material=timber,
        )
        frame.visual(
            Box((1.20, 0.12, 0.10)),
            origin=Origin(xyz=(0.0, y, 1.44)),
            material=timber,
        )
        frame.visual(
            Box((1.36, 0.12, 0.12)),
            origin=Origin(xyz=(0.0, y, 1.90)),
            material=timber,
        )
        frame.visual(
            Box((1.36, 0.16, 0.10)),
            origin=Origin(xyz=(0.0, y, 1.64)),
            material=steel,
        )
    for x in (-0.68, 0.68):
        frame.visual(
            Box((0.18, 1.82, 0.14)),
            origin=Origin(xyz=(x, 0.0, 0.07)),
            material=timber,
        )
        frame.visual(
            Box((0.12, 1.64, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.86)),
            material=timber,
        )
        frame.visual(
            Box((0.12, 1.48, 0.10)),
            origin=Origin(xyz=(x, 0.0, 1.44)),
            material=timber,
        )
        frame.visual(
            Box((0.12, 1.64, 0.12)),
            origin=Origin(xyz=(x, 0.0, 1.90)),
            material=timber,
        )
    frame.visual(
        Box((1.24, 1.24, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.92)),
        material=timber,
    )
    for x in (-0.585, 0.585):
        frame.visual(
            Box((0.18, 0.16, 0.36)),
            origin=Origin(xyz=(x, 0.0, 1.70)),
            material=steel,
        )
    frame.inertial = Inertial.from_geometry(
        Box((1.54, 1.54, 1.96)),
        mass=1120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
    )

    roof = model.part("roof")
    roof.visual(
        Box((1.34, 1.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=timber,
    )
    roof_shell = LoftGeometry(
        [
            [(-0.84, -0.84, 0.04), (0.84, -0.84, 0.04), (0.84, 0.84, 0.04), (-0.84, 0.84, 0.04)],
            [(-0.70, -0.70, 0.10), (0.70, -0.70, 0.10), (0.70, 0.70, 0.10), (-0.70, 0.70, 0.10)],
            [(-0.48, -0.48, 0.32), (0.48, -0.48, 0.32), (0.48, 0.48, 0.32), (-0.48, 0.48, 0.32)],
            [(-0.16, -0.16, 0.66), (0.16, -0.16, 0.66), (0.16, 0.16, 0.66), (-0.16, 0.16, 0.66)],
        ],
        cap=True,
        closed=True,
    )
    roof.visual(
        mesh_from_geometry(roof_shell, ASSETS.mesh_path("roof_shell.obj")),
        material=slate,
    )
    roof.visual(
        Cylinder(radius=0.04, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=steel,
    )
    roof.visual(
        Sphere(radius=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        material=steel,
    )
    roof.inertial = Inertial.from_geometry(
        Box((1.68, 1.68, 0.92)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    bell = model.part("bell")
    bell_profile = [
        (0.0, 0.04),
        (0.08, 0.04),
        (0.13, 0.00),
        (0.19, -0.06),
        (0.24, -0.16),
        (0.29, -0.30),
        (0.32, -0.48),
        (0.33, -0.58),
        (0.30, -0.64),
        (0.25, -0.68),
        (0.0, -0.68),
    ]
    bell.visual(
        Cylinder(radius=0.035, length=0.06),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
    )
    bell.visual(
        Cylinder(radius=0.035, length=0.06),
        origin=Origin(xyz=(0.96, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
    )
    bell.visual(
        Box((0.86, 0.14, 0.10)),
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        material=timber,
    )
    for x in (0.36, 0.64):
        bell.visual(
            Box((0.08, 0.08, 0.22)),
            origin=Origin(xyz=(x, 0.0, -0.11)),
            material=steel,
        )
    bell.visual(
        Box((0.32, 0.12, 0.08)),
        origin=Origin(xyz=(0.50, 0.0, -0.22)),
        material=steel,
    )
    bell.visual(
        mesh_from_geometry(
            LatheGeometry(bell_profile, segments=64), ASSETS.mesh_path("bell_body.obj")
        ),
        origin=Origin(xyz=(0.495, 0.0, -0.26)),
        material=bronze,
    )
    bell.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.495, 0.0, -0.52)),
        material=steel,
    )
    bell.visual(
        Sphere(radius=0.065),
        origin=Origin(xyz=(0.495, 0.0, -0.855)),
        material=steel,
    )
    bell.inertial = Inertial.from_geometry(
        Box((1.00, 0.76, 1.02)),
        mass=290.0,
        origin=Origin(xyz=(0.495, 0.0, -0.34)),
    )

    model.articulation(
        "pedestal_to_frame",
        ArticulationType.FIXED,
        parent="pedestal",
        child="belfry_frame",
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )
    model.articulation(
        "frame_to_roof",
        ArticulationType.FIXED,
        parent="belfry_frame",
        child="roof",
        origin=Origin(xyz=(0.0, 0.0, 1.96)),
    )
    model.articulation(
        "frame_to_bell",
        ArticulationType.REVOLUTE,
        parent="belfry_frame",
        child="bell",
        origin=Origin(xyz=(-0.495, 0.0, 1.52)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=-0.40,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_gap("belfry_frame", "pedestal", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_gap("roof", "belfry_frame", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_overlap("roof", "belfry_frame", axes="xy", min_overlap=1.15)
    ctx.expect_aabb_overlap("bell", "pedestal", axes="xy", min_overlap=0.60)
    ctx.expect_aabb_overlap("bell", "belfry_frame", axes="xy", min_overlap=0.60)
    ctx.expect_aabb_gap("bell", "pedestal", axis="z", max_gap=0.60, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "frame_to_bell",
        "bell",
        world_axis="y",
        direction="positive",
        min_delta=0.06,
    )

    with ctx.pose(frame_to_bell=-0.40):
        ctx.expect_aabb_overlap("bell", "pedestal", axes="xy", min_overlap=0.44)
        ctx.expect_aabb_overlap("bell", "belfry_frame", axes="xy", min_overlap=0.44)
        ctx.expect_aabb_gap("bell", "pedestal", axis="z", max_gap=0.55, max_penetration=0.0)

    with ctx.pose(frame_to_bell=0.40):
        ctx.expect_aabb_overlap("bell", "pedestal", axes="xy", min_overlap=0.44)
        ctx.expect_aabb_overlap("bell", "belfry_frame", axes="xy", min_overlap=0.44)
        ctx.expect_aabb_gap("bell", "pedestal", axis="z", max_gap=0.55, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
