from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


# >>> USER_CODE_START
def _add_square_tube(
    part,
    *,
    outer: float,
    wall: float,
    height: float,
    z_bottom: float,
    material: str,
) -> None:
    inner = outer - 2.0 * wall
    side_offset = outer / 2.0 - wall / 2.0
    z_center = z_bottom + height / 2.0
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(side_offset, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(-side_offset, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, side_offset, z_center)),
        material=material,
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, -side_offset, z_center)),
        material=material,
    )


def _add_square_ring(
    part,
    *,
    outer: float,
    inner: float,
    height: float,
    z_center: float,
    material: str,
) -> None:
    wall = (outer - inner) / 2.0
    side_offset = outer / 2.0 - wall / 2.0
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(side_offset, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(-side_offset, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, side_offset, z_center)),
        material=material,
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, -side_offset, z_center)),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_telescoping_mast", assets=ASSETS)

    model.material("housing_dark", rgba=(0.21, 0.23, 0.26, 1.0))
    model.material("housing_light", rgba=(0.34, 0.36, 0.40, 1.0))
    model.material("mast_graphite", rgba=(0.11, 0.12, 0.14, 1.0))
    model.material("machined_aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("optic_glass", rgba=(0.18, 0.40, 0.55, 0.75))
    model.material("accent_orange", rgba=(0.90, 0.48, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.44, 0.32, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material="housing_dark",
    )
    base.visual(
        Box((0.30, 0.22, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material="housing_light",
    )
    base.visual(
        Box((0.08, 0.18, 0.025)),
        origin=Origin(xyz=(-0.14, 0.0, 0.1025)),
        material="housing_dark",
    )
    base.visual(
        Box((0.016, 0.18, 0.06)),
        origin=Origin(xyz=(0.228, 0.0, 0.075)),
        material="machined_aluminum",
    )
    base.visual(
        Box((0.010, 0.08, 0.012)),
        origin=Origin(xyz=(0.237, 0.0, 0.108)),
        material="accent_orange",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(0.0, 0.167, 0.085), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_aluminum",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(0.0, -0.167, 0.085), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_aluminum",
    )
    for x in (-0.16, 0.16):
        for y in (-0.11, 0.11):
            base.visual(
                Cylinder(radius=0.024, length=0.012),
                origin=Origin(xyz=(x, y, 0.006)),
                material="rubber",
            )
    _add_square_ring(
        base,
        outer=0.118,
        inner=0.096,
        height=0.016,
        z_center=0.145,
        material="machined_aluminum",
    )
    _add_square_tube(
        base,
        outer=0.112,
        wall=0.008,
        height=0.184,
        z_bottom=0.149,
        material="mast_graphite",
    )
    _add_square_ring(
        base,
        outer=0.122,
        inner=0.094,
        height=0.018,
        z_center=0.340,
        material="machined_aluminum",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.32, 0.35)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
    )

    mast_stage = model.part("mast_stage")
    mast_stage.visual(
        Box((0.082, 0.082, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="machined_aluminum",
    )
    _add_square_tube(
        mast_stage,
        outer=0.088,
        wall=0.007,
        height=0.366,
        z_bottom=0.008,
        material="mast_graphite",
    )
    mast_stage.visual(
        Box((0.084, 0.084, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.378)),
        material="machined_aluminum",
    )
    _add_square_tube(
        mast_stage,
        outer=0.078,
        wall=0.005,
        height=0.136,
        z_bottom=0.352,
        material="machined_aluminum",
    )
    mast_stage.visual(
        Cylinder(radius=0.029, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.495)),
        material="machined_aluminum",
    )
    mast_stage.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.506)),
        material="accent_orange",
    )
    mast_stage.visual(
        Cylinder(radius=0.038, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.519)),
        material="machined_aluminum",
    )
    mast_stage.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, 0.52)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="machined_aluminum",
    )
    pan_head.visual(
        Box((0.09, 0.09, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material="housing_light",
    )
    pan_head.visual(
        Box((0.052, 0.052, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material="housing_dark",
    )
    pan_head.visual(
        Box((0.15, 0.08, 0.055)),
        origin=Origin(xyz=(0.010, 0.0, 0.0855)),
        material="housing_light",
    )
    pan_head.visual(
        Box((0.030, 0.078, 0.042)),
        origin=Origin(xyz=(0.100, 0.0, 0.0855)),
        material="housing_dark",
    )
    pan_head.visual(
        Box((0.004, 0.050, 0.028)),
        origin=Origin(xyz=(0.113, 0.0, 0.086)),
        material="optic_glass",
    )
    pan_head.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(-0.082, 0.0, 0.083), rpy=(0.0, pi / 2.0, 0.0)),
        material="housing_dark",
    )
    pan_head.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(-0.008, 0.0, 0.117)),
        material="accent_orange",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.20, 0.10, 0.13)),
        mass=1.8,
        origin=Origin(xyz=(0.005, 0.0, 0.065)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.28,
            effort=150.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "pan_yaw",
        ArticulationType.REVOLUTE,
        parent=mast_stage,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.528)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=20.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("mast_stage", "base", axes="xy", min_overlap=0.06)
    ctx.expect_origin_distance("mast_stage", "base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("pan_head", "mast_stage", axes="xy", min_overlap=0.04)
    ctx.expect_origin_distance("pan_head", "mast_stage", axes="xy", max_dist=0.03)
    ctx.expect_aabb_gap("pan_head", "mast_stage", axis="z", max_gap=0.02, max_penetration=0.0)
    ctx.expect_origin_gap("pan_head", "base", axis="z", min_gap=0.24)
    ctx.expect_joint_motion_axis(
        "mast_extension",
        "mast_stage",
        world_axis="z",
        direction="positive",
        min_delta=0.12,
    )

    with ctx.pose(mast_extension=0.28):
        ctx.expect_aabb_overlap("mast_stage", "base", axes="xy", min_overlap=0.06)
        ctx.expect_origin_distance("mast_stage", "base", axes="xy", max_dist=0.01)
        ctx.expect_aabb_overlap("pan_head", "mast_stage", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_gap("pan_head", "mast_stage", axis="z", max_gap=0.02, max_penetration=0.0)
        ctx.expect_origin_gap("pan_head", "base", axis="z", min_gap=0.50)

    with ctx.pose(pan_yaw=pi / 2.0):
        ctx.expect_aabb_overlap("pan_head", "mast_stage", axes="xy", min_overlap=0.04)
        ctx.expect_origin_distance("pan_head", "mast_stage", axes="xy", max_dist=0.03)
        ctx.expect_aabb_gap("pan_head", "mast_stage", axis="z", max_gap=0.02, max_penetration=0.0)

    with ctx.pose({"mast_extension": 0.28, "pan_yaw": -pi / 2.0}):
        ctx.expect_aabb_overlap("pan_head", "mast_stage", axes="xy", min_overlap=0.04)
        ctx.expect_origin_distance("pan_head", "mast_stage", axes="xy", max_dist=0.03)
        ctx.expect_aabb_gap("pan_head", "mast_stage", axis="z", max_gap=0.02, max_penetration=0.0)
        ctx.expect_origin_gap("pan_head", "base", axis="z", min_gap=0.50)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
