from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
RAIL_LENGTH = 0.42
RAIL_WIDTH = 0.032
RAIL_HEIGHT = 0.012
RAIL_HOLE_DIAMETER = 0.0055
RAIL_BORE_DIAMETER = 0.0095
RAIL_BORE_DEPTH = 0.003
RAIL_HOLE_X = (-0.175, -0.105, -0.035, 0.035, 0.105, 0.175)

CARRIAGE_LENGTH = 0.09
CARRIAGE_WIDTH = 0.062
CARRIAGE_HEIGHT = 0.038
CARRIAGE_HOLE_POINTS = (
    (-0.022, -0.016),
    (-0.022, 0.016),
    (0.022, -0.016),
    (0.022, 0.016),
)
WIPER_THICKNESS = 0.004
WIPER_HEIGHT = 0.016
TRAVEL_HALF_RANGE = 0.12


def _build_rail_mesh():
    import cadquery as cq

    rail_shape = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, -RAIL_HEIGHT))
        .edges("|X")
        .fillet(0.001)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(x_pos, 0.0) for x_pos in RAIL_HOLE_X])
        .cboreHole(
            RAIL_HOLE_DIAMETER,
            RAIL_BORE_DIAMETER,
            RAIL_BORE_DEPTH,
            RAIL_HEIGHT,
        )
    )
    return mesh_from_cadquery(rail_shape, MESH_DIR / "linear_stage_rail.obj")


def _build_carriage_mesh():
    import cadquery as cq

    carriage_shape = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_WIDTH,
            CARRIAGE_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(CARRIAGE_LENGTH - 0.022, CARRIAGE_WIDTH - 0.018)
        .cutBlind(-0.0025)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(CARRIAGE_HOLE_POINTS)
        .hole(0.005, 0.01)
    )
    return mesh_from_cadquery(carriage_shape, MESH_DIR / "linear_stage_carriage.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tool_linear_stage", assets=ASSETS)
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("carriage_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("wiper_black", rgba=(0.08, 0.08, 0.09, 1.0))

    rail = model.part("rail")
    rail.visual(_build_rail_mesh(), material="rail_steel")

    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -RAIL_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(_build_carriage_mesh(), material="carriage_steel")
    carriage.visual(
        Box((WIPER_THICKNESS, CARRIAGE_WIDTH * 0.90, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(
                CARRIAGE_LENGTH / 2.0 - WIPER_THICKNESS / 2.0,
                0.0,
                WIPER_HEIGHT / 2.0,
            )
        ),
        material="wiper_black",
    )
    carriage.visual(
        Box((WIPER_THICKNESS, CARRIAGE_WIDTH * 0.90, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(
                -CARRIAGE_LENGTH / 2.0 + WIPER_THICKNESS / 2.0,
                0.0,
                WIPER_HEIGHT / 2.0,
            )
        ),
        material="wiper_black",
    )

    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent="rail",
        child="carriage",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=1500.0,
            velocity=0.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.expect_origin_gap("carriage", "rail", axis="z", min_gap=0.0)
    ctx.expect_aabb_gap("carriage", "rail", axis="z", max_gap=0.002, max_penetration=0.0)
    ctx.expect_aabb_overlap("carriage", "rail", axes="xy", min_overlap=0.002)
    ctx.expect_joint_motion_axis(
        "rail_to_carriage",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.04,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
