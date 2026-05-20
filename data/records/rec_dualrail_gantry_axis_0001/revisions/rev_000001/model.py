from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_LENGTH = 0.58
BASE_WIDTH = 0.19
BASE_THICKNESS = 0.016
RAIL_Y = 0.06
PEDESTAL_LENGTH = 0.52
PEDESTAL_WIDTH = 0.028
PEDESTAL_HEIGHT = 0.014
RAIL_LENGTH = 0.50
RAIL_WIDTH = 0.015
RAIL_HEIGHT = 0.010
CENTER_RIB_LENGTH = 0.48
CENTER_RIB_WIDTH = 0.032
CENTER_RIB_HEIGHT = 0.016
END_BLOCK_LENGTH = 0.032
END_BLOCK_WIDTH = 0.160
END_BLOCK_HEIGHT = 0.030
SIDE_STRIP_LENGTH = 0.50
SIDE_STRIP_WIDTH = 0.014
SIDE_STRIP_HEIGHT = 0.012

RAIL_TOP_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + RAIL_HEIGHT
CARRIAGE_CLEARANCE = 0.002
JOINT_Z = RAIL_TOP_Z + (CARRIAGE_CLEARANCE / 2.0)

GUIDE_BLOCK_LENGTH = 0.065
GUIDE_BLOCK_WIDTH = 0.032
GUIDE_BLOCK_HEIGHT = 0.023
GUIDE_BLOCK_X = 0.036
GUIDE_BLOCK_CENTER_Z = (CARRIAGE_CLEARANCE / 2.0) + (GUIDE_BLOCK_HEIGHT / 2.0)
RAIL_CAP_LENGTH = 0.145
RAIL_CAP_WIDTH = 0.020
RAIL_CAP_HEIGHT = 0.012
RAIL_CAP_CENTER_Z = 0.024
CENTER_WEB_LENGTH = 0.112
CENTER_WEB_WIDTH = 0.030
CENTER_WEB_HEIGHT = 0.025
CENTER_WEB_CENTER_Z = CENTER_WEB_HEIGHT / 2.0
BRIDGE_LENGTH = 0.145
BRIDGE_WIDTH = 0.172
BRIDGE_HEIGHT = 0.014
BRIDGE_CENTER_Z = CARRIAGE_CLEARANCE + GUIDE_BLOCK_HEIGHT + (BRIDGE_HEIGHT / 2.0)
SADDLE_LENGTH = 0.090
SADDLE_WIDTH = 0.110
SADDLE_HEIGHT = 0.016
SADDLE_CENTER_Z = CARRIAGE_CLEARANCE + GUIDE_BLOCK_HEIGHT + BRIDGE_HEIGHT + (SADDLE_HEIGHT / 2.0)
RIB_LENGTH = 0.014
RIB_WIDTH = 0.100
RIB_HEIGHT = 0.028
RIB_X = 0.052
RIB_CENTER_Z = CARRIAGE_CLEARANCE + GUIDE_BLOCK_HEIGHT + BRIDGE_HEIGHT + (RIB_HEIGHT / 2.0)

SLIDE_LIMIT = 0.20


def _add_box_visual(part, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry_axis", assets=ASSETS)

    model.material("anodized_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("rail_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("carriage_silver", rgba=(0.84, 0.86, 0.89, 1.0))
    model.material("block_graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("accent_blue", rgba=(0.22, 0.33, 0.54, 1.0))

    base = model.part("base")
    _add_box_visual(
        base,
        (BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS),
        (0.0, 0.0, BASE_THICKNESS / 2.0),
        "anodized_black",
    )
    _add_box_visual(
        base,
        (CENTER_RIB_LENGTH, CENTER_RIB_WIDTH, CENTER_RIB_HEIGHT),
        (0.0, 0.0, BASE_THICKNESS + (CENTER_RIB_HEIGHT / 2.0)),
        "anodized_black",
    )
    _add_box_visual(
        base,
        (PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT),
        (0.0, RAIL_Y, BASE_THICKNESS + (PEDESTAL_HEIGHT / 2.0)),
        "anodized_black",
    )
    _add_box_visual(
        base,
        (PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT),
        (0.0, -RAIL_Y, BASE_THICKNESS + (PEDESTAL_HEIGHT / 2.0)),
        "anodized_black",
    )
    _add_box_visual(
        base,
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, RAIL_Y, BASE_THICKNESS + PEDESTAL_HEIGHT + (RAIL_HEIGHT / 2.0)),
        "rail_steel",
    )
    _add_box_visual(
        base,
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, -RAIL_Y, BASE_THICKNESS + PEDESTAL_HEIGHT + (RAIL_HEIGHT / 2.0)),
        "rail_steel",
    )
    _add_box_visual(
        base,
        (END_BLOCK_LENGTH, END_BLOCK_WIDTH, END_BLOCK_HEIGHT),
        ((BASE_LENGTH / 2.0) - (END_BLOCK_LENGTH / 2.0), 0.0, END_BLOCK_HEIGHT / 2.0),
        "accent_blue",
    )
    _add_box_visual(
        base,
        (END_BLOCK_LENGTH, END_BLOCK_WIDTH, END_BLOCK_HEIGHT),
        ((-BASE_LENGTH / 2.0) + (END_BLOCK_LENGTH / 2.0), 0.0, END_BLOCK_HEIGHT / 2.0),
        "accent_blue",
    )
    _add_box_visual(
        base,
        (SIDE_STRIP_LENGTH, SIDE_STRIP_WIDTH, SIDE_STRIP_HEIGHT),
        (
            0.0,
            (BASE_WIDTH / 2.0) - (SIDE_STRIP_WIDTH / 2.0) - 0.005,
            BASE_THICKNESS + (SIDE_STRIP_HEIGHT / 2.0),
        ),
        "anodized_black",
    )
    _add_box_visual(
        base,
        (SIDE_STRIP_LENGTH, SIDE_STRIP_WIDTH, SIDE_STRIP_HEIGHT),
        (
            0.0,
            (-BASE_WIDTH / 2.0) + (SIDE_STRIP_WIDTH / 2.0) + 0.005,
            BASE_THICKNESS + (SIDE_STRIP_HEIGHT / 2.0),
        ),
        "anodized_black",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, RAIL_TOP_Z)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z / 2.0)),
    )

    carriage = model.part("carriage")
    for x in (-GUIDE_BLOCK_X, GUIDE_BLOCK_X):
        _add_box_visual(
            carriage,
            (GUIDE_BLOCK_LENGTH, GUIDE_BLOCK_WIDTH, GUIDE_BLOCK_HEIGHT),
            (x, RAIL_Y, GUIDE_BLOCK_CENTER_Z),
            "block_graphite",
        )
        _add_box_visual(
            carriage,
            (GUIDE_BLOCK_LENGTH, GUIDE_BLOCK_WIDTH, GUIDE_BLOCK_HEIGHT),
            (x, -RAIL_Y, GUIDE_BLOCK_CENTER_Z),
            "block_graphite",
        )
    _add_box_visual(
        carriage,
        (RAIL_CAP_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_HEIGHT),
        (0.0, RAIL_Y, RAIL_CAP_CENTER_Z),
        "carriage_silver",
    )
    _add_box_visual(
        carriage,
        (RAIL_CAP_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_HEIGHT),
        (0.0, -RAIL_Y, RAIL_CAP_CENTER_Z),
        "carriage_silver",
    )
    _add_box_visual(
        carriage,
        (CENTER_WEB_LENGTH, CENTER_WEB_WIDTH, CENTER_WEB_HEIGHT),
        (0.0, 0.0, CENTER_WEB_CENTER_Z),
        "carriage_silver",
    )
    _add_box_visual(
        carriage,
        (BRIDGE_LENGTH, BRIDGE_WIDTH, BRIDGE_HEIGHT),
        (0.0, 0.0, BRIDGE_CENTER_Z),
        "carriage_silver",
    )
    _add_box_visual(
        carriage,
        (SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_HEIGHT),
        (0.0, 0.0, SADDLE_CENTER_Z),
        "accent_blue",
    )
    _add_box_visual(
        carriage,
        (RIB_LENGTH, RIB_WIDTH, RIB_HEIGHT),
        (RIB_X, 0.0, RIB_CENTER_Z),
        "carriage_silver",
    )
    _add_box_visual(
        carriage,
        (RIB_LENGTH, RIB_WIDTH, RIB_HEIGHT),
        (-RIB_X, 0.0, RIB_CENTER_Z),
        "carriage_silver",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((BRIDGE_LENGTH, BRIDGE_WIDTH, 0.055)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    model.articulation(
        "gantry_slide",
        ArticulationType.PRISMATIC,
        parent="base",
        child="carriage",
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_LIMIT,
            upper=SLIDE_LIMIT,
            effort=250.0,
            velocity=1.0,
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
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("carriage", "base", axes="xy", max_dist=0.005)
    ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_gap("carriage", "base", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "gantry_slide",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.10,
    )

    with ctx.pose(gantry_slide=-SLIDE_LIMIT):
        ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_gap("carriage", "base", axis="z", max_gap=0.003, max_penetration=0.0)

    with ctx.pose(gantry_slide=SLIDE_LIMIT):
        ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_gap("carriage", "base", axis="z", max_gap=0.003, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
