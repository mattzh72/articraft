from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
HUB_RADIUS = 0.046
HUB_HEIGHT = 0.054
HUB_RING_OUTER_RADIUS = 0.034
HUB_RING_INNER_RADIUS = 0.024
HUB_RING_HEIGHT = 0.008
SPINDLE_RADIUS = 0.017
SPINDLE_HEIGHT = 0.072

LUG_LENGTH = 0.034
LUG_WIDTH = 0.026
LUG_HEIGHT = 0.022
LUG_OFFSET_X = HUB_RADIUS + 0.5 * LUG_LENGTH - 0.008

JOINT_RADIUS = 0.058
BRANCH_ANGLES = (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)
BRANCH_NAMES = ("arm_a", "arm_b", "arm_c")

ARM_BARREL_RADIUS = 0.015
ARM_BARREL_LENGTH = 0.028
ARM_SHOULDER_LENGTH = 0.032
ARM_SHOULDER_WIDTH = 0.026
ARM_SHOULDER_HEIGHT = 0.018
ARM_SHOULDER_CENTER_X = 0.022
ARM_BODY_LENGTH = 0.118
ARM_BODY_WIDTH = 0.018
ARM_BODY_HEIGHT = 0.012
ARM_BODY_CENTER_X = 0.092
ARM_TIP_PAD_LENGTH = 0.026
ARM_TIP_PAD_WIDTH = 0.030
ARM_TIP_PAD_HEIGHT = 0.008
ARM_TIP_PAD_CENTER_X = 0.145
ARM_TIP_RADIUS = 0.014
ARM_TIP_X = 0.160
ARM_INERTIAL_LENGTH = 0.156
ARM_INERTIAL_WIDTH = 0.030
ARM_INERTIAL_HEIGHT = 0.022
ARM_INERTIAL_CENTER_X = 0.082

ARM_LIMITS = MotionLimits(lower=-0.65, upper=0.90, effort=8.0, velocity=1.5)


def _build_hub_shape() -> cq.Workplane:
    core = (
        cq.Workplane("XY")
        .circle(HUB_RADIUS)
        .extrude(HUB_HEIGHT)
        .translate((0.0, 0.0, -0.5 * HUB_HEIGHT))
    )
    spindle = (
        cq.Workplane("XY")
        .circle(SPINDLE_RADIUS)
        .extrude(SPINDLE_HEIGHT)
        .translate((0.0, 0.0, -0.5 * SPINDLE_HEIGHT))
    )
    ring = (
        cq.Workplane("XY")
        .circle(HUB_RING_OUTER_RADIUS)
        .circle(HUB_RING_INNER_RADIUS)
        .extrude(HUB_RING_HEIGHT)
        .translate((0.0, 0.0, -0.5 * HUB_RING_HEIGHT))
    )
    lug = (
        cq.Workplane("XY")
        .box(LUG_LENGTH, LUG_WIDTH, LUG_HEIGHT)
        .translate((LUG_OFFSET_X, 0.0, 0.0))
    )

    hub = core.union(spindle).union(ring)
    for angle_deg in (0.0, 120.0, 240.0):
        hub = hub.union(lug.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))
    return hub


def _build_arm_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .circle(ARM_BARREL_RADIUS)
        .extrude(ARM_BARREL_LENGTH)
        .translate((0.0, 0.0, -0.5 * ARM_BARREL_LENGTH))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )
    shoulder = (
        cq.Workplane("XY")
        .box(
            ARM_SHOULDER_LENGTH,
            ARM_SHOULDER_WIDTH,
            ARM_SHOULDER_HEIGHT,
        )
        .translate((ARM_SHOULDER_CENTER_X, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(
            ARM_BODY_LENGTH,
            ARM_BODY_WIDTH,
            ARM_BODY_HEIGHT,
        )
        .translate((ARM_BODY_CENTER_X, 0.0, 0.0))
    )
    tip_pad = (
        cq.Workplane("XY")
        .box(
            ARM_TIP_PAD_LENGTH,
            ARM_TIP_PAD_WIDTH,
            ARM_TIP_PAD_HEIGHT,
        )
        .translate((ARM_TIP_PAD_CENTER_X, 0.0, 0.0))
    )
    tip = cq.Workplane("XY").sphere(ARM_TIP_RADIUS).translate((ARM_TIP_X, 0.0, 0.0))

    return barrel.union(shoulder).union(beam).union(tip_pad).union(tip)


def _add_branch_part(model: ArticulatedObject, mesh, name: str):
    branch = model.part(name)
    branch.visual(mesh, material="arm_blue")





    branch.inertial = Inertial.from_geometry(
        Box((ARM_INERTIAL_LENGTH, ARM_INERTIAL_WIDTH, ARM_INERTIAL_HEIGHT)),
        mass=0.24,
        origin=Origin(xyz=(ARM_INERTIAL_CENTER_X, 0.0, 0.0)),
    )
    return branch


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_tree_mechanism", assets=ASSETS)

    model.material("hub_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("arm_blue", rgba=(0.23, 0.45, 0.72, 1.0))

    hub_mesh = mesh_from_cadquery(_build_hub_shape(), "rotary_tree_hub.obj", assets=ASSETS)
    arm_mesh = mesh_from_cadquery(_build_arm_shape(), "rotary_tree_arm.obj", assets=ASSETS)

    hub = model.part("hub")
    hub.visual(hub_mesh, material="hub_gray")


    for angle in BRANCH_ANGLES:
        pass
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        mass=1.15,
    )

    for name, angle in zip(BRANCH_NAMES, BRANCH_ANGLES):
        branch = _add_branch_part(model, arm_mesh, name)
        model.articulation(
            f"hub_to_{name}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=branch,
            origin=Origin(
                xyz=(JOINT_RADIUS * cos(angle), JOINT_RADIUS * sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=ARM_LIMITS,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.003, overlap_volume_tol=0.0)

    for name in BRANCH_NAMES:
        ctx.expect_origin_distance(name, "hub", axes="xy", max_dist=0.12)
        ctx.expect_aabb_gap(name, "hub", axis="z", max_gap=0.04, max_penetration=0.055)
        ctx.expect_joint_motion_axis(
            f"hub_to_{name}",
            name,
            world_axis="z",
            direction="positive",
            min_delta=0.012,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
