from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CANOPY_SIZE = 0.24
CANOPY_THICKNESS = 0.024
CANOPY_SKIRT_SIZE = 0.18
CANOPY_SKIRT_THICKNESS = 0.012
STEM_RADIUS = 0.017
STEM_LENGTH = 0.10
HUB_RADIUS = 0.046
HUB_HEIGHT = 0.04
PAN_COLLAR_RADIUS = 0.012
PAN_COLLAR_HEIGHT = 0.02
PAN_ARM_LENGTH = 0.05
PAN_ARM_WIDTH = 0.028
PAN_ARM_THICKNESS = 0.014
YOKE_BRIDGE_LENGTH = 0.026
YOKE_BRIDGE_WIDTH = 0.08
YOKE_BRIDGE_THICKNESS = 0.014
YOKE_CHEEK_LENGTH = 0.014
YOKE_CHEEK_THICKNESS = 0.006
YOKE_CHEEK_HEIGHT = 0.044
YOKE_HALF_SPAN = 0.037
TILT_PIVOT_X = 0.08
TILT_PIVOT_Z = -0.022
HEAD_SHELL_RADIUS = 0.03
HEAD_INNER_RADIUS = 0.026
HEAD_SHELL_LENGTH = 0.084
HEAD_BEZEL_RADIUS = 0.034
HEAD_BEZEL_INNER_RADIUS = 0.028
HEAD_BEZEL_LENGTH = 0.012
HEAD_REAR_RADIUS = 0.022
HEAD_REAR_LENGTH = 0.014
HEAD_BODY_Z_OFFSET = -0.012
HEAD_BODY_X_OFFSET = -0.006
TRUNNION_RADIUS = 0.0048
TRUNNION_LENGTH = 0.068
LENS_RADIUS = 0.0285
LENS_THICKNESS = 0.002
LENS_CENTER_X = 0.083
PAN_LIMIT = math.radians(80.0)
TILT_LOWER = math.radians(-20.0)
TILT_UPPER = math.radians(66.0)


def _build_head_shell() -> cq.Workplane:
    shell_ring = (
        cq.Workplane("YZ")
        .circle(HEAD_SHELL_RADIUS)
        .circle(HEAD_INNER_RADIUS)
        .extrude(HEAD_SHELL_LENGTH)
    )
    front_bezel = (
        cq.Workplane("YZ", origin=(HEAD_SHELL_LENGTH, 0.0, 0.0))
        .circle(HEAD_BEZEL_RADIUS)
        .circle(HEAD_BEZEL_INNER_RADIUS)
        .extrude(HEAD_BEZEL_LENGTH)
    )
    rear_cap = cq.Workplane("YZ").circle(HEAD_REAR_RADIUS).extrude(HEAD_REAR_LENGTH)
    return shell_ring.union(front_bezel).union(rear_cap)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_spotlight_cluster")

    canopy_white = model.material("canopy_white", rgba=(0.94, 0.94, 0.95, 1.0))
    stem_white = model.material("stem_white", rgba=(0.88, 0.88, 0.89, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    lamp_black = model.material("lamp_black", rgba=(0.10, 0.10, 0.11, 1.0))
    lens_smoke = model.material("lens_smoke", rgba=(0.42, 0.45, 0.48, 0.78))

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Box((CANOPY_SIZE, CANOPY_SIZE, CANOPY_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=canopy_white,
        name="canopy_shell",
    )
    ceiling_mount.visual(
        Box((CANOPY_SKIRT_SIZE, CANOPY_SKIRT_SIZE, CANOPY_SKIRT_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=canopy_white,
        name="canopy_skirt",
    )
    ceiling_mount.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=stem_white,
        name="drop_stem",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=graphite,
        name="stem_collar",
    )
    ceiling_mount.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="hub_shell",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.03, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=graphite,
        name="hub_cap",
    )

    head_shell_mesh = mesh_from_cadquery(_build_head_shell(), "spotlight_head_shell")

    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        pan_x = (HUB_RADIUS + PAN_COLLAR_RADIUS) * math.cos(yaw)
        pan_y = (HUB_RADIUS + PAN_COLLAR_RADIUS) * math.sin(yaw)
        pan = model.part(f"pan_{index}")
        pan.visual(
            Cylinder(radius=PAN_COLLAR_RADIUS, length=PAN_COLLAR_HEIGHT),
            material=graphite,
            name="pan_collar",
        )
        pan.visual(
            Box((PAN_ARM_LENGTH, PAN_ARM_WIDTH, PAN_ARM_THICKNESS)),
            origin=Origin(xyz=(0.028, 0.0, -0.007)),
            material=graphite,
            name="pan_arm",
        )
        pan.visual(
            Box((YOKE_BRIDGE_LENGTH, YOKE_BRIDGE_WIDTH, YOKE_BRIDGE_THICKNESS)),
            origin=Origin(xyz=(0.060, 0.0, -0.007)),
            material=graphite,
            name="yoke_bridge",
        )
        pan.visual(
            Box((YOKE_CHEEK_LENGTH, YOKE_CHEEK_THICKNESS, YOKE_CHEEK_HEIGHT)),
            origin=Origin(xyz=(TILT_PIVOT_X, YOKE_HALF_SPAN, TILT_PIVOT_Z)),
            material=graphite,
            name="yoke_cheek_0",
        )
        pan.visual(
            Box((YOKE_CHEEK_LENGTH, YOKE_CHEEK_THICKNESS, YOKE_CHEEK_HEIGHT)),
            origin=Origin(xyz=(TILT_PIVOT_X, -YOKE_HALF_SPAN, TILT_PIVOT_Z)),
            material=graphite,
            name="yoke_cheek_1",
        )

        head = model.part(f"head_{index}")
        head.visual(
            head_shell_mesh,
            origin=Origin(xyz=(HEAD_BODY_X_OFFSET, 0.0, HEAD_BODY_Z_OFFSET)),
            material=lamp_black,
            name="body_shell",
        )
        head.visual(
            Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="trunnion_pin",
        )
        head.visual(
            Cylinder(radius=LENS_RADIUS, length=LENS_THICKNESS),
            origin=Origin(
                xyz=(LENS_CENTER_X, 0.0, HEAD_BODY_Z_OFFSET),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=lens_smoke,
            name="front_lens",
        )

        model.articulation(
            f"mount_to_pan_{index}",
            ArticulationType.REVOLUTE,
            parent=ceiling_mount,
            child=pan,
            origin=Origin(xyz=(pan_x, pan_y, 0.0), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=2.2,
                lower=-PAN_LIMIT,
                upper=PAN_LIMIT,
            ),
        )
        model.articulation(
            f"pan_{index}_to_head_{index}",
            ArticulationType.REVOLUTE,
            parent=pan,
            child=head,
            origin=Origin(xyz=(TILT_PIVOT_X, 0.0, TILT_PIVOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.5,
                lower=TILT_LOWER,
                upper=TILT_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ceiling_mount = object_model.get_part("ceiling_mount")

    for index in range(4):
        pan = object_model.get_part(f"pan_{index}")
        head = object_model.get_part(f"head_{index}")

        ctx.expect_contact(
            pan,
            ceiling_mount,
            elem_a="pan_collar",
            elem_b="hub_shell",
            contact_tol=0.0015,
            name=f"pan_{index} collar meets the hub",
        )
        ctx.expect_gap(
            ceiling_mount,
            head,
            axis="z",
            positive_elem="canopy_shell",
            negative_elem="body_shell",
            min_gap=0.04,
            name=f"head_{index} hangs beneath the canopy",
        )

    rest_lens = _aabb_center(ctx.part_element_world_aabb("head_0", elem="front_lens"))

    with ctx.pose(mount_to_pan_0=math.radians(40.0)):
        panned_lens = _aabb_center(ctx.part_element_world_aabb("head_0", elem="front_lens"))

    ctx.check(
        "head_0 pans around the hub",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[1] + 0.05
        and panned_lens[0] > 0.12,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    with ctx.pose(pan_0_to_head_0=TILT_UPPER):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb("head_0", elem="front_lens"))

    ctx.check(
        "head_0 tilts downward from the yoke",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] < rest_lens[2] - 0.05,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
