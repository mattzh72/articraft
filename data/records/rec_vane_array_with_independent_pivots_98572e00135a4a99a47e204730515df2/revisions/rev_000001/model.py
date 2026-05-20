from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 0.90
FRAME_DEPTH = 0.10
FRAME_HEIGHT = 0.64
INNER_WIDTH = 0.76
INNER_HEIGHT = 0.52

VANE_COUNT = 7
VANE_SPACING = 0.070
VANE_ZS = tuple((i - (VANE_COUNT - 1) / 2.0) * VANE_SPACING for i in range(VANE_COUNT))

BLADE_LENGTH = 0.68
BLADE_CHORD = 0.085
BLADE_THICKNESS = 0.014
BASE_PITCH = math.radians(22.0)

PIVOT_RADIUS = 0.008
PIVOT_HOLE_RADIUS = 0.0076
PIVOT_STUB_LENGTH = 0.10
BEARING_RADIUS = 0.024
BEARING_THICKNESS = 0.008


def _frame_shape() -> cq.Workplane:
    """One continuous frame with real central opening and pivot holes."""
    frame = cq.Workplane("XY").box(FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)
    opening = cq.Workplane("XY").box(INNER_WIDTH, FRAME_DEPTH + 0.04, INNER_HEIGHT)
    frame = frame.cut(opening)

    # Raised bearing collars on the inner side faces make each pivot support visible.
    for z in VANE_ZS:
        left_bearing = (
            cq.Workplane("YZ")
            .center(0.0, z)
            .circle(BEARING_RADIUS)
            .extrude(BEARING_THICKNESS)
            .translate((-INNER_WIDTH / 2.0, 0.0, 0.0))
        )
        right_bearing = (
            cq.Workplane("YZ")
            .center(0.0, z)
            .circle(BEARING_RADIUS)
            .extrude(BEARING_THICKNESS)
            .translate((INNER_WIDTH / 2.0 - BEARING_THICKNESS, 0.0, 0.0))
        )
        frame = frame.union(left_bearing).union(right_bearing)

    # Through-holes are slightly undersized to model tight captured bushings.
    for z in VANE_ZS:
        hole = (
            cq.Workplane("YZ")
            .center(0.0, z)
            .circle(PIVOT_HOLE_RADIUS)
            .extrude(FRAME_WIDTH + 0.10)
            .translate((-FRAME_WIDTH / 2.0 - 0.05, 0.0, 0.0))
        )
        frame = frame.cut(hole)

    return frame


def _airfoil_vane_shape() -> cq.Workplane:
    """Rounded louver blade, extruded along the hinge/pivot axis."""
    return (
        cq.Workplane("YZ")
        .ellipse(BLADE_CHORD / 2.0, BLADE_THICKNESS / 2.0)
        .extrude(BLADE_LENGTH)
        .translate((-BLADE_LENGTH / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louver_vane_bank")

    dark_anodized = model.material("dark_anodized_aluminum", rgba=(0.06, 0.07, 0.08, 1.0))
    satin_blade = model.material("satin_aluminum_vanes", rgba=(0.68, 0.72, 0.74, 1.0))
    black_bearing = model.material("black_bearing_stubs", rgba=(0.015, 0.015, 0.014, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "louver_frame", tolerance=0.0008),
        material=dark_anodized,
        name="frame_body",
    )

    vane_mesh = mesh_from_cadquery(_airfoil_vane_shape(), "rounded_vane_airfoil", tolerance=0.0005)
    stub_origin_rpy = (0.0, math.pi / 2.0, 0.0)

    for i, z in enumerate(VANE_ZS):
        vane = model.part(f"vane_{i}")
        vane.visual(
            vane_mesh,
            origin=Origin(rpy=(BASE_PITCH, 0.0, 0.0)),
            material=satin_blade,
            name="airfoil",
        )
        vane.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIVOT_STUB_LENGTH),
            origin=Origin(
                xyz=(-BLADE_LENGTH / 2.0 - PIVOT_STUB_LENGTH / 2.0, 0.0, 0.0),
                rpy=stub_origin_rpy,
            ),
            material=black_bearing,
            name="end_stub_0",
        )
        vane.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIVOT_STUB_LENGTH),
            origin=Origin(
                xyz=(BLADE_LENGTH / 2.0 + PIVOT_STUB_LENGTH / 2.0, 0.0, 0.0),
                rpy=stub_origin_rpy,
            ),
            material=black_bearing,
            name="end_stub_1",
        )
        model.articulation(
            f"frame_to_vane_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.45, upper=0.45, effort=1.5, velocity=1.2),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    vane_parts = [object_model.get_part(f"vane_{i}") for i in range(VANE_COUNT)]
    vane_joints = [object_model.get_articulation(f"frame_to_vane_{i}") for i in range(VANE_COUNT)]

    ctx.check("one independent revolute per vane", len(vane_joints) == VANE_COUNT)
    for i, joint in enumerate(vane_joints):
        limits = joint.motion_limits
        ctx.check(
            f"vane_{i} rotates about long axis",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(a, 6) for a in joint.axis) == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"axis={joint.axis}, limits={limits}",
        )
        ctx.expect_overlap(
            vane_parts[i],
            frame,
            axes="x",
            elem_a="end_stub_0",
            elem_b="frame_body",
            min_overlap=0.050,
            name=f"vane_{i} first pivot stub enters side frame",
        )
        ctx.allow_overlap(
            frame,
            vane_parts[i],
            elem_a="frame_body",
            elem_b="end_stub_0",
            reason="The pivot stub is intentionally captured in a tight bushing hole in the side frame.",
        )
        ctx.expect_overlap(
            vane_parts[i],
            frame,
            axes="x",
            elem_a="end_stub_1",
            elem_b="frame_body",
            min_overlap=0.050,
            name=f"vane_{i} second pivot stub enters side frame",
        )
        ctx.allow_overlap(
            frame,
            vane_parts[i],
            elem_a="frame_body",
            elem_b="end_stub_1",
            reason="The pivot stub is intentionally captured in a tight bushing hole in the side frame.",
        )

    for i in range(VANE_COUNT - 1):
        ctx.expect_gap(
            vane_parts[i + 1],
            vane_parts[i],
            axis="z",
            min_gap=0.010,
            positive_elem="airfoil",
            negative_elem="airfoil",
            name=f"vane_{i} and vane_{i + 1} are separated at rest",
        )

    driven = vane_parts[VANE_COUNT // 2]
    driven_joint = vane_joints[VANE_COUNT // 2]
    rest_aabb = ctx.part_element_world_aabb(driven, elem="airfoil")
    with ctx.pose({driven_joint: driven_joint.motion_limits.upper}):
        tilted_aabb = ctx.part_element_world_aabb(driven, elem="airfoil")
    if rest_aabb is not None and tilted_aabb is not None:
        rest_height = rest_aabb[1][2] - rest_aabb[0][2]
        tilted_height = tilted_aabb[1][2] - tilted_aabb[0][2]
        ctx.check(
            "vane rotation changes blade pitch",
            tilted_height > rest_height + 0.015,
            details=f"rest_height={rest_height:.4f}, tilted_height={tilted_height:.4f}",
        )
    else:
        ctx.fail("vane rotation changes blade pitch", "could not measure blade AABBs")

    return ctx.report()


object_model = build_object_model()
