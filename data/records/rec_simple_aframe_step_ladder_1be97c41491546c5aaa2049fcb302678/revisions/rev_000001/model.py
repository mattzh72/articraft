from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HINGE_X = 0.065
HINGE_Z = 1.38


def _origin_between(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Origin for a box/cylinder whose local +Z axis runs from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # All structural members in this model lie in XZ planes or run along Y, so
    # a pitch-only orientation is sufficient for the sloped ladder rails.
    pitch = math.atan2(dx, dz)
    return (
        Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        length,
    )


def _add_sloped_box(part, name: str, p0: tuple[float, float, float], p1: tuple[float, float, float], section: float, material: Material) -> None:
    origin, length = _origin_between(p0, p1)
    part.visual(Box((section, section, length)), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_aluminum = model.material("dark_anodized_edges", rgba=(0.32, 0.34, 0.34, 1.0))
    blue_plastic = model.material("blue_plastic_cap", rgba=(0.05, 0.22, 0.80, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.55, 0.56, 0.54, 1.0))

    front = model.part("front_frame")
    rail_y = 0.315
    front_bottom_x = -0.48
    front_bottom_z = 0.045
    front_top_x = -0.045
    front_top_z = 1.35

    for idx, y in enumerate((-rail_y, rail_y)):
        _add_sloped_box(
            front,
            f"front_rail_{idx}",
            (front_bottom_x, y, front_bottom_z),
            (front_top_x, y, front_top_z),
            0.052,
            aluminum,
        )

    front.visual(
        Box((0.24, 0.82, 0.052)),
        origin=Origin(xyz=(front_bottom_x - 0.01, 0.0, 0.026)),
        material=black_rubber,
        name="front_foot_bar",
    )

    tread_specs = (
        (0.34, 0.150),
        (0.64, 0.170),
        (0.94, 0.190),
    )
    for tread_idx, (z, depth) in enumerate(tread_specs):
        x = front_bottom_x + (z - front_bottom_z) / (front_top_z - front_bottom_z) * (front_top_x - front_bottom_x)
        front.visual(
            Box((depth, 0.76, 0.046)),
            origin=Origin(xyz=(x - 0.015, 0.0, z)),
            material=aluminum,
            name=f"tread_{tread_idx}",
        )
        for strip_idx, strip_x in enumerate((-depth * 0.25, depth * 0.18)):
            front.visual(
                Box((0.016, 0.68, 0.010)),
                origin=Origin(xyz=(x + strip_x, 0.0, z + 0.028)),
                material=dark_aluminum,
                name=f"tread_strip_{tread_idx}_{strip_idx}",
            )

    front.visual(
        Box((0.265, 0.80, 0.080)),
        origin=Origin(xyz=(-0.095, 0.0, HINGE_Z)),
        material=blue_plastic,
        name="top_cap",
    )
    front.visual(
        Box((0.070, 0.140, 0.060)),
        origin=Origin(xyz=(0.040, -0.280, HINGE_Z)),
        material=blue_plastic,
        name="hinge_lug_0",
    )
    front.visual(
        Box((0.070, 0.140, 0.060)),
        origin=Origin(xyz=(0.040, 0.280, HINGE_Z)),
        material=blue_plastic,
        name="hinge_lug_1",
    )
    # Interleaved fixed knuckles on the climbing frame; the rear frame carries
    # the center knuckle and rotates about the same axis.
    front.visual(
        Cylinder(radius=0.027, length=0.160),
        origin=Origin(xyz=(HINGE_X, -0.295, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="front_hinge_knuckle_0",
    )
    front.visual(
        Cylinder(radius=0.027, length=0.160),
        origin=Origin(xyz=(HINGE_X, 0.295, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="front_hinge_knuckle_1",
    )

    rear = model.part("rear_frame")
    rear_bottom_x = 0.52
    rear_bottom_z = -1.33
    rear_top_x = 0.042
    rear_top_z = -0.095
    for idx, y in enumerate((-rail_y, rail_y)):
        _add_sloped_box(
            rear,
            f"rear_leg_{idx}",
            (rear_bottom_x, y, rear_bottom_z),
            (rear_top_x, y, rear_top_z),
            0.048,
            aluminum,
        )

    rear.visual(
        Box((0.210, 0.82, 0.050)),
        origin=Origin(xyz=(rear_bottom_x + 0.015, 0.0, rear_bottom_z + 0.002)),
        material=black_rubber,
        name="rear_foot_bar",
    )
    rear.visual(
        Box((0.055, 0.73, 0.046)),
        origin=Origin(xyz=(0.038, 0.0, -0.100)),
        material=aluminum,
        name="rear_top_bar",
    )
    for brace_idx, z in enumerate((-0.50, -0.88)):
        x = rear_top_x + (z - rear_top_z) / (rear_bottom_z - rear_top_z) * (rear_bottom_x - rear_top_x)
        rear.visual(
            Box((0.050, 0.70, 0.040)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"rear_cross_bar_{brace_idx}",
        )

    rear.visual(
        Cylinder(radius=0.025, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="rear_hinge_knuckle",
    )
    rear.visual(
        Box((0.050, 0.180, 0.095)),
        origin=Origin(xyz=(0.012, 0.0, -0.052)),
        material=hinge_steel,
        name="rear_hinge_web",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    ctx.check(
        "single rear-frame hinge",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.parent == "front_frame"
        and hinge.child == "rear_frame",
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            rear,
            front,
            axis="x",
            positive_elem="rear_foot_bar",
            negative_elem="front_foot_bar",
            min_gap=0.55,
            name="open A-frame foot spread",
        )
        ctx.expect_overlap(
            rear,
            front,
            axes="y",
            elem_a="rear_foot_bar",
            elem_b="front_foot_bar",
            min_overlap=0.65,
            name="front and rear frames share centered width",
        )

        left = ctx.part_element_world_aabb(front, elem="front_rail_0")
        right = ctx.part_element_world_aabb(front, elem="front_rail_1")
        rear_left = ctx.part_element_world_aabb(rear, elem="rear_leg_0")
        rear_right = ctx.part_element_world_aabb(rear, elem="rear_leg_1")
        symmetric = (
            left is not None
            and right is not None
            and rear_left is not None
            and rear_right is not None
            and abs(left[0][1] + right[1][1]) < 0.004
            and abs(left[1][1] + right[0][1]) < 0.004
            and abs(rear_left[0][1] + rear_right[1][1]) < 0.004
            and abs(rear_left[1][1] + rear_right[0][1]) < 0.004
        )
        ctx.check("open frame is symmetric about center plane", symmetric)

    rest_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_bar")
    rest_x = None if rest_foot is None else (rest_foot[0][0] + rest_foot[1][0]) * 0.5
    with ctx.pose({hinge: 0.50}):
        folded_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_bar")
        folded_x = None if folded_foot is None else (folded_foot[0][0] + folded_foot[1][0]) * 0.5

    ctx.check(
        "rear frame folds toward climbing frame",
        rest_x is not None and folded_x is not None and folded_x < rest_x - 0.55,
        details=f"rest_x={rest_x}, folded_x={folded_x}",
    )

    return ctx.report()


object_model = build_object_model()
