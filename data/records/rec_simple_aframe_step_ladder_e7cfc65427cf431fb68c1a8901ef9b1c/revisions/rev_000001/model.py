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


def _member_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> Origin:
    """Return a transform whose local +Z axis runs from start to end."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    horizontal = math.hypot(dx, dy)
    pitch = math.atan2(horizontal, dz)
    yaw = math.atan2(dy, dx) if horizontal > 1.0e-9 else 0.0
    return Origin(
        xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
        rpy=(0.0, pitch, yaw),
    )


def _member_length(start: tuple[float, float, float], end: tuple[float, float, float]) -> float:
    return math.dist(start, end)


def _add_box_member(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness: tuple[float, float] = (0.055, 0.045),
    material: Material | str | None = None,
) -> None:
    part.visual(
        Box((thickness[0], thickness[1], _member_length(start, end))),
        origin=_member_origin(start, end),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = Material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    tread_mat = Material("ribbed_dark_tread", rgba=(0.035, 0.040, 0.040, 1.0))
    rubber = Material("black_rubber_feet", rgba=(0.015, 0.015, 0.014, 1.0))
    cap_mat = Material("blue_polymer_cap", rgba=(0.08, 0.20, 0.62, 1.0))
    hinge_mat = Material("dark_hinge_pin", rgba=(0.16, 0.16, 0.15, 1.0))
    for mat in (aluminum, tread_mat, rubber, cap_mat, hinge_mat):
        model.material(mat)

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    hinge_xyz = (0.20, 0.0, 1.22)

    # Front climbing frame: two leaning side rails with wide, horizontal treads.
    front.visual(
        Box((0.45, 0.72, 0.08)),
        origin=Origin(xyz=(-0.055, 0.0, 1.22)),
        material=cap_mat,
        name="top_cap",
    )
    front.visual(
        Box((0.11, 0.14, 0.036)),
        origin=Origin(xyz=(0.165, -0.245, 1.22)),
        material=aluminum,
        name="hinge_lug_0",
    )
    front.visual(
        Box((0.11, 0.14, 0.036)),
        origin=Origin(xyz=(0.165, 0.245, 1.22)),
        material=aluminum,
        name="hinge_lug_1",
    )
    front.visual(
        Cylinder(radius=0.035, length=0.19),
        origin=Origin(xyz=hinge_xyz[:1] + (-0.245,) + hinge_xyz[2:], rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="front_hinge_barrel_0",
    )
    front.visual(
        Cylinder(radius=0.035, length=0.19),
        origin=Origin(xyz=hinge_xyz[:1] + (0.245,) + hinge_xyz[2:], rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="front_hinge_barrel_1",
    )

    for y, suffix in ((-0.28, "0"), (0.28, "1")):
        _add_box_member(
            front,
            f"front_side_rail_{suffix}",
            start=(-0.52, y, 0.055),
            end=(-0.12, y, 1.185),
            thickness=(0.058, 0.050),
            material=aluminum,
        )

    front.visual(
        Box((0.18, 0.78, 0.052)),
        origin=Origin(xyz=(-0.52, 0.0, 0.026)),
        material=rubber,
        name="front_foot_bar",
    )
    front.visual(
        Box((0.09, 0.66, 0.045)),
        origin=Origin(xyz=(-0.45, 0.0, 0.18)),
        material=aluminum,
        name="front_lower_spreader",
    )

    tread_specs = (
        (0.28, -0.42, "0"),
        (0.55, -0.325, "1"),
        (0.82, -0.23, "2"),
    )
    for z, x, suffix in tread_specs:
        front.visual(
            Box((0.30, 0.66, 0.055)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"wide_tread_{suffix}",
        )
        for strip_y in (-0.18, 0.0, 0.18):
            front.visual(
                Box((0.265, 0.045, 0.010)),
                origin=Origin(xyz=(x - 0.01, strip_y, z + 0.031)),
                material=tread_mat,
                name=f"grip_strip_{suffix}_{int((strip_y + 0.18) / 0.18)}",
            )

    # Rear frame: broad splayed support legs, a lower stabilizer, and a central
    # hinge knuckle that folds about the same top-cap line.
    rear.visual(
        Cylinder(radius=0.032, length=0.22),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="rear_hinge_barrel",
    )
    rear.visual(
        Box((0.10, 0.68, 0.044)),
        origin=Origin(xyz=(0.120, 0.0, -0.085)),
        material=aluminum,
        name="rear_top_crossbar",
    )
    rear.visual(
        Box((0.080, 0.18, 0.050)),
        origin=Origin(xyz=(0.045, 0.0, -0.045)),
        material=aluminum,
        name="rear_hinge_strap",
    )

    rear_leg_tops = ((0.120, -0.275, -0.085), (0.120, 0.275, -0.085))
    rear_leg_feet = ((0.72, -0.36, -1.175), (0.72, 0.36, -1.175))
    for idx, (top, foot) in enumerate(zip(rear_leg_tops, rear_leg_feet)):
        _add_box_member(
            rear,
            f"rear_support_rail_{idx}",
            start=top,
            end=foot,
            thickness=(0.055, 0.048),
            material=aluminum,
        )

    rear.visual(
        Box((0.11, 0.82, 0.048)),
        origin=Origin(xyz=(0.58, 0.0, -0.96)),
        material=aluminum,
        name="rear_lower_spreader",
    )
    rear.visual(
        Box((0.18, 0.86, 0.055)),
        origin=Origin(xyz=(0.72, 0.0, -1.18)),
        material=rubber,
        name="rear_foot_bar",
    )

    model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=hinge_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=0.0, upper=0.46),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("front_to_rear")

    ctx.check(
        "single revolute top hinge",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_gap(
        rear,
        front,
        axis="x",
        min_gap=1.10,
        positive_elem="rear_foot_bar",
        negative_elem="front_foot_bar",
        name="lower supports have wide spread stance",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="xz",
        min_overlap=0.045,
        elem_a="front_hinge_barrel_0",
        elem_b="rear_hinge_barrel",
        name="rear hinge barrel shares top cap hinge line",
    )

    def _element_center_x(part, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_rear_foot_x = _element_center_x(rear, "rear_foot_bar")
    with ctx.pose({hinge: 0.42}):
        folded_rear_foot_x = _element_center_x(rear, "rear_foot_bar")
    ctx.check(
        "rear frame folds toward front frame",
        rest_rear_foot_x is not None
        and folded_rear_foot_x is not None
        and folded_rear_foot_x < rest_rear_foot_x - 0.35,
        details=f"rest_x={rest_rear_foot_x}, folded_x={folded_rear_foot_x}",
    )

    return ctx.report()


object_model = build_object_model()
