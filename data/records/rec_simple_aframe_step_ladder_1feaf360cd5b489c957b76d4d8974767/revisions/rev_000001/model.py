from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_axis(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis(a, b)),
        material=material,
        name=name,
    )


def _front_rail_point(z: float) -> tuple[float, float]:
    hinge_z = 1.72
    foot_z = 0.03
    t = (hinge_z - z) / (hinge_z - foot_z)
    half_x = 0.16 + (0.24 - 0.16) * t
    y = 0.02 + (0.21 - 0.02) * t
    return (half_x, y)


def _rear_rail_point(z_local: float) -> tuple[float, float]:
    top_z = -0.03
    foot_z = -1.69
    t = (top_z - z_local) / (top_z - foot_z)
    half_x = 0.11 + (0.20 - 0.11) * t
    y = -0.025 + (-0.29 + 0.025) * t
    return (half_x, y)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.81, 0.83, 1.0))
    step_aluminum = model.material("step_aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    top_cap_plastic = model.material("top_cap_plastic", rgba=(0.16, 0.16, 0.18, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.56, 0.57, 0.60, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    rail_width = 0.045
    rail_depth = 0.020

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.55, 1.76)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.10, 0.88)),
    )

    top_cap_center = (0.0, 0.02, 1.74)
    front_frame.visual(
        Box((0.42, 0.22, 0.06)),
        origin=Origin(xyz=top_cap_center),
        material=top_cap_plastic,
        name="top_cap_body",
    )
    front_frame.visual(
        Box((0.085, 0.12, 0.04)),
        origin=Origin(xyz=(-0.12, -0.09, 1.71)),
        material=top_cap_plastic,
        name="left_hinge_cheek",
    )
    front_frame.visual(
        Box((0.085, 0.12, 0.04)),
        origin=Origin(xyz=(0.12, -0.09, 1.71)),
        material=top_cap_plastic,
        name="right_hinge_cheek",
    )
    front_frame.visual(
        Box((0.34, 0.11, 0.018)),
        origin=Origin(xyz=(0.0, 0.08, 1.705)),
        material=step_aluminum,
        name="top_platform",
    )

    left_front_top = (-0.16, 0.02, 1.71)
    left_front_bottom = (-0.24, 0.21, 0.02)
    right_front_top = (0.16, 0.02, 1.71)
    right_front_bottom = (0.24, 0.21, 0.02)
    _add_box_beam(
        front_frame,
        left_front_top,
        left_front_bottom,
        width=rail_width,
        depth=rail_depth,
        material=aluminum,
        name="left_front_rail",
    )
    _add_box_beam(
        front_frame,
        right_front_top,
        right_front_bottom,
        width=rail_width,
        depth=rail_depth,
        material=aluminum,
        name="right_front_rail",
    )

    for index, z in enumerate((1.20, 0.90, 0.60, 0.30), start=1):
        half_x, y = _front_rail_point(z)
        front_frame.visual(
            Box((half_x * 2.0 + rail_width * 0.65, 0.11, 0.025)),
            origin=Origin(xyz=(0.0, y - 0.01, z)),
            material=step_aluminum,
            name=f"step_{index}",
        )

    brace_half_x, brace_y = _front_rail_point(0.14)
    front_frame.visual(
        Box((brace_half_x * 2.0 + rail_width * 0.75, 0.050, 0.025)),
        origin=Origin(xyz=(0.0, brace_y, 0.14)),
        material=aluminum,
        name="lower_front_brace",
    )

    front_frame.visual(
        Cylinder(radius=0.013, length=0.10),
        origin=Origin(xyz=(-0.12, -0.11, 1.72), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_hinge_barrel",
    )
    front_frame.visual(
        Cylinder(radius=0.013, length=0.10),
        origin=Origin(xyz=(0.12, -0.11, 1.72), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_hinge_barrel",
    )
    front_frame.visual(
        Box((0.070, 0.065, 0.020)),
        origin=Origin(xyz=(-0.24, 0.21, 0.01)),
        material=foot_rubber,
        name="front_left_foot",
    )
    front_frame.visual(
        Box((0.070, 0.065, 0.020)),
        origin=Origin(xyz=(0.24, 0.21, 0.01)),
        material=foot_rubber,
        name="front_right_foot",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.40, 1.72)),
        mass=4.6,
        origin=Origin(xyz=(0.0, -0.17, -0.86)),
    )

    rear_frame.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="rear_hinge_barrel",
    )
    rear_frame.visual(
        Box((0.10, 0.025, 0.060)),
        origin=Origin(xyz=(0.0, -0.015, -0.03)),
        material=hinge_steel,
        name="rear_hinge_leaf",
    )
    rear_frame.visual(
        Box((0.30, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.04, -0.06)),
        material=aluminum,
        name="rear_top_bridge",
    )

    left_rear_top = (-0.11, -0.025, -0.03)
    left_rear_bottom = (-0.20, -0.29, -1.69)
    right_rear_top = (0.11, -0.025, -0.03)
    right_rear_bottom = (0.20, -0.29, -1.69)
    _add_box_beam(
        rear_frame,
        left_rear_top,
        left_rear_bottom,
        width=0.036,
        depth=0.018,
        material=aluminum,
        name="left_rear_rail",
    )
    _add_box_beam(
        rear_frame,
        right_rear_top,
        right_rear_bottom,
        width=0.036,
        depth=0.018,
        material=aluminum,
        name="right_rear_rail",
    )

    for index, z in enumerate((-0.46, -0.88, -1.28), start=1):
        half_x, y = _rear_rail_point(z)
        rear_frame.visual(
            Box((half_x * 2.0 + 0.030, 0.035, 0.020)),
            origin=Origin(xyz=(0.0, y, z)),
            material=aluminum,
            name=f"rear_spreader_{index}",
        )

    rear_brace_half_x, rear_brace_y = _rear_rail_point(-1.57)
    rear_frame.visual(
        Box((rear_brace_half_x * 2.0 + 0.030, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, rear_brace_y, -1.57)),
        material=aluminum,
        name="rear_lower_brace",
    )
    rear_frame.visual(
        Box((0.060, 0.055, 0.020)),
        origin=Origin(xyz=(-0.20, -0.29, -1.70)),
        material=foot_rubber,
        name="rear_left_foot",
    )
    rear_frame.visual(
        Box((0.060, 0.055, 0.020)),
        origin=Origin(xyz=(0.20, -0.29, -1.70)),
        material=foot_rubber,
        name="rear_right_foot",
    )

    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.11, 1.72)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.10, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    rear_hinge = object_model.get_articulation("rear_frame_hinge")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="front_left_foot",
        negative_elem="rear_left_foot",
        min_gap=0.42,
        max_gap=0.62,
        name="open stance keeps rear feet behind the climbing frame",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="x",
        elem_a="top_cap_body",
        elem_b="rear_top_bridge",
        min_overlap=0.20,
        name="rear frame stays centered beneath the top cap",
    )

    step_aabb = ctx.part_element_world_aabb(front_frame, elem="step_2")
    if step_aabb is None:
        ctx.fail("second tread exists", "step_2 AABB could not be resolved")
    else:
        tread_width = step_aabb[1][0] - step_aabb[0][0]
        tread_depth = step_aabb[1][1] - step_aabb[0][1]
        ctx.check(
            "front treads are wide and usable",
            tread_width >= 0.34 and tread_depth >= 0.10,
            details=f"width={tread_width:.3f}, depth={tread_depth:.3f}",
        )

    top_cap_aabb = ctx.part_element_world_aabb(front_frame, elem="top_cap_body")
    if top_cap_aabb is None:
        ctx.fail("top cap height exists", "top_cap_body AABB could not be resolved")
    else:
        ctx.check(
            "ladder has a tall working height",
            top_cap_aabb[1][2] >= 1.75,
            details=f"top_cap_max_z={top_cap_aabb[1][2]:.3f}",
        )

    rear_foot_rest = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    with ctx.pose({rear_hinge: 0.28}):
        rear_foot_folded = ctx.part_element_world_aabb(rear_frame, elem="rear_left_foot")
    if rear_foot_rest is None or rear_foot_folded is None:
        ctx.fail("rear frame fold pose available", "rear foot AABB missing in one of the test poses")
    else:
        rest_y = 0.5 * (rear_foot_rest[0][1] + rear_foot_rest[1][1])
        folded_y = 0.5 * (rear_foot_folded[0][1] + rear_foot_folded[1][1])
        ctx.check(
            "rear support folds forward about the top hinge",
            folded_y > rest_y + 0.40,
            details=f"rest_y={rest_y:.3f}, folded_y={folded_y:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
