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


def _rpy_for_segment(
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
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _front_y_for_z(z: float) -> float:
    top_y = 0.015
    top_z = 0.93
    bottom_y = 0.24
    bottom_z = 0.035
    t = (top_z - z) / (top_z - bottom_z)
    return top_y + (bottom_y - top_y) * t


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.83, 0.84, 0.86, 1.0))
    tread_aluminum = model.material("tread_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    cap_plastic = model.material("cap_plastic", rgba=(0.16, 0.18, 0.20, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.35, 0.37, 0.40, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.32, 1.02)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.13, 0.51)),
    )

    left_front_top = (-0.155, 0.015, 0.935)
    left_front_bottom = (-0.180, 0.240, 0.040)
    right_front_top = (0.155, 0.015, 0.935)
    right_front_bottom = (0.180, 0.240, 0.040)

    _add_box_beam(
        front_frame,
        left_front_top,
        left_front_bottom,
        width=0.030,
        depth=0.060,
        material=aluminum,
        name="left_front_rail",
    )
    _add_box_beam(
        front_frame,
        right_front_top,
        right_front_bottom,
        width=0.030,
        depth=0.060,
        material=aluminum,
        name="right_front_rail",
    )

    front_frame.visual(
        Box((0.360, 0.120, 0.055)),
        origin=Origin(xyz=(0.0, 0.060, 0.955)),
        material=cap_plastic,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.315, 0.095, 0.018)),
        origin=Origin(xyz=(0.0, 0.058, 0.992)),
        material=cap_plastic,
        name="top_tray",
    )

    front_frame.visual(
        Box((0.016, 0.046, 0.076)),
        origin=Origin(xyz=(-0.060, -0.010, 0.957)),
        material=hinge_steel,
        name="left_front_hinge_plate",
    )
    front_frame.visual(
        Box((0.016, 0.046, 0.076)),
        origin=Origin(xyz=(0.060, -0.010, 0.957)),
        material=hinge_steel,
        name="right_front_hinge_plate",
    )
    front_frame.visual(
        Cylinder(radius=0.017, length=0.170),
        origin=Origin(xyz=(0.0, -0.016, 0.995), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="front_hinge_barrel",
    )

    for index, step_z in enumerate((0.785, 0.595, 0.405, 0.215), start=1):
        step_y = _front_y_for_z(step_z)
        front_frame.visual(
            Box((0.314, 0.092, 0.026)),
            origin=Origin(xyz=(0.0, step_y, step_z)),
            material=tread_aluminum,
            name=f"tread_{index}",
        )
        front_frame.visual(
            Box((0.314, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, step_y + 0.036, step_z + 0.010)),
            material=hinge_steel,
            name=f"tread_{index}_front_lip",
        )

    front_frame.visual(
        Box((0.092, 0.040, 0.040)),
        origin=Origin(xyz=(-0.180, 0.242, 0.020)),
        material=rubber,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.092, 0.040, 0.040)),
        origin=Origin(xyz=(0.180, 0.242, 0.020)),
        material=rubber,
        name="right_front_foot",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.40, 0.26, 0.98)),
        mass=4.4,
        origin=Origin(xyz=(0.0, -0.12, -0.49)),
    )

    left_rear_top = (-0.145, -0.030, -0.050)
    left_rear_bottom = (-0.170, -0.220, -0.958)
    right_rear_top = (0.145, -0.030, -0.050)
    right_rear_bottom = (0.170, -0.220, -0.958)

    rear_frame.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(-0.120, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_rear_hinge_barrel",
    )
    rear_frame.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_rear_hinge_barrel",
    )
    rear_frame.visual(
        Box((0.018, 0.052, 0.072)),
        origin=Origin(xyz=(-0.120, -0.020, -0.034)),
        material=hinge_steel,
        name="left_rear_hinge_plate",
    )
    rear_frame.visual(
        Box((0.018, 0.052, 0.072)),
        origin=Origin(xyz=(0.120, -0.020, -0.034)),
        material=hinge_steel,
        name="right_rear_hinge_plate",
    )
    rear_frame.visual(
        Box((0.288, 0.042, 0.028)),
        origin=Origin(xyz=(0.0, -0.050, -0.115)),
        material=hinge_steel,
        name="rear_top_header",
    )

    _add_box_beam(
        rear_frame,
        left_rear_top,
        left_rear_bottom,
        width=0.028,
        depth=0.052,
        material=aluminum,
        name="left_rear_rail",
    )
    _add_box_beam(
        rear_frame,
        right_rear_top,
        right_rear_bottom,
        width=0.028,
        depth=0.052,
        material=aluminum,
        name="right_rear_rail",
    )
    _add_box_beam(
        rear_frame,
        (-0.140, -0.105, -0.355),
        (0.140, -0.105, -0.355),
        width=0.024,
        depth=0.040,
        material=aluminum,
        name="rear_mid_brace",
    )
    _add_box_beam(
        rear_frame,
        (-0.150, -0.170, -0.665),
        (0.150, -0.170, -0.665),
        width=0.024,
        depth=0.040,
        material=aluminum,
        name="rear_lower_brace",
    )

    rear_frame.visual(
        Box((0.085, 0.038, 0.040)),
        origin=Origin(xyz=(-0.170, -0.220, -0.975)),
        material=rubber,
        name="left_rear_foot",
    )
    rear_frame.visual(
        Box((0.085, 0.038, 0.040)),
        origin=Origin(xyz=(0.170, -0.220, -0.975)),
        material=rubber,
        name="right_rear_foot",
    )

    model.articulation(
        "rear_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.016, 0.995)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.6,
            lower=0.0,
            upper=0.47,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_fold_hinge")

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="y",
        positive_elem="left_front_foot",
        negative_elem="left_rear_foot",
        min_gap=0.38,
        name="front feet sit well ahead of rear feet in the open A-frame pose",
    )
    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="front_hinge_barrel",
        elem_b="left_rear_hinge_barrel",
        name="left rear hinge barrel contacts the front hinge barrel at the knuckle joint",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="yz",
        elem_a="front_hinge_barrel",
        elem_b="left_rear_hinge_barrel",
        min_overlap=0.03,
        name="interleaved hinge barrels stay aligned on the same hinge axis",
    )

    rear_foot_open = ctx.part_element_world_aabb(rear_frame, elem="left_rear_foot")
    with ctx.pose({hinge: 0.45}):
        rear_foot_folded = ctx.part_element_world_aabb(rear_frame, elem="left_rear_foot")
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="z",
            positive_elem="top_cap",
            negative_elem="rear_top_header",
            min_gap=0.0,
            max_gap=0.14,
            name="rear frame stays tucked just below the top cap when folded",
        )

    open_y = rear_foot_open[0][1] if rear_foot_open is not None else None
    folded_y = rear_foot_folded[0][1] if rear_foot_folded is not None else None
    ctx.check(
        "rear support frame folds forward toward the front frame",
        open_y is not None and folded_y is not None and folded_y > open_y + 0.35,
        details=f"open rear foot min y={open_y}, folded rear foot min y={folded_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
