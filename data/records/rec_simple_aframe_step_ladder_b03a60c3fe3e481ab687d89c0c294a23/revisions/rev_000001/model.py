from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot

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
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    tread_aluminum = model.material("tread_aluminum", rgba=(0.86, 0.87, 0.88, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.20, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.50, 0.24, 1.22)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.06, 0.61)),
    )

    rail_radius = 0.015
    brace_radius = 0.010
    rear_rail_radius = 0.013

    left_front_bottom = (0.22, 0.015, 0.055)
    right_front_bottom = (-0.22, 0.015, 0.055)
    left_front_top = (0.17, 0.045, 1.125)
    right_front_top = (-0.17, 0.045, 1.125)

    _add_member(
        front_frame,
        left_front_bottom,
        left_front_top,
        radius=rail_radius,
        material=aluminum,
        name="left_front_rail",
    )
    _add_member(
        front_frame,
        right_front_bottom,
        right_front_top,
        radius=rail_radius,
        material=aluminum,
        name="right_front_rail",
    )

    front_frame.visual(
        Box((0.46, 0.090, 0.048)),
        origin=Origin(xyz=(0.0, 0.036, 1.150)),
        material=dark_plastic,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.37, 0.05, 0.028)),
        origin=Origin(xyz=(0.0, 0.020, 1.115)),
        material=aluminum,
        name="top_cross_support",
    )

    front_frame.visual(
        Cylinder(radius=0.014, length=0.058),
        origin=Origin(xyz=(0.195, 0.098, 1.145), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_plastic,
        name="left_front_hinge_sleeve",
    )
    front_frame.visual(
        Cylinder(radius=0.014, length=0.058),
        origin=Origin(xyz=(-0.195, 0.098, 1.145), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_plastic,
        name="right_front_hinge_sleeve",
    )
    front_frame.visual(
        Box((0.028, 0.036, 0.085)),
        origin=Origin(xyz=(0.182, 0.077, 1.113)),
        material=dark_plastic,
        name="left_hinge_cheek",
    )
    front_frame.visual(
        Box((0.028, 0.036, 0.085)),
        origin=Origin(xyz=(-0.182, 0.077, 1.113)),
        material=dark_plastic,
        name="right_hinge_cheek",
    )

    tread_specs = [
        ("tread_1", 0.26, 0.105),
        ("tread_2", 0.49, 0.102),
        ("tread_3", 0.72, 0.098),
        ("tread_4", 0.93, 0.094),
    ]
    for tread_name, z, depth in tread_specs:
        y = 0.020 + (0.045 - 0.015) * ((z - 0.07) / (1.125 - 0.07))
        front_frame.visual(
            Box((0.360, depth, 0.022)),
            origin=Origin(xyz=(0.0, y, z)),
            material=tread_aluminum,
            name=tread_name,
        )

    _add_member(
        front_frame,
        (0.165, 0.025, 0.255),
        (0.050, 0.032, 1.070),
        radius=brace_radius,
        material=aluminum,
        name="left_inner_brace",
    )
    _add_member(
        front_frame,
        (-0.165, 0.025, 0.255),
        (-0.050, 0.032, 1.070),
        radius=brace_radius,
        material=aluminum,
        name="right_inner_brace",
    )

    front_frame.visual(
        Box((0.060, 0.075, 0.060)),
        origin=Origin(xyz=(0.220, 0.015, 0.030)),
        material=rubber,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.060, 0.075, 0.060)),
        origin=Origin(xyz=(-0.220, 0.015, 0.030)),
        material=rubber,
        name="right_front_foot",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.62, 1.16)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.28, -0.58)),
    )

    left_rear_top = (0.168, 0.040, -0.030)
    right_rear_top = (-0.168, 0.040, -0.030)
    left_rear_bottom = (0.205, 0.535, -1.095)
    right_rear_bottom = (-0.205, 0.535, -1.095)

    rear_frame.visual(
        Box((0.350, 0.032, 0.040)),
        origin=Origin(xyz=(0.0, 0.042, -0.050)),
        material=aluminum,
        name="rear_top_header",
    )
    rear_frame.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.144, 0.002, -0.004), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_plastic,
        name="left_rear_hinge_sleeve",
    )
    rear_frame.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(-0.144, 0.002, -0.004), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_plastic,
        name="right_rear_hinge_sleeve",
    )
    rear_frame.visual(
        Box((0.024, 0.022, 0.072)),
        origin=Origin(xyz=(0.155, 0.020, -0.032)),
        material=dark_plastic,
    )
    rear_frame.visual(
        Box((0.024, 0.022, 0.072)),
        origin=Origin(xyz=(-0.155, 0.020, -0.032)),
        material=dark_plastic,
    )

    _add_member(
        rear_frame,
        left_rear_top,
        left_rear_bottom,
        radius=rear_rail_radius,
        material=aluminum,
        name="left_rear_rail",
    )
    _add_member(
        rear_frame,
        right_rear_top,
        right_rear_bottom,
        radius=rear_rail_radius,
        material=aluminum,
        name="right_rear_rail",
    )

    rear_bar_specs = [
        ("rear_bar_1", 0.18, -0.33),
        ("rear_bar_2", 0.31, -0.60),
        ("rear_bar_3", 0.435, -0.87),
    ]
    for bar_name, y, z in rear_bar_specs:
        rear_frame.visual(
            Cylinder(radius=0.010, length=0.390),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, 1.57079632679, 0.0)),
            material=aluminum,
            name=bar_name,
        )

    _add_member(
        rear_frame,
        (0.150, 0.090, -0.120),
        (0.030, 0.420, -0.820),
        radius=0.008,
        material=aluminum,
        name="left_rear_diagonal",
    )
    _add_member(
        rear_frame,
        (-0.150, 0.090, -0.120),
        (-0.030, 0.420, -0.820),
        radius=0.008,
        material=aluminum,
        name="right_rear_diagonal",
    )

    rear_frame.visual(
        Box((0.050, 0.070, 0.046)),
        origin=Origin(xyz=(0.205, 0.535, -1.118)),
        material=rubber,
        name="left_rear_foot",
    )
    rear_frame.visual(
        Box((0.050, 0.070, 0.046)),
        origin=Origin(xyz=(-0.205, 0.535, -1.118)),
        material=rubber,
        name="right_rear_foot",
    )

    model.articulation(
        "rear_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.098, 1.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.4,
            lower=-0.45,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("rear_fold_hinge")

    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="x",
        min_overlap=0.34,
        name="front and rear frames share the ladder width",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="left_rear_foot",
            negative_elem="left_front_foot",
            min_gap=0.40,
            name="rear feet stand well behind the climbing feet when open",
        )

    with ctx.pose({hinge: -0.45}):
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="left_rear_foot",
            negative_elem="left_front_foot",
            max_gap=0.06,
            max_penetration=0.02,
            name="rear frame folds near the front frame at the closed limit",
        )
        ctx.expect_overlap(
            rear_frame,
            front_frame,
            axes="x",
            elem_a="left_rear_foot",
            elem_b="left_front_foot",
            min_overlap=0.03,
            name="folded feet stay laterally aligned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
