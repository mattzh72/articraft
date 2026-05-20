from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
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


FRAME_WIDTH = 0.92
FRAME_DEPTH = 0.24
FRAME_HEIGHT = 2.07

BASE_HEIGHT = 0.14
TOP_HEIGHT = 0.12
TOP_Z = FRAME_HEIGHT - (TOP_HEIGHT / 2.0)

COLUMN_CENTER_X = 0.33
COLUMN_SIZE = 0.11
COLUMN_HEIGHT = FRAME_HEIGHT - BASE_HEIGHT - TOP_HEIGHT
COLUMN_Z = BASE_HEIGHT + (COLUMN_HEIGHT / 2.0)

CARRIAGE_HOME_Z = 0.70
CARRIAGE_TRAVEL = 0.90
CARRIAGE_OUTER_WIDTH = 0.84
CARRIAGE_DEPTH = 0.18
CARRIAGE_HEIGHT = 0.54


def _box_shape(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _guide_sleeve(center_x: float, center_z: float) -> cq.Workplane:
    outer = _box_shape((0.18, 0.18, 0.10), (center_x, 0.0, center_z))
    inner = _box_shape((COLUMN_SIZE + 0.014, COLUMN_SIZE + 0.014, 0.12), (center_x, 0.0, center_z))
    return outer.cut(inner)


def _carriage_shape() -> cq.Workplane:
    top_beam = _box_shape((0.50, 0.18, 0.06), (0.0, 0.0, 0.24))
    bottom_beam = _box_shape((0.50, 0.18, 0.06), (0.0, 0.0, -0.24))
    left_tower = _box_shape((0.07, 0.18, 0.42), (-0.215, 0.0, 0.0))
    right_tower = _box_shape((0.07, 0.18, 0.42), (0.215, 0.0, 0.0))

    front_panel = _box_shape((0.40, 0.04, 0.46), (0.0, 0.055, 0.0))
    front_panel_window = _box_shape((0.22, 0.05, 0.24), (0.0, 0.055, 0.0))
    front_panel = front_panel.cut(front_panel_window)

    return (
        top_beam.union(bottom_beam)
        .union(left_tower)
        .union(right_tower)
        .union(front_panel)
        .union(_guide_sleeve(-COLUMN_CENTER_X, -0.15))
        .union(_guide_sleeve(-COLUMN_CENTER_X, 0.15))
        .union(_guide_sleeve(COLUMN_CENTER_X, -0.15))
        .union(_guide_sleeve(COLUMN_CENTER_X, 0.15))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage")

    model.material("frame_blue", rgba=(0.24, 0.36, 0.52, 1.0))
    model.material("column_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("carriage_yellow", rgba=(0.91, 0.73, 0.16, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material="frame_blue",
        name="base_crosshead",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH * 0.82, TOP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        material="frame_blue",
        name="top_crosshead",
    )
    frame.visual(
        Box((COLUMN_SIZE, COLUMN_SIZE, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, 0.0, COLUMN_Z)),
        material="column_gray",
        name="left_column",
    )
    frame.visual(
        Box((COLUMN_SIZE, COLUMN_SIZE, COLUMN_HEIGHT)),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.0, COLUMN_Z)),
        material="column_gray",
        name="right_column",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=96.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_shell"),
        material="carriage_yellow",
        name="carriage_shell",
    )
    guide_pad_t = 0.008
    guide_pad_d = 0.06
    guide_pad_h = 0.08
    for side_name, column_center, inner_face, outer_face in (
        ("left", -COLUMN_CENTER_X, -COLUMN_CENTER_X + (COLUMN_SIZE / 2.0), -COLUMN_CENTER_X - (COLUMN_SIZE / 2.0)),
        ("right", COLUMN_CENTER_X, COLUMN_CENTER_X - (COLUMN_SIZE / 2.0), COLUMN_CENTER_X + (COLUMN_SIZE / 2.0)),
    ):
        for z_name, z_center in (("lower", -0.15), ("upper", 0.15)):
            carriage.visual(
                Box((guide_pad_t, guide_pad_d, guide_pad_h)),
                origin=Origin(
                    xyz=(
                        inner_face + (guide_pad_t / 2.0 if side_name == "left" else -guide_pad_t / 2.0),
                        0.0,
                        z_center,
                    )
                ),
                material="column_gray",
                name=f"{side_name}_{z_name}_inboard_pad",
            )
            carriage.visual(
                Box((guide_pad_t, guide_pad_d, guide_pad_h)),
                origin=Origin(
                    xyz=(
                        outer_face - (guide_pad_t / 2.0 if side_name == "left" else -guide_pad_t / 2.0),
                        0.0,
                        z_center,
                    )
                ),
                material="column_gray",
                name=f"{side_name}_{z_name}_outboard_pad",
            )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_OUTER_WIDTH, CARRIAGE_DEPTH, CARRIAGE_HEIGHT)),
        mass=34.0,
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=7000.0,
            velocity=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")

    ctx.check(
        "vertical prismatic lift joint",
        lift.articulation_type == ArticulationType.PRISMATIC
        and all(isclose(a, b, abs_tol=1e-9) for a, b in zip(lift.axis, (0.0, 0.0, 1.0))),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_within(
            carriage,
            frame,
            axes="xy",
            margin=0.02,
            name="carriage stays inside frame footprint at rest",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            negative_elem="base_crosshead",
            min_gap=0.24,
            name="carriage clears base crosshead at rest",
        )
        rest_pos = ctx.part_world_position(carriage)

    upper = lift.motion_limits.upper if lift.motion_limits is not None else None
    upper_pos = None
    if upper is not None:
        with ctx.pose({lift: upper}):
            ctx.expect_within(
                carriage,
                frame,
                axes="xy",
                margin=0.02,
                name="carriage stays inside frame footprint at full lift",
            )
            ctx.expect_gap(
                frame,
                carriage,
                axis="z",
                positive_elem="top_crosshead",
                min_gap=0.06,
                name="carriage clears top crosshead at full lift",
            )
            upper_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage moves upward along columns",
        rest_pos is not None and upper_pos is not None and upper_pos[2] > rest_pos[2] + 0.5,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
