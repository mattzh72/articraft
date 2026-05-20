from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_WIDTH = 0.64
FRAME_HEIGHT = 1.78
BASE_DEPTH = 0.72
RAIL_X = 0.27
MAST_HEIGHT = 1.60
MAST_WIDTH = 0.08
MAST_DEPTH = 0.04


def _add_frame(model: ArticulatedObject) -> None:
    frame = model.part("frame")

    frame.visual(
        Box((0.08, BASE_DEPTH, 0.05)),
        origin=Origin(xyz=(-RAIL_X, 0.0, 0.025)),
        material="wood",
        name="left_foot",
    )
    frame.visual(
        Box((0.08, BASE_DEPTH, 0.05)),
        origin=Origin(xyz=(RAIL_X, 0.0, 0.025)),
        material="wood",
        name="right_foot",
    )
    frame.visual(
        Box((FRAME_WIDTH - 0.08, 0.07, 0.07)),
        origin=Origin(xyz=(0.0, -0.22, 0.085)),
        material="wood",
        name="front_base_stretcher",
    )
    frame.visual(
        Box((FRAME_WIDTH - 0.08, 0.07, 0.07)),
        origin=Origin(xyz=(0.0, 0.22, 0.085)),
        material="wood",
        name="rear_base_stretcher",
    )
    frame.visual(
        Box((0.075, 0.05, FRAME_HEIGHT)),
        origin=Origin(xyz=(-RAIL_X, 0.0, FRAME_HEIGHT / 2.0)),
        material="wood",
        name="left_rail",
    )
    frame.visual(
        Box((0.075, 0.05, FRAME_HEIGHT)),
        origin=Origin(xyz=(RAIL_X, 0.0, FRAME_HEIGHT / 2.0)),
        material="wood",
        name="right_rail",
    )
    frame.visual(
        Box((FRAME_WIDTH, 0.07, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        material="wood",
        name="top_crossbar",
    )
    frame.visual(
        Box((FRAME_WIDTH - 0.08, 0.06, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material="wood",
        name="lower_crossbar",
    )
    frame.visual(
        Box((MAST_WIDTH, MAST_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material="wood",
        name="mast",
    )
    frame.visual(
        Box((0.40, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, -0.03, 0.72)),
        material="wood",
        name="canvas_tray",
    )
    frame.visual(
        Box((0.40, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, -0.08, 0.765)),
        material="wood",
        name="tray_lip",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0.313, 0.0, 0.90), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="metal_dark",
        name="frame_hinge_barrel",
    )


def _add_top_clamp(model: ArticulatedObject) -> None:
    clamp = model.part("top_clamp")

    clamp.visual(
        Box((0.018, 0.082, 0.12)),
        origin=Origin(xyz=(-0.049, 0.0, 0.0)),
        material="wood",
        name="guide_left",
    )
    clamp.visual(
        Box((0.018, 0.082, 0.12)),
        origin=Origin(xyz=(0.049, 0.0, 0.0)),
        material="wood",
        name="guide_right",
    )
    clamp.visual(
        Box((0.122, 0.018, 0.12)),
        origin=Origin(xyz=(0.0, -0.032, 0.0)),
        material="wood",
        name="guide_front",
    )
    clamp.visual(
        Box((0.122, 0.018, 0.12)),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material="wood",
        name="guide_back",
    )
    clamp.visual(
        Box((0.34, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, -0.054, 0.0)),
        material="wood",
        name="jaw_bar",
    )
    clamp.visual(
        Box((0.18, 0.02, 0.022)),
        origin=Origin(xyz=(0.0, -0.077, -0.032)),
        material="wood",
        name="pressure_pad",
    )
    clamp.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.0, -0.087, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="metal_dark",
        name="clamp_knob",
    )

    model.articulation(
        "mast_to_top_clamp",
        ArticulationType.PRISMATIC,
        parent="frame",
        child=clamp,
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.40, effort=80.0, velocity=0.25),
    )


def _add_reference_shelf(model: ArticulatedObject) -> None:
    shelf = model.part("reference_shelf")

    shelf.visual(
        Box((0.018, 0.22, 0.24)),
        origin=Origin(xyz=(0.021, 0.0, 0.12)),
        material="wood",
        name="panel",
    )
    shelf.visual(
        Box((0.028, 0.22, 0.03)),
        origin=Origin(xyz=(0.026, 0.0, 0.225)),
        material="wood",
        name="retaining_lip",
    )
    shelf.visual(
        Cylinder(radius=0.012, length=0.04),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="metal_dark",
        name="hinge_knuckle_front",
    )
    shelf.visual(
        Cylinder(radius=0.012, length=0.04),
        origin=Origin(xyz=(0.0, 0.068, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="metal_dark",
        name="hinge_knuckle_rear",
    )

    model.articulation(
        "frame_to_reference_shelf",
        ArticulationType.REVOLUTE,
        parent="frame",
        child=shelf,
        origin=Origin(xyz=(0.313, 0.0, 0.90)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=pi / 2.0, effort=12.0, velocity=1.5),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_display_easel")

    model.material("wood", rgba=(0.70, 0.56, 0.36, 1.0))
    model.material("wood_dark", rgba=(0.56, 0.40, 0.23, 1.0))
    model.material("metal_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    _add_frame(model)
    _add_top_clamp(model)
    _add_reference_shelf(model)

    return model


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple(upper - lower for lower, upper in zip(aabb[0], aabb[1]))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    clamp = object_model.get_part("top_clamp")
    shelf = object_model.get_part("reference_shelf")
    clamp_slide = object_model.get_articulation("mast_to_top_clamp")
    shelf_hinge = object_model.get_articulation("frame_to_reference_shelf")

    clamp_limits = clamp_slide.motion_limits
    shelf_limits = shelf_hinge.motion_limits
    clamp_upper = clamp_limits.upper if clamp_limits is not None and clamp_limits.upper is not None else 0.0
    shelf_upper = shelf_limits.upper if shelf_limits is not None and shelf_limits.upper is not None else 0.0

    ctx.expect_origin_distance(
        clamp,
        frame,
        axes="xy",
        max_dist=0.002,
        name="top clamp stays centered on the mast axis",
    )
    ctx.expect_overlap(
        clamp,
        frame,
        axes="z",
        elem_a="guide_front",
        elem_b="mast",
        min_overlap=0.10,
        name="top clamp carriage overlaps the mast height",
    )

    rest_clamp_pos = ctx.part_world_position(clamp)
    with ctx.pose({clamp_slide: clamp_upper}):
        ctx.expect_origin_distance(
            clamp,
            frame,
            axes="xy",
            max_dist=0.002,
            name="raised top clamp remains centered on the mast axis",
        )
        raised_clamp_pos = ctx.part_world_position(clamp)

    ctx.check(
        "top clamp raises along the central mast",
        rest_clamp_pos is not None
        and raised_clamp_pos is not None
        and raised_clamp_pos[2] > rest_clamp_pos[2] + 0.20,
        details=f"rest={rest_clamp_pos}, raised={raised_clamp_pos}",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(shelf, elem="panel")
    closed_panel_size = _aabb_size(closed_panel_aabb)
    frame_aabb = ctx.part_world_aabb(frame)

    with ctx.pose({shelf_hinge: shelf_upper}):
        open_panel_aabb = ctx.part_element_world_aabb(shelf, elem="panel")
        open_panel_size = _aabb_size(open_panel_aabb)

    ctx.check(
        "reference shelf folds from a vertical stowed pose to a horizontal support pose",
        closed_panel_size is not None
        and open_panel_size is not None
        and closed_panel_size[2] > 0.22
        and closed_panel_size[0] < 0.03
        and open_panel_size[0] > 0.22
        and open_panel_size[2] < 0.03,
        details=f"closed={closed_panel_size}, open={open_panel_size}",
    )
    ctx.check(
        "opened reference shelf projects beyond the side of the frame",
        frame_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][0] > frame_aabb[1][0] + 0.18,
        details=f"frame={frame_aabb}, shelf_open={open_panel_aabb}",
    )
    ctx.expect_gap(
        shelf,
        frame,
        axis="x",
        positive_elem="panel",
        negative_elem="right_rail",
        min_gap=0.004,
        max_gap=0.03,
        name="stowed reference shelf sits just outboard of the side rail",
    )

    return ctx.report()


object_model = build_object_model()
