from __future__ import annotations

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


SPAN_LIMIT = 0.78
TOOL_TRAVEL = 0.32


def _box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_inspection_portal")

    model.material("powder_coated_frame", rgba=(0.18, 0.22, 0.26, 1.0))
    model.material("cut_metal_edges", rgba=(0.42, 0.45, 0.48, 1.0))
    model.material("linear_rail_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    model.material("shuttle_dark", rgba=(0.08, 0.10, 0.12, 1.0))
    model.material("carriage_blue", rgba=(0.12, 0.34, 0.68, 1.0))
    model.material("tool_black", rgba=(0.02, 0.02, 0.025, 1.0))
    model.material("lens_glass", rgba=(0.05, 0.12, 0.18, 0.85))

    frame = model.part("frame")
    _box(frame, (0.38, 0.52, 0.06), (-1.15, 0.0, 0.03), "powder_coated_frame", "foot_0")
    _box(frame, (0.38, 0.52, 0.06), (1.15, 0.0, 0.03), "powder_coated_frame", "foot_1")
    _box(frame, (0.16, 0.24, 1.72), (-1.15, 0.0, 0.86), "powder_coated_frame", "box_leg_0")
    _box(frame, (0.16, 0.24, 1.72), (1.15, 0.0, 0.86), "powder_coated_frame", "box_leg_1")
    _box(frame, (2.55, 0.26, 0.20), (0.0, 0.0, 1.81), "powder_coated_frame", "top_beam")
    _box(frame, (2.36, 0.035, 0.05), (0.0, -0.095, 1.69), "linear_rail_steel", "rail_0")
    _box(frame, (2.36, 0.035, 0.05), (0.0, 0.095, 1.69), "linear_rail_steel", "rail_1")
    _box(frame, (0.28, 0.28, 0.012), (-1.15, 0.0, 1.912), "cut_metal_edges", "cap_0")
    _box(frame, (0.28, 0.28, 0.012), (1.15, 0.0, 1.912), "cut_metal_edges", "cap_1")

    shuttle = model.part("shuttle")
    _box(shuttle, (0.34, 0.045, 0.028), (0.0, -0.095, -0.014), "linear_rail_steel", "shoe_0")
    _box(shuttle, (0.34, 0.045, 0.028), (0.0, 0.095, -0.014), "linear_rail_steel", "shoe_1")
    _box(shuttle, (0.44, 0.34, 0.16), (0.0, 0.0, -0.108), "shuttle_dark", "shuttle_body")
    _box(shuttle, (0.22, 0.16, 0.04), (0.0, 0.0, -0.208), "shuttle_dark", "guide_bridge")
    _box(shuttle, (0.022, 0.040, 0.62), (-0.083, 0.0, -0.515), "linear_rail_steel", "guide_0")
    _box(shuttle, (0.022, 0.040, 0.62), (0.083, 0.0, -0.515), "linear_rail_steel", "guide_1")
    _box(shuttle, (0.32, 0.018, 0.055), (0.0, -0.179, -0.106), "cut_metal_edges", "wiper_0")
    _box(shuttle, (0.32, 0.018, 0.055), (0.0, 0.179, -0.106), "cut_metal_edges", "wiper_1")

    tool_carriage = model.part("tool_carriage")
    _box(tool_carriage, (0.10, 0.08, 0.40), (0.0, 0.0, -0.20), "carriage_blue", "slide_body")
    _box(tool_carriage, (0.13, 0.095, 0.08), (0.0, 0.0, -0.44), "tool_black", "sensor_head")
    tool_carriage.visual(
        Cylinder(radius=0.018, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        material="tool_black",
        name="probe_tip",
    )
    tool_carriage.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.505)),
        material="lens_glass",
        name="inspection_lens",
    )

    model.articulation(
        "beam_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, 1.665)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-SPAN_LIMIT, upper=SPAN_LIMIT, effort=900.0, velocity=0.55),
    )
    model.articulation(
        "tool_slide",
        ArticulationType.PRISMATIC,
        parent=shuttle,
        child=tool_carriage,
        origin=Origin(xyz=(0.0, 0.0, -0.228)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=TOOL_TRAVEL, effort=350.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    shuttle = object_model.get_part("shuttle")
    tool_carriage = object_model.get_part("tool_carriage")
    beam_slide = object_model.get_articulation("beam_slide")
    tool_slide = object_model.get_articulation("tool_slide")

    ctx.expect_gap(
        frame,
        shuttle,
        axis="z",
        positive_elem="rail_0",
        negative_elem="shoe_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="shuttle shoe touches the beam rail",
    )
    ctx.expect_gap(
        shuttle,
        tool_carriage,
        axis="z",
        positive_elem="guide_bridge",
        negative_elem="slide_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="tool carriage seats under the shuttle",
    )
    ctx.expect_within(
        tool_carriage,
        shuttle,
        axes="xy",
        inner_elem="slide_body",
        outer_elem="guide_bridge",
        margin=0.0,
        name="narrow tool carriage fits inside the shuttle guide",
    )

    with ctx.pose({beam_slide: SPAN_LIMIT}):
        ctx.expect_within(
            shuttle,
            frame,
            axes="x",
            inner_elem="shuttle_body",
            outer_elem="top_beam",
            margin=0.0,
            name="shuttle remains under the right side of the beam",
        )
    with ctx.pose({beam_slide: -SPAN_LIMIT}):
        ctx.expect_within(
            shuttle,
            frame,
            axes="x",
            inner_elem="shuttle_body",
            outer_elem="top_beam",
            margin=0.0,
            name="shuttle remains under the left side of the beam",
        )

    rest_position = ctx.part_world_position(tool_carriage)
    with ctx.pose({tool_slide: TOOL_TRAVEL}):
        lowered_position = ctx.part_world_position(tool_carriage)
        ctx.expect_overlap(
            tool_carriage,
            shuttle,
            axes="z",
            elem_a="slide_body",
            elem_b="guide_0",
            min_overlap=0.20,
            name="lowered carriage remains engaged with vertical guide",
        )
    ctx.check(
        "tool carriage lowers vertically",
        rest_position is not None
        and lowered_position is not None
        and lowered_position[2] < rest_position[2] - 0.25,
        details=f"rest={rest_position}, lowered={lowered_position}",
    )

    return ctx.report()


object_model = build_object_model()
