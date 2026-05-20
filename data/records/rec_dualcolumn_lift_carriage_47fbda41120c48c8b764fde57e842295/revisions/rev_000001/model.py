from __future__ import annotations

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


def _guide_block_mesh() -> cq.Workplane:
    """Compact saddle guide block with a real vertical clearance bore."""
    block = cq.Workplane("XY").box(0.068, 0.072, 0.240)
    bore = cq.Workplane("XY").cylinder(0.300, 0.025)
    return block.cut(bore).edges("|Z").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rail_service_lift")

    painted_frame = model.material("painted_frame", color=(0.08, 0.09, 0.10, 1.0))
    rail_steel = model.material("polished_guide_steel", color=(0.72, 0.76, 0.78, 1.0))
    dark_trim = model.material("dark_trim", color=(0.02, 0.025, 0.03, 1.0))
    saddle_blue = model.material("blue_saddle_casting", color=(0.05, 0.20, 0.55, 1.0))
    work_orange = model.material("orange_square_work_face", color=(0.95, 0.36, 0.08, 1.0))
    bearing_bronze = model.material("bearing_bronze", color=(0.74, 0.48, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.34, 0.18, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=painted_frame,
        name="base_plate",
    )
    frame.visual(
        Box((0.30, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, 0.020, 0.900)),
        material=painted_frame,
        name="top_bridge",
    )
    frame.visual(
        Box((0.055, 0.090, 0.070)),
        origin=Origin(xyz=(-0.095, 0.035, 0.072)),
        material=dark_trim,
        name="lower_clamp_0",
    )
    frame.visual(
        Box((0.055, 0.090, 0.070)),
        origin=Origin(xyz=(0.095, 0.035, 0.072)),
        material=dark_trim,
        name="lower_clamp_1",
    )
    frame.visual(
        Box((0.055, 0.090, 0.070)),
        origin=Origin(xyz=(-0.095, 0.035, 0.842)),
        material=dark_trim,
        name="upper_clamp_0",
    )
    frame.visual(
        Box((0.055, 0.090, 0.070)),
        origin=Origin(xyz=(0.095, 0.035, 0.842)),
        material=dark_trim,
        name="upper_clamp_1",
    )
    for index, x_pos in enumerate((-0.095, 0.095)):
        frame.visual(
            Cylinder(radius=0.018, length=0.870),
            origin=Origin(xyz=(x_pos, 0.035, 0.465)),
            material=rail_steel,
            name=f"guide_{index}",
        )

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.220, 0.018, 0.220)),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        material=work_orange,
        name="work_face",
    )
    saddle.visual(
        Box((0.270, 0.066, 0.115)),
        origin=Origin(xyz=(0.0, -0.025, 0.0)),
        material=saddle_blue,
        name="cross_web",
    )
    guide_mesh = _guide_block_mesh()
    for index, x_pos in enumerate((-0.095, 0.095)):
        saddle.visual(
            mesh_from_cadquery(guide_mesh, f"saddle_guide_block_{index}", tolerance=0.0008),
            origin=Origin(xyz=(x_pos, 0.035, 0.0)),
            material=saddle_blue,
            name=f"guide_block_{index}",
        )
        for side, sign in (("minus", -1.0), ("plus", 1.0)):
            saddle.visual(
                Box((0.010, 0.030, 0.180)),
                origin=Origin(xyz=(x_pos + sign * 0.023, 0.035, 0.0)),
                material=bearing_bronze,
                name=f"wear_pad_{index}_{side}",
            )

    model.articulation(
        "saddle_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.420),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    saddle = object_model.get_part("saddle")
    slide = object_model.get_articulation("saddle_slide")

    ctx.check(
        "single vertical prismatic stage",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={len(object_model.articulations)}, type={slide.articulation_type}, axis={slide.axis}",
    )

    for index in (0, 1):
        ctx.expect_within(
            frame,
            saddle,
            axes="xy",
            inner_elem=f"guide_{index}",
            outer_elem=f"guide_block_{index}",
            margin=0.0,
            name=f"guide {index} rail lies inside saddle block footprint",
        )
        ctx.expect_overlap(
            saddle,
            frame,
            axes="z",
            elem_a=f"guide_block_{index}",
            elem_b=f"guide_{index}",
            min_overlap=0.220,
            name=f"guide {index} retained overlap at lower travel",
        )

    lower_position = ctx.part_world_position(saddle)
    with ctx.pose({slide: 0.420}):
        upper_position = ctx.part_world_position(saddle)
        for index in (0, 1):
            ctx.expect_overlap(
                saddle,
                frame,
                axes="z",
                elem_a=f"guide_block_{index}",
                elem_b=f"guide_{index}",
                min_overlap=0.220,
                name=f"guide {index} retained overlap at upper travel",
            )
    ctx.check(
        "saddle moves upward on guides",
        lower_position is not None
        and upper_position is not None
        and upper_position[2] > lower_position[2] + 0.40,
        details=f"lower={lower_position}, upper={upper_position}",
    )

    return ctx.report()


object_model = build_object_model()
