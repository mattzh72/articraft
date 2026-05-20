from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sliding_desk_drawer")

    maple = model.material("clear_coated_maple", color=(0.72, 0.50, 0.30, 1.0))
    drawer_wood = model.material("slightly_darker_drawer_front", color=(0.50, 0.31, 0.17, 1.0))
    raw_plywood = model.material("drawer_interior_plywood", color=(0.64, 0.46, 0.28, 1.0))
    dark_shadow = model.material("shadowed_cavity", color=(0.08, 0.07, 0.06, 1.0))
    brushed_metal = model.material("brushed_metal", color=(0.62, 0.62, 0.58, 1.0))

    carcass = model.part("carcass")

    # Minimal desk body: a top, two cheek panels, a bottom shelf, a rear panel,
    # four square legs, and simple metal drawer runners.  The front is left open
    # so the drawer box can slide out along +X.
    carcass.visual(
        Box((0.66, 0.86, 0.035)),
        origin=Origin(xyz=(-0.27, 0.0, 0.7325)),
        material=maple,
        name="desktop",
    )
    carcass.visual(
        Box((0.56, 0.030, 0.216)),
        origin=Origin(xyz=(-0.27, 0.365, 0.6075)),
        material=maple,
        name="side_panel_0",
    )
    carcass.visual(
        Box((0.56, 0.030, 0.216)),
        origin=Origin(xyz=(-0.27, -0.365, 0.6075)),
        material=maple,
        name="side_panel_1",
    )
    carcass.visual(
        Box((0.56, 0.82, 0.030)),
        origin=Origin(xyz=(-0.27, 0.0, 0.497)),
        material=maple,
        name="bottom_shelf",
    )
    carcass.visual(
        Box((0.030, 0.73, 0.216)),
        origin=Origin(xyz=(-0.545, 0.0, 0.6075)),
        material=dark_shadow,
        name="rear_panel",
    )
    for i, (x, y) in enumerate(((0.005, 0.392), (0.005, -0.392), (-0.545, 0.392), (-0.545, -0.392))):
        carcass.visual(
            Box((0.045, 0.045, 0.49)),
            origin=Origin(xyz=(x, y, 0.245)),
            material=maple,
            name=f"leg_{i}",
        )
    carcass.visual(
        Box((0.52, 0.028, 0.018)),
        origin=Origin(xyz=(-0.285, 0.336, 0.560)),
        material=brushed_metal,
        name="runner_0",
    )
    carcass.visual(
        Box((0.52, 0.028, 0.018)),
        origin=Origin(xyz=(-0.285, -0.336, 0.560)),
        material=brushed_metal,
        name="runner_1",
    )

    drawer = model.part("drawer")

    # The drawer part frame is at the front opening centerline.  At q=0 the
    # front plate is seated just proud of the carcass while the open-top box
    # extends back into the desk.
    drawer.visual(
        Box((0.030, 0.67, 0.170)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=drawer_wood,
        name="front_panel",
    )
    drawer.visual(
        Box((0.49, 0.60, 0.018)),
        origin=Origin(xyz=(-0.245, 0.0, -0.055)),
        material=raw_plywood,
        name="bottom_tray",
    )
    drawer.visual(
        Box((0.49, 0.018, 0.112)),
        origin=Origin(xyz=(-0.245, 0.301, -0.005)),
        material=raw_plywood,
        name="side_wall_0",
    )
    drawer.visual(
        Box((0.49, 0.018, 0.112)),
        origin=Origin(xyz=(-0.245, -0.301, -0.005)),
        material=raw_plywood,
        name="side_wall_1",
    )
    drawer.visual(
        Box((0.018, 0.61, 0.112)),
        origin=Origin(xyz=(-0.482, 0.0, -0.005)),
        material=raw_plywood,
        name="back_wall",
    )
    drawer.visual(
        Box((0.46, 0.012, 0.018)),
        origin=Origin(xyz=(-0.250, 0.316, -0.040)),
        material=brushed_metal,
        name="slide_0",
    )
    drawer.visual(
        Box((0.46, 0.012, 0.018)),
        origin=Origin(xyz=(-0.250, -0.316, -0.040)),
        material=brushed_metal,
        name="slide_1",
    )
    drawer.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.047, 0.160, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="handle_post_0",
    )
    drawer.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.047, -0.160, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="handle_post_1",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.38),
        origin=Origin(xyz=(0.076, 0.0, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="handle_bar",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.34),
        motion_properties=MotionProperties(damping=1.5, friction=0.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("drawer_slide")
    limits = slide.motion_limits

    ctx.check(
        "single prismatic drawer axis",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.30 <= limits.upper <= 0.40,
        details=f"joint={slide}, limits={limits}",
    )

    ctx.expect_within(
        drawer,
        carcass,
        axes="yz",
        inner_elem="bottom_tray",
        margin=0.002,
        name="drawer tray sits inside the desk opening",
    )
    ctx.expect_contact(
        drawer,
        carcass,
        elem_a="slide_0",
        elem_b="runner_0",
        contact_tol=0.002,
        name="drawer slide is guided by the carcass runner",
    )
    ctx.expect_overlap(
        drawer,
        carcass,
        axes="x",
        elem_a="slide_0",
        elem_b="runner_0",
        min_overlap=0.44,
        name="closed drawer runner is fully engaged",
    )

    rest_pos = ctx.part_world_position(drawer)
    upper = limits.upper if limits is not None and limits.upper is not None else 0.0
    with ctx.pose({slide: upper}):
        ctx.expect_within(
            drawer,
            carcass,
            axes="yz",
            inner_elem="bottom_tray",
            margin=0.002,
            name="extended drawer stays level in the opening",
        )
        ctx.expect_contact(
            drawer,
            carcass,
            elem_a="slide_0",
            elem_b="runner_0",
            contact_tol=0.002,
            name="extended drawer remains on the runner",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="x",
            elem_a="slide_0",
            elem_b="runner_0",
            min_overlap=0.10,
            name="extended drawer remains retained in the carcass",
        )
        extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer translates straight outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.33
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
