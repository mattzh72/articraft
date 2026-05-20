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


def _center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="archive_paper_punch")

    body_color = model.material("body_color", rgba=(0.72, 0.74, 0.77, 1.0))
    trim_color = model.material("trim_color", rgba=(0.22, 0.24, 0.27, 1.0))
    handle_color = model.material("handle_color", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_color = model.material("wheel_color", rgba=(0.15, 0.15, 0.16, 1.0))
    accent_color = model.material("accent_color", rgba=(0.82, 0.16, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.140, 0.310, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_color,
        name="base_plate",
    )
    body.visual(
        Box((0.118, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.127, 0.017)),
        material=trim_color,
        name="front_guide",
    )
    body.visual(
        Box((0.114, 0.110, 0.044)),
        origin=Origin(xyz=(0.0, -0.070, 0.034)),
        material=body_color,
        name="rear_housing",
    )
    body.visual(
        Box((0.098, 0.054, 0.020)),
        origin=Origin(xyz=(0.0, -0.032, 0.066)),
        material=trim_color,
        name="top_cap",
    )
    body.visual(
        Box((0.022, 0.040, 0.028)),
        origin=Origin(xyz=(-0.041, -0.118, 0.070)),
        material=trim_color,
        name="hinge_support_0",
    )
    body.visual(
        Box((0.022, 0.040, 0.028)),
        origin=Origin(xyz=(0.041, -0.118, 0.070)),
        material=trim_color,
        name="hinge_support_1",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.036, -0.133, 0.077), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_color,
        name="hinge_stub_0",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.036, -0.133, 0.077), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_color,
        name="hinge_stub_1",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.054),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_color,
        name="hinge_tube",
    )
    handle.visual(
        Box((0.076, 0.208, 0.016)),
        origin=Origin(xyz=(0.0, 0.103, 0.015)),
        material=handle_color,
        name="lever_beam",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.102),
        origin=Origin(xyz=(0.0, 0.208, 0.023), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_color,
        name="grip",
    )
    handle.visual(
        Box((0.094, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, 0.184, 0.017)),
        material=handle_color,
        name="grip_pad",
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        Box((0.032, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.000, 0.002)),
        material=trim_color,
        name="runner",
    )
    paper_stop.visual(
        Box((0.108, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.007, 0.009)),
        material=body_color,
        name="stop_base",
    )
    paper_stop.visual(
        Box((0.104, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, -0.014, 0.025)),
        material=body_color,
        name="paper_fence",
    )
    paper_stop.visual(
        Box((0.014, 0.006, 0.022)),
        origin=Origin(xyz=(0.043, -0.006, 0.025)),
        material=trim_color,
        name="wheel_bracket",
    )
    paper_stop.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.043, -0.006, 0.014)),
        material=accent_color,
        name="wheel_mount",
    )

    indicator_wheel = model.part("indicator_wheel")
    indicator_wheel.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wheel_color,
        name="wheel_disc",
    )
    indicator_wheel.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=accent_color,
        name="wheel_hub",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, -0.133, 0.077)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "body_to_paper_stop",
        ArticulationType.PRISMATIC,
        parent=body,
        child=paper_stop,
        origin=Origin(xyz=(0.0, 0.127, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=-0.024, upper=0.024),
    )
    model.articulation(
        "paper_stop_to_indicator_wheel",
        ArticulationType.CONTINUOUS,
        parent=paper_stop,
        child=indicator_wheel,
        origin=Origin(xyz=(0.043, 0.000, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    paper_stop = object_model.get_part("paper_stop")
    indicator_wheel = object_model.get_part("indicator_wheel")

    handle_joint = object_model.get_articulation("body_to_handle")
    stop_joint = object_model.get_articulation("body_to_paper_stop")
    wheel_joint = object_model.get_articulation("paper_stop_to_indicator_wheel")

    ctx.expect_gap(
        paper_stop,
        body,
        axis="z",
        positive_elem="runner",
        negative_elem="front_guide",
        max_penetration=1e-5,
        max_gap=0.001,
        name="paper stop runner sits on the front guide",
    )
    ctx.expect_overlap(
        paper_stop,
        body,
        axes="x",
        elem_a="stop_base",
        elem_b="front_guide",
        min_overlap=0.060,
        name="paper stop spans the front guide",
    )
    ctx.expect_contact(
        indicator_wheel,
        paper_stop,
        elem_a="wheel_disc",
        elem_b="wheel_bracket",
        contact_tol=1e-6,
        name="indicator wheel is carried by the stop bracket",
    )
    ctx.expect_origin_gap(
        paper_stop,
        body,
        axis="y",
        min_gap=0.120,
        max_gap=0.135,
        name="paper stop assembly stays at the front of the punch",
    )

    closed_grip = None
    open_grip = None
    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="lever_beam",
            negative_elem="rear_housing",
            min_gap=0.0,
            max_gap=0.030,
            name="closed handle rests just above the punch housing",
        )
        closed_grip = ctx.part_element_world_aabb(handle, elem="grip")

    with ctx.pose({handle_joint: 1.0}):
        open_grip = ctx.part_element_world_aabb(handle, elem="grip")
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="grip",
            negative_elem="top_cap",
            min_gap=0.090,
            name="opened handle lifts well above the housing",
        )

    ctx.check(
        "handle grip rises when opened",
        _center_z(closed_grip) is not None
        and _center_z(open_grip) is not None
        and _center_z(open_grip) > _center_z(closed_grip) + 0.090,
        details=f"closed={closed_grip}, open={open_grip}",
    )

    lower_stop_pos = None
    upper_stop_pos = None
    lower_wheel_pos = None
    upper_wheel_pos = None
    with ctx.pose({stop_joint: -0.024}):
        lower_stop_pos = ctx.part_world_position(paper_stop)
        lower_wheel_pos = ctx.part_world_position(indicator_wheel)
        ctx.expect_overlap(
            paper_stop,
            body,
            axes="x",
            elem_a="stop_base",
            elem_b="front_guide",
            min_overlap=0.035,
            name="paper stop remains retained on the guide at the left extreme",
        )
    with ctx.pose({stop_joint: 0.024}):
        upper_stop_pos = ctx.part_world_position(paper_stop)
        upper_wheel_pos = ctx.part_world_position(indicator_wheel)
        ctx.expect_overlap(
            paper_stop,
            body,
            axes="x",
            elem_a="stop_base",
            elem_b="front_guide",
            min_overlap=0.035,
            name="paper stop remains retained on the guide at the right extreme",
        )

    ctx.check(
        "paper stop slides transversely across the front guide",
        lower_stop_pos is not None
        and upper_stop_pos is not None
        and upper_stop_pos[0] > lower_stop_pos[0] + 0.040,
        details=f"lower={lower_stop_pos}, upper={upper_stop_pos}",
    )
    ctx.check(
        "indicator wheel rides with the paper stop",
        lower_wheel_pos is not None
        and upper_wheel_pos is not None
        and upper_wheel_pos[0] > lower_wheel_pos[0] + 0.040,
        details=f"lower={lower_wheel_pos}, upper={upper_wheel_pos}",
    )

    wheel_rest_pos = ctx.part_world_position(indicator_wheel)
    spun_wheel = None
    with ctx.pose({wheel_joint: 1.6}):
        spun_wheel = ctx.part_world_position(indicator_wheel)
        ctx.expect_contact(
            indicator_wheel,
            paper_stop,
            elem_a="wheel_disc",
            elem_b="wheel_bracket",
            contact_tol=1e-6,
            name="indicator wheel stays seated on its axle bracket when spun",
        )
    ctx.check(
        "indicator wheel spins in place",
        wheel_rest_pos is not None
        and spun_wheel is not None
        and abs(spun_wheel[0] - wheel_rest_pos[0]) < 1e-6
        and abs(spun_wheel[1] - wheel_rest_pos[1]) < 1e-6
        and abs(spun_wheel[2] - wheel_rest_pos[2]) < 1e-6,
        details=f"rest={wheel_rest_pos}, spun={spun_wheel}",
    )

    return ctx.report()


object_model = build_object_model()
