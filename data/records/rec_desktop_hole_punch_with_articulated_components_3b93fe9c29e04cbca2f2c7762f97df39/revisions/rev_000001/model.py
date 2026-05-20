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


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_hole_desktop_punch")

    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    body_mid = model.material("body_mid", rgba=(0.28, 0.29, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    stop_gray = model.material("stop_gray", rgba=(0.76, 0.78, 0.80, 1.0))
    selector_red = model.material("selector_red", rgba=(0.72, 0.18, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.23, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_dark,
        name="base_plate",
    )
    base.visual(
        Box((0.14, 0.30, 0.040)),
        origin=Origin(xyz=(-0.015, 0.0, 0.032)),
        material=body_mid,
        name="main_body",
    )
    base.visual(
        Box((0.036, 0.30, 0.020)),
        origin=Origin(xyz=(0.046, 0.0, 0.022)),
        material=body_mid,
        name="front_nose",
    )

    for idx, y in enumerate((-0.105, -0.035, 0.035, 0.105)):
        base.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(0.004, y, 0.047), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=f"punch_head_{idx}",
        )

    base.visual(
        Box((0.022, 0.022, 0.056)),
        origin=Origin(xyz=(-0.082, -0.137, 0.040)),
        material=body_mid,
        name="hinge_cheek_0",
    )
    base.visual(
        Box((0.022, 0.022, 0.056)),
        origin=Origin(xyz=(-0.082, 0.137, 0.040)),
        material=body_mid,
        name="hinge_cheek_1",
    )
    base.visual(
        Box((0.028, 0.30, 0.018)),
        origin=Origin(xyz=(-0.096, 0.0, 0.041)),
        material=body_dark,
        name="rear_bridge",
    )

    base.visual(
        Box((0.018, 0.012, 0.028)),
        origin=Origin(xyz=(0.082, -0.133, 0.026)),
        material=body_mid,
        name="rail_support_0",
    )
    base.visual(
        Box((0.018, 0.012, 0.028)),
        origin=Origin(xyz=(0.082, 0.133, 0.026)),
        material=body_mid,
        name="rail_support_1",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.278),
        origin=Origin(xyz=(0.082, 0.0, 0.026), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_rail",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.252),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    handle.visual(
        Box((0.020, 0.210, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, 0.001)),
        material=plastic_black,
        name="hinge_web",
    )
    handle.visual(
        Box((0.160, 0.290, 0.014)),
        origin=Origin(xyz=(0.080, 0.0, 0.007)),
        material=plastic_black,
        name="handle_beam",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.270),
        origin=Origin(xyz=(0.088, 0.0, 0.016), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="handle_crown",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.290),
        origin=Origin(xyz=(0.146, 0.0, 0.018), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
        name="front_grip",
    )

    stop = model.part("paper_stop")
    stop.visual(
        Box((0.018, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=stop_gray,
        name="top_shoe",
    )
    stop.visual(
        Box((0.018, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.0115)),
        material=stop_gray,
        name="bottom_shoe",
    )
    stop.visual(
        Box((0.010, 0.022, 0.030)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=stop_gray,
        name="rear_bridge",
    )
    stop.visual(
        Box((0.060, 0.078, 0.024)),
        origin=Origin(xyz=(0.038, 0.0, 0.018)),
        material=stop_gray,
        name="stop_fence",
    )
    stop.visual(
        Box((0.020, 0.010, 0.016)),
        origin=Origin(xyz=(0.035, 0.039, 0.016)),
        material=stop_gray,
        name="selector_bracket",
    )
    stop.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.035, 0.047, 0.016), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="selector_axle",
    )

    selector = model.part("selector_disc")
    selector.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=selector_red,
        name="disc_body",
    )
    selector.visual(
        Box((0.008, 0.004, 0.010)),
        origin=Origin(xyz=(0.013, 0.004, 0.0)),
        material=selector_red,
        name="selector_tab",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.070, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=30.0, velocity=2.5),
    )
    model.articulation(
        "base_to_paper_stop",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stop,
        origin=Origin(xyz=(0.082, 0.0, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.090, upper=0.090, effort=8.0, velocity=0.20),
    )
    model.articulation(
        "paper_stop_to_selector_disc",
        ArticulationType.CONTINUOUS,
        parent=stop,
        child=selector,
        origin=Origin(xyz=(0.035, 0.052, 0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    stop = object_model.get_part("paper_stop")
    selector = object_model.get_part("selector_disc")

    handle_joint = object_model.get_articulation("base_to_handle")
    stop_joint = object_model.get_articulation("base_to_paper_stop")
    selector_joint = object_model.get_articulation("paper_stop_to_selector_disc")

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="handle_beam",
            negative_elem="punch_head_1",
            min_gap=0.004,
            max_gap=0.020,
            name="closed handle sits just above the punch heads",
        )
        closed_grip = ctx.part_element_world_aabb(handle, elem="front_grip")

    with ctx.pose({handle_joint: 1.05}):
        open_grip = ctx.part_element_world_aabb(handle, elem="front_grip")

    closed_grip_center = _aabb_center(closed_grip)
    open_grip_center = _aabb_center(open_grip)
    ctx.check(
        "handle opens upward from the rear hinge",
        closed_grip_center is not None
        and open_grip_center is not None
        and open_grip_center[2] > closed_grip_center[2] + 0.080
        and open_grip_center[0] < closed_grip_center[0] - 0.030,
        details=f"closed={closed_grip_center}, open={open_grip_center}",
    )

    with ctx.pose({stop_joint: 0.0}):
        ctx.expect_gap(
            stop,
            base,
            axis="z",
            positive_elem="top_shoe",
            negative_elem="front_rail",
            min_gap=0.0005,
            max_gap=0.0020,
            name="top shoe clears the front rail with a small running gap",
        )
        ctx.expect_gap(
            base,
            stop,
            axis="z",
            positive_elem="front_rail",
            negative_elem="bottom_shoe",
            min_gap=0.0005,
            max_gap=0.0020,
            name="bottom shoe clears the front rail with a small running gap",
        )
        ctx.expect_overlap(
            stop,
            base,
            axes="y",
            elem_a="top_shoe",
            elem_b="front_rail",
            min_overlap=0.020,
            name="paper stop remains captured on the transverse rail",
        )
        stop_rest = ctx.part_world_position(stop)

    with ctx.pose({stop_joint: 0.080}):
        stop_shifted = ctx.part_world_position(stop)
        ctx.expect_overlap(
            stop,
            base,
            axes="y",
            elem_a="top_shoe",
            elem_b="front_rail",
            min_overlap=0.020,
            name="paper stop remains captured on the rail at the outer setting",
        )

    ctx.check(
        "paper stop slides laterally on the front rail",
        stop_rest is not None
        and stop_shifted is not None
        and stop_shifted[1] > stop_rest[1] + 0.060
        and abs(stop_shifted[0] - stop_rest[0]) < 1e-6
        and abs(stop_shifted[2] - stop_rest[2]) < 1e-6,
        details=f"rest={stop_rest}, shifted={stop_shifted}",
    )

    with ctx.pose({selector_joint: 0.0}):
        ctx.expect_gap(
            selector,
            stop,
            axis="y",
            positive_elem="disc_body",
            negative_elem="selector_axle",
            max_gap=0.0005,
            max_penetration=0.0001,
            name="selector disc seats against the short axle shoulder",
        )
        tab_rest = ctx.part_element_world_aabb(selector, elem="selector_tab")

    with ctx.pose({selector_joint: 1.0}):
        tab_rotated = ctx.part_element_world_aabb(selector, elem="selector_tab")

    tab_rest_center = _aabb_center(tab_rest)
    tab_rotated_center = _aabb_center(tab_rotated)
    ctx.check(
        "selector disc rotates its pointer around the stop-mounted axle",
        tab_rest_center is not None
        and tab_rotated_center is not None
        and abs(tab_rotated_center[0] - tab_rest_center[0]) > 0.004
        and abs(tab_rotated_center[2] - tab_rest_center[2]) > 0.004,
        details=f"rest={tab_rest_center}, rotated={tab_rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()
