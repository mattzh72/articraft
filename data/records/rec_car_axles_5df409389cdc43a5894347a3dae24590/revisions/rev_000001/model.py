from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snowmobile_front_ski_spindle")

    satin_black = model.material("satin_black", rgba=(0.015, 0.017, 0.018, 1.0))
    chassis_paint = model.material("chassis_paint", rgba=(0.08, 0.11, 0.14, 1.0))
    powder_red = model.material("powder_red", rgba=(0.74, 0.05, 0.04, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.25, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.005, 0.006, 0.007, 1.0))
    rubber = model.material("rubber", rgba=(0.025, 0.025, 0.023, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.07, 0.72, 0.15)),
        origin=Origin(xyz=(-0.085, 0.0, 0.01)),
        material=chassis_paint,
        name="bulkhead",
    )
    chassis.visual(
        Box((0.08, 0.66, 0.055)),
        origin=Origin(xyz=(-0.035, 0.0, -0.072)),
        material=chassis_paint,
        name="lower_rail",
    )
    chassis.visual(
        Box((0.08, 0.66, 0.035)),
        origin=Origin(xyz=(-0.035, 0.0, 0.092)),
        material=chassis_paint,
        name="upper_flange",
    )

    # Four clevis ears carry the two inboard A-arm bushings while remaining clear
    # of the rotating wishbone barrels.
    for side_index, y0 in enumerate((-0.22, 0.22)):
        for tab_index, sign in enumerate((-1.0, 1.0)):
            chassis.visual(
                Box((0.085, 0.024, 0.108)),
                origin=Origin(xyz=(-0.015, y0 + sign * 0.052, 0.0)),
                material=dark_steel,
                name=f"clevis_tab_{side_index}_{tab_index}",
            )
        _add_tube(
            chassis,
            (-0.057, y0 - 0.052, 0.0),
            (-0.057, y0 + 0.052, 0.0),
            0.010,
            bright_steel,
            name=f"pivot_pin_{side_index}",
        )

    wishbone = model.part("wishbone")
    outer = (0.64, 0.0, -0.045)
    for side_index, y0 in enumerate((-0.22, 0.22)):
        _add_tube(
            wishbone,
            (0.0, y0 - 0.040, 0.0),
            (0.0, y0 + 0.040, 0.0),
            0.032,
            rubber,
            name=f"inner_bushing_{side_index}",
        )
        _add_tube(
            wishbone,
            (0.012, y0, 0.0),
            outer,
            0.018,
            powder_red,
            name=f"arm_tube_{side_index}",
        )
        _add_tube(
            wishbone,
            (0.090, y0 * 0.88, -0.010),
            (0.46, y0 * 0.25, -0.038),
            0.010,
            powder_red,
            name=f"web_tube_{side_index}",
        )

    _add_tube(
        wishbone,
        (0.19, -0.175, -0.012),
        (0.19, 0.175, -0.012),
        0.012,
        powder_red,
        name="cross_tube",
    )
    wishbone.visual(
        Sphere(radius=0.058),
        origin=Origin(xyz=outer),
        material=dark_steel,
        name="outer_joint",
    )
    wishbone.visual(
        Cylinder(radius=0.030, length=0.35),
        origin=Origin(xyz=(outer[0], outer[1], outer[2] - 0.175)),
        material=bright_steel,
        name="king_post",
    )
    wishbone.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(outer[0], outer[1], outer[2] - 0.020)),
        material=dark_steel,
        name="kingpin_cap",
    )

    ski_saddle = model.part("ski_saddle")
    for plate_index, y in enumerate((-0.065, 0.065)):
        ski_saddle.visual(
            Box((0.18, 0.018, 0.30)),
            origin=Origin(xyz=(0.0, y, -0.235)),
            material=dark_steel,
            name=f"side_plate_{plate_index}",
        )
        ski_saddle.visual(
            Box((0.13, 0.016, 0.17)),
            origin=Origin(xyz=(0.0, y, -0.410)),
            material=dark_steel,
            name=f"ski_lug_{plate_index}",
        )

    for bridge_index, x in enumerate((-0.105, 0.105)):
        ski_saddle.visual(
            Box((0.034, 0.150, 0.038)),
            origin=Origin(xyz=(x, 0.0, -0.190)),
            material=dark_steel,
            name=f"offset_bridge_{bridge_index}",
        )
        ski_saddle.visual(
            Box((0.034, 0.135, 0.034)),
            origin=Origin(xyz=(x, 0.0, -0.338)),
            material=dark_steel,
            name=f"lower_bridge_{bridge_index}",
        )

    ski_saddle.visual(
        Cylinder(radius=0.038, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=dark_steel,
        name="steering_sleeve",
    )
    for rib_index, x in enumerate((-0.046, 0.046)):
        ski_saddle.visual(
            Box((0.018, 0.150, 0.035)),
            origin=Origin(xyz=(x, 0.0, -0.225)),
            material=dark_steel,
            name=f"sleeve_rib_{rib_index}",
        )

    ski_saddle.visual(
        Cylinder(radius=0.020, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, -0.415), rpy=(0.0, math.pi / 2.0, math.pi / 2.0)),
        material=bright_steel,
        name="ski_axle",
    )
    ski_saddle.visual(
        Box((0.22, 0.13, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.445)),
        material=dark_steel,
        name="mount_pad",
    )
    ski_saddle.visual(
        Box((1.05, 0.17, 0.035)),
        origin=Origin(xyz=(0.04, 0.0, -0.500)),
        material=plastic_black,
        name="ski_runner",
    )
    ski_saddle.visual(
        Box((0.29, 0.17, 0.034)),
        origin=Origin(xyz=(0.63, 0.0, -0.455), rpy=(0.0, -0.47, 0.0)),
        material=plastic_black,
        name="upturned_tip",
    )
    ski_saddle.visual(
        Box((0.86, 0.024, 0.018)),
        origin=Origin(xyz=(-0.04, 0.0, -0.474)),
        material=bright_steel,
        name="wear_bar",
    )

    model.articulation(
        "chassis_to_wishbone",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=wishbone,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=2.0, lower=-0.28, upper=0.35),
    )
    model.articulation(
        "wishbone_to_ski_saddle",
        ArticulationType.REVOLUTE,
        parent=wishbone,
        child=ski_saddle,
        origin=Origin(xyz=outer),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=75.0, velocity=3.0, lower=-0.72, upper=0.72),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    wishbone = object_model.get_part("wishbone")
    ski_saddle = object_model.get_part("ski_saddle")
    suspension = object_model.get_articulation("chassis_to_wishbone")
    kingpin = object_model.get_articulation("wishbone_to_ski_saddle")

    ctx.check(
        "front ski spindle has the requested three major links",
        {part.name for part in object_model.parts} == {"chassis", "wishbone", "ski_saddle"},
    )
    ctx.allow_overlap(
        wishbone,
        ski_saddle,
        elem_a="king_post",
        elem_b="steering_sleeve",
        reason="The vertical king-post is intentionally represented as captured inside the saddle steering sleeve.",
    )
    ctx.expect_within(
        wishbone,
        ski_saddle,
        axes="xy",
        inner_elem="king_post",
        outer_elem="steering_sleeve",
        margin=0.001,
        name="king-post is centered in the saddle steering sleeve",
    )
    ctx.expect_overlap(
        wishbone,
        ski_saddle,
        axes="z",
        elem_a="king_post",
        elem_b="steering_sleeve",
        min_overlap=0.140,
        name="king-post remains deeply inserted in the saddle sleeve",
    )
    ctx.expect_contact(
        chassis,
        wishbone,
        elem_a="clevis_tab_0_1",
        elem_b="inner_bushing_0",
        contact_tol=0.002,
        name="rear inner bushing is captured by chassis clevis",
    )
    ctx.expect_contact(
        chassis,
        wishbone,
        elem_a="clevis_tab_1_0",
        elem_b="inner_bushing_1",
        contact_tol=0.002,
        name="front inner bushing is captured by chassis clevis",
    )
    ctx.expect_gap(
        ski_saddle,
        wishbone,
        axis="y",
        positive_elem="side_plate_1",
        negative_elem="king_post",
        min_gap=0.010,
        max_gap=0.055,
        name="saddle side plate clears the king-post bearing",
    )
    ctx.expect_gap(
        wishbone,
        ski_saddle,
        axis="y",
        positive_elem="king_post",
        negative_elem="side_plate_0",
        min_gap=0.010,
        max_gap=0.055,
        name="opposite saddle plate clears the king-post bearing",
    )
    ctx.expect_overlap(
        wishbone,
        ski_saddle,
        axes="z",
        elem_a="king_post",
        elem_b="side_plate_0",
        min_overlap=0.20,
        name="saddle straddles the vertical king-post",
    )

    rest_pos = ctx.part_world_position(ski_saddle)
    with ctx.pose({suspension: 0.30}):
        raised_pos = ctx.part_world_position(ski_saddle)
    ctx.check(
        "positive A-arm travel lifts the outer spindle end",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.12,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({kingpin: 0.55}):
        steered_aabb = ctx.part_element_world_aabb(ski_saddle, elem="ski_runner")
    rest_aabb = ctx.part_element_world_aabb(ski_saddle, elem="ski_runner")
    if steered_aabb is None or rest_aabb is None:
        ctx.fail("kingpin steering pose is measurable", "ski runner AABBs were unavailable")
    else:
        rest_center_y = (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
        steered_center_y = (steered_aabb[0][1] + steered_aabb[1][1]) * 0.5
        ctx.check(
            "kingpin revolute joint steers the ski saddle in yaw",
            abs(steered_center_y - rest_center_y) > 0.015,
            details=f"rest_y={rest_center_y}, steered_y={steered_center_y}",
        )

    return ctx.report()


object_model = build_object_model()
