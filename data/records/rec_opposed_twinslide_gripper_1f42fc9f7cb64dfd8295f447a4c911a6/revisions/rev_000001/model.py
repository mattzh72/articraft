from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _finger_mesh(
    *,
    base_x: float = 0.035,
    length: float = 0.465,
    base_width: float = 0.020,
    tip_width: float = 0.013,
    thickness: float = 0.012,
):
    """Thin tapered fork tine with a rounded nose, extruded in local Z."""
    tip_x = base_x + length
    radius = tip_width * 0.5
    nose_center_x = tip_x - radius
    profile: list[tuple[float, float]] = [
        (base_x, -base_width * 0.5),
        (nose_center_x, -tip_width * 0.5),
    ]
    for i in range(1, 10):
        angle = -math.pi * 0.5 + math.pi * (i / 10.0)
        profile.append(
            (
                nose_center_x + math.cos(angle) * radius,
                math.sin(angle) * radius,
            )
        )
    profile.extend(
        [
            (nose_center_x, tip_width * 0.5),
            (base_x, base_width * 0.5),
        ]
    )
    return ExtrudeGeometry.centered(profile, thickness)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wafer_handling_gripper")

    block_mat = model.material("black_anodized", color=(0.02, 0.025, 0.03, 1.0))
    plate_mat = model.material("satin_aluminum", color=(0.55, 0.58, 0.60, 1.0))
    rail_mat = model.material("polished_steel", color=(0.78, 0.80, 0.82, 1.0))
    carriage_mat = model.material("carriage_graphite", color=(0.07, 0.075, 0.08, 1.0))
    ceramic_mat = model.material("wafer_ceramic", color=(0.90, 0.88, 0.80, 1.0))
    pad_mat = model.material("soft_contact_pad", color=(0.02, 0.018, 0.015, 1.0))

    body = model.part("center_block")
    body.visual(
        Box((0.160, 0.150, 0.085)),
        origin=Origin(xyz=(-0.100, 0.0, 0.0425)),
        material=block_mat,
        name="actuator_block",
    )
    body.visual(
        Box((0.070, 0.230, 0.040)),
        origin=Origin(xyz=(-0.005, 0.0, 0.020)),
        material=block_mat,
        name="front_bridge",
    )
    body.visual(
        Box((0.150, 0.390, 0.026)),
        origin=Origin(xyz=(0.100, 0.0, 0.013)),
        material=plate_mat,
        name="guide_base",
    )

    body.visual(
        Box((0.018, 0.175, 0.018)),
        origin=Origin(xyz=(0.065, 0.0925, 0.0345)),
        material=rail_mat,
        name="upper_guide_rail_0",
    )
    body.visual(
        Box((0.018, 0.175, 0.018)),
        origin=Origin(xyz=(0.135, 0.0925, 0.0345)),
        material=rail_mat,
        name="upper_guide_rail_1",
    )
    body.visual(
        Box((0.018, 0.175, 0.018)),
        origin=Origin(xyz=(0.065, -0.0925, 0.0345)),
        material=rail_mat,
        name="lower_guide_rail_0",
    )
    body.visual(
        Box((0.018, 0.175, 0.018)),
        origin=Origin(xyz=(0.135, -0.0925, 0.0345)),
        material=rail_mat,
        name="lower_guide_rail_1",
    )

    for stop_name, stop_y in (("upper", 0.185), ("lower", -0.185)):
        body.visual(
            Box((0.145, 0.016, 0.052)),
            origin=Origin(xyz=(0.100, stop_y, 0.040)),
            material=block_mat,
            name=f"{stop_name}_travel_stop",
        )

    for screw_i, (sx, sy) in enumerate(
        ((-0.140, -0.050), (-0.060, -0.050), (-0.140, 0.050), (-0.060, 0.050))
    ):
        body.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(sx, sy, 0.0865)),
            material=rail_mat,
            name=f"cap_screw_{screw_i}",
        )

    finger_mesh = mesh_from_geometry(_finger_mesh(), "tapered_fork_finger")

    def make_jaw(name: str, *, inner_sign: float):
        jaw = model.part(name)
        jaw.visual(
            Box((0.130, 0.040, 0.038)),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=carriage_mat,
            name="carriage",
        )
        jaw.visual(
            Box((0.095, 0.032, 0.018)),
            origin=Origin(xyz=(-0.004, 0.0, 0.047)),
            material=carriage_mat,
            name="carriage_cap",
        )
        jaw.visual(
            finger_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.028)),
            material=ceramic_mat,
            name="finger",
        )
        jaw.visual(
            Box((0.205, 0.0045, 0.006)),
            origin=Origin(xyz=(0.355, inner_sign * 0.0095, 0.034)),
            material=pad_mat,
            name="wafer_pad",
        )
        jaw.visual(
            Box((0.060, 0.007, 0.006)),
            origin=Origin(xyz=(0.470, inner_sign * 0.0077, 0.035)),
            material=pad_mat,
            name="tip_pad",
        )
        return jaw

    upper_jaw = make_jaw("upper_jaw", inner_sign=-1.0)
    lower_jaw = make_jaw("lower_jaw", inner_sign=1.0)

    slide_limits = MotionLimits(effort=24.0, velocity=0.16, lower=-0.060, upper=0.035)
    model.articulation(
        "upper_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_jaw,
        origin=Origin(xyz=(0.100, 0.085, 0.0435)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=slide_limits,
    )
    model.articulation(
        "lower_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_jaw,
        origin=Origin(xyz=(0.100, -0.085, 0.0435)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=slide_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("center_block")
    upper_jaw = object_model.get_part("upper_jaw")
    lower_jaw = object_model.get_part("lower_jaw")
    upper_slide = object_model.get_articulation("upper_slide")
    lower_slide = object_model.get_articulation("lower_slide")

    ctx.expect_gap(
        upper_jaw,
        body,
        axis="z",
        positive_elem="carriage",
        negative_elem="upper_guide_rail_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="upper carriage rides on fixed guide",
    )
    ctx.expect_gap(
        lower_jaw,
        body,
        axis="z",
        positive_elem="carriage",
        negative_elem="lower_guide_rail_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="lower carriage rides on fixed guide",
    )
    ctx.expect_within(
        upper_jaw,
        body,
        axes="y",
        inner_elem="carriage",
        outer_elem="upper_guide_rail_0",
        margin=0.003,
        name="upper carriage stays inside upper guide length",
    )
    ctx.expect_within(
        lower_jaw,
        body,
        axes="y",
        inner_elem="carriage",
        outer_elem="lower_guide_rail_0",
        margin=0.003,
        name="lower carriage stays inside lower guide length",
    )

    rest_upper = ctx.part_world_position(upper_jaw)
    rest_lower = ctx.part_world_position(lower_jaw)

    with ctx.pose({upper_slide: -0.055, lower_slide: -0.055}):
        inward_upper = ctx.part_world_position(upper_jaw)
        inward_lower = ctx.part_world_position(lower_jaw)
        ctx.expect_gap(
            upper_jaw,
            lower_jaw,
            axis="y",
            positive_elem="finger",
            negative_elem="finger",
            min_gap=0.020,
            max_gap=0.070,
            name="fork fingers close without colliding",
        )
        ctx.expect_within(
            upper_jaw,
            body,
            axes="y",
            inner_elem="carriage",
            outer_elem="upper_guide_rail_0",
            margin=0.003,
            name="upper carriage remains guided when inward",
        )
        ctx.expect_within(
            lower_jaw,
            body,
            axes="y",
            inner_elem="carriage",
            outer_elem="lower_guide_rail_0",
            margin=0.003,
            name="lower carriage remains guided when inward",
        )

    with ctx.pose({upper_slide: 0.030, lower_slide: 0.030}):
        outward_upper = ctx.part_world_position(upper_jaw)
        outward_lower = ctx.part_world_position(lower_jaw)

    ctx.check(
        "slides move inward toward center",
        rest_upper is not None
        and rest_lower is not None
        and inward_upper is not None
        and inward_lower is not None
        and abs(inward_upper[1]) < abs(rest_upper[1])
        and abs(inward_lower[1]) < abs(rest_lower[1]),
        details=f"rest=({rest_upper}, {rest_lower}), inward=({inward_upper}, {inward_lower})",
    )
    ctx.check(
        "slides move outward away from center",
        rest_upper is not None
        and rest_lower is not None
        and outward_upper is not None
        and outward_lower is not None
        and abs(outward_upper[1]) > abs(rest_upper[1])
        and abs(outward_lower[1]) > abs(rest_lower[1]),
        details=f"rest=({rest_upper}, {rest_lower}), outward=({outward_upper}, {outward_lower})",
    )

    return ctx.report()


object_model = build_object_model()
