from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_finger_parallel_gripper")

    gunmetal = Material("dark gunmetal anodized steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("matte black rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    rail_mat = Material("polished linear rail steel", rgba=(0.72, 0.74, 0.74, 1.0))
    jaw_mat = Material("brushed aluminum jaw blocks", rgba=(0.55, 0.58, 0.60, 1.0))
    warning = Material("orange jaw witness marks", rgba=(0.95, 0.36, 0.08, 1.0))

    spine = model.part("spine")

    # Dense center spine and guide bed.  The sliding jaws ride above this mass
    # on two cross rails, making the central support visibly heavier than the
    # slender fork fingers.
    spine.visual(
        Box((0.14, 0.52, 0.10)),
        origin=Origin(xyz=(0.0, 0.03, 0.07)),
        material=gunmetal,
        name="center_spine",
    )
    spine.visual(
        Box((0.57, 0.26, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.1325)),
        material=gunmetal,
        name="guide_bed",
    )
    for y, rail_name in ((-0.075, "rail_rear"), (0.075, "rail_front")):
        spine.visual(
            Cylinder(radius=0.012, length=0.56),
            origin=Origin(xyz=(0.0, y, 0.156), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_mat,
            name=rail_name,
        )

    for x, stop_name in ((-0.30, "end_stop_0"), (0.30, "end_stop_1")):
        spine.visual(
            Box((0.035, 0.25, 0.11)),
            origin=Origin(xyz=(x, 0.0, 0.185)),
            material=gunmetal,
            name=stop_name,
        )

    for x in (-0.15, 0.15):
        for y in (-0.095, 0.095):
            spine.visual(
                Cylinder(radius=0.011, length=0.006),
                origin=Origin(xyz=(x, y, 0.148)),
                material=rail_mat,
                name=f"bed_screw_{x:+.2f}_{y:+.2f}",
            )

    def add_fork_jaw(name: str, inner_sign: float) -> object:
        jaw = model.part(name)
        jaw.visual(
            Box((0.075, 0.20, 0.09)),
            origin=Origin(),
            material=jaw_mat,
            name="guide_body",
        )
        jaw.visual(
            Box((0.092, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, -0.110, -0.028)),
            material=warning,
            name="rear_witness_bar",
        )
        for z, label in ((-0.042, "lower"), (0.030, "upper")):
            jaw.visual(
                Box((0.032, 0.36, 0.032)),
                origin=Origin(xyz=(0.0, 0.280, z)),
                material=jaw_mat,
                name=f"finger_{label}",
            )
            jaw.visual(
                Box((0.006, 0.255, 0.024)),
                origin=Origin(xyz=(inner_sign * 0.019, 0.330, z)),
                material=black,
                name=f"grip_pad_{label}",
            )
            jaw.visual(
                Box((0.036, 0.018, 0.036)),
                origin=Origin(xyz=(0.0, 0.469, z)),
                material=black,
                name=f"soft_tip_{label}",
            )
        return jaw

    # jaw_0 is the +X-side jaw and jaw_1 is its mirrored mate.  Each child frame
    # is at the center of its deep guide block; the long fingers are slender
    # cantilevered forks that project forward from that block.
    jaw_0 = add_fork_jaw("jaw_0", inner_sign=-1.0)
    jaw_1 = add_fork_jaw("jaw_1", inner_sign=1.0)

    model.articulation(
        "jaw_0_slide",
        ArticulationType.PRISMATIC,
        parent=spine,
        child=jaw_0,
        origin=Origin(xyz=(0.065, 0.0, 0.213)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=-0.020, upper=0.085),
    )
    model.articulation(
        "jaw_1_slide",
        ArticulationType.PRISMATIC,
        parent=spine,
        child=jaw_1,
        origin=Origin(xyz=(-0.065, 0.0, 0.213)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=-0.020, upper=0.085),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    jaw_0 = object_model.get_part("jaw_0")
    jaw_1 = object_model.get_part("jaw_1")
    slide_0 = object_model.get_articulation("jaw_0_slide")
    slide_1 = object_model.get_articulation("jaw_1_slide")

    ctx.check(
        "two independent prismatic jaw slides",
        slide_0.articulation_type == ArticulationType.PRISMATIC
        and slide_1.articulation_type == ArticulationType.PRISMATIC
        and slide_0.mimic is None
        and slide_1.mimic is None,
        details=f"slide_0={slide_0.articulation_type}, slide_1={slide_1.articulation_type}",
    )

    guide_height = jaw_0.get_visual("guide_body").geometry.size[2]
    finger_height = jaw_0.get_visual("finger_lower").geometry.size[2]
    ctx.check(
        "guide bodies are visibly deeper than fingers",
        guide_height > 2.5 * finger_height,
        details=f"guide_height={guide_height}, finger_height={finger_height}",
    )

    ctx.expect_contact(
        jaw_0,
        spine,
        elem_a="guide_body",
        elem_b="rail_front",
        contact_tol=0.001,
        name="jaw_0 guide rides on the front rail",
    )
    ctx.expect_contact(
        jaw_1,
        spine,
        elem_a="guide_body",
        elem_b="rail_front",
        contact_tol=0.001,
        name="jaw_1 guide rides on the front rail",
    )
    ctx.expect_gap(
        jaw_0,
        jaw_1,
        axis="x",
        positive_elem="guide_body",
        negative_elem="guide_body",
        min_gap=0.010,
        name="neutral guide bodies straddle the center without touching",
    )

    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({slide_0: 0.080}):
        moved_0 = ctx.part_world_position(jaw_0)
        fixed_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({slide_1: 0.080}):
        moved_1 = ctx.part_world_position(jaw_1)
        fixed_0 = ctx.part_world_position(jaw_0)
    ctx.check(
        "jaw_0 opens independently away from center",
        rest_0 is not None
        and moved_0 is not None
        and rest_1 is not None
        and fixed_1 is not None
        and moved_0[0] > rest_0[0] + 0.070
        and abs(fixed_1[0] - rest_1[0]) < 1e-6,
        details=f"rest_0={rest_0}, moved_0={moved_0}, rest_1={rest_1}, fixed_1={fixed_1}",
    )
    ctx.check(
        "jaw_1 opens independently away from center",
        rest_1 is not None
        and moved_1 is not None
        and rest_0 is not None
        and fixed_0 is not None
        and moved_1[0] < rest_1[0] - 0.070
        and abs(fixed_0[0] - rest_0[0]) < 1e-6,
        details=f"rest_1={rest_1}, moved_1={moved_1}, rest_0={rest_0}, fixed_0={fixed_0}",
    )

    with ctx.pose({slide_0: -0.020, slide_1: -0.020}):
        ctx.expect_gap(
            jaw_0,
            jaw_1,
            axis="x",
            positive_elem="grip_pad_lower",
            negative_elem="grip_pad_lower",
            min_gap=0.020,
            name="closing jaws leave a controlled central slot",
        )

    return ctx.report()


object_model = build_object_model()
