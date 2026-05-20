from __future__ import annotations

import math

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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_beam(
    part,
    *,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    size_x: float,
    size_y: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((size_x, size_y, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_h_frame_easel")

    wood = model.material("wood", rgba=(0.62, 0.46, 0.29, 1.0))
    darker_wood = model.material("darker_wood", rgba=(0.45, 0.31, 0.18, 1.0))
    metal = model.material("metal", rgba=(0.33, 0.34, 0.36, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.10, 1.0))
    grip = model.material("grip", rgba=(0.16, 0.13, 0.09, 1.0))

    frame = model.part("frame")

    frame.visual(
        Box((0.07, 0.54, 0.06)),
        origin=Origin(xyz=(-0.22, -0.02, 0.03)),
        material=wood,
        name="runner_0",
    )
    frame.visual(
        Box((0.07, 0.54, 0.06)),
        origin=Origin(xyz=(0.22, -0.02, 0.03)),
        material=wood,
        name="runner_1",
    )
    frame.visual(
        Box((0.51, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.22, 0.09)),
        material=darker_wood,
        name="front_base_rail",
    )
    frame.visual(
        Box((0.44, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, -0.26, 0.065)),
        material=darker_wood,
        name="rear_base_rail",
    )

    frame.visual(
        Box((0.07, 0.05, 1.36)),
        origin=Origin(xyz=(-0.22, 0.21, 0.74)),
        material=wood,
        name="upright_0",
    )
    frame.visual(
        Box((0.07, 0.05, 1.36)),
        origin=Origin(xyz=(0.22, 0.21, 0.74)),
        material=wood,
        name="upright_1",
    )
    frame.visual(
        Box((0.50, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.21, 1.45)),
        material=darker_wood,
        name="top_header",
    )
    frame.visual(
        Box((0.49, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.21, 0.82)),
        material=darker_wood,
        name="mid_header",
    )

    frame.visual(
        Box((0.08, 0.06, 1.56)),
        origin=Origin(xyz=(0.0, 0.17, 0.84)),
        material=wood,
        name="mast",
    )
    frame.visual(
        Box((0.18, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.17, 0.60)),
        material=darker_wood,
        name="gearbox",
    )
    frame.visual(
        Box((0.15, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.17, 1.30)),
        material=darker_wood,
        name="upper_guide",
    )
    frame.visual(
        Box((0.16, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.19, 1.56)),
        material=darker_wood,
        name="top_clamp",
    )
    frame.visual(
        Box((0.05, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.205, 1.47)),
        material=darker_wood,
        name="clamp_stem",
    )

    _add_beam(
        frame,
        a=(-0.22, -0.24, 0.06),
        b=(-0.20, 0.18, 1.42),
        size_x=0.045,
        size_y=0.05,
        material=wood,
        name="rear_brace_0",
    )
    _add_beam(
        frame,
        a=(0.22, -0.24, 0.06),
        b=(0.20, 0.18, 1.42),
        size_x=0.045,
        size_y=0.05,
        material=wood,
        name="rear_brace_1",
    )
    frame.visual(
        Box((0.43, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.015, 0.89)),
        material=darker_wood,
        name="rear_tie",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.025, 0.13, 0.18)),
        origin=Origin(xyz=(-0.0525, 0.0, 0.09)),
        material=wood,
        name="guide_0",
    )
    tray.visual(
        Box((0.025, 0.13, 0.18)),
        origin=Origin(xyz=(0.0525, 0.0, 0.09)),
        material=wood,
        name="guide_1",
    )
    tray.visual(
        Box((0.17, 0.02, 0.18)),
        origin=Origin(xyz=(0.0, 0.075, 0.09)),
        material=darker_wood,
        name="guide_bridge",
    )
    tray.visual(
        Box((0.58, 0.11, 0.03)),
        origin=Origin(xyz=(0.0, 0.13, 0.015)),
        material=wood,
        name="tray_shelf",
    )
    tray.visual(
        Box((0.58, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.175, 0.04)),
        material=darker_wood,
        name="tray_lip",
    )
    tray.visual(
        Box((0.12, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.09, 0.045)),
        material=darker_wood,
        name="center_block",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.010, length=0.04),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub",
    )
    crank.visual(
        Box((0.012, 0.018, 0.11)),
        origin=Origin(xyz=(0.034, 0.0, -0.055)),
        material=metal,
        name="arm",
    )
    crank.visual(
        Cylinder(radius=0.008, length=0.06),
        origin=Origin(xyz=(0.038, 0.0, -0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip,
        name="grip",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.038, 0.0, -0.088)),
        material=metal,
        name="grip_spindle",
    )

    tray_slide = model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.17, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.92),
    )
    tray_slide.meta["qc_samples"] = [0.0, 0.46, 0.92]

    crank_spin = model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.09, 0.17, 0.60)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    crank_spin.meta["qc_samples"] = [0.0, math.pi / 2.0, math.pi]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    crank = object_model.get_part("crank")
    tray_slide = object_model.get_articulation("tray_slide")
    crank_spin = object_model.get_articulation("crank_spin")

    ctx.expect_gap(
        tray,
        frame,
        axis="z",
        min_gap=0.18,
        max_gap=0.36,
        positive_elem="tray_shelf",
        negative_elem="front_base_rail",
        name="tray shelf starts above the lower frame rail",
    )
    ctx.expect_overlap(
        tray,
        frame,
        axes="x",
        min_overlap=0.06,
        elem_a="guide_bridge",
        elem_b="mast",
        name="tray carriage stays centered on the mast",
    )
    ctx.expect_gap(
        crank,
        frame,
        axis="x",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem="shaft",
        negative_elem="gearbox",
        name="crank shaft emerges flush from the gearbox side",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="yz",
        min_overlap=0.02,
        elem_a="hub",
        elem_b="gearbox",
        name="crank hub stays aligned with the adjustment housing",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_gap(
            tray,
            frame,
            axis="z",
            min_gap=1.08,
            max_gap=1.30,
            positive_elem="tray_shelf",
            negative_elem="front_base_rail",
            name="raised tray reaches the upper half of the mast",
        )
        tray_high = ctx.part_world_position(tray)
    ctx.check(
        "tray moves upward along the mast",
        tray_rest is not None
        and tray_high is not None
        and tray_high[2] > tray_rest[2] + 0.85,
        details=f"rest={tray_rest}, high={tray_high}",
    )

    grip_start = ctx.part_element_world_aabb(crank, elem="grip")
    with ctx.pose({crank_spin: math.pi / 2.0}):
        grip_quarter = ctx.part_element_world_aabb(crank, elem="grip")
    ctx.check(
        "crank grip traces a rotated position around the short shaft",
        grip_start is not None
        and grip_quarter is not None
        and abs(grip_quarter[0][2] - grip_start[0][2]) > 0.04,
        details=f"start={grip_start}, quarter={grip_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
