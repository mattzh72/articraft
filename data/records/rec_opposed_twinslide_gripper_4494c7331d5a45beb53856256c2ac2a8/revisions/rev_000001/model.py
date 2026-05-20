from __future__ import annotations

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
    mesh_from_cadquery,
)
import cadquery as cq


SLIDE_TRAVEL = 0.014


def _rounded_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    radius: float,
) -> cq.Workplane:
    """Small machined block with lightly radiused vertical edges."""
    shape = cq.Workplane("XY").box(size[0], size[1], size[2])
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape.translate(center)


def _base_body() -> cq.Workplane:
    """Low stationary spine and guide ways that sit below the moving slides."""
    base = _rounded_box((0.300, 0.310, 0.014), (0.0, 0.0, 0.007), 0.007)
    center_spine = _rounded_box((0.068, 0.292, 0.010), (0.0, 0.0, 0.019), 0.004)

    body = base.union(center_spine)
    for side in (-1.0, 1.0):
        guide = _rounded_box((0.074, 0.250, 0.010), (side * 0.092, 0.0, 0.019), 0.004)
        inner_lip = _rounded_box((0.010, 0.238, 0.010), (side * 0.045, 0.0, 0.019), 0.0025)
        body = body.union(guide).union(inner_lip)

    return body


def _slide_body(side: float) -> cq.Workplane:
    """One moving shallow slide with an integral compact jaw."""
    inward = -side
    plate = _rounded_box((0.078, 0.238, 0.012), (0.0, 0.0, 0.0), 0.004)
    raised_runner = _rounded_box((0.036, 0.218, 0.010), (side * 0.010, 0.0, 0.010), 0.003)
    jaw_neck = _rounded_box((0.034, 0.086, 0.020), (inward * 0.036, 0.0, 0.010), 0.003)
    jaw_block = _rounded_box((0.026, 0.062, 0.036), (inward * 0.054, 0.0, 0.012), 0.0025)
    rear_stop_tab = _rounded_box((0.018, 0.052, 0.015), (side * 0.034, 0.094, 0.006), 0.002)
    front_stop_tab = _rounded_box((0.018, 0.052, 0.015), (side * 0.034, -0.094, 0.006), 0.002)
    return (
        plate.union(raised_runner)
        .union(jaw_neck)
        .union(jaw_block)
        .union(rear_stop_tab)
        .union(front_stop_tab)
    )


def _jaw_pad(side: float) -> cq.Workplane:
    """Rubber jaw insert with three small grip ribs, facing the center gap."""
    inward = -side
    pad = _rounded_box((0.005, 0.054, 0.030), (inward * 0.0695, 0.0, 0.014), 0.0015)
    for z in (0.004, 0.014, 0.024):
        rib = _rounded_box((0.0022, 0.046, 0.0022), (inward * 0.0730, 0.0, z), 0.0007)
        pad = pad.union(rib)
    return pad


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_twin_slide_gripper")

    aluminum = model.material("satin_aluminum", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.18, 0.34, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    work_mark = model.material("work_area_mark", rgba=(0.02, 0.025, 0.026, 1.0))

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(_base_body(), "spine_body", tolerance=0.0006),
        material=aluminum,
        name="spine_body",
    )
    for index, side in enumerate((1.0, -1.0)):
        spine.visual(
            mesh_from_cadquery(
                _rounded_box((0.010, 0.268, 0.017), (side * 0.143, 0.0, 0.0225), 0.003),
                f"outer_lip_{index}",
                tolerance=0.0006,
            ),
            material=aluminum,
            name=f"outer_lip_{index}",
        )
        spine.visual(
            mesh_from_cadquery(
                _rounded_box((0.032, 0.018, 0.022), (side * 0.094, 0.136, 0.025), 0.003),
                f"end_stop_{index}_0",
                tolerance=0.0006,
            ),
            material=aluminum,
            name=f"end_stop_{index}_0",
        )
        spine.visual(
            mesh_from_cadquery(
                _rounded_box((0.032, 0.018, 0.022), (side * 0.094, -0.136, 0.025), 0.003),
                f"end_stop_{index}_1",
                tolerance=0.0006,
            ),
            material=aluminum,
            name=f"end_stop_{index}_1",
        )
    # Dark inset on the top of the spine marks the small rectangular work area.
    spine.visual(
        Box((0.050, 0.074, 0.0014)),
        origin=Origin(xyz=(0.0, 0.0, 0.0244)),
        material=work_mark,
        name="work_area",
    )
    for i, (x, y) in enumerate(
        ((-0.108, -0.124), (-0.108, 0.124), (0.108, -0.124), (0.108, 0.124))
    ):
        spine.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(x, y, 0.0148)),
            material=dark_steel,
            name=f"screw_{i}",
        )

    for index, side in enumerate((1.0, -1.0)):
        slide = model.part(f"slide_{index}")
        slide.visual(
            mesh_from_cadquery(_slide_body(side), f"slide_{index}_body", tolerance=0.0006),
            material=blue_anodized,
            name="slide_body",
        )
        slide.visual(
            mesh_from_cadquery(_jaw_pad(side), f"slide_{index}_pad", tolerance=0.0005),
            material=rubber,
            name="jaw_pad",
        )

        model.articulation(
            f"spine_to_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=spine,
            child=slide,
            origin=Origin(xyz=(side * 0.092, 0.0, 0.030)),
            axis=(-side, 0.0, 0.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.08, lower=0.0, upper=SLIDE_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    slide_0 = object_model.get_part("slide_0")
    slide_1 = object_model.get_part("slide_1")
    joint_0 = object_model.get_articulation("spine_to_slide_0")
    joint_1 = object_model.get_articulation("spine_to_slide_1")

    ctx.check(
        "two independent prismatic slide joints",
        joint_0.articulation_type == ArticulationType.PRISMATIC
        and joint_1.articulation_type == ArticulationType.PRISMATIC
        and joint_0.mimic is None
        and joint_1.mimic is None,
        details=f"joint_0={joint_0.articulation_type}, joint_1={joint_1.articulation_type}",
    )

    ctx.expect_gap(
        slide_0,
        spine,
        axis="z",
        min_gap=0.0,
        max_gap=0.0010,
        positive_elem="slide_body",
        negative_elem="spine_body",
        name="slide_0 rides just above its guide way",
    )
    ctx.expect_gap(
        slide_1,
        spine,
        axis="z",
        min_gap=0.0,
        max_gap=0.0010,
        positive_elem="slide_body",
        negative_elem="spine_body",
        name="slide_1 rides just above its guide way",
    )
    ctx.expect_overlap(
        slide_0,
        slide_1,
        axes="yz",
        elem_a="jaw_pad",
        elem_b="jaw_pad",
        min_overlap=0.025,
        name="opposed jaws share a rectangular work area",
    )
    ctx.expect_gap(
        slide_0,
        slide_1,
        axis="x",
        min_gap=0.034,
        max_gap=0.046,
        positive_elem="jaw_pad",
        negative_elem="jaw_pad",
        name="open jaw gap is modest",
    )

    rest_0 = ctx.part_world_position(slide_0)
    rest_1 = ctx.part_world_position(slide_1)
    with ctx.pose({joint_0: SLIDE_TRAVEL, joint_1: SLIDE_TRAVEL}):
        ctx.expect_gap(
            slide_0,
            slide_1,
            axis="x",
            min_gap=0.006,
            max_gap=0.018,
            positive_elem="jaw_pad",
            negative_elem="jaw_pad",
            name="closed jaws leave a small rectangular work area",
        )
        closed_0 = ctx.part_world_position(slide_0)
        closed_1 = ctx.part_world_position(slide_1)

    ctx.check(
        "both slides move inward when actuated",
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and closed_0[0] < rest_0[0] - 0.010
        and closed_1[0] > rest_1[0] + 0.010,
        details=f"rest_0={rest_0}, closed_0={closed_0}, rest_1={rest_1}, closed_1={closed_1}",
    )

    return ctx.report()


object_model = build_object_model()
