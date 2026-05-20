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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_vise")

    body_cast = model.material("body_cast", rgba=(0.20, 0.35, 0.49, 1.0))
    cover_paint = model.material("cover_paint", rgba=(0.17, 0.29, 0.42, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.80, 0.81, 0.83, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.38, 0.41, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.24, 0.20, 0.025)),
        origin=Origin(xyz=(-0.01, 0.0, 0.0125)),
        material=body_cast,
        name="base_plate",
    )
    body.visual(
        Box((0.11, 0.12, 0.095)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0475)),
        material=body_cast,
        name="pedestal",
    )
    body.visual(
        Box((0.19, 0.025, 0.08)),
        origin=Origin(xyz=(0.095, 0.0375, 0.10)),
        material=body_cast,
        name="guide_left",
    )
    body.visual(
        Box((0.19, 0.025, 0.08)),
        origin=Origin(xyz=(0.095, -0.0375, 0.10)),
        material=body_cast,
        name="guide_right",
    )
    body.visual(
        Box((0.12, 0.145, 0.04)),
        origin=Origin(xyz=(-0.06, 0.0, 0.16)),
        material=body_cast,
        name="rear_bridge",
    )
    body.visual(
        Box((0.03, 0.145, 0.11)),
        origin=Origin(xyz=(-0.015, 0.0, 0.195)),
        material=body_cast,
        name="fixed_jaw_body",
    )
    body.visual(
        Box((0.008, 0.135, 0.075)),
        origin=Origin(xyz=(0.004, 0.0, 0.1825)),
        material=jaw_steel,
        name="fixed_jaw_insert",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(-0.116, 0.063, 0.182), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(-0.116, -0.063, 0.182), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_1",
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.16, 0.05, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, 0.04)),
        material=dark_steel,
        name="guide_tongue",
    )
    moving_jaw.visual(
        Box((0.035, 0.145, 0.105)),
        origin=Origin(xyz=(0.0175, 0.0, 0.1375)),
        material=body_cast,
        name="jaw_body",
    )
    moving_jaw.visual(
        Box((0.008, 0.135, 0.075)),
        origin=Origin(xyz=(0.004, 0.0, 0.1225)),
        material=jaw_steel,
        name="jaw_insert",
    )
    moving_jaw.visual(
        Box((0.058, 0.045, 0.06)),
        origin=Origin(xyz=(0.139, 0.0, 0.05)),
        material=body_cast,
        name="screw_boss",
    )
    moving_jaw.visual(
        Box((0.10, 0.12, 0.05)),
        origin=Origin(xyz=(0.07, 0.0, 0.145)),
        material=body_cast,
        name="upper_cap",
    )
    moving_jaw.visual(
        Box((0.05, 0.04, 0.05)),
        origin=Origin(xyz=(0.04, 0.0, 0.075)),
        material=body_cast,
        name="guide_neck",
    )
    moving_jaw.visual(
        Box((0.06, 0.045, 0.08)),
        origin=Origin(xyz=(0.10, 0.0, 0.095)),
        material=body_cast,
        name="nose_web",
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.02, length=0.048),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_steel,
        name="hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.009, length=0.22),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_steel,
        name="bar",
    )
    screw_handle.visual(
        Cylinder(radius=0.014, length=0.03),
        origin=Origin(xyz=(0.024, 0.102, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_steel,
        name="grip_0",
    )
    screw_handle.visual(
        Cylinder(radius=0.014, length=0.03),
        origin=Origin(xyz=(0.024, -0.102, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_steel,
        name="grip_1",
    )

    anvil_cover = model.part("anvil_cover")
    anvil_cover.visual(
        Box((0.08, 0.104, 0.012)),
        origin=Origin(xyz=(0.04, 0.0, 0.004)),
        material=cover_paint,
        name="cover_panel",
    )
    anvil_cover.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cover_barrel",
    )

    jaw_slide = model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moving_jaw,
        origin=Origin(xyz=(0.014, 0.0, 0.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.12,
            lower=0.0,
            upper=0.10,
        ),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw_handle,
        origin=Origin(xyz=(0.168, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )
    cover_hinge = model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=anvil_cover,
        origin=Origin(xyz=(-0.116, 0.0, 0.182)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    jaw_slide.meta["default_pose"] = 0.0
    cover_hinge.meta["default_pose"] = 0.0

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    moving_jaw = object_model.get_part("moving_jaw")
    screw_handle = object_model.get_part("screw_handle")
    anvil_cover = object_model.get_part("anvil_cover")

    jaw_slide = object_model.get_articulation("jaw_slide")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.expect_gap(
        moving_jaw,
        body,
        axis="x",
        positive_elem="jaw_insert",
        negative_elem="fixed_jaw_insert",
        min_gap=0.004,
        max_gap=0.010,
        name="jaw starts nearly closed",
    )
    ctx.expect_gap(
        body,
        moving_jaw,
        axis="y",
        positive_elem="guide_left",
        negative_elem="guide_tongue",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="tongue bears on the left guide face",
    )
    ctx.expect_gap(
        moving_jaw,
        body,
        axis="y",
        positive_elem="guide_tongue",
        negative_elem="guide_right",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="tongue bears on the right guide face",
    )
    ctx.expect_gap(
        anvil_cover,
        body,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="rear_bridge",
        min_gap=0.0,
        max_gap=0.002,
        name="cover sits flush on the anvil pad",
    )
    ctx.expect_contact(
        screw_handle,
        moving_jaw,
        elem_a="hub",
        elem_b="screw_boss",
        contact_tol=5e-4,
        name="handle seats against the screw boss",
    )

    closed_jaw_pos = ctx.part_world_position(moving_jaw)
    closed_cover_aabb = ctx.part_element_world_aabb(anvil_cover, elem="cover_panel")

    with ctx.pose({jaw_slide: 0.10}):
        open_jaw_pos = ctx.part_world_position(moving_jaw)
        ctx.expect_overlap(
            moving_jaw,
            body,
            axes="x",
            elem_a="guide_tongue",
            elem_b="guide_left",
            min_overlap=0.07,
            name="guide tongue stays engaged at full opening",
        )

    ctx.check(
        "moving jaw opens forward",
        closed_jaw_pos is not None
        and open_jaw_pos is not None
        and open_jaw_pos[0] > closed_jaw_pos[0] + 0.08,
        details=f"closed={closed_jaw_pos}, open={open_jaw_pos}",
    )

    with ctx.pose({cover_hinge: 1.05}):
        open_cover_aabb = ctx.part_element_world_aabb(anvil_cover, elem="cover_panel")

    ctx.check(
        "cover flips up from the rear hinge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.06,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
