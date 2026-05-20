from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_box")

    box_finish = model.material("box_finish", rgba=(0.60, 0.45, 0.31, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.66, 0.51, 0.35, 1.0))
    lining = model.material("lining", rgba=(0.23, 0.14, 0.10, 1.0))

    outer_w = 0.320
    outer_d = 0.220
    base_h = 0.140
    bottom_t = 0.014
    wall_t = 0.016

    lid_w = 0.328
    lid_d = 0.224
    lid_skin_t = 0.008
    lid_skirt_t = 0.012
    lid_skirt_drop = 0.012
    lid_clearance = 0.0025

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=lining,
        name="bottom_panel",
    )

    wall_z = base_h / 2.0 + bottom_t / 2.0 - 0.001
    wall_h = base_h - bottom_t + 0.002
    body.visual(
        Box((outer_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -(outer_d - wall_t) / 2.0, wall_z)),
        material=box_finish,
        name="front_wall",
    )
    body.visual(
        Box((outer_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, (outer_d - wall_t) / 2.0, wall_z)),
        material=box_finish,
        name="rear_wall",
    )
    side_d = outer_d - 2.0 * wall_t + 0.002
    body.visual(
        Box((wall_t, side_d, wall_h)),
        origin=Origin(xyz=(-(outer_w - wall_t) / 2.0, 0.0, wall_z)),
        material=box_finish,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, side_d, wall_h)),
        origin=Origin(xyz=((outer_w - wall_t) / 2.0, 0.0, wall_z)),
        material=box_finish,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, outer_d - 2.0 * wall_t, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t + 0.002)),
        material=lining,
        name="inner_floor",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, base_h)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_skin_t)),
        origin=Origin(xyz=(0.0, -lid_d / 2.0, 0.002)),
        material=lid_finish,
        name="lid_skin",
    )

    skirt_outer_w = outer_w - 2.0 * wall_t - 2.0 * lid_clearance
    skirt_outer_d = outer_d - 2.0 * wall_t - 2.0 * lid_clearance
    rear_skirt_clear = 0.020
    front_skirt_t = 0.020
    side_skirt_len = skirt_outer_d - rear_skirt_clear - front_skirt_t + 0.002
    side_skirt_y = -(rear_skirt_clear + side_skirt_len / 2.0)
    side_skirt_x = skirt_outer_w / 2.0 - lid_skirt_t / 2.0
    skirt_z = -(lid_skirt_drop / 2.0 + 0.001)
    front_skirt_y = -(skirt_outer_d - front_skirt_t / 2.0)

    lid.visual(
        Box((lid_skirt_t, side_skirt_len, lid_skirt_drop)),
        origin=Origin(xyz=(-side_skirt_x, side_skirt_y, skirt_z)),
        material=lid_finish,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_skirt_t, side_skirt_len, lid_skirt_drop)),
        origin=Origin(xyz=(side_skirt_x, side_skirt_y, skirt_z)),
        material=lid_finish,
        name="right_skirt",
    )
    lid.visual(
        Box((skirt_outer_w, front_skirt_t, lid_skirt_drop)),
        origin=Origin(xyz=(0.0, front_skirt_y, skirt_z)),
        material=lid_finish,
        name="front_skirt",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_skin_t + lid_skirt_drop)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -lid_d / 2.0, 0.001)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, outer_d / 2.0, base_h + 0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_skin",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed lid sits neatly on the box rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_skin",
        min_overlap=0.20,
        name="closed lid covers the body opening",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_skin")
    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_skin")

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.09
        and open_aabb[0][1] > closed_aabb[0][1] + 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
