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
    model = ArticulatedObject(name="small_sewing_box")

    body_wood = model.material("body_wood", color=(0.53, 0.35, 0.23))
    lid_finish = model.material("lid_finish", color=(0.80, 0.72, 0.60))
    hardware = model.material("hardware", color=(0.62, 0.55, 0.43))

    outer_w = 0.32
    outer_d = 0.21
    body_h = 0.11
    lid_t = 0.012

    left_wall_t = 0.012
    right_wall_t = 0.036
    front_wall_t = 0.012
    rear_wall_t = 0.016
    floor_t = 0.008
    seam = 0.001

    clear_w = outer_w - left_wall_t - right_wall_t
    clear_d = outer_d - front_wall_t - rear_wall_t

    body = model.part("body")
    body.visual(
        Box((outer_w - left_wall_t - right_wall_t + 2.0 * seam, clear_d, floor_t)),
        origin=Origin(
            xyz=(
                (left_wall_t - right_wall_t) * 0.5,
                (rear_wall_t - front_wall_t) * 0.5,
                floor_t * 0.5,
            )
        ),
        material=body_wood,
        name="floor",
    )
    body.visual(
        Box((left_wall_t, outer_d, body_h)),
        origin=Origin(xyz=(-outer_w * 0.5 + left_wall_t * 0.5, 0.0, body_h * 0.5)),
        material=body_wood,
        name="left_wall",
    )
    body.visual(
        Box((right_wall_t, outer_d, body_h)),
        origin=Origin(xyz=(outer_w * 0.5 - right_wall_t * 0.5, 0.0, body_h * 0.5)),
        material=body_wood,
        name="right_wall",
    )
    body.visual(
        Box((clear_w + 2.0 * seam, front_wall_t, body_h)),
        origin=Origin(
            xyz=(
                (left_wall_t - right_wall_t) * 0.5,
                -outer_d * 0.5 + front_wall_t * 0.5,
                body_h * 0.5,
            )
        ),
        material=body_wood,
        name="front_wall",
    )
    body.visual(
        Box((clear_w + 2.0 * seam, rear_wall_t, body_h)),
        origin=Origin(
            xyz=(
                (left_wall_t - right_wall_t) * 0.5,
                outer_d * 0.5 - rear_wall_t * 0.5,
                body_h * 0.5,
            )
        ),
        material=body_wood,
        name="rear_wall",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((outer_w, outer_d, lid_t)),
        origin=Origin(xyz=(0.0, -outer_d * 0.5, lid_t * 0.5)),
        material=lid_finish,
        name="lid_panel",
    )
    lid.visual(
        Box((outer_w, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, -0.007, 0.003)),
        material=hardware,
        name="hinge_leaf",
    )
    lid.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, lid_t)),
        mass=0.42,
        origin=Origin(xyz=(0.0, -outer_d * 0.5, lid_t * 0.5)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, outer_d * 0.5, body_h)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.9,
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
        positive_elem="lid_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid rests on the body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.16,
        name="lid panel covers the box opening footprint",
    )

    right_wall = ctx.part_element_world_aabb(body, elem="right_wall")
    left_wall = ctx.part_element_world_aabb(body, elem="left_wall")
    if right_wall is not None and left_wall is not None:
        right_width = right_wall[1][0] - right_wall[0][0]
        left_width = left_wall[1][0] - left_wall[0][0]
        ctx.check(
            "body is heavier on one side",
            right_width > left_width + 0.015,
            details=f"left_width={left_width:.4f}, right_width={right_width:.4f}",
        )
    else:
        ctx.fail("body is heavier on one side", "could not measure side wall widths")

    closed_lid = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.2}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward about the rear edge",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.08,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
