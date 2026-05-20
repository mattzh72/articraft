from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_box")

    wood = model.material("painted_wood", rgba=(0.77, 0.63, 0.46, 1.0))
    lining = model.material("interior_lining", rgba=(0.67, 0.49, 0.34, 1.0))

    outer_x = 0.26
    outer_y = 0.18
    body_h = 0.12
    wall_t = 0.009
    bottom_t = 0.008
    lid_t = 0.008
    wall_h = body_h - bottom_t

    body = model.part("body")
    body.visual(
        Box((outer_x, outer_y, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=wood,
        name="bottom_panel",
    )
    body.visual(
        Box((outer_x, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, outer_y / 2.0 - wall_t / 2.0, bottom_t + wall_h / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((outer_x, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + wall_t / 2.0, bottom_t + wall_h / 2.0)),
        material=wood,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, outer_y - 2.0 * wall_t, wall_h)),
        origin=Origin(
            xyz=(outer_x / 2.0 - wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)
        ),
        material=wood,
        name="right_wall",
    )
    body.visual(
        Box((wall_t, outer_y - 2.0 * wall_t, wall_h)),
        origin=Origin(
            xyz=(-outer_x / 2.0 + wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)
        ),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((outer_x - 2.0 * wall_t, outer_y - 2.0 * wall_t, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t + 0.001)),
        material=lining,
        name="interior_floor",
    )

    lid = model.part("lid")
    lid.visual(
        Box((outer_x, outer_y, lid_t)),
        origin=Origin(xyz=(0.0, outer_y / 2.0, lid_t / 2.0)),
        material=wood,
        name="lid_panel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -outer_y / 2.0, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="lid_panel",
            name="closed lid sits on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.16,
            name="closed lid covers the full body opening",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    closed_top = None if closed_aabb is None else closed_aabb[1][2]
    open_top = None if open_aabb is None else open_aabb[1][2]
    ctx.check(
        "lid opens upward above the box",
        closed_top is not None and open_top is not None and open_top > closed_top + 0.05,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
