from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    body_color = model.material("painted_body", color=(0.82, 0.72, 0.62))
    lid_color = model.material("plain_lid", color=(0.74, 0.63, 0.54))
    lining_color = model.material("interior_shadow", color=(0.57, 0.42, 0.32))

    outer_w = 0.24
    outer_d = 0.16
    outer_h = 0.09
    wall_t = 0.008
    bottom_t = 0.008

    lid_w = outer_w + 0.008
    lid_d = outer_d + 0.004
    lid_t = 0.008

    wall_h = outer_h - bottom_t

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=body_color,
        name="bottom_panel",
    )
    body.visual(
        Box((outer_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -(outer_d - wall_t) / 2.0, bottom_t + wall_h / 2.0)),
        material=body_color,
        name="rear_wall",
    )
    body.visual(
        Box((outer_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, (outer_d - wall_t) / 2.0, bottom_t + wall_h / 2.0)),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((wall_t, outer_d - 2.0 * wall_t, wall_h)),
        origin=Origin(xyz=(-(outer_w - wall_t) / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d - 2.0 * wall_t, wall_h)),
        origin=Origin(xyz=((outer_w - wall_t) / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, outer_d - 2.0 * wall_t, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t + 0.001)),
        material=lining_color,
        name="inner_bottom",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, lid_t / 2.0)),
        material=lid_color,
        name="lid_panel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t)),
        mass=0.18,
        origin=Origin(xyz=(0.0, lid_d / 2.0, lid_t / 2.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -outer_d / 2.0, outer_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=2.1,
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
            negative_elem="rear_wall",
            name="closed lid seats on the body rim at the hinge edge",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.14,
            elem_a="lid_panel",
            name="closed lid covers the box opening footprint",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10
        and open_aabb[0][1] < closed_aabb[0][1] + 0.01,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
