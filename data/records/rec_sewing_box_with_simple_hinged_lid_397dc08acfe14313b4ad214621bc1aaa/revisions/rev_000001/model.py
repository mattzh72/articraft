from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_box")

    outer_w = 0.220
    outer_d = 0.150
    body_h = 0.090
    wall_t = 0.006
    bottom_t = 0.006

    overhang = 0.003
    lid_w = outer_w + 2.0 * overhang
    lid_d = outer_d + 2.0 * overhang
    lid_t = 0.008
    skirt_t = 0.0025
    skirt_drop = 0.013

    hinge_r = 0.004
    hinge_axis_y = -outer_d / 2.0 - 0.003
    hinge_axis_z = body_h - hinge_r
    body_leaf_t = 0.002
    body_leaf_h = 0.022
    lid_leaf_d = 0.014
    lid_leaf_t = 0.003
    hinge_outer_len = 0.018
    hinge_center_len = 0.022
    hinge_spacing = 0.006
    left_hinge_x = -(hinge_center_len / 2.0 + hinge_spacing + hinge_outer_len / 2.0)
    right_hinge_x = -left_hinge_x

    wood = model.material("wood", rgba=(0.61, 0.43, 0.27, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.29, 1.0))

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=wood,
        name="bottom",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=(-(outer_w - wall_t) / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, body_h)),
        origin=Origin(xyz=((outer_w - wall_t) / 2.0, 0.0, body_h / 2.0)),
        material=wood,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, (outer_d - wall_t) / 2.0, body_h / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(outer_d - wall_t) / 2.0, body_h / 2.0)),
        material=wood,
        name="rear_wall",
    )

    for name, hinge_x in (("hinge_left", left_hinge_x), ("hinge_right", right_hinge_x)):
        body.visual(
            Box((hinge_outer_len, body_leaf_t, body_leaf_h)),
            origin=Origin(
                xyz=(
                    hinge_x,
                    -outer_d / 2.0 - body_leaf_t / 2.0,
                    hinge_axis_z - body_leaf_h / 2.0 + 0.002,
                )
            ),
            material=brass,
            name=f"{name}_leaf",
        )
        body.visual(
            Cylinder(radius=hinge_r, length=hinge_outer_len),
            origin=Origin(
                xyz=(hinge_x, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=brass,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(
            xyz=(
                0.0,
                lid_d / 2.0,
                hinge_r + lid_t / 2.0 + 0.0005,
            )
        ),
        material=wood,
        name="panel",
    )
    lid.visual(
        Box((lid_w - 2.0 * skirt_t, skirt_t, skirt_drop)),
        origin=Origin(
            xyz=(
                0.0,
                lid_d - skirt_t / 2.0,
                hinge_r + 0.0005 - skirt_drop / 2.0,
            )
        ),
        material=wood,
        name="front_skirt",
    )
    lid.visual(
        Box((skirt_t, lid_d - skirt_t, skirt_drop)),
        origin=Origin(
            xyz=(
                -(lid_w - skirt_t) / 2.0,
                lid_d / 2.0,
                hinge_r + 0.0005 - skirt_drop / 2.0,
            )
        ),
        material=wood,
        name="left_skirt",
    )
    lid.visual(
        Box((skirt_t, lid_d - skirt_t, skirt_drop)),
        origin=Origin(
            xyz=(
                (lid_w - skirt_t) / 2.0,
                lid_d / 2.0,
                hinge_r + 0.0005 - skirt_drop / 2.0,
            )
        ),
        material=wood,
        name="right_skirt",
    )
    lid.visual(
        Box((hinge_center_len, lid_leaf_d, lid_leaf_t)),
        origin=Origin(
            xyz=(
                0.0,
                lid_leaf_d / 2.0,
                hinge_r + 0.0015,
            )
        ),
        material=brass,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=hinge_center_len),
        origin=Origin(
            xyz=(0.0, 0.0, hinge_r),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=brass,
        name="hinge_center",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t + skirt_drop)),
        mass=0.28,
        origin=Origin(xyz=(0.0, lid_d / 2.0, (lid_t + skirt_drop) / 2.0)),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("rear_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="panel",
            negative_elem="front_wall",
            max_gap=0.0015,
            max_penetration=0.0,
            name="closed lid sits just above the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="panel",
            elem_b="bottom",
            min_overlap=0.145,
            name="lid panel covers the box body footprint",
        )
        closed_front_skirt = ctx.part_element_world_aabb(lid, elem="front_skirt")

    with ctx.pose({hinge: 1.95}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="front_wall",
            min_gap=0.055,
            name="opened lid lifts the front edge well above the body",
        )
        open_front_skirt = ctx.part_element_world_aabb(lid, elem="front_skirt")

    ctx.check(
        "lid front edge rises when opened",
        closed_front_skirt is not None
        and open_front_skirt is not None
        and open_front_skirt[0][2] > closed_front_skirt[0][2] + 0.055,
        details=f"closed={closed_front_skirt}, open={open_front_skirt}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
