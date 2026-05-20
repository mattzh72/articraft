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
    model = ArticulatedObject(name="sewing_box")

    body_wood = model.material("body_wood", rgba=(0.62, 0.43, 0.27, 1.0))
    lid_wood = model.material("lid_wood", rgba=(0.70, 0.50, 0.31, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.65, 0.28, 1.0))
    liner = model.material("liner", rgba=(0.75, 0.59, 0.49, 1.0))

    body_width = 0.28
    body_depth = 0.18
    body_height = 0.11
    wall_thickness = 0.008
    floor_thickness = 0.008

    lid_width = 0.254
    lid_depth = 0.186
    lid_thickness = 0.010

    bracket_width = 0.012
    bracket_depth = 0.032
    bracket_height = 0.040
    bracket_y = -0.094

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_wood,
        name="floor",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, wall_thickness, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - wall_thickness / 2.0,
                floor_thickness + (body_height - floor_thickness) / 2.0,
            )
        ),
        material=body_wood,
        name="front_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, wall_thickness, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -body_depth / 2.0 + wall_thickness / 2.0,
                floor_thickness + (body_height - floor_thickness) / 2.0,
            )
        ),
        material=body_wood,
        name="back_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - wall_thickness / 2.0,
                0.0,
                floor_thickness + (body_height - floor_thickness) / 2.0,
            )
        ),
        material=body_wood,
        name="right_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth, body_height - floor_thickness)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + wall_thickness / 2.0,
                0.0,
                floor_thickness + (body_height - floor_thickness) / 2.0,
            )
        ),
        material=body_wood,
        name="left_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, body_depth - 2.0 * wall_thickness, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness + 0.002)),
        material=liner,
        name="inner_liner",
    )
    body.visual(
        Box((bracket_width, bracket_depth, bracket_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - bracket_width / 2.0,
                bracket_y,
                body_height - 0.006,
            )
        ),
        material=body_wood,
        name="right_bracket",
    )
    body.visual(
        Box((bracket_width, bracket_depth, bracket_height)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + bracket_width / 2.0,
                bracket_y,
                body_height - 0.006,
            )
        ),
        material=body_wood,
        name="left_bracket",
    )
    body.visual(
        Box((0.048, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 + 0.001, body_height * 0.54)),
        material=brass,
        name="front_pull",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, lid_depth / 2.0, lid_thickness / 2.0)),
        material=lid_wood,
        name="lid_panel",
    )
    lid.visual(
        Box((0.016, 0.018, 0.022)),
        origin=Origin(xyz=(0.120, -0.005, -0.007)),
        material=lid_wood,
        name="right_hinge_ear",
    )
    lid.visual(
        Box((0.016, 0.018, 0.022)),
        origin=Origin(xyz=(-0.120, -0.005, -0.007)),
        material=lid_wood,
        name="left_hinge_ear",
    )
    lid.visual(
        Box((0.110, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, lid_depth - 0.012, 0.004)),
        material=brass,
        name="lid_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_thickness + 0.020)),
        mass=0.45,
        origin=Origin(xyz=(0.0, lid_depth / 2.0, 0.004)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.096, body_height + 0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="right_hinge_ear",
            elem_b="right_bracket",
            name="right hinge ear bears on the right bracket",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_hinge_ear",
            elem_b="left_bracket",
            name="left hinge ear bears on the left bracket",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            min_gap=0.001,
            max_gap=0.004,
            name="lid rests just above the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="floor",
            min_overlap=0.17,
            name="lid panel covers the storage opening footprint",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.6}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid swings upward about the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.11,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
