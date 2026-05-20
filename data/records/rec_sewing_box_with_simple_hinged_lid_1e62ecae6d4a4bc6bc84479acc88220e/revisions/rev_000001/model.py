from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="small_sewing_box")

    body_color = model.material("body_color", rgba=(0.66, 0.52, 0.39, 1.0))
    lid_color = model.material("lid_color", rgba=(0.83, 0.77, 0.67, 1.0))
    hinge_color = model.material("hinge_color", rgba=(0.48, 0.38, 0.22, 1.0))
    lining_color = model.material("lining_color", rgba=(0.73, 0.63, 0.54, 1.0))

    outer_length = 0.24
    outer_width = 0.16
    box_height = 0.10
    wall_thickness = 0.008
    bottom_thickness = 0.006

    lid_skin_thickness = 0.008
    lid_frame_depth = 0.012
    lid_frame_thickness = 0.008

    hinge_radius = 0.005
    body_knuckle_length = 0.045
    lid_knuckle_length = 0.064
    knuckle_y_offset = 0.0575
    hinge_axis_x = -(outer_length / 2.0) - hinge_radius
    lid_rear_offset = hinge_radius

    inner_length = outer_length - (2.0 * wall_thickness)
    inner_width = outer_width - (2.0 * wall_thickness)
    wall_height = box_height - bottom_thickness

    body = model.part("body")
    body.visual(
        Box((outer_length, outer_width, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material=lining_color,
        name="bottom_panel",
    )
    body.visual(
        Box((wall_thickness, outer_width, wall_height)),
        origin=Origin(
            xyz=((outer_length / 2.0) - (wall_thickness / 2.0), 0.0, bottom_thickness + (wall_height / 2.0))
        ),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((wall_thickness, outer_width, wall_height)),
        origin=Origin(
            xyz=(-(outer_length / 2.0) + (wall_thickness / 2.0), 0.0, bottom_thickness + (wall_height / 2.0))
        ),
        material=body_color,
        name="rear_wall",
    )
    body.visual(
        Box((inner_length, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, (outer_width / 2.0) - (wall_thickness / 2.0), bottom_thickness + (wall_height / 2.0))
        ),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((inner_length, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, -(outer_width / 2.0) + (wall_thickness / 2.0), bottom_thickness + (wall_height / 2.0))
        ),
        material=body_color,
        name="left_wall",
    )
    for suffix, y_center in (("left", -knuckle_y_offset), ("right", knuckle_y_offset)):
        body.visual(
            Box((0.012, body_knuckle_length, 0.016)),
            origin=Origin(xyz=(hinge_axis_x, y_center, box_height - 0.004)),
            material=hinge_color,
            name=f"hinge_leaf_{suffix}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=body_knuckle_length),
            origin=Origin(
                xyz=(hinge_axis_x, y_center, box_height),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_color,
            name=f"hinge_knuckle_{suffix}",
        )
    body.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width, box_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, box_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((outer_length, outer_width, lid_skin_thickness)),
        origin=Origin(xyz=(lid_rear_offset + (outer_length / 2.0), 0.0, lid_skin_thickness / 2.0)),
        material=lid_color,
        name="lid_top_panel",
    )
    lid.visual(
        Box((inner_length - 0.008, lid_frame_thickness, lid_frame_depth)),
        origin=Origin(
            xyz=(
                lid_rear_offset + (outer_length / 2.0),
                (inner_width / 2.0) - (lid_frame_thickness / 2.0),
                -(lid_frame_depth / 2.0),
            )
        ),
        material=lining_color,
        name="lid_side_rail_right",
    )
    lid.visual(
        Box((inner_length - 0.008, lid_frame_thickness, lid_frame_depth)),
        origin=Origin(
            xyz=(
                lid_rear_offset + (outer_length / 2.0),
                -(inner_width / 2.0) + (lid_frame_thickness / 2.0),
                -(lid_frame_depth / 2.0),
            )
        ),
        material=lining_color,
        name="lid_side_rail_left",
    )
    lid.visual(
        Box((lid_frame_thickness, inner_width - 0.008, lid_frame_depth)),
        origin=Origin(
            xyz=(
                lid_rear_offset + outer_length - wall_thickness - (lid_frame_thickness / 2.0),
                0.0,
                -(lid_frame_depth / 2.0),
            )
        ),
        material=lining_color,
        name="lid_front_rail",
    )
    lid.visual(
        Box((0.012, lid_knuckle_length, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, 0.001)),
        material=hinge_color,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=lid_knuckle_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_color,
        name="lid_center_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width, lid_skin_thickness + lid_frame_depth)),
        mass=0.22,
        origin=Origin(xyz=(lid_rear_offset + (outer_length / 2.0), 0.0, -0.002)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, box_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_top_panel",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid sits on the body walls",
        )
        ctx.expect_within(
            lid,
            body,
            axes="xy",
            inner_elem="lid_front_rail",
            outer_elem="bottom_panel",
            margin=0.0,
            name="front locating rail stays within the box footprint",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_top_panel",
            elem_b="bottom_panel",
            min_overlap=0.14,
            name="lid covers the box opening footprint",
        )

    closed_panel = ctx.part_element_world_aabb(lid, elem="lid_top_panel")
    with ctx.pose({lid_hinge: 1.2}):
        opened_panel = ctx.part_element_world_aabb(lid, elem="lid_top_panel")

    ctx.check(
        "lid raises upward when opened",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][2] > closed_panel[1][2] + 0.08,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
