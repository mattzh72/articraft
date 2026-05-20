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

    wood = model.material("painted_wood", rgba=(0.70, 0.58, 0.47, 1.0))
    inner_wood = model.material("inner_wood", rgba=(0.60, 0.49, 0.39, 1.0))
    brass = model.material("brass_hinge", rgba=(0.73, 0.61, 0.28, 1.0))

    body_length = 0.260
    body_width = 0.180
    body_height = 0.090
    wall = 0.008
    bottom = 0.006

    lid_length = 0.258
    lid_width = 0.176
    lid_thickness = 0.008
    lid_skirt_depth = 0.014
    lid_skirt_thickness = 0.004

    hinge_radius = 0.005
    hinge_axis_x = -(body_length * 0.5) - 0.002
    hinge_axis_z = body_height + 0.003
    hinge_side_barrel_length = 0.044
    hinge_center_barrel_length = 0.068
    hinge_side_barrel_offset_y = 0.054

    body = model.part("body")
    wall_height = body_height - bottom
    wall_center_z = bottom + wall_height * 0.5

    body.visual(
        Box((body_length, body_width, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom * 0.5)),
        material=inner_wood,
        name="bottom_panel",
    )
    body.visual(
        Box((wall, body_width, wall_height)),
        origin=Origin(xyz=((body_length * 0.5) - (wall * 0.5), 0.0, wall_center_z)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((wall, body_width, wall_height)),
        origin=Origin(xyz=(-(body_length * 0.5) + (wall * 0.5), 0.0, wall_center_z)),
        material=wood,
        name="rear_wall",
    )
    body.visual(
        Box((body_length - (2.0 * wall), wall, wall_height)),
        origin=Origin(xyz=(0.0, (body_width * 0.5) - (wall * 0.5), wall_center_z)),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((body_length - (2.0 * wall), wall, wall_height)),
        origin=Origin(xyz=(0.0, -(body_width * 0.5) + (wall * 0.5), wall_center_z)),
        material=wood,
        name="right_wall",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=hinge_side_barrel_length),
        origin=Origin(
            xyz=(hinge_axis_x, -hinge_side_barrel_offset_y, hinge_axis_z),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=hinge_side_barrel_length),
        origin=Origin(
            xyz=(hinge_axis_x, hinge_side_barrel_offset_y, hinge_axis_z),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    panel_rear_offset = 0.006
    panel_center_x = panel_rear_offset + (lid_length * 0.5)
    panel_center_z = body_height + (lid_thickness * 0.5) - hinge_axis_z
    front_band_length = 0.020
    front_band_center_x = panel_rear_offset + lid_length - (front_band_length * 0.5)

    lid.visual(
        Box((lid_length, lid_width, lid_thickness)),
        origin=Origin(xyz=(panel_center_x, 0.0, panel_center_z)),
        material=wood,
        name="lid_panel",
    )
    lid.visual(
        Box((front_band_length, lid_width, lid_thickness)),
        origin=Origin(xyz=(front_band_center_x, 0.0, panel_center_z)),
        material=wood,
        name="lid_front_band",
    )
    lid.visual(
        Box((0.012, hinge_center_barrel_length, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, 0.004)),
        material=brass,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=hinge_center_barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=brass,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.216, lid_skirt_thickness, lid_skirt_depth)),
        origin=Origin(
            xyz=(0.130, (body_width * 0.5) - wall - (lid_skirt_thickness * 0.5), -0.004),
        ),
        material=inner_wood,
        name="left_lid_skirt",
    )
    lid.visual(
        Box((0.216, lid_skirt_thickness, lid_skirt_depth)),
        origin=Origin(
            xyz=(0.130, -((body_width * 0.5) - wall - (lid_skirt_thickness * 0.5)), -0.004),
        ),
        material=inner_wood,
        name="right_lid_skirt",
    )
    lid.visual(
        Box((0.008, 0.152, lid_skirt_depth)),
        origin=Origin(xyz=(0.246, 0.0, -0.004)),
        material=inner_wood,
        name="front_lid_skirt",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.020)),
        mass=0.24,
        origin=Origin(xyz=(0.135, 0.0, -0.002)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
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
            positive_elem="lid_panel",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid sits on the box top",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="bottom_panel",
            min_overlap=0.170,
            name="lid spans most of the body footprint",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_band")
    with ctx.pose({hinge: 1.20}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_band",
            negative_elem="front_wall",
            min_gap=0.120,
            name="opened lid front edge rises well above the body",
        )
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_band")

    ctx.check(
        "positive hinge motion lifts the front edge",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.120,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
