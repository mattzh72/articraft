from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="small_sewing_box")

    body_material = model.material("painted_wood", rgba=(0.57, 0.40, 0.28, 1.0))
    lid_material = model.material("dark_stain", rgba=(0.41, 0.28, 0.18, 1.0))
    hinge_material = model.material("brass", rgba=(0.76, 0.64, 0.36, 1.0))

    body = model.part("body")
    lid = model.part("lid")

    body_depth = 0.19
    body_width = 0.28
    body_height = 0.11
    wall = 0.008
    capture = 0.001

    lid_thickness = 0.010
    lid_drop = 0.016
    lid_skirt_thickness = 0.006
    lid_depth = body_depth + 0.012
    lid_width = body_width + 0.014
    lid_back_offset = 0.004

    hinge_radius = 0.008
    hinge_axis_x = -body_depth / 2.0 - hinge_radius - 0.001
    hinge_axis_z = body_height
    body_knuckle_length = 0.09
    lid_knuckle_length = 0.08
    knuckle_center_offset_y = 0.089

    wall_height = body_height - wall + capture
    wall_center_z = (body_height + wall - capture) / 2.0
    side_wall_depth = body_depth - 2.0 * wall + capture

    body.visual(
        Box((body_depth, body_width, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=body_material,
        name="floor",
    )
    body.visual(
        Box((wall, body_width, wall_height)),
        origin=Origin(xyz=(body_depth / 2.0 - wall / 2.0, 0.0, wall_center_z)),
        material=body_material,
        name="front_wall",
    )
    body.visual(
        Box((wall, body_width, wall_height)),
        origin=Origin(xyz=(-body_depth / 2.0 + wall / 2.0, 0.0, wall_center_z)),
        material=body_material,
        name="rear_wall",
    )
    body.visual(
        Box((side_wall_depth, wall, wall_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall / 2.0, wall_center_z)),
        material=body_material,
        name="left_wall",
    )
    body.visual(
        Box((side_wall_depth, wall, wall_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall / 2.0, wall_center_z)),
        material=body_material,
        name="right_wall",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=body_knuckle_length),
        origin=Origin(
            xyz=(hinge_axis_x, -knuckle_center_offset_y, hinge_axis_z),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_material,
        name="left_hinge_knuckle",
    )
    body.visual(
        Box((0.010, body_knuckle_length, 0.012)),
        origin=Origin(
            xyz=(hinge_axis_x + hinge_radius / 2.0, -knuckle_center_offset_y, body_height - 0.004),
        ),
        material=hinge_material,
        name="left_hinge_mount",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=body_knuckle_length),
        origin=Origin(
            xyz=(hinge_axis_x, knuckle_center_offset_y, hinge_axis_z),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_material,
        name="right_hinge_knuckle",
    )
    body.visual(
        Box((0.010, body_knuckle_length, 0.012)),
        origin=Origin(
            xyz=(hinge_axis_x + hinge_radius / 2.0, knuckle_center_offset_y, body_height - 0.004),
        ),
        material=hinge_material,
        name="right_hinge_mount",
    )

    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        origin=Origin(
            xyz=(lid_back_offset + lid_depth / 2.0, 0.0, lid_thickness / 2.0),
        ),
        material=lid_material,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_skirt_thickness, lid_width, lid_drop)),
        origin=Origin(
            xyz=(lid_back_offset + lid_depth - lid_skirt_thickness / 2.0, 0.0, -lid_drop / 2.0),
        ),
        material=lid_material,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_depth - lid_skirt_thickness, lid_skirt_thickness, lid_drop)),
        origin=Origin(
            xyz=(
                lid_back_offset + (lid_depth - lid_skirt_thickness) / 2.0,
                lid_width / 2.0 - lid_skirt_thickness / 2.0,
                -lid_drop / 2.0,
            ),
        ),
        material=lid_material,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_depth - lid_skirt_thickness, lid_skirt_thickness, lid_drop)),
        origin=Origin(
            xyz=(
                lid_back_offset + (lid_depth - lid_skirt_thickness) / 2.0,
                -lid_width / 2.0 + lid_skirt_thickness / 2.0,
                -lid_drop / 2.0,
            ),
        ),
        material=lid_material,
        name="right_skirt",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=lid_knuckle_length),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_material,
        name="lid_hinge_knuckle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
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
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

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
            name="closed lid sits on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.18,
            name="lid covers the sewing box body",
        )
        closed_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    lid_opens = (
        closed_panel is not None
        and open_panel is not None
        and ((open_panel[0][2] + open_panel[1][2]) / 2.0)
        > ((closed_panel[0][2] + closed_panel[1][2]) / 2.0) + 0.045
        and ((open_panel[0][0] + open_panel[1][0]) / 2.0)
        < ((closed_panel[0][0] + closed_panel[1][0]) / 2.0) - 0.03
    )
    ctx.check(
        "lid opens upward from the rear hinge",
        lid_opens,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
