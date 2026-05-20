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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_pedal_trash_bin")

    body_metal = model.material("body_metal", color=(0.73, 0.75, 0.77, 1.0))
    dark_plastic = model.material("dark_plastic", color=(0.18, 0.19, 0.20, 1.0))
    pedal_finish = model.material("pedal_finish", color=(0.12, 0.12, 0.13, 1.0))

    body_width = 0.42
    body_depth = 0.42
    body_height = 0.70
    wall_thickness = 0.015
    floor_thickness = 0.018

    lid_width = 0.45
    lid_depth = 0.45
    lid_thickness = 0.024
    lid_skirt_depth = 0.05
    lid_skirt_thickness = 0.010

    hinge_axis_z = body_height

    pedal_axle_radius = 0.009
    pedal_axle_length = 0.18
    pedal_plate_depth = 0.085
    pedal_plate_width = 0.18
    pedal_plate_thickness = 0.014
    pedal_mount_z = 0.055
    pedal_mount_x = body_depth / 2.0 + 0.018

    body = model.part("body")
    body.visual(
        Box((wall_thickness, body_width, body_height)),
        origin=Origin(xyz=(body_depth / 2.0 - wall_thickness / 2.0, 0.0, body_height / 2.0)),
        material=body_metal,
        name="body_front_wall",
    )
    body.visual(
        Box((wall_thickness, body_width, body_height)),
        origin=Origin(xyz=(-body_depth / 2.0 + wall_thickness / 2.0, 0.0, body_height / 2.0)),
        material=body_metal,
        name="body_back_wall",
    )
    body.visual(
        Box((body_depth, wall_thickness, body_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall_thickness / 2.0, body_height / 2.0)),
        material=body_metal,
        name="body_left_wall",
    )
    body.visual(
        Box((body_depth, wall_thickness, body_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall_thickness / 2.0, body_height / 2.0)),
        material=body_metal,
        name="body_right_wall",
    )
    body.visual(
        Box((body_depth - wall_thickness, body_width - wall_thickness, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_metal,
        name="body_floor",
    )
    body.visual(
        Box((0.028, body_width - 0.02, 0.018)),
        origin=Origin(xyz=(-body_depth / 2.0 + 0.014, 0.0, body_height - 0.029)),
        material=dark_plastic,
        name="rear_hinge_strip",
    )
    body.visual(
        Box((0.032, 0.050, 0.050)),
        origin=Origin(xyz=(pedal_mount_x - 0.006, 0.115, pedal_mount_z)),
        material=dark_plastic,
        name="pedal_bracket_left",
    )
    body.visual(
        Box((0.032, 0.050, 0.050)),
        origin=Origin(xyz=(pedal_mount_x - 0.006, -0.115, pedal_mount_z)),
        material=dark_plastic,
        name="pedal_bracket_right",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, lid_thickness / 2.0)),
        material=dark_plastic,
        name="lid_top_panel",
    )
    lid.visual(
        Box((lid_skirt_thickness, lid_width - 0.04, lid_skirt_depth)),
        origin=Origin(
            xyz=(lid_depth - lid_skirt_thickness / 2.0, 0.0, -lid_skirt_depth / 2.0 + lid_thickness / 2.0)
        ),
        material=dark_plastic,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((lid_depth - 0.04, lid_skirt_thickness, lid_skirt_depth)),
        origin=Origin(
            xyz=(lid_depth / 2.0, lid_width / 2.0 - lid_skirt_thickness / 2.0, -lid_skirt_depth / 2.0 + lid_thickness / 2.0)
        ),
        material=dark_plastic,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((lid_depth - 0.04, lid_skirt_thickness, lid_skirt_depth)),
        origin=Origin(
            xyz=(lid_depth / 2.0, -lid_width / 2.0 + lid_skirt_thickness / 2.0, -lid_skirt_depth / 2.0 + lid_thickness / 2.0)
        ),
        material=dark_plastic,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((0.032, lid_width - 0.08, 0.032)),
        origin=Origin(xyz=(0.032, 0.0, -0.004)),
        material=dark_plastic,
        name="lid_rear_leaf",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=pedal_axle_radius, length=pedal_axle_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pedal_finish,
        name="pedal_axle",
    )
    pedal.visual(
        Box((pedal_plate_depth, pedal_plate_width, pedal_plate_thickness)),
        origin=Origin(
            xyz=(0.048, 0.0, 0.012),
            rpy=(0.0, -0.30, 0.0),
        ),
        material=pedal_finish,
        name="pedal_plate",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-body_depth / 2.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(pedal_mount_x, 0.0, pedal_mount_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=0.0,
            upper=0.55,
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
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")

    ctx.check(
        "lid hinge uses rear transverse axis",
        lid_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "pedal hinge uses front transverse axis",
        pedal_hinge.axis == (0.0, 1.0, 0.0),
        details=f"axis={pedal_hinge.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_top_panel",
            max_gap=0.025,
            max_penetration=0.0,
            name="closed lid sits on the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_top_panel",
            min_overlap=0.38,
            name="closed lid covers the square opening",
        )
        ctx.expect_contact(
            pedal,
            body,
            elem_a="pedal_axle",
            name="pedal axle is mounted to the front brackets",
        )
        ctx.expect_origin_gap(
            pedal,
            body,
            axis="x",
            min_gap=0.0,
            name="pedal projects from the front face",
        )

    rest_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    ctx.check(
        "lid front edge rises when opened",
        rest_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[1][2] > rest_lid_front[1][2] + 0.16,
        details=f"rest={rest_lid_front}, open={open_lid_front}",
    )

    rest_pedal_plate = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    with ctx.pose({pedal_hinge: 0.45}):
        pressed_pedal_plate = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    ctx.check(
        "pedal plate rotates downward when pressed",
        rest_pedal_plate is not None
        and pressed_pedal_plate is not None
        and pressed_pedal_plate[0][2] < rest_pedal_plate[0][2] - 0.01,
        details=f"rest={rest_pedal_plate}, pressed={pressed_pedal_plate}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
