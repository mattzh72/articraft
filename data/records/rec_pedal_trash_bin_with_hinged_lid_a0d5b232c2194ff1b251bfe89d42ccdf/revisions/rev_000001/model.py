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
    model = ArticulatedObject(name="kitchen_pedal_trash_bin")

    stainless = model.material("stainless_steel", rgba=(0.77, 0.79, 0.80, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    charcoal = model.material("charcoal_trim", rgba=(0.21, 0.22, 0.23, 1.0))

    body = model.part("body")
    lid = model.part("lid")
    pedal = model.part("pedal")

    body_height = 0.62
    body_width = 0.30
    body_depth = 0.26
    wall = 0.005
    overlap = 0.002

    # Main thin-walled rectangular shell with an open top.
    body.visual(
        Box((body_depth, wall, body_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall / 2.0, body_height / 2.0)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((body_depth, wall, body_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall / 2.0, body_height / 2.0)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((wall, body_width - 2.0 * wall + overlap, body_height)),
        origin=Origin(xyz=(body_depth / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=stainless,
        name="front_wall",
    )
    body.visual(
        Box((wall, body_width - 2.0 * wall + overlap, body_height)),
        origin=Origin(xyz=(-body_depth / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=stainless,
        name="back_wall",
    )
    body.visual(
        Box((body_depth - 2.0 * wall + overlap, body_width - 2.0 * wall + overlap, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=stainless,
        name="bottom_plate",
    )

    # Lower front brackets that visibly support the pedal pivot.
    pedal_bracket_size = (0.018, 0.018, 0.072)
    pedal_bracket_x = body_depth / 2.0 + 0.001
    pedal_bracket_y = 0.119
    pedal_bracket_z = pedal_bracket_size[2] / 2.0
    body.visual(
        Box(pedal_bracket_size),
        origin=Origin(xyz=(pedal_bracket_x, pedal_bracket_y, pedal_bracket_z)),
        material=charcoal,
        name="right_pedal_bracket",
    )
    body.visual(
        Box(pedal_bracket_size),
        origin=Origin(xyz=(pedal_bracket_x, -pedal_bracket_y, pedal_bracket_z)),
        material=charcoal,
        name="left_pedal_bracket",
    )

    # Rear hinge knuckles on the shell.
    hinge_radius = 0.004
    shell_hinge_length = 0.048
    hinge_x = -body_depth / 2.0 - 0.004
    hinge_z = body_height - hinge_radius
    hinge_y = 0.104
    body.visual(
        Cylinder(radius=hinge_radius, length=shell_hinge_length),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=shell_hinge_length),
        origin=Origin(xyz=(hinge_x, -hinge_y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_hinge_barrel",
    )

    # Lid as a simple cap with shallow side skirts and a center hinge knuckle.
    lid_depth = 0.264
    lid_width = 0.304
    lid_thickness = 0.012
    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, 0.010)),
        material=dark_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.250, 0.005, 0.026)),
        origin=Origin(xyz=(0.125, 0.1535, -0.005)),
        material=dark_plastic,
        name="right_lid_skirt",
    )
    lid.visual(
        Box((0.250, 0.005, 0.026)),
        origin=Origin(xyz=(0.125, -0.1535, -0.005)),
        material=dark_plastic,
        name="left_lid_skirt",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="lid_barrel",
    )

    # Wide foot pedal with a cross tube and short support arms.
    pedal.visual(
        Box((0.064, 0.224, 0.012)),
        origin=Origin(xyz=(0.040, 0.0, -0.010)),
        material=dark_plastic,
        name="pedal_plate",
    )
    pedal.visual(
        Cylinder(radius=0.006, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="pedal_axle",
    )
    pedal.visual(
        Box((0.020, 0.012, 0.028)),
        origin=Origin(xyz=(0.010, 0.096, -0.012)),
        material=charcoal,
        name="right_pedal_arm",
    )
    pedal.visual(
        Box((0.020, 0.012, 0.028)),
        origin=Origin(xyz=(0.010, -0.096, -0.012)),
        material=charcoal,
        name="left_pedal_arm",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(body_depth / 2.0 + 0.008, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=4.0,
            lower=-0.12,
            upper=0.45,
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

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            max_gap=0.01,
            max_penetration=0.0,
            name="closed lid sits just above the shell rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.24,
            name="closed lid spans the bin opening",
        )
        ctx.expect_gap(
            pedal,
            body,
            axis="x",
            positive_elem="pedal_plate",
            negative_elem="front_wall",
            min_gap=0.008,
            max_gap=0.025,
            name="pedal projects forward from the lower front face",
        )

        closed_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
        closed_pedal = ctx.part_element_world_aabb(pedal, elem="pedal_plate")

    with ctx.pose({lid_hinge: 1.10}):
        open_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({pedal_hinge: 0.35}):
        pressed_pedal = ctx.part_element_world_aabb(pedal, elem="pedal_plate")

    lid_opens = (
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10
        and open_lid[1][0] < closed_lid[1][0] - 0.10
    )
    ctx.check(
        "lid rotates upward from the rear hinge",
        lid_opens,
        details=f"closed_lid={closed_lid}, open_lid={open_lid}",
    )

    pedal_presses = (
        closed_pedal is not None
        and pressed_pedal is not None
        and pressed_pedal[0][2] < closed_pedal[0][2] - 0.01
        and pressed_pedal[1][0] < closed_pedal[1][0]
    )
    ctx.check(
        "pedal rotates downward when pressed",
        pedal_presses,
        details=f"closed_pedal={closed_pedal}, pressed_pedal={pressed_pedal}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
