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
    model = ArticulatedObject(name="offroad_glove_compartment")

    dashboard_plastic = model.material(
        "dashboard_plastic",
        rgba=(0.17, 0.18, 0.19, 1.0),
    )
    cavity_liner = model.material(
        "cavity_liner",
        rgba=(0.09, 0.10, 0.11, 1.0),
    )
    door_plastic = model.material(
        "door_plastic",
        rgba=(0.12, 0.13, 0.14, 1.0),
    )
    latch_trim = model.material(
        "latch_trim",
        rgba=(0.05, 0.05, 0.05, 1.0),
    )
    hinge_metal = model.material(
        "hinge_metal",
        rgba=(0.42, 0.44, 0.46, 1.0),
    )

    fascia_width = 0.48
    fascia_height = 0.30
    fascia_thickness = 0.014
    opening_width = 0.33
    opening_height = 0.19
    cavity_depth = 0.18
    wall = 0.010
    bezel_margin_y = (fascia_width - opening_width) / 2.0
    bezel_margin_z = (fascia_height - opening_height) / 2.0

    door_width = opening_width - 0.006
    door_height = opening_height - 0.006
    door_thickness = 0.012
    door_gap_x = 0.001
    door_plane_x = fascia_thickness + door_gap_x + door_thickness / 2.0
    hinge_axis_y = -(opening_width / 2.0 + 0.006)

    dashboard_body = model.part("dashboard_body")

    dashboard_body.visual(
        Box((fascia_thickness, fascia_width, bezel_margin_z)),
        origin=Origin(
            xyz=(
                fascia_thickness / 2.0,
                0.0,
                opening_height / 2.0 + bezel_margin_z / 2.0,
            )
        ),
        material=dashboard_plastic,
        name="fascia_top",
    )
    dashboard_body.visual(
        Box((fascia_thickness, fascia_width, bezel_margin_z)),
        origin=Origin(
            xyz=(
                fascia_thickness / 2.0,
                0.0,
                -(opening_height / 2.0 + bezel_margin_z / 2.0),
            )
        ),
        material=dashboard_plastic,
        name="fascia_bottom",
    )
    dashboard_body.visual(
        Box((fascia_thickness, bezel_margin_y, opening_height)),
        origin=Origin(
            xyz=(
                fascia_thickness / 2.0,
                -(opening_width / 2.0 + bezel_margin_y / 2.0),
                0.0,
            )
        ),
        material=dashboard_plastic,
        name="fascia_left",
    )
    dashboard_body.visual(
        Box((fascia_thickness, bezel_margin_y, opening_height)),
        origin=Origin(
            xyz=(
                fascia_thickness / 2.0,
                opening_width / 2.0 + bezel_margin_y / 2.0,
                0.0,
            )
        ),
        material=dashboard_plastic,
        name="fascia_right",
    )

    cavity_outer_width = opening_width + 2.0 * wall
    cavity_outer_height = opening_height + 2.0 * wall

    dashboard_body.visual(
        Box((cavity_depth, cavity_outer_width, wall)),
        origin=Origin(
            xyz=(
                -cavity_depth / 2.0,
                0.0,
                (cavity_outer_height - wall) / 2.0,
            )
        ),
        material=cavity_liner,
        name="cavity_top_wall",
    )
    dashboard_body.visual(
        Box((cavity_depth, cavity_outer_width, wall)),
        origin=Origin(
            xyz=(
                -cavity_depth / 2.0,
                0.0,
                -(cavity_outer_height - wall) / 2.0,
            )
        ),
        material=cavity_liner,
        name="cavity_bottom_wall",
    )
    dashboard_body.visual(
        Box((cavity_depth, wall, opening_height)),
        origin=Origin(
            xyz=(
                -cavity_depth / 2.0,
                -(cavity_outer_width - wall) / 2.0,
                0.0,
            )
        ),
        material=cavity_liner,
        name="cavity_left_wall",
    )
    dashboard_body.visual(
        Box((cavity_depth, wall, opening_height)),
        origin=Origin(
            xyz=(
                -cavity_depth / 2.0,
                (cavity_outer_width - wall) / 2.0,
                0.0,
            )
        ),
        material=cavity_liner,
        name="cavity_right_wall",
    )
    dashboard_body.visual(
        Box((wall, cavity_outer_width, cavity_outer_height)),
        origin=Origin(
            xyz=(
                -(cavity_depth - wall / 2.0),
                0.0,
                0.0,
            )
        ),
        material=cavity_liner,
        name="cavity_back",
    )

    dashboard_body.visual(
        Box((fascia_thickness, 0.016, opening_height)),
        origin=Origin(xyz=(fascia_thickness / 2.0, hinge_axis_y, 0.0)),
        material=dashboard_plastic,
        name="hinge_side_strap",
    )
    dashboard_body.visual(
        Box((0.020, 0.012, 0.026)),
        origin=Origin(xyz=(0.010, hinge_axis_y, 0.059)),
        material=dashboard_plastic,
        name="hinge_upper_bracket",
    )
    dashboard_body.visual(
        Box((0.020, 0.012, 0.026)),
        origin=Origin(xyz=(0.010, hinge_axis_y, -0.059)),
        material=dashboard_plastic,
        name="hinge_lower_bracket",
    )
    dashboard_body.visual(
        Cylinder(radius=0.0055, length=0.048),
        origin=Origin(xyz=(door_plane_x, hinge_axis_y, 0.059)),
        material=hinge_metal,
        name="hinge_upper_knuckle",
    )
    dashboard_body.visual(
        Cylinder(radius=0.0055, length=0.048),
        origin=Origin(xyz=(door_plane_x, hinge_axis_y, -0.059)),
        material=hinge_metal,
        name="hinge_lower_knuckle",
    )

    glovebox_door = model.part("glovebox_door")
    panel_center_y = opening_width / 2.0 + 0.006

    glovebox_door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(0.0, panel_center_y, 0.0)),
        material=door_plastic,
        name="door_outer_panel",
    )
    glovebox_door.visual(
        Box((0.040, door_width - 0.024, door_height - 0.024)),
        origin=Origin(xyz=(-0.026, panel_center_y, 0.0)),
        material=door_plastic,
        name="door_inner_lip",
    )
    glovebox_door.visual(
        Box((0.004, door_width - 0.090, 0.020)),
        origin=Origin(xyz=(door_thickness / 2.0 + 0.002, panel_center_y, 0.050)),
        material=door_plastic,
        name="upper_rib",
    )
    glovebox_door.visual(
        Box((0.004, door_width - 0.090, 0.020)),
        origin=Origin(xyz=(door_thickness / 2.0 + 0.002, panel_center_y, -0.050)),
        material=door_plastic,
        name="lower_rib",
    )
    glovebox_door.visual(
        Box((door_thickness, 0.014, 0.070)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=door_plastic,
        name="door_hinge_leaf",
    )
    glovebox_door.visual(
        Cylinder(radius=0.0055, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_metal,
        name="door_center_knuckle",
    )
    glovebox_door.visual(
        Box((0.004, 0.066, 0.098)),
        origin=Origin(xyz=(door_thickness / 2.0 + 0.002, 0.291, -0.003)),
        material=door_plastic,
        name="latch_pocket_frame",
    )

    paddle_latch = model.part("paddle_latch")
    paddle_latch.visual(
        Cylinder(radius=0.004, length=0.044),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="latch_pivot_pin",
    )
    paddle_latch.visual(
        Box((0.007, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=latch_trim,
        name="latch_paddle",
    )
    paddle_latch.visual(
        Box((0.011, 0.040, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, -0.066)),
        material=latch_trim,
        name="latch_pull_tab",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=dashboard_body,
        child=glovebox_door,
        origin=Origin(xyz=(door_plane_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=0.0,
            upper=1.7,
        ),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=glovebox_door,
        child=paddle_latch,
        origin=Origin(xyz=(door_thickness / 2.0 + 0.0045, 0.291, 0.032)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=0.65,
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

    dashboard_body = object_model.get_part("dashboard_body")
    glovebox_door = object_model.get_part("glovebox_door")
    paddle_latch = object_model.get_part("paddle_latch")
    door_hinge = object_model.get_articulation("body_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.check(
        "door hinge uses a vertical opening axis",
        tuple(round(v, 3) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "paddle latch uses a short horizontal pivot",
        tuple(round(v, 3) for v in latch_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={latch_joint.axis}",
    )

    ctx.expect_overlap(
        glovebox_door,
        dashboard_body,
        axes="yz",
        elem_a="door_outer_panel",
        min_overlap=0.18,
        name="closed door covers the storage opening footprint",
    )
    ctx.expect_within(
        paddle_latch,
        glovebox_door,
        axes="yz",
        inner_elem="latch_paddle",
        outer_elem="door_outer_panel",
        margin=0.010,
        name="paddle latch stays within the door face boundary",
    )

    closed_door_aabb = ctx.part_element_world_aabb(glovebox_door, elem="door_outer_panel")
    fascia_aabb = ctx.part_element_world_aabb(dashboard_body, elem="fascia_top")
    if closed_door_aabb is not None and fascia_aabb is not None:
        flush_gap = closed_door_aabb[0][0] - fascia_aabb[1][0]
        ctx.check(
            "door sits nearly flush in front of the fascia",
            0.0005 <= flush_gap <= 0.0035,
            details=f"flush_gap={flush_gap}",
        )
    else:
        ctx.fail("door flush check has exact geometry", f"door={closed_door_aabb}, fascia={fascia_aabb}")

    closed_latch_aabb = ctx.part_element_world_aabb(paddle_latch, elem="latch_paddle")

    with ctx.pose({door_hinge: 1.2}):
        opened_door_aabb = ctx.part_element_world_aabb(glovebox_door, elem="door_outer_panel")
        ctx.check(
            "door swings outward from the dashboard",
            closed_door_aabb is not None
            and opened_door_aabb is not None
            and opened_door_aabb[1][0] > closed_door_aabb[1][0] + 0.10,
            details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
        )

    with ctx.pose({latch_joint: 0.45}):
        opened_latch_aabb = ctx.part_element_world_aabb(paddle_latch, elem="latch_paddle")
        ctx.check(
            "paddle latch rotates outward for finger access",
            closed_latch_aabb is not None
            and opened_latch_aabb is not None
            and opened_latch_aabb[1][0] > closed_latch_aabb[1][0] + 0.010,
            details=f"closed={closed_latch_aabb}, opened={opened_latch_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
