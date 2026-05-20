from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reception_desk")

    laminate = model.material("laminate", rgba=(0.71, 0.67, 0.60, 1.0))
    dark_top = model.material("dark_top", rgba=(0.23, 0.22, 0.20, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.69, 0.65, 0.58, 1.0))
    pull = model.material("pull", rgba=(0.22, 0.23, 0.24, 1.0))

    overall_width = 1.80
    overall_depth = 0.78
    rear_plane_y = overall_depth / 2.0
    front_face_y = -overall_depth / 2.0

    counter_depth = 0.34
    counter_height = 1.06
    counter_thickness = 0.04

    work_height = 0.76
    work_thickness = 0.036
    work_depth = 0.56

    shell_thickness = 0.036
    front_panel_thickness = 0.05
    front_panel_height = counter_height - counter_thickness

    right_side_center_x = overall_width / 2.0 - shell_thickness / 2.0
    left_side_center_x = -right_side_center_x

    pedestal_divider_thickness = 0.018
    pedestal_divider_center_x = 0.445
    pedestal_rear_opening_width = 0.410
    drawer_center_x = 0.659
    pedestal_bottom_height = 0.09
    pedestal_height = work_height - work_thickness

    body = model.part("body")
    body.visual(
        Box((overall_width, front_panel_thickness, front_panel_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + front_panel_thickness / 2.0,
                front_panel_height / 2.0,
            )
        ),
        material=laminate,
        name="front_panel",
    )
    body.visual(
        Box((overall_width, counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + counter_depth / 2.0,
                counter_height - counter_thickness / 2.0,
            )
        ),
        material=dark_top,
        name="counter_top",
    )
    body.visual(
        Box((shell_thickness, overall_depth, front_panel_height)),
        origin=Origin(xyz=(left_side_center_x, 0.0, front_panel_height / 2.0)),
        material=laminate,
        name="left_end",
    )
    body.visual(
        Box((shell_thickness, overall_depth, front_panel_height)),
        origin=Origin(xyz=(right_side_center_x, 0.0, front_panel_height / 2.0)),
        material=laminate,
        name="right_end",
    )
    body.visual(
        Box((overall_width - 2.0 * shell_thickness, work_depth, work_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                rear_plane_y - work_depth / 2.0,
                work_height - work_thickness / 2.0,
            )
        ),
        material=dark_top,
        name="work_top",
    )
    body.visual(
        Box((1.30, 0.05, 0.08)),
        origin=Origin(xyz=(-0.214, -0.145, 0.684)),
        material=laminate,
        name="work_apron",
    )
    body.visual(
        Box((pedestal_divider_thickness, work_depth, pedestal_height)),
        origin=Origin(
            xyz=(
                pedestal_divider_center_x,
                rear_plane_y - work_depth / 2.0,
                pedestal_height / 2.0,
            )
        ),
        material=laminate,
        name="pedestal_divider",
    )
    body.visual(
        Box((0.464, work_depth, pedestal_bottom_height)),
        origin=Origin(
            xyz=(
                0.668,
                rear_plane_y - work_depth / 2.0,
                pedestal_bottom_height / 2.0,
            )
        ),
        material=laminate,
        name="pedestal_base",
    )

    rear_frame_thickness = 0.018
    rear_frame_y = rear_plane_y - rear_frame_thickness / 2.0
    body.visual(
        Box((pedestal_rear_opening_width, rear_frame_thickness, 0.035)),
        origin=Origin(xyz=(drawer_center_x, rear_frame_y, 0.1075)),
        material=laminate,
        name="bottom_rail",
    )
    body.visual(
        Box((pedestal_rear_opening_width, rear_frame_thickness, 0.018)),
        origin=Origin(xyz=(drawer_center_x, rear_frame_y, 0.305)),
        material=laminate,
        name="mid_rail_0",
    )
    body.visual(
        Box((pedestal_rear_opening_width, rear_frame_thickness, 0.018)),
        origin=Origin(xyz=(drawer_center_x, rear_frame_y, 0.494)),
        material=laminate,
        name="mid_rail_1",
    )
    body.visual(
        Box((pedestal_rear_opening_width, rear_frame_thickness, 0.05)),
        origin=Origin(xyz=(drawer_center_x, rear_frame_y, 0.699)),
        material=laminate,
        name="top_rail",
    )

    drawer_front_width = 0.400
    drawer_front_height = 0.163
    drawer_front_thickness = 0.018
    drawer_box_width = 0.370
    drawer_box_depth = 0.45
    drawer_box_height = 0.11
    drawer_side_thickness = 0.012
    drawer_bottom_thickness = 0.01
    drawer_box_center_y = -(drawer_front_thickness + drawer_box_depth / 2.0)
    drawer_box_center_z = -0.02
    drawer_centers_z = (0.2105, 0.3995, 0.5885)

    for index, drawer_center_z in enumerate(drawer_centers_z):
        runner_center_z = drawer_center_z - 0.02
        body.visual(
            Box((0.010, 0.32, 0.03)),
            origin=Origin(xyz=(0.459, 0.21, runner_center_z)),
            material=pull,
            name=f"runner_left_{index}",
        )
        body.visual(
            Box((0.010, 0.32, 0.03)),
            origin=Origin(xyz=(0.859, 0.21, runner_center_z)),
            material=pull,
            name=f"runner_right_{index}",
        )

        drawer = model.part(f"drawer_{index}")
        drawer.visual(
            Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
            origin=Origin(xyz=(0.0, -drawer_front_thickness / 2.0, 0.0)),
            material=drawer_face,
            name="front",
        )
        drawer.visual(
            Box((drawer_box_width, drawer_box_depth, drawer_bottom_thickness)),
            origin=Origin(
                xyz=(
                    0.0,
                    drawer_box_center_y,
                    -drawer_box_height / 2.0,
                )
            ),
            material=drawer_face,
            name="box_bottom",
        )
        drawer.visual(
            Box((drawer_side_thickness, drawer_box_depth, drawer_box_height)),
            origin=Origin(
                xyz=(
                    -drawer_box_width / 2.0 + drawer_side_thickness / 2.0,
                    drawer_box_center_y,
                    drawer_box_center_z,
                )
            ),
            material=drawer_face,
            name="box_side_0",
        )
        drawer.visual(
            Box((drawer_side_thickness, drawer_box_depth, drawer_box_height)),
            origin=Origin(
                xyz=(
                    drawer_box_width / 2.0 - drawer_side_thickness / 2.0,
                    drawer_box_center_y,
                    drawer_box_center_z,
                )
            ),
            material=drawer_face,
            name="box_side_1",
        )
        drawer.visual(
            Box((drawer_box_width, drawer_side_thickness, drawer_box_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    -(drawer_front_thickness + drawer_box_depth - drawer_side_thickness / 2.0),
                    drawer_box_center_z,
                )
            ),
            material=drawer_face,
            name="box_back",
        )
        drawer.visual(
            Box((0.010, 0.32, 0.03)),
            origin=Origin(xyz=(-0.190, -0.16, -0.02)),
            material=pull,
            name="runner_left",
        )
        drawer.visual(
            Box((0.010, 0.32, 0.03)),
            origin=Origin(xyz=(0.190, -0.16, -0.02)),
            material=pull,
            name="runner_right",
        )
        drawer.visual(
            Box((0.130, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, 0.025, 0.0)),
            material=pull,
            name="pull_bar",
        )
        drawer.visual(
            Box((0.010, 0.020, 0.010)),
            origin=Origin(xyz=(-0.045, 0.010, 0.0)),
            material=pull,
            name="pull_post_0",
        )
        drawer.visual(
            Box((0.010, 0.020, 0.010)),
            origin=Origin(xyz=(0.045, 0.010, 0.0)),
            material=pull,
            name="pull_post_1",
        )

        model.articulation(
            f"body_to_drawer_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(drawer_center_x, rear_plane_y, drawer_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.30,
                lower=0.0,
                upper=0.28,
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    body_aabb = ctx.part_world_aabb(body)
    counter_aabb = ctx.part_element_world_aabb(body, elem="counter_top")
    work_aabb = ctx.part_element_world_aabb(body, elem="work_top")
    front_panel_aabb = ctx.part_element_world_aabb(body, elem="front_panel")

    ctx.check(
        "work surface sits well below the visitor counter",
        counter_aabb is not None
        and work_aabb is not None
        and counter_aabb[0][2] >= work_aabb[1][2] + 0.22,
        details=f"counter={counter_aabb}, work={work_aabb}",
    )
    ctx.check(
        "front modesty panel rises above the work surface",
        front_panel_aabb is not None
        and work_aabb is not None
        and front_panel_aabb[1][2] >= work_aabb[1][2] + 0.24,
        details=f"front_panel={front_panel_aabb}, work={work_aabb}",
    )

    for index in range(3):
        drawer = object_model.get_part(f"drawer_{index}")
        slide = object_model.get_articulation(f"body_to_drawer_{index}")
        limits = slide.motion_limits
        upper = 0.0 if limits is None or limits.upper is None else limits.upper

        rest_pos = ctx.part_world_position(drawer)
        front_rest = ctx.part_element_world_aabb(drawer, elem="front")

        ctx.check(
            f"drawer_{index} closes flush with the pedestal rear",
            body_aabb is not None
            and front_rest is not None
            and abs(front_rest[1][1] - body_aabb[1][1]) <= 0.002,
            details=f"body={body_aabb}, drawer_front={front_rest}",
        )

        with ctx.pose({slide: upper}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="box_bottom",
                min_overlap=0.18,
                name=f"drawer_{index} keeps retained insertion when extended",
            )

        ctx.check(
            f"drawer_{index} extends outward from the desk",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] >= rest_pos[1] + 0.24,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
