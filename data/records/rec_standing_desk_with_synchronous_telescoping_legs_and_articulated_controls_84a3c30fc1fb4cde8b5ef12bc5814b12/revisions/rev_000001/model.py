from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LIFT_TRAVEL = 0.35
OUTER_TOP_Z = 0.58
INNER_POST_TOP_LOCAL_Z = 0.19
TOP_FRAME_Z = INNER_POST_TOP_LOCAL_Z + 0.0495
CONTROL_POD_ORIGIN = (0.43, -0.245, -0.025)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    wood = model.material("warm_maple_top", rgba=(0.78, 0.58, 0.36, 1.0))
    wood_edge = model.material("rounded_edge_banding", rgba=(0.64, 0.43, 0.24, 1.0))
    metal = model.material("satin_black_metal", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_metal = model.material("dark_column_sleeve", rgba=(0.025, 0.027, 0.030, 1.0))
    inner_metal = model.material("brushed_inner_stage", rgba=(0.48, 0.50, 0.52, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    label = model.material("white_control_marking", rgba=(0.93, 0.94, 0.92, 1.0))

    lower = model.part("lower_frame")

    # Compact home-office footprint: two small T-feet tied together by a low
    # crossmember so the root reads as one supported welded/base assembly.
    for index, x in enumerate((-0.32, 0.32)):
        lower.visual(
            Box((0.080, 0.560, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.020)),
            material=metal,
            name=f"foot_{index}",
        )
    lower.visual(
        Box((0.720, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=metal,
        name="floor_crossbar",
    )

    column_height = 0.50
    column_center_z = 0.08 + column_height * 0.5
    for index, x in enumerate((-0.32, 0.32)):
        front_wall_name = "column_0_front_wall" if index == 0 else "column_1_front_wall"
        top_lip_name = "column_0_top_lip" if index == 0 else "column_1_top_lip"
        lower.visual(
            Box((0.090, 0.010, column_height)),
            origin=Origin(xyz=(x, -0.023, column_center_z)),
            material=dark_metal,
            name=front_wall_name,
        )
        lower.visual(
            Box((0.090, 0.010, column_height)),
            origin=Origin(xyz=(x, 0.023, column_center_z)),
            material=dark_metal,
            name=f"column_{index}_rear_wall",
        )
        lower.visual(
            Box((0.010, 0.070, column_height)),
            origin=Origin(xyz=(x - 0.033, 0.0, column_center_z)),
            material=dark_metal,
            name=f"column_{index}_outer_wall",
        )
        lower.visual(
            Box((0.010, 0.070, column_height)),
            origin=Origin(xyz=(x + 0.033, 0.0, column_center_z)),
            material=dark_metal,
            name=f"column_{index}_inner_wall",
        )
        lower.visual(
            Box((0.086, 0.010, 0.018)),
            origin=Origin(xyz=(x, -0.033, OUTER_TOP_Z - 0.009)),
            material=metal,
            name=top_lip_name,
        )
        lower.visual(
            Box((0.086, 0.010, 0.018)),
            origin=Origin(xyz=(x, 0.033, OUTER_TOP_Z - 0.009)),
            material=metal,
            name=f"column_{index}_rear_lip",
        )
        lower.visual(
            Box((0.010, 0.056, 0.018)),
            origin=Origin(xyz=(x - 0.043, 0.0, OUTER_TOP_Z - 0.009)),
            material=metal,
            name=f"column_{index}_outer_lip",
        )
        lower.visual(
            Box((0.010, 0.056, 0.018)),
            origin=Origin(xyz=(x + 0.043, 0.0, OUTER_TOP_Z - 0.009)),
            material=metal,
            name=f"column_{index}_inner_lip",
        )

    for index, x in enumerate((-0.32, 0.32)):
        stage = model.part(f"inner_stage_{index}")
        stage.visual(
            Box((0.056, 0.036, 0.640)),
            origin=Origin(xyz=(0.0, 0.0, -0.130)),
            material=inner_metal,
            name="inner_post",
        )
        stage.visual(
            Box((0.070, 0.050, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, INNER_POST_TOP_LOCAL_Z + 0.010)),
            material=inner_metal,
            name="top_cap",
        )
        model.articulation(
            f"lower_to_stage_{index}",
            ArticulationType.PRISMATIC,
            parent=lower,
            child=stage,
            origin=Origin(xyz=(x, 0.0, OUTER_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=180.0,
                velocity=0.10,
                lower=0.0,
                upper=LIFT_TRAVEL,
            ),
            mimic=(Mimic("lower_to_stage_0") if index == 1 else None),
        )

    top = model.part("top_assembly")
    top.visual(
        Box((1.050, 0.460, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=wood,
        name="desktop_slab",
    )
    # Slightly darker edge banding gives the narrow top a finished furniture
    # look instead of a plain construction block.
    top.visual(
        Box((1.050, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, -0.230, 0.035)),
        material=wood_edge,
        name="front_edge_band",
    )
    top.visual(
        Box((1.050, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, 0.230, 0.035)),
        material=wood_edge,
        name="rear_edge_band",
    )
    top.visual(
        Box((0.014, 0.460, 0.040)),
        origin=Origin(xyz=(-0.525, 0.0, 0.035)),
        material=wood_edge,
        name="side_edge_band_0",
    )
    top.visual(
        Box((0.014, 0.460, 0.040)),
        origin=Origin(xyz=(0.525, 0.0, 0.035)),
        material=wood_edge,
        name="side_edge_band_1",
    )

    # The black top frame is a visible, separate rail structure below the top,
    # not fused to the lifting columns.  A center rail and bolted plates tie the
    # two telescoping stages to the tabletop.
    top.visual(
        Box((0.920, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.175, 0.0)),
        material=metal,
        name="front_frame_rail",
    )
    top.visual(
        Box((0.920, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.175, 0.0)),
        material=metal,
        name="rear_frame_rail",
    )
    top.visual(
        Box((0.035, 0.350, 0.035)),
        origin=Origin(xyz=(-0.455, 0.0, 0.0)),
        material=metal,
        name="side_frame_rail_0",
    )
    top.visual(
        Box((0.035, 0.350, 0.035)),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=metal,
        name="side_frame_rail_1",
    )
    top.visual(
        Box((0.760, 0.042, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="center_frame_rail",
    )
    for index, x in enumerate((-0.32, 0.32)):
        stage_mount_plate_name = "stage_mount_plate_0" if index == 0 else "stage_mount_plate_1"
        top.visual(
            Box((0.140, 0.110, 0.012)),
            origin=Origin(xyz=(x, 0.0, -0.0235)),
            material=metal,
            name=stage_mount_plate_name,
        )
        top.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x - 0.040, -0.030, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=inner_metal,
            name=f"mount_bolt_{index}_0",
        )
        top.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x + 0.040, 0.030, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=inner_metal,
            name=f"mount_bolt_{index}_1",
        )

    model.articulation(
        "stage_0_to_top",
        ArticulationType.FIXED,
        parent="inner_stage_0",
        child=top,
        origin=Origin(xyz=(0.32, 0.0, TOP_FRAME_Z)),
    )

    pod = model.part("control_pod")
    pod.visual(
        Box((0.160, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="pod_body",
    )
    pod.visual(
        Box((0.112, 0.040, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=metal,
        name="underdesk_mount_tab",
    )
    pod.visual(
        Box((0.006, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, -0.034, -0.018)),
        material=dark_metal,
        name="center_divider",
    )
    pod.visual(
        Cylinder(radius=0.003, length=0.060),
        origin=Origin(xyz=(0.033, -0.034, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_metal,
        name="up_pivot_pin",
    )
    pod.visual(
        Cylinder(radius=0.003, length=0.060),
        origin=Origin(xyz=(-0.033, -0.034, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inner_metal,
        name="down_pivot_pin",
    )

    model.articulation(
        "top_to_control_pod",
        ArticulationType.FIXED,
        parent=top,
        child=pod,
        origin=Origin(xyz=CONTROL_POD_ORIGIN),
    )

    def add_paddle(part_name: str, joint_name: str, x: float, arrow_up: bool) -> None:
        paddle = model.part(part_name)
        paddle.visual(
            Box((0.054, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=rubber,
            name="hinge_block",
        )
        paddle.visual(
            Box((0.050, 0.006, 0.030)),
            origin=Origin(xyz=(0.0, -0.005, -0.018)),
            material=rubber,
            name="paddle_face",
        )
        paddle.visual(
            Box((0.004, 0.001, 0.014)),
            origin=Origin(xyz=(0.0, -0.0083, -0.019)),
            material=label,
            name="arrow_stem",
        )
        chevron_z = -0.010 if arrow_up else -0.028
        chevron_angle = 0.65 if arrow_up else -0.65
        paddle.visual(
            Box((0.018, 0.001, 0.003)),
            origin=Origin(xyz=(-0.006, -0.0083, chevron_z), rpy=(0.0, chevron_angle, 0.0)),
            material=label,
            name="arrow_wing_0",
        )
        paddle.visual(
            Box((0.018, 0.001, 0.003)),
            origin=Origin(xyz=(0.006, -0.0083, chevron_z), rpy=(0.0, -chevron_angle, 0.0)),
            material=label,
            name="arrow_wing_1",
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=pod,
            child=paddle,
            origin=Origin(xyz=(x, -0.034, -0.004)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=0.0, upper=0.25),
        )

    add_paddle("up_paddle", "pod_to_up_paddle", 0.033, True)
    add_paddle("down_paddle", "pod_to_down_paddle", -0.033, False)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_frame")
    stage_0 = object_model.get_part("inner_stage_0")
    stage_1 = object_model.get_part("inner_stage_1")
    top = object_model.get_part("top_assembly")
    pod = object_model.get_part("control_pod")
    up = object_model.get_part("up_paddle")
    down = object_model.get_part("down_paddle")
    lift = object_model.get_articulation("lower_to_stage_0")
    up_joint = object_model.get_articulation("pod_to_up_paddle")

    ctx.allow_overlap(
        pod,
        up,
        elem_a="up_pivot_pin",
        elem_b="hinge_block",
        reason="The up paddle hinge block is intentionally captured around the short local pivot pin.",
    )
    ctx.allow_overlap(
        pod,
        down,
        elem_a="down_pivot_pin",
        elem_b="hinge_block",
        reason="The down paddle hinge block is intentionally captured around the short local pivot pin.",
    )
    ctx.expect_overlap(
        pod,
        up,
        axes="xz",
        min_overlap=0.003,
        elem_a="up_pivot_pin",
        elem_b="hinge_block",
        name="up paddle hinge captures its local pivot",
    )
    ctx.expect_overlap(
        pod,
        down,
        axes="xz",
        min_overlap=0.003,
        elem_a="down_pivot_pin",
        elem_b="hinge_block",
        name="down paddle hinge captures its local pivot",
    )
    ctx.expect_gap(
        pod,
        up,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0005,
        positive_elem="pod_body",
        negative_elem="hinge_block",
        name="up paddle hinge block seats on the pod face",
    )
    ctx.expect_gap(
        pod,
        down,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0005,
        positive_elem="pod_body",
        negative_elem="hinge_block",
        name="down paddle hinge block seats on the pod face",
    )

    ctx.expect_contact(
        stage_0,
        top,
        elem_a="top_cap",
        elem_b="stage_mount_plate_0",
        contact_tol=0.0005,
        name="primary stage cap touches its top-frame plate",
    )
    ctx.expect_contact(
        stage_1,
        top,
        elem_a="top_cap",
        elem_b="stage_mount_plate_1",
        contact_tol=0.0005,
        name="mimic stage cap touches its top-frame plate",
    )
    ctx.expect_gap(
        top,
        lower,
        axis="z",
        min_gap=0.14,
        positive_elem="center_frame_rail",
        negative_elem="column_0_top_lip",
        name="top frame rail remains visibly above outer columns",
    )
    ctx.expect_gap(
        up,
        down,
        axis="x",
        min_gap=0.008,
        max_gap=0.020,
        positive_elem="paddle_face",
        negative_elem="paddle_face",
        name="up and down paddles are separate faces",
    )

    rest_top = ctx.part_world_position(top)
    rest_stage_0 = ctx.part_world_position(stage_0)
    rest_stage_1 = ctx.part_world_position(stage_1)
    with ctx.pose({lift: LIFT_TRAVEL}):
        extended_top = ctx.part_world_position(top)
        extended_stage_0 = ctx.part_world_position(stage_0)
        extended_stage_1 = ctx.part_world_position(stage_1)
        ctx.expect_contact(
            stage_1,
            top,
            elem_a="top_cap",
            elem_b="stage_mount_plate_1",
            contact_tol=0.0005,
            name="mimic stage remains under top plate when extended",
        )
        ctx.expect_overlap(
            stage_0,
            lower,
            axes="z",
            min_overlap=0.08,
            elem_a="inner_post",
            elem_b="column_0_front_wall",
            name="primary telescoping post remains inserted",
        )
        ctx.expect_overlap(
            stage_1,
            lower,
            axes="z",
            min_overlap=0.08,
            elem_a="inner_post",
            elem_b="column_1_front_wall",
            name="mimic telescoping post remains inserted",
        )

    ctx.check(
        "synchronized lift travel",
        rest_top is not None
        and extended_top is not None
        and rest_stage_0 is not None
        and extended_stage_0 is not None
        and rest_stage_1 is not None
        and extended_stage_1 is not None
        and extended_top[2] > rest_top[2] + 0.30
        and abs((extended_stage_0[2] - rest_stage_0[2]) - LIFT_TRAVEL) < 0.002
        and abs((extended_stage_1[2] - rest_stage_1[2]) - LIFT_TRAVEL) < 0.002,
        details=(
            f"top={rest_top}->{extended_top}, "
            f"stage0={rest_stage_0}->{extended_stage_0}, "
            f"stage1={rest_stage_1}->{extended_stage_1}"
        ),
    )

    rest_aabb = ctx.part_element_world_aabb(up, elem="paddle_face")
    with ctx.pose({up_joint: 0.25}):
        pressed_aabb = ctx.part_element_world_aabb(up, elem="paddle_face")
        ctx.expect_gap(
            up,
            down,
            axis="x",
            min_gap=0.008,
            positive_elem="paddle_face",
            negative_elem="paddle_face",
            name="paddle split remains visible while one paddle is pressed",
        )

    rest_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
    pressed_y = None if pressed_aabb is None else (pressed_aabb[0][1] + pressed_aabb[1][1]) * 0.5
    ctx.check(
        "up paddle rotates inward on local pivot",
        rest_y is not None and pressed_y is not None and pressed_y > rest_y + 0.002,
        details=f"rest_y={rest_y}, pressed_y={pressed_y}",
    )

    return ctx.report()


object_model = build_object_model()
