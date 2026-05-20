from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_toaster_oven")

    steel = model.material("steel", rgba=(0.76, 0.77, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.14, 0.18, 0.85))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.62, 0.64, 0.67, 1.0))

    body_width = 0.42
    body_depth = 0.32
    body_height = 0.25

    side_wall = 0.015
    top_wall = 0.020
    bottom_wall = 0.015
    back_wall = 0.015
    front_frame = 0.015

    opening_width = 0.28
    opening_height = 0.15
    opening_center_y = -0.035
    opening_bottom = -0.045
    opening_center_z = opening_bottom + opening_height / 2.0

    control_width = 0.09
    left_trim_width = 0.020

    front_x = body_depth / 2.0
    inner_front_x = front_x - front_frame
    inner_rear_x = -body_depth / 2.0 + back_wall

    cavity_floor_thickness = 0.010
    cavity_floor_z = opening_bottom - cavity_floor_thickness / 2.0

    body = model.part("body")
    body.visual(
        Box((body_depth, side_wall, body_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + side_wall / 2.0, 0.0)),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((body_depth, side_wall, body_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - side_wall / 2.0, 0.0)),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((body_depth, body_width, top_wall)),
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0 - top_wall / 2.0)),
        material=steel,
        name="top_shell",
    )
    body.visual(
        Box((body_depth, body_width, bottom_wall)),
        origin=Origin(xyz=(0.0, 0.0, -body_height / 2.0 + bottom_wall / 2.0)),
        material=steel,
        name="bottom_shell",
    )
    body.visual(
        Box((back_wall, body_width - 2.0 * side_wall, body_height - top_wall - bottom_wall)),
        origin=Origin(xyz=(-body_depth / 2.0 + back_wall / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_panel",
    )
    body.visual(
        Box((inner_front_x - inner_rear_x, opening_width, cavity_floor_thickness)),
        origin=Origin(
            xyz=((inner_front_x + inner_rear_x) / 2.0, opening_center_y, cavity_floor_z)
        ),
        material=dark_steel,
        name="cavity_floor",
    )

    body.visual(
        Box((front_frame, body_width - 2.0 * side_wall, top_wall)),
        origin=Origin(
            xyz=(front_x - front_frame / 2.0, 0.0, body_height / 2.0 - top_wall / 2.0)
        ),
        material=steel,
        name="front_header",
    )
    body.visual(
        Box((front_frame, body_width - 2.0 * side_wall, cavity_floor_thickness)),
        origin=Origin(xyz=(front_x - front_frame / 2.0, 0.0, cavity_floor_z)),
        material=steel,
        name="front_sill",
    )
    body.visual(
        Box((front_frame, left_trim_width, opening_height)),
        origin=Origin(
            xyz=(
                front_x - front_frame / 2.0,
                opening_center_y - opening_width / 2.0 - left_trim_width / 2.0,
                opening_center_z,
            )
        ),
        material=steel,
        name="left_trim",
    )
    body.visual(
        Box((front_frame, control_width, opening_height)),
        origin=Origin(
            xyz=(
                front_x - front_frame / 2.0,
                opening_center_y + opening_width / 2.0 + control_width / 2.0,
                opening_center_z,
            )
        ),
        material=steel,
        name="control_panel",
    )

    body.visual(
        Box((inner_front_x - inner_rear_x, 0.010, opening_height)),
        origin=Origin(
            xyz=(0.0, opening_center_y - opening_width / 2.0 - 0.005, opening_center_z)
        ),
        material=dark_steel,
        name="left_jamb",
    )
    body.visual(
        Box((inner_front_x - inner_rear_x, 0.010, opening_height)),
        origin=Origin(
            xyz=(0.0, opening_center_y + opening_width / 2.0 + 0.005, opening_center_z)
        ),
        material=dark_steel,
        name="right_jamb",
    )

    guide_length = 0.12
    guide_height = 0.010
    guide_width = 0.010
    guide_center_z = cavity_floor_z - cavity_floor_thickness / 2.0 - guide_height / 2.0
    guide_center_x = inner_front_x - guide_length / 2.0
    tray_center_y = opening_center_y
    tray_outer_width = 0.27
    body.visual(
        Box((guide_length, guide_width, guide_height)),
        origin=Origin(
            xyz=(guide_center_x, tray_center_y - tray_outer_width / 2.0, guide_center_z)
        ),
        material=dark_steel,
        name="guide_0",
    )
    body.visual(
        Box((guide_length, guide_width, guide_height)),
        origin=Origin(
            xyz=(guide_center_x, tray_center_y + tray_outer_width / 2.0, guide_center_z)
        ),
        material=dark_steel,
        name="guide_1",
    )

    door = model.part("door")
    door_thickness = 0.018
    door.visual(
        Box((door_thickness, opening_width, opening_height)),
        origin=Origin(xyz=(door_thickness / 2.0, 0.0, opening_height / 2.0)),
        material=steel,
        name="door_panel",
    )
    door.visual(
        Box((0.004, 0.19, 0.085)),
        origin=Origin(xyz=(door_thickness - 0.002, 0.0, 0.080)),
        material=glass,
        name="door_glass",
    )
    handle_z = 0.118
    door.visual(
        Box((0.020, 0.014, 0.012)),
        origin=Origin(xyz=(0.020, -0.060, handle_z)),
        material=black,
        name="handle_post_0",
    )
    door.visual(
        Box((0.020, 0.014, 0.012)),
        origin=Origin(xyz=(0.020, 0.060, handle_z)),
        material=black,
        name="handle_post_1",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(
            xyz=(0.032, 0.0, handle_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="handle_bar",
    )

    tray = model.part("tray")
    tray_depth = 0.16
    tray_lip_thickness = 0.010
    tray_lip_height = 0.030
    tray_wall_thickness = 0.004
    tray_wall_height = 0.028
    tray_back_thickness = 0.010
    tray_floor_thickness = 0.003
    tray.visual(
        Box((tray_depth, tray_outer_width, tray_floor_thickness)),
        origin=Origin(xyz=(-tray_depth / 2.0, 0.0, -0.011)),
        material=tray_finish,
        name="tray_floor",
    )
    tray.visual(
        Box((tray_lip_thickness, tray_outer_width + 0.028, tray_lip_height)),
        origin=Origin(xyz=(tray_lip_thickness / 2.0, 0.0, 0.0)),
        material=tray_finish,
        name="tray_lip",
    )
    tray.visual(
        Box((tray_depth - tray_back_thickness, tray_wall_thickness, tray_wall_height)),
        origin=Origin(
            xyz=(-(tray_depth - tray_back_thickness) / 2.0, -tray_outer_width / 2.0, 0.001)
        ),
        material=tray_finish,
        name="tray_side_0",
    )
    tray.visual(
        Box((tray_depth - tray_back_thickness, tray_wall_thickness, tray_wall_height)),
        origin=Origin(
            xyz=((-(tray_depth - tray_back_thickness) / 2.0), tray_outer_width / 2.0, 0.001)
        ),
        material=tray_finish,
        name="tray_side_1",
    )
    tray.visual(
        Box((tray_back_thickness, tray_outer_width - 2.0 * tray_wall_thickness, tray_wall_height)),
        origin=Origin(xyz=(-tray_depth + tray_back_thickness / 2.0, 0.0, 0.001)),
        material=tray_finish,
        name="tray_back",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.024,
            body_style="skirted",
            top_diameter=0.036,
            base_diameter=0.046,
            crown_radius=0.002,
            edge_radius=0.001,
            side_draft_deg=5.0,
            center=False,
        ),
        "toaster_knob",
    )

    knob_positions = (0.055, -0.010)
    for index, knob_z in enumerate(knob_positions):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="knob_cap",
        )
        model.articulation(
            f"knob_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(front_x, opening_center_y + opening_width / 2.0 + control_width / 2.0, knob_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(inner_front_x, opening_center_y, opening_bottom)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(inner_front_x + 0.010, tray_center_y, -0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.070,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    tray = object_model.get_part("tray")

    door_hinge = object_model.get_articulation("door_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        min_overlap=0.14,
        name="crumb tray remains inserted at rest",
    )
    ctx.expect_within(
        tray,
        body,
        axes="yz",
        margin=0.0,
        name="crumb tray stays within the body envelope",
    )

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.08,
            name="crumb tray keeps retained insertion when pulled out",
        )
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            margin=0.0,
            name="crumb tray stays aligned in the guide slot when extended",
        )
        extended_pos = ctx.part_world_position(tray)
    ctx.check(
        "crumb tray slides forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle_bar")
    ctx.check(
        "door opens downward",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][2] < closed_handle[0][2] - 0.07
        and open_handle[1][0] > closed_handle[1][0] + 0.04,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    for joint_name in ("knob_0_spin", "knob_1_spin"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
