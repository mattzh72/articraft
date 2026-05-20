from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Cylinder,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="children_double_sided_easel")

    maple = model.material("maple", rgba=(0.79, 0.66, 0.48, 1.0))
    whiteboard = model.material("whiteboard", rgba=(0.96, 0.96, 0.94, 1.0))
    chalkboard = model.material("chalkboard", rgba=(0.14, 0.28, 0.19, 1.0))
    tray_red = model.material("tray_red", rgba=(0.83, 0.24, 0.20, 1.0))
    brace_blue = model.material("brace_blue", rgba=(0.29, 0.53, 0.76, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.59, 0.63, 1.0))
    paper = model.material("paper", rgba=(0.97, 0.95, 0.90, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.20, 1.0))

    frame_half_width = 0.32
    rail_width = 0.05
    frame_depth = 0.045
    rail_height = 1.12
    top_beam_height = 0.06
    bottom_beam_height = 0.08
    panel_width = 0.58
    panel_height = 0.72
    tray_width = 0.60
    rear_angle = math.radians(22.0)
    rear_offset_x = -0.04
    rear_cos = math.cos(rear_angle)
    rear_sin = math.sin(rear_angle)

    def rear_pose(local_x: float, local_z: float) -> tuple[float, float]:
        return (
            (local_x * rear_cos) + (local_z * rear_sin),
            (-local_x * rear_sin) + (local_z * rear_cos),
        )

    def add_front(
        part,
        geometry,
        xyz: tuple[float, float, float],
        material,
        name: str,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(geometry, origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_rear(
        part,
        geometry,
        local_xyz: tuple[float, float, float],
        material,
        name: str,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        local_x, y, local_z = local_xyz
        x, z = rear_pose(local_x, local_z)
        part.visual(
            geometry,
            origin=Origin(xyz=(x, y, z), rpy=(rpy[0], rpy[1] + rear_angle, rpy[2])),
            material=material,
            name=name,
        )

    front_frame = model.part("front_frame")
    for index, side_y in enumerate((-frame_half_width + (rail_width / 2.0), frame_half_width - (rail_width / 2.0))):
        add_front(
            front_frame,
            Box((frame_depth, rail_width, rail_height)),
            xyz=(0.0, side_y, -rail_height / 2.0),
            material=maple,
            name=f"front_rail_{index}",
        )
        add_front(
            front_frame,
            Box((0.18, rail_width + 0.01, 0.04)),
            xyz=(0.06, side_y, -1.10),
            material=maple,
            name=f"front_foot_{index}",
        )

    add_front(
        front_frame,
        Box((frame_depth, 0.66, top_beam_height)),
        xyz=(0.0, 0.0, -0.03),
        material=maple,
        name="top_beam",
    )
    add_front(
        front_frame,
        Box((frame_depth, 0.60, bottom_beam_height)),
        xyz=(0.0, 0.0, -0.95),
        material=maple,
        name="bottom_beam",
    )
    add_front(
        front_frame,
        Box((0.012, panel_width, panel_height)),
        xyz=(0.0, 0.0, -0.49),
        material=whiteboard,
        name="front_panel",
    )
    add_front(
        front_frame,
        Box((0.12, tray_width, 0.028)),
        xyz=(0.045, 0.0, -0.80),
        material=tray_red,
        name="front_tray",
    )
    add_front(
        front_frame,
        Box((0.018, tray_width, 0.05)),
        xyz=(0.096, 0.0, -0.775),
        material=tray_red,
        name="front_tray_lip",
    )

    latch_side_y = frame_half_width + 0.015
    roll_support_y = 0.247
    front_latch_z = -0.91
    front_latch_x = 0.012
    latch_tab_offset = 0.055
    for index, side_y in enumerate((-latch_side_y, latch_side_y)):
        add_front(
            front_frame,
            Box((0.036, 0.03, 0.012)),
            xyz=(0.0, side_y, front_latch_z + latch_tab_offset),
            material=maple,
            name=f"latch_pad_upper_{index}",
        )
        add_front(
            front_frame,
            Box((0.036, 0.03, 0.012)),
            xyz=(0.0, side_y, front_latch_z - latch_tab_offset),
            material=maple,
            name=f"latch_pad_lower_{index}",
        )
        add_front(
            front_frame,
            Cylinder(radius=0.009, length=0.034),
            xyz=(front_latch_x, side_y, front_latch_z),
            material=steel,
            name=f"latch_pin_{index}",
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
    for index, side_y in enumerate((-roll_support_y, roll_support_y)):
        add_front(
            front_frame,
            Box((0.06, 0.022, 0.10)),
            xyz=(0.03, side_y, 0.03),
            material=tray_red,
            name=f"roll_support_{index}",
        )

    rear_frame = model.part("rear_frame")
    for index, side_y in enumerate((-frame_half_width + (rail_width / 2.0), frame_half_width - (rail_width / 2.0))):
        add_rear(
            rear_frame,
            Box((frame_depth, rail_width, rail_height)),
            local_xyz=(0.0, side_y, -rail_height / 2.0),
            material=maple,
            name=f"rear_rail_{index}",
        )
        add_rear(
            rear_frame,
            Box((0.18, rail_width + 0.01, 0.04)),
            local_xyz=(-0.02, side_y, -1.10),
            material=maple,
            name=f"rear_foot_{index}",
        )

    add_rear(
        rear_frame,
        Box((frame_depth, 0.66, top_beam_height)),
        local_xyz=(0.0, 0.0, -0.03),
        material=maple,
        name="rear_top_beam",
    )
    add_rear(
        rear_frame,
        Box((frame_depth, 0.60, bottom_beam_height)),
        local_xyz=(0.0, 0.0, -0.95),
        material=maple,
        name="rear_bottom_beam",
    )
    add_rear(
        rear_frame,
        Box((0.012, panel_width, panel_height)),
        local_xyz=(0.0, 0.0, -0.49),
        material=chalkboard,
        name="rear_panel",
    )
    add_rear(
        rear_frame,
        Box((0.10, tray_width, 0.028)),
        local_xyz=(-0.045, 0.0, -0.80),
        material=tray_red,
        name="rear_tray",
    )
    add_rear(
        rear_frame,
        Box((0.018, tray_width, 0.05)),
        local_xyz=(-0.092, 0.0, -0.775),
        material=tray_red,
        name="rear_tray_lip",
    )

    rear_hinge_local_x = 0.0
    rear_hinge_local_z = -0.74
    mount_tab_offset = 0.026
    mount_tab_x = -0.045
    rear_hinge_x, rear_hinge_z = rear_pose(rear_hinge_local_x, rear_hinge_local_z)
    for index, side_y in enumerate((-latch_side_y, latch_side_y)):
        add_rear(
            rear_frame,
            Box((0.05, 0.032, 0.018)),
            local_xyz=(mount_tab_x, side_y, rear_hinge_local_z + mount_tab_offset),
            material=maple,
            name=f"brace_mount_upper_{index}",
        )
        add_rear(
            rear_frame,
            Box((0.05, 0.032, 0.018)),
            local_xyz=(mount_tab_x, side_y, rear_hinge_local_z - mount_tab_offset),
            material=maple,
            name=f"brace_mount_lower_{index}",
        )
    model.articulation(
        "rear_mount",
        ArticulationType.FIXED,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(rear_offset_x, 0.0, 0.0)),
    )

    latch_points = (
        (front_latch_x, -latch_side_y, front_latch_z),
        (front_latch_x, latch_side_y, front_latch_z),
    )
    hinge_points = (
        (rear_offset_x + rear_hinge_x, -latch_side_y, rear_hinge_z),
        (rear_offset_x + rear_hinge_x, latch_side_y, rear_hinge_z),
    )

    brace_reach = math.hypot(latch_points[0][0] - hinge_points[0][0], latch_points[0][2] - hinge_points[0][2]) - 0.025
    brace_pitch = math.atan2(
        -(latch_points[0][2] - hinge_points[0][2]),
        latch_points[0][0] - hinge_points[0][0],
    )
    brace_bar_length = brace_reach - 0.025
    brace_tip_length = 0.032
    brace_tip_backset = 0.018
    brace_bar_center = (
        0.5 * brace_bar_length * math.cos(brace_pitch),
        0.0,
        -0.5 * brace_bar_length * math.sin(brace_pitch),
    )
    brace_tip_center = (
        (brace_reach - brace_tip_backset - (brace_tip_length / 2.0)) * math.cos(brace_pitch),
        0.0,
        -(brace_reach - brace_tip_backset - (brace_tip_length / 2.0)) * math.sin(brace_pitch),
    )

    for index in range(2):
        brace = model.part(f"brace_{index}")
        brace.visual(
            Cylinder(radius=0.014, length=0.032),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hinge_barrel",
        )
        brace.visual(
            Box((brace_bar_length, 0.018, 0.032)),
            origin=Origin(xyz=brace_bar_center, rpy=(0.0, brace_pitch, 0.0)),
            material=brace_blue,
            name="arm",
        )
        brace.visual(
            Box((brace_tip_length, 0.028, 0.040)),
            origin=Origin(xyz=brace_tip_center, rpy=(0.0, brace_pitch, 0.0)),
            material=brace_blue,
            name="tip",
        )
        brace.visual(
            Box((brace_tip_backset + 0.006, 0.014, 0.014)),
            origin=Origin(
                xyz=(
                    (brace_reach - (brace_tip_backset / 2.0) - 0.003) * math.cos(brace_pitch),
                    0.0,
                    -(brace_reach - (brace_tip_backset / 2.0) - 0.003) * math.sin(brace_pitch),
                ),
                rpy=(0.0, brace_pitch, 0.0),
            ),
            material=brace_blue,
            name="tip_bridge",
        )
        brace.visual(
            Cylinder(radius=0.008, length=0.032),
            origin=Origin(
                xyz=(
                    brace_reach * math.cos(brace_pitch),
                    0.0,
                    -brace_reach * math.sin(brace_pitch),
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=charcoal,
            name="tip_pin",
        )

    model.articulation(
        "brace_0_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child="brace_0",
        origin=Origin(xyz=(rear_hinge_x, -latch_side_y, rear_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.15, upper=1.35),
    )
    model.articulation(
        "brace_1_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child="brace_1",
        origin=Origin(xyz=(rear_hinge_x, latch_side_y, rear_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.15, upper=1.35),
        mimic=Mimic(joint="brace_0_hinge"),
    )

    paper_roll = model.part("paper_roll")
    paper_roll.visual(
        Cylinder(radius=0.012, length=0.472),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    paper_roll.visual(
        Cylinder(radius=0.038, length=0.44),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paper,
        name="roll_body",
    )
    for index, side_y in enumerate((-0.228, 0.228)):
        paper_roll.visual(
            Cylinder(radius=0.048, length=0.016),
            origin=Origin(xyz=(0.0, side_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"end_cap_{index}",
        )
    paper_roll.visual(
        Box((0.028, 0.018, 0.014)),
        origin=Origin(xyz=(0.050, 0.228, 0.0)),
        material=charcoal,
        name="handle",
    )

    model.articulation(
        "paper_roll_axis",
        ArticulationType.CONTINUOUS,
        parent=front_frame,
        child=paper_roll,
        origin=Origin(xyz=(0.03, 0.0, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    brace_0 = object_model.get_part("brace_0")
    paper_roll = object_model.get_part("paper_roll")
    brace_hinge = object_model.get_articulation("brace_0_hinge")
    paper_roll_axis = object_model.get_articulation("paper_roll_axis")

    def elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="yz",
        elem_a="front_panel",
        elem_b="rear_panel",
        min_overlap=0.50,
        name="front and rear panel faces align across the easel",
    )
    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem="front_tray_lip",
        negative_elem="rear_tray_lip",
        min_gap=0.24,
        name="front and rear trays sit on opposite sides of the easel",
    )
    ctx.expect_gap(
        paper_roll,
        front_frame,
        axis="z",
        positive_elem="roll_body",
        negative_elem="top_beam",
        min_gap=0.012,
        max_gap=0.090,
        name="paper roll rides above the top beam",
    )
    ctx.expect_gap(
        front_frame,
        brace_0,
        axis="x",
        positive_elem="latch_pin_0",
        negative_elem="tip_pin",
        min_gap=0.001,
        max_gap=0.018,
        name="resting brace tip stays parked near the front latch",
    )

    rest_tip = elem_center(brace_0, "tip")
    with ctx.pose({brace_hinge: 1.05}):
        folded_tip = elem_center(brace_0, "tip")
    ctx.check(
        "brace folds rearward from its engaged position",
        rest_tip is not None
        and folded_tip is not None
        and folded_tip[0] < rest_tip[0] - 0.12,
        details=f"rest_tip={rest_tip}, folded_tip={folded_tip}",
    )

    rest_handle = elem_center(paper_roll, "handle")
    with ctx.pose({paper_roll_axis: 1.20}):
        turned_handle = elem_center(paper_roll, "handle")
    ctx.check(
        "paper roll bar spins about the horizontal support axis",
        rest_handle is not None
        and turned_handle is not None
        and abs(turned_handle[0] - rest_handle[0]) > 0.020
        and abs(turned_handle[2] - rest_handle[2]) > 0.015,
        details=f"rest_handle={rest_handle}, turned_handle={turned_handle}",
    )

    return ctx.report()


object_model = build_object_model()
