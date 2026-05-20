from __future__ import annotations

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
    model = ArticulatedObject(name="bridge_supported_cantilever_arm")

    frame_mat = model.material("powder_coated_frame", color=(0.10, 0.12, 0.14, 1.0))
    brace_mat = model.material("dark_bridge_braces", color=(0.16, 0.18, 0.20, 1.0))
    link_mat = model.material("safety_orange_links", color=(0.95, 0.38, 0.08, 1.0))
    wrist_mat = model.material("brushed_wrist_plate", color=(0.72, 0.74, 0.72, 1.0))
    boss_mat = model.material("black_bushed_pivots", color=(0.02, 0.02, 0.025, 1.0))

    shoulder_height = 0.55
    upper_len = 0.70
    fore_len = 0.42

    frame = model.part("frame")
    frame.visual(
        Box((0.42, 0.30, 0.04)),
        origin=Origin(xyz=(-0.13, 0.0, 0.02)),
        material=frame_mat,
        name="floor_plate",
    )
    frame.visual(
        Box((0.07, 0.24, 0.61)),
        origin=Origin(xyz=(-0.25, 0.0, 0.325)),
        material=frame_mat,
        name="rear_upright",
    )
    frame.visual(
        Box((0.28, 0.13, 0.05)),
        origin=Origin(xyz=(-0.12, 0.0, 0.625)),
        material=brace_mat,
        name="upper_bridge",
    )
    frame.visual(
        Box((0.43, 0.050, 0.036)),
        origin=Origin(xyz=(-0.135, 0.0, 0.315), rpy=(0.0, -1.10, 0.0)),
        material=brace_mat,
        name="diagonal_bridge",
    )
    # Two shoulder cheeks form an open yoke; the moving upper-link hub sits in the gap.
    frame.visual(
        Box((0.10, 0.028, 0.15)),
        origin=Origin(xyz=(-0.015, -0.066, shoulder_height)),
        material=frame_mat,
        name="shoulder_cheek_near",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, -0.066, shoulder_height), rpy=(pi / 2.0, 0.0, 0.0)),
        material=boss_mat,
        name="shoulder_boss_near",
    )
    frame.visual(
        Box((0.10, 0.028, 0.15)),
        origin=Origin(xyz=(-0.015, 0.066, shoulder_height)),
        material=frame_mat,
        name="shoulder_cheek_far",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, 0.066, shoulder_height), rpy=(pi / 2.0, 0.0, 0.0)),
        material=boss_mat,
        name="shoulder_boss_far",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.043, length=0.104),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=boss_mat,
        name="shoulder_barrel",
    )
    upper_link.visual(
        Box((0.120, 0.070, 0.024)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=link_mat,
        name="shoulder_web",
    )
    upper_link.visual(
        Box((0.050, 0.150, 0.030)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=link_mat,
        name="rail_crosshead",
    )
    for side, y in (("near", -0.064), ("far", 0.064)):
        upper_link.visual(
            Box((0.595, 0.026, 0.044)),
            origin=Origin(xyz=(0.375, y, 0.0)),
            material=link_mat,
            name=f"upper_rail_{side}",
        )
        upper_link.visual(
            Cylinder(radius=0.047, length=0.026),
            origin=Origin(xyz=(upper_len, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=boss_mat,
            name=f"elbow_boss_{side}",
        )
    upper_link.visual(
        Box((0.050, 0.150, 0.030)),
        origin=Origin(xyz=(upper_len - 0.075, 0.0, 0.0)),
        material=link_mat,
        name="elbow_end_bridge",
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.037, length=0.102),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=boss_mat,
        name="elbow_barrel",
    )
    forelink.visual(
        Box((0.340, 0.048, 0.040)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=link_mat,
        name="forearm_web",
    )
    forelink.visual(
        Box((0.056, 0.108, 0.034)),
        origin=Origin(xyz=(fore_len - 0.060, 0.0, 0.0)),
        material=link_mat,
        name="wrist_fork_bridge",
    )
    for side, y in (("near", -0.046), ("far", 0.046)):
        forelink.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(fore_len, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=boss_mat,
            name=f"wrist_boss_{side}",
        )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Cylinder(radius=0.027, length=0.074),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=boss_mat,
        name="wrist_barrel",
    )
    wrist_plate.visual(
        Box((0.030, 0.036, 0.018)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=wrist_mat,
        name="plate_neck",
    )
    wrist_plate.visual(
        Box((0.095, 0.110, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=wrist_mat,
        name="tool_plate",
    )
    for y in (-0.034, 0.034):
        wrist_plate.visual(
            Cylinder(radius=0.007, length=0.020),
            origin=Origin(xyz=(0.112, y, 0.0)),
            material=boss_mat,
            name=f"mount_button_{'near' if y < 0 else 'far'}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, shoulder_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.75, upper=1.05),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist_plate,
        origin=Origin(xyz=(fore_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.2, lower=-1.8, upper=1.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    wrist_plate = object_model.get_part("wrist_plate")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.check("three revolute joints", len(object_model.articulations) == 3)
    for joint in (shoulder, elbow, wrist):
        ctx.check(f"{joint.name} is revolute", joint.articulation_type == ArticulationType.REVOLUTE)

    ctx.expect_origin_gap(forelink, upper_link, axis="x", min_gap=0.68, max_gap=0.72, name="long upper link projects forward")
    ctx.expect_origin_gap(wrist_plate, forelink, axis="x", min_gap=0.40, max_gap=0.44, name="shorter forelink projects forward")
    ctx.expect_overlap(frame, upper_link, axes="xz", elem_a="shoulder_cheek_near", elem_b="shoulder_barrel", min_overlap=0.025, name="shoulder barrel aligns with support yoke")

    rest_elbow = ctx.part_world_position(forelink)
    rest_wrist = ctx.part_world_position(wrist_plate)
    with ctx.pose({shoulder: 0.70}):
        raised_elbow = ctx.part_world_position(forelink)
    with ctx.pose({elbow: 0.85}):
        bent_wrist = ctx.part_world_position(wrist_plate)
    with ctx.pose({wrist: 0.90}):
        wrist_box = ctx.part_element_world_aabb(wrist_plate, elem="tool_plate")

    ctx.check(
        "shoulder raises the cantilever",
        rest_elbow is not None and raised_elbow is not None and raised_elbow[2] > rest_elbow[2] + 0.20,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )
    ctx.check(
        "elbow bends the forelink",
        rest_wrist is not None and bent_wrist is not None and bent_wrist[2] > rest_wrist[2] + 0.20,
        details=f"rest={rest_wrist}, bent={bent_wrist}",
    )
    ctx.check(
        "wrist plate rotates visibly",
        wrist_box is not None and wrist_box[1][2] - wrist_box[0][2] > 0.070,
        details=f"tool_plate_aabb={wrist_box}",
    )

    return ctx.report()


object_model = build_object_model()
