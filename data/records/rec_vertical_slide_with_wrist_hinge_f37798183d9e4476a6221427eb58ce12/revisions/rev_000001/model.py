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
    model = ArticulatedObject(name="fork_backed_slide_wrist")

    frame_mat = model.material("powder_coated_frame", color=(0.08, 0.09, 0.10, 1.0))
    rail_mat = model.material("polished_steel", color=(0.72, 0.74, 0.76, 1.0))
    carriage_mat = model.material("blue_anodized_carriage", color=(0.05, 0.18, 0.42, 1.0))
    bearing_mat = model.material("black_linear_bearing", color=(0.015, 0.015, 0.018, 1.0))
    face_mat = model.material("orange_output_face", color=(0.95, 0.42, 0.08, 1.0))
    pin_mat = model.material("dark_hinge_bushing", color=(0.03, 0.03, 0.035, 1.0))

    frame = model.part("rear_frame")
    frame.visual(
        Box((0.55, 0.52, 0.06)),
        origin=Origin(xyz=(0.08, 0.0, 0.03)),
        material=frame_mat,
        name="base_plate",
    )
    for y, name in ((-0.20, "fork_tine_0"), (0.20, "fork_tine_1")):
        frame.visual(
            Box((0.07, 0.06, 0.72)),
            origin=Origin(xyz=(-0.13, y, 0.42)),
            material=frame_mat,
            name=name,
        )
    frame.visual(
        Box((0.10, 0.46, 0.06)),
        origin=Origin(xyz=(-0.08, 0.0, 0.81)),
        material=frame_mat,
        name="top_bridge",
    )
    frame.visual(
        Box((0.10, 0.46, 0.06)),
        origin=Origin(xyz=(-0.08, 0.0, 0.09)),
        material=frame_mat,
        name="bottom_bridge",
    )
    for y, suffix in ((-0.12, "0"), (0.12, "1")):
        frame.visual(
            Box((0.07, 0.055, 0.04)),
            origin=Origin(xyz=(-0.005, y, 0.10)),
            material=frame_mat,
            name=f"lower_rail_socket_{suffix}",
        )
        frame.visual(
            Box((0.07, 0.055, 0.04)),
            origin=Origin(xyz=(-0.005, y, 0.77)),
            material=frame_mat,
            name=f"upper_rail_socket_{suffix}",
        )
    frame.visual(
        Cylinder(radius=0.014, length=0.66),
        origin=Origin(xyz=(0.0, -0.12, 0.445)),
        material=rail_mat,
        name="guide_rod_0",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.66),
        origin=Origin(xyz=(0.0, 0.12, 0.445)),
        material=rail_mat,
        name="guide_rod_1",
    )

    carriage = model.part("slide_carriage")
    carriage.visual(
        Box((0.044, 0.074, 0.17)),
        origin=Origin(xyz=(0.036, -0.12, 0.0)),
        material=bearing_mat,
        name="bearing_0",
    )
    carriage.visual(
        Box((0.044, 0.074, 0.17)),
        origin=Origin(xyz=(0.036, 0.12, 0.0)),
        material=bearing_mat,
        name="bearing_1",
    )
    carriage.visual(
        Box((0.06, 0.29, 0.08)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=carriage_mat,
        name="front_crosshead",
    )
    carriage.visual(
        Box((0.07, 0.16, 0.24)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=carriage_mat,
        name="wrist_web",
    )
    carriage.visual(
        Box((0.070, 0.045, 0.090)),
        origin=Origin(xyz=(0.175, -0.095, 0.0)),
        material=carriage_mat,
        name="hinge_ear_0",
    )
    carriage.visual(
        Cylinder(radius=0.017, length=0.045),
        origin=Origin(xyz=(0.175, -0.095, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="ear_bushing_0",
    )
    carriage.visual(
        Box((0.070, 0.045, 0.090)),
        origin=Origin(xyz=(0.175, 0.095, 0.0)),
        material=carriage_mat,
        name="hinge_ear_1",
    )
    carriage.visual(
        Cylinder(radius=0.017, length=0.045),
        origin=Origin(xyz=(0.175, 0.095, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="ear_bushing_1",
    )

    face = model.part("output_face")
    face.visual(
        Cylinder(radius=0.024, length=0.145),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="center_hinge_barrel",
    )
    face.visual(
        Box((0.070, 0.12, 0.040)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=face_mat,
        name="short_neck",
    )
    face.visual(
        Box((0.035, 0.20, 0.14)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=face_mat,
        name="mounting_face",
    )
    face.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.114, -0.060, 0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=pin_mat,
        name="bolt_boss_0",
    )
    face.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.114, 0.060, 0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=pin_mat,
        name="bolt_boss_1",
    )
    face.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.114, -0.060, -0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=pin_mat,
        name="bolt_boss_2",
    )
    face.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.114, 0.060, -0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=pin_mat,
        name="bolt_boss_3",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.35, lower=0.0, upper=0.32),
    )
    model.articulation(
        "carriage_to_face",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=face,
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.8, lower=-0.70, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("rear_frame")
    carriage = object_model.get_part("slide_carriage")
    face = object_model.get_part("output_face")
    slide = object_model.get_articulation("frame_to_carriage")
    wrist = object_model.get_articulation("carriage_to_face")

    with ctx.pose({slide: 0.0, wrist: 0.0}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="x",
            positive_elem="bearing_0",
            negative_elem="guide_rod_0",
            max_gap=0.003,
            max_penetration=0.0005,
            name="bearing_0 runs on guide_rod_0",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="x",
            positive_elem="bearing_1",
            negative_elem="guide_rod_1",
            max_gap=0.003,
            max_penetration=0.0005,
            name="bearing_1 runs on guide_rod_1",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="bearing_0",
            elem_b="guide_rod_0",
            min_overlap=0.12,
            name="lower slide is retained on first rod",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="bearing_1",
            elem_b="guide_rod_1",
            min_overlap=0.12,
            name="lower slide is retained on second rod",
        )
        ctx.expect_contact(
            face,
            carriage,
            elem_a="center_hinge_barrel",
            elem_b="ear_bushing_0",
            contact_tol=0.001,
            name="wrist barrel seats in first fork ear",
        )
        ctx.expect_contact(
            face,
            carriage,
            elem_a="center_hinge_barrel",
            elem_b="ear_bushing_1",
            contact_tol=0.001,
            name="wrist barrel seats in second fork ear",
        )
        rest_carriage_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.32, wrist: 0.0}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="bearing_0",
            elem_b="guide_rod_0",
            min_overlap=0.12,
            name="raised slide remains on first rod",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="bearing_1",
            elem_b="guide_rod_1",
            min_overlap=0.12,
            name="raised slide remains on second rod",
        )
        raised_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage travels vertically upward",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.30,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    with ctx.pose({wrist: 0.0}):
        rest_face_aabb = ctx.part_element_world_aabb(face, elem="mounting_face")
    with ctx.pose({wrist: 0.65}):
        raised_face_aabb = ctx.part_element_world_aabb(face, elem="mounting_face")

    rest_face_center = (
        None
        if rest_face_aabb is None
        else tuple((rest_face_aabb[0][i] + rest_face_aabb[1][i]) * 0.5 for i in range(3))
    )
    raised_face_center = (
        None
        if raised_face_aabb is None
        else tuple((raised_face_aabb[0][i] + raised_face_aabb[1][i]) * 0.5 for i in range(3))
    )
    ctx.check(
        "positive wrist pitch lifts output face",
        rest_face_center is not None
        and raised_face_center is not None
        and raised_face_center[2] > rest_face_center[2] + 0.04,
        details=f"rest={rest_face_center}, raised={raised_face_center}",
    )

    return ctx.report()


object_model = build_object_model()
