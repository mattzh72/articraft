from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _make_stand_frame() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.305, 0.248, 0.055)
        .translate((0.0, 0.0, 0.0275))
        .edges("|Z")
        .fillet(0.014)
    )

    control_pod = (
        cq.Workplane("XY")
        .box(0.094, 0.084, 0.020)
        .translate((0.078, 0.056, 0.065))
        .edges("|Z")
        .fillet(0.006)
    )

    left_support = (
        cq.Workplane("XY")
        .box(0.030, 0.080, 0.250)
        .translate((-0.135, -0.028, 0.155))
        .edges("|Z")
        .fillet(0.006)
    )
    right_support = (
        cq.Workplane("XY")
        .box(0.030, 0.080, 0.250)
        .translate((0.135, -0.028, 0.155))
        .edges("|Z")
        .fillet(0.006)
    )

    frame = (
        base.union(control_pod)
        .union(left_support)
        .union(right_support)
    )

    return frame


def _make_shell_base() -> cq.Workplane:
    lower_shell = (
        cq.Workplane("XY")
        .workplane(offset=-0.047)
        .circle(0.120)
        .extrude(0.040)
    )

    lower_shell = (
        lower_shell.faces(">Z")
        .workplane()
        .circle(0.098)
        .cutBlind(-0.018)
    )
    return lower_shell


def _make_shell_lid() -> cq.Workplane:
    lid_shell = (
        cq.Workplane("XY")
        .workplane(offset=0.002)
        .center(0.0, 0.103)
        .circle(0.120)
        .extrude(0.034)
    )

    lid_shell = (
        lid_shell.faces("<Z")
        .workplane()
        .center(0.0, 0.103)
        .circle(0.098)
        .cutBlind(0.015)
    )
    return lid_shell


def _make_control_pod() -> cq.Workplane:
    pod = (
        cq.Workplane("XY")
        .box(0.094, 0.084, 0.020)
        .edges("|Z")
        .fillet(0.006)
    )
    pocket_points = [(-0.016, 0.010), (0.016, 0.010)]
    pod = (
        pod.faces(">Z")
        .workplane()
        .pushPoints(pocket_points)
        .circle(0.0105)
        .cutBlind(-0.0025)
    )
    pod = (
        pod.faces(">Z")
        .workplane()
        .pushPoints(pocket_points)
        .circle(0.0055)
        .cutBlind(-0.013)
    )
    return pod


def _aabb_center_y(aabb) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][1] + aabb[1][1]) * 0.5


def _aabb_max_z(aabb) -> float | None:
    if aabb is None:
        return None
    return aabb[1][2]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_belgian_waffle_maker")

    stand_black = model.material("stand_black", rgba=(0.16, 0.16, 0.17, 1.0))
    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    shell_steel = model.material("shell_steel", rgba=(0.72, 0.73, 0.74, 1.0))
    cooking_iron = model.material("cooking_iron", rgba=(0.23, 0.23, 0.24, 1.0))
    button_amber = model.material("button_amber", rgba=(0.86, 0.63, 0.22, 1.0))
    button_dark = model.material("button_dark", rgba=(0.18, 0.18, 0.18, 1.0))

    stand_frame = model.part("stand_frame")
    stand_frame.visual(
        Box((0.305, 0.248, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=stand_black,
        name="base_plate",
    )
    stand_frame.visual(
        mesh_from_cadquery(_make_control_pod(), "control_pod"),
        origin=Origin(xyz=(0.078, 0.056, 0.065)),
        material=stand_black,
        name="control_pod",
    )
    stand_frame.visual(
        Box((0.030, 0.080, 0.250)),
        origin=Origin(xyz=(-0.135, -0.028, 0.155)),
        material=stand_black,
        name="left_support",
    )
    stand_frame.visual(
        Box((0.030, 0.080, 0.250)),
        origin=Origin(xyz=(0.135, -0.028, 0.155)),
        material=stand_black,
        name="right_support",
    )
    stand_frame.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(-0.128, -0.014, 0.247), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="left_bearing",
    )
    stand_frame.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.128, -0.014, 0.247), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="right_bearing",
    )

    shell_base = model.part("shell_base")
    shell_base.visual(
        mesh_from_cadquery(_make_shell_base(), "shell_base"),
        material=shell_black,
        name="lower_shell",
    )
    shell_base.visual(
        Cylinder(radius=0.095, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=cooking_iron,
        name="lower_plate",
    )
    shell_base.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(-0.114, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_steel,
        name="left_trunnion",
    )
    shell_base.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.114, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_steel,
        name="right_trunnion",
    )
    shell_base.visual(
        Box((0.054, 0.020, 0.014)),
        origin=Origin(xyz=(-0.066, -0.110, -0.001)),
        material=shell_black,
        name="body_hinge_mount_0",
    )
    shell_base.visual(
        Box((0.054, 0.020, 0.014)),
        origin=Origin(xyz=(0.066, -0.110, -0.001)),
        material=shell_black,
        name="body_hinge_mount_1",
    )
    shell_base.visual(
        Cylinder(radius=0.007, length=0.048),
        origin=Origin(xyz=(-0.066, -0.103, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_steel,
        name="body_hinge_0",
    )
    shell_base.visual(
        Cylinder(radius=0.007, length=0.048),
        origin=Origin(xyz=(0.066, -0.103, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_steel,
        name="body_hinge_1",
    )
    shell_base.visual(
        Box((0.106, 0.024, 0.015)),
        origin=Origin(xyz=(0.0, 0.109, -0.013)),
        material=shell_steel,
        name="body_handle",
    )

    shell_lid = model.part("shell_lid")
    shell_lid.visual(
        mesh_from_cadquery(_make_shell_lid(), "shell_lid"),
        material=shell_black,
        name="lid_shell",
    )
    shell_lid.visual(
        Cylinder(radius=0.095, length=0.005),
        origin=Origin(xyz=(0.0, 0.103, 0.0045)),
        material=cooking_iron,
        name="upper_plate",
    )
    shell_lid.visual(
        Box((0.068, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, 0.001)),
        material=shell_black,
        name="lid_hinge_bridge",
    )
    shell_lid.visual(
        Cylinder(radius=0.007, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_steel,
        name="lid_hinge",
    )
    shell_lid.visual(
        Box((0.112, 0.026, 0.016)),
        origin=Origin(xyz=(0.0, 0.213, 0.022)),
        material=shell_steel,
        name="lid_handle",
    )

    model.articulation(
        "stand_to_shell",
        ArticulationType.CONTINUOUS,
        parent=stand_frame,
        child=shell_base,
        origin=Origin(xyz=(0.0, -0.014, 0.247)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell_base,
        child=shell_lid,
        origin=Origin(xyz=(0.0, -0.103, -0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=0.0,
            upper=1.95,
        ),
    )

    button_offsets = (-0.016, 0.016)
    for index, local_x in enumerate(button_offsets):
        button = model.part(f"program_button_{index}")
        button.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=button_dark,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=button_amber,
            name="button_cap",
        )

        model.articulation(
            f"stand_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=stand_frame,
            child=button,
            origin=Origin(xyz=(0.078 + local_x, 0.066, 0.065)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand_frame = object_model.get_part("stand_frame")
    shell_base = object_model.get_part("shell_base")
    shell_lid = object_model.get_part("shell_lid")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    flip_joint = object_model.get_articulation("stand_to_shell")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    button_joint_0 = object_model.get_articulation("stand_to_program_button_0")
    button_joint_1 = object_model.get_articulation("stand_to_program_button_1")

    for elem_name in (
        "body_hinge_mount_0",
        "body_hinge_mount_1",
        "body_hinge_0",
        "body_hinge_1",
        "left_trunnion",
        "right_trunnion",
    ):
        ctx.allow_overlap(
            shell_base,
            shell_lid,
            elem_a=elem_name,
            elem_b="lid_shell",
            reason="The rear hinge brackets are simplified as nested cover geometry under the lid shell.",
        )
    for button in (button_0, button_1):
        ctx.allow_overlap(
            button,
            stand_frame,
            elem_a="button_stem",
            elem_b="control_pod",
            reason="The hidden button stem is intentionally represented as sliding inside the control-pod pocket.",
        )

    with ctx.pose(
        {
            lid_hinge: 0.0,
            flip_joint: 0.0,
            button_joint_0: 0.0,
            button_joint_1: 0.0,
        }
    ):
        ctx.expect_overlap(
            shell_lid,
            shell_base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="lower_shell",
            min_overlap=0.18,
            name="closed lid covers the lower shell",
        )
        ctx.expect_gap(
            shell_lid,
            shell_base,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="lower_shell",
            max_gap=0.012,
            max_penetration=0.0,
            name="closed lid seats above the lower shell without penetration",
        )
        ctx.expect_gap(
            button_0,
            stand_frame,
            axis="z",
            positive_elem="button_cap",
            negative_elem="control_pod",
            min_gap=0.0,
            max_gap=0.002,
            name="first program button sits proud of the control pod",
        )
        ctx.expect_gap(
            button_1,
            stand_frame,
            axis="z",
            positive_elem="button_cap",
            negative_elem="control_pod",
            min_gap=0.0,
            max_gap=0.002,
            name="second program button sits proud of the control pod",
        )
        ctx.expect_gap(
            button_1,
            button_0,
            axis="x",
            min_gap=0.010,
            name="program buttons remain visibly discrete",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(shell_lid, elem="lid_shell")
        closed_handle_aabb = ctx.part_element_world_aabb(shell_base, elem="body_handle")
        button_0_rest = ctx.part_element_world_aabb(button_0, elem="button_cap")
        button_1_rest = ctx.part_element_world_aabb(button_1, elem="button_cap")

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper, flip_joint: 0.0}):
            open_lid_aabb = ctx.part_element_world_aabb(shell_lid, elem="lid_shell")
        ctx.check(
            "lid opens upward from the rear hinge",
            _aabb_max_z(open_lid_aabb) is not None
            and _aabb_max_z(closed_lid_aabb) is not None
            and _aabb_max_z(open_lid_aabb) > _aabb_max_z(closed_lid_aabb) + 0.055,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    with ctx.pose({flip_joint: math.pi, lid_hinge: 0.0}):
        flipped_handle_aabb = ctx.part_element_world_aabb(shell_base, elem="body_handle")
    ctx.check(
        "shell flips around the side support axis",
        _aabb_center_y(closed_handle_aabb) is not None
        and _aabb_center_y(flipped_handle_aabb) is not None
        and _aabb_center_y(closed_handle_aabb) > 0.05
        and _aabb_center_y(flipped_handle_aabb) < -0.05,
        details=f"closed={closed_handle_aabb}, flipped={flipped_handle_aabb}",
    )

    button_limits_0 = button_joint_0.motion_limits
    if button_limits_0 is not None and button_limits_0.upper is not None:
        with ctx.pose({button_joint_0: button_limits_0.upper}):
            button_0_pressed = ctx.part_element_world_aabb(button_0, elem="button_cap")
        ctx.check(
            "first program button depresses downward",
            button_0_rest is not None
            and button_0_pressed is not None
            and button_0_pressed[0][2] < button_0_rest[0][2] - 0.0015,
            details=f"rest={button_0_rest}, pressed={button_0_pressed}",
        )

    button_limits_1 = button_joint_1.motion_limits
    if button_limits_1 is not None and button_limits_1.upper is not None:
        with ctx.pose({button_joint_1: button_limits_1.upper}):
            button_1_pressed = ctx.part_element_world_aabb(button_1, elem="button_cap")
        ctx.check(
            "second program button depresses downward",
            button_1_rest is not None
            and button_1_pressed is not None
            and button_1_pressed[0][2] < button_1_rest[0][2] - 0.0015,
            details=f"rest={button_1_rest}, pressed={button_1_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
