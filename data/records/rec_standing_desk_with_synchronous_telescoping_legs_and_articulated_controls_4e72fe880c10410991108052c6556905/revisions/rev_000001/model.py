from __future__ import annotations

import cadquery as cq

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
    mesh_from_cadquery,
)


FOOT_THICKNESS = 0.03
OUTER_HEIGHT = 0.55
OUTER_WIDTH = 0.09
OUTER_DEPTH = 0.06
OUTER_WALL = 0.005
INNER_WIDTH = OUTER_WIDTH - 2.0 * OUTER_WALL
INNER_DEPTH = OUTER_DEPTH - 2.0 * OUTER_WALL
STAGE_LENGTH = 0.57
STAGE_BODY_TOP = 0.075
STAGE_BODY_CENTER_Z = (STAGE_BODY_TOP - (STAGE_LENGTH - STAGE_BODY_TOP)) / 2.0
STAGE_HEAD_HEIGHT = 0.03
STAGE_HEAD_CENTER_Z = STAGE_BODY_TOP + STAGE_HEAD_HEIGHT / 2.0
LIFT_TRAVEL = 0.42
LIFT_ORIGIN_Z = FOOT_THICKNESS + OUTER_HEIGHT


def _outer_column_mesh(name: str) -> object:
    outer = cq.Workplane("XY").box(OUTER_WIDTH, OUTER_DEPTH, OUTER_HEIGHT).translate(
        (0.0, 0.0, OUTER_HEIGHT / 2.0)
    )
    inner_height = OUTER_HEIGHT - OUTER_WALL + 0.002
    inner = cq.Workplane("XY").box(INNER_WIDTH, INNER_DEPTH, inner_height).translate(
        (0.0, 0.0, OUTER_WALL + inner_height / 2.0)
    )
    sleeve = outer.cut(inner).edges("|Z").fillet(0.003)
    return mesh_from_cadquery(sleeve, name, tolerance=0.0008, angular_tolerance=0.08)


def _desktop_mesh(name: str) -> object:
    outline = [
        (-0.20, -0.35),
        (1.55, -0.35),
        (1.55, 0.40),
        (0.70, 0.40),
        (0.70, 1.10),
        (-0.20, 1.10),
    ]
    top = (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(0.036)
        .edges("|Z")
        .fillet(0.025)
    )
    return mesh_from_cadquery(top, name, tolerance=0.0012, angular_tolerance=0.1)


def _controller_mesh(name: str) -> object:
    width = 0.17
    depth = 0.05
    height = 0.03
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-width / 2.0, 0.0),
                (width / 2.0, 0.0),
                (width / 2.0, -0.018),
                (width / 2.0 - 0.010, -height),
                (-width / 2.0 + 0.008, -height),
                (-width / 2.0, -0.018),
            ]
        )
        .close()
        .extrude(depth)
    )

    for x_pos in (-0.048, -0.020, 0.008):
        pocket = cq.Workplane("XY").box(0.026, 0.012, 0.018).translate((x_pos, 0.006, -0.017))
        body = body.cut(pocket)

    power_pocket = cq.Workplane("XY").box(0.018, 0.012, 0.018).translate((0.056, 0.006, -0.017))
    body = body.cut(power_pocket)
    body = body.edges("|Y").fillet(0.002)
    return mesh_from_cadquery(body, name, tolerance=0.0007, angular_tolerance=0.08)


def _add_column_base(model: ArticulatedObject, name: str, *, foot_size: tuple[float, float, float]) -> object:
    part = model.part(name)
    part.visual(
        Box(foot_size),
        origin=Origin(xyz=(0.0, 0.0, foot_size[2] / 2.0)),
        material="graphite",
        name="foot",
    )
    part.visual(
        _outer_column_mesh(f"{name}_outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, FOOT_THICKNESS)),
        material="graphite",
        name="outer_sleeve",
    )
    return part


def _add_stage(model: ArticulatedObject, name: str) -> object:
    part = model.part(name)
    part.visual(
        Box((INNER_WIDTH, INNER_DEPTH - 0.002, STAGE_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, STAGE_BODY_CENTER_Z)),
        material="silver",
        name="stage_body",
    )
    part.visual(
        Box((0.12, 0.10, STAGE_HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, STAGE_HEAD_CENTER_Z)),
        material="graphite",
        name="mount_head",
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_standing_desk")

    model.material("graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("silver", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("oak", rgba=(0.73, 0.62, 0.45, 1.0))
    model.material("controller_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("button_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("indicator_red", rgba=(0.50, 0.12, 0.12, 1.0))

    front_return_base = _add_column_base(
        model,
        "front_return_base",
        foot_size=(0.10, 0.68, FOOT_THICKNESS),
    )
    front_outer_base = _add_column_base(
        model,
        "front_outer_base",
        foot_size=(0.10, 0.68, FOOT_THICKNESS),
    )
    rear_return_base = _add_column_base(
        model,
        "rear_return_base",
        foot_size=(0.58, 0.10, FOOT_THICKNESS),
    )

    model.articulation(
        "front_return_to_front_outer_base",
        ArticulationType.FIXED,
        parent=front_return_base,
        child=front_outer_base,
        origin=Origin(xyz=(1.38, 0.02, 0.0)),
    )
    model.articulation(
        "front_return_to_rear_return_base",
        ArticulationType.FIXED,
        parent=front_return_base,
        child=rear_return_base,
        origin=Origin(xyz=(0.22, 0.95, 0.0)),
    )

    front_return_stage = _add_stage(model, "front_return_stage")
    front_outer_stage = _add_stage(model, "front_outer_stage")
    rear_return_stage = _add_stage(model, "rear_return_stage")

    lift_limits = MotionLimits(
        effort=800.0,
        velocity=0.06,
        lower=0.0,
        upper=LIFT_TRAVEL,
    )
    front_return_lift = model.articulation(
        "front_return_lift",
        ArticulationType.PRISMATIC,
        parent=front_return_base,
        child=front_return_stage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "front_outer_lift",
        ArticulationType.PRISMATIC,
        parent=front_outer_base,
        child=front_outer_stage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.06,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
        mimic=Mimic(joint="front_return_lift"),
    )
    model.articulation(
        "rear_return_lift",
        ArticulationType.PRISMATIC,
        parent=rear_return_base,
        child=rear_return_stage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.06,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
        mimic=Mimic(joint="front_return_lift"),
    )

    desktop = model.part("desktop")
    desktop.visual(
        _desktop_mesh("desktop_top"),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material="oak",
        name="desktop_top",
    )
    desktop.visual(
        Box((0.18, 0.16, 0.05)),
        origin=Origin(xyz=(0.05, 0.05, 0.025)),
        material="graphite",
        name="primary_mount",
    )
    desktop.visual(
        Box((1.26, 0.09, 0.05)),
        origin=Origin(xyz=(0.69, 0.02, 0.025)),
        material="graphite",
        name="front_beam",
    )
    desktop.visual(
        Box((0.14, 0.12, 0.05)),
        origin=Origin(xyz=(1.38, 0.02, 0.025)),
        material="graphite",
        name="outer_mount",
    )
    desktop.visual(
        Box((0.10, 0.88, 0.05)),
        origin=Origin(xyz=(0.22, 0.48, 0.025)),
        material="graphite",
        name="return_beam",
    )
    desktop.visual(
        Box((0.14, 0.14, 0.05)),
        origin=Origin(xyz=(0.22, 0.95, 0.025)),
        material="graphite",
        name="rear_mount",
    )
    desktop.visual(
        Box((0.42, 0.09, 0.05)),
        origin=Origin(xyz=(0.20, 0.95, 0.025)),
        material="graphite",
        name="rear_crossbeam",
    )
    desktop.visual(
        Box((0.30, 0.20, 0.05)),
        origin=Origin(xyz=(0.17, 0.12, 0.025)),
        material="graphite",
        name="corner_gusset",
    )

    model.articulation(
        "front_return_stage_to_desktop",
        ArticulationType.FIXED,
        parent=front_return_stage,
        child=desktop,
        origin=Origin(xyz=(0.0, 0.0, STAGE_BODY_TOP + STAGE_HEAD_HEIGHT)),
    )

    controller_pod = model.part("controller_pod")
    controller_pod.visual(
        Box((0.17, 0.05, 0.007)),
        origin=Origin(xyz=(0.0, 0.025, -0.0035)),
        material="controller_black",
        name="housing",
    )
    controller_pod.visual(
        Box((0.17, 0.007, 0.026)),
        origin=Origin(xyz=(0.0, 0.0465, -0.017)),
        material="controller_black",
        name="rear_wall",
    )
    controller_pod.visual(
        Box((0.007, 0.05, 0.026)),
        origin=Origin(xyz=(-0.0815, 0.025, -0.017)),
        material="controller_black",
        name="left_wall",
    )
    controller_pod.visual(
        Box((0.007, 0.05, 0.026)),
        origin=Origin(xyz=(0.0815, 0.025, -0.017)),
        material="controller_black",
        name="right_wall",
    )
    controller_pod.visual(
        Box((0.17, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, 0.026, -0.027)),
        material="controller_black",
        name="bottom_shell",
    )
    controller_pod.visual(
        Box((0.17, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.003, -0.008)),
        material="controller_black",
        name="front_strip_top",
    )
    controller_pod.visual(
        Box((0.17, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.003, -0.026)),
        material="controller_black",
        name="front_strip_bottom",
    )
    controller_pod.visual(
        Box((0.014, 0.006, 0.014)),
        origin=Origin(xyz=(0.056, 0.003, -0.017)),
        material="controller_black",
        name="power_bezel",
    )
    model.articulation(
        "desktop_to_controller_pod",
        ArticulationType.FIXED,
        parent=desktop,
        child=controller_pod,
        origin=Origin(xyz=(0.98, -0.35, 0.048)),
    )

    preset_x_positions = (-0.048, -0.020, 0.008)
    for index, x_pos in enumerate(preset_x_positions):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box((0.022, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material="button_black",
            name="cap",
        )
        model.articulation(
            f"preset_button_{index}_push",
            ArticulationType.PRISMATIC,
            parent=controller_pod,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, -0.017)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material="indicator_red",
        name="cap",
    )
    model.articulation(
        "power_button_push",
        ArticulationType.PRISMATIC,
        parent=controller_pod,
        child=power_button,
        origin=Origin(xyz=(0.056, 0.0, -0.017)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_return_base = object_model.get_part("front_return_base")
    front_outer_base = object_model.get_part("front_outer_base")
    rear_return_base = object_model.get_part("rear_return_base")
    front_return_stage = object_model.get_part("front_return_stage")
    front_outer_stage = object_model.get_part("front_outer_stage")
    rear_return_stage = object_model.get_part("rear_return_stage")
    desktop = object_model.get_part("desktop")
    controller_pod = object_model.get_part("controller_pod")
    preset_button_0 = object_model.get_part("preset_button_0")
    preset_button_1 = object_model.get_part("preset_button_1")
    power_button = object_model.get_part("power_button")

    front_return_lift = object_model.get_articulation("front_return_lift")
    preset_button_1_push = object_model.get_articulation("preset_button_1_push")
    power_button_push = object_model.get_articulation("power_button_push")

    ctx.allow_overlap(
        front_return_base,
        front_return_stage,
        elem_a="outer_sleeve",
        elem_b="stage_body",
        reason="The outer column is authored as an open shell visual, but overlap QC treats the sleeve volume as solid while the lift stage is intended to slide inside it.",
    )
    ctx.allow_overlap(
        front_outer_base,
        front_outer_stage,
        elem_a="outer_sleeve",
        elem_b="stage_body",
        reason="The outer column is authored as an open shell visual, but overlap QC treats the sleeve volume as solid while the lift stage is intended to slide inside it.",
    )
    ctx.allow_overlap(
        rear_return_base,
        rear_return_stage,
        elem_a="outer_sleeve",
        elem_b="stage_body",
        reason="The outer column is authored as an open shell visual, but overlap QC treats the sleeve volume as solid while the lift stage is intended to slide inside it.",
    )

    for base_name, stage_name in (
        (front_return_base, front_return_stage),
        (front_outer_base, front_outer_stage),
        (rear_return_base, rear_return_stage),
    ):
        ctx.expect_within(
            stage_name,
            base_name,
            axes="xy",
            inner_elem="stage_body",
            outer_elem="outer_sleeve",
            margin=0.0015,
            name=f"{stage_name.name} stays guided inside {base_name.name}",
        )
        ctx.expect_overlap(
            stage_name,
            base_name,
            axes="z",
            elem_a="stage_body",
            elem_b="outer_sleeve",
            min_overlap=0.07,
            name=f"{stage_name.name} retains insertion in {base_name.name}",
        )

    rest_desktop = ctx.part_world_position(desktop)
    with ctx.pose({front_return_lift: LIFT_TRAVEL}):
        ctx.expect_within(
            front_return_stage,
            front_return_base,
            axes="xy",
            inner_elem="stage_body",
            outer_elem="outer_sleeve",
            margin=0.0015,
            name="front return stage stays guided at full lift",
        )
        ctx.expect_within(
            front_outer_stage,
            front_outer_base,
            axes="xy",
            inner_elem="stage_body",
            outer_elem="outer_sleeve",
            margin=0.0015,
            name="front outer stage stays guided at full lift",
        )
        ctx.expect_within(
            rear_return_stage,
            rear_return_base,
            axes="xy",
            inner_elem="stage_body",
            outer_elem="outer_sleeve",
            margin=0.0015,
            name="rear return stage stays guided at full lift",
        )
        ctx.expect_overlap(
            front_return_stage,
            front_return_base,
            axes="z",
            elem_a="stage_body",
            elem_b="outer_sleeve",
            min_overlap=0.07,
            name="front return stage retains insertion at full lift",
        )
        ctx.expect_overlap(
            front_outer_stage,
            front_outer_base,
            axes="z",
            elem_a="stage_body",
            elem_b="outer_sleeve",
            min_overlap=0.07,
            name="front outer stage retains insertion at full lift",
        )
        ctx.expect_overlap(
            rear_return_stage,
            rear_return_base,
            axes="z",
            elem_a="stage_body",
            elem_b="outer_sleeve",
            min_overlap=0.07,
            name="rear return stage retains insertion at full lift",
        )
        ctx.expect_gap(
            desktop,
            front_outer_stage,
            axis="z",
            positive_elem="outer_mount",
            negative_elem="mount_head",
            max_gap=0.0,
            max_penetration=0.0,
            name="front outer stage meets the rigid corner frame at full lift",
        )
        ctx.expect_gap(
            desktop,
            rear_return_stage,
            axis="z",
            positive_elem="rear_mount",
            negative_elem="mount_head",
            max_gap=0.0,
            max_penetration=0.0,
            name="rear return stage meets the rigid corner frame at full lift",
        )
        raised_desktop = ctx.part_world_position(desktop)

    ctx.check(
        "desktop rises upward when the lift extends",
        rest_desktop is not None
        and raised_desktop is not None
        and raised_desktop[2] > rest_desktop[2] + 0.40,
        details=f"rest={rest_desktop}, raised={raised_desktop}",
    )

    ctx.expect_gap(
        desktop,
        controller_pod,
        axis="z",
        positive_elem="desktop_top",
        negative_elem="housing",
        max_gap=0.0,
        max_penetration=0.0,
        name="controller pod is mounted directly beneath the desktop",
    )

    preset_0_rest = ctx.part_world_position(preset_button_0)
    preset_1_rest = ctx.part_world_position(preset_button_1)
    with ctx.pose({preset_button_1_push: 0.004}):
        preset_1_pressed = ctx.part_world_position(preset_button_1)
        preset_0_unmoved = ctx.part_world_position(preset_button_0)

    ctx.check(
        "preset button 1 depresses without dragging the adjacent preset button",
        preset_1_rest is not None
        and preset_1_pressed is not None
        and preset_0_rest is not None
        and preset_0_unmoved is not None
        and preset_1_pressed[1] > preset_1_rest[1] + 0.003
        and abs(preset_0_unmoved[1] - preset_0_rest[1]) < 1e-6,
        details=(
            f"preset_1_rest={preset_1_rest}, preset_1_pressed={preset_1_pressed}, "
            f"preset_0_rest={preset_0_rest}, preset_0_unmoved={preset_0_unmoved}"
        ),
    )

    power_rest = ctx.part_world_position(power_button)
    with ctx.pose({power_button_push: 0.004}):
        power_pressed = ctx.part_world_position(power_button)

    ctx.check(
        "power button depresses independently",
        power_rest is not None and power_pressed is not None and power_pressed[1] > power_rest[1] + 0.003,
        details=f"power_rest={power_rest}, power_pressed={power_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
