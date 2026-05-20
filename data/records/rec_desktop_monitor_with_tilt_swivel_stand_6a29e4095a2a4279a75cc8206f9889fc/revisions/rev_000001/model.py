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


def _pedestal_base_shape() -> object:
    return (
        cq.Workplane("XY")
        .ellipse(0.105, 0.090)
        .workplane(offset=0.016)
        .ellipse(0.072, 0.058)
        .loft(combine=True)
        .edges(">Z")
        .fillet(0.004)
    )


def _outer_sleeve_shape() -> object:
    sleeve = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.008))
        .rect(0.030, 0.046)
        .extrude(0.180)
    )
    sleeve = sleeve.faces(">Z").workplane().rect(0.022, 0.038).cutBlind(-0.170)
    return sleeve.edges("|Z").fillet(0.003)


def _display_housing_shape() -> object:
    body = (
        cq.Workplane("XY")
        .box(0.045, 0.516, 0.315)
        .translate((-0.060, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.011)
    )
    body = (
        body.faces("<X")
        .workplane(centerOption="CenterOfBoundBox")
        .center(0.0, 0.012)
        .rect(0.456, 0.248)
        .cutBlind(0.006)
    )
    body = (
        body.faces(">X")
        .workplane(centerOption="CenterOfBoundBox")
        .rect(0.108, 0.074)
        .extrude(0.008)
    )

    for y_pos in (0.176, 0.148, 0.120, 0.092):
        body = (
            body.faces("<X")
            .workplane(centerOption="CenterOfBoundBox")
            .center(y_pos, -0.132)
            .rect(0.012, 0.006)
            .cutBlind(0.014)
        )

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor")

    housing = model.material("housing", rgba=(0.19, 0.20, 0.22, 1.0))
    trim = model.material("trim", rgba=(0.28, 0.29, 0.31, 1.0))
    stand = model.material("stand", rgba=(0.38, 0.40, 0.43, 1.0))
    button = model.material("button", rgba=(0.17, 0.18, 0.19, 1.0))
    screen = model.material("screen", rgba=(0.06, 0.08, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_pedestal_base_shape(), "monitor_base"),
        material=trim,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=stand,
        name="swivel_plinth",
    )

    outer_column = model.part("outer_column")
    outer_column.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=stand,
        name="swivel_collar",
    )
    outer_column.visual(
        Box((0.002, 0.046, 0.180)),
        origin=Origin(xyz=(-0.012, 0.0, 0.098)),
        material=stand,
        name="sleeve_front",
    )
    outer_column.visual(
        Box((0.002, 0.046, 0.180)),
        origin=Origin(xyz=(0.012, 0.0, 0.098)),
        material=stand,
        name="sleeve_rear",
    )
    outer_column.visual(
        Box((0.022, 0.004, 0.180)),
        origin=Origin(xyz=(0.0, -0.021, 0.098)),
        material=stand,
        name="sleeve_left",
    )
    outer_column.visual(
        Box((0.022, 0.004, 0.180)),
        origin=Origin(xyz=(0.0, 0.021, 0.098)),
        material=stand,
        name="sleeve_right",
    )
    outer_column.visual(
        Box((0.022, 0.038, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stand,
        name="sleeve_floor",
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Box((0.022, 0.036, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=stand,
        name="mast_shank",
    )
    inner_mast.visual(
        Box((0.020, 0.040, 0.110)),
        origin=Origin(xyz=(0.011, 0.0, 0.045)),
        material=stand,
        name="head_neck",
    )
    inner_mast.visual(
        Box((0.014, 0.040, 0.050)),
        origin=Origin(xyz=(0.022, 0.0, 0.105)),
        material=stand,
        name="head_pad",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_display_housing_shape(), "monitor_display_housing"),
        material=housing,
        name="housing",
    )
    display.visual(
        Box((0.052, 0.030, 0.060)),
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
        material=housing,
        name="rear_mount",
    )
    display.visual(
        Box((0.007, 0.446, 0.240)),
        origin=Origin(xyz=(-0.0790, 0.0, 0.012)),
        material=screen,
        name="screen",
    )

    model.articulation(
        "stand_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=outer_column,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )
    model.articulation(
        "neck_slide",
        ArticulationType.PRISMATIC,
        parent=outer_column,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "display_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=display,
        origin=Origin(xyz=(0.015, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-0.35,
            upper=0.50,
        ),
    )

    for index, y_pos in enumerate((0.176, 0.148, 0.120, 0.092)):
        button_part = model.part(f"button_{index}")
        button_part.visual(
            Box((0.008, 0.016, 0.006)),
            origin=Origin(xyz=(-0.004, 0.0, 0.0)),
            material=button,
            name="cap",
        )
        button_part.visual(
            Box((0.010, 0.012, 0.006)),
            origin=Origin(xyz=(0.003, 0.0, 0.0)),
            material=button,
            name="stem",
        )
        model.articulation(
            f"display_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button_part,
            origin=Origin(xyz=(-0.0785, y_pos, -0.132)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_column = object_model.get_part("outer_column")
    inner_mast = object_model.get_part("inner_mast")
    display = object_model.get_part("display")
    neck_slide = object_model.get_articulation("neck_slide")
    stand_swivel = object_model.get_articulation("stand_swivel")
    display_tilt = object_model.get_articulation("display_tilt")

    ctx.allow_overlap(
        display,
        inner_mast,
        elem_a="rear_mount",
        elem_b="head_neck",
        reason="The tilt hinge is simplified as a nested neck tongue inside the display's rear hinge housing.",
    )

    ctx.expect_within(
        inner_mast,
        outer_column,
        axes="xy",
        inner_elem="mast_shank",
        margin=0.0006,
        name="mast stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        inner_mast,
        outer_column,
        axes="z",
        elem_a="mast_shank",
        min_overlap=0.135,
        name="mast retains insertion at rest",
    )

    rest_display_pos = ctx.part_world_position(display)
    with ctx.pose({neck_slide: 0.080}):
        ctx.expect_within(
            inner_mast,
            outer_column,
            axes="xy",
            inner_elem="mast_shank",
            margin=0.0006,
            name="mast stays centered in sleeve when raised",
        )
        ctx.expect_overlap(
            inner_mast,
            outer_column,
            axes="z",
            elem_a="mast_shank",
            min_overlap=0.055,
            name="mast retains insertion when raised",
        )
        raised_display_pos = ctx.part_world_position(display)

    ctx.check(
        "neck raises display",
        rest_display_pos is not None
        and raised_display_pos is not None
        and raised_display_pos[2] > rest_display_pos[2] + 0.07,
        details=f"rest={rest_display_pos}, raised={raised_display_pos}",
    )

    rest_aabb = ctx.part_world_aabb(display)
    with ctx.pose({display_tilt: 0.50}):
        tilted_aabb = ctx.part_world_aabb(display)
    rest_x_span = rest_aabb[1][0] - rest_aabb[0][0] if rest_aabb is not None else None
    tilt_x_span = tilted_aabb[1][0] - tilted_aabb[0][0] if tilted_aabb is not None else None
    ctx.check(
        "display tilts about horizontal hinge",
        rest_x_span is not None and tilt_x_span is not None and tilt_x_span > rest_x_span + 0.09,
        details=f"rest_x_span={rest_x_span}, tilt_x_span={tilt_x_span}",
    )

    with ctx.pose({stand_swivel: math.pi / 2.0}):
        swivel_aabb = ctx.part_world_aabb(display)
    rest_y_span = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else None
    rest_depth = rest_aabb[1][0] - rest_aabb[0][0] if rest_aabb is not None else None
    swivel_width = swivel_aabb[1][0] - swivel_aabb[0][0] if swivel_aabb is not None else None
    swivel_y_span = swivel_aabb[1][1] - swivel_aabb[0][1] if swivel_aabb is not None else None
    ctx.check(
        "display swivels around vertical stand axis",
        rest_y_span is not None
        and rest_depth is not None
        and swivel_width is not None
        and swivel_y_span is not None
        and rest_y_span > 0.45
        and rest_depth < 0.12
        and abs(swivel_width - rest_y_span) < 0.03
        and abs(swivel_y_span - rest_depth) < 0.03,
        details=(
            f"rest_y_span={rest_y_span}, rest_depth={rest_depth}, "
            f"swivel_width={swivel_width}, swivel_y_span={swivel_y_span}"
        ),
    )

    housing_aabb = ctx.part_element_world_aabb(display, elem="housing")
    housing_front_x = housing_aabb[0][0] if housing_aabb is not None else None
    for index in range(4):
        button_part = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"display_to_button_{index}")

        rest_cap_aabb = ctx.part_element_world_aabb(button_part, elem="cap")
        with ctx.pose({button_joint: 0.0015}):
            pressed_cap_aabb = ctx.part_element_world_aabb(button_part, elem="cap")

        rest_front = rest_cap_aabb[0][0] if rest_cap_aabb is not None else None
        pressed_front = pressed_cap_aabb[0][0] if pressed_cap_aabb is not None else None

        ctx.check(
            f"button_{index} sits proud of lower bezel",
            housing_front_x is not None and rest_front is not None and rest_front < housing_front_x - 0.0025,
            details=f"housing_front_x={housing_front_x}, button_front_x={rest_front}",
        )
        ctx.check(
            f"button_{index} presses inward",
            rest_front is not None and pressed_front is not None and pressed_front > rest_front + 0.001,
            details=f"rest_front={rest_front}, pressed_front={pressed_front}",
        )

    return ctx.report()


object_model = build_object_model()
