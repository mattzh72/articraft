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


CABINET_DEPTH = 0.34
CABINET_WIDTH = 0.42
CABINET_BOTTOM = 0.045
CABINET_TOP_FRONT = 0.615
CABINET_TOP_REAR = 0.68
BODY_FRONT_X = CABINET_DEPTH * 0.5
BODY_REAR_X = -CABINET_DEPTH * 0.5
BODY_HALF_WIDTH = CABINET_WIDTH * 0.5
WALL = 0.012

BUCKET_TRAVEL = 0.145
CONSOLE_PITCH = math.atan((CABINET_TOP_REAR - CABINET_TOP_FRONT) / (0.22))


def _console_z(x_pos: float) -> float:
    front_break_x = 0.10
    rear_break_x = -0.12
    slope = (CABINET_TOP_REAR - CABINET_TOP_FRONT) / (rear_break_x - front_break_x)
    return CABINET_TOP_FRONT + slope * (x_pos - front_break_x)


def _make_cabinet_shell():
    outer_profile = [
        (BODY_FRONT_X, CABINET_BOTTOM),
        (BODY_FRONT_X, 0.585),
        (0.10, CABINET_TOP_FRONT),
        (-0.12, CABINET_TOP_REAR),
        (BODY_REAR_X, CABINET_TOP_REAR),
        (BODY_REAR_X, CABINET_BOTTOM),
    ]
    inner_profile = [
        (BODY_FRONT_X - WALL, CABINET_BOTTOM + 0.010),
        (BODY_FRONT_X - WALL, 0.560),
        (0.093, 0.592),
        (-0.130, 0.655),
        (BODY_REAR_X + WALL, 0.655),
        (BODY_REAR_X + WALL, CABINET_BOTTOM + 0.010),
    ]

    outer = (
        cq.Workplane("XY")
        .polyline(outer_profile)
        .close()
        .extrude(CABINET_WIDTH)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, BODY_HALF_WIDTH, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .polyline(inner_profile)
        .close()
        .extrude(CABINET_WIDTH - 2.0 * WALL)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, BODY_HALF_WIDTH - WALL, 0.0))
    )

    bucket_opening = (
        cq.Workplane("XY")
        .box(0.050, 0.356, 0.242)
        .translate((BODY_FRONT_X - 0.018, 0.0, 0.171))
    )
    filter_opening = (
        cq.Workplane("XY")
        .box(0.050, 0.286, 0.340)
        .translate((BODY_REAR_X + 0.018, 0.0, 0.410))
    )

    return outer.cut(inner).cut(bucket_opening).cut(filter_opening)


def _make_bucket_shape():
    reservoir_outer = (
        cq.Workplane("XY")
        .box(0.232, 0.324, 0.212)
        .translate((-0.116, 0.0, 0.106))
    )
    reservoir_inner = (
        cq.Workplane("XY")
        .box(0.222, 0.314, 0.206)
        .translate((-0.116, 0.0, 0.109))
    )
    front_face = (
        cq.Workplane("XY")
        .box(0.016, 0.336, 0.228)
        .translate((-0.008, 0.0, 0.114))
    )
    grip_recess = (
        cq.Workplane("XY")
        .box(0.010, 0.120, 0.032)
        .translate((-0.002, 0.0, 0.142))
    )

    return reservoir_outer.cut(reservoir_inner).union(front_face).cut(grip_recess)


def _add_caster_yoke(
    cabinet,
    *,
    x_pos: float,
    y_pos: float,
    index: int,
    material,
) -> None:
    cabinet.visual(
        Box((0.036, 0.032, 0.006)),
        origin=Origin(xyz=(x_pos, y_pos, 0.042)),
        material=material,
        name=f"caster_bridge_{index}",
    )
    for sign in (-1.0, 1.0):
        cabinet.visual(
            Box((0.022, 0.004, 0.024)),
            origin=Origin(xyz=(x_pos, y_pos + sign * 0.010, 0.027)),
            material=material,
            name=f"caster_leg_{index}_{int(sign > 0.0)}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basement_dehumidifier")

    cabinet_plastic = model.material("cabinet_plastic", rgba=(0.84, 0.85, 0.83, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    bucket_plastic = model.material("bucket_plastic", rgba=(0.90, 0.91, 0.93, 1.0))
    control_dark = model.material("control_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    button_light = model.material("button_light", rgba=(0.78, 0.80, 0.82, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.32, 0.33, 0.35, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_make_cabinet_shell(), "cabinet_shell"),
        material=cabinet_plastic,
        name="cabinet_shell",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.020, 0.0, _console_z(0.020) + 0.002), rpy=(0.0, CONSOLE_PITCH, 0.0)),
        material=trim_dark,
        name="knob_plinth",
    )
    for index, y_pos in enumerate((-0.052, 0.052)):
        cabinet.visual(
            Box((0.018, 0.010, 0.003)),
            origin=Origin(xyz=(-0.028, y_pos, _console_z(-0.028) + 0.0015), rpy=(0.0, CONSOLE_PITCH, 0.0)),
            material=trim_dark,
            name=f"button_plinth_{index}",
        )
    for index, y_pos in enumerate((-0.150, 0.150)):
        cabinet.visual(
            Box((0.210, 0.022, 0.005)),
            origin=Origin(xyz=(0.010, y_pos, 0.0575)),
            material=trim_dark,
            name=f"bucket_rail_{index}",
        )
    cabinet.visual(
        Box((0.012, 0.010, 0.300)),
        origin=Origin(xyz=(BODY_REAR_X - 0.006, 0.170, 0.410)),
        material=trim_dark,
        name="filter_mount",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (0.132, 0.168),
            (0.132, -0.168),
            (-0.132, 0.168),
            (-0.132, -0.168),
        )
    ):
        _add_caster_yoke(cabinet, x_pos=x_pos, y_pos=y_pos, index=index, material=trim_dark)

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_make_bucket_shape(), "bucket_reservoir"),
        material=bucket_plastic,
        name="bucket_reservoir",
    )
    bucket.visual(
        Box((0.016, 0.336, 0.228)),
        origin=Origin(xyz=(-0.008, 0.0, 0.114)),
        material=bucket_plastic,
        name="bucket_face",
    )
    model.articulation(
        "bucket_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=bucket,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.22,
            lower=0.0,
            upper=BUCKET_TRAVEL,
        ),
    )

    filter_panel = model.part("filter_panel")
    panel_width = 0.306
    panel_height = 0.350
    stile = 0.018
    rail = 0.018
    slat_span = panel_width - 0.036
    filter_panel.visual(
        Box((0.010, stile, panel_height)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=trim_dark,
        name="hinge_stile",
    )
    filter_panel.visual(
        Box((0.010, stile, panel_height)),
        origin=Origin(xyz=(0.0, -(panel_width - 0.009), 0.0)),
        material=trim_dark,
        name="free_stile",
    )
    filter_panel.visual(
        Box((0.010, panel_width, rail)),
        origin=Origin(xyz=(0.0, -(panel_width * 0.5), panel_height * 0.5 - rail * 0.5)),
        material=trim_dark,
        name="top_rail",
    )
    filter_panel.visual(
        Box((0.010, panel_width, rail)),
        origin=Origin(xyz=(0.0, -(panel_width * 0.5), -(panel_height * 0.5 - rail * 0.5))),
        material=trim_dark,
        name="bottom_rail",
    )
    for slat_index, z_pos in enumerate((-0.102, -0.050, 0.002, 0.054, 0.106)):
        filter_panel.visual(
            Box((0.006, slat_span, 0.010)),
            origin=Origin(xyz=(0.0, -(panel_width * 0.5), z_pos)),
            material=trim_dark,
            name=f"slat_{slat_index}",
        )
    filter_panel.visual(
        Cylinder(radius=0.006, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    filter_panel.visual(
        Box((0.014, 0.010, 0.040)),
        origin=Origin(xyz=(-0.005, -(panel_width - 0.005), 0.0)),
        material=trim_dark,
        name="finger_tab",
    )
    model.articulation(
        "filter_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=filter_panel,
        origin=Origin(xyz=(BODY_REAR_X - 0.006, panel_width * 0.5, 0.410)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.4,
            lower=0.0,
            upper=1.95,
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=control_dark,
        name="knob_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.021, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=control_dark,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.004, 0.012, 0.003)),
        origin=Origin(xyz=(0.022, 0.0, 0.033)),
        material=button_light,
        name="pointer",
    )
    model.articulation(
        "selector_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.020, 0.0, _console_z(0.020)), rpy=(0.0, CONSOLE_PITCH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    for index, y_pos in enumerate((-0.052, 0.052)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.030, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=button_light,
            name="button_cap",
        )
        button.visual(
            Box((0.020, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=trim_dark,
            name="button_insert",
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(-0.028, y_pos, _console_z(-0.028)), rpy=(0.0, CONSOLE_PITCH, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    for index, (x_pos, y_pos) in enumerate(
        (
            (0.132, 0.168),
            (0.132, -0.168),
            (-0.132, 0.168),
            (-0.132, -0.168),
        )
    ):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_core,
            name="hub",
        )
        model.articulation(
            f"caster_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, 0.018)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    bucket = object_model.get_part("bucket")
    filter_panel = object_model.get_part("filter_panel")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    selector_knob = object_model.get_part("selector_knob")

    bucket_slide = object_model.get_articulation("bucket_slide")
    filter_hinge = object_model.get_articulation("filter_hinge")
    button_0_press = object_model.get_articulation("button_0_press")
    button_1_press = object_model.get_articulation("button_1_press")

    ctx.allow_isolated_part(
        selector_knob,
        reason="The rotary selector sits on a very tight cosmetic plinth gap above the sloped console.",
    )
    ctx.allow_isolated_part(
        button_0,
        reason="The raised push button keeps a small cosmetic seam above its console plinth.",
    )
    ctx.allow_isolated_part(
        button_1,
        reason="The raised push button keeps a small cosmetic seam above its console plinth.",
    )

    bucket_limits = bucket_slide.motion_limits
    filter_limits = filter_hinge.motion_limits
    button_0_limits = button_0_press.motion_limits
    button_1_limits = button_1_press.motion_limits

    closed_bucket_pos = ctx.part_world_position(bucket)
    cabinet_shell_aabb = ctx.part_element_world_aabb(cabinet, elem="cabinet_shell")
    ctx.expect_within(
        bucket,
        cabinet,
        axes="yz",
        inner_elem="bucket_reservoir",
        outer_elem="cabinet_shell",
        margin=0.020,
        name="bucket reservoir stays within cabinet opening width and height",
    )
    ctx.expect_overlap(
        bucket,
        cabinet,
        axes="x",
        elem_a="bucket_reservoir",
        elem_b="cabinet_shell",
        min_overlap=0.180,
        name="bucket remains deeply inserted when closed",
    )
    ctx.check(
        "bucket face sits flush with cabinet front",
        closed_bucket_pos is not None
        and cabinet_shell_aabb is not None
        and abs(closed_bucket_pos[0] - cabinet_shell_aabb[1][0]) <= 0.003,
        details=f"bucket_pos={closed_bucket_pos}, cabinet_shell_aabb={cabinet_shell_aabb}",
    )

    extended_bucket_pos = None
    if bucket_limits is not None and bucket_limits.upper is not None:
        with ctx.pose({bucket_slide: bucket_limits.upper}):
            ctx.expect_within(
                bucket,
                cabinet,
                axes="yz",
                inner_elem="bucket_reservoir",
                outer_elem="cabinet_shell",
                margin=0.020,
                name="bucket remains aligned while extended",
            )
            ctx.expect_overlap(
                bucket,
                cabinet,
                axes="x",
                elem_a="bucket_reservoir",
                elem_b="cabinet_shell",
                min_overlap=0.085,
                name="bucket keeps retained insertion at full extension",
            )
            extended_bucket_pos = ctx.part_world_position(bucket)
    ctx.check(
        "bucket pulls forward",
        closed_bucket_pos is not None
        and extended_bucket_pos is not None
        and extended_bucket_pos[0] > closed_bucket_pos[0] + 0.12,
        details=f"closed={closed_bucket_pos}, extended={extended_bucket_pos}",
    )

    closed_filter_aabb = ctx.part_world_aabb(filter_panel)
    ctx.expect_gap(
        cabinet,
        filter_panel,
        axis="x",
        positive_elem="cabinet_shell",
        max_gap=0.012,
        max_penetration=0.008,
        name="filter panel sits close to the rear opening when closed",
    )
    ctx.expect_overlap(
        filter_panel,
        cabinet,
        axes="yz",
        elem_b="cabinet_shell",
        min_overlap=0.220,
        name="filter panel covers the rear opening footprint",
    )
    open_filter_aabb = None
    if filter_limits is not None and filter_limits.upper is not None:
        with ctx.pose({filter_hinge: filter_limits.upper}):
            open_filter_aabb = ctx.part_world_aabb(filter_panel)
    ctx.check(
        "filter panel swings outward from the cabinet rear",
        closed_filter_aabb is not None
        and open_filter_aabb is not None
        and open_filter_aabb[0][0] < closed_filter_aabb[0][0] - 0.080,
        details=f"closed={closed_filter_aabb}, open={open_filter_aabb}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_0_pressed = None
    if button_0_limits is not None and button_0_limits.upper is not None:
        with ctx.pose({button_0_press: button_0_limits.upper}):
            button_0_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "button 0 depresses into the sloped console",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.003,
        details=f"rest={button_0_rest}, pressed={button_0_pressed}",
    )

    button_1_rest = ctx.part_world_position(button_1)
    button_1_pressed = None
    if button_1_limits is not None and button_1_limits.upper is not None:
        with ctx.pose({button_1_press: button_1_limits.upper}):
            button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "button 1 depresses into the sloped console",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.003,
        details=f"rest={button_1_rest}, pressed={button_1_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
