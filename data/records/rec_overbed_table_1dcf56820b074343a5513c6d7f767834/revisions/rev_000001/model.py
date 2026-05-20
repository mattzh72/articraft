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


def _add_panel(
    part,
    *,
    panel_name: str,
    frame_name: str,
    size_x: float,
    size_y: float,
    thickness: float,
    y_direction: float,
    material,
    lip_material,
    frame_material,
    lip_height: float = 0.012,
    lip_thickness: float = 0.012,
) -> None:
    center_y = y_direction * size_y * 0.5
    outer_y = y_direction * (size_y - lip_thickness * 0.5)

    part.visual(
        Box((size_x, size_y, thickness)),
        origin=Origin(xyz=(0.0, center_y, thickness * 0.5)),
        material=material,
        name=panel_name,
    )
    part.visual(
        Box((lip_thickness, size_y - lip_thickness * 1.2, lip_height)),
        origin=Origin(xyz=(size_x * 0.5 - lip_thickness * 0.5, center_y, thickness + lip_height * 0.5)),
        material=lip_material,
        name=f"{panel_name}_front_lip",
    )
    part.visual(
        Box((lip_thickness, size_y - lip_thickness * 1.2, lip_height)),
        origin=Origin(xyz=(-size_x * 0.5 + lip_thickness * 0.5, center_y, thickness + lip_height * 0.5)),
        material=lip_material,
        name=f"{panel_name}_rear_lip",
    )
    part.visual(
        Box((size_x - lip_thickness * 2.2, lip_thickness, lip_height)),
        origin=Origin(xyz=(0.0, outer_y, thickness + lip_height * 0.5)),
        material=lip_material,
        name=f"{panel_name}_outer_lip",
    )
    part.visual(
        Box((size_x * 0.62, min(size_y * 0.24, 0.090), thickness)),
        origin=Origin(xyz=(0.0, y_direction * size_y * 0.34, -thickness * 0.5)),
        material=frame_material,
        name=frame_name,
    )


def _add_caster_wheel(part, *, tire_material, hub_material) -> None:
    wheel_rotation = Origin(rpy=(pi * 0.5, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.035, length=0.022),
        origin=wheel_rotation,
        material=tire_material,
        name="wheel_tire",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hub_material,
        name="wheel_hub_inner",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hub_material,
        name="wheel_hub_outer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehabilitation_overbed_table")

    frame_paint = model.material("frame_paint", rgba=(0.83, 0.85, 0.87, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.66, 0.69, 0.72, 1.0))
    top_laminate = model.material("top_laminate", rgba=(0.82, 0.74, 0.60, 1.0))
    top_edge = model.material("top_edge", rgba=(0.61, 0.50, 0.38, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    caster_hub = model.material("caster_hub", rgba=(0.54, 0.56, 0.58, 1.0))
    chrome = model.material("chrome", rgba=(0.75, 0.77, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.08, 0.66, 0.03)),
        origin=Origin(xyz=(0.04, 0.0, 0.10)),
        material=frame_paint,
        name="rear_crossbar",
    )
    base.visual(
        Box((0.74, 0.06, 0.03)),
        origin=Origin(xyz=(0.37, -0.28, 0.10)),
        material=frame_paint,
        name="base_rail_0",
    )
    base.visual(
        Box((0.74, 0.06, 0.03)),
        origin=Origin(xyz=(0.37, 0.28, 0.10)),
        material=frame_paint,
        name="base_rail_1",
    )
    base.visual(
        Box((0.14, 0.10, 0.02)),
        origin=Origin(xyz=(0.07, 0.0, 0.125)),
        material=trim_grey,
        name="mast_pedestal",
    )

    caster_positions = (
        ("caster_0", 0.09, -0.28),
        ("caster_1", 0.65, -0.28),
        ("caster_2", 0.09, 0.28),
        ("caster_3", 0.65, 0.28),
    )
    for _, caster_x, caster_y in caster_positions:
        base.visual(
            Box((0.05, 0.03, 0.025)),
            origin=Origin(xyz=(caster_x, caster_y, 0.0725)),
            material=trim_grey,
        )
        base.visual(
            Box((0.02, 0.008, 0.04)),
            origin=Origin(xyz=(caster_x, caster_y - 0.016, 0.04)),
            material=trim_grey,
        )
        base.visual(
            Box((0.02, 0.008, 0.04)),
            origin=Origin(xyz=(caster_x, caster_y + 0.016, 0.04)),
            material=trim_grey,
        )

    sleeve = model.part("sleeve")
    sleeve.visual(
        Box((0.078, 0.006, 0.42)),
        origin=Origin(xyz=(0.0, 0.026, 0.21)),
        material=frame_paint,
        name="sleeve_front",
    )
    sleeve.visual(
        Box((0.078, 0.006, 0.42)),
        origin=Origin(xyz=(0.0, -0.026, 0.21)),
        material=frame_paint,
        name="sleeve_back",
    )
    sleeve.visual(
        Box((0.006, 0.046, 0.42)),
        origin=Origin(xyz=(0.036, 0.0, 0.21)),
        material=frame_paint,
        name="sleeve_side_0",
    )
    sleeve.visual(
        Box((0.006, 0.046, 0.42)),
        origin=Origin(xyz=(-0.036, 0.0, 0.21)),
        material=frame_paint,
        name="sleeve_side_1",
    )
    sleeve.visual(
        Box((0.098, 0.078, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_grey,
        name="sleeve_collar",
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Box((0.066, 0.046, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=chrome,
        name="mast_tube",
    )
    inner_mast.visual(
        Box((0.072, 0.052, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.361)),
        material=trim_grey,
        name="mast_cap",
    )

    support_head = model.part("support_head")
    support_head.visual(
        Box((0.09, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=frame_paint,
        name="head_block",
    )
    support_head.visual(
        Box((0.16, 0.055, 0.028)),
        origin=Origin(xyz=(0.08, 0.0, 0.018)),
        material=frame_paint,
        name="support_arm",
    )
    support_head.visual(
        Box((0.08, 0.14, 0.032)),
        origin=Origin(xyz=(0.18, -0.015, 0.030)),
        material=frame_paint,
        name="tray_head",
    )
    support_head.visual(
        Cylinder(radius=0.009, length=0.08),
        origin=Origin(xyz=(0.18, 0.0, 0.046), rpy=(0.0, pi * 0.5, 0.0)),
        material=trim_grey,
        name="main_head",
    )
    support_head.visual(
        Box((0.05, 0.08, 0.008)),
        origin=Origin(xyz=(0.18, 0.05, 0.035)),
        material=frame_paint,
        name="wing_bracket",
    )

    main_top = model.part("main_top")
    _add_panel(
        main_top,
        panel_name="top_panel",
        frame_name="top_frame",
        size_x=0.44,
        size_y=0.52,
        thickness=0.018,
        y_direction=-1.0,
        material=top_laminate,
        lip_material=top_edge,
        frame_material=trim_grey,
    )
    main_top.visual(
        Box((0.012, 0.070, 0.020)),
        origin=Origin(xyz=(0.220, -0.180, 0.008)),
        material=trim_grey,
        name="handle_mount",
    )

    reading_wing = model.part("reading_wing")
    _add_panel(
        reading_wing,
        panel_name="wing_panel",
        frame_name="wing_frame",
        size_x=0.30,
        size_y=0.18,
        thickness=0.016,
        y_direction=1.0,
        material=top_laminate,
        lip_material=top_edge,
        frame_material=trim_grey,
        lip_height=0.010,
        lip_thickness=0.010,
    )

    release_handle = model.part("release_handle")
    release_handle.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_plastic,
        name="handle_pivot",
    )
    release_handle.visual(
        Box((0.016, 0.016, 0.020)),
        origin=Origin(xyz=(0.008, 0.0, -0.010)),
        material=dark_plastic,
        name="handle_link",
    )
    release_handle.visual(
        Box((0.064, 0.016, 0.016)),
        origin=Origin(xyz=(0.032, 0.0, -0.018)),
        material=dark_plastic,
        name="handle_arm",
    )
    release_handle.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.066, 0.0, -0.018)),
        material=dark_plastic,
        name="handle_grip",
    )

    for caster_name, _, _ in caster_positions:
        caster = model.part(caster_name)
        _add_caster_wheel(caster, tire_material=dark_plastic, hub_material=caster_hub)

    model.articulation(
        "base_to_sleeve",
        ArticulationType.FIXED,
        parent=base,
        child=sleeve,
        origin=Origin(xyz=(0.07, 0.0, 0.135)),
    )
    model.articulation(
        "mast_height",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.14),
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.FIXED,
        parent=inner_mast,
        child=support_head,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )
    model.articulation(
        "main_tilt",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=main_top,
        origin=Origin(xyz=(0.18, 0.0, 0.055)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=0.0, upper=1.18),
    )
    model.articulation(
        "wing_tilt",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=reading_wing,
        origin=Origin(xyz=(0.18, 0.02, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=1.05),
    )
    model.articulation(
        "handle_pull",
        ArticulationType.REVOLUTE,
        parent=main_top,
        child=release_handle,
        origin=Origin(xyz=(0.235, -0.180, 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.45, upper=0.20),
    )

    for caster_name, caster_x, caster_y in caster_positions:
        model.articulation(
            f"{caster_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster_name,
            origin=Origin(xyz=(caster_x, caster_y, 0.025)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    inner_mast = object_model.get_part("inner_mast")
    support_head = object_model.get_part("support_head")
    main_top = object_model.get_part("main_top")
    reading_wing = object_model.get_part("reading_wing")

    mast_height = object_model.get_articulation("mast_height")
    main_tilt = object_model.get_articulation("main_tilt")
    wing_tilt = object_model.get_articulation("wing_tilt")

    mast_limits = mast_height.motion_limits
    main_limits = main_tilt.motion_limits
    wing_limits = wing_tilt.motion_limits

    if mast_limits is not None and mast_limits.upper is not None:
        with ctx.pose({mast_height: 0.0}):
            ctx.expect_within(
                inner_mast,
                sleeve,
                axes="xy",
                margin=0.010,
                name="inner mast stays centered in the sleeve at low height",
            )
            ctx.expect_overlap(
                inner_mast,
                sleeve,
                axes="z",
                min_overlap=0.20,
                name="inner mast remains well inserted at low height",
            )
            rest_origin = ctx.part_world_position(main_top)

        with ctx.pose({mast_height: mast_limits.upper}):
            ctx.expect_within(
                inner_mast,
                sleeve,
                axes="xy",
                margin=0.010,
                name="inner mast stays centered in the sleeve when raised",
            )
            ctx.expect_overlap(
                inner_mast,
                sleeve,
                axes="z",
                min_overlap=0.08,
                name="inner mast keeps retained insertion at full height",
            )
            raised_origin = ctx.part_world_position(main_top)

        ctx.check(
            "mast raises the split top upward",
            rest_origin is not None
            and raised_origin is not None
            and raised_origin[2] > rest_origin[2] + 0.10,
            details=f"rest={rest_origin}, raised={raised_origin}",
        )

    with ctx.pose({main_tilt: 0.0, wing_tilt: 0.0}):
        ctx.expect_gap(
            reading_wing,
            main_top,
            axis="y",
            positive_elem="wing_panel",
            negative_elem="top_panel",
            min_gap=0.015,
            max_gap=0.030,
            name="reading wing sits just beside the main top",
        )
        ctx.expect_overlap(
            reading_wing,
            main_top,
            axes="x",
            elem_a="wing_panel",
            elem_b="top_panel",
            min_overlap=0.28,
            name="reading wing lines up with the main top front to back",
        )
        ctx.expect_contact(
            reading_wing,
            support_head,
            elem_a="wing_frame",
            elem_b="wing_bracket",
            name="reading wing rests on the shared support bracket",
        )

    if main_limits is not None and main_limits.upper is not None:
        main_rest_aabb = ctx.part_element_world_aabb(main_top, elem="top_panel")
        with ctx.pose({main_tilt: main_limits.upper}):
            main_open_aabb = ctx.part_element_world_aabb(main_top, elem="top_panel")
        ctx.check(
            "main top tilts upward",
            main_rest_aabb is not None
            and main_open_aabb is not None
            and main_open_aabb[1][2] > main_rest_aabb[1][2] + 0.12,
            details=f"rest={main_rest_aabb}, open={main_open_aabb}",
        )

    if wing_limits is not None and wing_limits.upper is not None:
        wing_rest_aabb = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
        with ctx.pose({wing_tilt: wing_limits.upper}):
            wing_open_aabb = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
        ctx.check(
            "reading wing tilts upward",
            wing_rest_aabb is not None
            and wing_open_aabb is not None
            and wing_open_aabb[1][2] > wing_rest_aabb[1][2] + 0.07,
            details=f"rest={wing_rest_aabb}, open={wing_open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
