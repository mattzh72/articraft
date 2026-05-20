from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(width: float, depth: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, depth, radius), thickness),
        name,
    )


def _add_caster_mount(part, *, bracket_material, wheel_material) -> None:
    part.visual(
        Box((0.022, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=bracket_material,
        name="crown",
    )
    for y_pos, elem_name in ((-0.0145, "fork_0"), (0.0145, "fork_1")):
        part.visual(
            Box((0.020, 0.007, 0.036)),
            origin=Origin(xyz=(0.0, y_pos, 0.011)),
            material=bracket_material,
            name=elem_name,
        )
    for y_pos, elem_name in ((-0.013, "boss_0"), (0.013, "boss_1")):
        part.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bracket_material,
            name=elem_name,
        )


def _add_caster_wheel(part, *, wheel_material, hub_material) -> None:
    part.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name="wheel_tire",
    )
    for y_pos, elem_name in ((-0.009, "hub_0"), (0.009, "hub_1")):
        part.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_material,
            name=elem_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehab_overbed_table")

    frame = model.material("frame", rgba=(0.78, 0.79, 0.81, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    laminate = model.material("laminate", rgba=(0.88, 0.84, 0.75, 1.0))
    edge_trim = model.material("edge_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    main_panel_mesh = _rounded_panel_mesh(0.588, 0.420, 0.018, 0.028, "main_top_panel")
    wing_panel_mesh = _rounded_panel_mesh(0.180, 0.420, 0.018, 0.024, "side_wing_panel")

    base = model.part("base")
    base.visual(
        Box((0.060, 0.560, 0.024)),
        origin=Origin(xyz=(0.030, 0.000, 0.071)),
        material=frame,
        name="rear_beam",
    )
    for y_pos, elem_name in ((-0.250, "leg_0"), (0.250, "leg_1")):
        base.visual(
            Box((0.640, 0.060, 0.024)),
            origin=Origin(xyz=(0.380, y_pos, 0.071)),
            material=frame,
            name=elem_name,
        )
    base.visual(
        Box((0.085, 0.085, 0.028)),
        origin=Origin(xyz=(0.095, -0.205, 0.097)),
        material=frame,
        name="mast_socket",
    )
    base.visual(
        Box((0.065, 0.0055, 0.550)),
        origin=Origin(xyz=(0.110, -0.22475, 0.358)),
        material=chrome,
        name="mast_outer_0",
    )
    base.visual(
        Box((0.065, 0.0055, 0.550)),
        origin=Origin(xyz=(0.110, -0.18525, 0.358)),
        material=chrome,
        name="mast_outer_1",
    )
    base.visual(
        Box((0.0055, 0.034, 0.550)),
        origin=Origin(xyz=(0.08025, -0.205, 0.358)),
        material=chrome,
        name="mast_outer_2",
    )
    base.visual(
        Box((0.0055, 0.034, 0.550)),
        origin=Origin(xyz=(0.13975, -0.205, 0.358)),
        material=chrome,
        name="mast_outer_3",
    )
    base.visual(
        Box((0.090, 0.020, 0.050)),
        origin=Origin(xyz=(0.085, -0.172, 0.118)),
        material=frame,
        name="mast_gusset",
    )

    caster_positions = (
        (0.050, -0.250),
        (0.650, -0.250),
        (0.050, 0.250),
        (0.650, 0.250),
    )
    for index, (x_pos, y_pos) in enumerate(caster_positions):
        mount = model.part(f"caster_mount_{index}")
        _add_caster_mount(mount, bracket_material=steel_dark, wheel_material=chrome)
        model.articulation(
            f"base_to_caster_mount_{index}",
            ArticulationType.FIXED,
            parent=base,
            child=mount,
            origin=Origin(xyz=(x_pos, y_pos, 0.025)),
        )

        wheel = model.part(f"caster_{index}")
        _add_caster_wheel(wheel, wheel_material=caster_rubber, hub_material=steel_dark)
        model.articulation(
            f"caster_mount_{index}_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=mount,
            child=wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

    mast_inner = model.part("mast_inner")
    mast_inner.visual(
        Box((0.054, 0.034, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=chrome,
        name="inner_tube",
    )
    base_to_mast = model.articulation(
        "base_to_mast_inner",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_inner,
        origin=Origin(xyz=(0.110, -0.205, 0.633)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.220),
    )

    support_head = model.part("support_head")
    support_head.visual(
        Box((0.076, 0.056, 0.044)),
        origin=Origin(xyz=(0.0, -0.220, -0.020)),
        material=frame,
        name="mast_cap_block",
    )
    support_head.visual(
        Box((0.100, 0.220, 0.022)),
        origin=Origin(xyz=(-0.060, -0.110, -0.010)),
        material=frame,
        name="support_arm",
    )
    support_head.visual(
        Box((0.220, 0.100, 0.022)),
        origin=Origin(xyz=(-0.090, 0.000, -0.010)),
        material=frame,
        name="head_plate",
    )
    support_head.visual(
        Box((0.018, 0.220, 0.070)),
        origin=Origin(xyz=(-0.035, -0.110, -0.040)),
        material=steel_dark,
        name="web",
    )
    support_head.visual(
        Cylinder(radius=0.007, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="hinge_knuckle",
    )
    model.articulation(
        "mast_inner_to_support_head",
        ArticulationType.FIXED,
        parent=mast_inner,
        child=support_head,
        origin=Origin(xyz=(0.0, 0.205, 0.392)),
    )

    side_wing = model.part("side_wing")
    side_wing.visual(
        wing_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=laminate,
        name="wing_panel",
    )
    model.articulation(
        "support_head_to_side_wing",
        ArticulationType.FIXED,
        parent=support_head,
        child=side_wing,
        origin=Origin(xyz=(-0.097, 0.0, 0.0)),
    )

    main_top = model.part("main_top")
    main_top.visual(
        main_panel_mesh,
        origin=Origin(xyz=(0.304, 0.0, 0.010)),
        material=laminate,
        name="tray_panel",
    )
    main_top.visual(
        Box((0.260, 0.080, 0.022)),
        origin=Origin(xyz=(0.180, 0.0, -0.010)),
        material=edge_trim,
        name="tray_rib",
    )
    for y_pos, elem_name in ((-0.132, "hinge_leaf_0"), (0.132, "hinge_leaf_1")):
        main_top.visual(
            Box((0.016, 0.070, 0.012)),
            origin=Origin(xyz=(0.008, y_pos, 0.006)),
            material=steel_dark,
            name=elem_name,
        )
    for y_pos, elem_name in ((-0.132, "hinge_knuckle_0"), (0.132, "hinge_knuckle_1")):
        main_top.visual(
            Cylinder(radius=0.0065, length=0.070),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
            name=elem_name,
        )
    main_top.visual(
        Cylinder(radius=0.004, length=0.100),
        origin=Origin(xyz=(0.594, 0.0, 0.019), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="clip_knuckle",
    )
    support_head_to_main_top = model.articulation(
        "support_head_to_main_top",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=main_top,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    tilt_handle = model.part("tilt_handle")
    tilt_handle.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="grip",
    )
    tilt_handle.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="stub",
    )
    model.articulation(
        "support_head_to_tilt_handle",
        ArticulationType.FIXED,
        parent=support_head,
        child=tilt_handle,
        origin=Origin(xyz=(0.020, -0.135, -0.006)),
    )

    clip_bar = model.part("clip_bar")
    for y_pos, elem_name in ((-0.132, "clip_knuckle_0"), (0.132, "clip_knuckle_1")):
        clip_bar.visual(
            Cylinder(radius=0.0038, length=0.072),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_dark,
            name=elem_name,
        )
    for y_pos, elem_name in ((-0.132, "support_0"), (0.132, "support_1")):
        clip_bar.visual(
            Box((0.024, 0.030, 0.006)),
            origin=Origin(xyz=(-0.012, y_pos, 0.005)),
            material=steel_dark,
            name=elem_name,
        )
    clip_bar.visual(
        Cylinder(radius=0.0045, length=0.260),
        origin=Origin(xyz=(-0.026, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="bar_rod",
    )
    main_top_to_clip_bar = model.articulation(
        "main_top_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=main_top,
        child=clip_bar,
        origin=Origin(xyz=(0.594, 0.0, 0.019)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast_inner = object_model.get_articulation("base_to_mast_inner")
    top_hinge = object_model.get_articulation("support_head_to_main_top")
    clip_hinge = object_model.get_articulation("main_top_to_clip_bar")

    support_head = object_model.get_part("support_head")
    side_wing = object_model.get_part("side_wing")
    main_top = object_model.get_part("main_top")
    clip_bar = object_model.get_part("clip_bar")

    ctx.expect_contact(
        side_wing,
        support_head,
        elem_a="wing_panel",
        elem_b="head_plate",
        name="side wing sits on the shared support head",
    )
    ctx.expect_gap(
        main_top,
        side_wing,
        axis="x",
        positive_elem="tray_panel",
        negative_elem="wing_panel",
        min_gap=0.0,
        max_gap=0.018,
        name="split top seam stays narrow",
    )
    wing_aabb = ctx.part_element_world_aabb(side_wing, elem="wing_panel")
    tray_aabb = ctx.part_element_world_aabb(main_top, elem="tray_panel")
    tops_level = (
        wing_aabb is not None
        and tray_aabb is not None
        and abs(wing_aabb[1][2] - tray_aabb[1][2]) <= 0.0015
    )
    ctx.check(
        "split tops stay level when closed",
        tops_level,
        details=f"wing_aabb={wing_aabb}, tray_aabb={tray_aabb}",
    )

    rest_head_pos = ctx.part_world_position(support_head)
    mast_upper = mast_inner.motion_limits.upper if mast_inner.motion_limits is not None else None
    if mast_upper is not None:
        with ctx.pose({mast_inner: mast_upper}):
            raised_head_pos = ctx.part_world_position(support_head)
        ctx.check(
            "mast raises the support head",
            rest_head_pos is not None
            and raised_head_pos is not None
            and raised_head_pos[2] > rest_head_pos[2] + 0.18,
            details=f"rest={rest_head_pos}, raised={raised_head_pos}",
        )

    closed_tray = ctx.part_element_world_aabb(main_top, elem="tray_panel")
    top_upper = top_hinge.motion_limits.upper if top_hinge.motion_limits is not None else None
    if top_upper is not None:
        with ctx.pose({top_hinge: top_upper}):
            tilted_tray = ctx.part_element_world_aabb(main_top, elem="tray_panel")
        ctx.check(
            "main top tilts upward",
            closed_tray is not None
            and tilted_tray is not None
            and tilted_tray[1][2] > closed_tray[1][2] + 0.16,
            details=f"closed={closed_tray}, tilted={tilted_tray}",
        )

    ctx.expect_gap(
        clip_bar,
        main_top,
        axis="z",
        positive_elem="bar_rod",
        negative_elem="tray_panel",
        min_gap=0.004,
        max_gap=0.030,
        name="clip bar stands proud of the tray when closed",
    )
    clip_upper = clip_hinge.motion_limits.upper if clip_hinge.motion_limits is not None else None
    if clip_upper is not None:
        with ctx.pose({clip_hinge: clip_upper}):
            ctx.expect_gap(
                clip_bar,
                main_top,
                axis="z",
                positive_elem="bar_rod",
                negative_elem="tray_panel",
                min_gap=0.020,
                name="clip bar flips up above the tray",
            )

    return ctx.report()


object_model = build_object_model()
