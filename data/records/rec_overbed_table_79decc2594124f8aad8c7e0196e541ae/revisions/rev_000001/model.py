from __future__ import annotations

import math

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


CASTER_SPECS = (
    ("front_outer", -0.287, 0.3275, "outer_rail"),
    ("rear_outer", -0.287, -0.3275, "outer_rail"),
    ("front_inner", 0.287, 0.3275, "front_leg"),
    ("rear_inner", 0.287, -0.3275, "rear_leg"),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehabilitation_overbed_table")

    frame_paint = model.material("frame_paint", rgba=(0.78, 0.80, 0.82, 1.0))
    mast_paint = model.material("mast_paint", rgba=(0.72, 0.74, 0.76, 1.0))
    head_paint = model.material("head_paint", rgba=(0.63, 0.65, 0.68, 1.0))
    laminate = model.material("laminate", rgba=(0.84, 0.77, 0.63, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.31, 0.28, 0.24, 1.0))
    plastic = model.material("plastic", rgba=(0.17, 0.18, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    base = model.part("base")
    rail_z = 0.105
    rail_h = 0.030
    outer_rail_x = -0.2975
    leg_y = 0.3275
    base.visual(
        Box((0.065, 0.720, rail_h)),
        origin=Origin(xyz=(outer_rail_x, 0.0, rail_z)),
        material=frame_paint,
        name="outer_rail",
    )
    base.visual(
        Box((0.595, 0.065, rail_h)),
        origin=Origin(xyz=(0.0325, leg_y, rail_z)),
        material=frame_paint,
        name="front_leg",
    )
    base.visual(
        Box((0.595, 0.065, rail_h)),
        origin=Origin(xyz=(0.0325, -leg_y, rail_z)),
        material=frame_paint,
        name="rear_leg",
    )
    base.visual(
        Box((0.045, 0.145, 0.070)),
        origin=Origin(xyz=(-0.315, 0.0, 0.155)),
        material=head_paint,
        name="mast_socket",
    )

    sleeve_center = (-0.252, 0.0, 0.325)
    sleeve_height = 0.410
    sleeve_x = 0.074
    sleeve_y = 0.094
    sleeve_wall = 0.005
    base.visual(
        Box((sleeve_x, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0], sleeve_center[1] + sleeve_y * 0.5 - sleeve_wall * 0.5, sleeve_center[2])),
        material=mast_paint,
        name="sleeve_front",
    )
    base.visual(
        Box((sleeve_x, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0], sleeve_center[1] - sleeve_y * 0.5 + sleeve_wall * 0.5, sleeve_center[2])),
        material=mast_paint,
        name="sleeve_rear",
    )
    base.visual(
        Box((sleeve_wall, sleeve_y - 2.0 * sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0] + sleeve_x * 0.5 - sleeve_wall * 0.5, sleeve_center[1], sleeve_center[2])),
        material=mast_paint,
        name="sleeve_inner_wall",
    )
    base.visual(
        Box((sleeve_wall, sleeve_y - 2.0 * sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0] - sleeve_x * 0.5 + sleeve_wall * 0.5, sleeve_center[1], sleeve_center[2])),
        material=mast_paint,
        name="sleeve_outer_wall",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.052, 0.084, 0.740)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mast_paint,
        name="mast_column",
    )
    mast.visual(
        Box((0.044, 0.064, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=head_paint,
        name="mast_cap",
    )

    support_head = model.part("support_head")
    support_head.visual(
        Box((0.100, 0.160, 0.070)),
        origin=Origin(xyz=(-0.100, 0.0, -0.050)),
        material=head_paint,
        name="head_body",
    )
    support_head.visual(
        Box((0.050, 0.082, 0.140)),
        origin=Origin(xyz=(-0.137, 0.0, -0.090)),
        material=head_paint,
        name="head_neck",
    )
    support_head.visual(
        Box((0.120, 0.080, 0.020)),
        origin=Origin(xyz=(-0.080, 0.0, -0.020)),
        material=head_paint,
        name="head_arm",
    )
    support_head.visual(
        Box((0.160, 0.440, 0.028)),
        origin=Origin(xyz=(-0.080, 0.0, 0.014)),
        material=laminate,
        name="wing_panel",
    )
    support_head.visual(
        Box((0.118, 0.160, 0.014)),
        origin=Origin(xyz=(-0.075, 0.0, -0.007)),
        material=dark_trim,
        name="wing_support",
    )
    support_head.visual(
        Box((0.016, 0.440, 0.040)),
        origin=Origin(xyz=(-0.028, 0.0, -0.006)),
        material=head_paint,
        name="hinge_block",
    )
    support_head.visual(
        Box((0.032, 0.050, 0.020)),
        origin=Origin(xyz=(-0.048, 0.080, -0.042)),
        material=head_paint,
        name="handle_mount",
    )

    main_top = model.part("main_top")
    main_top.visual(
        Box((0.680, 0.440, 0.028)),
        origin=Origin(xyz=(0.340, 0.0, 0.014)),
        material=laminate,
        name="main_panel",
    )
    main_top.visual(
        Box((0.170, 0.180, 0.018)),
        origin=Origin(xyz=(0.085, 0.0, -0.009)),
        material=dark_trim,
        name="hinge_rib",
    )
    main_top.visual(
        Cylinder(radius=0.016, length=0.180),
        origin=Origin(xyz=(0.000, 0.0, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=head_paint,
        name="hinge_barrel",
    )

    tilt_handle = model.part("tilt_handle")
    tilt_handle.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic,
        name="handle_hub",
    )
    tilt_handle.visual(
        Box((0.018, 0.108, 0.020)),
        origin=Origin(xyz=(0.0, 0.054, -0.010)),
        material=plastic,
        name="handle_lever",
    )
    tilt_handle.visual(
        Box((0.026, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, 0.108, -0.012)),
        material=rubber,
        name="handle_grip",
    )

    for prefix, caster_x, caster_y, _base_elem in CASTER_SPECS:
        fork = model.part(f"{prefix}_fork")
        fork.visual(
            Box((0.044, 0.006, 0.062)),
            origin=Origin(xyz=(0.0, 0.015, 0.022)),
            material=head_paint,
            name="left_plate",
        )
        fork.visual(
            Box((0.044, 0.006, 0.062)),
            origin=Origin(xyz=(0.0, -0.015, 0.022)),
            material=head_paint,
            name="right_plate",
        )
        fork.visual(
            Box((0.044, 0.034, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.048)),
            material=head_paint,
            name="crown",
        )
        fork.visual(
            Box((0.024, 0.024, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.056)),
            material=head_paint,
            name="stem_block",
        )
        fork.visual(
            Box((0.012, 0.006, 0.012)),
            origin=Origin(xyz=(0.028, 0.012, 0.038)),
            material=head_paint,
            name="lock_lug_0",
        )
        fork.visual(
            Box((0.012, 0.006, 0.012)),
            origin=Origin(xyz=(0.028, -0.012, 0.038)),
            material=head_paint,
            name="lock_lug_1",
        )

        wheel = model.part(f"{prefix}_wheel")
        wheel.visual(
            Cylinder(radius=0.030, length=0.024),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.015, length=0.030),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=head_paint,
            name="hub",
        )
        wheel.visual(
            Box((0.026, 0.010, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=head_paint,
            name="hub_key",
        )

        lock_tab = model.part(f"{prefix}_lock_tab")
        lock_tab.visual(
            Cylinder(radius=0.005, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=plastic,
            name="pivot_hub",
        )
        lock_tab.visual(
            Box((0.018, 0.016, 0.016)),
            origin=Origin(xyz=(0.006, 0.0, 0.008)),
            material=plastic,
            name="cam",
        )
        lock_tab.visual(
            Box((0.032, 0.016, 0.010)),
            origin=Origin(xyz=(0.024, 0.0, 0.018)),
            material=plastic,
            name="pedal",
        )

        model.articulation(
            f"base_to_{prefix}_fork",
            ArticulationType.FIXED,
            parent=base,
            child=fork,
            origin=Origin(xyz=(caster_x, caster_y, 0.030)),
        )
        model.articulation(
            f"{prefix}_fork_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=25.0,
            ),
        )
        model.articulation(
            f"{prefix}_fork_to_lock_tab",
            ArticulationType.REVOLUTE,
            parent=fork,
            child=lock_tab,
            origin=Origin(xyz=(0.028, 0.0, 0.038)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=3.0,
                lower=-0.550,
                upper=0.350,
            ),
        )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(-0.252, 0.0, 0.530)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=0.0,
            upper=0.220,
        ),
    )
    model.articulation(
        "mast_to_support_head",
        ArticulationType.FIXED,
        parent=mast,
        child=support_head,
        origin=Origin(xyz=(0.188, 0.0, 0.318)),
    )
    model.articulation(
        "support_head_to_main_top",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=main_top,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=0.950,
        ),
    )
    model.articulation(
        "support_head_to_tilt_handle",
        ArticulationType.REVOLUTE,
        parent=support_head,
        child=tilt_handle,
        origin=Origin(xyz=(-0.048, 0.115, -0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-0.550,
            upper=0.350,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    support_head = object_model.get_part("support_head")
    main_top = object_model.get_part("main_top")
    mast_slide = object_model.get_articulation("base_to_mast")
    top_hinge = object_model.get_articulation("support_head_to_main_top")

    with ctx.pose({top_hinge: 0.0}):
        ctx.expect_gap(
            main_top,
            support_head,
            axis="x",
            positive_elem="main_panel",
            negative_elem="wing_panel",
            max_gap=0.002,
            max_penetration=0.0,
            name="main panel meets the fixed wing at the split seam",
        )
        ctx.expect_overlap(
            main_top,
            support_head,
            axes="y",
            elem_a="main_panel",
            elem_b="wing_panel",
            min_overlap=0.420,
            name="main panel and wing share the same support span",
        )

    mast_rest = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: mast_slide.motion_limits.upper}):
        mast_high = ctx.part_world_position(mast)
    ctx.check(
        "mast raises the table head upward",
        mast_rest is not None and mast_high is not None and mast_high[2] > mast_rest[2] + 0.18,
        details=f"rest={mast_rest}, raised={mast_high}",
    )

    closed_panel = ctx.part_element_world_aabb(main_top, elem="main_panel")
    wing_panel = ctx.part_element_world_aabb(support_head, elem="wing_panel")
    with ctx.pose({top_hinge: top_hinge.motion_limits.upper}):
        tilted_panel = ctx.part_element_world_aabb(main_top, elem="main_panel")
    ctx.check(
        "main panel tilts above the fixed wing",
        closed_panel is not None
        and tilted_panel is not None
        and wing_panel is not None
        and tilted_panel[1][2] > wing_panel[1][2] + 0.16
        and tilted_panel[1][2] > closed_panel[1][2] + 0.18,
        details=f"closed={closed_panel}, wing={wing_panel}, tilted={tilted_panel}",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3)) if aabb is not None else None

    for prefix, _caster_x, _caster_y, base_elem in CASTER_SPECS:
        fork = object_model.get_part(f"{prefix}_fork")
        wheel = object_model.get_part(f"{prefix}_wheel")
        lock_tab = object_model.get_part(f"{prefix}_lock_tab")
        wheel_spin = object_model.get_articulation(f"{prefix}_fork_to_wheel")
        lock_joint = object_model.get_articulation(f"{prefix}_fork_to_lock_tab")

        ctx.expect_contact(
            fork,
            "base",
            elem_a="stem_block",
            elem_b=base_elem,
            name=f"{prefix} caster fork mounts to the base frame",
        )
        ctx.expect_contact(
            wheel,
            fork,
            elem_a="tire",
            elem_b="left_plate",
            name=f"{prefix} wheel stays captured inside the visible fork",
        )

        rest_key = _aabb_center(ctx.part_element_world_aabb(wheel, elem="hub_key"))
        with ctx.pose({wheel_spin: 1.1}):
            spun_key = _aabb_center(ctx.part_element_world_aabb(wheel, elem="hub_key"))
        ctx.check(
            f"{prefix} wheel spins about its axle",
            rest_key is not None
            and spun_key is not None
            and (
                abs(spun_key[0] - rest_key[0]) > 0.010
                or abs(spun_key[2] - rest_key[2]) > 0.010
            ),
            details=f"rest_key={rest_key}, spun_key={spun_key}",
        )

        rest_pedal = _aabb_center(ctx.part_element_world_aabb(lock_tab, elem="pedal"))
        with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
            moved_pedal = _aabb_center(ctx.part_element_world_aabb(lock_tab, elem="pedal"))
        ctx.check(
            f"{prefix} lock tab rotates on its local pivot",
            rest_pedal is not None
            and moved_pedal is not None
            and (
                abs(moved_pedal[0] - rest_pedal[0]) > 0.006
                or abs(moved_pedal[2] - rest_pedal[2]) > 0.006
            ),
            details=f"rest_pedal={rest_pedal}, moved_pedal={moved_pedal}",
        )

    return ctx.report()


object_model = build_object_model()
