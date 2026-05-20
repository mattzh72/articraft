from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _lofted_circles(profile: list[tuple[float, float]]) -> cq.Workplane:
    wp = cq.Workplane("XY")
    last_z = 0.0
    for index, (z, radius) in enumerate(profile):
        if index == 0:
            if abs(z) > 1e-9:
                wp = wp.workplane(offset=z)
            wp = wp.circle(radius)
        else:
            wp = wp.workplane(offset=z - last_z).circle(radius)
        last_z = z
    return wp.loft(combine=True)


def _build_base_shell() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.31, 0.24, 0.030).translate((0.0, 0.0, 0.015))
    body = cq.Workplane("XY").box(0.26, 0.205, 0.105).translate((0.0, 0.0, 0.0825))
    shoulder = cq.Workplane("XY").box(0.21, 0.165, 0.045).translate((-0.005, 0.0, 0.1325))
    seat = cq.Workplane("XY").circle(0.074).extrude(0.014).translate((0.0, 0.0, 0.142))
    return foot.union(body).union(shoulder).union(seat)


def _build_bowl_shell() -> cq.Workplane:
    outer = _lofted_circles(
        [
            (0.000, 0.038),
            (0.018, 0.048),
            (0.062, 0.100),
            (0.138, 0.123),
            (0.188, 0.129),
        ]
    )
    inner = _lofted_circles(
        [
            (0.014, 0.022),
            (0.032, 0.034),
            (0.066, 0.087),
            (0.136, 0.114),
            (0.182, 0.120),
        ]
    )
    shell = outer.cut(inner)

    base_ring = cq.Workplane("XY").circle(0.056).extrude(0.018)
    handle_outer = cq.Workplane("XY").box(0.074, 0.036, 0.168).translate((0.138, 0.0, 0.102))
    handle_inner = cq.Workplane("XY").box(0.036, 0.018, 0.106).translate((0.144, 0.0, 0.106))
    handle = handle_outer.cut(handle_inner)
    handle_bridge = cq.Workplane("XY").box(0.028, 0.036, 0.120).translate((0.108, 0.0, 0.094))
    right_hinge_ear = cq.Workplane("XY").box(0.014, 0.026, 0.014).translate((-0.130, 0.060, 0.198))
    left_hinge_ear = cq.Workplane("XY").box(0.014, 0.026, 0.014).translate((-0.130, -0.060, 0.198))

    return shell.union(base_ring).union(handle).union(handle_bridge).union(right_hinge_ear).union(left_hinge_ear)


def _build_lid_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.248, 0.242, 0.026).translate((0.124, 0.0, 0.005))
    inner = cq.Workplane("XY").box(0.220, 0.214, 0.030).translate((0.128, 0.0, 0.003))
    shell = outer.cut(inner)

    chute_opening = cq.Workplane("XY").box(0.070, 0.048, 0.040).translate((0.156, 0.0, 0.010))
    side_catch_right = cq.Workplane("XY").box(0.026, 0.016, 0.014).translate((0.192, 0.115, 0.010))
    side_catch_left = cq.Workplane("XY").box(0.026, 0.016, 0.014).translate((0.192, -0.115, 0.010))

    right_barrel = (
        cq.Workplane("XY")
        .circle(0.008)
        .extrude(0.042)
        .translate((0.0, 0.0, -0.021))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.008, 0.060, 0.000))
    )
    left_barrel = (
        cq.Workplane("XY")
        .circle(0.008)
        .extrude(0.042)
        .translate((0.0, 0.0, -0.021))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.008, -0.060, 0.000))
    )
    right_tab = cq.Workplane("XY").box(0.018, 0.018, 0.014).translate((0.016, 0.060, 0.002))
    left_tab = cq.Workplane("XY").box(0.018, 0.018, 0.014).translate((0.016, -0.060, 0.002))

    return (
        shell.cut(chute_opening)
        .union(side_catch_right)
        .union(side_catch_left)
        .union(right_barrel)
        .union(left_barrel)
        .union(right_tab)
        .union(left_tab)
    )


def _build_chute_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.086, 0.062, 0.118).translate((0.156, 0.0, 0.074))
    inner = cq.Workplane("XY").box(0.068, 0.046, 0.140).translate((0.156, 0.0, 0.074))
    return outer.cut(inner)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deluxe_food_processor")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    band_dark = model.material("band_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    clear_shell = model.material("clear_shell", rgba=(0.82, 0.90, 0.94, 0.38))
    clear_pusher = model.material("clear_pusher", rgba=(0.90, 0.94, 0.96, 0.42))
    latch_finish = model.material("latch_finish", rgba=(0.20, 0.21, 0.22, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.82, 0.84, 0.86, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    button_finish = model.material("button_finish", rgba=(0.26, 0.27, 0.29, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shell(), "food_processor_base_shell"),
        material=body_white,
        name="base_shell",
    )
    base.visual(
        Box((0.026, 0.170, 0.074)),
        origin=Origin(xyz=(0.142, 0.0, 0.074)),
        material=band_dark,
        name="control_band",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=trim_dark,
        name="seat_trim",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_build_bowl_shell(), "food_processor_bowl_shell"),
        material=clear_shell,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.011, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=trim_dark,
        name="spindle_post",
    )
    bowl.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=trim_dark,
        name="spindle_tip",
    )
    bowl.visual(
        Box((0.020, 0.028, 0.020)),
        origin=Origin(xyz=(0.060, 0.102, 0.058)),
        material=trim_dark,
        name="right_latch_pad",
    )
    bowl.visual(
        Box((0.020, 0.028, 0.020)),
        origin=Origin(xyz=(0.060, -0.102, 0.058)),
        material=trim_dark,
        name="left_latch_pad",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "food_processor_lid_shell"),
        material=clear_shell,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_build_chute_shell(), "food_processor_chute_shell"),
        material=clear_shell,
        name="chute_shell",
    )
    lid.visual(
        Box((0.058, 0.168, 0.006)),
        origin=Origin(xyz=(0.225, 0.0, 0.014)),
        material=clear_shell,
        name="front_lip",
    )

    lid_joint = model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(-0.123, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.061, 0.041, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, -0.0575)),
        material=clear_pusher,
        name="pusher_body",
    )
    pusher.visual(
        Box((0.082, 0.060, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=trim_dark,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.040, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=trim_dark,
        name="pusher_grip",
    )
    pusher_joint = model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.156, 0.0, 0.128)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.055,
        ),
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=blade_metal,
        name="hub",
    )
    blade.visual(
        Box((0.138, 0.015, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(0.0, 0.0, 0.30)),
        material=blade_metal,
        name="blade_profile",
    )
    blade.visual(
        Box((0.088, 0.013, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, 0.008), rpy=(0.0, 0.0, -0.52)),
        material=blade_metal,
        name="upper_blade",
    )
    model.articulation(
        "bowl_to_blade",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=30.0),
    )

    for side, name in ((-1.0, "left_latch"), (1.0, "right_latch")):
        latch = model.part(name)
        latch.visual(
            Cylinder(radius=0.011, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=latch_finish,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.018, 0.014, 0.126)),
            origin=Origin(xyz=(0.000, -side * 0.007, 0.068)),
            material=latch_finish,
            name="arm_body",
        )
        latch.visual(
            Box((0.020, 0.038, 0.012)),
            origin=Origin(xyz=(0.006, -side * 0.010, 0.134)),
            material=latch_finish,
            name="hook_tip",
        )
        latch.visual(
            Box((0.020, 0.020, 0.018)),
            origin=Origin(xyz=(0.000, -side * 0.010, 0.016)),
            material=latch_finish,
            name="lower_bridge",
        )
        model.articulation(
            f"bowl_to_{name}",
            ArticulationType.REVOLUTE,
            parent=bowl,
            child=latch,
            origin=Origin(xyz=(0.060, side * 0.135, 0.058)),
            axis=(-side, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=2.0,
                lower=0.0,
                upper=math.radians(58.0),
            ),
        )

    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.026,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.058, 0.006, flare=0.08),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "food_processor_selector_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_finish,
        name="dial_shell",
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.154, 0.0, 0.084)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    for y_pos, name in ((-0.058, "left_rocker"), (0.058, "right_rocker")):
        rocker = model.part(name)
        rocker.visual(
            Box((0.018, 0.034, 0.024)),
            origin=Origin(xyz=(0.010, 0.0, 0.0)),
            material=button_finish,
            name="rocker_cap",
        )
        rocker.visual(
            Box((0.010, 0.022, 0.014)),
            origin=Origin(xyz=(0.004, 0.0, 0.0)),
            material=trim_dark,
            name="rocker_body",
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=rocker,
            origin=Origin(xyz=(0.153, y_pos, 0.060)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=3.0,
                lower=-0.24,
                upper=0.24,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    blade = object_model.get_part("blade")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    left_rocker = object_model.get_part("left_rocker")
    right_rocker = object_model.get_part("right_rocker")

    lid_joint = object_model.get_articulation("bowl_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    left_latch_joint = object_model.get_articulation("bowl_to_left_latch")
    right_latch_joint = object_model.get_articulation("bowl_to_right_latch")
    left_rocker_joint = object_model.get_articulation("base_to_left_rocker")
    right_rocker_joint = object_model.get_articulation("base_to_right_rocker")

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_overlap(
            lid,
            bowl,
            axes="xy",
            min_overlap=0.18,
            name="lid covers the bowl opening when closed",
        )
        ctx.expect_gap(
            lid,
            bowl,
            axis="z",
            min_gap=0.004,
            max_gap=0.030,
            positive_elem="front_lip",
            negative_elem="bowl_shell",
            name="closed lid clears the bowl rim",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    lid_limits = lid_joint.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_joint: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward",
            open_lid_aabb is not None
            and closed_lid_aabb is not None
            and float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.10,
            details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
        )

    pusher_limits = pusher_joint.motion_limits
    rest_pusher_pos = None
    extended_pusher_pos = None
    if pusher_limits is not None and pusher_limits.upper is not None:
        with ctx.pose({pusher_joint: 0.0}):
            ctx.expect_within(
                pusher,
                lid,
                axes="xy",
                inner_elem="pusher_body",
                outer_elem="chute_shell",
                margin=0.006,
                name="pusher stays centered in the chute at rest",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_body",
                elem_b="chute_shell",
                min_overlap=0.070,
                name="pusher remains inserted in the chute at rest",
            )
            rest_pusher_pos = ctx.part_world_position(pusher)

        with ctx.pose({pusher_joint: pusher_limits.upper}):
            ctx.expect_within(
                pusher,
                lid,
                axes="xy",
                inner_elem="pusher_body",
                outer_elem="chute_shell",
                margin=0.006,
                name="pusher stays centered in the chute when pressed",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_body",
                elem_b="chute_shell",
                min_overlap=0.030,
                name="pusher retains insertion at full press",
            )
            extended_pusher_pos = ctx.part_world_position(pusher)

        ctx.check(
            "pusher slides downward",
            rest_pusher_pos is not None
            and extended_pusher_pos is not None
            and float(extended_pusher_pos[2]) < float(rest_pusher_pos[2]) - 0.05,
            details=f"rest={rest_pusher_pos!r}, pressed={extended_pusher_pos!r}",
        )

    ctx.expect_within(
        blade,
        bowl,
        axes="xy",
        inner_elem="blade_profile",
        outer_elem="bowl_shell",
        margin=0.015,
        name="blade stays within the prep cavity footprint",
    )

    left_limits = left_latch_joint.motion_limits
    right_limits = right_latch_joint.motion_limits
    if (
        left_limits is not None
        and left_limits.upper is not None
        and right_limits is not None
        and right_limits.upper is not None
    ):
        with ctx.pose({left_latch_joint: 0.0, right_latch_joint: 0.0}):
            left_closed = _aabb_center(ctx.part_element_world_aabb(left_latch, elem="hook_tip"))
            right_closed = _aabb_center(ctx.part_element_world_aabb(right_latch, elem="hook_tip"))
        with ctx.pose({left_latch_joint: left_limits.upper, right_latch_joint: right_limits.upper}):
            left_open = _aabb_center(ctx.part_element_world_aabb(left_latch, elem="hook_tip"))
            right_open = _aabb_center(ctx.part_element_world_aabb(right_latch, elem="hook_tip"))

        ctx.check(
            "left latch opens outward",
            left_closed is not None and left_open is not None and float(left_open[1]) < float(left_closed[1]) - 0.02,
            details=f"closed={left_closed!r}, open={left_open!r}",
        )
        ctx.check(
            "right latch opens outward",
            right_closed is not None and right_open is not None and float(right_open[1]) > float(right_closed[1]) + 0.02,
            details=f"closed={right_closed!r}, open={right_open!r}",
        )

    left_lower = left_rocker_joint.motion_limits.lower if left_rocker_joint.motion_limits is not None else None
    left_upper = left_rocker_joint.motion_limits.upper if left_rocker_joint.motion_limits is not None else None
    right_lower = right_rocker_joint.motion_limits.lower if right_rocker_joint.motion_limits is not None else None
    right_upper = right_rocker_joint.motion_limits.upper if right_rocker_joint.motion_limits is not None else None

    if left_lower is not None and left_upper is not None:
        with ctx.pose({left_rocker_joint: left_lower}):
            left_low = ctx.part_element_world_aabb(left_rocker, elem="rocker_cap")
        with ctx.pose({left_rocker_joint: left_upper}):
            left_high = ctx.part_element_world_aabb(left_rocker, elem="rocker_cap")
        ctx.check(
            "left rocker tilts",
            left_low is not None and left_high is not None and abs(float(left_high[1][2]) - float(left_low[1][2])) > 0.003,
            details=f"low={left_low!r}, high={left_high!r}",
        )

    if right_lower is not None and right_upper is not None:
        with ctx.pose({right_rocker_joint: right_lower}):
            right_low = ctx.part_element_world_aabb(right_rocker, elem="rocker_cap")
        with ctx.pose({right_rocker_joint: right_upper}):
            right_high = ctx.part_element_world_aabb(right_rocker, elem="rocker_cap")
        ctx.check(
            "right rocker tilts",
            right_low is not None and right_high is not None and abs(float(right_high[1][2]) - float(right_low[1][2])) > 0.003,
            details=f"low={right_low!r}, high={right_high!r}",
        )

    return ctx.report()


object_model = build_object_model()
