from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def rect_profile(width: float, height: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def build_handwheel_geometry() -> TorusGeometry:
    wheel = TorusGeometry(radius=0.038, tube=0.0045, radial_segments=18, tubular_segments=40).rotate_x(math.pi / 2.0)
    wheel.merge(CylinderGeometry(radius=0.010, height=0.018, radial_segments=24).rotate_x(math.pi / 2.0))

    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        spoke = BoxGeometry((0.072, 0.008, 0.004)).rotate_y(angle)
        wheel.merge(spoke)

    wheel.merge(
        CylinderGeometry(radius=0.0055, height=0.016, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.038)
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_drill_stand")

    matte_black = model.material("matte_black", rgba=(0.15, 0.16, 0.17, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.43, 0.46, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_black = model.material("knob_black", rgba=(0.05, 0.05, 0.05, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.180, 0.120, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=matte_black,
        name="base_shell",
    )
    stand.visual(
        Box((0.145, 0.092, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark_steel,
        name="base_cap",
    )
    stand.visual(
        Box((0.080, 0.070, 0.036)),
        origin=Origin(xyz=(-0.030, 0.0, 0.074)),
        material=machine_gray,
        name="column_pedestal",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.420),
        origin=Origin(xyz=(-0.035, 0.0, 0.302)),
        material=brushed_steel,
        name="column",
    )
    stand.visual(
        Cylinder(radius=0.027, length=0.028),
        origin=Origin(xyz=(-0.035, 0.0, 0.526)),
        material=dark_steel,
        name="column_cap",
    )
    stand.visual(
        Box((0.014, 0.030, 0.280)),
        origin=Origin(xyz=(-0.002, 0.0, 0.270)),
        material=dark_steel,
        name="rack_bar",
    )
    tooth_pitch = 0.016
    for index in range(15):
        stand.visual(
            Box((0.008, 0.028, 0.008)),
            origin=Origin(xyz=(0.006, 0.0, 0.150 + index * tooth_pitch)),
            material=dark_steel,
            name=f"rack_tooth_{index}",
        )

    stand.visual(
        Box((0.034, 0.070, 0.044)),
        origin=Origin(xyz=(-0.020, 0.0, 0.112)),
        material=machine_gray,
        name="table_clamp",
    )
    stand.visual(
        Box((0.044, 0.050, 0.020)),
        origin=Origin(xyz=(0.008, 0.0, 0.112)),
        material=machine_gray,
        name="table_arm",
    )
    stand.visual(
        Box((0.016, 0.076, 0.028)),
        origin=Origin(xyz=(0.020, 0.0, 0.118)),
        material=machine_gray,
        name="table_support",
    )
    for side_y, suffix in ((0.038, "0"), (-0.038, "1")):
        stand.visual(
            Box((0.016, 0.016, 0.050)),
            origin=Origin(xyz=(0.036, side_y, 0.132)),
            material=machine_gray,
            name=f"table_cheek_{suffix}",
        )

    table = model.part("table")
    table_plate = ExtrudeWithHolesGeometry(
        rect_profile(0.160, 0.120),
        [rect_profile(0.082, 0.022, cx=0.016)],
        0.012,
    )
    table.visual(
        mesh_from_geometry(table_plate, "tilting_table_plate"),
        origin=Origin(xyz=(0.085, 0.0, 0.024)),
        material=dark_steel,
        name="table_plate",
    )
    table.visual(
        Box((0.032, 0.056, 0.024)),
        origin=Origin(xyz=(0.016, 0.0, 0.000)),
        material=machine_gray,
        name="table_mount",
    )
    table.visual(
        Cylinder(radius=0.008, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    table.visual(
        Box((0.086, 0.028, 0.024)),
        origin=Origin(xyz=(0.050, 0.0, 0.006)),
        material=machine_gray,
        name="center_rib",
    )
    table.visual(
        Box((0.080, 0.014, 0.022)),
        origin=Origin(xyz=(0.070, 0.040, 0.008)),
        material=machine_gray,
        name="side_rib_0",
    )
    table.visual(
        Box((0.080, 0.014, 0.022)),
        origin=Origin(xyz=(0.070, -0.040, 0.008)),
        material=machine_gray,
        name="side_rib_1",
    )
    table.visual(
        Box((0.160, 0.010, 0.010)),
        origin=Origin(xyz=(0.085, 0.0, 0.035)),
        material=machine_gray,
        name="front_fence",
    )

    head = model.part("head")
    head.visual(
        Box((0.048, 0.022, 0.110)),
        origin=Origin(xyz=(0.022, 0.041, 0.0)),
        material=machine_gray,
        name="left_guide",
    )
    head.visual(
        Box((0.048, 0.022, 0.110)),
        origin=Origin(xyz=(0.022, -0.041, 0.0)),
        material=machine_gray,
        name="right_guide",
    )
    head.visual(
        Box((0.058, 0.110, 0.095)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=machine_gray,
        name="carriage_body",
    )
    head.visual(
        Box((0.050, 0.062, 0.030)),
        origin=Origin(xyz=(0.068, 0.0, 0.060)),
        material=dark_steel,
        name="motor_cap",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.060, 0.0, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="feed_housing",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.060, 0.063, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="shaft_stub",
    )
    head.visual(
        Box((0.028, 0.060, 0.058)),
        origin=Origin(xyz=(0.109, 0.0, -0.010)),
        material=dark_steel,
        name="spindle_mount",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(0.126, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="spindle_nose",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(xyz=(0.162, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="chuck",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_geometry(build_handwheel_geometry(), "drill_stand_handwheel"),
        material=handle_black,
        name="wheel",
    )

    head_slide = model.articulation(
        "stand_to_head",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=head,
        origin=Origin(xyz=(-0.035, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=-0.060,
            upper=0.100,
        ),
    )

    table_tilt = model.articulation(
        "stand_to_table",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=table,
        origin=Origin(xyz=(0.036, 0.0, 0.132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.80,
        ),
    )

    model.articulation(
        "head_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=handwheel,
        origin=Origin(xyz=(0.060, 0.075, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )

    head_slide.meta["qc_note"] = "The carriage slides along the column via the rack-and-pinion feed."
    table_tilt.meta["qc_note"] = "Positive tilt lifts the table front edge."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    table = object_model.get_part("table")
    head = object_model.get_part("head")
    handwheel = object_model.get_part("handwheel")

    head_slide = object_model.get_articulation("stand_to_head")
    table_tilt = object_model.get_articulation("stand_to_table")
    handwheel_spin = object_model.get_articulation("head_to_handwheel")

    ctx.expect_gap(
        table,
        stand,
        axis="z",
        positive_elem="table_plate",
        negative_elem="base_shell",
        min_gap=0.100,
        name="table sits well above the magnetic base",
    )
    ctx.expect_overlap(
        head,
        table,
        axes="xy",
        elem_a="spindle_nose",
        elem_b="table_plate",
        min_overlap=0.020,
        name="spindle is positioned above the tilting table",
    )

    lower_feed = head_slide.motion_limits.lower
    upper_feed = head_slide.motion_limits.upper
    if lower_feed is not None:
        with ctx.pose({head_slide: lower_feed}):
            ctx.expect_gap(
                head,
                table,
                axis="z",
                positive_elem="carriage_body",
                negative_elem="table_plate",
                min_gap=0.008,
                name="lowest head feed still clears the table",
            )
            low_head_pos = ctx.part_world_position(head)
    else:
        low_head_pos = None

    if upper_feed is not None:
        with ctx.pose({head_slide: upper_feed}):
            high_head_pos = ctx.part_world_position(head)
    else:
        high_head_pos = None

    ctx.check(
        "head carriage travels upward along the column",
        low_head_pos is not None
        and high_head_pos is not None
        and high_head_pos[2] > low_head_pos[2] + 0.12,
        details=f"low={low_head_pos}, high={high_head_pos}",
    )

    rest_table_aabb = ctx.part_element_world_aabb(table, elem="table_plate")
    table_upper = table_tilt.motion_limits.upper
    if table_upper is not None:
        with ctx.pose({table_tilt: table_upper}):
            tilted_table_aabb = ctx.part_element_world_aabb(table, elem="table_plate")
    else:
        tilted_table_aabb = None

    ctx.check(
        "table tilt raises the front edge",
        rest_table_aabb is not None
        and tilted_table_aabb is not None
        and tilted_table_aabb[1][2] > rest_table_aabb[1][2] + 0.05,
        details=f"rest={rest_table_aabb}, tilted={tilted_table_aabb}",
    )

    rest_wheel_pos = ctx.part_world_position(handwheel)
    with ctx.pose({handwheel_spin: math.pi / 2.0}):
        spun_wheel_pos = ctx.part_world_position(handwheel)

    ctx.check(
        "handwheel rotates on a fixed side shaft",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and max(abs(a - b) for a, b in zip(rest_wheel_pos, spun_wheel_pos)) < 1e-7,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
