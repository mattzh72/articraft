from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CROSSBEAM_Z = 2.20
TOP_PIVOT_Z = 2.08
HANGER_SPACING = 0.46
HANGER_X = HANGER_SPACING / 2.0
HANGER_LENGTH = 1.34


def _rod_origin_between(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[Origin, float]:
    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return Origin(), 0.0
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((ax + bx) / 2.0, (ay + by) / 2.0, (az + bz) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_rod(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material: Material | str,
    name: str,
) -> None:
    origin, length = _rod_origin_between(a, b)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _ellipse_profile(
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    *,
    segments: int = 32,
) -> list[tuple[float, float]]:
    return [
        (cx + rx * cos(2.0 * pi * i / segments), cy + ry * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _front_panel_mesh():
    outer = rounded_rect_profile(0.46, 0.24, 0.035, corner_segments=8)
    holes = [
        _ellipse_profile(-0.085, -0.025, 0.058, 0.074),
        _ellipse_profile(0.085, -0.025, 0.058, 0.074),
    ]
    panel = ExtrudeWithHolesGeometry(outer, holes, 0.035, center=True)
    panel.rotate_x(pi / 2.0).translate(0.23, 0.205, -0.19)
    return mesh_from_geometry(panel, "front_leg_hole_panel")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toddler_bucket_swing")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    red_plastic = model.material("molded_red_plastic", rgba=(0.90, 0.08, 0.04, 1.0))
    yellow_plastic = model.material("safety_yellow_plastic", rgba=(0.98, 0.74, 0.08, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    frame = model.part("frame")
    _add_rod(
        frame,
        (-1.18, 0.0, CROSSBEAM_Z),
        (1.18, 0.0, CROSSBEAM_Z),
        radius=0.045,
        material=dark_steel,
        name="top_crossbeam",
    )
    for x in (-1.06, 1.06):
        _add_rod(
            frame,
            (x, -0.03, CROSSBEAM_Z - 0.015),
            (x, -0.68, 0.045),
            radius=0.034,
            material=dark_steel,
            name=f"rear_leg_{0 if x < 0 else 1}",
        )
        _add_rod(
            frame,
            (x, 0.03, CROSSBEAM_Z - 0.015),
            (x, 0.68, 0.045),
            radius=0.034,
            material=dark_steel,
            name=f"front_leg_{0 if x < 0 else 1}",
        )
        _add_rod(
            frame,
            (x, -0.77, 0.035),
            (x, 0.77, 0.035),
            radius=0.030,
            material=dark_steel,
            name=f"ground_rail_{0 if x < 0 else 1}",
        )
        frame.visual(
            Box((0.16, 0.09, 0.018)),
            origin=Origin(xyz=(x, -0.76, 0.012)),
            material=black_rubber,
            name=f"rear_foot_{0 if x < 0 else 1}",
        )
        frame.visual(
            Box((0.16, 0.09, 0.018)),
            origin=Origin(xyz=(x, 0.76, 0.012)),
            material=black_rubber,
            name=f"front_foot_{0 if x < 0 else 1}",
        )
    _add_rod(
        frame,
        (-1.04, 0.505, 0.62),
        (1.04, 0.505, 0.62),
        radius=0.018,
        material=dark_steel,
        name="front_spreader",
    )
    _add_rod(
        frame,
        (-1.04, -0.505, 0.62),
        (1.04, -0.505, 0.62),
        radius=0.018,
        material=dark_steel,
        name="rear_spreader",
    )
    for x in (-HANGER_X, HANGER_X):
        suffix = 0 if x < 0 else 1
        for y in (-0.036, 0.036):
            frame.visual(
                Box((0.105, 0.012, 0.170)),
                origin=Origin(xyz=(x, y, TOP_PIVOT_Z + 0.055)),
                material=galvanized,
                name=f"pivot_cheek_{suffix}_{0 if y < 0 else 1}",
            )
    frame.visual(
        Cylinder(radius=0.017, length=0.120),
        origin=Origin(xyz=(-HANGER_X, 0.0, TOP_PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="top_pin_0",
    )
    for lug_x in (-HANGER_X - 0.052, -HANGER_X + 0.052):
        frame.visual(
            Box((0.016, 0.050, 0.072)),
            origin=Origin(xyz=(lug_x, 0.0, TOP_PIVOT_Z + 0.045)),
            material=galvanized,
            name=f"pivot_lug_0_{0 if lug_x < -HANGER_X else 1}",
        )
    frame.visual(
        Cylinder(radius=0.017, length=0.120),
        origin=Origin(xyz=(HANGER_X, 0.0, TOP_PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="top_pin_1",
    )
    for lug_x in (HANGER_X - 0.052, HANGER_X + 0.052):
        frame.visual(
            Box((0.016, 0.050, 0.072)),
            origin=Origin(xyz=(lug_x, 0.0, TOP_PIVOT_Z + 0.045)),
            material=galvanized,
            name=f"pivot_lug_1_{0 if lug_x < HANGER_X else 1}",
        )

    hangers = []
    for index, x in enumerate((-HANGER_X, HANGER_X)):
        hanger = model.part(f"hanger_{index}")
        hanger.visual(
            Cylinder(radius=0.027, length=0.075),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name="upper_eye",
        )
        hanger.visual(
            Cylinder(radius=0.014, length=HANGER_LENGTH - 0.052),
            origin=Origin(xyz=(0.0, 0.0, -HANGER_LENGTH / 2.0), rpy=(pi, 0.0, 0.0)),
            material=galvanized,
            name="rigid_link",
        )
        hanger.visual(
            Cylinder(radius=0.029, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, -HANGER_LENGTH), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name="lower_eye",
        )
        hanger.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=(0.0, 0.0, -HANGER_LENGTH)),
            material=dark_steel,
            name="lower_pin_head",
        )
        hangers.append(hanger)
        model.articulation(
            f"top_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=hanger,
            origin=Origin(xyz=(x, 0.0, TOP_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.62, upper=0.62, effort=80.0, velocity=2.2),
            mimic=Mimic(joint="top_pivot_0") if index == 1 else None,
        )

    bucket = model.part("bucket")
    bucket.visual(
        Box((0.42, 0.380, 0.035)),
        origin=Origin(xyz=(0.23, 0.0, -0.305)),
        material=red_plastic,
        name="seat_pan",
    )
    bucket.visual(
        Box((0.46, 0.040, 0.420)),
        origin=Origin(xyz=(0.23, -0.195, -0.105)),
        material=red_plastic,
        name="high_back",
    )
    bucket.visual(
        Box((0.030, 0.420, 0.310)),
        origin=Origin(xyz=(0.055, 0.0, -0.165)),
        material=red_plastic,
        name="side_wall_0",
    )
    bucket.visual(
        Box((0.030, 0.420, 0.310)),
        origin=Origin(xyz=(0.405, 0.0, -0.165)),
        material=red_plastic,
        name="side_wall_1",
    )
    bucket.visual(_front_panel_mesh(), material=red_plastic, name="front_leg_panel")
    bucket.visual(
        Cylinder(radius=0.018, length=0.460),
        origin=Origin(xyz=(0.23, -0.215, 0.110), rpy=(0.0, pi / 2.0, 0.0)),
        material=red_plastic,
        name="back_rim",
    )
    for index, x in enumerate((0.055, 0.405)):
        bucket.visual(
            Cylinder(radius=0.015, length=0.350),
            origin=Origin(xyz=(x, 0.0, -0.055), rpy=(pi / 2.0, 0.0, 0.0)),
            material=red_plastic,
            name=f"side_rim_{index}",
        )
    bucket.visual(
        Cylinder(radius=0.016, length=0.420),
        origin=Origin(xyz=(0.23, 0.205, -0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=red_plastic,
        name="front_rim",
    )
    bucket.visual(
        Cylinder(radius=0.033, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=red_plastic,
        name="hanger_socket_0",
    )
    bucket.visual(
        Box((0.045, 0.026, 0.040)),
        origin=Origin(xyz=(0.034, -0.043, -0.020)),
        material=red_plastic,
        name="socket_web_0",
    )
    bucket.visual(
        Cylinder(radius=0.033, length=0.046),
        origin=Origin(xyz=(0.46, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=red_plastic,
        name="hanger_socket_1",
    )
    bucket.visual(
        Box((0.045, 0.026, 0.040)),
        origin=Origin(xyz=(0.426, -0.043, -0.020)),
        material=red_plastic,
        name="socket_web_1",
    )
    bucket.visual(
        Box((0.052, 0.044, 0.056)),
        origin=Origin(xyz=(0.020, 0.205, -0.035)),
        material=red_plastic,
        name="bar_pivot_pad_0",
    )
    bucket.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.020, 0.205, -0.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=red_plastic,
        name="bar_socket_0",
    )
    bucket.visual(
        Box((0.052, 0.044, 0.056)),
        origin=Origin(xyz=(0.440, 0.205, -0.035)),
        material=red_plastic,
        name="bar_pivot_pad_1",
    )
    bucket.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.440, 0.205, -0.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=red_plastic,
        name="bar_socket_1",
    )

    model.articulation(
        "bucket_mount",
        ArticulationType.FIXED,
        parent=hangers[0],
        child=bucket,
        origin=Origin(xyz=(0.0, 0.0, -HANGER_LENGTH)),
    )

    safety_bar = model.part("safety_bar")
    side_arm_0_origin, side_arm_0_length = _rod_origin_between(
        (-0.210, 0.022, 0.0), (-0.210, 0.070, -0.025)
    )
    safety_bar.visual(
        Cylinder(radius=0.012, length=side_arm_0_length),
        origin=side_arm_0_origin,
        material=yellow_plastic,
        name="side_arm_0",
    )
    side_arm_1_origin, side_arm_1_length = _rod_origin_between(
        (0.210, 0.022, 0.0), (0.210, 0.070, -0.025)
    )
    safety_bar.visual(
        Cylinder(radius=0.012, length=side_arm_1_length),
        origin=side_arm_1_origin,
        material=yellow_plastic,
        name="side_arm_1",
    )
    safety_bar.visual(
        Cylinder(radius=0.019, length=0.435),
        origin=Origin(xyz=(0.0, 0.070, -0.025), rpy=(0.0, pi / 2.0, 0.0)),
        material=yellow_plastic,
        name="front_tube",
    )
    safety_bar.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(-0.210, 0.0, 0.0)),
        material=yellow_plastic,
        name="pivot_knob_0",
    )
    safety_bar.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        material=yellow_plastic,
        name="pivot_knob_1",
    )
    model.articulation(
        "bar_pivot",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=safety_bar,
        origin=Origin(xyz=(0.23, 0.205, -0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.25, upper=1.25, effort=8.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hanger_0 = object_model.get_part("hanger_0")
    hanger_1 = object_model.get_part("hanger_1")
    bucket = object_model.get_part("bucket")
    safety_bar = object_model.get_part("safety_bar")
    top_pivot = object_model.get_articulation("top_pivot_0")
    bar_pivot = object_model.get_articulation("bar_pivot")

    ctx.allow_overlap(
        hanger_0,
        bucket,
        elem_a="lower_eye",
        elem_b="hanger_socket_0",
        reason="The rigid hanger eye is intentionally captured around the bucket side pivot socket.",
    )
    ctx.allow_overlap(
        hanger_1,
        bucket,
        elem_a="lower_eye",
        elem_b="hanger_socket_1",
        reason="The second rigid hanger eye is intentionally captured around the bucket side pivot socket.",
    )
    ctx.allow_overlap(
        hanger_0,
        bucket,
        elem_a="lower_pin_head",
        elem_b="hanger_socket_0",
        reason="The hanger pivot pin head is intentionally seated inside the molded bucket socket.",
    )
    ctx.allow_overlap(
        hanger_1,
        bucket,
        elem_a="lower_pin_head",
        elem_b="hanger_socket_1",
        reason="The second hanger pivot pin head is intentionally seated inside the molded bucket socket.",
    )
    ctx.allow_overlap(
        hanger_0,
        bucket,
        elem_a="rigid_link",
        elem_b="hanger_socket_0",
        reason="The lower end of the rigid hanger link is intentionally nested through the bucket pivot socket.",
    )
    ctx.allow_overlap(
        hanger_1,
        bucket,
        elem_a="rigid_link",
        elem_b="hanger_socket_1",
        reason="The second rigid hanger link is intentionally nested through the bucket pivot socket.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="pivot_knob_0",
        elem_b="bar_socket_0",
        reason="The rotating safety bar knob is seated in the molded side pivot socket.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="pivot_knob_1",
        elem_b="bar_socket_1",
        reason="The rotating safety bar knob is seated in the molded side pivot socket.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="pivot_knob_0",
        elem_b="bar_pivot_pad_0",
        reason="The safety bar pivot knob is partly recessed into the molded pivot pad.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="pivot_knob_1",
        elem_b="bar_pivot_pad_1",
        reason="The second safety bar pivot knob is partly recessed into the molded pivot pad.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="side_arm_0",
        elem_b="bar_socket_0",
        reason="The safety-bar side arm enters the pivot socket so the bar reads as a captured rotating bail.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="side_arm_1",
        elem_b="bar_socket_1",
        reason="The second safety-bar side arm enters the pivot socket so the bar reads as a captured rotating bail.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="side_arm_0",
        elem_b="bar_pivot_pad_0",
        reason="The safety-bar side arm is locally recessed into the molded pivot pad at the hinge.",
    )
    ctx.allow_overlap(
        safety_bar,
        bucket,
        elem_a="side_arm_1",
        elem_b="bar_pivot_pad_1",
        reason="The second safety-bar side arm is locally recessed into the molded pivot pad at the hinge.",
    )
    ctx.allow_overlap(
        "frame",
        hanger_0,
        elem_a="top_pin_0",
        elem_b="upper_eye",
        reason="The top pin passes through the hanger eye as the revolute swing bearing.",
    )
    ctx.allow_overlap(
        "frame",
        hanger_1,
        elem_a="top_pin_1",
        elem_b="upper_eye",
        reason="The second top pin passes through the hanger eye as the revolute swing bearing.",
    )

    ctx.expect_overlap(
        hanger_0,
        bucket,
        axes="xyz",
        elem_a="lower_eye",
        elem_b="hanger_socket_0",
        min_overlap=0.020,
        name="first hanger eye is captured by bucket socket",
    )
    ctx.expect_overlap(
        hanger_1,
        bucket,
        axes="xyz",
        elem_a="lower_eye",
        elem_b="hanger_socket_1",
        min_overlap=0.020,
        name="second hanger eye is captured by bucket socket",
    )
    ctx.expect_overlap(
        hanger_0,
        bucket,
        axes="xyz",
        elem_a="lower_pin_head",
        elem_b="hanger_socket_0",
        min_overlap=0.018,
        name="first hanger pin head seats in socket",
    )
    ctx.expect_overlap(
        hanger_1,
        bucket,
        axes="xyz",
        elem_a="lower_pin_head",
        elem_b="hanger_socket_1",
        min_overlap=0.018,
        name="second hanger pin head seats in socket",
    )
    ctx.expect_overlap(
        hanger_0,
        bucket,
        axes="xyz",
        elem_a="rigid_link",
        elem_b="hanger_socket_0",
        min_overlap=0.005,
        name="first rigid link remains nested at bucket pivot",
    )
    ctx.expect_overlap(
        hanger_1,
        bucket,
        axes="xyz",
        elem_a="rigid_link",
        elem_b="hanger_socket_1",
        min_overlap=0.005,
        name="second rigid link remains nested at bucket pivot",
    )
    ctx.expect_overlap(
        safety_bar,
        bucket,
        axes="xyz",
        elem_a="pivot_knob_0",
        elem_b="bar_socket_0",
        min_overlap=0.012,
        name="first safety bar knob is retained in side socket",
    )
    ctx.expect_overlap(
        safety_bar,
        bucket,
        axes="xyz",
        elem_a="pivot_knob_1",
        elem_b="bar_socket_1",
        min_overlap=0.012,
        name="second safety bar knob is retained in side socket",
    )
    ctx.expect_overlap(
        safety_bar,
        bucket,
        axes="xyz",
        elem_a="side_arm_0",
        elem_b="bar_socket_0",
        min_overlap=0.005,
        name="first safety bar side arm enters pivot socket",
    )
    ctx.expect_overlap(
        safety_bar,
        bucket,
        axes="xyz",
        elem_a="side_arm_1",
        elem_b="bar_socket_1",
        min_overlap=0.005,
        name="second safety bar side arm enters pivot socket",
    )
    ctx.expect_overlap(
        safety_bar,
        bucket,
        axes="xyz",
        elem_a="side_arm_0",
        elem_b="bar_pivot_pad_0",
        min_overlap=0.005,
        name="first safety bar side arm is supported by pivot pad",
    )
    ctx.expect_overlap(
        safety_bar,
        bucket,
        axes="xyz",
        elem_a="side_arm_1",
        elem_b="bar_pivot_pad_1",
        min_overlap=0.005,
        name="second safety bar side arm is supported by pivot pad",
    )
    ctx.expect_overlap(
        "frame",
        hanger_0,
        axes="xyz",
        elem_a="top_pin_0",
        elem_b="upper_eye",
        min_overlap=0.015,
        name="first top pin passes through hanger eye",
    )
    ctx.expect_overlap(
        "frame",
        hanger_1,
        axes="xyz",
        elem_a="top_pin_1",
        elem_b="upper_eye",
        min_overlap=0.015,
        name="second top pin passes through hanger eye",
    )

    rest_bucket = ctx.part_world_position(bucket)
    rest_hanger = ctx.part_element_world_aabb(hanger_1, elem="lower_eye")
    with ctx.pose({top_pivot: 0.45}):
        swung_bucket = ctx.part_world_position(bucket)
        swung_hanger = ctx.part_element_world_aabb(hanger_1, elem="lower_eye")
    ctx.check(
        "bucket swings forward as pendulum",
        rest_bucket is not None
        and swung_bucket is not None
        and swung_bucket[1] > rest_bucket[1] + 0.15
        and swung_bucket[2] > rest_bucket[2] + 0.08,
        details=f"rest={rest_bucket}, swung={swung_bucket}",
    )
    ctx.check(
        "paired hanger follows top pivot",
        rest_hanger is not None and swung_hanger is not None and swung_hanger[0][1] > rest_hanger[0][1] + 0.15,
        details=f"rest={rest_hanger}, swung={swung_hanger}",
    )

    closed_tube = ctx.part_element_world_aabb(safety_bar, elem="front_tube")
    with ctx.pose({bar_pivot: 1.0}):
        raised_tube = ctx.part_element_world_aabb(safety_bar, elem="front_tube")
    ctx.check(
        "front safety bar rotates upward",
        closed_tube is not None and raised_tube is not None and raised_tube[0][2] > closed_tube[0][2] + 0.035,
        details=f"closed={closed_tube}, raised={raised_tube}",
    )

    return ctx.report()


object_model = build_object_model()
