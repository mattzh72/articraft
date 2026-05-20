from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _shift_profile(
    profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_food_processor")

    white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("dark_control_strip", rgba=(0.03, 0.035, 0.04, 1.0))
    clear = model.material("smoky_clear_plastic", rgba=(0.65, 0.82, 0.95, 0.42))
    grey = model.material("soft_grey", rgba=(0.34, 0.35, 0.36, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    amber = model.material("amber_button", rgba=(0.93, 0.56, 0.18, 1.0))

    base = model.part("base")
    base_body = ExtrudeGeometry.from_z0(
        superellipse_profile(0.32, 0.28, exponent=3.8, segments=72),
        0.34,
        cap=True,
    )
    top_pedestal = ExtrudeGeometry.from_z0(
        superellipse_profile(0.19, 0.17, exponent=3.4, segments=64),
        0.045,
        cap=True,
    ).translate(0.0, 0.0, 0.34)
    base_mesh = base_body.merge(top_pedestal)
    base.visual(
        mesh_from_geometry(base_mesh, "motor_base_shell"),
        material=white,
        name="motor_base_shell",
    )
    base.visual(
        Box((0.118, 0.006, 0.245)),
        origin=Origin(xyz=(0.0, -0.1425, 0.172)),
        material=dark,
        name="control_strip",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.389)),
        material=grey,
        name="drive_socket",
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.078, 0.000),
            (0.110, 0.014),
            (0.114, 0.300),
            (0.120, 0.328),
            (0.116, 0.340),
        ],
        [
            (0.040, 0.018),
            (0.088, 0.033),
            (0.094, 0.305),
            (0.097, 0.335),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "transparent_bowl_shell"),
        material=clear,
        name="bowl_shell",
    )

    lid = model.part("lid")
    feed_offset_y = 0.030
    lid_plate = ExtrudeWithHolesGeometry(
        _circle_profile(0.128, 96),
        [
            _shift_profile(
                rounded_rect_profile(0.092, 0.064, 0.014, corner_segments=8),
                0.0,
                feed_offset_y,
            )
        ],
        0.022,
        cap=True,
        center=False,
    )
    lid_skirt = LatheGeometry.from_shell_profiles(
        [(0.092, -0.026), (0.094, -0.006), (0.096, 0.000)],
        [(0.081, -0.023), (0.083, -0.006), (0.085, 0.000)],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    lid_mesh = lid_plate.merge(lid_skirt)
    lid.visual(
        mesh_from_geometry(lid_mesh, "clear_lid_with_feed_opening"),
        material=clear,
        name="lid_shell",
    )

    chimney = model.part("chimney")
    chimney_outer = rounded_rect_profile(0.118, 0.088, 0.018, corner_segments=10)
    chimney_inner = rounded_rect_profile(0.086, 0.058, 0.012, corner_segments=10)
    chimney_tube = ExtrudeWithHolesGeometry(
        chimney_outer,
        [chimney_inner],
        0.220,
        cap=True,
        center=False,
    )
    chimney.visual(
        mesh_from_geometry(chimney_tube, "feed_chimney_shell"),
        material=clear,
        name="chimney_tube",
    )

    pusher = model.part("pusher")
    pusher_plug = ExtrudeGeometry(
        rounded_rect_profile(0.074, 0.048, 0.010, corner_segments=8),
        0.172,
        cap=True,
        center=True,
    ).translate(0.0, 0.0, -0.086)
    pusher_cap = ExtrudeGeometry(
        rounded_rect_profile(0.135, 0.102, 0.020, corner_segments=10),
        0.026,
        cap=True,
        center=True,
    ).translate(0.0, 0.0, 0.013)
    pusher.visual(
        mesh_from_geometry(pusher_plug, "pusher_insert"),
        material=grey,
        name="pusher_plug",
    )
    pusher.visual(
        mesh_from_geometry(pusher_cap, "pusher_top_cap"),
        material=grey,
        name="pusher_cap",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.012, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.1115)),
        material=steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=black,
        name="spindle_hub",
    )
    spindle.visual(
        Box((0.162, 0.024, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.083), rpy=(0.0, 0.14, 0.18)),
        material=steel,
        name="blade_low",
    )
    spindle.visual(
        Box((0.145, 0.022, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.142), rpy=(0.0, -0.12, math.pi / 2.0 + 0.16)),
        material=steel,
        name="blade_high",
    )

    dial = model.part("dial")
    dial_knob = KnobGeometry(
        0.066,
        0.026,
        body_style="skirted",
        top_diameter=0.052,
        edge_radius=0.0012,
        grip=KnobGrip(style="fluted", count=22, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
    )
    dial.visual(
        mesh_from_geometry(dial_knob, "main_dial_knob"),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="dial_cap",
    )

    for idx, x in enumerate((-0.033, 0.033)):
        rocker = model.part(f"rocker_{idx}")
        rocker.visual(
            Box((0.052, 0.017, 0.036)),
            origin=Origin(xyz=(0.0, -0.0085, 0.0)),
            material=amber,
            name="rocker_paddle",
        )
        rocker.visual(
            Cylinder(radius=0.006, length=0.058),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="rocker_pivot",
        )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
    )
    model.articulation(
        "lid_to_chimney",
        ArticulationType.FIXED,
        parent=lid,
        child=chimney,
        origin=Origin(xyz=(0.0, feed_offset_y, 0.022)),
    )
    model.articulation(
        "chimney_to_pusher",
        ArticulationType.PRISMATIC,
        parent=chimney,
        child=pusher,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.120),
    )
    model.articulation(
        "base_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.389)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.1455, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    for idx, x in enumerate((-0.033, 0.033)):
        model.articulation(
            f"base_to_rocker_{idx}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=f"rocker_{idx}",
            origin=Origin(xyz=(x, -0.1455, 0.124)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.18, upper=0.18),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pusher = object_model.get_part("pusher")
    chimney = object_model.get_part("chimney")
    bowl = object_model.get_part("bowl")
    spindle = object_model.get_part("spindle")
    pusher_slide = object_model.get_articulation("chimney_to_pusher")

    ctx.expect_within(
        pusher,
        chimney,
        axes="xy",
        inner_elem="pusher_plug",
        outer_elem="chimney_tube",
        margin=0.003,
        name="pusher fits inside feed chimney footprint",
    )
    ctx.expect_overlap(
        pusher,
        chimney,
        axes="z",
        elem_a="pusher_plug",
        elem_b="chimney_tube",
        min_overlap=0.150,
        name="pusher is seated deeply in chimney at rest",
    )
    rest_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.120}):
        ctx.expect_within(
            pusher,
            chimney,
            axes="xy",
            inner_elem="pusher_plug",
            outer_elem="chimney_tube",
            margin=0.003,
            name="lifted pusher stays aligned to chute",
        )
        ctx.expect_overlap(
            pusher,
            chimney,
            axes="z",
            elem_a="pusher_plug",
            elem_b="chimney_tube",
            min_overlap=0.045,
            name="lifted pusher remains captured in chute",
        )
        raised_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher translates upward along chimney axis",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    ctx.expect_origin_distance(
        spindle,
        bowl,
        axes="xy",
        max_dist=0.001,
        name="blade spindle lies on bowl centerline",
    )
    ctx.expect_within(
        spindle,
        bowl,
        axes="xy",
        inner_elem="blade_low",
        outer_elem="bowl_shell",
        margin=0.0,
        name="lower blade stays inside bowl diameter",
    )

    ctx.check(
        "spindle and dial are continuous rotary controls",
        object_model.get_articulation("base_to_spindle").articulation_type
        == ArticulationType.CONTINUOUS
        and object_model.get_articulation("base_to_dial").articulation_type
        == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "rocker buttons use independent short pivots",
        object_model.get_articulation("base_to_rocker_0").articulation_type
        == ArticulationType.REVOLUTE
        and object_model.get_articulation("base_to_rocker_1").articulation_type
        == ArticulationType.REVOLUTE,
    )

    return ctx.report()


object_model = build_object_model()
