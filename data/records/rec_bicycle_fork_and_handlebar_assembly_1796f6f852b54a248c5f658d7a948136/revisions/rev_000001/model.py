from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _hollow_head_tube():
    return LatheGeometry.from_shell_profiles(
        [
            (0.038, -0.100),
            (0.043, -0.093),
            (0.043, -0.076),
            (0.038, -0.068),
            (0.038, 0.068),
            (0.043, 0.076),
            (0.043, 0.093),
            (0.038, 0.100),
        ],
        [
            (0.0195, -0.108),
            (0.0195, 0.108),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _crown_body():
    profile = rounded_rect_profile(0.125, 0.165, radius=0.026, corner_segments=10)
    return ExtrudeGeometry(profile, 0.060, center=True)


def _fork_blade(side_y: float):
    return sweep_profile_along_spline(
        [
            (0.018, side_y, -0.255),
            (0.033, side_y * 1.04, -0.360),
            (0.052, side_y * 1.02, -0.500),
            (0.066, side_y, -0.635),
        ],
        profile=rounded_rect_profile(0.031, 0.014, radius=0.005, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )


def _drop_handlebar():
    return tube_from_spline_points(
        [
            (0.085, -0.210, 0.075),
            (0.128, -0.210, 0.074),
            (0.196, -0.210, 0.107),
            (0.213, -0.210, 0.160),
            (0.181, -0.208, 0.202),
            (0.132, -0.180, 0.216),
            (0.130, -0.085, 0.216),
            (0.130, 0.000, 0.216),
            (0.130, 0.085, 0.216),
            (0.132, 0.180, 0.216),
            (0.181, 0.208, 0.202),
            (0.213, 0.210, 0.160),
            (0.196, 0.210, 0.107),
            (0.128, 0.210, 0.074),
            (0.085, 0.210, 0.075),
        ],
        radius=0.0105,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_fork_cockpit")

    frame_paint = model.material("frame_paint", rgba=(0.06, 0.12, 0.20, 1.0))
    carbon = model.material("carbon_satin", rgba=(0.015, 0.016, 0.018, 1.0))
    alloy = model.material("brushed_alloy", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_rubber = model.material("black_bar_tape", rgba=(0.025, 0.025, 0.025, 1.0))

    head_tube = model.part("head_tube")
    head_tube.visual(
        _mesh("head_tube_shell", _hollow_head_tube()),
        material=frame_paint,
        name="head_tube_shell",
    )
    head_tube.visual(
        _mesh(
            "top_tube_stub",
            tube_from_spline_points(
                [(-0.038, 0.0, 0.040), (-0.155, 0.0, 0.055), (-0.300, 0.0, 0.058)],
                radius=0.018,
                samples_per_segment=8,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=frame_paint,
        name="top_tube_stub",
    )
    head_tube.visual(
        _mesh(
            "down_tube_stub",
            tube_from_spline_points(
                [(-0.037, 0.0, -0.058), (-0.150, 0.0, -0.160), (-0.270, 0.0, -0.300)],
                radius=0.022,
                samples_per_segment=8,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=frame_paint,
        name="down_tube_stub",
    )
    head_tube.visual(
        _mesh("upper_headset_cup", TorusGeometry(radius=0.0285, tube=0.0048)),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=alloy,
        name="upper_headset_cup",
    )
    head_tube.visual(
        _mesh("lower_headset_cup", TorusGeometry(radius=0.0285, tube=0.0048)),
        origin=Origin(xyz=(0.0, 0.0, -0.107)),
        material=alloy,
        name="lower_headset_cup",
    )

    steerer = model.part("steerer")
    steerer.visual(
        Cylinder(radius=0.0138, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=alloy,
        name="steerer_tube",
    )
    steerer.visual(
        Cylinder(radius=0.0195, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=alloy,
        name="upper_bearing_race",
    )
    steerer.visual(
        Cylinder(radius=0.0195, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material=alloy,
        name="lower_bearing_race",
    )
    steerer.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=alloy,
        name="stem_steerer_clamp",
    )
    steerer.visual(
        Cylinder(radius=0.014, length=0.118),
        origin=Origin(xyz=(0.066, 0.0, 0.203), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="stem_extension",
    )
    steerer.visual(
        Cylinder(radius=0.025, length=0.060),
        origin=Origin(xyz=(0.130, 0.0, 0.214), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="bar_clamp",
    )
    steerer.visual(
        Box((0.012, 0.074, 0.038)),
        origin=Origin(xyz=(0.151, 0.0, 0.214)),
        material=alloy,
        name="stem_faceplate",
    )
    steerer.visual(
        _mesh("drop_handlebar", _drop_handlebar()),
        material=dark_rubber,
        name="drop_handlebar",
    )
    steerer.visual(
        Cylinder(radius=0.016, length=0.072),
        origin=Origin(xyz=(0.130, 0.0, 0.214), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="bare_bar_sleeve",
    )

    steerer.visual(
        _mesh("crown_body", _crown_body()),
        origin=Origin(xyz=(0.022, 0.0, -0.238)),
        material=carbon,
        name="crown_body",
    )
    steerer.visual(
        Cylinder(radius=0.027, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, -0.194)),
        material=carbon,
        name="crown_socket",
    )
    steerer.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.151)),
        material=alloy,
        name="crown_race",
    )
    for index, side_y, blade_name in (
        (0, -0.056, "blade_0"),
        (1, 0.056, "blade_1"),
    ):
        steerer.visual(
            _mesh(blade_name, _fork_blade(side_y)),
            material=carbon,
            name=blade_name,
        )
        steerer.visual(
            Box((0.042, 0.016, 0.045)),
            origin=Origin(xyz=(0.070, side_y, -0.655)),
            material=alloy,
            name=f"dropout_{index}",
        )
        steerer.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(xyz=(0.082, side_y, -0.670), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=alloy,
            name=f"axle_slot_{index}",
        )

    model.articulation(
        "headset_steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steerer,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-math.radians(70.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    steerer = object_model.get_part("steerer")
    steering = object_model.get_articulation("headset_steering")

    ctx.allow_overlap(
        head_tube,
        steerer,
        elem_a="head_tube_shell",
        elem_b="upper_bearing_race",
        reason=(
            "The upper bearing race is intentionally seated inside the simplified "
            "hollow head-tube shell so the steering assembly is visibly captured."
        ),
    )
    ctx.allow_overlap(
        head_tube,
        steerer,
        elem_a="head_tube_shell",
        elem_b="lower_bearing_race",
        reason=(
            "The lower bearing race is intentionally seated inside the simplified "
            "hollow head-tube shell so the steering assembly is visibly captured."
        ),
    )

    ctx.expect_within(
        steerer,
        head_tube,
        axes="xy",
        inner_elem="steerer_tube",
        outer_elem="head_tube_shell",
        margin=0.0,
        name="steerer centered inside hollow head tube",
    )
    ctx.expect_overlap(
        steerer,
        head_tube,
        axes="z",
        elem_a="steerer_tube",
        elem_b="head_tube_shell",
        min_overlap=0.18,
        name="steerer passes through head tube length",
    )
    for elem_name in ("upper_bearing_race", "lower_bearing_race"):
        ctx.expect_within(
            steerer,
            head_tube,
            axes="xy",
            inner_elem=elem_name,
            outer_elem="head_tube_shell",
            margin=0.0,
            name=f"{elem_name} is concentric with headset bore",
        )
        ctx.expect_overlap(
            steerer,
            head_tube,
            axes="z",
            elem_a=elem_name,
            elem_b="head_tube_shell",
            min_overlap=0.015,
            name=f"{elem_name} remains seated in head tube",
        )
    ctx.expect_overlap(
        steerer,
        steerer,
        axes="x",
        elem_a="blade_0",
        elem_b="blade_1",
        min_overlap=0.0,
        name="fork blades share fore-aft crown alignment",
    )

    rest_bar = ctx.part_element_world_aabb(steerer, elem="bar_clamp")
    with ctx.pose({steering: math.radians(35.0)}):
        turned_bar = ctx.part_element_world_aabb(steerer, elem="bar_clamp")
    if rest_bar is not None and turned_bar is not None:
        rest_center_y = 0.5 * (rest_bar[0][1] + rest_bar[1][1])
        turned_center_y = 0.5 * (turned_bar[0][1] + turned_bar[1][1])
    else:
        rest_center_y = turned_center_y = None
    ctx.check(
        "handlebar rotates with steerer",
        rest_center_y is not None
        and turned_center_y is not None
        and turned_center_y > rest_center_y + 0.035,
        details=f"rest_y={rest_center_y}, turned_y={turned_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
