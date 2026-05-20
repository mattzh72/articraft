from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_tube(outer_radius: float, inner_radius: float, length: float, name: str):
    """A true hollow cylindrical tube, authored from z=0 to z=length."""
    shape = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=0.00035,
        angular_tolerance=0.06,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.025, 0.027, 0.030, 1.0))
    satin_black = model.material("satin_black", rgba=(0.075, 0.078, 0.082, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    kashima_gold = model.material("kashima_gold", rgba=(0.78, 0.55, 0.24, 1.0))
    brushed_alloy = model.material("brushed_alloy", rgba=(0.56, 0.58, 0.57, 1.0))
    cable_black = model.material("cable_black", rgba=(0.010, 0.010, 0.012, 1.0))
    ferrule_blue = model.material("ferrule_blue", rgba=(0.08, 0.22, 0.58, 1.0))
    bolt_dark = model.material("bolt_dark", rgba=(0.020, 0.020, 0.022, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _annular_tube(0.0180, 0.0134, 0.360, "outer_sleeve"),
        material=anodized_black,
        name="outer_sleeve",
    )
    outer_tube.visual(
        _annular_tube(0.0235, 0.0134, 0.034, "top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
        material=satin_black,
        name="top_collar",
    )
    outer_tube.visual(
        _annular_tube(0.0210, 0.0128, 0.012, "wiper_seal"),
        origin=Origin(xyz=(0.0, 0.0, 0.374)),
        material=graphite,
        name="wiper_seal",
    )
    # Four short polymer guide pads sit just inside the seal and make the
    # telescoping stanchion physically supported without filling the whole bore.
    for index, (x, y, sx, sy) in enumerate(
        (
            (0.0133, 0.0, 0.0042, 0.0040),
            (-0.0133, 0.0, 0.0042, 0.0040),
            (0.0, 0.0133, 0.0040, 0.0042),
            (0.0, -0.0133, 0.0040, 0.0042),
        )
    ):
        outer_tube.visual(
            Box((sx, sy, 0.012)),
            origin=Origin(xyz=(x, y, 0.380)),
            material=graphite,
            name=f"guide_pad_{index}",
        )
    outer_tube.visual(
        Cylinder(radius=0.0182, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=satin_black,
        name="bottom_endcap",
    )

    for index, z in enumerate((0.118, 0.246)):
        outer_tube.visual(
            _annular_tube(0.0205, 0.0174, 0.010, f"cable_band_{index}"),
            origin=Origin(xyz=(0.0, 0.0, z - 0.005)),
            material=graphite,
            name=f"cable_band_{index}",
        )
        outer_tube.visual(
            Box((0.015, 0.013, 0.012)),
            origin=Origin(xyz=(0.0270, 0.0, z)),
            material=graphite,
            name=f"cable_clip_{index}",
        )

    cable_path = tube_from_spline_points(
        [
            (0.026, 0.000, 0.052),
            (0.032, 0.004, 0.095),
            (0.033, 0.002, 0.180),
            (0.031, -0.002, 0.270),
            (0.025, -0.001, 0.344),
        ],
        radius=0.0027,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    outer_tube.visual(
        mesh_from_geometry(cable_path, "side_cable_housing"),
        material=cable_black,
        name="side_cable",
    )
    outer_tube.visual(
        Cylinder(radius=0.0056, length=0.019),
        origin=Origin(xyz=(0.023, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ferrule_blue,
        name="lower_cable_port",
    )
    outer_tube.visual(
        Cylinder(radius=0.0054, length=0.018),
        origin=Origin(xyz=(0.023, 0.0, 0.342), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ferrule_blue,
        name="upper_cable_stop",
    )
    outer_tube.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(xyz=(0.019, 0.0, 0.326)),
        material=bolt_dark,
        name="actuator_cover",
    )
    outer_tube.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.395)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0112, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=kashima_gold,
        name="sliding_stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0145, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.236)),
        material=satin_black,
        name="head_neck",
    )
    inner_post.visual(
        Box((0.058, 0.044, 0.021)),
        origin=Origin(xyz=(0.0, 0.0, 0.253)),
        material=brushed_alloy,
        name="lower_cradle",
    )
    inner_post.visual(
        Box((0.049, 0.038, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.273)),
        material=brushed_alloy,
        name="top_clamp",
    )
    for index, x in enumerate((-0.013, 0.013)):
        inner_post.visual(
            Cylinder(radius=0.0036, length=0.096),
            origin=Origin(xyz=(x, 0.0, 0.267), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bolt_dark,
            name=f"saddle_rail_{index}",
        )
    for index, y in enumerate((-0.014, 0.014)):
        inner_post.visual(
            Cylinder(radius=0.0043, length=0.016),
            origin=Origin(xyz=(0.0, y, 0.282)),
            material=bolt_dark,
            name=f"clamp_bolt_{index}",
        )
        inner_post.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(0.0, y, 0.291)),
            material=bolt_dark,
            name=f"bolt_head_{index}",
        )
    inner_post.visual(
        Box((0.018, 0.015, 0.018)),
        origin=Origin(xyz=(0.034, 0.0, 0.253)),
        material=brushed_alloy,
        name="side_clamp_ear",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.070, 0.055, 0.590)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "post_travel",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.150),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    travel = object_model.get_articulation("post_travel")

    ctx.check(
        "dropper has vertical prismatic travel",
        travel.articulation_type == ArticulationType.PRISMATIC
        and travel.axis == (0.0, 0.0, 1.0)
        and travel.motion_limits is not None
        and travel.motion_limits.upper is not None
        and travel.motion_limits.upper >= 0.145,
        details=f"type={travel.articulation_type}, axis={travel.axis}, limits={travel.motion_limits}",
    )
    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem="sliding_stanchion",
        outer_elem="outer_sleeve",
        margin=0.0005,
        name="stanchion centered in sleeve bore",
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="z",
        elem_a="sliding_stanchion",
        elem_b="outer_sleeve",
        min_overlap=0.105,
        name="collapsed post retains insertion",
    )

    rest_pos = ctx.part_world_position(inner_post)
    upper = travel.motion_limits.upper if travel.motion_limits and travel.motion_limits.upper else 0.150
    with ctx.pose({travel: upper}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="sliding_stanchion",
            outer_elem="outer_sleeve",
            margin=0.0005,
            name="extended stanchion stays coaxial",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="sliding_stanchion",
            elem_b="outer_sleeve",
            min_overlap=0.100,
            name="extended post remains inserted",
        )
        extended_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "post extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.140,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
