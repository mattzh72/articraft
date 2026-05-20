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
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_between(part, start, end, radius, *, material, name, overrun=0.015):
    """Add a round tube whose centerline lies in the XZ plane."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dz = ez - sz
    length = math.hypot(dx, dz) + overrun
    theta = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, theta, 0.0),
        ),
        material=material,
        name=name,
    )


def _hollow_tube(length: float, outer_radius: float, inner_radius: float):
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _saddle_shape():
    # A compact exercise-bike saddle: broad rear, tapered nose, modest thickness.
    profile = [
        (0.155, 0.000),
        (0.095, 0.052),
        (-0.055, 0.095),
        (-0.155, 0.070),
        (-0.175, 0.000),
        (-0.155, -0.070),
        (-0.055, -0.095),
        (0.095, -0.052),
    ]
    return cq.Workplane("XY").polyline(profile).close().extrude(0.045)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("powder_coated_black", rgba=(0.02, 0.023, 0.026, 1.0))
    graphite = model.material("graphite_shroud", rgba=(0.09, 0.10, 0.11, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    accent_blue = model.material("blue_trim", rgba=(0.02, 0.23, 0.62, 1.0))
    saddle_vinyl = model.material("black_saddle", rgba=(0.015, 0.012, 0.010, 1.0))
    console_gray = model.material("console_gray", rgba=(0.12, 0.13, 0.14, 1.0))

    frame = model.part("frame")

    # Floor stabilizers and central tray-like spine.
    frame.visual(
        Cylinder(radius=0.036, length=0.68),
        origin=Origin(xyz=(-0.54, 0.0, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="rear_stabilizer",
    )
    frame.visual(
        Cylinder(radius=0.036, length=0.68),
        origin=Origin(xyz=(0.66, 0.0, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="front_stabilizer",
    )
    for x, prefix in [(-0.54, "rear"), (0.66, "front")]:
        for y, suffix in [(-0.36, "foot_0"), (0.36, "foot_1")]:
            frame.visual(
                Cylinder(radius=0.044, length=0.082),
                origin=Origin(xyz=(x, y, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_rubber,
                name=f"{prefix}_{suffix}",
            )
    frame.visual(
        Cylinder(radius=0.032, length=1.23),
        origin=Origin(xyz=(0.06, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_black,
        name="floor_spine",
    )
    frame.visual(
        Box((0.94, 0.25, 0.13)),
        origin=Origin(xyz=(0.10, 0.0, 0.195)),
        material=graphite,
        name="base_tray",
    )

    # Large stationary flywheel shroud with a much smaller exposed spinning wheel.
    frame.visual(
        Cylinder(radius=0.285, length=0.22),
        origin=Origin(xyz=(0.38, 0.0, 0.435), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="flywheel_shroud",
    )
    frame.visual(
        mesh_from_cadquery(_hollow_tube(0.010, 0.303, 0.268), "shroud_trim"),
        origin=Origin(xyz=(0.38, -0.120, 0.435), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_blue,
        name="shroud_trim",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(xyz=(0.38, 0.0, 0.435), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="flywheel_axle_boss",
    )

    # Bottom bracket and welded support struts.
    bottom_bracket_xyz = (-0.055, 0.0, 0.430)
    frame.visual(
        Cylinder(radius=0.058, length=0.24),
        origin=Origin(xyz=bottom_bracket_xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="bottom_bracket_boss",
    )
    _tube_between(
        frame,
        (-0.19, 0.0, 0.245),
        (-0.055, 0.0, 0.365),
        0.024,
        material=frame_black,
        name="bracket_down_tube",
    )
    _tube_between(
        frame,
        (-0.085, 0.085, 0.485),
        (-0.370, 0.085, 0.655),
        0.023,
        material=frame_black,
        name="seat_stay",
    )
    _tube_between(
        frame,
        (0.45, 0.0, 0.245),
        (0.38, 0.0, 0.360),
        0.023,
        material=frame_black,
        name="front_shroud_stay",
    )

    # Hollow seat sleeve so the telescoping post reads as a real tube-in-tube fit.
    frame.visual(
        mesh_from_cadquery(_hollow_tube(0.58, 0.036, 0.027), "seat_sleeve"),
        origin=Origin(xyz=(-0.315, 0.0, 0.250)),
        material=frame_black,
        name="seat_sleeve",
    )
    frame.visual(
        Box((0.13, 0.11, 0.030)),
        origin=Origin(xyz=(-0.315, 0.0, 0.250)),
        material=frame_black,
        name="seat_sleeve_base",
    )
    frame.visual(
        mesh_from_cadquery(_hollow_tube(0.026, 0.050, 0.020), "seat_clamp"),
        origin=Origin(xyz=(-0.315, 0.0, 0.817)),
        material=brushed_metal,
        name="seat_clamp",
    )

    # Handlebar mast, crossbar, upright grips, and simple console.
    _tube_between(
        frame,
        (0.49, 0.0, 0.565),
        (0.79, 0.0, 1.120),
        0.026,
        material=frame_black,
        name="handlebar_mast",
    )
    frame.visual(
        Cylinder(radius=0.023, length=0.56),
        origin=Origin(xyz=(0.81, 0.0, 1.135), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="handlebar_crossbar",
    )
    for y, name in [(-0.24, "grip_0"), (0.24, "grip_1")]:
        frame.visual(
            Cylinder(radius=0.020, length=0.22),
            origin=Origin(xyz=(0.82, y, 1.235)),
            material=dark_rubber,
            name=name,
        )
    frame.visual(
        Box((0.16, 0.055, 0.10)),
        origin=Origin(xyz=(0.76, 0.0, 1.190), rpy=(0.0, 0.0, 0.0)),
        material=console_gray,
        name="console_pod",
    )
    frame.visual(
        Box((0.115, 0.010, 0.055)),
        origin=Origin(xyz=(0.765, -0.031, 1.198)),
        material=accent_blue,
        name="display_lens",
    )

    flywheel = model.part("flywheel")
    flywheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.145,
                0.036,
                rim=WheelRim(
                    inner_radius=0.093,
                    flange_height=0.006,
                    flange_thickness=0.004,
                    bead_seat_depth=0.002,
                ),
                hub=WheelHub(radius=0.033, width=0.030, cap_style="domed"),
                face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="split_y", count=6, thickness=0.004, window_radius=0.008),
                bore=WheelBore(style="round", diameter=0.014),
            ),
            "flywheel_ring",
        ),
        origin=Origin(xyz=(0.0, -0.150, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=brushed_metal,
        name="flywheel_ring",
    )
    flywheel.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="axle",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="axle",
    )
    crank.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, -0.148, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="crank_disc",
    )
    crank.visual(
        Box((0.028, 0.018, 0.255)),
        origin=Origin(xyz=(0.0, -0.165, -0.125)),
        material=brushed_metal,
        name="lower_arm",
    )
    crank.visual(
        Box((0.028, 0.018, 0.255)),
        origin=Origin(xyz=(0.0, 0.165, 0.125)),
        material=brushed_metal,
        name="upper_arm",
    )
    crank.visual(
        Box((0.105, 0.052, 0.026)),
        origin=Origin(xyz=(0.0, -0.180, -0.258)),
        material=dark_rubber,
        name="pedal_0",
    )
    crank.visual(
        Box((0.105, 0.052, 0.026)),
        origin=Origin(xyz=(0.0, 0.180, 0.258)),
        material=dark_rubber,
        name="pedal_1",
    )

    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.021, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=brushed_metal,
        name="inner_post",
    )
    seat_post.visual(
        Cylinder(radius=0.026, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=brushed_metal,
        name="saddle_stem",
    )
    seat_post.visual(
        Box((0.13, 0.10, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=brushed_metal,
        name="saddle_rail",
    )
    seat_post.visual(
        mesh_from_cadquery(_saddle_shape(), "saddle_cushion"),
        origin=Origin(xyz=(-0.015, 0.0, 0.275)),
        material=saddle_vinyl,
        name="saddle_cushion",
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.38, 0.0, 0.435)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=bottom_bracket_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=10.0),
    )
    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.315, 0.0, 0.830)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank")
    seat_post = object_model.get_part("seat_post")
    flywheel_spin = object_model.get_articulation("flywheel_spin")
    crank_spin = object_model.get_articulation("crank_spin")
    seat_height = object_model.get_articulation("seat_height")

    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_shroud",
        elem_b="axle",
        reason="The flywheel axle is intentionally captured through the solid shroud proxy.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="axle",
        reason="The flywheel axle is intentionally nested in the metal axle boss.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bottom_bracket_boss",
        elem_b="axle",
        reason="The crank axle is intentionally captured inside the bottom-bracket boss.",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_clamp",
        elem_b="inner_post",
        reason="The seat clamp is modeled as a lightly compressed collar gripping the sliding post.",
    )

    ctx.check(
        "required continuous joints",
        flywheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and crank_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"flywheel={flywheel_spin.articulation_type}, crank={crank_spin.articulation_type}",
    )
    ctx.check(
        "seat post prismatic joint",
        seat_height.articulation_type == ArticulationType.PRISMATIC,
        details=f"seat joint={seat_height.articulation_type}",
    )

    ctx.expect_within(
        flywheel,
        frame,
        axes="xz",
        inner_elem="flywheel_ring",
        outer_elem="flywheel_shroud",
        margin=0.003,
        name="small flywheel sits within larger shroud silhouette",
    )
    ctx.expect_gap(
        frame,
        flywheel,
        axis="y",
        positive_elem="flywheel_shroud",
        negative_elem="flywheel_ring",
        min_gap=0.010,
        name="exposed flywheel clears shroud side",
    )
    ctx.expect_within(
        flywheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="flywheel_shroud",
        margin=0.004,
        name="flywheel axle centered in housing",
    )
    ctx.expect_overlap(
        flywheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="flywheel_shroud",
        min_overlap=0.18,
        name="flywheel axle passes through shroud",
    )
    ctx.expect_within(
        flywheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="flywheel_axle_boss",
        margin=0.003,
        name="flywheel axle centered in axle boss",
    )
    ctx.expect_overlap(
        flywheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="flywheel_axle_boss",
        min_overlap=0.20,
        name="flywheel axle passes through axle boss",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="bottom_bracket_boss",
        margin=0.003,
        name="crank axle centered in bottom bracket",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bottom_bracket_boss",
        min_overlap=0.20,
        name="crank axle passes through bottom bracket",
    )
    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.002,
        name="seat post remains inside column",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.36,
        name="lowered seat post retains insertion",
    )
    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_clamp",
        margin=0.002,
        name="seat post runs through clamp collar",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_clamp",
        min_overlap=0.020,
        name="clamp collar grips seat post height",
    )

    rest = ctx.part_world_position(seat_post)
    with ctx.pose({seat_height: 0.16}):
        raised = ctx.part_world_position(seat_post)
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.002,
            name="raised seat post stays aligned in column",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.20,
            name="raised seat post retains insertion",
        )
    ctx.check(
        "seat post adjusts upward",
        rest is not None and raised is not None and raised[2] > rest[2] + 0.14,
        details=f"rest={rest}, raised={raised}",
    )

    return ctx.report()


object_model = build_object_model()
