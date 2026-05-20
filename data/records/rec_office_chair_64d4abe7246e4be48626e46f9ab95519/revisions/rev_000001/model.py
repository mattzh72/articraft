from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _superellipse_loop(
    size_x: float,
    size_y: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    exponent: float = 3.2,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    """Rounded-rectangle-like loop in the XY plane, used for padded lofts."""
    pts: list[tuple[float, float, float]] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        x = center_x + 0.5 * size_x * math.copysign(abs(c) ** (2.0 / exponent), c)
        y = center_y + 0.5 * size_y * math.copysign(abs(s) ** (2.0 / exponent), s)
        pts.append((x, y, z))
    return pts


def _padded_loft(
    sections: list[tuple[float, float, float, float]],
    *,
    exponent: float = 3.2,
    segments: int = 56,
):
    """Build a soft, capped loft from (z, size_x, size_y, center_x) sections."""
    profiles = [
        _superellipse_loop(
            size_x,
            size_y,
            z,
            center_x=center_x,
            exponent=exponent,
            segments=segments,
        )
        for z, size_x, size_y, center_x in sections
    ]
    return LoftGeometry(profiles, cap=True, closed=True)


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(tube, name, tolerance=0.001, angular_tolerance=0.08)


def _add_radial_box(part, name: str, theta: float, radial: float, tangential: float, z: float, size):
    x = radial * math.cos(theta) - tangential * math.sin(theta)
    y = radial * math.sin(theta) + tangential * math.cos(theta)
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, theta)),
        material="brushed_metal",
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_office_chair")

    model.material("black_leather", rgba=(0.015, 0.013, 0.012, 1.0))
    model.material("lumbar_leather", rgba=(0.045, 0.040, 0.036, 1.0))
    model.material("brushed_metal", rgba=(0.52, 0.54, 0.56, 1.0))
    model.material("dark_metal", rgba=(0.045, 0.048, 0.052, 1.0))
    model.material("soft_black", rgba=(0.005, 0.005, 0.006, 1.0))
    model.material("charcoal_plastic", rgba=(0.08, 0.075, 0.07, 1.0))

    # Root: wide five-star base with a real hollow gas-lift sleeve and caster forks.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.105, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material="brushed_metal",
        name="base_hub",
    )
    base.visual(
        _tube_mesh("outer_sleeve", outer_radius=0.060, inner_radius=0.043, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material="brushed_metal",
        name="outer_sleeve",
    )
    base.visual(
        _tube_mesh("sleeve_collar", outer_radius=0.078, inner_radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.397)),
        material="dark_metal",
        name="sleeve_collar",
    )

    caster_radius = 0.045
    caster_radial = 0.585
    wheel_parts = []
    for i in range(5):
        theta = 2.0 * math.pi * i / 5.0 + math.pi / 2.0
        _add_radial_box(
            base,
            f"star_rib_{i}",
            theta,
            radial=0.320,
            tangential=0.0,
            z=0.125,
            size=(0.510, 0.066, 0.036),
        )
        _add_radial_box(
            base,
            f"caster_bridge_{i}",
            theta,
            radial=0.552,
            tangential=0.0,
            z=0.106,
            size=(0.120, 0.115, 0.020),
        )
        _add_radial_box(
            base,
            f"caster_cheek_a_{i}",
            theta,
            radial=0.585,
            tangential=0.046,
            z=0.058,
            size=(0.082, 0.010, 0.080),
        )
        _add_radial_box(
            base,
            f"caster_cheek_b_{i}",
            theta,
            radial=0.585,
            tangential=-0.046,
            z=0.058,
            size=(0.082, 0.010, 0.080),
        )

        caster = model.part(f"caster_{i}")
        caster.visual(
            Cylinder(radius=caster_radius, length=0.022),
            origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="soft_black",
            name="tire_a",
        )
        caster.visual(
            Cylinder(radius=caster_radius, length=0.022),
            origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="soft_black",
            name="tire_b",
        )
        caster.visual(
            Cylinder(radius=0.020, length=0.026),
            origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_metal",
            name="hub_a",
        )
        caster.visual(
            Cylinder(radius=0.020, length=0.026),
            origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_metal",
            name="hub_b",
        )
        caster.visual(
            Cylinder(radius=0.006, length=0.082),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_metal",
            name="axle",
        )
        wheel_parts.append((caster, theta))
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(
                xyz=(caster_radial * math.cos(theta), caster_radial * math.sin(theta), caster_radius),
                rpy=(0.0, 0.0, theta + math.pi / 2.0),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    # Moving seat frame: inner gas-lift post, stamped plate, recline housing,
    # armrest guide channels, and the underside knob boss.
    seat_frame = model.part("seat_frame")
    seat_frame.visual(
        Cylinder(radius=0.043, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material="brushed_metal",
        name="inner_post",
    )
    seat_frame.visual(
        Box((0.550, 0.540, 0.035)),
        origin=Origin(xyz=(0.030, 0.0, 0.095)),
        material="dark_metal",
        name="seat_plate",
    )
    seat_frame.visual(
        Box((0.520, 0.032, 0.060)),
        origin=Origin(xyz=(0.030, 0.286, 0.065)),
        material="dark_metal",
        name="plate_flange_left",
    )
    seat_frame.visual(
        Box((0.520, 0.032, 0.060)),
        origin=Origin(xyz=(0.030, -0.286, 0.065)),
        material="dark_metal",
        name="plate_flange_right",
    )
    seat_frame.visual(
        Box((0.032, 0.500, 0.050)),
        origin=Origin(xyz=(0.318, 0.0, 0.070)),
        material="dark_metal",
        name="front_lip",
    )
    seat_frame.visual(
        Cylinder(radius=0.066, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="brushed_metal",
        name="post_collar",
    )
    seat_frame.visual(
        Box((0.145, 0.255, 0.075)),
        origin=Origin(xyz=(-0.305, 0.0, 0.110)),
        material="dark_metal",
        name="recline_housing",
    )
    seat_frame.visual(
        Cylinder(radius=0.033, length=0.270),
        origin=Origin(xyz=(-0.340, 0.0, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="hinge_barrel",
    )
    seat_frame.visual(
        Box((0.065, 0.038, 0.035)),
        origin=Origin(xyz=(-0.175, -0.300, 0.064)),
        material="dark_metal",
        name="knob_boss",
    )

    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        y = y_sign * 0.335
        outer_y = y_sign * 0.363
        seat_frame.visual(
            Box((0.012, 0.074, 0.250)),
            origin=Origin(xyz=(0.043, y, 0.150)),
            material="brushed_metal",
            name=f"{side_name}_guide_front",
        )
        seat_frame.visual(
            Box((0.012, 0.074, 0.250)),
            origin=Origin(xyz=(-0.043, y, 0.150)),
            material="brushed_metal",
            name=f"{side_name}_guide_rear",
        )
        seat_frame.visual(
            Box((0.098, 0.012, 0.250)),
            origin=Origin(xyz=(0.0, outer_y, 0.150)),
            material="brushed_metal",
            name=f"{side_name}_guide_outer",
        )
        seat_frame.visual(
            Box((0.120, 0.055, 0.035)),
            origin=Origin(xyz=(0.0, y_sign * 0.285, 0.065)),
            material="dark_metal",
            name=f"{side_name}_guide_foot",
        )

    model.articulation(
        "base_to_seat",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.120, effort=650.0, velocity=0.20),
    )

    seat_cushion = model.part("seat_cushion")
    cushion_mesh = mesh_from_geometry(
        _padded_loft(
            [
                (-0.050, 0.530, 0.545, 0.000),
                (-0.020, 0.575, 0.590, 0.000),
                (0.030, 0.585, 0.600, 0.000),
                (0.052, 0.545, 0.560, 0.000),
            ],
            exponent=4.0,
        ),
        "seat_cushion",
    )
    seat_cushion.visual(cushion_mesh, material="black_leather", name="padded_box")
    seat_cushion.visual(
        Box((0.530, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.294, 0.010)),
        material="lumbar_leather",
        name="side_seam_left",
    )
    seat_cushion.visual(
        Box((0.530, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.294, 0.010)),
        material="lumbar_leather",
        name="side_seam_right",
    )
    model.articulation(
        "seat_to_cushion",
        ArticulationType.FIXED,
        parent=seat_frame,
        child=seat_cushion,
        origin=Origin(xyz=(0.080, 0.0, 0.162)),
    )

    # User-adjustable vertical T-shaped armrests sliding in the metal channels.
    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        armrest = model.part(f"{side_name}_armrest")
        armrest.visual(
            Box((0.044, 0.026, 0.260)),
            origin=Origin(xyz=(0.0, 0.0, 0.130)),
            material="brushed_metal",
            name="slide_post",
        )
        armrest.visual(
            Box((0.074, 0.024, 0.060)),
            origin=Origin(xyz=(0.0, 0.0, 0.090)),
            material="charcoal_plastic",
            name="guide_glide",
        )
        armrest.visual(
            Box((0.420, 0.104, 0.045)),
            origin=Origin(xyz=(0.025, 0.0, 0.300)),
            material="black_leather",
            name="top_pad",
        )
        armrest.visual(
            Box((0.095, 0.066, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, 0.260)),
            material="charcoal_plastic",
            name="t_socket",
        )
        model.articulation(
            f"seat_to_{side_name}_armrest",
            ArticulationType.PRISMATIC,
            parent=seat_frame,
            child=armrest,
            origin=Origin(xyz=(0.0, y_sign * 0.335, 0.050)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.120, effort=80.0, velocity=0.15),
        )

    # Reclining high back: hinge frame at the visible housing pivot.
    backrest = model.part("backrest")
    back_mesh = mesh_from_geometry(
        _padded_loft(
            [
                (0.070, 0.085, 0.420, -0.040),
                (0.250, 0.110, 0.510, -0.055),
                (0.560, 0.105, 0.560, -0.080),
                (0.850, 0.085, 0.500, -0.110),
                (0.910, 0.060, 0.400, -0.115),
            ],
            exponent=3.4,
            segments=64,
        ),
        "backrest_panel",
    )
    backrest.visual(back_mesh, material="black_leather", name="tall_panel")
    lumbar_mesh = mesh_from_geometry(
        _padded_loft(
            [
                (0.250, 0.018, 0.285, 0.006),
                (0.330, 0.032, 0.375, 0.012),
                (0.455, 0.030, 0.390, 0.012),
                (0.535, 0.014, 0.300, 0.006),
            ],
            exponent=3.0,
            segments=48,
        ),
        "lumbar_panel",
    )
    backrest.visual(lumbar_mesh, material="lumbar_leather", name="lumbar_panel")
    for y in (-0.178, 0.178):
        backrest.visual(
            Box((0.038, 0.030, 0.135)),
            origin=Origin(xyz=(-0.026, y, 0.055)),
            material="brushed_metal",
            name=f"hinge_lug_{'a' if y < 0 else 'b'}",
        )
    backrest.visual(
        Cylinder(radius=0.018, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="hinge_pin",
    )
    backrest.visual(
        Box((0.060, 0.360, 0.020)),
        origin=Origin(xyz=(-0.045, 0.0, 0.120)),
        material="brushed_metal",
        name="lower_crossbar",
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_frame,
        child=backrest,
        origin=Origin(xyz=(-0.340, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=160.0, velocity=0.7),
    )

    # Right underside tilt-lock knob, articulated as a rotary control.
    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        mesh_from_geometry(
            KnobGeometry(0.074, 0.032, body_style="lobed", crown_radius=0.002, edge_radius=0.001),
            "tilt_lock_knob",
        ),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="charcoal_plastic",
        name="lobed_knob",
    )
    model.articulation(
        "seat_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=seat_frame,
        child=tilt_knob,
        origin=Origin(xyz=(-0.180, -0.318, 0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat_frame = object_model.get_part("seat_frame")
    backrest = object_model.get_part("backrest")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")

    height = object_model.get_articulation("base_to_seat")
    recline = object_model.get_articulation("seat_to_backrest")
    left_slide = object_model.get_articulation("seat_to_left_armrest")
    right_slide = object_model.get_articulation("seat_to_right_armrest")

    ctx.allow_overlap(
        backrest,
        seat_frame,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The backrest hinge pin is intentionally captured inside the visible recline barrel.",
    )
    ctx.expect_overlap(
        backrest,
        seat_frame,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.240,
        name="captured hinge pin spans the recline barrel",
    )
    ctx.allow_overlap(
        base,
        seat_frame,
        elem_a="outer_sleeve",
        elem_b="inner_post",
        reason="The gas-lift post is intentionally modeled as a retained nested fit inside the fixed sleeve.",
    )
    ctx.allow_overlap(
        base,
        seat_frame,
        elem_a="sleeve_collar",
        elem_b="inner_post",
        reason="The top collar acts as a close bearing around the gas-lift post.",
    )
    ctx.expect_overlap(
        seat_frame,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="sleeve_collar",
        min_overlap=0.020,
        name="inner post passes through the sleeve collar bearing",
    )
    for i in range(5):
        caster = object_model.get_part(f"caster_{i}")
        for cheek in ("a", "b"):
            ctx.allow_overlap(
                base,
                caster,
                elem_a=f"caster_cheek_{cheek}_{i}",
                elem_b="axle",
                reason="The caster axle is captured in the fork cheek bore.",
            )
            ctx.expect_overlap(
                caster,
                base,
                axes="z",
                elem_a="axle",
                elem_b=f"caster_cheek_{cheek}_{i}",
                min_overlap=0.010,
                name=f"caster {i} axle is held by yoke cheek {cheek}",
            )

    ctx.expect_within(
        seat_frame,
        base,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="inner post is centered inside the fixed sleeve",
    )
    ctx.expect_overlap(
        seat_frame,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="outer_sleeve",
        min_overlap=0.080,
        name="low seat height retains gas-lift insertion",
    )

    low_seat = ctx.part_world_position(seat_frame)
    with ctx.pose({height: 0.120}):
        high_seat = ctx.part_world_position(seat_frame)
        ctx.expect_overlap(
            seat_frame,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.050,
            name="raised seat height still retained in sleeve",
        )
    ctx.check(
        "seat height lift moves upward",
        low_seat is not None and high_seat is not None and high_seat[2] > low_seat[2] + 0.10,
        details=f"low={low_seat}, high={high_seat}",
    )

    left_low = ctx.part_world_position(left_armrest)
    right_low = ctx.part_world_position(right_armrest)
    with ctx.pose({left_slide: 0.120, right_slide: 0.120}):
        left_high = ctx.part_world_position(left_armrest)
        right_high = ctx.part_world_position(right_armrest)
    ctx.check(
        "both armrests slide vertically",
        left_low is not None
        and left_high is not None
        and right_low is not None
        and right_high is not None
        and left_high[2] > left_low[2] + 0.10
        and right_high[2] > right_low[2] + 0.10,
        details=f"left {left_low}->{left_high}, right {right_low}->{right_high}",
    )

    rest_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({recline: 0.35}):
        reclined_aabb = ctx.part_world_aabb(backrest)
    if rest_aabb is not None and reclined_aabb is not None:
        rest_top_back_x = rest_aabb[0][0]
        reclined_top_back_x = reclined_aabb[0][0]
        ok = reclined_top_back_x < rest_top_back_x - 0.03
    else:
        ok = False
        rest_top_back_x = reclined_top_back_x = None
    ctx.check(
        "positive recline tilts the high back rearward",
        ok,
        details=f"rest_min_x={rest_top_back_x}, reclined_min_x={reclined_top_back_x}",
    )

    return ctx.report()


object_model = build_object_model()
