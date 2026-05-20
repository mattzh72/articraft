from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _flange_geometry(
    *,
    outer_radius: float,
    center_hole_radius: float,
    bolt_circle_radius: float,
    bolt_hole_radius: float,
    thickness: float,
) -> ExtrudeWithHolesGeometry:
    holes = [_circle_profile(center_hole_radius, segments=56)]
    for i in range(4):
        angle = i * math.pi / 2.0 + math.pi / 4.0
        cx = bolt_circle_radius * math.cos(angle)
        cy = bolt_circle_radius * math.sin(angle)
        holes.append(
            [
                (cx + bolt_hole_radius * math.cos(2.0 * math.pi * j / 28), cy + bolt_hole_radius * math.sin(2.0 * math.pi * j / 28))
                for j in range(28)
            ]
        )
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=96),
        holes,
        thickness,
        center=False,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_bridge_rotary_stack")

    dark_cast = model.material("dark_cast_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    drum_metal = model.material("satin_blued_steel", rgba=(0.25, 0.31, 0.36, 1.0))
    machined = model.material("brushed_steel", rgba=(0.67, 0.69, 0.66, 1.0))
    bridge_paint = model.material("bridge_safety_orange", rgba=(0.95, 0.45, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    red_mark = model.material("red_index_paint", rgba=(0.85, 0.05, 0.03, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.31, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_cast,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.135, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=machined,
        name="main_bearing_seat",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=dark_cast,
        name="center_spigot",
    )

    lower_drum = model.part("lower_drum")
    drum_mesh = LatheGeometry(
        [
            (0.0, 0.000),
            (0.164, 0.000),
            (0.198, 0.018),
            (0.206, 0.055),
            (0.206, 0.245),
            (0.198, 0.282),
            (0.164, 0.300),
            (0.0, 0.300),
        ],
        segments=96,
    )
    lower_drum.visual(
        mesh_from_geometry(drum_mesh, "stepped_lower_drum"),
        origin=Origin(),
        material=drum_metal,
        name="stepped_shell",
    )
    lower_drum.visual(
        Cylinder(radius=0.214, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=black_rubber,
        name="lower_grip_band",
    )
    lower_drum.visual(
        Cylinder(radius=0.214, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=black_rubber,
        name="upper_grip_band",
    )
    lower_drum.visual(
        Box((0.016, 0.050, 0.175)),
        origin=Origin(xyz=(0.205, 0.0, 0.150)),
        material=red_mark,
        name="drum_index_rib",
    )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.260, 0.170, 0.025)),
        origin=Origin(xyz=(0.060, 0.0, 0.0125)),
        material=bridge_paint,
        name="drum_saddle",
    )
    bridge.visual(
        Box((0.405, 0.108, 0.065)),
        origin=Origin(xyz=(0.338, 0.0, 0.0575)),
        material=bridge_paint,
        name="cantilever_box",
    )
    bridge.visual(
        Box((0.320, 0.014, 0.050)),
        origin=Origin(xyz=(0.350, 0.061, 0.030)),
        material=bridge_paint,
        name="side_web_0",
    )
    bridge.visual(
        Box((0.320, 0.014, 0.050)),
        origin=Origin(xyz=(0.350, -0.061, 0.030)),
        material=bridge_paint,
        name="side_web_1",
    )
    bridge.visual(
        Cylinder(radius=0.112, length=0.030),
        origin=Origin(xyz=(0.540, 0.0, 0.080)),
        material=machined,
        name="offset_bearing_pad",
    )
    bridge.visual(
        Cylinder(radius=0.036, length=0.052),
        origin=Origin(xyz=(0.540, 0.0, 0.056)),
        material=dark_cast,
        name="offset_pivot_post",
    )

    upper_flange = model.part("upper_flange")
    upper_flange.visual(
        mesh_from_geometry(
            _flange_geometry(
                outer_radius=0.108,
                center_hole_radius=0.036,
                bolt_circle_radius=0.074,
                bolt_hole_radius=0.009,
                thickness=0.035,
            ),
            "compact_upper_flange",
        ),
        origin=Origin(),
        material=machined,
        name="annular_plate",
    )
    upper_flange.visual(
        Cylinder(radius=0.049, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=drum_metal,
        name="raised_hub",
    )
    upper_flange.visual(
        Box((0.020, 0.092, 0.006)),
        origin=Origin(xyz=(0.078, 0.0, 0.038)),
        material=red_mark,
        name="flange_index_bar",
    )
    for i in range(4):
        angle = i * math.pi / 2.0 + math.pi / 4.0
        upper_flange.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(
                xyz=(
                    0.074 * math.cos(angle),
                    0.074 * math.sin(angle),
                    0.040,
                )
            ),
            material=dark_cast,
            name=f"bolt_head_{i}",
        )

    model.articulation(
        "base_to_lower_drum",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_drum,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "lower_drum_to_bridge",
        ArticulationType.FIXED,
        parent=lower_drum,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
    )
    model.articulation(
        "bridge_to_upper_flange",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=upper_flange,
        origin=Origin(xyz=(0.540, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_drum = object_model.get_part("lower_drum")
    bridge = object_model.get_part("bridge")
    upper_flange = object_model.get_part("upper_flange")
    lower_spin = object_model.get_articulation("base_to_lower_drum")
    upper_spin = object_model.get_articulation("bridge_to_upper_flange")

    ctx.expect_gap(
        lower_drum,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="stepped_shell",
        negative_elem="main_bearing_seat",
        name="lower drum seats on main bearing without sinking",
    )
    ctx.expect_contact(
        bridge,
        lower_drum,
        elem_a="drum_saddle",
        elem_b="stepped_shell",
        contact_tol=0.0015,
        name="cantilever saddle is mounted on the drum top",
    )
    ctx.expect_contact(
        upper_flange,
        bridge,
        elem_a="annular_plate",
        elem_b="offset_bearing_pad",
        contact_tol=0.0015,
        name="upper flange is carried by the offset pad",
    )
    ctx.expect_origin_distance(
        lower_drum,
        upper_flange,
        axes="xy",
        min_dist=0.50,
        name="upper rotary axis is deliberately offset from main axis",
    )

    lower_axis = tuple(round(v, 6) for v in lower_spin.axis)
    upper_axis = tuple(round(v, 6) for v in upper_spin.axis)
    ctx.check(
        "rotary axes are parallel and vertical",
        lower_axis == (0.0, 0.0, 1.0) and upper_axis == (0.0, 0.0, 1.0),
        details=f"lower_axis={lower_axis}, upper_axis={upper_axis}",
    )

    rest_upper = ctx.part_world_position(upper_flange)
    with ctx.pose({lower_spin: math.pi / 2.0, upper_spin: -math.pi / 2.0}):
        turned_upper = ctx.part_world_position(upper_flange)
        ctx.expect_origin_distance(
            lower_drum,
            upper_flange,
            axes="xy",
            min_dist=0.50,
            name="offset upper stage stays radial when lower drum turns",
        )

    ctx.check(
        "lower drum sweep carries the offset flange around the main axis",
        rest_upper is not None
        and turned_upper is not None
        and turned_upper[1] > rest_upper[1] + 0.45
        and abs(turned_upper[0]) < 0.08,
        details=f"rest={rest_upper}, turned={turned_upper}",
    )

    return ctx.report()


object_model = build_object_model()
