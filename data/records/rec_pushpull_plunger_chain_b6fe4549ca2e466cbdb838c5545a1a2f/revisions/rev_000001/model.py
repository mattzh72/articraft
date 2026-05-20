from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_x_tube(
    *,
    length: float,
    inner_radius: float,
    body_radius: float,
    collar_radius: float,
    collar_length: float,
):
    """A continuous hollow sleeve, authored as a lathe along local Z then rotated to X."""
    half = length * 0.5
    collar_inner = half - collar_length
    outer_profile = [
        (collar_radius, -half),
        (collar_radius, -collar_inner),
        (body_radius, -collar_inner - 0.004),
        (body_radius, collar_inner + 0.004),
        (collar_radius, collar_inner),
        (collar_radius, half),
    ]
    inner_profile = [
        (inner_radius, -half),
        (inner_radius, -collar_inner),
        (inner_radius, -collar_inner - 0.004),
        (inner_radius, collar_inner + 0.004),
        (inner_radius, collar_inner),
        (inner_radius, half),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _hollow_y_boss(*, width: float, inner_radius: float, outer_radius: float):
    """A lever pivot boss with a real clearance bore along local Y."""
    half = width * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(-math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_latch_plunger")

    black_oxide = model.material("black_oxide", rgba=(0.04, 0.045, 0.05, 1.0))
    cast_steel = model.material("cast_steel", rgba=(0.35, 0.37, 0.39, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    polished_rod = model.material("polished_rod", rgba=(0.86, 0.88, 0.90, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.63, 0.08, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        Box((0.44, 0.13, 0.014)),
        origin=Origin(xyz=(0.035, 0.0, -0.058)),
        material=black_oxide,
        name="base_plate",
    )
    sleeve.visual(
        mesh_from_geometry(
            _hollow_x_tube(
                length=0.320,
                inner_radius=0.023,
                body_radius=0.032,
                collar_radius=0.039,
                collar_length=0.022,
            ),
            "grounded_sleeve",
        ),
        material=brushed_steel,
        name="sleeve_shell",
    )
    for x_pos, name in [(-0.085, "rear_saddle"), (0.080, "front_saddle")]:
        sleeve.visual(
            Box((0.046, 0.090, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, -0.039)),
            material=cast_steel,
            name=name,
        )
    sleeve.visual(
        Box((0.105, 0.088, 0.023)),
        origin=Origin(xyz=(0.245, 0.0, -0.0395)),
        material=cast_steel,
        name="fork_foot",
    )
    for y_pos, name in [(-0.031, "fork_cheek_0"), (0.031, "fork_cheek_1")]:
        sleeve.visual(
            Box((0.092, 0.010, 0.104)),
            origin=Origin(xyz=(0.245, y_pos, 0.024)),
            material=cast_steel,
            name=name,
        )
    sleeve.visual(
        Cylinder(radius=0.0055, length=0.086),
        origin=Origin(xyz=(0.245, 0.0, 0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_pin",
    )
    for y_pos, name in [(-0.046, "pin_head_0"), (0.046, "pin_head_1")]:
        sleeve.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(0.245, y_pos, 0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=name,
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.016, length=0.250),
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rod,
        name="shaft",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.02601, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="nose_head",
    )
    plunger.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(-0.218, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="rear_stop",
    )

    output_lever = model.part("output_lever")
    output_lever.visual(
        mesh_from_geometry(
            _hollow_y_boss(width=0.024, inner_radius=0.0052, outer_radius=0.0145),
            "lever_pivot_boss",
        ),
        material=safety_yellow,
        name="pivot_boss",
    )
    drive_angle = math.atan2(-0.040, -0.032)
    output_lever.visual(
        Cylinder(radius=0.0058, length=0.0375),
        origin=Origin(xyz=(-0.0255, 0.0, -0.0204), rpy=(0.0, drive_angle, 0.0)),
        material=safety_yellow,
        name="drive_arm",
    )
    output_lever.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.040, 0.0, -0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="contact_roller",
    )
    output_angle = math.atan2(0.026, 0.038)
    output_lever.visual(
        Cylinder(radius=0.0058, length=0.0325),
        origin=Origin(xyz=(0.0170, 0.0, 0.0248), rpy=(0.0, output_angle, 0.0)),
        material=safety_yellow,
        name="output_arm",
    )
    output_lever.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(0.026, 0.0, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="output_tip",
    )

    model.articulation(
        "sleeve_to_plunger",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=plunger,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.045),
    )
    model.articulation(
        "fork_to_lever",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=output_lever,
        origin=Origin(xyz=(0.245, 0.0, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.75, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("output_lever")
    slide = object_model.get_articulation("sleeve_to_plunger")
    pivot = object_model.get_articulation("fork_to_lever")

    ctx.allow_overlap(
        sleeve,
        lever,
        elem_a="pivot_pin",
        elem_b="pivot_boss",
        reason="The grounded fork pin is intentionally captured in the lever boss bore as a compact revolute bearing proxy.",
    )

    ctx.check(
        "plunger uses a prismatic sleeve joint",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.040,
    )
    ctx.check(
        "output lever uses a fork pivot",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot.axis == (0.0, 1.0, 0.0)
        and pivot.motion_limits is not None
        and pivot.motion_limits.lower is not None
        and pivot.motion_limits.lower < -0.4,
    )
    ctx.expect_within(
        plunger,
        sleeve,
        axes="yz",
        inner_elem="shaft",
        outer_elem="sleeve_shell",
        margin=0.002,
        name="plunger shaft stays centered in sleeve bore",
    )
    ctx.expect_overlap(
        plunger,
        sleeve,
        axes="x",
        elem_a="shaft",
        elem_b="sleeve_shell",
        min_overlap=0.150,
        name="plunger remains inserted in sleeve at rest",
    )
    ctx.expect_contact(
        plunger,
        lever,
        elem_a="nose_head",
        elem_b="contact_roller",
        contact_tol=0.0025,
        name="plunger nose meets lever roller",
    )
    ctx.expect_overlap(
        sleeve,
        lever,
        axes="y",
        elem_a="pivot_pin",
        elem_b="pivot_boss",
        min_overlap=0.018,
        name="lever boss is carried by the fork pin",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.040}):
        ctx.expect_within(
            plunger,
            sleeve,
            axes="yz",
            inner_elem="shaft",
            outer_elem="sleeve_shell",
            margin=0.002,
            name="extended plunger remains centered in sleeve",
        )
        ctx.expect_overlap(
            plunger,
            sleeve,
            axes="x",
            elem_a="shaft",
            elem_b="sleeve_shell",
            min_overlap=0.110,
            name="extended plunger retains sleeve insertion",
        )
        extended_pos = ctx.part_world_position(plunger)
    ctx.check(
        "plunger extends forward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.035,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_tip = ctx.part_element_world_aabb(lever, elem="output_tip")
    with ctx.pose({pivot: -0.65}):
        turned_tip = ctx.part_element_world_aabb(lever, elem="output_tip")
    if rest_tip is not None and turned_tip is not None:
        rest_tip_z = 0.5 * (rest_tip[0][2] + rest_tip[1][2])
        turned_tip_z = 0.5 * (turned_tip[0][2] + turned_tip[1][2])
        moved = abs(turned_tip_z - rest_tip_z) > 0.006
    else:
        moved = False
        rest_tip_z = turned_tip_z = None
    ctx.check(
        "output lever swings about fork pin",
        moved,
        details=f"rest_tip_z={rest_tip_z}, turned_tip_z={turned_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
