from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 40,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(
    length: float,
    radius: float,
    *,
    arc_segments: int = 18,
) -> list[tuple[float, float]]:
    """Flat-sided link outline with circular compact pin ends."""

    points: list[tuple[float, float]] = []
    for i in range(arc_segments + 1):
        angle = -math.pi / 2.0 + math.pi * i / arc_segments
        points.append((length + radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(arc_segments + 1):
        angle = math.pi / 2.0 + math.pi * i / arc_segments
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _tapered_tab_profile(
    length: float,
    root_radius: float,
    tab_radius: float,
    *,
    arc_segments: int = 18,
) -> list[tuple[float, float]]:
    """Tapered flat-sided second link ending in a smaller rounded tab."""

    points: list[tuple[float, float]] = []
    for i in range(arc_segments + 1):
        angle = -math.pi / 2.0 + math.pi * i / arc_segments
        points.append((length + tab_radius * math.cos(angle), tab_radius * math.sin(angle)))
    for i in range(arc_segments + 1):
        angle = math.pi / 2.0 + math.pi * i / arc_segments
        points.append((root_radius * math.cos(angle), root_radius * math.sin(angle)))
    return points


def _cheek_profile(*, radius: float = 0.060, rear_x: float = -0.125) -> list[tuple[float, float]]:
    """D-nosed fixed side cheek: flat rear edge, rounded pin nose."""

    points: list[tuple[float, float]] = []
    for i in range(19):
        angle = -math.pi / 2.0 + math.pi * i / 18
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    points.append((rear_x, radius))
    points.append((rear_x, -radius))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_two_joint_chain")

    dark_steel = Material("dark_burnished_steel", color=(0.20, 0.22, 0.23, 1.0))
    satin_steel = Material("satin_ground_steel", color=(0.56, 0.58, 0.57, 1.0))
    pin_steel = Material("polished_pin_steel", color=(0.78, 0.76, 0.70, 1.0))
    blackened = Material("blackened_socket_heads", color=(0.04, 0.04, 0.04, 1.0))

    first_pitch = 0.300
    second_pitch = 0.235
    plate_radius = 0.032
    tab_radius = 0.021
    pin_clearance_radius = 0.014
    small_hole_radius = 0.0065

    fixed_cheek = model.part("fixed_cheek")
    fixed_cheek.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _cheek_profile(),
                [
                    _circle_profile(-0.082, 0.035, 0.006),
                    _circle_profile(-0.082, -0.035, 0.006),
                ],
                0.006,
            ),
            "fixed_cheek_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_steel,
        name="cheek_plate",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=satin_steel,
        name="cheek_boss",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=pin_steel,
        name="root_pin_shaft",
    )
    fixed_cheek.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=pin_steel,
        name="root_pin_head",
    )
    for index, y in enumerate((-0.035, 0.035)):
        fixed_cheek.visual(
            Cylinder(radius=0.009, length=0.003),
            origin=Origin(xyz=(-0.082, y, -0.0055)),
            material=blackened,
            name=f"mount_bolt_{index}",
        )

    inner_link = model.part("inner_link")
    inner_link.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _capsule_profile(first_pitch, plate_radius),
                [
                    _circle_profile(0.0, 0.0, pin_clearance_radius),
                    _circle_profile(first_pitch, 0.0, pin_clearance_radius),
                ],
                0.006,
            ),
            "inner_link_plate",
        ),
        origin=Origin(),
        material=satin_steel,
        name="link_plate",
    )
    inner_link.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.0, 0.0, 0.028),
                [_circle_profile(0.0, 0.0, pin_clearance_radius)],
                0.004,
            ),
            "inner_link_proximal_boss",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_steel,
        name="proximal_boss",
    )
    inner_link.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(first_pitch, 0.0, 0.005)),
        material=satin_steel,
        name="distal_boss",
    )
    inner_link.visual(
        Cylinder(radius=0.010, length=0.025),
        origin=Origin(xyz=(first_pitch, 0.0, 0.0155)),
        material=pin_steel,
        name="distal_pin_shaft",
    )
    inner_link.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(first_pitch, 0.0, 0.025)),
        material=pin_steel,
        name="distal_pin_head",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _tapered_tab_profile(second_pitch, plate_radius, tab_radius),
                [
                    _circle_profile(0.0, 0.0, pin_clearance_radius),
                    _circle_profile(second_pitch, 0.0, small_hole_radius),
                ],
                0.006,
            ),
            "outer_link_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_steel,
        name="link_plate",
    )
    outer_link.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.0, 0.0, 0.027),
                [_circle_profile(0.0, 0.0, pin_clearance_radius)],
                0.004,
            ),
            "outer_link_joint_boss",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=satin_steel,
        name="joint_boss",
    )
    outer_link.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(second_pitch, 0.0, 0.017),
                [_circle_profile(second_pitch, 0.0, small_hole_radius)],
                0.004,
            ),
            "outer_link_end_tab_pad",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=satin_steel,
        name="end_tab_pad",
    )

    model.articulation(
        "cheek_to_inner",
        ArticulationType.REVOLUTE,
        parent=fixed_cheek,
        child=inner_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "inner_to_outer",
        ArticulationType.REVOLUTE,
        parent=inner_link,
        child=outer_link,
        origin=Origin(xyz=(first_pitch, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-1.60, upper=1.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_cheek = object_model.get_part("fixed_cheek")
    inner_link = object_model.get_part("inner_link")
    outer_link = object_model.get_part("outer_link")
    cheek_to_inner = object_model.get_articulation("cheek_to_inner")
    inner_to_outer = object_model.get_articulation("inner_to_outer")

    ctx.check(
        "two revolute chain joints",
        len(object_model.articulations) == 2
        and cheek_to_inner.articulation_type == ArticulationType.REVOLUTE
        and inner_to_outer.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "parallel supported axes",
        tuple(cheek_to_inner.axis) == (0.0, 0.0, 1.0)
        and tuple(inner_to_outer.axis) == (0.0, 0.0, 1.0),
        details=f"axes={cheek_to_inner.axis}, {inner_to_outer.axis}",
    )

    ctx.allow_overlap(
        fixed_cheek,
        inner_link,
        elem_a="root_pin_shaft",
        elem_b="link_plate",
        reason=(
            "The fixed cheek pin shaft is intentionally captured through the "
            "inner link bore so the first revolute axis reads as a supported pin."
        ),
    )
    ctx.allow_overlap(
        inner_link,
        outer_link,
        elem_a="distal_pin_shaft",
        elem_b="link_plate",
        reason=(
            "The distal pin shaft is intentionally captured through the outer "
            "link bore so the second revolute axis reads as a supported pin."
        ),
    )
    ctx.expect_within(
        fixed_cheek,
        inner_link,
        axes="xy",
        inner_elem="root_pin_shaft",
        outer_elem="link_plate",
        name="first pin shaft is centered inside the inner link footprint",
    )
    ctx.expect_overlap(
        fixed_cheek,
        inner_link,
        axes="z",
        elem_a="root_pin_shaft",
        elem_b="link_plate",
        min_overlap=0.005,
        name="first pin shaft passes through the plate thickness",
    )
    ctx.expect_within(
        inner_link,
        outer_link,
        axes="xy",
        inner_elem="distal_pin_shaft",
        outer_elem="link_plate",
        name="second pin shaft is centered inside the outer link footprint",
    )
    ctx.expect_overlap(
        inner_link,
        outer_link,
        axes="z",
        elem_a="distal_pin_shaft",
        elem_b="link_plate",
        min_overlap=0.005,
        name="second pin shaft passes through the plate thickness",
    )

    ctx.expect_contact(
        fixed_cheek,
        inner_link,
        elem_a="root_pin_head",
        elem_b="proximal_boss",
        contact_tol=0.00075,
        name="first joint pin head bears on proximal boss",
    )
    ctx.expect_contact(
        inner_link,
        outer_link,
        elem_a="distal_pin_head",
        elem_b="joint_boss",
        contact_tol=0.00075,
        name="second joint pin head bears on supported boss",
    )

    base_pos = ctx.part_world_position(outer_link)
    with ctx.pose({cheek_to_inner: 0.55, inner_to_outer: -0.80}):
        moved_pos = ctx.part_world_position(outer_link)
    ctx.check(
        "outer link follows the two revolute chain",
        base_pos is not None
        and moved_pos is not None
        and abs(moved_pos[1] - base_pos[1]) > 0.10
        and abs(moved_pos[2] - base_pos[2]) < 0.001,
        details=f"base={base_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
