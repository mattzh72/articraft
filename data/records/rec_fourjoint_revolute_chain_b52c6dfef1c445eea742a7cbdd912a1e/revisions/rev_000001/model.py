from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LINK_LAYER = 0.012
PLATE_THICKNESS = 0.010
BOSS_THICKNESS = 0.018
PIN_HOLE_RADIUS = 0.0085
PIN_RADIUS = 0.0058
PIN_HEAD_RADIUS = 0.014
PIN_HEAD_THICKNESS = 0.006
PIN_HEAD_OFFSET = LINK_LAYER + BOSS_THICKNESS * 0.5 + PIN_HEAD_THICKNESS * 0.5
PIN_SHAFT_LENGTH = 2.0 * (LINK_LAYER + BOSS_THICKNESS * 0.5)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + radius * cos(2.0 * pi * i / segments), cy + radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _capsule_profile(length: float, radius: float, *, arc_segments: int = 18) -> list[tuple[float, float]]:
    """Flat-sided two-boss link outline in the local X/Z drawing plane."""
    points: list[tuple[float, float]] = [(length, radius), (0.0, radius)]
    for i in range(1, arc_segments + 1):
        angle = pi / 2.0 + pi * i / arc_segments
        points.append((radius * cos(angle), radius * sin(angle)))
    points.append((length, -radius))
    for i in range(1, arc_segments):
        angle = -pi / 2.0 + pi * i / arc_segments
        points.append((length + radius * cos(angle), radius * sin(angle)))
    return points


def _extruded_plate_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    hole_profiles: list[list[tuple[float, float]]],
    thickness: float,
):
    # ExtrudeWithHolesGeometry extrudes in local Z; rotate so the plates are thin
    # in local Y and present their broad faces in the X/Z side view.
    geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geometry, name)


def _link_plate_mesh(name: str, length: float, boss_radius: float):
    return _extruded_plate_mesh(
        name,
        _capsule_profile(length, boss_radius),
        [
            _circle_profile(PIN_HOLE_RADIUS, center=(0.0, 0.0)),
            _circle_profile(PIN_HOLE_RADIUS, center=(length, 0.0)),
        ],
        PLATE_THICKNESS,
    )


def _tab_plate_mesh(name: str, length: float, boss_radius: float):
    return _extruded_plate_mesh(
        name,
        _capsule_profile(length, boss_radius),
        [
            _circle_profile(PIN_HOLE_RADIUS, center=(0.0, 0.0)),
            _circle_profile(0.0055, center=(length, 0.0), segments=28),
        ],
        PLATE_THICKNESS,
    )


def _boss_ring_mesh(name: str, outer_radius: float, inner_radius: float = PIN_HOLE_RADIUS):
    return _extruded_plate_mesh(
        name,
        _circle_profile(outer_radius, segments=48),
        [_circle_profile(inner_radius, segments=36)],
        BOSS_THICKNESS,
    )


def _add_pin_stack(
    part,
    *,
    shaft_name: str,
    neg_head_name: str,
    pos_head_name: str,
    x: float,
    material,
) -> None:
    pin_origin = Origin(xyz=(x, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_SHAFT_LENGTH),
        origin=pin_origin,
        material=material,
        name=shaft_name,
    )
    for head_name, y in ((neg_head_name, -PIN_HEAD_OFFSET), (pos_head_name, PIN_HEAD_OFFSET)):
        part.visual(
            Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
            origin=Origin(xyz=(x, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=material,
            name=head_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_four_joint_chain")

    cheek_paint = model.material("cheek_paint", rgba=(0.16, 0.19, 0.21, 1.0))
    link_steel = model.material("link_steel", rgba=(0.64, 0.66, 0.67, 1.0))
    boss_steel = model.material("boss_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.82, 0.80, 0.74, 1.0))
    tab_paint = model.material("tab_paint", rgba=(0.34, 0.47, 0.62, 1.0))

    cheek = model.part("cheek")
    cheek_profile = rounded_rect_profile(0.120, 0.165, 0.012, corner_segments=8)
    cheek_holes = [
        _circle_profile(PIN_HOLE_RADIUS, center=(0.0, 0.0)),
        _circle_profile(0.005, center=(-0.034, 0.052), segments=28),
        _circle_profile(0.005, center=(-0.034, -0.052), segments=28),
    ]
    cheek.visual(
        _extruded_plate_mesh("cheek_plate", cheek_profile, cheek_holes, PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, -LINK_LAYER, 0.0)),
        material=cheek_paint,
        name="plate",
    )
    cheek.visual(
        _boss_ring_mesh("cheek_pin_boss", 0.030),
        origin=Origin(xyz=(0.0, -LINK_LAYER, 0.0)),
        material=boss_steel,
        name="pin_boss",
    )
    _add_pin_stack(
        cheek,
        shaft_name="base_pin_shaft",
        neg_head_name="base_pin_neg_head",
        pos_head_name="base_pin_pos_head",
        x=0.0,
        material=pin_steel,
    )

    link_lengths = (0.160, 0.145, 0.130)
    link_layers = (LINK_LAYER, -LINK_LAYER, LINK_LAYER)
    links = []
    for index, (length, layer) in enumerate(zip(link_lengths, link_layers)):
        link = model.part(f"link_{index}")
        link.visual(
            _link_plate_mesh(f"link_{index}_plate", length, 0.027),
            origin=Origin(xyz=(0.0, layer, 0.0)),
            material=link_steel,
            name="plate",
        )
        boss_mesh = _boss_ring_mesh(f"link_{index}_boss_ring", 0.026)
        link.visual(
            boss_mesh,
            origin=Origin(xyz=(0.0, layer, 0.0)),
            material=boss_steel,
            name="prox_boss",
        )
        link.visual(
            boss_mesh,
            origin=Origin(xyz=(length, layer, 0.0)),
            material=boss_steel,
            name="dist_boss",
        )
        _add_pin_stack(
            link,
            shaft_name="dist_pin_shaft",
            neg_head_name="dist_pin_neg_head",
            pos_head_name="dist_pin_pos_head",
            x=length,
            material=pin_steel,
        )
        links.append(link)

    end_tab = model.part("end_tab")
    end_tab_length = 0.078
    end_tab.visual(
        _tab_plate_mesh("end_tab_plate", end_tab_length, 0.025),
        origin=Origin(xyz=(0.0, -LINK_LAYER, 0.0)),
        material=tab_paint,
        name="plate",
    )
    end_tab.visual(
        _boss_ring_mesh("end_tab_pivot_boss", 0.025),
        origin=Origin(xyz=(0.0, -LINK_LAYER, 0.0)),
        material=boss_steel,
        name="pivot_boss",
    )
    end_tab.visual(
        _boss_ring_mesh("end_tab_tip_pad", 0.015, inner_radius=0.0055),
        origin=Origin(xyz=(end_tab_length, -LINK_LAYER, 0.0)),
        material=pin_steel,
        name="tip_pad",
    )

    joint_limits = MotionLimits(effort=18.0, velocity=4.0, lower=-1.35, upper=1.35)
    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=links[0],
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=links[0],
        child=links[1],
        origin=Origin(xyz=(link_lengths[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=links[1],
        child=links[2],
        origin=Origin(xyz=(link_lengths[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "joint_3",
        ArticulationType.REVOLUTE,
        parent=links[2],
        child=end_tab,
        origin=Origin(xyz=(link_lengths[2], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("base_pivot"),
        object_model.get_articulation("joint_1"),
        object_model.get_articulation("joint_2"),
        object_model.get_articulation("joint_3"),
    ]
    ctx.check(
        "four parallel revolute joints",
        len(joints) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"joints={[(j.name, j.articulation_type, j.axis) for j in joints]}",
    )

    captured_pin_interfaces = [
        ("cheek", "link_0", "base_pin_shaft", "prox_boss", "base cheek pin is captured through the first link boss"),
        ("link_0", "link_1", "dist_pin_shaft", "prox_boss", "joint 1 pin is captured through the child link boss"),
        ("link_1", "link_2", "dist_pin_shaft", "prox_boss", "joint 2 pin is captured through the child link boss"),
        ("link_2", "end_tab", "dist_pin_shaft", "pivot_boss", "joint 3 pin is captured through the end tab boss"),
    ]
    for parent, child, parent_elem, child_elem, reason in captured_pin_interfaces:
        ctx.allow_overlap(parent, child, elem_a=parent_elem, elem_b=child_elem, reason=reason)
        ctx.allow_overlap(
            parent,
            child,
            elem_a=parent_elem,
            elem_b="plate",
            reason=f"{reason}; the pin shaft also passes through the child plate web.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=parent_elem,
            outer_elem=child_elem,
            margin=0.001,
            name=f"{parent_elem} centered in {child} boss",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=parent_elem,
            elem_b=child_elem,
            min_overlap=0.015,
            name=f"{parent_elem} retained through {child} boss",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=parent_elem,
            elem_b="plate",
            min_overlap=0.008,
            name=f"{parent_elem} retained through {child} plate",
        )

    ctx.expect_gap(
        "link_0",
        "link_1",
        axis="y",
        min_gap=0.004,
        positive_elem="plate",
        negative_elem="plate",
        name="alternating link plates clear at joint 1",
    )
    ctx.expect_gap(
        "link_2",
        "end_tab",
        axis="y",
        min_gap=0.004,
        positive_elem="plate",
        negative_elem="plate",
        name="final link clears end tab layer",
    )
    ctx.expect_contact(
        "link_0",
        "link_1",
        elem_a="dist_pin_neg_head",
        elem_b="prox_boss",
        contact_tol=0.001,
        name="joint 1 pin head bears on child boss",
    )

    rest_pos = ctx.part_world_position("end_tab")
    with ctx.pose({"joint_1": 0.80, "joint_2": -0.55, "joint_3": 0.65}):
        folded_pos = ctx.part_world_position("end_tab")
    ctx.check(
        "articulated chain folds in side plane",
        rest_pos is not None
        and folded_pos is not None
        and abs(folded_pos[2] - rest_pos[2]) > 0.030
        and abs(folded_pos[1] - rest_pos[1]) < 0.002,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
