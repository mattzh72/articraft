from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PLATE_THICKNESS = 0.014
BOSS_PROTRUSION = 0.005
BOSS_EMBED = 0.0005
LINK_RADIUS = 0.034
BOSS_RADIUS = 0.029
PIN_HOLE_RADIUS = 0.014
PIN_RADIUS = 0.010


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _offset_circle_profile(center_x: float, radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [(center_x + x, y) for x, y in _circle_profile(radius, segments)]


def _capsule_profile(length: float, radius: float, arc_segments: int = 18) -> list[tuple[float, float]]:
    """Obround outline in local X/Z profile space, pin centers at x=0 and x=length."""
    points: list[tuple[float, float]] = [(0.0, -radius), (length, -radius)]

    for i in range(1, arc_segments + 1):
        a = -0.5 * math.pi + math.pi * i / arc_segments
        points.append((length + radius * math.cos(a), radius * math.sin(a)))

    points.append((0.0, radius))

    for i in range(1, arc_segments + 1):
        a = 0.5 * math.pi + math.pi * i / arc_segments
        points.append((radius * math.cos(a), radius * math.sin(a)))

    return points


def _boss_ring(center_x: float, *, y_center: float, radius: float = BOSS_RADIUS):
    ring = ExtrudeWithHolesGeometry(
        _circle_profile(radius, 48),
        [_circle_profile(PIN_HOLE_RADIUS, 32)],
        BOSS_PROTRUSION,
        center=True,
    )
    ring.rotate_x(math.pi / 2.0)
    ring.translate(center_x, y_center, 0.0)
    return ring


def _link_plate_mesh(
    *,
    length: float,
    name: str,
    y_offset: float = 0.0,
    end_holes: tuple[tuple[float, float], ...] | None = None,
):
    holes = [(0.0, PIN_HOLE_RADIUS), (length, PIN_HOLE_RADIUS)]
    if end_holes:
        holes.extend(end_holes)

    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(length, LINK_RADIUS),
        [_offset_circle_profile(x, radius, 36) for x, radius in holes],
        PLATE_THICKNESS,
        center=True,
    )
    # ExtrudeWithHolesGeometry extrudes along local Z.  Rotate the mesh so the
    # flat plate thickness lies along object Y and the side profile is X/Z.
    geom.rotate_x(math.pi / 2.0)

    boss_center = PLATE_THICKNESS / 2.0 + BOSS_PROTRUSION / 2.0 - BOSS_EMBED
    for x, radius in holes:
        if radius >= PIN_HOLE_RADIUS * 0.95:
            geom.merge(_boss_ring(x, y_center=boss_center))
            geom.merge(_boss_ring(x, y_center=-boss_center))

    geom.translate(0.0, y_offset, 0.0)
    return mesh_from_geometry(geom, name)


def _pin(part, *, x: float, y_center: float, length: float, name: str, material: str) -> None:
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=length),
        origin=Origin(xyz=(x, y_center, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{name}_shaft",
    )
    cap_radius = 0.018
    cap_len = 0.006
    cap_embed = 0.0004
    part.visual(
        Cylinder(radius=cap_radius, length=cap_len),
        origin=Origin(
            xyz=(x, y_center + length / 2.0 + cap_len / 2.0 - cap_embed, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=f"{name}_cap_0",
    )
    part.visual(
        Cylinder(radius=cap_radius, length=cap_len),
        origin=Origin(
            xyz=(x, y_center - length / 2.0 - cap_len / 2.0 + cap_embed, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=f"{name}_cap_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_revolute_chain")

    model.material("blued_steel", rgba=(0.10, 0.12, 0.14, 1.0))
    model.material("brushed_link", rgba=(0.62, 0.64, 0.61, 1.0))
    model.material("dark_pin", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("oxide_cheek", rgba=(0.18, 0.20, 0.22, 1.0))

    link_0_length = 0.240
    link_1_length = 0.220
    tab_length = 0.130
    offset_link_1 = 0.030

    cheek = model.part("cheek")
    # A compact fixed cheek/yoke: two side plates straddle the first link and
    # are joined by a rear web and a foot, so the first revolute axis is visibly
    # supported rather than floating in space.
    cheek.visual(
        Box((0.104, 0.010, 0.122)),
        origin=Origin(xyz=(0.002, 0.024, 0.000)),
        material="oxide_cheek",
        name="cheek_plate_0",
    )
    cheek.visual(
        Box((0.104, 0.010, 0.122)),
        origin=Origin(xyz=(0.002, -0.024, 0.000)),
        material="oxide_cheek",
        name="cheek_plate_1",
    )
    cheek.visual(
        Box((0.036, 0.058, 0.120)),
        origin=Origin(xyz=(-0.070, 0.000, 0.000)),
        material="oxide_cheek",
        name="rear_web",
    )
    cheek.visual(
        Box((0.158, 0.058, 0.020)),
        origin=Origin(xyz=(-0.022, 0.000, -0.066)),
        material="oxide_cheek",
        name="mount_foot",
    )
    cheek.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(-0.065, 0.031, -0.066), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_pin",
        name="foot_bolt_0",
    )
    cheek.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.022, 0.031, -0.066), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_pin",
        name="foot_bolt_1",
    )
    _pin(cheek, x=0.0, y_center=0.0, length=0.064, name="root_pin", material="dark_pin")

    link_0 = model.part("link_0")
    link_0.visual(
        _link_plate_mesh(length=link_0_length, name="link_0_plate"),
        material="brushed_link",
        name="link_plate",
    )
    _pin(link_0, x=link_0_length, y_center=0.014, length=0.058, name="joint_pin", material="dark_pin")

    link_1 = model.part("link_1")
    link_1.visual(
        _link_plate_mesh(length=link_1_length, name="link_1_plate", y_offset=offset_link_1),
        material="brushed_link",
        name="link_plate",
    )
    _pin(link_1, x=link_1_length, y_center=0.014, length=0.058, name="joint_pin", material="dark_pin")

    end_tab = model.part("end_tab")
    end_tab.visual(
        _link_plate_mesh(
            length=tab_length,
            name="end_tab_plate",
            end_holes=((0.098, 0.007),),
        ),
        material="brushed_link",
        name="tab_plate",
    )

    limits = MotionLimits(effort=12.0, velocity=2.0, lower=-1.25, upper=1.25)
    model.articulation(
        "cheek_to_link",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_to_link",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link_0_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_to_tab",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_tab,
        origin=Origin(xyz=(link_1_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cheek = object_model.get_part("cheek")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_tab = object_model.get_part("end_tab")
    joint_0 = object_model.get_articulation("cheek_to_link")
    joint_1 = object_model.get_articulation("link_to_link")
    joint_2 = object_model.get_articulation("link_to_tab")

    ctx.allow_overlap(
        cheek,
        link_0,
        elem_a="root_pin_shaft",
        elem_b="link_plate",
        reason="The fixed cheek pin is intentionally captured through the first link bearing bore.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="joint_pin_shaft",
        elem_b="link_plate",
        reason="The middle pin is intentionally captured through the offset link bearing bore.",
    )
    ctx.allow_overlap(
        link_1,
        end_tab,
        elem_a="joint_pin_shaft",
        elem_b="tab_plate",
        reason="The end pin is intentionally captured through the end tab bearing bore.",
    )

    ctx.check(
        "three revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "parallel supported axes",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in (joint_0, joint_1, joint_2)),
        details=f"axes={[j.axis for j in (joint_0, joint_1, joint_2)]}",
    )

    ctx.expect_origin_gap(
        link_1,
        link_0,
        axis="x",
        min_gap=0.235,
        max_gap=0.245,
        name="first moving link reaches second pin",
    )
    ctx.expect_origin_gap(
        end_tab,
        link_1,
        axis="x",
        min_gap=0.215,
        max_gap=0.225,
        name="second moving link reaches end tab pin",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        min_gap=0.003,
        positive_elem="link_plate",
        negative_elem="link_plate",
        name="staggered link plates clear at middle joint",
    )
    ctx.expect_gap(
        link_1,
        end_tab,
        axis="y",
        min_gap=0.003,
        positive_elem="link_plate",
        negative_elem="tab_plate",
        name="staggered end tab clears third joint",
    )
    ctx.expect_within(
        cheek,
        link_0,
        axes="xz",
        inner_elem="root_pin_shaft",
        outer_elem="link_plate",
        margin=0.0,
        name="root pin is centered in first bearing boss",
    )
    ctx.expect_within(
        link_0,
        link_1,
        axes="xz",
        inner_elem="joint_pin_shaft",
        outer_elem="link_plate",
        margin=0.0,
        name="middle pin is centered in second bearing boss",
    )
    ctx.expect_within(
        link_1,
        end_tab,
        axes="xz",
        inner_elem="joint_pin_shaft",
        outer_elem="tab_plate",
        margin=0.0,
        name="end pin is centered in tab bearing boss",
    )
    ctx.expect_overlap(
        cheek,
        link_0,
        axes="y",
        elem_a="root_pin_shaft",
        elem_b="link_plate",
        min_overlap=0.010,
        name="root pin spans first link thickness",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="y",
        elem_a="joint_pin_shaft",
        elem_b="link_plate",
        min_overlap=0.010,
        name="middle pin spans offset link thickness",
    )
    ctx.expect_overlap(
        link_1,
        end_tab,
        axes="y",
        elem_a="joint_pin_shaft",
        elem_b="tab_plate",
        min_overlap=0.010,
        name="end pin spans tab thickness",
    )

    rest_aabb = ctx.part_element_world_aabb(end_tab, elem="tab_plate")
    rest_z = None
    if rest_aabb is not None:
        rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    with ctx.pose({joint_2: 0.7}):
        swung_aabb = ctx.part_element_world_aabb(end_tab, elem="tab_plate")
        swung_z = None
        if swung_aabb is not None:
            swung_z = (swung_aabb[0][2] + swung_aabb[1][2]) / 2.0
    ctx.check(
        "end tab swings about third pin",
        rest_z is not None and swung_z is not None and swung_z < rest_z - 0.025,
        details=f"rest_z={rest_z}, swung_z={swung_z}",
    )

    return ctx.report()


object_model = build_object_model()
