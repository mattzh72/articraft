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
    TorusGeometry,
    mesh_from_geometry,
)


LINK_0_LENGTH = 0.180
LINK_1_LENGTH = 0.160
LINK_2_LENGTH = 0.130
LINK_LANE_SIDE = 0.055
PIN_CENTER_Y = LINK_LANE_SIDE * 0.5


def _circle_profile(
    radius: float,
    center: tuple[float, float] = (0.0, 0.0),
    *,
    segments: int = 48,
    reverse: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    points = [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    if reverse:
        points.reverse()
    return points


def _capsule_profile(
    length: float,
    radius: float,
    *,
    arc_segments: int = 24,
) -> list[tuple[float, float]]:
    """Vertical obround profile with hinge centers at local y=0 and y=length."""
    points: list[tuple[float, float]] = []
    # Top half: right tangent -> top crown -> left tangent.
    for i in range(arc_segments + 1):
        angle = -math.pi * i / arc_segments
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    # Bottom half: left tangent -> lower crown -> right tangent.
    points.append((-radius, length))
    for i in range(1, arc_segments + 1):
        angle = math.pi - math.pi * i / arc_segments
        points.append(
            (radius * math.cos(angle), length + radius * math.sin(angle))
        )
    return points


def _lever_plate_geometry(
    length: float,
    *,
    outer_radius: float,
    hole_radius: float,
    thickness: float,
    lane_y: float,
) -> ExtrudeWithHolesGeometry:
    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(length, outer_radius),
        [
            _circle_profile(hole_radius, (0.0, 0.0)),
            _circle_profile(hole_radius, (0.0, length)),
        ],
        thickness,
        center=True,
    )
    # Mesh profiles are authored in local XY and extruded along local Z. Rotate
    # the extrusion axis onto world Y so each plate is a flat strap around a
    # horizontal hinge pin, while local +Y becomes the hanging -Z direction.
    geom.rotate_x(-math.pi / 2.0)
    geom.translate(0.0, lane_y, 0.0)
    return geom


def _pin_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _hole_ring_geometry(
    *,
    lane_y: float,
    z: float,
    ring_radius: float = 0.010,
    tube_radius: float = 0.0015,
) -> TorusGeometry:
    geom = TorusGeometry(radius=ring_radius, tube=tube_radius, radial_segments=18, tubular_segments=36)
    geom.rotate_x(-math.pi / 2.0)
    geom.translate(0.0, lane_y, z)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_lever_chain")

    bracket_paint = model.material("dark_blue_painted_steel", rgba=(0.05, 0.08, 0.12, 1.0))
    plate_metal = model.material("brushed_zinc_link", rgba=(0.58, 0.60, 0.57, 1.0))
    pin_metal = model.material("polished_pin_steel", rgba=(0.82, 0.84, 0.80, 1.0))
    tab_metal = model.material("small_bare_end_tab", rgba=(0.72, 0.66, 0.52, 1.0))
    dark_bushing = model.material("dark_hole_bushing", rgba=(0.015, 0.014, 0.012, 1.0))

    support = model.part("support_bracket")
    support.visual(
        Box((0.180, 0.130, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=bracket_paint,
        name="top_plate",
    )
    for y in (-0.047, 0.047):
        support.visual(
            Box((0.052, 0.016, 0.095)),
            origin=Origin(xyz=(0.0, y, 0.032)),
            material=bracket_paint,
            name=f"side_cheek_{'neg' if y < 0.0 else 'pos'}",
        )
        support.visual(
            Cylinder(radius=0.027, length=0.016),
            origin=_pin_origin(0.0, y, 0.0),
            material=bracket_paint,
            name=f"cheek_boss_{'neg' if y < 0.0 else 'pos'}",
        )
    support.visual(
        Cylinder(radius=0.006, length=0.146),
        origin=_pin_origin(0.0, 0.0, 0.0),
        material=pin_metal,
        name="support_pin",
    )
    for y in (-0.073, 0.073):
        support.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=_pin_origin(0.0, y, 0.0),
            material=pin_metal,
            name=f"support_pin_head_{'neg' if y < 0.0 else 'pos'}",
        )
    for x in (-0.060, 0.060):
        for y in (-0.040, 0.040):
            support.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=Origin(xyz=(x, y, 0.0965)),
                material=pin_metal,
                name=f"bolt_{'negx' if x < 0.0 else 'posx'}_{'negy' if y < 0.0 else 'posy'}",
            )

    link_0 = model.part("link_0")
    link_0.visual(
        mesh_from_geometry(
            _lever_plate_geometry(
                LINK_0_LENGTH,
                outer_radius=0.024,
                hole_radius=0.010,
                thickness=0.018,
                lane_y=0.0,
            ),
            "link_0_plate",
        ),
        material=plate_metal,
        name="link_plate",
    )
    link_0.visual(
        mesh_from_geometry(_hole_ring_geometry(lane_y=0.0, z=0.0), "link_0_top_bushing"),
        material=dark_bushing,
        name="top_bushing",
    )
    link_0.visual(
        mesh_from_geometry(
            _hole_ring_geometry(lane_y=0.0, z=-LINK_0_LENGTH),
            "link_0_lower_bushing",
        ),
        material=dark_bushing,
        name="lower_bushing",
    )
    link_0.visual(
        Cylinder(radius=0.006, length=0.132),
        origin=_pin_origin(0.0, PIN_CENTER_Y, -LINK_0_LENGTH),
        material=pin_metal,
        name="lower_pin",
    )
    for y in (-0.038, 0.093):
        link_0.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=_pin_origin(0.0, y, -LINK_0_LENGTH),
            material=pin_metal,
            name=f"lower_pin_head_{'neg' if y < 0.0 else 'pos'}",
        )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_geometry(
            _lever_plate_geometry(
                LINK_1_LENGTH,
                outer_radius=0.023,
                hole_radius=0.010,
                thickness=0.018,
                lane_y=LINK_LANE_SIDE,
            ),
            "link_1_plate",
        ),
        material=plate_metal,
        name="link_plate",
    )
    link_1.visual(
        mesh_from_geometry(
            _hole_ring_geometry(lane_y=LINK_LANE_SIDE, z=0.0),
            "link_1_top_bushing",
        ),
        material=dark_bushing,
        name="top_bushing",
    )
    link_1.visual(
        mesh_from_geometry(
            _hole_ring_geometry(lane_y=LINK_LANE_SIDE, z=-LINK_1_LENGTH),
            "link_1_lower_bushing",
        ),
        material=dark_bushing,
        name="lower_bushing",
    )
    link_1.visual(
        Cylinder(radius=0.006, length=0.132),
        origin=_pin_origin(0.0, PIN_CENTER_Y, -LINK_1_LENGTH),
        material=pin_metal,
        name="lower_pin",
    )
    for y in (-0.038, 0.093):
        link_1.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=_pin_origin(0.0, y, -LINK_1_LENGTH),
            material=pin_metal,
            name=f"lower_pin_head_{'neg' if y < 0.0 else 'pos'}",
        )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_geometry(
            _lever_plate_geometry(
                LINK_2_LENGTH,
                outer_radius=0.018,
                hole_radius=0.008,
                thickness=0.016,
                lane_y=0.0,
            ),
            "link_2_plate",
        ),
        material=plate_metal,
        name="link_plate",
    )
    link_2.visual(
        mesh_from_geometry(
            _hole_ring_geometry(lane_y=0.0, z=0.0, ring_radius=0.0088, tube_radius=0.0012),
            "link_2_top_bushing",
        ),
        material=dark_bushing,
        name="top_bushing",
    )
    link_2.visual(
        mesh_from_geometry(
            _hole_ring_geometry(
                lane_y=0.0,
                z=-LINK_2_LENGTH,
                ring_radius=0.0072,
                tube_radius=0.0010,
            ),
            "link_2_tab_bushing",
        ),
        material=dark_bushing,
        name="tab_bushing",
    )
    link_2.visual(
        Box((0.045, 0.016, 0.016)),
        origin=Origin(xyz=(0.022, 0.0, -LINK_2_LENGTH - 0.018)),
        material=tab_metal,
        name="end_tab_neck",
    )
    link_2.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=_pin_origin(0.047, 0.0, -LINK_2_LENGTH - 0.018),
        material=tab_metal,
        name="end_tab_tip",
    )

    model.articulation(
        "support_to_link_0",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, -LINK_0_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -LINK_1_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    joint_0 = object_model.get_articulation("support_to_link_0")
    joint_1 = object_model.get_articulation("link_0_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")

    ctx.allow_overlap(
        "link_0",
        "support_bracket",
        elem_a="link_plate",
        elem_b="support_pin",
        reason="The top hinge pin is intentionally captured through the first link eye.",
    )
    ctx.allow_overlap(
        "link_0",
        "link_1",
        elem_a="lower_pin",
        elem_b="link_plate",
        reason="The middle hinge pin is intentionally captured through the second link eye.",
    )
    ctx.allow_overlap(
        "link_1",
        "link_2",
        elem_a="lower_pin",
        elem_b="link_plate",
        reason="The lower hinge pin is intentionally captured through the third link eye.",
    )

    hinge_joints = (joint_0, joint_1, joint_2)
    ctx.check(
        "three parallel revolute hinge axes",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in hinge_joints)
        and all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in hinge_joints),
        details=f"joints={[(j.name, j.articulation_type, j.axis) for j in object_model.articulations]}",
    )

    ctx.expect_origin_gap(
        link_0,
        link_1,
        axis="z",
        min_gap=LINK_0_LENGTH - 0.002,
        max_gap=LINK_0_LENGTH + 0.002,
        name="second link hangs from the first lower hinge",
    )
    ctx.expect_origin_gap(
        link_1,
        link_2,
        axis="z",
        min_gap=LINK_1_LENGTH - 0.002,
        max_gap=LINK_1_LENGTH + 0.002,
        name="third link hangs from the second lower hinge",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="y",
        min_gap=0.025,
        positive_elem="link_plate",
        negative_elem="link_plate",
        name="first and second link plates are axially spaced on their pin",
    )
    ctx.expect_gap(
        link_1,
        link_2,
        axis="y",
        min_gap=0.025,
        positive_elem="link_plate",
        negative_elem="link_plate",
        name="second and third link plates are axially spaced on their pin",
    )
    ctx.expect_within(
        "support_bracket",
        link_0,
        axes="xz",
        inner_elem="support_pin",
        outer_elem="link_plate",
        margin=0.002,
        name="top pin is centered in the first link eye",
    )
    ctx.expect_within(
        link_0,
        link_1,
        axes="xz",
        inner_elem="lower_pin",
        outer_elem="link_plate",
        margin=0.002,
        name="middle pin is centered in the second link eye",
    )
    ctx.expect_within(
        link_1,
        link_2,
        axes="xz",
        inner_elem="lower_pin",
        outer_elem="link_plate",
        margin=0.002,
        name="lower pin is centered in the third link eye",
    )

    rest_link_1 = ctx.part_world_position(link_1)
    with ctx.pose({joint_0: 0.42}):
        swung_link_1 = ctx.part_world_position(link_1)
    ctx.check(
        "top hinge swings the hanging chain",
        rest_link_1 is not None
        and swung_link_1 is not None
        and swung_link_1[0] < rest_link_1[0] - 0.060,
        details=f"rest={rest_link_1}, swung={swung_link_1}",
    )

    rest_link_2 = ctx.part_world_position(link_2)
    with ctx.pose({joint_1: 0.42}):
        swung_link_2 = ctx.part_world_position(link_2)
    ctx.check(
        "middle hinge swings the lower link",
        rest_link_2 is not None
        and swung_link_2 is not None
        and swung_link_2[0] < rest_link_2[0] - 0.052,
        details=f"rest={rest_link_2}, swung={swung_link_2}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(link_2, elem="end_tab_tip")
    with ctx.pose({joint_2: 0.42}):
        swung_tab_aabb = ctx.part_element_world_aabb(link_2, elem="end_tab_tip")
    if rest_tab_aabb is not None and swung_tab_aabb is not None:
        rest_tab_x = (rest_tab_aabb[0][0] + rest_tab_aabb[1][0]) / 2.0
        swung_tab_x = (swung_tab_aabb[0][0] + swung_tab_aabb[1][0]) / 2.0
    else:
        rest_tab_x = swung_tab_x = None
    ctx.check(
        "lower hinge rotates the small end tab",
        rest_tab_x is not None and swung_tab_x is not None and swung_tab_x < rest_tab_x - 0.040,
        details=f"rest_x={rest_tab_x}, swung_x={swung_tab_x}",
    )

    return ctx.report()


object_model = build_object_model()
