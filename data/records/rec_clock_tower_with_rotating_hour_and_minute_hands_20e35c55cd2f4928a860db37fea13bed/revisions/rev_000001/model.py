from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _square_pyramid(base_half: float, height: float) -> MeshGeometry:
    roof = MeshGeometry()
    v0 = roof.add_vertex(-base_half, -base_half, 0.0)
    v1 = roof.add_vertex(base_half, -base_half, 0.0)
    v2 = roof.add_vertex(base_half, base_half, 0.0)
    v3 = roof.add_vertex(-base_half, base_half, 0.0)
    apex = roof.add_vertex(0.0, 0.0, height)

    roof.add_face(v0, v2, v1)
    roof.add_face(v0, v3, v2)
    roof.add_face(v0, v1, apex)
    roof.add_face(v1, v2, apex)
    roof.add_face(v2, v3, apex)
    roof.add_face(v3, v0, apex)
    return roof

def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swiss_town_hall_clock_tower")

    stone = model.material("stone", rgba=(0.78, 0.75, 0.70, 1.0))
    trim_stone = model.material("trim_stone", rgba=(0.68, 0.65, 0.61, 1.0))
    roof_tile = model.material("roof_tile", rgba=(0.58, 0.25, 0.16, 1.0))
    dial = model.material("dial", rgba=(0.94, 0.92, 0.84, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.15, 0.16, 0.18, 1.0))

    shaft_width = 9.0
    shaft_depth = 9.0
    shaft_height = 25.6
    plinth_width = 9.8
    plinth_height = 1.2
    cornice_lower_size = (9.7, 9.7, 0.45)
    cornice_upper_size = (10.5, 10.5, 0.65)
    roof_base_z = shaft_height + cornice_lower_size[2] + cornice_upper_size[2]
    roof_height = 6.1
    turret_radius = 1.15
    turret_height = 6.2
    turret_bottom_z = shaft_height - turret_height
    turret_center_z = turret_bottom_z + turret_height * 0.5
    turret_cap_height = 0.24
    turret_cap_z = roof_base_z - turret_cap_height * 0.5
    turret_offset = shaft_width * 0.5 - 0.45
    clock_center_z = 19.2
    face_thickness = 0.12
    face_radius = 1.48
    bezel_radius = 1.62
    south_wall_y = -shaft_depth * 0.5

    tower = model.part("tower")
    tower.visual(
        Box((plinth_width, plinth_width, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=trim_stone,
        name="plinth",
    )
    tower.visual(
        Box((shaft_width, shaft_depth, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, shaft_height * 0.5)),
        material=stone,
        name="shaft",
    )
    tower.visual(
        Box(cornice_lower_size),
        origin=Origin(
            xyz=(0.0, 0.0, shaft_height + cornice_lower_size[2] * 0.5),
        ),
        material=trim_stone,
        name="cornice_lower",
    )
    tower.visual(
        Box(cornice_upper_size),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                shaft_height + cornice_lower_size[2] + cornice_upper_size[2] * 0.5,
            ),
        ),
        material=trim_stone,
        name="cornice_upper",
    )

    for side_name, x_sign in (("west", -1.0), ("east", 1.0)):
        tower.visual(
            Cylinder(radius=turret_radius, length=turret_height),
            origin=Origin(
                xyz=(x_sign * turret_offset, -turret_offset, turret_center_z),
            ),
            material=stone,
            name=f"{side_name}_turret",
        )
        tower.visual(
            Cylinder(radius=turret_radius * 1.10, length=turret_cap_height),
            origin=Origin(
                xyz=(x_sign * turret_offset, -turret_offset, turret_cap_z),
            ),
            material=trim_stone,
            name=f"{side_name}_turret_cap",
        )

    roof_mesh = mesh_from_geometry(
        _square_pyramid(base_half=5.35, height=roof_height),
        "tower_pyramid_roof",
    )
    tower.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, roof_base_z)),
        material=roof_tile,
        name="roof",
    )

    clock_face = model.part("clock_face")
    clock_face.visual(
        Cylinder(radius=bezel_radius, length=0.10),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="bezel",
    )
    clock_face.visual(
        Cylinder(radius=face_radius, length=face_thickness),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial,
        name="dial",
    )

    marker_depth = 0.018
    marker_y = -(face_thickness * 0.5 + marker_depth * 0.5)
    marker_long = 0.34
    marker_short = 0.10
    marker_span = 1.08
    clock_face.visual(
        Box((marker_long, marker_depth, marker_short)),
        origin=Origin(xyz=(0.0, marker_y, marker_span)),
        material=dark_metal,
        name="marker_12",
    )
    clock_face.visual(
        Box((marker_short, marker_depth, marker_long)),
        origin=Origin(xyz=(marker_span, marker_y, 0.0)),
        material=dark_metal,
        name="marker_3",
    )
    clock_face.visual(
        Box((marker_long, marker_depth, marker_short)),
        origin=Origin(xyz=(0.0, marker_y, -marker_span)),
        material=dark_metal,
        name="marker_6",
    )
    clock_face.visual(
        Box((marker_short, marker_depth, marker_long)),
        origin=Origin(xyz=(-marker_span, marker_y, 0.0)),
        material=dark_metal,
        name="marker_9",
    )

    model.articulation(
        "tower_to_clock_face",
        ArticulationType.FIXED,
        parent=tower,
        child=clock_face,
        origin=Origin(
            xyz=(0.0, south_wall_y - face_thickness * 0.5, clock_center_z),
        ),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.08, length=0.06),
        origin=Origin(xyz=(0.0, -0.09, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="arbor",
    )
    hour_hand.visual(
        Box((0.05, 0.020, 0.24)),
        origin=Origin(xyz=(0.0, -0.09, 0.20)),
        material=dark_metal,
        name="blade_neck",
    )
    hour_hand.visual(
        Box((0.10, 0.020, 0.34)),
        origin=Origin(xyz=(0.0, -0.09, 0.49)),
        material=dark_metal,
        name="blade_mid",
    )
    hour_hand.visual(
        Box((0.04, 0.020, 0.24)),
        origin=Origin(xyz=(0.0, -0.09, 0.78)),
        material=dark_metal,
        name="blade_tip",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.10, length=0.024),
        origin=Origin(xyz=(0.0, -0.132, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    minute_hand.visual(
        Box((0.04, 0.024, 0.34)),
        origin=Origin(xyz=(0.0, -0.156, 0.27)),
        material=dark_metal,
        name="blade_neck",
    )
    minute_hand.visual(
        Box((0.075, 0.024, 0.44)),
        origin=Origin(xyz=(0.0, -0.156, 0.66)),
        material=dark_metal,
        name="blade_mid",
    )
    minute_hand.visual(
        Box((0.03, 0.024, 0.40)),
        origin=Origin(xyz=(0.0, -0.156, 1.08)),
        material=dark_metal,
        name="blade_tip",
    )
    minute_hand.visual(
        Box((0.025, 0.024, 0.14)),
        origin=Origin(xyz=(0.0, -0.156, -0.03)),
        material=dark_metal,
        name="counterweight",
    )

    full_turn = 2.0 * pi
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=0.0,
            upper=full_turn,
        ),
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=full_turn,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    face_joint = object_model.get_articulation("tower_to_clock_face")
    hour_joint = object_model.get_articulation("clock_face_to_hour_hand")
    minute_joint = object_model.get_articulation("clock_face_to_minute_hand")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "clock tower parts present",
        all(part is not None for part in (tower, clock_face, hour_hand, minute_hand)),
        details=str([part.name for part in (tower, clock_face, hour_hand, minute_hand)]),
    )
    ctx.check(
        "clock face mounted as fixed subassembly",
        face_joint.articulation_type == ArticulationType.FIXED
        and face_joint.parent == "tower"
        and face_joint.child == "clock_face",
        details=f"type={face_joint.articulation_type}, parent={face_joint.parent}, child={face_joint.child}",
    )
    ctx.check(
        "clock hands use concentric south-facing revolute axes",
        hour_joint.articulation_type == ArticulationType.REVOLUTE
        and minute_joint.articulation_type == ArticulationType.REVOLUTE
        and hour_joint.axis == (0.0, 1.0, 0.0)
        and minute_joint.axis == (0.0, 1.0, 0.0)
        and hour_joint.origin.xyz == (0.0, 0.0, 0.0)
        and minute_joint.origin.xyz == (0.0, 0.0, 0.0),
        details=(
            f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}, "
            f"hour_origin={hour_joint.origin.xyz}, minute_origin={minute_joint.origin.xyz}"
        ),
    )

    ctx.expect_contact(
        clock_face,
        tower,
        contact_tol=1e-4,
        name="clock face touches the south tower wall",
    )
    ctx.expect_contact(
        hour_hand,
        clock_face,
        elem_a="arbor",
        elem_b="dial",
        contact_tol=1e-6,
        name="hour hand arbor bears on the dial center",
    )
    ctx.expect_contact(
        minute_hand,
        hour_hand,
        elem_a="hub",
        elem_b="arbor",
        contact_tol=1e-6,
        name="minute hand hub stacks onto the hour arbor",
    )
    ctx.expect_origin_gap(
        tower,
        clock_face,
        axis="y",
        min_gap=4.5,
        max_gap=4.7,
        name="clock face projects from the south face",
    )
    ctx.expect_overlap(
        clock_face,
        tower,
        axes="xz",
        min_overlap=2.8,
        name="clock face sits within the upper shaft frontage",
    )
    ctx.expect_gap(
        clock_face,
        hour_hand,
        axis="y",
        min_gap=0.008,
        max_gap=0.03,
        positive_elem="dial",
        negative_elem="blade_neck",
        name="hour hand stands proud of the dial",
    )
    ctx.expect_gap(
        clock_face,
        minute_hand,
        axis="y",
        min_gap=0.010,
        max_gap=0.10,
        positive_elem="dial",
        negative_elem="blade_neck",
        name="minute hand clears the dial with realistic standoff",
    )

    rest_hour_center = _aabb_center(ctx.part_world_aabb(hour_hand))
    with ctx.pose({hour_joint: pi / 2.0}):
        turned_hour_center = _aabb_center(ctx.part_world_aabb(hour_hand))
    ctx.check(
        "hour hand rotates about the hub",
        rest_hour_center is not None
        and turned_hour_center is not None
        and hypot(
            rest_hour_center[0] - turned_hour_center[0],
            rest_hour_center[2] - turned_hour_center[2],
        )
        > 0.25,
        details=f"rest={rest_hour_center}, turned={turned_hour_center}",
    )

    rest_minute_center = _aabb_center(ctx.part_world_aabb(minute_hand))
    with ctx.pose({minute_joint: pi / 2.0}):
        turned_minute_center = _aabb_center(ctx.part_world_aabb(minute_hand))
    ctx.check(
        "minute hand rotates independently about the hub",
        rest_minute_center is not None
        and turned_minute_center is not None
        and hypot(
            rest_minute_center[0] - turned_minute_center[0],
            rest_minute_center[2] - turned_minute_center[2],
        )
        > 0.30,
        details=f"rest={rest_minute_center}, turned={turned_minute_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
