from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


PLINTH_SIZE = 2.40
PLINTH_HEIGHT = 0.60

SHAFT_BASE_Z = PLINTH_HEIGHT
SHAFT_TOP_Z = 7.80
SHAFT_BOTTOM_SIZE = 1.82
SHAFT_MID_SIZE = 1.60
SHAFT_TOP_SIZE = 1.42

BELFRY_BASE_Z = 7.80
BELFRY_CORNICE_HEIGHT = 0.18
BELFRY_WALL_BASE_Z = BELFRY_BASE_Z + BELFRY_CORNICE_HEIGHT
BELFRY_WALL_HEIGHT = 1.78
BELFRY_WALL_TOP_Z = BELFRY_WALL_BASE_Z + BELFRY_WALL_HEIGHT
BELFRY_OUTER = 1.54
BELFRY_WALL_THICKNESS = 0.12
BELFRY_FRONT_WIDTH = BELFRY_OUTER - 2.0 * BELFRY_WALL_THICKNESS
BELFRY_TOP_CORNICE_SIZE = 1.62

CLOCK_STAGE_BASE_Z = BELFRY_WALL_TOP_Z + BELFRY_CORNICE_HEIGHT
CLOCK_STAGE_HEIGHT = 0.92
CLOCK_STAGE_TOP_Z = CLOCK_STAGE_BASE_Z + CLOCK_STAGE_HEIGHT
CLOCK_STAGE_SIZE = 1.34

ROOF_BASE_SIZE = 1.18
ROOF_TIP_Z = 12.20

CLOCK_FACE_CENTER_Z = CLOCK_STAGE_BASE_Z + 0.48
CLOCK_STAGE_FRONT_Y = CLOCK_STAGE_SIZE * 0.5


def _square_loop(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _pointed_arch_profile(
    width: float,
    *,
    sill_height: float,
    spring_height: float,
    segments_per_side: int = 10,
) -> list[tuple[float, float]]:
    half_width = width * 0.5
    radius = width
    rise = math.sqrt(max(radius * radius - half_width * half_width, 0.0))
    peak_height = spring_height + rise

    points: list[tuple[float, float]] = [
        (-half_width, sill_height),
        (-half_width, spring_height),
    ]

    right_center_x = half_width
    for index in range(1, segments_per_side):
        angle = math.pi - (math.pi / 3.0) * (index / segments_per_side)
        points.append(
            (
                right_center_x + radius * math.cos(angle),
                spring_height + radius * math.sin(angle),
            )
        )

    points.append((0.0, peak_height))

    left_center_x = -half_width
    for index in range(segments_per_side - 1, 0, -1):
        angle = (math.pi / 3.0) * (index / segments_per_side)
        points.append(
            (
                left_center_x + radius * math.cos(angle),
                spring_height + radius * math.sin(angle),
            )
        )

    points.extend(
        [
            (half_width, spring_height),
            (half_width, sill_height),
        ]
    )
    return points


def _build_belfry_wall_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    mesh_name: str,
    rotate_side: bool = False,
):
    outer = [
        (-width * 0.5, 0.0),
        (width * 0.5, 0.0),
        (width * 0.5, height),
        (-width * 0.5, height),
    ]
    opening = _pointed_arch_profile(
        min(width * 0.52, 0.64),
        sill_height=0.20,
        spring_height=0.96,
    )
    wall = ExtrudeWithHolesGeometry(
        outer_profile=outer,
        hole_profiles=[opening],
        height=thickness,
        cap=True,
        center=True,
        closed=True,
    )
    wall.rotate_x(math.pi / 2.0)
    if rotate_side:
        wall.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(wall, mesh_name)


def _build_flat_plate_mesh(
    profile: list[tuple[float, float]],
    *,
    thickness: float,
    mesh_name: str,
):
    plate = ExtrudeGeometry(
        profile,
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    plate.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(plate, mesh_name)


def _blade_profile(width: float, length: float) -> list[tuple[float, float]]:
    half_root = width * 0.5
    half_shoulder = width * 0.34
    half_neck = width * 0.12
    shoulder_z = length * 0.52
    neck_z = length * 0.88
    return [
        (-half_root, 0.0),
        (half_root, 0.0),
        (half_shoulder, shoulder_z),
        (half_neck, neck_z),
        (0.0, length),
        (-half_neck, neck_z),
        (-half_shoulder, shoulder_z),
    ]


def _tail_profile(width: float, length: float) -> list[tuple[float, float]]:
    half_root = width * 0.5
    half_shoulder = width * 0.30
    shoulder_z = length * 0.54
    return [
        (-half_root, 0.0),
        (half_root, 0.0),
        (half_shoulder, shoulder_z),
        (0.0, length),
        (-half_shoulder, shoulder_z),
    ]


def _add_hand_geometry(
    hand_part,
    *,
    boss_name: str,
    blade_name: str,
    tail_name: str,
    blade_mesh_name: str,
    tail_mesh_name: str,
    material,
    boss_radius: float,
    boss_center_y: float,
    blade_width: float,
    blade_thickness: float,
    blade_length: float,
    tail_width: float,
    tail_length: float,
) -> None:
    hand_part.visual(
        Cylinder(radius=boss_radius, length=blade_thickness),
        origin=Origin(
            xyz=(0.0, boss_center_y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=boss_name,
    )
    hand_part.visual(
        _build_flat_plate_mesh(
            _blade_profile(blade_width, blade_length),
            thickness=blade_thickness,
            mesh_name=blade_mesh_name,
        ),
        origin=Origin(xyz=(0.0, boss_center_y, 0.0)),
        material=material,
        name=blade_name,
    )
    hand_part.visual(
        _build_flat_plate_mesh(
            _tail_profile(tail_width, tail_length),
            thickness=blade_thickness,
            mesh_name=tail_mesh_name,
        ),
        origin=Origin(xyz=(0.0, boss_center_y, 0.0), rpy=(0.0, math.pi, 0.0)),
        material=material,
        name=tail_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gothic_clock_tower")

    stone = model.material("stone", rgba=(0.60, 0.60, 0.58, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.46, 0.46, 0.44, 1.0))
    slate = model.material("slate", rgba=(0.22, 0.23, 0.25, 1.0))
    clock_face = model.material("clock_face", rgba=(0.91, 0.89, 0.82, 1.0))
    metal = model.material("metal", rgba=(0.10, 0.10, 0.11, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.63, 0.54, 0.32, 1.0))

    front_wall_mesh = _build_belfry_wall_mesh(
        width=BELFRY_FRONT_WIDTH,
        height=BELFRY_WALL_HEIGHT,
        thickness=BELFRY_WALL_THICKNESS,
        mesh_name="belfry_front_back_wall",
    )
    side_wall_mesh = _build_belfry_wall_mesh(
        width=BELFRY_OUTER,
        height=BELFRY_WALL_HEIGHT,
        thickness=BELFRY_WALL_THICKNESS,
        mesh_name="belfry_side_wall",
        rotate_side=True,
    )
    clock_bezel_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.405,
            tube=0.016,
            radial_segments=18,
            tubular_segments=48,
        ),
        "clock_bezel_ring_v2",
    )

    tower = model.part("tower")
    tower.visual(
        Box((PLINTH_SIZE, PLINTH_SIZE, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=dark_stone,
        name="plinth",
    )

    shaft_mesh = section_loft(
        [
            _square_loop(SHAFT_BOTTOM_SIZE, SHAFT_BASE_Z),
            _square_loop(SHAFT_MID_SIZE, 4.40),
            _square_loop(SHAFT_TOP_SIZE, SHAFT_TOP_Z),
        ]
    )
    tower.visual(
        mesh_from_geometry(shaft_mesh, "lower_shaft"),
        material=stone,
        name="lower_shaft",
    )
    tower.visual(
        Box((1.96, 1.96, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + 0.04)),
        material=dark_stone,
        name="shaft_stringcourse",
    )
    tower.visual(
        Box((1.62, 1.62, BELFRY_CORNICE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, BELFRY_BASE_Z + BELFRY_CORNICE_HEIGHT * 0.5)
        ),
        material=dark_stone,
        name="belfry_base_cornice",
    )

    front_back_y = BELFRY_OUTER * 0.5 - BELFRY_WALL_THICKNESS * 0.5
    side_x = BELFRY_OUTER * 0.5 - BELFRY_WALL_THICKNESS * 0.5
    tower.visual(
        front_wall_mesh,
        origin=Origin(xyz=(0.0, front_back_y, BELFRY_WALL_BASE_Z)),
        material=stone,
        name="belfry_front",
    )
    tower.visual(
        front_wall_mesh,
        origin=Origin(
            xyz=(0.0, -front_back_y, BELFRY_WALL_BASE_Z),
            rpy=(0.0, 0.0, math.pi),
        ),
        material=stone,
        name="belfry_back",
    )
    tower.visual(
        side_wall_mesh,
        origin=Origin(xyz=(side_x, 0.0, BELFRY_WALL_BASE_Z)),
        material=stone,
        name="belfry_right",
    )
    tower.visual(
        side_wall_mesh,
        origin=Origin(
            xyz=(-side_x, 0.0, BELFRY_WALL_BASE_Z),
            rpy=(0.0, 0.0, math.pi),
        ),
        material=stone,
        name="belfry_left",
    )

    tower.visual(
        Box(
            (
                BELFRY_TOP_CORNICE_SIZE,
                BELFRY_TOP_CORNICE_SIZE,
                BELFRY_CORNICE_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(0.0, 0.0, BELFRY_WALL_TOP_Z + BELFRY_CORNICE_HEIGHT * 0.5)
        ),
        material=dark_stone,
        name="belfry_top_cornice",
    )
    tower.visual(
        Box((CLOCK_STAGE_SIZE, CLOCK_STAGE_SIZE, CLOCK_STAGE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, CLOCK_STAGE_BASE_Z + CLOCK_STAGE_HEIGHT * 0.5)
        ),
        material=stone,
        name="clock_stage",
    )

    roof_mesh = section_loft(
        [
            _square_loop(ROOF_BASE_SIZE, CLOCK_STAGE_TOP_Z),
            _square_loop(0.60, 11.45),
            _square_loop(0.08, ROOF_TIP_Z),
        ]
    )
    tower.visual(
        mesh_from_geometry(roof_mesh, "roof_spire"),
        material=slate,
        name="roof_spire",
    )
    tower.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, ROOF_TIP_Z + 0.09)),
        material=aged_brass,
        name="spire_finial_stem",
    )
    tower.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.0, 0.0, ROOF_TIP_Z + 0.235)),
        material=aged_brass,
        name="spire_finial",
    )

    dial_axis_rotation = (math.pi / 2.0, 0.0, 0.0)
    tower.visual(
        Cylinder(radius=0.40, length=0.018),
        origin=Origin(
            xyz=(0.0, CLOCK_STAGE_FRONT_Y + 0.009, CLOCK_FACE_CENTER_Z),
            rpy=dial_axis_rotation,
        ),
        material=clock_face,
        name="clock_face",
    )
    tower.visual(
        clock_bezel_mesh,
        origin=Origin(
            xyz=(0.0, CLOCK_STAGE_FRONT_Y + 0.009, CLOCK_FACE_CENTER_Z),
            rpy=dial_axis_rotation,
        ),
        material=dark_stone,
        name="clock_bezel",
    )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        marker_radius = 0.325
        quarter = index % 3 == 0
        marker_width = 0.022 if quarter else 0.014
        marker_length = 0.080 if quarter else 0.045
        tower.visual(
            Box((marker_width, 0.004, marker_length)),
            origin=Origin(
                xyz=(
                    math.sin(angle) * marker_radius,
                    CLOCK_STAGE_FRONT_Y + 0.019,
                    CLOCK_FACE_CENTER_Z + math.cos(angle) * marker_radius,
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=aged_brass if quarter else metal,
            name=f"clock_marker_{index:02d}",
        )
    tower.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(
            xyz=(0.0, CLOCK_STAGE_FRONT_Y + 0.028, CLOCK_FACE_CENTER_Z),
            rpy=dial_axis_rotation,
        ),
        material=metal,
        name="clock_spindle",
    )
    tower.inertial = Inertial.from_geometry(
        Box((PLINTH_SIZE, PLINTH_SIZE, ROOF_TIP_Z + 0.30)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, (ROOF_TIP_Z + 0.30) * 0.5)),
    )

    hour_hand = model.part("hour_hand")
    _add_hand_geometry(
        hour_hand,
        boss_name="hour_boss",
        blade_name="hour_blade",
        tail_name="hour_tail",
        blade_mesh_name="hour_blade_mesh",
        tail_mesh_name="hour_tail_mesh",
        material=metal,
        boss_radius=0.028,
        boss_center_y=0.006,
        blade_width=0.060,
        blade_thickness=0.004,
        blade_length=0.22,
        tail_width=0.030,
        tail_length=0.06,
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.060, 0.004, 0.28)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.006, 0.08)),
    )

    minute_hand = model.part("minute_hand")
    _add_hand_geometry(
        minute_hand,
        boss_name="minute_boss",
        blade_name="minute_blade",
        tail_name="minute_tail",
        blade_mesh_name="minute_blade_mesh",
        tail_mesh_name="minute_tail_mesh",
        material=metal,
        boss_radius=0.022,
        boss_center_y=0.010,
        blade_width=0.042,
        blade_thickness=0.004,
        blade_length=0.31,
        tail_width=0.022,
        tail_length=0.08,
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.042, 0.004, 0.39)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.010, 0.115)),
    )

    clock_hub_origin = Origin(
        xyz=(0.0, CLOCK_STAGE_FRONT_Y + 0.034, CLOCK_FACE_CENTER_Z)
    )
    full_turn = 2.0 * math.pi
    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hour_hand,
        origin=clock_hub_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.30,
            lower=0.0,
            upper=full_turn,
        ),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=minute_hand,
        origin=clock_hub_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.45,
            lower=0.0,
            upper=full_turn,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    clock_face = tower.get_visual("clock_face")
    clock_spindle = tower.get_visual("clock_spindle")
    hour_boss = hour_hand.get_visual("hour_boss")
    minute_boss = minute_hand.get_visual("minute_boss")
    hour_blade = hour_hand.get_visual("hour_blade")
    minute_blade = minute_hand.get_visual("minute_blade")

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

    tower_aabb = ctx.part_world_aabb(tower)
    if tower_aabb is None:
        ctx.fail("tower_geometry_present", "tower part has no measurable geometry")
    else:
        tower_size_x = tower_aabb[1][0] - tower_aabb[0][0]
        tower_size_y = tower_aabb[1][1] - tower_aabb[0][1]
        tower_size_z = tower_aabb[1][2] - tower_aabb[0][2]
        ctx.check(
            "tower_height_realistic",
            12.0 <= tower_size_z <= 13.0,
            f"tower height {tower_size_z:.3f} m is outside the expected range",
        )
        ctx.check(
            "tower_footprint_realistic",
            2.25 <= tower_size_x <= 2.55 and 2.25 <= tower_size_y <= 2.55,
            (
                "tower footprint is outside the expected range: "
                f"{tower_size_x:.3f} x {tower_size_y:.3f} m"
            ),
        )

    face_aabb = ctx.part_element_world_aabb(tower, elem=clock_face.name)
    if face_aabb is None:
        ctx.fail("clock_face_present", "clock face visual is missing")
    else:
        face_center_z = 0.5 * (face_aabb[0][2] + face_aabb[1][2])
        face_front_y = face_aabb[1][1]
        ctx.check(
            "clock_face_near_top",
            10.1 <= face_center_z <= 10.5 and face_front_y > 0.67,
            (
                "clock face is not mounted on the upper front stage: "
                f"z={face_center_z:.3f}, front_y={face_front_y:.3f}"
            ),
        )

    ctx.check(
        "hour_joint_axis",
        tuple(hour_joint.axis) == (0.0, 1.0, 0.0),
        f"hour hand axis should face out of the wall, got {hour_joint.axis}",
    )
    ctx.check(
        "minute_joint_axis",
        tuple(minute_joint.axis) == (0.0, 1.0, 0.0),
        f"minute hand axis should face out of the wall, got {minute_joint.axis}",
    )
    ctx.check(
        "hand_joints_share_hub",
        tuple(hour_joint.origin.xyz) == tuple(minute_joint.origin.xyz),
        (
            "hour and minute hands should be concentric at the face hub, got "
            f"{hour_joint.origin.xyz} and {minute_joint.origin.xyz}"
        ),
    )

    with ctx.pose({hour_joint: 0.0, minute_joint: 0.0}):
        ctx.expect_contact(
            hour_hand,
            tower,
            elem_a=hour_boss,
            elem_b=clock_spindle,
            contact_tol=1e-5,
            name="hour_boss_contacts_spindle",
        )
        ctx.expect_overlap(
            hour_hand,
            tower,
            axes="xz",
            elem_a=hour_boss,
            elem_b=clock_spindle,
            min_overlap=0.04,
            name="hour_boss_centered_on_spindle",
        )
        ctx.expect_contact(
            minute_hand,
            hour_hand,
            elem_a=minute_boss,
            elem_b=hour_boss,
            contact_tol=1e-5,
            name="minute_boss_contacts_hour_boss",
        )
        ctx.expect_overlap(
            minute_hand,
            hour_hand,
            axes="xz",
            elem_a=minute_boss,
            elem_b=hour_boss,
            min_overlap=0.035,
            name="minute_boss_centered_on_hour_boss",
        )
        ctx.expect_gap(
            minute_hand,
            hour_hand,
            axis="y",
            positive_elem=minute_boss,
            negative_elem=hour_boss,
            max_gap=0.0,
            max_penetration=0.0,
            name="boss_stack_is_flush",
        )
        ctx.expect_gap(
            minute_hand,
            hour_hand,
            axis="y",
            positive_elem=minute_blade,
            negative_elem=hour_blade,
            min_gap=0.0,
            max_gap=0.01,
            name="hand_blades_are_layered",
        )

    with ctx.pose({hour_joint: math.pi / 2.0, minute_joint: math.pi}):
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="quarter_turn_pose_no_floating")
        ctx.expect_contact(
            hour_hand,
            tower,
            elem_a=hour_boss,
            elem_b=clock_spindle,
            contact_tol=1e-5,
            name="quarter_turn_hour_contact",
        )
        ctx.expect_contact(
            minute_hand,
            hour_hand,
            elem_a=minute_boss,
            elem_b=hour_boss,
            contact_tol=1e-5,
            name="quarter_turn_minute_contact",
        )

    for articulation, prefix in (
        (hour_joint, "hour_hand"),
        (minute_joint, "minute_hand"),
    ):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{prefix}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{prefix}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{prefix}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{prefix}_upper_no_floating")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=False,
        ignore_fixed=False,
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=20,
        name="clock_sampled_pose_no_floating",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
