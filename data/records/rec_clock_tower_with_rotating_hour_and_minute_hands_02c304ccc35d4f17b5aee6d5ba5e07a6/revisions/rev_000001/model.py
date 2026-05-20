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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _pointed_arch_profile(
    *,
    width: float,
    base_y: float,
    rect_height: float,
    apex_y: float,
    shoulder_segments: int = 4,
) -> list[tuple[float, float]]:
    half_w = width * 0.5
    spring_y = base_y + rect_height
    profile: list[tuple[float, float]] = [
        (half_w, base_y),
        (half_w, spring_y),
    ]
    for step in range(1, shoulder_segments):
        t = step / shoulder_segments
        x = half_w * (1.0 - 0.55 * t)
        y = spring_y + (apex_y - spring_y) * (t * t)
        profile.append((x, y))
    profile.append((0.0, apex_y))
    for step in range(shoulder_segments - 1, 0, -1):
        t = step / shoulder_segments
        x = -half_w * (1.0 - 0.55 * t)
        y = spring_y + (apex_y - spring_y) * (t * t)
        profile.append((x, y))
    profile.extend(
        [
            (-half_w, spring_y),
            (-half_w, base_y),
        ]
    )
    return profile


def _square_loop(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _build_belfry_wall_mesh(*, width: float, height: float, thickness: float):
    outer = [
        (-width * 0.5, -height * 0.5),
        (width * 0.5, -height * 0.5),
        (width * 0.5, height * 0.5),
        (-width * 0.5, height * 0.5),
    ]
    opening = _pointed_arch_profile(
        width=1.65,
        base_y=-height * 0.5 + 0.90,
        rect_height=2.05,
        apex_y=-height * 0.5 + 4.10,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [opening],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "belfry_wall")


def _build_roof_mesh(base_size: float, roof_height: float):
    geom = section_loft(
        [
            _square_loop(base_size, 0.0),
            _square_loop(base_size * 0.56, roof_height * 0.58),
            _square_loop(base_size * 0.08, roof_height),
        ]
    )
    return mesh_from_geometry(geom, "tower_roof")


def _build_bell_mesh():
    geom = LatheGeometry(
        [
            (0.02, 0.28),
            (0.10, 0.27),
            (0.18, 0.23),
            (0.29, 0.06),
            (0.41, -0.26),
            (0.51, -0.62),
            (0.60, -0.88),
            (0.68, -1.00),
            (0.54, -1.06),
            (0.00, -1.06),
        ],
        segments=56,
    )
    return mesh_from_geometry(geom, "tower_bell")


def _add_clock_face(
    model: ArticulatedObject,
    tower,
    *,
    face_name: str,
    yaw: float,
    face_center: tuple[float, float, float],
    dial_material,
    frame_material,
    hand_material,
) -> None:
    dial = model.part(f"{face_name}_dial")
    dial.visual(
        Box((1.94, 0.18, 1.94)),
        origin=Origin(xyz=(0.0, 0.09, 0.0)),
        material=frame_material,
        name="dial_frame",
    )
    dial.visual(
        Cylinder(radius=0.84, length=0.035),
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_material,
        name="dial_disc",
    )
    dial.visual(
        Cylinder(radius=0.93, length=0.045),
        origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=0.07, length=0.02),
        origin=Origin(xyz=(0.0, 0.17, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="dial_center_cap",
    )
    dial.inertial = Inertial.from_geometry(
        Box((1.94, 0.18, 1.94)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.09, 0.0)),
    )

    model.articulation(
        f"tower_to_{face_name}_dial",
        ArticulationType.FIXED,
        parent=tower,
        child=dial,
        origin=Origin(xyz=face_center, rpy=(0.0, 0.0, yaw)),
    )

    hour_hand = model.part(f"{face_name}_hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.08, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hand_material,
        name="hub",
    )
    hour_hand.visual(
        Box((0.12, 0.018, 0.72)),
        origin=Origin(xyz=(0.0, 0.009, 0.18)),
        material=hand_material,
        name="hour_blade",
    )
    hour_hand.visual(
        Box((0.20, 0.018, 0.18)),
        origin=Origin(xyz=(0.0, 0.009, 0.33)),
        material=hand_material,
        name="hour_spade",
    )
    hour_hand.visual(
        Box((0.08, 0.018, 0.20)),
        origin=Origin(xyz=(0.0, 0.009, -0.13)),
        material=hand_material,
        name="hour_counterweight",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.20, 0.018, 0.92)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.009, 0.20)),
    )

    model.articulation(
        f"{face_name}_hour_rotation",
        ArticulationType.CONTINUOUS,
        parent=dial,
        child=hour_hand,
        origin=Origin(xyz=(0.0, 0.18, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.5),
    )

    minute_hand = model.part(f"{face_name}_minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.06, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hand_material,
        name="hub",
    )
    minute_hand.visual(
        Box((0.08, 0.014, 1.06)),
        origin=Origin(xyz=(0.0, 0.007, 0.30)),
        material=hand_material,
        name="minute_blade",
    )
    minute_hand.visual(
        Box((0.15, 0.014, 0.20)),
        origin=Origin(xyz=(0.0, 0.007, 0.55)),
        material=hand_material,
        name="minute_tip",
    )
    minute_hand.visual(
        Box((0.06, 0.014, 0.18)),
        origin=Origin(xyz=(0.0, 0.007, -0.14)),
        material=hand_material,
        name="minute_counterweight",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.15, 0.014, 1.24)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.007, 0.34)),
    )

    model.articulation(
        f"{face_name}_minute_rotation",
        ArticulationType.CONTINUOUS,
        parent=dial,
        child=minute_hand,
        origin=Origin(xyz=(0.0, 0.198, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.8),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gothic_bell_clock_tower")

    stone = model.material("stone", rgba=(0.66, 0.65, 0.61, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.54, 0.53, 0.50, 1.0))
    dial_ivory = model.material("dial_ivory", rgba=(0.88, 0.86, 0.78, 1.0))
    oxidized_metal = model.material("oxidized_metal", rgba=(0.16, 0.18, 0.18, 1.0))
    bronze = model.material("bronze", rgba=(0.58, 0.42, 0.20, 1.0))
    slate = model.material("slate", rgba=(0.22, 0.24, 0.28, 1.0))

    tower = model.part("tower_body")

    plinth_size = 7.2
    shaft_width = 5.2
    shaft_height = 27.0
    shaft_bottom = 1.2
    shaft_top = shaft_bottom + shaft_height
    belfry_width = 5.6
    belfry_height = 5.0
    belfry_bottom = shaft_top
    belfry_center_z = belfry_bottom + belfry_height * 0.5
    parapet_height = 0.8
    roof_base_z = belfry_bottom + belfry_height + parapet_height

    belfry_wall_mesh = _build_belfry_wall_mesh(
        width=belfry_width,
        height=belfry_height,
        thickness=0.45,
    )
    roof_mesh = _build_roof_mesh(base_size=4.8, roof_height=5.6)

    tower.visual(
        Box((plinth_size, plinth_size, 1.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.6)),
        material=stone_dark,
        name="foundation_plinth",
    )
    tower.visual(
        Box((shaft_width, shaft_width, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, shaft_bottom + shaft_height * 0.5)),
        material=stone,
        name="masonry_shaft",
    )
    for sx in (-1.95, 1.95):
        for sy in (-1.95, 1.95):
            tower.visual(
                Box((0.82, 0.82, 16.0)),
                origin=Origin(xyz=(sx, sy, 8.0)),
                material=stone_dark,
                name=f"buttress_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    tower.visual(
        Box((6.0, 6.0, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 21.0)),
        material=stone_dark,
        name="clock_stage_stringcourse",
    )
    tower.visual(
        Box((5.8, 5.8, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, shaft_top + 0.175)),
        material=stone_dark,
        name="belfry_floor_band",
    )
    tower.visual(
        Box((4.15, 4.15, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, belfry_bottom + 0.125)),
        material=stone_dark,
        name="belfry_floor",
    )
    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        direction_x = math.sin(yaw)
        direction_y = math.cos(yaw)
        tower.visual(
            belfry_wall_mesh,
            origin=Origin(
                xyz=(
                    direction_x * (belfry_width * 0.5 - 0.225),
                    direction_y * (belfry_width * 0.5 - 0.225),
                    belfry_center_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=stone,
            name=f"belfry_wall_{index}",
        )
    for yaw in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0):
        direction_x = math.sin(yaw)
        direction_y = math.cos(yaw)
        tangent_x = math.cos(yaw)
        tangent_y = -math.sin(yaw)
        tower.visual(
            Box((1.45, 0.22, 0.28)),
            origin=Origin(
                xyz=(
                    direction_x * 2.46 + tangent_x * 1.35,
                    direction_y * 2.46 + tangent_y * 1.35,
                    24.1,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=stone_dark,
            name=f"clock_hood_left_{int((yaw % (2.0 * math.pi)) * 1000)}",
        )
        tower.visual(
            Box((1.45, 0.22, 0.28)),
            origin=Origin(
                xyz=(
                    direction_x * 2.46 - tangent_x * 1.35,
                    direction_y * 2.46 - tangent_y * 1.35,
                    24.1,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=stone_dark,
            name=f"clock_hood_right_{int((yaw % (2.0 * math.pi)) * 1000)}",
        )
    for yaw in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0):
        direction_x = math.sin(yaw)
        direction_y = math.cos(yaw)
        tangent_x = math.cos(yaw)
        tangent_y = -math.sin(yaw)
        tower.visual(
            Box((5.6, 0.45, parapet_height)),
            origin=Origin(
                xyz=(
                    direction_x * (belfry_width * 0.5 - 0.225),
                    direction_y * (belfry_width * 0.5 - 0.225),
                    belfry_bottom + belfry_height + parapet_height * 0.5,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=stone_dark,
            name=f"parapet_wall_{int((yaw % (2.0 * math.pi)) * 1000)}",
        )
        for side in (-1.0, 1.0):
            tower.visual(
                Box((0.48, 0.48, 1.25)),
                origin=Origin(
                    xyz=(
                        direction_x * 2.52 + tangent_x * side * 2.52,
                        direction_y * 2.52 + tangent_y * side * 2.52,
                        roof_base_z - 0.15,
                    ),
                ),
                material=stone_dark,
                name=f"pinnacle_{int((yaw % (2.0 * math.pi)) * 1000)}_{'pos' if side > 0 else 'neg'}",
            )
    tower.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, roof_base_z)),
        material=slate,
        name="spire_roof",
    )
    tower.visual(
        Cylinder(radius=0.10, length=1.2),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 6.2)),
        material=oxidized_metal,
        name="roof_finial",
    )
    tower.visual(
        Box((0.55, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 6.65)),
        material=oxidized_metal,
        name="crossbar",
    )
    tower.visual(
        Box((0.08, 0.08, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 6.42)),
        material=oxidized_metal,
        name="cross_upright",
    )
    tower.visual(
        Box((4.72, 0.36, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 31.25)),
        material=stone_dark,
        name="bell_beam",
    )
    for x_pos in (-1.95, 1.95):
        tower.visual(
            Box((0.24, 0.24, 2.66)),
            origin=Origin(xyz=(x_pos, 0.0, 29.78)),
            material=stone_dark,
            name=f"bell_frame_post_{'west' if x_pos < 0.0 else 'east'}",
        )
    tower.inertial = Inertial.from_geometry(
        Box((plinth_size, plinth_size, roof_base_z + 6.8)),
        mass=280000.0,
        origin=Origin(xyz=(0.0, 0.0, (roof_base_z + 6.8) * 0.5)),
    )

    bell = model.part("bell")
    bell_mesh = _build_bell_mesh()
    bell.visual(
        Box((1.55, 0.34, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
        material=stone_dark,
        name="headstock",
    )
    bell.visual(
        Cylinder(radius=0.11, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        material=stone_dark,
        name="crown_spindle",
    )
    bell.visual(
        bell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.88)),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.05, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, -1.72)),
        material=oxidized_metal,
        name="clapper",
    )
    bell.inertial = Inertial.from_geometry(
        Box((1.55, 0.66, 2.4)),
        mass=1400.0,
        origin=Origin(xyz=(0.0, 0.0, -1.2)),
    )
    model.articulation(
        "tower_to_bell",
        ArticulationType.FIXED,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 31.11)),
    )

    dial_offset = shaft_width * 0.5
    clock_center_z = 24.3
    face_specs = (
        ("north", 0.0, (0.0, dial_offset, clock_center_z)),
        ("east", -math.pi / 2.0, (dial_offset, 0.0, clock_center_z)),
        ("south", math.pi, (0.0, -dial_offset, clock_center_z)),
        ("west", math.pi / 2.0, (-dial_offset, 0.0, clock_center_z)),
    )
    for face_name, yaw, face_center in face_specs:
        _add_clock_face(
            model,
            tower,
            face_name=face_name,
            yaw=yaw,
            face_center=face_center,
            dial_material=dial_ivory,
            frame_material=stone_dark,
            hand_material=oxidized_metal,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    tower = object_model.get_part("tower_body")
    bell = object_model.get_part("bell")

    ctx.expect_contact(bell, tower, name="bell assembly is hung from the tower beam")

    face_names = ("north", "east", "south", "west")
    for face_name in face_names:
        dial = object_model.get_part(f"{face_name}_dial")
        hour_hand = object_model.get_part(f"{face_name}_hour_hand")
        minute_hand = object_model.get_part(f"{face_name}_minute_hand")
        hour_joint = object_model.get_articulation(f"{face_name}_hour_rotation")
        minute_joint = object_model.get_articulation(f"{face_name}_minute_rotation")

        ctx.expect_contact(dial, tower, name=f"{face_name} dial is mounted to the masonry shaft")
        ctx.expect_contact(hour_hand, dial, name=f"{face_name} hour hand seats against the dial")
        ctx.expect_contact(
            minute_hand,
            hour_hand,
            name=f"{face_name} minute hand stacks ahead of the hour hand",
        )
        ctx.check(
            f"{face_name} hour hand uses a continuous coaxial joint",
            hour_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(value, 6) for value in hour_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={hour_joint.articulation_type}, axis={hour_joint.axis}",
        )
        ctx.check(
            f"{face_name} minute hand uses a continuous coaxial joint",
            minute_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(value, 6) for value in minute_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={minute_joint.articulation_type}, axis={minute_joint.axis}",
        )
        hour_origin = hour_joint.origin.xyz
        minute_origin = minute_joint.origin.xyz
        ctx.check(
            f"{face_name} clock hands are coaxially stacked",
            abs(hour_origin[0] - minute_origin[0]) < 1e-9
            and abs(hour_origin[2] - minute_origin[2]) < 1e-9
            and minute_origin[1] > hour_origin[1] + 0.01,
            details=f"hour_origin={hour_origin}, minute_origin={minute_origin}",
        )

    def _spans(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            upper[0] - lower[0],
            upper[1] - lower[1],
            upper[2] - lower[2],
        )

    north_minute = object_model.get_part("north_minute_hand")
    north_hour = object_model.get_part("north_hour_hand")
    east_minute = object_model.get_part("east_minute_hand")
    north_minute_joint = object_model.get_articulation("north_minute_rotation")
    north_hour_joint = object_model.get_articulation("north_hour_rotation")
    east_minute_joint = object_model.get_articulation("east_minute_rotation")

    north_minute_rest = _spans(ctx.part_world_aabb(north_minute))
    north_hour_rest = _spans(ctx.part_world_aabb(north_hour))
    east_minute_rest = _spans(ctx.part_world_aabb(east_minute))

    with ctx.pose({north_minute_joint: math.pi / 2.0}):
        north_minute_quarter = _spans(ctx.part_world_aabb(north_minute))
    with ctx.pose({north_hour_joint: math.pi / 2.0}):
        north_hour_quarter = _spans(ctx.part_world_aabb(north_hour))
    with ctx.pose({east_minute_joint: math.pi / 2.0}):
        east_minute_quarter = _spans(ctx.part_world_aabb(east_minute))

    ctx.check(
        "north minute hand rotates from vertical toward horizontal across the dial",
        north_minute_rest is not None
        and north_minute_quarter is not None
        and north_minute_rest[2] > north_minute_rest[0]
        and north_minute_quarter[0] > north_minute_quarter[2],
        details=f"rest={north_minute_rest}, quarter={north_minute_quarter}",
    )
    ctx.check(
        "north hour hand rotates independently about the same dial center",
        north_hour_rest is not None
        and north_hour_quarter is not None
        and north_hour_rest[2] > north_hour_rest[0]
        and north_hour_quarter[0] > north_hour_quarter[2],
        details=f"rest={north_hour_rest}, quarter={north_hour_quarter}",
    )
    ctx.check(
        "east minute hand uses the side-face axis instead of the front-face axis",
        east_minute_rest is not None
        and east_minute_quarter is not None
        and east_minute_rest[2] > east_minute_rest[1]
        and east_minute_quarter[1] > east_minute_quarter[2],
        details=f"rest={east_minute_rest}, quarter={east_minute_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
