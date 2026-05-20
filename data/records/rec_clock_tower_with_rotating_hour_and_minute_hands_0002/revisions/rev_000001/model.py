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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _rect_loop(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d, z),
        (half_w, -half_d, z),
        (half_w, half_d, z),
        (-half_w, half_d, z),
    ]


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _pointed_arch_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    shoulder = height * 0.58
    inner_shoulder = width * 0.18
    return [
        (-half_w, 0.0),
        (half_w, 0.0),
        (half_w, shoulder),
        (inner_shoulder, height * 0.80),
        (0.0, height),
        (-inner_shoulder, height * 0.80),
        (-half_w, shoulder),
    ]


def _belfry_panel_mesh(
    *,
    name: str,
    panel_width: float,
    panel_height: float,
    panel_thickness: float,
    hole_profiles: list[list[tuple[float, float]]],
):
    outer = [
        (-panel_width * 0.5, 0.0),
        (panel_width * 0.5, 0.0),
        (panel_width * 0.5, panel_height),
        (-panel_width * 0.5, panel_height),
    ]
    geom = ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        panel_thickness,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _gable_mesh(name: str, width: float, height: float, depth: float):
    half_w = width * 0.5
    profile = [
        (-half_w, 0.0),
        (half_w, 0.0),
        (half_w * 0.72, height * 0.18),
        (0.0, height),
        (-half_w * 0.72, height * 0.18),
    ]
    geom = ExtrudeGeometry.centered(profile, depth, cap=True, closed=True).rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _hand_mesh(
    *,
    name: str,
    length: float,
    base_width: float,
    tip_width: float,
    tail_length: float,
    tail_width: float,
    thickness: float,
):
    profile = [
        (-tail_width * 0.5, -tail_length),
        (tail_width * 0.5, -tail_length),
        (tail_width * 0.5, -0.10),
        (base_width * 0.5, 0.04),
        (base_width * 0.26, length * 0.62),
        (tip_width * 0.7, length * 0.85),
        (0.0, length),
        (-tip_width * 0.7, length * 0.85),
        (-base_width * 0.26, length * 0.62),
        (-base_width * 0.5, 0.04),
        (-tail_width * 0.5, -0.10),
    ]
    geom = ExtrudeGeometry.centered(profile, thickness, cap=True, closed=True).rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gothic_clock_tower")

    stone = model.material("stone", rgba=(0.69, 0.68, 0.64, 1.0))
    trim_stone = model.material("trim_stone", rgba=(0.58, 0.57, 0.53, 1.0))
    slate = model.material("slate", rgba=(0.18, 0.21, 0.24, 1.0))
    dial_ivory = model.material("dial_ivory", rgba=(0.90, 0.87, 0.79, 1.0))
    bronze = model.material("bronze", rgba=(0.30, 0.22, 0.12, 1.0))
    belfry_shadow = model.material("belfry_shadow", rgba=(0.14, 0.11, 0.09, 1.0))

    tower = model.part("tower")

    shaft_width = 4.8
    shaft_depth = 4.2
    shaft_height = 20.6
    plinth_height = 1.4
    shaft_top = plinth_height + shaft_height
    clock_center_z = 16.8
    front_face_y = shaft_depth * 0.5

    tower.visual(
        Box((6.2, 5.4, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=trim_stone,
        name="plinth",
    )
    tower.visual(
        Box((shaft_width, shaft_depth, shaft_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + shaft_height * 0.5)),
        material=stone,
        name="shaft",
    )

    buttress_height = 18.2
    buttress_origin_z = plinth_height + buttress_height * 0.5
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.72, 0.56, buttress_height)),
                origin=Origin(
                    xyz=(
                        x_sign * (shaft_width * 0.5 - 0.36),
                        y_sign * (shaft_depth * 0.5 - 0.28),
                        buttress_origin_z,
                    )
                ),
                material=trim_stone,
                name=f"buttress_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign > 0 else 'b'}",
            )

    tower.visual(
        Box((3.6, 0.36, 0.22)),
        origin=Origin(xyz=(0.0, front_face_y + 0.18, 15.0)),
        material=trim_stone,
        name="dial_ledge",
    )

    clock_gable = _gable_mesh("clock_gable", width=3.4, height=1.65, depth=0.28)
    tower.visual(
        clock_gable,
        origin=Origin(xyz=(0.0, front_face_y + 0.14, 19.35)),
        material=trim_stone,
        name="clock_gable",
    )

    tower.visual(
        Cylinder(radius=1.55, length=0.10),
        origin=Origin(
            xyz=(0.0, front_face_y + 0.05, clock_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dial_ivory,
        name="clock_dial",
    )
    bezel_geom = TorusGeometry(radius=1.65, tube=0.10, radial_segments=20, tubular_segments=56).rotate_x(
        math.pi / 2.0
    )
    tower.visual(
        mesh_from_geometry(bezel_geom, "clock_bezel"),
        origin=Origin(xyz=(0.0, front_face_y + 0.09, clock_center_z)),
        material=trim_stone,
        name="clock_bezel",
    )
    tower.visual(
        Cylinder(radius=0.14, length=0.04),
        origin=Origin(
            xyz=(0.0, front_face_y + 0.12, clock_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bronze,
        name="clock_hub",
    )

    tower.visual(
        Box((5.6, 5.0, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, shaft_top + 0.4)),
        material=trim_stone,
        name="belfry_lower_cornice",
    )

    belfry_panel_height = 4.4
    belfry_base_z = shaft_top + 0.8
    belfry_mid_z = belfry_base_z + belfry_panel_height * 0.5
    belfry_top_z = belfry_base_z + belfry_panel_height

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.60, 0.60, belfry_panel_height)),
                origin=Origin(
                    xyz=(
                        x_sign * 2.50,
                        y_sign * 2.20,
                        belfry_mid_z,
                    )
                ),
                material=stone,
                name=f"belfry_pier_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign > 0 else 'b'}",
            )

    front_panel = _belfry_panel_mesh(
        name="front_belfry_panel",
        panel_width=4.20,
        panel_height=belfry_panel_height,
        panel_thickness=0.18,
        hole_profiles=[
            _shift_profile(_pointed_arch_profile(1.16, 3.50), dx=-0.92, dy=0.52),
            _shift_profile(_pointed_arch_profile(1.16, 3.50), dx=0.92, dy=0.52),
        ],
    )
    side_panel = _belfry_panel_mesh(
        name="side_belfry_panel",
        panel_width=3.70,
        panel_height=belfry_panel_height,
        panel_thickness=0.18,
        hole_profiles=[_shift_profile(_pointed_arch_profile(1.32, 3.50), dy=0.52)],
    )

    tower.visual(
        front_panel,
        origin=Origin(xyz=(0.0, 2.41, belfry_base_z)),
        material=trim_stone,
        name="front_belfry_panel",
    )
    tower.visual(
        front_panel,
        origin=Origin(xyz=(0.0, -2.41, belfry_base_z)),
        material=trim_stone,
        name="rear_belfry_panel",
    )
    tower.visual(
        side_panel,
        origin=Origin(xyz=(2.71, 0.0, belfry_base_z), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=trim_stone,
        name="right_belfry_panel",
    )
    tower.visual(
        side_panel,
        origin=Origin(xyz=(-2.71, 0.0, belfry_base_z), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=trim_stone,
        name="left_belfry_panel",
    )

    tower.visual(
        Box((3.0, 2.6, belfry_panel_height)),
        origin=Origin(xyz=(0.0, 0.0, belfry_mid_z)),
        material=belfry_shadow,
        name="belfry_shadow_core",
    )

    tower.visual(
        Box((6.0, 5.4, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, belfry_top_z + 0.4)),
        material=trim_stone,
        name="belfry_upper_cornice",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.34, 0.34, 1.2)),
                origin=Origin(
                    xyz=(
                        x_sign * 2.66,
                        y_sign * 2.36,
                        belfry_top_z + 1.4,
                    )
                ),
                material=trim_stone,
                name=f"pinnacle_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign > 0 else 'b'}",
            )

    roof_geom = section_loft(
        [
            _rect_loop(5.8, 5.2, 0.0),
            _rect_loop(3.2, 2.8, 2.6),
            _rect_loop(1.2, 1.0, 5.8),
            _rect_loop(0.25, 0.25, 7.6),
        ]
    )
    tower.visual(
        mesh_from_geometry(roof_geom, "spire_roof"),
        origin=Origin(xyz=(0.0, 0.0, belfry_top_z + 0.8)),
        material=slate,
        name="spire_roof",
    )
    tower.visual(
        Cylinder(radius=0.08, length=1.4),
        origin=Origin(xyz=(0.0, 0.0, belfry_top_z + 0.8 + 8.3)),
        material=bronze,
        name="spire_finial",
    )

    tower.inertial = Inertial.from_geometry(
        Box((6.2, 5.4, 37.0)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 18.5)),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        _hand_mesh(
            name="hour_hand_blade",
            length=0.96,
            base_width=0.20,
            tip_width=0.07,
            tail_length=0.30,
            tail_width=0.08,
            thickness=0.022,
        ),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=bronze,
        name="hour_blade",
    )
    hour_hand.visual(
        Cylinder(radius=0.16, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="hour_hub",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.32, 0.022, 1.30)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.011, 0.33)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        _hand_mesh(
            name="minute_hand_blade",
            length=1.38,
            base_width=0.16,
            tip_width=0.055,
            tail_length=0.38,
            tail_width=0.06,
            thickness=0.018,
        ),
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        material=bronze,
        name="minute_blade",
    )
    minute_hand.visual(
        Cylinder(radius=0.12, length=0.018),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="minute_hub",
    )
    minute_hand.visual(
        Cylinder(radius=0.07, length=0.014),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="minute_cap",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.26, 0.050, 1.78)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.031, 0.49)),
    )

    clock_joint_origin = Origin(xyz=(0.0, front_face_y + 0.14, clock_center_z))
    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hour_hand,
        origin=clock_joint_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=-math.tau,
            upper=math.tau,
        ),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=minute_hand,
        origin=clock_joint_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=-math.tau,
            upper=math.tau,
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

    ctx.expect_contact(
        hour_hand,
        tower,
        elem_a="hour_hub",
        elem_b="clock_hub",
        name="hour hand seats on center arbor",
    )
    ctx.expect_contact(
        minute_hand,
        hour_hand,
        elem_a="minute_hub",
        elem_b="hour_hub",
        name="minute hand stacks on hour hand hub",
    )
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="y",
        positive_elem="minute_hub",
        negative_elem="hour_hub",
        min_gap=0.0,
        max_gap=0.001,
        name="clock hand stack stays separated by face planes",
    )
    ctx.expect_within(
        hour_hand,
        tower,
        axes="xz",
        inner_elem="hour_blade",
        outer_elem="clock_dial",
        margin=0.08,
        name="hour hand remains within dial extent",
    )
    ctx.expect_within(
        minute_hand,
        tower,
        axes="xz",
        inner_elem="minute_blade",
        outer_elem="clock_dial",
        margin=0.08,
        name="minute hand remains within dial extent",
    )
    ctx.check(
        "hour joint axis faces out of the clock face",
        tuple(hour_joint.axis) == (0.0, 1.0, 0.0),
        f"axis={hour_joint.axis!r}",
    )
    ctx.check(
        "minute joint axis faces out of the clock face",
        tuple(minute_joint.axis) == (0.0, 1.0, 0.0),
        f"axis={minute_joint.axis!r}",
    )

    minute_rest = ctx.part_element_world_aabb(minute_hand, elem="minute_blade")
    assert minute_rest is not None
    with ctx.pose({minute_joint: math.pi / 2.0}):
        minute_quarter = ctx.part_element_world_aabb(minute_hand, elem="minute_blade")
        assert minute_quarter is not None
        ctx.check(
            "minute hand rotates across the dial",
            minute_quarter[0][0] < minute_rest[0][0] - 1.1,
            f"rest_min_x={minute_rest[0][0]:.3f}, turned_min_x={minute_quarter[0][0]:.3f}",
        )
        ctx.expect_contact(
            minute_hand,
            hour_hand,
            elem_a="minute_hub",
            elem_b="hour_hub",
            name="minute hand hub stays seated while rotated",
        )
        ctx.expect_within(
            minute_hand,
            tower,
            axes="xz",
            inner_elem="minute_blade",
            outer_elem="clock_dial",
            margin=0.08,
            name="minute hand quarter-turn still stays on the dial",
        )

    hour_rest = ctx.part_element_world_aabb(hour_hand, elem="hour_blade")
    assert hour_rest is not None
    with ctx.pose({hour_joint: -math.pi / 3.0}):
        hour_offset = ctx.part_element_world_aabb(hour_hand, elem="hour_blade")
        assert hour_offset is not None
        ctx.check(
            "hour hand rotates away from noon",
            hour_offset[1][0] > hour_rest[1][0] + 0.65,
            f"rest_max_x={hour_rest[1][0]:.3f}, turned_max_x={hour_offset[1][0]:.3f}",
        )
        ctx.expect_contact(
            hour_hand,
            tower,
            elem_a="hour_hub",
            elem_b="clock_hub",
            name="hour hand arbor contact persists under rotation",
        )
        ctx.expect_within(
            hour_hand,
            tower,
            axes="xz",
            inner_elem="hour_blade",
            outer_elem="clock_dial",
            margin=0.08,
            name="hour hand rotated pose still stays on the dial",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
