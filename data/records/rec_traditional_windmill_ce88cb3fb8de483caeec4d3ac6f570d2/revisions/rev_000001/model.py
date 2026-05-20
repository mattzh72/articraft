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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _octagon_section(
    radius_x: float,
    radius_y: float,
    chamfer: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (radius_x - chamfer, -radius_y, z),
        (radius_x, -radius_y + chamfer, z),
        (radius_x, radius_y - chamfer, z),
        (radius_x - chamfer, radius_y, z),
        (-radius_x + chamfer, radius_y, z),
        (-radius_x, radius_y - chamfer, z),
        (-radius_x, -radius_y + chamfer, z),
        (-radius_x + chamfer, -radius_y, z),
    ]


def _tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 56,
):
    outer_profile = [
        (outer_radius, -length * 0.5),
        (outer_radius, length * 0.5),
    ]
    inner_profile = [
        (inner_radius, -length * 0.5),
        (inner_radius, length * 0.5),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_traditional_windmill")

    weathered_oak = model.material("weathered_oak", rgba=(0.58, 0.47, 0.33, 1.0))
    stone = model.material("stone", rgba=(0.69, 0.67, 0.63, 1.0))
    cap_graphite = model.material("cap_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    sail_offwhite = model.material("sail_offwhite", rgba=(0.87, 0.85, 0.80, 1.0))
    signal_red = model.material("signal_red", rgba=(0.72, 0.16, 0.10, 1.0))

    tower_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _octagon_section(1.55, 1.55, 0.46, 0.80),
                _octagon_section(1.33, 1.33, 0.39, 3.85),
                _octagon_section(1.07, 1.07, 0.30, 6.54),
            ]
        ),
        "windmill_tower_shell",
    )
    tower_ring_mesh = _tube_mesh(
        "tower_yaw_thrust_ring",
        outer_radius=1.12,
        inner_radius=0.78,
        length=0.10,
    )
    cap_shoe_mesh = _tube_mesh(
        "cap_yaw_shoe",
        outer_radius=1.10,
        inner_radius=0.80,
        length=0.08,
    )
    yaw_sleeve_mesh = _tube_mesh(
        "cap_yaw_sleeve",
        outer_radius=0.22,
        inner_radius=0.135,
        length=0.24,
    )
    tower = model.part("tower")
    tower.visual(
        Box((4.20, 4.20, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=stone,
        name="foundation_plinth",
    )
    tower.visual(
        Box((3.00, 3.00, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=stone,
        name="stone_curb",
    )
    tower.visual(tower_shell_mesh, material=weathered_oak, name="tower_shell")
    tower.visual(
        Box((0.70, 0.08, 1.40)),
        origin=Origin(xyz=(0.0, 1.50, 1.50)),
        material=dark_steel,
        name="access_door",
    )
    tower.visual(
        Box((0.60, 0.10, 0.24)),
        origin=Origin(xyz=(0.0, 1.08, 6.16)),
        material=steel,
        name="front_datum_pad",
    )
    tower.visual(
        Box((0.10, 0.60, 0.24)),
        origin=Origin(xyz=(-1.08, 0.0, 6.16)),
        material=steel,
        name="side_datum_pad",
    )
    tower.visual(
        Box((2.05, 2.05, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 6.67)),
        material=weathered_oak,
        name="cap_table",
    )
    tower.visual(
        tower_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 6.85)),
        material=dark_steel,
        name="yaw_thrust_ring",
    )
    tower.visual(
        Cylinder(radius=0.12, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 6.71)),
        material=steel,
        name="yaw_kingpin",
    )
    tower.visual(
        Box((0.18, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, 1.025, 6.68)),
        material=signal_red,
        name="tower_index_mark",
    )
    tower.inertial = Inertial.from_geometry(
        Box((4.20, 4.20, 7.05)),
        mass=16000.0,
        origin=Origin(xyz=(0.0, 0.0, 3.525)),
    )

    cap = model.part("cap")
    cap.visual(
        cap_shoe_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="yaw_shoe",
    )
    cap.visual(
        yaw_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=steel,
        name="yaw_sleeve",
    )
    cap.visual(
        Box((2.30, 1.95, 0.18)),
        origin=Origin(xyz=(0.05, 0.0, 0.17)),
        material=cap_graphite,
        name="deck_base",
    )
    cap.visual(
        Box((1.84, 1.75, 0.48)),
        origin=Origin(xyz=(0.06, 0.0, 0.45)),
        material=cap_graphite,
        name="main_body",
    )
    cap.visual(
        Box((1.55, 1.05, 0.24)),
        origin=Origin(xyz=(-0.02, 0.0, 0.79)),
        material=cap_graphite,
        name="roof_ridge",
    )
    cap.visual(
        Box((0.55, 1.20, 0.18)),
        origin=Origin(xyz=(-0.88, 0.0, 0.38)),
        material=cap_graphite,
        name="rear_service_face",
    )
    cap.visual(
        Box((0.52, 0.10, 0.30)),
        origin=Origin(xyz=(0.84, 0.41, 0.47)),
        material=cap_graphite,
        name="left_nose_side",
    )
    cap.visual(
        Box((0.52, 0.10, 0.30)),
        origin=Origin(xyz=(0.84, -0.41, 0.47)),
        material=cap_graphite,
        name="right_nose_side",
    )
    cap.visual(
        Box((0.96, 0.84, 0.22)),
        origin=Origin(xyz=(1.24, 0.0, 0.25)),
        material=dark_steel,
        name="bearing_bed",
    )
    cap.visual(
        Box((0.22, 0.52, 0.24)),
        origin=Origin(xyz=(1.14, 0.0, 0.48)),
        material=dark_steel,
        name="rear_pedestal",
    )
    cap.visual(
        Box((0.22, 0.52, 0.24)),
        origin=Origin(xyz=(1.46, 0.0, 0.48)),
        material=dark_steel,
        name="front_pedestal",
    )
    cap.visual(
        Box((0.18, 0.42, 0.10)),
        origin=Origin(xyz=(1.14, 0.0, 0.62)),
        material=steel,
        name="rear_lower_saddle",
    )
    cap.visual(
        Box((0.18, 0.08, 0.26)),
        origin=Origin(xyz=(1.14, 0.18, 0.76)),
        material=steel,
        name="rear_left_cheek",
    )
    cap.visual(
        Box((0.18, 0.08, 0.26)),
        origin=Origin(xyz=(1.14, -0.18, 0.76)),
        material=steel,
        name="rear_right_cheek",
    )
    cap.visual(
        Box((0.14, 0.42, 0.06)),
        origin=Origin(xyz=(1.14, 0.0, 0.92)),
        material=steel,
        name="rear_upper_keeper",
    )
    cap.visual(
        Box((0.02, 0.04, 0.32)),
        origin=Origin(xyz=(1.03, 0.135, 0.76)),
        material=steel,
        name="left_thrust_pad",
    )
    cap.visual(
        Box((0.02, 0.04, 0.32)),
        origin=Origin(xyz=(1.03, -0.135, 0.76)),
        material=steel,
        name="right_thrust_pad",
    )
    cap.visual(
        Box((0.18, 0.42, 0.10)),
        origin=Origin(xyz=(1.46, 0.0, 0.62)),
        material=steel,
        name="front_lower_saddle",
    )
    cap.visual(
        Box((0.18, 0.08, 0.26)),
        origin=Origin(xyz=(1.46, 0.18, 0.76)),
        material=steel,
        name="front_left_cheek",
    )
    cap.visual(
        Box((0.18, 0.08, 0.26)),
        origin=Origin(xyz=(1.46, -0.18, 0.76)),
        material=steel,
        name="front_right_cheek",
    )
    cap.visual(
        Box((0.14, 0.42, 0.06)),
        origin=Origin(xyz=(1.46, 0.0, 0.92)),
        material=steel,
        name="front_upper_keeper",
    )
    cap.visual(
        Box((0.90, 0.24, 0.05)),
        origin=Origin(xyz=(-0.12, 0.0, 0.925)),
        material=steel,
        name="roof_datum_pad",
    )
    cap.visual(
        Box((0.24, 0.80, 0.05)),
        origin=Origin(xyz=(0.28, 0.0, 0.915)),
        material=steel,
        name="cross_datum_pad",
    )
    cap.visual(
        Box((0.12, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, 0.97, 0.10)),
        material=signal_red,
        name="cap_index_pointer",
    )
    cap.visual(
        Cylinder(radius=0.035, length=0.34),
        origin=Origin(xyz=(1.30, 0.29, 0.56), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="left_adjuster",
    )
    cap.visual(
        Cylinder(radius=0.035, length=0.34),
        origin=Origin(xyz=(1.30, -0.29, 0.56), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="right_adjuster",
    )
    cap.visual(
        Box((0.14, 0.10, 0.24)),
        origin=Origin(xyz=(1.30, 0.29, 0.48)),
        material=dark_steel,
        name="left_adjuster_bracket",
    )
    cap.visual(
        Box((0.14, 0.10, 0.24)),
        origin=Origin(xyz=(1.30, -0.29, 0.48)),
        material=dark_steel,
        name="right_adjuster_bracket",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.40, 2.00, 1.05)),
        mass=1100.0,
        origin=Origin(xyz=(0.18, 0.0, 0.48)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.09, length=0.80),
        origin=Origin(xyz=(0.28, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="main_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.17, length=0.04),
        origin=Origin(xyz=(-0.14, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="thrust_collar",
    )
    rotor.visual(
        Cylinder(radius=0.22, length=0.40),
        origin=Origin(xyz=(0.82, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    rotor.visual(
        Box((0.24, 0.46, 0.46)),
        origin=Origin(xyz=(1.02, 0.0, 0.0)),
        material=dark_steel,
        name="hub_block",
    )
    rotor.visual(
        Cylinder(radius=0.33, length=0.08),
        origin=Origin(xyz=(1.18, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="hub_face",
    )
    rotor.visual(
        Box((0.05, 0.12, 0.18)),
        origin=Origin(xyz=(1.22, 0.0, 0.20)),
        material=signal_red,
        name="hub_index_tab",
    )

    blade_angles = (
        math.pi * 0.25,
        math.pi * 0.75,
        math.pi * 1.25,
        math.pi * 1.75,
    )
    blade_slats = (1.32, 1.72, 2.12, 2.52, 2.92, 3.32, 3.72)
    for blade_index, angle in enumerate(blade_angles):
        rotor.visual(
            Box((0.14, 0.12, 1.55)),
            origin=Origin(xyz=(1.02, 0.0, 0.54), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"stock_{blade_index}",
        )
        rotor.visual(
            Box((0.08, 0.74, 0.08)),
            origin=Origin(xyz=(1.05, 0.0, 1.05), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"root_bridge_{blade_index}",
        )
        rotor.visual(
            Box((0.08, 0.06, 3.15)),
            origin=Origin(xyz=(1.05, 0.31, 2.55), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"leading_spar_{blade_index}",
        )
        rotor.visual(
            Box((0.08, 0.06, 3.15)),
            origin=Origin(xyz=(1.05, -0.31, 2.55), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"trailing_spar_{blade_index}",
        )
        for slat_index, slat_z in enumerate(blade_slats):
            rotor.visual(
                Box((0.05, 0.72, 0.035)),
                origin=Origin(xyz=(1.06, 0.0, slat_z), rpy=(angle, 0.0, 0.0)),
                material=sail_offwhite,
                name=f"lattice_{blade_index}_{slat_index}",
            )
        rotor.visual(
            Box((0.06, 0.76, 0.06)),
            origin=Origin(xyz=(1.05, 0.0, 4.10), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"tip_rail_{blade_index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((1.40, 8.60, 8.60)),
        mass=850.0,
        origin=Origin(xyz=(1.10, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 6.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.18),
    )
    model.articulation(
        "cap_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(1.14, 0.0, 0.76)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_cap_yaw")
    spin = object_model.get_articulation("cap_to_rotor_spin")

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
        "yaw_axis_vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        f"expected vertical yaw axis, got {yaw.axis}",
    )
    ctx.check(
        "rotor_axis_longitudinal",
        spin.axis == (1.0, 0.0, 0.0),
        f"expected rotor shaft axis along +X, got {spin.axis}",
    )

    with ctx.pose({yaw: 0.0, spin: 0.0}):
        ctx.expect_gap(
            cap,
            tower,
            axis="z",
            positive_elem="yaw_shoe",
            negative_elem="yaw_thrust_ring",
            max_gap=0.001,
            max_penetration=0.0,
            name="yaw_bearing_axial_gap",
        )
        ctx.expect_overlap(
            cap,
            tower,
            axes="xy",
            elem_a="yaw_shoe",
            elem_b="yaw_thrust_ring",
            min_overlap=1.50,
            name="yaw_bearing_overlap_footprint",
        )
        ctx.expect_within(
            tower,
            cap,
            axes="xy",
            inner_elem="yaw_kingpin",
            outer_elem="yaw_sleeve",
            margin=0.0,
            name="yaw_kingpin_within_sleeve",
        )
        ctx.expect_gap(
            rotor,
            cap,
            axis="z",
            positive_elem="main_shaft",
            negative_elem="rear_lower_saddle",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear_lower_bearing_clearance",
        )
        ctx.expect_gap(
            cap,
            rotor,
            axis="z",
            positive_elem="rear_upper_keeper",
            negative_elem="main_shaft",
            min_gap=0.03,
            max_gap=0.05,
            name="rear_upper_bearing_clearance",
        )
        ctx.expect_gap(
            cap,
            rotor,
            axis="y",
            positive_elem="rear_left_cheek",
            negative_elem="main_shaft",
            min_gap=0.04,
            max_gap=0.06,
            name="shaft_within_rear_bearing_seat",
        )
        ctx.expect_gap(
            rotor,
            cap,
            axis="z",
            positive_elem="main_shaft",
            negative_elem="front_lower_saddle",
            max_gap=0.001,
            max_penetration=0.0,
            name="front_lower_bearing_clearance",
        )
        ctx.expect_gap(
            cap,
            rotor,
            axis="z",
            positive_elem="front_upper_keeper",
            negative_elem="main_shaft",
            min_gap=0.03,
            max_gap=0.05,
            name="front_upper_bearing_clearance",
        )
        ctx.expect_gap(
            cap,
            rotor,
            axis="y",
            positive_elem="front_left_cheek",
            negative_elem="main_shaft",
            min_gap=0.04,
            max_gap=0.06,
            name="shaft_within_front_bearing_seat",
        )
        ctx.expect_contact(
            cap,
            rotor,
            elem_a="left_thrust_pad",
            elem_b="thrust_collar",
            name="left_thrust_pad_contact",
        )
        ctx.expect_contact(
            cap,
            rotor,
            elem_a="right_thrust_pad",
            elem_b="thrust_collar",
            name="right_thrust_pad_contact",
        )
        ctx.expect_gap(
            rotor,
            cap,
            axis="x",
            positive_elem="hub_barrel",
            negative_elem="front_lower_saddle",
            min_gap=0.15,
            max_gap=0.30,
            name="hub_to_front_bearing_service_gap",
        )

        rest_pos = ctx.part_world_position(rotor)

    with ctx.pose({yaw: math.pi * 0.5, spin: 0.0}):
        yawed_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw_moves_rotor_around_tower",
        rest_pos is not None
        and yawed_pos is not None
        and abs(yawed_pos[0]) < 0.12
        and yawed_pos[1] > 1.0
        and abs(rest_pos[0] - 1.14) < 0.05,
        f"unexpected rotor positions rest={rest_pos} yawed={yawed_pos}",
    )

    with ctx.pose({yaw: 0.0, spin: math.pi / 3.0}):
        spun_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor_spin_keeps_shaft_center_fixed",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-6,
        f"rotor origin moved under spin rest={rest_pos} spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
