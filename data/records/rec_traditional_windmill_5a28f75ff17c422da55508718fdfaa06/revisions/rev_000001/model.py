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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _cap_section(
    x: float,
    half_width: float,
    eave_z: float,
    crown_z: float,
    *,
    bottom_z: float = 0.24,
) -> list[tuple[float, float, float]]:
    roof_rise = crown_z - eave_z
    return [
        (x, -0.88 * half_width, bottom_z),
        (x, -1.00 * half_width, bottom_z + 0.18),
        (x, -0.94 * half_width, eave_z),
        (x, -0.66 * half_width, eave_z + 0.52 * roof_rise),
        (x, -0.30 * half_width, eave_z + 0.84 * roof_rise),
        (x, 0.0, crown_z),
        (x, 0.30 * half_width, eave_z + 0.84 * roof_rise),
        (x, 0.66 * half_width, eave_z + 0.52 * roof_rise),
        (x, 0.94 * half_width, eave_z),
        (x, 1.00 * half_width, bottom_z + 0.18),
        (x, 0.88 * half_width, bottom_z),
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    aabb_min, aabb_max = aabb
    return tuple((aabb_min[index] + aabb_max[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.78, 0.73, 0.66, 1.0))
    limestone = model.material("limestone", rgba=(0.67, 0.63, 0.57, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.33, 0.22, 0.14, 1.0))
    weathered_oak = model.material("weathered_oak", rgba=(0.58, 0.48, 0.34, 1.0))
    roof_shingle = model.material("roof_shingle", rgba=(0.22, 0.16, 0.11, 1.0))
    iron = model.material("iron", rgba=(0.24, 0.25, 0.28, 1.0))
    painted_trim = model.material("painted_trim", rgba=(0.92, 0.89, 0.82, 1.0))
    window_black = model.material("window_black", rgba=(0.08, 0.08, 0.09, 1.0))

    cap_roof_mesh = mesh_from_geometry(
        section_loft(
            [
                _cap_section(-1.95, 2.18, 1.30, 2.34),
                _cap_section(-0.90, 2.45, 1.38, 2.72),
                _cap_section(0.20, 2.74, 1.46, 3.02),
                _cap_section(1.20, 2.56, 1.34, 2.64),
                _cap_section(2.08, 2.08, 1.16, 2.12),
            ]
        ),
        "windmill_cap_roof",
    )

    tower = model.part("tower")
    tower.visual(
        Box((9.20, 9.20, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=limestone,
        name="foundation_plinth",
    )
    tower.visual(
        Box((8.30, 8.30, 3.95)),
        origin=Origin(xyz=(0.0, 0.0, 2.525)),
        material=stone,
        name="tower_stage_lower",
    )
    tower.visual(
        Box((7.30, 7.30, 3.30)),
        origin=Origin(xyz=(0.0, 0.0, 6.100)),
        material=stone,
        name="tower_stage_mid",
    )
    tower.visual(
        Box((6.20, 6.20, 2.75)),
        origin=Origin(xyz=(0.0, 0.0, 9.125)),
        material=stone,
        name="tower_stage_upper",
    )
    tower.visual(
        Box((5.35, 5.35, 0.95)),
        origin=Origin(xyz=(0.0, 0.0, 10.375)),
        material=stone,
        name="tower_head_stage",
    )
    tower.visual(
        Box((4.95, 4.95, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 11.025)),
        material=dark_oak,
        name="tower_curb",
    )
    tower.visual(
        Box((1.35, 0.22, 2.35)),
        origin=Origin(xyz=(3.82, 0.0, 1.40)),
        material=dark_oak,
        name="front_door",
    )
    tower.visual(
        Box((0.85, 0.14, 1.20)),
        origin=Origin(xyz=(3.55, 0.0, 4.80)),
        material=window_black,
        name="front_window_lower",
    )
    tower.visual(
        Box((0.72, 0.14, 0.98)),
        origin=Origin(xyz=(3.10, 0.0, 8.35)),
        material=window_black,
        name="front_window_upper",
    )
    tower.inertial = Inertial.from_geometry(
        Box((9.20, 9.20, 11.20)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 5.60)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=2.42, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=dark_oak,
        name="turning_curb",
    )
    cap.visual(
        Box((5.10, 5.10, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=dark_oak,
        name="cap_sill",
    )
    cap.visual(
        cap_roof_mesh,
        material=roof_shingle,
        name="cap_roof",
    )
    cap.visual(
        Box((1.70, 0.76, 1.35)),
        origin=Origin(xyz=(2.78, -0.78, 1.12)),
        material=dark_oak,
        name="left_bearing_cheek",
    )
    cap.visual(
        Box((1.70, 0.76, 1.35)),
        origin=Origin(xyz=(2.78, 0.78, 1.12)),
        material=dark_oak,
        name="right_bearing_cheek",
    )
    cap.visual(
        Cylinder(radius=0.44, length=1.48),
        origin=Origin(xyz=(2.98, 0.0, 1.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="windshaft_bearing",
    )
    cap.visual(
        Box((1.10, 1.30, 0.42)),
        origin=Origin(xyz=(-1.72, 0.0, 1.08)),
        material=dark_oak,
        name="rear_weight_box",
    )
    cap.visual(
        Box((1.90, 0.40, 0.28)),
        origin=Origin(xyz=(-2.18, 0.0, 0.72)),
        material=painted_trim,
        name="tail_beam_stub",
    )
    cap.inertial = Inertial.from_geometry(
        Box((5.60, 5.60, 3.20)),
        mass=2500.0,
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.28, length=0.28),
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_axle_stub",
    )
    sail_hub.visual(
        Cylinder(radius=0.76, length=0.30),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_disc",
    )
    sail_hub.visual(
        Cylinder(radius=0.40, length=1.42),
        origin=Origin(xyz=(0.82, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_barrel",
    )
    sail_hub.visual(
        Cylinder(radius=0.26, length=0.42),
        origin=Origin(xyz=(1.55, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_trim,
        name="hub_spinner",
    )

    blade_angles = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    slat_positions = (2.25, 3.65, 5.05, 6.45, 7.70)
    for blade_index, angle in enumerate(blade_angles):
        prefix = f"blade_{blade_index}"
        sail_hub.visual(
            Box((0.18, 1.18, 1.60)),
            origin=Origin(xyz=(0.32, 0.0, 0.88), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"{prefix}_root_frame",
        )
        sail_hub.visual(
            Box((0.12, 0.28, 6.35)),
            origin=Origin(xyz=(0.34, 0.48, 4.55), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"{prefix}_leading_spar",
        )
        sail_hub.visual(
            Box((0.10, 0.22, 6.35)),
            origin=Origin(xyz=(0.34, -0.46, 4.55), rpy=(angle, 0.0, 0.0)),
            material=weathered_oak,
            name=f"{prefix}_trailing_spar",
        )
        for slat_index, z_pos in enumerate(slat_positions):
            sail_hub.visual(
                Box((0.07, 1.16, 0.14)),
                origin=Origin(xyz=(0.35, 0.0, z_pos), rpy=(angle, 0.0, 0.0)),
                material=painted_trim,
                name=f"{prefix}_slat_{slat_index}",
            )
        sail_hub.visual(
            Box((0.08, 1.04, 0.16)),
            origin=Origin(xyz=(0.35, 0.0, 8.22), rpy=(angle, 0.0, 0.0)),
            material=painted_trim,
            name=f"{prefix}_tip_cap",
        )
    sail_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=1.05, length=16.8),
        mass=1400.0,
        origin=Origin(xyz=(0.48, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 11.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=0.25),
    )
    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(3.72, 0.0, 1.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    cap_joint = object_model.get_articulation("tower_to_cap")
    hub_joint = object_model.get_articulation("cap_to_hub")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="turning_curb",
        negative_elem="tower_curb",
        max_gap=0.01,
        max_penetration=0.001,
        name="cap curb rests on tower curb",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        elem_a="turning_curb",
        elem_b="tower_curb",
        min_overlap=3.8,
        name="cap curb remains broadly seated on tower top",
    )
    ctx.expect_origin_gap(
        sail_hub,
        tower,
        axis="x",
        min_gap=3.5,
        name="sail hub projects well ahead of tower body",
    )

    rest_hub_pos = ctx.part_world_position(sail_hub)
    with ctx.pose({cap_joint: math.pi / 2.0}):
        quarter_turn_hub_pos = ctx.part_world_position(sail_hub)
    ctx.check(
        "cap rotation yaws the hub around the tower axis",
        rest_hub_pos is not None
        and quarter_turn_hub_pos is not None
        and abs(quarter_turn_hub_pos[0]) < 0.9
        and quarter_turn_hub_pos[1] > 3.3
        and abs(quarter_turn_hub_pos[2] - rest_hub_pos[2]) < 0.01,
        details=f"rest={rest_hub_pos}, quarter_turn={quarter_turn_hub_pos}",
    )

    hub_center = ctx.part_world_position(sail_hub)
    rest_tip_center = _aabb_center(ctx.part_element_world_aabb(sail_hub, elem="blade_0_tip_cap"))
    with ctx.pose({hub_joint: math.pi / 2.0}):
        turned_tip_center = _aabb_center(
            ctx.part_element_world_aabb(sail_hub, elem="blade_0_tip_cap")
        )
    ctx.check(
        "hub spin swings a sail tip around the windshaft axis",
        hub_center is not None
        and rest_tip_center is not None
        and turned_tip_center is not None
        and rest_tip_center[2] > hub_center[2] + 6.5
        and abs(turned_tip_center[2] - hub_center[2]) < 1.1
        and abs(turned_tip_center[1] - hub_center[1]) > 6.5,
        details=(
            f"hub_center={hub_center}, rest_tip_center={rest_tip_center}, "
            f"turned_tip_center={turned_tip_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
