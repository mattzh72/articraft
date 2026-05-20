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
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_section(radius: float, z: float, *, segments: int = 40) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
            z,
        )
        for index in range(segments)
    ]


def _cap_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.73, 0.71, 0.66, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.49, 0.33, 0.21, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.27, 0.19, 0.12, 1.0))
    iron = model.material("iron", rgba=(0.24, 0.25, 0.27, 1.0))
    gray_shingle = model.material("gray_shingle", rgba=(0.33, 0.34, 0.36, 1.0))

    tower = model.part("tower")
    tower.visual(
        _save_mesh(
            "tower_body",
            section_loft(
                [
                    _circle_section(2.45, 0.00, segments=48),
                    _circle_section(2.38, 0.35, segments=48),
                    _circle_section(2.22, 1.80, segments=48),
                    _circle_section(2.04, 4.10, segments=48),
                    _circle_section(1.88, 5.55, segments=48),
                    _circle_section(1.80, 6.02, segments=48),
                ]
            ),
        ),
        material=stone,
        name="tower_body",
    )
    tower.visual(
        Cylinder(radius=2.52, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=stone,
        name="foundation_ring",
    )
    tower.visual(
        Cylinder(radius=1.90, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 6.05)),
        material=dark_timber,
        name="cap_track",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.45, length=6.10),
        mass=16000.0,
        origin=Origin(xyz=(0.0, 0.0, 3.05)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=1.88, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_timber,
        name="cap_ring",
    )
    cap.visual(
        _save_mesh(
            "cap_shell",
            section_loft(
                [
                    _cap_section(0.78, 0.34, 0.24, 0.08, 0.33),
                    _cap_section(0.42, 1.10, 0.66, 0.12, 0.46),
                    _cap_section(-0.10, 1.70, 0.98, 0.16, 0.58),
                    _cap_section(-0.62, 1.34, 0.76, 0.14, 0.48),
                    _cap_section(-0.95, 0.62, 0.36, 0.10, 0.32),
                ]
            ),
        ),
        material=gray_shingle,
        name="cap_shell",
    )
    cap.visual(
        Box((0.64, 0.82, 0.10)),
        origin=Origin(xyz=(0.46, 0.0, 0.18)),
        material=weathered_wood,
        name="front_stage",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        cap.visual(
            Box((0.40, 0.10, 0.74)),
            origin=Origin(xyz=(0.52, side_sign * 0.23, 0.47)),
            material=dark_timber,
            name=f"{side_name}_support_cheek",
        )
        cap.visual(
            Box((0.18, 0.14, 0.20)),
            origin=Origin(xyz=(0.70, side_sign * 0.18, 0.54)),
            material=dark_timber,
            name=f"{side_name}_bearing_cheek",
        )
    cap.visual(
        Box((0.22, 0.50, 0.08)),
        origin=Origin(xyz=(0.70, 0.0, 0.28)),
        material=dark_timber,
        name="lower_bearing_bridge",
    )
    cap.visual(
        Box((0.16, 0.50, 0.08)),
        origin=Origin(xyz=(0.68, 0.0, 0.82)),
        material=dark_timber,
        name="upper_bearing_bridge",
    )
    cap.visual(
        Box((0.04, 0.36, 0.24)),
        origin=Origin(xyz=(0.80, 0.0, 0.54)),
        material=iron,
        name="rear_bearing_block",
    )
    cap.visual(
        Box((0.18, 0.05, 0.18)),
        origin=Origin(xyz=(0.74, 0.16, 0.54)),
        material=iron,
        name="left_bearing_collar",
    )
    cap.visual(
        Box((0.18, 0.05, 0.18)),
        origin=Origin(xyz=(0.74, -0.16, 0.54)),
        material=iron,
        name="right_bearing_collar",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.10, 1.90, 1.10)),
        mass=1350.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.55)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.10, length=0.86),
        origin=Origin(xyz=(0.43, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="windshaft",
    )
    hub.visual(
        Cylinder(radius=0.24, length=0.36),
        origin=Origin(xyz=(0.38, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_timber,
        name="hub_barrel",
    )
    hub.visual(
        Cylinder(radius=0.18, length=0.16),
        origin=Origin(xyz=(0.63, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_end_cap",
    )
    hub.visual(
        Box((0.18, 0.28, 0.28)),
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
        material=dark_timber,
        name="hub_spider_block",
    )
    blade_rolls = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    for blade_index, blade_roll in enumerate(blade_rolls, start=1):
        blade_prefix = f"blade_{blade_index}"
        hub.visual(
            Box((0.08, 0.12, 1.20)),
            origin=Origin(xyz=(0.26, 0.0, 0.60), rpy=(blade_roll, 0.0, 0.0)),
            material=weathered_wood,
            name=f"{blade_prefix}_root_spar",
        )
        hub.visual(
            Box((0.05, 0.04, 4.24)),
            origin=Origin(xyz=(0.26, 0.34, 3.31), rpy=(blade_roll, 0.0, 0.0)),
            material=weathered_wood,
            name=f"{blade_prefix}_leading_rail",
        )
        hub.visual(
            Box((0.05, 0.04, 4.24)),
            origin=Origin(xyz=(0.26, -0.34, 3.31), rpy=(blade_roll, 0.0, 0.0)),
            material=weathered_wood,
            name=f"{blade_prefix}_trailing_rail",
        )
        hub.visual(
            Box((0.05, 0.74, 0.08)),
            origin=Origin(xyz=(0.26, 0.0, 1.22), rpy=(blade_roll, 0.0, 0.0)),
            material=dark_timber,
            name=f"{blade_prefix}_root_crossbar",
        )
        for slat_index, slat_z in enumerate((1.90, 2.60, 3.30, 4.00, 4.70), start=1):
            hub.visual(
                Box((0.03, 0.74, 0.05)),
                origin=Origin(xyz=(0.26, 0.0, slat_z), rpy=(blade_roll, 0.0, 0.0)),
                material=weathered_wood,
                name=f"{blade_prefix}_lattice_slat_{slat_index}",
            )
    hub.inertial = Inertial.from_geometry(
        Box((1.20, 10.80, 10.80)),
        mass=950.0,
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 6.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.25),
    )
    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(0.82, 0.0, 0.54)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("hub")
    cap_joint = object_model.get_articulation("tower_to_cap")
    hub_joint = object_model.get_articulation("cap_to_hub")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        max_gap=0.02,
        max_penetration=0.0,
        positive_elem="cap_ring",
        negative_elem="cap_track",
        name="cap ring sits on tower track",
    )
    ctx.expect_origin_gap(
        hub,
        cap,
        axis="x",
        min_gap=0.70,
        name="hub projects forward from the cap",
    )

    rest_hub_pos = ctx.part_world_position(hub)
    with ctx.pose({cap_joint: math.pi / 2.0}):
        quarter_turn_hub_pos = ctx.part_world_position(hub)
    ctx.check(
        "cap rotation carries the hub around the tower axis",
        rest_hub_pos is not None
        and quarter_turn_hub_pos is not None
        and rest_hub_pos[0] > 0.70
        and abs(quarter_turn_hub_pos[0]) < 0.15
        and quarter_turn_hub_pos[1] > 0.70,
        details=f"rest={rest_hub_pos}, quarter_turn={quarter_turn_hub_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    rest_blade_center = _aabb_center(ctx.part_element_world_aabb(hub, elem="blade_1_leading_rail"))
    with ctx.pose({hub_joint: math.pi / 2.0}):
        quarter_turn_blade_center = _aabb_center(ctx.part_element_world_aabb(hub, elem="blade_1_leading_rail"))
    ctx.check(
        "hub rotation swings a sail through the wheel plane",
        rest_blade_center is not None
        and quarter_turn_blade_center is not None
        and rest_blade_center[2] > quarter_turn_blade_center[2] + 2.5
        and quarter_turn_blade_center[1] < rest_blade_center[1] - 3.0,
        details=f"rest={rest_blade_center}, quarter_turn={quarter_turn_blade_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
