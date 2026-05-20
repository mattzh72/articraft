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
    tube_from_spline_points,
)


def _octagon_section(radius_x: float, radius_y: float, z: float) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(8):
        angle = math.pi / 8.0 + index * (math.pi / 4.0)
        points.append((radius_x * math.cos(angle), radius_y * math.sin(angle), z))
    return points


def _yz_section(x: float, width: float, height: float, radius: float, base_z: float) -> list[tuple[float, float, float]]:
    center_z = base_z + height * 0.5
    return [(x, y, z + center_z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def _add_blade(part, blade_index: int, roll: float, material) -> None:
    blade_origin = lambda xyz: Origin(xyz=xyz, rpy=(roll, 0.0, 0.0))

    part.visual(
        Box((0.060, 0.20, 0.68)),
        origin=blade_origin((0.09, 0.0, 0.34)),
        material=material,
        name=f"blade_{blade_index}_root",
    )
    part.visual(
        Box((0.055, 0.44, 0.080)),
        origin=blade_origin((0.08, 0.0, 0.70)),
        material=material,
        name=f"blade_{blade_index}_root_spreader",
    )
    for spar_name, y_pos in (("leading_spar", -0.18), ("trailing_spar", 0.18)):
        part.visual(
            Box((0.048, 0.080, 2.35)),
            origin=blade_origin((0.03, y_pos, 1.78)),
            material=material,
            name=f"blade_{blade_index}_{spar_name}",
        )
    part.visual(
        Box((0.042, 0.46, 0.075)),
        origin=blade_origin((0.03, 0.0, 2.88)),
        material=material,
        name=f"blade_{blade_index}_outer_frame",
    )
    for slat_index, slat_z in enumerate((0.95, 1.25, 1.55, 1.85, 2.15, 2.45)):
        part.visual(
            Box((0.022, 0.35, 0.032)),
            origin=blade_origin((0.03, 0.0, slat_z)),
            material=material,
            name=f"blade_{blade_index}_slat_{slat_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.74, 0.72, 0.67, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.46, 0.34, 0.22, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.23, 0.17, 0.11, 1.0))
    tar_roof = model.material("tar_roof", rgba=(0.18, 0.16, 0.14, 1.0))
    iron = model.material("iron", rgba=(0.24, 0.24, 0.26, 1.0))

    cap_sections = [
        _yz_section(-1.05, 1.50, 0.62, 0.16, 0.18),
        _yz_section(-0.35, 2.00, 0.88, 0.20, 0.18),
        _yz_section(0.45, 2.26, 1.10, 0.26, 0.18),
        _yz_section(1.05, 1.86, 0.90, 0.20, 0.18),
        _yz_section(1.42, 1.08, 0.58, 0.14, 0.18),
    ]
    cap_shell = mesh_from_geometry(section_loft(cap_sections), "cap_shell")
    tail_beam = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-1.02, 0.0, 0.54),
                (-1.48, 0.0, 0.58),
                (-2.08, 0.0, 0.62),
                (-2.75, 0.0, 0.66),
            ],
            radius=0.060,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "tail_beam",
    )

    tower_body = model.part("tower_body")
    tower_body.visual(
        Cylinder(radius=2.35, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=stone,
        name="foundation_ring",
    )
    tower_body.visual(
        Cylinder(radius=2.10, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=stone,
        name="tower_stage_0",
    )
    tower_body.visual(
        Cylinder(radius=1.95, length=1.55),
        origin=Origin(xyz=(0.0, 0.0, 1.82)),
        material=stone,
        name="tower_stage_1",
    )
    tower_body.visual(
        Cylinder(radius=1.78, length=1.95),
        origin=Origin(xyz=(0.0, 0.0, 3.45)),
        material=stone,
        name="tower_stage_2",
    )
    tower_body.visual(
        Cylinder(radius=1.58, length=1.85),
        origin=Origin(xyz=(0.0, 0.0, 5.28)),
        material=stone,
        name="tower_stage_3",
    )
    tower_body.visual(
        Cylinder(radius=1.40, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, 6.58)),
        material=stone,
        name="tower_stage_4",
    )
    tower_body.visual(
        Cylinder(radius=1.52, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 6.98)),
        material=weathered_wood,
        name="turntable_curb",
    )
    tower_body.visual(
        Cylinder(radius=0.22, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 7.22)),
        material=dark_wood,
        name="central_support",
    )
    tower_body.inertial = Inertial.from_geometry(
        Box((4.70, 4.70, 7.40)),
        mass=8200.0,
        origin=Origin(xyz=(0.0, 0.0, 3.70)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=1.42, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_wood,
        name="turntable_ring",
    )
    cap.visual(cap_shell, material=tar_roof, name="cap_shell")
    cap.visual(
        Cylinder(radius=0.24, length=0.46),
        origin=Origin(xyz=(1.37, 0.0, 0.80), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="windshaft_bearing",
    )
    cap.visual(tail_beam, material=weathered_wood, name="tail_beam")
    cap.inertial = Inertial.from_geometry(
        Box((4.20, 2.50, 1.35)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.10, length=0.44),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="shaft_spindle",
    )
    sail_hub.visual(
        Cylinder(radius=0.27, length=0.34),
        origin=Origin(xyz=(0.21, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="hub_drum",
    )
    sail_hub.visual(
        Box((0.12, 0.68, 0.68)),
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        material=dark_wood,
        name="hub_cross_block",
    )
    for blade_index in range(4):
        _add_blade(
            sail_hub,
            blade_index=blade_index,
            roll=blade_index * (math.pi / 2.0),
            material=weathered_wood,
        )
    sail_hub.inertial = Inertial.from_geometry(
        Box((0.70, 6.10, 6.10)),
        mass=420.0,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 7.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.35),
    )
    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(1.60, 0.0, 0.80)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_body = object_model.get_part("tower_body")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    cap_turn = object_model.get_articulation("tower_to_cap")
    hub_turn = object_model.get_articulation("cap_to_hub")

    def aabb_center(bounds):
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((low + high) * 0.5 for low, high in zip(lower, upper))

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_gap(
        cap,
        tower_body,
        axis="z",
        positive_elem="turntable_ring",
        negative_elem="central_support",
        max_gap=0.001,
        max_penetration=0.0,
        name="cap ring sits directly on the central support",
    )
    ctx.expect_overlap(
        cap,
        tower_body,
        axes="xy",
        elem_a="turntable_ring",
        elem_b="central_support",
        min_overlap=0.40,
        name="central support stays under the rotating cap stage",
    )
    ctx.expect_origin_gap(
        sail_hub,
        cap,
        axis="x",
        min_gap=1.45,
        max_gap=1.75,
        name="hub projects forward from the cap face",
    )

    rest_hub_pos = ctx.part_world_position(sail_hub)
    with ctx.pose({cap_turn: math.pi / 2.0}):
        turned_hub_pos = ctx.part_world_position(sail_hub)
    ctx.check(
        "cap rotates the whole sail assembly around the tower axis",
        rest_hub_pos is not None
        and turned_hub_pos is not None
        and rest_hub_pos[0] > 1.5
        and abs(rest_hub_pos[1]) < 0.05
        and turned_hub_pos[1] > 1.5
        and abs(turned_hub_pos[0]) < 0.05,
        details=f"rest_hub_pos={rest_hub_pos}, turned_hub_pos={turned_hub_pos}",
    )

    blade_rest = ctx.part_element_world_aabb(sail_hub, elem="blade_0_outer_frame")
    with ctx.pose({hub_turn: math.pi / 2.0}):
        blade_spun = ctx.part_element_world_aabb(sail_hub, elem="blade_0_outer_frame")
    blade_rest_center = aabb_center(blade_rest)
    blade_spun_center = aabb_center(blade_spun)
    ctx.check(
        "hub rotation spins a sail around the windshaft axis",
        blade_rest_center is not None
        and blade_spun_center is not None
        and blade_rest_center[2] > blade_spun_center[2] + 2.5
        and blade_spun_center[1] < -2.5,
        details=f"blade_rest_center={blade_rest_center}, blade_spun_center={blade_spun_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
