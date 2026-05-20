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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carillon_bell_tower")

    stone = model.material("stone", rgba=(0.66, 0.66, 0.64, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.55, 0.55, 0.53, 1.0))
    timber = model.material("timber", rgba=(0.39, 0.28, 0.18, 1.0))
    bronze = model.material("bronze", rgba=(0.70, 0.50, 0.22, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.19, 1.0))
    roof_metal = model.material("roof_metal", rgba=(0.28, 0.31, 0.33, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _square_loop(side: float, z: float) -> list[tuple[float, float, float]]:
        half = side * 0.5
        return [
            (-half, -half, z),
            (half, -half, z),
            (half, half, z),
            (-half, half, z),
        ]

    def _roof_geometry() -> object:
        return section_loft(
            [
                _square_loop(5.70, 0.0),
                _square_loop(4.40, 1.20),
                _square_loop(0.80, 2.35),
            ]
        )

    def _bell_shell_geometry(mouth_diameter: float, bell_height: float):
        wall = max(0.018, mouth_diameter * 0.040)
        outer = [
            (mouth_diameter * 0.055, 0.00 * bell_height),
            (mouth_diameter * 0.080, 0.08 * bell_height),
            (mouth_diameter * 0.135, 0.18 * bell_height),
            (mouth_diameter * 0.245, 0.34 * bell_height),
            (mouth_diameter * 0.355, 0.56 * bell_height),
            (mouth_diameter * 0.455, 0.84 * bell_height),
            (mouth_diameter * 0.500, 1.00 * bell_height),
        ]
        inner = [
            (0.0, 0.06 * bell_height),
            (max(0.005, mouth_diameter * 0.040), 0.14 * bell_height),
            (max(0.010, outer[2][0] - wall), 0.22 * bell_height),
            (max(0.015, outer[3][0] - wall), 0.38 * bell_height),
            (max(0.020, outer[4][0] - wall), 0.60 * bell_height),
            (max(0.025, outer[5][0] - wall), 0.86 * bell_height),
            (max(0.030, outer[6][0] - wall), bell_height - wall * 0.6),
        ]
        return LatheGeometry.from_shell_profiles(outer, inner, segments=72)

    tower = model.part("tower")
    tower.inertial = Inertial.from_geometry(
        Box((5.8, 5.8, 18.4)),
        mass=48000.0,
        origin=Origin(xyz=(0.0, 0.0, 9.2)),
    )

    # Stone shaft and masonry stage.
    tower.visual(
        Box((5.60, 5.60, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=stone_dark,
        name="base_plinth",
    )
    tower.visual(
        Box((5.00, 5.00, 11.90)),
        origin=Origin(xyz=(0.0, 0.0, 6.65)),
        material=stone,
        name="shaft",
    )
    tower.visual(
        Box((5.40, 5.40, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 12.80)),
        material=stone_dark,
        name="cornice_band",
    )
    tower.visual(
        Box((4.60, 4.60, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 13.12)),
        material=stone_dark,
        name="belfry_floor",
    )

    pier_xy = 2.33
    pier_size = 0.66
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((pier_size, pier_size, 3.00)),
                origin=Origin(xyz=(x_sign * pier_xy, y_sign * pier_xy, 14.50)),
                material=stone,
                name=f"pier_{'p' if x_sign > 0 else 'n'}x_{'p' if y_sign > 0 else 'n'}y",
            )

    sill_z = 13.53
    sill_h = 0.58
    tower.visual(
        Box((4.10, 0.46, sill_h)),
        origin=Origin(xyz=(0.0, 2.33, sill_z)),
        material=stone,
        name="front_sill",
    )
    tower.visual(
        Box((4.10, 0.46, sill_h)),
        origin=Origin(xyz=(0.0, -2.33, sill_z)),
        material=stone,
        name="rear_sill",
    )
    tower.visual(
        Box((0.46, 4.10, sill_h)),
        origin=Origin(xyz=(2.33, 0.0, sill_z)),
        material=stone,
        name="right_sill",
    )
    tower.visual(
        Box((0.46, 4.10, sill_h)),
        origin=Origin(xyz=(-2.33, 0.0, sill_z)),
        material=stone,
        name="left_sill",
    )

    lintel_z = 15.62
    tower.visual(
        Box((4.10, 0.52, 0.40)),
        origin=Origin(xyz=(0.0, 2.33, lintel_z)),
        material=stone_dark,
        name="front_lintel",
    )
    tower.visual(
        Box((4.10, 0.52, 0.40)),
        origin=Origin(xyz=(0.0, -2.33, lintel_z)),
        material=stone_dark,
        name="rear_lintel",
    )
    tower.visual(
        Box((0.52, 4.10, 0.40)),
        origin=Origin(xyz=(2.33, 0.0, lintel_z)),
        material=stone_dark,
        name="right_lintel",
    )
    tower.visual(
        Box((0.52, 4.10, 0.40)),
        origin=Origin(xyz=(-2.33, 0.0, lintel_z)),
        material=stone_dark,
        name="left_lintel",
    )

    tower.visual(
        Box((5.30, 5.30, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 15.91)),
        material=stone_dark,
        name="roof_seat",
    )
    tower.visual(
        _mesh("tower_roof", _roof_geometry()),
        origin=Origin(xyz=(0.0, 0.0, 16.00)),
        material=roof_metal,
        name="roof",
    )

    # Belfry carrier beams that support the row of headstocks.
    axis_z = 14.62
    tower.visual(
        Box((4.80, 0.22, 0.22)),
        origin=Origin(xyz=(0.0, 0.78, axis_z + 0.34)),
        material=timber,
        name="front_carrier_beam",
    )
    tower.visual(
        Box((4.80, 0.22, 0.22)),
        origin=Origin(xyz=(0.0, -0.78, axis_z + 0.34)),
        material=timber,
        name="rear_carrier_beam",
    )
    for x_pos, y_pos in ((-2.25, 0.78), (2.25, 0.78), (-2.25, -0.78), (2.25, -0.78)):
        tower.visual(
            Box((0.16, 0.12, 0.75)),
            origin=Origin(xyz=(x_pos, y_pos, 15.445)),
            material=timber,
            name=f"brace_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
        )

    bell_specs = [
        ("bell_1", -1.62, 0.82, 0.80),
        ("bell_2", -0.77, 0.72, 0.71),
        ("bell_3", 0.00, 0.63, 0.63),
        ("bell_4", 0.73, 0.55, 0.56),
        ("bell_5", 1.38, 0.48, 0.50),
    ]

    for bell_name, x_pos, mouth_diameter, bell_height in bell_specs:
        headstock_width = max(0.16, mouth_diameter * 0.22)
        headstock_span = 1.14
        tower.visual(
            Box((headstock_width, headstock_span, 0.14)),
            origin=Origin(xyz=(x_pos, 0.0, axis_z + 0.20)),
            material=timber,
            name=f"{bell_name}_headstock",
        )
        for side, y_pos in (("front", 0.62), ("rear", -0.62)):
            tower.visual(
                Box((0.10, 0.12, 0.34)),
                origin=Origin(xyz=(x_pos, y_pos, axis_z + 0.12)),
                material=timber,
                name=f"{bell_name}_{side}_hanger",
            )
        for side, y_pos in (("front", 0.52), ("rear", -0.52)):
            tower.visual(
                Cylinder(radius=0.055, length=0.08),
                origin=Origin(xyz=(x_pos, y_pos, axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_metal,
                name=f"{bell_name}_{side}_bearing",
            )

        bell = model.part(bell_name)
        trunnion_drop = bell_height * 0.18
        shaft_length = 0.96
        strap_width = max(0.08, mouth_diameter * 0.16)
        strap_depth = 0.16
        bell.visual(
            Cylinder(radius=max(0.022, mouth_diameter * 0.035), length=shaft_length),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="axle",
        )
        bell.visual(
            Box((strap_width, strap_depth, trunnion_drop)),
            origin=Origin(xyz=(0.0, 0.0, -trunnion_drop * 0.5)),
            material=dark_metal,
            name="suspension_strap",
        )
        bell.visual(
            Cylinder(radius=max(0.040, mouth_diameter * 0.070), length=0.12),
            origin=Origin(
                xyz=(0.0, 0.0, -trunnion_drop),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name="crown_block",
        )
        bell.visual(
            _mesh(f"{bell_name}_shell", _bell_shell_geometry(mouth_diameter, bell_height)),
            origin=Origin(xyz=(0.0, 0.0, -trunnion_drop), rpy=(math.pi, 0.0, 0.0)),
            material=bronze,
            name="bell_shell",
        )
        bell.inertial = Inertial.from_geometry(
            Cylinder(radius=mouth_diameter * 0.5, length=bell_height),
            mass=220.0 * mouth_diameter,
            origin=Origin(xyz=(0.0, 0.0, -(trunnion_drop + bell_height * 0.48))),
        )
        model.articulation(
            f"{bell_name}_swing",
            ArticulationType.REVOLUTE,
            parent=tower,
            child=bell,
            origin=Origin(xyz=(x_pos, 0.0, axis_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1800.0,
                velocity=1.0,
                lower=-0.35,
                upper=0.35,
            ),
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

    tower = object_model.get_part("tower")
    bells = [object_model.get_part(f"bell_{index}") for index in range(1, 6)]
    swings = [object_model.get_articulation(f"bell_{index}_swing") for index in range(1, 6)]

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.check("tower exists", tower is not None, details="tower part missing")

    for index, (bell, joint) in enumerate(zip(bells, swings), start=1):
        limits = joint.motion_limits
        axis_ok = tuple(joint.axis) == (0.0, 1.0, 0.0)
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        )
        ctx.check(
            f"bell_{index} joint is horizontal revolute",
            axis_ok and limits_ok,
            details=f"axis={joint.axis}, limits={limits}",
        )
        shell_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
        shell_below_axis = shell_aabb is not None and shell_aabb[1][2] < joint.origin.xyz[2] - 0.05
        ctx.check(
            f"bell_{index} hangs below its hinge axis",
            shell_below_axis,
            details=f"shell_aabb={shell_aabb}, axis_z={joint.origin.xyz[2]}",
        )

    shell_widths = []
    for bell in bells:
        shell_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
        shell_widths.append(None if shell_aabb is None else shell_aabb[1][0] - shell_aabb[0][0])
    ctx.check(
        "bell shells step down in size across the set",
        all(
            shell_widths[i] is not None
            and shell_widths[i + 1] is not None
            and shell_widths[i] > shell_widths[i + 1] + 0.02
            for i in range(4)
        ),
        details=f"shell_widths={shell_widths}",
    )

    rest_centers = [
        _aabb_center(ctx.part_element_world_aabb(bell, elem="bell_shell"))
        for bell in bells
    ]
    with ctx.pose({swings[0]: 0.24}):
        swung_center = _aabb_center(ctx.part_element_world_aabb(bells[0], elem="bell_shell"))
        neighbor_center = _aabb_center(ctx.part_element_world_aabb(bells[1], elem="bell_shell"))
    ctx.check(
        "bell_1 swings without dragging bell_2",
        swung_center is not None
        and rest_centers[0] is not None
        and abs(swung_center[0] - rest_centers[0][0]) > 0.05
        and neighbor_center is not None
        and rest_centers[1] is not None
        and abs(neighbor_center[0] - rest_centers[1][0]) < 0.01,
        details=f"rest={rest_centers[0]}, swung={swung_center}, neighbor_rest={rest_centers[1]}, neighbor={neighbor_center}",
    )

    with ctx.pose({swings[4]: -0.24}):
        swung_center = _aabb_center(ctx.part_element_world_aabb(bells[4], elem="bell_shell"))
        neighbor_center = _aabb_center(ctx.part_element_world_aabb(bells[3], elem="bell_shell"))
    ctx.check(
        "bell_5 swings independently of bell_4",
        swung_center is not None
        and rest_centers[4] is not None
        and abs(swung_center[0] - rest_centers[4][0]) > 0.03
        and neighbor_center is not None
        and rest_centers[3] is not None
        and abs(neighbor_center[0] - rest_centers[3][0]) < 0.01,
        details=f"rest={rest_centers[4]}, swung={swung_center}, neighbor_rest={rest_centers[3]}, neighbor={neighbor_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
