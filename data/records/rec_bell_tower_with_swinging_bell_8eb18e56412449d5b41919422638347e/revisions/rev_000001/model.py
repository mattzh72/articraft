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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _bearing_shell(inner_radius: float, outer_radius: float, length: float):
    _ = inner_radius
    return Cylinder(radius=outer_radius, length=length)


def _bell_shell():
    outer_profile = [
        (0.12, 0.00),
        (0.16, -0.03),
        (0.22, -0.12),
        (0.34, -0.30),
        (0.48, -0.66),
        (0.58, -0.98),
        (0.64, -1.18),
        (0.67, -1.30),
    ]
    inner_profile = [
        (0.08, -0.03),
        (0.13, -0.10),
        (0.22, -0.34),
        (0.34, -0.72),
        (0.47, -1.04),
        (0.57, -1.24),
    ]

    def ring(radius: float, z: float, segments: int) -> list[tuple[float, float, float]]:
        return [
            (
                radius * math.cos((2.0 * math.pi * i) / segments),
                radius * math.sin((2.0 * math.pi * i) / segments),
                z,
            )
            for i in range(segments)
        ]

    def add_ring(geom: MeshGeometry, pts: list[tuple[float, float, float]]) -> list[int]:
        return [geom.add_vertex(x, y, z) for x, y, z in pts]

    def bridge_rings(geom: MeshGeometry, ring_a: list[int], ring_b: list[int], flip: bool = False) -> None:
        count = len(ring_a)
        for i in range(count):
            a0 = ring_a[i]
            a1 = ring_a[(i + 1) % count]
            b0 = ring_b[i]
            b1 = ring_b[(i + 1) % count]
            if flip:
                geom.add_face(a0, b1, b0)
                geom.add_face(a0, a1, b1)
            else:
                geom.add_face(a0, b0, b1)
                geom.add_face(a0, b1, a1)

    geom = MeshGeometry()
    segments = 56

    outer_rings = [add_ring(geom, ring(radius, z, segments)) for radius, z in outer_profile]
    inner_rings = [add_ring(geom, ring(radius, z, segments)) for radius, z in inner_profile]

    for idx in range(len(outer_rings) - 1):
        bridge_rings(geom, outer_rings[idx], outer_rings[idx + 1], flip=False)
    for idx in range(len(inner_rings) - 1):
        bridge_rings(geom, inner_rings[idx + 1], inner_rings[idx], flip=False)

    bridge_rings(geom, outer_rings[0], inner_rings[0], flip=False)
    bridge_rings(geom, inner_rings[-1], outer_rings[-1], flip=False)

    return mesh_from_geometry(geom, "bronze_bell_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="english_parish_church_bell_tower")

    stone = model.material("stone", rgba=(0.68, 0.67, 0.63, 1.0))
    darker_stone = model.material("darker_stone", rgba=(0.60, 0.59, 0.55, 1.0))
    oak = model.material("oak", rgba=(0.45, 0.31, 0.17, 1.0))
    bronze = model.material("bronze", rgba=(0.61, 0.43, 0.19, 1.0))
    iron = model.material("iron", rgba=(0.22, 0.22, 0.24, 1.0))

    tower_width = 6.40
    wall_thickness = 0.75
    inner_clear = tower_width - (2.0 * wall_thickness)
    lower_stage_height = 12.65
    opening_bottom = 13.25
    opening_top = 15.45
    opening_height = opening_top - opening_bottom
    opening_width = 1.70
    upper_stage_top = 16.80
    parapet_height = 0.48
    merlon_height = 0.70
    tower_top = upper_stage_top + parapet_height + merlon_height

    tower = model.part("tower")
    tower.visual(
        Box((7.00, 7.00, 0.90)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=darker_stone,
        name="plinth",
    )

    for sign, face_name in ((1.0, "east"), (-1.0, "west")):
        tower.visual(
            Box((wall_thickness, tower_width, lower_stage_height)),
            origin=Origin(
                xyz=(sign * (0.5 * tower_width - 0.5 * wall_thickness), 0.0, 0.5 * lower_stage_height)
            ),
            material=stone,
            name=f"{face_name}_lower_wall",
        )
        tower.visual(
            Box((wall_thickness, tower_width, opening_bottom - lower_stage_height)),
            origin=Origin(
                xyz=(
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    0.0,
                    lower_stage_height + 0.5 * (opening_bottom - lower_stage_height),
                )
            ),
            material=stone,
            name=f"{face_name}_sill_wall",
        )
        tower.visual(
            Box((wall_thickness, 0.5 * (tower_width - opening_width), opening_height)),
            origin=Origin(
                xyz=(
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    0.25 * (tower_width + opening_width),
                    opening_bottom + 0.5 * opening_height,
                )
            ),
            material=stone,
            name=f"{face_name}_north_jamb",
        )
        tower.visual(
            Box((wall_thickness, 0.5 * (tower_width - opening_width), opening_height)),
            origin=Origin(
                xyz=(
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    -0.25 * (tower_width + opening_width),
                    opening_bottom + 0.5 * opening_height,
                )
            ),
            material=stone,
            name=f"{face_name}_south_jamb",
        )
        tower.visual(
            Box((wall_thickness, tower_width, upper_stage_top - opening_top)),
            origin=Origin(
                xyz=(
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    0.0,
                    opening_top + 0.5 * (upper_stage_top - opening_top),
                )
            ),
            material=stone,
            name=f"{face_name}_lintel_wall",
        )
        tower.visual(
            Box((wall_thickness, tower_width, parapet_height)),
            origin=Origin(
                xyz=(
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    0.0,
                    upper_stage_top + 0.5 * parapet_height,
                )
            ),
            material=stone,
            name=f"{face_name}_parapet",
        )

    for sign, face_name in ((1.0, "north"), (-1.0, "south")):
        tower.visual(
            Box((inner_clear, wall_thickness, lower_stage_height)),
            origin=Origin(
                xyz=(0.0, sign * (0.5 * tower_width - 0.5 * wall_thickness), 0.5 * lower_stage_height)
            ),
            material=stone,
            name=f"{face_name}_lower_wall",
        )
        tower.visual(
            Box((inner_clear, wall_thickness, opening_bottom - lower_stage_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    lower_stage_height + 0.5 * (opening_bottom - lower_stage_height),
                )
            ),
            material=stone,
            name=f"{face_name}_sill_wall",
        )
        tower.visual(
            Box((0.5 * (tower_width - opening_width), wall_thickness, opening_height)),
            origin=Origin(
                xyz=(
                    0.25 * (tower_width + opening_width),
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    opening_bottom + 0.5 * opening_height,
                )
            ),
            material=stone,
            name=f"{face_name}_east_jamb",
        )
        tower.visual(
            Box((0.5 * (tower_width - opening_width), wall_thickness, opening_height)),
            origin=Origin(
                xyz=(
                    -0.25 * (tower_width + opening_width),
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    opening_bottom + 0.5 * opening_height,
                )
            ),
            material=stone,
            name=f"{face_name}_west_jamb",
        )
        tower.visual(
            Box((inner_clear, wall_thickness, upper_stage_top - opening_top)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    opening_top + 0.5 * (upper_stage_top - opening_top),
                )
            ),
            material=stone,
            name=f"{face_name}_lintel_wall",
        )
        tower.visual(
            Box((inner_clear, wall_thickness, parapet_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * (0.5 * tower_width - 0.5 * wall_thickness),
                    upper_stage_top + 0.5 * parapet_height,
                )
            ),
            material=stone,
            name=f"{face_name}_parapet",
        )

    tower.visual(
        Box((inner_clear, inner_clear, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 12.55)),
        material=darker_stone,
        name="belfry_floor",
    )

    for sign_x, sign_y, corner_name in (
        (1.0, 1.0, "ne"),
        (1.0, -1.0, "se"),
        (-1.0, 1.0, "nw"),
        (-1.0, -1.0, "sw"),
    ):
        tower.visual(
            Box((0.90, 0.90, merlon_height)),
            origin=Origin(
                xyz=(
                    sign_x * (0.5 * tower_width - 0.45),
                    sign_y * (0.5 * tower_width - 0.45),
                    upper_stage_top + parapet_height + 0.5 * merlon_height,
                )
            ),
            material=stone,
            name=f"corner_merlon_{corner_name}",
        )

    for sign in (1.0, -1.0):
        tower.visual(
            Box((0.90, 0.80, merlon_height)),
            origin=Origin(
                xyz=(
                    sign * (0.5 * tower_width - 0.45),
                    0.0,
                    upper_stage_top + parapet_height + 0.5 * merlon_height,
                )
            ),
            material=stone,
            name=f"east_west_center_merlon_{int(sign)}",
        )
        tower.visual(
            Box((0.80, 0.90, merlon_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * (0.5 * tower_width - 0.45),
                    upper_stage_top + parapet_height + 0.5 * merlon_height,
                )
            ),
            material=stone,
            name=f"north_south_center_merlon_{int(sign)}",
        )

    band_depth = 0.92
    for sign, face_name in ((1.0, "east"), (-1.0, "west")):
        tower.visual(
            Box((band_depth, tower_width + 0.16, 0.18)),
            origin=Origin(
                xyz=(sign * (0.5 * tower_width - 0.5 * band_depth), 0.0, 12.62)
            ),
            material=darker_stone,
            name=f"{face_name}_stringcourse",
        )
    for sign, face_name in ((1.0, "north"), (-1.0, "south")):
        tower.visual(
            Box((inner_clear + 0.18, band_depth, 0.18)),
            origin=Origin(
                xyz=(0.0, sign * (0.5 * tower_width - 0.5 * band_depth), 12.62)
            ),
            material=darker_stone,
            name=f"{face_name}_stringcourse",
        )

    tower.inertial = Inertial.from_geometry(
        Box((7.0, 7.0, tower_top)),
        mass=180000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * tower_top)),
    )

    frame = model.part("belfry_frame")
    sill_z = -1.75
    axle_y = 1.18
    post_height = 2.55

    frame.visual(
        Box((inner_clear, 0.24, 0.24)),
        origin=Origin(xyz=(0.0, 0.96, sill_z)),
        material=oak,
        name="north_sill",
    )
    frame.visual(
        Box((inner_clear, 0.24, 0.24)),
        origin=Origin(xyz=(0.0, -0.96, sill_z)),
        material=oak,
        name="south_sill",
    )
    frame.visual(
        Box((0.24, 2.16, 0.24)),
        origin=Origin(xyz=(0.98, 0.0, sill_z)),
        material=oak,
        name="east_tie",
    )
    frame.visual(
        Box((0.24, 2.16, 0.24)),
        origin=Origin(xyz=(-0.98, 0.0, sill_z)),
        material=oak,
        name="west_tie",
    )

    for px, py, post_name in (
        (0.98, 0.96, "ne"),
        (0.98, -0.96, "se"),
        (-0.98, 0.96, "nw"),
        (-0.98, -0.96, "sw"),
    ):
        frame.visual(
            Box((0.18, 0.18, post_height)),
            origin=Origin(xyz=(px, py, sill_z + 0.12 + 0.5 * post_height)),
            material=oak,
            name=f"post_{post_name}",
        )

    frame.visual(
        Box((2.14, 0.26, 0.22)),
        origin=Origin(xyz=(0.0, axle_y, 0.20)),
        material=oak,
        name="north_head_beam_upper",
    )
    frame.visual(
        Box((2.14, 0.26, 0.22)),
        origin=Origin(xyz=(0.0, axle_y, -0.20)),
        material=oak,
        name="north_head_beam_lower",
    )
    frame.visual(
        Box((2.14, 0.26, 0.22)),
        origin=Origin(xyz=(0.0, -axle_y, 0.20)),
        material=oak,
        name="south_head_beam_upper",
    )
    frame.visual(
        Box((2.14, 0.26, 0.22)),
        origin=Origin(xyz=(0.0, -axle_y, -0.20)),
        material=oak,
        name="south_head_beam_lower",
    )
    frame.visual(
        _bearing_shell(0.058, 0.12, 0.16),
        origin=Origin(xyz=(0.0, axle_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="left_bearing",
    )
    frame.visual(
        _bearing_shell(0.058, 0.12, 0.16),
        origin=Origin(xyz=(0.0, -axle_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="right_bearing",
    )
    frame.inertial = Inertial.from_geometry(
        Box((inner_clear, 2.6, 2.9)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.0, -0.45)),
    )

    bell = model.part("bell_assembly")
    bell.visual(
        Cylinder(radius=0.058, length=3.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_shaft",
    )
    bell.visual(
        Box((0.34, 1.74, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=oak,
        name="headstock",
    )
    bell.visual(
        _bell_shell(),
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
        material=bronze,
        name="bell_shell",
    )
    for sx, sy, strap_name in (
        (0.11, 0.10, "ne"),
        (0.11, -0.10, "se"),
        (-0.11, 0.10, "nw"),
        (-0.11, -0.10, "sw"),
    ):
        bell.visual(
            Box((0.05, 0.12, 0.18)),
            origin=Origin(xyz=(sx, sy, -0.20)),
            material=iron,
            name=f"crown_strap_{strap_name}",
        )
    bell.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.47, tube=0.035, radial_segments=18, tubular_segments=56),
            "ringing_wheel_rim",
        ),
        origin=Origin(xyz=(0.0, 1.45, -0.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oak,
        name="ringing_wheel",
    )
    bell.visual(
        Cylinder(radius=0.08, length=0.20),
        origin=Origin(xyz=(0.0, 1.45, -0.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oak,
        name="wheel_hub",
    )
    for index, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
        bell.visual(
            Box((0.05, 0.05, 0.90)),
            origin=Origin(xyz=(0.0, 1.45, -0.02), rpy=(0.0, angle, 0.0)),
            material=oak,
            name=f"wheel_spoke_{index}",
        )
    bell.inertial = Inertial.from_geometry(
        Box((1.40, 3.10, 1.75)),
        mass=1550.0,
        origin=Origin(xyz=(0.0, 0.55, -0.55)),
    )

    model.articulation(
        "tower_to_frame",
        ArticulationType.FIXED,
        parent=tower,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, 15.0)),
    )
    model.articulation(
        "frame_to_bell",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=1.6,
            lower=-3.20,
            upper=3.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    frame = object_model.get_part("belfry_frame")
    bell = object_model.get_part("bell_assembly")
    swing = object_model.get_articulation("frame_to_bell")

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
    ctx.allow_overlap(
        bell,
        frame,
        elem_a="axle_shaft",
        elem_b="left_bearing",
        reason="The iron bell axle is intentionally represented as seated inside a simplified solid bearing block.",
    )
    ctx.allow_overlap(
        bell,
        frame,
        elem_a="axle_shaft",
        elem_b="right_bearing",
        reason="The iron bell axle is intentionally represented as seated inside a simplified solid bearing block.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    axis = swing.axis
    ctx.check(
        "bell axle is horizontal across the tower",
        axis is not None
        and abs(axis[0]) < 1e-6
        and abs(abs(axis[1]) - 1.0) < 1e-6
        and abs(axis[2]) < 1e-6,
        details=f"axis={axis}",
    )
    ctx.expect_origin_distance(
        frame,
        tower,
        axes="xy",
        max_dist=0.001,
        name="belfry frame stays centered in the tower shaft",
    )
    ctx.expect_contact(
        frame,
        tower,
        elem_a="north_sill",
        contact_tol=0.001,
        name="north sill bears on the tower masonry",
    )
    ctx.expect_contact(
        bell,
        frame,
        elem_a="axle_shaft",
        elem_b="left_bearing",
        contact_tol=0.003,
        name="bell axle seats in the north bearing",
    )
    ctx.expect_contact(
        bell,
        frame,
        elem_a="axle_shaft",
        elem_b="right_bearing",
        contact_tol=0.003,
        name="bell axle seats in the south bearing",
    )

    rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    rest_center_x = None
    if rest_aabb is not None:
        rest_center_x = 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])

    with ctx.pose({swing: 1.10}):
        ctx.expect_contact(
            bell,
            frame,
            elem_a="axle_shaft",
            elem_b="left_bearing",
            contact_tol=0.003,
            name="north bearing stays engaged while ringing",
        )
        swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
        swung_center_x = None
        if swung_aabb is not None:
            swung_center_x = 0.5 * (swung_aabb[0][0] + swung_aabb[1][0])
        ctx.check(
            "positive bell angle swings the bell toward the east opening",
            rest_center_x is not None
            and swung_center_x is not None
            and swung_center_x > rest_center_x + 0.35,
            details=f"rest_center_x={rest_center_x}, swung_center_x={swung_center_x}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
