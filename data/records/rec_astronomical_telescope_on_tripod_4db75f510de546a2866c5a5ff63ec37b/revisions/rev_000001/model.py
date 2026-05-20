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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 64,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, z_min),
                (outer_radius, z_max),
            ],
            [
                (inner_radius, z_min),
                (inner_radius, z_max),
            ],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dobsonian_reflector")

    birch_ply = model.material("birch_ply", rgba=(0.45, 0.33, 0.20, 1.0))
    dark_laminate = model.material("dark_laminate", rgba=(0.20, 0.14, 0.10, 1.0))
    powder_black = model.material("powder_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.49, 0.52, 1.0))
    aluminum = model.material("aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.72, 0.78, 0.86, 1.0))

    mirror_cell_shell = _shell_mesh(
        "mirror_cell_shell",
        outer_radius=0.255,
        inner_radius=0.225,
        z_min=-0.30,
        z_max=0.05,
    )
    upper_cage_bottom_ring = _shell_mesh(
        "upper_cage_bottom_ring",
        outer_radius=0.235,
        inner_radius=0.215,
        z_min=1.04,
        z_max=1.08,
    )
    upper_cage_top_ring = _shell_mesh(
        "upper_cage_top_ring",
        outer_radius=0.235,
        inner_radius=0.215,
        z_min=1.18,
        z_max=1.22,
    )

    ground_board = model.part("ground_board")
    ground_board.visual(
        Box((0.98, 0.88, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=birch_ply,
        name="ground_board_panel",
    )
    ground_board.visual(
        Box((0.74, 0.64, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0405)),
        material=dark_laminate,
        name="azimuth_wear_surface",
    )

    rocker_box = model.part("rocker_box")
    rocker_box.visual(
        Cylinder(radius=0.19, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="azimuth_bearing",
    )
    rocker_box.visual(
        Box((0.69, 0.64, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=birch_ply,
        name="rocker_base_panel",
    )
    rocker_box.visual(
        Box((0.03, 0.64, 0.76)),
        origin=Origin(xyz=(-0.30, 0.0, 0.44)),
        material=birch_ply,
        name="left_wall",
    )
    rocker_box.visual(
        Box((0.03, 0.64, 0.76)),
        origin=Origin(xyz=(0.30, 0.0, 0.44)),
        material=birch_ply,
        name="right_wall",
    )
    rocker_box.visual(
        Box((0.57, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, 0.305, 0.17)),
        material=birch_ply,
        name="front_panel",
    )
    rocker_box.visual(
        Box((0.57, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, -0.305, 0.17)),
        material=birch_ply,
        name="rear_panel",
    )
    rocker_box.visual(
        Box((0.57, 0.03, 0.10)),
        origin=Origin(xyz=(0.0, -0.305, 0.63)),
        material=dark_laminate,
        name="rear_brace",
    )

    ota = model.part("optical_tube_assembly")
    ota.visual(
        mirror_cell_shell,
        material=powder_black,
        name="mirror_cell_shell",
    )
    ota.visual(
        Cylinder(radius=0.205, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=mirror_glass,
        name="primary_mirror",
    )
    ota.visual(
        Box((0.45, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        material=steel,
        name="mirror_support_cross_x",
    )
    ota.visual(
        Box((0.04, 0.45, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        material=steel,
        name="mirror_support_cross_y",
    )
    ota.visual(
        Cylinder(radius=0.045, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=steel,
        name="mirror_support_hub",
    )
    ota.visual(
        Cylinder(radius=0.27, length=0.022),
        origin=Origin(xyz=(-0.273, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_laminate,
        name="left_trunnion",
    )
    ota.visual(
        Cylinder(radius=0.27, length=0.022),
        origin=Origin(xyz=(0.273, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_laminate,
        name="right_trunnion",
    )
    ota.visual(
        Box((0.06, 0.22, 0.14)),
        origin=Origin(xyz=(-0.255, 0.0, -0.03)),
        material=birch_ply,
        name="left_trunnion_bracket",
    )
    ota.visual(
        Box((0.06, 0.22, 0.14)),
        origin=Origin(xyz=(0.255, 0.0, -0.03)),
        material=birch_ply,
        name="right_trunnion_bracket",
    )

    truss_radius = 0.216
    truss_length = 1.04
    truss_center_z = 0.54
    for index in range(8):
        angle = (math.tau * index) / 8.0
        ota.visual(
            Cylinder(radius=0.0095, length=truss_length),
            origin=Origin(
                xyz=(
                    truss_radius * math.cos(angle),
                    truss_radius * math.sin(angle),
                    truss_center_z,
                )
            ),
            material=aluminum,
            name=f"truss_{index:02d}",
        )

    ota.visual(
        upper_cage_bottom_ring,
        material=powder_black,
        name="upper_cage_bottom_ring",
    )
    ota.visual(
        upper_cage_top_ring,
        material=powder_black,
        name="upper_cage_top_ring",
    )

    cage_strut_z = 1.13
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        ota.visual(
            Box((0.028, 0.014, 0.10)),
            origin=Origin(
                xyz=(
                    0.225 * math.cos(angle),
                    0.225 * math.sin(angle),
                    cage_strut_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_black,
            name=f"cage_strut_{index:02d}",
        )

    ota.visual(
        Cylinder(radius=0.033, length=0.11),
        origin=Origin(xyz=(0.278, 0.0, 1.11), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="focuser_tube",
    )
    ota.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        material=steel,
        name="secondary_hub",
    )
    ota.visual(
        Box((0.43, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        material=steel,
        name="spider_vane_x",
    )
    ota.visual(
        Box((0.006, 0.43, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        material=steel,
        name="spider_vane_y",
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=ground_board,
        child=rocker_box,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.2),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=rocker_box,
        child=ota,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.7,
            lower=-1.35,
            upper=0.10,
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

    ground_board = object_model.get_part("ground_board")
    rocker_box = object_model.get_part("rocker_box")
    ota = object_model.get_part("optical_tube_assembly")
    azimuth = object_model.get_articulation("azimuth_rotation")
    altitude = object_model.get_articulation("altitude_axis")

    ctx.check(
        "dobsonian primary parts exist",
        all(part is not None for part in (ground_board, rocker_box, ota)),
        details="Expected ground board, rocker box, and optical tube assembly.",
    )
    ctx.check(
        "azimuth joint is continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={azimuth.articulation_type}",
    )
    ctx.check(
        "altitude joint has realistic range",
        altitude.motion_limits is not None
        and altitude.motion_limits.lower is not None
        and altitude.motion_limits.upper is not None
        and altitude.motion_limits.lower <= -1.2
        and altitude.motion_limits.upper >= 0.0,
        details=f"limits={altitude.motion_limits}",
    )

    ctx.expect_gap(
        rocker_box,
        ground_board,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="azimuth_bearing",
        negative_elem="ground_board_panel",
        name="azimuth bearing sits on the ground board",
    )
    ctx.expect_gap(
        ota,
        rocker_box,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="left_trunnion",
        negative_elem="left_wall",
        name="left trunnion nests inside the left rocker wall",
    )
    ctx.expect_gap(
        rocker_box,
        ota,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="right_wall",
        negative_elem="right_trunnion",
        name="right trunnion nests inside the right rocker wall",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    upper_rest = _aabb_center(ctx.part_element_world_aabb(ota, elem="upper_cage_top_ring"))
    with ctx.pose({altitude: -1.0}):
        upper_lowered = _aabb_center(ctx.part_element_world_aabb(ota, elem="upper_cage_top_ring"))
    ctx.check(
        "altitude joint lowers the tube toward the horizon",
        upper_rest is not None
        and upper_lowered is not None
        and upper_lowered[1] > upper_rest[1] + 0.70
        and upper_lowered[2] < upper_rest[2] - 0.40,
        details=f"rest={upper_rest}, lowered={upper_lowered}",
    )

    right_wall_rest = _aabb_center(ctx.part_element_world_aabb(rocker_box, elem="right_wall"))
    with ctx.pose({azimuth: math.pi / 2.0}):
        right_wall_rotated = _aabb_center(ctx.part_element_world_aabb(rocker_box, elem="right_wall"))
    ctx.check(
        "azimuth joint swings the rocker box around the ground board",
        right_wall_rest is not None
        and right_wall_rotated is not None
        and abs(right_wall_rotated[0]) < 0.08
        and right_wall_rotated[1] > 0.22,
        details=f"rest={right_wall_rest}, rotated={right_wall_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
