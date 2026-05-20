from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


try:
    HERE = Path(__file__).resolve().parent
    ASSETS = AssetContext.from_script(__file__)
except Exception:
    HERE = Path("/tmp/articraft_assets")
    HERE.mkdir(parents=True, exist_ok=True)
    ASSETS = AssetContext(HERE)


def _mesh_path(name: str):
    if hasattr(ASSETS, "mesh_path"):
        return ASSETS.mesh_path(name)
    return HERE / name


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, _mesh_path(name))


def _arch_profile(width: float, rise: float, base_z: float, *, samples: int = 18) -> list[tuple[float, float]]:
    half_width = width * 0.5
    return [
        (
            half_width * math.cos(math.pi - (math.pi * index / samples)),
            base_z + rise * math.sin(math.pi - (math.pi * index / samples)),
        )
        for index in range(samples + 1)
    ]


def _shell_section(
    y_pos: float,
    outer_width: float,
    outer_rise: float,
    inner_width: float,
    inner_rise: float,
    *,
    outer_base_z: float = 0.12,
    inner_base_z: float = 0.15,
    samples: int = 18,
) -> list[tuple[float, float, float]]:
    outer = [(x_pos, y_pos, z_pos) for x_pos, z_pos in _arch_profile(outer_width, outer_rise, outer_base_z, samples=samples)]
    inner = [(x_pos, y_pos, z_pos) for x_pos, z_pos in _arch_profile(inner_width, inner_rise, inner_base_z, samples=samples)]
    return outer + list(reversed(inner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wood_fired_oven", assets=ASSETS)

    masonry = model.material("masonry", rgba=(0.71, 0.59, 0.46, 1.0))
    hearth_stone = model.material("hearth_stone", rgba=(0.79, 0.73, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.43, 0.45, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    forged_steel = model.material("forged_steel", rgba=(0.11, 0.11, 0.12, 1.0))

    housing = model.part("housing")
    surround = model.part("surround")
    door = model.part("door")
    handle = model.part("handle")

    housing.visual(
        Box((0.96, 1.04, 0.12)),
        origin=Origin(xyz=(0.0, 0.52, 0.06)),
        material=masonry,
        name="support_base",
    )
    housing.visual(
        Box((0.74, 0.88, 0.03)),
        origin=Origin(xyz=(0.0, 0.50, 0.135)),
        material=hearth_stone,
        name="hearth_floor",
    )
    front_portal = ExtrudeWithHolesGeometry(
        _arch_profile(0.88, 0.50, 0.12),
        [_arch_profile(0.56, 0.34, 0.15)],
        height=0.12,
        center=True,
    ).rotate_x(math.pi / 2.0)
    housing.visual(
        _save_mesh("oven_front_portal.obj", front_portal),
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
        material=masonry,
        name="front_portal",
    )
    oven_shell = section_loft(
        [
            _shell_section(0.12, 0.88, 0.50, 0.56, 0.34),
            _shell_section(0.30, 0.90, 0.56, 0.66, 0.41),
            _shell_section(0.55, 0.86, 0.54, 0.62, 0.39),
            _shell_section(0.80, 0.72, 0.40, 0.44, 0.22),
        ],
        cap=False,
        solid=False,
        repair="mesh",
    )
    housing.visual(
        _save_mesh("oven_shell.obj", oven_shell),
        material=masonry,
        name="oven_shell",
    )
    housing.visual(
        Box((0.72, 0.12, 0.34)),
        origin=Origin(xyz=(0.0, 0.86, 0.29)),
        material=masonry,
        name="back_wall",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.96, 1.04, 0.62)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.52, 0.31)),
    )

    mouth_ring = ExtrudeWithHolesGeometry(
        _arch_profile(0.56, 0.34, 0.12),
        [_arch_profile(0.44, 0.27, 0.15)],
        height=0.028,
        center=True,
    ).rotate_x(math.pi / 2.0)
    surround.visual(
        _save_mesh("oven_mouth_ring.obj", mouth_ring),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="mouth_ring",
    )
    surround.visual(
        Cylinder(radius=0.018, length=0.05),
        origin=Origin(xyz=(-0.265, -0.018, 0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_ear",
    )
    surround.visual(
        Cylinder(radius=0.018, length=0.05),
        origin=Origin(xyz=(0.265, -0.018, 0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_ear",
    )
    surround.visual(
        Cylinder(radius=0.011, length=0.56),
        origin=Origin(xyz=(0.0, -0.018, 0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_rod",
    )
    surround.inertial = Inertial.from_geometry(
        Box((0.56, 0.05, 0.35)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.02, 0.23)),
    )

    door_leaf = ExtrudeGeometry(
        _arch_profile(0.48, 0.30, 0.03),
        0.05,
        center=True,
    ).rotate_x(math.pi / 2.0)
    door.visual(
        _save_mesh("oven_door_leaf.obj", door_leaf),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=dark_steel,
        name="door_leaf",
    )
    door.visual(
        Box((0.06, 0.042, 0.14)),
        origin=Origin(xyz=(-0.262, -0.017, 0.087)),
        material=dark_steel,
        name="left_hinge_bracket",
    )
    door.visual(
        Box((0.06, 0.042, 0.14)),
        origin=Origin(xyz=(0.262, -0.017, 0.087)),
        material=dark_steel,
        name="right_hinge_bracket",
    )
    door.visual(
        Box((0.05, 0.050, 0.030)),
        origin=Origin(xyz=(-0.170, -0.0425, 0.106)),
        material=dark_steel,
        name="left_knuckle",
    )
    door.visual(
        Box((0.05, 0.050, 0.030)),
        origin=Origin(xyz=(0.170, -0.0425, 0.106)),
        material=dark_steel,
        name="right_knuckle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.50, 0.07, 0.34)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.028, 0.17)),
    )

    handle.visual(
        _save_mesh(
            "oven_handle_bar.obj",
            tube_from_spline_points(
                [
                    (-0.115, -0.122, 0.190),
                    (-0.080, -0.147, 0.202),
                    (0.080, -0.147, 0.202),
                    (0.115, -0.122, 0.190),
                ],
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=forged_steel,
        name="forged_bar",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(-0.115, -0.095, 0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=forged_steel,
        name="left_boss",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.115, -0.095, 0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=forged_steel,
        name="right_boss",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.28, 0.05, 0.05)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -0.092, 0.168)),
    )

    model.articulation(
        "housing_to_surround",
        ArticulationType.FIXED,
        parent=housing,
        child=surround,
        origin=Origin(),
    )
    model.articulation(
        "surround_to_door",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=door,
        origin=Origin(xyz=(0.0, -0.018, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.FIXED,
        parent=door,
        child=handle,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    housing = object_model.get_part("housing")
    surround = object_model.get_part("surround")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("surround_to_door")

    front_portal = housing.get_visual("front_portal")
    mouth_ring = surround.get_visual("mouth_ring")
    left_hinge_ear = surround.get_visual("left_hinge_ear")
    right_hinge_ear = surround.get_visual("right_hinge_ear")
    door_leaf = door.get_visual("door_leaf")
    left_boss = handle.get_visual("left_boss")
    right_boss = handle.get_visual("right_boss")
    forged_bar = handle.get_visual("forged_bar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        housing,
        surround,
        elem_a=front_portal,
        elem_b=mouth_ring,
        reason="The steel mouth surround is mortared into the oven's front masonry portal.",
    )
    ctx.allow_overlap(
        door,
        surround,
        elem_a=door.get_visual("left_hinge_bracket"),
        elem_b=left_hinge_ear,
        reason="The left hinge bracket is modeled as a captured hinge leaf interleaved into the left surround ear.",
    )
    ctx.allow_overlap(
        door,
        surround,
        elem_a=door.get_visual("right_hinge_bracket"),
        elem_b=right_hinge_ear,
        reason="The right hinge bracket is modeled as a captured hinge leaf interleaved into the right surround ear.",
    )
    ctx.allow_overlap(
        handle,
        door,
        elem_a=left_boss,
        elem_b=door_leaf,
        reason="The forged handle bosses are welded through the thick steel door leaf.",
    )
    ctx.allow_overlap(
        handle,
        door,
        elem_a=right_boss,
        elem_b=door_leaf,
        reason="The forged handle bosses are welded through the thick steel door leaf.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    housing_aabb = ctx.part_world_aabb(housing)
    mouth_aabb = ctx.part_element_world_aabb(surround, elem=mouth_ring)
    assert housing_aabb is not None
    assert mouth_aabb is not None
    housing_width = housing_aabb[1][0] - housing_aabb[0][0]
    housing_depth = housing_aabb[1][1] - housing_aabb[0][1]
    housing_height = housing_aabb[1][2] - housing_aabb[0][2]
    mouth_width = mouth_aabb[1][0] - mouth_aabb[0][0]
    mouth_height = mouth_aabb[1][2] - mouth_aabb[0][2]
    ctx.check(
        "housing_scale_realistic",
        0.85 <= housing_width <= 1.05 and 0.95 <= housing_depth <= 1.10 and 0.55 <= housing_height <= 0.75,
        details=f"housing dims were {(housing_width, housing_depth, housing_height)}",
    )
    ctx.check(
        "mouth_scale_realistic",
        0.50 <= mouth_width <= 0.62 and 0.30 <= mouth_height <= 0.38,
        details=f"mouth surround dims were {(mouth_width, mouth_height)}",
    )

    ctx.expect_contact(surround, housing, elem_a=mouth_ring, elem_b=front_portal)
    ctx.expect_contact(handle, door, elem_a=left_boss, elem_b=door_leaf)
    ctx.expect_contact(handle, door, elem_a=right_boss, elem_b=door_leaf)
    ctx.expect_within(surround, housing, axes="xz", inner_elem=mouth_ring, outer_elem=front_portal, margin=0.02)
    ctx.expect_within(door, surround, axes="xz", inner_elem=door_leaf, outer_elem=mouth_ring, margin=0.02)
    ctx.expect_gap(
        surround,
        door,
        axis="y",
        min_gap=0.018,
        max_gap=0.030,
        positive_elem=mouth_ring,
        negative_elem=door_leaf,
    )
    ctx.expect_overlap(door, surround, axes="xz", elem_a=door_leaf, elem_b=mouth_ring, min_overlap=0.28)

    rest_bar_aabb = ctx.part_element_world_aabb(handle, elem=forged_bar)
    assert rest_bar_aabb is not None
    rest_bar_center_y = 0.5 * (rest_bar_aabb[0][1] + rest_bar_aabb[1][1])
    rest_bar_center_z = 0.5 * (rest_bar_aabb[0][2] + rest_bar_aabb[1][2])
    limits = door_hinge.motion_limits
    assert limits is not None
    assert limits.lower is not None
    assert limits.upper is not None

    with ctx.pose({door_hinge: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="door_closed_no_floating")

    with ctx.pose({door_hinge: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_unintended_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_gap(
            surround,
            handle,
            axis="y",
            min_gap=0.12,
            positive_elem=mouth_ring,
            negative_elem=forged_bar,
            name="door_swings_forward_when_open",
        )
        ctx.expect_gap(
            surround,
            handle,
            axis="z",
            min_gap=0.008,
            positive_elem=mouth_ring,
            negative_elem=forged_bar,
            name="handle_drops_below_opening_when_open",
        )
        open_bar_aabb = ctx.part_element_world_aabb(handle, elem=forged_bar)
        assert open_bar_aabb is not None
        open_bar_center_y = 0.5 * (open_bar_aabb[0][1] + open_bar_aabb[1][1])
        open_bar_center_z = 0.5 * (open_bar_aabb[0][2] + open_bar_aabb[1][2])
        ctx.check(
            "door_motion_is_downward",
            open_bar_center_z < rest_bar_center_z - 0.05 and open_bar_center_y < rest_bar_center_y - 0.08,
            details=(
                f"bar center moved from {(rest_bar_center_y, rest_bar_center_z)} "
                f"to {(open_bar_center_y, open_bar_center_z)}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
