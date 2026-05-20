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
)


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _add_segment_cylinder(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _build_beauty_dish_shell():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.026, -0.030),
            (0.070, -0.027),
            (0.145, -0.012),
            (0.218, 0.026),
            (0.266, 0.082),
            (0.283, 0.135),
        ],
        [
            (0.036, -0.010),
            (0.082, -0.006),
            (0.155, 0.016),
            (0.225, 0.053),
            (0.263, 0.104),
            (0.275, 0.132),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return mesh_from_geometry(shell, "beauty_dish_shell")


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beauty_dish_boom_arm")

    black_powder = model.material("black_powder", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.73, 0.74, 0.76, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.92, 0.92, 0.90, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.082, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="tripod_hub",
    )
    stand.visual(
        Cylinder(radius=0.034, length=1.74),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=black_powder,
        name="main_post",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.96)),
        material=black_powder,
        name="upper_post",
    )
    stand.visual(
        Box((0.11, 0.11, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 2.004)),
        material=dark_steel,
        name="tilt_head",
    )
    stand.visual(
        Box((0.050, 0.026, 0.086)),
        origin=Origin(xyz=(0.0, 0.068, 2.031)),
        material=dark_steel,
        name="tilt_clamp_left",
    )
    stand.visual(
        Box((0.050, 0.026, 0.086)),
        origin=Origin(xyz=(0.0, -0.068, 2.031)),
        material=dark_steel,
        name="tilt_clamp_right",
    )
    leg_specs = [
        ((0.0, 0.0, 0.11), (0.62, 0.00, 0.025), "leg_front"),
        ((0.0, 0.0, 0.11), (-0.31, 0.54, 0.025), "leg_left"),
        ((0.0, 0.0, 0.11), (-0.31, -0.54, 0.025), "leg_right"),
    ]
    for start, end, name in leg_specs:
        _add_segment_cylinder(
            stand,
            start=start,
            end=end,
            radius=0.018,
            material=black_powder,
            name=name,
        )
        stand.visual(
            Box((0.08, 0.03, 0.018)),
            origin=Origin(
                xyz=(end[0], end[1], 0.009),
                rpy=(0.0, 0.0, math.atan2(end[1], end[0]) if (end[0] or end[1]) else 0.0),
            ),
            material=rubber,
            name=f"{name}_foot",
        )
    stand.inertial = Inertial.from_geometry(
        Box((1.30, 1.20, 2.18)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.028, length=0.11),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_barrel",
    )
    boom.visual(
        Cylinder(radius=0.022, length=1.10),
        origin=Origin(xyz=(0.55, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_powder,
        name="boom_main_tube",
    )
    boom.visual(
        Box((0.08, 0.16, 0.05)),
        origin=Origin(xyz=(1.010, 0.0, -0.055)),
        material=dark_steel,
        name="yoke_mount_block",
    )
    boom.visual(
        Box((0.030, 0.022, 0.105)),
        origin=Origin(xyz=(1.115, 0.210, -0.060)),
        material=dark_steel,
        name="yoke_left",
    )
    boom.visual(
        Box((0.030, 0.022, 0.105)),
        origin=Origin(xyz=(1.115, -0.210, -0.060)),
        material=dark_steel,
        name="yoke_right",
    )
    _add_segment_cylinder(
        boom,
        start=(0.990, 0.055, -0.050),
        end=(1.108, 0.210, -0.082),
        radius=0.013,
        material=dark_steel,
        name="yoke_brace_left",
    )
    _add_segment_cylinder(
        boom,
        start=(0.990, -0.055, -0.050),
        end=(1.108, -0.210, -0.082),
        radius=0.013,
        material=dark_steel,
        name="yoke_brace_right",
    )
    boom.inertial = Inertial.from_geometry(
        Box((1.45, 0.50, 0.22)),
        mass=2.7,
        origin=Origin(xyz=(0.42, 0.0, -0.04)),
    )

    dish = model.part("dish")
    dish.visual(
        Box((0.060, 0.090, 0.052)),
        origin=Origin(xyz=(0.065, 0.0, -0.018)),
        material=dark_steel,
        name="dish_yoke_block",
    )
    dish.visual(
        Box((0.048, 0.154, 0.024)),
        origin=Origin(xyz=(0.024, 0.122, -0.004)),
        material=dark_steel,
        name="dish_wing_left",
    )
    dish.visual(
        Box((0.048, 0.154, 0.024)),
        origin=Origin(xyz=(0.024, -0.122, -0.004)),
        material=dark_steel,
        name="dish_wing_right",
    )
    dish.visual(
        Cylinder(radius=0.058, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=black_powder,
        name="rear_housing",
    )
    dish.visual(
        _build_beauty_dish_shell(),
        origin=Origin(xyz=(0.0, 0.0, -0.085), rpy=(math.pi, 0.0, 0.0)),
        material=reflector_white,
        name="dish_shell",
    )
    dish.visual(
        Cylinder(radius=0.009, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=satin_aluminum,
        name="deflector_stem",
    )
    dish.visual(
        Cylinder(radius=0.060, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.133)),
        material=reflector_white,
        name="deflector_disk",
    )
    dish.inertial = Inertial.from_geometry(
        Box((0.62, 0.62, 0.26)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )

    model.articulation(
        "stand_to_boom",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 2.063)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.1,
            lower=-0.50,
            upper=1.00,
        ),
    )
    model.articulation(
        "boom_to_dish",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=dish,
        origin=Origin(xyz=(1.115, 0.0, -0.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.4,
            lower=-1.00,
            upper=0.90,
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

    stand = object_model.get_part("stand")
    boom = object_model.get_part("boom")
    dish = object_model.get_part("dish")
    boom_joint = object_model.get_articulation("stand_to_boom")
    dish_joint = object_model.get_articulation("boom_to_dish")

    ctx.expect_origin_gap(
        dish,
        stand,
        axis="x",
        min_gap=0.95,
        name="dish sits outboard of the stand post",
    )
    ctx.expect_gap(
        boom,
        dish,
        axis="z",
        positive_elem="boom_main_tube",
        negative_elem="dish_shell",
        min_gap=0.03,
        name="dish shell hangs below the boom tube",
    )
    ctx.expect_contact(
        stand,
        boom,
        contact_tol=0.0005,
        name="boom pivot is supported by the stand head",
    )
    ctx.expect_contact(
        boom,
        dish,
        contact_tol=0.0005,
        name="dish yoke is mounted into the boom end",
    )

    rest_dish_pos = ctx.part_world_position(dish)
    with ctx.pose({boom_joint: 0.60}):
        raised_dish_pos = ctx.part_world_position(dish)
    ctx.check(
        "boom joint raises the dish",
        rest_dish_pos is not None
        and raised_dish_pos is not None
        and raised_dish_pos[2] > rest_dish_pos[2] + 0.40,
        details=f"rest={rest_dish_pos}, raised={raised_dish_pos}",
    )

    rest_shell = ctx.part_element_world_aabb(dish, elem="dish_shell")
    with ctx.pose({dish_joint: 0.55}):
        tilted_shell = ctx.part_element_world_aabb(dish, elem="dish_shell")
    rest_shell_center = _aabb_center(rest_shell)
    tilted_shell_center = _aabb_center(tilted_shell)
    ctx.check(
        "dish tilt sweeps the reflector outward",
        rest_shell_center is not None
        and tilted_shell_center is not None
        and tilted_shell_center[0] > rest_shell_center[0] + 0.05,
        details=f"rest_shell_center={rest_shell_center}, tilted_shell_center={tilted_shell_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
