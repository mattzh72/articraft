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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _ring_sector_profile(
    *,
    inner_radius: float,
    outer_radius: float,
    start_angle: float,
    end_angle: float,
    samples: int = 28,
) -> list[tuple[float, float]]:
    outer = []
    inner = []
    for index in range(samples):
        t = index / (samples - 1)
        angle = start_angle + (end_angle - start_angle) * t
        outer.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle)))
        inner.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle)))
    return outer + list(reversed(inner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chop_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    darker_aluminum = model.material("darker_aluminum", rgba=(0.58, 0.60, 0.63, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.86, 0.87, 0.89, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    guard_orange = model.material("guard_orange", rgba=(0.91, 0.43, 0.13, 1.0))
    accent_red = model.material("accent_red", rgba=(0.72, 0.16, 0.11, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.48, 0.36, 0.26)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )
    base.visual(
        Box((0.44, 0.085, 0.05)),
        origin=Origin(xyz=(0.0, 0.125, 0.025)),
        material=cast_aluminum,
        name="left_base_rail",
    )
    base.visual(
        Box((0.44, 0.085, 0.05)),
        origin=Origin(xyz=(0.0, -0.125, 0.025)),
        material=cast_aluminum,
        name="right_base_rail",
    )
    base.visual(
        Box((0.06, 0.25, 0.05)),
        origin=Origin(xyz=(0.19, 0.0, 0.025)),
        material=cast_aluminum,
        name="front_crossbeam",
    )
    base.visual(
        Box((0.18, 0.23, 0.05)),
        origin=Origin(xyz=(-0.14, 0.0, 0.025)),
        material=cast_aluminum,
        name="rear_bridge",
    )

    table_outer = rounded_rect_profile(0.36, 0.22, 0.022, corner_segments=10)
    blade_slot = _translate_profile(
        rounded_rect_profile(0.18, 0.016, 0.008, corner_segments=8),
        dx=0.06,
        dy=0.0,
    )
    table_geom = ExtrudeWithHolesGeometry(
        table_outer,
        [blade_slot],
        height=0.012,
        center=True,
    )
    base.visual(
        _save_mesh("table_top", table_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=darker_aluminum,
        name="table_top",
    )

    base.visual(
        Box((0.012, 0.13, 0.08)),
        origin=Origin(xyz=(-0.082, 0.095, 0.102)),
        material=steel,
        name="left_fence",
    )
    base.visual(
        Box((0.012, 0.13, 0.08)),
        origin=Origin(xyz=(-0.082, -0.095, 0.102)),
        material=steel,
        name="right_fence",
    )
    base.visual(
        Box((0.07, 0.09, 0.148)),
        origin=Origin(xyz=(-0.195, 0.0, 0.124)),
        material=darker_aluminum,
        name="pivot_column",
    )
    base.visual(
        Box((0.038, 0.036, 0.07)),
        origin=Origin(xyz=(-0.16, 0.047, 0.218)),
        material=darker_aluminum,
        name="left_pivot_ear",
    )
    base.visual(
        Box((0.038, 0.036, 0.07)),
        origin=Origin(xyz=(-0.16, -0.047, 0.218)),
        material=darker_aluminum,
        name="right_pivot_ear",
    )
    base.visual(
        Box((0.085, 0.07, 0.028)),
        origin=Origin(xyz=(-0.16, 0.0, 0.075)),
        material=accent_red,
        name="pivot_cap",
    )

    arm = model.part("arm")
    arm.inertial = Inertial.from_geometry(
        Box((0.40, 0.26, 0.20)),
        mass=6.0,
        origin=Origin(xyz=(0.20, 0.04, 0.02)),
    )
    arm.visual(
        Cylinder(radius=0.022, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="pivot_collar",
    )

    arm_beam = sweep_profile_along_spline(
        [
            (0.0, 0.0, 0.0),
            (0.055, 0.0, 0.010),
            (0.145, 0.0, 0.002),
            (0.225, 0.0, -0.012),
        ],
        profile=rounded_rect_profile(0.052, 0.074, 0.010, corner_segments=8),
        samples_per_segment=16,
        cap_profile=True,
    )
    arm.visual(_save_mesh("arm_beam", arm_beam), material=motor_gray, name="arm_beam")

    arm.visual(
        Box((0.072, 0.05, 0.06)),
        origin=Origin(xyz=(0.208, 0.0, -0.012)),
        material=motor_gray,
        name="spindle_block",
    )

    handle_geom = tube_from_spline_points(
        [
            (0.055, 0.0, 0.032),
            (0.095, 0.0, 0.082),
            (0.165, 0.0, 0.086),
            (0.206, 0.0, 0.028),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    arm.visual(_save_mesh("upper_handle", handle_geom), material=handle_black, name="handle_tube")
    arm.visual(
        Box((0.11, 0.05, 0.045)),
        origin=Origin(xyz=(0.12, 0.0, 0.047)),
        material=handle_black,
        name="handle_body",
    )

    blade_center = Origin(xyz=(0.245, 0.0, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0))
    arm.visual(
        Cylinder(radius=0.105, length=0.003),
        origin=blade_center,
        material=blade_steel,
        name="blade_disc",
    )
    arm.visual(
        Cylinder(radius=0.031, length=0.10),
        origin=Origin(xyz=(0.245, 0.045, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="arbor_housing",
    )

    guard_profile = _ring_sector_profile(
        inner_radius=0.110,
        outer_radius=0.129,
        start_angle=math.radians(-28.0),
        end_angle=math.radians(212.0),
        samples=30,
    )
    guard_geom = ExtrudeGeometry(guard_profile, 0.024, center=True).rotate_x(-math.pi / 2.0)
    arm.visual(
        _save_mesh("blade_guard", guard_geom),
        origin=Origin(xyz=(0.245, -0.002, -0.018)),
        material=guard_orange,
        name="guard_shell",
    )
    arm.visual(
        Box((0.11, 0.024, 0.042)),
        origin=Origin(xyz=(0.194, 0.0, 0.053)),
        material=guard_orange,
        name="guard_bracket",
    )

    arm.visual(
        Cylinder(radius=0.052, length=0.12),
        origin=Origin(xyz=(0.247, 0.13, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=motor_gray,
        name="motor_housing",
    )
    arm.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(0.247, 0.197, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="motor_endcap",
    )
    arm.visual(
        Box((0.06, 0.048, 0.034)),
        origin=Origin(xyz=(0.205, 0.092, -0.012)),
        material=motor_gray,
        name="motor_mount",
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.16, 0.0, 0.232)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(38.0),
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

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    hinge = object_model.get_articulation("arm_hinge")
    base.get_visual("table_top")
    base.get_visual("left_pivot_ear")
    base.get_visual("right_pivot_ear")
    arm.get_visual("blade_disc")
    arm.get_visual("guard_shell")

    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="blade_disc",
        negative_elem="table_top",
        min_gap=0.035,
        max_gap=0.080,
        name="raised blade clears the table",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        elem_a="blade_disc",
        elem_b="table_top",
        min_overlap=0.002,
        name="blade sits over the table slot region",
    )
    ctx.expect_gap(
        arm,
        base,
        axis="x",
        positive_elem="blade_disc",
        negative_elem="left_fence",
        min_gap=0.040,
        max_gap=0.090,
        name="blade is positioned in front of the work fence",
    )

    def _center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    rest_blade = _center(ctx.part_element_world_aabb(arm, elem="blade_disc"))
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        lowered_blade = _center(ctx.part_element_world_aabb(arm, elem="blade_disc"))
        ctx.expect_overlap(
            arm,
            base,
            axes="y",
            elem_a="blade_disc",
            elem_b="table_top",
            min_overlap=0.002,
            name="lowered blade stays centered over the slot line",
        )

    ctx.check(
        "blade descends when the hinge closes",
        rest_blade is not None
        and lowered_blade is not None
        and lowered_blade[2] < rest_blade[2] - 0.12,
        details=f"rest_blade={rest_blade}, lowered_blade={lowered_blade}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
