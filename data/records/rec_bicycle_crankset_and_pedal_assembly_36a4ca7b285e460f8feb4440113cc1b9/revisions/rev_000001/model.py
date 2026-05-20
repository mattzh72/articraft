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
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _mirror_profile_x(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-x, y) for x, y in reversed(profile)]


def _signed_profile(profile: list[tuple[float, float]], sign: float) -> list[tuple[float, float]]:
    return profile if sign >= 0.0 else _mirror_profile_x(profile)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _gear_profile(outer_radius: float, root_radius: float, teeth: int, phase: float = 0.0) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    total = teeth * 2
    for index in range(total):
        angle = 2.0 * math.pi * index / total + phase
        radius = outer_radius if index % 2 == 0 else root_radius
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _crank_base_web_mesh(sign: float, name: str):
    profile = [
        (0.039, 0.015),
        (0.049, 0.017),
        (0.059, 0.012),
        (0.067, 0.004),
        (0.070, -0.005),
        (0.064, -0.024),
        (0.051, -0.032),
        (0.041, -0.026),
        (0.039, -0.010),
    ]
    geom = ExtrudeGeometry(_signed_profile(profile, sign), 0.015, center=True)
    geom.rotate_x(-math.pi / 2.0)
    return _save_mesh(name, geom)


def _crank_arm_mesh(sign: float, name: str):
    profile = [
        (0.015, 0.010),
        (0.026, 0.005),
        (0.045, -0.020),
        (0.071, -0.070),
        (0.097, -0.126),
        (0.115, -0.160),
        (0.122, -0.170),
        (0.107, -0.176),
        (0.083, -0.158),
        (0.055, -0.104),
        (0.031, -0.038),
        (0.017, -0.006),
    ]
    geom = ExtrudeGeometry(_signed_profile(profile, sign), 0.016, center=True)
    geom.rotate_x(math.pi / 2.0)
    return _save_mesh(name, geom)


def _pedal_platform_mesh(sign: float, name: str):
    outer = _shift_profile(
        _signed_profile(rounded_rect_profile(0.096, 0.058, 0.010, corner_segments=8), sign),
        dx=0.052 * sign,
    )
    hole_a = _shift_profile(
        _signed_profile(rounded_rect_profile(0.024, 0.015, 0.0035, corner_segments=6), sign),
        dx=0.034 * sign,
    )
    hole_b = _shift_profile(
        _signed_profile(rounded_rect_profile(0.024, 0.015, 0.0035, corner_segments=6), sign),
        dx=0.064 * sign,
    )
    geom = ExtrudeWithHolesGeometry(outer, [hole_a, hole_b], 0.012, center=True)
    return _save_mesh(name, geom)


def _chainring_mesh(name: str):
    geom = ExtrudeWithHolesGeometry(
        _gear_profile(0.092, 0.087, teeth=46),
        [_circle_profile(0.032, segments=72)],
        0.004,
        center=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def _spider_plate_mesh(name: str):
    geom = ExtrudeWithHolesGeometry(
        _gear_profile(0.054, 0.032, teeth=5, phase=math.pi / 5.0),
        [_circle_profile(0.029, segments=48)],
        0.010,
        center=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_bike_compact_crankset")

    alloy = model.material("alloy", rgba=(0.76, 0.77, 0.80, 1.0))
    forged_alloy = model.material("forged_alloy", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_shell = model.material("dark_shell", rgba=(0.16, 0.17, 0.18, 1.0))
    chainring_steel = model.material("chainring_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.10, 0.10, 0.11, 1.0))

    shell = model.part("bottom_bracket_shell")
    shell.visual(
        Cylinder(radius=0.0205, length=0.078),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="shell_body",
    )
    shell.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(-0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="left_inner_cup",
    )
    shell.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="right_inner_cup",
    )
    shell.inertial = Inertial.from_geometry(
        Box((0.090, 0.055, 0.055)),
        mass=0.8,
    )

    left_base = model.part("left_crank_base")
    left_base.visual(
        Cylinder(radius=0.0225, length=0.007),
        origin=Origin(xyz=(-0.0425, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="shoulder_hub",
    )
    left_base.visual(_crank_base_web_mesh(-1.0, "left_crank_base_web"), material=forged_alloy, name="base_web")
    left_base.visual(
        Cylinder(radius=0.0155, length=0.014),
        origin=Origin(xyz=(-0.061, 0.0, -0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="shoulder_barrel",
    )
    left_base.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(-0.055, 0.0, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="crank_bolt",
    )
    left_base.inertial = Inertial.from_geometry(
        Box((0.040, 0.050, 0.060)),
        mass=0.35,
        origin=Origin(xyz=(-0.053, 0.0, -0.006)),
    )

    right_base = model.part("right_crank_base")
    right_base.visual(
        Cylinder(radius=0.0225, length=0.007),
        origin=Origin(xyz=(0.0425, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="shoulder_hub",
    )
    right_base.visual(_crank_base_web_mesh(1.0, "right_crank_base_web"), material=forged_alloy, name="base_web")
    right_base.visual(
        Cylinder(radius=0.0155, length=0.014),
        origin=Origin(xyz=(0.061, 0.0, -0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="shoulder_barrel",
    )
    right_base.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.055, 0.0, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="crank_bolt",
    )
    right_base.visual(
        _chainring_mesh("compact_chainring"),
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        material=chainring_steel,
        name="chainring",
    )
    right_base.visual(
        _spider_plate_mesh("right_spider_plate"),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material=forged_alloy,
        name="spider_plate",
    )
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0 - math.pi / 2.0
        bolt_radius = 0.045
        right_base.visual(
            Cylinder(radius=0.0055, length=0.022),
            origin=Origin(
                xyz=(0.062, bolt_radius * math.cos(angle), bolt_radius * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_shell,
            name=f"chainring_bolt_{index + 1}",
        )
    right_base.inertial = Inertial.from_geometry(
        Box((0.075, 0.195, 0.195)),
        mass=0.8,
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
    )

    left_arm = model.part("left_crank_arm")
    left_arm.visual(_crank_arm_mesh(-1.0, "left_crank_arm_body"), material=forged_alloy, name="arm_body")
    left_arm.visual(
        Cylinder(radius=0.017, length=0.007),
        origin=Origin(xyz=(0.0, -0.0105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hinge_knuckle_inner",
    )
    left_arm.visual(
        Cylinder(radius=0.017, length=0.007),
        origin=Origin(xyz=(0.0, 0.0105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hinge_knuckle_outer",
    )
    left_arm.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.124, 0.0, -0.164), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="pedal_boss",
    )
    left_arm.inertial = Inertial.from_geometry(
        Box((0.150, 0.030, 0.190)),
        mass=0.45,
        origin=Origin(xyz=(-0.066, 0.0, -0.086)),
    )

    right_arm = model.part("right_crank_arm")
    right_arm.visual(_crank_arm_mesh(1.0, "right_crank_arm_body"), material=forged_alloy, name="arm_body")
    right_arm.visual(
        Cylinder(radius=0.017, length=0.007),
        origin=Origin(xyz=(0.0, -0.0105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hinge_knuckle_inner",
    )
    right_arm.visual(
        Cylinder(radius=0.017, length=0.007),
        origin=Origin(xyz=(0.0, 0.0105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hinge_knuckle_outer",
    )
    right_arm.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.124, 0.0, -0.164), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_shell,
        name="pedal_boss",
    )
    right_arm.inertial = Inertial.from_geometry(
        Box((0.150, 0.030, 0.190)),
        mass=0.45,
        origin=Origin(xyz=(0.066, 0.0, -0.086)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.0105, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="hinge_sleeve",
    )
    left_pedal.visual(_pedal_platform_mesh(-1.0, "left_pedal_platform"), material=pedal_black, name="platform_body")
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.105, 0.060, 0.022)),
        mass=0.12,
        origin=Origin(xyz=(-0.053, 0.0, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.0105, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="hinge_sleeve",
    )
    right_pedal.visual(_pedal_platform_mesh(1.0, "right_pedal_platform"), material=pedal_black, name="platform_body")
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.105, 0.060, 0.022)),
        mass=0.12,
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
    )

    model.articulation(
        "shell_to_left_base",
        ArticulationType.FIXED,
        parent=shell,
        child=left_base,
        origin=Origin(),
    )
    model.articulation(
        "shell_to_right_base",
        ArticulationType.FIXED,
        parent=shell,
        child=right_base,
        origin=Origin(),
    )
    model.articulation(
        "left_shoulder_fold",
        ArticulationType.REVOLUTE,
        parent=left_base,
        child=left_arm,
        origin=Origin(xyz=(-0.061, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0, lower=-1.05, upper=0.0),
    )
    model.articulation(
        "right_shoulder_fold",
        ArticulationType.REVOLUTE,
        parent=right_base,
        child=right_arm,
        origin=Origin(xyz=(0.061, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "left_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=left_arm,
        child=left_pedal,
        origin=Origin(xyz=(-0.133, 0.0, -0.164)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "right_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=right_arm,
        child=right_pedal,
        origin=Origin(xyz=(0.133, 0.0, -0.164)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bottom_bracket_shell")
    left_base = object_model.get_part("left_crank_base")
    right_base = object_model.get_part("right_crank_base")
    left_arm = object_model.get_part("left_crank_arm")
    right_arm = object_model.get_part("right_crank_arm")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")

    right_base.get_visual("chainring")
    left_pedal.get_visual("platform_body")
    right_pedal.get_visual("platform_body")

    left_shoulder = object_model.get_articulation("left_shoulder_fold")
    right_shoulder = object_model.get_articulation("right_shoulder_fold")
    left_pedal_fold = object_model.get_articulation("left_pedal_fold")
    right_pedal_fold = object_model.get_articulation("right_pedal_fold")

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
    ctx.expect_contact(shell, left_base, elem_b="shoulder_hub", name="left_base_contacts_shell")
    ctx.expect_contact(shell, right_base, elem_b="shoulder_hub", name="right_base_contacts_shell")
    ctx.expect_contact(left_base, left_arm, name="left_arm_hinged_to_left_base")
    ctx.expect_contact(right_base, right_arm, name="right_arm_hinged_to_right_base")
    ctx.expect_contact(left_arm, left_pedal, name="left_pedal_hinged_to_left_arm")
    ctx.expect_contact(right_arm, right_pedal, name="right_pedal_hinged_to_right_arm")
    ctx.expect_origin_gap(shell, left_pedal, axis="z", min_gap=0.13, name="left_pedal_hangs_below_shell")
    ctx.expect_origin_gap(shell, right_pedal, axis="z", min_gap=0.13, name="right_pedal_hangs_below_shell")
    ctx.expect_gap(
        right_base,
        shell,
        axis="x",
        positive_elem="chainring",
        min_gap=0.025,
        max_gap=0.040,
        name="chainring_sits_outboard_of_shell",
    )
    ctx.check("left_shoulder_axis", tuple(left_shoulder.axis) == (0.0, 1.0, 0.0), f"axis={left_shoulder.axis}")
    ctx.check("right_shoulder_axis", tuple(right_shoulder.axis) == (0.0, 1.0, 0.0), f"axis={right_shoulder.axis}")
    ctx.check("left_pedal_axis", tuple(left_pedal_fold.axis) == (1.0, 0.0, 0.0), f"axis={left_pedal_fold.axis}")
    ctx.check("right_pedal_axis", tuple(right_pedal_fold.axis) == (1.0, 0.0, 0.0), f"axis={right_pedal_fold.axis}")
    ctx.allow_overlap(
        left_arm,
        left_pedal,
        elem_a="pedal_boss",
        elem_b="hinge_sleeve",
        reason="Left folding pedal sleeve wraps the pedal boss around the pedal axle.",
    )
    ctx.allow_overlap(
        right_arm,
        right_pedal,
        elem_a="pedal_boss",
        elem_b="hinge_sleeve",
        reason="Right folding pedal sleeve wraps the pedal boss around the pedal axle.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    left_rest = ctx.part_world_position(left_pedal)
    right_rest = ctx.part_world_position(right_pedal)
    if left_rest is None or right_rest is None:
        ctx.fail("pedal_rest_positions_available", "Could not resolve pedal world positions in rest pose.")
    else:
        with ctx.pose({left_shoulder: -0.75, right_shoulder: 0.75}):
            left_folded = ctx.part_world_position(left_pedal)
            right_folded = ctx.part_world_position(right_pedal)
            if left_folded is None or right_folded is None:
                ctx.fail("shoulder_fold_pose_positions_available", "Could not resolve folded pedal positions.")
            else:
                ctx.check(
                    "left_shoulder_folds_inward",
                    left_folded[0] > left_rest[0] + 0.05,
                    f"rest_x={left_rest[0]:.4f}, folded_x={left_folded[0]:.4f}",
                )
                ctx.check(
                    "right_shoulder_folds_inward",
                    right_folded[0] < right_rest[0] - 0.05,
                    f"rest_x={right_rest[0]:.4f}, folded_x={right_folded[0]:.4f}",
                )
                ctx.expect_contact(left_base, left_arm, name="left_arm_stays_seated_when_folded")
                ctx.expect_contact(right_base, right_arm, name="right_arm_stays_seated_when_folded")

    left_platform_open = ctx.part_element_world_aabb(left_pedal, elem="platform_body")
    right_platform_open = ctx.part_element_world_aabb(right_pedal, elem="platform_body")
    if left_platform_open is None or right_platform_open is None:
        ctx.fail("pedal_platform_aabbs_available", "Could not resolve pedal platform extents in rest pose.")
    else:
        with ctx.pose({left_pedal_fold: 1.35, right_pedal_fold: 1.35}):
            left_platform_folded = ctx.part_element_world_aabb(left_pedal, elem="platform_body")
            right_platform_folded = ctx.part_element_world_aabb(right_pedal, elem="platform_body")
            if left_platform_folded is None or right_platform_folded is None:
                ctx.fail("pedal_fold_aabbs_available", "Could not resolve folded pedal platform extents.")
            else:
                ctx.check(
                    "left_pedal_platform_rotates_up",
                    left_platform_folded[1][2] > left_platform_open[1][2] + 0.018,
                    f"open_max_z={left_platform_open[1][2]:.4f}, folded_max_z={left_platform_folded[1][2]:.4f}",
                )
                ctx.check(
                    "right_pedal_platform_rotates_up",
                    right_platform_folded[1][2] > right_platform_open[1][2] + 0.018,
                    f"open_max_z={right_platform_open[1][2]:.4f}, folded_max_z={right_platform_folded[1][2]:.4f}",
                )
                ctx.expect_contact(left_arm, left_pedal, name="left_pedal_stays_seated_when_folded")
                ctx.expect_contact(right_arm, right_pedal, name="right_pedal_stays_seated_when_folded")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
