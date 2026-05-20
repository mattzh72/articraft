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
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _housing_section(
    center_x: float,
    y: float,
    center_z: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + dx, y, center_z + dz)
        for dx, dz in rounded_rect_profile(width, height, radius, corner_segments=6)
    ]


def _half_housing_mesh(side: float):
    sections = [
        _housing_section(side * 0.066, 0.030, 0.079, 0.082, 0.070, 0.012),
        _housing_section(side * 0.060, -0.004, 0.096, 0.082, 0.088, 0.014),
        _housing_section(side * 0.050, -0.040, 0.102, 0.072, 0.082, 0.013),
        _housing_section(side * 0.040, -0.074, 0.096, 0.060, 0.062, 0.010),
    ]
    return section_loft(sections)


def _objective_cap_mesh(name: str):
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.0405, 0.000),
            (0.0398, 0.007),
            (0.0390, 0.013),
        ],
        [
            (0.0350, 0.000),
            (0.0350, 0.0095),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_x(-math.pi / 2.0)
    return _mesh(name, shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_vision_porro_binocular")

    armor_green = model.material("armor_green", rgba=(0.25, 0.29, 0.22, 1.0))
    bridge_green = model.material("bridge_green", rgba=(0.19, 0.22, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.16, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.07, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.26, 0.40, 0.47, 0.50))
    anodized = model.material("anodized", rgba=(0.36, 0.39, 0.40, 1.0))

    left_body = model.part("left_body")
    left_body.visual(_mesh("left_housing_shell", _half_housing_mesh(-1.0)), material=armor_green, name="housing_shell")
    left_body.visual(
        Cylinder(radius=0.034, length=0.108),
        origin=Origin(xyz=(-0.078, 0.083, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_green,
        name="objective_barrel",
    )
    left_body.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(-0.078, 0.132, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bridge_green,
        name="objective_front_ring",
    )
    left_body.visual(
        Cylinder(radius=0.031, length=0.002),
        origin=Origin(xyz=(-0.078, 0.131, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="objective_glass",
    )
    left_body.visual(
        Cylinder(radius=0.020, length=0.058),
        origin=Origin(xyz=(-0.038, -0.069, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_green,
        name="eyepiece_tube",
    )
    left_body.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(-0.038, -0.104, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="eyecup",
    )
    left_body.visual(Box((0.028, 0.040, 0.020)), origin=Origin(xyz=(-0.032, -0.014, 0.102)), material=bridge_green, name="bridge_block")
    left_body.visual(Box((0.024, 0.012, 0.012)), origin=Origin(xyz=(-0.012, 0.000, 0.086)), material=bridge_green, name="hinge_bar_lower")
    left_body.visual(Box((0.024, 0.012, 0.012)), origin=Origin(xyz=(-0.012, 0.000, 0.114)), material=bridge_green, name="hinge_bar_upper")
    left_body.visual(Cylinder(radius=0.008, length=0.016), origin=Origin(xyz=(0.000, 0.000, 0.086)), material=anodized, name="hinge_knuckle_lower")
    left_body.visual(Cylinder(radius=0.008, length=0.016), origin=Origin(xyz=(0.000, 0.000, 0.114)), material=anodized, name="hinge_knuckle_upper")
    left_body.visual(Box((0.030, 0.022, 0.018)), origin=Origin(xyz=(-0.024, -0.008, 0.136)), material=bridge_green, name="focus_mount")
    left_body.visual(
        Cylinder(radius=0.003, length=0.006),
        origin=Origin(xyz=(-0.009, -0.008, 0.136), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="focus_axle_support",
    )
    left_body.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(-0.125, 0.145, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="cap_hinge_boss",
    )
    left_body.visual(Box((0.028, 0.010, 0.008)), origin=Origin(xyz=(-0.110, 0.140, 0.108)), material=bridge_green, name="cap_boss_web")
    left_body.inertial = Inertial.from_geometry(Box((0.155, 0.265, 0.155)), mass=0.62, origin=Origin(xyz=(-0.050, 0.012, 0.090)))

    right_body = model.part("right_body")
    right_body.visual(_mesh("right_housing_shell", _half_housing_mesh(1.0)), material=armor_green, name="housing_shell")
    right_body.visual(
        Cylinder(radius=0.034, length=0.108),
        origin=Origin(xyz=(0.078, 0.083, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_green,
        name="objective_barrel",
    )
    right_body.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(0.078, 0.132, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bridge_green,
        name="objective_front_ring",
    )
    right_body.visual(
        Cylinder(radius=0.031, length=0.002),
        origin=Origin(xyz=(0.078, 0.131, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="objective_glass",
    )
    right_body.visual(
        Cylinder(radius=0.020, length=0.058),
        origin=Origin(xyz=(0.038, -0.069, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_green,
        name="eyepiece_tube",
    )
    right_body.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.038, -0.104, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="eyecup",
    )
    right_body.visual(Box((0.028, 0.040, 0.020)), origin=Origin(xyz=(0.032, -0.014, 0.102)), material=bridge_green, name="bridge_block")
    right_body.visual(Box((0.024, 0.012, 0.012)), origin=Origin(xyz=(0.012, 0.000, 0.100)), material=bridge_green, name="hinge_bar_center")
    right_body.visual(Cylinder(radius=0.008, length=0.012), origin=Origin(xyz=(0.000, 0.000, 0.100)), material=anodized, name="hinge_knuckle_center")
    right_body.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.125, 0.145, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="cap_hinge_boss",
    )
    right_body.visual(Box((0.028, 0.010, 0.008)), origin=Origin(xyz=(0.110, 0.140, 0.108)), material=bridge_green, name="cap_boss_web")
    right_body.inertial = Inertial.from_geometry(Box((0.155, 0.265, 0.155)), mass=0.62, origin=Origin(xyz=(0.050, 0.012, 0.090)))

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(Cylinder(radius=0.009, length=0.014), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=rubber_black, name="wheel_tire")
    focus_wheel.visual(Cylinder(radius=0.005, length=0.006), origin=Origin(xyz=(-0.003, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)), material=graphite, name="wheel_hub")
    focus_wheel.visual(Box((0.004, 0.003, 0.006)), origin=Origin(xyz=(0.000, 0.000, 0.010)), material=graphite, name="wheel_ridge")
    focus_wheel.inertial = Inertial.from_geometry(Box((0.018, 0.022, 0.024)), mass=0.05, origin=Origin())

    left_objective_cap = model.part("left_objective_cap")
    left_objective_cap.visual(Box((0.082, 0.002, 0.078)), origin=Origin(xyz=(0.047, -0.007, -0.041)), material=graphite, name="cap_shell")
    left_objective_cap.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="hinge_knuckle",
    )
    left_objective_cap.visual(Box((0.060, 0.004, 0.006)), origin=Origin(xyz=(0.023, -0.005, -0.008)), material=graphite, name="cap_arm")
    left_objective_cap.inertial = Inertial.from_geometry(Box((0.084, 0.010, 0.084)), mass=0.03, origin=Origin(xyz=(0.000, -0.004, -0.030)))

    right_objective_cap = model.part("right_objective_cap")
    right_objective_cap.visual(Box((0.082, 0.002, 0.078)), origin=Origin(xyz=(-0.047, -0.007, -0.041)), material=graphite, name="cap_shell")
    right_objective_cap.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="hinge_knuckle",
    )
    right_objective_cap.visual(Box((0.060, 0.004, 0.006)), origin=Origin(xyz=(-0.023, -0.005, -0.008)), material=graphite, name="cap_arm")
    right_objective_cap.inertial = Inertial.from_geometry(Box((0.084, 0.010, 0.084)), mass=0.03, origin=Origin(xyz=(0.000, -0.004, -0.030)))

    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.8, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "focus_wheel_spin",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=focus_wheel,
        origin=Origin(xyz=(0.000, -0.008, 0.136)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "left_objective_cap_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=left_objective_cap,
        origin=Origin(xyz=(-0.125, 0.145, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=1.9),
    )
    model.articulation(
        "right_objective_cap_hinge",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=right_objective_cap,
        origin=Origin(xyz=(0.125, 0.145, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=1.9),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_wheel = object_model.get_part("focus_wheel")
    left_objective_cap = object_model.get_part("left_objective_cap")
    right_objective_cap = object_model.get_part("right_objective_cap")

    center_hinge = object_model.get_articulation("center_hinge")
    focus_wheel_spin = object_model.get_articulation("focus_wheel_spin")
    left_cap_hinge = object_model.get_articulation("left_objective_cap_hinge")
    right_cap_hinge = object_model.get_articulation("right_objective_cap_hinge")

    left_hinge_lower = left_body.get_visual("hinge_knuckle_lower")
    left_hinge_upper = left_body.get_visual("hinge_knuckle_upper")
    right_hinge_center = right_body.get_visual("hinge_knuckle_center")
    focus_axle_support = left_body.get_visual("focus_axle_support")
    focus_hub = focus_wheel.get_visual("wheel_hub")
    focus_ridge = focus_wheel.get_visual("wheel_ridge")
    left_cap_boss = left_body.get_visual("cap_hinge_boss")
    left_cap_boss_web = left_body.get_visual("cap_boss_web")
    right_cap_boss = right_body.get_visual("cap_hinge_boss")
    right_cap_boss_web = right_body.get_visual("cap_boss_web")
    left_cap_knuckle = left_objective_cap.get_visual("hinge_knuckle")
    left_cap_arm = left_objective_cap.get_visual("cap_arm")
    right_cap_knuckle = right_objective_cap.get_visual("hinge_knuckle")
    right_cap_arm = right_objective_cap.get_visual("cap_arm")
    left_cap_shell = left_objective_cap.get_visual("cap_shell")
    right_cap_shell = right_objective_cap.get_visual("cap_shell")
    left_objective_ring = left_body.get_visual("objective_front_ring")
    right_objective_ring = right_body.get_visual("objective_front_ring")
    right_objective_barrel = right_body.get_visual("objective_barrel")
    left_objective_barrel = left_body.get_visual("objective_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.allow_overlap(
        left_body,
        left_objective_cap,
        elem_a=left_cap_boss,
        elem_b=left_cap_knuckle,
        reason="Objective cap hinge pin passes through the cap knuckle at the pivot.",
    )
    ctx.allow_overlap(
        right_body,
        right_objective_cap,
        elem_a=right_cap_boss,
        elem_b=right_cap_knuckle,
        reason="Objective cap hinge pin passes through the cap knuckle at the pivot.",
    )
    ctx.allow_overlap(
        left_body,
        left_objective_cap,
        elem_a=left_cap_boss,
        elem_b=left_cap_arm,
        reason="Cap arm represents a coarse hinge yoke wrapping around the body-side hinge boss.",
    )
    ctx.allow_overlap(
        right_body,
        right_objective_cap,
        elem_a=right_cap_boss,
        elem_b=right_cap_arm,
        reason="Cap arm represents a coarse hinge yoke wrapping around the body-side hinge boss.",
    )
    ctx.allow_overlap(
        left_body,
        left_objective_cap,
        elem_a=left_cap_boss_web,
        elem_b=left_cap_arm,
        reason="Body-side hinge support web is simplified and intentionally shares volume with the cap yoke.",
    )
    ctx.allow_overlap(
        right_body,
        right_objective_cap,
        elem_a=right_cap_boss_web,
        elem_b=right_cap_arm,
        reason="Body-side hinge support web is simplified and intentionally shares volume with the cap yoke.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_body,
        right_body,
        elem_a=left_hinge_lower,
        elem_b=right_hinge_center,
        name="center_hinge_lower_contact",
    )
    ctx.expect_contact(
        left_body,
        right_body,
        elem_a=left_hinge_upper,
        elem_b=right_hinge_center,
        name="center_hinge_upper_contact",
    )
    ctx.expect_contact(
        focus_wheel,
        left_body,
        elem_a=focus_hub,
        elem_b=focus_axle_support,
        name="focus_wheel_axle_contact",
    )
    ctx.expect_contact(
        left_objective_cap,
        left_body,
        elem_a=left_cap_knuckle,
        elem_b=left_cap_boss,
        name="left_cap_hinge_contact",
    )
    ctx.expect_contact(
        right_objective_cap,
        right_body,
        elem_a=right_cap_knuckle,
        elem_b=right_cap_boss,
        name="right_cap_hinge_contact",
    )

    with ctx.pose({left_cap_hinge: 0.0, right_cap_hinge: 0.0}):
        ctx.expect_gap(
            left_objective_cap,
            left_body,
            axis="y",
            positive_elem=left_cap_shell,
            negative_elem=left_objective_ring,
            max_gap=0.002,
            max_penetration=1e-5,
            name="left_cap_closed_seating",
        )
        ctx.expect_gap(
            right_objective_cap,
            right_body,
            axis="y",
            positive_elem=right_cap_shell,
            negative_elem=right_objective_ring,
            max_gap=0.002,
            max_penetration=1e-5,
            name="right_cap_closed_seating",
        )
        ctx.expect_overlap(
            left_objective_cap,
            left_body,
            axes="xz",
            elem_a=left_cap_shell,
            elem_b=left_objective_ring,
            min_overlap=0.060,
            name="left_cap_closed_covers_objective",
        )
        ctx.expect_overlap(
            right_objective_cap,
            right_body,
            axes="xz",
            elem_a=right_cap_shell,
            elem_b=right_objective_ring,
            min_overlap=0.060,
            name="right_cap_closed_covers_objective",
        )

    with ctx.pose({left_cap_hinge: 1.8, right_cap_hinge: 1.8}):
        left_open_center = _aabb_center(
            ctx.part_element_world_aabb(left_objective_cap, elem=left_cap_shell)
        )
        left_obj_center = _aabb_center(ctx.part_element_world_aabb(left_body, elem=left_objective_barrel))
        right_open_center = _aabb_center(
            ctx.part_element_world_aabb(right_objective_cap, elem=right_cap_shell)
        )
        right_obj_center = _aabb_center(
            ctx.part_element_world_aabb(right_body, elem=right_objective_barrel)
        )
        if left_open_center is None or left_obj_center is None:
            ctx.fail("left_cap_open_pose_readable", "Missing left cap or objective AABB")
        else:
            ctx.check(
                "left_cap_open_pose_readable",
                left_open_center[2] > left_obj_center[2] + 0.035,
                details=(
                    f"Expected left cap center above objective by > 0.035 m; "
                    f"got cap z={left_open_center[2]:.4f}, objective z={left_obj_center[2]:.4f}"
                ),
            )
        if right_open_center is None or right_obj_center is None:
            ctx.fail("right_cap_open_pose_readable", "Missing right cap or objective AABB")
        else:
            ctx.check(
                "right_cap_open_pose_readable",
                right_open_center[2] > right_obj_center[2] + 0.035,
                details=(
                    f"Expected right cap center above objective by > 0.035 m; "
                    f"got cap z={right_open_center[2]:.4f}, objective z={right_obj_center[2]:.4f}"
                ),
            )

    with ctx.pose({focus_wheel_spin: 0.0}):
        ridge_rest = _aabb_center(ctx.part_element_world_aabb(focus_wheel, elem=focus_ridge))
    with ctx.pose({focus_wheel_spin: 1.5}):
        ridge_turned = _aabb_center(ctx.part_element_world_aabb(focus_wheel, elem=focus_ridge))
    if ridge_rest is None or ridge_turned is None:
        ctx.fail("focus_wheel_rotates", "Missing focus wheel ridge AABB")
    else:
        ctx.check(
            "focus_wheel_rotates",
            abs(ridge_turned[1] - ridge_rest[1]) > 0.0085
            and abs(ridge_turned[2] - ridge_rest[2]) > 0.008,
            details=(
                f"Focus ridge moved too little: rest={ridge_rest}, turned={ridge_turned}"
            ),
        )

    hinge_limits = center_hinge.motion_limits
    if hinge_limits is not None:
        with ctx.pose({center_hinge: hinge_limits.lower}):
            right_narrow = _aabb_center(ctx.part_element_world_aabb(right_body, elem=right_objective_barrel))
            ctx.fail_if_parts_overlap_in_current_pose(name="center_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="center_hinge_lower_no_floating")
        with ctx.pose({center_hinge: hinge_limits.upper}):
            right_wide = _aabb_center(ctx.part_element_world_aabb(right_body, elem=right_objective_barrel))
            ctx.fail_if_parts_overlap_in_current_pose(name="center_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="center_hinge_upper_no_floating")
        if right_narrow is None or right_wide is None:
            ctx.fail("center_hinge_changes_barrel_spacing", "Missing right objective barrel AABB")
        else:
            ctx.check(
                "center_hinge_changes_barrel_spacing",
                abs(right_wide[0] - right_narrow[0]) > 0.010,
                details=(
                    f"Expected right barrel x to change by > 0.010 m across hinge range; "
                    f"narrow={right_narrow[0]:.4f}, wide={right_wide[0]:.4f}"
                ),
            )

    for articulation in (focus_wheel_spin, left_cap_hinge, right_cap_hinge):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    with ctx.pose(
        {
            center_hinge: -0.12,
            left_cap_hinge: 1.8,
            right_cap_hinge: 1.8,
            focus_wheel_spin: 1.3,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="operating_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="operating_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
