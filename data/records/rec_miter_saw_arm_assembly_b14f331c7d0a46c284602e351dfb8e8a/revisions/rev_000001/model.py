from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_sector_profile(
    *,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    samples: int = 20,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    for idx in range(samples + 1):
        theta = start_angle + (end_angle - start_angle) * idx / samples
        y = outer_radius * cos(theta)
        z = outer_radius * sin(theta)
        profile.append((z, y))
    for idx in range(samples, -1, -1):
        theta = start_angle + (end_angle - start_angle) * idx / samples
        y = inner_radius * cos(theta)
        z = inner_radius * sin(theta)
        profile.append((z, y))
    return profile


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        0.5 * (low[0] + high[0]),
        0.5 * (low[1] + high[1]),
        0.5 * (low[2] + high[2]),
    )


def _aabb_span(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
    axis: str,
) -> float | None:
    if aabb is None:
        return None
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    low, high = aabb
    return high[axis_index] - low[axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_miter_saw")

    model.material("cast_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    model.material("machined_gray", rgba=(0.66, 0.67, 0.69, 1.0))
    model.material("dark_gray", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("guard_red", rgba=(0.67, 0.14, 0.10, 1.0))
    model.material("blade_steel", rgba=(0.82, 0.83, 0.86, 1.0))
    model.material("warning_yellow", rgba=(0.84, 0.73, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.74, 0.48, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="cast_gray",
        name="base_plinth",
    )
    base.visual(
        Box((0.48, 0.40, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material="machined_gray",
        name="upper_deck",
    )
    base.visual(
        Box((0.08, 0.26, 0.06)),
        origin=Origin(xyz=(-0.24, 0.0, 0.08)),
        material="cast_gray",
        name="left_leg_web",
    )
    base.visual(
        Box((0.08, 0.26, 0.06)),
        origin=Origin(xyz=(0.24, 0.0, 0.08)),
        material="cast_gray",
        name="right_leg_web",
    )
    base.visual(
        Box((0.62, 0.04, 0.09)),
        origin=Origin(xyz=(0.0, 0.20, 0.125)),
        material="machined_gray",
        name="rear_fence",
    )
    base.visual(
        Box((0.12, 0.035, 0.07)),
        origin=Origin(xyz=(-0.29, 0.19, 0.115)),
        material="machined_gray",
        name="left_fence_wing",
    )
    base.visual(
        Box((0.12, 0.035, 0.07)),
        origin=Origin(xyz=(0.29, 0.19, 0.115)),
        material="machined_gray",
        name="right_fence_wing",
    )
    base.visual(
        Box((0.18, 0.09, 0.16)),
        origin=Origin(xyz=(0.0, 0.215, 0.165)),
        material="dark_gray",
        name="rear_tower",
    )
    base.visual(
        Box((0.10, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.225, 0.065)),
        material="dark_gray",
        name="stop_hinge_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.74, 0.48, 0.16)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.165, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="machined_gray",
        name="turntable_disc",
    )
    turntable.visual(
        Cylinder(radius=0.055, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material="dark_gray",
        name="turntable_hub",
    )
    turntable.visual(
        Box((0.05, 0.27, 0.006)),
        origin=Origin(xyz=(0.0, -0.01, 0.015)),
        material="warning_yellow",
        name="cutline_insert",
    )
    turntable.visual(
        Box((0.10, 0.07, 0.02)),
        origin=Origin(xyz=(0.0, -0.145, 0.012)),
        material="dark_gray",
        name="front_detent",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.165, length=0.028),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_gray",
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.024, 0.29, 0.048)),
        origin=Origin(xyz=(-0.071, -0.105, 0.108), rpy=(-0.78, 0.0, 0.0)),
        material="dark_gray",
        name="left_arm_rail",
    )
    arm.visual(
        Box((0.024, 0.29, 0.048)),
        origin=Origin(xyz=(0.071, -0.105, 0.108), rpy=(-0.78, 0.0, 0.0)),
        material="dark_gray",
        name="right_arm_rail",
    )
    arm.visual(
        Box((0.038, 0.085, 0.09)),
        origin=Origin(xyz=(-0.067, -0.20, 0.22)),
        material="dark_gray",
        name="left_bevel_plate",
    )
    arm.visual(
        Box((0.038, 0.085, 0.09)),
        origin=Origin(xyz=(0.067, -0.20, 0.22)),
        material="dark_gray",
        name="right_bevel_plate",
    )
    arm.visual(
        Box((0.20, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.02, 0.07)),
        material="dark_gray",
        name="upper_bridge",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.18, 0.30, 0.14)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.10, 0.11)),
    )

    upper_guard = ExtrudeGeometry.centered(
        _annular_sector_profile(
            outer_radius=0.155,
            inner_radius=0.132,
            start_angle=-0.20 * pi,
            end_angle=1.18 * pi,
            samples=24,
        ),
        0.05,
        cap=True,
        closed=True,
    )
    upper_guard.rotate_y(pi / 2.0).translate(0.0, 0.0, -0.18)

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.048, length=0.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_gray",
        name="bevel_hub",
    )
    head.visual(
        Box((0.07, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, -0.01, -0.005)),
        material="dark_gray",
        name="bevel_knuckle",
    )
    head.visual(
        Box((0.14, 0.07, 0.06)),
        origin=Origin(xyz=(0.085, -0.11, -0.025)),
        material="dark_gray",
        name="gearbox_bridge",
    )
    head.visual(
        Box((0.06, 0.08, 0.09)),
        origin=Origin(xyz=(0.005, -0.075, -0.06)),
        material="guard_red",
        name="guard_mount",
    )
    head.visual(
        Box((0.07, 0.12, 0.10)),
        origin=Origin(xyz=(0.115, -0.13, 0.045)),
        material="dark_gray",
        name="upper_spine",
    )
    head.visual(
        Cylinder(radius=0.06, length=0.18),
        origin=Origin(xyz=(0.19, -0.13, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="rubber_black",
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.12),
        origin=Origin(xyz=(0.015, -0.075, -0.12), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_gray",
        name="blade_gearcase",
    )
    head.visual(
        Box((0.035, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.025, -0.05)),
        material="guard_red",
        name="guard_spine",
    )
    head.visual(
        mesh_from_geometry(upper_guard, "upper_blade_guard"),
        material="guard_red",
        name="upper_blade_guard",
    )
    head.visual(
        Cylinder(radius=0.13, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.18), rpy=(0.0, pi / 2.0, 0.0)),
        material="blade_steel",
        name="blade_disc",
    )
    head.visual(
        Box((0.04, 0.16, 0.035)),
        origin=Origin(xyz=(0.15, -0.16, 0.11), rpy=(-0.75, 0.0, 0.0)),
        material="rubber_black",
        name="top_handle",
    )
    head.visual(
        Box((0.038, 0.05, 0.10)),
        origin=Origin(xyz=(0.15, -0.23, 0.09)),
        material="rubber_black",
        name="front_grip",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.36)),
        mass=11.0,
        origin=Origin(xyz=(0.08, 0.0, -0.05)),
    )

    stop_arm = model.part("stop_arm")
    stop_arm.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_gray",
        name="stop_hinge_collar",
    )
    stop_arm.visual(
        Box((0.035, 0.28, 0.02)),
        origin=Origin(xyz=(0.0, -0.15, 0.0)),
        material="machined_gray",
        name="stop_bar",
    )
    stop_arm.visual(
        Box((0.06, 0.02, 0.07)),
        origin=Origin(xyz=(0.0, -0.28, 0.025)),
        material="warning_yellow",
        name="stop_tab",
    )
    stop_arm.inertial = Inertial.from_geometry(
        Box((0.06, 0.30, 0.08)),
        mass=1.5,
        origin=Origin(xyz=(0.0, -0.15, 0.025)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.8,
            lower=-0.96,
            upper=1.05,
        ),
    )
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.215, 0.29)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.6,
            lower=-0.40,
            upper=0.50,
        ),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.0, -0.20, 0.22)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-0.82,
            upper=0.82,
        ),
    )
    model.articulation(
        "base_to_stop_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stop_arm,
        origin=Origin(xyz=(0.0, -0.256, 0.08)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.20,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    stop_arm = object_model.get_part("stop_arm")

    miter_joint = object_model.get_articulation("base_to_turntable")
    arm_joint = object_model.get_articulation("base_to_arm")
    bevel_joint = object_model.get_articulation("arm_to_head")
    stop_joint = object_model.get_articulation("base_to_stop_arm")

    with ctx.pose({miter_joint: 0.0, arm_joint: 0.0, bevel_joint: 0.0, stop_joint: 0.0}):
        ctx.expect_contact(
            turntable,
            base,
            elem_a="turntable_disc",
            elem_b="upper_deck",
            contact_tol=0.002,
            name="turntable rests on the machined deck",
        )
        ctx.expect_gap(
            head,
            turntable,
            axis="z",
            positive_elem="blade_disc",
            negative_elem="turntable_disc",
            min_gap=0.07,
            name="raised blade clears the turntable",
        )
        ctx.expect_contact(
            stop_arm,
            base,
            elem_a="stop_hinge_collar",
            elem_b="stop_hinge_boss",
            contact_tol=0.0025,
            name="stop arm mounts at the front hinge boss",
        )
        ctx.expect_contact(
            arm,
            head,
            elem_a="left_bevel_plate",
            elem_b="bevel_hub",
            contact_tol=0.001,
            name="bevel head rides on the arm trunnion plates",
        )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({arm_joint: 0.25}):
        raised_head_pos = ctx.part_world_position(head)
    ctx.check(
        "arm positive motion raises the saw head",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.035,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    front_detent_rest = _aabb_center(ctx.part_element_world_aabb(turntable, elem="front_detent"))
    with ctx.pose({miter_joint: 0.65}):
        front_detent_turned = _aabb_center(ctx.part_element_world_aabb(turntable, elem="front_detent"))
    ctx.check(
        "turntable rotates around the vertical miter axis",
        front_detent_rest is not None
        and front_detent_turned is not None
        and abs(front_detent_turned[0] - front_detent_rest[0]) > 0.07
        and abs(front_detent_turned[2] - front_detent_rest[2]) < 0.01,
        details=f"rest={front_detent_rest}, turned={front_detent_turned}",
    )

    blade_upright = ctx.part_element_world_aabb(head, elem="blade_disc")
    with ctx.pose({bevel_joint: 0.60}):
        blade_beveled = ctx.part_element_world_aabb(head, elem="blade_disc")
    upright_x_span = _aabb_span(blade_upright, "x")
    beveled_x_span = _aabb_span(blade_beveled, "x")
    ctx.check(
        "bevel joint tilts the blade laterally",
        upright_x_span is not None
        and beveled_x_span is not None
        and beveled_x_span > upright_x_span + 0.12,
        details=f"upright_x_span={upright_x_span}, beveled_x_span={beveled_x_span}",
    )

    stop_rest = _aabb_center(ctx.part_element_world_aabb(stop_arm, elem="stop_bar"))
    with ctx.pose({stop_joint: 0.60}):
        stop_folded = _aabb_center(ctx.part_element_world_aabb(stop_arm, elem="stop_bar"))
    ctx.check(
        "stop arm folds upward from the base front edge",
        stop_rest is not None
        and stop_folded is not None
        and stop_folded[2] > stop_rest[2] + 0.06
        and stop_folded[1] > stop_rest[1] - 0.02,
        details=f"rest={stop_rest}, folded={stop_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
