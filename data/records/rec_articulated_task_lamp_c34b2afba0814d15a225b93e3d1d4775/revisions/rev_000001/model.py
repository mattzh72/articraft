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


def _build_shade_shell():
    outer_profile = [
        (0.022, 0.000),
        (0.034, -0.018),
        (0.054, -0.060),
        (0.074, -0.120),
        (0.088, -0.150),
    ]
    inner_profile = [
        (0.013, -0.004),
        (0.026, -0.022),
        (0.047, -0.062),
        (0.070, -0.120),
        (0.088, -0.150),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, "drafting_lamp_shade_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalance_drafting_floor_lamp")

    base_finish = model.material("base_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    weight_finish = model.material("weight_finish", rgba=(0.42, 0.35, 0.22, 1.0))
    shade_finish = model.material("shade_finish", rgba=(0.90, 0.90, 0.86, 1.0))
    accent_dark = model.material("accent_dark", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.17, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=base_finish,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=accent_dark,
        name="base_socket",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.08),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.013, length=1.22),
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        material=arm_metal,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=0.022, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 1.2175)),
        material=accent_dark,
        name="pivot_collar",
    )
    post.visual(
        Box((0.022, 0.008, 0.048)),
        origin=Origin(xyz=(-0.018, 0.025, 1.259)),
        material=accent_dark,
        name="rear_riser_left",
    )
    post.visual(
        Box((0.022, 0.008, 0.048)),
        origin=Origin(xyz=(-0.018, -0.025, 1.259)),
        material=accent_dark,
        name="rear_riser_right",
    )
    post.visual(
        Box((0.038, 0.006, 0.052)),
        origin=Origin(xyz=(0.004, 0.024, 1.255)),
        material=accent_dark,
        name="fork_left",
    )
    post.visual(
        Box((0.038, 0.006, 0.052)),
        origin=Origin(xyz=(0.004, -0.024, 1.255)),
        material=accent_dark,
        name="fork_right",
    )
    post.visual(
        Box((0.050, 0.064, 0.014)),
        origin=Origin(xyz=(-0.006, 0.0, 1.287)),
        material=accent_dark,
        name="fork_bridge",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.09, 0.09, 1.30)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.FIXED,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.019, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_dark,
        name="pivot_hub",
    )
    boom.visual(
        Cylinder(radius=0.010, length=1.020),
        origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_metal,
        name="main_arm",
    )
    boom.visual(
        Cylinder(radius=0.055, length=0.055),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=weight_finish,
        name="counterweight",
    )
    boom.visual(
        Cylinder(radius=0.015, length=0.038),
        origin=Origin(xyz=(0.779, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_dark,
        name="tip_spigot",
    )
    boom.inertial = Inertial.from_geometry(
        Box((1.05, 0.12, 0.12)),
        mass=2.2,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
    )

    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=post,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 1.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.80,
        ),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_dark,
        name="shade_collar",
    )
    shade.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=accent_dark,
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=accent_dark,
        name="shade_socket",
    )
    shade.visual(
        _build_shade_shell(),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=shade_finish,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(
            xyz=(0.010, 0.065, -0.160),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=accent_dark,
        name="shade_handle_stem",
    )
    shade.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(
            xyz=(0.020, 0.087, -0.160),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=accent_dark,
        name="shade_handle_grip",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.27)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(0.813, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-1.20,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    post = object_model.get_part("post")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")
    boom_pivot = object_model.get_articulation("post_to_boom")
    shade_roll = object_model.get_articulation("boom_to_shade")

    def center_of_aabb(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    ctx.expect_contact(
        post,
        base,
        elem_a="post_shaft",
        elem_b="base_socket",
        contact_tol=0.0015,
        name="post seats into the weighted base socket",
    )
    ctx.expect_contact(
        shade,
        boom,
        elem_a="shade_collar",
        elem_b="tip_spigot",
        contact_tol=0.0015,
        name="shade collar seats on the boom tip spigot",
    )

    with ctx.pose({boom_pivot: 0.0, shade_roll: 0.0}):
        hub_aabb = ctx.part_element_world_aabb(boom, elem="pivot_hub")
        left_fork_aabb = ctx.part_element_world_aabb(post, elem="fork_left")
        right_fork_aabb = ctx.part_element_world_aabb(post, elem="fork_right")
        tip_rest_aabb = ctx.part_element_world_aabb(boom, elem="tip_spigot")
        shade_rest_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

    fork_clearance_ok = (
        hub_aabb is not None
        and left_fork_aabb is not None
        and right_fork_aabb is not None
        and right_fork_aabb[1][1] <= hub_aabb[0][1] + 0.003
        and hub_aabb[1][1] <= left_fork_aabb[0][1] + 0.003
        and hub_aabb[0][2] >= min(left_fork_aabb[0][2], right_fork_aabb[0][2]) - 0.002
        and hub_aabb[1][2] <= max(left_fork_aabb[1][2], right_fork_aabb[1][2]) + 0.002
    )
    ctx.check(
        "boom pivot hub stays nested between the fork cheeks",
        fork_clearance_ok,
        details=(
            f"hub={hub_aabb}, left_fork={left_fork_aabb}, "
            f"right_fork={right_fork_aabb}"
        ),
    )

    with ctx.pose({boom_pivot: math.radians(35.0), shade_roll: 0.0}):
        tip_raised_aabb = ctx.part_element_world_aabb(boom, elem="tip_spigot")

    tip_rest_center = center_of_aabb(tip_rest_aabb) if tip_rest_aabb is not None else None
    tip_raised_center = (
        center_of_aabb(tip_raised_aabb) if tip_raised_aabb is not None else None
    )
    ctx.check(
        "positive boom rotation lifts the lamp head upward",
        tip_rest_center is not None
        and tip_raised_center is not None
        and tip_raised_center[2] > tip_rest_center[2] + 0.35,
        details=f"rest={tip_rest_center}, raised={tip_raised_center}",
    )

    with ctx.pose({boom_pivot: 0.0, shade_roll: 0.95}):
        shade_swiveled_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

    shade_rest_center = (
        center_of_aabb(shade_rest_aabb) if shade_rest_aabb is not None else None
    )
    shade_swiveled_center = (
        center_of_aabb(shade_swiveled_aabb) if shade_swiveled_aabb is not None else None
    )
    ctx.check(
        "shade rotates around the terminal arm axis",
        shade_rest_center is not None
        and shade_swiveled_center is not None
        and abs(shade_swiveled_center[1] - shade_rest_center[1]) > 0.10
        and shade_swiveled_center[2] > shade_rest_center[2] + 0.05,
        details=f"rest={shade_rest_center}, swiveled={shade_swiveled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
