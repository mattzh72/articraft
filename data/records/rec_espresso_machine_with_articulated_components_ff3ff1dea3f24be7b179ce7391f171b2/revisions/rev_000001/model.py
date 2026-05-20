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
    tube_from_spline_points,
)


def _xy_section(
    depth_x: float,
    width_y: float,
    radius: float,
    z: float,
    *,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + x_center, y + y_center, z) for x, y in rounded_rect_profile(depth_x, width_y, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lever_espresso_machine")

    body_enamel = model.material("body_enamel", rgba=(0.92, 0.90, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.14, 0.12, 0.10, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.40, 0.28, 0.03)),
        origin=Origin(xyz=(0.02, 0.0, 0.015)),
        material=body_enamel,
        name="base_plinth",
    )

    body_shell = section_loft(
        [
            _xy_section(0.18, 0.20, 0.040, 0.03, x_center=-0.040),
            _xy_section(0.17, 0.18, 0.038, 0.20, x_center=-0.036),
            _xy_section(0.15, 0.16, 0.034, 0.38, x_center=-0.026),
            _xy_section(0.12, 0.14, 0.028, 0.54, x_center=-0.014),
        ]
    )
    body.visual(mesh_from_geometry(body_shell, "body_shell"), material=body_enamel, name="shell")
    body.visual(
        Box((0.10, 0.10, 0.16)),
        origin=Origin(xyz=(0.03, 0.0, 0.30)),
        material=body_enamel,
        name="front_shoulder",
    )
    body.visual(
        Box((0.05, 0.10, 0.06)),
        origin=Origin(xyz=(0.02, 0.10, 0.27)),
        material=steel,
        name="wand_boss",
    )
    body.visual(
        Box((0.24, 0.02, 0.013)),
        origin=Origin(xyz=(0.09, 0.10, 0.0365)),
        material=steel,
        name="tray_guide_outer",
    )
    body.visual(
        Box((0.24, 0.02, 0.013)),
        origin=Origin(xyz=(0.09, -0.10, 0.0365)),
        material=steel,
        name="tray_guide_inner",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.56)),
        mass=12.0,
        origin=Origin(xyz=(0.02, 0.0, 0.28)),
    )

    group_head = model.part("group_head")
    group_head.visual(
        Box((0.04, 0.10, 0.09)),
        origin=Origin(xyz=(0.02, 0.0, 0.02)),
        material=steel,
        name="rear_block",
    )
    group_head.visual(
        Cylinder(radius=0.032, length=0.065),
        origin=Origin(xyz=(0.065, 0.0, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="brew_body",
    )
    group_head.visual(
        Cylinder(radius=0.033, length=0.024),
        origin=Origin(xyz=(0.095, 0.0, -0.012)),
        material=steel,
        name="brew_collar",
    )
    group_head.visual(
        Box((0.032, 0.014, 0.12)),
        origin=Origin(xyz=(0.016, 0.052, 0.078)),
        material=steel,
        name="lever_support_0",
    )
    group_head.visual(
        Box((0.032, 0.014, 0.12)),
        origin=Origin(xyz=(0.016, -0.052, 0.078)),
        material=steel,
        name="lever_support_1",
    )
    group_head.visual(
        Box((0.022, 0.118, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.139)),
        material=steel,
        name="lever_bridge",
    )
    group_head.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.17)),
        mass=2.0,
        origin=Origin(xyz=(0.06, 0.0, 0.045)),
    )

    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=group_head,
        origin=Origin(xyz=(0.08, 0.0, 0.28)),
    )

    brew_lever = model.part("brew_lever")
    lever_arm_0 = tube_from_spline_points(
        [(0.0, 0.030, 0.0), (0.035, 0.030, 0.095), (0.110, 0.030, 0.205)],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=18,
    )
    lever_arm_1 = tube_from_spline_points(
        [(0.0, -0.030, 0.0), (0.035, -0.030, 0.095), (0.110, -0.030, 0.205)],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=18,
    )
    brew_lever.visual(mesh_from_geometry(lever_arm_0, "brew_lever_arm_0"), material=steel, name="arm_0")
    brew_lever.visual(mesh_from_geometry(lever_arm_1, "brew_lever_arm_1"), material=steel, name="arm_1")
    brew_lever.visual(
        Box((0.012, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="pivot_shaft",
    )
    brew_lever.visual(
        Box((0.032, 0.14, 0.032)),
        origin=Origin(xyz=(0.115, 0.0, 0.205)),
        material=dark_handle,
        name="handgrip",
    )
    brew_lever.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.25)),
        mass=0.9,
        origin=Origin(xyz=(0.08, 0.0, 0.11)),
    )

    model.articulation(
        "group_head_to_brew_lever",
        ArticulationType.REVOLUTE,
        parent=group_head,
        child=brew_lever,
        origin=Origin(xyz=(0.038, 0.0, 0.128)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.4,
            lower=-1.15,
            upper=0.20,
        ),
    )

    portafilter = model.part("portafilter")
    basket_shell = LatheGeometry.from_shell_profiles(
        [(0.030, 0.0), (0.031, -0.006), (0.029, -0.022), (0.026, -0.040), (0.022, -0.060)],
        [(0.0, -0.001), (0.026, -0.006), (0.023, -0.023), (0.020, -0.041), (0.017, -0.056)],
        segments=48,
    )
    portafilter.visual(mesh_from_geometry(basket_shell, "portafilter_basket"), material=steel, name="basket")
    portafilter.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=steel,
        name="collar",
    )
    portafilter.visual(
        Box((0.035, 0.018, 0.016)),
        origin=Origin(xyz=(0.018, 0.0, -0.018)),
        material=steel,
        name="handle_stem",
    )
    portafilter.visual(
        Box((0.14, 0.026, 0.022)),
        origin=Origin(xyz=(0.095, 0.0, -0.020)),
        material=dark_handle,
        name="handle",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.18, 0.06, 0.08)),
        mass=0.6,
        origin=Origin(xyz=(0.08, 0.0, -0.025)),
    )

    model.articulation(
        "group_head_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=group_head,
        child=portafilter,
        origin=Origin(xyz=(0.095, 0.0, -0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-0.70,
            upper=0.70,
        ),
    )

    steam_wand = model.part("steam_wand")
    wand_pipe = tube_from_spline_points(
        [(0.0, 0.0, 0.0), (0.0, 0.018, 0.0), (0.005, 0.034, -0.055), (0.008, 0.034, -0.180)],
        radius=0.0045,
        samples_per_segment=18,
        radial_segments=16,
    )
    steam_wand.visual(mesh_from_geometry(wand_pipe, "steam_wand_pipe"), material=steel, name="pipe")
    steam_wand.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=steel,
        name="pivot_collar",
    )
    steam_wand.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.008, 0.034, -0.189)),
        material=steel,
        name="tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.04, 0.08, 0.22)),
        mass=0.25,
        origin=Origin(xyz=(0.004, 0.026, -0.09)),
    )

    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.02, 0.16, 0.29)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.2,
        ),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.19, 0.22, 0.010)),
        origin=Origin(xyz=(0.095, 0.0, 0.005)),
        material=steel,
        name="pan",
    )
    drip_tray.visual(
        Box((0.19, 0.008, 0.028)),
        origin=Origin(xyz=(0.095, 0.106, 0.019)),
        material=steel,
        name="wall_0",
    )
    drip_tray.visual(
        Box((0.19, 0.008, 0.028)),
        origin=Origin(xyz=(0.095, -0.106, 0.019)),
        material=steel,
        name="wall_1",
    )
    drip_tray.visual(
        Box((0.010, 0.22, 0.022)),
        origin=Origin(xyz=(0.005, 0.0, 0.011)),
        material=steel,
        name="rear_wall",
    )
    drip_tray.visual(
        Box((0.010, 0.22, 0.036)),
        origin=Origin(xyz=(0.185, 0.0, 0.018)),
        material=steel,
        name="front_wall",
    )
    drip_tray.visual(
        Box((0.030, 0.24, 0.012)),
        origin=Origin(xyz=(0.185, 0.0, 0.042)),
        material=body_enamel,
        name="front_fascia",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.05)),
        mass=0.8,
        origin=Origin(xyz=(0.10, 0.0, 0.025)),
    )

    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.055, 0.0, 0.043)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
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

    group_head = object_model.get_part("group_head")
    brew_lever = object_model.get_part("brew_lever")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    drip_tray = object_model.get_part("drip_tray")

    lever_joint = object_model.get_articulation("group_head_to_brew_lever")
    portafilter_joint = object_model.get_articulation("group_head_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.expect_gap(
        group_head,
        portafilter,
        axis="z",
        positive_elem="brew_collar",
        negative_elem="collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="portafilter seats under brew collar",
    )

    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        lever_raised = aabb_center(ctx.part_element_world_aabb(brew_lever, elem="handgrip"))
    with ctx.pose({lever_joint: lever_joint.motion_limits.lower}):
        lever_lowered = aabb_center(ctx.part_element_world_aabb(brew_lever, elem="handgrip"))
    ctx.check(
        "brew lever handgrip travels downward",
        lever_raised is not None
        and lever_lowered is not None
        and lever_lowered[2] < lever_raised[2] - 0.10,
        details=f"raised={lever_raised}, lowered={lever_lowered}",
    )

    with ctx.pose({portafilter_joint: portafilter_joint.motion_limits.lower}):
        handle_left = aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({portafilter_joint: portafilter_joint.motion_limits.upper}):
        handle_right = aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    ctx.check(
        "portafilter handle sweeps around vertical brew axis",
        handle_left is not None
        and handle_right is not None
        and abs(handle_right[1] - handle_left[1]) > 0.11,
        details=f"lower={handle_left}, upper={handle_right}",
    )

    with ctx.pose({wand_joint: wand_joint.motion_limits.lower}):
        tip_inboard = aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tip"))
    with ctx.pose({wand_joint: wand_joint.motion_limits.upper}):
        tip_outboard = aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tip"))
    ctx.check(
        "steam wand tip swings around side pivot",
        tip_inboard is not None
        and tip_outboard is not None
        and math.hypot(tip_outboard[0] - tip_inboard[0], tip_outboard[1] - tip_inboard[1]) > 0.06,
        details=f"inboard={tip_inboard}, outboard={tip_outboard}",
    )

    tray_rest = ctx.part_world_position(drip_tray)
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        tray_extended = ctx.part_world_position(drip_tray)
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_extended is not None and tray_extended[0] > tray_rest[0] + 0.06,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
