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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _yz_section(x: float, width_y: float, height_z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for z, y in rounded_rect_profile(height_z, width_y, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="espresso_machine")

    body_silver = model.material("body_silver", rgba=(0.80, 0.81, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.11, 0.12, 0.13, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.75, 0.77, 1.0))
    body = model.part("body")

    body_shell = section_loft(
        [
            _yz_section(0.155, 0.250, 0.280, 0.050),
            _yz_section(0.055, 0.285, 0.330, 0.060),
            _yz_section(-0.070, 0.285, 0.340, 0.065),
            _yz_section(-0.175, 0.250, 0.300, 0.050),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=body_silver,
        name="shell",
    )
    body.visual(
        Box((0.272, 0.320, 0.018)),
        origin=Origin(xyz=(-0.006, 0.0, 0.009)),
        material=dark_trim,
        name="base_plinth",
    )
    body.visual(
        Box((0.080, 0.130, 0.010)),
        origin=Origin(xyz=(0.085, 0.0, 0.316)),
        material=dark_trim,
        name="top_plate",
    )
    body.visual(
        Box((0.172, 0.120, 0.024)),
        origin=Origin(xyz=(-0.010, 0.0, 0.302)),
        material=dark_trim,
        name="top_deck_core",
    )
    body.visual(
        Box((0.130, 0.004, 0.036)),
        origin=Origin(xyz=(-0.040, 0.047, 0.298)),
        material=black_plastic,
        name="reservoir_wall_left",
    )
    body.visual(
        Box((0.130, 0.004, 0.036)),
        origin=Origin(xyz=(-0.040, -0.047, 0.298)),
        material=black_plastic,
        name="reservoir_wall_right",
    )
    body.visual(
        Box((0.004, 0.090, 0.036)),
        origin=Origin(xyz=(0.023, 0.0, 0.298)),
        material=black_plastic,
        name="reservoir_wall_front",
    )
    body.visual(
        Box((0.004, 0.090, 0.036)),
        origin=Origin(xyz=(-0.103, 0.0, 0.298)),
        material=black_plastic,
        name="reservoir_wall_rear",
    )
    body.visual(
        Box((0.116, 0.090, 0.012)),
        origin=Origin(xyz=(0.103, 0.0, 0.286)),
        material=dark_trim,
        name="cup_warmer",
    )
    body.visual(
        Box((0.090, 0.010, 0.006)),
        origin=Origin(xyz=(0.190, 0.075, 0.041)),
        material=black_plastic,
        name="tray_guide_left",
    )
    body.visual(
        Box((0.090, 0.010, 0.006)),
        origin=Origin(xyz=(0.190, -0.075, 0.041)),
        material=black_plastic,
        name="tray_guide_right",
    )
    body.visual(
        Box((0.024, 0.024, 0.008)),
        origin=Origin(xyz=(0.118, 0.108, 0.004)),
        material=black_plastic,
        name="foot_left_front",
    )
    body.visual(
        Box((0.024, 0.024, 0.008)),
        origin=Origin(xyz=(0.118, -0.108, 0.004)),
        material=black_plastic,
        name="foot_right_front",
    )
    body.visual(
        Box((0.024, 0.024, 0.008)),
        origin=Origin(xyz=(-0.118, 0.108, 0.004)),
        material=black_plastic,
        name="foot_left_rear",
    )
    body.visual(
        Box((0.024, 0.024, 0.008)),
        origin=Origin(xyz=(-0.118, -0.108, 0.004)),
        material=black_plastic,
        name="foot_right_rear",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.064),
        origin=Origin(xyz=(0.164, 0.0, 0.215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="group_head_body",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.196, 0.0, 0.215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="group_head_ring",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.154, -0.146, 0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="steam_pivot_housing",
    )
    body.visual(
        Box((0.016, 0.018, 0.034)),
        origin=Origin(xyz=(0.146, -0.138, 0.220)),
        material=dark_trim,
        name="steam_mount",
    )
    body.visual(
        Box((0.034, 0.004, 0.026)),
        origin=Origin(xyz=(-0.028, 0.142, 0.138)),
        material=dark_trim,
        name="power_bezel",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.310, 0.290, 0.340)),
        mass=12.0,
        origin=Origin(xyz=(-0.005, 0.0, 0.170)),
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.134, 0.102, 0.008)),
        origin=Origin(xyz=(0.067, 0.0, 0.004)),
        material=black_plastic,
        name="cover",
    )
    flap.visual(
        Box((0.020, 0.050, 0.008)),
        origin=Origin(xyz=(0.126, 0.0, 0.000)),
        material=dark_trim,
        name="tab",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.134, 0.102, 0.010)),
        mass=0.18,
        origin=Origin(xyz=(0.067, 0.0, 0.004)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.036, 0.018, 0.016)),
        origin=Origin(xyz=(0.022, 0.0, -0.013)),
        material=steel,
        name="neck",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.128),
        origin=Origin(xyz=(0.084, 0.0, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="handle",
    )
    portafilter.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(0.128, 0.0, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="grip",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.178, 0.060, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.074, 0.0, -0.014)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=steel,
        name="pivot_stem",
    )
    steam_wand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, -0.018),
                    (0.010, 0.0, -0.050),
                    (0.022, 0.002, -0.100),
                    (0.032, 0.018, -0.155),
                ],
                radius=0.0055,
                samples_per_segment=14,
                radial_segments=18,
            ),
            "steam_wand_tube",
        ),
        material=steel,
        name="tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.032, 0.018, -0.162)),
        material=steel,
        name="tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.045, 0.185)),
        mass=0.20,
        origin=Origin(xyz=(0.018, 0.010, -0.090)),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.028, 0.010, 0.022)),
        origin=Origin(),
        material=black_plastic,
        name="body",
    )
    power_rocker.visual(
        Box((0.024, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_trim,
        name="rocker_top",
    )
    power_rocker.inertial = Inertial.from_geometry(
        Box((0.028, 0.010, 0.022)),
        mass=0.03,
        origin=Origin(),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.122, 0.146, 0.005)),
        origin=Origin(xyz=(0.061, 0.0, 0.0025)),
        material=steel,
        name="bottom",
    )
    drip_tray.visual(
        Box((0.122, 0.004, 0.022)),
        origin=Origin(xyz=(0.061, 0.071, 0.011)),
        material=steel,
        name="wall_left",
    )
    drip_tray.visual(
        Box((0.122, 0.004, 0.022)),
        origin=Origin(xyz=(0.061, -0.071, 0.011)),
        material=steel,
        name="wall_right",
    )
    drip_tray.visual(
        Box((0.004, 0.142, 0.022)),
        origin=Origin(xyz=(0.002, 0.0, 0.011)),
        material=steel,
        name="wall_rear",
    )
    drip_tray.visual(
        Box((0.012, 0.154, 0.030)),
        origin=Origin(xyz=(0.116, 0.0, 0.015)),
        material=dark_trim,
        name="front_lip",
    )
    drip_tray.visual(
        Box((0.110, 0.010, 0.010)),
        origin=Origin(xyz=(0.055, 0.066, 0.033)),
        material=black_plastic,
        name="rail_left",
    )
    drip_tray.visual(
        Box((0.110, 0.010, 0.010)),
        origin=Origin(xyz=(0.055, -0.066, 0.033)),
        material=black_plastic,
        name="rail_right",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.128, 0.154, 0.040)),
        mass=0.42,
        origin=Origin(xyz=(0.064, 0.0, 0.018)),
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(-0.103, 0.0, 0.314)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.192, 0.0, 0.177)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.0,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.154, -0.146, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.70,
            upper=1.15,
        ),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(-0.028, 0.149, 0.138)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=-0.32,
            upper=0.32,
        ),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.156, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    power_rocker = object_model.get_part("power_rocker")
    drip_tray = object_model.get_part("drip_tray")

    flap_joint = object_model.get_articulation("body_to_flap")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        min_gap=0.0005,
        max_gap=0.012,
        positive_elem="group_head_body",
        negative_elem="basket",
        name="portafilter sits just below the group head",
    )
    ctx.expect_overlap(
        body,
        portafilter,
        axes="xy",
        min_overlap=0.030,
        elem_a="group_head_body",
        elem_b="basket",
        name="portafilter basket stays aligned with the brew head",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="cover",
        negative_elem="top_deck_core",
        name="fill flap closes flush over the top opening",
    )
    ctx.expect_overlap(
        body,
        drip_tray,
        axes="x",
        min_overlap=0.040,
        elem_a="tray_guide_left",
        elem_b="rail_left",
        name="drip tray remains engaged in the guide at rest",
    )

    flap_closed = ctx.part_element_world_aabb(flap, elem="tab")
    with ctx.pose({flap_joint: 1.10}):
        flap_open = ctx.part_element_world_aabb(flap, elem="tab")
    flap_closed_center = aabb_center(flap_closed)
    flap_open_center = aabb_center(flap_open)
    ctx.check(
        "fill flap opens upward",
        flap_closed_center is not None
        and flap_open_center is not None
        and flap_open_center[2] > flap_closed_center[2] + 0.060,
        details=f"closed={flap_closed_center}, open={flap_open_center}",
    )

    handle_locked = ctx.part_element_world_aabb(portafilter, elem="handle")
    with ctx.pose({portafilter_joint: -0.55}):
        handle_unlocked = ctx.part_element_world_aabb(portafilter, elem="handle")
    handle_locked_center = aabb_center(handle_locked)
    handle_unlocked_center = aabb_center(handle_unlocked)
    ctx.check(
        "portafilter rotates around the vertical brew axis",
        handle_locked_center is not None
        and handle_unlocked_center is not None
        and abs(handle_unlocked_center[1] - handle_locked_center[1]) > 0.040,
        details=f"locked={handle_locked_center}, unlocked={handle_unlocked_center}",
    )

    wand_rest = ctx.part_element_world_aabb(steam_wand, elem="tip")
    with ctx.pose({wand_joint: 0.85}):
        wand_swung = ctx.part_element_world_aabb(steam_wand, elem="tip")
    wand_rest_center = aabb_center(wand_rest)
    wand_swung_center = aabb_center(wand_swung)
    ctx.check(
        "steam wand swings around its side pivot",
        wand_rest_center is not None
        and wand_swung_center is not None
        and wand_swung_center[1] > wand_rest_center[1] + 0.015,
        details=f"rest={wand_rest_center}, swung={wand_swung_center}",
    )

    rocker_rest = ctx.part_element_world_aabb(power_rocker, elem="rocker_top")
    with ctx.pose({rocker_joint: 0.22}):
        rocker_on = ctx.part_element_world_aabb(power_rocker, elem="rocker_top")
    rocker_rest_center = aabb_center(rocker_rest)
    rocker_on_center = aabb_center(rocker_on)
    ctx.check(
        "power rocker tips outward on its side mount",
        rocker_rest_center is not None
        and rocker_on_center is not None
        and rocker_on_center[1] > rocker_rest_center[1] + 0.001,
        details=f"rest={rocker_rest_center}, on={rocker_on_center}",
    )

    tray_rest = ctx.part_world_position(drip_tray)
    with ctx.pose({tray_joint: 0.055}):
        tray_extended = ctx.part_world_position(drip_tray)
        ctx.expect_overlap(
            body,
            drip_tray,
            axes="x",
            min_overlap=0.020,
            elem_a="tray_guide_left",
            elem_b="rail_left",
            name="drip tray retains insertion when extended",
        )
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_extended is not None and tray_extended[0] > tray_rest[0] + 0.045,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
