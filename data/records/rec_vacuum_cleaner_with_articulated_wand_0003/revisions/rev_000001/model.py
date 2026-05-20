from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_articulated_vacuum", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.28, 0.30, 0.32, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    deep_black = model.material("deep_black", rgba=(0.07, 0.07, 0.08, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.55, 0.60, 0.65, 0.35))
    warm_metal = model.material("warm_metal", rgba=(0.63, 0.62, 0.58, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def xy_section(z: float, width: float, depth: float, radius: float, y_shift: float = 0.0):
        return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius)]

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Box((0.310, 0.125, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=matte_graphite,
        name="top_shell",
    )
    floor_nozzle.visual(
        Box((0.018, 0.118, 0.034)),
        origin=Origin(xyz=(-0.146, 0.0, 0.017)),
        material=deep_black,
        name="left_rail",
    )
    floor_nozzle.visual(
        Box((0.018, 0.118, 0.034)),
        origin=Origin(xyz=(0.146, 0.0, 0.017)),
        material=deep_black,
        name="right_rail",
    )
    floor_nozzle.visual(
        Box((0.252, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, -0.049, 0.020)),
        material=satin_graphite,
        name="rear_bridge",
    )
    floor_nozzle.visual(
        Box((0.276, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.056, 0.012)),
        material=deep_black,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.110, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.025, 0.025)),
        material=satin_graphite,
        name="throat_panel",
    )
    nozzle_cover = section_loft(
        [
            xy_section(0.024, 0.286, 0.080, 0.022, y_shift=0.025),
            xy_section(0.040, 0.306, 0.106, 0.030, y_shift=0.030),
            xy_section(0.054, 0.180, 0.050, 0.014, y_shift=0.016),
        ]
    )
    floor_nozzle.visual(
        save_mesh("vacuum_nozzle_cover.obj", nozzle_cover),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=matte_graphite,
        name="nozzle_cover",
    )
    floor_nozzle.visual(
        Box((0.044, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, -0.050, 0.043)),
        material=satin_graphite,
        name="turntable_plinth",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, -0.050, 0.052)),
        material=warm_metal,
        name="turntable_disc",
    )
    floor_nozzle.visual(
        Box((0.008, 0.018, 0.018)),
        origin=Origin(xyz=(-0.134, 0.008, 0.018)),
        material=satin_graphite,
        name="left_bearing",
    )
    floor_nozzle.visual(
        Box((0.008, 0.018, 0.018)),
        origin=Origin(xyz=(0.134, 0.008, 0.018)),
        material=satin_graphite,
        name="right_bearing",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.310, 0.125, 0.056)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    nozzle_swivel = model.part("nozzle_swivel")
    nozzle_swivel.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=warm_metal,
        name="swivel_collar",
    )
    nozzle_swivel.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=satin_graphite,
        name="swivel_stem",
    )
    nozzle_swivel.visual(
        Box((0.018, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, 0.038)),
        material=matte_graphite,
        name="linkage_anchor",
    )
    nozzle_swivel.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.009, 0.050)),
        material=matte_graphite,
        name="anchor_riser",
    )
    nozzle_swivel.visual(
        Box((0.028, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, 0.060)),
        material=satin_graphite,
        name="yoke_bridge",
    )
    nozzle_swivel.visual(
        Box((0.005, 0.018, 0.028)),
        origin=Origin(xyz=(-0.010, -0.006, 0.070)),
        material=matte_graphite,
        name="left_ear",
    )
    nozzle_swivel.visual(
        Box((0.005, 0.018, 0.028)),
        origin=Origin(xyz=(0.010, -0.006, 0.070)),
        material=matte_graphite,
        name="right_ear",
    )
    nozzle_swivel.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(-0.0135, -0.006, 0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="left_pin",
    )
    nozzle_swivel.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.0135, -0.006, 0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="right_pin",
    )
    nozzle_swivel.inertial = Inertial.from_geometry(
        Box((0.034, 0.030, 0.086)),
        mass=0.16,
        origin=Origin(xyz=(0.0, -0.004, 0.043)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.0065, length=0.015),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="pivot_sleeve",
    )
    lower_wand.visual(
        Box((0.010, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.008, 0.010)),
        material=matte_graphite,
        name="pivot_web",
    )
    lower_wand.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.008, 0.028)),
        material=matte_graphite,
        name="pivot_anchor",
    )
    lower_wand.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, -0.002, 0.046)),
        material=satin_graphite,
        name="lower_collar",
    )
    lower_wand.visual(
        Cylinder(radius=0.016, length=0.360),
        origin=Origin(xyz=(0.0, -0.002, 0.227)),
        material=satin_aluminum,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.008, 0.016, 0.150)),
        origin=Origin(xyz=(0.0, -0.012, 0.205)),
        material=satin_graphite,
        name="seam_strip",
    )
    lower_wand.visual(
        Box((0.028, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.405)),
        material=satin_graphite,
        name="upper_bridge",
    )
    lower_wand.visual(
        Box((0.005, 0.018, 0.028)),
        origin=Origin(xyz=(-0.010, -0.006, 0.418)),
        material=matte_graphite,
        name="upper_left_ear",
    )
    lower_wand.visual(
        Box((0.005, 0.018, 0.028)),
        origin=Origin(xyz=(0.010, -0.006, 0.418)),
        material=matte_graphite,
        name="upper_right_ear",
    )
    lower_wand.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(-0.0135, -0.006, 0.418), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="upper_left_pin",
    )
    lower_wand.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.0135, -0.006, 0.418), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="upper_right_pin",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.032, 0.030, 0.440)),
        mass=0.34,
        origin=Origin(xyz=(0.0, -0.002, 0.220)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.0065, length=0.015),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="pivot_sleeve",
    )
    upper_wand.visual(
        Box((0.010, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.008, 0.010)),
        material=matte_graphite,
        name="pivot_web",
    )
    upper_wand.visual(
        Box((0.014, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.008, 0.026)),
        material=matte_graphite,
        name="fold_anchor",
    )
    upper_wand.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, -0.002, 0.032)),
        material=satin_graphite,
        name="hinge_ring",
    )
    upper_wand.visual(
        Cylinder(radius=0.015, length=0.300),
        origin=Origin(xyz=(0.0, -0.002, 0.180)),
        material=satin_aluminum,
        name="upper_tube",
    )
    upper_wand.visual(
        Box((0.008, 0.014, 0.150)),
        origin=Origin(xyz=(0.0, -0.011, 0.170)),
        material=satin_graphite,
        name="rear_spine",
    )
    upper_wand.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, -0.002, 0.330)),
        material=satin_graphite,
        name="interface_collar",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.032, 0.026, 0.348)),
        mass=0.28,
        origin=Origin(xyz=(0.0, -0.002, 0.174)),
    )

    main_body = model.part("main_body")
    body_shell = section_loft(
        [
            xy_section(0.000, 0.060, 0.048, 0.014),
            xy_section(0.090, 0.092, 0.076, 0.022),
            xy_section(0.210, 0.080, 0.060, 0.018),
            xy_section(0.300, 0.054, 0.040, 0.012),
        ]
    )
    main_body.visual(
        save_mesh("vacuum_body_shell.obj", body_shell),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=matte_graphite,
        name="body_shell",
    )
    main_body.visual(
        Box((0.032, 0.028, 0.042)),
        origin=Origin(xyz=(0.0, -0.003, 0.021)),
        material=satin_graphite,
        name="wand_socket",
    )
    main_body.visual(
        Cylinder(radius=0.050, length=0.160),
        origin=Origin(xyz=(0.0, 0.055, 0.130), rpy=(pi / 2.0, 0.0, 0.0)),
        material=smoked_clear,
        name="dust_bin",
    )
    main_body.visual(
        Cylinder(radius=0.054, length=0.010),
        origin=Origin(xyz=(0.0, 0.130, 0.130), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="bin_front_trim",
    )
    main_body.visual(
        Cylinder(radius=0.038, length=0.055),
        origin=Origin(xyz=(0.0, -0.032, 0.138), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="motor_cap",
    )
    main_body.visual(
        Cylinder(radius=0.016, length=0.108),
        origin=Origin(xyz=(0.0, 0.075, 0.130), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="cyclone_core",
    )
    main_body.visual(
        Box((0.072, 0.050, 0.110)),
        origin=Origin(xyz=(0.0, -0.028, 0.056)),
        material=deep_black,
        name="battery_pack",
    )
    main_body.visual(
        Box((0.050, 0.034, 0.140)),
        origin=Origin(xyz=(0.0, -0.030, 0.205)),
        material=deep_black,
        name="grip_body",
    )
    main_body.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, -0.045, 0.270), rpy=(0.0, pi / 2.0, 0.0)),
        material=deep_black,
        name="top_grip",
    )
    main_body.visual(
        Box((0.050, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.020, 0.210)),
        material=satin_graphite,
        name="display_strip",
    )
    main_body.visual(
        Box((0.020, 0.006, 0.028)),
        origin=Origin(xyz=(0.034, 0.010, 0.198)),
        material=warm_metal,
        name="power_button",
    )
    main_body.visual(
        Box((0.028, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.040, 0.238)),
        material=satin_graphite,
        name="trigger_bridge",
    )
    main_body.visual(
        Box((0.006, 0.014, 0.016)),
        origin=Origin(xyz=(-0.014, -0.048, 0.238)),
        material=satin_graphite,
        name="trigger_left_boss",
    )
    main_body.visual(
        Box((0.006, 0.014, 0.016)),
        origin=Origin(xyz=(0.014, -0.048, 0.238)),
        material=satin_graphite,
        name="trigger_right_boss",
    )
    main_body.inertial = Inertial.from_geometry(
        Box((0.100, 0.190, 0.300)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.030, 0.150)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="pivot_bar",
    )
    trigger.visual(
        Box((0.014, 0.014, 0.036)),
        origin=Origin(xyz=(0.0, -0.008, -0.018)),
        material=matte_graphite,
        name="trigger_body",
    )
    trigger.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, -0.012, -0.037), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_graphite,
        name="trigger_tip",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.046)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.008, -0.020)),
    )

    brush_roll = model.part("brush_roll")
    brush_roll.visual(
        Cylinder(radius=0.015, length=0.228),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=deep_black,
        name="roller_core",
    )
    brush_roll.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(-0.122, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="left_endcap",
    )
    brush_roll.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.122, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="right_endcap",
    )
    brush_roll.visual(
        Box((0.228, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=satin_aluminum,
        name="front_bristle_strip",
    )
    brush_roll.visual(
        Box((0.228, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=satin_aluminum,
        name="rear_bristle_strip",
    )
    brush_roll.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.260),
        mass=0.08,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "floor_to_nozzle_swivel",
        ArticulationType.REVOLUTE,
        parent=floor_nozzle,
        child=nozzle_swivel,
        origin=Origin(xyz=(0.0, -0.050, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-radians(65.0),
            upper=radians(65.0),
        ),
    )
    model.articulation(
        "swivel_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=nozzle_swivel,
        child=lower_wand,
        origin=Origin(xyz=(0.0, -0.006, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.0,
            lower=-radians(20.0),
            upper=radians(40.0),
        ),
    )
    model.articulation(
        "lower_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=(0.0, -0.006, 0.418)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=radians(35.0),
        ),
    )
    model.articulation(
        "upper_wand_to_main_body",
        ArticulationType.FIXED,
        parent=upper_wand,
        child=main_body,
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
    )
    model.articulation(
        "main_body_to_trigger",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=trigger,
        origin=Origin(xyz=(0.0, -0.048, 0.238)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=-radians(18.0),
            upper=radians(12.0),
        ),
    )
    model.articulation(
        "floor_to_brush_roll",
        ArticulationType.CONTINUOUS,
        parent=floor_nozzle,
        child=brush_roll,
        origin=Origin(xyz=(0.0, 0.008, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    floor_nozzle = object_model.get_part("floor_nozzle")
    nozzle_swivel = object_model.get_part("nozzle_swivel")
    lower_wand = object_model.get_part("lower_wand")
    upper_wand = object_model.get_part("upper_wand")
    main_body = object_model.get_part("main_body")
    trigger = object_model.get_part("trigger")
    brush_roll = object_model.get_part("brush_roll")

    nozzle_yaw = object_model.get_articulation("floor_to_nozzle_swivel")
    nozzle_pitch = object_model.get_articulation("swivel_to_lower_wand")
    wand_fold = object_model.get_articulation("lower_to_upper_wand")
    trigger_hinge = object_model.get_articulation("main_body_to_trigger")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        nozzle_swivel,
        lower_wand,
        reason="Captured lower trunnion pivot intentionally shares the axle envelope inside the swivel yoke.",
    )
    ctx.allow_overlap(
        lower_wand,
        upper_wand,
        reason="Fold-joint clevis captures the upper wand pivot sleeve with intentional pin-envelope interpenetration.",
    )
    ctx.allow_overlap(
        upper_wand,
        main_body,
        reason="Upper wand interface collar is intentionally nested into the main body socket.",
    )
    ctx.allow_overlap(
        main_body,
        trigger,
        reason="Trigger pivot bar is intentionally captured within the handle bosses and trigger pocket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts(max_pose_samples=16, name="pose_sample_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(
        floor_nozzle,
        nozzle_swivel,
        elem_a="turntable_disc",
        elem_b="swivel_collar",
        name="swivel_turntable_contact",
    )
    ctx.expect_gap(
        lower_wand,
        nozzle_swivel,
        axis="x",
        positive_elem="pivot_sleeve",
        negative_elem="left_pin",
        min_gap=0.0,
        max_gap=0.005,
        name="lower_wand_left_pivot_gap",
    )
    ctx.expect_gap(
        nozzle_swivel,
        lower_wand,
        axis="x",
        positive_elem="right_pin",
        negative_elem="pivot_sleeve",
        min_gap=0.0,
        max_gap=0.005,
        name="lower_wand_right_pivot_gap",
    )
    ctx.expect_gap(
        upper_wand,
        lower_wand,
        axis="x",
        positive_elem="pivot_sleeve",
        negative_elem="upper_left_pin",
        min_gap=0.0,
        max_gap=0.005,
        name="upper_wand_left_pivot_gap",
    )
    ctx.expect_gap(
        lower_wand,
        upper_wand,
        axis="x",
        positive_elem="upper_right_pin",
        negative_elem="pivot_sleeve",
        min_gap=0.0,
        max_gap=0.005,
        name="upper_wand_right_pivot_gap",
    )
    ctx.expect_contact(
        upper_wand,
        main_body,
        elem_a="interface_collar",
        elem_b="wand_socket",
        name="body_socket_contact",
    )
    ctx.expect_contact(
        main_body,
        trigger,
        elem_a="trigger_left_boss",
        elem_b="pivot_bar",
        name="trigger_left_pivot_contact",
    )
    ctx.expect_contact(
        main_body,
        trigger,
        elem_a="trigger_right_boss",
        elem_b="pivot_bar",
        name="trigger_right_pivot_contact",
    )
    ctx.expect_contact(
        floor_nozzle,
        brush_roll,
        elem_a="left_bearing",
        elem_b="left_endcap",
        name="brush_roll_left_support_contact",
    )
    ctx.expect_contact(
        floor_nozzle,
        brush_roll,
        elem_a="right_bearing",
        elem_b="right_endcap",
        name="brush_roll_right_support_contact",
    )
    ctx.expect_gap(
        floor_nozzle,
        brush_roll,
        axis="z",
        positive_elem="top_shell",
        negative_elem="roller_core",
        min_gap=0.0005,
        max_gap=0.0030,
        name="brush_roll_top_clearance",
    )
    ctx.expect_gap(
        main_body,
        floor_nozzle,
        axis="z",
        min_gap=0.80,
        max_gap=0.95,
        name="upright_body_clearance_above_nozzle",
    )
    ctx.expect_origin_distance(
        main_body,
        floor_nozzle,
        axes="xy",
        max_dist=0.10,
        name="body_centered_over_nozzle",
    )

    floor_aabb = ctx.part_world_aabb(floor_nozzle)
    body_aabb = ctx.part_world_aabb(main_body)
    assert floor_aabb is not None
    assert body_aabb is not None
    nozzle_width = floor_aabb[1][0] - floor_aabb[0][0]
    total_height = body_aabb[1][2] - floor_aabb[0][2]
    ctx.check(
        "realistic_nozzle_width",
        0.28 <= nozzle_width <= 0.34,
        f"expected nozzle width in [0.28, 0.34], got {nozzle_width:.3f}",
    )
    ctx.check(
        "realistic_total_height",
        1.10 <= total_height <= 1.28,
        f"expected total height in [1.10, 1.28], got {total_height:.3f}",
    )

    for articulation in (nozzle_yaw, nozzle_pitch, wand_fold, trigger_hinge):
        limits = articulation.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    nozzle_pitch_limits = nozzle_pitch.motion_limits
    wand_fold_limits = wand_fold.motion_limits
    assert nozzle_pitch_limits is not None
    assert wand_fold_limits is not None
    assert nozzle_pitch_limits.upper is not None
    assert wand_fold_limits.upper is not None

    with ctx.pose({nozzle_pitch: nozzle_pitch_limits.upper, wand_fold: wand_fold_limits.upper}):
        ctx.expect_gap(
            main_body,
            floor_nozzle,
            axis="z",
            min_gap=0.30,
            name="folded_body_stays_off_floor",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_fold_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_fold_pose_no_floating")

    trigger_rest_aabb = ctx.part_world_aabb(trigger)
    assert trigger_rest_aabb is not None
    trigger_limits = trigger_hinge.motion_limits
    assert trigger_limits is not None
    assert trigger_limits.upper is not None
    with ctx.pose({trigger_hinge: trigger_limits.upper}):
        trigger_aabb = ctx.part_world_aabb(trigger)
        assert trigger_aabb is not None
        ctx.check(
            "trigger_pulls_downward",
            trigger_aabb[0][2] < trigger_rest_aabb[0][2] - 0.002,
            (
                "expected trigger to swing downward "
                f"(rest min z {trigger_rest_aabb[0][2]:.3f}, pulled min z {trigger_aabb[0][2]:.3f})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
