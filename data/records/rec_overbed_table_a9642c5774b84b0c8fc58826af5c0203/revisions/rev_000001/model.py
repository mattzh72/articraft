from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_overbed_table")

    powder_coat = model.material("warm_white_powder_coat", color=(0.86, 0.88, 0.84, 1.0))
    brushed_metal = model.material("brushed_stainless_steel", color=(0.63, 0.66, 0.66, 1.0))
    grey_plastic = model.material("soft_grey_plastic", color=(0.46, 0.48, 0.48, 1.0))
    black_rubber = model.material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    tray_laminate = model.material("speckled_warm_grey_laminate", color=(0.74, 0.72, 0.66, 1.0))
    dark_release = model.material("dark_release_plastic", color=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    # C-shaped rolling base: a side spine and two open-ended legs that can roll
    # under a bed, with all members physically tied together at the spine.
    base.visual(
        Box((0.080, 0.720, 0.050)),
        origin=Origin(xyz=(-0.360, 0.000, 0.095)),
        material=powder_coat,
        name="side_spine",
    )
    base.visual(
        Box((0.780, 0.080, 0.050)),
        origin=Origin(xyz=(0.000, -0.320, 0.095)),
        material=powder_coat,
        name="front_leg",
    )
    base.visual(
        Box((0.780, 0.080, 0.050)),
        origin=Origin(xyz=(0.000, 0.320, 0.095)),
        material=powder_coat,
        name="rear_leg",
    )

    # Column foot, gussets, and a hollow-looking square outer sleeve.  The four
    # walls leave a clear central bore for the sliding inner column.
    base.visual(
        Box((0.160, 0.150, 0.036)),
        origin=Origin(xyz=(-0.360, 0.040, 0.118)),
        material=powder_coat,
        name="column_foot",
    )
    base.visual(
        Box((0.130, 0.018, 0.135)),
        origin=Origin(xyz=(-0.360, -0.025, 0.180), rpy=(0.0, 0.55, 0.0)),
        material=powder_coat,
        name="front_gusset",
    )
    base.visual(
        Box((0.130, 0.018, 0.135)),
        origin=Origin(xyz=(-0.360, 0.105, 0.180), rpy=(0.0, 0.55, 0.0)),
        material=powder_coat,
        name="rear_gusset",
    )
    base.visual(
        Box((0.100, 0.012, 0.480)),
        origin=Origin(xyz=(-0.360, 0.000, 0.340)),
        material=brushed_metal,
        name="outer_front_wall",
    )
    base.visual(
        Box((0.100, 0.012, 0.480)),
        origin=Origin(xyz=(-0.360, 0.080, 0.340)),
        material=brushed_metal,
        name="outer_rear_wall",
    )
    base.visual(
        Box((0.012, 0.080, 0.480)),
        origin=Origin(xyz=(-0.404, 0.040, 0.340)),
        material=brushed_metal,
        name="outer_side_wall_0",
    )
    base.visual(
        Box((0.012, 0.080, 0.480)),
        origin=Origin(xyz=(-0.316, 0.040, 0.340)),
        material=brushed_metal,
        name="outer_side_wall_1",
    )
    base.visual(
        Box((0.060, 0.013, 0.070)),
        origin=Origin(xyz=(-0.360, 0.0125, 0.560)),
        material=grey_plastic,
        name="front_guide_pad",
    )
    base.visual(
        Box((0.060, 0.013, 0.070)),
        origin=Origin(xyz=(-0.360, 0.0675, 0.560)),
        material=grey_plastic,
        name="rear_guide_pad",
    )
    for z, prefix in ((0.109, "lower"), (0.580, "upper")):
        base.visual(
            Box((0.118, 0.012, 0.020)),
            origin=Origin(xyz=(-0.360, -0.010, z)),
            material=brushed_metal,
            name=f"{prefix}_collar_front",
        )
        base.visual(
            Box((0.118, 0.012, 0.020)),
            origin=Origin(xyz=(-0.360, 0.090, z)),
            material=brushed_metal,
            name=f"{prefix}_collar_rear",
        )
        base.visual(
            Box((0.012, 0.098, 0.020)),
            origin=Origin(xyz=(-0.416, 0.040, z)),
            material=brushed_metal,
            name=f"{prefix}_collar_side_0",
        )
        base.visual(
            Box((0.012, 0.098, 0.020)),
            origin=Origin(xyz=(-0.304, 0.040, z)),
            material=brushed_metal,
            name=f"{prefix}_collar_side_1",
        )

    caster_locations = (
        (-0.360, -0.320),
        (0.360, -0.320),
        (-0.360, 0.320),
        (0.360, 0.320),
    )
    for idx, (x, y) in enumerate(caster_locations):
        base.visual(
            Box((0.004, 0.026, 0.060)),
            origin=Origin(xyz=(x - 0.018, y, 0.056)),
            material=brushed_metal,
            name=f"caster_fork_{idx}_0",
        )
        base.visual(
            Box((0.004, 0.026, 0.060)),
            origin=Origin(xyz=(x + 0.018, y, 0.056)),
            material=brushed_metal,
            name=f"caster_fork_{idx}_1",
        )
        base.visual(
            Cylinder(radius=0.009, length=0.020),
            origin=Origin(xyz=(x, y, 0.100)),
            material=brushed_metal,
            name=f"caster_stem_{idx}",
        )
        base.visual(
            Cylinder(radius=0.0045, length=0.052),
            origin=Origin(xyz=(x, y, 0.036), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"caster_axle_{idx}",
        )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.052, 0.042, 0.500)),
        origin=Origin(xyz=(0.000, 0.000, -0.080)),
        material=brushed_metal,
        name="inner_post",
    )
    inner_column.visual(
        Box((0.410, 0.076, 0.060)),
        origin=Origin(xyz=(0.185, -0.010, 0.187)),
        material=brushed_metal,
        name="support_head",
    )
    inner_column.visual(
        Cylinder(radius=0.018, length=0.470),
        origin=Origin(xyz=(0.185, 0.000, 0.232), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_metal,
        name="tray_hinge_barrel",
    )
    inner_column.visual(
        Box((0.150, 0.110, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.145)),
        material=grey_plastic,
        name="height_release_collar",
    )

    model.articulation(
        "base_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_column,
        origin=Origin(xyz=(-0.360, 0.040, 0.580)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.270),
    )

    tray = model.part("tray")
    # The tray frame sits on the tilt hinge line; the tray surface is lifted just
    # above the hinge barrel and carries a raised lip around all four edges.
    tray.visual(
        Box((0.820, 0.420, 0.026)),
        origin=Origin(xyz=(0.180, 0.000, 0.031)),
        material=tray_laminate,
        name="tray_panel",
    )
    tray.visual(
        Box((0.830, 0.026, 0.052)),
        origin=Origin(xyz=(0.180, -0.210, 0.062)),
        material=grey_plastic,
        name="front_lip",
    )
    tray.visual(
        Box((0.830, 0.026, 0.052)),
        origin=Origin(xyz=(0.180, 0.210, 0.062)),
        material=grey_plastic,
        name="rear_lip",
    )
    tray.visual(
        Box((0.026, 0.420, 0.052)),
        origin=Origin(xyz=(-0.230, 0.000, 0.062)),
        material=grey_plastic,
        name="side_lip_0",
    )
    tray.visual(
        Box((0.026, 0.420, 0.052)),
        origin=Origin(xyz=(0.590, 0.000, 0.062)),
        material=grey_plastic,
        name="side_lip_1",
    )
    tray.visual(
        Box((0.034, 0.032, 0.052)),
        origin=Origin(xyz=(-0.070, 0.236, 0.114)),
        material=brushed_metal,
        name="clip_support_0",
    )
    tray.visual(
        Box((0.034, 0.032, 0.052)),
        origin=Origin(xyz=(0.430, 0.236, 0.114)),
        material=brushed_metal,
        name="clip_support_1",
    )
    tray.visual(
        Box((0.034, 0.024, 0.038)),
        origin=Origin(xyz=(0.060, -0.223, 0.017)),
        material=dark_release,
        name="lever_hinge_tab_0",
    )
    tray.visual(
        Box((0.034, 0.024, 0.038)),
        origin=Origin(xyz=(0.300, -0.223, 0.017)),
        material=dark_release,
        name="lever_hinge_tab_1",
    )

    model.articulation(
        "inner_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=tray,
        origin=Origin(xyz=(0.185, 0.000, 0.232)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.6, lower=-0.35, upper=0.35),
    )

    lever = model.part("release_lever")
    lever.visual(
        Box((0.300, 0.030, 0.014)),
        origin=Origin(xyz=(0.000, -0.041, -0.051)),
        material=dark_release,
        name="squeeze_grip",
    )
    lever.visual(
        Box((0.020, 0.040, 0.034)),
        origin=Origin(xyz=(-0.120, -0.018, -0.035)),
        material=dark_release,
        name="lever_arm_0",
    )
    lever.visual(
        Box((0.020, 0.040, 0.034)),
        origin=Origin(xyz=(0.120, -0.018, -0.035)),
        material=dark_release,
        name="lever_arm_1",
    )

    model.articulation(
        "tray_to_release_lever",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lever,
        origin=Origin(xyz=(0.180, -0.230, 0.016)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=0.55),
    )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        Cylinder(radius=0.008, length=0.580),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_metal,
        name="hinge_rod",
    )
    clip_bar.visual(
        Box((0.580, 0.032, 0.006)),
        origin=Origin(xyz=(0.000, -0.020, -0.006)),
        material=brushed_metal,
        name="chart_clamp_strip",
    )
    clip_bar.visual(
        Box((0.120, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, -0.035, -0.015)),
        material=grey_plastic,
        name="finger_pad",
    )

    model.articulation(
        "tray_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.180, 0.238, 0.148)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.05),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.036,
            0.024,
            inner_radius=0.026,
            tread=TireTread(style="ribbed", depth=0.002, count=12, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.003, radius=0.0015),
        ),
        "small_caster_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.027,
            0.020,
            rim=WheelRim(inner_radius=0.016, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(radius=0.009, width=0.018, cap_style="domed"),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0018, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.005),
        ),
        "small_caster_wheel",
    )
    for idx, (x, y) in enumerate(caster_locations):
        caster = model.part(f"caster_{idx}")
        caster.visual(
            tire_mesh,
            origin=Origin(),
            material=black_rubber,
            name="rubber_tire",
        )
        caster.visual(
            wheel_mesh,
            origin=Origin(),
            material=grey_plastic,
            name="wheel_core",
        )
        model.articulation(
            f"base_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.036)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_column = object_model.get_part("inner_column")
    tray = object_model.get_part("tray")
    lever = object_model.get_part("release_lever")
    clip_bar = object_model.get_part("clip_bar")
    lift = object_model.get_articulation("base_to_inner_column")
    tray_tilt = object_model.get_articulation("inner_column_to_tray")
    lever_joint = object_model.get_articulation("tray_to_release_lever")
    clip_joint = object_model.get_articulation("tray_to_clip_bar")

    # The inner post is intentionally retained inside the hollow outer sleeve at
    # both the low and high table settings, without treating the sleeve as a
    # solid proxy.
    ctx.expect_within(
        inner_column,
        base,
        axes="xy",
        inner_elem="inner_post",
        margin=0.002,
        name="inner post centered in outer sleeve",
    )
    ctx.expect_overlap(
        inner_column,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="outer_front_wall",
        min_overlap=0.240,
        name="low height column remains inserted",
    )
    low_pos = ctx.part_world_position(inner_column)
    with ctx.pose({lift: 0.270}):
        ctx.expect_within(
            inner_column,
            base,
            axes="xy",
            inner_elem="inner_post",
            margin=0.002,
            name="raised inner post still centered",
        )
        ctx.expect_overlap(
            inner_column,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_front_wall",
            min_overlap=0.060,
            name="raised column retains insertion",
        )
        high_pos = ctx.part_world_position(inner_column)
    ctx.check(
        "prismatic lift raises the tray support",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.25,
        details=f"low={low_pos}, high={high_pos}",
    )

    ctx.expect_overlap(
        tray,
        tray,
        axes="xy",
        elem_a="front_lip",
        elem_b="tray_panel",
        min_overlap=0.012,
        name="raised front lip is integrated with tray panel",
    )
    ctx.expect_gap(
        tray,
        lever,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lever_hinge_tab_0",
        negative_elem="lever_arm_0",
        name="release lever hangs from front hinge tab",
    )
    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: 0.55}):
        squeezed_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "squeeze lever rotates upward",
        rest_lever is not None
        and squeezed_lever is not None
        and squeezed_lever[1][2] > rest_lever[1][2] + 0.010,
        details=f"rest={rest_lever}, squeezed={squeezed_lever}",
    )

    ctx.expect_gap(
        clip_bar,
        tray,
        axis="z",
        min_gap=0.000,
        max_gap=0.030,
        positive_elem="hinge_rod",
        negative_elem="clip_support_0",
        name="clip bar is carried above its support",
    )
    ctx.expect_overlap(
        clip_bar,
        tray,
        axes="x",
        elem_a="hinge_rod",
        elem_b="clip_support_0",
        min_overlap=0.020,
        name="clip hinge line crosses support",
    )
    rest_clip = ctx.part_world_aabb(clip_bar)
    with ctx.pose({clip_joint: 0.75}):
        lifted_clip = ctx.part_world_aabb(clip_bar)
    ctx.check(
        "chart clip bar rotates away from rim",
        rest_clip is not None
        and lifted_clip is not None
        and lifted_clip[1][2] > rest_clip[1][2] + 0.006,
        details=f"rest={rest_clip}, lifted={lifted_clip}",
    )

    rest_tray = ctx.part_world_aabb(tray)
    with ctx.pose({tray_tilt: 0.30}):
        tilted_tray = ctx.part_world_aabb(tray)
    ctx.check(
        "tray tilts on support head hinge",
        rest_tray is not None
        and tilted_tray is not None
        and abs(tilted_tray[1][2] - rest_tray[1][2]) > 0.020,
        details=f"rest={rest_tray}, tilted={tilted_tray}",
    )

    for idx in range(4):
        caster = object_model.get_part(f"caster_{idx}")
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"caster_axle_{idx}",
            elem_b="wheel_core",
            reason="The caster axle is intentionally captured through the wheel hub bore.",
        )
        ctx.expect_overlap(
            caster,
            base,
            axes="x",
            elem_a="wheel_core",
            elem_b=f"caster_axle_{idx}",
            min_overlap=0.018,
            name=f"caster {idx} axle crosses wheel hub",
        )
        ctx.expect_within(
            caster,
            base,
            axes="x",
            outer_elem=f"caster_fork_{idx}_0",
            margin=0.080,
            name=f"caster {idx} sits between fork plates",
        )

    return ctx.report()


object_model = build_object_model()
