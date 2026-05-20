from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float, name: str):
    shape = cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)
    return mesh_from_cadquery(shape, name)


def _square_tube(outer: float, inner: float, length: float, name: str):
    shape = cq.Workplane("XY").rect(outer, outer).rect(inner, inner).extrude(length)
    return mesh_from_cadquery(shape, name)


def _cyl_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin(x: float, y: float, z: float, rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehabilitation_overbed_table")

    powder_coat = model.material("warm_white_powder_coat", rgba=(0.86, 0.88, 0.84, 1.0))
    dark_plastic = model.material("dark_gray_plastic", rgba=(0.03, 0.035, 0.04, 1.0))
    caster_hub_mat = model.material("caster_gray_hub", rgba=(0.55, 0.58, 0.58, 1.0))
    laminate = model.material("pale_maple_laminate", rgba=(0.86, 0.74, 0.55, 1.0))
    edge_trim = model.material("soft_gray_edge_trim", rgba=(0.58, 0.60, 0.58, 1.0))
    black_grip = model.material("black_rubber_grip", rgba=(0.015, 0.015, 0.014, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))

    base = model.part("base")
    # U-shaped low mobile base: open at +X, tied at the rear mast end.
    base.visual(Box((0.78, 0.055, 0.050)), origin=_origin(0.035, 0.285, 0.115), material=powder_coat, name="base_leg_0")
    base.visual(Box((0.78, 0.055, 0.050)), origin=_origin(0.035, -0.285, 0.115), material=powder_coat, name="base_leg_1")
    base.visual(Box((0.070, 0.620, 0.050)), origin=_origin(-0.355, 0.0, 0.115), material=powder_coat, name="rear_crossbar")
    base.visual(Box((0.140, 0.120, 0.026)), origin=_origin(-0.355, 0.0, 0.158), material=powder_coat, name="mast_foot_plate")
    base.visual(_square_tube(0.078, 0.052, 0.600, "outer_mast_sleeve"), origin=_origin(-0.355, 0.0, 0.105), material=powder_coat, name="outer_sleeve")
    base.visual(Cylinder(radius=0.017, length=0.030), origin=_origin(-0.355, -0.052, 0.700, (pi / 2.0, 0.0, 0.0)), material=stainless, name="height_lock_knob")

    caster_points = (
        (-0.300, 0.285),
        (0.385, 0.285),
        (-0.300, -0.285),
        (0.385, -0.285),
    )
    for idx, (x, y) in enumerate(caster_points):
        # Each caster fork is part of the welded base.  The wheel itself is a
        # separate continuous joint around the visible axle.
        base.visual(Box((0.055, 0.070, 0.012)), origin=_origin(x, y, 0.104), material=powder_coat, name=f"caster_crown_{idx}")
        base.visual(Box((0.018, 0.008, 0.072)), origin=_origin(x, y - 0.023, 0.063), material=powder_coat, name=f"caster_fork_{idx}_0")
        base.visual(Box((0.018, 0.008, 0.072)), origin=_origin(x, y + 0.023, 0.063), material=powder_coat, name=f"caster_fork_{idx}_1")
        base.visual(Cylinder(radius=0.006, length=0.070), origin=_origin(x, y, 0.045, (pi / 2.0, 0.0, 0.0)), material=stainless, name=f"caster_axle_{idx}")

    mast = model.part("mast")
    mast.visual(Box((0.040, 0.040, 0.540)), origin=_origin(0.0, 0.0, -0.010), material=powder_coat, name="inner_post")
    mast.visual(Box((0.007, 0.030, 0.070)), origin=_origin(0.02275, 0.0, -0.240), material=dark_plastic, name="glide_pad_x0")
    mast.visual(Box((0.007, 0.030, 0.070)), origin=_origin(-0.02275, 0.0, -0.240), material=dark_plastic, name="glide_pad_x1")
    mast.visual(Box((0.030, 0.007, 0.070)), origin=_origin(0.0, 0.02275, -0.240), material=dark_plastic, name="glide_pad_y0")
    mast.visual(Box((0.030, 0.007, 0.070)), origin=_origin(0.0, -0.02275, -0.240), material=dark_plastic, name="glide_pad_y1")
    mast.visual(Box((0.520, 0.640, 0.040)), origin=_origin(0.260, -0.050, 0.250), material=powder_coat, name="support_head")
    mast.visual(Box((0.080, 0.110, 0.120)), origin=_origin(0.045, 0.0, 0.215), material=powder_coat, name="head_neck")
    mast.visual(Cylinder(radius=0.012, length=0.480), origin=_origin(0.240, 0.050, 0.290, (pi / 2.0, 0.0, 0.0)), material=stainless, name="top_hinge_pin")
    mast.visual(Box((0.042, 0.030, 0.052)), origin=_origin(0.240, -0.160, 0.276), material=powder_coat, name="hinge_saddle_0")
    mast.visual(Box((0.042, 0.030, 0.052)), origin=_origin(0.240, 0.050, 0.276), material=powder_coat, name="hinge_saddle_1")
    mast.visual(Box((0.042, 0.030, 0.052)), origin=_origin(0.240, 0.200, 0.276), material=powder_coat, name="hinge_saddle_2")
    mast.visual(Cylinder(radius=0.019, length=0.046), origin=_origin(0.255, 0.286, 0.246, (pi / 2.0, 0.0, 0.0)), material=stainless, name="handle_boss")

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=_origin(-0.355, 0.0, 0.680),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.240, effort=120.0, velocity=0.18),
    )

    main_top = model.part("main_top")
    main_top.visual(_rounded_box((0.680, 0.430, 0.030), 0.030, "main_laminate_panel"), origin=_origin(0.370, 0.0, 0.020), material=laminate, name="main_panel")
    main_top.visual(Box((0.014, 0.450, 0.026)), origin=_origin(0.720, 0.0, 0.049), material=edge_trim, name="front_rim")
    main_top.visual(Box((0.700, 0.014, 0.022)), origin=_origin(0.375, 0.222, 0.046), material=edge_trim, name="side_rim_0")
    main_top.visual(Box((0.700, 0.014, 0.022)), origin=_origin(0.375, -0.222, 0.046), material=edge_trim, name="side_rim_1")
    main_top.visual(Box((0.070, 0.034, 0.012)), origin=_origin(0.034, -0.160, 0.018), material=stainless, name="top_hinge_leaf_0")
    main_top.visual(Box((0.070, 0.034, 0.012)), origin=_origin(0.034, 0.050, 0.018), material=stainless, name="top_hinge_leaf_1")
    main_top.visual(Box((0.070, 0.034, 0.012)), origin=_origin(0.034, 0.200, 0.018), material=stainless, name="top_hinge_leaf_2")

    model.articulation(
        "top_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=main_top,
        origin=_origin(0.240, 0.050, 0.290),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=18.0, velocity=0.8),
    )

    side_wing = model.part("side_wing")
    side_wing.visual(_rounded_box((0.340, 0.180, 0.030), 0.020, "side_wing_panel"), origin=_origin(0.0, 0.0, 0.020), material=laminate, name="wing_panel")
    side_wing.visual(Box((0.360, 0.012, 0.020)), origin=_origin(0.0, -0.084, 0.044), material=edge_trim, name="wing_outer_rim")
    side_wing.visual(Box((0.044, 0.044, 0.019)), origin=_origin(-0.095, 0.040, -0.004), material=powder_coat, name="wing_mount_pad_0")
    side_wing.visual(Box((0.044, 0.044, 0.019)), origin=_origin(0.095, 0.040, -0.004), material=powder_coat, name="wing_mount_pad_1")
    model.articulation(
        "mast_to_wing",
        ArticulationType.FIXED,
        parent=mast,
        child=side_wing,
        origin=_origin(0.400, -0.310, 0.2835),
    )

    tilt_handle = model.part("tilt_handle")
    tilt_handle.visual(Cylinder(radius=0.018, length=0.055), origin=_origin(0.0, 0.0, 0.0, (pi / 2.0, 0.0, 0.0)), material=stainless, name="handle_pivot")
    tilt_handle.visual(Box((0.018, 0.012, 0.145)), origin=_origin(0.018, 0.0, -0.072), material=stainless, name="release_lever")
    tilt_handle.visual(Cylinder(radius=0.012, length=0.090), origin=_origin(0.030, 0.0, -0.150, (pi / 2.0, 0.0, 0.0)), material=black_grip, name="release_grip")
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=tilt_handle,
        origin=_origin(0.255, 0.315, 0.246),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.45, effort=4.0, velocity=1.5),
    )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(Cylinder(radius=0.0055, length=0.390), origin=_origin(0.0, 0.0, 0.0, (pi / 2.0, 0.0, 0.0)), material=stainless, name="clip_hinge_rod")
    clip_bar.visual(Box((0.076, 0.014, 0.014)), origin=_origin(-0.038, -0.150, 0.010), material=stainless, name="clip_support_0")
    clip_bar.visual(Box((0.076, 0.014, 0.014)), origin=_origin(-0.038, 0.150, 0.010), material=stainless, name="clip_support_1")
    clip_bar.visual(Cylinder(radius=0.008, length=0.390), origin=_origin(-0.076, 0.0, 0.023, (pi / 2.0, 0.0, 0.0)), material=stainless, name="chart_bar")
    model.articulation(
        "clip_hinge",
        ArticulationType.REVOLUTE,
        parent=main_top,
        child=clip_bar,
        origin=_origin(0.716, 0.0, 0.0665),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=2.0, velocity=2.0),
    )

    for idx, (x, y) in enumerate(caster_points):
        caster = model.part(f"caster_{idx}")
        caster.visual(Cylinder(radius=0.045, length=0.026), origin=_origin(0.0, 0.0, 0.0, (pi / 2.0, 0.0, 0.0)), material=dark_plastic, name="tire")
        caster.visual(Cylinder(radius=0.022, length=0.032), origin=_origin(0.0, 0.0, 0.0, (pi / 2.0, 0.0, 0.0)), material=caster_hub_mat, name="hub")
        model.articulation(
            f"caster_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=_origin(x, y, 0.045),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    main_top = object_model.get_part("main_top")
    side_wing = object_model.get_part("side_wing")
    tilt_handle = object_model.get_part("tilt_handle")
    clip_bar = object_model.get_part("clip_bar")
    height_slide = object_model.get_articulation("height_slide")
    top_tilt = object_model.get_articulation("top_tilt")
    clip_hinge = object_model.get_articulation("clip_hinge")

    # Captured shafts/pins are intentionally represented as local solid-in-solid
    # insertions so the visible mechanisms read as mounted rather than floating.
    ctx.allow_overlap(
        "mast",
        "tilt_handle",
        elem_a="handle_boss",
        elem_b="handle_pivot",
        reason="The release handle pivot is a captured shaft seated in the side boss.",
    )
    for idx in range(4):
        ctx.allow_overlap(
            "base",
            f"caster_{idx}",
            elem_a=f"caster_axle_{idx}",
            elem_b="hub",
            reason="Each caster hub spins around a visible axle pin through the fork.",
        )
        ctx.allow_overlap(
            "base",
            f"caster_{idx}",
            elem_a=f"caster_axle_{idx}",
            elem_b="tire",
            reason="The caster tire is a simplified solid roller with the axle passing through its center.",
        )
    ctx.allow_overlap(
        "main_top",
        "clip_bar",
        elem_a="front_rim",
        elem_b="clip_hinge_rod",
        reason="The chart clip hinge rod is captured along the tray rim.",
    )
    for pad in ("glide_pad_x0", "glide_pad_x1", "glide_pad_y0", "glide_pad_y1"):
        ctx.allow_overlap(
            "base",
            "mast",
            elem_a="outer_sleeve",
            elem_b=pad,
            reason="Low-friction guide pads ride lightly against the inside of the telescoping mast sleeve.",
        )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="outer_sleeve",
        margin=0.001,
        name="mast stays centered in outer sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="outer_sleeve",
        min_overlap=0.020,
        name="low mast setting remains inserted",
    )
    rest_height = ctx.part_world_position(mast)
    with ctx.pose({height_slide: 0.240}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="outer_sleeve",
            margin=0.001,
            name="raised mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.015,
            name="raised mast retains insertion",
        )
        raised_height = ctx.part_world_position(mast)
    ctx.check(
        "height slide raises the support head",
        rest_height is not None and raised_height is not None and raised_height[2] > rest_height[2] + 0.20,
        details=f"rest={rest_height}, raised={raised_height}",
    )

    ctx.expect_gap(
        side_wing,
        mast,
        axis="z",
        positive_elem="wing_mount_pad_0",
        negative_elem="support_head",
        max_gap=0.001,
        max_penetration=0.001,
        name="side wing pad sits on shared support head",
    )
    ctx.expect_gap(
        main_top,
        mast,
        axis="z",
        positive_elem="top_hinge_leaf_1",
        negative_elem="top_hinge_pin",
        max_gap=0.001,
        max_penetration=0.001,
        name="main top hinge leaf bears on head pin",
    )
    ctx.expect_overlap(
        clip_bar,
        main_top,
        axes="y",
        elem_a="clip_hinge_rod",
        elem_b="front_rim",
        min_overlap=0.300,
        name="clip rod spans the tray front rim",
    )
    ctx.expect_gap(
        clip_bar,
        main_top,
        axis="z",
        positive_elem="clip_hinge_rod",
        negative_elem="front_rim",
        max_gap=0.001,
        max_penetration=0.002,
        name="clip rod is seated on rim hinge line",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="glide_pad_x0",
        elem_b="outer_sleeve",
        min_overlap=0.050,
        name="mast guide pad remains inside sleeve",
    )

    low_top_aabb = ctx.part_element_world_aabb(main_top, elem="main_panel")
    with ctx.pose({top_tilt: 0.85}):
        tilted_top_aabb = ctx.part_element_world_aabb(main_top, elem="main_panel")
    ctx.check(
        "main top tilt lifts the tray surface",
        low_top_aabb is not None
        and tilted_top_aabb is not None
        and tilted_top_aabb[1][2] > low_top_aabb[1][2] + 0.16,
        details=f"low={low_top_aabb}, tilted={tilted_top_aabb}",
    )

    low_clip_aabb = ctx.part_world_aabb(clip_bar)
    with ctx.pose({clip_hinge: 1.10}):
        lifted_clip_aabb = ctx.part_world_aabb(clip_bar)
    ctx.check(
        "chart clip bar hinges upward from the tray rim",
        low_clip_aabb is not None
        and lifted_clip_aabb is not None
        and lifted_clip_aabb[1][2] > low_clip_aabb[1][2] + 0.035,
        details=f"low={low_clip_aabb}, lifted={lifted_clip_aabb}",
    )

    ctx.expect_overlap(
        tilt_handle,
        mast,
        axes="xyz",
        elem_a="handle_pivot",
        elem_b="handle_boss",
        min_overlap=0.010,
        name="tilt handle pivot is captured in boss",
    )
    for idx in range(4):
        caster = object_model.get_part(f"caster_{idx}")
        ctx.expect_overlap(
            caster,
            base,
            axes="xyz",
            elem_a="hub",
            elem_b=f"caster_axle_{idx}",
            min_overlap=0.006,
            name=f"caster {idx} hub contains axle",
        )
        spin = object_model.get_articulation(f"caster_spin_{idx}")
        ctx.check(
            f"caster {idx} uses continuous spin",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            details=str(spin.articulation_type),
        )

    return ctx.report()


object_model = build_object_model()
