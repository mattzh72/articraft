from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    try:
        return mesh_from_geometry(geometry, name)
    except Exception as exc:  # pragma: no cover - diagnostic context for compile-time geometry errors
        raise RuntimeError(f"failed to mesh {name}: {exc}") from exc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pastel_stand_mixer")

    pastel_mint = model.material("pastel_mint", rgba=(0.62, 0.84, 0.78, 1.0))
    mint_shadow = model.material("mint_shadow", rgba=(0.46, 0.68, 0.63, 1.0))
    cream = model.material("cream", rgba=(0.93, 0.88, 0.78, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.76, 0.74, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.16, 0.18, 0.18, 1.0))
    warm_label = model.material("warm_label", rgba=(0.96, 0.78, 0.62, 1.0))

    base = model.part("base")
    base_shell = ExtrudeGeometry(
        rounded_rect_profile(0.60, 0.34, 0.075, corner_segments=10),
        0.080,
        center=True,
    ).translate(0.0, 0.0, 0.040)
    base.visual(_mesh(base_shell, "base_shell"), material=pastel_mint, name="base_shell")
    base.visual(
        _mesh(
            ExtrudeGeometry(
                rounded_rect_profile(0.175, 0.210, 0.050, corner_segments=8),
                0.275,
                center=True,
            ).translate(-0.205, 0.0, 0.1925),
            "rear_pedestal",
        ),
        material=pastel_mint,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.31, 0.018, 0.010)),
        origin=Origin(xyz=(0.055, 0.127, 0.085)),
        material=mint_shadow,
        name="slide_rail_0",
    )
    base.visual(
        Box((0.31, 0.018, 0.010)),
        origin=Origin(xyz=(0.055, -0.127, 0.085)),
        material=mint_shadow,
        name="slide_rail_1",
    )
    base.visual(
        Box((0.13, 0.062, 0.004)),
        origin=Origin(xyz=(0.303, 0.083, 0.040)),
        material=dark_slot,
        name="lock_slot",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(-0.18, 0.105, 0.360), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_knuckle_0",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(-0.18, -0.105, 0.360), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_knuckle_1",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(-0.18, 0.145, 0.360), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_cap_0",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(-0.18, -0.145, 0.360), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_cap_1",
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.245, 0.205, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cream,
        name="carriage_plate",
    )
    bowl_carriage.visual(
        Box((0.075, 0.030, 0.034)),
        origin=Origin(xyz=(-0.080, 0.088, 0.044)),
        material=cream,
        name="bowl_lug_0",
    )
    bowl_carriage.visual(
        Box((0.075, 0.030, 0.034)),
        origin=Origin(xyz=(-0.080, -0.088, 0.044)),
        material=cream,
        name="bowl_lug_1",
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.050, 0.030),
            (0.082, 0.055),
            (0.116, 0.125),
            (0.136, 0.195),
            (0.142, 0.224),
        ],
        [
            (0.036, 0.038),
            (0.068, 0.062),
            (0.104, 0.128),
            (0.124, 0.190),
            (0.130, 0.214),
        ],
        segments=88,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    bowl_carriage.visual(
        _mesh(bowl_shell, "bowl_shell"),
        material=stainless,
        name="bowl_shell",
    )
    bowl_carriage.visual(
        _mesh(
            TorusGeometry(radius=0.139, tube=0.007, radial_segments=18, tubular_segments=88).translate(
                0.0, 0.0, 0.224
            ),
            "rolled_rim",
        ),
        material=stainless,
        name="rolled_rim",
    )

    head = model.part("head")
    head_body = (
        CapsuleGeometry(radius=0.078, length=0.315, radial_segments=32, height_segments=10)
        .rotate_y(pi / 2.0)
        .scale(1.0, 1.45, 0.88)
        .translate(0.275, 0.0, 0.052)
    )
    head.visual(_mesh(head_body, "head_body"), material=pastel_mint, name="head_body")
    head.visual(
        Box((0.075, 0.078, 0.082)),
        origin=Origin(xyz=(0.035, 0.0, 0.038)),
        material=pastel_mint,
        name="hinge_neck",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    head.visual(
        _mesh(TorusGeometry(radius=0.027, tube=0.0045, radial_segments=14, tubular_segments=48).translate(0.255, 0.0, -0.043), "tool_socket"),
        material=stainless,
        name="tool_socket",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.026),
        origin=Origin(xyz=(0.255, 0.0, -0.026)),
        material=stainless,
        name="tool_boss",
    )
    head.visual(
        Box((0.090, 0.020, 0.016)),
        origin=Origin(xyz=(0.205, -0.118, 0.070)),
        material=warm_label,
        name="pastel_badge",
    )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=stainless,
        name="hook_collar",
    )
    hook.visual(
        Cylinder(radius=0.006, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=stainless,
        name="hook_shaft",
    )
    hook_curve = tube_from_spline_points(
        [
            (0.0, 0.0, -0.080),
            (0.030, 0.000, -0.105),
            (0.060, 0.020, -0.145),
            (0.046, 0.056, -0.178),
            (-0.006, 0.058, -0.174),
            (-0.044, 0.026, -0.142),
            (-0.034, -0.010, -0.112),
        ],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    hook.visual(_mesh(hook_curve, "hook_tube"), material=stainless, name="hook_tube")

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.027, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=cream,
        name="selector_knob",
    )
    speed_selector.visual(
        Box((0.006, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=warm_label,
        name="selector_mark",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.016, 0.048, 0.020)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=cream,
        name="lock_tab",
    )
    head_lock.visual(
        Box((0.010, 0.018, 0.026)),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=warm_label,
        name="lock_grip",
    )

    model.articulation(
        "bowl_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.060, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=-0.035, upper=0.035),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.180, 0.0, 0.360)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=0.72),
    )
    model.articulation(
        "hook_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hook,
        origin=Origin(xyz=(0.255, 0.0, -0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "speed_select",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.2995, 0.0, 0.050), rpy=(0.0, pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "head_lock_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(0.300, 0.083, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=-0.018, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bowl = object_model.get_part("bowl_carriage")
    base = object_model.get_part("base")
    hook = object_model.get_part("hook")

    bowl_slide = object_model.get_articulation("bowl_slide")
    head_tilt = object_model.get_articulation("head_tilt")
    hook_spin = object_model.get_articulation("hook_spin")
    speed_select = object_model.get_articulation("speed_select")
    head_lock_slide = object_model.get_articulation("head_lock_slide")

    ctx.allow_overlap(
        bowl,
        hook,
        elem_a="bowl_shell",
        elem_b="hook_tube",
        reason="The dough hook intentionally hangs inside the hollow bowl; the thin bowl shell is represented by a closed collision proxy.",
    )

    ctx.check(
        "golden kinematic joint types",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and hook_spin.articulation_type == ArticulationType.CONTINUOUS
        and speed_select.articulation_type == ArticulationType.REVOLUTE
        and head_lock_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"types={bowl_slide.articulation_type}, {head_tilt.articulation_type}, "
            f"{hook_spin.articulation_type}, {speed_select.articulation_type}, "
            f"{head_lock_slide.articulation_type}"
        ),
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="carriage_plate",
        negative_elem="base_shell",
        name="bowl carriage rides on base",
    )
    ctx.expect_within(
        hook,
        bowl,
        axes="xy",
        margin=0.006,
        inner_elem="hook_tube",
        outer_elem="bowl_shell",
        name="dough hook hangs inside bowl footprint",
    )

    with ctx.pose({bowl_slide: bowl_slide.motion_limits.lower}):
        bowl_back = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        bowl_front = ctx.part_world_position(bowl)
    ctx.check(
        "bowl slide has short fore-aft travel",
        bowl_back is not None and bowl_front is not None and 0.060 <= bowl_front[0] - bowl_back[0] <= 0.080,
        details=f"back={bowl_back}, front={bowl_front}",
    )

    with ctx.pose({head_tilt: 0.0}):
        hook_closed = ctx.part_world_position(hook)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        hook_tilted = ctx.part_world_position(hook)
    ctx.check(
        "head tilt lifts the tool axis",
        hook_closed is not None and hook_tilted is not None and hook_tilted[2] > hook_closed[2] + 0.08,
        details=f"closed={hook_closed}, tilted={hook_tilted}",
    )

    lock = object_model.get_part("head_lock")
    with ctx.pose({head_lock_slide: head_lock_slide.motion_limits.lower}):
        lock_in = ctx.part_world_position(lock)
    with ctx.pose({head_lock_slide: head_lock_slide.motion_limits.upper}):
        lock_out = ctx.part_world_position(lock)
    ctx.check(
        "head lock control slides laterally",
        lock_in is not None and lock_out is not None and lock_out[1] - lock_in[1] > 0.030,
        details=f"in={lock_in}, out={lock_out}",
    )

    return ctx.report()


object_model = build_object_model()
