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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.84, 0.14, 0.12, 1.0))
    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.16, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.18, 0.18, 0.18, 1.0))

    def xy_section(
        width_x: float,
        width_y: float,
        radius: float,
        z: float,
        *,
        cx: float = 0.0,
        cy: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(cx + x, cy + y, z) for x, y in rounded_rect_profile(width_x, width_y, radius)]

    def yz_section(
        width_y: float,
        height_z: float,
        radius: float,
        x: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z_center + z) for y, z in rounded_rect_profile(width_y, height_z, radius)]

    base = model.part("base")

    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.34, 0.22, 0.05), 0.042),
        "mixer_base_plate",
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=body_paint,
        name="base_plate",
    )

    neck_shell = mesh_from_geometry(
        section_loft(
            [
                xy_section(0.13, 0.12, 0.032, 0.042, cx=-0.072),
                xy_section(0.105, 0.11, 0.029, 0.145, cx=-0.060),
                xy_section(0.082, 0.10, 0.024, 0.235, cx=-0.048),
                xy_section(0.068, 0.09, 0.020, 0.292, cx=-0.036),
            ]
        ),
        "mixer_pedestal_neck",
    )
    base.visual(neck_shell, material=body_paint, name="pedestal_neck")

    base.visual(
        Box((0.12, 0.018, 0.010)),
        origin=Origin(xyz=(0.085, -0.033, 0.047)),
        material=dark_trim,
        name="left_rail",
    )
    base.visual(
        Box((0.12, 0.018, 0.010)),
        origin=Origin(xyz=(0.085, 0.033, 0.047)),
        material=dark_trim,
        name="right_rail",
    )
    base.visual(
        Box((0.05, 0.03, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, 0.049)),
        material=body_paint,
        name="front_carriage_mount",
    )

    base.visual(
        Box((0.028, 0.024, 0.038)),
        origin=Origin(xyz=(-0.036, -0.054, 0.310)),
        material=body_paint,
        name="left_hinge_ear",
    )
    base.visual(
        Box((0.028, 0.024, 0.038)),
        origin=Origin(xyz=(-0.036, 0.054, 0.310)),
        material=body_paint,
        name="right_hinge_ear",
    )

    base.visual(
        Box((0.024, 0.014, 0.018)),
        origin=Origin(xyz=(-0.018, 0.106, 0.048)),
        material=body_paint,
        name="selector_boss",
    )
    base.visual(
        Box((0.050, 0.016, 0.004)),
        origin=Origin(xyz=(-0.105, -0.072, 0.044)),
        material=dark_trim,
        name="lock_guide",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.34)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.090, 0.014, 0.008)),
        origin=Origin(xyz=(0.050, -0.033, 0.004)),
        material=dark_trim,
        name="left_runner",
    )
    bowl_carriage.visual(
        Box((0.090, 0.014, 0.008)),
        origin=Origin(xyz=(0.050, 0.033, 0.004)),
        material=dark_trim,
        name="right_runner",
    )
    bowl_carriage.visual(
        Box((0.120, 0.104, 0.008)),
        origin=Origin(xyz=(0.078, 0.0, 0.012)),
        material=body_paint,
        name="carriage_tray",
    )
    bowl_carriage.visual(
        Box((0.020, 0.096, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.027)),
        material=body_paint,
        name="rear_yoke",
    )

    bowl_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.032, 0.000),
                (0.042, 0.010),
                (0.075, 0.055),
                (0.100, 0.110),
                (0.108, 0.168),
                (0.104, 0.178),
            ],
            [
                (0.000, 0.012),
                (0.030, 0.018),
                (0.066, 0.058),
                (0.094, 0.112),
                (0.100, 0.172),
            ],
            segments=64,
        ),
        "mixer_bowl_shell",
    )
    bowl_carriage.visual(
        bowl_shell,
        origin=Origin(xyz=(0.145, 0.0, 0.016)),
        material=stainless,
        name="bowl_shell",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.20)),
        mass=2.2,
        origin=Origin(xyz=(0.145, 0.0, 0.10)),
    )

    head = model.part("head")
    head_shell = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.096, 0.112, 0.028, 0.000, z_center=-0.008),
                yz_section(0.160, 0.170, 0.042, 0.100, z_center=-0.004),
                yz_section(0.154, 0.158, 0.040, 0.190, z_center=-0.010),
                yz_section(0.116, 0.110, 0.028, 0.282, z_center=-0.016),
            ]
        ),
        "mixer_head_shell",
    )
    head.visual(
        head_shell,
        origin=Origin(xyz=(0.040, 0.0, 0.055)),
        material=body_paint,
        name="head_shell",
    )
    head.visual(
        Box((0.050, 0.100, 0.050)),
        origin=Origin(xyz=(0.024, 0.0, 0.040)),
        material=body_paint,
        name="rear_shoulder",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.084),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.020),
        origin=Origin(xyz=(0.270, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.060),
        origin=Origin(xyz=(0.210, 0.0, -0.060)),
        material=stainless,
        name="drive_collar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.30, 0.16, 0.18)),
        mass=5.2,
        origin=Origin(xyz=(0.150, 0.0, -0.008)),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.0065, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=stainless,
        name="shaft",
    )
    paddle.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=stainless,
        name="hub_collar",
    )
    beater_outer = [
        (-0.040, -0.019),
        (-0.032, -0.028),
        (-0.006, -0.033),
        (0.022, -0.029),
        (0.038, -0.016),
        (0.040, 0.000),
        (0.038, 0.016),
        (0.022, 0.029),
        (-0.006, 0.033),
        (-0.032, 0.028),
        (-0.040, 0.019),
    ]
    beater_hole = [[
        (-0.022, -0.011),
        (-0.014, -0.018),
        (0.004, -0.020),
        (0.018, -0.014),
        (0.024, 0.000),
        (0.018, 0.014),
        (0.004, 0.020),
        (-0.014, 0.018),
        (-0.022, 0.011),
    ]]
    beater_frame = mesh_from_geometry(
        ExtrudeWithHolesGeometry(beater_outer, beater_hole, 0.005, center=True),
        "mixer_flat_beater_frame",
    )
    paddle.visual(
        beater_frame,
        origin=Origin(xyz=(0.0, 0.006, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="beater_frame",
    )
    paddle.visual(
        Box((0.006, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.004, -0.060)),
        material=stainless,
        name="beater_neck",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((0.014, 0.064, 0.124)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.004, -0.068)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="selector_axle",
    )
    speed_selector.visual(
        Box((0.010, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=dark_trim,
        name="selector_arm",
    )
    speed_selector.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
        material=knob_cap,
        name="selector_tip",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.016, 0.046, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    lock_release = model.part("lock_release")
    lock_release.visual(
        Box((0.030, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_trim,
        name="lock_slider_body",
    )
    lock_release.visual(
        Box((0.014, 0.018, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, 0.012)),
        material=knob_cap,
        name="lock_slider_grip",
    )
    lock_release.inertial = Inertial.from_geometry(
        Box((0.032, 0.018, 0.020)),
        mass=0.05,
        origin=Origin(xyz=(0.005, 0.0, 0.008)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.025, 0.0, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.08,
            lower=0.0,
            upper=0.038,
        ),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.036, 0.0, 0.310)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.210, 0.0, -0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(-0.018, 0.1182, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=math.radians(-25.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "base_to_lock_release",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_release,
        origin=Origin(xyz=(-0.105, -0.072, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=-0.008,
            upper=0.010,
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
    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    paddle = object_model.get_part("paddle")
    speed_selector = object_model.get_part("speed_selector")
    lock_release = object_model.get_part("lock_release")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_hinge = object_model.get_articulation("base_to_head")
    paddle_spin = object_model.get_articulation("head_to_paddle")
    selector_joint = object_model.get_articulation("base_to_speed_selector")
    lock_joint = object_model.get_articulation("base_to_lock_release")

    def center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    ctx.check(
        "bowl carriage uses short prismatic travel",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and bowl_slide.motion_limits is not None
        and bowl_slide.motion_limits.lower == 0.0
        and bowl_slide.motion_limits.upper is not None
        and bowl_slide.motion_limits.upper <= 0.04,
        details=str(bowl_slide.motion_limits),
    )
    ctx.check(
        "tilt head uses rear horizontal hinge",
        head_hinge.articulation_type == ArticulationType.REVOLUTE
        and head_hinge.axis == (0.0, -1.0, 0.0),
        details=f"type={head_hinge.articulation_type}, axis={head_hinge.axis}",
    )
    ctx.check(
        "paddle uses continuous vertical drive",
        paddle_spin.articulation_type == ArticulationType.CONTINUOUS
        and paddle_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={paddle_spin.articulation_type}, axis={paddle_spin.axis}",
    )
    ctx.check(
        "speed selector is revolute",
        selector_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={selector_joint.articulation_type}",
    )
    ctx.check(
        "lock release is prismatic",
        lock_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={lock_joint.articulation_type}",
    )

    ctx.expect_gap(
        bowl_carriage,
        base,
        axis="z",
        positive_elem="left_runner",
        negative_elem="left_rail",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="left runner rests on left rail",
    )
    ctx.expect_gap(
        bowl_carriage,
        base,
        axis="z",
        positive_elem="right_runner",
        negative_elem="right_rail",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="right runner rests on right rail",
    )
    ctx.expect_overlap(
        bowl_carriage,
        base,
        axes="x",
        elem_a="left_runner",
        elem_b="left_rail",
        min_overlap=0.085,
        name="left runner stays inserted at rest",
    )
    ctx.expect_overlap(
        bowl_carriage,
        base,
        axes="x",
        elem_a="right_runner",
        elem_b="right_rail",
        min_overlap=0.085,
        name="right runner stays inserted at rest",
    )
    ctx.expect_overlap(
        paddle,
        bowl_carriage,
        axes="xy",
        min_overlap=0.020,
        name="paddle hangs within bowl footprint",
    )

    rest_bowl_pos = ctx.part_world_position(bowl_carriage)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        extended_bowl_pos = ctx.part_world_position(bowl_carriage)
        ctx.expect_overlap(
            bowl_carriage,
            base,
            axes="x",
            elem_a="left_runner",
            elem_b="left_rail",
            min_overlap=0.075,
            name="left runner retains insertion when extended",
        )
        ctx.expect_overlap(
            bowl_carriage,
            base,
            axes="x",
            elem_a="right_runner",
            elem_b="right_rail",
            min_overlap=0.075,
            name="right runner retains insertion when extended",
        )
    ctx.check(
        "bowl carriage extends forward",
        rest_bowl_pos is not None
        and extended_bowl_pos is not None
        and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.03,
        details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
    )

    rest_paddle_pos = ctx.part_world_position(paddle)
    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        raised_paddle_pos = ctx.part_world_position(paddle)
    ctx.check(
        "head tilts upward away from bowl",
        rest_paddle_pos is not None
        and raised_paddle_pos is not None
        and raised_paddle_pos[2] > rest_paddle_pos[2] + 0.12,
        details=f"rest={rest_paddle_pos}, raised={raised_paddle_pos}",
    )

    rest_selector_tip = center_from_aabb(ctx.part_element_world_aabb(speed_selector, elem="selector_tip"))
    with ctx.pose({selector_joint: selector_joint.motion_limits.upper}):
        raised_selector_tip = center_from_aabb(ctx.part_element_world_aabb(speed_selector, elem="selector_tip"))
    ctx.check(
        "speed selector sweeps upward",
        rest_selector_tip is not None
        and raised_selector_tip is not None
        and raised_selector_tip[2] > rest_selector_tip[2] + 0.015,
        details=f"rest={rest_selector_tip}, raised={raised_selector_tip}",
    )

    rest_lock_pos = ctx.part_world_position(lock_release)
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        extended_lock_pos = ctx.part_world_position(lock_release)
    ctx.check(
        "lock release slides forward",
        rest_lock_pos is not None
        and extended_lock_pos is not None
        and extended_lock_pos[0] > rest_lock_pos[0] + 0.008,
        details=f"rest={rest_lock_pos}, extended={extended_lock_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
