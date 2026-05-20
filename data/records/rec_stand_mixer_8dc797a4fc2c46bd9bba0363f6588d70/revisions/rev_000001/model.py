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
    CylinderGeometry,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contemporary_stand_mixer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.96, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.88, 0.89, 0.91, 1.0))

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        *,
        z_center: float | None = None,
    ) -> list[tuple[float, float, float]]:
        center = height / 2.0 if z_center is None else z_center
        return [(x, y, z + center) for z, y in rounded_rect_profile(height, width, radius)]

    base = model.part("base")
    base_shell_geom = section_loft(
        [
            yz_section(-0.04, 0.15, 0.072, 0.028, z_center=0.036),
            yz_section(0.06, 0.24, 0.058, 0.026, z_center=0.029),
            yz_section(0.16, 0.27, 0.046, 0.022, z_center=0.023),
            yz_section(0.23, 0.20, 0.036, 0.016, z_center=0.018),
        ]
    )
    base_shell_geom.merge(
        section_loft(
            [
                yz_section(-0.15, 0.12, 0.22, 0.032, z_center=0.11),
                yz_section(-0.10, 0.16, 0.30, 0.045, z_center=0.15),
                yz_section(-0.04, 0.13, 0.17, 0.032, z_center=0.085),
            ]
        )
    )
    base.visual(
        mesh_from_geometry(base_shell_geom, "base_shell"),
        material=body_white,
        name="base_shell",
    )
    base.visual(
        Box((0.29, 0.15, 0.008)),
        origin=Origin(xyz=(0.05, 0.0, 0.074)),
        material=trim_dark,
        name="carriage_track",
    )
    base.visual(
        Box((0.05, 0.012, 0.045)),
        origin=Origin(xyz=(0.055, 0.129, 0.040)),
        material=trim_dark,
        name="speed_control_panel",
    )
    base.visual(
        Box((0.050, 0.024, 0.010)),
        origin=Origin(xyz=(-0.070, 0.0, 0.079)),
        material=trim_dark,
        name="head_lock_guide",
    )
    base.visual(
        Box((0.050, 0.020, 0.090)),
        origin=Origin(xyz=(-0.110, 0.040, 0.285)),
        material=body_white,
        name="left_hinge_fork",
    )
    base.visual(
        Box((0.050, 0.020, 0.090)),
        origin=Origin(xyz=(-0.110, -0.040, 0.285)),
        material=body_white,
        name="right_hinge_fork",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.28, 0.31)),
        mass=7.5,
        origin=Origin(xyz=(0.04, 0.0, 0.155)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.12, 0.14, 0.014)),
        origin=Origin(xyz=(0.06, 0.0, 0.007)),
        material=body_white,
        name="carriage_sled",
    )
    carriage.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.06, 0.0, 0.024)),
        material=body_white,
        name="bowl_pedestal",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(0.06, 0.0, 0.037)),
        material=trim_dark,
        name="bowl_seat",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.14, 0.05)),
        mass=0.9,
        origin=Origin(xyz=(0.06, 0.0, 0.025)),
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.032, 0.0),
            (0.072, 0.012),
            (0.108, 0.065),
            (0.121, 0.125),
            (0.125, 0.182),
        ],
        [
            (0.0, 0.004),
            (0.061, 0.014),
            (0.101, 0.065),
            (0.114, 0.124),
            (0.118, 0.178),
        ],
        segments=72,
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_geom, "mixing_bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.182),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
    )

    head = model.part("head")
    head_shell_geom = section_loft(
        [
            yz_section(0.045, 0.075, 0.070, 0.022, z_center=0.045),
            yz_section(0.125, 0.155, 0.150, 0.048, z_center=0.078),
            yz_section(0.225, 0.145, 0.125, 0.042, z_center=0.060),
            yz_section(0.305, 0.095, 0.074, 0.024, z_center=0.034),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell_geom, "head_shell"),
        material=body_white,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.060, 0.050, 0.040)),
        origin=Origin(xyz=(0.030, 0.0, 0.020)),
        material=body_white,
        name="hinge_knuckle",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.192, 0.0, -0.009)),
        material=trim_dark,
        name="drive_housing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.18)),
        mass=4.2,
        origin=Origin(xyz=(0.15, 0.0, 0.03)),
    )

    attachment = model.part("attachment")
    dough_hook_geom = CylinderGeometry(radius=0.009, height=0.032).translate(0.0, 0.0, -0.016)
    dough_hook_geom.merge(CylinderGeometry(radius=0.014, height=0.018).translate(0.0, 0.0, -0.041))
    dough_hook_geom.merge(
        tube_from_spline_points(
            [
                (0.0, 0.0, -0.048),
                (0.012, 0.0, -0.060),
                (0.026, 0.0, -0.084),
                (0.030, 0.0, -0.115),
                (0.020, 0.0, -0.145),
                (0.000, 0.0, -0.158),
                (-0.015, 0.0, -0.142),
                (-0.010, 0.0, -0.112),
            ],
            radius=0.007,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
    )
    attachment.visual(
        mesh_from_geometry(dough_hook_geom, "dough_hook"),
        material=steel,
        name="dough_hook",
    )
    attachment.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.22),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Box((0.015, 0.006, 0.028)),
        origin=Origin(xyz=(0.0075, 0.003, 0.014)),
        material=trim_dark,
        name="lever_stem",
    )
    speed_lever.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.020, 0.004, 0.030)),
        material=trim_dark,
        name="lever_paddle",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.030, 0.010, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.015, 0.005, 0.020)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.020, 0.016, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
        material=trim_dark,
        name="lock_slider",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.020, 0.016, 0.008)),
        mass=0.05,
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.03, 0.0, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.10, lower=0.0, upper=0.055),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.06, 0.0, 0.040)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.11, 0.0, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_attachment",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=attachment,
        origin=Origin(xyz=(0.192, 0.0, -0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=16.0),
    )
    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(0.050, 0.135, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=math.radians(-35.0),
            upper=math.radians(20.0),
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.085, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    attachment = object_model.get_part("attachment")
    speed_lever = object_model.get_part("speed_lever")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    spindle = object_model.get_articulation("head_to_attachment")
    speed_joint = object_model.get_articulation("base_to_speed_lever")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    def type_name(value: object) -> str:
        return getattr(value, "name", str(value)).split(".")[-1].lower()

    def axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(axis, expected))

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo_i + hi_i) / 2.0 for lo_i, hi_i in zip(lo, hi))

    def aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple(hi_i - lo_i for lo_i, hi_i in zip(lo, hi))

    ctx.check(
        "canonical mixer articulation types",
        type_name(bowl_slide.articulation_type) == "prismatic"
        and type_name(head_tilt.articulation_type) == "revolute"
        and type_name(spindle.articulation_type) == "continuous"
        and type_name(speed_joint.articulation_type) == "revolute"
        and type_name(lock_joint.articulation_type) == "prismatic",
        details=(
            f"slide={bowl_slide.articulation_type}, tilt={head_tilt.articulation_type}, "
            f"spindle={spindle.articulation_type}, speed={speed_joint.articulation_type}, "
            f"lock={lock_joint.articulation_type}"
        ),
    )
    ctx.check(
        "canonical mixer articulation axes",
        axis_matches(bowl_slide.axis, (1.0, 0.0, 0.0))
        and axis_matches(head_tilt.axis, (0.0, -1.0, 0.0))
        and axis_matches(spindle.axis, (0.0, 0.0, 1.0))
        and axis_matches(speed_joint.axis, (0.0, 1.0, 0.0))
        and axis_matches(lock_joint.axis, (1.0, 0.0, 0.0)),
        details=(
            f"slide={bowl_slide.axis}, tilt={head_tilt.axis}, spindle={spindle.axis}, "
            f"speed={speed_joint.axis}, lock={lock_joint.axis}"
        ),
    )
    ctx.check(
        "articulation limits match stand mixer travel",
        bowl_slide.motion_limits is not None
        and bowl_slide.motion_limits.upper is not None
        and bowl_slide.motion_limits.upper >= 0.05
        and head_tilt.motion_limits is not None
        and head_tilt.motion_limits.upper is not None
        and head_tilt.motion_limits.upper >= 1.0
        and speed_joint.motion_limits is not None
        and speed_joint.motion_limits.lower is not None
        and speed_joint.motion_limits.upper is not None
        and speed_joint.motion_limits.lower < 0.0 < speed_joint.motion_limits.upper
        and lock_joint.motion_limits is not None
        and lock_joint.motion_limits.upper is not None
        and 0.008 <= lock_joint.motion_limits.upper <= 0.02,
        details=(
            f"slide={bowl_slide.motion_limits}, tilt={head_tilt.motion_limits}, "
            f"speed={speed_joint.motion_limits}, lock={lock_joint.motion_limits}"
        ),
    )

    ctx.expect_overlap(
        attachment,
        bowl,
        axes="xy",
        min_overlap=0.02,
        name="dough hook hangs over the bowl footprint at rest",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        bowl_extended = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage slides forward",
        bowl_rest is not None and bowl_extended is not None and bowl_extended[0] > bowl_rest[0] + 0.05,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    hook_rest = ctx.part_world_position(attachment)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        hook_open = ctx.part_world_position(attachment)
        ctx.expect_gap(
            attachment,
            bowl,
            axis="z",
            min_gap=0.05,
            name="tilted head lifts the dough hook clear of the bowl",
        )
    ctx.check(
        "head tilt raises the attachment",
        hook_rest is not None
        and hook_open is not None
        and hook_open[2] > hook_rest[2] + 0.12,
        details=f"rest={hook_rest}, open={hook_open}",
    )

    attachment_aabb_0 = ctx.part_world_aabb(attachment)
    attachment_origin_0 = ctx.part_world_position(attachment)
    with ctx.pose({spindle: math.pi / 2.0}):
        attachment_aabb_90 = ctx.part_world_aabb(attachment)
        attachment_origin_90 = ctx.part_world_position(attachment)
    dims_0 = aabb_dims(attachment_aabb_0)
    dims_90 = aabb_dims(attachment_aabb_90)
    ctx.check(
        "dough hook spins about a vertical spindle",
        attachment_origin_0 is not None
        and attachment_origin_90 is not None
        and dims_0 is not None
        and dims_90 is not None
        and abs(attachment_origin_90[0] - attachment_origin_0[0]) < 1e-6
        and abs(attachment_origin_90[1] - attachment_origin_0[1]) < 1e-6
        and abs(attachment_origin_90[2] - attachment_origin_0[2]) < 1e-6
        and dims_0[0] > dims_90[0] + 0.015
        and dims_90[1] > dims_0[1] + 0.015,
        details=(
            f"origin_0={attachment_origin_0}, origin_90={attachment_origin_90}, "
            f"dims_0={dims_0}, dims_90={dims_90}"
        ),
    )

    lever_low = None
    lever_high = None
    with ctx.pose({speed_joint: speed_joint.motion_limits.lower}):
        lever_low = aabb_center(ctx.part_element_world_aabb(speed_lever, elem="lever_paddle"))
    with ctx.pose({speed_joint: speed_joint.motion_limits.upper}):
        lever_high = aabb_center(ctx.part_element_world_aabb(speed_lever, elem="lever_paddle"))
    ctx.check(
        "speed lever sweeps through a short rotary arc",
        lever_low is not None
        and lever_high is not None
        and lever_high[0] > lever_low[0] + 0.02
        and lever_low[2] > lever_high[2] + 0.01,
        details=f"low={lever_low}, high={lever_high}",
    )

    lock_rest = aabb_center(ctx.part_element_world_aabb(head_lock, elem="lock_slider"))
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        lock_extended = aabb_center(ctx.part_element_world_aabb(head_lock, elem="lock_slider"))
    ctx.check(
        "head lock slider travels linearly on the base",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.01
        and abs(lock_extended[1] - lock_rest[1]) < 1e-6
        and abs(lock_extended[2] - lock_rest[2]) < 1e-6,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
