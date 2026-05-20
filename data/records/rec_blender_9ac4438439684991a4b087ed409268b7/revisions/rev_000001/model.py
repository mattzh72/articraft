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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _yz_section(
    width_y: float,
    height_z: float,
    radius: float,
    x_pos: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y_pos, z_pos + z_center) for z_pos, y_pos in rounded_rect_profile(height_z, width_y, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="milkshake_spindle_mixer")

    cream_paint = model.material("cream_paint", rgba=(0.92, 0.90, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base_shell = ExtrudeGeometry.from_z0(
        superellipse_profile(0.120, 0.190, exponent=2.2, segments=56),
        0.030,
        cap=True,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_shell"),
        material=cream_paint,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.058, 0.0, 0.033)),
        material=dark_trim,
        name="cup_seat",
    )
    base.visual(
        Box((0.050, 0.060, 0.014)),
        origin=Origin(xyz=(-0.052, 0.0, 0.037)),
        material=dark_trim,
        name="post_plinth",
    )
    base.visual(
        Box((0.024, 0.018, 0.004)),
        origin=Origin(xyz=(0.060, 0.030, 0.002)),
        material=rubber,
        name="foot_left",
    )
    base.visual(
        Box((0.024, 0.018, 0.004)),
        origin=Origin(xyz=(0.060, -0.030, 0.002)),
        material=rubber,
        name="foot_right",
    )
    base.visual(
        Box((0.020, 0.018, 0.004)),
        origin=Origin(xyz=(-0.050, 0.0, 0.002)),
        material=rubber,
        name="foot_rear",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.190, 0.120, 0.055)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.011, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=chrome,
        name="main_post",
    )
    post.visual(
        Box((0.024, 0.046, 0.020)),
        origin=Origin(xyz=(-0.012, 0.0, 0.330)),
        material=chrome,
        name="pivot_cap",
    )
    post.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.003, 0.019, 0.350), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_ear_left",
    )
    post.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.003, -0.019, 0.350), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_ear_right",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.040, 0.050, 0.360)),
        mass=1.5,
        origin=Origin(xyz=(0.003, 0.0, 0.180)),
    )

    cup = model.part("cup")
    cup_geom = LatheGeometry.from_shell_profiles(
        [
            (0.016, 0.000),
            (0.022, 0.004),
            (0.031, 0.016),
            (0.038, 0.094),
            (0.041, 0.146),
            (0.043, 0.158),
        ],
        [
            (0.000, 0.004),
            (0.026, 0.012),
            (0.033, 0.094),
            (0.036, 0.146),
            (0.038, 0.154),
        ],
        segments=56,
    )
    cup.visual(
        mesh_from_geometry(cup_geom, "cup_shell"),
        material=brushed_steel,
        name="cup_shell",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.158),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
    )

    arm = model.part("motor_arm")
    arm_shell_geom = section_loft(
        [
            _yz_section(0.026, 0.034, 0.010, 0.022, z_center=-0.014),
            _yz_section(0.072, 0.074, 0.022, 0.052, z_center=-0.016),
            _yz_section(0.060, 0.062, 0.018, 0.092, z_center=-0.017),
            _yz_section(0.034, 0.036, 0.010, 0.122, z_center=-0.017),
        ]
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="rear_hinge_barrel",
    )
    arm.visual(
        Box((0.022, 0.022, 0.018)),
        origin=Origin(xyz=(0.011, 0.0, -0.012)),
        material=dark_trim,
        name="hinge_neck",
    )
    arm.visual(
        mesh_from_geometry(arm_shell_geom, "arm_shell"),
        material=cream_paint,
        name="arm_shell",
    )
    arm.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.110, 0.0, -0.030)),
        material=dark_trim,
        name="gearcase",
    )
    arm.visual(
        Box((0.036, 0.040, 0.016)),
        origin=Origin(xyz=(0.050, 0.0, 0.014)),
        material=dark_trim,
        name="motor_band",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.132, 0.075, 0.080)),
        mass=1.8,
        origin=Origin(xyz=(0.066, 0.0, -0.006)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_trim,
        name="drive_hub",
    )
    spindle.visual(
        Cylinder(radius=0.0045, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, -0.096)),
        material=chrome,
        name="shaft_body",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=chrome,
        name="shaft_collar",
    )
    spindle.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.167)),
        material=chrome,
        name="agitator_disk",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.172),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.FIXED,
        parent=base,
        child=post,
        origin=Origin(xyz=(-0.052, 0.0, 0.044)),
    )
    model.articulation(
        "base_to_cup",
        ArticulationType.FIXED,
        parent=base,
        child=cup,
        origin=Origin(xyz=(0.058, 0.0, 0.036)),
    )
    model.articulation(
        "post_to_arm",
        ArticulationType.REVOLUTE,
        parent=post,
        child=arm,
        origin=Origin(xyz=(0.003, 0.0, 0.350)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(6.0),
        ),
    )
    model.articulation(
        "arm_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=spindle,
        origin=Origin(xyz=(0.110, 0.0, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=28.0),
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
    post = object_model.get_part("post")
    cup = object_model.get_part("cup")
    arm = object_model.get_part("motor_arm")
    spindle = object_model.get_part("spindle")

    swing = object_model.get_articulation("post_to_arm")
    spin = object_model.get_articulation("arm_to_spindle")

    ctx.check(
        "arm articulation is revolute",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={swing.articulation_type}",
    )
    ctx.check(
        "spindle articulation is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.check(
        "arm closes downward from raised pose",
        swing.motion_limits is not None
        and swing.motion_limits.lower is not None
        and swing.motion_limits.upper is not None
        and swing.motion_limits.lower < 0.0 < swing.motion_limits.upper,
        details=f"limits={swing.motion_limits}",
    )

    ctx.expect_contact(
        cup,
        base,
        elem_a="cup_shell",
        elem_b="cup_seat",
        name="cup sits on the base seat",
    )
    ctx.expect_contact(
        post,
        base,
        elem_a="main_post",
        elem_b="post_plinth",
        name="post stands on the rear plinth",
    )

    with ctx.pose({swing: 0.0}):
        ctx.expect_within(
            spindle,
            cup,
            axes="xy",
            inner_elem="shaft_body",
            outer_elem="cup_shell",
            margin=0.004,
            name="lowered spindle stays centered over the cup",
        )
        ctx.expect_overlap(
            spindle,
            cup,
            axes="z",
            elem_a="shaft_body",
            elem_b="cup_shell",
            min_overlap=0.020,
            name="lowered spindle enters the cup opening",
        )

    mixed_pos = ctx.part_world_position(spindle)
    cup_pos = ctx.part_world_position(cup)
    with ctx.pose({swing: math.radians(-30.0)}):
        ctx.expect_gap(
            spindle,
            cup,
            axis="z",
            positive_elem="shaft_body",
            negative_elem="cup_shell",
            min_gap=0.020,
            name="raised arm lifts the spindle clear of the cup",
        )
        raised_pos = ctx.part_world_position(spindle)

    ctx.check(
        "raised arm moves spindle upward for cup loading",
        mixed_pos is not None
        and raised_pos is not None
        and raised_pos[2] > mixed_pos[2] + 0.025,
        details=f"mixed={mixed_pos}, raised={raised_pos}",
    )
    ctx.check(
        "mixing position stays centered over the cup",
        mixed_pos is not None
        and cup_pos is not None
        and abs(mixed_pos[0] - cup_pos[0]) < 0.020
        and abs(mixed_pos[1] - cup_pos[1]) < 0.005,
        details=f"spindle={mixed_pos}, cup={cup_pos}",
    )
    ctx.check(
        "spin joint is aligned with the spindle axis",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
