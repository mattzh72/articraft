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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    body_enamel = model.material("body_enamel", rgba=(0.79, 0.16, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.88, 0.89, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    shadow_trim = model.material("shadow_trim", rgba=(0.22, 0.22, 0.24, 1.0))

    def xy_section(
        cx: float,
        z: float,
        sx: float,
        sy: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [(cx + x, y, z) for x, y in rounded_rect_profile(sx, sy, radius)]

    def yz_oval_section(
        x: float,
        width_y: float,
        height_z: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z_center + z)
            for y, z in superellipse_profile(
                width_y,
                height_z,
                exponent=2.7,
                segments=48,
            )
        ]

    base = model.part("base")

    pedestal_geom = ExtrudeGeometry(
        rounded_rect_profile(0.34, 0.22, 0.055, corner_segments=8),
        0.045,
    )
    base.visual(
        mesh_from_geometry(pedestal_geom, "base_pedestal"),
        origin=Origin(xyz=(0.060, 0.000, 0.0225)),
        material=body_enamel,
        name="pedestal",
    )

    deck_geom = ExtrudeGeometry(
        rounded_rect_profile(0.19, 0.14, 0.028, corner_segments=8),
        0.018,
    )
    base.visual(
        mesh_from_geometry(deck_geom, "base_deck"),
        origin=Origin(xyz=(0.100, 0.000, 0.054)),
        material=body_enamel,
        name="deck",
    )

    column_geom = section_loft(
        [
            xy_section(-0.078, 0.040, 0.112, 0.140, 0.035),
            xy_section(-0.070, 0.155, 0.100, 0.128, 0.032),
            xy_section(-0.061, 0.245, 0.090, 0.112, 0.028),
            xy_section(-0.056, 0.276, 0.082, 0.105, 0.025),
        ]
    )
    base.visual(
        mesh_from_geometry(column_geom, "rear_column"),
        material=body_enamel,
        name="rear_column",
    )

    base.visual(
        Box((0.075, 0.120, 0.022)),
        origin=Origin(xyz=(-0.055, 0.000, 0.266)),
        material=body_enamel,
        name="hinge_saddle",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(-0.020, 0.064, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_trim,
        name="selector_boss",
    )
    base.visual(
        Box((0.060, 0.020, 0.008)),
        origin=Origin(xyz=(-0.086, 0.040, 0.273)),
        material=shadow_trim,
        name="lock_rail",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.31)),
        mass=10.0,
        origin=Origin(xyz=(0.010, 0.000, 0.155)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Box((0.110, 0.015, 0.008)),
        origin=Origin(xyz=(0.000, -0.036, 0.004)),
        material=shadow_trim,
        name="left_runner",
    )
    bowl.visual(
        Box((0.110, 0.015, 0.008)),
        origin=Origin(xyz=(0.000, 0.036, 0.004)),
        material=shadow_trim,
        name="right_runner",
    )
    bowl.visual(
        Box((0.126, 0.095, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.013)),
        material=dark_trim,
        name="carriage_plate",
    )
    bowl.visual(
        Cylinder(radius=0.031, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.028)),
        material=steel,
        name="bowl_mount",
    )
    bowl_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.020, 0.000),
            (0.046, 0.012),
            (0.078, 0.040),
            (0.098, 0.082),
            (0.106, 0.124),
            (0.106, 0.134),
        ],
        [
            (0.012, 0.006),
            (0.040, 0.018),
            (0.072, 0.042),
            (0.092, 0.082),
            (0.100, 0.128),
        ],
        segments=64,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell_geom, "mixer_bowl_shell"),
        origin=Origin(xyz=(0.000, 0.000, 0.038)),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Box((0.22, 0.13, 0.18)),
        mass=1.2,
        origin=Origin(xyz=(0.000, 0.000, 0.090)),
    )

    head = model.part("head")
    head_shell_geom = section_loft(
        [
            yz_oval_section(0.018, 0.094, 0.094, z_center=0.038),
            yz_oval_section(0.115, 0.128, 0.108, z_center=0.036),
            yz_oval_section(0.220, 0.148, 0.118, z_center=0.030),
            yz_oval_section(0.312, 0.108, 0.092, z_center=0.020),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell_geom, "mixer_head_shell"),
        material=body_enamel,
        name="head_shell",
    )
    head.visual(
        Box((0.055, 0.086, 0.056)),
        origin=Origin(xyz=(0.012, 0.000, 0.016)),
        material=body_enamel,
        name="rear_cheek",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.110),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_trim,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.058, 0.066, 0.042)),
        origin=Origin(xyz=(0.140, 0.000, -0.008)),
        material=shadow_trim,
        name="hub_neck",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.048),
        origin=Origin(xyz=(0.158, 0.000, -0.010)),
        material=shadow_trim,
        name="hub_housing",
    )
    head.visual(
        Box((0.070, 0.090, 0.028)),
        origin=Origin(xyz=(0.265, 0.000, -0.010)),
        material=body_enamel,
        name="chin_fairing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.33, 0.16, 0.17)),
        mass=4.8,
        origin=Origin(xyz=(0.175, 0.000, 0.026)),
    )

    hook = model.part("hook")
    hook_geom = CylinderGeometry(radius=0.009, height=0.040).translate(0.0, 0.0, -0.020)
    hook_geom.merge(CylinderGeometry(radius=0.012, height=0.016).translate(0.0, 0.0, -0.048))
    hook_geom.merge(
        tube_from_spline_points(
            [
                (0.000, 0.000, -0.035),
                (0.002, 0.000, -0.058),
                (0.010, 0.000, -0.082),
                (0.020, 0.000, -0.108),
                (0.027, 0.000, -0.132),
                (0.024, 0.000, -0.149),
                (0.010, 0.000, -0.159),
                (-0.005, 0.000, -0.151),
            ],
            radius=0.005,
            samples_per_segment=18,
            radial_segments=18,
        )
    )
    hook.visual(
        mesh_from_geometry(hook_geom, "dough_hook"),
        material=steel,
        name="hook_body",
    )
    hook.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(-0.005, 0.000, -0.151)),
        material=steel,
        name="hook_tip",
    )
    hook.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.19),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, -0.095)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="selector_knob",
    )
    speed_selector.visual(
        Box((0.020, 0.006, 0.008)),
        origin=Origin(xyz=(0.010, 0.000, 0.012)),
        material=dark_trim,
        name="selector_tab",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.026, 0.012, 0.024)),
        mass=0.05,
        origin=Origin(xyz=(0.006, 0.000, 0.006)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.009, 0.000, 0.003)),
        material=dark_trim,
        name="lock_stem",
    )
    head_lock.visual(
        Box((0.024, 0.018, 0.008)),
        origin=Origin(xyz=(0.014, 0.000, 0.010)),
        material=dark_trim,
        name="lock_tab",
    )
    head_lock.visual(
        Box((0.010, 0.006, 0.006)),
        origin=Origin(xyz=(0.025, 0.000, 0.003)),
        material=dark_trim,
        name="lock_nose",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.030, 0.018, 0.016)),
        mass=0.04,
        origin=Origin(xyz=(0.015, 0.000, 0.007)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.095, 0.000, 0.063)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=0.035),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.055, 0.000, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "head_to_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hook,
        origin=Origin(xyz=(0.158, 0.000, -0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(-0.015, 0.081, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.112, 0.040, 0.277)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.014,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    hook = object_model.get_part("hook")
    speed_selector = object_model.get_part("speed_selector")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_tilt = object_model.get_articulation("base_to_head")
    hook_spin = object_model.get_articulation("head_to_hook")
    selector_joint = object_model.get_articulation("base_to_speed_selector")
    head_lock_joint = object_model.get_articulation("base_to_head_lock")

    def approx(value: float, target: float, tol: float = 1e-6) -> bool:
        return abs(value - target) <= tol

    def approx_axis(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
        return all(approx(a, b) for a, b in zip(axis, target))

    def aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def aabb_center_x(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    bowl_limits = bowl_slide.motion_limits
    head_limits = head_tilt.motion_limits
    hook_limits = hook_spin.motion_limits
    selector_limits = selector_joint.motion_limits
    lock_limits = head_lock_joint.motion_limits

    ctx.check(
        "bowl carriage articulation is a short forward prismatic slide",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and approx_axis(bowl_slide.axis, (1.0, 0.0, 0.0))
        and bowl_limits is not None
        and bowl_limits.lower is not None
        and bowl_limits.upper is not None
        and approx(bowl_limits.lower, 0.0)
        and 0.02 <= bowl_limits.upper <= 0.05,
        details=f"type={bowl_slide.articulation_type}, axis={bowl_slide.axis}, limits={bowl_limits}",
    )
    ctx.check(
        "rear hinge articulation is an upward head tilt revolute joint",
        head_tilt.articulation_type == ArticulationType.REVOLUTE
        and approx_axis(head_tilt.axis, (0.0, -1.0, 0.0))
        and head_limits is not None
        and head_limits.lower is not None
        and head_limits.upper is not None
        and approx(head_limits.lower, 0.0)
        and 0.9 <= head_limits.upper <= 1.2,
        details=f"type={head_tilt.articulation_type}, axis={head_tilt.axis}, limits={head_limits}",
    )
    ctx.check(
        "tool drive articulation is a continuous vertical spin",
        hook_spin.articulation_type == ArticulationType.CONTINUOUS
        and approx_axis(hook_spin.axis, (0.0, 0.0, 1.0))
        and hook_limits is not None
        and hook_limits.lower is None
        and hook_limits.upper is None,
        details=f"type={hook_spin.articulation_type}, axis={hook_spin.axis}, limits={hook_limits}",
    )
    ctx.check(
        "speed selector uses a small revolute control range",
        selector_joint.articulation_type == ArticulationType.REVOLUTE
        and approx_axis(selector_joint.axis, (0.0, 1.0, 0.0))
        and selector_limits is not None
        and selector_limits.lower is not None
        and selector_limits.upper is not None
        and selector_limits.lower < 0.0 < selector_limits.upper
        and selector_limits.upper <= 0.7,
        details=f"type={selector_joint.articulation_type}, axis={selector_joint.axis}, limits={selector_limits}",
    )
    ctx.check(
        "head lock uses a short prismatic control stroke",
        head_lock_joint.articulation_type == ArticulationType.PRISMATIC
        and approx_axis(head_lock_joint.axis, (1.0, 0.0, 0.0))
        and lock_limits is not None
        and lock_limits.lower is not None
        and lock_limits.upper is not None
        and approx(lock_limits.lower, 0.0)
        and 0.005 <= lock_limits.upper <= 0.02,
        details=f"type={head_lock_joint.articulation_type}, axis={head_lock_joint.axis}, limits={lock_limits}",
    )

    ctx.expect_within(
        hook,
        bowl,
        axes="xy",
        inner_elem="hook_body",
        outer_elem="bowl_shell",
        margin=0.0,
        name="hook stays centered over the bowl footprint at rest",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_limits.upper}):
        bowl_extended = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage extends forward",
        bowl_rest is not None
        and bowl_extended is not None
        and bowl_extended[0] > bowl_rest[0] + 0.02,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    chin_rest = ctx.part_element_world_aabb(head, elem="chin_fairing")
    with ctx.pose({head_tilt: head_limits.upper}):
        chin_open = ctx.part_element_world_aabb(head, elem="chin_fairing")
        ctx.expect_gap(
            hook,
            bowl,
            axis="z",
            min_gap=0.10,
            name="tilted head lifts the hook clear of the bowl",
        )
    ctx.check(
        "head front rises when the tilt head opens",
        chin_rest is not None
        and chin_open is not None
        and aabb_center_z(chin_open) is not None
        and aabb_center_z(chin_rest) is not None
        and aabb_center_z(chin_open) > aabb_center_z(chin_rest) + 0.18,
        details=f"closed={chin_rest}, open={chin_open}",
    )

    selector_low = None
    selector_high = None
    with ctx.pose({selector_joint: selector_limits.lower}):
        selector_low = ctx.part_element_world_aabb(speed_selector, elem="selector_tab")
    with ctx.pose({selector_joint: selector_limits.upper}):
        selector_high = ctx.part_element_world_aabb(speed_selector, elem="selector_tab")
    ctx.check(
        "speed selector visibly swings through its detent arc",
        selector_low is not None
        and selector_high is not None
        and aabb_center_x(selector_high) is not None
        and aabb_center_x(selector_low) is not None
        and aabb_center_x(selector_high) > aabb_center_x(selector_low) + 0.01,
        details=f"low={selector_low}, high={selector_high}",
    )

    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({head_lock_joint: lock_limits.upper}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slider moves forward into engagement",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.01,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
