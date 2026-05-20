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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
):
    return [(cx + x, cy + y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    depth: float,
    height: float,
    radius: float,
    x: float,
    *,
    cy: float = 0.0,
    cz: float = 0.0,
):
    return [(x, cy + y, cz + z) for y, z in rounded_rect_profile(depth, height, radius)]


def _build_dough_hook_mesh():
    return _save_mesh(
        "dough_hook_body",
        tube_from_spline_points(
            [
                (0.0, 0.0, -0.046),
                (0.010, 0.0, -0.060),
                (0.024, 0.0, -0.086),
                (0.036, 0.0, -0.115),
                (0.040, 0.0, -0.141),
                (0.032, 0.0, -0.160),
                (0.017, 0.0, -0.173),
                (0.001, 0.0, -0.162),
                (0.004, 0.0, -0.139),
                (0.015, 0.0, -0.120),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bakery_prep_stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.42, 0.44, 0.47, 1.0))
    body_shadow = model.material("body_shadow", rgba=(0.22, 0.24, 0.27, 1.0))
    bowl_steel = model.material("bowl_steel", rgba=(0.83, 0.86, 0.89, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.10, 0.11, 0.12, 1.0))

    dough_hook_mesh = _build_dough_hook_mesh()

    pedestal = model.part("pedestal")
    pedestal.visual(
        _save_mesh(
            "pedestal_foot",
            ExtrudeGeometry(rounded_rect_profile(0.44, 0.30, 0.055), 0.05),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=body_shadow,
        name="base_foot",
    )
    pedestal.visual(
        _save_mesh(
            "pedestal_deck",
            ExtrudeGeometry(rounded_rect_profile(0.28, 0.22, 0.035), 0.06),
        ),
        origin=Origin(xyz=(0.06, 0.0, 0.08)),
        material=body_paint,
        name="front_deck",
    )
    pedestal.visual(
        _save_mesh(
            "pedestal_column",
            section_loft(
                [
                    _xy_section(0.18, 0.18, 0.045, 0.05, cx=-0.08),
                    _xy_section(0.17, 0.17, 0.042, 0.18, cx=-0.09),
                    _xy_section(0.14, 0.15, 0.035, 0.30, cx=-0.10),
                    _xy_section(0.09, 0.11, 0.024, 0.385, cx=-0.125),
                ]
            ),
        ),
        material=body_paint,
        name="rear_column",
    )
    pedestal.visual(
        Box((0.070, 0.124, 0.030)),
        origin=Origin(xyz=(-0.135, 0.0, 0.375)),
        material=body_shadow,
        name="hinge_yoke_bridge",
    )
    pedestal.visual(
        Box((0.035, 0.030, 0.070)),
        origin=Origin(xyz=(-0.115, -0.047, 0.425)),
        material=body_shadow,
        name="left_hinge_lug",
    )
    pedestal.visual(
        Box((0.035, 0.030, 0.070)),
        origin=Origin(xyz=(-0.115, 0.047, 0.425)),
        material=body_shadow,
        name="right_hinge_lug",
    )
    pedestal.visual(
        Box((0.20, 0.028, 0.014)),
        origin=Origin(xyz=(0.065, -0.098, 0.117)),
        material=body_shadow,
        name="left_slide_rail",
    )
    pedestal.visual(
        Box((0.20, 0.028, 0.014)),
        origin=Origin(xyz=(0.065, 0.098, 0.117)),
        material=body_shadow,
        name="right_slide_rail",
    )
    pedestal.visual(
        Box((0.10, 0.16, 0.015)),
        origin=Origin(xyz=(-0.01, 0.0, 0.1175)),
        material=body_shadow,
        name="rear_carriage_stop",
    )
    pedestal.visual(
        Box((0.042, 0.016, 0.024)),
        origin=Origin(xyz=(-0.105, -0.084, 0.245)),
        material=body_shadow,
        name="lock_button_housing",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.44, 0.30, 0.40)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.170, 0.210, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=body_shadow,
        name="tray_base",
    )
    bowl_carriage.visual(
        Box((0.120, 0.016, 0.038)),
        origin=Origin(xyz=(-0.025, -0.103, 0.019)),
        material=body_paint,
        name="left_carriage_side",
    )
    bowl_carriage.visual(
        Box((0.120, 0.016, 0.038)),
        origin=Origin(xyz=(-0.025, 0.103, 0.019)),
        material=body_paint,
        name="right_carriage_side",
    )
    bowl_carriage.visual(
        Box((0.040, 0.032, 0.020)),
        origin=Origin(xyz=(-0.060, 0.0, 0.010)),
        material=body_shadow,
        name="rear_cross_brace",
    )
    bowl_carriage.visual(
        Box((0.085, 0.020, 0.028)),
        origin=Origin(xyz=(0.015, -0.119, 0.033)),
        material=body_shadow,
        name="left_upper_cradle",
    )
    bowl_carriage.visual(
        Box((0.085, 0.020, 0.028)),
        origin=Origin(xyz=(0.015, 0.119, 0.033)),
        material=body_shadow,
        name="right_upper_cradle",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.13)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        _save_mesh(
            "mixing_bowl_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.055, 0.0),
                    (0.070, 0.006),
                    (0.090, 0.024),
                    (0.126, 0.080),
                    (0.158, 0.150),
                    (0.172, 0.188),
                ],
                [
                    (0.0, 0.004),
                    (0.060, 0.010),
                    (0.078, 0.025),
                    (0.114, 0.081),
                    (0.145, 0.151),
                    (0.160, 0.184),
                ],
                segments=56,
            ),
        ),
        material=bowl_steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.172, length=0.188),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.031, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_shadow,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.080, 0.048, 0.050)),
        origin=Origin(xyz=(0.040, 0.0, 0.040)),
        material=body_shadow,
        name="hinge_neck",
    )
    head.visual(
        _save_mesh(
            "head_shell",
            section_loft(
                [
                    _yz_section(0.09, 0.09, 0.022, 0.040, cz=0.045),
                    _yz_section(0.14, 0.16, 0.040, 0.150, cz=0.045),
                    _yz_section(0.16, 0.18, 0.046, 0.200, cz=0.038),
                    _yz_section(0.17, 0.17, 0.044, 0.310, cz=0.016),
                    _yz_section(0.13, 0.11, 0.030, 0.420, cz=-0.004),
                ]
            ),
        ),
        material=body_paint,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.047, length=0.084),
        origin=Origin(xyz=(0.235, 0.0, -0.047)),
        material=body_shadow,
        name="gearcase_boss",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.36, 0.18, 0.20)),
        mass=10.0,
        origin=Origin(xyz=(0.18, 0.0, 0.01)),
    )

    dough_hook = model.part("dough_hook")
    dough_hook.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=polished_steel,
        name="hook_shaft",
    )
    dough_hook.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=body_shadow,
        name="hook_collar",
    )
    dough_hook.visual(
        dough_hook_mesh,
        material=polished_steel,
        name="hook_body",
    )
    dough_hook.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.19),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_polymer,
        name="selector_dial",
    )
    speed_selector.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_shadow,
        name="selector_hub",
    )
    speed_selector.visual(
        Box((0.010, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black_polymer,
        name="selector_lever",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.040, 0.016, 0.032)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    head_lock_release = model.part("head_lock_release")
    head_lock_release.visual(
        Box((0.030, 0.010, 0.016)),
        origin=Origin(),
        material=black_polymer,
        name="lock_button",
    )
    head_lock_release.inertial = Inertial.from_geometry(
        Box((0.030, 0.010, 0.016)),
        mass=0.04,
    )

    model.articulation(
        "pedestal_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=bowl_carriage,
        origin=Origin(xyz=(0.100, 0.0, 0.124)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.12, lower=0.0, upper=0.065),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.045, 0.0, 0.018)),
    )
    model.articulation(
        "pedestal_to_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(-0.115, 0.0, 0.425)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(63.0),
        ),
    )
    model.articulation(
        "head_to_dough_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=dough_hook,
        origin=Origin(xyz=(0.260, 0.0, -0.089)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "pedestal_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=speed_selector,
        origin=Origin(xyz=(0.080, 0.136, 0.103)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "pedestal_to_head_lock_release",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=head_lock_release,
        origin=Origin(xyz=(-0.105, -0.097, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    bowl_carriage = object_model.get_part("bowl_carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    dough_hook = object_model.get_part("dough_hook")
    speed_selector = object_model.get_part("speed_selector")
    head_lock_release = object_model.get_part("head_lock_release")

    carriage_slide = object_model.get_articulation("pedestal_to_bowl_carriage")
    head_hinge = object_model.get_articulation("pedestal_to_head")
    selector_pivot = object_model.get_articulation("pedestal_to_speed_selector")
    lock_slide = object_model.get_articulation("pedestal_to_head_lock_release")

    slide_upper = carriage_slide.motion_limits.upper
    hinge_upper = head_hinge.motion_limits.upper
    selector_lower = selector_pivot.motion_limits.lower
    selector_upper = selector_pivot.motion_limits.upper
    lock_upper = lock_slide.motion_limits.upper

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_gap(
        bowl,
        bowl_carriage,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="tray_base",
        max_gap=0.001,
        max_penetration=0.0005,
        name="bowl seats on carriage tray",
    )
    ctx.expect_overlap(
        dough_hook,
        bowl,
        axes="xy",
        min_overlap=0.02,
        name="hook hangs over bowl opening at rest",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({carriage_slide: slide_upper}):
        bowl_extended = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage slides forward",
        bowl_rest is not None
        and bowl_extended is not None
        and bowl_extended[0] > bowl_rest[0] + 0.05,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    hook_rest = ctx.part_world_position(dough_hook)
    with ctx.pose({head_hinge: hinge_upper}):
        hook_raised = ctx.part_world_position(dough_hook)
        ctx.expect_gap(
            dough_hook,
            bowl,
            axis="z",
            min_gap=0.15,
            name="tilt head lifts hook clear of bowl",
        )
    ctx.check(
        "tilt head raises hook",
        hook_rest is not None
        and hook_raised is not None
        and hook_raised[2] > hook_rest[2] + 0.18,
        details=f"rest={hook_rest}, raised={hook_raised}",
    )

    with ctx.pose({selector_pivot: selector_lower}):
        lever_low = ctx.part_element_world_aabb(speed_selector, elem="selector_lever")
    with ctx.pose({selector_pivot: selector_upper}):
        lever_high = ctx.part_element_world_aabb(speed_selector, elem="selector_lever")
    ctx.check(
        "speed selector rotates through its range",
        lever_low is not None
        and lever_high is not None
        and lever_high[1][0] > lever_low[1][0] + 0.018,
        details=f"low={lever_low}, high={lever_high}",
    )

    lock_rest = ctx.part_world_position(head_lock_release)
    with ctx.pose({lock_slide: lock_upper}):
        lock_released = ctx.part_world_position(head_lock_release)
        ctx.expect_gap(
            pedestal,
            head_lock_release,
            axis="y",
            positive_elem="lock_button_housing",
            negative_elem="lock_button",
            min_gap=0.010,
            name="head lock release pulls out of housing",
        )
    ctx.check(
        "head lock release translates outward",
        lock_rest is not None
        and lock_released is not None
        and lock_released[1] < lock_rest[1] - 0.010,
        details=f"rest={lock_rest}, released={lock_released}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
