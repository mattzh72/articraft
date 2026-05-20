from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
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
    model = ArticulatedObject(name="commercial_bakery_stand_mixer")

    machine_white = model.material("machine_white", rgba=(0.88, 0.89, 0.87, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    stainless = model.material("stainless", rgba=(0.83, 0.85, 0.88, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.42, 0.44, 0.47, 1.0))

    def yz_section(
        x: float,
        width_y: float,
        height_z: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_center)
            for z, y in rounded_rect_profile(height_z, width_y, radius, corner_segments=8)
        ]

    model_base = model.part("base")

    model_base.visual(
        Box((0.44, 0.30, 0.075)),
        origin=Origin(xyz=(0.015, 0.0, 0.0375)),
        material=machine_white,
        name="pedestal_plinth",
    )
    model_base.visual(
        Box((0.23, 0.24, 0.030)),
        origin=Origin(xyz=(0.105, 0.0, 0.060)),
        material=machine_white,
        name="front_mass",
    )

    pedestal_shell = section_loft(
        [
            yz_section(-0.135, 0.22, 0.28, 0.04, 0.205),
            yz_section(-0.075, 0.20, 0.36, 0.05, 0.275),
            yz_section(-0.030, 0.16, 0.16, 0.040, 0.360),
        ]
    )
    model_base.visual(
        mesh_from_geometry(pedestal_shell, "mixer_pedestal_shell"),
        material=machine_white,
        name="pedestal_shell",
    )

    model_base.visual(
        Box((0.060, 0.060, 0.090)),
        origin=Origin(xyz=(-0.072, -0.090, 0.485)),
        material=machine_white,
        name="left_hinge_cheek",
    )
    model_base.visual(
        Box((0.060, 0.060, 0.090)),
        origin=Origin(xyz=(-0.072, 0.090, 0.485)),
        material=machine_white,
        name="right_hinge_cheek",
    )

    model_base.visual(
        Box((0.145, 0.024, 0.050)),
        origin=Origin(xyz=(0.0825, -0.055, 0.099)),
        material=machine_white,
        name="left_slide_rail",
    )
    model_base.visual(
        Box((0.145, 0.024, 0.050)),
        origin=Origin(xyz=(0.0825, 0.055, 0.099)),
        material=machine_white,
        name="right_slide_rail",
    )
    model_base.visual(
        Box((0.040, 0.126, 0.016)),
        origin=Origin(xyz=(0.028, 0.0, 0.130)),
        material=machine_white,
        name="slide_bridge",
    )
    model_base.visual(
        Box((0.024, 0.115, 0.030)),
        origin=Origin(xyz=(0.006, 0.0, 0.095)),
        material=dark_trim,
        name="carriage_stop",
    )
    model_base.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(-0.060, 0.092, 0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="selector_mount",
    )
    model_base.visual(
        Box((0.072, 0.016, 0.028)),
        origin=Origin(xyz=(-0.085, -0.104, 0.426)),
        material=steel_dark,
        name="lock_release_track",
    )

    model_base.inertial = Inertial.from_geometry(
        Box((0.44, 0.30, 0.56)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.125, 0.086, 0.024)),
        origin=Origin(xyz=(0.0625, 0.0, 0.012)),
        material=steel_dark,
        name="slide_tongue",
    )
    bowl_carriage.visual(
        Box((0.070, 0.080, 0.048)),
        origin=Origin(xyz=(0.088, 0.0, 0.032)),
        material=steel_dark,
        name="saddle_pillar",
    )
    bowl_carriage.visual(
        Box((0.135, 0.190, 0.018)),
        origin=Origin(xyz=(0.118, 0.0, 0.061)),
        material=steel_dark,
        name="bowl_saddle",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.19, 0.08)),
        mass=2.6,
        origin=Origin(xyz=(0.090, 0.0, 0.040)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=model_base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.020, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.08,
            lower=0.0,
            upper=0.050,
        ),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.040, 0.000),
            (0.066, 0.015),
            (0.112, 0.056),
            (0.145, 0.140),
            (0.154, 0.174),
            (0.164, 0.188),
            (0.171, 0.194),
        ],
        [
            (0.000, 0.006),
            (0.050, 0.018),
            (0.102, 0.056),
            (0.131, 0.138),
            (0.139, 0.171),
            (0.149, 0.184),
        ],
        segments=72,
        end_cap="round",
        lip_samples=10,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "mixer_bowl_shell"),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.052, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=steel_dark,
        name="bowl_foot",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.171, length=0.194),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.125, 0.0, 0.070)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            yz_section(0.045, 0.155, 0.130, 0.035, -0.010),
            yz_section(0.145, 0.215, 0.170, 0.052, -0.008),
            yz_section(0.250, 0.205, 0.145, 0.046, -0.020),
            yz_section(0.335, 0.150, 0.110, 0.036, -0.035),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "mixer_head_shell"),
        material=machine_white,
        name="head_shell",
    )
    head.visual(
        Box((0.055, 0.120, 0.060)),
        origin=Origin(xyz=(0.032, 0.0, -0.006)),
        material=machine_white,
        name="head_knuckle",
    )
    head.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.310, 0.0, -0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machine_white,
        name="head_nose",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.220, 0.0, -0.090)),
        material=machine_white,
        name="planetary_collar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.20)),
        mass=9.5,
        origin=Origin(xyz=(0.180, 0.0, -0.050)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=model_base,
        child=head,
        origin=Origin(xyz=(-0.075, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="selector_hub",
    )
    speed_selector.visual(
        Box((0.060, 0.012, 0.012)),
        origin=Origin(xyz=(0.030, 0.022, 0.0)),
        material=dark_trim,
        name="selector_arm",
    )
    speed_selector.visual(
        Box((0.018, 0.020, 0.018)),
        origin=Origin(xyz=(0.060, 0.022, 0.0)),
        material=steel_dark,
        name="selector_grip",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.078, 0.028, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(0.038, 0.016, 0.0)),
    )

    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=model_base,
        child=speed_selector,
        origin=Origin(xyz=(-0.060, 0.100, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(36.0),
        ),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=steel_dark,
        name="planetary_hub",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=stainless,
        name="spindle_shaft",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.060),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    model.articulation(
        "head_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=spindle,
        origin=Origin(xyz=(0.220, 0.0, -0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )

    lock_release = model.part("lock_release")
    lock_release.visual(
        Box((0.028, 0.012, 0.022)),
        origin=Origin(xyz=(0.014, -0.006, 0.0)),
        material=dark_trim,
        name="release_shoe",
    )
    lock_release.visual(
        Box((0.018, 0.016, 0.018)),
        origin=Origin(xyz=(0.018, -0.020, 0.020)),
        material=steel_dark,
        name="release_tab",
    )
    lock_release.inertial = Inertial.from_geometry(
        Box((0.030, 0.024, 0.036)),
        mass=0.05,
        origin=Origin(xyz=(0.015, -0.012, 0.010)),
    )

    model.articulation(
        "base_to_lock_release",
        ArticulationType.PRISMATIC,
        parent=model_base,
        child=lock_release,
        origin=Origin(xyz=(-0.121, -0.112, 0.426)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.05,
            lower=0.0,
            upper=0.014,
        ),
    )

    dough_hook = model.part("dough_hook")
    hook_geom = CylinderGeometry(radius=0.009, height=0.022).translate(0.0, 0.0, -0.011)
    hook_geom.merge(
        tube_from_spline_points(
            [
                (-0.004, -0.010, -0.014),
                (0.010, -0.006, -0.032),
                (0.018, 0.002, -0.056),
                (0.018, 0.014, -0.086),
                (0.008, 0.021, -0.114),
                (-0.006, 0.018, -0.138),
                (-0.014, 0.006, -0.154),
                (-0.012, -0.004, -0.162),
                (-0.005, -0.010, -0.168),
            ],
            radius=0.0072,
            samples_per_segment=18,
            radial_segments=18,
        )
    )
    hook_geom.merge(
        ConeGeometry(radius=0.0072, height=0.012, radial_segments=18, closed=True).translate(
            -0.005, -0.010, -0.174
        )
    )
    dough_hook.visual(
        mesh_from_geometry(hook_geom, "spiral_dough_hook"),
        material=stainless,
        name="hook_body",
    )
    dough_hook.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.180),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    model.articulation(
        "spindle_to_dough_hook",
        ArticulationType.FIXED,
        parent=spindle,
        child=dough_hook,
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    dough_hook = object_model.get_part("dough_hook")
    speed_selector = object_model.get_part("speed_selector")
    lock_release = object_model.get_part("lock_release")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    spindle_spin = object_model.get_articulation("head_to_spindle")
    selector_pivot = object_model.get_articulation("base_to_speed_selector")
    release_slide = object_model.get_articulation("base_to_lock_release")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_origin_distance(
        dough_hook,
        bowl,
        axes="xy",
        max_dist=0.012,
        name="dough hook hangs centered over the bowl at rest",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        extended_bowl_pos = ctx.part_world_position(bowl)
    ctx.check(
        "bowl carriage slides forward",
        rest_bowl_pos is not None
        and extended_bowl_pos is not None
        and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.035,
        details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
    )

    rest_nose_center = aabb_center(ctx.part_element_world_aabb(head, elem="head_nose"))
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        open_nose_center = aabb_center(ctx.part_element_world_aabb(head, elem="head_nose"))
        ctx.expect_gap(
            dough_hook,
            bowl,
            axis="z",
            min_gap=0.030,
            name="open head lifts the dough hook clear of the bowl",
        )
    ctx.check(
        "head opens upward about the rear hinge",
        rest_nose_center is not None
        and open_nose_center is not None
        and open_nose_center[2] > rest_nose_center[2] + 0.090,
        details=f"rest={rest_nose_center}, open={open_nose_center}",
    )

    ctx.check(
        "spindle articulation is continuous",
        spindle_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spindle_spin.articulation_type}",
    )

    rest_selector_center = aabb_center(
        ctx.part_element_world_aabb(speed_selector, elem="selector_grip")
    )
    with ctx.pose({selector_pivot: selector_pivot.motion_limits.upper}):
        high_selector_center = aabb_center(
            ctx.part_element_world_aabb(speed_selector, elem="selector_grip")
        )
    ctx.check(
        "speed selector rotates through its setting arc",
        rest_selector_center is not None
        and high_selector_center is not None
        and high_selector_center[2] > rest_selector_center[2] + 0.020,
        details=f"rest={rest_selector_center}, high={high_selector_center}",
    )

    rest_release_pos = ctx.part_world_position(lock_release)
    with ctx.pose({release_slide: release_slide.motion_limits.upper}):
        actuated_release_pos = ctx.part_world_position(lock_release)
    ctx.check(
        "lock release slides forward on the base",
        rest_release_pos is not None
        and actuated_release_pos is not None
        and actuated_release_pos[0] > rest_release_pos[0] + 0.010,
        details=f"rest={rest_release_pos}, actuated={actuated_release_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
