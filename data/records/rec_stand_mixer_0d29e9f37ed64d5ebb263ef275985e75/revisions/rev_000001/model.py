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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_stand_mixer")

    def yz_section(
        x: float,
        width_y: float,
        height_z: float,
        corner: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_center)
            for y, z in rounded_rect_profile(width_y, height_z, corner, corner_segments=8)
        ]

    body_teal = model.material("body_teal", rgba=(0.38, 0.80, 0.78, 1.0))
    cream = model.material("cream", rgba=(0.98, 0.94, 0.86, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.88, 0.90, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.32, 0.21, 0.048), 0.038),
            "mixer_base_plinth",
        ),
        origin=Origin(xyz=(0.020, 0.000, 0.019)),
        material=cream,
        name="plinth",
    )

    base_housing_mesh = section_loft(
        [
            yz_section(-0.120, 0.136, 0.182, 0.044, 0.122),
            yz_section(-0.060, 0.162, 0.226, 0.052, 0.152),
            yz_section(0.010, 0.148, 0.235, 0.050, 0.166),
            yz_section(0.060, 0.102, 0.150, 0.034, 0.132),
        ]
    )
    base.visual(
        mesh_from_geometry(base_housing_mesh, "mixer_base_housing"),
        material=body_teal,
        name="housing",
    )
    base.visual(
        Box((0.170, 0.126, 0.014)),
        origin=Origin(xyz=(0.116, 0.000, 0.045)),
        material=cream,
        name="track_deck",
    )
    base.visual(
        Box((0.084, 0.122, 0.018)),
        origin=Origin(xyz=(0.040, 0.000, 0.047)),
        material=cream,
        name="track_bridge",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(-0.084, 0.066, 0.312), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="hinge_lug_left",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(-0.084, -0.066, 0.312), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="hinge_lug_right",
    )
    base.visual(
        Box((0.064, 0.018, 0.036)),
        origin=Origin(xyz=(-0.076, 0.072, 0.276)),
        material=body_teal,
        name="hinge_tower_left",
    )
    base.visual(
        Box((0.064, 0.018, 0.036)),
        origin=Origin(xyz=(-0.076, -0.072, 0.276)),
        material=body_teal,
        name="hinge_tower_right",
    )
    base.visual(
        Box((0.045, 0.014, 0.018)),
        origin=Origin(xyz=(-0.056, 0.084, 0.247)),
        material=cream,
        name="lock_track",
    )
    base.visual(
        Box((0.024, 0.020, 0.030)),
        origin=Origin(xyz=(-0.010, 0.074, 0.205)),
        material=cream,
        name="speed_mount",
    )
    base.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.108, 0.072, 0.048)),
        material=rubber,
        name="rear_foot_left",
    )
    base.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.108, -0.072, 0.048)),
        material=rubber,
        name="rear_foot_right",
    )
    base.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.140, 0.072, 0.048)),
        material=rubber,
        name="front_foot_left",
    )
    base.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.140, -0.072, 0.048)),
        material=rubber,
        name="front_foot_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.230, 0.360)),
        mass=8.2,
        origin=Origin(xyz=(0.010, 0.000, 0.180)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.102, 0.118, 0.015)),
        origin=Origin(xyz=(0.051, 0.000, 0.0075)),
        material=cream,
        name="carriage_shoe",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.074, 0.000, 0.024)),
        material=cream,
        name="bowl_seat",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.027, length=0.032),
        origin=Origin(xyz=(0.074, 0.000, 0.040)),
        material=cream,
        name="seat_pedestal",
    )

    bowl_outer_profile = [
        (0.020, 0.000),
        (0.047, 0.010),
        (0.076, 0.040),
        (0.086, 0.078),
        (0.089, 0.102),
        (0.093, 0.108),
    ]
    bowl_inner_profile = [
        (0.000, 0.004),
        (0.042, 0.014),
        (0.070, 0.042),
        (0.080, 0.080),
        (0.085, 0.104),
    ]
    bowl_carriage.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                bowl_outer_profile,
                bowl_inner_profile,
                segments=64,
            ),
            "mixer_bowl_shell",
        ),
        origin=Origin(xyz=(0.074, 0.000, 0.050)),
        material=steel,
        name="bowl_shell",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.200, 0.190, 0.180)),
        mass=1.4,
        origin=Origin(xyz=(0.074, 0.000, 0.090)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.080, 0.000, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.12,
            lower=0.0,
            upper=0.040,
        ),
    )

    head = model.part("head")
    head_shell_mesh = section_loft(
        [
            yz_section(0.000, 0.090, 0.125, 0.028, 0.035),
            yz_section(0.080, 0.148, 0.176, 0.046, 0.038),
            yz_section(0.170, 0.142, 0.148, 0.040, 0.026),
            yz_section(0.235, 0.104, 0.104, 0.030, 0.006),
            yz_section(0.268, 0.070, 0.074, 0.020, -0.004),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell_mesh, "mixer_head_shell"),
        origin=Origin(xyz=(0.000, 0.000, 0.065)),
        material=body_teal,
        name="head_shell",
    )
    head.visual(
        Box((0.054, 0.086, 0.056)),
        origin=Origin(xyz=(0.014, 0.000, 0.034)),
        material=cream,
        name="rear_boss",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(-0.002, 0.037, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="hinge_knuckle_left",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(-0.002, -0.037, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="hinge_knuckle_right",
    )
    head.visual(
        Cylinder(radius=0.038, length=0.030),
        origin=Origin(xyz=(0.230, 0.000, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="nose_ring",
    )
    head.visual(
        Box((0.080, 0.096, 0.042)),
        origin=Origin(xyz=(0.160, 0.000, 0.004)),
        material=cream,
        name="gearbox_belly",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.290, 0.170, 0.200)),
        mass=4.6,
        origin=Origin(xyz=(0.140, 0.000, 0.030)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.084, 0.000, 0.312)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    tool = model.part("tool")
    tool.visual(
        Cylinder(radius=0.0105, length=0.060),
        origin=Origin(xyz=(0.000, 0.000, -0.030)),
        material=steel,
        name="drive_shaft",
    )
    tool.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, -0.068)),
        material=steel,
        name="ferrule",
    )
    dough_hook_mesh = tube_from_spline_points(
        [
            (0.000, 0.000, -0.082),
            (0.008, 0.000, -0.104),
            (0.017, 0.000, -0.128),
            (0.022, 0.000, -0.153),
            (0.017, 0.000, -0.170),
            (0.004, 0.000, -0.181),
            (-0.010, 0.000, -0.170),
            (-0.010, 0.000, -0.138),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    tool.visual(
        mesh_from_geometry(dough_hook_mesh, "mixer_dough_hook"),
        material=steel,
        name="hook_body",
    )
    tool.inertial = Inertial.from_geometry(
        Box((0.070, 0.030, 0.220)),
        mass=0.35,
        origin=Origin(xyz=(0.002, 0.000, -0.102)),
    )

    model.articulation(
        "head_to_tool",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool,
        origin=Origin(xyz=(0.230, 0.000, -0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=16.0),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="speed_pivot",
    )
    speed_control.visual(
        Box((0.028, 0.010, 0.010)),
        origin=Origin(xyz=(0.016, 0.000, 0.000)),
        material=charcoal,
        name="speed_arm",
    )
    speed_control.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.032, 0.000, 0.000)),
        material=cream,
        name="speed_tip",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.044, 0.014, 0.018)),
        mass=0.08,
        origin=Origin(xyz=(0.016, 0.000, 0.000)),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.010, 0.088, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=-0.40,
            upper=0.40,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.022, 0.012, 0.010)),
        origin=Origin(xyz=(0.011, 0.000, 0.005)),
        material=charcoal,
        name="lock_slider",
    )
    head_lock.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.017, 0.000, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="lock_grip",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.032, 0.020, 0.024)),
        mass=0.06,
        origin=Origin(xyz=(0.016, 0.000, 0.010)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.074, 0.098, 0.242)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    tool = object_model.get_part("tool")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    head_lock_joint = object_model.get_articulation("base_to_head_lock")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        bowl_carriage,
        base,
        axis="z",
        positive_elem="carriage_shoe",
        negative_elem="track_deck",
        max_gap=0.002,
        max_penetration=0.0,
        name="carriage shoe rests on the deck",
    )
    ctx.expect_overlap(
        bowl_carriage,
        base,
        axes="x",
        elem_a="carriage_shoe",
        elem_b="track_deck",
        min_overlap=0.090,
        name="carriage stays retained on the track at rest",
    )
    ctx.expect_overlap(
        tool,
        bowl_carriage,
        axes="xy",
        elem_a="hook_body",
        elem_b="bowl_shell",
        min_overlap=0.015,
        name="dough hook hangs within the bowl footprint",
    )

    carriage_rest = ctx.part_world_position(bowl_carriage)
    lock_rest = ctx.part_world_position(head_lock)
    speed_low_tip = None
    speed_high_tip = None
    nose_rest = aabb_center(ctx.part_element_world_aabb(head, elem="nose_ring"))
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        ctx.expect_overlap(
            bowl_carriage,
            base,
            axes="x",
            elem_a="carriage_shoe",
            elem_b="track_deck",
            min_overlap=0.080,
            name="carriage keeps insertion when slid forward",
        )
        carriage_extended = ctx.part_world_position(bowl_carriage)
    ctx.check(
        "bowl carriage slides forward",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.030,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        nose_open = aabb_center(ctx.part_element_world_aabb(head, elem="nose_ring"))
    ctx.check(
        "head nose lifts when opened",
        nose_rest is not None
        and nose_open is not None
        and nose_open[2] > nose_rest[2] + 0.060,
        details=f"closed={nose_rest}, open={nose_open}",
    )

    with ctx.pose({speed_joint: speed_joint.motion_limits.lower}):
        speed_low_tip = aabb_center(ctx.part_element_world_aabb(speed_control, elem="speed_tip"))
    with ctx.pose({speed_joint: speed_joint.motion_limits.upper}):
        speed_high_tip = aabb_center(ctx.part_element_world_aabb(speed_control, elem="speed_tip"))
    ctx.check(
        "speed lever swings through a visible arc",
        speed_low_tip is not None
        and speed_high_tip is not None
        and speed_high_tip[2] > speed_low_tip[2] + 0.018,
        details=f"low={speed_low_tip}, high={speed_high_tip}",
    )

    with ctx.pose({head_lock_joint: head_lock_joint.motion_limits.upper}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slider extends forward",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.010,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
