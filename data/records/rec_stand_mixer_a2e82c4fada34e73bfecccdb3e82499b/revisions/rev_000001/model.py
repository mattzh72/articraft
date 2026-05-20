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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    section_loft,
)


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z) for y, z in rounded_rect_profile(width, height, radius)]


def _build_flat_beater_mesh():
    outer = sample_catmull_rom_spline_2d(
        [
            (0.016, 0.024),
            (0.040, -0.002),
            (0.050, -0.035),
            (0.044, -0.078),
            (0.026, -0.108),
            (0.000, -0.118),
            (-0.026, -0.108),
            (-0.044, -0.078),
            (-0.050, -0.035),
            (-0.040, -0.002),
            (-0.016, 0.024),
        ],
        samples_per_segment=10,
        closed=True,
    )
    inner = sample_catmull_rom_spline_2d(
        [
            (0.010, 0.002),
            (0.022, -0.020),
            (0.025, -0.062),
            (0.014, -0.093),
            (0.000, -0.102),
            (-0.014, -0.093),
            (-0.025, -0.062),
            (-0.022, -0.020),
            (-0.010, 0.002),
        ],
        samples_per_segment=10,
        closed=True,
    )

    paddle = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        0.006,
        center=True,
    )
    paddle.rotate_x(math.pi / 2.0).translate(0.0, 0.0, -0.046)

    shaft = CylinderGeometry(radius=0.0065, height=0.052).translate(0.0, 0.0, -0.026)
    ferrule = CylinderGeometry(radius=0.010, height=0.026).translate(0.0, 0.0, -0.045)
    shaft.merge(ferrule)
    shaft.merge(paddle)
    return mesh_from_geometry(shaft, "flat_beater")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_premium_stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.86, 0.84, 0.80, 1.0))
    bowl_steel = model.material("bowl_steel", rgba=(0.88, 0.90, 0.92, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.20, 1.0))
    rail_trim = model.material("rail_trim", rgba=(0.34, 0.35, 0.37, 1.0))

    base = model.part("base")

    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.340, 0.220, 0.050), 0.045),
        "mixer_base_plate",
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=body_paint,
        name="base_plate",
    )

    front_deck = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.165, 0.155, 0.028), 0.018),
        "mixer_front_deck",
    )
    base.visual(
        front_deck,
        origin=Origin(xyz=(0.074, 0.0, 0.054)),
        material=body_paint,
        name="front_deck",
    )
    base.visual(
        Box((0.124, 0.022, 0.014)),
        origin=Origin(xyz=(0.073, 0.043, 0.063)),
        material=rail_trim,
        name="right_slide_rail",
    )
    base.visual(
        Box((0.124, 0.022, 0.014)),
        origin=Origin(xyz=(0.073, -0.043, 0.063)),
        material=rail_trim,
        name="left_slide_rail",
    )

    pedestal_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.104, 0.185, 0.030, -0.108, z_center=0.135),
                _yz_section(0.095, 0.240, 0.030, -0.078, z_center=0.170),
                _yz_section(0.082, 0.258, 0.026, -0.052, z_center=0.198),
            ]
        ),
        "mixer_pedestal",
    )
    base.visual(
        pedestal_mesh,
        material=body_paint,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.050, 0.074, 0.020)),
        origin=Origin(xyz=(-0.081, 0.0, 0.291)),
        material=body_paint,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.038, 0.010, 0.040)),
        origin=Origin(xyz=(-0.083, 0.047, 0.309)),
        material=body_paint,
        name="right_hinge_cheek",
    )
    base.visual(
        Box((0.038, 0.010, 0.040)),
        origin=Origin(xyz=(-0.083, -0.047, 0.309)),
        material=body_paint,
        name="left_hinge_cheek",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.220, 0.360)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.110, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_trim,
        name="carriage_block",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.068, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=dark_trim,
        name="bowl_platform",
    )
    bowl_carriage.visual(
        Box((0.084, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.046, 0.024)),
        material=rail_trim,
        name="right_runner",
    )
    bowl_carriage.visual(
        Box((0.084, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.046, 0.024)),
        material=rail_trim,
        name="left_runner",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.122, 0.112, 0.034)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.072, 0.0, 0.057)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.08,
            lower=0.0,
            upper=0.026,
        ),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Box((0.016, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        material=dark_trim,
        name="knob_mount",
    )
    speed_knob.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="knob_body",
    )
    speed_knob.visual(
        Box((0.010, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.014, 0.014)),
        material=dark_trim,
        name="knob_pointer",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Box((0.026, 0.022, 0.028)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.011, 0.006)),
    )

    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.020, 0.0775, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_trim,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.020, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=rail_trim,
        name="lock_thumb",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.020, 0.016, 0.014)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.083, 0.047, 0.329)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.006,
        ),
    )

    bowl = model.part("bowl")
    bowl_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.030, 0.000),
                (0.053, 0.010),
                (0.082, 0.040),
                (0.102, 0.092),
                (0.110, 0.145),
                (0.113, 0.154),
                (0.118, 0.158),
            ],
            [
                (0.000, 0.006),
                (0.049, 0.015),
                (0.078, 0.044),
                (0.096, 0.092),
                (0.104, 0.148),
                (0.110, 0.154),
            ],
            segments=56,
            end_cap="round",
            lip_samples=10,
        ),
        "mixer_bowl",
    )
    bowl.visual(
        bowl_shell,
        material=bowl_steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.158),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    head = model.part("head")
    head_shell = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.092, 0.102, 0.028, 0.034, z_center=0.024),
                _yz_section(0.145, 0.136, 0.042, 0.110, z_center=0.012),
                _yz_section(0.090, 0.094, 0.026, 0.228, z_center=-0.006),
            ]
        ),
        "mixer_head_shell",
    )
    head.visual(
        head_shell,
        material=body_paint,
        name="head_shell",
    )
    head.visual(
        Box((0.036, 0.062, 0.044)),
        origin=Origin(xyz=(0.018, 0.0, 0.030)),
        material=body_paint,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.084),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_paint,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.048),
        origin=Origin(xyz=(0.156, 0.0, -0.036)),
        material=dark_trim,
        name="drive_nose",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.250, 0.150, 0.150)),
        mass=4.8,
        origin=Origin(xyz=(0.120, 0.0, -0.002)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.085, 0.0, 0.324)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    beater = model.part("beater")
    beater.visual(
        _build_flat_beater_mesh(),
        material=dark_trim,
        name="flat_beater",
    )
    beater.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.164),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.156, 0.0, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl_carriage = object_model.get_part("bowl_carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    speed_knob = object_model.get_part("speed_knob")
    head_lock = object_model.get_part("head_lock")
    carriage_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    speed_control = object_model.get_articulation("base_to_speed_knob")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_overlap(
        bowl,
        bowl_carriage,
        axes="xy",
        min_overlap=0.100,
        name="bowl stays centered on the carriage",
    )
    ctx.expect_origin_distance(
        beater,
        bowl,
        axes="xy",
        max_dist=0.012,
        name="beater is centered over the bowl",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.018,
        max_gap=0.110,
        name="closed head sits just above the bowl rim",
    )

    carriage_rest = ctx.part_world_position(bowl_carriage)
    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        carriage_extended = ctx.part_world_position(bowl_carriage)
    ctx.check(
        "bowl carriage slides forward",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.020,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    beater_closed = ctx.part_world_position(beater)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        beater_open = ctx.part_world_position(beater)
    ctx.check(
        "tilt head raises upward",
        beater_closed is not None
        and beater_open is not None
        and beater_open[2] > beater_closed[2] + 0.055,
        details=f"closed={beater_closed}, open={beater_open}",
    )

    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({lock_slide: lock_slide.motion_limits.upper}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slider moves upward in its short slot",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[2] > lock_rest[2] + 0.004,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    knob_rest = aabb_center(ctx.part_element_world_aabb(speed_knob, elem="knob_pointer"))
    with ctx.pose({speed_control: speed_control.motion_limits.upper}):
        knob_fast = aabb_center(ctx.part_element_world_aabb(speed_knob, elem="knob_pointer"))
    ctx.check(
        "speed knob rotates through its range",
        knob_rest is not None
        and knob_fast is not None
        and knob_fast[0] > knob_rest[0] + 0.010,
        details=f"rest={knob_rest}, fast={knob_fast}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
