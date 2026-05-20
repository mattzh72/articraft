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


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x, center_y + y, z)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for z, y in rounded_rect_profile(height, width, radius)
    ]


def _build_bowl_mesh():
    outer = [
        (0.018, 0.000),
        (0.050, 0.008),
        (0.084, 0.032),
        (0.101, 0.072),
        (0.107, 0.102),
    ]
    inner = [
        (0.000, 0.003),
        (0.042, 0.012),
        (0.076, 0.034),
        (0.093, 0.072),
        (0.099, 0.097),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_whisk_loop(angle: float):
    c = math.cos(angle)
    s = math.sin(angle)
    return tube_from_spline_points(
        [
            (0.0095 * c, 0.0095 * s, -0.045),
            (0.019 * c, 0.019 * s, -0.057),
            (0.029 * c, 0.029 * s, -0.073),
            (0.038 * c, 0.038 * s, -0.089),
            (0.0, 0.0, -0.101),
            (-0.038 * c, -0.038 * s, -0.089),
            (-0.029 * c, -0.029 * s, -0.073),
            (-0.019 * c, -0.019 * s, -0.057),
            (-0.0095 * c, -0.0095 * s, -0.045),
        ],
        radius=0.0015,
        samples_per_segment=14,
        radial_segments=14,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer")

    cream = model.material("cream", rgba=(0.92, 0.87, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _xy_section(0.155, 0.220, 0.050, 0.028, center_x=-0.040),
                    _xy_section(0.138, 0.176, 0.042, 0.100, center_x=-0.055),
                    _xy_section(0.118, 0.142, 0.036, 0.172, center_x=-0.048),
                    _xy_section(0.095, 0.108, 0.028, 0.220, center_x=-0.038),
                ]
            ),
            "base_shell",
        ),
        material=cream,
        name="base_shell",
    )
    base.visual(
        Box((0.270, 0.340, 0.028)),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=cream,
        name="pedestal_foot",
    )
    base.visual(
        Box((0.075, 0.095, 0.024)),
        origin=Origin(xyz=(0.012, 0.000, 0.040)),
        material=cream,
        name="front_riser",
    )
    base.visual(
        Box((0.100, 0.078, 0.014)),
        origin=Origin(xyz=(0.050, 0.000, 0.048)),
        material=cream,
        name="carriage_track",
    )
    base.visual(
        Box((0.056, 0.092, 0.054)),
        origin=Origin(xyz=(-0.086, 0.000, 0.247)),
        material=cream,
        name="rear_column",
    )
    base.visual(
        Box((0.040, 0.082, 0.010)),
        origin=Origin(xyz=(-0.104, 0.000, 0.279)),
        material=cream,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.024, 0.032, 0.006)),
        origin=Origin(xyz=(-0.086, -0.056, 0.271)),
        material=cream,
        name="button_pad",
    )
    base.visual(
        Box((0.020, 0.030, 0.024)),
        origin=Origin(xyz=(-0.050, 0.073, 0.193)),
        material=cream,
        name="lever_mount",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.086, 0.072, 0.016)),
        origin=Origin(xyz=(0.043, 0.000, 0.008)),
        material=cream,
        name="carriage_saddle",
    )
    carriage.visual(
        Box((0.034, 0.052, 0.046)),
        origin=Origin(xyz=(0.060, 0.000, 0.037)),
        material=cream,
        name="carriage_upright",
    )
    carriage.visual(
        Cylinder(radius=0.056, length=0.010),
        origin=Origin(xyz=(0.088, 0.000, 0.065)),
        material=cream,
        name="carriage_plate",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_build_bowl_mesh(), "mixing_bowl_shell"),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=steel,
        name="bowl_foot",
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(0.102, 0.110, 0.028, 0.030, z_center=0.032),
                    _yz_section(0.168, 0.176, 0.050, 0.135, z_center=0.034),
                    _yz_section(0.154, 0.156, 0.046, 0.228, z_center=0.010),
                    _yz_section(0.090, 0.104, 0.028, 0.322, z_center=-0.006),
                ]
            ),
            "head_shell",
        ),
        material=cream,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.218, 0.000, -0.060)),
        material=cream,
        name="drive_housing",
    )
    head.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(0.102, 0.074, 0.020, 0.194, z_center=-0.020),
                    _yz_section(0.084, 0.056, 0.018, 0.248, z_center=-0.028),
                    _yz_section(0.060, 0.040, 0.014, 0.292, z_center=-0.018),
                ]
            ),
            "front_neck_shell",
        ),
        material=cream,
        name="front_neck",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.296, 0.000, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="nose_cap",
    )
    head.visual(
        Box((0.070, 0.080, 0.038)),
        origin=Origin(xyz=(0.000, 0.000, -0.002)),
        material=cream,
        name="rear_hinge_knuckle",
    )

    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.0048, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, -0.012)),
        material=steel,
        name="whisk_shaft",
    )
    whisk.visual(
        Cylinder(radius=0.0082, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, -0.034)),
        material=steel,
        name="whisk_upper_hub",
    )
    whisk.visual(
        Cylinder(radius=0.0110, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.049)),
        material=steel,
        name="whisk_lower_hub",
    )
    whisk.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.101)),
        material=steel,
        name="whisk_tip",
    )
    for index in range(6):
        whisk.visual(
            mesh_from_geometry(_build_whisk_loop(index * math.pi / 6.0), f"whisk_loop_{index}"),
            material=steel,
            name=f"whisk_loop_{index}",
        )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="lever_pivot",
    )
    speed_lever.visual(
        Box((0.014, 0.044, 0.008)),
        origin=Origin(xyz=(0.000, 0.024, 0.003)),
        material=dark_trim,
        name="lever_arm",
    )
    speed_lever.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.000, 0.047, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="lever_knob",
    )

    lock_button = model.part("lock_button")
    lock_button.visual(
        Cylinder(radius=0.0085, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=dark_trim,
        name="button_plunger",
    )
    lock_button.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=dark_trim,
        name="button_cap",
    )
    lock_button.visual(
        Box((0.016, 0.016, 0.018)),
        origin=Origin(xyz=(0.000, 0.008, 0.011)),
        material=dark_trim,
        name="button_latch",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.030, 0.000, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.095, 0.000, 0.070)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.090, 0.000, 0.305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.218, 0.000, -0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=20.0,
        ),
    )
    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.030, 0.073, 0.193)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "base_to_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.086, -0.056, 0.281)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0045,
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
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_lever = object_model.get_part("speed_lever")
    lock_button = object_model.get_part("lock_button")

    bowl_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    whisk_spin = object_model.get_articulation("head_to_whisk")
    lever_pivot = object_model.get_articulation("base_to_speed_lever")
    button_slide = object_model.get_articulation("base_to_lock_button")

    ctx.check(
        "all prompt-critical parts exist",
        all(
            part is not None
            for part in (base, carriage, bowl, head, whisk, speed_lever, lock_button)
        ),
    )
    ctx.check(
        "articulation types match prompt",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and whisk_spin.articulation_type == ArticulationType.CONTINUOUS
        and lever_pivot.articulation_type == ArticulationType.REVOLUTE
        and button_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.expect_contact(
        bowl,
        carriage,
        elem_a="bowl_foot",
        elem_b="carriage_plate",
        contact_tol=0.001,
        name="bowl seats on the carriage plate",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="carriage_saddle",
        elem_b="carriage_track",
        contact_tol=0.001,
        name="carriage rests on the base track at home",
    )
    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        margin=0.018,
        name="whisk stays centered over the bowl",
    )
    ctx.expect_contact(
        whisk,
        head,
        elem_a="whisk_shaft",
        elem_b="drive_housing",
        contact_tol=0.001,
        name="whisk shaft seats in the drive housing",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="rear_hinge_knuckle",
        elem_b="hinge_bridge",
        contact_tol=0.001,
        name="tilt head is supported on the rear hinge bridge",
    )
    ctx.expect_contact(
        lock_button,
        head,
        elem_a="button_latch",
        elem_b="rear_hinge_knuckle",
        contact_tol=0.001,
        name="head lock button reaches the head latch knuckle",
    )

    carriage_home = ctx.part_world_position(carriage)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            base,
            elem_a="carriage_saddle",
            elem_b="carriage_track",
            contact_tol=0.001,
            name="carriage remains supported when extended",
        )
    ctx.check(
        "bowl carriage slides forward",
        carriage_home is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_home[0] + 0.015,
        details=f"home={carriage_home}, extended={carriage_extended}",
    )

    whisk_home = ctx.part_world_position(whisk)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        whisk_lifted = ctx.part_world_position(whisk)
    ctx.check(
        "tilt head raises the whisk",
        whisk_home is not None
        and whisk_lifted is not None
        and whisk_lifted[2] > whisk_home[2] + 0.10,
        details=f"home={whisk_home}, lifted={whisk_lifted}",
    )

    lever_home = ctx.part_element_world_aabb(speed_lever, elem="lever_knob")
    with ctx.pose({lever_pivot: lever_pivot.motion_limits.upper}):
        lever_raised = ctx.part_element_world_aabb(speed_lever, elem="lever_knob")
    ctx.check(
        "speed lever pivots upward",
        lever_home is not None
        and lever_raised is not None
        and lever_raised[1][2] > lever_home[1][2] + 0.010,
        details=f"home={lever_home}, raised={lever_raised}",
    )

    button_home = ctx.part_world_position(lock_button)
    with ctx.pose({button_slide: button_slide.motion_limits.upper}):
        button_pressed = ctx.part_world_position(lock_button)
    ctx.check(
        "head lock button presses downward",
        button_home is not None
        and button_pressed is not None
        and button_pressed[2] < button_home[2] - 0.003,
        details=f"home={button_home}, pressed={button_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
