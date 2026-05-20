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
    model = ArticulatedObject(name="consumer_stand_mixer")

    body_orange = model.material("body_orange", rgba=(0.93, 0.34, 0.14, 1.0))
    chrome = model.material("chrome", rgba=(0.90, 0.91, 0.93, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.20, 1.0))
    control_cream = model.material("control_cream", rgba=(0.95, 0.93, 0.85, 1.0))

    def xy_section(
        z: float,
        width: float,
        depth: float,
        radius: float,
        x_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x + x_shift, y, z) for x, y in rounded_rect_profile(width, depth, radius)]

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        *,
        z_center: float = 0.0,
        y_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y + y_center, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]

    base = model.part("base")

    base.visual(
        mesh_from_geometry(
            section_loft(
                [
                    xy_section(0.0, 0.38, 0.24, 0.10, x_shift=0.02),
                    xy_section(0.030, 0.36, 0.22, 0.095, x_shift=0.02),
                    xy_section(0.060, 0.31, 0.18, 0.075, x_shift=0.025),
                ]
            ),
            "base_shell",
        ),
        material=body_orange,
        name="base_shell",
    )
    base.visual(
        mesh_from_geometry(
            section_loft(
                [
                    xy_section(0.048, 0.12, 0.13, 0.040, x_shift=-0.095),
                    xy_section(0.140, 0.11, 0.12, 0.040, x_shift=-0.098),
                    xy_section(0.235, 0.095, 0.11, 0.034, x_shift=-0.102),
                    xy_section(0.315, 0.080, 0.095, 0.028, x_shift=-0.105),
                ]
            ),
            "rear_pedestal",
        ),
        material=body_orange,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.205, 0.122, 0.014)),
        origin=Origin(xyz=(0.075, 0.0, 0.061)),
        material=chrome,
        name="slide_deck",
    )
    base.visual(
        Box((0.150, 0.074, 0.010)),
        origin=Origin(xyz=(0.047, 0.0, 0.059)),
        material=dark_trim,
        name="slide_track",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.156, 0.0, 0.126), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lock_button_bezel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.24, 0.32)),
        mass=8.0,
        origin=Origin(xyz=(0.01, 0.0, 0.16)),
    )

    bowl_stage = model.part("bowl_stage")
    bowl_stage.visual(
        Box((0.155, 0.115, 0.014)),
        origin=Origin(xyz=(0.0775, 0.0, 0.007)),
        material=satin_steel,
        name="stage_plate",
    )
    bowl_stage.visual(
        Box((0.100, 0.050, 0.020)),
        origin=Origin(xyz=(0.035, 0.0, 0.020)),
        material=dark_trim,
        name="stage_tongue",
    )
    bowl_stage.visual(
        Cylinder(radius=0.034, length=0.028),
        origin=Origin(xyz=(0.090, 0.0, 0.028)),
        material=satin_steel,
        name="bowl_pedestal",
    )
    bowl_stage.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.090, 0.0, 0.048)),
        material=satin_steel,
        name="bowl_seat",
    )
    bowl_stage.inertial = Inertial.from_geometry(
        Box((0.155, 0.115, 0.060)),
        mass=1.0,
        origin=Origin(xyz=(0.078, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_bowl_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_stage,
        origin=Origin(xyz=(0.005, 0.0, 0.068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.08, lower=0.0, upper=0.040),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.015, 0.000),
                    (0.048, 0.010),
                    (0.083, 0.046),
                    (0.097, 0.095),
                    (0.101, 0.137),
                    (0.106, 0.145),
                ],
                [
                    (0.000, 0.006),
                    (0.044, 0.014),
                    (0.078, 0.046),
                    (0.092, 0.096),
                    (0.097, 0.139),
                ],
                segments=64,
                end_cap="round",
                lip_samples=10,
            ),
            "mixing_bowl",
        ),
        material=chrome,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.106, length=0.145),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
    )

    model.articulation(
        "stage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_stage,
        child=bowl,
        origin=Origin(xyz=(0.090, 0.0, 0.054)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(
            section_loft(
                [
                    yz_section(0.018, 0.100, 0.120, 0.035, z_center=0.090),
                    yz_section(0.090, 0.176, 0.182, 0.060, z_center=0.100),
                    yz_section(0.180, 0.182, 0.170, 0.056, z_center=0.088),
                    yz_section(0.255, 0.128, 0.126, 0.042, z_center=0.070),
                    yz_section(0.305, 0.080, 0.082, 0.026, z_center=0.060),
                ]
            ),
            "head_shell",
        ),
        material=body_orange,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.200, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hub_nose",
    )
    head.visual(
        Box((0.028, 0.090, 0.040)),
        origin=Origin(xyz=(0.014, 0.0, 0.020)),
        material=dark_trim,
        name="hinge_saddle",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.31, 0.19, 0.20)),
        mass=4.6,
        origin=Origin(xyz=(0.155, 0.0, 0.085)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.105, 0.0, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="speed_pivot",
    )
    speed_lever.visual(
        Box((0.032, 0.010, 0.010)),
        origin=Origin(xyz=(0.016, 0.012, 0.0)),
        material=dark_trim,
        name="speed_handle",
    )
    speed_lever.visual(
        Box((0.012, 0.014, 0.012)),
        origin=Origin(xyz=(0.036, 0.014, 0.002)),
        material=control_cream,
        name="speed_tip",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.050, 0.020, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.020, 0.010, 0.001)),
    )

    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.082, 0.0605, 0.132)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=math.radians(-28.0),
            upper=math.radians(22.0),
        ),
    )

    head_lock_button = model.part("head_lock_button")
    head_lock_button.visual(
        Cylinder(radius=0.0075, length=0.008),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_cream,
        name="lock_cap",
    )
    head_lock_button.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lock_plunger",
    )
    head_lock_button.inertial = Inertial.from_geometry(
        Box((0.022, 0.016, 0.016)),
        mass=0.03,
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_head_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock_button,
        origin=Origin(xyz=(-0.156, 0.0, 0.126)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.03,
            lower=-0.008,
            upper=0.0,
        ),
    )

    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.0052, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chrome,
        name="whisk_drive_stub",
    )
    whisk.visual(
        Cylinder(radius=0.0042, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=satin_steel,
        name="whisk_shaft",
    )
    whisk.visual(
        Cylinder(radius=0.0105, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=chrome,
        name="whisk_ferrule",
    )

    loop_count = 8
    for index in range(loop_count):
        angle = index * math.tau / loop_count
        c = math.cos(angle)
        s = math.sin(angle)
        whisk.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.008 * c, 0.008 * s, -0.077),
                        (0.018 * c, 0.018 * s, -0.089),
                        (0.029 * c, 0.029 * s, -0.108),
                        (0.038 * c, 0.038 * s, -0.126),
                        (0.0, 0.0, -0.140),
                        (-0.038 * c, -0.038 * s, -0.126),
                        (-0.029 * c, -0.029 * s, -0.108),
                        (-0.018 * c, -0.018 * s, -0.089),
                        (-0.008 * c, -0.008 * s, -0.077),
                    ],
                    radius=0.0014,
                    samples_per_segment=18,
                    radial_segments=14,
                    cap_ends=True,
                ),
                f"whisk_loop_{index}",
            ),
            material=satin_steel,
            name=f"whisk_loop_{index}",
        )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.170),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.200, 0.0, -0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl_stage = object_model.get_part("bowl_stage")
    bowl = object_model.get_part("bowl")
    whisk = object_model.get_part("whisk")
    speed_lever = object_model.get_part("speed_lever")
    head_lock_button = object_model.get_part("head_lock_button")

    bowl_stage_joint = object_model.get_articulation("base_to_bowl_stage")
    head_joint = object_model.get_articulation("base_to_head")
    whisk_joint = object_model.get_articulation("head_to_whisk")
    speed_joint = object_model.get_articulation("base_to_speed_lever")
    lock_joint = object_model.get_articulation("base_to_head_lock_button")

    ctx.expect_contact(
        bowl_stage,
        base,
        elem_a="stage_plate",
        elem_b="slide_deck",
        contact_tol=0.0015,
        name="bowl stage sits on the polished slide deck",
    )
    ctx.expect_overlap(
        bowl,
        bowl_stage,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="bowl_seat",
        min_overlap=0.090,
        name="bowl is centered over the small slide carriage",
    )
    ctx.expect_overlap(
        bowl,
        whisk,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="whisk_ferrule",
        min_overlap=0.020,
        name="whisk drive stays centered over the bowl",
    )

    stage_upper = bowl_stage_joint.motion_limits.upper or 0.0
    head_upper = head_joint.motion_limits.upper or 0.0

    rest_stage_pos = ctx.part_world_position(bowl_stage)
    with ctx.pose({bowl_stage_joint: stage_upper}):
        extended_stage_pos = ctx.part_world_position(bowl_stage)
    ctx.check(
        "bowl carriage translates forward on a short prismatic stage",
        rest_stage_pos is not None
        and extended_stage_pos is not None
        and extended_stage_pos[0] > rest_stage_pos[0] + 0.030,
        details=f"rest={rest_stage_pos}, extended={extended_stage_pos}",
    )

    rest_whisk_pos = ctx.part_world_position(whisk)
    with ctx.pose({head_joint: head_upper}):
        raised_whisk_pos = ctx.part_world_position(whisk)
    ctx.check(
        "rear hinge lifts the mixer head upward",
        rest_whisk_pos is not None
        and raised_whisk_pos is not None
        and raised_whisk_pos[2] > rest_whisk_pos[2] + 0.080,
        details=f"rest={rest_whisk_pos}, raised={raised_whisk_pos}",
    )

    ctx.check(
        "whisk uses a continuous vertical spin axis",
        whisk_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in whisk_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={whisk_joint.articulation_type}, axis={whisk_joint.axis}",
    )

    speed_lower = speed_joint.motion_limits.lower or 0.0
    speed_upper = speed_joint.motion_limits.upper or 0.0
    with ctx.pose({speed_joint: speed_lower}):
        low_tip = ctx.part_element_world_aabb(speed_lever, elem="speed_tip")
    with ctx.pose({speed_joint: speed_upper}):
        high_tip = ctx.part_element_world_aabb(speed_lever, elem="speed_tip")
    ctx.check(
        "speed control is a small revolute lever on the base",
        speed_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in speed_joint.axis) == (0.0, 1.0, 0.0)
        and low_tip is not None
        and high_tip is not None
        and abs(high_tip[0][2] - low_tip[0][2]) > 0.008,
        details=f"type={speed_joint.articulation_type}, axis={speed_joint.axis}, low_tip={low_tip}, high_tip={high_tip}",
    )

    ctx.allow_overlap(
        base,
        head_lock_button,
        elem_a="rear_pedestal",
        elem_b="lock_plunger",
        reason="The head-lock plunger retracts into an internal pedestal cavity that is visually simplified by the closed body shell.",
    )
    rest_lock_pos = ctx.part_world_position(head_lock_button)
    lock_lower = lock_joint.motion_limits.lower or 0.0
    with ctx.pose({lock_joint: lock_lower}):
        pressed_lock_pos = ctx.part_world_position(head_lock_button)
    ctx.check(
        "head-lock is a short prismatic push button on the base",
        lock_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 3) for v in lock_joint.axis) == (-1.0, 0.0, 0.0)
        and rest_lock_pos is not None
        and pressed_lock_pos is not None
        and pressed_lock_pos[0] > rest_lock_pos[0] + 0.005,
        details=f"type={lock_joint.articulation_type}, axis={lock_joint.axis}, rest={rest_lock_pos}, pressed={pressed_lock_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
