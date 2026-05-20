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


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luxury_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.21, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.90, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")

    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.32, 0.23, 0.055), 0.055),
        "mixer_base_plate",
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.045, 0.0, 0.0275)),
        material=body_finish,
        name="base_plate",
    )

    column_geom = section_loft(
        [
            _xy_section(0.140, 0.120, 0.032, 0.0),
            _xy_section(0.122, 0.145, 0.040, 0.085),
            _xy_section(0.098, 0.115, 0.036, 0.210),
            _xy_section(0.074, 0.092, 0.028, 0.260),
        ]
    )
    base.visual(
        mesh_from_geometry(column_geom, "mixer_column"),
        origin=Origin(xyz=(-0.085, 0.0, 0.055)),
        material=body_finish,
        name="column_shell",
    )
    base.visual(
        Box((0.170, 0.124, 0.010)),
        origin=Origin(xyz=(0.075, 0.0, 0.060)),
        material=chrome,
        name="carriage_deck",
    )
    for side_y in (-0.046, 0.046):
        base.visual(
            Box((0.118, 0.016, 0.006)),
            origin=Origin(xyz=(0.080, side_y, 0.058)),
            material=chrome,
            name=f"slide_rail_{'left' if side_y < 0.0 else 'right'}",
        )
    base.visual(
        Box((0.050, 0.088, 0.016)),
        origin=Origin(xyz=(-0.057, 0.0, 0.317)),
        material=chrome,
        name="hinge_saddle",
    )
    base.visual(
        Box((0.030, 0.046, 0.026)),
        origin=Origin(xyz=(-0.109, 0.084, 0.228)),
        material=chrome,
        name="lock_guide",
    )
    base.visual(
        Box((0.040, 0.016, 0.052)),
        origin=Origin(xyz=(0.028, 0.107, 0.079)),
        material=chrome,
        name="dial_mount",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.32, 0.23, 0.31)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.132, 0.172, 0.012)),
        origin=Origin(xyz=(0.055, 0.0, 0.006)),
        material=chrome,
        name="carriage_platform",
    )
    carriage.visual(
        Box((0.026, 0.150, 0.040)),
        origin=Origin(xyz=(-0.012, 0.0, 0.026)),
        material=body_finish,
        name="carriage_backrest",
    )
    carriage.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.082, 0.0, 0.012)),
        material=chrome,
        name="bowl_seat",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.130, 0.170, 0.064)),
        mass=1.2,
        origin=Origin(xyz=(0.030, 0.0, 0.032)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.035, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.08,
            lower=0.0,
            upper=0.026,
        ),
    )

    bowl = model.part("bowl")
    bowl_outer = [
        (0.032, 0.0),
        (0.058, 0.008),
        (0.090, 0.040),
        (0.111, 0.115),
        (0.114, 0.175),
        (0.122, 0.186),
    ]
    bowl_inner = [
        (0.0, 0.006),
        (0.048, 0.014),
        (0.084, 0.043),
        (0.104, 0.115),
        (0.108, 0.176),
    ]
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                bowl_outer,
                bowl_inner,
                segments=72,
                end_cap="round",
                lip_samples=10,
            ),
            "mixer_bowl",
        ),
        material=stainless,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.122, length=0.186),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.082, 0.0, 0.018)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            _yz_section(0.086, 0.086, 0.024, 0.050, z_center=0.030),
            _yz_section(0.156, 0.152, 0.048, 0.135, z_center=0.038),
            _yz_section(0.172, 0.164, 0.050, 0.225, z_center=0.035),
            _yz_section(0.122, 0.116, 0.034, 0.315, z_center=0.010),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "mixer_head_shell"),
        material=body_finish,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(
            xyz=(0.030, 0.0, 0.042),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="head_hinge_barrel",
    )
    head.visual(
        Box((0.050, 0.060, 0.012)),
        origin=Origin(xyz=(0.040, 0.0, 0.016)),
        material=chrome,
        name="rear_mount_block",
    )
    head.visual(
        Cylinder(radius=0.062, length=0.040),
        origin=Origin(
            xyz=(0.224, 0.0, 0.008),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="front_trim_ring",
    )
    head.visual(
        Box((0.060, 0.048, 0.012)),
        origin=Origin(xyz=(0.185, 0.0, -0.006)),
        material=chrome,
        name="hub_bridge",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.172, 0.0, -0.014)),
        material=chrome,
        name="drive_hub",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(xyz=(0.172, 0.0, -0.037)),
        material=chrome,
        name="drive_spindle",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.305, 0.180, 0.205)),
        mass=4.8,
        origin=Origin(xyz=(0.150, 0.0, -0.010)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.055, 0.0, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    paddle = model.part("paddle")
    beater_frame = ExtrudeWithHolesGeometry(
        [
            (-0.007, 0.000),
            (0.007, 0.000),
            (0.007, -0.020),
            (0.028, -0.020),
            (0.028, -0.126),
            (0.014, -0.126),
            (0.014, -0.114),
            (-0.014, -0.114),
            (-0.014, -0.126),
            (-0.028, -0.126),
            (-0.028, -0.020),
            (-0.007, -0.020),
        ],
        [
            [
                (-0.012, -0.030),
                (0.012, -0.030),
                (0.012, -0.102),
                (-0.012, -0.102),
            ]
        ],
        0.008,
        center=True,
    )
    beater_frame.rotate_x(math.pi / 2.0)
    paddle.visual(
        mesh_from_geometry(beater_frame, "mixer_paddle_frame"),
        material=stainless,
        name="beater_frame",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((0.056, 0.008, 0.126)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.063)),
    )

    model.articulation(
        "head_to_paddle_drive",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.172, 0.0, -0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=12.0,
        ),
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.009, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="dial_knob",
    )
    speed_dial.visual(
        Box((0.007, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.014, 0.013)),
        material=dark_trim,
        name="dial_lever",
    )
    speed_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.017, length=0.018),
        mass=0.12,
        origin=Origin(
            xyz=(0.0, 0.009, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
    )

    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.028, 0.115, 0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-0.25,
            upper=1.35,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(
            xyz=(0.013, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="lock_pin",
    )
    head_lock.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(
            xyz=(0.022, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="lock_tab",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.030, 0.018, 0.018)),
        mass=0.08,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.094, 0.084, 0.228)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.03,
            lower=0.0,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    paddle = object_model.get_part("paddle")
    speed_dial = object_model.get_part("speed_dial")
    head_lock = object_model.get_part("head_lock")

    slide = object_model.get_articulation("base_to_carriage")
    tilt = object_model.get_articulation("base_to_head")
    drive = object_model.get_articulation("head_to_paddle_drive")
    dial_joint = object_model.get_articulation("base_to_speed_dial")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "bowl carriage uses a short prismatic base slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and 0.0 < slide.motion_limits.upper <= 0.03,
        details=f"{slide.articulation_type=}, limits={slide.motion_limits}",
    )
    ctx.check(
        "head uses a rear tilt hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.upper is not None
        and tilt.axis == (0.0, -1.0, 0.0),
        details=f"{tilt.axis=}, limits={tilt.motion_limits}",
    )
    ctx.check(
        "paddle uses a continuous vertical drive",
        drive.articulation_type == ArticulationType.CONTINUOUS and drive.axis == (0.0, 0.0, 1.0),
        details=f"{drive.articulation_type=}, {drive.axis=}",
    )
    ctx.check(
        "speed control stays a revolute dial on the base",
        dial_joint.articulation_type == ArticulationType.REVOLUTE
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is not None
        and dial_joint.motion_limits.upper is not None,
        details=f"{dial_joint.articulation_type=}, limits={dial_joint.motion_limits}",
    )
    ctx.check(
        "head lock stays a short prismatic plunger",
        lock_joint.articulation_type == ArticulationType.PRISMATIC
        and lock_joint.motion_limits is not None
        and lock_joint.motion_limits.upper is not None
        and 0.0 < lock_joint.motion_limits.upper <= 0.012,
        details=f"{lock_joint.articulation_type=}, limits={lock_joint.motion_limits}",
    )

    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="carriage_platform",
        elem_b="carriage_deck",
        min_overlap=0.105,
        name="carriage platform remains captured on the deck at rest",
    )
    ctx.expect_within(
        paddle,
        bowl,
        axes="xy",
        margin=0.02,
        name="paddle stays centered over the bowl in the working pose",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_ring_aabb = ctx.part_element_world_aabb(head, elem="front_trim_ring")
    rest_lock_pos = ctx.part_world_position(head_lock)

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="carriage_platform",
            elem_b="carriage_deck",
            min_overlap=0.095,
            name="carriage platform retains insertion at full slide extension",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    with ctx.pose({tilt: tilt.motion_limits.upper}):
        ctx.expect_gap(
            paddle,
            bowl,
            axis="z",
            min_gap=0.04,
            positive_elem="beater_frame",
            negative_elem="bowl_shell",
            name="tilted head lifts the paddle clear of the bowl rim",
        )
        open_ring_aabb = ctx.part_element_world_aabb(head, elem="front_trim_ring")

    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        extended_lock_pos = ctx.part_world_position(head_lock)

    ctx.check(
        "carriage slides forward",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.02,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )
    ctx.check(
        "tilting the head raises the front trim",
        rest_ring_aabb is not None
        and open_ring_aabb is not None
        and open_ring_aabb[0][2] > rest_ring_aabb[0][2] + 0.08,
        details=f"rest={rest_ring_aabb}, open={open_ring_aabb}",
    )
    ctx.check(
        "head lock plunger extends forward",
        rest_lock_pos is not None
        and extended_lock_pos is not None
        and extended_lock_pos[0] > rest_lock_pos[0] + 0.008,
        details=f"rest={rest_lock_pos}, extended={extended_lock_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
