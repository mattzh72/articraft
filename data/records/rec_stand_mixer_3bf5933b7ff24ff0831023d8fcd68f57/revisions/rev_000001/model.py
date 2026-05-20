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
)


def _z_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_offset, y + y_offset, z)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_offset)
        for z, y in rounded_rect_profile(height, width, radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    body = model.material("body_matte", rgba=(0.34, 0.36, 0.39, 1.0))
    trim = model.material("trim_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.34, 0.23, 0.048), 0.056),
            "mixer_base_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=body,
        name="base_shell",
    )
    base.visual(
        Box((0.022, 0.060, 0.116)),
        origin=Origin(xyz=(0.020, 0.0, 0.114)),
        material=body,
        name="carriage_guide",
    )
    neck_sections = [
        _z_section(0.136, 0.122, 0.035, 0.034, x_offset=-0.104),
        _z_section(0.118, 0.110, 0.032, 0.176, x_offset=-0.100),
        _z_section(0.102, 0.092, 0.028, 0.314, x_offset=-0.094),
    ]
    base.visual(
        mesh_from_geometry(section_loft(neck_sections), "mixer_neck"),
        material=body,
        name="neck_shell",
    )
    for side, y_pos in (("left", -0.036), ("right", 0.036)):
        base.visual(
            Cylinder(radius=0.024, length=0.024),
            origin=Origin(
                xyz=(-0.094, y_pos, 0.340),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=body,
            name=f"hinge_pivot_{side}",
        )
    base.visual(
        Box((0.026, 0.018, 0.030)),
        origin=Origin(xyz=(-0.094, -0.036, 0.307)),
        material=body,
        name="hinge_support_left",
    )
    base.visual(
        Box((0.026, 0.018, 0.030)),
        origin=Origin(xyz=(-0.094, 0.036, 0.307)),
        material=body,
        name="hinge_support_right",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.118, -0.078),
            (-0.118, 0.078),
            (0.118, -0.078),
            (0.118, 0.078),
        )
    ):
        base.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, -0.005)),
            material=rubber,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.23, 0.39)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.024, 0.060, 0.108)),
        origin=Origin(xyz=(-0.068, 0.0, 0.054)),
        material=trim,
        name="slider_spine",
    )
    carriage.visual(
        Box((0.040, 0.032, 0.010)),
        origin=Origin(xyz=(-0.038, 0.0, 0.005)),
        material=trim,
        name="underbridge",
    )
    carriage.visual(
        Box((0.114, 0.150, 0.012)),
        origin=Origin(xyz=(0.037, 0.0, 0.016)),
        material=trim,
        name="bowl_platform",
    )
    carriage.visual(
        Box((0.020, 0.070, 0.010)),
        origin=Origin(xyz=(0.084, 0.0, 0.005)),
        material=trim,
        name="front_rib",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.116, 0.150, 0.102)),
        mass=0.9,
        origin=Origin(xyz=(0.032, 0.0, 0.048)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.111, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.10,
            lower=0.0,
            upper=0.042,
        ),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.056, 0.000),
                    (0.070, 0.012),
                    (0.118, 0.060),
                    (0.132, 0.126),
                    (0.132, 0.164),
                    (0.136, 0.170),
                ],
                [
                    (0.030, 0.006),
                    (0.052, 0.018),
                    (0.110, 0.064),
                    (0.124, 0.126),
                    (0.124, 0.164),
                ],
                segments=64,
            ),
            "mixer_bowl_shell",
        ),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.136, length=0.170),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.082, 0.0, 0.022)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.024, length=0.048),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=body,
        name="hinge_lug",
    )
    head.visual(
        Box((0.076, 0.040, 0.040)),
        origin=Origin(xyz=(0.038, 0.0, 0.026)),
        material=body,
        name="hinge_yoke",
    )
    head.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(0.106, 0.118, 0.030, 0.060, z_offset=0.028),
                    _yz_section(0.154, 0.176, 0.046, 0.186, z_offset=0.034),
                    _yz_section(0.114, 0.126, 0.036, 0.330, z_offset=0.026),
                ]
            ),
            "mixer_head_shell",
        ),
        material=body,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.064),
        origin=Origin(xyz=(0.312, 0.0, -0.034)),
        material=body,
        name="drive_housing",
    )
    head.visual(
        Box((0.080, 0.084, 0.024)),
        origin=Origin(xyz=(0.288, 0.0, -0.010)),
        material=body,
        name="nose_fairing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.33, 0.17, 0.19)),
        mass=4.6,
        origin=Origin(xyz=(0.165, 0.0, 0.030)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.094, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.006, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=steel,
        name="shaft",
    )
    paddle.visual(
        Box((0.034, 0.014, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, -0.098)),
        material=steel,
        name="upper_bar",
    )
    paddle.visual(
        Box((0.012, 0.014, 0.068)),
        origin=Origin(xyz=(0.014, 0.0, -0.132)),
        material=steel,
        name="outer_blade",
    )
    paddle.visual(
        Box((0.012, 0.014, 0.052)),
        origin=Origin(xyz=(-0.011, 0.0, -0.122)),
        material=steel,
        name="inner_blade",
    )
    paddle.visual(
        Box((0.036, 0.014, 0.012)),
        origin=Origin(xyz=(0.003, 0.0, -0.164)),
        material=steel,
        name="lower_bar",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((0.046, 0.020, 0.182)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
    )
    model.articulation(
        "head_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.312, 0.0, -0.066)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=14.0),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="knob_body",
    )
    speed_knob.visual(
        Box((0.012, 0.014, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=steel,
        name="knob_pointer",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.014),
        mass=0.06,
        origin=Origin(),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.102, 0.122, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=-0.7,
            upper=0.7,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.028, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim,
        name="lock_slider",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.028, 0.014, 0.010)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.076, 0.084, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.04,
            lower=0.0,
            upper=0.012,
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
    speed_knob = object_model.get_part("speed_knob")
    head_lock = object_model.get_part("head_lock")

    lift_joint = object_model.get_articulation("base_to_carriage")
    tilt_joint = object_model.get_articulation("base_to_head")
    paddle_joint = object_model.get_articulation("head_to_paddle")
    knob_joint = object_model.get_articulation("base_to_speed_knob")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.expect_gap(
        bowl,
        carriage,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="bowl_platform",
        max_gap=0.002,
        max_penetration=0.0005,
        name="bowl seats on the carriage platform",
    )
    ctx.expect_overlap(
        bowl,
        carriage,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="bowl_platform",
        min_overlap=0.10,
        name="bowl footprint stays over the compact carriage",
    )
    ctx.expect_gap(
        head,
        paddle,
        axis="z",
        positive_elem="drive_housing",
        negative_elem="shaft",
        max_gap=0.001,
        max_penetration=0.0,
        name="paddle shaft meets the drive housing",
    )
    ctx.expect_overlap(
        paddle,
        bowl,
        axes="xy",
        elem_a="lower_bar",
        elem_b="bowl_shell",
        min_overlap=0.012,
        name="paddle hangs within the bowl footprint",
    )

    carriage_rest = ctx.part_world_position(carriage)
    lock_rest = ctx.part_world_position(head_lock)
    nose_rest = ctx.part_element_world_aabb(head, elem="nose_fairing")
    knob_rest = ctx.part_element_world_aabb(speed_knob, elem="knob_pointer")
    paddle_rest = ctx.part_element_world_aabb(paddle, elem="outer_blade")

    with ctx.pose({lift_joint: 0.042}):
        carriage_lifted = ctx.part_world_position(carriage)
    ctx.check(
        "bowl carriage lifts upward",
        carriage_rest is not None
        and carriage_lifted is not None
        and carriage_lifted[2] > carriage_rest[2] + 0.03,
        details=f"rest={carriage_rest}, lifted={carriage_lifted}",
    )

    with ctx.pose({tilt_joint: math.radians(58.0)}):
        nose_tilted = ctx.part_element_world_aabb(head, elem="nose_fairing")
    ctx.check(
        "head tilts upward from the rear hinge",
        nose_rest is not None
        and nose_tilted is not None
        and nose_tilted[1][2] > nose_rest[1][2] + 0.10,
        details=f"rest={nose_rest}, tilted={nose_tilted}",
    )

    with ctx.pose({paddle_joint: math.pi / 2.0}):
        paddle_quarter = ctx.part_element_world_aabb(paddle, elem="outer_blade")
    ctx.check(
        "paddle spins continuously under the head",
        paddle_rest is not None
        and paddle_quarter is not None
        and abs((paddle_quarter[0][1] + paddle_quarter[1][1]) - (paddle_rest[0][1] + paddle_rest[1][1])) > 0.015,
        details=f"rest={paddle_rest}, quarter_turn={paddle_quarter}",
    )

    with ctx.pose({knob_joint: 0.7}):
        knob_turned = ctx.part_element_world_aabb(speed_knob, elem="knob_pointer")
    ctx.check(
        "speed control knob rotates on the base side",
        knob_rest is not None
        and knob_turned is not None
        and abs((knob_turned[0][2] + knob_turned[1][2]) - (knob_rest[0][2] + knob_rest[1][2])) > 0.010,
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    with ctx.pose({lock_joint: 0.012}):
        lock_slid = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slides forward on the base",
        lock_rest is not None
        and lock_slid is not None
        and lock_slid[0] > lock_rest[0] + 0.010,
        details=f"rest={lock_rest}, slid={lock_slid}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
