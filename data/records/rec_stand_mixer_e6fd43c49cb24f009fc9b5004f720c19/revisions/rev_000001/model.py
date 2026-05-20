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


BASE_LENGTH = 0.38
BASE_WIDTH = 0.27
BASE_PLINTH_HEIGHT = 0.028
UPPER_DECK_HEIGHT = 0.024
SLIDE_BED_HEIGHT = 0.008
SLIDE_CLOSED_X = 0.115
SLIDE_TOP_Z = 0.060
SLIDE_TRAVEL = 0.055
HINGE_X = -0.072
HINGE_Z = 0.388
HEAD_TILT_MAX = math.radians(63.0)
SPEED_CONTROL_MAX = math.radians(55.0)
LOCK_RELEASE_TRAVEL = 0.016


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + cx, y + cy, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    zc: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + zc) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.74, 0.18, 0.16, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.86, 0.87, 0.89, 1.0))
    satin = model.material("satin", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")

    base_plinth = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(BASE_LENGTH, BASE_WIDTH, 0.052), BASE_PLINTH_HEIGHT),
        "base_plinth",
    )
    base.visual(base_plinth, origin=Origin(xyz=(0.02, 0.0, 0.0)), material=body_finish, name="base_plinth")

    upper_deck = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.24, 0.18, 0.038), UPPER_DECK_HEIGHT),
        "upper_deck",
    )
    base.visual(
        upper_deck,
        origin=Origin(xyz=(0.015, 0.0, BASE_PLINTH_HEIGHT)),
        material=body_finish,
        name="upper_deck",
    )

    base.visual(
        Box((0.23, 0.17, SLIDE_BED_HEIGHT)),
        origin=Origin(xyz=(0.10, 0.0, SLIDE_TOP_Z - SLIDE_BED_HEIGHT * 0.5)),
        material=satin,
        name="slide_bed",
    )

    base.visual(
        Box((0.112, 0.156, 0.160)),
        origin=Origin(xyz=(-0.074, 0.0, 0.132)),
        material=body_finish,
        name="lower_column",
    )
    base.visual(
        Box((0.094, 0.132, 0.140)),
        origin=Origin(xyz=(-0.082, 0.0, 0.282)),
        material=body_finish,
        name="upper_column",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.108),
        origin=Origin(xyz=(-0.102, 0.0, 0.164), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_finish,
        name="column_front_round",
    )
    base.visual(
        Box((0.050, 0.108, 0.012)),
        origin=Origin(xyz=(-0.082, 0.0, HINGE_Z - 0.032)),
        material=body_finish,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.020, 0.110, 0.068)),
        origin=Origin(xyz=(-0.119, 0.0, 0.378)),
        material=body_finish,
        name="hinge_shelf",
    )
    base.visual(
        Box((0.028, 0.028, 0.064)),
        origin=Origin(xyz=(HINGE_X - 0.004, -0.062, HINGE_Z)),
        material=satin,
        name="hinge_cheek_left",
    )
    base.visual(
        Box((0.028, 0.028, 0.064)),
        origin=Origin(xyz=(HINGE_X - 0.004, 0.062, HINGE_Z)),
        material=satin,
        name="hinge_cheek_right",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(HINGE_X - 0.004, -0.062, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="hinge_pin_left",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(HINGE_X - 0.004, 0.062, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="hinge_pin_right",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(-0.054, 0.070, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="speed_bezel",
    )
    base.visual(
        Box((0.026, 0.018, 0.032)),
        origin=Origin(xyz=(-0.054, 0.058, 0.145)),
        material=satin,
        name="speed_mount_block",
    )
    base.visual(
        Box((0.034, 0.008, 0.026)),
        origin=Origin(xyz=(-0.149, -0.084, 0.132)),
        material=satin,
        name="lock_track",
    )
    base.visual(
        Box((0.020, 0.012, 0.020)),
        origin=Origin(xyz=(-0.138, -0.082, 0.132)),
        material=satin,
        name="lock_track_bridge",
    )
    for sy in (-0.090, 0.090):
        base.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(0.17, sy, 0.004)),
            material=rubber,
            name=f"foot_front_{'left' if sy > 0 else 'right'}",
        )
        base.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(-0.12, sy, 0.004)),
            material=rubber,
            name=f"foot_rear_{'left' if sy > 0 else 'right'}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.34)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.182, 0.158, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.138, 0.118, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_dark,
        name="carriage_guide",
    )
    carriage.visual(
        Cylinder(radius=0.096, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin,
        name="bowl_seat",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.19, 0.17, 0.03)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(SLIDE_CLOSED_X, 0.0, SLIDE_TOP_Z + 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    bowl = model.part("bowl")
    bowl_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.000, 0.000),
                (0.045, 0.000),
                (0.055, 0.010),
                (0.105, 0.050),
                (0.120, 0.118),
                (0.118, 0.160),
                (0.124, 0.168),
            ],
            [
                (0.000, 0.006),
                (0.040, 0.010),
                (0.050, 0.018),
                (0.097, 0.052),
                (0.111, 0.118),
                (0.110, 0.158),
            ],
            segments=72,
        ),
        "mixing_bowl_shell",
    )
    bowl.visual(bowl_shell, material=steel, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.124, length=0.168),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.072, length=0.220),
        origin=Origin(xyz=(0.132, 0.0, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_finish,
        name="head_shell",
    )
    head.visual(
        Box((0.180, 0.148, 0.048)),
        origin=Origin(xyz=(0.150, 0.0, -0.040)),
        material=body_finish,
        name="head_lower_pan",
    )
    head.visual(
        Cylinder(radius=0.050, length=0.074),
        origin=Origin(xyz=(0.240, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_finish,
        name="nose_cap",
    )
    head.visual(
        Box((0.078, 0.116, 0.078)),
        origin=Origin(xyz=(0.222, 0.0, -0.018)),
        material=body_finish,
        name="nose_block",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.092),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="head_hinge_collar",
    )
    head.visual(
        Cylinder(radius=0.019, length=0.052),
        origin=Origin(xyz=(0.187, 0.0, -0.074)),
        material=satin,
        name="spindle_housing",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.18, 0.18)),
        mass=5.0,
        origin=Origin(xyz=(0.11, 0.0, -0.012)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.0,
            lower=0.0,
            upper=HEAD_TILT_MAX,
        ),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.007, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=steel,
        name="paddle_shaft",
    )
    paddle.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=satin,
        name="paddle_collar",
    )
    paddle.visual(
        Box((0.056, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
        material=steel,
        name="paddle_top_bridge",
    )
    paddle.visual(
        Box((0.010, 0.006, 0.084)),
        origin=Origin(xyz=(-0.020, 0.0, -0.133)),
        material=steel,
        name="paddle_left_leg",
    )
    paddle.visual(
        Box((0.010, 0.006, 0.084)),
        origin=Origin(xyz=(0.020, 0.0, -0.133)),
        material=steel,
        name="paddle_right_leg",
    )
    paddle.visual(
        Box((0.046, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.174)),
        material=steel,
        name="paddle_bottom_bridge",
    )
    paddle.visual(
        Box((0.008, 0.004, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material=steel,
        name="paddle_center_blade",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((0.08, 0.02, 0.19)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    model.articulation(
        "head_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=paddle,
        origin=Origin(xyz=(0.187, 0.0, -0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="speed_knob",
    )
    speed_control.visual(
        Box((0.030, 0.010, 0.010)),
        origin=Origin(xyz=(0.015, 0.011, 0.0)),
        material=trim_dark,
        name="speed_tab",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.032, 0.018, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.012, 0.009, 0.0)),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.054, 0.076, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=0.0,
            upper=SPEED_CONTROL_MAX,
        ),
    )

    lock_release = model.part("lock_release")
    lock_release.visual(
        Box((0.024, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_dark,
        name="lock_button",
    )
    lock_release.visual(
        Box((0.014, 0.004, 0.008)),
        origin=Origin(xyz=(-0.011, 0.0, 0.0)),
        material=trim_dark,
        name="lock_slider_body",
    )
    lock_release.inertial = Inertial.from_geometry(
        Box((0.026, 0.008, 0.014)),
        mass=0.04,
        origin=Origin(xyz=(-0.001, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lock_release",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_release,
        origin=Origin(xyz=(-0.116, -0.084, 0.132)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.06,
            lower=0.0,
            upper=LOCK_RELEASE_TRAVEL,
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
    speed_control = object_model.get_part("speed_control")
    lock_release = object_model.get_part("lock_release")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    paddle_spin = object_model.get_articulation("head_to_paddle")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_lock_release")

    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        elem_a="carriage_plate",
        elem_b="slide_bed",
        min_overlap=0.12,
        name="carriage sits squarely on the slide bed",
    )
    ctx.expect_within(
        paddle,
        bowl,
        axes="xy",
        margin=0.020,
        name="paddle stays within the bowl footprint at rest",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="spindle_housing",
        negative_elem="bowl_shell",
        min_gap=0.015,
        name="spindle housing clears the bowl rim at rest",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: SLIDE_TRAVEL}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="carriage_plate",
            elem_b="slide_bed",
            min_overlap=0.12,
            name="extended carriage still retains slide engagement",
        )
    ctx.check(
        "bowl carriage slides forward",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.04,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    paddle_rest_aabb = ctx.part_world_aabb(paddle)
    with ctx.pose({head_tilt: HEAD_TILT_MAX}):
        paddle_lifted_aabb = ctx.part_world_aabb(paddle)
        ctx.expect_gap(
            paddle,
            bowl,
            axis="z",
            min_gap=0.035,
            name="raised head lifts the paddle above the bowl",
        )
    ctx.check(
        "head tilt raises the paddle",
        paddle_rest_aabb is not None
        and paddle_lifted_aabb is not None
        and paddle_lifted_aabb[0][2] > paddle_rest_aabb[0][2] + 0.06,
        details=f"rest={paddle_rest_aabb}, lifted={paddle_lifted_aabb}",
    )

    with ctx.pose({paddle_spin: math.pi / 2.0}):
        ctx.expect_within(
            paddle,
            bowl,
            axes="xy",
            margin=0.020,
            name="spinning paddle stays inside the bowl footprint",
        )

    speed_rest = ctx.part_element_world_aabb(speed_control, elem="speed_tab")
    with ctx.pose({speed_joint: SPEED_CONTROL_MAX}):
        speed_rotated = ctx.part_element_world_aabb(speed_control, elem="speed_tab")
    ctx.check(
        "speed control pivots through a visible arc",
        speed_rest is not None
        and speed_rotated is not None
        and speed_rotated[1][2] > speed_rest[1][2] + 0.01,
        details=f"rest={speed_rest}, rotated={speed_rotated}",
    )

    lock_rest = ctx.part_world_position(lock_release)
    with ctx.pose({lock_joint: LOCK_RELEASE_TRAVEL}):
        lock_extended = ctx.part_world_position(lock_release)
        ctx.expect_overlap(
            lock_release,
            base,
            axes="yz",
            elem_a="lock_button",
            elem_b="lock_track",
            min_overlap=0.006,
            name="lock release remains aligned with the base side",
        )
    ctx.check(
        "lock release translates outward",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.010,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
