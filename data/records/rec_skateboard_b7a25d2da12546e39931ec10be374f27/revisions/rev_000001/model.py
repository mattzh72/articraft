from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


DECK_LENGTH = 0.81
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.012
TRUCK_X = 0.19
AXLE_Z = -0.058
HALF_AXLE = 0.145
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.034
WHEEL_HALF_SPAN = WHEEL_WIDTH * 0.49
HUB_CAP_LENGTH = 0.004
HUB_CAP_EMBED = 0.001
HUB_CAP_RADIUS = 0.0125
AXLE_SPACER_LENGTH = 0.004


def _yz_section(x_pos: float, *, width: float, thickness: float, z_shift: float) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_coord, z_coord + z_shift)
        for y_coord, z_coord in rounded_rect_profile(
            width,
            thickness,
            radius=min(thickness * 0.45, 0.005),
            corner_segments=6,
        )
    ]


def _build_deck_mesh():
    return repair_loft(
        section_loft(
            [
                _yz_section(-0.405, width=0.132, thickness=DECK_THICKNESS, z_shift=0.046),
                _yz_section(-0.335, width=0.165, thickness=DECK_THICKNESS, z_shift=0.020),
                _yz_section(-0.220, width=0.194, thickness=DECK_THICKNESS, z_shift=0.002),
                _yz_section(0.000, width=DECK_WIDTH, thickness=DECK_THICKNESS, z_shift=0.000),
                _yz_section(0.220, width=0.194, thickness=DECK_THICKNESS, z_shift=0.002),
                _yz_section(0.335, width=0.165, thickness=DECK_THICKNESS, z_shift=0.020),
                _yz_section(0.405, width=0.132, thickness=DECK_THICKNESS, z_shift=0.046),
            ]
        ),
        repair="mesh",
    )


def _build_grip_mesh():
    return repair_loft(
        section_loft(
            [
                _yz_section(-0.392, width=0.118, thickness=0.0015, z_shift=0.0520),
                _yz_section(-0.322, width=0.152, thickness=0.0015, z_shift=0.0256),
                _yz_section(-0.214, width=0.184, thickness=0.0015, z_shift=0.0076),
                _yz_section(0.000, width=0.194, thickness=0.0015, z_shift=0.0056),
                _yz_section(0.214, width=0.184, thickness=0.0015, z_shift=0.0076),
                _yz_section(0.322, width=0.152, thickness=0.0015, z_shift=0.0256),
                _yz_section(0.392, width=0.118, thickness=0.0015, z_shift=0.0520),
            ]
        ),
        repair="mesh",
    )


def _build_wheel_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (0.0105, -half_width * 0.95),
        (0.0170, -half_width * 0.98),
        (0.0235, -half_width * 0.82),
        (WHEEL_RADIUS * 0.97, -half_width * 0.36),
        (WHEEL_RADIUS, 0.0),
        (WHEEL_RADIUS * 0.97, half_width * 0.36),
        (0.0235, half_width * 0.82),
        (0.0170, half_width * 0.98),
        (0.0105, half_width * 0.95),
        (0.0095, half_width * 0.40),
        (0.0090, 0.0),
        (0.0095, -half_width * 0.40),
        (0.0105, -half_width * 0.95),
    ]
    return LatheGeometry(profile, segments=56).rotate_x(pi / 2.0)


def _add_wheel_visuals(part, *, wheel_mesh, tire_mat, hub_mat, inner_side: float) -> None:
    part.visual(wheel_mesh, material=tire_mat, name="wheel_shell")
    part.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_LENGTH),
        origin=Origin(
            xyz=(0.0, inner_side * (WHEEL_HALF_SPAN + HUB_CAP_LENGTH * 0.5 - HUB_CAP_EMBED), 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=hub_mat,
        name="inner_bearing_cap",
    )
    part.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_LENGTH),
        origin=Origin(
            xyz=(0.0, -inner_side * (WHEEL_HALF_SPAN + HUB_CAP_LENGTH * 0.5 - HUB_CAP_EMBED), 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=hub_mat,
        name="outer_bearing_cap",
    )


def _add_truck_hanger_visuals(part, *, hanger_mat, axle_mat) -> None:
    beam_half = HALF_AXLE - (WHEEL_HALF_SPAN + HUB_CAP_LENGTH - HUB_CAP_EMBED + AXLE_SPACER_LENGTH)
    spacer_center = HALF_AXLE - (
        WHEEL_HALF_SPAN + HUB_CAP_LENGTH - HUB_CAP_EMBED + AXLE_SPACER_LENGTH * 0.5
    )
    part.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=axle_mat,
        name="upper_bushing",
    )
    part.visual(
        Box((0.076, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=hanger_mat,
        name="hanger_core",
    )
    part.visual(
        Box((0.040, 0.224, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=hanger_mat,
        name="hanger_crossbar",
    )
    part.visual(
        Cylinder(radius=0.0048, length=beam_half * 2.0),
        origin=Origin(xyz=(0.0, 0.0, -0.030), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_mat,
        name="axle_beam",
    )
    part.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=AXLE_SPACER_LENGTH),
        origin=Origin(
            xyz=(0.0, spacer_center, -0.030),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=axle_mat,
        name="left_axle_spacer",
    )
    part.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=AXLE_SPACER_LENGTH),
        origin=Origin(
            xyz=(0.0, -spacer_center, -0.030),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=axle_mat,
        name="right_axle_spacer",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=axle_mat,
        name="lower_bushing",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard")

    deck_wood = model.material("deck_wood", rgba=(0.54, 0.38, 0.23, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.08, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.90, 0.90, 0.86, 1.0))

    deck = model.part("deck")
    deck_mesh = mesh_from_geometry(_build_deck_mesh(), "deck_shell")
    deck.visual(deck_mesh, material=deck_wood, name="deck_shell")

    grip_mesh = mesh_from_geometry(_build_grip_mesh(), "grip_tape")
    deck.visual(grip_mesh, material=grip_black, name="grip_tape")

    deck.visual(
        Box((0.090, 0.105, 0.018)),
        origin=Origin(xyz=(-TRUCK_X, 0.0, -0.013)),
        material=grip_black,
        name="rear_support",
    )
    deck.visual(
        Box((0.090, 0.105, 0.018)),
        origin=Origin(xyz=(TRUCK_X, 0.0, -0.013)),
        material=grip_black,
        name="front_support",
    )
    deck.visual(
        Box((0.102, 0.136, 0.005)),
        origin=Origin(xyz=(TRUCK_X, 0.0, -0.0235)),
        material=truck_metal,
        name="front_baseplate",
    )
    deck.visual(
        Box((0.102, 0.136, 0.005)),
        origin=Origin(xyz=(-TRUCK_X, 0.0, -0.0235)),
        material=truck_metal,
        name="rear_baseplate",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.070)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    underbrace = model.part("underbrace")
    underbrace.visual(
        Cylinder(radius=0.007, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, -0.029), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="brace_tube",
    )
    underbrace.inertial = Inertial.from_geometry(
        Cylinder(radius=0.007, length=0.290),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.029), rpy=(0.0, pi / 2.0, 0.0)),
    )
    model.articulation(
        "deck_to_underbrace",
        ArticulationType.FIXED,
        parent=deck,
        child=underbrace,
        origin=Origin(),
    )

    wheel_mesh = mesh_from_geometry(_build_wheel_mesh(), "wheel_shell")

    front_truck = model.part("front_truck")
    _add_truck_hanger_visuals(front_truck, hanger_mat=truck_metal, axle_mat=steel_dark)
    front_truck.inertial = Inertial.from_geometry(
        Box((0.080, 0.300, 0.060)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )

    rear_truck = model.part("rear_truck")
    _add_truck_hanger_visuals(rear_truck, hanger_mat=truck_metal, axle_mat=steel_dark)
    rear_truck.inertial = Inertial.from_geometry(
        Box((0.080, 0.300, 0.060)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )

    for wheel_name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    ):
        wheel = model.part(wheel_name)
        _add_wheel_visuals(
            wheel,
            wheel_mesh=wheel_mesh,
            tire_mat=wheel_white,
            hub_mat=steel_dark,
            inner_side=-1.0 if "left" in wheel_name else 1.0,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            mass=0.11,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_truck,
        origin=Origin(xyz=(TRUCK_X, 0.0, -0.025)),
        axis=(-0.56, 0.0, 0.83),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_truck,
        origin=Origin(xyz=(-TRUCK_X, 0.0, -0.025)),
        axis=(0.56, 0.0, 0.83),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.40, upper=0.40),
    )

    for joint_name, parent_name, wheel_name, y_pos in (
        ("front_left_spin", front_truck, "front_left_wheel", HALF_AXLE),
        ("front_right_spin", front_truck, "front_right_wheel", -HALF_AXLE),
        ("rear_left_spin", rear_truck, "rear_left_wheel", HALF_AXLE),
        ("rear_right_spin", rear_truck, "rear_right_wheel", -HALF_AXLE),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent_name,
            child=wheel_name,
            origin=Origin(xyz=(0.0, y_pos, AXLE_Z + 0.028)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=45.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")

    ctx.expect_contact(
        front_truck,
        deck,
        elem_a="upper_bushing",
        elem_b="front_baseplate",
        name="front truck seats against front baseplate",
    )
    ctx.expect_contact(
        rear_truck,
        deck,
        elem_a="upper_bushing",
        elem_b="rear_baseplate",
        name="rear truck seats against rear baseplate",
    )
    ctx.expect_contact(
        front_left_wheel,
        front_truck,
        elem_a="inner_bearing_cap",
        elem_b="left_axle_spacer",
        name="front left wheel is supported on axle spacer",
    )
    ctx.expect_contact(
        front_right_wheel,
        front_truck,
        elem_a="inner_bearing_cap",
        elem_b="right_axle_spacer",
        name="front right wheel is supported on axle spacer",
    )
    ctx.expect_contact(
        rear_left_wheel,
        rear_truck,
        elem_a="inner_bearing_cap",
        elem_b="left_axle_spacer",
        name="rear left wheel is supported on axle spacer",
    )
    ctx.expect_contact(
        rear_right_wheel,
        rear_truck,
        elem_a="inner_bearing_cap",
        elem_b="right_axle_spacer",
        name="rear right wheel is supported on axle spacer",
    )

    for joint_name in (
        "front_left_spin",
        "front_right_spin",
        "rear_left_spin",
        "rear_right_spin",
    ):
        spin_joint = object_model.get_articulation(joint_name)
        limits = spin_joint.motion_limits
        ctx.check(
            f"{joint_name} remains a continuous axle spin joint",
            spin_joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and tuple(spin_joint.axis) == (0.0, 1.0, 0.0),
            details=(
                f"type={spin_joint.articulation_type}, axis={spin_joint.axis}, "
                f"limits=({None if limits is None else limits.lower}, {None if limits is None else limits.upper})"
            ),
        )

    rest_front_left = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_steer: front_steer.motion_limits.upper}):
        steered_front_left = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front truck steering swings the wheel assembly",
        rest_front_left is not None
        and steered_front_left is not None
        and abs(steered_front_left[0] - rest_front_left[0]) > 0.02
        and abs(steered_front_left[2] - rest_front_left[2]) > 0.01,
        details=f"rest={rest_front_left}, steered={steered_front_left}",
    )

    rest_rear_left = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({rear_steer: rear_steer.motion_limits.upper}):
        steered_rear_left = ctx.part_world_position(rear_left_wheel)
    ctx.check(
        "rear truck steering swings the wheel assembly",
        rest_rear_left is not None
        and steered_rear_left is not None
        and abs(steered_rear_left[0] - rest_rear_left[0]) > 0.02
        and abs(steered_rear_left[2] - rest_rear_left[2]) > 0.01,
        details=f"rest={rest_rear_left}, steered={steered_rear_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
