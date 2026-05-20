from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_footrest(part, *, x_sign: float, frame_material, plate_material) -> None:
    top = (0.0, 0.0, 0.0)
    bend = (0.0, 0.000, -0.075)
    bottom = (0.0, 0.015, -0.235)
    _add_member(part, top, bend, radius=0.010, material=frame_material)
    _add_member(part, bend, bottom, radius=0.010, material=frame_material)
    part.visual(
        Box((0.095, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, 0.082, -0.240)),
        material=plate_material,
        name="footplate",
    )
    part.visual(
        Box((0.095, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.136, -0.225)),
        material=plate_material,
        name="toe_lip",
    )


def _add_rear_wheel(
    part,
    *,
    side_sign: float,
    tire_mesh,
    handrim_mesh,
    tire_material,
    rim_material,
    handrim_material,
    hub_material,
) -> None:
    wheel_roll = Origin(rpy=(0.0, pi / 2.0, 0.0))

    part.visual(
        tire_mesh,
        origin=Origin(xyz=(side_sign * 0.020, 0.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.271, length=0.018),
        origin=Origin(xyz=(side_sign * 0.015, 0.0, 0.0), rpy=wheel_roll.rpy),
        material=hub_material,
        name="rim_ring",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.028),
        origin=Origin(xyz=(side_sign * 0.014, 0.0, 0.0), rpy=wheel_roll.rpy),
        material=hub_material,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(side_sign * 0.005, 0.0, 0.0), rpy=wheel_roll.rpy),
        material=rim_material,
        name="bearing_cap",
    )

    for spoke_index in range(6):
        angle = spoke_index * (pi / 3.0)
        hub_point = (
            side_sign * 0.013,
            cos(angle) * 0.052,
            sin(angle) * 0.052,
        )
        rim_point = (
            side_sign * 0.016,
            cos(angle) * 0.271,
            sin(angle) * 0.271,
        )
        _add_member(
            part,
            hub_point,
            rim_point,
            radius=0.0045,
            material=rim_material,
            name=f"spoke_{spoke_index}",
        )

    part.visual(
        handrim_mesh,
        origin=Origin(xyz=(side_sign * 0.054, 0.0, 0.0)),
        material=handrim_material,
        name="handrim",
    )
    for bracket_index, angle in enumerate((pi / 6.0, 5.0 * pi / 6.0, 3.0 * pi / 2.0)):
        rim_bracket_point = (
            side_sign * 0.024,
            cos(angle) * 0.271,
            sin(angle) * 0.271,
        )
        handrim_bracket_point = (
            side_sign * 0.049,
            cos(angle) * 0.244,
            sin(angle) * 0.244,
        )
        _add_member(
            part,
            rim_bracket_point,
            handrim_bracket_point,
            radius=0.003,
            material=handrim_material,
            name=f"handrim_bracket_{bracket_index}",
        )


def _add_caster_swivel(part, *, frame_material, hub_material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=hub_material,
        name="stem",
    )
    part.visual(
        Box((0.036, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, -0.018, -0.076)),
        material=hub_material,
        name="fork_crown",
    )
    part.visual(
        Box((0.004, 0.012, 0.094)),
        origin=Origin(xyz=(0.015, -0.018, -0.123)),
        material=frame_material,
        name="left_fork_arm",
    )
    part.visual(
        Box((0.004, 0.012, 0.094)),
        origin=Origin(xyz=(-0.015, -0.018, -0.123)),
        material=frame_material,
        name="right_fork_arm",
    )
    _add_member(
        part,
        (0.0, 0.0, -0.020),
        (0.0, -0.018, -0.076),
        radius=0.008,
        material=hub_material,
        name="fork_neck",
    )


def _add_caster_wheel(part, *, tire_material, hub_material) -> None:
    axle_roll = (0.0, pi / 2.0, 0.0)
    part.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(rpy=axle_roll),
        material=tire_material,
        name="wheel_tire",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(rpy=axle_roll),
        material=hub_material,
        name="wheel_hub",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=axle_roll),
        material=hub_material,
        name="axle_bushing",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_grey = model.material("frame_grey", rgba=(0.29, 0.31, 0.34, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    upholstery = model.material("upholstery", rgba=(0.14, 0.15, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.82, 0.84, 0.86, 1.0))
    handrim_metal = model.material("handrim_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    rear_tire_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.288, tube=0.017).rotate_y(pi / 2.0),
        "rear_wheel_tire",
    )
    handrim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.250, tube=0.006).rotate_y(pi / 2.0),
        "rear_wheel_handrim",
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.54, 0.74, 0.96)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.01, 0.48)),
    )

    tube_r = 0.016
    seat_left_front = (0.215, 0.170, 0.480)
    seat_left_rear = (0.215, -0.120, 0.480)
    seat_right_front = (-0.215, 0.170, 0.480)
    seat_right_rear = (-0.215, -0.120, 0.480)
    rear_left_axle = (0.260, -0.100, 0.320)
    rear_right_axle = (-0.260, -0.100, 0.320)
    caster_left_mount = (0.225, 0.250, 0.205)
    caster_right_mount = (-0.225, 0.250, 0.205)
    back_left_top = (0.215, -0.170, 0.860)
    back_right_top = (-0.215, -0.170, 0.860)

    _add_member(frame, seat_left_front, seat_left_rear, radius=tube_r, material=frame_grey, name="left_seat_rail")
    _add_member(frame, seat_right_front, seat_right_rear, radius=tube_r, material=frame_grey, name="right_seat_rail")
    _add_member(frame, seat_left_rear, back_left_top, radius=tube_r, material=frame_grey, name="left_back_upright")
    _add_member(frame, seat_right_rear, back_right_top, radius=tube_r, material=frame_grey, name="right_back_upright")
    _add_member(frame, seat_left_front, caster_left_mount, radius=tube_r, material=frame_grey, name="left_front_down_tube")
    _add_member(frame, seat_right_front, caster_right_mount, radius=tube_r, material=frame_grey, name="right_front_down_tube")
    _add_member(frame, seat_left_rear, rear_left_axle, radius=tube_r, material=frame_grey, name="left_rear_brace")
    _add_member(frame, seat_right_rear, rear_right_axle, radius=tube_r, material=frame_grey, name="right_rear_brace")
    _add_member(frame, rear_left_axle, rear_right_axle, radius=tube_r, material=frame_grey, name="axle_cross_tube")
    _add_member(
        frame,
        (0.215, 0.160, 0.470),
        (-0.215, 0.160, 0.470),
        radius=0.016,
        material=frame_grey,
        name="front_seat_brace",
    )
    _add_member(
        frame,
        (0.215, -0.095, 0.462),
        (-0.215, -0.095, 0.462),
        radius=0.016,
        material=frame_grey,
        name="rear_seat_brace",
    )
    _add_member(
        frame,
        (0.105, 0.160, 0.470),
        (0.105, 0.185, 0.355),
        radius=0.012,
        material=frame_grey,
        name="left_footrest_hanger",
    )
    _add_member(
        frame,
        (-0.105, 0.160, 0.470),
        (-0.105, 0.185, 0.355),
        radius=0.012,
        material=frame_grey,
        name="right_footrest_hanger",
    )
    _add_member(
        frame,
        (0.205, -0.160, 0.615),
        (-0.205, -0.160, 0.615),
        radius=0.012,
        material=frame_grey,
        name="backrest_lower_bar",
    )
    _add_member(
        frame,
        (0.205, -0.185, 0.885),
        (-0.205, -0.185, 0.885),
        radius=0.012,
        material=frame_grey,
        name="push_handle_bar",
    )
    _add_member(
        frame,
        (0.205, -0.165, 0.840),
        (0.205, -0.215, 0.905),
        radius=0.013,
        material=frame_grey,
        name="left_push_handle",
    )
    _add_member(
        frame,
        (-0.205, -0.165, 0.840),
        (-0.205, -0.215, 0.905),
        radius=0.013,
        material=frame_grey,
        name="right_push_handle",
    )
    _add_member(
        frame,
        caster_left_mount,
        caster_right_mount,
        radius=0.012,
        material=frame_grey,
        name="caster_cross_tube",
    )

    frame.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=caster_left_mount),
        material=satin_black,
        name="left_caster_socket",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=caster_right_mount),
        material=satin_black,
        name="right_caster_socket",
    )
    frame.visual(
        Box((0.032, 0.068, 0.120)),
        origin=Origin(xyz=(0.258, -0.100, 0.360)),
        material=satin_black,
        name="left_axle_plate",
    )
    frame.visual(
        Box((0.032, 0.068, 0.120)),
        origin=Origin(xyz=(-0.258, -0.100, 0.360)),
        material=satin_black,
        name="right_axle_plate",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.279, -0.100, 0.320), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.279, -0.100, 0.320), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="right_axle_stub",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.388, 0.385, 0.022)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )
    seat.visual(
        Box((0.388, 0.385, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=upholstery,
        name="seat_sling",
    )
    seat.visual(
        Box((0.360, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.165, 0.006)),
        material=satin_black,
        name="seat_front_band",
    )
    seat.visual(
        Box((0.360, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, -0.165, 0.006)),
        material=satin_black,
        name="seat_rear_band",
    )
    seat.visual(
        Box((0.012, 0.360, 0.018)),
        origin=Origin(xyz=(0.196, 0.0, 0.009)),
        material=satin_black,
        name="seat_left_sleeve",
    )
    seat.visual(
        Box((0.012, 0.360, 0.018)),
        origin=Origin(xyz=(-0.196, 0.0, 0.009)),
        material=satin_black,
        name="seat_right_sleeve",
    )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.360, 0.020, 0.430)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )
    backrest.visual(
        Box((0.360, 0.020, 0.430)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=upholstery,
        name="backrest_panel",
    )
    backrest.visual(
        Box((0.019, 0.018, 0.240)),
        origin=Origin(xyz=(0.1895, -0.004, 0.235)),
        material=satin_black,
        name="left_backrest_sleeve",
    )
    backrest.visual(
        Box((0.019, 0.018, 0.240)),
        origin=Origin(xyz=(-0.1895, -0.004, 0.235)),
        material=satin_black,
        name="right_backrest_sleeve",
    )

    left_footrest = model.part("left_footrest")
    left_footrest.inertial = Inertial.from_geometry(
        Box((0.110, 0.190, 0.310)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.085, -0.155)),
    )
    _add_footrest(left_footrest, x_sign=1.0, frame_material=frame_grey, plate_material=satin_black)

    right_footrest = model.part("right_footrest")
    right_footrest.inertial = Inertial.from_geometry(
        Box((0.110, 0.190, 0.310)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.085, -0.155)),
    )
    _add_footrest(right_footrest, x_sign=-1.0, frame_material=frame_grey, plate_material=satin_black)

    model.articulation(
        "frame_to_seat",
        ArticulationType.FIXED,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.0, 0.020, 0.496)),
    )
    model.articulation(
        "frame_to_backrest",
        ArticulationType.FIXED,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.132, 0.535)),
    )
    model.articulation(
        "frame_to_left_footrest",
        ArticulationType.FIXED,
        parent=frame,
        child=left_footrest,
        origin=Origin(xyz=(0.105, 0.185, 0.355)),
    )
    model.articulation(
        "frame_to_right_footrest",
        ArticulationType.FIXED,
        parent=frame,
        child=right_footrest,
        origin=Origin(xyz=(-0.105, 0.185, 0.355)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.060),
        mass=1.9,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel(
        rear_left_wheel,
        side_sign=1.0,
        tire_mesh=rear_tire_mesh,
        handrim_mesh=handrim_mesh,
        tire_material=rubber,
        rim_material=satin_black,
        handrim_material=handrim_metal,
        hub_material=wheel_metal,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.060),
        mass=1.9,
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel(
        rear_right_wheel,
        side_sign=-1.0,
        tire_mesh=rear_tire_mesh,
        handrim_mesh=handrim_mesh,
        tire_material=rubber,
        rim_material=satin_black,
        handrim_material=handrim_metal,
        hub_material=wheel_metal,
    )

    left_caster_swivel = model.part("left_caster_swivel")
    left_caster_swivel.inertial = Inertial.from_geometry(
        Box((0.040, 0.050, 0.170)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.010, -0.085)),
    )
    _add_caster_swivel(
        left_caster_swivel,
        frame_material=frame_grey,
        hub_material=satin_black,
    )

    right_caster_swivel = model.part("right_caster_swivel")
    right_caster_swivel.inertial = Inertial.from_geometry(
        Box((0.040, 0.050, 0.170)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.010, -0.085)),
    )
    _add_caster_swivel(
        right_caster_swivel,
        frame_material=frame_grey,
        hub_material=satin_black,
    )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.026),
        mass=0.35,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(left_caster_wheel, tire_material=rubber, hub_material=wheel_metal)

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.026),
        mass=0.35,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(right_caster_wheel, tire_material=rubber, hub_material=wheel_metal)

    model.articulation(
        "frame_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.284, -0.100, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )
    model.articulation(
        "frame_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.284, -0.100, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )
    model.articulation(
        "frame_to_left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_swivel,
        origin=Origin(xyz=(0.225, 0.250, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "frame_to_right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_swivel,
        origin=Origin(xyz=(-0.225, 0.250, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_swivel_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=left_caster_swivel,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, -0.018, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_swivel_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=right_caster_swivel,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, -0.018, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    left_caster_swivel = object_model.get_part("left_caster_swivel")
    right_caster_swivel = object_model.get_part("right_caster_swivel")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    left_caster_joint = object_model.get_articulation("frame_to_left_caster_swivel")
    right_caster_joint = object_model.get_articulation("frame_to_right_caster_swivel")
    rear_left_joint = object_model.get_articulation("frame_to_rear_left_wheel")
    rear_right_joint = object_model.get_articulation("frame_to_rear_right_wheel")

    ctx.expect_contact(
        backrest,
        frame,
        elem_a="left_backrest_sleeve",
        elem_b="left_back_upright",
        name="left backrest sleeve clamps onto upright",
    )
    ctx.expect_contact(
        backrest,
        frame,
        elem_a="right_backrest_sleeve",
        elem_b="right_back_upright",
        name="right backrest sleeve clamps onto upright",
    )
    ctx.expect_contact(
        rear_left_wheel,
        frame,
        elem_a="hub_shell",
        elem_b="left_axle_stub",
        name="left rear wheel hub meets axle stub",
    )
    ctx.expect_contact(
        rear_right_wheel,
        frame,
        elem_a="hub_shell",
        elem_b="right_axle_stub",
        name="right rear wheel hub meets axle stub",
    )
    ctx.expect_contact(
        left_caster_swivel,
        frame,
        elem_a="stem",
        elem_b="left_caster_socket",
        name="left caster stem seats in socket",
    )
    ctx.expect_contact(
        right_caster_swivel,
        frame,
        elem_a="stem",
        elem_b="right_caster_socket",
        name="right caster stem seats in socket",
    )
    ctx.expect_contact(
        left_caster_wheel,
        left_caster_swivel,
        elem_a="wheel_hub",
        elem_b="left_fork_arm",
        name="left caster wheel hub meets fork arm",
    )
    ctx.expect_contact(
        right_caster_wheel,
        right_caster_swivel,
        elem_a="wheel_hub",
        elem_b="right_fork_arm",
        name="right caster wheel hub meets fork arm",
    )

    ctx.check(
        "rear wheels use continuous axle spin joints",
        rear_left_joint.articulation_type == ArticulationType.CONTINUOUS
        and rear_right_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(rear_left_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(rear_right_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"left_type={rear_left_joint.articulation_type}, left_axis={rear_left_joint.axis}, "
            f"right_type={rear_right_joint.articulation_type}, right_axis={rear_right_joint.axis}"
        ),
    )
    ctx.check(
        "caster stems swivel continuously about vertical axes",
        left_caster_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_caster_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_caster_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(right_caster_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"left_type={left_caster_joint.articulation_type}, left_axis={left_caster_joint.axis}, "
            f"right_type={right_caster_joint.articulation_type}, right_axis={right_caster_joint.axis}"
        ),
    )

    left_caster_rest = ctx.part_world_position(left_caster_wheel)
    with ctx.pose({left_caster_joint: 0.8}):
        left_caster_swiveled = ctx.part_world_position(left_caster_wheel)
    ctx.check(
        "left caster wheel swings outboard when the caster swivels",
        left_caster_rest is not None
        and left_caster_swiveled is not None
        and left_caster_swiveled[0] > left_caster_rest[0] + 0.010,
        details=f"rest={left_caster_rest}, swiveled={left_caster_swiveled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
