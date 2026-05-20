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
    tube_from_spline_points,
)


def _tube_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _wheel_tire_mesh(name: str, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.41, -half_width * 0.92),
        (radius * 0.66, -half_width),
        (radius * 0.90, -half_width * 0.86),
        (radius * 0.98, -half_width * 0.42),
        (radius, 0.0),
        (radius * 0.98, half_width * 0.42),
        (radius * 0.90, half_width * 0.86),
        (radius * 0.66, half_width),
        (radius * 0.41, half_width * 0.92),
        (radius * 0.38, 0.0),
        (radius * 0.41, -half_width * 0.92),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=56).rotate_x(math.pi / 2.0), name)


def _wheel_part(
    model: ArticulatedObject,
    *,
    name: str,
    tire_mesh_name: str,
    tire_radius: float,
    tire_width: float,
    stem_side: float,
    tire_material,
    hub_material,
):
    wheel = model.part(name)
    wheel.visual(
        _wheel_tire_mesh(tire_mesh_name, tire_radius, tire_width),
        material=tire_material,
        name="tire_shell",
    )
    wheel.visual(
        Cylinder(radius=tire_radius * 0.60, length=tire_width * 0.76),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="rim_shell",
    )
    wheel.visual(
        Cylinder(radius=tire_radius * 0.22, length=tire_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub_core",
    )
    wheel.visual(
        Cylinder(radius=0.0035, length=0.015),
        origin=Origin(
            xyz=(0.010, stem_side * (tire_width * 0.38), tire_radius * 0.57),
        ),
        material=hub_material,
        name="valve_stem",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=1.6,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    return wheel


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_hand_truck")

    frame_gray = model.material("frame_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.57, 0.59, 0.62, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    frame = model.part("frame")
    frame.visual(
        _tube_mesh(
            "hand_truck_upright_hoop",
            [
                (0.060, 0.145, 0.080),
                (0.056, 0.145, 0.460),
                (0.044, 0.132, 0.820),
                (0.030, 0.080, 0.962),
                (0.024, 0.000, 0.995),
                (0.030, -0.080, 0.962),
                (0.044, -0.132, 0.820),
                (0.056, -0.145, 0.460),
                (0.060, -0.145, 0.080),
            ],
            radius=0.014,
        ),
        material=frame_gray,
        name="upright_hoop",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.060, 0.0, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="lower_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.280),
        origin=Origin(xyz=(0.056, 0.0, 0.460), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="mid_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.240),
        origin=Origin(xyz=(0.044, 0.0, 0.820), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="top_crossbar",
    )
    frame.visual(
        _tube_mesh(
            "hand_truck_left_axle_brace",
            [(0.060, 0.145, 0.080), (0.028, 0.145, 0.048), (0.000, 0.145, 0.000)],
            radius=0.013,
        ),
        material=frame_gray,
        name="left_axle_brace",
    )
    frame.visual(
        _tube_mesh(
            "hand_truck_right_axle_brace",
            [(0.060, -0.145, 0.080), (0.028, -0.145, 0.048), (0.000, -0.145, 0.000)],
            radius=0.013,
        ),
        material=frame_gray,
        name="right_axle_brace",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.320),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="axle_sleeve",
    )
    frame.visual(
        _tube_mesh(
            "hand_truck_left_toe_support",
            [(0.055, 0.138, 0.050), (0.052, 0.138, -0.030), (0.050, 0.138, -0.124)],
            radius=0.010,
        ),
        material=frame_gray,
        name="left_toe_support",
    )
    frame.visual(
        _tube_mesh(
            "hand_truck_right_toe_support",
            [(0.055, -0.138, 0.050), (0.052, -0.138, -0.030), (0.050, -0.138, -0.124)],
            radius=0.010,
        ),
        material=frame_gray,
        name="right_toe_support",
    )
    frame.visual(
        Box((0.030, 0.270, 0.018)),
        origin=Origin(xyz=(0.050, 0.0, -0.124)),
        material=frame_gray,
        name="toe_stiffener",
    )
    frame.visual(
        Box((0.180, 0.300, 0.006)),
        origin=Origin(xyz=(0.140, 0.0, -0.136)),
        material=plate_gray,
        name="toe_plate",
    )
    frame.visual(
        Box((0.016, 0.300, 0.030)),
        origin=Origin(xyz=(0.222, 0.0, -0.121)),
        material=plate_gray,
        name="toe_lip",
    )
    frame.visual(
        Box((0.018, 0.018, 0.050)),
        origin=Origin(xyz=(0.060, 0.026, 0.055)),
        material=frame_gray,
        name="left_stand_hanger",
    )
    frame.visual(
        Box((0.018, 0.018, 0.050)),
        origin=Origin(xyz=(0.060, -0.026, 0.055)),
        material=frame_gray,
        name="right_stand_hanger",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.360, 0.320, 1.150)),
        mass=8.2,
        origin=Origin(xyz=(0.050, 0.0, 0.430)),
    )

    left_wheel = _wheel_part(
        model,
        name="left_wheel",
        tire_mesh_name="hand_truck_left_tire",
        tire_radius=0.145,
        tire_width=0.046,
        stem_side=1.0,
        tire_material=tire_black,
        hub_material=hub_gray,
    )
    right_wheel = _wheel_part(
        model,
        name="right_wheel",
        tire_mesh_name="hand_truck_right_tire",
        tire_radius=0.145,
        tire_width=0.046,
        stem_side=-1.0,
        tire_material=tire_black,
        hub_material=hub_gray,
    )

    parking_stand = model.part("parking_stand")
    parking_stand.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="hinge_barrel",
    )
    parking_stand.visual(
        _tube_mesh(
            "hand_truck_parking_stand",
            [
                (-0.010, 0.012, 0.000),
                (-0.048, 0.038, -0.010),
                (-0.110, 0.045, -0.018),
                (-0.170, 0.045, -0.020),
                (-0.170, -0.045, -0.020),
                (-0.110, -0.045, -0.018),
                (-0.048, -0.038, -0.010),
                (-0.010, -0.012, 0.000),
            ],
            radius=0.009,
        ),
        material=frame_gray,
        name="stand_loop",
    )
    parking_stand.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(-0.170, 0.0, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="stand_foot",
    )
    parking_stand.inertial = Inertial.from_geometry(
        Box((0.190, 0.110, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(-0.085, 0.0, -0.012)),
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(0.000, 0.183, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(0.000, -0.183, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "parking_stand_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=parking_stand,
        origin=Origin(xyz=(0.055, 0.0, 0.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    parking_stand = object_model.get_part("parking_stand")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")
    stand_hinge = object_model.get_articulation("parking_stand_hinge")

    ctx.expect_gap(
        left_wheel,
        frame,
        axis="y",
        min_gap=0.0,
        max_gap=0.020,
        name="left wheel sits just outboard of the frame",
    )
    ctx.expect_gap(
        frame,
        right_wheel,
        axis="y",
        min_gap=0.0,
        max_gap=0.020,
        name="right wheel sits just outboard of the frame",
    )
    ctx.expect_gap(
        frame,
        parking_stand,
        axis="z",
        positive_elem="lower_crossbar",
        negative_elem="stand_loop",
        min_gap=0.0,
        max_gap=0.030,
        name="folded stand tucks beneath the lower crossbar",
    )
    ctx.expect_contact(
        parking_stand,
        frame,
        elem_a="hinge_barrel",
        elem_b="left_stand_hanger",
        name="stand hinge barrel seats against the left hanger",
    )
    ctx.expect_contact(
        parking_stand,
        frame,
        elem_a="hinge_barrel",
        elem_b="right_stand_hanger",
        name="stand hinge barrel seats against the right hanger",
    )

    for joint in (left_spin, right_spin):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a continuous wheel spin",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and abs(joint.axis[0]) < 1e-6
            and abs(joint.axis[1] - 1.0) < 1e-6
            and abs(joint.axis[2]) < 1e-6,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    with ctx.pose({left_spin: 0.0}):
        stem_rest = _aabb_center(ctx.part_element_world_aabb(left_wheel, elem="valve_stem"))
    with ctx.pose({left_spin: math.pi / 2.0}):
        stem_turned = _aabb_center(ctx.part_element_world_aabb(left_wheel, elem="valve_stem"))
    ctx.check(
        "left wheel stem moves on a wheel-spin arc",
        stem_rest is not None
        and stem_turned is not None
        and abs(stem_rest[1] - stem_turned[1]) < 0.003
        and abs(stem_rest[2] - stem_turned[2]) > 0.060
        and abs(stem_rest[0] - stem_turned[0]) > 0.060,
        details=f"rest={stem_rest}, turned={stem_turned}",
    )

    stand_limits = stand_hinge.motion_limits
    with ctx.pose({stand_hinge: 0.0}):
        stand_rest = ctx.part_element_world_aabb(parking_stand, elem="stand_foot")
    with ctx.pose({stand_hinge: stand_limits.upper if stand_limits is not None and stand_limits.upper is not None else 0.0}):
        stand_open = ctx.part_element_world_aabb(parking_stand, elem="stand_foot")
    rest_center = _aabb_center(stand_rest)
    open_center = _aabb_center(stand_open)
    ctx.check(
        "parking stand rotates down out from under the frame",
        rest_center is not None
        and open_center is not None
        and stand_rest is not None
        and stand_open is not None
        and stand_open[0][2] < stand_rest[0][2] - 0.120
        and open_center[0] > rest_center[0] + 0.080,
        details=f"rest={stand_rest}, open={stand_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
