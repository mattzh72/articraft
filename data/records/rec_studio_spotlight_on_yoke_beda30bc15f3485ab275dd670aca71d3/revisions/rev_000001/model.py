from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cylinder_between(part, start, end, radius, material, name):
    """Add a cylinder whose local Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero length cylinder {name}")
    mx, my, mz = (sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.018, 0.018, 0.016, 1.0))
    warm_lens = model.material("warm_lens", rgba=(1.0, 0.74, 0.32, 0.42))
    warm_bulb = model.material("warm_bulb", rgba=(1.0, 0.86, 0.48, 1.0))

    # Root: a light tripod stand with an exposed stationary bearing race.
    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.060, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=dark_metal,
        name="tripod_hub",
    )
    for index, angle in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        foot = (0.43 * math.cos(angle), 0.43 * math.sin(angle), 0.020)
        knee = (0.045 * math.cos(angle), 0.045 * math.sin(angle), 0.062)
        _cylinder_between(
            stand,
            knee,
            foot,
            radius=0.012,
            material=brushed_steel,
            name=f"tripod_leg_{index}",
        )
        stand.visual(
            Cylinder(radius=0.045, length=0.022),
            origin=Origin(xyz=foot, rpy=(math.pi / 2.0, 0.0, angle)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )

    stand.visual(
        Cylinder(radius=0.028, length=0.800),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=brushed_steel,
        name="lower_tube",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.195),
        origin=Origin(xyz=(0.0, 0.0, 0.9725)),
        material=brushed_steel,
        name="upper_tube",
    )
    stand.visual(
        Cylinder(radius=0.040, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.890)),
        material=dark_metal,
        name="height_clamp",
    )
    stand.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.078, 0.0, 0.890), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="clamp_handle",
    )
    stand.visual(
        Cylinder(radius=0.070, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 1.081)),
        material=dark_metal,
        name="fixed_bearing_base",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(radius=0.062, tube=0.0045, radial_segments=12, tubular_segments=48), "lower_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 1.098)),
        material=brushed_steel,
        name="lower_bearing_race",
    )
    for index in range(8):
        angle = index * math.tau / 8.0
        stand.visual(
            Sphere(radius=0.008),
            origin=Origin(xyz=(0.062 * math.cos(angle), 0.062 * math.sin(angle), 1.107)),
            material=brushed_steel,
            name=(
                "bearing_ball_0"
                if index == 0
                else "bearing_ball_1"
                if index == 1
                else "bearing_ball_2"
                if index == 2
                else "bearing_ball_3"
                if index == 3
                else "bearing_ball_4"
                if index == 4
                else "bearing_ball_5"
                if index == 5
                else "bearing_ball_6"
                if index == 6
                else "bearing_ball_7"
            ),
        )

    # Panning child: a visibly open U-yoke on top of the exposed rotary stage.
    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.088, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_black,
        name="turntable_plate",
    )
    yoke.visual(
        mesh_from_geometry(TorusGeometry(radius=0.062, tube=0.004, radial_segments=12, tubular_segments=48), "upper_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=brushed_steel,
        name="upper_bearing_race",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_metal,
        name="pan_spindle",
    )
    yoke.visual(
        Box((0.070, 0.430, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=satin_black,
        name="yoke_bridge",
    )
    for side, y in enumerate((-0.205, 0.205)):
        yoke.visual(
            Box((0.055, 0.035, 0.480)),
            origin=Origin(xyz=(0.0, y, 0.390)),
            material=satin_black,
            name=f"yoke_arm_{side}",
        )
        yoke.visual(
            Cylinder(radius=0.045, length=0.020),
            origin=Origin(
                xyz=(0.0, -0.1775 if y < 0 else 0.1775, 0.430),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name="bearing_pad_0" if side == 0 else "bearing_pad_1",
        )
        _cylinder_between(
            yoke,
            (0.0, 0.018 if y > 0 else -0.018, 0.145),
            (0.0, y * 0.82, 0.265),
            radius=0.009,
            material=brushed_steel,
            name=f"open_strut_{side}",
        )

    # Tilting child: a hollow metal lamp can with front lip, bulb, handle, and
    # a horizontal trunnion axle captured between the yoke bearing pads.
    lamp = model.part("lamp")
    can_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.102, -0.175),
            (0.112, -0.120),
            (0.126, 0.080),
            (0.132, 0.205),
        ],
        inner_profile=[
            (0.078, -0.150),
            (0.092, -0.110),
            (0.108, 0.070),
            (0.114, 0.178),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    can_shell.rotate_y(math.pi / 2.0)
    lamp.visual(
        mesh_from_geometry(can_shell, "can_shell"),
        material=satin_black,
        name="can_shell",
    )
    lamp.visual(
        Cylinder(radius=0.105, length=0.024),
        origin=Origin(xyz=(-0.172, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    lamp.visual(
        Cylinder(radius=0.112, length=0.006),
        origin=Origin(xyz=(0.177, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_lens,
        name="front_glass",
    )
    lamp.visual(
        mesh_from_geometry(TorusGeometry(radius=0.112, tube=0.007, radial_segments=14, tubular_segments=56), "lens_retainer"),
        origin=Origin(xyz=(0.177, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lens_retainer",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.095),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_bulb,
        name="bulb",
    )
    lamp.visual(
        Cylinder(radius=0.014, length=0.245),
        origin=Origin(xyz=(-0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lamp_socket",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.335),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="tilt_axle",
    )
    handle_geom = tube_from_spline_points(
        [
            (-0.115, -0.060, 0.132),
            (-0.060, -0.020, 0.178),
            (0.020, 0.020, 0.188),
            (0.120, 0.060, 0.132),
        ],
        radius=0.006,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    lamp.visual(mesh_from_geometry(handle_geom, "top_handle"), material=brushed_steel, name="top_handle")
    lamp.visual(
        Box((0.026, 0.018, 0.038)),
        origin=Origin(xyz=(-0.115, -0.060, 0.125)),
        material=dark_metal,
        name="handle_foot_0",
    )
    lamp.visual(
        Box((0.026, 0.018, 0.038)),
        origin=Origin(xyz=(0.120, 0.060, 0.125)),
        material=dark_metal,
        name="handle_foot_1",
    )
    for index, y in enumerate((-0.036, 0.0, 0.036)):
        lamp.visual(
            Box((0.006, 0.018, 0.060)),
            origin=Origin(xyz=(-0.181, y, 0.0)),
            material=brushed_steel,
            name=f"rear_vent_{index}",
        )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        # With the can facing local +X, positive motion raises the beam.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.2,
            lower=math.radians(-40.0),
            upper=math.radians(60.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.expect_gap(
        yoke,
        stand,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_plate",
        negative_elem="bearing_ball_0",
        name="rotary stage rests on exposed bearing balls",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="tilt_axle",
        elem_b="bearing_pad_0",
        contact_tol=0.001,
        name="tilt axle is carried by one yoke bearing",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="tilt_axle",
        elem_b="bearing_pad_1",
        contact_tol=0.001,
        name="tilt axle is carried by opposite yoke bearing",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="y",
        inner_elem="can_shell",
        margin=0.0,
        name="lamp can sits inside yoke span",
    )

    rest_front = ctx.part_element_world_aabb(lamp, elem="front_glass")
    with ctx.pose({tilt: math.radians(35.0)}):
        raised_front = ctx.part_element_world_aabb(lamp, elem="front_glass")
    ctx.check(
        "positive tilt raises the beam",
        rest_front is not None
        and raised_front is not None
        and raised_front[1][2] > rest_front[1][2] + 0.030,
        details=f"rest={rest_front}, raised={raised_front}",
    )

    with ctx.pose({pan: math.radians(50.0)}):
        ctx.expect_gap(
            yoke,
            stand,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="turntable_plate",
            negative_elem="bearing_ball_0",
            name="panned yoke remains seated on rotary stage",
        )

    return ctx.report()


object_model = build_object_model()
