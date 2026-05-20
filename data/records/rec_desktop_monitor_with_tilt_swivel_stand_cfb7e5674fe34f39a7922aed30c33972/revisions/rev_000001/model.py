from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _curved_slab(
    width: float,
    height: float,
    thickness: float,
    *,
    center_y: float,
    curve_depth: float,
    segments_x: int = 36,
    segments_z: int = 8,
) -> MeshGeometry:
    """Wide curved monitor slab, with edges wrapping toward the viewer (-Y)."""
    geom = MeshGeometry()
    front: list[list[int]] = []
    back: list[list[int]] = []
    half_w = width / 2.0
    half_h = height / 2.0

    for iz in range(segments_z + 1):
        z = -half_h + height * iz / segments_z
        f_row: list[int] = []
        b_row: list[int] = []
        for ix in range(segments_x + 1):
            x = -half_w + width * ix / segments_x
            curve = curve_depth * (x / half_w) ** 2
            y_front = center_y - curve
            y_back = y_front + thickness
            f_row.append(geom.add_vertex(x, y_front, z))
            b_row.append(geom.add_vertex(x, y_back, z))
        front.append(f_row)
        back.append(b_row)

    for iz in range(segments_z):
        for ix in range(segments_x):
            f00, f10 = front[iz][ix], front[iz][ix + 1]
            f01, f11 = front[iz + 1][ix], front[iz + 1][ix + 1]
            b00, b10 = back[iz][ix], back[iz][ix + 1]
            b01, b11 = back[iz + 1][ix], back[iz + 1][ix + 1]
            geom.add_face(f00, f01, f11)
            geom.add_face(f00, f11, f10)
            geom.add_face(b00, b10, b11)
            geom.add_face(b00, b11, b01)

    # Close the four thin edges of the slab.
    for ix in range(segments_x):
        # bottom
        f0, f1 = front[0][ix], front[0][ix + 1]
        b0, b1 = back[0][ix], back[0][ix + 1]
        geom.add_face(f0, b1, b0)
        geom.add_face(f0, f1, b1)
        # top
        f0, f1 = front[-1][ix], front[-1][ix + 1]
        b0, b1 = back[-1][ix], back[-1][ix + 1]
        geom.add_face(f0, b0, b1)
        geom.add_face(f0, b1, f1)

    for iz in range(segments_z):
        # left side
        f0, f1 = front[iz][0], front[iz + 1][0]
        b0, b1 = back[iz][0], back[iz + 1][0]
        geom.add_face(f0, b0, b1)
        geom.add_face(f0, b1, f1)
        # right side
        f0, f1 = front[iz][-1], front[iz + 1][-1]
        b0, b1 = back[iz][-1], back[iz + 1][-1]
        geom.add_face(f0, f1, b1)
        geom.add_face(f0, b1, b0)

    return geom


def _tripod_foot(part, *, angle_deg: float, length: float, width: float, material: Material, pad: Material) -> None:
    angle = math.radians(angle_deg)
    cx = math.sin(angle) * length * 0.33
    cy = math.cos(angle) * length * 0.33
    yaw = -angle
    part.visual(
        Box((width, length, 0.026)),
        origin=Origin(xyz=(cx, cy, 0.018), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=f"foot_{int(angle_deg)}",
    )
    tx = math.sin(angle) * length * 0.58
    ty = math.cos(angle) * length * 0.58
    part.visual(
        Box((width * 0.72, 0.075, 0.006)),
        origin=Origin(xyz=(tx, ty, 0.004), rpy=(0.0, 0.0, yaw)),
        material=pad,
        name=f"rubber_pad_{int(angle_deg)}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.008, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.017, 0.020, 1.0))
    dark_shell = model.material("dark_graphite_shell", rgba=(0.055, 0.060, 0.070, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.005, 0.015, 0.035, 0.82))
    metal = model.material("dark_gunmetal", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    accent = model.material("cyan_rgb_accent", rgba=(0.0, 0.68, 1.0, 1.0))

    base = model.part("base")
    _tripod_foot(base, angle_deg=0.0, length=0.58, width=0.115, material=satin_black, pad=rubber)
    _tripod_foot(base, angle_deg=122.0, length=0.48, width=0.095, material=satin_black, pad=rubber)
    _tripod_foot(base, angle_deg=-122.0, length=0.48, width=0.095, material=satin_black, pad=rubber)
    base.visual(
        Cylinder(radius=0.090, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=metal,
        name="swivel_bearing",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.073, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal,
        name="turntable",
    )
    # Open-front sleeve: the smaller inner mast is visibly guided inside this U-channel.
    column.visual(Box((0.096, 0.014, 0.390)), origin=Origin(xyz=(0.0, 0.025, 0.220)), material=matte_black, name="sleeve_back")
    column.visual(Box((0.014, 0.064, 0.390)), origin=Origin(xyz=(-0.041, 0.000, 0.220)), material=matte_black, name="sleeve_side_0")
    column.visual(Box((0.014, 0.064, 0.390)), origin=Origin(xyz=(0.041, 0.000, 0.220)), material=matte_black, name="sleeve_side_1")
    column.visual(Box((0.096, 0.012, 0.030)), origin=Origin(xyz=(0.0, 0.025, 0.047)), material=metal, name="lower_rear_collar")
    column.visual(Box((0.012, 0.056, 0.030)), origin=Origin(xyz=(-0.042, 0.003, 0.047)), material=metal, name="lower_side_collar_0")
    column.visual(Box((0.012, 0.056, 0.030)), origin=Origin(xyz=(0.042, 0.003, 0.047)), material=metal, name="lower_side_collar_1")
    column.visual(Box((0.096, 0.012, 0.020)), origin=Origin(xyz=(0.0, 0.025, 0.415)), material=metal, name="upper_rear_collar")
    column.visual(Box((0.012, 0.056, 0.020)), origin=Origin(xyz=(-0.042, 0.003, 0.415)), material=metal, name="upper_side_collar_0")
    column.visual(Box((0.012, 0.056, 0.020)), origin=Origin(xyz=(0.042, 0.003, 0.415)), material=metal, name="upper_side_collar_1")

    mast = model.part("mast")
    mast.visual(
        Box((0.047, 0.030, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=dark_shell,
        name="mast_post",
    )
    mast.visual(
        Box((0.069, 0.024, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=metal,
        name="guide_shoe",
    )
    mast.visual(
        Box((0.080, 0.046, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=metal,
        name="top_clamp",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.032, length=0.075),
        origin=Origin(xyz=(0.0, 0.000, 0.037)),
        material=metal,
        name="neck_socket",
    )
    arms = MeshGeometry()
    arms.merge(
        tube_from_spline_points(
            [(0.0, 0.0, 0.020), (-0.070, 0.018, 0.044), (-0.165, 0.055, 0.100)],
            radius=0.016,
            samples_per_segment=10,
            radial_segments=16,
        )
    )
    arms.merge(
        tube_from_spline_points(
            [(0.0, 0.0, 0.020), (0.070, 0.018, 0.044), (0.165, 0.055, 0.100)],
            radius=0.016,
            samples_per_segment=10,
            radial_segments=16,
        )
    )
    yoke.visual(mesh_from_geometry(arms, "split_yoke_arms"), material=matte_black, name="split_arms")
    yoke.visual(
        Cylinder(radius=0.047, length=0.038),
        origin=Origin(xyz=(-0.177, 0.055, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_pad_0",
    )
    yoke.visual(
        Cylinder(radius=0.047, length=0.038),
        origin=Origin(xyz=(0.177, 0.055, 0.100), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_pad_1",
    )

    screen = model.part("screen")
    shell = _curved_slab(0.940, 0.420, 0.032, center_y=-0.145, curve_depth=0.072)
    screen.visual(mesh_from_geometry(shell, "display_shell"), material=dark_shell, name="display_shell")
    panel = _curved_slab(0.845, 0.322, 0.004, center_y=-0.149, curve_depth=0.064)
    screen.visual(mesh_from_geometry(panel, "display_glass"), material=glass, name="display_glass")
    bottom_bezel = _curved_slab(0.930, 0.065, 0.006, center_y=-0.154, curve_depth=0.070, segments_z=2)
    screen.visual(
        mesh_from_geometry(bottom_bezel, "bottom_bezel"),
        origin=Origin(xyz=(0.0, 0.0, -0.171)),
        material=matte_black,
        name="bottom_bezel",
    )
    screen.visual(
        Box((0.155, 0.115, 0.048)),
        origin=Origin(xyz=(0.0, -0.058, 0.000)),
        material=satin_black,
        name="rear_mount",
    )
    screen.visual(
        Cylinder(radius=0.030, length=0.180),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="tilt_barrel",
    )
    screen.visual(
        Cylinder(radius=0.012, length=0.392),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="tilt_axle",
    )
    screen.visual(Box((0.250, 0.006, 0.016)), origin=Origin(xyz=(0.0, -0.110, 0.128)), material=accent, name="rear_accent_top")
    screen.visual(Box((0.180, 0.006, 0.012)), origin=Origin(xyz=(0.0, -0.110, -0.105)), material=accent, name="rear_accent_bottom")
    for i, z in enumerate((-0.070, -0.045, -0.020)):
        screen.visual(
            Box((0.150, 0.005, 0.006)),
            origin=Origin(xyz=(0.0, -0.111, z)),
            material=matte_black,
            name=f"rear_vent_{i}",
        )
    screen.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.285, -0.148, -0.219)),
        material=matte_black,
        name="joystick_socket",
    )

    gimbal = model.part("joystick_gimbal")
    gimbal.visual(
        Sphere(radius=0.0105),
        origin=Origin(),
        material=metal,
        name="gimbal_ball",
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=metal,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.0, 0.0, -0.047)),
        material=rubber,
        name="joystick_cap",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4),
    )
    model.articulation(
        "column_to_mast",
        ArticulationType.PRISMATIC,
        parent=column,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.180),
    )
    model.articulation(
        "mast_to_yoke",
        ArticulationType.FIXED,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.000, 0.173)),
    )
    model.articulation(
        "yoke_to_screen",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=screen,
        origin=Origin(xyz=(0.0, 0.055, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.22, upper=0.32),
    )
    model.articulation(
        "screen_to_gimbal",
        ArticulationType.REVOLUTE,
        parent=screen,
        child=gimbal,
        origin=Origin(xyz=(0.285, -0.148, -0.219)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.5, lower=-0.28, upper=0.28),
    )
    model.articulation(
        "gimbal_to_joystick",
        ArticulationType.REVOLUTE,
        parent=gimbal,
        child=joystick,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.5, lower=-0.28, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    mast = object_model.get_part("mast")
    yoke = object_model.get_part("yoke")
    screen = object_model.get_part("screen")
    gimbal = object_model.get_part("joystick_gimbal")
    joystick = object_model.get_part("joystick")

    height = object_model.get_articulation("column_to_mast")
    tilt = object_model.get_articulation("yoke_to_screen")
    stick_roll = object_model.get_articulation("screen_to_gimbal")
    stick_pitch = object_model.get_articulation("gimbal_to_joystick")

    ctx.allow_overlap(
        yoke,
        screen,
        elem_a="pivot_pad_0",
        elem_b="tilt_axle",
        reason="The tilt axle is intentionally captured inside the left yoke pivot pad bore.",
    )
    ctx.allow_overlap(
        yoke,
        screen,
        elem_a="pivot_pad_1",
        elem_b="tilt_axle",
        reason="The tilt axle is intentionally captured inside the right yoke pivot pad bore.",
    )
    ctx.allow_overlap(
        yoke,
        screen,
        elem_a="split_arms",
        elem_b="tilt_axle",
        reason="The exposed split yoke arms terminate around the same captured tilt axle that carries the screen.",
    )
    ctx.allow_overlap(
        screen,
        gimbal,
        elem_a="joystick_socket",
        elem_b="gimbal_ball",
        reason="The tiny joystick gimbal ball is intentionally seated in the underside socket.",
    )

    ctx.expect_overlap(yoke, screen, axes="x", elem_a="pivot_pad_0", elem_b="tilt_axle", min_overlap=0.020, name="left yoke pad captures tilt axle")
    ctx.expect_overlap(yoke, screen, axes="x", elem_a="pivot_pad_1", elem_b="tilt_axle", min_overlap=0.020, name="right yoke pad captures tilt axle")
    ctx.expect_overlap(yoke, screen, axes="x", elem_a="split_arms", elem_b="tilt_axle", min_overlap=0.030, name="split yoke arms meet tilt axle")
    ctx.expect_overlap(screen, gimbal, axes="xy", elem_a="joystick_socket", elem_b="gimbal_ball", min_overlap=0.010, name="joystick ball sits in socket")
    ctx.expect_contact(gimbal, joystick, elem_a="gimbal_ball", elem_b="joystick_stem", contact_tol=0.003, name="joystick stem is supported by gimbal ball")

    ctx.expect_within(mast, column, axes="xy", inner_elem="mast_post", margin=0.004, name="mast centered inside height sleeve")
    ctx.expect_overlap(mast, column, axes="z", elem_a="mast_post", min_overlap=0.240, name="collapsed mast retained in sleeve")

    low_pos = ctx.part_world_position(mast)
    with ctx.pose({height: 0.180}):
        ctx.expect_within(mast, column, axes="xy", inner_elem="mast_post", margin=0.004, name="raised mast remains guided")
        ctx.expect_overlap(mast, column, axes="z", elem_a="mast_post", min_overlap=0.080, name="raised mast retained in sleeve")
        high_pos = ctx.part_world_position(mast)
    ctx.check(
        "height adjustment raises stand",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.15,
        details=f"low={low_pos}, high={high_pos}",
    )

    rest_screen = ctx.part_element_world_aabb(screen, elem="display_shell")
    with ctx.pose({tilt: 0.25}):
        tilted_screen = ctx.part_element_world_aabb(screen, elem="display_shell")
    ctx.check(
        "screen tilt changes display attitude",
        rest_screen is not None
        and tilted_screen is not None
        and abs((tilted_screen[1][1] - tilted_screen[0][1]) - (rest_screen[1][1] - rest_screen[0][1])) > 0.015,
        details=f"rest={rest_screen}, tilted={tilted_screen}",
    )

    rest_cap_aabb = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
    with ctx.pose({stick_roll: 0.22, stick_pitch: -0.20}):
        moved_cap_aabb = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
    rest_cap = None if rest_cap_aabb is None else tuple((rest_cap_aabb[0][i] + rest_cap_aabb[1][i]) * 0.5 for i in range(3))
    moved_cap = None if moved_cap_aabb is None else tuple((moved_cap_aabb[0][i] + moved_cap_aabb[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "joystick pivots locally",
        rest_cap is not None
        and moved_cap is not None
        and abs(moved_cap[0] - rest_cap[0]) > 0.006
        and abs(moved_cap[2] - rest_cap[2]) < 0.010,
        details=f"rest={rest_cap}, moved={moved_cap}",
    )

    return ctx.report()


object_model = build_object_model()
