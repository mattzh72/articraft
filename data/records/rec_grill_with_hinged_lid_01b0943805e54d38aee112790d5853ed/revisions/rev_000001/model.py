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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_quad(
    mesh: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
    *,
    reverse: bool = False,
) -> None:
    if reverse:
        mesh.add_face(a, c, b)
        mesh.add_face(a, d, c)
    else:
        mesh.add_face(a, b, c)
        mesh.add_face(a, c, d)


def _barrel_shell_segment(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    x_segments: int = 18,
    angle_segments: int = 40,
) -> MeshGeometry:
    mesh = MeshGeometry()
    xs = [
        (-0.5 * length) + (length * i / x_segments)
        for i in range(x_segments + 1)
    ]
    angles = [
        start_angle + ((end_angle - start_angle) * i / angle_segments)
        for i in range(angle_segments + 1)
    ]

    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x in xs:
        outer_row: list[int] = []
        inner_row: list[int] = []
        for angle in angles:
            c = math.cos(angle)
            s = math.sin(angle)
            outer_row.append(mesh.add_vertex(x, outer_radius * c, outer_radius * s))
            inner_row.append(mesh.add_vertex(x, inner_radius * c, inner_radius * s))
        outer.append(outer_row)
        inner.append(inner_row)

    for xi in range(x_segments):
        for ai in range(angle_segments):
            _add_quad(
                mesh,
                outer[xi][ai],
                outer[xi + 1][ai],
                outer[xi + 1][ai + 1],
                outer[xi][ai + 1],
            )
            _add_quad(
                mesh,
                inner[xi][ai],
                inner[xi][ai + 1],
                inner[xi + 1][ai + 1],
                inner[xi + 1][ai],
            )

    for xi in range(x_segments):
        _add_quad(
            mesh,
            outer[xi][0],
            inner[xi][0],
            inner[xi + 1][0],
            outer[xi + 1][0],
        )
        _add_quad(
            mesh,
            outer[xi][angle_segments],
            outer[xi + 1][angle_segments],
            inner[xi + 1][angle_segments],
            inner[xi][angle_segments],
        )

    for ai in range(angle_segments):
        _add_quad(
            mesh,
            outer[0][ai],
            outer[0][ai + 1],
            inner[0][ai + 1],
            inner[0][ai],
            reverse=True,
        )
        _add_quad(
            mesh,
            outer[x_segments][ai],
            inner[x_segments][ai],
            inner[x_segments][ai + 1],
            outer[x_segments][ai + 1],
        )

    return mesh


def _wheel_tire_mesh(radius: float, width: float) -> MeshGeometry:
    half_width = width * 0.5
    profile = [
        (radius * 0.66, -half_width * 0.96),
        (radius * 0.80, -half_width * 0.98),
        (radius * 0.91, -half_width * 0.76),
        (radius * 0.98, -half_width * 0.30),
        (radius, 0.0),
        (radius * 0.98, half_width * 0.30),
        (radius * 0.91, half_width * 0.76),
        (radius * 0.80, half_width * 0.98),
        (radius * 0.66, half_width * 0.96),
        (radius * 0.62, half_width * 0.28),
        (radius * 0.60, 0.0),
        (radius * 0.62, -half_width * 0.28),
        (radius * 0.66, -half_width * 0.96),
    ]
    return LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0)


def _add_wheel_visuals(part, prefix: str, *, radius: float, width: float, rubber, steel, hub) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        mesh_from_geometry(_wheel_tire_mesh(radius, width), f"{prefix}_tire"),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.64, length=width * 0.72),
        origin=spin_origin,
        material=steel,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=radius * 0.68, length=width * 0.12),
        origin=Origin(xyz=(width * 0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="outer_rim_face",
    )
    part.visual(
        Cylinder(radius=radius * 0.68, length=width * 0.12),
        origin=Origin(xyz=(-width * 0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="inner_rim_face",
    )
    part.visual(
        Cylinder(radius=radius * 0.18, length=width + 0.015),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub,
        name="hub",
    )

    spoke_inner = radius * 0.22
    spoke_outer = radius * 0.58
    for spoke_index in range(6):
        angle = spoke_index * math.tau / 6.0
        c = math.cos(angle)
        s = math.sin(angle)
        _add_member(
            part,
            (0.0, spoke_inner * c, spoke_inner * s),
            (0.0, spoke_outer * c, spoke_outer * s),
            radius * 0.038,
            steel,
            name=f"spoke_{spoke_index}",
        )


def _polar_yz(radius: float, angle_deg: float) -> tuple[float, float]:
    angle = math.radians(angle_deg)
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_charcoal_grill")

    dark_enamel = model.material("dark_enamel", rgba=(0.12, 0.12, 0.13, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.20, 0.20, 0.20, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.28, 0.29, 0.30, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.48, 0.33, 0.20, 1.0))

    body_length = 0.70
    body_outer_radius = 0.22
    shell_thickness = 0.014
    body_inner_radius = body_outer_radius - shell_thickness
    body_center_z = 0.62
    frame_half_width = 0.22
    axle_y = -0.18
    axle_z = 0.16
    wheel_center_x = 0.34
    wheel_radius = 0.16
    wheel_width = 0.055

    lid_front_angle = 35.0
    lid_rear_angle = 145.0
    lid_hinge_radius = body_outer_radius - (shell_thickness * 0.5)
    lid_hinge_y, lid_hinge_z = _polar_yz(lid_hinge_radius, lid_rear_angle)
    hinge_barrel_offset = 0.024
    hinge_offset_y, hinge_offset_z = _polar_yz(hinge_barrel_offset, lid_rear_angle)
    hinge_outer_y, hinge_outer_z = _polar_yz(body_outer_radius, lid_rear_angle)
    hinge_anchor_y, hinge_anchor_z = _polar_yz(body_outer_radius - 0.006, lid_rear_angle)
    lid_strap_anchor_y, lid_strap_anchor_z = _polar_yz(body_outer_radius - 0.006, 138.0)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.42, 0.85)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
    )

    _add_member(frame, (-frame_half_width, 0.18, 0.36), (frame_half_width, 0.18, 0.36), 0.014, frame_paint, "front_top_rail")
    _add_member(frame, (-frame_half_width, 0.18, 0.36), (-frame_half_width, -0.08, 0.36), 0.014, frame_paint, "left_side_rail")
    _add_member(frame, (frame_half_width, 0.18, 0.36), (frame_half_width, -0.08, 0.36), 0.014, frame_paint, "right_side_rail")
    _add_member(frame, (-frame_half_width, 0.18, 0.36), (-frame_half_width, 0.18, 0.014), 0.014, frame_paint, "left_front_leg")
    _add_member(frame, (frame_half_width, 0.18, 0.36), (frame_half_width, 0.18, 0.014), 0.014, frame_paint, "right_front_leg")
    _add_member(frame, (-frame_half_width, 0.18, 0.09), (frame_half_width, 0.18, 0.09), 0.012, frame_paint, "front_lower_brace")
    _add_member(frame, (-frame_half_width, 0.06, 0.36), (-frame_half_width, 0.06, 0.396), 0.010, frame_paint, "left_front_support_post")
    _add_member(frame, (frame_half_width, 0.06, 0.36), (frame_half_width, 0.06, 0.396), 0.010, frame_paint, "right_front_support_post")
    _add_member(frame, (-frame_half_width, -0.06, 0.36), (-frame_half_width, -0.06, 0.396), 0.010, frame_paint, "left_rear_support_post")
    _add_member(frame, (frame_half_width, -0.06, 0.36), (frame_half_width, -0.06, 0.396), 0.010, frame_paint, "right_rear_support_post")
    _add_member(frame, (-frame_half_width, 0.06, 0.396), (frame_half_width, 0.06, 0.396), 0.012, frame_paint, "front_body_cradle")
    _add_member(frame, (-frame_half_width, -0.06, 0.396), (frame_half_width, -0.06, 0.396), 0.012, frame_paint, "rear_body_cradle")
    _add_member(frame, (-frame_half_width, -0.08, 0.36), (-frame_half_width, axle_y, axle_z), 0.012, frame_paint, "left_rear_hanger")
    _add_member(frame, (frame_half_width, -0.08, 0.36), (frame_half_width, axle_y, axle_z), 0.012, frame_paint, "right_rear_hanger")
    _add_member(frame, (-0.305, axle_y, axle_z), (0.305, axle_y, axle_z), 0.012, frame_paint, "common_axle")
    frame.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(-frame_half_width, 0.18, 0.007)),
        material=frame_paint,
        name="left_foot",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(frame_half_width, 0.18, 0.007)),
        material=frame_paint,
        name="right_foot",
    )

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            _barrel_shell_segment(
                length=body_length,
                outer_radius=body_outer_radius,
                inner_radius=body_inner_radius,
                start_angle=math.radians(lid_rear_angle + 0.7),
                end_angle=math.radians(lid_front_angle - 0.7 + 360.0),
            ),
            "grill_body_shell",
        ),
        material=dark_enamel,
        name="body_shell",
    )
    body.visual(
        Box((0.40, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.06, -0.206)),
        material=dark_enamel,
        name="front_mount_rail",
    )
    body.visual(
        Box((0.40, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, -0.06, -0.206)),
        material=dark_enamel,
        name="rear_mount_rail",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.15),
        origin=Origin(
            xyz=(-0.15, lid_hinge_y + hinge_offset_y, lid_hinge_z + hinge_offset_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=wheel_steel,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.15),
        origin=Origin(
            xyz=(0.15, lid_hinge_y + hinge_offset_y, lid_hinge_z + hinge_offset_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=wheel_steel,
        name="right_hinge_barrel",
    )
    body.visual(
        Box((0.024, 0.012, 0.048)),
        origin=Origin(
            xyz=(-0.15, lid_hinge_y + hinge_offset_y, lid_hinge_z + hinge_offset_z - 0.016)
        ),
        material=wheel_steel,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.024, 0.012, 0.048)),
        origin=Origin(
            xyz=(0.15, lid_hinge_y + hinge_offset_y, lid_hinge_z + hinge_offset_z - 0.016)
        ),
        material=wheel_steel,
        name="right_hinge_bracket",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_outer_radius * 2.0, body_outer_radius * 1.4)),
        mass=10.0,
    )

    lid = model.part("lid")
    lid_shell = _barrel_shell_segment(
        length=body_length,
        outer_radius=body_outer_radius,
        inner_radius=body_inner_radius,
        start_angle=math.radians(lid_front_angle + 0.7),
        end_angle=math.radians(lid_rear_angle - 0.7),
    ).translate(0.0, -lid_hinge_y, -lid_hinge_z)
    lid.visual(
        mesh_from_geometry(lid_shell, "grill_lid_shell"),
        material=dark_enamel,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.15),
        origin=Origin(
            xyz=(0.0, hinge_offset_y, hinge_offset_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=wheel_steel,
        name="center_hinge_barrel",
    )
    _add_member(
        lid,
        (-0.045, lid_strap_anchor_y - lid_hinge_y, lid_strap_anchor_z - lid_hinge_z),
        (-0.045, hinge_offset_y, hinge_offset_z),
        0.005,
        wheel_steel,
        "left_hinge_strap",
    )
    _add_member(
        lid,
        (0.045, lid_strap_anchor_y - lid_hinge_y, lid_strap_anchor_z - lid_hinge_z),
        (0.045, hinge_offset_y, hinge_offset_z),
        0.005,
        wheel_steel,
        "right_hinge_strap",
    )

    handle_mount_angle = 78.0
    base_y, base_z = _polar_yz(body_outer_radius - 0.004, handle_mount_angle)
    top_y, top_z = _polar_yz(body_outer_radius + 0.034, handle_mount_angle)
    base_y -= lid_hinge_y
    base_z -= lid_hinge_z
    top_y -= lid_hinge_y
    top_z -= lid_hinge_z
    _add_member(lid, (-0.085, base_y, base_z), (-0.085, top_y, top_z), 0.006, wheel_steel, "handle_left_post")
    _add_member(lid, (0.085, base_y, base_z), (0.085, top_y, top_z), 0.006, wheel_steel, "handle_right_post")
    _add_member(lid, (-0.11, top_y, top_z), (0.11, top_y, top_z), 0.012, handle_wood, "handle_grip")
    lid.inertial = Inertial.from_geometry(
        Box((body_length, 0.26, 0.18)),
        mass=3.5,
        origin=Origin(xyz=(0.0, -lid_hinge_y + 0.09, -lid_hinge_z + 0.08)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        "left_grill_wheel",
        radius=wheel_radius,
        width=wheel_width,
        rubber=wheel_rubber,
        steel=wheel_steel,
        hub=hub_dark,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        "right_grill_wheel",
        radius=wheel_radius,
        width=wheel_width,
        rubber=wheel_rubber,
        steel=wheel_steel,
        hub=hub_dark,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_body",
        ArticulationType.FIXED,
        parent=frame,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, body_center_z)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, lid_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(wheel_center_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-wheel_center_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.18, name="closed lid covers the barrel opening")
    ctx.expect_contact(left_wheel, frame, contact_tol=0.002, name="left wheel rides on the common axle")
    ctx.expect_contact(right_wheel, frame, contact_tol=0.002, name="right wheel rides on the common axle")

    ctx.check(
        "wheel joints share the axle axis",
        left_spin.axis == (1.0, 0.0, 0.0) and right_spin.axis == (1.0, 0.0, 0.0),
        details=f"left_axis={left_spin.axis}, right_axis={right_spin.axis}",
    )

    left_pos = ctx.part_world_position(left_wheel)
    right_pos = ctx.part_world_position(right_wheel)
    ctx.check(
        "wheels flank the grill on one axle line",
        left_pos is not None
        and right_pos is not None
        and left_pos[0] > 0.30
        and right_pos[0] < -0.30
        and abs(left_pos[1] - right_pos[1]) < 1e-6
        and abs(left_pos[2] - right_pos[2]) < 1e-6,
        details=f"left={left_pos}, right={right_pos}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.1}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid rotates upward from the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10
        and open_aabb[0][1] < closed_aabb[0][1] - 0.05,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
