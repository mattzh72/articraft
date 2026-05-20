from __future__ import annotations

from math import acos, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionProperties,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


Vec3 = tuple[float, float, float]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: Vec3) -> float:
    return sqrt(_dot(v, v))


def _unit(v: Vec3) -> Vec3:
    n = _norm(v)
    if n <= 0.0:
        raise ValueError("zero-length tube")
    return (v[0] / n, v[1] / n, v[2] / n)


def _add_tube(
    target: MeshGeometry,
    start: Vec3,
    end: Vec3,
    radius: float = 0.014,
    *,
    overlap: float | None = None,
    segments: int = 18,
) -> None:
    """Merge a round wheelchair-frame tube into target, aligned between points."""
    direction = _unit(_sub(end, start))
    extra = radius * 0.85 if overlap is None else overlap
    a = _add(start, _scale(direction, -extra))
    b = _add(end, _scale(direction, extra))
    v = _sub(b, a)
    length = _norm(v)
    center = _scale(_add(a, b), 0.5)

    tube = CylinderGeometry(radius, length, radial_segments=segments, closed=True)
    z_axis = (0.0, 0.0, 1.0)
    dot = max(-1.0, min(1.0, _dot(z_axis, direction)))
    axis = _cross(z_axis, direction)
    axis_len = _norm(axis)
    if axis_len > 1e-9:
        tube.rotate(axis, acos(dot))
    elif dot < 0.0:
        tube.rotate_x(pi)
    tube.translate(*center)
    target.merge(tube)


def _frame_tube_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    r = 0.014

    # Seat perimeter: wider at the rear, pulling into a narrow caster support.
    for y0, y1 in ((0.24, 0.20), (-0.24, -0.20)):
        _add_tube(geom, (-0.30, y0, 0.46), (0.30, y1, 0.46), r)
    _add_tube(geom, (-0.30, -0.24, 0.46), (-0.30, 0.24, 0.46), r)
    _add_tube(geom, (0.30, -0.20, 0.46), (0.30, 0.20, 0.46), r)

    # Braced side triangles.  The broad rear axle is a named visual so the
    # captured wheel-bearing overlap can be scoped precisely in tests.
    for side in (-1.0, 1.0):
        y_seat = side * 0.24
        y_front = side * 0.15
        y_axle = side * 0.31
        caster_node = (0.295, y_front, 0.20)
        _add_tube(geom, (-0.30, y_seat, 0.46), (-0.30, y_seat, 0.92), r)
        _add_tube(geom, (-0.18, y_axle, 0.31), (-0.30, y_seat, 0.46), r)
        _add_tube(geom, (-0.18, y_axle, 0.31), (0.30, side * 0.20, 0.46), r)
        _add_tube(geom, (0.30, side * 0.20, 0.46), caster_node, r)
        _add_tube(geom, (-0.18, y_axle, 0.31), caster_node, r * 0.85)
        _add_tube(geom, (0.295, side * 0.060, 0.20), (0.50, side * 0.115, 0.13), r * 0.75)
        _add_tube(geom, (0.50, side * 0.115, 0.13), (0.61, side * 0.115, 0.13), r * 0.65)

        # Rear push-handle bends.
        _add_tube(geom, (-0.30, y_seat, 0.92), (-0.43, y_seat, 0.96), r)

    # Narrow front cross-member and under-seat X bracing.
    _add_tube(geom, (0.295, -0.15, 0.20), (0.295, 0.15, 0.20), r)
    _add_tube(geom, (-0.30, -0.24, 0.46), (0.30, 0.20, 0.46), r * 0.65)
    _add_tube(geom, (-0.30, 0.24, 0.46), (0.30, -0.20, 0.46), r * 0.65)
    _add_tube(geom, (-0.30, -0.24, 0.92), (-0.30, 0.24, 0.92), r)
    return geom


def _handrim_geometry() -> MeshGeometry:
    geom = TorusGeometry(0.260, 0.0045, radial_segments=20, tubular_segments=56)
    geom.rotate_y(pi / 2.0)
    geom.translate(0.037, 0.0, 0.0)
    for i in range(8):
        a = 2.0 * pi * i / 8.0
        y = 0.260 * cos(a)
        z = 0.260 * sin(a)
        _add_tube(geom, (0.010, y, z), (0.037, y, z), 0.0032, overlap=0.002, segments=10)
    return geom


def _add_rear_wheel_visuals(part, side: str, metal: Material, rubber: Material, handrim: Material) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.275,
                0.036,
                rim=WheelRim(inner_radius=0.205, flange_height=0.010, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.042,
                    width=0.032,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.052, hole_diameter=0.004),
                ),
                face=WheelFace(dish_depth=0.008, front_inset=0.003, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=12, thickness=0.0038, window_radius=0.014),
                bore=WheelBore(style="round", diameter=0.030),
            ),
            f"{side}_rear_rim",
        ),
        material=metal,
        name="rim_spokes",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.310,
                0.045,
                inner_radius=0.276,
                tread=TireTread(style="circumferential", depth=0.003, count=4),
                grooves=(
                    TireGroove(center_offset=-0.010, width=0.0035, depth=0.002),
                    TireGroove(center_offset=0.010, width=0.0035, depth=0.002),
                ),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
            ),
            f"{side}_rear_tire",
        ),
        material=rubber,
        name="tire",
    )
    part.visual(
        mesh_from_geometry(_handrim_geometry(), f"{side}_pushrim"),
        material=handrim,
        name="pushrim",
    )


def _add_caster_wheel_visuals(part, side: str, metal: Material, rubber: Material) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.058,
                0.028,
                rim=WheelRim(inner_radius=0.040, flange_height=0.004, flange_thickness=0.002),
                hub=WheelHub(radius=0.018, width=0.024, cap_style="flat"),
                face=WheelFace(dish_depth=0.002, front_inset=0.001),
                spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
                bore=WheelBore(style="round", diameter=0.012),
            ),
            f"{side}_caster_rim",
        ),
        material=metal,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.080,
                0.034,
                inner_radius=0.059,
                tread=TireTread(style="circumferential", depth=0.002, count=2),
                sidewall=TireSidewall(style="rounded", bulge=0.035),
            ),
            f"{side}_caster_tire",
        ),
        material=rubber,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")
    polished_tube = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_fabric = model.material("navy_fabric", rgba=(0.03, 0.06, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    footplate_black = model.material("matte_black", rgba=(0.02, 0.02, 0.018, 1.0))
    wheel_metal = model.material("spoke_aluminum", rgba=(0.84, 0.86, 0.86, 1.0))
    handrim_metal = model.material("polished_handrim", rgba=(0.92, 0.93, 0.90, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_frame_tube_geometry(), "wheelchair_tubular_frame"),
        material=polished_tube,
        name="tube_frame",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.75),
        origin=Origin(xyz=(-0.18, 0.0, 0.31), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=polished_tube,
        name="rear_axle",
    )
    frame.visual(
        Box((0.48, 0.46, 0.040)),
        origin=Origin(xyz=(0.02, 0.0, 0.485)),
        material=dark_fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.045, 0.46, 0.39)),
        origin=Origin(xyz=(-0.315, 0.0, 0.715)),
        material=dark_fabric,
        name="backrest_sling",
    )
    for side in (-1.0, 1.0):
        y = side * 0.115
        caster_y = side * 0.15
        side_name = "left" if side > 0 else "right"
        frame.visual(
            Box((0.085, 0.075, 0.055)),
            origin=Origin(xyz=(0.3275, caster_y, 0.2175)),
            material=polished_tube,
            name=f"{side_name}_caster_socket",
        )
        frame.visual(
            Box((0.18, 0.105, 0.018)),
            origin=Origin(xyz=(0.62, y, 0.124), rpy=(0.0, -0.08, 0.0)),
            material=footplate_black,
            name=f"footplate_{side_name}",
        )
        frame.visual(
            Cylinder(radius=0.019, length=0.11),
            origin=Origin(xyz=(-0.475, side * 0.24, 0.965), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name=f"push_handle_{side_name}",
        )

    left_rear_wheel = model.part("left_rear_wheel")
    right_rear_wheel = model.part("right_rear_wheel")
    _add_rear_wheel_visuals(left_rear_wheel, "left", wheel_metal, rubber, handrim_metal)
    _add_rear_wheel_visuals(right_rear_wheel, "right", wheel_metal, rubber, handrim_metal)

    left_caster_fork = model.part("left_caster_fork")
    right_caster_fork = model.part("right_caster_fork")
    left_caster_wheel = model.part("left_caster_wheel")
    right_caster_wheel = model.part("right_caster_wheel")

    for fork in (left_caster_fork, right_caster_fork):
        fork.visual(
            Cylinder(radius=0.012, length=0.160),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=polished_tube,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.135, 0.078, 0.018)),
            origin=Origin(xyz=(-0.055, 0.0, -0.026)),
            material=polished_tube,
            name="fork_crown",
        )
        for side in (-1.0, 1.0):
            fork.visual(
                Box((0.045, 0.008, 0.168)),
                origin=Origin(xyz=(-0.100, side * 0.033, -0.115)),
                material=polished_tube,
                name=f"fork_blade_{'left' if side > 0 else 'right'}",
            )
        fork.visual(
            Cylinder(radius=0.0065, length=0.082),
            origin=Origin(xyz=(-0.100, 0.0, -0.120), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=polished_tube,
            name="caster_axle",
        )

    _add_caster_wheel_visuals(left_caster_wheel, "left", wheel_metal, rubber)
    _add_caster_wheel_visuals(right_caster_wheel, "right", wheel_metal, rubber)

    model.articulation(
        "left_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.18, 0.37, 0.31), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "right_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.18, -0.37, 0.31), rpy=(0.0, 0.0, -pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    for side, fork, wheel in (
        ("left", left_caster_fork, left_caster_wheel),
        ("right", right_caster_fork, right_caster_wheel),
    ):
        y = 0.15 if side == "left" else -0.15
        model.articulation(
            f"{side}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(0.36, y, 0.20)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
            motion_properties=MotionProperties(damping=0.04, friction=0.02),
        )
        model.articulation(
            f"{side}_caster_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-0.100, 0.0, -0.120), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=16.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    required_joints = {
        "left_rear_spin": ArticulationType.CONTINUOUS,
        "right_rear_spin": ArticulationType.CONTINUOUS,
        "left_caster_swivel": ArticulationType.CONTINUOUS,
        "right_caster_swivel": ArticulationType.CONTINUOUS,
        "left_caster_spin": ArticulationType.CONTINUOUS,
        "right_caster_spin": ArticulationType.CONTINUOUS,
    }
    for name, expected_type in required_joints.items():
        joint = object_model.get_articulation(name)
        ctx.check(
            f"{name}_continuous",
            joint is not None and joint.articulation_type == expected_type,
            details=f"{name} should be a continuous wheelchair wheel/caster joint.",
        )

    frame = object_model.get_part("frame")
    left_rear = object_model.get_part("left_rear_wheel")
    right_rear = object_model.get_part("right_rear_wheel")
    left_fork = object_model.get_part("left_caster_fork")
    right_fork = object_model.get_part("right_caster_fork")
    left_caster = object_model.get_part("left_caster_wheel")
    right_caster = object_model.get_part("right_caster_wheel")

    for side, fork in (("left", left_fork), ("right", right_fork)):
        ctx.allow_overlap(
            frame,
            fork,
            elem_a=f"{side}_caster_socket",
            elem_b="swivel_stem",
            reason="The caster swivel stem is intentionally captured inside the front frame socket.",
        )
        ctx.expect_overlap(
            frame,
            fork,
            axes="z",
            elem_a=f"{side}_caster_socket",
            elem_b="swivel_stem",
            min_overlap=0.025,
            name=f"{side}_caster_stem_inserted_vertically",
        )
        ctx.expect_overlap(
            frame,
            fork,
            axes="xy",
            elem_a=f"{side}_caster_socket",
            elem_b="swivel_stem",
            min_overlap=0.020,
            name=f"{side}_caster_stem_centered_in_socket",
        )

    for side, wheel in (("left", left_rear), ("right", right_rear)):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="rear_axle",
            elem_b="rim_spokes",
            reason="The fixed rear axle is intentionally represented as passing through the wheel hub bearing.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="rear_axle",
            elem_b="rim_spokes",
            min_overlap=0.020,
            name=f"{side}_rear_wheel_retained_on_axle",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="xz",
            elem_a="rear_axle",
            elem_b="rim_spokes",
            min_overlap=0.020,
            name=f"{side}_rear_axle_passes_through_hub",
        )

    for side, fork, wheel in (("left", left_fork, left_caster), ("right", right_fork, right_caster)):
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="caster_axle",
            elem_b="rim",
            reason="The caster fork axle is intentionally captured through the small wheel hub.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="y",
            elem_a="caster_axle",
            elem_b="rim",
            min_overlap=0.025,
            name=f"{side}_caster_wheel_retained_on_axle",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="xz",
            elem_a="caster_axle",
            elem_b="rim",
            min_overlap=0.008,
            name=f"{side}_caster_axle_passes_through_hub",
        )

    left_rear_pos = ctx.part_world_position(left_rear)
    right_rear_pos = ctx.part_world_position(right_rear)
    left_fork_pos = ctx.part_world_position(left_fork)
    right_fork_pos = ctx.part_world_position(right_fork)
    rear_track = abs(left_rear_pos[1] - right_rear_pos[1]) if left_rear_pos and right_rear_pos else 0.0
    caster_track = abs(left_fork_pos[1] - right_fork_pos[1]) if left_fork_pos and right_fork_pos else 1.0
    ctx.check(
        "rear_support_broader_than_front",
        rear_track > caster_track + 0.35,
        details=f"rear_track={rear_track:.3f}, caster_track={caster_track:.3f}",
    )

    rear_aabb = ctx.part_world_aabb(left_rear)
    caster_aabb = ctx.part_world_aabb(left_caster)
    if rear_aabb is not None:
        mins, maxs = rear_aabb
        rear_diameter = max(maxs[2] - mins[2], maxs[0] - mins[0])
        ctx.check("large_rear_wheel_size", rear_diameter > 0.58, details=f"diameter={rear_diameter:.3f}")
        ctx.check("rear_tire_meets_ground", abs(mins[2]) < 0.010, details=f"min_z={mins[2]:.4f}")
    else:
        ctx.fail("rear_wheel_aabb_available", "Expected rear wheel AABB.")
    if caster_aabb is not None:
        mins, maxs = caster_aabb
        caster_diameter = max(maxs[2] - mins[2], maxs[0] - mins[0])
        ctx.check("small_front_caster_size", 0.14 <= caster_diameter <= 0.18, details=f"diameter={caster_diameter:.3f}")
        ctx.check("caster_tire_meets_ground", abs(mins[2]) < 0.012, details=f"min_z={mins[2]:.4f}")
    else:
        ctx.fail("caster_wheel_aabb_available", "Expected caster wheel AABB.")

    swivel = object_model.get_articulation("left_caster_swivel")
    rest_pos = ctx.part_world_position(left_caster)
    with ctx.pose({swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(left_caster)
    ctx.check(
        "caster_swivel_moves_trailing_wheel",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) > 0.025
        and abs(rest_pos[1] - turned_pos[1]) > 0.025,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
