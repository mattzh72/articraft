from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


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


def _build_envelope_mesh():
    return LatheGeometry(
        [
            (0.0, -9.50),
            (0.42, -9.05),
            (0.95, -8.35),
            (1.68, -6.70),
            (2.14, -4.20),
            (2.28, -1.20),
            (2.26, 1.90),
            (2.06, 4.90),
            (1.58, 7.25),
            (0.82, 8.85),
            (0.28, 9.35),
            (0.0, 9.50),
        ],
        segments=96,
    ).rotate_y(math.pi / 2.0)


def _build_engine_pod_mesh():
    return LatheGeometry(
        [
            (0.0, -0.82),
            (0.14, -0.76),
            (0.24, -0.48),
            (0.29, -0.05),
            (0.28, 0.42),
            (0.22, 0.76),
            (0.10, 0.95),
            (0.0, 1.02),
        ],
        segments=56,
    ).rotate_y(math.pi / 2.0)


def _vertical_surface_mesh(profile: list[tuple[float, float]], thickness: float):
    return ExtrudeGeometry(profile, thickness, center=True).rotate_x(math.pi / 2.0)


def _horizontal_surface_mesh(profile: list[tuple[float, float]], thickness: float):
    return ExtrudeGeometry(profile, thickness, center=True)


def _add_pylon_visuals(part, *, side_sign: float, frame_material, cap_material) -> None:
    part.visual(
        Box((0.48, 0.08, 0.12)),
        origin=Origin(xyz=(0.02, 0.04 * side_sign, 0.0)),
        material=cap_material,
        name="root_pad",
    )
    _add_member(
        part,
        (0.18, 0.04 * side_sign, 0.02),
        (0.18, 1.07 * side_sign, -0.89),
        radius=0.065,
        material=frame_material,
        name="upper_strut",
    )
    _add_member(
        part,
        (-0.16, 0.04 * side_sign, -0.01),
        (0.02, 1.00 * side_sign, -0.95),
        radius=0.058,
        material=frame_material,
        name="lower_strut",
    )
    _add_member(
        part,
        (0.22, 0.44 * side_sign, -0.28),
        (-0.08, 0.94 * side_sign, -0.86),
        radius=0.040,
        material=frame_material,
        name="brace",
    )
    part.visual(
        Box((0.34, 0.28, 0.12)),
        origin=Origin(xyz=(0.10, 1.18 * side_sign, -0.94)),
        material=cap_material,
        name="pivot_cap",
    )


def _add_engine_visuals(part, *, shell_name: str, shell_material, metal_material) -> None:
    part.visual(
        Box((0.30, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=metal_material,
        name="mount_pad",
    )
    part.visual(
        Box((0.46, 0.24, 0.14)),
        origin=Origin(xyz=(0.04, 0.0, -0.12)),
        material=metal_material,
        name="mount_block",
    )
    part.visual(
        _save_mesh(shell_name, _build_engine_pod_mesh()),
        origin=Origin(xyz=(0.10, 0.0, -0.29)),
        material=shell_material,
        name="pod_shell",
    )
    part.visual(
        Cylinder(radius=0.10, length=0.24),
        origin=Origin(xyz=(-0.67, 0.0, -0.29), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_material,
        name="exhaust_ring",
    )
    part.visual(
        Cylinder(radius=0.055, length=0.32),
        origin=Origin(xyz=(0.98, 0.0, -0.29), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_material,
        name="shaft_stub",
    )


def _add_propeller_visuals(part, *, blade_material, spinner_material) -> None:
    part.visual(
        Cylinder(radius=0.085, length=0.16),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spinner_material,
        name="hub",
    )
    part.visual(
        Sphere(radius=0.09),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=spinner_material,
        name="spinner",
    )
    for blade_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        part.visual(
            Box((0.10, 0.05, 0.82)),
            origin=Origin(xyz=(0.07, 0.0, 0.41), rpy=(angle, 0.16, 0.0)),
            material=blade_material,
            name=f"blade_{blade_index}",
        )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="communications_blimp")

    model.material("envelope_white", rgba=(0.89, 0.92, 0.94, 1.0))
    model.material("tail_white", rgba=(0.86, 0.89, 0.92, 1.0))
    model.material("gondola_gray", rgba=(0.31, 0.34, 0.38, 1.0))
    model.material("frame_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("metal_gray", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("window_dark", rgba=(0.10, 0.13, 0.16, 1.0))
    model.material("prop_black", rgba=(0.06, 0.06, 0.07, 1.0))

    hull = model.part("hull")
    hull.visual(
        _save_mesh("blimp_envelope", _build_envelope_mesh()),
        material="envelope_white",
        name="envelope",
    )
    hull.visual(
        Box((4.80, 0.56, 0.28)),
        origin=Origin(xyz=(0.18, 0.0, -2.10)),
        material="gondola_gray",
        name="keel",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.95, -0.32), (-0.95, 0.32), (0.95, -0.32), (0.95, 0.32))
    ):
        hull.visual(
            Box((0.12, 0.10, 0.72)),
            origin=Origin(xyz=(x_pos, y_pos, -2.46)),
            material="frame_gray",
            name=f"suspension_{index}",
        )

    gondola = model.part("gondola")
    gondola.visual(
        Box((3.40, 1.25, 1.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.525)),
        material="gondola_gray",
        name="cabin",
    )
    gondola.visual(
        Box((2.10, 0.82, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material="frame_gray",
        name="roof",
    )
    gondola.visual(
        Cylinder(radius=0.32, length=0.42),
        origin=Origin(xyz=(1.91, 0.0, -0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="gondola_gray",
        name="nose_fairing",
    )
    gondola.visual(
        Box((0.12, 0.90, 0.42)),
        origin=Origin(xyz=(1.62, 0.0, -0.48)),
        material="window_dark",
        name="window_strip",
    )

    left_pylon = model.part("left_pylon")
    _add_pylon_visuals(
        left_pylon,
        side_sign=1.0,
        frame_material="frame_gray",
        cap_material="metal_gray",
    )

    right_pylon = model.part("right_pylon")
    _add_pylon_visuals(
        right_pylon,
        side_sign=-1.0,
        frame_material="frame_gray",
        cap_material="metal_gray",
    )

    left_engine = model.part("left_engine")
    _add_engine_visuals(
        left_engine,
        shell_name="left_engine_shell",
        shell_material="gondola_gray",
        metal_material="metal_gray",
    )

    right_engine = model.part("right_engine")
    _add_engine_visuals(
        right_engine,
        shell_name="right_engine_shell",
        shell_material="gondola_gray",
        metal_material="metal_gray",
    )

    left_prop = model.part("left_prop")
    _add_propeller_visuals(
        left_prop,
        blade_material="prop_black",
        spinner_material="metal_gray",
    )

    right_prop = model.part("right_prop")
    _add_propeller_visuals(
        right_prop,
        blade_material="prop_black",
        spinner_material="metal_gray",
    )

    top_fin = model.part("top_fin")
    top_fin.visual(
        _save_mesh(
            "top_fin_shell",
            _vertical_surface_mesh(
                [
                    (0.0, 0.0),
                    (-0.34, 1.86),
                    (-0.92, 1.56),
                    (-1.25, 0.08),
                ],
                0.10,
            ),
        ),
        material="tail_white",
        name="fin",
    )
    top_fin.visual(
        Box((0.28, 0.12, 0.28)),
        origin=Origin(xyz=(0.14, 0.0, 0.14)),
        material="tail_white",
        name="root_fairing",
    )

    rudder = model.part("rudder")
    rudder.visual(
        _save_mesh(
            "rudder_shell",
            _vertical_surface_mesh(
                [
                    (0.0, 0.0),
                    (-0.12, 1.34),
                    (-0.48, 1.18),
                    (-0.62, 0.02),
                ],
                0.075,
            ),
        ),
        material="tail_white",
        name="rudder_surface",
    )

    bottom_fin = model.part("bottom_fin")
    bottom_fin.visual(
        _save_mesh(
            "bottom_fin_shell",
            _vertical_surface_mesh(
                [
                    (0.0, 0.0),
                    (-0.34, -1.86),
                    (-0.92, -1.56),
                    (-1.25, -0.08),
                ],
                0.10,
            ),
        ),
        material="tail_white",
        name="fin",
    )
    bottom_fin.visual(
        Box((0.28, 0.12, 0.28)),
        origin=Origin(xyz=(0.14, 0.0, -0.14)),
        material="tail_white",
        name="root_fairing",
    )

    left_stabilizer = model.part("left_stabilizer")
    left_stabilizer.visual(
        _save_mesh(
            "left_stabilizer_shell",
            _horizontal_surface_mesh(
                [
                    (0.0, 0.0),
                    (-0.26, 1.88),
                    (-0.82, 1.63),
                    (-1.18, 1.28),
                    (-1.28, 0.12),
                ],
                0.08,
            ),
        ),
        material="tail_white",
        name="stabilizer",
    )
    left_stabilizer.visual(
        Box((0.30, 0.24, 0.12)),
        origin=Origin(xyz=(0.15, 0.12, 0.0)),
        material="tail_white",
        name="root_fairing",
    )

    right_stabilizer = model.part("right_stabilizer")
    right_stabilizer.visual(
        _save_mesh(
            "right_stabilizer_shell",
            _horizontal_surface_mesh(
                [
                    (0.0, 0.0),
                    (-0.26, -1.88),
                    (-0.82, -1.63),
                    (-1.18, -1.28),
                    (-1.28, -0.12),
                ],
                0.08,
            ),
        ),
        material="tail_white",
        name="stabilizer",
    )
    right_stabilizer.visual(
        Box((0.30, 0.24, 0.12)),
        origin=Origin(xyz=(0.15, -0.12, 0.0)),
        material="tail_white",
        name="root_fairing",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        _save_mesh(
            "left_elevator_shell",
            _horizontal_surface_mesh(
                [
                    (0.0, 0.0),
                    (-0.08, 1.24),
                    (-0.42, 1.10),
                    (-0.62, 0.02),
                ],
                0.065,
            ),
        ),
        material="tail_white",
        name="elevator_surface",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        _save_mesh(
            "right_elevator_shell",
            _horizontal_surface_mesh(
                [
                    (0.0, 0.0),
                    (-0.08, -1.24),
                    (-0.42, -1.10),
                    (-0.62, -0.02),
                ],
                0.065,
            ),
        ),
        material="tail_white",
        name="elevator_surface",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -2.82)),
    )
    model.articulation(
        "hull_to_left_pylon",
        ArticulationType.FIXED,
        parent=hull,
        child=left_pylon,
        origin=Origin(xyz=(0.15, 1.74, -1.48)),
    )
    model.articulation(
        "hull_to_right_pylon",
        ArticulationType.FIXED,
        parent=hull,
        child=right_pylon,
        origin=Origin(xyz=(0.15, -1.74, -1.48)),
    )
    model.articulation(
        "left_engine_vector",
        ArticulationType.REVOLUTE,
        parent=left_pylon,
        child=left_engine,
        origin=Origin(xyz=(0.10, 1.18, -1.00)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.5,
            lower=-math.radians(35.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "right_engine_vector",
        ArticulationType.REVOLUTE,
        parent=right_pylon,
        child=right_engine,
        origin=Origin(xyz=(0.10, -1.18, -1.00)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.5,
            lower=-math.radians(35.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "left_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=left_engine,
        child=left_prop,
        origin=Origin(xyz=(1.22, 0.0, -0.29)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=55.0),
    )
    model.articulation(
        "right_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=right_engine,
        child=right_prop,
        origin=Origin(xyz=(1.22, 0.0, -0.29)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=55.0),
    )
    model.articulation(
        "hull_to_top_fin",
        ArticulationType.FIXED,
        parent=hull,
        child=top_fin,
        origin=Origin(xyz=(-9.05, 0.0, 0.62)),
    )
    model.articulation(
        "top_fin_to_rudder",
        ArticulationType.REVOLUTE,
        parent=top_fin,
        child=rudder,
        origin=Origin(xyz=(-1.25, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.2,
            lower=-math.radians(25.0),
            upper=math.radians(25.0),
        ),
    )
    model.articulation(
        "hull_to_bottom_fin",
        ArticulationType.FIXED,
        parent=hull,
        child=bottom_fin,
        origin=Origin(xyz=(-9.05, 0.0, -0.62)),
    )
    model.articulation(
        "hull_to_left_stabilizer",
        ArticulationType.FIXED,
        parent=hull,
        child=left_stabilizer,
        origin=Origin(xyz=(-8.85, 0.72, 0.02)),
    )
    model.articulation(
        "hull_to_right_stabilizer",
        ArticulationType.FIXED,
        parent=hull,
        child=right_stabilizer,
        origin=Origin(xyz=(-8.85, -0.72, 0.02)),
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=left_stabilizer,
        child=left_elevator,
        origin=Origin(xyz=(-1.28, 0.12, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.5,
            lower=-math.radians(18.0),
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=right_stabilizer,
        child=right_elevator,
        origin=Origin(xyz=(-1.28, -0.12, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.5,
            lower=-math.radians(18.0),
            upper=math.radians(18.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "top_fin",
        "hull",
        elem_a="root_fairing",
        elem_b="envelope",
        reason="The upper tail fin root is intentionally faired into the blimp envelope skin.",
    )
    ctx.allow_overlap(
        "bottom_fin",
        "hull",
        elem_a="root_fairing",
        elem_b="envelope",
        reason="The lower tail fin root is intentionally faired into the blimp envelope skin.",
    )
    ctx.allow_overlap(
        "left_stabilizer",
        "hull",
        elem_a="root_fairing",
        elem_b="envelope",
        reason="The left tailplane root is intentionally blended into the envelope skin.",
    )
    ctx.allow_overlap(
        "right_stabilizer",
        "hull",
        elem_a="root_fairing",
        elem_b="envelope",
        reason="The right tailplane root is intentionally blended into the envelope skin.",
    )
    ctx.allow_overlap(
        "left_pylon",
        "hull",
        elem_a="root_pad",
        elem_b="envelope",
        reason="The left engine pylon root pad is intentionally embedded into the envelope mount point.",
    )
    ctx.allow_overlap(
        "right_pylon",
        "hull",
        elem_a="root_pad",
        elem_b="envelope",
        reason="The right engine pylon root pad is intentionally embedded into the envelope mount point.",
    )

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    left_pylon = object_model.get_part("left_pylon")
    right_pylon = object_model.get_part("right_pylon")
    left_engine = object_model.get_part("left_engine")
    right_engine = object_model.get_part("right_engine")
    left_prop = object_model.get_part("left_prop")
    right_prop = object_model.get_part("right_prop")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    left_engine_vector = object_model.get_articulation("left_engine_vector")
    right_engine_vector = object_model.get_articulation("right_engine_vector")
    left_prop_spin = object_model.get_articulation("left_prop_spin")
    right_prop_spin = object_model.get_articulation("right_prop_spin")
    top_fin_to_rudder = object_model.get_articulation("top_fin_to_rudder")
    left_elevator_hinge = object_model.get_articulation("left_elevator_hinge")
    right_elevator_hinge = object_model.get_articulation("right_elevator_hinge")

    ctx.expect_gap(
        hull,
        gondola,
        axis="z",
        positive_elem="envelope",
        negative_elem="cabin",
        min_gap=0.35,
        max_gap=0.80,
        name="gondola hangs clearly below the cigar envelope",
    )
    ctx.expect_within(
        gondola,
        hull,
        axes="xy",
        inner_elem="cabin",
        outer_elem="envelope",
        margin=0.25,
        name="gondola stays under the hull centerline footprint",
    )
    ctx.expect_contact(
        left_engine,
        left_pylon,
        elem_a="mount_pad",
        elem_b="pivot_cap",
        name="left engine pod is carried by the left pylon",
    )
    ctx.expect_contact(
        right_engine,
        right_pylon,
        elem_a="mount_pad",
        elem_b="pivot_cap",
        name="right engine pod is carried by the right pylon",
    )

    for joint in (left_prop_spin, right_prop_spin):
        ctx.check(
            f"{joint.name} stays a continuous shaft spin",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None,
            details=(
                f"type={joint.articulation_type}, axis={joint.axis}, "
                f"limits={joint.motion_limits}"
            ),
        )

    left_rest = _aabb_center(ctx.part_element_world_aabb(left_prop, elem="hub"))
    right_rest = _aabb_center(ctx.part_element_world_aabb(right_prop, elem="hub"))
    left_vector_upper = (
        left_engine_vector.motion_limits.upper
        if left_engine_vector.motion_limits is not None
        else None
    )
    right_vector_upper = (
        right_engine_vector.motion_limits.upper
        if right_engine_vector.motion_limits is not None
        else None
    )
    if left_rest is not None and right_rest is not None and left_vector_upper is not None and right_vector_upper is not None:
        with ctx.pose({left_engine_vector: left_vector_upper, right_engine_vector: right_vector_upper}):
            left_raised = _aabb_center(ctx.part_element_world_aabb(left_prop, elem="hub"))
            right_raised = _aabb_center(ctx.part_element_world_aabb(right_prop, elem="hub"))
        ctx.check(
            "engine pods vector upward at positive command",
            left_raised is not None
            and right_raised is not None
            and left_raised[2] > left_rest[2] + 0.12
            and right_raised[2] > right_rest[2] + 0.12,
            details=(
                f"left_rest={left_rest}, left_raised={left_raised}, "
                f"right_rest={right_rest}, right_raised={right_raised}"
            ),
        )

    rudder_rest = _aabb_center(ctx.part_element_world_aabb(rudder, elem="rudder_surface"))
    rudder_upper = (
        top_fin_to_rudder.motion_limits.upper
        if top_fin_to_rudder.motion_limits is not None
        else None
    )
    if rudder_rest is not None and rudder_upper is not None:
        with ctx.pose({top_fin_to_rudder: rudder_upper}):
            rudder_deflected = _aabb_center(
                ctx.part_element_world_aabb(rudder, elem="rudder_surface")
            )
        ctx.check(
            "rudder swings about the trailing hinge line",
            rudder_deflected is not None and rudder_deflected[1] < rudder_rest[1] - 0.06,
            details=f"rest={rudder_rest}, deflected={rudder_deflected}",
        )

    left_elevator_rest = _aabb_center(
        ctx.part_element_world_aabb(left_elevator, elem="elevator_surface")
    )
    right_elevator_rest = _aabb_center(
        ctx.part_element_world_aabb(right_elevator, elem="elevator_surface")
    )
    left_elevator_upper = (
        left_elevator_hinge.motion_limits.upper
        if left_elevator_hinge.motion_limits is not None
        else None
    )
    right_elevator_upper = (
        right_elevator_hinge.motion_limits.upper
        if right_elevator_hinge.motion_limits is not None
        else None
    )
    if (
        left_elevator_rest is not None
        and right_elevator_rest is not None
        and left_elevator_upper is not None
        and right_elevator_upper is not None
    ):
        with ctx.pose(
            {
                left_elevator_hinge: left_elevator_upper,
                right_elevator_hinge: right_elevator_upper,
            }
        ):
            left_elevator_up = _aabb_center(
                ctx.part_element_world_aabb(left_elevator, elem="elevator_surface")
            )
            right_elevator_up = _aabb_center(
                ctx.part_element_world_aabb(right_elevator, elem="elevator_surface")
            )
        ctx.check(
            "horizontal control surfaces lift at positive deflection",
            left_elevator_up is not None
            and right_elevator_up is not None
            and left_elevator_up[2] > left_elevator_rest[2] + 0.03
            and right_elevator_up[2] > right_elevator_rest[2] + 0.03,
            details=(
                f"left_rest={left_elevator_rest}, left_up={left_elevator_up}, "
                f"right_rest={right_elevator_rest}, right_up={right_elevator_up}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
