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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_barrel_shell_mesh():
    outer_profile = [
        (0.0215, 0.000),
        (0.0215, 0.118),
        (0.0245, 0.132),
        (0.0170, 0.145),
        (0.0090, 0.164),
        (0.0065, 0.182),
    ]
    inner_profile = [
        (0.0180, 0.000),
        (0.0180, 0.126),
        (0.0125, 0.140),
        (0.0040, 0.166),
        (0.0025, 0.182),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, "safety_syringe_barrel_shell")


def _build_ring_mesh(*, outer_radius: float, inner_radius: float, length: float, name: str):
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, length)],
        [(inner_radius, 0.0), (inner_radius, length)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_syringe")

    frame_steel = model.material("frame_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    plunger_metal = model.material("plunger_metal", rgba=(0.47, 0.50, 0.54, 1.0))
    clear_poly = model.material("clear_poly", rgba=(0.74, 0.84, 0.92, 0.35))
    seal_black = model.material("seal_black", rgba=(0.12, 0.12, 0.13, 1.0))
    safety_red = model.material("safety_red", rgba=(0.75, 0.14, 0.10, 1.0))
    mark_black = model.material("mark_black", rgba=(0.08, 0.09, 0.10, 1.0))

    barrel_shell_mesh = _build_barrel_shell_mesh()
    front_stop_ring_mesh = _build_ring_mesh(
        outer_radius=0.0180,
        inner_radius=0.0135,
        length=0.0040,
        name="safety_syringe_front_stop_ring",
    )

    body = model.part("body")
    body.visual(barrel_shell_mesh, material=clear_poly, name="barrel_shell")
    body.visual(
        front_stop_ring_mesh,
        origin=Origin(xyz=(0.127, 0.0, 0.0)),
        material=frame_steel,
        name="front_stop_ring",
    )
    body.visual(
        Box((0.342, 0.024, 0.010)),
        origin=Origin(xyz=(-0.021, 0.0, 0.050)),
        material=frame_steel,
        name="top_rail",
    )
    body.visual(
        Box((0.342, 0.024, 0.010)),
        origin=Origin(xyz=(-0.021, 0.0, -0.050)),
        material=frame_steel,
        name="bottom_rail",
    )
    body.visual(
        Box((0.016, 0.070, 0.120)),
        origin=Origin(xyz=(-0.200, 0.0, 0.0)),
        material=frame_steel,
        name="rear_stop_bulkhead",
    )
    body.visual(
        Box((0.020, 0.070, 0.110)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material=frame_steel,
        name="front_support_block",
    )
    body.visual(
        Box((0.020, 0.090, 0.012)),
        origin=Origin(xyz=(-0.200, 0.0, 0.044)),
        material=frame_steel,
        name="rear_upper_web",
    )
    body.visual(
        Box((0.020, 0.090, 0.012)),
        origin=Origin(xyz=(-0.200, 0.0, -0.044)),
        material=frame_steel,
        name="rear_lower_web",
    )
    body.visual(
        Box((0.020, 0.070, 0.012)),
        origin=Origin(xyz=(0.132, 0.0, 0.044)),
        material=frame_steel,
        name="front_upper_web",
    )
    body.visual(
        Box((0.020, 0.070, 0.012)),
        origin=Origin(xyz=(0.132, 0.0, -0.044)),
        material=frame_steel,
        name="front_lower_web",
    )
    body.visual(
        Box((0.020, 0.014, 0.090)),
        origin=Origin(xyz=(0.132, 0.028, 0.0)),
        material=frame_steel,
        name="front_clamp_right",
    )
    body.visual(
        Box((0.020, 0.014, 0.090)),
        origin=Origin(xyz=(0.132, -0.028, 0.0)),
        material=frame_steel,
        name="front_clamp_left",
    )
    body.visual(
        Box((0.028, 0.028, 0.012)),
        origin=Origin(xyz=(-0.210, 0.049, -0.028)),
        material=frame_steel,
        name="right_finger_tab_stem",
    )
    body.visual(
        Box((0.028, 0.028, 0.012)),
        origin=Origin(xyz=(-0.210, -0.049, -0.028)),
        material=frame_steel,
        name="left_finger_tab_stem",
    )
    body.visual(
        Box((0.028, 0.034, 0.012)),
        origin=Origin(xyz=(-0.224, 0.076, -0.028)),
        material=frame_steel,
        name="right_finger_tab",
    )
    body.visual(
        Box((0.028, 0.034, 0.012)),
        origin=Origin(xyz=(-0.224, -0.076, -0.028)),
        material=frame_steel,
        name="left_finger_tab",
    )
    body.visual(
        Box((0.020, 0.020, 0.040)),
        origin=Origin(xyz=(-0.200, 0.0, 0.080)),
        material=frame_steel,
        name="lockout_post",
    )
    body.visual(
        Box((0.032, 0.078, 0.012)),
        origin=Origin(xyz=(-0.200, 0.0, 0.106)),
        material=frame_steel,
        name="lock_pin_bridge",
    )
    body.visual(
        Box((0.016, 0.016, 0.036)),
        origin=Origin(xyz=(-0.200, 0.031, 0.124)),
        material=safety_red,
        name="right_lockout_ear",
    )
    body.visual(
        Box((0.016, 0.016, 0.036)),
        origin=Origin(xyz=(-0.200, -0.031, 0.124)),
        material=safety_red,
        name="left_lockout_ear",
    )
    body.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(-0.200, 0.018, 0.116)),
        material=frame_steel,
        name="lock_pin_clip_right",
    )
    body.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(-0.200, -0.018, 0.116)),
        material=frame_steel,
        name="lock_pin_clip_left",
    )
    body.visual(
        Cylinder(radius=0.0026, length=0.046),
        origin=Origin(xyz=(-0.200, 0.0, 0.124), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_red,
        name="stored_lock_pin",
    )

    for index in range(9):
        x_pos = 0.032 + 0.010 * index
        mark_width = 0.010 if index % 4 == 0 else 0.006
        mark_thickness = 0.0012
        body.visual(
            Box((mark_thickness, mark_width, 0.0008)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0216)),
            material=mark_black,
            name=f"graduation_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.470, 0.180, 0.170)),
        mass=1.8,
        origin=Origin(xyz=(-0.040, 0.0, 0.01)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0045, length=0.188),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_metal,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0162, length=0.008),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_black,
        name="piston_head",
    )
    plunger.visual(
        Cylinder(radius=0.0138, length=0.004),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_metal,
        name="piston_backup_plate",
    )
    plunger.visual(
        Box((0.016, 0.078, 0.072)),
        origin=Origin(xyz=(-0.184, 0.0, 0.0)),
        material=safety_red,
        name="rear_stop_plate",
    )
    plunger.visual(
        Box((0.110, 0.070, 0.010)),
        origin=Origin(xyz=(-0.110, 0.0, 0.040)),
        material=plunger_metal,
        name="top_guide_shoe",
    )
    plunger.visual(
        Box((0.110, 0.070, 0.010)),
        origin=Origin(xyz=(-0.110, 0.0, -0.040)),
        material=plunger_metal,
        name="bottom_guide_shoe",
    )
    plunger.visual(
        Box((0.100, 0.012, 0.080)),
        origin=Origin(xyz=(-0.145, 0.034, 0.0)),
        material=plunger_metal,
        name="right_side_web",
    )
    plunger.visual(
        Box((0.100, 0.012, 0.080)),
        origin=Origin(xyz=(-0.145, -0.034, 0.0)),
        material=plunger_metal,
        name="left_side_web",
    )
    plunger.visual(
        Box((0.020, 0.082, 0.070)),
        origin=Origin(xyz=(-0.177, 0.0, 0.0)),
        material=plunger_metal,
        name="rear_carriage_bridge",
    )
    plunger.visual(
        Box((0.068, 0.012, 0.012)),
        origin=Origin(xyz=(-0.226, 0.044, 0.0)),
        material=plunger_metal,
        name="right_handle_strut",
    )
    plunger.visual(
        Box((0.068, 0.012, 0.012)),
        origin=Origin(xyz=(-0.226, -0.044, 0.0)),
        material=plunger_metal,
        name="left_handle_strut",
    )
    plunger.visual(
        Box((0.030, 0.130, 0.018)),
        origin=Origin(xyz=(-0.274, 0.0, 0.0)),
        material=plunger_metal,
        name="handle_bar",
    )

    for y_pos in (-0.046, 0.046):
        plunger.visual(
            Cylinder(radius=0.0026, length=0.014),
            origin=Origin(
                xyz=(-0.274, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=fastener_steel,
        )

    plunger.inertial = Inertial.from_geometry(
        Box((0.330, 0.140, 0.100)),
        mass=0.6,
        origin=Origin(xyz=(-0.155, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=0.105,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("body_to_plunger")
    stroke = slide.motion_limits.upper if slide.motion_limits and slide.motion_limits.upper else 0.0

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(max_pose_samples=5)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.expect_origin_distance(
        body,
        plunger,
        axes="yz",
        min_dist=0.0,
        max_dist=1e-6,
        name="plunger_remains_coaxial_with_barrel",
    )

    open_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            plunger,
            body,
            axis="x",
            positive_elem="rear_stop_plate",
            negative_elem="rear_stop_bulkhead",
            max_gap=0.0005,
            max_penetration=0.0,
            name="rear_overtravel_stop_seats_at_retracted_limit",
        )

    with ctx.pose({slide: 0.5 * stroke}):
        ctx.expect_contact(
            plunger,
            body,
            elem_a="top_guide_shoe",
            elem_b="top_rail",
            name="top_guide_shoe_stays_supported_midstroke",
        )
        ctx.expect_contact(
            plunger,
            body,
            elem_a="bottom_guide_shoe",
            elem_b="bottom_rail",
            name="bottom_guide_shoe_stays_supported_midstroke",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_midstroke")

    with ctx.pose({slide: stroke}):
        ctx.expect_gap(
            body,
            plunger,
            axis="x",
            positive_elem="front_stop_ring",
            negative_elem="piston_head",
            max_gap=0.0005,
            max_penetration=0.0,
            name="front_overtravel_stop_seats_at_full_insert",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_insert")
        closed_pos = ctx.part_world_position(plunger)

    if open_pos is None or closed_pos is None:
        ctx.fail("plunger_motion_positions_resolve", "Unable to resolve plunger world positions.")
    else:
        ctx.check(
            "plunger_advances_forward_through_stroke",
            (closed_pos[0] - open_pos[0]) > 0.100,
            details=(
                f"Expected forward plunger travel close to stroke. "
                f"Open x={open_pos[0]:.4f}, closed x={closed_pos[0]:.4f}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
