from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


ENVELOPE_SECTIONS = (
    (-26.5, 0.14, 0.18),
    (-21.0, 2.50, 2.90),
    (-13.0, 5.55, 6.05),
    (-5.0, 6.25, 6.55),
    (0.0, 6.30, 6.60),
    (5.0, 6.20, 6.45),
    (14.0, 4.95, 5.30),
    (21.0, 3.00, 3.25),
    (26.5, 0.10, 0.14),
)

CABIN_CENTER_Z = -9.45
CABIN_HALF_WIDTH = 1.75
CABIN_HALF_HEIGHT = 1.65
CABIN_ROOF_Z = CABIN_CENTER_Z + CABIN_HALF_HEIGHT


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _ellipse_loop(
    x_pos: float,
    y_radius: float,
    z_radius: float,
    *,
    points: int = 28,
) -> list[tuple[float, float, float]]:
    loop: list[tuple[float, float, float]] = []
    for index in range(points):
        theta = 2.0 * math.pi * index / points
        loop.append((x_pos, y_radius * math.cos(theta), z_radius * math.sin(theta)))
    return loop


def _rounded_rect_loop(
    x_pos: float,
    half_width: float,
    half_height: float,
    corner_radius: float,
    *,
    arc_segments: int = 5,
) -> list[tuple[float, float, float]]:
    straight_y = max(half_width - corner_radius, 0.0)
    straight_z = max(half_height - corner_radius, 0.0)
    corners = (
        (straight_y, straight_z, 0.0),
        (-straight_y, straight_z, math.pi / 2.0),
        (-straight_y, -straight_z, math.pi),
        (straight_y, -straight_z, 3.0 * math.pi / 2.0),
    )
    loop: list[tuple[float, float, float]] = []
    for cy, cz, start_angle in corners:
        for step in range(arc_segments):
            angle = start_angle + 0.5 * math.pi * step / arc_segments
            loop.append(
                (
                    x_pos,
                    cy + corner_radius * math.cos(angle),
                    cz + corner_radius * math.sin(angle),
                )
            )
    return loop


def _envelope_radii_at_x(x_pos: float) -> tuple[float, float]:
    for index in range(len(ENVELOPE_SECTIONS) - 1):
        x0, y0, z0 = ENVELOPE_SECTIONS[index]
        x1, y1, z1 = ENVELOPE_SECTIONS[index + 1]
        if x0 <= x_pos <= x1:
            t = 0.0 if abs(x1 - x0) < 1e-9 else (x_pos - x0) / (x1 - x0)
            return (_lerp(y0, y1, t), _lerp(z0, z1, t))
    _, y_last, z_last = ENVELOPE_SECTIONS[-1]
    return (y_last, z_last)


def _envelope_surface_point(x_pos: float, y_pos: float) -> tuple[float, float, float]:
    y_radius, z_radius = _envelope_radii_at_x(x_pos)
    ratio = max(-1.0, min(1.0, y_pos / y_radius))
    z_pos = -z_radius * math.sqrt(max(0.0, 1.0 - ratio * ratio))
    return (x_pos, y_pos, z_pos)


def _build_envelope_geometry():
    return section_loft([_ellipse_loop(x, y, z) for x, y, z in ENVELOPE_SECTIONS])


def _build_cabin_geometry():
    cabin_sections = [
        _rounded_rect_loop(-8.0, 0.55, 0.55, 0.20),
        _rounded_rect_loop(-6.0, 1.20, 1.10, 0.28),
        _rounded_rect_loop(-2.0, CABIN_HALF_WIDTH, CABIN_HALF_HEIGHT, 0.34),
        _rounded_rect_loop(2.0, CABIN_HALF_WIDTH, CABIN_HALF_HEIGHT, 0.34),
        _rounded_rect_loop(6.0, 1.35, 1.18, 0.28),
        _rounded_rect_loop(8.0, 0.42, 0.48, 0.18),
    ]
    return section_loft(cabin_sections)


def _surface_section_loop(
    lead_x: float,
    trail_x: float,
    thickness: float,
    *,
    axis_value: float,
    axis_name: str,
) -> list[tuple[float, float, float]]:
    mid_a = lead_x * 0.64 + trail_x * 0.36
    mid_b = lead_x * 0.26 + trail_x * 0.74
    profile_2d = [
        (lead_x, 0.0),
        (mid_a, 0.60 * thickness),
        (mid_b, 0.34 * thickness),
        (trail_x, 0.12 * thickness),
        (trail_x, -0.12 * thickness),
        (mid_b, -0.34 * thickness),
        (mid_a, -0.60 * thickness),
        (lead_x, 0.0),
    ]
    if axis_name == "y":
        return [(x_pos, axis_value, z_pos) for x_pos, z_pos in profile_2d]
    return [(x_pos, y_pos, axis_value) for x_pos, y_pos in profile_2d]


def _horizontal_surface_geometry(
    *,
    side_sign: float,
    root_span: float,
    tip_span: float,
    root_lead: float,
    root_trail: float,
    tip_lead: float,
    tip_trail: float,
    root_thickness: float,
    tip_thickness: float,
):
    span_positions = (root_span, 0.55 * tip_span, tip_span)
    loops = []
    for span_pos in span_positions:
        t = 0.0 if abs(tip_span - root_span) < 1e-9 else (span_pos - root_span) / (tip_span - root_span)
        loops.append(
            _surface_section_loop(
                _lerp(root_lead, tip_lead, t),
                _lerp(root_trail, tip_trail, t),
                _lerp(root_thickness, tip_thickness, t),
                axis_value=side_sign * span_pos,
                axis_name="y",
            )
        )
    return section_loft(loops)


def _vertical_surface_geometry(
    *,
    sign: float,
    root_span: float,
    tip_span: float,
    root_lead: float,
    root_trail: float,
    tip_lead: float,
    tip_trail: float,
    root_thickness: float,
    tip_thickness: float,
):
    span_positions = (root_span, 0.55 * tip_span, tip_span)
    loops = []
    for span_pos in span_positions:
        t = 0.0 if abs(tip_span - root_span) < 1e-9 else (span_pos - root_span) / (tip_span - root_span)
        loops.append(
            _surface_section_loop(
                _lerp(root_lead, tip_lead, t),
                _lerp(root_trail, tip_trail, t),
                _lerp(root_thickness, tip_thickness, t),
                axis_value=sign * span_pos,
                axis_name="z",
            )
        )
    return section_loft(loops)


def _build_pod_geometry():
    pod_sections = [
        _ellipse_loop(-0.55, 0.08, 0.10, points=18),
        _ellipse_loop(-0.10, 0.26, 0.28, points=18),
        _ellipse_loop(0.85, 0.41, 0.43, points=18),
        _ellipse_loop(1.55, 0.38, 0.40, points=18),
        _ellipse_loop(2.35, 0.20, 0.22, points=18),
        _ellipse_loop(2.75, 0.04, 0.05, points=18),
    ]
    return section_loft([[(x_pos, y_pos, z_pos) for x_pos, y_pos, z_pos in loop] for loop in pod_sections])


def _add_support_visual(
    part,
    name: str,
    *,
    local_xy: tuple[float, float],
    bottom_z: float,
    top_z: float,
    material,
    radius: float = 0.11,
):
    total_length = top_z - bottom_z
    cap_radius = radius
    stem_length = max(total_length - 2.0 * cap_radius, 0.02)
    x_pos, y_pos = local_xy
    part.visual(
        Sphere(radius=cap_radius),
        origin=Origin(xyz=(x_pos, y_pos, top_z - cap_radius)),
        material=material,
        name=f"{name}_top_cap",
    )
    part.visual(
        Cylinder(radius=radius * 0.92, length=stem_length),
        origin=Origin(xyz=(x_pos, y_pos, 0.5 * (bottom_z + top_z))),
        material=material,
        name=f"{name}_stem",
    )
    part.visual(
        Sphere(radius=cap_radius),
        origin=Origin(xyz=(x_pos, y_pos, bottom_z + cap_radius)),
        material=material,
        name=f"{name}_bottom_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_blimp")

    envelope_fabric = model.material("envelope_fabric", rgba=(0.88, 0.90, 0.92, 1.0))
    cabin_paint = model.material("cabin_paint", rgba=(0.31, 0.34, 0.38, 1.0))
    support_gray = model.material("support_gray", rgba=(0.57, 0.60, 0.63, 1.0))
    fin_gray = model.material("fin_gray", rgba=(0.74, 0.77, 0.80, 1.0))
    pod_gray = model.material("pod_gray", rgba=(0.45, 0.48, 0.52, 1.0))
    prop_black = model.material("prop_black", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.52, 0.69, 0.83, 0.42))

    envelope = model.part("envelope")
    envelope.visual(
        mesh_from_geometry(_build_envelope_geometry(), "envelope_shell"),
        material=envelope_fabric,
        name="envelope_shell",
    )

    cabin = model.part("cabin")
    cabin.visual(
        mesh_from_geometry(_build_cabin_geometry(), "cabin_shell"),
        material=cabin_paint,
        name="cabin_shell",
    )
    cabin.visual(
        Box((13.4, 1.70, 0.22)),
        origin=Origin(xyz=(0.2, 0.0, 1.32)),
        material=cabin_paint,
        name="cabin_roof_spine",
    )
    cabin.visual(
        Box((10.8, 0.12, 0.72)),
        origin=Origin(xyz=(0.1, 1.71, 0.50)),
        material=glass,
        name="left_window_band",
    )
    cabin.visual(
        Box((10.8, 0.12, 0.72)),
        origin=Origin(xyz=(0.1, -1.71, 0.50)),
        material=glass,
        name="right_window_band",
    )
    cabin.visual(
        Box((0.34, 1.18, 0.84)),
        origin=Origin(xyz=(7.58, 0.0, 0.56)),
        material=glass,
        name="front_glazing",
    )

    _add_support_visual(cabin, "support_0", local_xy=(-4.2, 1.08), bottom_z=1.26, top_z=3.06, material=support_gray)
    _add_support_visual(cabin, "support_1", local_xy=(-4.2, -1.08), bottom_z=1.26, top_z=3.06, material=support_gray)
    _add_support_visual(cabin, "support_2", local_xy=(4.2, 1.08), bottom_z=1.26, top_z=3.01, material=support_gray)
    _add_support_visual(cabin, "support_3", local_xy=(4.2, -1.08), bottom_z=1.26, top_z=3.01, material=support_gray)
    cabin.visual(
        Cylinder(radius=0.16, length=1.50),
        origin=Origin(xyz=(1.10, 2.48, 0.55), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=support_gray,
        name="left_pylon",
    )
    cabin.visual(
        Cylinder(radius=0.16, length=1.50),
        origin=Origin(xyz=(1.10, -2.48, 0.55), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=support_gray,
        name="right_pylon",
    )

    model.articulation(
        "envelope_to_cabin",
        ArticulationType.FIXED,
        parent=envelope,
        child=cabin,
        origin=Origin(xyz=(0.0, 0.0, CABIN_CENTER_Z)),
    )

    left_pod = model.part("left_pod")
    left_pod.visual(
        mesh_from_geometry(_build_pod_geometry(), "left_pod_shell"),
        material=pod_gray,
        name="pod_shell",
    )
    right_pod = model.part("right_pod")
    right_pod.visual(
        mesh_from_geometry(_build_pod_geometry(), "right_pod_shell"),
        material=pod_gray,
        name="pod_shell",
    )

    model.articulation(
        "cabin_to_left_pod",
        ArticulationType.FIXED,
        parent=cabin,
        child=left_pod,
        origin=Origin(xyz=(1.10, 3.80, 0.55), rpy=(0.0, 0.0, math.pi / 2.0)),
    )
    model.articulation(
        "cabin_to_right_pod",
        ArticulationType.FIXED,
        parent=cabin,
        child=right_pod,
        origin=Origin(xyz=(1.10, -3.80, 0.55), rpy=(0.0, 0.0, -math.pi / 2.0)),
    )

    for prop_name, parent_part in (("left_propeller", left_pod), ("right_propeller", right_pod)):
        propeller = model.part(prop_name)
        propeller.visual(
            Cylinder(radius=0.16, length=0.30),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=prop_black,
            name="hub",
        )
        propeller.visual(
            Sphere(radius=0.18),
            origin=Origin(xyz=(0.06, 0.0, 0.0)),
            material=pod_gray,
            name="spinner",
        )
        for blade_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
            propeller.visual(
                Box((0.06, 1.85, 0.24)),
                origin=Origin(
                    xyz=(0.0, 0.94 * math.cos(angle), 0.94 * math.sin(angle)),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=prop_black,
                name=f"blade_{blade_index}",
            )
        model.articulation(
            f"{parent_part.name}_to_{prop_name}",
            ArticulationType.CONTINUOUS,
            parent=parent_part,
            child=propeller,
            origin=Origin(xyz=(2.86, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=28.0),
        )

    fin_specs = [
        (
            "top_fin",
            _vertical_surface_geometry(
                sign=1.0,
                root_span=-0.18,
                tip_span=4.10,
                root_lead=0.82,
                root_trail=-2.55,
                tip_lead=0.30,
                tip_trail=-1.35,
                root_thickness=0.28,
                tip_thickness=0.12,
            ),
            Origin(xyz=(23.8, 0.0, 1.55)),
        ),
        (
            "bottom_fin",
            _vertical_surface_geometry(
                sign=-1.0,
                root_span=-0.18,
                tip_span=4.10,
                root_lead=0.82,
                root_trail=-2.55,
                tip_lead=0.30,
                tip_trail=-1.35,
                root_thickness=0.28,
                tip_thickness=0.12,
            ),
            Origin(xyz=(23.8, 0.0, -1.55)),
        ),
        (
            "left_fin",
            _horizontal_surface_geometry(
                side_sign=1.0,
                root_span=-0.18,
                tip_span=4.75,
                root_lead=0.94,
                root_trail=-2.65,
                tip_lead=0.38,
                tip_trail=-1.50,
                root_thickness=0.22,
                tip_thickness=0.10,
            ),
            Origin(xyz=(23.6, 1.58, 0.0)),
        ),
        (
            "right_fin",
            _horizontal_surface_geometry(
                side_sign=-1.0,
                root_span=-0.18,
                tip_span=4.75,
                root_lead=0.94,
                root_trail=-2.65,
                tip_lead=0.38,
                tip_trail=-1.50,
                root_thickness=0.22,
                tip_thickness=0.10,
            ),
            Origin(xyz=(23.6, -1.58, 0.0)),
        ),
    ]
    for fin_name, fin_geom, fin_origin in fin_specs:
        fin = model.part(fin_name)
        fin.visual(
            mesh_from_geometry(fin_geom, f"{fin_name}_shell"),
            material=fin_gray,
            name="fin_shell",
        )
        model.articulation(
            f"envelope_to_{fin_name}",
            ArticulationType.FIXED,
            parent=envelope,
            child=fin,
            origin=fin_origin,
        )

    top_rudder = model.part("top_rudder")
    top_rudder.visual(
        mesh_from_geometry(
            _vertical_surface_geometry(
                sign=1.0,
                root_span=-0.12,
                tip_span=4.00,
                root_lead=0.0,
                root_trail=-1.45,
                tip_lead=0.0,
                tip_trail=-0.86,
                root_thickness=0.16,
                tip_thickness=0.07,
            ),
            "top_rudder_shell",
        ),
        material=fin_gray,
        name="surface_shell",
    )
    bottom_rudder = model.part("bottom_rudder")
    bottom_rudder.visual(
        mesh_from_geometry(
            _vertical_surface_geometry(
                sign=-1.0,
                root_span=-0.12,
                tip_span=4.00,
                root_lead=0.0,
                root_trail=-1.45,
                tip_lead=0.0,
                tip_trail=-0.86,
                root_thickness=0.16,
                tip_thickness=0.07,
            ),
            "bottom_rudder_shell",
        ),
        material=fin_gray,
        name="surface_shell",
    )
    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        mesh_from_geometry(
            _horizontal_surface_geometry(
                side_sign=1.0,
                root_span=-0.12,
                tip_span=4.65,
                root_lead=0.0,
                root_trail=-1.55,
                tip_lead=0.0,
                tip_trail=-0.94,
                root_thickness=0.14,
                tip_thickness=0.06,
            ),
            "left_elevator_shell",
        ),
        material=fin_gray,
        name="surface_shell",
    )
    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        mesh_from_geometry(
            _horizontal_surface_geometry(
                side_sign=-1.0,
                root_span=-0.12,
                tip_span=4.65,
                root_lead=0.0,
                root_trail=-1.55,
                tip_lead=0.0,
                tip_trail=-0.94,
                root_thickness=0.14,
                tip_thickness=0.06,
            ),
            "right_elevator_shell",
        ),
        material=fin_gray,
        name="surface_shell",
    )

    model.articulation(
        "top_fin_to_top_rudder",
        ArticulationType.REVOLUTE,
        parent="top_fin",
        child=top_rudder,
        origin=Origin(xyz=(-2.55, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "bottom_fin_to_bottom_rudder",
        ArticulationType.REVOLUTE,
        parent="bottom_fin",
        child=bottom_rudder,
        origin=Origin(xyz=(-2.55, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "left_fin_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent="left_fin",
        child=left_elevator,
        origin=Origin(xyz=(-2.65, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "right_fin_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent="right_fin",
        child=right_elevator,
        origin=Origin(xyz=(-2.65, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    envelope = object_model.get_part("envelope")
    cabin = object_model.get_part("cabin")
    left_pod = object_model.get_part("left_pod")
    right_pod = object_model.get_part("right_pod")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    top_fin = object_model.get_part("top_fin")
    bottom_fin = object_model.get_part("bottom_fin")
    left_fin = object_model.get_part("left_fin")
    right_fin = object_model.get_part("right_fin")
    top_rudder = object_model.get_part("top_rudder")
    bottom_rudder = object_model.get_part("bottom_rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    top_rudder_joint = object_model.get_articulation("top_fin_to_top_rudder")
    bottom_rudder_joint = object_model.get_articulation("bottom_fin_to_bottom_rudder")
    left_elevator_joint = object_model.get_articulation("left_fin_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("right_fin_to_right_elevator")

    for elem_name in (
        "support_0_top_cap",
        "support_1_top_cap",
        "support_2_top_cap",
        "support_3_top_cap",
    ):
        ctx.allow_overlap(
            cabin,
            envelope,
            elem_a=elem_name,
            elem_b="envelope_shell",
            reason="The cabin support fairings intentionally nest slightly into the envelope skin to represent reinforced attachment sockets.",
        )

    ctx.allow_overlap(
        cabin,
        left_pod,
        elem_a="left_pylon",
        elem_b="pod_shell",
        reason="The left nacelle mounts on a partially faired pylon that nests slightly into the pod root shell.",
    )
    ctx.allow_overlap(
        cabin,
        right_pod,
        elem_a="right_pylon",
        elem_b="pod_shell",
        reason="The right nacelle mounts on a partially faired pylon that nests slightly into the pod root shell.",
    )
    ctx.allow_overlap(
        left_pod,
        left_propeller,
        elem_a="pod_shell",
        elem_b="hub",
        reason="The left propeller hub is intentionally seated partly inside the pod nose fairing.",
    )
    ctx.allow_overlap(
        left_pod,
        left_propeller,
        elem_a="pod_shell",
        elem_b="spinner",
        reason="The left spinner slightly enters the simplified pod lip to represent a tight spinner-to-cowl fairing.",
    )
    ctx.allow_overlap(
        right_pod,
        right_propeller,
        elem_a="pod_shell",
        elem_b="hub",
        reason="The right propeller hub is intentionally seated partly inside the pod nose fairing.",
    )
    ctx.allow_overlap(
        right_pod,
        right_propeller,
        elem_a="pod_shell",
        elem_b="spinner",
        reason="The right spinner slightly enters the simplified pod lip to represent a tight spinner-to-cowl fairing.",
    )

    for fin_part in (top_fin, bottom_fin, left_fin, right_fin):
        ctx.allow_overlap(
            envelope,
            fin_part,
            elem_a="envelope_shell",
            elem_b="fin_shell",
            reason="The fixed tailplane roots are intentionally blended into the airship envelope rather than separated by an artificial gap.",
        )
    for surface_part in (top_rudder, bottom_rudder, left_elevator, right_elevator):
        ctx.allow_overlap(
            envelope,
            surface_part,
            elem_a="envelope_shell",
            elem_b="surface_shell",
            reason="The modeled control-surface roots nest slightly into the tail-root fairing to avoid an unrealistic open slot at the envelope attachment.",
        )

    ctx.expect_gap(
        envelope,
        cabin,
        axis="z",
        min_gap=0.8,
        max_gap=1.6,
        positive_elem="envelope_shell",
        negative_elem="cabin_shell",
        name="envelope remains clearly above the passenger cabin",
    )
    ctx.expect_gap(
        left_pod,
        cabin,
        axis="y",
        min_gap=1.1,
        positive_elem="pod_shell",
        negative_elem="cabin_shell",
        name="left propeller pod stays clearly outboard of the cabin",
    )
    ctx.expect_gap(
        cabin,
        right_pod,
        axis="y",
        min_gap=1.1,
        positive_elem="cabin_shell",
        negative_elem="pod_shell",
        name="right propeller pod stays clearly outboard of the cabin",
    )

    rest_top_box = ctx.part_world_aabb(top_rudder)
    rest_bottom_box = ctx.part_world_aabb(bottom_rudder)
    rest_left_box = ctx.part_world_aabb(left_elevator)
    rest_right_box = ctx.part_world_aabb(right_elevator)

    upper_rudder = top_rudder_joint.motion_limits.upper if top_rudder_joint.motion_limits else None
    upper_bottom_rudder = bottom_rudder_joint.motion_limits.upper if bottom_rudder_joint.motion_limits else None
    upper_left_elevator = left_elevator_joint.motion_limits.upper if left_elevator_joint.motion_limits else None
    upper_right_elevator = right_elevator_joint.motion_limits.upper if right_elevator_joint.motion_limits else None

    if (
        upper_rudder is not None
        and upper_bottom_rudder is not None
        and upper_left_elevator is not None
        and upper_right_elevator is not None
    ):
        with ctx.pose(
            {
                top_rudder_joint: upper_rudder,
                bottom_rudder_joint: upper_bottom_rudder,
                left_elevator_joint: upper_left_elevator,
                right_elevator_joint: upper_right_elevator,
            }
        ):
            posed_top_box = ctx.part_world_aabb(top_rudder)
            posed_bottom_box = ctx.part_world_aabb(bottom_rudder)
            posed_left_box = ctx.part_world_aabb(left_elevator)
            posed_right_box = ctx.part_world_aabb(right_elevator)

        ctx.check(
            "rudders deflect toward positive y at upper limit",
            rest_top_box is not None
            and rest_bottom_box is not None
            and posed_top_box is not None
            and posed_bottom_box is not None
            and posed_top_box[1][1] > rest_top_box[1][1] + 0.08
            and posed_bottom_box[1][1] > rest_bottom_box[1][1] + 0.08,
            details=f"rest_top={rest_top_box}, posed_top={posed_top_box}, rest_bottom={rest_bottom_box}, posed_bottom={posed_bottom_box}",
        )
        ctx.check(
            "elevators rise at upper limit",
            rest_left_box is not None
            and rest_right_box is not None
            and posed_left_box is not None
            and posed_right_box is not None
            and posed_left_box[1][2] > rest_left_box[1][2] + 0.08
            and posed_right_box[1][2] > rest_right_box[1][2] + 0.08,
            details=f"rest_left={rest_left_box}, posed_left={posed_left_box}, rest_right={rest_right_box}, posed_right={posed_right_box}",
        )

    return ctx.report()


object_model = build_object_model()
