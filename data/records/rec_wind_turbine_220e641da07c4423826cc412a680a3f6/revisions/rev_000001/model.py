from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


TOWER_TOP_Z = 5.37
NACELLE_HUB_X = 1.20
NACELLE_HUB_Z = 0.45
HUB_SOCKET_X = 0.20
HUB_SOCKET_RADIUS = 0.48
BLADE_ROOT_LENGTH = 0.24
BLADE_LENGTH = 1.90


def _axis_cylinder_origin(
    xyz: tuple[float, float, float],
    *,
    axis: str,
) -> Origin:
    """Return an Origin for an SDK cylinder whose local +Z must point along an axis."""
    if axis == "x":
        return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))
    if axis == "y":
        return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))
    return Origin(xyz=xyz)


def _radial_unit(theta: float) -> tuple[float, float, float]:
    return (0.0, math.cos(theta), math.sin(theta))


def _radial_cylinder_origin(theta: float, radius: float, *, x: float = 0.0) -> Origin:
    v = _radial_unit(theta)
    return Origin(
        xyz=(x, v[1] * radius, v[2] * radius),
        rpy=(theta - math.pi / 2.0, 0.0, 0.0),
    )


def _airfoil_section(
    theta: float,
    span: float,
    chord: float,
    thickness: float,
    pitch: float,
) -> list[tuple[float, float, float]]:
    """Airfoil-ish blade section in the blade part frame.

    The blade span direction is radial in the hub YZ plane.  Chord lies mostly
    tangent to the rotor disk with a small axial pitch component so pitch joints
    visibly feather the airfoil around the load-carrying root axis.
    """
    radial = _radial_unit(theta)
    tangent = (0.0, -math.sin(theta), math.cos(theta))
    axial = (1.0, 0.0, 0.0)

    chord_vec = (
        axial[0] * math.sin(pitch) + tangent[0] * math.cos(pitch),
        axial[1] * math.sin(pitch) + tangent[1] * math.cos(pitch),
        axial[2] * math.sin(pitch) + tangent[2] * math.cos(pitch),
    )
    thick_vec = (
        axial[0] * math.cos(pitch) - tangent[0] * math.sin(pitch),
        axial[1] * math.cos(pitch) - tangent[1] * math.sin(pitch),
        axial[2] * math.cos(pitch) - tangent[2] * math.sin(pitch),
    )
    center = (radial[0] * span, radial[1] * span, radial[2] * span)

    # Ordered closed loop; chord coordinate first, thickness coordinate second.
    outline = (
        (-0.52, -0.08),
        (-0.36, 0.42),
        (-0.08, 0.62),
        (0.34, 0.36),
        (0.52, 0.04),
        (0.38, -0.28),
        (-0.04, -0.50),
        (-0.42, -0.30),
    )
    pts: list[tuple[float, float, float]] = []
    for c, t in outline:
        pts.append(
            (
                center[0] + chord_vec[0] * (c * chord) + thick_vec[0] * (t * thickness),
                center[1] + chord_vec[1] * (c * chord) + thick_vec[1] * (t * thickness),
                center[2] + chord_vec[2] * (c * chord) + thick_vec[2] * (t * thickness),
            )
        )
    return pts


def _blade_shell(theta: float, *, tip_only: bool = False):
    if tip_only:
        specs = (
            (1.50, 0.17, 0.038, 0.02),
            (1.70, 0.13, 0.030, -0.01),
            (1.90, 0.075, 0.020, -0.04),
        )
    else:
        specs = (
            (0.16, 0.44, 0.100, 0.22),
            (0.45, 0.38, 0.080, 0.16),
            (0.90, 0.27, 0.060, 0.08),
            (1.30, 0.21, 0.047, 0.04),
            (1.58, 0.16, 0.036, 0.01),
        )
    sections = [_airfoil_section(theta, *spec) for spec in specs]
    return repair_loft(section_loft(sections))


def _make_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wind_turbine")

    galvanized = model.material("galvanized_steel", rgba=(0.50, 0.54, 0.56, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    service_yellow = model.material("service_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    nacelle_paint = model.material("weathered_white", rgba=(0.82, 0.84, 0.80, 1.0))
    blade_white = model.material("blade_white", rgba=(0.91, 0.92, 0.88, 1.0))
    worn_orange = model.material("wear_orange", rgba=(0.95, 0.28, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.03, 0.035, 0.04, 1.0))

    tower = model.part("tower")
    tower.visual(Box((1.20, 1.20, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=dark_steel, name="base_plate")
    tower.visual(Cylinder(radius=0.22, length=5.20), origin=Origin(xyz=(0.0, 0.0, 2.65)), material=galvanized, name="tower_tube")
    tower.visual(Cylinder(radius=0.33, length=0.12), origin=Origin(xyz=(0.0, 0.0, 5.31)), material=dark_steel, name="lower_yaw_bearing")
    tower.visual(Cylinder(radius=0.46, length=0.08), origin=Origin(xyz=(0.0, 0.0, 5.33)), material=service_yellow, name="service_collar")
    for index, (x, y) in enumerate(((-0.42, -0.42), (0.42, -0.42), (-0.42, 0.42), (0.42, 0.42))):
        tower.visual(
            Cylinder(radius=0.075, length=0.035),
            origin=Origin(xyz=(x, y, 0.115)),
            material=dark_steel,
            name=f"anchor_bolt_{index}",
        )

    # Ladder and stand-off brackets are welded to the tower; every rung touches
    # both rails and the rails are tied back to the tube by chunky brackets.
    for rail_x in (-0.16, 0.16):
        tower.visual(Cylinder(radius=0.022, length=4.15), origin=Origin(xyz=(rail_x, -0.48, 2.75)), material=galvanized, name=f"ladder_rail_{rail_x:+.2f}")
    for idx, z in enumerate((0.82, 1.35, 1.88, 2.41, 2.94, 3.47, 4.00, 4.53)):
        tower.visual(
            Cylinder(radius=0.018, length=0.39),
            origin=_axis_cylinder_origin((0.0, -0.48, z), axis="x"),
            material=galvanized,
            name=f"ladder_rung_{idx}",
        )
    for idx, z in enumerate((0.90, 2.20, 3.50, 4.78)):
        for rail_x in (-0.16, 0.16):
            _make_box(tower, (0.055, 0.39, 0.055), (rail_x, -0.300, z), galvanized, f"ladder_standoff_{idx}_{0 if rail_x < 0 else 1}")

    nacelle = model.part("nacelle")
    nacelle.visual(Cylinder(radius=0.43, length=0.12), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=dark_steel, name="upper_yaw_bearing")
    nacelle.visual(Box((1.58, 0.84, 0.66)), origin=Origin(xyz=(0.26, 0.0, 0.45)), material=nacelle_paint, name="service_body")
    nacelle.visual(
        Cylinder(radius=0.35, length=0.20),
        origin=_axis_cylinder_origin((1.10, 0.0, NACELLE_HUB_Z), axis="x"),
        material=dark_steel,
        name="front_bearing",
    )
    nacelle.visual(
        Cylinder(radius=0.24, length=0.12),
        origin=_axis_cylinder_origin((-0.58, 0.0, 0.43), axis="x"),
        material=dark_steel,
        name="rear_cooling_port",
    )
    _make_box(nacelle, (0.62, 0.035, 0.44), (-0.18, -0.435, 0.45), service_yellow, "access_frame")
    _make_box(nacelle, (0.62, 0.045, 0.05), (0.26, -0.435, 0.73), dark_steel, "door_header")
    _make_box(nacelle, (0.90, 0.12, 0.08), (0.05, 0.0, 0.80), dark_steel, "roof_rail")
    _make_box(nacelle, (0.14, 0.92, 0.06), (0.88, 0.0, 0.80), service_yellow, "lift_lug")
    # Exposed hinge knuckles on the nacelle side, with a clear gap for the moving hatch knuckle.
    nacelle.visual(
        Cylinder(radius=0.028, length=0.13),
        origin=Origin(xyz=(-0.49, -0.485, 0.29)),
        material=dark_steel,
        name="hatch_fixed_knuckle_lower",
    )
    _make_box(nacelle, (0.055, 0.08, 0.10), (-0.49, -0.445, 0.29), dark_steel, "hatch_hinge_leaf_lower")
    nacelle.visual(
        Cylinder(radius=0.028, length=0.13),
        origin=Origin(xyz=(-0.49, -0.485, 0.61)),
        material=dark_steel,
        name="hatch_fixed_knuckle_upper",
    )
    _make_box(nacelle, (0.055, 0.08, 0.10), (-0.49, -0.445, 0.61), dark_steel, "hatch_hinge_leaf_upper")

    hatch = model.part("access_hatch")
    _make_box(hatch, (0.55, 0.035, 0.36), (0.275, -0.070, 0.0), nacelle_paint, "hatch_panel")
    _make_box(hatch, (0.09, 0.035, 0.05), (0.43, -0.095, 0.0), rubber, "hatch_handle")
    hatch.visual(Cylinder(radius=0.025, length=0.19), origin=Origin(xyz=(0.0, -0.045, 0.0)), material=dark_steel, name="hatch_knuckle")
    _make_box(hatch, (0.10, 0.035, 0.26), (0.045, -0.075, 0.0), dark_steel, "hatch_leaf")

    hub = model.part("rotor_hub")
    hub.visual(
        Cylinder(radius=0.36, length=0.08),
        origin=_axis_cylinder_origin((0.04, 0.0, 0.0), axis="x"),
        material=service_yellow,
        name="replaceable_wear_flange",
    )
    hub.visual(
        Cylinder(radius=0.28, length=0.34),
        origin=_axis_cylinder_origin((0.17, 0.0, 0.0), axis="x"),
        material=dark_steel,
        name="hub_shell",
    )
    hub.visual(Sphere(radius=0.20), origin=Origin(xyz=(0.37, 0.0, 0.0)), material=nacelle_paint, name="nose_cap")
    for index in range(8):
        angle = index * math.tau / 8.0
        hub.visual(
            Cylinder(radius=0.026, length=0.030),
            origin=_axis_cylinder_origin((0.080, 0.285 * math.cos(angle), 0.285 * math.sin(angle)), axis="x"),
            material=galvanized,
            name=f"flange_bolt_{index}",
        )

    blade_angles = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    for index, theta in enumerate(blade_angles):
        v = _radial_unit(theta)
        hub.visual(
            Cylinder(radius=0.135, length=0.32),
            origin=_radial_cylinder_origin(theta, HUB_SOCKET_RADIUS - 0.16, x=HUB_SOCKET_X),
            material=dark_steel,
            name=f"blade_socket_{index}",
        )
        hub.visual(
            Cylinder(radius=0.165, length=0.045),
            origin=_radial_cylinder_origin(theta, HUB_SOCKET_RADIUS - 0.023, x=HUB_SOCKET_X),
            material=service_yellow,
            name=f"socket_wear_ring_{index}",
        )
        blade = model.part(f"blade_{index}")
        blade.visual(
            Cylinder(radius=0.108, length=BLADE_ROOT_LENGTH),
            origin=_radial_cylinder_origin(theta, BLADE_ROOT_LENGTH / 2.0),
            material=dark_steel,
            name="root_bearing",
        )
        blade.visual(
            Cylinder(radius=0.135, length=0.045),
            origin=_radial_cylinder_origin(theta, BLADE_ROOT_LENGTH + 0.010),
            material=service_yellow,
            name="root_wear_collar",
        )
        blade.visual(mesh_from_geometry(_blade_shell(theta), f"blade_{index}_shell"), material=blade_white, name="blade_shell")
        blade.visual(mesh_from_geometry(_blade_shell(theta, tip_only=True), f"blade_{index}_tip"), material=worn_orange, name="tip_wear_cap")

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.20, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "nacelle_to_hatch",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child=hatch,
        origin=Origin(xyz=(-0.49, -0.485, 0.45)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "nacelle_to_hub",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=hub,
        origin=Origin(xyz=(NACELLE_HUB_X, 0.0, NACELLE_HUB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=6.0),
    )
    for index, theta in enumerate(blade_angles):
        v = _radial_unit(theta)
        model.articulation(
            f"hub_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=f"blade_{index}",
            origin=Origin(xyz=(HUB_SOCKET_X, v[1] * HUB_SOCKET_RADIUS, v[2] * HUB_SOCKET_RADIUS)),
            axis=v,
            motion_limits=MotionLimits(effort=900.0, velocity=0.55, lower=-0.35, upper=0.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("rotor_hub")
    hatch = object_model.get_part("access_hatch")

    ctx.expect_contact(tower, nacelle, elem_a="lower_yaw_bearing", elem_b="upper_yaw_bearing", name="yaw bearing stack is seated")
    ctx.expect_contact(nacelle, hub, elem_a="front_bearing", elem_b="replaceable_wear_flange", name="hub flange bears on nacelle bearing")
    ctx.expect_contact(nacelle, hatch, elem_a="hatch_fixed_knuckle_lower", elem_b="hatch_knuckle", contact_tol=0.08, name="service hatch hinge knuckles share a pin line")

    for index in range(3):
        blade = object_model.get_part(f"blade_{index}")
        ctx.allow_overlap(
            hub,
            blade,
            elem_a=f"blade_socket_{index}",
            elem_b="root_bearing",
            reason="The cylindrical blade root is intentionally captured inside the hub socket sleeve to show the load path.",
        )
        ctx.expect_contact(
            hub,
            blade,
            elem_a=f"blade_socket_{index}",
            elem_b="root_bearing",
            contact_tol=0.002,
            name=f"blade {index} root socket contact",
        )
        ctx.expect_overlap(
            hub,
            blade,
            axes="x",
            elem_a=f"blade_socket_{index}",
            elem_b="root_bearing",
            min_overlap=0.10,
            name=f"blade {index} root load path shares hub plane",
        )

    hatch_joint = object_model.get_articulation("nacelle_to_hatch")
    yaw_joint = object_model.get_articulation("tower_to_nacelle")
    rotor_joint = object_model.get_articulation("nacelle_to_hub")
    blade_pitch = object_model.get_articulation("hub_to_blade_0")

    closed_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({hatch_joint: 1.0}):
        open_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    closed_y = None if closed_aabb is None else (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
    open_y = None if open_aabb is None else (open_aabb[0][1] + open_aabb[1][1]) * 0.5
    ctx.check(
        "access hatch opens outward",
        closed_y is not None and open_y is not None and open_y < closed_y - 0.15,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    rest_hub = ctx.part_world_position(hub)
    with ctx.pose({rotor_joint: 0.9, blade_pitch: 0.25}):
        ctx.expect_origin_distance(hub, "blade_0", axes="yz", min_dist=0.45, max_dist=0.75, name="pitched blade root remains on hub radius")
    with ctx.pose({yaw_joint: 0.5}):
        moved_hub = ctx.part_world_position(hub)
    ctx.check(
        "yaw moves nacelle-mounted rotor around tower",
        rest_hub is not None and moved_hub is not None and abs(moved_hub[1] - rest_hub[1]) > 0.4,
        details=f"rest={rest_hub}, moved={moved_hub}",
    )

    return ctx.report()


object_model = build_object_model()
