from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
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


def _build_envelope_mesh():
    sections = [
        (-6.35, 0.08),
        (-5.90, 0.34),
        (-5.10, 0.82),
        (-3.90, 1.55),
        (-2.10, 2.00),
        (0.00, 2.18),
        (1.90, 2.08),
        (3.50, 1.70),
        (4.75, 1.00),
        (5.65, 0.34),
        (6.05, 0.08),
    ]
    body = cq.Workplane("YZ").workplane(offset=sections[0][0]).circle(sections[0][1])
    prev_x = sections[0][0]
    for x_pos, radius in sections[1:]:
        body = body.workplane(offset=x_pos - prev_x).circle(radius)
        prev_x = x_pos
    return mesh_from_cadquery(body.loft(combine=True), "envelope")


def _build_vertical_surface_mesh(name: str, *, chord: float, span: float, thickness: float):
    profile = [
        (0.00, 0.00),
        (-0.16 * chord, 0.00),
        (-0.98 * chord, 0.18 * span),
        (-0.72 * chord, span),
        (0.04 * chord, 0.74 * span),
    ]
    shape = cq.Workplane("XZ").polyline(profile).close().extrude(thickness, both=True)
    return mesh_from_cadquery(shape, name)


def _build_horizontal_surface_mesh(name: str, *, chord: float, span: float, thickness: float):
    profile = [
        (0.00, 0.00),
        (-0.14 * chord, 0.00),
        (-0.98 * chord, 0.16 * span),
        (-0.72 * chord, span),
        (0.10 * chord, 0.82 * span),
    ]
    shape = cq.Workplane("XY").polyline(profile).close().extrude(thickness, both=True)
    return mesh_from_cadquery(shape, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_sign_blimp")

    hull_white = model.material("hull_white", rgba=(0.91, 0.93, 0.95, 1.0))
    sign_blue = model.material("sign_blue", rgba=(0.16, 0.33, 0.78, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.43, 0.46, 0.50, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    glass = model.material("glass", rgba=(0.60, 0.74, 0.88, 0.45))

    vertical_tail_mesh = _build_vertical_surface_mesh(
        "vertical_tail", chord=1.05, span=1.45, thickness=0.08
    )
    horizontal_tail_mesh = _build_horizontal_surface_mesh(
        "horizontal_tail", chord=0.98, span=1.52, thickness=0.08
    )
    propeller_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.42,
            0.08,
            3,
            thickness=0.055,
            blade_pitch_deg=34.0,
            blade_sweep_deg=24.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=18.0, camber=0.10),
            hub=FanRotorHub(
                style="spinner",
                rear_collar_height=0.035,
                rear_collar_radius=0.068,
                bore_diameter=0.020,
            ),
        ),
        "propeller",
    )

    body = model.part("body")
    body.visual(_build_envelope_mesh(), material=hull_white, name="envelope")
    body.visual(
        Box((7.10, 0.18, 1.12)),
        origin=Origin(xyz=(0.50, 1.99, 0.12)),
        material=sign_blue,
        name="port_sign",
    )
    body.visual(
        Box((7.10, 0.18, 1.12)),
        origin=Origin(xyz=(0.50, -1.99, 0.12)),
        material=sign_blue,
        name="starboard_sign",
    )
    body.visual(
        Cylinder(radius=0.17, length=0.06),
        origin=Origin(xyz=(6.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="nose_cap",
    )

    cabin_center = (0.08, 0.0, -2.73)
    body.visual(
        Box((2.55, 0.98, 0.62)),
        origin=Origin(xyz=cabin_center),
        material=trim_gray,
        name="gondola_body",
    )
    body.visual(
        Box((1.72, 0.86, 0.16)),
        origin=Origin(xyz=(0.10, 0.0, -2.34)),
        material=trim_gray,
        name="gondola_roof",
    )
    body.visual(
        Box((1.60, 0.06, 0.20)),
        origin=Origin(xyz=(0.28, 0.52, -2.60)),
        material=glass,
        name="port_window_strip",
    )
    body.visual(
        Box((1.60, 0.06, 0.20)),
        origin=Origin(xyz=(0.28, -0.52, -2.60)),
        material=glass,
        name="starboard_window_strip",
    )
    body.visual(
        Cylinder(radius=0.31, length=0.52),
        origin=Origin(xyz=(1.36, 0.0, -2.73), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="gondola_nose",
    )
    body.visual(
        Cylinder(radius=0.24, length=0.34),
        origin=Origin(xyz=(-1.36, 0.0, -2.73), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="gondola_tail",
    )

    for x_top, x_bottom in ((0.88, 0.84), (-0.74, -0.70)):
        for side in (-0.24, 0.24):
            _add_member(
                body,
                (x_top, side * 0.55, -1.95),
                (x_bottom, side, -2.42),
                radius=0.045,
                material=dark_gray,
            )

    nacelle_x = 0.28
    nacelle_y = 2.66
    nacelle_z = -0.42
    for side_sign, prefix in ((1.0, "port"), (-1.0, "starboard")):
        y_center = nacelle_y * side_sign
        body.visual(
            Cylinder(radius=0.20, length=0.86),
            origin=Origin(xyz=(nacelle_x, y_center, nacelle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_gray,
            name=f"{prefix}_nacelle_body",
        )
        body.visual(
            Cylinder(radius=0.22, length=0.14),
            origin=Origin(
                xyz=(nacelle_x + 0.50, y_center, nacelle_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim_gray,
            name=f"{prefix}_nacelle_nose",
        )
        body.visual(
            Cylinder(radius=0.16, length=0.20),
            origin=Origin(
                xyz=(nacelle_x - 0.52, y_center, nacelle_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim_gray,
            name=f"{prefix}_nacelle_tail",
        )
        body.visual(
            Box((0.36, 0.44, 0.40)),
            origin=Origin(xyz=(0.28, 2.34 * side_sign, -0.40)),
            material=trim_gray,
            name=f"{prefix}_mount_pad",
        )
        _add_member(
            body,
            (0.10, 2.04 * side_sign, -0.22),
            (-0.08, 2.24 * side_sign, -0.22),
            radius=0.040,
            material=dark_gray,
        )
        _add_member(
            body,
            (0.48, 2.02 * side_sign, -0.54),
            (0.54, 2.24 * side_sign, -0.54),
            radius=0.040,
            material=dark_gray,
        )
        _add_member(
            body,
            (0.26, 2.04 * side_sign, -0.36),
            (0.26, 2.22 * side_sign, -0.36),
            radius=0.045,
            material=dark_gray,
        )
        body.visual(
            Cylinder(radius=0.045, length=0.24),
            origin=Origin(
                xyz=(nacelle_x + 0.62, y_center, nacelle_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_gray,
            name=f"{prefix}_prop_shaft",
        )

    body.visual(
        vertical_tail_mesh,
        origin=Origin(xyz=(-5.02, 0.0, 0.72)),
        material=hull_white,
        name="top_fin",
    )
    body.visual(
        vertical_tail_mesh,
        origin=Origin(xyz=(-5.02, 0.0, -0.72), rpy=(math.pi, 0.0, 0.0)),
        material=hull_white,
        name="bottom_fin",
    )
    body.visual(
        horizontal_tail_mesh,
        origin=Origin(xyz=(-4.96, 0.0, 0.02)),
        material=hull_white,
        name="left_stabilizer",
    )
    body.visual(
        horizontal_tail_mesh,
        origin=Origin(xyz=(-4.96, 0.0, 0.02), rpy=(math.pi, 0.0, 0.0)),
        material=hull_white,
        name="right_stabilizer",
    )
    body.visual(
        Cylinder(radius=0.18, length=0.32),
        origin=Origin(xyz=(-4.90, 0.0, 0.80), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hull_white,
        name="top_fin_root",
    )
    body.visual(
        Cylinder(radius=0.18, length=0.32),
        origin=Origin(xyz=(-4.90, 0.0, -0.80), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hull_white,
        name="bottom_fin_root",
    )
    body.visual(
        Cylinder(radius=0.16, length=0.34),
        origin=Origin(xyz=(-4.84, 0.74, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hull_white,
        name="left_stabilizer_root",
    )
    body.visual(
        Cylinder(radius=0.16, length=0.34),
        origin=Origin(xyz=(-4.84, -0.74, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hull_white,
        name="right_stabilizer_root",
    )
    body.visual(
        Box((0.10, 0.08, 0.92)),
        origin=Origin(xyz=(-6.05, 0.0, 1.45)),
        material=dark_gray,
        name="top_fin_hinge_beam",
    )
    body.visual(
        Box((0.10, 0.08, 0.92)),
        origin=Origin(xyz=(-6.05, 0.0, -1.45)),
        material=dark_gray,
        name="bottom_fin_hinge_beam",
    )
    body.visual(
        Box((0.10, 1.02, 0.08)),
        origin=Origin(xyz=(-5.93, 0.99, 0.02)),
        material=dark_gray,
        name="left_hinge_beam",
    )
    body.visual(
        Box((0.34, 0.08, 0.08)),
        origin=Origin(xyz=(-5.76, 0.99, 0.02)),
        material=dark_gray,
        name="left_hinge_strut",
    )
    body.visual(
        Box((0.10, 1.02, 0.08)),
        origin=Origin(xyz=(-5.93, -0.99, 0.02)),
        material=dark_gray,
        name="right_hinge_beam",
    )
    body.visual(
        Box((0.34, 0.08, 0.08)),
        origin=Origin(xyz=(-5.76, -0.99, 0.02)),
        material=dark_gray,
        name="right_hinge_strut",
    )

    for z_center, prefix in ((0.34, "top"), (-0.34, "bottom")):
        body.visual(
            Cylinder(radius=0.040, length=0.32),
            origin=Origin(xyz=(-5.94, 0.0, z_center + (0.64 if prefix == "top" else -0.64))),
            material=dark_gray,
            name=f"{prefix}_fin_upper_knuckle",
        )
        body.visual(
            Cylinder(radius=0.040, length=0.32),
            origin=Origin(xyz=(-5.94, 0.0, z_center + (0.04 if prefix == "top" else -0.04))),
            material=dark_gray,
            name=f"{prefix}_fin_lower_knuckle",
        )

    for y_center, prefix in ((0.36, "left"), (-0.36, "right")):
        body.visual(
            Cylinder(radius=0.035, length=0.26),
            origin=Origin(
                xyz=(-5.82, y_center + (0.40 if prefix == "left" else -0.40), 0.02),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_gray,
            name=f"{prefix}_stabilizer_inner_knuckle",
        )
    body.visual(
        Box((0.22, 0.24, 0.14)),
        origin=Origin(xyz=(-1.00, 0.0, -2.86)),
        material=trim_gray,
        name="wheel_fork_block",
    )
    body.visual(
        Box((0.26, 0.04, 0.30)),
        origin=Origin(xyz=(-1.08, 0.10, -3.02)),
        material=dark_gray,
        name="wheel_fork_port",
    )
    body.visual(
        Box((0.26, 0.04, 0.30)),
        origin=Origin(xyz=(-1.08, -0.10, -3.02)),
        material=dark_gray,
        name="wheel_fork_starboard",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(-1.10, 0.07, -3.13), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="wheel_axle_port",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(-1.10, -0.07, -3.13), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="wheel_axle_starboard",
    )

    body.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=(-5.94, 0.0, 1.06)),
        material=dark_gray,
        name="tail_light_top",
    )
    body.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=(-5.94, 0.0, -1.06)),
        material=dark_gray,
        name="tail_light_bottom",
    )
    body.visual(
        Box((0.18, 0.06, 0.06)),
        origin=Origin(xyz=(5.92, 0.0, 0.18)),
        material=rubber,
        name="nose_marker",
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        propeller_mesh,
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rotor",
    )
    left_propeller.visual(
        Cylinder(radius=0.085, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="hub",
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        propeller_mesh,
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rotor",
    )
    right_propeller.visual(
        Cylinder(radius=0.085, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="hub",
    )

    top_rudder = model.part("top_rudder")
    top_rudder.visual(
        Cylinder(radius=0.036, length=0.28),
        origin=Origin(xyz=(-0.07, 0.0, 0.0)),
        material=dark_gray,
        name="barrel",
    )
    top_rudder.visual(
        Box((0.50, 0.055, 0.92)),
        origin=Origin(xyz=(-0.27, 0.0, 0.0)),
        material=hull_white,
        name="panel",
    )

    bottom_rudder = model.part("bottom_rudder")
    bottom_rudder.visual(
        Cylinder(radius=0.036, length=0.28),
        origin=Origin(xyz=(-0.07, 0.0, 0.0)),
        material=dark_gray,
        name="barrel",
    )
    bottom_rudder.visual(
        Box((0.50, 0.055, 0.92)),
        origin=Origin(xyz=(-0.27, 0.0, 0.0)),
        material=hull_white,
        name="panel",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        Cylinder(radius=0.030, length=0.20),
        origin=Origin(xyz=(-0.06, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="barrel",
    )
    left_elevator.visual(
        Box((0.44, 1.02, 0.055)),
        origin=Origin(xyz=(-0.24, 0.0, 0.0)),
        material=hull_white,
        name="panel",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        Cylinder(radius=0.030, length=0.20),
        origin=Origin(xyz=(-0.06, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="barrel",
    )
    right_elevator.visual(
        Box((0.44, 1.02, 0.055)),
        origin=Origin(xyz=(-0.24, 0.0, 0.0)),
        material=hull_white,
        name="panel",
    )

    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        Cylinder(radius=0.090, length=0.090),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    mooring_wheel.visual(
        Cylinder(radius=0.042, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="hub",
    )

    model.articulation(
        "left_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_propeller,
        origin=Origin(xyz=(1.02, nacelle_y, -0.42)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=24.0),
    )
    model.articulation(
        "right_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_propeller,
        origin=Origin(xyz=(1.02, -nacelle_y, -0.42)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=24.0),
    )
    model.articulation(
        "top_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_rudder,
        origin=Origin(xyz=(-6.08, 0.0, 1.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "bottom_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bottom_rudder,
        origin=Origin(xyz=(-6.08, 0.0, -1.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_elevator,
        origin=Origin(xyz=(-5.96, 0.99, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_elevator,
        origin=Origin(xyz=(-5.96, -0.99, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "mooring_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mooring_wheel,
        origin=Origin(xyz=(-1.10, 0.0, -3.13)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    top_rudder = object_model.get_part("top_rudder")
    bottom_rudder = object_model.get_part("bottom_rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    mooring_wheel = object_model.get_part("mooring_wheel")

    left_prop_spin = object_model.get_articulation("left_prop_spin")
    top_rudder_hinge = object_model.get_articulation("top_rudder_hinge")
    bottom_rudder_hinge = object_model.get_articulation("bottom_rudder_hinge")
    left_elevator_hinge = object_model.get_articulation("left_elevator_hinge")
    right_elevator_hinge = object_model.get_articulation("right_elevator_hinge")

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb is not None else None

    ctx.expect_contact(
        left_propeller,
        body,
        elem_a="hub",
        elem_b="port_prop_shaft",
        name="left propeller hub meets port shaft",
    )
    ctx.expect_contact(
        right_propeller,
        body,
        elem_a="hub",
        elem_b="starboard_prop_shaft",
        name="right propeller hub meets starboard shaft",
    )
    ctx.expect_contact(top_rudder, body, name="top rudder is hinge-supported")
    ctx.expect_contact(bottom_rudder, body, name="bottom rudder is hinge-supported")
    ctx.expect_contact(left_elevator, body, name="left elevator is hinge-supported")
    ctx.expect_contact(right_elevator, body, name="right elevator is hinge-supported")
    ctx.expect_contact(mooring_wheel, body, name="mooring wheel rides on its fork axle")

    ctx.expect_origin_gap(
        left_propeller,
        body,
        axis="y",
        min_gap=2.2,
        name="left propeller stays outboard of the hull centerline",
    )
    ctx.expect_origin_gap(
        body,
        right_propeller,
        axis="y",
        min_gap=2.2,
        name="right propeller stays outboard of the hull centerline",
    )
    ctx.expect_origin_gap(
        body,
        mooring_wheel,
        axis="z",
        min_gap=2.8,
        name="mooring wheel hangs below the gondola",
    )

    top_rest = _aabb_center(ctx.part_element_world_aabb(top_rudder, elem="panel"))
    bottom_rest = _aabb_center(ctx.part_element_world_aabb(bottom_rudder, elem="panel"))
    left_rest = _aabb_center(ctx.part_element_world_aabb(left_elevator, elem="panel"))
    right_rest = _aabb_center(ctx.part_element_world_aabb(right_elevator, elem="panel"))
    with ctx.pose({top_rudder_hinge: top_rudder_hinge.motion_limits.upper}):
        top_open = _aabb_center(ctx.part_element_world_aabb(top_rudder, elem="panel"))
    with ctx.pose({bottom_rudder_hinge: bottom_rudder_hinge.motion_limits.upper}):
        bottom_open = _aabb_center(ctx.part_element_world_aabb(bottom_rudder, elem="panel"))
    with ctx.pose({left_elevator_hinge: left_elevator_hinge.motion_limits.upper}):
        left_up = _aabb_center(ctx.part_element_world_aabb(left_elevator, elem="panel"))
    with ctx.pose({right_elevator_hinge: right_elevator_hinge.motion_limits.upper}):
        right_up = _aabb_center(ctx.part_element_world_aabb(right_elevator, elem="panel"))

    ctx.check(
        "top rudder swings sideways",
        top_rest is not None and top_open is not None and top_open[1] < top_rest[1] - 0.08,
        details=f"rest={top_rest}, deflected={top_open}",
    )
    ctx.check(
        "bottom rudder swings sideways",
        bottom_rest is not None and bottom_open is not None and bottom_open[1] < bottom_rest[1] - 0.08,
        details=f"rest={bottom_rest}, deflected={bottom_open}",
    )
    ctx.check(
        "left elevator raises its trailing edge",
        left_rest is not None and left_up is not None and left_up[2] > left_rest[2] + 0.05,
        details=f"rest={left_rest}, raised={left_up}",
    )
    ctx.check(
        "right elevator raises its trailing edge",
        right_rest is not None and right_up is not None and right_up[2] > right_rest[2] + 0.05,
        details=f"rest={right_rest}, raised={right_up}",
    )

    with ctx.pose({left_prop_spin: math.pi / 3.0}):
        ctx.expect_contact(
            left_propeller,
            body,
            elem_a="hub",
            elem_b="port_prop_shaft",
            name="left propeller stays seated while rotating",
        )

    return ctx.report()


object_model = build_object_model()
