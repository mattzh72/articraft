from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)


BODY_SECTIONS = (
    (-0.092, 0.0000, 0.0180, 0.0380),
    (-0.074, 0.0000, 0.0230, 0.0510),
    (-0.040, 0.0000, 0.0270, 0.0560),
    (0.000, 0.0000, 0.0250, 0.0570),
    (0.040, 0.0000, 0.0215, 0.0540),
    (0.072, 0.0000, 0.0165, 0.0380),
    (0.092, 0.0010, 0.0100, 0.0170),
)

BUTTON_TRAVEL = 0.0023
BODY_EXPONENT = 4.0


def _interp_section(y: float) -> tuple[float, float, float, float]:
    if y <= BODY_SECTIONS[0][0]:
        return BODY_SECTIONS[0]
    if y >= BODY_SECTIONS[-1][0]:
        return BODY_SECTIONS[-1]
    for a, b in zip(BODY_SECTIONS, BODY_SECTIONS[1:]):
        if a[0] <= y <= b[0]:
            t = (y - a[0]) / (b[0] - a[0])
            return (
                y,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
                a[3] + (b[3] - a[3]) * t,
            )
    return BODY_SECTIONS[-1]


def _body_top_z(y: float) -> float:
    return _interp_section(y)[2]


def _body_half_width(y: float) -> float:
    return 0.5 * _interp_section(y)[3]


def _body_top_roll(y: float) -> float:
    for a, b in zip(BODY_SECTIONS, BODY_SECTIONS[1:]):
        if a[0] <= y <= b[0]:
            slope = (b[2] - a[2]) / (b[0] - a[0])
            return -math.atan(slope)
    edge_a, edge_b = BODY_SECTIONS[-2], BODY_SECTIONS[-1]
    slope = (edge_b[2] - edge_a[2]) / (edge_b[0] - edge_a[0])
    return -math.atan(slope)


def _body_mesh():
    return mesh_from_geometry(
        superellipse_side_loft(
            BODY_SECTIONS,
            exponents=BODY_EXPONENT,
            segments=64,
            cap=True,
            closed=True,
        ),
        "remote_body",
    )


def _pill_button_mesh(width: float, length: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, length, radius, corner_segments=8),
            height,
            cap=True,
            closed=True,
        ),
        name,
    )


def _wheel_housing_mesh():
    plate = BoxGeometry((0.003, 0.026, 0.020)).translate(0.0015, 0.0000, 0.0000)
    cheek_a = BoxGeometry((0.010, 0.003, 0.020)).translate(0.0050, 0.0105, 0.0000)
    cheek_b = BoxGeometry((0.010, 0.003, 0.020)).translate(0.0050, -0.0105, 0.0000)
    plate.merge(cheek_a).merge(cheek_b)
    return mesh_from_geometry(plate, "thumbwheel_housing")


def _guard_mesh(closed_roll: float):
    barrel = CylinderGeometry(0.0018, 0.016, radial_segments=20).rotate_y(math.pi / 2.0)
    barrel.translate(0.0, 0.0, 0.0016)

    bridge = BoxGeometry((0.016, 0.007, 0.003)).translate(0.0, 0.0035, 0.0015)
    bridge.rotate_x(closed_roll)

    shield = BoxGeometry((0.028, 0.022, 0.0026)).translate(0.0, 0.0138, 0.0033)
    shield.rotate_x(closed_roll)

    lip = BoxGeometry((0.028, 0.002, 0.0040)).translate(0.0, 0.0252, 0.0040)
    lip.rotate_x(closed_roll)

    barrel.merge(bridge).merge(shield).merge(lip)
    return mesh_from_geometry(barrel, "laser_guard")


def _button_origin(x: float, y: float, clearance: float = 0.0002) -> Origin:
    return Origin(
        xyz=(x, y, _body_top_z(y) + clearance),
        rpy=(_body_top_roll(y), 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="presentation_remote")

    model.material("body_shell", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("body_trim", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("button_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("button_highlight", rgba=(0.41, 0.43, 0.46, 1.0))
    model.material("thumbwheel_rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("guard_tint", rgba=(0.86, 0.18, 0.16, 0.45))
    model.material("laser_red", rgba=(0.90, 0.12, 0.10, 0.70))

    body = model.part("body")
    body.visual(_body_mesh(), material="body_shell", name="shell")

    wheel_housing = model.part("wheel_housing")
    wheel_housing.visual(_wheel_housing_mesh(), material="body_trim", name="housing")
    wheel_mount_y = -0.006
    wheel_mount_z = 0.0135
    model.articulation(
        "body_to_wheel_housing",
        ArticulationType.FIXED,
        parent=body,
        child=wheel_housing,
        origin=Origin(
            xyz=(_body_half_width(wheel_mount_y) + 0.00005, wheel_mount_y, wheel_mount_z),
        ),
    )

    thumbwheel = model.part("thumbwheel")
    thumbwheel.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="thumbwheel_rubber",
        name="wheel",
    )
    model.articulation(
        "wheel_housing_to_thumbwheel",
        ArticulationType.CONTINUOUS,
        parent=wheel_housing,
        child=thumbwheel,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=8.0),
    )

    laser_window = model.part("laser_window")
    laser_window.visual(
        Cylinder(radius=0.0032, length=0.0022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="laser_red",
        name="lens",
    )
    model.articulation(
        "body_to_laser_window",
        ArticulationType.FIXED,
        parent=body,
        child=laser_window,
        origin=Origin(xyz=(0.0, 0.0892, 0.0095)),
    )

    guard_y = 0.071
    guard = model.part("guard")
    guard.visual(
        _guard_mesh(_body_top_roll(guard_y)),
        material="guard_tint",
        name="shield",
    )
    model.articulation(
        "body_to_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=guard,
        origin=Origin(xyz=(0.0, guard_y, _body_top_z(guard_y) + 0.00010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    button_mesh_small = _pill_button_mesh(0.014, 0.012, 0.0036, 0.0035, "button_small")
    button_mesh_wide = _pill_button_mesh(0.018, 0.012, 0.0036, 0.0035, "button_wide")
    button_mesh_long = _pill_button_mesh(0.026, 0.013, 0.0038, 0.0038, "button_long")

    button_specs = (
        ("button_0", button_mesh_wide, "button_highlight", (-0.013, 0.016), 0.0020),
        ("button_1", button_mesh_wide, "button_highlight", (0.013, 0.016), 0.0020),
        ("button_2", button_mesh_small, "button_dark", (0.000, -0.010), 0.0023),
        ("button_3", button_mesh_long, "button_dark", (0.000, -0.041), 0.0022),
        ("button_4", button_mesh_small, "button_dark", (0.000, -0.072), 0.0021),
    )

    for name, mesh, material, (x, y), travel in button_specs:
        button = model.part(name)
        button.visual(mesh, material=material, name="cap")
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=_button_origin(x, y),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.06,
                lower=0.0,
                upper=travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    guard = object_model.get_part("guard")
    laser_window = object_model.get_part("laser_window")
    wheel_housing = object_model.get_part("wheel_housing")
    thumbwheel = object_model.get_part("thumbwheel")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_3 = object_model.get_part("button_3")

    guard_joint = object_model.get_articulation("body_to_guard")
    wheel_joint = object_model.get_articulation("wheel_housing_to_thumbwheel")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")
    button_3_joint = object_model.get_articulation("body_to_button_3")

    ctx.expect_contact(button_0, body, name="upper left button seats on the body")
    ctx.expect_contact(button_1, body, name="upper right button seats on the body")
    ctx.expect_contact(button_3, body, name="lower presentation button seats on the body")
    ctx.expect_overlap(guard, laser_window, axes="xy", min_overlap=0.002, name="closed guard covers the laser window")
    ctx.expect_within(thumbwheel, wheel_housing, axes="yz", margin=0.004, name="thumbwheel stays centered in its side housing")

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    rest_button_3 = ctx.part_world_position(button_3)
    rest_guard_aabb = ctx.part_element_world_aabb(guard, elem="shield")

    with ctx.pose({button_0_joint: 0.0020}):
        pressed_button_0 = ctx.part_world_position(button_0)
        still_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 depresses independently",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and still_button_1 is not None
        and rest_button_1 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.0012
        and abs(still_button_1[2] - rest_button_1[2]) < 1e-6,
        details=f"rest0={rest_button_0}, pressed0={pressed_button_0}, rest1={rest_button_1}, still1={still_button_1}",
    )

    with ctx.pose({button_3_joint: 0.0020}):
        pressed_button_3 = ctx.part_world_position(button_3)
    ctx.check(
        "lower button also depresses",
        rest_button_3 is not None
        and pressed_button_3 is not None
        and pressed_button_3[2] < rest_button_3[2] - 0.0010,
        details=f"rest={rest_button_3}, pressed={pressed_button_3}",
    )

    with ctx.pose({guard_joint: 1.25}):
        open_guard_aabb = ctx.part_element_world_aabb(guard, elem="shield")
        ctx.expect_gap(
            guard,
            laser_window,
            axis="z",
            min_gap=0.0025,
            positive_elem="shield",
            negative_elem="lens",
            name="opened guard lifts well above the laser window",
        )
    ctx.check(
        "guard rotates upward",
        rest_guard_aabb is not None
        and open_guard_aabb is not None
        and open_guard_aabb[1][2] > rest_guard_aabb[1][2] + 0.015,
        details=f"closed={rest_guard_aabb}, open={open_guard_aabb}",
    )

    ctx.check(
        "thumbwheel articulation is continuous",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint.motion_limits is not None
        and wheel_joint.motion_limits.lower is None
        and wheel_joint.motion_limits.upper is None,
        details=f"type={wheel_joint.articulation_type}, limits={wheel_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
