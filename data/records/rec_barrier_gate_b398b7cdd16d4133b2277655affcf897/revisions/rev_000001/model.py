from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    horizontal = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(horizontal, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_wheel(model: ArticulatedObject, name: str, *, tire_mat, hub_mat):
    wheel = model.part(name)
    spin_axis_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    wheel.visual(
        Cylinder(radius=0.08, length=0.040),
        origin=spin_axis_origin,
        material=tire_mat,
        name="wheel_tread",
    )
    wheel.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=spin_axis_origin,
        material=hub_mat,
        name="wheel_hub",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.046),
        origin=spin_axis_origin,
        material=hub_mat,
        name="axle_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.08, length=0.040),
        mass=12.0,
        origin=spin_axis_origin,
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_span_security_gate")

    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.65, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.39, 0.41, 0.44, 1.0))
    post_paint = model.material("post_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    galvanized = model.material("galvanized", rgba=(0.73, 0.74, 0.76, 1.0))
    boom_white = model.material("boom_white", rgba=(0.93, 0.94, 0.95, 1.0))
    safety_red = model.material("safety_red", rgba=(0.78, 0.12, 0.10, 1.0))
    amber = model.material("amber", rgba=(0.92, 0.63, 0.14, 0.85))

    site_base = model.part("site_base")
    site_base.visual(
        Box((4.80, 1.20, 0.12)),
        origin=Origin(xyz=(2.20, 0.0, -0.06)),
        material=concrete,
        name="foundation_slab",
    )
    site_base.visual(
        Box((0.92, 0.92, 0.16)),
        origin=Origin(xyz=(0.35, -0.12, -0.08)),
        material=concrete,
        name="post_pedestal",
    )
    site_base.visual(
        Box((3.90, 0.16, 0.03)),
        origin=Origin(xyz=(2.18, 0.0, -0.015)),
        material=rail_steel,
        name="rail_base",
    )
    site_base.visual(
        Box((3.90, 0.09, 0.05)),
        origin=Origin(xyz=(2.18, 0.0, 0.025)),
        material=galvanized,
        name="track_rail",
    )
    site_base.visual(
        Box((0.08, 0.18, 0.16)),
        origin=Origin(xyz=(4.18, 0.0, 0.08)),
        material=post_paint,
        name="rail_stop",
    )

    site_base.visual(
        Box((0.42, 0.30, 1.15)),
        origin=Origin(xyz=(0.25, -0.28, 0.575)),
        material=post_paint,
        name="motor_post",
    )
    site_base.visual(
        Box((0.48, 0.36, 0.08)),
        origin=Origin(xyz=(0.25, -0.28, 1.16)),
        material=post_paint,
        name="post_cap",
    )
    site_base.visual(
        Box((0.26, 0.16, 0.72)),
        origin=Origin(xyz=(0.25, -0.16, 0.46)),
        material=galvanized,
        name="service_door",
    )
    site_base.visual(
        Box((0.32, 0.20, 0.10)),
        origin=Origin(xyz=(0.31, -0.20, 1.05)),
        material=post_paint,
        name="guide_canopy",
    )
    site_base.visual(
        Box((0.16, 0.52, 0.06)),
        origin=Origin(xyz=(0.40, -0.04, 1.23)),
        material=galvanized,
        name="guide_upper",
    )
    site_base.visual(
        Box((0.16, 0.28, 0.05)),
        origin=Origin(xyz=(0.40, 0.0, 1.01)),
        material=galvanized,
        name="guide_lower",
    )
    for side in (-0.10, 0.10):
        site_base.visual(
            Box((0.16, 0.04, 0.32)),
            origin=Origin(xyz=(0.40, side, 1.13)),
            material=galvanized,
            name=f"guide_cheek_{'neg' if side < 0 else 'pos'}",
        )
        site_base.visual(
            Cylinder(radius=0.022, length=0.22),
            origin=Origin(xyz=(0.40, side * 0.84, 1.13)),
            material=rail_steel,
            name=f"guide_roller_{'neg' if side < 0 else 'pos'}",
        )
    site_base.visual(
        Cylinder(radius=0.030, length=0.08),
        origin=Origin(xyz=(0.11, -0.28, 1.23)),
        material=amber,
        name="warning_beacon",
    )
    site_base.inertial = Inertial.from_geometry(
        Box((4.80, 1.20, 1.35)),
        mass=520.0,
        origin=Origin(xyz=(2.20, 0.0, 0.555)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.74, 0.24, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=post_paint,
        name="lower_frame",
    )
    trolley.visual(
        Box((0.52, 0.20, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=rail_steel,
        name="upper_base",
    )
    trolley.visual(
        Box((0.20, 0.16, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=post_paint,
        name="mast_body",
    )
    trolley.visual(
        Box((0.36, 0.20, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        material=galvanized,
        name="saddle_top",
    )
    trolley.visual(
        Box((0.12, 0.06, 0.10)),
        origin=Origin(xyz=(-0.12, -0.13, 0.99)),
        material=post_paint,
        name="saddle_backstop",
    )
    trolley.visual(
        Cylinder(radius=0.026, length=0.08),
        origin=Origin(xyz=(-0.16, -0.13, 1.08)),
        material=amber,
        name="trolley_beacon",
    )
    for x_wheel in (-0.27, -0.09, 0.09, 0.27):
        trolley.visual(
            Box((0.032, 0.010, 0.08)),
            origin=Origin(xyz=(x_wheel, -0.034, 0.040)),
            material=galvanized,
            name=f"fork_left_{x_wheel:.2f}",
        )
        trolley.visual(
            Box((0.032, 0.010, 0.08)),
            origin=Origin(xyz=(x_wheel, 0.034, 0.040)),
            material=galvanized,
            name=f"fork_right_{x_wheel:.2f}",
        )
        trolley.visual(
            Box((0.034, 0.078, 0.014)),
            origin=Origin(xyz=(x_wheel, 0.0, 0.087)),
            material=galvanized,
            name=f"fork_bridge_{x_wheel:.2f}",
        )
    _add_member(trolley, (-0.31, -0.08, 0.16), (-0.05, -0.06, 0.77), 0.013, galvanized)
    _add_member(trolley, (-0.31, 0.08, 0.16), (-0.05, 0.06, 0.77), 0.013, galvanized)
    _add_member(trolley, (0.31, -0.08, 0.16), (0.05, -0.06, 0.77), 0.013, galvanized)
    _add_member(trolley, (0.31, 0.08, 0.16), (0.05, 0.06, 0.77), 0.013, galvanized)
    trolley.inertial = Inertial.from_geometry(
        Box((0.84, 0.28, 1.10)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    boom = model.part("boom")
    boom.visual(
        Box((6.20, 0.12, 0.12)),
        origin=Origin(xyz=(2.35, 0.0, 0.0)),
        material=boom_white,
        name="boom_beam",
    )
    boom.visual(
        Box((0.22, 0.18, 0.08)),
        origin=Origin(xyz=(0.10, 0.0, 0.08)),
        material=galvanized,
        name="boom_mount_block",
    )
    boom.visual(
        Box((2.60, 0.06, 0.05)),
        origin=Origin(xyz=(1.10, 0.0, 0.30)),
        material=galvanized,
        name="truss_top_chord",
    )
    truss_lower = (-0.30, 0.0, 0.06)
    truss_upper_nodes = ((0.15, 0.0, 0.30), (0.80, 0.0, 0.30), (1.45, 0.0, 0.30), (2.10, 0.0, 0.30))
    truss_lower_nodes = ((0.05, 0.0, 0.06), (0.70, 0.0, 0.06), (1.35, 0.0, 0.06), (2.00, 0.0, 0.06), (2.65, 0.0, 0.06))
    _add_member(boom, truss_lower, truss_upper_nodes[0], 0.013, galvanized)
    for idx, upper_node in enumerate(truss_upper_nodes):
        _add_member(boom, truss_lower_nodes[idx], upper_node, 0.011, galvanized)
        _add_member(boom, upper_node, truss_lower_nodes[idx + 1], 0.011, galvanized)
    for stripe_x in (0.55, 1.75, 2.95, 4.15, 5.35):
        boom.visual(
            Box((0.30, 0.006, 0.12)),
            origin=Origin(xyz=(stripe_x, 0.063, 0.0)),
            material=safety_red,
            name=f"stripe_pos_{stripe_x:.2f}",
        )
        boom.visual(
            Box((0.30, 0.006, 0.12)),
            origin=Origin(xyz=(stripe_x, -0.063, 0.0)),
            material=safety_red,
            name=f"stripe_neg_{stripe_x:.2f}",
        )
    boom.visual(
        Box((0.24, 0.18, 0.18)),
        origin=Origin(xyz=(5.33, 0.0, 0.0)),
        material=safety_red,
        name="boom_tip",
    )
    boom.inertial = Inertial.from_geometry(
        Box((6.20, 0.18, 0.45)),
        mass=95.0,
        origin=Origin(xyz=(2.35, 0.0, 0.10)),
    )

    wheel_positions = (-0.27, -0.09, 0.09, 0.27)
    [
        _build_wheel(model, "wheel_1", tire_mat=rail_steel, hub_mat=galvanized),
        _build_wheel(model, "wheel_2", tire_mat=rail_steel, hub_mat=galvanized),
        _build_wheel(model, "wheel_3", tire_mat=rail_steel, hub_mat=galvanized),
        _build_wheel(model, "wheel_4", tire_mat=rail_steel, hub_mat=galvanized),
    ]

    model.articulation(
        "trolley_slide",
        ArticulationType.PRISMATIC,
        parent=site_base,
        child=trolley,
        origin=Origin(xyz=(0.95, 0.0, 0.13)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.45, lower=0.0, upper=2.20),
    )
    model.articulation(
        "boom_mount",
        ArticulationType.FIXED,
        parent=trolley,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
    )

    for wheel_name, x_wheel in zip(
        ("wheel_1", "wheel_2", "wheel_3", "wheel_4"),
        wheel_positions,
    ):
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=trolley,
            child=wheel_name,
            origin=Origin(xyz=(x_wheel, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    site_base = object_model.get_part("site_base")
    trolley = object_model.get_part("trolley")
    boom = object_model.get_part("boom")
    wheel_1 = object_model.get_part("wheel_1")
    wheel_4 = object_model.get_part("wheel_4")
    trolley_slide = object_model.get_articulation("trolley_slide")

    ctx.expect_gap(
        wheel_1,
        site_base,
        axis="z",
        positive_elem="wheel_tread",
        negative_elem="track_rail",
        max_gap=0.003,
        max_penetration=0.0,
        name="lead wheel rides on the rail",
    )
    ctx.expect_gap(
        wheel_4,
        site_base,
        axis="z",
        positive_elem="wheel_tread",
        negative_elem="track_rail",
        max_gap=0.003,
        max_penetration=0.0,
        name="trail wheel rides on the rail",
    )
    ctx.expect_gap(
        boom,
        trolley,
        axis="z",
        positive_elem="boom_beam",
        negative_elem="saddle_top",
        max_gap=0.001,
        max_penetration=1e-6,
        name="boom sits directly on the trolley saddle",
    )

    wheel_joints = [
        object_model.get_articulation("wheel_1_spin"),
        object_model.get_articulation("wheel_2_spin"),
        object_model.get_articulation("wheel_3_spin"),
        object_model.get_articulation("wheel_4_spin"),
    ]
    ctx.check(
        "wheel axles are continuous joints on the trolley",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            and tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0)
            for joint in wheel_joints
        ),
        details=str([(joint.name, joint.articulation_type, joint.axis) for joint in wheel_joints]),
    )

    rest_trolley_pos = ctx.part_world_position(trolley)
    rest_boom_pos = ctx.part_world_position(boom)
    rest_wheel_pos = ctx.part_world_position(wheel_1)
    upper = trolley_slide.motion_limits.upper if trolley_slide.motion_limits is not None else None

    if upper is None:
        ctx.fail("trolley slide has an upper limit", "Expected a prismatic upper travel limit.")
        return ctx.report()

    with ctx.pose({trolley_slide: upper}):
        ctx.expect_gap(
            wheel_4,
            site_base,
            axis="z",
            positive_elem="wheel_tread",
            negative_elem="track_rail",
            max_gap=0.003,
            max_penetration=0.0,
            name="extended trolley still keeps the rear wheel on the rail",
        )
        extended_trolley_pos = ctx.part_world_position(trolley)
        extended_boom_pos = ctx.part_world_position(boom)

    with ctx.pose({"wheel_1_spin": 1.7}):
        spun_wheel_pos = ctx.part_world_position(wheel_1)

    trolley_dx = None if rest_trolley_pos is None or extended_trolley_pos is None else extended_trolley_pos[0] - rest_trolley_pos[0]
    boom_dx = None if rest_boom_pos is None or extended_boom_pos is None else extended_boom_pos[0] - rest_boom_pos[0]

    ctx.check(
        "trolley translates along the ground rail",
        trolley_dx is not None and trolley_dx > upper - 0.02,
        details=f"rest={rest_trolley_pos}, extended={extended_trolley_pos}, dx={trolley_dx}, expected>{upper - 0.02}",
    )
    ctx.check(
        "boom stays rigidly mounted to the trolley during travel",
        boom_dx is not None
        and trolley_dx is not None
        and abs(boom_dx - trolley_dx) < 1e-4
        and rest_boom_pos is not None
        and extended_boom_pos is not None
        and abs(extended_boom_pos[1] - rest_boom_pos[1]) < 1e-4
        and abs(extended_boom_pos[2] - rest_boom_pos[2]) < 1e-4,
        details=(
            f"rest_boom={rest_boom_pos}, extended_boom={extended_boom_pos}, "
            f"boom_dx={boom_dx}, trolley_dx={trolley_dx}"
        ),
    )
    ctx.check(
        "wheel spin occurs about a fixed axle center",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_wheel_pos, spun_wheel_pos)),
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
