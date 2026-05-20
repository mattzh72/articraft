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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


SHELL_WIDTH = 0.100
SHELL_DEPTH = 0.092
SHELL_HEIGHT = 0.082
SHELL_BORE_RADIUS = 0.0185
SHELL_HALF_WIDTH = SHELL_WIDTH * 0.5

SPINDLE_OUTER_RADIUS = 0.0145
SPINDLE_INNER_RADIUS = 0.0105
SPINDLE_LENGTH = 0.172
SPINDLE_HALF_LENGTH = SPINDLE_LENGTH * 0.5

ARM_INTERFACE_Y = SPINDLE_HALF_LENGTH
ARM_BOSS_THICKNESS = 0.015
PEDAL_BOSS_THICKNESS = 0.016
ARM_LENGTH = 0.172
PEDAL_X_OFFSET = 0.008


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _arm_section(
    width_x: float,
    thickness_y: float,
    z_pos: float,
    *,
    lateral_sign: float,
    x_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    radius = min(width_x * 0.18, thickness_y * 0.45)
    section_2d = rounded_rect_profile(
        width_x,
        thickness_y,
        radius=max(radius, 0.0015),
        corner_segments=6,
    )
    return [
        (x + x_shift, lateral_sign * (y + (thickness_y * 0.5)), z_pos)
        for x, y in section_2d
    ]


def _build_shell_mesh():
    shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(SHELL_DEPTH, SHELL_HEIGHT, 0.010, corner_segments=8),
        [_circle_profile(SHELL_BORE_RADIUS, segments=48)],
        SHELL_WIDTH,
        center=True,
    )
    return shell.rotate_x(math.pi / 2.0)


def _build_spindle_mesh():
    spindle = LatheGeometry.from_shell_profiles(
        [
            (SPINDLE_OUTER_RADIUS, -SPINDLE_HALF_LENGTH),
            (SPINDLE_OUTER_RADIUS, SPINDLE_HALF_LENGTH),
        ],
        [
            (SPINDLE_INNER_RADIUS, -SPINDLE_HALF_LENGTH),
            (SPINDLE_INNER_RADIUS, SPINDLE_HALF_LENGTH),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    return spindle.rotate_x(math.pi / 2.0)


def _build_bearing_ring_mesh(*, outer_radius: float, inner_radius: float, length: float):
    ring = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -length * 0.5),
            (outer_radius, length * 0.5),
        ],
        [
            (inner_radius, -length * 0.5),
            (inner_radius, length * 0.5),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return ring.rotate_x(math.pi / 2.0)


def _build_crank_arm_mesh(lateral_sign: float):
    return section_loft(
        [
            _arm_section(0.042, ARM_BOSS_THICKNESS, 0.000, lateral_sign=lateral_sign, x_shift=0.000),
            _arm_section(0.028, 0.013, -0.050, lateral_sign=lateral_sign, x_shift=0.003),
            _arm_section(0.022, 0.0115, -0.110, lateral_sign=lateral_sign, x_shift=0.006),
            _arm_section(0.032, PEDAL_BOSS_THICKNESS, -ARM_LENGTH, lateral_sign=lateral_sign, x_shift=PEDAL_X_OFFSET),
        ]
    )


def _chainring_outer_profile(
    teeth: int,
    *,
    outer_radius: float,
    root_radius: float,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for tooth in range(teeth):
        base = (2.0 * math.pi * tooth) / teeth
        points.append((root_radius * math.cos(base), root_radius * math.sin(base)))
        mid = base + (math.pi / teeth)
        points.append((outer_radius * math.cos(mid), outer_radius * math.sin(mid)))
    return points


def _build_chainring_mesh():
    outer = _chainring_outer_profile(
        32,
        outer_radius=0.068,
        root_radius=0.0625,
    )
    hole_profiles = [_circle_profile(0.0165, segments=36)]
    spider_window = rounded_rect_profile(0.024, 0.018, 0.004, corner_segments=6)
    for angle in (
        math.pi / 4.0,
        3.0 * math.pi / 4.0,
        5.0 * math.pi / 4.0,
        7.0 * math.pi / 4.0,
    ):
        hole_profiles.append(
            _transform_profile(
                spider_window,
                dx=0.031 * math.cos(angle),
                dy=0.031 * math.sin(angle),
                angle=angle,
            )
        )
    chainring = ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        0.0035,
        center=True,
    )
    return chainring.rotate_x(math.pi / 2.0)


def _build_pedal_body_mesh():
    body = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.108, 0.076, 0.010, corner_segments=8),
        [rounded_rect_profile(0.060, 0.028, 0.006, corner_segments=6)],
        0.060,
        center=True,
    )
    return body.rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tandem_stoker_crankset")

    shell_black = model.material("shell_black", rgba=(0.17, 0.18, 0.19, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    forged_alloy = model.material("forged_alloy", rgba=(0.70, 0.72, 0.75, 1.0))
    chainring_steel = model.material("chainring_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    pedal_composite = model.material("pedal_composite", rgba=(0.10, 0.11, 0.12, 1.0))

    bb_shell = model.part("bb_shell")
    bb_shell.visual(
        _mesh("bb_shell_body", _build_shell_mesh()),
        material=shell_black,
        name="shell_body",
    )
    bb_shell.inertial = Inertial.from_geometry(
        Box((SHELL_DEPTH, SHELL_WIDTH, SHELL_HEIGHT)),
        mass=1.7,
        origin=Origin(),
    )

    spindle = model.part("spindle")
    spindle.visual(
        _mesh("hollow_spindle", _build_spindle_mesh()),
        material=spindle_steel,
        name="spindle_tube",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(
            xyz=(0.0, -SHELL_HALF_WIDTH - 0.003, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="left_bearing_collar",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(
            xyz=(0.0, SHELL_HALF_WIDTH + 0.003, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="right_bearing_collar",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=SPINDLE_OUTER_RADIUS, length=SPINDLE_LENGTH),
        mass=0.55,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        _mesh("left_crank_arm_body", _build_crank_arm_mesh(-1.0)),
        material=forged_alloy,
        name="arm_body",
    )
    left_crank.visual(
        Cylinder(radius=0.025, length=ARM_BOSS_THICKNESS),
        origin=Origin(
            xyz=(0.0, -ARM_BOSS_THICKNESS * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=forged_alloy,
        name="spindle_boss",
    )
    left_crank.visual(
        Cylinder(radius=0.016, length=PEDAL_BOSS_THICKNESS),
        origin=Origin(
            xyz=(PEDAL_X_OFFSET, -PEDAL_BOSS_THICKNESS * 0.5, -ARM_LENGTH),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=forged_alloy,
        name="pedal_boss",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.050, PEDAL_BOSS_THICKNESS, ARM_LENGTH + 0.015)),
        mass=0.65,
        origin=Origin(xyz=(0.004, -PEDAL_BOSS_THICKNESS * 0.5, -0.086)),
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        _mesh("right_crank_arm_body", _build_crank_arm_mesh(1.0)),
        material=forged_alloy,
        name="arm_body",
    )
    right_crank.visual(
        Cylinder(radius=0.025, length=ARM_BOSS_THICKNESS),
        origin=Origin(
            xyz=(0.0, ARM_BOSS_THICKNESS * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=forged_alloy,
        name="spindle_boss",
    )
    right_crank.visual(
        Cylinder(radius=0.016, length=PEDAL_BOSS_THICKNESS),
        origin=Origin(
            xyz=(PEDAL_X_OFFSET, PEDAL_BOSS_THICKNESS * 0.5, -ARM_LENGTH),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=forged_alloy,
        name="pedal_boss",
    )
    right_crank.visual(
        _mesh("stoker_chainring", _build_chainring_mesh()),
        origin=Origin(xyz=(0.0, 0.0135, 0.0)),
        material=chainring_steel,
        name="chainring",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.140, 0.090, 0.145)),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(
            xyz=(0.0, -0.010, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="spindle_stub",
    )
    left_pedal.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(
            xyz=(0.0, -0.020, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="axle_housing",
    )
    left_pedal.visual(
        _mesh("left_platform_pedal", _build_pedal_body_mesh()),
        origin=Origin(xyz=(0.0, -0.036, 0.0)),
        material=pedal_composite,
        name="pedal_body",
    )
    left_pedal.visual(
        Box((0.022, 0.024, 0.036)),
        origin=Origin(xyz=(0.020, -0.026, 0.0)),
        material=pedal_composite,
        name="outer_web_right",
    )
    left_pedal.visual(
        Box((0.022, 0.024, 0.036)),
        origin=Origin(xyz=(-0.020, -0.026, 0.0)),
        material=pedal_composite,
        name="outer_web_left",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.108, 0.064, 0.078)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.036, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(
            xyz=(0.0, 0.010, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="spindle_stub",
    )
    right_pedal.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(
            xyz=(0.0, 0.020, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="axle_housing",
    )
    right_pedal.visual(
        _mesh("right_platform_pedal", _build_pedal_body_mesh()),
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        material=pedal_composite,
        name="pedal_body",
    )
    right_pedal.visual(
        Box((0.022, 0.024, 0.036)),
        origin=Origin(xyz=(0.020, 0.026, 0.0)),
        material=pedal_composite,
        name="outer_web_right",
    )
    right_pedal.visual(
        Box((0.022, 0.024, 0.036)),
        origin=Origin(xyz=(-0.020, 0.026, 0.0)),
        material=pedal_composite,
        name="outer_web_left",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.108, 0.064, 0.078)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
    )

    model.articulation(
        "bb_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(0.0, -ARM_INTERFACE_Y, 0.0)),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.0, ARM_INTERFACE_Y, 0.0), rpy=(0.0, math.pi, 0.0)),
    )
    model.articulation(
        "left_crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(PEDAL_X_OFFSET, -PEDAL_BOSS_THICKNESS, -ARM_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=20.0,
        ),
    )
    model.articulation(
        "right_crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(PEDAL_X_OFFSET, PEDAL_BOSS_THICKNESS, -ARM_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=20.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bb_shell = object_model.get_part("bb_shell")
    spindle = object_model.get_part("spindle")
    left_crank = object_model.get_part("left_crank")
    right_crank = object_model.get_part("right_crank")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")

    spindle_joint = object_model.get_articulation("bb_to_spindle")
    left_pedal_joint = object_model.get_articulation("left_crank_to_left_pedal")
    right_pedal_joint = object_model.get_articulation("right_crank_to_right_pedal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
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

    ctx.expect_contact(
        spindle,
        bb_shell,
        name="spindle collars seat against shell",
    )
    ctx.expect_contact(
        left_crank,
        spindle,
        name="left crank is mounted on spindle end",
    )
    ctx.expect_contact(
        right_crank,
        spindle,
        name="right crank is mounted on spindle end",
    )
    ctx.expect_contact(
        left_pedal,
        left_crank,
        name="left pedal spindle meets left crank boss",
    )
    ctx.expect_contact(
        right_pedal,
        right_crank,
        name="right pedal spindle meets right crank boss",
    )
    ctx.expect_gap(
        right_crank,
        bb_shell,
        axis="y",
        min_gap=0.040,
        positive_elem="chainring",
        negative_elem="shell_body",
        name="chainring sits clearly outboard of shell",
    )
    ctx.check(
        "all rotating axes are lateral",
        spindle_joint.axis == (0.0, 1.0, 0.0)
        and left_pedal_joint.axis == (0.0, 1.0, 0.0)
        and right_pedal_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"spindle={spindle_joint.axis}, "
            f"left_pedal={left_pedal_joint.axis}, "
            f"right_pedal={right_pedal_joint.axis}"
        ),
    )

    left_pedal_rest = ctx.part_world_position(left_pedal)
    with ctx.pose({spindle_joint: math.pi / 2.0}):
        left_pedal_quarter_turn = ctx.part_world_position(left_pedal)
    ctx.check(
        "quarter turn lifts the downstroke pedal",
        left_pedal_rest is not None
        and left_pedal_quarter_turn is not None
        and left_pedal_quarter_turn[2] > left_pedal_rest[2] + 0.10,
        details=f"rest={left_pedal_rest}, quarter_turn={left_pedal_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
