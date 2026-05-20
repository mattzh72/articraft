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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
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


def _chainring_outer_profile(
    *,
    tooth_count: int,
    root_radius: float,
    shoulder_radius: float,
    tip_radius: float,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    pitch = (2.0 * math.pi) / tooth_count
    for tooth_index in range(tooth_count):
        center_angle = tooth_index * pitch
        tip_fraction = 0.19 if tooth_index % 2 == 0 else 0.11
        for frac, radius in (
            (-0.50, root_radius),
            (-0.30, shoulder_radius),
            (-tip_fraction, tip_radius),
            (tip_fraction, tip_radius),
            (0.30, shoulder_radius),
            (0.50, root_radius),
        ):
            theta = center_angle + frac * pitch
            profile.append((radius * math.cos(theta), radius * math.sin(theta)))
    return profile


def _crank_arm_profile() -> list[tuple[float, float]]:
    return [
        (0.020, 0.019),
        (0.030, 0.024),
        (0.048, 0.024),
        (0.082, 0.017),
        (0.132, 0.013),
        (0.162, 0.013),
        (0.176, 0.018),
        (0.176, -0.018),
        (0.162, -0.013),
        (0.132, -0.013),
        (0.082, -0.017),
        (0.048, -0.024),
        (0.030, -0.024),
        (0.020, -0.019),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_crankset")

    shell_paint = model.material("shell_paint", rgba=(0.30, 0.31, 0.33, 1.0))
    crank_alloy = model.material("crank_alloy", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.18, 0.19, 0.21, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    pedal_body = model.material("pedal_body", rgba=(0.14, 0.15, 0.16, 1.0))

    shell_mesh = _save_mesh(
        "bb_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.026, -0.041),
                (0.026, -0.0375),
                (0.0225, -0.0365),
                (0.0225, 0.0365),
                (0.026, 0.0375),
                (0.026, 0.041),
            ],
            [
                (0.0175, -0.041),
                (0.0175, 0.041),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    boss_ring_mesh = _save_mesh(
        "crank_boss_ring",
        LatheGeometry.from_shell_profiles(
            [(0.034, -0.0075), (0.034, 0.0075)],
            [(0.0155, -0.0075), (0.0155, 0.0075)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    arm_body_mesh = _save_mesh("crank_arm_body", ExtrudeGeometry(_crank_arm_profile(), 0.015, center=True))

    ring_windows = [_circle_profile(0.020, segments=30)]
    ring_window_profile = rounded_rect_profile(0.020, 0.032, 0.0045, corner_segments=6)
    for angle in (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0):
        ring_windows.append(
            _transform_profile(
                ring_window_profile,
                dx=0.036 * math.cos(angle),
                dy=0.036 * math.sin(angle),
                angle=angle,
            )
        )
    chainring_mesh = _save_mesh(
        "narrow_wide_chainring",
        ExtrudeWithHolesGeometry(
            _chainring_outer_profile(
                tooth_count=32,
                root_radius=0.061,
                shoulder_radius=0.0675,
                tip_radius=0.072,
            ),
            ring_windows,
            height=0.0036,
            center=True,
        ),
    )

    pedal_mesh = _save_mesh(
        "pedal_platform",
        ExtrudeGeometry(rounded_rect_profile(0.102, 0.062, 0.010, corner_segments=8), 0.014, center=True),
    )

    bb_shell = model.part("bottom_bracket_shell")
    bb_shell.visual(shell_mesh, material=shell_paint, name="shell_body")
    bb_shell.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.082)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.0142, length=0.164),
        material=spindle_steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.020, length=0.0145),
        origin=Origin(xyz=(0.0, 0.0, 0.04825)),
        material=spindle_steel,
        name="drive_interface",
    )
    spindle.visual(
        Cylinder(radius=0.020, length=0.0145),
        origin=Origin(xyz=(0.0, 0.0, -0.04825)),
        material=spindle_steel,
        name="left_interface",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.164),
        mass=0.75,
    )

    drive_crank = model.part("drive_crank")
    drive_crank.visual(
        boss_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=crank_alloy,
        name="boss_ring",
    )
    drive_crank.visual(
        Box((0.030, 0.010, 0.006)),
        origin=Origin(xyz=(0.031, 0.0, 0.056)),
        material=crank_alloy,
        name="spider_mount",
    )
    for spider_index, (spider_xyz, spider_yaw) in enumerate(
        (
            ((0.0, 0.031, 0.056), math.pi / 2.0),
            ((-0.031, 0.0, 0.056), math.pi),
            ((0.0, -0.031, 0.056), -math.pi / 2.0),
        )
    ):
        drive_crank.visual(
            Box((0.030, 0.010, 0.006)),
            origin=Origin(xyz=spider_xyz, rpy=(0.0, 0.0, spider_yaw)),
            material=crank_alloy,
            name=f"spider_arm_{spider_index}",
        )
    for spider_index, spider_xyz in enumerate(
        (
            (0.046, 0.0, 0.056),
            (0.0, 0.046, 0.056),
            (-0.046, 0.0, 0.056),
            (0.0, -0.046, 0.056),
        )
    ):
        drive_crank.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=spider_xyz),
            material=crank_alloy,
            name=f"chainring_tab_{spider_index}",
        )
    drive_crank.visual(
        arm_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=crank_alloy,
        name="arm_body",
    )
    drive_crank.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.175, 0.0, 0.063)),
        material=crank_alloy,
        name="pedal_eye",
    )
    drive_crank.inertial = Inertial.from_geometry(
        Box((0.190, 0.070, 0.090)),
        mass=0.7,
        origin=Origin(xyz=(0.090, 0.0, 0.060)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        boss_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.063)),
        material=crank_alloy,
        name="boss_ring",
    )
    left_crank.visual(
        arm_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.063)),
        material=crank_alloy,
        name="arm_body",
    )
    left_crank.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.175, 0.0, -0.063)),
        material=crank_alloy,
        name="pedal_eye",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.190, 0.070, 0.090)),
        mass=0.65,
        origin=Origin(xyz=(0.090, 0.0, -0.060)),
    )

    chainring = model.part("chainring")
    chainring.visual(chainring_mesh, material=dark_alloy, name="chainring_plate")
    chainring.inertial = Inertial.from_geometry(
        Box((0.150, 0.150, 0.006)),
        mass=0.14,
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_alloy,
        name="axle_sleeve",
    )
    right_pedal.visual(
        pedal_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.029), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=pedal_body,
        name="pedal_body",
    )
    right_pedal.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.029), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="pedal_crossbar",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.105, 0.070, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=dark_alloy,
        name="axle_sleeve",
    )
    left_pedal.visual(
        pedal_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.029), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=pedal_body,
        name="pedal_body",
    )
    left_pedal.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.029), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="pedal_crossbar",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.105, 0.070, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
    )

    model.articulation(
        "shell_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=25.0),
    )
    model.articulation(
        "spindle_to_drive_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=drive_crank,
        origin=Origin(),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "spindle_to_chainring",
        ArticulationType.FIXED,
        parent=spindle,
        child=chainring,
        origin=Origin(xyz=(0.0, 0.0, 0.0512)),
    )
    model.articulation(
        "drive_crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=drive_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.175, 0.0, 0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=30.0),
    )
    model.articulation(
        "left_crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(0.175, 0.0, -0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    bb_shell = object_model.get_part("bottom_bracket_shell")
    drive_crank = object_model.get_part("drive_crank")
    left_crank = object_model.get_part("left_crank")
    chainring = object_model.get_part("chainring")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    crank_spin = object_model.get_articulation("shell_to_spindle")
    right_pedal_spin = object_model.get_articulation("drive_crank_to_right_pedal")
    left_pedal_spin = object_model.get_articulation("left_crank_to_left_pedal")

    ctx.expect_origin_gap(
        chainring,
        bb_shell,
        axis="z",
        min_gap=0.049,
        max_gap=0.054,
        name="boost chainline offset sits on the drive side",
    )
    ctx.expect_gap(
        drive_crank,
        bb_shell,
        axis="z",
        positive_elem="arm_body",
        negative_elem="shell_body",
        min_gap=0.010,
        max_gap=0.025,
        name="drive crank clears the shell",
    )
    ctx.expect_gap(
        bb_shell,
        left_crank,
        axis="z",
        positive_elem="shell_body",
        negative_elem="arm_body",
        min_gap=0.010,
        max_gap=0.025,
        name="left crank clears the shell",
    )
    ctx.expect_contact(
        chainring,
        drive_crank,
        elem_a="chainring_plate",
        contact_tol=0.001,
        name="chainring seats against the integrated spider",
    )

    rest_right_pos = ctx.part_world_position(right_pedal)
    with ctx.pose({crank_spin: math.pi / 2.0}):
        quarter_right_pos = ctx.part_world_position(right_pedal)
    ctx.check(
        "crank rotation carries the right pedal forward around the spindle",
        rest_right_pos is not None
        and quarter_right_pos is not None
        and quarter_right_pos[1] > rest_right_pos[1] + 0.14
        and abs(quarter_right_pos[0]) < 0.03,
        details=f"rest={rest_right_pos}, quarter_turn={quarter_right_pos}",
    )

    right_body_aabb_0 = None
    right_body_aabb_90 = None
    with ctx.pose({right_pedal_spin: 0.0}):
        right_body_aabb_0 = ctx.part_element_world_aabb(right_pedal, elem="pedal_body")
    with ctx.pose({right_pedal_spin: math.pi / 2.0}):
        right_body_aabb_90 = ctx.part_element_world_aabb(right_pedal, elem="pedal_body")
    ctx.check(
        "right pedal spins about its axle",
        right_body_aabb_0 is not None
        and right_body_aabb_90 is not None
        and (right_body_aabb_0[1][1] - right_body_aabb_0[0][1]) > (right_body_aabb_0[1][0] - right_body_aabb_0[0][0]) + 0.02
        and (right_body_aabb_90[1][0] - right_body_aabb_90[0][0]) > (right_body_aabb_90[1][1] - right_body_aabb_90[0][1]) + 0.02,
        details=f"rest={right_body_aabb_0}, spun={right_body_aabb_90}",
    )

    left_body_aabb_0 = None
    left_body_aabb_90 = None
    with ctx.pose({left_pedal_spin: 0.0}):
        left_body_aabb_0 = ctx.part_element_world_aabb(left_pedal, elem="pedal_body")
    with ctx.pose({left_pedal_spin: math.pi / 2.0}):
        left_body_aabb_90 = ctx.part_element_world_aabb(left_pedal, elem="pedal_body")
    ctx.check(
        "left pedal spins about its axle",
        left_body_aabb_0 is not None
        and left_body_aabb_90 is not None
        and (left_body_aabb_0[1][1] - left_body_aabb_0[0][1]) > (left_body_aabb_0[1][0] - left_body_aabb_0[0][0]) + 0.02
        and (left_body_aabb_90[1][0] - left_body_aabb_90[0][0]) > (left_body_aabb_90[1][1] - left_body_aabb_90[0][1]) + 0.02,
        details=f"rest={left_body_aabb_0}, spun={left_body_aabb_90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
