from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


SHELL_WIDTH = 0.092
SHELL_OUTER_RADIUS = 0.033
SHELL_INNER_RADIUS = 0.019
SPINDLE_RADIUS = 0.015
SPINDLE_LENGTH = 0.068
ARM_INTERFACE_X = 0.050
ARM_HALF_THICKNESS = 0.012
CRANK_LENGTH = 0.1725
TIP_FORWARD_OFFSET = 0.012
PEDAL_INTERFACE_OFFSET = 0.024
CHAINRING_OFFSET = 0.010


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _xy_section(
    center_x: float,
    center_y: float,
    z_pos: float,
    width_x: float,
    height_y: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + px, center_y + py, z_pos)
        for px, py in rounded_rect_profile(width_x, height_y, corner_radius, corner_segments=8)
    ]


def _ellipse_profile(radius_y: float, radius_z: float, *, samples: int = 96) -> list[tuple[float, float]]:
    return [
        (radius_y * cos((tau * i) / samples), radius_z * sin((tau * i) / samples))
        for i in range(samples)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dy: float = 0.0,
    dz: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * y - s * z + dy, s * y + c * z + dz) for y, z in profile]


def _toothed_oval_profile(
    radius_y: float,
    radius_z: float,
    *,
    teeth: int,
    tooth_height: float,
    samples_per_tooth: int = 8,
) -> list[tuple[float, float]]:
    samples = teeth * samples_per_tooth
    points: list[tuple[float, float]] = []
    for index in range(samples):
        theta = (tau * index) / samples
        tooth_wave = (0.5 + 0.5 * cos(teeth * theta)) ** 1.8
        scale = 1.0 + (tooth_height * tooth_wave / max(radius_y, radius_z))
        points.append((radius_y * scale * cos(theta), radius_z * scale * sin(theta)))
    return points


def _bottom_bracket_shell_mesh() -> MeshGeometry:
    shell = MeshGeometry()
    rib_radius = 0.0070
    rib_center_radius = 0.0255
    for index in range(12):
        angle = (tau * index) / 12.0
        y_pos = rib_center_radius * cos(angle)
        z_pos = rib_center_radius * sin(angle)
        rib = CylinderGeometry(radius=rib_radius, height=SHELL_WIDTH, radial_segments=20).rotate_y(pi / 2.0)
        rib.translate(0.0, y_pos, z_pos)
        shell.merge(rib)
    return shell


def _arm_body_mesh(side_sign: float) -> MeshGeometry:
    center_x = side_sign * ARM_HALF_THICKNESS
    tip_y = TIP_FORWARD_OFFSET
    body = section_loft(
        [
            _xy_section(center_x, 0.000, 0.000, 0.022, 0.024, 0.005),
            _xy_section(center_x, 0.003, -0.040, 0.021, 0.020, 0.0045),
            _xy_section(center_x, 0.007, -0.105, 0.019, 0.016, 0.0040),
            _xy_section(center_x, tip_y, -CRANK_LENGTH, 0.016, 0.014, 0.0035),
        ]
    )
    spindle_boss = CylinderGeometry(radius=0.020, height=0.024, radial_segments=28).rotate_y(pi / 2.0)
    spindle_boss.translate(center_x, 0.0, 0.0)
    pedal_eye = CylinderGeometry(radius=0.0105, height=0.024, radial_segments=24).rotate_y(pi / 2.0)
    pedal_eye.translate(center_x, tip_y, -CRANK_LENGTH)
    web = BoxGeometry((0.018, 0.012, 0.030)).translate(center_x, tip_y * 0.65, -CRANK_LENGTH + 0.010)
    return _merge([body, spindle_boss, pedal_eye, web])


def _chainring_mesh() -> MeshGeometry:
    outer = _toothed_oval_profile(0.086, 0.079, teeth=40, tooth_height=0.006, samples_per_tooth=8)
    inner = _ellipse_profile(0.022, 0.022, samples=56)
    window_profile = rounded_rect_profile(0.028, 0.040, 0.006, corner_segments=8)
    holes = [inner]
    for angle in (pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0):
        holes.append(_translate_profile(window_profile, dy=0.0, dz=0.052, angle=angle))
    ring = ExtrudeWithHolesGeometry(
        outer,
        holes,
        height=0.005,
        center=True,
    ).rotate_y(pi / 2.0)
    ring.translate(CHAINRING_OFFSET, 0.0, 0.0)
    return ring


def _pedal_mesh(side_sign: float) -> MeshGeometry:
    body = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.078, 0.050, 0.010, corner_segments=8),
        [rounded_rect_profile(0.044, 0.016, 0.004, corner_segments=8)],
        height=0.034,
        center=True,
    ).rotate_y(pi / 2.0)
    body.translate(side_sign * 0.037, 0.0, 0.0)

    axle = CylinderGeometry(radius=0.0055, height=0.058, radial_segments=20).rotate_y(pi / 2.0)
    axle.translate(side_sign * 0.029, 0.0, 0.0)

    front_clip = CylinderGeometry(radius=0.0035, height=0.060, radial_segments=16).rotate_x(pi / 2.0)
    front_clip.translate(side_sign * 0.038, 0.0, 0.012)
    rear_clip = CylinderGeometry(radius=0.0035, height=0.060, radial_segments=16).rotate_x(pi / 2.0)
    rear_clip.translate(side_sign * 0.038, 0.0, -0.012)

    center_cage = BoxGeometry((0.020, 0.018, 0.024)).translate(side_sign * 0.041, 0.0, 0.0)
    outboard_pad = BoxGeometry((0.014, 0.052, 0.012)).translate(side_sign * 0.052, 0.0, 0.0)
    return _merge([body, axle, front_clip, rear_clip, center_cage, outboard_pad])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gravel_one_by_crankset")

    forged_aluminum = model.material("forged_aluminum", rgba=(0.28, 0.29, 0.31, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.18, 0.18, 0.20, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    composite_black = model.material("composite_black", rgba=(0.10, 0.11, 0.12, 1.0))
    ring_black = model.material("ring_black", rgba=(0.07, 0.07, 0.08, 1.0))

    bb_shell = model.part("bottom_bracket_shell")
    bb_shell.visual(
        _mesh("bottom_bracket_shell", _bottom_bracket_shell_mesh()),
        material=dark_alloy,
        name="shell",
    )
    bb_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=SHELL_OUTER_RADIUS, length=SHELL_WIDTH),
        mass=0.85,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    crank_core = model.part("crank_core")
    crank_core.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="spindle",
    )
    crank_core.visual(
        Cylinder(radius=0.0185, length=0.016),
        origin=Origin(xyz=(0.042, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="right_collar",
    )
    crank_core.visual(
        Cylinder(radius=0.0185, length=0.016),
        origin=Origin(xyz=(-0.042, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="left_collar",
    )
    crank_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.017, length=0.100),
        mass=0.60,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    left_arm = model.part("left_crank_arm")
    left_arm.visual(
        _mesh("left_crank_arm", _arm_body_mesh(-1.0)),
        material=forged_aluminum,
        name="arm_body",
    )
    left_arm.inertial = Inertial.from_geometry(
        Box((0.024, 0.030, 0.190)),
        mass=0.42,
        origin=Origin(xyz=(-0.012, 0.004, -0.086)),
    )

    right_arm = model.part("right_crank_arm")
    right_arm.visual(
        _mesh("right_crank_arm", _arm_body_mesh(1.0)),
        material=forged_aluminum,
        name="arm_body",
    )
    right_arm.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=forged_aluminum,
        name="spider_mount",
    )
    right_arm.visual(
        _mesh("oval_chainring", _chainring_mesh()),
        material=ring_black,
        name="chainring",
    )
    right_arm.inertial = Inertial.from_geometry(
        Box((0.030, 0.185, 0.190)),
        mass=0.68,
        origin=Origin(xyz=(0.008, 0.0, -0.030)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        _mesh("left_pedal", _pedal_mesh(-1.0)),
        material=composite_black,
        name="pedal_body",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.060, 0.078, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        _mesh("right_pedal", _pedal_mesh(1.0)),
        material=composite_black,
        name="pedal_body",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.060, 0.078, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "shell_to_crank",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=crank_core,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=16.0),
    )
    model.articulation(
        "crank_to_left_arm",
        ArticulationType.FIXED,
        parent=crank_core,
        child=left_arm,
        origin=Origin(xyz=(-ARM_INTERFACE_X, 0.0, 0.0), rpy=(pi, 0.0, 0.0)),
    )
    model.articulation(
        "crank_to_right_arm",
        ArticulationType.FIXED,
        parent=crank_core,
        child=right_arm,
        origin=Origin(xyz=(ARM_INTERFACE_X, 0.0, 0.0)),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_arm,
        child=left_pedal,
        origin=Origin(xyz=(-PEDAL_INTERFACE_OFFSET, TIP_FORWARD_OFFSET, -CRANK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_arm,
        child=right_pedal,
        origin=Origin(xyz=(PEDAL_INTERFACE_OFFSET, TIP_FORWARD_OFFSET, -CRANK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bb_shell = object_model.get_part("bottom_bracket_shell")
    crank_core = object_model.get_part("crank_core")
    left_arm = object_model.get_part("left_crank_arm")
    right_arm = object_model.get_part("right_crank_arm")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")

    crank_spin = object_model.get_articulation("shell_to_crank")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")

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

    ctx.check(
        "main crank axis is lateral",
        crank_spin.axis == (1.0, 0.0, 0.0),
        details=f"axis={crank_spin.axis}",
    )
    ctx.check(
        "pedal axles are lateral",
        left_pedal_spin.axis == (1.0, 0.0, 0.0) and right_pedal_spin.axis == (1.0, 0.0, 0.0),
        details=f"left={left_pedal_spin.axis}, right={right_pedal_spin.axis}",
    )

    ctx.expect_overlap(
        crank_core,
        bb_shell,
        axes="x",
        elem_a="spindle",
        elem_b="shell",
        min_overlap=0.060,
        name="spindle spans the shell width on the axle axis",
    )
    ctx.expect_within(
        crank_core,
        bb_shell,
        axes="yz",
        inner_elem="spindle",
        outer_elem="shell",
        margin=0.020,
        name="spindle stays centered within the shell envelope",
    )
    ctx.expect_contact(
        crank_core,
        bb_shell,
        elem_a="left_collar",
        elem_b="shell",
        name="left bearing collar seats against the shell support",
    )
    ctx.expect_contact(
        crank_core,
        bb_shell,
        elem_a="right_collar",
        elem_b="shell",
        name="right bearing collar seats against the shell support",
    )
    ctx.expect_contact(left_arm, crank_core, name="left crank arm seats on the spindle")
    ctx.expect_contact(right_arm, crank_core, name="right crank arm seats on the spindle")
    ctx.expect_contact(left_pedal, left_arm, name="left pedal threads into the crank end")
    ctx.expect_contact(right_pedal, right_arm, name="right pedal threads into the crank end")
    ctx.expect_gap(
        right_arm,
        bb_shell,
        axis="x",
        positive_elem="chainring",
        negative_elem="shell",
        min_gap=0.003,
        max_gap=0.012,
        name="oval chainring clears the shell on the drive side",
    )

    rest_right = ctx.part_world_position(right_pedal)
    with ctx.pose({crank_spin: pi / 2.0}):
        quarter_turn_right = ctx.part_world_position(right_pedal)

    ctx.check(
        "crank quarter turn swings the right pedal forward and upward",
        rest_right is not None
        and quarter_turn_right is not None
        and quarter_turn_right[1] > rest_right[1] + 0.12
        and quarter_turn_right[2] > rest_right[2] + 0.12,
        details=f"rest={rest_right}, quarter_turn={quarter_turn_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
