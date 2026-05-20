from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * cos((2.0 * pi * index) / segments),
            radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _chainring_tooth_profile(
    tip_radius: float,
    root_radius: float,
    tooth_count: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    total_points = tooth_count * 2
    for index in range(total_points):
        angle = (2.0 * pi * index) / total_points
        radius = tip_radius if index % 2 == 0 else root_radius
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    outer_profile: list[tuple[float, float]] | None = None,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile or _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            height=thickness,
            center=True,
        ).rotate_x(pi / 2.0),
        name,
    )


def _shell_of_revolution(name: str, outer_profile, inner_profile):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(pi / 2.0),
        name,
    )


def _add_hollow_arm_segment(
    part,
    *,
    center_z: float,
    length: float,
    outer_x: float,
    outer_y: float,
    wall: float,
    material,
) -> None:
    inner_x = max(outer_x - 2.0 * wall, wall)
    side_x = outer_x * 0.5 - wall * 0.5
    top_y = outer_y * 0.5 - wall * 0.5
    part.visual(
        Box((wall, outer_y, length)),
        origin=Origin(xyz=(side_x, 0.0, center_z)),
        material=material,
    )
    part.visual(
        Box((wall, outer_y, length)),
        origin=Origin(xyz=(-side_x, 0.0, center_z)),
        material=material,
    )
    part.visual(
        Box((inner_x, wall, length)),
        origin=Origin(xyz=(0.0, top_y, center_z)),
        material=material,
    )
    part.visual(
        Box((inner_x, wall, length)),
        origin=Origin(xyz=(0.0, -top_y, center_z)),
        material=material,
    )


def _add_spider(part, *, material, plane_y: float, reach: float) -> None:
    for index in range(5):
        angle = (2.0 * pi * index) / 5.0
        arm_length = reach
        center_radius = arm_length * 0.5
        part.visual(
            Box((arm_length, 0.004, 0.010)),
            origin=Origin(
                xyz=(cos(angle) * center_radius, plane_y, sin(angle) * center_radius),
                rpy=(0.0, -angle, 0.0),
            ),
            material=material,
            name=f"spider_arm_{index}",
        )
        part.visual(
            Cylinder(radius=0.0060, length=0.007),
            origin=Origin(
                xyz=(cos(angle) * reach, plane_y - 0.001, sin(angle) * reach),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"spider_mount_{index}",
        )


def _build_crank_arm(
    model: ArticulatedObject,
    part_name: str,
    *,
    arm_material,
    metal_material,
    boss_mesh,
    pedal_eye_mesh,
    include_spider: bool,
):
    arm = model.part(part_name)
    arm.inertial = Inertial.from_geometry(
        Box((0.040, 0.030, 0.185)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )
    arm.visual(boss_mesh, material=metal_material, name="arm_boss")
    _add_hollow_arm_segment(
        arm,
        center_z=-0.055,
        length=0.058,
        outer_x=0.034,
        outer_y=0.021,
        wall=0.0032,
        material=arm_material,
    )
    _add_hollow_arm_segment(
        arm,
        center_z=-0.108,
        length=0.052,
        outer_x=0.029,
        outer_y=0.018,
        wall=0.0030,
        material=arm_material,
    )
    _add_hollow_arm_segment(
        arm,
        center_z=-0.148,
        length=0.034,
        outer_x=0.024,
        outer_y=0.016,
        wall=0.0028,
        material=arm_material,
    )
    arm.visual(pedal_eye_mesh, origin=Origin(xyz=(0.0, 0.0, -0.172)), material=metal_material, name="pedal_eye")
    arm.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.154)),
        material=arm_material,
    )
    if include_spider:
        _add_spider(arm, material=metal_material, plane_y=0.001, reach=0.056)
    return arm


def _build_pedal(model: ArticulatedObject, part_name: str, *, body_material, metal_material):
    pedal = model.part(part_name)
    pedal.inertial = Inertial.from_geometry(
        Box((0.090, 0.080, 0.022)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.042, 0.0)),
    )
    pedal.visual(
        Cylinder(radius=0.0065, length=0.070),
        origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="axle_stub",
    )
    pedal.visual(
        Box((0.082, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.048, 0.0)),
        material=body_material,
        name="pedal_body",
    )
    pedal.visual(
        Box((0.028, 0.022, 0.014)),
        origin=Origin(xyz=(0.026, 0.047, 0.0)),
        material=body_material,
    )
    pedal.visual(
        Box((0.030, 0.024, 0.014)),
        origin=Origin(xyz=(-0.026, 0.047, 0.0)),
        material=body_material,
    )
    pedal.visual(
        Box((0.058, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.048, 0.0065)),
        material=metal_material,
    )
    pedal.visual(
        Box((0.058, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.048, -0.0065)),
        material=metal_material,
    )
    pedal.visual(
        Cylinder(radius=0.0025, length=0.030),
        origin=Origin(xyz=(0.026, 0.049, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_material,
    )
    pedal.visual(
        Cylinder(radius=0.0025, length=0.032),
        origin=Origin(xyz=(-0.026, 0.049, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_material,
    )
    return pedal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_chainring_road_crankset")

    carbon_black = model.material("carbon_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    chainring_grey = model.material("chainring_grey", rgba=(0.48, 0.49, 0.50, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.12, 0.12, 0.13, 1.0))

    shell_mesh = _shell_of_revolution(
        "bottom_bracket_shell",
        outer_profile=[
            (0.023, -0.034),
            (0.026, -0.028),
            (0.026, 0.028),
            (0.023, 0.034),
        ],
        inner_profile=[
            (0.017, -0.034),
            (0.017, 0.034),
        ],
    )
    arm_boss_mesh = _shell_of_revolution(
        "crank_arm_boss",
        outer_profile=[
            (0.030, -0.011),
            (0.030, 0.011),
        ],
        inner_profile=[
            (0.022, -0.011),
            (0.022, 0.011),
        ],
    )
    pedal_eye_mesh = _shell_of_revolution(
        "pedal_eye",
        outer_profile=[
            (0.013, -0.010),
            (0.013, 0.010),
        ],
        inner_profile=[
            (0.0065, -0.010),
            (0.0065, 0.010),
        ],
    )
    outer_ring_mesh = _annulus_mesh(
        "outer_chainring",
        outer_radius=0.108,
        inner_radius=0.090,
        thickness=0.003,
        outer_profile=_chainring_tooth_profile(0.108, 0.104, 52),
    )
    middle_ring_mesh = _annulus_mesh(
        "middle_chainring",
        outer_radius=0.084,
        inner_radius=0.067,
        thickness=0.003,
        outer_profile=_chainring_tooth_profile(0.084, 0.0805, 42),
    )
    inner_ring_mesh = _annulus_mesh(
        "inner_chainring",
        outer_radius=0.069,
        inner_radius=0.051,
        thickness=0.003,
        outer_profile=_chainring_tooth_profile(0.069, 0.0655, 30),
    )

    shell = model.part("bottom_bracket_shell")
    shell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.068),
        mass=0.38,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    shell.visual(shell_mesh, material=dark_steel, name="shell_body")

    spindle = model.part("spindle_assembly")
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.150),
        mass=0.68,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    spindle.visual(
        Cylinder(radius=0.012, length=0.150),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="right_journal",
    )
    spindle.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="left_journal",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_arm_stub",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_arm_stub",
    )

    right_arm = _build_crank_arm(
        model,
        "right_crank_arm",
        arm_material=carbon_black,
        metal_material=dark_steel,
        boss_mesh=arm_boss_mesh,
        pedal_eye_mesh=pedal_eye_mesh,
        include_spider=True,
    )
    left_arm = _build_crank_arm(
        model,
        "left_crank_arm",
        arm_material=carbon_black,
        metal_material=dark_steel,
        boss_mesh=arm_boss_mesh,
        pedal_eye_mesh=pedal_eye_mesh,
        include_spider=False,
    )

    chainrings = model.part("chainring_cluster")
    chainrings.inertial = Inertial.from_geometry(
        Cylinder(radius=0.108, length=0.018),
        mass=1.05,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    chainrings.visual(outer_ring_mesh, origin=Origin(xyz=(0.0, 0.005, 0.0)), material=chainring_grey, name="outer_ring")
    chainrings.visual(middle_ring_mesh, origin=Origin(xyz=(0.0, 0.000, 0.0)), material=chainring_grey, name="middle_ring")
    chainrings.visual(inner_ring_mesh, origin=Origin(xyz=(0.0, -0.005, 0.0)), material=chainring_grey, name="inner_ring")
    for index in range(5):
        angle = (2.0 * pi * index) / 5.0
        x = cos(angle) * 0.056
        z = sin(angle) * 0.056
        outer_tab_radius = 0.074
        middle_tab_radius = 0.0615
        chainrings.visual(
            Cylinder(radius=0.0045, length=0.017),
            origin=Origin(xyz=(x, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_alloy,
            name=f"ring_bolt_sleeve_{index}",
        )
        chainrings.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, 0.007, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_alloy,
            name=f"drive_bolt_head_{index}",
        )
        chainrings.visual(
            Cylinder(radius=0.0060, length=0.003),
            origin=Origin(xyz=(x, -0.007, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_alloy,
        )
        chainrings.visual(
            Box((0.036, 0.004, 0.010)),
            origin=Origin(
                xyz=(cos(angle) * outer_tab_radius, 0.005, sin(angle) * outer_tab_radius),
                rpy=(0.0, -angle, 0.0),
            ),
            material=chainring_grey,
        )
        chainrings.visual(
            Box((0.019, 0.004, 0.010)),
            origin=Origin(
                xyz=(cos(angle) * middle_tab_radius, 0.0, sin(angle) * middle_tab_radius),
                rpy=(0.0, -angle, 0.0),
            ),
            material=chainring_grey,
        )

    right_pedal = _build_pedal(model, "right_pedal", body_material=pedal_black, metal_material=satin_alloy)
    left_pedal = _build_pedal(model, "left_pedal", body_material=pedal_black, metal_material=satin_alloy)

    bb_spin = model.articulation(
        "bottom_bracket_spin",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=18.0),
    )
    model.articulation(
        "right_arm_mount",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_arm,
        origin=Origin(xyz=(0.0, 0.047, 0.0)),
    )
    model.articulation(
        "left_arm_mount",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_arm,
        origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(pi, 0.0, 0.0)),
    )
    model.articulation(
        "drive_spider_mount",
        ArticulationType.FIXED,
        parent=right_arm,
        child=chainrings,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_arm,
        child=right_pedal,
        origin=Origin(xyz=(0.0, 0.0, -0.172)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=35.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_arm,
        child=left_pedal,
        origin=Origin(xyz=(0.0, 0.0, -0.172)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bottom_bracket_shell")
    spindle = object_model.get_part("spindle_assembly")
    right_arm = object_model.get_part("right_crank_arm")
    left_arm = object_model.get_part("left_crank_arm")
    chainrings = object_model.get_part("chainring_cluster")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    bb_spin = object_model.get_articulation("bottom_bracket_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

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
    ctx.allow_overlap(
        shell,
        spindle,
        elem_a="shell_body",
        elem_b="right_journal",
        reason="The drive-side journal is intentionally represented as a bearing seat nested inside the hollow bottom-bracket shell.",
    )
    ctx.allow_overlap(
        shell,
        spindle,
        elem_a="shell_body",
        elem_b="left_journal",
        reason="The non-drive-side journal is intentionally represented as a bearing seat nested inside the hollow bottom-bracket shell.",
    )
    ctx.allow_overlap(
        right_arm,
        spindle,
        elem_a="arm_boss",
        elem_b="right_arm_stub",
        reason="The right crank-arm boss intentionally sockets over the spindle stub as a hollow clamp interface.",
    )
    ctx.allow_overlap(
        left_arm,
        spindle,
        elem_a="arm_boss",
        elem_b="left_arm_stub",
        reason="The left crank-arm boss intentionally sockets over the spindle stub as a hollow clamp interface.",
    )
    ctx.allow_overlap(
        right_arm,
        right_pedal,
        elem_a="pedal_eye",
        elem_b="axle_stub",
        reason="The right pedal axle intentionally passes through the hollow pedal eye at the crank tip.",
    )
    ctx.allow_overlap(
        left_arm,
        left_pedal,
        elem_a="pedal_eye",
        elem_b="axle_stub",
        reason="The left pedal axle intentionally passes through the hollow pedal eye at the crank tip.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        spindle,
        shell,
        elem_a="right_journal",
        elem_b="shell_body",
        name="drive-side journal seats in the bottom bracket shell",
    )
    ctx.expect_contact(
        spindle,
        shell,
        elem_a="left_journal",
        elem_b="shell_body",
        name="non-drive-side journal seats in the bottom bracket shell",
    )
    ctx.expect_contact(
        right_arm,
        spindle,
        elem_a="arm_boss",
        elem_b="right_arm_stub",
        name="right arm clamps onto the drive-side spindle stub",
    )
    ctx.expect_contact(
        left_arm,
        spindle,
        elem_a="arm_boss",
        elem_b="left_arm_stub",
        name="left arm clamps onto the non-drive-side spindle stub",
    )
    ctx.expect_contact(
        chainrings,
        right_arm,
        elem_a="drive_bolt_head_0",
        elem_b="spider_mount_0",
        name="chainring stack bolts seat against the right-arm spider",
    )
    ctx.expect_contact(
        right_pedal,
        right_arm,
        elem_a="axle_stub",
        elem_b="pedal_eye",
        name="right pedal axle runs through the right crank eye",
    )
    ctx.expect_contact(
        left_pedal,
        left_arm,
        elem_a="axle_stub",
        elem_b="pedal_eye",
        name="left pedal axle runs through the left crank eye",
    )
    ctx.expect_origin_distance(
        chainrings,
        spindle,
        axes="xz",
        max_dist=0.001,
        name="chainring stack stays concentric with the spindle axis",
    )
    ctx.check(
        "bottom bracket spins about the shell axis",
        bb_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={bb_spin.axis}",
    )
    ctx.check(
        "pedal axles spin about each crank thickness axis",
        right_pedal_spin.axis == (0.0, 1.0, 0.0) and left_pedal_spin.axis == (0.0, 1.0, 0.0),
        details=f"right={right_pedal_spin.axis}, left={left_pedal_spin.axis}",
    )

    rest_right = ctx.part_world_position(right_pedal)
    rest_left = ctx.part_world_position(left_pedal)
    with ctx.pose({bb_spin: pi / 2.0}):
        quarter_right = ctx.part_world_position(right_pedal)
        quarter_left = ctx.part_world_position(left_pedal)
    ctx.check(
        "cranks stay 180 degrees opposed and quarter-turn around the bottom bracket",
        rest_right is not None
        and rest_left is not None
        and quarter_right is not None
        and quarter_left is not None
        and rest_right[2] < -0.16
        and rest_left[2] > 0.16
        and quarter_right[0] < -0.16
        and quarter_left[0] > 0.16,
        details=(
            f"rest_right={rest_right}, rest_left={rest_left}, "
            f"quarter_right={quarter_right}, quarter_left={quarter_left}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
