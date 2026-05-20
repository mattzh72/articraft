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


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh_x(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
    segments: int = 64,
):
    ring = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [_circle_profile(inner_radius, segments)],
        thickness,
        center=True,
    )
    ring.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(ring, name)


def _arm_section(width_x: float, width_y: float, z_pos: float) -> list[tuple[float, float, float]]:
    return [(x, y, z_pos) for x, y in rounded_rect_profile(width_x, width_y, radius=min(width_x, width_y) * 0.22)]


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="downhill_freeride_crankset")

    shell_black = model.material("shell_black", rgba=(0.16, 0.16, 0.17, 1.0))
    forged_alloy = model.material("forged_alloy", rgba=(0.71, 0.72, 0.74, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.33, 0.34, 0.36, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.48, 0.49, 0.51, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    chainring_black = model.material("chainring_black", rgba=(0.10, 0.10, 0.11, 1.0))

    bb_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.028, -0.040),
                (0.0315, -0.037),
                (0.0315, -0.032),
                (0.0285, -0.028),
                (0.0280, 0.028),
                (0.0315, 0.032),
                (0.0315, 0.037),
                (0.028, 0.040),
            ],
            [
                (0.0198, -0.040),
                (0.0198, -0.030),
                (0.0210, -0.028),
                (0.0210, 0.028),
                (0.0198, 0.030),
                (0.0198, 0.040),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "bb_shell_mesh",
    )
    arm_body_mesh = mesh_from_geometry(
        section_loft(
            [
                _arm_section(0.022, 0.050, -0.002),
                _arm_section(0.019, 0.041, -0.048),
                _arm_section(0.017, 0.033, -0.100),
                _arm_section(0.017, 0.027, -0.144),
                _arm_section(0.018, 0.032, -0.168),
            ]
        ),
        "oversized_crank_arm_body",
    )
    chainring_mesh = _annulus_mesh_x(
        outer_radius=0.088,
        inner_radius=0.050,
        thickness=0.004,
        name="chainring_ring",
    )
    bash_guard_mesh = _annulus_mesh_x(
        outer_radius=0.112,
        inner_radius=0.095,
        thickness=0.006,
        name="bash_guard_ring",
    )
    pedal_sleeve_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.0085, -0.024), (0.0085, 0.024)],
            [(0.0060, -0.024), (0.0060, 0.024)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        "pedal_center_sleeve",
    )

    bb_shell = model.part("bb_shell")
    bb_shell.visual(
        bb_shell_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_black,
        name="shell_body",
    )
    bb_shell.inertial = Inertial.from_geometry(
        Box((0.082, 0.064, 0.064)),
        mass=0.95,
        origin=Origin(),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.015, length=0.088),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="axle_body",
    )
    spindle.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="right_interface",
    )
    spindle.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="left_interface",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((0.090, 0.040, 0.040)),
        mass=0.72,
        origin=Origin(),
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="spindle_boss",
    )
    right_crank.visual(arm_body_mesh, material=forged_alloy, name="arm_body")
    right_crank.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="pedal_eye",
    )
    right_crank.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="spider_carrier",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        right_crank.visual(
            Box((0.011, 0.014, 0.072)),
            origin=Origin(
                xyz=(0.0185, 0.0, 0.058),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_alloy,
            name=f"spider_arm_{index}",
        )
    right_crank.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=chainring_black,
        name="chainring",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        right_crank.visual(
            Box((0.010, 0.010, 0.020)),
            origin=Origin(
                xyz=(0.027, 0.0, 0.091),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_alloy,
            name=f"guard_brace_{index}",
        )
    right_crank.visual(
        bash_guard_mesh,
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material=chainring_black,
        name="bash_guard",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.070, 0.225, 0.225)),
        mass=1.35,
        origin=Origin(xyz=(0.018, 0.0, -0.045)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="spindle_boss",
    )
    left_crank.visual(arm_body_mesh, material=forged_alloy, name="arm_body")
    left_crank.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="pedal_eye",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.036, 0.055, 0.192)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
    )

    right_pedal_axle = model.part("right_pedal_axle")
    right_pedal_axle.visual(
        Cylinder(radius=0.0045, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="axle_spindle",
    )
    right_pedal_axle.visual(
        Cylinder(radius=0.0075, length=0.008),
        origin=Origin(xyz=(-0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="inboard_collar",
    )
    right_pedal_axle.visual(
        Cylinder(radius=0.0070, length=0.006),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="outboard_collar",
    )
    right_pedal_axle.inertial = Inertial.from_geometry(
        Box((0.064, 0.016, 0.016)),
        mass=0.10,
        origin=Origin(),
    )

    left_pedal_axle = model.part("left_pedal_axle")
    left_pedal_axle.visual(
        Cylinder(radius=0.0045, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="axle_spindle",
    )
    left_pedal_axle.visual(
        Cylinder(radius=0.0075, length=0.008),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="inboard_collar",
    )
    left_pedal_axle.visual(
        Cylinder(radius=0.0070, length=0.006),
        origin=Origin(xyz=(-0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="outboard_collar",
    )
    left_pedal_axle.inertial = Inertial.from_geometry(
        Box((0.064, 0.016, 0.016)),
        mass=0.10,
        origin=Origin(),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        pedal_sleeve_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="center_sleeve",
    )
    right_pedal.visual(
        Box((0.058, 0.074, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=pedal_black,
        name="top_rail",
    )
    right_pedal.visual(
        Box((0.058, 0.074, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=pedal_black,
        name="bottom_rail",
    )
    right_pedal.visual(
        Box((0.058, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=pedal_black,
        name="front_rail",
    )
    right_pedal.visual(
        Box((0.058, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=pedal_black,
        name="rear_rail",
    )
    for index, x_pos in enumerate((-0.025, 0.025)):
        for jndex, y_pos in enumerate((-0.024, 0.024)):
            right_pedal.visual(
                Box((0.010, 0.018, 0.016)),
                origin=Origin(xyz=(x_pos, y_pos, 0.0)),
                material=pedal_black,
                name=f"corner_block_{index}_{jndex}",
            )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.068, 0.090, 0.024)),
        mass=0.28,
        origin=Origin(),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        pedal_sleeve_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="center_sleeve",
    )
    left_pedal.visual(
        Box((0.058, 0.074, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=pedal_black,
        name="top_rail",
    )
    left_pedal.visual(
        Box((0.058, 0.074, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=pedal_black,
        name="bottom_rail",
    )
    left_pedal.visual(
        Box((0.058, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=pedal_black,
        name="front_rail",
    )
    left_pedal.visual(
        Box((0.058, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=pedal_black,
        name="rear_rail",
    )
    for index, x_pos in enumerate((-0.025, 0.025)):
        for jndex, y_pos in enumerate((-0.024, 0.024)):
            left_pedal.visual(
                Box((0.010, 0.018, 0.016)),
                origin=Origin(xyz=(x_pos, y_pos, 0.0)),
                material=pedal_black,
                name=f"corner_block_{index}_{jndex}",
            )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.068, 0.090, 0.024)),
        mass=0.28,
        origin=Origin(),
    )

    model.articulation(
        "bb_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=30.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(-0.065, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
    )
    model.articulation(
        "right_crank_to_pedal_axle",
        ArticulationType.FIXED,
        parent=right_crank,
        child=right_pedal_axle,
        origin=Origin(xyz=(0.045, 0.0, -0.170)),
    )
    model.articulation(
        "left_crank_to_pedal_axle",
        ArticulationType.FIXED,
        parent=left_crank,
        child=left_pedal_axle,
        origin=Origin(xyz=(-0.045, 0.0, -0.170)),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_pedal_axle,
        child=right_pedal,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=35.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_pedal_axle,
        child=left_pedal,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=35.0),
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

    bb_shell = object_model.get_part("bb_shell")
    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    right_pedal_axle = object_model.get_part("right_pedal_axle")
    left_pedal_axle = object_model.get_part("left_pedal_axle")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    crank_spin = object_model.get_articulation("bb_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    ctx.expect_contact(
        spindle,
        right_crank,
        elem_a="right_interface",
        elem_b="spindle_boss",
        contact_tol=0.0005,
        name="right crank seats on spindle shoulder",
    )
    ctx.expect_contact(
        spindle,
        left_crank,
        elem_a="left_interface",
        elem_b="spindle_boss",
        contact_tol=0.0005,
        name="left crank seats on spindle shoulder",
    )
    ctx.expect_contact(
        right_pedal_axle,
        right_crank,
        elem_a="inboard_collar",
        elem_b="pedal_eye",
        contact_tol=0.0005,
        name="right pedal axle seats against pedal eye",
    )
    ctx.expect_contact(
        left_pedal_axle,
        left_crank,
        elem_a="inboard_collar",
        elem_b="pedal_eye",
        contact_tol=0.0005,
        name="left pedal axle seats against pedal eye",
    )
    ctx.expect_origin_gap(
        left_pedal_axle,
        right_pedal_axle,
        axis="z",
        min_gap=0.32,
        name="pedal axles start on opposite sides of the spindle",
    )
    ctx.expect_origin_gap(
        right_crank,
        bb_shell,
        axis="x",
        min_gap=0.05,
        max_gap=0.07,
        name="drive side crank sits outboard of the bottom bracket shell",
    )

    right_pedal_rest = ctx.part_world_position(right_pedal_axle)
    with ctx.pose({crank_spin: math.pi / 2.0}):
        right_pedal_quarter = ctx.part_world_position(right_pedal_axle)
    ctx.check(
        "crank rotation swings the right pedal forward around the shell axis",
        right_pedal_rest is not None
        and right_pedal_quarter is not None
        and right_pedal_quarter[1] > 0.14
        and abs(right_pedal_quarter[2]) < 0.03,
        details=f"rest={right_pedal_rest}, quarter_turn={right_pedal_quarter}",
    )

    right_top_rest = _aabb_center(ctx.part_element_world_aabb(right_pedal, elem="top_rail"))
    with ctx.pose({right_pedal_spin: math.pi / 2.0}):
        right_top_turned = _aabb_center(ctx.part_element_world_aabb(right_pedal, elem="top_rail"))
    ctx.check(
        "right pedal platform rotates about its axle",
        right_top_rest is not None
        and right_top_turned is not None
        and abs(right_top_rest[2] - right_top_turned[2]) > 0.008
        and abs(right_top_rest[1] - right_top_turned[1]) > 0.008,
        details=f"rest={right_top_rest}, turned={right_top_turned}",
    )

    left_top_rest = _aabb_center(ctx.part_element_world_aabb(left_pedal, elem="top_rail"))
    with ctx.pose({left_pedal_spin: math.pi / 2.0}):
        left_top_turned = _aabb_center(ctx.part_element_world_aabb(left_pedal, elem="top_rail"))
    ctx.check(
        "left pedal platform rotates about its axle",
        left_top_rest is not None
        and left_top_turned is not None
        and abs(left_top_rest[2] - left_top_turned[2]) > 0.008
        and abs(left_top_rest[1] - left_top_turned[1]) > 0.008,
        details=f"rest={left_top_rest}, turned={left_top_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
