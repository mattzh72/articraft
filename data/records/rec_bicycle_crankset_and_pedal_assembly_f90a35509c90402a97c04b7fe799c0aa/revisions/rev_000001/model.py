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
    repair_loft,
    section_loft,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 72, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (radius * cos(phase + tau * i / segments), radius * sin(phase + tau * i / segments))
        for i in range(segments)
    ]


def _yz_loop(profile_yz: list[tuple[float, float]], x_value: float) -> list[tuple[float, float, float]]:
    return [(x_value, y, z) for y, z in profile_yz]


def _plate_from_yz_profile(
    profile_yz: list[tuple[float, float]],
    *,
    x_center: float,
    thickness: float,
) -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _yz_loop(profile_yz, x_center - thickness * 0.5),
                _yz_loop(profile_yz, x_center + thickness * 0.5),
            ]
        ),
        repair="mesh",
    )


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    x_center: float,
    segments: int = 72,
) -> MeshGeometry:
    return (
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [list(reversed(_circle_profile(inner_radius, segments=segments)))],
            height=thickness,
            center=True,
        )
        .rotate_y(pi / 2.0)
        .translate(x_center, 0.0, 0.0)
    )


def _rotated_x(geometry: MeshGeometry, angle: float) -> MeshGeometry:
    return geometry.copy().rotate((1.0, 0.0, 0.0), angle)


def _build_bottom_bracket_shell() -> MeshGeometry:
    shell = _annulus_mesh(
        outer_radius=0.0242,
        inner_radius=0.0116,
        thickness=0.045,
        x_center=0.0,
        segments=64,
    )
    shell.merge(
        _annulus_mesh(
            outer_radius=0.0250,
            inner_radius=0.0124,
            thickness=0.011,
            x_center=-0.028,
            segments=64,
        )
    )
    shell.merge(
        _annulus_mesh(
            outer_radius=0.0250,
            inner_radius=0.0124,
            thickness=0.011,
            x_center=0.028,
            segments=64,
        )
    )
    shell.merge(
        _annulus_mesh(
            outer_radius=0.0295,
            inner_radius=0.0118,
            thickness=0.0075,
            x_center=-0.03725,
            segments=64,
        )
    )
    shell.merge(
        _annulus_mesh(
            outer_radius=0.0295,
            inner_radius=0.0118,
            thickness=0.0075,
            x_center=0.03725,
            segments=64,
        )
    )
    shell.merge(
        _annulus_mesh(
            outer_radius=0.0295,
            inner_radius=0.0180,
            thickness=0.004,
            x_center=-0.041,
            segments=64,
        )
    )
    shell.merge(
        _annulus_mesh(
            outer_radius=0.0295,
            inner_radius=0.0180,
            thickness=0.004,
            x_center=0.041,
            segments=64,
        )
    )
    return shell


def _build_spindle() -> MeshGeometry:
    spindle = CylinderGeometry(radius=0.0108, height=0.128, radial_segments=64).rotate_y(pi / 2.0)
    spindle.merge(CylinderGeometry(radius=0.0120, height=0.011, radial_segments=64).rotate_y(pi / 2.0).translate(0.028, 0.0, 0.0))
    spindle.merge(CylinderGeometry(radius=0.0120, height=0.011, radial_segments=64).rotate_y(pi / 2.0).translate(-0.028, 0.0, 0.0))
    spindle.merge(CylinderGeometry(radius=0.0122, height=0.012, radial_segments=64).rotate_y(pi / 2.0).translate(0.047, 0.0, 0.0))
    spindle.merge(CylinderGeometry(radius=0.0122, height=0.012, radial_segments=64).rotate_y(pi / 2.0).translate(-0.047, 0.0, 0.0))
    return spindle


def _right_arm_profile() -> list[tuple[float, float]]:
    return [
        (-0.014, -0.020),
        (-0.024, -0.048),
        (-0.022, -0.094),
        (-0.018, -0.136),
        (-0.013, -0.160),
        (0.015, -0.162),
        (0.023, -0.136),
        (0.028, -0.088),
        (0.024, -0.040),
        (0.012, -0.018),
    ]


def _build_arm_plate(*, x_center: float, flipped: bool) -> MeshGeometry:
    profile = _right_arm_profile()
    if flipped:
        profile = [(-y, -z) for y, z in profile]
    return _plate_from_yz_profile(profile, x_center=x_center, thickness=0.018)


def _build_spindle_boss(*, x_center: float) -> MeshGeometry:
    return _annulus_mesh(
        outer_radius=0.031,
        inner_radius=0.0138,
        thickness=0.014,
        x_center=x_center,
        segments=64,
    )


def _build_pedal_eye(*, x_center: float, y_center: float, z_center: float) -> MeshGeometry:
    return _annulus_mesh(
        outer_radius=0.018,
        inner_radius=0.0068,
        thickness=0.022,
        x_center=x_center,
        segments=56,
    ).translate(0.0, y_center, z_center)


def _build_right_spider() -> MeshGeometry:
    spider = _annulus_mesh(
        outer_radius=0.034,
        inner_radius=0.0129,
        thickness=0.010,
        x_center=0.049,
        segments=72,
    )
    arm_profile = [(-0.004, -0.023), (-0.011, -0.060), (0.012, -0.089), (0.006, -0.031)]
    base_arm = _plate_from_yz_profile(arm_profile, x_center=0.049, thickness=0.010)
    for index in range(5):
        spider.merge(_rotated_x(base_arm, index * tau / 5.0))
    for index in range(5):
        angle = -pi / 2.0 + index * tau / 5.0
        bolt = CylinderGeometry(radius=0.0036, height=0.012, radial_segments=18).rotate_y(pi / 2.0)
        bolt.translate(0.049, 0.058 * sin(angle), 0.058 * cos(angle))
        spider.merge(bolt)
    return spider


def _build_pedal_body(*, side: str) -> MeshGeometry:
    x_sign = 1.0 if side == "right" else -1.0
    barrel_center = 0.015 * x_sign
    frame_center = 0.029 * x_sign
    pedal = CylinderGeometry(radius=0.0072, height=0.008, radial_segments=20).rotate_y(pi / 2.0).translate(
        barrel_center,
        0.0,
        0.0,
    )
    pedal.merge(BoxGeometry((0.020, 0.084, 0.013)).translate(frame_center, 0.0, 0.0))
    pedal.merge(CylinderGeometry(radius=0.0085, height=0.020, radial_segments=20).rotate_y(pi / 2.0).translate(frame_center, 0.029, 0.0))
    pedal.merge(CylinderGeometry(radius=0.0085, height=0.020, radial_segments=20).rotate_y(pi / 2.0).translate(frame_center, -0.029, 0.0))
    pedal.merge(BoxGeometry((0.004, 0.084, 0.004)).translate((frame_center + 0.008 * x_sign), 0.0, 0.004))
    pedal.merge(BoxGeometry((0.004, 0.084, 0.004)).translate((frame_center + 0.008 * x_sign), 0.0, -0.004))
    return pedal


def _build_pedal_spindle(*, side: str) -> MeshGeometry:
    x_sign = 1.0 if side == "right" else -1.0
    spindle = CylinderGeometry(radius=0.0048, height=0.028, radial_segments=20).rotate_y(pi / 2.0).translate(
        -0.004 * x_sign,
        0.0,
        0.0,
    )
    spindle.merge(
        CylinderGeometry(radius=0.0080, height=0.004, radial_segments=20)
        .rotate_y(pi / 2.0)
        .translate(0.009 * x_sign, 0.0, 0.0)
    )
    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_triple_crankset")

    shell_paint = model.material("shell_paint", rgba=(0.19, 0.19, 0.20, 1.0))
    alloy = model.material("alloy", rgba=(0.80, 0.81, 0.82, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.88, 0.89, 0.90, 1.0))
    chainring_steel = model.material("chainring_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    bottom_bracket_shell = model.part("bottom_bracket_shell")
    bottom_bracket_shell.visual(
        _mesh("bottom_bracket_shell_mesh", _build_bottom_bracket_shell()),
        material=shell_paint,
        name="bb_shell_assembly",
    )
    bottom_bracket_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.086),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        _mesh("spindle_mesh", _build_spindle()),
        material=polished_alloy,
        name="spindle_axle",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.128),
        mass=0.35,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        _mesh("right_arm_plate_mesh", _build_arm_plate(x_center=0.050, flipped=False)),
        material=alloy,
        name="right_arm_plate",
    )
    right_crank.visual(
        _mesh("right_spindle_boss_mesh", _build_spindle_boss(x_center=0.050)),
        material=alloy,
        name="right_spindle_boss",
    )
    right_crank.visual(
        _mesh("right_pedal_eye_mesh", _build_pedal_eye(x_center=0.050, y_center=0.006, z_center=-0.172)),
        material=alloy,
        name="right_pedal_eye",
    )
    right_crank.visual(
        _mesh("right_spider_mesh", _build_right_spider()),
        material=alloy,
        name="right_spider",
    )
    right_crank.visual(
        _mesh("outer_chainring_mesh", _annulus_mesh(outer_radius=0.108, inner_radius=0.056, thickness=0.0038, x_center=0.0535, segments=84)),
        material=chainring_steel,
        name="outer_chainring",
    )
    right_crank.visual(
        _mesh("middle_chainring_mesh", _annulus_mesh(outer_radius=0.093, inner_radius=0.050, thickness=0.0036, x_center=0.0490, segments=78)),
        material=chainring_steel,
        name="middle_chainring",
    )
    right_crank.visual(
        _mesh("inner_chainring_mesh", _annulus_mesh(outer_radius=0.074, inner_radius=0.034, thickness=0.0032, x_center=0.0445, segments=72)),
        material=chainring_steel,
        name="inner_chainring",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.13, 0.24, 0.24)),
        mass=1.05,
        origin=Origin(xyz=(0.050, 0.0, -0.030)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        _mesh("left_arm_plate_mesh", _build_arm_plate(x_center=-0.050, flipped=True)),
        material=alloy,
        name="left_arm_plate",
    )
    left_crank.visual(
        _mesh("left_spindle_boss_mesh", _build_spindle_boss(x_center=-0.050)),
        material=alloy,
        name="left_spindle_boss",
    )
    left_crank.visual(
        _mesh("left_pedal_eye_mesh", _build_pedal_eye(x_center=-0.050, y_center=-0.006, z_center=0.172)),
        material=alloy,
        name="left_pedal_eye",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.10, 0.08, 0.24)),
        mass=0.72,
        origin=Origin(xyz=(-0.050, 0.0, 0.030)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        _mesh("right_pedal_spindle_mesh", _build_pedal_spindle(side="right")),
        material=polished_alloy,
        name="right_pedal_spindle",
    )
    right_pedal.visual(
        _mesh("right_pedal_body_mesh", _build_pedal_body(side="right")),
        material=rubber,
        name="right_pedal_body",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.024, 0.090, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        _mesh("left_pedal_spindle_mesh", _build_pedal_spindle(side="left")),
        material=polished_alloy,
        name="left_pedal_spindle",
    )
    left_pedal.visual(
        _mesh("left_pedal_body_mesh", _build_pedal_body(side="left")),
        material=rubber,
        name="left_pedal_body",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.024, 0.090, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
    )

    model.articulation(
        "shell_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket_shell,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=25.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(),
    )
    model.articulation(
        "right_crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.050, 0.006, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "left_crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.050, -0.006, 0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
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

    bottom_bracket_shell = object_model.get_part("bottom_bracket_shell")
    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    shell_to_spindle = object_model.get_articulation("shell_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_crank_to_right_pedal")
    left_pedal_spin = object_model.get_articulation("left_crank_to_left_pedal")

    right_crank.get_visual("outer_chainring")
    right_crank.get_visual("middle_chainring")
    right_crank.get_visual("inner_chainring")

    ctx.check(
        "main crank joint is continuous about the spindle axis",
        shell_to_spindle.articulation_type == ArticulationType.CONTINUOUS and tuple(shell_to_spindle.axis) == (1.0, 0.0, 0.0),
        details=f"type={shell_to_spindle.articulation_type}, axis={shell_to_spindle.axis}",
    )
    ctx.check(
        "pedal joints are continuous about their spindle axes",
        right_pedal_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_pedal_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(right_pedal_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(left_pedal_spin.axis) == (1.0, 0.0, 0.0),
        details=f"right={right_pedal_spin.axis}, left={left_pedal_spin.axis}",
    )

    with ctx.pose({shell_to_spindle: 0.0, right_pedal_spin: 0.0, left_pedal_spin: 0.0}):
        ctx.expect_within(
            spindle,
            bottom_bracket_shell,
            axes="yz",
            inner_elem="spindle_axle",
            outer_elem="bb_shell_assembly",
            margin=0.0015,
            name="spindle stays centered inside the bottom-bracket shell",
        )
        ctx.expect_overlap(
            spindle,
            bottom_bracket_shell,
            axes="x",
            elem_a="spindle_axle",
            elem_b="bb_shell_assembly",
            min_overlap=0.060,
            name="spindle spans through the shell width",
        )
        ctx.expect_within(
            right_pedal,
            right_crank,
            axes="yz",
            inner_elem="right_pedal_spindle",
            outer_elem="right_pedal_eye",
            margin=0.0015,
            name="right pedal spindle stays centered in the right crank eye",
        )
        ctx.expect_overlap(
            right_pedal,
            right_crank,
            axes="x",
            elem_a="right_pedal_spindle",
            elem_b="right_pedal_eye",
            min_overlap=0.012,
            name="right pedal spindle remains inserted through the crank eye",
        )
        ctx.expect_within(
            left_pedal,
            left_crank,
            axes="yz",
            inner_elem="left_pedal_spindle",
            outer_elem="left_pedal_eye",
            margin=0.0015,
            name="left pedal spindle stays centered in the left crank eye",
        )
        ctx.expect_overlap(
            left_pedal,
            left_crank,
            axes="x",
            elem_a="left_pedal_spindle",
            elem_b="left_pedal_eye",
            min_overlap=0.012,
            name="left pedal spindle remains inserted through the crank eye",
        )

    rest_right_pos = ctx.part_world_position(right_pedal)
    rest_left_pos = ctx.part_world_position(left_pedal)
    with ctx.pose({shell_to_spindle: pi / 2.0}):
        quarter_turn_right_pos = ctx.part_world_position(right_pedal)
        quarter_turn_left_pos = ctx.part_world_position(left_pedal)
    ctx.check(
        "opposed crank arms swing the pedals to opposite quadrants when the spindle turns",
        rest_right_pos is not None
        and rest_left_pos is not None
        and quarter_turn_right_pos is not None
        and quarter_turn_left_pos is not None
        and quarter_turn_right_pos[1] > rest_right_pos[1] + 0.12
        and quarter_turn_right_pos[2] > rest_right_pos[2] + 0.12
        and quarter_turn_left_pos[1] < rest_left_pos[1] - 0.12
        and quarter_turn_left_pos[2] < rest_left_pos[2] - 0.12,
        details=(
            f"rest_right={rest_right_pos}, quarter_right={quarter_turn_right_pos}, "
            f"rest_left={rest_left_pos}, quarter_left={quarter_turn_left_pos}"
        ),
    )

    right_body_rest = ctx.part_element_world_aabb(right_pedal, elem="right_pedal_body")
    left_body_rest = ctx.part_element_world_aabb(left_pedal, elem="left_pedal_body")
    with ctx.pose({right_pedal_spin: pi / 2.0, left_pedal_spin: pi / 2.0}):
        right_body_spun = ctx.part_element_world_aabb(right_pedal, elem="right_pedal_body")
        left_body_spun = ctx.part_element_world_aabb(left_pedal, elem="left_pedal_body")

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "pedal bodies roll around their spindle axes",
        right_body_rest is not None
        and left_body_rest is not None
        and right_body_spun is not None
        and left_body_spun is not None
        and _span(right_body_rest, 1) is not None
        and _span(right_body_rest, 2) is not None
        and _span(left_body_rest, 1) is not None
        and _span(left_body_rest, 2) is not None
        and _span(right_body_rest, 1) > _span(right_body_rest, 2)
        and _span(left_body_rest, 1) > _span(left_body_rest, 2)
        and _span(right_body_spun, 2) > _span(right_body_spun, 1)
        and _span(left_body_spun, 2) > _span(left_body_spun, 1),
        details=(
            f"right_rest={right_body_rest}, right_spun={right_body_spun}, "
            f"left_rest={left_body_rest}, left_spun={left_body_spun}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
