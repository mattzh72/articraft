from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    *,
    segments: int = 32,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _rotate_profile(
    profile: list[tuple[float, float]],
    angle: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        (
            cx + c * (x - cx) - s * (y - cy),
            cy + s * (x - cx) + c * (y - cy),
        )
        for x, y in profile
    ]


def _chainring_tooth_profile(
    *,
    teeth: int,
    root_radius: float,
    tip_radius: float,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for i in range(teeth * 2):
        angle = math.pi * i / teeth
        radius = tip_radius if i % 2 == 0 else root_radius
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _keyed_inner_profile(
    *,
    radius: float,
    flat_x: float,
    flat_half_height: float,
    segments: int = 40,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        if x > flat_x and abs(y) < flat_half_height:
            x = flat_x
        profile.append((x, y))
    return profile


def _arm_outer_profile() -> list[tuple[float, float]]:
    return [
        (-0.018, 0.012),
        (-0.012, 0.020),
        (-0.003, 0.024),
        (0.010, 0.024),
        (0.020, 0.020),
        (0.034, 0.016),
        (0.060, 0.014),
        (0.094, 0.011),
        (0.126, 0.010),
        (0.148, 0.010),
        (0.158, 0.013),
        (0.168, 0.017),
        (0.176, 0.016),
        (0.182, 0.010),
        (0.185, 0.000),
        (0.182, -0.010),
        (0.176, -0.016),
        (0.168, -0.017),
        (0.158, -0.013),
        (0.148, -0.010),
        (0.126, -0.010),
        (0.094, -0.011),
        (0.060, -0.014),
        (0.034, -0.016),
        (0.020, -0.020),
        (0.010, -0.024),
        (-0.003, -0.024),
        (-0.012, -0.020),
        (-0.018, -0.012),
        (-0.021, 0.000),
    ]


def _arm_mesh(*, with_chainring: bool):
    arm = ExtrudeWithHolesGeometry(
        _arm_outer_profile(),
        [
            _circle_profile(0.0152, segments=36, center=(0.0, 0.0)),
            _circle_profile(0.0070, segments=28, center=(0.165, 0.0)),
        ],
        0.018,
        center=True,
    ).rotate_x(math.pi / 2.0)

    if with_chainring:
        chainring = ExtrudeWithHolesGeometry(
            _chainring_tooth_profile(teeth=48, root_radius=0.093, tip_radius=0.099),
            [
                _keyed_inner_profile(radius=0.058, flat_x=0.046, flat_half_height=0.012),
                *[
                    _circle_profile(
                        0.0045,
                        center=(
                            0.072 * math.cos(2.0 * math.pi * i / 5.0),
                            0.072 * math.sin(2.0 * math.pi * i / 5.0),
                        ),
                        segments=18,
                    )
                    for i in range(5)
                ],
            ],
            0.004,
            center=True,
        ).rotate_x(math.pi / 2.0).translate(0.0, 0.010, 0.0)
        arm.merge(chainring)

        spider_profile = [
            (0.022, -0.008),
            (0.042, -0.006),
            (0.068, -0.005),
            (0.068, 0.005),
            (0.042, 0.006),
            (0.022, 0.008),
        ]
        for arm_index in range(1, 5):
            arm.merge(
                ExtrudeGeometry(
                    _rotate_profile(spider_profile, 2.0 * math.pi * arm_index / 5.0),
                    0.010,
                    center=True,
                )
                .rotate_x(math.pi / 2.0)
                .translate(0.0, 0.006, 0.0)
            )

        for arm_index in range(5):
            angle = 2.0 * math.pi * arm_index / 5.0
            arm.merge(
                CylinderGeometry(radius=0.006, height=0.012, radial_segments=18)
                .rotate_x(math.pi / 2.0)
                .translate(
                    0.072 * math.cos(angle),
                    0.007,
                    0.072 * math.sin(angle),
                )
            )

    return arm


def _pedal_mesh(*, side_sign: float):
    pedal = CylinderGeometry(radius=0.0042, height=0.030, radial_segments=18).rotate_x(math.pi / 2.0)
    pedal.translate(0.0, 0.008 * side_sign, 0.0)
    pedal.merge(
        CylinderGeometry(radius=0.010, height=0.006, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.012 * side_sign, 0.0)
    )
    pedal.merge(
        CylinderGeometry(radius=0.0075, height=0.020, radial_segments=22)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.026 * side_sign, 0.0)
    )
    pedal.merge(BoxGeometry((0.016, 0.048, 0.014)).translate(0.0, 0.054 * side_sign, 0.0))
    pedal.merge(BoxGeometry((0.090, 0.018, 0.016)).translate(0.0, 0.086 * side_sign, 0.0))
    pedal.merge(BoxGeometry((0.012, 0.018, 0.024)).translate(0.039, 0.086 * side_sign, 0.0))
    pedal.merge(BoxGeometry((0.012, 0.018, 0.024)).translate(-0.039, 0.086 * side_sign, 0.0))
    return pedal


def _bb_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.029, -0.043),
            (0.033, -0.041),
            (0.033, -0.034),
            (0.030, -0.031),
            (0.030, 0.031),
            (0.033, 0.034),
            (0.033, 0.041),
            (0.029, 0.043),
        ],
        [
            (0.015, -0.043),
            (0.015, 0.043),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(math.pi / 2.0)


def _spindle_mesh():
    spindle = CylinderGeometry(radius=0.0105, height=0.170, radial_segments=28).rotate_x(math.pi / 2.0)
    spindle.merge(
        CylinderGeometry(radius=0.0148, height=0.012, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.042, 0.0)
    )
    spindle.merge(
        CylinderGeometry(radius=0.0148, height=0.012, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.042, 0.0)
    )
    spindle.merge(
        CylinderGeometry(radius=0.018, height=0.006, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.046, 0.0)
    )
    spindle.merge(
        CylinderGeometry(radius=0.018, height=0.006, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.046, 0.0)
    )
    spindle.merge(
        CylinderGeometry(radius=0.008, height=0.010, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.082, 0.0)
    )
    spindle.merge(
        CylinderGeometry(radius=0.008, height=0.010, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.082, 0.0)
    )
    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixie_track_crankset")

    anodized_alloy = model.material("anodized_alloy", rgba=(0.74, 0.75, 0.78, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.84, 0.85, 0.87, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.21, 0.22, 0.24, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    reflector_amber = model.material("reflector_amber", rgba=(0.84, 0.43, 0.08, 1.0))

    bb_shell = model.part("bb_shell")
    bb_shell.visual(
        mesh_from_geometry(_bb_shell_mesh(), "bb_shell"),
        material=dark_steel,
        name="bb_shell_body",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_geometry(_spindle_mesh(), "spindle"),
        material=dark_steel,
        name="spindle_body",
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        mesh_from_geometry(_arm_mesh(with_chainring=True), "right_crank_assembly"),
        material=anodized_alloy,
        name="right_crank_assembly",
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        mesh_from_geometry(_arm_mesh(with_chainring=False), "left_crank_assembly"),
        material=anodized_alloy,
        name="left_crank_assembly",
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        mesh_from_geometry(_pedal_mesh(side_sign=1.0), "right_pedal"),
        material=pedal_black,
        name="right_pedal_body",
    )
    right_pedal.visual(
        mesh_from_geometry(BoxGeometry((0.026, 0.006, 0.008)), "right_reflector"),
        origin=Origin(xyz=(0.0, 0.086, 0.010)),
        material=reflector_amber,
        name="right_reflector",
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        mesh_from_geometry(_pedal_mesh(side_sign=-1.0), "left_pedal"),
        material=pedal_black,
        name="left_pedal_body",
    )
    left_pedal.visual(
        mesh_from_geometry(BoxGeometry((0.026, 0.006, 0.008)), "left_reflector"),
        origin=Origin(xyz=(0.0, -0.086, 0.010)),
        material=reflector_amber,
        name="left_reflector",
    )

    model.articulation(
        "bb_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.0, 0.057, 0.0)),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(0.0, -0.057, 0.0), rpy=(0.0, math.pi, 0.0)),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bb_shell = object_model.get_part("bb_shell")
    spindle = object_model.get_part("spindle")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    bb_to_spindle = object_model.get_articulation("bb_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    ctx.expect_within(
        spindle,
        bb_shell,
        axes="xz",
        margin=0.005,
        inner_elem="spindle_body",
        outer_elem="bb_shell_body",
        name="spindle stays centered inside the bottom bracket shell",
    )
    ctx.expect_overlap(
        spindle,
        bb_shell,
        axes="y",
        min_overlap=0.070,
        elem_a="spindle_body",
        elem_b="bb_shell_body",
        name="spindle remains retained through the shell",
    )

    rest_right = ctx.part_world_position(right_pedal)
    rest_left = ctx.part_world_position(left_pedal)
    with ctx.pose({bb_to_spindle: math.pi / 2.0}):
        quarter_right = ctx.part_world_position(right_pedal)
        quarter_left = ctx.part_world_position(left_pedal)

    ctx.check(
        "crank rotation sweeps the right pedal around the spindle",
        rest_right is not None
        and quarter_right is not None
        and abs(quarter_right[0] - rest_right[0]) > 0.08
        and abs(quarter_right[2] - rest_right[2]) > 0.08,
        details=f"rest_right={rest_right}, quarter_right={quarter_right}",
    )
    ctx.check(
        "opposed crank arms stay 180 degrees apart",
        rest_right is not None
        and rest_left is not None
        and quarter_right is not None
        and quarter_left is not None
        and abs(rest_right[0] + rest_left[0]) < 0.01
        and abs(rest_right[2] + rest_left[2]) < 0.01
        and abs(quarter_right[0] + quarter_left[0]) < 0.01
        and abs(quarter_right[2] + quarter_left[2]) < 0.01,
        details=(
            f"rest_right={rest_right}, rest_left={rest_left}, "
            f"quarter_right={quarter_right}, quarter_left={quarter_left}"
        ),
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    right_reflector_rest = _aabb_center(ctx.part_element_world_aabb(right_pedal, elem="right_reflector"))
    left_reflector_rest = _aabb_center(ctx.part_element_world_aabb(left_pedal, elem="left_reflector"))
    with ctx.pose({right_pedal_spin: math.pi / 2.0, left_pedal_spin: -math.pi / 2.0}):
        right_reflector_spun = _aabb_center(ctx.part_element_world_aabb(right_pedal, elem="right_reflector"))
        left_reflector_spun = _aabb_center(ctx.part_element_world_aabb(left_pedal, elem="left_reflector"))

    ctx.check(
        "right pedal rotates continuously about its axle",
        right_reflector_rest is not None
        and right_reflector_spun is not None
        and abs(right_reflector_rest[2] - right_reflector_spun[2]) > 0.008,
        details=f"rest={right_reflector_rest}, spun={right_reflector_spun}",
    )
    ctx.check(
        "left pedal rotates continuously about its axle",
        left_reflector_rest is not None
        and left_reflector_spun is not None
        and abs(left_reflector_rest[2] - left_reflector_spun[2]) > 0.008,
        details=f"rest={left_reflector_rest}, spun={left_reflector_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
