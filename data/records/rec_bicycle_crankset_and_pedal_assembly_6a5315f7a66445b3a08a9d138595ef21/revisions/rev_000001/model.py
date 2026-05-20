from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _chainring_profile(teeth: int, root_radius: float, tip_radius: float) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for i in range(teeth * 2):
        angle = 2.0 * pi * i / (teeth * 2)
        radius = tip_radius if i % 2 == 0 else root_radius
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _arm_section(thickness: float, width: float, z: float) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(thickness, width, radius=min(thickness, width) * 0.30, corner_segments=6)
    return [(x, y, z) for x, y in profile]


def _arm_mesh() -> object:
    beam = section_loft(
        [
            _arm_section(0.024, 0.040, -0.002),
            _arm_section(0.018, 0.034, -0.030),
            _arm_section(0.013, 0.024, -0.090),
            _arm_section(0.015, 0.020, -0.145),
            _arm_section(0.018, 0.028, -0.172),
        ]
    )
    spindle_boss = CylinderGeometry(radius=0.024, height=0.024, radial_segments=36).rotate_y(pi / 2.0)
    pedal_boss = CylinderGeometry(radius=0.010, height=0.018, radial_segments=28).rotate_y(pi / 2.0).translate(
        0.0, 0.0, -0.172
    )
    gusset = BoxGeometry((0.012, 0.024, 0.030)).translate(0.0, 0.0, -0.015)
    beam.merge(spindle_boss)
    beam.merge(gusset)
    beam.merge(pedal_boss)
    return beam


def _spider_mesh() -> object:
    spider = CylinderGeometry(radius=0.026, height=0.016, radial_segments=40).rotate_y(pi / 2.0)
    spider.merge(CylinderGeometry(radius=0.052, height=0.010, radial_segments=48).rotate_y(pi / 2.0))
    spider.merge(
        CylinderGeometry(radius=0.026, height=0.004, radial_segments=36).rotate_y(pi / 2.0).translate(0.010, 0.0, 0.0)
    )

    arm_length = 0.056
    pad_radius = 0.008
    for angle in (pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0):
        arm = BoxGeometry((0.008, 0.012, arm_length)).rotate_x(angle).translate(
            0.002,
            sin(angle) * arm_length * 0.28,
            cos(angle) * arm_length * 0.28,
        )
        pad = CylinderGeometry(radius=pad_radius, height=0.008, radial_segments=24).rotate_y(pi / 2.0).translate(
            0.006,
            sin(angle) * 0.058,
            cos(angle) * 0.058,
        )
        spider.merge(arm)
        spider.merge(pad)
    return spider


def _bb_shell_mesh() -> object:
    outer_profile = [
        (0.0215, -0.034),
        (0.0225, -0.031),
        (0.0225, 0.031),
        (0.0215, 0.034),
    ]
    inner_profile = [
        (0.0175, -0.034),
        (0.0175, 0.034),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    ).rotate_y(pi / 2.0)


def _tube_mesh(inner_radius: float, outer_radius: float, length: float) -> object:
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
        lip_samples=3,
    ).rotate_y(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_meter_crankset")

    shell_alloy = model.material("shell_alloy", rgba=(0.23, 0.24, 0.26, 1.0))
    forged_alloy = model.material("forged_alloy", rgba=(0.67, 0.69, 0.71, 1.0))
    spider_black = model.material("spider_black", rgba=(0.09, 0.09, 0.10, 1.0))
    chainring_black = model.material("chainring_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    pedal_alloy = model.material("pedal_alloy", rgba=(0.34, 0.35, 0.37, 1.0))

    bb_shell = model.part("bb_shell")
    bb_shell.visual(mesh_from_geometry(_bb_shell_mesh(), "bb_shell_mesh"), material=shell_alloy, name="bb_shell")
    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="spindle_body",
    )
    spindle.visual(
        Cylinder(radius=0.0175, length=0.008),
        origin=Origin(xyz=(-0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_support_collar",
    )
    spindle.visual(
        Cylinder(radius=0.0175, length=0.008),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_support_collar",
    )

    spider = model.part("spider")
    spider.visual(mesh_from_geometry(_spider_mesh(), "spider_mesh"), material=spider_black, name="spider_hub")

    chainring = model.part("chainring")
    chainring_geom = ExtrudeWithHolesGeometry(
        _chainring_profile(teeth=42, root_radius=0.086, tip_radius=0.092),
        [_circle_profile(0.048, segments=56)],
        height=0.004,
        center=True,
    ).rotate_y(pi / 2.0)
    chainring.visual(mesh_from_geometry(chainring_geom, "chainring_mesh"), material=chainring_black, name="chainring_plate")

    right_crank = model.part("right_crank")
    right_crank.visual(mesh_from_geometry(_arm_mesh(), "right_crank_mesh"), material=forged_alloy, name="crank_spindle_boss")

    left_crank = model.part("left_crank")
    left_crank.visual(mesh_from_geometry(_arm_mesh(), "left_crank_mesh"), material=forged_alloy, name="crank_spindle_boss")

    for side_name, side_sign in (("right", 1.0), ("left", -1.0)):
        axle = model.part(f"{side_name}_pedal_axle")
        axle.visual(
            Cylinder(radius=0.0048, length=0.027),
            origin=Origin(xyz=(0.0135 * side_sign, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="pedal_axle",
        )
        axle.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.002 * side_sign, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="pedal_shoulder",
        )
        axle.visual(
            Box((0.010, 0.018, 0.018)),
            origin=Origin(xyz=(0.022 * side_sign, 0.0, 0.0)),
            material=steel,
            name="pedal_float_mount",
        )

        body = model.part(f"{side_name}_pedal_body")
        body.visual(
            Box((0.010, 0.020, 0.018)),
            origin=Origin(xyz=(0.005 * side_sign, 0.0, 0.0)),
            material=pedal_alloy,
            name="pedal_root_block",
        )
        body.visual(
            Box((0.012, 0.072, 0.004)),
            origin=Origin(xyz=(0.016 * side_sign, 0.0, 0.010)),
            material=pedal_alloy,
            name="pedal_outer_plate",
        )
        body.visual(
            Box((0.012, 0.072, 0.004)),
            origin=Origin(xyz=(0.012 * side_sign, 0.0, -0.010)),
            material=pedal_alloy,
            name="pedal_lower_plate",
        )
        body.visual(
            Box((0.010, 0.018, 0.016)),
            origin=Origin(xyz=(0.010 * side_sign, 0.027, 0.0)),
            material=pedal_alloy,
            name="pedal_front_claw",
        )
        body.visual(
            Box((0.010, 0.018, 0.016)),
            origin=Origin(xyz=(0.010 * side_sign, -0.027, 0.0)),
            material=pedal_alloy,
            name="pedal_rear_claw",
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
        "spindle_to_spider",
        ArticulationType.FIXED,
        parent=spindle,
        child=spider,
        origin=Origin(xyz=(0.063, 0.0, 0.0)),
    )
    model.articulation(
        "spider_to_chainring",
        ArticulationType.FIXED,
        parent=spider,
        child=chainring,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )
    model.articulation(
        "spider_to_right_crank",
        ArticulationType.FIXED,
        parent=spider,
        child=right_crank,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(-0.067, 0.0, 0.0), rpy=(pi, 0.0, 0.0)),
    )

    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child="right_pedal_axle",
        origin=Origin(xyz=(0.009, 0.0, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )
    model.articulation(
        "right_pedal_float",
        ArticulationType.REVOLUTE,
        parent="right_pedal_axle",
        child="right_pedal_body",
        origin=Origin(xyz=(0.027, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-0.26, upper=0.26),
    )

    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child="left_pedal_axle",
        origin=Origin(xyz=(-0.009, 0.0, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )
    model.articulation(
        "left_pedal_float",
        ArticulationType.REVOLUTE,
        parent="left_pedal_axle",
        child="left_pedal_body",
        origin=Origin(xyz=(-0.027, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-0.26, upper=0.26),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


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

    spindle = object_model.get_part("spindle")
    spider = object_model.get_part("spider")
    chainring = object_model.get_part("chainring")
    left_crank = object_model.get_part("left_crank")
    right_crank = object_model.get_part("right_crank")
    left_pedal_axle = object_model.get_part("left_pedal_axle")
    left_pedal_body = object_model.get_part("left_pedal_body")
    right_pedal_axle = object_model.get_part("right_pedal_axle")
    right_pedal_body = object_model.get_part("right_pedal_body")

    crank_spin = object_model.get_articulation("bb_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    right_pedal_float = object_model.get_articulation("right_pedal_float")

    ctx.expect_contact(
        spindle,
        spider,
        elem_a="spindle_body",
        elem_b="spider_hub",
        name="spider seats against the spindle",
    )
    ctx.expect_contact(
        spider,
        right_crank,
        elem_a="spider_hub",
        elem_b="crank_spindle_boss",
        name="right crank clamps against the spider",
    )
    ctx.expect_contact(
        spindle,
        left_crank,
        elem_a="spindle_body",
        elem_b="crank_spindle_boss",
        name="left crank seats directly on the spindle",
    )
    ctx.expect_contact(
        spider,
        chainring,
        elem_a="spider_hub",
        elem_b="chainring_plate",
        name="chainring mounts to the spider",
    )
    ctx.expect_contact(
        right_pedal_axle,
        right_pedal_body,
        elem_a="pedal_float_mount",
        elem_b="pedal_root_block",
        name="right pedal body seats on the float mount",
    )
    ctx.expect_contact(
        left_pedal_axle,
        left_pedal_body,
        elem_a="pedal_float_mount",
        elem_b="pedal_root_block",
        name="left pedal body seats on the float mount",
    )
    ctx.expect_origin_gap(
        left_pedal_axle,
        right_pedal_axle,
        axis="z",
        min_gap=0.30,
        name="left and right pedals start on opposite sides of the crank circle",
    )

    rest_right_pedal = ctx.part_world_position(right_pedal_axle)
    with ctx.pose({crank_spin: pi / 2.0}):
        quarter_turn_right_pedal = ctx.part_world_position(right_pedal_axle)
    ctx.check(
        "crank spin carries the drive-side pedal forward",
        rest_right_pedal is not None
        and quarter_turn_right_pedal is not None
        and quarter_turn_right_pedal[1] > rest_right_pedal[1] + 0.15
        and quarter_turn_right_pedal[2] > rest_right_pedal[2] + 0.15,
        details=f"rest={rest_right_pedal}, quarter_turn={quarter_turn_right_pedal}",
    )

    front_claw_rest = _aabb_center(ctx.part_element_world_aabb(right_pedal_body, elem="pedal_front_claw"))
    with ctx.pose({right_pedal_spin: pi / 2.0}):
        front_claw_spun = _aabb_center(ctx.part_element_world_aabb(right_pedal_body, elem="pedal_front_claw"))
    ctx.check(
        "pedal spins about its axle",
        front_claw_rest is not None
        and front_claw_spun is not None
        and front_claw_spun[2] > front_claw_rest[2] + 0.018,
        details=f"rest={front_claw_rest}, spun={front_claw_spun}",
    )

    outer_plate_rest = _aabb_center(ctx.part_element_world_aabb(right_pedal_body, elem="pedal_outer_plate"))
    with ctx.pose({right_pedal_float: 0.22}):
        outer_plate_floated = _aabb_center(ctx.part_element_world_aabb(right_pedal_body, elem="pedal_outer_plate"))
    ctx.check(
        "pedal float tilts the body about the pedal long axis",
        outer_plate_rest is not None
        and outer_plate_floated is not None
        and outer_plate_floated[2] < outer_plate_rest[2] - 0.002,
        details=f"rest={outer_plate_rest}, floated={outer_plate_floated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
