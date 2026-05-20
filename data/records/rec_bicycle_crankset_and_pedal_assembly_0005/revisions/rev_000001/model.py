from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, segments: int) -> list[tuple[float, float]]:
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


def _build_ring_mesh(
    outer_radius: float,
    inner_radius: float,
    width: float,
    *,
    outer_segments: int = 56,
    inner_segments: int = 40,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, outer_segments),
        [_circle_profile(inner_radius, inner_segments)],
        height=width,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _build_crank_arm_mesh():
    outer = [
        (0.018, -0.012),
        (0.024, -0.018),
        (0.055, -0.019),
        (0.106, -0.015),
        (0.146, -0.013),
        (0.164, -0.015),
        (0.172, -0.010),
        (0.176, 0.000),
        (0.172, 0.010),
        (0.164, 0.015),
        (0.146, 0.013),
        (0.106, 0.015),
        (0.055, 0.019),
        (0.024, 0.018),
        (0.018, 0.012),
        (0.014, 0.000),
    ]
    inner = [
        (0.036, -0.006),
        (0.044, -0.010),
        (0.084, -0.009),
        (0.125, -0.007),
        (0.146, -0.005),
        (0.150, 0.000),
        (0.146, 0.005),
        (0.125, 0.007),
        (0.084, 0.009),
        (0.044, 0.010),
        (0.036, 0.006),
        (0.032, 0.000),
    ]
    return ExtrudeWithHolesGeometry(
        outer,
        [inner],
        height=0.018,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _build_chainring_mesh():
    window_profile = rounded_rect_profile(0.030, 0.056, radius=0.006, corner_segments=6)
    holes = [_circle_profile(0.024, 36)]
    for index in range(5):
        angle = (2.0 * math.pi * index / 5.0) + (math.pi / 10.0)
        holes.append(
            _transform_profile(
                window_profile,
                dx=math.cos(angle) * 0.057,
                dy=math.sin(angle) * 0.057,
                angle=angle,
            )
        )
    return ExtrudeWithHolesGeometry(
        _circle_profile(0.104, 84),
        holes,
        height=0.004,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _add_platform_pedal(part, *, side_sign: float, alloy, dark_metal):
    pedal_y = side_sign * 0.046
    part.visual(
        Cylinder(radius=0.0060, length=0.020),
        origin=Origin(xyz=(0.0, side_sign * 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="spindle",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, side_sign * 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="inner_hub",
    )
    part.visual(
        Box((0.094, 0.070, 0.004)),
        origin=Origin(xyz=(0.0, pedal_y, 0.012)),
        material=alloy,
        name="top_plate",
    )
    part.visual(
        Box((0.094, 0.070, 0.004)),
        origin=Origin(xyz=(0.0, pedal_y, -0.012)),
        material=alloy,
        name="bottom_plate",
    )
    part.visual(
        Box((0.006, 0.070, 0.024)),
        origin=Origin(xyz=(0.044, pedal_y, 0.0)),
        material=alloy,
        name="front_bar",
    )
    part.visual(
        Box((0.006, 0.070, 0.024)),
        origin=Origin(xyz=(-0.044, pedal_y, 0.0)),
        material=alloy,
        name="rear_bar",
    )
    part.visual(
        Box((0.094, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, side_sign * 0.014, 0.0)),
        material=alloy,
        name="inboard_plate",
    )
    part.visual(
        Box((0.094, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, side_sign * 0.078, 0.0)),
        material=alloy,
        name="outboard_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_crankset", assets=ASSETS)

    forged_alloy = model.material("forged_alloy", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.08, 0.08, 0.09, 1.0))

    arm_mesh = _save_mesh("road_bike_crank_arm.obj", _build_crank_arm_mesh())
    chainring_mesh = _save_mesh("road_bike_chainring.obj", _build_chainring_mesh())
    cup_mesh = _save_mesh("road_bike_bottom_bracket_cup.obj", _build_ring_mesh(0.028, 0.018, 0.012))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(cup_mesh, origin=Origin(xyz=(0.0, -0.040, 0.0)), material=dark_metal, name="right_cup")
    bottom_bracket.visual(cup_mesh, origin=Origin(xyz=(0.0, 0.040, 0.0)), material=dark_metal, name="left_cup")
    bottom_bracket.visual(
        Box((0.008, 0.068, 0.008)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=anodized_black,
        name="connector_right",
    )
    bottom_bracket.visual(
        Box((0.008, 0.068, 0.008)),
        origin=Origin(xyz=(-0.022, 0.0, 0.0)),
        material=anodized_black,
        name="connector_left",
    )
    bottom_bracket.inertial = Inertial.from_geometry(Box((0.070, 0.100, 0.070)), mass=0.28)

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.014, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="spindle_shaft",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_interface",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_interface",
    )
    crankset.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, -0.053, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_arm_root",
    )
    crankset.visual(
        arm_mesh,
        origin=Origin(xyz=(0.0, -0.069, 0.0)),
        material=forged_alloy,
        name="right_arm_panel",
    )
    crankset.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.172, -0.069, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_pedal_boss",
    )
    crankset.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.0, -0.062, 0.0)),
        material=anodized_black,
        name="spider_chainring",
    )
    crankset.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="chainring_hub",
    )
    for index in range(5):
        angle = (2.0 * math.pi * index / 5.0) + (math.pi / 10.0)
        crankset.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(
                xyz=(math.cos(angle) * 0.043, -0.062, math.sin(angle) * 0.043),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=forged_alloy,
            name=f"chainring_bolt_{index}",
        )
    crankset.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_arm_root",
    )
    crankset.visual(
        arm_mesh,
        origin=Origin(xyz=(0.0, 0.069, 0.0), rpy=(0.0, math.pi, 0.0)),
        material=forged_alloy,
        name="left_arm_panel",
    )
    crankset.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(-0.172, 0.069, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_pedal_boss",
    )
    crankset.inertial = Inertial.from_geometry(
        Box((0.360, 0.180, 0.220)),
        mass=1.34,
        origin=Origin(),
    )

    right_pedal = model.part("right_pedal")
    _add_platform_pedal(right_pedal, side_sign=-1.0, alloy=forged_alloy, dark_metal=dark_metal)
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.094, 0.096, 0.024)),
        mass=0.24,
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    _add_platform_pedal(left_pedal, side_sign=1.0, alloy=forged_alloy, dark_metal=dark_metal)
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.094, 0.096, 0.024)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.046, 0.0)),
    )

    model.articulation(
        "bottom_bracket_spin",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=12.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=right_pedal,
        origin=Origin(xyz=(0.172, -0.074, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=left_pedal,
        origin=Origin(xyz=(-0.172, 0.074, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    bottom_bracket_spin = object_model.get_articulation("bottom_bracket_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    right_cup = bottom_bracket.get_visual("right_cup")
    left_cup = bottom_bracket.get_visual("left_cup")
    spindle_shaft = crankset.get_visual("spindle_shaft")
    right_arm_root = crankset.get_visual("right_arm_root")
    right_arm_panel = crankset.get_visual("right_arm_panel")
    left_arm_root = crankset.get_visual("left_arm_root")
    left_arm_panel = crankset.get_visual("left_arm_panel")
    right_pedal_boss = crankset.get_visual("right_pedal_boss")
    left_pedal_boss = crankset.get_visual("left_pedal_boss")
    spider_chainring = crankset.get_visual("spider_chainring")
    right_spindle = right_pedal.get_visual("spindle")
    left_spindle = left_pedal.get_visual("spindle")
    right_top_plate = right_pedal.get_visual("top_plate")
    left_top_plate = left_pedal.get_visual("top_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=12)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_contact(crankset, bottom_bracket, elem_a=right_arm_root, elem_b=right_cup)
    ctx.expect_contact(crankset, bottom_bracket, elem_a=left_arm_root, elem_b=left_cup)
    ctx.expect_contact(right_pedal, crankset, elem_a=right_spindle, elem_b=right_pedal_boss)
    ctx.expect_contact(left_pedal, crankset, elem_a=left_spindle, elem_b=left_pedal_boss)
    ctx.expect_gap(
        bottom_bracket,
        crankset,
        axis="y",
        min_gap=0.001,
        max_gap=0.016,
        positive_elem=right_cup,
        negative_elem=spider_chainring,
    )
    ctx.expect_overlap(
        crankset,
        bottom_bracket,
        axes="xz",
        min_overlap=0.02,
        elem_a=spider_chainring,
    )
    ctx.expect_origin_distance(right_pedal, left_pedal, axes="x", min_dist=0.32)

    with ctx.pose({bottom_bracket_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="quarter_turn_no_floating")
        ctx.expect_contact(crankset, bottom_bracket, elem_a=right_arm_root, elem_b=right_cup)
        ctx.expect_contact(crankset, bottom_bracket, elem_a=left_arm_root, elem_b=left_cup)
        ctx.expect_contact(right_pedal, crankset, elem_a=right_spindle, elem_b=right_pedal_boss)
        ctx.expect_contact(left_pedal, crankset, elem_a=left_spindle, elem_b=left_pedal_boss)
        right_pos = ctx.part_world_position(right_pedal)
        left_pos = ctx.part_world_position(left_pedal)
        assert right_pos is not None
        assert left_pos is not None
        assert right_pos[2] < -0.16
        assert left_pos[2] > 0.16
        assert left_pos[2] - right_pos[2] > 0.34
        assert abs(right_pos[0]) < 0.03
        assert abs(left_pos[0]) < 0.03

    right_top_rest = ctx.part_element_world_aabb(right_pedal, elem=right_top_plate)
    left_top_rest = ctx.part_element_world_aabb(left_pedal, elem=left_top_plate)
    assert right_top_rest is not None
    assert left_top_rest is not None

    with ctx.pose({right_pedal_spin: math.pi / 2.0, left_pedal_spin: -math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pedal_spin_no_overlap")
        ctx.fail_if_isolated_parts(name="pedal_spin_no_floating")
        ctx.expect_contact(right_pedal, crankset, elem_a=right_spindle, elem_b=right_pedal_boss)
        ctx.expect_contact(left_pedal, crankset, elem_a=left_spindle, elem_b=left_pedal_boss)
        right_top_turn = ctx.part_element_world_aabb(right_pedal, elem=right_top_plate)
        left_top_turn = ctx.part_element_world_aabb(left_pedal, elem=left_top_plate)
        assert right_top_turn is not None
        assert left_top_turn is not None
        right_rest_center = tuple((a + b) * 0.5 for a, b in zip(right_top_rest[0], right_top_rest[1]))
        left_rest_center = tuple((a + b) * 0.5 for a, b in zip(left_top_rest[0], left_top_rest[1]))
        right_turn_center = tuple((a + b) * 0.5 for a, b in zip(right_top_turn[0], right_top_turn[1]))
        left_turn_center = tuple((a + b) * 0.5 for a, b in zip(left_top_turn[0], left_top_turn[1]))
        assert abs(right_turn_center[0] - right_rest_center[0]) > 0.008
        assert abs(left_turn_center[0] - left_rest_center[0]) > 0.008
        assert abs(right_turn_center[2] - right_rest_center[2]) > 0.008
        assert abs(left_turn_center[2] - left_rest_center[2]) > 0.008
        ctx.expect_overlap(crankset, right_pedal, axes="xz", min_overlap=0.01, elem_a=right_pedal_boss)
        ctx.expect_overlap(crankset, left_pedal, axes="xz", min_overlap=0.01, elem_a=left_pedal_boss)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
