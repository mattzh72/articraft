from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PLATFORM_WIDTH = 2.15
PLATFORM_LENGTH = 2.35
LIP_LENGTH = 0.42
GUIDE_CENTER_Y = 1.55
GUIDE_CENTER_X = 0.99
GUIDE_ROOF_THICKNESS = 0.045
GUIDE_HEIGHT = 0.26


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _box_geom(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _cylinder_x(
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    radial_segments: int = 28,
) -> MeshGeometry:
    return (
        CylinderGeometry(radius=radius, height=length, radial_segments=radial_segments)
        .rotate_y(math.pi / 2.0)
        .translate(*center)
    )


def _build_bracket_mesh() -> MeshGeometry:
    width = PLATFORM_WIDTH + 0.22
    cheek_x = 0.5 * PLATFORM_WIDTH + 0.02
    return _merge_geometries(
        _box_geom((width, 0.06, 0.46), (0.0, -0.19, 0.03)),
        _box_geom((PLATFORM_WIDTH + 0.32, 0.18, 0.10), (0.0, -0.09, -0.13)),
        _box_geom((0.16, 0.14, 0.24), (cheek_x, -0.08, -0.08)),
        _box_geom((0.16, 0.14, 0.24), (-cheek_x, -0.08, -0.08)),
        _box_geom((0.34, 0.10, 0.16), (0.0, -0.09, -0.07)),
        _cylinder_x(0.038, PLATFORM_WIDTH - 0.30, (0.0, -0.055, -0.005)),
    )


def _build_platform_mesh() -> MeshGeometry:
    rib_length = PLATFORM_LENGTH - 0.22
    rib_x_positions = (-0.80, -0.60, -0.40, -0.20, 0.0, 0.20, 0.40, 0.60, 0.80)

    geometries = [
        _box_geom((PLATFORM_WIDTH, PLATFORM_LENGTH, 0.012), (0.0, PLATFORM_LENGTH / 2.0, 0.134)),
        _box_geom((0.10, PLATFORM_LENGTH, 0.14), (PLATFORM_WIDTH / 2.0 - 0.05, PLATFORM_LENGTH / 2.0, 0.07)),
        _box_geom((0.10, PLATFORM_LENGTH, 0.14), (-PLATFORM_WIDTH / 2.0 + 0.05, PLATFORM_LENGTH / 2.0, 0.07)),
        _box_geom((PLATFORM_WIDTH, 0.10, 0.13), (0.0, PLATFORM_LENGTH - 0.05, 0.065)),
        _box_geom((PLATFORM_WIDTH - 0.16, 0.14, 0.12), (0.0, 0.07, 0.06)),
        _box_geom((PLATFORM_WIDTH - 0.24, 0.11, 0.11), (0.0, 0.55, 0.055)),
        _box_geom((PLATFORM_WIDTH - 0.24, 0.11, 0.11), (0.0, 1.10, 0.055)),
        _box_geom((PLATFORM_WIDTH - 0.24, 0.11, 0.11), (0.0, 1.65, 0.055)),
    ]

    for x_pos in rib_x_positions:
        geometries.append(_box_geom((0.045, rib_length, 0.009), (x_pos, PLATFORM_LENGTH / 2.0, 0.1445)))

    return _merge_geometries(*geometries)


def _build_guide_mesh() -> MeshGeometry:
    wall = 0.015
    width = 0.11
    depth = 0.22
    return _merge_geometries(
        _box_geom((width, depth, GUIDE_ROOF_THICKNESS), (0.0, 0.0, -GUIDE_ROOF_THICKNESS / 2.0)),
        _box_geom((wall, depth, GUIDE_HEIGHT), (width / 2.0 - wall / 2.0, 0.0, -GUIDE_HEIGHT / 2.0)),
        _box_geom((wall, depth, GUIDE_HEIGHT), (-width / 2.0 + wall / 2.0, 0.0, -GUIDE_HEIGHT / 2.0)),
        _box_geom((width - 2.0 * wall, wall, GUIDE_HEIGHT - GUIDE_ROOF_THICKNESS), (0.0, depth / 2.0 - wall / 2.0, -(GUIDE_HEIGHT + GUIDE_ROOF_THICKNESS) / 2.0)),
        _box_geom((width - 2.0 * wall, wall, GUIDE_HEIGHT - GUIDE_ROOF_THICKNESS), (0.0, -depth / 2.0 + wall / 2.0, -(GUIDE_HEIGHT + GUIDE_ROOF_THICKNESS) / 2.0)),
    )


def _build_leg_mesh() -> MeshGeometry:
    return _merge_geometries(
        _box_geom((0.074, 0.170, 0.03), (0.0, 0.0, -0.060)),
        _box_geom((0.060, 0.140, 0.62), (0.0, 0.0, -0.385)),
    )


def _build_lip_mesh() -> MeshGeometry:
    return _merge_geometries(
        _box_geom((PLATFORM_WIDTH - 0.08, 0.34, 0.020), (0.0, 0.17, -0.010)),
        _box_geom((PLATFORM_WIDTH - 0.12, 0.08, 0.010), (0.0, 0.38, -0.005)),
        _box_geom((0.09, 0.22, 0.06), (PLATFORM_WIDTH / 2.0 - 0.14, 0.15, -0.050)),
        _box_geom((0.09, 0.22, 0.06), (-PLATFORM_WIDTH / 2.0 + 0.14, 0.15, -0.050)),
        _cylinder_x(0.028, PLATFORM_WIDTH - 0.24, (0.0, 0.03, -0.028)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="container_dock_leveler")

    bracket_paint = model.material("bracket_paint", rgba=(0.19, 0.20, 0.22, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.45, 0.47, 0.49, 1.0))
    lip_steel = model.material("lip_steel", rgba=(0.53, 0.54, 0.55, 1.0))
    leg_steel = model.material("leg_steel", rgba=(0.60, 0.61, 0.62, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.26, 0.27, 0.29, 1.0))

    dock_bracket = model.part("dock_bracket")
    dock_bracket.visual(
        mesh_from_geometry(_build_bracket_mesh(), "dock_leveler_bracket"),
        material=bracket_paint,
        name="bracket_body",
    )
    dock_bracket.visual(
        Box((PLATFORM_WIDTH + 0.10, 0.26, 0.05)),
        origin=Origin(xyz=(0.0, 0.04, -0.025)),
        material=bracket_paint,
        name="support_shelf",
    )
    dock_bracket.inertial = Inertial.from_geometry(
        Box((PLATFORM_WIDTH + 0.28, 0.34, 0.46)),
        mass=180.0,
        origin=Origin(xyz=(0.0, -0.03, -0.02)),
    )

    platform = model.part("platform")
    platform.visual(
        mesh_from_geometry(_build_platform_mesh(), "dock_leveler_platform"),
        material=deck_steel,
        name="deck_frame",
    )
    platform.visual(
        Box((0.12, 0.24, GUIDE_ROOF_THICKNESS)),
        origin=Origin(xyz=(-GUIDE_CENTER_X, GUIDE_CENTER_Y, -GUIDE_ROOF_THICKNESS / 2.0)),
        material=deck_steel,
        name="left_leg_guide",
    )
    platform.visual(
        Box((0.015, 0.24, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(-GUIDE_CENTER_X + 0.0525, GUIDE_CENTER_Y, -GUIDE_HEIGHT / 2.0),
        ),
        material=deck_steel,
        name="left_leg_guide_outer_wall",
    )
    platform.visual(
        Box((0.015, 0.24, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(-GUIDE_CENTER_X - 0.0525, GUIDE_CENTER_Y, -GUIDE_HEIGHT / 2.0),
        ),
        material=deck_steel,
        name="left_leg_guide_inner_wall",
    )
    platform.visual(
        Box((0.09, 0.015, GUIDE_HEIGHT - GUIDE_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(-GUIDE_CENTER_X, GUIDE_CENTER_Y + 0.1125, -(GUIDE_HEIGHT + GUIDE_ROOF_THICKNESS) / 2.0),
        ),
        material=deck_steel,
        name="left_leg_guide_front_wall",
    )
    platform.visual(
        Box((0.09, 0.015, GUIDE_HEIGHT - GUIDE_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(-GUIDE_CENTER_X, GUIDE_CENTER_Y - 0.1125, -(GUIDE_HEIGHT + GUIDE_ROOF_THICKNESS) / 2.0),
        ),
        material=deck_steel,
        name="left_leg_guide_back_wall",
    )
    platform.visual(
        Box((0.12, 0.24, GUIDE_ROOF_THICKNESS)),
        origin=Origin(xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y, -GUIDE_ROOF_THICKNESS / 2.0)),
        material=deck_steel,
        name="right_leg_guide",
    )
    platform.visual(
        Box((0.015, 0.24, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(GUIDE_CENTER_X - 0.0525, GUIDE_CENTER_Y, -GUIDE_HEIGHT / 2.0),
        ),
        material=deck_steel,
        name="right_leg_guide_inner_wall",
    )
    platform.visual(
        Box((0.015, 0.24, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(GUIDE_CENTER_X + 0.0525, GUIDE_CENTER_Y, -GUIDE_HEIGHT / 2.0),
        ),
        material=deck_steel,
        name="right_leg_guide_outer_wall",
    )
    platform.visual(
        Box((0.09, 0.015, GUIDE_HEIGHT - GUIDE_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y + 0.1125, -(GUIDE_HEIGHT + GUIDE_ROOF_THICKNESS) / 2.0),
        ),
        material=deck_steel,
        name="right_leg_guide_front_wall",
    )
    platform.visual(
        Box((0.09, 0.015, GUIDE_HEIGHT - GUIDE_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y - 0.1125, -(GUIDE_HEIGHT + GUIDE_ROOF_THICKNESS) / 2.0),
        ),
        material=deck_steel,
        name="right_leg_guide_back_wall",
    )
    platform.inertial = Inertial.from_geometry(
        Box((PLATFORM_WIDTH, PLATFORM_LENGTH, 0.16)),
        mass=320.0,
        origin=Origin(xyz=(0.0, PLATFORM_LENGTH / 2.0, 0.08)),
    )

    lip_mesh = mesh_from_geometry(_build_lip_mesh(), "dock_leveler_approach_lip")
    approach_lip = model.part("approach_lip")
    approach_lip.visual(lip_mesh, material=lip_steel, name="lip_plate")
    approach_lip.inertial = Inertial.from_geometry(
        Box((PLATFORM_WIDTH - 0.08, LIP_LENGTH, 0.08)),
        mass=58.0,
        origin=Origin(xyz=(0.0, LIP_LENGTH / 2.0, -0.03)),
    )

    left_leg = model.part("left_support_leg")
    left_leg.visual(
        Box((0.074, 0.170, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=leg_steel,
        name="leg_carriage",
    )
    left_leg.visual(
        Box((0.060, 0.140, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
        material=leg_steel,
        name="leg_post",
    )
    left_leg.visual(
        Box((0.16, 0.24, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.709)),
        material=foot_dark,
        name="foot_pad",
    )
    left_leg.inertial = Inertial.from_geometry(
        Box((0.16, 0.24, 0.678)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -0.384)),
    )

    right_leg = model.part("right_support_leg")
    right_leg.visual(
        Box((0.074, 0.170, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=leg_steel,
        name="leg_carriage",
    )
    right_leg.visual(
        Box((0.060, 0.140, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
        material=leg_steel,
        name="leg_post",
    )
    right_leg.visual(
        Box((0.16, 0.24, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.709)),
        material=foot_dark,
        name="foot_pad",
    )
    right_leg.inertial = Inertial.from_geometry(
        Box((0.16, 0.24, 0.678)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, -0.384)),
    )

    model.articulation(
        "platform_pivot",
        ArticulationType.REVOLUTE,
        parent=dock_bracket,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.9,
            lower=-0.22,
            upper=0.60,
        ),
    )
    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=approach_lip,
        origin=Origin(xyz=(0.0, PLATFORM_LENGTH, 0.14)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5000.0,
            velocity=1.6,
            lower=-1.35,
            upper=0.30,
        ),
    )
    model.articulation(
        "left_leg_slide",
        ArticulationType.PRISMATIC,
        parent=platform,
        child=left_leg,
        origin=Origin(xyz=(-GUIDE_CENTER_X, GUIDE_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=0.30,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "right_leg_slide",
        ArticulationType.PRISMATIC,
        parent=platform,
        child=right_leg,
        origin=Origin(xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=0.30,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dock_bracket = object_model.get_part("dock_bracket")
    platform = object_model.get_part("platform")
    approach_lip = object_model.get_part("approach_lip")
    left_leg = object_model.get_part("left_support_leg")
    right_leg = object_model.get_part("right_support_leg")

    platform_pivot = object_model.get_articulation("platform_pivot")
    lip_hinge = object_model.get_articulation("lip_hinge")
    left_leg_slide = object_model.get_articulation("left_leg_slide")
    right_leg_slide = object_model.get_articulation("right_leg_slide")

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
    ctx.check(
        "platform pivot uses horizontal axis",
        tuple(platform_pivot.axis) == (1.0, 0.0, 0.0),
        f"platform pivot axis was {platform_pivot.axis!r}",
    )
    ctx.check(
        "lip hinge uses horizontal axis",
        tuple(lip_hinge.axis) == (1.0, 0.0, 0.0),
        f"lip hinge axis was {lip_hinge.axis!r}",
    )
    ctx.check(
        "side legs slide vertically",
        tuple(left_leg_slide.axis) == (0.0, 0.0, -1.0)
        and tuple(right_leg_slide.axis) == (0.0, 0.0, -1.0),
        (
            "expected both leg prismatic axes to be vertical downward, got "
            f"{left_leg_slide.axis!r} and {right_leg_slide.axis!r}"
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        platform,
        dock_bracket,
        axis="z",
        positive_elem="deck_frame",
        negative_elem="support_shelf",
        max_gap=0.0015,
        max_penetration=0.0,
        name="platform sits on the rear support shelf",
    )
    ctx.expect_gap(
        approach_lip,
        platform,
        axis="y",
        positive_elem="lip_plate",
        negative_elem="deck_frame",
        max_gap=0.0015,
        max_penetration=0.0,
        name="approach lip starts at the front deck edge",
    )
    ctx.expect_contact(
        left_leg,
        platform,
        elem_a="leg_carriage",
        elem_b="left_leg_guide",
        name="left support leg is captured by its guide sleeve",
    )
    ctx.expect_contact(
        right_leg,
        platform,
        elem_a="leg_carriage",
        elem_b="right_leg_guide",
        name="right support leg is captured by its guide sleeve",
    )
    ctx.expect_origin_distance(
        left_leg,
        right_leg,
        axes="x",
        min_dist=1.80,
        max_dist=2.05,
        name="support legs span the platform width",
    )

    deck_rest = ctx.part_element_world_aabb(platform, elem="deck_frame")
    lip_rest = ctx.part_element_world_aabb(approach_lip, elem="lip_plate")
    left_leg_rest = ctx.part_element_world_aabb(left_leg, elem="leg_post")
    right_leg_rest = ctx.part_element_world_aabb(right_leg, elem="leg_post")
    assert deck_rest is not None
    assert lip_rest is not None
    assert left_leg_rest is not None
    assert right_leg_rest is not None

    with ctx.pose({platform_pivot: 0.42}):
        deck_raised = ctx.part_element_world_aabb(platform, elem="deck_frame")
        assert deck_raised is not None
        ctx.check(
            "platform pivot raises the front edge",
            deck_raised[1][2] > deck_rest[1][2] + 0.70,
            f"raised deck max z {deck_raised[1][2]:.3f} did not exceed rest {deck_rest[1][2]:.3f} by 0.70 m",
        )

    with ctx.pose({lip_hinge: -1.05}):
        lip_dropped = ctx.part_element_world_aabb(approach_lip, elem="lip_plate")
        assert lip_dropped is not None
        ctx.check(
            "approach lip folds downward",
            lip_dropped[0][2] < lip_rest[0][2] - 0.30,
            f"dropped lip min z {lip_dropped[0][2]:.3f} was not at least 0.30 m below rest {lip_rest[0][2]:.3f}",
        )

    with ctx.pose({left_leg_slide: 0.18}):
        left_leg_extended = ctx.part_element_world_aabb(left_leg, elem="leg_post")
        assert left_leg_extended is not None
        ctx.check(
            "left support leg telescopes downward",
            left_leg_extended[0][2] < left_leg_rest[0][2] - 0.16,
            (
                f"left leg min z moved from {left_leg_rest[0][2]:.3f} to "
                f"{left_leg_extended[0][2]:.3f}"
            ),
        )

    with ctx.pose({right_leg_slide: 0.18}):
        right_leg_extended = ctx.part_element_world_aabb(right_leg, elem="leg_post")
        assert right_leg_extended is not None
        ctx.check(
            "right support leg telescopes downward",
            right_leg_extended[0][2] < right_leg_rest[0][2] - 0.16,
            (
                f"right leg min z moved from {right_leg_rest[0][2]:.3f} to "
                f"{right_leg_extended[0][2]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
