from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.05
BASE_WIDTH = 0.26
BASE_HEIGHT = 0.04
RAIL_LENGTH = 0.92
RAIL_WIDTH = 0.04
RAIL_HEIGHT = 0.05
RAIL_CENTER_Y = 0.075
BASE_END_BLOCK_LENGTH = 0.12
BASE_END_BLOCK_HEIGHT = 0.05

SLIDE_TRAVEL = 0.52
SLIDE_HOME_X = -SLIDE_TRAVEL / 2.0

SHOULDER_AXIS_Z = BASE_HEIGHT + RAIL_HEIGHT + 0.10
SHOULDER_HUB_RADIUS = 0.042
SHOULDER_HUB_LENGTH = 0.084
YOKE_PLATE_THICKNESS = 0.012

CARRIAGE_TOWER_LENGTH = 0.10
CARRIAGE_SADDLE_LENGTH = 0.16
CARRIAGE_SHOE_LENGTH = 0.18
CARRIAGE_SHOE_HEIGHT = 0.024

UPPER_LINK_LENGTH = 0.455
UPPER_LINK_BEAM_WIDTH = 0.055
UPPER_LINK_BEAM_HEIGHT = 0.07

ELBOW_HUB_RADIUS = 0.035
ELBOW_HUB_LENGTH = 0.054
FORELINK_LENGTH = 0.42
FORELINK_BEAM_WIDTH = 0.05
FORELINK_BEAM_HEIGHT = 0.06


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _cylinder_y(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _base_shape() -> cq.Workplane:
    plate = _box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, (0.0, 0.0, BASE_HEIGHT / 2.0))
    left_rail = _box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        RAIL_HEIGHT,
        (0.0, RAIL_CENTER_Y, BASE_HEIGHT + RAIL_HEIGHT / 2.0),
    )
    right_rail = _box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        RAIL_HEIGHT,
        (0.0, -RAIL_CENTER_Y, BASE_HEIGHT + RAIL_HEIGHT / 2.0),
    )
    left_end_block = _box(
        BASE_END_BLOCK_LENGTH,
        BASE_WIDTH * 0.82,
        BASE_END_BLOCK_HEIGHT,
        (
            -(BASE_LENGTH - BASE_END_BLOCK_LENGTH) / 2.0,
            0.0,
            BASE_HEIGHT + BASE_END_BLOCK_HEIGHT / 2.0,
        ),
    )
    right_end_block = _box(
        BASE_END_BLOCK_LENGTH,
        BASE_WIDTH * 0.82,
        BASE_END_BLOCK_HEIGHT,
        (
            (BASE_LENGTH - BASE_END_BLOCK_LENGTH) / 2.0,
            0.0,
            BASE_HEIGHT + BASE_END_BLOCK_HEIGHT / 2.0,
        ),
    )
    left_foot = _box(0.16, 0.08, 0.018, (-0.31, 0.0, 0.009))
    right_foot = _box(0.16, 0.08, 0.018, (0.31, 0.0, 0.009))
    return (
        plate.union(left_rail)
        .union(right_rail)
        .union(left_end_block)
        .union(right_end_block)
        .union(left_foot)
        .union(right_foot)
    )


def _carriage_shape() -> cq.Workplane:
    left_shoe = _box(CARRIAGE_SHOE_LENGTH, 0.048, CARRIAGE_SHOE_HEIGHT, (-0.10, RAIL_CENTER_Y, -0.088))
    right_shoe = _box(CARRIAGE_SHOE_LENGTH, 0.048, CARRIAGE_SHOE_HEIGHT, (-0.10, -RAIL_CENTER_Y, -0.088))
    left_saddle = _box(CARRIAGE_SADDLE_LENGTH, 0.064, 0.028, (-0.10, RAIL_CENTER_Y, -0.064))
    right_saddle = _box(CARRIAGE_SADDLE_LENGTH, 0.064, 0.028, (-0.10, -RAIL_CENTER_Y, -0.064))
    bridge = _box(0.18, 0.13, 0.028, (-0.10, 0.0, -0.048))
    rear_column = _box(0.05, 0.10, 0.085, (-0.16, 0.0, -0.004))
    throat_fill = _box(0.04, 0.05, 0.05, (-0.03, 0.0, -0.012))
    shoulder_mount = _box(0.012, 0.06, 0.024, (-0.006, 0.0, 0.0))

    return (
        left_shoe.union(right_shoe)
        .union(left_saddle)
        .union(right_saddle)
        .union(bridge)
        .union(rear_column)
        .union(shoulder_mount)
        .union(throat_fill)
    )


def _shoulder_link_shape() -> cq.Workplane:
    root_pad = _box(0.012, 0.05, 0.024, (0.006, 0.0, 0.0))
    root_neck = _box(0.08, 0.04, 0.028, (0.05, 0.0, 0.0))
    main_beam = _box(0.28, 0.045, 0.045, (0.21, 0.0, 0.0))
    elbow_neck = _box(0.11, 0.04, 0.03, (0.4025, 0.0, 0.0))
    elbow_mount = _box(0.012, 0.05, 0.024, (UPPER_LINK_LENGTH - 0.006, 0.0, 0.0))

    return root_pad.union(root_neck).union(main_beam).union(elbow_neck).union(elbow_mount)


def _forelink_shape() -> cq.Workplane:
    root_pad = _box(0.012, 0.05, 0.024, (0.006, 0.0, 0.0))
    root_neck = _box(0.08, 0.036, 0.026, (0.05, 0.0, 0.0))
    main_beam = _box(0.26, 0.04, 0.042, (0.19, 0.0, 0.0))
    lower_web = _box(0.11, 0.03, 0.014, (0.10, 0.0, -0.02))
    tip_block = _box(0.10, 0.07, 0.055, (0.37, 0.0, 0.0))
    return root_pad.union(root_neck).union(main_beam).union(lower_web).union(tip_block)


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_shoulder_elbow_chain")

    base_color = model.material("base_gray", rgba=(0.24, 0.25, 0.28, 1.0))
    carriage_color = model.material("carriage_gray", rgba=(0.47, 0.49, 0.52, 1.0))
    arm_color = model.material("arm_orange", rgba=(0.88, 0.42, 0.14, 1.0))
    fore_color = model.material("fore_gray", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_slide"),
        origin=Origin(),
        material=base_color,
        name="base_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        origin=Origin(),
        material=carriage_color,
        name="carriage_body",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        mesh_from_cadquery(_shoulder_link_shape(), "shoulder_link"),
        origin=Origin(),
        material=arm_color,
        name="shoulder_body",
    )

    forelink = model.part("forelink")
    forelink.visual(
        mesh_from_cadquery(_forelink_shape(), "forelink"),
        origin=Origin(),
        material=fore_color,
        name="forelink_body",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.5,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=1.5,
            lower=-0.55,
            upper=1.25,
        ),
    )

    model.articulation(
        "shoulder_to_forelink",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forelink,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=2.0,
            lower=0.0,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    shoulder_link = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")

    slide = object_model.get_articulation("base_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_shoulder")
    elbow = object_model.get_articulation("shoulder_to_forelink")

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

    ctx.expect_contact(carriage, base, name="carriage_contacts_rail_in_home_pose")
    ctx.expect_contact(shoulder_link, carriage, name="shoulder_joint_has_physical_mount")
    ctx.expect_contact(forelink, shoulder_link, name="elbow_joint_has_physical_mount")

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(carriage, base, name="carriage_contacts_rail_at_max_slide")

    home_carriage_x = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_carriage_x = ctx.part_world_position(carriage)
    ctx.check(
        "slide_moves_carriage_along_positive_x",
        home_carriage_x is not None
        and extended_carriage_x is not None
        and extended_carriage_x[0] - home_carriage_x[0] > SLIDE_TRAVEL - 1e-4,
        details=f"home={home_carriage_x}, extended={extended_carriage_x}",
    )

    with ctx.pose({slide: SLIDE_TRAVEL * 0.35, shoulder: 0.0, elbow: 0.0}):
        shoulder_rest_z = _aabb_center_z(ctx.part_element_world_aabb(shoulder_link, elem="shoulder_body"))
    with ctx.pose({slide: SLIDE_TRAVEL * 0.35, shoulder: 0.9, elbow: 0.0}):
        shoulder_raised_z = _aabb_center_z(ctx.part_element_world_aabb(shoulder_link, elem="shoulder_body"))
    ctx.check(
        "positive_shoulder_rotation_raises_upper_link",
        shoulder_rest_z is not None
        and shoulder_raised_z is not None
        and shoulder_raised_z > shoulder_rest_z + 0.10,
        details=f"rest_z={shoulder_rest_z}, raised_z={shoulder_raised_z}",
    )

    with ctx.pose({slide: SLIDE_TRAVEL * 0.35, shoulder: 0.35, elbow: 0.0}):
        fore_rest_z = _aabb_center_z(ctx.part_element_world_aabb(forelink, elem="forelink_body"))
    with ctx.pose({slide: SLIDE_TRAVEL * 0.35, shoulder: 0.35, elbow: 0.9}):
        fore_bent_z = _aabb_center_z(ctx.part_element_world_aabb(forelink, elem="forelink_body"))
        ctx.expect_contact(shoulder_link, carriage, name="shoulder_mount_stays_connected_in_pose")
        ctx.expect_contact(forelink, shoulder_link, name="elbow_mount_stays_connected_in_pose")
    ctx.check(
        "positive_elbow_rotation_lifts_forelink",
        fore_rest_z is not None and fore_bent_z is not None and fore_bent_z > fore_rest_z + 0.015,
        details=f"rest_z={fore_rest_z}, bent_z={fore_bent_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
