from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PIN_RADIUS = 0.003
PIN_HOLE_RADIUS = 0.0034
PIN_END_PROUD = 0.0015
PIN_HEAD_RADIUS = 0.005
PIN_HEAD_THICKNESS = 0.0016

CHEEK_THICKNESS = 0.006
CHEEK_INNER_GAP = 0.012
CLEVIS_WIDTH = CHEEK_INNER_GAP + 2.0 * CHEEK_THICKNESS

BARREL_RADIUS = 0.008
OUTER_BARREL_LENGTH = 0.004

REAR_BLOCK_LENGTH = 0.020
REAR_BLOCK_HEIGHT = 0.038
CHEEK_ARM_LENGTH = 0.030
CHEEK_HEIGHT = 0.022

TAB_KNUCKLE_LENGTH = CHEEK_INNER_GAP
TAB_WIDTH = 0.0095
TAB_THICKNESS = 0.0045
TAB_LENGTH = 0.090
TAB_HOLE_RADIUS = 0.0033
TAB_HOLE_X = 0.072


def _y_axis_cylinder(radius: float, length: float, *, y_center: float = 0.0, x: float = 0.0, z: float = 0.0):
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(x, y_center - length / 2.0, z),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )


def _clevis_body_shape():
    cheek_center_y = CHEEK_INNER_GAP / 2.0 + CHEEK_THICKNESS / 2.0
    cheek_center_x = -CHEEK_ARM_LENGTH / 2.0
    rear_block_center_x = -(CHEEK_ARM_LENGTH + REAR_BLOCK_LENGTH) / 2.0
    outer_barrel_center_y = CHEEK_INNER_GAP / 2.0 + CHEEK_THICKNESS + OUTER_BARREL_LENGTH / 2.0

    rear_block = cq.Workplane("XY").box(
        REAR_BLOCK_LENGTH,
        CLEVIS_WIDTH,
        REAR_BLOCK_HEIGHT,
    ).translate((rear_block_center_x, 0.0, 0.0))

    left_cheek = cq.Workplane("XY").box(CHEEK_ARM_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT).translate(
        (cheek_center_x, cheek_center_y, 0.0)
    )
    right_cheek = cq.Workplane("XY").box(CHEEK_ARM_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT).translate(
        (cheek_center_x, -cheek_center_y, 0.0)
    )

    left_barrel = _y_axis_cylinder(BARREL_RADIUS, OUTER_BARREL_LENGTH, y_center=outer_barrel_center_y)
    right_barrel = _y_axis_cylinder(BARREL_RADIUS, OUTER_BARREL_LENGTH, y_center=-outer_barrel_center_y)

    body = (
        rear_block.union(left_cheek)
        .union(right_cheek)
        .union(left_barrel)
        .union(right_barrel)
        .clean()
    )

    pin_bore = _y_axis_cylinder(PIN_HOLE_RADIUS, CLEVIS_WIDTH + 2.0 * OUTER_BARREL_LENGTH + 0.002)
    return body.cut(pin_bore).clean()


def _moving_tab_shape():
    plate_start_x = 0.0035
    straight_length = TAB_LENGTH - plate_start_x - TAB_WIDTH / 2.0
    plate = (
        cq.Workplane("XY")
        .box(straight_length, TAB_WIDTH, TAB_THICKNESS, centered=(False, True, True))
        .translate((plate_start_x, 0.0, 0.0))
        .union(
            cq.Workplane("XY")
            .circle(TAB_WIDTH / 2.0)
            .extrude(TAB_THICKNESS, both=True)
            .translate((TAB_LENGTH - TAB_WIDTH / 2.0, 0.0, 0.0))
        )
    )

    mounting_hole = (
        cq.Workplane("XY")
        .circle(TAB_HOLE_RADIUS)
        .extrude(TAB_THICKNESS * 3.0, both=True)
        .translate((TAB_HOLE_X, 0.0, 0.0))
    )
    leaf = plate.cut(mounting_hole)

    knuckle_outer = _y_axis_cylinder(BARREL_RADIUS * 0.92, TAB_KNUCKLE_LENGTH)
    knuckle_inner = _y_axis_cylinder(PIN_HOLE_RADIUS, TAB_KNUCKLE_LENGTH + 0.002)

    return knuckle_outer.union(leaf).cut(knuckle_inner).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_and_tab_hinge")

    model.material("bracket_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("leaf_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("pin_steel", rgba=(0.79, 0.81, 0.84, 1.0))

    fixed_clevis = model.part("fixed_clevis")
    fixed_clevis.visual(
        Box((REAR_BLOCK_LENGTH, CLEVIS_WIDTH, REAR_BLOCK_HEIGHT)),
        origin=Origin(xyz=(-(CHEEK_ARM_LENGTH + REAR_BLOCK_LENGTH) / 2.0, 0.0, 0.0)),
        material="bracket_steel",
        name="rear_block",
    )
    cheek_center_y = CHEEK_INNER_GAP / 2.0 + CHEEK_THICKNESS / 2.0
    cheek_center_x = -CHEEK_ARM_LENGTH / 2.0
    fixed_clevis.visual(
        Box((CHEEK_ARM_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(xyz=(cheek_center_x, cheek_center_y, 0.0)),
        material="bracket_steel",
        name="left_cheek",
    )
    fixed_clevis.visual(
        Box((CHEEK_ARM_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(xyz=(cheek_center_x, -cheek_center_y, 0.0)),
        material="bracket_steel",
        name="right_cheek",
    )
    outer_barrel_center_y = CHEEK_INNER_GAP / 2.0 + CHEEK_THICKNESS + OUTER_BARREL_LENGTH / 2.0
    fixed_clevis.visual(
        Cylinder(radius=BARREL_RADIUS, length=OUTER_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, outer_barrel_center_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bracket_steel",
        name="left_barrel",
    )
    fixed_clevis.visual(
        Cylinder(radius=BARREL_RADIUS, length=OUTER_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, -outer_barrel_center_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bracket_steel",
        name="right_barrel",
    )
    pin_length = CLEVIS_WIDTH + 2.0 * OUTER_BARREL_LENGTH + 2.0 * PIN_END_PROUD
    fixed_clevis.visual(
        Cylinder(radius=PIN_RADIUS, length=pin_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="hinge_pin_shaft",
    )
    fixed_clevis.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(
            xyz=(0.0, pin_length / 2.0 - PIN_HEAD_THICKNESS / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="pin_steel",
        name="hinge_pin_head",
    )

    moving_tab = model.part("moving_tab")
    moving_tab.visual(
        mesh_from_cadquery(_moving_tab_shape(), "moving_tab_leaf"),
        material="leaf_steel",
        name="tab_leaf",
    )

    model.articulation(
        "clevis_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_clevis,
        child=moving_tab,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_clevis = object_model.get_part("fixed_clevis")
    moving_tab = object_model.get_part("moving_tab")
    hinge = object_model.get_articulation("clevis_hinge")

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

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            moving_tab,
            fixed_clevis,
            name="tab_supported_in_clevis_closed_pose",
        )
        ctx.expect_within(
            moving_tab,
            fixed_clevis,
            axes="y",
            margin=0.0008,
            name="tab_captured_between_clevis_cheeks",
        )
        ctx.expect_overlap(
            moving_tab,
            fixed_clevis,
            axes="xz",
            min_overlap=0.008,
            name="tab_shared_hinge_axis_region",
        )

    open_angle = hinge.motion_limits.upper if hinge.motion_limits and hinge.motion_limits.upper is not None else 1.2
    with ctx.pose({hinge: open_angle}):
        fixed_aabb = ctx.part_world_aabb(fixed_clevis)
        tab_aabb = ctx.part_world_aabb(moving_tab)
        opens_upward = (
            fixed_aabb is not None
            and tab_aabb is not None
            and tab_aabb[1][2] > fixed_aabb[1][2] + 0.030
        )
        ctx.check(
            "tab_opens_upward_at_positive_limit",
            opens_upward,
            details=(
                f"expected tab z-max to rise above fixed clevis by > 0.030 m at open pose; "
                f"fixed_aabb={fixed_aabb}, tab_aabb={tab_aabb}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
