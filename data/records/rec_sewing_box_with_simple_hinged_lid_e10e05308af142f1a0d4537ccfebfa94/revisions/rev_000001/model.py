from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_DEPTH = 0.34
BODY_WIDTH = 0.24
BODY_HEIGHT = 0.17
BODY_WALL = 0.008

LID_TOP_THICKNESS = 0.006
LID_SKIRT_THICKNESS = 0.006
LID_OUTER_DEPTH = 0.33
LID_OUTER_WIDTH = 0.256
LID_SKIRT_DROP = 0.034
LID_SEATING_GAP = 0.0015

HINGE_RADIUS = 0.008
HINGE_AXIS_X = -BODY_DEPTH / 2.0 - 0.004
HINGE_AXIS_Z = BODY_HEIGHT + 0.009
BODY_HINGE_LENGTH = 0.05
LID_HINGE_LENGTH = 0.04
HINGE_Y_CENTERS_BODY = (-0.09, 0.0, 0.09)
HINGE_Y_CENTERS_LID = (-0.045, 0.045)

SKID_LENGTH = 0.24
SKID_WIDTH = 0.018
SKID_HEIGHT = 0.012
SKID_Y_OFFSET = 0.075


def _y_axis_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_sewing_box")

    body_color = model.material("body_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    lid_color = model.material("lid_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    wear_color = model.material("wear_black", rgba=(0.12, 0.12, 0.12, 1.0))
    handle_color = model.material("handle_black", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_WALL / 2.0)),
        material=body_color,
        name="bottom_plate",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_WIDTH / 2.0 - BODY_WALL / 2.0, BODY_HEIGHT / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -BODY_WIDTH / 2.0 + BODY_WALL / 2.0, BODY_HEIGHT / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 - BODY_WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 + BODY_WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=body_color,
        name="back_wall",
    )

    rail_height = 0.118
    rail_size = (BODY_DEPTH - 0.08, 0.012, 0.012)
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.0, -BODY_WIDTH / 2.0 + BODY_WALL + rail_size[1] / 2.0, rail_height)),
        material=body_color,
        name="left_service_rail",
    )
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(0.0, BODY_WIDTH / 2.0 - BODY_WALL - rail_size[1] / 2.0, rail_height)),
        material=body_color,
        name="right_service_rail",
    )

    rear_doubler_size = (0.022, BODY_WIDTH - 0.036, 0.028)
    body.visual(
        Box(rear_doubler_size),
        origin=Origin(
            xyz=(
                -BODY_DEPTH / 2.0 + rear_doubler_size[0] / 2.0 + 0.002,
                0.0,
                BODY_HEIGHT - rear_doubler_size[2] / 2.0 - 0.014,
            )
        ),
        material=body_color,
        name="rear_doubler",
    )

    body_leaf_size = (0.016, BODY_HINGE_LENGTH, 0.020)
    for idx, y_center in enumerate(HINGE_Y_CENTERS_BODY, start=1):
        body.visual(
            Box(body_leaf_size),
            origin=Origin(
                xyz=(
                    HINGE_AXIS_X + body_leaf_size[0] / 2.0,
                    y_center,
                    HINGE_AXIS_Z - 0.008,
                )
            ),
            material=body_color,
            name=f"body_hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=BODY_HINGE_LENGTH),
            origin=_y_axis_cylinder_origin(HINGE_AXIS_X, y_center, HINGE_AXIS_Z),
            material=body_color,
            name=f"body_knuckle_{idx}",
        )

    lid = model.part("lid")
    lid_top_center_x = 0.187
    lid_top_center_z = BODY_HEIGHT + LID_SEATING_GAP + LID_TOP_THICKNESS / 2.0 - HINGE_AXIS_Z
    lid.visual(
        Box((LID_OUTER_DEPTH, LID_OUTER_WIDTH, LID_TOP_THICKNESS)),
        origin=Origin(xyz=(lid_top_center_x, 0.0, lid_top_center_z)),
        material=lid_color,
        name="top_panel",
    )

    skirt_center_z = lid_top_center_z - LID_TOP_THICKNESS / 2.0 - LID_SKIRT_DROP / 2.0
    lid.visual(
        Box((LID_OUTER_DEPTH - 0.010, LID_SKIRT_THICKNESS, LID_SKIRT_DROP)),
        origin=Origin(
            xyz=(
                lid_top_center_x + 0.003,
                -LID_OUTER_WIDTH / 2.0 + LID_SKIRT_THICKNESS / 2.0,
                skirt_center_z,
            )
        ),
        material=lid_color,
        name="left_skirt",
    )
    lid.visual(
        Box((LID_OUTER_DEPTH - 0.010, LID_SKIRT_THICKNESS, LID_SKIRT_DROP)),
        origin=Origin(
            xyz=(
                lid_top_center_x + 0.003,
                LID_OUTER_WIDTH / 2.0 - LID_SKIRT_THICKNESS / 2.0,
                skirt_center_z,
            )
        ),
        material=lid_color,
        name="right_skirt",
    )
    lid.visual(
        Box((LID_SKIRT_THICKNESS, LID_OUTER_WIDTH - 2.0 * LID_SKIRT_THICKNESS, LID_SKIRT_DROP)),
        origin=Origin(
            xyz=(
                lid_top_center_x + LID_OUTER_DEPTH / 2.0 - LID_SKIRT_THICKNESS / 2.0,
                0.0,
                skirt_center_z,
            )
        ),
        material=lid_color,
        name="front_skirt",
    )

    pull_size = (0.026, 0.105, 0.014)
    lid.visual(
        Box(pull_size),
        origin=Origin(
            xyz=(
                lid_top_center_x + 0.105,
                0.0,
                lid_top_center_z + LID_TOP_THICKNESS / 2.0 + pull_size[2] / 2.0,
            )
        ),
        material=handle_color,
        name="pull_handle",
    )

    hinge_strap_size = (0.034, 0.026, 0.018)
    lid_leaf_size = (0.020, LID_HINGE_LENGTH, 0.016)
    for idx, y_center in enumerate(HINGE_Y_CENTERS_LID, start=1):
        lid.visual(
            Box(hinge_strap_size),
            origin=Origin(xyz=(0.017, y_center, 0.003)),
            material=lid_color,
            name=f"lid_hinge_strap_{idx}",
        )
        lid.visual(
            Box(lid_leaf_size),
            origin=Origin(xyz=(0.010, y_center, 0.002)),
            material=lid_color,
            name=f"lid_hinge_leaf_{idx}",
        )
        lid.visual(
            Cylinder(radius=HINGE_RADIUS, length=LID_HINGE_LENGTH),
            origin=_y_axis_cylinder_origin(0.0, y_center, 0.0),
            material=lid_color,
            name=f"lid_knuckle_{idx}",
        )

    left_skid = model.part("left_skid")
    left_skid.visual(
        Box((SKID_LENGTH, SKID_WIDTH, SKID_HEIGHT)),
        material=wear_color,
        name="skid_bar",
    )

    right_skid = model.part("right_skid")
    right_skid.visual(
        Box((SKID_LENGTH, SKID_WIDTH, SKID_HEIGHT)),
        material=wear_color,
        name="skid_bar",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    model.articulation(
        "body_to_left_skid",
        ArticulationType.FIXED,
        parent=body,
        child=left_skid,
        origin=Origin(xyz=(0.0, -SKID_Y_OFFSET, -SKID_HEIGHT / 2.0)),
    )
    model.articulation(
        "body_to_right_skid",
        ArticulationType.FIXED,
        parent=body,
        child=right_skid,
        origin=Origin(xyz=(0.0, SKID_Y_OFFSET, -SKID_HEIGHT / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_skid = object_model.get_part("left_skid")
    right_skid = object_model.get_part("right_skid")
    hinge = object_model.get_articulation("body_to_lid")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="hinge_motion_clearance")

    ctx.check(
        "lid_hinge_axis_orientation",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"expected hinge axis (0,-1,0), got {hinge.axis}",
    )
    ctx.expect_contact(left_skid, body, name="left_skid_attached")
    ctx.expect_contact(right_skid, body, name="right_skid_attached")
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.22, name="lid_covers_body_footprint")
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        positive_elem="front_skirt",
        negative_elem="front_wall",
        min_gap=0.001,
        max_gap=0.0045,
        name="front_lid_service_clearance",
    )

    with ctx.pose({hinge: 1.85}):
        body_aabb = ctx.part_world_aabb(body)
        lid_aabb = ctx.part_world_aabb(lid)
        opened_clear = (
            body_aabb is not None
            and lid_aabb is not None
            and lid_aabb[1][2] > body_aabb[1][2] + 0.11
            and lid_aabb[0][0] < body_aabb[0][0] - 0.025
        )
        ctx.check(
            "lid_opens_for_maintenance_access",
            opened_clear,
            details=f"open-pose body_aabb={body_aabb}, lid_aabb={lid_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
