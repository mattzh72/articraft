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


PLATE_WIDTH = 0.22
PLATE_HEIGHT = 0.34
PLATE_THICKNESS = 0.012
PLATE_Y_CENTER = -0.062
PLATE_CORNER_RADIUS = 0.018
MOUNT_HOLE_RADIUS = 0.005
MOUNT_HOLE_X = 0.078
MOUNT_HOLE_Z = 0.118

RIB_THICKNESS = 0.014
RIB_X_CENTER = 0.052
RIB_REAR_Y = -0.069
RIB_FRONT_Y = -0.016
RIB_Z_MAX = 0.136
RIB_Z_NECK = 0.064

SUPPORT_COLUMN_RADIUS = 0.045
SUPPORT_COLUMN_REAR_Y = -0.036
SUPPORT_COLUMN_DEPTH = 0.036

HEAD_HUB_OUTER_RADIUS = 0.052
HEAD_HUB_INNER_RADIUS = 0.019
HEAD_HUB_REAR_Y = 0.0
HEAD_HUB_DEPTH = 0.026
HEAD_FACE_RADIUS = 0.056
HEAD_FACE_REAR_Y = 0.012
HEAD_FACE_DEPTH = 0.012

HEAD_BLOCK_SIZE = (0.050, 0.022, 0.108)
HEAD_BLOCK_CENTER = (0.028, 0.032, 0.0)
STAGE_PAD_SIZE = (0.055, 0.014, 0.086)
STAGE_PAD_CENTER = (0.0795, 0.046, 0.0)


def _mount_hole_points() -> list[tuple[float, float]]:
    return [
        (-MOUNT_HOLE_X, -MOUNT_HOLE_Z),
        (-MOUNT_HOLE_X, MOUNT_HOLE_Z),
        (MOUNT_HOLE_X, -MOUNT_HOLE_Z),
        (MOUNT_HOLE_X, MOUNT_HOLE_Z),
    ]


def _y_cylinder(radius: float, length: float, rear_y: float, inner_radius: float | None = None):
    section = cq.Workplane("XY").circle(radius)
    if inner_radius is not None:
        section = section.circle(inner_radius)
    return (
        section.extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, rear_y, 0.0))
    )


def _support_body_shape():
    plate = (
        cq.Workplane("XZ")
        .rect(PLATE_WIDTH, PLATE_HEIGHT)
        .extrude(PLATE_THICKNESS)
        .translate((0.0, PLATE_Y_CENTER - PLATE_THICKNESS / 2.0, 0.0))
        .edges("|Y")
        .fillet(PLATE_CORNER_RADIUS)
    )
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_mount_hole_points())
        .circle(MOUNT_HOLE_RADIUS)
        .cutThruAll()
    )

    rib_profile = [
        (RIB_REAR_Y, -RIB_Z_MAX),
        (RIB_REAR_Y, RIB_Z_MAX),
        (RIB_FRONT_Y, RIB_Z_NECK),
        (RIB_FRONT_Y, -RIB_Z_NECK),
    ]
    right_rib = (
        cq.Workplane("YZ")
        .polyline(rib_profile)
        .close()
        .extrude(RIB_THICKNESS)
        .translate((RIB_X_CENTER - RIB_THICKNESS / 2.0, 0.0, 0.0))
    )
    left_rib = (
        cq.Workplane("YZ")
        .polyline(rib_profile)
        .close()
        .extrude(RIB_THICKNESS)
        .translate((-RIB_X_CENTER - RIB_THICKNESS / 2.0, 0.0, 0.0))
    )

    column = _y_cylinder(
        SUPPORT_COLUMN_RADIUS,
        SUPPORT_COLUMN_DEPTH,
        SUPPORT_COLUMN_REAR_Y,
    )

    return plate.union(left_rib).union(right_rib).union(column)


def _head_main_shape():
    hub = _y_cylinder(
        HEAD_HUB_OUTER_RADIUS,
        HEAD_HUB_DEPTH,
        HEAD_HUB_REAR_Y,
        inner_radius=HEAD_HUB_INNER_RADIUS,
    )
    face = _y_cylinder(HEAD_FACE_RADIUS, HEAD_FACE_DEPTH, HEAD_FACE_REAR_Y)
    block = cq.Workplane("XY").box(*HEAD_BLOCK_SIZE).translate(HEAD_BLOCK_CENTER)
    return hub.union(face).union(block)


def _stage_pad_shape():
    return cq.Workplane("XY").box(*STAGE_PAD_SIZE).translate(STAGE_PAD_CENTER)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_rotary_module")

    model.material("painted_steel", rgba=(0.26, 0.29, 0.32, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_anodized", rgba=(0.18, 0.20, 0.23, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_support_body_shape(), "support_body"),
        material="painted_steel",
        name="support_body",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_main_shape(), "head_main"),
        material="machined_aluminum",
        name="head_main",
    )
    head.visual(
        mesh_from_cadquery(_stage_pad_shape(), "stage_pad"),
        material="dark_anodized",
        name="stage_pad",
    )

    model.articulation(
        "support_to_head",
        ArticulationType.REVOLUTE,
        parent=support,
        child=head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    head = object_model.get_part("head")
    rotary = object_model.get_articulation("support_to_head")

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

    ctx.check(
        "rotary_axis_is_vertical",
        tuple(round(value, 6) for value in rotary.axis) == (0.0, 0.0, 1.0),
        details=f"axis={rotary.axis}",
    )
    ctx.expect_contact(
        head,
        support,
        elem_a="head_main",
        elem_b="support_body",
        contact_tol=5e-4,
        name="head_bearing_face_contacts_support",
    )

    closed_pad = ctx.part_element_world_aabb(head, elem="stage_pad")
    with ctx.pose({rotary: 0.9}):
        opened_pad = ctx.part_element_world_aabb(head, elem="stage_pad")

    if closed_pad is None or opened_pad is None:
        ctx.fail("stage_pad_pose_probe", "missing stage_pad world AABB in one or more poses")
    else:
        closed_center = tuple(
            (closed_pad[0][i] + closed_pad[1][i]) * 0.5 for i in range(3)
        )
        opened_center = tuple(
            (opened_pad[0][i] + opened_pad[1][i]) * 0.5 for i in range(3)
        )
        ctx.check(
            "stage_pad_swings_around_vertical_axis",
            opened_center[1] > closed_center[1] + 0.02
            and opened_center[0] < closed_center[0] - 0.05
            and abs(opened_center[2] - closed_center[2]) < 0.005,
            details=f"closed_center={closed_center}, opened_center={opened_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
