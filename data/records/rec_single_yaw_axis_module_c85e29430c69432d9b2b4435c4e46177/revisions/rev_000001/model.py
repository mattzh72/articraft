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


WALL_THICKNESS = 0.012
WALL_WIDTH = 0.130
WALL_HEIGHT = 0.190

AXIS_HEIGHT = 0.105
JOINT_X = 0.079

ARM_LENGTH = 0.036
ARM_WIDTH = 0.042
ARM_HEIGHT = 0.054
ARM_CENTER_X = WALL_THICKNESS / 2.0 + ARM_LENGTH / 2.0

BLOCK_LENGTH = 0.028
BLOCK_WIDTH = 0.068
BLOCK_HEIGHT = 0.086
BLOCK_REAR_X = WALL_THICKNESS / 2.0 + ARM_LENGTH
BLOCK_CENTER_X = BLOCK_REAR_X + BLOCK_LENGTH / 2.0
BLOCK_FRONT_X = BLOCK_REAR_X + BLOCK_LENGTH

NOSE_LENGTH = 0.018
NOSE_RADIUS = 0.030
NOSE_FRONT_LOCAL_X = BLOCK_FRONT_X + NOSE_LENGTH - JOINT_X

BORE_RADIUS = 0.0135
SHAFT_RADIUS = 0.0120
SHAFT_HEIGHT = 0.094

FLANGE_RADIUS = 0.026
FLANGE_THICKNESS = 0.006
FLANGE_REAR_LOCAL_X = NOSE_FRONT_LOCAL_X
FLANGE_CENTER_LOCAL_X = FLANGE_REAR_LOCAL_X + FLANGE_THICKNESS / 2.0

PILOT_RADIUS = 0.014
PILOT_LENGTH = 0.018
PILOT_START_LOCAL_X = FLANGE_REAR_LOCAL_X + FLANGE_THICKNESS
PILOT_CENTER_LOCAL_X = PILOT_START_LOCAL_X + PILOT_LENGTH / 2.0

FACEPLATE_THICKNESS = 0.010
FACEPLATE_WIDTH = 0.072
FACEPLATE_HEIGHT = 0.082
FACEPLATE_REAR_LOCAL_X = PILOT_START_LOCAL_X + PILOT_LENGTH
FACEPLATE_CENTER_LOCAL_X = FACEPLATE_REAR_LOCAL_X + FACEPLATE_THICKNESS / 2.0


def _make_support_shape() -> cq.Workplane:
    wall = cq.Workplane("XY").box(
        WALL_THICKNESS,
        WALL_WIDTH,
        WALL_HEIGHT,
        centered=(True, True, False),
    )

    arm = cq.Workplane("XY").box(ARM_LENGTH, ARM_WIDTH, ARM_HEIGHT).translate(
        (ARM_CENTER_X, 0.0, AXIS_HEIGHT)
    )

    cartridge_block = cq.Workplane("XY").box(
        BLOCK_LENGTH,
        BLOCK_WIDTH,
        BLOCK_HEIGHT,
    ).translate((BLOCK_CENTER_X, 0.0, AXIS_HEIGHT))

    cartridge_nose = (
        cq.Workplane("YZ")
        .circle(NOSE_RADIUS)
        .extrude(NOSE_LENGTH)
        .translate((BLOCK_FRONT_X, 0.0, AXIS_HEIGHT))
    )

    support = wall.union(arm).union(cartridge_block).union(cartridge_nose)

    vertical_bore = (
        cq.Workplane("XY")
        .center(JOINT_X, 0.0)
        .circle(BORE_RADIUS)
        .extrude(BLOCK_HEIGHT + 0.012)
        .translate((0.0, 0.0, AXIS_HEIGHT - BLOCK_HEIGHT / 2.0 - 0.006))
    )

    mount_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.043, 0.040),
                (0.043, 0.040),
                (-0.043, 0.148),
                (0.043, 0.148),
            ]
        )
        .circle(0.006)
        .extrude(WALL_THICKNESS * 3.0, both=True)
    )

    return support.cut(vertical_bore).cut(mount_holes)


def _make_output_spindle() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_HEIGHT)
        .translate((0.0, 0.0, -SHAFT_HEIGHT / 2.0))
    )


def _make_output_flange() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(FLANGE_RADIUS)
        .extrude(FLANGE_THICKNESS)
        .translate((FLANGE_REAR_LOCAL_X, 0.0, 0.0))
    )


def _make_output_pilot() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(PILOT_RADIUS)
        .extrude(PILOT_LENGTH)
        .translate((PILOT_START_LOCAL_X, 0.0, 0.0))
    )


def _make_output_faceplate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            FACEPLATE_THICKNESS,
            FACEPLATE_WIDTH,
            FACEPLATE_HEIGHT,
        )
        .translate((FACEPLATE_CENTER_LOCAL_X, 0.0, 0.0))
        .edges("|X")
        .fillet(0.005)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_yaw_rotary_module")

    dark_steel = model.material("dark_steel", color=(0.28, 0.30, 0.33))
    aluminum = model.material("aluminum", color=(0.78, 0.80, 0.83))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_make_support_shape(), "support_body"),
        material=dark_steel,
        name="support_body",
    )

    output = model.part("output")
    output.visual(
        mesh_from_cadquery(_make_output_spindle(), "output_spindle"),
        material=aluminum,
        name="output_spindle",
    )
    output.visual(
        mesh_from_cadquery(_make_output_flange(), "output_flange"),
        material=aluminum,
        name="output_flange",
    )
    output.visual(
        mesh_from_cadquery(_make_output_pilot(), "output_pilot"),
        material=aluminum,
        name="output_pilot",
    )
    output.visual(
        mesh_from_cadquery(_make_output_faceplate(), "output_faceplate"),
        material=aluminum,
        name="output_faceplate",
    )

    model.articulation(
        "support_to_output",
        ArticulationType.REVOLUTE,
        parent=support,
        child=output,
        origin=Origin(xyz=(JOINT_X, 0.0, AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-2.2,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    output = object_model.get_part("output")
    yaw = object_model.get_articulation("support_to_output")

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

    limits = yaw.motion_limits
    ctx.check(
        "yaw_joint_is_vertical_revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE and yaw.axis == (0.0, 0.0, 1.0),
        f"expected a vertical revolute axis, got type={yaw.articulation_type} axis={yaw.axis}",
    )
    ctx.check(
        "yaw_limits_allow_bidirectional_rotation",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        f"expected signed revolute limits around zero, got {limits}",
    )

    with ctx.pose({yaw: 0.0}):
        ctx.expect_contact(
            output,
            support,
            contact_tol=5e-4,
            name="output_flange_is_supported_by_cartridge",
        )
        closed_aabb = ctx.part_element_world_aabb(output, elem="output_faceplate")
        ctx.expect_gap(
            output,
            support,
            axis="x",
            positive_elem="output_faceplate",
            min_gap=0.015,
            name="faceplate_projects_forward_of_support",
        )

    with ctx.pose({yaw: limits.upper if limits and limits.upper is not None else 0.0}):
        ctx.expect_contact(
            output,
            support,
            contact_tol=5e-4,
            name="output_remains_supported_when_turned",
        )
        open_aabb = ctx.part_element_world_aabb(output, elem="output_faceplate")

    if closed_aabb is None or open_aabb is None:
        ctx.fail(
            "output_aabb_available",
            "could not resolve output bounds in closed and open poses",
        )
    else:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
        open_center_y = (open_aabb[0][1] + open_aabb[1][1]) / 2.0
        ctx.check(
            "positive_yaw_moves_faceplate_toward_positive_y",
            open_center_y > closed_center_y + 0.015,
            (
                "expected positive yaw to swing the carried faceplate toward +Y; "
                f"closed_center_y={closed_center_y:.4f}, open_center_y={open_center_y:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
