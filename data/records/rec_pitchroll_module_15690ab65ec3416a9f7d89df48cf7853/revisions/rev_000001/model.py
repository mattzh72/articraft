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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_RADIUS = 0.110
BASE_THICKNESS = 0.018
COLUMN_X = 0.084
COLUMN_Y = 0.068
COLUMN_HEIGHT = 0.145
PLATFORM_X = 0.094
PLATFORM_Y = 0.074
PLATFORM_THICKNESS = 0.018
CHEEK_THICKNESS = 0.024
CHEEK_Y = 0.106
CHEEK_HEIGHT = 0.092
CHEEK_CENTER_X = 0.028

ROLL_AXIS_Z = BASE_THICKNESS + COLUMN_HEIGHT + PLATFORM_THICKNESS + (CHEEK_HEIGHT / 2.0)

OUTER_FRAME_X = 0.032
OUTER_FRAME_Y = 0.150
OUTER_FRAME_Z = 0.170
OUTER_WINDOW_Y = 0.104
OUTER_WINDOW_Z = 0.126

INNER_BODY_X = 0.028
INNER_BODY_Y = 0.048
INNER_BODY_Z = 0.080
INNER_BODY_CENTER_X = 0.0
PITCH_TRUNNION_RADIUS = 0.0135
PITCH_TRUNNION_LENGTH = OUTER_WINDOW_Y


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_pitch_roll_unit")

    model.material("powder_coat", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("satin_alloy", rgba=(0.70, 0.72, 0.75, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="powder_coat",
        name="base_flange",
    )
    pedestal.visual(
        Box((COLUMN_X, COLUMN_Y, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + (COLUMN_HEIGHT / 2.0))),
        material="powder_coat",
        name="tower_column",
    )
    pedestal.visual(
        Box((PLATFORM_X, PLATFORM_Y, PLATFORM_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + (PLATFORM_THICKNESS / 2.0))
        ),
        material="powder_coat",
        name="head_shoulder",
    )
    pedestal.visual(
        Box((CHEEK_THICKNESS, CHEEK_Y, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                -CHEEK_CENTER_X,
                0.0,
                BASE_THICKNESS + COLUMN_HEIGHT + PLATFORM_THICKNESS + (CHEEK_HEIGHT / 2.0),
            )
        ),
        material="powder_coat",
        name="left_cheek",
    )
    pedestal.visual(
        Box((CHEEK_THICKNESS, CHEEK_Y, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                CHEEK_CENTER_X,
                0.0,
                BASE_THICKNESS + COLUMN_HEIGHT + PLATFORM_THICKNESS + (CHEEK_HEIGHT / 2.0),
            )
        ),
        material="powder_coat",
        name="right_cheek",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.220, 0.120, ROLL_AXIS_Z + (CHEEK_HEIGHT / 2.0))),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, (ROLL_AXIS_Z + (CHEEK_HEIGHT / 2.0)) / 2.0)),
    )

    outer_member = model.part("outer_member")
    outer_member.visual(
        Box((OUTER_FRAME_X, (OUTER_FRAME_Y - OUTER_WINDOW_Y) / 2.0, OUTER_FRAME_Z)),
        origin=Origin(
            xyz=(0.0, ((OUTER_WINDOW_Y / 2.0) + ((OUTER_FRAME_Y - OUTER_WINDOW_Y) / 4.0)), 0.0)
        ),
        material="graphite",
        name="front_roll_side",
    )
    outer_member.visual(
        Box((OUTER_FRAME_X, (OUTER_FRAME_Y - OUTER_WINDOW_Y) / 2.0, OUTER_FRAME_Z)),
        origin=Origin(
            xyz=(0.0, -((OUTER_WINDOW_Y / 2.0) + ((OUTER_FRAME_Y - OUTER_WINDOW_Y) / 4.0)), 0.0)
        ),
        material="graphite",
        name="rear_roll_side",
    )
    outer_member.visual(
        Box((OUTER_FRAME_X, OUTER_WINDOW_Y + 0.004, (OUTER_FRAME_Z - OUTER_WINDOW_Z) / 2.0)),
        origin=Origin(
            xyz=(0.0, 0.0, (OUTER_WINDOW_Z / 2.0) + ((OUTER_FRAME_Z - OUTER_WINDOW_Z) / 4.0))
        ),
        material="graphite",
        name="top_roll_bridge",
    )
    outer_member.visual(
        Box((0.012, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, (OUTER_FRAME_Z / 2.0) + 0.005)),
        material="satin_alloy",
        name="roll_indicator",
    )
    outer_member.inertial = Inertial.from_geometry(
        Box((OUTER_FRAME_X, OUTER_FRAME_Y, OUTER_FRAME_Z)),
        mass=0.62,
        origin=Origin(),
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Box((INNER_BODY_X, INNER_BODY_Y, INNER_BODY_Z)),
        origin=Origin(xyz=(INNER_BODY_CENTER_X, 0.0, 0.0)),
        material="satin_alloy",
        name="cradle_body",
    )
    inner_cradle.visual(
        Cylinder(radius=PITCH_TRUNNION_RADIUS, length=PITCH_TRUNNION_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="satin_alloy",
        name="pitch_trunnion",
    )
    inner_cradle.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material="satin_alloy",
        name="output_pad",
    )
    inner_cradle.inertial = Inertial.from_geometry(
        Box((0.034, PITCH_TRUNNION_LENGTH, 0.096)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "pedestal_to_outer_roll",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=outer_member,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.70,
            upper=0.70,
        ),
    )
    model.articulation(
        "outer_roll_to_inner_pitch",
        ArticulationType.REVOLUTE,
        parent=outer_member,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.85,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    outer_member = object_model.get_part("outer_member")
    inner_cradle = object_model.get_part("inner_cradle")
    roll_joint = object_model.get_articulation("pedestal_to_outer_roll")
    pitch_joint = object_model.get_articulation("outer_roll_to_inner_pitch")

    def elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

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

    ctx.expect_contact(
        pedestal,
        outer_member,
        name="outer roll frame is seated in the pedestal clevis",
    )
    ctx.expect_contact(
        outer_member,
        inner_cradle,
        name="inner cradle is carried by the outer roll frame",
    )

    pedestal_box = ctx.part_world_aabb(pedestal)
    outer_box = ctx.part_world_aabb(outer_member)
    inner_box = ctx.part_world_aabb(inner_cradle)
    outer_origin = ctx.part_world_position(outer_member)
    outer_y = None if outer_box is None else outer_box[1][1] - outer_box[0][1]
    outer_z = None if outer_box is None else outer_box[1][2] - outer_box[0][2]
    inner_y = None if inner_box is None else inner_box[1][1] - inner_box[0][1]
    inner_z = None if inner_box is None else inner_box[1][2] - inner_box[0][2]

    ctx.check(
        "outer member is visibly larger than inner cradle",
        outer_y is not None
        and outer_z is not None
        and inner_y is not None
        and inner_z is not None
        and outer_y > inner_y + 0.030
        and outer_z > inner_z + 0.050,
        details=f"outer_y={outer_y}, inner_y={inner_y}, outer_z={outer_z}, inner_z={inner_z}",
    )
    ctx.check(
        "compact head is mounted above the pedestal shoulder",
        outer_origin is not None and outer_origin[2] > BASE_THICKNESS + COLUMN_HEIGHT + 0.030,
        details=f"outer_origin={outer_origin}",
    )
    ctx.check(
        "roll and pitch axes are perpendicular",
        abs(
            roll_joint.axis[0] * pitch_joint.axis[0]
            + roll_joint.axis[1] * pitch_joint.axis[1]
            + roll_joint.axis[2] * pitch_joint.axis[2]
        )
        < 1e-9,
        details=f"roll_axis={roll_joint.axis}, pitch_axis={pitch_joint.axis}",
    )

    roll_indicator_rest = elem_center(outer_member, "roll_indicator")
    with ctx.pose({roll_joint: 0.45}):
        ctx.expect_contact(
            pedestal,
            outer_member,
            name="roll trunnions stay seated when the outer member rolls",
        )
        roll_indicator_rolled = elem_center(outer_member, "roll_indicator")
    ctx.check(
        "outer member rolls around the pedestal axis",
        roll_indicator_rest is not None
        and roll_indicator_rolled is not None
        and abs(roll_indicator_rolled[1] - roll_indicator_rest[1]) > 0.020,
        details=f"rest={roll_indicator_rest}, rolled={roll_indicator_rolled}",
    )

    output_pad_rest = elem_center(inner_cradle, "output_pad")
    with ctx.pose({pitch_joint: 0.55}):
        ctx.expect_contact(
            outer_member,
            inner_cradle,
            name="pitch trunnions stay seated when the cradle pitches",
        )
        output_pad_pitched = elem_center(inner_cradle, "output_pad")
    ctx.check(
        "pitch articulation moves the output pad off the neutral axis",
        output_pad_rest is not None
        and output_pad_pitched is not None
        and abs(output_pad_pitched[0] - output_pad_rest[0]) > 0.020,
        details=f"rest={output_pad_rest}, pitched={output_pad_pitched}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
