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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.150
BASE_WIDTH = 0.100
BASE_PLATE_THICKNESS = 0.008
UPRIGHT_X = -0.045
UPRIGHT_THICKNESS = 0.020
UPRIGHT_WIDTH = 0.060
UPRIGHT_HEIGHT = 0.072
SHOULDER_X = -0.018
SHOULDER_Z = 0.078
BASE_EAR_LENGTH = 0.034
BASE_EAR_THICKNESS = 0.010
BASE_EAR_GAP = 0.028
BASE_EAR_HEIGHT = 0.044
BASE_EAR_BOTTOM = 0.050

LOWER_LINK_LENGTH = 0.105
LOWER_BARREL_RADIUS = 0.010
LOWER_BARREL_LENGTH = BASE_EAR_GAP
LOWER_BEAM_WIDTH = 0.018
LOWER_BEAM_HEIGHT = 0.018
LOWER_FORK_GAP = 0.020
LOWER_FORK_CHEEK = 0.008

UPPER_LINK_LENGTH = 0.095
UPPER_BARREL_RADIUS = 0.009
UPPER_BARREL_LENGTH = LOWER_FORK_GAP
UPPER_BEAM_WIDTH = 0.016
UPPER_BEAM_HEIGHT = 0.016
UPPER_PAD_RADIUS = 0.018
UPPER_PAD_HEIGHT = 0.008

SWIVEL_PUCK_RADIUS = 0.016
SWIVEL_PUCK_HEIGHT = 0.008
TILT_AXIS_X = 0.030
TILT_AXIS_Z = 0.020
YOKE_GAP = 0.022
YOKE_CHEEK = 0.007

HEAD_TRUNNION_RADIUS = 0.0065
HEAD_TRUNNION_LENGTH = YOKE_GAP
HEAD_FRAME_WIDTH = 0.086
HEAD_FRAME_HEIGHT = 0.068


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="console_monitor_arm")

    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("graphite", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("dark_aluminum", rgba=(0.52, 0.55, 0.58, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_PLATE_THICKNESS)),
        material="powder_black",
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS / 2.0)),
        name="base_plate",
    )
    base_bracket.visual(
        Box((UPRIGHT_THICKNESS, UPRIGHT_WIDTH, UPRIGHT_HEIGHT)),
        material="powder_black",
        origin=Origin(
            xyz=(UPRIGHT_X, 0.0, BASE_PLATE_THICKNESS + UPRIGHT_HEIGHT / 2.0)
        ),
        name="upright",
    )
    base_bracket.visual(
        Box((0.040, BASE_EAR_GAP + 2.0 * BASE_EAR_THICKNESS, 0.020)),
        material="powder_black",
        origin=Origin(xyz=(SHOULDER_X - 0.010, 0.0, BASE_PLATE_THICKNESS + 0.028)),
        name="shoulder_block",
    )
    ear_y = BASE_EAR_GAP / 2.0 + BASE_EAR_THICKNESS / 2.0
    ear_x = SHOULDER_X - 0.006
    for side_name, sign in (("left_ear", 1.0), ("right_ear", -1.0)):
        base_bracket.visual(
            Box((BASE_EAR_LENGTH, BASE_EAR_THICKNESS, BASE_EAR_HEIGHT)),
            material="powder_black",
            origin=Origin(
                xyz=(
                    ear_x,
                    sign * ear_y,
                    BASE_EAR_BOTTOM + BASE_EAR_HEIGHT / 2.0,
                )
            ),
            name=side_name,
        )
    for side_name, sign in (("left_boss", 1.0), ("right_boss", -1.0)):
        base_bracket.visual(
            Cylinder(radius=0.007, length=0.004),
            material="powder_black",
            origin=Origin(
                xyz=(
                    SHOULDER_X,
                    sign * (BASE_EAR_GAP / 2.0 + BASE_EAR_THICKNESS),
                    SHOULDER_Z,
                ),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            name=side_name,
        )
    base_bracket.visual(
        Box((0.036, BASE_EAR_GAP + 2.0 * BASE_EAR_THICKNESS, 0.028)),
        material="powder_black",
        origin=Origin(xyz=(SHOULDER_X - 0.020, 0.0, BASE_PLATE_THICKNESS + 0.014)),
        name="rear_rib",
    )

    lower_link = model.part("lower_link")
    lower_link.visual(
        Cylinder(radius=LOWER_BARREL_RADIUS, length=LOWER_BARREL_LENGTH),
        material="dark_aluminum",
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="rear_barrel",
    )
    lower_link.visual(
        Box((0.072, LOWER_BEAM_WIDTH, LOWER_BEAM_HEIGHT)),
        material="dark_aluminum",
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        name="beam",
    )
    lower_link.visual(
        Box((0.032, 0.036, 0.018)),
        material="dark_aluminum",
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        name="crosshead",
    )
    cheek_y = LOWER_FORK_GAP / 2.0 + LOWER_FORK_CHEEK / 2.0
    for side_name, sign in (("left_cheek", 1.0), ("right_cheek", -1.0)):
        lower_link.visual(
            Box((0.020, LOWER_FORK_CHEEK, 0.024)),
            material="dark_aluminum",
            origin=Origin(xyz=(LOWER_LINK_LENGTH, sign * cheek_y, 0.0)),
            name=side_name,
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=UPPER_BARREL_RADIUS, length=UPPER_BARREL_LENGTH),
        material="aluminum",
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="rear_barrel",
    )
    upper_link.visual(
        Box((0.068, UPPER_BEAM_WIDTH, UPPER_BEAM_HEIGHT)),
        material="aluminum",
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        name="beam",
    )
    upper_link.visual(
        Box((0.018, UPPER_BEAM_WIDTH, UPPER_BEAM_HEIGHT)),
        material="aluminum",
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        name="root_bridge",
    )
    upper_link.visual(
        Box((0.028, 0.030, 0.020)),
        material="aluminum",
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
        name="nose_block",
    )
    upper_link.visual(
        Cylinder(radius=UPPER_PAD_RADIUS, length=UPPER_PAD_HEIGHT),
        material="aluminum",
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, UPPER_PAD_HEIGHT / 2.0)),
        name="swivel_pad",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Cylinder(radius=SWIVEL_PUCK_RADIUS, length=SWIVEL_PUCK_HEIGHT),
        material="graphite",
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_PUCK_HEIGHT / 2.0)),
        name="puck",
    )
    head_swivel.visual(
        Box((0.020, 0.022, 0.012)),
        material="graphite",
        origin=Origin(xyz=(0.013, 0.0, 0.014)),
        name="neck",
    )
    head_swivel.visual(
        Box((0.010, 0.032, 0.012)),
        material="graphite",
        origin=Origin(xyz=(0.017, 0.0, 0.020)),
        name="bridge",
    )
    yoke_y = YOKE_GAP / 2.0 + YOKE_CHEEK / 2.0
    for side_name, sign in (("left_yoke", 1.0), ("right_yoke", -1.0)):
        head_swivel.visual(
            Box((0.016, YOKE_CHEEK, 0.036)),
            material="graphite",
            origin=Origin(xyz=(TILT_AXIS_X, sign * yoke_y, TILT_AXIS_Z)),
            name=side_name,
        )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Cylinder(radius=HEAD_TRUNNION_RADIUS, length=HEAD_TRUNNION_LENGTH),
        material="graphite",
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="trunnion",
    )
    head_frame.visual(
        Box((0.014, 0.022, 0.018)),
        material="graphite",
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        name="neck",
    )
    head_frame.visual(
        Box((0.006, 0.010, HEAD_FRAME_HEIGHT)),
        material="graphite",
        origin=Origin(xyz=(0.019, 0.038, 0.0)),
        name="left_rail",
    )
    head_frame.visual(
        Box((0.006, 0.010, HEAD_FRAME_HEIGHT)),
        material="graphite",
        origin=Origin(xyz=(0.019, -0.038, 0.0)),
        name="right_rail",
    )
    head_frame.visual(
        Box((0.006, HEAD_FRAME_WIDTH, 0.010)),
        material="graphite",
        origin=Origin(xyz=(0.019, 0.0, 0.029)),
        name="top_rail",
    )
    head_frame.visual(
        Box((0.006, HEAD_FRAME_WIDTH, 0.010)),
        material="graphite",
        origin=Origin(xyz=(0.019, 0.0, -0.029)),
        name="bottom_rail",
    )
    head_frame.visual(
        Box((0.004, 0.024, 0.050)),
        material="graphite",
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        name="center_plate",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=lower_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.10, effort=18.0, velocity=1.5),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=upper_link,
        origin=Origin(xyz=(LOWER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.25, upper=0.35, effort=16.0, velocity=1.8),
    )
    model.articulation(
        "head_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=head_swivel,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, UPPER_PAD_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.55, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=head_frame,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, TILT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=0.55, effort=4.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    lower_link = object_model.get_part("lower_link")
    upper_link = object_model.get_part("upper_link")
    head_swivel = object_model.get_part("head_swivel")
    head_frame = object_model.get_part("head_frame")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    head_swivel_joint = object_model.get_articulation("head_swivel_joint")
    head_tilt_joint = object_model.get_articulation("head_tilt_joint")

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

    for part_name in (
        "base_bracket",
        "lower_link",
        "upper_link",
        "head_swivel",
        "head_frame",
    ):
        ctx.check(
            f"{part_name} exists",
            object_model.get_part(part_name) is not None,
            details=f"missing part {part_name}",
        )

    ctx.expect_contact(
        lower_link,
        base_bracket,
        name="lower link stays mounted in the base bracket ears",
    )
    ctx.expect_contact(
        upper_link,
        lower_link,
        name="upper link stays captured in the lower link fork",
    )
    ctx.expect_contact(
        head_swivel,
        upper_link,
        name="swivel head sits on the upper link pad",
    )
    ctx.expect_contact(
        head_frame,
        head_swivel,
        name="head frame trunnion stays supported by the tilt yoke",
    )

    ctx.check(
        "fold joints pitch in the main arm plane",
        shoulder_joint.axis == (0.0, -1.0, 0.0) and elbow_joint.axis == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder_joint.axis}, elbow={elbow_joint.axis}",
    )
    ctx.check(
        "head joints keep separate swivel and tilt axes",
        head_swivel_joint.axis == (0.0, 0.0, 1.0) and head_tilt_joint.axis == (0.0, -1.0, 0.0),
        details=f"swivel={head_swivel_joint.axis}, tilt={head_tilt_joint.axis}",
    )

    folded_pose = {
        shoulder_joint: 1.00,
        elbow_joint: -2.00,
        head_swivel_joint: 0.0,
        head_tilt_joint: 0.0,
    }
    extended_pose = {
        shoulder_joint: 0.15,
        elbow_joint: -0.30,
        head_swivel_joint: 0.0,
        head_tilt_joint: 0.0,
    }

    with ctx.pose(folded_pose):
        folded_head = ctx.part_world_position(head_frame)
        ctx.check(
            "folded head stays tucked near the base bracket",
            folded_head is not None and 0.09 <= folded_head[0] <= 0.16,
            details=f"folded_head={folded_head}",
        )
        ctx.check(
            "folded head remains above the grounded base plate",
            folded_head is not None and folded_head[2] >= 0.06,
            details=f"folded_head={folded_head}",
        )

    with ctx.pose(extended_pose):
        upper_extended = ctx.part_world_position(head_frame)

    ctx.check(
        "fold arm extends the head frame forward when opened",
        folded_head is not None
        and upper_extended is not None
        and upper_extended[0] > folded_head[0] + 0.07,
        details=f"folded={folded_head}, extended={upper_extended}",
    )

    swivel_rest = ctx.part_world_position(head_frame)
    with ctx.pose({head_swivel_joint: 0.80}):
        swivel_turned = ctx.part_world_position(head_frame)
    ctx.check(
        "swivel joint yaws the head frame sideways",
        swivel_rest is not None
        and swivel_turned is not None
        and swivel_turned[1] > swivel_rest[1] + 0.015,
        details=f"rest={swivel_rest}, turned={swivel_turned}",
    )

    tilt_rest_aabb = ctx.part_world_aabb(head_frame)
    with ctx.pose({head_tilt_joint: 0.45}):
        tilt_aabb = ctx.part_world_aabb(head_frame)
    ctx.check(
        "tilt joint changes the head frame orientation envelope",
        tilt_rest_aabb is not None
        and tilt_aabb is not None
        and abs(tilt_aabb[1][0] - tilt_rest_aabb[1][0]) > 0.008,
        details=f"rest={tilt_rest_aabb}, tilted={tilt_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
