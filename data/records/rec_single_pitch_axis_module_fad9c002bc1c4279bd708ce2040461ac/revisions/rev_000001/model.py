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


BRACKET_WIDTH = 0.280
BRACKET_DEPTH = 0.105
TOP_THICK = 0.014
CHEEK_THICK = 0.016
CHEEK_DEPTH = 0.082
CHEEK_DROP = 0.118
RIB_DEPTH = 0.018
RIB_HEIGHT = 0.018
RIB_Y = -0.024
RIB_Z = -0.030

TRUNNION_RADIUS = 0.014
AXIS_Z = -0.040
INNER_CHEEK_SPAN = BRACKET_WIDTH - 2.0 * CHEEK_THICK

FACE_WIDTH = 0.208
FACE_HEIGHT = 0.148
FACE_THICK = 0.016
FACE_CENTER_Y = 0.024
FACE_CENTER_Z = -0.111

SPINE_WIDTH = 0.060
SPINE_DEPTH = 0.030
SPINE_HEIGHT = 0.100
SPINE_CENTER_Y = 0.010
SPINE_CENTER_Z = -0.072

HUB_WIDTH = 0.072
HUB_DEPTH = 0.030
HUB_HEIGHT = 0.026
HUB_CENTER_Z = -0.010

FACE_MOUNT_WIDTH = 0.082
FACE_MOUNT_DEPTH = 0.016
FACE_MOUNT_HEIGHT = 0.070
FACE_MOUNT_CENTER_Y = 0.014
FACE_MOUNT_CENTER_Z = -0.102


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_trunnion_module")

    bracket_color = model.material("bracket_powder_coat", rgba=(0.18, 0.20, 0.23, 1.0))
    carrier_color = model.material("carrier_painted_metal", rgba=(0.73, 0.75, 0.78, 1.0))

    bracket = model.part("top_bracket")
    bracket.visual(
        Box((BRACKET_WIDTH, BRACKET_DEPTH, TOP_THICK)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bracket_color,
        name="top_plate",
    )
    bracket.visual(
        Box((CHEEK_THICK, CHEEK_DEPTH, CHEEK_DROP)),
        origin=Origin(
            xyz=(-BRACKET_WIDTH / 2.0 + CHEEK_THICK / 2.0, 0.0, -TOP_THICK / 2.0 - CHEEK_DROP / 2.0)
        ),
        material=bracket_color,
        name="left_cheek",
    )
    bracket.visual(
        Box((CHEEK_THICK, CHEEK_DEPTH, CHEEK_DROP)),
        origin=Origin(
            xyz=(BRACKET_WIDTH / 2.0 - CHEEK_THICK / 2.0, 0.0, -TOP_THICK / 2.0 - CHEEK_DROP / 2.0)
        ),
        material=bracket_color,
        name="right_cheek",
    )
    bracket.visual(
        Box((INNER_CHEEK_SPAN, RIB_DEPTH, RIB_HEIGHT)),
        origin=Origin(xyz=(0.0, RIB_Y, RIB_Z)),
        material=bracket_color,
        name="lower_tie",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_WIDTH, BRACKET_DEPTH, TOP_THICK + CHEEK_DROP)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -CHEEK_DROP / 2.0)),
    )

    carrier = model.part("carried_face")
    carrier.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=INNER_CHEEK_SPAN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=carrier_color,
        name="trunnion_shaft",
    )
    carrier.visual(
        Box((HUB_WIDTH, HUB_DEPTH, HUB_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, HUB_CENTER_Z)),
        material=carrier_color,
        name="hub_block",
    )
    carrier.visual(
        Box((SPINE_WIDTH, SPINE_DEPTH, SPINE_HEIGHT)),
        origin=Origin(xyz=(0.0, SPINE_CENTER_Y, SPINE_CENTER_Z)),
        material=carrier_color,
        name="spine_block",
    )
    carrier.visual(
        Box((FACE_MOUNT_WIDTH, FACE_MOUNT_DEPTH, FACE_MOUNT_HEIGHT)),
        origin=Origin(xyz=(0.0, FACE_MOUNT_CENTER_Y, FACE_MOUNT_CENTER_Z)),
        material=carrier_color,
        name="face_mount",
    )
    carrier.visual(
        Box((FACE_WIDTH, FACE_THICK, FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, FACE_CENTER_Y, FACE_CENTER_Z)),
        material=carrier_color,
        name="face_plate",
    )
    carrier.inertial = Inertial.from_geometry(
        Box((INNER_CHEEK_SPAN, 0.050, 0.210)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.010, -0.105)),
    )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.80,
            upper=0.90,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("top_bracket")
    carrier = object_model.get_part("carried_face")
    trunnion = object_model.get_articulation("pitch_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = trunnion.motion_limits
    ctx.check(
        "parts_and_joint_present",
        bracket is not None and carrier is not None and trunnion is not None,
        "Expected top bracket, carried face, and one revolute trunnion joint.",
    )
    ctx.check(
        "horizontal_pitch_axis",
        tuple(round(v, 6) for v in trunnion.axis) == (1.0, 0.0, 0.0),
        f"Expected horizontal X-axis trunnion, got {trunnion.axis}.",
    )
    ctx.check(
        "pitch_limits_span_neutral",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        f"Unexpected limits: {limits}.",
    )

    with ctx.pose({trunnion: 0.0}):
        ctx.expect_contact(
            carrier,
            bracket,
            elem_a="trunnion_shaft",
            elem_b="left_cheek",
            name="left_cheek_supports_trunnion",
        )
        ctx.expect_contact(
            carrier,
            bracket,
            elem_a="trunnion_shaft",
            elem_b="right_cheek",
            name="right_cheek_supports_trunnion",
        )
        ctx.expect_gap(
            bracket,
            carrier,
            axis="z",
            positive_elem="top_plate",
            negative_elem="face_plate",
            min_gap=0.040,
            max_gap=0.070,
            name="carried_face_hangs_below_top_support",
        )
        ctx.expect_overlap(
            carrier,
            bracket,
            axes="x",
            elem_a="face_plate",
            elem_b="top_plate",
            min_overlap=0.18,
            name="underslung_face_stays_between_cheeks",
        )

    neutral_aabb = None
    tilted_aabb = None
    with ctx.pose({trunnion: 0.0}):
        neutral_aabb = ctx.part_element_world_aabb(carrier, elem="face_plate")
    with ctx.pose({trunnion: limits.upper if limits and limits.upper is not None else 0.8}):
        ctx.expect_contact(
            carrier,
            bracket,
            elem_a="trunnion_shaft",
            elem_b="left_cheek",
            name="left_cheek_still_supports_at_upper_pitch",
        )
        ctx.expect_contact(
            carrier,
            bracket,
            elem_a="trunnion_shaft",
            elem_b="right_cheek",
            name="right_cheek_still_supports_at_upper_pitch",
        )
        tilted_aabb = ctx.part_element_world_aabb(carrier, elem="face_plate")

    if neutral_aabb is not None and tilted_aabb is not None:
        neutral_center_y = 0.5 * (neutral_aabb[0][1] + neutral_aabb[1][1])
        tilted_center_y = 0.5 * (tilted_aabb[0][1] + tilted_aabb[1][1])
        ctx.check(
            "pitched_face_moves_fore_aft",
            abs(tilted_center_y - neutral_center_y) > 0.015,
            (
                "Face plate did not move enough in Y at upper pitch: "
                f"neutral_y={neutral_center_y:.4f}, tilted_y={tilted_center_y:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
