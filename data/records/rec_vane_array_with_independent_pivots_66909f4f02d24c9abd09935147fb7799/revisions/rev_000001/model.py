from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 1.00
FRAME_HEIGHT = 1.20
FRAME_DEPTH = 0.085
JAMB_WIDTH = 0.075
RAIL_HEIGHT = 0.080
OPENING_WIDTH = FRAME_WIDTH - 2.0 * JAMB_WIDTH
OPENING_HEIGHT = FRAME_HEIGHT - 2.0 * RAIL_HEIGHT

SLAT_COUNT = 6
SLAT_CHORD = 0.115
SLAT_THICKNESS = 0.028
SLAT_PIVOT_RADIUS = 0.0085
SLAT_END_BOSS_RADIUS = 0.012
SLAT_END_BOSS_LENGTH = 0.022
SOCKET_RADIUS = 0.0105
SOCKET_DEPTH = 0.028
END_CLEARANCE = 0.006

REST_PITCH_DEG = -28.0
SLAT_LOWER = -0.45
SLAT_UPPER = 0.75

AXIS_MARGIN_Z = 0.130
AXIS_PITCH = (OPENING_HEIGHT - 2.0 * AXIS_MARGIN_Z) / (SLAT_COUNT - 1)
AXIS_ZS = [
    -OPENING_HEIGHT / 2.0 + AXIS_MARGIN_Z + index * AXIS_PITCH
    for index in range(SLAT_COUNT)
]

LEFT_INNER_FACE_X = -OPENING_WIDTH / 2.0
RIGHT_INNER_FACE_X = OPENING_WIDTH / 2.0
LEFT_PIVOT_X = LEFT_INNER_FACE_X
RIGHT_PIVOT_X = RIGHT_INNER_FACE_X
PIVOT_SPAN = RIGHT_PIVOT_X - LEFT_PIVOT_X

BLADE_START_X = END_CLEARANCE
BLADE_END_X = PIVOT_SPAN - END_CLEARANCE
BLADE_SPAN = BLADE_END_X - BLADE_START_X
BLADE_AXIS_DROP = 0.018


def _make_frame_shape(axis_zs: list[float]) -> cq.Workplane:
    left_post = cq.Workplane("XY").box(JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT).translate(
        (-FRAME_WIDTH / 2.0 + JAMB_WIDTH / 2.0, 0.0, 0.0)
    )
    right_post = cq.Workplane("XY").box(JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT).translate(
        (FRAME_WIDTH / 2.0 - JAMB_WIDTH / 2.0, 0.0, 0.0)
    )
    top_rail = cq.Workplane("XY").box(OPENING_WIDTH, FRAME_DEPTH, RAIL_HEIGHT).translate(
        (0.0, 0.0, FRAME_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0)
    )
    bottom_rail = cq.Workplane("XY").box(
        OPENING_WIDTH, FRAME_DEPTH, RAIL_HEIGHT
    ).translate((0.0, 0.0, -FRAME_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0))

    left_sockets = (
        cq.Workplane("YZ", origin=(LEFT_INNER_FACE_X - SOCKET_DEPTH, 0.0, 0.0))
        .pushPoints([(0.0, z) for z in axis_zs])
        .circle(SOCKET_RADIUS)
        .extrude(SOCKET_DEPTH)
    )
    right_sockets = (
        cq.Workplane("YZ", origin=(RIGHT_INNER_FACE_X, 0.0, 0.0))
        .pushPoints([(0.0, z) for z in axis_zs])
        .circle(SOCKET_RADIUS)
        .extrude(SOCKET_DEPTH)
    )

    left_post = left_post.cut(left_sockets)
    right_post = right_post.cut(right_sockets)

    frame = left_post.union(right_post).union(top_rail).union(bottom_rail)
    frame = frame.edges("|Y").fillet(0.007)
    return frame


def _make_slat_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("YZ")
        .slot2D(SLAT_CHORD, SLAT_THICKNESS)
        .extrude(BLADE_SPAN)
        .translate((BLADE_START_X, 0.0, -BLADE_AXIS_DROP))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), REST_PITCH_DEG)
    )

    left_boss = cq.Workplane("YZ").circle(SLAT_END_BOSS_RADIUS).extrude(
        SLAT_END_BOSS_LENGTH
    )
    right_boss = (
        cq.Workplane("YZ")
        .circle(SLAT_END_BOSS_RADIUS)
        .extrude(SLAT_END_BOSS_LENGTH)
        .translate((PIVOT_SPAN - SLAT_END_BOSS_LENGTH, 0.0, 0.0))
    )

    slat = blade.union(left_boss).union(right_boss)
    slat = slat.edges("|X").fillet(0.003)
    return slat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louver_pivot_bank")

    model.material("frame_finish", rgba=(0.22, 0.23, 0.24, 1.0))
    model.material("slat_finish", rgba=(0.78, 0.80, 0.82, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-FRAME_WIDTH / 2.0 + JAMB_WIDTH / 2.0, 0.0, 0.0)),
        material="frame_finish",
        name="left_post",
    )
    frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(FRAME_WIDTH / 2.0 - JAMB_WIDTH / 2.0, 0.0, 0.0)),
        material="frame_finish",
        name="right_post",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0 - RAIL_HEIGHT / 2.0)),
        material="frame_finish",
        name="top_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_HEIGHT / 2.0 + RAIL_HEIGHT / 2.0)),
        material="frame_finish",
        name="bottom_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=7.5,
    )

    slat_mesh = mesh_from_cadquery(_make_slat_shape(), "louver_slat")

    for index, z_axis in enumerate(AXIS_ZS, start=1):
        slat = model.part(f"slat_{index}")
        slat.visual(
            slat_mesh,
            material="slat_finish",
            name="slat_shell",
        )
        slat.inertial = Inertial.from_geometry(
            Box((BLADE_SPAN, SLAT_CHORD, 0.080)),
            mass=0.45,
            origin=Origin(
                xyz=((BLADE_START_X + BLADE_END_X) / 2.0, 0.0, -BLADE_AXIS_DROP)
            ),
        )

        model.articulation(
            f"frame_to_slat_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(LEFT_PIVOT_X, 0.0, z_axis)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.5,
                lower=SLAT_LOWER,
                upper=SLAT_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{index}") for index in range(1, SLAT_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"frame_to_slat_{index}")
        for index in range(1, SLAT_COUNT + 1)
    ]

    ctx.check("frame part exists", frame is not None)

    for index, (slat, joint) in enumerate(zip(slats, joints), start=1):
        ctx.check(f"slat {index} exists", slat is not None)
        limits = joint.motion_limits
        ctx.check(
            f"slat {index} hinge axis is longitudinal",
            joint.axis == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"slat {index} hinge limits are authored",
            limits is not None
            and limits.lower == SLAT_LOWER
            and limits.upper == SLAT_UPPER,
            details=f"limits={limits}",
        )

    for lower, upper, pair_index in zip(slats[:-1], slats[1:], range(1, SLAT_COUNT)):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.030,
            name=f"rest gap between slat {pair_index} and slat {pair_index + 1}",
        )

    center_index = SLAT_COUNT // 2
    center_slat = slats[center_index]
    center_joint = joints[center_index]

    rest_aabb = ctx.part_world_aabb(center_slat)
    rest_center_y = None
    if rest_aabb is not None:
        rest_center_y = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0

    with ctx.pose({center_joint: 0.55}):
        posed_aabb = ctx.part_world_aabb(center_slat)
        posed_center_y = None
        if posed_aabb is not None:
            posed_center_y = (posed_aabb[0][1] + posed_aabb[1][1]) / 2.0

        ctx.check(
            "center slat swings forward when opened",
            rest_center_y is not None
            and posed_center_y is not None
            and posed_center_y > rest_center_y + 0.008,
            details=f"rest_center_y={rest_center_y}, posed_center_y={posed_center_y}",
        )

        ctx.expect_gap(
            slats[center_index + 1],
            center_slat,
            axis="z",
            min_gap=0.010,
            name="opened center slat clears the slat above",
        )
        ctx.expect_gap(
            center_slat,
            slats[center_index - 1],
            axis="z",
            min_gap=0.010,
            name="opened center slat clears the slat below",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
