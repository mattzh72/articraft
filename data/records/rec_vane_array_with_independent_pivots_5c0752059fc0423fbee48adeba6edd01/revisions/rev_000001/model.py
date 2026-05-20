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


FRAME_OUTER_WIDTH = 0.490
FRAME_OUTER_HEIGHT = 0.350
FRAME_DEPTH = 0.060
FRAME_RAIL = 0.035
OPENING_WIDTH = FRAME_OUTER_WIDTH - 2.0 * FRAME_RAIL
OPENING_HEIGHT = FRAME_OUTER_HEIGHT - 2.0 * FRAME_RAIL

VANE_COUNT = 5
VANE_PITCH = OPENING_HEIGHT / VANE_COUNT
VANE_HEIGHT = 0.032
VANE_THICKNESS = 0.008

PIVOT_BOSS_RADIUS = 0.010
PIVOT_JOURNAL_RADIUS = 0.008
PIVOT_JOURNAL_LENGTH = 0.012
BLADE_LENGTH = OPENING_WIDTH - 2.0 * PIVOT_JOURNAL_LENGTH

VANE_LIMIT = 0.95
VANE_Z_OFFSETS = [
    (-OPENING_HEIGHT / 2.0 + VANE_PITCH / 2.0) + index * VANE_PITCH
    for index in range(VANE_COUNT)
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airflow_vane_set")

    model.material("frame_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("vane_finish", rgba=(0.78, 0.80, 0.82, 1.0))

    frame = model.part("frame")
    left_rail_x = -(OPENING_WIDTH / 2.0 + FRAME_RAIL / 2.0)
    right_rail_x = OPENING_WIDTH / 2.0 + FRAME_RAIL / 2.0
    top_rail_z = OPENING_HEIGHT / 2.0 + FRAME_RAIL / 2.0
    bottom_rail_z = -(OPENING_HEIGHT / 2.0 + FRAME_RAIL / 2.0)

    frame.visual(
        Box((FRAME_RAIL, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(left_rail_x, 0.0, 0.0)),
        material="frame_finish",
        name="left_rail",
    )
    frame.visual(
        Box((FRAME_RAIL, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(right_rail_x, 0.0, 0.0)),
        material="frame_finish",
        name="right_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, top_rail_z)),
        material="frame_finish",
        name="top_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, FRAME_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, bottom_rail_z)),
        material="frame_finish",
        name="bottom_rail",
    )

    for index, z_offset in enumerate(VANE_Z_OFFSETS, start=1):
        frame.visual(
            Cylinder(radius=PIVOT_BOSS_RADIUS, length=FRAME_RAIL),
            origin=Origin(
                xyz=(left_rail_x, 0.0, z_offset),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="frame_finish",
            name=f"left_pivot_boss_{index}",
        )
        frame.visual(
            Cylinder(radius=PIVOT_BOSS_RADIUS, length=FRAME_RAIL),
            origin=Origin(
                xyz=(right_rail_x, 0.0, z_offset),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="frame_finish",
            name=f"right_pivot_boss_{index}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=2.6,
    )

    for index, z_offset in enumerate(VANE_Z_OFFSETS, start=1):
        vane = model.part(f"vane_{index}")
        vane.visual(
            Box((BLADE_LENGTH, VANE_THICKNESS, VANE_HEIGHT)),
            origin=Origin(),
            material="vane_finish",
            name=f"vane_{index}_blade",
        )
        vane.visual(
            Cylinder(radius=PIVOT_JOURNAL_RADIUS, length=PIVOT_JOURNAL_LENGTH),
            origin=Origin(
                xyz=(-OPENING_WIDTH / 2.0 + PIVOT_JOURNAL_LENGTH / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="vane_finish",
            name=f"vane_{index}_left_journal",
        )
        vane.visual(
            Cylinder(radius=PIVOT_JOURNAL_RADIUS, length=PIVOT_JOURNAL_LENGTH),
            origin=Origin(
                xyz=(OPENING_WIDTH / 2.0 - PIVOT_JOURNAL_LENGTH / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="vane_finish",
            name=f"vane_{index}_right_journal",
        )
        vane.inertial = Inertial.from_geometry(
            Box((OPENING_WIDTH, 2.0 * PIVOT_JOURNAL_RADIUS, VANE_HEIGHT)),
            mass=0.14,
        )

        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    z_offset,
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-VANE_LIMIT,
                upper=VANE_LIMIT,
                effort=1.0,
                velocity=2.0,
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
    mid_vane = object_model.get_part("vane_3")
    neighbor_vane = object_model.get_part("vane_4")
    mid_joint = object_model.get_articulation("frame_to_vane_3")

    for index in range(1, VANE_COUNT + 1):
        vane = object_model.get_part(f"vane_{index}")
        joint = object_model.get_articulation(f"frame_to_vane_{index}")

        limits = joint.motion_limits
        axis_ok = joint.axis == (1.0, 0.0, 0.0)
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        )
        ctx.check(
            f"vane_{index} articulation axis and limits",
            axis_ok and limits_ok,
            details=f"axis={joint.axis}, limits={limits}",
        )
        ctx.expect_within(
            vane,
            frame,
            axes="xz",
            margin=0.0,
            name=f"vane_{index} sits within frame silhouette",
        )
        ctx.expect_contact(
            vane,
            frame,
            contact_tol=1e-5,
            name=f"vane_{index} mounts on frame pivots",
        )

    rest_mid_aabb = ctx.part_world_aabb(mid_vane)
    rest_neighbor_pos = ctx.part_world_position(neighbor_vane)
    with ctx.pose({mid_joint: VANE_LIMIT}):
        opened_mid_aabb = ctx.part_world_aabb(mid_vane)
        opened_neighbor_pos = ctx.part_world_position(neighbor_vane)

    rest_y_span = None
    opened_y_span = None
    if rest_mid_aabb is not None:
        rest_y_span = rest_mid_aabb[1][1] - rest_mid_aabb[0][1]
    if opened_mid_aabb is not None:
        opened_y_span = opened_mid_aabb[1][1] - opened_mid_aabb[0][1]

    ctx.check(
        "mid vane rotates out of plane",
        rest_y_span is not None
        and opened_y_span is not None
        and opened_y_span > rest_y_span + 0.008,
        details=f"rest_y_span={rest_y_span}, opened_y_span={opened_y_span}",
    )
    ctx.check(
        "vane articulations are uncoupled",
        rest_neighbor_pos is not None
        and opened_neighbor_pos is not None
        and max(
            abs(opened_neighbor_pos[0] - rest_neighbor_pos[0]),
            abs(opened_neighbor_pos[1] - rest_neighbor_pos[1]),
            abs(opened_neighbor_pos[2] - rest_neighbor_pos[2]),
        )
        < 1e-6,
        details=f"rest_neighbor_pos={rest_neighbor_pos}, opened_neighbor_pos={opened_neighbor_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
