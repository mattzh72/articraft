from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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


HOUSING_WIDTH = 0.78
HOUSING_HEIGHT = 0.58
HOUSING_DEPTH = 0.09
FRAME_SIDE = 0.05
FRAME_TOP = 0.045
BACK_THICKNESS = 0.006
PIVOT_PAD_LENGTH = 0.008
PIVOT_PAD_DEPTH = 0.020
PIVOT_PAD_HEIGHT = 0.028

OPENING_WIDTH = HOUSING_WIDTH - 2.0 * FRAME_SIDE
OPENING_HEIGHT = HOUSING_HEIGHT - 2.0 * FRAME_TOP

VANE_COUNT = 6
VANE_PITCH = 0.074
VANE_DEPTH = 0.056
VANE_THICKNESS = 0.012
VANE_COLLAR_LENGTH = 0.004
VANE_BOSS_LENGTH = 0.008
VANE_COLLAR_RADIUS = 0.010
VANE_BOSS_RADIUS = 0.007
VANE_BLADE_LENGTH = OPENING_WIDTH - 2.0 * (
    VANE_COLLAR_LENGTH + VANE_BOSS_LENGTH + PIVOT_PAD_LENGTH
)
VANE_TOTAL_LENGTH = VANE_BLADE_LENGTH + 2.0 * (VANE_COLLAR_LENGTH + VANE_BOSS_LENGTH)
VANE_LIMIT = radians(38.0)


def _vane_z_positions() -> list[float]:
    mid = (VANE_COUNT - 1) / 2.0
    return [(mid - index) * VANE_PITCH for index in range(VANE_COUNT)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_louver_bank")

    model.material("housing_finish", rgba=(0.22, 0.24, 0.26, 1.0))
    model.material("vane_finish", rgba=(0.78, 0.80, 0.82, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH, BACK_THICKNESS, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, -HOUSING_DEPTH / 2.0 + BACK_THICKNESS / 2.0, 0.0)),
        material="housing_finish",
        name="back_panel",
    )
    housing.visual(
        Box((FRAME_SIDE, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(-(HOUSING_WIDTH - FRAME_SIDE) / 2.0, 0.0, 0.0)),
        material="housing_finish",
        name="left_side_rail",
    )
    housing.visual(
        Box((FRAME_SIDE, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=((HOUSING_WIDTH - FRAME_SIDE) / 2.0, 0.0, 0.0)),
        material="housing_finish",
        name="right_side_rail",
    )
    housing.visual(
        Box((OPENING_WIDTH, HOUSING_DEPTH, FRAME_TOP)),
        origin=Origin(xyz=(0.0, 0.0, (HOUSING_HEIGHT - FRAME_TOP) / 2.0)),
        material="housing_finish",
        name="top_rail",
    )
    housing.visual(
        Box((OPENING_WIDTH, HOUSING_DEPTH, FRAME_TOP)),
        origin=Origin(xyz=(0.0, 0.0, -(HOUSING_HEIGHT - FRAME_TOP) / 2.0)),
        material="housing_finish",
        name="bottom_rail",
    )

    for index, z_pos in enumerate(_vane_z_positions(), start=1):
        housing.visual(
            Box((PIVOT_PAD_LENGTH, PIVOT_PAD_DEPTH, PIVOT_PAD_HEIGHT)),
            origin=Origin(
                xyz=(
                    -(OPENING_WIDTH / 2.0 - PIVOT_PAD_LENGTH / 2.0),
                    0.0,
                    z_pos,
                )
            ),
            material="housing_finish",
            name=f"left_pivot_pad_{index}",
        )
        housing.visual(
            Box((PIVOT_PAD_LENGTH, PIVOT_PAD_DEPTH, PIVOT_PAD_HEIGHT)),
            origin=Origin(
                xyz=(
                    OPENING_WIDTH / 2.0 - PIVOT_PAD_LENGTH / 2.0,
                    0.0,
                    z_pos,
                )
            ),
            material="housing_finish",
            name=f"right_pivot_pad_{index}",
        )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        mass=14.0,
    )

    for index, z_pos in enumerate(_vane_z_positions(), start=1):
        vane = model.part(f"vane_{index}")
        vane.visual(
            Box((VANE_BLADE_LENGTH, VANE_DEPTH, VANE_THICKNESS)),
            material="vane_finish",
            name="blade",
        )
        vane.visual(
            Cylinder(radius=VANE_COLLAR_RADIUS, length=VANE_COLLAR_LENGTH),
            origin=Origin(
                xyz=(-(VANE_BLADE_LENGTH + VANE_COLLAR_LENGTH) / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="vane_finish",
            name="left_collar",
        )
        vane.visual(
            Cylinder(radius=VANE_COLLAR_RADIUS, length=VANE_COLLAR_LENGTH),
            origin=Origin(
                xyz=((VANE_BLADE_LENGTH + VANE_COLLAR_LENGTH) / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="vane_finish",
            name="right_collar",
        )
        vane.visual(
            Cylinder(radius=VANE_BOSS_RADIUS, length=VANE_BOSS_LENGTH),
            origin=Origin(
                xyz=(
                    -(
                        VANE_BLADE_LENGTH / 2.0
                        + VANE_COLLAR_LENGTH
                        + VANE_BOSS_LENGTH / 2.0
                    ),
                    0.0,
                    0.0,
                ),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="vane_finish",
            name="left_boss",
        )
        vane.visual(
            Cylinder(radius=VANE_BOSS_RADIUS, length=VANE_BOSS_LENGTH),
            origin=Origin(
                xyz=(
                    VANE_BLADE_LENGTH / 2.0
                    + VANE_COLLAR_LENGTH
                    + VANE_BOSS_LENGTH / 2.0,
                    0.0,
                    0.0,
                ),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="vane_finish",
            name="right_boss",
        )
        vane.inertial = Inertial.from_geometry(
            Box((VANE_TOTAL_LENGTH, VANE_DEPTH, VANE_THICKNESS)),
            mass=0.85,
        )
        model.articulation(
            f"housing_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.2,
                lower=-VANE_LIMIT,
                upper=VANE_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    vanes = [object_model.get_part(f"vane_{index}") for index in range(1, VANE_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"housing_to_vane_{index}")
        for index in range(1, VANE_COUNT + 1)
    ]

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
        "all_vanes_use_parallel_revolute_axes",
        all(
            abs(joint.axis[0]) > 0.999
            and abs(joint.axis[1]) < 1e-9
            and abs(joint.axis[2]) < 1e-9
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper
            for joint in joints
        ),
        details="Every vane should rotate about the shared horizontal x-axis with bidirectional limits.",
    )

    for index, vane in enumerate(vanes, start=1):
        ctx.expect_contact(
            vane,
            housing,
            contact_tol=0.0006,
            name=f"vane_{index}_bosses_touch_housing",
        )

    for index in range(VANE_COUNT - 1):
        ctx.expect_gap(
            vanes[index],
            vanes[index + 1],
            axis="z",
            min_gap=0.02,
            name=f"rest_gap_vane_{index + 1}_to_vane_{index + 2}",
        )

    with ctx.pose({joint: radians(32.0) for joint in joints}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_overlap_free")
        for index in range(VANE_COUNT - 1):
            ctx.expect_gap(
                vanes[index],
                vanes[index + 1],
                axis="z",
                min_gap=0.008,
                name=f"open_gap_vane_{index + 1}_to_vane_{index + 2}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
