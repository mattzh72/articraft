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


FRAME_FLANGE_WIDTH = 0.430
FRAME_FLANGE_HEIGHT = 0.250
FRAME_BODY_WIDTH = 0.400
FRAME_BODY_HEIGHT = 0.220
FRAME_TOTAL_DEPTH = 0.045
FRAME_FLANGE_THICKNESS = 0.004
FRAME_BODY_DEPTH = FRAME_TOTAL_DEPTH - FRAME_FLANGE_THICKNESS

OPENING_WIDTH = 0.366
OPENING_HEIGHT = 0.178

BLADE_COUNT = 5
BLADE_PITCH = 0.032
BLADE_CHORD = 0.026
BLADE_THICKNESS = 0.0032
PIVOT_RADIUS = 0.003
BLADE_PIN_EXPOSED = 0.006
PIN_EMBED = 0.001
PIN_TOTAL_LENGTH = BLADE_PIN_EXPOSED + PIN_EMBED
BLADE_BODY_LENGTH = OPENING_WIDTH - (2.0 * BLADE_PIN_EXPOSED)

BLADE_LIMIT = 0.90
POSE_CHECK_ANGLE = 0.75

BLADE_ZS = tuple(
    (index - ((BLADE_COUNT - 1) / 2.0)) * BLADE_PITCH for index in range(BLADE_COUNT)
)


FRAME_FRONT_Y = (-FRAME_TOTAL_DEPTH / 2.0) + (FRAME_FLANGE_THICKNESS / 2.0)
FRAME_REAR_Y = FRAME_FRONT_Y + (FRAME_FLANGE_THICKNESS / 2.0) + (FRAME_BODY_DEPTH / 2.0)
FRAME_FLANGE_SIDE_WIDTH = (FRAME_FLANGE_WIDTH - OPENING_WIDTH) / 2.0
FRAME_FLANGE_RAIL_HEIGHT = (FRAME_FLANGE_HEIGHT - OPENING_HEIGHT) / 2.0
FRAME_BODY_SIDE_WIDTH = (FRAME_BODY_WIDTH - OPENING_WIDTH) / 2.0
FRAME_BODY_RAIL_HEIGHT = (FRAME_BODY_HEIGHT - OPENING_HEIGHT) / 2.0
PIN_CENTER_OFFSET = (BLADE_BODY_LENGTH / 2.0) + ((BLADE_PIN_EXPOSED - PIN_EMBED) / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_vent_louver")
    model.material("frame_paint", rgba=(0.86, 0.87, 0.88, 1.0))
    model.material("blade_metal", rgba=(0.74, 0.76, 0.79, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_FLANGE_SIDE_WIDTH, FRAME_FLANGE_THICKNESS, FRAME_FLANGE_HEIGHT)),
        material="frame_paint",
        name="front_left_stile",
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH / 2.0 + FRAME_FLANGE_SIDE_WIDTH / 2.0),
                FRAME_FRONT_Y,
                0.0,
            )
        ),
    )
    frame.visual(
        Box((FRAME_FLANGE_SIDE_WIDTH, FRAME_FLANGE_THICKNESS, FRAME_FLANGE_HEIGHT)),
        material="frame_paint",
        name="front_right_stile",
        origin=Origin(
            xyz=(
                OPENING_WIDTH / 2.0 + FRAME_FLANGE_SIDE_WIDTH / 2.0,
                FRAME_FRONT_Y,
                0.0,
            )
        ),
    )
    frame.visual(
        Box((FRAME_FLANGE_WIDTH, FRAME_FLANGE_THICKNESS, FRAME_FLANGE_RAIL_HEIGHT)),
        material="frame_paint",
        name="front_top_rail",
        origin=Origin(
            xyz=(
                0.0,
                FRAME_FRONT_Y,
                OPENING_HEIGHT / 2.0 + FRAME_FLANGE_RAIL_HEIGHT / 2.0,
            )
        ),
    )
    frame.visual(
        Box((FRAME_FLANGE_WIDTH, FRAME_FLANGE_THICKNESS, FRAME_FLANGE_RAIL_HEIGHT)),
        material="frame_paint",
        name="front_bottom_rail",
        origin=Origin(
            xyz=(
                0.0,
                FRAME_FRONT_Y,
                -(OPENING_HEIGHT / 2.0 + FRAME_FLANGE_RAIL_HEIGHT / 2.0),
            )
        ),
    )
    frame.visual(
        Box((FRAME_BODY_SIDE_WIDTH, FRAME_BODY_DEPTH, FRAME_BODY_HEIGHT)),
        material="frame_paint",
        name="rear_left_stile",
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH / 2.0 + FRAME_BODY_SIDE_WIDTH / 2.0),
                FRAME_REAR_Y,
                0.0,
            )
        ),
    )
    frame.visual(
        Box((FRAME_BODY_SIDE_WIDTH, FRAME_BODY_DEPTH, FRAME_BODY_HEIGHT)),
        material="frame_paint",
        name="rear_right_stile",
        origin=Origin(
            xyz=(
                OPENING_WIDTH / 2.0 + FRAME_BODY_SIDE_WIDTH / 2.0,
                FRAME_REAR_Y,
                0.0,
            )
        ),
    )
    frame.visual(
        Box((FRAME_BODY_WIDTH, FRAME_BODY_DEPTH, FRAME_BODY_RAIL_HEIGHT)),
        material="frame_paint",
        name="rear_top_rail",
        origin=Origin(
            xyz=(
                0.0,
                FRAME_REAR_Y,
                OPENING_HEIGHT / 2.0 + FRAME_BODY_RAIL_HEIGHT / 2.0,
            )
        ),
    )
    frame.visual(
        Box((FRAME_BODY_WIDTH, FRAME_BODY_DEPTH, FRAME_BODY_RAIL_HEIGHT)),
        material="frame_paint",
        name="rear_bottom_rail",
        origin=Origin(
            xyz=(
                0.0,
                FRAME_REAR_Y,
                -(OPENING_HEIGHT / 2.0 + FRAME_BODY_RAIL_HEIGHT / 2.0),
            )
        ),
    )

    for index, z in enumerate(BLADE_ZS, start=1):
        blade = model.part(f"blade_{index}")
        blade.visual(
            Box((BLADE_BODY_LENGTH, BLADE_CHORD, BLADE_THICKNESS)),
            material="blade_metal",
            name="blade_body",
        )
        blade.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIN_TOTAL_LENGTH),
            material="blade_metal",
            name="left_pivot_pin",
            origin=Origin(
                xyz=(-PIN_CENTER_OFFSET, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
        )
        blade.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIN_TOTAL_LENGTH),
            material="blade_metal",
            name="right_pivot_pin",
            origin=Origin(
                xyz=(PIN_CENTER_OFFSET, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
        )
        model.articulation(
            f"frame_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.5,
                lower=-BLADE_LIMIT,
                upper=BLADE_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    blades = [object_model.get_part(f"blade_{index}") for index in range(1, BLADE_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"frame_to_blade_{index}")
        for index in range(1, BLADE_COUNT + 1)
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

    root_parts = object_model.root_parts()
    ctx.check(
        "single_fixed_frame_root",
        len(root_parts) == 1 and root_parts[0].name == "frame",
        f"expected only frame as root, got {[part.name for part in root_parts]}",
    )

    for blade, joint in zip(blades, joints):
        ctx.expect_contact(
            blade,
            frame,
            name=f"{blade.name}_pivot_contact_closed",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_definition",
            joint.articulation_type == ArticulationType.REVOLUTE
            and abs(joint.axis[0] - 1.0) < 1e-9
            and abs(joint.axis[1]) < 1e-9
            and abs(joint.axis[2]) < 1e-9
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + BLADE_LIMIT) < 1e-9
            and abs(limits.upper - BLADE_LIMIT) < 1e-9,
            f"joint={joint.name} axis={joint.axis} limits={limits}",
        )

    for lower_blade, upper_blade in zip(blades, blades[1:]):
        ctx.expect_origin_gap(
            upper_blade,
            lower_blade,
            axis="z",
            min_gap=BLADE_PITCH - 0.001,
            max_gap=BLADE_PITCH + 0.001,
            name=f"{lower_blade.name}_to_{upper_blade.name}_pitch",
        )

    with ctx.pose({joint: POSE_CHECK_ANGLE for joint in joints}):
        for blade in blades:
            ctx.expect_contact(
                blade,
                frame,
                name=f"{blade.name}_pivot_contact_open",
            )
        ctx.fail_if_parts_overlap_in_current_pose(name="opened_pose_no_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
