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


FRAME_WIDTH = 0.58
FRAME_DEPTH = 0.18
BASE_HEIGHT = 0.055
BRIDGE_WIDTH = 0.52
BRIDGE_DEPTH = 0.16
BRIDGE_HEIGHT = 0.045
FRAME_HEIGHT = 0.72

COLUMN_SIZE = 0.05
COLUMN_CENTER_X = 0.18
COLUMN_HEIGHT = FRAME_HEIGHT - BASE_HEIGHT - BRIDGE_HEIGHT
COLUMN_CENTER_Z = BASE_HEIGHT + (COLUMN_HEIGHT / 2.0)

LIFT_ORIGIN_Z = 0.40
LIFT_TRAVEL = 0.23


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _build_guide_block_shape() -> cq.Workplane:
    outer_width = 0.092
    outer_depth = 0.09
    outer_height = 0.12
    guide_clearance = 0.004

    block = cq.Workplane("XY").box(outer_width, outer_depth, outer_height)
    block = block.cut(
        cq.Workplane("XY").box(
            COLUMN_SIZE + guide_clearance,
            COLUMN_SIZE + guide_clearance,
            outer_height + 0.01,
        )
    )
    return block.edges("|Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_bridge_lift_carriage")

    model.material("frame_paint", rgba=(0.26, 0.29, 0.32, 1.0))
    model.material("guide_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_paint", rgba=(0.90, 0.64, 0.16, 1.0))
    model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        size=(FRAME_WIDTH, FRAME_DEPTH, BASE_HEIGHT),
        xyz=(0.0, 0.0, BASE_HEIGHT / 2.0),
        material="frame_paint",
        name="lower_base",
    )
    _add_box(
        frame,
        size=(0.20, 0.10, 0.02),
        xyz=(-0.16, 0.0, 0.01),
        material="frame_paint",
        name="left_foot",
    )
    _add_box(
        frame,
        size=(0.20, 0.10, 0.02),
        xyz=(0.16, 0.0, 0.01),
        material="frame_paint",
        name="right_foot",
    )
    _add_box(
        frame,
        size=(BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_HEIGHT),
        xyz=(0.0, 0.0, FRAME_HEIGHT - (BRIDGE_HEIGHT / 2.0)),
        material="frame_paint",
        name="top_bridge",
    )
    _add_box(
        frame,
        size=(COLUMN_SIZE, COLUMN_SIZE, COLUMN_HEIGHT),
        xyz=(-COLUMN_CENTER_X, 0.0, COLUMN_CENTER_Z),
        material="guide_steel",
        name="left_column",
    )
    _add_box(
        frame,
        size=(COLUMN_SIZE, COLUMN_SIZE, COLUMN_HEIGHT),
        xyz=(COLUMN_CENTER_X, 0.0, COLUMN_CENTER_Z),
        material="guide_steel",
        name="right_column",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_guide_block_shape(), "left_guide_block"),
        origin=Origin(xyz=(-COLUMN_CENTER_X, 0.0, -0.03)),
        material="carriage_paint",
        name="left_guide_block",
    )
    carriage.visual(
        mesh_from_cadquery(_build_guide_block_shape(), "right_guide_block"),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.0, -0.03)),
        material="carriage_paint",
        name="right_guide_block",
    )
    carriage.visual(
        Box((0.054, 0.004, 0.10)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, 0.027, -0.03)),
        material="pad_black",
        name="left_front_pad",
    )
    carriage.visual(
        Box((0.054, 0.004, 0.10)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, -0.027, -0.03)),
        material="pad_black",
        name="left_rear_pad",
    )
    carriage.visual(
        Box((0.054, 0.004, 0.10)),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.027, -0.03)),
        material="pad_black",
        name="right_front_pad",
    )
    carriage.visual(
        Box((0.054, 0.004, 0.10)),
        origin=Origin(xyz=(COLUMN_CENTER_X, -0.027, -0.03)),
        material="pad_black",
        name="right_rear_pad",
    )
    carriage.visual(
        Box((0.278, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material="carriage_paint",
        name="top_tie",
    )
    carriage.visual(
        Box((0.028, 0.022, 0.25)),
        origin=Origin(xyz=(-0.122, 0.0, -0.165)),
        material="carriage_paint",
        name="left_hanger",
    )
    carriage.visual(
        Box((0.028, 0.022, 0.25)),
        origin=Origin(xyz=(0.122, 0.0, -0.165)),
        material="carriage_paint",
        name="right_hanger",
    )
    carriage.visual(
        Box((0.23, 0.11, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.305)),
        material="carriage_paint",
        name="load_platform",
    )
    carriage.visual(
        Box((0.23, 0.014, 0.048)),
        origin=Origin(xyz=(0.0, 0.048, -0.296)),
        material="carriage_paint",
        name="front_lip",
    )
    carriage.visual(
        Box((0.09, 0.038, 0.08)),
        origin=Origin(xyz=(0.0, -0.018, -0.255)),
        material="carriage_paint",
        name="rear_rib",
    )
    carriage.visual(
        Box((0.18, 0.065, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.288)),
        material="pad_black",
        name="load_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.46, 0.11, 0.34)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=1600.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")
    base = frame.get_visual("lower_base")
    bridge = frame.get_visual("top_bridge")
    left_column = frame.get_visual("left_column")
    right_column = frame.get_visual("right_column")
    left_guide = carriage.get_visual("left_guide_block")
    right_guide = carriage.get_visual("right_guide_block")
    left_front_pad = carriage.get_visual("left_front_pad")
    right_front_pad = carriage.get_visual("right_front_pad")

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

    limits = lift.motion_limits
    ctx.check(
        "lift_axis_is_vertical",
        lift.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical prismatic axis, got {lift.axis}",
    )
    ctx.check(
        "lift_has_single_upward_travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and 0.18 <= limits.upper <= 0.30,
        details=f"unexpected motion limits: {limits}",
    )

    ctx.expect_overlap(
        carriage,
        frame,
        axes="yz",
        elem_a=left_guide,
        elem_b=left_column,
        min_overlap=0.045,
        name="left_guide_block_tracks_left_column",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="yz",
        elem_a=right_guide,
        elem_b=right_column,
        min_overlap=0.045,
        name="right_guide_block_tracks_right_column",
    )
    ctx.expect_contact(
        carriage,
        frame,
        elem_a=left_front_pad,
        elem_b=left_column,
        contact_tol=1e-6,
        name="left_pad_contacts_left_column",
    )
    ctx.expect_contact(
        carriage,
        frame,
        elem_a=right_front_pad,
        elem_b=right_column,
        contact_tol=1e-6,
        name="right_pad_contacts_right_column",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        negative_elem=base,
        min_gap=0.015,
        max_gap=0.04,
        name="carriage_clears_lower_base_at_bottom",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem=bridge,
        min_gap=0.22,
        max_gap=0.30,
        name="carriage_hangs_below_top_bridge_at_bottom",
    )

    low_z = ctx.part_world_position(carriage)
    high_z = None
    if limits is not None and limits.upper is not None:
        with ctx.pose({lift: limits.upper}):
            high_z = ctx.part_world_position(carriage)
            ctx.expect_contact(
                carriage,
                frame,
                elem_a=left_front_pad,
                elem_b=left_column,
                contact_tol=1e-6,
                name="left_pad_stays_in_contact_with_left_column_at_top",
            )
            ctx.expect_contact(
                carriage,
                frame,
                elem_a=right_front_pad,
                elem_b=right_column,
                contact_tol=1e-6,
                name="right_pad_stays_in_contact_with_right_column_at_top",
            )
            ctx.expect_gap(
                frame,
                carriage,
                axis="z",
                positive_elem=bridge,
                min_gap=0.015,
                max_gap=0.03,
                name="carriage_stops_below_bridge_at_top",
            )
    ctx.check(
        "lift_moves_carriage_upward",
        low_z is not None and high_z is not None and high_z[2] > low_z[2] + 0.20,
        details=f"expected upward travel > 0.20 m, got low={low_z}, high={high_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
