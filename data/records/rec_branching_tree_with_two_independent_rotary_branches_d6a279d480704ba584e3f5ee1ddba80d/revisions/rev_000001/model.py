from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_RADIUS = 0.22
BASE_THICK = 0.045
PLINTH_SIZE = 0.18
PLINTH_HEIGHT = 0.05
COLUMN_RADIUS = 0.05
COLUMN_HEIGHT = 1.12
TOP_CAP_THICK = 0.018

BLOCK_HEIGHT = 0.07
BLOCK_OUTER_RADIUS = 0.08
EAR_LENGTH = 0.12
EAR_WIDTH = 0.09
PAD_RADIUS = 0.042
PAD_HEIGHT = 0.015
PAD_CENTER_X = 0.135

HUB_RADIUS = 0.04
HUB_HEIGHT = 0.028
ARM_LENGTH = 0.29
ARM_WIDTH = 0.055
ARM_THICK = 0.036
STEM_LENGTH = 0.05
STEM_WIDTH = 0.04
STEM_HEIGHT = 0.085
FACEPLATE_THICK = 0.012
FACEPLATE_WIDTH = 0.095
FACEPLATE_HEIGHT = 0.11

LOWER_BLOCK_Z = BASE_THICK + 0.38
UPPER_BLOCK_Z = BASE_THICK + 0.76


def _make_pedestal_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICK)
    plinth = cq.Workplane("XY").box(PLINTH_SIZE, PLINTH_SIZE, PLINTH_HEIGHT).translate(
        (0.0, 0.0, BASE_THICK + PLINTH_HEIGHT / 2.0)
    )
    column = cq.Workplane("XY").circle(COLUMN_RADIUS).extrude(COLUMN_HEIGHT).translate(
        (0.0, 0.0, BASE_THICK)
    )
    cap = cq.Workplane("XY").circle(COLUMN_RADIUS * 1.08).extrude(TOP_CAP_THICK).translate(
        (0.0, 0.0, BASE_THICK + COLUMN_HEIGHT)
    )

    gusset_profile = [
        (COLUMN_RADIUS + 0.004, BASE_THICK),
        (0.145, BASE_THICK),
        (0.115, BASE_THICK + 0.16),
        (COLUMN_RADIUS + 0.004, BASE_THICK + 0.24),
    ]
    gusset = (
        cq.Workplane("XZ")
        .polyline(gusset_profile)
        .close()
        .extrude(0.018, both=True)
    )

    pedestal = base.union(plinth).union(column).union(cap)
    for angle in (0, 90, 180, 270):
        pedestal = pedestal.union(gusset.rotate((0, 0, 0), (0, 0, 1), angle))

    lower_block = _make_support_block_shape().translate((0.0, 0.0, LOWER_BLOCK_Z))
    upper_block = _make_support_block_shape().rotate((0, 0, 0), (0, 0, 1), 180).translate(
        (0.0, 0.0, UPPER_BLOCK_Z)
    )
    pedestal = pedestal.union(lower_block).union(upper_block)
    return pedestal


def _make_support_block_shape() -> cq.Workplane:
    body_length = 0.10
    body_width = 0.08
    jaw_length = 0.022

    body = (
        cq.Workplane("XY")
        .box(body_length, body_width, BLOCK_HEIGHT)
        .translate((COLUMN_RADIUS + body_length / 2.0, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.008)
    )
    jaw = cq.Workplane("XY").box(jaw_length, body_width * 0.88, BLOCK_HEIGHT * 0.92).translate(
        (COLUMN_RADIUS + jaw_length / 2.0, 0.0, 0.0)
    )
    pad = cq.Workplane("XY").circle(PAD_RADIUS).extrude(PAD_HEIGHT).translate(
        (PAD_CENTER_X, 0.0, BLOCK_HEIGHT / 2.0)
    )
    web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.082, -BLOCK_HEIGHT * 0.20),
                (PAD_CENTER_X - 0.024, -BLOCK_HEIGHT * 0.20),
                (PAD_CENTER_X - 0.006, BLOCK_HEIGHT * 0.22),
                (0.092, BLOCK_HEIGHT * 0.14),
            ]
        )
        .close()
        .extrude(body_width * 0.5, both=True)
    )

    return body.union(jaw).union(web).union(pad)


def _make_branch_core_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(HUB_RADIUS).extrude(HUB_HEIGHT)
    beam = cq.Workplane("XY").box(ARM_LENGTH, ARM_WIDTH, ARM_THICK).translate(
        (ARM_LENGTH / 2.0, 0.0, HUB_HEIGHT + ARM_THICK / 2.0 - 0.01)
    )
    stem = cq.Workplane("XY").box(STEM_LENGTH, STEM_WIDTH, STEM_HEIGHT).translate(
        (ARM_LENGTH - 0.005, 0.0, 0.065)
    )
    rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (ARM_LENGTH - 0.10, HUB_HEIGHT * 0.55),
                (ARM_LENGTH - 0.025, HUB_HEIGHT * 0.55),
                (ARM_LENGTH - 0.025, 0.082),
            ]
        )
        .close()
        .extrude(0.038, both=True)
    )

    return hub.union(beam).union(stem).union(rib)


def _make_faceplate_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(
        FACEPLATE_THICK,
        FACEPLATE_WIDTH,
        FACEPLATE_HEIGHT,
    ).translate((ARM_LENGTH + 0.023, 0.0, 0.095))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tooling_tree")

    model.material("pedestal_steel", color=(0.22, 0.24, 0.27))
    model.material("block_steel", color=(0.42, 0.44, 0.47))
    model.material("arm_paint", color=(0.83, 0.40, 0.12))
    model.material("plate_steel", color=(0.76, 0.78, 0.80))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal_shape(), "pedestal_shell"),
        material="pedestal_steel",
        name="pedestal_shell",
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        mesh_from_cadquery(_make_branch_core_shape(), "lower_branch_core"),
        material="arm_paint",
        name="arm_core",
    )
    lower_branch.visual(
        mesh_from_cadquery(_make_faceplate_shape(), "lower_branch_faceplate"),
        material="plate_steel",
        name="faceplate",
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        mesh_from_cadquery(_make_branch_core_shape(), "upper_branch_core"),
        material="arm_paint",
        name="arm_core",
    )
    upper_branch.visual(
        mesh_from_cadquery(_make_faceplate_shape(), "upper_branch_faceplate"),
        material="plate_steel",
        name="faceplate",
    )

    branch_limits = MotionLimits(
        effort=25.0,
        velocity=1.5,
        lower=-1.75,
        upper=1.75,
    )
    model.articulation(
        "pedestal_to_lower_branch",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=lower_branch,
        origin=Origin(
            xyz=(PAD_CENTER_X, 0.0, LOWER_BLOCK_Z + BLOCK_HEIGHT / 2.0 + PAD_HEIGHT)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=branch_limits,
    )
    model.articulation(
        "pedestal_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_branch,
        origin=Origin(
            xyz=(-PAD_CENTER_X, 0.0, UPPER_BLOCK_Z + BLOCK_HEIGHT / 2.0 + PAD_HEIGHT),
            rpy=(0.0, 0.0, math.pi),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-1.75,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    lower_branch = object_model.get_part("lower_branch")
    upper_branch = object_model.get_part("upper_branch")
    lower_joint = object_model.get_articulation("pedestal_to_lower_branch")
    upper_joint = object_model.get_articulation("pedestal_to_upper_branch")

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
        "all_prompt_parts_present",
        all(part is not None for part in (pedestal, lower_branch, upper_branch)),
        "pedestal assembly and both rotary branches must all exist",
    )
    ctx.check(
        "independent_branch_joint_wiring",
        lower_joint.parent == pedestal.name
        and lower_joint.child == lower_branch.name
        and upper_joint.parent == pedestal.name
        and upper_joint.child == upper_branch.name,
        "each branch must hinge independently from the pedestal assembly with no shared linkage",
    )
    ctx.check(
        "branch_axes_are_vertical",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0) and tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        "both rotary branches should swivel about vertical axes at their own support pads",
    )
    ctx.check(
        "joint_origins_are_mirrored_and_stacked",
        lower_joint.origin.xyz[0] > 0.10
        and upper_joint.origin.xyz[0] < -0.10
        and 0.30 < (upper_joint.origin.xyz[2] - lower_joint.origin.xyz[2]) < 0.45,
        "the two branch pivots should sit on opposite support blocks at distinct heights",
    )

    ctx.expect_contact(
        lower_branch,
        pedestal,
        contact_tol=0.002,
        name="lower_branch_is_supported_by_lower_clamp_block",
    )
    ctx.expect_contact(
        upper_branch,
        pedestal,
        contact_tol=0.002,
        name="upper_branch_is_supported_by_upper_clamp_block",
    )

    def _elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    def _xy_shift(
        before: tuple[float, float, float] | None,
        after: tuple[float, float, float] | None,
    ) -> float:
        if before is None or after is None:
            return 0.0
        return math.hypot(after[0] - before[0], after[1] - before[1])

    lower_rest_plate = _elem_center("lower_branch", "faceplate")
    upper_rest_plate = _elem_center("upper_branch", "faceplate")
    ctx.check(
        "branch_faceplates_resolve",
        lower_rest_plate is not None and upper_rest_plate is not None,
        "each branch should expose a named rectangular faceplate visual",
    )

    with ctx.pose({lower_joint: 1.1}):
        lower_rotated_plate = _elem_center("lower_branch", "faceplate")
        upper_when_lower_moves = _elem_center("upper_branch", "faceplate")
        ctx.fail_if_parts_overlap_in_current_pose(name="lower_branch_clearance_in_rotated_pose")
    ctx.check(
        "lower_branch_moves_without_dragging_upper_branch",
        _xy_shift(lower_rest_plate, lower_rotated_plate) > 0.12
        and _xy_shift(upper_rest_plate, upper_when_lower_moves) < 1e-5,
        "rotating the lower branch should move only its own faceplate",
    )

    with ctx.pose({upper_joint: -1.1}):
        upper_rotated_plate = _elem_center("upper_branch", "faceplate")
        lower_when_upper_moves = _elem_center("lower_branch", "faceplate")
        ctx.fail_if_parts_overlap_in_current_pose(name="upper_branch_clearance_in_rotated_pose")
    ctx.check(
        "upper_branch_moves_without_dragging_lower_branch",
        _xy_shift(upper_rest_plate, upper_rotated_plate) > 0.12
        and _xy_shift(lower_rest_plate, lower_when_upper_moves) < 1e-5,
        "rotating the upper branch should move only its own faceplate",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
