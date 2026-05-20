from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)


FOOT_L = 0.250
FOOT_W = 0.130
FOOT_T = 0.018
FOOT_FILLET = 0.004
MOUNT_HOLE_D = 0.009
MOUNT_CBORE_D = 0.016
MOUNT_CBORE_DEPTH = 0.005
MOUNT_POINTS = (
    (-0.090, -0.040),
    (-0.090, 0.040),
    (0.090, -0.040),
    (0.090, 0.040),
)

AXIS_Z = 0.060
BLOCK_X = 0.060
BLOCK_LEN = 0.034
BLOCK_PAD_W = 0.076
BLOCK_PAD_H = 0.012
BLOCK_CHEEK_T = 0.014
SHAFT_R = 0.018
CONTACT_PAD_R = 0.004
CONTACT_PAD_Y = SHAFT_R + CONTACT_PAD_R
CHEEK_INNER_Y = CONTACT_PAD_Y + CONTACT_PAD_R - 0.002
CHEEK_CENTER_Y = CHEEK_INNER_Y + (BLOCK_CHEEK_T / 2.0)
BLOCK_BRIDGE_T = 0.010
BLOCK_AXIS_Z = AXIS_Z - FOOT_T
BLOCK_BRIDGE_Z = BLOCK_AXIS_Z + SHAFT_R + 0.004 + (BLOCK_BRIDGE_T / 2.0)
BLOCK_CHEEK_H = BLOCK_BRIDGE_Z + (BLOCK_BRIDGE_T / 2.0)
BLOCK_TOP_Z = FOOT_T + BLOCK_CHEEK_H

SHAFT_L = 0.200
FLANGE_R = 0.032
FLANGE_T = 0.012
FLANGE_X = -0.094
HUB_R = 0.024
HUB_L = 0.026
HUB_X = 0.008
EAR_W = 0.014
EAR_H = 0.020
EAR_Y = HUB_R + 0.006
SPINDLE_CLEAR_FOOT = 0.001


def _make_foot_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(FOOT_L, FOOT_W, FOOT_T, centered=(True, True, False))
    foot = foot.edges("|Z").fillet(FOOT_FILLET)
    foot = (
        foot.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(MOUNT_POINTS)
        .cboreHole(MOUNT_HOLE_D, MOUNT_CBORE_D, MOUNT_CBORE_DEPTH)
    )
    return foot


def _x_axis_origin(
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _add_bearing_block_visuals(part) -> None:
    part.visual(
        Box((BLOCK_LEN, BLOCK_PAD_W, BLOCK_PAD_H)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_PAD_H / 2.0)),
        material="module_gray",
        name="pad",
    )
    part.visual(
        Box((BLOCK_LEN, BLOCK_CHEEK_T, BLOCK_CHEEK_H)),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, BLOCK_CHEEK_H / 2.0)),
        material="module_gray",
        name="upper_cheek_pos_y",
    )
    part.visual(
        Box((BLOCK_LEN, BLOCK_CHEEK_T, BLOCK_CHEEK_H)),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, BLOCK_CHEEK_H / 2.0)),
        material="module_gray",
        name="upper_cheek_neg_y",
    )
    part.visual(
        Box((BLOCK_LEN, BLOCK_PAD_W, BLOCK_BRIDGE_T)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_BRIDGE_Z)),
        material="module_gray",
        name="top_bridge",
    )
    part.visual(
        Cylinder(radius=CONTACT_PAD_R, length=BLOCK_LEN - 0.004),
        origin=_x_axis_origin(y=CONTACT_PAD_Y, z=BLOCK_AXIS_Z),
        material="dark_bearing",
        name="contact_pad_pos_y",
    )
    part.visual(
        Cylinder(radius=CONTACT_PAD_R, length=BLOCK_LEN - 0.004),
        origin=_x_axis_origin(y=-CONTACT_PAD_Y, z=BLOCK_AXIS_Z),
        material="dark_bearing",
        name="contact_pad_neg_y",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_spindle_module")

    model.material("module_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("dark_bearing", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("machined_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    foot = model.part("foot")
    foot.visual(
        mesh_from_cadquery(_make_foot_shape(), "spindle_foot"),
        origin=Origin(),
        material="module_gray",
        name="foot_plate",
    )
    foot.inertial = Inertial.from_geometry(
        Box((FOOT_L, FOOT_W, FOOT_T)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, FOOT_T / 2.0)),
    )

    left_block = model.part("left_block")
    _add_bearing_block_visuals(left_block)
    left_block.inertial = Inertial.from_geometry(
        Box((BLOCK_LEN, BLOCK_PAD_W, BLOCK_CHEEK_H)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, BLOCK_CHEEK_H / 2.0)),
    )

    right_block = model.part("right_block")
    _add_bearing_block_visuals(right_block)
    right_block.inertial = Inertial.from_geometry(
        Box((BLOCK_LEN, BLOCK_PAD_W, BLOCK_CHEEK_H)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, BLOCK_CHEEK_H / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_L),
        origin=_x_axis_origin(),
        material="machined_steel",
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_T),
        origin=_x_axis_origin(x=FLANGE_X),
        material="machined_steel",
        name="flange",
    )
    spindle.visual(
        Cylinder(radius=HUB_R, length=HUB_L),
        origin=_x_axis_origin(x=HUB_X),
        material="machined_steel",
        name="clamp_hub",
    )
    spindle.visual(
        Box((HUB_L, EAR_W, EAR_H)),
        origin=Origin(xyz=(HUB_X, EAR_Y, 0.0)),
        material="machined_steel",
        name="clamp_ear",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((SHAFT_L, 0.090, 0.090)),
        mass=1.3,
        origin=Origin(),
    )

    model.articulation(
        "foot_to_left_block",
        ArticulationType.FIXED,
        parent=foot,
        child=left_block,
        origin=Origin(xyz=(-BLOCK_X, 0.0, FOOT_T)),
    )
    model.articulation(
        "foot_to_right_block",
        ArticulationType.FIXED,
        parent=foot,
        child=right_block,
        origin=Origin(xyz=(BLOCK_X, 0.0, FOOT_T)),
    )
    model.articulation(
        "foot_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=foot,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot = object_model.get_part("foot")
    left_block = object_model.get_part("left_block")
    right_block = object_model.get_part("right_block")
    spindle = object_model.get_part("spindle")
    spin = object_model.get_articulation("foot_to_spindle")

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
        "spindle uses a continuous rotary joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected CONTINUOUS, got {spin.articulation_type!r}",
    )
    ctx.check(
        "spindle joint axis runs along +X",
        abs(spin.axis[0] - 1.0) < 1e-9 and abs(spin.axis[1]) < 1e-9 and abs(spin.axis[2]) < 1e-9,
        details=f"axis was {spin.axis!r}",
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_contact(foot, left_block, name="left block is mounted to the foot")
        ctx.expect_contact(foot, right_block, name="right block is mounted to the foot")
        ctx.expect_contact(spindle, left_block, name="left bearing block carries the spindle")
        ctx.expect_contact(spindle, right_block, name="right bearing block carries the spindle")
        ctx.expect_origin_gap(
            spindle,
            foot,
            axis="z",
            min_gap=AXIS_Z - 1e-6,
            max_gap=AXIS_Z + 1e-6,
            name="spindle axis height above the foot datum",
        )

    with ctx.pose({spin: 0.0}):
        ear_0 = ctx.part_element_world_aabb(spindle, elem="clamp_ear")
    with ctx.pose({spin: math.pi / 2.0}):
        ear_90 = ctx.part_element_world_aabb(spindle, elem="clamp_ear")
        ctx.fail_if_parts_overlap_in_current_pose(name="no quarter-turn spindle collision")
    with ctx.pose({spin: -math.pi / 2.0}):
        ear_down = ctx.part_element_world_aabb(spindle, elem="clamp_ear")
        ctx.fail_if_parts_overlap_in_current_pose(name="no downward clamp-hub collision")

    rotates_visibly = (
        ear_0 is not None
        and ear_90 is not None
        and ear_90[1][2] > ear_0[1][2] + 0.015
    )
    ctx.check(
        "positive rotation lifts the clamp hub toward +Z",
        rotates_visibly,
        details=f"rest clamp-ear aabb={ear_0!r}, quarter-turn clamp-ear aabb={ear_90!r}",
    )

    clears_foot = ear_down is not None and ear_down[0][2] >= FOOT_T + SPINDLE_CLEAR_FOOT
    ctx.check(
        "clamp hub clears the foot through a downward quarter turn",
        clears_foot,
        details=(
            f"lowest clamp-ear z was {None if ear_down is None else ear_down[0][2]!r}; "
            f"required at least {FOOT_T + SPINDLE_CLEAR_FOOT:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
