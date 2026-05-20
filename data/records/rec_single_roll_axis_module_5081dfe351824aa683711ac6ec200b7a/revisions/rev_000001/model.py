from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SADDLE_LEN = 0.130
SADDLE_W = 0.150
SADDLE_H = 0.048

POST_LEN = 0.076
POST_THICK = 0.022
POST_H = 0.064
POST_Y = 0.054

BRIDGE_LEN = 0.076
BRIDGE_W = 0.100
BRIDGE_H = 0.022
BRIDGE_BOTTOM = 0.112

AXIS_Z = 0.081
CHANNEL_R = 0.032
JOURNAL_R = 0.0135
SHAFT_LEN = 0.128
SHAFT_CENTER_X = 0.026
HEAD_LEN = 0.034
HEAD_R = 0.022
HEAD_CENTER_X = -0.001
SUPPORT_PAD_LEN = 0.026
SUPPORT_PAD_R = 0.0065
LOWER_PAD_Z = AXIS_Z - HEAD_R - SUPPORT_PAD_R
UPPER_PAD_Z = AXIS_Z + HEAD_R + SUPPORT_PAD_R
FLANGE_LEN = 0.018
FLANGE_R = 0.028
FLANGE_CENTER_X = 0.073
REAR_LOCK_LEN = 0.010
REAR_LOCK_R = 0.018
REAR_LOCK_X = -0.040

INDEX_LUG_SIZE = (0.012, 0.018, 0.016)
INDEX_LUG_CENTER = (
    FLANGE_CENTER_X + 0.005,
    0.0,
    0.034,
)


def _lower_saddle_shape() -> cq.Workplane:
    saddle = cq.Workplane("XY").box(
        SADDLE_LEN,
        SADDLE_W,
        SADDLE_H,
        centered=(True, True, False),
    )
    saddle = saddle.edges("|Z").fillet(0.006)

    channel = (
        cq.Workplane("YZ")
        .center(0.0, AXIS_Z)
        .circle(CHANNEL_R)
        .extrude(SADDLE_LEN, both=True)
    )
    return saddle.cut(channel)


def _post_shape(y_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(POST_LEN, POST_THICK, POST_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, y_center, SADDLE_H))
    )


def _top_bridge_shape() -> cq.Workplane:
    bridge = (
        cq.Workplane("XY")
        .box(BRIDGE_LEN, BRIDGE_W, BRIDGE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, BRIDGE_BOTTOM))
    )
    relief = (
        cq.Workplane("YZ")
        .center(0.0, AXIS_Z)
        .circle(CHANNEL_R)
        .extrude(BRIDGE_LEN, both=True)
    )
    return bridge.cut(relief)


def _support_pad_shape(z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(SUPPORT_PAD_R)
        .extrude(SUPPORT_PAD_LEN / 2.0, both=True)
        .translate((HEAD_CENTER_X, 0.0, z_center))
    )


def _spindle_core_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .circle(JOURNAL_R)
        .extrude(SHAFT_LEN / 2.0, both=True)
        .translate((SHAFT_CENTER_X, 0.0, 0.0))
    )
    head = (
        cq.Workplane("YZ")
        .circle(HEAD_R)
        .extrude(HEAD_LEN / 2.0, both=True)
        .translate((HEAD_CENTER_X, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(FLANGE_R)
        .extrude(FLANGE_LEN / 2.0, both=True)
        .translate((FLANGE_CENTER_X, 0.0, 0.0))
    )
    rear_lock = (
        cq.Workplane("YZ")
        .circle(REAR_LOCK_R)
        .extrude(REAR_LOCK_LEN / 2.0, both=True)
        .translate((REAR_LOCK_X, 0.0, 0.0))
    )

    spindle = shaft.union(head).union(flange).union(rear_lock)

    bolt_circle_r = 0.015
    hole_r = 0.0035
    bolt_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (bolt_circle_r, 0.0),
                (-bolt_circle_r, 0.0),
                (0.0, bolt_circle_r),
                (0.0, -bolt_circle_r),
            ]
        )
        .circle(hole_r)
        .extrude(FLANGE_LEN * 1.5, both=True)
        .translate((FLANGE_CENTER_X, 0.0, 0.0))
    )
    spindle = spindle.cut(bolt_holes)

    return spindle


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_supported_roll_axis_spindle")

    support_gray = model.material("support_gray", rgba=(0.25, 0.27, 0.29, 1.0))
    machined_alloy = model.material("machined_alloy", rgba=(0.73, 0.75, 0.78, 1.0))
    oxide_black = model.material("oxide_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_module")
    base.visual(
        mesh_from_cadquery(_lower_saddle_shape(), "lower_saddle"),
        material=support_gray,
        name="lower_saddle",
    )
    base.visual(
        mesh_from_cadquery(_post_shape(-POST_Y), "left_post"),
        material=support_gray,
        name="left_post",
    )
    base.visual(
        mesh_from_cadquery(_post_shape(POST_Y), "right_post"),
        material=support_gray,
        name="right_post",
    )
    base.visual(
        mesh_from_cadquery(_top_bridge_shape(), "top_bridge"),
        material=support_gray,
        name="top_bridge",
    )
    base.visual(
        mesh_from_cadquery(_support_pad_shape(LOWER_PAD_Z), "lower_pad"),
        material=oxide_black,
        name="lower_pad",
    )
    base.visual(
        mesh_from_cadquery(_support_pad_shape(UPPER_PAD_Z), "upper_pad"),
        material=oxide_black,
        name="upper_pad",
    )

    spindle = model.part("spindle_head")
    spindle.visual(
        mesh_from_cadquery(_spindle_core_shape(), "spindle_core"),
        material=machined_alloy,
        name="spindle_core",
    )
    spindle.visual(
        Box(INDEX_LUG_SIZE),
        origin=Origin(xyz=INDEX_LUG_CENTER),
        material=oxide_black,
        name="index_lug",
    )

    model.articulation(
        "base_to_spindle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=4.0,
            lower=-2.6,
            upper=2.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_module")
    spindle = object_model.get_part("spindle_head")
    roll = object_model.get_articulation("base_to_spindle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=1e-5)
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

    limits = roll.motion_limits
    axis_ok = tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0)
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -2.5
        and limits.upper >= 2.5
    )
    ctx.check(
        "roll_axis_joint_definition",
        roll.articulation_type == ArticulationType.REVOLUTE and axis_ok and limits_ok,
        f"type={roll.articulation_type}, axis={roll.axis}, limits={limits}",
    )

    ctx.expect_overlap(
        base,
        spindle,
        axes="x",
        elem_a="lower_saddle",
        elem_b="spindle_core",
        min_overlap=0.10,
        name="spindle_runs_over_lower_saddle",
    )
    ctx.expect_contact(
        base,
        spindle,
        elem_a="lower_pad",
        elem_b="spindle_core",
        contact_tol=0.0005,
        name="spindle_supported_on_lower_pad",
    )
    ctx.expect_contact(
        base,
        spindle,
        elem_a="upper_pad",
        elem_b="spindle_core",
        contact_tol=0.0005,
        name="spindle_captured_by_upper_pad",
    )
    ctx.expect_gap(
        spindle,
        base,
        axis="z",
        positive_elem="spindle_core",
        negative_elem="lower_saddle",
        min_gap=0.004,
        max_gap=0.010,
        name="spindle_clears_lower_saddle",
    )
    ctx.expect_gap(
        base,
        spindle,
        axis="z",
        positive_elem="top_bridge",
        negative_elem="spindle_core",
        min_gap=0.003,
        max_gap=0.010,
        name="bridge_caps_spindle_without_contact",
    )

    lower_pad_aabb = ctx.part_element_world_aabb(base, elem="lower_pad")
    upper_pad_aabb = ctx.part_element_world_aabb(base, elem="upper_pad")
    spindle_aabb = ctx.part_element_world_aabb(spindle, elem="spindle_core")
    if lower_pad_aabb and upper_pad_aabb and spindle_aabb:
        spindle_center_z = _aabb_center(spindle_aabb)[2]
        captured = lower_pad_aabb[1][2] < spindle_center_z < upper_pad_aabb[0][2]
        ctx.check(
            "spindle_axis_captured_between_fixed_supports",
            captured,
            (
                f"lower_pad_max_z={lower_pad_aabb[1][2]:.4f}, "
                f"spindle_center_z={spindle_center_z:.4f}, "
                f"upper_pad_min_z={upper_pad_aabb[0][2]:.4f}"
            ),
        )

    base_aabb = ctx.part_world_aabb(base)
    spindle_world_aabb = ctx.part_world_aabb(spindle)
    if base_aabb and spindle_world_aabb:
        front_protrusion = spindle_world_aabb[1][0] - base_aabb[1][0]
        ctx.check(
            "output_flange_projects_beyond_supports",
            front_protrusion >= 0.012,
            f"front_protrusion={front_protrusion:.4f} m",
        )

    with ctx.pose({roll: 1.2}):
        lug_aabb = ctx.part_element_world_aabb(spindle, elem="index_lug")
        if lug_aabb:
            lug_center = _aabb_center(lug_aabb)
            ctx.check(
                "positive_roll_rotates_index_lug_around_x_axis",
                lug_center[1] < -0.024 and lug_center[2] > AXIS_Z + 0.008,
                f"lug_center={lug_center}",
            )

    with ctx.pose({roll: -1.2}):
        lug_aabb = ctx.part_element_world_aabb(spindle, elem="index_lug")
        if lug_aabb:
            lug_center = _aabb_center(lug_aabb)
            ctx.check(
                "negative_roll_rotates_index_lug_opposite_direction",
                lug_center[1] > 0.024 and lug_center[2] > AXIS_Z + 0.008,
                f"lug_center={lug_center}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
