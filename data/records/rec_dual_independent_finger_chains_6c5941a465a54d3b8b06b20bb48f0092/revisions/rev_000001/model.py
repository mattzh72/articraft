from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


SUPPORT_W = 0.18
SUPPORT_D = 0.07
SUPPORT_T = 0.012

PALM_NECK_W = 0.036
PALM_NECK_D = 0.024
PALM_NECK_H = 0.024
PALM_W = 0.112
PALM_D = 0.042
PALM_H = 0.028

BARREL_R = 0.006
BARREL_LEN = 0.008
CLEVIS_GAP = 0.012
CLEVIS_PLATE_T = 0.0025
CLEVIS_Y = CLEVIS_GAP / 2.0 + CLEVIS_PLATE_T / 2.0
SUPPORT_HANGER_H = 0.024
BRIDGE_H = 0.006

LEFT_KNUCKLE = (-0.042, 0.0, -(PALM_NECK_H + PALM_H + 0.012))
RIGHT_KNUCKLE = (0.042, 0.0, -(PALM_NECK_H + PALM_H + 0.012))

LEFT_PROX_LEN = 0.072
LEFT_MID_LEN = 0.058
LEFT_DIST_LEN = 0.046

RIGHT_PROX_LEN = 0.062
RIGHT_MID_LEN = 0.050
RIGHT_DIST_LEN = 0.052


def _cyl_y(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _extrude_xz_profile(points: list[tuple[float, float]], thickness: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness / 2.0, both=True)


def make_support_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(SUPPORT_W, SUPPORT_D, SUPPORT_T)
    left_rail = cq.Workplane("XY").box(0.018, 0.024, 0.030).translate((-0.066, 0.0, -0.015))
    right_rail = cq.Workplane("XY").box(0.018, 0.024, 0.030).translate((0.066, 0.0, -0.015))
    rear_rib = cq.Workplane("XY").box(0.138, 0.012, 0.010).translate((0.0, -0.024, -0.007))
    mount_pad = cq.Workplane("XY").box(0.058, 0.040, 0.006).translate((0.0, 0.0, 0.009))
    hanger = cq.Workplane("XY").box(0.028, 0.020, SUPPORT_HANGER_H).translate(
        (0.0, 0.0, -(SUPPORT_T / 2.0 + SUPPORT_HANGER_H / 2.0))
    )
    return plate.union(left_rail).union(right_rail).union(rear_rib).union(mount_pad).union(hanger)


def make_palm_shape() -> cq.Workplane:
    neck = cq.Workplane("XY").box(PALM_NECK_W, PALM_NECK_D, PALM_NECK_H).translate(
        (0.0, 0.0, -PALM_NECK_H / 2.0)
    )
    palm = cq.Workplane("XY").box(PALM_W, PALM_D, PALM_H).translate(
        (0.0, 0.0, -(PALM_NECK_H + PALM_H / 2.0))
    )
    shoulder_l = cq.Workplane("XY").box(0.030, PALM_D, 0.012).translate((-0.020, 0.0, -0.028))
    shoulder_r = cq.Workplane("XY").box(0.030, PALM_D, 0.012).translate((0.020, 0.0, -0.028))
    lower_bridge = cq.Workplane("XY").box(0.094, 0.018, 0.010).translate((0.0, 0.0, -0.049))
    left_knuckle_pad = cq.Workplane("XY").box(0.020, 0.016, 0.020).translate(
        (LEFT_KNUCKLE[0], 0.0, LEFT_KNUCKLE[2] + 0.010)
    )
    right_knuckle_pad = cq.Workplane("XY").box(0.020, 0.016, 0.020).translate(
        (RIGHT_KNUCKLE[0], 0.0, RIGHT_KNUCKLE[2] + 0.010)
    )
    return neck.union(palm).union(shoulder_l).union(shoulder_r).union(lower_bridge).union(left_knuckle_pad).union(
        right_knuckle_pad
    )


def make_link_with_clevis(length: float, body_w: float, body_t: float) -> cq.Workplane:
    top_w = body_w * 1.30
    waist_w = body_w * 0.76
    bottom_w = body_w * 1.12
    profile = [
        (top_w / 2.0, 0.0),
        (top_w / 2.0, -0.010),
        (waist_w / 2.0, -0.024),
        (waist_w / 2.0, -(length - 0.018)),
        (bottom_w / 2.0, -(length - 0.010)),
        (bottom_w / 2.0, -length),
        (-bottom_w / 2.0, -length),
        (-bottom_w / 2.0, -(length - 0.010)),
        (-waist_w / 2.0, -(length - 0.018)),
        (-waist_w / 2.0, -0.024),
        (-top_w / 2.0, -0.010),
        (-top_w / 2.0, 0.0),
    ]
    plate = _extrude_xz_profile(profile, body_t)
    upper_boss = cq.Workplane("XY").box(body_w * 1.05, body_t * 1.06, 0.016).translate((0.0, 0.0, -0.010))
    lower_boss = cq.Workplane("XY").box(body_w * 1.02, body_t * 1.08, 0.016).translate(
        (0.0, 0.0, -(length - 0.008))
    )
    return plate.union(upper_boss).union(lower_boss)


def make_terminal_link(
    length: float,
    body_w: float,
    body_t: float,
    *,
    pad_w: float,
    pad_len: float,
    pad_x: float,
    toe_len: float = 0.0,
    toe_drop: float = 0.0,
    toe_x: float = 0.0,
) -> cq.Workplane:
    top_w = body_w * 1.24
    waist_w = body_w * 0.74
    tip_w = max(body_w * 1.05, pad_w * 0.82)
    profile = [
        (top_w / 2.0, 0.0),
        (top_w / 2.0, -0.010),
        (waist_w / 2.0, -0.022),
        (waist_w / 2.0, -(length - 0.018)),
        (tip_w / 2.0, -(length - 0.006)),
        (tip_w / 2.0, -(length + 0.006)),
        (-tip_w / 2.0, -(length + 0.006)),
        (-tip_w / 2.0, -(length - 0.006)),
        (-waist_w / 2.0, -(length - 0.018)),
        (-waist_w / 2.0, -0.022),
        (-top_w / 2.0, -0.010),
        (-top_w / 2.0, 0.0),
    ]
    plate = _extrude_xz_profile(profile, body_t)
    upper_boss = cq.Workplane("XY").box(body_w * 1.02, body_t * 1.06, 0.016).translate((0.0, 0.0, -0.010))
    pad = cq.Workplane("XY").box(pad_w, body_t * 1.10, pad_len).translate(
        (pad_x, 0.0, -(length - 0.002))
    )
    shape = plate.union(upper_boss).union(pad)
    if toe_len > 0.0 and toe_drop > 0.0:
        toe = cq.Workplane("XY").box(pad_w * 0.58, body_t * 1.02, toe_len).translate(
            (pad_x + toe_x, 0.0, -(length + toe_drop - toe_len / 2.0))
        )
        shape = shape.union(toe)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_dual_finger_rig")

    support_m = model.material("support_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    palm_m = model.material("palm_alloy", rgba=(0.45, 0.47, 0.50, 1.0))
    link_m = model.material("link_gunmetal", rgba=(0.23, 0.25, 0.28, 1.0))
    pad_m = model.material("pad_rubber", rgba=(0.12, 0.13, 0.14, 1.0))

    support = model.part("top_support")
    support.visual(
        mesh_from_cadquery(make_support_shape(), "top_support"),
        material=support_m,
        name="support_frame",
    )

    palm = model.part("palm_block")
    palm.visual(
        mesh_from_cadquery(make_palm_shape(), "palm_block"),
        material=palm_m,
        name="palm_body",
    )

    left_prox = model.part("left_proximal")
    left_prox.visual(
        mesh_from_cadquery(make_link_with_clevis(LEFT_PROX_LEN, 0.016, 0.010), "left_proximal"),
        material=link_m,
        name="left_proximal_link",
    )

    left_mid = model.part("left_middle")
    left_mid.visual(
        mesh_from_cadquery(make_link_with_clevis(LEFT_MID_LEN, 0.014, 0.009), "left_middle"),
        material=link_m,
        name="left_middle_link",
    )

    left_dist = model.part("left_distal")
    left_dist.visual(
        mesh_from_cadquery(
            make_terminal_link(
                LEFT_DIST_LEN,
                0.013,
                0.009,
                pad_w=0.016,
                pad_len=0.018,
                pad_x=0.004,
            ),
            "left_distal",
        ),
        material=pad_m,
        name="left_distal_link",
    )

    right_prox = model.part("right_proximal")
    right_prox.visual(
        mesh_from_cadquery(make_link_with_clevis(RIGHT_PROX_LEN, 0.018, 0.010), "right_proximal"),
        material=link_m,
        name="right_proximal_link",
    )

    right_mid = model.part("right_middle")
    right_mid.visual(
        mesh_from_cadquery(make_link_with_clevis(RIGHT_MID_LEN, 0.016, 0.009), "right_middle"),
        material=link_m,
        name="right_middle_link",
    )

    right_dist = model.part("right_distal")
    right_dist.visual(
        mesh_from_cadquery(
            make_terminal_link(
                RIGHT_DIST_LEN,
                0.015,
                0.009,
                pad_w=0.021,
                pad_len=0.020,
                pad_x=-0.006,
                toe_len=0.016,
                toe_drop=0.008,
                toe_x=-0.003,
            ),
            "right_distal",
        ),
        material=pad_m,
        name="right_distal_link",
    )

    model.articulation(
        "support_to_palm",
        ArticulationType.FIXED,
        parent=support,
        child=palm,
        origin=Origin(xyz=(0.0, 0.0, -(SUPPORT_T / 2.0 + SUPPORT_HANGER_H))),
    )

    model.articulation(
        "left_knuckle",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_prox,
        origin=Origin(xyz=LEFT_KNUCKLE),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.15, upper=1.15),
    )
    model.articulation(
        "left_middle_joint",
        ArticulationType.REVOLUTE,
        parent=left_prox,
        child=left_mid,
        origin=Origin(xyz=(0.0, 0.0, -LEFT_PROX_LEN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.7, lower=0.0, upper=1.25),
    )
    model.articulation(
        "left_distal_joint",
        ArticulationType.REVOLUTE,
        parent=left_mid,
        child=left_dist,
        origin=Origin(xyz=(0.0, 0.0, -LEFT_MID_LEN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.20),
    )

    model.articulation(
        "right_knuckle",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_prox,
        origin=Origin(xyz=RIGHT_KNUCKLE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.15, upper=1.15),
    )
    model.articulation(
        "right_middle_joint",
        ArticulationType.REVOLUTE,
        parent=right_prox,
        child=right_mid,
        origin=Origin(xyz=(0.0, 0.0, -RIGHT_PROX_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.7, lower=0.0, upper=1.25),
    )
    model.articulation(
        "right_distal_joint",
        ArticulationType.REVOLUTE,
        parent=right_mid,
        child=right_dist,
        origin=Origin(xyz=(0.0, 0.0, -RIGHT_MID_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    palm = object_model.get_part("palm_block")
    left_prox = object_model.get_part("left_proximal")
    left_mid = object_model.get_part("left_middle")
    left_dist = object_model.get_part("left_distal")
    right_prox = object_model.get_part("right_proximal")
    right_mid = object_model.get_part("right_middle")
    right_dist = object_model.get_part("right_distal")

    left_knuckle = object_model.get_articulation("left_knuckle")
    right_knuckle = object_model.get_articulation("right_knuckle")

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

    ctx.expect_contact(palm, support, contact_tol=0.001, name="palm_is_carried_by_support")
    ctx.expect_contact(left_prox, palm, contact_tol=0.001, name="left_knuckle_is_supported")
    ctx.expect_contact(left_mid, left_prox, contact_tol=0.001, name="left_middle_joint_is_supported")
    ctx.expect_contact(left_dist, left_mid, contact_tol=0.001, name="left_distal_joint_is_supported")
    ctx.expect_contact(right_prox, palm, contact_tol=0.001, name="right_knuckle_is_supported")
    ctx.expect_contact(right_mid, right_prox, contact_tol=0.001, name="right_middle_joint_is_supported")
    ctx.expect_contact(right_dist, right_mid, contact_tol=0.001, name="right_distal_joint_is_supported")

    rest_left_mid_x = ctx.part_world_position(left_mid)[0]
    rest_right_mid_x = ctx.part_world_position(right_mid)[0]

    with ctx.pose({left_knuckle: 0.55}):
        moved_left_mid_x = ctx.part_world_position(left_mid)[0]
        moved_right_mid_x = ctx.part_world_position(right_mid)[0]
        ctx.check(
            "left_chain_curls_inward",
            moved_left_mid_x > rest_left_mid_x + 0.01,
            f"expected left middle x to move inward from {rest_left_mid_x:.4f}, got {moved_left_mid_x:.4f}",
        )
        ctx.check(
            "right_chain_stays_independent_when_left_moves",
            abs(moved_right_mid_x - rest_right_mid_x) < 1e-6,
            f"expected right chain to stay fixed when left knuckle moves, got Δx={moved_right_mid_x - rest_right_mid_x:.6f}",
        )

    with ctx.pose({right_knuckle: 0.55}):
        moved_left_mid_x = ctx.part_world_position(left_mid)[0]
        moved_right_mid_x = ctx.part_world_position(right_mid)[0]
        ctx.check(
            "right_chain_curls_inward",
            moved_right_mid_x < rest_right_mid_x - 0.01,
            f"expected right middle x to move inward from {rest_right_mid_x:.4f}, got {moved_right_mid_x:.4f}",
        )
        ctx.check(
            "left_chain_stays_independent_when_right_moves",
            abs(moved_left_mid_x - rest_left_mid_x) < 1e-6,
            f"expected left chain to stay fixed when right knuckle moves, got Δx={moved_left_mid_x - rest_left_mid_x:.6f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
