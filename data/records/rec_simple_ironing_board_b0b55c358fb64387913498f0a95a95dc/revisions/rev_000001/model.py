from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


DECK_PROFILE = [
    (-0.680, -0.160),
    (-0.705, -0.105),
    (-0.705, 0.105),
    (-0.680, 0.160),
    (-0.300, 0.190),
    (0.080, 0.190),
    (0.320, 0.150),
    (0.525, 0.090),
    (0.640, 0.036),
    (0.690, 0.000),
    (0.640, -0.036),
    (0.525, -0.090),
    (0.320, -0.150),
    (0.080, -0.190),
    (-0.300, -0.190),
]

HINGE_Z = -0.014
LEG_DROP = 0.840
FRONT_HINGE_X = 0.200
REAR_HINGE_X = -0.200
FRONT_FOOT_DX = -0.440
REAR_FOOT_DX = 0.440


def _scaled_profile(scale: float) -> list[tuple[float, float]]:
    return [(x * scale, y * scale) for x, y in DECK_PROFILE]


def _add_leg_assembly(
    part,
    *,
    foot_dx: float,
    rail_y: float,
    rail_thickness_y: float,
    pivot_y: float,
    pivot_boss_length: float,
    hinge_pad_x: float,
    cross_x: float,
    pivot_block: bool = False,
    material: Material,
    accent: Material,
) -> None:
    leg_length = math.hypot(abs(foot_dx), LEG_DROP)
    rail_angle = math.atan2(abs(foot_dx), LEG_DROP)
    y_rotation = -math.copysign(rail_angle, foot_dx)
    rail_center_x = foot_dx * 0.5
    rail_center_z = -LEG_DROP * 0.5
    foot_bar_x = foot_dx
    cross_z = -LEG_DROP * (abs(cross_x) / abs(foot_dx))

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        y = rail_y * side_sign
        part.visual(
            Box((0.028, rail_thickness_y, leg_length)),
            origin=Origin(xyz=(rail_center_x, y, rail_center_z), rpy=(0.0, y_rotation, 0.0)),
            material=material,
            name=f"{side_name}_rail",
        )
        part.visual(
            Box((0.040, 0.030, 0.060)),
            origin=Origin(xyz=(0.000, y, -0.030)),
            material=material,
            name=f"{side_name}_top_collar",
        )
        part.visual(
            Box((0.050, 0.032, 0.060)),
            origin=Origin(xyz=(foot_bar_x, y, -LEG_DROP + 0.030)),
            material=material,
            name=f"{side_name}_foot_collar",
        )
        part.visual(
            Cylinder(radius=0.020, length=pivot_boss_length),
            origin=Origin(
                xyz=(cross_x, pivot_y * side_sign, cross_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=accent,
            name=f"{side_name}_pivot_boss",
        )

    part.visual(
        Box((0.040, 0.300, 0.020)),
        origin=Origin(xyz=(hinge_pad_x, 0.000, -0.045)),
        material=material,
        name="top_bridge",
    )
    part.visual(
        Box((0.032, 0.060, 0.030)),
        origin=Origin(xyz=(hinge_pad_x, 0.000, -0.020)),
        material=material,
        name="hinge_hanger",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.070),
        origin=Origin(
            xyz=(hinge_pad_x, 0.000, 0.000),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=accent,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.070, 0.380, 0.028)),
        origin=Origin(xyz=(foot_bar_x, 0.000, -LEG_DROP)),
        material=material,
        name="foot_bar",
    )
    part.visual(
        Box((0.060, 0.420, 0.010)),
        origin=Origin(xyz=(foot_bar_x, 0.000, -LEG_DROP - 0.019)),
        material=accent,
        name="foot_pad_strip",
    )

    if pivot_block:
        part.visual(
            Box((0.044, 0.024, 0.040)),
            origin=Origin(xyz=(0.265, 0.067, -0.505)),
            material=accent,
            name="lock_pivot_block",
        )
        part.visual(
            Box((0.060, 0.020, 0.034)),
            origin=Origin(xyz=(0.140, 0.070, -0.365)),
            material=accent,
            name="lock_index_tab",
        )
        part.visual(
            Box((0.018, 0.034, 0.520)),
            origin=Origin(xyz=(0.092, 0.072, -0.276)),
            material=material,
            name="lock_side_plate",
        )
        part.visual(
            Box((0.120, 0.034, 0.030)),
            origin=Origin(xyz=(0.086, 0.072, -0.060)),
            material=material,
            name="lock_top_link",
        )
        part.visual(
            Box((0.050, 0.026, 0.030)),
            origin=Origin(xyz=(0.076, 0.068, -0.505)),
            material=material,
            name="lock_pivot_bridge",
        )
        part.visual(
            Box((0.174, 0.028, 0.030)),
            origin=Origin(xyz=(0.179, 0.081, -0.505)),
            material=material,
            name="lock_mid_beam",
        )
        part.visual(
            Box((0.060, 0.026, 0.030)),
            origin=Origin(xyz=(0.114, 0.068, -0.365)),
            material=material,
            name="lock_index_bridge",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_ironing_board")

    deck_steel = model.material("deck_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    deck_pad = model.material("deck_pad", rgba=(0.58, 0.66, 0.78, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.18, 0.20, 0.23, 1.0))
    alignment_orange = model.material("alignment_orange", rgba=(0.94, 0.50, 0.12, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.08, 0.08, 0.09, 1.0))

    deck = model.part("deck")
    deck_shell_geom = ExtrudeGeometry.from_z0(DECK_PROFILE, 0.018, cap=True, closed=True)
    deck_pad_geom = ExtrudeGeometry.from_z0(_scaled_profile(0.985), 0.006, cap=True, closed=True)
    deck.visual(
        mesh_from_geometry(deck_shell_geom, "deck_shell"),
        material=deck_steel,
        name="deck_shell",
    )
    deck.visual(
        mesh_from_geometry(deck_pad_geom, "deck_pad"),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=deck_pad,
        name="deck_pad",
    )
    deck.visual(
        Box((0.920, 0.024, 0.030)),
        origin=Origin(xyz=(-0.060, 0.115, -0.015)),
        material=dark_frame,
        name="right_datum_rail",
    )
    deck.visual(
        Box((0.920, 0.024, 0.030)),
        origin=Origin(xyz=(-0.060, -0.115, -0.015)),
        material=dark_frame,
        name="left_datum_rail",
    )
    deck.visual(
        Box((0.180, 0.230, 0.028)),
        origin=Origin(xyz=(-0.540, 0.000, -0.014)),
        material=dark_frame,
        name="tail_reference_block",
    )
    for hinge_prefix, hinge_x in (("front_hinge", 0.214), ("rear_hinge", -0.214)):
        for side_name, side_y in (("left", -0.060), ("right", 0.060)):
            deck.visual(
                Box((0.024, 0.050, 0.028)),
                origin=Origin(xyz=(hinge_x, side_y, HINGE_Z)),
                material=dark_frame,
                name=f"{hinge_prefix}_{side_name}_lug",
            )
            deck.visual(
                Cylinder(radius=0.014, length=0.050),
                origin=Origin(
                    xyz=(hinge_x, side_y, HINGE_Z),
                    rpy=(math.pi * 0.5, 0.0, 0.0),
                ),
                material=polymer_black,
                name=f"{hinge_prefix}_{side_name}_barrel",
            )
    deck.visual(
        Box((0.150, 0.080, 0.008)),
        origin=Origin(xyz=(0.035, 0.055, -0.004)),
        material=dark_frame,
        name="lock_rack_plate",
    )
    for step_name, step_x in (
        ("lock_step_low", 0.000),
        ("lock_step_mid", 0.035),
        ("lock_step_high", 0.070),
    ):
        deck.visual(
            Box((0.016, 0.060, 0.028)),
            origin=Origin(xyz=(step_x, 0.055, -0.022)),
            material=alignment_orange,
            name=step_name,
        )
    deck.visual(
        Box((0.660, 0.008, 0.0015)),
        origin=Origin(xyz=(-0.090, 0.102, 0.02475)),
        material=alignment_orange,
        name="right_long_index",
    )
    deck.visual(
        Box((0.660, 0.008, 0.0015)),
        origin=Origin(xyz=(-0.090, -0.102, 0.02475)),
        material=alignment_orange,
        name="left_long_index",
    )
    for mark_name, mark_x in (
        ("index_mark_low", 0.000),
        ("index_mark_mid", 0.035),
        ("index_mark_high", 0.070),
    ):
        deck.visual(
            Box((0.008, 0.150, 0.0015)),
            origin=Origin(xyz=(mark_x, 0.000, 0.02475)),
            material=alignment_orange,
            name=mark_name,
        )

    front_leg = model.part("front_leg")
    _add_leg_assembly(
        front_leg,
        foot_dx=FRONT_FOOT_DX,
        rail_y=0.143,
        rail_thickness_y=0.012,
        pivot_y=0.143,
        pivot_boss_length=0.012,
        hinge_pad_x=-0.014,
        cross_x=-0.200,
        material=dark_frame,
        accent=polymer_black,
    )

    rear_leg = model.part("rear_leg")
    _add_leg_assembly(
        rear_leg,
        foot_dx=REAR_FOOT_DX,
        rail_y=0.157,
        rail_thickness_y=0.010,
        pivot_y=0.157,
        pivot_boss_length=0.010,
        hinge_pad_x=0.014,
        cross_x=0.200,
        pivot_block=True,
        material=dark_frame,
        accent=polymer_black,
    )

    lock_strut = model.part("lock_strut")
    lock_dx = -0.030
    lock_dz = 0.469
    lock_length = math.hypot(abs(lock_dx), lock_dz)
    lock_angle = math.atan2(abs(lock_dx), lock_dz)
    lock_rotation = math.copysign(lock_angle, lock_dx)
    lock_strut.visual(
        Box((0.020, 0.014, lock_length)),
        origin=Origin(
            xyz=(lock_dx * 0.5, -0.012, lock_dz * 0.5),
            rpy=(0.0, lock_rotation, 0.0),
        ),
        material=dark_frame,
        name="brace_bar",
    )
    lock_strut.visual(
        Box((0.024, 0.024, 0.028)),
        origin=Origin(xyz=(0.000, -0.012, 0.000)),
        material=polymer_black,
        name="pivot_pad",
    )
    lock_strut.visual(
        Box((0.026, 0.024, 0.012)),
        origin=Origin(xyz=(lock_dx, -0.012, lock_dz + 0.008)),
        material=alignment_orange,
        name="latch_pad",
    )
    lock_strut.visual(
        Box((0.038, 0.026, 0.042)),
        origin=Origin(xyz=(lock_dx * 0.80, -0.020, lock_dz - 0.004)),
        material=dark_frame,
        name="latch_head",
    )
    lock_strut.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.000, -0.012, 0.000), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=polymer_black,
        name="pivot_bushing",
    )

    front_hinge = model.articulation(
        "deck_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_leg,
        origin=Origin(xyz=(FRONT_HINGE_X, 0.000, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=-1.05,
            upper=0.0,
        ),
    )
    rear_hinge = model.articulation(
        "deck_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_leg,
        origin=Origin(xyz=(REAR_HINGE_X, 0.000, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=1.05,
        ),
    )
    lock_joint = model.articulation(
        "rear_leg_to_lock_strut",
        ArticulationType.REVOLUTE,
        parent=rear_leg,
        child=lock_strut,
        origin=Origin(xyz=(0.265, 0.055, -0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    model.meta["open_pose"] = {
        front_hinge.name: 0.0,
        rear_hinge.name: 0.0,
        lock_joint.name: 0.0,
    }
    return model
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_leg = object_model.get_part("front_leg")
    rear_leg = object_model.get_part("rear_leg")
    lock_strut = object_model.get_part("lock_strut")
    front_hinge = object_model.get_articulation("deck_to_front_leg")
    rear_hinge = object_model.get_articulation("deck_to_rear_leg")
    lock_joint = object_model.get_articulation("rear_leg_to_lock_strut")

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

    ctx.expect_gap(
        deck,
        front_leg,
        axis="y",
        positive_elem="front_hinge_right_barrel",
        negative_elem="hinge_barrel",
        max_gap=0.001,
        max_penetration=0.000001,
        name="front_leg_hinge_mounts_to_deck",
    )
    ctx.expect_gap(
        deck,
        rear_leg,
        axis="y",
        positive_elem="rear_hinge_right_barrel",
        negative_elem="hinge_barrel",
        max_gap=0.001,
        max_penetration=0.000001,
        name="rear_leg_hinge_mounts_to_deck",
    )
    ctx.expect_contact(rear_leg, lock_strut, elem_a="lock_pivot_block", elem_b="pivot_pad", name="lock_strut_mounts_to_rear_leg")

    with ctx.pose({front_hinge: 0.0, rear_hinge: 0.0, lock_joint: 0.0}):
        ctx.expect_contact(
            lock_strut,
            deck,
            elem_a="latch_pad",
            elem_b="lock_step_mid",
            name="open_pose_lock_latch_contacts_mid_step",
        )
        ctx.expect_gap(
            rear_leg,
            front_leg,
            axis="y",
            positive_elem="right_pivot_boss",
            negative_elem="right_pivot_boss",
            min_gap=0.001,
            max_gap=0.004,
            name="controlled_scissor_clearance_at_right_pivot",
        )
        ctx.expect_overlap(
            front_leg,
            rear_leg,
            axes="xz",
            elem_a="right_rail",
            elem_b="right_rail",
            min_overlap=0.080,
            name="right_side_rails_read_as_scissor",
        )
        ctx.expect_gap(
            deck,
            front_leg,
            axis="z",
            positive_elem="deck_shell",
            negative_elem="foot_bar",
            min_gap=0.800,
            max_gap=0.860,
            name="front_foot_bar_sits_at_open_height",
        )
        ctx.expect_gap(
            deck,
            rear_leg,
            axis="z",
            positive_elem="deck_shell",
            negative_elem="foot_bar",
            min_gap=0.800,
            max_gap=0.860,
            name="rear_foot_bar_sits_at_open_height",
        )

    front_limits = front_hinge.motion_limits
    rear_limits = rear_hinge.motion_limits
    lock_limits = lock_joint.motion_limits
    ctx.check(
        "front_leg_joint_limit_represents_fold_under_motion",
        front_hinge.axis == (0.0, 1.0, 0.0)
        and front_limits is not None
        and front_limits.lower is not None
        and front_limits.upper == 0.0
        and front_limits.lower <= -1.0,
        details="front leg should use an explicit hinge axis and a near-flat folded lower limit",
    )
    ctx.check(
        "rear_leg_joint_limit_represents_fold_under_motion",
        rear_hinge.axis == (0.0, 1.0, 0.0)
        and rear_limits is not None
        and rear_limits.lower == 0.0
        and rear_limits.upper is not None
        and rear_limits.upper >= 1.0,
        details="rear leg should mirror the folding range from the opposite hinge side",
    )
    ctx.check(
        "lock_strut_joint_is_explicitly_unlockable",
        lock_joint.axis == (0.0, -1.0, 0.0)
        and lock_limits is not None
        and lock_limits.lower == 0.0
        and lock_limits.upper is not None
        and lock_limits.upper >= 1.0,
        details="lock strut should rotate away from the rack with a clear unlocked range",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
