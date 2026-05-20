from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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


FRAME_WIDTH = 1.20
FRAME_DEPTH = 0.20
FRAME_HEIGHT = 0.16
FRAME_Z = 0.16

BRACKET_X = 0.12
BRACKET_SIZE = (0.05, 0.18, 0.24)
BRACKET_Z = -0.04

PIVOT_SLEEVE_RADIUS = 0.06
PIVOT_SLEEVE_LENGTH = 0.19

BALANCE_WEB_SIZE = (0.12, 1.38, 0.14)
BALANCE_WEB_Z = -0.24
BALANCE_NECK_SIZE = (0.12, 0.22, 0.17)
BALANCE_NECK_Z = -0.085
SADDLE_SIZE = (0.18, 0.16, 0.07)
SADDLE_Z = -0.275

AXLE_RADIUS = 0.07
AXLE_LENGTH = 1.76
AXLE_Y = 0.67
AXLE_Z = -0.38
SPINDLE_RADIUS = 0.04
SPINDLE_LENGTH = 0.08
SPINDLE_CENTER_X = 0.92

HUB_DRUM_RADIUS = 0.12
HUB_DRUM_LENGTH = 0.14
HUB_FLANGE_RADIUS = 0.18
HUB_FLANGE_LENGTH = 0.03
HUB_CAP_RADIUS = 0.06
HUB_CAP_LENGTH = 0.05


def _add_hub_visuals(part, *, outboard_sign: float, material: str) -> None:
    drum_center = outboard_sign * (HUB_DRUM_LENGTH * 0.5)
    flange_center = outboard_sign * (HUB_DRUM_LENGTH + HUB_FLANGE_LENGTH) * 0.5
    cap_center = outboard_sign * (
        HUB_DRUM_LENGTH + HUB_FLANGE_LENGTH + HUB_CAP_LENGTH
    ) * 0.5

    part.visual(
        Cylinder(radius=HUB_DRUM_RADIUS, length=HUB_DRUM_LENGTH),
        origin=Origin(xyz=(drum_center, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=material,
        name="hub_drum",
    )
    part.visual(
        Cylinder(radius=HUB_FLANGE_RADIUS, length=HUB_FLANGE_LENGTH),
        origin=Origin(xyz=(flange_center, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=material,
        name="hub_flange",
    )
    part.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_LENGTH),
        origin=Origin(xyz=(cap_center, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=material,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tandem_bogie_rear_axle_pair")

    frame_mat = model.material("frame_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    beam_mat = model.material("balance_beam_black", rgba=(0.10, 0.10, 0.11, 1.0))
    axle_mat = model.material("axle_steel", rgba=(0.17, 0.18, 0.19, 1.0))
    hub_mat = model.material("hub_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    frame = model.part("frame_crossmember")
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_Z)),
        material=frame_mat,
        name="crossmember",
    )
    for side, x_pos in (("left", -BRACKET_X), ("right", BRACKET_X)):
        frame.visual(
            Box(BRACKET_SIZE),
            origin=Origin(xyz=(x_pos, 0.0, BRACKET_Z)),
            material=frame_mat,
            name=f"{side}_hanger",
        )

    balance = model.part("center_balance_beam")
    balance.visual(
        Cylinder(radius=PIVOT_SLEEVE_RADIUS, length=PIVOT_SLEEVE_LENGTH),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=beam_mat,
        name="pivot_sleeve",
    )
    balance.visual(
        Box(BALANCE_NECK_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BALANCE_NECK_Z)),
        material=beam_mat,
        name="center_neck",
    )
    balance.visual(
        Box(BALANCE_WEB_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BALANCE_WEB_Z)),
        material=beam_mat,
        name="balance_web",
    )
    for name, y_pos in (("front", AXLE_Y), ("rear", -AXLE_Y)):
        balance.visual(
            Box(SADDLE_SIZE),
            origin=Origin(xyz=(0.0, y_pos, SADDLE_Z)),
            material=beam_mat,
            name=f"{name}_saddle",
        )

    def make_axle(name: str, y_pos: float):
        axle = model.part(name)
        axle.visual(
            Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
            material=axle_mat,
            name="axle_tube",
        )
        axle.visual(
            Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_LENGTH),
            origin=Origin(
                xyz=(SPINDLE_CENTER_X, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)
            ),
            material=axle_mat,
            name="right_spindle",
        )
        axle.visual(
            Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_LENGTH),
            origin=Origin(
                xyz=(-SPINDLE_CENTER_X, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)
            ),
            material=axle_mat,
            name="left_spindle",
        )
        model.articulation(
            f"balance_to_{name}",
            ArticulationType.FIXED,
            parent=balance,
            child=axle,
            origin=Origin(xyz=(0.0, y_pos, AXLE_Z)),
        )
        return axle

    front_axle = make_axle("front_axle_beam", AXLE_Y)
    rear_axle = make_axle("rear_axle_beam", -AXLE_Y)

    for axle_name, axle_part, y_pos in (
        ("front", front_axle, AXLE_Y),
        ("rear", rear_axle, -AXLE_Y),
    ):
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            hub = model.part(f"{axle_name}_{side_name}_hub")
            _add_hub_visuals(hub, outboard_sign=side_sign, material=hub_mat.name)
            model.articulation(
                f"{axle_name}_{side_name}_hub_spin",
                ArticulationType.CONTINUOUS,
                parent=axle_part,
                child=hub,
                origin=Origin(xyz=(side_sign * (SPINDLE_CENTER_X + SPINDLE_LENGTH * 0.5), 0.0, 0.0)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=25.0, velocity=40.0),
            )

    model.articulation(
        "frame_to_balance_beam",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=balance,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.2,
            lower=-0.38,
            upper=0.38,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    required_parts = (
        "frame_crossmember",
        "center_balance_beam",
        "front_axle_beam",
        "rear_axle_beam",
        "front_left_hub",
        "front_right_hub",
        "rear_left_hub",
        "rear_right_hub",
    )
    required_joints = (
        "frame_to_balance_beam",
        "balance_to_front_axle_beam",
        "balance_to_rear_axle_beam",
        "front_left_hub_spin",
        "front_right_hub_spin",
        "rear_left_hub_spin",
        "rear_right_hub_spin",
    )

    resolved_parts = {}
    missing_parts = []
    for name in required_parts:
        try:
            resolved_parts[name] = object_model.get_part(name)
        except Exception:
            missing_parts.append(name)
    ctx.check(
        "required_parts_present",
        not missing_parts,
        details=f"Missing parts: {missing_parts}",
    )

    resolved_joints = {}
    missing_joints = []
    for name in required_joints:
        try:
            resolved_joints[name] = object_model.get_articulation(name)
        except Exception:
            missing_joints.append(name)
    ctx.check(
        "required_joints_present",
        not missing_joints,
        details=f"Missing joints: {missing_joints}",
    )

    if missing_parts or missing_joints:
        return ctx.report()

    frame = resolved_parts["frame_crossmember"]
    balance = resolved_parts["center_balance_beam"]
    front_axle = resolved_parts["front_axle_beam"]
    rear_axle = resolved_parts["rear_axle_beam"]
    front_left_hub = resolved_parts["front_left_hub"]
    front_right_hub = resolved_parts["front_right_hub"]
    rear_left_hub = resolved_parts["rear_left_hub"]
    rear_right_hub = resolved_parts["rear_right_hub"]

    pivot = resolved_joints["frame_to_balance_beam"]
    hub_spins = [resolved_joints[name] for name in required_joints[3:]]

    ctx.check(
        "balance_pivot_axis_is_x",
        all(
            isclose(a, b, abs_tol=1e-9)
            for a, b in zip(pivot.axis, (1.0, 0.0, 0.0))
        ),
        details=f"Expected frame_to_balance_beam axis (1, 0, 0), got {pivot.axis}",
    )
    ctx.check(
        "hub_axes_are_x",
        all(
            all(isclose(a, b, abs_tol=1e-9) for a, b in zip(joint.axis, (1.0, 0.0, 0.0)))
            for joint in hub_spins
        ),
        details="One or more hub spin joints are not aligned to the axle beam axis.",
    )

    ctx.expect_contact(balance, frame, name="balance_beam_supported_by_frame")
    ctx.expect_contact(front_axle, balance, name="front_axle_mounted_to_balance_beam")
    ctx.expect_contact(rear_axle, balance, name="rear_axle_mounted_to_balance_beam")

    ctx.expect_contact(front_left_hub, front_axle, name="front_left_hub_seated_on_spindle")
    ctx.expect_contact(front_right_hub, front_axle, name="front_right_hub_seated_on_spindle")
    ctx.expect_contact(rear_left_hub, rear_axle, name="rear_left_hub_seated_on_spindle")
    ctx.expect_contact(rear_right_hub, rear_axle, name="rear_right_hub_seated_on_spindle")

    ctx.expect_origin_gap(
        front_axle,
        rear_axle,
        axis="y",
        min_gap=1.30,
        max_gap=1.40,
        name="axle_spacing_matches_tandem_bogie_layout",
    )
    ctx.expect_origin_distance(
        front_left_hub,
        front_right_hub,
        axes="x",
        min_dist=1.90,
        max_dist=1.94,
        name="front_track_width_is_realistic",
    )
    ctx.expect_origin_distance(
        rear_left_hub,
        rear_right_hub,
        axes="x",
        min_dist=1.90,
        max_dist=1.94,
        name="rear_track_width_is_realistic",
    )

    with ctx.pose({pivot: 0.30}):
        ctx.expect_contact(
            balance,
            frame,
            name="pivot_sleeve_stays_captured_in_positive_rock",
        )
        ctx.expect_contact(
            front_left_hub,
            front_axle,
            name="front_left_hub_remains_mounted_when_rocked",
        )
        ctx.expect_contact(
            rear_right_hub,
            rear_axle,
            name="rear_right_hub_remains_mounted_when_rocked",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
