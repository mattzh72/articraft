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


FINGER_WIDTH = 0.028
CENTER_KNUCKLE_WIDTH = 0.010
JOINT_SIDE_CLEARANCE = 0.0
EAR_WIDTH = (FINGER_WIDTH - CENTER_KNUCKLE_WIDTH - 2.0 * JOINT_SIDE_CLEARANCE) / 2.0
HINGE_RADIUS = 0.008
PIN_RADIUS = 0.0065
BODY_HEIGHT = 0.015
BODY_Z_CENTER = -0.0055
DORSAL_RIDGE_HEIGHT = 0.006
DORSAL_RIDGE_Z_CENTER = 0.004
REAR_EAR_LENGTH = 0.012
FRONT_NECK_LENGTH = 0.013
TIP_RADIUS = 0.007

PROXIMAL_LENGTH = 0.046
MIDDLE_LENGTH = 0.034
DISTAL_LENGTH = 0.029

SPINE_RAIL_WIDTH = 0.014
SPINE_RAIL_HEIGHT = 0.008
SPINE_RAIL_Z_CENTER = 0.030


def _add_box_visual(
    part,
    *,
    name: str,
    x0: float,
    x1: float,
    width: float,
    height: float,
    z_center: float,
    material: str,
    y_center: float = 0.0,
) -> None:
    part.visual(
        Box((x1 - x0, width, height)),
        origin=Origin(xyz=((x0 + x1) / 2.0, y_center, z_center)),
        material=material,
        name=name,
    )


def _add_y_cylinder_visual(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    x: float,
    y: float,
    z: float,
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_phalanx_part(part, *, prefix: str, length: float, material: str, distal: bool = False) -> None:
    ear_center_y = CENTER_KNUCKLE_WIDTH / 2.0 + JOINT_SIDE_CLEARANCE + EAR_WIDTH / 2.0

    for side_name, side in (("left", -1.0), ("right", 1.0)):
        y_center = side * ear_center_y
        _add_y_cylinder_visual(
            part,
            name=f"{prefix}_{side_name}_rear_ear",
            radius=HINGE_RADIUS,
            length=EAR_WIDTH,
            x=0.0,
            y=y_center,
            z=0.0,
            material=material,
        )
        _add_box_visual(
            part,
            name=f"{prefix}_{side_name}_rear_cheek",
            x0=0.0,
            x1=REAR_EAR_LENGTH,
            width=EAR_WIDTH,
            height=BODY_HEIGHT,
            z_center=BODY_Z_CENTER,
            y_center=y_center,
            material=material,
        )

    if distal:
        tip_blend_start = length - 0.009
        tip_center_x = length - 0.005
        _add_box_visual(
            part,
            name=f"{prefix}_main_body",
            x0=REAR_EAR_LENGTH,
            x1=tip_blend_start,
            width=FINGER_WIDTH,
            height=BODY_HEIGHT,
            z_center=BODY_Z_CENTER,
            material=material,
        )
        _add_box_visual(
            part,
            name=f"{prefix}_dorsal_ridge",
            x0=REAR_EAR_LENGTH + 0.004,
            x1=tip_blend_start,
            width=FINGER_WIDTH - 0.006,
            height=DORSAL_RIDGE_HEIGHT,
            z_center=DORSAL_RIDGE_Z_CENTER,
            material=material,
        )
        _add_box_visual(
            part,
            name=f"{prefix}_tip_blend",
            x0=tip_blend_start,
            x1=tip_center_x,
            width=FINGER_WIDTH,
            height=BODY_HEIGHT * 0.92,
            z_center=BODY_Z_CENTER + 0.0008,
            material=material,
        )
        _add_y_cylinder_visual(
            part,
            name=f"{prefix}_tip_cap",
            radius=TIP_RADIUS,
            length=FINGER_WIDTH,
            x=tip_center_x,
            y=0.0,
            z=BODY_Z_CENTER,
            material=material,
        )
    else:
        neck_start = length - FRONT_NECK_LENGTH
        _add_box_visual(
            part,
            name=f"{prefix}_main_body",
            x0=REAR_EAR_LENGTH,
            x1=neck_start,
            width=FINGER_WIDTH,
            height=BODY_HEIGHT,
            z_center=BODY_Z_CENTER,
            material=material,
        )
        _add_box_visual(
            part,
            name=f"{prefix}_dorsal_ridge",
            x0=REAR_EAR_LENGTH + 0.004,
            x1=neck_start,
            width=FINGER_WIDTH - 0.006,
            height=DORSAL_RIDGE_HEIGHT,
            z_center=DORSAL_RIDGE_Z_CENTER,
            material=material,
        )
        _add_box_visual(
            part,
            name=f"{prefix}_front_neck",
            x0=neck_start,
            x1=length - PIN_RADIUS * 0.20,
            width=CENTER_KNUCKLE_WIDTH,
            height=BODY_HEIGHT,
            z_center=BODY_Z_CENTER,
            material=material,
        )
        _add_y_cylinder_visual(
            part,
            name=f"{prefix}_front_knuckle",
            radius=PIN_RADIUS,
            length=CENTER_KNUCKLE_WIDTH,
            x=length,
            y=0.0,
            z=0.0,
            material=material,
        )


def _build_rear_spine(part, *, material: str) -> None:
    rail_end = PROXIMAL_LENGTH + MIDDLE_LENGTH + DISTAL_LENGTH + 0.014
    rung_z_center = 0.017
    rung_height = 0.018

    _add_box_visual(
        part,
        name="spine_base",
        x0=-0.040,
        x1=-0.014,
        width=0.020,
        height=0.020,
        z_center=0.002,
        material=material,
    )
    _add_box_visual(
        part,
        name="spine_shoulder",
        x0=-0.022,
        x1=-0.006,
        width=0.016,
        height=0.014,
        z_center=0.015,
        material=material,
    )
    _add_box_visual(
        part,
        name="spine_root_post",
        x0=-0.010,
        x1=0.002,
        width=CENTER_KNUCKLE_WIDTH,
        height=0.030,
        z_center=0.015,
        material=material,
    )
    _add_y_cylinder_visual(
        part,
        name="spine_root_knuckle",
        radius=PIN_RADIUS,
        length=CENTER_KNUCKLE_WIDTH,
        x=0.0,
        y=0.0,
        z=0.0,
        material=material,
    )
    _add_box_visual(
        part,
        name="spine_rail",
        x0=-0.004,
        x1=rail_end,
        width=SPINE_RAIL_WIDTH,
        height=SPINE_RAIL_HEIGHT,
        z_center=SPINE_RAIL_Z_CENTER,
        material=material,
    )
    for rung_x in (
        0.007,
        PROXIMAL_LENGTH + 0.006,
        PROXIMAL_LENGTH + MIDDLE_LENGTH + 0.006,
    ):
        _add_box_visual(
            part,
            name=f"rung_{int(round(rung_x * 1000.0))}",
            x0=rung_x - 0.003,
            x1=rung_x + 0.003,
            width=SPINE_RAIL_WIDTH,
            height=rung_height,
            z_center=rung_z_center,
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_backed_finger_chain")

    model.material("spine_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("phalanx_finish", rgba=(0.73, 0.75, 0.79, 1.0))

    rear_spine = model.part("rear_spine")
    _build_rear_spine(rear_spine, material="spine_finish")

    proximal = model.part("proximal_phalanx")
    _build_phalanx_part(proximal, prefix="proximal", length=PROXIMAL_LENGTH, material="phalanx_finish")

    middle = model.part("middle_phalanx")
    _build_phalanx_part(middle, prefix="middle", length=MIDDLE_LENGTH, material="phalanx_finish")

    distal = model.part("distal_phalanx")
    _build_phalanx_part(
        distal,
        prefix="distal",
        length=DISTAL_LENGTH,
        material="phalanx_finish",
        distal=True,
    )

    model.articulation(
        "spine_to_proximal",
        ArticulationType.REVOLUTE,
        parent=rear_spine,
        child=proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=4.0, velocity=2.5),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=3.0, velocity=2.8),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.00, effort=2.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_spine = object_model.get_part("rear_spine")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")
    spine_to_proximal = object_model.get_articulation("spine_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

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

    joints = (spine_to_proximal, proximal_to_middle, middle_to_distal)
    axes_ok = all(
        abs(joint.axis[0]) < 1e-9 and abs(joint.axis[1] - 1.0) < 1e-9 and abs(joint.axis[2]) < 1e-9
        for joint in joints
    )
    ctx.check(
        "three parallel revolute axes",
        axes_ok,
        details=f"joint axes are {[joint.axis for joint in joints]}",
    )

    ctx.expect_contact(
        proximal,
        rear_spine,
        contact_tol=0.0008,
        name="proximal clevis is supported by rear spine knuckle",
    )
    ctx.expect_contact(
        middle,
        proximal,
        contact_tol=0.0008,
        name="middle clevis is supported by proximal knuckle",
    )
    ctx.expect_contact(
        distal,
        middle,
        contact_tol=0.0008,
        name="distal clevis is supported by middle knuckle",
    )

    ctx.expect_overlap(
        proximal,
        rear_spine,
        axes="xz",
        min_overlap=0.012,
        name="root hinge reads with generous overlap",
    )
    ctx.expect_overlap(
        middle,
        proximal,
        axes="xz",
        min_overlap=0.012,
        name="middle hinge reads with generous overlap",
    )
    ctx.expect_overlap(
        distal,
        middle,
        axes="xz",
        min_overlap=0.011,
        name="distal hinge reads with generous overlap",
    )

    rest_tip_position = ctx.part_world_position(distal)
    with ctx.pose(
        {
            spine_to_proximal: 0.70 * spine_to_proximal.motion_limits.upper,
            proximal_to_middle: 0.70 * proximal_to_middle.motion_limits.upper,
            middle_to_distal: 0.70 * middle_to_distal.motion_limits.upper,
        }
    ):
        curled_tip_position = ctx.part_world_position(distal)
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap in representative curled pose")

    ctx.check(
        "positive flexion curls the chain downward",
        curled_tip_position is not None
        and rest_tip_position is not None
        and curled_tip_position[2] < rest_tip_position[2] - 0.012
        and curled_tip_position[0] < rest_tip_position[0] - 0.010,
        details=f"rest={rest_tip_position}, curled={curled_tip_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
