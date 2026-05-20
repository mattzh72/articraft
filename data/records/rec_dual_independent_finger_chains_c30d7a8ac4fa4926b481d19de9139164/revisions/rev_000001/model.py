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


SUPPORT_BEAM = (0.18, 0.05, 0.018)
SUPPORT_CAP = (0.082, 0.03, 0.01)
SUPPORT_HANGER = (0.018, 0.02, 0.022)
SUPPORT_HANGER_X = 0.04
SUPPORT_LOWER_BRACE = (0.095, 0.012, 0.012)

PALM_SIZE = (0.118, 0.034, 0.028)
PALM_CENTER_Z = -0.045
PALM_ROOT_X = 0.033
PALM_ROOT_JOINT_Z = -(PALM_SIZE[2] / 2.0 + 0.007)

PIN_RADIUS = 0.0035
PIN_SPAN = 0.024
CLEVIS_SLOT = 0.010
CLEVIS_HEIGHT = 0.016
CLEVIS_SLOT_HEIGHT = 0.012

LEFT_AXIS = (0.0, -1.0, 0.0)
RIGHT_AXIS = (0.0, 1.0, 0.0)

LEFT_PROX_LEN = 0.05
LEFT_MID_LEN = 0.04
LEFT_DIST_LEN = 0.034
RIGHT_PROX_LEN = 0.054
RIGHT_MID_LEN = 0.038
RIGHT_DIST_LEN = 0.032


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _hinge_hole(
    *,
    z: float,
    radius: float = PIN_RADIUS,
    depth: float = PIN_SPAN,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(depth / 2.0 + 0.002, both=True)
        .translate((0.0, 0.0, z))
    )


def _clevis_block(
    *,
    width: float,
    depth: float,
    height: float,
    joint_z: float,
    slot_depth: float = CLEVIS_SLOT,
    slot_height: float = CLEVIS_SLOT_HEIGHT,
    pin_radius: float = PIN_RADIUS,
) -> cq.Workplane:
    ear_thickness = (depth - slot_depth) / 2.0
    roof_thickness = max(0.003, height - slot_height)
    ear_y = slot_depth / 2.0 + ear_thickness / 2.0
    roof_center_z = joint_z + height / 2.0 - roof_thickness / 2.0

    front_ear = _box((width, ear_thickness, height), (0.0, ear_y, joint_z))
    rear_ear = _box((width, ear_thickness, height), (0.0, -ear_y, joint_z))
    roof = _box((width, depth, roof_thickness), (0.0, 0.0, roof_center_z))
    return front_ear.union(rear_ear).union(roof).cut(
        _hinge_hole(z=joint_z, radius=pin_radius, depth=depth)
    )


def _tapered_bar(
    *,
    length: float,
    top_width: float,
    bottom_width: float,
    thickness: float,
    bottom_z: float,
) -> cq.Workplane:
    top_z = -0.001
    shoulder_z = bottom_z + 0.012
    return (
        cq.Workplane("XZ")
        .moveTo(-top_width / 2.0, top_z)
        .lineTo(-top_width / 2.0, shoulder_z)
        .lineTo(-bottom_width / 2.0, bottom_z)
        .lineTo(bottom_width / 2.0, bottom_z)
        .lineTo(top_width / 2.0, shoulder_z)
        .lineTo(top_width / 2.0, top_z)
        .close()
        .extrude(thickness / 2.0, both=True)
    )


def _make_support_shape() -> cq.Workplane:
    beam = _box(SUPPORT_BEAM, (0.0, 0.0, 0.0))
    cap = _box(
        SUPPORT_CAP,
        (0.0, 0.0, SUPPORT_BEAM[2] / 2.0 + SUPPORT_CAP[2] / 2.0),
    )
    lower_brace = _box(
        SUPPORT_LOWER_BRACE,
        (0.0, 0.0, -SUPPORT_BEAM[2] / 2.0 - SUPPORT_LOWER_BRACE[2] / 2.0),
    )
    support = beam.union(cap).union(lower_brace)
    hanger_center_z = -SUPPORT_BEAM[2] / 2.0 - SUPPORT_HANGER[2] / 2.0
    for x_pos in (-SUPPORT_HANGER_X, SUPPORT_HANGER_X):
        support = support.union(_box(SUPPORT_HANGER, (x_pos, 0.0, hanger_center_z)))
    return support.edges("|Y").fillet(0.0015)


def _make_palm_shape() -> cq.Workplane:
    palm = _box(PALM_SIZE, (0.0, 0.0, 0.0)).edges("|Z").fillet(0.0015)
    underside_spine = _box((0.045, 0.018, 0.01), (0.0, 0.0, -PALM_SIZE[2] / 2.0 - 0.003))
    palm = palm.union(underside_spine)
    for x_pos in (-PALM_ROOT_X, PALM_ROOT_X):
        root_mount = _clevis_block(
            width=0.02,
            depth=PIN_SPAN,
            height=CLEVIS_HEIGHT,
            joint_z=PALM_ROOT_JOINT_Z,
        ).translate((x_pos, 0.0, 0.0))
        palm = palm.union(root_mount)
    return palm


def _make_intermediate_link(
    *,
    length: float,
    top_width: float,
    mid_width: float,
    thickness: float,
    clevis_width: float,
) -> cq.Workplane:
    body = _tapered_bar(
        length=length,
        top_width=top_width,
        bottom_width=mid_width,
        thickness=thickness,
        bottom_z=-(length - 0.005),
    )
    root_pin = _hinge_hole(z=0.0, radius=PIN_RADIUS, depth=PIN_SPAN)
    tip_clevis = _clevis_block(
        width=clevis_width,
        depth=PIN_SPAN,
        height=CLEVIS_HEIGHT,
        joint_z=-length,
    )
    return body.union(root_pin).union(tip_clevis)


def _make_terminal_link(
    *,
    length: float,
    top_width: float,
    tip_width: float,
    thickness: float,
) -> cq.Workplane:
    body = _tapered_bar(
        length=length,
        top_width=top_width,
        bottom_width=tip_width,
        thickness=thickness,
        bottom_z=-length,
    )
    root_pin = _hinge_hole(z=0.0, radius=PIN_RADIUS, depth=PIN_SPAN)
    fingertip = _box((tip_width * 1.2, thickness * 1.15, 0.018), (0.0, 0.0, -length + 0.009))
    toe = _box((tip_width * 0.9, thickness * 0.9, 0.012), (0.0, 0.0, -length - 0.004))
    return body.union(root_pin).union(fingertip).union(toe)


def _add_mesh_part(
    *,
    model: ArticulatedObject,
    name: str,
    shape: cq.Workplane,
    material: str,
):
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, name), material=material, name=f"{name}_body")
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_dual_finger_rig")

    model.material("support_dark", rgba=(0.18, 0.2, 0.23, 1.0))
    model.material("palm_gray", rgba=(0.36, 0.39, 0.43, 1.0))
    model.material("finger_alloy", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("tip_dark", rgba=(0.25, 0.27, 0.3, 1.0))

    top_support = _add_mesh_part(
        model=model,
        name="top_support",
        shape=_make_support_shape(),
        material="support_dark",
    )
    palm_block = _add_mesh_part(
        model=model,
        name="palm_block",
        shape=_make_palm_shape(),
        material="palm_gray",
    )

    left_proximal = _add_mesh_part(
        model=model,
        name="left_proximal",
        shape=_make_intermediate_link(
            length=LEFT_PROX_LEN,
            top_width=0.02,
            mid_width=0.015,
            thickness=0.009,
            clevis_width=0.018,
        ),
        material="finger_alloy",
    )
    left_middle = _add_mesh_part(
        model=model,
        name="left_middle",
        shape=_make_intermediate_link(
            length=LEFT_MID_LEN,
            top_width=0.018,
            mid_width=0.013,
            thickness=0.0085,
            clevis_width=0.016,
        ),
        material="finger_alloy",
    )
    left_distal = _add_mesh_part(
        model=model,
        name="left_distal",
        shape=_make_terminal_link(
            length=LEFT_DIST_LEN,
            top_width=0.016,
            tip_width=0.014,
            thickness=0.008,
        ),
        material="tip_dark",
    )

    right_proximal = _add_mesh_part(
        model=model,
        name="right_proximal",
        shape=_make_intermediate_link(
            length=RIGHT_PROX_LEN,
            top_width=0.021,
            mid_width=0.016,
            thickness=0.0095,
            clevis_width=0.0185,
        ),
        material="finger_alloy",
    )
    right_middle = _add_mesh_part(
        model=model,
        name="right_middle",
        shape=_make_intermediate_link(
            length=RIGHT_MID_LEN,
            top_width=0.0185,
            mid_width=0.0135,
            thickness=0.0085,
            clevis_width=0.016,
        ),
        material="finger_alloy",
    )
    right_distal = _add_mesh_part(
        model=model,
        name="right_distal",
        shape=_make_terminal_link(
            length=RIGHT_DIST_LEN,
            top_width=0.016,
            tip_width=0.013,
            thickness=0.0078,
        ),
        material="tip_dark",
    )

    model.articulation(
        "support_to_palm",
        ArticulationType.FIXED,
        parent=top_support,
        child=palm_block,
        origin=Origin(xyz=(0.0, 0.0, PALM_CENTER_Z)),
    )

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm_block,
        child=left_proximal,
        origin=Origin(xyz=(-PALM_ROOT_X, 0.0, PALM_ROOT_JOINT_Z)),
        axis=LEFT_AXIS,
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.0),
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, 0.0, -LEFT_PROX_LEN)),
        axis=LEFT_AXIS,
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.08),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, 0.0, -LEFT_MID_LEN)),
        axis=LEFT_AXIS,
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=0.95),
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm_block,
        child=right_proximal,
        origin=Origin(xyz=(PALM_ROOT_X, 0.0, PALM_ROOT_JOINT_Z)),
        axis=RIGHT_AXIS,
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.0),
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, 0.0, -RIGHT_PROX_LEN)),
        axis=RIGHT_AXIS,
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.06),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, 0.0, -RIGHT_MID_LEN)),
        axis=RIGHT_AXIS,
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=0.92),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    palm_block = object_model.get_part("palm_block")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_root = object_model.get_articulation("palm_to_left_proximal")
    left_mid_joint = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip_joint = object_model.get_articulation("left_middle_to_left_distal")
    right_root = object_model.get_articulation("palm_to_right_proximal")
    right_mid_joint = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip_joint = object_model.get_articulation("right_middle_to_right_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        palm_block,
        left_proximal,
        elem_a="palm_block_body",
        elem_b="left_proximal_body",
        reason="The left root hinge is represented as a captive axle nested inside the palm clevis.",
    )
    ctx.allow_overlap(
        left_proximal,
        left_middle,
        elem_a="left_proximal_body",
        elem_b="left_middle_body",
        reason="The left middle hinge uses an intentionally nested clevis-and-axle proxy.",
    )
    ctx.allow_overlap(
        left_middle,
        left_distal,
        elem_a="left_middle_body",
        elem_b="left_distal_body",
        reason="The left distal hinge uses an intentionally nested clevis-and-axle proxy.",
    )
    ctx.allow_overlap(
        palm_block,
        right_proximal,
        elem_a="palm_block_body",
        elem_b="right_proximal_body",
        reason="The right root hinge is represented as a captive axle nested inside the palm clevis.",
    )
    ctx.allow_overlap(
        right_proximal,
        right_middle,
        elem_a="right_proximal_body",
        elem_b="right_middle_body",
        reason="The right middle hinge uses an intentionally nested clevis-and-axle proxy.",
    )
    ctx.allow_overlap(
        right_middle,
        right_distal,
        elem_a="right_middle_body",
        elem_b="right_distal_body",
        reason="The right distal hinge uses an intentionally nested clevis-and-axle proxy.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(top_support, palm_block, contact_tol=1e-5, name="support carries palm block")
    ctx.expect_contact(palm_block, left_proximal, contact_tol=1e-5, name="left root hinge is supported")
    ctx.expect_contact(left_proximal, left_middle, contact_tol=1e-5, name="left middle hinge is supported")
    ctx.expect_contact(left_middle, left_distal, contact_tol=1e-5, name="left distal hinge is supported")
    ctx.expect_contact(palm_block, right_proximal, contact_tol=1e-5, name="right root hinge is supported")
    ctx.expect_contact(right_proximal, right_middle, contact_tol=1e-5, name="right middle hinge is supported")
    ctx.expect_contact(right_middle, right_distal, contact_tol=1e-5, name="right distal hinge is supported")

    ctx.check(
        "left finger uses inward-closing joint axes",
        left_root.axis == LEFT_AXIS and left_mid_joint.axis == LEFT_AXIS and left_tip_joint.axis == LEFT_AXIS,
        details=f"axes={[left_root.axis, left_mid_joint.axis, left_tip_joint.axis]}",
    )
    ctx.check(
        "right finger uses inward-closing joint axes",
        right_root.axis == RIGHT_AXIS and right_mid_joint.axis == RIGHT_AXIS and right_tip_joint.axis == RIGHT_AXIS,
        details=f"axes={[right_root.axis, right_mid_joint.axis, right_tip_joint.axis]}",
    )

    rest_left_tip = ctx.part_world_position(left_distal)
    rest_right_tip = ctx.part_world_position(right_distal)
    rest_right_root = ctx.part_world_position(right_proximal)

    with ctx.pose({left_root: 0.55, left_mid_joint: 0.45, left_tip_joint: 0.3}):
        curled_left_tip = ctx.part_world_position(left_distal)
        parked_right_tip = ctx.part_world_position(right_distal)
        parked_right_root = ctx.part_world_position(right_proximal)

    with ctx.pose({right_root: 0.55, right_mid_joint: 0.42, right_tip_joint: 0.28}):
        curled_right_tip = ctx.part_world_position(right_distal)

    ctx.check(
        "left finger curls inward without dragging right chain",
        rest_left_tip is not None
        and curled_left_tip is not None
        and rest_right_tip is not None
        and parked_right_tip is not None
        and rest_right_root is not None
        and parked_right_root is not None
        and curled_left_tip[0] > rest_left_tip[0] + 0.01
        and abs(parked_right_tip[0] - rest_right_tip[0]) < 1e-6
        and abs(parked_right_tip[2] - rest_right_tip[2]) < 1e-6
        and abs(parked_right_root[0] - rest_right_root[0]) < 1e-6,
        details=(
            f"rest_left_tip={rest_left_tip}, curled_left_tip={curled_left_tip}, "
            f"rest_right_tip={rest_right_tip}, parked_right_tip={parked_right_tip}, "
            f"rest_right_root={rest_right_root}, parked_right_root={parked_right_root}"
        ),
    )
    ctx.check(
        "right finger curls inward",
        rest_right_tip is not None
        and curled_right_tip is not None
        and curled_right_tip[0] < rest_right_tip[0] - 0.01,
        details=f"rest_right_tip={rest_right_tip}, curled_right_tip={curled_right_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
