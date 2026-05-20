from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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


MESH_TOL = 0.00035
ANG_TOL = 0.08

CHEEK_T = 0.006

LINK_1_LEN = 0.190
LINK_2_LEN = 0.155
LINK_3_LEN = 0.125

LINK_1_EYE_W = 0.018
LINK_2_EYE_W = 0.015
LINK_3_EYE_W = 0.012

LINK_1_EYE_R = 0.015
LINK_2_EYE_R = 0.0125
LINK_3_EYE_R = 0.010

PIN_1_R = 0.0065
PIN_2_R = 0.0055
PIN_3_R = 0.0045

ROOT_GAP = LINK_1_EYE_W
JOINT_2_GAP = LINK_2_EYE_W
JOINT_3_GAP = LINK_3_EYE_W

LINK_1_CLEVIS_H = 0.024
LINK_2_CLEVIS_H = 0.019
SUPPORT_CLEVIS_H = 0.030

END_PAD_SIZE = (0.028, 0.018, 0.006)
END_PAD_CENTER_X = 0.122


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate(center)
    )


def _ring_y(
    *,
    outer_radius: float,
    inner_radius: float,
    width: float,
    center_x: float = 0.0,
) -> cq.Workplane:
    outer = _y_cylinder(outer_radius, width, (center_x, 0.0, 0.0))
    inner = _y_cylinder(inner_radius, width + 0.003, (center_x, 0.0, 0.0))
    return outer.cut(inner)


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_y_cylinder_visual(
    part,
    *,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _bridge_support_shape() -> cq.Workplane:
    support = _box((0.095, 0.050, 0.014), (-0.055, 0.065, -0.083))
    support = support.union(_box((0.095, 0.050, 0.014), (-0.055, -0.065, -0.083)))
    support = support.union(_box((0.055, 0.022, 0.058), (-0.055, 0.065, -0.047)))
    support = support.union(_box((0.055, 0.022, 0.058), (-0.055, -0.065, -0.047)))
    support = support.union(_box((0.064, 0.152, 0.016), (-0.050, 0.000, -0.016)))
    root_bridge = _box((0.018, ROOT_GAP + 2.0 * CHEEK_T, 0.014), (-0.026, 0.000, 0.000))
    hanger = _box((0.018, 0.030, 0.024), (-0.026, 0.000, -0.012))
    root_cheek_center_x = -0.006
    root_cheek_len = 0.040
    root_cheeks = _box(
        (root_cheek_len, CHEEK_T, SUPPORT_CLEVIS_H),
        (root_cheek_center_x, ROOT_GAP / 2.0 + CHEEK_T / 2.0, 0.000),
    )
    root_cheeks = root_cheeks.union(
        _box(
            (root_cheek_len, CHEEK_T, SUPPORT_CLEVIS_H),
            (root_cheek_center_x, -ROOT_GAP / 2.0 - CHEEK_T / 2.0, 0.000),
        )
    )
    support = support.union(hanger).union(root_bridge).union(root_cheeks)
    return support


def _serial_link_shape(
    *,
    length: float,
    prox_eye_width: float,
    prox_eye_radius: float,
    prox_pin_radius: float,
    beam_w0: float,
    beam_h0: float,
    beam_w1: float,
    beam_h1: float,
    distal_gap: float,
    distal_eye_radius: float,
    distal_pin_radius: float,
    distal_cheek_len: float,
    distal_cheek_height: float,
) -> cq.Workplane:
    eye = _ring_y(
        outer_radius=prox_eye_radius,
        inner_radius=prox_pin_radius,
        width=prox_eye_width,
    )
    distal_outer_w = distal_gap + 2.0 * CHEEK_T
    clevis_back_x = length - distal_eye_radius - 0.016
    base_len = 0.012
    clevis_base_center_x = clevis_back_x + base_len / 2.0
    beam = (
        cq.Workplane("YZ")
        .workplane(offset=prox_eye_radius * 0.55)
        .rect(beam_w0, beam_h0)
        .workplane(offset=clevis_back_x - 0.002)
        .rect(beam_w1, beam_h1)
        .loft(combine=True)
    )
    clevis_base = _box(
        (base_len, distal_outer_w, min(beam_h1 + 0.003, distal_cheek_height * 0.70)),
        (clevis_base_center_x, 0.000, 0.000),
    )
    cheek_back_x = clevis_back_x
    cheek_front_x = length + distal_eye_radius + 0.003
    cheek_len = max(distal_cheek_len, cheek_front_x - cheek_back_x)
    cheek_center_x = (cheek_front_x + cheek_back_x) / 2.0
    cheeks = _box(
        (cheek_len, CHEEK_T, distal_cheek_height),
        (cheek_center_x, distal_gap / 2.0 + CHEEK_T / 2.0, 0.000),
    )
    cheeks = cheeks.union(
        _box(
            (cheek_len, CHEEK_T, distal_cheek_height),
            (cheek_center_x, -distal_gap / 2.0 - CHEEK_T / 2.0, 0.000),
        )
    )
    return eye.union(beam).union(clevis_base).union(cheeks)


def _terminal_link_shape() -> cq.Workplane:
    eye = _ring_y(
        outer_radius=LINK_3_EYE_R,
        inner_radius=PIN_3_R,
        width=LINK_3_EYE_W,
    )
    beam = (
        cq.Workplane("YZ")
        .workplane(offset=LINK_3_EYE_R * 0.55)
        .rect(0.017, 0.012)
        .workplane(offset=0.100)
        .rect(0.010, 0.008)
        .loft(combine=True)
    )
    tip = _box((0.016, 0.010, 0.007), (0.100, 0.000, 0.000))
    return eye.union(beam).union(tip)


def _foot_bolt_positions() -> list[tuple[float, float, float]]:
    foot_centers = (0.065, -0.065)
    x_offsets = (-0.074, -0.036)
    y_offsets = (-0.014, 0.014)
    z = -0.0745
    return [(x, yc + yo, z) for yc in foot_centers for x in x_offsets for yo in y_offsets]


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=MESH_TOL,
        angular_tolerance=ANG_TOL,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_folding_arm_chain")

    model.material("painted_steel", rgba=(0.24, 0.27, 0.31, 1.0))
    model.material("link_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("joint_hardware", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("end_pad_black", rgba=(0.10, 0.11, 0.12, 1.0))

    bridge_support = model.part("bridge_support")
    _add_box_visual(
        bridge_support,
        size=(0.095, 0.050, 0.014),
        center=(-0.055, 0.065, -0.083),
        material="painted_steel",
        name="foot_upper",
    )
    _add_box_visual(
        bridge_support,
        size=(0.095, 0.050, 0.014),
        center=(-0.055, -0.065, -0.083),
        material="painted_steel",
        name="foot_lower",
    )
    _add_box_visual(
        bridge_support,
        size=(0.055, 0.022, 0.058),
        center=(-0.055, 0.065, -0.047),
        material="painted_steel",
        name="upright_upper",
    )
    _add_box_visual(
        bridge_support,
        size=(0.055, 0.022, 0.058),
        center=(-0.055, -0.065, -0.047),
        material="painted_steel",
        name="upright_lower",
    )
    _add_box_visual(
        bridge_support,
        size=(0.064, 0.152, 0.016),
        center=(-0.050, 0.000, -0.016),
        material="painted_steel",
        name="bridge_crossmember",
    )
    _add_box_visual(
        bridge_support,
        size=(0.018, 0.030, 0.024),
        center=(-0.029, 0.000, -0.012),
        material="painted_steel",
        name="root_hanger",
    )
    _add_box_visual(
        bridge_support,
        size=(0.014, ROOT_GAP + 2.0 * CHEEK_T, 0.014),
        center=(-0.029, 0.000, 0.000),
        material="painted_steel",
        name="root_tie_bar",
    )
    _add_box_visual(
        bridge_support,
        size=(0.028, CHEEK_T, SUPPORT_CLEVIS_H),
        center=(-0.008, ROOT_GAP / 2.0 + CHEEK_T / 2.0, 0.000),
        material="painted_steel",
        name="root_cheek_left",
    )
    _add_box_visual(
        bridge_support,
        size=(0.028, CHEEK_T, SUPPORT_CLEVIS_H),
        center=(-0.008, -ROOT_GAP / 2.0 - CHEEK_T / 2.0, 0.000),
        material="painted_steel",
        name="root_cheek_right",
    )
    for idx, pos in enumerate(_foot_bolt_positions(), start=1):
        bridge_support.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=pos),
            material="joint_hardware",
            name=f"mount_bolt_{idx}",
        )

    link_1 = model.part("link_1")
    _add_y_cylinder_visual(
        link_1,
        radius=LINK_1_EYE_R,
        length=LINK_1_EYE_W,
        center=(0.000, 0.000, 0.000),
        material="link_aluminum",
        name="link_1_eye",
    )
    _add_box_visual(
        link_1,
        size=(0.060, 0.018, 0.019),
        center=(0.040, 0.000, 0.000),
        material="link_aluminum",
        name="link_1_beam_a",
    )
    _add_box_visual(
        link_1,
        size=(0.060, 0.015, 0.016),
        center=(0.095, 0.000, 0.000),
        material="link_aluminum",
        name="link_1_beam_b",
    )
    _add_box_visual(
        link_1,
        size=(0.040, 0.012, 0.013),
        center=(0.145, 0.000, 0.000),
        material="link_aluminum",
        name="link_1_beam_c",
    )
    _add_box_visual(
        link_1,
        size=(0.010, JOINT_2_GAP + 2.0 * CHEEK_T, 0.011),
        center=(0.169, 0.000, 0.000),
        material="link_aluminum",
        name="link_1_clevis_tie",
    )
    _add_box_visual(
        link_1,
        size=(0.030, CHEEK_T, LINK_1_CLEVIS_H),
        center=(0.187, JOINT_2_GAP / 2.0 + CHEEK_T / 2.0, 0.000),
        material="link_aluminum",
        name="link_1_clevis_left",
    )
    _add_box_visual(
        link_1,
        size=(0.030, CHEEK_T, LINK_1_CLEVIS_H),
        center=(0.187, -JOINT_2_GAP / 2.0 - CHEEK_T / 2.0, 0.000),
        material="link_aluminum",
        name="link_1_clevis_right",
    )

    link_2 = model.part("link_2")
    _add_y_cylinder_visual(
        link_2,
        radius=LINK_2_EYE_R,
        length=LINK_2_EYE_W,
        center=(0.000, 0.000, 0.000),
        material="link_aluminum",
        name="link_2_eye",
    )
    _add_box_visual(
        link_2,
        size=(0.048, 0.015, 0.016),
        center=(0.033, 0.000, 0.000),
        material="link_aluminum",
        name="link_2_beam_a",
    )
    _add_box_visual(
        link_2,
        size=(0.046, 0.013, 0.013),
        center=(0.080, 0.000, 0.000),
        material="link_aluminum",
        name="link_2_beam_b",
    )
    _add_box_visual(
        link_2,
        size=(0.030, 0.010, 0.010),
        center=(0.118, 0.000, 0.000),
        material="link_aluminum",
        name="link_2_beam_c",
    )
    _add_box_visual(
        link_2,
        size=(0.010, JOINT_3_GAP + 2.0 * CHEEK_T, 0.009),
        center=(0.138, 0.000, 0.000),
        material="link_aluminum",
        name="link_2_clevis_tie",
    )
    _add_box_visual(
        link_2,
        size=(0.024, CHEEK_T, LINK_2_CLEVIS_H),
        center=(0.153, JOINT_3_GAP / 2.0 + CHEEK_T / 2.0, 0.000),
        material="link_aluminum",
        name="link_2_clevis_left",
    )
    _add_box_visual(
        link_2,
        size=(0.024, CHEEK_T, LINK_2_CLEVIS_H),
        center=(0.153, -JOINT_3_GAP / 2.0 - CHEEK_T / 2.0, 0.000),
        material="link_aluminum",
        name="link_2_clevis_right",
    )

    link_3 = model.part("link_3")
    _add_y_cylinder_visual(
        link_3,
        radius=LINK_3_EYE_R,
        length=LINK_3_EYE_W,
        center=(0.000, 0.000, 0.000),
        material="link_aluminum",
        name="link_3_eye",
    )
    _add_box_visual(
        link_3,
        size=(0.040, 0.012, 0.012),
        center=(0.028, 0.000, 0.000),
        material="link_aluminum",
        name="link_3_beam_a",
    )
    _add_box_visual(
        link_3,
        size=(0.034, 0.010, 0.010),
        center=(0.065, 0.000, 0.000),
        material="link_aluminum",
        name="link_3_beam_b",
    )
    _add_box_visual(
        link_3,
        size=(0.020, 0.008, 0.008),
        center=(0.090, 0.000, 0.000),
        material="link_aluminum",
        name="link_3_beam_c",
    )
    _add_box_visual(
        link_3,
        size=(0.016, 0.010, 0.007),
        center=(0.106, 0.000, 0.000),
        material="link_aluminum",
        name="link_3_tip",
    )
    link_3.visual(
        Box(END_PAD_SIZE),
        origin=Origin(xyz=(END_PAD_CENTER_X, 0.0, 0.0)),
        material="end_pad_black",
        name="end_pad",
    )

    model.articulation(
        "support_to_link_1",
        ArticulationType.REVOLUTE,
        parent=bridge_support,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.15, effort=40.0, velocity=1.2),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.45, upper=1.45, effort=28.0, velocity=1.6),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.50, upper=1.30, effort=18.0, velocity=1.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge_support = object_model.get_part("bridge_support")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    root_joint = object_model.get_articulation("support_to_link_1")
    elbow_joint = object_model.get_articulation("link_1_to_link_2")
    wrist_joint = object_model.get_articulation("link_2_to_link_3")

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

    ctx.expect_contact(link_1, bridge_support, name="root_clevis_contacts_first_link")
    ctx.expect_contact(link_1, link_2, name="first_interlink_clevis_contacts_second_link")
    ctx.expect_contact(link_2, link_3, name="second_interlink_clevis_contacts_terminal_link")

    ctx.expect_origin_gap(
        link_2,
        bridge_support,
        axis="x",
        min_gap=LINK_1_LEN - 0.001,
        max_gap=LINK_1_LEN + 0.001,
        name="first_link_sets_expected_joint_spacing",
    )
    ctx.expect_origin_gap(
        link_3,
        link_2,
        axis="x",
        min_gap=LINK_2_LEN - 0.001,
        max_gap=LINK_2_LEN + 0.001,
        name="second_link_sets_expected_joint_spacing",
    )

    with ctx.pose({root_joint: 0.80}):
        lifted_joint = ctx.part_world_position(link_2)
        ctx.check(
            "positive_root_rotation_lifts_chain",
            lifted_joint is not None and lifted_joint[0] > 0.12 and lifted_joint[2] > 0.12,
            details=f"link_2 origin at {lifted_joint}",
        )

    with ctx.pose({root_joint: 0.45, elbow_joint: 1.00, wrist_joint: 0.65}):
        end_pad_aabb = ctx.part_element_world_aabb(link_3, elem="end_pad")
        if end_pad_aabb is None:
            ctx.fail("distal_end_pad_exists_in_folded_pose", "No AABB returned for link_3:end_pad")
        else:
            end_pad_center = tuple((lo + hi) / 2.0 for lo, hi in zip(end_pad_aabb[0], end_pad_aabb[1]))
            ctx.check(
                "folded_pose_keeps_end_pad_clear_and_elevated",
                end_pad_center[0] > 0.10 and end_pad_center[2] > 0.16,
                details=f"folded end pad center at {end_pad_center}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
