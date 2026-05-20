from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


PALM_THICK = 0.020
PALM_DEPTH = 0.038
PALM_WIDTH = 0.082
PALM_BODY_LENGTH = 0.016
PALM_BODY_CENTER_X = -0.010
PALM_BOSS_START_X = -0.006
PALM_BOSS_HEIGHT = 0.022
ROOT_SPREAD = 0.046

JOINT_TOTAL_WIDTH = 0.014
CENTER_BARREL_LEN = JOINT_TOTAL_WIDTH

ROOT_RADIUS = 0.0065
MIDDLE_RADIUS = 0.0056
TIP_RADIUS = 0.0046
ROOT_JOINT_X = 0.0085
ROOT_CONTACT_X = ROOT_JOINT_X - ROOT_RADIUS

PROXIMAL_LENGTH = 0.042
MIDDLE_LENGTH = 0.032
DISTAL_LENGTH = 0.024


def _barrel(*, radius: float, x_center: float = 0.0, z_center: float = 0.0, width: float = JOINT_TOTAL_WIDTH) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(width)
        .translate((0.0, 0.0, -width / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x_center, 0.0, z_center))
    )


def _tapered_body(
    *,
    start_x: float,
    end_x: float,
    body_thickness: float,
    root_height: float,
    mid_height: float,
    tip_height: float,
    tip_thickness_scale: float,
) -> cq.Workplane:
    span = end_x - start_x
    return (
        cq.Workplane("YZ")
        .workplane(offset=start_x)
        .rect(body_thickness, root_height)
        .workplane(offset=span * 0.56)
        .rect(body_thickness * 0.92, mid_height)
        .workplane(offset=span * 0.44)
        .rect(body_thickness * tip_thickness_scale, tip_height)
        .loft(combine=True)
    )


def _end_pad(*, contact_x: float, length: float, width: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .translate((contact_x - length / 2.0, 0.0, 0.0))
    )


def _make_link_shape(
    *,
    length: float,
    root_height: float,
    mid_height: float,
    tip_height: float,
    root_radius: float,
    distal_radius: float | None,
    body_thickness: float,
    tip_thickness_scale: float,
) -> cq.Workplane:
    contact_x = length if distal_radius is None else length - distal_radius
    body_start_x = max(root_radius * 0.55, 0.0028)
    body = _tapered_body(
        start_x=body_start_x,
        end_x=contact_x,
        body_thickness=body_thickness,
        root_height=root_height,
        mid_height=mid_height,
        tip_height=tip_height,
        tip_thickness_scale=tip_thickness_scale,
    )
    root_web_start = -root_radius * 0.35
    root_web_end = body_start_x + root_radius * 0.95
    root_web = (
        cq.Workplane("XY")
        .box(
            root_web_end - root_web_start,
            body_thickness * 0.82,
            root_height * 0.82,
        )
        .translate(((root_web_start + root_web_end) / 2.0, 0.0, 0.0))
    )
    end_pad_length = max(root_radius * 1.3, 0.0045)
    shape = _barrel(radius=root_radius).union(root_web).union(body).union(
        _end_pad(
            contact_x=contact_x,
            length=end_pad_length,
            width=body_thickness * 0.78,
            height=max(tip_height * 0.92, root_height * 0.56),
        )
    )

    if distal_radius is None:
        tip_radius = max(body_thickness * 0.32, tip_height * 0.34)
        tip = cq.Workplane("XY").sphere(tip_radius).translate((length - tip_radius * 0.75, 0.0, 0.0))
        return shape.union(tip)

    return shape


def _make_palm_shape() -> cq.Workplane:
    palm = (
        cq.Workplane("XY")
        .box(PALM_BODY_LENGTH, PALM_DEPTH, PALM_WIDTH)
        .translate((PALM_BODY_CENTER_X, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.0022)
    )
    boss_length = ROOT_CONTACT_X - PALM_BOSS_START_X
    for z_offset in (-ROOT_SPREAD / 2.0, ROOT_SPREAD / 2.0):
        boss = (
            cq.Workplane("YZ")
            .workplane(offset=PALM_BOSS_START_X)
            .rect(0.014, PALM_BOSS_HEIGHT)
            .workplane(offset=boss_length * 0.55)
            .rect(0.013, PALM_BOSS_HEIGHT * 0.90)
            .workplane(offset=boss_length * 0.45)
            .rect(0.0115, PALM_BOSS_HEIGHT * 0.78)
            .loft(combine=True)
            .translate((0.0, 0.0, z_offset))
        )
        palm = palm.union(boss)
    return palm


def _add_link_part(
    *,
    model: ArticulatedObject,
    name: str,
    shape: cq.Workplane,
    material: str,
    length: float,
    z_height: float,
    mass: float,
) -> None:
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, name), material=material, name=f"{name}_shell")
    part.inertial = Inertial.from_geometry(
        Box((length, CENTER_BARREL_LEN, z_height)),
        mass=mass,
        origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parallel_finger_study")

    model.material("palm_graphite", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("proximal_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("middle_silver", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("distal_titanium", rgba=(0.56, 0.59, 0.63, 1.0))

    palm = model.part("palm")
    palm.visual(mesh_from_cadquery(_make_palm_shape(), "palm_plate"), material="palm_graphite", name="palm_shell")
    palm.inertial = Inertial.from_geometry(
        Box((PALM_THICK, PALM_DEPTH, PALM_WIDTH)),
        mass=0.34,
        origin=Origin(xyz=(-PALM_THICK / 2.0, 0.0, 0.0)),
    )

    proximal_shape = _make_link_shape(
        length=PROXIMAL_LENGTH,
        root_height=0.018,
        mid_height=0.017,
        tip_height=0.0145,
        root_radius=ROOT_RADIUS,
        distal_radius=MIDDLE_RADIUS,
        body_thickness=0.0092,
        tip_thickness_scale=0.82,
    )
    middle_shape = _make_link_shape(
        length=MIDDLE_LENGTH,
        root_height=0.0155,
        mid_height=0.0142,
        tip_height=0.0122,
        root_radius=MIDDLE_RADIUS,
        distal_radius=TIP_RADIUS,
        body_thickness=0.0084,
        tip_thickness_scale=0.78,
    )
    distal_shape = _make_link_shape(
        length=DISTAL_LENGTH,
        root_height=0.0132,
        mid_height=0.0110,
        tip_height=0.0086,
        root_radius=TIP_RADIUS,
        distal_radius=None,
        body_thickness=0.0072,
        tip_thickness_scale=0.68,
    )

    for side in ("left", "right"):
        _add_link_part(
            model=model,
            name=f"{side}_proximal",
            shape=proximal_shape,
            material="proximal_silver",
            length=PROXIMAL_LENGTH,
            z_height=0.020,
            mass=0.065,
        )
        _add_link_part(
            model=model,
            name=f"{side}_middle",
            shape=middle_shape,
            material="middle_silver",
            length=MIDDLE_LENGTH,
            z_height=0.017,
            mass=0.045,
        )
        _add_link_part(
            model=model,
            name=f"{side}_distal",
            shape=distal_shape,
            material="distal_titanium",
            length=DISTAL_LENGTH,
            z_height=0.014,
            mass=0.028,
        )

    for side, z_offset in (("left", ROOT_SPREAD / 2.0), ("right", -ROOT_SPREAD / 2.0)):
        proximal = model.get_part(f"{side}_proximal")
        middle = model.get_part(f"{side}_middle")
        distal = model.get_part(f"{side}_distal")
        model.articulation(
            f"{side}_root",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(ROOT_JOINT_X, 0.0, z_offset)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=2.4, lower=-0.10, upper=1.18),
        )
        model.articulation(
            f"{side}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.32),
        )
        model.articulation(
            f"{side}_tip_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=3.4, lower=0.0, upper=1.12),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_root = object_model.get_articulation("left_root")
    left_middle_joint = object_model.get_articulation("left_middle_joint")
    left_tip_joint = object_model.get_articulation("left_tip_joint")
    right_root = object_model.get_articulation("right_root")
    right_middle_joint = object_model.get_articulation("right_middle_joint")
    right_tip_joint = object_model.get_articulation("right_tip_joint")

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

    ctx.expect_contact(palm, left_proximal, name="left_root_knuckle_contact")
    ctx.expect_contact(palm, right_proximal, name="right_root_knuckle_contact")
    ctx.expect_contact(left_proximal, left_middle, name="left_middle_knuckle_contact")
    ctx.expect_contact(left_middle, left_distal, name="left_tip_knuckle_contact")
    ctx.expect_contact(right_proximal, right_middle, name="right_middle_knuckle_contact")
    ctx.expect_contact(right_middle, right_distal, name="right_tip_knuckle_contact")

    ctx.expect_origin_distance(
        left_proximal,
        right_proximal,
        axes="z",
        min_dist=ROOT_SPREAD - 0.001,
        max_dist=ROOT_SPREAD + 0.001,
        name="root_knuckles_are_distinct",
    )
    ctx.expect_origin_gap(left_distal, palm, axis="x", min_gap=0.070, name="left_chain_reaches_forward")
    ctx.expect_origin_gap(right_distal, palm, axis="x", min_gap=0.070, name="right_chain_reaches_forward")

    rest_left_distal = ctx.part_world_position(left_distal)
    rest_right_distal = ctx.part_world_position(right_distal)
    with ctx.pose(
        {
            left_root: 0.52,
            left_middle_joint: 0.68,
            left_tip_joint: 0.46,
        }
    ):
        posed_left_distal = ctx.part_world_position(left_distal)
        posed_right_distal = ctx.part_world_position(right_distal)
        ctx.expect_gap(left_distal, palm, axis="x", min_gap=0.0, max_penetration=0.0, name="left_flex_keeps_distal_ahead_of_palm")

    left_rises = (
        rest_left_distal is not None
        and posed_left_distal is not None
        and posed_left_distal[2] > rest_left_distal[2] + 0.018
    )
    ctx.check(
        "left_chain_positive_rotation_curls_upward",
        left_rises,
        details=f"rest={rest_left_distal}, posed={posed_left_distal}",
    )

    right_unchanged = (
        rest_right_distal is not None
        and posed_right_distal is not None
        and all(isclose(a, b, abs_tol=1e-6) for a, b in zip(rest_right_distal, posed_right_distal))
    )
    ctx.check(
        "right_chain_is_independent_of_left_pose",
        right_unchanged,
        details=f"rest={rest_right_distal}, posed={posed_right_distal}",
    )

    with ctx.pose(
        {
            right_root: 0.44,
            right_middle_joint: 0.62,
            right_tip_joint: 0.40,
        }
    ):
        ctx.expect_gap(right_distal, palm, axis="x", min_gap=0.0, max_penetration=0.0, name="right_flex_keeps_distal_ahead_of_palm")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
