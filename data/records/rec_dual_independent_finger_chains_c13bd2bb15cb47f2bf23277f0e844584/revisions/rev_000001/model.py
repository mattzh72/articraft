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


PALM_WIDTH = 0.092
PALM_DEPTH = 0.050
PALM_HEIGHT = 0.020
PALM_PLATFORM_WIDTH = 0.060
PALM_PLATFORM_DEPTH = 0.034
PALM_PLATFORM_HEIGHT = 0.008
PALM_COVER_WIDTH = 0.054
PALM_COVER_DEPTH = 0.028
PALM_COVER_HEIGHT = 0.014

JOINT_Z = 0.014
BARREL_RADIUS = 0.0045
BARREL_HEIGHT = 0.010
PLATE_THICKNESS = 0.003
LINK_BODY_THICKNESS = 0.007
LINK_WIDTH_LEFT = 0.012
LINK_WIDTH_RIGHT = 0.011
CLEVIS_WIDTH = 0.016
CLEVIS_DEPTH = 0.012

LEFT_ROOT_X = -0.035
LEFT_ROOT_Y = 0.030
RIGHT_ROOT_X = 0.034
RIGHT_ROOT_Y = 0.028

LEFT_PROXIMAL_LENGTH = 0.038
LEFT_MIDDLE_LENGTH = 0.033
LEFT_DISTAL_LENGTH = 0.031
RIGHT_PROXIMAL_LENGTH = 0.027
RIGHT_MIDDLE_LENGTH = 0.021
RIGHT_DISTAL_LENGTH = 0.018

PAD_THICKNESS = 0.004
LEFT_PAD_SIZE = (PAD_THICKNESS, 0.012, 0.010)
RIGHT_PAD_SIZE = (PAD_THICKNESS, 0.010, 0.009)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cylinder(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XY").circle(radius).extrude(height, both=True).translate((x, y, z))


def _half_cylinder(
    radius: float,
    height: float,
    center: tuple[float, float, float],
    *,
    forward: bool,
) -> cq.Workplane:
    x, y, z = center
    body = _cylinder(radius, height, center)
    mask_center_y = y + radius * 0.5 if forward else y - radius * 0.5
    mask = _box((2.2 * radius, radius, height + 0.004), (x, mask_center_y, z))
    return body.intersect(mask)


def _support_block(x: float, y: float, width: float, plate_depth: float = 0.014) -> cq.Workplane:
    bridge = _box((width, 0.012, 0.022), (x, y - plate_depth + 0.004, 0.011))
    root_pad = _box((width * 0.9, 0.008, 0.008), (x, y - plate_depth * 0.55, 0.018))
    bottom_plate_z = JOINT_Z - BARREL_HEIGHT / 2.0 - PLATE_THICKNESS / 2.0
    top_plate_z = JOINT_Z + BARREL_HEIGHT / 2.0 + PLATE_THICKNESS / 2.0
    top_plate = _box((width, plate_depth, PLATE_THICKNESS), (x, y - plate_depth / 2.0, top_plate_z))
    bottom_plate = _box((width, plate_depth, PLATE_THICKNESS), (x, y - plate_depth / 2.0, bottom_plate_z))
    top_ear = _half_cylinder(width * 0.5, PLATE_THICKNESS, (x, y, top_plate_z), forward=False)
    bottom_ear = _half_cylinder(width * 0.5, PLATE_THICKNESS, (x, y, bottom_plate_z), forward=False)
    return bridge.union(root_pad).union(top_plate).union(bottom_plate).union(top_ear).union(bottom_ear)


def _make_palm_base() -> cq.Workplane:
    lower = _box((PALM_WIDTH, PALM_DEPTH, PALM_HEIGHT), (0.0, 0.0, 0.0)).edges("|Z").fillet(0.004)
    platform = _box(
        (PALM_PLATFORM_WIDTH, PALM_PLATFORM_DEPTH, PALM_PLATFORM_HEIGHT),
        (0.0, 0.0, 0.012),
    ).edges("|Z").fillet(0.003)
    underside_rail = _box((0.064, 0.018, 0.010), (0.0, 0.0, -0.010)).edges("|Z").fillet(0.002)
    left_block = _support_block(LEFT_ROOT_X, LEFT_ROOT_Y, 0.018)
    right_block = _support_block(RIGHT_ROOT_X, RIGHT_ROOT_Y, 0.017)
    waist_relief = _box((0.034, 0.022, 0.012), (0.0, 0.0, 0.004))
    return lower.union(platform).union(underside_rail).union(left_block).union(right_block).cut(waist_relief)


def _make_palm_cover() -> cq.Workplane:
    cover = cq.Workplane("XY").box(PALM_COVER_WIDTH, PALM_COVER_DEPTH, PALM_COVER_HEIGHT)
    cover = cover.edges("|Z").fillet(0.004)
    saddle_cut = (
        cq.Workplane("YZ")
        .center(0.0, 0.011)
        .circle(0.022)
        .extrude(PALM_COVER_WIDTH + 0.020, both=True)
    )
    front_break = _box((0.028, 0.008, 0.006), (0.0, 0.010, 0.003))
    return cover.cut(saddle_cut).cut(front_break)


def _forked_link(length: float, body_width: float, clevis_width: float) -> cq.Workplane:
    body_length = max(length - CLEVIS_DEPTH - BARREL_RADIUS, 0.010)
    root_lug = _half_cylinder(BARREL_RADIUS, BARREL_HEIGHT, (0.0, 0.0, 0.0), forward=True)
    beam = _box((body_width, body_length, LINK_BODY_THICKNESS), (0.0, BARREL_RADIUS + body_length / 2.0, 0.0))
    shoulder = _box((body_width * 0.92, 0.010, 0.010), (0.0, BARREL_RADIUS + 0.005, 0.0))
    bridge = _box(
        (clevis_width, 0.006, BARREL_HEIGHT + 2.0 * PLATE_THICKNESS),
        (0.0, length - CLEVIS_DEPTH + 0.003, 0.0),
    )
    top_plate = _box(
        (clevis_width, CLEVIS_DEPTH, PLATE_THICKNESS),
        (0.0, length - CLEVIS_DEPTH / 2.0, BARREL_HEIGHT / 2.0 + PLATE_THICKNESS / 2.0),
    )
    bottom_plate = _box(
        (clevis_width, CLEVIS_DEPTH, PLATE_THICKNESS),
        (0.0, length - CLEVIS_DEPTH / 2.0, -BARREL_HEIGHT / 2.0 - PLATE_THICKNESS / 2.0),
    )
    top_ear = _half_cylinder(
        clevis_width * 0.5,
        PLATE_THICKNESS,
        (0.0, length, BARREL_HEIGHT / 2.0 + PLATE_THICKNESS / 2.0),
        forward=False,
    )
    bottom_ear = _half_cylinder(
        clevis_width * 0.5,
        PLATE_THICKNESS,
        (0.0, length, -BARREL_HEIGHT / 2.0 - PLATE_THICKNESS / 2.0),
        forward=False,
    )
    return root_lug.union(beam).union(shoulder).union(bridge).union(top_plate).union(bottom_plate).union(top_ear).union(bottom_ear)


def _straight_terminal_link(length: float, body_width: float, tip_width: float) -> cq.Workplane:
    body_length = max(length - 0.015, 0.012)
    root_lug = _half_cylinder(BARREL_RADIUS, BARREL_HEIGHT, (0.0, 0.0, 0.0), forward=True)
    spine = _box((body_width, body_length, LINK_BODY_THICKNESS), (-0.001, BARREL_RADIUS + body_length / 2.0, 0.0))
    tip_blade = _box((tip_width * 0.34, 0.016, 0.007), (0.0005, length - 0.012, 0.0))
    inner_rib = _box((0.004, 0.012, 0.004), (0.0005, length - 0.014, -0.002))
    pad_rail = _box((0.003, 0.015, 0.010), (0.003, length - 0.0125, 0.0))
    rail_cap = _box((0.004, 0.007, 0.008), (0.0025, length - 0.0045, 0.0))
    return root_lug.union(spine).union(tip_blade).union(inner_rib).union(pad_rail).union(rail_cap)


def _hook_terminal_link(length: float, body_width: float) -> cq.Workplane:
    root_lug = _half_cylinder(BARREL_RADIUS, BARREL_HEIGHT, (0.0, 0.0, 0.0), forward=True)
    beam = _box((body_width, 0.013, LINK_BODY_THICKNESS), (0.0, 0.011, 0.0))
    lateral_bracket = _box((0.016, 0.010, 0.006), (-0.010, 0.014, -0.001))
    upper_gusset = _box((0.010, 0.008, 0.006), (-0.006, 0.010, 0.003))
    lower_gusset = _box((0.010, 0.008, 0.006), (-0.006, 0.010, -0.003))
    pad_seat = _box((0.006, 0.011, 0.008), (-0.015, 0.014, 0.0))
    hook_tip = (
        cq.Workplane("XY")
        .box(0.008, 0.012, 0.007)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 35.0)
        .translate((-0.0065, 0.019, 0.0))
    )
    tip_nose = _half_cylinder(0.004, 0.007, (-0.010, 0.024, 0.0), forward=False)
    return (
        root_lug.union(beam)
        .union(lateral_bracket)
        .union(upper_gusset)
        .union(lower_gusset)
        .union(pad_seat)
        .union(hook_tip)
        .union(tip_nose)
    )


def _make_pad(size: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    pad = cq.Workplane("XY").box(sx, sy, sz)
    return pad.edges("|Z").fillet(min(sx, sy, sz) * 0.2)


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
):
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=f"{name}_shell")
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robot_pinch_hand")

    model.material("palm_dark", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("cover_gray", rgba=(0.33, 0.36, 0.40, 1.0))
    model.material("machined_alloy", rgba=(0.75, 0.78, 0.81, 1.0))
    model.material("rubber_pad", rgba=(0.13, 0.13, 0.14, 1.0))

    palm_base = _add_mesh_part(
        model,
        name="palm_base",
        shape=_make_palm_base(),
        mesh_name="palm_base",
        material="palm_dark",
    )
    palm_cover = _add_mesh_part(
        model,
        name="palm_cover",
        shape=_make_palm_cover(),
        mesh_name="palm_cover",
        material="cover_gray",
    )

    left_proximal = _add_mesh_part(
        model,
        name="left_proximal",
        shape=_forked_link(LEFT_PROXIMAL_LENGTH, LINK_WIDTH_LEFT, CLEVIS_WIDTH),
        mesh_name="left_proximal",
        material="machined_alloy",
    )
    left_middle = _add_mesh_part(
        model,
        name="left_middle",
        shape=_forked_link(LEFT_MIDDLE_LENGTH, 0.011, 0.015),
        mesh_name="left_middle",
        material="machined_alloy",
    )
    left_distal = _add_mesh_part(
        model,
        name="left_distal",
        shape=_straight_terminal_link(LEFT_DISTAL_LENGTH, 0.0105, 0.014),
        mesh_name="left_distal",
        material="machined_alloy",
    )

    right_proximal = _add_mesh_part(
        model,
        name="right_proximal",
        shape=_forked_link(RIGHT_PROXIMAL_LENGTH, LINK_WIDTH_RIGHT, 0.015),
        mesh_name="right_proximal",
        material="machined_alloy",
    )
    right_middle = _add_mesh_part(
        model,
        name="right_middle",
        shape=_forked_link(RIGHT_MIDDLE_LENGTH, 0.010, 0.014),
        mesh_name="right_middle",
        material="machined_alloy",
    )
    right_distal = _add_mesh_part(
        model,
        name="right_distal",
        shape=_hook_terminal_link(RIGHT_DISTAL_LENGTH, 0.0095),
        mesh_name="right_distal",
        material="machined_alloy",
    )

    left_pad = _add_mesh_part(
        model,
        name="left_pad",
        shape=_make_pad(LEFT_PAD_SIZE),
        mesh_name="left_pad",
        material="rubber_pad",
    )
    right_pad = _add_mesh_part(
        model,
        name="right_pad",
        shape=_make_pad(RIGHT_PAD_SIZE),
        mesh_name="right_pad",
        material="rubber_pad",
    )

    model.articulation(
        "palm_to_cover",
        ArticulationType.FIXED,
        parent=palm_base,
        child=palm_cover,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm_base,
        child=left_proximal,
        origin=Origin(xyz=(LEFT_ROOT_X, LEFT_ROOT_Y, JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=0.0, upper=0.75),
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, LEFT_PROXIMAL_LENGTH, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, LEFT_MIDDLE_LENGTH, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0, lower=0.0, upper=0.95),
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm_base,
        child=right_proximal,
        origin=Origin(xyz=(RIGHT_ROOT_X, RIGHT_ROOT_Y, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=0.0, upper=0.75),
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, RIGHT_PROXIMAL_LENGTH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, RIGHT_MIDDLE_LENGTH, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0, lower=0.0, upper=0.95),
    )

    model.articulation(
        "left_distal_to_pad",
        ArticulationType.FIXED,
        parent=left_distal,
        child=left_pad,
        origin=Origin(xyz=(0.0065, LEFT_DISTAL_LENGTH - 0.0125, 0.0)),
    )
    model.articulation(
        "right_distal_to_pad",
        ArticulationType.FIXED,
        parent=right_distal,
        child=right_pad,
        origin=Origin(xyz=(-0.020, 0.014, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm_base = object_model.get_part("palm_base")
    palm_cover = object_model.get_part("palm_cover")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")
    left_pad = object_model.get_part("left_pad")
    right_pad = object_model.get_part("right_pad")

    left_root = object_model.get_articulation("palm_to_left_proximal")
    left_mid = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip = object_model.get_articulation("left_middle_to_left_distal")
    right_root = object_model.get_articulation("palm_to_right_proximal")
    right_mid = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip = object_model.get_articulation("right_middle_to_right_distal")

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

    ctx.expect_contact(palm_cover, palm_base, name="cover_seats_on_palm")
    ctx.expect_contact(left_proximal, palm_base, name="left_root_knuckle_is_supported")
    ctx.expect_contact(left_middle, left_proximal, name="left_middle_is_pinned")
    ctx.expect_contact(left_distal, left_middle, name="left_distal_is_pinned")
    ctx.expect_contact(right_proximal, palm_base, name="right_root_knuckle_is_supported")
    ctx.expect_contact(right_middle, right_proximal, name="right_middle_is_pinned")
    ctx.expect_contact(right_distal, right_middle, name="right_distal_is_pinned")
    ctx.expect_contact(left_pad, left_distal, name="left_pad_is_mounted")
    ctx.expect_contact(right_pad, right_distal, name="right_pad_is_mounted")

    ctx.allow_overlap(
        left_distal,
        left_middle,
        reason="Interleaved clevis plates and root knuckle simplify the captured distal hinge volume.",
    )
    ctx.allow_overlap(
        left_middle,
        left_proximal,
        reason="Interleaved clevis plates and root knuckle simplify the proximal captured hinge volume under curl.",
    )
    ctx.allow_overlap(
        right_distal,
        right_middle,
        reason="Interleaved clevis plates and root knuckle simplify the captured hooked-finger hinge volume.",
    )
    ctx.allow_overlap(
        right_middle,
        right_proximal,
        reason="Interleaved clevis plates and root knuckle simplify the proximal captured hinge volume under curl.",
    )
    ctx.allow_overlap(
        left_distal,
        left_pad,
        reason="Replaceable left fingertip pad is modeled as a seated insert in a shallow machined pocket on the distal jaw.",
    )

    ctx.expect_gap(right_pad, left_pad, axis="x", min_gap=0.030, max_gap=0.085, name="open_jaws_have_working_gap")
    ctx.expect_overlap(left_pad, right_pad, axes="z", min_overlap=0.006, name="pads_share_vertical_working_band")

    left_aabb = ctx.part_world_aabb(left_distal)
    right_aabb = ctx.part_world_aabb(right_distal)
    if left_aabb is None or right_aabb is None:
        ctx.fail("finger_reach_asymmetry", "missing distal link bounds")
    else:
        left_reach = left_aabb[1][1]
        right_reach = right_aabb[1][1]
        ctx.check(
            "left_finger_reaches_farther_than_right",
            left_reach > right_reach + 0.004,
            f"left reach {left_reach:.4f} m, right reach {right_reach:.4f} m",
        )

    pinch_pose = {
        left_root: 0.18,
        left_mid: 0.18,
        left_tip: 0.14,
        right_root: 0.0,
        right_mid: 0.0,
        right_tip: 0.0,
    }
    with ctx.pose(pinch_pose):
        ctx.expect_gap(
            right_pad,
            left_pad,
            axis="x",
            min_gap=0.0,
            max_gap=0.012,
            name="pinch_pose_brings_pads_together",
        )
        ctx.expect_gap(
            left_pad,
            right_pad,
            axis="y",
            min_gap=0.0,
            max_gap=0.016,
            name="pinch_pose_keeps_pads_in_the_same_work_zone",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="pinch_pose_no_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
