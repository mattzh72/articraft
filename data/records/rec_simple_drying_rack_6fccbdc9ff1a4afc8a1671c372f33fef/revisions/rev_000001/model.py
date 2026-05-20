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


TUBE = 0.022
RAIL_Y = 0.014
RAIL_Z = 0.008
BODY_W = 0.86
DEPTH = 0.54
PLANE_Z = 0.82
SKID_X = 0.038
SKID_Z = 0.022
WING_W = 0.34
HINGE_LINK_LEN = 0.04
HINGE_CLEAR_Z = 0.018


def add_box(
    part,
    size,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder(
    part,
    radius,
    length,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_body(part, frame_mat, rail_mat, seal_mat):
    x_side = BODY_W / 2.0 - TUBE / 2.0
    y_side = DEPTH / 2.0 - TUBE / 2.0
    post_center_z = PLANE_Z / 2.0 + SKID_Z / 2.0

    # Four upright legs in corrosion-resistant square tube.
    for x in (-x_side, x_side):
        for y in (-y_side, y_side):
            add_box(
                part,
                (TUBE, TUBE, PLANE_Z),
                (x, y, post_center_z),
                material=frame_mat,
            )
            add_box(
                part,
                (0.034, 0.034, 0.010),
                (x, y, PLANE_Z + 0.014),
                material=seal_mat,
            )

    # Ground skids give the freestanding rack a broad weather-tolerant stance.
    for x in (-x_side, x_side):
        add_box(
            part,
            (SKID_X, DEPTH, SKID_Z),
            (x, 0.0, SKID_Z / 2.0),
            material=frame_mat,
        )

    # Lower tie rails keep the skids working as one grounded assembly.
    lower_span = BODY_W - 2.0 * TUBE + 0.004
    for y in (-0.18, 0.18):
        add_box(
            part,
            (lower_span, TUBE, TUBE),
            (0.0, y, SKID_Z + TUBE / 2.0),
            material=frame_mat,
        )

    # Top perimeter frame around the main drying deck.
    add_box(
        part,
        (BODY_W, TUBE, TUBE),
        (0.0, y_side, PLANE_Z),
        material=frame_mat,
        name="front_top_rail",
    )
    add_box(
        part,
        (BODY_W, TUBE, TUBE),
        (0.0, -y_side, PLANE_Z),
        material=frame_mat,
        name="rear_top_rail",
    )
    add_box(
        part,
        (TUBE, DEPTH - 2.0 * TUBE + 0.004, TUBE),
        (-x_side, 0.0, PLANE_Z),
        material=frame_mat,
        name="left_top_side",
    )
    add_box(
        part,
        (TUBE, DEPTH - 2.0 * TUBE + 0.004, TUBE),
        (x_side, 0.0, PLANE_Z),
        material=frame_mat,
        name="right_top_side",
    )

    # Main hanging rails; slightly inset and sealed by the perimeter frame.
    rail_span = BODY_W - 2.0 * TUBE + 0.004
    for idx, y in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18), start=1):
        add_box(
            part,
            (rail_span, RAIL_Y, RAIL_Z),
            (0.0, y, PLANE_Z),
            material=rail_mat,
            name=f"center_rail_{idx}",
        )


def build_hinge_link(part, side_sign, frame_mat, hardware_mat, seal_mat):
    x_dir = float(side_sign)
    # Back plate seals to the body side rail without penetration.
    add_box(
        part,
        (0.024, 0.50, 0.020),
        (x_dir * 0.012, 0.0, 0.0),
        material=frame_mat,
        name="seal_plate",
    )
    # End posts hold a hood over the hinge sweep so rain sheds away from the joint.
    for y in (-0.275, 0.275):
        add_box(
            part,
            (0.020, 0.050, 0.036),
            (x_dir * 0.012, y, 0.0),
            material=seal_mat,
        )
    add_box(
        part,
        (0.052, 0.60, 0.014),
        (x_dir * 0.026, 0.0, 0.025),
        material=seal_mat,
        name="hood",
    )
    add_cylinder(
        part,
        radius=0.008,
        length=0.52,
        xyz=(x_dir * 0.030, 0.0, HINGE_CLEAR_Z + 0.001),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=hardware_mat,
        name="pin_housing",
    )
    add_box(
        part,
        (0.018, 0.18, 0.016),
        (x_dir * 0.030, 0.0, -0.018),
        material=frame_mat,
        name="stop_lug",
    )


def build_wing(part, side_sign, frame_mat, rail_mat, seal_mat):
    x_dir = float(side_sign)
    y_side = DEPTH / 2.0 - TUBE / 2.0
    wing_center_x = x_dir * (WING_W / 2.0)
    outer_upright_x = x_dir * (WING_W - TUBE / 2.0)
    rail_span = WING_W - 2.0 * TUBE + 0.004

    add_box(
        part,
        (TUBE, DEPTH, TUBE),
        (x_dir * (TUBE / 2.0), 0.0, 0.0),
        material=frame_mat,
        name="hinge_stile",
    )
    add_box(
        part,
        (TUBE, DEPTH, TUBE),
        (outer_upright_x, 0.0, 0.0),
        material=frame_mat,
        name="outer_stile",
    )
    for name, y in (("front_rail", y_side), ("rear_rail", -y_side)):
        add_box(
            part,
            (WING_W, TUBE, TUBE),
            (wing_center_x, y, 0.0),
            material=frame_mat,
            name=name,
        )
    for idx, y in enumerate((-0.14, 0.0, 0.14), start=1):
        add_box(
            part,
            (rail_span, RAIL_Y, RAIL_Z),
            (wing_center_x, y, 0.0),
            material=rail_mat,
            name=f"wing_rail_{idx}",
        )
    # A capped outer edge keeps water from sitting on the open tube ends.
    add_box(
        part,
        (0.050, DEPTH + 0.030, 0.010),
        (x_dir * (WING_W + 0.021), 0.0, 0.016),
        material=seal_mat,
        name="outer_drip_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_wing_drying_rack")

    frame_mat = model.material("powder_coated_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    rail_mat = model.material("anodized_hanging_rail", rgba=(0.67, 0.69, 0.72, 1.0))
    hardware_mat = model.material("stainless_hardware", rgba=(0.54, 0.56, 0.58, 1.0))
    seal_mat = model.material("sealed_polymer", rgba=(0.16, 0.17, 0.18, 1.0))

    rack_frame = model.part("rack_frame")
    left_hinge_link = model.part("left_hinge_link")
    right_hinge_link = model.part("right_hinge_link")
    left_wing_frame = model.part("left_wing_frame")
    right_wing_frame = model.part("right_wing_frame")

    build_body(rack_frame, frame_mat, rail_mat, seal_mat)
    build_hinge_link(left_hinge_link, side_sign=-1, frame_mat=frame_mat, hardware_mat=hardware_mat, seal_mat=seal_mat)
    build_hinge_link(right_hinge_link, side_sign=1, frame_mat=frame_mat, hardware_mat=hardware_mat, seal_mat=seal_mat)
    build_wing(left_wing_frame, side_sign=-1, frame_mat=frame_mat, rail_mat=rail_mat, seal_mat=seal_mat)
    build_wing(right_wing_frame, side_sign=1, frame_mat=frame_mat, rail_mat=rail_mat, seal_mat=seal_mat)

    model.articulation(
        "frame_to_left_hinge",
        ArticulationType.FIXED,
        parent=rack_frame,
        child=left_hinge_link,
        origin=Origin(xyz=(-BODY_W / 2.0, 0.0, PLANE_Z)),
    )
    model.articulation(
        "frame_to_right_hinge",
        ArticulationType.FIXED,
        parent=rack_frame,
        child=right_hinge_link,
        origin=Origin(xyz=(BODY_W / 2.0, 0.0, PLANE_Z)),
    )
    model.articulation(
        "left_wing_fold",
        ArticulationType.REVOLUTE,
        parent=left_hinge_link,
        child=left_wing_frame,
        origin=Origin(xyz=(-HINGE_LINK_LEN + 0.010, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "right_wing_fold",
        ArticulationType.REVOLUTE,
        parent=right_hinge_link,
        child=right_wing_frame,
        origin=Origin(xyz=(HINGE_LINK_LEN - 0.010, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rack_frame = object_model.get_part("rack_frame")
    left_hinge_link = object_model.get_part("left_hinge_link")
    right_hinge_link = object_model.get_part("right_hinge_link")
    left_wing_frame = object_model.get_part("left_wing_frame")
    right_wing_frame = object_model.get_part("right_wing_frame")
    left_wing_fold = object_model.get_articulation("left_wing_fold")
    right_wing_fold = object_model.get_articulation("right_wing_fold")

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

    ctx.expect_contact(left_hinge_link, rack_frame, name="left_hinge_link_sealed_to_frame")
    ctx.expect_contact(right_hinge_link, rack_frame, name="right_hinge_link_sealed_to_frame")

    with ctx.pose({left_wing_fold: 0.0, right_wing_fold: 0.0}):
        ctx.expect_contact(left_wing_frame, left_hinge_link, name="left_wing_supported_at_hinge")
        ctx.expect_contact(right_wing_frame, right_hinge_link, name="right_wing_supported_at_hinge")
        rack_aabb = ctx.part_world_aabb(rack_frame)
        left_aabb = ctx.part_world_aabb(left_wing_frame)
        right_aabb = ctx.part_world_aabb(right_wing_frame)
        left_gap_ok = (
            rack_aabb is not None
            and left_aabb is not None
            and 0.0 <= rack_aabb[0][0] - left_aabb[1][0] <= 0.03
        )
        right_gap_ok = (
            rack_aabb is not None
            and right_aabb is not None
            and 0.0 <= right_aabb[0][0] - rack_aabb[1][0] <= 0.03
        )
        ctx.check(
            "left_wing_closes_outboard_of_frame",
            left_gap_ok,
            "Left wing should park just outside the main rack without crossing the hinge line.",
        )
        ctx.check(
            "right_wing_closes_outboard_of_frame",
            right_gap_ok,
            "Right wing should park just outside the main rack without crossing the hinge line.",
        )

    left_limits = left_wing_fold.motion_limits
    right_limits = right_wing_fold.motion_limits
    limits_ok = (
        left_limits is not None
        and right_limits is not None
        and left_limits.lower == 0.0
        and right_limits.lower == 0.0
        and left_limits.upper is not None
        and right_limits.upper is not None
        and 1.0 <= left_limits.upper <= 1.2
        and 1.0 <= right_limits.upper <= 1.2
    )
    ctx.check(
        "wing_stop_positions_defined",
        limits_ok,
        "Both wings should have positive folding travel with explicit stop limits.",
    )

    with ctx.pose({left_wing_fold: 1.0, right_wing_fold: 1.0}):
        rack_aabb = ctx.part_world_aabb(rack_frame)
        left_cap_aabb = ctx.part_element_world_aabb(left_wing_frame, elem="outer_drip_cap")
        right_cap_aabb = ctx.part_element_world_aabb(right_wing_frame, elem="outer_drip_cap")
        left_ok = (
            rack_aabb is not None
            and left_cap_aabb is not None
            and left_cap_aabb[0][2] > rack_aabb[1][2] + 0.18
        )
        right_ok = (
            rack_aabb is not None
            and right_cap_aabb is not None
            and right_cap_aabb[0][2] > rack_aabb[1][2] + 0.18
        )
        ctx.check(
            "left_wing_folds_upward",
            left_ok,
            "Left wing outer drip cap should rise well above the main rack at the open stop.",
        )
        ctx.check(
            "right_wing_folds_upward",
            right_ok,
            "Right wing outer drip cap should rise well above the main rack at the open stop.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
