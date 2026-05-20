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


BODY_LEN = 0.170
BODY_W = 0.072
BASE_T = 0.010
GUIDE_LEN = 0.126
GUIDE_W = 0.028
GUIDE_T = 0.003
GUIDE_CENTER_Y = 0.020
RIB_LEN = 0.142
RIB_W = 0.012
RIB_T = 0.010

SLIDE_LEN = 0.112
SLIDE_W = 0.020
SLIDE_T = 0.009
SLIDE_OPEN_Y = 0.025
SLIDE_TRAVEL = 0.010

JAW_BASE_L = 0.026
JAW_BASE_W = 0.016
JAW_BASE_T = 0.004
JAW_FINGER_L = 0.020
JAW_FINGER_W = 0.006
JAW_FINGER_H = 0.013
JAW_FINGER_OFFSET_Y = 0.005
JAW_X_OFFSET = 0.023


def _floor_box(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False))


def _spine_shape() -> cq.Workplane:
    base = _floor_box(BODY_LEN, BODY_W, BASE_T)
    left_guide = _floor_box(GUIDE_LEN, GUIDE_W, GUIDE_T).translate(
        (0.0, -GUIDE_CENTER_Y, BASE_T)
    )
    right_guide = _floor_box(GUIDE_LEN, GUIDE_W, GUIDE_T).translate(
        (0.0, GUIDE_CENTER_Y, BASE_T)
    )
    center_rib = _floor_box(RIB_LEN, RIB_W, RIB_T).translate((0.0, 0.0, BASE_T))
    front_bridge = _floor_box(0.034, 0.020, 0.004).translate((0.050, 0.0, BASE_T))
    rear_pad = _floor_box(0.034, 0.048, 0.003).translate((-0.064, 0.0, BASE_T))

    shape = base.union(left_guide).union(right_guide).union(center_rib).union(front_bridge).union(
        rear_pad
    )

    rib_slot = _floor_box(0.088, 0.005, 0.003).translate((0.000, 0.0, BASE_T + RIB_T - 0.003))
    mount_pockets = (
        cq.Workplane("XY")
        .pushPoints([(-0.048, -0.022), (-0.048, 0.022)])
        .circle(0.0045)
        .extrude(0.002)
        .translate((0.0, 0.0, BASE_T + 0.001))
    )

    return shape.cut(rib_slot).cut(mount_pockets)


def _slide_shape(side_sign: float) -> cq.Workplane:
    base = _floor_box(SLIDE_LEN, SLIDE_W, SLIDE_T)
    front_pad = _floor_box(0.014, SLIDE_W, 0.002).translate((0.046, 0.0, SLIDE_T))
    rear_tab = _floor_box(0.016, 0.018, 0.002).translate((-0.045, 0.0, SLIDE_T))
    outer_rib = _floor_box(0.050, 0.004, 0.003).translate((-0.016, side_sign * 0.0065, SLIDE_T))
    top_pocket = _floor_box(0.070, 0.009, 0.0025).translate((-0.004, 0.0, SLIDE_T - 0.0025))

    return base.union(front_pad).union(rear_tab).union(outer_rib).cut(top_pocket)


def _jaw_shape(side_sign: float) -> cq.Workplane:
    base = _floor_box(JAW_BASE_L, JAW_BASE_W, JAW_BASE_T)
    finger = _floor_box(JAW_FINGER_L, JAW_FINGER_W, JAW_FINGER_H).translate(
        (0.004, side_sign * JAW_FINGER_OFFSET_Y, JAW_BASE_T)
    )
    brace = _floor_box(0.012, 0.010, 0.006).translate((-0.006, side_sign * 0.003, JAW_BASE_T))
    return base.union(finger).union(brace)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_slide_gripper")

    spine_mat = model.material("spine_anodized", rgba=(0.29, 0.31, 0.34, 1.0))
    slide_mat = model.material("slide_satin", rgba=(0.63, 0.65, 0.68, 1.0))
    jaw_mat = model.material("jaw_blackened", rgba=(0.14, 0.15, 0.17, 1.0))

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(_spine_shape(), "center_spine"),
        origin=Origin(),
        material=spine_mat,
        name="spine_shell",
    )

    left_slide = model.part("left_slide")
    left_slide.visual(
        mesh_from_cadquery(_slide_shape(-1.0), "left_slide"),
        origin=Origin(),
        material=slide_mat,
        name="slide_shell",
    )

    right_slide = model.part("right_slide")
    right_slide.visual(
        mesh_from_cadquery(_slide_shape(1.0), "right_slide"),
        origin=Origin(),
        material=slide_mat,
        name="slide_shell",
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(_jaw_shape(1.0), "left_jaw"),
        origin=Origin(),
        material=jaw_mat,
        name="jaw_shell",
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(_jaw_shape(-1.0), "right_jaw"),
        origin=Origin(),
        material=jaw_mat,
        name="jaw_shell",
    )

    model.articulation(
        "spine_to_left_slide",
        ArticulationType.PRISMATIC,
        parent=spine,
        child=left_slide,
        origin=Origin(xyz=(0.0, -SLIDE_OPEN_Y, BASE_T + GUIDE_T)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.10,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "spine_to_right_slide",
        ArticulationType.PRISMATIC,
        parent=spine,
        child=right_slide,
        origin=Origin(xyz=(0.0, SLIDE_OPEN_Y, BASE_T + GUIDE_T)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.10,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "left_slide_to_left_jaw",
        ArticulationType.FIXED,
        parent=left_slide,
        child=left_jaw,
        origin=Origin(xyz=(JAW_X_OFFSET, 0.0, SLIDE_T)),
    )
    model.articulation(
        "right_slide_to_right_jaw",
        ArticulationType.FIXED,
        parent=right_slide,
        child=right_jaw,
        origin=Origin(xyz=(JAW_X_OFFSET, 0.0, SLIDE_T)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    left_slide = object_model.get_part("left_slide")
    right_slide = object_model.get_part("right_slide")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_prismatic = object_model.get_articulation("spine_to_left_slide")
    right_prismatic = object_model.get_articulation("spine_to_right_slide")

    left_upper = left_prismatic.motion_limits.upper if left_prismatic.motion_limits else 0.0
    right_upper = right_prismatic.motion_limits.upper if right_prismatic.motion_limits else 0.0

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

    ctx.expect_contact(left_slide, spine, contact_tol=0.0005, name="left slide supported at open")
    ctx.expect_contact(right_slide, spine, contact_tol=0.0005, name="right slide supported at open")
    ctx.expect_contact(left_jaw, left_slide, contact_tol=0.0005, name="left jaw mounted to slide")
    ctx.expect_contact(right_jaw, right_slide, contact_tol=0.0005, name="right jaw mounted to slide")

    with ctx.pose({left_prismatic: 0.0, right_prismatic: 0.0}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="y",
            min_gap=0.032,
            max_gap=0.036,
            name="open jaw gap",
        )

        left_open_y = ctx.part_world_position(left_slide)[1]
        right_open_y = ctx.part_world_position(right_slide)[1]

    with ctx.pose({left_prismatic: left_upper, right_prismatic: right_upper}):
        ctx.expect_contact(
            left_slide,
            spine,
            contact_tol=0.0005,
            name="left slide stays supported when closed",
        )
        ctx.expect_contact(
            right_slide,
            spine,
            contact_tol=0.0005,
            name="right slide stays supported when closed",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="y",
            min_gap=0.012,
            max_gap=0.016,
            name="closed jaw work area width",
        )
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="xz",
            min_overlap=0.015,
            name="closed jaws define shared rectangular work area",
        )

        left_closed_y = ctx.part_world_position(left_slide)[1]
        right_closed_y = ctx.part_world_position(right_slide)[1]

    ctx.check(
        "left slide closes inward",
        left_closed_y > left_open_y + 0.009,
        details=f"left slide y did not advance inward enough: open={left_open_y:.4f}, closed={left_closed_y:.4f}",
    )
    ctx.check(
        "right slide closes inward",
        right_closed_y < right_open_y - 0.009,
        details=f"right slide y did not advance inward enough: open={right_open_y:.4f}, closed={right_closed_y:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
