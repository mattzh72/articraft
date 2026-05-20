from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.26
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.024
PEDESTAL_FORK_WIDTH = 0.064
SHOULDER_Z = 0.145

LINK1_LENGTH = 0.265
LINK2_LENGTH = 0.225
LINK3_LENGTH = 0.175

LINK1_HEIGHT = 0.068
LINK2_HEIGHT = 0.058
LINK3_HEIGHT = 0.050

LINK1_WIDTH = 0.058
LINK2_WIDTH = 0.052
LINK3_WIDTH = 0.046

LINK1_TONGUE_WIDTH = 0.028
LINK2_TONGUE_WIDTH = 0.024
LINK3_TONGUE_WIDTH = 0.020

END_TAB_LENGTH = 0.032
END_TAB_WIDTH = 0.018
END_TAB_HEIGHT = 0.024

JOINT_YOKE_LEN = 0.030
JOINT_BACKSET = 0.010
JOINT_FRONT_1 = 0.022
JOINT_FRONT_2 = 0.020
JOINT_FRONT_3 = 0.018

LINK1_YOKE_SLOT = LINK2_TONGUE_WIDTH
LINK2_YOKE_SLOT = LINK3_TONGUE_WIDTH


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_open_link_visuals(
    part,
    *,
    name_prefix: str,
    material: str,
    length: float,
    outer_h: float,
    total_w: float,
    tongue_w: float,
    tongue_front: float,
    distal_slot_w: float | None,
) -> None:
    rail_w = max((total_w - tongue_w) / 2.0, 0.010)
    rail_offset = total_w / 2.0 - rail_w / 2.0
    tongue_len = JOINT_BACKSET + tongue_front
    center_beam_len = tongue_front + 0.030
    rail_start = center_beam_len
    distal_len = JOINT_YOKE_LEN if distal_slot_w is not None else 0.028
    rail_end = length - distal_len
    rail_len = max(rail_end - rail_start, 0.020)

    _add_box_visual(
        part,
        size=(tongue_len, tongue_w, outer_h * 0.74),
        center=((tongue_front - JOINT_BACKSET) / 2.0, 0.0, 0.0),
        material=material,
        name=f"{name_prefix}_tongue",
    )
    _add_box_visual(
        part,
        size=(center_beam_len, tongue_w, outer_h * 0.16),
        center=(center_beam_len / 2.0, 0.0, 0.0),
        material=material,
        name=f"{name_prefix}_prox_web",
    )

    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        _add_box_visual(
            part,
            size=(rail_len, rail_w, outer_h),
            center=(rail_start + rail_len / 2.0, sign * rail_offset, 0.0),
            material=material,
            name=f"{name_prefix}_rail_{side_name}",
        )

    _add_box_visual(
        part,
        size=(0.022, total_w * 0.76, outer_h * 0.18),
        center=(rail_start + rail_len * 0.46, 0.0, 0.0),
        material=material,
        name=f"{name_prefix}_mid_brace",
    )

    if distal_slot_w is not None:
        cheek_w = (total_w - distal_slot_w) / 2.0
        cheek_offset = distal_slot_w / 2.0 + cheek_w / 2.0
        for sign, side_name in ((-1.0, "left"), (1.0, "right")):
            _add_box_visual(
                part,
                size=(JOINT_YOKE_LEN, cheek_w, outer_h * 0.84),
                center=(length - JOINT_YOKE_LEN / 2.0, sign * cheek_offset, 0.0),
                material=material,
                name=f"{name_prefix}_yoke_{side_name}",
            )
        _add_box_visual(
            part,
            size=(0.016, total_w * 0.80, outer_h * 0.18),
            center=(length - JOINT_YOKE_LEN - 0.008, 0.0, 0.0),
            material=material,
            name=f"{name_prefix}_distal_brace",
        )
    else:
        _add_box_visual(
            part,
            size=(0.028, total_w * 0.72, outer_h * 0.72),
            center=(length - 0.014, 0.0, 0.0),
            material=material,
            name=f"{name_prefix}_tip_block",
        )


def _add_pedestal_visuals(part, *, material: str) -> None:
    _add_box_visual(
        part,
        size=(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS),
        center=(0.0, 0.0, BASE_THICKNESS / 2.0),
        material=material,
        name="pedestal_base",
    )
    mast_height = SHOULDER_Z - BASE_THICKNESS + 0.012
    _add_box_visual(
        part,
        size=(0.090, 0.080, mast_height),
        center=(-0.072, 0.0, BASE_THICKNESS + mast_height / 2.0 - 0.006),
        material=material,
        name="pedestal_mast",
    )
    _add_box_visual(
        part,
        size=(0.040, 0.028, 0.110),
        center=(-0.040, 0.0, 0.082),
        material=material,
        name="pedestal_spine",
    )
    _add_box_visual(
        part,
        size=(0.016, PEDESTAL_FORK_WIDTH, 0.020),
        center=(-0.034, 0.0, SHOULDER_Z - 0.026),
        material=material,
        name="pedestal_fork_bridge",
    )

    cheek_w = (PEDESTAL_FORK_WIDTH - LINK1_TONGUE_WIDTH) / 2.0
    cheek_offset = LINK1_TONGUE_WIDTH / 2.0 + cheek_w / 2.0
    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        _add_box_visual(
            part,
            size=(JOINT_YOKE_LEN, cheek_w, LINK1_HEIGHT * 0.86),
            center=(-JOINT_YOKE_LEN / 2.0, sign * cheek_offset, SHOULDER_Z),
            material=material,
            name=f"pedestal_cheek_{side_name}",
        )


def _aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple(upper - lower for lower, upper in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_revolute_chain")

    model.material("pedestal_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("link_dark", rgba=(0.46, 0.49, 0.52, 1.0))
    model.material("link_mid", rgba=(0.56, 0.59, 0.62, 1.0))
    model.material("link_light", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("tab_black", rgba=(0.12, 0.13, 0.14, 1.0))

    pedestal = model.part("pedestal")
    _add_pedestal_visuals(
        pedestal,
        material="pedestal_gray",
    )

    link1 = model.part("link1")
    _add_open_link_visuals(
        link1,
        name_prefix="link1",
        material="link_dark",
        length=LINK1_LENGTH,
        outer_h=LINK1_HEIGHT,
        total_w=LINK1_WIDTH,
        tongue_w=LINK1_TONGUE_WIDTH,
        tongue_front=JOINT_FRONT_1,
        distal_slot_w=LINK1_YOKE_SLOT,
    )

    link2 = model.part("link2")
    _add_open_link_visuals(
        link2,
        name_prefix="link2",
        material="link_mid",
        length=LINK2_LENGTH,
        outer_h=LINK2_HEIGHT,
        total_w=LINK2_WIDTH,
        tongue_w=LINK2_TONGUE_WIDTH,
        tongue_front=JOINT_FRONT_2,
        distal_slot_w=LINK2_YOKE_SLOT,
    )

    link3 = model.part("link3")
    _add_open_link_visuals(
        link3,
        name_prefix="link3",
        material="link_light",
        length=LINK3_LENGTH,
        outer_h=LINK3_HEIGHT,
        total_w=LINK3_WIDTH,
        tongue_w=LINK3_TONGUE_WIDTH,
        tongue_front=JOINT_FRONT_3,
        distal_slot_w=None,
    )
    link3.visual(
        Box((END_TAB_LENGTH, END_TAB_WIDTH, END_TAB_HEIGHT)),
        origin=Origin(xyz=(LINK3_LENGTH + END_TAB_LENGTH / 2.0, 0.0, 0.0)),
        material="tab_black",
        name="end_tab",
    )

    model.articulation(
        "pedestal_to_link1",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=-0.55,
            upper=1.25,
        ),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.35,
        ),
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.20,
            upper=1.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")

    shoulder = object_model.get_articulation("pedestal_to_link1")
    elbow = object_model.get_articulation("link1_to_link2")
    wrist = object_model.get_articulation("link2_to_link3")

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

    ctx.expect_contact(link1, pedestal, name="pedestal_supports_link1")
    ctx.expect_contact(link2, link1, name="link1_supports_link2")
    ctx.expect_contact(link3, link2, name="link2_supports_link3")

    link1_dims = _aabb_dims(ctx.part_world_aabb(link1))
    link2_dims = _aabb_dims(ctx.part_world_aabb(link2))
    link3_dims = _aabb_dims(ctx.part_world_aabb(link3))
    dims_step_down = (
        link1_dims is not None
        and link2_dims is not None
        and link3_dims is not None
        and link1_dims[0] > link2_dims[0] > link3_dims[0]
        and link1_dims[1] > link2_dims[1] > link3_dims[1]
    )
    ctx.check(
        "links_step_down_toward_tip",
        dims_step_down,
        details=f"link dims: {link1_dims}, {link2_dims}, {link3_dims}",
    )

    axes_parallel = shoulder.axis == elbow.axis == wrist.axis
    ctx.check(
        "joint_axes_parallel",
        axes_parallel,
        details=f"axes: {shoulder.axis}, {elbow.axis}, {wrist.axis}",
    )

    rest_tip = ctx.part_element_world_aabb(link3, elem="end_tab")
    with ctx.pose({shoulder: 0.70, elbow: 0.55, wrist: 0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_flexed_pose")
        flex_tip = ctx.part_element_world_aabb(link3, elem="end_tab")

    tip_lifts = False
    if rest_tip is not None and flex_tip is not None:
        rest_tip_z = (rest_tip[0][2] + rest_tip[1][2]) / 2.0
        flex_tip_z = (flex_tip[0][2] + flex_tip[1][2]) / 2.0
        tip_lifts = flex_tip_z > rest_tip_z + 0.10
    else:
        rest_tip_z = None
        flex_tip_z = None
    ctx.check(
        "end_tab_lifts_when_chain_flexes",
        tip_lifts,
        details=f"rest_tip_z={rest_tip_z}, flex_tip_z={flex_tip_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
