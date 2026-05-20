from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


ROOT_LENGTH = 0.52
ROOT_WIDTH = 0.160
ROOT_HEIGHT = 0.140
ROOT_INNER_WIDTH = 0.124
ROOT_INNER_HEIGHT = 0.104
ROOT_REAR_PLATE = 0.016
ROOT_FRONT_RING = 0.060
ROOT_FRONT_OUTER_WIDTH = 0.172
ROOT_FRONT_OUTER_HEIGHT = 0.150
ROOT_FRONT_OPENING_WIDTH = 0.126
ROOT_FRONT_OPENING_HEIGHT = 0.106
ROOT_FOOT_LENGTH = 0.170
ROOT_FOOT_WIDTH = 0.184
ROOT_FOOT_HEIGHT = 0.030

MID_LENGTH = 0.56
MID_BODY_WIDTH = 0.108
MID_BODY_HEIGHT = 0.082
MID_INNER_WIDTH = 0.090
MID_INNER_HEIGHT = 0.064
MID_REAR_PLATE = 0.014
MID_REAR_CAP_LENGTH = 0.036
MID_REAR_CAP_WIDTH = 0.116
MID_REAR_CAP_HEIGHT = 0.098
MID_FRONT_RING = 0.050
MID_FRONT_OUTER_WIDTH = 0.122
MID_FRONT_OUTER_HEIGHT = 0.092
MID_FRONT_OPENING_WIDTH = 0.092
MID_FRONT_OPENING_HEIGHT = 0.066
MID_GUIDE_X0 = 0.020
MID_GUIDE_LENGTH = 0.310
MID_SIDE_LAND = 0.008
MID_SIDE_LAND_HEIGHT = 0.074
MID_SIDE_LAND_Z = -0.015
MID_BOTTOM_LAND = 0.011
MID_SHOULDER_X0 = 0.100
MID_SHOULDER_LENGTH = 0.060
MID_SHOULDER_WIDTH = 0.118
MID_SHOULDER_HEIGHT = 0.092

TOP_BODY_LENGTH = 0.430
TOP_TOTAL_LENGTH = 0.448
TOP_BODY_WIDTH = 0.074
TOP_BODY_HEIGHT = 0.050
TOP_INNER_WIDTH = 0.058
TOP_INNER_HEIGHT = 0.034
TOP_REAR_PLATE = 0.012
TOP_GUIDE_X0 = 0.018
TOP_GUIDE_LENGTH = 0.222
TOP_SIDE_LAND = 0.008
TOP_TOP_LAND = 0.007
TOP_SHOULDER_X0 = 0.065
TOP_SHOULDER_LENGTH = 0.055
TOP_SHOULDER_WIDTH = 0.084
TOP_SHOULDER_HEIGHT = 0.058
TOP_NOSE_X0 = 0.400
TOP_NOSE_LENGTH = 0.030
TOP_NOSE_WIDTH = 0.088
TOP_NOSE_HEIGHT = 0.070
TOOL_PAD_X0 = 0.430
TOOL_PAD_LENGTH = 0.018
TOOL_PAD_SIZE = 0.110

ROOT_TO_MID_ORIGIN_X = 0.060
ROOT_TO_MID_TRAVEL = 0.240
MID_TO_TOP_ORIGIN_X = 0.170
MID_TO_TOP_TRAVEL = 0.220


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _build_root_part(model: ArticulatedObject):
    root = model.part("root_sleeve")
    side_wall = (ROOT_WIDTH - ROOT_INNER_WIDTH) / 2.0
    roof_wall = (ROOT_HEIGHT - ROOT_INNER_HEIGHT) / 2.0
    ring_side = (ROOT_FRONT_OUTER_WIDTH - ROOT_FRONT_OPENING_WIDTH) / 2.0
    ring_cap = (ROOT_FRONT_OUTER_HEIGHT - ROOT_FRONT_OPENING_HEIGHT) / 2.0

    _add_box_visual(
        root,
        name="left_wall",
        size=(ROOT_LENGTH, side_wall, ROOT_HEIGHT),
        center=(ROOT_LENGTH / 2.0, ROOT_INNER_WIDTH / 2.0 + side_wall / 2.0, 0.0),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="right_wall",
        size=(ROOT_LENGTH, side_wall, ROOT_HEIGHT),
        center=(ROOT_LENGTH / 2.0, -(ROOT_INNER_WIDTH / 2.0 + side_wall / 2.0), 0.0),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="top_wall",
        size=(ROOT_LENGTH, ROOT_INNER_WIDTH, roof_wall),
        center=(ROOT_LENGTH / 2.0, 0.0, ROOT_INNER_HEIGHT / 2.0 + roof_wall / 2.0),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="bottom_wall",
        size=(ROOT_LENGTH, ROOT_INNER_WIDTH, roof_wall),
        center=(ROOT_LENGTH / 2.0, 0.0, -(ROOT_INNER_HEIGHT / 2.0 + roof_wall / 2.0)),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="rear_plate",
        size=(ROOT_REAR_PLATE, ROOT_INNER_WIDTH, ROOT_INNER_HEIGHT),
        center=(ROOT_REAR_PLATE / 2.0, 0.0, 0.0),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="front_left_cheek",
        size=(ROOT_FRONT_RING, ring_side, ROOT_FRONT_OUTER_HEIGHT),
        center=(
            ROOT_LENGTH - ROOT_FRONT_RING / 2.0,
            ROOT_FRONT_OPENING_WIDTH / 2.0 + ring_side / 2.0,
            0.0,
        ),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="front_right_cheek",
        size=(ROOT_FRONT_RING, ring_side, ROOT_FRONT_OUTER_HEIGHT),
        center=(
            ROOT_LENGTH - ROOT_FRONT_RING / 2.0,
            -(ROOT_FRONT_OPENING_WIDTH / 2.0 + ring_side / 2.0),
            0.0,
        ),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="front_top_stop",
        size=(ROOT_FRONT_RING, ROOT_FRONT_OPENING_WIDTH, ring_cap),
        center=(
            ROOT_LENGTH - ROOT_FRONT_RING / 2.0,
            0.0,
            ROOT_FRONT_OPENING_HEIGHT / 2.0 + ring_cap / 2.0,
        ),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="front_bottom_stop",
        size=(ROOT_FRONT_RING, ROOT_FRONT_OPENING_WIDTH, ring_cap),
        center=(
            ROOT_LENGTH - ROOT_FRONT_RING / 2.0,
            0.0,
            -(ROOT_FRONT_OPENING_HEIGHT / 2.0 + ring_cap / 2.0),
        ),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="mount_foot",
        size=(ROOT_FOOT_LENGTH, ROOT_FOOT_WIDTH, ROOT_FOOT_HEIGHT),
        center=(ROOT_FOOT_LENGTH / 2.0, 0.0, -(ROOT_HEIGHT / 2.0 + ROOT_FOOT_HEIGHT / 2.0)),
        material="housing_dark",
    )
    _add_box_visual(
        root,
        name="lower_chin",
        size=(0.110, ROOT_FRONT_OUTER_WIDTH, 0.016),
        center=(ROOT_LENGTH - 0.055, 0.0, -(ROOT_HEIGHT / 2.0 + 0.013)),
        material="housing_dark",
    )
    root.inertial = Inertial.from_geometry(
        Box((ROOT_LENGTH, ROOT_FOOT_WIDTH, ROOT_HEIGHT + ROOT_FOOT_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(ROOT_LENGTH / 2.0, 0.0, -ROOT_FOOT_HEIGHT / 2.0)),
    )
    return root


def _build_middle_part(model: ArticulatedObject):
    middle = model.part("middle_stage")
    side_wall = (MID_BODY_WIDTH - MID_INNER_WIDTH) / 2.0
    roof_wall = (MID_BODY_HEIGHT - MID_INNER_HEIGHT) / 2.0
    ring_side = (MID_FRONT_OUTER_WIDTH - MID_FRONT_OPENING_WIDTH) / 2.0
    ring_cap = (MID_FRONT_OUTER_HEIGHT - MID_FRONT_OPENING_HEIGHT) / 2.0

    _add_box_visual(
        middle,
        name="left_wall",
        size=(MID_LENGTH, side_wall, MID_BODY_HEIGHT),
        center=(MID_LENGTH / 2.0, MID_INNER_WIDTH / 2.0 + side_wall / 2.0, 0.0),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="right_wall",
        size=(MID_LENGTH, side_wall, MID_BODY_HEIGHT),
        center=(MID_LENGTH / 2.0, -(MID_INNER_WIDTH / 2.0 + side_wall / 2.0), 0.0),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="top_wall",
        size=(MID_LENGTH, MID_INNER_WIDTH, roof_wall),
        center=(MID_LENGTH / 2.0, 0.0, MID_INNER_HEIGHT / 2.0 + roof_wall / 2.0),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="bottom_wall",
        size=(MID_LENGTH, MID_INNER_WIDTH, roof_wall),
        center=(MID_LENGTH / 2.0, 0.0, -(MID_INNER_HEIGHT / 2.0 + roof_wall / 2.0)),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="rear_plate",
        size=(MID_REAR_PLATE, MID_INNER_WIDTH, MID_INNER_HEIGHT),
        center=(MID_REAR_PLATE / 2.0, 0.0, 0.0),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="rear_cap",
        size=(MID_REAR_CAP_LENGTH, MID_REAR_CAP_WIDTH, MID_REAR_CAP_HEIGHT),
        center=(MID_REAR_CAP_LENGTH / 2.0, 0.0, 0.0),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="left_slide_land",
        size=(MID_GUIDE_LENGTH, MID_SIDE_LAND, MID_SIDE_LAND_HEIGHT),
        center=(
            MID_GUIDE_X0 + MID_GUIDE_LENGTH / 2.0,
            MID_BODY_WIDTH / 2.0 + MID_SIDE_LAND / 2.0,
            MID_SIDE_LAND_Z,
        ),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="right_slide_land",
        size=(MID_GUIDE_LENGTH, MID_SIDE_LAND, MID_SIDE_LAND_HEIGHT),
        center=(
            MID_GUIDE_X0 + MID_GUIDE_LENGTH / 2.0,
            -(MID_BODY_WIDTH / 2.0 + MID_SIDE_LAND / 2.0),
            MID_SIDE_LAND_Z,
        ),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="bottom_slide_land",
        size=(MID_GUIDE_LENGTH, 0.090, MID_BOTTOM_LAND),
        center=(
            MID_GUIDE_X0 + MID_GUIDE_LENGTH / 2.0,
            0.0,
            -(MID_BODY_HEIGHT / 2.0 + MID_BOTTOM_LAND / 2.0),
        ),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="stop_shoulder",
        size=(MID_SHOULDER_LENGTH, MID_SHOULDER_WIDTH, MID_SHOULDER_HEIGHT),
        center=(MID_SHOULDER_X0 + MID_SHOULDER_LENGTH / 2.0, 0.0, 0.0),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="front_left_cheek",
        size=(MID_FRONT_RING, ring_side, MID_FRONT_OUTER_HEIGHT),
        center=(
            MID_LENGTH - MID_FRONT_RING / 2.0,
            MID_FRONT_OPENING_WIDTH / 2.0 + ring_side / 2.0,
            0.0,
        ),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="front_right_cheek",
        size=(MID_FRONT_RING, ring_side, MID_FRONT_OUTER_HEIGHT),
        center=(
            MID_LENGTH - MID_FRONT_RING / 2.0,
            -(MID_FRONT_OPENING_WIDTH / 2.0 + ring_side / 2.0),
            0.0,
        ),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="front_top_stop",
        size=(MID_FRONT_RING, MID_FRONT_OPENING_WIDTH, ring_cap),
        center=(
            MID_LENGTH - MID_FRONT_RING / 2.0,
            0.0,
            MID_FRONT_OPENING_HEIGHT / 2.0 + ring_cap / 2.0,
        ),
        material="machined_stage",
    )
    _add_box_visual(
        middle,
        name="front_bottom_stop",
        size=(MID_FRONT_RING, MID_FRONT_OPENING_WIDTH, ring_cap),
        center=(
            MID_LENGTH - MID_FRONT_RING / 2.0,
            0.0,
            -(MID_FRONT_OPENING_HEIGHT / 2.0 + ring_cap / 2.0),
        ),
        material="machined_stage",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MID_LENGTH, ROOT_INNER_WIDTH, ROOT_INNER_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(MID_LENGTH / 2.0, 0.0, 0.0)),
    )
    return middle


def _build_top_part(model: ArticulatedObject):
    top = model.part("top_stage")
    side_wall = (TOP_BODY_WIDTH - TOP_INNER_WIDTH) / 2.0
    roof_wall = (TOP_BODY_HEIGHT - TOP_INNER_HEIGHT) / 2.0

    _add_box_visual(
        top,
        name="left_wall",
        size=(TOP_BODY_LENGTH, side_wall, TOP_BODY_HEIGHT),
        center=(TOP_BODY_LENGTH / 2.0, TOP_INNER_WIDTH / 2.0 + side_wall / 2.0, 0.0),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="right_wall",
        size=(TOP_BODY_LENGTH, side_wall, TOP_BODY_HEIGHT),
        center=(TOP_BODY_LENGTH / 2.0, -(TOP_INNER_WIDTH / 2.0 + side_wall / 2.0), 0.0),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="top_wall",
        size=(TOP_BODY_LENGTH, TOP_INNER_WIDTH, roof_wall),
        center=(TOP_BODY_LENGTH / 2.0, 0.0, TOP_INNER_HEIGHT / 2.0 + roof_wall / 2.0),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="bottom_wall",
        size=(TOP_BODY_LENGTH, TOP_INNER_WIDTH, roof_wall),
        center=(TOP_BODY_LENGTH / 2.0, 0.0, -(TOP_INNER_HEIGHT / 2.0 + roof_wall / 2.0)),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="rear_plate",
        size=(TOP_REAR_PLATE, TOP_INNER_WIDTH, TOP_INNER_HEIGHT),
        center=(TOP_REAR_PLATE / 2.0, 0.0, 0.0),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="left_slide_land",
        size=(TOP_GUIDE_LENGTH, TOP_SIDE_LAND, 0.044),
        center=(
            TOP_GUIDE_X0 + TOP_GUIDE_LENGTH / 2.0,
            TOP_BODY_WIDTH / 2.0 + TOP_SIDE_LAND / 2.0,
            0.010,
        ),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="right_slide_land",
        size=(TOP_GUIDE_LENGTH, TOP_SIDE_LAND, 0.044),
        center=(
            TOP_GUIDE_X0 + TOP_GUIDE_LENGTH / 2.0,
            -(TOP_BODY_WIDTH / 2.0 + TOP_SIDE_LAND / 2.0),
            0.010,
        ),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="top_slide_land",
        size=(TOP_GUIDE_LENGTH, 0.060, TOP_TOP_LAND),
        center=(
            TOP_GUIDE_X0 + TOP_GUIDE_LENGTH / 2.0,
            0.0,
            TOP_BODY_HEIGHT / 2.0 + TOP_TOP_LAND / 2.0,
        ),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="stop_shoulder",
        size=(TOP_SHOULDER_LENGTH, TOP_SHOULDER_WIDTH, TOP_SHOULDER_HEIGHT),
        center=(TOP_SHOULDER_X0 + TOP_SHOULDER_LENGTH / 2.0, 0.0, 0.0),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="nose_block",
        size=(TOP_NOSE_LENGTH, TOP_NOSE_WIDTH, TOP_NOSE_HEIGHT),
        center=(TOP_NOSE_X0 + TOP_NOSE_LENGTH / 2.0, 0.0, 0.0),
        material="light_stage",
    )
    _add_box_visual(
        top,
        name="tool_pad",
        size=(TOOL_PAD_LENGTH, TOOL_PAD_SIZE, TOOL_PAD_SIZE),
        center=(TOOL_PAD_X0 + TOOL_PAD_LENGTH / 2.0, 0.0, 0.0),
        material="light_stage",
    )
    top.inertial = Inertial.from_geometry(
        Box((TOP_TOTAL_LENGTH, TOOL_PAD_SIZE, TOOL_PAD_SIZE)),
        mass=1.9,
        origin=Origin(xyz=(TOP_TOTAL_LENGTH / 2.0, 0.0, 0.0)),
    )
    return top


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_boom")

    model.material("housing_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("machined_stage", rgba=(0.61, 0.65, 0.70, 1.0))
    model.material("light_stage", rgba=(0.78, 0.80, 0.83, 1.0))

    root = _build_root_part(model)
    middle = _build_middle_part(model)
    top = _build_top_part(model)

    model.articulation(
        "root_to_middle",
        ArticulationType.PRISMATIC,
        parent=root,
        child=middle,
        origin=Origin(xyz=(ROOT_TO_MID_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.35,
            lower=0.0,
            upper=ROOT_TO_MID_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_top",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=top,
        origin=Origin(xyz=(MID_TO_TOP_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1100.0,
            velocity=0.40,
            lower=0.0,
            upper=MID_TO_TOP_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_sleeve")
    middle = object_model.get_part("middle_stage")
    top = object_model.get_part("top_stage")
    root_to_middle = object_model.get_articulation("root_to_middle")
    middle_to_top = object_model.get_articulation("middle_to_top")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0015)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    ctx.check(
        "prismatic_axes_aligned",
        root_to_middle.axis == (1.0, 0.0, 0.0) and middle_to_top.axis == (1.0, 0.0, 0.0),
        details=f"axes={root_to_middle.axis}, {middle_to_top.axis}",
    )
    ctx.check(
        "travel_limits_are_serial_extensions",
        (
            root_to_middle.motion_limits is not None
            and middle_to_top.motion_limits is not None
            and root_to_middle.motion_limits.lower == 0.0
            and middle_to_top.motion_limits.lower == 0.0
            and root_to_middle.motion_limits.upper is not None
            and middle_to_top.motion_limits.upper is not None
            and root_to_middle.motion_limits.upper >= 0.20
            and middle_to_top.motion_limits.upper >= 0.18
        ),
        details=(
            f"root_to_middle={root_to_middle.motion_limits}, "
            f"middle_to_top={middle_to_top.motion_limits}"
        ),
    )

    ctx.expect_contact(
        root,
        middle,
        contact_tol=0.0015,
        name="root_and_middle_guides_touch_at_home",
    )
    ctx.expect_contact(
        middle,
        top,
        contact_tol=0.0015,
        name="middle_and_top_guides_touch_at_home",
    )

    with ctx.pose({root_to_middle: ROOT_TO_MID_TRAVEL}):
        ctx.expect_contact(
            root,
            middle,
            contact_tol=0.0015,
            name="root_and_middle_keep_guide_overlap_fully_extended",
        )

    with ctx.pose(
        {
            root_to_middle: ROOT_TO_MID_TRAVEL,
            middle_to_top: MID_TO_TOP_TRAVEL,
        }
    ):
        ctx.expect_contact(
            middle,
            top,
            contact_tol=0.0015,
            name="middle_and_top_keep_guide_overlap_fully_extended",
        )
        ctx.expect_gap(
            top,
            root,
            axis="x",
            min_gap=0.16,
            name="top_stage_clears_root_housing_at_full_extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
