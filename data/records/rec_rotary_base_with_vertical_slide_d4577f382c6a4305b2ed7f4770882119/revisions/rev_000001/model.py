from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_WIDTH = 0.34
FRAME_DEPTH = 0.18
BASE_THICKNESS = 0.028
PEDESTAL_RADIUS = 0.045
PEDESTAL_HEIGHT = 0.052
BEARING_FLANGE_RADIUS = 0.062
BEARING_FLANGE_THICKNESS = 0.014
YAW_JOINT_Y = 0.040
REAR_Y = -0.078
REAR_BEAM_WIDTH = 0.260
REAR_BEAM_DEPTH = 0.024
REAR_BEAM_HEIGHT = 0.072
UPRIGHT_X = 0.118
UPRIGHT_WIDTH = 0.034
UPRIGHT_DEPTH = 0.024
UPRIGHT_HEIGHT = 0.280
TOP_BRIDGE_WIDTH = 0.254
TOP_BRIDGE_DEPTH = 0.024
TOP_BRIDGE_THICKNESS = 0.040

YAW_JOINT_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + BEARING_FLANGE_THICKNESS
YAW_LIMIT = 2.35

STAGE_BASE_RADIUS = 0.072
STAGE_BASE_THICKNESS = 0.018
STAGE_HUB_RADIUS = 0.050
STAGE_HUB_HEIGHT = 0.040
STAGE_TOWER_WIDTH = 0.145
STAGE_TOWER_THICKNESS = 0.020
STAGE_TOWER_BASE_Z = 0.028
STAGE_TOWER_HEIGHT = 0.272
GUIDE_RAIL_WIDTH = 0.022
GUIDE_RAIL_DEPTH = 0.014
GUIDE_RAIL_HEIGHT = 0.245
GUIDE_RAIL_X = 0.044
GUIDE_RAIL_BASE_Z = 0.055
STAGE_TOP_BRIDGE_WIDTH = 0.118
STAGE_TOP_BRIDGE_DEPTH = 0.026
STAGE_TOP_BRIDGE_THICKNESS = 0.030

SLIDE_JOINT_Z = 0.000
SLIDE_TRAVEL = 0.180
SLIDER_PAD_WIDTH = 0.022
SLIDER_PAD_DEPTH = 0.008
SLIDER_PAD_HEIGHT = 0.110
SLIDER_PAD_Y = 0.018
MAIN_BODY_WIDTH = 0.105
MAIN_BODY_DEPTH = 0.030
MAIN_BODY_HEIGHT = 0.120
MAIN_BODY_Y = 0.037
OUTPUT_PLATE_WIDTH = 0.150
OUTPUT_PLATE_DEPTH = 0.010
OUTPUT_PLATE_HEIGHT = 0.170
OUTPUT_PLATE_Y = 0.115
OUTPUT_PLATE_Z = 0.085


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_rotary_lift_head")

    model.material("frame_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("stage_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("slide_gray", rgba=(0.84, 0.86, 0.88, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="frame_gray",
        name="base_plate",
    )
    rear_frame.visual(
        Box((0.060, 0.118, 0.036)),
        origin=Origin(xyz=(0.0, -0.008, 0.046)),
        material="frame_gray",
        name="neck_beam",
    )
    rear_frame.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, YAW_JOINT_Y, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
        material="frame_gray",
        name="pedestal",
    )
    rear_frame.visual(
        Cylinder(radius=BEARING_FLANGE_RADIUS, length=BEARING_FLANGE_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                YAW_JOINT_Y,
                BASE_THICKNESS + PEDESTAL_HEIGHT + BEARING_FLANGE_THICKNESS / 2.0,
            )
        ),
        material="frame_gray",
        name="bearing_flange",
    )
    rear_frame.visual(
        Box((REAR_BEAM_WIDTH, REAR_BEAM_DEPTH, REAR_BEAM_HEIGHT)),
        origin=Origin(
            xyz=(0.0, REAR_Y, BASE_THICKNESS + REAR_BEAM_HEIGHT / 2.0)
        ),
        material="frame_gray",
        name="rear_beam",
    )
    rear_frame.visual(
        Box((UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT)),
        origin=Origin(
            xyz=(
                -0.110,
                REAR_Y,
                BASE_THICKNESS + REAR_BEAM_HEIGHT + UPRIGHT_HEIGHT / 2.0,
            )
        ),
        material="frame_gray",
        name="left_upright",
    )
    rear_frame.visual(
        Box((UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT)),
        origin=Origin(
            xyz=(
                0.110,
                REAR_Y,
                BASE_THICKNESS + REAR_BEAM_HEIGHT + UPRIGHT_HEIGHT / 2.0,
            )
        ),
        material="frame_gray",
        name="right_upright",
    )
    rear_frame.visual(
        Box((TOP_BRIDGE_WIDTH, TOP_BRIDGE_DEPTH, TOP_BRIDGE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                REAR_Y,
                BASE_THICKNESS
                + REAR_BEAM_HEIGHT
                + UPRIGHT_HEIGHT
                + TOP_BRIDGE_THICKNESS / 2.0,
            )
        ),
        material="frame_gray",
        name="top_bridge",
    )
    rear_frame.visual(
        Box(
            (
                0.090,
                0.016,
                BASE_THICKNESS + REAR_BEAM_HEIGHT + UPRIGHT_HEIGHT - (BASE_THICKNESS + REAR_BEAM_HEIGHT),
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                REAR_Y,
                (BASE_THICKNESS + REAR_BEAM_HEIGHT + UPRIGHT_HEIGHT + (BASE_THICKNESS + REAR_BEAM_HEIGHT))
                / 2.0,
            )
        ),
        material="frame_gray",
        name="back_panel",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, 0.44)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="stage_gray",
        name="turntable_disc",
    )
    yaw_stage.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material="stage_gray",
        name="hub",
    )
    yaw_stage.visual(
        Box((0.110, 0.018, 0.286)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material="stage_gray",
        name="mast",
    )
    yaw_stage.visual(
        Box((0.018, 0.014, 0.286)),
        origin=Origin(xyz=(-0.040, 0.016, 0.175)),
        material="stage_gray",
        name="left_rail",
    )
    yaw_stage.visual(
        Box((0.018, 0.014, 0.286)),
        origin=Origin(xyz=(0.040, 0.016, 0.175)),
        material="stage_gray",
        name="right_rail",
    )
    yaw_stage.visual(
        Box((0.088, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.012, 0.345)),
        material="stage_gray",
        name="top_cap",
    )
    yaw_stage.visual(
        Box((0.030, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, 0.006, 0.332)),
        material="stage_gray",
        name="cap_post",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.12, 0.06, 0.34)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    vertical_slide = model.part("vertical_slide")
    vertical_slide.visual(
        Box((0.018, 0.060, 0.130)),
        origin=Origin(xyz=(-0.050, 0.053, 0.095)),
        material="slide_gray",
        name="left_shoe",
    )
    vertical_slide.visual(
        Box((0.018, 0.060, 0.130)),
        origin=Origin(xyz=(0.050, 0.053, 0.095)),
        material="slide_gray",
        name="right_shoe",
    )
    vertical_slide.visual(
        Box((0.110, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.073, 0.050)),
        material="slide_gray",
        name="lower_crosshead",
    )
    vertical_slide.visual(
        Box((0.110, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.073, 0.130)),
        material="slide_gray",
        name="upper_crosshead",
    )
    vertical_slide.visual(
        Box((0.100, 0.028, 0.110)),
        origin=Origin(xyz=(0.0, 0.088, 0.090)),
        material="slide_gray",
        name="carriage_body",
    )
    vertical_slide.visual(
        Box((0.016, 0.040, 0.130)),
        origin=Origin(xyz=(-0.032, 0.108, 0.085)),
        material="slide_gray",
        name="left_plate_standoff",
    )
    vertical_slide.visual(
        Box((0.016, 0.040, 0.130)),
        origin=Origin(xyz=(0.032, 0.108, 0.085)),
        material="slide_gray",
        name="right_plate_standoff",
    )
    vertical_slide.visual(
        Box((OUTPUT_PLATE_WIDTH, OUTPUT_PLATE_DEPTH, OUTPUT_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, OUTPUT_PLATE_Y, OUTPUT_PLATE_Z)),
        material="slide_gray",
        name="output_plate",
    )
    vertical_slide.inertial = Inertial.from_geometry(
        Box((OUTPUT_PLATE_WIDTH, 0.06, OUTPUT_PLATE_HEIGHT)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.055, OUTPUT_PLATE_HEIGHT / 2.0)),
    )

    model.articulation(
        "frame_to_yaw",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, YAW_JOINT_Y, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.3,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )
    model.articulation(
        "yaw_to_slide",
        ArticulationType.PRISMATIC,
        parent=yaw_stage,
        child=vertical_slide,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.22,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    yaw_stage = object_model.get_part("yaw_stage")
    vertical_slide = object_model.get_part("vertical_slide")
    yaw = object_model.get_articulation("frame_to_yaw")
    lift = object_model.get_articulation("yaw_to_slide")

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

    ctx.check(
        "yaw articulation is revolute about +Z",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "lift articulation is prismatic along +Z",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0),
        f"type={lift.articulation_type}, axis={lift.axis}",
    )

    with ctx.pose({yaw: 0.0, lift: 0.0}):
        ctx.expect_contact(
            yaw_stage,
            rear_frame,
            contact_tol=0.0015,
            name="yaw stage seats on rear frame bearing flange",
        )
        ctx.expect_contact(
            vertical_slide,
            yaw_stage,
            contact_tol=0.0015,
            name="slide carriage remains supported on guide rails",
        )
        ctx.expect_gap(
            vertical_slide,
            yaw_stage,
            axis="y",
            min_gap=0.045,
            max_gap=0.080,
            positive_elem="output_plate",
            name="plain output plate stays proud of the guide stage",
        )

    slide_z0 = None
    slide_z1 = None
    with ctx.pose({lift: 0.0}):
        pos0 = ctx.part_world_position(vertical_slide)
        if pos0 is not None:
            slide_z0 = pos0[2]
    with ctx.pose({lift: SLIDE_TRAVEL}):
        pos1 = ctx.part_world_position(vertical_slide)
        if pos1 is not None:
            slide_z1 = pos1[2]
        ctx.expect_contact(
            vertical_slide,
            yaw_stage,
            contact_tol=0.0015,
            name="raised carriage still bears on the guide rails",
        )
    ctx.check(
        "lift travel moves the carriage upward by commanded stroke",
        slide_z0 is not None
        and slide_z1 is not None
        and abs((slide_z1 - slide_z0) - SLIDE_TRAVEL) <= 0.001,
        f"z0={slide_z0}, z1={slide_z1}, expected_delta={SLIDE_TRAVEL}",
    )

    plate_center_rest = None
    plate_center_yawed = None
    with ctx.pose({yaw: 0.0}):
        plate_center_rest = _aabb_center(
            ctx.part_element_world_aabb(vertical_slide, elem="output_plate")
        )
    with ctx.pose({yaw: 1.0}):
        plate_center_yawed = _aabb_center(
            ctx.part_element_world_aabb(vertical_slide, elem="output_plate")
        )
    ctx.check(
        "positive yaw swings the output plate toward world -X",
        plate_center_rest is not None
        and plate_center_yawed is not None
        and abs(plate_center_rest[0]) < 0.01
        and plate_center_yawed[0] < -0.03,
        f"rest_center={plate_center_rest}, yawed_center={plate_center_yawed}",
    )

    with ctx.pose({yaw: 1.0, lift: SLIDE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no overlaps at raised and yawed working pose"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
