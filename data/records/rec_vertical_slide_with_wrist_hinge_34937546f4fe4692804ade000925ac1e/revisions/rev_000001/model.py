from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


FRAME_HEIGHT = 0.30
FRAME_WIDTH = 0.14
FRAME_BACK_PLATE_T = 0.012
FRAME_CHEEK_T = 0.018
FRAME_CHEEK_DEPTH = 0.032
FRAME_CHEEK_HEIGHT = 0.23
FRAME_TOP_BRIDGE_T = 0.018
FRAME_BOTTOM_FOOT_T = 0.036
GUIDE_Y = 0.028

CARRIAGE_HOME_Z = 0.105
SLIDE_TRAVEL = 0.10

WRIST_PITCH_TEST = 0.75


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rear_frame_shape() -> cq.Workplane:
    shape = _box((0.042, FRAME_WIDTH, FRAME_HEIGHT), (-0.035, 0.0, FRAME_HEIGHT / 2.0))
    shape = shape.cut(_box((0.032, 0.056, 0.220), (-0.030, 0.0, 0.150)))
    shape = shape.union(_box((0.020, 0.016, 0.170), (-0.024, GUIDE_Y, 0.120)))
    shape = shape.union(_box((0.020, 0.016, 0.170), (-0.024, -GUIDE_Y, 0.120)))
    shape = shape.union(_box((0.026, 0.072, 0.050), (-0.043, 0.0, 0.235)))
    shape = shape.union(_box((0.026, 0.040, 0.008), (-0.034, 0.040, 0.004)))
    shape = shape.union(_box((0.026, 0.040, 0.008), (-0.034, -0.040, 0.004)))

    return shape


def _slide_carriage_shape() -> cq.Workplane:
    left_slide_shoe = _box((0.012, 0.016, 0.110), (-0.008, GUIDE_Y, 0.0))
    right_slide_shoe = _box((0.012, 0.016, 0.110), (-0.008, -GUIDE_Y, 0.0))
    rear_web = _box((0.012, 0.050, 0.078), (0.004, 0.0, 0.0))
    main_block = _box((0.028, 0.056, 0.080), (0.022, 0.0, 0.0))
    lower_face_block = _box((0.014, 0.032, 0.016), (0.030, 0.0, -0.020))
    left_hinge_ear = _box((0.010, 0.010, 0.024), (0.041, 0.015, 0.016))
    right_hinge_ear = _box((0.010, 0.010, 0.024), (0.041, -0.015, 0.016))

    shape = left_slide_shoe.union(right_slide_shoe)
    shape = shape.union(rear_web)
    shape = shape.union(main_block)
    shape = shape.union(lower_face_block)
    shape = shape.union(left_hinge_ear)
    shape = shape.union(right_hinge_ear)

    return shape


def _wrist_face_shape() -> cq.Workplane:
    hinge_tongue = _box((0.010, 0.018, 0.018), (0.005, 0.0, 0.0))
    hinge_barrel = (
        cq.Workplane("XZ")
        .circle(0.005)
        .extrude(0.018)
        .translate((0.005, -0.009, 0.000))
    )
    neck = _box((0.014, 0.020, 0.014), (0.014, 0.0, -0.010))
    face_plate = _box((0.018, 0.052, 0.056), (0.028, 0.0, -0.034))
    lower_lip = _box((0.014, 0.036, 0.010), (0.036, 0.0, -0.062))

    shape = hinge_tongue.union(hinge_barrel)
    shape = shape.union(neck)
    shape = shape.union(face_plate)
    shape = shape.union(lower_lip)

    return shape


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_slide_wrist_unit")

    model.material("frame_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("carriage_alloy", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("wrist_paint", rgba=(0.86, 0.44, 0.12, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_cadquery(_rear_frame_shape(), "rear_frame"),
        material="frame_steel",
        name="frame_shell",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.060, FRAME_WIDTH, FRAME_HEIGHT)),
        mass=8.5,
        origin=Origin(xyz=(-0.035, 0.0, FRAME_HEIGHT / 2.0)),
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        mesh_from_cadquery(_slide_carriage_shape(), "slide_carriage"),
        material="carriage_alloy",
        name="carriage_shell",
    )
    slide_carriage.inertial = Inertial.from_geometry(
        Box((0.056, 0.056, 0.110)),
        mass=2.8,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(
        mesh_from_cadquery(_wrist_face_shape(), "wrist_face"),
        material="wrist_paint",
        name="wrist_shell",
    )
    wrist_face.inertial = Inertial.from_geometry(
        Box((0.046, 0.052, 0.074)),
        mass=0.9,
        origin=Origin(xyz=(0.026, 0.0, -0.032)),
    )

    model.articulation(
        "frame_to_slide",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=slide_carriage,
        origin=Origin(xyz=(0.000, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=wrist_face,
        origin=Origin(xyz=(0.046, 0.0, 0.016)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    slide_carriage = object_model.get_part("slide_carriage")
    wrist_face = object_model.get_part("wrist_face")
    slide = object_model.get_articulation("frame_to_slide")
    wrist = object_model.get_articulation("carriage_to_wrist")

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
        "mechanism_axes_match_prompt",
        slide.axis == (0.0, 0.0, 1.0) and wrist.axis == (0.0, -1.0, 0.0),
        f"slide axis={slide.axis}, wrist axis={wrist.axis}",
    )

    ctx.expect_contact(
        rear_frame,
        slide_carriage,
        name="slide_carriage_is_supported_by_frame",
    )
    ctx.expect_contact(
        slide_carriage,
        wrist_face,
        name="wrist_face_is_supported_by_carriage",
    )
    ctx.expect_within(
        slide_carriage,
        rear_frame,
        axes="yz",
        margin=0.0,
        name="slide_carriage_is_captured_in_fork_frame",
    )

    home_position = ctx.part_world_position(slide_carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        top_position = ctx.part_world_position(slide_carriage)
        ctx.expect_within(
            slide_carriage,
            rear_frame,
            axes="yz",
            margin=0.0,
            name="slide_carriage_stays_captured_at_top_of_travel",
        )

    ctx.check(
        "prismatic_stage_moves_upward",
        (
            home_position is not None
            and top_position is not None
            and (top_position[2] - home_position[2]) > (0.95 * SLIDE_TRAVEL)
        ),
        f"home={home_position}, top={top_position}, expected_z_travel≈{SLIDE_TRAVEL}",
    )

    home_wrist_aabb = ctx.part_element_world_aabb(wrist_face, elem="wrist_shell")
    with ctx.pose({wrist: WRIST_PITCH_TEST}):
        pitched_wrist_aabb = ctx.part_element_world_aabb(wrist_face, elem="wrist_shell")

    home_wrist_center = _aabb_center(home_wrist_aabb)
    pitched_wrist_center = _aabb_center(pitched_wrist_aabb)
    ctx.check(
        "wrist_positive_rotation_tips_output_forward_and_up",
        (
            home_wrist_center is not None
            and pitched_wrist_center is not None
            and pitched_wrist_center[0] > home_wrist_center[0] + 0.010
            and pitched_wrist_center[2] > home_wrist_center[2] + 0.010
        ),
        f"home_center={home_wrist_center}, pitched_center={pitched_wrist_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
