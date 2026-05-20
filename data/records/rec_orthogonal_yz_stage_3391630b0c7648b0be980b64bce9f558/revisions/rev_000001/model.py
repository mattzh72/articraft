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


FRAME_W = 0.74
FRAME_H = 1.14
FRAME_PANEL_T = 0.016
FRAME_REAR_DEPTH = 0.044
X_RAIL_LEN = 0.60
X_RAIL_T = 0.010
X_RAIL_H = 0.050
X_RAIL_Z = 0.155
X_TRAVEL = 0.17

BEAM_CARRIAGE_W = 0.27
BEAM_CARRIAGE_H = 0.46
BEAM_CARRIAGE_T = 0.024
BEAM_GROOVE_LEN = 0.23
BEAM_GROOVE_DEPTH = 0.011
BEAM_BODY_L = 0.56
BEAM_BODY_D = 0.092
BEAM_BODY_H = 0.10

GUIDE_MAST_W = 0.096
GUIDE_MAST_D = 0.038
GUIDE_MAST_H = 0.412
GUIDE_MAST_BACK_Y = 0.056
GUIDE_MAST_Z = -0.210
GUIDE_RAIL_W = 0.022
GUIDE_RAIL_T = 0.010
GUIDE_RAIL_H = 0.368
GUIDE_RAIL_X = 0.026
GUIDE_RAIL_CENTER_Y = 0.099
GUIDE_RAIL_CENTER_Z = -0.190

Z_TRAVEL = 0.24
X_STAGE_Y = X_RAIL_T
Z_STAGE_ORIGIN_Y = GUIDE_MAST_BACK_Y + GUIDE_MAST_D + X_RAIL_T
Z_STAGE_ORIGIN_Z = -0.012
SLIDE_CARRIAGE_W = 0.13
SLIDE_CARRIAGE_D = 0.052
SLIDE_CARRIAGE_H = 0.16
SLIDE_GROOVE_DEPTH = 0.011


def _box_back(
    sx: float,
    sy: float,
    sz: float,
    *,
    x: float = 0.0,
    back_y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, False, True)).translate(
        (x, back_y, z)
    )


def _fuse(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _frame_body_shape() -> cq.Workplane:
    plate = _box_back(FRAME_W, FRAME_PANEL_T, FRAME_H, back_y=-FRAME_PANEL_T, z=0.0)
    slot_x = FRAME_W * 0.5 - 0.070
    slot_z = FRAME_H * 0.5 - 0.120
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-slot_x, slot_z),
                (slot_x, slot_z),
                (-slot_x, -slot_z),
                (slot_x, -slot_z),
            ]
        )
        .slot2D(0.050, 0.014, angle=90)
        .cutThruAll()
    )

    side_x = FRAME_W * 0.5 - 0.034
    top_z = FRAME_H * 0.5 - 0.045
    body = _fuse(
        plate,
        _box_back(0.044, FRAME_REAR_DEPTH, FRAME_H - 0.080, x=-side_x, back_y=-FRAME_PANEL_T - FRAME_REAR_DEPTH, z=0.0),
        _box_back(0.044, FRAME_REAR_DEPTH, FRAME_H - 0.080, x=side_x, back_y=-FRAME_PANEL_T - FRAME_REAR_DEPTH, z=0.0),
        _box_back(FRAME_W - 0.140, FRAME_REAR_DEPTH, 0.060, back_y=-FRAME_PANEL_T - FRAME_REAR_DEPTH, z=top_z),
        _box_back(FRAME_W - 0.140, FRAME_REAR_DEPTH, 0.060, back_y=-FRAME_PANEL_T - FRAME_REAR_DEPTH, z=-top_z),
        _box_back(0.054, 0.034, FRAME_H - 0.220, x=-0.180, back_y=-FRAME_PANEL_T - 0.034, z=0.0),
        _box_back(0.054, 0.034, FRAME_H - 0.220, x=0.180, back_y=-FRAME_PANEL_T - 0.034, z=0.0),
    )
    return body


def _beam_carriage_shape() -> cq.Workplane:
    carriage = _box_back(
        BEAM_CARRIAGE_W,
        BEAM_CARRIAGE_T,
        BEAM_CARRIAGE_H,
        back_y=0.0,
        z=0.0,
    )
    carriage = (
        carriage.faces("<Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, X_RAIL_Z), (0.0, -X_RAIL_Z)])
        .rect(BEAM_GROOVE_LEN, X_RAIL_H + 0.008)
        .cutBlind(-BEAM_GROOVE_DEPTH)
    )

    center_spine = _box_back(0.19, 0.032, 0.18, back_y=BEAM_CARRIAGE_T, z=0.010)
    left_web = _box_back(0.020, 0.040, 0.130, x=-0.102, back_y=BEAM_CARRIAGE_T, z=0.015)
    right_web = _box_back(0.020, 0.040, 0.130, x=0.102, back_y=BEAM_CARRIAGE_T, z=0.015)
    x_sensor_tab = _box_back(0.014, 0.010, 0.060, x=0.140, back_y=0.006, z=0.195)
    return _fuse(carriage, center_spine, left_web, right_web, x_sensor_tab)


def _beam_body_shape() -> cq.Workplane:
    beam = _box_back(BEAM_BODY_L, BEAM_BODY_D, BEAM_BODY_H, back_y=0.032, z=0.040)
    beam = beam.edges("|X").fillet(0.004)
    lower_wear_strip = _box_back(BEAM_BODY_L - 0.070, 0.010, 0.020, back_y=0.032, z=-0.030)
    return _fuse(beam, lower_wear_strip)


def _guide_mast_shape() -> cq.Workplane:
    mast = _box_back(
        GUIDE_MAST_W,
        GUIDE_MAST_D,
        GUIDE_MAST_H,
        back_y=GUIDE_MAST_BACK_Y,
        z=GUIDE_MAST_Z,
    )
    shoulder = _box_back(0.132, 0.052, 0.024, back_y=0.042, z=-0.022)
    slide_sensor_block = _box_back(0.026, 0.016, 0.064, x=0.064, back_y=0.078, z=-0.048)
    return _fuse(mast, shoulder, slide_sensor_block)


def _slide_carriage_shape() -> cq.Workplane:
    carriage = _box_back(
        SLIDE_CARRIAGE_W,
        SLIDE_CARRIAGE_D,
        SLIDE_CARRIAGE_H,
        back_y=0.0,
        z=-(SLIDE_CARRIAGE_H * 0.5),
    )
    carriage = (
        carriage.faces("<Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-GUIDE_RAIL_X, -0.090), (GUIDE_RAIL_X, -0.090)])
        .rect(GUIDE_RAIL_W + 0.008, 0.185)
        .cutBlind(-SLIDE_GROOVE_DEPTH)
    )
    z_sensor_tab = _box_back(0.014, 0.012, 0.052, x=-0.072, back_y=0.012, z=-0.060)
    return _fuse(carriage, z_sensor_tab)


def _slide_hanger_shape() -> cq.Workplane:
    front_plate = _box_back(0.155, 0.016, 0.320, back_y=0.056, z=-0.260)
    bridge = _box_back(0.092, 0.072, 0.060, back_y=0.000, z=-0.125)
    left_side = _box_back(0.018, 0.060, 0.200, x=-0.052, back_y=0.006, z=-0.240)
    right_side = _box_back(0.018, 0.060, 0.200, x=0.052, back_y=0.006, z=-0.240)
    foot = _box_back(0.145, 0.060, 0.045, back_y=0.006, z=-0.422)
    return _fuse(front_plate, bridge, left_side, right_side, foot)


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    name: str,
    material: str,
) -> None:
    part.visual(mesh_from_cadquery(shape, name), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_service_panel_axis")

    model.material("frame_blue", rgba=(0.24, 0.29, 0.35, 1.0))
    model.material("rail_steel", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("beam_alloy", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("stage_dark", rgba=(0.26, 0.28, 0.31, 1.0))

    back_frame = model.part("back_frame")
    _add_mesh_visual(back_frame, _frame_body_shape(), "frame_body", "frame_blue")
    back_frame.visual(
        Box((X_RAIL_LEN, X_RAIL_T, X_RAIL_H)),
        origin=Origin(xyz=(0.0, X_RAIL_T * 0.5, X_RAIL_Z)),
        material="rail_steel",
        name="upper_rail",
    )
    back_frame.visual(
        Box((X_RAIL_LEN, X_RAIL_T, X_RAIL_H)),
        origin=Origin(xyz=(0.0, X_RAIL_T * 0.5, -X_RAIL_Z)),
        material="rail_steel",
        name="lower_rail",
    )
    back_frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_PANEL_T + FRAME_REAR_DEPTH, FRAME_H)),
        mass=28.0,
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
    )

    beam_stage = model.part("beam_stage")
    _add_mesh_visual(beam_stage, _beam_carriage_shape(), "carriage_frame", "stage_dark")
    _add_mesh_visual(beam_stage, _beam_body_shape(), "beam_body", "beam_alloy")
    _add_mesh_visual(beam_stage, _guide_mast_shape(), "guide_mast", "stage_dark")
    beam_stage.visual(
        Box((GUIDE_RAIL_W, GUIDE_RAIL_T, GUIDE_RAIL_H)),
        origin=Origin(xyz=(-GUIDE_RAIL_X, GUIDE_RAIL_CENTER_Y, GUIDE_RAIL_CENTER_Z)),
        material="rail_steel",
        name="guide_left_rail",
    )
    beam_stage.visual(
        Box((GUIDE_RAIL_W, GUIDE_RAIL_T, GUIDE_RAIL_H)),
        origin=Origin(xyz=(GUIDE_RAIL_X, GUIDE_RAIL_CENTER_Y, GUIDE_RAIL_CENTER_Z)),
        material="rail_steel",
        name="guide_right_rail",
    )
    beam_stage.inertial = Inertial.from_geometry(
        Box((BEAM_BODY_L, 0.124, 0.50)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.062, -0.160)),
    )

    z_slide = model.part("z_slide")
    _add_mesh_visual(z_slide, _slide_carriage_shape(), "slide_carriage", "stage_dark")
    _add_mesh_visual(z_slide, _slide_hanger_shape(), "slide_hanger", "beam_alloy")
    z_slide.inertial = Inertial.from_geometry(
        Box((0.155, 0.072, 0.445)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.036, -0.222)),
    )

    model.articulation(
        "frame_to_beam",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=beam_stage,
        origin=Origin(xyz=(0.0, X_STAGE_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=2200.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "beam_to_slide",
        ArticulationType.PRISMATIC,
        parent=beam_stage,
        child=z_slide,
        origin=Origin(xyz=(0.0, Z_STAGE_ORIGIN_Y, Z_STAGE_ORIGIN_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=900.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_frame = object_model.get_part("back_frame")
    beam_stage = object_model.get_part("beam_stage")
    z_slide = object_model.get_part("z_slide")
    x_stage = object_model.get_articulation("frame_to_beam")
    z_stage = object_model.get_articulation("beam_to_slide")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "x_stage_axis_is_lateral",
        tuple(round(v, 3) for v in x_stage.axis) == (1.0, 0.0, 0.0),
        f"expected beam stage axis (1,0,0), got {x_stage.axis}",
    )
    ctx.check(
        "z_stage_axis_is_vertical",
        tuple(round(v, 3) for v in z_stage.axis) == (0.0, 0.0, -1.0),
        f"expected z slide axis (0,0,-1), got {z_stage.axis}",
    )

    ctx.expect_contact(
        back_frame,
        beam_stage,
        elem_a="upper_rail",
        elem_b="carriage_frame",
        contact_tol=0.001,
        name="beam_carriage_contacts_upper_rail",
    )
    ctx.expect_contact(
        back_frame,
        beam_stage,
        elem_a="lower_rail",
        elem_b="carriage_frame",
        contact_tol=0.001,
        name="beam_carriage_contacts_lower_rail",
    )
    ctx.expect_contact(
        beam_stage,
        z_slide,
        elem_a="guide_left_rail",
        elem_b="slide_carriage",
        contact_tol=0.001,
        name="z_slide_contacts_left_guide",
    )
    ctx.expect_contact(
        beam_stage,
        z_slide,
        elem_a="guide_right_rail",
        elem_b="slide_carriage",
        contact_tol=0.001,
        name="z_slide_contacts_right_guide",
    )

    for beam_pos, label in ((-X_TRAVEL, "left"), (X_TRAVEL, "right")):
        with ctx.pose({x_stage: beam_pos}):
            ctx.expect_overlap(
                beam_stage,
                back_frame,
                axes="x",
                elem_a="carriage_frame",
                elem_b="upper_rail",
                min_overlap=0.22,
                name=f"upper_rail_overlap_at_{label}_travel",
            )
            ctx.expect_overlap(
                beam_stage,
                back_frame,
                axes="x",
                elem_a="carriage_frame",
                elem_b="lower_rail",
                min_overlap=0.22,
                name=f"lower_rail_overlap_at_{label}_travel",
            )
            ctx.expect_gap(
                beam_stage,
                back_frame,
                axis="y",
                positive_elem="beam_body",
                negative_elem="upper_rail",
                min_gap=0.018,
                max_gap=0.040,
                name=f"beam_body_clears_rails_at_{label}_travel",
            )

    ctx.expect_gap(
        z_slide,
        back_frame,
        axis="y",
        positive_elem="slide_hanger",
        negative_elem="frame_body",
        min_gap=0.11,
        name="hanging_slide_stays_forward_of_back_frame",
    )

    with ctx.pose({z_stage: 0.0}):
        ctx.expect_overlap(
            z_slide,
            beam_stage,
            axes="z",
            elem_a="slide_carriage",
            elem_b="guide_left_rail",
            min_overlap=0.14,
            name="left_guide_overlap_at_top",
        )
        ctx.expect_overlap(
            z_slide,
            beam_stage,
            axes="z",
            elem_a="slide_carriage",
            elem_b="guide_right_rail",
            min_overlap=0.14,
            name="right_guide_overlap_at_top",
        )
        ctx.expect_gap(
            beam_stage,
            z_slide,
            axis="z",
            positive_elem="beam_body",
            negative_elem="slide_hanger",
            min_gap=0.060,
            max_gap=0.110,
            name="beam_body_clears_slide_hanger_at_top",
        )

    with ctx.pose({z_stage: Z_TRAVEL}):
        ctx.expect_overlap(
            z_slide,
            beam_stage,
            axes="z",
            elem_a="slide_carriage",
            elem_b="guide_left_rail",
            min_overlap=0.12,
            name="left_guide_overlap_at_bottom",
        )
        ctx.expect_overlap(
            z_slide,
            beam_stage,
            axes="z",
            elem_a="slide_carriage",
            elem_b="guide_right_rail",
            min_overlap=0.12,
            name="right_guide_overlap_at_bottom",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
