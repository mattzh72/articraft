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


BASE_L = 0.420
BASE_W = 0.240
BASE_T = 0.028

LOWER_RAIL_L = 0.340
LOWER_RAIL_W = 0.024
LOWER_RAIL_H = 0.014
LOWER_RAIL_Y = 0.060

LOWER_CARRIAGE_L = 0.250
LOWER_CARRIAGE_W = 0.190
LOWER_CARRIAGE_H = 0.050
LOWER_TRAVEL = 0.075

LOWER_STOP_X = 0.175
LOWER_STOP_Y = 0.108
LOWER_STOP_XSZ = 0.032
LOWER_STOP_YSZ = 0.018
LOWER_STOP_ZSZ = 0.022

UPPER_RAIL_X = 0.036
UPPER_RAIL_W = 0.018
UPPER_RAIL_L = 0.150
UPPER_RAIL_H = 0.011

UPPER_SLIDE_X = 0.140
UPPER_SLIDE_Y = 0.110
UPPER_SLIDE_H = 0.036
UPPER_TRAVEL = 0.030

UPPER_STOP_X = 0.086
UPPER_STOP_Y = 0.095
UPPER_STOP_XSZ = 0.018
UPPER_STOP_YSZ = 0.018
UPPER_STOP_ZSZ = 0.018

PAD_X = 0.170
PAD_Y = 0.125
PAD_TOP_T = 0.016
PAD_PEDESTAL_T = 0.008
PAD_PED_X = 0.116
PAD_PED_Y = 0.080
PAD_TOTAL_H = PAD_TOP_T + PAD_PEDESTAL_T

CONTACT_TOL = 5e-4


def _linear_positions(count: int, span: float) -> list[float]:
    if count == 1:
        return [0.0]
    step = span / (count - 1)
    start = -span / 2.0
    return [start + (step * i) for i in range(count)]


def _cylinder_cut_array(
    body: cq.Workplane,
    points: list[tuple[float, float]],
    *,
    radius: float,
    depth: float,
    top_z: float,
) -> cq.Workplane:
    cutter = (
        cq.Workplane("XY")
        .pushPoints(points)
        .cylinder(depth, radius, centered=(True, True, False))
        .translate((0.0, 0.0, top_z - depth))
    )
    return body.cut(cutter)


def _box_at(size_x: float, size_y: float, size_z: float, *, z0: float, x: float = 0.0, y: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((x, y, z0))
    )


def _base_plate_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    body = body.cut(_box_at(0.292, 0.062, 0.006, z0=BASE_T - 0.006))
    body = body.cut(_box_at(0.360, 0.164, 0.004, z0=0.004))

    corner_pts = [(sx, sy) for sx in (-0.165, 0.165) for sy in (-0.085, 0.085)]
    body = _cylinder_cut_array(body, corner_pts, radius=0.0075, depth=0.004, top_z=BASE_T)
    body = _cylinder_cut_array(body, corner_pts, radius=0.0045, depth=BASE_T, top_z=BASE_T)
    return body


def _rail_shape(length: float, width: float, height: float, axis: str) -> cq.Workplane:
    rail = cq.Workplane("XY").box(length, width, height * 0.58, centered=(True, True, False))
    rail = rail.union(_box_at(length, width * 0.72, height * 0.42, z0=height * 0.58))
    if axis == "x":
        hole_pts = [(x, 0.0) for x in _linear_positions(6, length - 0.080)]
    else:
        hole_pts = [(0.0, y) for y in _linear_positions(4, length - 0.050)]
    rail = _cylinder_cut_array(rail, hole_pts, radius=0.0042, depth=0.003, top_z=height)
    rail = _cylinder_cut_array(rail, hole_pts, radius=0.0022, depth=height, top_z=height)
    return rail


def _stop_block_shape(size_x: float, size_y: float, size_z: float) -> cq.Workplane:
    block = cq.Workplane("XY").box(size_x, size_y, size_z, centered=(True, True, False))
    block = block.cut(_box_at(size_x * 0.52, size_y * 0.28, size_z * 0.40, z0=size_z * 0.60, y=size_y * 0.18))
    screw_pitch = size_x * 0.44
    screw_pts = [(-screw_pitch / 2.0, 0.0), (screw_pitch / 2.0, 0.0)]
    block = _cylinder_cut_array(block, screw_pts, radius=0.0038, depth=0.003, top_z=size_z)
    block = _cylinder_cut_array(block, screw_pts, radius=0.0022, depth=size_z, top_z=size_z)
    return block


def _lower_carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        LOWER_CARRIAGE_L,
        LOWER_CARRIAGE_W,
        LOWER_CARRIAGE_H,
        centered=(True, True, False),
    )
    body = body.cut(_box_at(LOWER_CARRIAGE_L - 0.036, 0.074, 0.018, z0=0.0))
    body = body.cut(_box_at(0.050, 0.150, 0.007, z0=LOWER_CARRIAGE_H - 0.007))
    body = body.cut(_box_at(0.034, 0.128, 0.005, z0=LOWER_CARRIAGE_H - 0.005, x=-0.080))
    body = body.cut(_box_at(0.034, 0.128, 0.005, z0=LOWER_CARRIAGE_H - 0.005, x=0.080))
    body = body.cut(_box_at(0.060, 0.032, 0.006, z0=LOWER_CARRIAGE_H - 0.006, y=-0.068))
    body = body.cut(_box_at(0.060, 0.032, 0.006, z0=LOWER_CARRIAGE_H - 0.006, y=0.068))

    pad_pts = [(-0.094, -0.065), (-0.094, 0.065), (0.094, -0.065), (0.094, 0.065)]
    body = _cylinder_cut_array(body, pad_pts, radius=0.0032, depth=0.005, top_z=LOWER_CARRIAGE_H)
    return body


def _cross_slide_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        UPPER_SLIDE_X,
        UPPER_SLIDE_Y,
        UPPER_SLIDE_H,
        centered=(True, True, False),
    )
    body = body.cut(_box_at(0.048, 0.082, 0.014, z0=0.0))
    body = body.cut(_box_at(0.074, 0.034, 0.005, z0=UPPER_SLIDE_H - 0.005, y=-0.034))
    body = body.cut(_box_at(0.074, 0.034, 0.005, z0=UPPER_SLIDE_H - 0.005, y=0.034))
    body = body.cut(_box_at(0.038, 0.060, 0.006, z0=UPPER_SLIDE_H - 0.006, x=-0.040))
    body = body.cut(_box_at(0.038, 0.060, 0.006, z0=UPPER_SLIDE_H - 0.006, x=0.040))

    top_hole_pts = [(-0.052, -0.045), (-0.052, 0.045), (0.052, -0.045), (0.052, 0.045)]
    body = _cylinder_cut_array(body, top_hole_pts, radius=0.0030, depth=0.0045, top_z=UPPER_SLIDE_H)
    return body


def _instrument_pad_shape() -> cq.Workplane:
    pedestal = cq.Workplane("XY").box(PAD_PED_X, PAD_PED_Y, PAD_PEDESTAL_T, centered=(True, True, False))
    top_plate = (
        cq.Workplane("XY")
        .box(PAD_X, PAD_Y, PAD_TOP_T, centered=(True, True, False))
        .translate((0.0, 0.0, PAD_PEDESTAL_T))
    )
    pad = pedestal.union(top_plate)

    hole_pts = [(x, y) for x in (-0.045, 0.0, 0.045) for y in (-0.028, 0.028)]
    pad = _cylinder_cut_array(pad, hole_pts, radius=0.0033, depth=0.006, top_z=PAD_TOTAL_H)
    pad = pad.cut(_box_at(0.120, 0.072, 0.005, z0=PAD_PEDESTAL_T + 0.002))
    return pad


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_slide_table")

    model.material("base_iron", rgba=(0.28, 0.30, 0.32, 1.0))
    model.material("ground_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("machined_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("anodized_black", rgba=(0.18, 0.19, 0.21, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="base_iron",
        name="base_plate_body",
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    lower_way_left = model.part("lower_way_left")
    lower_way_left.visual(
        mesh_from_cadquery(_rail_shape(LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H, "x"), "lower_way_left"),
        material="ground_steel",
        name="lower_way_left_body",
    )
    lower_way_left.inertial = Inertial.from_geometry(
        Box((LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, LOWER_RAIL_H / 2.0)),
    )

    lower_way_right = model.part("lower_way_right")
    lower_way_right.visual(
        mesh_from_cadquery(_rail_shape(LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H, "x"), "lower_way_right"),
        material="ground_steel",
        name="lower_way_right_body",
    )
    lower_way_right.inertial = Inertial.from_geometry(
        Box((LOWER_RAIL_L, LOWER_RAIL_W, LOWER_RAIL_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, LOWER_RAIL_H / 2.0)),
    )

    lower_stop_neg = model.part("lower_stop_neg")
    lower_stop_neg.visual(
        mesh_from_cadquery(
            _stop_block_shape(LOWER_STOP_XSZ, LOWER_STOP_YSZ, LOWER_STOP_ZSZ),
            "lower_stop_neg",
        ),
        material="anodized_black",
        name="lower_stop_neg_body",
    )
    lower_stop_neg.inertial = Inertial.from_geometry(
        Box((LOWER_STOP_XSZ, LOWER_STOP_YSZ, LOWER_STOP_ZSZ)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STOP_ZSZ / 2.0)),
    )

    lower_stop_pos = model.part("lower_stop_pos")
    lower_stop_pos.visual(
        mesh_from_cadquery(
            _stop_block_shape(LOWER_STOP_XSZ, LOWER_STOP_YSZ, LOWER_STOP_ZSZ),
            "lower_stop_pos",
        ),
        material="anodized_black",
        name="lower_stop_pos_body",
    )
    lower_stop_pos.inertial = Inertial.from_geometry(
        Box((LOWER_STOP_XSZ, LOWER_STOP_YSZ, LOWER_STOP_ZSZ)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STOP_ZSZ / 2.0)),
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "lower_carriage"),
        material="machined_aluminum",
        name="lower_carriage_body",
    )
    lower_carriage.inertial = Inertial.from_geometry(
        Box((LOWER_CARRIAGE_L, LOWER_CARRIAGE_W, LOWER_CARRIAGE_H)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, LOWER_CARRIAGE_H / 2.0)),
    )

    upper_way_left = model.part("upper_way_left")
    upper_way_left.visual(
        mesh_from_cadquery(_rail_shape(UPPER_RAIL_W, UPPER_RAIL_L, UPPER_RAIL_H, "y"), "upper_way_left"),
        material="ground_steel",
        name="upper_way_left_body",
    )
    upper_way_left.inertial = Inertial.from_geometry(
        Box((UPPER_RAIL_W, UPPER_RAIL_L, UPPER_RAIL_H)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, UPPER_RAIL_H / 2.0)),
    )

    upper_way_right = model.part("upper_way_right")
    upper_way_right.visual(
        mesh_from_cadquery(_rail_shape(UPPER_RAIL_W, UPPER_RAIL_L, UPPER_RAIL_H, "y"), "upper_way_right"),
        material="ground_steel",
        name="upper_way_right_body",
    )
    upper_way_right.inertial = Inertial.from_geometry(
        Box((UPPER_RAIL_W, UPPER_RAIL_L, UPPER_RAIL_H)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, UPPER_RAIL_H / 2.0)),
    )

    upper_stop_neg = model.part("upper_stop_neg")
    upper_stop_neg.visual(
        mesh_from_cadquery(
            _stop_block_shape(UPPER_STOP_XSZ, UPPER_STOP_YSZ, UPPER_STOP_ZSZ),
            "upper_stop_neg",
        ),
        material="anodized_black",
        name="upper_stop_neg_body",
    )
    upper_stop_neg.inertial = Inertial.from_geometry(
        Box((UPPER_STOP_XSZ, UPPER_STOP_YSZ, UPPER_STOP_ZSZ)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STOP_ZSZ / 2.0)),
    )

    upper_stop_pos = model.part("upper_stop_pos")
    upper_stop_pos.visual(
        mesh_from_cadquery(
            _stop_block_shape(UPPER_STOP_XSZ, UPPER_STOP_YSZ, UPPER_STOP_ZSZ),
            "upper_stop_pos",
        ),
        material="anodized_black",
        name="upper_stop_pos_body",
    )
    upper_stop_pos.inertial = Inertial.from_geometry(
        Box((UPPER_STOP_XSZ, UPPER_STOP_YSZ, UPPER_STOP_ZSZ)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STOP_ZSZ / 2.0)),
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        mesh_from_cadquery(_cross_slide_shape(), "cross_slide"),
        material="machined_aluminum",
        name="cross_slide_body",
    )
    cross_slide.inertial = Inertial.from_geometry(
        Box((UPPER_SLIDE_X, UPPER_SLIDE_Y, UPPER_SLIDE_H)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SLIDE_H / 2.0)),
    )

    instrument_pad = model.part("instrument_pad")
    instrument_pad.visual(
        mesh_from_cadquery(_instrument_pad_shape(), "instrument_pad"),
        material="anodized_black",
        name="instrument_pad_body",
    )
    instrument_pad.inertial = Inertial.from_geometry(
        Box((PAD_X, PAD_Y, PAD_TOTAL_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, PAD_TOTAL_H / 2.0)),
    )

    model.articulation(
        "base_to_lower_way_left",
        ArticulationType.FIXED,
        parent=base_plate,
        child=lower_way_left,
        origin=Origin(xyz=(0.0, -LOWER_RAIL_Y, BASE_T)),
    )
    model.articulation(
        "base_to_lower_way_right",
        ArticulationType.FIXED,
        parent=base_plate,
        child=lower_way_right,
        origin=Origin(xyz=(0.0, LOWER_RAIL_Y, BASE_T)),
    )
    model.articulation(
        "base_to_lower_stop_neg",
        ArticulationType.FIXED,
        parent=base_plate,
        child=lower_stop_neg,
        origin=Origin(xyz=(-LOWER_STOP_X, LOWER_STOP_Y, BASE_T)),
    )
    model.articulation(
        "base_to_lower_stop_pos",
        ArticulationType.FIXED,
        parent=base_plate,
        child=lower_stop_pos,
        origin=Origin(xyz=(LOWER_STOP_X, LOWER_STOP_Y, BASE_T)),
    )
    model.articulation(
        "base_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=base_plate,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + LOWER_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_TRAVEL,
            upper=LOWER_TRAVEL,
            effort=400.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "lower_carriage_to_upper_way_left",
        ArticulationType.FIXED,
        parent=lower_carriage,
        child=upper_way_left,
        origin=Origin(xyz=(-UPPER_RAIL_X, 0.0, LOWER_CARRIAGE_H)),
    )
    model.articulation(
        "lower_carriage_to_upper_way_right",
        ArticulationType.FIXED,
        parent=lower_carriage,
        child=upper_way_right,
        origin=Origin(xyz=(UPPER_RAIL_X, 0.0, LOWER_CARRIAGE_H)),
    )
    model.articulation(
        "lower_carriage_to_upper_stop_neg",
        ArticulationType.FIXED,
        parent=lower_carriage,
        child=upper_stop_neg,
        origin=Origin(xyz=(UPPER_STOP_X, -UPPER_STOP_Y, LOWER_CARRIAGE_H)),
    )
    model.articulation(
        "lower_carriage_to_upper_stop_pos",
        ArticulationType.FIXED,
        parent=lower_carriage,
        child=upper_stop_pos,
        origin=Origin(xyz=(UPPER_STOP_X, UPPER_STOP_Y, LOWER_CARRIAGE_H)),
    )
    model.articulation(
        "lower_carriage_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, LOWER_CARRIAGE_H + UPPER_RAIL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-UPPER_TRAVEL,
            upper=UPPER_TRAVEL,
            effort=220.0,
            velocity=0.12,
        ),
    )
    model.articulation(
        "cross_slide_to_instrument_pad",
        ArticulationType.FIXED,
        parent=cross_slide,
        child=instrument_pad,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SLIDE_H)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    lower_way_left = object_model.get_part("lower_way_left")
    lower_way_right = object_model.get_part("lower_way_right")
    lower_stop_neg = object_model.get_part("lower_stop_neg")
    lower_stop_pos = object_model.get_part("lower_stop_pos")
    lower_carriage = object_model.get_part("lower_carriage")
    upper_way_left = object_model.get_part("upper_way_left")
    upper_way_right = object_model.get_part("upper_way_right")
    upper_stop_neg = object_model.get_part("upper_stop_neg")
    upper_stop_pos = object_model.get_part("upper_stop_pos")
    cross_slide = object_model.get_part("cross_slide")
    instrument_pad = object_model.get_part("instrument_pad")

    lower_axis = object_model.get_articulation("base_to_lower_carriage")
    upper_axis = object_model.get_articulation("lower_carriage_to_cross_slide")

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
        "lower_axis_is_prismatic_x",
        lower_axis.articulation_type == ArticulationType.PRISMATIC and tuple(lower_axis.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis prismatic lower stage, got type={lower_axis.articulation_type}, axis={lower_axis.axis}",
    )
    ctx.check(
        "upper_axis_is_prismatic_y",
        upper_axis.articulation_type == ArticulationType.PRISMATIC and tuple(upper_axis.axis) == (0.0, 1.0, 0.0),
        details=f"expected y-axis prismatic cross-slide, got type={upper_axis.articulation_type}, axis={upper_axis.axis}",
    )

    ctx.expect_contact(lower_way_left, base_plate, contact_tol=CONTACT_TOL, name="lower_way_left_mounted_to_base")
    ctx.expect_contact(lower_way_right, base_plate, contact_tol=CONTACT_TOL, name="lower_way_right_mounted_to_base")
    ctx.expect_contact(lower_stop_neg, base_plate, contact_tol=CONTACT_TOL, name="lower_negative_stop_mounted_to_base")
    ctx.expect_contact(lower_stop_pos, base_plate, contact_tol=CONTACT_TOL, name="lower_positive_stop_mounted_to_base")
    ctx.expect_contact(lower_carriage, lower_way_left, contact_tol=CONTACT_TOL, name="lower_carriage_supported_on_left_way")
    ctx.expect_contact(lower_carriage, lower_way_right, contact_tol=CONTACT_TOL, name="lower_carriage_supported_on_right_way")

    ctx.expect_contact(upper_way_left, lower_carriage, contact_tol=CONTACT_TOL, name="upper_way_left_mounted_to_lower_carriage")
    ctx.expect_contact(upper_way_right, lower_carriage, contact_tol=CONTACT_TOL, name="upper_way_right_mounted_to_lower_carriage")
    ctx.expect_contact(upper_stop_neg, lower_carriage, contact_tol=CONTACT_TOL, name="upper_negative_stop_mounted_to_lower_carriage")
    ctx.expect_contact(upper_stop_pos, lower_carriage, contact_tol=CONTACT_TOL, name="upper_positive_stop_mounted_to_lower_carriage")
    ctx.expect_contact(cross_slide, upper_way_left, contact_tol=CONTACT_TOL, name="cross_slide_supported_on_left_upper_way")
    ctx.expect_contact(cross_slide, upper_way_right, contact_tol=CONTACT_TOL, name="cross_slide_supported_on_right_upper_way")
    ctx.expect_contact(instrument_pad, cross_slide, contact_tol=CONTACT_TOL, name="instrument_pad_grounded_to_cross_slide")

    with ctx.pose({lower_axis: LOWER_TRAVEL}):
        ctx.expect_overlap(
            lower_carriage,
            lower_way_left,
            axes="x",
            min_overlap=0.200,
            name="lower_carriage_retains_left_way_overlap_at_positive_limit",
        )
        ctx.expect_overlap(
            lower_carriage,
            lower_way_right,
            axes="x",
            min_overlap=0.200,
            name="lower_carriage_retains_right_way_overlap_at_positive_limit",
        )
        ctx.expect_gap(
            lower_stop_pos,
            lower_carriage,
            axis="y",
            min_gap=0.003,
            name="lower_positive_stop_stays_clear_of_carriage",
        )

    with ctx.pose({lower_axis: -LOWER_TRAVEL}):
        ctx.expect_overlap(
            lower_carriage,
            lower_way_left,
            axes="x",
            min_overlap=0.200,
            name="lower_carriage_retains_left_way_overlap_at_negative_limit",
        )
        ctx.expect_overlap(
            lower_carriage,
            lower_way_right,
            axes="x",
            min_overlap=0.200,
            name="lower_carriage_retains_right_way_overlap_at_negative_limit",
        )
        ctx.expect_gap(
            lower_stop_neg,
            lower_carriage,
            axis="y",
            min_gap=0.003,
            name="lower_negative_stop_stays_clear_of_carriage",
        )

    with ctx.pose({upper_axis: UPPER_TRAVEL}):
        ctx.expect_overlap(
            cross_slide,
            upper_way_left,
            axes="y",
            min_overlap=0.098,
            name="cross_slide_retains_left_upper_way_overlap_at_positive_limit",
        )
        ctx.expect_overlap(
            cross_slide,
            upper_way_right,
            axes="y",
            min_overlap=0.098,
            name="cross_slide_retains_right_upper_way_overlap_at_positive_limit",
        )
        ctx.expect_gap(
            upper_stop_pos,
            cross_slide,
            axis="x",
            min_gap=0.006,
            name="upper_positive_stop_stays_clear_of_cross_slide",
        )

    with ctx.pose({upper_axis: -UPPER_TRAVEL}):
        ctx.expect_overlap(
            cross_slide,
            upper_way_left,
            axes="y",
            min_overlap=0.098,
            name="cross_slide_retains_left_upper_way_overlap_at_negative_limit",
        )
        ctx.expect_overlap(
            cross_slide,
            upper_way_right,
            axes="y",
            min_overlap=0.098,
            name="cross_slide_retains_right_upper_way_overlap_at_negative_limit",
        )
        ctx.expect_gap(
            upper_stop_neg,
            cross_slide,
            axis="x",
            min_gap=0.006,
            name="upper_negative_stop_stays_clear_of_cross_slide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
