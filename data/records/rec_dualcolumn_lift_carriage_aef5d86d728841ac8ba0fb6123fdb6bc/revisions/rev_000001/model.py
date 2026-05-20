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


MAST_HEIGHT = 1.52
COLUMN_BASE_Z = 0.03
COLUMN_HEIGHT = 1.46
COLUMN_CENTER_X = 0.185
CHANNEL_WIDTH = 0.095
CHANNEL_DEPTH = 0.094
WEB_THICKNESS = 0.012
FLANGE_THICKNESS = 0.012

LOWER_CROSSMEMBER_SIZE = (0.314, 0.052, 0.094)
LOWER_CROSSMEMBER_CENTER_Z = 0.057
UPPER_CROSSHEAD_SIZE = (0.320, 0.060, 0.094)
UPPER_CROSSHEAD_CENTER_Z = 1.463

GUSSET_THICKNESS = 0.006
GUSSET_REACH = 0.056
GUSSET_RISE = 0.110
GUSSET_Y = 0.039

WEAR_STRIP_THICKNESS = 0.010
WEAR_STRIP_DEPTH = 0.028
WEAR_STRIP_HEIGHT = 1.14
WEAR_STRIP_CENTER_Z = 0.76
BOLT_HEAD_RADIUS = 0.007
BOLT_HEAD_LENGTH = 0.0025
WEAR_STRIP_BOLT_Z = (-0.39, -0.13, 0.13, 0.39)

SIDE_PLATE_THICKNESS = 0.018
SIDE_PLATE_DEPTH = 0.032
SIDE_PLATE_HEIGHT = 0.500
SIDE_PLATE_CENTER_X = 0.127

CARRIAGE_HEIGHT = 0.560
CARRIAGE_DEPTH = 0.036
CROSSBAR_THICKNESS = 0.060
CROSSBAR_LENGTH = 0.255
CROSSBAR_CENTER_Z = 0.250
RIB_WIDTH = 0.014
RIB_DEPTH = 0.014
RIB_HEIGHT = 0.360
RIB_CENTER_X = 0.060

GUIDE_CLUSTER_CENTER_Z = 0.195
GUIDE_CLUSTER_HEIGHT = 0.090
GUIDE_BACKING_X = 0.168
GUIDE_BACKING_SIZE = (0.020, 0.014, GUIDE_CLUSTER_HEIGHT)
GUIDE_PAD_X = 0.1945
GUIDE_PAD_Y = 0.018
GUIDE_PAD_SIZE = (0.032, 0.024, GUIDE_CLUSTER_HEIGHT)
GUIDE_ARM_X = 0.148
GUIDE_ARM_SIZE = (0.040, 0.018, GUIDE_CLUSTER_HEIGHT)

CARRIAGE_HOME_Z = 0.410
CARRIAGE_TRAVEL = 0.640


def _union_shapes(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _centered_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _channel_shape(open_to_inboard: bool) -> cq.Workplane:
    web_center_x = (
        -(CHANNEL_WIDTH / 2.0) + (WEB_THICKNESS / 2.0)
        if open_to_inboard
        else (CHANNEL_WIDTH / 2.0) - (WEB_THICKNESS / 2.0)
    )
    flange_center_x = (WEB_THICKNESS / 2.0) if open_to_inboard else -(WEB_THICKNESS / 2.0)
    web = _centered_box(
        (WEB_THICKNESS, CHANNEL_DEPTH, COLUMN_HEIGHT),
        (web_center_x, 0.0, 0.0),
    )
    front_flange = _centered_box(
        (CHANNEL_WIDTH - WEB_THICKNESS, FLANGE_THICKNESS, COLUMN_HEIGHT),
        (flange_center_x, (CHANNEL_DEPTH / 2.0) - (FLANGE_THICKNESS / 2.0), 0.0),
    )
    rear_flange = _centered_box(
        (CHANNEL_WIDTH - WEB_THICKNESS, FLANGE_THICKNESS, COLUMN_HEIGHT),
        (flange_center_x, -(CHANNEL_DEPTH / 2.0) + (FLANGE_THICKNESS / 2.0), 0.0),
    )
    return _union_shapes(web, front_flange, rear_flange)


def _crossmember_shape(size: tuple[float, float, float], center_z: float) -> cq.Workplane:
    return _centered_box(size, (0.0, 0.0, center_z))


def _lower_crossmember_shape() -> cq.Workplane:
    body = _crossmember_shape(LOWER_CROSSMEMBER_SIZE, LOWER_CROSSMEMBER_CENTER_Z)
    connector_thickness_x = 0.040
    connector_depth_y = 0.018
    flange_y = (CHANNEL_DEPTH / 2.0) - (FLANGE_THICKNESS / 2.0)
    connector_z = LOWER_CROSSMEMBER_CENTER_Z
    connectors = []
    for sign_x in (-1.0, 1.0):
        connector_x = sign_x * ((LOWER_CROSSMEMBER_SIZE[0] / 2.0) + (connector_thickness_x / 2.0))
        for sign_y in (-1.0, 1.0):
            connectors.append(
                _centered_box(
                    (connector_thickness_x, connector_depth_y, LOWER_CROSSMEMBER_SIZE[2]),
                    (connector_x, sign_y * flange_y, connector_z),
                )
            )
    return _union_shapes(body, *connectors)


def _gusset_plate(sign_x: float, lower: bool, front: bool) -> cq.Workplane:
    inner_face_x = sign_x * (COLUMN_CENTER_X - (CHANNEL_WIDTH / 2.0) + 0.010)
    tip_x = inner_face_x - (sign_x * GUSSET_REACH)
    if lower:
        base_z = LOWER_CROSSMEMBER_CENTER_Z + (LOWER_CROSSMEMBER_SIZE[2] / 2.0) - 0.004
        tip_z = base_z + GUSSET_RISE
    else:
        base_z = UPPER_CROSSHEAD_CENTER_Z - (UPPER_CROSSHEAD_SIZE[2] / 2.0) + 0.004
        tip_z = base_z - GUSSET_RISE
    y_center = GUSSET_Y if front else -GUSSET_Y
    return (
        cq.Workplane("XZ")
        .polyline([(inner_face_x, base_z), (tip_x, base_z), (inner_face_x, tip_z)])
        .close()
        .extrude(GUSSET_THICKNESS)
        .translate((0.0, y_center - (GUSSET_THICKNESS / 2.0), 0.0))
    )


def _mast_gussets_shape() -> cq.Workplane:
    plates = [
        _gusset_plate(sign_x, lower, front)
        for sign_x in (-1.0, 1.0)
        for lower in (True, False)
        for front in (True, False)
    ]
    return _union_shapes(*plates)


def _wear_strip_shape(outboard_positive_x: bool) -> cq.Workplane:
    strip = cq.Workplane("XY").box(WEAR_STRIP_THICKNESS, WEAR_STRIP_DEPTH, WEAR_STRIP_HEIGHT)
    bolt_origin_x = (WEAR_STRIP_THICKNESS / 2.0) if outboard_positive_x else (-(WEAR_STRIP_THICKNESS / 2.0) - BOLT_HEAD_LENGTH)
    bolts = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, z) for z in WEAR_STRIP_BOLT_Z])
        .circle(BOLT_HEAD_RADIUS)
        .extrude(BOLT_HEAD_LENGTH)
        .translate((bolt_origin_x, 0.0, 0.0))
    )
    return strip.union(bolts)


def _guide_cluster(sign_x: float, z_center: float) -> cq.Workplane:
    backing = _centered_box(
        GUIDE_BACKING_SIZE,
        (sign_x * GUIDE_BACKING_X, 0.0, z_center),
    )
    arm = _centered_box(
        GUIDE_ARM_SIZE,
        (sign_x * GUIDE_ARM_X, 0.0, z_center),
    )
    front_pad = _centered_box(
        GUIDE_PAD_SIZE,
        (sign_x * GUIDE_PAD_X, GUIDE_PAD_Y, z_center),
    )
    rear_pad = _centered_box(
        GUIDE_PAD_SIZE,
        (sign_x * GUIDE_PAD_X, -GUIDE_PAD_Y, z_center),
    )
    return _union_shapes(backing, arm, front_pad, rear_pad)


def _carriage_shape() -> cq.Workplane:
    top_bar = _centered_box(
        (CROSSBAR_LENGTH, CARRIAGE_DEPTH, CROSSBAR_THICKNESS),
        (0.0, 0.0, CROSSBAR_CENTER_Z),
    )
    bottom_bar = _centered_box(
        (CROSSBAR_LENGTH, CARRIAGE_DEPTH, CROSSBAR_THICKNESS),
        (0.0, 0.0, -CROSSBAR_CENTER_Z),
    )
    left_side = _centered_box(
        (SIDE_PLATE_THICKNESS, SIDE_PLATE_DEPTH, SIDE_PLATE_HEIGHT),
        (-SIDE_PLATE_CENTER_X, 0.0, 0.0),
    )
    right_side = _centered_box(
        (SIDE_PLATE_THICKNESS, SIDE_PLATE_DEPTH, SIDE_PLATE_HEIGHT),
        (SIDE_PLATE_CENTER_X, 0.0, 0.0),
    )
    left_rib = _centered_box(
        (RIB_WIDTH, RIB_DEPTH, RIB_HEIGHT),
        (-RIB_CENTER_X, 0.0, 0.0),
    )
    right_rib = _centered_box(
        (RIB_WIDTH, RIB_DEPTH, RIB_HEIGHT),
        (RIB_CENTER_X, 0.0, 0.0),
    )
    left_upper_cluster = _guide_cluster(-1.0, GUIDE_CLUSTER_CENTER_Z)
    left_lower_cluster = _guide_cluster(-1.0, -GUIDE_CLUSTER_CENTER_Z)
    right_upper_cluster = _guide_cluster(1.0, GUIDE_CLUSTER_CENTER_Z)
    right_lower_cluster = _guide_cluster(1.0, -GUIDE_CLUSTER_CENTER_Z)
    return _union_shapes(
        top_bar,
        bottom_bar,
        left_side,
        right_side,
        left_rib,
        right_rib,
        left_upper_cluster,
        left_lower_cluster,
        right_upper_cluster,
        right_lower_cluster,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacker_mast_module")

    model.material("painted_steel", rgba=(0.31, 0.34, 0.37, 1.0))
    model.material("machined_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("wear_strip", rgba=(0.80, 0.73, 0.42, 1.0))
    model.material("carriage_steel", rgba=(0.77, 0.29, 0.13, 1.0))

    mast = model.part("mast")
    left_channel = _channel_shape(open_to_inboard=True).translate(
        (-COLUMN_CENTER_X, 0.0, COLUMN_BASE_Z + (COLUMN_HEIGHT / 2.0))
    )
    right_channel = _channel_shape(open_to_inboard=False).translate(
        (COLUMN_CENTER_X, 0.0, COLUMN_BASE_Z + (COLUMN_HEIGHT / 2.0))
    )
    mast.visual(
        mesh_from_cadquery(left_channel, "mast_left_channel"),
        material="painted_steel",
        name="left_channel",
    )
    mast.visual(
        mesh_from_cadquery(right_channel, "mast_right_channel"),
        material="painted_steel",
        name="right_channel",
    )
    mast.visual(
        mesh_from_cadquery(
            _lower_crossmember_shape(),
            "mast_lower_crossmember",
        ),
        material="painted_steel",
        name="lower_crossmember",
    )
    mast.visual(
        mesh_from_cadquery(
            _crossmember_shape(UPPER_CROSSHEAD_SIZE, UPPER_CROSSHEAD_CENTER_Z),
            "mast_upper_crosshead",
        ),
        material="painted_steel",
        name="upper_crosshead",
    )
    mast.visual(
        mesh_from_cadquery(_mast_gussets_shape(), "mast_gussets"),
        material="painted_steel",
        name="gussets",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.465, CHANNEL_DEPTH, MAST_HEIGHT)),
        mass=132.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    left_wear_strip = model.part("left_wear_strip")
    left_wear_strip.visual(
        mesh_from_cadquery(_wear_strip_shape(outboard_positive_x=True), "left_wear_strip"),
        material="wear_strip",
        name="body",
    )
    left_wear_strip.inertial = Inertial.from_geometry(
        Box((WEAR_STRIP_THICKNESS + BOLT_HEAD_LENGTH, WEAR_STRIP_DEPTH, WEAR_STRIP_HEIGHT)),
        mass=4.0,
        origin=Origin(),
    )

    right_wear_strip = model.part("right_wear_strip")
    right_wear_strip.visual(
        mesh_from_cadquery(_wear_strip_shape(outboard_positive_x=False), "right_wear_strip"),
        material="wear_strip",
        name="body",
    )
    right_wear_strip.inertial = Inertial.from_geometry(
        Box((WEAR_STRIP_THICKNESS + BOLT_HEAD_LENGTH, WEAR_STRIP_DEPTH, WEAR_STRIP_HEIGHT)),
        mass=4.0,
        origin=Origin(),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_frame"),
        material="carriage_steel",
        name="body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.415, 0.062, CARRIAGE_HEIGHT)),
        mass=58.0,
        origin=Origin(),
    )

    left_strip_center_x = -(
        COLUMN_CENTER_X + (CHANNEL_WIDTH / 2.0) - WEB_THICKNESS - (WEAR_STRIP_THICKNESS / 2.0)
    )
    right_strip_center_x = -left_strip_center_x

    model.articulation(
        "mast_to_left_wear_strip",
        ArticulationType.FIXED,
        parent=mast,
        child=left_wear_strip,
        origin=Origin(xyz=(left_strip_center_x, 0.0, WEAR_STRIP_CENTER_Z)),
    )
    model.articulation(
        "mast_to_right_wear_strip",
        ArticulationType.FIXED,
        parent=mast,
        child=right_wear_strip,
        origin=Origin(xyz=(right_strip_center_x, 0.0, WEAR_STRIP_CENTER_Z)),
    )
    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=6000.0,
            velocity=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    left_wear_strip = object_model.get_part("left_wear_strip")
    right_wear_strip = object_model.get_part("right_wear_strip")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("mast_to_carriage")

    left_channel = mast.get_visual("left_channel")
    right_channel = mast.get_visual("right_channel")
    lower_crossmember = mast.get_visual("lower_crossmember")
    upper_crosshead = mast.get_visual("upper_crosshead")

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

    limits = slide.motion_limits
    ctx.check(
        "carriage_slide_is_vertical_prismatic",
        slide.joint_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"joint_type={slide.joint_type} axis={slide.axis}",
    )
    ctx.check(
        "carriage_slide_limits_match_mast_travel",
        limits is not None
        and limits.lower == 0.0
        and abs((limits.upper or 0.0) - CARRIAGE_TRAVEL) < 1e-9,
        details=f"limits={limits}",
    )

    ctx.expect_contact(
        left_wear_strip,
        mast,
        elem_b=left_channel,
        name="left_wear_strip_bolted_to_left_channel",
    )
    ctx.expect_contact(
        right_wear_strip,
        mast,
        elem_b=right_channel,
        name="right_wear_strip_bolted_to_right_channel",
    )
    ctx.expect_contact(
        carriage,
        left_wear_strip,
        name="carriage_bears_on_left_wear_strip",
    )
    ctx.expect_contact(
        carriage,
        right_wear_strip,
        name="carriage_bears_on_right_wear_strip",
    )
    ctx.expect_overlap(
        carriage,
        left_wear_strip,
        axes="z",
        min_overlap=0.40,
        name="left_guide_has_believable_engagement_height",
    )
    ctx.expect_overlap(
        carriage,
        right_wear_strip,
        axes="z",
        min_overlap=0.40,
        name="right_guide_has_believable_engagement_height",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="xy",
        margin=0.0,
        name="carriage_stays_between_mast_columns",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="z",
        negative_elem=lower_crossmember,
        min_gap=0.02,
        max_gap=0.04,
        name="carriage_clears_lower_crossmember_at_bottom_of_travel",
    )

    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_top_of_travel")
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem=upper_crosshead,
            min_gap=0.07,
            max_gap=0.11,
            name="carriage_clears_upper_crosshead_at_top_of_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
