from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.50
BASE_TIE_WIDTH = 0.84
BASE_TIE_THICKNESS = 0.028
BASE_TIE_LENGTH = 0.110
BASE_TIE_X = 0.620

RAIL_CENTER_Y = 0.330
RAIL_PAD_LENGTH = 1.320
RAIL_PAD_WIDTH = 0.060
RAIL_PAD_HEIGHT = 0.024

TRACK_LENGTH = 1.240
TRACK_WIDTH = 0.036
TRACK_HEIGHT = 0.010
TRACK_TOP_Z = BASE_TIE_THICKNESS + RAIL_PAD_HEIGHT + TRACK_HEIGHT

STOP_LENGTH = 0.020
STOP_WIDTH = 0.060
STOP_HEIGHT = 0.024
STOP_X = 0.645

PORTAL_TRAVEL = 0.420
CARRIAGE_TRAVEL = 0.170

PORTAL_TRUCK_LENGTH = 0.220
PORTAL_TRUCK_WIDTH = 0.110
PORTAL_TRUCK_HEIGHT = 0.101
PORTAL_TRUCK_CENTER_Z = 0.0545
PORTAL_PAD_LENGTH = 0.180
PORTAL_PAD_WIDTH = 0.040
PORTAL_PAD_HEIGHT = 0.005
PORTAL_PAD_TOP_Z = PORTAL_PAD_HEIGHT

TRUCK_COVER_LENGTH = 0.240
TRUCK_COVER_WIDTH = 0.140
TRUCK_COVER_HEIGHT = 0.010
TRUCK_COVER_CENTER_Z = 0.110

SKIRT_THICKNESS = 0.014
SKIRT_HEIGHT = 0.030
SKIRT_OFFSET_Y = 0.038

COLUMN_DEPTH = 0.120
COLUMN_WIDTH = 0.090
COLUMN_HEIGHT = 0.220
COLUMN_CENTER_Z = 0.205

BEAM_DEPTH = 0.170
BEAM_WIDTH = 0.680
BEAM_HEIGHT = 0.160
BEAM_CENTER_Z = 0.390
BEAM_FRONT_X = BEAM_DEPTH / 2.0

POCKET_DEPTH = 0.070
POCKET_WIDTH = 0.540
POCKET_HEIGHT = 0.118
POCKET_CENTER_X = BEAM_FRONT_X - (POCKET_DEPTH / 2.0)

LOWER_BRACE_DEPTH = 0.100
LOWER_BRACE_WIDTH = 0.580
LOWER_BRACE_HEIGHT = 0.050
LOWER_BRACE_CENTER_X = -0.025
LOWER_BRACE_CENTER_Z = 0.155

CARRIAGE_JOINT_X = 0.036
CARRIAGE_JOINT_Z = BEAM_CENTER_Z

SLIDE_SHOE_DEPTH = 0.045
SLIDE_SHOE_WIDTH = 0.160
SLIDE_SHOE_HEIGHT = 0.092

FRONT_PLATE_DEPTH = 0.050
FRONT_PLATE_WIDTH = 0.135
FRONT_PLATE_HEIGHT = 0.150
FRONT_PLATE_CENTER_X = 0.076

CONNECTOR_DEPTH = 0.029
CONNECTOR_WIDTH = 0.110
CONNECTOR_HEIGHT = 0.120
CONNECTOR_CENTER_X = 0.0365

CHEEK_DEPTH = 0.034
CHEEK_WIDTH = 0.014
CHEEK_HEIGHT = 0.170
CHEEK_CENTER_X = 0.034
CHEEK_CENTER_Y = 0.070

LOWER_NOSE_DEPTH = 0.050
LOWER_NOSE_WIDTH = 0.056
LOWER_NOSE_HEIGHT = 0.050
LOWER_NOSE_CENTER_X = 0.088
LOWER_NOSE_CENTER_Z = -0.092

UPPER_CAP_DEPTH = 0.036
UPPER_CAP_WIDTH = 0.074
UPPER_CAP_HEIGHT = 0.030
UPPER_CAP_CENTER_X = 0.060
UPPER_CAP_CENTER_Z = 0.088

GUIDE_LENGTH = 0.580
GUIDE_DEPTH = 0.036
GUIDE_HEIGHT = 0.012
GUIDE_CENTER_X = 0.018
GUIDE_LOWER_CENTER_Z = 0.338
GUIDE_UPPER_CENTER_Z = 0.442


def _box(size: tuple[float, float, float], center: tuple[float, float, float], *, fillet: float = 0.0) -> cq.Workplane:
    wp = cq.Workplane("XY").box(*size)
    if fillet > 0.0:
        wp = wp.edges("|Z").fillet(fillet)
    return wp.translate(center)


def _fuse(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _make_base_frame() -> cq.Workplane:
    shapes = [
        _box(
            (BASE_TIE_LENGTH, BASE_TIE_WIDTH, BASE_TIE_THICKNESS),
            (-BASE_TIE_X, 0.0, BASE_TIE_THICKNESS / 2.0),
            fillet=0.008,
        ),
        _box(
            (BASE_TIE_LENGTH, BASE_TIE_WIDTH, BASE_TIE_THICKNESS),
            (BASE_TIE_X, 0.0, BASE_TIE_THICKNESS / 2.0),
            fillet=0.008,
        ),
    ]
    for rail_center_y in (-RAIL_CENTER_Y, RAIL_CENTER_Y):
        shapes.append(
            _box(
                (RAIL_PAD_LENGTH, RAIL_PAD_WIDTH, RAIL_PAD_HEIGHT),
                (0.0, rail_center_y, BASE_TIE_THICKNESS + (RAIL_PAD_HEIGHT / 2.0)),
                fillet=0.004,
            )
        )

    base = _fuse(shapes)
    return base


def _make_portal_end_assembly(rail_center_y: float) -> cq.Workplane:
    truck = _box(
        (PORTAL_TRUCK_LENGTH, PORTAL_TRUCK_WIDTH, PORTAL_TRUCK_HEIGHT),
        (0.0, rail_center_y, PORTAL_TRUCK_CENTER_Z),
        fillet=0.006,
    )
    cover = _box(
        (TRUCK_COVER_LENGTH, TRUCK_COVER_WIDTH, TRUCK_COVER_HEIGHT),
        (0.0, rail_center_y, TRUCK_COVER_CENTER_Z),
        fillet=0.004,
    )
    column = _box(
        (COLUMN_DEPTH, COLUMN_WIDTH, COLUMN_HEIGHT),
        (0.0, rail_center_y, COLUMN_CENTER_Z),
        fillet=0.006,
    )
    skirt_inner = _box(
        (PORTAL_TRUCK_LENGTH - 0.020, SKIRT_THICKNESS, SKIRT_HEIGHT),
        (0.0, rail_center_y - (SKIRT_OFFSET_Y if rail_center_y > 0.0 else -SKIRT_OFFSET_Y), 0.0),
    )
    skirt_outer = _box(
        (PORTAL_TRUCK_LENGTH - 0.020, SKIRT_THICKNESS, SKIRT_HEIGHT),
        (0.0, rail_center_y + (SKIRT_OFFSET_Y if rail_center_y > 0.0 else -SKIRT_OFFSET_Y), 0.0),
    )
    gusset = (
        cq.Workplane("YZ")
        .moveTo(rail_center_y - 0.040, 0.095)
        .lineTo(rail_center_y - 0.040, 0.255)
        .lineTo(rail_center_y + 0.010, 0.315)
        .lineTo(rail_center_y + 0.040, 0.315)
        .lineTo(rail_center_y + 0.040, 0.095)
        .close()
        .extrude(COLUMN_DEPTH / 2.0, both=True)
    )
    return _fuse([truck, cover, column, skirt_inner, skirt_outer, gusset])


def _make_bridge_beam() -> cq.Workplane:
    rear_web = _box(
        (0.048, BEAM_WIDTH, BEAM_HEIGHT - 0.012),
        (-0.061, 0.0, BEAM_CENTER_Z),
        fillet=0.006,
    )
    top_chord = _box(
        (0.110, BEAM_WIDTH, 0.028),
        (-0.030, 0.0, BEAM_CENTER_Z + (BEAM_HEIGHT / 2.0) - 0.014),
        fillet=0.004,
    )
    bottom_chord = _box(
        (0.110, BEAM_WIDTH, 0.028),
        (-0.030, 0.0, BEAM_CENTER_Z - (BEAM_HEIGHT / 2.0) + 0.014),
        fillet=0.004,
    )
    lower_brace = _box(
        (0.082, 0.560, 0.040),
        (-0.028, 0.0, 0.190),
        fillet=0.004,
    )
    left_front_flange = _box(
        (0.018, 0.040, BEAM_HEIGHT - 0.010),
        (0.071, -0.320, BEAM_CENTER_Z),
        fillet=0.004,
    )
    right_front_flange = _box(
        (0.018, 0.040, BEAM_HEIGHT - 0.010),
        (0.071, 0.320, BEAM_CENTER_Z),
        fillet=0.004,
    )
    left_cover_flange = _box(
        (0.024, 0.050, 0.016),
        (0.066, -0.315, BEAM_CENTER_Z + 0.052),
        fillet=0.003,
    )
    right_cover_flange = _box(
        (0.024, 0.050, 0.016),
        (0.066, 0.315, BEAM_CENTER_Z + 0.052),
        fillet=0.003,
    )
    return _fuse(
        [
            rear_web,
            top_chord,
            bottom_chord,
            lower_brace,
            left_front_flange,
            right_front_flange,
            left_cover_flange,
            right_cover_flange,
        ]
    )


def _make_carriage_body() -> cq.Workplane:
    connector = _box(
        (CONNECTOR_DEPTH, CONNECTOR_WIDTH, CONNECTOR_HEIGHT),
        (CONNECTOR_CENTER_X, 0.0, 0.0),
        fillet=0.004,
    )
    left_cheek = _box(
        (CHEEK_DEPTH, CHEEK_WIDTH, CHEEK_HEIGHT),
        (CHEEK_CENTER_X, -CHEEK_CENTER_Y, 0.0),
        fillet=0.003,
    )
    right_cheek = _box(
        (CHEEK_DEPTH, CHEEK_WIDTH, CHEEK_HEIGHT),
        (CHEEK_CENTER_X, CHEEK_CENTER_Y, 0.0),
        fillet=0.003,
    )
    lower_nose = _box(
        (LOWER_NOSE_DEPTH, LOWER_NOSE_WIDTH, LOWER_NOSE_HEIGHT),
        (LOWER_NOSE_CENTER_X, 0.0, LOWER_NOSE_CENTER_Z),
        fillet=0.003,
    )
    upper_cap = _box(
        (UPPER_CAP_DEPTH, UPPER_CAP_WIDTH, UPPER_CAP_HEIGHT),
        (UPPER_CAP_CENTER_X, 0.0, UPPER_CAP_CENTER_Z),
        fillet=0.003,
    )
    return _fuse([connector, left_cheek, right_cheek, lower_nose, upper_cap])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_portal_gantry")

    model.material("base_cast", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("rail_steel", rgba=(0.60, 0.63, 0.67, 1.0))
    model.material("portal_paint", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("cover_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("carriage_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_frame(), "base_frame"),
        material="base_cast",
        name="base_frame",
    )
    for name, rail_center_y in (
        ("left_track_strip", -RAIL_CENTER_Y),
        ("right_track_strip", RAIL_CENTER_Y),
    ):
        base.visual(
            Box((TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    rail_center_y,
                    BASE_TIE_THICKNESS + RAIL_PAD_HEIGHT + (TRACK_HEIGHT / 2.0),
                )
            ),
            material="rail_steel",
            name=name,
        )
    for name, stop_x, rail_center_y in (
        ("left_negative_stop", -STOP_X, -RAIL_CENTER_Y),
        ("left_positive_stop", STOP_X, -RAIL_CENTER_Y),
        ("right_negative_stop", -STOP_X, RAIL_CENTER_Y),
        ("right_positive_stop", STOP_X, RAIL_CENTER_Y),
    ):
        base.visual(
            Box((STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT)),
            origin=Origin(
                xyz=(
                    stop_x,
                    rail_center_y,
                    BASE_TIE_THICKNESS + RAIL_PAD_HEIGHT + (STOP_HEIGHT / 2.0),
                )
            ),
            material="cover_dark",
            name=name,
        )

    portal = model.part("portal")
    portal.visual(
        mesh_from_cadquery(_make_portal_end_assembly(-RAIL_CENTER_Y), "portal_left_end_truck"),
        material="portal_paint",
        name="left_end_truck",
    )
    portal.visual(
        mesh_from_cadquery(_make_portal_end_assembly(RAIL_CENTER_Y), "portal_right_end_truck"),
        material="portal_paint",
        name="right_end_truck",
    )
    portal.visual(
        mesh_from_cadquery(_make_bridge_beam(), "portal_bridge_beam"),
        material="portal_paint",
        name="bridge_beam",
    )
    portal.visual(
        Box((PORTAL_PAD_LENGTH, PORTAL_PAD_WIDTH, PORTAL_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_CENTER_Y, PORTAL_PAD_HEIGHT / 2.0)),
        material="rail_steel",
        name="left_truck_pad",
    )
    portal.visual(
        Box((PORTAL_PAD_LENGTH, PORTAL_PAD_WIDTH, PORTAL_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, PORTAL_PAD_HEIGHT / 2.0)),
        material="rail_steel",
        name="right_truck_pad",
    )
    portal.visual(
        Box((GUIDE_DEPTH, GUIDE_LENGTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, GUIDE_LOWER_CENTER_Z)),
        material="rail_steel",
        name="lower_carriage_guide",
    )
    portal.visual(
        Box((GUIDE_DEPTH, GUIDE_LENGTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, GUIDE_UPPER_CENTER_Z)),
        material="rail_steel",
        name="upper_carriage_guide",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_body(), "carriage_body"),
        material="carriage_dark",
        name="carriage_body",
    )
    carriage.visual(
        Box((FRONT_PLATE_DEPTH, FRONT_PLATE_WIDTH, FRONT_PLATE_HEIGHT)),
        origin=Origin(xyz=(FRONT_PLATE_CENTER_X, 0.0, 0.0)),
        material="cover_dark",
        name="front_plate",
    )
    carriage.visual(
        Box((SLIDE_SHOE_DEPTH, SLIDE_SHOE_WIDTH, SLIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="rail_steel",
        name="slide_shoe",
    )

    model.articulation(
        "base_to_portal",
        ArticulationType.PRISMATIC,
        parent=base,
        child=portal,
        origin=Origin(xyz=(0.0, 0.0, TRACK_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-PORTAL_TRAVEL,
            upper=PORTAL_TRAVEL,
            effort=1800.0,
            velocity=0.55,
        ),
    )
    model.articulation(
        "portal_to_carriage",
        ArticulationType.PRISMATIC,
        parent=portal,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_JOINT_X, 0.0, CARRIAGE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
            effort=650.0,
            velocity=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    portal = object_model.get_part("portal")
    carriage = object_model.get_part("carriage")
    portal_slide = object_model.get_articulation("base_to_portal")
    carriage_slide = object_model.get_articulation("portal_to_carriage")

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
        "portal axis follows base rails",
        tuple(portal_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis prismatic motion, got {portal_slide.axis}",
    )
    ctx.check(
        "carriage axis runs across beam",
        tuple(carriage_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected y-axis prismatic motion, got {carriage_slide.axis}",
    )

    for side in ("left", "right"):
        ctx.expect_contact(
            portal,
            base,
            elem_a=f"{side}_truck_pad",
            elem_b=f"{side}_track_strip",
            contact_tol=1e-6,
            name=f"{side} truck pad seats on its rail strip",
        )
        ctx.expect_overlap(
            portal,
            base,
            axes="xy",
            elem_a=f"{side}_truck_pad",
            elem_b=f"{side}_track_strip",
            min_overlap=0.030,
            name=f"{side} truck pad overlaps its rail strip footprint",
        )

    for pose_name, portal_pos in (
        ("portal_negative_limit", -PORTAL_TRAVEL),
        ("portal_positive_limit", PORTAL_TRAVEL),
    ):
        with ctx.pose({portal_slide: portal_pos}):
            for side in ("left", "right"):
                ctx.expect_contact(
                    portal,
                    base,
                    elem_a=f"{side}_truck_pad",
                    elem_b=f"{side}_track_strip",
                    contact_tol=1e-6,
                    name=f"{pose_name} keeps {side} truck on rail",
                )
                ctx.expect_overlap(
                    portal,
                    base,
                    axes="x",
                    elem_a=f"{side}_truck_pad",
                    elem_b=f"{side}_track_strip",
                    min_overlap=0.120,
                    name=f"{pose_name} keeps {side} truck pad fully supported",
                )

    for pose_name, carriage_pos in (
        ("carriage_negative_limit", -CARRIAGE_TRAVEL),
        ("carriage_center", 0.0),
        ("carriage_positive_limit", CARRIAGE_TRAVEL),
    ):
        with ctx.pose({carriage_slide: carriage_pos}):
            ctx.expect_contact(
                carriage,
                portal,
                elem_a="slide_shoe",
                elem_b="lower_carriage_guide",
                contact_tol=1e-6,
                name=f"{pose_name} keeps slide shoe seated on lower guide",
            )
            ctx.expect_contact(
                carriage,
                portal,
                elem_a="slide_shoe",
                elem_b="upper_carriage_guide",
                contact_tol=1e-6,
                name=f"{pose_name} keeps slide shoe captured under upper guide",
            )
            ctx.expect_within(
                carriage,
                portal,
                axes="y",
                inner_elem="slide_shoe",
                outer_elem="bridge_beam",
                margin=0.020,
                name=f"{pose_name} keeps slide shoe inside bridge beam window",
            )
            ctx.expect_gap(
                carriage,
                portal,
                axis="x",
                positive_elem="front_plate",
                negative_elem="bridge_beam",
                min_gap=0.001,
                max_gap=0.010,
                name=f"{pose_name} keeps front plate proud of beam face",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
