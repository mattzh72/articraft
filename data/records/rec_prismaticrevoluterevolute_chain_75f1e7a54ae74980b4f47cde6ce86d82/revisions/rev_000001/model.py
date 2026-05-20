from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_X = 0.28
BASE_Y = 0.20
BASE_Z = 0.03

PEDESTAL_X = 0.16
PEDESTAL_Y = 0.14
PEDESTAL_Z = 0.05

UPRIGHT_X = 0.10
UPRIGHT_Y = 0.12
UPRIGHT_Z = 0.76
UPRIGHT_BOTTOM_Z = BASE_Z + PEDESTAL_Z
UPRIGHT_CENTER_Z = UPRIGHT_BOTTOM_Z + (UPRIGHT_Z / 2.0)

TOP_CAP_X = 0.14
TOP_CAP_Y = 0.14
TOP_CAP_Z = 0.04
TOP_CAP_CENTER_Z = UPRIGHT_BOTTOM_Z + UPRIGHT_Z + (TOP_CAP_Z / 2.0)

RAIL_X = 0.010
RAIL_Y = 0.018
RAIL_Z = 0.60
RAIL_CENTER_X = (UPRIGHT_X / 2.0) + (RAIL_X / 2.0)
RAIL_Y_OFFSET = 0.028
RAIL_CENTER_Z = 0.42

STOP_X = 0.012
STOP_Y = 0.066
STOP_Z = 0.018
STOP_CENTER_X = (UPRIGHT_X / 2.0) + (STOP_X / 2.0)
LOWER_STOP_CENTER_Z = 0.204
UPPER_STOP_CENTER_Z = 0.616

SHOULDER_AXIS_X = 0.126
SHOULDER_JOINT_X = 0.040
SLIDE_HOME_Z = 0.380
SLIDE_LOWER = -0.120
SLIDE_UPPER = 0.180

CARRIAGE_BODY_X = 0.064
CARRIAGE_BODY_Y = 0.108
CARRIAGE_BODY_Z = 0.170
CARRIAGE_BODY_CENTER_X = -0.010

SHOE_X = 0.012
SHOE_Y = 0.018
SHOE_Z = 0.036
SHOE_CENTER_X = -0.060
SHOE_Y_OFFSET = RAIL_Y_OFFSET
SHOE_Z_OFFSET = 0.050

TAB_X = 0.014
TAB_Y = 0.052
TAB_Z = 0.014
TAB_CENTER_X = -0.054
TAB_Z_OFFSET = 0.078

CLEVIS_CHEEK_X = 0.018
CLEVIS_CHEEK_Y = 0.006
CLEVIS_CHEEK_Z = 0.056
CLEVIS_CHEEK_CENTER_X = SHOULDER_JOINT_X
CLEVIS_CHEEK_CENTER_Y = 0.011
UPPER_LINK_T = 0.014

UPPER_LINK_LEN = 0.182
UPPER_ELBOW_Z = 0.016
UPPER_LINK_ROOT_R = 0.018

FOREARM_T = 0.014
FOREARM_LEN = 0.150
FOREARM_ROOT_R = 0.014

SHOULDER_LOWER = -0.45
SHOULDER_UPPER = 0.95
ELBOW_LOWER = -0.15
ELBOW_UPPER = 1.20


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _mast_body_shape() -> cq.Workplane:
    base_plate = _box((BASE_X, BASE_Y, BASE_Z), (0.0, 0.0, BASE_Z / 2.0))
    pedestal = _box(
        (PEDESTAL_X, PEDESTAL_Y, PEDESTAL_Z),
        (0.0, 0.0, BASE_Z + (PEDESTAL_Z / 2.0)),
    )
    upright = _box((UPRIGHT_X, UPRIGHT_Y, UPRIGHT_Z), (0.0, 0.0, UPRIGHT_CENTER_Z))
    top_cap = _box((TOP_CAP_X, TOP_CAP_Y, TOP_CAP_Z), (0.0, 0.0, TOP_CAP_CENTER_Z))
    lower_backbone = _box((0.070, 0.090, 0.16), (-0.012, 0.0, 0.16))
    return base_plate.union(pedestal).union(upright).union(top_cap).union(lower_backbone)


def _mast_rails_shape() -> cq.Workplane:
    rail_a = _box((RAIL_X, RAIL_Y, RAIL_Z), (RAIL_CENTER_X, RAIL_Y_OFFSET, RAIL_CENTER_Z))
    rail_b = _box((RAIL_X, RAIL_Y, RAIL_Z), (RAIL_CENTER_X, -RAIL_Y_OFFSET, RAIL_CENTER_Z))
    return rail_a.union(rail_b)


def _stop_shape(center_z: float) -> cq.Workplane:
    return _box((STOP_X, STOP_Y, STOP_Z), (STOP_CENTER_X, 0.0, center_z))


def _carriage_body_shape() -> cq.Workplane:
    rear_block = _box((0.058, 0.076, CARRIAGE_BODY_Z), (-0.033, 0.0, 0.0))
    top_rib = _box((0.086, 0.064, 0.022), (-0.006, 0.0, 0.048))
    bottom_rib = _box((0.086, 0.064, 0.022), (-0.006, 0.0, -0.048))
    cheek_a = _box(
        (CLEVIS_CHEEK_X, CLEVIS_CHEEK_Y, CLEVIS_CHEEK_Z),
        (CLEVIS_CHEEK_CENTER_X, CLEVIS_CHEEK_CENTER_Y, 0.0),
    )
    cheek_b = _box(
        (CLEVIS_CHEEK_X, CLEVIS_CHEEK_Y, CLEVIS_CHEEK_Z),
        (CLEVIS_CHEEK_CENTER_X, -CLEVIS_CHEEK_CENTER_Y, 0.0),
    )
    cheek_brace_a = _box((0.030, CLEVIS_CHEEK_Y, 0.016), (0.020, CLEVIS_CHEEK_CENTER_Y, 0.032))
    cheek_brace_b = _box((0.030, CLEVIS_CHEEK_Y, 0.016), (0.020, -CLEVIS_CHEEK_CENTER_Y, 0.032))
    lower_brace_a = _box((0.030, CLEVIS_CHEEK_Y, 0.016), (0.020, CLEVIS_CHEEK_CENTER_Y, -0.032))
    lower_brace_b = _box((0.030, CLEVIS_CHEEK_Y, 0.016), (0.020, -CLEVIS_CHEEK_CENTER_Y, -0.032))
    collar_a = _cyl_y(0.011, 0.004, (SHOULDER_JOINT_X, 0.017, 0.0))
    collar_b = _cyl_y(0.011, 0.004, (SHOULDER_JOINT_X, -0.017, 0.0))
    return (
        rear_block.union(top_rib)
        .union(bottom_rib)
        .union(cheek_a)
        .union(cheek_b)
        .union(cheek_brace_a)
        .union(cheek_brace_b)
        .union(lower_brace_a)
        .union(lower_brace_b)
        .union(collar_a)
        .union(collar_b)
    )


def _guide_pads_shape() -> cq.Workplane:
    pads: cq.Workplane | None = None
    for y_val in (-SHOE_Y_OFFSET, SHOE_Y_OFFSET):
        for z_val in (-SHOE_Z_OFFSET, SHOE_Z_OFFSET):
            pad = _box((SHOE_X, SHOE_Y, SHOE_Z), (SHOE_CENTER_X, y_val, z_val))
            stem = _box((0.032, 0.014, 0.022), (-0.044, y_val, z_val))
            feature = pad.union(stem)
            pads = feature if pads is None else pads.union(feature)
    assert pads is not None
    return pads


def _tab_shape(center_z: float) -> cq.Workplane:
    striker = _box((TAB_X, TAB_Y, TAB_Z), (TAB_CENTER_X, 0.0, center_z))
    rib = _box((0.020, 0.024, 0.020), (-0.046, 0.0, center_z))
    return striker.union(rib)


def _upper_link_shape() -> cq.Workplane:
    root_barrel = _cyl_y(0.012, 0.016, (0.0, 0.0, 0.0))
    root_lug = _box((0.026, 0.016, 0.020), (0.016, 0.0, 0.002))
    beam = _box((0.100, 0.016, 0.018), (0.067, 0.0, 0.009))
    elbow_brace_a = _box((0.060, 0.006, 0.014), (0.147, 0.010, 0.018))
    elbow_brace_b = _box((0.060, 0.006, 0.014), (0.147, -0.010, 0.018))
    elbow_cheek_a = _box((0.018, 0.006, 0.036), (UPPER_LINK_LEN, 0.010, UPPER_ELBOW_Z))
    elbow_cheek_b = _box((0.018, 0.006, 0.036), (UPPER_LINK_LEN, -0.010, UPPER_ELBOW_Z))
    return (
        root_barrel.union(root_lug)
        .union(beam)
        .union(elbow_brace_a)
        .union(elbow_brace_b)
        .union(elbow_cheek_a)
        .union(elbow_cheek_b)
    )


def _forearm_shape() -> cq.Workplane:
    root_barrel = _cyl_y(0.010, 0.010, (0.0, 0.0, 0.0))
    root_lug = _box((0.018, FOREARM_T, 0.016), (0.012, 0.0, 0.0))
    beam = _box((0.088, FOREARM_T, 0.016), (0.074, 0.0, 0.008))
    tip_pad = _box((0.036, 0.020, 0.028), (0.126, 0.0, 0.005))
    return root_barrel.union(root_lug).union(beam).union(tip_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_slide_side_arm")

    model.material("frame_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_black", rgba=(0.16, 0.17, 0.20, 1.0))
    model.material("arm_alloy", rgba=(0.74, 0.76, 0.79, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_mast_body_shape(), "mast_body"),
        material="frame_gray",
        name="body",
    )
    mast.visual(
        mesh_from_cadquery(_mast_rails_shape(), "mast_rails"),
        material="rail_steel",
        name="rails",
    )
    mast.visual(
        mesh_from_cadquery(_stop_shape(LOWER_STOP_CENTER_Z), "mast_lower_stop"),
        material="frame_gray",
        name="lower_stop",
    )
    mast.visual(
        mesh_from_cadquery(_stop_shape(UPPER_STOP_CENTER_Z), "mast_upper_stop"),
        material="frame_gray",
        name="upper_stop",
    )
    mast.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, TOP_CAP_CENTER_Z + (TOP_CAP_Z / 2.0))),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body"),
        material="carriage_black",
        name="body",
    )
    carriage.visual(
        mesh_from_cadquery(_guide_pads_shape(), "carriage_guide_pads"),
        material="carriage_black",
        name="guide_pads",
    )
    carriage.visual(
        mesh_from_cadquery(_tab_shape(-TAB_Z_OFFSET), "carriage_lower_tab"),
        material="carriage_black",
        name="lower_tab",
    )
    carriage.visual(
        mesh_from_cadquery(_tab_shape(TAB_Z_OFFSET), "carriage_upper_tab"),
        material="carriage_black",
        name="upper_tab",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.18)),
        mass=6.0,
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link_body"),
        material="arm_alloy",
        name="body",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.20, UPPER_LINK_T, 0.06)),
        mass=2.5,
        origin=Origin(xyz=(0.09, 0.0, 0.010)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm_body"),
        material="arm_alloy",
        name="body",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.16, FOREARM_T, 0.05)),
        mass=1.6,
        origin=Origin(xyz=(0.074, 0.0, 0.006)),
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=900.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LOWER,
            upper=SHOULDER_UPPER,
            effort=120.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LINK_LEN, 0.0, UPPER_ELBOW_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LOWER,
            upper=ELBOW_UPPER,
            effort=80.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    slide = object_model.get_articulation("mast_slide")
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")

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
    ctx.allow_overlap(
        carriage,
        upper_link,
        reason="shoulder joint uses coaxial bearing collars and a simplified mesh representation without explicit through-holes",
    )
    ctx.allow_overlap(
        upper_link,
        forearm,
        reason="elbow joint uses coaxial clevis sleeves and a simplified mesh representation without explicit through-holes",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "articulation axes match mechanism",
        slide.axis == (0.0, 0.0, 1.0)
        and shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0),
        details=f"slide={slide.axis}, shoulder={shoulder.axis}, elbow={elbow.axis}",
    )

    ctx.expect_contact(
        carriage,
        mast,
        elem_a="guide_pads",
        elem_b="rails",
        contact_tol=0.0015,
        name="carriage guide pads stay seated on mast rails",
    )
    ctx.expect_contact(
        upper_link,
        carriage,
        contact_tol=0.0015,
        name="upper link remains seated in carriage clevis",
    )
    ctx.expect_contact(
        forearm,
        upper_link,
        contact_tol=0.0015,
        name="forearm remains seated in elbow clevis",
    )

    with ctx.pose({slide: SLIDE_LOWER}):
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="lower_stop",
            negative_elem="lower_tab",
            min_gap=0.0,
            max_gap=0.0035,
            name="lower carriage tab meets lower mast stop cleanly",
        )

    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            positive_elem="upper_tab",
            negative_elem="upper_stop",
            min_gap=0.0,
            max_gap=0.0035,
            name="upper carriage tab meets upper mast stop cleanly",
        )

    with ctx.pose({slide: SLIDE_LOWER, shoulder: SHOULDER_LOWER, elbow: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="lowered articulation pose stays clear of mast and carriage",
        )

    with ctx.pose({slide: 0.0, shoulder: SHOULDER_UPPER, elbow: ELBOW_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="folded articulation pose stays clear of mast and carriage",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
