from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_LENGTH = 0.150
BASE_WIDTH = 0.108
BASE_THICKNESS = 0.016
PEDESTAL_LENGTH = 0.070
PEDESTAL_WIDTH = 0.078
PEDESTAL_HEIGHT = 0.044
SHOULDER_Z = 0.084

SHOULDER_INNER_GAP = 0.044
SHOULDER_OUTER_WIDTH = 0.060
SHOULDER_EAR_THICKNESS = (SHOULDER_OUTER_WIDTH - SHOULDER_INNER_GAP) / 2.0
SHOULDER_EAR_LENGTH = 0.034
SHOULDER_EAR_HEIGHT = 0.048

UPPER_LENGTH = 0.112
UPPER_BODY_LENGTH = 0.082
UPPER_BODY_WIDTH = 0.028
UPPER_BODY_HEIGHT = 0.030

ELBOW_INNER_GAP = 0.046
ELBOW_OUTER_WIDTH = 0.062
ELBOW_EAR_THICKNESS = (ELBOW_OUTER_WIDTH - ELBOW_INNER_GAP) / 2.0
ELBOW_EAR_LENGTH = 0.028
ELBOW_EAR_HEIGHT = 0.036

FOREARM_LENGTH = 0.188
FOREARM_BODY_LENGTH = 0.118
FOREARM_BODY_WIDTH = 0.032
FOREARM_BODY_HEIGHT = 0.034
NOSE_OUTER_RADIUS = 0.018
NOSE_GUIDE_RADIUS = 0.0205
NOSE_BORE_RADIUS = 0.0115

SLIDE_STROKE = 0.055
SLIDE_ROD_RADIUS = 0.0105
SLIDE_ROD_LENGTH = 0.095
FLANGE_SIZE = 0.038
FLANGE_THICKNESS = 0.010

SHOULDER_LIMITS = (0.0, 0.95)
ELBOW_LIMITS = (-0.20, 1.00)


def _box(size: tuple[float, float, float], center: tuple[float, float, float], fillet: float | None = None):
    solid = cq.Workplane("XY").box(*size)
    if fillet is not None and fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid.translate(center)


def _cyl_x(length: float, radius: float, center: tuple[float, float, float]):
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _cyl_y(length: float, radius: float, center: tuple[float, float, float]):
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _base_shape():
    foot = _box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS), (0.0, 0.0, BASE_THICKNESS / 2.0), fillet=0.010)
    upper_pad = _box((0.090, 0.078, 0.010), (-0.010, 0.0, BASE_THICKNESS + 0.005), fillet=0.005)
    pedestal = _box((0.052, 0.072, 0.040), (-0.018, 0.0, 0.046), fillet=0.006)

    ear_y = (SHOULDER_INNER_GAP / 2.0) + (SHOULDER_EAR_THICKNESS / 2.0)
    left_ear = _box((0.014, SHOULDER_EAR_THICKNESS, 0.044), (0.004, ear_y, SHOULDER_Z), fillet=0.002)
    right_ear = _box((0.014, SHOULDER_EAR_THICKNESS, 0.044), (0.004, -ear_y, SHOULDER_Z), fillet=0.002)
    left_gusset = _box((0.018, 0.012, 0.026), (-0.010, ear_y, 0.068), fillet=0.002)
    right_gusset = _box((0.018, 0.012, 0.026), (-0.010, -ear_y, 0.068), fillet=0.002)
    rear_bridge = _box((0.014, SHOULDER_OUTER_WIDTH, 0.010), (-0.018, 0.0, 0.060), fillet=0.002)

    left_cap = _cyl_y(0.004, 0.018, (0.004, (SHOULDER_OUTER_WIDTH / 2.0) + 0.002, SHOULDER_Z))
    right_cap = _cyl_y(0.004, 0.018, (0.004, -(SHOULDER_OUTER_WIDTH / 2.0) - 0.002, SHOULDER_Z))

    return foot.union(upper_pad).union(pedestal).union(left_ear).union(right_ear).union(left_gusset).union(
        right_gusset
    ).union(rear_bridge).union(left_cap).union(right_cap)


def _upper_link_shape():
    shoulder_barrel = _cyl_y(SHOULDER_INNER_GAP - 0.004, 0.009, (0.004, 0.0, 0.0))
    root_cheek = _box((0.016, 0.018, 0.018), (0.020, 0.0, -0.008), fillet=0.002)

    beam_outer = _box((0.046, 0.020, 0.016), (0.052, 0.0, -0.014), fillet=0.003)
    beam_inner = _box((0.026, 0.010, 0.008), (0.052, 0.0, -0.014))
    top_ridge = _box((0.022, 0.010, 0.004), (0.054, 0.0, -0.007))
    beam = beam_outer.cut(beam_inner).union(top_ridge)
    distal_web = _box((0.012, 0.016, 0.016), (0.082, 0.0, -0.012), fillet=0.002)

    elbow_y = (ELBOW_INNER_GAP / 2.0) + (ELBOW_EAR_THICKNESS / 2.0)
    left_ear = _box((0.012, ELBOW_EAR_THICKNESS, 0.034), (UPPER_LENGTH, elbow_y, 0.0), fillet=0.002)
    right_ear = _box((0.012, ELBOW_EAR_THICKNESS, 0.034), (UPPER_LENGTH, -elbow_y, 0.0), fillet=0.002)
    left_cap = _cyl_y(0.004, 0.012, (UPPER_LENGTH, (ELBOW_OUTER_WIDTH / 2.0) + 0.002, 0.0))
    right_cap = _cyl_y(0.004, 0.012, (UPPER_LENGTH, -(ELBOW_OUTER_WIDTH / 2.0) - 0.002, 0.0))

    return shoulder_barrel.union(root_cheek).union(beam).union(distal_web).union(left_ear).union(right_ear).union(
        left_cap
    ).union(right_cap)


def _forearm_shape():
    elbow_barrel = _cyl_y(ELBOW_INNER_GAP - 0.004, 0.009, (0.004, 0.0, 0.0))
    root_band = _box((0.012, 0.018, 0.016), (0.018, 0.0, -0.010), fillet=0.002)

    beam_outer = _box((0.090, 0.022, 0.016), (0.074, 0.0, -0.014), fillet=0.003)
    beam_inner = _box((0.054, 0.010, 0.008), (0.076, 0.0, -0.014))
    top_ridge = _box((0.038, 0.010, 0.004), (0.082, 0.0, -0.007))
    beam = beam_outer.cut(beam_inner).union(top_ridge)

    transition_block = _box((0.022, 0.028, 0.024), (0.144, 0.0, -0.006), fillet=0.003)
    sleeve = _cyl_x(0.060, NOSE_OUTER_RADIUS, (FOREARM_LENGTH - 0.030, 0.0, 0.0))
    guide_a = _cyl_x(0.006, NOSE_GUIDE_RADIUS, (FOREARM_LENGTH - 0.014, 0.0, 0.0))
    guide_b = _cyl_x(0.006, NOSE_GUIDE_RADIUS, (FOREARM_LENGTH - 0.036, 0.0, 0.0))
    guide_c = _cyl_x(0.006, 0.0195, (FOREARM_LENGTH - 0.054, 0.0, 0.0))

    bore = _cyl_x(0.100, NOSE_BORE_RADIUS, (FOREARM_LENGTH - 0.050, 0.0, 0.0))

    return (
        elbow_barrel.union(root_band)
        .union(beam)
        .union(transition_block)
        .union(sleeve)
        .union(guide_a)
        .union(guide_b)
        .union(guide_c)
        .cut(bore)
    )


def _slide_rod_shape():
    rod = _cyl_x(SLIDE_ROD_LENGTH, SLIDE_ROD_RADIUS, (-SLIDE_ROD_LENGTH / 2.0, 0.0, 0.0))
    gland_ring = _cyl_x(0.006, 0.0108, (-0.003, 0.0, 0.0))
    return rod.union(gland_ring)


def _slide_flange_shape():
    flange = _box((FLANGE_THICKNESS, FLANGE_SIZE, FLANGE_SIZE), (FLANGE_THICKNESS / 2.0, 0.0, 0.0), fillet=0.004)
    bolt_offset = 0.0115
    bolt_x = FLANGE_THICKNESS + 0.002
    bolt_radius = 0.0032
    bolt_length = 0.004
    for y_pos in (-bolt_offset, bolt_offset):
        for z_pos in (-bolt_offset, bolt_offset):
            flange = flange.union(_cyl_x(bolt_length, bolt_radius, (bolt_x, y_pos, z_pos)))
    return flange


def _configure_materials(model: ArticulatedObject) -> None:
    model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_aluminum", rgba=(0.69, 0.72, 0.75, 1.0))
    model.material("anodized_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    model.material("dark_oxide", rgba=(0.24, 0.25, 0.27, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_inspection_arm")
    _configure_materials(model)

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base"), material="graphite", name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, SHOULDER_Z + 0.032)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (SHOULDER_Z + 0.032) / 2.0)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(mesh_from_cadquery(_upper_link_shape(), "upper_link"), material="machined_aluminum", name="upper_shell")
    upper_link.inertial = Inertial.from_geometry(
        Box((UPPER_LENGTH + 0.010, 0.050, 0.050)),
        mass=0.7,
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(mesh_from_cadquery(_forearm_shape(), "forearm"), material="anodized_steel", name="forearm_shell")
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH + 0.006, 0.056, 0.056)),
        mass=0.95,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    slide_tip = model.part("slide_tip")
    slide_tip.visual(mesh_from_cadquery(_slide_rod_shape(), "slide_rod"), material="machined_aluminum", name="rod")
    slide_tip.visual(mesh_from_cadquery(_slide_flange_shape(), "slide_flange"), material="dark_oxide", name="flange")
    slide_tip.inertial = Inertial.from_geometry(
        Box((SLIDE_ROD_LENGTH + FLANGE_THICKNESS, FLANGE_SIZE, FLANGE_SIZE)),
        mass=0.28,
        origin=Origin(
            xyz=((-(SLIDE_ROD_LENGTH / 2.0) + (FLANGE_THICKNESS / 2.0)) / 2.0, 0.0, 0.0)
        ),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
            effort=18.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
            effort=12.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "tip_slide",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=slide_tip,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_STROKE,
            effort=9.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    slide_tip = object_model.get_part("slide_tip")

    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    tip_slide = object_model.get_articulation("tip_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        base,
        upper_link,
        reason="captured shoulder journal is intentionally modeled as a nested hinge fit inside the pedestal yoke",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0025)
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
        "critical_parts_present",
        all(part is not None for part in (base, upper_link, forearm, slide_tip)),
        "base, upper link, forearm, and slide tip must all exist",
    )
    ctx.check(
        "mechanism_axes_match_prompt",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and tip_slide.axis == (1.0, 0.0, 0.0),
        "shoulder/elbow must pitch about Y and the inspection tip must slide along X",
    )
    ctx.check(
        "joint_limits_are_grounded",
        isclose(shoulder.motion_limits.lower, SHOULDER_LIMITS[0])
        and isclose(shoulder.motion_limits.upper, SHOULDER_LIMITS[1])
        and isclose(elbow.motion_limits.lower, ELBOW_LIMITS[0])
        and isclose(elbow.motion_limits.upper, ELBOW_LIMITS[1])
        and isclose(tip_slide.motion_limits.upper, SLIDE_STROKE),
        "unexpected articulation limits on the arm chain",
    )

    ctx.expect_contact(
        upper_link,
        base,
        contact_tol=0.0015,
        name="shoulder_hub_supported_in_base_yoke",
    )
    ctx.expect_contact(
        forearm,
        upper_link,
        contact_tol=0.0025,
        name="elbow_hub_supported_in_upper_clevis",
    )

    with ctx.pose({tip_slide: 0.0}):
        ctx.expect_contact(
            slide_tip,
            forearm,
            elem_a="flange",
            contact_tol=0.001,
            name="retracted_flange_seats_on_nose",
        )
        ctx.expect_within(
            slide_tip,
            forearm,
            axes="yz",
            inner_elem="rod",
            margin=0.003,
            name="retracted_rod_stays_guided_in_nose",
        )

    with ctx.pose({tip_slide: SLIDE_STROKE}):
        ctx.expect_gap(
            slide_tip,
            forearm,
            axis="x",
            positive_elem="flange",
            min_gap=SLIDE_STROKE - 0.001,
            max_gap=SLIDE_STROKE + 0.001,
            name="extended_flange_matches_prismatic_travel",
        )
        ctx.expect_within(
            slide_tip,
            forearm,
            axes="yz",
            inner_elem="rod",
            margin=0.003,
            name="extended_rod_remains_guided_in_nose",
        )

    with ctx.pose({shoulder: 0.72, elbow: 0.55, tip_slide: SLIDE_STROKE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_forward_inspection_pose")
    with ctx.pose({shoulder: 0.08, elbow: 0.92, tip_slide: 0.020}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_compact_folded_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
