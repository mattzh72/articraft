from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


SHOULDER_Z = 0.140
SHOULDER_AXIS_X = 0.045
UPPER_LINK_LENGTH = 0.255
FOREARM_LENGTH = 0.145

SHOULDER_GAP = 0.056
ELBOW_GAP = 0.052
YOKE_CHEEK_THICKNESS = 0.018


def _rounded_box(
    size: tuple[float, float, float],
    *,
    center: tuple[float, float, float],
    fillet: float,
):
    shape = cq.Workplane("XY").box(*size).translate(center)
    if fillet > 0.0:
        shape = shape.edges("|Z").fillet(fillet)
    return shape


def _cylinder_y(
    radius: float,
    length: float,
    *,
    center: tuple[float, float, float],
):
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def _cylinder_z(
    radius: float,
    length: float,
    *,
    center: tuple[float, float, float],
):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length / 2.0))
    )


def _side_gusset(
    points: list[tuple[float, float]],
    *,
    y_center: float,
    thickness: float,
):
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, y_center - thickness / 2.0, 0.0))
    )


def _make_shoulder_housing_shape():
    base_plate = _rounded_box(
        (0.160, 0.120, 0.018),
        center=(-0.020, 0.0, 0.009),
        fillet=0.006,
    )
    pedestal = _rounded_box(
        (0.096, 0.084, 0.098),
        center=(-0.048, 0.0, 0.058),
        fillet=0.010,
    )
    top_block = _rounded_box(
        (0.030, 0.040, 0.026),
        center=(-0.018, 0.0, 0.104),
        fillet=0.005,
    )

    body = base_plate.union(pedestal).union(top_block)

    gusset_profile = [
        (-0.076, 0.094),
        (-0.028, 0.094),
        (SHOULDER_AXIS_X - 0.010, SHOULDER_Z - 0.022),
        (-0.050, SHOULDER_Z - 0.024),
    ]

    cheek_y = SHOULDER_GAP / 2.0 + YOKE_CHEEK_THICKNESS / 2.0
    for sign in (-1.0, 1.0):
        y_center = sign * cheek_y
        cheek = _rounded_box(
            (0.016, YOKE_CHEEK_THICKNESS, 0.050),
            center=(SHOULDER_AXIS_X - 0.004, y_center, SHOULDER_Z),
            fillet=0.004,
        )
        cheek = cheek.union(
            _cylinder_y(
                0.024,
                YOKE_CHEEK_THICKNESS,
                center=(SHOULDER_AXIS_X, y_center, SHOULDER_Z),
            )
        )
        cheek = cheek.union(
            _cylinder_y(
                0.028,
                0.006,
                center=(SHOULDER_AXIS_X, sign * 0.046, SHOULDER_Z),
            )
        )
        body = body.union(cheek)
        body = body.union(
            _side_gusset(
                gusset_profile,
                y_center=y_center,
                thickness=YOKE_CHEEK_THICKNESS,
            )
        )

    body = body.cut(
        _cylinder_y(
            0.009,
            SHOULDER_GAP + 2.0 * YOKE_CHEEK_THICKNESS + 0.028,
            center=(SHOULDER_AXIS_X, 0.0, SHOULDER_Z),
        )
    )

    for x_pos in (-0.070, 0.030):
        for y_pos in (-0.042, 0.042):
            body = body.cut(_cylinder_z(0.006, 0.024, center=(x_pos, y_pos, 0.009)))

    return body


def _make_upper_link_shape():
    shoulder_lug = _cylinder_y(0.007, 0.018, center=(0.0, 0.0, 0.0))
    root_neck = _rounded_box(
        (0.016, 0.014, 0.020),
        center=(0.012, 0.0, 0.0),
        fillet=0.003,
    )
    transition = _rounded_box(
        (0.026, 0.018, 0.030),
        center=(0.028, 0.0, 0.0),
        fillet=0.004,
    )
    beam = _rounded_box(
        (0.170, 0.026, 0.048),
        center=(0.124, 0.0, 0.0),
        fillet=0.004,
    )
    beam = beam.cut(
        _rounded_box(
            (0.110, 0.012, 0.022),
            center=(0.126, 0.0, 0.0),
            fillet=0.002,
        )
    )
    top_rib = _rounded_box(
        (0.126, 0.010, 0.008),
        center=(0.136, 0.0, 0.020),
        fillet=0.002,
    )
    bottom_rib = _rounded_box(
        (0.126, 0.010, 0.008),
        center=(0.136, 0.0, -0.020),
        fillet=0.002,
    )

    body = shoulder_lug.union(root_neck).union(transition).union(beam).union(top_rib).union(bottom_rib)

    elbow_cheek_y = ELBOW_GAP / 2.0 + YOKE_CHEEK_THICKNESS / 2.0
    elbow_gusset = [
        (0.212, -0.018),
        (0.228, -0.018),
        (UPPER_LINK_LENGTH - 0.010, -0.028),
        (UPPER_LINK_LENGTH - 0.022, -0.004),
    ]
    for sign in (-1.0, 1.0):
        y_center = sign * elbow_cheek_y
        cheek = _rounded_box(
            (0.016, YOKE_CHEEK_THICKNESS, 0.054),
            center=(UPPER_LINK_LENGTH - 0.004, y_center, 0.0),
            fillet=0.004,
        )
        cheek = cheek.union(
            _cylinder_y(
                0.030,
                YOKE_CHEEK_THICKNESS,
                center=(UPPER_LINK_LENGTH, y_center, 0.0),
            )
        )
        cheek = cheek.union(
            _cylinder_y(
                0.034,
                0.006,
                center=(UPPER_LINK_LENGTH, sign * 0.046, 0.0),
            )
        )
        body = body.union(cheek)
        profile = [(x, z if sign > 0 else -z) for (x, z) in elbow_gusset]
        body = body.union(
            _side_gusset(profile, y_center=y_center, thickness=YOKE_CHEEK_THICKNESS)
        )

    body = body.cut(
        _cylinder_y(
            0.0075,
            ELBOW_GAP + 2.0 * YOKE_CHEEK_THICKNESS + 0.028,
            center=(UPPER_LINK_LENGTH, 0.0, 0.0),
        )
    )

    return body


def _make_forearm_shape():
    knuckle = _cylinder_y(0.0055, 0.016, center=(0.0, 0.0, 0.0))
    root_block = _rounded_box(
        (0.014, 0.012, 0.016),
        center=(0.009, 0.0, 0.0),
        fillet=0.003,
    )
    beam = _rounded_box(
        (0.090, 0.016, 0.022),
        center=(0.064, 0.0, 0.0),
        fillet=0.004,
    )
    beam = beam.cut(
        _rounded_box(
            (0.044, 0.008, 0.010),
            center=(0.066, 0.0, 0.0),
            fillet=0.002,
        )
    )
    top_rib = _rounded_box(
        (0.062, 0.008, 0.006),
        center=(0.074, 0.0, 0.011),
        fillet=0.002,
    )
    bottom_rib = _rounded_box(
        (0.062, 0.008, 0.006),
        center=(0.074, 0.0, -0.011),
        fillet=0.002,
    )
    carrier = _rounded_box(
        (0.024, 0.022, 0.038),
        center=(FOREARM_LENGTH - 0.018, 0.0, 0.0),
        fillet=0.004,
    )
    pad_plate = _rounded_box(
        (0.012, 0.052, 0.060),
        center=(FOREARM_LENGTH + 0.004, 0.0, 0.0),
        fillet=0.004,
    )

    return knuckle.union(root_block).union(beam).union(top_rib).union(bottom_rib).union(carrier).union(pad_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_elbow_arm")

    model.material("cast_iron", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("machined_steel", rgba=(0.68, 0.70, 0.74, 1.0))
    model.material("anodized_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("rubber_pad", rgba=(0.07, 0.07, 0.08, 1.0))

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Box((0.160, 0.120, 0.018)),
        origin=Origin(xyz=(-0.020, 0.0, 0.009)),
        material="cast_iron",
        name="base_plate",
    )
    shoulder_housing.visual(
        Box((0.096, 0.084, 0.098)),
        origin=Origin(xyz=(-0.048, 0.0, 0.058)),
        material="cast_iron",
        name="pedestal",
    )
    shoulder_housing.visual(
        Box((0.030, 0.040, 0.026)),
        origin=Origin(xyz=(-0.018, 0.0, 0.104)),
        material="cast_iron",
        name="shoulder_cap",
    )
    for sign, name_suffix in ((-1.0, "neg"), (1.0, "pos")):
        cheek_y = sign * (SHOULDER_GAP / 2.0 + YOKE_CHEEK_THICKNESS / 2.0)
        shoulder_housing.visual(
            Box((0.016, YOKE_CHEEK_THICKNESS, 0.050)),
            origin=Origin(xyz=(SHOULDER_AXIS_X - 0.004, cheek_y, SHOULDER_Z)),
            material="cast_iron",
            name=f"shoulder_cheek_{name_suffix}",
        )
        shoulder_housing.visual(
            Cylinder(radius=0.024, length=YOKE_CHEEK_THICKNESS),
            origin=Origin(
                xyz=(SHOULDER_AXIS_X, cheek_y, SHOULDER_Z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="cast_iron",
            name=f"shoulder_boss_{name_suffix}",
        )
        shoulder_housing.visual(
            Box((0.070, YOKE_CHEEK_THICKNESS, 0.040)),
            origin=Origin(xyz=(0.012, cheek_y, 0.118)),
            material="cast_iron",
            name=f"shoulder_gusset_{name_suffix}",
        )
    shoulder_housing.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(-0.070, -0.042, 0.009)),
        material="machined_steel",
        name="mount_bolt_1",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(-0.070, 0.042, 0.009)),
        material="machined_steel",
        name="mount_bolt_2",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.030, -0.042, 0.009)),
        material="machined_steel",
        name="mount_bolt_3",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.030, 0.042, 0.009)),
        material="machined_steel",
        name="mount_bolt_4",
    )
    shoulder_housing.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.170)),
        mass=3.8,
        origin=Origin(xyz=(-0.024, 0.0, 0.085)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.009, length=SHOULDER_GAP),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="shoulder_journal",
    )
    upper_link.visual(
        Box((0.020, 0.016, 0.020)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material="machined_steel",
        name="shoulder_neck",
    )
    upper_link.visual(
        Box((0.032, 0.018, 0.026)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material="machined_steel",
        name="upper_transition",
    )
    upper_link.visual(
        Box((0.170, 0.026, 0.048)),
        origin=Origin(xyz=(0.124, 0.0, 0.0)),
        material="machined_steel",
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.126, 0.010, 0.008)),
        origin=Origin(xyz=(0.136, 0.0, 0.020)),
        material="machined_steel",
        name="upper_rib_top",
    )
    upper_link.visual(
        Box((0.126, 0.010, 0.008)),
        origin=Origin(xyz=(0.136, 0.0, -0.020)),
        material="machined_steel",
        name="upper_rib_bottom",
    )
    for sign, name_suffix in ((-1.0, "neg"), (1.0, "pos")):
        cheek_y = sign * (ELBOW_GAP / 2.0 + YOKE_CHEEK_THICKNESS / 2.0)
        upper_link.visual(
            Box((0.016, YOKE_CHEEK_THICKNESS, 0.054)),
            origin=Origin(xyz=(UPPER_LINK_LENGTH - 0.004, cheek_y, 0.0)),
            material="machined_steel",
            name=f"elbow_cheek_{name_suffix}",
        )
        upper_link.visual(
            Cylinder(radius=0.030, length=YOKE_CHEEK_THICKNESS),
            origin=Origin(
                xyz=(UPPER_LINK_LENGTH, cheek_y, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="machined_steel",
            name=f"elbow_boss_{name_suffix}",
        )
        upper_link.visual(
            Box((0.040, YOKE_CHEEK_THICKNESS, 0.024)),
            origin=Origin(xyz=(UPPER_LINK_LENGTH - 0.030, cheek_y, 0.018 * sign)),
            material="machined_steel",
            name=f"elbow_rib_{name_suffix}",
        )
        upper_link.visual(
            Box((0.024, 0.026, 0.020)),
            origin=Origin(xyz=(UPPER_LINK_LENGTH - 0.034, 0.026 * sign, 0.012 * sign)),
            material="machined_steel",
            name=f"elbow_web_{name_suffix}",
        )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.270, 0.090, 0.110)),
        mass=1.25,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.0075, length=ELBOW_GAP),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="anodized_dark",
        name="elbow_journal",
    )
    forearm.visual(
        Box((0.020, 0.012, 0.016)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material="anodized_dark",
        name="forearm_root",
    )
    forearm.visual(
        Box((0.056, 0.010, 0.014)),
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        material="anodized_dark",
        name="forearm_tie",
    )
    forearm.visual(
        Box((0.060, 0.018, 0.024)),
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
        material="anodized_dark",
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.044, 0.010, 0.006)),
        origin=Origin(xyz=(0.106, 0.0, 0.012)),
        material="anodized_dark",
        name="forearm_rib_top",
    )
    forearm.visual(
        Box((0.044, 0.010, 0.006)),
        origin=Origin(xyz=(0.106, 0.0, -0.012)),
        material="anodized_dark",
        name="forearm_rib_bottom",
    )
    forearm.visual(
        Box((0.040, 0.022, 0.038)),
        origin=Origin(xyz=(FOREARM_LENGTH - 0.010, 0.0, 0.0)),
        material="anodized_dark",
        name="pad_carrier",
    )
    forearm.visual(
        Box((0.004, 0.048, 0.056)),
        origin=Origin(xyz=(FOREARM_LENGTH + 0.008, 0.0, 0.0)),
        material="rubber_pad",
        name="contact_pad",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.165, 0.060, 0.075)),
        mass=0.72,
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_housing,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-0.55,
            upper=1.05,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=1.6,
            lower=-0.90,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder_housing = object_model.get_part("shoulder_housing")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")

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

    upper_aabb = ctx.part_world_aabb(upper_link)
    forearm_aabb = ctx.part_world_aabb(forearm)
    upper_dx = upper_aabb[1][0] - upper_aabb[0][0] if upper_aabb is not None else 0.0
    upper_dy = upper_aabb[1][1] - upper_aabb[0][1] if upper_aabb is not None else 0.0
    upper_dz = upper_aabb[1][2] - upper_aabb[0][2] if upper_aabb is not None else 0.0
    forearm_dx = forearm_aabb[1][0] - forearm_aabb[0][0] if forearm_aabb is not None else 0.0
    forearm_dy = forearm_aabb[1][1] - forearm_aabb[0][1] if forearm_aabb is not None else 0.0
    forearm_dz = forearm_aabb[1][2] - forearm_aabb[0][2] if forearm_aabb is not None else 0.0

    ctx.check(
        "parallel_horizontal_joint_axes",
        shoulder_pitch.axis == (0.0, 1.0, 0.0) and elbow_pitch.axis == (0.0, 1.0, 0.0),
        f"shoulder axis={shoulder_pitch.axis}, elbow axis={elbow_pitch.axis}",
    )
    ctx.check(
        "links_have_distinct_proportions",
        upper_dx > forearm_dx + 0.080 and upper_dy > forearm_dy + 0.020,
        (
            f"upper dims=({upper_dx:.3f}, {upper_dy:.3f}, {upper_dz:.3f}), "
            f"forearm dims=({forearm_dx:.3f}, {forearm_dy:.3f}, {forearm_dz:.3f})"
        ),
    )
    ctx.check(
        "contact_pad_present",
        ctx.part_element_world_aabb(forearm, elem="contact_pad") is not None,
        "forearm is missing the rectangular end pad visual",
    )
    ctx.expect_overlap(
        upper_link,
        shoulder_housing,
        axes="yz",
        min_overlap=0.040,
        name="upper_link_seated_in_shoulder_housing",
    )
    ctx.expect_overlap(
        forearm,
        upper_link,
        axes="yz",
        min_overlap=0.020,
        name="forearm_seated_in_elbow_hub",
    )

    with ctx.pose({shoulder_pitch: 0.80, elbow_pitch: 0.65}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_lifted_work_pose")

    with ctx.pose({shoulder_pitch: -0.30, elbow_pitch: 1.00}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_compact_bend_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
