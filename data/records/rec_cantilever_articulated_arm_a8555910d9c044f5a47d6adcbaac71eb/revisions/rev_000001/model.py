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


BASE_FOOT_X = 0.160
BASE_FOOT_Y = 0.120
BASE_FOOT_T = 0.014
COLUMN_X = 0.060
COLUMN_Y = 0.056
COLUMN_Z = 0.088
SHOULDER_X = 0.094
SHOULDER_Z = BASE_FOOT_T + COLUMN_Z + 0.002

LINK1_LEN = 0.170
LINK2_LEN = 0.145

BASE_YOKE_GAP = 0.028
BASE_CHEEK_T = 0.010
ELBOW_GAP = 0.024
ELBOW_CHEEK_T = 0.009
WRIST_GAP = 0.022
WRIST_CHEEK_T = 0.008


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BASE_FOOT_X, BASE_FOOT_Y, BASE_FOOT_T)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.048, -0.036), (-0.048, 0.036), (0.048, -0.036), (0.048, 0.036)])
        .hole(0.010)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, BASE_FOOT_T / 2.0))
    )

    column = _box((COLUMN_X, COLUMN_Y, COLUMN_Z), (-0.036, 0.0, BASE_FOOT_T + COLUMN_Z / 2.0))
    arm_stem = _box((0.118, 0.034, 0.018), (0.016, 0.0, SHOULDER_Z - 0.031))
    shoulder_pad = _box((0.032, 0.040, 0.018), (0.074, 0.0, SHOULDER_Z - 0.011))
    top_cap = _box((0.018, 0.040, 0.010), (0.085, 0.0, SHOULDER_Z + 0.015))

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.050, BASE_FOOT_T + 0.010),
                (-0.022, BASE_FOOT_T + 0.010),
                (0.042, SHOULDER_Z - 0.040),
                (0.012, SHOULDER_Z - 0.040),
            ]
        )
        .close()
        .extrude(0.046)
        .translate((0.0, -0.023, 0.0))
    )

    base = foot.union(column).union(arm_stem).union(shoulder_pad).union(top_cap).union(gusset)

    cheek_y = BASE_YOKE_GAP / 2.0 + BASE_CHEEK_T / 2.0
    bridge = _box((0.016, 0.048, 0.018), (SHOULDER_X - 0.016, 0.0, SHOULDER_Z))
    base = base.union(bridge)
    for sign in (-1.0, 1.0):
        y_center = sign * cheek_y
        cheek = _box(
            (0.020, BASE_CHEEK_T, 0.044),
            (SHOULDER_X - 0.010, y_center, SHOULDER_Z),
        )
        base = base.union(cheek)

    return base


def _primary_link_shape() -> cq.Workplane:
    root_lug = _box((0.020, 0.020, 0.036), (0.010, 0.0, 0.0))
    root_body = _box((0.028, 0.026, 0.024), (0.030, 0.0, 0.0))
    beam = _box((0.112, 0.022, 0.018), (0.086, 0.0, 0.0))
    spine = _box((0.060, 0.014, 0.012), (0.094, 0.0, 0.012))
    elbow_bridge = _box((0.024, 0.040, 0.018), (LINK1_LEN - 0.022, 0.0, 0.0))
    link = root_lug.union(root_body).union(beam).union(spine).union(elbow_bridge)

    cheek_y = ELBOW_GAP / 2.0 + ELBOW_CHEEK_T / 2.0
    for sign in (-1.0, 1.0):
        y_center = sign * cheek_y
        cheek = _box((0.016, ELBOW_CHEEK_T, 0.036), (LINK1_LEN - 0.008, y_center, 0.0))
        link = link.union(cheek)

    return link


def _secondary_link_shape() -> cq.Workplane:
    root_lug = _box((0.022, ELBOW_GAP, 0.028), (0.001, 0.0, 0.0))
    root_body = _box((0.034, 0.020, 0.022), (0.024, 0.0, 0.0))
    beam = _box((0.094, 0.020, 0.017), (0.077, 0.0, 0.0))
    spine = _box((0.050, 0.014, 0.011), (0.084, 0.0, 0.011))
    wrist_bridge = _box((0.020, 0.036, 0.016), (LINK2_LEN - 0.020, 0.0, 0.0))
    link = root_lug.union(root_body).union(beam).union(spine).union(wrist_bridge)

    cheek_y = WRIST_GAP / 2.0 + WRIST_CHEEK_T / 2.0
    for sign in (-1.0, 1.0):
        y_center = sign * cheek_y
        cheek = _box((0.014, WRIST_CHEEK_T, 0.030), (LINK2_LEN - 0.007, y_center, 0.0))
        link = link.union(cheek)

    return link


def _wrist_shape() -> cq.Workplane:
    root_lug = _box((0.020, WRIST_GAP, 0.024), (0.000, 0.0, 0.0))
    neck = _box((0.034, 0.020, 0.018), (0.024, 0.0, 0.0))
    brace = _box((0.014, 0.034, 0.034), (0.040, 0.0, 0.0))
    face_plate = _box((0.012, 0.052, 0.052), (0.053, 0.0, 0.0))
    return root_lug.union(neck).union(brace).union(face_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_cantilever_arm")

    model.material("powder_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("dark_anodized", rgba=(0.32, 0.35, 0.38, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "cantilever_base"), material="powder_graphite", name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.104)),
        mass=4.8,
        origin=Origin(xyz=(-0.010, 0.0, 0.052)),
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        mesh_from_cadquery(_primary_link_shape(), "cantilever_primary_link"),
        material="machined_aluminum",
        name="primary_link_body",
    )
    primary_link.inertial = Inertial.from_geometry(
        Box((0.170, 0.048, 0.048)),
        mass=1.1,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        mesh_from_cadquery(_secondary_link_shape(), "cantilever_secondary_link"),
        material="machined_aluminum",
        name="secondary_link_body",
    )
    secondary_link.inertial = Inertial.from_geometry(
        Box((0.145, 0.042, 0.042)),
        mass=0.8,
        origin=Origin(xyz=(0.0725, 0.0, 0.0)),
    )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(
        mesh_from_cadquery(_wrist_shape(), "cantilever_wrist_face"),
        material="dark_anodized",
        name="wrist_face_body",
    )
    wrist_face.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.060)),
        mass=0.35,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=primary_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.15, effort=35.0, velocity=1.2),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.20, upper=1.45, effort=22.0, velocity=1.5),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=wrist_face,
        origin=Origin(xyz=(LINK2_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=10.0, velocity=2.0),
    )

    return model


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    wrist_face = object_model.get_part("wrist_face")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        primary_link,
        secondary_link,
        reason="Simplified elbow uses nested clevis-and-lug geometry at the shared pivot envelope.",
    )
    ctx.allow_overlap(
        secondary_link,
        wrist_face,
        reason="Simplified wrist uses nested hinge knuckles at the shared pivot envelope.",
    )

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
        "pitch_axes_configured",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist.axis == (0.0, -1.0, 0.0),
        details=f"axes were shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )

    ctx.expect_contact(primary_link, base, contact_tol=0.001, name="shoulder_joint_seated")
    ctx.expect_contact(secondary_link, primary_link, contact_tol=0.001, name="elbow_joint_seated")
    ctx.expect_contact(wrist_face, secondary_link, contact_tol=0.001, name="wrist_joint_seated")
    ctx.expect_gap(wrist_face, base, axis="x", min_gap=0.18, name="wrist_clearly_overhangs_base")

    rest_wrist_z = _aabb_center_z(ctx.part_world_aabb(wrist_face))
    with ctx.pose({shoulder: 0.65}):
        shoulder_raised_wrist_z = _aabb_center_z(ctx.part_world_aabb(wrist_face))
    with ctx.pose({elbow: 0.85}):
        elbow_raised_wrist_z = _aabb_center_z(ctx.part_world_aabb(wrist_face))
    with ctx.pose({wrist: 0.85}):
        wrist_raised_z = _aabb_center_z(ctx.part_world_aabb(wrist_face))

    ctx.check(
        "shoulder_positive_motion_lifts_arm",
        rest_wrist_z is not None
        and shoulder_raised_wrist_z is not None
        and shoulder_raised_wrist_z > rest_wrist_z + 0.09,
        details=f"rest_z={rest_wrist_z}, shoulder_pose_z={shoulder_raised_wrist_z}",
    )
    ctx.check(
        "elbow_positive_motion_lifts_wrist",
        rest_wrist_z is not None
        and elbow_raised_wrist_z is not None
        and elbow_raised_wrist_z > rest_wrist_z + 0.04,
        details=f"rest_z={rest_wrist_z}, elbow_pose_z={elbow_raised_wrist_z}",
    )
    ctx.check(
        "wrist_positive_motion_tilts_face_up",
        rest_wrist_z is not None
        and wrist_raised_z is not None
        and wrist_raised_z > rest_wrist_z + 0.01,
        details=f"rest_z={rest_wrist_z}, wrist_pose_z={wrist_raised_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
