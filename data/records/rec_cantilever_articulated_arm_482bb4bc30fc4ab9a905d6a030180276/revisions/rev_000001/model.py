from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

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


BASE_LENGTH = 0.28
BASE_WIDTH = 0.20
BASE_HEIGHT = 0.035
COUNTER_LENGTH = 0.12
COUNTER_WIDTH = 0.18
COUNTER_HEIGHT = 0.095

COLUMN_LENGTH = 0.10
COLUMN_WIDTH = 0.11
COLUMN_HEIGHT = 0.48

BEAM_LENGTH = 0.36
BEAM_WIDTH = 0.11
BEAM_HEIGHT = 0.08

SHOULDER_X = 0.14
SHOULDER_Z = 0.58

SHOULDER_LINK_LENGTH = 0.44
ELBOW_LINK_LENGTH = 0.36
WRIST_LINK_LENGTH = 0.20

JOINT_RADIUS = 0.018
SUPPORT_GAP = 0.028
STAGE_2_GAP = 0.026
STAGE_3_GAP = 0.024
EAR_THICKNESS = 0.014

SHOULDER_LIMITS = (-0.60, 0.95)
ELBOW_LIMITS = (-1.25, 1.20)
WRIST_LIMITS = (-0.95, 1.05)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _xz_profile_prism(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width / 2.0, both=True)


def _underslung_link_shape(
    *,
    length: float,
    body_width: float,
    body_height: float,
    root_lug_width: float,
    child_gap: float,
) -> cq.Workplane:
    body_top = -0.034
    body_z = body_top - (body_height / 2.0)
    web_width = max(body_width * 0.72, root_lug_width + 0.012)
    child_ear_y = (child_gap / 2.0) + (EAR_THICKNESS / 2.0)
    root_block_length = 0.024
    neck_length = 0.060
    body_start = 0.076
    body_length = max(length - 0.190, 0.150)
    distal_block_length = 0.080
    distal_block_center = length - 0.064
    fork_ear_length = 0.032

    shape = _box(
        (root_block_length, root_lug_width, 0.050),
        (root_block_length / 2.0, 0.0, 0.0),
    )
    shape = shape.union(
        _box(
            (neck_length, web_width, 0.088),
            (0.052, 0.0, -0.044),
        )
    )
    shape = shape.union(
        _box(
            (body_length, body_width, body_height),
            (body_start + (body_length / 2.0), 0.0, body_z),
        )
    )
    shape = shape.union(
        _box(
            (0.135, body_width * 0.76, 0.020),
            (body_start + 0.055, 0.0, -0.020),
        )
    )
    shape = shape.union(
        _box(
            (distal_block_length, web_width, 0.074),
            (distal_block_center, 0.0, -0.038),
        )
    )
    shape = shape.union(
        _box(
            (0.024, web_width * 0.82, 0.046),
            (length - 0.108, 0.0, -0.034),
        )
    )

    for y_pos in (-child_ear_y, child_ear_y):
        shape = shape.union(
            _box(
                (fork_ear_length, EAR_THICKNESS, 0.052),
                (length - (fork_ear_length / 2.0), y_pos, 0.0),
            )
        )

    return shape


def _wrist_stage_shape(
    *,
    length: float,
    body_width: float,
    body_height: float,
    root_lug_width: float,
) -> cq.Workplane:
    body_top = -0.030
    body_z = body_top - (body_height / 2.0)
    web_width = max(body_width * 0.78, root_lug_width + 0.012)
    root_block_length = 0.024

    shape = _box(
        (root_block_length, root_lug_width, 0.048),
        (root_block_length / 2.0, 0.0, 0.0),
    )
    shape = shape.union(_box((0.072, web_width, 0.084), (0.050, 0.0, -0.042)))
    shape = shape.union(
        _box(
            (length - 0.030, body_width, body_height),
            (0.036 + ((length - 0.030) / 2.0), 0.0, body_z),
        )
    )
    shape = shape.union(_box((0.060, web_width, 0.066), (length - 0.020, 0.0, -0.034)))
    shape = shape.union(_box((0.068, body_width * 0.84, 0.022), (length + 0.004, 0.0, -0.030)))
    shape = shape.union(_box((0.022, body_width * 0.80, 0.092), (length + 0.018, 0.0, -0.012)))
    shape = shape.union(_box((0.040, body_width * 0.54, 0.046), (length + 0.048, 0.0, -0.022)))
    return shape


def _support_frame_shape() -> cq.Workplane:
    beam_center_x = -0.040
    beam_center_z = 0.65
    support_ear_y = (SUPPORT_GAP / 2.0) + (EAR_THICKNESS / 2.0)

    shape = _box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT), (-0.045, 0.0, BASE_HEIGHT / 2.0))
    shape = shape.union(
        _box(
            (COUNTER_LENGTH, COUNTER_WIDTH, COUNTER_HEIGHT),
            (-0.115, 0.0, BASE_HEIGHT + COUNTER_HEIGHT / 2.0),
        )
    )
    shape = shape.union(
        _box(
            (COLUMN_LENGTH, COLUMN_WIDTH, COLUMN_HEIGHT),
            (-0.080, 0.0, BASE_HEIGHT + (COLUMN_HEIGHT / 2.0)),
        )
    )
    shape = shape.union(_box((0.240, BEAM_WIDTH, BEAM_HEIGHT), (beam_center_x, 0.0, beam_center_z)))

    brace = _xz_profile_prism(
        [
            (-0.120, 0.295),
            (-0.015, 0.295),
            (0.110, 0.610),
            (0.015, 0.610),
        ],
        0.062,
    )
    shape = shape.union(brace)

    shape = shape.union(_box((0.034, 0.050, 0.092), (0.082, 0.0, 0.604)))
    shape = shape.union(_box((0.078, SUPPORT_GAP + (2.0 * EAR_THICKNESS), 0.020), (0.101, 0.0, 0.620)))
    for y_pos in (-support_ear_y, support_ear_y):
        shape = shape.union(_box((0.032, EAR_THICKNESS, 0.056), (SHOULDER_X - 0.016, y_pos, SHOULDER_Z)))

    shape = shape.union(_box((0.110, 0.070, 0.030), (0.035, 0.0, beam_center_z - 0.026)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_cantilever_arm")

    model.material("support_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("arm_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("wrist_gray", rgba=(0.36, 0.38, 0.41, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_support_frame_shape(), "support_frame"),
        material="support_graphite",
        name="support_shell",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.40, 0.22, 0.68)),
        mass=16.0,
        origin=Origin(xyz=(-0.025, 0.0, 0.33)),
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        mesh_from_cadquery(
            _underslung_link_shape(
                length=SHOULDER_LINK_LENGTH,
                body_width=0.072,
                body_height=0.074,
                root_lug_width=SUPPORT_GAP,
                child_gap=STAGE_2_GAP,
            ),
            "shoulder_link",
        ),
        material="arm_aluminum",
        name="shoulder_shell",
    )
    shoulder_link.inertial = Inertial.from_geometry(
        Box((SHOULDER_LINK_LENGTH, 0.09, 0.15)),
        mass=4.2,
        origin=Origin(xyz=(SHOULDER_LINK_LENGTH * 0.55, 0.0, -0.052)),
    )

    elbow_link = model.part("elbow_link")
    elbow_link.visual(
        mesh_from_cadquery(
            _underslung_link_shape(
                length=ELBOW_LINK_LENGTH,
                body_width=0.066,
                body_height=0.068,
                root_lug_width=STAGE_2_GAP,
                child_gap=STAGE_3_GAP,
            ),
            "elbow_link",
        ),
        material="arm_aluminum",
        name="elbow_shell",
    )
    elbow_link.inertial = Inertial.from_geometry(
        Box((ELBOW_LINK_LENGTH, 0.082, 0.14)),
        mass=3.1,
        origin=Origin(xyz=(ELBOW_LINK_LENGTH * 0.54, 0.0, -0.050)),
    )

    wrist_link = model.part("wrist_link")
    wrist_link.visual(
        mesh_from_cadquery(
            _wrist_stage_shape(
                length=WRIST_LINK_LENGTH,
                body_width=0.056,
                body_height=0.062,
                root_lug_width=STAGE_3_GAP,
            ),
            "wrist_link",
        ),
        material="wrist_gray",
        name="wrist_shell",
    )
    wrist_link.inertial = Inertial.from_geometry(
        Box((WRIST_LINK_LENGTH + 0.08, 0.070, 0.13)),
        mass=1.8,
        origin=Origin(xyz=(WRIST_LINK_LENGTH * 0.62, 0.0, -0.040)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=shoulder_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=radians(35.0),
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=(SHOULDER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=radians(45.0),
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
        ),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=elbow_link,
        child=wrist_link,
        origin=Origin(xyz=(ELBOW_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=radians(70.0),
            lower=WRIST_LIMITS[0],
            upper=WRIST_LIMITS[1],
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    shoulder_link = object_model.get_part("shoulder_link")
    elbow_link = object_model.get_part("elbow_link")
    wrist_link = object_model.get_part("wrist_link")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

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

    ctx.expect_contact(
        support_frame,
        shoulder_link,
        contact_tol=0.0025,
        name="support clevis carries shoulder stage",
    )
    ctx.expect_contact(
        shoulder_link,
        elbow_link,
        contact_tol=0.0025,
        name="shoulder stage carries elbow stage",
    )
    ctx.expect_contact(
        elbow_link,
        wrist_link,
        contact_tol=0.0025,
        name="elbow stage carries wrist stage",
    )

    ctx.check(
        "serial joints are all pitch revolutes",
        shoulder_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and wrist_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(shoulder_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(elbow_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(wrist_joint.axis) == (0.0, -1.0, 0.0),
        "Expected shoulder, elbow, and wrist to form a serial pitch chain.",
    )

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.0}):
        neutral_elbow_z = ctx.part_world_position(elbow_link)[2]
        neutral_wrist_z = ctx.part_world_position(wrist_link)[2]

    with ctx.pose({shoulder_joint: 0.70, elbow_joint: 0.0, wrist_joint: 0.0}):
        raised_elbow_z = ctx.part_world_position(elbow_link)[2]
        raised_wrist_z = ctx.part_world_position(wrist_link)[2]

    ctx.check(
        "positive shoulder motion lifts the downstream chain",
        raised_elbow_z > neutral_elbow_z + 0.10 and raised_wrist_z > neutral_wrist_z + 0.12,
        (
            f"Expected shoulder lift to raise elbow/wrist origins, got "
            f"elbow {neutral_elbow_z:.3f}->{raised_elbow_z:.3f}, "
            f"wrist {neutral_wrist_z:.3f}->{raised_wrist_z:.3f}."
        ),
    )

    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 0.0, wrist_joint: 0.0}):
        elbow_neutral_wrist_z = ctx.part_world_position(wrist_link)[2]

    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 0.85, wrist_joint: 0.0}):
        elbow_raised_wrist_z = ctx.part_world_position(wrist_link)[2]

    ctx.check(
        "positive elbow motion raises the wrist stage",
        elbow_raised_wrist_z > elbow_neutral_wrist_z + 0.08,
        (
            f"Expected elbow lift to raise the wrist stage, got "
            f"{elbow_neutral_wrist_z:.3f}->{elbow_raised_wrist_z:.3f}."
        ),
    )

    with ctx.pose({shoulder_joint: 0.35, elbow_joint: 0.30, wrist_joint: 0.0}):
        wrist_neutral_center_z = _center_z(ctx.part_world_aabb(wrist_link))

    with ctx.pose({shoulder_joint: 0.35, elbow_joint: 0.30, wrist_joint: 0.70}):
        wrist_raised_center_z = _center_z(ctx.part_world_aabb(wrist_link))

    ctx.check(
        "positive wrist motion pitches the end stage upward",
        wrist_raised_center_z > wrist_neutral_center_z + 0.012,
        (
            f"Expected wrist pitch to raise the end stage center, got "
            f"{wrist_neutral_center_z:.3f}->{wrist_raised_center_z:.3f}."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
