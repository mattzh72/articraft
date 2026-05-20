from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


BODY_RADIUS = 0.036
BODY_THICKNESS = 0.028
BODY_FRONT_Y = BODY_THICKNESS * 0.5
SLOT_HALF_WIDTH = 0.0205
SLOT_FLOOR_CENTER_Z = 0.019
SHACKLE_PIVOT_X = -0.014
SHACKLE_PIVOT_Z = 0.0125
SHACKLE_BAR_RADIUS = 0.0045
SHACKLE_PIN_RADIUS = 0.0032
SHACKLE_COLLAR_RADIUS = 0.0052
SHACKLE_COLLAR_LENGTH = 0.0024
DIAL_X_OFFSETS = (-0.0165, 0.0, 0.0165)
DIAL_CENTER_Z = -0.013
DIAL_OUTER_RADIUS = 0.0074
DIAL_THICKNESS = 0.006
DIAL_HEAD_RADIUS = 0.0043
DIAL_HEAD_THICKNESS = 0.0015
DIAL_REAR_STUB_LENGTH = 0.004
SHACKLE_BAR_FACE_OFFSET = BODY_FRONT_Y + SHACKLE_BAR_RADIUS + 0.0002
DIAL_SEAT_RADIUS = 0.0038
DIAL_SEAT_THICKNESS = 0.0012
DIAL_CLIP_STEM_RADIUS = 0.0024
DIAL_CLIP_STEM_LENGTH = 0.0010


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _outer_arc_points(radius: float, start_angle: float, end_angle: float, segments: int):
    return [
        (radius * math.cos(angle), radius * math.sin(angle))
        for angle in (
            start_angle + (end_angle - start_angle) * step / (segments - 1)
            for step in range(segments)
        )
    ]


def _circle_profile(radius: float, segments: int = 36, *, cx: float = 0.0, cy: float = 0.0):
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * step) / segments),
            cy + radius * math.sin((2.0 * math.pi * step) / segments),
        )
        for step in range(segments)
    ]


def _build_body_shell_mesh():
    mouth_z = math.sqrt(BODY_RADIUS**2 - SLOT_HALF_WIDTH**2)
    right_mouth_angle = math.atan2(mouth_z, SLOT_HALF_WIDTH)
    left_mouth_angle = math.pi - right_mouth_angle

    profile = _outer_arc_points(
        BODY_RADIUS,
        left_mouth_angle,
        right_mouth_angle + (2.0 * math.pi),
        88,
    )
    profile.append((SLOT_HALF_WIDTH, SLOT_FLOOR_CENTER_Z))
    for step in range(1, 24):
        angle = math.pi * step / 24.0
        profile.append(
            (
                SLOT_HALF_WIDTH * math.cos(angle),
                SLOT_FLOOR_CENTER_Z - SLOT_HALF_WIDTH * math.sin(angle),
            )
        )
    profile.append((-SLOT_HALF_WIDTH, SLOT_FLOOR_CENTER_Z))
    profile.append((-SLOT_HALF_WIDTH, mouth_z))

    return ExtrudeGeometry(profile, BODY_THICKNESS, cap=True, center=True).rotate_x(math.pi / 2.0)


def _build_shackle_bar_mesh():
    return wire_from_points(
        [
            (0.0, 0.0, 0.0056),
            (0.0, 0.0, 0.015),
            (0.005, 0.0, 0.026),
            (0.014, 0.0, 0.031),
            (0.023, 0.0, 0.026),
            (0.028, 0.0, 0.015),
            (0.028, 0.0, 0.0005),
        ],
        radius=SHACKLE_BAR_RADIUS,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.0085,
        corner_segments=10,
    )


def _axis_matches(axis, expected) -> bool:
    return tuple(round(float(value), 3) for value in axis) == expected


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_discus_padlock")

    body_steel = model.material("body_steel", rgba=(0.17, 0.18, 0.19, 1.0))
    face_black = model.material("face_black", rgba=(0.08, 0.09, 0.10, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    dial_metal = model.material("dial_metal", rgba=(0.66, 0.63, 0.54, 1.0))
    indicator = model.material("indicator", rgba=(0.90, 0.91, 0.87, 1.0))

    body = model.part("body")
    body.visual(_save_mesh("padlock_body_shell", _build_body_shell_mesh()), material=body_steel, name="body_shell")
    body.visual(
        Cylinder(radius=0.031, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=face_black,
        name="front_face_boss",
    )
    body.visual(
        Cylinder(radius=SHACKLE_PIN_RADIUS, length=BODY_THICKNESS * 0.92),
        origin=Origin(
            xyz=(SHACKLE_PIVOT_X, 0.0, SHACKLE_PIVOT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shackle_steel,
        name="shackle_pin",
    )
    for index, x_pos in enumerate(DIAL_X_OFFSETS, start=1):
        body.visual(
            Cylinder(radius=DIAL_SEAT_RADIUS, length=DIAL_SEAT_THICKNESS),
            origin=Origin(
                xyz=(
                    x_pos,
                    BODY_FRONT_Y - (DIAL_SEAT_THICKNESS * 0.5),
                    DIAL_CENTER_Z,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=shackle_steel,
            name=f"dial_{index}_seat",
        )
        body.visual(
            Box((0.0048, 0.0016, 0.0024)),
            origin=Origin(
                xyz=(x_pos, BODY_FRONT_Y + 0.0008, DIAL_CENTER_Z + DIAL_OUTER_RADIUS + 0.0032),
            ),
            material=indicator,
            name=f"dial_{index}_index_mark",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_RADIUS * 2.0, BODY_THICKNESS, BODY_RADIUS * 2.0)),
        mass=0.55,
    )

    shackle = model.part("shackle")
    shackle.visual(
        Cylinder(radius=SHACKLE_COLLAR_RADIUS, length=SHACKLE_COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, BODY_FRONT_Y + (SHACKLE_COLLAR_LENGTH * 0.5), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shackle_steel,
        name="pivot_ring",
    )
    shackle.visual(
        Cylinder(radius=SHACKLE_BAR_RADIUS * 0.96, length=0.0046),
        origin=Origin(
            xyz=(0.0, BODY_FRONT_Y + 0.0023, 0.0060),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shackle_steel,
        name="pivot_bridge",
    )
    shackle.visual(
        _save_mesh("padlock_shackle_bar", _build_shackle_bar_mesh()),
        origin=Origin(xyz=(0.0, SHACKLE_BAR_FACE_OFFSET, 0.0)),
        material=shackle_steel,
        name="bar",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.038, 0.013, 0.040)),
        mass=0.12,
        origin=Origin(xyz=(0.014, 0.0, 0.020)),
    )

    for index, x_pos in enumerate(DIAL_X_OFFSETS, start=1):
        dial = model.part(f"dial_{index}")
        dial.visual(
            Cylinder(radius=DIAL_OUTER_RADIUS, length=DIAL_THICKNESS),
            origin=Origin(
                xyz=(0.0, BODY_FRONT_Y + (DIAL_THICKNESS * 0.5), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dial_metal,
            name="wheel",
        )
        dial.visual(
            Cylinder(radius=DIAL_CLIP_STEM_RADIUS, length=DIAL_CLIP_STEM_LENGTH),
            origin=Origin(
                xyz=(
                    0.0,
                    BODY_FRONT_Y + DIAL_THICKNESS + (DIAL_CLIP_STEM_LENGTH * 0.5),
                    0.0,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=shackle_steel,
            name="clip_stem",
        )
        dial.visual(
            Cylinder(radius=DIAL_HEAD_RADIUS, length=DIAL_HEAD_THICKNESS),
            origin=Origin(
                xyz=(
                    0.0,
                    BODY_FRONT_Y + DIAL_THICKNESS + DIAL_CLIP_STEM_LENGTH + (DIAL_HEAD_THICKNESS * 0.5),
                    0.0,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=shackle_steel,
            name="clip_head",
        )
        dial.visual(
            Box((0.0026, 0.0008, 0.0022)),
            origin=Origin(xyz=(0.0, BODY_FRONT_Y + DIAL_THICKNESS + 0.0004, DIAL_OUTER_RADIUS - 0.0016)),
            material=indicator,
            name="digit_mark",
        )
        dial.inertial = Inertial.from_geometry(
            Cylinder(radius=DIAL_OUTER_RADIUS, length=DIAL_THICKNESS),
            mass=0.018,
            origin=Origin(xyz=(0.0, BODY_FRONT_Y + (DIAL_THICKNESS * 0.5), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"dial_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(x_pos, 0.0, DIAL_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=12.0),
        )

    model.articulation(
        "shackle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(SHACKLE_PIVOT_X, 0.0, SHACKLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    shackle_joint = object_model.get_articulation("shackle_pivot")
    dial_parts = [object_model.get_part(f"dial_{index}") for index in range(1, 4)]
    dial_joints = [object_model.get_articulation(f"dial_{index}_spin") for index in range(1, 4)]

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
        "shackle_axis_on_side_pivot",
        _axis_matches(shackle_joint.axis, (0.0, 1.0, 0.0)),
        f"expected shackle pivot axis (0, 1, 0), got {shackle_joint.axis}",
    )
    ctx.check(
        "shackle_has_opening_range",
        shackle_joint.motion_limits is not None
        and shackle_joint.motion_limits.lower == 0.0
        and shackle_joint.motion_limits.upper is not None
        and shackle_joint.motion_limits.upper >= 1.0,
        "shackle should open upward from the closed pose with a realistic positive range",
    )

    for index, dial_joint in enumerate(dial_joints, start=1):
        ctx.check(
            f"dial_{index}_is_continuous_on_face_axis",
            dial_joint.articulation_type == ArticulationType.CONTINUOUS
            and _axis_matches(dial_joint.axis, (0.0, 1.0, 0.0)),
            f"dial {index} should spin continuously around its local face shaft; got {dial_joint.articulation_type} axis {dial_joint.axis}",
        )

    ctx.expect_contact(
        shackle,
        body,
        contact_tol=5e-4,
        name="shackle_is_captive_in_closed_pose",
    )

    for index, dial in enumerate(dial_parts, start=1):
        ctx.expect_contact(
            dial,
            body,
            contact_tol=2e-4,
            name=f"dial_{index}_is_captured_on_body",
        )
        ctx.expect_overlap(
            dial,
            body,
            axes="xz",
            min_overlap=0.011,
            name=f"dial_{index}_sits_within_lock_face_footprint",
        )

    with ctx.pose({shackle_joint: 1.0}):
        ctx.expect_contact(
            shackle,
            body,
            contact_tol=5e-4,
            name="shackle_stays_captive_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
