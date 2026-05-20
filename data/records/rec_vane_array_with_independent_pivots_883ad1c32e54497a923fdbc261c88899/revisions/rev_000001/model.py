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


OUTER_W = 0.56
OUTER_H = 0.44
FRAME_D = 0.068
RAIL_W = 0.032
BAR_H = 0.03
OPENING_W = OUTER_W - 2.0 * RAIL_W
OPENING_H = OUTER_H - 2.0 * BAR_H

BLADE_COUNT = 7
AXIS_Y = 0.0
STACK_MARGIN_Z = 0.028
BLADE_PITCH = (OPENING_H - 2.0 * STACK_MARGIN_Z) / (BLADE_COUNT - 1)

SLEEVE_PROTRUSION = 0.004
SLEEVE_EMBED = 0.011
SLEEVE_LENGTH = SLEEVE_PROTRUSION + SLEEVE_EMBED
SLEEVE_OUTER_R = 0.0105
SLEEVE_HOLE_R = 0.0046
SLEEVE_PAD_X = 0.010
SLEEVE_PAD_Y = 0.020
SLEEVE_PAD_Z = 0.028

CONTACT_FACE_X = OPENING_W / 2.0 - SLEEVE_PROTRUSION
BODY_SPAN = 0.466
COLLAR_R = 0.0080
COLLAR_LEN = 0.006
STEM_R = 0.0056

BLADE_CHORD = 0.052
LEAD_R = 0.0048
TRAIL_R = 0.0018
BLADE_VISUAL_TILT = -0.33


def _blade_z_positions() -> list[float]:
    base = -0.5 * (BLADE_COUNT - 1) * BLADE_PITCH
    return [base + idx * BLADE_PITCH for idx in range(BLADE_COUNT)]


def _make_frame_shape(z_positions: list[float]) -> cq.Workplane:
    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_W, FRAME_D, OUTER_H)
        .translate((-(OUTER_W / 2.0 - RAIL_W / 2.0), 0.0, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_W, FRAME_D, OUTER_H)
        .translate(((OUTER_W / 2.0 - RAIL_W / 2.0), 0.0, 0.0))
    )
    top_bar = (
        cq.Workplane("XY")
        .box(OUTER_W, FRAME_D, BAR_H)
        .translate((0.0, 0.0, OUTER_H / 2.0 - BAR_H / 2.0))
    )
    bottom_bar = (
        cq.Workplane("XY")
        .box(OUTER_W, FRAME_D, BAR_H)
        .translate((0.0, 0.0, -(OUTER_H / 2.0 - BAR_H / 2.0)))
    )
    frame = left_rail.union(right_rail).union(top_bar).union(bottom_bar)

    rib_len = OUTER_H - 0.062
    rib_depth = 0.006
    rib_width = 0.012
    for sx in (-1.0, 1.0):
        rib_x = sx * (OUTER_W / 2.0 + rib_depth / 2.0 - 0.003)
        for sy in (-1.0, 1.0):
            rib_y = sy * 0.017
            frame = frame.union(
                cq.Workplane("XY")
                .box(rib_depth, rib_width, rib_len)
                .translate((rib_x, rib_y, 0.0))
            )

    rail_mid_x = OUTER_W / 2.0 - RAIL_W / 2.0
    sleeve_center_abs_x = OPENING_W / 2.0 + 0.5 * (SLEEVE_EMBED - SLEEVE_PROTRUSION)
    through_len = RAIL_W + SLEEVE_PROTRUSION + 0.012

    for z in z_positions:
        for sx in (-1.0, 1.0):
            sleeve_center_x = sx * sleeve_center_abs_x
            pad_center_x = sx * (OPENING_W / 2.0 - 0.001)
            hole_center_x = sx * rail_mid_x

            sleeve = (
                cq.Workplane("YZ")
                .center(AXIS_Y, z)
                .circle(SLEEVE_OUTER_R)
                .extrude(SLEEVE_LENGTH / 2.0, both=True)
                .translate((sleeve_center_x, 0.0, 0.0))
            )
            pad = (
                cq.Workplane("XY")
                .box(SLEEVE_PAD_X, SLEEVE_PAD_Y, SLEEVE_PAD_Z)
                .translate((pad_center_x, AXIS_Y, z))
            )
            hole = (
                cq.Workplane("YZ")
                .center(AXIS_Y, z)
                .circle(SLEEVE_HOLE_R)
                .extrude(through_len / 2.0, both=True)
                .translate((hole_center_x, 0.0, 0.0))
            )
            frame = frame.union(sleeve).union(pad).cut(hole)

    return frame


def _make_blade_shapes() -> tuple[cq.Workplane, cq.Workplane]:
    front = 0.023
    rear = 0.029

    core_points = [
        (-rear, 0.0009),
        (-0.020, 0.0037),
        (-0.006, 0.0054),
        (0.008, 0.0058),
        (0.019, 0.0041),
        (front, 0.0011),
        (0.020, -0.0018),
        (0.010, -0.0042),
        (-0.004, -0.0050),
        (-0.018, -0.0034),
        (-rear, -0.0012),
    ]
    body = cq.Workplane("YZ").polyline(core_points).close().extrude(BODY_SPAN / 2.0, both=True)

    nose = (
        cq.Workplane("YZ")
        .center(front - LEAD_R * 0.35, 0.0)
        .circle(LEAD_R)
        .extrude(BODY_SPAN / 2.0, both=True)
    )
    rear_hem = (
        cq.Workplane("YZ")
        .center(-rear + TRAIL_R * 0.75, -0.0015)
        .circle(TRAIL_R)
        .extrude(BODY_SPAN / 2.0, both=True)
    )
    rear_flange = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.020, -0.0018),
                (-rear + 0.0035, -0.0018),
                (-rear + 0.0020, -0.0004),
                (-0.020, -0.0006),
            ]
        )
        .close()
        .extrude(BODY_SPAN / 2.0, both=True)
    )
    blade_body = body.union(nose).union(rear_hem).union(rear_flange)

    body_half = BODY_SPAN / 2.0
    stem_len = CONTACT_FACE_X - body_half
    stem_center_abs_x = body_half + stem_len / 2.0
    collar_center_abs_x = CONTACT_FACE_X - COLLAR_LEN / 2.0
    blade_collars = None
    for sx in (-1.0, 1.0):
        stem = (
            cq.Workplane("YZ")
            .circle(STEM_R)
            .extrude(stem_len / 2.0, both=True)
            .translate((sx * stem_center_abs_x, 0.0, 0.0))
        )
        collar = (
            cq.Workplane("YZ")
            .circle(COLLAR_R)
            .extrude(COLLAR_LEN / 2.0, both=True)
            .translate((sx * collar_center_abs_x, 0.0, 0.0))
        )
        blade_body = blade_body.union(stem)
        blade_collars = collar if blade_collars is None else blade_collars.union(collar)

    return blade_body, blade_collars


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ventilation_shutter_module")

    frame_color = model.material("frame_color", rgba=(0.20, 0.22, 0.24, 1.0))
    blade_color = model.material("blade_color", rgba=(0.78, 0.80, 0.82, 1.0))

    z_positions = _blade_z_positions()
    blade_body_shape, blade_collar_shape = _make_blade_shapes()

    frame = model.part("frame")
    left_rail_x = -(OUTER_W / 2.0 - RAIL_W / 2.0)
    right_rail_x = OUTER_W / 2.0 - RAIL_W / 2.0
    top_bar_z = OUTER_H / 2.0 - BAR_H / 2.0
    bottom_bar_z = -(OUTER_H / 2.0 - BAR_H / 2.0)

    frame.visual(
        Box((RAIL_W, FRAME_D, OUTER_H)),
        origin=Origin(xyz=(left_rail_x, 0.0, 0.0)),
        material=frame_color,
        name="left_rail",
    )
    frame.visual(
        Box((RAIL_W, FRAME_D, OUTER_H)),
        origin=Origin(xyz=(right_rail_x, 0.0, 0.0)),
        material=frame_color,
        name="right_rail",
    )
    frame.visual(
        Box((OUTER_W, FRAME_D, BAR_H)),
        origin=Origin(xyz=(0.0, 0.0, top_bar_z)),
        material=frame_color,
        name="top_bar",
    )
    frame.visual(
        Box((OUTER_W, FRAME_D, BAR_H)),
        origin=Origin(xyz=(0.0, 0.0, bottom_bar_z)),
        material=frame_color,
        name="bottom_bar",
    )

    rib_len = OUTER_H - 0.062
    rib_depth = 0.006
    rib_width = 0.012
    for side_name, rail_x, sx in (
        ("left", left_rail_x, -1.0),
        ("right", right_rail_x, 1.0),
    ):
        for rib_idx, sy in enumerate((-1.0, 1.0), start=1):
            frame.visual(
                Box((rib_depth, rib_width, rib_len)),
                origin=Origin(
                    xyz=(
                        rail_x + sx * (RAIL_W / 2.0 + rib_depth / 2.0 - 0.003),
                        sy * 0.017,
                        0.0,
                    )
                ),
                material=frame_color,
                name=f"{side_name}_stiffener_{rib_idx}",
            )

    rail_mid_x = OUTER_W / 2.0 - RAIL_W / 2.0
    sleeve_center_abs_x = OPENING_W / 2.0 + 0.5 * (SLEEVE_EMBED - SLEEVE_PROTRUSION)
    for idx, z in enumerate(z_positions, start=1):
        for side_name, sx in (("left", -1.0), ("right", 1.0)):
            frame.visual(
                Cylinder(radius=SLEEVE_OUTER_R, length=SLEEVE_LENGTH),
                origin=Origin(
                    xyz=(sx * sleeve_center_abs_x, AXIS_Y, z),
                    rpy=(0.0, 1.5707963267948966, 0.0),
                ),
                material=frame_color,
                name=f"{side_name}_sleeve_{idx}",
            )
            frame.visual(
                Box((SLEEVE_PAD_X, SLEEVE_PAD_Y, SLEEVE_PAD_Z)),
                origin=Origin(
                    xyz=(sx * (OPENING_W / 2.0 - 0.001), AXIS_Y, z),
                ),
                material=frame_color,
                name=f"{side_name}_pad_{idx}",
            )
    frame.inertial = Inertial.from_geometry(
        Box((OUTER_W, FRAME_D, OUTER_H)),
        mass=4.4,
        origin=Origin(),
    )

    blade_box = Box((2.0 * CONTACT_FACE_X, BLADE_CHORD, 0.012))
    for idx, z in enumerate(z_positions, start=1):
        blade = model.part(f"blade_{idx}")
        blade.visual(
            mesh_from_cadquery(blade_body_shape, f"shutter_blade_body_v5_{idx}"),
            origin=Origin(rpy=(BLADE_VISUAL_TILT, 0.0, 0.0)),
            material=blade_color,
            name="blade_body",
        )
        blade.visual(
            mesh_from_cadquery(blade_collar_shape, f"shutter_blade_collar_v5_{idx}"),
            origin=Origin(rpy=(BLADE_VISUAL_TILT, 0.0, 0.0)),
            material=blade_color,
            name="blade_collar",
        )
        blade.inertial = Inertial.from_geometry(
            blade_box,
            mass=0.16,
            origin=Origin(),
        )
        model.articulation(
            f"frame_to_blade_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(0.0, AXIS_Y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=1.8,
                lower=0.0,
                upper=1.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    blades = [object_model.get_part(f"blade_{idx}") for idx in range(1, BLADE_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"frame_to_blade_{idx}")
        for idx in range(1, BLADE_COUNT + 1)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for blade in blades:
        ctx.allow_overlap(
            blade,
            frame,
            elem_a="blade_collar",
            reason="blade thrust collars intentionally nest against the side-rail bearing sleeves to capture each revolute vane",
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

    axes_ok = all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in joints)
    limits_ok = all(
        joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper == 1.0
        for joint in joints
    )
    pitch_steps = [
        joints[idx + 1].origin.xyz[2] - joints[idx].origin.xyz[2]
        for idx in range(len(joints) - 1)
    ]
    pitch_ok = max(pitch_steps) - min(pitch_steps) < 1e-9

    ctx.check("blade_joint_axes_parallel", axes_ok, "all blade pivots should share the +X axis")
    ctx.check(
        "blade_joint_limits_consistent",
        limits_ok,
        "every blade should rotate through the same one-way shutter motion range",
    )
    ctx.check(
        "blade_pitch_consistent",
        pitch_ok,
        f"inconsistent blade pitch values: {pitch_steps}",
    )

    for idx, blade in enumerate(blades, start=1):
        ctx.expect_contact(frame, blade, name=f"blade_{idx}_captured_by_side_rails")
        ctx.expect_within(blade, frame, axes="y", margin=0.0, name=f"blade_{idx}_within_frame_depth")

    for lower, upper in zip(blades, blades[1:]):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.006,
            name=f"{lower.name}_to_{upper.name}_rest_gap",
        )

    with ctx.pose({joint: 1.0 for joint in joints}):
        for idx, blade in enumerate(blades, start=1):
            ctx.expect_contact(frame, blade, name=f"blade_{idx}_open_pose_still_captured")
            ctx.expect_within(
                blade,
                frame,
                axes="y",
                margin=0.0,
                name=f"blade_{idx}_open_pose_within_frame_depth",
            )
        for lower, upper in zip(blades, blades[1:]):
            ctx.expect_gap(
                upper,
                lower,
                axis="z",
                min_gap=0.002,
                name=f"{lower.name}_to_{upper.name}_open_gap",
            )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=32,
        name="blade_array_clear_across_motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
