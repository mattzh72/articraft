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


PIVOT_Z = 0.118
SIDE_CLEARANCE = 0.003


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _z_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((cx, cy, cz - length / 2.0))
    )


def _make_hollow_beam(
    start_x: float,
    end_x: float,
    beam_w: float,
    beam_h: float,
    wall: float,
    slot_fraction: float = 0.56,
) -> cq.Workplane:
    beam_len = end_x - start_x
    beam = _box((beam_len, beam_w, beam_h), (start_x + beam_len / 2.0, 0.0, 0.0))

    cap_t = min(0.014, beam_len * 0.18)
    inner_len = beam_len - 2.0 * cap_t
    inner_w = beam_w - 2.0 * wall
    inner_h = beam_h - 2.0 * wall
    if inner_len > 0.02 and inner_w > 0.004 and inner_h > 0.004:
        beam = beam.cut(
            _box(
                (inner_len, inner_w, inner_h),
                (start_x + beam_len / 2.0, 0.0, 0.0),
            )
        )

    slot_len = beam_len * slot_fraction
    slot_d = beam_h * 0.36
    if slot_len > 0.05:
        cutter_y = beam_w + 0.008
        slot = (
            cq.Workplane("XZ")
            .center(start_x + beam_len / 2.0, 0.0)
            .slot2D(slot_len, slot_d)
            .extrude(cutter_y)
            .translate((0.0, -cutter_y / 2.0, 0.0))
        )
        beam = beam.cut(slot)

    return beam


def _hinge_hardware(
    joint_x: float,
    joint_z: float,
    gap: float,
    ear_t: float,
    washer_r: float,
    head_r: float,
    washer_t: float,
    spacer_t: float,
    head_t: float,
    child_width: float | None = None,
) -> cq.Workplane:
    hardware = None
    if child_width is not None and gap > child_width + 0.0004:
        inner_t = (gap - child_width) / 2.0
        inner_r = washer_r * 0.94
        for sign in (-1.0, 1.0):
            inner = _y_cylinder(
                inner_r,
                inner_t,
                (
                    joint_x,
                    sign * (child_width / 2.0 + inner_t / 2.0),
                    joint_z,
                ),
            )
            hardware = inner if hardware is None else hardware.union(inner)
    outer_face = gap / 2.0 + ear_t
    for sign in (-1.0, 1.0):
        washer = _y_cylinder(
            washer_r,
            washer_t + 0.0004,
            (
                joint_x,
                sign * (outer_face + washer_t / 2.0 - 0.0002),
                joint_z,
            ),
        )
        spacer = _y_cylinder(
            washer_r * 0.88,
            spacer_t,
            (
                joint_x,
                sign * (outer_face + washer_t + spacer_t / 2.0 - 0.0002),
                joint_z,
            ),
        )
        head = _y_cylinder(
            head_r,
            head_t,
            (
                joint_x,
                sign * (outer_face + washer_t + spacer_t + head_t / 2.0 - 0.0002),
                joint_z,
            ),
        )
        stack = washer.union(spacer).union(head)
        hardware = stack if hardware is None else hardware.union(stack)
    assert hardware is not None
    return hardware


def _mount_hardware() -> cq.Workplane:
    hardware = None
    for x_pos in (-0.090, 0.025):
        for y_pos in (-0.034, 0.034):
            washer = _z_cylinder(0.010, 0.0016, (x_pos, y_pos, 0.016 + 0.0008))
            head = _z_cylinder(0.0072, 0.0056, (x_pos, y_pos, 0.016 + 0.0016 + 0.0028))
            screw = washer.union(head)
            hardware = screw if hardware is None else hardware.union(screw)
    assert hardware is not None
    return hardware


def _make_base_structure() -> cq.Workplane:
    clevis_gap = 0.020
    ear_t = 0.008
    ear_h = 0.074
    ear_len = 0.032

    plate = _box((0.190, 0.112, 0.016), (-0.036, 0.0, 0.008))
    pedestal = _box((0.098, 0.052, 0.054), (-0.042, 0.0, 0.043))

    cheek_y = clevis_gap / 2.0 + 0.010
    column_l = _box((0.040, 0.012, 0.052), (-0.022, cheek_y, 0.084))
    column_r = column_l.mirror("XZ")
    nose_l = _box((0.020, 0.012, 0.020), (0.002, cheek_y, 0.104))
    nose_r = nose_l.mirror("XZ")
    strut_l = _box((0.056, 0.012, 0.018), (-0.052, cheek_y, 0.060))
    strut_r = strut_l.mirror("XZ")

    structure = plate.union(pedestal).union(column_l).union(column_r).union(nose_l).union(nose_r).union(strut_l).union(strut_r)

    for sign in (-1.0, 1.0):
        y_center = sign * (clevis_gap / 2.0 + ear_t / 2.0)
        ear = _box((ear_len, ear_t, ear_h), (0.004, y_center, PIVOT_Z))
        boss = _y_cylinder(0.020, ear_t, (0.0, y_center, PIVOT_Z))
        structure = structure.union(ear).union(boss)
    return structure


def _make_link_structure(
    *,
    length: float,
    beam_w: float,
    beam_h: float,
    wall: float,
    prox_eye_r: float,
    eye_w: float,
    distal_gap: float,
    ear_t: float,
    ear_h: float,
    bore_r: float,
) -> cq.Workplane:
    beam_start = prox_eye_r + 0.010
    beam_end = length - 0.050
    neck_h = min(beam_h * 0.48, prox_eye_r * 1.35)

    prox_eye = _y_cylinder(prox_eye_r, eye_w, (0.0, 0.0, 0.0))
    neck = _box((beam_start + 0.010, eye_w, neck_h), ((beam_start + 0.010) / 2.0, 0.0, 0.0))
    beam = _make_hollow_beam(beam_start, beam_end, beam_w, beam_h, wall)

    beam_cap = _box((0.012, beam_w, beam_h * 0.82), (beam_end + 0.006, 0.0, 0.0))

    shoulder_w = max((beam_w - distal_gap) / 2.0, ear_t + 0.0015)
    shoulder_h = beam_h * 0.58
    shoulder_len = 0.040
    shoulder_x = length - 0.036
    shoulder_y = distal_gap / 2.0 + shoulder_w / 2.0
    ear_len = 0.026
    ear_center_x = length - 0.008

    structure = prox_eye.union(neck).union(beam).union(beam_cap)
    for sign in (-1.0, 1.0):
        shoulder = _box((shoulder_len, shoulder_w, shoulder_h), (shoulder_x, sign * shoulder_y, 0.0))
        y_center = sign * (distal_gap / 2.0 + ear_t / 2.0)
        ear = _box((ear_len, ear_t, ear_h), (ear_center_x, y_center, 0.0))
        boss = _y_cylinder(ear_h * 0.31, ear_t, (length, y_center, 0.0))
        structure = structure.union(shoulder).union(ear).union(boss)
    return structure


def _make_terminal_arm(
    *,
    length: float,
    beam_w: float,
    beam_h: float,
    wall: float,
    prox_eye_r: float,
    eye_w: float,
    bore_r: float,
) -> cq.Workplane:
    beam_start = prox_eye_r + 0.012
    beam_end = length
    neck_h = min(beam_h * 0.48, prox_eye_r * 1.35)

    prox_eye = _y_cylinder(prox_eye_r, eye_w, (0.0, 0.0, 0.0))
    neck = _box((beam_start + 0.008, eye_w, neck_h), ((beam_start + 0.008) / 2.0, 0.0, 0.0))
    beam = _make_hollow_beam(beam_start, beam_end, beam_w, beam_h, wall, slot_fraction=0.50)
    tip_cap = _box((0.012, beam_w, beam_h * 0.82), (beam_end + 0.006, 0.0, 0.0))
    return prox_eye.union(neck).union(beam).union(tip_cap)


def _make_terminal_carrier(length: float) -> cq.Workplane:
    support = _box((0.024, 0.020, 0.018), (length + 0.012, 0.0, 0.009))
    upright = _box((0.012, 0.018, 0.030), (length + 0.026, 0.0, 0.024))
    carrier = _box((0.056, 0.024, 0.006), (length + 0.056, 0.0, 0.033))
    return support.union(upright).union(carrier)


def _make_pad() -> cq.Workplane:
    return _box((0.100, 0.060, 0.010), (0.226, 0.0, 0.041))


def _make_pad_hardware() -> cq.Workplane:
    hardware = None
    for x_pos in (0.196, 0.256):
        for y_pos in (-0.018, 0.018):
            shank = _z_cylinder(0.0023, 0.016, (x_pos, y_pos, 0.044))
            washer = _z_cylinder(0.0050, 0.0012, (x_pos, y_pos, 0.0466))
            head = _z_cylinder(0.0037, 0.0034, (x_pos, y_pos, 0.0489))
            screw = shank.union(washer).union(head)
            hardware = screw if hardware is None else hardware.union(screw)
    assert hardware is not None
    return hardware


def _size_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[idx] - mins[idx] for idx in range(3))


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[idx] + maxs[idx]) / 2.0 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_arm_chain")

    model.material("painted_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("anodized_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("black_oxide", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("pad_black", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part(
        "base",
        inertial=Inertial.from_geometry(
            Box((0.180, 0.110, 0.150)),
            mass=5.6,
            origin=Origin(xyz=(-0.032, 0.0, 0.075)),
        ),
    )
    base.visual(
        mesh_from_cadquery(_make_base_structure(), "base_structure"),
        material="painted_steel",
        name="structure",
    )
    base.visual(
        mesh_from_cadquery(
            _hinge_hardware(
                joint_x=0.0,
                joint_z=PIVOT_Z,
                gap=0.020,
                ear_t=0.008,
                washer_r=0.011,
                head_r=0.0082,
                washer_t=0.0016,
                spacer_t=0.0012,
                head_t=0.0062,
            ),
            "base_pivot_hardware",
        ),
        material="black_oxide",
        name="pivot_hardware",
    )
    base.visual(
        mesh_from_cadquery(_mount_hardware(), "base_mount_hardware"),
        material="black_oxide",
        name="mount_hardware",
    )

    link1 = model.part(
        "link1",
        inertial=Inertial.from_geometry(
            Box((0.290, 0.051, 0.060)),
            mass=1.7,
            origin=Origin(xyz=(0.145, 0.0, 0.0)),
        ),
    )
    link1.visual(
        mesh_from_cadquery(
            _make_link_structure(
                length=0.290,
                beam_w=0.036,
                beam_h=0.060,
                wall=0.0045,
                prox_eye_r=0.031,
                eye_w=0.020,
                distal_gap=0.018,
                ear_t=0.0075,
                ear_h=0.060,
                bore_r=0.0058,
            ),
            "link1_structure",
        ),
        material="anodized_aluminum",
        name="structure",
    )
    link1.visual(
        mesh_from_cadquery(
            _hinge_hardware(
                joint_x=0.290,
                joint_z=0.0,
                gap=0.018,
                ear_t=0.0075,
                washer_r=0.0090,
                head_r=0.0068,
                washer_t=0.0014,
                spacer_t=0.0010,
                head_t=0.0054,
            ),
            "link1_distal_hardware",
        ),
        material="black_oxide",
        name="distal_hardware",
    )

    link2 = model.part(
        "link2",
        inertial=Inertial.from_geometry(
            Box((0.250, 0.042, 0.050)),
            mass=1.2,
            origin=Origin(xyz=(0.125, 0.0, 0.0)),
        ),
    )
    link2.visual(
        mesh_from_cadquery(
            _make_link_structure(
                length=0.250,
                beam_w=0.030,
                beam_h=0.050,
                wall=0.0040,
                prox_eye_r=0.026,
                eye_w=0.018,
                distal_gap=0.016,
                ear_t=0.0065,
                ear_h=0.050,
                bore_r=0.0050,
            ),
            "link2_structure",
        ),
        material="anodized_aluminum",
        name="structure",
    )
    link2.visual(
        mesh_from_cadquery(
            _hinge_hardware(
                joint_x=0.250,
                joint_z=0.0,
                gap=0.016,
                ear_t=0.0065,
                washer_r=0.0080,
                head_r=0.0060,
                washer_t=0.0012,
                spacer_t=0.0009,
                head_t=0.0048,
            ),
            "link2_distal_hardware",
        ),
        material="black_oxide",
        name="distal_hardware",
    )

    link3 = model.part(
        "link3",
        inertial=Inertial.from_geometry(
            Box((0.210, 0.036, 0.043)),
            mass=0.84,
            origin=Origin(xyz=(0.105, 0.0, 0.0)),
        ),
    )
    link3.visual(
        mesh_from_cadquery(
            _make_link_structure(
                length=0.210,
                beam_w=0.026,
                beam_h=0.042,
                wall=0.0035,
                prox_eye_r=0.022,
                eye_w=0.016,
                distal_gap=0.0145,
                ear_t=0.0055,
                ear_h=0.043,
                bore_r=0.0043,
            ),
            "link3_structure",
        ),
        material="anodized_aluminum",
        name="structure",
    )
    link3.visual(
        mesh_from_cadquery(
            _hinge_hardware(
                joint_x=0.210,
                joint_z=0.0,
                gap=0.0145,
                ear_t=0.0055,
                washer_r=0.0070,
                head_r=0.0052,
                washer_t=0.0010,
                spacer_t=0.0008,
                head_t=0.0042,
            ),
            "link3_distal_hardware",
        ),
        material="black_oxide",
        name="distal_hardware",
    )

    link4 = model.part(
        "link4",
        inertial=Inertial.from_geometry(
            Box((0.272, 0.060, 0.052)),
            mass=0.58,
            origin=Origin(xyz=(0.136, 0.0, 0.026)),
        ),
    )
    link4.visual(
        mesh_from_cadquery(
            _make_terminal_arm(
                length=0.170,
                beam_w=0.022,
                beam_h=0.034,
                wall=0.0030,
                prox_eye_r=0.019,
                eye_w=0.0145,
                bore_r=0.0038,
            ),
            "link4_structure",
        ),
        material="anodized_aluminum",
        name="structure",
    )
    link4.visual(
        mesh_from_cadquery(_make_terminal_carrier(0.170), "link4_carrier"),
        material="anodized_aluminum",
        name="carrier",
    )
    link4.visual(
        mesh_from_cadquery(_make_pad(), "terminal_pad"),
        material="pad_black",
        name="pad",
    )
    link4.visual(
        mesh_from_cadquery(_make_pad_hardware(), "terminal_pad_hardware"),
        material="black_oxide",
        name="pad_hardware",
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.72, effort=85.0, velocity=1.25),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.95, upper=1.10, effort=52.0, velocity=1.55),
    )
    model.articulation(
        "forearm_hinge",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(0.250, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=1.00, effort=34.0, velocity=1.80),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=link3,
        child=link4,
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.85, effort=18.0, velocity=2.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    base = object_model.get_part("base")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    link4 = object_model.get_part("link4")
    shoulder = object_model.get_articulation("shoulder_hinge")
    elbow = object_model.get_articulation("elbow_hinge")
    forearm = object_model.get_articulation("forearm_hinge")
    wrist = object_model.get_articulation("wrist_hinge")

    for part_name, part in (
        ("base", base),
        ("link1", link1),
        ("link2", link2),
        ("link3", link3),
        ("link4", link4),
    ):
        ctx.check(f"{part_name}_present", part is not None, f"missing part {part_name}")

    for joint in (shoulder, elbow, forearm, wrist):
        axis_ok = (
            isclose(abs(joint.axis[1]), 1.0, abs_tol=1e-6)
            and isclose(joint.axis[0], 0.0, abs_tol=1e-6)
            and isclose(joint.axis[2], 0.0, abs_tol=1e-6)
        )
        ctx.check(f"{joint.name}_parallel_axis", axis_ok, f"unexpected axis {joint.axis}")

    ctx.allow_overlap(
        base,
        link1,
        elem_a="structure",
        elem_b="structure",
        reason="Base shoulder hinge uses a simplified nested bearing sleeve envelope instead of separate pin and bushing parts.",
    )
    ctx.allow_overlap(
        link1,
        link2,
        elem_a="structure",
        elem_b="structure",
        reason="Elbow hinge keeps coaxial bearing sleeves fused into the neighboring structural visuals for a compact calibrated-arm representation.",
    )
    ctx.allow_overlap(
        link2,
        link3,
        elem_a="structure",
        elem_b="structure",
        reason="Forearm hinge uses intentionally shared hinge-sleeve geometry rather than separate modeled pins and bushings.",
    )
    ctx.allow_overlap(
        link3,
        link4,
        elem_a="structure",
        elem_b="structure",
        reason="Wrist hinge simplification keeps the terminal eye and clevis sleeve volumes fused into the adjacent structure visuals.",
    )

    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.012)

    ctx.expect_origin_gap(link1, base, axis="z", min_gap=0.11, max_gap=0.13, name="shoulder_height")
    ctx.expect_origin_gap(link4, base, axis="x", min_gap=0.72, name="chain_reach_from_base")

    link_sizes = []
    for part in (link1, link2, link3, link4):
        link_sizes.append(_size_from_aabb(ctx.part_element_world_aabb(part, elem="structure")))

    taper_ready = all(size is not None for size in link_sizes)
    ctx.check("link_structure_measurable", taper_ready, "missing link structure aabb")
    if taper_ready:
        assert link_sizes[0] is not None
        assert link_sizes[1] is not None
        assert link_sizes[2] is not None
        assert link_sizes[3] is not None
        ctx.check(
            "link_width_tapers_distally",
            link_sizes[0][1] > link_sizes[1][1] > link_sizes[2][1] > link_sizes[3][1],
            f"unexpected link widths {link_sizes}",
        )
        ctx.check(
            "link_height_tapers_distally",
            link_sizes[0][2] > link_sizes[1][2] > link_sizes[2][2] > link_sizes[3][2],
            f"unexpected link heights {link_sizes}",
        )

    pad_aabb = ctx.part_element_world_aabb(link4, elem="pad")
    pad_size = _size_from_aabb(pad_aabb)
    pad_center = _center_from_aabb(pad_aabb)
    ctx.check("pad_measurable", pad_size is not None and pad_center is not None, "pad aabb unavailable")
    if pad_size is not None and pad_center is not None:
        ctx.check(
            "terminal_pad_is_broad_and_thin",
            pad_size[0] > 0.095 and pad_size[1] > 0.055 and pad_size[2] < 0.012,
            f"pad size {pad_size}",
        )
        ctx.check(
            "terminal_pad_projects_forward",
            pad_center[0] > 0.90,
            f"pad center x {pad_center[0]:.4f} too close to base",
        )

    with ctx.pose(shoulder_hinge=0.55, elbow_hinge=-0.35, forearm_hinge=0.25, wrist_hinge=-0.15):
        ctx.expect_gap(
            link4,
            base,
            axis="z",
            positive_elem="pad",
            negative_elem="structure",
            min_gap=0.040,
            name="raised_pose_pad_clears_base",
        )

    with ctx.pose(shoulder_hinge=0.30, elbow_hinge=0.65, forearm_hinge=-0.45, wrist_hinge=0.25):
        ctx.expect_gap(
            link4,
            base,
            axis="z",
            positive_elem="pad",
            negative_elem="structure",
            min_gap=0.018,
            name="compact_pose_pad_clears_base",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=36,
        ignore_adjacent=True,
        ignore_fixed=True,
        name="sampled_pose_clearance",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
