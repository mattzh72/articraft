from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


STEEL_DARK = "steel_dark"
STEEL_MID = "steel_mid"
ALUMINUM = "aluminum"
BLACK = "black"


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_z(
    radius: float,
    z0: float,
    z1: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(z1 - z0).translate((x, y, z0))


def _base_shape() -> cq.Workplane:
    clamp_body = _box((0.110, 0.075, 0.200), (-0.010, 0.0, 0.100))
    clamp_opening = _box((0.075, 0.055, 0.120), (0.025, 0.0, 0.100))
    clamp = clamp_body.cut(clamp_opening)

    jaw_pad = _box((0.030, 0.075, 0.024), (-0.032, 0.0, 0.188))
    rear_rib = _box((0.026, 0.050, 0.090), (-0.046, 0.0, 0.065))
    lower_gusset = _box((0.026, 0.040, 0.050), (-0.012, 0.0, 0.028))

    screw_boss = _cyl_z(0.015, -0.004, 0.016, x=0.022)
    screw_shaft = _cyl_z(0.009, -0.084, -0.004, x=0.022)
    screw_pad = _cyl_z(0.018, -0.092, -0.084, x=0.022)
    knob_cross = (
        cq.Workplane("XZ")
        .circle(0.0055)
        .extrude(0.074, both=True)
        .translate((0.022, 0.0, -0.022))
    )
    knob_hub = _cyl_z(0.012, -0.032, -0.012, x=0.022)

    post = _cyl_z(0.018, 0.200, 0.360, x=-0.005)
    collar = _cyl_z(0.028, 0.192, 0.214, x=-0.005)
    shoulder_disk = _cyl_z(0.036, 0.360, 0.370, x=-0.005)

    base = (
        clamp.union(jaw_pad)
        .union(rear_rib)
        .union(lower_gusset)
        .union(screw_boss)
        .union(screw_shaft)
        .union(screw_pad)
        .union(knob_cross)
        .union(knob_hub)
        .union(post)
        .union(collar)
        .union(shoulder_disk)
    )

    return base.edges("|Z").fillet(0.003)


def _arm_shape(
    *,
    base_hub_radius: float,
    base_hub_height: float,
    beam_length: float,
    beam_width: float,
    beam_height: float,
    beam_start_x: float,
    beam_center_z: float,
    end_x: float,
    end_radius: float,
    end_height: float,
) -> cq.Workplane:
    hub = _cyl_z(base_hub_radius, 0.0, base_hub_height)
    beam = _box(
        (beam_length, beam_width, beam_height),
        (beam_start_x + beam_length / 2.0, 0.0, beam_center_z),
    ).edges("|Z").fillet(min(0.007, beam_height * 0.4))
    end_hub = _cyl_z(end_radius, 0.0, end_height, x=end_x)
    top_rib = _box(
        (beam_length * 0.72, beam_width * 0.34, beam_height * 0.42),
        (beam_start_x + beam_length * 0.50, 0.0, beam_center_z + beam_height * 0.32),
    )
    underside = _box(
        (beam_length * 0.58, beam_width * 0.26, beam_height * 0.24),
        (beam_start_x + beam_length * 0.44, 0.0, beam_center_z - beam_height * 0.24),
    )

    arm = hub.union(beam).union(end_hub).union(top_rib).union(underside).combine()
    return arm.clean()


def _lower_arm_shape() -> cq.Workplane:
    return _arm_shape(
        base_hub_radius=0.038,
        base_hub_height=0.018,
        beam_length=0.302,
        beam_width=0.062,
        beam_height=0.022,
        beam_start_x=0.030,
        beam_center_z=0.018,
        end_x=0.345,
        end_radius=0.036,
        end_height=0.028,
    )


def _upper_arm_shape() -> cq.Workplane:
    return _arm_shape(
        base_hub_radius=0.036,
        base_hub_height=0.016,
        beam_length=0.286,
        beam_width=0.058,
        beam_height=0.020,
        beam_start_x=0.022,
        beam_center_z=0.017,
        end_x=0.325,
        end_radius=0.033,
        end_height=0.024,
    )


def _head_shape() -> cq.Workplane:
    hub = _cyl_z(0.031, 0.0, 0.016)
    body = _box((0.060, 0.044, 0.026), (0.036, 0.0, 0.021)).edges("|Z").fillet(0.004)
    neck = _box((0.040, 0.028, 0.024), (0.070, 0.0, 0.022)).edges("|Z").fillet(0.003)
    clevis_blank = _box((0.028, 0.064, 0.046), (0.097, 0.0, 0.026)).edges("|X").fillet(0.002)
    clevis_slot = _box((0.022, 0.038, 0.034), (0.101, 0.0, 0.026))
    clevis = clevis_blank.cut(clevis_slot)
    top_cap = _box((0.028, 0.040, 0.014), (0.046, 0.0, 0.035)).edges("|Z").fillet(0.003)

    return hub.union(body).union(neck).union(clevis).union(top_cap).combine().clean()


def _vesa_plate_shape() -> cq.Workplane:
    trunnion = cq.Workplane("XZ").circle(0.006).extrude(0.040, both=True)
    tilt_block = _box((0.020, 0.028, 0.040), (0.010, 0.0, 0.0)).edges("|X").fillet(0.002)
    support_arm = _box((0.028, 0.020, 0.034), (0.030, 0.0, 0.0)).edges("|X").fillet(0.002)
    backing = _box((0.026, 0.066, 0.066), (0.052, 0.0, 0.0)).edges("|X").fillet(0.002)
    plate_stand = _box((0.012, 0.050, 0.050), (0.060, 0.0, 0.0)).edges("|X").fillet(0.0015)
    plate = (
        cq.Workplane("YZ")
        .rect(0.118, 0.118)
        .extrude(0.005)
        .translate((0.060, 0.0, 0.0))
    )
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.05, -0.05), (0.05, -0.05), (-0.05, 0.05), (0.05, 0.05)])
        .hole(0.006)
    )

    return (
        trunnion.union(tilt_block)
        .union(support_arm)
        .union(backing)
        .union(plate_stand)
        .union(plate)
        .combine()
        .clean()
    )


def _joint_has_axis(joint, expected: tuple[float, float, float]) -> bool:
    return all(isclose(a, b, abs_tol=1e-9) for a, b in zip(joint.axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_monitor_arm")

    model.material(STEEL_DARK, rgba=(0.18, 0.19, 0.21, 1.0))
    model.material(STEEL_MID, rgba=(0.40, 0.43, 0.46, 1.0))
    model.material(ALUMINUM, rgba=(0.72, 0.74, 0.77, 1.0))
    model.material(BLACK, rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "monitor_arm_base"), material=STEEL_DARK, name="base_shell")

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_lower_arm_shape(), "monitor_arm_lower_arm"),
        material=ALUMINUM,
        name="lower_arm_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "monitor_arm_upper_arm"),
        material=ALUMINUM,
        name="upper_arm_shell",
    )

    head = model.part("head")
    head.visual(mesh_from_cadquery(_head_shape(), "monitor_arm_head"), material=BLACK, name="head_shell")

    vesa_plate = model.part("vesa_plate")
    vesa_plate.visual(
        mesh_from_cadquery(_vesa_plate_shape(), "monitor_arm_vesa_plate"),
        material=STEEL_MID,
        name="vesa_plate_shell",
    )

    model.articulation(
        "base_fold",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(-0.005, 0.0, 0.370)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=50.0, velocity=2.0),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.345, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.7, upper=2.7, effort=38.0, velocity=2.2),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(0.325, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.8, upper=1.8, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=vesa_plate,
        origin=Origin(xyz=(0.097, 0.0, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.50, effort=8.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("head")
    vesa_plate = object_model.get_part("vesa_plate")

    base_fold = object_model.get_articulation("base_fold")
    elbow_fold = object_model.get_articulation("elbow_fold")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

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
        head,
        vesa_plate,
        reason="captured clevis-and-trunnion tilt hinge is represented as a simplified interlocking assembly",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in ("base", "lower_arm", "upper_arm", "head", "vesa_plate"):
        ctx.check(f"part_present_{part_name}", object_model.get_part(part_name) is not None)

    ctx.expect_contact(lower_arm, base, name="lower_arm_supported_on_base")
    ctx.expect_contact(upper_arm, lower_arm, name="upper_arm_supported_on_lower_arm")
    ctx.expect_contact(head, upper_arm, name="head_supported_on_upper_arm")
    ctx.expect_contact(vesa_plate, head, name="vesa_plate_supported_on_head")

    ctx.check(
        "joint_axes_match_monitor_arm_mechanism",
        _joint_has_axis(base_fold, (0.0, 0.0, 1.0))
        and _joint_has_axis(elbow_fold, (0.0, 0.0, 1.0))
        and _joint_has_axis(head_swivel, (0.0, 0.0, 1.0))
        and _joint_has_axis(head_tilt, (0.0, 1.0, 0.0)),
        details=(
            f"axes were base={base_fold.axis}, elbow={elbow_fold.axis}, "
            f"swivel={head_swivel.axis}, tilt={head_tilt.axis}"
        ),
    )

    ctx.check(
        "motion_limits_span_realistic_ranges",
        (
            base_fold.motion_limits is not None
            and elbow_fold.motion_limits is not None
            and head_swivel.motion_limits is not None
            and head_tilt.motion_limits is not None
            and base_fold.motion_limits.upper is not None
            and base_fold.motion_limits.lower is not None
            and elbow_fold.motion_limits.upper is not None
            and elbow_fold.motion_limits.lower is not None
            and head_swivel.motion_limits.upper is not None
            and head_swivel.motion_limits.lower is not None
            and head_tilt.motion_limits.upper is not None
            and head_tilt.motion_limits.lower is not None
            and base_fold.motion_limits.upper - base_fold.motion_limits.lower > 4.5
            and elbow_fold.motion_limits.upper - elbow_fold.motion_limits.lower > 4.5
            and head_swivel.motion_limits.upper - head_swivel.motion_limits.lower > 3.0
            and head_tilt.motion_limits.lower < 0.0 < head_tilt.motion_limits.upper
        ),
    )

    straight_plate_pos = ctx.part_world_position(vesa_plate)
    with ctx.pose(base_fold=1.0, elbow_fold=-1.55):
        folded_plate_pos = ctx.part_world_position(vesa_plate)
    ctx.check(
        "arm_links_fold_to_change_reach",
        straight_plate_pos is not None
        and folded_plate_pos is not None
        and folded_plate_pos[0] < straight_plate_pos[0] - 0.16
        and abs(folded_plate_pos[1] - straight_plate_pos[1]) > 0.06,
        details=f"straight={straight_plate_pos}, folded={folded_plate_pos}",
    )

    with ctx.pose(head_tilt=0.40):
        tilted_up_aabb = ctx.part_world_aabb(vesa_plate)
    with ctx.pose(head_tilt=-0.40):
        tilted_down_aabb = ctx.part_world_aabb(vesa_plate)
    ctx.check(
        "vesa_plate_tilt_changes_pose",
        tilted_up_aabb is not None
        and tilted_down_aabb is not None
        and tilted_up_aabb[0][2] < tilted_down_aabb[0][2] - 0.005,
        details=f"tilt_up={tilted_up_aabb}, tilt_down={tilted_down_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
