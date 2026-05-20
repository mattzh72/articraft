from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_HEIGHT = 0.138
LOWER_STAGE_HEIGHT = 0.084
MID_STAGE_HEIGHT = 0.106
TOP_STAGE_HEIGHT = 0.034


def _solid_cylinder(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _ring(outer_radius: float, inner_radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _bolt_positions(radius: float, count: int, start_deg: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.radians(start_deg) + math.tau * index / count),
            radius * math.sin(math.radians(start_deg) + math.tau * index / count),
        )
        for index in range(count)
    ]


def _cut_bolt_circle(
    body: cq.Workplane,
    *,
    bolt_radius: float,
    hole_radius: float,
    count: int,
    top_z: float,
    hole_depth: float,
    cbore_radius: float | None = None,
    cbore_depth: float | None = None,
    start_deg: float = 0.0,
) -> cq.Workplane:
    for x_pos, y_pos in _bolt_positions(bolt_radius, count, start_deg):
        body = body.cut(_solid_cylinder(hole_radius, hole_depth, top_z - hole_depth).translate((x_pos, y_pos, 0.0)))
        if cbore_radius is not None and cbore_depth is not None:
            body = body.cut(
                _solid_cylinder(cbore_radius, cbore_depth, top_z - cbore_depth).translate((x_pos, y_pos, 0.0))
            )
    return body


def _cut_annular_groove(
    body: cq.Workplane,
    *,
    inner_radius: float,
    outer_radius: float,
    top_z: float,
    depth: float,
) -> cq.Workplane:
    return body.cut(_ring(outer_radius, inner_radius, depth, top_z - depth))


def _cut_side_groove(
    body: cq.Workplane,
    *,
    inner_radius: float,
    outer_radius: float,
    z0: float,
    height: float,
) -> cq.Workplane:
    return body.cut(_ring(outer_radius, inner_radius, height, z0))


def _cut_split_slot(
    body: cq.Workplane,
    *,
    radius_center: float,
    slot_length: float,
    slot_width: float,
    z0: float,
    height: float,
) -> cq.Workplane:
    cutter = (
        cq.Workplane("XY")
        .box(slot_length, slot_width, height)
        .translate((radius_center, 0.0, z0 + 0.5 * height))
    )
    return body.cut(cutter)


def _build_base_housing_shape() -> cq.Workplane:
    body = _solid_cylinder(0.220, 0.028)
    body = body.union(_solid_cylinder(0.185, 0.080, 0.028))
    body = body.union(_solid_cylinder(0.195, 0.030, 0.108))
    body = body.union(_solid_cylinder(0.100, 0.018, 0.120))
    body = body.cut(_solid_cylinder(0.060, 0.090, 0.020))
    body = _cut_annular_groove(body, inner_radius=0.158, outer_radius=0.170, top_z=BASE_HEIGHT, depth=0.003)
    body = _cut_annular_groove(body, inner_radius=0.108, outer_radius=0.118, top_z=BASE_HEIGHT, depth=0.002)
    body = _cut_bolt_circle(
        body,
        bolt_radius=0.176,
        hole_radius=0.0045,
        count=12,
        top_z=BASE_HEIGHT,
        hole_depth=0.010,
        cbore_radius=0.0075,
        cbore_depth=0.004,
        start_deg=15.0,
    )
    return body


def _build_lower_stage_shape() -> cq.Workplane:
    body = _solid_cylinder(0.205, 0.030)
    body = body.union(_solid_cylinder(0.185, 0.012, 0.030))
    body = body.union(_solid_cylinder(0.105, 0.036, 0.042))
    body = body.union(_ring(0.120, 0.092, 0.006, 0.078))
    body = body.cut(_solid_cylinder(0.040, LOWER_STAGE_HEIGHT, 0.0))
    body = body.cut(_ring(0.150, 0.060, 0.014, 0.0))
    body = _cut_annular_groove(body, inner_radius=0.160, outer_radius=0.172, top_z=0.042, depth=0.002)
    body = _cut_annular_groove(body, inner_radius=0.096, outer_radius=0.108, top_z=LOWER_STAGE_HEIGHT, depth=0.0015)
    body = _cut_bolt_circle(
        body,
        bolt_radius=0.165,
        hole_radius=0.0045,
        count=12,
        top_z=0.042,
        hole_depth=0.010,
        cbore_radius=0.0075,
        cbore_depth=0.004,
    )
    body = _cut_bolt_circle(
        body,
        bolt_radius=0.100,
        hole_radius=0.0038,
        count=8,
        top_z=LOWER_STAGE_HEIGHT,
        hole_depth=0.012,
        cbore_radius=0.0065,
        cbore_depth=0.0035,
        start_deg=22.5,
    )
    body = _cut_split_slot(
        body,
        radius_center=0.105,
        slot_length=0.026,
        slot_width=0.006,
        z0=0.074,
        height=0.010,
    )
    return body


def _build_mid_stage_shape() -> cq.Workplane:
    body = _ring(0.110, 0.070, 0.012)
    body = body.union(_ring(0.125, 0.070, 0.072, 0.012))
    body = body.union(_ring(0.094, 0.058, 0.022, 0.084))
    body = _cut_side_groove(body, inner_radius=0.120, outer_radius=0.130, z0=0.030, height=0.004)
    body = _cut_side_groove(body, inner_radius=0.120, outer_radius=0.130, z0=0.064, height=0.004)
    body = _cut_annular_groove(body, inner_radius=0.070, outer_radius=0.080, top_z=MID_STAGE_HEIGHT, depth=0.002)
    body = _cut_bolt_circle(
        body,
        bolt_radius=0.078,
        hole_radius=0.0038,
        count=8,
        top_z=MID_STAGE_HEIGHT,
        hole_depth=0.012,
        cbore_radius=0.0065,
        cbore_depth=0.004,
        start_deg=22.5,
    )
    body = _cut_split_slot(
        body,
        radius_center=0.090,
        slot_length=0.020,
        slot_width=0.005,
        z0=0.094,
        height=0.010,
    )
    return body


def _build_top_stage_shape() -> cq.Workplane:
    body = _solid_cylinder(0.075, 0.016)
    body = body.union(_solid_cylinder(0.062, 0.010, 0.016))
    body = body.union(_solid_cylinder(0.020, 0.008, 0.026))
    body = body.cut(_ring(0.055, 0.026, 0.004, 0.0))
    body = _cut_side_groove(body, inner_radius=0.073, outer_radius=0.082, z0=0.006, height=0.004)
    body = _cut_annular_groove(body, inner_radius=0.024, outer_radius=0.032, top_z=0.026, depth=0.0015)
    body = _cut_bolt_circle(
        body,
        bolt_radius=0.044,
        hole_radius=0.0030,
        count=6,
        top_z=0.026,
        hole_depth=0.012,
        cbore_radius=0.0055,
        cbore_depth=0.004,
        start_deg=30.0,
    )
    body = _cut_bolt_circle(
        body,
        bolt_radius=0.030,
        hole_radius=0.0022,
        count=2,
        top_z=0.026,
        hole_depth=0.006,
        start_deg=0.0,
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_fixture_stack")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("lower_stage_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("mid_stage_graphite", rgba=(0.48, 0.50, 0.54, 1.0))
    model.material("top_tooling_aluminum", rgba=(0.82, 0.83, 0.85, 1.0))

    base_housing = model.part("base_housing")
    base_housing.visual(
        mesh_from_cadquery(_build_base_housing_shape(), "base_housing"),
        material="base_charcoal",
        name="housing",
    )
    base_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.220, length=BASE_HEIGHT),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BASE_HEIGHT)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_build_lower_stage_shape(), "lower_stage"),
        material="lower_stage_steel",
        name="turntable",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=LOWER_STAGE_HEIGHT),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * LOWER_STAGE_HEIGHT)),
    )

    mid_stage = model.part("mid_stage")
    mid_stage.visual(
        mesh_from_cadquery(_build_mid_stage_shape(), "mid_stage"),
        material="mid_stage_graphite",
        name="drum_ring",
    )
    mid_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=MID_STAGE_HEIGHT),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * MID_STAGE_HEIGHT)),
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        mesh_from_cadquery(_build_top_stage_shape(), "top_stage"),
        material="top_tooling_aluminum",
        name="platen",
    )
    top_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=TOP_STAGE_HEIGHT),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * TOP_STAGE_HEIGHT)),
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base_housing,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.9, lower=-3.02, upper=3.02),
    )
    model.articulation(
        "lower_to_mid_stage",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=mid_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-2.70, upper=2.70),
    )
    model.articulation(
        "mid_to_top_stage",
        ArticulationType.REVOLUTE,
        parent=mid_stage,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, MID_STAGE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_housing = object_model.get_part("base_housing")
    lower_stage = object_model.get_part("lower_stage")
    mid_stage = object_model.get_part("mid_stage")
    top_stage = object_model.get_part("top_stage")

    base_to_lower = object_model.get_articulation("base_to_lower_stage")
    lower_to_mid = object_model.get_articulation("lower_to_mid_stage")
    mid_to_top = object_model.get_articulation("mid_to_top_stage")

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

    joints = (base_to_lower, lower_to_mid, mid_to_top)
    limits_ok = all(
        joint.articulation_type == ArticulationType.REVOLUTE
        and joint.axis == (0.0, 0.0, 1.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower is not None
        and joint.motion_limits.upper is not None
        and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper
        for joint in joints
    )
    ctx.check(
        "three_coaxial_revolute_stages",
        limits_ok,
        "Lower, middle, and top stages must all be vertical revolute joints with bounded travel.",
    )

    ctx.expect_contact(lower_stage, base_housing, name="lower_stage_seated_on_base")
    ctx.expect_contact(mid_stage, lower_stage, name="mid_stage_seated_on_lower_stage")
    ctx.expect_contact(top_stage, mid_stage, name="top_stage_seated_on_mid_stage")

    ctx.expect_overlap(lower_stage, base_housing, axes="xy", min_overlap=0.38, name="lower_stage_broad_overlap")
    ctx.expect_within(mid_stage, lower_stage, axes="xy", margin=0.002, name="mid_stage_within_lower_envelope")
    ctx.expect_within(top_stage, mid_stage, axes="xy", margin=0.002, name="top_stage_within_mid_envelope")

    ctx.expect_origin_distance(lower_stage, base_housing, axes="xy", max_dist=1e-6, name="lower_stage_coaxial")
    ctx.expect_origin_distance(mid_stage, lower_stage, axes="xy", max_dist=1e-6, name="mid_stage_coaxial")
    ctx.expect_origin_distance(top_stage, mid_stage, axes="xy", max_dist=1e-6, name="top_stage_coaxial")

    with ctx.pose(
        {
            base_to_lower: 1.15,
            lower_to_mid: -0.85,
            mid_to_top: 0.95,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_offset_pose")
        ctx.expect_contact(lower_stage, base_housing, name="lower_stage_contact_in_offset_pose")
        ctx.expect_contact(mid_stage, lower_stage, name="mid_stage_contact_in_offset_pose")
        ctx.expect_contact(top_stage, mid_stage, name="top_stage_contact_in_offset_pose")
        ctx.expect_origin_distance(top_stage, base_housing, axes="xy", max_dist=1e-6, name="stack_remains_coaxial")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
