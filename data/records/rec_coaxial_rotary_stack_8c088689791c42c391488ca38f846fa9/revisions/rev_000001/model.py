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


AXIS_LIMIT = 2.0 * math.pi
BASE_TO_PLATTER_Z = 0.046
PLATTER_TO_RING_Z = 0.024
RING_TO_CUP_Z = 0.042


def _radial_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + (2.0 * math.pi * i / count)),
            radius * math.sin(phase + (2.0 * math.pi * i / count)),
        )
        for i in range(count)
    ]


def _add_cap_screws(
    body: cq.Workplane,
    *,
    points: list[tuple[float, float]],
    base_z: float,
    head_d: float,
    head_h: float,
    shank_d: float,
    sink: float,
    recess_d: float,
    recess_h: float,
) -> cq.Workplane:
    heads = (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(head_d / 2.0)
        .extrude(head_h)
        .translate((0.0, 0.0, base_z))
    )
    shanks = (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(shank_d / 2.0)
        .extrude(sink)
        .translate((0.0, 0.0, base_z - sink))
    )
    recesses = (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(recess_d / 2.0)
        .extrude(recess_h + 0.0006)
        .translate((0.0, 0.0, base_z + head_h - recess_h))
    )
    return body.union(shanks).union(heads).cut(recesses)


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _build_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.170).extrude(0.014)
    base = base.union(cq.Workplane("XY").circle(0.154).extrude(0.018).translate((0.0, 0.0, 0.014)))
    base = base.union(cq.Workplane("XY").circle(0.140).extrude(0.010).translate((0.0, 0.0, 0.032)))
    base = base.union(cq.Workplane("XY").circle(0.122).extrude(0.004).translate((0.0, 0.0, 0.042)))

    foot_relief = (
        cq.Workplane("XY")
        .circle(0.152)
        .circle(0.078)
        .extrude(0.003)
        .translate((0.0, 0.0, 0.011))
    )
    cap_split = (
        cq.Workplane("XY")
        .circle(0.149)
        .circle(0.126)
        .extrude(0.0016)
        .translate((0.0, 0.0, 0.031))
    )
    base = base.cut(foot_relief)
    base = base.cut(cap_split)

    base = _add_cap_screws(
        base,
        points=_radial_points(0.145, 8, phase=math.pi / 8.0),
        base_z=0.031,
        head_d=0.011,
        head_h=0.0035,
        shank_d=0.005,
        sink=0.004,
        recess_d=0.0046,
        recess_h=0.0018,
    )
    return base


def _build_platter_shape() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(0.153).extrude(0.010)
    platter = platter.union(cq.Workplane("XY").circle(0.129).extrude(0.010).translate((0.0, 0.0, 0.010)))
    platter = platter.union(cq.Workplane("XY").circle(0.092).extrude(0.004).translate((0.0, 0.0, 0.020)))

    underside_outer_relief = cq.Workplane("XY").circle(0.153).circle(0.127).extrude(0.006)
    through_bore = cq.Workplane("XY").circle(0.036).extrude(0.032)
    top_groove = (
        cq.Workplane("XY")
        .circle(0.141)
        .circle(0.102)
        .extrude(0.0015)
        .translate((0.0, 0.0, 0.0185))
    )

    platter = platter.cut(underside_outer_relief)
    platter = platter.cut(through_bore)
    platter = platter.cut(top_groove)
    platter = _add_cap_screws(
        platter,
        points=_radial_points(0.123, 8, phase=math.pi / 8.0),
        base_z=0.020,
        head_d=0.010,
        head_h=0.0035,
        shank_d=0.0048,
        sink=0.004,
        recess_d=0.0042,
        recess_h=0.0016,
    )
    return platter


def _build_ring_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.106).extrude(0.018)
    ring = ring.union(cq.Workplane("XY").circle(0.096).extrude(0.018).translate((0.0, 0.0, 0.018)))
    ring = ring.union(cq.Workplane("XY").circle(0.082).extrude(0.006).translate((0.0, 0.0, 0.036)))

    center_bore = cq.Workplane("XY").circle(0.038).extrude(0.050)
    underside_relief = cq.Workplane("XY").circle(0.106).circle(0.090).extrude(0.008)
    top_pocket = (
        cq.Workplane("XY")
        .circle(0.073)
        .circle(0.044)
        .extrude(0.006)
        .translate((0.0, 0.0, 0.036))
    )
    top_groove = (
        cq.Workplane("XY")
        .circle(0.101)
        .circle(0.086)
        .extrude(0.0015)
        .translate((0.0, 0.0, 0.034))
    )

    ring = ring.cut(center_bore)
    ring = ring.cut(underside_relief)
    ring = ring.cut(top_pocket)
    ring = ring.cut(top_groove)
    ring = _add_cap_screws(
        ring,
        points=_radial_points(0.093, 6, phase=math.pi / 6.0),
        base_z=0.036,
        head_d=0.009,
        head_h=0.004,
        shank_d=0.0045,
        sink=0.004,
        recess_d=0.0038,
        recess_h=0.0015,
    )
    return ring


def _build_cup_shape() -> cq.Workplane:
    cup = cq.Workplane("XY").circle(0.078).extrude(0.008)
    cup = cup.union(cq.Workplane("XY").circle(0.058).extrude(0.032).translate((0.0, 0.0, 0.008)))
    cup = cup.union(cq.Workplane("XY").circle(0.064).extrude(0.008).translate((0.0, 0.0, 0.040)))

    underside_relief = cq.Workplane("XY").circle(0.032).extrude(0.004)
    inner_cavity = cq.Workplane("XY").circle(0.038).extrude(0.030).translate((0.0, 0.0, 0.012))
    rim_groove = (
        cq.Workplane("XY")
        .circle(0.056)
        .circle(0.044)
        .extrude(0.0015)
        .translate((0.0, 0.0, 0.045))
    )

    cup = cup.cut(underside_relief)
    cup = cup.cut(inner_cavity)
    cup = cup.cut(rim_groove)
    cup = _add_cap_screws(
        cup,
        points=_radial_points(0.048, 4, phase=math.pi / 4.0),
        base_z=0.040,
        head_d=0.008,
        head_h=0.004,
        shank_d=0.004,
        sink=0.003,
        recess_d=0.0032,
        recess_h=0.0013,
    )
    return cup


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_ring_turntable")

    model.material("cast_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("gunmetal", rgba=(0.36, 0.38, 0.41, 1.0))
    model.material("fixture_gray", rgba=(0.62, 0.64, 0.68, 1.0))

    base = model.part("base_housing")
    _add_mesh_visual(base, _build_base_shape(), "nested_turntable_base", "cast_dark", "base_shell")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.046),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    platter = model.part("low_platter")
    _add_mesh_visual(platter, _build_platter_shape(), "nested_turntable_platter", "machined_steel", "platter_shell")
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.153, length=0.026),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    ring = model.part("intermediate_ring")
    _add_mesh_visual(ring, _build_ring_shape(), "nested_turntable_ring", "gunmetal", "ring_shell")
    ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.106, length=0.042),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    cup = model.part("fixture_cup")
    _add_mesh_visual(cup, _build_cup_shape(), "nested_turntable_cup", "fixture_gray", "cup_shell")
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.078, length=0.050),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_PLATTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.5,
            lower=-AXIS_LIMIT,
            upper=AXIS_LIMIT,
        ),
    )
    model.articulation(
        "platter_to_ring",
        ArticulationType.REVOLUTE,
        parent=platter,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_TO_RING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=-AXIS_LIMIT,
            upper=AXIS_LIMIT,
        ),
    )
    model.articulation(
        "ring_to_cup",
        ArticulationType.REVOLUTE,
        parent=ring,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, RING_TO_CUP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.2,
            lower=-AXIS_LIMIT,
            upper=AXIS_LIMIT,
        ),
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
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    base = object_model.get_part("base_housing")
    platter = object_model.get_part("low_platter")
    ring = object_model.get_part("intermediate_ring")
    cup = object_model.get_part("fixture_cup")

    base_to_platter = object_model.get_articulation("base_to_platter")
    platter_to_ring = object_model.get_articulation("platter_to_ring")
    ring_to_cup = object_model.get_articulation("ring_to_cup")

    joints = (base_to_platter, platter_to_ring, ring_to_cup)
    ctx.check(
        "three_revolute_stages_share_vertical_axis",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints)
        and all(tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0) for joint in joints)
        and all(
            abs(joint.origin.xyz[0]) < 1e-9 and abs(joint.origin.xyz[1]) < 1e-9
            for joint in joints
        ),
        "All three moving stages should be revolute joints aligned to the common vertical centerline.",
    )

    ctx.expect_contact(base, platter, name="base_supports_platter")
    ctx.expect_contact(platter, ring, name="platter_supports_ring")
    ctx.expect_contact(ring, cup, name="ring_supports_cup")

    ctx.expect_within(ring, platter, axes="xy", margin=0.0, name="ring_stays_inside_platter_footprint")
    ctx.expect_within(cup, ring, axes="xy", margin=0.0, name="cup_stays_inside_ring_footprint")

    ctx.expect_origin_distance(base, platter, axes="xy", max_dist=1e-6, name="platter_is_coaxial_with_base")
    ctx.expect_origin_distance(base, ring, axes="xy", max_dist=1e-6, name="ring_is_coaxial_with_base")
    ctx.expect_origin_distance(base, cup, axes="xy", max_dist=1e-6, name="cup_is_coaxial_with_base")

    with ctx.pose(
        {
            base_to_platter: 0.95,
            platter_to_ring: -1.30,
            ring_to_cup: 1.70,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_offset_pose")
        ctx.expect_contact(base, platter, name="base_supports_platter_in_offset_pose")
        ctx.expect_contact(platter, ring, name="platter_supports_ring_in_offset_pose")
        ctx.expect_contact(ring, cup, name="ring_supports_cup_in_offset_pose")
        ctx.expect_origin_distance(base, cup, axes="xy", max_dist=1e-6, name="offset_pose_keeps_common_centerline")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
