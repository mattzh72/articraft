from __future__ import annotations

import math

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


TOWER_HEIGHT = 70.0
HUB_HEIGHT_OFFSET = 2.8
ROTOR_JOINT_X = 7.3


def _make_tower_shape() -> cq.Workplane:
    """Concrete foundation plus a tapered tubular-looking tower."""

    foundation = cq.Workplane("XY").circle(4.2).extrude(0.65)
    tower_shell = (
        cq.Workplane("XY")
        .circle(2.15)
        .workplane(offset=TOWER_HEIGHT - 0.45)
        .circle(0.92)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.45))
    )
    base_flange = cq.Workplane("XY").circle(2.55).extrude(0.35).translate((0.0, 0.0, 0.38))
    top_flange = cq.Workplane("XY").circle(1.32).extrude(0.22).translate((0.0, 0.0, TOWER_HEIGHT - 0.22))
    return foundation.union(tower_shell).union(base_flange).union(top_flange)


def _make_nacelle_shape() -> cq.Workplane:
    """Streamlined yawing nacelle with a vertical yaw bearing and front shaft collar."""

    yaw_bearing = cq.Workplane("XY").circle(1.42).extrude(0.82)
    pedestal = cq.Workplane("XY").circle(1.05).extrude(1.35).translate((0.0, 0.0, 0.45))
    body = (
        cq.Workplane("XY")
        .box(12.0, 4.1, 3.35)
        .edges()
        .fillet(0.28)
        .translate((1.0, 0.0, 2.78))
    )
    roof_hatch = (
        cq.Workplane("XY")
        .box(2.5, 1.35, 0.22)
        .edges()
        .fillet(0.08)
        .translate((-1.6, 0.0, 4.47))
    )
    rear_service_cap = (
        cq.Workplane("YZ")
        .circle(1.28)
        .extrude(0.42)
        .translate((-5.35, 0.0, 2.78))
    )
    # This bearing ends exactly at the rotor joint plane, so the hub seats
    # against it without needing inter-part penetration.
    front_bearing = (
        cq.Workplane("YZ")
        .circle(0.62)
        .extrude(0.92)
        .translate((ROTOR_JOINT_X - 0.92, 0.0, HUB_HEIGHT_OFFSET))
    )
    return (
        yaw_bearing.union(pedestal)
        .union(body)
        .union(roof_hatch)
        .union(rear_service_cap)
        .union(front_bearing)
    )


def _make_hub_shape() -> cq.Workplane:
    """Hub shell aligned to the local +X main shaft axis."""

    rear_flange = cq.Workplane("YZ").circle(1.65).extrude(0.24)
    hub_body = cq.Workplane("YZ").circle(1.42).extrude(1.26).translate((0.10, 0.0, 0.0))
    nose = (
        cq.Workplane("YZ")
        .circle(1.38)
        .workplane(offset=1.25)
        .circle(0.18)
        .loft(combine=True)
        .translate((1.00, 0.0, 0.0))
    )
    return rear_flange.union(hub_body).union(nose)


def _make_blade_shape(angle_degrees: float) -> cq.Workplane:
    """One broad, tapered turbine blade, rooted in a cylindrical cuff."""

    # A blade is a thin, tapered airfoil-like solid.  The profile is drawn in
    # the YZ plane and extruded along X so the rotor shaft remains the local X
    # axis.  Root chord is intentionally broad and the tip narrow, matching a
    # utility-scale HAWT silhouette.
    blade = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-2.15, 1.18),
                (2.15, 1.18),
                (0.46, 48.0),
                (-0.52, 48.0),
            ]
        )
        .close()
        .extrude(0.50)
        .translate((0.38, 0.0, 0.0))
    )
    cuff = cq.Workplane("XY").circle(0.66).extrude(2.75).translate((0.66, 0.0, 0.10))
    blade_with_cuff = cuff.union(blade)
    return blade_with_cuff.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_degrees)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    tower_paint = model.material("tower_paint", rgba=(0.78, 0.80, 0.82, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.86, 0.88, 0.90, 1.0))
    blade_paint = model.material("blade_paint", rgba=(0.96, 0.96, 0.92, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.62, 0.64, 0.66, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_make_tower_shape(), "tower"),
        material=tower_paint,
        name="tapered_tower",
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_cadquery(_make_nacelle_shape(), "nacelle"),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    # A named front-bearing proxy lets tests prove that the rotor is seated on
    # the main shaft line rather than floating in front of the nacelle.
    nacelle.visual(
        mesh_from_cadquery(
            cq.Workplane("YZ")
            .circle(0.64)
            .extrude(0.03)
            .translate((ROTOR_JOINT_X - 0.03, 0.0, HUB_HEIGHT_OFFSET)),
            "front_bearing_face",
        ),
        material=hub_metal,
        name="front_bearing",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_cadquery(_make_hub_shape(), "hub_shell"),
        material=hub_metal,
        name="hub_shell",
    )
    for index, angle in enumerate((0.0, 120.0, 240.0)):
        rotor.visual(
            mesh_from_cadquery(_make_blade_shape(angle), f"blade_{index}"),
            material=blade_paint,
            name=f"blade_{index}",
        )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2_500_000.0, velocity=0.20, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(ROTOR_JOINT_X, 0.0, HUB_HEIGHT_OFFSET)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1_200_000.0, velocity=2.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    def _coord(vec, index: int) -> float:
        if hasattr(vec, "__getitem__"):
            return float(vec[index])
        return float((vec.x, vec.y, vec.z)[index])

    def _center_from_aabb(aabb, index: int) -> float:
        return 0.5 * (_coord(aabb[0], index) + _coord(aabb[1], index))

    def _size_from_aabb(aabb, index: int) -> float:
        return _coord(aabb[1], index) - _coord(aabb[0], index)

    ctx.check(
        "rotor has continuous shaft spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "nacelle yaws on tower axis",
        yaw.articulation_type == ArticulationType.REVOLUTE and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )

    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem="hub_shell",
        negative_elem="front_bearing",
        max_gap=0.025,
        max_penetration=0.0,
        name="hub seats on front bearing plane",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="yz",
        elem_a="hub_shell",
        elem_b="front_bearing",
        min_overlap=0.80,
        name="hub surrounds the main shaft bearing",
    )

    rotor_aabb = ctx.part_world_aabb(rotor)
    tower_aabb = ctx.part_world_aabb(tower)
    if rotor_aabb is not None and tower_aabb is not None:
        rotor_span = max(_size_from_aabb(rotor_aabb, 1), _size_from_aabb(rotor_aabb, 2))
        tower_width = max(_size_from_aabb(tower_aabb, 0), _size_from_aabb(tower_aabb, 1))
        ctx.check(
            "rotor dominates fixed support",
            rotor_span > 80.0 and rotor_span > 9.0 * tower_width,
            details=f"rotor_span={rotor_span:.3f}, tower_width={tower_width:.3f}",
        )
        ctx.check(
            "blade tips clear the ground",
            _coord(rotor_aabb[0], 2) > 25.0,
            details=f"rotor_min_z={_coord(rotor_aabb[0], 2):.3f}",
        )
    else:
        ctx.fail("rotor and tower dimensions measurable", "missing AABB")

    rest_blade_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_blade_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    if rest_blade_aabb is not None and spun_blade_aabb is not None:
        ctx.check(
            "spin carries a blade around the shaft",
            _center_from_aabb(rest_blade_aabb, 2) > _center_from_aabb(spun_blade_aabb, 2) + 15.0
            and abs(_center_from_aabb(spun_blade_aabb, 1)) > 15.0,
            details=(
                f"rest_center_yz=({_center_from_aabb(rest_blade_aabb, 1):.3f},"
                f" {_center_from_aabb(rest_blade_aabb, 2):.3f}), "
                f"spun_center_yz=({_center_from_aabb(spun_blade_aabb, 1):.3f},"
                f" {_center_from_aabb(spun_blade_aabb, 2):.3f})"
            ),
        )
    else:
        ctx.fail("blade pose dimensions measurable", "missing blade_0 AABB")

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw swings rotor around vertical tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and rest_rotor_pos[0] > 7.0
        and abs(rest_rotor_pos[1]) < 0.05
        and yawed_rotor_pos[1] > 7.0
        and abs(yawed_rotor_pos[0]) < 0.05,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    return ctx.report()


object_model = build_object_model()
