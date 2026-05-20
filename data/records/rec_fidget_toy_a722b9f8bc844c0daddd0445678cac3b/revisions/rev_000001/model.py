from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.007
CENTRAL_HOLE_RADIUS = 0.0086
CAP_RADIUS = 0.012
CAP_THICKNESS = 0.0025
CAP_CLEARANCE = 0.0


def _slot_solid(radius: float, length: float, width: float, angle_deg: float) -> cq.Workplane:
    """A centered rounded slot extruded symmetrically about z=0."""
    angle = math.radians(angle_deg)
    return (
        cq.Workplane("XY")
        .center(radius * math.cos(angle), radius * math.sin(angle))
        .slot2D(length, width, angle=angle_deg)
        .extrude(PLATE_THICKNESS)
        .translate((0.0, 0.0, -PLATE_THICKNESS / 2.0))
    )


def _make_lobe_plate() -> cq.Workplane:
    """Continuous four-lobe spinner plate with a true central bearing hole."""
    plate = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(PLATE_THICKNESS)
        .translate((0.0, 0.0, -PLATE_THICKNESS / 2.0))
    )

    for angle in (0.0, 90.0, 180.0, 270.0):
        # Slim throat from the hub outwards, then a wider rounded rectangular
        # weighted tip. The solids intentionally overlap and are unioned into one
        # continuous molded body before the bearing hole is cut.
        plate = plate.union(_slot_solid(0.020, 0.046, 0.013, angle))
        plate = plate.union(_slot_solid(0.0345, 0.022, 0.023, angle))

    cutter = (
        cq.Workplane("XY")
        .circle(CENTRAL_HOLE_RADIUS)
        .extrude(PLATE_THICKNESS * 3.0)
        .translate((0.0, 0.0, -PLATE_THICKNESS * 1.5))
    )
    return plate.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quad_lobe_fidget_spinner")

    plastic = Material("satin_cobalt_plastic", rgba=(0.03, 0.22, 0.78, 1.0))
    steel = Material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark = Material("dark_cap_plastic", rgba=(0.035, 0.035, 0.04, 1.0))

    hub = model.part("hub")
    cap_z = PLATE_THICKNESS / 2.0 + CAP_CLEARANCE + CAP_THICKNESS / 2.0
    hub.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, cap_z)),
        material=dark,
        name="top_cap",
    )
    hub.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -cap_z)),
        material=dark,
        name="bottom_cap",
    )
    hub.visual(
        Cylinder(radius=0.0055, length=2.0 * cap_z + CAP_THICKNESS),
        origin=Origin(),
        material=steel,
        name="axle",
    )

    lobes = model.part("lobes")
    lobes.visual(
        mesh_from_cadquery(_make_lobe_plate(), "quad_lobe_plate", tolerance=0.00045),
        material=plastic,
        name="spinner_plate",
    )

    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        lobes.visual(
            Cylinder(radius=0.0072, length=PLATE_THICKNESS * 1.04),
            origin=Origin(xyz=(0.0345 * math.cos(angle), 0.0345 * math.sin(angle), 0.0)),
            material=steel,
            name=f"weight_{index}",
        )

    model.articulation(
        "hub_to_lobes",
        ArticulationType.CONTINUOUS,
        parent=hub,
        child=lobes,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=35.0),
        motion_properties=MotionProperties(damping=0.002, friction=0.001),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("hub")
    lobes = object_model.get_part("lobes")
    spin = object_model.get_articulation("hub_to_lobes")

    ctx.check(
        "single free spinning joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        hub,
        lobes,
        axis="z",
        positive_elem="top_cap",
        negative_elem="spinner_plate",
        max_gap=0.0001,
        max_penetration=0.000001,
        name="top cap seats on bearing plate",
    )
    ctx.expect_gap(
        lobes,
        hub,
        axis="z",
        positive_elem="spinner_plate",
        negative_elem="bottom_cap",
        max_gap=0.0001,
        max_penetration=0.000001,
        name="bottom cap seats on bearing plate",
    )
    ctx.expect_overlap(
        hub,
        lobes,
        axes="xy",
        elem_a="top_cap",
        elem_b="spinner_plate",
        min_overlap=0.010,
        name="cap covers central bearing hub",
    )

    rest_aabb = ctx.part_element_world_aabb(lobes, elem="weight_0")
    with ctx.pose({spin: math.pi / 4.0}):
        spun_aabb = ctx.part_element_world_aabb(lobes, elem="weight_0")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    rest_center = _aabb_center(rest_aabb)
    spun_center = _aabb_center(spun_aabb)
    ctx.check(
        "tip weight follows spin pose",
        rest_center is not None
        and spun_center is not None
        and spun_center[0] < rest_center[0] - 0.006
        and spun_center[1] > rest_center[1] + 0.018,
        details=f"rest={rest_center}, spun={spun_center}",
    )

    return ctx.report()


object_model = build_object_model()
