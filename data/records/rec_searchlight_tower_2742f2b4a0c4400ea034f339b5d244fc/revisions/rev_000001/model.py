from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


STEEL = Material("weathered_dark_steel", rgba=(0.13, 0.15, 0.14, 1.0))
OLIVE = Material("faded_olive_enamel", rgba=(0.22, 0.30, 0.22, 1.0))
BLACK = Material("matte_black_housing", rgba=(0.015, 0.016, 0.014, 1.0))
BRASS = Material("aged_bronze_bearing", rgba=(0.56, 0.43, 0.24, 1.0))
BOLT = Material("rubbed_bolt_steel", rgba=(0.45, 0.46, 0.42, 1.0))
GLASS = Material("warm_searchlight_glass", rgba=(0.85, 0.93, 1.0, 0.72))
RUBBER = Material("black_rubber_gasket", rgba=(0.03, 0.03, 0.025, 1.0))


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate(
        (center[0], center[1], center[2] - length / 2.0)
    )


def _cq_cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _make_yoke_frame() -> cq.Workplane:
    """Rotating pan carriage with separated vertical pan and horizontal tilt bearings."""
    frame = _cq_cylinder_z(0.235, 0.070, (0.0, 0.0, 0.035))
    frame = frame.union(_cq_cylinder_z(0.165, 0.045, (0.0, 0.0, 0.0925)))
    frame = frame.union(_cq_box((0.56, 0.66, 0.060), (0.0, 0.0, 0.145)))

    # Two carried yoke cheeks; the tilt-axis holes are cut after the side blocks
    # are joined, so the spotlight trunnion is visibly supported without collision.
    for side in (-1.0, 1.0):
        frame = frame.union(_cq_box((0.145, 0.055, 0.500), (0.0, side * 0.280, 0.365)))
        frame = frame.union(_cq_cylinder_y(0.115, 0.095, (0.0, side * 0.280, 0.480)))
        # Practical welded/gusseted retrofit ribs where the old yoke cheeks meet
        # the new turntable adapter.
        frame = frame.union(_cq_box((0.040, 0.060, 0.250), (0.090, side * 0.245, 0.265)))
        frame = frame.union(_cq_box((0.040, 0.060, 0.250), (-0.090, side * 0.245, 0.265)))

    # The hole is intentionally a close bushing fit around the authored
    # trunnion shaft; the scoped overlap allowance in tests classifies the tiny
    # represented interference as the captured bearing interface.
    shaft_clearance = _cq_cylinder_y(0.038, 0.760, (0.0, 0.0, 0.480))
    return frame.cut(shaft_clearance)


def _add_base_bolts(part, z: float, radius: float, name_prefix: str) -> None:
    for i, (x, y) in enumerate(
        (
            (-radius, -radius),
            (-radius, radius),
            (radius, -radius),
            (radius, radius),
        )
    ):
        part.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(x, y, z)),
            material=BOLT,
            name=f"{name_prefix}_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_searchlight_tower")
    for material in (STEEL, OLIVE, BLACK, BRASS, BOLT, GLASS, RUBBER):
        model.materials.append(material)

    tower = model.part("tower")

    # Old-school foundation and mast: heavy flanges, a simple round column,
    # bolted access plates, and diagonal struts that make the tower read as a
    # reinforced retrofit rather than a bare pole.
    tower.visual(
        Box((0.82, 0.82, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=STEEL,
        name="skid_base",
    )
    tower.visual(
        Cylinder(radius=0.165, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=OLIVE,
        name="lower_pedestal",
    )
    tower.visual(
        Cylinder(radius=0.070, length=1.720),
        origin=Origin(xyz=(0.0, 0.0, 1.160)),
        material=OLIVE,
        name="main_mast",
    )

    for i, z in enumerate((0.54, 1.18, 1.88)):
        tower.visual(
            Cylinder(radius=0.092, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=BRASS if i == 2 else STEEL,
            name=f"mast_collar_{i}",
        )

    # Four bolted diagonal braces, embedded at their ends in the base and mast.
    brace_specs = (
        ((0.170, 0.0, 0.365), (0.0, -0.46, 0.0)),
        ((-0.170, 0.0, 0.365), (0.0, 0.46, 0.0)),
        ((0.0, 0.170, 0.365), (0.46, 0.0, 0.0)),
        ((0.0, -0.170, 0.365), (-0.46, 0.0, 0.0)),
    )
    for i, (xyz, rpy) in enumerate(brace_specs):
        tower.visual(
            Box((0.045, 0.060, 0.620)),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=STEEL,
            name=f"diagonal_brace_{i}",
        )

    tower.visual(
        Box((0.205, 0.270, 0.340)),
        origin=Origin(xyz=(0.145, 0.0, 0.935)),
        material=STEEL,
        name="service_box",
    )
    tower.visual(
        Box((0.020, 0.205, 0.250)),
        origin=Origin(xyz=(0.257, 0.0, 0.935)),
        material=OLIVE,
        name="service_hatch",
    )
    tower.visual(
        Cylinder(radius=0.012, length=0.065),
        origin=Origin(xyz=(0.273, 0.065, 0.935), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BOLT,
        name="hatch_handle",
    )
    tower.visual(
        Box((0.030, 0.018, 0.270)),
        origin=Origin(xyz=(0.275, -0.100, 0.935)),
        material=BRASS,
        name="hatch_hinge",
    )
    tower.visual(
        Cylinder(radius=0.020, length=0.880),
        origin=Origin(xyz=(0.065, -0.050, 1.430)),
        material=RUBBER,
        name="service_conduit",
    )

    tower.visual(
        Cylinder(radius=0.250, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 2.060)),
        material=STEEL,
        name="pan_stator",
    )
    tower.visual(
        Cylinder(radius=0.305, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 2.013)),
        material=BRASS,
        name="top_adapter",
    )
    _add_base_bolts(tower, 0.089, 0.305, "base_bolt")
    for i, angle in enumerate([k * math.tau / 8.0 for k in range(8)]):
        tower.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(
                xyz=(0.285 * math.cos(angle), 0.285 * math.sin(angle), 2.035)
            ),
            material=BOLT,
            name=f"top_bolt_{i}",
        )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        mesh_from_cadquery(_make_yoke_frame(), "pan_yoke_frame", tolerance=0.0008),
        material=OLIVE,
        name="yoke_frame",
    )
    for i, angle in enumerate([k * math.tau / 8.0 for k in range(8)]):
        pan_yoke.visual(
            Cylinder(radius=0.011, length=0.016),
            origin=Origin(
                xyz=(0.180 * math.cos(angle), 0.180 * math.sin(angle), 0.078)
            ),
            material=BOLT,
            name=f"rotor_bolt_{i}",
        )
    for side in (-1.0, 1.0):
        for i, (x, z) in enumerate(((-0.062, 0.430), (0.062, 0.430), (-0.062, 0.530), (0.062, 0.530))):
            pan_yoke.visual(
                Cylinder(radius=0.010, length=0.012),
                origin=Origin(
                    xyz=(x, side * 0.322, z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=BOLT,
                name=f"bearing_cap_bolt_{'p' if side > 0 else 'n'}_{i}",
            )

    head = model.part("searchlight_head")
    head.visual(
        Cylinder(radius=0.180, length=0.580),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK,
        name="lamp_barrel",
    )
    head.visual(
        Cylinder(radius=0.198, length=0.055),
        origin=Origin(xyz=(0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.158, length=0.014),
        origin=Origin(xyz=(0.424, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GLASS,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.162, length=0.045),
        origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRASS,
        name="trunnion_shaft",
    )
    for side in (-1.0, 1.0):
        head.visual(
            Cylinder(radius=0.074, length=0.040),
            origin=Origin(
                xyz=(0.0, side * 0.180, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=STEEL,
            name=f"trunnion_collar_{'p' if side > 0 else 'n'}",
        )
    for i, x in enumerate((-0.030, 0.095, 0.220)):
        head.visual(
            Box((0.060, 0.090, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.188)),
            material=STEEL,
            name=f"cooling_rib_{i}",
        )
    head.visual(
        Box((0.018, 0.120, 0.145)),
        origin=Origin(xyz=(-0.263, 0.0, -0.005)),
        material=OLIVE,
        name="rear_service_hatch",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 2.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.75, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.65, lower=-0.55, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_yoke = object_model.get_part("pan_yoke")
    head = object_model.get_part("searchlight_head")
    pan = object_model.get_articulation("pan_axis")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.allow_overlap(
        pan_yoke,
        head,
        elem_a="yoke_frame",
        elem_b="trunnion_shaft",
        reason="The tilt trunnion is intentionally captured as a close bearing fit inside the yoke blocks.",
    )

    ctx.expect_gap(
        pan_yoke,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="yoke_frame",
        negative_elem="pan_stator",
        name="turntable rotor seats on stator bearing",
    )
    ctx.expect_within(
        head,
        pan_yoke,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="yoke_frame",
        margin=0.004,
        name="trunnion runs through both yoke bearing blocks",
    )
    ctx.expect_overlap(
        head,
        pan_yoke,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="yoke_frame",
        min_overlap=0.40,
        name="trunnion remains captured across yoke width",
    )

    rest_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({tilt: 0.55}):
        raised_lens = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "positive tilt raises the searchlight beam",
        rest_lens is not None
        and raised_lens is not None
        and raised_lens[0][2] > rest_lens[0][2] + 0.12,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    rest_head = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({pan: 0.80}):
        panned_head = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "pan axis rotates the carried yoke and head",
        rest_head is not None
        and panned_head is not None
        and abs(panned_head[0][1] - rest_head[0][1]) > 0.25,
        details=f"rest={rest_head}, panned={panned_head}",
    )

    return ctx.report()


object_model = build_object_model()
