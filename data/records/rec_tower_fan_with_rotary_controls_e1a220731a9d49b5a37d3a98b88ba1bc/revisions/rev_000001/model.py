from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OSCILLATION_SWEEP = math.radians(42.0)


def _mesh(shape, name: str):
    return mesh_from_cadquery(shape, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def _build_base_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .circle(0.160)
        .extrude(0.018)
        .faces(">Z")
        .circle(0.125)
        .extrude(0.010)
    )

    motor_housing = (
        cq.Workplane("XY")
        .workplane(offset=0.028)
        .circle(0.088)
        .workplane(offset=0.032)
        .circle(0.067)
        .workplane(offset=0.026)
        .circle(0.052)
        .loft(combine=True)
    )

    pivot_collar = cq.Workplane("XY").workplane(offset=0.086).circle(0.042).extrude(0.018)

    return base_plate.union(motor_housing).union(pivot_collar)


def _build_column_shell() -> cq.Workplane:
    outer_shell = (
        cq.Workplane("XY")
        .ellipse(0.066, 0.092)
        .workplane(offset=0.400)
        .ellipse(0.062, 0.088)
        .workplane(offset=0.420)
        .ellipse(0.054, 0.076)
        .loft(combine=True)
    )
    top_cap = cq.Workplane("XY").workplane(offset=0.820).ellipse(0.054, 0.076).extrude(0.025)

    inner_void = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .ellipse(0.058, 0.084)
        .workplane(offset=0.396)
        .ellipse(0.054, 0.080)
        .workplane(offset=0.412)
        .ellipse(0.044, 0.064)
        .loft(combine=True)
    )

    front_opening = (
        cq.Workplane("XZ")
        .center(0.0, 0.420)
        .slot2D(0.660, 0.094, angle=90.0)
        .extrude(0.200, both=True)
    )

    return outer_shell.union(top_cap).cut(inner_void).cut(front_opening)


def _build_front_grille() -> cq.Workplane:
    grille = None
    slat_offsets = (-0.030, -0.022, -0.014, -0.006, 0.002, 0.010, 0.018, 0.026)
    for offset in slat_offsets:
        slat = (
            cq.Workplane("XZ")
            .workplane(offset=-0.071)
            .center(offset, 0.420)
            .slot2D(0.676, 0.005, angle=90.0)
            .extrude(0.008, both=True)
        )
        grille = slat if grille is None else grille.union(slat)
    for height in (0.082, 0.758):
        rail = (
            cq.Workplane("XZ")
            .workplane(offset=-0.071)
            .center(0.0, height)
            .slot2D(0.072, 0.006, angle=0.0)
            .extrude(0.008, both=True)
        )
        grille = rail if grille is None else grille.union(rail)
    return grille


def _build_control_cluster() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=0.836)
        .ellipse(0.040, 0.058)
        .extrude(0.016)
        .translate((0.0, -0.006, 0.0))
    )


def _build_rotor_shape() -> cq.Workplane:
    rotor = cq.Workplane("XY").circle(0.018).extrude(0.670)
    rotor = rotor.union(cq.Workplane("XY").circle(0.040).extrude(0.012))
    rotor = rotor.union(cq.Workplane("XY").workplane(offset=0.658).circle(0.040).extrude(0.012))
    rotor = rotor.union(cq.Workplane("XY").circle(0.006).extrude(0.690))

    for index in range(24):
        fin = (
            cq.Workplane("XY")
            .box(0.0018, 0.028, 0.646)
            .translate((0.0, 0.030, 0.335))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), index * 15.0)
        )
        rotor = rotor.union(fin)

    return rotor


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.67, 0.69, 0.72, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(_mesh(_build_base_shape(), "tower_fan_base"), material=body_white, name="base_shell")
    base.visual(
        Cylinder(radius=0.138, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=graphite,
        name="base_pad",
    )

    column = model.part("column")
    column.visual(
        _mesh(_build_column_shell(), "tower_fan_column_shell"),
        material=body_white,
        name="column_shell",
    )
    column.visual(
        _mesh(_build_front_grille(), "tower_fan_front_grille"),
        material=charcoal,
        name="front_grille",
    )
    column.visual(
        _mesh(_build_control_cluster(), "tower_fan_control_cluster"),
        material=graphite,
        name="control_cluster",
    )
    column.visual(
        Cylinder(radius=0.012, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=graphite,
        name="rotor_pedestal",
    )

    rotor = model.part("rotor")
    rotor.visual(
        _mesh(_build_rotor_shape(), "tower_fan_rotor"),
        material=charcoal,
        name="blower_rotor",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=knob_silver,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=knob_black,
        name="knob_stem",
    )
    speed_knob.visual(
        Box((0.004, 0.015, 0.004)),
        origin=Origin(xyz=(0.0, 0.019, 0.011)),
        material=knob_black,
        name="knob_pointer",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-OSCILLATION_SWEEP,
            upper=OSCILLATION_SWEEP,
        ),
    )

    model.articulation(
        "column_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=28.0),
    )

    model.articulation(
        "column_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -0.006, 0.864)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    rotor = object_model.get_part("rotor")
    speed_knob = object_model.get_part("speed_knob")

    oscillation = object_model.get_articulation("base_to_column")
    rotor_spin = object_model.get_articulation("column_to_rotor")
    knob_spin = object_model.get_articulation("column_to_speed_knob")

    ctx.expect_gap(
        column,
        base,
        axis="z",
        positive_elem="column_shell",
        negative_elem="base_shell",
        max_gap=0.002,
        max_penetration=0.0005,
        name="column seats on the base pivot collar",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="xy",
        elem_a="column_shell",
        elem_b="base_shell",
        min_overlap=0.080,
        name="column stays centered over the round base",
    )
    ctx.expect_within(
        rotor,
        column,
        axes="xy",
        inner_elem="blower_rotor",
        outer_elem="column_shell",
        margin=0.010,
        name="blower rotor stays inside the tower column footprint",
    )
    ctx.expect_overlap(
        speed_knob,
        column,
        axes="xy",
        elem_a="knob_body",
        elem_b="control_cluster",
        min_overlap=0.040,
        name="speed knob sits over the top control cluster",
    )
    ctx.expect_gap(
        speed_knob,
        column,
        axis="z",
        positive_elem="knob_stem",
        negative_elem="control_cluster",
        max_gap=0.0015,
        max_penetration=0.0,
        name="speed knob stem meets the control cluster",
    )

    base_aabb = ctx.part_world_aabb(base)
    column_aabb = ctx.part_world_aabb(column)
    knob_aabb = ctx.part_world_aabb(speed_knob)
    overall_top = max(aabb[1][2] for aabb in (base_aabb, column_aabb, knob_aabb) if aabb is not None)
    overall_bottom = base_aabb[0][2] if base_aabb is not None else None
    overall_height = None if overall_bottom is None else overall_top - overall_bottom
    ctx.check(
        "tower fan stands at domestic appliance scale",
        overall_height is not None and 0.90 <= overall_height <= 1.10,
        details=f"overall_height={overall_height}",
    )

    ctx.check(
        "fan uses the requested articulation mechanisms",
        (
            oscillation.articulation_type == ArticulationType.REVOLUTE
            and rotor_spin.articulation_type == ArticulationType.CONTINUOUS
            and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        ),
        details=(
            f"oscillation={oscillation.articulation_type}, "
            f"rotor_spin={rotor_spin.articulation_type}, knob_spin={knob_spin.articulation_type}"
        ),
    )

    oscillation_limits = oscillation.motion_limits
    ctx.check(
        "oscillation sweep is realistic",
        oscillation_limits is not None
        and oscillation_limits.lower is not None
        and oscillation_limits.upper is not None
        and oscillation_limits.lower <= -0.65
        and oscillation_limits.upper >= 0.65,
        details=f"limits={oscillation_limits}",
    )

    with ctx.pose({oscillation: 0.0}):
        rest_grille_center = _aabb_center(ctx.part_element_world_aabb(column, elem="front_grille"))
    with ctx.pose({oscillation: math.radians(35.0)}):
        swung_grille_center = _aabb_center(ctx.part_element_world_aabb(column, elem="front_grille"))

    ctx.check(
        "oscillation yaws the tower side to side",
        rest_grille_center is not None
        and swung_grille_center is not None
        and abs(swung_grille_center[0] - rest_grille_center[0]) > 0.030,
        details=f"rest={rest_grille_center}, swung={swung_grille_center}",
    )

    return ctx.report()


object_model = build_object_model()
