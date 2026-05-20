from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A softly radiused rectangular housing, authored in meters."""
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz)
    try:
        shape = shape.edges("|Z").fillet(radius)
        shape = shape.edges("#Z").fillet(min(radius * 0.35, sz * 0.18))
    except Exception:
        # Fillets are visual refinement only; keep the mass credible if OCC
        # cannot solve a very small edge case.
        shape = cq.Workplane("XY").box(sx, sy, sz)
    return shape


def _waffle_plate() -> cq.Workplane:
    """Square cast plate with raised waffle ribs on its local +Z face."""
    plate_size = 0.255
    base_t = 0.008
    rib_h = 0.006
    rib_w = 0.010
    rib_span = 0.218
    plate = cq.Workplane("XY").box(plate_size, plate_size, base_t)
    try:
        plate = plate.edges("|Z").fillet(0.014)
    except Exception:
        pass

    for p in (-0.084, -0.042, 0.0, 0.042, 0.084):
        plate = plate.union(
            cq.Workplane("XY")
            .box(rib_span, rib_w, rib_h)
            .translate((0.0, p, base_t / 2.0 + rib_h / 2.0))
        )
        plate = plate.union(
            cq.Workplane("XY")
            .box(rib_w, rib_span, rib_h)
            .translate((p, 0.0, base_t / 2.0 + rib_h / 2.0))
        )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_clamshell_waffle_maker")

    model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("warm_black", rgba=(0.035, 0.033, 0.030, 1.0))
    model.material("dark_plastic", rgba=(0.060, 0.060, 0.058, 1.0))
    model.material("cast_aluminum", rgba=(0.62, 0.62, 0.58, 1.0))
    model.material("brushed_steel", rgba=(0.77, 0.76, 0.72, 1.0))
    model.material("cream_marking", rgba=(0.90, 0.86, 0.72, 1.0))

    lower = model.part("lower_body")
    upper = model.part("upper_shell")
    dial = model.part("dial")

    lower.visual(
        mesh_from_cadquery(_rounded_box((0.360, 0.340, 0.060), 0.035), "lower_housing"),
        origin=Origin(xyz=(0.020, 0.0, 0.030)),
        material="matte_black",
        name="lower_housing",
    )
    lower.visual(
        mesh_from_cadquery(_waffle_plate(), "lower_waffle_plate"),
        origin=Origin(xyz=(0.020, 0.0, 0.064)),
        material="cast_aluminum",
        name="lower_waffle_plate",
    )

    # Rear hinge block and interleaved fixed knuckles keep the lid visually
    # carried by a real support rather than hovering above the body.
    lower.visual(
        Box((0.052, 0.300, 0.028)),
        origin=Origin(xyz=(-0.176, 0.0, 0.062)),
        material="dark_plastic",
        name="rear_block",
    )
    for idx, y in enumerate((-0.108, 0.108)):
        lower.visual(
            Box((0.045, 0.058, 0.046)),
            origin=Origin(xyz=(-0.168, y, 0.079)),
            material="dark_plastic",
            name=f"hinge_cheek_{idx}",
        )
        lower.visual(
            Cylinder(radius=0.014, length=0.056),
            origin=Origin(xyz=(-0.160, y, 0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="dark_plastic",
            name=f"lower_barrel_{idx}",
        )
    lower.visual(
        Cylinder(radius=0.0045, length=0.292),
        origin=Origin(xyz=(-0.160, 0.0, 0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="hinge_pin",
    )

    # Countertop feet give the appliance a believable small-kitchen scale.
    for idx, (x, y) in enumerate(((-0.105, -0.105), (-0.105, 0.105), (0.145, -0.105), (0.145, 0.105))):
        lower.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material="warm_black",
            name=f"foot_{idx}",
        )

    # Side thermostat graphics stay on the housing; the knob itself is a
    # separate continuous-control part mounted through its own shaft.
    lower.visual(
        Box((0.082, 0.003, 0.060)),
        origin=Origin(xyz=(0.075, 0.1715, 0.036)),
        material="warm_black",
        name="dial_panel",
    )
    for idx, (x, z, h) in enumerate(((0.040, 0.061, 0.014), (0.075, 0.067, 0.018), (0.110, 0.061, 0.014))):
        lower.visual(
            Box((0.004, 0.004, h)),
            origin=Origin(xyz=(x, 0.174, z)),
            material="cream_marking",
            name=f"dial_tick_{idx}",
        )

    upper.visual(
        mesh_from_cadquery(_rounded_box((0.325, 0.335, 0.060), 0.034), "upper_housing"),
        origin=Origin(xyz=(0.178, 0.0, 0.005)),
        material="matte_black",
        name="upper_housing",
    )
    upper.visual(
        mesh_from_cadquery(_waffle_plate(), "upper_waffle_plate"),
        origin=Origin(xyz=(0.170, 0.0, -0.017), rpy=(math.pi, 0.0, 0.0)),
        material="cast_aluminum",
        name="upper_waffle_plate",
    )
    upper.visual(
        Cylinder(radius=0.013, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="dark_plastic",
        name="upper_barrel",
    )
    upper.visual(
        Box((0.040, 0.090, 0.024)),
        origin=Origin(xyz=(0.032, 0.0, 0.000)),
        material="dark_plastic",
        name="hinge_web",
    )

    # Short insulated handle fixed to the front of the top shell.
    upper.visual(
        Box((0.060, 0.026, 0.026)),
        origin=Origin(xyz=(0.348, -0.060, -0.002)),
        material="dark_plastic",
        name="handle_mount_0",
    )
    upper.visual(
        Box((0.060, 0.026, 0.026)),
        origin=Origin(xyz=(0.348, 0.060, -0.002)),
        material="dark_plastic",
        name="handle_mount_1",
    )
    upper.visual(
        Cylinder(radius=0.018, length=0.175),
        origin=Origin(xyz=(0.388, 0.0, -0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="dark_plastic",
        name="front_handle",
    )

    dial.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.055,
                0.020,
                body_style="skirted",
                top_diameter=0.043,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                edge_radius=0.0015,
            ),
            "thermostat_dial",
        ),
        origin=Origin(xyz=(0.0, 0.036, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="dark_plastic",
        name="dial_cap",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(-0.160, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.25),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=lower,
        child=dial,
        origin=Origin(xyz=(0.075, 0.172, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_body")
    upper = object_model.get_part("upper_shell")
    dial = object_model.get_part("dial")
    rear_hinge = object_model.get_articulation("rear_hinge")
    dial_spin = object_model.get_articulation("dial_spin")

    ctx.allow_overlap(
        lower,
        upper,
        elem_a="hinge_pin",
        elem_b="upper_barrel",
        reason="The metal hinge pin is intentionally captured through the rotating upper hinge barrel.",
    )

    with ctx.pose({rear_hinge: 0.0}):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem="upper_waffle_plate",
            negative_elem="lower_waffle_plate",
            min_gap=0.001,
            max_gap=0.012,
            name="closed waffle plates have a narrow working gap",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="xz",
            elem_a="upper_barrel",
            elem_b="hinge_pin",
            min_overlap=0.006,
            name="upper barrel is centered on hinge pin",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="y",
            elem_a="upper_barrel",
            elem_b="hinge_pin",
            min_overlap=0.095,
            name="hinge pin spans through upper barrel",
        )

    closed_handle_aabb = ctx.part_element_world_aabb(upper, elem="front_handle")
    with ctx.pose({rear_hinge: 1.0}):
        raised_handle_aabb = ctx.part_element_world_aabb(upper, elem="front_handle")
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem="front_handle",
            negative_elem="lower_housing",
            min_gap=0.060,
            name="front handle lifts clear when lid opens",
        )
    ctx.check(
        "upper shell rises on rear hinge",
        closed_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[0][2] > closed_handle_aabb[0][2] + 0.050,
        details=f"closed_handle_aabb={closed_handle_aabb}, raised_handle_aabb={raised_handle_aabb}",
    )

    ctx.expect_gap(
        dial,
        lower,
        axis="y",
        positive_elem="dial_cap",
        negative_elem="dial_panel",
        min_gap=0.010,
        max_gap=0.035,
        name="dial cap stands proud of side wall",
    )
    with ctx.pose({dial_spin: math.pi}):
        ctx.expect_overlap(
            dial,
            lower,
            axes="xz",
            elem_a="dial_shaft",
            elem_b="dial_panel",
            min_overlap=0.010,
            name="rotating dial stays on its side shaft",
        )

    return ctx.report()


object_model = build_object_model()
