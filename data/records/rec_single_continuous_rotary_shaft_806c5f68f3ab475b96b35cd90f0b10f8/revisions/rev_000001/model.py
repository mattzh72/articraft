from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_labeling_platen")

    dark_housing = _material("dark_powder_coat", (0.10, 0.11, 0.12, 1.0))
    black_rubber = _material("black_rubber", (0.015, 0.015, 0.014, 1.0))
    brushed_aluminum = _material("brushed_aluminum", (0.72, 0.72, 0.68, 1.0))
    satin_steel = _material("satin_steel", (0.52, 0.54, 0.53, 1.0))
    safety_yellow = _material("yellow_index_marker", (1.0, 0.78, 0.08, 1.0))

    base = model.part("base")

    # Compact cast cylindrical base housing, with a broad foot and softened
    # shoulder.  The geometry is lathed as one continuous solid so it reads as
    # a manufactured housing rather than a plain cylinder.
    housing_profile = [
        (0.0, 0.000),
        (0.175, 0.000),
        (0.195, 0.014),
        (0.195, 0.045),
        (0.178, 0.060),
        (0.178, 0.145),
        (0.160, 0.175),
        (0.0, 0.175),
    ]
    base.visual(
        mesh_from_geometry(
            LatheGeometry(housing_profile, segments=80),
            "base_housing",
        ),
        material=dark_housing,
        name="housing",
    )
    base.visual(
        Cylinder(radius=0.086, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.197)),
        material=satin_steel,
        name="bearing_collar",
    )
    base.visual(
        Cylinder(radius=0.182, length=0.010),
        # The pad protrudes just below the casting and overlaps the cast foot
        # slightly, representing the compressed rubber underside.
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black_rubber,
        name="foot_pad",
    )

    platen = model.part("platen")

    # Child frame lies on the vertical centerline at the top of the fixed
    # bearing collar.  All rotating hardware is authored above that origin.
    platen.visual(
        Cylinder(radius=0.040, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=satin_steel,
        name="central_spindle",
    )
    rotating_hub_profile = [
        (0.0, 0.248),
        (0.050, 0.248),
        (0.066, 0.268),
        (0.066, 0.294),
        (0.116, 0.312),
        (0.116, 0.326),
        (0.0, 0.326),
    ]
    platen.visual(
        mesh_from_geometry(
            LatheGeometry(rotating_hub_profile, segments=72),
            "rotating_hub",
        ),
        material=brushed_aluminum,
        name="hub",
    )
    platen.visual(
        Cylinder(radius=0.430, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=brushed_aluminum,
        name="top_plate",
    )
    platen.visual(
        Cylinder(radius=0.385, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.364)),
        material=black_rubber,
        name="work_mat",
    )
    platen.visual(
        mesh_from_geometry(TorusGeometry(radius=0.415, tube=0.014), "raised_outer_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=brushed_aluminum,
        name="outer_rim",
    )
    platen.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.305, 0.0, 0.381)),
        material=safety_yellow,
        name="index_pin",
    )
    platen.visual(
        Box((0.250, 0.014, 0.003)),
        origin=Origin(xyz=(0.147, 0.0, 0.369)),
        material=safety_yellow,
        name="index_line",
    )

    model.articulation(
        "base_to_platen",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, 0.219)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    platen = object_model.get_part("platen")
    spin = object_model.get_articulation("base_to_platen")

    def _size(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])

    def _center(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)

    ctx.check(
        "one continuous vertical spin joint",
        len(object_model.articulations) == 1
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"joint_type={spin.articulation_type}, axis={spin.axis}",
    )

    top_aabb = ctx.part_element_world_aabb(platen, elem="top_plate")
    housing_aabb = ctx.part_element_world_aabb(base, elem="housing")
    top_size = _size(top_aabb)
    housing_size = _size(housing_aabb)
    ctx.check(
        "top plate dominates silhouette",
        top_size is not None
        and housing_size is not None
        and top_size[0] > 2.0 * housing_size[0]
        and top_size[1] > 2.0 * housing_size[1],
        details=f"top_size={top_size}, housing_size={housing_size}",
    )

    ctx.expect_contact(
        base,
        platen,
        elem_a="bearing_collar",
        elem_b="central_spindle",
        contact_tol=0.001,
        name="spindle is seated on bearing collar",
    )
    ctx.expect_overlap(
        platen,
        base,
        axes="xy",
        elem_a="central_spindle",
        elem_b="bearing_collar",
        min_overlap=0.075,
        name="spindle is centered in collar footprint",
    )

    with ctx.pose({spin: 0.0}):
        pin0 = ctx.part_element_world_aabb(platen, elem="index_pin")
    with ctx.pose({spin: math.pi / 2.0}):
        pin90 = ctx.part_element_world_aabb(platen, elem="index_pin")

    c0 = _center(pin0)
    c90 = _center(pin90)
    ctx.check(
        "top plate and spindle spin about centerline",
        c0 is not None
        and c90 is not None
        and c0[0] > 0.28
        and abs(c0[1]) < 0.025
        and c90[1] > 0.28
        and abs(c90[0]) < 0.025,
        details=f"index pin centers: q0={c0}, q90={c90}",
    )

    return ctx.report()


object_model = build_object_model()
