from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder helper: SDK cylinders are local-Z, this turns them along world Y."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_y_cylinder_origin(*xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_singleleaf_drawbridge")

    matte_concrete = model.material("matte_concrete", rgba=(0.56, 0.56, 0.53, 1.0))
    warm_concrete = model.material("warm_concrete", rgba=(0.68, 0.66, 0.61, 1.0))
    asphalt = model.material("fine_matte_asphalt", rgba=(0.09, 0.095, 0.095, 1.0))
    dark_seal = model.material("dark_compression_seal", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_steel = model.material("satin_burnished_steel", rgba=(0.53, 0.55, 0.56, 1.0))
    dark_steel = model.material("matte_dark_steel", rgba=(0.13, 0.14, 0.15, 1.0))
    graphite = model.material("graphite_paint", rgba=(0.23, 0.25, 0.27, 1.0))
    safety_line = model.material("muted_lane_marking", rgba=(0.82, 0.78, 0.58, 1.0))

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.265, tube=0.055, radial_segments=36, tubular_segments=72).rotate_x(
            math.pi / 2.0
        ),
        "satin_bearing_ring",
    )

    abutment = model.part("abutment")
    _add_box(abutment, "foundation_block", (1.60, 6.55, 0.98), (-0.94, 0.0, -0.80), matte_concrete)
    _add_box(abutment, "approach_slab", (3.25, 4.55, 0.26), (-2.31, 0.0, -0.205), warm_concrete)
    _add_box(abutment, "approach_nose", (0.62, 3.74, 0.24), (-0.39, 0.0, -0.195), warm_concrete)
    _add_box(abutment, "approach_asphalt", (3.22, 4.18, 0.035), (-2.315, 0.0, -0.0575), asphalt)
    _add_box(abutment, "fixed_seal", (0.030, 3.74, 0.045), (-0.115, 0.0, -0.0525), dark_seal)
    _add_box(abutment, "rear_expansion_line", (0.035, 4.20, 0.028), (-0.72, 0.0, -0.061), dark_seal)

    # Low, continuous side curbs and a cross tie make the abutment frame read as one built structure.
    for side_index, side in enumerate((-1.0, 1.0)):
        y = side * 2.42
        _add_box(abutment, f"approach_curb_{side_index}", (3.25, 0.18, 0.27), (-2.31, y, -0.055), warm_concrete)
        _add_box(abutment, f"side_pier_{side_index}", (0.74, 0.42, 1.22), (-0.15, side * 3.14, -0.53), matte_concrete)
        _add_box(abutment, f"bearing_plinth_{side_index}", (0.82, 0.60, 0.36), (0.0, side * 2.66, -0.485), dark_steel)
        _add_box(abutment, f"bearing_saddle_{side_index}", (0.66, 0.46, 0.22), (0.0, side * 2.66, -0.42), graphite)
        _add_box(abutment, f"bearing_web_{side_index}", (0.12, 0.48, 0.50), (-0.42, side * 2.66, -0.19), graphite)
        _add_box(abutment, f"bearing_web_{side_index}_outer", (0.12, 0.48, 0.50), (0.42, side * 2.66, -0.19), graphite)
        abutment.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, side * 2.66, 0.0)),
            material=satin_steel,
            name=f"bearing_ring_{side_index}",
        )
        _add_y_cylinder(
            abutment,
            f"bearing_cap_pin_{side_index}",
            0.026,
            0.66,
            (0.0, side * 2.66, 0.345),
            satin_steel,
        )
        for bolt_index, bx in enumerate((-0.26, 0.26)):
            for by_index, by in enumerate((-0.18, 0.18)):
                abutment.visual(
                    Cylinder(radius=0.038, length=0.045),
                    origin=Origin(xyz=(bx, side * 2.66 + side * by, -0.2825)),
                    material=satin_steel,
                    name=f"plinth_bolt_{side_index}_{bolt_index}_{by_index}",
                )

    _add_box(abutment, "rear_cross_tie", (0.28, 5.85, 0.46), (-0.91, 0.0, -0.42), matte_concrete)

    bridge_leaf = model.part("bridge_leaf")
    _add_box(bridge_leaf, "deck_slab", (11.92, 4.28, 0.22), (6.04, 0.0, -0.18), graphite)
    _add_box(bridge_leaf, "road_surface", (11.58, 3.70, 0.040), (6.14, 0.0, -0.049), asphalt)
    _add_box(bridge_leaf, "leaf_seal", (0.055, 4.10, 0.048), (0.085, 0.0, -0.043), dark_seal)
    _add_box(bridge_leaf, "tip_nose", (0.28, 4.26, 0.26), (12.08, 0.0, -0.17), graphite)

    for side_index, side in enumerate((-1.0, 1.0)):
        y = side * 2.08
        _add_box(bridge_leaf, f"side_girder_{side_index}", (11.92, 0.28, 0.68), (6.04, y, -0.08), dark_steel)
        _add_box(bridge_leaf, f"curb_cap_{side_index}", (11.55, 0.16, 0.20), (6.20, side * 2.04, 0.055), graphite)
        _add_box(bridge_leaf, f"satin_edge_trim_{side_index}", (11.70, 0.060, 0.070), (6.12, side * 2.25, 0.225), satin_steel)
        _add_box(bridge_leaf, f"hinge_cheek_{side_index}", (0.88, 0.34, 0.78), (0.32, y, -0.02), dark_steel)
        _add_y_cylinder(
            bridge_leaf,
            f"trunnion_hub_{side_index}",
            0.305,
            0.36,
            (0.0, side * 2.36, 0.0),
            satin_steel,
        )
        for post_index, px in enumerate((0.70, 2.10, 3.55, 5.00, 6.45, 7.90, 9.35, 10.80)):
            _add_box(
                bridge_leaf,
                f"rail_post_{side_index}_{post_index}",
                (0.075, 0.075, 0.55),
                (px, side * 2.11, 0.315),
                satin_steel,
            )
        _add_box(
            bridge_leaf,
            f"top_rail_{side_index}",
            (10.70, 0.060, 0.075),
            (5.82, side * 2.11, 0.615),
            satin_steel,
        )

    _add_y_cylinder(bridge_leaf, "trunnion_shaft", 0.065, 5.72, (0.0, 0.0, 0.0), satin_steel)
    for sleeve_index, side in enumerate((-1.0, 1.0)):
        _add_y_cylinder(
            bridge_leaf,
            f"bearing_sleeve_{sleeve_index}",
            0.180,
            0.16,
            (0.0, side * 2.66, 0.0),
            satin_steel,
        )
    _add_box(bridge_leaf, "hinge_transom", (0.38, 4.30, 0.48), (0.27, 0.0, -0.13), dark_steel)
    _add_box(bridge_leaf, "underside_center_rib", (11.30, 0.18, 0.24), (6.20, 0.0, -0.405), dark_steel)
    for rib_index, y in enumerate((-1.15, 1.15)):
        _add_box(bridge_leaf, f"underside_long_rib_{rib_index}", (11.20, 0.14, 0.22), (6.20, y, -0.400), dark_steel)
    for beam_index, x in enumerate((0.80, 2.45, 4.10, 5.75, 7.40, 9.05, 10.70)):
        _add_box(bridge_leaf, f"crossbeam_{beam_index}", (0.16, 4.78, 0.32), (x, 0.0, -0.355), dark_steel)
    for stripe_index, y in enumerate((-0.82, 0.82)):
        _add_box(bridge_leaf, f"lane_marking_{stripe_index}", (10.70, 0.040, 0.018), (6.30, y, -0.020), safety_line)
    for plate_index, x in enumerate((1.55, 3.25, 4.95, 6.65, 8.35, 10.05)):
        _add_box(bridge_leaf, f"deck_plate_break_{plate_index}", (0.028, 4.00, 0.020), (x, 0.0, -0.018), dark_seal)
    for bolt_index, x in enumerate((0.55, 1.55, 2.55, 3.55, 4.55, 5.55, 6.55, 7.55, 8.55, 9.55, 10.55)):
        for side_index, y in enumerate((-1.86, 1.86)):
            bridge_leaf.visual(
                Cylinder(radius=0.036, length=0.030),
                origin=Origin(xyz=(x, y, -0.032)),
                material=satin_steel,
                name=f"deck_bolt_{bolt_index}_{side_index}",
            )

    abutment.inertial = Inertial.from_geometry(
        Box((4.20, 6.60, 1.40)),
        mass=185000.0,
        origin=Origin(xyz=(-1.15, 0.0, -0.55)),
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((12.20, 5.10, 0.90)),
        mass=62000.0,
        origin=Origin(xyz=(6.05, 0.0, -0.10)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=abutment,
        child=bridge_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=700000.0, velocity=0.20, lower=0.0, upper=1.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    abutment = object_model.get_part("abutment")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            abutment,
            axis="x",
            positive_elem="deck_slab",
            negative_elem="approach_nose",
            min_gap=0.09,
            max_gap=0.22,
            name="closed roadway seam stays tight",
        )
        ctx.expect_overlap(
            leaf,
            abutment,
            axes="y",
            elem_a="deck_slab",
            elem_b="approach_nose",
            min_overlap=3.6,
            name="roadway widths align across seam",
        )
        for side_index in (0, 1):
            ctx.expect_within(
                leaf,
                abutment,
                axes="xz",
                inner_elem="trunnion_shaft",
                outer_elem=f"bearing_ring_{side_index}",
                margin=0.0,
                name=f"trunnion is centered in bearing ring {side_index}",
            )
            ctx.expect_overlap(
                leaf,
                abutment,
                axes="y",
                elem_a="trunnion_shaft",
                elem_b=f"bearing_ring_{side_index}",
                min_overlap=0.05,
                name=f"trunnion passes through bearing ring {side_index}",
            )

    rest_aabb = ctx.part_element_world_aabb(leaf, elem="tip_nose")
    with ctx.pose({hinge: 1.18}):
        raised_aabb = ctx.part_element_world_aabb(leaf, elem="tip_nose")
    ctx.check(
        "leaf raises clear at upper limit",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 10.0,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
