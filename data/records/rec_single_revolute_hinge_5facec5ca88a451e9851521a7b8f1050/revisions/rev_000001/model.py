from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _leaf_plate_mesh(
    name: str,
    *,
    width: float,
    length: float,
    thickness: float,
    hole_centers: tuple[tuple[float, float], ...],
    hole_radius: float,
):
    plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, length, 0.010, corner_segments=8),
        [_circle_profile(hole_radius, center=center, segments=32) for center in hole_centers],
        thickness,
        center=True,
    )
    plate.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(plate, name)


def _knuckle_mesh(name: str, *, length: float, outer_radius: float, inner_radius: float):
    # Lathe a rectangular ring section around the local Z pin axis so the
    # knuckle is a real open tube rather than a solid decorative cylinder.
    half = length * 0.5
    profile = [
        (outer_radius, -half),
        (outer_radius, half),
        (inner_radius, half),
        (inner_radius, -half),
        (outer_radius, -half),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mortised_equipment_hinge")

    blackened_steel = model.material("blackened_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.56, 0.58, 0.59, 1.0))
    polished_pin = model.material("polished_pin", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_screw = model.material("dark_screw", rgba=(0.05, 0.055, 0.06, 1.0))

    hinge_length = 0.320
    barrel_outer = 0.018
    barrel_inner = 0.0072
    pin_radius = 0.0054
    fixed_leaf_width = 0.120
    moving_leaf_width = 0.092
    fixed_thickness = 0.010
    moving_thickness = 0.006
    leaf_clearance = 0.0022
    tab_overlap = 0.0040

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        _leaf_plate_mesh(
            "fixed_leaf_plate",
            width=fixed_leaf_width,
            length=0.360,
            thickness=fixed_thickness,
            hole_centers=(
                (-0.030, -0.112),
                (0.030, -0.112),
                (-0.030, 0.112),
                (0.030, 0.112),
            ),
            hole_radius=0.0062,
        ),
        origin=Origin(xyz=(barrel_outer + leaf_clearance + fixed_leaf_width * 0.5, 0.0, 0.0)),
        material=blackened_steel,
        name="fixed_leaf_plate",
    )
    fixed_leaf.visual(
        Box((fixed_leaf_width + 0.018, 0.005, 0.330)),
        origin=Origin(
            xyz=(
                barrel_outer + leaf_clearance + fixed_leaf_width * 0.5 + 0.004,
                -0.0075,
                0.0,
            )
        ),
        material=blackened_steel,
        name="fixed_backing_pad",
    )

    fixed_knuckles = (
        ("bottom_knuckle", -0.128, 0.064),
        ("middle_knuckle", 0.000, 0.064),
        ("top_knuckle", 0.128, 0.064),
    )
    for name, z_center, length in fixed_knuckles:
        fixed_leaf.visual(
            _knuckle_mesh(f"fixed_{name}", length=length, outer_radius=barrel_outer, inner_radius=barrel_inner),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=blackened_steel,
            name=name,
        )
        fixed_leaf.visual(
            Box((0.012, fixed_thickness, length - 0.004)),
            origin=Origin(
                xyz=(
                    barrel_outer + 0.002,
                    0.0,
                    z_center,
                )
            ),
            material=blackened_steel,
            name=f"{name}_tab",
        )

    fixed_leaf.visual(
        Cylinder(radius=pin_radius, length=hinge_length + 0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=polished_pin,
        name="pin_shaft",
    )
    fixed_leaf.visual(
        Cylinder(radius=0.010, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, hinge_length * 0.5 + 0.0025)),
        material=polished_pin,
        name="pin_head",
    )
    fixed_leaf.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -hinge_length * 0.5 - 0.0022)),
        material=polished_pin,
        name="pin_peen",
    )
    fixed_plate_x = barrel_outer + leaf_clearance + fixed_leaf_width * 0.5
    for index, (x, z) in enumerate(((-0.030, -0.112), (0.030, -0.112), (-0.030, 0.112), (0.030, 0.112))):
        fixed_leaf.visual(
            Cylinder(radius=0.0095, length=0.0032),
            origin=Origin(
                xyz=(fixed_plate_x + x, 0.0062, z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_screw,
            name=f"fixed_screw_{index}",
        )
    fixed_leaf.inertial = Inertial.from_geometry(
        Box((0.160, 0.036, 0.370)),
        mass=1.4,
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        _leaf_plate_mesh(
            "moving_leaf_plate",
            width=moving_leaf_width,
            length=0.330,
            thickness=moving_thickness,
            hole_centers=(
                (0.018, -0.092),
                (-0.026, 0.0),
                (0.018, 0.092),
            ),
            hole_radius=0.0050,
        ),
        origin=Origin(xyz=(-(barrel_outer + leaf_clearance + moving_leaf_width * 0.5), 0.0, 0.0)),
        material=satin_steel,
        name="moving_leaf_plate",
    )
    moving_knuckles = (
        ("lower_knuckle", -0.064, 0.052),
        ("upper_knuckle", 0.064, 0.052),
    )
    for name, z_center, length in moving_knuckles:
        moving_leaf.visual(
            _knuckle_mesh(f"moving_{name}", length=length, outer_radius=barrel_outer, inner_radius=barrel_inner),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=satin_steel,
            name=name,
        )
        moving_leaf.visual(
            Box((0.012, moving_thickness, length - 0.004)),
            origin=Origin(
                xyz=(
                    -(barrel_outer + 0.002),
                    0.0,
                    z_center,
                )
            ),
            material=satin_steel,
            name=f"{name}_tab",
        )
    moving_plate_x = -(barrel_outer + leaf_clearance + moving_leaf_width * 0.5)
    for index, (x, z) in enumerate(((0.018, -0.092), (-0.026, 0.0), (0.018, 0.092))):
        moving_leaf.visual(
            Cylinder(radius=0.0078, length=0.0026),
            origin=Origin(
                xyz=(
                    moving_plate_x + x,
                    0.0042,
                    z,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_screw,
            name=f"moving_screw_{index}",
        )
    moving_leaf.inertial = Inertial.from_geometry(
        Box((0.130, 0.030, 0.340)),
        mass=0.65,
        origin=Origin(xyz=(-0.066, 0.0, 0.0)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=1.5708),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_leaf")
    moving = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check(
        "single pin-axis revolute",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 4) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={len(object_model.articulations)}, axis={hinge.axis}",
    )
    ctx.expect_gap(
        fixed,
        moving,
        axis="x",
        min_gap=0.015,
        positive_elem="fixed_leaf_plate",
        negative_elem="moving_leaf_plate",
        name="opposed leaves clear at the mortise line",
    )
    ctx.expect_gap(
        moving,
        fixed,
        axis="z",
        min_gap=0.004,
        max_gap=0.009,
        positive_elem="lower_knuckle",
        negative_elem="bottom_knuckle",
        name="lower moving knuckle clears bottom fixed knuckle",
    )
    ctx.expect_gap(
        fixed,
        moving,
        axis="z",
        min_gap=0.004,
        max_gap=0.009,
        positive_elem="middle_knuckle",
        negative_elem="lower_knuckle",
        name="middle fixed knuckle alternates above lower moving knuckle",
    )
    ctx.expect_overlap(
        fixed,
        moving,
        axes="xy",
        min_overlap=0.010,
        elem_a="pin_shaft",
        elem_b="lower_knuckle",
        name="moving knuckle wraps the central pin projection",
    )

    rest_aabb = ctx.part_element_world_aabb(moving, elem="moving_leaf_plate")
    with ctx.pose({hinge: 1.20}):
        swung_aabb = ctx.part_element_world_aabb(moving, elem="moving_leaf_plate")
        ctx.expect_gap(
            fixed,
            moving,
            axis="y",
            min_gap=0.010,
            positive_elem="fixed_leaf_plate",
            negative_elem="moving_leaf_plate",
            name="swung moving leaf clears the fixed support leaf",
        )
    if rest_aabb is not None and swung_aabb is not None:
        rest_y = (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
        swung_y = (swung_aabb[0][1] + swung_aabb[1][1]) * 0.5
    else:
        rest_y = None
        swung_y = None
    ctx.check(
        "moving leaf swings around the barrel",
        rest_y is not None and swung_y is not None and swung_y < rest_y - 0.045,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
