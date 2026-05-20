from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HINGE_LENGTH = 0.360
LEAF_WIDTH = 0.180
LEAF_HEIGHT = 0.340
LEAF_THICKNESS = 0.008
LEAF_Y_OFFSET = 0.010
BARREL_RADIUS = 0.025
KNUCKLE_GAP = 0.0
KNUCKLE_LENGTH = (HINGE_LENGTH - 4.0 * KNUCKLE_GAP) / 5.0
FIXED_PLATE_CENTER_X = -0.122
MOVING_PLATE_CENTER_X = 0.122
FIXED_PLATE_INNER_X = FIXED_PLATE_CENTER_X + LEAF_WIDTH * 0.5
MOVING_PLATE_INNER_X = MOVING_PLATE_CENTER_X - LEAF_WIDTH * 0.5


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _translated_profile(
    profile: list[tuple[float, float]], dx: float, dz: float
) -> list[tuple[float, float]]:
    return [(x + dx, z + dz) for x, z in profile]


def _leaf_plate_mesh(name: str):
    outer = rounded_rect_profile(LEAF_WIDTH, LEAF_HEIGHT, radius=0.018, corner_segments=8)
    screw_hole = _circle_profile(0.0105, segments=32)
    hole_profiles = [
        _translated_profile(screw_hole, dx, dz)
        for dx in (-0.050, 0.050)
        for dz in (-0.105, 0.105)
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            hole_profiles,
            LEAF_THICKNESS,
            center=True,
        ),
        name,
    )


def _add_screw_head(part, *, x: float, z: float, y_face: float, outward_sign: float, metal, slot):
    # Cylinder axes are local +Z, so roll by 90 degrees to make a low round cap on the leaf face.
    part.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(x, y_face + outward_sign * 0.0015, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name=f"screw_head_{x:.2f}_{z:.2f}".replace("-", "m"),
    )
    part.visual(
        Box((0.021, 0.0010, 0.0042)),
        origin=Origin(xyz=(x, y_face + outward_sign * 0.0037, z), rpy=(0.0, 0.0, 0.18)),
        material=slot,
        name=f"screw_slot_{x:.2f}_{z:.2f}".replace("-", "m"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_equipment_door_hinge")

    leaf_paint = model.material("blackened_leaf_steel", rgba=(0.10, 0.11, 0.11, 1.0))
    barrel_steel = model.material("oiled_barrel_steel", rgba=(0.33, 0.34, 0.34, 1.0))
    bolt_steel = model.material("bright_bolt_heads", rgba=(0.70, 0.71, 0.68, 1.0))
    dark_slot = model.material("dark_recesses", rgba=(0.025, 0.025, 0.022, 1.0))

    fixed_plate_mesh = _leaf_plate_mesh("fixed_leaf_plate")
    moving_plate_mesh = _leaf_plate_mesh("moving_leaf_plate")

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        fixed_plate_mesh,
        origin=Origin(
            xyz=(FIXED_PLATE_CENTER_X, -LEAF_Y_OFFSET, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=leaf_paint,
        name="leaf_plate",
    )
    fixed_leaf.visual(
        Box((0.024, 0.011, LEAF_HEIGHT - 0.030)),
        origin=Origin(xyz=(FIXED_PLATE_INNER_X - 0.012, -LEAF_Y_OFFSET, 0.0)),
        material=leaf_paint,
        name="hinge_edge_land",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        moving_plate_mesh,
        origin=Origin(
            xyz=(MOVING_PLATE_CENTER_X, LEAF_Y_OFFSET, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=leaf_paint,
        name="leaf_plate",
    )
    moving_leaf.visual(
        Box((0.024, 0.011, LEAF_HEIGHT - 0.030)),
        origin=Origin(xyz=(MOVING_PLATE_INNER_X + 0.012, LEAF_Y_OFFSET, 0.0)),
        material=leaf_paint,
        name="hinge_edge_land",
    )

    z0 = -HINGE_LENGTH * 0.5 + KNUCKLE_LENGTH * 0.5
    knuckle_centers = [z0 + index * (KNUCKLE_LENGTH + KNUCKLE_GAP) for index in range(5)]
    for index, z_center in enumerate(knuckle_centers):
        target = fixed_leaf if index % 2 == 0 else moving_leaf
        leaf_x = FIXED_PLATE_INNER_X if index % 2 == 0 else MOVING_PLATE_INNER_X
        tab_center_x = leaf_x + (-0.006 if index % 2 == 0 else 0.006)
        tab_center_y = -LEAF_Y_OFFSET if index % 2 == 0 else LEAF_Y_OFFSET
        target.visual(
            Cylinder(radius=BARREL_RADIUS, length=KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=barrel_steel,
            name=f"barrel_knuckle_{index}",
        )
        target.visual(
            Box((0.034, 0.014, KNUCKLE_LENGTH * 0.92)),
            origin=Origin(xyz=(tab_center_x, tab_center_y, z_center)),
            material=barrel_steel,
            name=f"rolled_tab_{index}",
        )

    fixed_leaf.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, HINGE_LENGTH * 0.5 + 0.0035)),
        material=bolt_steel,
        name="upper_pin_head",
    )
    fixed_leaf.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -HINGE_LENGTH * 0.5 - 0.0035)),
        material=bolt_steel,
        name="lower_pin_head",
    )

    for base_x in (FIXED_PLATE_CENTER_X, MOVING_PLATE_CENTER_X):
        target = fixed_leaf if base_x < 0.0 else moving_leaf
        y_face = -LEAF_Y_OFFSET - LEAF_THICKNESS * 0.5 if base_x < 0.0 else LEAF_Y_OFFSET + LEAF_THICKNESS * 0.5
        outward = -1.0 if base_x < 0.0 else 1.0
        for dx in (-0.050, 0.050):
            for z in (-0.105, 0.105):
                _add_screw_head(
                    target,
                    x=base_x + dx,
                    z=z,
                    y_face=y_face,
                    outward_sign=outward,
                    metal=bolt_steel,
                    slot=dark_slot,
                )

    model.articulation(
        "barrel_axis",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.8, lower=0.0, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("barrel_axis")

    ctx.expect_gap(
        moving_leaf,
        fixed_leaf,
        axis="x",
        positive_elem="leaf_plate",
        negative_elem="leaf_plate",
        min_gap=0.055,
        name="closed leaves keep a clean barrel-side edge offset",
    )
    ctx.expect_overlap(
        moving_leaf,
        fixed_leaf,
        axes="z",
        elem_a="leaf_plate",
        elem_b="leaf_plate",
        min_overlap=0.300,
        name="broad leaves span the same equipment-door height",
    )
    ctx.check(
        "moving leaf has one revolute barrel-axis joint",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in (hinge.axis or ())) == (0.0, 0.0, 1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )

    with ctx.pose({hinge: 1.20}):
        ctx.expect_gap(
            moving_leaf,
            fixed_leaf,
            axis="y",
            positive_elem="leaf_plate",
            negative_elem="leaf_plate",
            min_gap=0.025,
            name="moving leaf swings outward about the barrel",
        )

    return ctx.report()


object_model = build_object_model()
