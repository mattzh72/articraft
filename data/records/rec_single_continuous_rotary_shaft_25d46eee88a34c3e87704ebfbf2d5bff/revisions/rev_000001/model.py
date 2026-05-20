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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_labeling_platen")

    cast_iron = Material("satin_cast_iron", rgba=(0.18, 0.20, 0.21, 1.0))
    dark_bore = Material("dark_burnished_bore", rgba=(0.035, 0.037, 0.038, 1.0))
    machined_steel = Material("brushed_machined_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    underside_steel = Material("shadowed_plate_steel", rgba=(0.50, 0.52, 0.51, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base")

    # One revolved casting: a compact cylindrical base, a raised bearing boss,
    # and a true central bore so the rotating journal has real clearance.
    base_housing = LatheGeometry(
        [
            (0.220, 0.000),
            (0.220, 0.018),
            (0.192, 0.030),
            (0.192, 0.120),
            (0.115, 0.120),
            (0.095, 0.170),
            (0.060, 0.170),
            (0.060, 0.000),
        ],
        segments=96,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(base_housing, "base_housing"),
        origin=Origin(),
        material=cast_iron,
        name="base_housing",
    )

    bore_liner = LatheGeometry(
        [
            (0.060, 0.040),
            (0.060, 0.170),
            (0.050, 0.170),
            (0.050, 0.040),
        ],
        segments=80,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(bore_liner, "bore_liner"),
        origin=Origin(),
        material=dark_bore,
        name="bore_liner",
    )

    foot_ring = LatheGeometry(
        [
            (0.205, -0.014),
            (0.205, 0.000),
            (0.132, 0.000),
            (0.132, -0.014),
        ],
        segments=80,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(foot_ring, "foot_ring"),
        origin=Origin(),
        material=rubber,
        name="foot_ring",
    )

    platen = model.part("platen")

    # The child frame sits on the vertical spin axis at the top of the bearing
    # boss. The lower journal reaches down into the hollow base bore; all larger
    # shoulders and flanges remain just above the fixed housing.
    platen.visual(
        Cylinder(radius=0.043, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=machined_steel,
        name="lower_journal",
    )
    platen.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=machined_steel,
        name="lower_shoulder",
    )
    platen.visual(
        Cylinder(radius=0.036, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=machined_steel,
        name="upper_spindle",
    )
    platen.visual(
        Cylinder(radius=0.058, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=machined_steel,
        name="mid_shoulder",
    )
    platen.visual(
        Cylinder(radius=0.092, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.253)),
        material=machined_steel,
        name="plate_hub",
    )

    # Broad work platen: intentionally much wider than the base, with a shallow
    # retaining rim and a circular underside stiffener.
    platen.visual(
        Cylinder(radius=0.420, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=machined_steel,
        name="top_plate",
    )
    top_lip = LatheGeometry(
        [
            (0.405, 0.318),
            (0.420, 0.318),
            (0.420, 0.333),
            (0.405, 0.327),
        ],
        segments=96,
        closed=True,
    )
    platen.visual(
        mesh_from_geometry(top_lip, "top_lip"),
        origin=Origin(),
        material=machined_steel,
        name="top_lip",
    )
    underside_ring = LatheGeometry(
        [
            (0.338, 0.248),
            (0.372, 0.248),
            (0.372, 0.283),
            (0.338, 0.283),
        ],
        segments=96,
        closed=True,
    )
    platen.visual(
        mesh_from_geometry(underside_ring, "underside_ring"),
        origin=Origin(),
        material=underside_steel,
        name="underside_ring",
    )

    for idx in range(8):
        angle = idx * math.tau / 8.0
        platen.visual(
            Box((0.300, 0.024, 0.032)),
            origin=Origin(xyz=(0.215, 0.0, 0.267), rpy=(0.0, 0.0, angle)),
            material=underside_steel,
            name=f"rib_{idx}",
        )

    spin = model.articulation(
        "base_to_platen",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.01),
    )
    spin.meta["qc_samples"] = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platen = object_model.get_part("platen")
    spin = object_model.get_articulation("base_to_platen")

    ctx.check(
        "single continuous vertical spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    base_box = ctx.part_element_world_aabb(base, elem="base_housing")
    plate_box = ctx.part_element_world_aabb(platen, elem="top_plate")
    base_diameter = None if base_box is None else base_box[1][0] - base_box[0][0]
    plate_diameter = None if plate_box is None else plate_box[1][0] - plate_box[0][0]
    ctx.check(
        "top plate dominates base silhouette",
        base_diameter is not None
        and plate_diameter is not None
        and plate_diameter > 1.75 * base_diameter,
        details=f"base_diameter={base_diameter}, plate_diameter={plate_diameter}",
    )

    for angle, label in ((0.0, "rest"), (math.pi / 2.0, "quarter_turn"), (math.pi, "half_turn")):
        with ctx.pose({spin: angle}):
            ctx.expect_within(
                platen,
                base,
                axes="xy",
                inner_elem="lower_journal",
                outer_elem="bore_liner",
                margin=0.0,
                name=f"journal centered in bore at {label}",
            )
            ctx.expect_overlap(
                platen,
                base,
                axes="z",
                elem_a="lower_journal",
                elem_b="bore_liner",
                min_overlap=0.120,
                name=f"journal remains inserted at {label}",
            )
            ctx.expect_gap(
                platen,
                base,
                axis="z",
                positive_elem="lower_shoulder",
                negative_elem="base_housing",
                min_gap=0.0,
                max_gap=0.0005,
                name=f"retainer shoulder clears housing at {label}",
            )
            ctx.expect_contact(
                platen,
                base,
                elem_a="lower_shoulder",
                elem_b="base_housing",
                contact_tol=0.0005,
                name=f"thrust shoulder is grounded at {label}",
            )

    return ctx.report()


object_model = build_object_model()
