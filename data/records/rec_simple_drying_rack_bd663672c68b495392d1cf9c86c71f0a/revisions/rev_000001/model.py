from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_drying_rack")

    galvanized = model.material("hot_dip_galvanized", rgba=(0.63, 0.66, 0.64, 1.0))
    yellow = model.material("safety_yellow_powdercoat", rgba=(0.96, 0.66, 0.08, 1.0))
    dark_steel = model.material("blackened_hinge_steel", rgba=(0.05, 0.055, 0.055, 1.0))
    rubber = model.material("replaceable_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    red = model.material("red_stop_pads", rgba=(0.72, 0.04, 0.03, 1.0))
    zinc = model.material("zinc_service_plates", rgba=(0.78, 0.78, 0.70, 1.0))

    def tube_x(part, name, center, length, radius, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def tube_y(part, name, center, length, radius, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(-pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    base = model.part("base")

    # Fixed central drying deck: chunky welded rails and serviceable hinge shafts.
    for y in (-0.24, 0.24):
        tube_x(base, f"deck_side_{'neg' if y < 0 else 'pos'}", (0.0, y, 0.86), 1.18, 0.018, galvanized)
    for x in (-0.58, 0.58):
        tube_y(base, f"deck_end_{'neg' if x < 0 else 'pos'}", (x, 0.0, 0.86), 0.52, 0.018, galvanized)
    for i, y in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        tube_x(base, f"fixed_hanging_rail_{i}", (0.0, y, 0.885), 1.18, 0.010, galvanized)

    # A-frame leg weldments and lower runners.  The tubes slightly pass into one
    # another, as real welded work-rack tubes do, so the base is one connected part.
    for y in (-0.38, 0.38):
        tube_x(base, f"floor_runner_{'neg' if y < 0 else 'pos'}", (0.0, y, 0.045), 1.12, 0.020, galvanized)
    for x in (-0.50, 0.50):
        for side in (-1.0, 1.0):
            theta = side * 0.165
            base.visual(
                Cylinder(radius=0.018, length=0.82),
                origin=Origin(xyz=(x, side * 0.315, 0.445), rpy=(theta, 0.0, 0.0)),
                material=galvanized,
                name=f"leg_{'neg' if x < 0 else 'pos'}_{'neg' if side < 0 else 'pos'}",
            )
    tube_x(base, "lower_cross_tie", (0.0, 0.0, 0.19), 1.02, 0.014, galvanized)
    for x in (-0.52, 0.52):
        tube_y(base, f"maintenance_crossbar_{'neg' if x < 0 else 'pos'}", (x, 0.0, 0.19), 0.77, 0.014, galvanized)

    # Replaceable rubber feet clamp around the lower runners.
    for x in (-0.55, 0.55):
        for y in (-0.38, 0.38):
            base.visual(
                Box((0.11, 0.075, 0.035)),
                origin=Origin(xyz=(x, y, 0.023)),
                material=rubber,
                name=f"rubber_foot_{'neg' if x < 0 else 'pos'}_{'neg' if y < 0 else 'pos'}",
            )

    # Full-length hinge pins, exposed support saddles, stop blocks, and bolt heads.
    for side in (-1.0, 1.0):
        side_name = "neg" if side < 0 else "pos"
        tube_x(base, f"hinge_shaft_{side_name}", (0.0, side * 0.31, 0.86), 1.22, 0.014, dark_steel)
        for x in (-0.50, 0.0, 0.50):
            base.visual(
                Box((0.085, 0.070, 0.040)),
                origin=Origin(xyz=(x, side * 0.275, 0.842)),
                material=zinc,
                name=f"shaft_saddle_{side_name}_{x:+.1f}",
            )
            base.visual(
                Box((0.105, 0.070, 0.030)),
                origin=Origin(xyz=(x, side * 0.345, 0.855)),
                material=red,
                name=f"fold_stop_{side_name}_{x:+.1f}",
            )
            for bx in (-0.025, 0.025):
                base.visual(
                    Cylinder(radius=0.007, length=0.010),
                    origin=Origin(xyz=(x + bx, side * 0.255, 0.868)),
                    material=dark_steel,
                    name=f"saddle_bolt_{side_name}_{x:+.1f}_{'neg' if bx < 0 else 'pos'}",
                )

    def add_wing(name: str, side: float):
        wing = model.part(name)
        side_name = "neg" if side < 0 else "pos"

        # Two service bushings sit on the base hinge shaft; visible gaps leave the
        # base saddles clear for inspection and replacement.
        for i, x in enumerate((-0.28, 0.28)):
            tube_x(wing, f"hinge_bushing_{i}", (x, 0.0, 0.0), 0.28, 0.026, rubber)
            wing.visual(
                Box((0.20, 0.085, 0.034)),
                origin=Origin(xyz=(x, side * 0.067, 0.018)),
                material=zinc,
                name=f"hinge_link_plate_{i}",
            )
            for bx in (-0.065, 0.065):
                wing.visual(
                    Cylinder(radius=0.007, length=0.010),
                    origin=Origin(xyz=(x + bx, side * 0.089, 0.040)),
                    material=dark_steel,
                    name=f"link_bolt_{i}_{'neg' if bx < 0 else 'pos'}",
                )

        # Perimeter frame and parallel hanging rails.  End bars and a middle
        # service crossbar tie every rail back into the hinge links.
        tube_x(wing, "inner_frame_tube", (0.0, side * 0.080, 0.030), 1.10, 0.015, yellow)
        tube_x(wing, "outer_frame_tube", (0.0, side * 0.500, 0.030), 1.10, 0.015, yellow)
        for i, y in enumerate((0.160, 0.240, 0.320, 0.400)):
            tube_x(wing, f"hanging_rail_{i}", (0.0, side * y, 0.045), 1.08, 0.0095, yellow)
        for i, x in enumerate((-0.54, 0.0, 0.54)):
            tube_y(wing, f"crossbar_{i}", (x, side * 0.290, 0.034), 0.46, 0.0125, yellow)

        # Field-replaceable wear strips and end bumpers clamp over the outer rail.
        for i, x in enumerate((-0.36, 0.0, 0.36)):
            tube_x(wing, f"replaceable_wear_sleeve_{i}", (x, side * 0.500, 0.030), 0.16, 0.020, rubber)
        for x in (-0.57, 0.57):
            wing.visual(
                Box((0.055, 0.065, 0.045)),
                origin=Origin(xyz=(x, side * 0.500, 0.030)),
                material=rubber,
                name=f"outer_bumper_{'neg' if x < 0 else 'pos'}",
            )
        wing.visual(
            Box((0.28, 0.050, 0.026)),
            origin=Origin(xyz=(0.0, side * 0.520, 0.055)),
            material=red,
            name=f"visible_stop_face_{side_name}",
        )
        return wing

    wing_0 = add_wing("wing_0", 1.0)
    wing_1 = add_wing("wing_1", -1.0)

    model.articulation(
        "base_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wing_0,
        origin=Origin(xyz=(0.0, 0.31, 0.86)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.55),
        motion_properties=MotionProperties(damping=0.35, friction=0.18),
    )
    model.articulation(
        "base_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wing_1,
        origin=Origin(xyz=(0.0, -0.31, 0.86)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.55),
        motion_properties=MotionProperties(damping=0.35, friction=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    hinge_0 = object_model.get_articulation("base_to_wing_0")
    hinge_1 = object_model.get_articulation("base_to_wing_1")

    for side_name, wing in (("pos", wing_0), ("neg", wing_1)):
        for i in (0, 1):
            ctx.allow_overlap(
                base,
                wing,
                elem_a=f"hinge_shaft_{side_name}",
                elem_b=f"hinge_bushing_{i}",
                reason="A replaceable polymer hinge bushing is intentionally captured around the fixed steel shaft.",
            )
            ctx.expect_overlap(
                base,
                wing,
                axes="x",
                elem_a=f"hinge_shaft_{side_name}",
                elem_b=f"hinge_bushing_{i}",
                min_overlap=0.20,
                name=f"{side_name} wing bushing {i} has retained shaft length",
            )
            ctx.expect_within(
                wing,
                base,
                axes="z",
                inner_elem=f"hinge_bushing_{i}",
                outer_elem=f"hinge_shaft_{side_name}",
                margin=0.014,
                name=f"{side_name} wing bushing {i} is centered on hinge pin height",
            )

    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        ctx.expect_gap(
            wing_0,
            base,
            axis="y",
            positive_elem="inner_frame_tube",
            negative_elem="deck_side_pos",
            min_gap=0.030,
            name="positive wing clears central deck when open",
        )
        ctx.expect_gap(
            base,
            wing_1,
            axis="y",
            positive_elem="deck_side_neg",
            negative_elem="inner_frame_tube",
            min_gap=0.030,
            name="negative wing clears central deck when open",
        )

    open_0 = ctx.part_world_aabb(wing_0)
    open_1 = ctx.part_world_aabb(wing_1)
    with ctx.pose({hinge_0: 1.55, hinge_1: 1.55}):
        folded_0 = ctx.part_world_aabb(wing_0)
        folded_1 = ctx.part_world_aabb(wing_1)
        ctx.expect_gap(
            wing_0,
            base,
            axis="z",
            positive_elem="outer_frame_tube",
            negative_elem="hinge_shaft_pos",
            min_gap=0.30,
            name="positive wing stop folds upward",
        )
        ctx.expect_gap(
            wing_1,
            base,
            axis="z",
            positive_elem="outer_frame_tube",
            negative_elem="hinge_shaft_neg",
            min_gap=0.30,
            name="negative wing stop folds upward",
        )
    ctx.check(
        "folded wings are narrower than service position",
        open_0 is not None
        and open_1 is not None
        and folded_0 is not None
        and folded_1 is not None
        and (folded_0[1][1] - folded_1[0][1]) < (open_0[1][1] - open_1[0][1]) - 0.25,
        details=f"open_0={open_0}, open_1={open_1}, folded_0={folded_0}, folded_1={folded_1}",
    )

    return ctx.report()


object_model = build_object_model()
