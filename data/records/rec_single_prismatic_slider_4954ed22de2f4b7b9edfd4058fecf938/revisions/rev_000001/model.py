from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_prismatic_slider_study")

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.65, 0.67, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.17, 0.31, 1.0))
    cover_black = model.material("black_oxide_cover", rgba=(0.02, 0.025, 0.028, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    fastener = model.material("blackened_fasteners", rgba=(0.01, 0.01, 0.012, 1.0))

    rail_y = (-0.09, 0.09)
    bed_top = 0.055
    rail_height = 0.035
    rail_top = bed_top + rail_height

    fixed_base = model.part("fixed_base")
    fixed_base.visual(
        Box((1.24, 0.34, bed_top)),
        origin=Origin(xyz=(0.0, 0.0, bed_top / 2.0)),
        material=cast_iron,
        name="bed_plate",
    )
    # Deep side webs and end plates make the root part read as a fabricated
    # machine slide base rather than a flat display plinth.
    for i, y in enumerate((-0.157, 0.157)):
        fixed_base.visual(
            Box((1.20, 0.026, 0.095)),
            origin=Origin(xyz=(0.0, y, 0.052)),
            material=cast_iron,
            name=f"side_web_{i}",
        )
    for i, x in enumerate((-0.59, 0.59)):
        fixed_base.visual(
            Box((0.045, 0.32, 0.078)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=cast_iron,
            name=f"end_bracket_{i}",
        )

    # Linear guide ways: raised rail seats plus two hardened rectangular rails.
    for i, y in enumerate(rail_y):
        fixed_base.visual(
            Box((1.10, 0.060, 0.014)),
            origin=Origin(xyz=(0.0, y, bed_top + 0.006)),
            material=machined_steel,
            name=f"rail_seat_{i}",
        )
        fixed_base.visual(
            Box((1.10, 0.032, rail_height)),
            origin=Origin(xyz=(0.0, y, bed_top + rail_height / 2.0)),
            material=rail_steel,
            name=f"rail_{i}",
        )
        for j, x in enumerate((-0.45, -0.25, -0.05, 0.15, 0.35, 0.50)):
            for k, dy in enumerate((-0.028, 0.028)):
                fixed_base.visual(
                    Cylinder(radius=0.0045, length=0.004),
                    origin=Origin(xyz=(x, y + dy, bed_top + 0.015)),
                    material=fastener,
                    name=f"rail_screw_{i}_{j}_{k}",
                )

    # Adjustable hard stop blocks.  Their inboard faces nearly meet the carriage
    # bumpers at the lower and upper travel limits.
    fixed_base.visual(
        Box((0.030, 0.220, 0.060)),
        origin=Origin(xyz=(-0.485, 0.0, bed_top + 0.030)),
        material=machined_steel,
        name="left_stop",
    )
    fixed_base.visual(
        Box((0.030, 0.220, 0.060)),
        origin=Origin(xyz=(0.405, 0.0, bed_top + 0.030)),
        material=machined_steel,
        name="right_stop",
    )
    for x, label in [(-0.485, "left"), (0.405, "right")]:
        fixed_base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.075, bed_top + 0.062)),
            material=fastener,
            name=f"{label}_stop_screw_0",
        )
        fixed_base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, 0.075, bed_top + 0.062)),
            material=fastener,
            name=f"{label}_stop_screw_1",
        )

    # Low central accordion dust-cover stand-ins.  They are fixed below the
    # carriage envelope, with a continuous spine so the pleats are supported.
    for group, x0 in enumerate((-0.285, 0.185)):
        fixed_base.visual(
            Box((0.260, 0.052, 0.010)),
            origin=Origin(xyz=(x0, 0.0, bed_top + 0.0045)),
            material=rubber,
            name=f"bellows_spine_{group}",
        )
        for j in range(9):
            x = x0 - 0.105 + j * 0.026
            fixed_base.visual(
                Box((0.010, 0.060, 0.034)),
                origin=Origin(xyz=(x, 0.0, bed_top + 0.022)),
                material=rubber,
                name=f"bellows_pleat_{group}_{j}",
            )

    # Four bolted mounting ears tied into the base side edges.
    for ix, x in enumerate((-0.46, 0.46)):
        for iy, y in enumerate((-0.205, 0.205)):
            fixed_base.visual(
                Box((0.155, 0.070, 0.018)),
                origin=Origin(xyz=(x, y, 0.012)),
                material=cast_iron,
                name=f"mount_ear_{ix}_{iy}",
            )
            fixed_base.visual(
                Cylinder(radius=0.019, length=0.006),
                origin=Origin(xyz=(x, y, 0.0225)),
                material=fastener,
                name=f"ear_bolt_{ix}_{iy}",
            )

    carriage = model.part("carriage")
    # Four bearing trucks ride on the rails.  The dark pads touch the ground
    # rail tops while the jaws straddle the rails with visible clearance.
    bearing_x = (-0.145, 0.145)
    for ix, x in enumerate(bearing_x):
        for iy, y in enumerate(rail_y):
            idx = ix * 2 + iy
            carriage.visual(
                Box((0.105, 0.030, 0.006)),
                origin=Origin(xyz=(x, y, 0.003)),
                material=rubber,
                name=f"bearing_pad_{idx}",
            )
            carriage.visual(
                Box((0.115, 0.066, 0.032)),
                origin=Origin(xyz=(x, y, 0.019)),
                material=carriage_blue,
                name=f"bearing_block_{idx}",
            )
            jaw_offset = 0.016 + 0.006 + 0.004
            for side, sy in enumerate((y - jaw_offset, y + jaw_offset)):
                carriage.visual(
                    Box((0.115, 0.012, 0.034)),
                    origin=Origin(xyz=(x, sy, 0.017)),
                    material=carriage_blue,
                    name=f"rail_jaw_{idx}_{side}",
                )

    carriage.visual(
        Box((0.464, 0.230, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.0515)),
        material=carriage_blue,
        name="carriage_plate",
    )
    for i, y in enumerate((-0.124, 0.124)):
        carriage.visual(
            Box((0.360, 0.018, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.066)),
            material=carriage_blue,
            name=f"side_cheek_{i}",
        )

    # Bolted access covers on top of the moving carriage, intentionally flat and
    # removable-looking rather than product-styled.
    for i, x in enumerate((-0.095, 0.095)):
        carriage.visual(
            Box((0.165, 0.085, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.071)),
            material=cover_black,
            name=f"access_cover_{i}",
        )
        for j, sx in enumerate((-0.060, 0.060)):
            for k, sy in enumerate((-0.030, 0.030)):
                carriage.visual(
                    Cylinder(radius=0.0042, length=0.004),
                    origin=Origin(xyz=(x + sx, sy, 0.075)),
                    material=fastener,
                    name=f"cover_screw_{i}_{j}_{k}",
                )

    carriage.visual(
        Box((0.330, 0.038, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=machined_steel,
        name="top_spine",
    )
    for ix, x in enumerate((-0.120, 0.120)):
        for iy, y in enumerate((-0.162, 0.162)):
            carriage.visual(
                Box((0.110, 0.058, 0.012)),
                origin=Origin(xyz=(x, y, 0.070)),
                material=carriage_blue,
                name=f"carriage_ear_{ix}_{iy}",
            )
            carriage.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(x, y, 0.077)),
                material=fastener,
                name=f"carriage_ear_bolt_{ix}_{iy}",
            )

    # Replaceable bumper pads that meet the adjustable stops at the joint limits.
    carriage.visual(
        Box((0.018, 0.160, 0.034)),
        origin=Origin(xyz=(-0.240, 0.0, 0.023)),
        material=rubber,
        name="left_bumper",
    )
    carriage.visual(
        Box((0.018, 0.160, 0.034)),
        origin=Origin(xyz=(0.240, 0.0, 0.023)),
        material=rubber,
        name="right_bumper",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=fixed_base,
        child=carriage,
        origin=Origin(xyz=(-0.220, 0.0, rail_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.36),
        motion_properties=MotionProperties(damping=18.0, friction=7.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_base = object_model.get_part("fixed_base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")

    # The carriage is not a floating block: its bearing pads sit on the exposed
    # rails in the retracted pose.
    with ctx.pose({slide: 0.0}):
        for idx, rail_name in enumerate(("rail_0", "rail_1", "rail_0", "rail_1")):
            ctx.expect_contact(
                carriage,
                fixed_base,
                elem_a=f"bearing_pad_{idx}",
                elem_b=rail_name,
                contact_tol=1e-5,
                name=f"bearing pad {idx} rides on {rail_name}",
            )
        ctx.expect_gap(
            carriage,
            fixed_base,
            axis="x",
            positive_elem="left_bumper",
            negative_elem="left_stop",
            min_gap=0.0,
            max_gap=0.003,
            name="lower travel bumper nearly meets stop",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.36}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            fixed_base,
            carriage,
            axis="x",
            positive_elem="right_stop",
            negative_elem="right_bumper",
            min_gap=0.0,
            max_gap=0.003,
            name="upper travel bumper nearly meets stop",
        )
        for idx, rail_name in enumerate(("rail_0", "rail_1", "rail_0", "rail_1")):
            ctx.expect_overlap(
                carriage,
                fixed_base,
                axes="x",
                min_overlap=0.09,
                elem_a=f"bearing_pad_{idx}",
                elem_b=rail_name,
                name=f"extended bearing pad {idx} remains supported",
            )

    ctx.check(
        "prismatic joint moves carriage along +x",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.34,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
