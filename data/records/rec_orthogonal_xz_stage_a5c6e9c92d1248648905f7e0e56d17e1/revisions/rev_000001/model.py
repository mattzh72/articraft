from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name, radius, length, xyz, rpy, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_fed_lift_stage")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    rail_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.75, 1.0))
    carriage_blue = model.material("machined_blue", rgba=(0.05, 0.16, 0.32, 1.0))
    black = model.material("black_oxide", rgba=(0.015, 0.015, 0.014, 1.0))
    brass = model.material("bronze_wear", rgba=(0.78, 0.52, 0.22, 1.0))
    ram_chrome = model.material("polished_ram", rgba=(0.83, 0.86, 0.87, 1.0))
    platform_grey = model.material("brushed_platform", rgba=(0.55, 0.57, 0.58, 1.0))
    red = model.material("red_stop", rgba=(0.75, 0.05, 0.03, 1.0))

    base = model.part("base_beam")
    _box(base, "beam_web", (1.30, 0.24, 0.16), (0.0, 0.0, 0.080), cast_iron)
    _box(base, "front_flange", (1.30, 0.035, 0.055), (0.0, -0.137, 0.125), cast_iron)
    _box(base, "rear_flange", (1.30, 0.035, 0.055), (0.0, 0.137, 0.125), cast_iron)

    for i, y in enumerate((-0.075, 0.075)):
        _box(base, f"rail_{i}", (1.14, 0.028, 0.035), (0.0, y, 0.1775), rail_steel)
        _box(base, f"rail_oil_groove_{i}", (1.06, 0.006, 0.004), (0.0, y, 0.1970), black)
        for j, x in enumerate((-0.46, -0.23, 0.0, 0.23, 0.46)):
            _box(
                base,
                f"rail_clamp_{i}_{j}",
                (0.055, 0.040, 0.006),
                (x, y, 0.1980),
                black,
            )

    for i, x in enumerate((-0.585, 0.585)):
        _box(base, f"end_stop_{i}", (0.050, 0.225, 0.070), (x, 0.0, 0.217), red)
        _box(base, f"stop_bumper_{i}", (0.016, 0.120, 0.050), (x * 0.985, 0.0, 0.214), black)

    _box(base, "side_screw_trough", (1.05, 0.040, 0.050), (0.0, -0.171, 0.135), cast_iron)
    base.visual(
        Cylinder(radius=0.017, length=1.04),
        origin=Origin(xyz=(0.0, -0.171, 0.158), rpy=(0.0, 1.57079632679, 0.0)),
        material=rail_steel,
        name="feed_screw",
    )
    for i, x in enumerate((-0.46, 0.0, 0.46)):
        _box(base, f"screw_pillow_{i}", (0.055, 0.068, 0.060), (x, -0.150, 0.120), cast_iron)
        _box(base, f"pillow_cap_{i}", (0.060, 0.045, 0.020), (x, -0.171, 0.160), black)

    carriage = model.part("lower_carriage")
    _box(carriage, "carriage_deck", (0.300, 0.235, 0.035), (0.0, 0.0, 0.0175), carriage_blue)
    for i, y in enumerate((-0.075, 0.075)):
        _box(carriage, f"bearing_cap_{i}", (0.245, 0.056, 0.022), (0.0, y, -0.004), black)
        _box(carriage, f"bearing_wiper_{i}", (0.260, 0.010, 0.018), (0.0, y * 1.37, 0.004), brass)

    _box(carriage, "side_feed_arm", (0.170, 0.050, 0.038), (0.000, -0.128, 0.038), carriage_blue)
    carriage.visual(
        Box((0.090, 0.065, 0.055)),
        origin=Origin(xyz=(0.000, -0.166, 0.045)),
        material=black,
        name="drive_nut_lug",
    )
    _box(carriage, "front_rib", (0.260, 0.018, 0.065), (0.000, -0.105, 0.062), carriage_blue)
    _box(carriage, "rear_rib", (0.260, 0.018, 0.065), (0.000, 0.105, 0.062), carriage_blue)
    _box(carriage, "cross_rib_0", (0.024, 0.205, 0.060), (-0.100, 0.0, 0.060), carriage_blue)
    _box(carriage, "cross_rib_1", (0.024, 0.205, 0.060), (0.100, 0.0, 0.060), carriage_blue)

    carriage.visual(
        Box((0.185, 0.185, 0.060)),
        origin=Origin(xyz=(0.020, 0.0, 0.065)),
        material=carriage_blue,
        name="guide_saddle",
    )
    for i, y in enumerate((-0.078, 0.078)):
        _box(carriage, f"guide_post_{i}", (0.060, 0.032, 0.620), (0.020, y, 0.405), carriage_blue)
        _box(carriage, f"post_cap_{i}", (0.075, 0.050, 0.030), (0.020, y, 0.730), black)
    carriage.visual(
        Box((0.064, 0.010, 0.520)),
        origin=Origin(xyz=(0.020, -0.0546, 0.405)),
        material=brass,
        name="guide_strip_0",
    )
    carriage.visual(
        Box((0.064, 0.010, 0.520)),
        origin=Origin(xyz=(0.020, 0.0546, 0.405)),
        material=brass,
        name="guide_strip_1",
    )

    _box(carriage, "front_guide_bridge", (0.035, 0.190, 0.038), (0.062, 0.0, 0.650), carriage_blue)
    _box(carriage, "rear_lower_bridge", (0.035, 0.175, 0.045), (-0.018, 0.0, 0.126), carriage_blue)

    ram = model.part("vertical_ram")
    ram.visual(
        Box((0.040, 0.040, 0.660)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=ram_chrome,
        name="ram_column",
    )
    ram.visual(
        Box((0.058, 0.018, 0.170)),
        origin=Origin(xyz=(0.0, -0.029, 0.175)),
        material=black,
        name="ram_bearing_pad_0",
    )
    ram.visual(
        Box((0.058, 0.018, 0.170)),
        origin=Origin(xyz=(0.0, 0.029, 0.175)),
        material=black,
        name="ram_bearing_pad_1",
    )
    for i, y in enumerate((-0.031, 0.031)):
        _box(ram, f"upper_bearing_pad_{i}", (0.058, 0.018, 0.140), (0.0, y * 0.935, 0.475), black)

    _box(ram, "top_collars", (0.090, 0.090, 0.035), (0.0, 0.0, 0.642), black)
    _box(ram, "platform_plate", (0.240, 0.190, 0.050), (0.0, 0.0, 0.685), platform_grey)
    _box(ram, "platform_lip_front", (0.240, 0.018, 0.035), (0.0, -0.095, 0.718), black)
    _box(ram, "platform_lip_rear", (0.240, 0.018, 0.035), (0.0, 0.095, 0.718), black)
    _box(ram, "platform_lip_end_0", (0.018, 0.190, 0.035), (-0.120, 0.0, 0.718), black)
    _box(ram, "platform_lip_end_1", (0.018, 0.190, 0.035), (0.120, 0.0, 0.718), black)

    model.articulation(
        "horizontal_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.340, 0.0, 0.214)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.28, lower=0.0, upper=0.680),
    )
    model.articulation(
        "vertical_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=ram,
        origin=Origin(xyz=(0.020, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.18, lower=0.0, upper=0.300),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_beam")
    carriage = object_model.get_part("lower_carriage")
    ram = object_model.get_part("vertical_ram")
    horizontal = object_model.get_articulation("horizontal_slide")
    vertical = object_model.get_articulation("vertical_slide")

    for i in range(2):
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            min_gap=0.003,
            max_gap=0.006,
            positive_elem=f"bearing_cap_{i}",
            negative_elem=f"rail_{i}",
            name=f"bearing cap {i} clears rail at rest",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.20,
            elem_a=f"bearing_cap_{i}",
            elem_b=f"rail_{i}",
            name=f"bearing cap {i} remains on rail at rest",
        )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        min_gap=0.050,
        positive_elem="drive_nut_lug",
        negative_elem="feed_screw",
        name="side drive lug passes above feed screw",
    )

    ctx.expect_gap(
        carriage,
        ram,
        axis="y",
        min_gap=0.010,
        max_gap=0.020,
        positive_elem="guide_strip_1",
        negative_elem="ram_bearing_pad_1",
        name="positive guide clearance",
    )
    ctx.expect_gap(
        ram,
        carriage,
        axis="y",
        min_gap=0.010,
        max_gap=0.020,
        positive_elem="ram_bearing_pad_0",
        negative_elem="guide_strip_0",
        name="negative guide clearance",
    )
    ctx.expect_within(
        ram,
        carriage,
        axes="xy",
        margin=0.0,
        inner_elem="ram_column",
        outer_elem="guide_saddle",
        name="ram centered over carriage saddle",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_ram_pos = ctx.part_world_position(ram)
    with ctx.pose({horizontal: 0.680, vertical: 0.300}):
        for i in range(2):
            ctx.expect_gap(
                carriage,
                base,
                axis="z",
                min_gap=0.003,
                max_gap=0.006,
                positive_elem=f"bearing_cap_{i}",
                negative_elem=f"rail_{i}",
                name=f"bearing cap {i} clears rail at full travel",
            )
            ctx.expect_overlap(
                carriage,
                base,
                axes="x",
                min_overlap=0.20,
                elem_a=f"bearing_cap_{i}",
                elem_b=f"rail_{i}",
                name=f"bearing cap {i} remains on rail at full travel",
            )
        ctx.expect_gap(
            carriage,
            ram,
            axis="y",
            min_gap=0.010,
            max_gap=0.020,
            positive_elem="guide_strip_1",
            negative_elem="ram_bearing_pad_1",
            name="positive guide clearance at lift",
        )
        ctx.expect_gap(
            ram,
            carriage,
            axis="y",
            min_gap=0.010,
            max_gap=0.020,
            positive_elem="ram_bearing_pad_0",
            negative_elem="guide_strip_0",
            name="negative guide clearance at lift",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)
        extended_ram_pos = ctx.part_world_position(ram)

    ctx.check(
        "horizontal stage moves along x",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.60,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )
    ctx.check(
        "vertical stage rises from carriage",
        rest_ram_pos is not None
        and extended_ram_pos is not None
        and extended_ram_pos[2] > rest_ram_pos[2] + 0.25,
        details=f"rest={rest_ram_pos}, extended={extended_ram_pos}",
    )

    return ctx.report()


object_model = build_object_model()
