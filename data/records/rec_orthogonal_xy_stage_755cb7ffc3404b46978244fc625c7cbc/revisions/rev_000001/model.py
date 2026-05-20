from __future__ import annotations

import math

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


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xy_stage_mechanical_study")

    cast = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    plate = model.material("machined_plate", rgba=(0.42, 0.45, 0.46, 1.0))
    rail = model.material("ground_rail_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    black = model.material("black_oxide_blocks", rgba=(0.03, 0.035, 0.04, 1.0))
    cover = model.material("removable_cover_bluegrey", rgba=(0.22, 0.28, 0.32, 1.0))
    fastener = model.material("dark_cap_screws", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("matte_rubber_stops", rgba=(0.015, 0.014, 0.012, 1.0))
    cable = model.material("black_cable_chain", rgba=(0.025, 0.025, 0.023, 1.0))
    bronze = model.material("oiled_bronze_nuts", rgba=(0.57, 0.42, 0.21, 1.0))

    base = model.part("base")
    _box(base, "base_plate", (0.95, 0.72, 0.050), (0.0, 0.0, 0.025), cast)
    _box(base, "under_rib_0", (0.040, 0.64, 0.025), (-0.43, 0.0, -0.0125), cast)
    _box(base, "under_rib_1", (0.040, 0.64, 0.025), (0.43, 0.0, -0.0125), cast)
    _box(base, "under_cross_rib_0", (0.80, 0.035, 0.025), (0.0, -0.29, -0.0125), cast)
    _box(base, "under_cross_rib_1", (0.80, 0.035, 0.025), (0.0, 0.29, -0.0125), cast)
    for i, (x, y) in enumerate(((-0.38, -0.28), (-0.38, 0.28), (0.38, -0.28), (0.38, 0.28))):
        _cyl(base, f"leveling_foot_{i}", 0.045, 0.020, (x, y, -0.010), rubber)

    # Lower Y axis: two exposed guide rails on raised support ways.
    for i, x in enumerate((-0.31, 0.31)):
        _box(base, f"y_rail_support_{i}", (0.070, 0.670, 0.015), (x, 0.0, 0.0575), plate)
        _box(base, f"y_rail_{i}", (0.036, 0.620, 0.024), (x, 0.0, 0.0770), rail)
        for j, y in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
            _cyl(base, f"y_rail_bolt_{i}_{j}", 0.006, 0.004, (x, y, 0.091), fastener)
        for k, y in enumerate((-0.329, 0.329)):
            _box(base, f"y_limit_block_{i}_{k}", (0.075, 0.032, 0.055), (x, y, 0.0775), black)
            bumper_y = -0.308 if y < 0 else 0.308
            _cyl(
                base,
                f"y_limit_bumper_{i}_{k}",
                0.010,
                0.010,
                (x, bumper_y, 0.095),
                rubber,
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            )

    _cyl(
        base,
        "y_drive_screw",
        0.011,
        0.620,
        (0.0, 0.0, 0.105),
        rail,
        rpy=(-math.pi / 2.0, 0.0, 0.0),
    )
    for i, y in enumerate((-0.335, 0.335)):
        _box(base, f"y_screw_pillow_{i}", (0.060, 0.055, 0.075), (0.0, y, 0.0875), black)
        _cyl(base, f"y_pillow_cap_{i}", 0.014, 0.006, (0.0, y, 0.128), fastener)

    _box(base, "base_cable_tray", (0.025, 0.620, 0.025), (-0.455, 0.0, 0.0625), cable)
    for j, y in enumerate((-0.25, -0.18, -0.11, -0.04, 0.03, 0.10, 0.17, 0.24)):
        _box(base, f"base_chain_link_{j}", (0.034, 0.045, 0.018), (-0.455, y, 0.084), cable)
    _box(base, "base_chain_anchor", (0.050, 0.090, 0.070), (-0.430, -0.315, 0.085), black)

    y_slide = model.part("y_slide")
    _box(y_slide, "y_saddle_plate", (0.72, 0.46, 0.045), (0.0, 0.0, 0.160), plate)
    # Front and rear saddle flanges are split around the central Y ballscrew
    # clearance window rather than unrealistically passing through the screw.
    for side_index, y in enumerate((-0.215, 0.215)):
        _box(y_slide, f"y_side_flange_{side_index}_0", (0.285, 0.025, 0.040), (-0.1975, y, 0.1175), plate)
        _box(y_slide, f"y_side_flange_{side_index}_1", (0.285, 0.025, 0.040), (0.1975, y, 0.1175), plate)
    _box(y_slide, "y_plate_stiffener_0", (0.040, 0.390, 0.035), (-0.18, 0.0, 0.120), plate)
    _box(y_slide, "y_plate_stiffener_1", (0.040, 0.390, 0.035), (0.18, 0.0, 0.120), plate)

    bearing_index = 0
    for rail_x in (-0.31, 0.31):
        for y in (-0.18, 0.18):
            _box(
                y_slide,
                f"y_bearing_bridge_{bearing_index}",
                (0.078, 0.112, 0.0485),
                (rail_x, y, 0.11325),
                black,
            )
            for side, sx in enumerate((-1.0, 1.0)):
                _box(
                    y_slide,
                    f"y_bearing_skirt_{bearing_index}_{side}",
                    (0.012, 0.112, 0.050),
                    (rail_x + sx * 0.028, y, 0.114),
                    black,
                )
            bearing_index += 1

    _box(y_slide, "y_nut_bridge", (0.065, 0.090, 0.011), (0.0, 0.0, 0.132), bronze)
    _box(y_slide, "y_nut_ear_0", (0.014, 0.090, 0.035), (-0.025, 0.0, 0.1095), bronze)
    _box(y_slide, "y_nut_ear_1", (0.014, 0.090, 0.035), (0.025, 0.0, 0.1095), bronze)

    # The second stage's support base is carried by the Y saddle, so the two
    # guide directions remain visibly stacked and orthogonal.
    _box(y_slide, "x_axis_base", (0.64, 0.34, 0.018), (0.0, 0.0, 0.1915), plate)
    _box(y_slide, "x_base_stiffener_0", (0.055, 0.32, 0.030), (-0.265, 0.0, 0.1675), plate)
    _box(y_slide, "x_base_stiffener_1", (0.055, 0.32, 0.030), (0.265, 0.0, 0.1675), plate)
    for i, y in enumerate((-0.14, 0.14)):
        _box(y_slide, f"x_rail_{i}", (0.560, 0.032, 0.024), (0.0, y, 0.2125), rail)
        for j, x in enumerate((-0.20, 0.0, 0.20)):
            _cyl(y_slide, f"x_rail_bolt_{i}_{j}", 0.0045, 0.004, (x, y, 0.2265), fastener)
        for k, x in enumerate((-0.295, 0.295)):
            _box(y_slide, f"x_limit_block_{i}_{k}", (0.035, 0.072, 0.052), (x, y, 0.2085), black)
            bumper_x = -0.275 if x < 0 else 0.275
            _cyl(
                y_slide,
                f"x_limit_bumper_{i}_{k}",
                0.008,
                0.010,
                (bumper_x, y, 0.231),
                rubber,
                rpy=(0.0, math.pi / 2.0, 0.0),
            )

    _cyl(
        y_slide,
        "x_drive_screw",
        0.008,
        0.590,
        (0.0, 0.0, 0.236),
        rail,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    for i, x in enumerate((-0.290, 0.290)):
        _box(y_slide, f"x_screw_pillow_{i}", (0.055, 0.055, 0.065), (x, 0.0, 0.233), black)
        _cyl(
            y_slide,
            f"x_pillow_cap_{i}",
            0.011,
            0.005,
            (x, 0.0, 0.268),
            fastener,
        )

    for i, y in enumerate((-0.190, 0.190)):
        _box(y_slide, f"y_access_cover_{i}", (0.230, 0.070, 0.008), (0.0, y, 0.1865), cover)
        for j, x in enumerate((-0.080, 0.080)):
            _cyl(y_slide, f"y_cover_screw_{i}_{j}", 0.0045, 0.003, (x, y, 0.192), fastener)
    _box(y_slide, "y_chain_foot", (0.065, 0.120, 0.014), (-0.340, -0.180, 0.1895), black)
    _box(y_slide, "y_chain_bracket", (0.025, 0.120, 0.080), (-0.372, -0.180, 0.229), black)
    _box(y_slide, "y_limit_flag", (0.040, 0.020, 0.060), (0.365, 0.0, 0.132), black)

    x_slide = model.part("x_slide")
    _box(x_slide, "top_table", (0.420, 0.340, 0.040), (0.0, 0.0, 0.285), plate)
    _box(x_slide, "top_under_rib_0", (0.035, 0.220, 0.025), (-0.120, 0.0, 0.2525), plate)
    _box(x_slide, "top_under_rib_1", (0.035, 0.220, 0.025), (0.120, 0.0, 0.2525), plate)

    bearing_index = 0
    for rail_y in (-0.14, 0.14):
        for x in (-0.130, 0.130):
            _box(
                x_slide,
                f"x_bearing_bridge_{bearing_index}",
                (0.110, 0.074, 0.0405),
                (x, rail_y, 0.24475),
                black,
            )
            for side, sy in enumerate((-1.0, 1.0)):
                _box(
                    x_slide,
                    f"x_bearing_skirt_{bearing_index}_{side}",
                    (0.110, 0.012, 0.050),
                    (x, rail_y + sy * 0.026, 0.2495),
                    black,
                )
            bearing_index += 1

    _box(x_slide, "x_nut_bridge", (0.085, 0.055, 0.014), (0.0, 0.0, 0.258), bronze)
    _box(x_slide, "x_nut_ear_0", (0.085, 0.012, 0.030), (0.0, -0.020, 0.2365), bronze)
    _box(x_slide, "x_nut_ear_1", (0.085, 0.012, 0.030), (0.0, 0.020, 0.2365), bronze)

    for i, y in enumerate((-0.090, 0.0, 0.090)):
        _box(x_slide, f"table_tslot_{i}", (0.380, 0.012, 0.003), (0.0, y, 0.3065), black)
    for i, x in enumerate((-0.115, 0.115)):
        _box(x_slide, f"top_access_cover_{i}", (0.120, 0.078, 0.006), (x, 0.122, 0.308), cover)
        for j, y in enumerate((0.095, 0.149)):
            _cyl(x_slide, f"top_cover_screw_{i}_{j}", 0.0038, 0.003, (x, y, 0.3125), fastener)
    _box(x_slide, "top_chain_foot", (0.060, 0.060, 0.012), (-0.195, 0.190, 0.311), black)
    _box(x_slide, "top_chain_bracket", (0.025, 0.030, 0.070), (-0.225, 0.220, 0.340), black)
    _box(x_slide, "x_limit_flag", (0.018, 0.025, 0.045), (0.204, 0.168, 0.2425), black)

    model.articulation(
        "base_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_slide,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.25, lower=-0.110, upper=0.110),
        motion_properties=MotionProperties(damping=25.0, friction=8.0),
    )
    model.articulation(
        "y_slide_to_x_slide",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=x_slide,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.22, lower=-0.140, upper=0.140),
        motion_properties=MotionProperties(damping=18.0, friction=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    y_slide = object_model.get_part("y_slide")
    x_slide = object_model.get_part("x_slide")
    y_joint = object_model.get_articulation("base_to_y_slide")
    x_joint = object_model.get_articulation("y_slide_to_x_slide")

    ctx.expect_contact(
        y_slide,
        base,
        elem_a="y_bearing_bridge_0",
        elem_b="y_rail_0",
        name="lower guide carriage bears on the Y rail",
    )
    ctx.expect_contact(
        x_slide,
        y_slide,
        elem_a="x_bearing_bridge_0",
        elem_b="x_rail_0",
        name="upper guide carriage bears on the X rail",
    )
    ctx.expect_overlap(
        y_slide,
        base,
        axes="y",
        elem_a="y_bearing_bridge_0",
        elem_b="y_rail_0",
        min_overlap=0.08,
        name="Y carriage retained on rail at centered pose",
    )
    ctx.expect_overlap(
        x_slide,
        y_slide,
        axes="x",
        elem_a="x_bearing_bridge_0",
        elem_b="x_rail_0",
        min_overlap=0.08,
        name="X carriage retained on rail at centered pose",
    )

    y_rest = ctx.part_world_position(y_slide)
    x_rest = ctx.part_world_position(x_slide)
    with ctx.pose({y_joint: 0.110}):
        ctx.expect_overlap(
            y_slide,
            base,
            axes="y",
            elem_a="y_bearing_bridge_0",
            elem_b="y_rail_0",
            min_overlap=0.08,
            name="Y carriage retained on rail at positive travel",
        )
        y_positive = ctx.part_world_position(y_slide)
    with ctx.pose({y_joint: -0.110}):
        y_negative = ctx.part_world_position(y_slide)

    with ctx.pose({x_joint: 0.140}):
        ctx.expect_overlap(
            x_slide,
            y_slide,
            axes="x",
            elem_a="x_bearing_bridge_0",
            elem_b="x_rail_0",
            min_overlap=0.08,
            name="X carriage retained on rail at positive travel",
        )
        x_positive = ctx.part_world_position(x_slide)
    with ctx.pose({x_joint: -0.140}):
        x_negative = ctx.part_world_position(x_slide)

    ctx.check(
        "Y slide translates only along the Y axis",
        y_rest is not None
        and y_positive is not None
        and y_negative is not None
        and y_positive[1] > y_rest[1] + 0.10
        and y_negative[1] < y_rest[1] - 0.10
        and abs(y_positive[0] - y_rest[0]) < 1e-6
        and abs(y_positive[2] - y_rest[2]) < 1e-6,
        details=f"rest={y_rest}, positive={y_positive}, negative={y_negative}",
    )
    ctx.check(
        "X table translates only along the X axis",
        x_rest is not None
        and x_positive is not None
        and x_negative is not None
        and x_positive[0] > x_rest[0] + 0.13
        and x_negative[0] < x_rest[0] - 0.13
        and abs(x_positive[1] - x_rest[1]) < 1e-6
        and abs(x_positive[2] - x_rest[2]) < 1e-6,
        details=f"rest={x_rest}, positive={x_positive}, negative={x_negative}",
    )

    return ctx.report()


object_model = build_object_model()
