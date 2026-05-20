from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _box(part, name: str, size: tuple[float, float, float], xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, rpy, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_prismatic_study")

    painted_steel = _mat(model, "painted_steel", (0.34, 0.37, 0.38, 1.0))
    dark_steel = _mat(model, "dark_steel", (0.05, 0.055, 0.06, 1.0))
    rail_steel = _mat(model, "ground_rail_steel", (0.62, 0.64, 0.62, 1.0))
    bare_aluminum = _mat(model, "bare_aluminum", (0.74, 0.76, 0.72, 1.0))
    bronze = _mat(model, "bearing_bronze", (0.72, 0.49, 0.20, 1.0))
    rubber = _mat(model, "black_rubber", (0.01, 0.01, 0.012, 1.0))
    cover_blue = _mat(model, "anodized_cover", (0.12, 0.18, 0.23, 1.0))

    base = model.part("base_frame")
    _box(base, "bed_plate", (1.42, 0.34, 0.060), (0.0, 0.0, 0.030), painted_steel)
    _box(base, "center_rib", (1.18, 0.050, 0.040), (0.0, 0.0, 0.080), painted_steel)
    for y, idx in [(-0.095, 0), (0.095, 1)]:
        _box(base, f"rail_{idx}", (1.24, 0.045, 0.045), (0.0, y, 0.0825), rail_steel)
        _box(base, f"rail_foot_{idx}", (1.30, 0.070, 0.012), (0.0, y, 0.066), dark_steel)
    for x, idx in [(-0.62, 0), (0.38, 1)]:
        _box(base, f"end_stop_{idx}", (0.050, 0.285, 0.100), (x, 0.0, 0.110), dark_steel)
        _box(base, f"bumper_{idx}", (0.018, 0.150, 0.052), (x + (0.034 if x < 0.0 else -0.034), 0.0, 0.130), rubber)
    _box(base, "datum_scale_bar", (1.10, 0.012, 0.010), (-0.02, -0.165, 0.065), bare_aluminum)

    base_cover = model.part("base_cover")
    _box(base_cover, "cover_plate", (0.290, 0.078, 0.006), (0.0, 0.0, 0.0), cover_blue)
    for sx in [-0.115, 0.115]:
        for sy in [-0.026, 0.026]:
            _cyl(base_cover, "cover_screw", 0.0075, 0.004, (sx, sy, 0.0045), (0.0, 0.0, 0.0), dark_steel)

    carriage = model.part("base_carriage")
    for y, idx in [(-0.095, 0), (0.095, 1)]:
        _box(carriage, f"shoe_{idx}", (0.235, 0.072, 0.045), (0.0, y, -0.1075), bare_aluminum)
        _box(carriage, f"gib_strip_{idx}", (0.218, 0.010, 0.022), (0.0, y * 1.31, -0.102), bronze)
    _box(carriage, "saddle_plate", (0.290, 0.292, 0.038), (0.0, 0.0, -0.099), painted_steel)
    _box(carriage, "rear_web", (0.035, 0.250, 0.165), (-0.105, 0.0, -0.035), painted_steel)
    for y, idx in [(-0.108, 0), (0.108, 1)]:
        _box(carriage, f"side_plate_{idx}", (0.145, 0.028, 0.270), (0.0, y, 0.010), painted_steel)
        _cyl(
            carriage,
            f"bearing_cap_{idx}",
            0.066,
            0.030,
            (0.0, y + (-0.018 if y < 0.0 else 0.018), 0.0),
            (-math.pi / 2.0, 0.0, 0.0),
            dark_steel,
        )
    _cyl(carriage, "axle_pin", 0.023, 0.274, (0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0), rail_steel)
    _box(carriage, "joint_guard_bridge", (0.170, 0.250, 0.035), (0.0, 0.0, 0.128), dark_steel)

    elbow = model.part("elbow")
    _cyl(elbow, "elbow_hub", 0.062, 0.154, (0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0), bronze)
    for y, idx in [(-0.048, 0), (0.048, 1)]:
        _box(elbow, f"arm_side_web_{idx}", (0.386, 0.026, 0.076), (0.255, y, 0.0), painted_steel)
    _box(elbow, "arm_bottom_web", (0.430, 0.096, 0.020), (0.225, 0.0, -0.048), painted_steel)
    _box(elbow, "arm_top_rib", (0.330, 0.040, 0.018), (0.245, 0.0, 0.047), dark_steel)
    _box(elbow, "guide_lower_plate", (0.250, 0.154, 0.018), (0.505, 0.0, -0.061), dark_steel)
    _box(elbow, "guide_upper_plate", (0.250, 0.154, 0.018), (0.505, 0.0, 0.061), dark_steel)
    for y, idx in [(-0.067, 0), (0.067, 1)]:
        _box(elbow, f"guide_side_wall_{idx}", (0.250, 0.018, 0.140), (0.505, y, 0.0), dark_steel)
        _box(elbow, f"linear_bearing_strip_{idx}", (0.230, 0.006, 0.092), (0.505, y * 0.77, 0.0), bronze)
    _box(elbow, "rear_output_bumper", (0.028, 0.118, 0.014), (0.366, 0.0, 0.077), rubber)
    _box(elbow, "front_output_bumper", (0.028, 0.118, 0.014), (0.644, 0.0, 0.077), rubber)

    guide_cover = model.part("guide_cover")
    _box(guide_cover, "cover_plate", (0.165, 0.006, 0.102), (0.0, 0.0, 0.0), cover_blue)
    for sx in [-0.062, 0.062]:
        for sz in [-0.037, 0.037]:
            _cyl(guide_cover, "cover_screw", 0.006, 0.003, (sx, 0.0042, sz), (-math.pi / 2.0, 0.0, 0.0), dark_steel)

    output = model.part("output_stage")
    _box(output, "ram_body", (0.720, 0.048, 0.042), (0.160, 0.0, 0.0), bare_aluminum)
    _box(output, "top_wear_strip", (0.600, 0.030, 0.008), (0.140, 0.0, 0.025), rail_steel)
    _box(output, "bottom_wear_strip", (0.600, 0.030, 0.008), (0.140, 0.0, -0.025), rail_steel)
    for y, idx in [(-0.030, 0), (0.030, 1)]:
        _box(output, f"side_key_{idx}", (0.535, 0.012, 0.026), (0.120, y, 0.0), dark_steel)
    _box(output, "front_tool_plate", (0.046, 0.112, 0.100), (0.542, 0.0, 0.0), painted_steel)
    _cyl(output, "output_nose", 0.022, 0.090, (0.595, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0), rail_steel)
    _box(output, "rear_keeper", (0.028, 0.060, 0.076), (-0.205, 0.0, 0.0), dark_steel)

    model.articulation(
        "base_to_cover",
        ArticulationType.FIXED,
        parent=base,
        child=base_cover,
        origin=Origin(xyz=(0.120, 0.0, 0.103)),
    )
    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.380, 0.0, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=0.0, upper=0.520),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.2, lower=-0.80, upper=1.10),
    )
    model.articulation(
        "elbow_to_cover",
        ArticulationType.FIXED,
        parent=elbow,
        child=guide_cover,
        origin=Origin(xyz=(0.505, 0.079, 0.0)),
    )
    model.articulation(
        "output_slide",
        ArticulationType.PRISMATIC,
        parent=elbow,
        child=output,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.35, lower=0.0, upper=0.320),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    carriage = object_model.get_part("base_carriage")
    elbow = object_model.get_part("elbow")
    output = object_model.get_part("output_stage")
    base_slide = object_model.get_articulation("base_slide")
    elbow_joint = object_model.get_articulation("elbow_joint")
    output_slide = object_model.get_articulation("output_slide")

    ctx.allow_overlap(
        carriage,
        elbow,
        elem_a="axle_pin",
        elem_b="elbow_hub",
        reason="The exposed axle pin is intentionally captured through the rotary hub bore.",
    )
    ctx.expect_within(
        carriage,
        elbow,
        axes="xz",
        inner_elem="axle_pin",
        outer_elem="elbow_hub",
        margin=0.001,
        name="pin diameter stays inside elbow hub bore envelope",
    )
    ctx.expect_overlap(
        carriage,
        elbow,
        axes="y",
        elem_a="axle_pin",
        elem_b="elbow_hub",
        min_overlap=0.145,
        name="axle pin spans the elbow hub",
    )

    for shoe, rail in [("shoe_0", "rail_0"), ("shoe_1", "rail_1")]:
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem=shoe,
            negative_elem=rail,
            min_gap=0.0,
            max_gap=0.001,
            name=f"{shoe} rides on {rail}",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xy",
            elem_a=shoe,
            elem_b=rail,
            min_overlap=0.04,
            name=f"{shoe} remains captured on {rail}",
        )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: 0.520}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="shoe_0",
            elem_b="rail_0",
            min_overlap=0.20,
            name="base slide retains rail engagement at travel limit",
        )

    rest_output = ctx.part_world_position(output)
    with ctx.pose({output_slide: 0.320}):
        extended_output = ctx.part_world_position(output)
        ctx.expect_overlap(
            output,
            elbow,
            axes="x",
            elem_a="ram_body",
            elem_b="guide_upper_plate",
            min_overlap=0.075,
            name="output ram remains inserted in guide at travel limit",
        )

    with ctx.pose({elbow_joint: 0.85}):
        raised_output = ctx.part_world_position(output)

    ctx.check(
        "base prismatic stage moves along +X",
        rest_carriage is not None and extended_carriage is not None and extended_carriage[0] > rest_carriage[0] + 0.45,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "secondary prismatic stage moves along elbow +X",
        rest_output is not None and extended_output is not None and extended_output[0] > rest_output[0] + 0.30,
        details=f"rest={rest_output}, extended={extended_output}",
    )
    ctx.check(
        "revolute elbow lifts distal stage",
        rest_output is not None and raised_output is not None and raised_output[2] > rest_output[2] + 0.25,
        details=f"rest={rest_output}, raised={raised_output}",
    )

    return ctx.report()


object_model = build_object_model()
