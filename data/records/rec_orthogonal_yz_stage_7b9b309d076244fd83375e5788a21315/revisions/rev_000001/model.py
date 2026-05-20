from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _tri_gusset(name: str):
    """Right-triangle rib: local +X is stand-off, local +Y becomes world +Z."""
    profile = [(0.0, 0.0), (0.18, 0.0), (0.0, 0.25)]
    return mesh_from_geometry(ExtrudeGeometry(profile, 0.045, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_positioning_stage")

    painted_iron = _mat("dark painted cast iron", (0.10, 0.12, 0.13, 1.0))
    machined_steel = _mat("brushed linear rail steel", (0.68, 0.70, 0.70, 1.0))
    polished_steel = _mat("polished bearing ways", (0.86, 0.86, 0.82, 1.0))
    saddle_blue = _mat("blue anodized saddle", (0.08, 0.22, 0.42, 1.0))
    carriage_gray = _mat("matte carriage cover", (0.23, 0.25, 0.26, 1.0))
    rubber = _mat("black rubber bumpers", (0.01, 0.01, 0.012, 1.0))
    amber = _mat("amber stop hardware", (0.90, 0.54, 0.13, 1.0))

    # Fixed side-mounted plate.  The plate is a tall vertical wall in the Y-Z
    # plane; +X is the machine stand-off direction, Y is the crosswise axis,
    # and Z is vertical.
    backplate = model.part("backplate")
    backplate.visual(
        Box((0.080, 1.300, 1.600)),
        origin=Origin(xyz=(0.000, 0.000, 0.800)),
        material=painted_iron,
        name="wall_plate",
    )
    backplate.visual(
        Box((0.125, 1.360, 0.045)),
        origin=Origin(xyz=(0.015, 0.000, 1.595)),
        material=painted_iron,
        name="top_cap",
    )
    backplate.visual(
        Box((0.125, 1.360, 0.045)),
        origin=Origin(xyz=(0.015, 0.000, 0.025)),
        material=painted_iron,
        name="bottom_cap",
    )
    for side, y in (("neg", -0.670), ("pos", 0.670)):
        backplate.visual(
            Box((0.120, 0.050, 1.560)),
            origin=Origin(xyz=(0.020, y, 0.805)),
            material=painted_iron,
            name=f"{side}_side_flange",
        )

    # Crosswise Y-axis linear guide rail set, bolted directly into the plate.
    for label, z in (("upper", 1.120), ("lower", 0.820)):
        backplate.visual(
            Box((0.038, 1.120, 0.080)),
            origin=Origin(xyz=(0.055, 0.000, z)),
            material=painted_iron,
            name=f"{label}_rail_backer",
        )
        backplate.visual(
            Box((0.045, 1.070, 0.038)),
            origin=Origin(xyz=(0.084, 0.000, z)),
            material=machined_steel,
            name=f"{label}_rail",
        )
        backplate.visual(
            Box((0.009, 1.020, 0.022)),
            origin=Origin(xyz=(0.1085, 0.000, z)),
            material=polished_steel,
            name=f"{label}_rail_runner",
        )
        # Low-profile counterbored bolt heads sit behind the bearing clearance.
        for i, y in enumerate((-0.430, -0.220, 0.000, 0.220, 0.430)):
            backplate.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(0.107, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=machined_steel,
                name=f"{label}_rail_bolt_{i}",
            )

    # Fixed end stops for the horizontal Y axis.  They are carried on the plate,
    # not on a tabletop, and sit just outside the usable saddle travel.
    for side, y in (("neg", -0.529), ("pos", 0.529)):
        backplate.visual(
            Box((0.155, 0.025, 0.050)),
            origin=Origin(xyz=(0.114, y, 0.970)),
            material=amber,
            name=f"{side}_y_stop_arm",
        )
        backplate.visual(
            Box((0.105, 0.055, 0.135)),
            origin=Origin(xyz=(0.240, y, 0.970)),
            material=amber,
            name=f"{side}_y_stop",
        )
        backplate.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.296, y, 0.970), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"{side}_y_bumper_face",
        )

    # Webbed triangular ribs keep the rail shelf visibly tied into the tall
    # backplate instead of reading as floating bars.
    gusset_mesh = _tri_gusset("backplate_gusset")
    for i, y in enumerate((-0.600, -0.525, 0.525, 0.600)):
        backplate.visual(
            gusset_mesh,
            origin=Origin(xyz=(0.035, y, 0.590), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_iron,
            name=f"lower_gusset_{i}",
        )

    # Moving horizontal saddle.  Its part frame is the center of the Y slide
    # truck; at q=0 it sits near the negative-Y stop and moves toward +Y.
    saddle = model.part("saddle")
    saddle.visual(
        Box((0.070, 0.380, 0.390)),
        origin=Origin(xyz=(0.050, 0.000, 0.000)),
        material=saddle_blue,
        name="cross_saddle_casting",
    )
    for label, z in (("upper", 0.150), ("lower", -0.150)):
        saddle.visual(
            Box((0.070, 0.265, 0.082)),
            origin=Origin(xyz=(0.016, 0.000, z)),
            material=saddle_blue,
            name=f"{label}_bearing_truck",
        )
        saddle.visual(
            Box((0.009, 0.245, 0.052)),
            origin=Origin(xyz=(-0.0225, 0.000, z)),
            material=polished_steel,
            name=f"{label}_bearing_face",
        )
    for side, y in (("neg", -0.205), ("pos", 0.205)):
        saddle.visual(
            Box((0.055, 0.038, 0.086)),
            origin=Origin(xyz=(0.105, y, 0.000)),
            material=rubber,
            name=f"{side}_saddle_bumper",
        )

    # Downward Z-axis spine and rails mounted on the saddle casting.  This turns
    # the moving cross saddle into a side-mounted hanging axis carrier.
    saddle.visual(
        Box((0.094, 0.205, 0.640)),
        origin=Origin(xyz=(0.112, 0.000, -0.355)),
        material=saddle_blue,
        name="z_spine",
    )
    saddle.visual(
        Box((0.040, 0.315, 0.520)),
        origin=Origin(xyz=(0.095, 0.000, -0.405)),
        material=saddle_blue,
        name="z_back_web",
    )
    for side, y in (("neg", -0.078), ("pos", 0.078)):
        saddle.visual(
            Box((0.035, 0.035, 0.620)),
            origin=Origin(xyz=(0.177, y, -0.390)),
            material=machined_steel,
            name=f"{side}_z_rail",
        )
        saddle.visual(
            Box((0.008, 0.026, 0.580)),
            origin=Origin(xyz=(0.1985, y, -0.390)),
            material=polished_steel,
            name=f"{side}_z_rail_face",
        )
    saddle.visual(
        Box((0.030, 0.055, 0.560)),
        origin=Origin(xyz=(0.150, 0.000, -0.390)),
        material=painted_iron,
        name="z_screw_cover",
    )

    # Z-axis hard stops and their web brackets, placed outside the carriage
    # sweep envelope so the carriage can travel without clipping them.
    for side, y in (("neg", -0.165), ("pos", 0.165)):
        saddle.visual(
            Box((0.078, 0.080, 0.045)),
            origin=Origin(xyz=(0.160, y * 0.82, -0.115)),
            material=amber,
            name=f"{side}_top_stop_bracket",
        )
        saddle.visual(
            Box((0.048, 0.045, 0.060)),
            origin=Origin(xyz=(0.219, y, -0.115)),
            material=rubber,
            name=f"{side}_top_z_stop",
        )
        saddle.visual(
            Box((0.078, 0.080, 0.045)),
            origin=Origin(xyz=(0.160, y * 0.82, -0.700)),
            material=amber,
            name=f"{side}_bottom_stop_bracket",
        )
        saddle.visual(
            Box((0.048, 0.045, 0.060)),
            origin=Origin(xyz=(0.219, y, -0.700)),
            material=rubber,
            name=f"{side}_bottom_z_stop",
        )

    y_axis = model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=saddle,
        origin=Origin(xyz=(0.140, -0.260, 0.970)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.520),
    )
    y_axis.meta["description"] = "Horizontal crosswise Y slide on the wall plate."

    # Hanging vertical carriage.  It rides in front of the Z rails and moves
    # downward on a prismatic Z axis without entering the rail, cover, or stop
    # volumes.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.086, 0.260, 0.320)),
        origin=Origin(xyz=(0.050, 0.000, -0.220)),
        material=carriage_gray,
        name="carriage_cover",
    )
    for side, y in (("neg", -0.078), ("pos", 0.078)):
        carriage.visual(
            Box((0.018, 0.045, 0.285)),
            origin=Origin(xyz=(0.0065, y, -0.220)),
            material=polished_steel,
            name=f"{side}_rear_shoe",
        )
    carriage.visual(
        Box((0.036, 0.205, 0.160)),
        origin=Origin(xyz=(0.107, 0.000, -0.300)),
        material=machined_steel,
        name="tooling_plate",
    )
    carriage.visual(
        Box((0.052, 0.142, 0.090)),
        origin=Origin(xyz=(0.084, 0.000, -0.385)),
        material=machined_steel,
        name="lower_tool_block",
    )
    carriage.visual(
        Box((0.052, 0.030, 0.050)),
        origin=Origin(xyz=(0.086, 0.140, -0.120)),
        material=amber,
        name="top_striker",
    )
    carriage.visual(
        Box((0.052, 0.030, 0.050)),
        origin=Origin(xyz=(0.086, -0.140, -0.395)),
        material=amber,
        name="bottom_striker",
    )

    z_axis = model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=carriage,
        origin=Origin(xyz=(0.205, 0.000, -0.080)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.340),
    )
    z_axis.meta["description"] = "Vertical hanging carriage Z slide."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    saddle = object_model.get_part("saddle")
    carriage = object_model.get_part("carriage")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")

    with ctx.pose({y_axis: 0.0, z_axis: 0.0}):
        ctx.expect_gap(
            saddle,
            backplate,
            axis="x",
            positive_elem="upper_bearing_face",
            negative_elem="upper_rail_runner",
            min_gap=-0.0005,
            max_gap=0.003,
            name="upper Y bearing face runs clear of rail",
        )
        ctx.expect_gap(
            saddle,
            backplate,
            axis="y",
            positive_elem="neg_saddle_bumper",
            negative_elem="neg_y_stop",
            min_gap=0.015,
            name="negative Y stop has approach clearance",
        )
        ctx.expect_gap(
            carriage,
            saddle,
            axis="x",
            positive_elem="neg_rear_shoe",
            negative_elem="neg_z_rail_face",
            min_gap=-0.0005,
            max_gap=0.004,
            name="Z rear shoe runs clear of rail face",
        )
        ctx.expect_overlap(
            carriage,
            saddle,
            axes="yz",
            elem_a="neg_rear_shoe",
            elem_b="neg_z_rail",
            min_overlap=0.025,
            name="upper Z shoe remains captured on rail",
        )
        ctx.expect_gap(
            carriage,
            saddle,
            axis="y",
            positive_elem="carriage_cover",
            negative_elem="neg_top_z_stop",
            min_gap=0.010,
            name="upper Z stop clears carriage cover side",
        )

    rest_y = ctx.part_world_position(saddle)
    rest_z = ctx.part_world_position(carriage)
    with ctx.pose({y_axis: 0.520, z_axis: 0.340}):
        ctx.expect_within(
            saddle,
            backplate,
            axes="y",
            inner_elem="cross_saddle_casting",
            outer_elem="upper_rail",
            margin=0.020,
            name="saddle stays within fixed Y rail length",
        )
        ctx.expect_gap(
            backplate,
            saddle,
            axis="y",
            positive_elem="pos_y_stop",
            negative_elem="pos_saddle_bumper",
            min_gap=0.015,
            name="positive Y stop has approach clearance",
        )
        ctx.expect_gap(
            carriage,
            saddle,
            axis="x",
            positive_elem="pos_rear_shoe",
            negative_elem="pos_z_rail_face",
            min_gap=-0.0005,
            max_gap=0.004,
            name="lowered Z shoe runs clear of rail face",
        )
        ctx.expect_overlap(
            carriage,
            saddle,
            axes="yz",
            elem_a="pos_rear_shoe",
            elem_b="pos_z_rail",
            min_overlap=0.025,
            name="lowered Z shoe retains rail engagement",
        )
        ctx.expect_gap(
            carriage,
            saddle,
            axis="x",
            positive_elem="neg_rear_shoe",
            negative_elem="neg_bottom_stop_bracket",
            min_gap=0.002,
            name="lower Z bracket clears negative shoe",
        )
        ctx.expect_gap(
            carriage,
            saddle,
            axis="x",
            positive_elem="pos_rear_shoe",
            negative_elem="pos_bottom_stop_bracket",
            min_gap=0.002,
            name="lower Z bracket clears positive shoe",
        )
        end_y = ctx.part_world_position(saddle)
        end_z = ctx.part_world_position(carriage)

    ctx.check(
        "Y axis moves saddle left to right",
        rest_y is not None and end_y is not None and end_y[1] > rest_y[1] + 0.45,
        details=f"rest={rest_y}, end={end_y}",
    )
    ctx.check(
        "Z axis lowers the hanging carriage",
        rest_z is not None and end_z is not None and end_z[2] < rest_z[2] - 0.30,
        details=f"rest={rest_z}, end={end_z}",
    )

    return ctx.report()


object_model = build_object_model()
