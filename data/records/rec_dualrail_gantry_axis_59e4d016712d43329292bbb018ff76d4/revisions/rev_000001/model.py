from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BED_LENGTH = 1.28
BED_WIDTH = 0.82
BED_BASE_THICKNESS = 0.09
BED_POCKET_DEPTH = 0.04
BED_RAIL_SUPPORT_LENGTH = 1.08
BED_RAIL_SUPPORT_WIDTH = 0.13
BED_RAIL_SUPPORT_HEIGHT = 0.19
BED_RAIL_Y = 0.305
BED_RAIL_LENGTH = 1.02
BED_RAIL_WIDTH = 0.04
BED_RAIL_HEIGHT = 0.018

BRIDGE_TRAVEL = 0.30
BRIDGE_SHOE_LENGTH = 0.18
BRIDGE_SHOE_WIDTH = 0.08
BRIDGE_SHOE_HEIGHT = 0.028
BRIDGE_TRUCK_BODY_WIDTH = 0.11
BRIDGE_TRUCK_BODY_HEIGHT = 0.068
BRIDGE_SIDE_PLATE_THICKNESS = 0.048
BRIDGE_SIDE_PLATE_HEIGHT = 0.155
BRIDGE_LEG_Y = 0.274
BRIDGE_BEAM_DEPTH = 0.14
BRIDGE_BEAM_LENGTH = 0.56
BRIDGE_BEAM_HEIGHT = 0.10
BRIDGE_BEAM_BOTTOM_Z = 0.176
BRIDGE_GUIDE_LENGTH = 0.48
BRIDGE_GUIDE_WIDTH = 0.028
BRIDGE_GUIDE_HEIGHT = 0.014
BRIDGE_GUIDE_X = 0.038

CARRIAGE_TRAVEL = 0.14
CARRIAGE_SHOE_DEPTH = 0.055
CARRIAGE_SHOE_LENGTH = 0.16
CARRIAGE_SHOE_HEIGHT = 0.022
CARRIAGE_BODY_DEPTH = 0.15
CARRIAGE_BODY_LENGTH = 0.16
CARRIAGE_BODY_HEIGHT = 0.058
CARRIAGE_TOP_DEPTH = 0.17
CARRIAGE_TOP_LENGTH = 0.13
CARRIAGE_TOP_HEIGHT = 0.085
OUTPUT_PLATE_THICKNESS = 0.018
OUTPUT_PLATE_WIDTH = 0.17
OUTPUT_PLATE_HEIGHT = 0.14

BED_RAIL_TOP_Z = BED_BASE_THICKNESS + BED_RAIL_SUPPORT_HEIGHT + BED_RAIL_HEIGHT
BRIDGE_GUIDE_TOP_Z = BRIDGE_BEAM_BOTTOM_Z + BRIDGE_BEAM_HEIGHT + BRIDGE_GUIDE_HEIGHT


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _machine_bed_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BED_LENGTH, BED_WIDTH, BED_BASE_THICKNESS)
        .translate((0.0, 0.0, BED_BASE_THICKNESS / 2.0))
    )

    pocket = (
        cq.Workplane("XY")
        .box(BED_LENGTH - 0.22, BED_WIDTH - 0.34, BED_POCKET_DEPTH)
        .translate((0.0, 0.0, BED_BASE_THICKNESS - BED_POCKET_DEPTH / 2.0))
    )

    left_support = (
        cq.Workplane("XY")
        .box(BED_RAIL_SUPPORT_LENGTH, BED_RAIL_SUPPORT_WIDTH, BED_RAIL_SUPPORT_HEIGHT)
        .translate(
            (
                0.0,
                -BED_RAIL_Y,
                BED_BASE_THICKNESS + BED_RAIL_SUPPORT_HEIGHT / 2.0,
            )
        )
    )
    right_support = (
        cq.Workplane("XY")
        .box(BED_RAIL_SUPPORT_LENGTH, BED_RAIL_SUPPORT_WIDTH, BED_RAIL_SUPPORT_HEIGHT)
        .translate(
            (
                0.0,
                BED_RAIL_Y,
                BED_BASE_THICKNESS + BED_RAIL_SUPPORT_HEIGHT / 2.0,
            )
        )
    )

    left_end_rib = (
        cq.Workplane("XY")
        .box(0.17, 0.60, 0.11)
        .translate((-0.47, 0.0, BED_BASE_THICKNESS + 0.055))
    )
    right_end_rib = (
        cq.Workplane("XY")
        .box(0.17, 0.60, 0.11)
        .translate((0.47, 0.0, BED_BASE_THICKNESS + 0.055))
    )

    center_floor = (
        cq.Workplane("XY")
        .box(0.86, 0.30, 0.03)
        .translate((0.0, 0.0, 0.015))
    )

    return (
        base.cut(pocket)
        .union(left_support)
        .union(right_support)
        .union(left_end_rib)
        .union(right_end_rib)
        .union(center_floor)
    )


def _bridge_shape() -> cq.Workplane:
    left_truck = (
        cq.Workplane("XY")
        .box(BRIDGE_SHOE_LENGTH, BRIDGE_TRUCK_BODY_WIDTH, BRIDGE_TRUCK_BODY_HEIGHT)
        .translate(
            (
                0.0,
                -BED_RAIL_Y,
                BRIDGE_SHOE_HEIGHT + BRIDGE_TRUCK_BODY_HEIGHT / 2.0,
            )
        )
    )
    right_truck = (
        cq.Workplane("XY")
        .box(BRIDGE_SHOE_LENGTH, BRIDGE_TRUCK_BODY_WIDTH, BRIDGE_TRUCK_BODY_HEIGHT)
        .translate(
            (
                0.0,
                BED_RAIL_Y,
                BRIDGE_SHOE_HEIGHT + BRIDGE_TRUCK_BODY_HEIGHT / 2.0,
            )
        )
    )

    left_leg = (
        cq.Workplane("XY")
        .box(0.12, BRIDGE_SIDE_PLATE_THICKNESS, BRIDGE_SIDE_PLATE_HEIGHT)
        .translate(
            (
                0.0,
                -BRIDGE_LEG_Y,
                BRIDGE_SHOE_HEIGHT
                + BRIDGE_TRUCK_BODY_HEIGHT
                + BRIDGE_SIDE_PLATE_HEIGHT / 2.0,
            )
        )
    )
    right_leg = (
        cq.Workplane("XY")
        .box(0.12, BRIDGE_SIDE_PLATE_THICKNESS, BRIDGE_SIDE_PLATE_HEIGHT)
        .translate(
            (
                0.0,
                BRIDGE_LEG_Y,
                BRIDGE_SHOE_HEIGHT
                + BRIDGE_TRUCK_BODY_HEIGHT
                + BRIDGE_SIDE_PLATE_HEIGHT / 2.0,
            )
        )
    )

    beam = (
        cq.Workplane("XY")
        .box(BRIDGE_BEAM_DEPTH, BRIDGE_BEAM_LENGTH, BRIDGE_BEAM_HEIGHT)
        .translate(
            (
                0.0,
                0.0,
                BRIDGE_BEAM_BOTTOM_Z + BRIDGE_BEAM_HEIGHT / 2.0,
            )
        )
    )

    front_stiffener = (
        cq.Workplane("XY")
        .box(0.06, 0.36, 0.07)
        .translate((0.04, 0.0, 0.146))
    )

    return (
        left_truck
        .union(right_truck)
        .union(left_leg)
        .union(right_leg)
        .union(beam)
        .union(front_stiffener)
    )


def _carriage_shape() -> cq.Workplane:
    lower_body = (
        cq.Workplane("XY")
        .box(CARRIAGE_BODY_DEPTH, CARRIAGE_BODY_LENGTH, CARRIAGE_BODY_HEIGHT)
        .translate(
            (
                0.028,
                0.0,
                CARRIAGE_SHOE_HEIGHT + CARRIAGE_BODY_HEIGHT / 2.0,
            )
        )
    )
    upper_body = (
        cq.Workplane("XY")
        .box(CARRIAGE_TOP_DEPTH, CARRIAGE_TOP_LENGTH, CARRIAGE_TOP_HEIGHT)
        .translate(
            (
                0.038,
                0.0,
                CARRIAGE_SHOE_HEIGHT
                + CARRIAGE_BODY_HEIGHT
                + CARRIAGE_TOP_HEIGHT / 2.0
                - 0.003,
            )
        )
    )
    relief = (
        cq.Workplane("XY")
        .box(0.09, 0.07, 0.028)
        .translate((0.0, 0.0, CARRIAGE_SHOE_HEIGHT + 0.014))
    )
    return lower_body.union(upper_body).cut(relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_bed_bridge_axis")

    model.material("cast_iron", rgba=(0.32, 0.35, 0.38, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("gantry_gray", rgba=(0.84, 0.85, 0.87, 1.0))
    model.material("carriage_white", rgba=(0.92, 0.93, 0.95, 1.0))
    model.material("plate_blue", rgba=(0.20, 0.40, 0.78, 1.0))

    bed = model.part("bed")
    bed.visual(
        mesh_from_cadquery(_machine_bed_shape(), "machine_bed_shell"),
        material="cast_iron",
        name="bed_shell",
    )
    _add_box(
        bed,
        (BED_RAIL_LENGTH, BED_RAIL_WIDTH, BED_RAIL_HEIGHT),
        (0.0, -BED_RAIL_Y, BED_BASE_THICKNESS + BED_RAIL_SUPPORT_HEIGHT + BED_RAIL_HEIGHT / 2.0),
        "rail_steel",
        name="left_rail_strip",
    )
    _add_box(
        bed,
        (BED_RAIL_LENGTH, BED_RAIL_WIDTH, BED_RAIL_HEIGHT),
        (0.0, BED_RAIL_Y, BED_BASE_THICKNESS + BED_RAIL_SUPPORT_HEIGHT + BED_RAIL_HEIGHT / 2.0),
        "rail_steel",
        name="right_rail_strip",
    )
    bed.inertial = Inertial.from_geometry(
        Box((BED_LENGTH, BED_WIDTH, 0.32)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_bridge_shape(), "bridge_shell"),
        material="gantry_gray",
        name="bridge_shell",
    )
    _add_box(
        bridge,
        (BRIDGE_SHOE_LENGTH, BRIDGE_SHOE_WIDTH, BRIDGE_SHOE_HEIGHT),
        (0.0, -BED_RAIL_Y, BRIDGE_SHOE_HEIGHT / 2.0),
        "rail_steel",
        name="left_truck_shoe",
    )
    _add_box(
        bridge,
        (BRIDGE_SHOE_LENGTH, BRIDGE_SHOE_WIDTH, BRIDGE_SHOE_HEIGHT),
        (0.0, BED_RAIL_Y, BRIDGE_SHOE_HEIGHT / 2.0),
        "rail_steel",
        name="right_truck_shoe",
    )
    _add_box(
        bridge,
        (BRIDGE_GUIDE_WIDTH, BRIDGE_GUIDE_LENGTH, BRIDGE_GUIDE_HEIGHT),
        (BRIDGE_GUIDE_X, 0.0, BRIDGE_BEAM_BOTTOM_Z + BRIDGE_BEAM_HEIGHT + BRIDGE_GUIDE_HEIGHT / 2.0),
        "rail_steel",
        name="front_beam_guide",
    )
    _add_box(
        bridge,
        (BRIDGE_GUIDE_WIDTH, BRIDGE_GUIDE_LENGTH, BRIDGE_GUIDE_HEIGHT),
        (-BRIDGE_GUIDE_X, 0.0, BRIDGE_BEAM_BOTTOM_Z + BRIDGE_BEAM_HEIGHT + BRIDGE_GUIDE_HEIGHT / 2.0),
        "rail_steel",
        name="rear_beam_guide",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.18, 0.72, 0.32)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_shell"),
        material="carriage_white",
        name="carriage_shell",
    )
    _add_box(
        carriage,
        (CARRIAGE_SHOE_DEPTH, CARRIAGE_SHOE_LENGTH, CARRIAGE_SHOE_HEIGHT),
        (BRIDGE_GUIDE_X, 0.0, CARRIAGE_SHOE_HEIGHT / 2.0),
        "rail_steel",
        name="front_carriage_shoe",
    )
    _add_box(
        carriage,
        (CARRIAGE_SHOE_DEPTH, CARRIAGE_SHOE_LENGTH, CARRIAGE_SHOE_HEIGHT),
        (-BRIDGE_GUIDE_X, 0.0, CARRIAGE_SHOE_HEIGHT / 2.0),
        "rail_steel",
        name="rear_carriage_shoe",
    )
    _add_box(
        carriage,
        (OUTPUT_PLATE_THICKNESS, OUTPUT_PLATE_WIDTH, OUTPUT_PLATE_HEIGHT),
        (0.128, 0.0, 0.02),
        "plate_blue",
        name="output_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.18)),
        mass=12.0,
        origin=Origin(xyz=(0.04, 0.0, 0.09)),
    )

    model.articulation(
        "bed_to_bridge",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, BED_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
            effort=1800.0,
            velocity=0.50,
        ),
    )
    model.articulation(
        "bridge_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_GUIDE_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
            effort=800.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    bed = object_model.get_part("bed")
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    bridge_axis = object_model.get_articulation("bed_to_bridge")
    carriage_axis = object_model.get_articulation("bridge_to_carriage")

    left_rail = bed.get_visual("left_rail_strip")
    right_rail = bed.get_visual("right_rail_strip")
    left_truck = bridge.get_visual("left_truck_shoe")
    right_truck = bridge.get_visual("right_truck_shoe")
    front_guide = bridge.get_visual("front_beam_guide")
    rear_guide = bridge.get_visual("rear_beam_guide")
    front_shoe = carriage.get_visual("front_carriage_shoe")
    rear_shoe = carriage.get_visual("rear_carriage_shoe")
    carriage.get_visual("output_plate")

    ctx.expect_contact(
        bridge,
        bed,
        elem_a=left_truck,
        elem_b=left_rail,
        name="left bridge truck sits on the left bed rail",
    )
    ctx.expect_contact(
        bridge,
        bed,
        elem_a=right_truck,
        elem_b=right_rail,
        name="right bridge truck sits on the right bed rail",
    )
    ctx.expect_contact(
        carriage,
        bridge,
        elem_a=front_shoe,
        elem_b=front_guide,
        name="front carriage shoe sits on the front beam guide",
    )
    ctx.expect_contact(
        carriage,
        bridge,
        elem_a=rear_shoe,
        elem_b=rear_guide,
        name="rear carriage shoe sits on the rear beam guide",
    )

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({bridge_axis: BRIDGE_TRAVEL}):
        ctx.expect_overlap(
            bridge,
            bed,
            axes="x",
            elem_a=left_truck,
            elem_b=left_rail,
            min_overlap=0.12,
            name="bridge retains left rail engagement at max travel",
        )
        ctx.expect_overlap(
            bridge,
            bed,
            axes="x",
            elem_a=right_truck,
            elem_b=right_rail,
            min_overlap=0.12,
            name="bridge retains right rail engagement at max travel",
        )
        bridge_extended = ctx.part_world_position(bridge)

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_axis: CARRIAGE_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            bridge,
            axes="y",
            elem_a=front_shoe,
            elem_b=front_guide,
            min_overlap=0.08,
            name="carriage retains front guide engagement at max travel",
        )
        ctx.expect_overlap(
            carriage,
            bridge,
            axes="y",
            elem_a=rear_shoe,
            elem_b=rear_guide,
            min_overlap=0.08,
            name="carriage retains rear guide engagement at max travel",
        )
        carriage_extended = ctx.part_world_position(carriage)

    ctx.check(
        "bridge moves along +X at its upper limit",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[0] > bridge_rest[0] + 0.10,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )
    ctx.check(
        "carriage moves along +Y at its upper limit",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[1] > carriage_rest[1] + 0.08,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
