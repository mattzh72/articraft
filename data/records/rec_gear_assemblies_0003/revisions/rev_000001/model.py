from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_Y = 0.100
FRAME_Z = 0.205
PLATE_THICKNESS = 0.012
PLATE_CENTER_X = 0.006
BASE_LENGTH = 0.090
BASE_WIDTH = 0.130
BASE_THICKNESS = 0.012
BASE_CENTER_X = -0.025
BOSS_RADIUS = 0.017
BOSS_LENGTH = 0.022
BOSS_CENTER_X = 0.017
BORE_RADIUS = 0.0072
SHAFT_RADIUS = 0.006
SHAFT_LENGTH = 0.122
SHAFT_CENTER_X = 0.055
COLLAR_RADIUS = 0.011
COLLAR_LENGTH = 0.008
COLLAR_CENTER_X = 0.032
GEAR_THICKNESS = 0.012
GEAR_CENTER_X = 0.050
HANDWHEEL_CENTER_X = 0.102
HANDWHEEL_THICKNESS = 0.010

INPUT_Z = 0.075
IDLER_Z = 0.129
OUTPUT_Z = 0.166

INPUT_GEAR = {
    "body_radius": 0.030,
    "tooth_depth": 0.004,
    "tooth_width": 0.0048,
    "tooth_count": 24,
    "hub_radius": 0.012,
    "hub_length": 0.018,
    "phase_deg": 90.0,
}
IDLER_GEAR = {
    "body_radius": 0.018,
    "tooth_depth": 0.0035,
    "tooth_width": 0.0040,
    "tooth_count": 16,
    "hub_radius": 0.010,
    "hub_length": 0.016,
    "phase_deg": -90.0 + (360.0 / 16.0) / 2.0,
}
OUTPUT_GEAR = {
    "body_radius": 0.014,
    "tooth_depth": 0.003,
    "tooth_width": 0.0038,
    "tooth_count": 12,
    "hub_radius": 0.009,
    "hub_length": 0.014,
    "phase_deg": -90.0,
}


def _x_cylinder(radius: float, length: float, center_x: float, center_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((center_x, 0.0, center_z))
    )


def _cq_mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0006,
        angular_tolerance=0.08,
    )


def _make_spur_gear(
    *,
    body_radius: float,
    tooth_depth: float,
    tooth_width: float,
    tooth_count: int,
    thickness: float,
    bore_radius: float,
    hub_radius: float,
    hub_length: float,
    phase_deg: float,
    center_x: float,
) -> cq.Workplane:
    gear = cq.Workplane("XY").circle(body_radius).extrude(thickness / 2.0, both=True)
    gear = gear.union(cq.Workplane("XY").circle(hub_radius).extrude(hub_length / 2.0, both=True))

    for index in range(tooth_count):
        angle_deg = 360.0 * index / tooth_count
        tooth = (
            cq.Workplane("XY")
            .box(tooth_width, tooth_depth, thickness, centered=(True, True, True))
            .translate((0.0, body_radius + tooth_depth / 2.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        gear = gear.union(tooth)

    web_cut_radius = body_radius * 0.43
    web_offset = (body_radius + hub_radius) * 0.45
    for angle_deg in (0.0, 120.0, 240.0):
        window = (
            cq.Workplane("XY")
            .circle(web_cut_radius)
            .extrude((thickness + 0.004) / 2.0, both=True)
            .translate((0.0, web_offset, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        gear = gear.cut(window)

    gear = gear.cut(cq.Workplane("XY").circle(bore_radius).extrude((hub_length + 0.006) / 2.0, both=True))
    gear = gear.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    gear = gear.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), phase_deg)
    return gear.translate((center_x, 0.0, 0.0))


def _make_handwheel(center_x: float) -> cq.Workplane:
    rim_outer = 0.030
    rim_inner = 0.024
    hub_radius = 0.0105
    ring = (
        cq.Workplane("XY")
        .circle(rim_outer)
        .circle(rim_inner)
        .extrude(HANDWHEEL_THICKNESS / 2.0, both=True)
    )
    hub = cq.Workplane("XY").circle(hub_radius).extrude(0.016 / 2.0, both=True)

    wheel = ring.union(hub)
    spoke_length = rim_inner - hub_radius + 0.001
    spoke_center_y = hub_radius + spoke_length / 2.0
    for angle_deg in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(0.005, spoke_length, HANDWHEEL_THICKNESS * 0.8, centered=(True, True, True))
            .translate((0.0, spoke_center_y, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        wheel = wheel.union(spoke)

    wheel = wheel.cut(cq.Workplane("XY").circle(SHAFT_RADIUS - 0.0001).extrude(0.020 / 2.0, both=True))
    wheel = wheel.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    return wheel.translate((center_x, 0.0, 0.0))


def _make_side_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_THICKNESS, FRAME_Y, FRAME_Z, centered=(True, True, False))
        .translate((PLATE_CENTER_X, 0.0, 0.0))
    )
    lightening_slot = (
        cq.Workplane("YZ")
        .center(0.0, 0.115)
        .slot2D(0.090, 0.032, angle=90.0)
        .extrude(PLATE_THICKNESS + 0.006, both=True)
        .translate((PLATE_CENTER_X, 0.0, 0.0))
    )
    plate = plate.cut(lightening_slot)

    for shaft_z in (INPUT_Z, IDLER_Z, OUTPUT_Z):
        plate = plate.cut(_x_cylinder(BORE_RADIUS, BOSS_LENGTH + 0.020, BOSS_CENTER_X, shaft_z))
    return plate


def _make_boss(center_z: float) -> cq.Workplane:
    boss = _x_cylinder(BOSS_RADIUS, BOSS_LENGTH, BOSS_CENTER_X, center_z)
    return boss.cut(_x_cylinder(BORE_RADIUS, BOSS_LENGTH + 0.006, BOSS_CENTER_X, center_z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_train_bench", assets=ASSETS)

    steel = model.material("steel", rgba=(0.46, 0.49, 0.53, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.61, 0.28, 1.0))
    iron = model.material("iron", rgba=(0.38, 0.39, 0.41, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))

    frame = model.part("frame")
    base_foot = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .translate((BASE_CENTER_X, 0.0, 0.0))
    )
    left_gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.048, BASE_THICKNESS), (0.004, BASE_THICKNESS), (0.004, 0.108)])
        .close()
        .extrude(0.014)
        .translate((0.0, -0.036, 0.0))
    )
    right_gusset = (
        cq.Workplane("XZ")
        .polyline([(-0.048, BASE_THICKNESS), (0.004, BASE_THICKNESS), (0.004, 0.108)])
        .close()
        .extrude(0.014)
        .translate((0.0, 0.022, 0.0))
    )

    frame.visual(_cq_mesh(base_foot, "frame_base_foot.obj"), material=dark_steel, name="base_foot")
    frame.visual(_cq_mesh(_make_side_plate(), "frame_side_plate.obj"), material=steel, name="side_plate")
    frame.visual(_cq_mesh(left_gusset, "frame_left_gusset.obj"), material=steel, name="left_gusset")
    frame.visual(_cq_mesh(right_gusset, "frame_right_gusset.obj"), material=steel, name="right_gusset")
    frame.visual(_cq_mesh(_make_boss(INPUT_Z), "frame_input_boss.obj"), material=iron, name="input_boss")
    frame.visual(_cq_mesh(_make_boss(IDLER_Z), "frame_idler_boss.obj"), material=iron, name="idler_boss")
    frame.visual(_cq_mesh(_make_boss(OUTPUT_Z), "frame_output_boss.obj"), material=iron, name="output_boss")
    frame.inertial = Inertial.from_geometry(
        Box((0.100, 0.130, 0.205)),
        mass=6.5,
        origin=Origin(xyz=(-0.005, 0.0, 0.085)),
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(xyz=(SHAFT_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="shaft_core",
    )
    input_shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="input_collar",
    )
    input_shaft.visual(
        _cq_mesh(
            _make_spur_gear(
                **INPUT_GEAR,
                thickness=GEAR_THICKNESS,
                bore_radius=SHAFT_RADIUS,
                center_x=GEAR_CENTER_X,
            ),
            "input_gear.obj",
        ),
        material=brass,
        name="input_gear",
    )
    input_shaft.visual(
        _cq_mesh(_make_handwheel(HANDWHEEL_CENTER_X), "input_handwheel.obj"),
        material=steel,
        name="handwheel",
    )
    input_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.118),
        mass=1.2,
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    idler_shaft = model.part("idler_shaft")
    idler_shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.094),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="shaft_core",
    )
    idler_shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="idler_collar",
    )
    idler_shaft.visual(
        _cq_mesh(
            _make_spur_gear(
                **IDLER_GEAR,
                thickness=GEAR_THICKNESS,
                bore_radius=SHAFT_RADIUS,
                center_x=GEAR_CENTER_X,
            ),
            "idler_gear.obj",
        ),
        material=steel,
        name="idler_gear",
    )
    idler_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.094),
        mass=0.6,
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.098),
        origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="shaft_core",
    )
    output_shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="output_collar",
    )
    output_shaft.visual(
        _cq_mesh(
            _make_spur_gear(
                **OUTPUT_GEAR,
                thickness=GEAR_THICKNESS,
                bore_radius=SHAFT_RADIUS,
                center_x=GEAR_CENTER_X,
            ),
            "output_gear.obj",
        ),
        material=brass,
        name="output_gear",
    )
    output_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.098),
        mass=0.45,
        origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    full_turn_limits = MotionLimits(
        effort=8.0,
        velocity=8.0,
        lower=-math.tau,
        upper=math.tau,
    )
    model.articulation(
        "frame_to_input",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=input_shaft,
        origin=Origin(xyz=(0.0, 0.0, INPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=full_turn_limits,
    )
    model.articulation(
        "frame_to_idler",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=idler_shaft,
        origin=Origin(xyz=(0.0, 0.0, IDLER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0, lower=-math.tau, upper=math.tau),
    )
    model.articulation(
        "frame_to_output",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(0.0, 0.0, OUTPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0, lower=-math.tau, upper=math.tau),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("frame_to_input")
    idler_joint = object_model.get_articulation("frame_to_idler")
    output_joint = object_model.get_articulation("frame_to_output")

    input_boss = frame.get_visual("input_boss")
    idler_boss = frame.get_visual("idler_boss")
    output_boss = frame.get_visual("output_boss")
    side_plate = frame.get_visual("side_plate")

    input_collar = input_shaft.get_visual("input_collar")
    idler_collar = idler_shaft.get_visual("idler_collar")
    output_collar = output_shaft.get_visual("output_collar")
    input_gear = input_shaft.get_visual("input_gear")
    idler_gear = idler_shaft.get_visual("idler_gear")
    output_gear = output_shaft.get_visual("output_gear")
    handwheel = input_shaft.get_visual("handwheel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(input_shaft, frame, elem_a=input_collar, elem_b=input_boss, name="input shaft collar seats on boss")
    ctx.expect_contact(idler_shaft, frame, elem_a=idler_collar, elem_b=idler_boss, name="idler shaft collar seats on boss")
    ctx.expect_contact(output_shaft, frame, elem_a=output_collar, elem_b=output_boss, name="output shaft collar seats on boss")

    ctx.expect_gap(
        input_shaft,
        frame,
        axis="x",
        positive_elem=input_gear,
        negative_elem=input_boss,
        min_gap=0.012,
        max_gap=0.028,
        name="input gear clears frame boss",
    )
    ctx.expect_gap(
        idler_shaft,
        frame,
        axis="x",
        positive_elem=idler_gear,
        negative_elem=idler_boss,
        min_gap=0.012,
        max_gap=0.028,
        name="idler gear clears frame boss",
    )
    ctx.expect_gap(
        output_shaft,
        frame,
        axis="x",
        positive_elem=output_gear,
        negative_elem=output_boss,
        min_gap=0.012,
        max_gap=0.028,
        name="output gear clears frame boss",
    )
    ctx.expect_gap(
        input_shaft,
        input_shaft,
        axis="x",
        positive_elem=handwheel,
        negative_elem=input_gear,
        min_gap=0.030,
        max_gap=0.060,
        name="handwheel sits outboard of input gear",
    )

    ctx.expect_origin_distance(input_shaft, idler_shaft, axes="y", max_dist=0.001, name="input and idler stay in one gear plane")
    ctx.expect_origin_distance(idler_shaft, output_shaft, axes="y", max_dist=0.001, name="idler and output stay in one gear plane")
    ctx.expect_origin_distance(
        input_shaft,
        idler_shaft,
        axes="z",
        min_dist=0.053,
        max_dist=0.055,
        name="input to idler center distance matches mesh spacing",
    )
    ctx.expect_origin_distance(
        idler_shaft,
        output_shaft,
        axes="z",
        min_dist=0.036,
        max_dist=0.038,
        name="idler to output center distance matches mesh spacing",
    )

    ctx.expect_overlap(
        input_shaft,
        idler_shaft,
        axes="yz",
        elem_a=input_gear,
        elem_b=idler_gear,
        min_overlap=0.0012,
        name="input and idler gears visibly overlap in projection",
    )
    ctx.expect_overlap(
        idler_shaft,
        output_shaft,
        axes="yz",
        elem_a=idler_gear,
        elem_b=output_gear,
        min_overlap=0.001,
        name="idler and output gears visibly overlap in projection",
    )
    ctx.expect_within(
        input_shaft,
        frame,
        axes="yz",
        inner_elem=input_collar,
        outer_elem=side_plate,
        margin=0.020,
        name="input bearing seat stays within side frame envelope",
    )
    ctx.expect_within(
        output_shaft,
        frame,
        axes="yz",
        inner_elem=output_collar,
        outer_elem=side_plate,
        margin=0.020,
        name="output bearing seat stays within side frame envelope",
    )

    with ctx.pose({input_joint: 1.1, idler_joint: -1.6, output_joint: 0.9}):
        ctx.expect_contact(
            input_shaft,
            frame,
            elem_a=input_collar,
            elem_b=input_boss,
            name="input shaft remains seated when rotated",
        )
        ctx.expect_contact(
            idler_shaft,
            frame,
            elem_a=idler_collar,
            elem_b=idler_boss,
            name="idler shaft remains seated when rotated",
        )
        ctx.expect_contact(
            output_shaft,
            frame,
            elem_a=output_collar,
            elem_b=output_boss,
            name="output shaft remains seated when rotated",
        )
        ctx.expect_gap(
            input_shaft,
            frame,
            axis="x",
            positive_elem=handwheel,
            negative_elem=input_boss,
            min_gap=0.060,
            max_gap=0.095,
            name="handwheel stays outside the frame during motion",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
