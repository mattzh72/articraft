from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_DEPTH = 0.20
SIDE_PLATE_THICKNESS = 0.012
SIDE_PLATE_WIDTH = 0.18
SIDE_PLATE_HEIGHT = 0.50
SIDE_PLATE_BASE_Z = 0.04

SHAFT_RADIUS = 0.007
SHAFT_LENGTH = FRAME_DEPTH + 0.050
SHAFT_CENTER_Y = FRAME_DEPTH / 2.0

GEAR_MODULE = 0.0035
PINION_TEETH = 16
GEAR_TEETH = 40
GEAR_WIDTH = 0.016
GEAR_BORE = 0.010

BEARING_HOLE_RADIUS = 0.010
BOSS_RADIUS = 0.020
BOSS_PROTRUSION = 0.014
BOSS_FUSE_DEPTH = 0.002
BOSS_LENGTH = BOSS_PROTRUSION + BOSS_FUSE_DEPTH
GEAR_CENTER_OFFSET = GEAR_WIDTH / 2.0
COLLAR_RADIUS = 0.014
COLLAR_WIDTH = 0.006
LEFT_BOSS_OUTER_FACE_Y = -(SIDE_PLATE_THICKNESS / 2.0 + BOSS_PROTRUSION)
RIGHT_BOSS_OUTER_FACE_Y = FRAME_DEPTH + SIDE_PLATE_THICKNESS / 2.0 + BOSS_PROTRUSION
LEFT_COLLAR_CENTER_Y = LEFT_BOSS_OUTER_FACE_Y - COLLAR_WIDTH / 2.0
RIGHT_COLLAR_CENTER_Y = RIGHT_BOSS_OUTER_FACE_Y + COLLAR_WIDTH / 2.0

SHAFT_PITCH = 0.109
BOTTOM_SHAFT_Z = 0.130
SHAFT_ZS = [
    BOTTOM_SHAFT_Z + 3.0 * SHAFT_PITCH,
    BOTTOM_SHAFT_Z + 2.0 * SHAFT_PITCH,
    BOTTOM_SHAFT_Z + 1.0 * SHAFT_PITCH,
    BOTTOM_SHAFT_Z,
]

STAGE_Y = {
    "stage_1": 0.050,
    "stage_2": 0.100,
    "stage_3": 0.150,
}


def _make_side_plate(*, y_center: float, outboard_sign: float) -> cq.Workplane:
    plate_center_z = SIDE_PLATE_BASE_Z + SIDE_PLATE_HEIGHT / 2.0
    plate = (
        cq.Workplane("XY")
        .box(SIDE_PLATE_WIDTH, SIDE_PLATE_THICKNESS, SIDE_PLATE_HEIGHT)
        .translate((0.0, y_center, plate_center_z))
        .edges("|Y")
        .fillet(0.018)
    )

    total_cut_depth = SIDE_PLATE_THICKNESS + 2.0 * BOSS_LENGTH + 0.010
    slot_center_y = y_center - total_cut_depth / 2.0
    slot_cut = (
        cq.Workplane("XZ")
        .pushPoints([(-0.045, plate_center_z), (0.045, plate_center_z)])
        .slot2D(0.28, 0.032, angle=90)
        .extrude(total_cut_depth)
        .translate((0.0, slot_center_y, 0.0))
    )

    boss_center_y = y_center + outboard_sign * (
        SIDE_PLATE_THICKNESS / 2.0 + (BOSS_PROTRUSION - BOSS_FUSE_DEPTH) / 2.0
    )
    side = plate
    for z in SHAFT_ZS:
        boss = cq.Workplane(
            obj=cq.Solid.makeCylinder(
                BOSS_RADIUS,
                BOSS_LENGTH,
                cq.Vector(0.0, boss_center_y - BOSS_LENGTH / 2.0, z),
                cq.Vector(0.0, 1.0, 0.0),
            )
        )
        side = side.union(boss)

    for z in SHAFT_ZS:
        shaft_hole = cq.Workplane(
            obj=cq.Solid.makeCylinder(
                BEARING_HOLE_RADIUS,
                total_cut_depth,
                cq.Vector(0.0, y_center - total_cut_depth / 2.0, z),
                cq.Vector(0.0, 1.0, 0.0),
            )
        )
        side = side.cut(shaft_hole)

    return side.cut(slot_cut)


def _make_frame_shape() -> cq.Shape:
    left_plate = _make_side_plate(y_center=0.0, outboard_sign=-1.0)
    right_plate = _make_side_plate(y_center=FRAME_DEPTH, outboard_sign=1.0)

    rail_span = FRAME_DEPTH + SIDE_PLATE_THICKNESS + 0.012
    left_skid = (
        cq.Workplane("XY")
        .box(0.032, rail_span, 0.040)
        .translate((-0.055, FRAME_DEPTH / 2.0, 0.020))
    )
    right_skid = (
        cq.Workplane("XY")
        .box(0.032, rail_span, 0.040)
        .translate((0.055, FRAME_DEPTH / 2.0, 0.020))
    )
    top_tie = (
        cq.Workplane("XY")
        .box(0.045, rail_span, 0.028)
        .translate((0.0, FRAME_DEPTH / 2.0, SIDE_PLATE_BASE_Z + SIDE_PLATE_HEIGHT - 0.006))
    )

    return (
        left_plate.union(right_plate)
        .union(left_skid)
        .union(right_skid)
        .union(top_tie)
        .val()
    )


def _gear_mesh(teeth: int, name: str):
    gear = SpurGear(
        module=GEAR_MODULE,
        teeth_number=teeth,
        width=GEAR_WIDTH,
        bore_d=GEAR_BORE,
    )
    shape = cq.Workplane("XZ").gear(gear).val()
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.06)


def _add_shaft_part(
    model: ArticulatedObject,
    *,
    shaft_index: int,
    shaft_z: float,
    pinion_mesh,
    large_gear_mesh,
    shaft_material: str,
    gear_material: str,
) -> None:
    shaft = model.part(f"shaft_{shaft_index}")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, SHAFT_CENTER_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_WIDTH),
        origin=Origin(xyz=(0.0, LEFT_COLLAR_CENTER_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="left_collar",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_WIDTH),
        origin=Origin(xyz=(0.0, RIGHT_COLLAR_CENTER_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="right_collar",
    )

    if shaft_index == 1:
        shaft.visual(
            pinion_mesh,
            origin=Origin(xyz=(0.0, STAGE_Y["stage_1"] + GEAR_CENTER_OFFSET, 0.0)),
            material=gear_material,
            name="input_pinion",
        )
    elif shaft_index == 2:
        shaft.visual(
            large_gear_mesh,
            origin=Origin(xyz=(0.0, STAGE_Y["stage_1"] + GEAR_CENTER_OFFSET, 0.0)),
            material=gear_material,
            name="stage1_gear",
        )
        shaft.visual(
            pinion_mesh,
            origin=Origin(xyz=(0.0, STAGE_Y["stage_2"] + GEAR_CENTER_OFFSET, 0.0)),
            material=gear_material,
            name="stage2_pinion",
        )
    elif shaft_index == 3:
        shaft.visual(
            large_gear_mesh,
            origin=Origin(xyz=(0.0, STAGE_Y["stage_2"] + GEAR_CENTER_OFFSET, 0.0)),
            material=gear_material,
            name="stage2_gear",
        )
        shaft.visual(
            pinion_mesh,
            origin=Origin(xyz=(0.0, STAGE_Y["stage_3"] + GEAR_CENTER_OFFSET, 0.0)),
            material=gear_material,
            name="stage3_pinion",
        )
    elif shaft_index == 4:
        shaft.visual(
            large_gear_mesh,
            origin=Origin(xyz=(0.0, STAGE_Y["stage_3"] + GEAR_CENTER_OFFSET, 0.0)),
            material=gear_material,
            name="stage3_gear",
        )

    frame = model.get_part("frame")
    model.articulation(
        f"frame_to_shaft_{shaft_index}",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=20.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_reduction_gear_bench")

    frame_blue = model.material("frame_blue", rgba=(0.18, 0.27, 0.36, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.58, 0.62, 1.0))
    gear_bronze = model.material("gear_bronze", rgba=(0.70, 0.57, 0.30, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_shape(), "reduction_bench_frame", tolerance=0.0008, angular_tolerance=0.08),
        material=frame_blue,
        name="frame_body",
    )

    pinion_mesh = _gear_mesh(PINION_TEETH, "pinion_16t")
    large_gear_mesh = _gear_mesh(GEAR_TEETH, "gear_40t")

    _add_shaft_part(
        model,
        shaft_index=1,
        shaft_z=SHAFT_ZS[0],
        pinion_mesh=pinion_mesh,
        large_gear_mesh=large_gear_mesh,
        shaft_material=steel,
        gear_material=gear_bronze,
    )
    _add_shaft_part(
        model,
        shaft_index=2,
        shaft_z=SHAFT_ZS[1],
        pinion_mesh=pinion_mesh,
        large_gear_mesh=large_gear_mesh,
        shaft_material=steel,
        gear_material=gear_bronze,
    )
    _add_shaft_part(
        model,
        shaft_index=3,
        shaft_z=SHAFT_ZS[2],
        pinion_mesh=pinion_mesh,
        large_gear_mesh=large_gear_mesh,
        shaft_material=steel,
        gear_material=gear_bronze,
    )
    _add_shaft_part(
        model,
        shaft_index=4,
        shaft_z=SHAFT_ZS[3],
        pinion_mesh=pinion_mesh,
        large_gear_mesh=large_gear_mesh,
        shaft_material=steel,
        gear_material=gear_bronze,
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

    frame = object_model.get_part("frame")
    shafts = [object_model.get_part(f"shaft_{index}") for index in range(1, 5)]
    joints = [object_model.get_articulation(f"frame_to_shaft_{index}") for index in range(1, 5)]

    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.012,
        name="shaft revolute origins sit on the supported shaft axes",
    )

    axis_ok = all(
        joint.articulation_type == ArticulationType.REVOLUTE
        and all(isclose(value, expected, abs_tol=1e-9) for value, expected in zip(joint.axis, (0.0, 1.0, 0.0)))
        for joint in joints
    )
    ctx.check(
        "all four shafts use independent revolute joints about +Y",
        axis_ok,
        details=str([(joint.name, joint.articulation_type, joint.axis) for joint in joints]),
    )

    shaft_positions = [ctx.part_world_position(shaft) for shaft in shafts]
    spacing_checks = []
    for lower, upper in zip(shaft_positions[:-1], shaft_positions[1:]):
        if lower is None or upper is None:
            spacing_checks.append(False)
        else:
            spacing_checks.append(isclose(lower[2] - upper[2], SHAFT_PITCH, abs_tol=1e-4))

    ctx.check(
        "shaft centers form an evenly stepped reduction stack",
        all(spacing_checks)
        and all(pos is not None and isclose(pos[0], 0.0, abs_tol=1e-6) and isclose(pos[1], 0.0, abs_tol=1e-6) for pos in shaft_positions),
        details=str(shaft_positions),
    )

    ctx.expect_origin_distance(
        shafts[0],
        shafts[1],
        axes="z",
        min_dist=SHAFT_PITCH - 0.001,
        max_dist=SHAFT_PITCH + 0.001,
        name="shaft 1 and shaft 2 keep the first reduction spacing",
    )
    ctx.expect_origin_distance(
        shafts[1],
        shafts[2],
        axes="z",
        min_dist=SHAFT_PITCH - 0.001,
        max_dist=SHAFT_PITCH + 0.001,
        name="shaft 2 and shaft 3 keep the second reduction spacing",
    )
    ctx.expect_origin_distance(
        shafts[2],
        shafts[3],
        axes="z",
        min_dist=SHAFT_PITCH - 0.001,
        max_dist=SHAFT_PITCH + 0.001,
        name="shaft 3 and shaft 4 keep the third reduction spacing",
    )

    ctx.expect_overlap(
        shafts[0],
        shafts[1],
        axes="y",
        elem_a="input_pinion",
        elem_b="stage1_gear",
        min_overlap=0.014,
        name="stage 1 gears share an axial mesh plane",
    )
    ctx.expect_overlap(
        shafts[1],
        shafts[2],
        axes="y",
        elem_a="stage2_pinion",
        elem_b="stage2_gear",
        min_overlap=0.014,
        name="stage 2 gears share an axial mesh plane",
    )
    ctx.expect_overlap(
        shafts[2],
        shafts[3],
        axes="y",
        elem_a="stage3_pinion",
        elem_b="stage3_gear",
        min_overlap=0.014,
        name="stage 3 gears share an axial mesh plane",
    )

    with ctx.pose({joints[0]: 1.2, joints[1]: -0.9, joints[2]: 0.7, joints[3]: -1.1}):
        posed_positions = [ctx.part_world_position(shaft) for shaft in shafts]
    ctx.check(
        "shaft revolutes spin in place without drifting off their bearings",
        shaft_positions == posed_positions,
        details=f"rest={shaft_positions}, posed={posed_positions}",
    )

    ctx.expect_origin_gap(
        shafts[0],
        frame,
        axis="z",
        min_gap=-0.01,
        max_gap=0.50,
        name="top shaft sits within the height envelope of the frame",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
