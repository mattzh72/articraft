from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.34
BASE_DEPTH = 0.18
BASE_THK = 0.016

WORM_SHAFT_D = 0.018
SPINDLE_D = 0.020
OUTPUT_SHAFT_D = 0.018

WHEEL_MODULE = 0.004
WHEEL_TEETH = 28
WHEEL_WIDTH = 0.022
WHEEL_OUTER_R = WHEEL_MODULE * (WHEEL_TEETH + 2) / 2.0

PINION_MODULE = 0.004
PINION_TEETH = 12
PINION_WIDTH = 0.014
PINION_OUTER_R = PINION_MODULE * (PINION_TEETH + 2) / 2.0

OUTPUT_MODULE = 0.004
OUTPUT_TEETH = 18
OUTPUT_WIDTH = 0.018
OUTPUT_OUTER_R = OUTPUT_MODULE * (OUTPUT_TEETH + 2) / 2.0

WORM_OUTER_R = 0.013
WORM_LENGTH = 0.066
WORM_Y = -(WHEEL_OUTER_R + WORM_OUTER_R + 0.002)
WORM_BLOCK_X = 0.105
WORM_BLOCK_LEN = 0.044

WHEEL_Z = 0.096
UPPER_GEAR_Z = 0.166
OUTPUT_X = PINION_OUTER_R + OUTPUT_OUTER_R + 0.002

BRIDGE_THK = 0.018
BRIDGE_CENTER_Z = 0.218


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(*size, centered=(True, True, True))
        .translate(center)
    )


def cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((cx - length / 2.0, cy, cz))


def cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def union_all(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def rotate_z(solid: cq.Workplane, angle_deg: float) -> cq.Workplane:
    return solid.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)


def make_spur_gear(
    *,
    module: float,
    teeth: int,
    width: float,
    bore_d: float,
    hub_radius: float,
    hub_width: float,
    spoke_count: int,
    spoke_width: float,
) -> cq.Workplane:
    pitch_r = module * teeth / 2.0
    outer_r = module * (teeth + 2) / 2.0
    root_r = max(pitch_r - 1.25 * module, outer_r - 0.008)
    tooth_depth = outer_r - root_r + 0.0008
    tooth_pitch = 2.0 * pi * (root_r + tooth_depth * 0.45) / teeth
    tooth_width = tooth_pitch * 0.52

    rim = cylinder_z(root_r, width, (0.0, 0.0, 0.0))
    hub = cylinder_z(hub_radius, hub_width, (0.0, 0.0, 0.0))
    tooth = box_at(
        (tooth_depth, tooth_width, width),
        (root_r + tooth_depth / 2.0 - 0.0002, 0.0, 0.0),
    )

    solids: list[cq.Workplane] = [rim, hub]
    if spoke_count > 0:
        spoke_length = max(root_r - hub_radius - 0.004, 0.004)
        spoke = box_at(
            (spoke_length, spoke_width, width * 0.82),
            ((hub_radius + root_r) / 2.0 - 0.002, 0.0, 0.0),
        )
        for index in range(spoke_count):
            solids.append(rotate_z(spoke, 360.0 * index / spoke_count))

    for index in range(teeth):
        solids.append(rotate_z(tooth, 360.0 * index / teeth))

    gear = union_all(solids)
    if bore_d > 0.0:
        gear = gear.cut(cylinder_z(bore_d / 2.0, max(width, hub_width) + 0.004, (0.0, 0.0, 0.0)))
    return gear


def make_worm_thread() -> cq.Workplane:
    root_radius = 0.0105
    crest_height = WORM_OUTER_R - root_radius
    crest_width = 0.010
    twist_turns = 3.5

    root = cylinder_x(root_radius, WORM_LENGTH, (0.0, 0.0, 0.0))
    start_crest = (
        cq.Workplane("YZ")
        .center(0.0, root_radius + crest_height / 2.0)
        .rect(crest_width, crest_height)
        .twistExtrude(WORM_LENGTH, 360.0 * twist_turns)
        .translate((-WORM_LENGTH / 2.0, 0.0, 0.0))
    )
    second_crest = start_crest.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 180.0)
    return union_all([root, start_crest, second_crest])


def make_frame_bench() -> cq.Workplane:
    base = box_at((BASE_LEN, BASE_DEPTH, BASE_THK), (0.0, 0.0, BASE_THK / 2.0))
    front_rail = box_at((0.240, 0.022, 0.020), (0.0, WORM_Y, 0.026))
    rear_rail = box_at((0.246, 0.022, 0.020), (0.020, 0.074, 0.026))

    left_worm_block = union_all(
        [
            box_at((0.066, 0.056, 0.014), (-WORM_BLOCK_X, WORM_Y, 0.023)),
            box_at((0.048, 0.040, 0.048), (-WORM_BLOCK_X, WORM_Y, 0.060)),
            box_at((0.010, 0.044, 0.028), (-WORM_BLOCK_X - 0.018, WORM_Y, 0.096)),
            box_at((0.010, 0.044, 0.028), (-WORM_BLOCK_X + 0.018, WORM_Y, 0.096)),
            box_at((0.052, 0.044, 0.010), (-WORM_BLOCK_X, WORM_Y, 0.115)),
        ]
    )
    right_worm_block = union_all(
        [
            box_at((0.066, 0.056, 0.014), (WORM_BLOCK_X, WORM_Y, 0.023)),
            box_at((0.048, 0.040, 0.048), (WORM_BLOCK_X, WORM_Y, 0.060)),
            box_at((0.010, 0.044, 0.028), (WORM_BLOCK_X - 0.018, WORM_Y, 0.096)),
            box_at((0.010, 0.044, 0.028), (WORM_BLOCK_X + 0.018, WORM_Y, 0.096)),
            box_at((0.052, 0.044, 0.010), (WORM_BLOCK_X, WORM_Y, 0.115)),
        ]
    )

    spindle_lower_pedestal = box_at((0.030, 0.045, 0.055), (0.0, 0.022, 0.0435))
    spindle_lower_boss = cylinder_z(0.022, 0.010, (0.0, 0.0, 0.066))
    spindle_brace = box_at((0.018, 0.060, 0.026), (0.0, 0.048, 0.083))
    spindle_post = box_at((0.022, 0.022, 0.104), (0.0, 0.074, 0.144))
    spindle_cap_arm = box_at((0.016, 0.076, 0.012), (0.0, 0.038, 0.194))
    spindle_cap = box_at((0.052, 0.034, 0.010), (0.0, 0.0, 0.198))

    output_lower_boss = cylinder_z(0.018, 0.010, (OUTPUT_X, 0.0, 0.124))
    output_rib = box_at((0.016, 0.050, 0.018), (OUTPUT_X + 0.014, 0.025, 0.125))
    output_post_x = OUTPUT_X + 0.050
    output_post = box_at((0.020, 0.020, 0.104), (output_post_x, 0.074, 0.144))
    output_cap_arm_y = box_at((0.014, 0.076, 0.012), (OUTPUT_X, 0.038, 0.194))
    output_cap_arm_x = box_at((0.050, 0.014, 0.012), ((OUTPUT_X + output_post_x) / 2.0, 0.074, 0.194))
    output_cap = box_at((0.046, 0.032, 0.010), (OUTPUT_X, 0.0, 0.197))
    bridge = box_at((output_post_x + 0.020, 0.024, 0.018), (output_post_x / 2.0, 0.074, 0.185))

    frame = union_all(
        [
            base,
            front_rail,
            rear_rail,
            left_worm_block,
            right_worm_block,
            spindle_lower_pedestal,
            spindle_lower_boss,
            spindle_brace,
            spindle_post,
            spindle_cap_arm,
            spindle_cap,
            output_lower_boss,
            output_rib,
            output_post,
            output_cap_arm_y,
            output_cap_arm_x,
            output_cap,
            bridge,
        ]
    )

    frame = frame.cut(cylinder_x(0.0110, 0.340, (0.0, WORM_Y, WHEEL_Z)))
    frame = frame.cut(cylinder_z(0.0112, 0.160, (0.0, 0.0, 0.130)))
    frame = frame.cut(cylinder_z(WHEEL_OUTER_R + 0.004, 0.056, (0.0, 0.0, WHEEL_Z)))
    frame = frame.cut(cylinder_z(0.0102, 0.110, (OUTPUT_X, 0.0, 0.160)))
    return frame


def make_worm_cap() -> cq.Workplane:
    cap = union_all(
        [
            box_at((0.052, 0.044, 0.010), (0.0, 0.0, 0.0)),
            box_at((0.010, 0.044, 0.030), (-0.018, 0.0, -0.020)),
            box_at((0.010, 0.044, 0.030), (0.018, 0.0, -0.020)),
        ]
    )
    return cap.cut(cylinder_x(0.012, 0.070, (0.0, 0.0, -0.015)))


def make_vertical_cap(plate_x: float, plate_y: float, arm_y: float, shaft_diameter: float) -> cq.Workplane:
    cap = union_all(
        [
            box_at((plate_x, plate_y, 0.010), (0.0, 0.0, 0.0)),
            box_at((0.016, arm_y, 0.010), (0.0, arm_y / 2.0 - 0.006, 0.0)),
            box_at((0.028, 0.018, 0.020), (0.0, arm_y - 0.012, -0.005)),
        ]
    )
    return cap.cut(cylinder_z(shaft_diameter / 2.0, 0.030, (0.0, 0.0, 0.0)))


def make_frame() -> cq.Workplane:
    base = box_at((BASE_LEN, BASE_DEPTH, BASE_THK), (0.0, 0.0, BASE_THK / 2.0))
    front_beam = box_at((0.205, 0.030, 0.028), (0.0, WORM_Y, 0.030))
    rear_beam = box_at((0.180, 0.028, 0.024), (0.032, 0.052, 0.034))

    central_body = box_at((0.070, 0.075, 0.046), (0.0, 0.010, 0.039))
    central_seat = cylinder_z(0.030, 0.008, (0.0, 0.0, 0.066))
    output_body = box_at((0.052, 0.060, 0.104), (OUTPUT_X, 0.010, 0.068))
    output_seat = cylinder_z(0.026, 0.010, (OUTPUT_X, 0.0, 0.125))

    left_worm_block = union_all(
        [
            box_at((0.064, 0.074, 0.014), (-WORM_BLOCK_X, WORM_Y, 0.023)),
            box_at((WORM_BLOCK_LEN, 0.050, 0.060), (-WORM_BLOCK_X, WORM_Y, 0.046)),
            cylinder_x(0.022, WORM_BLOCK_LEN, (-WORM_BLOCK_X, WORM_Y, WHEEL_Z)),
        ]
    )
    right_worm_block = union_all(
        [
            box_at((0.064, 0.074, 0.014), (WORM_BLOCK_X, WORM_Y, 0.023)),
            box_at((WORM_BLOCK_LEN, 0.050, 0.060), (WORM_BLOCK_X, WORM_Y, 0.046)),
            cylinder_x(0.022, WORM_BLOCK_LEN, (WORM_BLOCK_X, WORM_Y, WHEEL_Z)),
        ]
    )

    central_post = box_at(
        (0.034, 0.022, BRIDGE_CENTER_Z - BRIDGE_THK / 2.0 - BASE_THK),
        (0.0, 0.052, (BRIDGE_CENTER_Z - BRIDGE_THK / 2.0 + BASE_THK) / 2.0),
    )
    output_post = box_at(
        (0.030, 0.022, BRIDGE_CENTER_Z - BRIDGE_THK / 2.0 - BASE_THK),
        (OUTPUT_X, 0.052, (BRIDGE_CENTER_Z - BRIDGE_THK / 2.0 + BASE_THK) / 2.0),
    )
    bridge = box_at(
        (OUTPUT_X + 0.085, 0.040, BRIDGE_THK),
        (OUTPUT_X / 2.0 + 0.008, 0.022, BRIDGE_CENTER_Z),
    )
    spindle_cap = cylinder_z(0.024, BRIDGE_THK, (0.0, 0.0, BRIDGE_CENTER_Z))
    output_cap = cylinder_z(0.022, BRIDGE_THK, (OUTPUT_X, 0.0, BRIDGE_CENTER_Z))

    frame = union_all(
        [
            base,
            front_beam,
            rear_beam,
            central_body,
            central_seat,
            output_body,
            output_seat,
            left_worm_block,
            right_worm_block,
            central_post,
            output_post,
            bridge,
            spindle_cap,
            output_cap,
        ]
    )

    frame = frame.cut(cylinder_x(WORM_SHAFT_D / 2.0 + 0.0015, 0.330, (0.0, WORM_Y, WHEEL_Z)))
    frame = frame.cut(cylinder_z(SPINDLE_D / 2.0 + 0.0015, 0.320, (0.0, 0.0, 0.160)))
    frame = frame.cut(cylinder_z(OUTPUT_SHAFT_D / 2.0 + 0.0015, 0.250, (OUTPUT_X, 0.0, 0.180)))

    frame = frame.cut(box_at((0.078, 0.052, 0.034), (0.0, 0.004, 0.048)))
    frame = frame.cut(box_at((0.044, 0.042, 0.046), (OUTPUT_X, 0.000, 0.106)))
    frame = frame.cut(box_at((0.104, 0.026, 0.010), (OUTPUT_X / 2.0 + 0.008, 0.020, 0.208)))
    frame = frame.cut(box_at((0.050, 0.055, 0.002), (-WORM_BLOCK_X, WORM_Y, WHEEL_Z)))
    frame = frame.cut(box_at((0.050, 0.055, 0.002), (WORM_BLOCK_X, WORM_Y, WHEEL_Z)))
    return frame


def make_worm_shaft_core() -> cq.Workplane:
    shaft_length = 0.294
    shaft = cylinder_x(WORM_SHAFT_D / 2.0, shaft_length, (0.0, 0.0, 0.0))
    left_collar = cylinder_x(0.016, 0.010, (-0.132, 0.0, 0.0))
    right_collar = cylinder_x(0.016, 0.010, (0.132, 0.0, 0.0))
    left_spacer = cylinder_x(0.013, 0.024, (-0.050, 0.0, 0.0))
    right_spacer = cylinder_x(0.013, 0.024, (0.050, 0.0, 0.0))
    return union_all([shaft, left_collar, right_collar, left_spacer, right_spacer])


def make_wheel_spindle_core() -> cq.Workplane:
    shaft = cylinder_z(SPINDLE_D / 2.0, 0.252, (0.0, 0.0, 0.044))
    lower_washer = cylinder_z(0.021, 0.006, (0.0, 0.0, -0.023))
    wheel_hub = cylinder_z(0.024, 0.034, (0.0, 0.0, 0.0))
    mid_spacer = cylinder_z(0.015, 0.026, (0.0, 0.0, 0.044))
    pinion_hub = cylinder_z(0.018, 0.022, (0.0, 0.0, 0.070))
    top_collar = cylinder_z(0.018, 0.006, (0.0, 0.0, 0.110))
    key_bar = box_at((0.006, 0.004, 0.060), (0.010, 0.0, 0.030))
    return union_all([shaft, lower_washer, wheel_hub, mid_spacer, pinion_hub, top_collar, key_bar])


def make_output_shaft_core() -> cq.Workplane:
    shaft = cylinder_z(OUTPUT_SHAFT_D / 2.0, 0.098, (0.0, 0.0, 0.004))
    lower_washer = cylinder_z(0.018, 0.006, (0.0, 0.0, -0.033))
    lower_spacer = cylinder_z(0.014, 0.014, (0.0, 0.0, -0.016))
    gear_hub = cylinder_z(0.020, 0.026, (0.0, 0.0, 0.0))
    top_spacer = cylinder_z(0.014, 0.012, (0.0, 0.0, 0.026))
    top_collar = cylinder_z(0.016, 0.006, (0.0, 0.0, 0.040))
    key_bar = box_at((0.005, 0.004, 0.042), (0.009, 0.0, 0.004))
    return union_all([shaft, lower_washer, lower_spacer, gear_hub, top_spacer, top_collar, key_bar])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="worm_indexing_unit")

    model.material("frame_paint", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("bronze", rgba=(0.62, 0.44, 0.18, 1.0))
    model.material("gear_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("darkened_steel", rgba=(0.40, 0.41, 0.44, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame_bench(), "indexer_frame_bench_v3"),
        material="frame_paint",
        name="frame_shell",
    )

    worm_shaft = model.part("worm_shaft")
    worm_shaft.visual(
        Cylinder(radius=WORM_SHAFT_D / 2.0, length=0.294),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="steel",
        name="shaft_core",
    )
    for x_pos, radius, length, name in (
        (-0.149, 0.014, 0.010, "left_retainer"),
        (0.149, 0.014, 0.010, "right_retainer"),
        (-WORM_BLOCK_X, 0.011, 0.014, "left_bearing_sleeve"),
        (WORM_BLOCK_X, 0.011, 0.014, "right_bearing_sleeve"),
    ):
        worm_shaft.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="steel",
            name=name,
        )
    worm_shaft.visual(
        mesh_from_cadquery(make_worm_thread(), "worm_thread_v2"),
        material="darkened_steel",
        name="worm_thread",
    )

    wheel_spindle = model.part("wheel_spindle")
    for radius, length, z_pos, name in (
        (SPINDLE_D / 2.0, 0.118, 0.045, "spindle_core"),
        (0.024, 0.034, 0.0, "wheel_hub"),
        (0.015, 0.020, 0.045, "mid_spacer"),
        (0.018, 0.018, 0.070, "pinion_hub"),
    ):
        wheel_spindle.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material="steel",
            name=name,
        )
    wheel_spindle.visual(
        Box((0.006, 0.004, 0.058)),
        origin=Origin(xyz=(0.010, 0.0, 0.030)),
        material="darkened_steel",
        name="spindle_key",
    )
    wheel_spindle.visual(
        mesh_from_cadquery(
            make_spur_gear(
                module=WHEEL_MODULE,
                teeth=WHEEL_TEETH,
                width=WHEEL_WIDTH,
                bore_d=SPINDLE_D * 0.85,
                hub_radius=0.024,
                hub_width=0.034,
                spoke_count=6,
                spoke_width=0.010,
            ),
            "worm_wheel_v2",
        ),
        material="bronze",
        name="worm_wheel",
    )
    wheel_spindle.visual(
        mesh_from_cadquery(
            make_spur_gear(
                module=PINION_MODULE,
                teeth=PINION_TEETH,
                width=PINION_WIDTH,
                bore_d=SPINDLE_D * 0.85,
                hub_radius=0.018,
                hub_width=0.018,
                spoke_count=4,
                spoke_width=0.008,
            ),
            "upper_pinion_v2",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material="gear_gray",
        name="upper_pinion",
    )

    output_shaft = model.part("output_shaft")
    for radius, length, z_pos, name in (
        (OUTPUT_SHAFT_D / 2.0, 0.066, 0.000, "output_core"),
        (0.014, 0.014, -0.016, "lower_spacer"),
        (0.020, 0.026, 0.0, "gear_hub"),
        (0.014, 0.010, 0.015, "top_spacer"),
    ):
        output_shaft.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material="steel",
            name=name,
        )
    output_shaft.visual(
        Box((0.005, 0.004, 0.040)),
        origin=Origin(xyz=(0.009, 0.0, 0.000)),
        material="darkened_steel",
        name="output_key",
    )
    output_shaft.visual(
        mesh_from_cadquery(
            make_spur_gear(
                module=OUTPUT_MODULE,
                teeth=OUTPUT_TEETH,
                width=OUTPUT_WIDTH,
                bore_d=OUTPUT_SHAFT_D * 0.85,
                hub_radius=0.020,
                hub_width=0.026,
                spoke_count=5,
                spoke_width=0.009,
            ),
            "output_gear_v2",
        ),
        material="darkened_steel",
        name="output_gear",
    )

    model.articulation(
        "frame_to_worm",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=worm_shaft,
        origin=Origin(xyz=(0.0, WORM_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "frame_to_wheel_spindle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_spindle,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=6.0),
    )
    model.articulation(
        "frame_to_output",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_X, 0.0, UPPER_GEAR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0013)
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

    frame = object_model.get_part("frame")
    worm = object_model.get_part("worm_shaft")
    spindle = object_model.get_part("wheel_spindle")
    output = object_model.get_part("output_shaft")

    worm_joint = object_model.get_articulation("frame_to_worm")
    spindle_joint = object_model.get_articulation("frame_to_wheel_spindle")
    output_joint = object_model.get_articulation("frame_to_output")

    ctx.check(
        "all benchmark parts exist",
        all(
            part is not None
            for part in (
                frame,
                worm,
                spindle,
                output,
            )
        ),
        "expected frame, worm shaft, wheel spindle, and output shaft parts",
    )
    ctx.check(
        "joint axes follow supported shafts",
        worm_joint.axis == (1.0, 0.0, 0.0)
        and spindle_joint.axis == (0.0, 0.0, 1.0)
        and output_joint.axis == (0.0, 0.0, 1.0),
        (
            f"axes were worm={worm_joint.axis}, "
            f"spindle={spindle_joint.axis}, output={output_joint.axis}"
        ),
    )

    ctx.expect_contact(worm, frame, contact_tol=0.0013, name="worm shaft is radially supported by frame pedestals")
    ctx.expect_contact(spindle, frame, contact_tol=0.0013, name="wheel spindle is seated on the lower frame boss")
    ctx.expect_contact(output, frame, contact_tol=0.0013, name="output shaft is seated on the lower frame boss")

    ctx.expect_overlap(
        spindle,
        worm,
        axes="xz",
        elem_a="worm_wheel",
        elem_b="worm_thread",
        min_overlap=0.020,
        name="worm and wheel share a deliberate mesh zone",
    )
    ctx.expect_gap(
        spindle,
        worm,
        axis="y",
        positive_elem="worm_wheel",
        negative_elem="worm_thread",
        min_gap=0.0001,
        max_gap=0.006,
        name="worm and wheel keep backlash clearance",
    )

    ctx.expect_overlap(
        output,
        spindle,
        axes="yz",
        elem_a="output_gear",
        elem_b="upper_pinion",
        min_overlap=0.012,
        name="upper gear train shares one mesh plane",
    )
    ctx.expect_gap(
        output,
        spindle,
        axis="x",
        positive_elem="output_gear",
        negative_elem="upper_pinion",
        min_gap=0.0001,
        max_gap=0.006,
        name="upper gear pair keeps believable side clearance",
    )

    ctx.expect_origin_distance(
        output,
        spindle,
        axes="x",
        min_dist=OUTPUT_X - 0.002,
        max_dist=OUTPUT_X + 0.002,
        name="output shaft stands beside the wheel spindle at the designed center distance",
    )
    ctx.expect_origin_gap(
        spindle,
        worm,
        axis="z",
        min_gap=WHEEL_Z - WHEEL_Z,
        max_gap=0.0,
        name="worm axis sits level with the wheel center",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
