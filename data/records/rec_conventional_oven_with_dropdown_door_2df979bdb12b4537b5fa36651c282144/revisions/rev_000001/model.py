from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

CABINET_WIDTH = 0.760
CABINET_HEIGHT = 1.080
CABINET_DEPTH = 0.620
CABINET_OPENING_WIDTH = 0.620
CABINET_OPENING_HEIGHT = 0.930
CABINET_SIDE_THICKNESS = (CABINET_WIDTH - CABINET_OPENING_WIDTH) * 0.5
CABINET_TOP_THICKNESS = (CABINET_HEIGHT - CABINET_OPENING_HEIGHT) * 0.5

OVEN_WIDTH = 0.595
OVEN_HEIGHT = 0.895
OVEN_DEPTH = 0.560
OVEN_FRONT_FACE_X = OVEN_DEPTH * 0.5
OVEN_CENTER_IN_CABINET_X = CABINET_DEPTH * 0.5 - OVEN_DEPTH * 0.5

CONTROL_BAND_HEIGHT = 0.085
CONTROL_BAND_CENTER_Z = OVEN_HEIGHT * 0.5 - CONTROL_BAND_HEIGHT * 0.5
DOOR_WIDTH = 0.545
DOOR_HEIGHT = 0.735
DOOR_THICKNESS = 0.038
DOOR_HINGE_Z = -0.402
DOOR_HINGE_X = OVEN_FRONT_FACE_X + 0.002

CAVITY_WIDTH = 0.485
CAVITY_HEIGHT = 0.620
CAVITY_DEPTH = 0.430
CAVITY_CENTER_Z = -0.055
CAVITY_FRONT_X = OVEN_FRONT_FACE_X - 0.055
CAVITY_BACK_FACE_X = CAVITY_FRONT_X - CAVITY_DEPTH
CAVITY_CENTER_X = (CAVITY_FRONT_X + CAVITY_BACK_FACE_X) * 0.5
SIDE_WALL_THICKNESS = 0.035
TOP_WALL_THICKNESS = 0.050
BOTTOM_WALL_THICKNESS = 0.060
BACK_WALL_THICKNESS = 0.030

SUPPORT_RAIL_LENGTH = 0.372
SUPPORT_RAIL_WIDTH = 0.016
SUPPORT_RAIL_HEIGHT = 0.010
RACK_Z = 0.025
RACK_JOINT_X = 0.170
RACK_TRAVEL = 0.180
RACK_RUNNER_LENGTH = 0.340
RACK_RUNNER_WIDTH = 0.010
RACK_RUNNER_HEIGHT = 0.010
RACK_CLEAR_WIDTH = 0.455
RACK_SIDE_RAIL_LENGTH = 0.300
RACK_SIDE_RAIL_RADIUS = 0.005
RACK_BAR_RADIUS = 0.0038
RACK_FRONT_X = 0.045
RACK_REAR_X = -0.255


def _cylinder_origin_along_y(
    *, x: float, y: float, z: float
) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _cylinder_origin_along_x(
    *, x: float, y: float, z: float
) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_wall_oven")

    cabinet_wood = model.material("cabinet_wood", rgba=(0.55, 0.43, 0.31, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    control_glass = model.material("control_glass", rgba=(0.08, 0.10, 0.12, 0.88))
    door_glass = model.material("door_glass", rgba=(0.16, 0.20, 0.24, 0.40))
    rack_chrome = model.material("rack_chrome", rgba=(0.80, 0.82, 0.85, 1.0))

    cabinet = model.part("cabinet_surround")
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_SIDE_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -(CABINET_OPENING_WIDTH * 0.5 + CABINET_SIDE_THICKNESS * 0.5), 0.0)
        ),
        material=cabinet_wood,
        name="left_panel",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_SIDE_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CABINET_OPENING_WIDTH * 0.5 + CABINET_SIDE_THICKNESS * 0.5, 0.0)
        ),
        material=cabinet_wood,
        name="right_panel",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_OPENING_WIDTH, CABINET_TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, CABINET_OPENING_HEIGHT * 0.5 + CABINET_TOP_THICKNESS * 0.5)
        ),
        material=cabinet_wood,
        name="top_panel",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_OPENING_WIDTH, CABINET_TOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, -(CABINET_OPENING_HEIGHT * 0.5 + CABINET_TOP_THICKNESS * 0.5))
        ),
        material=cabinet_wood,
        name="bottom_panel",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_DEPTH, CABINET_WIDTH, CABINET_HEIGHT)),
        mass=32.0,
    )

    body = model.part("oven_body")
    body.visual(
        Box((CAVITY_DEPTH, SIDE_WALL_THICKNESS, CAVITY_HEIGHT)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X,
                -(CAVITY_WIDTH * 0.5 + SIDE_WALL_THICKNESS * 0.5),
                CAVITY_CENTER_Z,
            )
        ),
        material=cavity_dark,
        name="left_cavity_wall",
    )
    body.visual(
        Box((CAVITY_DEPTH, SIDE_WALL_THICKNESS, CAVITY_HEIGHT)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X,
                CAVITY_WIDTH * 0.5 + SIDE_WALL_THICKNESS * 0.5,
                CAVITY_CENTER_Z,
            )
        ),
        material=cavity_dark,
        name="right_cavity_wall",
    )
    body.visual(
        Box((CAVITY_DEPTH, CAVITY_WIDTH + 2.0 * SIDE_WALL_THICKNESS, TOP_WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X,
                0.0,
                CAVITY_CENTER_Z + CAVITY_HEIGHT * 0.5 + TOP_WALL_THICKNESS * 0.5,
            )
        ),
        material=cavity_dark,
        name="top_cavity_wall",
    )
    body.visual(
        Box(
            (
                CAVITY_DEPTH,
                CAVITY_WIDTH + 2.0 * SIDE_WALL_THICKNESS,
                BOTTOM_WALL_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X,
                0.0,
                CAVITY_CENTER_Z - CAVITY_HEIGHT * 0.5 - BOTTOM_WALL_THICKNESS * 0.5,
            )
        ),
        material=cavity_dark,
        name="bottom_cavity_wall",
    )
    body.visual(
        Box(
            (
                BACK_WALL_THICKNESS,
                CAVITY_WIDTH + 2.0 * SIDE_WALL_THICKNESS,
                CAVITY_HEIGHT + TOP_WALL_THICKNESS + BOTTOM_WALL_THICKNESS,
            )
        ),
        origin=Origin(xyz=(CAVITY_BACK_FACE_X - BACK_WALL_THICKNESS * 0.5, 0.0, CAVITY_CENTER_Z)),
        material=cavity_dark,
        name="back_wall",
    )

    trim_face_x = OVEN_FRONT_FACE_X - 0.010
    side_trim_y = OVEN_WIDTH * 0.5 - 0.014
    body.visual(
        Box((0.020, 0.053, OVEN_HEIGHT)),
        origin=Origin(xyz=(trim_face_x, -side_trim_y, 0.0)),
        material=stainless,
        name="left_trim",
    )
    body.visual(
        Box((0.020, 0.053, OVEN_HEIGHT)),
        origin=Origin(xyz=(trim_face_x, side_trim_y, 0.0)),
        material=stainless,
        name="right_trim",
    )
    body.visual(
        Box((0.035, 0.028, OVEN_HEIGHT)),
        origin=Origin(xyz=(CAVITY_FRONT_X + 0.0175, -0.271, 0.0)),
        material=stainless,
        name="left_face_bridge",
    )
    body.visual(
        Box((0.035, 0.028, OVEN_HEIGHT)),
        origin=Origin(xyz=(CAVITY_FRONT_X + 0.0175, 0.271, 0.0)),
        material=stainless,
        name="right_face_bridge",
    )
    body.visual(
        Box((0.020, OVEN_WIDTH, 0.045)),
        origin=Origin(xyz=(trim_face_x, 0.0, -OVEN_HEIGHT * 0.5 + 0.0225)),
        material=stainless,
        name="bottom_trim",
    )
    body.visual(
        Box((0.020, OVEN_WIDTH, CONTROL_BAND_HEIGHT)),
        origin=Origin(xyz=(trim_face_x, 0.0, CONTROL_BAND_CENTER_Z)),
        material=stainless,
        name="control_band",
    )
    body.visual(
        Box((0.007, 0.470, 0.052)),
        origin=Origin(xyz=(OVEN_FRONT_FACE_X - 0.0035, 0.0, CONTROL_BAND_CENTER_Z)),
        material=control_glass,
        name="control_glass",
    )
    body.visual(
        Box((0.005, 0.420, 0.018)),
        origin=Origin(xyz=(OVEN_FRONT_FACE_X - 0.0025, 0.0, DOOR_HINGE_Z + DOOR_HEIGHT + 0.024)),
        material=control_glass,
        name="vent_slot",
    )

    support_rail_y = CAVITY_WIDTH * 0.5 - SUPPORT_RAIL_WIDTH * 0.5
    body.visual(
        Box((SUPPORT_RAIL_LENGTH, SUPPORT_RAIL_WIDTH, SUPPORT_RAIL_HEIGHT)),
        origin=Origin(xyz=(-0.010, -support_rail_y, RACK_Z - 0.010)),
        material=brushed_steel,
        name="left_support_rail",
    )
    body.visual(
        Box((SUPPORT_RAIL_LENGTH, SUPPORT_RAIL_WIDTH, SUPPORT_RAIL_HEIGHT)),
        origin=Origin(xyz=(-0.010, support_rail_y, RACK_Z - 0.010)),
        material=brushed_steel,
        name="right_support_rail",
    )
    body.inertial = Inertial.from_geometry(
        Box((OVEN_DEPTH, OVEN_WIDTH, OVEN_HEIGHT)),
        mass=29.0,
    )

    door = model.part("oven_door")
    door.visual(
        Cylinder(radius=0.010, length=DOOR_WIDTH),
        origin=_cylinder_origin_along_y(x=0.008, y=0.0, z=0.0),
        material=stainless,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.028, DOOR_WIDTH, 0.055)),
        origin=Origin(xyz=(0.018, 0.0, 0.030)),
        material=stainless,
        name="bottom_rail",
    )
    door.visual(
        Box((0.028, DOOR_WIDTH, 0.060)),
        origin=Origin(xyz=(0.018, 0.0, DOOR_HEIGHT - 0.030)),
        material=stainless,
        name="top_rail",
    )
    stile_height = DOOR_HEIGHT - 0.060
    stile_y = DOOR_WIDTH * 0.5 - 0.0275
    door.visual(
        Box((0.028, 0.055, stile_height)),
        origin=Origin(xyz=(0.018, -stile_y, DOOR_HEIGHT * 0.5)),
        material=stainless,
        name="left_stile",
    )
    door.visual(
        Box((0.028, 0.055, stile_height)),
        origin=Origin(xyz=(0.018, stile_y, DOOR_HEIGHT * 0.5)),
        material=stainless,
        name="right_stile",
    )
    door.visual(
        Box((0.010, 0.435, 0.500)),
        origin=Origin(xyz=(0.010, 0.0, 0.365)),
        material=door_glass,
        name="glass_panel",
    )
    door.visual(
        Box((0.016, 0.470, 0.580)),
        origin=Origin(xyz=(0.008, 0.0, 0.330)),
        material=cavity_dark,
        name="inner_panel",
    )
    handle_z = 0.605
    door.visual(
        Cylinder(radius=0.011, length=0.440),
        origin=_cylinder_origin_along_y(x=0.060, y=0.0, z=handle_z),
        material=rack_chrome,
        name="handle_bar",
    )
    for side, y in (("left", -0.160), ("right", 0.160)):
        door.visual(
            Cylinder(radius=0.007, length=0.045),
            origin=_cylinder_origin_along_x(x=0.0375, y=y, z=handle_z),
            material=rack_chrome,
            name=f"{side}_handle_post",
        )
        door.visual(
            Cylinder(radius=0.0085, length=0.010),
            origin=_cylinder_origin_along_x(x=0.020, y=y, z=handle_z),
            material=rack_chrome,
            name=f"{side}_handle_boss",
        )
    door.inertial = Inertial.from_geometry(
        Box((0.080, DOOR_WIDTH, DOOR_HEIGHT)),
        mass=11.0,
        origin=Origin(xyz=(0.040, 0.0, DOOR_HEIGHT * 0.5)),
    )

    rack = model.part("oven_rack")
    runner_y = CAVITY_WIDTH * 0.5 - SUPPORT_RAIL_WIDTH * 0.5
    rack_side_y = 0.227
    rack_front_x = 0.050
    rack_rear_x = -0.250
    rack_side_length = rack_front_x - rack_rear_x
    rack.visual(
        Box((RACK_RUNNER_LENGTH, RACK_RUNNER_WIDTH, RACK_RUNNER_HEIGHT)),
        origin=Origin(xyz=(-0.095, -runner_y, 0.0)),
        material=brushed_steel,
        name="left_runner",
    )
    rack.visual(
        Box((RACK_RUNNER_LENGTH, RACK_RUNNER_WIDTH, RACK_RUNNER_HEIGHT)),
        origin=Origin(xyz=(-0.095, runner_y, 0.0)),
        material=brushed_steel,
        name="right_runner",
    )
    rack.visual(
        Box((rack_side_length, 0.026, 0.024)),
        origin=Origin(xyz=(0.5 * (rack_front_x + rack_rear_x), -rack_side_y, 0.016)),
        material=rack_chrome,
        name="left_side_rail",
    )
    rack.visual(
        Box((rack_side_length, 0.026, 0.024)),
        origin=Origin(xyz=(0.5 * (rack_front_x + rack_rear_x), rack_side_y, 0.016)),
        material=rack_chrome,
        name="right_side_rail",
    )
    rack.visual(
        Box((0.016, 0.454, 0.024)),
        origin=Origin(xyz=(rack_front_x, 0.0, 0.016)),
        material=brushed_steel,
        name="front_crossmember",
    )
    rack.visual(
        Box((0.016, 0.454, 0.024)),
        origin=Origin(xyz=(rack_rear_x, 0.0, 0.016)),
        material=brushed_steel,
        name="rear_crossmember",
    )
    rack.visual(
        Box((0.012, 0.438, 0.040)),
        origin=Origin(xyz=(rack_front_x, 0.0, 0.034)),
        material=rack_chrome,
        name="front_lip",
    )
    rack.visual(
        Box((0.012, 0.438, 0.024)),
        origin=Origin(xyz=(rack_rear_x, 0.0, 0.028)),
        material=rack_chrome,
        name="rear_rail",
    )
    for index, x in enumerate((-0.200, -0.140, -0.080, -0.020, 0.040)):
        rack.visual(
            Box((0.008, 0.438, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=rack_chrome,
            name=f"deck_bar_{index}",
        )
    rack.inertial = Inertial.from_geometry(
        Box((0.360, RACK_CLEAR_WIDTH, 0.080)),
        mass=2.4,
        origin=Origin(xyz=(-0.090, 0.0, 0.030)),
    )

    model.articulation(
        "cabinet_to_body",
        ArticulationType.FIXED,
        parent=cabinet,
        child=body,
        origin=Origin(xyz=(OVEN_CENTER_IN_CABINET_X, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, 0.0, DOOR_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(RACK_JOINT_X, 0.0, RACK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.20,
            lower=0.0,
            upper=RACK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_surround")
    body = object_model.get_part("oven_body")
    door = object_model.get_part("oven_door")
    rack = object_model.get_part("oven_rack")
    door_hinge = object_model.get_articulation("body_to_door")
    rack_slide = object_model.get_articulation("body_to_rack")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.check(
        "all core parts resolved",
        all(part is not None for part in (cabinet, body, door, rack)),
        details="Expected cabinet_surround, oven_body, oven_door, and oven_rack parts.",
    )

    cabinet_left_panel = ctx.part_element_world_aabb(cabinet, elem="left_panel")
    body_left_trim = ctx.part_element_world_aabb(body, elem="left_trim")
    cabinet_front_x = cabinet_left_panel[1][0] if cabinet_left_panel is not None else None
    body_front_x = body_left_trim[1][0] if body_left_trim is not None else None
    ctx.check(
        "oven housing sits flush with cabinet surround",
        cabinet_front_x is not None
        and body_front_x is not None
        and abs(body_front_x - cabinet_front_x) <= 0.002,
        details=f"cabinet_front_x={cabinet_front_x}, body_front_x={body_front_x}",
    )

    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="glass_panel",
        negative_elem="control_glass",
        min_gap=0.004,
        max_gap=0.020,
        name="closed door glass stands just proud of the oven face",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.500,
        name="door covers the oven front opening",
    )

    ctx.expect_within(
        rack,
        body,
        axes="y",
        inner_elem="left_runner",
        outer_elem="left_support_rail",
        margin=0.004,
        name="left rack runner stays centered over the support rail",
    )
    ctx.expect_contact(
        rack,
        body,
        elem_a="left_runner",
        elem_b="left_support_rail",
        name="left rack runner rests on the support rail",
    )
    ctx.expect_overlap(
        rack,
        body,
        axes="x",
        elem_a="left_runner",
        elem_b="left_support_rail",
        min_overlap=0.250,
        name="collapsed rack remains retained in the support rail",
    )

    closed_top_center = _aabb_center(ctx.part_element_world_aabb(door, elem="top_rail"))
    with ctx.pose({door_hinge: 1.20}):
        open_top_center = _aabb_center(ctx.part_element_world_aabb(door, elem="top_rail"))
    ctx.check(
        "door opens downward and outward",
        closed_top_center is not None
        and open_top_center is not None
        and open_top_center[0] > closed_top_center[0] + 0.18
        and open_top_center[2] < closed_top_center[2] - 0.20,
        details=f"closed_top_center={closed_top_center}, open_top_center={open_top_center}",
    )

    rack_rest_pos = ctx.part_world_position(rack)
    with ctx.pose({rack_slide: RACK_TRAVEL}):
        ctx.expect_within(
            rack,
            body,
            axes="y",
            inner_elem="left_runner",
            outer_elem="left_support_rail",
            margin=0.004,
            name="extended rack runner stays centered over the support rail",
        )
        ctx.expect_contact(
            rack,
            body,
            elem_a="left_runner",
            elem_b="left_support_rail",
            name="extended rack runner remains supported by the rail",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="x",
            elem_a="left_runner",
            elem_b="left_support_rail",
            min_overlap=0.090,
            name="extended rack retains insertion in the support rail",
        )
        rack_extended_pos = ctx.part_world_position(rack)
    ctx.check(
        "rack pulls forward from the oven cavity",
        rack_rest_pos is not None
        and rack_extended_pos is not None
        and rack_extended_pos[0] > rack_rest_pos[0] + 0.16,
        details=f"rack_rest_pos={rack_rest_pos}, rack_extended_pos={rack_extended_pos}",
    )

    with ctx.pose({door_hinge: 1.20, rack_slide: RACK_TRAVEL}):
        ctx.expect_gap(
            rack,
            door,
            axis="z",
            min_gap=0.100,
            positive_elem="front_lip",
            negative_elem="top_rail",
            name="extended rack clears the opened door",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
