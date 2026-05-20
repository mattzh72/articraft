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


BODY_LENGTH = 2.10
BODY_WIDTH = 0.90
BODY_HEIGHT = 0.80
SHELL_THICKNESS = 0.035
LINER_THICKNESS = 0.025

OPENING_X_MIN = -0.9875
OPENING_X_MAX = -0.1625
OPENING_CENTER_X = 0.5 * (OPENING_X_MIN + OPENING_X_MAX)
LID_TRAVEL = OPENING_X_MAX - OPENING_X_MIN

LID_LENGTH = 0.82
LID_WIDTH = 0.182
LID_CENTER_Z = 0.810
LID_LANES = (-0.216, 0.0, 0.216)

RAIL_X_MIN = -1.015
RAIL_X_MAX = 0.705
RAIL_LENGTH = RAIL_X_MAX - RAIL_X_MIN
RAIL_CENTER_X = 0.5 * (RAIL_X_MIN + RAIL_X_MAX)
RAIL_WIDTH = 0.030
RAIL_HEIGHT = 0.030
RAIL_CENTER_Z = 0.785
RAIL_CENTERS_Y = (-0.325, -0.108, 0.108, 0.325)

CASTER_X = BODY_LENGTH * 0.5 - 0.12
CASTER_Y = BODY_WIDTH * 0.5 - 0.10
WHEEL_RADIUS = 0.045
WHEEL_WIDTH = 0.030


def _add_lid(
    model: ArticulatedObject,
    body,
    *,
    part_name: str,
    joint_name: str,
    lane_center_y: float,
    frame_material,
    glass_material,
) -> None:
    lid = model.part(part_name)
    lid.visual(
        Box((LID_LENGTH, LID_WIDTH - 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass_material,
        name="glass_panel",
    )
    lid.visual(
        Box((LID_LENGTH, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.5 * (LID_WIDTH - 0.014), 0.001)),
        material=frame_material,
        name="frame_back",
    )
    lid.visual(
        Box((LID_LENGTH, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.5 * (LID_WIDTH - 0.014), 0.001)),
        material=frame_material,
        name="frame_front",
    )
    lid.visual(
        Box((0.014, LID_WIDTH, 0.018)),
        origin=Origin(xyz=(0.5 * (LID_LENGTH - 0.014), 0.0, 0.001)),
        material=frame_material,
        name="frame_right",
    )
    lid.visual(
        Box((0.014, LID_WIDTH, 0.018)),
        origin=Origin(xyz=(-0.5 * (LID_LENGTH - 0.014), 0.0, 0.001)),
        material=frame_material,
        name="frame_left",
    )
    lid.visual(
        Box((0.160, 0.020, 0.010)),
        origin=Origin(xyz=(0.23, 0.0, 0.009)),
        material=frame_material,
        name="pull_handle",
    )
    lid.visual(
        Box((LID_LENGTH - 0.020, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.094, -0.008)),
        material=frame_material,
        name="runner_left",
    )
    lid.visual(
        Box((LID_LENGTH - 0.020, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.094, -0.008)),
        material=frame_material,
        name="runner_right",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_LENGTH, LID_WIDTH, 0.022)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(OPENING_CENTER_X, lane_center_y, LID_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )


def _add_caster(
    model: ArticulatedObject,
    body,
    *,
    corner_name: str,
    mount_x: float,
    mount_y: float,
    metal_material,
    wheel_material,
) -> None:
    fork = model.part(f"{corner_name}_caster_fork")
    fork.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=metal_material,
        name="swivel_stem",
    )
    fork.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=metal_material,
        name="swivel_housing",
    )
    fork.visual(
        Box((0.090, 0.060, 0.016)),
        origin=Origin(xyz=(0.014, 0.0, -0.058)),
        material=metal_material,
        name="fork_bridge",
    )
    fork.visual(
        Box((0.062, 0.012, 0.092)),
        origin=Origin(xyz=(0.028, 0.023, -0.112)),
        material=metal_material,
        name="fork_left_leg",
    )
    fork.visual(
        Box((0.062, 0.012, 0.092)),
        origin=Origin(xyz=(0.028, -0.023, -0.112)),
        material=metal_material,
        name="fork_right_leg",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.10, 0.08, 0.16)),
        mass=1.1,
        origin=Origin(xyz=(0.018, 0.0, -0.080)),
    )

    wheel = model.part(f"{corner_name}_caster_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.022, length=WHEEL_WIDTH + 0.004),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        f"body_to_{corner_name}_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=fork,
        origin=Origin(xyz=(mount_x, mount_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        f"{corner_name}_fork_to_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.028, 0.0, -0.112)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=25.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deep_island_freezer")

    body_white = model.material("body_white", rgba=(0.93, 0.95, 0.96, 1.0))
    liner_white = model.material("liner_white", rgba=(0.87, 0.90, 0.92, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.69, 0.72, 0.75, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.14, 0.14, 0.15, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.52, 0.54, 0.57, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    lid_glass = model.material("lid_glass", rgba=(0.70, 0.83, 0.88, 0.35))
    lid_frame = model.material("lid_frame", rgba=(0.37, 0.40, 0.43, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=body_white,
        name="outer_bottom",
    )
    body.visual(
        Box((BODY_LENGTH, SHELL_THICKNESS, BODY_HEIGHT - 0.040)),
        origin=Origin(
            xyz=(0.0, -(BODY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5), 0.420)
        ),
        material=body_white,
        name="front_shell",
    )
    body.visual(
        Box((BODY_LENGTH, SHELL_THICKNESS, BODY_HEIGHT - 0.040)),
        origin=Origin(
            xyz=(0.0, BODY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5, 0.420)
        ),
        material=body_white,
        name="back_shell",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_WIDTH - 2.0 * SHELL_THICKNESS, BODY_HEIGHT - 0.040)),
        origin=Origin(
            xyz=(-(BODY_LENGTH * 0.5 - SHELL_THICKNESS * 0.5), 0.0, 0.420)
        ),
        material=body_white,
        name="left_shell",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_WIDTH - 2.0 * SHELL_THICKNESS, BODY_HEIGHT - 0.040)),
        origin=Origin(
            xyz=(BODY_LENGTH * 0.5 - SHELL_THICKNESS * 0.5, 0.0, 0.420)
        ),
        material=body_white,
        name="right_shell",
    )

    body.visual(
        Box((0.825, 0.710, 0.025)),
        origin=Origin(xyz=(OPENING_CENTER_X, 0.0, 0.0525)),
        material=liner_white,
        name="storage_floor",
    )
    body.visual(
        Box((0.825, LINER_THICKNESS, 0.710)),
        origin=Origin(xyz=(OPENING_CENTER_X, -0.3475, 0.420)),
        material=liner_white,
        name="storage_front_liner",
    )
    body.visual(
        Box((0.825, LINER_THICKNESS, 0.710)),
        origin=Origin(xyz=(OPENING_CENTER_X, 0.3475, 0.420)),
        material=liner_white,
        name="storage_back_liner",
    )
    body.visual(
        Box((LINER_THICKNESS, 0.710, 0.710)),
        origin=Origin(xyz=(OPENING_X_MIN, 0.0, 0.420)),
        material=liner_white,
        name="storage_left_liner",
    )
    body.visual(
        Box((LINER_THICKNESS, 0.830, 0.710)),
        origin=Origin(xyz=(OPENING_X_MAX, 0.0, 0.420)),
        material=liner_white,
        name="compressor_bulkhead",
    )

    body.visual(
        Box((1.190, 0.830, 0.025)),
        origin=Origin(xyz=(0.420, 0.0, 0.7675)),
        material=deck_gray,
        name="service_deck",
    )
    for rail_index, rail_y in enumerate(RAIL_CENTERS_Y):
        body.visual(
            Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
            origin=Origin(xyz=(RAIL_CENTER_X, rail_y, RAIL_CENTER_Z)),
            material=rail_aluminum,
            name=f"top_rail_{rail_index}",
        )

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=0.050, length=0.180),
                origin=Origin(
                    xyz=(sx * 0.990, sy * 0.390, 0.690)
                ),
                material=bumper_black,
                name=f"corner_bumper_{'r' if sx > 0 else 'l'}_{'b' if sy > 0 else 'f'}",
            )

    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=160.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    _add_lid(
        model,
        body,
        part_name="lid_front",
        joint_name="body_to_lid_front",
        lane_center_y=LID_LANES[0],
        frame_material=lid_frame,
        glass_material=lid_glass,
    )
    _add_lid(
        model,
        body,
        part_name="lid_center",
        joint_name="body_to_lid_center",
        lane_center_y=LID_LANES[1],
        frame_material=lid_frame,
        glass_material=lid_glass,
    )
    _add_lid(
        model,
        body,
        part_name="lid_back",
        joint_name="body_to_lid_back",
        lane_center_y=LID_LANES[2],
        frame_material=lid_frame,
        glass_material=lid_glass,
    )

    _add_caster(
        model,
        body,
        corner_name="front_left",
        mount_x=-CASTER_X,
        mount_y=-CASTER_Y,
        metal_material=caster_metal,
        wheel_material=caster_rubber,
    )
    _add_caster(
        model,
        body,
        corner_name="front_right",
        mount_x=CASTER_X,
        mount_y=-CASTER_Y,
        metal_material=caster_metal,
        wheel_material=caster_rubber,
    )
    _add_caster(
        model,
        body,
        corner_name="rear_left",
        mount_x=-CASTER_X,
        mount_y=CASTER_Y,
        metal_material=caster_metal,
        wheel_material=caster_rubber,
    )
    _add_caster(
        model,
        body,
        corner_name="rear_right",
        mount_x=CASTER_X,
        mount_y=CASTER_Y,
        metal_material=caster_metal,
        wheel_material=caster_rubber,
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
    body = object_model.get_part("body")
    lid_specs = (
        ("lid_front", "body_to_lid_front", LID_LANES[0]),
        ("lid_center", "body_to_lid_center", LID_LANES[1]),
        ("lid_back", "body_to_lid_back", LID_LANES[2]),
    )
    for lid_name, joint_name, lane_y in lid_specs:
        lid = object_model.get_part(lid_name)
        joint = object_model.get_articulation(joint_name)
        rest_pos = ctx.part_world_position(lid)
        ctx.check(
            f"{lid_name} uses a longitudinal prismatic joint",
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.0025,
            name=f"{lid_name} sits on the top rails",
        )
        ctx.check(
            f"{lid_name} starts in its assigned lane",
            rest_pos is not None and abs(rest_pos[1] - lane_y) < 0.01,
            details=f"rest_pos={rest_pos}, expected_y={lane_y}",
        )
        with ctx.pose({joint: LID_TRAVEL}):
            extended_pos = ctx.part_world_position(lid)
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                min_gap=0.0,
                max_gap=0.0025,
                name=f"{lid_name} stays supported when opened",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="y",
                min_overlap=0.16,
                name=f"{lid_name} stays aligned with its rail lane",
            )
        ctx.check(
            f"{lid_name} slides onto the service deck",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.70,
            details=f"rest_pos={rest_pos}, extended_pos={extended_pos}",
        )

    caster_specs = (
        ("front_left", -1.0, -1.0),
        ("front_right", 1.0, -1.0),
        ("rear_left", -1.0, 1.0),
        ("rear_right", 1.0, 1.0),
    )
    for corner_name, sign_x, sign_y in caster_specs:
        wheel = object_model.get_part(f"{corner_name}_caster_wheel")
        swivel = object_model.get_articulation(f"body_to_{corner_name}_caster_swivel")
        spin = object_model.get_articulation(f"{corner_name}_fork_to_wheel_spin")
        wheel_pos = ctx.part_world_position(wheel)
        ctx.check(
            f"{corner_name} caster has a continuous swivel",
            swivel.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(value, 6) for value in swivel.axis) == (0.0, 0.0, 1.0),
            details=f"type={swivel.articulation_type}, axis={swivel.axis}",
        )
        ctx.check(
            f"{corner_name} wheel has a continuous spin axis",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(value, 6) for value in spin.axis) == (0.0, 1.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )
        ctx.check(
            f"{corner_name} caster sits beneath the matching corner",
            wheel_pos is not None
            and wheel_pos[0] * sign_x > 0.84
            and wheel_pos[1] * sign_y > 0.24
            and wheel_pos[2] < -0.09,
            details=f"wheel_pos={wheel_pos}",
        )

    front_left_wheel = object_model.get_part("front_left_caster_wheel")
    front_left_swivel = object_model.get_articulation("body_to_front_left_caster_swivel")
    rest_fl_pos = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_left_swivel: math.pi / 2.0}):
        swiveled_fl_pos = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front-left caster swivel reorients the wheel footprint",
        rest_fl_pos is not None
        and swiveled_fl_pos is not None
        and (
            abs(swiveled_fl_pos[0] - rest_fl_pos[0]) > 0.015
            or abs(swiveled_fl_pos[1] - rest_fl_pos[1]) > 0.015
        ),
        details=f"rest={rest_fl_pos}, swiveled={swiveled_fl_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
