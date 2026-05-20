from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
    place_on_face_uv,
)


WIDTH = 0.380
DEPTH = 0.255
HEIGHT = 0.615
WALL = 0.004

BUCKET_WIDTH = 0.316
BUCKET_DEPTH = 0.225
BUCKET_HEIGHT = 0.216
BUCKET_WALL = 0.003

BUCKET_OPEN_WIDTH = 0.326
BUCKET_OPEN_HEIGHT = 0.228
BUCKET_OPEN_BOTTOM = 0.034
BUCKET_JOINT_Z = 0.038

REAR_DOOR_WIDTH = 0.302
REAR_DOOR_HEIGHT = 0.248
REAR_DOOR_THICKNESS = 0.006
REAR_DOOR_CENTER_Z = 0.430

POD_WIDTH = 0.132
POD_DEPTH = 0.108
POD_HEIGHT = 0.028


def _build_body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.026)
        .edges(">Z")
        .fillet(0.012)
    )

    inner = (
        cq.Workplane("XY")
        .box(
            WIDTH - 2.0 * WALL,
            DEPTH - 2.0 * WALL,
            HEIGHT - 2.0 * WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, WALL))
    )

    shell = outer.cut(inner)

    bucket_opening = (
        cq.Workplane("XY")
        .box(
            BUCKET_OPEN_WIDTH,
            0.080,
            BUCKET_OPEN_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, DEPTH / 2.0 - 0.040, BUCKET_OPEN_BOTTOM + BUCKET_OPEN_HEIGHT / 2.0))
    )

    rear_filter_opening = (
        cq.Workplane("XY")
        .box(
            REAR_DOOR_WIDTH - 0.028,
            0.080,
            REAR_DOOR_HEIGHT - 0.028,
            centered=(True, True, True),
        )
        .translate((0.0, -DEPTH / 2.0 + 0.040, REAR_DOOR_CENTER_Z))
    )

    return shell.cut(bucket_opening).cut(rear_filter_opening)


def _projected_overlap(min_a: float, max_a: float, min_b: float, max_b: float) -> float:
    return min(max_a, max_b) - max(min_a, min_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="console_dehumidifier")

    body_white = model.material("body_white", rgba=(0.92, 0.93, 0.91, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.74, 0.76, 0.78, 1.0))
    bucket_white = model.material("bucket_white", rgba=(0.95, 0.95, 0.94, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.20, 0.22, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.11, 0.12, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.74, 0.20, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "dehumidifier_body_shell"),
        material=body_white,
        name="shell",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.304, 0.148),
                frame=0.012,
                face_thickness=0.004,
                duct_depth=0.012,
                duct_wall=0.0025,
                slat_pitch=0.016,
                slat_width=0.008,
                slat_angle_deg=32.0,
                corner_radius=0.008,
                slats=VentGrilleSlats(
                    profile="airfoil",
                    direction="down",
                    divider_count=1,
                    divider_width=0.004,
                ),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.0015),
                sleeve=VentGrilleSleeve(style="short", depth=0.010, wall=0.0025),
                center=False,
            ),
            "dehumidifier_front_grille",
        ),
        origin=Origin(
            xyz=(0.0, DEPTH / 2.0 - 0.006, 0.454),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_grey,
        name="intake_grille",
    )
    for index, x_sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.012, 0.200, 0.034)),
            origin=Origin(
                xyz=(
                    x_sign * (BUCKET_WIDTH / 2.0 + 0.008),
                    -0.005,
                    0.021,
                )
            ),
            material=trim_grey,
            name=f"bucket_rail_{index}",
        )

    bucket = model.part("bucket")
    bucket.visual(
        Box((BUCKET_WIDTH, BUCKET_DEPTH, BUCKET_WALL)),
        origin=Origin(xyz=(0.0, -BUCKET_DEPTH / 2.0, BUCKET_WALL / 2.0)),
        material=bucket_white,
        name="bucket_floor",
    )
    bucket.visual(
        Box((BUCKET_WALL, BUCKET_DEPTH, BUCKET_HEIGHT)),
        origin=Origin(
            xyz=(
                -BUCKET_WIDTH / 2.0 + BUCKET_WALL / 2.0,
                -BUCKET_DEPTH / 2.0,
                BUCKET_HEIGHT / 2.0,
            )
        ),
        material=bucket_white,
        name="bucket_side_0",
    )
    bucket.visual(
        Box((BUCKET_WALL, BUCKET_DEPTH, BUCKET_HEIGHT)),
        origin=Origin(
            xyz=(
                BUCKET_WIDTH / 2.0 - BUCKET_WALL / 2.0,
                -BUCKET_DEPTH / 2.0,
                BUCKET_HEIGHT / 2.0,
            )
        ),
        material=bucket_white,
        name="bucket_side_1",
    )
    bucket.visual(
        Box((BUCKET_WIDTH - 2.0 * BUCKET_WALL, BUCKET_WALL, BUCKET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -BUCKET_DEPTH + BUCKET_WALL / 2.0,
                BUCKET_HEIGHT / 2.0,
            )
        ),
        material=bucket_white,
        name="bucket_back",
    )
    bucket.visual(
        Box((BUCKET_WIDTH - 2.0 * BUCKET_WALL, BUCKET_WALL, 0.088)),
        origin=Origin(xyz=(0.0, -BUCKET_WALL / 2.0, 0.044)),
        material=bucket_white,
        name="front_lower_wall",
    )
    bucket.visual(
        Box((BUCKET_WIDTH - 2.0 * BUCKET_WALL, BUCKET_WALL, 0.016)),
        origin=Origin(
            xyz=(
                0.0,
                -BUCKET_WALL / 2.0,
                BUCKET_HEIGHT - 0.008,
            )
        ),
        material=bucket_white,
        name="front_grip_lip",
    )
    for index, x_sign in enumerate((-1.0, 1.0)):
        bucket.visual(
            Box((0.014, 0.150, 0.010)),
            origin=Origin(
                xyz=(
                    x_sign * (BUCKET_WIDTH / 2.0 + 0.007),
                    -0.115,
                    0.005,
                )
            ),
            material=trim_grey,
            name=f"runner_{index}",
        )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((POD_WIDTH, POD_DEPTH, POD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, POD_HEIGHT / 2.0)),
        material=charcoal,
        name="pod_body",
    )
    control_pod.visual(
        Box((POD_WIDTH - 0.020, POD_DEPTH - 0.018, 0.002)),
        origin=Origin(xyz=(0.002, 0.0, POD_HEIGHT - 0.001)),
        material=trim_grey,
        name="pod_face",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.019, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=button_black,
        name="dial_cap",
    )
    timer_dial.visual(
        Box((0.004, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, 0.011, 0.013)),
        material=trim_grey,
        name="dial_pointer",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=indicator_red,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.0055, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=button_black,
        name="button_stem",
    )

    for suffix in ("0", "1"):
        button = model.part(f"humidity_button_{suffix}")
        button.visual(
            Box((0.016, 0.014, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=trim_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.008, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=button_black,
            name="button_stem",
        )

    rear_filter_door = model.part("rear_filter_door")
    rear_filter_door.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (REAR_DOOR_WIDTH, REAR_DOOR_HEIGHT),
                REAR_DOOR_THICKNESS,
                slot_size=(0.030, 0.005),
                pitch=(0.040, 0.016),
                frame=0.014,
                corner_radius=0.010,
                slot_angle_deg=0.0,
                stagger=True,
                center=True,
            ),
            "rear_filter_door",
        ),
        origin=Origin(xyz=(-REAR_DOOR_WIDTH / 2.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_grey,
        name="door_panel",
    )
    rear_filter_door.visual(
        Box((0.012, 0.006, 0.040)),
        origin=Origin(xyz=(-REAR_DOOR_WIDTH + 0.006, -0.005, 0.0)),
        material=charcoal,
        name="pull_tab",
    )

    bucket_joint = model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.0005, BUCKET_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.22,
            lower=0.0,
            upper=0.165,
        ),
    )

    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(0.086, 0.018, HEIGHT - 0.0005)),
    )

    rear_door_joint = model.articulation(
        "body_to_rear_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_filter_door,
        origin=Origin(
            xyz=(REAR_DOOR_WIDTH / 2.0, -DEPTH / 2.0 - REAR_DOOR_THICKNESS / 2.0, REAR_DOOR_CENTER_Z)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )

    timer_mount = place_on_face_uv(control_pod, "+z", uv=(0.26, 0.56))
    model.articulation(
        "control_pod_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=control_pod,
        child=timer_dial,
        origin=timer_mount,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    power_mount = place_on_face_uv(control_pod, "+z", uv=(0.74, 0.67))
    model.articulation(
        "control_pod_to_power_button",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=power_button,
        origin=power_mount,
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0022,
        ),
    )

    for joint_name, child_name, uv in (
        ("control_pod_to_humidity_button_0", "humidity_button_0", (0.62, 0.29)),
        ("control_pod_to_humidity_button_1", "humidity_button_1", (0.81, 0.29)),
    ):
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=model.get_part(child_name),
            origin=place_on_face_uv(control_pod, "+z", uv=uv),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0018,
            ),
        )

    bucket_joint.meta["prompt_role"] = "water_bucket"
    rear_door_joint.meta["prompt_role"] = "filter_access_door"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    control_pod = object_model.get_part("control_pod")
    timer_dial = object_model.get_part("timer_dial")
    power_button = object_model.get_part("power_button")
    humidity_button_0 = object_model.get_part("humidity_button_0")
    humidity_button_1 = object_model.get_part("humidity_button_1")
    rear_filter_door = object_model.get_part("rear_filter_door")

    bucket_joint = object_model.get_articulation("body_to_bucket")
    timer_joint = object_model.get_articulation("control_pod_to_timer_dial")
    power_joint = object_model.get_articulation("control_pod_to_power_button")
    humidity_joint_0 = object_model.get_articulation("control_pod_to_humidity_button_0")
    humidity_joint_1 = object_model.get_articulation("control_pod_to_humidity_button_1")
    rear_door_joint = object_model.get_articulation("body_to_rear_filter_door")

    ctx.expect_within(
        bucket,
        body,
        axes="x",
        margin=0.020,
        name="bucket stays centered laterally in the body",
    )
    ctx.expect_within(
        control_pod,
        body,
        axes="x",
        margin=0.004,
        name="control pod stays on the top deck footprint",
    )

    body_aabb = ctx.part_world_aabb(body)
    bucket_aabb = ctx.part_world_aabb(bucket)
    door_aabb = ctx.part_world_aabb(rear_filter_door)
    bucket_rest_pos = ctx.part_world_position(bucket)
    dial_rest_pos = ctx.part_world_position(timer_dial)
    power_rest_pos = ctx.part_world_position(power_button)
    humidity_0_rest_pos = ctx.part_world_position(humidity_button_0)
    humidity_1_rest_pos = ctx.part_world_position(humidity_button_1)

    ctx.check(
        "bucket front sits nearly flush with the cabinet opening",
        body_aabb is not None
        and bucket_aabb is not None
        and abs(body_aabb[1][1] - bucket_aabb[1][1]) <= 0.008,
        details=f"body_aabb={body_aabb}, bucket_aabb={bucket_aabb}",
    )
    ctx.check(
        "rear filter door closes onto the rear face",
        body_aabb is not None
        and door_aabb is not None
        and abs(door_aabb[1][1] - body_aabb[0][1]) <= 0.002,
        details=f"body_aabb={body_aabb}, door_aabb={door_aabb}",
    )

    bucket_limits = bucket_joint.motion_limits
    if bucket_limits is not None and bucket_limits.upper is not None:
        with ctx.pose({bucket_joint: bucket_limits.upper}):
            bucket_open_aabb = ctx.part_world_aabb(bucket)
            bucket_open_pos = ctx.part_world_position(bucket)
        overlap_y = None
        if body_aabb is not None and bucket_open_aabb is not None:
            overlap_y = _projected_overlap(
                float(body_aabb[0][1]),
                float(body_aabb[1][1]),
                float(bucket_open_aabb[0][1]),
                float(bucket_open_aabb[1][1]),
            )
        ctx.check(
            "bucket slides outward at full extension",
            bucket_rest_pos is not None
            and bucket_open_pos is not None
            and bucket_open_pos[1] > bucket_rest_pos[1] + 0.12,
            details=f"rest={bucket_rest_pos}, open={bucket_open_pos}",
        )
        ctx.check(
            "bucket remains retained on the guide path when extended",
            overlap_y is not None and overlap_y > 0.045,
            details=f"overlap_y={overlap_y}, body_aabb={body_aabb}, bucket_open_aabb={bucket_open_aabb}",
        )

    with ctx.pose({timer_joint: 1.7}):
        dial_turn_pos = ctx.part_world_position(timer_dial)
    ctx.check(
        "timer dial rotates in place on the pod",
        dial_rest_pos is not None
        and dial_turn_pos is not None
        and all(abs(dial_turn_pos[i] - dial_rest_pos[i]) <= 1e-6 for i in range(3)),
        details=f"rest={dial_rest_pos}, turned={dial_turn_pos}",
    )

    with ctx.pose({power_joint: 0.0020}):
        power_pressed_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power button depresses into the control pod",
        power_rest_pos is not None
        and power_pressed_pos is not None
        and power_pressed_pos[2] < power_rest_pos[2] - 0.0015,
        details=f"rest={power_rest_pos}, pressed={power_pressed_pos}",
    )

    with ctx.pose({humidity_joint_0: 0.0016, humidity_joint_1: 0.0016}):
        humidity_0_pressed_pos = ctx.part_world_position(humidity_button_0)
        humidity_1_pressed_pos = ctx.part_world_position(humidity_button_1)
    ctx.check(
        "humidity buttons depress independently",
        humidity_0_rest_pos is not None
        and humidity_1_rest_pos is not None
        and humidity_0_pressed_pos is not None
        and humidity_1_pressed_pos is not None
        and humidity_0_pressed_pos[2] < humidity_0_rest_pos[2] - 0.0012
        and humidity_1_pressed_pos[2] < humidity_1_rest_pos[2] - 0.0012,
        details=(
            f"rest0={humidity_0_rest_pos}, pressed0={humidity_0_pressed_pos}, "
            f"rest1={humidity_1_rest_pos}, pressed1={humidity_1_pressed_pos}"
        ),
    )

    rear_door_limits = rear_door_joint.motion_limits
    if rear_door_limits is not None and rear_door_limits.upper is not None:
        with ctx.pose({rear_door_joint: rear_door_limits.upper}):
            open_door_aabb = ctx.part_world_aabb(rear_filter_door)
        ctx.check(
            "rear filter door swings clear of the cabinet rear face",
            body_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[0][1] < body_aabb[0][1] - 0.055,
            details=f"body_aabb={body_aabb}, open_door_aabb={open_door_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
