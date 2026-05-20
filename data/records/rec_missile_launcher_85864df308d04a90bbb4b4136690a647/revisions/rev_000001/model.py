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


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    xy = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(xy, dz)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_missile_launcher")

    base_olive = model.material("base_olive", rgba=(0.31, 0.36, 0.25, 1.0))
    pod_olive = model.material("pod_olive", rgba=(0.38, 0.43, 0.30, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.18, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    sight_glass = model.material("sight_glass", rgba=(0.12, 0.23, 0.26, 0.55))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.11, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_metal,
        name="lower_hub",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=dark_metal,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=dark_metal,
        name="bearing_cap",
    )

    leg_top_radius = 0.11
    foot_radius = 0.62
    leg_attach_z = 0.24
    foot_z = 0.05
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        start = (
            leg_top_radius * math.cos(angle),
            leg_top_radius * math.sin(angle),
            leg_attach_z,
        )
        end = (
            foot_radius * math.cos(angle),
            foot_radius * math.sin(angle),
            foot_z,
        )
        leg_origin, leg_length = _cylinder_between(start, end)
        base.visual(
            Cylinder(radius=0.027, length=leg_length),
            origin=leg_origin,
            material=base_olive,
            name=f"leg_{index}",
        )

        brace_start = (
            0.05 * math.cos(angle),
            0.05 * math.sin(angle),
            0.18,
        )
        brace_end = (
            0.30 * math.cos(angle),
            0.30 * math.sin(angle),
            0.14,
        )
        brace_origin, brace_length = _cylinder_between(brace_start, brace_end)
        base.visual(
            Cylinder(radius=0.015, length=brace_length),
            origin=brace_origin,
            material=dark_metal,
            name=f"brace_{index}",
        )

        foot_x = foot_radius * math.cos(angle)
        foot_y = foot_radius * math.sin(angle)
        foot_yaw = angle + math.pi / 2.0
        base.visual(
            Box((0.14, 0.055, 0.022)),
            origin=Origin(xyz=(foot_x, foot_y, 0.021), rpy=(0.0, 0.0, foot_yaw)),
            material=black_rubber,
            name=f"foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((1.30, 1.30, 0.98)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="yaw_ring",
    )
    turntable.visual(
        Box((0.30, 0.24, 0.05)),
        origin=Origin(xyz=(0.05, 0.0, 0.085)),
        material=base_olive,
        name="turret_deck",
    )
    turntable.visual(
        Box((0.14, 0.18, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 0.145)),
        material=dark_metal,
        name="center_bearing_block",
    )
    turntable.visual(
        Box((0.20, 0.04, 0.24)),
        origin=Origin(xyz=(0.04, 0.17, 0.19)),
        material=base_olive,
        name="left_yoke",
    )
    turntable.visual(
        Box((0.20, 0.04, 0.24)),
        origin=Origin(xyz=(0.04, -0.17, 0.19)),
        material=base_olive,
        name="right_yoke",
    )
    turntable.visual(
        Box((0.14, 0.30, 0.05)),
        origin=Origin(xyz=(0.00, 0.0, 0.11)),
        material=dark_metal,
        name="lower_crossbrace",
    )
    turntable.visual(
        Box((0.06, 0.30, 0.04)),
        origin=Origin(xyz=(0.12, 0.0, 0.29)),
        material=dark_metal,
        name="upper_crossbrace",
    )
    turntable.inertial = Inertial.from_geometry(
        Box((0.34, 0.38, 0.34)),
        mass=12.0,
        origin=Origin(xyz=(0.04, 0.0, 0.17)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Box((0.18, 0.24, 0.05)),
        origin=Origin(xyz=(0.19, 0.0, -0.03)),
        material=dark_metal,
        name="rear_bridge",
    )
    cradle.visual(
        Box((0.44, 0.05, 0.07)),
        origin=Origin(xyz=(0.32, 0.095, -0.005)),
        material=base_olive,
        name="left_rail",
    )
    cradle.visual(
        Box((0.44, 0.05, 0.07)),
        origin=Origin(xyz=(0.32, -0.095, -0.005)),
        material=base_olive,
        name="right_rail",
    )
    cradle.visual(
        Box((0.14, 0.05, 0.06)),
        origin=Origin(xyz=(0.05, 0.125, 0.005)),
        material=dark_metal,
        name="left_trunnion_housing",
    )
    cradle.visual(
        Box((0.14, 0.05, 0.06)),
        origin=Origin(xyz=(0.05, -0.125, 0.005)),
        material=dark_metal,
        name="right_trunnion_housing",
    )
    cradle.visual(
        Cylinder(radius=0.028, length=0.03),
        origin=Origin(xyz=(0.0, 0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    cradle.visual(
        Cylinder(radius=0.028, length=0.03),
        origin=Origin(xyz=(0.0, -0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    cradle.visual(
        Box((0.34, 0.18, 0.06)),
        origin=Origin(xyz=(0.55, 0.0, 0.03)),
        material=base_olive,
        name="saddle_block",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.78, 0.30, 0.20)),
        mass=10.0,
        origin=Origin(xyz=(0.31, 0.0, 0.02)),
    )

    model.articulation(
        "turntable_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.03, 0.0, 0.21)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.8,
            lower=math.radians(-20.0),
            upper=math.radians(55.0),
        ),
    )

    launch_pod = model.part("launch_pod")
    launch_pod.visual(
        Box((0.96, 0.44, 0.30)),
        material=pod_olive,
        name="pod_shell",
    )
    launch_pod.visual(
        Box((0.03, 0.40, 0.26)),
        origin=Origin(xyz=(0.465, 0.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    for row_index, z in enumerate((0.075, -0.075), start=1):
        for col_index, y in enumerate((-0.11, 0.11), start=1):
            launch_pod.visual(
                Cylinder(radius=0.074, length=0.04),
                origin=Origin(
                    xyz=(0.48, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black_rubber,
                name=f"launch_cell_{row_index}_{col_index}",
            )
    launch_pod.visual(
        Box((0.26, 0.16, 0.07)),
        origin=Origin(xyz=(-0.20, 0.0, 0.185)),
        material=base_olive,
        name="top_blister",
    )
    launch_pod.visual(
        Box((0.32, 0.16, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, -0.17)),
        material=dark_metal,
        name="pod_mount",
    )
    launch_pod.inertial = Inertial.from_geometry(
        Box((0.96, 0.44, 0.37)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "cradle_to_launch_pod",
        ArticulationType.FIXED,
        parent=cradle,
        child=launch_pod,
        origin=Origin(xyz=(0.58, 0.0, 0.26)),
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

    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    launch_pod = object_model.get_part("launch_pod")
    yaw_joint = object_model.get_articulation("base_to_turntable")
    pitch_joint = object_model.get_articulation("turntable_to_cradle")

    ctx.expect_contact(
        turntable,
        base,
        elem_a="yaw_ring",
        elem_b="bearing_cap",
        name="yaw ring sits on pedestal bearing",
    )
    ctx.expect_contact(
        cradle,
        turntable,
        elem_a="left_trunnion",
        elem_b="left_yoke",
        name="left trunnion seats against the yoke support",
    )
    ctx.expect_contact(
        launch_pod,
        cradle,
        elem_a="pod_mount",
        elem_b="saddle_block",
        name="launch pod mount bears on the cradle saddle",
    )
    ctx.expect_overlap(
        launch_pod,
        cradle,
        axes="xy",
        elem_a="pod_mount",
        elem_b="saddle_block",
        min_overlap=0.12,
        name="pod mount overlaps the saddle footprint",
    )

    rest_pos = ctx.part_world_position(launch_pod)
    with ctx.pose({yaw_joint: math.radians(45.0)}):
        yawed_pos = ctx.part_world_position(launch_pod)
    ctx.check(
        "positive yaw swings the pod toward +Y",
        rest_pos is not None and yawed_pos is not None and yawed_pos[1] > rest_pos[1] + 0.20,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    with ctx.pose({pitch_joint: math.radians(35.0)}):
        pitched_pos = ctx.part_world_position(launch_pod)
    ctx.check(
        "positive pitch raises the pod",
        rest_pos is not None and pitched_pos is not None and pitched_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, pitched={pitched_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
