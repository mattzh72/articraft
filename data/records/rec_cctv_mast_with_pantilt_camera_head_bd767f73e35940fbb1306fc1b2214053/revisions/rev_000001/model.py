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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


STATIONS = (
    ("east", 0.0),
    ("north", math.pi / 2.0),
    ("west", math.pi),
    ("south", -math.pi / 2.0),
)

POLE_HEIGHT = 4.50
POLE_RADIUS = 0.080
HUB_RADIUS = 0.140
HUB_LENGTH = 0.140
HUB_CENTER_Z = POLE_HEIGHT + HUB_LENGTH / 2.0
HUB_CAP_LENGTH = 0.028
SOCKET_LENGTH = 0.085

ARM_RADIUS = 0.024
ARM_LENGTH = 0.580
DROP_POST_RADIUS = 0.021
DROP_POST_LENGTH = 0.120
BRACE_RADIUS = 0.011

PAN_SPINDLE_RADIUS = 0.020
PAN_SPINDLE_LENGTH = 0.040
PAN_CANISTER_RADIUS = 0.032
PAN_CANISTER_LENGTH = 0.056

TILT_ORIGIN = (0.035, 0.0, -0.078)
TILT_LIMITS = MotionLimits(
    effort=18.0,
    velocity=1.8,
    lower=-1.05,
    upper=0.55,
)


def _radial_origin(radius: float, yaw: float, z: float, *, rpy: tuple[float, float, float]) -> Origin:
    return Origin(
        xyz=(radius * math.cos(yaw), radius * math.sin(yaw), z),
        rpy=rpy,
    )


def _build_arm_part(model: ArticulatedObject, station: str) -> None:
    steel = "galvanized_steel"
    arm = model.part(f"arm_{station}")

    arm.visual(
        Cylinder(radius=ARM_RADIUS, length=ARM_LENGTH),
        origin=Origin(xyz=(ARM_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="arm_tube",
    )
    arm.visual(
        Cylinder(radius=DROP_POST_RADIUS, length=DROP_POST_LENGTH),
        origin=Origin(xyz=(ARM_LENGTH, 0.0, -DROP_POST_LENGTH / 2.0)),
        material=steel,
        name="drop_post",
    )

    brace_start = (0.165, 0.0, -0.018)
    brace_end = (ARM_LENGTH - 0.040, 0.0, -DROP_POST_LENGTH + 0.022)
    dx = brace_end[0] - brace_start[0]
    dz = brace_end[2] - brace_start[2]
    brace_length = math.sqrt(dx * dx + dz * dz)
    brace_pitch = math.atan2(dx, dz)
    arm.visual(
        Cylinder(radius=BRACE_RADIUS, length=brace_length),
        origin=Origin(
            xyz=((brace_start[0] + brace_end[0]) / 2.0, 0.0, (brace_start[2] + brace_end[2]) / 2.0),
            rpy=(0.0, brace_pitch, 0.0),
        ),
        material=steel,
        name="support_brace",
    )


def _build_pan_part(model: ArticulatedObject, station: str) -> None:
    pan = model.part(f"pan_{station}")

    pan.visual(
        Cylinder(radius=PAN_SPINDLE_RADIUS, length=PAN_SPINDLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -PAN_SPINDLE_LENGTH / 2.0)),
        material="camera_dark",
        name="pan_spindle",
    )
    pan.visual(
        Cylinder(radius=PAN_CANISTER_RADIUS, length=PAN_CANISTER_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material="camera_dark",
        name="pan_canister",
    )
    pan.visual(
        Box((0.048, 0.114, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, -0.042)),
        material="camera_dark",
        name="yoke_bridge",
    )
    pan.visual(
        Box((0.064, 0.008, 0.088)),
        origin=Origin(xyz=(0.022, 0.053, -0.084)),
        material="camera_dark",
        name="left_cheek",
    )
    pan.visual(
        Box((0.064, 0.008, 0.088)),
        origin=Origin(xyz=(0.022, -0.053, -0.084)),
        material="camera_dark",
        name="right_cheek",
    )


def _build_camera_part(model: ArticulatedObject, station: str) -> None:
    camera = model.part(f"camera_{station}")

    camera.visual(
        Box((0.130, 0.074, 0.060)),
        origin=Origin(xyz=(0.060, 0.0, -0.030)),
        material="camera_dark",
        name="camera_shell",
    )
    camera.visual(
        Box((0.070, 0.086, 0.006)),
        origin=Origin(xyz=(0.102, 0.0, 0.002)),
        material="camera_black",
        name="sunshield",
    )
    camera.visual(
        Cylinder(radius=0.023, length=0.038),
        origin=Origin(xyz=(0.136, 0.0, -0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="lens_glass",
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.000, 0.042, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="camera_dark",
        name="left_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.000, -0.042, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="camera_dark",
        name="right_trunnion",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cluster_pole_cctv_mount")
    model.material("galvanized_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    model.material("camera_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("camera_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("lens_glass", rgba=(0.10, 0.12, 0.16, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=POLE_RADIUS, length=POLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT / 2.0)),
        material="galvanized_steel",
        name="pole_shaft",
    )
    mast.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, HUB_CENTER_Z)),
        material="galvanized_steel",
        name="hub_drum",
    )
    mast.visual(
        Cylinder(radius=HUB_RADIUS * 1.10, length=HUB_CAP_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT + HUB_LENGTH + HUB_CAP_LENGTH / 2.0)),
        material="galvanized_steel",
        name="hub_cap",
    )

    for station, yaw in STATIONS:
        mast.visual(
            Cylinder(radius=ARM_RADIUS, length=SOCKET_LENGTH),
            origin=_radial_origin(
                HUB_RADIUS + SOCKET_LENGTH / 2.0,
                yaw,
                HUB_CENTER_Z,
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material="galvanized_steel",
            name=f"socket_{station}",
        )

        _build_arm_part(model, station)
        _build_pan_part(model, station)
        _build_camera_part(model, station)

        arm = model.get_part(f"arm_{station}")
        pan = model.get_part(f"pan_{station}")
        camera = model.get_part(f"camera_{station}")

        model.articulation(
            f"mast_to_arm_{station}",
            ArticulationType.FIXED,
            parent=mast,
            child=arm,
            origin=_radial_origin(
                HUB_RADIUS + SOCKET_LENGTH,
                yaw,
                HUB_CENTER_Z,
                rpy=(0.0, 0.0, yaw),
            ),
        )
        model.articulation(
            f"arm_to_pan_{station}",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=pan,
            origin=Origin(xyz=(ARM_LENGTH, 0.0, -DROP_POST_LENGTH)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=2.4),
        )
        model.articulation(
            f"pan_to_camera_{station}",
            ArticulationType.REVOLUTE,
            parent=pan,
            child=camera,
            origin=Origin(xyz=TILT_ORIGIN),
            axis=(0.0, -1.0, 0.0),
            motion_limits=TILT_LIMITS,
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

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}

    for name in ["mast"]:
        ctx.check(f"{name} exists", name in part_names)

    for station, _ in STATIONS:
        for name in (f"arm_{station}", f"pan_{station}", f"camera_{station}"):
            ctx.check(f"{name} exists", name in part_names)
        for name in (f"mast_to_arm_{station}", f"arm_to_pan_{station}", f"pan_to_camera_{station}"):
            ctx.check(f"{name} exists", name in joint_names)

    mast = object_model.get_part("mast")

    for station, _ in STATIONS:
        arm = object_model.get_part(f"arm_{station}")
        pan = object_model.get_part(f"pan_{station}")
        camera = object_model.get_part(f"camera_{station}")
        pan_joint = object_model.get_articulation(f"arm_to_pan_{station}")
        tilt_joint = object_model.get_articulation(f"pan_to_camera_{station}")

        ctx.expect_contact(arm, mast, name=f"{station} arm is mounted to the hub")
        ctx.expect_contact(pan, arm, name=f"{station} pan bearing hangs from the arm")
        ctx.expect_contact(camera, pan, name=f"{station} camera body is carried by the tilt yoke")

        pan_limits = pan_joint.motion_limits
        tilt_limits = tilt_joint.motion_limits
        ctx.check(
            f"{station} pan joint is continuous",
            pan_joint.articulation_type == ArticulationType.CONTINUOUS
            and pan_joint.axis == (0.0, 0.0, 1.0)
            and pan_limits is not None
            and pan_limits.lower is None
            and pan_limits.upper is None,
            details=(
                f"type={pan_joint.articulation_type}, axis={pan_joint.axis}, "
                f"limits={None if pan_limits is None else (pan_limits.lower, pan_limits.upper)}"
            ),
        )
        ctx.check(
            f"{station} tilt joint has upward and downward travel",
            tilt_joint.articulation_type == ArticulationType.REVOLUTE
            and tilt_joint.axis == (0.0, -1.0, 0.0)
            and tilt_limits is not None
            and tilt_limits.lower is not None
            and tilt_limits.upper is not None
            and tilt_limits.lower < -0.8
            and tilt_limits.upper > 0.4,
            details=(
                f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}, "
                f"limits={None if tilt_limits is None else (tilt_limits.lower, tilt_limits.upper)}"
            ),
        )

    east_arm = object_model.get_part("arm_east")
    east_pan = object_model.get_part("pan_east")
    east_camera = object_model.get_part("camera_east")
    east_pan_joint = object_model.get_articulation("arm_to_pan_east")
    east_tilt_joint = object_model.get_articulation("pan_to_camera_east")

    def lens_center() -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(east_camera, elem="lens_barrel")
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    rest_lens = lens_center()
    with ctx.pose({east_pan_joint: math.pi / 2.0}):
        quarter_turn_lens = lens_center()
    ctx.check(
        "east camera pans about a vertical bearing",
        rest_lens is not None
        and quarter_turn_lens is not None
        and quarter_turn_lens[1] > rest_lens[1] + 0.10
        and quarter_turn_lens[0] < rest_lens[0] - 0.10,
        details=f"rest={rest_lens}, quarter_turn={quarter_turn_lens}",
    )

    with ctx.pose({east_tilt_joint: 0.45}):
        raised_lens = lens_center()
        ctx.expect_gap(
            east_arm,
            east_camera,
            axis="z",
            max_penetration=0.0,
            name="east raised camera stays below the arm",
        )
    ctx.check(
        "east camera tilts upward",
        rest_lens is not None and raised_lens is not None and raised_lens[2] > rest_lens[2] + 0.05,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    with ctx.pose({east_tilt_joint: -0.85}):
        lowered_lens = lens_center()
    ctx.check(
        "east camera tilts downward",
        rest_lens is not None and lowered_lens is not None and lowered_lens[2] < rest_lens[2] - 0.05,
        details=f"rest={rest_lens}, lowered={lowered_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
