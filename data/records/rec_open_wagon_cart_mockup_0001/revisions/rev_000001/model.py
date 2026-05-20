from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

HERE = Path(__file__).resolve().parent

WOOD = Material(name="weathered_wood", rgba=(0.58, 0.41, 0.24, 1.0))
PAINTED_METAL = Material(name="painted_metal", rgba=(0.18, 0.24, 0.25, 1.0))
STEEL = Material(name="brushed_steel", rgba=(0.67, 0.68, 0.70, 1.0))
RUBBER = Material(name="rubber_tire", rgba=(0.07, 0.07, 0.07, 1.0))

WHEEL_RADIUS = 0.12
WHEEL_WIDTH = 0.042
WHEEL_TRACK_Y = 0.255
REAR_AXLE_X = -0.24
FRONT_STEER_X = 0.24
AXLE_Z = 0.12


def _add_wheel_visuals(part) -> None:
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
    )
    part.visual(
        Cylinder(radius=0.094, length=0.035),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=STEEL,
    )
    part.visual(
        Cylinder(radius=0.036, length=0.054),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=PAINTED_METAL,
    )
    for side in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=0.053, length=0.008),
            origin=Origin(
                xyz=(0.0, side * 0.015, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=STEEL,
        )
        part.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(
                xyz=(0.0, side * 0.026, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=STEEL,
        )
        part.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(
                xyz=(0.0, side * 0.036, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=STEEL,
        )

    for spoke_index in range(8):
        angle = spoke_index * (pi / 4.0)
        part.visual(
            Box((0.012, 0.010, 0.078)),
            origin=Origin(
                xyz=(0.058 * sin(angle), 0.0, 0.058 * cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=PAINTED_METAL,
        )

    part.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_wagon_cart")
    model.materials.extend([WOOD, PAINTED_METAL, STEEL, RUBBER])

    body = model.part("body")
    body.visual(
        Box((0.62, 0.38, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.272)),
        material=WOOD,
    )
    for y in (-0.105, -0.035, 0.035, 0.105):
        body.visual(
            Box((0.58, 0.058, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.287)),
            material=WOOD,
        )
    for y in (-0.181, 0.181):
        body.visual(
            Box((0.58, 0.018, 0.110)),
            origin=Origin(xyz=(0.0, y, 0.338)),
            material=WOOD,
        )
        for x in (-0.18, 0.0, 0.18):
            body.visual(
                Box((0.020, 0.022, 0.110)),
                origin=Origin(xyz=(x, y, 0.338)),
                material=PAINTED_METAL,
            )
    for x in (-0.301, 0.301):
        body.visual(
            Box((0.018, 0.344, 0.100)),
            origin=Origin(xyz=(x, 0.0, 0.333)),
            material=WOOD,
        )
        for y in (-0.10, 0.10):
            body.visual(
                Box((0.022, 0.024, 0.100)),
                origin=Origin(xyz=(x, y, 0.333)),
                material=PAINTED_METAL,
            )

    for x in (-0.295, 0.295):
        for y in (-0.172, 0.172):
            body.visual(
                Box((0.028, 0.028, 0.132)),
                origin=Origin(xyz=(x, y, 0.338)),
                material=PAINTED_METAL,
            )

    for y in (-0.115, 0.115):
        body.visual(
            Box((0.52, 0.028, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.248)),
            material=PAINTED_METAL,
        )
    for x in (-0.21, 0.20):
        body.visual(
            Box((0.070, 0.270, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.248)),
            material=PAINTED_METAL,
        )

    body.visual(
        Box((0.090, 0.100, 0.028)),
        origin=Origin(xyz=(FRONT_STEER_X, 0.0, 0.235)),
        material=PAINTED_METAL,
    )
    body.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(FRONT_STEER_X, 0.0, 0.209)),
        material=STEEL,
    )
    body.visual(
        Box((0.110, 0.120, 0.016)),
        origin=Origin(xyz=(FRONT_STEER_X, 0.0, 0.189)),
        material=STEEL,
    )

    body.visual(
        Box((0.100, 0.452, 0.028)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, 0.201)),
        material=PAINTED_METAL,
    )
    body.visual(
        Box((0.080, 0.100, 0.042)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, 0.222)),
        material=STEEL,
    )
    for side in (-1.0, 1.0):
        body.visual(
            Box((0.080, 0.022, 0.100)),
            origin=Origin(xyz=(REAR_AXLE_X, side * 0.219, 0.165)),
            material=PAINTED_METAL,
        )
        body.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(
                xyz=(REAR_AXLE_X, side * WHEEL_TRACK_Y, AXLE_Z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=STEEL,
        )

    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.38, 0.22)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
    )

    front_steer = model.part("front_steer")
    front_steer.visual(
        Box((0.110, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=STEEL,
    )
    front_steer.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=STEEL,
    )
    front_steer.visual(
        Box((0.160, 0.100, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=PAINTED_METAL,
    )
    front_steer.visual(
        Box((0.100, 0.452, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=PAINTED_METAL,
    )
    front_steer.visual(
        Box((0.110, 0.090, 0.028)),
        origin=Origin(xyz=(0.085, 0.0, -0.055)),
        material=PAINTED_METAL,
    )
    for side in (-1.0, 1.0):
        front_steer.visual(
            Box((0.080, 0.022, 0.100)),
            origin=Origin(xyz=(0.0, side * 0.219, -0.065)),
            material=PAINTED_METAL,
        )
        front_steer.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(
                xyz=(0.0, side * WHEEL_TRACK_Y, -0.060),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=STEEL,
        )
        front_steer.visual(
            Box((0.030, 0.010, 0.042)),
            origin=Origin(xyz=(0.132, side * 0.022, -0.052)),
            material=STEEL,
        )
    front_steer.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(
            xyz=(0.125, 0.0, -0.055),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=STEEL,
    )
    front_steer.inertial = Inertial.from_geometry(
        Box((0.22, 0.36, 0.12)),
        mass=6.0,
        origin=Origin(xyz=(0.04, 0.0, -0.060)),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.028, 0.058, 0.022)),
        origin=Origin(),
        material=STEEL,
    )
    for side in (-1.0, 1.0):
        handle.visual(
            Cylinder(radius=0.009, length=0.340),
            origin=Origin(
                xyz=(0.170, side * 0.038, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=PAINTED_METAL,
        )
        handle.visual(
            Cylinder(radius=0.008, length=0.060),
            origin=Origin(xyz=(0.330, side * 0.038, 0.030)),
            material=PAINTED_METAL,
        )
    handle.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(
            xyz=(0.330, 0.0, 0.058),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=STEEL,
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(
            xyz=(0.330, 0.0, 0.058),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=RUBBER,
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.08)),
        mass=2.0,
        origin=Origin(xyz=(0.18, 0.0, 0.028)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(rear_left_wheel)

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(rear_right_wheel)

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(front_left_wheel)

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(front_right_wheel)

    model.articulation(
        "front_steer_swivel",
        ArticulationType.REVOLUTE,
        parent="body",
        child="front_steer",
        origin=Origin(xyz=(FRONT_STEER_X, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "pull_handle_pitch",
        ArticulationType.REVOLUTE,
        parent="front_steer",
        child="handle",
        origin=Origin(xyz=(0.125, 0.0, -0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.75,
            upper=0.45,
        ),
    )

    for name, parent, xyz in (
        (
            "rear_left_wheel_spin",
            "body",
            (REAR_AXLE_X, WHEEL_TRACK_Y, AXLE_Z),
        ),
        (
            "rear_right_wheel_spin",
            "body",
            (REAR_AXLE_X, -WHEEL_TRACK_Y, AXLE_Z),
        ),
        (
            "front_left_wheel_spin",
            "front_steer",
            (0.0, WHEEL_TRACK_Y, -0.060),
        ),
        (
            "front_right_wheel_spin",
            "front_steer",
            (0.0, -WHEEL_TRACK_Y, -0.060),
        ),
    ):
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=name.replace("_spin", ""),
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    for wheel_name, parent_name in (
        ("rear_left_wheel", "body"),
        ("rear_right_wheel", "body"),
        ("front_left_wheel", "front_steer"),
        ("front_right_wheel", "front_steer"),
    ):
        ctx.allow_overlap(
            wheel_name,
            parent_name,
            reason="A short stub axle intentionally sits inside the rotating wheel hub.",
        )
    ctx.allow_overlap(
        "handle",
        "front_steer",
        reason="The drawbar hinge pin passes through the front steering yoke.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("body", "front_steer", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_overlap("front_steer", "handle", axes="xy", min_overlap=0.02)
    ctx.expect_joint_motion_axis(
        "front_steer_swivel",
        "front_left_wheel",
        world_axis="x",
        direction="negative",
        min_delta=0.06,
    )
    ctx.expect_joint_motion_axis(
        "front_steer_swivel",
        "front_right_wheel",
        world_axis="x",
        direction="positive",
        min_delta=0.06,
    )
    ctx.expect_joint_motion_axis(
        "pull_handle_pitch",
        "handle",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    steer = object_model.get_articulation("front_steer_swivel")
    if (
        steer.motion_limits is None
        or steer.motion_limits.upper < 0.5
        or steer.motion_limits.lower > -0.5
    ):
        raise AssertionError("The front steering bolster should have a generous steering range.")

    for wheel_joint_name in (
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
        "front_left_wheel_spin",
        "front_right_wheel_spin",
    ):
        if (
            object_model.get_articulation(wheel_joint_name).articulation_type
            != ArticulationType.CONTINUOUS
        ):
            raise AssertionError(f"{wheel_joint_name} should be a continuous wheel joint.")

    rear_left = ctx.part_world_position("rear_left_wheel")
    rear_right = ctx.part_world_position("rear_right_wheel")
    front_left = ctx.part_world_position("front_left_wheel")
    front_right = ctx.part_world_position("front_right_wheel")
    handle_origin = ctx.part_world_position("handle")

    if not (rear_left[1] > 0.20 and rear_right[1] < -0.20):
        raise AssertionError("Rear wheels should sit clearly outside the cargo bed.")
    if not (front_left[0] > rear_left[0] + 0.40 and front_right[0] > rear_right[0] + 0.40):
        raise AssertionError("Front wheels should sit well ahead of the rear axle.")
    if not (
        abs(rear_left[2] - rear_right[2]) < 0.005 and abs(front_left[2] - rear_left[2]) < 0.005
    ):
        raise AssertionError("All four wheel hubs should ride at the same height.")
    if not (handle_origin[0] > front_left[0] and abs(handle_origin[1]) < 0.08):
        raise AssertionError("The pull handle should lead the front steering assembly.")

    with ctx.pose(front_steer_swivel=0.40):
        steered_left = ctx.part_world_position("front_left_wheel")
        steered_right = ctx.part_world_position("front_right_wheel")
        if not (
            steered_left[0] < front_left[0] - 0.08 and steered_right[0] > front_right[0] + 0.08
        ):
            raise AssertionError(
                "Positive steering should sweep the front wheels into a visible turn."
            )

    with ctx.pose(front_steer_swivel=-0.40):
        steered_left = ctx.part_world_position("front_left_wheel")
        steered_right = ctx.part_world_position("front_right_wheel")
        if not (
            steered_left[0] > front_left[0] + 0.08 and steered_right[0] < front_right[0] - 0.08
        ):
            raise AssertionError(
                "Negative steering should sweep the front wheels in the opposite direction."
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
