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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.27, 0.29, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.80, 0.83, 1.0))
    cap_black = model.material("cap_black", rgba=(0.07, 0.07, 0.08, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.96, 0.84, 1.34)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
    )

    frame.visual(
        Box((0.96, 0.84, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_trim,
        name="deck_plate",
    )
    frame.visual(
        Box((0.84, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_trim,
        name="threshold_spine",
    )

    post_positions = (
        (0.44, 0.37),
        (0.44, -0.37),
        (-0.44, 0.37),
        (-0.44, -0.37),
    )
    for index, (x_pos, y_pos) in enumerate(post_positions):
        frame.visual(
            Box((0.06, 0.06, 1.22)),
            origin=Origin(xyz=(x_pos, y_pos, 0.66)),
            material=frame_paint,
            name=f"post_{index}",
        )

    frame.visual(
        Box((0.84, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.37, 1.28)),
        material=frame_paint,
        name="front_top_rail",
    )
    frame.visual(
        Box((0.84, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.37, 1.28)),
        material=frame_paint,
        name="rear_top_rail",
    )
    frame.visual(
        Box((0.06, 0.68, 0.06)),
        origin=Origin(xyz=(0.44, 0.0, 1.28)),
        material=frame_paint,
        name="right_top_rail",
    )
    frame.visual(
        Box((0.06, 0.68, 0.06)),
        origin=Origin(xyz=(-0.44, 0.0, 1.28)),
        material=frame_paint,
        name="left_top_rail",
    )

    frame.visual(
        Box((0.84, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.37, 0.52)),
        material=frame_paint,
        name="front_guard_rail",
    )
    frame.visual(
        Box((0.84, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, -0.37, 0.52)),
        material=frame_paint,
        name="rear_guard_rail",
    )

    frame.visual(
        Cylinder(radius=0.065, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=frame_paint,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.082, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.865)),
        material=dark_trim,
        name="bearing_shoulder",
    )
    frame.visual(
        Box((0.16, 0.68, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
        material=dark_trim,
        name="bearing_cage",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=cap_black,
        name="top_cap",
    )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.18),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    rotor.visual(
        Cylinder(radius=0.10, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_trim,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=frame_paint,
        name="rotor_lower_skirt",
    )
    rotor.visual(
        Cylinder(radius=0.07, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=cap_black,
        name="hub_cap",
    )

    arm_length = 0.32
    arm_radius = 0.021
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(0.5 * arm_length * c, 0.5 * arm_length * s, 0.01),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=0.05),
            origin=Origin(
                xyz=((arm_length + 0.025) * c, (arm_length + 0.025) * s, 0.01),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=cap_black,
            name=f"arm_tip_{index}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
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
    rotor = object_model.get_part("rotor")
    rotor_joint = object_model.get_articulation("frame_to_rotor")

    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.02,
        name="rotor sweep stays within the outer frame footprint",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="rotor_lower_skirt",
        negative_elem="deck_plate",
        min_gap=0.82,
        max_gap=0.95,
        name="rotor assembly sits high above the deck plate",
    )

    with ctx.pose({rotor_joint: math.pi / 3.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.02,
            name="rotor remains contained after a third-turn rotation",
        )

    frame_aabb = ctx.part_world_aabb(frame)
    rotor_aabb = ctx.part_world_aabb(rotor)
    if frame_aabb is not None and rotor_aabb is not None:
        frame_min, frame_max = frame_aabb
        rotor_min, rotor_max = rotor_aabb
        frame_dx = frame_max[0] - frame_min[0]
        frame_dy = frame_max[1] - frame_min[1]
        frame_dz = frame_max[2] - frame_min[2]
        rotor_center_z = 0.5 * (rotor_min[2] + rotor_max[2])
        ctx.check(
            "frame reads as a tall vertical gate",
            frame_dz > frame_dx and frame_dz > frame_dy,
            details=f"dims=({frame_dx:.3f}, {frame_dy:.3f}, {frame_dz:.3f})",
        )
        ctx.check(
            "rotor head sits in the upper third of the structure",
            rotor_center_z > frame_min[2] + 0.72 * frame_dz,
            details=f"rotor_center_z={rotor_center_z:.3f}, frame_min_z={frame_min[2]:.3f}, frame_dz={frame_dz:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
