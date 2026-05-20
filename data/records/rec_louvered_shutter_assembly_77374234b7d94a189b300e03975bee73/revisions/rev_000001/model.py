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
    mesh_from_geometry,
    section_loft,
)


PANEL_WIDTH = 0.60
PANEL_HEIGHT = 1.10
FRAME_THICKNESS = 0.028
STILE_WIDTH = 0.060
OUTER_WALL_WIDTH = 0.018
LIP_DEPTH = STILE_WIDTH - OUTER_WALL_WIDTH
LIP_THICKNESS = 0.008
TOP_RAIL_HEIGHT = 0.080
BOTTOM_RAIL_HEIGHT = 0.080
OPENING_WIDTH = PANEL_WIDTH - 2.0 * STILE_WIDTH
OPENING_HEIGHT = PANEL_HEIGHT - TOP_RAIL_HEIGHT - BOTTOM_RAIL_HEIGHT
SLAT_COUNT = 18
SLAT_CHORD = 0.048
SLAT_THICKNESS = 0.008
SLAT_PITCH = 0.051
PIN_RADIUS = 0.0045
PIN_LENGTH = LIP_DEPTH
SLAT_LIMIT = math.radians(55.0)


def _slat_names() -> list[str]:
    return [f"slat_{index:02d}" for index in range(SLAT_COUNT)]


def _slat_joint_names() -> list[str]:
    return [f"frame_to_slat_{index:02d}" for index in range(SLAT_COUNT)]


def _slat_center_z(index: int) -> float:
    return (index - (SLAT_COUNT - 1) * 0.5) * SLAT_PITCH


def _slat_profile() -> list[tuple[float, float]]:
    half_chord = SLAT_CHORD * 0.5
    half_thickness = SLAT_THICKNESS * 0.5
    return [
        (0.0000, half_chord),
        (0.0016, half_chord * 0.92),
        (half_thickness * 0.95, half_chord * 0.32),
        (half_thickness, -half_chord * 0.12),
        (half_thickness * 0.62, -half_chord * 0.72),
        (0.0000, -half_chord),
        (-half_thickness * 0.45, -half_chord * 0.62),
        (-half_thickness * 0.85, -half_chord * 0.10),
        (-half_thickness * 0.90, half_chord * 0.34),
        (-half_thickness * 0.35, half_chord * 0.80),
    ]


def _build_slat_mesh():
    half_length = OPENING_WIDTH * 0.5
    profile = _slat_profile()
    left_section = [( -half_length, y, z) for y, z in profile]
    right_section = [(half_length, y, z) for y, z in profile]
    return mesh_from_geometry(section_loft([left_section, right_section]), "shutter_slat")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_shutter_panel")

    frame_paint = model.material("frame_paint", rgba=(0.95, 0.96, 0.94, 1.0))
    slat_paint = model.material("slat_paint", rgba=(0.90, 0.91, 0.88, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.62, 0.64, 0.67, 1.0))

    slat_mesh = _build_slat_mesh()

    frame = model.part("frame")
    frame.visual(
        Box((OUTER_WALL_WIDTH, FRAME_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(
            xyz=(-PANEL_WIDTH * 0.5 + OUTER_WALL_WIDTH * 0.5, 0.0, 0.0)
        ),
        material=frame_paint,
        name="left_outer_wall",
    )
    frame.visual(
        Box((OUTER_WALL_WIDTH, FRAME_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(
            xyz=(PANEL_WIDTH * 0.5 - OUTER_WALL_WIDTH * 0.5, 0.0, 0.0)
        ),
        material=frame_paint,
        name="right_outer_wall",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        lip_center_x = side_sign * (
            PANEL_WIDTH * 0.5 - OUTER_WALL_WIDTH - LIP_DEPTH * 0.5
        )
        for y_sign, lip_name in ((-1.0, "rear"), (1.0, "front")):
            frame.visual(
                Box((LIP_DEPTH, LIP_THICKNESS, OPENING_HEIGHT)),
                origin=Origin(
                    xyz=(
                        lip_center_x,
                        y_sign * (FRAME_THICKNESS * 0.5 - LIP_THICKNESS * 0.5),
                        0.0,
                    )
                ),
                material=frame_paint,
                name=f"{side_name}_{lip_name}_lip",
            )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_THICKNESS, TOP_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_HEIGHT * 0.5 - TOP_RAIL_HEIGHT * 0.5)),
        material=frame_paint,
        name="top_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_THICKNESS, BOTTOM_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, -PANEL_HEIGHT * 0.5 + BOTTOM_RAIL_HEIGHT * 0.5)
        ),
        material=frame_paint,
        name="bottom_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, FRAME_THICKNESS, PANEL_HEIGHT)),
        mass=4.8,
        origin=Origin(),
    )

    for index, slat_name in enumerate(_slat_names()):
        slat = model.part(slat_name)
        slat.visual(
            slat_mesh,
            material=slat_paint,
            name="blade",
        )
        for side_sign, pin_name in ((-1.0, "left_pin"), (1.0, "right_pin")):
            slat.visual(
                Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
                origin=Origin(
                    xyz=(side_sign * (OPENING_WIDTH * 0.5 + PIN_LENGTH * 0.5), 0.0, 0.0),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material=pin_metal,
                name=pin_name,
            )
        slat.inertial = Inertial.from_geometry(
            Box((OPENING_WIDTH + 2.0 * PIN_LENGTH, SLAT_THICKNESS, SLAT_CHORD)),
            mass=0.085,
            origin=Origin(),
        )
        model.articulation(
            f"frame_to_slat_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, _slat_center_z(index))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.5,
                lower=-SLAT_LIMIT,
                upper=SLAT_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    frame = object_model.get_part("frame")
    slats = [object_model.get_part(name) for name in _slat_names()]
    joints = [object_model.get_articulation(name) for name in _slat_joint_names()]

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "frame_has_all_slats",
        len(slats) == SLAT_COUNT,
        details=f"Expected {SLAT_COUNT} slats but found {len(slats)}.",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame_proportions_realistic",
            abs(frame_width - PANEL_WIDTH) < 0.002 and abs(frame_height - PANEL_HEIGHT) < 0.002,
            details=(
                f"Frame dimensions were {frame_width:.4f} m by {frame_height:.4f} m "
                f"instead of about {PANEL_WIDTH:.3f} m by {PANEL_HEIGHT:.3f} m."
            ),
        )

    slat_positions = [ctx.part_world_position(slat) for slat in slats]
    if all(position is not None for position in slat_positions):
        z_positions = [position[2] for position in slat_positions if position is not None]
        pitch_ok = all(
            abs((z_positions[index + 1] - z_positions[index]) - SLAT_PITCH) < 1e-4
            for index in range(len(z_positions) - 1)
        )
        ctx.check(
            "slat_pitch_uniform",
            pitch_ok,
            details="Slat centers are not evenly spaced along the panel height.",
        )

    for slat in slats:
        ctx.expect_contact(
            slat,
            frame,
            name=f"{slat.name}_mounted_to_frame",
        )
        ctx.expect_within(
            slat,
            frame,
            axes="yz",
            margin=0.0,
            name=f"{slat.name}_contained_in_panel_thickness_and_height",
        )

    for joint, slat in zip(joints, slats):
        limits = joint.motion_limits
        axis = tuple(round(value, 6) for value in joint.axis)
        ctx.check(
            f"{joint.name}_axis_is_longitudinal",
            axis == (1.0, 0.0, 0.0),
            details=f"Expected longitudinal x-axis revolute motion, got {joint.axis}.",
        )
        ctx.check(
            f"{joint.name}_has_realistic_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + SLAT_LIMIT) < 1e-6
            and abs(limits.upper - SLAT_LIMIT) < 1e-6,
            details="Slat joint limits should allow about +/-55 degrees of rotation.",
        )
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.expect_contact(
                    slat,
                    frame,
                    name=f"{joint.name}_lower_pose_still_supported",
                )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_lower_pose_no_overlap"
                )
            with ctx.pose({joint: limits.upper}):
                ctx.expect_contact(
                    slat,
                    frame,
                    name=f"{joint.name}_upper_pose_still_supported",
                )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_upper_pose_no_overlap"
                )

    ctx.fail_if_isolated_parts(
        max_pose_samples=24,
        name="sampled_pose_support_connectivity",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
