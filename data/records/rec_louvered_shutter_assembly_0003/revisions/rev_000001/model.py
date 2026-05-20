from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_WIDTH = 0.64
FRAME_HEIGHT = 1.16
FRAME_THICKNESS = 0.034
STILE_WIDTH = 0.06
RAIL_HEIGHT = 0.06

OPENING_WIDTH = FRAME_WIDTH - (2.0 * STILE_WIDTH)
OPENING_HEIGHT = FRAME_HEIGHT - (2.0 * RAIL_HEIGHT)

SLAT_COUNT = 12
SLAT_PITCH = 0.085
SLAT_CHORD = 0.072
SLAT_THICKNESS = 0.008
SLAT_BODY_LENGTH = OPENING_WIDTH - 0.016

PIN_RADIUS = 0.0032
PIN_CLEARANCE_RADIUS = 0.0038
CAP_RADIUS = 0.0052
CAP_THICKNESS = 0.004
PIN_INSERT = 0.001
PIN_SHAFT_LENGTH = (FRAME_WIDTH * 0.5 + CAP_THICKNESS) - (SLAT_BODY_LENGTH * 0.5) + PIN_INSERT
PIN_CENTER_X = (SLAT_BODY_LENGTH * 0.5) + (PIN_SHAFT_LENGTH * 0.5) - (PIN_INSERT * 0.5)
CAP_CENTER_X = (FRAME_WIDTH * 0.5) + (CAP_THICKNESS * 0.5)

SLAT_Z_POSITIONS = tuple(
    (index - ((SLAT_COUNT - 1) * 0.5)) * SLAT_PITCH for index in range(SLAT_COUNT)
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_stile_mesh():
    stile = BoxGeometry((STILE_WIDTH, FRAME_THICKNESS, FRAME_HEIGHT))
    for z_pos in SLAT_Z_POSITIONS:
        hole = CylinderGeometry(
            radius=PIN_CLEARANCE_RADIUS,
            height=STILE_WIDTH + 0.010,
            radial_segments=28,
        )
        hole.rotate_y(pi / 2.0).translate(0.0, 0.0, z_pos)
        stile = boolean_difference(stile, hole)
    return stile


def _build_slat_mesh():
    profile = sample_catmull_rom_spline_2d(
        [
            (-SLAT_CHORD * 0.50, 0.0),
            (-SLAT_CHORD * 0.25, SLAT_THICKNESS * 0.50),
            (SLAT_CHORD * 0.16, SLAT_THICKNESS * 0.42),
            (SLAT_CHORD * 0.50, 0.0),
            (SLAT_CHORD * 0.16, -SLAT_THICKNESS * 0.42),
            (-SLAT_CHORD * 0.25, -SLAT_THICKNESS * 0.50),
        ],
        samples_per_segment=10,
        closed=True,
    )[:-1]
    return ExtrudeGeometry.centered(
        profile,
        height=SLAT_BODY_LENGTH,
        cap=True,
        closed=True,
    ).rotate_y(pi / 2.0)


def _span(aabb, axis: str) -> float | None:
    if aabb is None:
        return None
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return float(aabb[1][axis_index] - aabb[0][axis_index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_shutter_panel", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.94, 0.93, 0.89, 1.0))
    slat_paint = model.material("slat_paint", rgba=(0.91, 0.90, 0.86, 1.0))
    hardware = model.material("hardware", rgba=(0.42, 0.43, 0.46, 1.0))

    stile_mesh = _save_mesh("shutter_stile.obj", _build_stile_mesh())
    slat_mesh = _save_mesh("shutter_slat.obj", _build_slat_mesh())

    frame = model.part("frame")
    frame.visual(
        stile_mesh,
        origin=Origin(xyz=(-(FRAME_WIDTH * 0.5) + (STILE_WIDTH * 0.5), 0.0, 0.0)),
        material=frame_paint,
        name="left_stile",
    )
    frame.visual(
        stile_mesh,
        origin=Origin(xyz=((FRAME_WIDTH * 0.5) - (STILE_WIDTH * 0.5), 0.0, 0.0)),
        material=frame_paint,
        name="right_stile",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, (FRAME_HEIGHT * 0.5) - (RAIL_HEIGHT * 0.5))),
        material=frame_paint,
        name="top_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_THICKNESS, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -(FRAME_HEIGHT * 0.5) + (RAIL_HEIGHT * 0.5))),
        material=frame_paint,
        name="bottom_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_THICKNESS, FRAME_HEIGHT)),
        mass=6.0,
        origin=Origin(),
    )

    pin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    right_pin_origin = Origin(xyz=(PIN_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0))
    left_pin_origin = Origin(xyz=(-PIN_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0))
    right_cap_origin = Origin(xyz=(CAP_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0))
    left_cap_origin = Origin(xyz=(-CAP_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0))

    for index, z_pos in enumerate(SLAT_Z_POSITIONS, start=1):
        slat = model.part(f"slat_{index:02d}")
        slat.visual(slat_mesh, material=slat_paint, name="slat_blade")
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_SHAFT_LENGTH),
            origin=left_pin_origin,
            material=hardware,
            name="left_pin",
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_SHAFT_LENGTH),
            origin=right_pin_origin,
            material=hardware,
            name="right_pin",
        )
        slat.visual(
            Cylinder(radius=CAP_RADIUS, length=CAP_THICKNESS),
            origin=left_cap_origin,
            material=hardware,
            name="left_cap",
        )
        slat.visual(
            Cylinder(radius=CAP_RADIUS, length=CAP_THICKNESS),
            origin=right_cap_origin,
            material=hardware,
            name="right_cap",
        )
        slat.inertial = Inertial.from_geometry(
            Box((FRAME_WIDTH + (2.0 * CAP_THICKNESS), SLAT_THICKNESS, SLAT_CHORD)),
            mass=0.18,
            origin=Origin(),
        )
        model.articulation(
            f"slat_{index:02d}_tilt",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.5,
                lower=0.0,
                upper=pi / 2.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_stile = frame.get_visual("left_stile")
    right_stile = frame.get_visual("right_stile")
    slats = [object_model.get_part(f"slat_{index:02d}") for index in range(1, SLAT_COUNT + 1)]
    joints = [object_model.get_articulation(f"slat_{index:02d}_tilt") for index in range(1, SLAT_COUNT + 1)]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "slat_count",
        len(slats) == SLAT_COUNT and len(joints) == SLAT_COUNT,
        f"expected {SLAT_COUNT} slats and joints, got {len(slats)} slats and {len(joints)} joints",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    frame_width = _span(frame_aabb, "x")
    frame_height = _span(frame_aabb, "z")
    frame_thickness = _span(frame_aabb, "y")
    ctx.check(
        "frame_dimensions",
        frame_width is not None
        and frame_height is not None
        and frame_thickness is not None
        and abs(frame_width - FRAME_WIDTH) < 0.003
        and abs(frame_height - FRAME_HEIGHT) < 0.003
        and abs(frame_thickness - FRAME_THICKNESS) < 0.003,
        (
            f"frame dims were width={frame_width}, height={frame_height}, "
            f"thickness={frame_thickness}"
        ),
    )

    world_positions: list[tuple[float, float, float]] = []
    for index, (slat, joint, z_expected) in enumerate(zip(slats, joints, SLAT_Z_POSITIONS), start=1):
        left_cap = slat.get_visual("left_cap")
        right_cap = slat.get_visual("right_cap")
        position = ctx.part_world_position(slat)
        if position is not None:
            world_positions.append(position)

        ctx.check(
            f"{slat.name}_positioned_on_axis",
            position is not None
            and abs(position[0]) < 1e-6
            and abs(position[1]) < 1e-6
            and abs(position[2] - z_expected) < 1e-6,
            f"expected world origin at (0, 0, {z_expected}), got {position}",
        )
        ctx.check(
            f"{joint.name}_axis_and_limits",
            joint.axis == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower - 0.0) < 1e-9
            and abs(joint.motion_limits.upper - (pi / 2.0)) < 1e-9,
            f"joint axis/limits were axis={joint.axis}, limits={joint.motion_limits}",
        )
        ctx.expect_gap(
            frame,
            slat,
            axis="x",
            positive_elem=left_stile,
            negative_elem=left_cap,
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"slat_{index:02d}_left_cap_seated_rest",
        )
        ctx.expect_gap(
            slat,
            frame,
            axis="x",
            positive_elem=right_cap,
            negative_elem=right_stile,
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"slat_{index:02d}_right_cap_seated_rest",
        )
        with ctx.pose({joint: joint.motion_limits.upper * 0.95}):
            ctx.expect_gap(
                frame,
                slat,
                axis="x",
                positive_elem=left_stile,
                negative_elem=left_cap,
                max_gap=0.001,
                max_penetration=0.0005,
                name=f"slat_{index:02d}_left_cap_seated_open",
            )
            ctx.expect_gap(
                slat,
                frame,
                axis="x",
                positive_elem=right_cap,
                negative_elem=right_stile,
                max_gap=0.001,
                max_penetration=0.0005,
                name=f"slat_{index:02d}_right_cap_seated_open",
            )

    pitches = [world_positions[i + 1][2] - world_positions[i][2] for i in range(len(world_positions) - 1)]
    ctx.check(
        "slat_pitch_uniform",
        len(world_positions) == SLAT_COUNT and all(abs(pitch - SLAT_PITCH) < 1e-6 for pitch in pitches),
        f"slat center pitches were {pitches}",
    )

    mid_index = SLAT_COUNT // 2
    mid_slat = slats[mid_index]
    mid_joint = joints[mid_index]
    mid_blade = mid_slat.get_visual("slat_blade")
    rest_blade_aabb = ctx.part_element_world_aabb(mid_slat, elem=mid_blade)
    rest_y_span = _span(rest_blade_aabb, "y")
    rest_z_span = _span(rest_blade_aabb, "z")
    ctx.check(
        "mid_slat_rest_orientation",
        rest_y_span is not None
        and rest_z_span is not None
        and rest_y_span < 0.012
        and rest_z_span > 0.068,
        f"rest spans were y={rest_y_span}, z={rest_z_span}",
    )
    with ctx.pose({mid_joint: mid_joint.motion_limits.upper * 0.95}):
        open_blade_aabb = ctx.part_element_world_aabb(mid_slat, elem=mid_blade)
        open_y_span = _span(open_blade_aabb, "y")
        open_z_span = _span(open_blade_aabb, "z")
        ctx.check(
            "mid_slat_open_orientation",
            open_y_span is not None
            and open_z_span is not None
            and open_y_span > 0.060
            and open_z_span < 0.020,
            f"open spans were y={open_y_span}, z={open_z_span}",
        )

    fully_open_pose = {joint: joint.motion_limits.upper * 0.95 for joint in joints}
    with ctx.pose(fully_open_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_slats_open_clear")

    mixed_pose = {
        joint: (joint.motion_limits.upper * 0.20 if index % 2 == 0 else joint.motion_limits.upper * 0.85)
        for index, joint in enumerate(joints)
    }
    with ctx.pose(mixed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="mixed_slats_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
