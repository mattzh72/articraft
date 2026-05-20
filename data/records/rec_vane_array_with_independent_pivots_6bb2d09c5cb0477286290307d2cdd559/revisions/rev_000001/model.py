from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_OUTER_WIDTH = 0.66
FRAME_OUTER_HEIGHT = 0.42
FRAME_DEPTH = 0.023
SIDE_RAIL_THICKNESS = 0.040
TOP_RAIL_THICKNESS = 0.040
OPENING_WIDTH = FRAME_OUTER_WIDTH - 2.0 * SIDE_RAIL_THICKNESS
OPENING_HEIGHT = FRAME_OUTER_HEIGHT - 2.0 * TOP_RAIL_THICKNESS

BLADE_COUNT = 5
BLADE_PITCH = 0.060
BLADE_CHORD = 0.036
BLADE_MAX_ANGLE = 0.18
BLADE_THICKNESS = 0.006
BLADE_BODY_OFFSET_Y = 0.026
BLADE_BODY_SPAN = OPENING_WIDTH - 0.044
PIVOT_BARREL_LENGTH = 0.010

REAR_BRIDGE_WIDTH = 0.022
REAR_BRIDGE_DEPTH = 0.009
REAR_FRAME_DEPTH = 0.014
REAR_BRIDGE_Y = -0.0095
REAR_BRIDGE_X_POSITIONS = (-0.17, 0.0, 0.17)
PIVOT_AXIS_Y = 0.0
PIVOT_PIN_RADIUS = 0.0050
PIVOT_PIN_LENGTH = 0.0


def _blade_z_positions() -> list[float]:
    start = -0.5 * BLADE_PITCH * (BLADE_COUNT - 1)
    return [start + index * BLADE_PITCH for index in range(BLADE_COUNT)]


def _make_frame_shape() -> cq.Workplane:
    left_rail = (
        cq.Workplane("XY")
        .box(SIDE_RAIL_THICKNESS, REAR_FRAME_DEPTH, FRAME_OUTER_HEIGHT)
        .translate(
            (
                -FRAME_OUTER_WIDTH / 2.0 + SIDE_RAIL_THICKNESS / 2.0,
                -REAR_FRAME_DEPTH / 2.0,
                0.0,
            )
        )
    )
    right_rail = (
        cq.Workplane("XY")
        .box(SIDE_RAIL_THICKNESS, REAR_FRAME_DEPTH, FRAME_OUTER_HEIGHT)
        .translate(
            (
                FRAME_OUTER_WIDTH / 2.0 - SIDE_RAIL_THICKNESS / 2.0,
                -REAR_FRAME_DEPTH / 2.0,
                0.0,
            )
        )
    )
    top_rail = (
        cq.Workplane("XY")
        .box(OPENING_WIDTH, REAR_FRAME_DEPTH, TOP_RAIL_THICKNESS)
        .translate(
            (
                0.0,
                -REAR_FRAME_DEPTH / 2.0,
                FRAME_OUTER_HEIGHT / 2.0 - TOP_RAIL_THICKNESS / 2.0,
            )
        )
    )
    bottom_rail = (
        cq.Workplane("XY")
        .box(OPENING_WIDTH, REAR_FRAME_DEPTH, TOP_RAIL_THICKNESS)
        .translate(
            (
                0.0,
                -REAR_FRAME_DEPTH / 2.0,
                -FRAME_OUTER_HEIGHT / 2.0 + TOP_RAIL_THICKNESS / 2.0,
            )
        )
    )
    frame = left_rail.union(right_rail).union(top_rail).union(bottom_rail)

    for bridge_x in REAR_BRIDGE_X_POSITIONS:
        bridge = (
            cq.Workplane("XY")
            .box(REAR_BRIDGE_WIDTH, REAR_BRIDGE_DEPTH, OPENING_HEIGHT)
            .translate((bridge_x, REAR_BRIDGE_Y, 0.0))
        )
        frame = frame.union(bridge)

    return frame


def _make_blade_shape() -> cq.Workplane:
    opening_half = OPENING_WIDTH / 2.0
    body_half = BLADE_BODY_SPAN / 2.0
    arm_length = opening_half - PIVOT_BARREL_LENGTH - body_half

    blade_body = (
        cq.Workplane("YZ")
        .center(BLADE_BODY_OFFSET_Y, 0.0)
        .ellipse(BLADE_CHORD / 2.0, BLADE_THICKNESS / 2.0)
        .extrude(body_half, both=True)
    )

    left_arm = (
        cq.Workplane("XY")
        .box(arm_length, BLADE_BODY_OFFSET_Y, BLADE_THICKNESS * 0.8)
        .translate(
            (
                -body_half - arm_length / 2.0,
                BLADE_BODY_OFFSET_Y / 2.0,
                0.0,
            )
        )
    )
    right_arm = (
        cq.Workplane("XY")
        .box(arm_length, BLADE_BODY_OFFSET_Y, BLADE_THICKNESS * 0.8)
        .translate(
            (
                body_half + arm_length / 2.0,
                BLADE_BODY_OFFSET_Y / 2.0,
                0.0,
            )
        )
    )
    left_barrel = (
        cq.Workplane("YZ")
        .circle(PIVOT_PIN_RADIUS)
        .extrude(PIVOT_BARREL_LENGTH)
        .translate((-opening_half, 0.0, 0.0))
    )
    right_barrel = (
        cq.Workplane("YZ")
        .circle(PIVOT_PIN_RADIUS)
        .extrude(PIVOT_BARREL_LENGTH)
        .translate((opening_half - PIVOT_BARREL_LENGTH, 0.0, 0.0))
    )

    return blade_body.union(left_arm).union(right_arm).union(left_barrel).union(right_barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_vane_array")

    model.material("frame_finish", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("blade_finish", rgba=(0.76, 0.79, 0.82, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_shape(), "rear_frame"),
        material="frame_finish",
        name="frame_shell",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=3.8,
        origin=Origin(),
    )

    blade_mesh = mesh_from_cadquery(_make_blade_shape(), "vane_blade")
    blade_box = Box(
        (
            OPENING_WIDTH,
            BLADE_CHORD,
            BLADE_THICKNESS,
        )
    )

    for index, z_pos in enumerate(_blade_z_positions(), start=1):
        blade = model.part(f"blade_{index}")
        blade.visual(blade_mesh, material="blade_finish", name="blade_shell")
        blade.inertial = Inertial.from_geometry(
            blade_box,
            mass=0.18,
            origin=Origin(),
        )
        model.articulation(
            f"frame_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(0.0, PIVOT_AXIS_Y, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.5,
                lower=-BLADE_MAX_ANGLE,
                upper=BLADE_MAX_ANGLE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    blades = [object_model.get_part(f"blade_{index}") for index in range(1, BLADE_COUNT + 1)]
    joints = [
        object_model.get_articulation(f"frame_to_blade_{index}")
        for index in range(1, BLADE_COUNT + 1)
    ]

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

    joint_axes_ok = all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in joints)
    joint_y_ok = all(abs(joint.origin.xyz[1] - PIVOT_AXIS_Y) < 1e-9 for joint in joints)
    z_positions = [joint.origin.xyz[2] for joint in joints]
    z_spacings = [z_positions[index + 1] - z_positions[index] for index in range(len(z_positions) - 1)]
    spacing_ok = all(abs(spacing - BLADE_PITCH) < 1e-9 for spacing in z_spacings)
    ctx.check(
        "parallel_blade_axes_and_spacing",
        joint_axes_ok and joint_y_ok and spacing_ok,
        details=(
            f"axes={[joint.axis for joint in joints]}, "
            f"y_offsets={[joint.origin.xyz[1] for joint in joints]}, "
            f"z_spacings={z_spacings}"
        ),
    )

    for blade in blades:
        ctx.expect_within(
            blade,
            frame,
            axes="xz",
            margin=0.0,
            name=f"{blade.name}_within_frame_footprint",
        )

    for lower, upper in zip(blades[:-1], blades[1:]):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.040,
            name=f"{lower.name}_to_{upper.name}_stack_gap",
        )

    for blade in blades:
        ctx.expect_contact(
            blade,
            frame,
            name=f"{blade.name}_supported_by_frame_pivots",
        )

    def _aabb_size(aabb):
        return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))

    blade_1_closed = ctx.part_element_world_aabb(blades[0], elem="blade_shell")
    blade_2_closed = ctx.part_element_world_aabb(blades[1], elem="blade_shell")
    with ctx.pose(frame_to_blade_1=0.12):
        blade_1_open = ctx.part_element_world_aabb(blades[0], elem="blade_shell")
        blade_2_still = ctx.part_element_world_aabb(blades[1], elem="blade_shell")

    blade_1_closed_size = _aabb_size(blade_1_closed)
    blade_1_open_size = _aabb_size(blade_1_open)
    blade_2_closed_size = _aabb_size(blade_2_closed)
    blade_2_still_size = _aabb_size(blade_2_still)
    blade_1_rotated = (
        abs(blade_1_open_size[1] - blade_1_closed_size[1]) > 0.003
        or abs(blade_1_open_size[2] - blade_1_closed_size[2]) > 0.003
    )
    blade_2_unchanged = all(
        abs(blade_2_still_size[axis] - blade_2_closed_size[axis]) < 1e-9 for axis in range(3)
    )
    ctx.check(
        "independent_blade_motion",
        blade_1_rotated and blade_2_unchanged,
        details=(
            f"blade_1_closed={blade_1_closed_size}, blade_1_open={blade_1_open_size}, "
            f"blade_2_closed={blade_2_closed_size}, blade_2_still={blade_2_still_size}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
