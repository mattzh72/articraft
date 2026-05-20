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


BASE_WIDTH = 0.19
BASE_DEPTH = 0.14
BASE_THICKNESS = 0.018

BACK_FRAME_WIDTH = 0.21
BACK_FRAME_DEPTH = 0.02
BACK_FRAME_HEIGHT = 0.314

RING_CENTER_Y = 0.018
RING_CENTER_Z = 0.255

OUTER_RING_WIDTH = 0.185
OUTER_RING_HEIGHT = 0.185
OUTER_RING_THICKNESS = 0.022
OUTER_RING_INNER_WIDTH = 0.118
OUTER_RING_INNER_HEIGHT = 0.118
OUTER_RING_CORNER = 0.028
OUTER_RING_INNER_CORNER = 0.016

TRUNNION_RADIUS = 0.026
TRUNNION_LENGTH = 0.030
TRUNNION_FUSE = 0.002

SPIDER_THICKNESS = 0.004
SPIDER_DISC_RADIUS = 0.018
SPIDER_BAR_WIDTH = 0.012

CRADLE_WIDTH = 0.092
CRADLE_HEIGHT = 0.116
CRADLE_THICKNESS = 0.017
CRADLE_WALL = 0.018
CRADLE_WINDOW_HEIGHT = 0.062
CRADLE_WINDOW_Z = 0.010
CRADLE_PIVOT_RADIUS = 0.014
CRADLE_PIVOT_THICKNESS = 0.002

OUTER_LOWER = -0.75
OUTER_UPPER = 0.75
INNER_LOWER = -0.85
INNER_UPPER = 0.85


def _box(length: float, depth: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, depth, height).translate(center)


def _x_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _y_plate_from_xz(profile: cq.Workplane, center_y: float, thickness: float) -> cq.Workplane:
    return profile.extrude(thickness / 2.0, both=True).translate((0.0, center_y, 0.0))


def _make_support_frame_shape() -> cq.Workplane:
    base = _box(BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS, (0.0, 0.0, BASE_THICKNESS / 2.0))
    rear_column = _box(
        0.070,
        0.024,
        BACK_FRAME_HEIGHT,
        (0.0, -0.048, BASE_THICKNESS + BACK_FRAME_HEIGHT / 2.0),
    )
    upper_bridge = _box(0.218, 0.024, 0.028, (0.0, -0.038, 0.312))
    lower_bridge = _box(0.146, 0.022, 0.022, (0.0, -0.036, 0.206))

    side_arm_width = 0.034
    side_arm_depth = 0.060
    side_arm_height = 0.094
    arm_center_x = OUTER_RING_WIDTH / 2.0 + side_arm_width / 2.0 + 0.001
    left_arm = _box(
        side_arm_width,
        side_arm_depth,
        side_arm_height,
        (-arm_center_x, -0.018, RING_CENTER_Z),
    )
    right_arm = _box(
        side_arm_width,
        side_arm_depth,
        side_arm_height,
        (arm_center_x, -0.018, RING_CENTER_Z),
    )
    arm_hole_length = side_arm_width + 0.006
    left_arm = left_arm.cut(
        _x_cylinder(TRUNNION_RADIUS, arm_hole_length, (-arm_center_x, RING_CENTER_Y, RING_CENTER_Z))
    )
    right_arm = right_arm.cut(
        _x_cylinder(TRUNNION_RADIUS, arm_hole_length, (arm_center_x, RING_CENTER_Y, RING_CENTER_Z))
    )

    foot_rib = _box(0.092, 0.052, 0.054, (0.0, -0.035, 0.045))

    return (
        base.union(rear_column)
        .union(upper_bridge)
        .union(lower_bridge)
        .union(left_arm)
        .union(right_arm)
        .union(foot_rib)
    )


def _make_outer_ring_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(OUTER_RING_WIDTH, OUTER_RING_THICKNESS, OUTER_RING_HEIGHT)
        .edges("|Y")
        .fillet(OUTER_RING_CORNER)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            OUTER_RING_INNER_WIDTH,
            OUTER_RING_THICKNESS + 0.004,
            OUTER_RING_INNER_HEIGHT,
        )
        .edges("|Y")
        .fillet(OUTER_RING_INNER_CORNER)
    )
    ring = outer.cut(inner)

    trunnion_center_x = OUTER_RING_WIDTH / 2.0 + TRUNNION_LENGTH / 2.0 - TRUNNION_FUSE
    left_trunnion = _x_cylinder(
        TRUNNION_RADIUS,
        TRUNNION_LENGTH,
        (-trunnion_center_x, 0.0, 0.0),
    )
    right_trunnion = _x_cylinder(
        TRUNNION_RADIUS,
        TRUNNION_LENGTH,
        (trunnion_center_x, 0.0, 0.0),
    )

    support_pad_width = 0.042
    support_pad_height = 0.034
    support_pad_thickness = 0.008
    support_rib_height = 0.056
    support_rib_width = 0.012
    spider_offset = OUTER_RING_THICKNESS / 2.0 + support_pad_thickness / 2.0 - 0.0005

    front_support = (
        _box(support_pad_width, support_pad_thickness, support_pad_height, (0.0, spider_offset, 0.0))
        .union(_box(support_rib_width, support_pad_thickness, support_rib_height, (0.0, spider_offset, 0.036)))
        .union(_box(support_rib_width, support_pad_thickness, support_rib_height, (0.0, spider_offset, -0.036)))
    )
    rear_support = (
        _box(support_pad_width, support_pad_thickness, support_pad_height, (0.0, -spider_offset, 0.0))
        .union(_box(support_rib_width, support_pad_thickness, support_rib_height, (0.0, -spider_offset, 0.036)))
        .union(_box(support_rib_width, support_pad_thickness, support_rib_height, (0.0, -spider_offset, -0.036)))
    )
    front_support = front_support.cut(
        _y_plate_from_xz(cq.Workplane("XZ").circle(CRADLE_PIVOT_RADIUS), spider_offset, support_pad_thickness + 0.004)
    )
    rear_support = rear_support.cut(
        _y_plate_from_xz(cq.Workplane("XZ").circle(CRADLE_PIVOT_RADIUS), -spider_offset, support_pad_thickness + 0.004)
    )

    return ring.union(left_trunnion).union(right_trunnion).union(front_support).union(rear_support)


def _make_inner_cradle_shape() -> cq.Workplane:
    left_cheek = _box(0.018, CRADLE_THICKNESS, 0.098, (-0.032, 0.0, -0.006))
    right_cheek = _box(0.018, CRADLE_THICKNESS, 0.098, (0.032, 0.0, -0.006))
    tray = _box(0.064, CRADLE_THICKNESS, 0.018, (0.0, 0.0, -0.042))
    hub_block = _box(0.050, CRADLE_THICKNESS, 0.028, (0.0, 0.0, 0.0))
    spindle = _y_plate_from_xz(
        cq.Workplane("XZ").circle(CRADLE_PIVOT_RADIUS),
        0.0,
        OUTER_RING_THICKNESS + 0.016,
    )

    return left_cheek.union(right_cheek).union(tray).union(hub_block).union(spindle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_pitch_roll_module")

    model.material("frame_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("ring_metal", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("cradle_dark", rgba=(0.28, 0.30, 0.34, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_make_support_frame_shape(), "support_frame"),
        material="frame_dark",
        name="support_frame_shell",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.244, BASE_DEPTH, 0.336)),
        mass=5.8,
        origin=Origin(xyz=(0.0, -0.01, 0.168)),
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(_make_outer_ring_shape(), "outer_ring"),
        material="ring_metal",
        name="outer_ring_shell",
    )
    outer_ring.inertial = Inertial.from_geometry(
        Box((0.215, 0.030, OUTER_RING_HEIGHT)),
        mass=1.6,
        origin=Origin(),
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(_make_inner_cradle_shape(), "inner_cradle"),
        material="cradle_dark",
        name="inner_cradle_shell",
    )
    inner_cradle.inertial = Inertial.from_geometry(
        Box((CRADLE_WIDTH, 0.022, CRADLE_HEIGHT)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=outer_ring,
        origin=Origin(xyz=(0.0, RING_CENTER_Y, RING_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.6,
            lower=OUTER_LOWER,
            upper=OUTER_UPPER,
        ),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=INNER_LOWER,
            upper=INNER_UPPER,
        ),
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

    support_frame = object_model.get_part("support_frame")
    outer_ring = object_model.get_part("outer_ring")
    inner_cradle = object_model.get_part("inner_cradle")
    outer_joint = object_model.get_articulation("support_to_outer")
    inner_joint = object_model.get_articulation("outer_to_inner")

    def span(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "all prompt-critical parts exist",
        all(part is not None for part in (support_frame, outer_ring, inner_cradle)),
    )

    axis_dot = sum(a * b for a, b in zip(outer_joint.axis, inner_joint.axis))
    ctx.check(
        "gimbal axes are perpendicular",
        abs(axis_dot) < 1e-6,
        details=f"outer_axis={outer_joint.axis}, inner_axis={inner_joint.axis}, dot={axis_dot}",
    )

    ctx.allow_overlap(
        outer_ring,
        support_frame,
        elem_a="outer_ring_shell",
        elem_b="support_frame_shell",
        reason="The outer ring trunnions are modeled as a nominal fitted bearing passing through the support-frame cheek bores.",
    )
    ctx.allow_overlap(
        inner_cradle,
        outer_ring,
        elem_a="inner_cradle_shell",
        elem_b="outer_ring_shell",
        reason="The inner cradle spindle is modeled as a nominal fitted bearing passing through the outer ring's pitch-axis support bores.",
    )

    ctx.expect_contact(
        outer_ring,
        support_frame,
        contact_tol=0.002,
        name="outer ring stays carried by the rear support frame",
    )
    ctx.expect_contact(
        inner_cradle,
        outer_ring,
        contact_tol=0.002,
        name="inner cradle stays supported inside the outer ring",
    )
    ctx.expect_within(
        inner_cradle,
        outer_ring,
        axes="xz",
        margin=0.0,
        name="inner cradle remains nested inside the outer member at rest",
    )

    rest_outer_aabb = ctx.part_world_aabb(outer_ring)
    with ctx.pose({outer_joint: OUTER_UPPER * 0.8}):
        posed_outer_aabb = ctx.part_world_aabb(outer_ring)
        ctx.expect_contact(
            outer_ring,
            support_frame,
            contact_tol=0.002,
            name="outer ring remains supported through roll travel",
        )

    rest_outer_y = span(rest_outer_aabb, "y")
    posed_outer_y = span(posed_outer_aabb, "y")
    ctx.check(
        "outer joint rotates the ring about its supported axis",
        rest_outer_y is not None
        and posed_outer_y is not None
        and posed_outer_y > rest_outer_y + 0.030,
        details=f"rest_y_span={rest_outer_y}, posed_y_span={posed_outer_y}",
    )

    rest_inner_aabb = ctx.part_world_aabb(inner_cradle)
    with ctx.pose({inner_joint: INNER_UPPER * 0.8}):
        posed_inner_aabb = ctx.part_world_aabb(inner_cradle)
        ctx.expect_within(
            inner_cradle,
            outer_ring,
            axes="xz",
            margin=0.002,
            name="inner cradle stays visually nested at pitch limit",
        )
        ctx.expect_contact(
            inner_cradle,
            outer_ring,
            contact_tol=0.002,
            name="inner cradle remains supported through pitch travel",
        )

    rest_inner_x = span(rest_inner_aabb, "x")
    posed_inner_x = span(posed_inner_aabb, "x")
    ctx.check(
        "inner cradle rotates on its perpendicular axis",
        rest_inner_x is not None
        and posed_inner_x is not None
        and posed_inner_x > rest_inner_x + 0.015,
        details=f"rest_x_span={rest_inner_x}, posed_x_span={posed_inner_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
