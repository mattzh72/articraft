from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.42
BASE_D = 0.24
BASE_H = 0.055

SPINE_W = 0.14
SPINE_D = 0.09
SPINE_H = 1.16

UPPER_AXIS = (0.0, 0.105, 1.00)
LOWER_AXIS = (0.0, -0.132, 0.52)


def _filleted_box(
    size: tuple[float, float, float],
    *,
    radius: float = 0.0,
    centered: tuple[bool, bool, bool] = (True, True, True),
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size, centered=centered)
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    if translate != (0.0, 0.0, 0.0):
        solid = solid.translate(translate)
    return solid


def make_spine_frame() -> cq.Workplane:
    base = _filleted_box(
        (BASE_W, BASE_D, BASE_H),
        radius=0.012,
        centered=(True, True, False),
    )
    spine = _filleted_box(
        (SPINE_W, SPINE_D, SPINE_H),
        radius=0.008,
        centered=(True, True, False),
        translate=(0.0, 0.0, BASE_H),
    )

    upper_backstrap = _filleted_box(
        (0.092, 0.026, 0.14),
        radius=0.004,
        translate=(0.0, 0.058, UPPER_AXIS[2]),
    )
    left_cheek = _filleted_box(
        (0.022, 0.092, 0.18),
        radius=0.004,
        translate=(0.039, 0.092, UPPER_AXIS[2]),
    )
    right_cheek = _filleted_box(
        (0.022, 0.092, 0.18),
        radius=0.004,
        translate=(-0.039, 0.092, UPPER_AXIS[2]),
    )
    left_boss = (
        cq.Workplane("YZ")
        .circle(0.032)
        .extrude(0.004, both=True)
        .translate((0.050, UPPER_AXIS[1], UPPER_AXIS[2]))
    )
    right_boss = (
        cq.Workplane("YZ")
        .circle(0.032)
        .extrude(0.004, both=True)
        .translate((-0.050, UPPER_AXIS[1], UPPER_AXIS[2]))
    )

    pedestal = _filleted_box(
        (0.18, 0.12, 0.16),
        radius=0.006,
        translate=(0.0, LOWER_AXIS[1], 0.37),
    )
    pedestal_cap = _filleted_box(
        (0.14, 0.10, 0.03),
        radius=0.004,
        translate=(0.0, LOWER_AXIS[1], 0.465),
    )
    stub_flange = (
        cq.Workplane("XY")
        .circle(0.065)
        .extrude(0.008)
        .translate((LOWER_AXIS[0], LOWER_AXIS[1], 0.472))
    )
    thrust_washer = (
        cq.Workplane("XY")
        .circle(0.034)
        .extrude(0.006)
        .translate((LOWER_AXIS[0], LOWER_AXIS[1], 0.514))
    )
    stub_post = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.048)
        .translate((LOWER_AXIS[0], LOWER_AXIS[1], 0.472))
    )

    return (
        base.union(spine)
        .union(upper_backstrap)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_boss)
        .union(right_boss)
        .union(pedestal)
        .union(pedestal_cap)
        .union(stub_flange)
        .union(thrust_washer)
        .union(stub_post)
    )


def make_upper_hub() -> cq.Workplane:
    return cq.Workplane("YZ").circle(0.020).extrude(0.028, both=True)


def make_upper_fork() -> cq.Workplane:
    bridge = _filleted_box(
        (0.046, 0.15, 0.022),
        radius=0.004,
        translate=(0.0, 0.085, 0.0),
    )
    shoulder = _filleted_box(
        (0.080, 0.05, 0.022),
        radius=0.005,
        translate=(0.0, 0.180, 0.0),
    )
    fork_body = _filleted_box(
        (0.120, 0.15, 0.022),
        radius=0.006,
        translate=(0.0, 0.255, 0.0),
    )
    fork_slot = _filleted_box(
        (0.050, 0.13, 0.05),
        translate=(0.0, 0.258, 0.0),
    )
    tine_left_pad = _filleted_box(
        (0.030, 0.050, 0.028),
        radius=0.004,
        translate=(0.037, 0.330, 0.0),
    )
    tine_right_pad = _filleted_box(
        (0.030, 0.050, 0.028),
        radius=0.004,
        translate=(-0.037, 0.330, 0.0),
    )

    return (
        bridge.union(shoulder)
        .union(fork_body.cut(fork_slot))
        .union(tine_left_pad)
        .union(tine_right_pad)
    )


def make_lower_hub() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.040).circle(0.022).extrude(0.028)
    cap = cq.Workplane("XY").circle(0.054).extrude(0.006).translate((0.0, 0.0, 0.028))
    return hub.union(cap)


def make_lower_plate() -> cq.Workplane:
    neck = _filleted_box(
        (0.10, 0.14, 0.018),
        radius=0.006,
        centered=(True, True, False),
        translate=(0.0, -0.08, 0.022),
    )
    plate = _filleted_box(
        (0.24, 0.18, 0.016),
        radius=0.018,
        centered=(True, True, False),
        translate=(0.0, -0.205, 0.026),
    )
    top_pad = _filleted_box(
        (0.16, 0.10, 0.012),
        radius=0.010,
        centered=(True, True, False),
        translate=(0.0, -0.265, 0.038),
    )

    return neck.union(plate).union(top_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixture_tree")

    frame_color = model.material("frame_gray", color=(0.28, 0.30, 0.33, 1.0))
    upper_color = model.material("upper_arm_silver", color=(0.70, 0.72, 0.75, 1.0))
    lower_color = model.material("lower_plate_silver", color=(0.62, 0.64, 0.67, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((BASE_W, BASE_D, BASE_H)),
        material=frame_color,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        name="base_shell",
    )
    spine.visual(
        Box((SPINE_W, SPINE_D, SPINE_H)),
        material=frame_color,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + SPINE_H / 2.0)),
        name="spine_shell",
    )
    spine.visual(
        Box((0.092, 0.026, 0.14)),
        material=frame_color,
        origin=Origin(xyz=(0.0, 0.058, UPPER_AXIS[2])),
        name="upper_backstrap",
    )
    spine.visual(
        Box((0.014, 0.086, 0.18)),
        material=frame_color,
        origin=Origin(xyz=(0.025, 0.095, UPPER_AXIS[2])),
        name="upper_cheek_left",
    )
    spine.visual(
        Box((0.014, 0.086, 0.18)),
        material=frame_color,
        origin=Origin(xyz=(-0.025, 0.095, UPPER_AXIS[2])),
        name="upper_cheek_right",
    )
    spine.visual(
        Box((0.010, 0.028, 0.032)),
        material=frame_color,
        origin=Origin(xyz=(0.037, UPPER_AXIS[1], UPPER_AXIS[2])),
        name="upper_boss_left",
    )
    spine.visual(
        Box((0.010, 0.028, 0.032)),
        material=frame_color,
        origin=Origin(xyz=(-0.037, UPPER_AXIS[1], UPPER_AXIS[2])),
        name="upper_boss_right",
    )
    spine.visual(
        Box((0.18, 0.12, 0.16)),
        material=frame_color,
        origin=Origin(xyz=(0.0, LOWER_AXIS[1], 0.37)),
        name="pedestal_block",
    )
    spine.visual(
        Box((0.08, 0.03, 0.18)),
        material=frame_color,
        origin=Origin(xyz=(0.0, -0.0585, 0.37)),
        name="pedestal_brace",
    )
    spine.visual(
        Box((0.14, 0.10, 0.03)),
        material=frame_color,
        origin=Origin(xyz=(0.0, LOWER_AXIS[1], 0.465)),
        name="pedestal_cap",
    )
    spine.visual(
        Cylinder(radius=0.014, length=0.04),
        material=frame_color,
        origin=Origin(xyz=(0.0, LOWER_AXIS[1], 0.50)),
        name="lower_stub_post",
    )
    spine.visual(
        Cylinder(radius=0.034, length=0.006),
        material=frame_color,
        origin=Origin(xyz=(0.0, LOWER_AXIS[1], 0.517)),
        name="lower_thrust_washer",
    )
    spine.visual(
        Cylinder(radius=0.065, length=0.008),
        material=frame_color,
        origin=Origin(xyz=(0.0, LOWER_AXIS[1], 0.476)),
        name="lower_flange",
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        Box((0.036, 0.032, 0.045)),
        material=upper_color,
        origin=Origin(),
        name="upper_hub_shell",
    )
    upper_branch.visual(
        Box((0.046, 0.130, 0.022)),
        material=upper_color,
        origin=Origin(xyz=(0.0, 0.081, 0.0)),
        name="upper_bridge",
    )
    upper_branch.visual(
        Box((0.120, 0.038, 0.022)),
        material=upper_color,
        origin=Origin(xyz=(0.0, 0.210, 0.0)),
        name="upper_fork_base",
    )
    upper_branch.visual(
        Box((0.048, 0.052, 0.022)),
        material=upper_color,
        origin=Origin(xyz=(0.0, 0.171, 0.0)),
        name="upper_neck",
    )
    upper_branch.visual(
        Box((0.032, 0.146, 0.022)),
        material=upper_color,
        origin=Origin(xyz=(0.044, 0.292, 0.0)),
        name="upper_tine_left",
    )
    upper_branch.visual(
        Box((0.032, 0.146, 0.022)),
        material=upper_color,
        origin=Origin(xyz=(-0.044, 0.292, 0.0)),
        name="upper_tine_right",
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        Cylinder(radius=0.045, length=0.014),
        material=lower_color,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        name="lower_hub_shell",
    )
    lower_branch.visual(
        Box((0.09, 0.10, 0.014)),
        material=lower_color,
        origin=Origin(xyz=(0.0, -0.072, 0.007)),
        name="lower_neck",
    )
    lower_branch.visual(
        Box((0.24, 0.18, 0.016)),
        material=lower_color,
        origin=Origin(xyz=(0.0, -0.212, 0.008)),
        name="lower_plate_shell",
    )
    lower_branch.visual(
        Box((0.16, 0.10, 0.012)),
        material=lower_color,
        origin=Origin(xyz=(0.0, -0.260, 0.022)),
        name="lower_pad",
    )

    model.articulation(
        "spine_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_branch,
        origin=Origin(xyz=UPPER_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-0.90,
            upper=0.80,
        ),
    )
    model.articulation(
        "spine_to_lower_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_branch,
        origin=Origin(xyz=LOWER_AXIS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=-1.00,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    spine = object_model.get_part("spine")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")
    upper_joint = object_model.get_articulation("spine_to_upper_branch")
    lower_joint = object_model.get_articulation("spine_to_lower_branch")

    ctx.check(
        "upper branch uses horizontal cross-shaft axis",
        tuple(round(v, 6) for v in upper_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={upper_joint.axis}",
    )
    ctx.check(
        "lower branch uses vertical stub axis",
        tuple(round(v, 6) for v in lower_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_joint.axis}",
    )
    ctx.check(
        "upper articulation spans realistic pitch range",
        upper_joint.motion_limits is not None
        and upper_joint.motion_limits.lower is not None
        and upper_joint.motion_limits.upper is not None
        and upper_joint.motion_limits.lower < 0.0 < upper_joint.motion_limits.upper,
        details=f"limits={upper_joint.motion_limits}",
    )
    ctx.check(
        "lower articulation spans realistic swivel range",
        lower_joint.motion_limits is not None
        and lower_joint.motion_limits.lower is not None
        and lower_joint.motion_limits.upper is not None
        and lower_joint.motion_limits.lower < 0.0 < lower_joint.motion_limits.upper,
        details=f"limits={lower_joint.motion_limits}",
    )

    ctx.expect_origin_gap(
        upper_branch,
        spine,
        axis="y",
        min_gap=0.09,
        max_gap=0.13,
        name="upper branch hub is mounted on the front side of the spine",
    )
    ctx.expect_origin_gap(
        spine,
        lower_branch,
        axis="y",
        min_gap=0.12,
        max_gap=0.16,
        name="lower branch hub is mounted on the back side pedestal",
    )
    ctx.expect_contact(
        upper_branch,
        spine,
        elem_a="upper_hub_shell",
        elem_b="upper_cheek_left",
        name="upper hub contacts the left side cheek",
    )
    ctx.expect_contact(
        upper_branch,
        spine,
        elem_a="upper_hub_shell",
        elem_b="upper_cheek_right",
        name="upper hub contacts the right side cheek",
    )
    ctx.expect_contact(
        lower_branch,
        spine,
        elem_a="lower_hub_shell",
        elem_b="lower_thrust_washer",
        name="lower hub sits on the pedestal thrust washer",
    )
    ctx.expect_overlap(
        upper_branch,
        spine,
        axes="x",
        min_overlap=0.05,
        name="upper fork branch is centered on spine width",
    )
    ctx.expect_overlap(
        lower_branch,
        spine,
        axes="x",
        min_overlap=0.08,
        name="lower plate branch is centered on spine width",
    )
    ctx.expect_origin_gap(
        upper_branch,
        lower_branch,
        axis="z",
        min_gap=0.40,
        name="upper branch sits well above lower branch",
    )

    with ctx.pose({upper_joint: -0.75}):
        ctx.expect_gap(
            upper_branch,
            lower_branch,
            axis="z",
            min_gap=0.14,
            name="pitched upper fork still clears lower plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
