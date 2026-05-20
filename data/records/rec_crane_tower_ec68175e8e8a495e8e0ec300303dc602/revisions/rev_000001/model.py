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
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.018, 0.0, -0.125),
            (0.034, 0.0, -0.155),
            (0.036, 0.0, -0.192),
            (0.018, 0.0, -0.226),
            (-0.012, 0.0, -0.236),
            (-0.034, 0.0, -0.212),
            (-0.028, 0.0, -0.176),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "gantry_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhead_travelling_gantry_crane")

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.69, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    portal_blue = model.material("portal_blue", rgba=(0.16, 0.32, 0.55, 1.0))
    bridge_yellow = model.material("bridge_yellow", rgba=(0.88, 0.76, 0.16, 1.0))
    machine_grey = model.material("machine_grey", rgba=(0.40, 0.43, 0.46, 1.0))
    cable_black = model.material("cable_black", rgba=(0.10, 0.10, 0.11, 1.0))
    signal_red = model.material("signal_red", rgba=(0.72, 0.12, 0.10, 1.0))

    hook_mesh = _build_hook_mesh()

    track_base = model.part("track_base")
    track_base.visual(
        Box((6.8, 10.8, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=concrete,
        name="base_slab",
    )
    track_base.visual(
        Box((0.16, 10.4, 0.12)),
        origin=Origin(xyz=(-2.8, 0.0, 0.22)),
        material=rail_steel,
        name="left_runway_rail",
    )
    track_base.visual(
        Box((0.16, 10.4, 0.12)),
        origin=Origin(xyz=(2.8, 0.0, 0.22)),
        material=rail_steel,
        name="right_runway_rail",
    )
    track_base.inertial = Inertial.from_geometry(
        Box((6.8, 10.8, 0.28)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    portal_frame = model.part("portal_frame")
    bogie_size = (0.36, 0.78, 0.24)
    for x, y, name in (
        (-2.8, -3.2, "front_left_bogie"),
        (2.8, -3.2, "front_right_bogie"),
        (-2.8, 3.2, "rear_left_bogie"),
        (2.8, 3.2, "rear_right_bogie"),
    ):
        portal_frame.visual(
            Box(bogie_size),
            origin=Origin(xyz=(x, y, 0.40)),
            material=machine_grey,
            name=name,
        )

    leg_height = 3.62
    leg_center_z = 0.52 + leg_height * 0.5
    for x, y, name in (
        (-2.8, -3.2, "front_left_leg"),
        (2.8, -3.2, "front_right_leg"),
        (-2.8, 3.2, "rear_left_leg"),
        (2.8, 3.2, "rear_right_leg"),
    ):
        portal_frame.visual(
            Box((0.30, 0.30, leg_height)),
            origin=Origin(xyz=(x, y, leg_center_z)),
            material=portal_blue,
            name=name,
        )

    portal_frame.visual(
        Box((0.24, 6.40, 0.28)),
        origin=Origin(xyz=(-2.53, 0.0, 0.38)),
        material=portal_blue,
        name="left_lower_sill",
    )
    portal_frame.visual(
        Box((0.24, 6.40, 0.28)),
        origin=Origin(xyz=(2.53, 0.0, 0.38)),
        material=portal_blue,
        name="right_lower_sill",
    )
    portal_frame.visual(
        Box((6.0, 0.22, 0.20)),
        origin=Origin(xyz=(0.0, -3.2, 2.20)),
        material=portal_blue,
        name="front_crossbeam_mid",
    )
    portal_frame.visual(
        Box((6.0, 0.22, 0.20)),
        origin=Origin(xyz=(0.0, 3.2, 2.20)),
        material=portal_blue,
        name="rear_crossbeam_mid",
    )
    portal_frame.visual(
        Box((0.42, 7.20, 0.26)),
        origin=Origin(xyz=(-2.8, 0.0, 4.23)),
        material=portal_blue,
        name="left_side_girder",
    )
    portal_frame.visual(
        Box((0.42, 7.20, 0.26)),
        origin=Origin(xyz=(2.8, 0.0, 4.23)),
        material=portal_blue,
        name="right_side_girder",
    )
    portal_frame.visual(
        Box((0.18, 7.20, 0.05)),
        origin=Origin(xyz=(-2.8, 0.0, 4.385)),
        material=rail_steel,
        name="portal_rail_left",
    )
    portal_frame.visual(
        Box((0.18, 7.20, 0.05)),
        origin=Origin(xyz=(2.8, 0.0, 4.385)),
        material=rail_steel,
        name="portal_rail_right",
    )
    portal_frame.visual(
        Box((6.02, 0.24, 0.26)),
        origin=Origin(xyz=(0.0, -3.2, 4.23)),
        material=portal_blue,
        name="front_top_beam",
    )
    portal_frame.visual(
        Box((6.02, 0.24, 0.26)),
        origin=Origin(xyz=(0.0, 3.2, 4.23)),
        material=portal_blue,
        name="rear_top_beam",
    )

    for y, prefix in ((-3.2, "front"), (3.2, "rear")):
        _add_member(
            portal_frame,
            (-2.64, y, 0.55),
            (2.64, y, 3.95),
            0.055,
            machine_grey,
            name=f"{prefix}_brace_a",
        )
        _add_member(
            portal_frame,
            (2.64, y, 0.55),
            (-2.64, y, 3.95),
            0.055,
            machine_grey,
            name=f"{prefix}_brace_b",
        )

    for x, prefix in ((-2.8, "left"), (2.8, "right")):
        _add_member(
            portal_frame,
            (x, -3.05, 0.52),
            (x, 3.05, 4.10),
            0.055,
            machine_grey,
            name=f"{prefix}_side_brace_a",
        )
        _add_member(
            portal_frame,
            (x, 3.05, 0.52),
            (x, -3.05, 4.10),
            0.055,
            machine_grey,
            name=f"{prefix}_side_brace_b",
        )

    portal_frame.inertial = Inertial.from_geometry(
        Box((6.2, 7.4, 4.45)),
        mass=9500.0,
        origin=Origin(xyz=(0.0, 0.0, 2.225)),
    )

    bridge = model.part("bridge_girder")
    bridge.visual(
        Box((0.52, 1.18, 0.18)),
        origin=Origin(xyz=(-2.8, 0.0, 0.09)),
        material=machine_grey,
        name="end_truck_left",
    )
    bridge.visual(
        Box((0.52, 1.18, 0.18)),
        origin=Origin(xyz=(2.8, 0.0, 0.09)),
        material=machine_grey,
        name="end_truck_right",
    )
    bridge.visual(
        Box((5.08, 0.26, 0.60)),
        origin=Origin(xyz=(0.0, -0.38, 0.48)),
        material=bridge_yellow,
        name="main_girder_left",
    )
    bridge.visual(
        Box((5.08, 0.26, 0.60)),
        origin=Origin(xyz=(0.0, 0.38, 0.48)),
        material=bridge_yellow,
        name="main_girder_right",
    )
    bridge.visual(
        Box((5.08, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.38, 0.82)),
        material=machine_grey,
        name="top_flange_left",
    )
    bridge.visual(
        Box((5.08, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.38, 0.82)),
        material=machine_grey,
        name="top_flange_right",
    )
    bridge.visual(
        Box((4.90, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -0.38, 0.875)),
        material=rail_steel,
        name="trolley_rail_left",
    )
    bridge.visual(
        Box((4.90, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.38, 0.875)),
        material=rail_steel,
        name="trolley_rail_right",
    )
    bridge.visual(
        Box((0.16, 0.16, 0.20)),
        origin=Origin(xyz=(-2.42, -0.38, 0.97)),
        material=signal_red,
        name="left_end_stop_inner",
    )
    bridge.visual(
        Box((0.16, 0.16, 0.20)),
        origin=Origin(xyz=(2.42, -0.38, 0.97)),
        material=signal_red,
        name="left_end_stop_outer",
    )
    bridge.visual(
        Box((0.16, 0.16, 0.20)),
        origin=Origin(xyz=(-2.42, 0.38, 0.97)),
        material=signal_red,
        name="right_end_stop_inner",
    )
    bridge.visual(
        Box((0.16, 0.16, 0.20)),
        origin=Origin(xyz=(2.42, 0.38, 0.97)),
        material=signal_red,
        name="right_end_stop_outer",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((5.8, 1.2, 1.05)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    trolley = model.part("crab_trolley")
    trolley.visual(
        Box((0.42, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.38, 0.04)),
        material=machine_grey,
        name="trolley_pad_left",
    )
    trolley.visual(
        Box((0.42, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.38, 0.04)),
        material=machine_grey,
        name="trolley_pad_right",
    )
    trolley.visual(
        Box((0.76, 0.84, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=bridge_yellow,
        name="trolley_frame",
    )
    trolley.visual(
        Box((0.42, 0.48, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=machine_grey,
        name="trolley_machinery",
    )
    trolley.visual(
        Cylinder(radius=0.11, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.27), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_grey,
        name="hoist_drum",
    )
    trolley.visual(
        Cylinder(radius=0.010, length=1.40),
        origin=Origin(xyz=(0.0, -0.10, -0.62)),
        material=cable_black,
        name="left_hoist_cable",
    )
    trolley.visual(
        Cylinder(radius=0.010, length=1.40),
        origin=Origin(xyz=(0.0, 0.10, -0.62)),
        material=cable_black,
        name="right_hoist_cable",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.80, 0.90, 1.70)),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, -0.45)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Box((0.20, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=machine_grey,
        name="hook_headplate",
    )
    hook_block.visual(
        Box((0.24, 0.26, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, -0.25)),
        material=bridge_yellow,
        name="hook_body",
    )
    hook_block.visual(
        Cylinder(radius=0.028, length=0.14),
        origin=Origin(xyz=(0.018, 0.0, -0.52)),
        material=machine_grey,
        name="hook_shank",
    )
    hook_block.visual(
        hook_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.46)),
        material=signal_red,
        name="hook_steel",
    )
    hook_block.inertial = Inertial.from_geometry(
        Box((0.35, 0.36, 0.88)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, -0.44)),
    )

    model.articulation(
        "track_to_portal",
        ArticulationType.FIXED,
        parent=track_base,
        child=portal_frame,
        origin=Origin(),
    )
    model.articulation(
        "portal_to_bridge_travel",
        ArticulationType.PRISMATIC,
        parent=portal_frame,
        child=bridge,
        origin=Origin(xyz=(0.0, -2.90, 4.41)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=1.2,
            lower=0.0,
            upper=5.80,
        ),
    )
    model.articulation(
        "bridge_to_trolley_travel",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=trolley,
        origin=Origin(xyz=(-1.95, 0.0, 0.89)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=1.0,
            lower=0.0,
            upper=3.90,
        ),
    )
    model.articulation(
        "trolley_to_hook_suspension",
        ArticulationType.FIXED,
        parent=trolley,
        child=hook_block,
        origin=Origin(xyz=(0.0, 0.0, -1.32)),
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

    portal_frame = object_model.get_part("portal_frame")
    bridge = object_model.get_part("bridge_girder")
    trolley = object_model.get_part("crab_trolley")
    hook_block = object_model.get_part("hook_block")

    bridge_travel = object_model.get_articulation("portal_to_bridge_travel")
    trolley_travel = object_model.get_articulation("bridge_to_trolley_travel")

    bridge_upper = bridge_travel.motion_limits.upper or 0.0
    trolley_upper = trolley_travel.motion_limits.upper or 0.0

    with ctx.pose({bridge_travel: 0.0}):
        ctx.expect_contact(
            bridge,
            portal_frame,
            elem_a="end_truck_left",
            elem_b="portal_rail_left",
            name="left end truck seats on left portal rail at start",
        )
        ctx.expect_contact(
            bridge,
            portal_frame,
            elem_a="end_truck_right",
            elem_b="portal_rail_right",
            name="right end truck seats on right portal rail at start",
        )

    with ctx.pose({bridge_travel: bridge_upper}):
        ctx.expect_contact(
            bridge,
            portal_frame,
            elem_a="end_truck_left",
            elem_b="portal_rail_left",
            name="left end truck stays supported at far travel",
        )
        ctx.expect_contact(
            bridge,
            portal_frame,
            elem_a="end_truck_right",
            elem_b="portal_rail_right",
            name="right end truck stays supported at far travel",
        )

    with ctx.pose({bridge_travel: bridge_upper * 0.5, trolley_travel: 0.0}):
        ctx.expect_contact(
            trolley,
            bridge,
            elem_a="trolley_pad_left",
            elem_b="trolley_rail_left",
            name="left trolley pad sits on left trolley rail at start",
        )
        ctx.expect_contact(
            trolley,
            bridge,
            elem_a="trolley_pad_right",
            elem_b="trolley_rail_right",
            name="right trolley pad sits on right trolley rail at start",
        )

    with ctx.pose({bridge_travel: bridge_upper * 0.5, trolley_travel: trolley_upper}):
        ctx.expect_contact(
            trolley,
            bridge,
            elem_a="trolley_pad_left",
            elem_b="trolley_rail_left",
            name="left trolley pad stays on left trolley rail at far reach",
        )
        ctx.expect_contact(
            trolley,
            bridge,
            elem_a="trolley_pad_right",
            elem_b="trolley_rail_right",
            name="right trolley pad stays on right trolley rail at far reach",
        )

    with ctx.pose({bridge_travel: 0.0}):
        bridge_rest_pos = ctx.part_world_position(bridge)
    with ctx.pose({bridge_travel: bridge_upper}):
        bridge_far_pos = ctx.part_world_position(bridge)
    ctx.check(
        "bridge girder travels along the portal rails",
        bridge_rest_pos is not None
        and bridge_far_pos is not None
        and bridge_far_pos[1] > bridge_rest_pos[1] + 5.0,
        details=f"rest={bridge_rest_pos}, far={bridge_far_pos}",
    )

    with ctx.pose({bridge_travel: bridge_upper * 0.5, trolley_travel: 0.0}):
        trolley_rest_pos = ctx.part_world_position(trolley)
    with ctx.pose({bridge_travel: bridge_upper * 0.5, trolley_travel: trolley_upper}):
        trolley_far_pos = ctx.part_world_position(trolley)
    ctx.check(
        "crab trolley traverses across the bridge girder",
        trolley_rest_pos is not None
        and trolley_far_pos is not None
        and trolley_far_pos[0] > trolley_rest_pos[0] + 3.5,
        details=f"rest={trolley_rest_pos}, far={trolley_far_pos}",
    )

    ctx.expect_origin_gap(
        trolley,
        hook_block,
        axis="z",
        min_gap=1.2,
        max_gap=1.5,
        name="hook block hangs below the crab trolley",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
