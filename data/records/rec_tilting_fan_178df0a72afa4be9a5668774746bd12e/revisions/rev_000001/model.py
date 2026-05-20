from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PIVOT_X = -0.14
PIVOT_Z = 1.05
GUARD_RADIUS = 0.36
ROTOR_RADIUS = 0.285


def _guard_disk_cq() -> cq.Workplane:
    """One fused flat industrial wire-guard disk in the local YZ plane."""
    thickness = 0.012
    wire = 0.010

    def disk(radius: float) -> cq.Workplane:
        return cq.Workplane("YZ").circle(radius).extrude(thickness).translate((-thickness / 2.0, 0.0, 0.0))

    def ring(radius: float, width: float = wire) -> cq.Workplane:
        return (
            cq.Workplane("YZ")
            .circle(radius + width / 2.0)
            .circle(radius - width / 2.0)
            .extrude(thickness)
            .translate((-thickness / 2.0, 0.0, 0.0))
        )

    guard = disk(0.052)
    for radius, width in ((0.125, 0.008), (0.215, 0.008), (0.300, 0.009), (GUARD_RADIUS, 0.014)):
        guard = guard.union(ring(radius, width))

    length = GUARD_RADIUS - 0.045
    mid = 0.045 + length / 2.0
    for i in range(16):
        angle = 360.0 * i / 16.0
        bar = (
            cq.Workplane("XY")
            .box(thickness, wire, length)
            .translate((0.0, 0.0, mid))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -angle)
        )
        guard = guard.union(bar)

    return guard


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_tilting_safety_fan")

    safety_yellow = model.material("safety_yellow", rgba=(0.96, 0.72, 0.08, 1.0))
    guard_black = model.material("powder_coated_guard_black", rgba=(0.015, 0.016, 0.014, 1.0))
    cast_steel = model.material("dark_cast_steel", rgba=(0.16, 0.18, 0.18, 1.0))
    parkerized = model.material("parkerized_hardware", rgba=(0.045, 0.045, 0.042, 1.0))
    warning_red = model.material("lockout_red", rgba=(0.78, 0.04, 0.025, 1.0))
    blade_aluminum = model.material("matte_aluminum_blades", rgba=(0.66, 0.69, 0.70, 1.0))

    guard_mesh = mesh_from_cadquery(_guard_disk_cq(), "fused_safety_guard")

    # Root support: base plate, column, yoke cheeks, diagonal braces, stops,
    # index/lockout plate, and visible bolted load paths are one welded stand.
    stand = model.part("stand")
    stand.visual(
        Box((0.82, 0.62, 0.075)),
        origin=Origin(xyz=(-0.08, 0.0, 0.0375)),
        material=cast_steel,
        name="base_plate",
    )
    stand.visual(
        Box((0.68, 0.12, 0.045)),
        origin=Origin(xyz=(-0.10, 0.0, 0.095)),
        material=safety_yellow,
        name="raised_spine",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.68),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.43)),
        material=cast_steel,
        name="column_tube",
    )
    stand.visual(
        Box((0.18, 0.92, 0.070)),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.615)),
        material=safety_yellow,
        name="yoke_crossbeam",
    )
    for side, y in (("0", 0.43), ("1", -0.43)):
        stand.visual(
            Box((0.090, 0.050, 0.69)),
            origin=Origin(xyz=(PIVOT_X, y, 0.925)),
            material=safety_yellow,
            name=f"yoke_cheek_{side}",
        )
        stand.visual(
            Cylinder(radius=0.074, length=0.040),
            origin=Origin(xyz=(PIVOT_X, y + (0.040 if y > 0 else -0.040), PIVOT_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=cast_steel,
            name=f"outer_bearing_cap_{side}",
        )
        stand.visual(
            Box((0.150, 0.018, 0.145)),
            origin=Origin(xyz=(PIVOT_X + 0.075, y * 0.985, PIVOT_Z + 0.025)),
            material=cast_steel,
            name=f"front_stop_block_{side}",
        )
        stand.visual(
            Box((0.150, 0.018, 0.145)),
            origin=Origin(xyz=(PIVOT_X - 0.075, y * 0.985, PIVOT_Z + 0.025)),
            material=cast_steel,
            name=f"rear_stop_block_{side}",
        )

    # Diagonal tube braces from the central column into both yoke cheeks.
    for side, y in (("0", 0.31), ("1", -0.31)):
        length = math.sqrt(y * y + 0.43 * 0.43)
        angle = math.atan2(y, 0.43)
        stand.visual(
            Cylinder(radius=0.020, length=length),
            origin=Origin(xyz=(PIVOT_X, y / 2.0, 0.54 + 0.43 / 2.0), rpy=(-angle, 0.0, 0.0)),
            material=cast_steel,
            name=f"diagonal_brace_{side}",
        )

    # Lockout/index plate and safety hasp logic on one side of the yoke.
    stand.visual(
        Box((0.105, 0.016, 0.250)),
        origin=Origin(xyz=(PIVOT_X - 0.020, -0.494, PIVOT_Z), rpy=(0.0, 0.0, 0.0)),
        material=cast_steel,
        name="index_plate",
    )
    for i, zoff in enumerate((-0.090, -0.045, 0.000, 0.045, 0.090)):
        stand.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(PIVOT_X + 0.020, -0.505, PIVOT_Z + zoff), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=parkerized,
            name=f"index_hole_liner_{i}",
        )
    stand.visual(
        Box((0.140, 0.020, 0.045)),
        origin=Origin(xyz=(PIVOT_X + 0.035, -0.505, PIVOT_Z + 0.110)),
        material=warning_red,
        name="lockout_hasp",
    )

    for i, (x, y) in enumerate(((-0.38, -0.24), (-0.38, 0.24), (0.22, -0.24), (0.22, 0.24))):
        stand.visual(
            Cylinder(radius=0.026, length=0.016),
            origin=Origin(xyz=(x, y, 0.083)),
            material=parkerized,
            name=f"anchor_bolt_{i}",
        )

    # Tilting fan head: guard cage, motor, fixed center boss, side pivot lugs,
    # anti-rotation tabs, and reinforced ears ride as one tilting assembly.
    head = model.part("head")
    for name, x in (("rear_guard", 0.055), ("front_guard", 0.225)):
        head.visual(
            guard_mesh,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=guard_black,
            name=name,
        )
    for i, (y, z) in enumerate(((GUARD_RADIUS, 0.0), (-GUARD_RADIUS, 0.0), (0.0, GUARD_RADIUS), (0.0, -GUARD_RADIUS))):
        head.visual(
            Cylinder(radius=0.008, length=0.158),
            origin=Origin(xyz=(0.140, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=guard_black,
            name=f"guard_tie_rod_{i}",
        )
    head.visual(
        Cylinder(radius=0.125, length=0.220),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_steel,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.066, length=0.050),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_steel,
        name="motor_nose_boss",
    )
    for i, z in enumerate((-0.080, -0.040, 0.040, 0.080)):
        head.visual(
            Box((0.175, 0.012, 0.010)),
            origin=Origin(xyz=(-0.055, 0.0, z)),
            material=parkerized,
            name=f"motor_cooling_rib_{i}",
        )
    for side, y in (("0", 0.335), ("1", -0.335)):
        head.visual(
            Box((0.055, 0.430, 0.080)),
            origin=Origin(xyz=(0.000, y / 2.0, 0.0)),
            material=cast_steel,
            name=f"pivot_ear_{side}",
        )
        head.visual(
            Cylinder(radius=0.045, length=0.140),
            origin=Origin(xyz=(0.000, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=cast_steel,
            name=f"trunnion_boss_{side}",
        )
        head.visual(
            Box((0.220, 0.035, 0.090)),
            origin=Origin(xyz=(0.080, y * 0.96, 0.060)),
            material=parkerized,
            name=f"travel_lug_{side}",
        )

    # Rotating impeller: the mesh blade pack and its shaft/boss are a single
    # rotor part spinning about the explicit motor axis.
    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.073,
                5,
                thickness=0.044,
                blade_pitch_deg=31.0,
                blade_sweep_deg=23.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=15.0, camber=0.12, tip_clearance=0.010),
                hub=FanRotorHub(style="capped", rear_collar_height=0.018, rear_collar_radius=0.058, bore_diameter=0.024),
            ),
            "heavy_duty_rotor",
        ),
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_aluminum,
        name="rotor_blades",
    )
    rotor.visual(
        Cylinder(radius=0.026, length=0.110),
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=parkerized,
        name="rotor_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.171, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=parkerized,
        name="front_hub_cap",
    )

    # A user-facing clamp knob is a separate movable lockout control.
    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        Cylinder(radius=0.055, length=0.034),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warning_red,
        name="knob_disk",
    )
    lock_knob.visual(
        Box((0.145, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=warning_red,
        name="knob_grip",
    )
    lock_knob.visual(
        Cylinder(radius=0.018, length=0.065),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="threaded_stud",
    )

    tilt = model.articulation(
        "stand_to_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.55, lower=-0.55, upper=0.55),
        motion_properties=MotionProperties(damping=2.5, friction=1.2),
    )
    tilt.meta["description"] = "Safety-limited tilt pivot through the reinforced side trunnions."

    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=110.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "stand_to_lock_knob",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lock_knob,
        origin=Origin(xyz=(PIVOT_X + 0.020, -0.545, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.15, friction=0.35),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("lock_knob")
    tilt = object_model.get_articulation("stand_to_head")
    spin = object_model.get_articulation("head_to_rotor")
    lock = object_model.get_articulation("stand_to_lock_knob")

    ctx.check("industrial_fan_parts_present", all(p is not None for p in (stand, head, rotor, knob)), "Missing a required fan part.")
    ctx.check("tilt_spin_and_lock_joints_present", all(j is not None for j in (tilt, spin, lock)), "Missing tilt, rotor spin, or lockout articulation.")

    if head is not None and rotor is not None:
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            margin=0.010,
            inner_elem="rotor_blades",
            outer_elem="front_guard",
            name="rotor disk sits inside the guarded diameter",
        )
        ctx.expect_gap(
            head,
            rotor,
            axis="x",
            min_gap=0.020,
            positive_elem="front_guard",
            negative_elem="rotor_blades",
            name="front guard clears spinning rotor",
        )
        ctx.expect_gap(
            rotor,
            head,
            axis="x",
            min_gap=0.018,
            positive_elem="rotor_blades",
            negative_elem="rear_guard",
            name="rear guard clears spinning rotor",
        )

    if head is not None and tilt is not None:
        rest = ctx.part_world_position(head)
        with ctx.pose({tilt: 0.45}):
            tilted = ctx.part_world_position(head)
            # The part origin stays at the trunnion axis; use the front guard
            # AABB to prove the head actually pitches around that axis.
            guard_aabb = ctx.part_element_world_aabb(head, elem="front_guard")
        if rest is not None and tilted is not None and guard_aabb is not None:
            mins, maxs = guard_aabb
            front_guard_center_z = (mins[2] + maxs[2]) / 2.0
            ctx.check(
                "tilt joint pitches guarded head downward at positive angle",
                front_guard_center_z < PIVOT_Z - 0.035,
                details=f"front_guard_center_z={front_guard_center_z:.3f}, pivot_z={PIVOT_Z:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
