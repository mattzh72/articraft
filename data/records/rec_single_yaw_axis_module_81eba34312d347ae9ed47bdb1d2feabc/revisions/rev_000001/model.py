from __future__ import annotations

from math import pi, cos, sin
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_pan_base")

    # Materials
    model.material("pedestal_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("metal_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("bearing_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("bolt_dark", rgba=(0.25, 0.25, 0.28, 1.0))
    model.material("mount_black", rgba=(0.12, 0.12, 0.14, 1.0))

    # --- FIXED BASE (pedestal) ---
    base = model.part("pedestal")
    
    # Main pedestal body - round base that sits on ground
    base.visual(
        Cylinder(radius=0.15, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material="pedestal_dark",
        name="pedestal_body",
    )
    
    # Base flange at bottom for stability (wider, thinner)
    base.visual(
        Cylinder(radius=0.18, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material="pedestal_dark",
        name="base_flange",
    )
    
    # Central bearing post that protrudes upward - the rotating part sits on this
    base.visual(
        Cylinder(radius=0.035, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material="bearing_steel",
        name="bearing_post",
    )
    
    # Top thrust washer/plate on the bearing post
    base.visual(
        Cylinder(radius=0.055, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        material="metal_silver",
        name="thrust_washer",
    )

    # --- ROTATING DISK ---
    # Part origin is at joint location (top of thrust_washer, z=0.268)
    # All rotator visuals are positioned relative to this part origin
    rotator = model.part("rotator")
    
    # Main rotating disk - sits directly on top of thrust_washer
    # thrust_washer top is at z=0.268 (world), so disk bottom should be at 0.268
    # Cylinder length=0.025, so center is at 0.268 + 0.0125 = 0.2805 (world)
    # Relative to part origin at 0.268: offset = 0.0125
    rotator.visual(
        Cylinder(radius=0.16, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="metal_silver",
        name="rotating_disk",
    )
    
    # Central bearing collar - extends downward to engage with bearing_post
    # bearing_post extends to z=0.26 (world), collar should overlap with it
    # Collar length=0.04, bottom at ~0.24 (world), top at ~0.28 (world)
    # Center at 0.26 (world), relative to part origin: offset = -0.008
    rotator.visual(
        Cylinder(radius=0.045, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material="bearing_steel",
        name="bearing_collar",
    )
    
    # Bolt circle - 6 bolts arranged in a circle on the rotating disk
    # Bolts sit on top of rotating disk
    # Disk top is at z=0.2925 (world: 0.268 + 0.0125 + 0.0125)
    # Bolt bottom at 0.2925, bolt length=0.030, so center at 0.3075 (world)
    # Relative to part origin: offset = 0.3075 - 0.268 = 0.0395
    bolt_radius = 0.12  # radius of bolt circle
    num_bolts = 6
    for i in range(num_bolts):
        angle = (2.0 * pi * i) / num_bolts
        bolt_x = bolt_radius * cos(angle)
        bolt_y = bolt_radius * sin(angle)
        rotator.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(bolt_x, bolt_y, 0.0395)),
            material="bolt_dark",
            name=f"bolt_{i}",
        )
    
    # Camera mounting plate - small rectangular plate on top of rotating disk
    # Rotating disk top is at z=0.025 (local: 0.0125 + 0.0125)
    # Plate bottom sits on disk top, thickness=0.012, so center at 0.025 + 0.006 = 0.031
    rotator.visual(
        Box((0.10, 0.08, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material="mount_black",
        name="camera_mount_plate",
    )
    
    # Mounting holes on the camera plate (4 corner holes)
    # Holes go through the plate, centered at same z as plate
    hole_positions = [(-0.035, -0.025), (0.035, -0.025), (-0.035, 0.025), (0.035, 0.025)]
    for idx, (hx, hy) in enumerate(hole_positions):
        rotator.visual(
            Cylinder(radius=0.004, length=0.015),
            origin=Origin(xyz=(hx, hy, 0.031)),
            material="bolt_dark",
            name=f"mount_hole_{idx}",
        )

    # --- ARTICULATION: Vertical yaw joint through center ---
    # Joint at center of rotation (top of thrust_washer)
    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotator,
        origin=Origin(xyz=(0.0, 0.0, 0.268)),
        axis=(0.0, 0.0, 1.0),  # Vertical axis for yaw rotation
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-pi,  # +/- 180 degrees
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal")
    rotator = object_model.get_part("rotator")
    yaw_joint = object_model.get_articulation("yaw_joint")

    # Basic existence checks
    ctx.check("pedestal_exists", base is not None, "Pedestal part missing")
    ctx.check("rotator_exists", rotator is not None, "Rotator part missing")
    ctx.check("yaw_joint_exists", yaw_joint is not None, "Yaw joint missing")

    if base is None or rotator is None or yaw_joint is None:
        return ctx.report()

    # Check articulation configuration
    ctx.check(
        "yaw_axis_vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        f"Expected vertical axis (0,0,1), got {yaw_joint.axis}",
    )
    ctx.check(
        "yaw_has_limits",
        yaw_joint.motion_limits is not None
        and yaw_joint.motion_limits.lower is not None
        and yaw_joint.motion_limits.upper is not None,
        "Yaw joint should have motion limits",
    )

    # Check that rotating disk is supported (not floating)
    # rotating_disk bottom at z=0.268, thrust_washer top at z=0.268
    ctx.expect_gap(
        "rotator",
        "pedestal",
        axis="z",
        min_gap=0.0,
        max_gap=0.025,
        positive_elem="rotating_disk",
        negative_elem="thrust_washer",
        name="rotating_disk_above_base",
    )

    # Test rotation - check that rotator moves in yaw
    rest_pos = ctx.part_world_position(rotator)
    
    # Test positive rotation (90 degrees)
    with ctx.pose({yaw_joint: pi / 2.0}):
        pos_90 = ctx.part_world_position(rotator)
        ctx.check(
            "yaw_positive_rotation",
            rest_pos is not None and pos_90 is not None 
            and abs(pos_90[0] - rest_pos[0]) < 0.01  # X should stay ~same (rotating around Z)
            and abs(pos_90[1] - rest_pos[1]) < 0.01,  # Y should stay ~same
            f"Position at 90deg: rest={rest_pos}, pos_90={pos_90}",
        )
        # Check that camera plate is still on top
        ctx.expect_gap(
            "rotator",
            "pedestal",
            axis="z",
            min_gap=0.0,
            max_gap=0.025,
            positive_elem="rotating_disk",
            negative_elem="thrust_washer",
            name="rotator_stays_above_base_at_90deg",
        )

    # Test full range - check that rotator doesn't collide unexpectedly
    with ctx.pose({yaw_joint: pi}):
        ctx.expect_gap(
            "rotator",
            "pedestal",
            axis="z",
            min_gap=0.0,
            max_gap=0.025,
            positive_elem="rotating_disk",
            negative_elem="thrust_washer",
            name="rotator_stays_above_base_at_180deg",
        )

    # Check bolt circle is on the rotating disk
    for i in range(6):
        ctx.expect_within(
            "rotator",
            "rotator",
            axes="xy",
            inner_elem=f"bolt_{i}",
            outer_elem="rotating_disk",
            margin=0.01,
            name=f"bolt_{i}_on_rotating_disk",
        )

    # Check camera mount plate is on the rotating disk
    ctx.expect_within(
        "rotator",
        "rotator",
        axes="xy",
        inner_elem="camera_mount_plate",
        outer_elem="rotating_disk",
        margin=0.01,
        name="camera_plate_on_rotating_disk",
    )

    # Allow intentional overlaps for bearing mechanism
    # 1. Bearing collar overlaps with bearing post (collar sits around post)
    ctx.allow_overlap(
        "pedestal",
        "rotator",
        elem_a="bearing_post",
        elem_b="bearing_collar",
        reason="Bearing collar is intentionally nested around the bearing post for rotational support",
    )
    
    # 2. Bearing collar overlaps with thrust washer (collar sits on washer)
    ctx.allow_overlap(
        "pedestal",
        "rotator",
        elem_a="thrust_washer",
        elem_b="bearing_collar",
        reason="Bearing collar intentionally sits on thrust washer for axial support",
    )
    
    # Prove the overlaps are intentional and correct
    ctx.expect_overlap(
        "pedestal",
        "rotator",
        axes="xy",
        elem_a="bearing_post",
        elem_b="bearing_collar",
        min_overlap=0.01,
        name="bearing_collar_overlaps_post_in_xy",
    )

    return ctx.report()


object_model = build_object_model()
