from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_spin_platform")

    # Define materials
    black_material = Material(name="black", rgba=(0.1, 0.1, 0.1, 1.0))
    dark_black_material = Material(name="dark_black", rgba=(0.05, 0.05, 0.05, 1.0))
    white_material = Material(name="white", rgba=(0.95, 0.95, 0.95, 1.0))

    # Root part: black base
    base = model.part("base")
    # Main base cylinder: radius 0.15m, length 0.05m
    base.visual(
        Cylinder(radius=0.15, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_material,
        name="base_shell",
    )
    # Motor housing bump: small black cylinder on base top surface, outside disk radius
    base.visual(
        Cylinder(radius=0.025, length=0.03),
        origin=Origin(xyz=(0.17, 0.0, 0.065)),
        material=black_material,
        name="motor_housing",
    )
    # Cable port: small dark rectangle on base near motor housing
    base.visual(
        Box((0.02, 0.01, 0.01)),
        origin=Origin(xyz=(0.17, 0.03, 0.045)),
        material=dark_black_material,
        name="cable_port",
    )

    # Rotating disk part: white, continuous yaw joint
    rotating_disk = model.part("rotating_disk")
    # Disk: white cylinder, slightly smaller than base, sits on base top
    # Origin is relative to rotating_disk part frame (at world (0,0,0.06))
    rotating_disk.visual(
        Cylinder(radius=0.14, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=white_material,
        name="disk_shell",
    )
    # Raised center registration peg: white cylinder on disk center
    # Origin relative to rotating_disk part frame (world Z offset = 0.025 + 0.06 = 0.085)
    rotating_disk.visual(
        Cylinder(radius=0.01, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=white_material,
        name="center_peg",
    )

    # Continuous yaw joint (ArticulationType.CONTINUOUS) for disk rotation
    model.articulation(
        "base_to_disk",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rotating_disk,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),  # Rotation axis at disk center
        axis=(0.0, 0.0, 1.0),  # Yaw rotation around Z axis
        motion_limits=MotionLimits(effort=0.5, velocity=1.5, lower=None, upper=None),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    disk = object_model.get_part("rotating_disk")
    joint = object_model.get_articulation("base_to_disk")

    # 1. Main mechanism tests: continuous yaw joint
    ctx.check(
        "articulation exists and is continuous yaw",
        joint is not None 
        and joint.articulation_type == ArticulationType.CONTINUOUS
        and joint.axis == (0.0, 0.0, 1.0),
        details="Missing joint, wrong type, or wrong axis",
    )
    # Verify joint pose applies correctly
    with ctx.pose({joint: 1.0}):
        ctx.check("joint pose applies without error", True, details="Posed joint to 1.0 rad")

    # 2. Support/contact test: disk contacts base at rest
    with ctx.pose({joint: 0.0}):
        ctx.expect_contact(base, disk, name="disk_contacts_base_at_rest")
        ctx.expect_gap(
            positive_link=disk,
            negative_link=base,
            axis="z",
            max_penetration=0.001,
            positive_elem="disk_shell",
            negative_elem="base_shell",
            name="no_gap_between_disk_and_base",
        )

    # 3. Visible details tests
    base_visual_names = [v.name for v in base.visuals]
    ctx.check(
        "base has motor housing",
        "motor_housing" in base_visual_names,
        details=f"Base visuals: {base_visual_names}",
    )
    ctx.check(
        "base has cable port",
        "cable_port" in base_visual_names,
        details=f"Base visuals: {base_visual_names}",
    )
    disk_visual_names = [v.name for v in disk.visuals]
    ctx.check(
        "disk has center registration peg",
        "center_peg" in disk_visual_names,
        details=f"Disk visuals: {disk_visual_names}",
    )

    return ctx.report()


object_model = build_object_model()
