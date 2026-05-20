from __future__ import annotations

from math import pi

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


SHOULDER_HEIGHT = 0.130
SHOULDER_LINK_LENGTH = 0.360
FOREARM_LENGTH = 0.280


def _add_box_tube(
    part,
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    x0: float,
    zc: float,
    material: str,
    prefix: str,
) -> None:
    """Four overlapping walls that read as a rigid rectangular box-section tube."""
    cx = x0 + length / 2.0
    overlap = 0.001
    part.visual(
        Box((length, width, wall + overlap)),
        origin=Origin(xyz=(cx, 0.0, zc + height / 2.0 - wall / 2.0)),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((length, width, wall + overlap)),
        origin=Origin(xyz=(cx, 0.0, zc - height / 2.0 + wall / 2.0)),
        material=material,
        name=f"{prefix}_bottom_wall",
    )
    for side, y in (("side_0", width / 2.0 - wall / 2.0), ("side_1", -width / 2.0 + wall / 2.0)):
        part.visual(
            Box((length, wall + overlap, height)),
            origin=Origin(xyz=(cx, y, zc)),
            material=material,
            name=f"{prefix}_{side}_wall",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_serial_elbow_arm")

    model.material("dark_cast_metal", rgba=(0.13, 0.14, 0.16, 1.0))
    model.material("blue_powdercoat", rgba=(0.13, 0.27, 0.48, 1.0))
    model.material("brushed_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    model.material("black_fasteners", rgba=(0.03, 0.03, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.120, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="dark_cast_metal",
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material="dark_cast_metal",
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.074, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT - 0.009)),
        material="brushed_steel",
        name="top_bearing",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=SHOULDER_HEIGHT),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT / 2.0)),
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="brushed_steel",
        name="shoulder_hub",
    )
    shoulder_link.visual(
        Cylinder(radius=0.058, length=0.043),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material="blue_powdercoat",
        name="shoulder_riser",
    )
    _add_box_tube(
        shoulder_link,
        length=0.239,
        width=0.058,
        height=0.042,
        wall=0.007,
        x0=0.056,
        zc=0.064,
        material="blue_powdercoat",
        prefix="shoulder_tube",
    )
    shoulder_link.visual(
        Cylinder(radius=0.066, length=0.028),
        origin=Origin(xyz=(SHOULDER_LINK_LENGTH, 0.0, 0.050)),
        material="brushed_steel",
        name="elbow_bearing",
    )
    shoulder_link.visual(
        Box((0.080, 0.060, 0.024)),
        origin=Origin(xyz=(SHOULDER_LINK_LENGTH - 0.040, 0.0, 0.050)),
        material="blue_powdercoat",
        name="elbow_block",
    )
    shoulder_link.inertial = Inertial.from_geometry(
        Box((SHOULDER_LINK_LENGTH, 0.070, 0.080)),
        mass=1.1,
        origin=Origin(xyz=(SHOULDER_LINK_LENGTH / 2.0, 0.0, 0.045)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.060, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material="brushed_steel",
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material="blue_powdercoat",
        name="elbow_riser",
    )
    _add_box_tube(
        forearm,
        length=0.165,
        width=0.050,
        height=0.036,
        wall=0.006,
        x0=0.058,
        zc=0.046,
        material="blue_powdercoat",
        prefix="forearm_tube",
    )
    forearm.visual(
        Box((0.070, 0.046, 0.034)),
        origin=Origin(xyz=(FOREARM_LENGTH - 0.035, 0.0, 0.046)),
        material="blue_powdercoat",
        name="wrist_adapter",
    )
    forearm.visual(
        Box((0.018, 0.110, 0.086)),
        origin=Origin(xyz=(FOREARM_LENGTH + 0.009, 0.0, 0.046)),
        material="brushed_steel",
        name="mounting_plate",
    )
    for row, z in enumerate((0.019, 0.073)):
        for col, y in enumerate((-0.036, 0.036)):
            forearm.visual(
                Cylinder(radius=0.0075, length=0.008),
                origin=Origin(
                    xyz=(FOREARM_LENGTH + 0.021, y, z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material="black_fasteners",
                name=f"plate_bolt_{row}_{col}",
            )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH + 0.030, 0.120, 0.090)),
        mass=0.8,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.045)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.75, upper=1.75, effort=55.0, velocity=1.8),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forearm,
        origin=Origin(xyz=(SHOULDER_LINK_LENGTH, 0.0, 0.064)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=38.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    base = object_model.get_part("base")
    shoulder_link = object_model.get_part("shoulder_link")
    forearm = object_model.get_part("forearm")

    ctx.check(
        "two rotary joints",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
        details=f"shoulder={shoulder.articulation_type}, elbow={elbow.articulation_type}",
    )
    ctx.expect_contact(
        shoulder_link,
        base,
        elem_a="shoulder_hub",
        elem_b="top_bearing",
        contact_tol=0.001,
        name="shoulder hub sits on bearing",
    )
    ctx.expect_contact(
        forearm,
        shoulder_link,
        elem_a="elbow_hub",
        elem_b="elbow_bearing",
        contact_tol=0.001,
        name="elbow hub sits on bearing",
    )
    ctx.expect_overlap(
        forearm,
        shoulder_link,
        axes="xy",
        elem_a="elbow_hub",
        elem_b="elbow_bearing",
        min_overlap=0.050,
        name="elbow hub footprint is captured",
    )

    rest_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.75}):
        yawed_elbow_pos = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder yaw moves elbow",
        rest_elbow_pos is not None
        and yawed_elbow_pos is not None
        and yawed_elbow_pos[1] > rest_elbow_pos[1] + 0.20,
        details=f"rest={rest_elbow_pos}, yawed={yawed_elbow_pos}",
    )

    def _elem_center_x(aabb):
        return (aabb[0][0] + aabb[1][0]) / 2.0 if aabb is not None else None

    rest_plate = ctx.part_element_world_aabb(forearm, elem="mounting_plate")
    with ctx.pose({elbow: 0.85}):
        bent_plate = ctx.part_element_world_aabb(forearm, elem="mounting_plate")
    rest_x = _elem_center_x(rest_plate)
    bent_x = _elem_center_x(bent_plate)
    ctx.check(
        "elbow rotates end plate",
        rest_x is not None and bent_x is not None and bent_x < rest_x - 0.035,
        details=f"rest_plate={rest_plate}, bent_plate={bent_plate}",
    )

    return ctx.report()


object_model = build_object_model()
