from __future__ import annotations

from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


SHOULDER_Z = 0.200
UPPER_LEN = 0.540
SHOULDER_LIMITS = (-0.35, 1.25)
ELBOW_LIMITS = (-0.85, 1.35)


def _tapered_box_x(
    *,
    length: float,
    root_y: float,
    root_z: float,
    tip_y: float,
    tip_z: float,
    x_start: float,
) -> cq.Workplane:
    """Loft a centered rectangular tube-like solid along local +X."""
    return (
        cq.Workplane("YZ")
        .rect(root_y, root_z)
        .workplane(offset=length)
        .rect(tip_y, tip_z)
        .loft(combine=True)
        .translate((x_start, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_manipulator_arm")

    model.material("powder_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("dark_anodized", rgba=(0.06, 0.07, 0.075, 1.0))
    model.material("satin_aluminum", rgba=(0.63, 0.66, 0.68, 1.0))
    model.material("machined_zinc", rgba=(0.74, 0.75, 0.72, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.015, 0.016, 1.0))

    base_yoke = model.part("base_yoke")
    base_yoke.visual(
        Cylinder(radius=0.180, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material="powder_steel",
        name="bench_foot",
    )
    base_yoke.visual(
        Box((0.160, 0.180, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material="powder_steel",
        name="pedestal",
    )
    base_yoke.visual(
        Box((0.175, 0.170, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material="powder_steel",
        name="yoke_bridge",
    )
    base_yoke.visual(
        Box((0.185, 0.024, 0.176)),
        origin=Origin(xyz=(0.0, 0.070, 0.206)),
        material="powder_steel",
        name="shoulder_plate_pos",
    )
    base_yoke.visual(
        Box((0.185, 0.024, 0.176)),
        origin=Origin(xyz=(0.0, -0.070, 0.206)),
        material="powder_steel",
        name="shoulder_plate_neg",
    )
    for y, name in ((0.091, "shoulder_journal_pos"), (-0.091, "shoulder_journal_neg")):
        base_yoke.visual(
            Cylinder(radius=0.064, length=0.018),
            origin=Origin(xyz=(0.0, y, SHOULDER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="machined_zinc",
            name=name,
        )
    for index, (x, y) in enumerate(((-0.118, -0.096), (-0.118, 0.096), (0.118, -0.096), (0.118, 0.096))):
        base_yoke.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.038)),
            material="machined_zinc",
            name=f"base_bolt_{index}",
        )
    base_yoke.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.036),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.046, length=0.116),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_anodized",
        name="shoulder_hub",
    )
    upper_link.visual(
        mesh_from_cadquery(
            _tapered_box_x(
                length=0.395,
                root_y=0.076,
                root_z=0.064,
                tip_y=0.052,
                tip_z=0.044,
                x_start=0.038,
            ),
            "upper_taper",
            tolerance=0.0008,
        ),
        material="satin_aluminum",
        name="tapered_web",
    )
    upper_link.visual(
        Box((0.062, 0.136, 0.048)),
        origin=Origin(xyz=(0.426, 0.0, 0.0)),
        material="satin_aluminum",
        name="elbow_bridge",
    )
    upper_link.visual(
        Box((0.192, 0.018, 0.154)),
        origin=Origin(xyz=(UPPER_LEN, 0.071, 0.0)),
        material="satin_aluminum",
        name="elbow_plate_pos",
    )
    upper_link.visual(
        Box((0.192, 0.018, 0.154)),
        origin=Origin(xyz=(UPPER_LEN, -0.071, 0.0)),
        material="satin_aluminum",
        name="elbow_plate_neg",
    )
    for y, name in ((0.087, "elbow_journal_pos"), (-0.087, "elbow_journal_neg")):
        upper_link.visual(
            Cylinder(radius=0.078, length=0.014),
            origin=Origin(xyz=(UPPER_LEN, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="machined_zinc",
            name=name,
        )
    for y, name in ((0.070, "elbow_stop_pos"), (-0.070, "elbow_stop_neg")):
        upper_link.visual(
            Box((0.020, 0.018, 0.040)),
            origin=Origin(xyz=(UPPER_LEN - 0.100, y, 0.076)),
            material="rubber_black",
            name=name,
        )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.540, 0.110, 0.090)),
        mass=1.8,
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
    )

    tool_carrier = model.part("tool_carrier")
    tool_carrier.visual(
        Cylinder(radius=0.056, length=0.124),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_anodized",
        name="elbow_cartridge",
    )
    for y, name in ((0.050, "cartridge_ring_pos"), (-0.050, "cartridge_ring_neg")):
        tool_carrier.visual(
            Cylinder(radius=0.064, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="machined_zinc",
            name=name,
        )
    tool_carrier.visual(
        mesh_from_cadquery(
            _tapered_box_x(
                length=0.205,
                root_y=0.052,
                root_z=0.052,
                tip_y=0.040,
                tip_z=0.038,
                x_start=0.040,
            ),
            "carrier_taper",
            tolerance=0.0008,
        ),
        material="satin_aluminum",
        name="carrier_beam",
    )
    tool_carrier.visual(
        Cylinder(radius=0.029, length=0.070),
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_zinc",
        name="tool_socket",
    )
    for y, name in ((0.033, "clamp_screw_pos"), (-0.033, "clamp_screw_neg")):
        tool_carrier.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.270, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="dark_anodized",
            name=name,
        )
    tool_carrier.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.311, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="rubber_black",
        name="tool_bore",
    )
    tool_carrier.inertial = Inertial.from_geometry(
        Box((0.315, 0.096, 0.075)),
        mass=0.9,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=base_yoke,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=SHOULDER_LIMITS[0], upper=SHOULDER_LIMITS[1], effort=120.0, velocity=1.2),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=tool_carrier,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=ELBOW_LIMITS[0], upper=ELBOW_LIMITS[1], effort=80.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shoulder = object_model.get_articulation("shoulder_hinge")
    elbow = object_model.get_articulation("elbow_hinge")
    base = object_model.get_part("base_yoke")
    upper = object_model.get_part("upper_link")
    carrier = object_model.get_part("tool_carrier")

    ctx.check(
        "two serial revolute joints",
        len(object_model.articulations) == 2
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and shoulder.parent == "base_yoke"
        and shoulder.child == "upper_link"
        and elbow.parent == "upper_link"
        and elbow.child == "tool_carrier",
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "hinge axes are supported and parallel",
        tuple(round(v, 4) for v in shoulder.axis) == (0.0, -1.0, 0.0)
        and tuple(round(v, 4) for v in elbow.axis) == (0.0, -1.0, 0.0),
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )

    for q, label in ((SHOULDER_LIMITS[0], "low"), (0.0, "level"), (SHOULDER_LIMITS[1], "high")):
        with ctx.pose({shoulder: q}):
            ctx.expect_gap(
                base,
                upper,
                axis="y",
                positive_elem="shoulder_plate_pos",
                negative_elem="shoulder_hub",
                max_gap=0.001,
                max_penetration=0.00001,
                name=f"shoulder hub is seated at positive yoke plate in {label} pose",
            )
            ctx.expect_gap(
                upper,
                base,
                axis="y",
                positive_elem="shoulder_hub",
                negative_elem="shoulder_plate_neg",
                max_gap=0.001,
                max_penetration=0.00001,
                name=f"shoulder hub is seated at negative yoke plate in {label} pose",
            )

    for q, label in ((ELBOW_LIMITS[0], "down"), (0.0, "straight"), (ELBOW_LIMITS[1], "raised")):
        with ctx.pose({elbow: q}):
            ctx.expect_gap(
                upper,
                carrier,
                axis="y",
                positive_elem="elbow_plate_pos",
                negative_elem="elbow_cartridge",
                max_gap=0.001,
                max_penetration=0.00001,
                name=f"elbow cartridge is seated at positive side plate in {label} pose",
            )
            ctx.expect_gap(
                carrier,
                upper,
                axis="y",
                positive_elem="elbow_cartridge",
                negative_elem="elbow_plate_neg",
                max_gap=0.001,
                max_penetration=0.00001,
                name=f"elbow cartridge is seated at negative side plate in {label} pose",
            )

    rest_socket = ctx.part_element_world_aabb(carrier, elem="tool_socket")
    with ctx.pose({shoulder: SHOULDER_LIMITS[1], elbow: ELBOW_LIMITS[1]}):
        raised_socket = ctx.part_element_world_aabb(carrier, elem="tool_socket")
    ctx.check(
        "positive joint motion raises the nose",
        rest_socket is not None
        and raised_socket is not None
        and raised_socket[1][2] > rest_socket[1][2] + 0.20,
        details=f"rest_socket={rest_socket}, raised_socket={raised_socket}",
    )

    return ctx.report()


object_model = build_object_model()
