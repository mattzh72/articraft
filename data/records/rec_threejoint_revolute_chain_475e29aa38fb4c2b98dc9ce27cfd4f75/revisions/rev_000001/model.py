from __future__ import annotations

import math

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


FORK_GAP = 0.092
PLATE_THICKNESS = 0.012
PLATE_Y = FORK_GAP / 2.0 + PLATE_THICKNESS / 2.0
PLATE_OUTER_Y = PLATE_Y + PLATE_THICKNESS / 2.0
HUB_LENGTH = 0.080


def _y_axis_origin(x: float, y: float, z: float) -> Origin:
    """Orient a default Z-axis cylinder so its length runs along world Y."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _x_axis_origin(x: float, y: float, z: float) -> Origin:
    """Orient a default Z-axis cylinder so its length runs along world X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _plate_mesh(width: float, height: float, thickness: float, bore: float, name: str):
    """Flat machined side plate: rounded outside, through bore, and four bolt holes."""
    bolt_pts = [
        (-width * 0.32, -height * 0.31),
        (-width * 0.32, height * 0.31),
        (width * 0.32, -height * 0.31),
        (width * 0.32, height * 0.31),
    ]
    plate = cq.Workplane("XY").box(width, height, thickness)
    plate = plate.edges("|Z").fillet(min(width, height) * 0.055)
    plate = plate.faces(">Z").workplane().circle(bore / 2.0).cutThruAll()
    plate = plate.faces(">Z").workplane().pushPoints(bolt_pts).circle(0.0055).cutThruAll()
    return mesh_from_cadquery(plate, name, tolerance=0.0007, angular_tolerance=0.08)


def _cover_mesh(width: float, height: float, thickness: float, name: str):
    cover = cq.Workplane("XY").box(width, height, thickness)
    cover = cover.edges("|Z").fillet(min(width, height) * 0.08)
    return mesh_from_cadquery(cover, name, tolerance=0.0007, angular_tolerance=0.08)


def _add_fork_plates(part, *, joint: str, x: float, z: float, material: str, plate_mesh, plate_names) -> None:
    """Two side plates plus exposed bearing caps and cap screws for one forked pivot."""
    for (suffix, sign), plate_name in zip((("pos", 1.0), ("neg", -1.0)), plate_names):
        part.visual(
            plate_mesh,
            origin=Origin(xyz=(x, sign * PLATE_Y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=plate_name,
        )
        cap_y = sign * (PLATE_OUTER_Y + 0.014 / 2.0 - 0.0006)
        part.visual(
            Cylinder(radius=0.052, length=0.014),
            origin=_y_axis_origin(x, cap_y, z),
            material="dark_oxide",
            name=f"{joint}_boss_{suffix}",
        )
        retainer_y = sign * (PLATE_OUTER_Y + 0.014 + 0.007 / 2.0 - 0.0011)
        part.visual(
            Cylinder(radius=0.025, length=0.007),
            origin=_y_axis_origin(x, retainer_y, z),
            material="black_fasteners",
            name=f"{joint}_cap_{suffix}",
        )
        for i, angle in enumerate((math.radians(35), math.radians(145), math.radians(215), math.radians(325))):
            part.visual(
                Cylinder(radius=0.006, length=0.005),
                origin=_y_axis_origin(
                    x + math.cos(angle) * 0.038,
                    sign * (PLATE_OUTER_Y + 0.014 + 0.005 / 2.0 - 0.0010),
                    z + math.sin(angle) * 0.038,
                ),
                material="black_fasteners",
                name=f"{joint}_screw_{suffix}_{i}",
            )


def _add_mount_bolts_z(part, positions, *, z: float, name_prefix: str) -> None:
    for i, (x, y) in enumerate(positions):
        part.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(x, y, z)),
            material="black_fasteners",
            name=f"{name_prefix}_bolt_{i}",
        )


def _add_access_cover(part, *, x: float, z: float, cover_mesh, name_prefix: str) -> None:
    part.visual(
        cover_mesh,
        origin=Origin(xyz=(x, 0.0, z)),
        material="dark_oxide",
        name=f"{name_prefix}_cover",
    )
    for i, (dx, dy) in enumerate(((-0.074, -0.018), (-0.074, 0.018), (0.074, -0.018), (0.074, 0.018))):
        part.visual(
            Cylinder(radius=0.0047, length=0.004),
            origin=Origin(xyz=(x + dx, dy, z + 0.005)),
            material="black_fasteners",
            name=f"{name_prefix}_cover_screw_{i}",
        )


def _add_thrust_washers(part, *, joint: str, radius: float) -> None:
    """Thin child-side washers touch the fork cheeks and make the pin capture legible."""
    for suffix, y in (("pos", 0.043), ("neg", -0.043)):
        part.visual(
            Cylinder(radius=radius, length=0.006),
            origin=_y_axis_origin(0.0, y, 0.0),
            material="etched_steel",
            name=f"{joint}_thrust_washer_{suffix}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_revolute_chain_study")

    model.material("bead_blast_aluminum", rgba=(0.66, 0.68, 0.66, 1.0))
    model.material("dark_oxide", rgba=(0.10, 0.11, 0.11, 1.0))
    model.material("black_fasteners", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("bronze_bushing", rgba=(0.74, 0.51, 0.24, 1.0))
    model.material("base_iron", rgba=(0.22, 0.24, 0.25, 1.0))
    model.material("etched_steel", rgba=(0.46, 0.48, 0.47, 1.0))

    large_plate_mesh = _plate_mesh(0.160, 0.140, PLATE_THICKNESS, 0.036, "large_side_plate")
    mid_plate_mesh = _plate_mesh(0.150, 0.130, PLATE_THICKNESS, 0.034, "mid_side_plate")
    small_plate_mesh = _plate_mesh(0.132, 0.116, PLATE_THICKNESS, 0.032, "small_side_plate")
    long_cover_mesh = _cover_mesh(0.180, 0.052, 0.006, "long_access_cover")
    short_cover_mesh = _cover_mesh(0.150, 0.046, 0.006, "short_access_cover")

    base = model.part("base")
    base.visual(
        Box((0.44, 0.36, 0.026)),
        origin=Origin(xyz=(-0.080, 0.0, -0.467)),
        material="base_iron",
        name="floor_plate",
    )
    for i, (x, y) in enumerate(((-0.225, -0.125), (-0.225, 0.125), (0.045, -0.125), (0.045, 0.125))):
        base.visual(
            Box((0.105, 0.070, 0.014)),
            origin=Origin(xyz=(x, y, -0.450)),
            material="etched_steel",
            name=f"mount_flange_{i}",
        )
    _add_mount_bolts_z(
        base,
        [(-0.247, -0.125), (-0.203, -0.125), (-0.247, 0.125), (-0.203, 0.125), (0.023, -0.125), (0.067, -0.125), (0.023, 0.125), (0.067, 0.125)],
        z=-0.440,
        name_prefix="floor",
    )
    base.visual(
        Box((0.086, 0.132, 0.410)),
        origin=Origin(xyz=(-0.112, 0.0, -0.242)),
        material="base_iron",
        name="upright_column",
    )
    for suffix, y in (("pos", 0.078), ("neg", -0.078)):
        base.visual(
            Box((0.182, 0.012, 0.255)),
            origin=Origin(xyz=(-0.112, 0.072 if suffix == "pos" else -0.072, -0.330)),
            material="base_iron",
            name=f"column_web_{suffix}",
        )
    base.visual(
        Box((0.068, 0.136, 0.064)),
        origin=Origin(xyz=(-0.108, 0.0, -0.040)),
        material="base_iron",
        name="shoulder_rear_bridge",
    )
    _add_fork_plates(
        base,
        joint="shoulder",
        x=0.0,
        z=0.0,
        material="base_iron",
        plate_mesh=large_plate_mesh,
        plate_names=("shoulder_plate_pos", "shoulder_plate_neg"),
    )

    upper = model.part("upper_link")
    upper.visual(
        Cylinder(radius=0.046, length=HUB_LENGTH),
        origin=_y_axis_origin(0.0, 0.0, 0.0),
        material="bronze_bushing",
        name="shoulder_hub",
    )
    _add_thrust_washers(upper, joint="shoulder", radius=0.041)
    upper.visual(
        Cylinder(radius=0.032, length=0.084),
        origin=_y_axis_origin(0.0, 0.0, 0.0),
        material="dark_oxide",
        name="shoulder_bearing_sleeve",
    )
    for suffix, y in (("pos", 0.025), ("neg", -0.025)):
        upper.visual(
            Box((0.432, 0.016, 0.032)),
            origin=Origin(xyz=(0.248, y, 0.0)),
            material="bead_blast_aluminum",
            name=f"upper_rail_{suffix}",
        )
    upper.visual(
        Box((0.354, 0.064, 0.012)),
        origin=Origin(xyz=(0.255, 0.0, 0.026)),
        material="bead_blast_aluminum",
        name="upper_top_stiffener",
    )
    upper.visual(
        Box((0.354, 0.064, 0.012)),
        origin=Origin(xyz=(0.255, 0.0, -0.026)),
        material="bead_blast_aluminum",
        name="upper_bottom_stiffener",
    )
    for i, x in enumerate((0.145, 0.255, 0.365)):
        upper.visual(
            Box((0.018, 0.070, 0.058)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="bead_blast_aluminum",
            name=f"upper_cross_rib_{i}",
        )
    _add_access_cover(upper, x=0.255, z=0.035, cover_mesh=long_cover_mesh, name_prefix="upper")
    upper.visual(
        Box((0.037, 0.116, 0.042)),
        origin=Origin(xyz=(0.476, 0.0, 0.0)),
        material="bead_blast_aluminum",
        name="elbow_fork_bridge",
    )
    _add_fork_plates(
        upper,
        joint="elbow",
        x=0.550,
        z=0.0,
        material="bead_blast_aluminum",
        plate_mesh=mid_plate_mesh,
        plate_names=("elbow_plate_pos", "elbow_plate_neg"),
    )
    for suffix, y in (("pos", 0.028), ("neg", -0.028)):
        upper.visual(
            Box((0.070, 0.010, 0.022)),
            origin=Origin(xyz=(0.460, y, 0.041)),
            material="dark_oxide",
            name=f"elbow_stop_block_{suffix}",
        )

    forearm = model.part("forearm_link")
    forearm.visual(
        Cylinder(radius=0.042, length=HUB_LENGTH),
        origin=_y_axis_origin(0.0, 0.0, 0.0),
        material="bronze_bushing",
        name="elbow_hub",
    )
    _add_thrust_washers(forearm, joint="elbow", radius=0.038)
    forearm.visual(
        Cylinder(radius=0.030, length=0.084),
        origin=_y_axis_origin(0.0, 0.0, 0.0),
        material="dark_oxide",
        name="elbow_bearing_sleeve",
    )
    for suffix, y in (("pos", 0.023), ("neg", -0.023)):
        forearm.visual(
            Box((0.310, 0.014, 0.028)),
            origin=Origin(xyz=(0.185, y, 0.0)),
            material="bead_blast_aluminum",
            name=f"forearm_rail_{suffix}",
        )
    forearm.visual(
        Box((0.252, 0.056, 0.010)),
        origin=Origin(xyz=(0.195, 0.0, 0.023)),
        material="bead_blast_aluminum",
        name="forearm_top_stiffener",
    )
    forearm.visual(
        Box((0.252, 0.056, 0.010)),
        origin=Origin(xyz=(0.195, 0.0, -0.023)),
        material="bead_blast_aluminum",
        name="forearm_bottom_stiffener",
    )
    for i, x in enumerate((0.130, 0.230, 0.315)):
        forearm.visual(
            Box((0.016, 0.062, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="bead_blast_aluminum",
            name=f"forearm_cross_rib_{i}",
        )
    _add_access_cover(forearm, x=0.205, z=0.031, cover_mesh=short_cover_mesh, name_prefix="forearm")
    forearm.visual(
        Box((0.034, 0.108, 0.038)),
        origin=Origin(xyz=(0.353, 0.0, 0.0)),
        material="bead_blast_aluminum",
        name="wrist_fork_bridge",
    )
    _add_fork_plates(
        forearm,
        joint="wrist",
        x=0.420,
        z=0.0,
        material="bead_blast_aluminum",
        plate_mesh=small_plate_mesh,
        plate_names=("wrist_plate_pos", "wrist_plate_neg"),
    )
    for suffix, y in (("pos", 0.027), ("neg", -0.027)):
        forearm.visual(
            Box((0.060, 0.010, 0.020)),
            origin=Origin(xyz=(0.340, y, 0.036)),
            material="dark_oxide",
            name=f"wrist_stop_block_{suffix}",
        )

    wrist = model.part("wrist_link")
    wrist.visual(
        Cylinder(radius=0.039, length=0.078),
        origin=_y_axis_origin(0.0, 0.0, 0.0),
        material="bronze_bushing",
        name="wrist_hub",
    )
    _add_thrust_washers(wrist, joint="wrist", radius=0.035)
    wrist.visual(
        Cylinder(radius=0.028, length=0.082),
        origin=_y_axis_origin(0.0, 0.0, 0.0),
        material="dark_oxide",
        name="wrist_bearing_sleeve",
    )
    wrist.visual(
        Box((0.104, 0.046, 0.032)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material="bead_blast_aluminum",
        name="wrist_neck",
    )
    wrist.visual(
        Box((0.020, 0.138, 0.106)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material="bead_blast_aluminum",
        name="output_flange",
    )
    wrist.visual(
        Cylinder(radius=0.044, length=0.020),
        origin=_x_axis_origin(0.145, 0.0, 0.0),
        material="dark_oxide",
        name="output_boss",
    )
    wrist.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=_x_axis_origin(0.160, 0.0, 0.0),
        material="black_fasteners",
        name="output_retainer_cap",
    )
    for i, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        wrist.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=_x_axis_origin(
                0.155,
                math.cos(angle) * 0.047,
                math.sin(angle) * 0.036,
            ),
            material="black_fasteners",
            name=f"output_flange_bolt_{i}",
        )

    model.articulation(
        "shoulder_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-1.05, upper=1.25),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.550, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "wrist_pivot",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.4, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm_link")
    wrist = object_model.get_part("wrist_link")
    shoulder = object_model.get_articulation("shoulder_pivot")
    elbow = object_model.get_articulation("elbow_pivot")
    wrist_joint = object_model.get_articulation("wrist_pivot")

    ctx.check(
        "three serial revolute pivots",
        len(object_model.articulations) == 3
        and [j.name for j in object_model.articulations] == ["shoulder_pivot", "elbow_pivot", "wrist_pivot"],
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    # Each hub is captured between fork side plates with a small real clearance,
    # not hidden broad interpenetration.
    for parent, child, joint_name, hub_name in (
        (base, upper, "shoulder", "shoulder_hub"),
        (upper, forearm, "elbow", "elbow_hub"),
        (forearm, wrist, "wrist", "wrist_hub"),
    ):
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            min_gap=0.002,
            max_gap=0.010,
            positive_elem=f"{joint_name}_plate_pos",
            negative_elem=hub_name,
            name=f"{joint_name} positive fork cheek clearance",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            min_gap=0.002,
            max_gap=0.010,
            positive_elem=hub_name,
            negative_elem=f"{joint_name}_plate_neg",
            name=f"{joint_name} negative fork cheek clearance",
        )

    def _element_center_z(part, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_elbow_z = _element_center_z(upper, "elbow_fork_bridge")
    with ctx.pose({shoulder: 0.65, elbow: 0.45, wrist_joint: -0.35}):
        raised_elbow_z = _element_center_z(upper, "elbow_fork_bridge")
        ctx.expect_gap(
            base,
            upper,
            axis="y",
            min_gap=0.001,
            positive_elem="shoulder_plate_pos",
            negative_elem="shoulder_hub",
            name="raised pose shoulder cheek still clears hub",
        )
        ctx.expect_gap(
            upper,
            forearm,
            axis="y",
            min_gap=0.001,
            positive_elem="elbow_plate_pos",
            negative_elem="elbow_hub",
            name="raised pose elbow cheek still clears hub",
        )

    ctx.check(
        "shoulder positive rotation raises distal bracket",
        rest_elbow_z is not None and raised_elbow_z is not None and raised_elbow_z > rest_elbow_z + 0.25,
        details=f"rest_elbow_z={rest_elbow_z}, raised_elbow_z={raised_elbow_z}",
    )

    return ctx.report()


object_model = build_object_model()
