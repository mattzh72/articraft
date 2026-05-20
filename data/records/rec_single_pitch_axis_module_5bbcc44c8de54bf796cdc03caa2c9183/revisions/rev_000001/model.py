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


def _cheek_plate(
    length_x: float,
    thickness_y: float,
    height_z: float,
    bore_radius: float,
    bore_z: float,
) -> cq.Workplane:
    """One side cheek with a real through-bore for the pitch shaft."""
    body = cq.Workplane("XY").box(length_x, thickness_y, height_z).translate(
        (0.0, 0.0, height_z / 2.0)
    )
    bore = (
        cq.Workplane("XZ")
        .center(0.0, bore_z)
        .circle(bore_radius)
        .extrude(thickness_y * 3.0, both=True)
    )
    return body.cut(bore)


def _bearing_ring(outer_radius: float, inner_radius: float, thickness_y: float) -> cq.Workplane:
    """A washer/bearing ring with a visible center clearance hole."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness_y, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tilting_cradle")

    dark_oxide = model.material("dark_oxide", rgba=(0.06, 0.065, 0.07, 1.0))
    blue_grey = model.material("blue_grey_powdercoat", rgba=(0.18, 0.25, 0.32, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    screw_black = model.material("black_socket_heads", rgba=(0.02, 0.02, 0.022, 1.0))

    base_len = 0.38
    base_w = 0.33
    base_h = 0.055
    cheek_len = 0.34
    cheek_t = 0.030
    cheek_h = 0.215
    inner_gap = 0.240
    cheek_y = inner_gap / 2.0 + cheek_t / 2.0
    shaft_z = 0.175
    bore_z_local = shaft_z - base_h

    base = model.part("base")
    base.visual(
        Box((base_len, base_w, base_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
        material=dark_oxide,
        name="base_block",
    )

    cheek_mesh = mesh_from_cadquery(
        _cheek_plate(cheek_len, cheek_t, cheek_h, bore_radius=0.018, bore_z=bore_z_local),
        "side_cheek_bored",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    for cheek_name, y in (("cheek_0", cheek_y), ("cheek_1", -cheek_y)):
        base.visual(
            cheek_mesh,
            origin=Origin(xyz=(0.0, y, base_h)),
            material=blue_grey,
            name=cheek_name,
        )

    ring_mesh = mesh_from_cadquery(
        _bearing_ring(outer_radius=0.031, inner_radius=0.014, thickness_y=0.008),
        "bearing_ring",
        tolerance=0.0005,
        angular_tolerance=0.06,
    )
    ring_y = cheek_y + cheek_t / 2.0 + 0.004
    for idx, y in enumerate((ring_y, -ring_y)):
        base.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, y, shaft_z)),
            material=satin_steel,
            name=f"bearing_ring_{idx}",
        )

    # Cap screws on the cheeks make the brackets read as mounted plates rather
    # than decorative fins.  They are slightly seated into the parent cheek.
    screw_len = 0.005
    screw_y = cheek_y + cheek_t / 2.0 + screw_len / 2.0 - 0.0005
    screw_positions = []
    for y_sign in (1.0, -1.0):
        for x in (-0.110, 0.110):
            for z in (base_h + 0.060, base_h + 0.155):
                screw_positions.append((x, y_sign * screw_y, z))
    for idx, (x, y, z) in enumerate(screw_positions):
        base.visual(
            Cylinder(radius=0.0065, length=screw_len),
            origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_black,
            name=f"cheek_screw_{idx}",
        )

    # Low stop tabs sit beside the tray sweep envelope, leaving visible pitch
    # clearance while providing credible hard-stop hardware.
    inner_face_y = inner_gap / 2.0
    tab_y = 0.010
    tab_size = (0.024, tab_y, 0.026)
    tab_z = base_h + 0.060
    for stop_name, x, y_sign in (
        ("front_stop_0", 0.170, 1.0),
        ("rear_stop_1", -0.170, 1.0),
        ("front_stop_2", 0.170, -1.0),
        ("rear_stop_3", -0.170, -1.0),
    ):
        tab_center_y = y_sign * (inner_face_y - tab_y / 2.0 + 0.0005)
        base.visual(
            Box(tab_size),
            origin=Origin(xyz=(x, tab_center_y, tab_z)),
            material=blue_grey,
            name=stop_name,
        )

    # Four top mounting bolts ground the compact base block to the imagined
    # bench or fixture beneath it.
    for idx, (x, y) in enumerate(((-0.140, -0.110), (-0.140, 0.110), (0.140, -0.110), (0.140, 0.110))):
        base.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(x, y, base_h + 0.002)),
            material=screw_black,
            name=f"base_bolt_{idx}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.245, 0.185, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=blue_grey,
        name="tray_bottom",
    )
    for idx, y in enumerate((0.095, -0.095)):
        tray.visual(
            Box((0.245, 0.010, 0.035)),
            origin=Origin(xyz=(0.0, y, -0.0325)),
            material=blue_grey,
            name=f"side_lip_{idx}",
        )
    tray.visual(
        Box((0.010, 0.200, 0.030)),
        origin=Origin(xyz=(0.1275, 0.0, -0.035)),
        material=blue_grey,
        name="front_lip",
    )
    tray.visual(
        Box((0.010, 0.200, 0.030)),
        origin=Origin(xyz=(-0.1275, 0.0, -0.035)),
        material=blue_grey,
        name="rear_lip",
    )

    for idx, (x, y) in enumerate(((-0.070, -0.055), (-0.070, 0.055), (0.070, -0.055), (0.070, 0.055))):
        tray.visual(
            Box((0.045, 0.028, 0.004)),
            origin=Origin(xyz=(x, y, -0.0455)),
            material=black_rubber,
            name=f"rubber_pad_{idx}",
        )

    tray.visual(
        Cylinder(radius=0.009, length=0.332),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_shaft",
    )
    for idx, y in enumerate((0.106, -0.106)):
        tray.visual(
            Cylinder(radius=0.025, length=0.016),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"trunnion_hub_{idx}",
        )
    for washer_name, y in (("bearing_washer_0", 0.115), ("bearing_washer_1", -0.115)):
        tray.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=washer_name,
        )
    for idx, y in enumerate((0.166, -0.166)):
        tray.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"shaft_cap_{idx}",
        )

    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    pitch = object_model.get_articulation("pitch")

    ctx.check(
        "single supported revolute stage",
        len(object_model.articulations) == 1
        and pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    for q, label in ((0.0, "level"), (-0.45, "nose_up"), (0.45, "nose_down")):
        with ctx.pose({pitch: q}):
            ctx.expect_gap(
                base,
                tray,
                axis="y",
                min_gap=0.0015,
                positive_elem="cheek_0",
                negative_elem="bearing_washer_0",
                name=f"{label} positive side bearing clearance",
            )
            ctx.expect_gap(
                tray,
                base,
                axis="y",
                min_gap=0.0015,
                positive_elem="bearing_washer_1",
                negative_elem="cheek_1",
                name=f"{label} negative side bearing clearance",
            )
            ctx.expect_gap(
                tray,
                base,
                axis="z",
                min_gap=0.018,
                positive_elem="tray_bottom",
                negative_elem="base_block",
                name=f"{label} tray clears base block",
            )
            ctx.expect_gap(
                base,
                tray,
                axis="x",
                min_gap=0.012,
                positive_elem="front_stop_0",
                negative_elem="front_lip",
                name=f"{label} front stop clear of lip",
            )
            ctx.expect_gap(
                tray,
                base,
                axis="x",
                min_gap=0.012,
                positive_elem="rear_lip",
                negative_elem="rear_stop_1",
                name=f"{label} rear stop clear of lip",
            )

    with ctx.pose({pitch: -0.45}):
        front_up = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({pitch: 0.45}):
        front_down = ctx.part_element_world_aabb(tray, elem="front_lip")
    if front_up is not None and front_down is not None:
        up_center_z = (front_up[0][2] + front_up[1][2]) / 2.0
        down_center_z = (front_down[0][2] + front_down[1][2]) / 2.0
    else:
        up_center_z = down_center_z = None
    ctx.check(
        "pitch motion visibly tilts tray",
        up_center_z is not None
        and down_center_z is not None
        and up_center_z > down_center_z + 0.08,
        details=f"front_lip_up_z={up_center_z}, front_lip_down_z={down_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
