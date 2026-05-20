from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


VANE_COUNT = 7
VANE_SPACING = 0.13
VANE_SPAN = 0.86
SHAFT_LENGTH = 1.12
SHAFT_RADIUS = 0.013
VANE_CHORD = 0.14
VANE_THICKNESS = 0.020
PIVOT_LIMIT = 0.45


def _vane_z(index: int) -> float:
    return (index - (VANE_COUNT - 1) / 2.0) * VANE_SPACING


def _cambered_blade_geometry() -> MeshGeometry:
    """Closed, spanwise-extruded hard-surface vane with a subtle foil section."""

    c = VANE_CHORD
    t = VANE_THICKNESS
    # Profile coordinates are in local (Y, Z), running around the foil section.
    profile_yz = [
        (-0.50 * c, 0.000 * t),
        (-0.42 * c, 0.20 * t),
        (-0.22 * c, 0.43 * t),
        (0.00 * c, 0.50 * t),
        (0.24 * c, 0.36 * t),
        (0.45 * c, 0.14 * t),
        (0.50 * c, 0.000 * t),
        (0.43 * c, -0.16 * t),
        (0.16 * c, -0.34 * t),
        (-0.15 * c, -0.40 * t),
        (-0.39 * c, -0.22 * t),
    ]

    geom = MeshGeometry()
    x_faces = (-VANE_SPAN / 2.0, VANE_SPAN / 2.0)
    rings: list[list[int]] = []
    for x in x_faces:
        ring = []
        for y, z in profile_yz:
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    n = len(profile_yz)
    for j in range(n):
        a = rings[0][j]
        b = rings[0][(j + 1) % n]
        c0 = rings[1][(j + 1) % n]
        d = rings[1][j]
        geom.add_face(a, b, c0)
        geom.add_face(a, c0, d)

    for ring, x in ((rings[0], x_faces[0]), (rings[1], x_faces[1])):
        center = geom.add_vertex(x, 0.0, 0.0)
        for j in range(n):
            if x < 0:
                geom.add_face(center, ring[(j + 1) % n], ring[j])
            else:
                geom.add_face(center, ring[j], ring[(j + 1) % n])

    return geom


def _axis_x() -> Origin:
    return Origin(rpy=(0.0, pi / 2.0, 0.0))


def _axis_y() -> Origin:
    return Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_pivot_vane_array")

    steel = model.material("bead_blasted_steel", rgba=(0.50, 0.52, 0.52, 1.0))
    vane_stock = model.material("satin_vane_stock", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("black_oxide_hardware", rgba=(0.04, 0.045, 0.05, 1.0))
    bronze = model.material("bronze_bearing_liners", rgba=(0.65, 0.46, 0.22, 1.0))
    cover_mat = model.material("dark_access_covers", rgba=(0.18, 0.19, 0.20, 1.0))

    blade_mesh = mesh_from_geometry(_cambered_blade_geometry(), "cambered_vane_blade")

    frame = model.part("frame")
    frame.visual(Box((0.07, 0.10, 1.08)), origin=Origin(xyz=(-0.60, -0.12, 0.0)), material=steel, name="side_rail_0")
    frame.visual(Box((0.07, 0.10, 1.08)), origin=Origin(xyz=(0.60, -0.12, 0.0)), material=steel, name="side_rail_1")
    frame.visual(Box((1.27, 0.10, 0.07)), origin=Origin(xyz=(0.0, -0.12, 0.535)), material=steel, name="top_crossbar")
    frame.visual(Box((1.27, 0.10, 0.07)), origin=Origin(xyz=(0.0, -0.12, -0.535)), material=steel, name="bottom_crossbar")
    frame.visual(Box((1.32, 0.16, 0.04)), origin=Origin(xyz=(0.0, -0.12, -0.585)), material=steel, name="base_flange")
    frame.visual(Cylinder(radius=0.011, length=1.18), origin=Origin(xyz=(0.0, -0.070, 0.465), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="spacer_top")
    frame.visual(Cylinder(radius=0.011, length=1.18), origin=Origin(xyz=(0.0, -0.070, -0.465), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="spacer_bottom")

    # Right-side linkage/stops rail and its hard spacers back to the main frame.
    frame.visual(Box((0.055, 0.035, 0.99)), origin=Origin(xyz=(0.47, 0.16, 0.0)), material=steel, name="linkage_rail")
    frame.visual(Box((0.045, 0.31, 0.035)), origin=Origin(xyz=(0.47, 0.015, 0.495)), material=steel, name="link_spacer_top")
    frame.visual(Box((0.045, 0.31, 0.035)), origin=Origin(xyz=(0.47, 0.015, -0.495)), material=steel, name="link_spacer_bottom")

    for i in range(VANE_COUNT):
        z = _vane_z(i)
        for side_index, sign in enumerate((-1.0, 1.0)):
            frame.visual(
                Box((0.052, 0.055, 0.070)),
                origin=Origin(xyz=(sign * 0.545, -0.055, z)),
                material=steel,
                name=f"bearing_foot_{i}_{side_index}",
            )
            frame.visual(
                Box((0.068, 0.058, 0.014)),
                origin=Origin(xyz=(sign * 0.513, 0.0, z + 0.028)),
                material=steel,
                name=f"bearing_upper_{i}_{side_index}",
            )
            frame.visual(
                Box((0.068, 0.058, 0.014)),
                origin=Origin(xyz=(sign * 0.513, 0.0, z - 0.028)),
                material=steel,
                name=f"bearing_lower_{i}_{side_index}",
            )
            frame.visual(
                Box((0.072, 0.020, 0.008)),
                origin=Origin(xyz=(sign * 0.513, 0.0, z + 0.017)),
                material=bronze,
                name=f"liner_upper_{i}_{side_index}",
            )
            frame.visual(
                Box((0.072, 0.020, 0.008)),
                origin=Origin(xyz=(sign * 0.513, 0.0, z - 0.017)),
                material=bronze,
                name=f"liner_lower_{i}_{side_index}",
            )
            frame.visual(
                Cylinder(radius=0.006, length=0.010),
                origin=Origin(xyz=(sign * 0.513, 0.031, z + 0.028), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=dark,
                name=f"cap_screw_upper_{i}_{side_index}",
            )
            frame.visual(
                Cylinder(radius=0.006, length=0.010),
                origin=Origin(xyz=(sign * 0.513, 0.031, z - 0.028), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=dark,
                name=f"cap_screw_lower_{i}_{side_index}",
            )

        frame.visual(
            Box((0.058, 0.032, 0.014)),
            origin=Origin(xyz=(0.47, 0.128, z + 0.075)),
            material=dark,
            name=("stop_upper_3" if i == 3 else f"stop_upper_{i}"),
        )
        frame.visual(
            Box((0.058, 0.032, 0.014)),
            origin=Origin(xyz=(0.47, 0.128, z - 0.075)),
            material=dark,
            name=("stop_lower_3" if i == 3 else f"stop_lower_{i}"),
        )

    for i in range(VANE_COUNT):
        vane = model.part(f"vane_{i}")
        vane.visual(blade_mesh, material=vane_stock, name="blade_skin")
        vane.visual(Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH), origin=_axis_x(), material=dark, name="shaft")
        vane.visual(Cylinder(radius=0.022, length=0.034), origin=Origin(xyz=(-0.445, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="collar_0")
        vane.visual(Cylinder(radius=0.022, length=0.034), origin=Origin(xyz=(0.445, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="collar_1")
        vane.visual(Box((0.024, VANE_CHORD * 0.76, 0.012)), origin=Origin(xyz=(-0.25, 0.0, -0.004)), material=steel, name="stiffener_0")
        vane.visual(Box((0.024, VANE_CHORD * 0.76, 0.012)), origin=Origin(xyz=(0.0, 0.0, -0.004)), material=steel, name="stiffener_1")
        vane.visual(Box((0.024, VANE_CHORD * 0.76, 0.012)), origin=Origin(xyz=(0.25, 0.0, -0.004)), material=steel, name="stiffener_2")
        vane.visual(Box((0.035, 0.115, 0.012)), origin=Origin(xyz=(0.455, 0.065, 0.0)), material=dark, name="link_stub")
        vane.visual(Cylinder(radius=0.009, length=0.045), origin=Origin(xyz=(0.455, 0.126, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="link_pin")
        model.articulation(
            f"pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, _vane_z(i))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=9.0, velocity=1.6, lower=-PIVOT_LIMIT, upper=PIVOT_LIMIT),
            motion_properties=MotionProperties(damping=0.03, friction=0.02),
        )

    for cover_index, sign in enumerate((-1.0, 1.0)):
        cover = model.part(f"access_cover_{cover_index}")
        cover.visual(Box((0.012, 0.16, 0.88)), material=cover_mat, name="cover_plate")
        for j, z in enumerate((-0.34, 0.0, 0.34)):
            for k, y in enumerate((-0.055, 0.055)):
                cover.visual(
                    Cylinder(radius=0.008, length=0.010),
                    origin=Origin(xyz=(sign * 0.008, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                    material=dark,
                    name=f"cover_screw_{j}_{k}",
                )
        model.articulation(
            f"cover_mount_{cover_index}",
            ArticulationType.FIXED,
            parent=frame,
            child=cover,
            origin=Origin(xyz=(sign * 0.641, -0.12, 0.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")

    pivot_names = [f"pivot_{i}" for i in range(VANE_COUNT)]
    vane_names = [f"vane_{i}" for i in range(VANE_COUNT)]
    ctx.check(
        "every vane has an independent revolute pivot",
        all(object_model.get_articulation(name).child == vane for name, vane in zip(pivot_names, vane_names)),
        details=f"pivots={pivot_names}, vanes={vane_names}",
    )
    for name in pivot_names:
        joint = object_model.get_articulation(name)
        limits = joint.motion_limits
        ctx.check(
            f"{name} has symmetric stop-limited travel",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower == -PIVOT_LIMIT
            and limits.upper == PIVOT_LIMIT,
            details=f"joint={joint}, limits={limits}",
        )

    for i in range(VANE_COUNT):
        vane = object_model.get_part(f"vane_{i}")
        ctx.expect_overlap(
            vane,
            frame,
            axes="x",
            min_overlap=0.055,
            elem_a="shaft",
            elem_b=f"bearing_upper_{i}_0",
            name=f"vane_{i} left journal spans bearing",
        )
        ctx.expect_overlap(
            vane,
            frame,
            axes="x",
            min_overlap=0.055,
            elem_a="shaft",
            elem_b=f"bearing_upper_{i}_1",
            name=f"vane_{i} right journal spans bearing",
        )
        ctx.expect_gap(
            frame,
            vane,
            axis="z",
            max_gap=0.001,
            max_penetration=0.000001,
            positive_elem=f"liner_upper_{i}_1",
            negative_elem="shaft",
            name=f"vane_{i} upper bearing clearance",
        )
        ctx.expect_gap(
            vane,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.000001,
            positive_elem="shaft",
            negative_elem=f"liner_lower_{i}_1",
            name=f"vane_{i} lower bearing clearance",
        )

    ctx.expect_contact(
        object_model.get_part("access_cover_0"),
        frame,
        elem_a="cover_plate",
        elem_b="side_rail_0",
        name="left access cover seats on rail",
    )
    ctx.expect_contact(
        object_model.get_part("access_cover_1"),
        frame,
        elem_a="cover_plate",
        elem_b="side_rail_1",
        name="right access cover seats on rail",
    )

    middle_pivot = object_model.get_articulation("pivot_3")
    middle_vane = object_model.get_part("vane_3")
    with ctx.pose({middle_pivot: PIVOT_LIMIT}):
        ctx.expect_gap(
            frame,
            middle_vane,
            axis="z",
            min_gap=0.001,
            max_gap=0.015,
            positive_elem="stop_upper_3",
            negative_elem="link_pin",
            name="upper stop tab is close at positive limit",
        )
    with ctx.pose({middle_pivot: -PIVOT_LIMIT}):
        ctx.expect_gap(
            middle_vane,
            frame,
            axis="z",
            min_gap=0.001,
            max_gap=0.015,
            positive_elem="link_pin",
            negative_elem="stop_lower_3",
            name="lower stop tab is close at negative limit",
        )

    return ctx.report()


object_model = build_object_model()
