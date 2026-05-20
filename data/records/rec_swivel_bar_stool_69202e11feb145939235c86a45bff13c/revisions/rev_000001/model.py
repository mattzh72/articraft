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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.15, 0.16, 0.17, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.18, 0.10, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base_shell = _mesh(
        "base_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.150, 0.0),
                (0.205, 0.004),
                (0.220, 0.015),
                (0.188, 0.028),
                (0.060, 0.028),
                (0.0, 0.028),
            ],
            segments=72,
        ),
    )
    pedestal_skirt = _mesh(
        "pedestal_skirt",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.105, 0.0),
                (0.098, 0.025),
                (0.076, 0.060),
                (0.058, 0.078),
                (0.0, 0.078),
            ],
            segments=64,
        ),
    )
    foot_ring_mesh = _mesh(
        "foot_ring",
        TorusGeometry(radius=0.175, tube=0.011, radial_segments=16, tubular_segments=64),
    )
    seat_cushion_mesh = _mesh(
        "seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.050, 0.0),
                (0.120, 0.006),
                (0.175, 0.018),
                (0.198, 0.040),
                (0.194, 0.058),
                (0.162, 0.071),
                (0.082, 0.078),
                (0.0, 0.078),
            ],
            segments=72,
        ),
    )

    base_support = model.part("base_support")
    base_support.visual(base_shell, material=dark_frame, name="base_shell")
    base_support.visual(
        Cylinder(radius=0.155, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="floor_pad",
    )
    base_support.visual(
        pedestal_skirt,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=chrome,
        name="pedestal_skirt",
    )
    base_support.visual(
        Cylinder(radius=0.040, length=0.485),
        origin=Origin(xyz=(0.0, 0.0, 0.3205)),
        material=satin_steel,
        name="main_column",
    )
    base_support.visual(
        Cylinder(radius=0.070, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.601)),
        material=chrome,
        name="bearing_housing",
    )
    base_support.visual(
        Cylinder(radius=0.075, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=dark_frame,
        name="bearing_cap",
    )
    base_support.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=chrome,
        name="foot_ring",
    )
    for name, xyz, size in [
        ("ring_spoke_pos_x", (0.103, 0.0, 0.310), (0.138, 0.018, 0.016)),
        ("ring_spoke_neg_x", (-0.103, 0.0, 0.310), (0.138, 0.018, 0.016)),
        ("ring_spoke_pos_y", (0.0, 0.103, 0.310), (0.018, 0.138, 0.016)),
        ("ring_spoke_neg_y", (0.0, -0.103, 0.310), (0.018, 0.138, 0.016)),
    ]:
        base_support.visual(Box(size), origin=Origin(xyz=xyz), material=chrome, name=name)
    base_support.inertial = Inertial.from_geometry(
        Cylinder(radius=0.220, length=0.648),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.324)),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        Cylinder(radius=0.030, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark_frame,
        name="swivel_spindle",
    )
    seat_assembly.visual(
        Cylinder(radius=0.090, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=dark_frame,
        name="swivel_plate",
    )
    for name, xyz, size in [
        ("seat_rib_front", (0.0, 0.070, 0.091), (0.028, 0.140, 0.018)),
        ("seat_rib_back", (0.0, -0.070, 0.091), (0.028, 0.140, 0.018)),
        ("seat_rib_right", (0.070, 0.0, 0.091), (0.140, 0.028, 0.018)),
        ("seat_rib_left", (-0.070, 0.0, 0.091), (0.140, 0.028, 0.018)),
    ]:
        seat_assembly.visual(Box(size), origin=Origin(xyz=xyz), material=dark_frame, name=name)
    seat_assembly.visual(
        Cylinder(radius=0.175, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=dark_frame,
        name="seat_pan",
    )
    seat_assembly.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat_assembly.visual(
        Cylinder(radius=0.170, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=chrome,
        name="seat_trim_ring",
    )
    seat_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.198, length=0.196),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_support,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.644)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    seat_assembly = object_model.get_part("seat_assembly")
    seat_swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_origin_distance(
        base_support,
        seat_assembly,
        axes="xy",
        max_dist=0.001,
        name="seat is centered over the pedestal",
    )
    ctx.expect_gap(
        seat_assembly,
        base_support,
        axis="z",
        max_gap=0.005,
        max_penetration=0.0,
        positive_elem="swivel_spindle",
        negative_elem="bearing_cap",
        name="swivel spindle sits directly above the bearing cap",
    )

    column_aabb = ctx.part_element_world_aabb(base_support, elem="main_column")
    cushion_aabb = ctx.part_element_world_aabb(seat_assembly, elem="seat_cushion")
    if column_aabb is not None and cushion_aabb is not None:
        column_diameter = max(
            column_aabb[1][0] - column_aabb[0][0],
            column_aabb[1][1] - column_aabb[0][1],
        )
        cushion_diameter = max(
            cushion_aabb[1][0] - cushion_aabb[0][0],
            cushion_aabb[1][1] - cushion_aabb[0][1],
        )
        ctx.check(
            "seat reads much larger than the fixed pedestal",
            cushion_diameter >= column_diameter * 4.0,
            details=(
                f"cushion_diameter={cushion_diameter:.4f}, "
                f"column_diameter={column_diameter:.4f}"
            ),
        )

    with ctx.pose({seat_swivel: math.pi / 2.0}):
        ctx.expect_origin_distance(
            base_support,
            seat_assembly,
            axes="xy",
            max_dist=0.001,
            name="seat remains centered when swiveled",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
