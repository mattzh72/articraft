from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    support_path = [
        (0.086, 0.0, 0.562),
        (0.098, 0.0, 0.610),
        (0.108, 0.0, 0.648),
        (0.102, 0.0, 0.682),
    ]
    left_support_mesh = mesh_from_geometry(
        tube_from_spline_points(
            support_path,
            radius=0.016,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "left_side_support",
    )
    right_support_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(-x, y, z) for x, y, z in support_path],
            radius=0.016,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "right_side_support",
    )
    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.160, tube=0.012, radial_segments=18, tubular_segments=48),
        "foot_ring",
    )

    cushion_geom = CylinderGeometry(radius=0.182, height=0.038)
    cushion_geom.translate(0.0, 0.0, 0.124)
    cushion_top = DomeGeometry(radius=0.182, radial_segments=32, height_segments=10)
    cushion_top.scale(1.0, 1.0, 0.22)
    cushion_top.translate(0.0, 0.0, 0.140)
    cushion_geom.merge(cushion_top)
    seat_cushion_mesh = mesh_from_geometry(cushion_geom, "seat_cushion")

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.235, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=chrome,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.100, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=chrome,
        name="base_hub",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.535),
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        material=chrome,
        name="pedestal_post",
    )
    base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        material=chrome,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.318), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="foot_ring_spoke_x",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.318), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="foot_ring_spoke_y",
    )
    base.visual(
        Cylinder(radius=0.092, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=dark_steel,
        name="bearing_collar",
    )
    base.visual(
        left_support_mesh,
        material=chrome,
        name="left_side_support",
    )
    base.visual(
        right_support_mesh,
        material=chrome,
        name="right_side_support",
    )
    base.visual(
        Cylinder(radius=0.225, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="floor_glide",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.47, 0.47, 0.61)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.044, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark_steel,
        name="swivel_bearing",
    )
    seat.visual(
        Cylinder(radius=0.074, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=dark_steel,
        name="mount_plate",
    )
    seat.visual(
        Cylinder(radius=0.155, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=chrome,
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        material=seat_vinyl,
        name="seat_cushion",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.364, 0.364, 0.19)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        elem_a="seat_pan",
        elem_b="floor_plate",
        min_overlap=0.29,
        name="seat stays centered over the pedestal base",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="swivel_bearing",
        negative_elem="bearing_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="swivel bearing sits on the collar without sinking into it",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="seat_pan",
        negative_elem="left_side_support",
        min_gap=0.012,
        name="seat pan clears the left side support",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="seat_pan",
        negative_elem="right_side_support",
        min_gap=0.012,
        name="seat pan clears the right side support",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(seat)
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="seat_pan",
            negative_elem="left_side_support",
            min_gap=0.012,
            name="seat pan still clears the left support when turned",
        )
        ctx.expect_gap(
            seat,
            base,
            axis="z",
            positive_elem="seat_pan",
            negative_elem="right_side_support",
            min_gap=0.012,
            name="seat pan still clears the right support when turned",
        )

    ctx.check(
        "seat rotates in place about the pedestal axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
