from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube_mesh(name: str, points, *, radius: float, samples: int = 10):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _belt_seat_mesh(name: str, *, width: float, depth: float, thickness: float):
    """A single connected, slightly sagged rubber belt seat mesh."""
    geom = MeshGeometry()
    nx = 10
    ny = 8
    top = []
    bottom = []
    for ix in range(nx + 1):
        row_top = []
        row_bottom = []
        x = -width / 2.0 + width * ix / nx
        for iy in range(ny + 1):
            y = -depth / 2.0 + depth * iy / ny
            # Playground belt seats curve upward at the front/back lips and sag
            # gently at the center where the rider sits.
            edge_fraction = abs(y) / (depth / 2.0)
            z_mid = 0.040 * (edge_fraction**2) - 0.018
            row_top.append(geom.add_vertex(x, y, z_mid + thickness / 2.0))
            row_bottom.append(geom.add_vertex(x, y, z_mid - thickness / 2.0))
        top.append(row_top)
        bottom.append(row_bottom)

    for ix in range(nx):
        for iy in range(ny):
            a = top[ix][iy]
            b = top[ix + 1][iy]
            c = top[ix + 1][iy + 1]
            d = top[ix][iy + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            a = bottom[ix][iy]
            b = bottom[ix][iy + 1]
            c = bottom[ix + 1][iy + 1]
            d = bottom[ix + 1][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    for ix in range(nx):
        # Front edge.
        a = top[ix][0]
        b = bottom[ix][0]
        c = bottom[ix + 1][0]
        d = top[ix + 1][0]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
        # Rear edge.
        a = top[ix][ny]
        b = top[ix + 1][ny]
        c = bottom[ix + 1][ny]
        d = bottom[ix][ny]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for iy in range(ny):
        # Left edge.
        a = top[0][iy]
        b = top[0][iy + 1]
        c = bottom[0][iy + 1]
        d = bottom[0][iy]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
        # Right edge.
        a = top[nx][iy]
        b = bottom[nx][iy]
        c = bottom[nx][iy + 1]
        d = top[nx][iy + 1]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_seat_a_frame_swing")

    galvanized = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.73, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.025, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    red_caps = model.material("red_plastic_caps", rgba=(0.80, 0.05, 0.04, 1.0))
    dark_bolts = model.material("dark_bolts", rgba=(0.08, 0.08, 0.08, 1.0))

    crossbeam_z = 2.25
    pivot_z = 2.08
    pivot_x = 0.35
    hanger_length = 1.25
    seat_width = 0.70

    support_frame = model.part("support_frame")
    support_frame.visual(
        Cylinder(radius=0.055, length=2.50),
        origin=Origin(xyz=(0.0, 0.0, crossbeam_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="top_crossbeam",
    )
    support_frame.visual(
        _tube_mesh(
            "side_frame_0",
            [(-1.15, -0.92, 0.055), (-1.15, 0.0, 2.225), (-1.15, 0.92, 0.055)],
            radius=0.043,
            samples=14,
        ),
        material=galvanized,
        name="side_frame_0",
    )
    support_frame.visual(
        _tube_mesh(
            "side_frame_1",
            [(1.15, -0.92, 0.055), (1.15, 0.0, 2.225), (1.15, 0.92, 0.055)],
            radius=0.043,
            samples=14,
        ),
        material=galvanized,
        name="side_frame_1",
    )
    for y, name in [(-0.92, "front_ground_tube"), (0.92, "rear_ground_tube")]:
        support_frame.visual(
            Cylinder(radius=0.035, length=2.34),
            origin=Origin(xyz=(0.0, y, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=name,
        )
    for x, name in [(-1.15, "side_brace_0"), (1.15, "side_brace_1")]:
        support_frame.visual(
            _tube_mesh(
                name,
                [(x, -0.61, 0.78), (x, 0.61, 0.78)],
                radius=0.025,
                samples=2,
            ),
            material=galvanized,
            name=name,
        )
    for x, y, name in [
        (-1.15, -0.61, "brace_collar_0"),
        (-1.15, 0.61, "brace_collar_1"),
        (1.15, -0.61, "brace_collar_2"),
        (1.15, 0.61, "brace_collar_3"),
    ]:
        support_frame.visual(
            Cylinder(radius=0.062, length=0.095),
            origin=Origin(xyz=(x, y, 0.78), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=name,
        )
    for x, y, name in [
        (-1.15, -0.92, "foot_pad_0"),
        (-1.15, 0.92, "foot_pad_1"),
        (1.15, -0.92, "foot_pad_2"),
        (1.15, 0.92, "foot_pad_3"),
    ]:
        support_frame.visual(
            Box((0.22, 0.13, 0.035)),
            origin=Origin(xyz=(x, y, 0.018)),
            material=red_caps,
            name=name,
        )

    for index, x, collar_name, tab_name, plate_a, plate_b in [
        (0, -pivot_x, "beam_collar_0", "hanger_tab_0", "clevis_plate_0a", "clevis_plate_0b"),
        (1, pivot_x, "beam_collar_1", "hanger_tab_1", "clevis_plate_1a", "clevis_plate_1b"),
    ]:
        support_frame.visual(
            Cylinder(radius=0.072, length=0.115),
            origin=Origin(xyz=(x, 0.0, crossbeam_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=safety_yellow,
            name=collar_name,
        )
        support_frame.visual(
            Box((0.050, 0.075, 0.080)),
            origin=Origin(xyz=(x, 0.0, 2.160)),
            material=safety_yellow,
            name=tab_name,
        )
        for offset, plate_name in [(-0.058, plate_a), (0.058, plate_b)]:
            support_frame.visual(
                Box((0.018, 0.105, 0.205)),
                origin=Origin(xyz=(x + offset, 0.0, 2.085)),
                material=safety_yellow,
                name=plate_name,
            )
    support_frame.visual(
        Cylinder(radius=0.014, length=0.150),
        origin=Origin(xyz=(-pivot_x, 0.0, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_bolts,
        name="pivot_pin_0",
    )
    support_frame.visual(
        Cylinder(radius=0.014, length=0.150),
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_bolts,
        name="pivot_pin_1",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((2.55, 1.95, 2.30)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
    )

    hanger_0 = model.part("hanger_0")
    hanger_0.visual(
        Cylinder(radius=0.026, length=0.070),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="top_bushing",
    )
    hanger_0.visual(
        Cylinder(radius=0.017, length=hanger_length - 0.025),
        origin=Origin(xyz=(0.0, 0.0, -(hanger_length + 0.025) / 2.0)),
        material=galvanized,
        name="hanger_rod",
    )

    hanger_1 = model.part("hanger_1")
    hanger_1.visual(
        Cylinder(radius=0.026, length=0.070),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="top_bushing",
    )
    hanger_1.visual(
        Cylinder(radius=0.017, length=hanger_length - 0.025),
        origin=Origin(xyz=(0.0, 0.0, -(hanger_length + 0.025) / 2.0)),
        material=galvanized,
        name="hanger_rod",
    )

    belt_seat = model.part("belt_seat")
    belt_seat.visual(
        _belt_seat_mesh("belt_seat_panel", width=0.78, depth=0.38, thickness=0.040),
        origin=Origin(xyz=(seat_width / 2.0, 0.0, -0.085)),
        material=dark_rubber,
        name="belt_panel",
    )
    belt_seat.visual(
        Box((0.045, 0.120, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=safety_yellow,
        name="seat_lug_0",
    )
    belt_seat.visual(
        Cylinder(radius=0.018, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_bolts,
        name="lug_pin_0",
    )
    belt_seat.visual(
        Box((0.035, 0.30, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=safety_yellow,
        name="end_clamp_0",
    )
    belt_seat.visual(
        Box((0.045, 0.120, 0.080)),
        origin=Origin(xyz=(seat_width, 0.0, -0.040)),
        material=safety_yellow,
        name="seat_lug_1",
    )
    belt_seat.visual(
        Cylinder(radius=0.018, length=0.135),
        origin=Origin(xyz=(seat_width, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_bolts,
        name="lug_pin_1",
    )
    belt_seat.visual(
        Box((0.035, 0.30, 0.030)),
        origin=Origin(xyz=(seat_width, 0.0, -0.078)),
        material=safety_yellow,
        name="end_clamp_1",
    )
    belt_seat.visual(
        Box((0.64, 0.035, 0.018)),
        origin=Origin(xyz=(seat_width / 2.0, -0.205, -0.075)),
        material=dark_rubber,
        name="front_lip",
    )
    belt_seat.visual(
        Box((0.64, 0.035, 0.018)),
        origin=Origin(xyz=(seat_width / 2.0, 0.205, -0.075)),
        material=dark_rubber,
        name="rear_lip",
    )
    belt_seat.inertial = Inertial.from_geometry(
        Box((0.80, 0.42, 0.12)),
        mass=4.0,
        origin=Origin(xyz=(seat_width / 2.0, 0.0, -0.065)),
    )

    left_joint = model.articulation(
        "top_pivot_0",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=hanger_0,
        origin=Origin(xyz=(-pivot_x, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.8, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "top_pivot_1",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=hanger_1,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.8, lower=-0.75, upper=0.75),
        mimic=Mimic(joint=left_joint.name),
    )
    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent=hanger_0,
        child=belt_seat,
        origin=Origin(xyz=(0.0, 0.0, -hanger_length)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    hanger_0 = object_model.get_part("hanger_0")
    hanger_1 = object_model.get_part("hanger_1")
    seat = object_model.get_part("belt_seat")
    pivot_0 = object_model.get_articulation("top_pivot_0")

    ctx.allow_overlap(
        support,
        hanger_0,
        elem_a="pivot_pin_0",
        elem_b="top_bushing",
        reason="The revolute bushing is intentionally captured around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        support,
        hanger_1,
        elem_a="pivot_pin_1",
        elem_b="top_bushing",
        reason="The revolute bushing is intentionally captured around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        seat,
        hanger_0,
        elem_a="lug_pin_0",
        elem_b="hanger_rod",
        reason="The bottom lug pin passes through the rigid hanger eye to retain the belt seat.",
    )
    ctx.allow_overlap(
        seat,
        hanger_1,
        elem_a="lug_pin_1",
        elem_b="hanger_rod",
        reason="The bottom lug pin passes through the rigid hanger eye to retain the belt seat.",
    )

    ctx.check(
        "two revolute top pivots",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE
            for name in ("top_pivot_0", "top_pivot_1")
        ),
        details="The two rigid hangers should each be carried by a top revolute joint.",
    )
    ctx.expect_contact(
        hanger_0,
        seat,
        elem_a="hanger_rod",
        elem_b="seat_lug_0",
        contact_tol=0.003,
        name="first hanger reaches its seat lug",
    )
    ctx.expect_overlap(
        support,
        hanger_0,
        axes="xyz",
        elem_a="pivot_pin_0",
        elem_b="top_bushing",
        min_overlap=0.020,
        name="first top pin is captured in bushing",
    )
    ctx.expect_overlap(
        support,
        hanger_1,
        axes="xyz",
        elem_a="pivot_pin_1",
        elem_b="top_bushing",
        min_overlap=0.020,
        name="second top pin is captured in bushing",
    )
    ctx.expect_overlap(
        seat,
        hanger_0,
        axes="xyz",
        elem_a="lug_pin_0",
        elem_b="hanger_rod",
        min_overlap=0.010,
        name="first bottom pin captures hanger",
    )
    ctx.expect_overlap(
        seat,
        hanger_1,
        axes="xyz",
        elem_a="lug_pin_1",
        elem_b="hanger_rod",
        min_overlap=0.010,
        name="second bottom pin captures hanger",
    )
    ctx.expect_contact(
        hanger_1,
        seat,
        elem_a="hanger_rod",
        elem_b="seat_lug_1",
        contact_tol=0.003,
        name="second hanger reaches its seat lug",
    )
    ctx.expect_gap(
        support,
        seat,
        axis="z",
        min_gap=1.0,
        name="crossbeam remains above suspended seat",
        positive_elem="top_crossbeam",
        negative_elem="belt_panel",
    )

    rest_pos = ctx.part_world_position(seat)
    with ctx.pose({pivot_0: 0.45}):
        swung_pos = ctx.part_world_position(seat)
        ctx.expect_contact(
            hanger_1,
            seat,
            elem_a="hanger_rod",
            elem_b="seat_lug_1",
            contact_tol=0.005,
            name="mimicked hanger stays attached while swinging",
        )

    ctx.check(
        "seat swings in the vertical plane",
        rest_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_pos[1] + 0.45
        and swung_pos[2] > rest_pos[2] + 0.10
        and abs(swung_pos[0] - rest_pos[0]) < 0.005,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
