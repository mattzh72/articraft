from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _cylinder_between(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, Origin]:
    """Return a URDF-cylinder length and origin for a segment from a to b."""
    ax, ay, az = a
    bx, by, bz = b
    vx, vy, vz = bx - ax, by - ay, bz - az
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.sqrt(vx * vx + vy * vy), vz)
    return length, Origin(xyz=((ax + bx) / 2, (ay + by) / 2, (az + bz) / 2), rpy=(0.0, pitch, yaw))


def _add_cylinder_between(part, name: str, a, b, radius: float, material) -> None:
    length, origin = _cylinder_between(a, b)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _belt_seat_geometry() -> MeshGeometry:
    """A thin rubber belt with cupped sag and slightly curled ends."""
    width_x = 0.34
    length_y = 0.82
    thickness = 0.032
    nx = 8
    ny = 18
    geom = MeshGeometry()
    top_ids: list[list[int]] = []
    bottom_ids: list[list[int]] = []

    for ix in range(nx + 1):
        row_top: list[int] = []
        row_bottom: list[int] = []
        x = -width_x / 2 + width_x * ix / nx
        x_norm = x / (width_x / 2)
        for iy in range(ny + 1):
            y = -length_y / 2 + length_y * iy / ny
            y_norm = y / (length_y / 2)
            # Center sags, front/rear ends curl upward, and side edges lift a little.
            z_center = -1.605 + 0.088 * (abs(y_norm) ** 2.2) + 0.020 * (abs(x_norm) ** 2.0)
            row_top.append(geom.add_vertex(x, y, z_center + thickness / 2))
            row_bottom.append(geom.add_vertex(x, y, z_center - thickness / 2))
        top_ids.append(row_top)
        bottom_ids.append(row_bottom)

    for ix in range(nx):
        for iy in range(ny):
            a = top_ids[ix][iy]
            b = top_ids[ix + 1][iy]
            c = top_ids[ix + 1][iy + 1]
            d = top_ids[ix][iy + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

            ab = bottom_ids[ix][iy]
            bb = bottom_ids[ix + 1][iy]
            cb = bottom_ids[ix + 1][iy + 1]
            db = bottom_ids[ix][iy + 1]
            geom.add_face(ab, cb, bb)
            geom.add_face(ab, db, cb)

    # Side walls around the belt thickness.
    for ix in range(nx):
        for iy in (0, ny):
            a = top_ids[ix][iy]
            b = top_ids[ix + 1][iy]
            c = bottom_ids[ix + 1][iy]
            d = bottom_ids[ix][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    for iy in range(ny):
        for ix in (0, nx):
            a = top_ids[ix][iy]
            b = top_ids[ix][iy + 1]
            c = bottom_ids[ix][iy + 1]
            d = bottom_ids[ix][iy]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    return geom


def _hanger_link_geometry(y: float) -> MeshGeometry:
    """Short metal hanger link: an eye around the pivot plus a short pendant stem."""
    eye = TorusGeometry(radius=0.035, tube=0.006, radial_segments=18, tubular_segments=32)
    eye.rotate_x(math.pi / 2).translate(0.0, y, 0.0)
    stem = wire_from_points(
        [(0.0, y, -0.035), (0.0, y, -0.165)],
        radius=0.007,
        radial_segments=12,
        cap_ends=True,
    )
    eye.merge(stem)
    return eye


def _chain_geometry(y: float) -> MeshGeometry:
    """A continuous small zig-zag metal chain run between hanger and seat."""
    points = [(0.0, y, -0.150)]
    steps = 16
    for i in range(1, steps + 1):
        z = -0.150 - i * (1.335 / steps)
        x = 0.020 if i % 2 else -0.020
        points.append((x, y, z))
    points.append((0.0, y, -1.505))
    return wire_from_points(
        points,
        radius=0.006,
        radial_segments=12,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.018,
        corner_segments=5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="belt_seat_a_frame_swing")

    frame_green = model.material("powder_coated_green", rgba=(0.05, 0.45, 0.28, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("dark_pivot_steel", rgba=(0.18, 0.19, 0.18, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    top_z = 2.38
    pivot_z = 2.27
    frame = model.part("frame")

    top_beam_length, top_beam_origin = _cylinder_between((0.0, -1.42, top_z), (0.0, 1.42, top_z))
    frame.visual(
        Cylinder(radius=0.055, length=top_beam_length),
        origin=top_beam_origin,
        material=frame_green,
        name="top_beam",
    )

    for y, label in [(-1.27, "front"), (1.27, "rear")]:
        _add_cylinder_between(frame, f"{label}_leg_0", (0.0, y, top_z - 0.02), (-0.88, y, 0.08), 0.043, frame_green)
        _add_cylinder_between(frame, f"{label}_leg_1", (0.0, y, top_z - 0.02), (0.88, y, 0.08), 0.043, frame_green)
        _add_cylinder_between(frame, f"{label}_cross_brace", (-0.54, y, 1.12), (0.54, y, 1.12), 0.025, frame_green)

    for x, label in [(-0.88, "side_0"), (0.88, "side_1")]:
        _add_cylinder_between(frame, f"{label}_foot_rail", (x, -1.27, 0.08), (x, 1.27, 0.08), 0.032, frame_green)

    # Beam clamps and short pivot pins for the two hanger positions.
    for y, label in [(-0.36, "front"), (0.36, "rear")]:
        frame.visual(
            Cylinder(radius=0.071, length=0.085),
            origin=Origin(xyz=(0.0, y, top_z), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name=f"{label}_beam_clamp",
        )
        for side, offset in [("0", -0.030), ("1", 0.030)]:
            frame.visual(
                Box((0.052, 0.008, 0.150)),
                origin=Origin(xyz=(0.0, y + offset, pivot_z + 0.055)),
                material=dark_steel,
                name=f"{label}_clevis_cheek_{side}",
            )
        frame.visual(
            Cylinder(radius=0.013, length=0.120),
            origin=Origin(xyz=(0.0, y, pivot_z), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"{label}_pivot_pin",
        )

    swing = model.part("seat_assembly")
    for y, label in [(-0.36, "front"), (0.36, "rear")]:
        swing.visual(
            mesh_from_geometry(_hanger_link_geometry(y), f"{label}_hanger_link"),
            material=steel,
            name=f"{label}_hanger_link",
        )
        swing.visual(
            mesh_from_geometry(_chain_geometry(y), f"{label}_chain"),
            material=steel,
            name=f"{label}_chain",
        )
        swing.visual(
            Box((0.23, 0.055, 0.022)),
            origin=Origin(xyz=(0.0, y, -1.493)),
            material=steel,
            name=f"{label}_seat_shackle",
        )

    swing.visual(
        mesh_from_geometry(_belt_seat_geometry(), "belt_seat"),
        material=rubber,
        name="belt_seat",
    )
    # Low raised ribs give the rubber belt a molded, flexible playground-seat look.
    for x, label in [(-0.105, "rib_0"), (0.0, "rib_1"), (0.105, "rib_2")]:
        swing.visual(
            Box((0.012, 0.64, 0.010)),
            origin=Origin(xyz=(x, 0.0, -1.585)),
            material=rubber,
            name=label,
        )

    model.articulation(
        "beam_to_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.6, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    swing = object_model.get_part("seat_assembly")
    joint = object_model.get_articulation("beam_to_swing")

    for label in ("front", "rear"):
        ctx.allow_overlap(
            frame,
            swing,
            elem_a=f"{label}_pivot_pin",
            elem_b=f"{label}_hanger_link",
            reason="The steel pivot pin is intentionally captured through the hanger eye.",
        )

    ctx.expect_gap(
        frame,
        swing,
        axis="z",
        positive_elem="top_beam",
        negative_elem="belt_seat",
        min_gap=1.20,
        name="belt hangs well below top beam",
    )
    for label in ("front", "rear"):
        ctx.expect_overlap(
            swing,
            frame,
            axes="xz",
            elem_a=f"{label}_hanger_link",
            elem_b=f"{label}_pivot_pin",
            min_overlap=0.018,
            name=f"{label} hanger surrounds pivot projection",
        )

    rest_box = ctx.part_element_world_aabb(swing, elem="belt_seat")
    rest_center_x = None if rest_box is None else (rest_box[0][0] + rest_box[1][0]) / 2
    with ctx.pose({joint: 0.55}):
        swung_box = ctx.part_element_world_aabb(swing, elem="belt_seat")
        swung_center_x = None if swung_box is None else (swung_box[0][0] + swung_box[1][0]) / 2
    ctx.check(
        "seat assembly swings about beam axis",
        rest_center_x is not None
        and swung_center_x is not None
        and abs(swung_center_x - rest_center_x) > 0.55,
        details=f"rest_center_x={rest_center_x}, swung_center_x={swung_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
