from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


TRAY_GREEN = Material("powder_coated_green", color=(0.08, 0.36, 0.15, 1.0))
FRAME_BLACK = Material("black_tubular_steel", color=(0.015, 0.017, 0.016, 1.0))
RUBBER = Material("matte_black_rubber", color=(0.005, 0.005, 0.004, 1.0))
RIM_RED = Material("painted_red_rim", color=(0.72, 0.06, 0.035, 1.0))
AXLE_STEEL = Material("galvanized_steel", color=(0.55, 0.56, 0.52, 1.0))


def _origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Return a cylinder origin/length with local +Z aligned from start to end."""

    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1e-9:
        raise ValueError("Cannot build a zero-length tube")
    nx, ny, nz = vx / length, vy / length, vz / length
    horizontal = math.sqrt(nx * nx + ny * ny)
    pitch = math.atan2(horizontal, nz)
    yaw = math.atan2(ny, nx) if horizontal > 1e-9 else 0.0
    return Origin(xyz=((sx + ex) / 2, (sy + ey) / 2, (sz + ez) / 2), rpy=(0.0, pitch, yaw)), length


def _tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material: Material,
    name: str,
) -> None:
    origin, length = _origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_ring_faces(mesh: MeshGeometry, outer: list[int], inner: list[int]) -> None:
    count = len(outer)
    for i in range(count):
        j = (i + 1) % count
        mesh.add_face(outer[i], outer[j], inner[j])
        mesh.add_face(outer[i], inner[j], inner[i])


def _add_loop_cap(mesh: MeshGeometry, loop: list[int], *, reverse: bool = False) -> None:
    center_index = mesh.add_vertex(
        sum(mesh.vertices[i][0] for i in loop) / len(loop),
        sum(mesh.vertices[i][1] for i in loop) / len(loop),
        sum(mesh.vertices[i][2] for i in loop) / len(loop),
    )
    for i in range(len(loop)):
        j = (i + 1) % len(loop)
        if reverse:
            mesh.add_face(center_index, j, i)
        else:
            mesh.add_face(center_index, i, j)


def _tray_shell_geometry() -> MeshGeometry:
    """A tapered, open wheelbarrow tray with real wall thickness and rounded corners."""

    mesh = MeshGeometry()
    sections = {
        "outer_bottom": (0.78, 0.42, 0.045, 0.02, 0.0, 0.38),
        "outer_top": (1.18, 0.74, 0.085, -0.02, 0.0, 0.74),
        "inner_top": (1.06, 0.62, 0.055, -0.02, 0.0, 0.705),
        "inner_bottom": (0.60, 0.28, 0.030, 0.02, 0.0, 0.435),
    }
    loops: dict[str, list[int]] = {}
    for key, (length, width, radius, cx, cy, z) in sections.items():
        loop: list[int] = []
        for px, py in rounded_rect_profile(length, width, radius, corner_segments=8):
            loop.append(mesh.add_vertex(px + cx, py + cy, z))
        loops[key] = loop

    _add_ring_faces(mesh, loops["outer_bottom"], loops["outer_top"])
    _add_ring_faces(mesh, loops["outer_top"], loops["inner_top"])
    _add_ring_faces(mesh, loops["inner_top"], loops["inner_bottom"])
    _add_loop_cap(mesh, loops["outer_bottom"], reverse=True)
    _add_loop_cap(mesh, loops["inner_bottom"])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_wheel_wheelbarrow")

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_tray_shell_geometry(), "tapered_tray_shell"),
        material=TRAY_GREEN,
        name="tray_shell",
    )

    # Rolled upper rim of the tray.
    _tube(frame, (-0.58, 0.375, 0.745), (0.54, 0.375, 0.745), 0.018, material=TRAY_GREEN, name="rim_side_0")
    _tube(frame, (-0.58, -0.375, 0.745), (0.54, -0.375, 0.745), 0.018, material=TRAY_GREEN, name="rim_side_1")
    _tube(frame, (0.55, -0.34, 0.745), (0.55, 0.34, 0.745), 0.018, material=TRAY_GREEN, name="front_rim")
    _tube(frame, (-0.60, -0.32, 0.745), (-0.60, 0.32, 0.745), 0.018, material=TRAY_GREEN, name="rear_rim")

    # Two long handle rails continue forward into the wheel fork.
    _tube(frame, (-1.08, 0.31, 0.66), (0.46, 0.26, 0.36), 0.018, material=FRAME_BLACK, name="handle_0")
    _tube(frame, (-1.08, -0.31, 0.66), (0.46, -0.26, 0.36), 0.018, material=FRAME_BLACK, name="handle_1")
    _tube(frame, (-1.18, 0.315, 0.68), (-0.90, 0.305, 0.625), 0.028, material=RUBBER, name="grip_0")
    _tube(frame, (-1.18, -0.315, 0.68), (-0.90, -0.305, 0.625), 0.028, material=RUBBER, name="grip_1")

    # Bracing under the tray and at the rear keeps the two handles visibly tied together.
    _tube(frame, (-0.52, -0.29, 0.51), (-0.52, 0.29, 0.51), 0.015, material=FRAME_BLACK, name="rear_crossbrace")
    _tube(frame, (0.23, -0.27, 0.405), (0.23, 0.27, 0.405), 0.015, material=FRAME_BLACK, name="front_crossbrace")
    _tube(frame, (-0.32, 0.28, 0.475), (0.30, -0.27, 0.405), 0.012, material=FRAME_BLACK, name="diagonal_brace_0")
    _tube(frame, (-0.32, -0.28, 0.475), (0.30, 0.27, 0.405), 0.012, material=FRAME_BLACK, name="diagonal_brace_1")

    # Rear resting legs with small feet, fixed to the handle rails.
    _tube(frame, (-0.50, 0.295, 0.545), (-0.62, 0.29, 0.045), 0.018, material=FRAME_BLACK, name="leg_0")
    _tube(frame, (-0.50, -0.295, 0.545), (-0.62, -0.29, 0.045), 0.018, material=FRAME_BLACK, name="leg_1")
    _tube(frame, (-0.72, 0.29, 0.035), (-0.50, 0.29, 0.035), 0.020, material=FRAME_BLACK, name="foot_0")
    _tube(frame, (-0.72, -0.29, 0.035), (-0.50, -0.29, 0.035), 0.020, material=FRAME_BLACK, name="foot_1")

    # Narrow fork around the single front wheel.  The plates sit outside the tire.
    _tube(frame, (0.42, 0.25, 0.365), (0.78, 0.085, 0.245), 0.018, material=FRAME_BLACK, name="fork_arm_0")
    _tube(frame, (0.42, -0.25, 0.365), (0.78, -0.085, 0.245), 0.018, material=FRAME_BLACK, name="fork_arm_1")
    frame.visual(Box((0.18, 0.014, 0.15)), origin=Origin(xyz=(0.76, 0.080, 0.255)), material=FRAME_BLACK, name="fork_plate_0")
    frame.visual(Box((0.18, 0.014, 0.15)), origin=Origin(xyz=(0.76, -0.080, 0.255)), material=FRAME_BLACK, name="fork_plate_1")
    _tube(frame, (0.78, 0.066, 0.245), (0.78, 0.095, 0.245), 0.032, material=AXLE_STEEL, name="axle_boss_0")
    _tube(frame, (0.78, -0.095, 0.245), (0.78, -0.066, 0.245), 0.032, material=AXLE_STEEL, name="axle_boss_1")

    wheel = model.part("front_wheel")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.240,
            0.115,
            inner_radius=0.165,
            tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "front_utility_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.165,
            0.082,
            rim=WheelRim(inner_radius=0.105, flange_height=0.012, flange_thickness=0.006, bead_seat_depth=0.005),
            hub=WheelHub(
                radius=0.048,
                width=0.090,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.056, hole_diameter=0.007),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.007, window_radius=0.016),
            bore=WheelBore(style="round", diameter=0.020),
        ),
        "front_red_rim",
    )
    wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2)), material=RUBBER, name="tire")
    wheel.visual(rim_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2)), material=RIM_RED, name="rim")

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.78, 0.0, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("front_wheel")
    axle = object_model.get_articulation("wheel_axle")

    ctx.check(
        "front wheel uses continuous axle joint",
        axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={axle.articulation_type}",
    )
    ctx.expect_origin_gap(wheel, frame, axis="x", min_gap=0.70, max_gap=0.86, name="wheel sits at the front of the frame")
    ctx.expect_gap(
        frame,
        wheel,
        axis="y",
        positive_elem="fork_plate_0",
        negative_elem="tire",
        min_gap=0.003,
        max_gap=0.035,
        name="positive fork plate clears tire",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="fork_plate_1",
        min_gap=0.003,
        max_gap=0.035,
        name="negative fork plate clears tire",
    )

    tire_aabb = ctx.part_element_world_aabb(wheel, elem="tire")
    frame_aabb = ctx.part_world_aabb(frame)
    if tire_aabb is not None and frame_aabb is not None:
        (tire_min, tire_max), (frame_min, frame_max) = tire_aabb, frame_aabb
        tire_diameter = max(tire_max[0] - tire_min[0], tire_max[2] - tire_min[2])
        frame_height = frame_max[2] - frame_min[2]
        ctx.check(
            "wheel is large relative to fixed frame",
            tire_diameter > 0.40 and tire_diameter > 0.55 * frame_height,
            details=f"tire_diameter={tire_diameter:.3f}, frame_height={frame_height:.3f}",
        )
    else:
        ctx.fail("wheel is large relative to fixed frame", "missing tire or frame AABB")

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({axle: math.pi / 2}):
        turned_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins in place about axle",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(rest_pos[i] - turned_pos[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
