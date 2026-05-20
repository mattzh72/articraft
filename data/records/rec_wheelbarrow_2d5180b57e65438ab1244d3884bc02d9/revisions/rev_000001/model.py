from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _add_loop(mesh: MeshGeometry, pts: list[tuple[float, float, float]]) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y, z in pts]


def _make_hollow_tray() -> MeshGeometry:
    """Open, tapered contractor tray with wall thickness and a folded rim."""

    mesh = MeshGeometry()

    # Corner order is front-left, front-right, rear-right, rear-left in the
    # wheelbarrow's local frame.  Front is negative X; rear handles are +X.
    outer_top = [
        (-0.54, -0.34, 0.72),
        (-0.54, 0.34, 0.72),
        (0.56, 0.34, 0.72),
        (0.56, -0.34, 0.72),
    ]
    outer_bottom = [
        (-0.35, -0.20, 0.40),
        (-0.35, 0.20, 0.40),
        (0.36, 0.20, 0.40),
        (0.36, -0.20, 0.40),
    ]
    inner_top = [
        (-0.50, -0.30, 0.695),
        (-0.50, 0.30, 0.695),
        (0.52, 0.30, 0.695),
        (0.52, -0.30, 0.695),
    ]
    inner_bottom = [
        (-0.29, -0.15, 0.445),
        (-0.29, 0.15, 0.445),
        (0.30, 0.15, 0.445),
        (0.30, -0.15, 0.445),
    ]

    ot = _add_loop(mesh, outer_top)
    ob = _add_loop(mesh, outer_bottom)
    it = _add_loop(mesh, inner_top)
    ib = _add_loop(mesh, inner_bottom)

    for i in range(4):
        j = (i + 1) % 4
        # Outer wall, rolled rim top, inner wall, and bottom thickness band.
        _add_quad(mesh, ob[i], ob[j], ot[j], ot[i])
        _add_quad(mesh, ot[i], ot[j], it[j], it[i])
        _add_quad(mesh, it[j], it[i], ib[i], ib[j])
        _add_quad(mesh, ob[j], ob[i], ib[i], ib[j])

    # Underside and inside floor caps.
    _add_quad(mesh, ob[0], ob[1], ob[2], ob[3])
    _add_quad(mesh, ib[3], ib[2], ib[1], ib[0])

    return mesh


def _cq_box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Shape:
    return cq.Workplane("XY").box(*size).translate(center).val()


def _cq_cylinder_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    overrun: float = 0.008,
) -> cq.Shape:
    start = cq.Vector(*p0)
    end = cq.Vector(*p1)
    direction = end - start
    length = direction.Length
    unit = direction.normalized()
    base = start - unit * overrun
    return cq.Solid.makeCylinder(radius, length + 2.0 * overrun, base, unit)


def _make_steel_frame() -> cq.Shape:
    """One fused, serviceable steel weldment: rails, fork, axle, legs, mounts."""

    solids: list[cq.Shape] = []

    def tube(p0, p1, radius=0.018, overrun=0.010) -> None:
        solids.append(_cq_cylinder_between(p0, p1, radius, overrun=overrun))

    # Two long tubular handles/side rails running from hand grips to the fork.
    for y in (-0.30, 0.30):
        tube((0.98, y, 0.66), (0.60, y * 0.96, 0.54), 0.019)
        tube((0.60, y * 0.96, 0.54), (0.18, y * 0.86, 0.39), 0.019)
        tube((0.18, y * 0.86, 0.39), (-0.48, y * 0.62, 0.36), 0.019)
        # Fork tine drops from the front rail down to the wheel axle boss.
        tube((-0.48, y * 0.62, 0.36), (-0.71, y * 0.42, 0.25), 0.023)
        # Rear stance leg and a diagonal gusset.
        tube((0.44, y * 0.90, 0.47), (0.58, y * 0.92, 0.09), 0.020)
        tube((0.36, y * 0.85, 0.38), (0.58, y * 0.92, 0.16), 0.014)

    # Crossmembers tie the rails together and visibly support the tray.
    tube((-0.48, -0.22, 0.52), (-0.48, 0.22, 0.52), 0.020)
    tube((-0.48, -0.18, 0.36), (-0.48, -0.18, 0.52), 0.016)
    tube((-0.48, 0.18, 0.36), (-0.48, 0.18, 0.52), 0.016)
    tube((-0.26, -0.28, 0.39), (-0.26, 0.28, 0.39), 0.018)
    tube((0.25, -0.29, 0.41), (0.25, 0.29, 0.41), 0.018)
    tube((0.58, -0.30, 0.12), (0.58, 0.30, 0.12), 0.016)

    # Tray saddles and bolt bosses penetrate the tray bottom to make the
    # maintenance-access mounting path obvious.
    for x in (-0.26, 0.25):
        for y in (-0.18, 0.18):
            tube((x, y, 0.385), (x, y, 0.465), 0.016, overrun=0.0)
            solids.append(_cq_box((x, y, 0.468), (0.075, 0.045, 0.018)))

    # Rear feet are broad replaceable shoes welded to the tubular legs.
    for y in (-0.276, 0.276):
        solids.append(_cq_box((0.595, y, 0.065), (0.18, 0.075, 0.060)))

    # Fork cheek reinforcement plates around the axle for a chunky field repair
    # silhouette.
    for y in (-0.132, 0.132):
        solids.append(_cq_box((-0.66, y, 0.295), (0.16, 0.018, 0.12)))

    frame = solids[0]
    for solid in solids[1:]:
        frame = frame.fuse(solid)
    return frame


def _make_axle_assembly() -> cq.Shape:
    """Service axle with shaft, spacer collars, and outer retaining nuts."""

    solids = [
        _cq_cylinder_between((-0.71, -0.18, 0.25), (-0.71, 0.18, 0.25), 0.022, overrun=0.0),
        _cq_cylinder_between((-0.71, -0.100, 0.25), (-0.71, -0.070, 0.25), 0.040, overrun=0.0),
        _cq_cylinder_between((-0.71, 0.070, 0.25), (-0.71, 0.100, 0.25), 0.040, overrun=0.0),
        _cq_cylinder_between((-0.71, -0.155, 0.25), (-0.71, -0.125, 0.25), 0.032, overrun=0.0),
        _cq_cylinder_between((-0.71, 0.125, 0.25), (-0.71, 0.155, 0.25), 0.032, overrun=0.0),
    ]
    axle = solids[0]
    for solid in solids[1:]:
        axle = axle.fuse(solid)
    return axle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wheelbarrow")

    tray_paint = model.material("worn_orange_paint", color=(0.86, 0.29, 0.08, 1.0))
    dark_steel = model.material("black_powdercoated_steel", color=(0.03, 0.035, 0.035, 1.0))
    tire_rubber = model.material("chunky_black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    galvanized = model.material("galvanized_service_metal", color=(0.58, 0.60, 0.58, 1.0))
    grip_rubber = model.material("replaceable_grip_rubber", color=(0.015, 0.012, 0.010, 1.0))
    bolt_dark = model.material("dark_bolt_heads", color=(0.02, 0.02, 0.018, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_geometry(_make_hollow_tray(), "hollow_tray"),
        material=tray_paint,
        name="tray_shell",
    )
    chassis.visual(
        mesh_from_cadquery(_make_steel_frame(), "steel_frame", tolerance=0.002),
        material=dark_steel,
        name="steel_frame",
    )
    chassis.visual(
        mesh_from_cadquery(_make_axle_assembly(), "axle_assembly", tolerance=0.0015),
        material=galvanized,
        name="axle_assembly",
    )

    # Replaceable rubber hand grips sleeve over the rear ends of the steel tube.
    for y, name in ((-0.30, "grip_0"), (0.30, "grip_1")):
        chassis.visual(
            mesh_from_cadquery(
                _cq_cylinder_between((0.88, y, 0.63), (1.04, y, 0.68), 0.030, overrun=0.0),
                name,
                tolerance=0.002,
            ),
            material=grip_rubber,
            name=name,
        )

    # Four large bolt heads are exposed inside the tray for clear service access.
    for x in (-0.26, 0.25):
        for y in (-0.18, 0.18):
            suffix = f"{0 if x < 0 else 1}_{0 if y < 0 else 1}"
            chassis.visual(
                mesh_from_cadquery(
                    _cq_cylinder_between((x, y, 0.438), (x, y, 0.468), 0.018, overrun=0.0),
                    f"bolt_head_{suffix}",
                    tolerance=0.0015,
                ),
                material=bolt_dark,
                name=f"bolt_head_{suffix}",
            )

    wheel = model.part("front_wheel")
    tire = TireGeometry(
        0.230,
        0.100,
        inner_radius=0.152,
        tread=TireTread(style="block", depth=0.012, count=22, land_ratio=0.56),
        grooves=(TireGroove(center_offset=0.0, width=0.010, depth=0.004),),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.012, radius=0.004),
    )
    rim = WheelGeometry(
        0.158,
        0.082,
        rim=WheelRim(
            inner_radius=0.095,
            flange_height=0.014,
            flange_thickness=0.006,
            bead_seat_depth=0.004,
        ),
        hub=WheelHub(
            radius=0.052,
            width=0.092,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=6, circle_diameter=0.070, hole_diameter=0.008),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.008, window_radius=0.026),
        bore=WheelBore(style="round", diameter=0.044),
    )
    wheel.visual(
        mesh_from_geometry(tire, "tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(rim, "rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=galvanized,
        name="rim",
    )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=wheel,
        origin=Origin(xyz=(-0.71, 0.0, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    wheel = object_model.get_part("front_wheel")
    axle = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        chassis,
        wheel,
        elem_a="axle_assembly",
        elem_b="rim",
        reason="The replaceable axle shaft is intentionally captured through the wheel hub bearing bore.",
    )

    ctx.check(
        "single front wheel spins on transverse axle",
        axle.articulation_type == ArticulationType.CONTINUOUS and tuple(axle.axis) == (0.0, 1.0, 0.0),
        details=f"type={axle.articulation_type}, axis={axle.axis}",
    )
    ctx.expect_overlap(
        wheel,
        chassis,
        axes="xz",
        elem_a="rim",
        elem_b="steel_frame",
        min_overlap=0.06,
        name="fork and axle surround the wheel hub in side view",
    )
    ctx.expect_within(
        chassis,
        wheel,
        axes="xz",
        inner_elem="axle_assembly",
        outer_elem="rim",
        margin=0.0,
        name="axle shaft stays inside the hub bore projection",
    )
    ctx.expect_overlap(
        chassis,
        wheel,
        axes="y",
        elem_a="axle_assembly",
        elem_b="rim",
        min_overlap=0.08,
        name="axle passes through the serviceable hub width",
    )
    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({axle: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel rotation stays captured on axle",
        rest_pos is not None and turned_pos is not None and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, rotated={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
