from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
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
    mesh_from_geometry,
    tube_from_spline_points,
    MeshGeometry,
)


def _quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _tray_shell() -> MeshGeometry:
    """Open, thin-walled trapezoid tray with an explicit visible cavity."""
    mesh = MeshGeometry()

    # Corner order is rear/negative-Y clockwise in the XY plane.
    outer_top = [
        (-0.56, -0.40, 0.620),
        (0.56, -0.40, 0.620),
        (0.56, 0.40, 0.620),
        (-0.56, 0.40, 0.620),
    ]
    outer_bottom = [
        (-0.35, -0.23, 0.300),
        (0.39, -0.23, 0.300),
        (0.39, 0.23, 0.300),
        (-0.35, 0.23, 0.300),
    ]
    inner_top = [
        (-0.505, -0.345, 0.582),
        (0.505, -0.345, 0.582),
        (0.505, 0.345, 0.582),
        (-0.505, 0.345, 0.582),
    ]
    inner_bottom = [
        (-0.305, -0.175, 0.345),
        (0.345, -0.175, 0.345),
        (0.345, 0.175, 0.345),
        (-0.305, 0.175, 0.345),
    ]

    loops = []
    for loop in (outer_top, outer_bottom, inner_top, inner_bottom):
        loops.append([mesh.add_vertex(*p) for p in loop])
    ot, ob, it, ib = loops

    for i in range(4):
        j = (i + 1) % 4
        _quad(mesh, ob[i], ob[j], ot[j], ot[i])  # outside wall
        _quad(mesh, it[i], it[j], ib[j], ib[i])  # inside wall
        _quad(mesh, ot[i], ot[j], it[j], it[i])  # flat datum rim
        _quad(mesh, ob[i], ib[i], ib[j], ob[j])  # floor thickness band

    mesh.add_face(ob[0], ob[2], ob[1])  # underside of the tray floor
    mesh.add_face(ob[0], ob[3], ob[2])
    mesh.add_face(ib[0], ib[1], ib[2])  # inside floor surface
    mesh.add_face(ib[0], ib[2], ib[3])
    return mesh


def _hollow_sleeve(height: float = 0.090) -> LatheGeometry:
    return LatheGeometry.from_shell_profiles(
        [(0.030, 0.0), (0.030, height)],
        [(0.014, 0.0), (0.014, height)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_alignment_wheelbarrow")

    tray_m = model.material("satin_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    frame_m = model.material("matte_graphite_tube", rgba=(0.08, 0.09, 0.10, 1.0))
    fork_m = model.material("anodized_fork", rgba=(0.17, 0.20, 0.22, 1.0))
    black_m = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    rim_m = model.material("machined_rim", rgba=(0.86, 0.86, 0.82, 1.0))
    datum_m = model.material("blue_datum_faces", rgba=(0.02, 0.24, 0.70, 1.0))
    mark_m = model.material("etched_index_marks", rgba=(1.0, 0.74, 0.10, 1.0))
    foot_m = model.material("stainless_adjuster", rgba=(0.78, 0.79, 0.76, 1.0))

    tray = model.part("tray")
    tray.visual(
        Box((1.070, 0.710, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.322)),
        material=tray_m,
        name="tray_floor",
    )
    tray.visual(
        Box((1.140, 0.055, 0.340)),
        origin=Origin(xyz=(0.0, -0.355, 0.475)),
        material=tray_m,
        name="side_wall_0",
    )
    tray.visual(
        Box((1.140, 0.055, 0.340)),
        origin=Origin(xyz=(0.0, 0.355, 0.475)),
        material=tray_m,
        name="side_wall_1",
    )
    tray.visual(
        Box((0.055, 0.765, 0.340)),
        origin=Origin(xyz=(0.535, 0.0, 0.475)),
        material=tray_m,
        name="front_wall",
    )
    tray.visual(
        Box((0.055, 0.765, 0.340)),
        origin=Origin(xyz=(-0.535, 0.0, 0.475)),
        material=tray_m,
        name="rear_wall",
    )
    tray.visual(
        Box((0.050, 0.640, 0.012)),
        origin=Origin(xyz=(0.535, 0.0, 0.650)),
        material=datum_m,
        name="front_datum_bar",
    )
    tray.visual(
        Box((0.620, 0.034, 0.012)),
        origin=Origin(xyz=(-0.215, -0.355, 0.650)),
        material=datum_m,
        name="side_datum_bar_0",
    )
    tray.visual(
        Box((0.620, 0.034, 0.012)),
        origin=Origin(xyz=(-0.215, 0.355, 0.650)),
        material=datum_m,
        name="side_datum_bar_1",
    )
    for side_index, y in enumerate((-0.355, 0.355)):
        for tick, x in enumerate((-0.40, -0.20, 0.0, 0.20, 0.40)):
            tray.visual(
                Box((0.010, 0.060, 0.005)),
                origin=Origin(xyz=(x, y, 0.647)),
                material=mark_m,
                name=f"rim_tick_{side_index}_{tick}",
            )
    for tick, y in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
        tray.visual(
            Box((0.045, 0.008, 0.005)),
            origin=Origin(xyz=(0.562, y, 0.647)),
            material=mark_m,
            name=f"front_tick_{tick}",
        )

    fork = model.part("front_fork")
    # Fork cheeks are side plates, joined by a high bridge so the tire has a
    # controlled clearance envelope under the bridge.
    fork.visual(
        Box((0.300, 0.038, 0.420)),
        origin=Origin(xyz=(0.840, -0.145, 0.370)),
        material=fork_m,
        name="fork_cheek_0",
    )
    fork.visual(
        Box((0.300, 0.038, 0.420)),
        origin=Origin(xyz=(0.840, 0.145, 0.370)),
        material=fork_m,
        name="fork_cheek_1",
    )
    fork.visual(
        Box((0.155, 0.330, 0.070)),
        origin=Origin(xyz=(0.700, 0.0, 0.560)),
        material=fork_m,
        name="upper_bridge",
    )
    fork.visual(
        Box((0.100, 0.240, 0.060)),
        origin=Origin(xyz=(0.6125, 0.0, 0.615)),
        material=datum_m,
        name="slotted_mount_face",
    )
    for y in (-0.145, 0.145):
        fork.visual(
            Box((0.018, 0.056, 0.300)),
            origin=Origin(xyz=(0.910, y, 0.375)),
            material=mark_m,
            name=f"axle_scale_{0 if y < 0 else 1}",
        )
    model.articulation(
        "tray_to_front_fork",
        ArticulationType.FIXED,
        parent=tray,
        child=fork,
        origin=Origin(),
    )

    axle = model.part("front_axle")
    axle.visual(
        Cylinder(radius=0.022, length=0.430),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=foot_m,
        name="axle",
    )
    for y, suffix in [(-0.192, "0"), (0.192, "1")]:
        axle.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=datum_m,
            name=f"gap_collar_{suffix}",
        )
    for y, suffix in [(-0.112, "0"), (0.112, "1")]:
        axle.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=mark_m,
            name=f"shim_washer_{suffix}",
        )
    model.articulation(
        "fork_to_axle",
        ArticulationType.FIXED,
        parent=fork,
        child=axle,
        origin=Origin(xyz=(0.850, 0.0, 0.250)),
    )

    wheel = model.part("wheel")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.240,
            0.095,
            inner_radius=0.174,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.010, count=24, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.004),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "front_utility_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.184,
            0.080,
            rim=WheelRim(
                inner_radius=0.116,
                flange_height=0.010,
                flange_thickness=0.005,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.044,
                width=0.070,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.060, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.008, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=8, thickness=0.006, window_radius=0.016),
            bore=WheelBore(style="round", diameter=0.044),
        ),
        "front_spoked_wheel",
    )
    wheel.visual(tire_mesh, material=black_m, name="tire")
    wheel.visual(rim_mesh, material=rim_m, name="rim")
    model.articulation(
        "axle_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=wheel,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=16.0),
        motion_properties=MotionProperties(damping=0.03, friction=0.02),
    )

    sleeve_mesh = mesh_from_geometry(_hollow_sleeve(), "threaded_foot_sleeve")
    for index, y in enumerate((-0.470, 0.470)):
        tube = model.part(f"side_tube_{index}")
        tube.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.650, y, 0.500),
                        (0.190, y, 0.405),
                        (-0.420, y, 0.365),
                        (-0.650, y, 0.175),
                        (-0.650, y, 0.085),
                        (-0.800, y, 0.560),
                        (-1.180, y, 0.730),
                    ],
                    radius=0.017,
                    samples_per_segment=14,
                    radial_segments=20,
                    cap_ends=True,
                ),
                f"side_tube_{index}_sweep",
            ),
            material=frame_m,
            name="bent_tube",
        )
        tube.visual(
            Cylinder(radius=0.027, length=0.230),
            origin=Origin(xyz=(-1.175, y, 0.730), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_m,
            name="handle_grip",
        )
        tube.visual(
            sleeve_mesh,
            origin=Origin(xyz=(-0.585, y, 0.062)),
            material=foot_m,
            name="foot_sleeve",
        )
        tube.visual(
            Box((0.120, 0.087, 0.050)),
            origin=Origin(xyz=(-0.330, y - math.copysign(0.044, y), 0.405)),
            material=datum_m,
            name="rail_datum_pad",
        )
        tube.visual(
            Box((0.070, 0.034, 0.028)),
            origin=Origin(xyz=(-0.633, y, 0.115)),
            material=frame_m,
            name="sleeve_bridge",
        )
        for tick, z in enumerate((0.082, 0.102, 0.122, 0.142)):
            tube.visual(
                Box((0.004, 0.034, 0.0025)),
                origin=Origin(xyz=(-0.553, y, z)),
                material=mark_m,
                name=f"foot_index_{tick}",
            )
        model.articulation(
            f"tray_to_side_tube_{index}",
            ArticulationType.FIXED,
            parent=tray,
            child=tube,
            origin=Origin(),
        )

        foot = model.part(f"level_foot_{index}")
        foot.visual(
            Cylinder(radius=0.010, length=0.145),
            origin=Origin(xyz=(0.0, 0.0, 0.077)),
            material=foot_m,
            name="threaded_stem",
        )
        foot.visual(
            Cylinder(radius=0.064, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=black_m,
            name="level_pad",
        )
        foot.visual(
            Cylinder(radius=0.027, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.056)),
            material=foot_m,
            name="jam_nut",
        )
        foot.visual(
            Box((0.055, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.029)),
            material=mark_m,
            name="witness_tab",
        )
        model.articulation(
            f"tube_to_level_foot_{index}",
            ArticulationType.PRISMATIC,
            parent=tube,
            child=foot,
            origin=Origin(xyz=(-0.585, y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.020, lower=-0.018, upper=0.040),
            motion_properties=MotionProperties(damping=0.12, friction=0.20),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("front_fork")
    axle = object_model.get_part("front_axle")
    wheel = object_model.get_part("wheel")

    for cheek_name in ("fork_cheek_0", "fork_cheek_1"):
        ctx.allow_overlap(
            axle,
            fork,
            elem_a="axle",
            elem_b=cheek_name,
            reason=(
                "The fixed axle is intentionally represented as passing through "
                "the fork cheek bore; the cheek hole detail is indicated by the "
                "index plates rather than cut from the box proxy."
            ),
        )
    ctx.expect_overlap(
        axle,
        fork,
        axes="y",
        min_overlap=0.030,
        elem_a="axle",
        elem_b="fork_cheek_0",
        name="axle is captured through fork cheek",
    )

    ctx.allow_overlap(
        axle,
        wheel,
        elem_a="axle",
        elem_b="rim",
        reason=(
            "The axle is intentionally seated through the wheel hub bore so "
            "the wheel spins around a visible shaft."
        ),
    )
    ctx.expect_overlap(
        axle,
        wheel,
        axes="y",
        min_overlap=0.070,
        elem_a="axle",
        elem_b="rim",
        name="wheel hub remains engaged on axle",
    )
    ctx.expect_origin_distance(
        axle,
        wheel,
        axes="xyz",
        max_dist=0.001,
        name="wheel spin origin stays on axle center",
    )

    spin = object_model.get_articulation("axle_to_wheel")
    p0 = ctx.part_world_position(wheel)
    with ctx.pose({spin: 1.0}):
        p1 = ctx.part_world_position(wheel)
    ctx.check(
        "wheel rotates without translating",
        p0 is not None
        and p1 is not None
        and max(abs(a - b) for a, b in zip(p0, p1)) < 0.001,
        details=f"rest={p0}, spun={p1}",
    )

    ctx.expect_within(
        wheel,
        fork,
        axes="y",
        inner_elem="tire",
        margin=0.010,
        name="tire runs between fork cheeks with visible side gap",
    )

    for index in (0, 1):
        tube = object_model.get_part(f"side_tube_{index}")
        foot = object_model.get_part(f"level_foot_{index}")
        adjuster = object_model.get_articulation(f"tube_to_level_foot_{index}")
        ctx.expect_within(
            foot,
            tube,
            axes="xy",
            inner_elem="threaded_stem",
            outer_elem="foot_sleeve",
            margin=0.001,
            name=f"level foot {index} stem stays centered in sleeve",
        )
        ctx.expect_overlap(
            foot,
            tube,
            axes="z",
            min_overlap=0.070,
            elem_a="threaded_stem",
            elem_b="foot_sleeve",
            name=f"level foot {index} keeps retained thread engagement",
        )
        q0 = ctx.part_world_position(foot)
        with ctx.pose({adjuster: 0.035}):
            q1 = ctx.part_world_position(foot)
        ctx.check(
            f"level foot {index} raises along calibration screw",
            q0 is not None and q1 is not None and q1[2] > q0[2] + 0.030,
            details=f"rest={q0}, raised={q1}",
        )

    return ctx.report()


object_model = build_object_model()
