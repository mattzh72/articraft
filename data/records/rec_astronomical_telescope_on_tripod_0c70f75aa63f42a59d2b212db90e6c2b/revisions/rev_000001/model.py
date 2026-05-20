from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


POLAR_ROLL = math.radians(48.0)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _origin_aligned_z(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(0.0, pitch, yaw)), length


def _ra_local_to_world(
    local_xyz: tuple[float, float, float],
    *,
    ra_origin: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z = local_xyz
    ox, oy, oz = ra_origin
    c = math.cos(POLAR_ROLL)
    s = math.sin(POLAR_ROLL)
    return (ox + x, oy + c * y - s * z, oz + s * y + c * z)


def _hollow_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
    segments: int = 72,
) -> tuple[object, Origin]:
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    shell.rotate_y(math.pi / 2.0).translate(*center)
    return mesh_from_geometry(shell, name), Origin()


def _ring_saddle_mesh(name: str, *, x: float, tube_z: float) -> object:
    """A split-era refractor clamp ring: annular collar plus a foot block."""
    ring = TorusGeometry(0.089, 0.018, radial_segments=24, tubular_segments=80)
    # The torus axis becomes the telescope/tube axis X.
    ring.rotate_y(math.pi / 2.0).translate(x, 0.0, tube_z)

    foot = BoxGeometry((0.115, 0.190, 0.130))
    foot.translate(x, 0.0, tube_z - 0.170)
    ring.merge(foot)
    return mesh_from_geometry(ring, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classical_achromatic_refractor")

    model.material("varnished_wood", rgba=(0.50, 0.28, 0.12, 1.0))
    model.material("dark_wood", rgba=(0.30, 0.16, 0.07, 1.0))
    model.material("brass", rgba=(0.83, 0.62, 0.28, 1.0))
    model.material("aged_black_metal", rgba=(0.02, 0.022, 0.025, 1.0))
    model.material("cream_enamel", rgba=(0.88, 0.84, 0.70, 1.0))
    model.material("blue_glass", rgba=(0.45, 0.72, 0.95, 0.45))
    model.material("polished_steel", rgba=(0.62, 0.64, 0.62, 1.0))

    base = model.part("base")

    # Three splayed wooden legs and a compact wooden head establish the
    # classical field-mount silhouette and a believable absolute scale.
    for i, angle in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        top = (0.17 * math.cos(angle), 0.17 * math.sin(angle), 0.58)
        foot = (0.82 * math.cos(angle), 0.82 * math.sin(angle), 0.06)
        origin, length = _origin_aligned_z(top, foot)
        base.visual(
            Box((0.070, 0.040, length)),
            origin=origin,
            material="varnished_wood",
            name=f"tripod_leg_{i}",
        )
        base.visual(
            Cylinder(radius=0.055, length=0.035),
            origin=Origin(xyz=foot),
            material="dark_wood",
            name=f"foot_pad_{i}",
        )

    base.visual(
        Cylinder(radius=0.160, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material="varnished_wood",
        name="tripod_hub",
    )
    base.visual(
        Box((0.40, 0.32, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.695)),
        material="varnished_wood",
        name="head_plate",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        material="varnished_wood",
        name="central_post",
    )

    ra_origin = (0.0, 0.0, 1.22)
    polar_rpy = (POLAR_ROLL, 0.0, 0.0)
    for side, x in enumerate((-0.095, 0.095)):
        base.visual(
            Box((0.065, 0.170, 0.540)),
            origin=Origin(
                xyz=_ra_local_to_world((x, 0.0, -0.035), ra_origin=ra_origin),
                rpy=polar_rpy,
            ),
            material="varnished_wood",
            name=f"polar_cheek_{side}",
        )
    base.visual(
        Box((0.335, 0.185, 0.090)),
        origin=Origin(
            xyz=_ra_local_to_world((0.0, 0.0, -0.335), ra_origin=ra_origin),
            rpy=polar_rpy,
        ),
        material="dark_wood",
        name="polar_crossblock",
    )
    base.visual(
        Box((0.34, 0.22, 0.16)),
        origin=Origin(xyz=(0.0, -0.04, 0.89), rpy=(0.0, 0.0, 0.0)),
        material="dark_wood",
        name="wedge_block",
    )
    for side, x in enumerate((-0.095, 0.095)):
        base.visual(
            Box((0.070, 0.120, 0.320)),
            origin=Origin(xyz=(x, 0.020, 0.980)),
            material="varnished_wood",
            name=f"polar_side_post_{side}",
        )
    for bearing_name, local_z in (("lower_polar_bearing", -0.105), ("upper_polar_bearing", 0.180)):
        base.visual(
            Cylinder(radius=0.064, length=0.120),
            origin=Origin(
                xyz=_ra_local_to_world((0.0, 0.0, local_z), ra_origin=ra_origin),
                rpy=polar_rpy,
            ),
            material="brass",
            name=bearing_name,
        )

    ra_axis = model.part("ra_axis")
    ra_axis.visual(
        Cylinder(radius=0.044, length=0.606),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material="polished_steel",
        name="polar_shaft",
    )
    ra_axis.visual(
        Cylinder(radius=0.118, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material="brass",
        name="ra_setting_circle",
    )
    ra_axis.visual(
        Cylinder(radius=0.072, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.460), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="aged_black_metal",
        name="dec_bearing",
    )
    ra_axis.visual(
        Cylinder(radius=0.030, length=0.160),
        origin=Origin(xyz=(0.0, -0.160, 0.310), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brass",
        name="ra_clamp_knob",
    )

    model.articulation(
        "right_ascension",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=ra_axis,
        origin=Origin(xyz=ra_origin, rpy=polar_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.8),
    )

    declination_assembly = model.part("declination_assembly")
    declination_assembly.visual(
        Cylinder(radius=0.041, length=0.560),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="polished_steel",
        name="dec_shaft",
    )
    declination_assembly.visual(
        Cylinder(radius=0.096, length=0.135),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="aged_black_metal",
        name="dec_hub",
    )
    declination_assembly.visual(
        Box((1.48, 0.180, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material="brass",
        name="saddle_rail",
    )
    declination_assembly.visual(
        Box((0.240, 0.180, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material="aged_black_metal",
        name="saddle_boss",
    )
    tube_z = 0.300
    declination_assembly.visual(
        _ring_saddle_mesh("ring_saddle_0", x=-0.58, tube_z=tube_z),
        material="brass",
        name="ring_saddle_0",
    )
    declination_assembly.visual(
        Cylinder(radius=0.012, length=0.075),
        origin=Origin(xyz=(-0.58, 0.0, tube_z + 0.128)),
        material="aged_black_metal",
        name="clamp_screw_0",
    )
    declination_assembly.visual(
        _ring_saddle_mesh("ring_saddle_1", x=0.58, tube_z=tube_z),
        material="brass",
        name="ring_saddle_1",
    )
    declination_assembly.visual(
        Cylinder(radius=0.012, length=0.075),
        origin=Origin(xyz=(0.58, 0.0, tube_z + 0.128)),
        material="aged_black_metal",
        name="clamp_screw_1",
    )

    # A counterweight and rod opposite the tube keep the German equatorial mount
    # visually plausible while remaining part of the same declination assembly.
    declination_assembly.visual(
        Cylinder(radius=0.020, length=0.620),
        origin=Origin(xyz=(0.0, -0.460, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="polished_steel",
        name="counterweight_rod",
    )
    declination_assembly.visual(
        Cylinder(radius=0.090, length=0.090),
        origin=Origin(xyz=(0.0, -0.720, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="aged_black_metal",
        name="counterweight",
    )

    model.articulation(
        "declination",
        ArticulationType.REVOLUTE,
        parent=ra_axis,
        child=declination_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-1.35, upper=1.35),
    )

    telescope_tube = model.part("telescope_tube")
    main_mesh, main_origin = _hollow_tube_mesh(
        "main_tube_mesh",
        outer_radius=0.072,
        inner_radius=0.066,
        length=1.850,
        center=(-0.050, 0.0, tube_z),
    )
    telescope_tube.visual(
        main_mesh,
        origin=main_origin,
        material="cream_enamel",
        name="main_tube",
    )
    dew_mesh, dew_origin = _hollow_tube_mesh(
        "dew_shield_mesh",
        outer_radius=0.088,
        inner_radius=0.070,
        length=0.335,
        center=(1.035, 0.0, tube_z),
    )
    telescope_tube.visual(
        dew_mesh,
        origin=dew_origin,
        material="aged_black_metal",
        name="dew_shield",
    )
    cell_mesh, cell_origin = _hollow_tube_mesh(
        "objective_cell_mesh",
        outer_radius=0.086,
        inner_radius=0.061,
        length=0.070,
        center=(1.155, 0.0, tube_z),
    )
    telescope_tube.visual(
        cell_mesh,
        origin=cell_origin,
        material="brass",
        name="objective_cell",
    )
    telescope_tube.visual(
        Cylinder(radius=0.064, length=0.008),
        origin=Origin(xyz=(1.154, 0.0, tube_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="blue_glass",
        name="objective_glass",
    )
    rear_mesh, rear_origin = _hollow_tube_mesh(
        "rear_cell_mesh",
        outer_radius=0.074,
        inner_radius=0.041,
        length=0.170,
        center=(-1.040, 0.0, tube_z),
    )
    telescope_tube.visual(
        rear_mesh,
        origin=rear_origin,
        material="aged_black_metal",
        name="rear_cell",
    )
    telescope_tube.visual(
        Cylinder(radius=0.043, length=0.245),
        origin=Origin(xyz=(-1.200, 0.0, tube_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brass",
        name="drawtube",
    )
    telescope_tube.visual(
        Cylinder(radius=0.028, length=0.105),
        origin=Origin(xyz=(-1.365, 0.0, tube_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="aged_black_metal",
        name="eyepiece",
    )
    telescope_tube.visual(
        Cylinder(radius=0.009, length=0.100),
        origin=Origin(xyz=(-1.055, 0.075, tube_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="polished_steel",
        name="focus_spindle",
    )

    model.articulation(
        "tube_mount",
        ArticulationType.FIXED,
        parent=declination_assembly,
        child=telescope_tube,
        origin=Origin(),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="aged_black_metal",
        name="knurled_wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.043, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brass",
        name="brass_cap",
    )
    model.articulation(
        "focus_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=telescope_tube,
        child=focus_knob,
        origin=Origin(xyz=(-1.055, 0.125, tube_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ra = object_model.get_articulation("right_ascension")
    dec = object_model.get_articulation("declination")
    focus = object_model.get_articulation("focus_knob_spin")
    base = object_model.get_part("base")
    tube = object_model.get_part("telescope_tube")
    saddle = object_model.get_part("declination_assembly")
    ra_axis = object_model.get_part("ra_axis")

    for bearing in ("lower_polar_bearing", "upper_polar_bearing"):
        ctx.allow_overlap(
            base,
            ra_axis,
            elem_a=bearing,
            elem_b="polar_shaft",
            reason="The polar shaft is intentionally captured through the wooden mount head bearing.",
        )
        ctx.expect_overlap(
            base,
            ra_axis,
            axes="z",
            elem_a=bearing,
            elem_b="polar_shaft",
            min_overlap=0.05,
            name=f"{bearing} retains polar shaft",
        )

    ctx.allow_overlap(
        ra_axis,
        saddle,
        elem_a="dec_bearing",
        elem_b="dec_shaft",
        reason="The declination shaft is intentionally captured inside the right-ascension head bearing.",
    )
    ctx.expect_within(
        saddle,
        ra_axis,
        axes="x",
        inner_elem="dec_shaft",
        outer_elem="dec_bearing",
        margin=0.002,
        name="declination shaft is laterally centered in bearing",
    )
    ctx.expect_overlap(
        saddle,
        ra_axis,
        axes="y",
        elem_a="dec_shaft",
        elem_b="dec_bearing",
        min_overlap=0.12,
        name="declination shaft has retained bearing insertion",
    )
    ctx.allow_overlap(
        ra_axis,
        saddle,
        elem_a="dec_bearing",
        elem_b="dec_hub",
        reason="The declination hub is intentionally represented as rotating around the bearing sleeve.",
    )
    ctx.expect_overlap(
        saddle,
        ra_axis,
        axes="y",
        elem_a="dec_hub",
        elem_b="dec_bearing",
        min_overlap=0.10,
        name="declination hub surrounds bearing sleeve",
    )
    ctx.allow_overlap(
        ra_axis,
        saddle,
        elem_a="polar_shaft",
        elem_b="dec_hub",
        reason="The right-ascension shaft enters the declination head as a local captured axle seat.",
    )
    ctx.expect_overlap(
        ra_axis,
        saddle,
        axes="z",
        elem_a="polar_shaft",
        elem_b="dec_hub",
        min_overlap=0.04,
        name="polar shaft enters declination head",
    )

    ctx.check(
        "right ascension is continuous",
        str(ra.articulation_type).lower().endswith("continuous"),
        details=str(ra.articulation_type),
    )
    ctx.check(
        "right ascension axis is tilted",
        0.75 < abs(ra.origin.rpy[0]) < 0.95,
        details=f"rpy={ra.origin.rpy}",
    )
    ctx.check(
        "declination is bounded revolute",
        dec.motion_limits is not None and dec.motion_limits.lower < 0.0 < dec.motion_limits.upper,
        details=f"limits={dec.motion_limits}",
    )
    ctx.check(
        "focus knob is continuous",
        str(focus.articulation_type).lower().endswith("continuous"),
        details=str(focus.articulation_type),
    )

    for ring in ("ring_saddle_0", "ring_saddle_1"):
        ctx.allow_overlap(
            saddle,
            tube,
            elem_a=ring,
            elem_b="main_tube",
            reason="The split saddle ring intentionally clamps lightly around the telescope tube.",
        )

    ctx.expect_within(
        tube,
        saddle,
        axes="yz",
        inner_elem="main_tube",
        outer_elem="ring_saddle_0",
        margin=0.010,
        name="tube sits inside first ring saddle outline",
    )
    ctx.expect_within(
        tube,
        saddle,
        axes="yz",
        inner_elem="main_tube",
        outer_elem="ring_saddle_1",
        margin=0.010,
        name="tube sits inside second ring saddle outline",
    )
    ctx.expect_overlap(
        tube,
        saddle,
        axes="x",
        elem_a="main_tube",
        elem_b="ring_saddle_0",
        min_overlap=0.04,
        name="first ring encircles tube along optical axis",
    )
    ctx.expect_overlap(
        tube,
        saddle,
        axes="x",
        elem_a="main_tube",
        elem_b="ring_saddle_1",
        min_overlap=0.04,
        name="second ring encircles tube along optical axis",
    )

    closed_aabb = ctx.part_element_world_aabb(tube, elem="objective_glass")
    with ctx.pose({dec: 0.75}):
        moved_aabb = ctx.part_element_world_aabb(tube, elem="objective_glass")
    closed_z = None if closed_aabb is None else (closed_aabb[0][2] + closed_aabb[1][2]) / 2.0
    moved_z = None if moved_aabb is None else (moved_aabb[0][2] + moved_aabb[1][2]) / 2.0
    ctx.check(
        "declination moves telescope tube",
        closed_z is not None and moved_z is not None and abs(moved_z - closed_z) > 0.05,
        details=f"closed_z={closed_z}, moved_z={moved_z}",
    )

    return ctx.report()


object_model = build_object_model()
