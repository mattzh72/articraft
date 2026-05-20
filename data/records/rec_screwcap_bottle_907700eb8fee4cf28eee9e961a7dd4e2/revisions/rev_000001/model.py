from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PITCH = 0.009
CAP_BOTTOM_Z = 0.155
CAP_ROTATION_LIMIT = 2.0 * pi
CAP_LIFT_LIMIT = PITCH


def _quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def annular_cylinder(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    mesh = MeshGeometry()
    outer_low: list[int] = []
    outer_high: list[int] = []
    inner_low: list[int] = []
    inner_high: list[int] = []

    for i in range(segments):
        t = 2.0 * pi * i / segments
        ct, st = cos(t), sin(t)
        outer_low.append(mesh.add_vertex(outer_radius * ct, outer_radius * st, z_min))
        outer_high.append(mesh.add_vertex(outer_radius * ct, outer_radius * st, z_max))
        inner_low.append(mesh.add_vertex(inner_radius * ct, inner_radius * st, z_min))
        inner_high.append(mesh.add_vertex(inner_radius * ct, inner_radius * st, z_max))

    for i in range(segments):
        j = (i + 1) % segments
        _quad(mesh, outer_low[i], outer_low[j], outer_high[j], outer_high[i])
        _quad(mesh, inner_low[j], inner_low[i], inner_high[i], inner_high[j])
        _quad(mesh, outer_high[i], outer_high[j], inner_high[j], inner_high[i])
        _quad(mesh, inner_low[i], inner_low[j], outer_low[j], outer_low[i])

    return mesh


def cap_shell(
    inner_radius: float,
    outer_radius: float,
    height: float,
    top_thickness: float,
    *,
    segments: int = 128,
) -> MeshGeometry:
    mesh = MeshGeometry()
    z0 = 0.0
    z_cavity_top = height - top_thickness
    z_top = height
    outer_low: list[int] = []
    outer_top: list[int] = []
    inner_low: list[int] = []
    inner_top: list[int] = []

    for i in range(segments):
        t = 2.0 * pi * i / segments
        ct, st = cos(t), sin(t)
        outer_low.append(mesh.add_vertex(outer_radius * ct, outer_radius * st, z0))
        outer_top.append(mesh.add_vertex(outer_radius * ct, outer_radius * st, z_top))
        inner_low.append(mesh.add_vertex(inner_radius * ct, inner_radius * st, z0))
        inner_top.append(mesh.add_vertex(inner_radius * ct, inner_radius * st, z_cavity_top))

    top_center = mesh.add_vertex(0.0, 0.0, z_top)
    inner_ceiling = mesh.add_vertex(0.0, 0.0, z_cavity_top)

    for i in range(segments):
        j = (i + 1) % segments
        _quad(mesh, outer_low[i], outer_low[j], outer_top[j], outer_top[i])
        _quad(mesh, inner_low[j], inner_low[i], inner_top[i], inner_top[j])
        _quad(mesh, inner_low[i], inner_low[j], outer_low[j], outer_low[i])
        mesh.add_face(top_center, outer_top[i], outer_top[j])
        mesh.add_face(inner_ceiling, inner_top[j], inner_top[i])

    return mesh


def helical_thread(
    root_radius: float,
    crest_radius: float,
    z_start: float,
    turns: float,
    *,
    pitch: float = PITCH,
    width: float = 0.0022,
    phase: float = 0.0,
    segments_per_turn: int = 48,
) -> MeshGeometry:
    mesh = MeshGeometry()
    steps = int(turns * segments_per_turn)
    root_low: list[int] = []
    root_high: list[int] = []
    crest_low: list[int] = []
    crest_high: list[int] = []

    for i in range(steps + 1):
        theta = phase + 2.0 * pi * i / segments_per_turn
        z = z_start + pitch * i / segments_per_turn
        ct, st = cos(theta), sin(theta)
        root_low.append(mesh.add_vertex(root_radius * ct, root_radius * st, z - width / 2.0))
        root_high.append(mesh.add_vertex(root_radius * ct, root_radius * st, z + width / 2.0))
        crest_low.append(mesh.add_vertex(crest_radius * ct, crest_radius * st, z - width / 2.0))
        crest_high.append(mesh.add_vertex(crest_radius * ct, crest_radius * st, z + width / 2.0))

    for i in range(steps):
        j = i + 1
        _quad(mesh, root_low[i], root_low[j], root_high[j], root_high[i])
        _quad(mesh, crest_low[j], crest_low[i], crest_high[i], crest_high[j])
        _quad(mesh, root_high[i], root_high[j], crest_high[j], crest_high[i])
        _quad(mesh, crest_low[i], crest_low[j], root_low[j], root_low[i])

    _quad(mesh, root_low[0], root_high[0], crest_high[0], crest_low[0])
    _quad(mesh, root_low[-1], crest_low[-1], crest_high[-1], root_high[-1])
    return mesh


def radial_box(
    part,
    name: str,
    theta: float,
    radius: float,
    z: float,
    size: tuple[float, float, float],
    material: Material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=(radius * cos(theta), radius * sin(theta), z), rpy=(0.0, 0.0, theta)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_screwcap_bottle")

    borosilicate = model.material("faint_blue_glass", rgba=(0.62, 0.82, 0.95, 0.34))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.02, 0.021, 0.023, 1.0))
    cap_clear = model.material("clear_cap_polymer", rgba=(0.78, 0.86, 0.94, 0.42))
    cap_blue = model.material("cap_thread_blue", rgba=(0.12, 0.32, 0.92, 1.0))
    datum_white = model.material("etched_white", rgba=(0.92, 0.94, 0.90, 1.0))
    warning_green = model.material("gap_green", rgba=(0.18, 0.72, 0.34, 1.0))
    gasket = model.material("white_ptfe_gasket", rgba=(0.94, 0.94, 0.88, 1.0))

    bottle = model.part("bottle")

    outer_profile = [
        (0.020, 0.000),
        (0.032, 0.008),
        (0.032, 0.113),
        (0.030, 0.125),
        (0.023, 0.139),
        (0.0148, 0.151),
        (0.0148, 0.197),
    ]
    inner_profile = [
        (0.024, 0.010),
        (0.028, 0.110),
        (0.026, 0.124),
        (0.019, 0.138),
        (0.0105, 0.151),
        (0.0105, 0.197),
    ]
    bottle.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=128,
                start_cap="flat",
                end_cap="flat",
            ),
            "hollow_bottle_body",
        ),
        material=borosilicate,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_geometry(annular_cylinder(0.0142, 0.0220, 0.147, 0.151), "bottle_datum_ring"),
        material=satin_steel,
        name="datum_ring",
    )
    bottle.visual(
        mesh_from_geometry(annular_cylinder(0.0100, 0.0170, 0.197, 0.199), "bottle_mouth_lip"),
        material=satin_steel,
        name="mouth_lip",
    )
    bottle.visual(
        mesh_from_geometry(
            helical_thread(0.0142, 0.0164, 0.161, 3.0, width=0.0020, phase=0.18),
            "bottle_external_thread",
        ),
        material=black_oxide,
        name="external_thread",
    )
    bottle.visual(
        mesh_from_geometry(annular_cylinder(0.0275, 0.0338, 0.004, 0.010), "bottle_base_ring"),
        material=satin_steel,
        name="base_ring",
    )
    bottle.visual(
        mesh_from_geometry(annular_cylinder(0.0144, 0.0162, 0.153, 0.156), "neck_reference_band"),
        material=warning_green,
        name="neck_reference_band",
    )

    for idx, theta in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        radial_box(
            bottle,
            f"datum_flat_{idx}",
            theta,
            0.0328,
            0.029,
            (0.0040, 0.0180, 0.0120),
            satin_steel,
        )

    radial_box(
        bottle,
        "fixed_index",
        0.0,
        0.0223,
        0.149,
        (0.0014, 0.0048, 0.0070),
        datum_white,
    )
    for idx, z in enumerate((0.1477, 0.1485, 0.1493, 0.1501, 0.1509)):
        radial_box(
            bottle,
            f"neck_scale_{idx}",
            0.0,
            0.0224,
            z,
            (0.0012, 0.0060 if idx == 2 else 0.0035, 0.00035),
            datum_white,
        )

    cap_axis = model.part("cap_axis")
    cap = model.part("cap")

    cap.visual(
        mesh_from_geometry(cap_shell(0.0205, 0.0240, 0.052, 0.005, segments=128), "cap_hollow_shell"),
        material=cap_clear,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_geometry(
            helical_thread(0.0208, 0.0177, 0.010, 3.0, width=0.0022, phase=0.18),
            "cap_internal_thread",
        ),
        material=cap_blue,
        name="internal_thread",
    )
    cap.visual(
        Cylinder(radius=0.0135, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0455)),
        material=gasket,
        name="seal_pad",
    )
    cap.visual(
        mesh_from_geometry(annular_cylinder(0.0234, 0.0251, 0.0015, 0.0050), "cap_lower_gap_band"),
        material=black_oxide,
        name="lower_gap_band",
    )
    cap.visual(
        mesh_from_geometry(annular_cylinder(0.0235, 0.0250, 0.037, 0.041), "cap_vernier_band"),
        material=black_oxide,
        name="vernier_band",
    )

    for idx in range(24):
        theta = 2.0 * pi * idx / 24.0
        radial_box(
            cap,
            f"grip_rib_{idx}",
            theta,
            0.02455,
            0.022,
            (0.0018, 0.0024, 0.026),
            satin_steel,
        )

    for idx in range(12):
        theta = 2.0 * pi * idx / 12.0
        radial_box(
            cap,
            f"vernier_tick_{idx}",
            theta,
            0.02535,
            0.0425,
            (0.0009, 0.0015 if idx % 3 else 0.0030, 0.009 if idx % 3 == 0 else 0.005),
            datum_white,
        )

    radial_box(
        cap,
        "moving_index",
        0.0,
        0.0254,
        0.030,
        (0.0010, 0.0044, 0.033),
        datum_white,
    )

    twist = model.articulation(
        "bottle_to_cap_axis",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=cap_axis,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=0.0, upper=CAP_ROTATION_LIMIT),
        motion_properties=MotionProperties(damping=0.02, friction=0.04),
    )
    model.articulation(
        "cap_axis_to_cap",
        ArticulationType.PRISMATIC,
        parent=cap_axis,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.025, lower=0.0, upper=CAP_LIFT_LIMIT),
        motion_properties=MotionProperties(damping=0.04, friction=0.08),
        meta={"screw_coupling": {"rotary_joint": twist.name, "pitch_m_per_turn": PITCH}},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    twist = object_model.get_articulation("bottle_to_cap_axis")
    slide = object_model.get_articulation("cap_axis_to_cap")

    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=0.0005,
        name="cap axis remains coaxial with bottle neck",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="datum_ring",
        min_gap=0.003,
        max_gap=0.0055,
        name="closed cap has controlled datum gap",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="seal_pad",
        negative_elem="mouth_lip",
        min_gap=0.0,
        max_gap=0.0002,
        name="seal pad seats on lip without interpenetration",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="internal_thread",
        elem_b="external_thread",
        min_overlap=0.020,
        name="internal and external threads are axially engaged",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="external_thread",
        outer_elem="cap_shell",
        margin=0.0005,
        name="neck thread stays inside cap envelope with clearance",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({twist: CAP_ROTATION_LIMIT, slide: CAP_LIFT_LIMIT}):
        raised_pos = ctx.part_world_position(cap)
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=0.0005,
            name="unscrewed cap remains coaxial",
        )
        ctx.expect_overlap(
            cap,
            bottle,
            axes="z",
            elem_a="internal_thread",
            elem_b="external_thread",
            min_overlap=0.010,
            name="threads remain partially engaged at one turn",
        )
    ctx.check(
        "cap lift follows screw pitch",
        rest_pos is not None
        and raised_pos is not None
        and abs((raised_pos[2] - rest_pos[2]) - PITCH) < 0.0008,
        details=f"rest={rest_pos}, raised={raised_pos}, pitch={PITCH}",
    )
    ctx.check(
        "axial lift joint records screw pitch",
        slide.meta.get("screw_coupling", {}).get("rotary_joint") == twist.name
        and abs(slide.meta.get("screw_coupling", {}).get("pitch_m_per_turn", 0.0) - PITCH) < 1e-9,
        details=f"meta={slide.meta}",
    )

    return ctx.report()


object_model = build_object_model()
