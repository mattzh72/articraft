from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


THREAD_PITCH = 0.016
SCREW_TRAVEL = 1.55 * math.tau
CAP_LIFT = THREAD_PITCH * SCREW_TRAVEL / math.tau


def _save_mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _merge_meshes(meshes: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for mesh in meshes:
        merged.merge(mesh)
    return merged


def _helix_points(
    *,
    radius: float,
    z0: float,
    height: float,
    turns: float,
    phase: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        angle = phase + turns * math.tau * t
        points.append((radius * math.cos(angle), radius * math.sin(angle), z0 + height * t))
    return points


def _thread_pair_mesh(
    *,
    radius: float,
    z0: float,
    height: float,
    turns: float,
    tube_radius: float,
    name: str,
) -> MeshGeometry:
    # Two coarse starts make the interface read like a field-service cap thread
    # rather than a fine consumer bottle thread.
    meshes = []
    samples = max(72, int(turns * 48))
    for phase in (0.0, math.pi):
        path = _helix_points(
            radius=radius,
            z0=z0,
            height=height,
            turns=turns,
            phase=phase,
            samples=samples,
        )
        meshes.append(
            tube_from_spline_points(
                path,
                radius=tube_radius,
                samples_per_segment=1,
                radial_segments=10,
                cap_ends=True,
            )
        )
    return _merge_meshes(meshes)


def _cap_shell_mesh() -> MeshGeometry:
    outer_radius = 0.070
    inner_radius = 0.054
    height = 0.145
    inner_top = 0.128
    window_z0 = 0.032
    window_z1 = 0.108
    window_width = math.radians(56.0)
    radial_segments = 96
    z_levels = [0.0, window_z0, window_z1, inner_top, height]
    inner_levels = z_levels[:-1]

    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []

    for z in z_levels:
        ring = []
        for i in range(radial_segments):
            angle = math.tau * i / radial_segments
            ring.append(geom.add_vertex(outer_radius * math.cos(angle), outer_radius * math.sin(angle), z))
        outer.append(ring)

    for z in inner_levels:
        ring = []
        for i in range(radial_segments):
            angle = math.tau * i / radial_segments
            ring.append(geom.add_vertex(inner_radius * math.cos(angle), inner_radius * math.sin(angle), z))
        inner.append(ring)

    def in_window(theta: float, z_mid: float) -> bool:
        if not (window_z0 < z_mid < window_z1):
            return False
        for center in (0.0, math.pi):
            delta = abs((theta - center + math.pi) % math.tau - math.pi)
            if delta < window_width / 2.0:
                return True
        return False

    # Outer and inner cylindrical walls, with two service inspection windows cut
    # through the skirt so the mating helical threads are visible.
    for j in range(len(z_levels) - 1):
        z_mid = 0.5 * (z_levels[j] + z_levels[j + 1])
        for i in range(radial_segments):
            nxt = (i + 1) % radial_segments
            theta_mid = math.tau * (i + 0.5) / radial_segments
            if not in_window(theta_mid, z_mid):
                geom.add_face(outer[j][i], outer[j][nxt], outer[j + 1][nxt])
                geom.add_face(outer[j][i], outer[j + 1][nxt], outer[j + 1][i])
            if j < len(inner_levels) - 1 and not in_window(theta_mid, z_mid):
                geom.add_face(inner[j][i], inner[j + 1][nxt], inner[j][nxt])
                geom.add_face(inner[j][i], inner[j + 1][i], inner[j + 1][nxt])

    # Bottom rim and underside shoulder tie the inner and outer walls into one
    # continuous cap shell.
    for level_index in (0, len(inner_levels) - 1):
        for i in range(radial_segments):
            nxt = (i + 1) % radial_segments
            geom.add_face(outer[level_index][i], inner[level_index][i], inner[level_index][nxt])
            geom.add_face(outer[level_index][i], inner[level_index][nxt], outer[level_index][nxt])

    # Closed top disk and an internal ceiling over the mouth cavity.
    top_center = geom.add_vertex(0.0, 0.0, height)
    for i in range(radial_segments):
        nxt = (i + 1) % radial_segments
        geom.add_face(top_center, outer[-1][i], outer[-1][nxt])

    ceiling_center = geom.add_vertex(0.0, 0.0, inner_top)
    ceiling_ring = inner[-1]
    for i in range(radial_segments):
        nxt = (i + 1) % radial_segments
        geom.add_face(ceiling_center, ceiling_ring[nxt], ceiling_ring[i])

    return geom


def _hex_profile(radius: float) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.tau * index / 6.0 + math.pi / 6.0),
            radius * math.sin(math.tau * index / 6.0 + math.pi / 6.0),
        )
        for index in range(6)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_screwcap_bottle")

    bottle_polymer = model.material("olive_hdpe", rgba=(0.34, 0.39, 0.27, 1.0))
    cap_polymer = model.material("graphite_cap", rgba=(0.10, 0.12, 0.13, 0.96))
    rubber = model.material("black_rubber", rgba=(0.025, 0.026, 0.024, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    bronze = model.material("replaceable_bronze", rgba=(0.78, 0.52, 0.22, 1.0))
    gasket_blue = model.material("service_gasket_blue", rgba=(0.03, 0.20, 0.84, 1.0))
    warning_red = model.material("red_witness_mark", rgba=(0.85, 0.06, 0.03, 1.0))

    bottle = model.part("bottle")
    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.054, 0.000),
            (0.084, 0.018),
            (0.096, 0.060),
            (0.099, 0.330),
            (0.090, 0.395),
            (0.067, 0.455),
            (0.047, 0.490),
            (0.043, 0.505),
        ],
        [
            (0.000, 0.012),
            (0.062, 0.026),
            (0.083, 0.070),
            (0.086, 0.325),
            (0.077, 0.390),
            (0.054, 0.448),
            (0.034, 0.490),
            (0.034, 0.505),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    bottle.visual(_save_mesh(bottle_shell, "bottle_hollow_shell"), material=bottle_polymer, name="bottle_shell")
    bottle.visual(
        _save_mesh(TorusGeometry(radius=0.078, tube=0.010, radial_segments=16, tubular_segments=80), "base_bumper"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=rubber,
        name="base_bumper",
    )
    bottle.visual(
        Cylinder(radius=0.087, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=rubber,
        name="base_pad",
    )
    # Bolt-on wear rails protect the bottle when it is dragged across a service truck bed.
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = 0.100 * math.cos(angle)
        y = 0.100 * math.sin(angle)
        bottle.visual(
            Box((0.012, 0.024, 0.285)),
            origin=Origin(xyz=(x, y, 0.230), rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"wear_rail_{index}",
        )
        for z in (0.115, 0.230, 0.345):
            bottle.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(0.103 * math.cos(angle), 0.103 * math.sin(angle), z), rpy=(0.0, math.pi / 2.0, angle)),
                material=stainless,
                name=f"rail_bolt_{index}_{int(z * 1000):03d}",
            )
    bottle.visual(
        Box((0.115, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.100, 0.315)),
        material=stainless,
        name="service_label_plate",
    )

    neck_thread = model.part("neck_thread")
    neck_sleeve = LatheGeometry.from_shell_profiles(
        [
            (0.042, 0.505),
            (0.043, 0.522),
            (0.043, 0.612),
            (0.038, 0.628),
        ],
        [
            (0.030, 0.505),
            (0.031, 0.522),
            (0.031, 0.612),
            (0.031, 0.628),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    neck_thread.visual(_save_mesh(neck_sleeve, "neck_thread_sleeve"), material=bronze, name="thread_sleeve")
    neck_thread.visual(
        _save_mesh(
            _thread_pair_mesh(
                radius=0.0438,
                z0=0.524,
                height=0.080,
                turns=0.080 / THREAD_PITCH,
                tube_radius=0.0024,
                name="external_thread",
            ),
            "external_thread",
        ),
        material=bronze,
        name="external_thread",
    )
    neck_thread.visual(
        _save_mesh(TorusGeometry(radius=0.036, tube=0.0035, radial_segments=12, tubular_segments=48), "mouth_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.629)),
        material=stainless,
        name="mouth_lip",
    )

    cap_axis = model.part("cap_axis")

    cap = model.part("cap")
    cap.visual(_save_mesh(_cap_shell_mesh(), "cap_windowed_shell"), material=cap_polymer, name="cap_shell")
    for index in range(18):
        angle = math.tau * index / 18.0
        # Leave the two inspection windows less obstructed by skipping the
        # strongest lug directly over each window center.
        if index in (0, 9):
            continue
        cap.visual(
            Box((0.012, 0.012, 0.100)),
            origin=Origin(
                xyz=(0.073 * math.cos(angle), 0.073 * math.sin(angle), 0.062),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"grip_lug_{index:02d}",
        )
    cap.visual(
        _save_mesh(LatheGeometry([(0.0, 0.145), (0.025, 0.145), (0.025, 0.154), (0.0, 0.154)], segments=6), "wrench_hex"),
        material=cap_polymer,
        name="wrench_hex",
    )
    cap.visual(
        Box((0.003, 0.034, 0.018)),
        origin=Origin(xyz=(0.0715, 0.0, 0.121)),
        material=warning_red,
        name="witness_stripe",
    )
    cap.visual(
        Box((0.036, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=stainless,
        name="wrench_arrow",
    )

    cap_insert = model.part("cap_insert")
    insert_sleeve = LatheGeometry.from_shell_profiles(
        [
            (0.0508, 0.030),
            (0.0508, 0.126),
        ],
        [
            (0.0476, 0.030),
            (0.0476, 0.126),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    cap_insert.visual(_save_mesh(insert_sleeve, "cap_insert_sleeve"), material=bronze, name="insert_sleeve")
    cap_insert.visual(
        _save_mesh(
            _thread_pair_mesh(
                radius=0.0512,
                z0=0.033,
                height=0.080,
                turns=0.080 / THREAD_PITCH,
                tube_radius=0.0019,
                name="internal_thread",
            ),
            "internal_thread",
        ),
        material=bronze,
        name="internal_thread",
    )
    for index in range(4):
        angle = math.tau * index / 4.0 + math.pi / 4.0
        cap_insert.visual(
            Box((0.004, 0.014, 0.015)),
            origin=Origin(
                xyz=(0.0524 * math.cos(angle), 0.0524 * math.sin(angle), 0.102),
                rpy=(0.0, 0.0, angle),
            ),
            material=bronze,
            name=f"insert_tab_{index}",
        )

    cap_gasket = model.part("cap_gasket")
    cap_gasket.visual(
        _save_mesh(TorusGeometry(radius=0.0430, tube=0.0058, radial_segments=12, tubular_segments=56), "cap_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=gasket_blue,
        name="seal",
    )

    model.articulation(
        "bottle_to_thread",
        ArticulationType.FIXED,
        parent=bottle,
        child=neck_thread,
        origin=Origin(),
    )
    model.articulation(
        "cap_screw",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=cap_axis,
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=0.0, upper=SCREW_TRAVEL),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=cap_axis,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.06, lower=0.0, upper=CAP_LIFT),
        meta={"paired_pitch_m_per_rad": THREAD_PITCH / math.tau},
    )
    model.articulation(
        "cap_to_insert",
        ArticulationType.FIXED,
        parent=cap,
        child=cap_insert,
        origin=Origin(),
    )
    model.articulation(
        "cap_to_gasket",
        ArticulationType.FIXED,
        parent=cap,
        child=cap_gasket,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    neck_thread = object_model.get_part("neck_thread")
    cap = object_model.get_part("cap")
    cap_insert = object_model.get_part("cap_insert")
    cap_gasket = object_model.get_part("cap_gasket")
    screw = object_model.get_articulation("cap_screw")
    lift = object_model.get_articulation("cap_lift")

    for index in range(4):
        ctx.allow_overlap(
            cap,
            cap_insert,
            elem_a="cap_shell",
            elem_b=f"insert_tab_{index}",
            reason="Replaceable bronze thread insert tabs are intentionally seated in matching cap pockets.",
        )
        ctx.expect_overlap(
            cap_insert,
            cap,
            axes="z",
            min_overlap=0.010,
            elem_a=f"insert_tab_{index}",
            elem_b="cap_shell",
            name=f"insert tab {index} is captured by cap shell height",
        )
    ctx.allow_overlap(
        cap_gasket,
        neck_thread,
        elem_a="seal",
        elem_b="mouth_lip",
        reason="The blue service gasket is intentionally compressed against the stainless mouth lip at the closed service pose.",
    )
    ctx.allow_overlap(
        cap_gasket,
        cap_insert,
        elem_a="seal",
        elem_b="insert_sleeve",
        reason="The replaceable gasket is seated in the bronze insert's retaining groove.",
    )

    ctx.expect_contact(
        bottle,
        neck_thread,
        elem_a="bottle_shell",
        elem_b="thread_sleeve",
        contact_tol=0.002,
        name="replaceable neck thread sleeve is seated on bottle mouth",
    )
    ctx.expect_within(
        neck_thread,
        cap,
        axes="xy",
        margin=0.004,
        outer_elem="cap_shell",
        name="cap shell remains coaxial around threaded neck",
    )
    ctx.expect_overlap(
        cap_insert,
        neck_thread,
        axes="z",
        min_overlap=0.070,
        elem_a="internal_thread",
        elem_b="external_thread",
        name="internal and external helical threads share engaged height",
    )
    ctx.expect_gap(
        cap_gasket,
        neck_thread,
        axis="z",
        max_penetration=0.011,
        positive_elem="seal",
        negative_elem="mouth_lip",
        name="closed gasket compression stays shallow and local",
    )
    ctx.expect_gap(
        cap_gasket,
        cap_insert,
        axis="z",
        max_penetration=0.008,
        positive_elem="seal",
        negative_elem="insert_sleeve",
        name="gasket is captured by the insert groove",
    )

    rest_cap = ctx.part_world_position(cap)
    rest_aabb = ctx.part_element_world_aabb(cap, elem="witness_stripe")
    with ctx.pose({screw: SCREW_TRAVEL, lift: CAP_LIFT}):
        lifted_cap = ctx.part_world_position(cap)
        lifted_aabb = ctx.part_element_world_aabb(cap, elem="witness_stripe")
        ctx.expect_overlap(
            cap_insert,
            neck_thread,
            axes="z",
            min_overlap=0.030,
            elem_a="internal_thread",
            elem_b="external_thread",
            name="unscrewed cap still retains thread engagement",
        )

    rest_stripe_y = None if rest_aabb is None else 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
    lifted_stripe_y = None if lifted_aabb is None else 0.5 * (lifted_aabb[0][1] + lifted_aabb[1][1])
    ctx.check(
        "screw motion lifts the cap by thread pitch",
        rest_cap is not None
        and lifted_cap is not None
        and lifted_cap[2] > rest_cap[2] + CAP_LIFT - 0.002,
        details=f"rest={rest_cap}, lifted={lifted_cap}, expected_lift={CAP_LIFT:.4f}",
    )
    ctx.check(
        "witness stripe rotates around the common screw axis",
        rest_stripe_y is not None
        and lifted_stripe_y is not None
        and abs(lifted_stripe_y - rest_stripe_y) > 0.020,
        details=f"rest_y={rest_stripe_y}, lifted_y={lifted_stripe_y}",
    )

    return ctx.report()


object_model = build_object_model()
