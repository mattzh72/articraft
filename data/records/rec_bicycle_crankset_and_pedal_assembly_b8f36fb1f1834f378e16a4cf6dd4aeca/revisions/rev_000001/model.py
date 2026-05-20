from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _extruded_loop_x(
    yz_points: list[tuple[float, float]],
    *,
    x_center: float,
    thickness: float,
) -> MeshGeometry:
    """Extrude a closed YZ profile along local/global X."""
    geom = MeshGeometry()
    half = thickness / 2.0
    front: list[int] = []
    back: list[int] = []
    for y, z in yz_points:
        front.append(geom.add_vertex(x_center + half, y, z))
    for y, z in yz_points:
        back.append(geom.add_vertex(x_center - half, y, z))

    n = len(yz_points)
    # Side faces.
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], front[j], back[j])
        geom.add_face(front[i], back[j], back[i])

    # Fan caps.  Profiles used here are convex or nearly convex.
    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    return geom


def _tube_x_mesh(
    *,
    x_center: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = 72,
) -> MeshGeometry:
    """A hollow cylinder/tube with its axis along X."""
    geom = MeshGeometry()
    x0 = x_center - length / 2.0
    x1 = x_center + length / 2.0
    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        outer0.append(geom.add_vertex(x0, outer_radius * ca, outer_radius * sa))
        outer1.append(geom.add_vertex(x1, outer_radius * ca, outer_radius * sa))
        inner0.append(geom.add_vertex(x0, inner_radius * ca, inner_radius * sa))
        inner1.append(geom.add_vertex(x1, inner_radius * ca, inner_radius * sa))

    for i in range(segments):
        j = (i + 1) % segments
        # Outside skin.
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # Inside bore.
        geom.add_face(inner0[i], inner1[j], inner0[j])
        geom.add_face(inner0[i], inner1[i], inner1[j])
        # End annuli.
        geom.add_face(outer0[i], inner0[j], outer0[j])
        geom.add_face(outer0[i], inner0[i], inner0[j])
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])
    return geom


def _toothed_chainring_mesh(
    *,
    x_center: float,
    thickness: float,
    tooth_count: int,
    tip_radius: float,
    root_radius: float,
    inner_radius: float,
) -> MeshGeometry:
    """Flat chainring in the YZ plane with alternating tooth-tip/root radii."""
    geom = MeshGeometry()
    n = tooth_count * 2
    xf = x_center + thickness / 2.0
    xb = x_center - thickness / 2.0
    outer_f: list[int] = []
    outer_b: list[int] = []
    inner_f: list[int] = []
    inner_b: list[int] = []

    for i in range(n):
        a = 2.0 * math.pi * i / n
        # Narrow, pointed teeth with valleys between them.
        r = tip_radius if i % 2 == 0 else root_radius
        ca, sa = math.cos(a), math.sin(a)
        outer_f.append(geom.add_vertex(xf, r * ca, r * sa))
        outer_b.append(geom.add_vertex(xb, r * ca, r * sa))
        inner_f.append(geom.add_vertex(xf, inner_radius * ca, inner_radius * sa))
        inner_b.append(geom.add_vertex(xb, inner_radius * ca, inner_radius * sa))

    for i in range(n):
        j = (i + 1) % n
        # Front and back annular faces.
        geom.add_face(outer_f[i], outer_f[j], inner_f[j])
        geom.add_face(outer_f[i], inner_f[j], inner_f[i])
        geom.add_face(outer_b[i], inner_b[j], outer_b[j])
        geom.add_face(outer_b[i], inner_b[i], inner_b[j])
        # Outer tooth wall and inner bore wall.
        geom.add_face(outer_f[i], outer_b[j], outer_f[j])
        geom.add_face(outer_f[i], outer_b[i], outer_b[j])
        geom.add_face(inner_f[i], inner_f[j], inner_b[j])
        geom.add_face(inner_f[i], inner_b[j], inner_b[i])
    return geom


def _radial_spoke_mesh(
    *,
    angle: float,
    x_center: float,
    thickness: float,
    r0: float,
    r1: float,
    width0: float,
    width1: float,
) -> MeshGeometry:
    """Tapered spider arm profile in the YZ plane, extruded along X."""
    ca, sa = math.cos(angle), math.sin(angle)
    # radial and perpendicular unit vectors in the YZ plane
    ry, rz = ca, sa
    py, pz = -sa, ca

    def pt(radius: float, half_width: float, sign: float) -> tuple[float, float]:
        return (
            radius * ry + sign * half_width * py,
            radius * rz + sign * half_width * pz,
        )

    profile = [
        pt(r0, width0 / 2.0, -1.0),
        pt(r1, width1 / 2.0, -1.0),
        pt(r1, width1 / 2.0, 1.0),
        pt(r0, width0 / 2.0, 1.0),
    ]
    return _extruded_loop_x(profile, x_center=x_center, thickness=thickness)


def _crank_arm_mesh(
    *,
    x_center: float,
    thickness: float,
    y_start: float,
    y_end: float,
    width_start: float,
    width_end: float,
) -> MeshGeometry:
    """A long forged crank arm, tapered in Z and extruded along X."""
    sign = 1.0 if y_end > y_start else -1.0
    cap_len = 0.012 * sign
    profile = [
        (y_start - cap_len, -width_start / 2.0),
        (y_start + 0.010 * sign, -width_start / 2.0),
        (y_end - 0.012 * sign, -width_end / 2.0),
        (y_end + 0.010 * sign, -width_end / 2.0),
        (y_end + 0.016 * sign, 0.0),
        (y_end + 0.010 * sign, width_end / 2.0),
        (y_end - 0.012 * sign, width_end / 2.0),
        (y_start + 0.010 * sign, width_start / 2.0),
        (y_start - cap_len, width_start / 2.0),
        (y_start - 0.018 * sign, 0.0),
    ]
    return _extruded_loop_x(profile, x_center=x_center, thickness=thickness)


def _groove_mesh(
    *,
    x_center: float,
    y_start: float,
    y_end: float,
    z_half_width: float,
    thickness: float = 0.0012,
) -> MeshGeometry:
    """Thin dark inset panel on the crank-arm face."""
    profile = [
        (y_start, -z_half_width),
        (y_end, -z_half_width * 0.72),
        (y_end, z_half_width * 0.72),
        (y_start, z_half_width),
    ]
    return _extruded_loop_x(profile, x_center=x_center, thickness=thickness)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_triple_crankset")

    forged_alloy = model.material("brushed_forged_alloy", rgba=(0.78, 0.76, 0.70, 1.0))
    polished = model.material("polished_worn_edges", rgba=(0.92, 0.90, 0.84, 1.0))
    dark_anodized = model.material("black_anodized_aluminum", rgba=(0.015, 0.016, 0.018, 1.0))
    gunmetal = model.material("gunmetal_chainring", rgba=(0.30, 0.31, 0.32, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.02, 0.02, 0.022, 1.0))
    seal_blue = model.material("rubber_seal_blue", rgba=(0.03, 0.09, 0.16, 1.0))
    bolt_steel = model.material("stainless_bolts", rgba=(0.72, 0.72, 0.68, 1.0))
    recess_black = model.material("matte_black_recess", rgba=(0.0, 0.0, 0.0, 1.0))

    # Root: stationary outboard-bearing unit with visible hollow cups.
    bearing_unit = model.part("bearing_unit")
    bearing_unit.visual(
        mesh_from_geometry(
            _tube_x_mesh(
                x_center=0.0,
                length=0.044,
                outer_radius=0.018,
                inner_radius=0.0137,
                segments=80,
            ),
            "center_sleeve",
        ),
        material=bearing_black,
        name="center_sleeve",
    )
    for side, suffix in ((1.0, "drive"), (-1.0, "left")):
        bearing_unit.visual(
            mesh_from_geometry(
                _tube_x_mesh(
                    x_center=side * 0.034,
                    length=0.024,
                    outer_radius=0.028,
                    inner_radius=0.0137,
                    segments=96,
                ),
                f"{suffix}_bearing_cup",
            ),
            material=dark_anodized,
            name=f"{suffix}_bearing_cup",
        )
        bearing_unit.visual(
            mesh_from_geometry(
                _tube_x_mesh(
                    x_center=side * 0.047,
                    length=0.0025,
                    outer_radius=0.020,
                    inner_radius=0.0137,
                    segments=80,
                ),
                f"{suffix}_bearing_seal",
            ),
            material=seal_blue,
            name=f"{suffix}_bearing_seal",
        )

    # Child rotating assembly: hollow spindle, cranks, forged spider, triple rings.
    crankset = model.part("crankset")
    crankset.visual(
        mesh_from_geometry(
            _tube_x_mesh(
                x_center=0.0,
                length=0.164,
                outer_radius=0.012,
                inner_radius=0.0065,
                segments=72,
            ),
            "hollow_spindle",
        ),
        material=dark_anodized,
        name="hollow_spindle",
    )

    # Three differently offset and sized chainrings (outer/middle/granny).
    ring_specs = [
        ("outer_chainring", 50, 0.105, 0.100, 0.075, 0.052, gunmetal),
        ("middle_chainring", 39, 0.084, 0.080, 0.057, 0.046, forged_alloy),
        ("inner_chainring", 30, 0.064, 0.060, 0.040, 0.040, gunmetal),
    ]
    for name, tooth_count, tip_r, root_r, inner_r, x, mat in ring_specs:
        crankset.visual(
            mesh_from_geometry(
                _toothed_chainring_mesh(
                    x_center=x,
                    thickness=0.0032,
                    tooth_count=tooth_count,
                    tip_radius=tip_r,
                    root_radius=root_r,
                    inner_radius=inner_r,
                ),
                name,
            ),
            material=mat,
            name=name,
        )

    # Forged five-arm spider and chainring bolts.
    spider_angles = [math.pi + 2.0 * math.pi * i / 5.0 for i in range(5)]
    for idx, angle in enumerate(spider_angles):
        crankset.visual(
            mesh_from_geometry(
                _radial_spoke_mesh(
                    angle=angle,
                    x_center=0.054,
                    thickness=0.012,
                    r0=0.021,
                    r1=0.079,
                    width0=0.020,
                    width1=0.014,
                ),
                f"spider_arm_{idx}",
            ),
            material=forged_alloy,
            name=f"spider_arm_{idx}",
        )
        y = 0.079 * math.cos(angle)
        z = 0.079 * math.sin(angle)
        crankset.visual(
            Cylinder(radius=0.0085, length=0.024),
            origin=Origin(xyz=(0.050, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"chainring_bolt_{idx}",
        )
        crankset.visual(
            Cylinder(radius=0.0048, length=0.006),
            origin=Origin(xyz=(0.0615, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=recess_black,
            name=f"bolt_hex_socket_{idx}",
        )
        crankset.visual(
            Cylinder(radius=0.0042, length=0.014),
            origin=Origin(
                xyz=(0.044, 0.052 * math.cos(angle), 0.052 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_steel,
            name=f"inner_ring_standoff_{idx}",
        )

    crankset.visual(
        Cylinder(radius=0.027, length=0.028),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="drive_hub",
    )
    crankset.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="crank_spline_cover",
    )

    # Right/drive crank arm with a black relief groove and pedal eye.
    crankset.visual(
        mesh_from_geometry(
            _crank_arm_mesh(
                x_center=0.071,
                thickness=0.016,
                y_start=-0.018,
                y_end=-0.172,
                width_start=0.034,
                width_end=0.021,
            ),
            "drive_crank_arm",
        ),
        material=forged_alloy,
        name="drive_crank_arm",
    )
    crankset.visual(
        mesh_from_geometry(
            _groove_mesh(x_center=0.0796, y_start=-0.045, y_end=-0.145, z_half_width=0.0055),
            "drive_arm_groove",
        ),
        material=recess_black,
        name="drive_arm_groove",
    )
    crankset.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.071, -0.172, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="drive_pedal_eye",
    )

    # Opposed left arm with clamp features and the preload cap at the spindle end.
    crankset.visual(
        mesh_from_geometry(
            _crank_arm_mesh(
                x_center=-0.064,
                thickness=0.016,
                y_start=0.018,
                y_end=0.172,
                width_start=0.034,
                width_end=0.021,
            ),
            "left_crank_arm",
        ),
        material=forged_alloy,
        name="left_crank_arm",
    )
    crankset.visual(
        mesh_from_geometry(
            _groove_mesh(x_center=-0.0726, y_start=0.046, y_end=0.145, z_half_width=0.0055),
            "left_arm_groove",
        ),
        material=recess_black,
        name="left_arm_groove",
    )
    crankset.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(-0.067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="left_clamp_boss",
    )
    crankset.visual(
        Box((0.0014, 0.033, 0.004)),
        origin=Origin(xyz=(-0.0764, 0.012, 0.0)),
        material=recess_black,
        name="clamp_split_slot",
    )
    crankset.visual(
        Box((0.011, 0.014, 0.036)),
        origin=Origin(xyz=(-0.067, 0.027, 0.0)),
        material=forged_alloy,
        name="clamp_lug",
    )
    for z in (-0.011, 0.011):
        crankset.visual(
            Cylinder(radius=0.0048, length=0.008),
            origin=Origin(xyz=(-0.075, 0.029, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"pinch_bolt_{'lower' if z < 0 else 'upper'}",
        )
    crankset.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(-0.0785, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=recess_black,
        name="preload_cap",
    )
    crankset.visual(
        Cylinder(radius=0.0065, length=0.0015),
        origin=Origin(xyz=(-0.082, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="preload_cap_socket",
    )
    crankset.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(-0.064, 0.172, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="left_pedal_eye",
    )

    spindle_joint = model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=bearing_unit,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )
    spindle_joint.meta["qc_samples"] = [0.0, math.pi / 2.0, math.pi]

    def add_pedal(name: str, side: float, y: float) -> None:
        pedal = model.part(name)
        body_center_x = side * 0.058
        pedal.visual(
            Cylinder(radius=0.0048, length=0.092),
            origin=Origin(xyz=(side * 0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name="pedal_axle",
        )
        pedal.visual(
            Box((0.074, 0.060, 0.014)),
            origin=Origin(xyz=(body_center_x, 0.0, 0.0)),
            material=dark_anodized,
            name="pedal_body",
        )
        # Top-face cleat-platform recess and retention jaws.
        pedal.visual(
            Box((0.046, 0.030, 0.0012)),
            origin=Origin(xyz=(body_center_x, 0.0, 0.0076)),
            material=recess_black,
            name="cleat_recess",
        )
        pedal.visual(
            Box((0.054, 0.006, 0.004)),
            origin=Origin(xyz=(body_center_x, 0.022, 0.009)),
            material=polished,
            name="front_cleat_jaw",
        )
        pedal.visual(
            Box((0.054, 0.006, 0.004)),
            origin=Origin(xyz=(body_center_x, -0.022, 0.009)),
            material=polished,
            name="rear_cleat_jaw",
        )
        # Black side windows make the small pedal read as lightweight/open.
        pedal.visual(
            Box((0.050, 0.0015, 0.006)),
            origin=Origin(xyz=(body_center_x, 0.03075, 0.0)),
            material=recess_black,
            name="front_window",
        )
        pedal.visual(
            Box((0.050, 0.0015, 0.006)),
            origin=Origin(xyz=(body_center_x, -0.03075, 0.0)),
            material=recess_black,
            name="rear_window",
        )

        joint = model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=crankset,
            child=pedal,
            origin=Origin(xyz=((0.081 if side > 0.0 else -0.074), y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=22.0),
        )
        joint.meta["qc_samples"] = [0.0, math.pi / 2.0]

    add_pedal("drive_pedal", 1.0, -0.172)
    add_pedal("left_pedal", -1.0, 0.172)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crankset = object_model.get_part("crankset")
    bearing_unit = object_model.get_part("bearing_unit")
    drive_pedal = object_model.get_part("drive_pedal")
    left_pedal = object_model.get_part("left_pedal")
    spindle_spin = object_model.get_articulation("spindle_spin")

    # The three chainrings should be concentric around the spindle but visibly
    # different in diameter.
    ctx.expect_within(
        crankset,
        crankset,
        axes="yz",
        inner_elem="inner_chainring",
        outer_elem="outer_chainring",
        margin=0.002,
        name="small ring nests within big ring projection",
    )
    ctx.expect_overlap(
        crankset,
        crankset,
        axes="yz",
        elem_a="outer_chainring",
        elem_b="middle_chainring",
        min_overlap=0.10,
        name="chainrings share a concentric plane",
    )
    ctx.expect_within(
        crankset,
        bearing_unit,
        axes="yz",
        inner_elem="hollow_spindle",
        outer_elem="drive_bearing_cup",
        margin=0.002,
        name="spindle passes through bearing bore",
    )

    # Pedal axles are mounted at the crank eyes and move with the crankset.
    ctx.expect_gap(
        drive_pedal,
        crankset,
        axis="x",
        positive_elem="pedal_axle",
        negative_elem="drive_pedal_eye",
        min_gap=-0.001,
        max_gap=0.006,
        name="drive pedal axle starts at pedal eye",
    )
    ctx.expect_gap(
        crankset,
        left_pedal,
        axis="x",
        positive_elem="left_pedal_eye",
        negative_elem="pedal_axle",
        min_gap=-0.001,
        max_gap=0.006,
        name="left pedal axle starts at pedal eye",
    )

    rest_aabb = ctx.part_world_aabb(crankset)
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(crankset)
    ctx.check(
        "crankset rotates about outboard spindle",
        rest_aabb is not None
        and turned_aabb is not None
        and abs((rest_aabb[1][2] - rest_aabb[0][2]) - (turned_aabb[1][1] - turned_aabb[0][1])) < 0.04,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
