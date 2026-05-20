from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _wedge_geometry(length: float, bottom_width: float, top_width: float, high: float, low: float) -> MeshGeometry:
    """A solid, sloped dovetail elevation wedge.  Local +X is the forward slide direction."""
    x0 = -0.5 * length
    x1 = 0.5 * length
    yb = 0.5 * bottom_width
    yt = 0.5 * top_width
    g = MeshGeometry()
    verts = [
        (x0, -yb, 0.0),
        (x0, yb, 0.0),
        (x1, yb, 0.0),
        (x1, -yb, 0.0),
        (x0, -yt, high),
        (x0, yt, high),
        (x1, yt, low),
        (x1, -yt, low),
    ]
    for vx, vy, vz in verts:
        g.add_vertex(vx, vy, vz)
    for face in (
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (3, 7, 6),
        (3, 6, 2),
        (1, 2, 6),
        (1, 6, 5),
        (0, 4, 7),
        (0, 7, 3),
    ):
        g.add_face(*face)
    return g


def _dovetail_lip_geometry(sign: float) -> MeshGeometry:
    """Long sloping guide lip bolted to the inner face of a bed rail."""
    x0, x1 = -0.50, 0.50
    y_outer = sign * 0.335
    y_inner = sign * 0.185
    z_low = 0.185
    z_high = 0.270
    g = MeshGeometry()
    verts = [
        (x0, y_outer, z_low),
        (x0, y_outer, z_high),
        (x0, y_inner, z_high),
        (x1, y_outer, z_low),
        (x1, y_outer, z_high),
        (x1, y_inner, z_high),
    ]
    for vx, vy, vz in verts:
        g.add_vertex(vx, vy, vz)
    # two triangular end caps and three rectangular faces
    for face in (
        (0, 1, 2),
        (3, 5, 4),
        (0, 3, 4),
        (0, 4, 1),
        (1, 4, 5),
        (1, 5, 2),
        (2, 5, 3),
        (2, 3, 0),
    ):
        g.add_face(*face)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="siege_mortar_trestle_bed")

    oak = model.material("aged_oak", rgba=(0.46, 0.28, 0.13, 1.0))
    end_grain = model.material("end_grain", rgba=(0.56, 0.36, 0.18, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.05, 0.055, 0.055, 1.0))
    worn_iron = model.material("worn_iron", rgba=(0.22, 0.22, 0.21, 1.0))
    bore_dark = model.material("bore_shadow", rgba=(0.005, 0.005, 0.004, 1.0))
    brass = model.material("worn_bronze", rgba=(0.48, 0.36, 0.17, 1.0))

    bed = model.part("bed")

    # Low ground trestle: two long bed rails, cross ties, and four solid corner feet.
    rail_z = 0.125
    for y, name in ((0.38, "rail_0"), (-0.38, "rail_1")):
        bed.visual(Box((1.34, 0.12, 0.13)), origin=Origin(xyz=(0.0, y, rail_z)), material=oak, name=name)
        bed.visual(Box((1.36, 0.018, 0.145)), origin=Origin(xyz=(0.0, y * 0.84, rail_z + 0.003)), material=worn_iron, name=f"{name}_strap")

    for x, name in ((-0.56, "cross_tie_0"), (0.0, "cross_tie_1"), (0.56, "cross_tie_2")):
        bed.visual(Box((0.14, 0.88, 0.085)), origin=Origin(xyz=(x, 0.0, 0.075)), material=oak, name=name)
        bed.visual(Box((0.15, 0.92, 0.018)), origin=Origin(xyz=(x, 0.0, 0.126)), material=black_iron, name=f"{name}_band")

    for i, (x, y) in enumerate(((-0.58, 0.38), (-0.58, -0.38), (0.58, 0.38), (0.58, -0.38))):
        bed.visual(Box((0.22, 0.20, 0.055)), origin=Origin(xyz=(x, y, 0.028)), material=end_grain, name=f"foot_{i}")

    # Wooden side plates form open U-shaped trunnion cradles lined with iron rings.
    trunnion_z = 0.62
    for y, name in ((0.47, "side_plate_0"), (-0.47, "side_plate_1")):
        bed.visual(Box((1.24, 0.060, 0.255)), origin=Origin(xyz=(0.0, y, 0.296)), material=oak, name=f"{name}_lower")
        bed.visual(Box((0.165, 0.060, 0.335)), origin=Origin(xyz=(-0.190, y, 0.575)), material=oak, name=f"{name}_cheek_0")
        bed.visual(Box((0.165, 0.060, 0.335)), origin=Origin(xyz=(0.190, y, 0.575)), material=oak, name=f"{name}_cheek_1")
        bed.visual(Box((0.545, 0.060, 0.050)), origin=Origin(xyz=(0.0, y, 0.725)), material=oak, name=f"{name}_cap")
        bed.visual(Box((0.180, 0.030, 0.1325)), origin=Origin(xyz=(0.0, y, 0.48975)), material=black_iron, name=f"{name}_saddle")

    ring_mesh = mesh_from_geometry(TorusGeometry(0.108, 0.014, radial_segments=36, tubular_segments=16).rotate_x(pi / 2.0), "trunnion_liner_ring")
    bed.visual(ring_mesh, origin=Origin(xyz=(0.0, 0.512, trunnion_z)), material=worn_iron, name="trunnion_liner_0")
    bed.visual(ring_mesh, origin=Origin(xyz=(0.0, -0.512, trunnion_z)), material=worn_iron, name="trunnion_liner_1")

    # The central dovetail slot and its sloping retaining lips guide the sliding wedge.
    bed.visual(mesh_from_geometry(_dovetail_lip_geometry(1.0), "dovetail_lip_positive"), material=black_iron, name="dovetail_lip_0")
    bed.visual(mesh_from_geometry(_dovetail_lip_geometry(-1.0), "dovetail_lip_negative"), material=black_iron, name="dovetail_lip_1")
    bed.visual(Box((0.96, 0.018, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.145)), material=worn_iron, name="slot_wear_plate")

    barrel = model.part("barrel")

    # A squat siege-mortar tube: bronze/iron reinforced, with a visible hollow wide bore.
    outer_profile = [
        (0.165, -0.430),
        (0.245, -0.365),
        (0.270, -0.190),
        (0.222, 0.020),
        (0.230, 0.315),
        (0.285, 0.455),
    ]
    inner_profile = [
        (0.024, -0.270),
        (0.095, -0.230),
        (0.132, -0.040),
        (0.148, 0.455),
    ]
    barrel_shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    base_elevation = 0.19
    bore_rpy = (0.0, pi / 2.0 - base_elevation, 0.0)
    barrel.visual(mesh_from_geometry(barrel_shell, "short_wide_bore_barrel"), origin=Origin(rpy=bore_rpy), material=brass, name="barrel_shell")
    bore_liner = LatheGeometry.from_shell_profiles(
        [(0.150, -0.030), (0.150, 0.438)],
        [(0.136, -0.030), (0.136, 0.438)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    barrel.visual(mesh_from_geometry(bore_liner, "dark_bore_liner"), origin=Origin(rpy=bore_rpy), material=bore_dark, name="bore_liner")
    barrel.visual(Cylinder(radius=0.064, length=1.05), origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)), material=worn_iron, name="trunnion_pin")
    barrel.visual(Cylinder(radius=0.088, length=0.075), origin=Origin(xyz=(0.0, 0.545, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=worn_iron, name="trunnion_collar_0")
    barrel.visual(Cylinder(radius=0.088, length=0.075), origin=Origin(xyz=(0.0, -0.545, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=worn_iron, name="trunnion_collar_1")
    barrel.visual(Sphere(radius=0.090), origin=Origin(xyz=(-0.420 * cos(base_elevation), 0.0, -0.420 * sin(base_elevation))), material=brass, name="breech_button")

    wedge = model.part("wedge")
    wedge.visual(
        mesh_from_geometry(_wedge_geometry(0.56, 0.300, 0.220, high=0.120, low=0.045), "sliding_elevation_wedge"),
        origin=Origin(),
        material=oak,
        name="wedge_body",
    )
    wedge.visual(Box((0.026, 0.250, 0.092)), origin=Origin(xyz=(0.292, 0.0, 0.070)), material=black_iron, name="strike_plate")
    wedge.visual(Cylinder(radius=0.018, length=0.220), origin=Origin(xyz=(0.315, 0.0, 0.122), rpy=(-pi / 2.0, 0.0, 0.0)), material=worn_iron, name="pull_bar")

    model.articulation(
        "barrel_trunnion",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, trunnion_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.42),
    )

    model.articulation(
        "wedge_slide",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=wedge,
        origin=Origin(xyz=(-0.135, 0.0, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.20, lower=0.0, upper=0.245),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    barrel = object_model.get_part("barrel")
    wedge = object_model.get_part("wedge")
    barrel_trunnion = object_model.get_articulation("barrel_trunnion")
    wedge_slide = object_model.get_articulation("wedge_slide")

    ctx.expect_within(
        wedge,
        bed,
        axes="y",
        inner_elem="wedge_body",
        outer_elem="dovetail_lip_0",
        margin=0.36,
        name="wedge rides between the bed rails",
    )
    ctx.expect_overlap(
        wedge,
        bed,
        axes="x",
        elem_a="wedge_body",
        elem_b="slot_wear_plate",
        min_overlap=0.30,
        name="wedge remains engaged in the fore-aft slot",
    )

    rest_barrel_aabb = ctx.part_world_aabb(barrel)
    rest_wedge_pos = ctx.part_world_position(wedge)
    with ctx.pose({barrel_trunnion: 0.36, wedge_slide: 0.20}):
        elevated_barrel_aabb = ctx.part_world_aabb(barrel)
        advanced_wedge_pos = ctx.part_world_position(wedge)
        ctx.expect_overlap(
            wedge,
            bed,
            axes="x",
            elem_a="wedge_body",
            elem_b="slot_wear_plate",
            min_overlap=0.25,
            name="advanced wedge stays captured by the slot",
        )

    ctx.check(
        "barrel tilts upward on its trunnions",
        rest_barrel_aabb is not None
        and elevated_barrel_aabb is not None
        and elevated_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.10,
        details=f"rest={rest_barrel_aabb}, elevated={elevated_barrel_aabb}",
    )
    ctx.check(
        "elevation wedge slides fore-aft",
        rest_wedge_pos is not None and advanced_wedge_pos is not None and advanced_wedge_pos[0] > rest_wedge_pos[0] + 0.15,
        details=f"rest={rest_wedge_pos}, advanced={advanced_wedge_pos}",
    )

    return ctx.report()


object_model = build_object_model()
