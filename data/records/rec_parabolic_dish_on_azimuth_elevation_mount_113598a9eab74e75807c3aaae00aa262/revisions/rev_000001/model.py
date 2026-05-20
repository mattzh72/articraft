from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _dish_reflector_mesh(radius: float = 0.48, depth: float = 0.15, thickness: float = 0.018) -> MeshGeometry:
    """Thin parabolic reflector shell, with its boresight along local -X."""

    segments = 72
    radial_steps = 14
    inner_radius = 0.035
    geom = MeshGeometry()

    inner: list[list[int]] = []
    outer: list[list[int]] = []
    for i in range(radial_steps + 1):
        r = inner_radius + (radius - inner_radius) * i / radial_steps
        # The rim is forward at negative X, while the deeper center is rearward.
        x_inner = 0.090 - depth * (r / radius) ** 2
        x_outer = x_inner + thickness
        inner_ring: list[int] = []
        outer_ring: list[int] = []
        for j in range(segments):
            a = 2.0 * math.pi * j / segments
            y = r * math.cos(a)
            z = r * math.sin(a)
            inner_ring.append(geom.add_vertex(x_inner, y, z))
            outer_ring.append(geom.add_vertex(x_outer, y, z))
        inner.append(inner_ring)
        outer.append(outer_ring)

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(radial_steps):
        for j in range(segments):
            n = (j + 1) % segments
            # Inner reflecting surface.
            add_quad(inner[i][j], inner[i][n], inner[i + 1][n], inner[i + 1][j])
            # Outer rear surface, reversed so the shell is consistently closed.
            add_quad(outer[i][j], outer[i + 1][j], outer[i + 1][n], outer[i][n])

    # Closed walls at the small central service opening and at the rolled rim.
    for j in range(segments):
        n = (j + 1) % segments
        add_quad(inner[0][j], outer[0][j], outer[0][n], inner[0][n])
        add_quad(inner[-1][j], inner[-1][n], outer[-1][n], outer[-1][j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_dish_yoke")

    dark_steel = model.material("dark_steel", color=(0.12, 0.13, 0.14, 1.0))
    base_blue = model.material("blue_grey_cast_base", color=(0.18, 0.26, 0.32, 1.0))
    reflector_white = model.material("matte_reflector_white", color=(0.82, 0.86, 0.84, 1.0))
    black_rubber = model.material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    brass = model.material("brass_fasteners", color=(0.75, 0.58, 0.26, 1.0))

    base = model.part("base")
    triangular_plate = ExtrudeGeometry.from_z0(
        [(-0.62, -0.43), (0.62, -0.43), (0.0, 0.62)],
        0.060,
        cap=True,
    )
    base.visual(
        mesh_from_geometry(triangular_plate, "triangular_plate"),
        material=base_blue,
        name="triangular_plate",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_steel,
        name="bearing_socket",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=black_rubber,
        name="azimuth_thrust_pad",
    )
    for idx, (x, y) in enumerate(((-0.42, -0.31), (0.42, -0.31), (0.0, 0.40))):
        base.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(xyz=(x, y, 0.066)),
            material=brass,
            name=f"foot_bolt_{idx}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.110, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_steel,
        name="turntable",
    )
    mast.visual(
        Cylinder(radius=0.060, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=dark_steel,
        name="mast_tube",
    )
    mast.visual(
        Box((0.160, 1.240, 0.080)),
        origin=Origin(xyz=(0.020, 0.0, 0.420)),
        material=dark_steel,
        name="lower_span",
    )
    mast.visual(
        Box((0.190, 0.190, 0.100)),
        origin=Origin(xyz=(0.020, 0.0, 0.780)),
        material=dark_steel,
        name="saddle_block",
    )
    for idx, y in enumerate((-0.58, 0.58)):
        mast.visual(
            Box((0.090, 0.080, 0.800)),
            origin=Origin(xyz=(0.060, y, 0.750)),
            material=dark_steel,
            name=f"arm_{idx}",
        )
        mast.visual(
            Box((0.180, 0.100, 0.170)),
            origin=Origin(xyz=(0.000, y, 1.150)),
            material=dark_steel,
            name=f"bearing_{idx}",
        )
    mast.visual(
        Cylinder(radius=0.180, length=0.020),
        origin=Origin(xyz=(0.0, 0.640, 1.150), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=base_blue,
        name="tilt_sector",
    )
    mast.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, 0.655, 1.150), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="sector_boss",
    )

    back_frame = model.part("back_frame")
    back_frame.visual(
        mesh_from_geometry(_dish_reflector_mesh().translate(-0.180, 0.0, 0.0), "reflector_shell"),
        material=reflector_white,
        name="reflector_shell",
    )
    back_frame.visual(
        mesh_from_geometry(TorusGeometry(0.480, 0.017).rotate_y(math.pi / 2.0).translate(-0.240, 0.0, 0.0), "rolled_rim"),
        material=reflector_white,
        name="rolled_rim",
    )
    back_frame.visual(
        mesh_from_geometry(TorusGeometry(0.350, 0.014).rotate_y(math.pi / 2.0).translate(0.150, 0.0, 0.0), "rear_hoop"),
        material=dark_steel,
        name="rear_hoop",
    )
    back_frame.visual(
        Box((0.030, 0.760, 0.030)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=dark_steel,
        name="horizontal_spoke",
    )
    back_frame.visual(
        Box((0.030, 0.030, 0.760)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=dark_steel,
        name="vertical_spoke",
    )
    back_frame.visual(
        Cylinder(radius=0.058, length=0.240),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="central_hub",
    )
    back_frame.visual(
        Cylinder(radius=0.038, length=1.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elevation_axle",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=75.0, velocity=0.8),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=back_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=-0.35, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    back_frame = object_model.get_part("back_frame")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.expect_contact(
        mast,
        base,
        elem_a="turntable",
        elem_b="azimuth_thrust_pad",
        contact_tol=0.002,
        name="mast turntable sits on azimuth bearing",
    )
    ctx.expect_contact(
        back_frame,
        mast,
        elem_a="elevation_axle",
        elem_b="bearing_0",
        contact_tol=0.003,
        name="axle reaches first side bearing",
    )
    ctx.expect_contact(
        back_frame,
        mast,
        elem_a="elevation_axle",
        elem_b="bearing_1",
        contact_tol=0.003,
        name="axle reaches second side bearing",
    )
    ctx.expect_within(
        back_frame,
        mast,
        axes="y",
        inner_elem="rolled_rim",
        outer_elem="lower_span",
        margin=0.010,
        name="reflector fits between side arms",
    )

    def _aabb_center_z(part, elem: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return (lo[2] + hi[2]) * 0.5

    rim_rest_z = _aabb_center_z(back_frame, "rolled_rim")
    with ctx.pose({elevation: 0.75}):
        rim_raised_z = _aabb_center_z(back_frame, "rolled_rim")
    ctx.check(
        "positive elevation raises reflector face",
        rim_rest_z is not None and rim_raised_z is not None and rim_raised_z > rim_rest_z + 0.025,
        details=f"rest_z={rim_rest_z}, raised_z={rim_raised_z}",
    )

    def _aabb_center_xy(part, elem: str) -> tuple[float, float] | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    with ctx.pose({azimuth: math.pi / 2.0}):
        sector_xy = _aabb_center_xy(mast, "tilt_sector")
    ctx.check(
        "azimuth rotates side tilt bracket around vertical mast",
        sector_xy is not None and sector_xy[0] < -0.45 and abs(sector_xy[1]) < 0.20,
        details=f"tilt_sector_xy={sector_xy}",
    )

    return ctx.report()


object_model = build_object_model()
