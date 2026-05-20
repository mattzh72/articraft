from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tower_shroud_mesh():
    """One continuous lathed, premium painted-metal tower shroud."""
    return LatheGeometry(
        [
            (0.000, 0.035),
            (0.225, 0.035),
            (0.255, 0.050),
            (0.268, 0.080),
            (0.250, 0.104),
            (0.160, 0.122),
            (0.128, 0.155),
            (0.086, 0.300),
            (0.064, 0.760),
            (0.058, 1.100),
            (0.073, 1.235),
            (0.096, 1.286),
            (0.102, 1.340),
            (0.096, 1.360),
            (0.000, 1.360),
        ],
        segments=80,
    )


def _head_shell_mesh():
    """Thin-walled searchlight barrel with a flared front lip."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.108, -0.255),
            (0.123, -0.232),
            (0.140, -0.145),
            (0.146, 0.095),
            (0.158, 0.225),
            (0.177, 0.315),
            (0.184, 0.350),
        ],
        [
            (0.088, -0.246),
            (0.103, -0.218),
            (0.118, -0.134),
            (0.124, 0.095),
            (0.134, 0.222),
            (0.150, 0.306),
            (0.156, 0.344),
        ],
        segments=88,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def _reflector_mesh():
    """Shallow polished reflector visible behind the smoked front lens."""
    return LatheGeometry(
        [
            (0.000, 0.142),
            (0.034, 0.148),
            (0.082, 0.176),
            (0.124, 0.232),
            (0.145, 0.298),
            (0.137, 0.314),
            (0.045, 0.250),
            (0.000, 0.218),
        ],
        segments=72,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_searchlight_tower")

    painted_metal = model.material("painted_metal", rgba=(0.80, 0.82, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.030, 0.032, 0.034, 1.0))
    graphite_polymer = model.material("graphite_polymer", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.64, 0.66, 0.68, 1.0))
    clear_lens = model.material("smoked_glass", rgba=(0.54, 0.72, 0.88, 0.46))
    warm_emitter = model.material("warm_emitter", rgba=(1.0, 0.80, 0.44, 1.0))

    base = model.part("base_tower")
    base.visual(
        Cylinder(radius=0.285, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="elastomer_foot",
    )
    base.visual(
        _mesh("tower_shroud", _tower_shroud_mesh()),
        material=painted_metal,
        name="tower_shroud",
    )
    base.visual(
        _mesh("base_edge_bead", TorusGeometry(radius=0.257, tube=0.007, radial_segments=16, tubular_segments=96)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=brushed_aluminum,
        name="base_edge_bead",
    )
    base.visual(
        Cylinder(radius=0.106, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.3375)),
        material=dark_metal,
        name="top_collar",
    )
    base.visual(
        Box((0.020, 0.052, 0.72)),
        origin=Origin(xyz=(-0.064, 0.0, 0.760)),
        material=graphite_polymer,
        name="rear_cable_chase",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(-0.270, 0.0, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="cable_grommet",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.285, length=1.36),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.112, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=dark_metal,
        name="pan_turntable",
    )
    pan_yoke.visual(
        Cylinder(radius=0.070, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=painted_metal,
        name="pan_neck",
    )
    pan_yoke.visual(
        _mesh(
            "trunnion_yoke",
            TrunnionYokeGeometry(
                (0.540, 0.135, 0.500),
                span_width=0.405,
                trunnion_diameter=0.084,
                trunnion_center_z=0.340,
                base_thickness=0.085,
                corner_radius=0.018,
                center=False,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.260), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=painted_metal,
        name="trunnion_yoke",
    )
    pan_yoke.visual(
        Box((0.170, 0.410, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=graphite_polymer,
        name="polymer_saddle_trim",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.24, 0.56, 0.76)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
    )

    head = model.part("spotlight_head")
    head.visual(
        _mesh("head_shell", _head_shell_mesh()),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_metal,
        name="head_shell",
    )
    head.visual(
        _mesh("front_bezel_ring", TorusGeometry(radius=0.166, tube=0.011, radial_segments=18, tubular_segments=88)),
        origin=Origin(xyz=(0.350, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.148, length=0.014),
        origin=Origin(xyz=(0.342, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_lens,
        name="front_lens",
    )
    head.visual(
        _mesh("lens_gasket", TorusGeometry(radius=0.153, tube=0.006, radial_segments=14, tubular_segments=72)),
        origin=Origin(xyz=(0.340, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lens_gasket",
    )
    head.visual(
        _mesh("reflector_bowl", _reflector_mesh()),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="reflector_bowl",
    )
    head.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=warm_emitter,
        name="led_emitter",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.480),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="light_carrier",
    )
    head.visual(
        Cylinder(radius=0.115, length=0.035),
        origin=Origin(xyz=(-0.264, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite_polymer,
        name="rear_cap",
    )
    head.visual(
        _mesh("rear_seam_bead", TorusGeometry(radius=0.112, tube=0.006, radial_segments=14, tubular_segments=72)),
        origin=Origin(xyz=(-0.246, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_seam_bead",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.590),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_trunnion",
    )
    head.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(xyz=(0.0, 0.284, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="trunnion_cap_0",
    )
    head.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(xyz=(0.0, -0.284, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="trunnion_cap_1",
    )
    head.visual(
        Box((0.220, 0.030, 0.020)),
        origin=Origin(xyz=(-0.060, 0.0, 0.154)),
        material=graphite_polymer,
        name="top_heat_spine",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.62),
        mass=2.2,
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.2, lower=-0.45, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_tower")
    pan_yoke = object_model.get_part("pan_yoke")
    head = object_model.get_part("spotlight_head")
    pan = object_model.get_articulation("pan_axis")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.expect_contact(
        base,
        pan_yoke,
        elem_a="top_collar",
        elem_b="pan_turntable",
        contact_tol=0.001,
        name="pan bearing sits on tower collar",
    )
    ctx.expect_contact(
        pan_yoke,
        head,
        elem_a="trunnion_yoke",
        elem_b="trunnion_cap_0",
        contact_tol=0.001,
        name="first tilt bearing cap bears on yoke cheek",
    )
    ctx.expect_contact(
        pan_yoke,
        head,
        elem_a="trunnion_yoke",
        elem_b="trunnion_cap_1",
        contact_tol=0.001,
        name="second tilt bearing cap bears on yoke cheek",
    )
    ctx.expect_overlap(
        head,
        pan_yoke,
        axes="xyz",
        elem_a="tilt_trunnion",
        elem_b="trunnion_yoke",
        min_overlap=0.055,
        name="tilt trunnion is carried through the yoke bore region",
    )

    pan_axis = tuple(pan.axis or ())
    tilt_axis = tuple(tilt.axis or ())
    axis_dot = sum(a * b for a, b in zip(pan_axis, tilt_axis))
    ctx.check(
        "pan and tilt axes are separated and perpendicular",
        len(pan_axis) == 3
        and len(tilt_axis) == 3
        and abs(axis_dot) < 1e-6
        and abs(pan_axis[2]) > 0.99
        and abs(tilt_axis[1]) > 0.99,
        details=f"pan_axis={pan_axis}, tilt_axis={tilt_axis}, dot={axis_dot}",
    )

    def elem_center(part, elem):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lens = elem_center(head, "front_lens")
    with ctx.pose({tilt: 0.65}):
        raised_lens = elem_center(head, "front_lens")
    with ctx.pose({tilt: -0.35}):
        lowered_lens = elem_center(head, "front_lens")
    ctx.check(
        "positive tilt raises the beam",
        rest_lens is not None
        and raised_lens is not None
        and lowered_lens is not None
        and raised_lens[2] > rest_lens[2] + 0.12
        and lowered_lens[2] < rest_lens[2] - 0.07,
        details=f"rest={rest_lens}, raised={raised_lens}, lowered={lowered_lens}",
    )

    with ctx.pose({pan: math.pi / 2.0}):
        panned_lens = elem_center(head, "front_lens")
    ctx.check(
        "pan rotates the head around the vertical tower axis",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[1] + 0.25
        and abs(panned_lens[2] - rest_lens[2]) < 0.01,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
