from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_x(
    *,
    x0: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
):
    """Open annular tube extruded along the object X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x0, 0.0, 0.0))
    )


def _frustum_x(
    *,
    x0: float,
    length: float,
    r0: float,
    r1: float,
):
    """Tapered clinical luer-style nozzle along +X."""
    return (
        cq.Workplane("YZ")
        .circle(r0)
        .workplane(offset=length)
        .circle(r1)
        .loft(combine=True)
        .translate((x0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_syringe")

    clear_polycarbonate = model.material(
        "smoke_clear_polycarbonate", rgba=(0.72, 0.86, 0.95, 0.36)
    )
    olive_molded = model.material("olive_molded_overmold", rgba=(0.19, 0.24, 0.16, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    black_ink = model.material("black_graduation_ink", rgba=(0.0, 0.0, 0.0, 1.0))
    orange_stop = model.material("orange_stop_band", rgba=(0.95, 0.28, 0.05, 1.0))
    steel = model.material("brushed_stainless_fasteners", rgba=(0.68, 0.68, 0.62, 1.0))

    barrel = model.part("barrel")

    # A real, hollow 10 ml-class barrel: 115 mm long, 24 mm OD, with a visible
    # bore instead of a solid placeholder.  The object axis is +X toward the
    # nozzle.
    barrel.visual(
        mesh_from_cadquery(
            _tube_x(x0=-0.055, length=0.115, outer_radius=0.0120, inner_radius=0.0087),
            "graduated_barrel_shell",
            tolerance=0.00035,
            angular_tolerance=0.06,
        ),
        material=clear_polycarbonate,
        name="barrel_shell",
    )

    for name, x0, length, outer_r, inner_r in (
        ("rear_collar", -0.064, 0.010, 0.0175, 0.0062),
        ("front_collar", 0.058, 0.012, 0.0165, 0.0088),
        ("luer_lock_collar", 0.067, 0.010, 0.0120, 0.0048),
    ):
        barrel.visual(
            mesh_from_cadquery(
                _tube_x(x0=x0, length=length, outer_radius=outer_r, inner_radius=inner_r),
                name,
                tolerance=0.00035,
                angular_tolerance=0.06,
            ),
            material=olive_molded,
            name=name,
        )

    barrel.visual(
        mesh_from_cadquery(
            _frustum_x(x0=0.076, length=0.026, r0=0.0049, r1=0.0017),
            "tapered_nozzle_tip",
            tolerance=0.00025,
            angular_tolerance=0.05,
        ),
        material=clear_polycarbonate,
        name="nozzle_tip",
    )
    barrel.visual(
        Cylinder(radius=0.00135, length=0.0007),
        origin=Origin(xyz=(0.10165, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_ink,
        name="nozzle_bore",
    )

    # Rugged molded cage rails.  They bridge the reinforced collars and make the
    # transparent syringe read as a serviceable utility tool rather than a
    # disposable shell.
    for name, xyz, size in (
        ("top_rail", (0.001, 0.0, 0.0142), (0.116, 0.006, 0.0042)),
        ("bottom_rail", (0.001, 0.0, -0.0142), (0.116, 0.006, 0.0042)),
        ("side_rail_0", (0.001, 0.0142, 0.0), (0.116, 0.0042, 0.006)),
        ("side_rail_1", (0.001, -0.0142, 0.0), (0.116, 0.0042, 0.006)),
    ):
        barrel.visual(Box(size), origin=Origin(xyz=xyz), material=olive_molded, name=name)

    # Finger flange and rear linear guide, with lugs thick enough to imply field
    # use and maintenance.
    for name, xyz, size in (
        ("finger_wing_0", (-0.069, 0.031, 0.0), (0.010, 0.039, 0.011)),
        ("finger_wing_1", (-0.069, -0.031, 0.0), (0.010, 0.039, 0.011)),
        ("upper_rear_web", (-0.069, 0.0, 0.019), (0.010, 0.020, 0.011)),
        ("lower_rear_web", (-0.069, 0.0, -0.019), (0.010, 0.020, 0.011)),
    ):
        barrel.visual(Box(size), origin=Origin(xyz=xyz), material=olive_molded, name=name)

    # Exposed service fasteners on the rugged flange and luer collar.
    for i, (x, y, z, radius) in enumerate(
        (
            (-0.0732, 0.038, 0.005, 0.0022),
            (-0.0732, 0.038, -0.005, 0.0022),
            (-0.0732, -0.038, 0.005, 0.0022),
            (-0.0732, -0.038, -0.005, 0.0022),
            (0.070, 0.0125, 0.0, 0.0017),
            (0.070, -0.0125, 0.0, 0.0017),
        )
    ):
        barrel.visual(
            Cylinder(radius=radius, length=0.0016),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"fastener_{i}",
        )

    # Printed graduations: major and minor ticks are slightly seated into the
    # outside of the clear barrel so they read as ink on the cylinder.
    for i in range(21):
        x = -0.047 + i * 0.0047
        major = i % 4 == 0
        tick_y = 0.0090 if major else 0.0052
        tick_z = 0.00036
        barrel.visual(
            Box((0.00055, tick_y, tick_z)),
            origin=Origin(xyz=(x, -0.0042, 0.01205)),
            material=black_ink,
            name=f"tick_{i}",
        )

    # Larger numbered-dose cues represented as heavy ink pads next to every
    # fourth graduation.
    for n, x in enumerate((-0.047, -0.0282, -0.0094, 0.0094, 0.0282, 0.047)):
        barrel.visual(
            Box((0.0022, 0.0012, 0.00038)),
            origin=Origin(xyz=(x, 0.0020, 0.01205)),
            material=black_ink,
            name=f"dose_dot_{n}",
        )
        barrel.visual(
            Box((0.0006, 0.0034, 0.00038)),
            origin=Origin(xyz=(x + 0.0015, 0.0020, 0.01205)),
            material=black_ink,
            name=f"dose_bar_{n}",
        )

    # Bayonet lugs on the luer lock collar.
    barrel.visual(
        Box((0.006, 0.004, 0.003)),
        origin=Origin(xyz=(0.072, 0.012, 0.0)),
        material=olive_molded,
        name="luer_lug_0",
    )
    barrel.visual(
        Box((0.006, 0.004, 0.003)),
        origin=Origin(xyz=(0.072, -0.012, 0.0)),
        material=olive_molded,
        name="luer_lug_1",
    )

    plunger = model.part("plunger")

    # The moving part's local frame sits at the rear guide plane when the
    # plunger is fully depressed.  A cross-section rod plus a central stem make
    # the guided linear motion mechanically legible.
    plunger.visual(
        Cylinder(radius=0.0024, length=0.122),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive_molded,
        name="center_rod",
    )
    plunger.visual(
        Box((0.118, 0.0022, 0.0092)),
        origin=Origin(xyz=(0.039, 0.0, 0.0)),
        material=olive_molded,
        name="vertical_web",
    )
    plunger.visual(
        Box((0.118, 0.0092, 0.0022)),
        origin=Origin(xyz=(0.039, 0.0, 0.0)),
        material=olive_molded,
        name="horizontal_web",
    )
    plunger.visual(
        Cylinder(radius=0.0045, length=0.022),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive_molded,
        name="thumb_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0102, length=0.0055),
        origin=Origin(xyz=(-0.0142, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=orange_stop,
        name="stop_collar",
    )
    plunger.visual(
        Box((0.012, 0.046, 0.028)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=olive_molded,
        name="thumb_pad",
    )
    for i, z in enumerate((-0.0105, 0.0, 0.0105)):
        plunger.visual(
            Box((0.013, 0.040, 0.0012)),
            origin=Origin(xyz=(-0.031, 0.0, z)),
            material=black_rubber,
            name=f"thumb_grip_{i}",
        )

    plunger.visual(
        Cylinder(radius=0.0078, length=0.014),
        origin=Origin(xyz=(0.106, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="piston_core",
    )
    for i, x in enumerate((0.100, 0.112)):
        plunger.visual(
            Cylinder(radius=0.0089, length=0.0022),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"seal_rib_{i}",
        )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.18, lower=0.0, upper=0.065),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    limits = slide.motion_limits
    ctx.check(
        "plunger has clinical travel limits",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.060 <= limits.upper <= 0.070,
        details=f"limits={limits}",
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="piston_core",
        outer_elem="barrel_shell",
        margin=0.0,
        name="piston is centered inside barrel bore envelope",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="piston_core",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="depressed piston remains inside graduated barrel",
    )
    ctx.expect_gap(
        barrel,
        plunger,
        axis="x",
        positive_elem="rear_collar",
        negative_elem="thumb_pad",
        min_gap=0.003,
        max_gap=0.020,
        name="thumb pad stops short of rear guide at full depression",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: limits.upper if limits and limits.upper is not None else 0.065}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="piston_core",
            outer_elem="barrel_shell",
            margin=0.0,
            name="retracted piston stays coaxial with barrel",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="piston_core",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="retracted piston remains retained in barrel",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger retracts along negative barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] < rest_pos[0] - 0.060,
        details=f"rest={rest_pos}, retracted={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
