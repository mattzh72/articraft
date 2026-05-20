from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylindrical_shell_mesh(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    segments: int,
    closed_angle: bool = False,
) -> MeshGeometry:
    """Thin tube/ring mesh along local X, optionally with a longitudinal slot."""
    geom = MeshGeometry()
    x_values = (-length / 2.0, length / 2.0)

    if closed_angle:
        angles = [start_angle + (end_angle - start_angle) * i / segments for i in range(segments)]
        intervals = [(i, (i + 1) % segments) for i in range(segments)]
    else:
        angles = [start_angle + (end_angle - start_angle) * i / segments for i in range(segments + 1)]
        intervals = [(i, i + 1) for i in range(segments)]

    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x in x_values:
        outer_row: list[int] = []
        inner_row: list[int] = []
        for theta in angles:
            c = math.cos(theta)
            s = math.sin(theta)
            outer_row.append(geom.add_vertex(x, outer_radius * s, outer_radius * c))
            inner_row.append(geom.add_vertex(x, inner_radius * s, inner_radius * c))
        outer.append(outer_row)
        inner.append(inner_row)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for j0, j1 in intervals:
        # Outer and inner cylindrical skins.
        quad(outer[0][j0], outer[1][j0], outer[1][j1], outer[0][j1])
        quad(inner[0][j1], inner[1][j1], inner[1][j0], inner[0][j0])

        # Axial lip faces at front and rear connect the visible wall thickness.
        quad(outer[0][j1], outer[0][j0], inner[0][j0], inner[0][j1])
        quad(outer[1][j0], outer[1][j1], inner[1][j1], inner[1][j0])

    if not closed_angle:
        for j in (0, len(angles) - 1):
            quad(outer[0][j], inner[0][j], inner[1][j], outer[1][j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.78, 0.92, 1.0, 0.34))
    frosted_edge = model.material("frosted_edge", rgba=(0.62, 0.82, 1.0, 0.62))
    blue_plastic = model.material("blue_plastic", rgba=(0.12, 0.34, 0.86, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.025, 1.0))
    print_black = model.material("printed_black", rgba=(0.01, 0.01, 0.012, 1.0))

    barrel = model.part("barrel")

    # Open-top transparent tube: the missing longitudinal window exposes the
    # plunger path while still reading as a hollow cylindrical syringe barrel.
    barrel.visual(
        mesh_from_geometry(
            _cylindrical_shell_mesh(
                length=0.150,
                outer_radius=0.0150,
                inner_radius=0.0122,
                start_angle=math.radians(42.0),
                end_angle=math.radians(318.0),
                segments=44,
            ),
            "barrel_cutaway_shell",
        ),
        material=clear_plastic,
        name="barrel_cutaway_shell",
    )
    for x, name in ((-0.079, "rear_collar"), (0.079, "front_collar")):
        barrel.visual(
            mesh_from_geometry(
                _cylindrical_shell_mesh(
                    length=0.008,
                    outer_radius=0.0180,
                    inner_radius=0.0122,
                    start_angle=0.0,
                    end_angle=2.0 * math.pi,
                    segments=48,
                    closed_angle=True,
                ),
                name,
            ),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=clear_plastic,
            name=name,
        )

    # Raised rails flank the cutaway slot; the moving guide shoe on the plunger
    # travels between them, making the prismatic path legible.
    for y, name in ((-0.0112, "guide_rail_0"), (0.0112, "guide_rail_1")):
        barrel.visual(
            Box((0.142, 0.0024, 0.0030)),
            origin=Origin(xyz=(0.0, y, 0.0124)),
            material=frosted_edge,
            name=name,
        )

    # Finger grip wings leave a central bore clear for the rod.
    for y, name in ((-0.031, "finger_wing_0"), (0.031, "finger_wing_1")):
        barrel.visual(
            Box((0.007, 0.032, 0.008)),
            origin=Origin(xyz=(-0.083, y, 0.0)),
            material=clear_plastic,
            name=name,
        )

    # Luer-style nozzle and tip at the front end.
    barrel.visual(
        Cylinder(radius=0.0124, length=0.004),
        origin=Origin(xyz=(0.0815, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_plastic,
        name="nozzle_socket",
    )
    barrel.visual(
        Cylinder(radius=0.0062, length=0.014),
        origin=Origin(xyz=(0.087, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_plastic,
        name="nozzle_base",
    )
    barrel.visual(
        Cylinder(radius=0.0036, length=0.032),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frosted_edge,
        name="nozzle_tip",
    )
    barrel.visual(
        Cylinder(radius=0.0020, length=0.0012),
        origin=Origin(xyz=(0.1266, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=print_black,
        name="nozzle_hole",
    )

    # Printed dose marks are tiny raised ink pads sitting on the lower side of
    # the clear shell, where they do not interfere with the cutaway slot.
    for i, x in enumerate((-0.050, -0.032, -0.014, 0.004, 0.022, 0.040)):
        barrel.visual(
            Box((0.0018, 0.006 if i % 2 else 0.010, 0.0009)),
            origin=Origin(xyz=(x, -0.0136, -0.0064), rpy=(0.0, 0.0, 0.0)),
            material=print_black,
            name=f"dose_mark_{i}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0030, length=0.125),
        origin=Origin(xyz=(-0.0375, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Box((0.018, 0.016, 0.0030)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0158)),
        material=blue_plastic,
        name="guide_shoe",
    )
    plunger.visual(
        Box((0.007, 0.0032, 0.024)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0088)),
        material=blue_plastic,
        name="guide_stem",
    )
    plunger.visual(
        Cylinder(radius=0.016, length=0.007),
        origin=Origin(xyz=(-0.103, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_plastic,
        name="thumb_pad",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        # The child frame sits at the rear mouth of the barrel.  Positive travel
        # pushes the plunger head forward toward the nozzle along the barrel axis.
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=0.085),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_cutaway_shell",
        margin=0.001,
        name="plunger head fits inside barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_cutaway_shell",
        min_overlap=0.010,
        name="retracted plunger head remains in barrel",
    )

    retracted = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.085}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_cutaway_shell",
            margin=0.001,
            name="extended plunger head stays centered",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_cutaway_shell",
            min_overlap=0.010,
            name="extended plunger head remains in barrel",
        )
        extended = ctx.part_world_position(plunger)

    ctx.check(
        "plunger translates toward nozzle",
        retracted is not None and extended is not None and extended[0] > retracted[0] + 0.080,
        details=f"retracted={retracted}, extended={extended}",
    )

    return ctx.report()


object_model = build_object_model()
