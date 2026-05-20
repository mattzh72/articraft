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
    rounded_rect_profile,
    ExtrudeWithHolesGeometry,
)


def _syringe_shell_mesh() -> MeshGeometry:
    """One continuous thin shell for the clear barrel, shoulder, and nozzle."""
    segments = 72
    # (axis_x, outer_radius, inner_radius) in meters.  The smaller front
    # radius forms the hollow luer/nozzle tip; both ends remain visibly open.
    profile = [
        (0.000, 0.0124, 0.0103),
        (0.004, 0.0130, 0.0103),
        (0.011, 0.0122, 0.0103),
        (0.112, 0.0122, 0.0103),
        (0.121, 0.0100, 0.0076),
        (0.133, 0.0060, 0.0034),
        (0.153, 0.0031, 0.00125),
        (0.158, 0.0026, 0.00125),
    ]

    mesh = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x, r_outer, r_inner in profile:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            c = math.cos(theta)
            s = math.sin(theta)
            outer_ring.append(mesh.add_vertex(x, r_outer * c, r_outer * s))
            inner_ring.append(mesh.add_vertex(x, r_inner * c, r_inner * s))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for j in range(len(profile) - 1):
        for i in range(segments):
            ni = (i + 1) % segments
            # Outer skin.
            mesh.add_face(outer[j][i], outer[j + 1][i], outer[j + 1][ni])
            mesh.add_face(outer[j][i], outer[j + 1][ni], outer[j][ni])
            # Inner bore, reversed so normals face the hollow interior.
            mesh.add_face(inner[j][ni], inner[j + 1][ni], inner[j + 1][i])
            mesh.add_face(inner[j][ni], inner[j + 1][i], inner[j][i])

    # Annular lips at the open rear and open nozzle end.
    for ring_index in (0, len(profile) - 1):
        for i in range(segments):
            ni = (i + 1) % segments
            if ring_index == 0:
                mesh.add_face(inner[ring_index][i], outer[ring_index][i], outer[ring_index][ni])
                mesh.add_face(inner[ring_index][i], outer[ring_index][ni], inner[ring_index][ni])
            else:
                mesh.add_face(outer[ring_index][i], inner[ring_index][i], inner[ring_index][ni])
                mesh.add_face(outer[ring_index][i], inner[ring_index][ni], outer[ring_index][ni])

    return mesh


def _rear_finger_flange_mesh() -> MeshGeometry:
    """Rounded finger flange with a central barrel hole."""
    outer = rounded_rect_profile(0.030, 0.086, 0.006, corner_segments=8)
    hole = [
        (0.0117 * math.cos(2.0 * math.pi * i / 48), 0.0117 * math.sin(2.0 * math.pi * i / 48))
        for i in range(48)
    ]
    flange = ExtrudeWithHolesGeometry(outer, [hole], 0.0065, cap=True, center=True, closed=True)
    # The extrude axis is local Z. Rotate so the thin dimension becomes syringe X.
    flange.rotate_y(math.pi / 2.0)
    flange.translate(0.0005, 0.0, 0.0)
    return flange


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.78, 0.94, 1.0, 0.34))
    frosted_plastic = model.material("frosted_plastic", rgba=(0.88, 0.94, 0.97, 0.72))
    rubber = model.material("black_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    ink = model.material("black_ink", rgba=(0.01, 0.01, 0.01, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_syringe_shell_mesh(), "syringe_barrel_shell"),
        material=clear_plastic,
        name="barrel_shell",
    )
    barrel.visual(
        mesh_from_geometry(_rear_finger_flange_mesh(), "syringe_finger_flange"),
        material=clear_plastic,
        name="finger_flange",
    )

    # Printed graduation ticks sit slightly proud on the upper side of the
    # transparent barrel.  They are broad enough to be visible but not a separate
    # mechanism.
    for idx, x in enumerate((0.026, 0.044, 0.062, 0.080, 0.098)):
        tick_length = 0.015 if idx % 2 == 0 else 0.010
        barrel.visual(
            Box((0.0012, tick_length, 0.00045)),
            origin=Origin(xyz=(x, 0.0, 0.01235)),
            material=ink,
            name=f"graduation_{idx}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.00955, length=0.012),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="plunger_head",
    )
    # Slightly proud seal lips make the rubber plunger head read as a syringe
    # piston while preserving a small clearance to the barrel bore.
    for idx, x in enumerate((0.0125, 0.0235)):
        plunger.visual(
            Cylinder(radius=0.00985, length=0.0022),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"seal_lip_{idx}",
        )
    rod_center_x = -0.035
    rod_length = 0.094
    plunger.visual(
        Box((rod_length, 0.0032, 0.0080)),
        origin=Origin(xyz=(rod_center_x, 0.0, 0.0)),
        material=frosted_plastic,
        name="vertical_rod",
    )
    plunger.visual(
        Box((rod_length, 0.0080, 0.0032)),
        origin=Origin(xyz=(rod_center_x, 0.0, 0.0)),
        material=frosted_plastic,
        name="cross_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0175, length=0.0065),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frosted_plastic,
        name="thumb_pad",
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.076),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    ctx.check(
        "plunger has one prismatic barrel-axis slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper > 0.07,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.0005,
        name="rubber head stays centered in barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="retracted plunger head is retained inside barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.0005,
            name="pressed plunger head remains centered in barrel",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="pressed plunger head remains inside barrel",
        )
        pressed_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive slide pushes plunger toward nozzle",
        rest_pos is not None and pressed_pos is not None and pressed_pos[0] > rest_pos[0] + 0.07,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
