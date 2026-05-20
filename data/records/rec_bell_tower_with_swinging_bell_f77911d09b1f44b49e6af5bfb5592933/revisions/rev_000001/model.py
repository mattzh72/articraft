from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


PIVOT_Z = 5.65


def _bell_shell_geometry() -> LatheGeometry:
    """A thin, open bronze bell body hanging below its trunnion axis."""
    outer = [
        (0.62, -1.28),
        (0.60, -1.18),
        (0.54, -1.02),
        (0.46, -0.82),
        (0.36, -0.58),
        (0.26, -0.36),
        (0.19, -0.23),
        (0.15, -0.18),
    ]
    inner = [
        (0.48, -1.22),
        (0.46, -1.10),
        (0.40, -0.92),
        (0.32, -0.68),
        (0.22, -0.44),
        (0.13, -0.28),
        (0.07, -0.22),
        (0.035, -0.20),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="concrete_bell_tower")

    concrete = model.material("weathered_concrete", color=(0.62, 0.61, 0.57, 1.0))
    darker_concrete = model.material("darker_concrete", color=(0.48, 0.48, 0.45, 1.0))
    steel = model.material("dark_galvanized_steel", color=(0.12, 0.14, 0.15, 1.0))
    bronze = model.material("aged_bronze", color=(0.74, 0.48, 0.20, 1.0))
    dark_bronze = model.material("dark_bronze_shadow", color=(0.42, 0.25, 0.11, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((2.30, 2.05, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=darker_concrete,
        name="base_plinth",
    )
    tower.visual(
        Box((1.38, 1.12, 3.72)),
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        material=concrete,
        name="rectangular_shaft",
    )
    tower.visual(
        Box((1.82, 1.56, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 4.19)),
        material=concrete,
        name="cap_slab",
    )

    # Four steel stanchions form a simple yoke frame.  They sit outside the
    # bell's swing envelope and carry bearing rings at the trunnion axis.
    for ix, x in enumerate((-0.78, 0.78)):
        for iy, y in enumerate((-0.16, 0.16)):
            tower.visual(
                Box((0.10, 0.08, 1.46)),
                origin=Origin(xyz=(x, y, 5.03)),
                material=steel,
                name=f"frame_post_{ix}_{iy}",
            )
            tower.visual(
                Box((0.22, 0.18, 0.08)),
                origin=Origin(xyz=(x, y, 4.36)),
                material=steel,
                name=f"post_foot_{ix}_{iy}",
            )

    for y, name_suffix in ((-0.16, "rear"), (0.16, "front")):
        tower.visual(
            Box((1.76, 0.08, 0.12)),
            origin=Origin(xyz=(0.0, y, 5.82)),
            material=steel,
            name=f"{name_suffix}_top_beam",
        )

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(0.095, 0.025, radial_segments=24, tubular_segments=48).rotate_y(
            math.pi / 2.0
        ),
        "bearing_ring_mesh",
    )
    tower.visual(
        bearing_mesh,
        origin=Origin(xyz=(-0.78, 0.0, PIVOT_Z)),
        material=steel,
        name="bearing_0",
    )
    tower.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.78, 0.0, PIVOT_Z)),
        material=steel,
        name="bearing_1",
    )

    # Small diagonal plates give the plain frame a credible bolted yoke look
    # while remaining outside the bell shell.
    for x in (-0.78, 0.78):
        tower.visual(
            Box((0.08, 0.05, 1.10)),
            origin=Origin(xyz=(x, -0.02, 5.10), rpy=(0.44, 0.0, 0.0)),
            material=steel,
            name=f"brace_{'neg' if x < 0 else 'pos'}",
        )

    bell = model.part("bell")
    bell.visual(
        mesh_from_geometry(_bell_shell_geometry(), "hollow_bell_shell"),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        mesh_from_geometry(TorusGeometry(0.575, 0.045, radial_segments=24, tubular_segments=96), "mouth_rim"),
        origin=Origin(xyz=(0.0, 0.0, -1.235)),
        material=bronze,
        name="mouth_rim",
    )
    bell.visual(
        mesh_from_geometry(TorusGeometry(0.335, 0.017, radial_segments=16, tubular_segments=80), "waist_band"),
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        material=dark_bronze,
        name="waist_band",
    )
    bell.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=bronze,
        name="crown_cap",
    )
    bell.visual(
        Cylinder(radius=0.071, length=1.64),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_axle",
    )
    bell.visual(
        Box((0.92, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=steel,
        name="headstock",
    )
    for x in (-0.26, 0.26):
        bell.visual(
            Box((0.08, 0.08, 0.28)),
            origin=Origin(xyz=(x, 0.0, -0.29)),
            material=steel,
            name=f"crown_strap_{'neg' if x < 0 else 'pos'}",
        )

    # A visible clapper hangs inside the hollow bell.  It is fixed to the bell
    # assembly here; the requested external motion is the bell's swing.
    bell.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.32)),
        material=steel,
        name="clapper_pivot",
    )
    bell.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=steel,
        name="clapper_hanger",
    )
    bell.visual(
        Cylinder(radius=0.018, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, -0.68)),
        material=steel,
        name="clapper_rod",
    )
    bell.visual(
        Sphere(radius=0.115),
        origin=Origin(xyz=(0.0, 0.0, -1.08)),
        material=dark_bronze,
        name="clapper_ball",
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.2, lower=-0.60, upper=0.60),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("tower_to_bell")

    ctx.check(
        "bell has symmetric swing limits",
        hinge.motion_limits.lower <= -0.55 and hinge.motion_limits.upper >= 0.55,
        details=f"limits={hinge.motion_limits}",
    )
    ctx.allow_overlap(
        bell,
        tower,
        elem_a="trunnion_axle",
        elem_b="bearing_0",
        reason="The trunnion axle is intentionally captured in the steel bearing ring.",
    )
    ctx.allow_overlap(
        bell,
        tower,
        elem_a="trunnion_axle",
        elem_b="bearing_1",
        reason="The trunnion axle is intentionally captured in the steel bearing ring.",
    )
    ctx.expect_gap(
        bell,
        tower,
        axis="z",
        positive_elem="bell_shell",
        negative_elem="cap_slab",
        min_gap=0.02,
        name="bell shell clears concrete cap",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="bearing_0",
        min_overlap=0.02,
        name="axle passes through first bearing plane",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="bearing_1",
        min_overlap=0.02,
        name="axle passes through second bearing plane",
    )

    def _element_center_y(part, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[1] + upper[1]) * 0.5

    with ctx.pose({hinge: 0.0}):
        rest_y = _element_center_y(bell, "bell_shell")
    with ctx.pose({hinge: 0.60}):
        swung_y = _element_center_y(bell, "bell_shell")

    ctx.check(
        "positive hinge angle swings bell forward",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.25,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
