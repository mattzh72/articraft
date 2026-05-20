from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _capsule_profile(
    length: float,
    width: float,
    *,
    segments_per_end: int = 24,
) -> list[tuple[float, float]]:
    radius = width * 0.5
    points: list[tuple[float, float]] = []
    for index in range(segments_per_end + 1):
        angle = -math.pi / 2.0 + math.pi * index / segments_per_end
        points.append((length + radius * math.cos(angle), radius * math.sin(angle)))
    for index in range(segments_per_end + 1):
        angle = math.pi / 2.0 + math.pi * index / segments_per_end
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_revolute_chain")

    plate_metal = model.material("matte_plate_metal", rgba=(0.43, 0.45, 0.46, 1.0))
    link_metal = model.material("satin_link_metal", rgba=(0.70, 0.72, 0.72, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.10, 0.11, 0.12, 1.0))
    bushing_bronze = model.material("bronze_bushings", rgba=(0.67, 0.48, 0.23, 1.0))

    mounting_plate = model.part("mounting_plate")
    plate_profile = _translate_profile(
        rounded_rect_profile(0.300, 0.150, 0.018, corner_segments=8),
        dx=-0.080,
    )
    screw_centers = [(-0.180, -0.052), (-0.180, 0.052), (-0.050, -0.052), (-0.050, 0.052)]
    plate_holes = [_circle_profile(0.0075, center=center, segments=28) for center in screw_centers]
    mounting_plate.visual(
        _mesh("mounting_plate_shell", ExtrudeWithHolesGeometry(plate_profile, plate_holes, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=plate_metal,
        name="plate_shell",
    )
    for screw_index, (x, y) in enumerate(screw_centers):
        mounting_plate.visual(
            Cylinder(radius=0.012, length=0.003),
            origin=Origin(xyz=(x, y, 0.0135)),
            material=dark_hardware,
            name=f"screw_head_{screw_index}",
        )
        mounting_plate.visual(
            Box((0.017, 0.0022, 0.0012)),
            origin=Origin(xyz=(x, y, 0.0155), rpy=(0.0, 0.0, math.pi / 4.0)),
            material=plate_metal,
            name=f"screw_slot_{screw_index}",
        )

    mounting_plate.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_hardware,
        name="root_lower_knuckle",
    )
    mounting_plate.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=dark_hardware,
        name="root_pin",
    )
    mounting_plate.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_hardware,
        name="root_pin_cap",
    )

    main_link = model.part("main_link")
    main_profile = _capsule_profile(0.300, 0.054, segments_per_end=28)
    main_holes = [_circle_profile(0.0155, center=(0.0, 0.0), segments=40)]
    main_link.visual(
        _mesh("main_link_flat_bar", ExtrudeWithHolesGeometry(main_profile, main_holes, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=link_metal,
        name="main_bar",
    )
    main_link.visual(
        _mesh(
            "main_link_proximal_bushing",
            ExtrudeWithHolesGeometry(
                _circle_profile(0.025, segments=48),
                [_circle_profile(0.0155, segments=40)],
                0.003,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=bushing_bronze,
        name="proximal_bushing",
    )
    main_link.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(xyz=(0.300, 0.0, 0.030)),
        material=bushing_bronze,
        name="distal_boss",
    )
    main_link.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.300, 0.0, 0.043)),
        material=dark_hardware,
        name="distal_pin",
    )
    main_link.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.300, 0.0, 0.060)),
        material=dark_hardware,
        name="distal_pin_cap",
    )

    distal_tab = model.part("distal_tab")
    tab_profile = _capsule_profile(0.165, 0.052, segments_per_end=28)
    tab_holes = [
        _circle_profile(0.0155, center=(0.0, 0.0), segments=40),
        _circle_profile(0.0065, center=(0.148, 0.0), segments=28),
    ]
    distal_tab.visual(
        _mesh("distal_tab_shell", ExtrudeWithHolesGeometry(tab_profile, tab_holes, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=link_metal,
        name="tab_shell",
    )
    distal_tab.visual(
        _mesh(
            "distal_tab_proximal_bushing",
            ExtrudeWithHolesGeometry(
                _circle_profile(0.024, segments=48),
                [_circle_profile(0.0155, segments=40)],
                0.003,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=bushing_bronze,
        name="joint_bushing",
    )
    distal_tab.visual(
        _mesh(
            "distal_tab_attachment_eye",
            ExtrudeWithHolesGeometry(
                _circle_profile(0.017, center=(0.148, 0.0), segments=44),
                [_circle_profile(0.0065, center=(0.148, 0.0), segments=28)],
                0.003,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=bushing_bronze,
        name="attachment_eye",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=mounting_plate,
        child=main_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=main_link,
        child=distal_tab,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-1.75, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mounting_plate = object_model.get_part("mounting_plate")
    main_link = object_model.get_part("main_link")
    distal_tab = object_model.get_part("distal_tab")
    root_pivot = object_model.get_articulation("root_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")

    ctx.check(
        "two revolute joints only",
        len(object_model.articulations) == 2
        and root_pivot.articulation_type == ArticulationType.REVOLUTE
        and elbow_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "both joints bend in the horizontal plane",
        tuple(root_pivot.axis) == (0.0, 0.0, 1.0) and tuple(elbow_pivot.axis) == (0.0, 0.0, 1.0),
        details=f"root_axis={root_pivot.axis}, elbow_axis={elbow_pivot.axis}",
    )

    ctx.allow_overlap(
        mounting_plate,
        main_link,
        elem_a="root_pin",
        elem_b="main_bar",
        reason="The root pin is intentionally captured through the bored proximal knuckle of the first flat link.",
    )
    ctx.allow_overlap(
        main_link,
        distal_tab,
        elem_a="distal_pin",
        elem_b="tab_shell",
        reason="The distal pin is intentionally captured through the bored knuckle of the compact end tab.",
    )

    ctx.expect_within(
        mounting_plate,
        main_link,
        axes="xy",
        inner_elem="root_pin",
        outer_elem="main_bar",
        margin=0.003,
        name="root pin is centered inside first-link knuckle footprint",
    )
    ctx.expect_overlap(
        mounting_plate,
        main_link,
        axes="z",
        elem_a="root_pin",
        elem_b="main_bar",
        min_overlap=0.006,
        name="root pin passes through first-link thickness",
    )
    ctx.expect_gap(
        main_link,
        mounting_plate,
        axis="z",
        positive_elem="main_bar",
        negative_elem="root_lower_knuckle",
        min_gap=0.001,
        max_gap=0.004,
        name="first link rides just above lower root knuckle",
    )
    ctx.expect_within(
        main_link,
        distal_tab,
        axes="xy",
        inner_elem="distal_pin",
        outer_elem="tab_shell",
        margin=0.003,
        name="distal pin is centered inside end-tab knuckle footprint",
    )
    ctx.expect_overlap(
        main_link,
        distal_tab,
        axes="z",
        elem_a="distal_pin",
        elem_b="tab_shell",
        min_overlap=0.006,
        name="distal pin passes through end-tab thickness",
    )

    def _xy_center(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)

    rest_main_center = _xy_center(main_link)
    with ctx.pose({root_pivot: 0.70}):
        swung_main_center = _xy_center(main_link)
    ctx.check(
        "root joint swings the first link in plane",
        rest_main_center is not None
        and swung_main_center is not None
        and swung_main_center[1] > rest_main_center[1] + 0.060,
        details=f"rest={rest_main_center}, swung={swung_main_center}",
    )

    rest_tab_center = _xy_center(distal_tab)
    with ctx.pose({elbow_pivot: 0.80}):
        bent_tab_center = _xy_center(distal_tab)
    ctx.check(
        "elbow joint swings the compact end tab in plane",
        rest_tab_center is not None
        and bent_tab_center is not None
        and bent_tab_center[1] > rest_tab_center[1] + 0.040,
        details=f"rest={rest_tab_center}, bent={bent_tab_center}",
    )

    return ctx.report()


object_model = build_object_model()
