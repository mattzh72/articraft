from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annular_disc(outer_radius: float, inner_radius: float, thickness: float):
    """A centered annular solid in the local XZ plane, with depth along local Y."""
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        center=True,
    )
    return geom.rotate_x(math.pi / 2.0)


def _rounded_panel(width: float, height: float, thickness: float, radius: float):
    """A centered rounded rectangle in the local XZ plane, with depth along local Y."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius),
        thickness,
        cap=True,
        center=True,
    )
    return geom.rotate_x(math.pi / 2.0)


def _rounded_frame(
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    thickness: float,
    radius: float,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, radius),
        [rounded_rect_profile(inner_width, inner_height, max(0.001, radius * 0.55))],
        thickness,
        center=True,
    )
    return geom.rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer_with_add_door")

    white = Material("warm_white_enamel", color=(0.94, 0.95, 0.94, 1.0))
    panel_white = Material("slightly_recessed_white", color=(0.86, 0.88, 0.88, 1.0))
    dark = Material("smoked_black_glass", color=(0.02, 0.025, 0.03, 1.0))
    glass = Material("blue_tinted_glass", color=(0.36, 0.62, 0.82, 0.45))
    rubber = Material("dark_rubber_gasket", color=(0.01, 0.012, 0.012, 1.0))
    metal = Material("brushed_steel", color=(0.58, 0.60, 0.62, 1.0))
    charcoal = Material("charcoal_plastic", color=(0.08, 0.09, 0.10, 1.0))

    model.materials.extend([white, panel_white, dark, glass, rubber, metal, charcoal])

    cabinet = model.part("cabinet")
    # Root body: real domestic front-loader scale, standing on four small feet.
    cabinet.visual(
        Box((0.68, 0.62, 0.84)),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=white,
        name="body_shell",
    )
    cabinet.visual(
        Box((0.64, 0.030, 0.125)),
        origin=Origin(xyz=(0.0, -0.325, 0.815)),
        material=panel_white,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.58, 0.018, 0.54)),
        origin=Origin(xyz=(0.0, -0.321, 0.47)),
        material=panel_white,
        name="front_recess",
    )
    cabinet.visual(
        mesh_from_geometry(_annular_disc(0.278, 0.205, 0.018), "porthole_gasket"),
        origin=Origin(xyz=(0.0, -0.315, 0.47)),
        material=rubber,
        name="porthole_gasket",
    )
    cabinet.visual(
        mesh_from_geometry(_annular_disc(0.310, 0.286, 0.010), "door_shadow_trim"),
        origin=Origin(xyz=(0.0, -0.320, 0.47)),
        material=charcoal,
        name="door_shadow_trim",
    )
    cabinet.visual(
        Box((0.205, 0.012, 0.046)),
        origin=Origin(xyz=(-0.205, -0.343, 0.825)),
        material=white,
        name="detergent_drawer",
    )
    cabinet.visual(
        Box((0.61, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, -0.319, 0.078)),
        material=panel_white,
        name="toe_kick",
    )
    for i, (x, y) in enumerate(
        [(-0.265, -0.225), (0.265, -0.225), (-0.265, 0.225), (0.265, 0.225)]
    ):
        cabinet.visual(
            Box((0.075, 0.075, 0.040)),
            origin=Origin(xyz=(x, y, 0.020)),
            material=charcoal,
            name=f"foot_{i}",
        )
    cabinet.visual(
        Box((0.020, 0.074, 0.48)),
        origin=Origin(xyz=(-0.315, -0.323, 0.47)),
        material=metal,
        name="main_hinge_mount",
    )

    main_door = model.part("main_door")
    # The main door part frame is on the vertical left hinge line; the circular
    # porthole assembly extends along local +X from that hinge.
    main_door.visual(
        mesh_from_geometry(_annular_disc(0.255, 0.172, 0.055), "main_door_ring"),
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        material=white,
        name="door_ring",
    )
    main_door.visual(
        Cylinder(radius=0.176, length=0.016),
        origin=Origin(xyz=(0.270, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="main_glass",
    )
    main_door.visual(
        Cylinder(radius=0.132, length=0.010),
        origin=Origin(xyz=(0.270, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="inner_smoke_lens",
    )
    main_door.visual(
        mesh_from_geometry(
            _rounded_frame(0.170, 0.130, 0.150, 0.110, 0.008, 0.018),
            "add_door_surround",
        ),
        origin=Origin(xyz=(0.270, -0.0315, 0.105)),
        material=charcoal,
        name="add_frame",
    )
    main_door.visual(
        Box((0.043, 0.018, 0.490)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=metal,
        name="hinge_leaf",
    )
    for i, z in enumerate((-0.178, 0.0, 0.178)):
        main_door.visual(
            Cylinder(radius=0.018, length=0.102),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=metal,
            name=f"hinge_knuckle_{i}",
        )
    main_door.visual(
        Box((0.030, 0.020, 0.080)),
        origin=Origin(xyz=(0.500, -0.020, 0.015)),
        material=white,
        name="pull_lip",
    )

    add_door = model.part("add_door")
    # The add-door child frame is its own left hinge line, set into the upper
    # part of the main porthole door.  Its panel also extends along local +X.
    add_door.visual(
        mesh_from_geometry(_rounded_panel(0.145, 0.105, 0.018, 0.014), "add_panel"),
        origin=Origin(xyz=(0.0725, 0.0, 0.0)),
        material=glass,
        name="add_panel",
    )
    add_door.visual(
        mesh_from_geometry(_rounded_frame(0.156, 0.116, 0.132, 0.092, 0.010, 0.014), "add_trim"),
        origin=Origin(xyz=(0.0725, -0.006, 0.0)),
        material=white,
        name="add_trim",
    )
    add_door.visual(
        Cylinder(radius=0.008, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="add_hinge_barrel",
    )
    add_door.visual(
        Box((0.020, 0.014, 0.055)),
        origin=Origin(xyz=(0.146, -0.006, -0.004)),
        material=white,
        name="add_pull_tab",
    )

    knob = model.part("program_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.078,
                0.032,
                body_style="skirted",
                top_diameter=0.064,
                grip=KnobGrip(style="fluted", count=24, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "program_knob",
        ),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="knob_cap",
    )

    button_origins = [(0.215, -0.340, 0.828), (0.265, -0.340, 0.828), (0.315, -0.340, 0.828)]
    for i, _origin in enumerate(button_origins):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.030, 0.012, 0.020)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=charcoal,
            name="button_cap",
        )

    main_hinge = model.articulation(
        "main_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=main_door,
        origin=Origin(xyz=(-0.270, -0.355, 0.470)),
        # The closed door extends along local +X; -Z makes positive q swing the
        # free edge outward toward the viewer/front (-Y).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    main_hinge.meta["qc_samples"] = [0.0, 0.55, 1.2, 1.75]

    add_hinge = model.articulation(
        "add_hinge",
        ArticulationType.REVOLUTE,
        parent=main_door,
        child=add_door,
        origin=Origin(xyz=(0.195, -0.0435, 0.105)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    add_hinge.meta["qc_samples"] = [0.0, 0.7, 1.35]

    model.articulation(
        "knob_spin",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.110, -0.340, 0.825)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-math.pi, upper=math.pi),
    )
    for i, origin_xyz in enumerate(button_origins):
        model.articulation(
            f"button_press_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=model.get_part(f"button_{i}"),
            origin=Origin(xyz=origin_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    main_door = object_model.get_part("main_door")
    add_door = object_model.get_part("add_door")
    main_hinge = object_model.get_articulation("main_hinge")
    add_hinge = object_model.get_articulation("add_hinge")

    with ctx.pose({main_hinge: 0.0, add_hinge: 0.0}):
        ctx.expect_gap(
            cabinet,
            main_door,
            axis="y",
            positive_elem="porthole_gasket",
            negative_elem="door_ring",
            min_gap=0.001,
            max_gap=0.010,
            name="main door closes just proud of gasket",
        )
        ctx.expect_overlap(
            main_door,
            cabinet,
            axes="xz",
            elem_a="door_ring",
            elem_b="porthole_gasket",
            min_overlap=0.38,
            name="main porthole ring covers cabinet opening",
        )
        ctx.expect_within(
            add_door,
            main_door,
            axes="xz",
            inner_elem="add_panel",
            outer_elem="add_frame",
            margin=0.004,
            name="add-door panel fits within inset surround",
        )
        ctx.expect_contact(
            add_door,
            main_door,
            elem_a="add_hinge_barrel",
            elem_b="add_frame",
            contact_tol=0.001,
            name="add-door hinge barrel is seated in the inset frame",
        )

    rest_aabb = ctx.part_element_world_aabb(main_door, elem="door_ring")
    with ctx.pose({main_hinge: 1.20, add_hinge: 0.0}):
        open_aabb = ctx.part_element_world_aabb(main_door, elem="door_ring")
    ctx.check(
        "main hinge swings porthole door outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.12,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    add_rest_aabb = ctx.part_element_world_aabb(add_door, elem="add_panel")
    with ctx.pose({main_hinge: 0.0, add_hinge: 0.95}):
        add_open_aabb = ctx.part_element_world_aabb(add_door, elem="add_panel")
    ctx.check(
        "secondary add-door swings outward independently",
        add_rest_aabb is not None
        and add_open_aabb is not None
        and add_open_aabb[0][1] < add_rest_aabb[0][1] - 0.035,
        details=f"rest_aabb={add_rest_aabb}, open_aabb={add_open_aabb}",
    )

    ctx.check(
        "main and add-door hinges have realistic travel",
        main_hinge.motion_limits is not None
        and add_hinge.motion_limits is not None
        and main_hinge.motion_limits.lower == 0.0
        and main_hinge.motion_limits.upper >= 1.5
        and add_hinge.motion_limits.lower == 0.0
        and add_hinge.motion_limits.upper >= 1.1,
        details=f"main={main_hinge.motion_limits}, add={add_hinge.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
