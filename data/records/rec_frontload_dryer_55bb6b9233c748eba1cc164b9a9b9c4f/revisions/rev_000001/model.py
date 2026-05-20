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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _annulus_on_front(
    *,
    center_y: float,
    center_z: float,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    center_x: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(center_y, center_z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness, both=True)
        .translate((center_x, 0.0, 0.0))
    )


def _build_cabinet_shell() -> cq.Workplane:
    depth = 0.56
    width = 0.60
    height = 0.84
    wall = 0.025
    front_x = depth / 2.0 - wall / 2.0
    back_x = -depth / 2.0 + wall / 2.0

    shell = _cq_box((wall, width, height), (back_x, 0.0, height / 2.0))
    shell = shell.union(_cq_box((depth, wall, height), (0.0, -width / 2.0 + wall / 2.0, height / 2.0)))
    shell = shell.union(_cq_box((depth, wall, height), (0.0, width / 2.0 - wall / 2.0, height / 2.0)))
    shell = shell.union(_cq_box((depth, width, wall), (0.0, 0.0, wall / 2.0)))
    shell = shell.union(_cq_box((depth, width, wall), (0.0, 0.0, height - wall / 2.0)))

    # Continuous front appliance frame around the door opening.
    shell = shell.union(_cq_box((wall, width, 0.18), (front_x, 0.0, 0.09)))
    shell = shell.union(_cq_box((wall, width, 0.10), (front_x, 0.0, 0.79)))
    shell = shell.union(_cq_box((wall, 0.06, 0.56), (front_x, -0.27, 0.46)))
    shell = shell.union(_cq_box((wall, 0.06, 0.56), (front_x, 0.27, 0.46)))

    # Rear bearing support boss for the rotating drum, fused to the back panel.
    boss = cq.Workplane("YZ").circle(0.045).extrude(0.04).translate((-0.255, 0.0, 0.46))
    shell = shell.union(boss)

    # Stacking lips on the top surface keep a second compact appliance located.
    shell = shell.union(_cq_box((0.18, 0.035, 0.012), (-0.12, -0.235, 0.846)))
    shell = shell.union(_cq_box((0.18, 0.035, 0.012), (-0.12, 0.235, 0.846)))
    shell = shell.union(_cq_box((0.18, 0.035, 0.012), (0.16, -0.235, 0.846)))
    shell = shell.union(_cq_box((0.18, 0.035, 0.012), (0.16, 0.235, 0.846)))

    return shell


def _build_door_panel() -> cq.Workplane:
    thickness = 0.040
    width = 0.480
    height = 0.560
    window_radius = 0.160

    panel = cq.Workplane("XY").box(thickness, width, height).translate((0.0, width / 2.0, 0.0))
    cutter = cq.Workplane("YZ").center(width / 2.0, 0.0).circle(window_radius).extrude(thickness * 3.0, both=True)
    panel = panel.cut(cutter)
    return panel


def _build_drum_basket() -> cq.Workplane:
    outer_radius = 0.205
    inner_radius = 0.184
    length = 0.385
    drum = cq.Workplane("YZ").circle(outer_radius).extrude(length / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(inner_radius).extrude((length + 0.020) / 2.0, both=True)
    drum = drum.cut(bore)

    # Three raised lifters are fused into the inner drum wall and rotate with it.
    for angle in (0.0, 120.0, 240.0):
        lifter = (
            cq.Workplane("XY")
            .box(0.300, 0.026, 0.030)
            .translate((0.010, 0.0, inner_radius - 0.010))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
        )
        drum = drum.union(lifter)

    hub = cq.Workplane("YZ").circle(0.040).extrude(0.040, both=True).translate((-0.210, 0.0, 0.0))
    drum = drum.union(hub)
    for angle in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(0.026, 0.175, 0.018)
            .translate((-0.183, 0.112, 0.0))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
        )
        drum = drum.union(spoke)

    return drum

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_stackable_ventless_dryer")

    warm_white = model.material("warm_white_powdercoat", rgba=(0.86, 0.85, 0.80, 1.0))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("smoked_transparent_glass", rgba=(0.18, 0.24, 0.28, 0.42))
    stainless = model.material("brushed_stainless_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    filter_plastic = model.material("pale_filter_mesh", rgba=(0.82, 0.86, 0.82, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.025, 0.600, 0.840)),
        origin=Origin(xyz=(-0.2675, 0.0, 0.420)),
        material=warm_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.560, 0.025, 0.840)),
        origin=Origin(xyz=(0.0, -0.2875, 0.420)),
        material=warm_white,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((0.560, 0.025, 0.840)),
        origin=Origin(xyz=(0.0, 0.2875, 0.420)),
        material=warm_white,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((0.560, 0.600, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=warm_white,
        name="base_panel",
    )
    cabinet.visual(
        Box((0.560, 0.600, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.8275)),
        material=warm_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.025, 0.600, 0.095)),
        origin=Origin(xyz=(0.2675, 0.0, 0.0475)),
        material=warm_white,
        name="front_rail",
    )
    cabinet.visual(
        Box((0.025, 0.600, 0.100)),
        origin=Origin(xyz=(0.2675, 0.0, 0.790)),
        material=warm_white,
        name="control_rail",
    )
    cabinet.visual(
        Box((0.025, 0.060, 0.560)),
        origin=Origin(xyz=(0.2675, -0.270, 0.460)),
        material=warm_white,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.025, 0.060, 0.560)),
        origin=Origin(xyz=(0.2675, 0.270, 0.460)),
        material=warm_white,
        name="front_stile_1",
    )
    cabinet.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(-0.2675, 0.0, 0.460), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="rear_bearing_boss",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _annulus_on_front(
                center_y=0.0,
                center_z=0.46,
                outer_radius=0.248,
                inner_radius=0.188,
                thickness=0.004,
                center_x=0.279,
            ),
            "door_opening_trim",
        ),
        material=dark_plastic,
        name="door_opening_trim",
    )
    cabinet.visual(
        Box((0.148, 0.012, 0.115)),
        origin=Origin(xyz=(0.205, -0.123, 0.155)),
        material=dark_plastic,
        name="filter_guide_0",
    )
    cabinet.visual(
        Box((0.148, 0.012, 0.115)),
        origin=Origin(xyz=(0.205, 0.123, 0.155)),
        material=dark_plastic,
        name="filter_guide_1",
    )
    cabinet.visual(
        Box((0.006, 0.480, 0.115)),
        origin=Origin(xyz=(0.282, 0.0, 0.1525)),
        material=dark_plastic,
        name="filter_slot_lip",
    )
    cabinet.visual(
        Box((0.024, 0.012, 0.125)),
        origin=Origin(xyz=(0.292, -0.275, 0.275)),
        material=dark_plastic,
        name="hinge_mount_0",
    )
    cabinet.visual(
        Box((0.024, 0.012, 0.125)),
        origin=Origin(xyz=(0.292, -0.275, 0.645)),
        material=dark_plastic,
        name="hinge_mount_1",
    )
    for index, x_pos in enumerate((-0.12, 0.16)):
        for side_index, y_pos in enumerate((-0.235, 0.235)):
            cabinet.visual(
                Box((0.180, 0.035, 0.012)),
                origin=Origin(xyz=(x_pos, y_pos, 0.846)),
                material=warm_white,
                name=f"stack_lip_{index}_{side_index}",
            )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_panel(), "door_panel", tolerance=0.001),
        material=warm_white,
        name="door_panel",
    )
    door.visual(
        mesh_from_cadquery(
            _annulus_on_front(
                center_y=0.240,
                center_z=0.0,
                outer_radius=0.178,
                inner_radius=0.143,
                thickness=0.012,
                center_x=0.023,
            ),
            "window_gasket",
            tolerance=0.001,
        ),
        material=black_rubber,
        name="window_gasket",
    )
    door.visual(
        Cylinder(radius=0.146, length=0.004),
        origin=Origin(xyz=(0.031, 0.240, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="glass_window",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=dark_plastic,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=dark_plastic,
        name="hinge_barrel_1",
    )

    lint_filter = model.part("lint_filter")
    lint_filter.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.215, 0.095),
                0.006,
                slot_size=(0.026, 0.006),
                pitch=(0.034, 0.016),
                frame=0.010,
                corner_radius=0.004,
                slot_angle_deg=15.0,
                stagger=True,
            ),
            "lint_filter_face",
        ),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=filter_plastic,
        name="lint_filter_face",
    )
    lint_filter.visual(
        Box((0.170, 0.224, 0.008)),
        origin=Origin(xyz=(-0.085, 0.0, 0.049)),
        material=dark_plastic,
        name="filter_top_rail",
    )
    lint_filter.visual(
        Box((0.170, 0.224, 0.008)),
        origin=Origin(xyz=(-0.085, 0.0, -0.049)),
        material=dark_plastic,
        name="filter_bottom_rail",
    )
    lint_filter.visual(
        Box((0.170, 0.012, 0.104)),
        origin=Origin(xyz=(-0.085, -0.112, 0.0)),
        material=dark_plastic,
        name="filter_side_rail_0",
    )
    lint_filter.visual(
        Box((0.170, 0.012, 0.104)),
        origin=Origin(xyz=(-0.085, 0.112, 0.0)),
        material=dark_plastic,
        name="filter_side_rail_1",
    )
    lint_filter.visual(
        Box((0.014, 0.145, 0.030)),
        origin=Origin(xyz=(0.003, 0.0, 0.034)),
        material=dark_plastic,
        name="filter_pull_lip",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_build_drum_basket(), "perforated_drum_basket", tolerance=0.0015),
        material=stainless,
        name="drum_basket",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.305, -0.255, 0.46)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.85),
    )
    model.articulation(
        "filter_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lint_filter,
        origin=Origin(xyz=(0.270, 0.0, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.135),
    )
    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(-0.005, 0.0, 0.46)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    lint_filter = object_model.get_part("lint_filter")
    drum = object_model.get_part("drum")
    door_hinge = object_model.get_articulation("door_hinge")
    filter_slide = object_model.get_articulation("filter_slide")

    ctx.expect_gap(
        door,
        cabinet,
        axis="x",
        min_gap=0.001,
        max_gap=0.015,
        positive_elem="door_panel",
        negative_elem="door_opening_trim",
        name="closed door sits just proud of the front rim",
    )
    ctx.expect_within(
        drum,
        cabinet,
        axes="yz",
        margin=0.0,
        elem_a="drum_basket",
        elem_b="rear_panel",
        name="rotating drum is contained inside the dryer body",
    )
    ctx.expect_overlap(
        lint_filter,
        cabinet,
        axes="x",
        min_overlap=0.050,
        elem_a="filter_top_rail",
        elem_b="filter_guide_0",
        name="lint filter remains inserted in the guide at rest",
    )
    ctx.allow_overlap(
        cabinet,
        drum,
        elem_a="rear_bearing_boss",
        elem_b="drum_basket",
        reason="The rotating drum hub is intentionally captured in the rear bearing boss.",
    )
    ctx.expect_gap(
        drum,
        cabinet,
        axis="x",
        positive_elem="drum_basket",
        negative_elem="rear_bearing_boss",
        max_gap=0.002,
        max_penetration=0.006,
        name="drum hub is seated in the rear bearing boss",
    )
    ctx.expect_overlap(
        drum,
        cabinet,
        axes="yz",
        min_overlap=0.040,
        elem_a="drum_basket",
        elem_b="rear_bearing_boss",
        name="drum bearing hub is aligned with the rear boss",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door hinge opens outward from vertical edge",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][0] > closed_aabb[1][0] + 0.12,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    rest_pos = ctx.part_world_position(lint_filter)
    with ctx.pose({door_hinge: 1.25, filter_slide: 0.135}):
        extended_pos = ctx.part_world_position(lint_filter)
        ctx.expect_overlap(
            lint_filter,
            cabinet,
            axes="x",
            min_overlap=0.025,
            elem_a="filter_top_rail",
            elem_b="filter_guide_0",
            name="extended lint filter keeps retained insertion",
        )
    ctx.check(
        "lint filter slides outward from the door opening rim",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
