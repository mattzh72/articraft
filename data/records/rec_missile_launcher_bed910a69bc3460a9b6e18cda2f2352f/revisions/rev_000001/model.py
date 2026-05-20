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


def _offset_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _canister_shell_mesh():
    """One continuous four-cell open launch-canister extrusion.

    The mesh is authored along local +Z and later rotated so that the cells run
    along the canister pack's local +X direction.
    """

    outer = rounded_rect_profile(1.08, 1.28, radius=0.055, corner_segments=8)
    cell = rounded_rect_profile(0.39, 0.48, radius=0.035, corner_segments=8)
    holes = []
    for zc in (-0.255, 0.255):
        for yc in (-0.305, 0.305):
            holes.append(_offset_profile(cell, zc, yc))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, 2.74, center=True),
        "four_cell_canister_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trailer_mounted_four_cell_launcher")

    olive = model.material("olive_drab", rgba=(0.30, 0.35, 0.22, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.20, 0.24, 0.16, 1.0))
    steel = model.material("dark_steel", rgba=(0.13, 0.14, 0.15, 1.0))
    deck_gray = model.material("painted_deck_gray", rgba=(0.36, 0.39, 0.38, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.025, 0.024, 0.022, 1.0))
    warning = model.material("warning_amber", rgba=(0.95, 0.56, 0.10, 1.0))

    trailer = model.part("trailer")
    trailer.visual(
        Box((3.45, 1.72, 0.18)),
        origin=Origin(xyz=(0.05, 0.0, 0.55)),
        material=deck_gray,
        name="deck_plate",
    )
    trailer.visual(
        Box((3.75, 0.16, 0.16)),
        origin=Origin(xyz=(0.03, 0.68, 0.40)),
        material=steel,
        name="side_rail_0",
    )
    trailer.visual(
        Box((3.75, 0.16, 0.16)),
        origin=Origin(xyz=(0.03, -0.68, 0.40)),
        material=steel,
        name="side_rail_1",
    )
    trailer.visual(
        Box((0.16, 1.48, 0.14)),
        origin=Origin(xyz=(-1.58, 0.0, 0.41)),
        material=steel,
        name="front_crossmember",
    )
    trailer.visual(
        Box((0.16, 1.48, 0.14)),
        origin=Origin(xyz=(1.62, 0.0, 0.41)),
        material=steel,
        name="rear_crossmember",
    )
    trailer.visual(
        Cylinder(radius=0.055, length=2.20),
        origin=Origin(xyz=(0.82, 0.0, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wheel_axle",
    )
    for index, y in enumerate((-1.04, 1.04)):
        trailer.visual(
            Cylinder(radius=0.37, length=0.25),
            origin=Origin(xyz=(0.82, y, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"tire_{index}",
        )
        trailer.visual(
            Cylinder(radius=0.20, length=0.27),
            origin=Origin(xyz=(0.82, y, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=deck_gray,
            name=f"wheel_hub_{index}",
        )
    trailer.visual(
        Box((1.25, 0.16, 0.14)),
        origin=Origin(xyz=(-1.95, 0.12, 0.42), rpy=(0.0, 0.0, 0.15)),
        material=steel,
        name="tow_bar_0",
    )
    trailer.visual(
        Box((1.25, 0.16, 0.14)),
        origin=Origin(xyz=(-1.95, -0.12, 0.42), rpy=(0.0, 0.0, -0.15)),
        material=steel,
        name="tow_bar_1",
    )
    trailer.visual(
        Cylinder(radius=0.13, length=0.06),
        origin=Origin(xyz=(-2.58, 0.0, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tow_eye",
    )
    trailer.visual(
        Cylinder(radius=0.54, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        material=steel,
        name="fixed_bearing_race",
    )
    trailer.visual(
        Cylinder(radius=0.36, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        material=dark_olive,
        name="turntable_socket",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.62, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_olive,
        name="rotating_bearing",
    )
    turntable.visual(
        Cylinder(radius=0.78, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=olive,
        name="slew_plate",
    )
    turntable.visual(
        Box((1.20, 1.70, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 0.27)),
        material=olive,
        name="yoke_footplate",
    )
    turntable.visual(
        Box((0.56, 1.00, 0.42)),
        origin=Origin(xyz=(-0.05, 0.0, 0.50)),
        material=dark_olive,
        name="elevation_drive_housing",
    )
    for index, y in enumerate((-0.86, 0.86)):
        turntable.visual(
            Box((0.22, 0.16, 1.25)),
            origin=Origin(xyz=(0.0, y, 0.92)),
            material=olive,
            name=f"yoke_arm_{index}",
        )
        turntable.visual(
            Cylinder(radius=0.23, length=0.18),
            origin=Origin(xyz=(0.0, y, 1.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"trunnion_bearing_{index}",
        )
    turntable.visual(
        Box((0.18, 1.48, 0.16)),
        origin=Origin(xyz=(-0.20, 0.0, 1.02)),
        material=dark_olive,
        name="rear_yoke_tie",
    )
    turntable.visual(
        Cylinder(radius=0.10, length=1.72),
        origin=Origin(xyz=(-0.18, 0.0, 0.86), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hydraulic_cross_tube",
    )

    canister = model.part("canister_pack")
    canister.visual(
        _canister_shell_mesh(),
        origin=Origin(xyz=(1.37, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="canister_shell",
    )
    canister.visual(
        Box((0.16, 1.36, 0.86)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=dark_olive,
        name="rear_frame",
    )
    canister.visual(
        Box((0.12, 1.42, 0.12)),
        origin=Origin(xyz=(2.72, 0.0, 0.58)),
        material=dark_olive,
        name="front_top_rail",
    )
    canister.visual(
        Box((0.12, 1.42, 0.12)),
        origin=Origin(xyz=(2.72, 0.0, -0.58)),
        material=dark_olive,
        name="front_bottom_rail",
    )
    for index, y in enumerate((-0.705, 0.705)):
        canister.visual(
            Cylinder(radius=0.14, length=0.13),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"trunnion_pin_{index}",
        )
    canister.visual(
        Box((0.38, 0.10, 0.16)),
        origin=Origin(xyz=(0.32, -0.68, -0.44)),
        material=steel,
        name="elevation_actuator_lug",
    )
    canister.visual(
        Box((0.07, 0.07, 0.48)),
        origin=Origin(xyz=(1.200, 0.645, 0.33)),
        material=dark_olive,
        name="panel_hinge_mount",
    )
    canister.visual(
        Box((0.18, 0.06, 0.10)),
        origin=Origin(xyz=(2.16, 0.665, -0.38)),
        material=warning,
        name="safety_label",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.52, 0.040, 0.42)),
        origin=Origin(xyz=(0.26, 0.026, 0.0)),
        material=dark_olive,
        name="panel_door",
    )
    service_panel.visual(
        Cylinder(radius=0.028, length=0.48),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    service_panel.visual(
        Box((0.16, 0.030, 0.055)),
        origin=Origin(xyz=(0.41, 0.061, 0.03)),
        material=steel,
        name="pull_handle",
    )
    service_panel.visual(
        Box((0.10, 0.012, 0.030)),
        origin=Origin(xyz=(0.18, 0.052, -0.15)),
        material=warning,
        name="status_plate",
    )

    model.articulation(
        "turntable_yaw",
        ArticulationType.REVOLUTE,
        parent=trailer,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.45, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "canister_elevation",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=canister,
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.35, lower=0.0, upper=1.05),
    )
    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=canister,
        child=service_panel,
        origin=Origin(xyz=(1.26, 0.655, 0.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    trailer = object_model.get_part("trailer")
    canister = object_model.get_part("canister_pack")
    service_panel = object_model.get_part("service_panel")
    yaw = object_model.get_articulation("turntable_yaw")
    elevation = object_model.get_articulation("canister_elevation")
    panel_hinge = object_model.get_articulation("panel_hinge")

    ctx.expect_gap(
        canister,
        trailer,
        axis="z",
        min_gap=0.70,
        name="canister pack is carried well above the trailer deck",
    )
    ctx.expect_gap(
        service_panel,
        canister,
        axis="y",
        positive_elem="panel_door",
        negative_elem="canister_shell",
        min_gap=0.0,
        max_gap=0.04,
        name="service panel sits just outside the canister side",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_door")
    with ctx.pose({panel_hinge: 1.20}):
        open_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_door")
    ctx.check(
        "service panel opens outward on its side hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.20,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    rest_canister_aabb = ctx.part_element_world_aabb(canister, elem="canister_shell")
    with ctx.pose({elevation: 0.85}):
        elevated_canister_aabb = ctx.part_element_world_aabb(canister, elem="canister_shell")
    ctx.check(
        "elevation yoke pitches the launch cells upward",
        rest_canister_aabb is not None
        and elevated_canister_aabb is not None
        and elevated_canister_aabb[1][2] > rest_canister_aabb[1][2] + 0.75,
        details=f"rest={rest_canister_aabb}, elevated={elevated_canister_aabb}",
    )

    rest_yaw_aabb = ctx.part_element_world_aabb(canister, elem="canister_shell")
    with ctx.pose({yaw: 0.75}):
        slewed_yaw_aabb = ctx.part_element_world_aabb(canister, elem="canister_shell")
    ctx.check(
        "turntable slews the elevated launcher about the vertical axis",
        rest_yaw_aabb is not None
        and slewed_yaw_aabb is not None
        and slewed_yaw_aabb[1][1] > rest_yaw_aabb[1][1] + 1.0,
        details=f"rest={rest_yaw_aabb}, slewed={slewed_yaw_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
