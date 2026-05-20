from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRONT_RPY = (math.pi / 2.0, 0.0, 0.0)


def _front_cylinder(part, *, name, radius, length, xyz, material):
    """Add a disk/cap whose axis points out of the shutter face."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=FRONT_RPY),
        material=material,
        name=name,
    )


def _add_screw_grid(part, *, prefix, points, radius, material):
    for index, xyz in enumerate(points):
        _front_cylinder(
            part,
            name=f"{prefix}_{index}",
            radius=radius,
            length=0.012,
            xyz=xyz,
            material=material,
        )


def _add_panel(panel, *, handed: int, metal, dark, hardware) -> None:
    """Build one hinged storm-shutter leaf.

    handed=+1 extends from its hinge toward +X; handed=-1 mirrors it.
    """
    width = 0.455
    sheet_start = 0.025
    sheet_width = width - sheet_start
    height = 1.12
    x_sheet = handed * (sheet_start + sheet_width / 2.0)

    panel.visual(
        Box((sheet_width, 0.026, height)),
        origin=Origin(xyz=(x_sheet, 0.0, 0.0)),
        material=metal,
        name="storm_sheet",
    )
    panel.visual(
        Box((0.060, 0.022, height)),
        origin=Origin(xyz=(handed * 0.055, -0.021, 0.0)),
        material=metal,
        name="hinge_stile",
    )
    panel.visual(
        Box((0.055, 0.023, height)),
        origin=Origin(xyz=(handed * (width - 0.0275), -0.022, 0.0)),
        material=metal,
        name="meeting_stile",
    )
    for z, name in ((0.5225, "top_rail"), (-0.5225, "bottom_rail")):
        panel.visual(
            Box((sheet_width, 0.021, 0.075)),
            origin=Origin(xyz=(x_sheet, -0.022, z)),
            material=metal,
            name=name,
        )
    panel.visual(
        Box((sheet_width, 0.020, 0.055)),
        origin=Origin(xyz=(x_sheet, -0.021, 0.0)),
        material=metal,
        name="middle_rail",
    )

    # Raised ribs make the leaf read as reinforced sheet-metal paneling rather
    # than a plain slab.
    for index, z in enumerate((-0.405, -0.300, -0.195, 0.195, 0.300, 0.405)):
        panel.visual(
            Box((0.305, 0.017, 0.022)),
            origin=Origin(xyz=(handed * 0.238, -0.031, z)),
            material=metal,
            name=f"panel_rib_{index}",
        )

    brace_angle = math.atan2(0.92, 0.305)
    panel.visual(
        Box((0.970, 0.018, 0.042)),
        origin=Origin(xyz=(handed * 0.240, -0.037, 0.0), rpy=(0.0, -handed * brace_angle, 0.0)),
        material=metal,
        name="diagonal_brace",
    )

    # Compressible-looking perimeter seal visible around the storm leaf.
    panel.visual(
        Box((sheet_width, 0.009, 0.018)),
        origin=Origin(xyz=(x_sheet, -0.034, 0.552)),
        material=dark,
        name="top_seal",
    )
    panel.visual(
        Box((sheet_width, 0.009, 0.018)),
        origin=Origin(xyz=(x_sheet, -0.034, -0.552)),
        material=dark,
        name="bottom_seal",
    )
    panel.visual(
        Box((0.018, 0.009, 1.06)),
        origin=Origin(xyz=(handed * (width - 0.010), -0.041, 0.0)),
        material=dark,
        name="meeting_seal",
    )

    # Three heavy hinge straps and the moving knuckles. Static frame knuckles
    # occupy the spaces above and below these center knuckles.
    for index, z in enumerate((-0.405, 0.0, 0.405)):
        panel.visual(
            Box((0.160, 0.023, 0.050)),
            origin=Origin(xyz=(handed * 0.080, -0.022, z)),
            material=hardware,
            name=f"hinge_strap_{index}",
        )
        panel.visual(
            Cylinder(radius=0.013, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hardware,
            name=f"hinge_knuckle_{index}",
        )
        _front_cylinder(
            panel,
            name=f"strap_bolt_{index}_0",
            radius=0.010,
            length=0.006,
            xyz=(handed * 0.048, -0.036, z),
            material=hardware,
        )
        _front_cylinder(
            panel,
            name=f"strap_bolt_{index}_1",
            radius=0.010,
            length=0.006,
            xyz=(handed * 0.125, -0.036, z),
            material=hardware,
        )

    # Smaller field fasteners on the reinforced rails.
    bolt_points = [
        (handed * 0.170, -0.036, 0.522),
        (handed * 0.330, -0.036, 0.522),
        (handed * 0.170, -0.036, -0.522),
        (handed * 0.330, -0.036, -0.522),
        (handed * 0.170, -0.036, 0.000),
        (handed * 0.330, -0.036, 0.000),
    ]
    _add_screw_grid(panel, prefix="rail_bolt", points=bolt_points, radius=0.008, material=hardware)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storm_shutter_assembly")

    storm_metal = model.material("powder_coated_aluminum", rgba=(0.43, 0.49, 0.50, 1.0))
    frame_metal = model.material("galvanized_frame", rgba=(0.56, 0.59, 0.58, 1.0))
    dark_gasket = model.material("black_weather_gasket", rgba=(0.015, 0.017, 0.016, 1.0))
    hardware = model.material("stainless_hardware", rgba=(0.72, 0.72, 0.68, 1.0))
    glass = model.material("dark_window_glass", rgba=(0.05, 0.075, 0.10, 0.82))

    frame = model.part("outer_frame")
    # Architectural frame around a real opening, not a solid placeholder.
    frame.visual(
        Box((0.120, 0.100, 1.360)),
        origin=Origin(xyz=(-0.540, 0.0, 0.0)),
        material=frame_metal,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.120, 0.100, 1.360)),
        origin=Origin(xyz=(0.540, 0.0, 0.0)),
        material=frame_metal,
        name="side_rail_1",
    )
    frame.visual(
        Box((1.200, 0.100, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        material=frame_metal,
        name="top_rail",
    )
    frame.visual(
        Box((1.200, 0.100, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, -0.660)),
        material=frame_metal,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.960, 0.012, 1.200)),
        origin=Origin(xyz=(0.0, 0.056, 0.0)),
        material=glass,
        name="protected_glass",
    )
    frame.visual(
        Box((1.280, 0.140, 0.035)),
        origin=Origin(xyz=(0.0, -0.035, 0.7375)),
        material=frame_metal,
        name="drip_cap",
    )
    frame.visual(
        Box((1.220, 0.155, 0.040)),
        origin=Origin(xyz=(0.0, -0.028, -0.740)),
        material=frame_metal,
        name="sill_lip",
    )

    # Black compression gaskets set into the inner frame.
    frame.visual(
        Box((0.014, 0.014, 1.180)),
        origin=Origin(xyz=(-0.473, -0.056, 0.0)),
        material=dark_gasket,
        name="side_gasket_0",
    )
    frame.visual(
        Box((0.014, 0.014, 1.180)),
        origin=Origin(xyz=(0.473, -0.056, 0.0)),
        material=dark_gasket,
        name="side_gasket_1",
    )
    frame.visual(
        Box((0.920, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.056, 0.593)),
        material=dark_gasket,
        name="top_gasket",
    )
    frame.visual(
        Box((0.920, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.056, -0.593)),
        material=dark_gasket,
        name="bottom_gasket",
    )

    # Frame screws and hinge plates are intentionally proud of the face so the
    # assembly reads as hurricane-rated architectural hardware.
    frame_screws = []
    for x in (-0.540, 0.540):
        for z in (-0.520, -0.260, 0.260, 0.520):
            frame_screws.append((x, -0.055, z))
    for x in (-0.360, -0.120, 0.120, 0.360):
        frame_screws.append((x, -0.055, 0.660))
        frame_screws.append((x, -0.055, -0.660))
    _add_screw_grid(frame, prefix="frame_screw", points=frame_screws, radius=0.014, material=hardware)

    for side in (-1, 1):
        for group, z in enumerate((-0.405, 0.0, 0.405)):
            frame.visual(
                Box((0.075, 0.030, 0.145)),
                origin=Origin(xyz=(side * 0.505, -0.050, z)),
                material=hardware,
                name=f"hinge_plate_{side}_{group}",
            )
            for offset_index, offset in enumerate((-0.045, 0.045)):
                frame.visual(
                    Box((0.032, 0.018, 0.030)),
                    origin=Origin(xyz=(side * 0.492, -0.073, z + offset)),
                    material=hardware,
                    name=f"barrel_tab_{side}_{group}_{offset_index}",
                )
                frame.visual(
                    Cylinder(radius=0.014, length=0.040),
                    origin=Origin(xyz=(side * 0.480, -0.085, z + offset)),
                    material=hardware,
                    name=f"hinge_knuckle_{side}_{group}_{offset_index}",
                )
            _front_cylinder(
                frame,
                name=f"hinge_bolt_{side}_{group}_0",
                radius=0.009,
                length=0.006,
                xyz=(side * 0.522, -0.068, z + 0.038),
                material=hardware,
            )
            _front_cylinder(
                frame,
                name=f"hinge_bolt_{side}_{group}_1",
                radius=0.009,
                length=0.006,
                xyz=(side * 0.522, -0.068, z - 0.038),
                material=hardware,
            )

    panel_0 = model.part("panel_0")
    _add_panel(panel_0, handed=1, metal=storm_metal, dark=dark_gasket, hardware=hardware)
    # Boss for the rotating storm latch handle.
    _front_cylinder(
        panel_0,
        name="latch_boss",
        radius=0.035,
        length=0.022,
        xyz=(0.425, -0.039, -0.300),
        material=hardware,
    )

    panel_1 = model.part("panel_1")
    _add_panel(panel_1, handed=-1, metal=storm_metal, dark=dark_gasket, hardware=hardware)
    # Keeper ears capture the handle without intersecting it at the locked pose.
    panel_1.visual(
        Box((0.070, 0.019, 0.024)),
        origin=Origin(xyz=(-0.425, -0.033, -0.345)),
        material=hardware,
        name="keeper_ear_0",
    )
    panel_1.visual(
        Box((0.070, 0.019, 0.024)),
        origin=Origin(xyz=(-0.425, -0.033, -0.255)),
        material=hardware,
        name="keeper_ear_1",
    )
    panel_1.visual(
        Box((0.020, 0.019, 0.115)),
        origin=Origin(xyz=(-0.455, -0.033, -0.300)),
        material=hardware,
        name="keeper_backer",
    )

    lock_handle = model.part("lock_handle")
    lock_handle.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=FRONT_RPY),
        material=hardware,
        name="pivot_disk",
    )
    lock_handle.visual(
        Box((0.150, 0.014, 0.032)),
        origin=Origin(xyz=(0.075, -0.006, 0.0)),
        material=hardware,
        name="handle_bar",
    )
    lock_handle.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.150, -0.006, 0.0), rpy=FRONT_RPY),
        material=hardware,
        name="grip_cap",
    )

    model.articulation(
        "frame_to_panel_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel_0,
        origin=Origin(xyz=(-0.480, -0.085, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.0, lower=0.0, upper=1.90),
    )
    model.articulation(
        "frame_to_panel_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel_1,
        origin=Origin(xyz=(0.480, -0.085, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.0, lower=0.0, upper=1.90),
    )
    model.articulation(
        "panel_to_lock_handle",
        ArticulationType.REVOLUTE,
        parent=panel_0,
        child=lock_handle,
        origin=Origin(xyz=(0.425, -0.050, -0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    panel_0 = object_model.get_part("panel_0")
    panel_1 = object_model.get_part("panel_1")
    handle = object_model.get_part("lock_handle")
    joint_0 = object_model.get_articulation("frame_to_panel_0")
    joint_1 = object_model.get_articulation("frame_to_panel_1")
    latch_joint = object_model.get_articulation("panel_to_lock_handle")

    with ctx.pose({joint_0: 0.0, joint_1: 0.0, latch_joint: 0.0}):
        ctx.expect_overlap(
            panel_0,
            frame,
            axes="xz",
            min_overlap=0.35,
            elem_a="storm_sheet",
            elem_b="protected_glass",
            name="first leaf covers the protected opening",
        )
        ctx.expect_overlap(
            panel_1,
            frame,
            axes="xz",
            min_overlap=0.35,
            elem_a="storm_sheet",
            elem_b="protected_glass",
            name="second leaf covers the protected opening",
        )
        ctx.expect_gap(
            frame,
            panel_0,
            axis="y",
            min_gap=0.005,
            max_gap=0.080,
            positive_elem="top_gasket",
            negative_elem="storm_sheet",
            name="closed leaves stand proud of the gasketed frame",
        )
        ctx.expect_overlap(
            handle,
            panel_1,
            axes="x",
            min_overlap=0.015,
            elem_a="handle_bar",
            elem_b="keeper_backer",
            name="lock handle reaches the keeper side",
        )

    rest_0 = ctx.part_world_aabb(panel_0)
    rest_1 = ctx.part_world_aabb(panel_1)
    with ctx.pose({joint_0: 1.20, joint_1: 1.20, latch_joint: 0.0}):
        open_0 = ctx.part_world_aabb(panel_0)
        open_1 = ctx.part_world_aabb(panel_1)

    ctx.check(
        "both panels swing outward",
        rest_0 is not None
        and rest_1 is not None
        and open_0 is not None
        and open_1 is not None
        and open_0[0][1] < rest_0[0][1] - 0.05
        and open_1[0][1] < rest_1[0][1] - 0.05,
        details=f"rest_0={rest_0}, open_0={open_0}, rest_1={rest_1}, open_1={open_1}",
    )

    handle_rest = ctx.part_element_world_aabb(handle, elem="handle_bar")
    with ctx.pose({latch_joint: 1.20}):
        handle_rotated = ctx.part_element_world_aabb(handle, elem="handle_bar")
    ctx.check(
        "lock handle rotates upward",
        handle_rest is not None
        and handle_rotated is not None
        and handle_rotated[1][2] > handle_rest[1][2] + 0.02,
        details=f"rest={handle_rest}, rotated={handle_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
