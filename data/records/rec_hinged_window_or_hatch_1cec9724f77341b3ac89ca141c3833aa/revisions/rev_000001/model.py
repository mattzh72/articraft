from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cellar_hatch_door")

    galvanized = Material("galvanized_steel", rgba=(0.56, 0.58, 0.56, 1.0))
    dark_steel = Material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    insulated_panel = Material("blue_gray_insulated_panel", rgba=(0.38, 0.46, 0.50, 1.0))
    foam_edge = Material("pale_insulation_edge", rgba=(0.78, 0.72, 0.58, 1.0))
    rubber = Material("black_rubber_gasket", rgba=(0.01, 0.01, 0.01, 1.0))

    frame = model.part("opening_frame")
    # A heavy rectangular steel curb around the cellar opening.
    frame.visual(Box((0.12, 1.00, 0.06)), origin=Origin(xyz=(-0.64, 0.0, 0.03)), material=galvanized, name="hinge_rail")
    frame.visual(Box((0.12, 1.00, 0.06)), origin=Origin(xyz=(0.64, 0.0, 0.03)), material=galvanized, name="strike_rail")
    frame.visual(Box((1.40, 0.12, 0.06)), origin=Origin(xyz=(0.0, 0.44, 0.03)), material=galvanized, name="front_rail")
    frame.visual(Box((1.40, 0.12, 0.06)), origin=Origin(xyz=(0.0, -0.44, 0.03)), material=galvanized, name="rear_rail")
    frame.visual(Box((0.060, 0.20, 0.012)), origin=Origin(xyz=(0.655, 0.0, 0.066)), material=dark_steel, name="lock_keeper")

    # Raised hinge pads lift the fixed leaves to the top surface of the thick lid.
    for idx, y in enumerate((-0.25, 0.25)):
        frame.visual(Box((0.09, 0.22, 0.108)), origin=Origin(xyz=(-0.655, y, 0.114)), material=galvanized, name=f"hinge_pad_{idx}")
        frame.visual(Box((0.095, 0.19, 0.006)), origin=Origin(xyz=(-0.630, y, 0.171)), material=dark_steel, name=f"fixed_leaf_{idx}")
        # Two fixed knuckles per hinge; the moving middle knuckle belongs to the lid.
        for suffix, dy in (("lower", -0.072), ("upper", 0.072)):
            frame.visual(
                Box((0.014, 0.055, 0.016)),
                origin=Origin(xyz=(-0.598, y + dy, 0.178)),
                material=dark_steel,
                name=f"fixed_leaf_curl_{idx}_{suffix}",
            )
            frame.visual(
                Cylinder(radius=0.015, length=0.055),
                origin=Origin(xyz=(-0.60, y + dy, 0.185), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"fixed_knuckle_{idx}_{suffix}",
            )

    lid = model.part("lid")
    # The child frame is the hinge line.  The sandwich panel extends inward along +X.
    lid.visual(Box((1.18, 0.78, 0.088)), origin=Origin(xyz=(0.60, 0.0, -0.073)), material=foam_edge, name="insulation_core")
    lid.visual(Box((1.20, 0.80, 0.012)), origin=Origin(xyz=(0.60, 0.0, -0.023)), material=insulated_panel, name="top_skin")
    lid.visual(Box((1.20, 0.030, 0.020)), origin=Origin(xyz=(0.60, 0.405, -0.073)), material=insulated_panel, name="front_edge_band")
    lid.visual(Box((1.20, 0.030, 0.020)), origin=Origin(xyz=(0.60, -0.405, -0.073)), material=insulated_panel, name="rear_edge_band")
    lid.visual(Box((0.030, 0.80, 0.020)), origin=Origin(xyz=(1.205, 0.0, -0.073)), material=insulated_panel, name="free_edge_band")
    lid.visual(Box((0.020, 0.80, 0.020)), origin=Origin(xyz=(0.015, 0.0, -0.073)), material=insulated_panel, name="hinge_edge_band")

    # Compressible perimeter gasket under the lid, shown clear of the frame at rest.
    lid.visual(Box((0.040, 0.74, 0.008)), origin=Origin(xyz=(0.035, 0.0, -0.121)), material=rubber, name="hinge_gasket")
    lid.visual(Box((0.040, 0.74, 0.008)), origin=Origin(xyz=(1.165, 0.0, -0.121)), material=rubber, name="free_gasket")
    lid.visual(Box((1.12, 0.030, 0.008)), origin=Origin(xyz=(0.60, 0.385, -0.121)), material=rubber, name="front_gasket")
    lid.visual(Box((1.12, 0.030, 0.008)), origin=Origin(xyz=(0.60, -0.385, -0.121)), material=rubber, name="rear_gasket")

    for idx, y in enumerate((-0.25, 0.25)):
        lid.visual(Box((0.150, 0.15, 0.006)), origin=Origin(xyz=(0.090, y, -0.014)), material=dark_steel, name=f"moving_leaf_{idx}")
        lid.visual(Box((0.018, 0.080, 0.016)), origin=Origin(xyz=(0.024, y, -0.007)), material=dark_steel, name=f"moving_leaf_curl_{idx}")
        lid.visual(
            Cylinder(radius=0.015, length=0.080),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"moving_knuckle_{idx}",
        )

    # A fixed boss around the handle pivot on the lid near the free edge.
    lid.visual(Cylinder(radius=0.065, length=0.012), origin=Origin(xyz=(1.08, 0.0, -0.011)), material=dark_steel, name="handle_boss")

    model.articulation(
        "frame_to_lid",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(-0.60, 0.0, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    handle = model.part("locking_handle")
    handle.visual(Cylinder(radius=0.052, length=0.024), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=dark_steel, name="pivot_hub")
    handle.visual(Box((0.055, 0.34, 0.026)), origin=Origin(xyz=(0.0, 0.0, 0.037)), material=dark_steel, name="lever_bar")
    handle.visual(Cylinder(radius=0.027, length=0.035), origin=Origin(xyz=(0.0, 0.16, 0.067)), material=galvanized, name="finger_knob")
    handle.visual(Box((0.12, 0.030, 0.014)), origin=Origin(xyz=(0.075, 0.0, 0.020)), material=galvanized, name="lock_pointer")

    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(1.08, 0.0, -0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-pi / 2.0, upper=pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("opening_frame")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("locking_handle")
    lid_hinge = object_model.get_articulation("frame_to_lid")
    handle_pivot = object_model.get_articulation("lid_to_handle")

    ctx.expect_contact(
        lid,
        frame,
        elem_a="free_gasket",
        elem_b="strike_rail",
        contact_tol=0.001,
        name="closed gasket rests on strike rail",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a="pivot_hub",
        elem_b="handle_boss",
        contact_tol=0.001,
        name="handle hub bears on lid boss",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="free_edge_band")
    with ctx.pose({lid_hinge: 1.15}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="free_edge_band")
    ctx.check(
        "side hinge lifts free edge upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.45,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="lever_bar")
    with ctx.pose({handle_pivot: pi / 2.0}):
        turned_handle_aabb = ctx.part_element_world_aabb(handle, elem="lever_bar")
    ctx.check(
        "locking handle turns across the lid",
        closed_handle_aabb is not None
        and turned_handle_aabb is not None
        and (closed_handle_aabb[1][1] - closed_handle_aabb[0][1]) > 0.25
        and (turned_handle_aabb[1][0] - turned_handle_aabb[0][0]) > 0.25,
        details=f"closed={closed_handle_aabb}, turned={turned_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
