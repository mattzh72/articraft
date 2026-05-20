from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_front_display_cabinet")

    wood = model.material("warm_oak", rgba=(0.62, 0.38, 0.18, 1.0))
    dark_wood = model.material("dark_end_grain", rgba=(0.32, 0.18, 0.08, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.65, 0.86, 0.95, 0.34))
    brass = model.material("brushed_brass", rgba=(0.86, 0.63, 0.25, 1.0))
    black = model.material("black_key_slot", rgba=(0.02, 0.018, 0.014, 1.0))

    cabinet_w = 1.00
    cabinet_d = 0.36
    cabinet_h = 1.80
    side_t = 0.035
    rail_t = 0.040
    front_y = -cabinet_d / 2.0
    back_y = cabinet_d / 2.0

    carcass = model.part("carcass")
    # Rectangular cabinet body: side walls, top/bottom, back, and a shallow face frame.
    carcass.visual(
        Box((side_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + side_t / 2.0, 0.0, cabinet_h / 2.0)),
        material=wood,
        name="side_stile_0",
    )
    carcass.visual(
        Box((side_t, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(cabinet_w / 2.0 - side_t / 2.0, 0.0, cabinet_h / 2.0)),
        material=wood,
        name="side_stile_1",
    )
    carcass.visual(
        Box((cabinet_w, cabinet_d, rail_t)),
        origin=Origin(xyz=(0.0, 0.0, rail_t / 2.0)),
        material=wood,
        name="bottom_panel",
    )
    carcass.visual(
        Box((cabinet_w, cabinet_d, rail_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h - rail_t / 2.0)),
        material=wood,
        name="top_panel",
    )
    carcass.visual(
        Box((cabinet_w, 0.025, cabinet_h)),
        origin=Origin(xyz=(0.0, back_y - 0.0125, cabinet_h / 2.0)),
        material=dark_wood,
        name="back_panel",
    )
    carcass.visual(
        Box((cabinet_w, 0.040, 0.085)),
        origin=Origin(xyz=(0.0, front_y + 0.020, cabinet_h - 0.075)),
        material=wood,
        name="front_top_rail",
    )
    carcass.visual(
        Box((cabinet_w, 0.040, 0.085)),
        origin=Origin(xyz=(0.0, front_y + 0.020, 0.075)),
        material=wood,
        name="front_bottom_rail",
    )
    carcass.visual(
        Box((0.055, 0.040, cabinet_h - 0.12)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + 0.0275, front_y + 0.020, cabinet_h / 2.0)),
        material=wood,
        name="front_stile_0",
    )
    carcass.visual(
        Box((0.055, 0.040, cabinet_h - 0.12)),
        origin=Origin(xyz=(cabinet_w / 2.0 - 0.0275, front_y + 0.020, cabinet_h / 2.0)),
        material=wood,
        name="front_stile_1",
    )
    fixed_hinge_y = front_y - 0.0175
    for side_name, x in (("0", -cabinet_w / 2.0 - 0.0005), ("1", cabinet_w / 2.0 + 0.0005)):
        for i, z in enumerate((0.340, 0.900, 1.460)):
            carcass.visual(
                Box((0.004, 0.060, 0.180)),
                origin=Origin(xyz=(x, fixed_hinge_y, z)),
                material=brass,
                name=f"fixed_hinge_leaf_{side_name}_{i}",
            )
    # Glass display shelves are let slightly into the side walls so they read as supported.
    for i, z in enumerate((0.55, 0.95, 1.35)):
        carcass.visual(
            Box((0.94, 0.275, 0.012)),
            origin=Origin(xyz=(0.0, 0.005, z)),
            material=glass,
            name=f"glass_shelf_{i}",
        )

    door_w = 0.470
    door_h = 1.640
    door_depth = 0.035
    door_bottom = 0.080
    hinge_x = cabinet_w / 2.0 - 0.023
    hinge_y = front_y - door_depth / 2.0
    door_frame_mesh = mesh_from_geometry(
        BezelGeometry(
            opening_size=(0.340, 1.455),
            outer_size=(door_w, door_h),
            depth=door_depth,
            opening_shape="rect",
            outer_shape="rect",
            center=True,
        ),
        "narrow_door_frame",
    )

    passive_door = model.part("passive_door")
    passive_door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h / 2.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="passive_frame",
    )
    passive_door.visual(
        Box((0.365, 0.006, 1.490)),
        origin=Origin(xyz=(door_w / 2.0, 0.003, door_h / 2.0)),
        material=glass,
        name="passive_glass",
    )
    passive_door.visual(
        Box((0.018, 0.006, 0.120)),
        origin=Origin(xyz=(door_w - 0.012, 0.0195, 0.885)),
        material=brass,
        name="strike_plate",
    )
    for i, z in enumerate((0.260, 0.820, 1.380)):
        passive_door.visual(
            Cylinder(radius=0.012, length=0.220),
            origin=Origin(xyz=(0.0, -0.002, z)),
            material=brass,
            name=f"hinge_knuckle_{i}",
        )

    active_door = model.part("active_door")
    active_door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(-door_w / 2.0, 0.0, door_h / 2.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="active_frame",
    )
    active_door.visual(
        Box((0.365, 0.006, 1.490)),
        origin=Origin(xyz=(-door_w / 2.0, 0.003, door_h / 2.0)),
        material=glass,
        name="active_glass",
    )
    # A narrow brass escutcheon surrounds the lock at the meeting stile.
    active_door.visual(
        Box((0.055, 0.006, 0.100)),
        origin=Origin(xyz=(-door_w + 0.030, -0.0185, 0.885)),
        material=brass,
        name="lock_escutcheon",
    )
    for i, z in enumerate((0.260, 0.820, 1.380)):
        active_door.visual(
            Cylinder(radius=0.012, length=0.220),
            origin=Origin(xyz=(0.0, -0.002, z)),
            material=brass,
            name=f"hinge_knuckle_{i}",
        )

    lock_cam = model.part("lock_cam")
    lock_cam.visual(
        Cylinder(radius=0.020, length=0.052),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="lock_barrel",
    )
    lock_cam.visual(
        Box((0.090, 0.012, 0.022)),
        origin=Origin(xyz=(-0.045, 0.0245, 0.0)),
        material=brass,
        name="cam_tab",
    )
    lock_cam.visual(
        Box((0.004, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -0.029, 0.0)),
        material=black,
        name="key_slot",
    )

    passive_hinge = model.articulation(
        "carcass_to_passive_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=passive_door,
        origin=Origin(xyz=(-hinge_x, hinge_y, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    active_hinge = model.articulation(
        "carcass_to_active_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=active_door,
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "active_door_to_lock_cam",
        ArticulationType.REVOLUTE,
        parent=active_door,
        child=lock_cam,
        origin=Origin(xyz=(-door_w + 0.030, 0.0, 0.885)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=pi / 2.0),
    )

    # Keep handles for static analysis in some SDK versions.
    _ = (passive_hinge, active_hinge)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    passive_door = object_model.get_part("passive_door")
    active_door = object_model.get_part("active_door")
    lock_cam = object_model.get_part("lock_cam")
    passive_hinge = object_model.get_articulation("carcass_to_passive_door")
    active_hinge = object_model.get_articulation("carcass_to_active_door")
    cam_pivot = object_model.get_articulation("active_door_to_lock_cam")

    ctx.allow_overlap(
        active_door,
        lock_cam,
        elem_a="active_frame",
        elem_b="lock_barrel",
        reason="The lock barrel is intentionally modeled as a short front-to-back shaft passing through the active door stile.",
    )
    ctx.allow_overlap(
        active_door,
        lock_cam,
        elem_a="lock_escutcheon",
        elem_b="lock_barrel",
        reason="The brass escutcheon is seated around the cam-lock barrel on the front face.",
    )

    ctx.expect_overlap(
        lock_cam,
        active_door,
        axes="xz",
        elem_a="lock_barrel",
        elem_b="active_frame",
        min_overlap=0.025,
        name="lock barrel passes through active door stile",
    )
    ctx.expect_overlap(
        lock_cam,
        active_door,
        axes="xz",
        elem_a="lock_barrel",
        elem_b="lock_escutcheon",
        min_overlap=0.018,
        name="lock barrel is centered in escutcheon",
    )
    ctx.expect_gap(
        carcass,
        passive_door,
        axis="y",
        positive_elem="front_stile_0",
        negative_elem="passive_frame",
        max_gap=0.001,
        max_penetration=0.0005,
        name="passive door closes against its outer stile",
    )
    ctx.expect_gap(
        carcass,
        active_door,
        axis="y",
        positive_elem="front_stile_1",
        negative_elem="active_frame",
        max_gap=0.001,
        max_penetration=0.0005,
        name="active door closes against its outer stile",
    )

    rest_passive_aabb = ctx.part_world_aabb(passive_door)
    rest_active_aabb = ctx.part_world_aabb(active_door)
    with ctx.pose({passive_hinge: 1.0, active_hinge: 1.0}):
        open_passive_aabb = ctx.part_world_aabb(passive_door)
        open_active_aabb = ctx.part_world_aabb(active_door)
    ctx.check(
        "both doors swing outward from the front",
        rest_passive_aabb is not None
        and rest_active_aabb is not None
        and open_passive_aabb is not None
        and open_active_aabb is not None
        and open_passive_aabb[0][1] < rest_passive_aabb[0][1] - 0.10
        and open_active_aabb[0][1] < rest_active_aabb[0][1] - 0.10,
        details=f"passive rest/open={rest_passive_aabb}/{open_passive_aabb}, active rest/open={rest_active_aabb}/{open_active_aabb}",
    )

    rest_cam_aabb = ctx.part_element_world_aabb(lock_cam, elem="cam_tab")
    with ctx.pose({cam_pivot: pi / 2.0}):
        turned_cam_aabb = ctx.part_element_world_aabb(lock_cam, elem="cam_tab")
    ctx.check(
        "cam lock quarter-turn rotates the rear tab",
        rest_cam_aabb is not None
        and turned_cam_aabb is not None
        and (turned_cam_aabb[1][2] - turned_cam_aabb[0][2]) > (rest_cam_aabb[1][2] - rest_cam_aabb[0][2]) + 0.04,
        details=f"rest={rest_cam_aabb}, turned={turned_cam_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
