from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cellar_hatch_door")

    frame_steel = model.material("frame_steel", rgba=(0.33, 0.35, 0.36, 1.0))
    lid_paint = model.material("lid_paint", rgba=(0.22, 0.28, 0.24, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.60, 0.62, 0.64, 1.0))
    insulation_liner = model.material("insulation_liner", rgba=(0.79, 0.80, 0.76, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.10, 1.0))
    handle_cap = model.material("handle_cap", rgba=(0.72, 0.73, 0.74, 1.0))

    opening_x = 0.90
    opening_y = 0.62
    outer_x = 1.04
    outer_y = 0.76
    curb_outer_x = 0.96
    curb_outer_y = 0.68
    flange_thickness = 0.03
    curb_depth = 0.10

    lid_length = 0.98
    lid_width = 0.72
    lid_thickness = 0.10
    hinge_axis_x = -0.502
    hinge_axis_z = flange_thickness + lid_thickness * 0.5
    hinge_radius = 0.016
    hinge_barrel_length = 0.032
    lid_knuckle_length = 0.026
    hinge_y_positions = (-0.21, 0.21)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((outer_x, outer_y, flange_thickness + curb_depth)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )
    frame_flange = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(outer_x, outer_y),
            [_rect_profile(opening_x, opening_y)],
            height=flange_thickness,
            center=True,
        ),
        "frame_flange",
    )
    frame.visual(
        frame_flange,
        origin=Origin(xyz=(0.0, 0.0, flange_thickness * 0.5)),
        material=frame_steel,
        name="frame_flange",
    )
    frame_curb = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(curb_outer_x, curb_outer_y),
            [_rect_profile(opening_x, opening_y)],
            height=curb_depth,
            center=True,
        ),
        "frame_curb",
    )
    frame.visual(
        frame_curb,
        origin=Origin(xyz=(0.0, 0.0, -curb_depth * 0.5)),
        material=frame_steel,
        name="frame_curb",
    )
    frame.visual(
        Box((0.032, outer_y, 0.05)),
        origin=Origin(xyz=(-0.536, 0.0, 0.055)),
        material=frame_steel,
        name="hinge_mount_strip",
    )
    for index, hinge_y in enumerate(hinge_y_positions, start=1):
        frame.visual(
            Box((0.024, 0.11, 0.070)),
            origin=Origin(xyz=(-0.540, hinge_y, 0.065)),
            material=frame_steel,
            name=f"frame_hinge_plate_{index}",
        )
        frame.visual(
            Box((0.044, 0.034, 0.040)),
            origin=Origin(xyz=(-0.518, hinge_y, hinge_axis_z)),
            material=frame_steel,
            name=f"frame_hinge_bridge_{index}",
        )
        frame.visual(
            Cylinder(radius=hinge_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(hinge_axis_x, hinge_y, hinge_axis_z),
                rpy=(-pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"frame_hinge_barrel_{index}",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, lid_thickness)),
        mass=48.0,
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
    )
    lid.visual(
        Box((lid_length, lid_width, lid_thickness)),
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
        material=lid_paint,
        name="lid_panel_shell",
    )
    lid.visual(
        Box((0.86, 0.56, 0.010)),
        origin=Origin(xyz=(0.52, 0.0, -0.045)),
        material=insulation_liner,
        name="lid_inner_liner",
    )
    lid.visual(
        Box((0.032, lid_width, 0.10)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=lid_paint,
        name="lid_edge_rail",
    )
    for index, hinge_y in enumerate(hinge_y_positions, start=1):
        lid.visual(
            Box((0.042, 0.11, 0.058)),
            origin=Origin(xyz=(0.040, hinge_y, 0.0)),
            material=hinge_metal,
            name=f"lid_hinge_strap_{index}",
        )
        lid.visual(
            Box((0.028, 0.028, 0.052)),
            origin=Origin(xyz=(0.014, hinge_y - 0.029, 0.0)),
            material=hinge_metal,
            name=f"lid_hinge_bridge_{index}_a",
        )
        lid.visual(
            Box((0.028, 0.028, 0.052)),
            origin=Origin(xyz=(0.014, hinge_y + 0.029, 0.0)),
            material=hinge_metal,
            name=f"lid_hinge_bridge_{index}_b",
        )
        lid.visual(
            Cylinder(radius=hinge_radius, length=lid_knuckle_length),
            origin=Origin(
                xyz=(0.0, hinge_y - 0.029, 0.0),
                rpy=(-pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"lid_hinge_knuckle_{index}_a",
        )
        lid.visual(
            Cylinder(radius=hinge_radius, length=lid_knuckle_length),
            origin=Origin(
                xyz=(0.0, hinge_y + 0.029, 0.0),
                rpy=(-pi * 0.5, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"lid_hinge_knuckle_{index}_b",
        )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.20, 0.08, 0.08)),
        mass=1.8,
        origin=Origin(xyz=(-0.07, 0.0, 0.04)),
    )
    handle.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=handle_black,
        name="handle_base",
    )
    handle.visual(
        Cylinder(radius=0.015, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=handle_cap,
        name="handle_spindle",
    )
    handle.visual(
        Box((0.16, 0.032, 0.024)),
        origin=Origin(xyz=(-0.08, 0.0, 0.052)),
        material=handle_black,
        name="handle_grip",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(-0.156, 0.0, 0.052), rpy=(0.0, pi * 0.5, 0.0)),
        material=handle_cap,
        name="handle_end_knob",
    )

    model.articulation(
        "frame_to_lid",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.83, 0.0, lid_thickness * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("frame_to_lid")
    handle_joint = object_model.get_articulation("lid_to_handle")

    with ctx.pose({lid_hinge: 0.0, handle_joint: 0.0}):
        ctx.expect_gap(
            lid,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="lid_panel_shell",
            negative_elem="frame_flange",
            name="closed lid seats on the opening frame",
        )
        ctx.expect_overlap(
            lid,
            frame,
            axes="xy",
            min_overlap=0.60,
            elem_a="lid_panel_shell",
            elem_b="frame_flange",
            name="closed lid covers the frame opening",
        )
        ctx.expect_contact(
            handle,
            lid,
            elem_a="handle_base",
            elem_b="lid_panel_shell",
            name="locking handle base is mounted to the lid surface",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel_shell")
    with ctx.pose({lid_hinge: 1.25, handle_joint: 0.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel_shell")
    lid_opens_upward = (
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.35
    )
    ctx.check(
        "lid rotates upward from the side hinge",
        lid_opens_upward,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({lid_hinge: 0.0, handle_joint: 0.0}):
        rest_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({lid_hinge: 0.0, handle_joint: 1.0}):
        turned_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    handle_rotates = False
    if rest_grip is not None and turned_grip is not None:
        rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(rest_grip[0], rest_grip[1]))
        turned_center = tuple((lo + hi) * 0.5 for lo, hi in zip(turned_grip[0], turned_grip[1]))
        handle_rotates = abs(turned_center[1] - rest_center[1]) > 0.05 and abs(turned_center[2] - rest_center[2]) < 0.02
    ctx.check(
        "locking handle rotates about its own pivot near the free edge",
        handle_rotates,
        details=f"rest={rest_grip}, turned={turned_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
