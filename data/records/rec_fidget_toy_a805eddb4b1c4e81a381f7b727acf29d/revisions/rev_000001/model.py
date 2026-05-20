from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
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
    segments: int = 24,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _build_button_mesh(name: str):
    dome_radius = 0.0076
    dome_height_scale = 0.72
    stem_radius = 0.0050
    stem_length = 0.0060
    retainer_radius = 0.0062
    retainer_thickness = 0.0012

    dome = DomeGeometry(
        dome_radius,
        radial_segments=28,
        height_segments=14,
        closed=True,
    ).scale(1.0, 1.0, dome_height_scale)
    stem = CylinderGeometry(stem_radius, stem_length, radial_segments=22).translate(
        0.0,
        0.0,
        (-0.5 * stem_length) + 0.0002,
    )
    retainer = CylinderGeometry(
        retainer_radius,
        retainer_thickness,
        radial_segments=22,
    ).translate(0.0, 0.0, -stem_length + 0.0006)
    dome.merge(stem)
    dome.merge(retainer)
    return mesh_from_geometry(dome, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pop_it_cube_fidget")

    shell_color = model.material("shell_silicone", rgba=(0.55, 0.72, 0.74, 1.0))
    lid_color = model.material("lid_silicone", rgba=(0.58, 0.76, 0.78, 1.0))
    button_color = model.material("button_silicone", rgba=(0.95, 0.63, 0.53, 1.0))

    outer_size = 0.096
    wall_thickness = 0.006
    base_thickness = 0.006
    wall_height = 0.090
    lid_size = 0.082
    lid_thickness = 0.006
    hinge_radius = 0.0035
    hinge_axis_y = -(outer_size * 0.5) + (wall_thickness * 0.5)
    hinge_axis_z = wall_height

    body = model.part("body")
    body.visual(
        Box((outer_size, outer_size, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=shell_color,
        name="base_plate",
    )
    body.visual(
        Box((outer_size, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, (outer_size * 0.5) - (wall_thickness * 0.5), wall_height * 0.5)
        ),
        material=shell_color,
        name="front_wall",
    )
    body.visual(
        Box((outer_size, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, -(outer_size * 0.5) + (wall_thickness * 0.5), wall_height * 0.5)
        ),
        material=shell_color,
        name="rear_wall",
    )
    side_wall_depth = outer_size - (2.0 * wall_thickness)
    body.visual(
        Box((wall_thickness, side_wall_depth, wall_height)),
        origin=Origin(
            xyz=((outer_size * 0.5) - (wall_thickness * 0.5), 0.0, wall_height * 0.5)
        ),
        material=shell_color,
        name="right_wall",
    )
    body.visual(
        Box((wall_thickness, side_wall_depth, wall_height)),
        origin=Origin(
            xyz=(-(outer_size * 0.5) + (wall_thickness * 0.5), 0.0, wall_height * 0.5)
        ),
        material=shell_color,
        name="left_wall",
    )

    body_barrel_length = 0.012
    body_barrel_centers = (-0.028, 0.0, 0.028)
    for index, center_x in enumerate(body_barrel_centers):
        body.visual(
            Cylinder(radius=hinge_radius, length=body_barrel_length),
            origin=Origin(
                xyz=(center_x, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=shell_color,
            name=f"body_hinge_barrel_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((outer_size, outer_size, wall_height)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, wall_height * 0.45)),
    )

    hole_radius = 0.0086
    panel_profile = rounded_rect_profile(lid_size, lid_size, 0.004, corner_segments=8)
    panel_center_y = 0.046
    grid_x = (-0.027, -0.009, 0.009, 0.027)
    grid_y_offsets = (-0.021, 0.0, 0.021)
    hole_profiles = [
        _circle_profile(hole_radius, center=(x, y), segments=28)
        for y in grid_y_offsets
        for x in grid_x
    ]
    lid_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            panel_profile,
            hole_profiles,
            lid_thickness,
            center=True,
            cap=True,
            closed=True,
        ),
        "pop_it_cube_lid_panel",
    )

    lid = model.part("lid")
    lid.visual(
        lid_panel_mesh,
        origin=Origin(xyz=(0.0, panel_center_y, lid_thickness * 0.5)),
        material=lid_color,
        name="lid_panel",
    )

    lid_tab_size = (0.017, 0.010, lid_thickness)
    for index, tab_x in enumerate((-0.0145, 0.0145)):
        lid.visual(
            Box(lid_tab_size),
            origin=Origin(xyz=(tab_x, 0.005, lid_thickness * 0.5)),
            material=lid_color,
            name=f"lid_hinge_tab_{index}",
        )

    lid_barrel_length = 0.015
    lid_barrel_centers = (-0.0145, 0.0145)
    for index, center_x in enumerate(lid_barrel_centers):
        lid.visual(
            Cylinder(radius=hinge_radius, length=lid_barrel_length),
            origin=Origin(
                xyz=(center_x, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=lid_color,
            name=f"lid_hinge_barrel_{index}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((lid_size, lid_size, 0.015)),
        mass=0.06,
        origin=Origin(xyz=(0.0, panel_center_y, 0.0075)),
    )

    lid_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )

    button_mesh = _build_button_mesh("pop_it_button")
    button_mass_geometry = Cylinder(radius=0.0076, length=0.013)
    button_names: list[str] = []
    button_joint_names: list[str] = []
    for row_index, button_y_offset in enumerate(grid_y_offsets):
        for col_index, button_x in enumerate(grid_x):
            button_name = f"button_{row_index}_{col_index}"
            joint_name = f"lid_to_{button_name}"
            button = model.part(button_name)
            button.visual(
                button_mesh,
                material=button_color,
                name="dome_button",
            )
            button.visual(
                Cylinder(radius=hole_radius, length=0.0032),
                origin=Origin(xyz=(0.0, 0.0, -0.0014)),
                material=button_color,
                name="button_guide",
            )
            button.inertial = Inertial.from_geometry(
                button_mass_geometry,
                mass=0.004,
                origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            )
            model.articulation(
                joint_name,
                ArticulationType.PRISMATIC,
                parent=lid,
                child=button,
                origin=Origin(xyz=(button_x, panel_center_y + button_y_offset, lid_thickness)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=0.4,
                    velocity=0.05,
                    lower=0.0,
                    upper=0.003,
                ),
            )
            button_names.append(button_name)
            button_joint_names.append(joint_name)

    model.meta["button_names"] = tuple(button_names)
    model.meta["button_joint_names"] = tuple(button_joint_names)
    model.meta["lid_hinge_name"] = lid_hinge.name
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
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    button_names = tuple(object_model.meta.get("button_names", ()))
    button_joint_names = tuple(object_model.meta.get("button_joint_names", ()))

    ctx.check(
        "twelve pop buttons are authored",
        len(button_names) == 12 and len(button_joint_names) == 12,
        details=f"buttons={len(button_names)}, joints={len(button_joint_names)}",
    )
    ctx.check(
        "lid hinge rotates around the rear edge",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper >= 1.5,
        details=f"axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_within(
            lid,
            body,
            axes="xy",
            inner_elem="lid_panel",
            outer_elem="base_plate",
            margin=0.0,
            name="closed lid stays inside the cube footprint",
        )
        for button_name in button_names:
            ctx.expect_within(
                object_model.get_part(button_name),
                lid,
                axes="xy",
                inner_elem="dome_button",
                outer_elem="lid_panel",
                margin=0.0,
                name=f"{button_name} sits within the lid face grid",
            )
        for joint_name in button_joint_names:
            button_joint = object_model.get_articulation(joint_name)
            ctx.check(
                f"{joint_name} compresses normal to the face",
                button_joint.articulation_type == ArticulationType.PRISMATIC
                and button_joint.axis == (0.0, 0.0, -1.0)
                and button_joint.motion_limits is not None
                and button_joint.motion_limits.upper == 0.003,
                details=f"type={button_joint.articulation_type}, axis={button_joint.axis}, limits={button_joint.motion_limits}",
            )

    lid_closed_aabb = None
    lid_open_aabb = None
    with ctx.pose({lid_hinge: 0.0}):
        lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.2}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid folds upward when opened",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.03,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        for joint_name, button_name in zip(button_joint_names, button_names):
            button_joint = object_model.get_articulation(joint_name)
            button = object_model.get_part(button_name)
            press_travel = 0.003
            if button_joint.motion_limits is not None and button_joint.motion_limits.upper is not None:
                press_travel = button_joint.motion_limits.upper
            rest_pos = ctx.part_world_position(button)
            pressed_pos = None
            with ctx.pose({button_joint: press_travel}):
                pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"{button_name} presses downward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[2] < rest_pos[2] - 0.002,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
