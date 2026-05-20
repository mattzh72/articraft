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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_panel_mesh(name: str, *, width: float, height: float, thickness: float, radius: float):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=12),
        thickness,
        center=True,
        closed=True,
    )
    geom.rotate_x(-pi / 2.0)
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_refrigerator")

    enamel = model.material("enamel", rgba=(0.86, 0.79, 0.67, 1.0))
    liner = model.material("liner", rgba=(0.95, 0.96, 0.97, 1.0))
    gasket = model.material("gasket", rgba=(0.21, 0.22, 0.24, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.40, 0.35, 0.31, 1.0))

    cabinet_width = 0.76
    cabinet_depth = 0.72
    cabinet_height = 1.68
    shell_thickness = 0.045
    back_panel_thickness = 0.020
    corner_radius = 0.090

    door_width = cabinet_width - 0.040
    door_height = cabinet_height - 0.050
    door_thickness = 0.055
    door_radius = 0.075
    door_gap = 0.006
    door_bottom = 0.025

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=86.0,
        origin=Origin(xyz=(0.0, cabinet_depth * 0.5, cabinet_height * 0.5)),
    )

    outer_profile = rounded_rect_profile(cabinet_width, cabinet_height, corner_radius, corner_segments=14)
    inner_profile = rounded_rect_profile(
        cabinet_width - 2.0 * shell_thickness,
        cabinet_height - 2.0 * shell_thickness,
        max(corner_radius - shell_thickness, 0.028),
        corner_segments=14,
    )
    shell_depth = cabinet_depth - back_panel_thickness
    cabinet_shell_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        shell_depth,
        center=True,
        closed=True,
    )
    cabinet_shell_geom.rotate_x(-pi / 2.0)
    cabinet.visual(
        _mesh("cabinet_shell", cabinet_shell_geom),
        origin=Origin(xyz=(0.0, shell_depth * 0.5, cabinet_height * 0.5)),
        material=enamel,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box(
            (
                cabinet_width - shell_thickness * 0.70,
                back_panel_thickness,
                cabinet_height - shell_thickness * 0.70,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth - back_panel_thickness * 0.5,
                cabinet_height * 0.5,
            )
        ),
        material=liner,
        name="back_panel",
    )

    gasket_depth = 0.008
    gasket_width = 0.020
    cabinet.visual(
        Box((gasket_width, gasket_depth, door_height - 0.050)),
        origin=Origin(xyz=(-door_width * 0.5 + gasket_width * 0.5, gasket_depth * 0.5, door_bottom + door_height * 0.5)),
        material=gasket,
        name="left_gasket",
    )
    cabinet.visual(
        Box((gasket_width, gasket_depth, door_height - 0.050)),
        origin=Origin(xyz=(door_width * 0.5 - gasket_width * 0.5, gasket_depth * 0.5, door_bottom + door_height * 0.5)),
        material=gasket,
        name="right_gasket",
    )
    cabinet.visual(
        Box((door_width - 2.0 * gasket_width, gasket_depth, gasket_width)),
        origin=Origin(xyz=(0.0, gasket_depth * 0.5, door_bottom + door_height - gasket_width * 0.5)),
        material=gasket,
        name="top_gasket",
    )
    cabinet.visual(
        Box((door_width - 2.0 * gasket_width, gasket_depth, gasket_width)),
        origin=Origin(xyz=(0.0, gasket_depth * 0.5, door_bottom + gasket_width * 0.5)),
        material=gasket,
        name="bottom_gasket",
    )

    hinge_radius = 0.012
    hinge_length = 0.070
    hinge_y = -0.010
    hinge_x = -door_width * 0.5 - hinge_radius * 0.8
    cabinet.visual(
        Cylinder(radius=hinge_radius, length=hinge_length),
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom + 0.100)),
        material=chrome,
        name="upper_hinge_barrel",
    )
    cabinet.visual(
        Cylinder(radius=hinge_radius, length=hinge_length),
        origin=Origin(xyz=(hinge_x, hinge_y, door_bottom + door_height - 0.100)),
        material=chrome,
        name="lower_hinge_barrel",
    )
    cabinet.visual(
        Box((cabinet_width * 0.55, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, cabinet_depth - 0.020, 0.018)),
        material=dark_trim,
        name="compressor_cover",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=24.0,
        origin=Origin(xyz=(door_width * 0.5, -(door_gap + door_thickness * 0.5), door_height * 0.5)),
    )
    door.visual(
        _rounded_panel_mesh(
            "door_shell",
            width=door_width,
            height=door_height,
            thickness=door_thickness,
            radius=door_radius,
        ),
        origin=Origin(xyz=(door_width * 0.5, -(door_gap + door_thickness * 0.5), door_height * 0.5)),
        material=enamel,
        name="door_shell",
    )
    door.visual(
        Box((door_width - 0.080, 0.010, door_height - 0.120)),
        origin=Origin(xyz=(door_width * 0.5, -(door_gap + 0.005), door_height * 0.5)),
        material=liner,
        name="door_liner",
    )
    door.visual(
        Box((door_width - 0.050, 0.010, 0.035)),
        origin=Origin(xyz=(door_width * 0.5, -(door_gap + 0.005), 0.035)),
        material=dark_trim,
        name="door_shelf_lip",
    )
    door.visual(
        Box((0.020, 0.010, door_height - 0.090)),
        origin=Origin(xyz=(door_width - 0.025, -(door_gap + 0.005), door_height * 0.5)),
        material=gasket,
        name="door_edge_gasket",
    )

    handle_latch = model.part("handle_latch")
    handle_latch.inertial = Inertial.from_geometry(
        Box((0.090, 0.075, 0.480)),
        mass=1.4,
        origin=Origin(xyz=(-0.028, -0.028, 0.0)),
    )
    handle_latch.visual(
        Box((0.026, 0.008, 0.110)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=chrome,
        name="mount_plate",
    )
    handle_latch.visual(
        Cylinder(radius=0.014, length=0.095),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=chrome,
        name="pivot_hub",
    )
    handle_latch.visual(
        Box((0.040, 0.020, 0.085)),
        origin=Origin(xyz=(-0.015, -0.014, 0.0)),
        material=chrome,
        name="latch_arm",
    )
    handle_latch.visual(
        Cylinder(radius=0.013, length=0.450),
        origin=Origin(xyz=(-0.034, -0.032, 0.0)),
        material=chrome,
        name="handle_bar",
    )
    handle_latch.visual(
        Box((0.030, 0.020, 0.060)),
        origin=Origin(xyz=(-0.020, -0.020, -0.155)),
        material=chrome,
        name="lower_latch_block",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-door_width * 0.5, 0.0, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.4, lower=0.0, upper=1.85),
    )
    model.articulation(
        "door_to_handle_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle_latch,
        origin=Origin(xyz=(door_width - 0.060, -(door_gap + door_thickness), door_height * 0.57)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.55),
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

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    handle_latch = object_model.get_part("handle_latch")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    latch_pivot = object_model.get_articulation("door_to_handle_latch")

    cabinet_shell = cabinet.get_visual("cabinet_shell")
    door_shell = door.get_visual("door_shell")
    handle_bar = handle_latch.get_visual("handle_bar")
    mount_plate = handle_latch.get_visual("mount_plate")

    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            min_gap=0.004,
            max_gap=0.012,
            positive_elem=cabinet_shell,
            negative_elem=door_shell,
            name="closed door sits just in front of cabinet shell",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.65,
            elem_a=door_shell,
            elem_b=cabinet_shell,
            name="door covers the main cabinet opening",
        )
        ctx.expect_gap(
            door,
            handle_latch,
            axis="y",
            min_gap=0.008,
            max_gap=0.025,
            positive_elem=door_shell,
            negative_elem=handle_bar,
            name="handle bar stands proud of the door face",
        )
        ctx.expect_contact(
            handle_latch,
            door,
            elem_a=mount_plate,
            elem_b=door_shell,
            contact_tol=0.0005,
            name="latch mount plate meets the door face",
        )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem=door_shell)
    with ctx.pose({door_hinge: 1.35, latch_pivot: 0.0}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem=door_shell)
    ctx.check(
        "door opens outward on a vertical side hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.22,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle_latch, elem=handle_bar)
    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.42}):
        open_handle_aabb = ctx.part_element_world_aabb(handle_latch, elem=handle_bar)
    ctx.check(
        "handle latch rotates away from the door",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.008,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
