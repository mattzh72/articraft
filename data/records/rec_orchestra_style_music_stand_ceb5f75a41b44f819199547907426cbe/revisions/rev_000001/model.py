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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    wire_from_points,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """CadQuery hollow round tube, authored in meters with its lower face on z=0."""
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _horizontal_cylinder_origin(
    x: float,
    y: float,
    z: float,
    *,
    yaw: float = 0.0,
) -> Origin:
    """Orient a URDF/CQ cylinder's local +Z axis into the horizontal XY plane."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conductor_music_stand")

    black_metal = model.material("matte_black_powder_coat", rgba=(0.005, 0.006, 0.007, 1.0))
    edge_black = model.material("slightly_worn_black_edges", rgba=(0.025, 0.024, 0.022, 1.0))
    rubber = model.material("black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    hinge_dark = model.material("dark_burnished_pivot_hardware", rgba=(0.035, 0.033, 0.030, 1.0))

    # Root: weighted floor base, three low stabilizing legs, and a hollow pedestal sleeve.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.105, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=edge_black,
        name="weighted_base",
    )
    pedestal.visual(
        Cylinder(radius=0.052, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=black_metal,
        name="center_hub",
    )
    for index, angle in enumerate((-math.pi / 2.0, math.pi / 6.0, 5.0 * math.pi / 6.0)):
        dx = math.cos(angle)
        dy = math.sin(angle)
        pedestal.visual(
            Cylinder(radius=0.010, length=0.52),
            origin=_horizontal_cylinder_origin(0.235 * dx, 0.235 * dy, 0.042, yaw=angle),
            material=black_metal,
            name=f"floor_leg_{index}",
        )
        pedestal.visual(
            Box((0.085, 0.036, 0.018)),
            origin=Origin(xyz=(0.485 * dx, 0.485 * dy, 0.023), rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )
    pedestal.visual(
        _tube_mesh(0.024, 0.017, 0.58, "pedestal_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black_metal,
        name="pedestal_sleeve",
    )
    pedestal.visual(
        _tube_mesh(0.031, 0.0175, 0.030, "lower_sleeve_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=edge_black,
        name="lower_sleeve_collar",
    )
    pedestal.visual(
        _tube_mesh(0.034, 0.0175, 0.035, "upper_sleeve_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.602)),
        material=edge_black,
        name="upper_sleeve_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.006, length=0.041),
        origin=_horizontal_cylinder_origin(0.0345, 0.0, 0.610),
        material=hinge_dark,
        name="height_screw",
    )
    pedestal.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=_horizontal_cylinder_origin(0.065, 0.0, 0.610),
        material=edge_black,
        name="height_knob",
    )

    # Sliding mast: long retained inner tube plus the compact yoke at the desk hinge.
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.014, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=black_metal,
        name="inner_tube",
    )
    mast.visual(
        Box((0.003, 0.018, 0.024)),
        origin=Origin(xyz=(0.0155, 0.0, -0.010)),
        material=hinge_dark,
        name="clamp_flat",
    )
    mast.visual(
        Cylinder(radius=0.024, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=edge_black,
        name="head_socket",
    )
    mast.visual(
        Box((0.140, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.015, 0.715)),
        material=edge_black,
        name="head_crossbar",
    )
    mast.visual(
        Box((0.016, 0.046, 0.080)),
        origin=Origin(xyz=(-0.058, 0.015, 0.760)),
        material=hinge_dark,
        name="head_yoke_0",
    )
    mast.visual(
        Box((0.016, 0.046, 0.080)),
        origin=Origin(xyz=(0.058, 0.015, 0.760)),
        material=hinge_dark,
        name="head_yoke_1",
    )

    # Broad perforated desk surface with rolled top, lower tray, side edges, hinge lug,
    # and pivot bosses for the two page retainers.
    desk = model.part("desk")
    desk.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.660, 0.380),
                0.006,
                hole_diameter=0.017,
                pitch=(0.044, 0.040),
                frame=0.034,
                corner_radius=0.026,
                stagger=True,
            ),
            "desk_perforated_panel",
        ),
        origin=Origin(xyz=(0.0, -0.006, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="panel",
    )
    desk.visual(
        Box((0.720, 0.055, 0.028)),
        origin=Origin(xyz=(0.0, -0.035, 0.045)),
        material=edge_black,
        name="lower_tray",
    )
    desk.visual(
        Box((0.720, 0.015, 0.060)),
        origin=Origin(xyz=(0.0, -0.066, 0.072)),
        material=edge_black,
        name="lower_lip",
    )
    for x, name in ((-0.335, "side_edge_0"), (0.335, "side_edge_1")):
        desk.visual(
            Box((0.030, 0.040, 0.360)),
            origin=Origin(xyz=(x, -0.018, 0.245)),
            material=edge_black,
            name=name,
        )
    desk.visual(
        Cylinder(radius=0.010, length=0.665),
        origin=_horizontal_cylinder_origin(0.0, -0.009, 0.438),
        material=edge_black,
        name="rolled_top",
    )
    desk.visual(
        Box((0.105, 0.018, 0.220)),
        origin=Origin(xyz=(0.0, 0.002, 0.155)),
        material=edge_black,
        name="rear_spine",
    )
    desk.visual(
        Box((0.080, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.010, 0.020)),
        material=hinge_dark,
        name="hinge_lug",
    )
    desk.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=_horizontal_cylinder_origin(0.0, 0.010, 0.0),
        material=hinge_dark,
        name="hinge_pin",
    )
    desk.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(-0.345, -0.033, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="side_pivot_0",
    )
    desk.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.345, -0.033, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="side_pivot_1",
    )

    def make_page_support(part_name: str, inward: float) -> object:
        support = model.part(part_name)
        support.visual(
            Cylinder(radius=0.010, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_dark,
            name="pivot_pin",
        )
        support.visual(
            mesh_from_geometry(
                wire_from_points(
                    [
                        (0.000, -0.012, 0.000),
                        (0.035 * inward, -0.014, 0.038),
                        (0.070 * inward, -0.014, 0.125),
                        (0.108 * inward, -0.014, 0.178),
                    ],
                    radius=0.004,
                    radial_segments=16,
                    cap_ends=True,
                    corner_mode="fillet",
                    corner_radius=0.018,
                ),
                f"{part_name}_formed_wire",
            ),
            material=black_metal,
            name="formed_wire",
        )
        support.visual(
            Box((0.066, 0.010, 0.024)),
            origin=Origin(xyz=(0.087 * inward, -0.018, 0.178)),
            material=edge_black,
            name="page_clip",
        )
        return support

    page_support_0 = make_page_support("page_support_0", inward=1.0)
    page_support_1 = make_page_support("page_support_1", inward=-1.0)

    model.articulation(
        "pedestal_to_mast",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.280, effort=95.0, velocity=0.22),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.65, effort=16.0, velocity=1.4),
    )
    model.articulation(
        "desk_to_page_support_0",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=page_support_0,
        origin=Origin(xyz=(-0.345, -0.057, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=1.0, effort=1.5, velocity=3.0),
    )
    model.articulation(
        "desk_to_page_support_1",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=page_support_1,
        origin=Origin(xyz=(0.345, -0.057, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=1.0, effort=1.5, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    support_0 = object_model.get_part("page_support_0")
    support_1 = object_model.get_part("page_support_1")

    mast_slide = object_model.get_articulation("pedestal_to_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    support_0_pivot = object_model.get_articulation("desk_to_page_support_0")
    support_1_pivot = object_model.get_articulation("desk_to_page_support_1")

    ctx.expect_within(
        mast,
        pedestal,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="pedestal_sleeve",
        margin=0.004,
        name="inner mast remains centered in the sleeve bore",
    )
    ctx.expect_overlap(
        mast,
        pedestal,
        axes="z",
        elem_a="inner_tube",
        elem_b="pedestal_sleeve",
        min_overlap=0.34,
        name="lower mast has retained insertion at minimum height",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.280}):
        ctx.expect_within(
            mast,
            pedestal,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="pedestal_sleeve",
            margin=0.004,
            name="extended mast remains centered in the sleeve bore",
        )
        ctx.expect_overlap(
            mast,
            pedestal,
            axes="z",
            elem_a="inner_tube",
            elem_b="pedestal_sleeve",
            min_overlap=0.09,
            name="extended mast still remains inserted in the sleeve",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward from the pedestal sleeve",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.25,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    ctx.expect_contact(
        desk,
        mast,
        elem_a="hinge_pin",
        elem_b="head_yoke_0",
        contact_tol=0.004,
        name="desk hinge pin reaches one mast yoke cheek",
    )
    ctx.expect_contact(
        desk,
        mast,
        elem_a="hinge_pin",
        elem_b="head_yoke_1",
        contact_tol=0.004,
        name="desk hinge pin reaches the other mast yoke cheek",
    )

    panel_aabb = ctx.part_element_world_aabb(desk, elem="panel")
    if panel_aabb is not None:
        panel_dx = panel_aabb[1][0] - panel_aabb[0][0]
        panel_dz = panel_aabb[1][2] - panel_aabb[0][2]
    else:
        panel_dx = panel_dz = 0.0
    ctx.check(
        "desk is a broad orchestra-style page surface",
        panel_dx > 0.60 and panel_dz > 0.34,
        details=f"panel_dx={panel_dx:.3f}, panel_dz={panel_dz:.3f}",
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_panel_aabb = ctx.part_element_world_aabb(desk, elem="panel")
    with ctx.pose({desk_tilt: 0.50}):
        tilted_panel_aabb = ctx.part_element_world_aabb(desk, elem="panel")
    rest_panel_center = aabb_center(rest_panel_aabb) if rest_panel_aabb is not None else None
    tilted_panel_center = aabb_center(tilted_panel_aabb) if tilted_panel_aabb is not None else None
    ctx.check(
        "desk tilts around the horizontal mast-head hinge",
        rest_panel_center is not None
        and tilted_panel_center is not None
        and tilted_panel_center[1] < rest_panel_center[1] - 0.06,
        details=f"rest={rest_panel_center}, tilted={tilted_panel_center}",
    )

    ctx.expect_contact(
        support_0,
        desk,
        elem_a="pivot_pin",
        elem_b="side_pivot_0",
        contact_tol=0.004,
        name="one page support is mounted on its side pivot",
    )
    ctx.expect_contact(
        support_1,
        desk,
        elem_a="pivot_pin",
        elem_b="side_pivot_1",
        contact_tol=0.004,
        name="other page support is mounted on its side pivot",
    )

    rest_0 = ctx.part_world_aabb(support_0)
    with ctx.pose({support_0_pivot: 0.65}):
        folded_0 = ctx.part_world_aabb(support_0)
    rest_1 = ctx.part_world_aabb(support_1)
    with ctx.pose({support_1_pivot: -0.65}):
        folded_1 = ctx.part_world_aabb(support_1)
    rest_0_center = aabb_center(rest_0) if rest_0 is not None else None
    folded_0_center = aabb_center(folded_0) if folded_0 is not None else None
    rest_1_center = aabb_center(rest_1) if rest_1 is not None else None
    folded_1_center = aabb_center(folded_1) if folded_1 is not None else None
    ctx.check(
        "side page supports rotate on small side pivots",
        rest_0_center is not None
        and folded_0_center is not None
        and rest_1_center is not None
        and folded_1_center is not None
        and folded_0_center[0] > rest_0_center[0] + 0.025
        and folded_1_center[0] < rest_1_center[0] - 0.025,
        details=(
            f"support_0 rest={rest_0_center}, folded={folded_0_center}; "
            f"support_1 rest={rest_1_center}, folded={folded_1_center}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
