from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _octagon_side_length(apothem: float) -> float:
    return 2.0 * apothem * math.tan(math.pi / 8.0)


def _octagon_face_pose(apothem: float, index: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    face_angle = (math.pi / 2.0) - index * (math.pi / 4.0)
    x = apothem * math.cos(face_angle)
    y = apothem * math.sin(face_angle)
    yaw = face_angle - (math.pi / 2.0)
    return (x, y, 0.0), (0.0, 0.0, yaw)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_lighthouse_lantern")

    cast_iron = model.material("cast_iron", rgba=(0.16, 0.18, 0.20, 1.0))
    iron_trim = model.material("iron_trim", rgba=(0.24, 0.25, 0.27, 1.0))
    roof_iron = model.material("roof_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.76, 0.90, 0.98, 0.30))
    fresnel_glass = model.material("fresnel_glass", rgba=(0.84, 0.96, 1.0, 0.42))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.28, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.80, 0.67, 0.34, 1.0))

    lantern_body = model.part("lantern_body")

    plinth_radius = 1.38
    plinth_height = 0.12
    curb_apothem = 1.25
    curb_wall_depth = 0.10
    curb_height = 0.66
    curb_bottom_z = plinth_height
    curb_center_z = curb_bottom_z + curb_height * 0.5
    curb_top_z = curb_bottom_z + curb_height
    curb_face_width = _octagon_side_length(curb_apothem)

    panel_opening_width = 0.46
    panel_opening_height = 0.42
    panel_sill_height = 0.10
    panel_top_height = curb_height - panel_sill_height - panel_opening_height
    panel_leaf_width = 0.44
    panel_leaf_height = 0.40
    panel_leaf_thickness = 0.03
    panel_hinge_x = -panel_leaf_width * 0.5
    panel_hinge_y = curb_apothem + (curb_wall_depth * 0.5) + 0.003
    panel_hinge_z = curb_bottom_z + panel_sill_height + panel_leaf_height * 0.5

    glass_apothem = 1.08
    glass_face_width = _octagon_side_length(glass_apothem)
    lower_ring_z = curb_top_z + 0.05
    lower_ring_height = 0.10
    pane_height = 1.40
    pane_bottom_z = lower_ring_z + lower_ring_height * 0.5 - 0.01
    pane_center_z = pane_bottom_z + pane_height * 0.5
    top_ring_height = 0.12
    top_ring_center_z = pane_bottom_z + pane_height + top_ring_height * 0.5 - 0.02

    roof_base_z = top_ring_center_z + top_ring_height * 0.5 - 0.02
    roof_height = 0.70
    vent_height = 0.18
    vent_radius = 0.18

    deck_radius = 1.23
    deck_height = 0.08
    deck_center_z = curb_top_z + deck_height * 0.5 - 0.01

    pedestal_height = 0.68
    pedestal_radius = 0.24
    pedestal_center_z = deck_center_z + deck_height * 0.5 + pedestal_height * 0.5
    optic_joint_z = deck_center_z + deck_height * 0.5 + pedestal_height

    lantern_body.visual(
        Cylinder(radius=plinth_radius, length=plinth_height),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=cast_iron,
        name="base_plinth",
    )

    left_width = (curb_face_width - panel_opening_width) * 0.5
    left_center_x = -(panel_opening_width * 0.5 + left_width * 0.5)
    right_center_x = panel_opening_width * 0.5 + left_width * 0.5
    front_outer_y = curb_apothem
    front_y_center = front_outer_y

    lantern_body.visual(
        Box((left_width, curb_wall_depth, curb_height)),
        origin=Origin(xyz=(left_center_x, front_y_center, curb_center_z)),
        material=cast_iron,
        name="curb_front_left",
    )
    lantern_body.visual(
        Box((left_width, curb_wall_depth, curb_height)),
        origin=Origin(xyz=(right_center_x, front_y_center, curb_center_z)),
        material=cast_iron,
        name="curb_front_right",
    )
    lantern_body.visual(
        Box((panel_opening_width, curb_wall_depth, panel_sill_height)),
        origin=Origin(
            xyz=(0.0, front_y_center, curb_bottom_z + panel_sill_height * 0.5)
        ),
        material=cast_iron,
        name="curb_front_sill",
    )
    lantern_body.visual(
        Box((panel_opening_width, curb_wall_depth, panel_top_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y_center,
                curb_bottom_z + panel_sill_height + panel_opening_height + panel_top_height * 0.5,
            )
        ),
        material=cast_iron,
        name="curb_front_lintel",
    )

    for face_index in range(1, 8):
        (face_x, face_y, _), face_rpy = _octagon_face_pose(curb_apothem, face_index)
        lantern_body.visual(
            Box((curb_face_width, curb_wall_depth, curb_height)),
            origin=Origin(xyz=(face_x, face_y, curb_center_z), rpy=face_rpy),
            material=cast_iron,
            name=f"curb_wall_{face_index}",
        )

    lantern_body.visual(
        Cylinder(radius=deck_radius, length=deck_height),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z)),
        material=iron_trim,
        name="service_deck",
    )

    for band_name, band_apothem, band_depth, band_height, band_center_z, band_material in (
        ("lower_ring", glass_apothem, 0.12, lower_ring_height, lower_ring_z, iron_trim),
        ("upper_ring", glass_apothem, 0.14, top_ring_height, top_ring_center_z, iron_trim),
    ):
        band_width = _octagon_side_length(band_apothem)
        for face_index in range(8):
            (face_x, face_y, _), face_rpy = _octagon_face_pose(band_apothem, face_index)
            lantern_body.visual(
                Box((band_width, band_depth, band_height)),
                origin=Origin(xyz=(face_x, face_y, band_center_z), rpy=face_rpy),
                material=band_material,
                name=f"{band_name}_{face_index}",
            )

    corner_radius = glass_apothem / math.cos(math.pi / 8.0)
    for corner_index in range(8):
        vertex_angle = (math.pi / 2.0) - (math.pi / 8.0) - corner_index * (math.pi / 4.0)
        x = corner_radius * math.cos(vertex_angle)
        y = corner_radius * math.sin(vertex_angle)
        lantern_body.visual(
            Box((0.10, 0.10, pane_height + 0.14)),
            origin=Origin(xyz=(x, y, pane_center_z)),
            material=cast_iron,
            name=f"corner_post_{corner_index}",
        )

    for face_index in range(8):
        (face_x, face_y, _), face_rpy = _octagon_face_pose(glass_apothem, face_index)
        lantern_body.visual(
            Box((glass_face_width - 0.12, 0.03, pane_height)),
            origin=Origin(xyz=(face_x, face_y, pane_center_z), rpy=face_rpy),
            material=lantern_glass,
            name=f"glass_pane_{face_index}",
        )

    lantern_body.visual(
        mesh_from_geometry(ConeGeometry(radius=1.12, height=roof_height), "roof_cone"),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + roof_height * 0.5)),
        material=roof_iron,
        name="roof_cone",
    )
    lantern_body.visual(
        Cylinder(radius=vent_radius, length=vent_height),
        origin=Origin(
            xyz=(0.0, 0.0, roof_base_z + roof_height + vent_height * 0.5 - 0.03)
        ),
        material=roof_iron,
        name="ventilator",
    )
    lantern_body.visual(
        Sphere(radius=0.11),
        origin=Origin(
            xyz=(0.0, 0.0, roof_base_z + roof_height + vent_height - 0.01)
        ),
        material=warm_brass,
        name="finial",
    )

    lantern_body.visual(
        Cylinder(radius=pedestal_radius, length=pedestal_height),
        origin=Origin(xyz=(0.0, 0.0, pedestal_center_z)),
        material=cast_iron,
        name="central_pedestal",
    )
    lantern_body.visual(
        Cylinder(radius=0.11, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, optic_joint_z - 0.04)),
        material=brass,
        name="optic_bearing",
    )

    optic = model.part("optic")
    optic.visual(
        Cylinder(radius=0.34, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=brass,
        name="turntable",
    )
    optic.visual(
        Cylinder(radius=0.30, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=warm_brass,
        name="lower_crown",
    )
    optic.visual(
        Cylinder(radius=0.42, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=fresnel_glass,
        name="lens_drum",
    )
    optic.visual(
        Cylinder(radius=0.30, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=warm_brass,
        name="upper_crown",
    )
    optic.visual(
        Cylinder(radius=0.08, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=brass,
        name="optic_cap",
    )
    optic.visual(
        Box((0.18, 0.10, 0.22)),
        origin=Origin(xyz=(0.45, 0.0, 0.31)),
        material=brass,
        name="optic_marker",
    )

    maintenance_panel = model.part("maintenance_panel")
    maintenance_panel.visual(
        Box((panel_leaf_width, panel_leaf_thickness, panel_leaf_height)),
        origin=Origin(xyz=(panel_leaf_width * 0.5, 0.0, 0.0)),
        material=cast_iron,
        name="panel_leaf",
    )
    maintenance_panel.visual(
        Box((0.05, 0.012, 0.30)),
        origin=Origin(xyz=(0.04, 0.013, 0.0)),
        material=iron_trim,
        name="panel_hinge_strap",
    )
    maintenance_panel.visual(
        Box((0.05, 0.012, 0.12)),
        origin=Origin(xyz=(0.35, 0.014, 0.0)),
        material=warm_brass,
        name="panel_handle",
    )

    model.articulation(
        "optic_rotation",
        ArticulationType.CONTINUOUS,
        parent=lantern_body,
        child=optic,
        origin=Origin(xyz=(0.0, 0.0, optic_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5),
    )
    model.articulation(
        "maintenance_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=lantern_body,
        child=maintenance_panel,
        origin=Origin(xyz=(panel_hinge_x, panel_hinge_y, panel_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=0.0,
            upper=1.45,
        ),
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

    lantern_body = object_model.get_part("lantern_body")
    optic = object_model.get_part("optic")
    maintenance_panel = object_model.get_part("maintenance_panel")
    optic_rotation = object_model.get_articulation("optic_rotation")
    panel_hinge = object_model.get_articulation("maintenance_panel_hinge")

    ctx.check(
        "optic rotates continuously about vertical shaft",
        optic_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(optic_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"type={optic_rotation.articulation_type}, axis={optic_rotation.axis}",
    )
    ctx.check(
        "maintenance panel uses a vertical side hinge",
        panel_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={panel_hinge.articulation_type}, axis={panel_hinge.axis}",
    )
    ctx.expect_origin_distance(
        optic,
        lantern_body,
        axes="xy",
        max_dist=0.001,
        name="optic stays centered on the pedestal",
    )

    optic_rest_pos = ctx.part_world_position(optic)
    optic_marker_rest = _aabb_center(
        ctx.part_element_world_aabb(optic, elem="optic_marker")
    )
    with ctx.pose({optic_rotation: math.pi * 0.5}):
        optic_turned_pos = ctx.part_world_position(optic)
        optic_marker_turned = _aabb_center(
            ctx.part_element_world_aabb(optic, elem="optic_marker")
        )
    ctx.check(
        "optic spins in place",
        optic_rest_pos is not None
        and optic_turned_pos is not None
        and abs(optic_rest_pos[0] - optic_turned_pos[0]) < 1e-6
        and abs(optic_rest_pos[1] - optic_turned_pos[1]) < 1e-6,
        details=f"rest={optic_rest_pos}, turned={optic_turned_pos}",
    )
    ctx.check(
        "optic marker sweeps around lantern center",
        optic_marker_rest is not None
        and optic_marker_turned is not None
        and abs(optic_marker_rest[0] - optic_marker_turned[0]) > 0.30
        and abs(optic_marker_rest[1] - optic_marker_turned[1]) > 0.30,
        details=f"rest={optic_marker_rest}, turned={optic_marker_turned}",
    )

    panel_closed_center = _aabb_center(ctx.part_world_aabb(maintenance_panel))
    panel_handle_closed = _aabb_center(
        ctx.part_element_world_aabb(maintenance_panel, elem="panel_handle")
    )
    with ctx.pose({panel_hinge: 1.0}):
        panel_handle_open = _aabb_center(
            ctx.part_element_world_aabb(maintenance_panel, elem="panel_handle")
        )
    ctx.check(
        "maintenance panel sits in the lantern curb",
        panel_closed_center is not None
        and abs(panel_closed_center[0]) < 0.02
        and 1.26 < panel_closed_center[1] < 1.34
        and 0.40 < panel_closed_center[2] < 0.44,
        details=f"closed_center={panel_closed_center}",
    )
    ctx.check(
        "maintenance panel swings outward when opened",
        panel_handle_closed is not None
        and panel_handle_open is not None
        and panel_handle_open[1] > panel_handle_closed[1] + 0.16,
        details=f"closed={panel_handle_closed}, open={panel_handle_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
