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
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_panel_insert(width: float, height: float):
    insert = SlotPatternPanelGeometry(
        (width, height),
        0.004,
        slot_size=(0.034, 0.008),
        pitch=(0.050, 0.040),
        frame=0.010,
        corner_radius=0.008,
        stagger=True,
        center=True,
    )
    insert.rotate_x(pi / 2.0)
    return insert


def _add_panel_visuals(
    part,
    *,
    panel_width: float,
    panel_height: float,
    panel_thickness: float,
    bottom_gap: float,
    stile_width: float,
    rail_height: float,
    hinge_clear: float,
    y_sign: float,
    frame_material,
    insert_mesh,
    insert_material,
    hinge_material,
    include_right_hinge: bool = True,
    include_end_bumper: bool = False,
) -> None:
    frame_center_y = y_sign * panel_thickness * 0.5
    right_hinge_y = y_sign * panel_thickness
    frame_span_x = panel_width - 2.0 * (stile_width + hinge_clear)
    inner_width = frame_span_x + 0.002
    inner_height = panel_height - 2.0 * rail_height + 0.002
    bottom_rail_center_z = bottom_gap + rail_height * 0.5
    top_rail_center_z = bottom_gap + panel_height - rail_height * 0.5
    mid_rail_center_z = bottom_gap + panel_height * 0.57
    barrel_radius = 0.009
    left_leaf_depth = 0.010
    right_leaf_depth = 0.010
    left_barrel_length = 0.320
    right_barrel_length = 0.180
    left_barrel_center_z = 0.540
    right_barrel_centers_z = (0.290, 0.790)
    left_leaf_height = 0.300
    right_leaf_height = 0.760
    hinge_strap_center_z = left_barrel_center_z

    part.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(hinge_clear + stile_width * 0.5, frame_center_y, bottom_gap + panel_height * 0.5)),
        material=frame_material,
        name="left_stile",
    )
    part.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(
            xyz=(panel_width - hinge_clear - stile_width * 0.5, frame_center_y, bottom_gap + panel_height * 0.5)
        ),
        material=frame_material,
        name="right_stile",
    )
    part.visual(
        Box((frame_span_x, panel_thickness, rail_height)),
        origin=Origin(xyz=(panel_width * 0.5, frame_center_y, bottom_rail_center_z)),
        material=frame_material,
        name="bottom_rail",
    )
    part.visual(
        Box((frame_span_x, panel_thickness, rail_height)),
        origin=Origin(xyz=(panel_width * 0.5, frame_center_y, top_rail_center_z)),
        material=frame_material,
        name="top_rail",
    )
    part.visual(
        Box((frame_span_x, panel_thickness, 0.048)),
        origin=Origin(xyz=(panel_width * 0.5, frame_center_y, mid_rail_center_z)),
        material=frame_material,
        name="mid_rail",
    )
    part.visual(
        insert_mesh,
        origin=Origin(xyz=(panel_width * 0.5, frame_center_y, bottom_gap + panel_height * 0.5)),
        material=insert_material,
        name="panel_insert",
    )

    part.visual(
        Box((0.034, left_leaf_depth, left_leaf_height)),
        origin=Origin(xyz=(0.017, y_sign * (barrel_radius + left_leaf_depth * 0.5), hinge_strap_center_z)),
        material=hinge_material,
        name="left_hinge_leaf",
    )
    part.visual(
        Cylinder(radius=barrel_radius, length=left_barrel_length),
        origin=Origin(xyz=(0.0, 0.0, left_barrel_center_z)),
        material=hinge_material,
        name="left_hinge_barrel",
    )

    if include_right_hinge:
        part.visual(
            Box((0.034, right_leaf_depth, right_leaf_height)),
            origin=Origin(
                xyz=(
                    panel_width - 0.017,
                    frame_center_y,
                    hinge_strap_center_z,
                )
            ),
            material=hinge_material,
            name="right_hinge_leaf",
        )
        for index, center_z in enumerate(right_barrel_centers_z):
            part.visual(
                Cylinder(radius=barrel_radius, length=right_barrel_length),
                origin=Origin(xyz=(panel_width, right_hinge_y, center_z)),
                material=hinge_material,
                name=f"right_hinge_barrel_{index}",
            )

    if include_end_bumper:
        part.visual(
            Box((0.018, panel_thickness + 0.012, 0.240)),
            origin=Origin(
                xyz=(
                    panel_width - 0.009,
                    frame_center_y,
                    bottom_gap + panel_height * 0.56,
                )
            ),
            material=insert_material,
            name="end_bumper",
        )
        part.visual(
            Box((0.032, 0.010, 0.120)),
            origin=Origin(
                xyz=(
                    panel_width - 0.020,
                    frame_center_y + y_sign * (panel_thickness * 0.5 + 0.005),
                    bottom_gap + panel_height * 0.62,
                )
            ),
            material=hinge_material,
            name="pull_handle",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_pedestrian_barrier")

    powder_yellow = model.material("powder_yellow", rgba=(0.92, 0.76, 0.12, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.16, 0.18, 0.20, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.60, 0.63, 0.67, 1.0))
    wall_steel = model.material("wall_steel", rgba=(0.43, 0.46, 0.50, 1.0))

    panel_width = 0.55
    panel_height = 1.02
    panel_thickness = 0.028
    bottom_gap = 0.03
    stile_width = 0.028
    rail_height = 0.060
    hinge_clear = 0.016
    overall_height = bottom_gap + panel_height

    insert_mesh = mesh_from_geometry(
        _build_panel_insert(
            panel_width - 2.0 * (stile_width + hinge_clear) + 0.002,
            panel_height - 2.0 * rail_height + 0.002,
        ),
        "barrier_panel_insert",
    )

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.018, 0.160, 1.10)),
        origin=Origin(xyz=(-0.009, 0.0, 0.55)),
        material=wall_steel,
        name="wall_plate",
    )
    for side_sign in (-1.0, 1.0):
        wall_bracket.visual(
            Box((0.020, 0.010, 0.760)),
            origin=Origin(xyz=(0.004, side_sign * 0.014, 0.540)),
            material=wall_steel,
            name=f"hinge_spine_{'pos' if side_sign > 0.0 else 'neg'}",
        )
    for center_z in (0.290, 0.790):
        wall_bracket.visual(
            Box((0.026, 0.050, 0.140)),
            origin=Origin(xyz=(-0.001, 0.0, center_z)),
            material=wall_steel,
            name=f"bracket_arm_{int(center_z * 100)}",
        )
        wall_bracket.visual(
            Cylinder(radius=0.009, length=0.180),
            origin=Origin(xyz=(0.015, 0.0, center_z)),
            material=hinge_steel,
            name=f"wall_hinge_barrel_{int(center_z * 100)}",
        )
    for index, anchor_z in enumerate((0.16, 0.40, 0.70, 0.94)):
        wall_bracket.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(-0.005, 0.0, anchor_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_steel,
            name=f"anchor_bolt_{index}",
        )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.080, 0.180, overall_height + 0.05)),
        mass=14.0,
        origin=Origin(xyz=(-0.010, 0.0, overall_height * 0.5)),
    )

    panel_1 = model.part("panel_1")
    _add_panel_visuals(
        panel_1,
        panel_width=panel_width,
        panel_height=panel_height,
        panel_thickness=panel_thickness,
        bottom_gap=bottom_gap,
        stile_width=stile_width,
        rail_height=rail_height,
        hinge_clear=hinge_clear,
        y_sign=1.0,
        frame_material=powder_yellow,
        insert_mesh=insert_mesh,
        insert_material=dark_insert,
        hinge_material=hinge_steel,
    )
    panel_1.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness * 2.0, overall_height)),
        mass=9.0,
        origin=Origin(xyz=(panel_width * 0.5, panel_thickness * 0.5, overall_height * 0.5)),
    )

    panel_2 = model.part("panel_2")
    _add_panel_visuals(
        panel_2,
        panel_width=panel_width,
        panel_height=panel_height,
        panel_thickness=panel_thickness,
        bottom_gap=bottom_gap,
        stile_width=stile_width,
        rail_height=rail_height,
        hinge_clear=hinge_clear,
        y_sign=-1.0,
        frame_material=powder_yellow,
        insert_mesh=insert_mesh,
        insert_material=dark_insert,
        hinge_material=hinge_steel,
    )
    panel_2.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness * 2.0, overall_height)),
        mass=9.0,
        origin=Origin(xyz=(panel_width * 0.5, -panel_thickness * 0.5, overall_height * 0.5)),
    )

    panel_3 = model.part("panel_3")
    _add_panel_visuals(
        panel_3,
        panel_width=panel_width,
        panel_height=panel_height,
        panel_thickness=panel_thickness,
        bottom_gap=bottom_gap,
        stile_width=stile_width,
        rail_height=rail_height,
        hinge_clear=hinge_clear,
        y_sign=1.0,
        frame_material=powder_yellow,
        insert_mesh=insert_mesh,
        insert_material=dark_insert,
        hinge_material=hinge_steel,
        include_right_hinge=False,
        include_end_bumper=True,
    )
    panel_3.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness * 2.2, overall_height)),
        mass=9.5,
        origin=Origin(xyz=(panel_width * 0.5, panel_thickness * 0.5, overall_height * 0.5)),
    )

    model.articulation(
        "wall_to_panel_1",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=panel_1,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.60),
    )
    model.articulation(
        "panel_1_to_panel_2",
        ArticulationType.REVOLUTE,
        parent=panel_1,
        child=panel_2,
        origin=Origin(xyz=(panel_width, panel_thickness, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.8, lower=0.0, upper=3.10),
    )
    model.articulation(
        "panel_2_to_panel_3",
        ArticulationType.REVOLUTE,
        parent=panel_2,
        child=panel_3,
        origin=Origin(xyz=(panel_width, -panel_thickness, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.8, lower=0.0, upper=3.10),
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

    panel_1 = object_model.get_part("panel_1")
    panel_2 = object_model.get_part("panel_2")
    panel_3 = object_model.get_part("panel_3")
    wall_joint = object_model.get_articulation("wall_to_panel_1")
    hinge_12 = object_model.get_articulation("panel_1_to_panel_2")
    hinge_23 = object_model.get_articulation("panel_2_to_panel_3")

    with ctx.pose({wall_joint: 0.0, hinge_12: 0.0, hinge_23: 0.0}):
        ctx.expect_origin_gap(
            panel_2,
            panel_1,
            axis="x",
            min_gap=0.549,
            max_gap=0.551,
            name="panel 2 hinge origin sits one panel width from panel 1",
        )
        ctx.expect_origin_gap(
            panel_3,
            panel_2,
            axis="x",
            min_gap=0.549,
            max_gap=0.551,
            name="panel 3 hinge origin sits one panel width from panel 2",
        )
        extended_aabb = ctx.part_world_aabb(panel_3)
        ctx.check(
            "extended barrier spans roughly 1.65 m",
            extended_aabb is not None and extended_aabb[1][0] > 1.62,
            details=f"panel_3_aabb={extended_aabb}",
        )

    with ctx.pose({wall_joint: pi / 2.0, hinge_12: pi, hinge_23: pi}):
        folded_aabbs = [ctx.part_world_aabb(panel) for panel in (panel_1, panel_2, panel_3)]
        valid_fold = all(aabb is not None for aabb in folded_aabbs)
        if valid_fold:
            min_x = min(aabb[0][0] for aabb in folded_aabbs if aabb is not None)
            max_x = max(aabb[1][0] for aabb in folded_aabbs if aabb is not None)
            min_y = min(aabb[0][1] for aabb in folded_aabbs if aabb is not None)
            max_y = max(aabb[1][1] for aabb in folded_aabbs if aabb is not None)
        else:
            min_x = max_x = min_y = max_y = None
        ctx.check(
            "folded barrier stacks close to the wall",
            valid_fold
            and min_x is not None
            and max_x is not None
            and min_y is not None
            and max_y is not None
            and (max_x - min_x) < 0.12
            and (max_y - min_y) < 0.62,
            details=f"folded_panel_extents={(min_x, max_x, min_y, max_y)}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
