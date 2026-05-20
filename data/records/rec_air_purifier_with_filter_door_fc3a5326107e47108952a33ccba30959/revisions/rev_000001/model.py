from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HOUSING_W = 0.58
HOUSING_D = 0.32
HOUSING_H = 0.12
WALL_T = 0.006
TOP_T = 0.006
FRONT_T = 0.008
BACK_T = 0.006

PANEL_W = 0.552
PANEL_D = 0.266
PANEL_T = 0.004
HINGE_Y = -0.150
PANEL_OPEN_ANGLE = 1.20

DRAWER_W = 0.49
DRAWER_D = 0.22
DRAWER_H = 0.054
DRAWER_CENTER_Z = 0.041
DRAWER_DROP = 0.05


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_desk_air_purifier")

    housing_color = model.material("housing_color", rgba=(0.19, 0.20, 0.22, 1.0))
    grille_color = model.material("grille_color", rgba=(0.10, 0.11, 0.12, 1.0))
    panel_color = model.material("panel_color", rgba=(0.16, 0.17, 0.19, 1.0))
    rail_color = model.material("rail_color", rgba=(0.55, 0.57, 0.60, 1.0))
    filter_frame_color = model.material("filter_frame_color", rgba=(0.18, 0.21, 0.23, 1.0))
    filter_media_color = model.material("filter_media_color", rgba=(0.88, 0.92, 0.89, 1.0))

    # x = width, y = front-to-back depth, z = height from the floor-facing underside.
    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_W, HOUSING_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_H - TOP_T / 2.0)),
        material=housing_color,
        name="top_cover",
    )
    housing.visual(
        Box((WALL_T, HOUSING_D - FRONT_T + 0.002, HOUSING_H - TOP_T)),
        origin=Origin(
            xyz=(
                -HOUSING_W / 2.0 + WALL_T / 2.0,
                0.004,
                (HOUSING_H - TOP_T) / 2.0,
            )
        ),
        material=housing_color,
        name="left_wall",
    )
    housing.visual(
        Box((WALL_T, HOUSING_D - FRONT_T + 0.002, HOUSING_H - TOP_T)),
        origin=Origin(
            xyz=(
                HOUSING_W / 2.0 - WALL_T / 2.0,
                0.004,
                (HOUSING_H - TOP_T) / 2.0,
            )
        ),
        material=housing_color,
        name="right_wall",
    )
    housing.visual(
        Box((HOUSING_W - 2.0 * WALL_T + 0.004, BACK_T, HOUSING_H - TOP_T)),
        origin=Origin(
            xyz=(
                0.0,
                HOUSING_D / 2.0 - BACK_T / 2.0,
                (HOUSING_H - TOP_T) / 2.0,
            )
        ),
        material=housing_color,
        name="back_wall",
    )
    housing.visual(
        Box((HOUSING_W - 2.0 * WALL_T + 0.004, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, 0.146, 0.008)),
        material=housing_color,
        name="rear_lower_beam",
    )

    housing.visual(
        Box((HOUSING_W - 2.0 * WALL_T + 0.004, FRONT_T, 0.018)),
        origin=Origin(xyz=(0.0, -HOUSING_D / 2.0 + FRONT_T / 2.0, 0.009)),
        material=housing_color,
        name="front_lower_beam",
    )
    housing.visual(
        Box((HOUSING_W - 2.0 * WALL_T + 0.004, FRONT_T, 0.020)),
        origin=Origin(xyz=(0.0, -HOUSING_D / 2.0 + FRONT_T / 2.0, 0.110)),
        material=housing_color,
        name="front_upper_beam",
    )
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Box((0.026, FRONT_T, 0.084)),
            origin=Origin(
                xyz=(
                    x_sign * 0.271,
                    -HOUSING_D / 2.0 + FRONT_T / 2.0,
                    0.058,
                )
            ),
            material=housing_color,
            name=f"front_{side}_stile",
        )

    housing.visual(
        Box((0.516, 0.010, 0.068)),
        origin=Origin(xyz=(0.0, -0.147, 0.058)),
        material=grille_color,
        name="grille_backing",
    )
    for index in range(8):
        housing.visual(
            Box((0.516, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, -0.156, 0.028 + 0.008 * index)),
            material=grille_color,
            name=f"grille_slat_{index}",
        )

    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Box((0.014, PANEL_D, 0.006)),
            origin=Origin(xyz=(x_sign * 0.277, -0.017, 0.007)),
            material=housing_color,
            name=f"panel_{side}_seat",
        )
    housing.visual(
        Box((PANEL_W, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.109, 0.007)),
        material=housing_color,
        name="panel_rear_seat",
    )

    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        housing.visual(
            Box((0.032, DRAWER_D + 0.004, 0.082)),
            origin=Origin(xyz=(x_sign * 0.268, 0.0, 0.051)),
            material=rail_color,
            name=f"{side}_drawer_track",
        )

    for index, center_x in enumerate((-0.16, 0.16)):
        housing.visual(
            Box((0.14, 0.05, 0.008)),
            origin=Origin(xyz=(center_x, 0.0, 0.120)),
            material=housing_color,
            name=f"mount_pad_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_W, HOUSING_D, HOUSING_H)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_H / 2.0)),
    )

    bottom_panel = model.part("bottom_panel")
    bottom_panel.visual(
        Box((PANEL_W, PANEL_D, PANEL_T)),
        origin=Origin(xyz=(0.0, PANEL_D / 2.0, PANEL_T / 2.0)),
        material=panel_color,
        name="panel_plate",
    )
    for index, center_x in enumerate((-0.16, 0.16)):
        bottom_panel.visual(
            Box((0.08, PANEL_D - 0.05, 0.008)),
            origin=Origin(xyz=(center_x, PANEL_D / 2.0 + 0.004, -0.003)),
            material=panel_color,
            name=f"stiffener_rib_{index}",
        )
    bottom_panel.visual(
        Box((0.12, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, PANEL_D - 0.007, -0.003)),
        material=panel_color,
        name="pull_lip",
    )
    bottom_panel.inertial = Inertial.from_geometry(
        Box((PANEL_W, PANEL_D, 0.012)),
        mass=0.85,
        origin=Origin(xyz=(0.0, PANEL_D / 2.0, 0.001)),
    )

    filter_drawer = model.part("filter_drawer")
    filter_drawer.visual(
        Box((DRAWER_W, DRAWER_D - 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=filter_frame_color,
        name="drawer_base",
    )
    filter_drawer.visual(
        Box((0.003, DRAWER_D - 0.006, 0.045)),
        origin=Origin(xyz=(-DRAWER_W / 2.0 + 0.0015, 0.0, -0.002)),
        material=filter_frame_color,
        name="left_side_wall",
    )
    filter_drawer.visual(
        Box((0.003, DRAWER_D - 0.006, 0.045)),
        origin=Origin(xyz=(DRAWER_W / 2.0 - 0.0015, 0.0, -0.002)),
        material=filter_frame_color,
        name="right_side_wall",
    )
    filter_drawer.visual(
        Box((DRAWER_W, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -DRAWER_D / 2.0 + 0.005, -0.001)),
        material=filter_frame_color,
        name="drawer_front",
    )
    filter_drawer.visual(
        Box((DRAWER_W, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, DRAWER_D / 2.0 - 0.005, -0.006)),
        material=filter_frame_color,
        name="drawer_rear",
    )
    filter_drawer.visual(
        Box((0.008, DRAWER_D - 0.008, 0.050)),
        origin=Origin(xyz=(-0.248, 0.0, 0.0)),
        material=rail_color,
        name="left_runner",
    )
    filter_drawer.visual(
        Box((0.008, DRAWER_D - 0.008, 0.050)),
        origin=Origin(xyz=(0.248, 0.0, 0.0)),
        material=rail_color,
        name="right_runner",
    )
    filter_drawer.visual(
        Box((0.452, 0.182, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=filter_media_color,
        name="filter_media_core",
    )
    for index in range(8):
        filter_drawer.visual(
            Box((0.452, 0.004, 0.042)),
            origin=Origin(xyz=(0.0, -0.077 + 0.022 * index, -0.002)),
            material=filter_media_color,
            name=f"pleat_{index}",
        )
    filter_drawer.visual(
        Box((0.12, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.101, -0.029)),
        material=filter_frame_color,
        name="drawer_handle",
    )
    filter_drawer.inertial = Inertial.from_geometry(
        Box((0.504, DRAWER_D, DRAWER_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_bottom_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=bottom_panel,
        origin=Origin(xyz=(0.0, HINGE_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_drawer,
        origin=Origin(xyz=(0.0, 0.0, DRAWER_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.12,
            lower=0.0,
            upper=DRAWER_DROP,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    bottom_panel = object_model.get_part("bottom_panel")
    filter_drawer = object_model.get_part("filter_drawer")
    panel_hinge = object_model.get_articulation("housing_to_bottom_panel")
    drawer_slide = object_model.get_articulation("housing_to_filter_drawer")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "bottom_panel_hinge_axis",
        panel_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"expected hinge axis (-1, 0, 0), got {panel_hinge.axis}",
    )
    ctx.check(
        "filter_drawer_slide_axis",
        drawer_slide.axis == (0.0, 0.0, -1.0),
        details=f"expected drawer slide axis (0, 0, -1), got {drawer_slide.axis}",
    )

    with ctx.pose({panel_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_contact(
            bottom_panel,
            housing,
            contact_tol=0.001,
            name="bottom_panel_seated_in_housing",
        )
        ctx.expect_gap(
            filter_drawer,
            bottom_panel,
            axis="z",
            min_gap=0.001,
            max_gap=0.004,
            name="drawer_clear_of_closed_panel",
        )
        ctx.expect_within(
            filter_drawer,
            housing,
            axes="xy",
            margin=0.0,
            name="drawer_within_housing_plan",
        )
        ctx.expect_contact(
            filter_drawer,
            housing,
            contact_tol=0.001,
            name="drawer_captured_by_tracks",
        )

    with ctx.pose({panel_hinge: PANEL_OPEN_ANGLE, drawer_slide: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_panel_pose_no_overlaps")
        panel_aabb = ctx.part_world_aabb(bottom_panel)
        housing_aabb = ctx.part_world_aabb(housing)
        panel_below = (
            panel_aabb is not None
            and housing_aabb is not None
            and panel_aabb[0][2] < housing_aabb[0][2] - 0.08
        )
        ctx.check(
            "panel_drops_below_housing",
            panel_below,
            details=f"panel aabb={panel_aabb}, housing aabb={housing_aabb}",
        )

    with ctx.pose({panel_hinge: PANEL_OPEN_ANGLE, drawer_slide: DRAWER_DROP}):
        ctx.fail_if_parts_overlap_in_current_pose(name="service_pose_no_overlaps")
        drawer_aabb = ctx.part_world_aabb(filter_drawer)
        housing_aabb = ctx.part_world_aabb(housing)
        drawer_extracted = (
            drawer_aabb is not None
            and housing_aabb is not None
            and drawer_aabb[0][2] < housing_aabb[0][2] - 0.03
        )
        ctx.check(
            "drawer_drops_below_housing",
            drawer_extracted,
            details=f"drawer aabb={drawer_aabb}, housing aabb={housing_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
