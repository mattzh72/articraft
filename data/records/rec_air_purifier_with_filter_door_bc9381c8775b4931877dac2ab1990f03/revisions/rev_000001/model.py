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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_cassette_air_purifier")

    plate_white = model.material("plate_white", rgba=(0.93, 0.94, 0.95, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.69, 0.72, 0.75, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.25, 0.27, 0.29, 1.0))
    filter_charcoal = model.material("filter_charcoal", rgba=(0.22, 0.24, 0.25, 1.0))
    filter_media = model.material("filter_media", rgba=(0.53, 0.62, 0.48, 1.0))

    def add_box(part, name, size, xyz, material):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    def add_x_cylinder(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    plate = model.part("ceiling_plate")
    add_box(plate, "left_rail", (0.06, 0.52, 0.018), (-0.33, 0.0, -0.009), plate_white)
    add_box(plate, "right_rail", (0.06, 0.52, 0.018), (0.33, 0.0, -0.009), plate_white)
    add_box(plate, "front_rail", (0.60, 0.016, 0.018), (0.0, 0.252, -0.009), plate_white)
    add_box(plate, "rear_rail", (0.60, 0.06, 0.018), (0.0, -0.23, -0.009), plate_white)
    add_box(
        plate,
        "left_panel_seat",
        (0.03, 0.30, 0.010),
        (-0.285, 0.05, -0.013),
        plate_white,
    )
    add_box(
        plate,
        "right_panel_seat",
        (0.03, 0.30, 0.010),
        (0.285, 0.05, -0.013),
        plate_white,
    )
    add_box(
        plate,
        "rear_panel_seat",
        (0.54, 0.016, 0.010),
        (0.0, -0.10, -0.013),
        plate_white,
    )
    add_box(
        plate,
        "hinge_anchor_bar",
        (0.52, 0.018, 0.008),
        (0.0, 0.191, -0.004),
        dark_hardware,
    )
    for name, x_pos in (
        ("left", -0.21),
        ("center", 0.0),
        ("right", 0.21),
    ):
        add_box(
            plate,
            f"hinge_upright_{name}",
            (0.090, 0.044, 0.010),
            (x_pos, 0.222, 0.005),
            dark_hardware,
        )
    add_x_cylinder(
        plate,
        "hinge_knuckle_left",
        radius=0.0065,
        length=0.09,
        xyz=(-0.21, 0.20, -0.010),
        material=dark_hardware,
    )
    add_x_cylinder(
        plate,
        "hinge_knuckle_center",
        radius=0.0065,
        length=0.09,
        xyz=(0.0, 0.20, -0.010),
        material=dark_hardware,
    )
    add_x_cylinder(
        plate,
        "hinge_knuckle_right",
        radius=0.0065,
        length=0.09,
        xyz=(0.21, 0.20, -0.010),
        material=dark_hardware,
    )

    housing = model.part("housing")
    add_box(housing, "top_panel", (0.56, 0.36, 0.008), (0.0, 0.0, 0.176), housing_gray)
    add_box(housing, "left_wall", (0.018, 0.36, 0.172), (-0.271, 0.0, 0.086), housing_gray)
    add_box(housing, "right_wall", (0.018, 0.36, 0.172), (0.271, 0.0, 0.086), housing_gray)
    add_box(housing, "back_wall", (0.524, 0.016, 0.172), (0.0, -0.172, 0.086), housing_gray)
    add_box(housing, "front_header", (0.524, 0.024, 0.060), (0.0, 0.168, 0.150), housing_gray)
    add_box(
        housing,
        "left_mount_flange",
        (0.040, 0.36, 0.010),
        (-0.280, 0.0, 0.005),
        housing_gray,
    )
    add_box(
        housing,
        "right_mount_flange",
        (0.040, 0.36, 0.010),
        (0.280, 0.0, 0.005),
        housing_gray,
    )
    add_box(
        housing,
        "rear_mount_flange",
        (0.520, 0.040, 0.010),
        (0.0, -0.180, 0.005),
        housing_gray,
    )
    add_box(
        housing,
        "front_mount_flange",
        (0.520, 0.030, 0.010),
        (0.0, 0.185, 0.005),
        housing_gray,
    )
    add_box(
        housing,
        "rear_base_beam",
        (0.524, 0.040, 0.020),
        (0.0, -0.150, 0.010),
        housing_gray,
    )
    add_box(
        housing,
        "left_guide_rail",
        (0.042, 0.290, 0.018),
        (-0.206, -0.015, 0.019),
        dark_hardware,
    )
    add_box(
        housing,
        "right_guide_rail",
        (0.042, 0.290, 0.018),
        (0.206, -0.015, 0.019),
        dark_hardware,
    )
    add_box(
        housing,
        "left_front_bracket",
        (0.060, 0.024, 0.018),
        (-0.241, 0.118, 0.009),
        housing_gray,
    )
    add_box(
        housing,
        "right_front_bracket",
        (0.060, 0.024, 0.018),
        (0.241, 0.118, 0.009),
        housing_gray,
    )

    panel = model.part("access_panel")
    add_box(panel, "left_side", (0.018, 0.30, 0.014), (-0.261, -0.150, -0.015), grille_gray)
    add_box(panel, "right_side", (0.018, 0.30, 0.014), (0.261, -0.150, -0.015), grille_gray)
    add_box(panel, "front_stile", (0.504, 0.018, 0.014), (0.0, -0.009, -0.015), grille_gray)
    add_box(panel, "rear_stile", (0.504, 0.018, 0.014), (0.0, -0.291, -0.015), grille_gray)
    for index, slat_y in enumerate((-0.055, -0.095, -0.135, -0.175, -0.215, -0.255), start=1):
        add_box(
            panel,
            f"slat_{index}",
            (0.504, 0.012, 0.009),
            (0.0, slat_y, -0.014),
            grille_gray,
        )
    add_box(panel, "pull_lip", (0.12, 0.016, 0.012), (0.0, -0.302, -0.014), dark_hardware)
    add_box(
        panel,
        "hinge_leaf_left",
        (0.060, 0.012, 0.010),
        (-0.105, -0.006, -0.006),
        dark_hardware,
    )
    add_box(
        panel,
        "hinge_leaf_right",
        (0.060, 0.012, 0.010),
        (0.105, -0.006, -0.006),
        dark_hardware,
    )

    filter_block = model.part("filter_block")
    add_box(
        filter_block,
        "cartridge_body",
        (0.40, 0.30, 0.050),
        (0.0, -0.150, 0.025),
        filter_charcoal,
    )
    for index, rib_x in enumerate((-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15), start=1):
        add_box(
            filter_block,
            f"pleat_rib_{index}",
            (0.018, 0.28, 0.010),
            (rib_x, -0.150, 0.0),
            filter_media,
        )
    add_box(
        filter_block,
        "left_shoe",
        (0.024, 0.280, 0.012),
        (-0.206, -0.150, -0.006),
        dark_hardware,
    )
    add_box(
        filter_block,
        "right_shoe",
        (0.024, 0.280, 0.012),
        (0.206, -0.150, -0.006),
        dark_hardware,
    )
    add_box(
        filter_block,
        "pull_handle",
        (0.160, 0.014, 0.014),
        (0.0, 0.007, 0.024),
        dark_hardware,
    )

    model.articulation(
        "plate_to_housing",
        ArticulationType.FIXED,
        parent=plate,
        child=housing,
    )
    model.articulation(
        "plate_to_panel",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=panel,
        origin=Origin(xyz=(0.0, 0.20, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_filter",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_block,
        origin=Origin(xyz=(0.0, 0.165, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("ceiling_plate")
    housing = object_model.get_part("housing")
    panel = object_model.get_part("access_panel")
    filter_block = object_model.get_part("filter_block")
    panel_hinge = object_model.get_articulation("plate_to_panel")
    filter_slide = object_model.get_articulation("housing_to_filter")

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

    ctx.expect_contact(
        housing,
        plate,
        name="housing is mounted to the ceiling plate frame",
    )
    ctx.expect_contact(
        panel,
        plate,
        elem_a="rear_stile",
        elem_b="rear_panel_seat",
        name="access panel seats against the ceiling frame when closed",
    )
    ctx.expect_contact(
        filter_block,
        housing,
        elem_a="left_shoe",
        elem_b="left_guide_rail",
        name="filter block rides on the left guide rail",
    )
    ctx.expect_contact(
        filter_block,
        housing,
        elem_a="right_shoe",
        elem_b="right_guide_rail",
        name="filter block rides on the right guide rail",
    )
    ctx.expect_within(
        filter_block,
        housing,
        axes="x",
        margin=0.0,
        name="filter block stays centered between the housing walls",
    )
    ctx.expect_overlap(
        filter_block,
        housing,
        axes="y",
        min_overlap=0.29,
        name="inserted filter block fills the housing depth",
    )

    closed_panel_aabb = ctx.part_world_aabb(panel)
    panel_open = panel_hinge.motion_limits.upper if panel_hinge.motion_limits is not None else 0.0
    with ctx.pose({panel_hinge: panel_open}):
        open_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "access panel drops downward when opened",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][2] < closed_panel_aabb[0][2] - 0.08,
        details=f"closed_aabb={closed_panel_aabb}, open_aabb={open_panel_aabb}",
    )

    rest_filter_aabb = ctx.part_world_aabb(filter_block)
    filter_out = filter_slide.motion_limits.upper if filter_slide.motion_limits is not None else 0.0
    with ctx.pose({filter_slide: filter_out}):
        extended_filter_aabb = ctx.part_world_aabb(filter_block)
        ctx.expect_within(
            filter_block,
            housing,
            axes="x",
            margin=0.0,
            name="extended filter stays laterally aligned on the guide rails",
        )
        ctx.expect_overlap(
            filter_block,
            housing,
            axes="y",
            min_overlap=0.12,
            name="extended filter retains insertion on the guide rails",
        )
    ctx.check(
        "filter block slides outward from the housing front",
        rest_filter_aabb is not None
        and extended_filter_aabb is not None
        and extended_filter_aabb[1][1] > rest_filter_aabb[1][1] + 0.14,
        details=f"rest_aabb={rest_filter_aabb}, extended_aabb={extended_filter_aabb}",
    )

    with ctx.pose({panel_hinge: panel_open, filter_slide: filter_out}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="opened access panel and extended filter remain collision-free"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
