from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="commercial_ice_cream_display_freezer")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.19, 1.0))
    caster_black = model.material("caster_black", rgba=(0.11, 0.11, 0.12, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.72, 0.84, 0.92, 0.35))

    body = model.part("body")

    body.visual(
        Box((1.55, 0.78, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=body_white,
        name="floor",
    )
    body.visual(
        Box((1.55, 0.05, 0.80)),
        origin=Origin(xyz=(0.0, 0.365, 0.495)),
        material=body_white,
        name="front_wall",
    )
    body.visual(
        Box((1.55, 0.05, 0.80)),
        origin=Origin(xyz=(0.0, -0.365, 0.495)),
        material=body_white,
        name="back_wall",
    )
    body.visual(
        Box((0.05, 0.68, 0.80)),
        origin=Origin(xyz=(-0.75, 0.0, 0.495)),
        material=body_white,
        name="left_end_wall",
    )
    body.visual(
        Box((0.05, 0.68, 0.80)),
        origin=Origin(xyz=(0.75, 0.0, 0.495)),
        material=body_white,
        name="right_end_wall",
    )
    body.visual(
        Box((0.035, 0.68, 0.80)),
        origin=Origin(xyz=(0.4575, 0.0, 0.495)),
        material=trim_gray,
        name="compressor_bulkhead",
    )
    body.visual(
        Box((0.25, 0.68, 0.035)),
        origin=Origin(xyz=(0.60, 0.0, 0.8775)),
        material=trim_gray,
        name="compressor_top",
    )

    def add_caster(name: str, x: float, y: float) -> None:
        caster = model.part(name)
        caster.visual(
            Box((0.09, 0.07, 0.012)),
            material=trim_gray,
            name="mount_plate",
        )
        caster.visual(
            Cylinder(radius=0.01, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, -0.024)),
            material=dark_trim,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.012, 0.008, 0.074)),
            origin=Origin(xyz=(0.0, 0.010, -0.043)),
            material=dark_trim,
            name="fork_left",
        )
        caster.visual(
            Box((0.012, 0.008, 0.074)),
            origin=Origin(xyz=(0.0, -0.010, -0.043)),
            material=dark_trim,
            name="fork_right",
        )
        caster.visual(
            Cylinder(radius=0.032, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, -0.074), rpy=(1.5707963267948966, 0.0, 0.0)),
            material=caster_black,
            name="wheel",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.FIXED,
            parent=body,
            child=caster,
            origin=Origin(xyz=(x, y, 0.089)),
        )

    add_caster("front_left_caster", -0.58, 0.28)
    add_caster("front_right_caster", 0.58, 0.28)
    add_caster("rear_left_caster", -0.58, -0.28)
    add_caster("rear_right_caster", 0.58, -0.28)

    front_rail = model.part("front_rail")
    front_rail.visual(
        Box((1.55, 0.042, 0.018)),
        material=rail_aluminum,
        name="base",
    )
    front_rail.visual(
        Box((1.55, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.016, 0.019)),
        material=rail_aluminum,
        name="outer_lip",
    )
    front_rail.visual(
        Box((1.55, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.016, 0.017)),
        material=rail_aluminum,
        name="inner_lip",
    )
    model.articulation(
        "body_to_front_rail",
        ArticulationType.FIXED,
        parent=body,
        child=front_rail,
        origin=Origin(xyz=(0.0, 0.365, 0.904)),
    )

    rear_rail = model.part("rear_rail")
    rear_rail.visual(
        Box((1.55, 0.042, 0.018)),
        material=rail_aluminum,
        name="base",
    )
    rear_rail.visual(
        Box((1.55, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.016, 0.019)),
        material=rail_aluminum,
        name="outer_lip",
    )
    rear_rail.visual(
        Box((1.55, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.016, 0.017)),
        material=rail_aluminum,
        name="inner_lip",
    )
    model.articulation(
        "body_to_rear_rail",
        ArticulationType.FIXED,
        parent=body,
        child=rear_rail,
        origin=Origin(xyz=(0.0, -0.365, 0.904)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((1.15, 0.014, 0.012)),
        origin=Origin(xyz=(0.575, 0.362, 0.006)),
        material=dark_trim,
        name="runner_front",
    )
    lid.visual(
        Box((1.15, 0.014, 0.012)),
        origin=Origin(xyz=(0.575, -0.362, 0.006)),
        material=dark_trim,
        name="runner_rear",
    )
    lid.visual(
        Box((1.15, 0.074, 0.024)),
        origin=Origin(xyz=(0.575, 0.332, 0.024)),
        material=rail_aluminum,
        name="front_frame",
    )
    lid.visual(
        Box((1.15, 0.074, 0.024)),
        origin=Origin(xyz=(0.575, -0.332, 0.024)),
        material=rail_aluminum,
        name="rear_frame",
    )
    lid.visual(
        Box((0.035, 0.62, 0.024)),
        origin=Origin(xyz=(0.0175, 0.0, 0.024)),
        material=rail_aluminum,
        name="left_bar",
    )
    lid.visual(
        Box((0.035, 0.62, 0.024)),
        origin=Origin(xyz=(1.1325, 0.0, 0.024)),
        material=rail_aluminum,
        name="right_bar",
    )
    lid.visual(
        Box((1.08, 0.59, 0.006)),
        origin=Origin(xyz=(0.575, 0.0, 0.021)),
        material=glass_tint,
        name="glass",
    )
    lid.visual(
        Box((0.08, 0.06, 0.018)),
        origin=Origin(xyz=(0.04, 0.0, 0.045)),
        material=dark_trim,
        name="handle",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.72, 0.0, 0.913)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=0.23,
        ),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.46, 0.018, 0.36)),
        origin=Origin(xyz=(-0.23, -0.007, 0.18)),
        material=trim_gray,
        name="panel_shell",
    )
    access_panel.visual(
        Cylinder(radius=0.009, length=0.36),
        origin=Origin(xyz=(0.009, -0.007, 0.18)),
        material=dark_trim,
        name="hinge_barrel",
    )
    access_panel.visual(
        Box((0.30, 0.002, 0.02)),
        origin=Origin(xyz=(-0.23, -0.017, 0.10)),
        material=dark_trim,
        name="louver_lower",
    )
    access_panel.visual(
        Box((0.30, 0.002, 0.02)),
        origin=Origin(xyz=(-0.23, -0.017, 0.17)),
        material=dark_trim,
        name="louver_mid",
    )
    access_panel.visual(
        Box((0.30, 0.002, 0.02)),
        origin=Origin(xyz=(-0.23, -0.017, 0.24)),
        material=dark_trim,
        name="louver_upper",
    )
    access_panel.visual(
        Box((0.04, 0.018, 0.08)),
        origin=Origin(xyz=(-0.40, -0.025, 0.19)),
        material=dark_trim,
        name="pull",
    )
    model.articulation(
        "body_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_panel,
        origin=Origin(xyz=(0.72, -0.392, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_rail = object_model.get_part("front_rail")
    rear_rail = object_model.get_part("rear_rail")
    lid = object_model.get_part("lid")
    access_panel = object_model.get_part("access_panel")
    lid_slide = object_model.get_articulation("body_to_lid")
    panel_hinge = object_model.get_articulation("body_to_access_panel")
    casters = [
        object_model.get_part("front_left_caster"),
        object_model.get_part("front_right_caster"),
        object_model.get_part("rear_left_caster"),
        object_model.get_part("rear_right_caster"),
    ]

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
        front_rail,
        body,
        elem_a="base",
        elem_b="front_wall",
        name="front rail is mounted on the cabinet front edge",
    )
    ctx.expect_contact(
        rear_rail,
        body,
        elem_a="base",
        elem_b="back_wall",
        name="rear rail is mounted on the cabinet rear edge",
    )

    for caster in casters:
        ctx.expect_contact(
            caster,
            body,
            elem_a="mount_plate",
            elem_b="floor",
            name=f"{caster.name} is bolted to the freezer underside",
        )

    lid_rest_position = ctx.part_world_position(lid)

    with ctx.pose({lid_slide: 0.0}):
        ctx.expect_contact(
            lid,
            front_rail,
            elem_a="runner_front",
            elem_b="base",
            name="front lid runner sits on the front guide rail",
        )
        ctx.expect_contact(
            lid,
            rear_rail,
            elem_a="runner_rear",
            elem_b="base",
            name="rear lid runner sits on the rear guide rail",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="left_bar",
            negative_elem="left_end_wall",
            min_gap=0.003,
            max_gap=0.02,
            name="closed lid clears the left cabinet end wall",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="x",
            positive_elem="compressor_bulkhead",
            negative_elem="right_bar",
            min_gap=0.004,
            max_gap=0.03,
            name="closed lid stops before the compressor bulkhead",
        )

    with ctx.pose({lid_slide: lid_slide.motion_limits.upper}):
        ctx.expect_contact(
            lid,
            front_rail,
            elem_a="runner_front",
            elem_b="base",
            name="front lid runner remains supported when opened",
        )
        ctx.expect_contact(
            lid,
            rear_rail,
            elem_a="runner_rear",
            elem_b="base",
            name="rear lid runner remains supported when opened",
        )
        ctx.expect_overlap(
            lid,
            front_rail,
            axes="x",
            elem_a="runner_front",
            elem_b="base",
            min_overlap=1.05,
            name="front guide keeps long overlap with the sliding lid",
        )
        ctx.expect_overlap(
            lid,
            rear_rail,
            axes="x",
            elem_a="runner_rear",
            elem_b="base",
            min_overlap=1.05,
            name="rear guide keeps long overlap with the sliding lid",
        )
        lid_open_position = ctx.part_world_position(lid)

    ctx.check(
        "lid slides toward the compressor end",
        lid_rest_position is not None
        and lid_open_position is not None
        and lid_open_position[0] > lid_rest_position[0] + 0.15,
        details=f"rest={lid_rest_position}, open={lid_open_position}",
    )

    with ctx.pose({panel_hinge: 0.0}):
        ctx.expect_contact(
            access_panel,
            body,
            elem_a="panel_shell",
            elem_b="back_wall",
            name="rear access panel seats against the back face",
        )
        ctx.expect_contact(
            access_panel,
            access_panel,
            elem_a="louver_lower",
            elem_b="panel_shell",
            name="lower louver is attached to the service panel skin",
        )
        ctx.expect_contact(
            access_panel,
            access_panel,
            elem_a="pull",
            elem_b="panel_shell",
            name="service pull handle is mounted on the panel skin",
        )
        panel_closed_aabb = ctx.part_element_world_aabb(access_panel, elem="panel_shell")

    with ctx.pose({panel_hinge: panel_hinge.motion_limits.upper}):
        panel_open_aabb = ctx.part_element_world_aabb(access_panel, elem="panel_shell")

    ctx.check(
        "rear access panel swings outward from its right hinge",
        panel_closed_aabb is not None
        and panel_open_aabb is not None
        and panel_open_aabb[0][1] < panel_closed_aabb[0][1] - 0.10,
        details=f"closed={panel_closed_aabb}, open={panel_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
