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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _y_section(
    width: float,
    height: float,
    radius: float,
    y: float,
    *,
    z_base: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_base + height * 0.5)
        for x, z in rounded_rect_profile(
            width,
            height,
            radius,
            corner_segments=8,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="creator_laptop_lift_hinge")

    aluminum = model.material("aluminum", rgba=(0.68, 0.69, 0.72, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.38, 0.39, 0.42, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    near_black = model.material("near_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    display_glass = model.material("display_glass", rgba=(0.10, 0.14, 0.17, 0.82))
    key_finish = model.material("key_finish", rgba=(0.17, 0.18, 0.19, 1.0))

    base_width = 0.356
    base_depth = 0.238
    lid_width = 0.344
    lid_depth = 0.224

    lower_chassis = model.part("lower_chassis")
    lower_chassis.visual(
        Box((base_width, base_depth, 0.0088)),
        origin=Origin(xyz=(0.0, 0.0, 0.0044)),
        material=aluminum,
        name="chassis_shell",
    )
    lower_chassis.visual(
        Box((base_width, 0.034, 0.0048)),
        origin=Origin(xyz=(0.0, 0.102, 0.0108)),
        material=aluminum,
        name="rear_riser",
    )
    lower_chassis.visual(
        Box((0.322, 0.186, 0.0022)),
        origin=Origin(xyz=(0.0, -0.010, 0.0096)),
        material=graphite,
        name="keyboard_bed",
    )
    lower_chassis.visual(
        Box((0.128, 0.076, 0.0008)),
        origin=Origin(xyz=(0.0, -0.080, 0.0107)),
        material=dark_aluminum,
        name="touchpad",
    )
    lower_chassis.visual(
        Box((0.032, 0.104, 0.0008)),
        origin=Origin(xyz=(-0.146, -0.004, 0.0107)),
        material=dark_aluminum,
        name="left_speaker_grille",
    )
    lower_chassis.visual(
        Box((0.032, 0.104, 0.0008)),
        origin=Origin(xyz=(0.146, -0.004, 0.0107)),
        material=dark_aluminum,
        name="right_speaker_grille",
    )
    lower_chassis.visual(
        Box((0.300, 0.010, 0.0030)),
        origin=Origin(xyz=(0.0, 0.124, 0.0132)),
        material=dark_aluminum,
        name="rear_hinge_bridge",
    )
    lower_chassis.visual(
        Cylinder(radius=0.0042, length=0.014),
        origin=Origin(xyz=(-0.146, 0.124, 0.0132), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_aluminum,
        name="left_hinge_cap",
    )
    lower_chassis.visual(
        Cylinder(radius=0.0042, length=0.014),
        origin=Origin(xyz=(0.146, 0.124, 0.0132), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_aluminum,
        name="right_hinge_cap",
    )
    lower_chassis.visual(
        Box((0.018, 0.008, 0.0012)),
        origin=Origin(xyz=(-0.115, -0.092, 0.0113)),
        material=rubber,
        name="lid_stop_left",
    )
    lower_chassis.visual(
        Box((0.018, 0.008, 0.0012)),
        origin=Origin(xyz=(0.115, -0.092, 0.0113)),
        material=rubber,
        name="lid_stop_right",
    )
    lower_chassis.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, 0.014)),
        mass=1.75,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    display_lid = model.part("display_lid")
    display_lid.visual(
        Box((lid_width, lid_depth, 0.0050)),
        origin=Origin(xyz=(0.0, -0.108, -0.0013)),
        material=aluminum,
        name="outer_shell",
    )
    display_lid.visual(
        Box((0.336, 0.216, 0.0014)),
        origin=Origin(xyz=(0.0, -0.108, -0.0019)),
        material=near_black,
        name="bezel_panel",
    )
    display_lid.visual(
        Box((0.308, 0.188, 0.0008)),
        origin=Origin(xyz=(0.0, -0.108, -0.0013)),
        material=display_glass,
        name="display_glass",
    )
    display_lid.visual(
        Box((0.222, 0.016, 0.0030)),
        origin=Origin(xyz=(0.0, 0.010, -0.0010)),
        material=graphite,
        name="rear_heel",
    )
    display_lid.visual(
        Cylinder(radius=0.0046, length=0.018),
        origin=Origin(xyz=(-0.124, 0.003, -0.0010), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_aluminum,
        name="left_knuckle",
    )
    display_lid.visual(
        Cylinder(radius=0.0046, length=0.018),
        origin=Origin(xyz=(0.124, 0.003, -0.0010), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_aluminum,
        name="right_knuckle",
    )
    display_lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.007)),
        mass=0.95,
        origin=Origin(xyz=(0.0, -0.111, 0.0)),
    )

    rear_support_linkage = model.part("rear_support_linkage")
    rear_support_linkage.visual(
        Cylinder(radius=0.0024, length=0.246),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=graphite,
        name="link_shaft",
    )
    rear_support_linkage.visual(
        Box((0.016, 0.010, 0.008)),
        origin=Origin(xyz=(-0.060, -0.001, 0.0008)),
        material=graphite,
        name="left_link_arm",
    )
    rear_support_linkage.visual(
        Box((0.016, 0.010, 0.008)),
        origin=Origin(xyz=(0.060, -0.001, 0.0008)),
        material=graphite,
        name="right_link_arm",
    )
    rear_support_linkage.visual(
        Box((0.126, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.000, -0.0012)),
        material=graphite,
        name="center_spine",
    )
    rear_support_linkage.visual(
        Box((0.010, 0.008, 0.0040)),
        origin=Origin(xyz=(-0.060, 0.005, 0.0010)),
        material=graphite,
        name="clip_tab_left",
    )
    rear_support_linkage.visual(
        Box((0.010, 0.008, 0.0040)),
        origin=Origin(xyz=(0.060, 0.005, 0.0010)),
        material=graphite,
        name="clip_tab_right",
    )
    rear_support_linkage.inertial = Inertial.from_geometry(
        Box((0.246, 0.016, 0.010)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.003, 0.002)),
    )

    rear_support_lip = model.part("rear_support_lip")
    rear_support_lip.visual(
        Box((0.228, 0.007, 0.0016)),
        origin=Origin(xyz=(0.0, -0.0060, -0.0032)),
        material=rubber,
        name="support_pad",
    )
    rear_support_lip.visual(
        Box((0.236, 0.014, 0.0040)),
        origin=Origin(xyz=(0.0, 0.0010, -0.0012)),
        material=graphite,
        name="rear_beam",
    )
    rear_support_lip.visual(
        Box((0.014, 0.008, 0.0040)),
        origin=Origin(xyz=(-0.060, 0.011, 0.0010)),
        material=graphite,
        name="clip_body_left",
    )
    rear_support_lip.visual(
        Box((0.014, 0.008, 0.0040)),
        origin=Origin(xyz=(0.060, 0.011, 0.0010)),
        material=graphite,
        name="clip_body_right",
    )
    rear_support_lip.inertial = Inertial.from_geometry(
        Box((0.236, 0.016, 0.010)),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.006, 0.0005)),
    )

    keyboard_rows = (0.026, 0.006, -0.014, -0.034)
    keyboard_cols = tuple(-0.104 + 0.026 * index for index in range(9))
    key_origin_z = 0.0052
    key_travel = 0.0014
    key_width = 0.0215
    key_depth = 0.0150

    for row_index, key_y in enumerate(keyboard_rows):
        for col_index, key_x in enumerate(keyboard_cols):
            key_name = f"key_r{row_index}_c{col_index}"
            key_part = model.part(key_name)
            key_part.visual(
                Box((key_width * 0.52, 0.0085, 0.0048)),
                origin=Origin(xyz=(0.0, 0.0, 0.0024)),
                material=near_black,
                name="stem",
            )
            key_part.visual(
                Box((key_width, key_depth, 0.0018)),
                origin=Origin(xyz=(0.0, 0.0, 0.0057)),
                material=key_finish,
                name="cap",
            )
            key_part.inertial = Inertial.from_geometry(
                Box((key_width, key_depth, 0.0066)),
                mass=0.0035,
                origin=Origin(xyz=(0.0, 0.0, 0.0033)),
            )
            model.articulation(
                f"lower_chassis_to_{key_name}",
                ArticulationType.PRISMATIC,
                parent=lower_chassis,
                child=key_part,
                origin=Origin(xyz=(key_x, key_y, key_origin_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=1.2,
                    velocity=0.03,
                    lower=0.0,
                    upper=key_travel,
                ),
            )

    spacebar = model.part("spacebar")
    spacebar.visual(
        Box((0.070, 0.0085, 0.0048)),
        origin=Origin(xyz=(0.0, 0.0, 0.0024)),
        material=near_black,
        name="stem",
    )
    spacebar.visual(
        Box((0.112, key_depth, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0057)),
        material=key_finish,
        name="cap",
    )
    spacebar.inertial = Inertial.from_geometry(
        Box((0.112, key_depth, 0.0066)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.0033)),
    )
    model.articulation(
        "lower_chassis_to_spacebar",
        ArticulationType.PRISMATIC,
        parent=lower_chassis,
        child=spacebar,
        origin=Origin(xyz=(0.0, -0.060, key_origin_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=0.03,
            lower=0.0,
            upper=key_travel,
        ),
    )

    model.articulation(
        "base_to_display_lid",
        ArticulationType.REVOLUTE,
        parent=lower_chassis,
        child=display_lid,
        origin=Origin(xyz=(0.0, 0.124, 0.0156)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=2.20,
        ),
    )
    model.articulation(
        "base_to_rear_support_linkage",
        ArticulationType.REVOLUTE,
        parent=lower_chassis,
        child=rear_support_linkage,
        origin=Origin(xyz=(0.0, 0.129, 0.0090)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "rear_support_linkage_to_lip",
        ArticulationType.FIXED,
        parent=rear_support_linkage,
        child=rear_support_lip,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_chassis = object_model.get_part("lower_chassis")
    display_lid = object_model.get_part("display_lid")
    rear_support_linkage = object_model.get_part("rear_support_linkage")
    rear_support_lip = object_model.get_part("rear_support_lip")
    hero_key = object_model.get_part("key_r1_c4")
    spacebar = object_model.get_part("spacebar")

    lid_hinge = object_model.get_articulation("base_to_display_lid")
    support_hinge = object_model.get_articulation("base_to_rear_support_linkage")
    hero_key_joint = object_model.get_articulation("lower_chassis_to_key_r1_c4")
    spacebar_joint = object_model.get_articulation("lower_chassis_to_spacebar")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "lid hinge axis",
        tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        f"expected lid hinge axis (-1, 0, 0), got {lid_hinge.axis}",
    )
    ctx.check(
        "support hinge axis",
        tuple(support_hinge.axis) == (-1.0, 0.0, 0.0),
        f"expected support hinge axis (-1, 0, 0), got {support_hinge.axis}",
    )
    ctx.check(
        "key plunge axis",
        tuple(hero_key_joint.axis) == (0.0, 0.0, -1.0),
        f"expected key plunge axis (0, 0, -1), got {hero_key_joint.axis}",
    )

    ctx.expect_contact(
        display_lid,
        lower_chassis,
        elem_a="outer_shell",
        elem_b="lid_stop_left",
    )
    ctx.expect_contact(
        display_lid,
        lower_chassis,
        elem_a="outer_shell",
        elem_b="lid_stop_right",
    )
    ctx.expect_contact(
        rear_support_lip,
        rear_support_linkage,
        elem_a="clip_body_left",
        elem_b="clip_tab_left",
    )
    ctx.expect_contact(
        rear_support_lip,
        rear_support_linkage,
        elem_a="clip_body_right",
        elem_b="clip_tab_right",
    )
    ctx.expect_overlap(hero_key, lower_chassis, axes="xy", min_overlap=0.012)
    ctx.expect_overlap(spacebar, lower_chassis, axes="xy", min_overlap=0.012)

    lid_rest = ctx.part_world_aabb(display_lid)
    support_rest = ctx.part_world_aabb(rear_support_lip)
    hero_key_rest = ctx.part_world_position(hero_key)
    spacebar_rest = ctx.part_world_position(spacebar)
    assert lid_rest is not None
    assert support_rest is not None
    assert hero_key_rest is not None
    assert spacebar_rest is not None

    with ctx.pose({lid_hinge: 2.0, support_hinge: 0.18}):
        lid_open = ctx.part_world_aabb(display_lid)
        support_open = ctx.part_world_aabb(rear_support_lip)
        assert lid_open is not None
        assert support_open is not None
        assert lid_open[1][2] > lid_rest[1][2] + 0.15
        assert (
            abs(support_open[0][2] - support_rest[0][2]) > 0.0005
            or abs(support_open[0][1] - support_rest[0][1]) > 0.0005
        )
        ctx.expect_contact(
            display_lid,
            rear_support_lip,
            elem_a="rear_heel",
            elem_b="support_pad",
        )

    with ctx.pose({hero_key_joint: 0.0012, spacebar_joint: 0.0012}):
        hero_key_pressed = ctx.part_world_position(hero_key)
        spacebar_pressed = ctx.part_world_position(spacebar)
        assert hero_key_pressed is not None
        assert spacebar_pressed is not None
        assert hero_key_pressed[2] < hero_key_rest[2] - 0.001
        assert spacebar_pressed[2] < spacebar_rest[2] - 0.001

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
