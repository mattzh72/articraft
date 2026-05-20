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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_laptop")

    body_dark = model.material("body_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    body_shadow = model.material("body_shadow", rgba=(0.10, 0.11, 0.12, 1.0))
    deck_black = model.material("deck_black", rgba=(0.09, 0.10, 0.11, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.08, 0.12, 0.15, 0.88))
    trim_grey = model.material("trim_grey", rgba=(0.24, 0.25, 0.28, 1.0))
    key_grey = model.material("key_grey", rgba=(0.20, 0.21, 0.23, 1.0))
    key_accent = model.material("key_accent", rgba=(0.31, 0.33, 0.36, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.05, 1.0))

    keycap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.018, 0.018, 0.0024, corner_segments=8),
            0.0022,
        ),
        "gaming_laptop_keycap",
    )

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.372, 0.256, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
        material=body_dark,
        name="chassis_shell",
    )
    chassis.visual(
        Box((0.372, 0.078, 0.041)),
        origin=Origin(xyz=(0.000, 0.089, 0.0205)),
        material=body_dark,
        name="rear_vent_housing",
    )
    chassis.visual(
        Box((0.344, 0.094, 0.007)),
        origin=Origin(xyz=(0.000, 0.044, 0.0135)),
        material=body_dark,
        name="mid_deck_rise",
    )
    chassis.visual(
        Box((0.302, 0.112, 0.0016)),
        origin=Origin(xyz=(0.000, -0.004, 0.0158)),
        material=deck_black,
        name="keyboard_well",
    )
    chassis.visual(
        Box((0.334, 0.062, 0.0018)),
        origin=Origin(xyz=(0.000, -0.086, 0.0152)),
        material=deck_black,
        name="palmrest_panel",
    )
    chassis.visual(
        Box((0.112, 0.074, 0.0009)),
        origin=Origin(xyz=(0.000, -0.088, 0.0159)),
        material=body_shadow,
        name="touchpad",
    )
    chassis.visual(
        Box((0.022, 0.110, 0.0012)),
        origin=Origin(xyz=(-0.145, -0.004, 0.0156)),
        material=body_shadow,
        name="left_speaker_strip",
    )
    chassis.visual(
        Box((0.022, 0.110, 0.0012)),
        origin=Origin(xyz=(0.145, -0.004, 0.0156)),
        material=body_shadow,
        name="right_speaker_strip",
    )
    chassis.visual(
        Box((0.324, 0.018, 0.004)),
        origin=Origin(xyz=(0.000, 0.101, 0.0350)),
        material=body_shadow,
        name="rear_exhaust_slot",
    )
    chassis.visual(
        Box((0.072, 0.016, 0.002)),
        origin=Origin(xyz=(-0.102, 0.110, 0.0395)),
        material=trim_grey,
        name="left_hinge_seat",
    )
    chassis.visual(
        Box((0.072, 0.016, 0.002)),
        origin=Origin(xyz=(0.102, 0.110, 0.0395)),
        material=trim_grey,
        name="right_hinge_seat",
    )
    chassis.visual(
        Cylinder(radius=0.005, length=0.304),
        origin=Origin(xyz=(0.000, 0.126, 0.0390), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_grey,
        name="hinge_spine",
    )
    chassis.visual(
        Box((0.330, 0.008, 0.002)),
        origin=Origin(xyz=(0.000, 0.087, 0.0335)),
        material=trim_grey,
        name="top_vent_grille",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        hinge_x = side_sign * 0.151
        chassis.visual(
            Box((0.005, 0.014, 0.012)),
            origin=Origin(xyz=(hinge_x - side_sign * 0.0135, 0.086, -0.0005)),
            material=body_shadow,
            name=f"{side_name}_foot_inner_clip",
        )
        chassis.visual(
            Box((0.005, 0.014, 0.012)),
            origin=Origin(xyz=(hinge_x + side_sign * 0.0135, 0.086, -0.0005)),
            material=body_shadow,
            name=f"{side_name}_foot_outer_clip",
        )
        chassis.visual(
            Box((0.034, 0.016, 0.003)),
            origin=Origin(xyz=(hinge_x, 0.086, -0.0015)),
            material=body_shadow,
            name=f"{side_name}_foot_mount_pad",
        )
    chassis.visual(
        Box((0.034, 0.018, 0.003)),
        origin=Origin(xyz=(-0.145, -0.105, 0.0015)),
        material=rubber,
        name="front_left_rubber_foot",
    )
    chassis.visual(
        Box((0.034, 0.018, 0.003)),
        origin=Origin(xyz=(0.145, -0.105, 0.0015)),
        material=rubber,
        name="front_right_rubber_foot",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.372, 0.256, 0.042)),
        mass=2.6,
        origin=Origin(xyz=(0.000, 0.000, 0.021)),
    )

    display_lid = model.part("display_lid")
    display_lid.visual(
        Box((0.358, 0.248, 0.008)),
        origin=Origin(xyz=(0.000, -0.124, 0.004)),
        material=body_dark,
        name="lid_shell",
    )
    display_lid.visual(
        Box((0.334, 0.206, 0.0012)),
        origin=Origin(xyz=(0.000, -0.132, 0.0010)),
        material=screen_glass,
        name="display_panel",
    )
    display_lid.visual(
        Box((0.010, 0.214, 0.0026)),
        origin=Origin(xyz=(-0.165, -0.132, 0.0018)),
        material=trim_grey,
        name="left_bezel",
    )
    display_lid.visual(
        Box((0.010, 0.214, 0.0026)),
        origin=Origin(xyz=(0.165, -0.132, 0.0018)),
        material=trim_grey,
        name="right_bezel",
    )
    display_lid.visual(
        Box((0.332, 0.011, 0.0026)),
        origin=Origin(xyz=(0.000, -0.236, 0.0018)),
        material=trim_grey,
        name="top_bezel",
    )
    display_lid.visual(
        Box((0.332, 0.021, 0.0026)),
        origin=Origin(xyz=(0.000, -0.028, 0.0018)),
        material=trim_grey,
        name="bottom_bezel",
    )
    display_lid.visual(
        Cylinder(radius=0.0017, length=0.004),
        origin=Origin(xyz=(0.000, -0.236, 0.0020)),
        material=body_shadow,
        name="webcam",
    )
    display_lid.inertial = Inertial.from_geometry(
        Box((0.358, 0.252, 0.009)),
        mass=1.1,
        origin=Origin(xyz=(0.000, -0.131, 0.0045)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=display_lid,
        origin=Origin(xyz=(0.000, 0.118, 0.0405)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )

    def add_key(
        part_name: str,
        joint_name: str,
        center_x: float,
        center_y: float,
        *,
        accented: bool = False,
    ) -> None:
        key_part = model.part(part_name)
        key_part.visual(
            keycap_mesh,
            material=key_accent if accented else key_grey,
            name="keycap",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((0.018, 0.018, 0.0022)),
            mass=0.004,
            origin=Origin(xyz=(0.000, 0.000, 0.0011)),
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=key_part,
            origin=Origin(xyz=(center_x, center_y, 0.0166)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=0.03,
                lower=0.0,
                upper=0.0008,
            ),
        )

    key_rows = [
        (0.024, ("key_q", "key_w", "key_e", "key_r", "key_t")),
        (0.000, ("key_a", "key_s", "key_d", "key_f", "key_g")),
        (-0.024, ("key_z", "key_x", "key_c", "key_v", "key_b")),
    ]
    for row_y, names in key_rows:
        for column_index, key_name in enumerate(names):
            add_key(
                key_name,
                f"{key_name}_switch",
                -0.070 + 0.022 * column_index,
                row_y,
                accented=key_name in {"key_w", "key_a", "key_s", "key_d"},
            )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        rear_foot = model.part(f"{side_name}_rear_foot")
        rear_foot.visual(
            Cylinder(radius=0.0045, length=0.022),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_grey,
            name="hinge_barrel",
        )
        rear_foot.visual(
            Box((0.018, 0.036, 0.004)),
            origin=Origin(xyz=(0.000, 0.018, -0.006)),
            material=body_dark,
            name="support_arm",
        )
        rear_foot.visual(
            Box((0.018, 0.010, 0.008)),
            origin=Origin(xyz=(0.000, 0.037, -0.010)),
            material=rubber,
            name="foot_pad",
        )
        rear_foot.inertial = Inertial.from_geometry(
            Box((0.022, 0.046, 0.018)),
            mass=0.05,
            origin=Origin(xyz=(0.000, 0.020, -0.007)),
        )
        model.articulation(
            f"{side_name}_rear_foot_hinge",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=rear_foot,
            origin=Origin(xyz=(side_sign * 0.151, 0.086, -0.0005)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.6,
                lower=0.0,
                upper=1.12,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    expected_parts = {
        "chassis",
        "display_lid",
        "left_rear_foot",
        "right_rear_foot",
        "key_w",
        "key_a",
        "key_s",
        "key_d",
    }
    expected_joints = {
        "lid_hinge",
        "left_rear_foot_hinge",
        "right_rear_foot_hinge",
        "key_w_switch",
        "key_a_switch",
        "key_s_switch",
        "key_d_switch",
    }
    actual_parts = {part.name for part in object_model.parts}
    actual_joints = {joint.name for joint in object_model.articulations}
    ctx.check(
        "expected_parts_present",
        expected_parts.issubset(actual_parts),
        details=f"Missing parts: {sorted(expected_parts - actual_parts)}",
    )
    ctx.check(
        "expected_joints_present",
        expected_joints.issubset(actual_joints),
        details=f"Missing joints: {sorted(expected_joints - actual_joints)}",
    )

    chassis = object_model.get_part("chassis")
    display_lid = object_model.get_part("display_lid")
    left_rear_foot = object_model.get_part("left_rear_foot")
    right_rear_foot = object_model.get_part("right_rear_foot")
    key_w = object_model.get_part("key_w")
    key_a = object_model.get_part("key_a")
    key_s = object_model.get_part("key_s")
    key_d = object_model.get_part("key_d")

    lid_hinge = object_model.get_articulation("lid_hinge")
    left_rear_foot_hinge = object_model.get_articulation("left_rear_foot_hinge")
    right_rear_foot_hinge = object_model.get_articulation("right_rear_foot_hinge")
    key_w_switch = object_model.get_articulation("key_w_switch")

    ctx.expect_overlap(display_lid, chassis, axes="xy", min_overlap=0.20)
    for key_part_name in ("key_w", "key_a", "key_s", "key_d"):
        ctx.expect_gap(
            key_part_name,
            chassis,
            axis="z",
            min_gap=0.0,
            max_gap=0.0008,
            positive_elem="keycap",
            negative_elem="keyboard_well",
            name=f"{key_part_name}_rests_just_above_key_well",
        )

    ctx.expect_contact(
        left_rear_foot,
        chassis,
        elem_a="hinge_barrel",
        elem_b="left_foot_inner_clip",
        name="left_foot_barrel_captured_in_mount",
    )
    ctx.expect_contact(
        right_rear_foot,
        chassis,
        elem_a="hinge_barrel",
        elem_b="right_foot_inner_clip",
        name="right_foot_barrel_captured_in_mount",
    )

    lid_closed_aabb = ctx.part_world_aabb(display_lid)
    key_w_rest = ctx.part_world_position(key_w)
    left_foot_rest_aabb = ctx.part_world_aabb(left_rear_foot)
    assert lid_closed_aabb is not None
    assert key_w_rest is not None
    assert left_foot_rest_aabb is not None

    with ctx.pose({lid_hinge: 1.25}):
        lid_open_aabb = ctx.part_world_aabb(display_lid)
        assert lid_open_aabb is not None
        ctx.check(
            "lid_opens_upward",
            lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.18,
            details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
        )

    with ctx.pose({key_w_switch: 0.0008}):
        key_w_pressed = ctx.part_world_position(key_w)
        assert key_w_pressed is not None
        ctx.check(
            "key_w_moves_downward",
            key_w_pressed[2] < key_w_rest[2] - 0.0006,
            details=f"rest={key_w_rest}, pressed={key_w_pressed}",
        )
        ctx.expect_contact(
            key_w,
            chassis,
            elem_a="keycap",
            elem_b="keyboard_well",
            name="key_w_bottoms_out_without_detaching",
        )

    with ctx.pose({left_rear_foot_hinge: 1.05, right_rear_foot_hinge: 1.05}):
        left_foot_open_aabb = ctx.part_world_aabb(left_rear_foot)
        right_foot_open_aabb = ctx.part_world_aabb(right_rear_foot)
        assert left_foot_open_aabb is not None
        assert right_foot_open_aabb is not None
        ctx.check(
            "rear_feet_rotate_downward",
            left_foot_open_aabb[0][2] < left_foot_rest_aabb[0][2] - 0.018
            and right_foot_open_aabb[0][2] < left_foot_rest_aabb[0][2] - 0.018,
            details=f"rest={left_foot_rest_aabb}, open_left={left_foot_open_aabb}, open_right={right_foot_open_aabb}",
        )
        ctx.expect_contact(
            left_rear_foot,
            chassis,
            elem_a="hinge_barrel",
            elem_b="left_foot_inner_clip",
            name="left_foot_remains_captured_when_open",
        )
        ctx.expect_contact(
            right_rear_foot,
            chassis,
            elem_a="hinge_barrel",
            elem_b="right_foot_inner_clip",
            name="right_foot_remains_captured_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
