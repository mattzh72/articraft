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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_laptop")

    aluminum = model.material("aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    key_black = model.material("key_black", rgba=(0.09, 0.10, 0.11, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.05, 0.05, 0.06, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.03, 0.03, 0.04, 1.0))

    base_w = 0.320
    base_d = 0.220
    base_h = 0.014
    base_floor_t = 0.0016
    base_deck_t = 0.0018
    wall_t = 0.0022
    wall_h = base_h - base_floor_t - base_deck_t
    corner_r = 0.014

    display_w = 0.316
    display_h = 0.214
    display_t = 0.008
    bezel_t = 0.0014
    back_cover_t = 0.0016
    lid_wall_t = 0.0020

    hinge_radius = 0.0036
    hinge_axis_y = base_d * 0.5 + 0.002
    hinge_axis_z = base_h + 0.0016
    hinge_leaf_y = hinge_axis_y - 0.008

    keyboard_open_w = 0.226
    keyboard_open_d = 0.120
    keyboard_open_y = 0.004
    key_top_z = 0.0116
    key_travel = 0.0016
    key_pitch = 0.019

    base_outer_profile = rounded_rect_profile(base_w, base_d, corner_r)
    base_floor_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(base_outer_profile, base_floor_t),
        "base_floor_panel",
    )
    deck_outer_profile = rounded_rect_profile(base_w - 0.004, base_d - 0.004, corner_r - 0.002)
    keyboard_hole_profile = [
        (x, y + keyboard_open_y)
        for x, y in rounded_rect_profile(keyboard_open_w, keyboard_open_d, 0.008)
    ]
    deck_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            deck_outer_profile,
            [keyboard_hole_profile],
            base_deck_t,
            center=True,
            cap=True,
            closed=True,
        ),
        "keyboard_deck_frame",
    )

    keycap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.016, 0.016, 0.003), 0.0016),
        "chiclet_keycap",
    )
    wide_keycap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.024, 0.016, 0.003), 0.0016),
        "wide_keycap",
    )
    spacebar_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.090, 0.016, 0.0035), 0.0016),
        "spacebar_keycap",
    )

    lid_back_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(display_w, display_h, 0.013),
            back_cover_t,
        ),
        "lid_back_cover",
    )
    bezel_outer_profile = rounded_rect_profile(display_w - 0.010, display_h - 0.010, 0.010)
    screen_hole_profile = rounded_rect_profile(display_w - 0.030, display_h - 0.040, 0.006)
    bezel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            bezel_outer_profile,
            [screen_hole_profile],
            bezel_t,
            center=True,
            cap=True,
            closed=True,
        ),
        "display_bezel_frame",
    )

    base = model.part("base")
    base.visual(
        base_floor_mesh,
        material=aluminum,
        name="bottom_cover",
    )
    base.visual(
        Box((base_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -base_d * 0.5 + wall_t * 0.5, base_floor_t + wall_h * 0.5)),
        material=aluminum,
        name="front_wall",
    )
    base.visual(
        Box((wall_t, base_d - 2.0 * wall_t, wall_h)),
        origin=Origin(
            xyz=(-base_w * 0.5 + wall_t * 0.5, 0.0, base_floor_t + wall_h * 0.5)
        ),
        material=aluminum,
        name="left_wall",
    )
    base.visual(
        Box((wall_t, base_d - 2.0 * wall_t, wall_h)),
        origin=Origin(
            xyz=(base_w * 0.5 - wall_t * 0.5, 0.0, base_floor_t + wall_h * 0.5)
        ),
        material=aluminum,
        name="right_wall",
    )
    base.visual(
        Box((0.118, wall_t, wall_h * 0.75)),
        origin=Origin(
            xyz=(-0.091, base_d * 0.5 - wall_t * 0.5, base_floor_t + wall_h * 0.375)
        ),
        material=aluminum,
        name="rear_wall_left",
    )
    base.visual(
        Box((0.118, wall_t, wall_h * 0.75)),
        origin=Origin(
            xyz=(0.091, base_d * 0.5 - wall_t * 0.5, base_floor_t + wall_h * 0.375)
        ),
        material=aluminum,
        name="rear_wall_right",
    )
    base.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, base_h - base_deck_t * 0.5)),
        material=aluminum,
        name="deck_plate",
    )
    base.visual(
        Box((keyboard_open_w + 0.010, keyboard_open_d + 0.008, 0.0012)),
        origin=Origin(xyz=(0.0, keyboard_open_y, 0.0116)),
        material=graphite,
        name="keyboard_well",
    )
    base.visual(
        Box((0.110, 0.068, 0.0008)),
        origin=Origin(xyz=(0.0, -0.074, base_h - 0.0004)),
        material=glass_dark,
        name="touchpad",
    )
    base.visual(
        Box((0.060, 0.004, 0.0005)),
        origin=Origin(xyz=(-0.102, -0.074, base_h - 0.00025)),
        material=sensor_black,
        name="left_speaker_slot",
    )
    base.visual(
        Box((0.060, 0.004, 0.0005)),
        origin=Origin(xyz=(0.102, -0.074, base_h - 0.00025)),
        material=sensor_black,
        name="right_speaker_slot",
    )
    base.visual(
        Box((0.030, 0.012, 0.0030)),
        origin=Origin(xyz=(-0.110, hinge_leaf_y + 0.001, hinge_axis_z - 0.0031)),
        material=hinge_dark,
        name="left_hinge_mount",
    )
    base.visual(
        Box((0.030, 0.012, 0.0030)),
        origin=Origin(xyz=(0.110, hinge_leaf_y + 0.001, hinge_axis_z - 0.0031)),
        material=hinge_dark,
        name="right_hinge_mount",
    )

    for side, side_sign in (("left", -1.0), ("right", 1.0)):
        x_center = side_sign * 0.110
        base.visual(
            Cylinder(radius=hinge_radius, length=0.016),
            origin=Origin(
                xyz=(x_center - side_sign * 0.018, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_dark,
            name=f"{side}_base_barrel_outer",
        )
        base.visual(
            Cylinder(radius=hinge_radius, length=0.016),
            origin=Origin(
                xyz=(x_center + side_sign * 0.018, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_dark,
            name=f"{side}_base_barrel_inner",
        )

    base.inertial = Inertial.from_geometry(
        Box((base_w, base_d, base_h)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, base_h * 0.5)),
    )

    display = model.part("display")
    display.visual(
        lid_back_mesh,
        origin=Origin(xyz=(0.0, -display_h * 0.5, display_t - back_cover_t)),
        material=aluminum,
        name="back_cover",
    )
    display.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, -display_h * 0.5, bezel_t * 0.5)),
        material=bezel_black,
        name="bezel_frame",
    )
    display.visual(
        Box((display_w - 0.028, display_h - 0.038, 0.0010)),
        origin=Origin(xyz=(0.0, -display_h * 0.5, 0.0011)),
        material=glass_dark,
        name="screen_panel",
    )
    display.visual(
        Box((lid_wall_t, display_h - 0.014, display_t - bezel_t - back_cover_t)),
        origin=Origin(
            xyz=(
                -display_w * 0.5 + lid_wall_t * 0.5,
                -display_h * 0.5,
                0.0014 + (display_t - bezel_t - back_cover_t) * 0.5,
            )
        ),
        material=aluminum,
        name="left_lid_wall",
    )
    display.visual(
        Box((lid_wall_t, display_h - 0.014, display_t - bezel_t - back_cover_t)),
        origin=Origin(
            xyz=(
                display_w * 0.5 - lid_wall_t * 0.5,
                -display_h * 0.5,
                0.0014 + (display_t - bezel_t - back_cover_t) * 0.5,
            )
        ),
        material=aluminum,
        name="right_lid_wall",
    )
    display.visual(
        Box((display_w - 0.014, lid_wall_t, display_t - bezel_t - back_cover_t)),
        origin=Origin(
            xyz=(
                0.0,
                -display_h + lid_wall_t * 0.5,
                0.0014 + (display_t - bezel_t - back_cover_t) * 0.5,
            )
        ),
        material=aluminum,
        name="top_lid_wall",
    )
    display.visual(
        Box((0.052, 0.004, 0.0014)),
        origin=Origin(xyz=(0.0, -display_h + 0.004, 0.0009)),
        material=sensor_black,
        name="camera_bar",
    )
    display.visual(
        Cylinder(radius=0.0022, length=0.004),
        origin=Origin(
            xyz=(0.0, -display_h + 0.004, 0.0007),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=sensor_black,
        name="webcam_lens",
    )
    display.visual(
        Box((0.028, 0.006, 0.0022)),
        origin=Origin(xyz=(-0.110, -0.003, 0.0011)),
        material=hinge_dark,
        name="left_hinge_leaf",
    )
    display.visual(
        Box((0.028, 0.006, 0.0022)),
        origin=Origin(xyz=(0.110, -0.003, 0.0011)),
        material=hinge_dark,
        name="right_hinge_leaf",
    )

    for side, side_sign in (("left", -1.0), ("right", 1.0)):
        x_center = side_sign * 0.110
        display.visual(
            Cylinder(radius=hinge_radius * 0.94, length=0.016),
            origin=Origin(
                xyz=(x_center, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_dark,
            name=f"{side}_display_barrel_center",
        )

    display.inertial = Inertial.from_geometry(
        Box((display_w, display_t, display_h)),
        mass=0.88,
        origin=Origin(xyz=(0.0, -display_h * 0.5, display_t * 0.5)),
    )

    model.articulation(
        "base_to_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=display,
        origin=Origin(
            xyz=(0.0, hinge_axis_y, hinge_axis_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.2,
            lower=-1.60,
            upper=1.60,
        ),
    )

    def add_key(
        name: str,
        *,
        x: float,
        y: float,
        mesh,
        size: tuple[float, float, float],
        mass: float,
    ) -> None:
        key = model.part(name)
        key.visual(
            Box((0.0045, 0.0045, 0.0010)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=graphite,
            name="key_stem",
        )
        key.visual(mesh, material=key_black, name="keycap")
        key.inertial = Inertial.from_geometry(
            Box(size),
            mass=mass,
            origin=Origin(xyz=(0.0, 0.0, size[2] * 0.5)),
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(x, y, key_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=0.03,
                lower=0.0,
                upper=key_travel,
            ),
        )

    for row_index, y in enumerate((0.043, 0.021, -0.001, -0.023)):
        column_count = 11 if row_index == 0 else 10
        x_start = -0.095 if row_index == 0 else -0.0855
        for column_index in range(column_count):
            add_key(
                f"key_r{row_index}_c{column_index}",
                x=x_start + column_index * key_pitch,
                y=y,
                mesh=keycap_mesh,
                size=(0.016, 0.016, 0.0016),
                mass=0.0045,
            )

    for index, x in enumerate((-0.103, -0.076, 0.076, 0.103)):
        add_key(
            f"modifier_key_{index}",
            x=x,
            y=-0.045,
            mesh=wide_keycap_mesh,
            size=(0.024, 0.016, 0.0016),
            mass=0.0050,
        )

    add_key(
        "spacebar",
        x=0.0,
        y=-0.045,
        mesh=spacebar_mesh,
        size=(0.090, 0.016, 0.0016),
        mass=0.010,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    display = object_model.get_part("display")
    spacebar = object_model.get_part("spacebar")
    sample_key = object_model.get_part("key_r1_c4")
    hinge = object_model.get_articulation("base_to_display")
    sample_key_joint = object_model.get_articulation("base_to_key_r1_c4")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "display_hinge_axis_is_lateral",
        tuple(hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "display_hinge_range_reaches_closed_and_folded",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower is not None
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.lower <= -1.55
        and hinge.motion_limits.upper >= 1.55,
        details=f"limits={hinge.motion_limits}",
    )
    ctx.check(
        "key_joint_is_vertical_prismatic",
        tuple(sample_key_joint.axis) == (0.0, 0.0, -1.0)
        and sample_key_joint.joint_type == ArticulationType.PRISMATIC,
        details=f"type={sample_key_joint.joint_type}, axis={sample_key_joint.axis}",
    )

    ctx.expect_within(spacebar, base, axes="x")
    ctx.expect_within(sample_key, base, axes="x")

    with ctx.pose({hinge: -1.57}):
        ctx.expect_gap(
            display,
            base,
            axis="z",
            min_gap=0.001,
            max_gap=0.006,
            positive_elem="screen_panel",
            negative_elem="deck_plate",
            name="closed_display_clears_keyboard_deck",
        )
        ctx.expect_overlap(
            display,
            base,
            axes="xy",
            min_overlap=0.18,
            elem_a="back_cover",
            elem_b="deck_plate",
            name="closed_display_covers_base_footprint",
        )

    with ctx.pose({hinge: 1.57}):
        ctx.expect_gap(
            display,
            base,
            axis="y",
            min_gap=0.001,
            positive_elem="back_cover",
            negative_elem="rear_wall_left",
            name="folded_display_moves_behind_base",
        )

    key_rest = ctx.part_world_position(sample_key)
    assert key_rest is not None
    with ctx.pose({sample_key_joint: 0.0016}):
        key_pressed = ctx.part_world_position(sample_key)
        assert key_pressed is not None
        ctx.check(
            "key_press_moves_downward",
            key_pressed[2] < key_rest[2] - 0.001,
            details=f"rest={key_rest}, pressed={key_pressed}",
        )
        ctx.expect_within(sample_key, base, axes="xy", margin=0.0)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            display,
            base,
            axis="z",
            min_gap=0.020,
            positive_elem="screen_panel",
            negative_elem="deck_plate",
            name="open_screen_rises_above_keyboard",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
