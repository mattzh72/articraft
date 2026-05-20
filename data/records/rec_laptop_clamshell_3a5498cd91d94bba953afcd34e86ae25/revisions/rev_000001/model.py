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
    tube_from_spline_points,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_laptop")

    shell_dark = model.material("shell_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.23, 0.25, 0.28, 1.0))
    shell_inner = model.material("shell_inner", rgba=(0.10, 0.11, 0.13, 1.0))
    key_black = model.material("key_black", rgba=(0.11, 0.12, 0.14, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    grip_dark = model.material("grip_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.08, 0.12, 0.16, 1.0))
    glass_soft = model.material("glass_soft", rgba=(0.14, 0.22, 0.30, 0.45))

    lower_width = 0.314
    lower_depth = 0.224
    lid_width = 0.308
    lid_depth = 0.212
    deck_outer_width = 0.306
    deck_outer_depth = 0.212
    keyboard_open_width = 0.274
    keyboard_open_depth = 0.104
    keyboard_open_y = -0.005
    handle_slot_width = 0.204
    handle_slot_depth = 0.024
    handle_slot_y = 0.065

    lower_shell = model.part("lower_shell")

    deck_outer = rounded_rect_profile(
        deck_outer_width,
        deck_outer_depth,
        0.016,
        corner_segments=10,
    )
    keyboard_hole = _shift_profile(
        rounded_rect_profile(
            keyboard_open_width,
            keyboard_open_depth,
            0.008,
            corner_segments=8,
        ),
        dy=keyboard_open_y,
    )
    handle_slot = _shift_profile(
        rounded_rect_profile(
            handle_slot_width,
            handle_slot_depth,
            0.006,
            corner_segments=6,
        ),
        dy=handle_slot_y,
    )
    top_deck_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            deck_outer,
            [keyboard_hole, handle_slot],
            height=0.0025,
            center=True,
        ),
        "travel_laptop_top_deck",
    )

    lower_shell.visual(
        Box((lower_width, lower_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shell_dark,
        name="bottom_shell",
    )
    lower_shell.visual(
        Box((0.004, lower_depth, 0.016)),
        origin=Origin(xyz=(-0.155, 0.0, 0.010)),
        material=shell_dark,
        name="left_wall",
    )
    lower_shell.visual(
        Box((0.004, lower_depth, 0.016)),
        origin=Origin(xyz=(0.155, 0.0, 0.010)),
        material=shell_dark,
        name="right_wall",
    )
    lower_shell.visual(
        Box((0.306, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.110, 0.009)),
        material=shell_dark,
        name="front_wall",
    )
    lower_shell.visual(
        Box((lower_width, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.106, 0.010)),
        material=shell_dark,
        name="rear_wall",
    )
    lower_shell.visual(
        top_deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.01875)),
        material=shell_mid,
        name="top_deck",
    )
    lower_shell.visual(
        Box((0.306, 0.120, 0.003)),
        origin=Origin(xyz=(0.0, keyboard_open_y, 0.0104)),
        material=shell_inner,
        name="keyboard_floor",
    )
    lower_shell.visual(
        Box((0.306, 0.058, 0.010)),
        origin=Origin(xyz=(0.0, -0.079, 0.009)),
        material=shell_inner,
        name="palmrest_support",
    )
    lower_shell.visual(
        Box((0.208, 0.024, 0.003)),
        origin=Origin(xyz=(0.0, handle_slot_y, 0.0155)),
        material=shell_inner,
        name="recess_floor",
    )
    lower_shell.visual(
        Box((0.214, 0.007, 0.008)),
        origin=Origin(xyz=(0.0, 0.0515, 0.019)),
        material=shell_mid,
        name="recess_front_lip",
    )
    lower_shell.visual(
        Box((0.010, 0.022, 0.012)),
        origin=Origin(xyz=(-0.109, handle_slot_y, 0.021)),
        material=shell_mid,
        name="left_handle_pivot_tower",
    )
    lower_shell.visual(
        Box((0.010, 0.022, 0.012)),
        origin=Origin(xyz=(0.109, handle_slot_y, 0.021)),
        material=shell_mid,
        name="right_handle_pivot_tower",
    )
    lower_shell.visual(
        Box((0.050, 0.082, 0.006)),
        origin=Origin(xyz=(-0.129, 0.065, 0.020)),
        material=shell_mid,
        name="left_recess_side_support",
    )
    lower_shell.visual(
        Box((0.050, 0.082, 0.006)),
        origin=Origin(xyz=(0.129, 0.065, 0.020)),
        material=shell_mid,
        name="right_recess_side_support",
    )
    lower_shell.visual(
        Box((0.104, 0.064, 0.001)),
        origin=Origin(xyz=(0.0, -0.064, 0.0195)),
        material=glass_dark,
        name="trackpad",
    )
    lower_shell.visual(
        Box((0.090, 0.004, 0.0008)),
        origin=Origin(xyz=(0.0, -0.032, 0.0194)),
        material=shell_mid,
        name="trackpad_divider",
    )
    lower_shell.inertial = Inertial.from_geometry(
        Box((lower_width, lower_depth, 0.028)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    display_lid = model.part("display_lid")

    lid_outer_profile = rounded_rect_profile(
        lid_width,
        lid_depth,
        0.014,
        corner_segments=10,
    )
    bezel_inner_profile = rounded_rect_profile(
        0.286,
        0.176,
        0.010,
        corner_segments=8,
    )
    lid_back_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            lid_outer_profile,
            0.004,
            center=True,
        ),
        "travel_laptop_lid_back",
    )
    bezel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            lid_outer_profile,
            [bezel_inner_profile],
            height=0.003,
            center=True,
        ),
        "travel_laptop_bezel",
    )

    display_lid.visual(
        lid_back_mesh,
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, 0.002)),
        material=shell_dark,
        name="lid_back",
    )
    display_lid.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, 0.0055)),
        material=shell_mid,
        name="display_bezel",
    )
    display_lid.visual(
        Box((0.288, 0.178, 0.001)),
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, 0.0059)),
        material=glass_soft,
        name="screen_glass",
    )
    display_lid.visual(
        Box((0.020, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, -0.192, 0.0060)),
        material=glass_dark,
        name="camera_bar",
    )
    display_lid.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(-0.121, -0.003, 0.0045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_mid,
        name="left_lid_hinge_barrel",
    )
    display_lid.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(0.121, -0.003, 0.0045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_mid,
        name="right_lid_hinge_barrel",
    )
    display_lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.008)),
        mass=0.78,
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, 0.004)),
    )

    carry_handle = model.part("carry_handle")
    handle_loop_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.094, 0.000, 0.000),
                (-0.091, -0.010, 0.003),
                (-0.082, -0.020, 0.008),
                (-0.056, -0.028, 0.010),
                (0.056, -0.028, 0.010),
                (0.082, -0.020, 0.008),
                (0.091, -0.010, 0.003),
                (0.094, 0.000, 0.000),
            ],
            radius=0.0035,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
        "travel_laptop_handle_loop",
    )
    carry_handle.visual(
        handle_loop_mesh,
        material=handle_metal,
        name="handle_loop",
    )
    carry_handle.visual(
        Box((0.126, 0.010, 0.0045)),
        origin=Origin(xyz=(0.0, -0.028, 0.010)),
        material=grip_dark,
        name="grip",
    )
    carry_handle.visual(
        Cylinder(radius=0.0048, length=0.012),
        origin=Origin(xyz=(-0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="left_pivot",
    )
    carry_handle.visual(
        Cylinder(radius=0.0048, length=0.012),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="right_pivot",
    )
    carry_handle.inertial = Inertial.from_geometry(
        Box((0.210, 0.040, 0.018)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.014, 0.009)),
    )

    model.articulation(
        "lower_shell_to_display",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=display_lid,
        origin=Origin(xyz=(0.0, 0.108, 0.024)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.4,
            lower=0.0,
            upper=2.35,
        ),
    )
    model.articulation(
        "lower_shell_to_handle",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=carry_handle,
        origin=Origin(xyz=(0.0, 0.093, 0.019)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.40,
        ),
    )

    row_specs = [
        {
            "count": 10,
            "y": 0.039,
            "pitch": 0.024,
            "size": (0.018, 0.010, 0.0016),
            "offset": 0.0,
        },
        {
            "count": 11,
            "y": 0.018,
            "pitch": 0.023,
            "size": (0.0185, 0.013, 0.0017),
            "offset": 0.0,
        },
        {
            "count": 10,
            "y": -0.003,
            "pitch": 0.024,
            "size": (0.019, 0.014, 0.0018),
            "offset": -0.008,
        },
        {
            "count": 9,
            "y": -0.024,
            "pitch": 0.025,
            "size": (0.020, 0.014, 0.0018),
            "offset": -0.012,
        },
    ]

    for row_index, row in enumerate(row_specs):
        count = row["count"]
        pitch = row["pitch"]
        key_size = row["size"]
        x_offset = row["offset"]
        x_start = -pitch * (count - 1) * 0.5 + x_offset
        for column_index in range(count):
            key_name = f"key_r{row_index}_c{column_index}"
            key_part = model.part(key_name)
            key_part.visual(
                Box(key_size),
                origin=Origin(xyz=(0.0, 0.0, -key_size[2] * 0.5)),
                material=key_black,
                name="keycap",
            )
            key_part.visual(
                Box((key_size[0] * 0.24, key_size[1] * 0.24, 0.0080)),
                origin=Origin(xyz=(0.0, 0.0, -0.0049)),
                material=shell_inner,
                name="plunger",
            )
            model.articulation(
                f"lower_shell_to_{key_name}",
                ArticulationType.PRISMATIC,
                parent=lower_shell,
                child=key_part,
                origin=Origin(
                    xyz=(
                        x_start + column_index * pitch,
                        row["y"],
                        0.0208,
                    )
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=0.3,
                    velocity=0.10,
                    lower=0.0,
                    upper=0.0013,
                ),
            )

    bottom_row_parts = [
        (-0.106, 0.024),
        (-0.076, 0.024),
        (-0.044, 0.032),
        (0.000, 0.088),
        (0.044, 0.032),
        (0.076, 0.024),
        (0.106, 0.024),
    ]
    for column_index, (center_x, width) in enumerate(bottom_row_parts):
        key_name = f"key_r4_c{column_index}"
        key_part = model.part(key_name)
        key_part.visual(
            Box((width, 0.014, 0.0018)),
            origin=Origin(xyz=(0.0, 0.0, -0.0009)),
            material=key_black,
            name="keycap",
        )
        key_part.visual(
            Box((width * 0.20, 0.0038, 0.0080)),
            origin=Origin(xyz=(0.0, 0.0, -0.0049)),
            material=shell_inner,
            name="plunger",
        )
        model.articulation(
            f"lower_shell_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=lower_shell,
            child=key_part,
            origin=Origin(xyz=(center_x, -0.045, 0.0208)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=0.10,
                lower=0.0,
                upper=0.0013,
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

    lower_shell = object_model.get_part("lower_shell")
    display_lid = object_model.get_part("display_lid")
    carry_handle = object_model.get_part("carry_handle")
    lid_hinge = object_model.get_articulation("lower_shell_to_display")
    handle_hinge = object_model.get_articulation("lower_shell_to_handle")
    home_row_key = object_model.get_part("key_r2_c4")
    home_row_joint = object_model.get_articulation("lower_shell_to_key_r2_c4")
    spacebar = object_model.get_part("key_r4_c3")

    ctx.check(
        "primary parts present",
        all(part is not None for part in (lower_shell, display_lid, carry_handle, home_row_key, spacebar)),
        details="Expected lower shell, display lid, carry handle, and representative keys.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            display_lid,
            lower_shell,
            axes="xy",
            elem_a="lid_back",
            elem_b="top_deck",
            min_overlap=0.20,
            name="closed lid covers the lower shell",
        )
        ctx.expect_gap(
            display_lid,
            lower_shell,
            axis="z",
            positive_elem="lid_back",
            negative_elem="top_deck",
            min_gap=0.003,
            max_gap=0.010,
            name="closed lid sits just above the deck",
        )

    rest_screen = ctx.part_element_world_aabb(display_lid, elem="screen_glass")
    with ctx.pose({lid_hinge: 1.70}):
        open_screen = ctx.part_element_world_aabb(display_lid, elem="screen_glass")
    ctx.check(
        "display opens upward",
        rest_screen is not None
        and open_screen is not None
        and open_screen[1][2] > rest_screen[1][2] + 0.14,
        details=f"rest_screen={rest_screen}, open_screen={open_screen}",
    )

    with ctx.pose({handle_hinge: 0.0}):
        ctx.expect_overlap(
            carry_handle,
            lower_shell,
            axes="xy",
            elem_a="grip",
            elem_b="recess_floor",
            min_overlap=0.008,
            name="handle grip sits over the rear recess",
        )
        ctx.expect_gap(
            carry_handle,
            lower_shell,
            axis="z",
            positive_elem="grip",
            negative_elem="recess_floor",
            min_gap=0.008,
            max_gap=0.020,
            name="folded handle floats just above the recess floor",
        )

    rest_grip = ctx.part_element_world_aabb(carry_handle, elem="grip")
    with ctx.pose({handle_hinge: 1.30}):
        open_grip = ctx.part_element_world_aabb(carry_handle, elem="grip")
    ctx.check(
        "handle lifts clear for carrying",
        rest_grip is not None
        and open_grip is not None
        and open_grip[1][2] > rest_grip[1][2] + 0.018,
        details=f"rest_grip={rest_grip}, open_grip={open_grip}",
    )

    ctx.expect_within(
        home_row_key,
        lower_shell,
        axes="xy",
        inner_elem="keycap",
        outer_elem="keyboard_floor",
        margin=0.010,
        name="representative key stays over the keyboard well",
    )

    key_rest = ctx.part_world_position(home_row_key)
    with ctx.pose({home_row_joint: 0.0013}):
        key_pressed = ctx.part_world_position(home_row_key)
    ctx.check(
        "representative key plunges downward",
        key_rest is not None
        and key_pressed is not None
        and key_pressed[2] < key_rest[2] - 0.0010,
        details=f"key_rest={key_rest}, key_pressed={key_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
