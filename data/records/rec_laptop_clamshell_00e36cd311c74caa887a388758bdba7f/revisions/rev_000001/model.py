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


BASE_WIDTH = 0.315
BASE_DEPTH = 0.225
BASE_HEIGHT = 0.016
KEYBOARD_FLOOR_Z = 0.01075
KEYBOARD_FLOOR_HEIGHT = 0.0015
KEY_ORIGIN_Z = 0.0152
KEY_TRAVEL = 0.0015
KEYCAP_HEIGHT = 0.0014
KEY_STEM_HEIGHT = 0.0030
GUIDE_HEIGHT = 0.0019
LID_DEFAULT_ROLL = -1.45
LID_OPEN_EXTRA = -0.35
LID_CLOSE_EXTRA = 1.45
HINGE_X = 0.105
HINGE_Y = 0.112
HINGE_Z = 0.0184


def _build_key_specs() -> list[dict[str, float | str]]:
    rows = (
        {
            "y": 0.055,
            "depth": 0.011,
            "offset": 0.0,
            "keys": (
                ("key_esc", 0.018),
                ("key_1", 0.018),
                ("key_2", 0.018),
                ("key_3", 0.018),
                ("key_4", 0.018),
                ("key_5", 0.018),
                ("key_6", 0.018),
                ("key_7", 0.018),
                ("key_8", 0.018),
                ("key_9", 0.018),
                ("key_0", 0.018),
                ("key_backspace", 0.028),
            ),
        },
        {
            "y": 0.035,
            "depth": 0.014,
            "offset": 0.004,
            "keys": (
                ("key_tab", 0.024),
                ("key_q", 0.018),
                ("key_w", 0.018),
                ("key_e", 0.018),
                ("key_r", 0.018),
                ("key_t", 0.018),
                ("key_y", 0.018),
                ("key_u", 0.018),
                ("key_i", 0.018),
                ("key_o", 0.018),
                ("key_p", 0.018),
                ("key_backslash", 0.024),
            ),
        },
        {
            "y": 0.015,
            "depth": 0.014,
            "offset": 0.006,
            "keys": (
                ("key_caps", 0.028),
                ("key_a", 0.018),
                ("key_s", 0.018),
                ("key_d", 0.018),
                ("key_f", 0.018),
                ("key_g", 0.018),
                ("key_h", 0.018),
                ("key_j", 0.018),
                ("key_k", 0.018),
                ("key_l", 0.018),
                ("key_enter", 0.030),
            ),
        },
        {
            "y": -0.005,
            "depth": 0.014,
            "offset": 0.010,
            "keys": (
                ("key_left_shift", 0.034),
                ("key_z", 0.018),
                ("key_x", 0.018),
                ("key_c", 0.018),
                ("key_v", 0.018),
                ("key_b", 0.018),
                ("key_n", 0.018),
                ("key_m", 0.018),
                ("key_up", 0.018),
                ("key_right_shift", 0.034),
            ),
        },
        {
            "y": -0.025,
            "depth": 0.013,
            "offset": 0.0,
            "keys": (
                ("key_ctrl", 0.020),
                ("key_fn", 0.020),
                ("key_alt", 0.020),
                ("key_spacebar", 0.090),
                ("key_alt_gr", 0.020),
                ("key_left", 0.018),
                ("key_down", 0.018),
                ("key_right", 0.018),
            ),
        },
    )
    gap = 0.003
    specs: list[dict[str, float | str]] = []
    for row in rows:
        widths = [float(width) for _, width in row["keys"]]
        total_width = sum(widths) + gap * (len(widths) - 1)
        cursor = -total_width * 0.5 + float(row["offset"])
        for name, width in row["keys"]:
            specs.append(
                {
                    "part_name": name,
                    "joint_name": f"base_to_{name}",
                    "guide_name": f"guide_{name}",
                    "x": cursor + width * 0.5,
                    "y": float(row["y"]),
                    "width": float(width),
                    "depth": float(row["depth"]),
                }
            )
            cursor += width + gap
    return specs


KEY_SPECS = _build_key_specs()


def _box_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (upper[0] - lower[0], upper[1] - lower[1], upper[2] - lower[2])


def _rounded_slab_mesh(
    *,
    width: float,
    depth: float,
    thickness: float,
    corner_radius: float,
    mesh_name: str,
):
    profile = rounded_rect_profile(
        width,
        depth,
        min(corner_radius, width * 0.24, depth * 0.24),
        corner_segments=8,
    )
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, cap=True, center=True, closed=True),
        mesh_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thin_laptop")

    aluminum = model.material("aluminum", rgba=(0.76, 0.77, 0.79, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.38, 0.40, 0.43, 1.0))
    key_black = model.material("key_black", rgba=(0.13, 0.14, 0.16, 1.0))
    touchpad_grey = model.material("touchpad_grey", rgba=(0.58, 0.60, 0.63, 1.0))
    screen_black = model.material("screen_black", rgba=(0.05, 0.06, 0.07, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.16, 0.21, 0.28, 0.90))

    base = model.part("base")
    base.visual(
        _rounded_slab_mesh(
            width=BASE_WIDTH,
            depth=BASE_DEPTH,
            thickness=0.010,
            corner_radius=0.015,
            mesh_name="base_bottom_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=aluminum,
        name="bottom_shell",
    )
    base.visual(
        Box((BASE_WIDTH, 0.082, 0.006)),
        origin=Origin(xyz=(0.0, -0.071, 0.013)),
        material=aluminum,
        name="front_palmrest",
    )
    base.visual(
        Box((0.230, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.098, 0.013)),
        material=aluminum,
        name="rear_deck",
    )
    base.visual(
        Box((0.018, 0.142, 0.006)),
        origin=Origin(xyz=(-0.1485, 0.025, 0.013)),
        material=aluminum,
        name="left_side_deck",
    )
    base.visual(
        Box((0.018, 0.142, 0.006)),
        origin=Origin(xyz=(0.1485, 0.025, 0.013)),
        material=aluminum,
        name="right_side_deck",
    )
    base.visual(
        Box((0.272, 0.112, KEYBOARD_FLOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.025, KEYBOARD_FLOOR_Z)),
        material=dark_aluminum,
        name="keyboard_floor",
    )
    base.visual(
        Box((0.014, 0.082, 0.0012)),
        origin=Origin(xyz=(-0.138, 0.016, 0.0156)),
        material=screen_black,
        name="left_speaker",
    )
    base.visual(
        Box((0.014, 0.082, 0.0012)),
        origin=Origin(xyz=(0.138, 0.016, 0.0156)),
        material=screen_black,
        name="right_speaker",
    )
    base.visual(
        Box((0.118, 0.076, 0.0004)),
        origin=Origin(xyz=(0.0, -0.064, 0.0158)),
        material=dark_aluminum,
        name="touchpad_border",
    )
    base.visual(
        Box((0.112, 0.070, 0.0006)),
        origin=Origin(xyz=(0.0, -0.064, 0.0157)),
        material=touchpad_grey,
        name="touchpad_surface",
    )
    base.visual(
        Box((0.040, 0.018, 0.006)),
        origin=Origin(xyz=(-HINGE_X, 0.103, 0.013)),
        material=aluminum,
        name="left_hinge_mount",
    )
    base.visual(
        Box((0.040, 0.018, 0.006)),
        origin=Origin(xyz=(HINGE_X, 0.103, 0.013)),
        material=aluminum,
        name="right_hinge_mount",
    )
    base.visual(
        Cylinder(radius=0.0024, length=0.020),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="left_hinge_pin",
    )
    base.visual(
        Cylinder(radius=0.0024, length=0.020),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="right_hinge_pin",
    )
    for spec in KEY_SPECS:
        base.visual(
            Box((float(spec["width"]) * 0.34, float(spec["depth"]) * 0.34, GUIDE_HEIGHT)),
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), 0.01245)),
            material=dark_aluminum,
            name=str(spec["guide_name"]),
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)),
        mass=1.28,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        _rounded_slab_mesh(
            width=0.312,
            depth=0.214,
            thickness=0.0055,
            corner_radius=0.013,
            mesh_name="lid_panel_shell",
        ),
        origin=Origin(xyz=(0.0, -0.108, 0.00275)),
        material=aluminum,
        name="lid_panel",
    )
    lid.visual(
        Box((0.298, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, -0.194, 0.0006)),
        material=screen_black,
        name="bezel_top",
    )
    lid.visual(
        Box((0.298, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, -0.022, 0.0006)),
        material=screen_black,
        name="bezel_bottom",
    )
    lid.visual(
        Box((0.006, 0.170, 0.0012)),
        origin=Origin(xyz=(-0.146, -0.108, 0.0006)),
        material=screen_black,
        name="bezel_left",
    )
    lid.visual(
        Box((0.006, 0.170, 0.0012)),
        origin=Origin(xyz=(0.146, -0.108, 0.0006)),
        material=screen_black,
        name="bezel_right",
    )
    lid.visual(
        Box((0.286, 0.178, 0.0008)),
        origin=Origin(xyz=(0.0, -0.110, 0.0008)),
        material=screen_glass,
        name="screen_glass",
    )
    lid.visual(
        Box((0.008, 0.003, 0.001)),
        origin=Origin(xyz=(0.0, -0.194, 0.0006)),
        material=dark_aluminum,
        name="webcam",
    )
    lid.visual(
        Cylinder(radius=0.0030, length=0.022),
        origin=Origin(xyz=(-HINGE_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="left_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.0030, length=0.022),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="right_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.312, 0.214, 0.006)),
        mass=0.74,
        origin=Origin(xyz=(0.0, -0.108, 0.003)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(LID_DEFAULT_ROLL, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.0,
            lower=LID_OPEN_EXTRA,
            upper=LID_CLOSE_EXTRA,
        ),
    )

    for spec in KEY_SPECS:
        key = model.part(str(spec["part_name"]))
        width = float(spec["width"])
        depth = float(spec["depth"])
        key.visual(
            Box((width, depth, KEYCAP_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, 0.0007)),
            material=key_black,
            name="keycap",
        )
        key.visual(
            Box((width * 0.42, depth * 0.42, KEY_STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, -0.0010)),
            material=dark_aluminum,
            name="stem",
        )
        key.inertial = Inertial.from_geometry(
            Box((width, depth, 0.004)),
            mass=0.0065 if width < 0.03 else 0.013,
            origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        )
        model.articulation(
            str(spec["joint_name"]),
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), KEY_ORIGIN_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=0.10,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("base_to_lid")

    ctx.allow_overlap(
        base,
        lid,
        elem_a="left_hinge_pin",
        elem_b="left_hinge_barrel",
        reason="The left laptop hinge barrel nests over its hinge pin to keep the lid physically connected through rotation.",
    )
    ctx.allow_overlap(
        base,
        lid,
        elem_a="right_hinge_pin",
        elem_b="right_hinge_barrel",
        reason="The right laptop hinge barrel nests over its hinge pin to keep the lid physically connected through rotation.",
    )
    ctx.allow_overlap(
        base,
        lid,
        elem_a="left_hinge_mount",
        elem_b="left_hinge_barrel",
        reason="The simplified left hinge barrel slightly enters the compact hinge mount that represents the enclosed hinge cassette.",
    )
    ctx.allow_overlap(
        base,
        lid,
        elem_a="right_hinge_mount",
        elem_b="right_hinge_barrel",
        reason="The simplified right hinge barrel slightly enters the compact hinge mount that represents the enclosed hinge cassette.",
    )

    key_parts: dict[str, object] = {}
    key_joints: dict[str, object] = {}
    for spec in KEY_SPECS:
        part_name = str(spec["part_name"])
        joint_name = str(spec["joint_name"])
        key_parts[part_name] = object_model.get_part(part_name)
        key_joints[part_name] = object_model.get_articulation(joint_name)
        ctx.allow_overlap(
            base,
            key_parts[part_name],
            elem_a=str(spec["guide_name"]),
            elem_b="stem",
            reason="Each key stem overlaps its guide boss so the laptop keys remain physically guided while plunging into the deck.",
        )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    base_size = _box_size(ctx.part_element_world_aabb(base, elem="bottom_shell"))
    ctx.check(
        "base_realistic_size",
        base_size is not None and 0.30 <= base_size[0] <= 0.33 and 0.22 <= base_size[1] <= 0.23 and 0.009 <= base_size[2] <= 0.012,
        f"Base size should read as a thin 13-inch-class laptop, got {base_size!r}.",
    )
    ctx.check(
        "keyboard_key_count",
        len(KEY_SPECS) >= 50,
        f"Expected a dense laptop keyboard, got only {len(KEY_SPECS)} articulated keys.",
    )
    ctx.check(
        "lid_hinge_axis",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"Lid hinge should rotate about the rear left-right axis, got {lid_hinge.axis!r}.",
    )
    lid_limits = lid_hinge.motion_limits
    ctx.check(
        "lid_hinge_limits",
        lid_limits is not None and lid_limits.lower is not None and lid_limits.upper is not None and lid_limits.lower < 0.0 < lid_limits.upper,
        f"Lid hinge should support both a more-open pose and a closed pose, got {lid_limits!r}.",
    )

    ctx.expect_overlap(
        lid,
        base,
        axes="x",
        min_overlap=0.26,
        elem_a="lid_panel",
        elem_b="bottom_shell",
        name="lid_matches_base_width",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.020,
        positive_elem="screen_glass",
        negative_elem="rear_deck",
        name="screen_clears_deck_in_default_open_pose",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="x",
        min_overlap=0.26,
        elem_a="screen_glass",
        elem_b="keyboard_floor",
        name="screen_sits_over_keyboard_span",
    )

    with ctx.pose({lid_hinge: LID_OPEN_EXTRA}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_limit_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_open_limit_no_floating")
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.020,
            positive_elem="screen_glass",
            negative_elem="rear_deck",
            name="lid_open_limit_screen_clearance",
        )

    with ctx.pose({lid_hinge: LID_CLOSE_EXTRA}):
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_closed_limit_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_closed_limit_no_floating")
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.0015,
            max_gap=0.0060,
            positive_elem="lid_panel",
            negative_elem="front_palmrest",
            name="lid_closed_clearance",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.180,
            elem_a="lid_panel",
            elem_b="bottom_shell",
            name="lid_covers_base_when_closed",
        )
        lid_size = _box_size(ctx.part_world_aabb(lid))
        ctx.check(
            "lid_realistic_size",
            lid_size is not None and 0.30 <= lid_size[0] <= 0.32 and 0.20 <= lid_size[1] <= 0.22 and 0.005 <= lid_size[2] <= 0.010,
            f"Lid should remain thin and wide when closed, got {lid_size!r}.",
        )

    for spec in KEY_SPECS:
        part_name = str(spec["part_name"])
        key_part = key_parts[part_name]
        key_joint = key_joints[part_name]
        limits = key_joint.motion_limits
        ctx.check(
            f"{part_name}_axis",
            tuple(key_joint.axis) == (0.0, 0.0, -1.0),
            f"{part_name} should plunge downward into the deck, got axis {key_joint.axis!r}.",
        )
        ctx.check(
            f"{part_name}_travel",
            limits is not None and limits.lower == 0.0 and limits.upper == KEY_TRAVEL,
            f"{part_name} should have short laptop-key travel, got {limits!r}.",
        )
        ctx.expect_within(
            key_part,
            base,
            axes="xy",
            inner_elem="keycap",
            outer_elem="keyboard_floor",
            margin=0.006,
            name=f"{part_name}_within_keyboard_deck",
        )
        ctx.expect_gap(
            key_part,
            base,
            axis="z",
            min_gap=0.0030,
            max_gap=0.0048,
            positive_elem="keycap",
            negative_elem="keyboard_floor",
            name=f"{part_name}_rest_above_keyboard_floor",
        )
        with ctx.pose({key_joint: KEY_TRAVEL}):
            ctx.expect_gap(
                key_part,
                base,
                axis="z",
                min_gap=0.0014,
                max_gap=0.0033,
                positive_elem="keycap",
                negative_elem="keyboard_floor",
                name=f"{part_name}_pressed_stays_guided",
            )

    for part_name in ("key_q", "key_g", "key_spacebar", "key_right_shift"):
        joint = key_joints[part_name]
        with ctx.pose({joint: KEY_TRAVEL}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{part_name}_pressed_pose_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{part_name}_pressed_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
