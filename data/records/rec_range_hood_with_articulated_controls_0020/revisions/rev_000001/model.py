from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_BACK_HEIGHT = 0.22
CANOPY_FRONT_HEIGHT = 0.16
PANEL_THICKNESS = 0.018
FRONT_FACE_BACK_Y = 0.468
FRONT_FACE_CENTER_Y = 0.490
BUTTON_TRAVEL = 0.004


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _curved_strip_profile(
    width: float,
    thickness: float,
    bow: float,
    *,
    segments: int = 12,
) -> list[tuple[float, float]]:
    half_width = width * 0.5
    front: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        x = -half_width + (width * t)
        normalized = 0.0 if half_width == 0.0 else x / half_width
        y = thickness + bow * (1.0 - (normalized * normalized))
        front.append((x, y))

    back = [(x, 0.0) for x, _ in reversed(front)]
    return front + back


def _front_strip_mesh(
    *,
    width: float,
    thickness: float,
    bow: float,
    height: float,
    filename: str,
):
    profile = _curved_strip_profile(width, thickness, bow)
    return _save_mesh(ExtrudeGeometry.from_z0(profile, height), filename)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 20,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _side_panel_mesh(
    *,
    depth: float,
    back_height: float,
    front_height: float,
    thickness: float,
    filename: str,
):
    profile = [
        (0.0, 0.0),
        (0.0, depth),
        (front_height, depth),
        (back_height, 0.0),
    ]
    geometry = ExtrudeGeometry.from_z0(profile, thickness).rotate_y(math.pi / 2.0)
    return _save_mesh(geometry, filename)


def _control_panel_mesh(filename: str):
    outer = [
        (-0.059, -0.031),
        (0.059, -0.031),
        (0.059, 0.031),
        (-0.059, 0.031),
    ]
    holes = [
        _circle_profile(0.0045, center=(-0.017, 0.016)),
        _circle_profile(0.0045, center=(0.017, 0.016)),
        _circle_profile(0.0045, center=(-0.017, -0.016)),
        _circle_profile(0.0045, center=(0.017, -0.016)),
    ]
    geometry = ExtrudeWithHolesGeometry(
        outer,
        holes,
        0.004,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(geometry, filename)


def _has_visual(part, visual_name: str) -> bool:
    try:
        part.get_visual(visual_name)
        return True
    except Exception:
        return False


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.45, 0.47, 0.49, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    filter_metal = model.material("filter_metal", rgba=(0.30, 0.32, 0.34, 1.0))
    lens = model.material("lens", rgba=(0.88, 0.90, 0.82, 0.65))
    button_finish = model.material("button_finish", rgba=(0.15, 0.15, 0.16, 1.0))

    canopy = model.part("canopy_body")

    lower_lip_mesh = _front_strip_mesh(
        width=CANOPY_WIDTH,
        thickness=0.016,
        bow=0.012,
        height=0.058,
        filename="canopy_lower_lip.obj",
    )
    upper_wing_mesh = _front_strip_mesh(
        width=0.36,
        thickness=0.016,
        bow=0.010,
        height=0.082,
        filename="canopy_upper_wing.obj",
    )
    control_panel_mesh = _control_panel_mesh("control_panel.obj")
    side_mesh = _side_panel_mesh(
        depth=CANOPY_DEPTH,
        back_height=CANOPY_BACK_HEIGHT,
        front_height=CANOPY_FRONT_HEIGHT,
        thickness=PANEL_THICKNESS,
        filename="canopy_side_panel.obj",
    )
    canopy.visual(
        Box((CANOPY_WIDTH, PANEL_THICKNESS, CANOPY_BACK_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS * 0.5, CANOPY_BACK_HEIGHT * 0.5)),
        material=stainless,
        name="back_panel",
    )
    canopy.visual(
        Box((0.389, math.hypot(0.472, CANOPY_BACK_HEIGHT - CANOPY_FRONT_HEIGHT) + 0.010, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(-0.2555, 0.254, 0.189),
            rpy=(math.atan2(CANOPY_FRONT_HEIGHT - CANOPY_BACK_HEIGHT, 0.472), 0.0, 0.0),
        ),
        material=stainless,
        name="left_top_panel",
    )
    canopy.visual(
        Box((0.389, math.hypot(0.472, CANOPY_BACK_HEIGHT - CANOPY_FRONT_HEIGHT) + 0.010, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.2555, 0.254, 0.189),
            rpy=(math.atan2(CANOPY_FRONT_HEIGHT - CANOPY_BACK_HEIGHT, 0.472), 0.0, 0.0),
        ),
        material=stainless,
        name="right_top_panel",
    )
    canopy.visual(
        side_mesh,
        origin=Origin(xyz=(-CANOPY_WIDTH * 0.5, 0.0, CANOPY_BACK_HEIGHT)),
        material=stainless,
        name="left_side_panel",
    )
    canopy.visual(
        side_mesh,
        origin=Origin(xyz=(CANOPY_WIDTH * 0.5 - PANEL_THICKNESS, 0.0, CANOPY_BACK_HEIGHT)),
        material=stainless,
        name="right_side_panel",
    )
    canopy.visual(
        lower_lip_mesh,
        origin=Origin(xyz=(0.0, FRONT_FACE_BACK_Y, 0.018)),
        material=stainless,
        name="lower_front_lip",
    )
    canopy.visual(
        upper_wing_mesh,
        origin=Origin(xyz=(-0.270, FRONT_FACE_BACK_Y, 0.078)),
        material=stainless,
        name="left_upper_fascia",
    )
    canopy.visual(
        upper_wing_mesh,
        origin=Origin(xyz=(0.270, FRONT_FACE_BACK_Y, 0.078)),
        material=stainless,
        name="right_upper_fascia",
    )

    bezel_depth = 0.024
    bezel_center_y = 0.478
    canopy.visual(
        Box((0.154, bezel_depth, 0.012)),
        origin=Origin(xyz=(0.0, bezel_center_y, 0.080)),
        material=brushed_dark,
        name="control_bezel_bottom",
    )
    canopy.visual(
        Box((0.154, bezel_depth, 0.012)),
        origin=Origin(xyz=(0.0, bezel_center_y, 0.154)),
        material=brushed_dark,
        name="control_bezel_top",
    )
    canopy.visual(
        Box((0.018, bezel_depth, 0.074)),
        origin=Origin(xyz=(-0.068, bezel_center_y, 0.117)),
        material=brushed_dark,
        name="control_bezel_left",
    )
    canopy.visual(
        Box((0.018, bezel_depth, 0.074)),
        origin=Origin(xyz=(0.068, bezel_center_y, 0.117)),
        material=brushed_dark,
        name="control_bezel_right",
    )

    button_specs = [
        ("button_top_left", -0.017, 0.131),
        ("button_top_right", 0.017, 0.131),
        ("button_bottom_left", -0.017, 0.099),
        ("button_bottom_right", 0.017, 0.099),
    ]
    canopy.visual(
        control_panel_mesh,
        origin=Origin(xyz=(0.0, 0.484, 0.117)),
        material=charcoal,
        name="control_inner_panel",
    )

    canopy.visual(
        Box((0.350, 0.240, 0.018)),
        origin=Origin(xyz=(0.0, 0.160, 0.217)),
        material=stainless,
        name="chimney_collar",
    )
    canopy.visual(
        Box((0.380, 0.440, 0.010)),
        origin=Origin(xyz=(-0.205, 0.238, 0.012)),
        material=filter_metal,
        name="left_filter",
    )
    canopy.visual(
        Box((0.380, 0.440, 0.010)),
        origin=Origin(xyz=(0.205, 0.238, 0.012)),
        material=filter_metal,
        name="right_filter",
    )
    canopy.visual(
        Box((0.090, 0.024, 0.008)),
        origin=Origin(xyz=(-0.230, 0.458, 0.008)),
        material=lens,
        name="left_light",
    )
    canopy.visual(
        Box((0.090, 0.024, 0.008)),
        origin=Origin(xyz=(0.230, 0.458, 0.008)),
        material=lens,
        name="right_light",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_BACK_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, CANOPY_DEPTH * 0.5, CANOPY_BACK_HEIGHT * 0.5)),
    )

    chimney = model.part("chimney_cover")
    chimney_width = 0.320
    chimney_depth = 0.240
    chimney_height = 0.720
    chimney_thickness = 0.012
    chimney.visual(
        Box((chimney_width, chimney_thickness, chimney_height)),
        origin=Origin(xyz=(0.0, chimney_thickness * 0.5, chimney_height * 0.5)),
        material=stainless,
        name="chimney_back",
    )
    chimney.visual(
        Box((chimney_width, chimney_thickness, chimney_height)),
        origin=Origin(xyz=(0.0, chimney_depth - chimney_thickness * 0.5, chimney_height * 0.5)),
        material=stainless,
        name="chimney_front",
    )
    chimney.visual(
        Box((chimney_thickness, chimney_depth, chimney_height)),
        origin=Origin(xyz=(-chimney_width * 0.5 + chimney_thickness * 0.5, chimney_depth * 0.5, chimney_height * 0.5)),
        material=stainless,
        name="chimney_left",
    )
    chimney.visual(
        Box((chimney_thickness, chimney_depth, chimney_height)),
        origin=Origin(xyz=(chimney_width * 0.5 - chimney_thickness * 0.5, chimney_depth * 0.5, chimney_height * 0.5)),
        material=stainless,
        name="chimney_right",
    )
    chimney.inertial = Inertial.from_geometry(
        Box((chimney_width, chimney_depth, chimney_height)),
        mass=4.0,
        origin=Origin(xyz=(0.0, chimney_depth * 0.5, chimney_height * 0.5)),
    )

    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.040, 0.226)),
    )

    for button_name, x_pos, z_pos in button_specs:
        button = model.part(button_name)
        button.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_finish,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=0.0075, length=0.002),
            origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_finish,
            name="button_retainer",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.024, 0.014, 0.024)),
            mass=0.03,
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
        )
        model.articulation(
            f"canopy_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(x_pos, 0.486, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    canopy = object_model.get_part("canopy_body")
    chimney = object_model.get_part("chimney_cover")
    button_names = [
        "button_top_left",
        "button_top_right",
        "button_bottom_left",
        "button_bottom_right",
    ]
    buttons = {name: object_model.get_part(name) for name in button_names}
    joints = {name: object_model.get_articulation(f"canopy_to_{name}") for name in button_names}
    chimney_joint = object_model.get_articulation("canopy_to_chimney")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=16)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "canopy_feature_visuals_present",
        all(
            _has_visual(canopy, visual_name)
            for visual_name in (
                "lower_front_lip",
                "left_upper_fascia",
                "right_upper_fascia",
                "chimney_collar",
                "control_inner_panel",
                "left_filter",
                "right_filter",
            )
        ),
        "Canopy is missing one or more key visual features.",
    )
    ctx.check(
        "chimney_feature_visuals_present",
        all(
            _has_visual(chimney, visual_name)
            for visual_name in ("chimney_back", "chimney_front", "chimney_left", "chimney_right")
        ),
        "Chimney cover is missing one or more shell panels.",
    )
    ctx.check(
        "button_joint_types",
        all(joint.articulation_type == ArticulationType.PRISMATIC for joint in joints.values())
        and chimney_joint.articulation_type == ArticulationType.FIXED,
        "Only the four push-buttons should be movable joints; the chimney must be fixed.",
    )
    ctx.check(
        "button_joint_axes",
        all(tuple(joint.axis) == (0.0, -1.0, 0.0) for joint in joints.values()),
        "Button joints must move inward normal to the front canopy face.",
    )

    canopy_aabb = ctx.part_world_aabb(canopy)
    if canopy_aabb is not None:
        canopy_width = canopy_aabb[1][0] - canopy_aabb[0][0]
        canopy_depth = canopy_aabb[1][1] - canopy_aabb[0][1]
        canopy_height = canopy_aabb[1][2] - canopy_aabb[0][2]
        ctx.check(
            "range_hood_canopy_proportions",
            0.84 <= canopy_width <= 0.96
            and 0.47 <= canopy_depth <= 0.53
            and 0.20 <= canopy_height <= 0.25,
            (
                "Canopy proportions should read like a real 90 cm class chimney hood; "
                f"got width={canopy_width:.3f}, depth={canopy_depth:.3f}, height={canopy_height:.3f}."
            ),
        )
    else:
        ctx.fail("range_hood_canopy_proportions", "Could not measure canopy bounds.")

    chimney_aabb = ctx.part_world_aabb(chimney)
    if chimney_aabb is not None:
        chimney_height = chimney_aabb[1][2] - chimney_aabb[0][2]
        chimney_width = chimney_aabb[1][0] - chimney_aabb[0][0]
        ctx.check(
            "chimney_cover_proportions",
            0.28 <= chimney_width <= 0.34 and 0.68 <= chimney_height <= 0.74,
            (
                "Chimney cover should be a narrow straight vertical duct cover; "
                f"got width={chimney_width:.3f}, height={chimney_height:.3f}."
            ),
        )
    else:
        ctx.fail("chimney_cover_proportions", "Could not measure chimney bounds.")

    ctx.expect_contact(chimney, canopy, elem_b="chimney_collar")
    ctx.expect_origin_distance(chimney, canopy, axes="x", max_dist=0.001)
    ctx.expect_overlap(chimney, canopy, axes="x", min_overlap=0.30, elem_b="chimney_collar")

    button_rest_positions = {name: ctx.part_world_position(button) for name, button in buttons.items()}
    ctx.check(
        "button_positions_resolved",
        all(position is not None for position in button_rest_positions.values()),
        "Could not resolve all button world positions.",
    )
    if all(position is not None for position in button_rest_positions.values()):
        top_left = button_rest_positions["button_top_left"]
        top_right = button_rest_positions["button_top_right"]
        bottom_left = button_rest_positions["button_bottom_left"]
        bottom_right = button_rest_positions["button_bottom_right"]
        assert top_left is not None
        assert top_right is not None
        assert bottom_left is not None
        assert bottom_right is not None
        x_pitch = top_right[0] - top_left[0]
        z_pitch = top_left[2] - bottom_left[2]
        group_center_x = (top_left[0] + top_right[0] + bottom_left[0] + bottom_right[0]) / 4.0
        ctx.check(
            "buttons_form_compact_square",
            0.030 <= x_pitch <= 0.040
            and 0.028 <= z_pitch <= 0.038
            and abs(group_center_x) <= 0.003,
            (
                "Buttons should form a compact centered 2x2 square cluster; "
                f"got x_pitch={x_pitch:.3f}, z_pitch={z_pitch:.3f}, center_x={group_center_x:.3f}."
            ),
        )

    for button_name, button in buttons.items():
        joint = joints[button_name]
        limits = joint.motion_limits
        ctx.expect_contact(
            button,
            canopy,
            elem_b="control_inner_panel",
            name=f"{button_name}_rest_contact",
        )
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.expect_contact(
                    button,
                    canopy,
                    elem_b="control_inner_panel",
                    name=f"{button_name}_pressed_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_name}_pressed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button_name}_pressed_no_floating")
                pressed_position = ctx.part_world_position(button)
                rest_position = button_rest_positions[button_name]
                if pressed_position is not None and rest_position is not None:
                    ctx.check(
                        f"{button_name}_press_travel",
                        pressed_position[1] < rest_position[1] - 0.0035,
                        (
                            f"{button_name} should move inward by about 4 mm when pressed; "
                            f"rest_y={rest_position[1]:.4f}, pressed_y={pressed_position[1]:.4f}."
                        ),
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
