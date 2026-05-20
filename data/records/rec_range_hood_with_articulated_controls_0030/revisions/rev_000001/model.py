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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    shadow = model.material("shadow", rgba=(0.12, 0.13, 0.14, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.10, 0.11, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.11
    panel_thickness = 0.003
    chimney_width = 0.34
    chimney_depth = 0.28
    chimney_height = 0.68
    chimney_wall = 0.003
    button_travel = 0.004
    stem_length = panel_thickness + button_travel

    face_center_z = canopy_height * 0.5
    front_outer_y = canopy_depth * 0.5
    front_inner_y = front_outer_y - panel_thickness

    def rect_profile(
        width: float, height: float, *, center: tuple[float, float] = (0.0, 0.0)
    ) -> list[tuple[float, float]]:
        cx, cy = center
        half_w = width * 0.5
        half_h = height * 0.5
        return [
            (cx - half_w, cy - half_h),
            (cx + half_w, cy - half_h),
            (cx + half_w, cy + half_h),
            (cx - half_w, cy + half_h),
        ]

    def circle_profile(
        radius: float,
        *,
        center: tuple[float, float] = (0.0, 0.0),
        segments: int = 24,
    ) -> list[tuple[float, float]]:
        cx, cy = center
        return [
            (
                cx + radius * math.cos((2.0 * math.pi * index) / segments),
                cy + radius * math.sin((2.0 * math.pi * index) / segments),
            )
            for index in range(segments)
        ]

    def front_panel_mesh():
        rect_hole_width = 0.024
        rect_hole_height = 0.008
        round_hole_radius = 0.006

        hole_profiles = [
            rect_profile(rect_hole_width, rect_hole_height, center=(-0.24, -0.018)),
            rect_profile(rect_hole_width, rect_hole_height, center=(-0.24, 0.010)),
            circle_profile(round_hole_radius, center=(0.21, -0.020)),
            circle_profile(round_hole_radius, center=(0.27, -0.020)),
            circle_profile(round_hole_radius, center=(0.24, 0.006)),
        ]

        geom = ExtrudeWithHolesGeometry(
            rect_profile(canopy_width, canopy_height),
            hole_profiles,
            panel_thickness,
            cap=True,
            center=True,
            closed=True,
        )
        geom.rotate_x(-math.pi / 2.0)
        return mesh_from_geometry(geom, ASSETS.mesh_path("range_hood_front_panel.obj"))

    def chimney_shell_mesh():
        geom = ExtrudeWithHolesGeometry(
            rect_profile(chimney_width, chimney_depth),
            [rect_profile(chimney_width - 2.0 * chimney_wall, chimney_depth - 2.0 * chimney_wall)],
            chimney_height,
            cap=False,
            center=False,
            closed=True,
        )
        return mesh_from_geometry(geom, ASSETS.mesh_path("range_hood_chimney_shell.obj"))

    hood_body = model.part("hood_body")
    hood_body.visual(
        front_panel_mesh(),
        origin=Origin(xyz=(0.0, front_outer_y - panel_thickness * 0.5, face_center_z)),
        material=stainless,
        name="canopy_front",
    )
    hood_body.visual(
        Box((canopy_width, canopy_depth, panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - panel_thickness * 0.5)),
        material=stainless,
        name="canopy_top",
    )
    hood_body.visual(
        Box((panel_thickness, canopy_depth, canopy_height + panel_thickness)),
        origin=Origin(
            xyz=(
                -canopy_width * 0.5 + panel_thickness * 0.5,
                0.0,
                (canopy_height + panel_thickness) * 0.5,
            )
        ),
        material=stainless,
        name="canopy_left",
    )
    hood_body.visual(
        Box((panel_thickness, canopy_depth, canopy_height + panel_thickness)),
        origin=Origin(
            xyz=(
                canopy_width * 0.5 - panel_thickness * 0.5,
                0.0,
                (canopy_height + panel_thickness) * 0.5,
            )
        ),
        material=stainless,
        name="canopy_right",
    )
    hood_body.visual(
        Box((canopy_width, panel_thickness, canopy_height + panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -canopy_depth * 0.5 + panel_thickness * 0.5,
                (canopy_height + panel_thickness) * 0.5,
            )
        ),
        material=stainless,
        name="canopy_back",
    )
    hood_body.visual(
        Box((0.34, 0.19, 0.004)),
        origin=Origin(xyz=(-0.17, 0.1525, 0.020), rpy=(0.12, 0.0, 0.0)),
        material=shadow,
        name="filter_left",
    )
    hood_body.visual(
        Box((0.34, 0.19, 0.004)),
        origin=Origin(xyz=(0.17, 0.1525, 0.020), rpy=(0.12, 0.0, 0.0)),
        material=shadow,
        name="filter_right",
    )
    hood_body.visual(
        chimney_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - 0.001)),
        material=stainless,
        name="chimney_shell",
    )

    def add_rect_button(
        *,
        part_name: str,
        joint_name: str,
        x: float,
        z: float,
        cap_size: tuple[float, float],
        stem_size: tuple[float, float],
        stop_size: tuple[float, float],
    ) -> None:
        cap_w, cap_h = cap_size
        stem_w, stem_h = stem_size
        stop_w, stop_h = stop_size
        cap_thickness = 0.004
        stop_thickness = 0.003

        button = model.part(part_name)
        button.visual(
            Box((cap_w, cap_thickness, cap_h)),
            origin=Origin(xyz=(0.0, stem_length * 0.5 + cap_thickness * 0.5, 0.0)),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Box((stem_w, stem_length, stem_h)),
            material=button_black,
            name="button_stem",
        )
        button.visual(
            Box((stop_w, stop_thickness, stop_h)),
            origin=Origin(xyz=(0.0, -(stem_length * 0.5 + stop_thickness * 0.5), 0.0)),
            material=button_black,
            name="button_stop",
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(x, front_inner_y + stem_length * 0.5, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.03,
                lower=0.0,
                upper=button_travel,
            ),
        )

    def add_round_button(
        *,
        part_name: str,
        joint_name: str,
        x: float,
        z: float,
        cap_radius: float,
        stem_radius: float,
        stop_radius: float,
    ) -> None:
        cap_thickness = 0.004
        stop_thickness = 0.003
        y_axis = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))

        button = model.part(part_name)
        button.visual(
            Cylinder(radius=cap_radius, length=cap_thickness),
            origin=Origin(
                xyz=(0.0, stem_length * 0.5 + cap_thickness * 0.5, 0.0),
                rpy=y_axis.rpy,
            ),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=stem_radius, length=stem_length),
            origin=Origin(rpy=y_axis.rpy),
            material=button_black,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=stop_radius, length=stop_thickness),
            origin=Origin(
                xyz=(0.0, -(stem_length * 0.5 + stop_thickness * 0.5), 0.0),
                rpy=y_axis.rpy,
            ),
            material=button_black,
            name="button_stop",
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(x, front_inner_y + stem_length * 0.5, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.03,
                lower=0.0,
                upper=button_travel,
            ),
        )

    add_rect_button(
        part_name="left_button_upper",
        joint_name="hood_to_left_button_upper",
        x=-0.24,
        z=0.073,
        cap_size=(0.032, 0.014),
        stem_size=(0.024, 0.008),
        stop_size=(0.038, 0.018),
    )
    add_rect_button(
        part_name="left_button_lower",
        joint_name="hood_to_left_button_lower",
        x=-0.24,
        z=0.045,
        cap_size=(0.032, 0.014),
        stem_size=(0.024, 0.008),
        stop_size=(0.038, 0.018),
    )
    add_round_button(
        part_name="right_button_top_left",
        joint_name="hood_to_right_button_top_left",
        x=0.21,
        z=0.075,
        cap_radius=0.008,
        stem_radius=0.006,
        stop_radius=0.0095,
    )
    add_round_button(
        part_name="right_button_top_right",
        joint_name="hood_to_right_button_top_right",
        x=0.27,
        z=0.075,
        cap_radius=0.008,
        stem_radius=0.006,
        stop_radius=0.0095,
    )
    add_round_button(
        part_name="right_button_bottom",
        joint_name="hood_to_right_button_bottom",
        x=0.24,
        z=0.049,
        cap_radius=0.008,
        stem_radius=0.006,
        stop_radius=0.0095,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    button_specs = [
        ("left_button_upper", "hood_to_left_button_upper"),
        ("left_button_lower", "hood_to_left_button_lower"),
        ("right_button_top_left", "hood_to_right_button_top_left"),
        ("right_button_top_right", "hood_to_right_button_top_right"),
        ("right_button_bottom", "hood_to_right_button_bottom"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    articulated = getattr(object_model, "articulations", ())
    ctx.check(
        "only_five_button_articulations",
        len(articulated) == 5,
        details=f"expected 5 articulations, found {len(articulated)}",
    )

    hood_aabb = ctx.part_world_aabb(hood_body)
    front_aabb = ctx.part_element_world_aabb(hood_body, elem="canopy_front")
    chimney_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_shell")
    assert hood_aabb is not None
    assert front_aabb is not None
    assert chimney_aabb is not None

    hood_size = tuple(hood_aabb[1][i] - hood_aabb[0][i] for i in range(3))
    front_size = tuple(front_aabb[1][i] - front_aabb[0][i] for i in range(3))
    chimney_size = tuple(chimney_aabb[1][i] - chimney_aabb[0][i] for i in range(3))

    ctx.check(
        "range_hood_width_realistic",
        0.88 <= hood_size[0] <= 0.92,
        details=f"hood width {hood_size[0]:.3f} m is outside realistic 0.88-0.92 m",
    )
    ctx.check(
        "range_hood_height_realistic",
        0.77 <= hood_size[2] <= 0.81,
        details=f"hood height {hood_size[2]:.3f} m is outside realistic 0.77-0.81 m",
    )
    ctx.check(
        "canopy_face_is_flat_panel",
        0.0025 <= front_size[1] <= 0.0045 and 0.105 <= front_size[2] <= 0.115,
        details=f"front face size={front_size}",
    )
    ctx.check(
        "chimney_shell_is_tall_and_narrow",
        chimney_size[2] >= 0.67 and chimney_size[0] <= 0.35 and chimney_size[1] <= 0.29,
        details=f"chimney size={chimney_size}",
    )
    ctx.check(
        "chimney_narrower_than_canopy",
        chimney_size[0] < front_size[0] - 0.45,
        details=f"front width={front_size[0]:.3f}, chimney width={chimney_size[0]:.3f}",
    )

    for part_name, joint_name in button_specs:
        button = object_model.get_part(part_name)
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        assert limits is not None
        assert limits.upper is not None

        ctx.expect_contact(
            button,
            hood_body,
            elem_a="button_stop",
            elem_b="canopy_front",
            name=f"{part_name}_released_stop_contact",
        )
        ctx.expect_contact(
            button,
            hood_body,
            elem_a="button_stem",
            elem_b="canopy_front",
            name=f"{part_name}_stem_guided_in_panel",
        )

        released_position = ctx.part_world_position(button)
        assert released_position is not None

        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_pressed_no_floating")
            ctx.expect_contact(
                button,
                hood_body,
                elem_a="button_cap",
                elem_b="canopy_front",
                name=f"{part_name}_pressed_cap_contact",
            )
            ctx.expect_contact(
                button,
                hood_body,
                elem_a="button_stem",
                elem_b="canopy_front",
                name=f"{part_name}_pressed_stem_guided",
            )

            pressed_position = ctx.part_world_position(button)
            assert pressed_position is not None
            ctx.check(
                f"{part_name}_moves_inward_only",
                (
                    pressed_position[1] < released_position[1] - 0.0035
                    and abs(pressed_position[0] - released_position[0]) < 1e-6
                    and abs(pressed_position[2] - released_position[2]) < 1e-6
                ),
                details=(
                    f"released={released_position}, pressed={pressed_position}; "
                    "expected inward Y motion only"
                ),
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
