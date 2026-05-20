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
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.79, 0.81, 0.83, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    filter_metal = model.material("filter_metal", rgba=(0.56, 0.58, 0.60, 1.0))

    canopy_bottom_width = 0.90
    canopy_bottom_depth = 0.50
    canopy_top_width = 0.56
    canopy_top_depth = 0.30
    canopy_top_center_y = 0.08
    canopy_height = 0.160
    panel_thickness = 0.012

    front_y = -canopy_bottom_depth * 0.5
    back_y = canopy_bottom_depth * 0.5
    top_front_y = canopy_top_center_y - canopy_top_depth * 0.5
    top_back_y = canopy_top_center_y + canopy_top_depth * 0.5
    bottom_half_width = canopy_bottom_width * 0.5
    top_half_width = canopy_top_width * 0.5
    strip_height = 0.058

    def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
        geometry.add_face(a, b, c)
        geometry.add_face(a, c, d)

    def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
        mag = math.sqrt(max(_dot(v, v), 1e-12))
        return (v[0] / mag, v[1] / mag, v[2] / mag)

    def _panel_prism_mesh(
        outer_loop: list[tuple[float, float, float]],
        *,
        thickness: float,
        inward_hint: tuple[float, float, float],
    ) -> MeshGeometry:
        geometry = MeshGeometry()
        normal = _normalize(_cross(_sub(outer_loop[1], outer_loop[0]), _sub(outer_loop[2], outer_loop[1])))
        if _dot(normal, inward_hint) < 0.0:
            normal = (-normal[0], -normal[1], -normal[2])
        inner_loop = [
            (
                point[0] + normal[0] * thickness,
                point[1] + normal[1] * thickness,
                point[2] + normal[2] * thickness,
            )
            for point in outer_loop
        ]

        outer_ids = [geometry.add_vertex(*point) for point in outer_loop]
        inner_ids = [geometry.add_vertex(*point) for point in inner_loop]

        for index in range(1, len(outer_ids) - 1):
            geometry.add_face(outer_ids[0], outer_ids[index], outer_ids[index + 1])
            geometry.add_face(inner_ids[0], inner_ids[index + 1], inner_ids[index])

        for index in range(len(outer_ids)):
            next_index = (index + 1) % len(outer_ids)
            _add_quad(
                geometry,
                outer_ids[index],
                outer_ids[next_index],
                inner_ids[next_index],
                inner_ids[index],
            )
        return geometry

    hood_body = model.part("hood_body")

    left_panel_mesh = mesh_from_geometry(
        _panel_prism_mesh(
            [
                (-bottom_half_width, front_y, 0.0),
                (-bottom_half_width, back_y, 0.0),
                (-top_half_width, top_back_y, canopy_height),
                (-top_half_width, top_front_y, canopy_height),
            ],
            thickness=panel_thickness,
            inward_hint=(1.0, 0.0, 0.25),
        ),
        ASSETS.mesh_path("range_hood_left_panel.obj"),
    )
    right_panel_mesh = mesh_from_geometry(
        _panel_prism_mesh(
            [
                (bottom_half_width, front_y, 0.0),
                (top_half_width, top_front_y, canopy_height),
                (top_half_width, top_back_y, canopy_height),
                (bottom_half_width, back_y, 0.0),
            ],
            thickness=panel_thickness,
            inward_hint=(-1.0, 0.0, 0.25),
        ),
        ASSETS.mesh_path("range_hood_right_panel.obj"),
    )
    rear_panel_mesh = mesh_from_geometry(
        _panel_prism_mesh(
            [
                (-bottom_half_width, back_y, 0.0),
                (bottom_half_width, back_y, 0.0),
                (top_half_width, top_back_y, canopy_height),
                (-top_half_width, top_back_y, canopy_height),
            ],
            thickness=panel_thickness,
            inward_hint=(0.0, -1.0, 0.12),
        ),
        ASSETS.mesh_path("range_hood_rear_panel.obj"),
    )
    front_panel_mesh = mesh_from_geometry(
        _panel_prism_mesh(
            [
                (-bottom_half_width, front_y, strip_height - 0.006),
                (bottom_half_width, front_y, strip_height - 0.006),
                (top_half_width, top_front_y, canopy_height),
                (-top_half_width, top_front_y, canopy_height),
            ],
            thickness=panel_thickness,
            inward_hint=(0.0, 0.92, -0.40),
        ),
        ASSETS.mesh_path("range_hood_front_panel.obj"),
    )

    hood_body.visual(left_panel_mesh, material=stainless, name="left_canopy_side")
    hood_body.visual(right_panel_mesh, material=stainless, name="right_canopy_side")
    hood_body.visual(rear_panel_mesh, material=stainless, name="rear_canopy_panel")
    hood_body.visual(front_panel_mesh, material=stainless, name="front_canopy_panel")
    hood_body.visual(
        Box((canopy_top_width, canopy_top_depth, 0.014)),
        origin=Origin(xyz=(0.0, canopy_top_center_y, canopy_height - 0.006)),
        material=stainless,
        name="canopy_top",
    )
    hood_body.visual(
        Box((canopy_bottom_width, 0.018, strip_height)),
        origin=Origin(xyz=(0.0, front_y + 0.009, strip_height * 0.5)),
        material=satin_steel,
        name="control_strip",
    )
    hood_body.visual(
        Box((0.56, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, front_y + 0.010, 0.004)),
        material=dark_trim,
        name="underside_front_lip",
    )
    chimney_width = 0.32
    chimney_depth = 0.22
    chimney_height = 0.72
    chimney_center_y = 0.11
    chimney_center_z = canopy_height + (chimney_height * 0.5) - 0.008
    hood_body.visual(
        Box((chimney_width, 0.012, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_center_y - (chimney_depth * 0.5) + 0.006, chimney_center_z)
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((chimney_width, 0.012, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_center_y + (chimney_depth * 0.5) - 0.006, chimney_center_z)
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((0.012, chimney_depth, chimney_height)),
        origin=Origin(xyz=(-(chimney_width * 0.5) + 0.006, chimney_center_y, chimney_center_z)),
        material=stainless,
        name="chimney_left",
    )
    hood_body.visual(
        Box((0.012, chimney_depth, chimney_height)),
        origin=Origin(xyz=((chimney_width * 0.5) - 0.006, chimney_center_y, chimney_center_z)),
        material=stainless,
        name="chimney_right",
    )

    pod_outer_width = 0.036
    pod_outer_height = 0.028
    pod_outer_depth = 0.024
    pod_opening = 0.014
    pod_rail_x = (pod_outer_width - pod_opening) * 0.5
    pod_rail_z = (pod_outer_height - pod_opening) * 0.5
    pod_center_y = front_y - (pod_outer_depth * 0.5)
    pod_center_z = 0.032
    hood_body.visual(
        Box((pod_rail_x, pod_outer_depth, pod_outer_height)),
        origin=Origin(xyz=(-(pod_opening * 0.5) - (pod_rail_x * 0.5), pod_center_y, pod_center_z)),
        material=dark_trim,
        name="button_pod_left",
    )
    hood_body.visual(
        Box((pod_rail_x, pod_outer_depth, pod_outer_height)),
        origin=Origin(xyz=((pod_opening * 0.5) + (pod_rail_x * 0.5), pod_center_y, pod_center_z)),
        material=dark_trim,
        name="button_pod_right",
    )
    hood_body.visual(
        Box((pod_outer_width, pod_outer_depth, pod_rail_z)),
        origin=Origin(xyz=(0.0, pod_center_y, pod_center_z + (pod_opening * 0.5) + (pod_rail_z * 0.5))),
        material=dark_trim,
        name="button_pod_top",
    )
    hood_body.visual(
        Box((pod_outer_width, pod_outer_depth, pod_rail_z)),
        origin=Origin(xyz=(0.0, pod_center_y, pod_center_z - (pod_opening * 0.5) - (pod_rail_z * 0.5))),
        material=dark_trim,
        name="button_pod_bottom",
    )

    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_bottom_width, canopy_bottom_depth, canopy_height + chimney_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + chimney_height) * 0.5)),
    )

    def _build_filter(filter_name: str, x_center: float) -> None:
        filter_part = model.part(filter_name)
        filter_part.visual(
            Box((0.30, 0.205, 0.006)),
            origin=Origin(rpy=(0.34, 0.0, 0.0)),
            material=filter_metal,
            name="filter_panel",
        )
        filter_part.visual(
            Box((0.024, 0.020, 0.116)),
            origin=Origin(xyz=(0.0, 0.062, 0.068)),
            material=dark_trim,
            name="filter_bracket",
        )
        filter_part.inertial = Inertial.from_geometry(
            Box((0.30, 0.205, 0.116)),
            mass=0.42,
            origin=Origin(xyz=(0.0, 0.020, 0.058)),
        )
        model.articulation(
            f"hood_body_to_{filter_name}",
            ArticulationType.FIXED,
            parent=hood_body,
            child=filter_part,
            origin=Origin(xyz=(x_center, 0.020, 0.021)),
        )

    _build_filter("left_filter", -0.165)
    _build_filter("right_filter", 0.165)

    def _build_knob(name: str) -> None:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.022, length=0.026),
            origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=charcoal,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.024, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=dark_trim,
            name="knob_skirt",
        )
        knob.visual(
            Box((0.004, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, -0.024, 0.014)),
            material=satin_steel,
            name="knob_marker",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.024, length=0.026),
            mass=0.18,
            origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        )

    _build_knob("left_knob")
    _build_knob("right_knob")

    center_button = model.part("center_button")
    center_button.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="button_cap",
    )
    center_button.visual(
        Box((pod_opening, 0.018, pod_opening)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=dark_trim,
        name="button_stem",
    )
    center_button.inertial = Inertial.from_geometry(
        Box((pod_opening, 0.026, pod_opening)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
    )

    model.articulation(
        "hood_body_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=hood_body,
        child="left_knob",
        origin=Origin(xyz=(-0.315, front_y, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=10.0),
    )
    model.articulation(
        "hood_body_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=hood_body,
        child="right_knob",
        origin=Origin(xyz=(0.315, front_y, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=10.0),
    )
    model.articulation(
        "hood_body_to_center_button",
        ArticulationType.PRISMATIC,
        parent=hood_body,
        child=center_button,
        origin=Origin(xyz=(0.0, front_y - pod_outer_depth, pod_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    left_filter = object_model.get_part("left_filter")
    right_filter = object_model.get_part("right_filter")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    center_button = object_model.get_part("center_button")

    left_joint = object_model.get_articulation("hood_body_to_left_knob")
    right_joint = object_model.get_articulation("hood_body_to_right_knob")
    button_joint = object_model.get_articulation("hood_body_to_center_button")

    hood_body.get_visual("canopy_top")
    hood_body.get_visual("control_strip")
    hood_body.get_visual("chimney_front")
    left_filter.get_visual("filter_panel")
    right_filter.get_visual("filter_panel")
    left_knob.get_visual("knob_body")
    right_knob.get_visual("knob_body")
    center_button.get_visual("button_cap")

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

    hood_aabb = ctx.part_world_aabb(hood_body)
    assert hood_aabb is not None
    hood_width = hood_aabb[1][0] - hood_aabb[0][0]
    hood_depth = hood_aabb[1][1] - hood_aabb[0][1]
    hood_height = hood_aabb[1][2] - hood_aabb[0][2]
    ctx.check(
        "hood_realistic_size",
        0.82 <= hood_width <= 0.96 and 0.46 <= hood_depth <= 0.55 and 0.84 <= hood_height <= 0.92,
        f"Observed hood size {(hood_width, hood_depth, hood_height)}",
    )

    ctx.check(
        "knob_joints_continuous_front_to_back",
        left_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_joint.axis == (0.0, 1.0, 0.0)
        and right_joint.axis == (0.0, 1.0, 0.0),
        f"Knob joint definitions: left={left_joint.articulation_type}/{left_joint.axis}, "
        f"right={right_joint.articulation_type}/{right_joint.axis}",
    )
    ctx.check(
        "button_joint_prismatic_front_to_back",
        button_joint.articulation_type == ArticulationType.PRISMATIC and button_joint.axis == (0.0, 1.0, 0.0),
        f"Button joint definition: {button_joint.articulation_type}/{button_joint.axis}",
    )

    left_pos = ctx.part_world_position(left_knob)
    right_pos = ctx.part_world_position(right_knob)
    button_rest_pos = ctx.part_world_position(center_button)
    assert left_pos is not None
    assert right_pos is not None
    assert button_rest_pos is not None
    ctx.check(
        "controls_spread_across_front_strip",
        left_pos[0] < -0.25 and right_pos[0] > 0.25 and abs(button_rest_pos[0]) < 0.01,
        f"Control positions: left={left_pos}, button={button_rest_pos}, right={right_pos}",
    )
    ctx.check(
        "controls_located_on_front",
        left_pos[1] < -0.24 and right_pos[1] < -0.24 and button_rest_pos[1] < -0.26,
        f"Front positions: left={left_pos[1]}, button={button_rest_pos[1]}, right={right_pos[1]}",
    )

    ctx.expect_contact(left_knob, hood_body)
    ctx.expect_contact(right_knob, hood_body)
    ctx.expect_contact(center_button, hood_body)
    ctx.expect_contact(left_filter, hood_body)
    ctx.expect_contact(right_filter, hood_body)

    with ctx.pose({left_joint: 1.4, right_joint: -1.1}):
        ctx.expect_contact(left_knob, hood_body, name="left_knob_rotated_contact")
        ctx.expect_contact(right_knob, hood_body, name="right_knob_rotated_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_rotated_no_floating")

    limits = button_joint.motion_limits
    assert limits is not None and limits.lower is not None and limits.upper is not None
    with ctx.pose({button_joint: limits.lower}):
        ctx.expect_contact(center_button, hood_body, name="button_rest_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="button_rest_no_overlap")
        ctx.fail_if_isolated_parts(name="button_rest_no_floating")
    with ctx.pose({button_joint: limits.upper}):
        button_pressed_pos = ctx.part_world_position(center_button)
        assert button_pressed_pos is not None
        ctx.check(
            "button_moves_inward_when_pressed",
            button_pressed_pos[1] > button_rest_pos[1] + 0.0055,
            f"Button y travel: rest={button_rest_pos[1]}, pressed={button_pressed_pos[1]}",
        )
        ctx.expect_contact(center_button, hood_body, name="button_pressed_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="button_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="button_pressed_no_floating")

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
