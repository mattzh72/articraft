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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _build_control_rail_mesh():
    def section(x: float) -> list[tuple[float, float, float]]:
        return [
            (x, 0.244, 0.772),
            (x, 0.315, 0.772),
            (x, 0.315, 0.821),
            (x, 0.304, 0.843),
            (x, 0.282, 0.862),
            (x, 0.244, 0.862),
        ]

    return section_loft([section(-0.35), section(0.35)])


def _build_knob_mesh(radius: float, length: float):
    return LatheGeometry(
        [
            (0.0, 0.0),
            (radius * 0.90, 0.0),
            (radius * 1.00, length * 0.12),
            (radius * 1.00, length * 0.70),
            (radius * 0.92, length * 0.86),
            (radius * 0.70, length),
            (0.0, length),
        ],
        segments=48,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_in_kitchen_range", assets=ASSETS)

    enamel = model.material("enamel", rgba=(0.90, 0.91, 0.93, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.09, 0.10, 0.12, 0.95))
    cavity_steel = model.material("cavity_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    door_glass = model.material("door_glass", rgba=(0.16, 0.20, 0.24, 0.45))
    knob_finish = model.material("knob_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    burner_mark = model.material("burner_mark", rgba=(0.24, 0.25, 0.27, 0.90))

    width = 0.76
    depth = 0.63
    body_height = 0.90
    cooktop_thickness = 0.012

    x_cylinder_rpy = (0.0, math.pi / 2.0, 0.0)
    y_cylinder_rpy = (-math.pi / 2.0, 0.0, 0.0)
    control_rail_mesh = mesh_from_geometry(
        _build_control_rail_mesh(),
        ASSETS.mesh_path("range_control_rail.obj"),
    )
    small_knob_mesh = mesh_from_geometry(
        _build_knob_mesh(0.024, 0.040),
        ASSETS.mesh_path("range_small_knob.obj"),
    )
    large_knob_mesh = mesh_from_geometry(
        _build_knob_mesh(0.031, 0.046),
        ASSETS.mesh_path("range_large_knob.obj"),
    )

    body = model.part("range_body")
    body.visual(
        Box((0.03, depth, body_height)),
        origin=Origin(xyz=(-0.365, 0.0, body_height * 0.5)),
        material=enamel,
        name="left_side_panel",
    )
    body.visual(
        Box((0.03, depth, body_height)),
        origin=Origin(xyz=(0.365, 0.0, body_height * 0.5)),
        material=enamel,
        name="right_side_panel",
    )
    body.visual(
        Box((0.70, 0.02, body_height)),
        origin=Origin(xyz=(0.0, -0.305, body_height * 0.5)),
        material=enamel,
        name="rear_panel",
    )
    body.visual(
        Box((0.70, 0.15, 0.11)),
        origin=Origin(xyz=(0.0, 0.195, 0.055)),
        material=dark_trim,
        name="toe_kick",
    )
    body.visual(
        Box((0.69, 0.03, 0.014)),
        origin=Origin(xyz=(0.0, 0.255, 0.103)),
        material=dark_trim,
        name="hinge_saddle",
    )
    body.visual(
        Box((0.65, 0.54, 0.015)),
        origin=Origin(xyz=(0.0, -0.0225, 0.1175)),
        material=cavity_steel,
        name="cavity_floor",
    )
    body.visual(
        Box((0.65, 0.54, 0.015)),
        origin=Origin(xyz=(0.0, -0.0225, 0.5925)),
        material=cavity_steel,
        name="cavity_ceiling",
    )
    body.visual(
        Box((0.005, 0.54, 0.46)),
        origin=Origin(xyz=(-0.3225, -0.0225, 0.355)),
        material=cavity_steel,
        name="cavity_left_liner",
    )
    body.visual(
        Box((0.005, 0.54, 0.46)),
        origin=Origin(xyz=(0.3225, -0.0225, 0.355)),
        material=cavity_steel,
        name="cavity_right_liner",
    )
    body.visual(
        Box((0.65, 0.005, 0.46)),
        origin=Origin(xyz=(0.0, -0.2925, 0.355)),
        material=cavity_steel,
        name="cavity_back_liner",
    )
    body.visual(
        Box((0.70, 0.562, 0.172)),
        origin=Origin(xyz=(0.0, -0.024, 0.686)),
        material=enamel,
        name="upper_oven_shell",
    )
    body.visual(
        control_rail_mesh,
        origin=Origin(),
        material=enamel,
        name="control_rail",
    )
    body.visual(
        Box((width - 0.002, depth + 0.002, cooktop_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height + cooktop_thickness * 0.5)),
        material=cooktop_glass,
        name="glass_cooktop",
    )
    for zone_x, zone_y, zone_r in (
        (-0.19, 0.14, 0.095),
        (0.19, 0.14, 0.095),
        (-0.19, -0.12, 0.080),
        (0.19, -0.12, 0.080),
        (0.0, 0.005, 0.110),
    ):
        body.visual(
            Cylinder(radius=zone_r, length=0.0012),
            origin=Origin(
                xyz=(zone_x, zone_y, body_height + cooktop_thickness - 0.0006)
            ),
            material=burner_mark,
            name=f"burner_zone_{zone_x:+.2f}_{zone_y:+.2f}",
        )
    body.inertial = Inertial.from_geometry(
        Box((width, depth, body_height + cooktop_thickness)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, (body_height + cooktop_thickness) * 0.5)),
    )

    oven_door = model.part("oven_door")
    oven_door.visual(
        Cylinder(radius=0.014, length=0.63),
        origin=Origin(rpy=x_cylinder_rpy),
        material=dark_trim,
        name="hinge_rod",
    )
    oven_door.visual(
        Box((0.69, 0.046, 0.49)),
        origin=Origin(xyz=(0.0, 0.023, 0.245)),
        material=enamel,
        name="door_panel",
    )
    oven_door.visual(
        Box((0.42, 0.006, 0.23)),
        origin=Origin(xyz=(0.0, 0.040, 0.285)),
        material=door_glass,
        name="door_window",
    )
    oven_door.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(0.0, 0.084, 0.435), rpy=x_cylinder_rpy),
        material=handle_finish,
        name="door_handle_bar",
    )
    oven_door.visual(
        Box((0.02, 0.045, 0.02)),
        origin=Origin(xyz=(-0.17, 0.061, 0.435)),
        material=handle_finish,
        name="door_handle_left_post",
    )
    oven_door.visual(
        Box((0.02, 0.045, 0.02)),
        origin=Origin(xyz=(0.17, 0.061, 0.435)),
        material=handle_finish,
        name="door_handle_right_post",
    )
    oven_door.inertial = Inertial.from_geometry(
        Box((0.69, 0.10, 0.50)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.03, 0.24)),
    )
    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=oven_door,
        origin=Origin(xyz=(0.0, 0.270, 0.124)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-1.45,
            upper=0.0,
        ),
    )

    def add_knob(
        part_name: str,
        joint_name: str,
        *,
        x: float,
        z: float,
        radius: float,
        length: float,
        mass: float,
        knob_mesh,
    ) -> None:
        knob = model.part(part_name)
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=y_cylinder_rpy),
            material=knob_finish,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=radius * 0.72, length=0.010),
            origin=Origin(xyz=(0.0, length - 0.005, 0.0), rpy=y_cylinder_rpy),
            material=dark_trim,
            name="knob_face",
        )
        knob.visual(
            Box((0.004, 0.008, radius * 1.15)),
            origin=Origin(xyz=(0.0, length - 0.004, 0.0)),
            material=handle_finish,
            name="knob_indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=length),
            mass=mass,
            origin=Origin(xyz=(0.0, length * 0.5, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, 0.315, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=8.0),
        )

    add_knob(
        "left_outer_burner_knob",
        "body_to_left_outer_burner_knob",
        x=-0.27,
        z=0.816,
        radius=0.024,
        length=0.040,
        mass=0.10,
        knob_mesh=small_knob_mesh,
    )
    add_knob(
        "left_inner_burner_knob",
        "body_to_left_inner_burner_knob",
        x=-0.14,
        z=0.816,
        radius=0.024,
        length=0.040,
        mass=0.10,
        knob_mesh=small_knob_mesh,
    )
    add_knob(
        "center_oven_knob",
        "body_to_center_oven_knob",
        x=0.0,
        z=0.816,
        radius=0.031,
        length=0.046,
        mass=0.14,
        knob_mesh=large_knob_mesh,
    )
    add_knob(
        "right_inner_burner_knob",
        "body_to_right_inner_burner_knob",
        x=0.14,
        z=0.816,
        radius=0.024,
        length=0.040,
        mass=0.10,
        knob_mesh=small_knob_mesh,
    )
    add_knob(
        "right_outer_burner_knob",
        "body_to_right_outer_burner_knob",
        x=0.27,
        z=0.816,
        radius=0.024,
        length=0.040,
        mass=0.10,
        knob_mesh=small_knob_mesh,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("range_body")
    oven_door = object_model.get_part("oven_door")
    left_outer = object_model.get_part("left_outer_burner_knob")
    left_inner = object_model.get_part("left_inner_burner_knob")
    center_knob = object_model.get_part("center_oven_knob")
    right_inner = object_model.get_part("right_inner_burner_knob")
    right_outer = object_model.get_part("right_outer_burner_knob")

    door_hinge = object_model.get_articulation("body_to_oven_door")
    knob_joints = [
        object_model.get_articulation("body_to_left_outer_burner_knob"),
        object_model.get_articulation("body_to_left_inner_burner_knob"),
        object_model.get_articulation("body_to_center_oven_knob"),
        object_model.get_articulation("body_to_right_inner_burner_knob"),
        object_model.get_articulation("body_to_right_outer_burner_knob"),
    ]
    knob_parts = [left_outer, left_inner, center_knob, right_inner, right_outer]

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

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_has_geometry", "range_body has no measurable geometry.")
    else:
        width_measured = body_aabb[1][0] - body_aabb[0][0]
        depth_measured = body_aabb[1][1] - body_aabb[0][1]
        height_measured = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "range_realistic_width",
            0.74 <= width_measured <= 0.78,
            f"Expected realistic slide-in range width, got {width_measured:.3f} m.",
        )
        ctx.check(
            "range_realistic_depth",
            0.62 <= depth_measured <= 0.65,
            f"Expected realistic range depth, got {depth_measured:.3f} m.",
        )
        ctx.check(
            "range_realistic_height",
            0.90 <= height_measured <= 0.93,
            f"Expected realistic cooktop height, got {height_measured:.3f} m.",
        )

    cooktop_aabb = ctx.part_element_world_aabb(body, elem="glass_cooktop")
    rail_aabb = ctx.part_element_world_aabb(body, elem="control_rail")
    if cooktop_aabb is not None and rail_aabb is not None:
        ctx.check(
            "cooktop_above_control_rail",
            cooktop_aabb[0][2] >= rail_aabb[1][2] + 0.03,
            "The glass cooktop should sit clearly above the front control rail.",
        )

    ctx.check(
        "oven_door_is_revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"Expected revolute oven door, got {door_hinge.articulation_type}.",
    )
    ctx.check(
        "oven_door_axis_horizontal",
        abs(door_hinge.axis[0]) > 0.99
        and abs(door_hinge.axis[1]) < 1e-6
        and abs(door_hinge.axis[2]) < 1e-6,
        f"Expected oven door hinge on x-axis, got {door_hinge.axis}.",
    )

    door_limits = door_hinge.motion_limits
    if door_limits is None or door_limits.lower is None or door_limits.upper is None:
        ctx.fail("oven_door_has_limits", "Oven door is missing finite motion limits.")
    else:
        ctx.check(
            "oven_door_limit_span",
            door_limits.lower <= -1.25 and abs(door_limits.upper) <= 1e-9,
            f"Expected a downward-opening door range near [-1.45, 0], got [{door_limits.lower}, {door_limits.upper}].",
        )

    for knob_joint in knob_joints:
        ctx.check(
            f"{knob_joint.name}_continuous",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{knob_joint.name} should be continuous.",
        )
        ctx.check(
            f"{knob_joint.name}_front_to_back_axis",
            abs(knob_joint.axis[1]) > 0.99
            and abs(knob_joint.axis[0]) < 1e-6
            and abs(knob_joint.axis[2]) < 1e-6,
            f"{knob_joint.name} should rotate on the front-to-back y-axis, got {knob_joint.axis}.",
        )

    for knob_part, knob_joint in zip(knob_parts, knob_joints):
        with ctx.pose({knob_joint: 1.1}):
            ctx.expect_contact(
                knob_part,
                body,
                elem_a="knob_body",
                elem_b="control_rail",
                contact_tol=5e-4,
                name=f"{knob_part.name}_mounted_to_control_rail",
            )
            ctx.expect_gap(
                knob_part,
                body,
                axis="y",
                positive_elem="knob_body",
                negative_elem="control_rail",
                max_gap=5e-4,
                max_penetration=0.0,
                name=f"{knob_part.name}_flush_to_control_rail",
            )

    side_knob_aabb = ctx.part_element_world_aabb(left_outer, elem="knob_body")
    center_knob_aabb = ctx.part_element_world_aabb(center_knob, elem="knob_body")
    if side_knob_aabb is not None and center_knob_aabb is not None:
        side_diameter = side_knob_aabb[1][0] - side_knob_aabb[0][0]
        center_diameter = center_knob_aabb[1][0] - center_knob_aabb[0][0]
        ctx.check(
            "center_knob_larger_than_burner_knobs",
            center_diameter >= side_diameter + 0.010,
            f"Expected larger center knob, got side {side_diameter:.3f} m and center {center_diameter:.3f} m.",
        )

    ctx.expect_contact(
        oven_door,
        body,
        elem_a="hinge_rod",
        elem_b="hinge_saddle",
        contact_tol=5e-4,
        name="oven_door_hinge_contact_closed",
    )
    ctx.expect_overlap(
        oven_door,
        body,
        axes="x",
        min_overlap=0.68,
        elem_a="door_panel",
        elem_b="control_rail",
        name="oven_door_broad_front_alignment",
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
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        closed_handle_aabb = ctx.part_element_world_aabb(oven_door, elem="door_handle_bar")
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_open_no_floating")
            ctx.expect_contact(
                oven_door,
                body,
                elem_a="hinge_rod",
                elem_b="hinge_saddle",
                contact_tol=5e-4,
                name="oven_door_hinge_contact_open",
            )
            open_handle_aabb = ctx.part_element_world_aabb(oven_door, elem="door_handle_bar")
            if closed_handle_aabb is not None and open_handle_aabb is not None:
                ctx.check(
                    "oven_door_opens_outward",
                    open_handle_aabb[1][1] >= closed_handle_aabb[1][1] + 0.30,
                    "Open oven door handle should move far forward of the closed front.",
                )
                ctx.check(
                    "oven_door_drops_down",
                    open_handle_aabb[1][2] <= closed_handle_aabb[0][2] - 0.20,
                    "Open oven door handle should drop well below its closed height.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
