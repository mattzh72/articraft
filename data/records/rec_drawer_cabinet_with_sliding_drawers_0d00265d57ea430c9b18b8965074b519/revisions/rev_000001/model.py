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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


COLUMN_CENTERS_Y = (0.40, 0.0, -0.40)
ROW_CENTERS_Z = (0.14, -0.14)
DRAWER_NAMES = (
    "upper_left_drawer",
    "upper_center_drawer",
    "upper_right_drawer",
    "lower_left_drawer",
    "lower_center_drawer",
    "lower_right_drawer",
)
LEG_NAMES = (
    "front_left_leg",
    "front_right_leg",
    "rear_left_leg",
    "rear_right_leg",
)


def _build_pull_handle():
    pull_path = [
        (0.0, -0.078, 0.0),
        (0.020, -0.060, 0.0),
        (0.034, -0.026, 0.0),
        (0.038, 0.0, 0.0),
        (0.034, 0.026, 0.0),
        (0.020, 0.060, 0.0),
        (0.0, 0.078, 0.0),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            pull_path,
            radius=0.0055,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
        "drawer_pull_handle",
    )


def _drawer_layout():
    specs = []
    for row_name, z in zip(("upper", "lower"), ROW_CENTERS_Z):
        for col_name, y in zip(("left", "center", "right"), COLUMN_CENTERS_Y):
            specs.append((f"{row_name}_{col_name}_drawer", y, z))
    return specs


def _add_leg(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    socket_x: float,
    socket_y: float,
    steel,
    dark_steel,
    rubber,
) -> None:
    sleeve_outer = 0.070
    sleeve_inner = 0.050
    sleeve_wall = 0.010
    sleeve_height = 0.070
    socket_center_z = -0.365
    body.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(socket_x, socket_y + (sleeve_inner + sleeve_wall) * 0.5, socket_center_z)),
        material=steel,
        name=f"{name}_socket_left_wall",
    )
    body.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(socket_x, socket_y - (sleeve_inner + sleeve_wall) * 0.5, socket_center_z)),
        material=steel,
        name=f"{name}_socket_right_wall",
    )
    body.visual(
        Box((sleeve_wall, sleeve_inner, sleeve_height)),
        origin=Origin(xyz=(socket_x + (sleeve_inner + sleeve_wall) * 0.5, socket_y, socket_center_z)),
        material=steel,
        name=f"{name}_socket_front_wall",
    )
    body.visual(
        Box((sleeve_wall, sleeve_inner, sleeve_height)),
        origin=Origin(xyz=(socket_x - (sleeve_inner + sleeve_wall) * 0.5, socket_y, socket_center_z)),
        material=steel,
        name=f"{name}_socket_back_wall",
    )

    leg = model.part(name)
    leg.visual(
        Box((0.046, 0.046, 0.160)),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=steel,
        name="leg_post",
    )
    leg.visual(
        Box((0.058, 0.058, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=dark_steel,
        name="stop_collar",
    )
    leg.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        material=dark_steel,
        name="threaded_stem",
    )
    leg.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.214)),
        material=steel,
        name="lock_nut",
    )
    leg.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=rubber,
        name="foot_pad",
    )
    leg.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, 0.240)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
    )

    model.articulation(
        f"cabinet_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=leg,
        origin=Origin(xyz=(socket_x, socket_y, -0.330)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.050,
            lower=0.0,
            upper=0.040,
        ),
    )


def _add_drawer(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    center_y: float,
    center_z: float,
    handle_mesh,
    body_steel,
    slide_steel,
    handle_steel,
) -> None:
    rail_center_x = -0.020
    outer_slide_size = (0.460, 0.026, 0.028)
    outer_slide_offset_y = 0.168
    inner_slide_offset_y = 0.149

    body.visual(
        Box(outer_slide_size),
        origin=Origin(xyz=(rail_center_x, center_y + outer_slide_offset_y, center_z - 0.030)),
        material=slide_steel,
        name=f"{name}_left_outer_slide",
    )
    body.visual(
        Box(outer_slide_size),
        origin=Origin(xyz=(rail_center_x, center_y - outer_slide_offset_y, center_z - 0.030)),
        material=slide_steel,
        name=f"{name}_right_outer_slide",
    )

    if "left_drawer" in name:
        body.visual(
            Box((0.460, 0.015, 0.018)),
            origin=Origin(xyz=(rail_center_x, center_y + 0.5885 - 0.4000, center_z - 0.030)),
            material=slide_steel,
            name=f"{name}_left_mount_plate",
        )
    if "right_drawer" in name:
        body.visual(
            Box((0.460, 0.015, 0.018)),
            origin=Origin(xyz=(rail_center_x, center_y - 0.5885 + 0.4000, center_z - 0.030)),
            material=slide_steel,
            name=f"{name}_right_mount_plate",
        )

    drawer = model.part(name)
    drawer.visual(
        Box((0.018, 0.330, 0.222)),
        origin=Origin(xyz=(0.359, 0.0, 0.0)),
        material=body_steel,
        name="front_panel",
    )
    drawer.visual(
        Box((0.452, 0.286, 0.004)),
        origin=Origin(xyz=(0.124, 0.0, -0.080)),
        material=body_steel,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.452, 0.008, 0.090)),
        origin=Origin(xyz=(0.124, 0.139, -0.035)),
        material=body_steel,
        name="left_wall",
    )
    drawer.visual(
        Box((0.452, 0.008, 0.090)),
        origin=Origin(xyz=(0.124, -0.139, -0.035)),
        material=body_steel,
        name="right_wall",
    )
    drawer.visual(
        Box((0.012, 0.286, 0.090)),
        origin=Origin(xyz=(-0.096, 0.0, -0.035)),
        material=body_steel,
        name="back_wall",
    )
    drawer.visual(
        Box((0.520, 0.012, 0.022)),
        origin=Origin(xyz=(-0.030, inner_slide_offset_y, -0.030)),
        material=slide_steel,
        name="left_slide",
    )
    drawer.visual(
        Box((0.520, 0.012, 0.022)),
        origin=Origin(xyz=(-0.030, -inner_slide_offset_y, -0.030)),
        material=slide_steel,
        name="right_slide",
    )
    drawer.visual(
        handle_mesh,
        origin=Origin(xyz=(0.368, 0.0, 0.0)),
        material=handle_steel,
        name="pull_handle",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.470, 0.330, 0.222)),
        mass=4.8,
        origin=Origin(xyz=(0.120, 0.0, -0.020)),
    )

    model.articulation(
        f"cabinet_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.020, center_y, center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.350,
            lower=0.0,
            upper=0.420,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_reagent_storage_cabinet")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    slide_steel = model.material("slide_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((0.700, 1.260, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.3325)),
        material=stainless,
        name="worktop",
    )
    body.visual(
        Box((0.620, 1.192, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=stainless,
        name="bottom_deck",
    )
    body.visual(
        Box((0.640, 0.024, 0.640)),
        origin=Origin(xyz=(0.0, 0.608, 0.0)),
        material=stainless,
        name="left_side_wall",
    )
    body.visual(
        Box((0.640, 0.024, 0.640)),
        origin=Origin(xyz=(0.0, -0.608, 0.0)),
        material=stainless,
        name="right_side_wall",
    )
    body.visual(
        Box((0.020, 1.192, 0.640)),
        origin=Origin(xyz=(-0.320, 0.0, 0.0)),
        material=stainless,
        name="back_panel",
    )
    body.visual(
        Box((0.620, 1.192, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="center_shelf",
    )
    body.visual(
        Box((0.620, 0.064, 0.640)),
        origin=Origin(xyz=(0.0, 0.200, 0.0)),
        material=stainless,
        name="left_partition",
    )
    body.visual(
        Box((0.620, 0.064, 0.640)),
        origin=Origin(xyz=(0.0, -0.200, 0.0)),
        material=stainless,
        name="right_partition",
    )
    body.visual(
        Box((0.020, 1.240, 0.060)),
        origin=Origin(xyz=(0.320, 0.0, 0.290)),
        material=stainless,
        name="top_face_rail",
    )
    body.visual(
        Box((0.020, 1.240, 0.040)),
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        material=stainless,
        name="mid_face_rail",
    )
    body.visual(
        Box((0.020, 1.240, 0.080)),
        origin=Origin(xyz=(0.320, 0.0, -0.280)),
        material=stainless,
        name="bottom_face_rail",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.700, 1.260, 0.700)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    handle_mesh = _build_pull_handle()

    for drawer_name, center_y, center_z in _drawer_layout():
        _add_drawer(
            model,
            body,
            name=drawer_name,
            center_y=center_y,
            center_z=center_z,
            handle_mesh=handle_mesh,
            body_steel=stainless,
            slide_steel=slide_steel,
            handle_steel=brushed_steel,
        )

    for leg_name, socket_x, socket_y in (
        ("front_left_leg", 0.270, 0.540),
        ("front_right_leg", 0.270, -0.540),
        ("rear_left_leg", -0.270, 0.540),
        ("rear_right_leg", -0.270, -0.540),
    ):
        _add_leg(
            model,
            body,
            name=leg_name,
            socket_x=socket_x,
            socket_y=socket_y,
            steel=brushed_steel,
            dark_steel=dark_steel,
            rubber=rubber,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
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

    drawer_depths = []
    for drawer_name in DRAWER_NAMES:
        drawer = object_model.get_part(drawer_name)
        slide_joint = object_model.get_articulation(f"cabinet_to_{drawer_name}")
        limits = slide_joint.motion_limits

        ctx.check(
            f"{drawer_name} uses front-to-back prismatic travel",
            tuple(slide_joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={slide_joint.axis}",
        )
        ctx.check(
            f"{drawer_name} has full-extension travel",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= 0.40,
            details=f"limits={limits}",
        )
        ctx.expect_contact(
            body,
            drawer,
            elem_a=f"{drawer_name}_left_outer_slide",
            elem_b="left_slide",
            name=f"{drawer_name} left slide is mounted on its cabinet rail",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide_joint: limits.upper if limits is not None and limits.upper is not None else 0.0}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="left_slide",
                elem_b=f"{drawer_name}_left_outer_slide",
                min_overlap=0.08,
                name=f"{drawer_name} retains rail engagement at full extension",
            )
            open_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"{drawer_name} opens outward from the cabinet face",
            rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 0.35,
            details=f"rest={rest_pos}, open={open_pos}",
        )

        tray_aabb = ctx.part_element_world_aabb(drawer, elem="tray_bottom")
        if tray_aabb is not None:
            drawer_depths.append(tray_aabb[1][0] - tray_aabb[0][0])

    ctx.check(
        "all six drawers share the same tray depth",
        len(drawer_depths) == 6 and max(drawer_depths) - min(drawer_depths) < 1e-6,
        details=f"depths={drawer_depths}",
    )

    for leg_name in LEG_NAMES:
        leg = object_model.get_part(leg_name)
        leg_joint = object_model.get_articulation(f"cabinet_to_{leg_name}")
        limits = leg_joint.motion_limits

        ctx.check(
            f"{leg_name} uses vertical leveling travel",
            tuple(leg_joint.axis) == (0.0, 0.0, -1.0),
            details=f"axis={leg_joint.axis}",
        )
        ctx.check(
            f"{leg_name} has realistic adjustment range",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and 0.03 <= limits.upper <= 0.06,
            details=f"limits={limits}",
        )
        ctx.expect_contact(
            body,
            leg,
            elem_b="stop_collar",
            name=f"{leg_name} is seated in its socket",
        )

        rest_pos = ctx.part_world_position(leg)
        with ctx.pose({leg_joint: limits.upper if limits is not None and limits.upper is not None else 0.0}):
            extended_pos = ctx.part_world_position(leg)
        ctx.check(
            f"{leg_name} extends downward when adjusted",
            rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.03,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
