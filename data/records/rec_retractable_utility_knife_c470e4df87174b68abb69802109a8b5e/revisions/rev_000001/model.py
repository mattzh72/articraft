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
)


def _extruded_xz_profile_mesh(
    profile_xz: list[tuple[float, float]],
    *,
    thickness: float,
    y_offset: float,
    name: str,
):
    geom = ExtrudeGeometry.centered(profile_xz, thickness)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(0.0, y_offset, 0.0)
    return mesh_from_geometry(geom, name)


def _blade_mesh(name: str):
    blade_profile = [
        (0.030, -0.0078),
        (0.050, -0.0078),
        (0.058, -0.0048),
        (0.080, -0.0024),
        (0.088, 0.0),
        (0.080, 0.0024),
        (0.058, 0.0048),
        (0.050, 0.0078),
        (0.030, 0.0078),
    ]
    geom = ExtrudeGeometry.centered(blade_profile, 0.0012)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_knife_with_spare_blade_tray")

    body_gray = model.material("body_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.10, 1.0))
    slider_orange = model.material("slider_orange", rgba=(0.88, 0.42, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))

    handle_body = model.part("handle_body")

    handle_width = 0.024
    side_wall = 0.0025
    side_y = handle_width / 2.0 - side_wall / 2.0

    side_outline = [
        (-0.078, -0.0115),
        (-0.070, -0.0152),
        (-0.030, -0.0160),
        (0.005, -0.0146),
        (0.034, -0.0138),
        (0.055, -0.0118),
        (0.070, -0.0082),
        (0.082, -0.0022),
        (0.078, 0.0100),
        (0.060, 0.0148),
        (0.018, 0.0158),
        (-0.038, 0.0152),
        (-0.069, 0.0132),
        (-0.078, 0.0102),
    ]
    left_shell = _extruded_xz_profile_mesh(
        side_outline,
        thickness=side_wall,
        y_offset=-side_y,
        name="left_shell",
    )
    right_shell = _extruded_xz_profile_mesh(
        side_outline,
        thickness=side_wall,
        y_offset=side_y,
        name="right_shell",
    )
    handle_body.visual(left_shell, material=body_gray, name="left_shell")
    handle_body.visual(right_shell, material=body_gray, name="right_shell")

    grip_outline = [
        (-0.060, -0.0100),
        (-0.025, -0.0124),
        (0.012, -0.0113),
        (0.035, -0.0080),
        (0.040, 0.0000),
        (0.026, 0.0100),
        (-0.006, 0.0116),
        (-0.042, 0.0102),
        (-0.060, 0.0068),
    ]
    grip_thickness = 0.0014
    left_grip = _extruded_xz_profile_mesh(
        grip_outline,
        thickness=grip_thickness,
        y_offset=-(handle_width / 2.0 + grip_thickness / 2.0 - 0.0002),
        name="left_grip",
    )
    right_grip = _extruded_xz_profile_mesh(
        grip_outline,
        thickness=grip_thickness,
        y_offset=handle_width / 2.0 + grip_thickness / 2.0 - 0.0002,
        name="right_grip",
    )
    handle_body.visual(left_grip, material=grip_black, name="left_grip")
    handle_body.visual(right_grip, material=grip_black, name="right_grip")

    handle_body.visual(
        Box((0.118, 0.0205, 0.0055)),
        origin=Origin(xyz=(-0.004, 0.0, -0.01325)),
        material=body_gray,
        name="lower_spine",
    )
    handle_body.visual(
        Box((0.034, 0.013, 0.0035)),
        origin=Origin(xyz=(0.016, 0.0, -0.00875)),
        material=body_gray,
        name="front_inner_rail",
    )
    handle_body.visual(
        Box((0.116, 0.007, 0.0025)),
        origin=Origin(xyz=(0.016, -0.00775, 0.01475)),
        material=body_gray,
        name="upper_rail_left",
    )
    handle_body.visual(
        Box((0.116, 0.007, 0.0025)),
        origin=Origin(xyz=(0.016, 0.00775, 0.01475)),
        material=body_gray,
        name="upper_rail_right",
    )
    handle_body.visual(
        Box((0.022, 0.0205, 0.004)),
        origin=Origin(xyz=(0.066, 0.0, 0.0105)),
        material=body_gray,
        name="nose_upper",
    )
    handle_body.visual(
        Box((0.028, 0.0205, 0.004)),
        origin=Origin(xyz=(0.062, 0.0, -0.0105)),
        material=body_gray,
        name="nose_lower",
    )
    handle_body.visual(
        Box((0.012, 0.0205, 0.005)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0125)),
        material=body_gray,
        name="tail_top_bridge",
    )
    handle_body.visual(
        Cylinder(radius=0.0015, length=0.005),
        origin=Origin(xyz=(-0.076, -0.0085, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_gray,
        name="hinge_lug_left",
    )
    handle_body.visual(
        Cylinder(radius=0.0015, length=0.005),
        origin=Origin(xyz=(-0.076, 0.0085, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_gray,
        name="hinge_lug_right",
    )
    handle_body.inertial = Inertial.from_geometry(
        Box((0.165, 0.024, 0.032)),
        mass=0.34,
    )

    blade_carriage = model.part("blade_carriage")
    blade_carriage.visual(
        Box((0.046, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=body_gray,
        name="carriage_core",
    )
    blade_carriage.visual(
        Box((0.020, 0.008, 0.004)),
        origin=Origin(xyz=(0.024, 0.0, -0.001)),
        material=steel,
        name="blade_clamp",
    )
    blade_carriage.visual(
        Box((0.009, 0.003, 0.011)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0075)),
        material=slider_orange,
        name="slider_stem",
    )
    blade_carriage.visual(
        Box((0.018, 0.0085, 0.007)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0165)),
        material=slider_orange,
        name="thumb_slider",
    )
    blade_carriage.visual(
        _blade_mesh("utility_blade"),
        material=steel,
        name="blade",
    )
    blade_carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.010, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(0.020, 0.0, 0.004)),
    )

    spare_tray = model.part("spare_tray")
    spare_tray.visual(
        Cylinder(radius=0.0014, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_gray,
        name="tray_knuckle",
    )
    spare_tray.visual(
        Box((0.004, 0.014, 0.022)),
        origin=Origin(xyz=(0.002, 0.0, 0.0124)),
        material=body_gray,
        name="tray_cap",
    )
    spare_tray.visual(
        Box((0.012, 0.002, 0.017)),
        origin=Origin(xyz=(0.010, -0.0055, 0.0125)),
        material=body_gray,
        name="tray_wall_left",
    )
    spare_tray.visual(
        Box((0.012, 0.002, 0.017)),
        origin=Origin(xyz=(0.010, 0.0055, 0.0125)),
        material=body_gray,
        name="tray_wall_right",
    )
    spare_tray.visual(
        Box((0.012, 0.011, 0.002)),
        origin=Origin(xyz=(0.010, 0.0, 0.0210)),
        material=body_gray,
        name="tray_end_wall",
    )
    spare_tray.visual(
        Box((0.0016, 0.007, 0.014)),
        origin=Origin(xyz=(0.0048, 0.0, 0.013)),
        material=steel,
        name="spare_blades",
    )
    spare_tray.inertial = Inertial.from_geometry(
        Box((0.020, 0.016, 0.024)),
        mass=0.03,
        origin=Origin(xyz=(0.007, 0.0, 0.013)),
    )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle_body,
        child=blade_carriage,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "handle_to_spare_tray",
        ArticulationType.REVOLUTE,
        parent=handle_body,
        child=spare_tray,
        origin=Origin(xyz=(-0.078, 0.0, -0.014)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_body = object_model.get_part("handle_body")
    blade_carriage = object_model.get_part("blade_carriage")
    spare_tray = object_model.get_part("spare_tray")
    slide = object_model.get_articulation("handle_to_blade_carriage")
    tray_hinge = object_model.get_articulation("handle_to_spare_tray")

    ctx.expect_within(
        blade_carriage,
        handle_body,
        axes="yz",
        inner_elem="carriage_core",
        margin=0.0005,
        name="carriage core stays within handle section at rest",
    )
    ctx.expect_overlap(
        blade_carriage,
        handle_body,
        axes="x",
        elem_a="carriage_core",
        min_overlap=0.040,
        name="carriage remains substantially inserted at rest",
    )
    ctx.expect_overlap(
        spare_tray,
        handle_body,
        axes="yz",
        elem_a="tray_cap",
        min_overlap=0.010,
        name="closed tray cap covers the tail opening",
    )

    rest_pos = ctx.part_world_position(blade_carriage)
    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    if slide_upper is not None:
        with ctx.pose({slide: slide_upper}):
            ctx.expect_within(
                blade_carriage,
                handle_body,
                axes="yz",
                inner_elem="carriage_core",
                margin=0.0005,
                name="carriage core stays guided in handle at full extension",
            )
            ctx.expect_overlap(
                blade_carriage,
                handle_body,
                axes="x",
                elem_a="carriage_core",
                min_overlap=0.018,
                name="carriage retains insertion at full extension",
            )
            extended_pos = ctx.part_world_position(blade_carriage)
        ctx.check(
            "blade carriage extends forward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.020,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    closed_tray_aabb = ctx.part_element_world_aabb(spare_tray, elem="tray_cap")
    tray_upper = tray_hinge.motion_limits.upper if tray_hinge.motion_limits is not None else None
    if tray_upper is not None:
        with ctx.pose({tray_hinge: tray_upper}):
            open_tray_aabb = ctx.part_element_world_aabb(spare_tray, elem="tray_cap")
        ctx.check(
            "spare tray folds down and outward from the tail",
            closed_tray_aabb is not None
            and open_tray_aabb is not None
            and (open_tray_aabb[0][2] + open_tray_aabb[1][2]) / 2.0
            < (closed_tray_aabb[0][2] + closed_tray_aabb[1][2]) / 2.0 - 0.006
            and (open_tray_aabb[0][0] + open_tray_aabb[1][0]) / 2.0
            < (closed_tray_aabb[0][0] + closed_tray_aabb[1][0]) / 2.0 - 0.006,
            details=f"closed={closed_tray_aabb}, open={open_tray_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
