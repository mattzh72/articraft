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


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _make_plate_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
    mesh_name: str,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _make_frame_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    hole_profile: list[tuple[float, float]],
    mesh_name: str,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, height, corner_radius, corner_segments=8),
        [hole_profile],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_function_tailgate")

    body_paint = model.material("body_paint", rgba=(0.64, 0.10, 0.12, 1.0))
    inner_paint = model.material("inner_paint", rgba=(0.52, 0.08, 0.10, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    liner = model.material("liner", rgba=(0.22, 0.23, 0.24, 1.0))
    hardware = model.material("hardware", rgba=(0.36, 0.38, 0.40, 1.0))

    bed_width = 1.72
    bed_length = 1.70
    bedside_thickness = 0.06
    floor_thickness = 0.04
    wall_height = 0.52
    floor_top = 0.55

    tailgate_width = 1.56
    tailgate_height = 0.53
    tailgate_thickness = 0.058

    center_open_width = 0.50
    center_open_height = 0.41
    center_leaf_width = 0.456
    center_leaf_height = 0.392
    center_leaf_thickness = 0.044
    center_open_offset_z = 0.020
    center_hinge_axis_y = -0.018
    center_leaf_center_x = 0.244

    center_open_bottom = (
        tailgate_height * 0.5
        + center_open_offset_z
        - center_open_height * 0.5
    )
    center_leaf_bottom = center_open_bottom + (center_open_height - center_leaf_height) * 0.5
    center_hinge_x = -0.244

    center_hole_profile = _offset_profile(
        rounded_rect_profile(center_open_width, center_open_height, 0.018, corner_segments=6),
        dy=center_open_offset_z,
    )
    tailgate_frame_mesh = _make_frame_mesh(
        width=tailgate_width,
        height=tailgate_height,
        thickness=tailgate_thickness,
        corner_radius=0.034,
        hole_profile=center_hole_profile,
        mesh_name="tailgate_outer_frame",
    )
    center_leaf_mesh = _make_plate_mesh(
        width=center_leaf_width,
        height=center_leaf_height,
        thickness=center_leaf_thickness,
        radius=0.014,
        mesh_name="tailgate_center_leaf",
    )

    bed = model.part("bed")
    bed.visual(
        Box((bed_width, bed_length, floor_thickness)),
        origin=Origin(xyz=(0.0, bed_length * 0.5, floor_top - floor_thickness * 0.5)),
        material=liner,
        name="bed_floor",
    )
    bed.visual(
        Box((bedside_thickness, bed_length, wall_height)),
        origin=Origin(
            xyz=(
                -bed_width * 0.5 + bedside_thickness * 0.5,
                bed_length * 0.5,
                floor_top + wall_height * 0.5,
            )
        ),
        material=body_paint,
        name="left_bedside",
    )
    bed.visual(
        Box((bedside_thickness, bed_length, wall_height)),
        origin=Origin(
            xyz=(
                bed_width * 0.5 - bedside_thickness * 0.5,
                bed_length * 0.5,
                floor_top + wall_height * 0.5,
            )
        ),
        material=body_paint,
        name="right_bedside",
    )
    bed.visual(
        Box((bed_width - 2.0 * bedside_thickness, bedside_thickness, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                bed_length - bedside_thickness * 0.5,
                floor_top + wall_height * 0.5,
            )
        ),
        material=body_paint,
        name="front_bed_wall",
    )
    bed.visual(
        Box((bed_width, 0.09, 0.018)),
        origin=Origin(xyz=(0.0, 0.22, floor_top + 0.009)),
        material=dark_trim,
        name="floor_rib_front",
    )
    bed.visual(
        Box((bed_width, 0.09, 0.018)),
        origin=Origin(xyz=(0.0, 0.78, floor_top + 0.009)),
        material=dark_trim,
        name="floor_rib_middle",
    )
    bed.visual(
        Box((bed_width, 0.09, 0.018)),
        origin=Origin(xyz=(0.0, 1.34, floor_top + 0.009)),
        material=dark_trim,
        name="floor_rib_rear",
    )
    bed.inertial = Inertial.from_geometry(
        Box((bed_width, bed_length, wall_height + floor_top)),
        mass=95.0,
        origin=Origin(xyz=(0.0, bed_length * 0.5, (floor_top + wall_height) * 0.5)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        tailgate_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, tailgate_height * 0.5)),
        material=inner_paint,
        name="tailgate_frame",
    )
    for pad_name, pad_x in (
        ("top_step_pad_left", -0.515),
        ("top_step_pad_right", 0.515),
    ):
        tailgate.visual(
            Box((0.43, 0.012, 0.032)),
            origin=Origin(
                xyz=(pad_x, -tailgate_thickness * 0.5 + 0.006, tailgate_height - 0.060)
            ),
            material=dark_trim,
            name=pad_name,
        )
    tailgate.visual(
        Box((0.020, 0.024, 0.210)),
        origin=Origin(xyz=(center_hinge_x - 0.014, -0.010, 0.285)),
        material=hardware,
        name="center_hinge_mount",
    )
    for hinge_name, hinge_x in (
        ("lower_hinge_roll_left", -0.46),
        ("lower_hinge_roll_center", 0.0),
        ("lower_hinge_roll_right", 0.46),
    ):
        tailgate.visual(
            Cylinder(radius=0.018, length=0.20),
            origin=Origin(
                xyz=(hinge_x, -tailgate_thickness * 0.5 + 0.005, 0.018),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware,
            name=hinge_name,
        )
    tailgate.visual(
        Cylinder(radius=0.010, length=0.074),
        origin=Origin(
            xyz=(center_hinge_x, center_hinge_axis_y, 0.207),
        ),
        material=hardware,
        name="frame_hinge_knuckle_upper",
    )
    tailgate.visual(
        Cylinder(radius=0.010, length=0.074),
        origin=Origin(
            xyz=(center_hinge_x, center_hinge_axis_y, 0.363),
        ),
        material=hardware,
        name="frame_hinge_knuckle_lower",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((tailgate_width, tailgate_thickness, tailgate_height)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, tailgate_height * 0.5)),
    )

    center_door = model.part("center_door")
    center_door.visual(
        center_leaf_mesh,
        origin=Origin(xyz=(center_leaf_center_x, -center_hinge_axis_y, center_leaf_height * 0.5)),
        material=body_paint,
        name="center_panel",
    )
    center_door.visual(
        Box((0.020, 0.028, center_leaf_height)),
        origin=Origin(xyz=(0.020, 0.014, center_leaf_height * 0.5)),
        material=hardware,
        name="hinge_stile",
    )
    center_door.visual(
        Box((0.060, 0.012, 0.060)),
        origin=Origin(xyz=(0.040, 0.008, 0.098)),
        material=hardware,
        name="hinge_leaf_upper",
    )
    center_door.visual(
        Box((0.060, 0.012, 0.060)),
        origin=Origin(xyz=(0.040, 0.008, 0.294)),
        material=hardware,
        name="hinge_leaf_lower",
    )
    center_door.visual(
        Box((0.060, 0.014, 0.022)),
        origin=Origin(
            xyz=(
                center_leaf_center_x + center_leaf_width * 0.24,
                -center_hinge_axis_y - center_leaf_thickness * 0.5 + 0.007,
                center_leaf_height * 0.58,
            )
        ),
        material=dark_trim,
        name="door_pull",
    )
    center_door.inertial = Inertial.from_geometry(
        Box((center_leaf_width + 0.016, center_leaf_thickness, center_leaf_height)),
        mass=8.5,
        origin=Origin(
            xyz=(center_leaf_center_x, -center_hinge_axis_y, center_leaf_height * 0.5)
        ),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, floor_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "tailgate_to_center_door",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=center_door,
        origin=Origin(xyz=(center_hinge_x, center_hinge_axis_y, center_leaf_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    tailgate = object_model.get_part("tailgate")
    center_door = object_model.get_part("center_door")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    center_hinge = object_model.get_articulation("tailgate_to_center_door")

    ctx.check(
        "tailgate hinge is a lower horizontal drop axis",
        tailgate_hinge.axis == (1.0, 0.0, 0.0)
        and tailgate_hinge.motion_limits is not None
        and tailgate_hinge.motion_limits.lower == 0.0
        and tailgate_hinge.motion_limits.upper is not None
        and tailgate_hinge.motion_limits.upper >= 1.5,
        details=f"axis={tailgate_hinge.axis}, limits={tailgate_hinge.motion_limits}",
    )
    ctx.check(
        "center door hinge is vertical inside the main gate",
        center_hinge.axis == (0.0, 0.0, -1.0)
        and center_hinge.motion_limits is not None
        and center_hinge.motion_limits.lower == 0.0
        and center_hinge.motion_limits.upper is not None
        and center_hinge.motion_limits.upper >= 1.3,
        details=f"axis={center_hinge.axis}, limits={center_hinge.motion_limits}",
    )

    with ctx.pose({tailgate_hinge: 0.0, center_hinge: 0.0}):
        ctx.expect_gap(
            tailgate,
            bed,
            axis="z",
            positive_elem="tailgate_frame",
            negative_elem="bed_floor",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed tailgate seats at the bed floor hinge line",
        )
        ctx.expect_within(
            center_door,
            tailgate,
            axes="xz",
            inner_elem="center_panel",
            outer_elem="tailgate_frame",
            margin=0.03,
            name="center door stays within the tailgate envelope when shut",
        )
        ctx.expect_contact(
            center_door,
            tailgate,
            elem_a="hinge_stile",
            elem_b="frame_hinge_knuckle_upper",
            contact_tol=0.0005,
            name="upper center-door hinge stile bears on the tailgate knuckle",
        )
        ctx.expect_contact(
            center_door,
            tailgate,
            elem_a="hinge_stile",
            elem_b="frame_hinge_knuckle_lower",
            contact_tol=0.0005,
            name="lower center-door hinge stile bears on the tailgate knuckle",
        )

        tailgate_closed_aabb = ctx.part_element_world_aabb(tailgate, elem="tailgate_frame")
        center_closed_aabb = ctx.part_element_world_aabb(center_door, elem="center_panel")

    with ctx.pose({tailgate_hinge: 1.45, center_hinge: 0.0}):
        tailgate_open_aabb = ctx.part_element_world_aabb(tailgate, elem="tailgate_frame")

    with ctx.pose({tailgate_hinge: 0.0, center_hinge: 1.25}):
        center_open_aabb = ctx.part_element_world_aabb(center_door, elem="center_panel")

    tailgate_opens_down = (
        tailgate_closed_aabb is not None
        and tailgate_open_aabb is not None
        and tailgate_open_aabb[0][1] < tailgate_closed_aabb[0][1] - 0.35
        and tailgate_open_aabb[1][2] < tailgate_closed_aabb[1][2] - 0.35
    )
    ctx.check(
        "main gate rotates downward from the bed opening",
        tailgate_opens_down,
        details=f"closed={tailgate_closed_aabb}, open={tailgate_open_aabb}",
    )

    center_swings_sideways = (
        center_closed_aabb is not None
        and center_open_aabb is not None
        and center_open_aabb[0][1] < center_closed_aabb[0][1] - 0.18
        and abs(center_open_aabb[0][2] - center_closed_aabb[0][2]) < 0.03
    )
    ctx.check(
        "center door swings sideways out of the tailgate opening",
        center_swings_sideways,
        details=f"closed={center_closed_aabb}, open={center_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
