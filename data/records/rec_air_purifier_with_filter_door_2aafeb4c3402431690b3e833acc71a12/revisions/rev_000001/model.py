from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 0.72
BODY_DEPTH = 0.56
BODY_HEIGHT = 1.40
CASTER_HEIGHT = 0.12
BODY_BOTTOM_Z = CASTER_HEIGHT
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT
PANEL_THICKNESS = 0.028


def _rect_profile(
    width: float,
    height: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _grille_mesh(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
    cols: int,
    rows: int,
    border_x: float,
    border_z: float,
    web: float,
):
    outer = _rect_profile(width, height)
    usable_w = width - 2.0 * border_x
    usable_h = height - 2.0 * border_z
    hole_w = (usable_w - (cols - 1) * web) / cols
    hole_h = (usable_h - (rows - 1) * web) / rows
    holes: list[list[tuple[float, float]]] = []
    start_x = -width * 0.5 + border_x + hole_w * 0.5
    start_z = -height * 0.5 + border_z + hole_h * 0.5
    for row in range(rows):
        hole_z = start_z + row * (hole_h + web)
        for col in range(cols):
            hole_x = start_x + col * (hole_w + web)
            holes.append(_rect_profile(hole_w, hole_h, cx=hole_x, cy=hole_z))
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer,
            holes,
            height=thickness,
            center=True,
        ).rotate_x(pi / 2.0),
    )


def _add_caster(part, *, steel, dark_steel, rubber) -> None:
    part.visual(
        Box((0.10, 0.08, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=steel,
        name="mount_plate",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=dark_steel,
        name="swivel_head",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=dark_steel,
        name="swivel_stem",
    )
    part.visual(
        Box((0.070, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=steel,
        name="yoke_bridge",
    )
    for side_sign in (-1.0, 1.0):
        part.visual(
            Box((0.012, 0.020, 0.050)),
            origin=Origin(xyz=(side_sign * 0.024, 0.0, -0.056)),
            material=steel,
            name=f"yoke_cheek_{'left' if side_sign < 0.0 else 'right'}",
        )
    part.visual(
        Cylinder(radius=0.006, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="wheel_hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_air_scrubber")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.80, 0.81, 0.82, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    blower_dark = model.material("blower_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_TOP_Z)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z * 0.5)),
    )

    body_center_z = BODY_BOTTOM_Z + BODY_HEIGHT * 0.5
    cabinet.visual(
        Box((PANEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH * 0.5 + PANEL_THICKNESS * 0.5, 0.0, body_center_z)),
        material=cabinet_paint,
        name="left_wall",
    )
    cabinet.visual(
        Box((PANEL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 - PANEL_THICKNESS * 0.5, 0.0, body_center_z)),
        material=cabinet_paint,
        name="right_wall",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * PANEL_THICKNESS + 0.012, PANEL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5, body_center_z)),
        material=cabinet_paint,
        name="back_wall",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 0.020, BODY_DEPTH - 0.020, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + PANEL_THICKNESS * 0.5)),
        material=trim_dark,
        name="base_pan",
    )
    cabinet.visual(
        Box((0.058, PANEL_THICKNESS, 1.18)),
        origin=Origin(xyz=(-0.332, BODY_DEPTH * 0.5 - PANEL_THICKNESS * 0.5, 0.770)),
        material=cabinet_paint,
        name="front_left_jamb",
    )
    cabinet.visual(
        Box((0.058, PANEL_THICKNESS, 1.18)),
        origin=Origin(xyz=(0.332, BODY_DEPTH * 0.5 - PANEL_THICKNESS * 0.5, 0.770)),
        material=cabinet_paint,
        name="front_right_jamb",
    )
    cabinet.visual(
        Box((0.672, PANEL_THICKNESS, 0.084)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 - PANEL_THICKNESS * 0.5, 0.162)),
        material=cabinet_paint,
        name="front_bottom_rail",
    )
    cabinet.visual(
        Box((0.672, PANEL_THICKNESS, 0.180)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 - PANEL_THICKNESS * 0.5, 1.430)),
        material=cabinet_paint,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * PANEL_THICKNESS + 0.010, 0.160, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.200, BODY_TOP_Z - PANEL_THICKNESS * 0.5)),
        material=cabinet_paint,
        name="top_front_rail",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * PANEL_THICKNESS + 0.010, 0.080, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.240, BODY_TOP_Z - PANEL_THICKNESS * 0.5)),
        material=cabinet_paint,
        name="top_rear_rail",
    )
    cabinet.visual(
        Box((0.120, 0.320, PANEL_THICKNESS)),
        origin=Origin(xyz=(-0.300, -0.040, BODY_TOP_Z - PANEL_THICKNESS * 0.5)),
        material=cabinet_paint,
        name="top_left_rail",
    )
    cabinet.visual(
        Box((0.120, 0.320, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.300, -0.040, BODY_TOP_Z - PANEL_THICKNESS * 0.5)),
        material=cabinet_paint,
        name="top_right_rail",
    )
    cabinet.visual(
        Box((0.400, 0.220, 0.320)),
        origin=Origin(xyz=(0.0, -0.170, 1.360)),
        material=blower_dark,
        name="blower_plenum",
    )
    for z_pos, name in ((0.380, "lower_hinge_backer"), (1.160, "upper_hinge_backer")):
        cabinet.visual(
            Box((0.018, PANEL_THICKNESS, 0.220)),
            origin=Origin(xyz=(-0.348, BODY_DEPTH * 0.5 - PANEL_THICKNESS * 0.5, z_pos)),
            material=hinge_steel,
            name=name,
        )
    cabinet.visual(
        Box((0.026, 0.030, 0.590)),
        origin=Origin(xyz=(-0.362, 0.267, 0.770)),
        material=hinge_steel,
        name="door_hinge_bridge",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=0.590),
        origin=Origin(xyz=(-0.375, 0.294, 0.770)),
        material=hinge_steel,
        name="door_center_knuckle",
    )
    cabinet.visual(
        Box((0.220, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.267, BODY_TOP_Z + 0.004)),
        material=hinge_steel,
        name="baffle_hinge_bridge",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.220),
        origin=Origin(xyz=(0.0, -0.255, BODY_TOP_Z + 0.024), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="baffle_center_knuckle",
    )

    access_door = model.part("access_door")
    access_door.inertial = Inertial.from_geometry(
        Box((0.622, 0.030, 1.120)),
        mass=18.0,
        origin=Origin(xyz=(0.316, 0.020, 0.0)),
    )

    access_door.visual(
        Box((0.066, 0.028, 1.120)),
        origin=Origin(xyz=(0.093, 0.008, 0.0)),
        material=cabinet_paint,
        name="left_stile",
    )
    access_door.visual(
        Box((0.066, 0.028, 1.120)),
        origin=Origin(xyz=(0.639, 0.008, 0.0)),
        material=cabinet_paint,
        name="right_stile",
    )
    access_door.visual(
        Box((0.622, 0.028, 0.120)),
        origin=Origin(xyz=(0.366, 0.008, 0.500)),
        material=cabinet_paint,
        name="top_rail",
    )
    access_door.visual(
        Box((0.622, 0.028, 0.140)),
        origin=Origin(xyz=(0.366, 0.008, -0.490)),
        material=cabinet_paint,
        name="bottom_rail",
    )
    access_door.visual(
        Box((0.480, 0.020, 0.200)),
        origin=Origin(xyz=(0.366, 0.006, 0.300)),
        material=cabinet_paint,
        name="upper_panel",
    )
    access_door.visual(
        _grille_mesh(
            "access_door_grille",
            width=0.480,
            height=0.720,
            thickness=0.0035,
            cols=5,
            rows=7,
            border_x=0.032,
            border_z=0.042,
            web=0.012,
        ),
        origin=Origin(xyz=(0.366, 0.022, -0.060)),
        material=trim_dark,
        name="door_grille",
    )
    for z_pos, name in ((0.390, "upper_knuckle"), (-0.390, "lower_knuckle")):
        access_door.visual(
            Cylinder(radius=0.012, length=0.190),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=hinge_steel,
            name=name,
        )
        access_door.visual(
            Box((0.056, 0.012, 0.200)),
            origin=Origin(xyz=(0.035, 0.010, z_pos)),
            material=hinge_steel,
            name=f"{name}_leaf",
        )
    access_door.visual(
        Box((0.014, 0.024, 0.036)),
        origin=Origin(xyz=(0.610, 0.031, 0.090)),
        material=hinge_steel,
        name="handle_upper_post",
    )
    access_door.visual(
        Box((0.014, 0.024, 0.036)),
        origin=Origin(xyz=(0.610, 0.031, -0.090)),
        material=hinge_steel,
        name="handle_lower_post",
    )
    access_door.visual(
        Cylinder(radius=0.012, length=0.240),
        origin=Origin(xyz=(0.610, 0.039, 0.0)),
        material=hinge_steel,
        name="handle_grip",
    )

    exhaust_baffle = model.part("exhaust_baffle")
    exhaust_baffle.inertial = Inertial.from_geometry(
        Box((0.480, 0.240, 0.070)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.120, 0.030)),
    )
    exhaust_baffle.visual(
        Box((0.480, 0.240, 0.016)),
        origin=Origin(xyz=(0.0, 0.138, -0.012)),
        material=cabinet_paint,
        name="baffle_panel",
    )
    exhaust_baffle.visual(
        Box((0.016, 0.240, 0.060)),
        origin=Origin(xyz=(-0.232, 0.138, 0.006)),
        material=cabinet_paint,
        name="left_flange",
    )
    exhaust_baffle.visual(
        Box((0.016, 0.240, 0.060)),
        origin=Origin(xyz=(0.232, 0.138, 0.006)),
        material=cabinet_paint,
        name="right_flange",
    )
    exhaust_baffle.visual(
        Box((0.440, 0.016, 0.050)),
        origin=Origin(xyz=(0.0, 0.250, 0.003)),
        material=cabinet_paint,
        name="front_lip",
    )
    exhaust_baffle.visual(
        Box((0.180, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, 0.256, 0.028)),
        material=hinge_steel,
        name="lift_tab",
    )
    exhaust_baffle.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(-0.160, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="left_baffle_knuckle",
    )
    exhaust_baffle.visual(
        Box((0.100, 0.012, 0.020)),
        origin=Origin(xyz=(-0.160, 0.016, 0.0)),
        material=hinge_steel,
        name="left_baffle_leaf",
    )
    exhaust_baffle.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="right_baffle_knuckle",
    )
    exhaust_baffle.visual(
        Box((0.100, 0.012, 0.020)),
        origin=Origin(xyz=(0.160, 0.016, 0.0)),
        material=hinge_steel,
        name="right_baffle_leaf",
    )

    caster_positions = {
        "front_left_caster": (-0.230, 0.180, BODY_BOTTOM_Z),
        "front_right_caster": (0.230, 0.180, BODY_BOTTOM_Z),
        "rear_left_caster": (-0.230, -0.180, BODY_BOTTOM_Z),
        "rear_right_caster": (0.230, -0.180, BODY_BOTTOM_Z),
    }
    for caster_name in caster_positions:
        caster = model.part(caster_name)
        caster.inertial = Inertial.from_geometry(
            Box((0.10, 0.08, 0.12)),
            mass=2.2,
            origin=Origin(xyz=(0.0, 0.0, -0.060)),
        )
        _add_caster(caster, steel=steel, dark_steel=trim_dark, rubber=rubber)

    model.articulation(
        "access_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=access_door,
        origin=Origin(xyz=(-0.375, 0.294, 0.770)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=0.0, upper=1.55),
    )
    model.articulation(
        "exhaust_baffle_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=exhaust_baffle,
        origin=Origin(xyz=(0.0, -0.255, BODY_TOP_Z + 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.10),
    )
    for caster_name, xyz in caster_positions.items():
        model.articulation(
            f"{caster_name}_mount",
            ArticulationType.FIXED,
            parent=cabinet,
            child=caster_name,
            origin=Origin(xyz=xyz),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    access_door = object_model.get_part("access_door")
    exhaust_baffle = object_model.get_part("exhaust_baffle")
    casters = [
        object_model.get_part("front_left_caster"),
        object_model.get_part("front_right_caster"),
        object_model.get_part("rear_left_caster"),
        object_model.get_part("rear_right_caster"),
    ]
    access_door_hinge = object_model.get_articulation("access_door_hinge")
    exhaust_baffle_hinge = object_model.get_articulation("exhaust_baffle_hinge")

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

    ctx.check(
        "door_hinge_axis_and_limits",
        access_door_hinge.axis == (0.0, 0.0, 1.0)
        and access_door_hinge.motion_limits is not None
        and access_door_hinge.motion_limits.lower == 0.0
        and access_door_hinge.motion_limits.upper is not None
        and access_door_hinge.motion_limits.upper >= 1.5,
        "Front access door should hinge about the vertical left edge with a wide service swing.",
    )
    ctx.check(
        "baffle_hinge_axis_and_limits",
        exhaust_baffle_hinge.axis == (1.0, 0.0, 0.0)
        and exhaust_baffle_hinge.motion_limits is not None
        and exhaust_baffle_hinge.motion_limits.lower == 0.0
        and exhaust_baffle_hinge.motion_limits.upper is not None
        and 1.0 <= exhaust_baffle_hinge.motion_limits.upper <= 1.2,
        "Top exhaust baffle should hinge about the rear edge on a lateral axis.",
    )

    for caster in casters:
        ctx.expect_contact(caster, cabinet, name=f"{caster.name}_mounted_to_cabinet")

    with ctx.pose({access_door_hinge: 0.0, exhaust_baffle_hinge: 0.0}):
        ctx.expect_contact(
            cabinet,
            access_door,
            elem_a="door_center_knuckle",
            elem_b="upper_knuckle",
            name="upper_hinge_knuckle_contact",
        )
        ctx.expect_contact(
            cabinet,
            access_door,
            elem_a="door_center_knuckle",
            elem_b="lower_knuckle",
            name="lower_hinge_knuckle_contact",
        )
        ctx.expect_gap(
            access_door,
            cabinet,
            axis="y",
            min_gap=0.002,
            max_gap=0.012,
            positive_elem="upper_panel",
            negative_elem="front_top_rail",
            name="door_closed_clearance",
        )
        ctx.expect_overlap(
            access_door,
            cabinet,
            axes="xz",
            min_overlap=0.50,
            name="door_covers_front_opening",
        )
        ctx.expect_gap(
            exhaust_baffle,
            cabinet,
            axis="z",
            min_gap=0.003,
            max_gap=0.020,
            positive_elem="baffle_panel",
            negative_elem="top_front_rail",
            name="baffle_closed_standoff",
        )
        ctx.expect_contact(
            cabinet,
            exhaust_baffle,
            elem_a="baffle_center_knuckle",
            elem_b="left_baffle_knuckle",
            name="left_baffle_knuckle_contact",
        )
        ctx.expect_contact(
            cabinet,
            exhaust_baffle,
            elem_a="baffle_center_knuckle",
            elem_b="right_baffle_knuckle",
            name="right_baffle_knuckle_contact",
        )
        ctx.expect_overlap(
            exhaust_baffle,
            cabinet,
            axes="xy",
            min_overlap=0.20,
            name="baffle_sits_over_top_exhaust",
        )

    with ctx.pose({access_door_hinge: 1.25}):
        door_aabb = ctx.part_world_aabb(access_door)
        door_y_max = door_aabb[1][1] if door_aabb is not None else None
        ctx.check(
            "door_swings_outward",
            door_y_max is not None and door_y_max > 0.60,
            f"Expected open door to project well forward of the cabinet; got y_max={door_y_max!r}.",
        )

    with ctx.pose({exhaust_baffle_hinge: 1.0}):
        baffle_aabb = ctx.part_world_aabb(exhaust_baffle)
        baffle_z_max = baffle_aabb[1][2] if baffle_aabb is not None else None
        ctx.check(
            "baffle_lifts_above_top",
            baffle_z_max is not None and baffle_z_max > BODY_TOP_Z + 0.18,
            f"Expected lifted baffle to rise above the cabinet top; got z_max={baffle_z_max!r}.",
        )

    with ctx.pose({access_door_hinge: 1.25, exhaust_baffle_hinge: 1.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_service_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
