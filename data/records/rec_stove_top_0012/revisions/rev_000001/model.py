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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

COUNTER_SIZE = (1.000, 0.700, 0.040)
COOKTOP_SIZE = (0.769, 0.519, 0.008)

COUNTER_TOP_Z = COUNTER_SIZE[2]
COOKTOP_CENTER_Z = COUNTER_TOP_Z - (COOKTOP_SIZE[2] * 0.5)
COOKTOP_BOTTOM_Z = COUNTER_TOP_Z - COOKTOP_SIZE[2]

SIDE_RAIL_WIDTH = (COUNTER_SIZE[0] - COOKTOP_SIZE[0]) * 0.5
FRONT_BACK_RAIL_DEPTH = (COUNTER_SIZE[1] - COOKTOP_SIZE[1]) * 0.5

CONTROL_STRIP_DEPTH = 0.055
REAR_FIELD_DEPTH = COOKTOP_SIZE[1] - CONTROL_STRIP_DEPTH
CONTROL_STRIP_CENTER_Y = -(COOKTOP_SIZE[1] * 0.5) + (CONTROL_STRIP_DEPTH * 0.5)
REAR_FIELD_CENTER_Y = (CONTROL_STRIP_DEPTH * 0.5) + (REAR_FIELD_DEPTH * 0.5) - (COOKTOP_SIZE[1] * 0.5)

BUTTON_RADIUS = 0.0048
BUTTON_HOLE_RADIUS = 0.0062
BUTTON_HEIGHT = 0.0060
BUTTON_PROUD = 0.0008
BUTTON_TRAVEL = 0.0010

BUTTON_SPECS = (
    ("button_pair_1_left", -0.185),
    ("button_pair_1_right", -0.149),
    ("button_pair_2_left", -0.022),
    ("button_pair_2_right", 0.014),
    ("button_pair_3_left", 0.142),
    ("button_pair_3_right", 0.178),
)

ZONE_SPECS = (
    ("zone_front_left", -0.185, -0.070, 0.090),
    ("zone_front_right", 0.185, -0.070, 0.100),
    ("zone_rear_left", -0.185, 0.120, 0.100),
    ("zone_rear_right", 0.185, 0.120, 0.090),
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + (radius * math.cos((2.0 * math.pi * index) / segments)),
            cy + (radius * math.sin((2.0 * math.pi * index) / segments)),
        )
        for index in range(segments)
    ]


def _build_control_strip_mesh():
    outer = rounded_rect_profile(COOKTOP_SIZE[0], CONTROL_STRIP_DEPTH, 0.006, corner_segments=6)
    holes = [_circle_profile(BUTTON_HOLE_RADIUS, center=(x, 0.0), segments=40) for _, x in BUTTON_SPECS]
    geom = ExtrudeWithHolesGeometry(
        outer,
        holes,
        COOKTOP_SIZE[2],
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("cooktop_control_strip.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shallow_electric_cooktop", assets=ASSETS)

    counter_stone = model.material("counter_stone", rgba=(0.67, 0.65, 0.61, 1.0))
    glass_black = model.material("glass_black", rgba=(0.07, 0.08, 0.09, 1.0))
    control_graphite = model.material("control_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    zone_outer = model.material("zone_outer", rgba=(0.64, 0.28, 0.24, 0.40))
    zone_inner = model.material("zone_inner", rgba=(0.88, 0.53, 0.45, 0.65))
    button_metal = model.material("button_metal", rgba=(0.79, 0.80, 0.82, 1.0))

    control_strip_mesh = _build_control_strip_mesh()

    counter = model.part("counter")
    counter.visual(
        Box((SIDE_RAIL_WIDTH, COUNTER_SIZE[1], COUNTER_SIZE[2])),
        origin=Origin(
            xyz=(
                -(COOKTOP_SIZE[0] * 0.5) - (SIDE_RAIL_WIDTH * 0.5),
                0.0,
                COUNTER_SIZE[2] * 0.5,
            )
        ),
        material=counter_stone,
        name="counter_left_rail",
    )
    counter.visual(
        Box((SIDE_RAIL_WIDTH, COUNTER_SIZE[1], COUNTER_SIZE[2])),
        origin=Origin(
            xyz=(
                (COOKTOP_SIZE[0] * 0.5) + (SIDE_RAIL_WIDTH * 0.5),
                0.0,
                COUNTER_SIZE[2] * 0.5,
            )
        ),
        material=counter_stone,
        name="counter_right_rail",
    )
    counter.visual(
        Box((COOKTOP_SIZE[0], FRONT_BACK_RAIL_DEPTH, COUNTER_SIZE[2])),
        origin=Origin(
            xyz=(
                0.0,
                -(COOKTOP_SIZE[1] * 0.5) - (FRONT_BACK_RAIL_DEPTH * 0.5),
                COUNTER_SIZE[2] * 0.5,
            )
        ),
        material=counter_stone,
        name="counter_front_rail",
    )
    counter.visual(
        Box((COOKTOP_SIZE[0], FRONT_BACK_RAIL_DEPTH, COUNTER_SIZE[2])),
        origin=Origin(
            xyz=(
                0.0,
                (COOKTOP_SIZE[1] * 0.5) + (FRONT_BACK_RAIL_DEPTH * 0.5),
                COUNTER_SIZE[2] * 0.5,
            )
        ),
        material=counter_stone,
        name="counter_back_rail",
    )
    counter.visual(
        Box((COOKTOP_SIZE[0], REAR_FIELD_DEPTH, COOKTOP_SIZE[2])),
        origin=Origin(xyz=(0.0, REAR_FIELD_CENTER_Y, COOKTOP_CENTER_Z)),
        material=glass_black,
        name="glass_field",
    )
    counter.visual(
        control_strip_mesh,
        origin=Origin(xyz=(0.0, CONTROL_STRIP_CENTER_Y, COOKTOP_CENTER_Z)),
        material=control_graphite,
        name="control_strip",
    )
    for zone_name, center_x, center_y, radius in ZONE_SPECS:
        counter.visual(
            Cylinder(radius=radius, length=0.0006),
            origin=Origin(xyz=(center_x, center_y, COUNTER_TOP_Z + 0.0003)),
            material=zone_outer,
            name=zone_name,
        )
        counter.visual(
            Cylinder(radius=radius * 0.72, length=0.0004),
            origin=Origin(xyz=(center_x, center_y, COUNTER_TOP_Z + 0.0002)),
            material=zone_inner,
            name=f"{zone_name}_inner",
        )
    counter.inertial = Inertial.from_geometry(
        Box(COUNTER_SIZE),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_SIZE[2] * 0.5)),
    )

    for button_name, center_x in BUTTON_SPECS:
        button = model.part(button_name)
        button.visual(
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_PROUD - (BUTTON_HEIGHT * 0.5))),
            material=button_metal,
            name="button_plunger",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_HEIGHT),
            mass=0.014,
            origin=Origin(xyz=(0.0, 0.0, BUTTON_PROUD - (BUTTON_HEIGHT * 0.5))),
        )
        model.articulation(
            f"counter_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=counter,
            child=button,
            origin=Origin(xyz=(center_x, CONTROL_STRIP_CENTER_Y, COUNTER_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.03,
                lower=-BUTTON_TRAVEL,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    counter = object_model.get_part("counter")
    button_parts = [object_model.get_part(name) for name, _ in BUTTON_SPECS]
    button_joints = [object_model.get_articulation(f"counter_to_{name}") for name, _ in BUTTON_SPECS]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    glass_aabb = ctx.part_element_world_aabb(counter, elem="glass_field")
    strip_aabb = ctx.part_element_world_aabb(counter, elem="control_strip")
    if glass_aabb is not None and strip_aabb is not None:
        cooktop_bottom = min(glass_aabb[0][2], strip_aabb[0][2])
        cooktop_top = max(glass_aabb[1][2], strip_aabb[1][2])
        cooktop_front = min(glass_aabb[0][1], strip_aabb[0][1])
        cooktop_back = max(glass_aabb[1][1], strip_aabb[1][1])
        ctx.check(
            "cooktop_is_shallow",
            (cooktop_top - cooktop_bottom) <= 0.009,
            details=f"thickness={cooktop_top - cooktop_bottom:.4f}",
        )
        ctx.check(
            "cooktop_is_flush_with_counter",
            abs(cooktop_top - COUNTER_TOP_Z) <= 5e-4,
            details=f"cooktop_top={cooktop_top:.4f}",
        )
        ctx.check(
            "no_base_below_counter",
            cooktop_bottom >= COOKTOP_BOTTOM_Z - 5e-4,
            details=f"cooktop_bottom={cooktop_bottom:.4f}",
        )
        ctx.check(
            "cooktop_depth_realistic",
            (cooktop_back - cooktop_front) >= 0.49,
            details=f"depth={cooktop_back - cooktop_front:.4f}",
        )

    for zone_name, _, _, _ in ZONE_SPECS:
        ctx.expect_within(
            counter,
            counter,
            axes="xy",
            inner_elem=zone_name,
            outer_elem="glass_field",
            name=f"{zone_name}_within_glass",
        )

    zone_aabbs = {
        zone_name: ctx.part_element_world_aabb(counter, elem=zone_name)
        for zone_name, _, _, _ in ZONE_SPECS
    }
    if all(aabb is not None for aabb in zone_aabbs.values()):
        centers = {
            name: (
                (aabb[0][0] + aabb[1][0]) * 0.5,
                (aabb[0][1] + aabb[1][1]) * 0.5,
            )
            for name, aabb in zone_aabbs.items()
            if aabb is not None
        }
        diameters = {
            name: aabb[1][0] - aabb[0][0]
            for name, aabb in zone_aabbs.items()
            if aabb is not None
        }
        ctx.check(
            "zones_form_two_by_two_layout",
            (
                centers["zone_front_left"][0] < -0.10
                and centers["zone_rear_left"][0] < -0.10
                and centers["zone_front_right"][0] > 0.10
                and centers["zone_rear_right"][0] > 0.10
                and centers["zone_rear_left"][1] > centers["zone_front_left"][1] + 0.14
                and centers["zone_rear_right"][1] > centers["zone_front_right"][1] + 0.14
            ),
            details=f"centers={centers!r}",
        )
        ctx.check(
            "zone_sizes_realistic",
            all(0.17 <= diameter <= 0.21 for diameter in diameters.values()),
            details=f"diameters={diameters!r}",
        )

    button_positions = [ctx.part_world_position(button) for button in button_parts]
    if all(position is not None for position in button_positions):
        xs = [position[0] for position in button_positions if position is not None]
        ys = [position[1] for position in button_positions if position is not None]
        pair_gaps = (xs[1] - xs[0], xs[3] - xs[2], xs[5] - xs[4])
        group_gaps = (xs[2] - xs[1], xs[4] - xs[3])
        ctx.check(
            "buttons_in_three_close_pairs",
            all(0.028 <= gap <= 0.045 for gap in pair_gaps) and all(gap >= 0.10 for gap in group_gaps),
            details=f"pair_gaps={pair_gaps!r} group_gaps={group_gaps!r}",
        )
        ctx.check(
            "buttons_along_front_edge",
            (max(ys) - min(ys) <= 1e-6) and (ys[0] < -0.22),
            details=f"ys={ys!r}",
        )

    for button_name, _ in BUTTON_SPECS:
        ctx.expect_contact(button_name, counter, contact_tol=1e-6, name=f"{button_name}_touches_counter")

    for button, joint in zip(button_parts, button_joints):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_pressed_no_floating")
            ctx.expect_contact(button, counter, contact_tol=1e-6, name=f"{joint.name}_pressed_contact")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: limits.lower}):
            pressed_pos = ctx.part_world_position(button)
        if rest_pos is not None and pressed_pos is not None:
            ctx.check(
                f"{button.name}_plunges_inward",
                pressed_pos[2] < rest_pos[2] - 0.0008,
                details=f"rest_z={rest_pos[2]:.4f} pressed_z={pressed_pos[2]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
