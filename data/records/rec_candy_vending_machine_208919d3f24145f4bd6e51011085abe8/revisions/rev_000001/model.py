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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_vending_machine")

    cabinet_w = 0.86
    cabinet_d = 0.78
    cabinet_h = 1.82
    front_y = cabinet_d * 0.5
    side_t = 0.026

    door_w = 0.64
    door_h = 1.38
    door_t = 0.034
    door_bottom_z = 0.30
    door_hinge_x = -0.365
    door_y = front_y + 0.036

    shelf_width = cabinet_w - (2.0 * side_t)
    shelf_depth = 0.57
    shelf_y = -0.075
    shelf_levels = (0.54, 0.81, 1.08, 1.35)

    pickup_hinge_x = -0.028
    pickup_hinge_y = front_y + 0.005
    pickup_hinge_z = 0.168

    body_red = model.material("body_red", rgba=(0.64, 0.11, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    shelf_white = model.material("shelf_white", rgba=(0.91, 0.91, 0.88, 1.0))
    coil_steel = model.material("coil_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.58, 0.72, 0.82, 0.22))
    trim_silver = model.material("trim_silver", rgba=(0.74, 0.76, 0.78, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    indicator_green = model.material("indicator_green", rgba=(0.24, 0.66, 0.34, 1.0))

    def _spiral_mesh(
        *,
        length: float,
        radius: float,
        turns: float,
        wire_radius: float,
        center_z: float,
        name: str,
    ):
        point_count = max(28, int(turns * 16))
        points = [(0.0, -length * 0.5 - 0.020, center_z - radius * 0.88)]
        for index in range(point_count + 1):
            t = index / point_count
            angle = turns * 2.0 * math.pi * t
            points.append(
                (
                    radius * math.cos(angle),
                    -length * 0.5 + length * t,
                    center_z + radius * math.sin(angle),
                )
            )
        points.append((0.0, length * 0.5 + 0.016, center_z - radius * 0.88))
        return mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=wire_radius,
                samples_per_segment=2,
                radial_segments=14,
                cap_ends=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
            name,
        )

    spiral_lane = _spiral_mesh(
        length=0.34,
        radius=0.041,
        turns=4.25,
        wire_radius=0.008,
        center_z=0.042,
        name="vending_spiral_lane",
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_w, cabinet_d, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=charcoal,
        name="base_plinth",
    )
    cabinet.visual(
        Box((side_t, cabinet_d, cabinet_h - 0.08)),
        origin=Origin(xyz=(-(cabinet_w - side_t) * 0.5, 0.0, 0.95)),
        material=body_red,
        name="left_wall",
    )
    cabinet.visual(
        Box((side_t, cabinet_d, cabinet_h - 0.08)),
        origin=Origin(xyz=((cabinet_w - side_t) * 0.5, 0.0, 0.95)),
        material=body_red,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_w - 2.0 * side_t, side_t, cabinet_h - 0.08)),
        origin=Origin(xyz=(0.0, -(cabinet_d - side_t) * 0.5, 0.95)),
        material=body_red,
        name="rear_wall",
    )
    cabinet.visual(
        Box((cabinet_w, cabinet_d, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h - 0.02)),
        material=body_red,
        name="top_cap",
    )
    cabinet.visual(
        Box((cabinet_w, 0.07, 0.10)),
        origin=Origin(xyz=(0.0, front_y - 0.035, 1.73)),
        material=body_red,
        name="top_marquee",
    )
    cabinet.visual(
        Box((0.68, 0.012, 0.065)),
        origin=Origin(xyz=(-0.045, front_y - 0.002, 1.73)),
        material=smoked_glass,
        name="brand_window",
    )
    cabinet.visual(
        Box((0.050, 0.070, door_h + 0.08)),
        origin=Origin(xyz=(door_hinge_x - 0.020, front_y - 0.035, door_bottom_z + door_h * 0.5)),
        material=body_red,
        name="left_door_jamb",
    )
    cabinet.visual(
        Box((0.042, 0.070, door_h + 0.08)),
        origin=Origin(xyz=(door_hinge_x + door_w + 0.021, front_y - 0.035, door_bottom_z + door_h * 0.5)),
        material=body_red,
        name="right_door_jamb",
    )
    cabinet.visual(
        Box((0.158, 0.10, 1.44)),
        origin=Origin(xyz=(0.345, front_y - 0.050, 0.99)),
        material=charcoal,
        name="selector_column",
    )
    cabinet.visual(
        Box((0.132, 0.020, 1.18)),
        origin=Origin(xyz=(0.345, front_y + 0.000, 1.06)),
        material=panel_black,
        name="selector_panel_face",
    )
    cabinet.visual(
        Box((0.090, 0.006, 0.085)),
        origin=Origin(xyz=(0.345, front_y + 0.006, 1.48)),
        material=indicator_green,
        name="status_display",
    )
    cabinet.visual(
        Box((0.040, 0.012, 0.016)),
        origin=Origin(xyz=(0.345, front_y + 0.004, 1.30)),
        material=trim_silver,
        name="coin_slot",
    )
    cabinet.visual(
        Box((0.050, 0.012, 0.090)),
        origin=Origin(xyz=(0.345, front_y + 0.004, 0.84)),
        material=trim_silver,
        name="cashless_reader",
    )
    cabinet.visual(
        Box((cabinet_w, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, front_y - 0.035, 0.115)),
        material=body_red,
        name="pickup_lower_rail",
    )
    cabinet.visual(
        Box((0.252, 0.070, 0.160)),
        origin=Origin(xyz=(-0.282, front_y - 0.035, 0.225)),
        material=body_red,
        name="pickup_left_cheek",
    )
    cabinet.visual(
        Box((0.318, 0.070, 0.160)),
        origin=Origin(xyz=(0.245, front_y - 0.035, 0.225)),
        material=body_red,
        name="pickup_right_cheek",
    )
    cabinet.visual(
        Box((cabinet_w, 0.070, 0.075)),
        origin=Origin(xyz=(0.0, front_y - 0.035, 0.327)),
        material=body_red,
        name="pickup_header",
    )
    cabinet.visual(
        Box((0.300, 0.250, 0.020)),
        origin=Origin(xyz=(-0.030, 0.210, 0.180), rpy=(0.34, 0.0, 0.0)),
        material=panel_black,
        name="pickup_ramp",
    )
    cabinet.visual(
        Box((0.018, 0.220, 0.130)),
        origin=Origin(xyz=(-0.170, 0.200, 0.215)),
        material=panel_black,
        name="pickup_inner_left",
    )
    cabinet.visual(
        Box((0.018, 0.220, 0.130)),
        origin=Origin(xyz=(0.110, 0.200, 0.215)),
        material=panel_black,
        name="pickup_inner_right",
    )
    cabinet.visual(
        Box((0.260, 0.040, 0.018)),
        origin=Origin(xyz=(-0.030, front_y - 0.020, pickup_hinge_z)),
        material=trim_silver,
        name="pickup_hinge_barrel",
    )

    cabinet.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(door_hinge_x, front_y + 0.001, 0.47)),
        material=trim_silver,
        name="cabinet_hinge_knuckle_0",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(door_hinge_x, front_y + 0.001, 0.99)),
        material=trim_silver,
        name="cabinet_hinge_knuckle_1",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(door_hinge_x, front_y + 0.001, 1.51)),
        material=trim_silver,
        name="cabinet_hinge_knuckle_2",
    )

    coil_x_positions = (-0.245, 0.0, 0.245)
    for shelf_index, shelf_top in enumerate(shelf_levels):
        cabinet.visual(
            Box((shelf_width, shelf_depth, 0.012)),
            origin=Origin(xyz=(0.0, shelf_y, shelf_top)),
            material=shelf_white,
            name=f"shelf_deck_{shelf_index}",
        )
        cabinet.visual(
            Box((shelf_width, 0.018, 0.055)),
            origin=Origin(xyz=(0.0, shelf_y + shelf_depth * 0.5 - 0.009, shelf_top + 0.021)),
            material=shelf_white,
            name=f"shelf_front_lip_{shelf_index}",
        )
        cabinet.visual(
            Box((0.028, shelf_depth, 0.050)),
            origin=Origin(xyz=(-0.190, shelf_y, shelf_top + 0.019)),
            material=shelf_white,
            name=f"shelf_divider_left_{shelf_index}",
        )
        cabinet.visual(
            Box((0.028, shelf_depth, 0.050)),
            origin=Origin(xyz=(0.190, shelf_y, shelf_top + 0.019)),
            material=shelf_white,
            name=f"shelf_divider_right_{shelf_index}",
        )
        for lane_index, x_pos in enumerate(coil_x_positions):
            cabinet.visual(
                spiral_lane,
                origin=Origin(xyz=(x_pos, -0.020, shelf_top)),
                material=coil_steel,
                name=f"spiral_{shelf_index}_{lane_index}",
            )

    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_w, cabinet_d, cabinet_h)),
        mass=118.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_h * 0.5)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, -0.015, 0.18)),
        material=trim_silver,
        name="hinge_barrel_lower",
    )
    front_door.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, -0.015, door_h * 0.5)),
        material=trim_silver,
        name="hinge_barrel_center",
    )
    front_door.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, -0.015, door_h - 0.18)),
        material=trim_silver,
        name="hinge_barrel_upper",
    )
    front_door.visual(
        Box((0.028, door_t, door_h)),
        origin=Origin(xyz=(0.014, 0.0, door_h * 0.5)),
        material=charcoal,
        name="left_stile",
    )
    front_door.visual(
        Box((0.028, door_t, door_h)),
        origin=Origin(xyz=(door_w - 0.014, 0.0, door_h * 0.5)),
        material=charcoal,
        name="right_stile",
    )
    front_door.visual(
        Box((door_w, door_t, 0.036)),
        origin=Origin(xyz=(door_w * 0.5, 0.0, 0.018)),
        material=charcoal,
        name="bottom_rail",
    )
    front_door.visual(
        Box((door_w, door_t, 0.036)),
        origin=Origin(xyz=(door_w * 0.5, 0.0, door_h - 0.018)),
        material=charcoal,
        name="top_rail",
    )
    front_door.visual(
        Box((door_w - 0.074, 0.014, door_h - 0.088)),
        origin=Origin(xyz=(door_w * 0.5, 0.0, door_h * 0.5)),
        material=smoked_glass,
        name="glass_pane",
    )
    front_door.visual(
        Box((0.026, 0.024, 0.42)),
        origin=Origin(xyz=(door_w - 0.035, door_t * 0.52, 0.78)),
        material=trim_silver,
        name="door_handle",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=16.0,
        origin=Origin(xyz=(door_w * 0.5, 0.0, door_h * 0.5)),
    )

    chute_flap = model.part("chute_flap")
    chute_flap.visual(
        Cylinder(radius=0.008, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_silver,
        name="flap_hinge",
    )
    chute_flap.visual(
        Box((0.248, 0.010, 0.118)),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=panel_black,
        name="flap_panel",
    )
    chute_flap.inertial = Inertial.from_geometry(
        Box((0.248, 0.014, 0.118)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim_silver,
        name="knob_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    selector_knob.visual(
        Box((0.010, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, 0.034, 0.018)),
        material=trim_silver,
        name="knob_pointer",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.076, 0.046, 0.076)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
    )

    model.articulation(
        "cabinet_to_front_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=front_door,
        origin=Origin(xyz=(door_hinge_x, door_y, door_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "cabinet_to_chute_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=chute_flap,
        origin=Origin(xyz=(pickup_hinge_x, pickup_hinge_y, pickup_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.345, front_y + 0.010, 1.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=9.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    front_door = object_model.get_part("front_door")
    chute_flap = object_model.get_part("chute_flap")
    selector_knob = object_model.get_part("selector_knob")

    door_hinge = object_model.get_articulation("cabinet_to_front_door")
    flap_hinge = object_model.get_articulation("cabinet_to_chute_flap")
    knob_joint = object_model.get_articulation("cabinet_to_selector_knob")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    ctx.check(
        "door hinge axis is vertical",
        tuple(round(value, 3) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "pickup flap hinge axis is horizontal",
        tuple(round(abs(value), 3) for value in flap_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "selector knob is continuous on a front-facing shaft",
        (
            knob_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(value, 3) for value in knob_joint.axis) == (0.0, 1.0, 0.0)
            and knob_limits is not None
            and knob_limits.lower is None
            and knob_limits.upper is None
        ),
        details=(
            f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, "
            f"limits={knob_limits}"
        ),
    )

    with ctx.pose({door_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_contact(
            front_door,
            cabinet,
            elem_a="hinge_barrel_center",
            elem_b="cabinet_hinge_knuckle_1",
            name="door hinge barrel seats on cabinet knuckle",
        )
        ctx.expect_gap(
            front_door,
            cabinet,
            axis="y",
            positive_elem="glass_pane",
            min_gap=0.010,
            max_gap=0.030,
            name="door glazing sits proud of the cabinet face",
        )
        ctx.expect_overlap(
            front_door,
            cabinet,
            axes="xz",
            min_overlap=0.60,
            name="door covers the main product bay",
        )
        ctx.expect_contact(
            selector_knob,
            cabinet,
            elem_b="selector_panel_face",
            name="selector knob mounts to the control panel",
        )

    closed_handle = _aabb_center(ctx.part_element_world_aabb(front_door, elem="door_handle"))
    with ctx.pose({door_hinge: 1.10}):
        open_handle = _aabb_center(ctx.part_element_world_aabb(front_door, elem="door_handle"))
    ctx.check(
        "door opens outward on the side hinge",
        (
            closed_handle is not None
            and open_handle is not None
            and open_handle[1] > closed_handle[1] + 0.18
            and open_handle[0] < closed_handle[0] - 0.10
        ),
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    closed_flap = _aabb_center(ctx.part_element_world_aabb(chute_flap, elem="flap_panel"))
    with ctx.pose({flap_hinge: 0.92}):
        open_flap = _aabb_center(ctx.part_element_world_aabb(chute_flap, elem="flap_panel"))
    ctx.check(
        "pickup flap swings outward and downward",
        (
            closed_flap is not None
            and open_flap is not None
            and open_flap[1] > closed_flap[1] + 0.03
            and open_flap[2] < closed_flap[2] - 0.02
        ),
        details=f"closed_flap={closed_flap}, open_flap={open_flap}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
