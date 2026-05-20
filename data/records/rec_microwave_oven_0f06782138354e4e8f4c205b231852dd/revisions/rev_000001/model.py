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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rounded_rect(width: float, height: float, radius: float) -> list[tuple[float, float]]:
    return rounded_rect_profile(
        width,
        height,
        min(radius, width * 0.49, height * 0.49),
        corner_segments=6,
    )


def _front_panel_mesh(
    *,
    outer_width: float,
    outer_height: float,
    thickness: float,
    holes: list[list[tuple[float, float]]],
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        _rounded_rect(outer_width, outer_height, 0.008),
        holes,
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_microwave")

    stainless = model.material("stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    cavity_enamel = model.material("cavity_enamel", rgba=(0.88, 0.88, 0.86, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.14, 0.16, 0.18, 0.42))
    handle_metal = model.material("handle_metal", rgba=(0.82, 0.83, 0.85, 1.0))
    button_silver = model.material("button_silver", rgba=(0.76, 0.77, 0.79, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.73, 0.84, 0.89, 0.40))
    indicator_white = model.material("indicator_white", rgba=(0.93, 0.94, 0.95, 1.0))

    housing = model.part("housing")

    housing.visual(
        Box((0.600, 0.420, 0.018)),
        origin=Origin(xyz=(0.000, 0.210, 0.009)),
        material=stainless,
        name="bottom_shell",
    )
    housing.visual(
        Box((0.600, 0.420, 0.020)),
        origin=Origin(xyz=(0.000, 0.210, 0.380)),
        material=stainless,
        name="top_shell",
    )
    housing.visual(
        Box((0.020, 0.420, 0.352)),
        origin=Origin(xyz=(-0.290, 0.210, 0.194)),
        material=stainless,
        name="left_shell",
    )
    housing.visual(
        Box((0.020, 0.420, 0.352)),
        origin=Origin(xyz=(0.290, 0.210, 0.194)),
        material=stainless,
        name="right_shell",
    )
    housing.visual(
        Box((0.560, 0.018, 0.352)),
        origin=Origin(xyz=(0.000, 0.411, 0.194)),
        material=stainless,
        name="back_shell",
    )
    housing.visual(
        Box((0.010, 0.384, 0.352)),
        origin=Origin(xyz=(0.160, 0.210, 0.194)),
        material=stainless,
        name="control_divider",
    )
    housing.visual(
        Box((0.114, 0.320, 0.290)),
        origin=Origin(xyz=(0.223, 0.250, 0.195)),
        material=control_black,
        name="control_core",
    )

    housing.visual(
        Box((0.433, 0.378, 0.002)),
        origin=Origin(xyz=(-0.0625, 0.211, 0.019)),
        material=cavity_enamel,
        name="cavity_floor",
    )
    housing.visual(
        Box((0.433, 0.378, 0.002)),
        origin=Origin(xyz=(-0.0625, 0.211, 0.369)),
        material=cavity_enamel,
        name="cavity_ceiling",
    )
    housing.visual(
        Box((0.002, 0.378, 0.348)),
        origin=Origin(xyz=(-0.279, 0.211, 0.194)),
        material=cavity_enamel,
        name="cavity_left_wall",
    )
    housing.visual(
        Box((0.002, 0.378, 0.348)),
        origin=Origin(xyz=(0.159, 0.211, 0.194)),
        material=cavity_enamel,
        name="cavity_right_wall",
    )
    housing.visual(
        Box((0.433, 0.002, 0.348)),
        origin=Origin(xyz=(-0.0625, 0.401, 0.194)),
        material=cavity_enamel,
        name="cavity_back",
    )

    control_holes = [
        _translate_profile(_rounded_rect(0.074, 0.030, 0.004), 0.000, 0.112),
        _translate_profile(_rounded_rect(0.026, 0.046, 0.004), -0.024, 0.035),
        _translate_profile(_rounded_rect(0.026, 0.046, 0.004), -0.024, -0.018),
        _translate_profile(_rounded_rect(0.026, 0.046, 0.004), -0.024, -0.071),
        _translate_profile(_rounded_rect(0.026, 0.046, 0.004), -0.024, -0.124),
        _translate_profile(_rounded_rect(0.054, 0.054, 0.008), 0.025, -0.097),
    ]
    control_panel_mesh = _front_panel_mesh(
        outer_width=0.115,
        outer_height=0.352,
        thickness=0.012,
        holes=control_holes,
        name="microwave_control_panel",
    )
    housing.visual(
        control_panel_mesh,
        origin=Origin(xyz=(0.2225, 0.006, 0.194)),
        material=control_black,
        name="control_fascia",
    )
    housing.visual(
        Box((0.080, 0.004, 0.036)),
        origin=Origin(xyz=(0.2225, 0.014, 0.306)),
        material=dark_glass,
        name="display_window",
    )
    housing.visual(
        Box((0.102, 0.006, 0.286)),
        origin=Origin(xyz=(0.223, 0.015, 0.195)),
        material=control_black,
        name="control_mount_plate",
    )

    housing.inertial = Inertial.from_geometry(
        Box((0.600, 0.420, 0.390)),
        mass=18.0,
        origin=Origin(xyz=(0.000, 0.210, 0.195)),
    )

    door = model.part("door")
    door_holes = [_rounded_rect(0.300, 0.210, 0.010)]
    door_frame_mesh = _front_panel_mesh(
        outer_width=0.431,
        outer_height=0.348,
        thickness=0.018,
        holes=door_holes,
        name="microwave_door_frame",
    )
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(0.2155, -0.009, 0.000)),
        material=control_black,
        name="door_frame",
    )
    door.visual(
        Box((0.308, 0.004, 0.218)),
        origin=Origin(xyz=(0.2155, -0.005, 0.000)),
        material=dark_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.018, 0.010, 0.022)),
        origin=Origin(xyz=(0.391, -0.022, 0.097)),
        material=handle_metal,
        name="handle_upper_mount",
    )
    door.visual(
        Box((0.018, 0.010, 0.022)),
        origin=Origin(xyz=(0.391, -0.022, -0.097)),
        material=handle_metal,
        name="handle_lower_mount",
    )
    door.visual(
        Box((0.014, 0.016, 0.180)),
        origin=Origin(xyz=(0.403, -0.028, 0.000)),
        material=handle_metal,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.431, 0.030, 0.348)),
        mass=3.8,
        origin=Origin(xyz=(0.2155, -0.015, 0.000)),
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.280, 0.000, 0.194)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.000, -0.006, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.000, -0.017, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_silver,
        name="knob_face",
    )
    knob.visual(
        Box((0.003, 0.004, 0.012)),
        origin=Origin(xyz=(0.000, -0.027, 0.013)),
        material=indicator_white,
        name="knob_indicator",
    )
    knob.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.000, 0.006, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="knob_shaft",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.030),
        mass=0.12,
        origin=Origin(xyz=(0.000, -0.015, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "housing_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.2475, 0.000, 0.097)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=5.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    button_zs = (0.229, 0.176, 0.123, 0.070)
    for index, button_z in enumerate(button_zs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.020, 0.010, 0.038)),
            origin=Origin(xyz=(0.000, -0.005, 0.000)),
            material=button_silver,
            name="button_body",
        )
        button.visual(
            Box((0.016, 0.004, 0.030)),
            origin=Origin(xyz=(0.000, -0.010, 0.000)),
            material=indicator_white,
            name="button_face",
        )
        button.visual(
            Box((0.010, 0.012, 0.022)),
            origin=Origin(xyz=(0.000, 0.006, 0.000)),
            material=control_black,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.020, 0.014, 0.038)),
            mass=0.03,
            origin=Origin(xyz=(0.000, -0.007, 0.000)),
        )
        model.articulation(
            f"housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(0.1985, 0.000, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.06,
                lower=0.0,
                upper=0.004,
            ),
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.145, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.013)),
        material=glass_clear,
        name="turntable_rim",
    )
    turntable.visual(
        Cylinder(radius=0.138, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=glass_clear,
        name="turntable_plate",
    )
    turntable.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=control_black,
        name="turntable_hub",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.016),
        mass=1.1,
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
    )
    model.articulation(
        "housing_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=turntable,
        origin=Origin(xyz=(-0.0625, 0.211, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    knob = object_model.get_part("control_knob")
    turntable = object_model.get_part("turntable")
    button_0 = object_model.get_part("button_0")

    door_hinge = object_model.get_articulation("housing_to_door")
    knob_joint = object_model.get_articulation("housing_to_control_knob")
    button_joint = object_model.get_articulation("housing_to_button_0")
    turntable_joint = object_model.get_articulation("housing_to_turntable")

    def _axis_ok(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))

    ctx.check(
        "door_hinge_axis",
        _axis_ok(door_hinge.axis, (0.0, 0.0, -1.0)),
        f"expected vertical side hinge, got {door_hinge.axis}",
    )
    ctx.check(
        "knob_axis_front_to_back",
        _axis_ok(knob_joint.axis, (0.0, 1.0, 0.0)),
        f"expected knob axis along depth, got {knob_joint.axis}",
    )
    ctx.check(
        "button_axis_front_to_back",
        _axis_ok(button_joint.axis, (0.0, 1.0, 0.0)),
        f"expected button plunger axis along depth, got {button_joint.axis}",
    )
    ctx.check(
        "turntable_axis_vertical",
        _axis_ok(turntable_joint.axis, (0.0, 0.0, 1.0)),
        f"expected vertical turntable axis, got {turntable_joint.axis}",
    )

    ctx.expect_contact(
        door,
        housing,
        name="door_sits_flush_against_housing",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        min_overlap=0.30,
        name="door_covers_cavity_front",
    )
    ctx.expect_contact(
        turntable,
        housing,
        elem_a="turntable_hub",
        elem_b="cavity_floor",
        name="turntable_hub_contacts_cavity_floor",
    )
    ctx.expect_within(
        turntable,
        housing,
        axes=("x", "y"),
        outer_elem="cavity_floor",
        margin=0.0,
        name="turntable_stays_within_cavity_floor",
    )

    door_closed_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    assert door_closed_aabb is not None
    door_closed = tuple(
        (door_closed_aabb[0][axis] + door_closed_aabb[1][axis]) * 0.5 for axis in range(3)
    )
    with ctx.pose({door_hinge: math.radians(90.0)}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
        assert door_open_aabb is not None
        door_open = tuple(
            (door_open_aabb[0][axis] + door_open_aabb[1][axis]) * 0.5 for axis in range(3)
        )
        ctx.check(
            "door_swings_outward",
            door_open[1] < door_closed[1] - 0.12 and door_open[0] < door_closed[0] - 0.10,
            f"closed={door_closed}, open={door_open}",
        )

    knob_rest = ctx.part_world_position(knob)
    assert knob_rest is not None
    with ctx.pose({knob_joint: 1.4}):
        knob_turned = ctx.part_world_position(knob)
        assert knob_turned is not None
        ctx.check(
            "knob_rotates_without_translating",
            math.dist(knob_rest, knob_turned) < 1e-6,
            f"rest={knob_rest}, turned={knob_turned}",
        )

    button_rest = ctx.part_world_position(button_0)
    assert button_rest is not None
    with ctx.pose({button_joint: 0.004}):
        button_pressed = ctx.part_world_position(button_0)
        assert button_pressed is not None
        ctx.check(
            "button_plunges_backward",
            button_pressed[1] > button_rest[1] + 0.003,
            f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
