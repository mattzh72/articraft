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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_hob", assets=ASSETS)

    counter_stone = model.material("counter_stone", rgba=(0.25, 0.25, 0.27, 1.0))
    glass_black = model.material("glass_black", rgba=(0.08, 0.08, 0.09, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.44, 0.45, 0.47, 1.0))
    cap_black = model.material("cap_black", rgba=(0.14, 0.14, 0.15, 1.0))
    grate_iron = model.material("grate_iron", rgba=(0.12, 0.12, 0.13, 1.0))
    control_silver = model.material("control_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    control_dark = model.material("control_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    counter_width = 1.00
    counter_depth = 0.64
    counter_thickness = 0.04
    cutout_width = 0.72
    cutout_depth = 0.48
    panel_width = 0.76
    panel_depth = 0.52
    panel_thickness = 0.008
    panel_top_z = panel_thickness

    counter = model.part("counter")
    side_strip_width = (counter_width - cutout_width) * 0.5
    front_strip_depth = (counter_depth - cutout_depth) * 0.5
    counter.visual(
        Box((side_strip_width, counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                -cutout_width * 0.5 - side_strip_width * 0.5,
                0.0,
                counter_thickness * 0.5,
            )
        ),
        material=counter_stone,
        name="left_counter_strip",
    )
    counter.visual(
        Box((side_strip_width, counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                cutout_width * 0.5 + side_strip_width * 0.5,
                0.0,
                counter_thickness * 0.5,
            )
        ),
        material=counter_stone,
        name="right_counter_strip",
    )
    counter.visual(
        Box((cutout_width, front_strip_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -cutout_depth * 0.5 - front_strip_depth * 0.5,
                counter_thickness * 0.5,
            )
        ),
        material=counter_stone,
        name="front_counter_strip",
    )
    counter.visual(
        Box((cutout_width, front_strip_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cutout_depth * 0.5 + front_strip_depth * 0.5,
                counter_thickness * 0.5,
            )
        ),
        material=counter_stone,
        name="rear_counter_strip",
    )
    counter.inertial = Inertial.from_geometry(
        Box((counter_width, counter_depth, counter_thickness)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, counter_thickness * 0.5)),
    )

    hob_panel = model.part("hob_panel")
    panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(panel_width, panel_depth, radius=0.022),
            panel_thickness,
            cap=True,
            center=False,
            closed=True,
        ),
        ASSETS.mesh_path("hob_glass_panel.obj"),
    )
    hob_panel.visual(
        panel_mesh,
        origin=Origin(),
        material=glass_black,
        name="panel_surface",
    )
    hob_panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_depth, panel_thickness)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, panel_thickness * 0.5)),
    )

    burner_layout = {
        "burner_left_front": ((-0.075, -0.145), 0.034, 0.022, 0.17),
        "burner_left_rear": ((-0.075, 0.145), 0.034, 0.022, 0.17),
        "burner_center": ((0.095, 0.000), 0.050, 0.032, 0.28),
        "burner_right_front": ((0.240, -0.145), 0.040, 0.026, 0.22),
        "burner_right_rear": ((0.240, 0.145), 0.040, 0.026, 0.22),
    }

    for burner_name, (xy, ring_radius, cap_radius, _mass) in burner_layout.items():
        hob_panel.visual(
            Cylinder(radius=ring_radius + 0.010, length=0.0015),
            origin=Origin(xyz=(xy[0], xy[1], panel_top_z + 0.00075)),
            material=burner_metal,
            name=f"{burner_name}_trim",
        )

    button_positions = {
        "button_upper": (0.314, 0.060),
        "button_lower": (0.314, -0.060),
    }
    for button_name, (button_x, button_y) in button_positions.items():
        hob_panel.visual(
            Cylinder(radius=0.014, length=0.0025),
            origin=Origin(
                xyz=(button_x, button_y, panel_top_z + 0.00125)
            ),
            material=control_dark,
            name=f"{button_name}_bezel",
        )

    model.articulation(
        "counter_to_hob_panel",
        ArticulationType.FIXED,
        parent=counter,
        child=hob_panel,
        origin=Origin(xyz=(0.0, 0.0, counter_thickness)),
    )

    def add_burner(
        name: str,
        *,
        xy: tuple[float, float],
        ring_radius: float,
        cap_radius: float,
        mass: float,
    ) -> None:
        burner = model.part(name)
        burner.visual(
            Cylinder(radius=ring_radius, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
            material=burner_metal,
            name="burner_ring",
        )
        burner.visual(
            Cylinder(radius=ring_radius * 0.86, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0065)),
            material=burner_metal,
            name="burner_crown",
        )
        burner.visual(
            Cylinder(radius=cap_radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
            material=cap_black,
            name="burner_cap",
        )
        burner.inertial = Inertial.from_geometry(
            Cylinder(radius=ring_radius, length=0.014),
            mass=mass,
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
        )
        model.articulation(
            f"hob_panel_to_{name}",
            ArticulationType.FIXED,
            parent=hob_panel,
            child=burner,
            origin=Origin(xyz=(xy[0], xy[1], panel_top_z)),
        )

    def add_pair_grate(
        name: str,
        *,
        xy: tuple[float, float],
        span_x: float,
        span_y: float,
    ) -> None:
        grate = model.part(name)
        foot_height = 0.018
        foot_x = span_x * 0.5 - 0.012
        foot_y = span_y * 0.5 - 0.018
        top_z = 0.021
        bar_thickness = 0.006
        rail_width = 0.008

        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                grate.visual(
                    Box((0.014, 0.014, foot_height)),
                    origin=Origin(
                        xyz=(sign_x * foot_x, sign_y * foot_y, foot_height * 0.5)
                    ),
                    material=grate_iron,
                    name=f"foot_{int(sign_x)}_{int(sign_y)}",
                )

        grate.visual(
            Box((span_x, rail_width, bar_thickness)),
            origin=Origin(xyz=(0.0, foot_y, top_z)),
            material=grate_iron,
            name="outer_front_bar",
        )
        grate.visual(
            Box((span_x, rail_width, bar_thickness)),
            origin=Origin(xyz=(0.0, -foot_y, top_z)),
            material=grate_iron,
            name="outer_rear_bar",
        )
        grate.visual(
            Box((rail_width, span_y, bar_thickness)),
            origin=Origin(xyz=(foot_x, 0.0, top_z)),
            material=grate_iron,
            name="outer_right_bar",
        )
        grate.visual(
            Box((rail_width, span_y, bar_thickness)),
            origin=Origin(xyz=(-foot_x, 0.0, top_z)),
            material=grate_iron,
            name="outer_left_bar",
        )
        grate.visual(
            Box((span_x * 0.78, rail_width, bar_thickness)),
            origin=Origin(xyz=(0.0, 0.055, top_z)),
            material=grate_iron,
            name="inner_rear_support",
        )
        grate.visual(
            Box((span_x * 0.78, rail_width, bar_thickness)),
            origin=Origin(xyz=(0.0, -0.055, top_z)),
            material=grate_iron,
            name="inner_front_support",
        )
        grate.visual(
            Box((rail_width, span_y * 0.72, bar_thickness)),
            origin=Origin(xyz=(0.0, 0.0, top_z)),
            material=grate_iron,
            name="center_spine",
        )
        grate.inertial = Inertial.from_geometry(
            Box((span_x, span_y, 0.024)),
            mass=0.85,
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
        )
        model.articulation(
            f"hob_panel_to_{name}",
            ArticulationType.FIXED,
            parent=hob_panel,
            child=grate,
            origin=Origin(xyz=(xy[0], xy[1], panel_top_z)),
        )

    def add_center_grate(
        name: str,
        *,
        xy: tuple[float, float],
    ) -> None:
        grate = model.part(name)
        foot_height = 0.018
        top_z = 0.021
        bar_thickness = 0.006
        half_x = 0.050
        half_y = 0.070
        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                grate.visual(
                    Box((0.014, 0.014, foot_height)),
                    origin=Origin(
                        xyz=(sign_x * half_x, sign_y * half_y, foot_height * 0.5)
                    ),
                    material=grate_iron,
                    name=f"foot_{int(sign_x)}_{int(sign_y)}",
                )
        grate.visual(
            Box((0.120, 0.008, bar_thickness)),
            origin=Origin(xyz=(0.0, half_y, top_z)),
            material=grate_iron,
            name="top_bar",
        )
        grate.visual(
            Box((0.120, 0.008, bar_thickness)),
            origin=Origin(xyz=(0.0, -half_y, top_z)),
            material=grate_iron,
            name="bottom_bar",
        )
        grate.visual(
            Box((0.008, 0.152, bar_thickness)),
            origin=Origin(xyz=(half_x, 0.0, top_z)),
            material=grate_iron,
            name="right_bar",
        )
        grate.visual(
            Box((0.008, 0.152, bar_thickness)),
            origin=Origin(xyz=(-half_x, 0.0, top_z)),
            material=grate_iron,
            name="left_bar",
        )
        grate.visual(
            Box((0.100, 0.008, bar_thickness)),
            origin=Origin(xyz=(0.0, 0.0, top_z)),
            material=grate_iron,
            name="center_cross_x",
        )
        grate.visual(
            Box((0.008, 0.140, bar_thickness)),
            origin=Origin(xyz=(0.0, 0.0, top_z)),
            material=grate_iron,
            name="center_cross_y",
        )
        grate.inertial = Inertial.from_geometry(
            Box((0.120, 0.160, 0.024)),
            mass=0.65,
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
        )
        model.articulation(
            f"hob_panel_to_{name}",
            ArticulationType.FIXED,
            parent=hob_panel,
            child=grate,
            origin=Origin(xyz=(xy[0], xy[1], panel_top_z)),
        )

    def add_knob(
        name: str,
        *,
        xy: tuple[float, float],
    ) -> None:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.021, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=control_silver,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
            material=control_dark,
            name="knob_cap",
        )
        knob.visual(
            Box((0.004, 0.013, 0.003)),
            origin=Origin(xyz=(0.0, 0.010, 0.0275)),
            material=control_silver,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.021, length=0.030),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
        )
        model.articulation(
            f"hob_panel_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=hob_panel,
            child=knob,
            origin=Origin(xyz=(xy[0], xy[1], panel_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.2, velocity=8.0),
        )

    def add_button(
        name: str,
        *,
        xy: tuple[float, float],
    ) -> None:
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=control_silver,
            name="button_body",
        )
        button.visual(
            Cylinder(radius=0.008, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=control_dark,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=0.010, length=0.008),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
        )
        model.articulation(
            f"hob_panel_to_{name}",
            ArticulationType.PRISMATIC,
            parent=hob_panel,
            child=button,
            origin=Origin(xyz=(xy[0], xy[1], panel_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=0.002,
            ),
        )

    for burner_name, (xy, ring_radius, cap_radius, mass) in burner_layout.items():
        add_burner(
            burner_name,
            xy=xy,
            ring_radius=ring_radius,
            cap_radius=cap_radius,
            mass=mass,
        )

    add_pair_grate("left_grate", xy=(-0.075, 0.0), span_x=0.130, span_y=0.320)
    add_center_grate("center_grate", xy=(0.095, 0.0))
    add_pair_grate("right_grate", xy=(0.240, 0.0), span_x=0.130, span_y=0.320)

    add_knob("knob_front", xy=(-0.285, -0.145))
    add_knob("knob_center", xy=(-0.225, -0.055))
    add_knob("knob_rear", xy=(-0.285, 0.035))

    for button_name, button_xy in button_positions.items():
        add_button(button_name, xy=button_xy)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    counter = object_model.get_part("counter")
    hob_panel = object_model.get_part("hob_panel")

    burner_names = (
        "burner_left_front",
        "burner_left_rear",
        "burner_center",
        "burner_right_front",
        "burner_right_rear",
    )
    grate_names = ("left_grate", "center_grate", "right_grate")
    knob_names = ("knob_front", "knob_center", "knob_rear")
    button_names = ("button_upper", "button_lower")

    burners = {name: object_model.get_part(name) for name in burner_names}
    grates = {name: object_model.get_part(name) for name in grate_names}
    knobs = {name: object_model.get_part(name) for name in knob_names}
    buttons = {name: object_model.get_part(name) for name in button_names}

    knob_joints = {
        name: object_model.get_articulation(f"hob_panel_to_{name}") for name in knob_names
    }
    button_joints = {
        name: object_model.get_articulation(f"hob_panel_to_{name}") for name in button_names
    }

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(hob_panel, counter, name="hob_panel_supported_by_counter")
    ctx.expect_gap(
        hob_panel,
        counter,
        axis="z",
        max_gap=1e-6,
        max_penetration=1e-6,
        name="hob_panel_flush_on_counter",
    )

    for name, burner in burners.items():
        ctx.expect_contact(burner, hob_panel, name=f"{name}_mounted_to_panel")
        ctx.expect_overlap(
            burner,
            hob_panel,
            axes="xy",
            min_overlap=0.040,
            name=f"{name}_over_panel_footprint",
        )

    for name, grate in grates.items():
        ctx.expect_contact(grate, hob_panel, name=f"{name}_rests_on_panel")
        ctx.expect_overlap(
            grate,
            hob_panel,
            axes="xy",
            min_overlap=0.080,
            name=f"{name}_within_panel_area",
        )

    for name, knob in knobs.items():
        ctx.expect_contact(knob, hob_panel, name=f"{name}_mounted")
        joint = knob_joints[name]
        limits = joint.motion_limits
        ctx.check(
            f"{name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    for name, button in buttons.items():
        joint = button_joints[name]
        limits = joint.motion_limits
        ctx.expect_contact(button, hob_panel, name=f"{name}_guided_in_panel_frame")
        ctx.check(
            f"{name}_is_prismatic_button",
            joint.articulation_type == ArticulationType.PRISMATIC
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == 0.002
            and tuple(joint.axis) == (0.0, 0.0, -1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    def _center_of_aabb(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    def _size_of_aabb(aabb):
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    knob_positions = {
        name: ctx.part_world_position(part) for name, part in knobs.items()
    }
    ctx.check(
        "knob_cluster_on_left",
        all(position is not None and position[0] < -0.18 for position in knob_positions.values()),
        details=str(knob_positions),
    )
    ctx.check(
        "knob_cluster_triangular",
        knob_positions["knob_front"] is not None
        and knob_positions["knob_center"] is not None
        and knob_positions["knob_rear"] is not None
        and abs(knob_positions["knob_front"][0] - knob_positions["knob_rear"][0]) < 0.004
        and knob_positions["knob_center"][0] > knob_positions["knob_front"][0] + 0.045
        and knob_positions["knob_front"][1] < knob_positions["knob_center"][1] < knob_positions["knob_rear"][1],
        details=str(knob_positions),
    )

    button_positions = {
        name: ctx.part_world_position(part) for name, part in buttons.items()
    }
    ctx.check(
        "buttons_stacked_on_right",
        button_positions["button_upper"] is not None
        and button_positions["button_lower"] is not None
        and button_positions["button_upper"][0] > 0.28
        and abs(button_positions["button_upper"][0] - button_positions["button_lower"][0]) < 0.002
        and button_positions["button_upper"][1] > button_positions["button_lower"][1] + 0.10,
        details=str(button_positions),
    )

    center_burner_aabb = ctx.part_world_aabb(burners["burner_center"])
    left_burner_aabb = ctx.part_world_aabb(burners["burner_left_front"])
    right_burner_aabb = ctx.part_world_aabb(burners["burner_right_front"])
    assert center_burner_aabb is not None
    assert left_burner_aabb is not None
    assert right_burner_aabb is not None
    center_dims = _size_of_aabb(center_burner_aabb)
    left_dims = _size_of_aabb(left_burner_aabb)
    right_dims = _size_of_aabb(right_burner_aabb)
    ctx.check(
        "center_burner_largest",
        center_dims[0] > left_dims[0] + 0.020 and center_dims[0] > right_dims[0] + 0.012,
        details=f"center={center_dims}, left={left_dims}, right={right_dims}",
    )

    no_sub_counter_parts: list[str] = []
    for name, part in (
        [("hob_panel", hob_panel)]
        + list(burners.items())
        + list(grates.items())
        + list(knobs.items())
        + list(buttons.items())
    ):
        aabb = ctx.part_world_aabb(part)
        assert aabb is not None
        if aabb[0][2] < 0.039999:
            no_sub_counter_parts.append(f"{name}:{aabb[0][2]:.4f}")
    ctx.check(
        "no_enclosed_body_below_counter",
        not no_sub_counter_parts,
        details=", ".join(no_sub_counter_parts),
    )

    for name, knob in knobs.items():
        joint = knob_joints[name]
        rest_indicator = ctx.part_element_world_aabb(knob, elem="indicator")
        rest_origin = ctx.part_world_position(knob)
        assert rest_indicator is not None
        assert rest_origin is not None
        rest_indicator_center = _center_of_aabb(rest_indicator)
        with ctx.pose({joint: math.pi * 0.5}):
            turned_indicator = ctx.part_element_world_aabb(knob, elem="indicator")
            turned_origin = ctx.part_world_position(knob)
            assert turned_indicator is not None
            assert turned_origin is not None
            turned_indicator_center = _center_of_aabb(turned_indicator)
            ctx.expect_contact(knob, hob_panel, name=f"{name}_quarter_turn_still_mounted")
            ctx.check(
                f"{name}_indicator_rotates",
                abs(turned_indicator_center[0] - rest_indicator_center[0]) > 0.008
                and abs(turned_indicator_center[1] - rest_indicator_center[1]) > 0.008
                and abs(turned_origin[0] - rest_origin[0]) < 1e-6
                and abs(turned_origin[1] - rest_origin[1]) < 1e-6,
                details=(
                    f"rest_indicator={rest_indicator_center}, "
                    f"turned_indicator={turned_indicator_center}, "
                    f"rest_origin={rest_origin}, turned_origin={turned_origin}"
                ),
            )

    for name, button in buttons.items():
        joint = button_joints[name]
        limits = joint.motion_limits
        assert limits is not None and limits.upper is not None
        rest_position = ctx.part_world_position(button)
        assert rest_position is not None
        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            assert pressed_position is not None
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{name}_pressed_no_floating")
            ctx.expect_contact(button, hob_panel, name=f"{name}_pressed_still_guided")
            ctx.check(
                f"{name}_moves_inward",
                pressed_position[2] < rest_position[2] - 0.0015,
                details=f"rest={rest_position}, pressed={pressed_position}",
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
