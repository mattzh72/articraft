from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_island_cooktop", assets=ASSETS)

    stone = model.material("stone", rgba=(0.88, 0.87, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.19, 0.21, 1.0))
    iron = model.material("iron", rgba=(0.16, 0.16, 0.17, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.32, 0.33, 0.35, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.78, 0.78, 0.8, 1.0))
    accent = model.material("accent", rgba=(0.95, 0.3, 0.2, 1.0))

    island_top = model.part("island_top")
    island_top.visual(
        Box((0.32, 0.64, 0.04)),
        origin=Origin(xyz=(-0.59, 0.0, 0.02)),
        material=stone,
        name="left_counter_strip",
    )
    island_top.visual(
        Box((0.32, 0.64, 0.04)),
        origin=Origin(xyz=(0.59, 0.0, 0.02)),
        material=stone,
        name="right_counter_strip",
    )
    island_top.visual(
        Box((0.86, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, -0.29, 0.02)),
        material=stone,
        name="front_counter_strip",
    )
    island_top.visual(
        Box((0.86, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.29, 0.02)),
        material=stone,
        name="back_counter_strip",
    )

    cooktop_panel = model.part("cooktop_panel")
    cooktop_panel.visual(
        Box((0.86, 0.52, 0.008)),
        material=dark_glass,
        name="glass_panel",
    )
    cooktop_panel.visual(
        Box((0.84, 0.50, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="trim_plate",
    )

    model.articulation(
        "island_to_panel",
        ArticulationType.FIXED,
        parent=island_top,
        child=cooktop_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    def add_burner(
        name: str,
        xyz: tuple[float, float, float],
        tray_radius: float,
        ring_radius: float,
        cap_radius: float,
    ) -> None:
        burner = model.part(name)
        burner.visual(
            Cylinder(radius=tray_radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=burner_metal,
            name="tray",
        )
        burner.visual(
            Cylinder(radius=ring_radius, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=iron,
            name="ring",
        )
        burner.visual(
            Cylinder(radius=cap_radius, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=iron,
            name="cap",
        )
        model.articulation(
            f"panel_to_{name}",
            ArticulationType.FIXED,
            parent=cooktop_panel,
            child=burner,
            origin=Origin(xyz=xyz),
        )

    add_burner("burner_front_left", (-0.26, -0.12, 0.004), 0.052, 0.036, 0.024)
    add_burner("burner_front_right", (0.26, -0.12, 0.004), 0.052, 0.036, 0.024)
    add_burner("burner_back_left", (-0.26, 0.14, 0.004), 0.052, 0.036, 0.024)
    add_burner("burner_back_right", (0.26, 0.14, 0.004), 0.052, 0.036, 0.024)
    add_burner("burner_center", (0.0, 0.02, 0.004), 0.072, 0.05, 0.034)

    grate = model.part("grate_network")
    for x in (-0.34, 0.34):
        grate.visual(
            Box((0.02, 0.02, 0.022)),
            origin=Origin(xyz=(x, -0.17, 0.011)),
            material=iron,
        )
        grate.visual(
            Box((0.02, 0.02, 0.022)),
            origin=Origin(xyz=(x, 0.19, 0.011)),
            material=iron,
        )
    grate.visual(
        Box((0.02, 0.42, 0.012)),
        origin=Origin(xyz=(-0.34, 0.01, 0.028)),
        material=iron,
        name="left_rail",
    )
    grate.visual(
        Box((0.02, 0.42, 0.012)),
        origin=Origin(xyz=(0.34, 0.01, 0.028)),
        material=iron,
        name="right_rail",
    )
    grate.visual(
        Box((0.70, 0.02, 0.012)),
        origin=Origin(xyz=(0.0, -0.17, 0.028)),
        material=iron,
        name="front_rail",
    )
    grate.visual(
        Box((0.70, 0.02, 0.012)),
        origin=Origin(xyz=(0.0, 0.19, 0.028)),
        material=iron,
        name="back_rail",
    )
    grate.visual(
        Box((0.02, 0.42, 0.012)),
        origin=Origin(xyz=(0.0, 0.01, 0.028)),
        material=iron,
        name="center_rail",
    )
    grate.visual(
        Box((0.02, 0.42, 0.012)),
        origin=Origin(xyz=(-0.17, 0.01, 0.028)),
        material=iron,
    )
    grate.visual(
        Box((0.02, 0.42, 0.012)),
        origin=Origin(xyz=(0.17, 0.01, 0.028)),
        material=iron,
    )
    grate.visual(
        Box((0.70, 0.02, 0.012)),
        origin=Origin(xyz=(0.0, 0.01, 0.028)),
        material=iron,
        name="mid_rail",
    )
    model.articulation(
        "panel_to_grate",
        ArticulationType.FIXED,
        parent=cooktop_panel,
        child=grate,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    control_fascia = model.part("control_fascia")
    control_fascia.visual(
        Box((0.28, 0.03, 0.006)),
        origin=Origin(xyz=(0.0, 0.023, 0.048)),
        material=steel,
        name="mount_bar",
    )
    control_fascia.visual(
        Box((0.26, 0.008, 0.108)),
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
        material=steel,
        name="back_plate",
    )
    for x, stem_name in ((-0.082, "left"), (0.082, "right")):
        control_fascia.visual(
            Box((0.023, 0.03, 0.004)),
            origin=Origin(xyz=(x, 0.015, 0.0075)),
            material=steel,
            name=f"{stem_name}_knob_bearing_top",
        )
        control_fascia.visual(
            Box((0.023, 0.03, 0.004)),
            origin=Origin(xyz=(x, 0.015, -0.0075)),
            material=steel,
            name=f"{stem_name}_knob_bearing_bottom",
        )
        control_fascia.visual(
            Box((0.004, 0.03, 0.015)),
            origin=Origin(xyz=(x - 0.0075, 0.015, 0.0)),
            material=steel,
            name=f"{stem_name}_knob_bearing_left",
        )
        control_fascia.visual(
            Box((0.004, 0.03, 0.015)),
            origin=Origin(xyz=(x + 0.0075, 0.015, 0.0)),
            material=steel,
            name=f"{stem_name}_knob_bearing_right",
        )
    for x, stem_name in ((-0.033, "left"), (0.0, "center"), (0.033, "right")):
        control_fascia.visual(
            Box((0.018, 0.032, 0.004)),
            origin=Origin(xyz=(x, 0.016, -0.029)),
            material=steel,
            name=f"{stem_name}_button_guide_top",
        )
        control_fascia.visual(
            Box((0.018, 0.032, 0.004)),
            origin=Origin(xyz=(x, 0.016, -0.041)),
            material=steel,
            name=f"{stem_name}_button_guide_bottom",
        )
        control_fascia.visual(
            Box((0.004, 0.032, 0.012)),
            origin=Origin(xyz=(x - 0.007, 0.016, -0.035)),
            material=steel,
            name=f"{stem_name}_button_guide_left",
        )
        control_fascia.visual(
            Box((0.004, 0.032, 0.012)),
            origin=Origin(xyz=(x + 0.007, 0.016, -0.035)),
            material=steel,
            name=f"{stem_name}_button_guide_right",
        )
    model.articulation(
        "panel_to_controls",
        ArticulationType.FIXED,
        parent=cooktop_panel,
        child=control_fascia,
        origin=Origin(xyz=(0.0, -0.225, -0.055)),
    )

    def add_knob(name: str, x: float) -> None:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=knob_finish,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.0055, length=0.03),
            origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="shaft",
        )
        knob.visual(
            Box((0.005, 0.0025, 0.012)),
            origin=Origin(xyz=(0.0, -0.01825, 0.013)),
            material=accent,
            name="pointer",
        )
        model.articulation(
            f"controls_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=control_fascia,
            child=knob,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=6.0),
        )

    add_knob("knob_left", -0.082)
    add_knob("knob_right", 0.082)

    def add_button(name: str, x: float) -> None:
        button = model.part(name)
        button.visual(
            Box((0.014, 0.008, 0.01)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=knob_finish,
            name="cap",
        )
        button.visual(
            Box((0.012, 0.024, 0.012)),
            origin=Origin(xyz=(0.0, 0.012, 0.0)),
            material=knob_finish,
            name="stem",
        )
        button.visual(
            Box((0.008, 0.002, 0.003)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=accent,
            name="button_mark",
        )
        model.articulation(
            f"controls_to_{name}",
            ArticulationType.PRISMATIC,
            parent=control_fascia,
            child=button,
            origin=Origin(xyz=(x, 0.0, -0.035)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=0.2,
                lower=0.0,
                upper=0.008,
            ),
        )

    add_button("button_left", -0.033)
    add_button("button_center", 0.0)
    add_button("button_right", 0.033)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    island_top = object_model.get_part("island_top")
    cooktop_panel = object_model.get_part("cooktop_panel")
    grate = object_model.get_part("grate_network")
    control_fascia = object_model.get_part("control_fascia")
    burner_front_left = object_model.get_part("burner_front_left")
    burner_front_right = object_model.get_part("burner_front_right")
    burner_back_left = object_model.get_part("burner_back_left")
    burner_back_right = object_model.get_part("burner_back_right")
    burner_center = object_model.get_part("burner_center")
    knob_left = object_model.get_part("knob_left")
    knob_right = object_model.get_part("knob_right")
    button_left = object_model.get_part("button_left")
    button_center = object_model.get_part("button_center")
    button_right = object_model.get_part("button_right")

    knob_left_joint = object_model.get_articulation("controls_to_knob_left")
    knob_right_joint = object_model.get_articulation("controls_to_knob_right")
    button_left_joint = object_model.get_articulation("controls_to_button_left")
    button_center_joint = object_model.get_articulation("controls_to_button_center")
    button_right_joint = object_model.get_articulation("controls_to_button_right")

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

    ctx.expect_contact(cooktop_panel, island_top, name="panel_supported_by_counter")
    ctx.expect_contact(control_fascia, cooktop_panel, name="controls_hung_from_panel")
    ctx.expect_contact(grate, cooktop_panel, name="grate_feet_touch_panel")

    for burner in (
        burner_front_left,
        burner_front_right,
        burner_back_left,
        burner_back_right,
        burner_center,
    ):
        ctx.expect_contact(burner, cooktop_panel, name=f"{burner.name}_seated_on_panel")

    ctx.expect_contact(knob_left, control_fascia, name="left_knob_mounted")
    ctx.expect_contact(knob_right, control_fascia, name="right_knob_mounted")
    ctx.expect_contact(button_left, control_fascia, name="left_button_guided")
    ctx.expect_contact(button_center, control_fascia, name="center_button_guided")
    ctx.expect_contact(button_right, control_fascia, name="right_button_guided")

    ctx.expect_origin_distance(
        cooktop_panel,
        island_top,
        axes="xy",
        max_dist=0.001,
        name="panel_centered_in_island_top",
    )
    ctx.expect_origin_gap(
        positive_link=knob_right,
        negative_link=knob_left,
        axis="x",
        min_gap=0.15,
        max_gap=0.17,
        name="two_large_knobs_side_by_side",
    )
    ctx.expect_origin_gap(
        positive_link=button_center,
        negative_link=button_left,
        axis="x",
        min_gap=0.03,
        max_gap=0.034,
        name="left_to_center_button_spacing",
    )
    ctx.expect_origin_gap(
        positive_link=button_right,
        negative_link=button_center,
        axis="x",
        min_gap=0.03,
        max_gap=0.034,
        name="center_to_right_button_spacing",
    )
    ctx.expect_origin_gap(
        positive_link=knob_left,
        negative_link=button_center,
        axis="z",
        min_gap=0.033,
        max_gap=0.039,
        name="buttons_below_knob_row",
    )
    ctx.expect_origin_gap(
        positive_link=burner_back_left,
        negative_link=burner_front_left,
        axis="y",
        min_gap=0.24,
        max_gap=0.28,
        name="left_burner_rows_front_to_back",
    )
    ctx.expect_origin_gap(
        positive_link=burner_back_right,
        negative_link=burner_front_right,
        axis="y",
        min_gap=0.24,
        max_gap=0.28,
        name="right_burner_rows_front_to_back",
    )

    ctx.check(
        "knob_axes_front_to_back",
        knob_left_joint.axis == (0.0, 1.0, 0.0) and knob_right_joint.axis == (0.0, 1.0, 0.0),
        details=f"knob axes were {knob_left_joint.axis} and {knob_right_joint.axis}",
    )
    ctx.check(
        "button_axes_front_to_back",
        button_left_joint.axis == (0.0, 1.0, 0.0)
        and button_center_joint.axis == (0.0, 1.0, 0.0)
        and button_right_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"button axes were {button_left_joint.axis}, "
            f"{button_center_joint.axis}, {button_right_joint.axis}"
        ),
    )
    ctx.check(
        "only_controls_articulate",
        sum(
            1
            for articulation in object_model.articulations
            if articulation.articulation_type != ArticulationType.FIXED
        )
        == 5,
        details=(
            "expected exactly five moving control articulations, found "
            f"{sum(1 for articulation in object_model.articulations if articulation.articulation_type != ArticulationType.FIXED)}"
        ),
    )
    ctx.check(
        "control_joint_types",
        knob_left_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_right_joint.articulation_type == ArticulationType.CONTINUOUS
        and button_left_joint.articulation_type == ArticulationType.PRISMATIC
        and button_center_joint.articulation_type == ArticulationType.PRISMATIC
        and button_right_joint.articulation_type == ArticulationType.PRISMATIC,
        details="control articulations did not match requested continuous knob / prismatic button types",
    )

    for joint in (button_left_joint, button_center_joint, button_right_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_limits",
            limits is not None and limits.lower == 0.0 and limits.upper == 0.008,
            details=f"unexpected limits for {joint.name}: {limits}",
        )
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_pressed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_pressed_no_floating")
                if joint is button_left_joint:
                    ctx.expect_contact(button_left, control_fascia, name="left_button_pressed_guided")
                elif joint is button_center_joint:
                    ctx.expect_contact(
                        button_center, control_fascia, name="center_button_pressed_guided"
                    )
                else:
                    ctx.expect_contact(button_right, control_fascia, name="right_button_pressed_guided")

    for joint in (knob_left_joint, knob_right_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"continuous knob joint {joint.name} should not have lower/upper bounds: {limits}",
        )

    with ctx.pose({knob_left_joint: pi / 2.0, knob_right_joint: -pi / 3.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_rotation_no_overlap")
        ctx.expect_contact(knob_left, control_fascia, name="left_knob_stays_mounted_when_rotated")
        ctx.expect_contact(knob_right, control_fascia, name="right_knob_stays_mounted_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
