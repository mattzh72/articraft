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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _button_housing_mesh():
    outer_profile = [
        (0.058, 0.000),
        (0.065, 0.006),
        (0.065, 0.050),
    ]
    inner_profile = [
        (0.046, 0.000),
        (0.050, 0.006),
        (0.050, 0.050),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
        ),
        "payout_button_housing",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_base_casino_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.63, 0.09, 0.12, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_gold = model.material("trim_gold", rgba=(0.86, 0.69, 0.26, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.20, 0.27, 0.31, 0.38))
    chrome = model.material("chrome", rgba=(0.86, 0.87, 0.88, 1.0))
    reel_white = model.material("reel_white", rgba=(0.96, 0.95, 0.92, 1.0))
    reel_symbol = model.material("reel_symbol", rgba=(0.78, 0.08, 0.10, 1.0))
    button_red = model.material("button_red", rgba=(0.84, 0.10, 0.14, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.76, 0.60, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_panel,
        name="plinth",
    )
    cabinet.visual(
        Box((0.66, 0.46, 0.55)),
        origin=Origin(xyz=(0.0, -0.03, 0.495)),
        material=cabinet_red,
        name="lower_body",
    )
    cabinet.visual(
        Box((0.70, 0.24, 0.05)),
        origin=Origin(xyz=(0.0, 0.20, 0.795)),
        material=trim_gold,
        name="shelf_deck",
    )
    cabinet.visual(
        Box((0.70, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, 0.295, 0.770)),
        material=dark_panel,
        name="shelf_front",
    )
    cabinet.visual(
        Box((0.12, 0.32, 0.56)),
        origin=Origin(xyz=(-0.27, 0.02, 1.10)),
        material=cabinet_red,
        name="upper_left_side",
    )
    cabinet.visual(
        Box((0.12, 0.32, 0.56)),
        origin=Origin(xyz=(0.27, 0.02, 1.10)),
        material=cabinet_red,
        name="upper_right_side",
    )
    cabinet.visual(
        Box((0.66, 0.32, 0.12)),
        origin=Origin(xyz=(0.0, 0.02, 0.88)),
        material=cabinet_red,
        name="window_sill",
    )
    cabinet.visual(
        Box((0.66, 0.32, 0.14)),
        origin=Origin(xyz=(0.0, 0.02, 1.31)),
        material=cabinet_red,
        name="window_header",
    )
    cabinet.visual(
        Box((0.66, 0.05, 0.46)),
        origin=Origin(xyz=(0.0, -0.115, 1.095)),
        material=dark_panel,
        name="upper_back",
    )
    cabinet.visual(
        Box((0.70, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.00, 1.47)),
        material=cabinet_red,
        name="marquee_top",
    )
    cabinet.visual(
        Box((0.05, 0.03, 0.30)),
        origin=Origin(xyz=(-0.205, 0.195, 1.09)),
        material=trim_gold,
        name="window_bezel_left",
    )
    cabinet.visual(
        Box((0.05, 0.03, 0.30)),
        origin=Origin(xyz=(0.205, 0.195, 1.09)),
        material=trim_gold,
        name="window_bezel_right",
    )
    cabinet.visual(
        Box((0.46, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.195, 1.22)),
        material=trim_gold,
        name="window_bezel_top",
    )
    cabinet.visual(
        Box((0.46, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.195, 0.96)),
        material=trim_gold,
        name="window_bezel_bottom",
    )
    cabinet.visual(
        Box((0.40, 0.004, 0.24)),
        origin=Origin(xyz=(0.0, 0.182, 1.09)),
        material=smoked_glass,
        name="reel_glass",
    )
    cabinet.visual(
        Box((0.015, 0.12, 0.14)),
        origin=Origin(xyz=(0.3375, 0.02, 1.08)),
        material=chrome,
        name="lever_bracket",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.76, 0.60, 1.56)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    reel_bank = model.part("reel_bank")
    reel_bank.visual(
        Box((0.40, 0.04, 0.26)),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        material=dark_panel,
        name="reel_backplate",
    )
    reel_positions = (-0.13, 0.0, 0.13)
    for index, x_pos in enumerate(reel_positions):
        reel_bank.visual(
            Cylinder(radius=0.078, length=0.09),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_white,
            name=f"reel_{index}",
        )
        reel_bank.visual(
            Box((0.050, 0.014, 0.050)),
            origin=Origin(xyz=(x_pos, 0.068, 0.0)),
            material=reel_symbol,
            name=f"reel_symbol_{index}",
        )
    reel_bank.inertial = Inertial.from_geometry(
        Box((0.40, 0.16, 0.26)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "cabinet_to_reel_bank",
        ArticulationType.FIXED,
        parent=cabinet,
        child=reel_bank,
        origin=Origin(xyz=(0.0, -0.005, 1.09)),
    )

    button_housing = model.part("button_housing")
    button_housing.visual(
        _button_housing_mesh(),
        material=chrome,
        name="housing_shell",
    )
    button_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.050),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )
    model.articulation(
        "cabinet_to_button_housing",
        ArticulationType.FIXED,
        parent=cabinet,
        child=button_housing,
        origin=Origin(xyz=(0.0, 0.25, 0.82)),
    )

    payout_button = model.part("payout_button")
    payout_button.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=button_red,
        name="button_cap",
    )
    payout_button.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=chrome,
        name="button_guide",
    )
    payout_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.018),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )
    model.articulation(
        "housing_to_payout_button",
        ArticulationType.PRISMATIC,
        parent=button_housing,
        child=payout_button,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.12,
            lower=0.0,
            upper=0.015,
        ),
    )

    lever = model.part("side_lever")
    lever.visual(
        Cylinder(radius=0.015, length=0.06),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lever_hub",
    )
    lever.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.030, 0.0, -0.028), rpy=(0.0, 2.23, 0.0)),
        material=chrome,
        name="lever_neck",
    )
    lever.visual(
        Cylinder(radius=0.013, length=0.24),
        origin=Origin(xyz=(0.102, 0.0, -0.094), rpy=(0.0, 2.23, 0.0)),
        material=chrome,
        name="lever_arm",
    )
    lever.visual(
        Sphere(radius=0.038),
        origin=Origin(xyz=(0.205, 0.0, -0.184)),
        material=knob_black,
        name="lever_knob",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.30, 0.08, 0.30)),
        mass=0.55,
        origin=Origin(xyz=(0.12, 0.0, -0.10)),
    )
    model.articulation(
        "cabinet_to_side_lever",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lever,
        origin=Origin(xyz=(0.36, 0.02, 1.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.0,
        ),
    )

    access_panel = model.part("front_access_panel")
    access_panel.visual(
        Box((0.52, 0.025, 0.42)),
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
        material=dark_panel,
        name="door_leaf",
    )
    access_panel.visual(
        Box((0.41, 0.008, 0.028)),
        origin=Origin(xyz=(0.265, 0.0165, 0.165)),
        material=trim_gold,
        name="door_frame_top",
    )
    access_panel.visual(
        Box((0.41, 0.008, 0.028)),
        origin=Origin(xyz=(0.265, 0.0165, -0.165)),
        material=trim_gold,
        name="door_frame_bottom",
    )
    access_panel.visual(
        Box((0.030, 0.008, 0.34)),
        origin=Origin(xyz=(0.060, 0.0165, 0.0)),
        material=trim_gold,
        name="door_frame_left",
    )
    access_panel.visual(
        Box((0.030, 0.008, 0.34)),
        origin=Origin(xyz=(0.470, 0.0165, 0.0)),
        material=trim_gold,
        name="door_frame_right",
    )
    access_panel.visual(
        Cylinder(radius=0.016, length=0.045),
        origin=Origin(xyz=(0.455, 0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="door_pull",
    )
    access_panel.inertial = Inertial.from_geometry(
        Box((0.52, 0.045, 0.42)),
        mass=6.5,
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
    )
    model.articulation(
        "cabinet_to_front_access_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=access_panel,
        origin=Origin(xyz=(-0.26, 0.2125, 0.48)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    reel_bank = object_model.get_part("reel_bank")
    button_housing = object_model.get_part("button_housing")
    payout_button = object_model.get_part("payout_button")
    lever = object_model.get_part("side_lever")
    access_panel = object_model.get_part("front_access_panel")

    payout_joint = object_model.get_articulation("housing_to_payout_button")
    lever_joint = object_model.get_articulation("cabinet_to_side_lever")
    panel_joint = object_model.get_articulation("cabinet_to_front_access_panel")

    ctx.expect_contact(
        button_housing,
        cabinet,
        elem_a="housing_shell",
        elem_b="shelf_deck",
        contact_tol=0.001,
        name="button housing seats on the shelf",
    )
    ctx.expect_origin_distance(
        button_housing,
        cabinet,
        axes="x",
        max_dist=0.001,
        name="payout button housing stays centered on the shelf",
    )
    ctx.expect_within(
        payout_button,
        button_housing,
        axes="xy",
        margin=0.028,
        inner_elem="button_cap",
        outer_elem="housing_shell",
        name="payout button stays within the housing footprint",
    )
    ctx.expect_gap(
        payout_button,
        cabinet,
        axis="z",
        positive_elem="button_cap",
        negative_elem="shelf_deck",
        min_gap=0.004,
        name="payout button cap remains above the shelf deck",
    )
    ctx.expect_contact(
        lever,
        cabinet,
        elem_a="lever_hub",
        elem_b="lever_bracket",
        contact_tol=0.0015,
        name="side lever hub is supported by the cabinet bracket",
    )
    ctx.expect_overlap(
        access_panel,
        cabinet,
        axes="xz",
        elem_a="door_leaf",
        elem_b="lower_body",
        min_overlap=0.35,
        name="closed access panel covers the front service opening area",
    )
    ctx.expect_gap(
        access_panel,
        cabinet,
        axis="y",
        positive_elem="door_leaf",
        negative_elem="lower_body",
        max_gap=0.001,
        max_penetration=0.0001,
        name="closed access panel sits flush on the cabinet front",
    )
    ctx.expect_overlap(
        reel_bank,
        cabinet,
        axes="xz",
        elem_a="reel_backplate",
        elem_b="reel_glass",
        min_overlap=0.20,
        name="reel bank sits behind the classic reel window",
    )
    ctx.expect_contact(
        reel_bank,
        cabinet,
        elem_a="reel_backplate",
        elem_b="upper_back",
        contact_tol=0.001,
        name="reel bank backplate mounts to the cabinet back",
    )

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    with ctx.pose({payout_joint: 0.0}):
        rest_button_aabb = ctx.part_element_world_aabb(payout_button, elem="button_cap")
    with ctx.pose({payout_joint: payout_joint.motion_limits.upper}):
        pressed_button_aabb = ctx.part_element_world_aabb(payout_button, elem="button_cap")
        ctx.expect_gap(
            payout_button,
            cabinet,
            axis="z",
            positive_elem="button_cap",
            negative_elem="shelf_deck",
            min_gap=0.0,
            name="pressed payout button still clears the shelf deck",
        )
    rest_button_z = _center_z(rest_button_aabb)
    pressed_button_z = _center_z(pressed_button_aabb)
    ctx.check(
        "payout button depresses downward",
        rest_button_z is not None
        and pressed_button_z is not None
        and pressed_button_z < rest_button_z - 0.010,
        details=f"rest_z={rest_button_z}, pressed_z={pressed_button_z}",
    )

    with ctx.pose({lever_joint: 0.0}):
        rest_knob_aabb = ctx.part_element_world_aabb(lever, elem="lever_knob")
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        pulled_knob_aabb = ctx.part_element_world_aabb(lever, elem="lever_knob")
    rest_knob_z = _center_z(rest_knob_aabb)
    pulled_knob_z = _center_z(pulled_knob_aabb)
    ctx.check(
        "side lever pulls downward around its transverse pivot",
        rest_knob_z is not None
        and pulled_knob_z is not None
        and pulled_knob_z < rest_knob_z - 0.080,
        details=f"rest_z={rest_knob_z}, pulled_z={pulled_knob_z}",
    )

    with ctx.pose({panel_joint: 0.0}):
        closed_panel_aabb = ctx.part_element_world_aabb(access_panel, elem="door_leaf")
    with ctx.pose({panel_joint: 1.2}):
        open_panel_aabb = ctx.part_element_world_aabb(access_panel, elem="door_leaf")
    closed_front = None if closed_panel_aabb is None else closed_panel_aabb[1][1]
    open_front = None if open_panel_aabb is None else open_panel_aabb[1][1]
    ctx.check(
        "front access panel swings outward on its vertical side hinge",
        closed_front is not None
        and open_front is not None
        and open_front > closed_front + 0.18,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
