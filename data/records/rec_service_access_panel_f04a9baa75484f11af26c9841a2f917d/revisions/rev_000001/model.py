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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_bolt_heads(
    part,
    *,
    xs: tuple[float, ...],
    zs: tuple[float, ...],
    y: float,
    material: str,
    prefix: str,
    radius: float = 0.0055,
    length: float = 0.004,
) -> None:
    index = 0
    for x in xs:
        for z in zs:
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=material,
                name=f"{prefix}_{index}",
            )
            index += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_service_access_panel")

    model.material("frame_gray", rgba=(0.25, 0.27, 0.29, 1.0))
    model.material("door_gray", rgba=(0.69, 0.70, 0.71, 1.0))
    model.material("steel", rgba=(0.48, 0.49, 0.51, 1.0))
    model.material("dark_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("safety_yellow", rgba=(0.92, 0.76, 0.11, 1.0))

    frame = model.part("frame")
    panel = model.part("service_panel")

    # Overall envelope: a deep framed opening with a heavy overlay panel.
    hinge_axis_x = -0.325
    hinge_axis_y = 0.018

    # Frame front ring
    frame.visual(
        Box((0.08, 0.012, 0.92)),
        origin=Origin(xyz=(-0.320, 0.002, 0.0)),
        material="frame_gray",
        name="left_jamb_face",
    )
    frame.visual(
        Box((0.08, 0.012, 0.92)),
        origin=Origin(xyz=(0.320, 0.002, 0.0)),
        material="frame_gray",
        name="right_jamb_face",
    )
    frame.visual(
        Box((0.56, 0.012, 0.08)),
        origin=Origin(xyz=(0.0, 0.002, 0.420)),
        material="frame_gray",
        name="top_header_face",
    )
    frame.visual(
        Box((0.56, 0.012, 0.08)),
        origin=Origin(xyz=(0.0, 0.002, -0.420)),
        material="frame_gray",
        name="bottom_sill_face",
    )

    # Sleeve depth and rear ring so the opening reads as a supported structure.
    frame.visual(
        Box((0.014, 0.070, 0.78)),
        origin=Origin(xyz=(-0.276, -0.035, 0.0)),
        material="frame_gray",
        name="left_return_wall",
    )
    frame.visual(
        Box((0.014, 0.070, 0.78)),
        origin=Origin(xyz=(0.276, -0.035, 0.0)),
        material="frame_gray",
        name="right_return_wall",
    )
    frame.visual(
        Box((0.56, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, -0.035, 0.384)),
        material="frame_gray",
        name="top_return_wall",
    )
    frame.visual(
        Box((0.56, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, -0.035, -0.384)),
        material="frame_gray",
        name="bottom_return_wall",
    )
    frame.visual(
        Box((0.08, 0.008, 0.92)),
        origin=Origin(xyz=(-0.320, -0.074, 0.0)),
        material="frame_gray",
        name="left_rear_jamb",
    )
    frame.visual(
        Box((0.08, 0.008, 0.92)),
        origin=Origin(xyz=(0.320, -0.074, 0.0)),
        material="frame_gray",
        name="right_rear_jamb",
    )
    frame.visual(
        Box((0.56, 0.008, 0.08)),
        origin=Origin(xyz=(0.0, -0.074, 0.420)),
        material="frame_gray",
        name="top_rear_header",
    )
    frame.visual(
        Box((0.56, 0.008, 0.08)),
        origin=Origin(xyz=(0.0, -0.074, -0.420)),
        material="frame_gray",
        name="bottom_rear_sill",
    )

    # Hinge side reinforcement, guard, and barrels.
    frame.visual(
        Box((0.046, 0.008, 0.78)),
        origin=Origin(xyz=(-0.337, 0.012, 0.0)),
        material="steel",
        name="hinge_doubler",
    )
    frame.visual(
        Box((0.024, 0.040, 0.84)),
        origin=Origin(xyz=(-0.349, 0.014, 0.0)),
        material="safety_yellow",
        name="hinge_guard",
    )
    frame.visual(
        Box((0.080, 0.006, 0.020)),
        origin=Origin(xyz=(-0.328, 0.010, 0.338), rpy=(0.0, 0.78, 0.0)),
        material="steel",
        name="upper_hinge_brace",
    )
    frame.visual(
        Box((0.080, 0.006, 0.020)),
        origin=Origin(xyz=(-0.328, 0.010, -0.338), rpy=(0.0, -0.78, 0.0)),
        material="steel",
        name="lower_hinge_brace",
    )
    for name, z_center, length in (
        ("hinge_barrel_top", 0.265, 0.165),
        ("hinge_barrel_mid", 0.0, 0.190),
        ("hinge_barrel_bottom", -0.265, 0.165),
    ):
        frame.visual(
            Cylinder(radius=0.011, length=length),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, z_center)),
            material="dark_steel",
            name=name,
        )

    # Latch keeper, lockout staple, and stop blocks on the exposed latch jamb.
    frame.visual(
        Box((0.032, 0.024, 0.22)),
        origin=Origin(xyz=(0.338, 0.020, 0.0)),
        material="safety_yellow",
        name="keeper_block",
    )
    frame.visual(
        Box((0.012, 0.020, 0.105)),
        origin=Origin(xyz=(0.322, 0.024, 0.0)),
        material="steel",
        name="strike_face",
    )
    frame.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(0.336, 0.023, 0.116)),
        material="steel",
        name="lockout_tab_lower",
    )
    frame.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(0.336, 0.023, 0.154)),
        material="steel",
        name="lockout_tab_upper",
    )
    frame.visual(
        Cylinder(radius=0.005, length=0.048),
        origin=Origin(xyz=(0.336, 0.029, 0.135)),
        material="dark_steel",
        name="lockout_staple",
    )
    frame.visual(
        Box((0.036, 0.018, 0.030)),
        origin=Origin(xyz=(-0.334, 0.022, 0.435)),
        material="safety_yellow",
        name="upper_overtravel_stop",
    )
    frame.visual(
        Box((0.036, 0.018, 0.030)),
        origin=Origin(xyz=(-0.334, 0.022, -0.435)),
        material="safety_yellow",
        name="lower_overtravel_stop",
    )

    _add_bolt_heads(
        frame,
        xs=(-0.344, -0.330),
        zs=(-0.280, 0.0, 0.280),
        y=0.016,
        material="dark_steel",
        prefix="hinge_bolt",
    )
    _add_bolt_heads(
        frame,
        xs=(0.336,),
        zs=(-0.080, 0.080),
        y=0.016,
        material="dark_steel",
        prefix="keeper_bolt",
    )

    # Door / service panel. The child frame is on the hinge axis, so all door
    # geometry extends along +X from the hinge line.
    panel.visual(
        Box((0.62, 0.004, 0.84)),
        origin=Origin(xyz=(0.325, -0.008, 0.0)),
        material="door_gray",
        name="door_skin",
    )
    panel.visual(
        Box((0.014, 0.042, 0.76)),
        origin=Origin(xyz=(0.072, -0.030, 0.0)),
        material="door_gray",
        name="hinge_return",
    )
    panel.visual(
        Box((0.014, 0.042, 0.76)),
        origin=Origin(xyz=(0.578, -0.030, 0.0)),
        material="door_gray",
        name="latch_return",
    )
    panel.visual(
        Box((0.506, 0.042, 0.014)),
        origin=Origin(xyz=(0.325, -0.030, 0.373)),
        material="door_gray",
        name="top_return",
    )
    panel.visual(
        Box((0.506, 0.042, 0.014)),
        origin=Origin(xyz=(0.325, -0.030, -0.373)),
        material="door_gray",
        name="bottom_return",
    )
    panel.visual(
        Box((0.110, 0.008, 0.68)),
        origin=Origin(xyz=(0.115, -0.022, 0.0)),
        material="steel",
        name="hinge_reinforcement",
    )
    panel.visual(
        Box((0.080, 0.030, 0.50)),
        origin=Origin(xyz=(0.555, -0.020, 0.0)),
        material="steel",
        name="latch_box",
    )
    panel.visual(
        Box((0.360, 0.006, 0.022)),
        origin=Origin(xyz=(0.308, -0.022, 0.158), rpy=(0.0, 0.52, 0.0)),
        material="steel",
        name="upper_load_brace",
    )
    panel.visual(
        Box((0.360, 0.006, 0.022)),
        origin=Origin(xyz=(0.308, -0.022, -0.158), rpy=(0.0, -0.52, 0.0)),
        material="steel",
        name="lower_load_brace",
    )
    for name, z_center in (
        ("hinge_leaf_top", 0.265),
        ("hinge_leaf_mid", 0.0),
        ("hinge_leaf_bottom", -0.265),
    ):
        panel.visual(
            Box((0.072, 0.006, 0.120)),
            origin=Origin(xyz=(0.058, -0.004, z_center)),
            material="steel",
            name=name,
        )

    # Latch side hardware with a clear guard around the release handle.
    panel.visual(
        Box((0.070, 0.040, 0.200)),
        origin=Origin(xyz=(0.575, 0.013, 0.0)),
        material="safety_yellow",
        name="handle_housing",
    )
    panel.visual(
        Box((0.018, 0.050, 0.160)),
        origin=Origin(xyz=(0.592, 0.029, 0.0)),
        material="dark_steel",
        name="handle_body",
    )
    panel.visual(
        Box((0.012, 0.024, 0.230)),
        origin=Origin(xyz=(0.542, 0.005, 0.0)),
        material="safety_yellow",
        name="guard_left",
    )
    panel.visual(
        Box((0.012, 0.024, 0.230)),
        origin=Origin(xyz=(0.606, 0.005, 0.0)),
        material="safety_yellow",
        name="guard_right",
    )
    panel.visual(
        Box((0.076, 0.024, 0.012)),
        origin=Origin(xyz=(0.574, 0.005, 0.118)),
        material="safety_yellow",
        name="guard_top",
    )
    panel.visual(
        Box((0.076, 0.024, 0.012)),
        origin=Origin(xyz=(0.574, 0.005, -0.118)),
        material="safety_yellow",
        name="guard_bottom",
    )
    panel.visual(
        Box((0.055, 0.008, 0.070)),
        origin=Origin(xyz=(0.620, -0.002, 0.145)),
        material="steel",
        name="lockout_ear",
    )
    panel.visual(
        Box((0.050, 0.020, 0.018)),
        origin=Origin(xyz=(0.095, -0.001, 0.400)),
        material="safety_yellow",
        name="door_stop_top",
    )
    panel.visual(
        Box((0.050, 0.020, 0.018)),
        origin=Origin(xyz=(0.095, -0.001, -0.400)),
        material="safety_yellow",
        name="door_stop_bottom",
    )

    _add_bolt_heads(
        panel,
        xs=(0.078, 0.128),
        zs=(-0.260, 0.0, 0.260),
        y=-0.004,
        material="dark_steel",
        prefix="panel_hinge_bolt",
    )
    _add_bolt_heads(
        panel,
        xs=(0.548, 0.602),
        zs=(-0.085, 0.085),
        y=-0.004,
        material="dark_steel",
        prefix="panel_latch_bolt",
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("service_panel")
    hinge = object_model.get_articulation("panel_hinge")

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
        "hinge_axis_is_vertical_and_opens_outward",
        hinge.axis == (0.0, 0.0, 1.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 1.6,
        "Expected a vertical swing hinge with a clearly openable heavy-duty travel limit.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            panel,
            frame,
            elem_a="door_skin",
            elem_b="left_jamb_face",
            contact_tol=0.001,
            name="hinge_side_of_panel_is_seated_on_frame",
        )
        ctx.expect_contact(
            panel,
            frame,
            elem_a="door_skin",
            elem_b="right_jamb_face",
            contact_tol=0.001,
            name="latch_side_of_panel_is_seated_on_frame",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="door_skin",
            min_overlap=0.60,
            name="panel_visibly_covers_the_opening",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="lockout_ear",
            elem_b="lockout_staple",
            min_overlap=0.045,
            name="lockout_ear_aligns_with_frame_staple",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="y",
            positive_elem="lockout_staple",
            negative_elem="lockout_ear",
            min_gap=0.001,
            max_gap=0.020,
            name="lockout_staple_sits_close_to_the_panel_ear",
        )

    with ctx.pose({hinge: 1.60}):
        ctx.expect_gap(
            panel,
            frame,
            axis="y",
            positive_elem="handle_body",
            min_gap=0.20,
            name="opened_panel_swings_handle_clear_of_frame",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="y",
            positive_elem="lockout_ear",
            min_gap=0.18,
            name="opened_panel_pulls_lockout_ear_clear_of_keeper_side",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
