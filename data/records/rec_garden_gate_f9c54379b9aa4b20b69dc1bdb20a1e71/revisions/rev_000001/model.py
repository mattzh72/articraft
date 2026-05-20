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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _x_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farm_pedestrian_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.43, 0.45, 0.47, 1.0))
    timber = model.material("treated_post", rgba=(0.48, 0.34, 0.20, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.62, 0.63, 0.66, 1.0))

    hinge_post = model.part("hinge_post")
    hinge_post.visual(
        Box((0.14, 0.14, 1.90)),
        origin=Origin(xyz=(-0.11, 0.0, 0.95)),
        material=timber,
        name="support_post",
    )
    hinge_post.visual(
        Box((0.16, 0.16, 0.10)),
        origin=Origin(xyz=(-0.11, 0.0, 0.05)),
        material=weathered_steel,
        name="post_shoe",
    )
    for name, z in (
        ("hinge_bracket_low", 0.30),
        ("hinge_bracket_mid", 0.67),
        ("hinge_bracket_high", 1.03),
    ):
        hinge_post.visual(
            Box((0.06, 0.016, 0.040)),
            origin=Origin(xyz=(-0.05, 0.030, z)),
            material=weathered_steel,
            name=name,
        )
    hinge_post.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 1.90)),
        mass=38.0,
        origin=Origin(xyz=(-0.11, 0.0, 0.95)),
    )

    gate_frame = model.part("gate_frame")
    gate_perimeter = wire_from_points(
        [
            (0.055, 0.0, 0.22),
            (1.40, 0.0, 0.22),
            (1.40, 0.0, 1.08),
            (0.055, 0.0, 1.08),
        ],
        radius=0.018,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.055,
        corner_segments=10,
    )
    gate_frame.visual(
        _mesh("gate_perimeter", gate_perimeter),
        material=galvanized,
        name="frame_perimeter",
    )
    for rail_name, rail_z in (
        ("rail_low", 0.42),
        ("rail_mid_low", 0.64),
        ("rail_mid_high", 0.86),
    ):
        gate_frame.visual(
            Cylinder(radius=0.013, length=1.345),
            origin=_x_cylinder_origin((0.7275, 0.0, rail_z)),
            material=galvanized,
            name=rail_name,
        )
    for sleeve_name, sleeve_z in (
        ("sleeve_low", 0.30),
        ("sleeve_mid", 0.67),
        ("sleeve_high", 1.03),
    ):
        gate_frame.visual(
            Cylinder(radius=0.019, length=0.12),
            origin=Origin(xyz=(0.0, 0.0, sleeve_z)),
            material=weathered_steel,
            name=sleeve_name,
        )
        gate_frame.visual(
            Box((0.055, 0.016, 0.012)),
            origin=Origin(xyz=(0.0275, 0.0, sleeve_z)),
            material=weathered_steel,
            name=f"{sleeve_name}_strap",
        )
    gate_frame.visual(
        Box((0.020, 0.032, 0.052)),
        origin=Origin(xyz=(1.428, 0.0, 0.79)),
        material=weathered_steel,
        name="latch_mount",
    )
    gate_frame.visual(
        Box((0.076, 0.046, 0.008)),
        origin=Origin(xyz=(1.334, 0.0, 0.156)),
        material=weathered_steel,
        name="swivel_top_plate",
    )
    gate_frame.visual(
        Box((0.020, 0.020, 0.050)),
        origin=Origin(xyz=(1.396, 0.0, 0.177)),
        material=weathered_steel,
        name="swivel_keeper_plate",
    )
    gate_frame.visual(
        Box((0.014, 0.006, 0.024)),
        origin=Origin(xyz=(1.334, 0.024, 0.144)),
        material=weathered_steel,
        name="swivel_side_clip_left",
    )
    gate_frame.visual(
        Box((0.014, 0.006, 0.024)),
        origin=Origin(xyz=(1.334, -0.024, 0.144)),
        material=weathered_steel,
        name="swivel_side_clip_right",
    )
    gate_frame.visual(
        Box((0.026, 0.020, 0.036)),
        origin=Origin(xyz=(1.373, 0.0, 0.174)),
        material=weathered_steel,
        name="swivel_support_brace",
    )
    gate_frame.visual(
        Box((0.026, 0.020, 0.060)),
        origin=Origin(xyz=(1.390, 0.0, 0.190)),
        material=weathered_steel,
        name="swivel_hanger",
    )
    gate_frame.inertial = Inertial.from_geometry(
        Box((1.46, 0.12, 1.12)),
        mass=22.0,
        origin=Origin(xyz=(0.73, 0.0, 0.60)),
    )

    caster_yoke = model.part("caster_yoke")
    caster_yoke.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(),
        material=weathered_steel,
        name="swivel_head",
    )
    caster_yoke.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.004, 0.0, -0.020)),
        material=weathered_steel,
        name="swivel_neck",
    )
    caster_yoke.visual(
        Box((0.024, 0.038, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, -0.038)),
        material=weathered_steel,
        name="fork_bridge",
    )
    caster_yoke.visual(
        Box((0.114, 0.006, 0.058)),
        origin=Origin(xyz=(-0.053, 0.0215, -0.070)),
        material=weathered_steel,
        name="left_fork",
    )
    caster_yoke.visual(
        Box((0.114, 0.006, 0.058)),
        origin=Origin(xyz=(-0.053, -0.0215, -0.070)),
        material=weathered_steel,
        name="right_fork",
    )
    caster_yoke.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=_y_cylinder_origin((-0.110, 0.018, -0.068)),
        material=wheel_metal,
        name="axle_cap_left",
    )
    caster_yoke.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=_y_cylinder_origin((-0.110, -0.018, -0.068)),
        material=wheel_metal,
        name="axle_cap_right",
    )
    caster_yoke.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.12)),
        mass=1.5,
        origin=Origin(xyz=(-0.020, 0.0, -0.040)),
    )

    support_wheel = model.part("support_wheel")
    support_wheel.visual(
        Cylinder(radius=0.093, length=0.030),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=dark_rubber,
        name="tire",
    )
    support_wheel.visual(
        Cylinder(radius=0.070, length=0.022),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=wheel_metal,
        name="rim",
    )
    support_wheel.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=weathered_steel,
        name="hub",
    )
    support_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.093, length=0.030),
        mass=1.8,
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
    )

    latch_loop = model.part("latch_loop")
    latch_ring = wire_from_points(
        [
            (0.0, -0.032, -0.004),
            (0.0, -0.050, -0.058),
            (0.0, 0.050, -0.058),
            (0.0, 0.032, -0.004),
        ],
        radius=0.0055,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.018,
        corner_segments=8,
    )
    latch_loop.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=_x_cylinder_origin((0.0, 0.0, 0.0)),
        material=weathered_steel,
        name="loop_sleeve",
    )
    latch_loop.visual(
        _mesh("latch_loop_ring", latch_ring),
        origin=Origin(xyz=(0.0135, 0.0, 0.0)),
        material=weathered_steel,
        name="loop_ring",
    )
    latch_loop.inertial = Inertial.from_geometry(
        Box((0.02, 0.12, 0.08)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
    )

    model.articulation(
        "gate_swing",
        ArticulationType.REVOLUTE,
        parent=hinge_post,
        child=gate_frame,
        origin=Origin(xyz=(-0.001, 0.030, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=gate_frame,
        child=caster_yoke,
        origin=Origin(xyz=(1.33, 0.0, 0.147)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0),
    )
    model.articulation(
        "support_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=caster_yoke,
        child=support_wheel,
        origin=Origin(xyz=(-0.110, 0.0, -0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "latch_loop_pivot",
        ArticulationType.REVOLUTE,
        parent=gate_frame,
        child=latch_loop,
        origin=Origin(xyz=(1.446, 0.0, 0.79)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-1.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hinge_post = object_model.get_part("hinge_post")
    gate_frame = object_model.get_part("gate_frame")
    caster_yoke = object_model.get_part("caster_yoke")
    support_wheel = object_model.get_part("support_wheel")
    latch_loop = object_model.get_part("latch_loop")
    gate_swing = object_model.get_articulation("gate_swing")
    caster_swivel = object_model.get_articulation("caster_swivel")
    support_wheel_spin = object_model.get_articulation("support_wheel_spin")
    latch_loop_pivot = object_model.get_articulation("latch_loop_pivot")

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
        "primary_articulation_axes",
        gate_swing.axis == (0.0, 0.0, 1.0)
        and caster_swivel.axis == (0.0, 0.0, 1.0)
        and support_wheel_spin.axis == (0.0, 1.0, 0.0)
        and latch_loop_pivot.axis == (1.0, 0.0, 0.0),
        details=(
            f"gate={gate_swing.axis}, caster={caster_swivel.axis}, "
            f"wheel={support_wheel_spin.axis}, latch={latch_loop_pivot.axis}"
        ),
    )

    with ctx.pose(
        {
            gate_swing: 0.0,
            caster_swivel: 0.0,
            support_wheel_spin: 0.0,
            latch_loop_pivot: 0.0,
        }
    ):
        ctx.expect_gap(
            gate_frame,
            caster_yoke,
            axis="z",
            positive_elem="swivel_top_plate",
            negative_elem="swivel_head",
            min_gap=0.0,
            max_gap=0.004,
            name="caster_head_under_top_plate",
        )
        ctx.expect_overlap(
            gate_frame,
            caster_yoke,
            axes="xy",
            min_overlap=0.020,
            elem_a="swivel_top_plate",
            elem_b="swivel_head",
            name="caster_head_footprint_under_bracket",
        )
        ctx.expect_gap(
            caster_yoke,
            support_wheel,
            axis="y",
            positive_elem="left_fork",
            negative_elem="tire",
            min_gap=0.002,
            max_gap=0.010,
            name="wheel_clear_of_left_fork",
        )
        ctx.expect_gap(
            support_wheel,
            caster_yoke,
            axis="y",
            positive_elem="tire",
            negative_elem="right_fork",
            min_gap=0.002,
            max_gap=0.010,
            name="wheel_clear_of_right_fork",
        )
        ctx.expect_contact(
            support_wheel,
            caster_yoke,
            elem_a="hub",
            elem_b="axle_cap_right",
            name="wheel_captured_on_right_axle_cap",
        )
        ctx.expect_contact(
            caster_yoke,
            support_wheel,
            elem_a="axle_cap_left",
            elem_b="hub",
            name="wheel_captured_on_left_axle_cap",
        )
        ctx.expect_overlap(
            gate_frame,
            support_wheel,
            axes="xy",
            min_overlap=0.020,
            name="wheel_stays_under_outer_corner",
        )
        ctx.expect_gap(
            gate_frame,
            support_wheel,
            axis="z",
            positive_elem="frame_perimeter",
            negative_elem="tire",
            min_gap=0.010,
            max_gap=0.050,
            name="wheel_hangs_below_gate_frame",
        )
        ctx.expect_gap(
            gate_frame,
            hinge_post,
            axis="x",
            positive_elem="sleeve_low",
            negative_elem="hinge_bracket_low",
            min_gap=0.0,
            max_gap=0.001,
            name="lower_hinge_sleeve_contacts_bracket",
        )
        ctx.expect_gap(
            latch_loop,
            gate_frame,
            axis="x",
            positive_elem="loop_sleeve",
            negative_elem="latch_mount",
            min_gap=0.0,
            max_gap=0.001,
            name="latch_loop_hangs_from_mount",
        )
        ctx.expect_overlap(
            latch_loop,
            gate_frame,
            axes="yz",
            min_overlap=0.012,
            elem_a="loop_sleeve",
            elem_b="latch_mount",
            name="latch_loop_sleeve_aligned_to_mount",
        )
        ctx.expect_contact(
            latch_loop,
            gate_frame,
            elem_a="loop_sleeve",
            elem_b="latch_mount",
            name="latch_loop_contacts_mount",
        )

    with ctx.pose({gate_swing: 1.10, caster_swivel: 0.45, latch_loop_pivot: 0.55}):
        ctx.expect_gap(
            gate_frame,
            caster_yoke,
            axis="z",
            positive_elem="swivel_top_plate",
            negative_elem="swivel_head",
            min_gap=0.0,
            max_gap=0.004,
            name="caster_capture_persists_when_gate_swings",
        )
        ctx.expect_overlap(
            gate_frame,
            caster_yoke,
            axes="xy",
            min_overlap=0.020,
            elem_a="swivel_top_plate",
            elem_b="swivel_head",
            name="caster_head_stays_under_bracket_when_gate_swings",
        )
        ctx.expect_overlap(
            gate_frame,
            support_wheel,
            axes="xy",
            min_overlap=0.020,
            name="wheel_remains_under_gate_when_open",
        )
        ctx.expect_contact(
            latch_loop,
            gate_frame,
            elem_a="loop_sleeve",
            elem_b="latch_mount",
            name="latch_loop_stays_mounted_in_open_pose",
        )

    _ = hinge_post
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
