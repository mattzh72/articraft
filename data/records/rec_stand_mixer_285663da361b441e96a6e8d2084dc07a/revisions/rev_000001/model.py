from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _loft_section_x(x: float, width: float, height: float, radius: float, *, z_offset: float = 0.0):
    """Rounded-rectangle cross-section in the local YZ plane."""
    return [(x, y, z + z_offset) for y, z in rounded_rect_profile(width, height, radius)]


def _loft_section_z(z: float, length: float, width: float, radius: float, *, x_offset: float = 0.0):
    """Rounded-rectangle cross-section in the local XY plane."""
    return [(x + x_offset, y, z) for x, y in rounded_rect_profile(length, width, radius)]


def _build_whisk_mesh():
    ferrule = CylinderGeometry(radius=0.009, height=0.040, radial_segments=32).translate(0.0, 0.0, -0.020)
    collar = CylinderGeometry(radius=0.015, height=0.026, radial_segments=36).translate(0.0, 0.0, -0.053)
    lower_band = CylinderGeometry(radius=0.020, height=0.014, radial_segments=36).translate(0.0, 0.0, -0.074)
    whisk = ferrule
    whisk.merge(collar)
    whisk.merge(lower_band)

    # Eight crossed balloon loops merge into the lower ferrule, giving the
    # attachment a single continuous, supported mesh rather than floating wires.
    for index in range(8):
        angle = index * math.pi / 8.0
        c = math.cos(angle)
        s = math.sin(angle)
        pts = [
            (0.010 * c, 0.010 * s, -0.065),
            (0.028 * c, 0.028 * s, -0.090),
            (0.050 * c, 0.050 * s, -0.135),
            (0.060 * c, 0.060 * s, -0.178),
            (0.000, 0.000, -0.215),
            (-0.060 * c, -0.060 * s, -0.178),
            (-0.050 * c, -0.050 * s, -0.135),
            (-0.028 * c, -0.028 * s, -0.090),
            (-0.010 * c, -0.010 * s, -0.065),
        ]
        whisk.merge(
            tube_from_spline_points(
                pts,
                radius=0.0017,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return whisk


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deluxe_home_baking_stand_mixer")

    enamel = model.material("cranberry_enamel", rgba=(0.62, 0.05, 0.08, 1.0))
    dark_enamel = model.material("dark_enamel_shadow", rgba=(0.27, 0.02, 0.035, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.82, 0.84, 0.83, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.023, 0.022, 1.0))
    ivory = model.material("ivory_button", rgba=(0.92, 0.88, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh(ExtrudeGeometry(rounded_rect_profile(0.54, 0.34, 0.065), 0.070), "base_shell"),
        origin=Origin(xyz=(-0.035, 0.0, 0.035)),
        material=enamel,
        name="base_shell",
    )
    base.visual(
        _save_mesh(ExtrudeGeometry(rounded_rect_profile(0.48, 0.29, 0.045), 0.012), "base_shadow"),
        origin=Origin(xyz=(-0.035, 0.0, 0.010)),
        material=dark_enamel,
        name="base_shadow",
    )
    base.visual(
        Box((0.24, 0.27, 0.012)),
        origin=Origin(xyz=(-0.145, 0.0, 0.076)),
        material=dark_enamel,
        name="carriage_pad",
    )
    base.visual(
        Box((0.12, 0.020, 0.022)),
        origin=Origin(xyz=(-0.145, 0.112, 0.092)),
        material=dark_metal,
        name="lift_track_0",
    )
    base.visual(
        Box((0.12, 0.020, 0.022)),
        origin=Origin(xyz=(-0.145, -0.112, 0.092)),
        material=dark_metal,
        name="lift_track_1",
    )
    base.visual(
        Box((0.035, 0.055, 0.052)),
        origin=Origin(xyz=(-0.218, 0.130, 0.104)),
        material=dark_metal,
        name="guide_post_0",
    )
    base.visual(
        Box((0.035, 0.055, 0.052)),
        origin=Origin(xyz=(-0.218, -0.130, 0.104)),
        material=dark_metal,
        name="guide_post_1",
    )
    base.visual(
        Box((0.205, 0.028, 0.170)),
        origin=Origin(xyz=(0.035, 0.156, 0.140)),
        material=enamel,
        name="control_pod",
    )
    base.visual(
        _save_mesh(
            section_loft(
                [
                    _loft_section_z(0.000, 0.155, 0.170, 0.040),
                    _loft_section_z(0.170, 0.125, 0.145, 0.035, x_offset=0.010),
                    _loft_section_z(0.335, 0.095, 0.120, 0.030, x_offset=0.025),
                ]
            ),
            "rear_column",
        ),
        origin=Origin(xyz=(0.155, 0.0, 0.070)),
        material=enamel,
        name="rear_column",
    )
    base.visual(
        Box((0.050, 0.030, 0.360)),
        origin=Origin(xyz=(0.176, 0.112, 0.250)),
        material=enamel,
        name="yoke_post_0",
    )
    base.visual(
        Box((0.050, 0.030, 0.360)),
        origin=Origin(xyz=(0.176, -0.112, 0.250)),
        material=enamel,
        name="yoke_post_1",
    )
    base.visual(
        Box((0.102, 0.030, 0.086)),
        origin=Origin(xyz=(0.176, 0.112, 0.438)),
        material=enamel,
        name="hinge_saddle_0",
    )
    base.visual(
        Box((0.102, 0.030, 0.086)),
        origin=Origin(xyz=(0.176, -0.112, 0.438)),
        material=enamel,
        name="hinge_saddle_1",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.035),
        origin=Origin(xyz=(0.170, 0.0975, 0.474), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_lug_0",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.035),
        origin=Origin(xyz=(0.170, -0.0975, 0.474), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_lug_1",
    )
    for index, (x, y) in enumerate([(-0.255, -0.125), (-0.255, 0.125), (0.155, -0.125), (0.155, 0.125)]):
        base.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(xyz=(x, y, 0.007)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )
    base.inertial = Inertial.from_geometry(Box((0.54, 0.34, 0.48)), mass=9.0, origin=Origin(xyz=(-0.02, 0.0, 0.19)))

    bowl_stage = model.part("bowl_stage")
    bowl_stage.visual(
        Box((0.215, 0.185, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="carriage_foot",
    )
    bowl_stage.visual(
        Cylinder(radius=0.060, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_metal,
        name="bowl_pedestal",
    )
    bowl_stage.visual(
        _save_mesh(
            LatheGeometry.from_shell_profiles(
                [
                    (0.040, 0.000),
                    (0.078, 0.012),
                    (0.120, 0.070),
                    (0.152, 0.156),
                    (0.166, 0.205),
                ],
                [
                    (0.012, 0.012),
                    (0.066, 0.024),
                    (0.111, 0.078),
                    (0.144, 0.154),
                    (0.156, 0.195),
                ],
                segments=72,
                start_cap="round",
                end_cap="flat",
                lip_samples=10,
            ),
            "stainless_bowl",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=stainless,
        name="bowl_shell",
    )
    bowl_stage.visual(
        _save_mesh(TorusGeometry(radius=0.161, tube=0.006, radial_segments=16, tubular_segments=72), "bowl_rolled_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.257)),
        material=stainless,
        name="rolled_lip",
    )
    bowl_stage.visual(
        _save_mesh(
            tube_from_spline_points(
                [
                    (0.000, 0.150, 0.170),
                    (0.028, 0.185, 0.170),
                    (0.060, 0.195, 0.145),
                    (0.028, 0.185, 0.118),
                    (0.000, 0.150, 0.118),
                ],
                radius=0.004,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            ),
            "bowl_handle",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=stainless,
        name="bowl_handle",
    )
    bowl_stage.inertial = Inertial.from_geometry(Cylinder(radius=0.17, length=0.27), mass=1.25, origin=Origin(xyz=(0.0, 0.0, 0.15)))

    head = model.part("head")
    head.visual(
        _save_mesh(
            section_loft(
                [
                    _loft_section_x(0.000, 0.116, 0.122, 0.033, z_offset=0.000),
                    _loft_section_x(-0.105, 0.170, 0.150, 0.048, z_offset=0.010),
                    _loft_section_x(-0.245, 0.186, 0.140, 0.050, z_offset=0.000),
                    _loft_section_x(-0.345, 0.118, 0.100, 0.034, z_offset=-0.018),
                ]
            ),
            "tilt_head_shell",
        ),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.032),
        origin=Origin(xyz=(-0.310, 0.0, -0.075)),
        material=dark_metal,
        name="drive_socket",
    )
    head.visual(
        _save_mesh(ExtrudeGeometry(rounded_rect_profile(0.105, 0.030, 0.010), 0.004), "trim_badge"),
        origin=Origin(xyz=(-0.228, -0.092, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="side_badge",
    )
    head.inertial = Inertial.from_geometry(Box((0.36, 0.19, 0.16)), mass=4.2, origin=Origin(xyz=(-0.18, 0.0, 0.0)))

    whisk = model.part("whisk")
    whisk.visual(
        _save_mesh(_build_whisk_mesh(), "wire_whisk"),
        material=stainless,
        name="wire_whisk",
    )
    whisk.inertial = Inertial.from_geometry(Cylinder(radius=0.065, length=0.22), mass=0.22, origin=Origin(xyz=(0.0, 0.0, -0.10)))

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        _save_mesh(
            KnobGeometry(
                0.052,
                0.034,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.060, 0.007, flare=0.06, chamfer=0.0015),
                grip=KnobGrip(style="fluted", count=18, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "speed_knob",
        ),
        material=ivory,
        name="knob_body",
    )
    speed_knob.inertial = Inertial.from_geometry(Cylinder(radius=0.030, length=0.034), mass=0.08)

    head_lock_button = model.part("head_lock_button")
    head_lock_button.visual(
        Box((0.046, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=ivory,
        name="lock_button_cap",
    )
    head_lock_button.visual(
        Box((0.026, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=dark_metal,
        name="lock_button_stem",
    )
    head_lock_button.inertial = Inertial.from_geometry(Box((0.046, 0.034, 0.024)), mass=0.04, origin=Origin(xyz=(0.0, 0.004, 0.0)))

    model.articulation(
        "base_to_bowl_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_stage,
        origin=Origin(xyz=(-0.145, 0.0, 0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.06, lower=0.0, upper=0.045),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.170, 0.0, 0.474)),
        # The closed head shell extends forward along local -X.  +Y raises
        # that forward nose for a conventional rear tilt-head mixer.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=math.radians(58.0)),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(-0.310, 0.0, -0.091)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=45.0),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(-0.035, 0.194, 0.136), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.9, velocity=4.0, lower=0.0, upper=math.radians(300.0)),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock_button,
        origin=Origin(xyz=(0.115, 0.170, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=0.0, upper=0.012),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl_stage = object_model.get_part("bowl_stage")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_knob = object_model.get_part("speed_knob")
    head_lock_button = object_model.get_part("head_lock_button")

    bowl_lift = object_model.get_articulation("base_to_bowl_stage")
    head_tilt = object_model.get_articulation("base_to_head")
    whisk_drive = object_model.get_articulation("head_to_whisk")
    knob_turn = object_model.get_articulation("base_to_speed_knob")
    lock_push = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "required articulation types",
        bowl_lift.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and whisk_drive.articulation_type == ArticulationType.CONTINUOUS
        and knob_turn.articulation_type == ArticulationType.REVOLUTE
        and lock_push.articulation_type == ArticulationType.PRISMATIC,
        details="Mixer should have lift, tilt, rotary whisk drive, speed knob, and lock button articulations.",
    )

    ctx.expect_gap(
        bowl_stage,
        base,
        axis="z",
        positive_elem="carriage_foot",
        negative_elem="carriage_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="bowl carriage sits on base pad",
    )
    ctx.expect_overlap(
        bowl_stage,
        base,
        axes="xy",
        elem_a="carriage_foot",
        elem_b="carriage_pad",
        min_overlap=0.10,
        name="bowl carriage is centered on the front pad",
    )
    ctx.expect_gap(
        speed_knob,
        base,
        axis="y",
        positive_elem="knob_body",
        negative_elem="control_pod",
        max_gap=0.004,
        max_penetration=0.0,
        name="speed knob is seated on side of base",
    )
    ctx.expect_gap(
        head_lock_button,
        base,
        axis="y",
        positive_elem="lock_button_cap",
        negative_elem="control_pod",
        max_gap=0.004,
        max_penetration=0.0,
        name="head lock button protrudes from base face",
    )
    ctx.expect_gap(
        head,
        whisk,
        axis="z",
        positive_elem="drive_socket",
        negative_elem="wire_whisk",
        max_gap=0.002,
        max_penetration=0.0001,
        name="whisk ferrule is captured by drive socket",
    )

    rest_bowl = ctx.part_world_position(bowl_stage)
    with ctx.pose({bowl_lift: 0.045}):
        raised_bowl = ctx.part_world_position(bowl_stage)
    ctx.check(
        "bowl lift raises vertically",
        rest_bowl is not None and raised_bowl is not None and raised_bowl[2] > rest_bowl[2] + 0.040,
        details=f"rest={rest_bowl}, raised={raised_bowl}",
    )

    rest_head_box = ctx.part_world_aabb(head)
    with ctx.pose({head_tilt: math.radians(58.0)}):
        tilted_head_box = ctx.part_world_aabb(head)
        tilted_whisk_box = ctx.part_world_aabb(whisk)
    ctx.check(
        "tilt hinge raises head nose",
        rest_head_box is not None
        and tilted_head_box is not None
        and tilted_head_box[1][2] > rest_head_box[1][2] + 0.07,
        details=f"rest_head_aabb={rest_head_box}, tilted_head_aabb={tilted_head_box}",
    )
    ctx.check(
        "tilted whisk clears bowl opening",
        tilted_whisk_box is not None
        and tilted_head_box is not None
        and tilted_whisk_box[0][2] > 0.19,
        details=f"tilted_whisk_aabb={tilted_whisk_box}",
    )

    rest_button = ctx.part_world_position(head_lock_button)
    with ctx.pose({lock_push: 0.012}):
        pressed_button = ctx.part_world_position(head_lock_button)
    ctx.check(
        "head lock button travels inward",
        rest_button is not None and pressed_button is not None and pressed_button[1] < rest_button[1] - 0.010,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
